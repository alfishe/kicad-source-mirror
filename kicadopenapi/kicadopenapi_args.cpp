/// @file kicadopenapi_args.cpp
/// @brief See kicadopenapi_args.h.
#include "kicadopenapi_args.h"

#include <nlohmann/json-schema.hpp>

#include <wx/log.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>


namespace
{

/// @brief Levenshtein distance, for "did you mean"
size_t distance( const std::string& a, const std::string& b )
{
    std::vector<size_t> prev( b.size() + 1 ), cur( b.size() + 1 );

    for( size_t j = 0; j <= b.size(); ++j )
        prev[j] = j;

    for( size_t i = 1; i <= a.size(); ++i )
    {
        cur[0] = i;

        for( size_t j = 1; j <= b.size(); ++j )
            cur[j] = std::min( { prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + ( a[i - 1] == b[j - 1] ? 0 : 1 ) } );

        std::swap( prev, cur );
    }

    return prev[b.size()];
}


/// @brief Keeps the first error the validator reports, with where it is
class FIRST_ERROR : public nlohmann::json_schema::basic_error_handler
{
public:
    void error( const nlohmann::json::json_pointer& aPointer, const nlohmann::json& aInstance,
                const std::string& aMessage ) override
    {
        nlohmann::json_schema::basic_error_handler::error( aPointer, aInstance, aMessage );

        if( m_message.empty() )
        {
            const std::string where = aPointer.to_string();
            m_message = ( where.empty() ? std::string( "arguments" ) : "'" + where.substr( 1 ) + "'" ) + ": "
                        + aMessage;
            m_pointer = aPointer;
        }
    }

    const std::string&                Message() const { return m_message; }
    const nlohmann::json::json_pointer& Pointer() const { return m_pointer; }

private:
    std::string                  m_message;
    nlohmann::json::json_pointer m_pointer;
};


struct COMPILED
{
    std::unique_ptr<nlohmann::json_schema::json_validator> validator;   ///< null: did not compile
    std::string                                            source;      ///< schema it was built from
};


std::mutex                      s_mutex;
std::map<std::string, COMPILED> s_cache;


/// @brief The validator of aMethod's schema, compiled on first use (and again if the schema changed)
const nlohmann::json_schema::json_validator* validatorFor( const std::string& aMethod, const nlohmann::json& aSchema )
{
    const std::string           source = aSchema.dump();
    std::lock_guard<std::mutex> lock( s_mutex );
    COMPILED&                   entry = s_cache[aMethod];

    if( entry.source == source )
        return entry.validator.get();   // null when it did not compile: not tried again

    entry.source = source;
    entry.validator.reset();

    try
    {
        // formats ("uri", "date-time") are documentation here, not checked
        auto validator = std::make_unique<nlohmann::json_schema::json_validator>(
                nullptr, []( const std::string&, const std::string& ) {} );
        validator->set_root_schema( aSchema );
        entry.validator = std::move( validator );
    }
    catch( const std::exception& e )
    {
        wxLogWarning( "api %s: its inputSchema does not compile (%s); only argument names are checked",
                      aMethod, e.what() );
    }

    return entry.validator.get();
}

/// @brief aValue with integral floats as integers: JSON has one number type, and 2.0 is an
/// integer to JSON Schema (draft 7) but not to the validator, which goes by the parsed type
nlohmann::json integralAsIntegers( const nlohmann::json& aValue )
{
    if( aValue.is_number_float() )
    {
        const double v = aValue.get<double>();

        if( std::isfinite( v ) && v == std::floor( v ) && std::abs( v ) < 9.0e15 )
            return static_cast<int64_t>( v );

        return aValue;
    }

    if( aValue.is_object() || aValue.is_array() )
    {
        nlohmann::json out = aValue;

        for( auto it = out.begin(); it != out.end(); ++it )
            *it = integralAsIntegers( *it );

        return out;
    }

    return aValue;
}

/// @brief "unknown argument 'key' (did you mean ...); <where> takes: a, b"; empty when aKey is known
std::string unknownName( const std::string& aKey, const std::vector<std::string>& aKnown, const std::string& aWhere )
{
    if( std::find( aKnown.begin(), aKnown.end(), aKey ) != aKnown.end() )
        return {};

    std::string nearest;
    size_t      best = std::max<size_t>( 3, aKey.size() / 3 ) + 1;   // only close names are suggested

    for( const std::string& k : aKnown )
    {
        if( const size_t d = distance( aKey, k ); d < best )
        {
            best = d;
            nearest = k;
        }
    }

    std::string list;

    for( const std::string& k : aKnown )
        list += ( list.empty() ? "" : ", " ) + k;

    return "unknown argument '" + aKey + "'" + ( nearest.empty() ? "" : " (did you mean '" + nearest + "'?)" ) + "; "
           + aWhere + " takes: " + list;
}


/// @brief The object schema that applies to an object value: aSchema itself when it lists
/// properties, else its only object alternative of oneOf / anyOf; null when none or ambiguous
const nlohmann::json* objectSchema( const nlohmann::json& aSchema )
{
    if( aSchema.contains( "properties" ) )
        return &aSchema;

    const nlohmann::json* found = nullptr;

    for( const char* key : { "oneOf", "anyOf" } )
    {
        if( !aSchema.contains( key ) || !aSchema[key].is_array() )
            continue;

        for( const nlohmann::json& alt : aSchema[key] )
        {
            if( !alt.is_object() || !alt.contains( "properties" ) )
                continue;

            if( found )
                return nullptr;

            found = &alt;
        }
    }

    return found;
}


/// @brief Unknown names inside nested objects (array items and object properties), as at the top
/// level: the first one found, or empty.  aPath is the JSON pointer of aValue ("moves/0")
std::string unknownNested( const nlohmann::json& aSchema, const nlohmann::json& aValue, const std::string& aPath )
{
    if( !aSchema.is_object() )
        return {};

    if( aValue.is_array() )
    {
        if( !aSchema.contains( "items" ) || !aSchema["items"].is_object() )
            return {};

        for( size_t i = 0; i < aValue.size(); ++i )
        {
            if( std::string e = unknownNested( aSchema["items"], aValue[i], aPath + "/" + std::to_string( i ) ); !e.empty() )
                return e;
        }

        return {};
    }

    if( !aValue.is_object() )
        return {};

    const nlohmann::json* schema = objectSchema( aSchema );

    if( !schema || !( *schema )["properties"].is_object() )
        return {};

    const nlohmann::json& props = ( *schema )["properties"];
    const bool closed = !schema->contains( "additionalProperties" ) || ( *schema )["additionalProperties"] == false;

    if( closed && !aPath.empty() )
    {
        std::vector<std::string> known;

        for( const auto& [key, prop] : props.items() )
            known.push_back( key );

        for( const auto& [key, value] : aValue.items() )
        {
            if( std::string e = unknownName( key, known, "'" + aPath + "'" ); !e.empty() )
                return e;
        }
    }

    for( const auto& [key, value] : aValue.items() )
    {
        if( !props.contains( key ) )
            continue;

        if( std::string e = unknownNested( props[key], value, aPath.empty() ? key : aPath + "/" + key ); !e.empty() )
            return e;
    }

    return {};
}

/// @brief The reference tokens of a JSON pointer ("/points/1" -> points, 1)
std::vector<std::string> tokens( const nlohmann::json::json_pointer& aPointer )
{
    std::vector<std::string>        out;
    nlohmann::json::json_pointer    p = aPointer;

    while( !p.empty() )
    {
        out.insert( out.begin(), p.back() );
        p.pop_back();
    }

    return out;
}

} // namespace


std::vector<std::string> KopenapiKnownArgs( const nlohmann::json& aSchema )
{
    std::vector<std::string> names;

    if( !aSchema.is_object() || !aSchema.contains( "properties" ) || !aSchema["properties"].is_object() )
        return names;

    // a schema that says it takes other names too is taken at its word
    if( aSchema.contains( "additionalProperties" ) && aSchema["additionalProperties"] != false )
        return names;

    for( const auto& [key, prop] : aSchema["properties"].items() )
        names.push_back( key );

    return names;
}


std::string KopenapiValidateArgs( const std::string& aMethod, const nlohmann::json& aSchema,
                                  const nlohmann::json& aArgs )
{
    if( !aArgs.is_object() )
        return "arguments must be a JSON object";

    const std::vector<std::string> known = KopenapiKnownArgs( aSchema );

    if( !known.empty() )
    {
        for( const auto& [key, value] : aArgs.items() )
        {
            if( std::string e = unknownName( key, known, aMethod ); !e.empty() )
                return e;
        }
    }

    if( std::string e = unknownNested( aSchema, aArgs, "" ); !e.empty() )
        return e;

    const nlohmann::json_schema::json_validator* validator = validatorFor( aMethod, aSchema );

    if( !validator )
        return {};

    FIRST_ERROR errors;

    try
    {
        validator->validate( integralAsIntegers( aArgs ), errors );
    }
    catch( const std::exception& e )
    {
        return std::string( "arguments: " ) + e.what();
    }

    if( !errors )
        return {};

    // "no subschema has succeeded" says little: an object checked against the one object form of
    // its oneOf / anyOf gives the reason that form has
    const nlohmann::json value = integralAsIntegers( aArgs );

    if( errors.Message().find( "subschema" ) != std::string::npos && value.contains( errors.Pointer() ) )
    {
        const nlohmann::json& at = value.at( errors.Pointer() );
        const nlohmann::json* schema = &aSchema;

        for( const std::string& token : tokens( errors.Pointer() ) )
        {
            const nlohmann::json* object = schema->is_object() ? objectSchema( *schema ) : nullptr;

            if( object && ( *object )["properties"].contains( token ) )
                schema = &( *object )["properties"][token];
            else if( schema->is_object() && schema->contains( "items" ) && ( *schema )["items"].is_object() )
                schema = &( *schema )["items"];
            else
                return errors.Message();
        }

        const nlohmann::json* form = at.is_object() && schema->is_object() && !schema->contains( "properties" )
                                             ? objectSchema( *schema )
                                             : nullptr;

        if( form )
        {
            try
            {
                nlohmann::json_schema::json_validator one( nullptr, []( const std::string&, const std::string& ) {} );
                one.set_root_schema( *form );
                FIRST_ERROR inner;
                one.validate( at, inner );

                if( inner )
                {
                    const std::string where = errors.Pointer().to_string() + inner.Pointer().to_string();
                    return "'" + where.substr( 1 ) + "': " + inner.Message().substr( inner.Message().find( ": " ) + 2 );
                }
            }
            catch( const std::exception& )
            {
            }
        }
    }

    return errors.Message();
}
