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
        }
    }

    const std::string& Message() const { return m_message; }

private:
    std::string m_message;
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
            if( std::find( known.begin(), known.end(), key ) != known.end() )
                continue;

            std::string nearest;
            size_t      best = std::max<size_t>( 3, key.size() / 3 ) + 1;   // only close names are suggested

            for( const std::string& k : known )
            {
                if( const size_t d = distance( key, k ); d < best )
                {
                    best = d;
                    nearest = k;
                }
            }

            std::string list;

            for( const std::string& k : known )
                list += ( list.empty() ? "" : ", " ) + k;

            return "unknown argument '" + key + "'" + ( nearest.empty() ? "" : " (did you mean '" + nearest + "'?)" )
                   + "; " + aMethod + " takes: " + list;
        }
    }

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

    return errors ? errors.Message() : std::string();
}
