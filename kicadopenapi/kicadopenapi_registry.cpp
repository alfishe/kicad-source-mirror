#include "kicadopenapi_registry.h"

#include <algorithm>
#include <cctype>


KOPENAPI_REGISTRY& KOPENAPI_REGISTRY::Get()
{
    static KOPENAPI_REGISTRY instance;
    return instance;
}


bool KOPENAPI_REGISTRY::Add( KOPENAPI_METHOD aMethod )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    std::string name = aMethod.name;
    reg.m_methods[name] = std::move( aMethod );
    return true;
}


std::vector<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Snapshot() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    std::vector<KOPENAPI_METHOD> out;

    for( const auto& [name, method] : m_methods )
        out.push_back( method );

    return out;
}


std::optional<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Find( const std::string& aName ) const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    auto it = m_methods.find( aName );

    if( it == m_methods.end() )
        return std::nullopt;

    return it->second;
}


static std::vector<std::string> tokenize( const std::string& aText )
{
    std::vector<std::string> tokens;
    std::string              current;

    for( char ch : aText )
    {
        if( std::isalnum( (unsigned char) ch ) )
        {
            current += (char) std::tolower( (unsigned char) ch );
        }
        else if( !current.empty() )
        {
            tokens.push_back( current );
            current.clear();
        }
    }

    if( !current.empty() )
        tokens.push_back( current );

    return tokens;
}


static std::string lower( std::string aText )
{
    for( char& ch : aText )
        ch = (char) std::tolower( (unsigned char) ch );

    return aText;
}


std::vector<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Search( const std::string& aQuery, size_t aLimit,
                                                        bool aIncludeGuiOnly ) const
{
    const std::vector<std::string> terms = tokenize( aQuery );
    std::vector<std::pair<int, KOPENAPI_METHOD>> scored;

    for( KOPENAPI_METHOD& method : Snapshot() )
    {
        if( method.guiOnly && !aIncludeGuiOnly )
            continue;

        int score = terms.empty() ? 1 : 0;

        const std::string name = lower( method.name );
        const std::string summary = lower( method.summary );
        std::string       properties;

        if( method.inputSchema.contains( "properties" ) && method.inputSchema["properties"].is_object() )
        {
            for( const auto& [key, value] : method.inputSchema["properties"].items() )
                properties += lower( key ) + " ";
        }

        for( const std::string& term : terms )
        {
            if( name.find( term ) != std::string::npos )
                score += 3;

            if( summary.find( term ) != std::string::npos )
                score += 2;

            if( properties.find( term ) != std::string::npos )
                score += 1;
        }

        if( score > 0 )
            scored.emplace_back( score, std::move( method ) );
    }

    std::stable_sort( scored.begin(), scored.end(),
                      []( const auto& a, const auto& b ) { return a.first > b.first; } );

    std::vector<KOPENAPI_METHOD> out;

    for( auto& [score, method] : scored )
    {
        if( out.size() >= aLimit )
            break;

        out.push_back( std::move( method ) );
    }

    return out;
}
