#include "kicadopenapi_util.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>


static char lowerChar( char aChar )
{
    return static_cast<char>( std::tolower( static_cast<unsigned char>( aChar ) ) );
}


bool KopenapiGlob( const std::string& aPattern, const std::string& aText )
{
    if( aPattern.empty() )
        return true;

    // Iterative wildcard match with backtracking to the last '*'
    size_t p = 0, t = 0, star = std::string::npos, mark = 0;

    while( t < aText.size() )
    {
        if( p < aPattern.size()
            && ( aPattern[p] == '?' || lowerChar( aPattern[p] ) == lowerChar( aText[t] ) ) )
        {
            ++p;
            ++t;
        }
        else if( p < aPattern.size() && aPattern[p] == '*' )
        {
            star = p++;
            mark = t;
        }
        else if( star != std::string::npos )
        {
            p = star + 1;
            t = ++mark;
        }
        else
        {
            return false;
        }
    }

    while( p < aPattern.size() && aPattern[p] == '*' )
        ++p;

    return p == aPattern.size();
}


bool KopenapiNaturalLess( const std::string& aA, const std::string& aB )
{
    size_t i = 0, j = 0;

    while( i < aA.size() && j < aB.size() )
    {
        if( std::isdigit( static_cast<unsigned char>( aA[i] ) )
            && std::isdigit( static_cast<unsigned char>( aB[j] ) ) )
        {
            size_t ie = i, je = j;

            while( ie < aA.size() && std::isdigit( static_cast<unsigned char>( aA[ie] ) ) )
                ++ie;

            while( je < aB.size() && std::isdigit( static_cast<unsigned char>( aB[je] ) ) )
                ++je;

            const std::string na = aA.substr( i, ie - i ), nb = aB.substr( j, je - j );
            const std::string ta = na.substr( std::min( na.find_first_not_of( '0' ), na.size() ) );
            const std::string tb = nb.substr( std::min( nb.find_first_not_of( '0' ), nb.size() ) );

            if( ta.size() != tb.size() )
                return ta.size() < tb.size();

            if( ta != tb )
                return ta < tb;

            i = ie;
            j = je;
            continue;
        }

        const char ca = lowerChar( aA[i] ), cb = lowerChar( aB[j] );

        if( ca != cb )
            return ca < cb;

        ++i;
        ++j;
    }

    return aA.size() - i < aB.size() - j;
}


nlohmann::json KopenapiPage( const nlohmann::json& aItems, const nlohmann::json& aArgs, size_t aDefaultLimit )
{
    size_t limit = aDefaultLimit;

    if( aArgs.contains( "limit" ) && aArgs["limit"].is_number_integer() )
        limit = static_cast<size_t>( std::clamp( aArgs["limit"].get<long long>(), 1LL, 1000LL ) );

    size_t offset = 0;

    if( aArgs.contains( "cursor" ) && aArgs["cursor"].is_string() )
        offset = static_cast<size_t>( std::strtoull( aArgs["cursor"].get<std::string>().c_str(), nullptr, 10 ) );

    nlohmann::json page = nlohmann::json::array();

    for( size_t i = offset; i < aItems.size() && page.size() < limit; ++i )
        page.push_back( aItems[i] );

    const size_t next = offset + page.size();

    return { { "items", page },
             { "total", aItems.size() },
             { "next_cursor", next < aItems.size() ? nlohmann::json( std::to_string( next ) ) : nlohmann::json() } };
}


nlohmann::json KopenapiPageSchema()
{
    return { { "limit", { { "type", "integer" }, { "minimum", 1 }, { "maximum", 1000 }, { "default", 100 } } },
             { "cursor", { { "type", "string" }, { "description", "next_cursor of the previous page" } } } };
}
