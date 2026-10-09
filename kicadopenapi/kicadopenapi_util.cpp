#include "kicadopenapi_util.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>


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


nlohmann::json KopenapiNetRoleFromName( const std::string& aNetName )
{
    // KiCad auto names carry pin names ("unconnected-(U5-VDD-Pad4)", "Net-(R1-Pad2)"):
    // their text says nothing about the net's role
    if( aNetName.rfind( "unconnected-", 0 ) == 0 )
        return { { "role", "unconnected" }, { "basis", { "auto name: a single unconnected pin" } } };

    if( aNetName.rfind( "Net-(", 0 ) == 0 )
        return { { "role", "signal" }, { "basis", { "auto name" } } };

    // Strip the hierarchical prefix ("/CPU/"), keeping a trailing '/' (active-low, "RESET/")
    const size_t slash = aNetName.size() > 1 ? aNetName.find_last_of( '/', aNetName.size() - 2 ) : std::string::npos;
    std::string  bare = slash == std::string::npos ? aNetName : aNetName.substr( slash + 1 );

    for( char& c : bare )
        c = static_cast<char>( std::toupper( static_cast<unsigned char>( c ) ) );

    auto has = [&]( const char* aToken ) { return bare.find( aToken ) != std::string::npos; };
    const bool voltageName = bare.size() > 1 && bare[0] == 'V' && std::isdigit( static_cast<unsigned char>( bare[1] ) );

    std::string role = "signal";

    if( bare == "GND" || has( "GND" ) || bare == "VSS" || bare == "0V" )
        role = "ground";
    else if( !bare.empty() && ( bare[0] == '+' || has( "VCC" ) || has( "VDD" ) || has( "VBAT" ) || has( "VIN" ) || voltageName ) )
        role = "power";
    else if( has( "CLK" ) || has( "CLOCK" ) || has( "XTAL" ) || has( "OSC" ) )
        role = "clock";
    else if( has( "RESET" ) || has( "RST" ) )
        role = "reset";

    nlohmann::json basis = nlohmann::json::array();

    if( role != "signal" )
        basis.push_back( "net name " + bare );

    return { { "role", role }, { "basis", basis } };
}


nlohmann::json KopenapiPagedSchema( nlohmann::json aProperties )
{
    for( const auto& [key, value] : KopenapiPageSchema().items() )
        aProperties[key] = value;

    return { { "type", "object" }, { "properties", aProperties } };
}


bool KopenapiIsConnectorRef( const std::string& aRef )
{
    for( const char* prefix : { "CON", "CN", "XS", "XP", "J", "P", "X" } )
    {
        const size_t n = std::strlen( prefix );

        if( aRef.size() > n && aRef.compare( 0, n, prefix ) == 0
            && std::isdigit( static_cast<unsigned char>( aRef[n] ) ) )
        {
            return true;
        }
    }

    return false;
}
