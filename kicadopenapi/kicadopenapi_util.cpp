#include "kicadopenapi_util.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <set>


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
        // '*' first: a literal '*' in the text (e.g. "*board — PCB Editor", a modified
        // document) must not consume the pattern's wildcard
        if( p < aPattern.size() && aPattern[p] == '*' )
        {
            star = p++;
            mark = t;
        }
        else if( p < aPattern.size()
                 && ( aPattern[p] == '?' || lowerChar( aPattern[p] ) == lowerChar( aText[t] ) ) )
        {
            ++p;
            ++t;
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
    // a named object: a range-for over items() of a temporary iterates a destroyed object
    const nlohmann::json page = KopenapiPageSchema();

    for( const auto& [key, value] : page.items() )
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


KOPENAPI_TEXT_MATCH KopenapiTextMatch( const std::string& aQuery, const std::string& aName, const std::string& aKeywords,
                                       const std::string& aDescription )
{
    auto lower = []( std::string aText )
    {
        for( char& ch : aText )
            ch = static_cast<char>( std::tolower( static_cast<unsigned char>( ch ) ) );

        return aText;
    };

    const std::string name = lower( aName ), keywords = lower( aKeywords ), description = lower( aDescription );
    const std::string query = lower( aQuery );

    KOPENAPI_TEXT_MATCH result;
    size_t              start = 0;

    while( start < query.size() )
    {
        const size_t      end = query.find_first_of( " \t", start );
        const std::string term = query.substr( start, end == std::string::npos ? std::string::npos : end - start );
        start = end == std::string::npos ? query.size() : end + 1;

        if( term.empty() )
            continue;

        result.terms++;
        int termScore = 0;

        // A name hit says most, but a term buried inside a longer name ("usb" in FSUSB42MUX)
        // says less than the part's own keywords and description
        if( name == term )
            termScore = 100;
        else if( name.rfind( term, 0 ) == 0 )
            termScore = 30;
        else if( name.find( term ) != std::string::npos )
            termScore = 8;

        if( keywords.find( term ) != std::string::npos )
            termScore += 12;

        if( description.find( term ) != std::string::npos )
            termScore += 6;

        if( termScore == 0 )
        {
            result.unmatched.push_back( term );
            continue;
        }

        result.matched++;
        result.score += termScore;
    }

    return result;
}


int KopenapiTextScore( const std::string& aQuery, const std::string& aName, const std::string& aKeywords,
                       const std::string& aDescription )
{
    const KOPENAPI_TEXT_MATCH m = KopenapiTextMatch( aQuery, aName, aKeywords, aDescription );
    return m.matched == m.terms ? m.score + 1 : 0;
}


nlohmann::json KopenapiRankedPage( std::vector<std::pair<KOPENAPI_TEXT_MATCH, nlohmann::json>> aRows,
                                   const nlohmann::json& aArgs )
{
    const bool anyFull = std::any_of( aRows.begin(), aRows.end(),
                                      []( const auto& r ) { return r.first.matched == r.first.terms; } );

    std::vector<std::pair<KOPENAPI_TEXT_MATCH, nlohmann::json>> kept;
    std::set<std::string>                                      missing;

    for( auto& [match, row] : aRows )
    {
        if( match.terms > 0 && match.matched == 0 )
            continue;

        if( anyFull && match.matched != match.terms )
            continue;

        row["score"] = match.score;

        if( !anyFull )
        {
            row["unmatched_terms"] = match.unmatched;
            missing.insert( match.unmatched.begin(), match.unmatched.end() );
        }

        kept.emplace_back( std::move( match ), std::move( row ) );
    }

    std::stable_sort( kept.begin(), kept.end(),
                      []( const auto& a, const auto& b )
                      {
                          if( a.first.matched != b.first.matched )
                              return a.first.matched > b.first.matched;

                          if( a.first.score != b.first.score )
                              return a.first.score > b.first.score;

                          return KopenapiNaturalLess( a.second.value( "lib_id", std::string() ),
                                                      b.second.value( "lib_id", std::string() ) );
                      } );

    std::vector<nlohmann::json> rows;

    for( auto& [match, row] : kept )
        rows.push_back( std::move( row ) );

    nlohmann::json page = KopenapiPage( rows, aArgs );

    if( !anyFull && !rows.empty() )
    {
        page["partial"] = true;
        page["note"] = "no item matches every word; best partial matches first (fewer or different words may help)";
    }

    return page;
}
