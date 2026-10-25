#include "kicadopenapi_util.h"

#include <algorithm>
#include <filesystem>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>


static char lowerChar( char aChar )
{
    return static_cast<char>( std::tolower( static_cast<unsigned char>( aChar ) ) );
}


/// @brief The first brace group with a comma at its top level: positions of '{' and '}', or npos
static std::pair<size_t, size_t> braceGroup( const std::string& aPattern )
{
    for( size_t open = aPattern.find( '{' ); open != std::string::npos; open = aPattern.find( '{', open + 1 ) )
    {
        int  depth = 0;
        bool comma = false;

        for( size_t i = open; i < aPattern.size(); ++i )
        {
            if( aPattern[i] == '{' )
                ++depth;
            else if( aPattern[i] == '}' && --depth == 0 )
            {
                if( comma )
                    return { open, i };

                break;
            }
            else if( aPattern[i] == ',' && depth == 1 )
                comma = true;
        }
    }

    return { std::string::npos, std::string::npos };
}


static bool globPlain( const std::string& aPattern, const std::string& aText );


bool KopenapiGlob( const std::string& aPattern, const std::string& aText )
{
    if( aPattern.empty() )
        return true;

    const auto [open, close] = braceGroup( aPattern );

    if( open == std::string::npos )
        return globPlain( aPattern, aText );

    const std::string head = aPattern.substr( 0, open ), tail = aPattern.substr( close + 1 );
    int               depth = 0;
    size_t            start = open + 1;

    for( size_t i = open + 1; i <= close; ++i )
    {
        if( aPattern[i] == '{' )
            ++depth;
        else if( aPattern[i] == '}' && depth > 0 )
            --depth;
        else if( ( aPattern[i] == ',' && depth == 0 ) || i == close )
        {
            const std::string alt = head + aPattern.substr( start, i - start ) + tail;

            if( alt.empty() ? aText.empty() : KopenapiGlob( alt, aText ) )
                return true;

            start = i + 1;
        }
    }

    return false;
}


std::vector<std::string> KopenapiGlobAlternatives( const std::string& aPattern )
{
    const auto [open, close] = braceGroup( aPattern );

    if( open == std::string::npos )
        return { aPattern };

    std::vector<std::string> out;
    const std::string        head = aPattern.substr( 0, open ), tail = aPattern.substr( close + 1 );
    int                      depth = 0;
    size_t                   start = open + 1;

    for( size_t i = open + 1; i <= close; ++i )
    {
        if( aPattern[i] == '{' )
            ++depth;
        else if( aPattern[i] == '}' && depth > 0 )
            --depth;
        else if( ( aPattern[i] == ',' && depth == 0 ) || i == close )
        {
            for( std::string& alt : KopenapiGlobAlternatives( head + aPattern.substr( start, i - start ) + tail ) )
                out.push_back( std::move( alt ) );

            start = i + 1;
        }
    }

    return out;
}


KOPENAPI_REF_FILTER::KOPENAPI_REF_FILTER( const nlohmann::json& aArgs, const char* aGlobKey, const char* aListKey )
{
    auto add = [&]( const nlohmann::json& aValue, const char* aKey )
    {
        if( !aValue.is_string() )
        {
            m_error = std::string( aKey ) + ": glob strings (e.g. \"U*\", \"{J8,U37,R86}\")";
            return;
        }

        for( std::string& alt : KopenapiGlobAlternatives( aValue.get<std::string>() ) )
        {
            if( !alt.empty() )
                m_patterns.push_back( std::move( alt ) );
        }
    };

    if( aGlobKey && aArgs.contains( aGlobKey ) && !( aArgs[aGlobKey].is_string() && aArgs[aGlobKey].get<std::string>().empty() ) )
        add( aArgs[aGlobKey], aGlobKey );

    if( aListKey && aArgs.contains( aListKey ) )
    {
        if( !aArgs[aListKey].is_array() )
            m_error = std::string( aListKey ) + ": an array of references or globs";
        else
        {
            for( const nlohmann::json& v : aArgs[aListKey] )
                add( v, aListKey );
        }
    }

    m_hit.assign( m_patterns.size(), 0 );
}


bool KOPENAPI_REF_FILTER::Match( const std::string& aRef )
{
    if( m_patterns.empty() )
        return true;

    bool any = false;

    for( size_t i = 0; i < m_patterns.size(); ++i )
    {
        if( globPlain( m_patterns[i], aRef ) )
        {
            m_hit[i] = 1;
            any = true;
        }
    }

    return any;
}


nlohmann::json KOPENAPI_REF_FILTER::Unmatched() const
{
    nlohmann::json out = nlohmann::json::array();

    for( size_t i = 0; i < m_patterns.size(); ++i )
    {
        if( !m_hit[i] )
            out.push_back( m_patterns[i] );
    }

    return out;
}


static bool globPlain( const std::string& aPattern, const std::string& aText )
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


nlohmann::json KopenapiPageSchema( int aDefaultLimit )
{
    return { { "limit", { { "type", "integer" }, { "minimum", 1 }, { "maximum", 1000 }, { "default", aDefaultLimit } } },
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


nlohmann::json KopenapiPagedSchema( nlohmann::json aProperties, int aDefaultLimit )
{
    // a named object: a range-for over items() of a temporary iterates a destroyed object
    const nlohmann::json page = KopenapiPageSchema( aDefaultLimit );

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


std::string KopenapiLogicCore( const std::string& aText )
{
    auto alpha = []( char c ) { return std::isalpha( static_cast<unsigned char>( c ) ) != 0; };
    size_t k = 0;

    while( k < aText.size() && alpha( aText[k] ) )
        ++k;

    // vendor prefixes only: AD7400 is no 7400
    static const std::set<std::string> vendors = { "", "sn", "mc", "cd", "m", "mm", "dm", "hd" };
    std::string prefix = aText.substr( 0, k );

    for( char& ch : prefix )
        ch = static_cast<char>( std::tolower( static_cast<unsigned char>( ch ) ) );

    if( !vendors.count( prefix ) || ( aText.compare( k, 2, "74" ) != 0 && aText.compare( k, 2, "54" ) != 0 ) )
        return std::string();

    k += 2;

    while( k < aText.size() && alpha( aText[k] ) )   // family: LS, HC, LVC, AHCT, CBTLV
        ++k;

    size_t end = k;

    while( end < aText.size() && std::isalnum( static_cast<unsigned char>( aText[end] ) ) )
        ++end;

    std::string number = aText.substr( k, end - k );

    while( !number.empty() && alpha( number.back() ) )   // suffix: A, D, PW
        number.pop_back();

    if( number.size() < 2 || !std::isdigit( static_cast<unsigned char>( number[0] ) ) )
        return std::string();

    for( char& ch : number )
        ch = static_cast<char>( std::tolower( static_cast<unsigned char>( ch ) ) );

    return "74" + number;
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
    const std::string nameCore = KopenapiLogicCore( name );

    // "usb-a" names USB_A as well
    std::string dashedName = name;
    std::replace( dashedName.begin(), dashedName.end(), '_', '-' );

    // A one-letter term ("USB C") is a whole token of the text, never any letter inside a word
    auto hasToken = []( const std::string& aText, const std::string& aTerm )
    {
        for( size_t at = aText.find( aTerm ); at != std::string::npos; at = aText.find( aTerm, at + 1 ) )
        {
            const size_t after = at + aTerm.size();

            if( ( at == 0 || !std::isalnum( (unsigned char) aText[at - 1] ) )
                && ( after == aText.size() || !std::isalnum( (unsigned char) aText[after] ) ) )
                return true;
        }

        return false;
    };

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
        auto in = [&]( const std::string& aText )
        {
            return term.size() == 1 ? hasToken( aText, term ) : aText.find( term ) != std::string::npos;
        };

        if( name == term || dashedName == term )
            termScore = 100;
        else if( term.size() > 1 && ( name.rfind( term, 0 ) == 0 || dashedName.rfind( term, 0 ) == 0 ) )
            termScore = 30;
        else if( in( name ) || in( dashedName ) )
            termScore = 8;
        else if( !nameCore.empty() && KopenapiLogicCore( term ) == nameCore )
            termScore = 20;   // the same logic function in another family: 74LS257 for 74LVC257

        if( in( keywords ) )
            termScore += 12;

        if( in( description ) )
            termScore += 6;

        if( termScore == 0 )
        {
            result.unmatched.push_back( term );
            continue;
        }

        result.matched++;
        result.score += termScore;
    }

    // the words together are the name ("USB A" = USB_A, "usb c" leads USB_C_Receptacle): before USB3_A
    std::string joined;

    for( size_t at = 0; at < query.size(); )
    {
        const size_t end = query.find_first_of( " \t", at );
        const std::string term = query.substr( at, end == std::string::npos ? std::string::npos : end - at );
        at = end == std::string::npos ? query.size() : end + 1;

        if( !term.empty() )
            joined += ( joined.empty() ? "" : "_" ) + term;
    }

    if( result.terms > 1 && ( name == joined || name.rfind( joined + "_", 0 ) == 0 ) )
        result.score += 40;

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

    nlohmann::json page = KopenapiPage( rows, aArgs, KOPENAPI_RANKED_PAGE );

    if( !anyFull && !rows.empty() )
    {
        page["partial"] = true;
        page["note"] = "no item matches every word; best partial matches first (fewer or different words may help)";
    }

    return page;
}


wxString KopenapiCanonicalPath( const wxString& aPath )
{
    std::error_code             ec;
    const std::filesystem::path in( aPath.ToStdString( wxConvUTF8 ) );
    std::filesystem::path       out = std::filesystem::weakly_canonical( std::filesystem::absolute( in, ec ), ec );

    if( ec || out.empty() )
        return aPath;

    return wxString::FromUTF8( out.string() );
}


KOPENAPI_LIST_CAP::KOPENAPI_LIST_CAP( const nlohmann::json& aArgs, size_t aDefault ) :
        m_limit( aDefault ),
        m_full( aArgs.is_object() && aArgs.value( "full", false ) )
{
    if( aArgs.is_object() && aArgs.contains( "list_limit" ) && aArgs["list_limit"].is_number() )
        m_limit = (size_t) std::max( 0.0, aArgs["list_limit"].get<double>() );

    if( m_full )
        m_limit = std::numeric_limits<size_t>::max();
}


void KOPENAPI_LIST_CAP::mark( nlohmann::json& aOut, const std::string& aKey, size_t aTotal ) const
{
    aOut[aKey + "_total"] = aTotal;

    if( !aOut.contains( "truncated" ) || !aOut["truncated"].is_array() )
        aOut["truncated"] = nlohmann::json::array();

    aOut["truncated"].push_back( aKey );
}


void KOPENAPI_LIST_CAP::Put( nlohmann::json& aOut, const std::string& aKey, nlohmann::json aList ) const
{
    if( !aList.is_array() || aList.size() <= m_limit )
    {
        aOut[aKey] = std::move( aList );
        return;
    }

    const size_t total = aList.size();
    aList.erase( aList.begin() + (std::ptrdiff_t) m_limit, aList.end() );
    aOut[aKey] = std::move( aList );
    mark( aOut, aKey, total );
}


void KOPENAPI_LIST_CAP::PutPreferring( nlohmann::json& aOut, const std::string& aKey, nlohmann::json aList,
                                       const std::function<bool( const nlohmann::json& )>& aFirst ) const
{
    if( !aList.is_array() || aList.size() <= m_limit )
    {
        aOut[aKey] = std::move( aList );
        return;
    }

    std::vector<bool> keep( aList.size(), false );
    size_t            kept = 0;

    for( size_t i = 0; i < aList.size() && kept < m_limit; ++i )
    {
        if( aFirst( aList[i] ) )
        {
            keep[i] = true;
            kept++;
        }
    }

    for( size_t i = 0; i < aList.size() && kept < m_limit; ++i )
    {
        if( !keep[i] )
        {
            keep[i] = true;
            kept++;
        }
    }

    nlohmann::json cut = nlohmann::json::array();

    for( size_t i = 0; i < aList.size(); ++i )
    {
        if( keep[i] )
            cut.push_back( std::move( aList[i] ) );
    }

    aOut[aKey] = std::move( cut );
    mark( aOut, aKey, aList.size() );
}


void KOPENAPI_LIST_CAP::PutObject( nlohmann::json& aOut, const std::string& aKey,
                                   const std::vector<std::pair<std::string, nlohmann::json>>& aEntries ) const
{
    nlohmann::json obj = nlohmann::json::object();

    for( size_t i = 0; i < aEntries.size() && i < m_limit; ++i )
        obj[aEntries[i].first] = aEntries[i].second;

    aOut[aKey] = std::move( obj );

    if( aEntries.size() > m_limit )
        mark( aOut, aKey, aEntries.size() );
}


nlohmann::json KOPENAPI_LIST_CAP::Schema( nlohmann::json aSchema, size_t aDefault )
{
    if( !aSchema.contains( "type" ) )
        aSchema["type"] = "object";

    aSchema["properties"]["list_limit"] = { { "type", "integer" },
                                            { "default", aDefault },
                                            { "minimum", 0 },
                                            { "description", "entries listed per list of the answer (uuids, items, rows); "
                                                             "counts stay whole; a cut list has <key>_total and is named in truncated; 0 = counts only" } };
    aSchema["properties"]["full"] = { { "type", "boolean" },
                                      { "default", false },
                                      { "description", "every entry of every list (large on a big board)" } };
    return aSchema;
}


long KopenapiUuidMatch( const std::vector<std::string>& aUuids, const std::string& aId, std::string& aError,
                        const std::function<std::string( size_t )>& aLabel )
{
    constexpr size_t kMinPrefix = 4;
    constexpr size_t kListed = 8;

    std::string id;

    for( char c : aId )
    {
        if( !std::isspace( static_cast<unsigned char>( c ) ) )
            id.push_back( static_cast<char>( std::tolower( static_cast<unsigned char>( c ) ) ) );
    }

    if( id.empty() )
    {
        aError = "uuid: empty";
        return -1;
    }

    std::vector<size_t>   hits;
    std::set<std::string> hitUuids;

    for( size_t i = 0; i < aUuids.size(); ++i )
    {
        const std::string& uuid = aUuids[i];

        if( uuid == id )
            return (long) i;

        if( id.size() >= kMinPrefix && uuid.compare( 0, id.size(), id ) == 0 && hitUuids.insert( uuid ).second )
            hits.push_back( i );
    }

    if( hits.size() == 1 )
        return (long) hits.front();

    if( hits.empty() )
    {
        aError = "no item with uuid '" + aId + "'"
                 + ( id.size() < kMinPrefix ? std::string( " (a uuid prefix needs at least 4 characters)" ) : std::string() );
        return -1;
    }

    std::string listed;

    for( size_t i = 0; i < hits.size() && i < kListed; ++i )
    {
        listed += ( i ? ", " : "" ) + aUuids[hits[i]];

        if( aLabel )
            listed += " (" + aLabel( hits[i] ) + ")";
    }

    if( hits.size() > kListed )
        listed += ", ...";

    aError = "uuid prefix '" + aId + "' is ambiguous: " + std::to_string( hits.size() )
             + " items start with it (" + listed + "); use the full uuid";
    return -1;
}
