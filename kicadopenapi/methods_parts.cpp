/*
 * kicadopenapi part selection (ROADMAP task 2.2): part_find.
 *
 * One call from "what the design needs" — a function or part name, a value, a package — to
 * parts ready to place: the library symbol, the value to give it, footprints that fit (the
 * symbol's own footprint, then its footprint filters + pin count + package words), and the
 * arguments for sch_symbol_add.  Generic passives (resistor, capacitor, LED, ...) map to KiCad's
 * Device symbols, whose footprint is chosen by package (0603, 1206, axial, ...).
 *
 * Lives in kicommon and combines the library methods of both editors through the registry
 * (sch_lib_symbol_search / _get, pcb_lib_footprint_search).
 */
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <kiway.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <initializer_list>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>


namespace
{

KOPENAPI_RESULT callMethod( KOPENAPI_CONTEXT& aCtx, const std::string& aName, const nlohmann::json& aArgs )
{
    std::optional<KOPENAPI_METHOD> method = KOPENAPI_REGISTRY::Get().Find( aName );

    if( !method )
        return KOPENAPI_RESULT::Error( 503, aName + " is not available (editor module not loaded)" );

    return method->handler( aCtx, aArgs );
}


std::string lower( std::string aText )
{
    for( char& ch : aText )
        ch = static_cast<char>( std::tolower( static_cast<unsigned char>( ch ) ) );

    return aText;
}


std::vector<std::string> words( const std::string& aText )
{
    std::vector<std::string> out;
    std::istringstream       in( lower( aText ) );
    std::string              word;

    while( in >> word )
        out.push_back( word );

    return out;
}


/// "D5.0mm" -> "5mm", "P2.50mm" -> "2.5mm": a dimension token without its letter and trailing zeros
std::string normalizeToken( std::string aToken )
{
    if( aToken.size() > 1 && std::isalpha( (unsigned char) aToken[0] ) && std::isdigit( (unsigned char) aToken[1] ) )
        aToken.erase( 0, 1 );

    const size_t dot = aToken.find( '.' );

    if( dot != std::string::npos )
    {
        size_t end = dot + 1;

        while( end < aToken.size() && std::isdigit( (unsigned char) aToken[end] ) )
            ++end;

        std::string fraction = aToken.substr( dot + 1, end - dot - 1 );

        while( !fraction.empty() && fraction.back() == '0' )
            fraction.pop_back();

        aToken = aToken.substr( 0, dot ) + ( fraction.empty() ? "" : "." + fraction ) + aToken.substr( end );
    }

    return aToken;
}


/**
 * How well a footprint name fits package words: per word 3 for a whole name token ("0603" in
 * R_0603_1608Metric, not in R_0201_0603Metric), 2 for the same dimension ("5mm" = "D5.0mm"),
 * 1 for a substring; 0 when a word does not occur at all.
 */
int packageScore( const std::vector<std::string>& aPackage, const std::string& aFootprint )
{
    std::string name = lower( aFootprint );
    name = name.substr( name.find( ':' ) == std::string::npos ? 0 : name.find( ':' ) + 1 );

    std::vector<std::string> tokens;
    std::string              token;

    for( char ch : name + "_" )
    {
        if( ch == '_' )
        {
            if( !token.empty() )
                tokens.push_back( token );

            token.clear();
        }
        else
        {
            token += ch;
        }
    }

    int total = 0;

    for( const std::string& word : aPackage )
    {
        int best = 0;

        for( const std::string& t : tokens )
        {
            if( t == word )
                best = std::max( best, 3 );
            else if( normalizeToken( t ) == normalizeToken( word ) )
                best = std::max( best, 2 );
        }

        if( best == 0 && name.find( word ) != std::string::npos )
            best = 1;

        if( best == 0 )
            return 0;

        total += best;
    }

    return total;
}


/// Generic parts: the word an engineer uses -> KiCad's generic symbol
const std::vector<std::pair<std::vector<std::string>, std::string>>& genericParts()
{
    static const std::vector<std::pair<std::vector<std::string>, std::string>> table = {
        { { "electrolytic", "polarized", "polarised", "tantalum" }, "Device:C_Polarized" },
        { { "resistor", "res" },                                      "Device:R" },
        { { "capacitor", "cap" },                                     "Device:C" },
        { { "inductor", "choke" },                                    "Device:L" },
        { { "led" },                                                  "Device:LED" },
        { { "schottky" },                                             "Device:D_Schottky" },
        { { "zener" },                                                "Device:D_Zener" },
        { { "diode" },                                                "Device:D" },
        { { "ferrite", "bead" },                                      "Device:FerriteBead" },
        { { "crystal", "xtal" },                                      "Device:Crystal" },
        { { "fuse" },                                                 "Device:Fuse" } };

    return table;
}


/// Rows x pins from "2x3", "1x04", "4 pin", "4-pin", "4pin", "4 way"; 0 when not given
std::pair<int, int> connectorLayout( const std::vector<std::string>& aQuery )
{
    for( size_t k = 0; k < aQuery.size(); ++k )
    {
        const std::string& w = aQuery[k];
        int                rows = 0, cols = 0;
        char               x = 0;

        if( std::sscanf( w.c_str(), "%d%c%d", &rows, &x, &cols ) == 3 && x == 'x' && rows >= 1 && rows <= 2 && cols >= 1
            && cols <= 40 )
        {
            return { rows, cols };
        }

        char unit[16] = {};

        if( std::sscanf( w.c_str(), "%d%15s", &cols, unit ) == 2 && cols >= 1 && cols <= 40
            && ( !std::strcmp( unit, "pin" ) || !std::strcmp( unit, "-pin" ) || !std::strcmp( unit, "p" )
                 || !std::strcmp( unit, "-way" ) || !std::strcmp( unit, "way" ) ) )
        {
            return { 1, cols };
        }

        if( std::sscanf( w.c_str(), "%d", &cols ) == 1 && std::to_string( cols ) == w && k + 1 < aQuery.size()
            && ( aQuery[k + 1] == "pin" || aQuery[k + 1] == "pins" || aQuery[k + 1] == "way" || aQuery[k + 1] == "position" )
            && cols >= 1 && cols <= 40 )
        {
            return { 1, cols };
        }
    }

    return { 0, 0 };
}


bool hasWord( const std::vector<std::string>& aQuery, std::initializer_list<const char*> aWords )
{
    return std::any_of( aQuery.begin(), aQuery.end(),
                        [&]( const std::string& w )
                        { return std::any_of( aWords.begin(), aWords.end(), [&]( const char* x ) { return w == x; } ); } );
}


/**
 * A generic part for the query, with the package words it implies:
 *  - "screw terminal 2 pin" -> Connector:Screw_Terminal_01x02, Phoenix MKDS 5.08 mm by default
 *  - "pin header 2x3", "JST XH 4 pin connector" -> Connector_Generic:Conn_02x03_Odd_Even /
 *    Conn_01x04, the rows x pins token ("2x03") and a 2.54 mm vertical pin header by default
 *  - resistor, capacitor, LED, ... -> Device:*
 */
std::string genericSymbol( const std::vector<std::string>& aQuery, std::vector<std::string>& aDefaultPackage,
                           std::vector<std::string>& aLayout )
{
    const auto [rows, cols] = connectorLayout( aQuery );
    char       buffer[64];

    if( cols > 0 && hasWord( aQuery, { "screw", "terminal" } ) )
    {
        std::snprintf( buffer, sizeof( buffer ), "1x%02d", cols );
        aLayout = { buffer };
        aDefaultPackage = { "phoenix", "p5.08mm" };
        std::snprintf( buffer, sizeof( buffer ), "Connector:Screw_Terminal_01x%02d", cols );
        return buffer;
    }

    if( cols > 0 && hasWord( aQuery, { "header", "connector", "conn", "socket", "jst", "molex", "receptacle", "plug" } ) )
    {
        std::snprintf( buffer, sizeof( buffer ), "%dx%02d", rows, cols );
        aLayout = { buffer };
        aDefaultPackage = { "pinheader", "p2.54mm", "vertical" };
        std::snprintf( buffer, sizeof( buffer ),
                       rows == 1 ? "Connector_Generic:Conn_01x%02d" : "Connector_Generic:Conn_02x%02d_Odd_Even", cols );
        return buffer;
    }

    for( const auto& [names, libId] : genericParts() )
    {
        for( const std::string& word : aQuery )
        {
            if( std::find( names.begin(), names.end(), word ) != names.end() )
                return libId;
        }
    }

    return std::string();
}


/// "10k", "4.7k", "4k7", "100nf", "10uf", "330", "16mhz", "1m" ...: a component value
bool looksLikeValue( const std::string& aWord )
{
    if( aWord.empty() || !std::isdigit( (unsigned char) aWord[0] ) )
        return false;

    // four digits are a chip size (0402, 0603, 1206, 2512), not a value
    if( aWord.size() == 4 && std::all_of( aWord.begin(), aWord.end(), ::isdigit ) )
        return false;

    size_t k = 0;

    while( k < aWord.size() && ( std::isdigit( (unsigned char) aWord[k] ) || aWord[k] == '.' ) )
        ++k;

    static const std::set<std::string> units = { "", "r", "k", "m", "g", "ohm", "kohm", "mohm", "p", "n", "u", "pf", "nf", "uf",
                                                 "mf", "f", "nh", "uh", "mh", "h", "hz", "khz", "mhz", "v", "a", "ma", "w" };
    std::string rest = aWord.substr( k );

    // "4k7" style
    if( rest.size() >= 2 && std::isalpha( (unsigned char) rest[0] ) && std::all_of( rest.begin() + 1, rest.end(), ::isdigit ) )
        return true;

    return units.count( rest ) > 0;
}


/// Words of a generic part's query that describe its package / series ("jst", "xh", "0603")
std::vector<std::string> packageWordsOf( const std::vector<std::string>& aQuery )
{
    static const std::set<std::string> skip = {
        "pin",       "pins",     "way",     "position", "connector", "conn",    "header",  "socket", "terminal", "screw",
        "resistor",  "res",      "capacitor", "cap",    "inductor",  "choke",   "led",     "diode",  "schottky", "zener",
        "ferrite",   "bead",     "crystal", "xtal",     "fuse",      "electrolytic", "polarized", "polarised", "tantalum",
        "smd",       "tht",      "a",       "an",       "for",       "with",    "of" };
    std::vector<std::string> out;

    for( const std::string& w : aQuery )
    {
        int  n = 0;
        char x = 0;
        int  m = 0;

        if( skip.count( w ) || looksLikeValue( w ) || std::sscanf( w.c_str(), "%d%c%d", &n, &x, &m ) == 3 )
            continue;

        if( w.size() < 4 && std::sscanf( w.c_str(), "%d", &n ) == 1 && std::to_string( n ) == w )
            continue;   // a pin count (four digits are a chip size: 0603, 1206)

        if( w.size() > 3 && ( w.ends_with( "pin" ) || w.ends_with( "way" ) ) && std::isdigit( (unsigned char) w[0] ) )
            continue;

        out.push_back( w );
    }

    return out;
}

} // namespace


static KOPENAPI_RESULT h_part_find( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string query = aArgs.value( "query", std::string() );

    if( query.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'query': what the part is (e.g. 'resistor', 'LED', 'ATtiny85', 'LDO 3.3V')" );

    const std::vector<std::string> queryWords = words( query );
    std::string                    value = aArgs.value( "value", std::string() );
    const std::vector<std::string> askedPackage = words( aArgs.value( "package", std::string() ) );
    const std::string              mount = aArgs.value( "mount", std::string() );
    const int                      limit = std::clamp( aArgs.value( "limit", 3 ), 1, 10 );
    const int                      footprints = std::clamp( aArgs.value( "footprints", 3 ), 1, 10 );

    if( !mount.empty() && mount != "smd" && mount != "tht" )
        return KOPENAPI_RESULT::Error( 400, "mount: smd or tht" );

    if( aCtx.kiway )
    {
        aCtx.kiway->KiFACE( KIWAY::FACE_SCH );
        aCtx.kiway->KiFACE( KIWAY::FACE_PCB );
    }

    // Symbols: a generic one (passives, connectors), a part number given as value, else a
    // library search ranked by fit to the package
    std::vector<std::string> defaultPackage, layout;
    const std::string        generic = genericSymbol( queryWords, defaultPackage, layout );
    std::vector<std::string> symbols;
    bool                     partial = false;

    // Package attempts, best first: asked words + words of the query (series: "jst xh"), then
    // asked words alone, then the generic default
    std::vector<std::vector<std::string>> attempts;

    if( !generic.empty() )
    {
        symbols.push_back( generic );

        // A value written into the query ("resistor 10k 0603"), as written (case kept); not for
        // connectors, whose numbers are pin counts
        if( value.empty() && generic.find( "Conn" ) == std::string::npos && generic.find( "Terminal" ) == std::string::npos )
        {
            std::istringstream in( query );
            std::string        w;

            while( value.empty() && in >> w )
            {
                if( looksLikeValue( lower( w ) ) )
                    value = w;
            }
        }

        std::vector<std::string> extra = packageWordsOf( queryWords );
        std::vector<std::string> base = askedPackage;

        if( !extra.empty() )
        {
            std::vector<std::string> both = base;
            both.insert( both.end(), extra.begin(), extra.end() );
            both.insert( both.end(), layout.begin(), layout.end() );
            attempts.push_back( both );
        }

        if( !base.empty() )
        {
            base.insert( base.end(), layout.begin(), layout.end() );
            attempts.push_back( base );
        }

        if( askedPackage.empty() )
        {
            std::vector<std::string> fallback = defaultPackage;
            fallback.insert( fallback.end(), layout.begin(), layout.end() );
            attempts.push_back( fallback );
        }
    }
    else
    {
        attempts.push_back( askedPackage );

        // A part number given as value ("AMS1117-3.3"): symbols named like it come first
        if( !value.empty() && !looksLikeValue( lower( value ) ) )
        {
            KOPENAPI_RESULT named = callMethod( aCtx, "sch_lib_symbol_search", { { "query", value }, { "limit", 20 } } );

            if( named.status == 200 && !named.body.value( "partial", false ) )
            {
                for( const nlohmann::json& row : named.body["items"] )
                {
                    const std::string id = row["lib_id"];
                    const std::string name = lower( id.substr( id.find( ':' ) + 1 ) );

                    if( !row.value( "power", false ) && name.rfind( lower( value ), 0 ) == 0 && (int) symbols.size() < limit )
                        symbols.push_back( id );
                }
            }
        }

        KOPENAPI_RESULT found = callMethod( aCtx, "sch_lib_symbol_search", { { "query", query }, { "limit", limit * 4 } } );

        if( found.status != 200 )
            return found;

        partial = found.body.value( "partial", false );
        std::vector<std::pair<bool, std::string>> ranked;

        for( const nlohmann::json& row : found.body["items"] )
        {
            if( row.value( "power", false ) )
                continue;

            ranked.emplace_back( !askedPackage.empty() && packageScore( askedPackage, row.value( "footprint", std::string() ) ) > 0,
                                 row["lib_id"].get<std::string>() );
        }

        std::stable_sort( ranked.begin(), ranked.end(), []( const auto& a, const auto& b ) { return a.first && !b.first; } );

        for( const auto& [fits, libId] : ranked )
        {
            if( (int) symbols.size() < limit && std::find( symbols.begin(), symbols.end(), libId ) == symbols.end() )
                symbols.push_back( libId );
        }
    }

    nlohmann::json candidates = nlohmann::json::array();
    std::vector<std::string> usedPackage;

    for( const std::string& libId : symbols )
    {
        KOPENAPI_RESULT card = callMethod( aCtx, "sch_lib_symbol_get", { { "lib_id", libId } } );

        if( card.status != 200 )
            continue;

        const nlohmann::json& c = card.body;
        const std::string     ownFootprint = c.value( "footprint", std::string() );
        const int             pins = c.value( "pins", 0 );
        const nlohmann::json  filters = c.value( "footprint_filters", nlohmann::json::array() );

        // Every fitting footprint found, then ranked: package fit, source (own footprint,
        // the symbol's filters, package words only), then the libraries' own order (plain
        // names before their variants)
        struct FIT
        {
            nlohmann::json row;
            std::string    why;
            int            source;
        };

        std::vector<FIT> found;   // all footprints the three sources give, unranked
        std::set<std::string> seen;

        auto add = [&]( const nlohmann::json& aRow, const std::string& aWhy, int aSource )
        {
            const std::string id = aRow["lib_id"];

            if( seen.insert( id ).second && ( mount.empty() || aRow.value( "type", mount ) == mount ) )
                found.push_back( { aRow, aWhy, aSource } );
        };

        auto search = [&]( nlohmann::json aSearch, const std::string& aWhy, int aSource )
        {
            if( !mount.empty() )
                aSearch["type"] = mount;

            aSearch["limit"] = 2000;
            KOPENAPI_RESULT result = callMethod( aCtx, "pcb_lib_footprint_search", aSearch );

            if( result.status != 200 || result.body.value( "partial", false ) )
                return;

            for( const nlohmann::json& row : result.body["items"] )
                add( row, aWhy, aSource );
        };

        if( !ownFootprint.empty() )
        {
            KOPENAPI_RESULT own = callMethod( aCtx, "pcb_lib_footprint_get", { { "lib_id", ownFootprint } } );

            if( own.status == 200 )
                add( own.body, "the symbol's own footprint", 0 );
        }

        // no text query: package words are judged by packageScore ("5mm" = "D5.0mm")
        if( !filters.empty() )
            search( { { "filters", filters }, { "pads", pins } }, "matches the symbol's footprint filters and pin count", 1 );

        nlohmann::json fits = nlohmann::json::array();

        for( const std::vector<std::string>& package : attempts )
        {
            std::vector<FIT> pool = found;

            // by package words and pin count alone (symbols without filters, or filters too narrow)
            if( !package.empty() )
            {
                std::vector<FIT> saved = std::move( found );
                std::set<std::string> savedSeen = seen;
                found.clear();

                for( const std::string& w : package )
                {
                    // one word at a time: the footprint search needs every word, packageScore
                    // knows "5mm" = "D5.0mm"
                    search( { { "query", w }, { "pads", pins } }, "matches the package and pin count", 2 );
                }

                pool.insert( pool.end(), found.begin(), found.end() );
                found = std::move( saved );
                seen = std::move( savedSeen );
            }

            std::vector<std::pair<int, FIT>> scored;

            for( FIT& fit : pool )
            {
                const int score = package.empty() ? 1 : packageScore( package, fit.row["lib_id"] );

                if( score > 0 )
                    scored.emplace_back( score, fit );
            }

            // footprints the symbol itself names or allows beat the package-only finds (C_0603 for a resistor)
            if( std::any_of( scored.begin(), scored.end(), []( const auto& f ) { return f.second.source < 2; } ) )
                std::erase_if( scored, []( const auto& f ) { return f.second.source == 2; } );

            std::stable_sort( scored.begin(), scored.end(),
                              []( const auto& a, const auto& b )
                              {
                                  if( a.first != b.first )
                                      return a.first > b.first;

                                  return a.second.source < b.second.source;   // then the libraries' own order
                              } );

            for( const auto& [score, fit] : scored )
            {
                if( (int) fits.size() < footprints )
                {
                    fits.push_back( { { "lib_id", fit.row["lib_id"] },
                                      { "description", fit.row.value( "description", std::string() ) },
                                      { "type", fit.row.value( "type", std::string() ) },
                                      { "why", fit.why } } );
                }
            }

            if( !fits.empty() )
            {
                usedPackage = package;
                break;
            }
        }

        // A value is the generic part's value; a specific part keeps its own unless it is that part
        const std::string own = c.value( "value", std::string() );
        const std::string useValue = !generic.empty() ? ( value.empty() ? own : value )
                                     : ( !value.empty() && lower( libId ).find( lower( value ) ) != std::string::npos ? value : own );
        nlohmann::json use = { { "lib_id", libId }, { "value", useValue } };

        if( !fits.empty() )
            use["footprint"] = fits[0]["lib_id"];

        candidates.push_back( { { "symbol", libId },
                                { "description", c.value( "description", std::string() ) },
                                { "pins", pins },
                                { "symbol_footprint", ownFootprint },
                                { "footprints", fits },
                                { "use", use } } );
    }

    // With a package asked for, parts that have no footprint of it go (if any other part has one)
    if( !askedPackage.empty()
        && std::any_of( candidates.begin(), candidates.end(), []( const nlohmann::json& c ) { return !c["footprints"].empty(); } ) )
    {
        nlohmann::json kept = nlohmann::json::array();

        for( const nlohmann::json& c : candidates )
        {
            if( !c["footprints"].empty() )
                kept.push_back( c );
        }

        candidates = kept;
    }

    nlohmann::json result = { { "query", query }, { "candidates", candidates } };

    if( !generic.empty() )
    {
        std::string words;

        for( const std::string& w : usedPackage )
            words += ( words.empty() ? "" : " " ) + w;

        result["note"] = "generic part " + generic + "; footprint chosen by the package words '" + words + "'"
                         + ( askedPackage.empty() && usedPackage == [&]
                                     {
                                         std::vector<std::string> d = defaultPackage;
                                         d.insert( d.end(), layout.begin(), layout.end() );
                                         return d;
                                     }()
                                     ? " (the default for this kind of part; give package for another)"
                                     : "" );
    }

    if( partial )
        result["partial"] = true;

    if( candidates.empty() )
        result["hint"] = "nothing found: fewer or other words (part number without suffix, function words)";

    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "part_find",
                   "Pick a part for the schematic in one call: what it is (resistor, capacitor, LED, "
                   "diode, crystal, 'screw terminal 2 pin', 'JST XH 4 pin connector', 'pin header 2x3', ... "
                   "or a part name / function like 'ATtiny85', 'LDO 3.3V'), value (or a part number like "
                   "AMS1117-3.3), package words ('0603', 'DIP-8', 'SOT-223'), smd/tht -> library symbols with "
                   "footprints that fit (own footprint, footprint filters + pin count + package / series "
                   "words) and 'use' = arguments for sch_symbol_add (lib_id, value, footprint)",
                   R"json({"type":"object","required":["query"],"properties":{
                        "query":{"type":"string","description":"what the part is: function or part name"},
                        "value":{"type":"string","description":"e.g. 10k, 100nF, or a part number (AMS1117-3.3)"},
                        "package":{"type":"string","description":"words in the footprint name, e.g. 0603, DIP-8, SOT-23"},
                        "mount":{"type":"string","enum":["smd","tht"]},
                        "limit":{"type":"integer","default":3,"description":"symbols, 1..10"},
                        "footprints":{"type":"integer","default":3,"description":"footprints per symbol, 1..10"}}})json"_json,
                   false, h_part_find, 600 );
