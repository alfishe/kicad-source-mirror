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


/// "pin header 2x3", "connector 1x4" -> Connector_Generic:Conn_02x03_Odd_Even / Conn_01x04
std::string genericConnector( const std::vector<std::string>& aQuery )
{
    const bool connector = std::any_of( aQuery.begin(), aQuery.end(), []( const std::string& w )
                                        { return w == "header" || w == "connector" || w == "conn" || w == "socket"; } );

    if( !connector )
        return std::string();

    for( const std::string& w : aQuery )
    {
        int rows = 0, cols = 0;
        char x = 0;

        if( std::sscanf( w.c_str(), "%d%c%d", &rows, &x, &cols ) == 3 && x == 'x' && rows >= 1 && rows <= 2 && cols >= 1
            && cols <= 40 )
        {
            char buffer[64];
            std::snprintf( buffer, sizeof( buffer ), rows == 1 ? "Connector_Generic:Conn_01x%02d" : "Connector_Generic:Conn_02x%02d_Odd_Even",
                           cols );
            return buffer;
        }
    }

    return std::string();
}


std::string genericSymbol( const std::vector<std::string>& aQuery )
{
    if( std::string conn = genericConnector( aQuery ); !conn.empty() )
        return conn;

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

} // namespace


static KOPENAPI_RESULT h_part_find( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string query = aArgs.value( "query", std::string() );

    if( query.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'query': what the part is (e.g. 'resistor', 'LED', 'ATtiny85', 'LDO 3.3V')" );

    const std::string              value = aArgs.value( "value", std::string() );
    std::vector<std::string>       package = words( aArgs.value( "package", std::string() ) );
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

    // Symbols: the generic one for a passive, else a library search ranked by fit to the package
    std::vector<std::string> symbols;
    const std::string        generic = genericSymbol( words( query ) );
    bool                     partial = false;

    if( !generic.empty() )
    {
        symbols.push_back( generic );

        // A generic connector: its rows x pins in the footprint name ("2x03"), and without a
        // package the usual 2.54 mm vertical pin header
        if( generic.rfind( "Connector_Generic:Conn_", 0 ) == 0 )
        {
            // "02x03" in the symbol name, "2x03" in footprint names
            const std::string layout = lower( generic.substr( std::string( "Connector_Generic:Conn_" ).size() + 1, 4 ) );

            if( package.empty() )
                package = { "pinheader", "p2.54mm", "vertical" };

            package.push_back( layout );
        }
    }
    else
    {
        KOPENAPI_RESULT found = callMethod( aCtx, "sch_lib_symbol_search", { { "query", query }, { "limit", limit * 4 } } );

        if( found.status != 200 )
            return found;

        partial = found.body.value( "partial", false );
        std::vector<std::pair<bool, std::string>> ranked;

        for( const nlohmann::json& row : found.body["items"] )
        {
            if( row.value( "power", false ) )
                continue;

            ranked.emplace_back( !package.empty() && packageScore( package, row.value( "footprint", std::string() ) ) > 0,
                                 row["lib_id"].get<std::string>() );
        }

        std::stable_sort( ranked.begin(), ranked.end(), []( const auto& a, const auto& b ) { return a.first && !b.first; } );

        for( const auto& [fits, libId] : ranked )
        {
            if( (int) symbols.size() < limit )
                symbols.push_back( libId );
        }
    }

    nlohmann::json candidates = nlohmann::json::array();

    for( const std::string& libId : symbols )
    {
        KOPENAPI_RESULT card = callMethod( aCtx, "sch_lib_symbol_get", { { "lib_id", libId } } );

        if( card.status != 200 )
            continue;

        const nlohmann::json& c = card.body;
        const std::string     ownFootprint = c.value( "footprint", std::string() );
        const int             pins = c.value( "pins", 0 );
        // Every fitting footprint found, then ranked: package fit, source (own footprint,
        // the symbol's filters, package words only), then the libraries' own order (plain
        // names before their variants)
        struct FIT
        {
            nlohmann::json row;
            std::string    why;
            int            source;
            int            score;
        };

        std::vector<FIT>      pool;
        std::set<std::string> seen;

        auto add = [&]( const nlohmann::json& aRow, const std::string& aWhy )
        {
            const std::string id = aRow["lib_id"];

            if( !seen.insert( id ).second )
                return;

            if( !mount.empty() && aRow.value( "type", mount ) != mount )
                return;

            const int score = package.empty() ? 1 : packageScore( package, id );

            if( score > 0 )
                pool.push_back( { aRow, aWhy, aWhy.rfind( "the symbol's own", 0 ) == 0 ? 0 : aWhy.find( "filters" ) != std::string::npos ? 1 : 2, score } );
        };

        // 1. the symbol's own footprint when it fits the package asked for
        if( !ownFootprint.empty() )
        {
            KOPENAPI_RESULT own = callMethod( aCtx, "pcb_lib_footprint_get", { { "lib_id", ownFootprint } } );

            if( own.status == 200 )
                add( own.body, "the symbol's own footprint" );
        }

        // 2. its footprint filters + pin count (+ package words, mount)
        std::string packageQuery;

        for( const std::string& w : package )
            packageQuery += ( packageQuery.empty() ? "" : " " ) + w;

        auto search = [&]( nlohmann::json aSearch, const std::string& aWhy )
        {
            if( !mount.empty() )
                aSearch["type"] = mount;

            aSearch["limit"] = 1000;
            KOPENAPI_RESULT found = callMethod( aCtx, "pcb_lib_footprint_search", aSearch );

            if( found.status != 200 || found.body.value( "partial", false ) )
                return;

            for( const nlohmann::json& row : found.body["items"] )
                add( row, aWhy );
        };

        const nlohmann::json filters = c.value( "footprint_filters", nlohmann::json::array() );

        if( !filters.empty() )
        {
            // no text query: the package words are judged by packageScore ("5mm" = "D5.0mm")
            search( { { "filters", filters }, { "pads", pins } }, "matches the symbol's footprint filters and pin count" );
        }

        // 3. by package words and pin count alone (symbols without filters, or filters too narrow)
        if( !packageQuery.empty() )
            search( { { "query", packageQuery }, { "pads", pins } }, "matches the package and pin count" );

        std::stable_sort( pool.begin(), pool.end(),
                          []( const FIT& a, const FIT& b )
                          {
                              if( a.score != b.score )
                                  return a.score > b.score;

                              return a.source < b.source;   // then the libraries' own order
                          } );

        nlohmann::json fits = nlohmann::json::array();

        for( const FIT& fit : pool )
        {
            if( (int) fits.size() < footprints )
            {
                fits.push_back( { { "lib_id", fit.row["lib_id"] },
                                  { "description", fit.row.value( "description", std::string() ) },
                                  { "type", fit.row.value( "type", std::string() ) },
                                  { "why", fit.why } } );
            }
        }

        const std::string useValue = !value.empty() ? value : c.value( "value", std::string() );
        nlohmann::json    use = { { "lib_id", libId }, { "value", useValue } };

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
    if( !package.empty()
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
        result["note"] = "generic part: " + generic + " with the value set; the footprint chosen by package"
                         + std::string( generic.rfind( "Connector_Generic:", 0 ) == 0 && aArgs.value( "package", std::string() ).empty()
                                                ? " (default: 2.54 mm vertical pin header; give package for another)"
                                                : "" );

    if( partial )
        result["partial"] = true;

    if( candidates.empty() )
        result["hint"] = "nothing found: fewer or other words (part number without suffix, function words)";

    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "part_find",
                   "Pick a part for the schematic in one call: what it is (resistor, capacitor, LED, "
                   "diode, crystal, ... or a part name / function like 'ATtiny85', 'LDO 3.3V'), value, "
                   "package words ('0603', 'DIP-8', 'SOT-223'), smd/tht -> library symbols with footprints "
                   "that fit (own footprint, footprint filters + pin count + package) and 'use' = "
                   "arguments for sch_symbol_add (lib_id, value, footprint)",
                   R"json({"type":"object","required":["query"],"properties":{
                        "query":{"type":"string","description":"what the part is: function or part name"},
                        "value":{"type":"string","description":"e.g. 10k, 100nF"},
                        "package":{"type":"string","description":"words in the footprint name, e.g. 0603, DIP-8, SOT-23"},
                        "mount":{"type":"string","enum":["smd","tht"]},
                        "limit":{"type":"integer","default":3,"description":"symbols, 1..10"},
                        "footprints":{"type":"integer","default":3,"description":"footprints per symbol, 1..10"}}})json"_json,
                   false, h_part_find, 600 );
