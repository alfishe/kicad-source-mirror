/// @file methods_design.cpp
/// @brief kicadopenapi cross-domain methods (docs/api/design-analysis-api.md principle A7): schematic <-> board.
///
///   design_parity  compare schematic and board: components, and nets by connectivity
///   net_get        one net seen from both sides, matched through pads
///
/// They live in kicommon and combine the domain methods through the registry (sch_netlist,
/// pcb_netlist, sch_net_get, pcb_net_get), so eeschema and pcbnew stay independent.  Nets are
/// matched by their pads (REF.PAD), not by name: converted designs name auto nets differently
/// on each side.
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>

#include <wx/filename.h>

#include <algorithm>
#include <map>
#include <set>


namespace
{

/// @brief Call another registered method in-process (we already run on the main thread)
KOPENAPI_RESULT callMethod( KOPENAPI_CONTEXT& aCtx, const std::string& aName, const nlohmann::json& aArgs = nlohmann::json::object() )
{
    std::optional<KOPENAPI_METHOD> method = KOPENAPI_REGISTRY::Get().Find( aName );

    if( !method )
        return KOPENAPI_RESULT::Error( 503, aName + " is not available (editor module not loaded)" );

    return method->handler( aCtx, aArgs );
}


std::string itemName( const std::string& aLibId )
{
    const size_t colon = aLibId.find( ':' );
    return colon == std::string::npos ? aLibId : aLibId.substr( colon + 1 );
}


/// @brief Bounded list with its full count
struct LIST
{
    nlohmann::json items = nlohmann::json::array();
    int            count = 0;
    int            limit = 50;

    void add( nlohmann::json aItem )
    {
        if( count++ < limit )
            items.push_back( std::move( aItem ) );
    }

    nlohmann::json json() const { return { { "count", count }, { "items", items } }; }
};

} // namespace


static KOPENAPI_RESULT h_design_parity( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_RESULT sch = callMethod( aCtx, "sch_netlist" );
    KOPENAPI_RESULT pcb = callMethod( aCtx, "pcb_netlist" );

    if( sch.status != 200 || pcb.status != 200 )
    {
        return KOPENAPI_RESULT::Error( 409, "design_parity needs both the schematic and the board open "
                                            "(sch_open + pcb_open of one project)" );
    }

    const int limit = std::clamp( aArgs.value( "list_limit", 50 ), 0, 100000 );
    auto      list = [limit]() { LIST l; l.limit = limit; return l; };

    // ---- components ----
    std::map<std::string, nlohmann::json> schComps, pcbComps;
    int                                   boardOnly = 0, unnamedOnBoard = 0, notOnBoard = 0;

    for( const nlohmann::json& c : sch.body["components"] )
    {
        if( c["on_board"].get<bool>() )
            schComps[c["ref"]] = c;
        else
            notOnBoard++;
    }

    for( const nlohmann::json& f : pcb.body["footprints"] )
    {
        if( f["ref"].get<std::string>().empty() )
            unnamedOnBoard++;
        else if( f["board_only"].get<bool>() )
            boardOnly++;
        else
            pcbComps[f["ref"]] = f;
    }

    LIST missingOnBoard = list(), missingInSchematic = list(), valueDiff = list(), footprintDiff = list();
    int  matched = 0, libraryOnlyDiff = 0;

    for( const auto& [ref, c] : schComps )
    {
        auto f = pcbComps.find( ref );

        if( f == pcbComps.end() )
        {
            missingOnBoard.add( ref );
            continue;
        }

        matched++;

        // "~" is KiCad's legacy spelling of an empty value
        auto value = []( const nlohmann::json& aValue )
        {
            const std::string v = aValue;
            return v == "~" ? std::string() : v;
        };

        if( value( c["value"] ) != value( f->second["value"] ) )
            valueDiff.add( { { "ref", ref }, { "schematic", c["value"] }, { "board", f->second["value"] } } );

        const std::string a = c["footprint"], b = f->second["footprint"];

        if( a != b )
        {
            if( itemName( a ) == itemName( b ) )
                libraryOnlyDiff++;
            else
                footprintDiff.add( { { "ref", ref }, { "schematic", a }, { "board", b } } );
        }
    }

    for( const auto& [ref, f] : pcbComps )
    {
        if( !schComps.count( ref ) )
            missingInSchematic.add( ref );
    }

    // ---- nets, by connectivity ----
    std::map<std::string, std::string>              schNetOfPad, pcbNetOfPad;
    std::map<std::string, std::vector<std::string>> schPads, pcbPads;
    std::set<std::string>                           padsWithoutNet;

    for( const nlohmann::json& n : sch.body["nets"] )
    {
        for( const nlohmann::json& p : n["pads"] )
        {
            schNetOfPad[p] = n["name"];
            schPads[n["name"]].push_back( p );
        }
    }

    for( const nlohmann::json& n : pcb.body["nets"] )
    {
        for( const nlohmann::json& p : n["pads"] )
        {
            pcbNetOfPad[p] = n["name"];
            pcbPads[n["name"]].push_back( p );
        }
    }

    for( const nlohmann::json& p : pcb.body["pads_without_net"] )
        padsWithoutNet.insert( p );

    LIST renamed = list(), split = list(), merged = list(), onlySchematic = list(), onlyBoard = list(),
         unconnected = list(), netless = list();
    int  identical = 0;

    for( const auto& [name, pads] : schPads )
    {
        std::set<std::string> boardNets;
        std::vector<std::string> onBoard;

        for( const std::string& pad : pads )
        {
            if( auto it = pcbNetOfPad.find( pad ); it != pcbNetOfPad.end() )
            {
                boardNets.insert( it->second );
                onBoard.push_back( pad );
            }
            else if( padsWithoutNet.count( pad ) )
            {
                // a single-pin net is an unconnected pin: the board pad simply has no net
                if( pads.size() == 1 )
                    unconnected.add( { { "pad", pad }, { "schematic_net", name } } );
                else
                    netless.add( { { "pad", pad }, { "schematic_net", name } } );
            }
            else
            {
                onlySchematic.add( { { "pad", pad }, { "schematic_net", name } } );
            }
        }

        if( boardNets.size() > 1 )
        {
            nlohmann::json parts = nlohmann::json::array();

            for( const std::string& b : boardNets )
                parts.push_back( b );

            split.add( { { "schematic_net", name }, { "board_nets", parts } } );
        }
        else if( boardNets.size() == 1 )
        {
            const std::string& b = *boardNets.begin();
            std::set<std::string> boardSide( pcbPads[b].begin(), pcbPads[b].end() );
            std::set<std::string> schSide( onBoard.begin(), onBoard.end() );

            if( boardSide == schSide || std::includes( schSide.begin(), schSide.end(), boardSide.begin(), boardSide.end() ) )
            {
                if( b == name )
                    identical++;
                else
                    renamed.add( { { "schematic", name }, { "board", b } } );
            }
        }
    }

    for( const auto& [name, pads] : pcbPads )
    {
        std::set<std::string> schNets;

        for( const std::string& pad : pads )
        {
            if( auto it = schNetOfPad.find( pad ); it != schNetOfPad.end() )
                schNets.insert( it->second );
            else
                onlyBoard.add( { { "pad", pad }, { "board_net", name } } );
        }

        if( schNets.size() > 1 )
        {
            nlohmann::json parts = nlohmann::json::array();

            for( const std::string& s : schNets )
                parts.push_back( s );

            merged.add( { { "board_net", name }, { "schematic_nets", parts } } );
        }
    }

    int anonymous = 0;

    for( const nlohmann::json& n : pcb.body["nets"] )
        anonymous += n["anonymous_pads"].get<int>();

    const bool consistent = missingOnBoard.count == 0 && missingInSchematic.count == 0 && valueDiff.count == 0
                            && footprintDiff.count == 0 && split.count == 0 && merged.count == 0
                            && onlySchematic.count == 0 && onlyBoard.count == 0 && netless.count == 0;

    return KOPENAPI_RESULT::Ok(
            { { "consistent", consistent },
              { "components",
                { { "schematic", schComps.size() },
                  { "board", pcbComps.size() },
                  { "matched", matched },
                  { "missing_on_board", missingOnBoard.json() },
                  { "missing_in_schematic", missingInSchematic.json() },
                  { "value_differs", valueDiff.json() },
                  { "footprint_differs", footprintDiff.json() },
                  { "footprint_library_only_differs", libraryOnlyDiff },
                  { "excluded_from_board", notOnBoard },
                  { "board_only_footprints", boardOnly },
                  { "unnamed_footprints", unnamedOnBoard } } },
              { "nets",
                { { "schematic", sch.body["nets"].size() },
                  { "board", pcb.body["nets"].size() },
                  { "identical", identical },
                  { "renamed", renamed.json() },
                  { "split_on_board", split.json() },
                  { "merged_on_board", merged.json() },
                  { "pads_only_in_schematic", onlySchematic.json() },
                  { "pads_only_on_board", onlyBoard.json() },
                  { "board_pad_without_net", netless.json() },
                  { "unconnected_pins", unconnected.json() },
                  { "unmapped_schematic_pins", sch.body["unmapped_pins"].size() },
                  { "anonymous_board_pads_on_nets", anonymous } } } } );
}


static KOPENAPI_RESULT h_net_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string name = aArgs.value( "name", std::string() );
    const std::string pin = aArgs.value( "pin", std::string() );

    if( name.empty() && pin.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'name' or 'pin' (REF.PIN)" );

    nlohmann::json lookup = name.empty() ? nlohmann::json{ { "pin", pin } } : nlohmann::json{ { "name", name } };

    KOPENAPI_RESULT sch = callMethod( aCtx, "sch_net_get", lookup );
    KOPENAPI_RESULT pcb = callMethod( aCtx, "pcb_net_get", lookup );

    const bool schOpen = sch.status != 409 && sch.status != 503;
    const bool pcbOpen = pcb.status != 409 && pcb.status != 503;

    if( !schOpen && !pcbOpen )
        return KOPENAPI_RESULT::Error( 409, "neither a schematic nor a board is open" );

    // Pads of the schematic net (resolved like the netlist exporter)
    std::set<std::string> schPads;

    if( sch.status == 200 )
    {
        for( const nlohmann::json& p : sch.body["pins"] )
        {
            for( const nlohmann::json& pad : p.value( "pads", nlohmann::json::array() ) )
                schPads.insert( p["ref"].get<std::string>() + "." + pad.get<std::string>() );
        }
    }

    // Board side: the board nets holding those pads (names may differ), else the lookup result
    nlohmann::json boardNets = nlohmann::json::array();
    std::set<std::string> boardPads, names;
    nlohmann::json basis = nlohmann::json::array();

    if( pcbOpen && !schPads.empty() )
    {
        KOPENAPI_RESULT netlist = callMethod( aCtx, "pcb_netlist" );

        if( netlist.status == 200 )
        {
            for( const nlohmann::json& n : netlist.body["nets"] )
            {
                for( const nlohmann::json& p : n["pads"] )
                {
                    if( schPads.count( p ) )
                    {
                        names.insert( n["name"] );
                        break;
                    }
                }
            }
        }

        basis.push_back( "board nets found through the schematic net's pads" );
    }
    else if( pcb.status == 200 )
    {
        names.insert( pcb.body["name"] );
        basis.push_back( "board net found by the lookup itself" );
    }

    for( const std::string& n : names )
    {
        KOPENAPI_RESULT card = n == pcb.body.value( "name", std::string() ) ? pcb
                                                                            : callMethod( aCtx, "pcb_net_get", { { "name", n } } );

        if( card.status != 200 )
            continue;

        for( const nlohmann::json& p : card.body["pads"] )
        {
            if( p["pin"].is_string() )
                boardPads.insert( p["pin"].get<std::string>() );
        }

        boardNets.push_back( card.body );
    }

    // Schematic side when only the board knew the name: find it through a pad
    if( sch.status == 404 && schOpen && !boardPads.empty() )
    {
        for( const std::string& pad : boardPads )
        {
            KOPENAPI_RESULT viaPad = callMethod( aCtx, "sch_net_get", { { "pin", pad } } );

            if( viaPad.status == 200 )
            {
                sch = viaPad;
                basis.push_back( "schematic net found through board pad " + pad + " (pad number = pin number)" );

                for( const nlohmann::json& p : sch.body["pins"] )
                {
                    for( const nlohmann::json& pd : p.value( "pads", nlohmann::json::array() ) )
                        schPads.insert( p["ref"].get<std::string>() + "." + pd.get<std::string>() );
                }

                break;
            }
        }
    }

    if( sch.status != 200 && boardNets.empty() )
        return KOPENAPI_RESULT::Error( 404, "net not found in the open documents" );

    nlohmann::json onlySch = nlohmann::json::array(), onlyBoard = nlohmann::json::array();

    for( const std::string& p : schPads )
    {
        if( !boardPads.count( p ) )
            onlySch.push_back( p );
    }

    for( const std::string& p : boardPads )
    {
        if( !schPads.count( p ) )
            onlyBoard.push_back( p );
    }

    std::string link = "schematic_only";

    if( sch.status == 200 && !boardNets.empty() )
    {
        if( boardNets.size() > 1 )
            link = "split_on_board";
        else if( !onlyBoard.empty() )
            link = "board_net_has_more_pads";
        else if( !onlySch.empty() )
            link = "pads_missing_on_board";
        else
            link = boardNets[0]["name"] == sch.body["name"] ? "identical" : "renamed";
    }
    else if( !boardNets.empty() )
    {
        link = "board_only";
    }

    return KOPENAPI_RESULT::Ok( { { "name", sch.status == 200 ? sch.body["name"] : boardNets[0]["name"] },
                                  { "link", link },
                                  { "pads_only_in_schematic", onlySch },
                                  { "pads_only_on_board", onlyBoard },
                                  { "basis", basis },
                                  { "schematic", sch.status == 200 ? sch.body : nlohmann::json() },
                                  { "board", boardNets } } );
}


static KOPENAPI_RESULT h_design_update_board( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    // 1. the schematic's netlist, unsaved edits included
    KOPENAPI_RESULT sch = callMethod( aCtx, "sch_netlist_kicad" );

    if( sch.status != 200 )
        return sch;

    // 2. the board: the open one, else the project's board file, created when missing
    std::string boardPath = aArgs.value( "board_path", std::string() );

    if( boardPath.empty() )
    {
        boardPath = sch.body.value( "schematic", std::string() );   // the project file

        if( const size_t dot = boardPath.rfind( '.' ); dot != std::string::npos )
            boardPath = boardPath.substr( 0, dot ) + ".kicad_pcb";
    }

    nlohmann::json  opened;
    KOPENAPI_RESULT probe = callMethod( aCtx, "pcb_stats" );

    if( probe.status == 409 || !aArgs.value( "board_path", std::string() ).empty() )
    {
        const bool exists = !boardPath.empty() && wxFileName::FileExists( wxString::FromUTF8( boardPath ) );

        if( !exists && aArgs.value( "dry_run", false ) )
            return KOPENAPI_RESULT::Error( 409, "no board yet at " + boardPath + ": run without dry_run to create it" );

        if( !exists && !aArgs.value( "create", true ) )
            return KOPENAPI_RESULT::Error( 404, "no board open and none at " + boardPath + " (create: true to make it)" );

        KOPENAPI_RESULT open = exists ? callMethod( aCtx, "pcb_open", { { "path", boardPath } } )
                                      : callMethod( aCtx, "pcb_new", { { "path", boardPath }, { "layers", aArgs.value( "layers", 2 ) } } );

        if( open.status != 200 )
            return open;

        opened = { { "path", boardPath }, { "created", !exists } };
    }

    // 3. KiCad's Update PCB from Schematic
    nlohmann::json apply = { { "netlist", sch.body["netlist"] } };

    for( const char* key : { "dry_run", "match", "delete_extra", "replace_footprints", "spread", "spread_width_mm" } )
    {
        if( aArgs.contains( key ) )
            apply[key] = aArgs[key];
    }

    KOPENAPI_RESULT result = callMethod( aCtx, "pcb_netlist_apply", apply );

    if( result.status == 200 )
    {
        if( !opened.is_null() )
            result.body["board"] = opened;

        result.body["next"] = "pcb_rules_set (net classes, clearances), pcb_outline_set, then placement; pcb_save";
    }

    return result;
}


KOPENAPI_REGISTER( "design_update_board",
                   "Schematic -> board in one call (KiCad's Update PCB from Schematic): takes the open "
                   "schematic's netlist (unsaved edits included), opens the project's board or creates it "
                   "(layers), adds / updates / removes footprints and nets, lays new footprints out beside "
                   "the board; report per change; dry_run to preview",
                   R"json({"type":"object","properties":{
                        "board_path":{"type":"string","description":"default: the schematic's project, .kicad_pcb"},
                        "create":{"type":"boolean","default":true},
                        "layers":{"type":"integer","default":2,"description":"copper layers of a new board"},
                        "dry_run":{"type":"boolean","default":false},
                        "match":{"type":"string","enum":["uuid","reference"],"default":"uuid"},
                        "delete_extra":{"type":"boolean","default":true},
                        "replace_footprints":{"type":"boolean","default":true}}})json"_json,
                   false, h_design_update_board, 600 );

KOPENAPI_REGISTER( "design_parity",
                   "Compare schematic and board (both open): components (missing on either side, value / "
                   "footprint differences) and nets by connectivity through pads — identical, renamed, split "
                   "or merged on the board, pads only on one side, unconnected pins; consistent flag",
                   R"json({"type":"object","properties":{
                        "list_limit":{"type":"integer","default":50,"description":"max listed items per finding"}}})json"_json,
                   false, h_design_parity, 120 );

KOPENAPI_REGISTER( "net_get",
                   "One net from both sides: the schematic net card and the board net card(s) holding its "
                   "pads, matched through pads (names may differ), with link status identical / renamed / "
                   "split_on_board / pads_missing_on_board / schematic_only / board_only; works with one "
                   "document open too",
                   R"json({"type":"object","properties":{
                        "name":{"type":"string","description":"net name on either side"},
                        "pin":{"type":"string","description":"REF.PIN"}}})json"_json,
                   false, h_net_get );

KOPENAPI_MARK_EDITING( "design_update_board" );
