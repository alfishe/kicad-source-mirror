/// @file methods_pcb_swap_nets.cpp
/// @brief kicadopenapi Swap Pad Nets / Swap Gate Nets: pin and gate swaps on the board with the copper
/// following, as the editor's pad context menu; the schematic follows through Update Schematic from PCB
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>
#include <net_swap.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <tool/tool_manager.h>

#include <algorithm>
#include <map>
#include <optional>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


std::string padName( const PAD* aPad )
{
    const FOOTPRINT* fp = aPad->GetParentFootprint();
    return ( fp ? str( fp->GetReference() ) : std::string( "?" ) ) + "." + str( aPad->GetNumber() );
}


std::string netName( BOARD* aBoard, int aNetCode )
{
    NETINFO_ITEM* net = aBoard->FindNet( aNetCode );
    return net ? str( net->GetNetname() ) : std::string();
}


std::string itemName( BOARD_CONNECTED_ITEM* aItem )
{
    if( aItem->Type() == PCB_PAD_T )
        return padName( static_cast<PAD*>( aItem ) );

    return str( aItem->m_Uuid.AsString() );
}


const char* itemKind( KICAD_T aType )
{
    switch( aType )
    {
    case PCB_PAD_T: return "pads";
    case PCB_TRACE_T: return "tracks";
    case PCB_ARC_T: return "arcs";
    case PCB_VIA_T: return "vias";
    default: return "shapes";
    }
}


nlohmann::json unitsJson( const FOOTPRINT* aFootprint )
{
    nlohmann::json units = nlohmann::json::array();

    for( const FOOTPRINT::FP_UNIT_INFO& unit : aFootprint->GetUnitInfo() )
    {
        nlohmann::json pins = nlohmann::json::array();

        for( const wxString& pin : unit.m_pins )
            pins.push_back( str( pin ) );

        units.push_back( { { "name", str( unit.m_unitName ) }, { "pins", pins } } );
    }

    return units;
}


KOPENAPI_RESULT h_pcb_swap_nets( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string mode = aArgs.value( "mode", std::string( "pads" ) );

    if( mode != "pads" && mode != "gates" )
        return KOPENAPI_RESULT::Error( 400, "mode: pads (Swap Pad Nets) or gates (Swap Gate Nets)" );

    if( !aArgs.contains( "pads" ) || !aArgs["pads"].is_array() || aArgs["pads"].empty() )
        return KOPENAPI_RESULT::Error( 400, "pads: [\"REF.PAD\", ...] in swap order" );

    std::vector<PAD*> pads;

    for( const nlohmann::json& p : aArgs["pads"] )
    {
        if( !p.is_string() )
            return KOPENAPI_RESULT::Error( 400, "pads: strings REF.PAD" );

        BOARD_ITEM* item = KopenapiFindBoardItem( board, p.get<std::string>() );

        if( !item || item->Type() != PCB_PAD_T )
            return KOPENAPI_RESULT::Error( 404, "no pad " + p.get<std::string>() + " (REF.PAD)" );

        PAD* pad = static_cast<PAD*>( item );

        if( std::find( pads.begin(), pads.end(), pad ) != pads.end() )
            return KOPENAPI_RESULT::Error( 400, "pad " + p.get<std::string>() + " given twice" );

        pads.push_back( pad );
    }

    NET_SWAP_PLAN  plan;
    bool           planned = false;
    const wxString targetUnit = wxString::FromUTF8( aArgs.value( "target_unit", std::string() ) );

    if( mode == "pads" )
    {
        if( pads.size() < 2 )
            return KOPENAPI_RESULT::Error( 400, "pads: at least two pads to swap" );

        planned = PlanPadNetSwap( board, pads, plan );

        if( !planned && plan.Error.IsEmpty() )
            return KOPENAPI_RESULT::Error( 422, "nothing to swap: the pads are all on one net" );
    }
    else
    {
        planned = PlanGateNetSwap( board, pads, targetUnit, plan );
    }

    if( !planned )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 422, str( plan.Error ) );

        if( mode == "gates" && pads.front()->GetParentFootprint() )
            r.body["units"] = unitsJson( pads.front()->GetParentFootprint() );

        return r;
    }

    nlohmann::json connected = nlohmann::json::array();

    for( PAD* pad : plan.ConnectedPads )
        connected.push_back( padName( pad ) );

    const std::string choice = aArgs.value( "connected_pads", std::string() );
    const bool        dryRun = aArgs.value( "dry_run", false );

    if( !choice.empty() && choice != "include" && choice != "ignore" )
        return KOPENAPI_RESULT::Error( 400, "connected_pads: include (Swap All Connected Pads) or ignore (Ignore "
                                            "Unselected Pads)" );

    if( choice.empty() && !connected.empty() && !dryRun )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, std::to_string( connected.size() )
                                                                 + " unselected pad(s) are connected to these nets: "
                                                                   "connected_pads include or ignore (tracks, vias "
                                                                   "and other copper swap either way)" );
        r.body["connected_pads"] = connected;
        return r;
    }

    const bool includeConnected = choice != "ignore";

    nlohmann::json locked = nlohmann::json::array();

    for( BOARD_CONNECTED_ITEM* item : plan.LockedItems( includeConnected ) )
        locked.push_back( itemName( item ) );

    if( !locked.empty() && !aArgs.value( "override_locks", false ) && !dryRun )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "cannot swap nets of locked pads or connected copper "
                                                         "without override_locks" );
        r.body["locked"] = locked;
        return r;
    }

    nlohmann::json padChanges = nlohmann::json::array();

    for( const auto& [pad, newNet] : plan.PadNewNets )
    {
        padChanges.push_back( { { "pad", padName( pad ) },
                                { "from", netName( board, pad->GetNetCode() ) },
                                { "to", netName( board, newNet ) } } );
    }

    std::map<std::string, int> itemCounts;

    for( const auto& [item, newNet] : plan.ItemNewNets )
    {
        if( item->GetNetCode() == newNet || ( item->Type() == PCB_PAD_T ) )
            continue;

        itemCounts[itemKind( item->Type() )]++;
    }

    int connectedPadsChanged = 0;

    for( PAD* pad : plan.ConnectedPads )
    {
        if( includeConnected && pad->GetNetCode() != plan.ItemNewNets.at( pad ) )
            connectedPadsChanged++;
    }

    nlohmann::json out = { { "mode", mode },
                           { "pads", padChanges },
                           { "copper", itemCounts },
                           { "connected_pads", connected },
                           { "connected_pads_changed", connectedPadsChanged },
                           { "dry_run", dryRun } };

    if( !locked.empty() )
        out["locked"] = locked;

    if( mode == "gates" )
    {
        nlohmann::json units = nlohmann::json::array();
        const auto&    info = pads.front()->GetParentFootprint()->GetUnitInfo();

        for( int idx : plan.Units )
            units.push_back( str( info[idx].m_unitName ) );

        out["units"] = units;
    }

    if( dryRun )
        return KOPENAPI_RESULT::Ok( out );

    BOARD_COMMIT commit( context->GetToolManager() );
    plan.Apply( commit, includeConnected );
    commit.Push( mode == "pads" ? _( "Swap Pad Nets" ) : _( "Swap Gate Nets" ) );
    board->BuildConnectivity();
    context->SetContentModified();

    // the schematic follows (Update Schematic from PCB: pin / unit swaps, else label renames)
    const nlohmann::json backAnnotate = aArgs.value( "back_annotate", nlohmann::json( "auto" ) );

    if( backAnnotate != false )
    {
        std::optional<KOPENAPI_METHOD> update = KOPENAPI_REGISTRY::Get().Find( "design_update_schematic" );
        KOPENAPI_RESULT                sch = update ? update->handler( aCtx, { { "references", false },
                                                                              { "footprints", false },
                                                                              { "values", false },
                                                                              { "net_names", true },
                                                                              { "attributes", false },
                                                                              { "other_fields", false } } )
                                                    : KOPENAPI_RESULT::Error( 503, "design_update_schematic is not available" );

        if( sch.status == 200 )
        {
            out["schematic"] = { { "changes", sch.body.value( "changes", 0 ) },
                                 { "errors", sch.body.value( "errors", 0 ) },
                                 { "messages", sch.body["messages"] } };
        }
        else
        {
            out["schematic"] = { { "skipped", sch.body.value( "error", nlohmann::json::object() )
                                                       .value( "message", std::string( "no schematic open" ) ) } };
        }
    }

    if( !out.contains( "schematic" ) )
        out["next"] = "the schematic keeps the old pins: design_update_schematic (KiCad's Update Schematic from PCB)";
    else if( out["schematic"].contains( "skipped" ) )
        out["next"] = "the schematic keeps the old pins: open it (sch_open), then design_update_schematic";

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_swap_nets",
                   "Swap Pad Nets (mode pads) / Swap Gate Nets (mode gates) as the editor's pad menu: pads "
                   "[\"REF.PAD\", ...] in order, each takes the net of the next (the last the first's); gates: "
                   "the units of one multi-unit footprint the pads touch rotate their nets pin by pin, or one "
                   "unit with target_unit (a unit name with as many pins) swaps the two. Tracks, arcs, vias and "
                   "shapes connected on the old net follow (zones stay); other pads the nets reach need "
                   "connected_pads include / ignore (409 lists them, the editor's question). Locked pads or "
                   "copper refuse (409) unless override_locks. One undo step; back_annotate (default auto: a "
                   "schematic is open) then runs Update Schematic from PCB with net names (pin / unit swaps "
                   "preferred, else label renames). dry_run previews. Errors answer the footprint's units",
                   R"json({"type":"object","required":["pads"],"properties":{
                        "pads":{"type":"array","items":{"type":"string"},"description":"REF.PAD in swap order"},
                        "mode":{"type":"string","enum":["pads","gates"],"default":"pads"},
                        "target_unit":{"type":"string","description":"gates with pads of one unit: the unit to swap with"},
                        "connected_pads":{"type":"string","enum":["include","ignore"]},
                        "override_locks":{"type":"boolean","default":false},
                        "back_annotate":{"oneOf":[{"type":"boolean"},{"type":"string","enum":["auto"]}],"default":"auto"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_swap_nets );

KOPENAPI_MARK_EDITING( "pcb_swap_nets" );
