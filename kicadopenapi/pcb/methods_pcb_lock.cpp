/// @file methods_pcb_lock.cpp
/// @brief pcb_lock / pcb_unlock: KiCad's lock flag on footprints, tracks, vias, zones and drawings, so that
/// placement, routing and deletion through the API (and the editor) leave them alone.

#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <frame_type.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <pcb_track.h>
#include <string_utils.h>
#include <tool/tool_manager.h>
#include <zone.h>

#include <map>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


const char* kindOf( const BOARD_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case PCB_FOOTPRINT_T: return "footprints";
    case PCB_VIA_T:       return "vias";
    case PCB_TRACE_T:
    case PCB_ARC_T:       return "tracks";
    case PCB_ZONE_T:      return "zones";
    default:              return "drawings";
    }
}


KOPENAPI_RESULT setLocked( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, bool aLock )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                 board = context->GetBoard();
    std::set<BOARD_ITEM*>  items;
    nlohmann::json         missing = nlohmann::json::array();
    bool                   selector = false;

    // footprints by reference
    if( aArgs.contains( "refs" ) )
    {
        if( !aArgs["refs"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "refs: footprint references" );

        selector = true;

        for( const nlohmann::json& ref : aArgs["refs"] )
        {
            bool found = false;

            for( FOOTPRINT* fp : board->Footprints() )
            {
                if( ref.is_string() && str( fp->GetReference() ) == ref.get<std::string>() )
                {
                    items.insert( fp );
                    found = true;
                }
            }

            if( !found )
                missing.push_back( ref );
        }
    }

    // any top-level item by uuid: footprint, track, via, zone, drawing
    if( aArgs.contains( "uuids" ) )
    {
        if( !aArgs["uuids"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "uuids: item uuids" );

        selector = true;

        std::vector<BOARD_ITEM*> topLevel( board->Footprints().begin(), board->Footprints().end() );
        topLevel.insert( topLevel.end(), board->Tracks().begin(), board->Tracks().end() );
        topLevel.insert( topLevel.end(), board->Zones().begin(), board->Zones().end() );
        topLevel.insert( topLevel.end(), board->Drawings().begin(), board->Drawings().end() );

        for( const nlohmann::json& u : aArgs["uuids"] )
        {
            std::string error;
            BOARD_ITEM* hit = u.is_string() ? KopenapiMatchUuid( topLevel, u.get<std::string>(), error ) : nullptr;

            if( !hit && error.find( "ambiguous" ) != std::string::npos )
                return KOPENAPI_RESULT::Error( 400, "uuids: " + error );

            if( !hit )
                missing.push_back( u );
            else
                items.insert( hit );
        }
    }

    // tracks and vias by the filters of pcb_tracks_delete
    const bool trackFilters = aArgs.contains( "net" ) || aArgs.contains( "nets" ) || aArgs.contains( "between" )
                              || aArgs.contains( "region" );

    if( trackFilters )
    {
        selector = true;

        nlohmann::json filters = aArgs;
        filters.erase( "uuids" );

        nlohmann::json                 unused = nlohmann::json::array();
        std::optional<KOPENAPI_RESULT> error;

        for( PCB_TRACK* track : KopenapiSelectTracks( board, filters, true, unused, error ) )
            items.insert( track );

        if( error )
            return *error;
    }

    // footprints lying wholly in the region
    if( aArgs.value( "region_footprints", false ) )
    {
        const nlohmann::json r = aArgs.value( "region", nlohmann::json() );

        if( !r.is_array() || r.size() != 4 )
            return KOPENAPI_RESULT::Error( 400, "region_footprints needs region: [x0_mm, y0_mm, x1_mm, y1_mm]" );

        BOX2I box( VECTOR2I( pcbIUScale.mmToIU( r[0].get<double>() ), pcbIUScale.mmToIU( r[1].get<double>() ) ), VECTOR2I( 0, 0 ) );
        box.SetEnd( VECTOR2I( pcbIUScale.mmToIU( r[2].get<double>() ), pcbIUScale.mmToIU( r[3].get<double>() ) ) );
        box.Normalize();

        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( box.Contains( KopenapiBoardItemBox( fp ) ) )
                items.insert( fp );
        }
    }

    if( !selector )
    {
        return KOPENAPI_RESULT::Error( 400, "say what: refs (footprints), uuids (footprints, tracks, vias, zones, drawings), "
                                            "net / nets, between [\"REF.PAD\", \"REF.PAD\"] or region (tracks and vias; "
                                            "region_footprints: true adds the footprints inside)" );
    }

    std::map<std::string, int> changed, already;
    BOARD_COMMIT               commit( context->GetToolManager() );
    const bool                 dryRun = aArgs.value( "dry_run", false );

    for( BOARD_ITEM* item : items )
    {
        if( item->IsLocked() == aLock )
        {
            already[kindOf( item )]++;
            continue;
        }

        changed[kindOf( item )]++;

        if( !dryRun )
        {
            commit.Modify( item );
            item->SetLocked( aLock );
        }
    }

    int total = 0;

    for( const auto& [kind, count] : changed )
        total += count;

    if( total && !dryRun )
    {
        commit.Push( aLock ? _( "Lock (API)" ) : _( "Unlock (API)" ) );

        if( !aCtx.headless && aCtx.kiway )
        {
            if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
                frame->GetCanvas()->Refresh();
        }
    }

    return KOPENAPI_RESULT::Ok( { { aLock ? "locked" : "unlocked", changed }, { "changed", total }, { "already", already },
                                  { "missing", missing }, { "applied", total > 0 && !dryRun } } );
}


KOPENAPI_RESULT h_pcb_lock( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return setLocked( aCtx, aArgs, true );
}


KOPENAPI_RESULT h_pcb_unlock( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return setLocked( aCtx, aArgs, false );
}


KOPENAPI_RESULT h_pcb_lock_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    nlohmann::json footprints = nlohmann::json::array(), zones = nlohmann::json::array(), drawings = nlohmann::json::array();
    std::map<std::string, nlohmann::json> nets;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( fp->IsLocked() )
            footprints.push_back( str( fp->GetReference() ) );
    }

    for( PCB_TRACK* track : board->Tracks() )
    {
        if( !track->IsLocked() )
            continue;

        nlohmann::json& net = nets[str( UnescapeString( track->GetNetname() ) )];

        if( net.is_null() )
            net = { { "tracks", 0 }, { "vias", 0 } };

        const char* kind = track->Type() == PCB_VIA_T ? "vias" : "tracks";
        net[kind] = net[kind].get<int>() + 1;
    }

    for( ZONE* zone : board->Zones() )
    {
        if( zone->IsLocked() )
            zones.push_back( { { "uuid", str( zone->m_Uuid.AsString() ) }, { "name", str( zone->GetZoneName() ) },
                               { "net", str( UnescapeString( zone->GetNetname() ) ) } } );
    }

    for( BOARD_ITEM* drawing : board->Drawings() )
    {
        if( drawing->IsLocked() )
            drawings.push_back( str( drawing->m_Uuid.AsString() ) );
    }

    return KOPENAPI_RESULT::Ok( { { "footprints", footprints }, { "copper_by_net", nets }, { "zones", zones },
                                  { "drawings", drawings } } );
}

} // namespace


#define LOCK_SCHEMA R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"},"description":"footprints by reference"},
                        "uuids":{"type":"array","items":{"type":"string"},"description":"footprints, tracks, vias, zones, drawings"},
                        "net":{"type":"string"},"nets":{"type":"array","items":{"type":"string"},"description":"tracks and vias of these nets"},
                        "between":{"type":"array","items":{"type":"string"},"description":"[\"REF.PAD\", \"REF.PAD\"]: the copper joining two pads"},
                        "region":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm]: tracks and vias in it"},
                        "region_mode":{"type":"string","enum":["inside","touching"],"default":"inside"},
                        "region_footprints":{"type":"boolean","default":false,"description":"also the footprints lying wholly in the region"},
                        "layer":{"type":"string"},
                        "kinds":{"type":"array","items":{"type":"string","enum":["track","via"]}},
                        "dry_run":{"type":"boolean","default":false}}})json"_json

KOPENAPI_REGISTER( "pcb_lock",
                   "Lock items so that placement, routing and deletion leave them alone (KiCad's lock flag, "
                   "saved with the board): footprints (refs), any item by uuid, tracks and vias by net, "
                   "between two pads or in a region (the selectors of pcb_tracks_delete). Locked footprints "
                   "are not moved by pcb_footprint_move / pcb_place_group / pcb_place_auto, locked copper is "
                   "not deleted by pcb_tracks_delete nor shoved by the router, locked drawings are not moved "
                   "or deleted - each unless override_locks. One undo step; answers what changed per kind",
                   LOCK_SCHEMA, false, h_pcb_lock );

KOPENAPI_REGISTER( "pcb_unlock", "Unlock items (the selectors of pcb_lock); one undo step; answers what changed per kind",
                   LOCK_SCHEMA, false, h_pcb_unlock );

KOPENAPI_REGISTER( "pcb_lock_list",
                   "What is locked on the board: footprint references, locked tracks / vias per net, zones, "
                   "drawings (with other conditions: pcb_query where \"A.Locked\")",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_lock_list );

KOPENAPI_MARK_EDITING( "pcb_lock", "pcb_unlock" );
KOPENAPI_MARK_CONCURRENT_READS( "pcb_lock_list" );
