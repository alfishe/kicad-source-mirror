/// @file methods_pcb_layer_tools.cpp
/// @brief kicadopenapi board-wide layer edits: the editor's Swap Layers and Remove Unused Pads
/// (unconnected pad / via layers removed or restored)
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <pad.h>
#include <padstack.h>
#include <pcb_generator.h>
#include <pcb_track.h>
#include <tool/tool_manager.h>
#include <zone.h>
#include <kicadopenapi_util.h>

#include <map>
#include <set>
#include <vector>


std::string KopenapiUnusedLayersMode( const PADSTACK& aPadstack )
{
    switch( aPadstack.UnconnectedLayerMode() )
    {
    case UNCONNECTED_LAYER_MODE::KEEP_ALL:                    return "keep";
    case UNCONNECTED_LAYER_MODE::START_END_ONLY:              return "start_end_only";
    case UNCONNECTED_LAYER_MODE::REMOVE_ALL:                  return "remove";
    case UNCONNECTED_LAYER_MODE::REMOVE_EXCEPT_START_AND_END: return "keep_outer";
    }

    return "keep";
}


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


/// @brief An enabled board layer by name; UNDEFINED_LAYER if none
PCB_LAYER_ID enabledLayer( BOARD* aBoard, const std::string& aName )
{
    const int id = aBoard->GetLayerID( wxString::FromUTF8( aName ) );

    if( id < 0 || id >= PCB_LAYER_ID_COUNT || !aBoard->IsLayerEnabled( (PCB_LAYER_ID) id ) )
        return UNDEFINED_LAYER;

    return (PCB_LAYER_ID) id;
}


std::string itemKind( const BOARD_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case PCB_TRACE_T:     return "tracks";
    case PCB_ARC_T:       return "tracks";
    case PCB_VIA_T:       return "vias";
    case PCB_ZONE_T:      return "zones";
    case PCB_GENERATOR_T: return "tuning_patterns";
    default:              return "graphics";
    }
}

} // namespace


/// Swap Layers (GLOBAL_EDIT_TOOL::SwapLayers): tracks (not those of a tuning pattern), blind / buried
/// vias, tuning patterns, zones and board graphics move by the map; footprints and through vias stay
static KOPENAPI_RESULT h_pcb_layers_swap( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( !aArgs.contains( "map" ) || !aArgs["map"].is_object() || aArgs["map"].empty() )
        return KOPENAPI_RESULT::Error( 400, "map: {\"from layer\": \"to layer\", ...}, e.g. {\"F.Cu\": \"B.Cu\", \"B.Cu\": \"F.Cu\"}" );

    std::map<PCB_LAYER_ID, PCB_LAYER_ID> layerMap;

    for( const auto& [from, to] : aArgs["map"].items() )
    {
        const PCB_LAYER_ID src = enabledLayer( board, from );
        const PCB_LAYER_ID dst = to.is_string() ? enabledLayer( board, to.get<std::string>() ) : UNDEFINED_LAYER;

        if( src == UNDEFINED_LAYER )
            return KOPENAPI_RESULT::Error( 400, "map: '" + from + "' is not a layer of this board (pcb_layer_list)" );

        if( dst == UNDEFINED_LAYER )
            return KOPENAPI_RESULT::Error( 400, "map: '" + from + "' -> " + to.dump() + ": not a layer of this board (pcb_layer_list)" );

        if( IsCopperLayer( src ) != IsCopperLayer( dst ) )
            return KOPENAPI_RESULT::Error( 400, "map: '" + from + "' -> '" + to.get<std::string>()
                                                + "': copper maps to copper, other layers to other layers" );

        if( src != dst )
            layerMap[src] = dst;
    }

    auto mapped = [&]( PCB_LAYER_ID aLayer )
    {
        auto it = layerMap.find( aLayer );
        return it == layerMap.end() ? aLayer : it->second;
    };

    struct CHANGE
    {
        BOARD_ITEM*  item;
        LSET         layers;
        PCB_LAYER_ID top = UNDEFINED_LAYER;
        PCB_LAYER_ID bottom = UNDEFINED_LAYER;
    };

    std::vector<CHANGE> changes;

    auto consider = [&]( BOARD_ITEM* aItem )
    {
        LSET newLayers;

        for( PCB_LAYER_ID layer : aItem->GetLayerSet() )
            newLayers.set( mapped( layer ) );

        if( newLayers != aItem->GetLayerSet() )
            changes.push_back( { aItem, newLayers } );
    };

    for( PCB_TRACK* track : board->Tracks() )
    {
        if( dynamic_cast<PCB_GENERATOR*>( track->GetParentGroup() ) )
            continue;

        if( track->Type() == PCB_VIA_T )
        {
            PCB_VIA* via = static_cast<PCB_VIA*>( track );

            if( via->GetViaType() == VIATYPE::THROUGH )
                continue;

            PCB_LAYER_ID top, bottom;
            via->LayerPair( &top, &bottom );

            if( mapped( top ) != top || mapped( bottom ) != bottom )
                changes.push_back( { via, LSET(), mapped( top ), mapped( bottom ) } );
        }
        else
        {
            consider( track );
        }
    }

    for( PCB_GENERATOR* generator : board->Generators() )
        consider( generator );

    for( ZONE* zone : board->Zones() )
        consider( zone );

    for( BOARD_ITEM* drawing : board->Drawings() )
        consider( drawing );

    const bool dryRun = aArgs.value( "dry_run", false );

    if( !aArgs.value( "override_locks", false ) )
    {
        nlohmann::json locked = nlohmann::json::array();

        for( const CHANGE& change : changes )
        {
            if( change.item->IsLocked() && locked.size() < 20 )
                locked.push_back( str( change.item->m_Uuid.AsString() ) );
        }

        if( !locked.empty() )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "locked items would move (pcb_unlock, or override_locks)" );
            r.body["locked"] = locked;
            return r;
        }
    }

    std::map<std::string, int> counts = { { "tracks", 0 }, { "vias", 0 }, { "zones", 0 }, { "graphics", 0 },
                                          { "tuning_patterns", 0 } };

    for( const CHANGE& change : changes )
        counts[itemKind( change.item )]++;

    if( !dryRun && !changes.empty() )
    {
        BOARD_COMMIT commit( context->GetToolManager() );

        for( const CHANGE& change : changes )
        {
            if( change.item->Type() == PCB_VIA_T )
            {
                commit.Modify( change.item );
                static_cast<PCB_VIA*>( change.item )->SetLayerPair( change.top, change.bottom );
            }
            else
            {
                commit.Modify( change.item, nullptr, RECURSE_MODE::RECURSE );
                change.item->SetLayerSet( change.layers );
            }
        }

        commit.Push( _( "Swap Layers (API)" ) );
    }

    nlohmann::json applied = nlohmann::json::object();

    for( const auto& [src, dst] : layerMap )
        applied[str( board->GetLayerName( src ) )] = str( board->GetLayerName( dst ) );

    nlohmann::json answer = { { "map", applied }, { "changed", changes.size() }, { "dry_run", dryRun } };

    for( const auto& [kind, count] : counts )
        answer[kind] = count;

    if( !changes.empty() && !dryRun )
    {
        bool zones = false;

        for( const CHANGE& change : changes )
            zones |= change.item->Type() == PCB_ZONE_T;

        if( zones )
            answer["next"] = "zones moved: refill them (pcb_zones_fill)";
    }

    return KOPENAPI_RESULT::Ok( answer );
}


/// Remove Unused Pads (DIALOG_UNUSED_PAD_LAYERS): unconnected copper layers of through-hole pads and
/// of vias removed (or kept on the outer / start-end layers), or restored
static KOPENAPI_RESULT h_pcb_pads_unused_layers( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string mode = aArgs.value( "mode", std::string( "remove" ) );

    static const std::map<std::string, UNCONNECTED_LAYER_MODE> modes = {
        { "remove", UNCONNECTED_LAYER_MODE::REMOVE_ALL },
        { "keep_outer", UNCONNECTED_LAYER_MODE::REMOVE_EXCEPT_START_AND_END },
        { "keep", UNCONNECTED_LAYER_MODE::KEEP_ALL } };

    if( !modes.count( mode ) )
        return KOPENAPI_RESULT::Error( 400, "mode: remove (every unconnected layer), keep_outer (keep the outer / start and end "
                                            "layers) or keep (restore all layers)" );

    const UNCONNECTED_LAYER_MODE target = modes.at( mode );
    const bool                   withPads = aArgs.value( "pads", true );
    const bool                   withVias = aArgs.value( "vias", true );

    auto viaQualifies = [&]( PCB_VIA* aVia )
    {
        if( aVia->GetViaType() == VIATYPE::THROUGH )
            return board->GetCopperLayerCount() > 2;

        PCB_LAYER_ID start = aVia->Padstack().StartLayer();
        PCB_LAYER_ID end = aVia->Padstack().EndLayer();

        if( start < 0 || end < 0 )
            return board->GetCopperLayerCount() > 2;

        return board->LayerDepth( start, end ) > 1;
    };

    auto padQualifies = [&]( PAD* aPad )
    {
        return aPad->GetAttribute() == PAD_ATTRIB::PTH;
    };

    std::vector<PAD*>     pads;
    std::vector<PCB_VIA*> vias;
    std::set<BOARD_ITEM*> seen;

    auto addPad = [&]( PAD* aPad )
    {
        if( withPads && padQualifies( aPad ) && seen.insert( aPad ).second )
            pads.push_back( aPad );
    };

    auto addVia = [&]( PCB_VIA* aVia )
    {
        if( withVias && viaQualifies( aVia ) && seen.insert( aVia ).second )
            vias.push_back( aVia );
    };

    if( aArgs.contains( "items" ) )
    {
        if( !aArgs["items"].is_array() || aArgs["items"].empty() )
            return KOPENAPI_RESULT::Error( 400, "items: footprint references, pads (REF.PAD) or via uuids" );

        for( const nlohmann::json& id : aArgs["items"] )
        {
            BOARD_ITEM* item = id.is_string() ? KopenapiFindBoardItem( board, id.get<std::string>() ) : nullptr;

            if( !item )
                return KOPENAPI_RESULT::Error( 404, "items: no footprint, pad or via " + id.dump() );

            if( item->Type() == PCB_FOOTPRINT_T )
            {
                for( PAD* pad : static_cast<FOOTPRINT*>( item )->Pads() )
                    addPad( pad );
            }
            else if( item->Type() == PCB_PAD_T )
            {
                addPad( static_cast<PAD*>( item ) );
            }
            else if( item->Type() == PCB_VIA_T )
            {
                addVia( static_cast<PCB_VIA*>( item ) );
            }
            else
            {
                return KOPENAPI_RESULT::Error( 400, "items: " + id.dump() + " is not a footprint, pad or via" );
            }
        }
    }
    else
    {
        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
                addPad( pad );
        }

        for( PCB_TRACK* track : board->Tracks() )
        {
            if( track->Type() == PCB_VIA_T )
                addVia( static_cast<PCB_VIA*>( track ) );
        }
    }

    std::erase_if( pads, [&]( PAD* p ) { return p->Padstack().UnconnectedLayerMode() == target; } );
    std::erase_if( vias, [&]( PCB_VIA* v ) { return v->Padstack().UnconnectedLayerMode() == target; } );

    const bool dryRun = aArgs.value( "dry_run", false );

    if( !dryRun && ( !pads.empty() || !vias.empty() ) )
    {
        BOARD_COMMIT          commit( context->GetToolManager() );
        std::set<FOOTPRINT*>  modified;

        for( PAD* pad : pads )
        {
            if( FOOTPRINT* fp = pad->GetParentFootprint(); fp && modified.insert( fp ).second )
                commit.Modify( fp );

            pad->SetUnconnectedLayerMode( target );
        }

        for( PCB_VIA* via : vias )
        {
            commit.Modify( via );
            via->Padstack().SetUnconnectedLayerMode( target );
        }

        commit.Push( _( "Remove Unused Pads (API)" ) );
    }

    nlohmann::json items = nlohmann::json::array();

    for( PAD* pad : pads )
    {
        items.push_back( { { "pad", str( pad->GetParentFootprint()->GetReference() ) + "." + str( pad->GetNumber() ) },
                           { "uuid", str( pad->m_Uuid.AsString() ) } } );
    }

    for( PCB_VIA* via : vias )
    {
        items.push_back( { { "via", str( via->m_Uuid.AsString() ) },
                           { "at_mm", { pcbIUScale.IUTomm( via->GetPosition().x ), pcbIUScale.IUTomm( via->GetPosition().y ) } } } );
    }

    nlohmann::json answer = { { "mode", mode }, { "pads", pads.size() }, { "vias", vias.size() }, { "dry_run", dryRun } };
    KOPENAPI_LIST_CAP( aArgs, 50 ).Put( answer, "items", items );

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_layers_swap",
                   "Swap Layers: move board items from layer to layer by a map ({\"F.Cu\": \"B.Cu\", \"B.Cu\": \"F.Cu\"}; "
                   "copper to copper, other layers to other layers; a layer not in the map stays): tracks, blind / "
                   "buried vias (their span), tuning patterns, zones and board graphics; footprints and through vias "
                   "stay. Locked items refused (409, listed) unless override_locks; dry_run counts only; one undo step; "
                   "answers the counts by kind",
                   R"json({"type":"object","required":["map"],"properties":{
                        "map":{"type":"object","additionalProperties":{"type":"string"},"description":"from layer -> to layer"},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_layers_swap );

KOPENAPI_REGISTER( "pcb_pads_unused_layers",
                   "Remove Unused Pads: the copper of through-hole pads and of vias on layers where nothing connects "
                   "(mode remove), the same but kept on the outer / start and end layers (keep_outer), or restored "
                   "(keep). All pads and vias, or items (footprint refs, REF.PAD, via uuids); pads / vias false to "
                   "leave that kind; through vias count only on boards with more than 2 copper layers. Answers how many "
                   "changed and which (items, first list_limit); dry_run counts only; one undo step. The mode of each: "
                   "unused_layers in pcb_via_list and the pad lists",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "mode":{"type":"string","enum":["remove","keep_outer","keep"],"default":"remove"},
                        "items":{"type":"array","items":{"type":"string"}},
                        "pads":{"type":"boolean","default":true},
                        "vias":{"type":"boolean","default":true},
                        "dry_run":{"type":"boolean","default":false}}})json"_json, 50 ),
                   false, h_pcb_pads_unused_layers );

KOPENAPI_MARK_EDITING( "pcb_layers_swap", "pcb_pads_unused_layers" );
