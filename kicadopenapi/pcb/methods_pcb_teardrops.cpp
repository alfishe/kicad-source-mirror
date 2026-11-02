/// @file methods_pcb_teardrops.cpp
/// @brief kicadopenapi pcb_teardrops: the editor's Edit > Edit Teardrops (add / remove / set on vias,
/// pads and track ends with its filters) and Board Setup > Teardrops (the defaults per shape).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_util.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_track.h>
#include <project/net_settings.h>
#include <string_utils.h>
#include <teardrop/teardrop.h>
#include <teardrop/teardrop_parameters.h>
#include <tool/tool_manager.h>
#include <zone.h>
#include <zone_filler.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


double round3( double aValue )
{
    return std::round( aValue * 1000.0 ) / 1000.0;
}


nlohmann::json paramsJson( const TEARDROP_PARAMETERS& aParams )
{
    return { { "enabled", aParams.m_Enabled },
             { "max_length_mm", toMm( aParams.m_TdMaxLen ) },
             { "max_width_mm", toMm( aParams.m_TdMaxWidth ) },
             { "length_ratio", round3( aParams.m_BestLengthRatio ) },
             { "width_ratio", round3( aParams.m_BestWidthRatio ) },
             { "size_filter_ratio", round3( aParams.m_WidthtoSizeFilterRatio ) },
             { "curved", aParams.m_CurvedEdges },
             { "two_tracks", aParams.m_AllowUseTwoTracks },
             { "pads_in_zones", aParams.m_TdOnPadsInZones } };
}


/// @brief aJson's keys onto aParams; an error message for an unknown key or a value out of range
std::string applyParams( const nlohmann::json& aJson, TEARDROP_PARAMETERS& aParams, const std::string& aWhere )
{
    if( !aJson.is_object() )
        return aWhere + ": an object of " + paramsJson( aParams ).dump();

    for( const auto& [key, value] : aJson.items() )
    {
        const std::string at = aWhere + "." + key;

        if( key == "enabled" || key == "curved" || key == "two_tracks" || key == "pads_in_zones" )
        {
            if( !value.is_boolean() )
                return at + ": true or false";

            bool& flag = key == "enabled"      ? aParams.m_Enabled
                         : key == "curved"     ? aParams.m_CurvedEdges
                         : key == "two_tracks" ? aParams.m_AllowUseTwoTracks
                                               : aParams.m_TdOnPadsInZones;
            flag = value.get<bool>();
            continue;
        }

        if( !value.is_number() )
            return at + ": a number";

        const double v = value.get<double>();

        if( key == "max_length_mm" || key == "max_width_mm" )
        {
            if( v < 0 || v > 100 )
                return at + ": 0 .. 100 mm (0: no limit)";

            ( key == "max_length_mm" ? aParams.m_TdMaxLen : aParams.m_TdMaxWidth ) = pcbIUScale.mmToIU( v );
        }
        else if( key == "length_ratio" || key == "width_ratio" )
        {
            if( v <= 0 || v > 10 )
                return at + ": a ratio of the pad / via size, above 0 up to 10";

            ( key == "length_ratio" ? aParams.m_BestLengthRatio : aParams.m_BestWidthRatio ) = v;
        }
        else if( key == "size_filter_ratio" )
        {
            if( v < 0 || v > 1 )
                return at + ": 0 .. 1 (track width / pad size above it: no teardrop)";

            aParams.m_WidthtoSizeFilterRatio = v;
        }
        else
        {
            return at + ": unknown (enabled, max_length_mm, max_width_mm, length_ratio, width_ratio, "
                        "size_filter_ratio, curved, two_tracks, pads_in_zones)";
        }
    }

    return std::string();
}


const std::map<std::string, TARGET_TD>& defaultTargets()
{
    static const std::map<std::string, TARGET_TD> targets = { { "round", TARGET_ROUND },
                                                              { "rect", TARGET_RECT },
                                                              { "track", TARGET_TRACK } };
    return targets;
}


nlohmann::json settingsJson( BOARD_DESIGN_SETTINGS& aBds )
{
    TEARDROP_PARAMETERS_LIST* list = aBds.GetTeadropParamsList();
    nlohmann::json            defaults = nlohmann::json::object();

    for( const auto& [name, target] : defaultTargets() )
        defaults[name] = paramsJson( *list->GetParameters( target ) );

    return { { "targets",
               { { "vias", list->m_TargetVias },
                 { "pth_pads", list->m_TargetPTHPads },
                 { "smd_pads", list->m_TargetSMDPads },
                 { "track_to_track", list->m_TargetTrack2Track },
                 { "round_only", list->m_UseRoundShapesOnly } } },
             { "defaults", defaults } };
}


nlohmann::json teardropsJson( BOARD* aBoard, size_t aLimit )
{
    nlohmann::json items = nlohmann::json::array();
    size_t         viaPad = 0, trackEnd = 0, unfilled = 0;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( !zone->IsTeardropArea() )
            continue;

        const bool end = zone->GetTeardropAreaType() == TEARDROP_TYPE::TD_TRACKEND;
        ( end ? trackEnd : viaPad )++;

        if( !zone->IsFilled() )
            ++unfilled;

        if( items.size() < aLimit )
        {
            const VECTOR2I c = zone->GetBoundingBox().GetCenter();
            items.push_back( { { "uuid", str( zone->m_Uuid.AsString() ) },
                               { "kind", end ? "track_end" : "via_pad" },
                               { "net", str( UnescapeString( zone->GetNetname() ) ) },
                               { "layer", str( aBoard->GetLayerName( zone->GetFirstLayer() ) ) },
                               { "at_mm", { toMm( c.x ), toMm( c.y ) } },
                               { "filled", zone->IsFilled() } } );
        }
    }

    size_t viasOn = 0, padsOn = 0;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->Type() == PCB_VIA_T && track->GetTeardropParams().m_Enabled )
            ++viasOn;
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetTeardropParams().m_Enabled )
                ++padsOn;
        }
    }

    return { { "count", viaPad + trackEnd },
             { "via_pad", viaPad },
             { "track_end", trackEnd },
             { "unfilled", unfilled },
             { "enabled_on", { { "vias", viasOn }, { "pads", padsOn } } },
             { "items", items },
             { "truncated", viaPad + trackEnd > items.size() } };
}


KOPENAPI_RESULT h_pcb_teardrops( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                 board = context->GetBoard();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    TEARDROP_PARAMETERS_LIST* list = bds.GetTeadropParamsList();
    const std::string      action = aArgs.value( "action", std::string( "get" ) );
    const size_t           limit = (size_t) std::max( 0, aArgs.value( "limit", 200 ) );

    if( action != "get" && action != "add" && action != "remove" && action != "remove_all" && action != "set" )
        return KOPENAPI_RESULT::Error( 400, "action: get, add, remove, remove_all or set (Board Setup defaults and targets only)" );

    if( action == "get" )
    {
        for( const char* key : { "defaults", "vias", "pth_pads", "smd_pads", "track_to_track", "round_only", "params" } )
        {
            if( aArgs.contains( key ) )
                return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": changes nothing with action get (set, add)" );
        }

        nlohmann::json out = settingsJson( bds );
        out["teardrops"] = teardropsJson( board, limit );
        return KOPENAPI_RESULT::Ok( out );
    }

    // validate everything before anything changes
    TEARDROP_PARAMETERS_LIST newList = *list;

    if( aArgs.contains( "defaults" ) )
    {
        if( !aArgs["defaults"].is_object() )
            return KOPENAPI_RESULT::Error( 400, "defaults: {round, rect, track} each a teardrop parameter object" );

        for( const auto& [name, value] : aArgs["defaults"].items() )
        {
            auto it = defaultTargets().find( name );

            if( it == defaultTargets().end() )
                return KOPENAPI_RESULT::Error( 400, "defaults." + name + ": unknown (round, rect, track)" );

            if( std::string e = applyParams( value, *newList.GetParameters( it->second ), "defaults." + name ); !e.empty() )
                return KOPENAPI_RESULT::Error( 400, e );
        }
    }

    for( const char* key : { "vias", "pth_pads", "smd_pads", "track_to_track", "round_only" } )
    {
        if( !aArgs.contains( key ) )
            continue;

        if( !aArgs[key].is_boolean() )
            return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": true or false" );

        const bool v = aArgs[key].get<bool>();
        const std::string k = key;
        ( k == "vias"             ? newList.m_TargetVias
          : k == "pth_pads"       ? newList.m_TargetPTHPads
          : k == "smd_pads"       ? newList.m_TargetSMDPads
          : k == "track_to_track" ? newList.m_TargetTrack2Track
                                  : newList.m_UseRoundShapesOnly ) = v;
    }

    std::optional<TEARDROP_PARAMETERS> specified;

    if( aArgs.contains( "params" ) )
    {
        if( action != "add" )
            return KOPENAPI_RESULT::Error( 400, "params: with action add (the values set on each pad / via)" );

        specified = TEARDROP_PARAMETERS();

        if( std::string e = applyParams( aArgs["params"], *specified, "params" ); !e.empty() )
            return KOPENAPI_RESULT::Error( 400, e );
    }

    // filters, as the dialog's
    std::set<int> nets;

    if( aArgs.contains( "nets" ) )
    {
        for( const nlohmann::json& n : aArgs["nets"] )
        {
            NETINFO_ITEM* net = n.is_string() ? KopenapiFindNet( board, n.get<std::string>() ) : nullptr;

            if( !net )
                return KopenapiNoSuchNet( board, n.is_string() ? n.get<std::string>() : n.dump(), "nets: " );

            nets.insert( net->GetNetCode() );
        }
    }

    const wxString netclass = wxString::FromUTF8( aArgs.value( "netclass", std::string() ) );

    if( !netclass.IsEmpty() && netclass != NETCLASS::Default && !bds.m_NetSettings->HasNetclass( netclass ) )
        return KOPENAPI_RESULT::Error( 404, "netclass: no net class '" + str( netclass ) + "' (pcb_rules_get)" );

    PCB_LAYER_ID layer = UNDEFINED_LAYER;

    if( aArgs.contains( "layer" ) )
    {
        const int id = board->GetLayerID( wxString::FromUTF8( aArgs.value( "layer", std::string() ) ) );

        if( id < 0 || !IsCopperLayer( id ) )
            return KOPENAPI_RESULT::Error( 400, "layer: a copper layer of this board" );

        layer = (PCB_LAYER_ID) id;
    }

    std::set<BOARD_CONNECTED_ITEM*> picked;
    const bool                      byItems = aArgs.contains( "items" );

    if( byItems )
    {
        for( const nlohmann::json& spec : aArgs["items"] )
        {
            BOARD_ITEM* item = spec.is_string() ? KopenapiFindBoardItem( board, spec.get<std::string>() ) : nullptr;

            if( !item )
                return KOPENAPI_RESULT::Error( 404, "items: no item " + spec.dump() + " on the board" );

            if( item->Type() == PCB_FOOTPRINT_T )
            {
                for( PAD* pad : static_cast<FOOTPRINT*>( item )->Pads() )
                    picked.insert( pad );
            }
            else if( item->Type() == PCB_PAD_T || item->Type() == PCB_VIA_T )
            {
                picked.insert( static_cast<BOARD_CONNECTED_ITEM*>( item ) );
            }
            else
            {
                return KOPENAPI_RESULT::Error( 400, "items: " + spec.dump() + " is a " + str( item->GetClass() )
                                                            + "; teardrops sit on pads and vias (REF, REF.PAD, a via uuid)" );
            }
        }
    }

    const bool existingOnly = aArgs.value( "existing_only", false );

    nlohmann::json out = nlohmann::json::object();
    const size_t   before = teardropsJson( board, 0 )["count"].get<size_t>();

    *list = newList;

    if( action == "set" )
    {
        context->SetContentModified();
        out = settingsJson( bds );
        out["teardrops"] = teardropsJson( board, limit );
        return KOPENAPI_RESULT::Ok( out );
    }

    board->SetLegacyTeardrops( false );

    auto freshConnectivity = [&]()
    {
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();

        if( conn && !KopenapiConnectivityCurrent( *conn ) )
            conn->RecalculateRatsnest();
    };

    freshConnectivity();

    const bool   removeAll = action == "remove_all";
    const bool   remove = action == "remove" || removeAll;
    BOARD_COMMIT commit( context->GetToolManager() );
    size_t       changed = 0;

    auto process = [&]( BOARD_CONNECTED_ITEM* aItem )
    {
        TEARDROP_PARAMETERS  was = aItem->GetTeardropParams();
        TEARDROP_PARAMETERS  now = was;

        if( remove )
        {
            now.m_Enabled = false;
        }
        else if( specified )
        {
            now = *specified;
            now.m_Enabled = true;
        }
        else
        {
            now = *list->GetParameters( TEARDROP_MANAGER::IsUniformlyRound( aItem ) ? TARGET_ROUND : TARGET_RECT );
            now.m_Enabled = true;
        }

        if( now == was )
            return;

        commit.Stage( aItem, CHT_MODIFY );
        aItem->GetTeardropParams() = now;
        ++changed;
    };

    auto visit = [&]( BOARD_CONNECTED_ITEM* aItem, bool aAlways )
    {
        if( byItems && !picked.count( aItem ) )
            return;

        if( !aAlways )
        {
            if( !nets.empty() && !nets.count( aItem->GetNetCode() ) )
                return;

            if( !netclass.IsEmpty() )
            {
                NETCLASS* nc = aItem->GetEffectiveNetClass();

                if( !nc || !nc->ContainsNetclassWithName( netclass ) )
                    return;
            }

            if( layer != UNDEFINED_LAYER && !aItem->IsOnLayer( layer ) )
                return;

            if( list->m_UseRoundShapesOnly && !TEARDROP_MANAGER::IsUniformlyRound( aItem ) )
                return;

            if( existingOnly && !aItem->GetTeardropParams().m_Enabled )
                return;
        }

        process( aItem );
    };

    if( list->m_TargetVias || removeAll )
    {
        for( PCB_TRACK* track : board->Tracks() )
        {
            if( track->Type() == PCB_VIA_T )
                visit( track, removeAll );
        }
    }

    for( FOOTPRINT* fp : board->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( removeAll )
                visit( pad, true );
            else if( list->m_TargetPTHPads && pad->GetAttribute() == PAD_ATTRIB::PTH )
                visit( pad, false );
            else if( list->m_TargetSMDPads
                     && ( pad->GetAttribute() == PAD_ATTRIB::SMD || pad->GetAttribute() == PAD_ATTRIB::CONN ) )
                visit( pad, false );
        }
    }

    if( ( list->m_TargetTrack2Track || removeAll ) && !byItems )
    {
        TEARDROP_MANAGER manager( board, context->GetToolManager() );
        manager.DeleteTrackToTrackTeardrops( commit );
        list->GetParameters( TARGET_TRACK )->m_Enabled = !remove;

        if( !remove )
            manager.AddTeardropsOnTracks( commit, nullptr, true );
    }

    // a headless commit is no board editor's and leaves teardrops alone: rebuilt here in both
    commit.Push( _( "Edit Teardrops (API)" ), SKIP_TEARDROPS );
    freshConnectivity();

    BOARD_COMMIT     update( context->GetToolManager() );
    TEARDROP_MANAGER updater( board, context->GetToolManager() );
    updater.UpdateTeardrops( update, nullptr, nullptr, true );
    update.Push( _( "Edit Teardrops (API)" ), SKIP_TEARDROPS | APPEND_UNDO );
    freshConnectivity();

    // teardrops are zones: fill them, and the pours they now stand beside
    bool filled = false;

    if( aArgs.value( "fill", true ) )
    {
        std::vector<ZONE*> zones;

        for( ZONE* zone : board->Zones() )
        {
            if( !zone->GetIsRuleArea() && ( zone->IsTeardropArea() || zone->IsFilled() ) )
                zones.push_back( zone );
        }

        if( !zones.empty() )
        {
            BOARD_COMMIT                fillCommit( context->GetToolManager() );
            ZONE_FILLER                 filler( board, &fillCommit );
            KOPENAPI_KEEPALIVE_REPORTER reporter;
            filler.SetProgressReporter( &reporter );

            filled = filler.Fill( zones );

            if( filled )
                fillCommit.Push( _( "Fill zones (API)" ) );
            else
                fillCommit.Revert();
        }

        board->BuildConnectivity();
    }

    context->SetContentModified();

    out = settingsJson( bds );
    out["changed"] = changed;
    out["filled"] = filled;
    out["teardrops"] = teardropsJson( board, limit );

    const size_t after = out["teardrops"]["count"].get<size_t>();
    out["added"] = after > before ? after - before : 0;
    out["removed"] = before > after ? before - after : 0;
    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_teardrops",
                   "Teardrops, as the editor's Edit > Edit Teardrops and Board Setup > Teardrops. action: get "
                   "(default: the teardrops on the board, kind via_pad / track_end, the targets and the defaults), "
                   "add (on the targeted vias / pads with the board defaults for round or rectangular shapes, or "
                   "params: the values to set), remove (the filtered ones), remove_all (every teardrop), set (only "
                   "the targets and defaults). Targets (kept in Board Setup): vias, pth_pads, smd_pads, "
                   "track_to_track, round_only. Filters: nets, netclass, layer, existing_only, items (REF, REF.PAD, "
                   "via uuid). defaults: {round, rect, track}; a parameter object: enabled, max_length_mm, "
                   "max_width_mm, length_ratio, width_ratio, size_filter_ratio, curved, two_tracks, pads_in_zones. "
                   "fill (default true) fills the teardrops and the pours that were filled",
                   R"json({"type":"object","properties":{
                        "action":{"type":"string","enum":["get","add","remove","remove_all","set"],"default":"get"},
                        "vias":{"type":"boolean"},"pth_pads":{"type":"boolean"},"smd_pads":{"type":"boolean"},
                        "track_to_track":{"type":"boolean"},"round_only":{"type":"boolean"},
                        "params":{"type":"object"},
                        "defaults":{"type":"object","properties":{"round":{"type":"object"},"rect":{"type":"object"},"track":{"type":"object"}}},
                        "nets":{"type":"array","items":{"type":"string"}},
                        "netclass":{"type":"string"},"layer":{"type":"string"},
                        "existing_only":{"type":"boolean","default":false},
                        "items":{"type":"array","items":{"type":"string"}},
                        "fill":{"type":"boolean","default":true},
                        "limit":{"type":"integer","default":200}}})json"_json,
                   false, h_pcb_teardrops );

KOPENAPI_MARK_EDITING( "pcb_teardrops" );
