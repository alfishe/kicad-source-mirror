/// @file methods_pcb_zones.cpp
/// @brief kicadopenapi copper zones: pcb_zone_add, pcb_zone_list, pcb_zone_update, pcb_zone_delete,
/// pcb_zones_fill.
///
/// A pour (typically GND on one or both copper layers) over the board outline or a polygon, with
/// its clearance, minimum width and thermal reliefs, filled by KiCad's own ZONE_FILLER (one undo
/// step).  The answer says what the fill did: area, islands, and how many connections are still
/// open (pads the pour could not reach need stitching vias or tracks).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <geometry/shape_poly_set.h>
#include <kicadopenapi_glow_view.h>
#include <kiway.h>
#include <netinfo.h>
#include <pcb_edit_frame.h>
#include <string_utils.h>
#include <tool/tool_manager.h>
#include <zone.h>
#include <zone_filler.h>

#include <cmath>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


int toIU( double aMm )
{
    return pcbIUScale.mmToIU( aMm );
}


double mm2( double aIU2 )
{
    return std::round( aIU2 / ( pcbIUScale.IU_PER_MM * pcbIUScale.IU_PER_MM ) * 10.0 ) / 10.0;
}


struct ZONE_GLOW_TRAITS
{
    using FRAME = PCB_EDIT_FRAME;

    static FRAME* Frame( KIWAY* aKiway )
    {
        return aKiway ? dynamic_cast<PCB_EDIT_FRAME*>( aKiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;
    }

    static EDA_ITEM* Resolve( FRAME* aFrame, const KIID& aId )
    {
        return aFrame->GetBoard() ? aFrame->GetBoard()->ResolveItem( aId, true ) : nullptr;
    }

    static void Brighten( FRAME* aFrame, EDA_ITEM* aItem, bool aOn )
    {
        if( aOn )
            aItem->SetBrightened();
        else
            aItem->ClearBrightened();

        aFrame->GetCanvas()->GetView()->Update( aItem, KIGFX::REPAINT );
    }

    static BOX2I Box( EDA_ITEM* aItem ) { return aItem->GetBoundingBox(); }
    static int   Mm() { return pcbIUScale.mmToIU( 1.0 ); }
    static void  ItemColour( FRAME*, bool, std::optional<KIGFX::COLOR4D>& ) {}
};


double toMm( int aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1000.0 ) / 1000.0;
}


const std::map<std::string, ZONE_CONNECTION>& padModes()
{
    static const std::map<std::string, ZONE_CONNECTION> modes = { { "tht_thermal", ZONE_CONNECTION::THT_THERMAL },
                                                                  { "thermal", ZONE_CONNECTION::THERMAL },
                                                                  { "solid", ZONE_CONNECTION::FULL },
                                                                  { "none", ZONE_CONNECTION::NONE } };
    return modes;
}


std::string padModeName( ZONE_CONNECTION aMode )
{
    for( const auto& [name, mode] : padModes() )
    {
        if( mode == aMode )
            return name;
    }

    return "inherited";
}


nlohmann::json layerNames( BOARD* aBoard, const LSET& aLayers )
{
    nlohmann::json l = nlohmann::json::array();

    for( PCB_LAYER_ID layer : aLayers.Seq() )
        l.push_back( str( aBoard->GetLayerName( layer ) ) );

    return l;
}


/// @brief The pieces a fill came out in, per layer, when a layer has more than one: each with its
/// box, area and the copper of the zone's net on it (pads, vias, tracks) - an island with pads and no
/// via is cut off from the rest and needs one
nlohmann::json islandsOf( BOARD* aBoard, ZONE* aZone )
{
    nlohmann::json out = nlohmann::json::array();

    for( PCB_LAYER_ID layer : aZone->GetLayerSet().Seq() )
    {
        if( !aZone->HasFilledPolysForLayer( layer ) )
            continue;

        const std::shared_ptr<SHAPE_POLY_SET>& fill = aZone->GetFilledPolysList( layer );

        if( fill->OutlineCount() < 2 )
            continue;

        for( int o = 0; o < fill->OutlineCount(); ++o )
        {
            SHAPE_POLY_SET island;
            island.AddOutline( fill->COutline( o ) );

            for( int h = 0; h < fill->HoleCount( o ); ++h )
                island.AddHole( fill->CHole( o, h ) );

            const BOX2I    box = island.BBox();
            nlohmann::json pads = nlohmann::json::array();
            int            vias = 0, tracks = 0;

            for( FOOTPRINT* fp : aBoard->Footprints() )
            {
                for( PAD* pad : fp->Pads() )
                {
                    if( pad->GetNetCode() == aZone->GetNetCode() && pad->IsOnLayer( layer )
                        && box.Intersects( pad->GetBoundingBox() )
                        && island.Collide( pad->GetEffectiveShape( layer ).get(), 0 ) )
                    {
                        pads.push_back( str( fp->GetReference() ) + "." + str( pad->GetNumber() ) );
                    }
                }
            }

            for( PCB_TRACK* track : aBoard->Tracks() )
            {
                if( track->GetNetCode() == aZone->GetNetCode() && track->IsOnLayer( layer )
                    && box.Intersects( track->GetBoundingBox() )
                    && island.Collide( track->GetEffectiveShape( layer ).get(), 0 ) )
                {
                    ( track->Type() == PCB_VIA_T ? vias : tracks )++;
                }
            }

            out.push_back( { { "layer", str( aBoard->GetLayerName( layer ) ) },
                             { "bbox_mm", { toMm( box.GetLeft() ), toMm( box.GetTop() ), toMm( box.GetRight() ), toMm( box.GetBottom() ) } },
                             { "area_mm2", mm2( island.Area() ) },
                             { "pads", pads },
                             { "vias", vias },
                             { "tracks", tracks } } );
        }
    }

    return out;
}


/// @brief A zone as the list and the answers show it
nlohmann::json zoneJson( BOARD* aBoard, ZONE* aZone )
{
    double area = 0;
    int    islands = 0;

    for( PCB_LAYER_ID layer : aZone->GetLayerSet().Seq() )
    {
        if( aZone->HasFilledPolysForLayer( layer ) )
        {
            area += aZone->GetFilledPolysList( layer )->Area();
            islands += aZone->GetFilledPolysList( layer )->OutlineCount();
        }
    }

    nlohmann::json z = { { "uuid", str( aZone->m_Uuid.AsString() ) },
                         { "net", str( UnescapeString( aZone->GetNetname() ) ) },
                         { "name", str( aZone->GetZoneName() ) },
                         { "layers", layerNames( aBoard, aZone->GetLayerSet() ) },
                         { "priority", (int) aZone->GetAssignedPriority() },
                         { "pads", padModeName( aZone->GetPadConnection() ) },
                         { "clearance_mm", aZone->GetLocalClearance() ? toMm( *aZone->GetLocalClearance() ) : 0.0 },
                         { "min_width_mm", toMm( aZone->GetMinThickness() ) },
                         { "thermal_gap_mm", toMm( aZone->GetThermalReliefGap() ) },
                         { "thermal_spoke_mm", toMm( aZone->GetThermalReliefSpokeWidth() ) },
                         { "filled", aZone->IsFilled() },
                         { "filled_mm2", mm2( area ) },
                         { "islands", islands } };

    nlohmann::json pieces = islandsOf( aBoard, aZone );

    if( !pieces.empty() )
        z["island_list"] = pieces;

    return z;
}


/// @brief Fill the zones, push, report per zone and what stays unconnected
nlohmann::json fill( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const std::vector<ZONE*>& aZones )
{
    BOARD*       board = aContext.GetBoard();
    BOARD_COMMIT commit( aContext.GetToolManager() );
    ZONE_FILLER  filler( board, &commit );
    KOPENAPI_KEEPALIVE_REPORTER reporter;
    filler.SetProgressReporter( &reporter );
    const bool   ok = filler.Fill( aZones );

    if( ok )
        commit.Push( _( "Fill zones (API)" ) );
    else
        commit.Revert();

    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();

    nlohmann::json zones = nlohmann::json::array();

    for( ZONE* zone : aZones )
        zones.push_back( zoneJson( board, zone ) );

    if( !aCtx.headless && aCtx.kiway )
    {
        if( auto* frame = ZONE_GLOW_TRAITS::Frame( aCtx.kiway ) )
            frame->GetCanvas()->Refresh();

        std::vector<KIID> ids;

        for( ZONE* zone : aZones )
            ids.push_back( zone->m_Uuid );

        KopenapiGlow<ZONE_GLOW_TRAITS>( aCtx.kiway, ids, ids.size() > 1 ? 300 : 0 );
    }

    return { { "filled", ok }, { "zones", zones },
             { "unrouted_left", (int) board->GetConnectivity()->GetUnconnectedCount( false ) } };
}

} // namespace


static KOPENAPI_RESULT h_pcb_zone_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*        board = context->GetBoard();
    const wxString netName = wxString::FromUTF8( aArgs.value( "net", std::string() ) );
    NETINFO_ITEM* net = board->FindNet( netName );

    if( !net )
        return KOPENAPI_RESULT::Error( 404, "net not found: " + aArgs.value( "net", std::string() ) + " (pcb_net_list)" );

    LSET layers;

    for( const nlohmann::json& l : aArgs.value( "layers", nlohmann::json::array( { "F.Cu", "B.Cu" } ) ) )
    {
        const int id = l.is_string() ? board->GetLayerID( wxString::FromUTF8( l.get<std::string>() ) ) : -1;

        if( id < 0 || !IsCopperLayer( id ) || !board->IsLayerEnabled( (PCB_LAYER_ID) id ) )
            return KOPENAPI_RESULT::Error( 400, "layers: enabled copper layers" );

        layers.set( id );
    }

    // The outline: the board's, or a polygon in mm
    SHAPE_LINE_CHAIN outline;

    if( aArgs.contains( "points" ) )
    {
        for( const nlohmann::json& p : aArgs["points"] )
        {
            if( !p.is_array() || p.size() != 2 )
                return KOPENAPI_RESULT::Error( 400, "points: [[x_mm, y_mm], ...]" );

            outline.Append( toIU( p[0].get<double>() ), toIU( p[1].get<double>() ) );
        }

        if( outline.PointCount() < 3 )
            return KOPENAPI_RESULT::Error( 400, "points: at least 3" );
    }
    else
    {
        SHAPE_POLY_SET boardShape;

        if( !board->GetBoardPolygonOutlines( boardShape, true ) || boardShape.OutlineCount() == 0 )
            return KOPENAPI_RESULT::Error( 409, "no board outline (pcb_outline_set) and no points" );

        outline = boardShape.Outline( 0 );
    }

    outline.SetClosed( true );

    auto* zone = new ZONE( board );
    zone->SetLayerSet( layers );
    zone->SetNet( net );
    zone->AddPolygon( outline );
    zone->SetZoneName( wxString::FromUTF8( aArgs.value( "name", std::string() ) ) );
    zone->SetAssignedPriority( aArgs.value( "priority", 0 ) );
    zone->SetLocalClearance( toIU( aArgs.value( "clearance_mm", 0.3 ) ) );
    zone->SetMinThickness( toIU( aArgs.value( "min_width_mm", 0.25 ) ) );
    zone->SetThermalReliefGap( toIU( aArgs.value( "thermal_gap_mm", 0.5 ) ) );
    zone->SetThermalReliefSpokeWidth( toIU( aArgs.value( "thermal_spoke_mm", 0.5 ) ) );
    const std::string padMode = aArgs.value( "pads", std::string( "tht_thermal" ) );

    if( !padModes().count( padMode ) )
    {
        delete zone;
        return KOPENAPI_RESULT::Error( 400, "pads: tht_thermal (thermal reliefs on through-hole pads, SMD pads solid), thermal, solid or none" );
    }

    zone->SetPadConnection( padModes().at( padMode ) );

    BOARD_COMMIT commit( context->GetToolManager() );
    commit.Add( zone );
    commit.Push( _( "Add zone (API)" ) );

    nlohmann::json result = { { "uuid", str( zone->m_Uuid.AsString() ) } };

    if( aArgs.value( "fill", true ) )
        result.update( fill( aCtx, *context, { zone } ) );

    return KOPENAPI_RESULT::Ok( result );
}


namespace
{

/// @brief The zones a call names: by uuid, or every copper zone of a net; none named: an error
std::vector<ZONE*> namedZones( BOARD* aBoard, const nlohmann::json& aArgs, std::string& aError )
{
    std::vector<ZONE*> zones;

    if( aArgs.contains( "uuid" ) )
    {
        const KIID id( wxString::FromUTF8( aArgs.value( "uuid", std::string() ) ) );

        for( ZONE* zone : aBoard->Zones() )
        {
            if( zone->m_Uuid == id )
                zones.push_back( zone );
        }

        if( zones.empty() )
            aError = "no zone with that uuid (pcb_zone_list)";
    }
    else if( aArgs.contains( "net" ) )
    {
        const wxString net = wxString::FromUTF8( aArgs.value( "net", std::string() ) );

        for( ZONE* zone : aBoard->Zones() )
        {
            if( !zone->GetIsRuleArea() && UnescapeString( zone->GetNetname() ) == net )
                zones.push_back( zone );
        }

        if( zones.empty() )
            aError = "no copper zone of net " + aArgs.value( "net", std::string() ) + " (pcb_zone_list)";
    }
    else
    {
        aError = "say which zone: uuid, or net (all its copper zones)";
    }

    return zones;
}

} // namespace


static KOPENAPI_RESULT h_pcb_zone_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    nlohmann::json items = nlohmann::json::array();
    const wxString net = wxString::FromUTF8( aArgs.value( "net", std::string() ) );

    for( ZONE* zone : board->Zones() )
    {
        if( zone->GetIsRuleArea() || ( !net.IsEmpty() && UnescapeString( zone->GetNetname() ) != net ) )
            continue;

        items.push_back( zoneJson( board, zone ) );
    }

    return KOPENAPI_RESULT::Ok( { { "items", items }, { "total", items.size() } } );
}


static KOPENAPI_RESULT h_pcb_zone_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*             board = context->GetBoard();
    std::string        error;
    std::vector<ZONE*> zones = namedZones( board, aArgs, error );

    if( zones.empty() )
        return KOPENAPI_RESULT::Error( 404, error );

    if( aArgs.contains( "pads" ) && !padModes().count( aArgs.value( "pads", std::string() ) ) )
        return KOPENAPI_RESULT::Error( 400, "pads: tht_thermal, thermal, solid or none" );

    for( const char* key : { "clearance_mm", "min_width_mm", "thermal_gap_mm", "thermal_spoke_mm" } )
    {
        if( aArgs.contains( key ) && ( !aArgs[key].is_number() || aArgs[key].get<double>() < 0 ) )
            return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": a number >= 0 (mm)" );
    }

    BOARD_COMMIT commit( context->GetToolManager() );

    for( ZONE* zone : zones )
    {
        commit.Modify( zone );

        if( aArgs.contains( "pads" ) )
            zone->SetPadConnection( padModes().at( aArgs["pads"].get<std::string>() ) );

        if( aArgs.contains( "clearance_mm" ) )
            zone->SetLocalClearance( toIU( aArgs["clearance_mm"].get<double>() ) );

        if( aArgs.contains( "min_width_mm" ) )
            zone->SetMinThickness( toIU( aArgs["min_width_mm"].get<double>() ) );

        if( aArgs.contains( "thermal_gap_mm" ) )
            zone->SetThermalReliefGap( toIU( aArgs["thermal_gap_mm"].get<double>() ) );

        if( aArgs.contains( "thermal_spoke_mm" ) )
            zone->SetThermalReliefSpokeWidth( toIU( aArgs["thermal_spoke_mm"].get<double>() ) );

        if( aArgs.contains( "priority" ) )
            zone->SetAssignedPriority( aArgs.value( "priority", 0 ) );

        if( aArgs.contains( "name" ) )
            zone->SetZoneName( wxString::FromUTF8( aArgs.value( "name", std::string() ) ) );
    }

    commit.Push( _( "Update zone (API)" ) );

    nlohmann::json result = { { "updated", zones.size() } };

    if( aArgs.value( "fill", true ) )
        result.update( fill( aCtx, *context, zones ) );

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_zone_delete( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*             board = context->GetBoard();
    std::string        error;
    std::vector<ZONE*> zones = namedZones( board, aArgs, error );

    if( zones.empty() )
        return KOPENAPI_RESULT::Error( 404, error );

    for( ZONE* zone : zones )
    {
        if( zone->IsLocked() && !aArgs.value( "override_locks", false ) )
            return KOPENAPI_RESULT::Error( 409, "zone " + str( zone->m_Uuid.AsString() ) + " is locked (pcb_unlock, or override_locks)" );
    }

    BOARD_COMMIT commit( context->GetToolManager() );

    for( ZONE* zone : zones )
        commit.Remove( zone );

    commit.Push( _( "Delete zone (API)" ) );
    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();

    return KOPENAPI_RESULT::Ok( { { "deleted", zones.size() },
                                  { "unrouted_left", (int) board->GetConnectivity()->GetUnconnectedCount( false ) } } );
}


static KOPENAPI_RESULT h_pcb_zones_fill( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    std::vector<ZONE*> zones;

    for( ZONE* zone : context->GetBoard()->Zones() )
    {
        if( !zone->GetIsRuleArea() )
            zones.push_back( zone );
    }

    if( zones.empty() )
        return KOPENAPI_RESULT::Error( 409, "no copper zones (pcb_zone_add)" );

    return KOPENAPI_RESULT::Ok( fill( aCtx, *context, zones ) );
}


KOPENAPI_REGISTER( "pcb_zone_add",
                   "Add a copper pour (e.g. GND) on one or more copper layers over the board outline or "
                   "a polygon (points in mm): clearance, minimum width, thermal reliefs or solid pads, "
                   "priority; filled at once by KiCad's zone filler (fill: false to skip); answers area, "
                   "islands and the connections still open",
                   R"json({"type":"object","required":["net"],"properties":{
                        "net":{"type":"string"},
                        "layers":{"type":"array","items":{"type":"string"},"default":["F.Cu","B.Cu"]},
                        "points":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"polygon [[x_mm, y_mm], ...]; default the board outline"},
                        "clearance_mm":{"type":"number","default":0.3},
                        "min_width_mm":{"type":"number","default":0.25},
                        "thermal_gap_mm":{"type":"number","default":0.5},
                        "thermal_spoke_mm":{"type":"number","default":0.5},
                        "pads":{"type":"string","enum":["tht_thermal","thermal","solid","none"],"default":"tht_thermal","description":"tht_thermal: thermal reliefs on through-hole pads, SMD pads solid (small SMD pads starve with reliefs)"},
                        "priority":{"type":"integer","default":0},
                        "name":{"type":"string"},
                        "fill":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_zone_add, 300 );

KOPENAPI_REGISTER( "pcb_zone_list",
                   "Copper zones (pours) with uuid, net, layers, priority, pad connection, clearance, minimum "
                   "width, thermal gap / spoke, filled area and islands; a fill in more than one piece on a "
                   "layer lists each (island_list: box, area, pads / vias / tracks of the net on it - one with "
                   "pads and no via is cut off and needs one)",
                   R"json({"type":"object","properties":{"net":{"type":"string"}}})json"_json,
                   false, h_pcb_zone_list );

KOPENAPI_REGISTER( "pcb_zone_update",
                   "Change copper zones (by uuid, or every zone of a net): pad connection, clearance, minimum "
                   "width, thermal gap / spoke, priority, name; refilled (fill: false to skip); one undo step",
                   R"json({"type":"object","properties":{
                        "uuid":{"type":"string"},"net":{"type":"string"},
                        "pads":{"type":"string","enum":["tht_thermal","thermal","solid","none"]},
                        "clearance_mm":{"type":"number"},"min_width_mm":{"type":"number"},
                        "thermal_gap_mm":{"type":"number"},"thermal_spoke_mm":{"type":"number"},
                        "priority":{"type":"integer"},"name":{"type":"string"},
                        "fill":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_zone_update, 300 );

KOPENAPI_REGISTER( "pcb_zone_delete",
                   "Delete copper zones (by uuid, or every zone of a net); locked ones refused (409) unless "
                   "override_locks; one undo step; answers the connections still open",
                   R"json({"type":"object","properties":{
                        "uuid":{"type":"string"},"net":{"type":"string"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_zone_delete );

KOPENAPI_REGISTER( "pcb_zones_fill",
                   "Refill every copper zone with KiCad's zone filler (after tracks or parts moved); "
                   "answers area and islands per zone and the connections still open",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_zones_fill, 300 );

KOPENAPI_MARK_EDITING( "pcb_zone_add", "pcb_zones_fill", "pcb_zone_update", "pcb_zone_delete" );
