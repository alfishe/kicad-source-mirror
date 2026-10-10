/*
 * kicadopenapi copper zones: pcb_zone_add, pcb_zones_fill.
 *
 * A pour (typically GND on one or both copper layers) over the board outline or a polygon, with
 * its clearance, minimum width and thermal reliefs, filled by KiCad's own ZONE_FILLER (one undo
 * step).  The answer says what the fill did: area, islands, and how many connections are still
 * open (pads the pour could not reach need stitching vias or tracks).
 */
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
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


/// Fill the zones, push, report per zone and what stays unconnected
nlohmann::json fill( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const std::vector<ZONE*>& aZones )
{
    BOARD*       board = aContext.GetBoard();
    BOARD_COMMIT commit( aContext.GetToolManager() );
    ZONE_FILLER  filler( board, &commit );
    const bool   ok = filler.Fill( aZones );

    if( ok )
        commit.Push( _( "Fill zones (API)" ) );
    else
        commit.Revert();

    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();

    nlohmann::json zones = nlohmann::json::array();

    for( ZONE* zone : aZones )
    {
        double area = 0;
        int    islands = 0;

        for( PCB_LAYER_ID layer : zone->GetLayerSet().Seq() )
        {
            if( !zone->HasFilledPolysForLayer( layer ) )
                continue;

            const std::shared_ptr<SHAPE_POLY_SET>& poly = zone->GetFilledPolysList( layer );
            area += poly->Area();
            islands += poly->OutlineCount();
        }

        zones.push_back( { { "uuid", str( zone->m_Uuid.AsString() ) },
                           { "net", str( UnescapeString( zone->GetNetname() ) ) },
                           { "layers", [&]()
                             {
                                 nlohmann::json l = nlohmann::json::array();

                                 for( PCB_LAYER_ID layer : zone->GetLayerSet().Seq() )
                                     l.push_back( str( board->GetLayerName( layer ) ) );

                                 return l;
                             }() },
                           { "filled_mm2", mm2( area ) },
                           { "islands", islands } } );
    }

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
    zone->SetPadConnection( aArgs.value( "pads", std::string( "thermal" ) ) == "solid" ? ZONE_CONNECTION::FULL
                                                                                        : ZONE_CONNECTION::THERMAL );

    BOARD_COMMIT commit( context->GetToolManager() );
    commit.Add( zone );
    commit.Push( _( "Add zone (API)" ) );

    nlohmann::json result = { { "uuid", str( zone->m_Uuid.AsString() ) } };

    if( aArgs.value( "fill", true ) )
        result.update( fill( aCtx, *context, { zone } ) );

    return KOPENAPI_RESULT::Ok( result );
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
                        "pads":{"type":"string","enum":["thermal","solid"],"default":"thermal"},
                        "priority":{"type":"integer","default":0},
                        "name":{"type":"string"},
                        "fill":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_zone_add, 300 );

KOPENAPI_REGISTER( "pcb_zones_fill",
                   "Refill every copper zone with KiCad's zone filler (after tracks or parts moved); "
                   "answers area and islands per zone and the connections still open",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_zones_fill, 300 );

KOPENAPI_MARK_EDITING( "pcb_zone_add", "pcb_zones_fill" );
