/// @file methods_pcb_zones.cpp
/// @brief kicadopenapi copper zones and keep-outs: pcb_zone_add, pcb_zone_list, pcb_zone_update,
/// pcb_zone_delete, pcb_zones_fill, pcb_keepout_add, pcb_keepout_list.
///
/// A pour (typically GND on one or both copper layers) over the board outline or a polygon, with
/// its clearance, minimum width and thermal reliefs, filled by KiCad's own ZONE_FILLER (one undo
/// step).  The answer says what the fill did: area, islands, and how many connections are still
/// open (pads the pour could not reach need stitching vias or tracks).  A keep-out is KiCad's rule
/// area: a ZONE with SetIsRuleArea and its do-not-allow flags; update and delete go through the zone
/// methods by uuid.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_util.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_rect.h>
#include <kicadopenapi_glow_view.h>
#include <kiway.h>
#include <netinfo.h>
#include <pcb_edit_frame.h>
#include <string_utils.h>
#include <tool/tool_manager.h>
#include <zone.h>
#include <zone_filler.h>

#include <algorithm>
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

    static void Brighten( FRAME* aFrame, EDA_ITEM* aItem, bool aOn ) { KopenapiPcbBrighten( aFrame, aItem, aOn ); }

    static BOX2I Box( EDA_ITEM* aItem ) { return aItem->GetBoundingBox(); }
    static int   Mm() { return pcbIUScale.mmToIU( 1.0 ); }
    static void  ItemColour( FRAME*, bool, std::optional<KIGFX::COLOR4D>& ) {}
};


double toMm( int aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1000.0 ) / 1000.0;
}


/// @brief The outline of points [[x, y], ...] or rect [x0, y0, x1, y1] in mm; empty with aError set
SHAPE_LINE_CHAIN outlineArg( const nlohmann::json& aArgs, std::string& aError )
{
    SHAPE_LINE_CHAIN outline;

    if( aArgs.contains( "points" ) && aArgs.contains( "rect" ) )
    {
        aError = "points or rect, not both";
        return outline;
    }

    if( aArgs.contains( "rect" ) )
    {
        const nlohmann::json& r = aArgs["rect"];

        if( !r.is_array() || r.size() != 4 || !std::all_of( r.begin(), r.end(), []( const nlohmann::json& v ) { return v.is_number(); } ) )
        {
            aError = "rect: [x0_mm, y0_mm, x1_mm, y1_mm]";
            return outline;
        }

        const int x0 = toIU( std::min( r[0].get<double>(), r[2].get<double>() ) );
        const int x1 = toIU( std::max( r[0].get<double>(), r[2].get<double>() ) );
        const int y0 = toIU( std::min( r[1].get<double>(), r[3].get<double>() ) );
        const int y1 = toIU( std::max( r[1].get<double>(), r[3].get<double>() ) );

        if( x0 == x1 || y0 == y1 )
        {
            aError = "rect: an area, not a line";
            return outline;
        }

        outline.Append( x0, y0 );
        outline.Append( x1, y0 );
        outline.Append( x1, y1 );
        outline.Append( x0, y1 );
    }
    else
    {
        for( const nlohmann::json& p : aArgs.value( "points", nlohmann::json::array() ) )
        {
            if( !p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number() )
            {
                aError = "points: [[x_mm, y_mm], ...]";
                return SHAPE_LINE_CHAIN();
            }

            outline.Append( toIU( p[0].get<double>() ), toIU( p[1].get<double>() ) );
        }

        if( outline.PointCount() < 3 )
        {
            aError = "points: at least 3 (or rect)";
            return SHAPE_LINE_CHAIN();
        }
    }

    outline.SetClosed( true );
    return outline;
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


const std::map<std::string, ISLAND_REMOVAL_MODE>& islandModes()
{
    static const std::map<std::string, ISLAND_REMOVAL_MODE> modes = { { "always", ISLAND_REMOVAL_MODE::ALWAYS },
                                                                      { "never", ISLAND_REMOVAL_MODE::NEVER },
                                                                      { "area", ISLAND_REMOVAL_MODE::AREA } };
    return modes;
}


std::string islandModeName( ISLAND_REMOVAL_MODE aMode )
{
    for( const auto& [name, mode] : islandModes() )
    {
        if( mode == aMode )
            return name;
    }

    return "never";
}


// remove_islands / min_island_mm2 checked; empty when valid
std::string islandArgsError( const nlohmann::json& aArgs )
{
    if( aArgs.contains( "remove_islands" )
        && ( !aArgs["remove_islands"].is_string() || !islandModes().count( aArgs["remove_islands"].get<std::string>() ) ) )
    {
        return "remove_islands: always, never or area (below min_island_mm2)";
    }

    if( aArgs.contains( "min_island_mm2" ) && ( !aArgs["min_island_mm2"].is_number() || aArgs["min_island_mm2"].get<double>() < 0 ) )
        return "min_island_mm2: a number >= 0";

    return {};
}


void applyIslandArgs( ZONE* aZone, const nlohmann::json& aArgs )
{
    if( aArgs.contains( "remove_islands" ) )
        aZone->SetIslandRemovalMode( islandModes().at( aArgs["remove_islands"].get<std::string>() ) );

    if( aArgs.contains( "min_island_mm2" ) )
    {
        const double iu = pcbIUScale.IU_PER_MM;
        aZone->SetMinIslandArea( (long long) ( aArgs["min_island_mm2"].get<double>() * iu * iu ) );
    }
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
                         { "remove_islands", islandModeName( aZone->GetIslandRemovalMode() ) },
                         { "min_island_mm2", mm2( (double) aZone->GetMinIslandArea() ) },
                         { "filled", aZone->IsFilled() },
                         { "filled_mm2", mm2( area ) },
                         { "islands", islands } };

    if( const SHAPE_POLY_SET* poly = aZone->Outline(); poly && poly->OutlineCount() > 0 )
    {
        const BOX2I    box = poly->BBox();
        nlohmann::json pts = nlohmann::json::array();

        for( const VECTOR2I& pt : poly->COutline( 0 ).CPoints() )
            pts.push_back( { toMm( pt.x ), toMm( pt.y ) } );

        z["bbox_mm"] = { toMm( box.GetLeft() ), toMm( box.GetTop() ), toMm( box.GetRight() ), toMm( box.GetBottom() ) };
        z["outline_mm"] = pts;
    }

    nlohmann::json pieces = islandsOf( aBoard, aZone );

    if( !pieces.empty() )
        z["island_list"] = pieces;

    return z;
}


/// @brief Fill the zones, push, report per zone and what stays unconnected
nlohmann::json fill( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const std::vector<ZONE*>& aZones,
                    const KOPENAPI_LIST_CAP* aCap = nullptr )
{
    BOARD*       board = aContext.GetBoard();
    BOARD_COMMIT commit( aContext.GetToolManager() );
    ZONE_FILLER  filler( board, &commit );
    const int    count = (int) aZones.size();

    KOPENAPI_PROGRESS_REPORTER reporter(
            [count]( const std::string& aMessage, int aPhase, double aFraction )
            {
                return KopenapiJobProgress( "zones_fill " + std::to_string( aPhase ), (int) std::lround( aFraction * 1000 ),
                                            1000, aMessage, { { "step", aPhase }, { "zones", count } } );
            } );

    filler.SetProgressReporter( &reporter );
    const bool   ok = filler.Fill( aZones );

    KopenapiKeepUiAlive();

    if( ok )
        commit.Push( _( "Fill zones (API)" ) );
    else
        commit.Revert();

    KopenapiKeepUiAlive();
    KopenapiReadPoint();
    board->BuildConnectivity();
    KopenapiKeepUiAlive();
    board->GetConnectivity()->RecalculateRatsnest();

    nlohmann::json zones = nlohmann::json::array();

    for( ZONE* zone : aZones )
    {
        KopenapiKeepUiAlive();
        nlohmann::json z = zoneJson( board, zone );

        // a polygon cut short would mislead: one over the limit is left out, its point count kept
        if( aCap && z.contains( "outline_mm" ) && z["outline_mm"].size() > aCap->Limit() )
        {
            z["outline_mm_total"] = z["outline_mm"].size();
            z.erase( "outline_mm" );
            z["truncated"] = nlohmann::json::array( { "outline_mm" } );
        }

        if( aCap && z.contains( "island_list" ) )
            aCap->Put( z, "island_list", z["island_list"] );

        zones.push_back( std::move( z ) );
    }

    if( !aCtx.headless && aCtx.kiway )
    {
        KopenapiKeepUiAlive();

        if( auto* frame = ZONE_GLOW_TRAITS::Frame( aCtx.kiway ) )
            frame->GetCanvas()->Refresh();

        std::vector<KIID> ids;

        for( ZONE* zone : aZones )
            ids.push_back( zone->m_Uuid );

        KopenapiGlow<ZONE_GLOW_TRAITS>( aCtx.kiway, ids, ids.size() > 1 ? 300 : 0 );
    }

    KopenapiKeepUiAlive();

    nlohmann::json out = { { "filled", ok } };

    if( aCap )
        aCap->Put( out, "zones", std::move( zones ) );
    else
        out["zones"] = std::move( zones );

    out.update( KopenapiUnroutedJson( board ) );

    // a job stopped while filling: nothing of the fill applied
    if( reporter.IsCancelled() )
        out["stopped"] = true;

    return out;
}

} // namespace


void KopenapiNetDomain( BOARD* aBoard, int aNetCode, int aLayer, SHAPE_POLY_SET& aOut )
{
    aOut.RemoveAllContours();
    const PCB_LAYER_ID layer = (PCB_LAYER_ID) aLayer;
    std::vector<ZONE*> zones;

    for( ZONE* zone : aBoard->Zones() )
        zones.push_back( zone );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( ZONE* zone : fp->Zones() )
            zones.push_back( zone );
    }

    SHAPE_POLY_SET barred;

    for( ZONE* zone : zones )
    {
        if( zone->GetIsRuleArea() && zone->GetDoNotAllowZoneFills() && zone->IsOnLayer( layer ) )
            barred.Append( zone->GetBoardOutline() );
    }

    for( ZONE* zone : zones )
    {
        if( zone->GetIsRuleArea() || zone->IsTeardropArea() || zone->GetNetCode() != aNetCode || !zone->IsOnLayer( layer ) )
            continue;

        SHAPE_POLY_SET mine = zone->GetBoardOutline();
        SHAPE_POLY_SET above = barred;

        for( ZONE* other : zones )
        {
            if( other == zone || other->GetIsRuleArea() || other->IsTeardropArea() || other->GetNetCode() == aNetCode
                || !other->IsOnLayer( layer ) || !other->HigherPriority( zone )
                || !other->GetBoundingBox().Intersects( zone->GetBoundingBox() ) )
            {
                continue;
            }

            above.Append( other->GetBoardOutline() );
        }

        if( above.OutlineCount() )
        {
            above.Simplify();
            mine.BooleanSubtract( above );
        }

        aOut.Append( mine );
    }

    if( !aOut.OutlineCount() )
        return;

    aOut.Simplify();
    SHAPE_POLY_SET board;

    if( aBoard->GetBoardPolygonOutlines( board, true ) && board.OutlineCount() )
        aOut.BooleanIntersection( board );
}


nlohmann::json KopenapiZoneIslands( BOARD* aBoard, ZONE* aZone )
{
    return islandsOf( aBoard, aZone );
}


static KOPENAPI_RESULT h_pcb_zone_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*        board = context->GetBoard();
    NETINFO_ITEM* net = KopenapiFindNet( board, aArgs.value( "net", std::string() ) );

    if( !net )
        return KopenapiNoSuchNet( board, aArgs.value( "net", std::string() ) );

    LSET layers;

    for( const nlohmann::json& l : aArgs.value( "layers", nlohmann::json::array( { "F.Cu", "B.Cu" } ) ) )
    {
        const int id = l.is_string() ? board->GetLayerID( wxString::FromUTF8( l.get<std::string>() ) ) : -1;

        if( id < 0 || !IsCopperLayer( id ) || !board->IsLayerEnabled( (PCB_LAYER_ID) id ) )
            return KOPENAPI_RESULT::Error( 400, "layers: enabled copper layers" );

        layers.set( id );
    }

    // The outline: a polygon or rect in mm, else the board's
    SHAPE_LINE_CHAIN outline;

    if( aArgs.contains( "points" ) || aArgs.contains( "rect" ) )
    {
        std::string error;
        outline = outlineArg( aArgs, error );

        if( !error.empty() )
            return KOPENAPI_RESULT::Error( 400, error );
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

    if( const std::string bad = islandArgsError( aArgs ); !bad.empty() )
    {
        delete zone;
        return KOPENAPI_RESULT::Error( 400, bad );
    }

    applyIslandArgs( zone, aArgs );
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

/// @brief Board zones and the zones inside footprints: what a zone uuid (or prefix) is matched against
std::vector<BOARD_ITEM*> allZones( BOARD* aBoard )
{
    std::vector<BOARD_ITEM*> zones( aBoard->Zones().begin(), aBoard->Zones().end() );

    for( FOOTPRINT* fp : aBoard->Footprints() )
        zones.insert( zones.end(), fp->Zones().begin(), fp->Zones().end() );

    return zones;
}


/// @brief The zones a call names: by uuid (or a unique prefix), or every copper zone of a net; none named: an error
std::vector<ZONE*> namedZones( BOARD* aBoard, const nlohmann::json& aArgs, std::string& aError )
{
    std::vector<ZONE*> zones;

    if( aArgs.contains( "uuid" ) )
    {
        std::string error;

        if( ZONE* zone = dynamic_cast<ZONE*>( KopenapiMatchUuid( allZones( aBoard ), aArgs.value( "uuid", std::string() ), error ) );
            zone && !zone->GetParentFootprint() )
        {
            zones.push_back( zone );
        }
        else
        {
            aError = ( error.empty() ? std::string( "that zone belongs to a footprint" ) : error )
                     + " (pcb_zone_list, pcb_keepout_list)";
        }
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


/// @brief 409 when the uuid names a rule area inside a footprint: it is the footprint's, changed in
/// its library and brought over by pcb_footprint_lib_update
std::optional<KOPENAPI_RESULT> footprintZoneRefusal( BOARD* aBoard, const nlohmann::json& aArgs )
{
    if( !aArgs.contains( "uuid" ) )
        return std::nullopt;

    std::string error;
    BOARD_ITEM* named = KopenapiMatchUuid( allZones( aBoard ), aArgs.value( "uuid", std::string() ), error );

    if( !named && error.find( "ambiguous" ) != std::string::npos )
        return KOPENAPI_RESULT::Error( 400, error );

    const KIID id = named ? named->m_Uuid : niluuid;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( ZONE* zone : fp->Zones() )
        {
            if( zone != named )
                continue;

            return KOPENAPI_RESULT::Error( 409, "zone " + str( id.AsString() ) + " ('" + str( zone->GetZoneName() ) + "') belongs to footprint "
                                                + str( fp->GetReference() ) + " (" + str( fp->GetFPID().Format() )
                                                + "): change it in that library footprint and bring it over with "
                                                  "pcb_footprint_lib_update, or add a board keep-out (pcb_keepout_add)" );
        }
    }

    return std::nullopt;
}


struct KEEPOUT_FLAG
{
    const char* name;
    bool ( ZONE::*get )() const;
    void ( ZONE::*set )( bool );
    bool byDefault;
};


const std::vector<KEEPOUT_FLAG>& keepoutFlags()
{
    static const std::vector<KEEPOUT_FLAG> flags = {
        { "no_tracks", &ZONE::GetDoNotAllowTracks, &ZONE::SetDoNotAllowTracks, true },
        { "no_vias", &ZONE::GetDoNotAllowVias, &ZONE::SetDoNotAllowVias, true },
        { "no_pads", &ZONE::GetDoNotAllowPads, &ZONE::SetDoNotAllowPads, true },
        { "no_copper_pour", &ZONE::GetDoNotAllowZoneFills, &ZONE::SetDoNotAllowZoneFills, true },
        { "no_footprints", &ZONE::GetDoNotAllowFootprints, &ZONE::SetDoNotAllowFootprints, false }
    };
    return flags;
}


/// @brief Copper layers by name, "*.Cu" for every enabled one; empty with aError set when one is not
LSET copperLayers( BOARD* aBoard, const nlohmann::json& aNames, std::string& aError )
{
    LSET       layers;
    const LSET enabled = LSET::AllCuMask( aBoard->GetCopperLayerCount() ) & aBoard->GetEnabledLayers();

    if( !aNames.is_array() || aNames.empty() )
    {
        aError = "layers: a list of enabled copper layers, or [\"*.Cu\"]";
        return layers;
    }

    for( const nlohmann::json& l : aNames )
    {
        if( l.is_string() && l.get<std::string>() == "*.Cu" )
        {
            layers |= enabled;
            continue;
        }

        const int id = l.is_string() ? aBoard->GetLayerID( wxString::FromUTF8( l.get<std::string>() ) ) : -1;

        if( id < 0 || !IsCopperLayer( id ) || !aBoard->IsLayerEnabled( (PCB_LAYER_ID) id ) )
        {
            aError = "layers: enabled copper layers, or \"*.Cu\" (" + l.dump() + " is not one)";
            return LSET();
        }

        layers.set( id );
    }

    return layers;
}


/// @brief The keep-out flags of aArgs checked; empty when valid
std::string keepoutFlagsError( const nlohmann::json& aArgs )
{
    for( const KEEPOUT_FLAG& flag : keepoutFlags() )
    {
        if( aArgs.contains( flag.name ) && !aArgs[flag.name].is_boolean() )
            return std::string( flag.name ) + ": true or false";
    }

    return {};
}


/// @brief What a keep-out bars that is inside it now: tracks, vias, pads, footprints (by courtyard,
/// the footprint's box without one), as KiCad's DRC reports them: a footprint's keep-out does not
/// bar the footprint itself, its pads are barred like any other
nlohmann::json keepoutViolations( BOARD* aBoard, ZONE* aZone )
{
    nlohmann::json        out = nlohmann::json::array();
    const SHAPE_POLY_SET  outline = aZone->GetBoardOutline();
    const SHAPE_POLY_SET* area = &outline;
    const FOOTPRINT*      owner = aZone->GetParentFootprint();

    if( area->OutlineCount() == 0 )
        return out;

    const BOX2I box = area->BBox();
    const LSET  layers = aZone->GetLayerSet();

    auto hits = [&]( BOARD_ITEM* aItem, PCB_LAYER_ID& aLayer )
    {
        if( !box.Intersects( aItem->GetBoundingBox() ) )
            return false;

        for( PCB_LAYER_ID layer : LSET( aItem->GetLayerSet() & layers ).CuStack() )
        {
            if( area->Collide( aItem->GetEffectiveShape( layer ).get(), 0 ) )
            {
                aLayer = layer;
                return true;
            }
        }

        return false;
    };

    PCB_LAYER_ID layer = UNDEFINED_LAYER;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        const bool via = track->Type() == PCB_VIA_T;

        if( !( via ? aZone->GetDoNotAllowVias() : aZone->GetDoNotAllowTracks() ) || !hits( track, layer ) )
            continue;

        nlohmann::json v = { { "type", via ? "via" : "track" },
                             { "uuid", str( track->m_Uuid.AsString() ) },
                             { "net", str( UnescapeString( track->GetNetname() ) ) },
                             { "layer", str( aBoard->GetLayerName( layer ) ) } };

        if( via )
        {
            v["at_mm"] = { toMm( track->GetPosition().x ), toMm( track->GetPosition().y ) };
        }
        else
        {
            v["from_mm"] = { toMm( track->GetStart().x ), toMm( track->GetStart().y ) };
            v["to_mm"] = { toMm( track->GetEnd().x ), toMm( track->GetEnd().y ) };
        }

        out.push_back( v );
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( aZone->GetDoNotAllowPads() )
        {
            for( PAD* pad : fp->Pads() )
            {
                if( hits( pad, layer ) )
                {
                    out.push_back( { { "type", "pad" },
                                     { "pad", str( fp->GetReference() ) + "." + str( pad->GetNumber() ) },
                                     { "net", str( UnescapeString( pad->GetNetname() ) ) },
                                     { "layer", str( aBoard->GetLayerName( layer ) ) } } );
                }
            }
        }

        if( !aZone->GetDoNotAllowFootprints() || fp == owner || !box.Intersects( fp->GetBoundingBox( false ) ) )
            continue;

        bool inside = false;
        bool courtyard = false;

        for( const auto& [crtyd, side] : { std::pair{ F_CrtYd, LSET::FrontMask() }, std::pair{ B_CrtYd, LSET::BackMask() } } )
        {
            const SHAPE_POLY_SET& outline = fp->GetCourtyard( crtyd );

            if( outline.IsEmpty() )
                continue;

            courtyard = true;

            if( !( layers & side ).any() )
                continue;

            SHAPE_POLY_SET common = *area;
            common.BooleanIntersection( outline );
            inside = inside || !common.IsEmpty();
        }

        if( !courtyard && layers.test( fp->GetSide() == B_Cu ? B_Cu : F_Cu ) )
        {
            const BOX2I b = fp->GetBoundingBox( false );
            SHAPE_RECT  rect( b.GetPosition(), b.GetWidth(), b.GetHeight() );
            inside = area->Collide( &rect, 0 );
        }

        if( inside )
            out.push_back( { { "type", "footprint" }, { "ref", str( fp->GetReference() ) }, { "courtyard", courtyard } } );
    }

    return out;
}


/// @brief A keep-out as the list and the answers show it
nlohmann::json keepoutJson( BOARD* aBoard, ZONE* aZone )
{
    nlohmann::json flags = nlohmann::json::object();

    for( const KEEPOUT_FLAG& flag : keepoutFlags() )
        flags[flag.name] = ( aZone->*flag.get )();

    nlohmann::json k = { { "uuid", str( aZone->m_Uuid.AsString() ) },
                         { "name", str( aZone->GetZoneName() ) },
                         { "layers", layerNames( aBoard, aZone->GetLayerSet() ) },
                         { "flags", flags },
                         { "locked", aZone->IsLocked() },
                         { "footprint_owned", aZone->GetParentFootprint() != nullptr } };

    if( const FOOTPRINT* owner = aZone->GetParentFootprint() )
    {
        k["footprint"] = str( owner->GetReference() );
        k["footprint_lib_id"] = str( owner->GetFPID().Format() );
    }

    // on the board: a footprint's zone keeps its outline in the footprint's own coordinates
    if( SHAPE_POLY_SET poly = aZone->GetBoardOutline(); poly.OutlineCount() > 0 )
    {
        const BOX2I    box = poly.BBox();
        nlohmann::json pts = nlohmann::json::array();

        for( const VECTOR2I& pt : poly.COutline( 0 ).CPoints() )
            pts.push_back( { toMm( pt.x ), toMm( pt.y ) } );

        k["area_mm2"] = mm2( poly.Area() );
        k["bbox_mm"] = { toMm( box.GetLeft() ), toMm( box.GetTop() ), toMm( box.GetRight() ), toMm( box.GetBottom() ) };
        k["outline_mm"] = pts;
    }

    return k;
}


/// @brief Filled copper pours sharing a layer with the keep-out and overlapping aArea: the ones a
/// change of the keep-out reshapes
std::vector<ZONE*> poursUnder( BOARD* aBoard, ZONE* aKeepout, const BOX2I& aArea )
{
    std::vector<ZONE*> zones;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone != aKeepout && !zone->GetIsRuleArea() && !zone->IsTeardropArea() && zone->IsFilled()
            && ( zone->GetLayerSet() & aKeepout->GetLayerSet() ).any() && aArea.Intersects( zone->GetBoundingBox() ) )
        {
            zones.push_back( zone );
        }
    }

    return zones;
}


/// @brief The answer of a keep-out edit, pushed as one undo step: the pours under it refilled into
/// aCommit (when aRefill), the keep-out and what it now bars
nlohmann::json pushKeepout( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, BOARD_COMMIT& aCommit, ZONE* aZone,
                            const BOX2I& aArea, bool aRefill, const wxString& aMessage )
{
    BOARD*             board = aContext.GetBoard();
    std::vector<ZONE*> pours = poursUnder( board, aZone, aArea );
    bool               filled = true;

    // not refilled now: the pours are marked stale, so methods judging copper by the fill refill first
    if( !aRefill )
    {
        for( ZONE* pour : pours )
            pour->SetNeedRefill( true );

        pours.clear();
    }

    if( !pours.empty() )
    {
        ZONE_FILLER                 filler( board, &aCommit );
        KOPENAPI_KEEPALIVE_REPORTER reporter;
        filler.SetProgressReporter( &reporter );
        filled = filler.Fill( pours );
    }

    aCommit.Push( aMessage );
    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();

    nlohmann::json       answer = keepoutJson( board, aZone );
    const nlohmann::json bad = keepoutViolations( board, aZone );
    constexpr size_t     shown = 200;

    answer["violations"] = bad.size() > shown ? nlohmann::json( std::vector<nlohmann::json>( bad.begin(), bad.begin() + shown ) ) : bad;
    answer["violations_total"] = bad.size();
    answer["zones_refilled"] = pours.size();

    if( !filled )
        answer["refill_failed"] = true;

    answer.update( KopenapiUnroutedJson( board ) );

    if( !aCtx.headless && aCtx.kiway )
    {
        if( auto* frame = ZONE_GLOW_TRAITS::Frame( aCtx.kiway ) )
            frame->GetCanvas()->Refresh();

        KopenapiGlow<ZONE_GLOW_TRAITS>( aCtx.kiway, { aZone->m_Uuid }, 0 );
    }

    return answer;
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


/// @brief pcb_zone_update of a keep-out (by uuid): outline, layers, name, flags; one undo step with
/// the pours under it refilled
static KOPENAPI_RESULT updateKeepout( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, ZONE* aZone, const nlohmann::json& aArgs )
{
    BOARD* board = aContext.GetBoard();

    for( const char* key : { "set_net", "pads", "clearance_mm", "min_width_mm", "thermal_gap_mm", "thermal_spoke_mm",
                             "remove_islands", "min_island_mm2", "priority" } )
    {
        if( aArgs.contains( key ) )
        {
            return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": a copper zone setting; a keep-out has points / rect, layers, "
                                                "name and no_tracks, no_vias, no_pads, no_copper_pour, no_footprints" );
        }
    }

    if( const std::string bad = keepoutFlagsError( aArgs ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad );

    std::string      error;
    SHAPE_LINE_CHAIN outline;

    if( aArgs.contains( "points" ) || aArgs.contains( "rect" ) )
    {
        outline = outlineArg( aArgs, error );

        if( !error.empty() )
            return KOPENAPI_RESULT::Error( 400, error );
    }

    LSET layers;

    if( aArgs.contains( "layers" ) )
    {
        layers = copperLayers( board, aArgs["layers"], error );

        if( !error.empty() )
            return KOPENAPI_RESULT::Error( 400, error );
    }

    if( aZone->IsLocked() && !aArgs.value( "override_locks", false ) )
        return KOPENAPI_RESULT::Error( 409, "keep-out " + str( aZone->m_Uuid.AsString() ) + " is locked (pcb_unlock, or override_locks)" );

    BOX2I area = aZone->GetBoundingBox();

    BOARD_COMMIT commit( aContext.GetToolManager() );
    commit.Modify( aZone );

    if( outline.PointCount() >= 3 )
    {
        SHAPE_POLY_SET poly;
        poly.AddOutline( outline );
        aZone->SetOutline( new SHAPE_POLY_SET( poly ) );
    }

    if( layers.any() )
        aZone->SetLayerSet( layers );

    if( aArgs.contains( "name" ) )
        aZone->SetZoneName( wxString::FromUTF8( aArgs.value( "name", std::string() ) ) );

    for( const KEEPOUT_FLAG& flag : keepoutFlags() )
    {
        if( aArgs.contains( flag.name ) )
            ( aZone->*flag.set )( aArgs[flag.name].get<bool>() );
    }

    area.Merge( aZone->GetBoundingBox() );
    nlohmann::json answer = pushKeepout( aCtx, aContext, commit, aZone, area, aArgs.value( "fill", true ),
                                         _( "Update keep-out (API)" ) );
    answer["updated"] = 1;
    return KOPENAPI_RESULT::Ok( answer );
}


static KOPENAPI_RESULT h_pcb_zone_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*             board = context->GetBoard();
    std::string        error;
    if( std::optional<KOPENAPI_RESULT> refused = footprintZoneRefusal( board, aArgs ) )
        return *refused;

    std::vector<ZONE*> zones = namedZones( board, aArgs, error );

    if( zones.empty() )
        return KOPENAPI_RESULT::Error( 404, error );

    if( zones.front()->GetIsRuleArea() )
        return updateKeepout( aCtx, *context, zones.front(), aArgs );

    for( const KEEPOUT_FLAG& flag : keepoutFlags() )
    {
        if( aArgs.contains( flag.name ) )
            return KOPENAPI_RESULT::Error( 400, std::string( flag.name ) + ": a keep-out flag; this is a copper zone (pcb_keepout_add)" );
    }

    if( aArgs.contains( "pads" ) && !padModes().count( aArgs.value( "pads", std::string() ) ) )
        return KOPENAPI_RESULT::Error( 400, "pads: tht_thermal, thermal, solid or none" );

    if( const std::string bad = islandArgsError( aArgs ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad );

    for( const char* key : { "clearance_mm", "min_width_mm", "thermal_gap_mm", "thermal_spoke_mm" } )
    {
        if( aArgs.contains( key ) && ( !aArgs[key].is_number() || aArgs[key].get<double>() < 0 ) )
            return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": a number >= 0 (mm)" );
    }

    if( aArgs.contains( "uuid" ) && aArgs.contains( "net" ) )
        return KOPENAPI_RESULT::Error( 400, "uuid or net selects the zones, not both (set_net changes the net)" );

    NETINFO_ITEM* setNet = nullptr;

    if( aArgs.contains( "set_net" ) )
    {
        setNet = KopenapiFindNet( board, aArgs.value( "set_net", std::string() ) );

        if( !setNet || setNet->GetNetCode() <= 0 )
            return KopenapiNoSuchNet( board, aArgs.value( "set_net", std::string() ), "set_net: " );
    }

    SHAPE_LINE_CHAIN outline;

    if( aArgs.contains( "points" ) || aArgs.contains( "rect" ) )
    {
        if( zones.size() != 1 )
            return KOPENAPI_RESULT::Error( 400, std::string( aArgs.contains( "rect" ) ? "rect" : "points" ) + ": one zone (by uuid)" );

        outline = outlineArg( aArgs, error );

        if( !error.empty() )
            return KOPENAPI_RESULT::Error( 400, error );
    }

    BOARD_COMMIT commit( context->GetToolManager() );

    for( ZONE* zone : zones )
    {
        commit.Modify( zone );

        if( outline.PointCount() >= 3 )
        {
            SHAPE_POLY_SET poly;
            poly.AddOutline( outline );
            zone->SetOutline( new SHAPE_POLY_SET( poly ) );
            zone->UnFill();
        }

        if( setNet )
        {
            zone->SetNet( setNet );
            zone->UnFill();
        }

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

        applyIslandArgs( zone, aArgs );

        // the fill no longer matches the settings until the zone is refilled (fill: false)
        for( const auto& [key, value] : aArgs.items() )
        {
            if( key != "uuid" && key != "net" && key != "name" && key != "fill" )
                zone->SetNeedRefill( true );
        }
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
    if( std::optional<KOPENAPI_RESULT> refused = footprintZoneRefusal( board, aArgs ) )
        return *refused;

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

    nlohmann::json answer = { { "deleted", zones.size() } };
    answer.update( KopenapiUnroutedJson( board ) );
    return KOPENAPI_RESULT::Ok( answer );
}


// entries per list of pcb_zones_fill's answer by default
constexpr size_t ZONES_FILL_LIST_CAP = 20;


static KOPENAPI_RESULT h_pcb_zones_fill( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
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

    const KOPENAPI_LIST_CAP cap( aArgs, ZONES_FILL_LIST_CAP );
    return KOPENAPI_RESULT::Ok( fill( aCtx, *context, zones, &cap ) );
}


static KOPENAPI_RESULT h_pcb_keepout_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*      board = context->GetBoard();
    std::string error;

    if( !aArgs.contains( "points" ) && !aArgs.contains( "rect" ) )
        return KOPENAPI_RESULT::Error( 400, "the area: points [[x_mm, y_mm], ...] or rect [x0_mm, y0_mm, x1_mm, y1_mm]" );

    const SHAPE_LINE_CHAIN outline = outlineArg( aArgs, error );

    if( !error.empty() )
        return KOPENAPI_RESULT::Error( 400, error );

    const LSET layers = copperLayers( board, aArgs.value( "layers", nlohmann::json::array( { "*.Cu" } ) ), error );

    if( !error.empty() )
        return KOPENAPI_RESULT::Error( 400, error );

    if( const std::string bad = keepoutFlagsError( aArgs ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad );

    auto* zone = new ZONE( board );
    zone->SetIsRuleArea( true );
    zone->SetLayerSet( layers );
    zone->AddPolygon( outline );
    zone->SetZoneName( wxString::FromUTF8( aArgs.value( "name", std::string() ) ) );

    for( const KEEPOUT_FLAG& flag : keepoutFlags() )
        ( zone->*flag.set )( aArgs.value( flag.name, flag.byDefault ) );

    // on the board before the commit is pushed, so the filler of the same undo step sees it
    BOARD_COMMIT commit( context->GetToolManager() );
    board->Add( zone );
    commit.Added( zone );

    return KOPENAPI_RESULT::Ok( pushKeepout( aCtx, *context, commit, zone, zone->GetBoundingBox(),
                                             aArgs.value( "fill", true ) && zone->GetDoNotAllowZoneFills(),
                                             _( "Add keep-out (API)" ) ) );
}


static KOPENAPI_RESULT h_pcb_keepout_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    nlohmann::json items = nlohmann::json::array();
    const bool     withViolations = aArgs.value( "violations", true );

    std::vector<ZONE*> zones( board->Zones().begin(), board->Zones().end() );

    for( FOOTPRINT* fp : board->Footprints() )
        zones.insert( zones.end(), fp->Zones().begin(), fp->Zones().end() );

    for( ZONE* zone : zones )
    {
        if( !zone->GetIsRuleArea() )
            continue;

        nlohmann::json k = keepoutJson( board, zone );

        if( withViolations )
        {
            const nlohmann::json bad = keepoutViolations( board, zone );
            k["violations_total"] = bad.size();
            k["violations"] = bad.size() > 50 ? nlohmann::json( std::vector<nlohmann::json>( bad.begin(), bad.begin() + 50 ) ) : bad;
        }

        items.push_back( k );
    }

    return KOPENAPI_RESULT::Ok( { { "items", items }, { "total", items.size() } } );
}


KOPENAPI_REGISTER( "pcb_zone_add",
                   "Add a copper pour (e.g. GND) on one or more copper layers over the board outline, "
                   "a polygon (points in mm) or a rect: clearance, minimum width, thermal reliefs or solid pads, "
                   "priority; filled at once by KiCad's zone filler (fill: false to skip); answers area, "
                   "islands and the connections still open",
                   R"json({"type":"object","required":["net"],"properties":{
                        "net":{"type":"string"},
                        "layers":{"type":"array","items":{"type":"string"},"default":["F.Cu","B.Cu"]},
                        "points":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"polygon [[x_mm, y_mm], ...] (or rect, not both); neither: the board outline"},
                        "rect":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm] (or points, not both)"},
                        "clearance_mm":{"type":"number","default":0.3},
                        "min_width_mm":{"type":"number","default":0.25},
                        "thermal_gap_mm":{"type":"number","default":0.5},
                        "thermal_spoke_mm":{"type":"number","default":0.5},
                        "remove_islands":{"type":"string","enum":["always","never","area"],"description":"drop fill pieces with no pad (area: those below min_island_mm2)"},
                        "min_island_mm2":{"type":"number"},
                        "pads":{"type":"string","enum":["tht_thermal","thermal","solid","none"],"default":"tht_thermal","description":"tht_thermal: thermal reliefs on through-hole pads, SMD pads solid (small SMD pads starve with reliefs)"},
                        "priority":{"type":"integer","default":0},
                        "name":{"type":"string"},
                        "fill":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_zone_add, 300 );

KOPENAPI_REGISTER( "pcb_zone_list",
                   "Copper zones (pours; keep-outs: pcb_keepout_list) with uuid, net, layers, outline (outline_mm, bbox_mm), priority, pad connection, clearance, minimum "
                   "width, thermal gap / spoke, filled area and islands; a fill in more than one piece on a "
                   "layer lists each (island_list: box, area, pads / vias / tracks of the net on it - one with "
                   "pads and no via is cut off and needs one)",
                   R"json({"type":"object","properties":{"net":{"type":"string"}}})json"_json,
                   false, h_pcb_zone_list );

KOPENAPI_REGISTER( "pcb_zone_update",
                   "Change copper zones (by uuid, or every zone of a net): net (set_net), outline (points or rect, one zone), pad "
                   "connection, clearance, minimum width, thermal gap / spoke, island removal, priority, name; refilled (fill: "
                   "false to skip); one undo step.  A keep-out (by uuid): points / rect, layers, name and its no_* flags, the "
                   "pours under it refilled; answers what it now bars (violations).  A rule area inside a footprint "
                   "(pcb_keepout_list footprint_owned): 409 naming the footprint; change it in the library footprint and "
                   "pcb_footprint_lib_update",
                   R"json({"type":"object","properties":{
                        "uuid":{"type":"string"},"net":{"type":"string","description":"every zone of this net (instead of uuid)"},
                        "set_net":{"type":"string","description":"new net of the zones"},
                        "points":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"new outline [[x_mm, y_mm], ...] (one zone; or rect)"},
                        "pads":{"type":"string","enum":["tht_thermal","thermal","solid","none"]},
                        "clearance_mm":{"type":"number"},"min_width_mm":{"type":"number"},
                        "thermal_gap_mm":{"type":"number"},"thermal_spoke_mm":{"type":"number"},
                        "remove_islands":{"type":"string","enum":["always","never","area"],"description":"drop fill pieces with no pad (area: those below min_island_mm2)"},
                        "min_island_mm2":{"type":"number"},
                        "priority":{"type":"integer"},"name":{"type":"string"},
                        "rect":{"type":"array","items":{"type":"number"},"description":"new outline [x0_mm, y0_mm, x1_mm, y1_mm] (one zone or keep-out; or points)"},
                        "layers":{"type":"array","items":{"type":"string"},"description":"keep-out: copper layers, \"*.Cu\" for all"},
                        "no_tracks":{"type":"boolean"},"no_vias":{"type":"boolean"},"no_pads":{"type":"boolean"},
                        "no_copper_pour":{"type":"boolean"},"no_footprints":{"type":"boolean"},
                        "override_locks":{"type":"boolean","default":false,"description":"keep-out: change a locked one"},
                        "fill":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_zone_update, 300 );

KOPENAPI_REGISTER( "pcb_zone_delete",
                   "Delete copper zones (by uuid, or every zone of a net) or a keep-out (by uuid); locked ones refused (409) "
                   "unless override_locks; a rule area inside a footprint refused (409, naming the footprint: it goes "
                   "with its library footprint); one undo step; answers the connections still open",
                   R"json({"type":"object","properties":{
                        "uuid":{"type":"string"},"net":{"type":"string","description":"every zone of this net (instead of uuid)"},
                        "set_net":{"type":"string","description":"new net of the zones"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_zone_delete );

KOPENAPI_REGISTER( "pcb_zones_fill",
                   "Refill every copper zone with KiCad's zone filler (after tracks or parts moved); "
                   "answers area and islands per zone and the connections still open; zones, each zone's island_list "
                   "and outline points capped at list_limit (an outline over it is left out, outline_mm_total gives its points; "
                   "counts whole), full: true for all",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{}})json"_json, ZONES_FILL_LIST_CAP ),
                   false, h_pcb_zones_fill, 300 );

KOPENAPI_REGISTER( "pcb_keepout_add",
                   "Add a keep-out (KiCad rule area) over a polygon (points in mm) or a rect on copper layers (default all, "
                   "\"*.Cu\"): no tracks, no vias, no pads, no copper pour (default all true), no footprints (default false); "
                   "the router, the guard of pcb_track_add / pcb_via_add and DRC keep out of it; the pours under it refilled "
                   "in the same undo step (fill: false to skip); answers uuid, layers, flags, area and what is inside it "
                   "now that it bars (violations: tracks, vias, pads, footprints by courtyard).  Change / delete by uuid: "
                   "pcb_zone_update, pcb_zone_delete",
                   R"json({"type":"object","properties":{
                        "points":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"polygon [[x_mm, y_mm], ...] (or rect)"},
                        "rect":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm] (or points)"},
                        "layers":{"type":"array","items":{"type":"string"},"default":["*.Cu"],"description":"copper layers; \"*.Cu\" for all enabled"},
                        "name":{"type":"string"},
                        "no_tracks":{"type":"boolean","default":true},
                        "no_vias":{"type":"boolean","default":true},
                        "no_pads":{"type":"boolean","default":true},
                        "no_copper_pour":{"type":"boolean","default":true},
                        "no_footprints":{"type":"boolean","default":false},
                        "fill":{"type":"boolean","default":true,"description":"refill the pours under it (no_copper_pour)"}}})json"_json,
                   false, h_pcb_keepout_add, 300 );

KOPENAPI_REGISTER( "pcb_keepout_list",
                   "Keep-outs (KiCad rule areas) of the board and inside footprints (footprint_owned, footprint: its ref, "
                   "footprint_lib_id) with uuid, name, layers, flags (no_tracks, no_vias, no_pads, no_copper_pour, "
                   "no_footprints), area, outline on the board (outline_mm, bbox_mm), locked, and the items inside each "
                   "that it bars (violations, first 50; as DRC: a footprint's keep-out does not bar the footprint itself, "
                   "its own pads are barred like any other; violations: false to skip).  A footprint's keep-out is changed in its library footprint "
                   "(pcb_footprint_lib_update), pcb_zone_update / pcb_zone_delete refuse it (409)",
                   R"json({"type":"object","properties":{"violations":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_keepout_list );

KOPENAPI_MARK_EDITING( "pcb_zone_add", "pcb_zones_fill", "pcb_zone_update", "pcb_zone_delete", "pcb_keepout_add" );
KOPENAPI_MARK_CONCURRENT_READS( "pcb_keepout_list", "pcb_zone_list" );
