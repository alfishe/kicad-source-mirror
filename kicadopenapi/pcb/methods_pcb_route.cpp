/*
 * kicadopenapi routing through KiCad's push-and-shove router (PNS) - experimental.
 *
 * pcb_route_connection drives PNS the way a person does in the editor: start a track on a pad,
 * move towards the target, let the router walk around / shove, fix the route.  Nothing reaches
 * the board directly: the router's interface (KOPENAPI_PNS_IFACE) keeps every change in one
 * staged BOARD_COMMIT, a guard checks the result for gross errors (new copper touching or
 * crossing another net - pads, tracks, vias, filled zones - within the clearance), and only then
 * is the commit pushed (one undo step in the GUI).  dry_run reverts after measuring: geometry
 * and metrics of what the router would do, for comparing variants.
 */
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <geometry/shape.h>
#include <geometry/shape_poly_set.h>
#include <kicadopenapi_glow_view.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_edit_frame.h>
#include <pcb_track.h>
#include <router/pns_kicad_iface.h>
#include <router/pns_placement_algo.h>
#include <router/pns_router.h>
#include <router/pns_routing_settings.h>
#include <router/pns_sizes_settings.h>
#include <router/pns_solid.h>
#include <string_utils.h>
#include <tool/tool_manager.h>
#include <tools/pcb_tool_base.h>
#include <zone.h>

#include <chrono>
#include <cmath>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return std::round( pcbIUScale.IUTomm( aIU ) * 1000.0 ) / 1000.0;
}


/// The tool the router's interface hangs its commits on (any PCB tool will do)
class KOPENAPI_ROUTE_HOST : public PCB_TOOL_BASE
{
public:
    KOPENAPI_ROUTE_HOST() : PCB_TOOL_BASE( "kicadopenapi.RouteHost" ) {}

    /// No menus or actions: it only lends its manager to the router's commits (the base Init
    /// builds context menus, which a headless process does not have)
    bool Init() override { return true; }
    void Reset( RESET_REASON ) override {}
};


PCB_TOOL_BASE* routeHost( TOOL_MANAGER* aManager )
{
    if( KOPENAPI_ROUTE_HOST* host = aManager->GetTool<KOPENAPI_ROUTE_HOST>() )
        return host;

    // Registered only: InitTools() would re-initialise every other tool of a GUI editor
    aManager->RegisterTool( new KOPENAPI_ROUTE_HOST() );
    return aManager->GetTool<KOPENAPI_ROUTE_HOST>();
}


/// PNS interface that stages instead of pushing, and remembers what it touched
class KOPENAPI_PNS_IFACE : public PNS_KICAD_IFACE
{
public:
    void Commit() override { m_fixed = true; }   // kept staged: the caller checks, then pushes or reverts

    // No view: the editor's previews (head, ratline, path line, hidden originals) are not drawn;
    // every layer and item counts as visible (the router skips "invisible" ones)
    void EraseView() override {}
    void HideItem( PNS::ITEM* ) override {}
    void DisplayItem( const PNS::ITEM*, int, bool, int ) override {}
    void DisplayPathLine( const SHAPE_LINE_CHAIN&, int ) override {}
    void DisplayRatline( const SHAPE_LINE_CHAIN&, PNS::NET_HANDLE ) override {}
    bool IsAnyLayerVisible( const PNS_LAYER_RANGE& ) const override { return true; }
    bool IsItemVisible( const PNS::ITEM* ) const override { return true; }

    void AddItem( PNS::ITEM* aItem ) override
    {
        PNS_KICAD_IFACE::AddItem( aItem );

        if( aItem->Parent() )
            m_added.push_back( aItem->Parent() );
    }

    void UpdateItem( PNS::ITEM* aItem ) override
    {
        PNS_KICAD_IFACE::UpdateItem( aItem );

        if( aItem->Parent() )
            m_modified.insert( aItem->Parent() );
    }

    void RemoveItem( PNS::ITEM* aItem ) override
    {
        if( aItem->Parent() && aItem->Parent()->Type() != PCB_PAD_T )
            m_removed.insert( aItem->Parent() );

        PNS_KICAD_IFACE::RemoveItem( aItem );
    }

    BOARD_COMMIT& Staged() { return *m_commit; }

    bool                     m_fixed = false;
    std::vector<BOARD_ITEM*> m_added;
    std::set<BOARD_ITEM*>    m_modified;
    std::set<BOARD_ITEM*>    m_removed;
};


int netClearance( BOARD_CONNECTED_ITEM* aItem, int aFloor )
{
    NETCLASS* netclass = aItem->GetEffectiveNetClass();
    return std::max( aFloor, netclass && netclass->HasClearance() ? netclass->GetClearance() : 0 );
}


/**
 * Gross errors in a staged route: new or moved copper (aChanged) within the clearance of copper
 * of another net - pads, tracks, vias, filled zones; removed items do not count.
 */
nlohmann::json guard( BOARD* aBoard, const std::vector<BOARD_CONNECTED_ITEM*>& aChanged,
                      const std::set<BOARD_ITEM*>& aRemoved )
{
    nlohmann::json violations = nlohmann::json::array();
    const int      floor = aBoard->GetDesignSettings().m_MinClearance;

    std::vector<BOARD_CONNECTED_ITEM*> others;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( !aRemoved.count( track ) )
            others.push_back( track );
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            others.push_back( pad );
    }

    for( BOARD_CONNECTED_ITEM* other : aChanged )
        others.push_back( other );

    auto describe = [&]( BOARD_CONNECTED_ITEM* aItem )
    {
        nlohmann::json d = { { "net", str( UnescapeString( aItem->GetNetname() ) ) } };

        if( aItem->Type() == PCB_PAD_T )
            d["pad"] = str( static_cast<PAD*>( aItem )->GetParentFootprint()->GetReference() ) + "."
                       + str( static_cast<PAD*>( aItem )->GetNumber() );
        else
            d["type"] = aItem->Type() == PCB_VIA_T ? "via" : "track";

        return d;
    };

    for( BOARD_CONNECTED_ITEM* item : aChanged )
    {
        const int clearance = netClearance( item, floor );

        for( PCB_LAYER_ID layer : item->GetLayerSet().CuStack() )
        {
            std::shared_ptr<SHAPE> shape = item->GetEffectiveShape( layer );

            for( BOARD_CONNECTED_ITEM* other : others )
            {
                if( other == item || other->GetNetCode() == item->GetNetCode() || !other->IsOnLayer( layer ) )
                    continue;

                const int need = std::max( clearance, netClearance( other, floor ) ) - 1;

                int actual = 0;

                if( shape->Collide( other->GetEffectiveShape( layer ).get(), need, &actual ) )
                {
                    violations.push_back( { { "layer", str( aBoard->GetLayerName( layer ) ) },
                                            { "new", describe( item ) },
                                            { "other", describe( other ) },
                                            { "distance_mm", toMm( actual ) },
                                            { "required_mm", toMm( need + 1 ) } } );
                }
            }

            for( ZONE* zone : aBoard->Zones() )
            {
                if( zone->GetNetCode() == item->GetNetCode() || !zone->IsOnLayer( layer ) || !zone->HasFilledPolysForLayer( layer ) )
                    continue;

                const std::shared_ptr<SHAPE_POLY_SET>& fill = zone->GetFilledPolysList( layer );

                if( fill && fill->Collide( shape.get(), std::max( clearance, netClearance( zone, floor ) ) - 1 ) )
                {
                    violations.push_back( { { "layer", str( aBoard->GetLayerName( layer ) ) },
                                            { "new", describe( item ) },
                                            { "other", { { "zone", str( zone->GetZoneName() ) },
                                                         { "net", str( UnescapeString( zone->GetNetname() ) ) } } } } );
                }
            }
        }
    }

    return violations;
}


std::optional<PAD*> findPad( BOARD* aBoard, const std::string& aRefPad )
{
    const size_t dot = aRefPad.rfind( '.' );

    if( dot == std::string::npos )
        return std::nullopt;

    const wxString ref = wxString::FromUTF8( aRefPad.substr( 0, dot ) );
    const wxString number = wxString::FromUTF8( aRefPad.substr( dot + 1 ) );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( fp->GetReference() != ref )
            continue;

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNumber() == number )
                return pad;
        }
    }

    return std::nullopt;
}

} // namespace


static KOPENAPI_RESULT h_pcb_route_connection( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                board = context->GetBoard();
    const auto            started = std::chrono::steady_clock::now();
    std::optional<PAD*>   from = findPad( board, aArgs.value( "from", std::string() ) );
    std::optional<PAD*>   to;

    if( !from )
        return KOPENAPI_RESULT::Error( 404, "give 'from' as REF.PAD of an existing pad" );

    if( aArgs.contains( "to" ) )
    {
        to = findPad( board, aArgs.value( "to", std::string() ) );

        if( !to )
            return KOPENAPI_RESULT::Error( 404, "'to' pad not found" );

        if( ( *to )->GetNetCode() != ( *from )->GetNetCode() || ( *from )->GetNetCode() <= 0 )
            return KOPENAPI_RESULT::Error( 422, "'from' and 'to' are not on one net" );
    }

    const std::string layerName = aArgs.value( "layer", std::string( "F.Cu" ) );
    const int         layerId = board->GetLayerID( wxString::FromUTF8( layerName ) );

    if( layerId < 0 || !IsCopperLayer( layerId ) || !board->IsLayerEnabled( (PCB_LAYER_ID) layerId ) )
        return KOPENAPI_RESULT::Error( 400, "layer: an enabled copper layer, e.g. F.Cu, B.Cu" );

    const PCB_LAYER_ID layer = (PCB_LAYER_ID) layerId;

    if( !( *from )->IsOnLayer( layer ) )
        return KOPENAPI_RESULT::Error( 422, "'from' pad is not on " + layerName + " (SMD pad on the other side?)" );

    const std::string modeName = aArgs.value( "mode", std::string( "walkaround" ) );
    PNS::PNS_MODE     mode = modeName == "shove" ? PNS::RM_Shove : modeName == "walkaround" ? PNS::RM_Walkaround : PNS::RM_MarkObstacles;

    if( modeName != "shove" && modeName != "walkaround" )
        return KOPENAPI_RESULT::Error( 400, "mode: walkaround (go around obstacles) or shove (push other tracks)" );

    const bool dryRun = aArgs.value( "dry_run", false );

    // The router, as the editor's router tool sets it up, on our staging interface
    auto iface = std::make_unique<KOPENAPI_PNS_IFACE>();
    iface->SetBoard( board );
    // no SetView: without a view the interface draws no previews (SetView( nullptr ) would
    // create a preview group that EraseView then updates on a null view)
    iface->SetHostTool( routeHost( context->GetToolManager() ) );

    PNS::ROUTING_SETTINGS settings( nullptr, "" );
    settings.SetMode( mode );

    auto router = std::make_unique<PNS::ROUTER>();
    router->SetInterface( iface.get() );
    router->LoadSettings( &settings );
    router->SyncWorld();
    router->SetMode( PNS::PNS_MODE_ROUTE_SINGLE );

    PNS::ITEM* startItem = router->GetWorld()->FindItemByParent( *from );

    if( !startItem )
        return KOPENAPI_RESULT::Error( 500, "the router does not know the start pad" );

    const VECTOR2I start = ( *from )->GetPosition();
    PNS::SIZES_SETTINGS sizes( router->Sizes() );
    iface->SetStartLayerFromPCBNew( layer );
    iface->ImportSizes( sizes, startItem, nullptr, start );
    sizes.AddLayerPair( iface->GetPNSLayerFromBoardLayer( F_Cu ), iface->GetPNSLayerFromBoardLayer( B_Cu ) );
    router->UpdateSizes( sizes );

    nlohmann::json result = { { "from", aArgs["from"] }, { "layer", layerName }, { "mode", modeName },
                              { "net", str( UnescapeString( ( *from )->GetNetname() ) ) },
                              { "track_width_mm", toMm( sizes.TrackWidth() ) },
                              { "clearance_mm", toMm( sizes.Clearance() ) } };

    if( !router->StartRouting( start, startItem, iface->GetPNSLayerFromBoardLayer( layer ) ) )
    {
        result["routed"] = false;
        result["reason"] = "could not start: " + str( router->FailureReason() );
        return KOPENAPI_RESULT::Ok( result );
    }

    // The target: the given pad, else the nearest unconnected anchor of the net (as the editor's
    // "Attempt Finish" does)
    VECTOR2I        target;
    PNS::ITEM*      targetItem = nullptr;
    PNS_LAYER_RANGE targetLayers;

    if( to )
    {
        target = ( *to )->GetPosition();
        targetItem = router->GetWorld()->FindItemByParent( *to );
        result["to"] = aArgs["to"];
    }
    else if( !router->GetNearestRatnestAnchor( target, targetLayers, targetItem ) )
    {
        router->StopRouting();
        result["routed"] = false;
        result["reason"] = "nothing left to connect on this net from that pad";
        return KOPENAPI_RESULT::Ok( result );
    }

    // Move towards the target until the head stops changing (as Finish does)
    PNS::PLACEMENT_ALGO* placer = router->Placer();
    VECTOR2I             previous;
    int                  tries = 8;

    do
    {
        previous = placer->CurrentEnd();
        router->Move( target, targetItem );
    } while( placer->CurrentEnd() != previous && --tries );

    const bool reached = placer->CurrentEnd() == target;

    // Fix the line in the router, then commit it to the board (as the router tool does);
    // our interface stages the commit instead of pushing it
    const bool fixed = reached && router->FixRoute( target, targetItem, false, false );

    if( fixed )
        router->CommitRouting();

    if( !fixed || !iface->m_fixed )
    {
        const VECTOR2I end = fixed ? target : placer->CurrentEnd();

        if( router->RoutingInProgress() )
            router->StopRouting();

        iface->Staged().Revert();
        result["routed"] = false;
        result["reason"] = reached ? "the router could not fix the route" : "blocked: the track stops before the target";
        result["stopped_at_mm"] = { toMm( end.x ), toMm( end.y ) };
        result["target_mm"] = { toMm( target.x ), toMm( target.y ) };
        return KOPENAPI_RESULT::Ok( result );
    }

    // What the route would put on the board, and the guard over it
    std::vector<BOARD_CONNECTED_ITEM*> changed;
    nlohmann::json                     segments = nlohmann::json::array();
    double                             length = 0;
    int                                vias = 0;

    for( BOARD_ITEM* item : iface->m_added )
    {
        if( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T )
        {
            PCB_TRACK* track = static_cast<PCB_TRACK*>( item );
            length += track->GetLength();
            segments.push_back( { { "from", { toMm( track->GetStart().x ), toMm( track->GetStart().y ) } },
                                  { "to", { toMm( track->GetEnd().x ), toMm( track->GetEnd().y ) } },
                                  { "layer", str( board->GetLayerName( track->GetLayer() ) ) },
                                  { "width_mm", toMm( track->GetWidth() ) } } );
        }
        else if( item->Type() == PCB_VIA_T )
        {
            vias++;
        }

        if( BOARD_CONNECTED_ITEM* connected = dynamic_cast<BOARD_CONNECTED_ITEM*>( item ) )
            changed.push_back( connected );
    }

    for( BOARD_ITEM* item : iface->m_modified )
    {
        if( BOARD_CONNECTED_ITEM* connected = dynamic_cast<BOARD_CONNECTED_ITEM*>( item ) )
            changed.push_back( connected );
    }

    const nlohmann::json violations = guard( board, changed, iface->m_removed );
    const double         direct = ( target - start ).EuclideanNorm();

    result["segments"] = segments;
    result["metrics"] = { { "length_mm", toMm( length ) },
                          { "direct_mm", toMm( direct ) },
                          { "detour", direct > 0 ? std::round( length / direct * 100.0 ) / 100.0 : 1.0 },
                          { "segments", segments.size() },
                          { "vias", vias },
                          { "shoved_items", iface->m_modified.size() + iface->m_removed.size() },
                          { "ms", std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() } };

    if( !violations.empty() )
    {
        iface->Staged().Revert();
        result["routed"] = false;
        result["reason"] = "rejected by the guard: the route would touch another net";
        result["violations"] = violations;
        return KOPENAPI_RESULT::Ok( result );
    }

    if( dryRun )
    {
        iface->Staged().Revert();
        result["routed"] = true;
        result["applied"] = false;
        return KOPENAPI_RESULT::Ok( result );
    }

    std::vector<KIID> added;

    for( BOARD_ITEM* item : iface->m_added )
        added.push_back( item->m_Uuid );

    iface->Staged().Push( _( "Route (API)" ) );
    board->GetConnectivity()->RecalculateRatsnest();

    if( !aCtx.headless && aCtx.kiway )
    {
        // GUI: the new tracks glow (and the canvas shows them)
        if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
            frame->GetCanvas()->Refresh();
    }

    result["routed"] = true;
    result["applied"] = true;
    result["unrouted_left"] = board->GetConnectivity()->GetUnconnectedCount( true );
    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "pcb_route_connection",
                   "EXPERIMENTAL. Route one connection with KiCad's push-and-shove router, as a person "
                   "does in the editor: from a pad (REF.PAD) to a pad of the same net, or to the nearest "
                   "unconnected point of its net; layer, mode walkaround / shove; track width and clearance "
                   "from the board's rules. A guard rejects routes whose copper would touch another net. "
                   "dry_run: measure only (geometry, length, detour, vias, shoved items), nothing changes",
                   R"json({"type":"object","required":["from"],"properties":{
                        "from":{"type":"string","description":"REF.PAD"},
                        "to":{"type":"string","description":"REF.PAD on the same net; default: nearest unconnected"},
                        "layer":{"type":"string","default":"F.Cu"},
                        "mode":{"type":"string","enum":["walkaround","shove"],"default":"walkaround"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_route_connection, 120 );
