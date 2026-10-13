/// @file methods_pcb_route.cpp
/// @brief kicadopenapi routing through KiCad's push-and-shove router (PNS) - experimental.
///
/// pcb_route_connection drives PNS the way a person does in the editor: start a track on a pad,
/// move towards the target, let the router walk around / shove, fix the route.  Nothing reaches
/// the board directly: the router's interface (KOPENAPI_PNS_IFACE) keeps every change in one
/// staged BOARD_COMMIT, a guard checks the result for gross errors (new copper touching or
/// crossing another net - pads, tracks, vias, filled zones - within the clearance), and only then
/// is the commit pushed (one undo step in the GUI).  dry_run reverts after measuring: geometry
/// and metrics of what the router would do, for comparing variants.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <ratsnest/ratsnest_data.h>
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
#include <wx/utils.h>
#include <zone_filler.h>

#include <chrono>
#include <deque>
#include <map>
#include <thread>
#include <cmath>
#include <optional>
#include <set>


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


/// @brief The tool the router's interface hangs its commits on (any PCB tool will do)
class KOPENAPI_ROUTE_HOST : public PCB_TOOL_BASE
{
public:
    KOPENAPI_ROUTE_HOST() : PCB_TOOL_BASE( "kicadopenapi.RouteHost" ) {}

    /// @brief No menus or actions: it only lends its manager to the router's commits (the base Init
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


/// @brief PNS interface that stages instead of pushing, and remembers what it touched
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


/// @brief Gross errors in a staged route: new or moved copper (aChanged) within the clearance of copper
/// of another net - pads, tracks, vias, filled zones; removed items do not count.
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


/// @brief A pad by REF.PAD.  Several pads can share a number (SOT-223: pin 2 and the tab): aNot excludes
/// one (the route's start), and the nearest to aNear wins.
std::optional<PAD*> findPad( BOARD* aBoard, const std::string& aRefPad, PAD* aNot = nullptr, PAD* aNear = nullptr )
{
    const size_t dot = aRefPad.rfind( '.' );

    if( dot == std::string::npos )
        return std::nullopt;

    const wxString ref = wxString::FromUTF8( aRefPad.substr( 0, dot ) );
    const wxString number = wxString::FromUTF8( aRefPad.substr( dot + 1 ) );

    std::optional<PAD*> best;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( fp->GetReference() != ref )
            continue;

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNumber() != number || pad == aNot )
                continue;

            if( !best || ( aNear && ( pad->GetPosition() - aNear->GetPosition() ).EuclideanNorm()
                                            < ( ( *best )->GetPosition() - aNear->GetPosition() ).EuclideanNorm() ) )
            {
                best = pad;
            }
        }
    }

    return best;
}

} // namespace


namespace
{

/// @brief Board editor glow for routes (no frame headless)
struct ROUTE_GLOW_TRAITS
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


struct ROUTE_REQUEST
{
    PAD*                  from = nullptr;
    BOARD_CONNECTED_ITEM* to = nullptr;        ///< nullptr: the nearest unconnected anchor of the net
    VECTOR2I              toPos;
    PCB_LAYER_ID          layer = F_Cu;
    PNS::PNS_MODE         mode = PNS::RM_Walkaround;
    bool                  dryRun = false;

    /// @brief Points on the way: with a layer the route drops a through via there and continues on
    /// that layer (a person pressing V while routing); without one it fixes a corner there (a click)
    std::vector<std::pair<VECTOR2I, std::optional<PCB_LAYER_ID>>> vias;
};


/// @brief One connection through PNS as a person routes it (see the file comment).  Fills aOut with what
/// happened (routed / reason / segments / metrics / violations); the staged commit is pushed only
/// when the guard is clean and it is no dry run.  aAdded: uuids of the new tracks / vias.
/// @brief 409 while the editor's interactive router is routing or dragging (it shares the global
/// router instance with ours)
static std::optional<KOPENAPI_RESULT> guiRouting()
{
    if( PNS::ROUTER* gui = PNS::ROUTER::GetInstance(); gui && gui->RoutingInProgress() )
        return KOPENAPI_RESULT::Error( 409, "the editor is routing interactively: finish or cancel it first" );

    return std::nullopt;
}


bool routeOne( PCB_CONTEXT& aContext, const ROUTE_REQUEST& aReq, nlohmann::json& aOut, std::vector<KIID>& aAdded )
{
    BOARD*     board = aContext.GetBoard();
    const auto started = std::chrono::steady_clock::now();

    auto iface = std::make_unique<KOPENAPI_PNS_IFACE>();
    iface->SetBoard( board );
    // no SetView: without a view the interface draws no previews (SetView( nullptr ) would
    // create a preview group that EraseView then updates on a null view)
    iface->SetHostTool( routeHost( aContext.GetToolManager() ) );

    PNS::ROUTING_SETTINGS settings( nullptr, "" );
    settings.SetMode( aReq.mode );

    // the router is a process-wide singleton: the editor's own router is current again afterwards
    struct RESTORE_ROUTER
    {
        PNS::ROUTER* previous = PNS::ROUTER::GetInstance();
        ~RESTORE_ROUTER() { PNS::ROUTER::SetInstance( previous ); }
    } restoreRouter;

    auto router = std::make_unique<PNS::ROUTER>();
    router->SetInterface( iface.get() );
    router->LoadSettings( &settings );
    router->SyncWorld();
    router->SetMode( PNS::PNS_MODE_ROUTE_SINGLE );

    PNS::ITEM* startItem = router->GetWorld()->FindItemByParent( aReq.from );

    if( !startItem )
    {
        aOut["routed"] = false;
        aOut["reason"] = "the router does not know the start pad";
        return false;
    }

    const VECTOR2I      start = aReq.from->GetPosition();
    PNS::SIZES_SETTINGS sizes( router->Sizes() );
    iface->SetStartLayerFromPCBNew( aReq.layer );
    iface->ImportSizes( sizes, startItem, nullptr, start );
    sizes.AddLayerPair( iface->GetPNSLayerFromBoardLayer( F_Cu ), iface->GetPNSLayerFromBoardLayer( B_Cu ) );
    router->UpdateSizes( sizes );

    aOut["net"] = str( UnescapeString( aReq.from->GetNetname() ) );
    aOut["layer"] = str( board->GetLayerName( aReq.layer ) );
    aOut["mode"] = aReq.mode == PNS::RM_Shove ? "shove" : "walkaround";
    aOut["track_width_mm"] = toMm( sizes.TrackWidth() );
    aOut["clearance_mm"] = toMm( sizes.Clearance() );

    if( !router->StartRouting( start, startItem, iface->GetPNSLayerFromBoardLayer( aReq.layer ) ) )
    {
        aOut["routed"] = false;
        aOut["reason"] = "could not start: " + str( router->FailureReason() );
        return false;
    }

    // Vias on the way: route to the point, drop a through via there, continue on the next layer
    PNS::PLACEMENT_ALGO* placer = router->Placer();

    for( const auto& [at, viaLayer] : aReq.vias )
    {
        if( !viaLayer )
        {
            // a corner: route to the point and fix what is there, on the same layer
            router->Move( at, nullptr );
            router->Move( at, nullptr );

            const bool there = placer->CurrentEnd() == at;

            // FixRoute answers whether the route is finished: here it only fixes what leads to the point
            if( there )
                router->FixRoute( at, nullptr, false, false );

            if( !there || !router->RoutingInProgress() )
            {
                const VECTOR2I end = placer->CurrentEnd();
                router->StopRouting();
                iface->Staged().Revert();
                aOut["routed"] = false;
                aOut["reason"] = "blocked before the waypoint at " + std::to_string( toMm( at.x ) ) + ", "
                                 + std::to_string( toMm( at.y ) );
                aOut["stopped_at_mm"] = { toMm( end.x ), toMm( end.y ) };
                return false;
            }

            continue;
        }

        const PCB_LAYER_ID layer = *viaLayer;

        PNS::SIZES_SETTINGS viaSizes( router->Sizes() );
        viaSizes.SetViaType( VIATYPE::THROUGH );
        viaSizes.ClearLayerPairs();
        viaSizes.AddLayerPair( router->GetCurrentLayer(), iface->GetPNSLayerFromBoardLayer( layer ) );
        router->UpdateSizes( viaSizes );

        if( !router->IsPlacingVia() )
            router->ToggleViaPlacement();

        router->Move( at, nullptr );
        router->Move( at, nullptr );

        if( placer->CurrentEnd() != at )
        {
            const VECTOR2I end = placer->CurrentEnd();
            router->StopRouting();
            iface->Staged().Revert();
            aOut["routed"] = false;
            aOut["reason"] = "blocked before the via at " + std::to_string( toMm( at.x ) ) + ", " + std::to_string( toMm( at.y ) );
            aOut["stopped_at_mm"] = { toMm( end.x ), toMm( end.y ) };
            return false;
        }

        router->FixRoute( at, nullptr, false, false );

        if( !router->SwitchLayer( iface->GetPNSLayerFromBoardLayer( layer ) ) )
        {
            router->StopRouting();
            iface->Staged().Revert();
            aOut["routed"] = false;
            aOut["reason"] = "could not change layer at the via";
            return false;
        }
    }

    VECTOR2I        target;
    PNS::ITEM*      targetItem = nullptr;
    PNS_LAYER_RANGE targetLayers;

    if( aReq.to )
    {
        target = aReq.toPos;
        targetItem = router->GetWorld()->FindItemByParent( aReq.to );

        if( targetItem )
            targetLayers = targetItem->Layers();
    }
    else if( !router->GetNearestRatnestAnchor( target, targetLayers, targetItem ) )
    {
        router->StopRouting();
        aOut["routed"] = false;
        aOut["reason"] = "nothing left to connect on this net from that pad";
        return false;
    }

    // Move towards the target until the head stops changing (as the editor's Attempt Finish)
    VECTOR2I             previous;
    int                  tries = 8;

    do
    {
        previous = placer->CurrentEnd();
        router->Move( target, targetItem );
    } while( placer->CurrentEnd() != previous && --tries );

    // At the target point and on one of its layers (an SMD pad is on one side only): what the
    // editor's Finish requires too
    const bool reached = placer->CurrentEnd() == target && targetLayers.Overlaps( router->GetCurrentLayer() );

    // Fix the line in the router, then commit it to the board (as the router tool does); our
    // interface stages the commit instead of pushing it
    const bool fixed = reached && router->FixRoute( target, targetItem, false, false );

    if( fixed )
        router->CommitRouting();

    if( !fixed || !iface->m_fixed )
    {
        const VECTOR2I end = fixed ? target : placer->CurrentEnd();

        if( router->RoutingInProgress() )
            router->StopRouting();

        iface->Staged().Revert();
        aOut["routed"] = false;
        aOut["reason"] = reached ? "the router could not fix the route" : "blocked: the track stops before the target";
        aOut["stopped_at_mm"] = { toMm( end.x ), toMm( end.y ) };
        aOut["target_mm"] = { toMm( target.x ), toMm( target.y ) };
        return false;
    }

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

    aOut["segments"] = segments;
    aOut["metrics"] = { { "length_mm", toMm( length ) },
                        { "direct_mm", toMm( direct ) },
                        { "detour", direct > 0 ? std::round( length / direct * 100.0 ) / 100.0 : 1.0 },
                        { "segments", segments.size() },
                        { "vias", vias },
                        { "shoved_items", iface->m_modified.size() + iface->m_removed.size() },
                        { "ms", std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() } };

    if( !violations.empty() )
    {
        iface->Staged().Revert();
        aOut["routed"] = false;
        aOut["reason"] = "rejected by the guard: the route would touch another net";
        aOut["violations"] = violations;
        return false;
    }

    if( aReq.dryRun )
    {
        iface->Staged().Revert();
        aOut["routed"] = true;
        aOut["applied"] = false;
        return true;
    }

    for( BOARD_ITEM* item : iface->m_added )
        aAdded.push_back( item->m_Uuid );

    iface->Staged().Push( _( "Route (API)" ) );
    board->GetConnectivity()->RecalculateRatsnest();
    aOut["routed"] = true;
    aOut["applied"] = true;
    return true;
}


void showRoutes( KOPENAPI_CONTEXT& aCtx, const std::vector<KIID>& aAdded )
{
    if( aCtx.headless || !aCtx.kiway )
        return;

    if( auto* frame = ROUTE_GLOW_TRAITS::Frame( aCtx.kiway ) )
        frame->GetCanvas()->Refresh();

    KopenapiGlow<ROUTE_GLOW_TRAITS>( aCtx.kiway, aAdded, aAdded.size() > 1 ? 150 : 0 );
}


std::optional<PCB_LAYER_ID> parseLayer( BOARD* aBoard, const std::string& aName )
{
    const int id = aBoard->GetLayerID( wxString::FromUTF8( aName ) );

    if( id < 0 || !IsCopperLayer( id ) || !aBoard->IsLayerEnabled( (PCB_LAYER_ID) id ) )
        return std::nullopt;

    return (PCB_LAYER_ID) id;
}

} // namespace


static KOPENAPI_RESULT h_pcb_route_connection( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD*              board = context->GetBoard();
    std::optional<PAD*> from = findPad( board, aArgs.value( "from", std::string() ) );

    if( !from )
        return KOPENAPI_RESULT::Error( 404, "give 'from' as REF.PAD of an existing pad" );

    ROUTE_REQUEST req;
    req.from = *from;
    req.dryRun = aArgs.value( "dry_run", false );

    if( aArgs.contains( "to" ) )
    {
        std::optional<PAD*> to = findPad( board, aArgs.value( "to", std::string() ), *from, *from );

        if( !to )
            return KOPENAPI_RESULT::Error( 404, "'to' pad not found" );

        if( ( *to )->GetNetCode() != ( *from )->GetNetCode() || ( *from )->GetNetCode() <= 0 )
            return KOPENAPI_RESULT::Error( 422, "'from' and 'to' are not on one net" );

        req.to = *to;
        req.toPos = ( *to )->GetPosition();
    }

    const std::string           layerName = aArgs.value( "layer", std::string( "F.Cu" ) );
    std::optional<PCB_LAYER_ID> layer = parseLayer( board, layerName );

    if( !layer )
        return KOPENAPI_RESULT::Error( 400, "layer: an enabled copper layer, e.g. F.Cu, B.Cu" );

    if( !( *from )->IsOnLayer( *layer ) )
        return KOPENAPI_RESULT::Error( 422, "'from' pad is not on " + layerName + " (SMD pad on the other side?)" );

    req.layer = *layer;

    const std::string modeName = aArgs.value( "mode", std::string( "walkaround" ) );

    if( modeName != "shove" && modeName != "walkaround" )
        return KOPENAPI_RESULT::Error( 400, "mode: walkaround (go around obstacles) or shove (push other tracks)" );

    req.mode = modeName == "shove" ? PNS::RM_Shove : PNS::RM_Walkaround;

    for( const nlohmann::json& w : aArgs.value( "waypoints", nlohmann::json::array() ) )
    {
        const nlohmann::json at = w.is_object() ? w.value( "at", nlohmann::json() ) : w;

        if( !at.is_array() || at.size() != 2 || !at[0].is_number() || !at[1].is_number() )
            return KOPENAPI_RESULT::Error( 400, "waypoints: [x_mm, y_mm] (a corner) or {at: [x_mm, y_mm], layer: \"B.Cu\"} "
                                                "(a via there, the route continues on that layer)" );

        std::optional<PCB_LAYER_ID> viaLayer;

        if( w.is_object() && w.contains( "layer" ) )
        {
            viaLayer = w["layer"].is_string() ? parseLayer( board, w["layer"].get<std::string>() ) : std::nullopt;

            if( !viaLayer )
                return KOPENAPI_RESULT::Error( 400, "waypoints: layer is an enabled copper layer" );
        }

        req.vias.emplace_back( VECTOR2I( pcbIUScale.mmToIU( at[0].get<double>() ), pcbIUScale.mmToIU( at[1].get<double>() ) ),
                               viaLayer );
    }

    nlohmann::json    result = { { "from", aArgs["from"] } };
    std::vector<KIID> added;

    if( aArgs.contains( "to" ) )
        result["to"] = aArgs["to"];

    routeOne( *context, req, result, added );

    if( result.value( "applied", false ) )
    {
        showRoutes( aCtx, added );
        result["unrouted_left"] = board->GetConnectivity()->GetUnconnectedCount( false );
    }

    return KOPENAPI_RESULT::Ok( result );
}


/// @brief Route many connections: the board's airwires (all, or those of given nets), shortest first,
/// each from a pad end; a connection that fails is tried again on the other layer, then in shove
/// mode.  One push per accepted route (rollback by steps undoes the whole call: it is one step).
static KOPENAPI_RESULT h_pcb_route( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD* board = context->GetBoard();
    const auto started = std::chrono::steady_clock::now();

    std::set<std::string> nets;

    for( const nlohmann::json& n : aArgs.value( "nets", nlohmann::json::array() ) )
    {
        if( n.is_string() )
            nets.insert( n.get<std::string>() );
    }

    std::vector<PCB_LAYER_ID> layers;

    for( const nlohmann::json& l : aArgs.value( "layers", nlohmann::json::array( { "F.Cu", "B.Cu" } ) ) )
    {
        std::optional<PCB_LAYER_ID> layer = l.is_string() ? parseLayer( board, l.get<std::string>() ) : std::nullopt;

        if( !layer )
            return KOPENAPI_RESULT::Error( 400, "layers: enabled copper layers in order of preference" );

        layers.push_back( *layer );
    }

    const std::string order = aArgs.value( "order", std::string( "short_first" ) );
    const bool        dryRun = aArgs.value( "dry_run", false );
    const int         stepMs = std::clamp( aArgs.value( "step_ms", 0 ), 0, 5000 );
    const std::string style = aArgs.value( "style", std::string( "direct" ) );

    if( style != "direct" && style != "rail" )
        return KOPENAPI_RESULT::Error( 400, "style: direct or rail" );
    size_t            glowFrom = 0;

    // The airwires to route: pad-to-pad (or pad-to-track) connections of the ratsnest
    board->BuildConnectivity();
    std::shared_ptr<CONNECTIVITY_DATA> connectivity = board->GetConnectivity();
    connectivity->RecalculateRatsnest();

    // the ends are found again by id before each job: an earlier route (shove) or the UI kept
    // alive between jobs may have removed or replaced them
    struct JOB
    {
        PAD*                  from;
        BOARD_CONNECTED_ITEM* to;
        KIID                  fromId;
        KIID                  toId;
        VECTOR2I              toPos;
        double                length;
        std::string           net;
        bool                  power;
    };

    std::vector<JOB> jobs;

    for( int code = 1; code < (int) board->GetNetCount(); ++code )
    {
        RN_NET*       rn = connectivity->GetRatsnestForNet( code );
        NETINFO_ITEM* info = board->FindNet( code );

        if( !rn || !info )
            continue;

        const std::string name = str( UnescapeString( info->GetNetname() ) );

        if( !nets.empty() && !nets.count( name ) )
            continue;

        NETCLASS*  netclass = info->GetNetClass();
        const bool power = netclass && netclass->GetName() != NETCLASS::Default;

        for( const CN_EDGE& edge : rn->GetEdges() )
        {
            std::shared_ptr<const CN_ANCHOR> a = edge.GetSourceNode(), b = edge.GetTargetNode();

            if( !a || !b )
                continue;

            BOARD_CONNECTED_ITEM* pa = a->Parent();
            BOARD_CONNECTED_ITEM* pb = b->Parent();

            if( pa->Type() != PCB_PAD_T )
                std::swap( a, b ), std::swap( pa, pb );

            if( pa->Type() != PCB_PAD_T )
                continue;   // track to track: later

            jobs.push_back( { static_cast<PAD*>( pa ), pb, pa->m_Uuid, pb->m_Uuid, b->Pos(),
                              (double) ( b->Pos() - a->Pos() ).EuclideanNorm(), name, power } );
        }
    }

    std::stable_sort( jobs.begin(), jobs.end(),
                      [&]( const JOB& x, const JOB& y )
                      {
                          if( order == "power_first" && x.power != y.power )
                              return x.power;

                          return x.length < y.length;
                      } );

    nlohmann::json    results = nlohmann::json::array();
    std::vector<KIID> added;
    int               routed = 0;
    double            length = 0;
    int               vias = 0;

    for( JOB& job : jobs )
    {
        KopenapiKeepUiAlive();

        job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
        job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

        if( !job.from || !job.to )
        {
            results.push_back( { { "net", job.net }, { "routed", false }, { "tries", 0 },
                                 { "reason", "an end of the connection is gone (removed or replaced by an earlier route)" } } );
            continue;
        }

        nlohmann::json attempt;
        bool           ok = false;
        int            tries = 0;

        // rail style: no direct attempts, every connection leaves its pads through vias and runs
        // on the other layer (power distribution)
        const bool rail = style == "rail";

        // preferred layers in order, walkaround first, then shove
        for( PNS::PNS_MODE mode : { PNS::RM_Walkaround, PNS::RM_Shove } )
        {
            for( PCB_LAYER_ID layer : layers )
            {
                // both ends on the layer (no vias yet)
                if( ok || rail || !job.from->IsOnLayer( layer ) || !job.to->IsOnLayer( layer ) )
                    continue;

                ROUTE_REQUEST req{ job.from, job.to, job.toPos, layer, mode, dryRun, {} };
                attempt = nlohmann::json::object();
                tries++;
                ok = routeOne( *context, req, attempt, added );
            }
        }

        // Both ends on one side only (SMD) and no way on it: escape through vias near each end
        // and cross on the other layer (candidate via spots around the pads, nearest the other
        // end first)
        if( !ok && aArgs.value( "vias", true ) && layers.size() >= 2 )
        {
            auto spots = [&]( BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aPos, const VECTOR2I& aToward )
            {
                std::vector<VECTOR2I> out;
                const BOX2I box = aEnd->GetBoundingBox();
                const int   reach = std::max( box.GetWidth(), box.GetHeight() ) / 2 + pcbIUScale.mmToIU( 1.3 );
                const int   grid = pcbIUScale.mmToIU( 0.25 );

                for( int k = 0; k < 8; ++k )
                {
                    const double   a = k * M_PI / 4;
                    VECTOR2I       p = aPos + VECTOR2I( KiROUND( reach * std::cos( a ) ), KiROUND( reach * std::sin( a ) ) );
                    p = VECTOR2I( KiROUND( (double) p.x / grid ) * grid, KiROUND( (double) p.y / grid ) * grid );
                    out.push_back( p );
                }

                // clear of every hole already there (earlier vias, THT pads): hole-to-hole rule
                const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
                const int                    drill = ds.GetCurrentViaDrill();
                auto                         holeClash = [&]( const VECTOR2I& aAt )
                {
                    for( PCB_TRACK* t : board->Tracks() )
                    {
                        if( t->Type() == PCB_VIA_T
                            && ( t->GetPosition() - aAt ).EuclideanNorm()
                                       < ( static_cast<PCB_VIA*>( t )->GetDrillValue() + drill ) / 2 + ds.m_HoleToHoleMin )
                        {
                            return true;
                        }
                    }

                    for( FOOTPRINT* fp : board->Footprints() )
                    {
                        for( PAD* p : fp->Pads() )
                        {
                            if( p->HasHole()
                                && ( p->GetPosition() - aAt ).EuclideanNorm()
                                           < ( std::max( p->GetDrillSize().x, p->GetDrillSize().y ) + drill ) / 2 + ds.m_HoleToHoleMin )
                            {
                                return true;
                            }
                        }
                    }

                    return false;
                };

                out.erase( std::remove_if( out.begin(), out.end(), holeClash ), out.end() );
                std::sort( out.begin(), out.end(), [&]( const VECTOR2I& x, const VECTOR2I& y )
                           { return ( x - aToward ).EuclideanNorm() < ( y - aToward ).EuclideanNorm(); } );

                if( out.size() > 4 )
                    out.resize( 4 );

                return out;
            };

            const PCB_LAYER_ID top = layers[0], other = layers[1];
            const VECTOR2I     from = job.from->GetPosition();

            for( const VECTOR2I& va : spots( job.from, from, job.toPos ) )
            {
                for( const VECTOR2I& vb : spots( job.to, job.toPos, from ) )
                {
                    if( ok )
                        break;

                    ROUTE_REQUEST req{ job.from, job.to, job.toPos, top, PNS::RM_Walkaround, dryRun, {} };
                    req.vias = { { va, other }, { vb, top } };
                    attempt = nlohmann::json::object();
                    tries++;
                    ok = routeOne( *context, req, attempt, added );
                }
            }
        }

        nlohmann::json row = { { "net", job.net },
                               { "from", str( job.from->GetParentFootprint()->GetReference() ) + "." + str( job.from->GetNumber() ) },
                               { "routed", ok }, { "tries", tries } };

        if( job.to->Type() == PCB_PAD_T )
            row["to"] = str( static_cast<PAD*>( job.to )->GetParentFootprint()->GetReference() ) + "."
                        + str( static_cast<PAD*>( job.to )->GetNumber() );

        // GUI, for watching: each accepted route is drawn before the next one starts
        if( ok && stepMs > 0 && !aCtx.headless && aCtx.kiway )
        {
            if( auto* frame = ROUTE_GLOW_TRAITS::Frame( aCtx.kiway ) )
            {
                std::vector<KIID> mine( added.begin() + glowFrom, added.end() );
                glowFrom = added.size();
                KopenapiGlow<ROUTE_GLOW_TRAITS>( aCtx.kiway, mine, 0 );
                frame->GetCanvas()->Refresh();

                // the UI keeps running (paint, glow, recordings) while the step shows
                const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds( stepMs );

                do
                {
                    KopenapiKeepUiAlive();
                    std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
                } while( std::chrono::steady_clock::now() < until );
            }
        }

        if( ok )
        {
            routed++;
            length += attempt["metrics"].value( "length_mm", 0.0 );
            vias += attempt["metrics"].value( "vias", 0 );
            row["layer"] = attempt["layer"];
            row["mode"] = attempt["mode"];
            row["length_mm"] = attempt["metrics"]["length_mm"];
            row["detour"] = attempt["metrics"]["detour"];
        }
        else
        {
            row["reason"] = attempt.value( "reason", std::string() );

            if( attempt.contains( "violations" ) )
                row["violations"] = attempt["violations"];
        }

        results.push_back( row );
    }

    if( stepMs == 0 )
        showRoutes( aCtx, added );

    connectivity->RecalculateRatsnest();

    return KOPENAPI_RESULT::Ok( { { "connections", jobs.size() },
                                  { "routed", routed },
                                  { "failed", (int) jobs.size() - routed },
                                  { "length_mm", std::round( length * 1000.0 ) / 1000.0 },
                                  { "vias", vias },
                                  { "dry_run", dryRun },
                                  { "unrouted_left", dryRun ? (int) jobs.size() - routed : (int) connectivity->GetUnconnectedCount( false ) },
                                  { "ms", std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() },
                                  { "results", results } } );
}


/// @brief Stitch a net's loose ends to its pour on another layer: for every open connection of the net,
/// an end on one side only (an SMD pad, an island of a zone) gets a through via at a free spot
/// next to it (and a short track from a pad); each via passes the net-merge guard and must land
/// inside the board; zones are refilled at the end.
static KOPENAPI_RESULT h_pcb_stitch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*        board = context->GetBoard();
    const wxString netName = wxString::FromUTF8( aArgs.value( "net", std::string( "GND" ) ) );
    NETINFO_ITEM* net = board->FindNet( netName );

    if( !net )
        return KOPENAPI_RESULT::Error( 404, "net not found: " + str( netName ) );

    NETCLASS*  netclass = net->GetNetClass();
    const int  viaDia = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : pcbIUScale.mmToIU( 0.8 );
    const int  viaDrill = netclass && netclass->HasViaDrill() ? netclass->GetViaDrill() : pcbIUScale.mmToIU( 0.4 );
    const int  width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : pcbIUScale.mmToIU( 0.3 );

    SHAPE_POLY_SET boardShape;
    board->GetBoardPolygonOutlines( boardShape, true );
    const int edgeClearance = board->GetDesignSettings().m_CopperEdgeClearance;

    // where a via may go: inside the outline, its copper clear of the edge
    SHAPE_POLY_SET inner = boardShape;

    if( inner.OutlineCount() )
        inner.Deflate( edgeClearance + viaDia / 2, CORNER_STRATEGY::ROUND_ALL_CORNERS, ARC_HIGH_DEF );

    auto refill = [&]()
    {
        BOARD_COMMIT fillCommit( context->GetToolManager() );
        ZONE_FILLER  filler( board, &fillCommit );
        KOPENAPI_KEEPALIVE_REPORTER reporter;
        filler.SetProgressReporter( &reporter );
        std::vector<ZONE*> zones( board->Zones().begin(), board->Zones().end() );

        if( !zones.empty() && filler.Fill( zones ) )
            fillCommit.Push( _( "Fill zones (API)" ) );
        else
            fillCommit.Revert();

        board->BuildConnectivity();
        board->GetConnectivity()->RecalculateRatsnest();
    };

    refill();

    std::vector<KIID> added;
    nlohmann::json    placed = nlohmann::json::array();
    std::set<BOARD_CONNECTED_ITEM*> done;

    // Loose ends: anchors of the net's open connections that sit on one layer only
    std::vector<std::pair<BOARD_CONNECTED_ITEM*, VECTOR2I>> ends;

    if( RN_NET* rn = board->GetConnectivity()->GetRatsnestForNet( net->GetNetCode() ) )
    {
        for( const CN_EDGE& edge : rn->GetEdges() )
        {
            for( const std::shared_ptr<const CN_ANCHOR>& anchor : { edge.GetSourceNode(), edge.GetTargetNode() } )
            {
                if( !anchor )
                    continue;

                BOARD_CONNECTED_ITEM* item = anchor->Parent();

                if( item->GetLayerSet().CuStack().size() == 1 || item->Type() == PCB_ZONE_T )
                    ends.emplace_back( item, anchor->Pos() );
            }
        }
    }

    const int step = pcbIUScale.mmToIU( 0.25 );

    for( const auto& [item, pos] : ends )
    {
        KopenapiKeepUiAlive();

        if( done.count( item ) && item->Type() != PCB_ZONE_T )
            continue;

        const PCB_LAYER_ID layer = item->Type() == PCB_ZONE_T ? item->GetLayerSet().CuStack().front()
                                                                : item->GetLayerSet().CuStack().front();
        const BOX2I box = item->Type() == PCB_ZONE_T ? BOX2I( pos, VECTOR2I( 0, 0 ) ) : item->GetBoundingBox();
        bool        ok = false;

        // Candidate spots, nearest first: around a pad; inside a zone island (on a grid over the
        // island, the via's copper fully in it)
        std::vector<VECTOR2I> candidates;

        if( item->Type() == PCB_ZONE_T )
        {
            ZONE* zone = static_cast<ZONE*>( item );

            for( PCB_LAYER_ID l : zone->GetLayerSet().CuStack() )
            {
                if( !zone->HasFilledPolysForLayer( l ) )
                    continue;

                const std::shared_ptr<SHAPE_POLY_SET>& fill = zone->GetFilledPolysList( l );

                for( int o = 0; o < fill->OutlineCount(); ++o )
                {
                    SHAPE_POLY_SET island;
                    island.AddOutline( fill->COutline( o ) );

                    for( int h = 0; h < fill->HoleCount( o ); ++h )
                        island.AddHole( fill->CHole( o, h ) );

                    if( !island.Contains( pos, -1, pcbIUScale.mmToIU( 0.05 ) ) )
                        continue;   // the island this anchor belongs to

                    SHAPE_POLY_SET room = island;
                    room.Deflate( viaDia / 2, CORNER_STRATEGY::ROUND_ALL_CORNERS, ARC_HIGH_DEF );

                    const BOX2I ib = island.BBox();

                    for( int x = ib.GetLeft(); x <= ib.GetRight(); x += step )
                    {
                        for( int y = ib.GetTop(); y <= ib.GetBottom(); y += step )
                        {
                            if( room.Contains( VECTOR2I( x, y ) ) )
                                candidates.emplace_back( x, y );
                        }
                    }
                }
            }
        }
        else
        {
            for( int ring = 0; ring < 6; ++ring )
            {
                const int reach = std::max( box.GetWidth(), box.GetHeight() ) / 2 + viaDia / 2 + pcbIUScale.mmToIU( 0.4 )
                                  + ring * pcbIUScale.mmToIU( 0.5 );

                for( int k = 0; k < 16; ++k )
                {
                    const double a = k * M_PI / 8;
                    VECTOR2I     at = pos + VECTOR2I( KiROUND( reach * std::cos( a ) ), KiROUND( reach * std::sin( a ) ) );
                    candidates.emplace_back( KiROUND( (double) at.x / step ) * step, KiROUND( (double) at.y / step ) * step );
                }
            }
        }

        std::stable_sort( candidates.begin(), candidates.end(), [&]( const VECTOR2I& x, const VECTOR2I& y )
                          { return ( x - pos ).EuclideanNorm() < ( y - pos ).EuclideanNorm(); } );

        if( candidates.size() > 400 )
            candidates.resize( 400 );

        {
            for( const VECTOR2I& at : candidates )
            {
                if( ok )
                    break;

                if( inner.OutlineCount() && !inner.Contains( at ) )
                    continue;   // too close to the board edge

                // keep clear of every hole (vias placed earlier, THT pads): hole-to-hole rule
                bool holeClash = false;
                const int webMin = board->GetDesignSettings().m_HoleToHoleMin;

                for( PCB_TRACK* t : board->Tracks() )
                {
                    if( t->Type() == PCB_VIA_T )
                    {
                        PCB_VIA* v = static_cast<PCB_VIA*>( t );
                        holeClash |= ( v->GetPosition() - at ).EuclideanNorm() < ( v->GetDrillValue() + viaDrill ) / 2 + webMin;
                    }
                }

                for( FOOTPRINT* fp : board->Footprints() )
                {
                    for( PAD* p : fp->Pads() )
                    {
                        if( p->HasHole() )
                            holeClash |= ( p->GetPosition() - at ).EuclideanNorm()
                                         < ( std::max( p->GetDrillSize().x, p->GetDrillSize().y ) + viaDrill ) / 2 + webMin;
                    }
                }

                if( holeClash )
                    continue;

                auto via = std::make_unique<PCB_VIA>( board );
                via->SetPosition( at );
                via->SetViaType( VIATYPE::THROUGH );
                via->SetLayerPair( F_Cu, B_Cu );
                via->SetWidth( viaDia );
                via->SetDrill( viaDrill );
                via->SetNet( net );

                std::unique_ptr<PCB_TRACK> track;

                if( item->Type() == PCB_PAD_T )
                {
                    track = std::make_unique<PCB_TRACK>( board );
                    track->SetStart( pos );
                    track->SetEnd( at );
                    track->SetWidth( width );
                    track->SetLayer( layer );
                    track->SetNet( net );
                }

                std::vector<BOARD_CONNECTED_ITEM*> fresh = { via.get() };

                if( track )
                    fresh.push_back( track.get() );

                if( !guard( board, fresh, {} ).empty() )
                    continue;

                BOARD_COMMIT commit( context->GetToolManager() );
                added.push_back( via->m_Uuid );
                commit.Add( via.release() );

                if( track )
                {
                    added.push_back( track->m_Uuid );
                    commit.Add( track.release() );
                }

                commit.Push( _( "Stitching via (API)" ) );
                placed.push_back( { { "at_mm", { toMm( at.x ), toMm( at.y ) } },
                                    { "for", item->Type() == PCB_PAD_T
                                                     ? str( static_cast<PAD*>( item )->GetParentFootprint()->GetReference() ) + "."
                                                               + str( static_cast<PAD*>( item )->GetNumber() )
                                                     : std::string( "zone island" ) } } );
                done.insert( item );
                ok = true;
            }
        }
    }

    refill();
    showRoutes( aCtx, added );

    return KOPENAPI_RESULT::Ok( { { "net", str( netName ) },
                                  { "vias", placed },
                                  { "unrouted_left", (int) board->GetConnectivity()->GetUnconnectedCount( false ) } } );
}


namespace
{

nlohmann::json trackJson( BOARD* aBoard, PCB_TRACK* aTrack )
{
    nlohmann::json row = { { "uuid", str( aTrack->m_Uuid.AsString() ) },
                           { "net", str( UnescapeString( aTrack->GetNetname() ) ) } };

    if( aTrack->Type() == PCB_VIA_T )
    {
        PCB_VIA* via = static_cast<PCB_VIA*>( aTrack );
        row["kind"] = "via";
        row["at_mm"] = { toMm( via->GetPosition().x ), toMm( via->GetPosition().y ) };
        row["layers"] = { str( aBoard->GetLayerName( via->TopLayer() ) ), str( aBoard->GetLayerName( via->BottomLayer() ) ) };
        row["diameter_mm"] = toMm( via->GetWidth( via->TopLayer() ) );
        row["drill_mm"] = toMm( via->GetDrillValue() );
    }
    else
    {
        row["kind"] = aTrack->Type() == PCB_ARC_T ? "arc" : "track";
        row["from_mm"] = { toMm( aTrack->GetStart().x ), toMm( aTrack->GetStart().y ) };
        row["to_mm"] = { toMm( aTrack->GetEnd().x ), toMm( aTrack->GetEnd().y ) };
        row["layer"] = str( aBoard->GetLayerName( aTrack->GetLayer() ) );
        row["width_mm"] = toMm( aTrack->GetWidth() );
        row["length_mm"] = toMm( aTrack->GetLength() );
    }

    if( aTrack->IsLocked() )
        row["locked"] = true;

    return row;
}


/// @brief The tracks and vias that join two pads of one net: the shortest chain of items touching end
/// to end (through other pads of the net too); empty when the pads are not joined by copper
std::vector<PCB_TRACK*> copperBetween( BOARD* aBoard, PAD* aFrom, PAD* aTo )
{
    std::vector<PCB_TRACK*> items;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->GetNetCode() == aFrom->GetNetCode() )
            items.push_back( track );
    }

    auto ends = []( PCB_TRACK* aTrack ) -> std::vector<VECTOR2I>
    {
        if( aTrack->Type() == PCB_VIA_T )
            return { aTrack->GetPosition() };

        return { aTrack->GetStart(), aTrack->GetEnd() };
    };

    // items by end point; pads of the net with the items ending on them
    std::map<std::pair<int, int>, std::vector<size_t>> atPoint;

    for( size_t i = 0; i < items.size(); i++ )
    {
        for( const VECTOR2I& e : ends( items[i] ) )
            atPoint[{ e.x, e.y }].push_back( i );
    }

    auto touches = []( PAD* aPad, PCB_TRACK* aTrack, const VECTOR2I& aEnd )
    {
        return ( aPad->GetLayerSet() & aTrack->GetLayerSet() ).any() && aPad->HitTest( aEnd, 0 );
    };

    std::vector<std::pair<PAD*, std::vector<size_t>>> pads;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() != aFrom->GetNetCode() )
                continue;

            pads.emplace_back( pad, std::vector<size_t>() );

            for( size_t i = 0; i < items.size(); i++ )
            {
                for( const VECTOR2I& e : ends( items[i] ) )
                {
                    if( touches( pad, items[i], e ) )
                    {
                        pads.back().second.push_back( i );
                        break;
                    }
                }
            }
        }
    }

    std::vector<std::vector<size_t>> padsOf( items.size() );

    for( size_t p = 0; p < pads.size(); p++ )
    {
        for( size_t i : pads[p].second )
            padsOf[i].push_back( p );
    }

    const size_t        none = items.size();
    std::vector<size_t> parent( items.size(), none );
    std::vector<bool>   seen( items.size(), false );
    std::deque<size_t>  queue;
    std::set<size_t>    goal;

    for( const auto& [pad, list] : pads )
    {
        if( pad == aFrom )
        {
            for( size_t i : list )
            {
                seen[i] = true;
                queue.push_back( i );
            }
        }
        else if( pad == aTo )
        {
            goal.insert( list.begin(), list.end() );
        }
    }

    while( !queue.empty() )
    {
        const size_t at = queue.front();
        queue.pop_front();

        if( goal.count( at ) )
        {
            std::vector<PCB_TRACK*> path;

            for( size_t i = at; i != none; i = parent[i] )
                path.push_back( items[i] );

            return path;
        }

        auto visit = [&]( size_t aNext )
        {
            if( !seen[aNext] && ( items[aNext]->GetLayerSet() & items[at]->GetLayerSet() ).any() )
            {
                seen[aNext] = true;
                parent[aNext] = at;
                queue.push_back( aNext );
            }
        };

        for( const VECTOR2I& e : ends( items[at] ) )
        {
            for( size_t next : atPoint[{ e.x, e.y }] )
                visit( next );
        }

        // through a pad of the net (not the two asked for): everything else ending on it
        for( size_t p : padsOf[at] )
        {
            if( pads[p].first == aFrom || pads[p].first == aTo )
                continue;

            for( size_t next : pads[p].second )
            {
                if( !seen[next] )
                {
                    seen[next] = true;
                    parent[next] = at;
                    queue.push_back( next );
                }
            }
        }
    }

    return {};
}


std::optional<BOX2I> regionOf( const nlohmann::json& aArgs )
{
    if( !aArgs.contains( "region" ) )
        return std::nullopt;

    const nlohmann::json& r = aArgs["region"];

    if( !r.is_array() || r.size() != 4 || !r[0].is_number() || !r[1].is_number() || !r[2].is_number() || !r[3].is_number() )
        return std::nullopt;

    BOX2I box( VECTOR2I( pcbIUScale.mmToIU( r[0].get<double>() ), pcbIUScale.mmToIU( r[1].get<double>() ) ), VECTOR2I( 0, 0 ) );
    box.SetEnd( VECTOR2I( pcbIUScale.mmToIU( r[2].get<double>() ), pcbIUScale.mmToIU( r[3].get<double>() ) ) );
    box.Normalize();
    return box;
}


/// @brief The tracks / vias a request names: uuids, or the ones passing every filter given (nets, between,
/// region, layer, kinds).  aError is set when the request is wrong.
std::vector<PCB_TRACK*> selectTracks( BOARD* aBoard, const nlohmann::json& aArgs, bool aNeedSelector, nlohmann::json& aMissing,
                                      std::optional<KOPENAPI_RESULT>& aError )
{
    std::vector<PCB_TRACK*> out;

    if( aArgs.contains( "uuids" ) )
    {
        if( !aArgs["uuids"].is_array() )
        {
            aError = KOPENAPI_RESULT::Error( 400, "uuids: track / via uuids (pcb_track_list)" );
            return out;
        }

        std::map<std::string, PCB_TRACK*> byId;

        for( PCB_TRACK* track : aBoard->Tracks() )
            byId[str( track->m_Uuid.AsString() )] = track;

        for( const nlohmann::json& u : aArgs["uuids"] )
        {
            auto it = u.is_string() ? byId.find( u.get<std::string>() ) : byId.end();

            if( it == byId.end() )
                aMissing.push_back( u );
            else
                out.push_back( it->second );
        }

        return out;
    }

    std::set<int> nets;

    if( aArgs.contains( "net" ) && !aArgs.contains( "nets" ) && !aArgs["net"].is_string() )
    {
        aError = KOPENAPI_RESULT::Error( 400, "net: a net name" );
        return out;
    }

    nlohmann::json netNames = aArgs.value( "nets", nlohmann::json::array() );

    if( aArgs.contains( "net" ) && aArgs["net"].is_string() )
        netNames.push_back( aArgs["net"] );

    for( const nlohmann::json& n : netNames )
    {
        NETINFO_ITEM* net = n.is_string() ? aBoard->FindNet( wxString::FromUTF8( n.get<std::string>() ) ) : nullptr;

        if( !net )
        {
            aError = KOPENAPI_RESULT::Error( 404, "no such net: " + n.dump() + " (pcb_net_list)" );
            return out;
        }

        nets.insert( net->GetNetCode() );
    }

    std::optional<std::set<PCB_TRACK*>> between;

    if( aArgs.contains( "between" ) )
    {
        const nlohmann::json& b = aArgs["between"];
        std::optional<PAD*>   a = b.is_array() && b.size() == 2 && b[0].is_string() ? findPad( aBoard, b[0].get<std::string>() )
                                                                                    : std::nullopt;
        std::optional<PAD*>   z = a && b[1].is_string() ? findPad( aBoard, b[1].get<std::string>(), *a, *a ) : std::nullopt;

        if( !a || !z )
        {
            aError = KOPENAPI_RESULT::Error( 404, "between: two pads [\"REF.PAD\", \"REF.PAD\"]" );
            return out;
        }

        if( ( *a )->GetNetCode() != ( *z )->GetNetCode() || ( *a )->GetNetCode() <= 0 )
        {
            aError = KOPENAPI_RESULT::Error( 422, "between: the two pads are not on one net" );
            return out;
        }

        const std::vector<PCB_TRACK*> path = copperBetween( aBoard, *a, *z );
        between = std::set<PCB_TRACK*>( path.begin(), path.end() );
    }

    if( aArgs.contains( "region" ) && !regionOf( aArgs ) )
    {
        aError = KOPENAPI_RESULT::Error( 400, "region: [x0_mm, y0_mm, x1_mm, y1_mm]" );
        return out;
    }

    const std::optional<BOX2I> region = regionOf( aArgs );
    const bool                 touching = aArgs.value( "region_mode", std::string( "inside" ) ) == "touching";
    std::optional<PCB_LAYER_ID> layer;

    if( aArgs.contains( "layer" ) )
    {
        layer = aArgs["layer"].is_string() ? parseLayer( aBoard, aArgs["layer"].get<std::string>() ) : std::nullopt;

        if( !layer )
        {
            aError = KOPENAPI_RESULT::Error( 400, "layer: an enabled copper layer" );
            return out;
        }
    }

    std::set<std::string> kinds;

    for( const nlohmann::json& k : aArgs.value( "kinds", nlohmann::json::array() ) )
    {
        if( !k.is_string() || ( k != "track" && k != "via" ) )
        {
            aError = KOPENAPI_RESULT::Error( 400, "kinds: track, via" );
            return out;
        }

        kinds.insert( k.get<std::string>() );
    }

    if( aNeedSelector && nets.empty() && !between && !region )
    {
        aError = KOPENAPI_RESULT::Error( 400, "say which copper: uuids, net / nets, between [\"REF.PAD\", \"REF.PAD\"] or "
                                              "region [x0, y0, x1, y1] (layer and kinds narrow it further)" );
        return out;
    }

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        const bool isVia = track->Type() == PCB_VIA_T;

        if( !nets.empty() && !nets.count( track->GetNetCode() ) )
            continue;

        if( between && !between->count( track ) )
            continue;

        if( !kinds.empty() && !kinds.count( isVia ? "via" : "track" ) )
            continue;

        if( layer && !track->IsOnLayer( *layer ) )
            continue;

        if( region )
        {
            const BOX2I box = track->GetBoundingBox();

            if( touching ? !region->Intersects( box ) : !region->Contains( box ) )
                continue;
        }

        out.push_back( track );
    }

    return out;
}

} // namespace


std::vector<PCB_TRACK*> KopenapiSelectTracks( BOARD* aBoard, const nlohmann::json& aArgs, bool aNeedSelector,
                                              nlohmann::json& aMissing, std::optional<KOPENAPI_RESULT>& aError )
{
    return selectTracks( aBoard, aArgs, aNeedSelector, aMissing, aError );
}


static KOPENAPI_RESULT h_pcb_track_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                         board = context->GetBoard();
    nlohmann::json                 missing = nlohmann::json::array();
    std::optional<KOPENAPI_RESULT> error;
    std::vector<PCB_TRACK*>        tracks = selectTracks( board, aArgs, false, missing, error );

    if( error )
        return *error;

    const size_t   limit = size_t( std::clamp( aArgs.value( "limit", 500 ), 1, 5000 ) );
    const size_t   offset = size_t( std::max( 0, aArgs.value( "offset", 0 ) ) );
    nlohmann::json items = nlohmann::json::array();
    double         length = 0;
    int            vias = 0;

    for( size_t i = 0; i < tracks.size(); i++ )
    {
        if( tracks[i]->Type() == PCB_VIA_T )
            vias++;
        else
            length += tracks[i]->GetLength();

        if( i >= offset && items.size() < limit )
            items.push_back( trackJson( board, tracks[i] ) );
    }

    nlohmann::json out = { { "items", items }, { "total", tracks.size() }, { "vias", vias },
                           { "length_mm", toMm( length ) }, { "missing", missing } };

    if( offset + items.size() < tracks.size() )
        out["next_offset"] = offset + items.size();

    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_tracks_delete( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD*                         board = context->GetBoard();
    nlohmann::json                 missing = nlohmann::json::array();
    std::optional<KOPENAPI_RESULT> error;
    std::vector<PCB_TRACK*>        selected = selectTracks( board, aArgs, true, missing, error );

    if( error )
        return *error;

    const bool               overrideLocks = aArgs.value( "override_locks", false );
    const bool               dryRun = aArgs.value( "dry_run", false );
    std::vector<PCB_TRACK*>  doomed;
    nlohmann::json           locked = nlohmann::json::array();
    std::map<std::string, nlohmann::json> byNet;
    double                   length = 0;
    int                      tracks = 0, vias = 0;

    for( PCB_TRACK* track : selected )
    {
        if( track->IsLocked() && !overrideLocks )
        {
            locked.push_back( str( track->m_Uuid.AsString() ) );
            continue;
        }

        doomed.push_back( track );

        nlohmann::json& net = byNet[str( UnescapeString( track->GetNetname() ) )];

        if( net.is_null() )
            net = { { "tracks", 0 }, { "vias", 0 }, { "length_mm", 0.0 } };

        if( track->Type() == PCB_VIA_T )
        {
            vias++;
            net["vias"] = net["vias"].get<int>() + 1;
        }
        else
        {
            tracks++;
            length += track->GetLength();
            net["tracks"] = net["tracks"].get<int>() + 1;
            net["length_mm"] = net["length_mm"].get<double>() + toMm( track->GetLength() );
        }
    }

    nlohmann::json out = { { "deleted", { { "tracks", tracks }, { "vias", vias }, { "length_mm", toMm( length ) } } },
                           { "by_net", byNet }, { "missing", missing }, { "locked_skipped", locked },
                           { "applied", false } };

    if( doomed.empty() )
    {
        out["reason"] = !locked.empty() ? "everything selected is locked (override_locks: true)"
                        : aArgs.contains( "between" ) ? "no copper joins these two pads"
                                                      : "nothing matches";
        return KOPENAPI_RESULT::Ok( out );
    }

    if( dryRun )
    {
        nlohmann::json uuids = nlohmann::json::array();

        for( PCB_TRACK* track : doomed )
            uuids.push_back( str( track->m_Uuid.AsString() ) );

        out["uuids"] = uuids;
        return KOPENAPI_RESULT::Ok( out );
    }

    BOARD_COMMIT commit( context->GetToolManager() );

    for( PCB_TRACK* track : doomed )
        commit.Remove( track );

    commit.Push( _( "Delete tracks (API)" ) );

    bool filledZones = false;

    for( ZONE* zone : board->Zones() )
        filledZones |= zone->IsFilled();

    if( filledZones && aArgs.value( "refill_zones", false ) )
    {
        BOARD_COMMIT fillCommit( context->GetToolManager() );
        ZONE_FILLER  filler( board, &fillCommit );
        KOPENAPI_KEEPALIVE_REPORTER reporter;
        filler.SetProgressReporter( &reporter );
        std::vector<ZONE*> zones( board->Zones().begin(), board->Zones().end() );

        if( filler.Fill( zones ) )
            fillCommit.Push( _( "Fill zones (API)" ) );
        else
            fillCommit.Revert();

        out["zones_refilled"] = true;
    }
    else if( filledZones )
    {
        out["note"] = "zone fills still show the removed copper's clearance: pcb_zones_fill (or refill_zones: true)";
    }

    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();
    showRoutes( aCtx, {} );

    out["applied"] = true;
    out["unrouted_left"] = board->GetConnectivity()->GetUnconnectedCount( false );
    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_track_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD* board = context->GetBoard();

    // the path: a pad, points, a pad; a point may carry "via": the layer the track continues on
    struct STEP
    {
        VECTOR2I                    at;
        std::optional<PCB_LAYER_ID> via;
    };

    std::vector<STEP> steps;
    NETINFO_ITEM*     net = nullptr;
    PAD*              fromPad = nullptr;
    PAD*              toPad = nullptr;

    if( aArgs.contains( "from" ) )
    {
        std::optional<PAD*> pad = aArgs["from"].is_string() ? findPad( board, aArgs["from"].get<std::string>() ) : std::nullopt;

        if( !pad )
            return KOPENAPI_RESULT::Error( 404, "from: REF.PAD of an existing pad" );

        fromPad = *pad;
        net = fromPad->GetNet();
        steps.push_back( { fromPad->GetPosition(), std::nullopt } );
    }

    if( aArgs.contains( "net" ) )
    {
        NETINFO_ITEM* named = aArgs["net"].is_string() ? board->FindNet( wxString::FromUTF8( aArgs["net"].get<std::string>() ) )
                                                       : nullptr;

        if( !named )
            return KOPENAPI_RESULT::Error( 404, "no such net (pcb_net_list)" );

        if( net && net != named )
            return KOPENAPI_RESULT::Error( 422, "'from' is on net " + str( UnescapeString( net->GetNetname() ) ) + ", not on " + aArgs["net"].get<std::string>() );

        net = named;
    }

    if( !net || net->GetNetCode() <= 0 )
        return KOPENAPI_RESULT::Error( 400, "give the net: 'from' (REF.PAD on a net) or 'net'" );

    for( const nlohmann::json& p : aArgs.value( "points", nlohmann::json::array() ) )
    {
        const nlohmann::json at = p.is_object() ? p.value( "at", nlohmann::json() ) : p;

        if( !at.is_array() || at.size() != 2 || !at[0].is_number() || !at[1].is_number() )
            return KOPENAPI_RESULT::Error( 400, "points: [x_mm, y_mm] or {at: [x_mm, y_mm], via: \"B.Cu\"} (a via there, "
                                                "the track continues on that layer)" );

        STEP step{ VECTOR2I( pcbIUScale.mmToIU( at[0].get<double>() ), pcbIUScale.mmToIU( at[1].get<double>() ) ), std::nullopt };

        if( p.is_object() && p.contains( "via" ) )
        {
            step.via = p["via"].is_string() ? parseLayer( board, p["via"].get<std::string>() ) : std::nullopt;

            if( !step.via )
                return KOPENAPI_RESULT::Error( 400, "points: via is an enabled copper layer" );
        }

        steps.push_back( step );
    }

    if( aArgs.contains( "to" ) )
    {
        std::optional<PAD*> pad = aArgs["to"].is_string() ? findPad( board, aArgs["to"].get<std::string>(), fromPad, fromPad )
                                                          : std::nullopt;

        if( !pad )
            return KOPENAPI_RESULT::Error( 404, "to: REF.PAD of an existing pad" );

        if( ( *pad )->GetNetCode() != net->GetNetCode() )
            return KOPENAPI_RESULT::Error( 422, "'to' is not on net " + str( UnescapeString( net->GetNetname() ) ) );

        toPad = *pad;
        steps.push_back( { toPad->GetPosition(), std::nullopt } );
    }

    if( steps.size() < 2 )
        return KOPENAPI_RESULT::Error( 400, "a track needs two points at least: from / points / to" );

    PCB_LAYER_ID layer = F_Cu;

    if( aArgs.contains( "layer" ) )
    {
        std::optional<PCB_LAYER_ID> named = aArgs["layer"].is_string() ? parseLayer( board, aArgs["layer"].get<std::string>() )
                                                                       : std::nullopt;

        if( !named )
            return KOPENAPI_RESULT::Error( 400, "layer: an enabled copper layer, e.g. F.Cu, B.Cu" );

        layer = *named;
    }
    else if( fromPad && !fromPad->IsOnLayer( F_Cu ) )
    {
        layer = B_Cu;   // an SMD pad on the bottom
    }

    if( fromPad && !fromPad->IsOnLayer( layer ) )
        return KOPENAPI_RESULT::Error( 422, "'from' pad is not on " + str( board->GetLayerName( layer ) ) );

    NETCLASS*  netclass = net->GetNetClass();
    const BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    const int  width = aArgs.contains( "width_mm" ) && aArgs["width_mm"].is_number()
                               ? pcbIUScale.mmToIU( aArgs["width_mm"].get<double>() )
                               : netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : pcbIUScale.mmToIU( 0.25 );
    const int  viaDia = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : pcbIUScale.mmToIU( 0.8 );
    const int  viaDrill = netclass && netclass->HasViaDrill() ? netclass->GetViaDrill() : pcbIUScale.mmToIU( 0.4 );

    if( width < bds.m_TrackMinWidth )
        return KOPENAPI_RESULT::Error( 422, "width_mm is below the board's minimum track width "
                                                    + std::to_string( toMm( bds.m_TrackMinWidth ) ).substr( 0, 5 ) + " mm" );

    std::vector<std::unique_ptr<PCB_TRACK>> made;
    std::vector<BOARD_CONNECTED_ITEM*>      changed;
    double                                  length = 0;
    int                                     vias = 0;
    nlohmann::json                          segments = nlohmann::json::array();

    for( size_t i = 1; i < steps.size(); i++ )
    {
        if( steps[i].at != steps[i - 1].at )
        {
            auto track = std::make_unique<PCB_TRACK>( board );
            track->SetStart( steps[i - 1].at );
            track->SetEnd( steps[i].at );
            track->SetWidth( width );
            track->SetLayer( layer );
            track->SetNet( net );
            length += track->GetLength();
            segments.push_back( { { "from", { toMm( track->GetStart().x ), toMm( track->GetStart().y ) } },
                                  { "to", { toMm( track->GetEnd().x ), toMm( track->GetEnd().y ) } },
                                  { "layer", str( board->GetLayerName( layer ) ) }, { "width_mm", toMm( width ) } } );
            changed.push_back( track.get() );
            made.push_back( std::move( track ) );
        }

        if( steps[i].via && *steps[i].via != layer )
        {
            auto via = std::make_unique<PCB_VIA>( board );
            via->SetViaType( VIATYPE::THROUGH );
            via->SetPosition( steps[i].at );
            via->SetWidth( PADSTACK::ALL_LAYERS, viaDia );
            via->SetDrill( viaDrill );
            via->SetLayerPair( F_Cu, B_Cu );
            via->SetNet( net );
            vias++;
            changed.push_back( via.get() );
            made.push_back( std::move( via ) );
            layer = *steps[i].via;
        }
    }

    if( made.empty() )
        return KOPENAPI_RESULT::Error( 400, "the points are all the same: nothing to add" );

    if( toPad && !toPad->IsOnLayer( layer ) )
        return KOPENAPI_RESULT::Error( 422, "the track arrives on " + str( board->GetLayerName( layer ) ) + ", 'to' pad is not on it "
                                                    "(add a point with via before it)" );

    nlohmann::json out = { { "net", str( UnescapeString( net->GetNetname() ) ) }, { "segments", segments },
                           { "length_mm", toMm( length ) }, { "vias", vias }, { "width_mm", toMm( width ) },
                           { "applied", false } };

    const nlohmann::json violations = guard( board, changed, {} );

    if( !violations.empty() )
    {
        KOPENAPI_RESULT rejected = KOPENAPI_RESULT::Error( 422, "rejected by the guard: this copper would touch another net" );
        rejected.body["violations"] = violations;
        rejected.body["segments"] = segments;
        return rejected;
    }

    if( aArgs.value( "dry_run", false ) )
        return KOPENAPI_RESULT::Ok( out );

    BOARD_COMMIT      commit( context->GetToolManager() );
    std::vector<KIID> added;
    nlohmann::json    uuids = nlohmann::json::array();

    for( std::unique_ptr<PCB_TRACK>& item : made )
    {
        added.push_back( item->m_Uuid );
        uuids.push_back( str( item->m_Uuid.AsString() ) );
        commit.Add( item.release() );
    }

    commit.Push( _( "Add track (API)" ) );
    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();
    showRoutes( aCtx, added );

    out["applied"] = true;
    out["uuids"] = uuids;
    out["unrouted_left"] = board->GetConnectivity()->GetUnconnectedCount( false );
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_route_connection",
                   "EXPERIMENTAL. Route one connection with KiCad's push-and-shove router, as a person "
                   "does in the editor: from a pad (REF.PAD) to a pad of the same net, or to the nearest "
                   "unconnected point of its net; layer, mode walkaround / shove; track width and clearance "
                   "from the board's rules; waypoints steer it (corners, vias with a layer change). A guard "
                   "rejects routes whose copper would touch another net. Undo one route: pcb_tracks_delete "
                   "(between its pads). "
                   "dry_run: measure only (geometry, length, detour, vias, shoved items), nothing changes",
                   R"json({"type":"object","required":["from"],"properties":{
                        "from":{"type":"string","description":"REF.PAD"},
                        "to":{"type":"string","description":"REF.PAD on the same net; default: nearest unconnected"},
                        "layer":{"type":"string","default":"F.Cu"},
                        "mode":{"type":"string","enum":["walkaround","shove"],"default":"walkaround"},
                        "waypoints":{"type":"array","description":"points to pass in order: [x_mm, y_mm] fixes a corner there; {at: [x_mm, y_mm], layer: 'B.Cu'} drops a through via there and continues on that layer","items":{}},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_route_connection, 120 );

KOPENAPI_REGISTER( "pcb_route",
                   "EXPERIMENTAL. Route the board's airwires (all, or of given nets) with KiCad's "
                   "push-and-shove router, one connection after another (order short_first or "
                   "power_first); a failed connection is tried on the next layer, then in shove mode; "
                   "every route passes the net-merge guard; per connection routed / reason, totals "
                   "(routed, length, vias, unrouted left); dry_run routes nothing",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"string"}},
                        "layers":{"type":"array","items":{"type":"string"},"default":["F.Cu","B.Cu"]},
                        "order":{"type":"string","enum":["short_first","power_first"],"default":"short_first"},
                        "vias":{"type":"boolean","default":true,"description":"connections blocked on their side: escape through vias near both ends"},
                        "style":{"type":"string","enum":["direct","rail"],"default":"direct","description":"rail (power nets): every connection leaves its pads through a via next to them and runs on the second layer"},
                        "step_ms":{"type":"integer","default":0,"description":"GUI: draw each route and wait this long before the next (for watching / recording)"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_route, 600 );


KOPENAPI_REGISTER( "pcb_stitch",
                   "Stitch a net (default GND) to its pour on the other layer: every open end on one side "
                   "only (SMD pad, zone island) gets a through via at a free spot next to it (with a short "
                   "track from a pad), guarded against touching other nets, inside the board; zones are "
                   "refilled; answers the vias and the connections still open",
                   R"json({"type":"object","properties":{"net":{"type":"string","default":"GND"}}})json"_json,
                   false, h_pcb_stitch, 300 );

KOPENAPI_REGISTER( "pcb_track_list",
                   "Tracks and vias with their uuids: all, or of nets, in a region, on a layer, or the copper "
                   "joining two pads (between); per item kind, net, ends / position, layer(s), width, length; "
                   "totals (count, vias, length)",
                   R"json({"type":"object","properties":{
                        "net":{"type":"string"},"nets":{"type":"array","items":{"type":"string"}},
                        "between":{"type":"array","items":{"type":"string"},"description":"[\"REF.PAD\", \"REF.PAD\"]: the chain of tracks / vias joining these two pads of one net"},
                        "region":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm]"},
                        "region_mode":{"type":"string","enum":["inside","touching"],"default":"inside"},
                        "layer":{"type":"string"},
                        "kinds":{"type":"array","items":{"type":"string","enum":["track","via"]}},
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "limit":{"type":"integer","default":500},"offset":{"type":"integer","default":0}}})json"_json,
                   false, h_pcb_track_list );

KOPENAPI_REGISTER( "pcb_tracks_delete",
                   "Remove tracks and vias in one undo step: by uuids, or the ones passing every filter given - "
                   "net / nets (unroute them), between two pads (undo one connection: the copper joining them), "
                   "region, layer, kinds. Pads, zones and footprints are never touched; locked copper is kept "
                   "(override_locks). dry_run lists what would go. Answers what was removed per net and the "
                   "connections now open (unrouted_left); route again with pcb_route / pcb_route_connection / "
                   "pcb_track_add",
                   R"json({"type":"object","properties":{
                        "uuids":{"type":"array","items":{"type":"string"},"description":"exactly these (pcb_track_list); the filters below are ignored"},
                        "net":{"type":"string"},"nets":{"type":"array","items":{"type":"string"}},
                        "between":{"type":"array","items":{"type":"string"},"description":"[\"REF.PAD\", \"REF.PAD\"]"},
                        "region":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm]"},
                        "region_mode":{"type":"string","enum":["inside","touching"],"default":"inside","description":"inside: wholly in the region; touching: any part in it"},
                        "layer":{"type":"string"},
                        "kinds":{"type":"array","items":{"type":"string","enum":["track","via"]}},
                        "refill_zones":{"type":"boolean","default":false},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_tracks_delete, 300 );

KOPENAPI_REGISTER( "pcb_track_add",
                   "Lay a track by hand along given points (no router): from a pad and / or to a pad (their "
                   "centres) through points in mm; a point with via continues on that layer through a via. "
                   "Width and via size from the net's class unless width_mm is given. The same guard as the "
                   "router: copper that would touch another net is refused (422 with the violations), nothing "
                   "is added then. One undo step; dry_run checks only",
                   R"json({"type":"object","properties":{
                        "from":{"type":"string","description":"REF.PAD: start at its centre, on its net"},
                        "to":{"type":"string","description":"REF.PAD of the same net: end at its centre"},
                        "net":{"type":"string","description":"needed without 'from'"},
                        "points":{"type":"array","description":"[x_mm, y_mm] or {at: [x_mm, y_mm], via: 'B.Cu'}","items":{}},
                        "layer":{"type":"string","description":"start layer; default F.Cu (B.Cu from a bottom SMD pad)"},
                        "width_mm":{"type":"number"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_track_add );

KOPENAPI_MARK_EDITING( "pcb_route_connection", "pcb_route", "pcb_stitch", "pcb_tracks_delete", "pcb_track_add" );
