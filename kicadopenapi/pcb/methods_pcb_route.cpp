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

    /// @brief Through vias on the way: at each point the route changes to the given layer (a person
    /// pressing V while routing)
    std::vector<std::pair<VECTOR2I, PCB_LAYER_ID>> vias;
};


/// @brief One connection through PNS as a person routes it (see the file comment).  Fills aOut with what
/// happened (routed / reason / segments / metrics / violations); the staged commit is pushed only
/// when the guard is clean and it is no dry run.  aAdded: uuids of the new tracks / vias.
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

    for( const auto& [at, layer] : aReq.vias )
    {
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

    struct JOB
    {
        PAD*                  from;
        BOARD_CONNECTED_ITEM* to;
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

            jobs.push_back( { static_cast<PAD*>( pa ), pb, b->Pos(), (double) ( b->Pos() - a->Pos() ).EuclideanNorm(), name, power } );
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

    for( const JOB& job : jobs )
    {
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

                std::sort( out.begin(), out.end(), [&]( const VECTOR2I& x, const VECTOR2I& y )
                           { return ( x - aToward ).EuclideanNorm() < ( y - aToward ).EuclideanNorm(); } );
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
                wxSafeYield();
                wxMilliSleep( stepMs );
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

KOPENAPI_MARK_EDITING( "pcb_route_connection", "pcb_route", "pcb_stitch" );
