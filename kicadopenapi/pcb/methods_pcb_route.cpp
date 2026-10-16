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
#include <advanced_config.h>
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <project/net_settings.h>
#include <geometry/shape_circle.h>
#include <geometry/shape_segment.h>
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
#include <limits>
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
/// of another net - pads, tracks, vias, filled zones; removed items do not count.  The clearance is
/// the board's rule for the pair (DRC engine: custom rules, net classes, board minimum), as DRC and
/// the router see it.  aZonesYield: the caller refills the zones afterwards, so other nets' fills give
/// way and do not count.  aStaged: copper of the same call that is not on the board yet.
nlohmann::json guard( BOARD* aBoard, const std::vector<BOARD_CONNECTED_ITEM*>& aChanged,
                      const std::set<BOARD_ITEM*>& aRemoved, bool aZonesYield = false,
                      const std::vector<BOARD_CONNECTED_ITEM*>* aStaged = nullptr )
{
    nlohmann::json               violations = nlohmann::json::array();
    const BOARD_DESIGN_SETTINGS& ds = aBoard->GetDesignSettings();
    const int                    floor = ds.m_MinClearance;
    std::shared_ptr<DRC_ENGINE>  drc = ds.m_DRCEngine;

    // no pair needs more than this (copper or hole clearance): shapes farther apart are never asked
    // for their rule
    const int widest = std::max( { ds.GetBiggestClearanceValue(), floor, ds.m_HoleClearance, 1 } );

    auto required = [&]( BOARD_ITEM* aA, BOARD_ITEM* aB, PCB_LAYER_ID aLayer ) -> int
    {
        if( drc )
        {
            DRC_CONSTRAINT c = drc->EvalRules( CLEARANCE_CONSTRAINT, aA, aB, aLayer );

            if( c.GetValue().HasMin() )
                return std::max( c.GetValue().Min(), 0 );
        }

        auto cl = []( BOARD_ITEM* aItem, int aFloor )
        {
            BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( aItem );
            return bci ? netClearance( bci, aFloor ) : aFloor;
        };

        return std::max( cl( aA, floor ), cl( aB, floor ) );
    };

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

    // copper of the same call not on the board yet (a commit not pushed)
    if( aStaged )
        others.insert( others.end(), aStaged->begin(), aStaged->end() );

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
        BOX2I reach = item->GetBoundingBox();
        reach.Inflate( widest );

        for( PCB_LAYER_ID layer : item->GetLayerSet().CuStack() )
        {
            std::shared_ptr<SHAPE> shape = item->GetEffectiveShape( layer );

            for( BOARD_CONNECTED_ITEM* other : others )
            {
                if( other == item || other->GetNetCode() == item->GetNetCode() || !other->IsOnLayer( layer )
                    || !reach.Intersects( other->GetBoundingBox() ) )
                {
                    continue;
                }

                std::shared_ptr<SHAPE> otherShape = other->GetEffectiveShape( layer );

                if( !shape->Collide( otherShape.get(), widest - 1 ) )
                    continue;

                const int need = required( item, other, layer ) - 1;
                int       actual = 0;

                if( need >= 0 && shape->Collide( otherShape.get(), need, &actual ) )
                {
                    violations.push_back( { { "layer", str( aBoard->GetLayerName( layer ) ) },
                                            { "new", describe( item ) },
                                            { "other", describe( other ) },
                                            { "distance_mm", toMm( actual ) },
                                            { "required_mm", toMm( need + 1 ) } } );
                    continue;
                }

                // copper to a drilled hole (a via's, a through pad's) of the other: the board's hole
                // clearance, either way round
                for( const auto& [copper, holed, copperShape] :
                     { std::tuple<BOARD_ITEM*, BOARD_ITEM*, SHAPE*>{ item, other, shape.get() },
                       std::tuple<BOARD_ITEM*, BOARD_ITEM*, SHAPE*>{ other, item, otherShape.get() } } )
                {
                    std::shared_ptr<SHAPE_SEGMENT> hole = holed->HasHole() ? holed->GetEffectiveHoleShape() : nullptr;

                    if( !hole )
                        continue;

                    int holeNeed = ds.m_HoleClearance;

                    if( drc )
                    {
                        DRC_CONSTRAINT c = drc->EvalRules( HOLE_CLEARANCE_CONSTRAINT, copper, holed, layer );

                        if( c.GetValue().HasMin() )
                            holeNeed = c.GetValue().Min();
                    }

                    if( holeNeed > 0 && copperShape->Collide( hole.get(), holeNeed - 1, &actual ) )
                    {
                        violations.push_back( { { "layer", str( aBoard->GetLayerName( layer ) ) },
                                                { "new", describe( item ) },
                                                { "other", describe( other ) },
                                                { "hole", true },
                                                { "distance_mm", toMm( actual ) },
                                                { "required_mm", toMm( holeNeed ) } } );
                        break;
                    }
                }
            }

            for( ZONE* zone : aBoard->Zones() )
            {
                if( ( aZonesYield && !zone->GetIsRuleArea() ) || zone->GetNetCode() == item->GetNetCode() || !zone->IsOnLayer( layer )
                    || !zone->HasFilledPolysForLayer( layer ) || !reach.Intersects( zone->GetBoundingBox() ) )
                {
                    continue;
                }

                const std::shared_ptr<SHAPE_POLY_SET>& fill = zone->GetFilledPolysList( layer );

                if( fill && fill->Collide( shape.get(), required( item, zone, layer ) - 1 ) )
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
    BOARD_CONNECTED_ITEM* from = nullptr;     ///< a pad, or a track / via to go on from
    BOARD_CONNECTED_ITEM* to = nullptr;        ///< nullptr: the nearest unconnected anchor of the net
    VECTOR2I              toPos;
    PCB_LAYER_ID          layer = F_Cu;
    PNS::PNS_MODE         mode = PNS::RM_Walkaround;
    bool                  dryRun = false;

    /// @brief Points on the way: with a layer the route drops a through via there and continues on
    /// that layer (a person pressing V while routing); without one it fixes a corner there (a click)
    std::vector<std::pair<VECTOR2I, std::optional<PCB_LAYER_ID>>> vias;

    /// @brief Route the pair: from and to are pads of the P (or N) net, the coupled pads of the other
    /// net are found next to them by KiCad's diff-pair router (net names ending in +/- or P/N)
    bool diffPair = false;

    /// @brief A narrower track than the net's class gives (> 0): the connection meets a fine-pitch pad
    int neckWidth = 0;

    /// @brief Where on `from` the route starts (a track's end); its position when not given
    std::optional<VECTOR2I> fromPos;

    /// @brief Via size for this route instead of the net class's (> 0)
    int viaDiameter = 0;
    int viaDrill = 0;

    /// @brief A pair's cursor path (a single route found first): the pair follows it
    std::vector<VECTOR2I> guide;
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


bool routeOneImpl( PCB_CONTEXT& aContext, const ROUTE_REQUEST& aReq, nlohmann::json& aOut, std::vector<KIID>& aAdded )
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

    // The same result on every run: the router's wall-clock limits (made for a hand on the mouse) are
    // lifted for the call, its iteration limits bound the work
    settings.SetShoveTimeLimit( 10000 );

    struct CLUSTER_TIMEOUT
    {
        int& limit = const_cast<ADVANCED_CFG&>( ADVANCED_CFG::GetCfg() ).m_PNSProcessClusterTimeout;
        int  previous = limit;
        CLUSTER_TIMEOUT() { limit = 10000; }
        ~CLUSTER_TIMEOUT() { limit = previous; }
    } clusterTimeout;

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
    const auto synced = std::chrono::steady_clock::now();
    router->SetMode( aReq.diffPair ? PNS::PNS_MODE_ROUTE_DIFF_PAIR : PNS::PNS_MODE_ROUTE_SINGLE );

    PNS::ITEM* startItem = router->GetWorld()->FindItemByParent( aReq.from );

    if( !startItem )
    {
        aOut["routed"] = false;
        aOut["reason"] = "the router does not know the start pad";
        return false;
    }

    const VECTOR2I      start = aReq.fromPos.value_or( aReq.from->GetPosition() );
    PNS::SIZES_SETTINGS sizes( router->Sizes() );
    iface->SetStartLayerFromPCBNew( aReq.layer );
    iface->ImportSizes( sizes, startItem, nullptr, start );
    sizes.AddLayerPair( iface->GetPNSLayerFromBoardLayer( F_Cu ), iface->GetPNSLayerFromBoardLayer( B_Cu ) );

    if( aReq.viaDiameter > 0 )
        sizes.SetViaDiameter( aReq.viaDiameter );

    if( aReq.viaDrill > 0 )
        sizes.SetViaDrill( aReq.viaDrill );

    if( aReq.neckWidth > 0 && aReq.neckWidth < sizes.TrackWidth() )
    {
        aOut["necked_from_mm"] = toMm( sizes.TrackWidth() );
        sizes.SetTrackWidth( aReq.neckWidth );
    }

    // a pair without diff-pair rules: the track width, the gap the clearance
    if( aReq.diffPair && sizes.DiffPairWidth() <= 0 )
        sizes.SetDiffPairWidth( sizes.TrackWidth() );

    if( aReq.diffPair && sizes.DiffPairGap() <= 0 )
        sizes.SetDiffPairGap( sizes.Clearance() );

    router->UpdateSizes( sizes );

    aOut["net"] = str( UnescapeString( aReq.from->GetNetname() ) );
    aOut["layer"] = str( board->GetLayerName( aReq.layer ) );
    aOut["mode"] = aReq.mode == PNS::RM_Shove ? "shove" : "walkaround";
    aOut["track_width_mm"] = toMm( sizes.TrackWidth() );
    aOut["clearance_mm"] = toMm( sizes.Clearance() );

    if( aReq.diffPair )
    {
        aOut["diff_pair"] = { { "width_mm", toMm( sizes.DiffPairWidth() ) }, { "gap_mm", toMm( sizes.DiffPairGap() ) } };

        if( !aReq.vias.empty() )
        {
            aOut["routed"] = false;
            aOut["reason"] = "a differential pair is routed without waypoints";
            return false;
        }
    }

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
                aOut["toward_mm"] = { toMm( at.x ), toMm( at.y ) };
                return false;
            }

            continue;
        }

        const PCB_LAYER_ID layer = *viaLayer;

        // a via with no track before it (at the point the route stands on) ends the routing in PNS,
        // and moving on from there uses a placer that has let go of its world
        if( placer->CurrentStart() == at )
        {
            router->StopRouting();
            iface->Staged().Revert();
            aOut["routed"] = false;
            aOut["reason"] = "the via at " + std::to_string( toMm( at.x ) ) + ", " + std::to_string( toMm( at.y ) )
                             + " is where the route already stands (its start): move it along the way";
            return false;
        }

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
                aOut["toward_mm"] = { toMm( at.x ), toMm( at.y ) };
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

    // a pair follows the cursor as a hand moves it: in steps of about a millimetre (a jump to the
    // target leaves the coupled walk nothing to start from)
    if( aReq.diffPair )
    {
        std::vector<VECTOR2I> path = { placer->CurrentStart() };
        path.insert( path.end(), aReq.guide.begin(), aReq.guide.end() );
        path.push_back( target );

        for( size_t leg = 1; leg < path.size(); ++leg )
        {
            const VECTOR2I from = path[leg - 1];
            const VECTOR2I to = path[leg];
            const int      steps = std::max( 1, (int) ( ( to - from ).EuclideanNorm() / pcbIUScale.mmToIU( 1.0 ) ) );

            for( int k = 1; k <= steps; ++k )
            {
                if( leg + 1 < path.size() || k < steps )
                    router->Move( from + ( to - from ) * k / steps, nullptr );
            }
        }
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
    // (the pair's placer answers the cursor as its end: it finishes only when it snapped onto the
    // target pads, which FixRoute tells)
    const bool reached = aReq.diffPair
                         || ( placer->CurrentEnd() == target && targetLayers.Overlaps( router->GetCurrentLayer() ) );

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
        aOut["reason"] = aReq.diffPair ? "the pair did not reach the target pads"
                         : reached     ? "the router could not fix the route"
                                       : "blocked: the track stops before the target";
        aOut["stopped_at_mm"] = { toMm( end.x ), toMm( end.y ) };
        aOut["target_mm"] = { toMm( target.x ), toMm( target.y ) };
        return false;
    }

    // After a shove PNS can hand back a copy of a segment it left where it was (the copy has the
    // original's parent, the original is not removed): the board has that copper already
    for( auto it = iface->m_added.begin(); it != iface->m_added.end(); )
    {
        if( ( *it )->Type() != PCB_TRACE_T )
        {
            ++it;
            continue;
        }

        PCB_TRACK* fresh = static_cast<PCB_TRACK*>( *it );
        bool       twin = false;

        for( PCB_TRACK* other : board->Tracks() )
        {
            if( other->Type() == PCB_TRACE_T && !iface->m_removed.count( other ) && other->GetNetCode() == fresh->GetNetCode()
                && other->GetLayer() == fresh->GetLayer() && other->GetWidth() == fresh->GetWidth()
                && ( ( other->GetStart() == fresh->GetStart() && other->GetEnd() == fresh->GetEnd() )
                     || ( other->GetStart() == fresh->GetEnd() && other->GetEnd() == fresh->GetStart() ) ) )
            {
                twin = true;
                break;
            }
        }

        if( twin )
        {
            iface->Staged().Unstage( fresh, nullptr );
            it = iface->m_added.erase( it );
        }
        else
        {
            ++it;
        }
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

    const auto           routedAt = std::chrono::steady_clock::now();
    const nlohmann::json violations = guard( board, changed, iface->m_removed );
    const auto           guardedAt = std::chrono::steady_clock::now();
    const double         direct = ( target - start ).EuclideanNorm();

    aOut["segments"] = segments;
    aOut["metrics"] = { { "length_mm", toMm( length ) },
                        { "direct_mm", toMm( direct ) },
                        { "detour", direct > 0 ? std::round( length / direct * 100.0 ) / 100.0 : 1.0 },
                        { "segments", segments.size() },
                        { "vias", vias },
                        { "shoved_items", iface->m_modified.size() + iface->m_removed.size() },
                        { "ms", std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() },
                        { "sync_ms", std::chrono::duration_cast<std::chrono::milliseconds>( synced - started ).count() },
                        { "route_ms", std::chrono::duration_cast<std::chrono::milliseconds>( routedAt - synced ).count() },
                        { "guard_ms", std::chrono::duration_cast<std::chrono::milliseconds>( guardedAt - routedAt ).count() } };

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


/// @brief What stands in the way of a route that stopped: copper of other nets on its layer that the
/// path from the stop point toward its aim (the target, or the via / corner it was heading for)
/// crosses within 5 mm, or that lies within 1 mm of the stop point; nearest first, at most three
/// (pads by REF.PAD, tracks and vias by net)
nlohmann::json blockersAt( BOARD* aBoard, const VECTOR2I& aAt, const VECTOR2I& aAim, PCB_LAYER_ID aLayer, int aNetCode,
                           int aHalfWidth )
{
    std::vector<std::pair<int, nlohmann::json>> found;
    const int      near = pcbIUScale.mmToIU( 1.0 );
    const VECTOR2I dir = aAim - aAt;
    const double   len = dir.EuclideanNorm();
    const int      look = pcbIUScale.mmToIU( 5.0 );
    const VECTOR2I end = len > look ? aAt + VECTOR2I( KiROUND( (double) dir.x * look / len ), KiROUND( (double) dir.y * look / len ) )
                                    : aAim;
    const SHAPE_SEGMENT path( aAt, end, 2 * aHalfWidth );
    const SHAPE_CIRCLE  probe( aAt, 0 );

    BOX2I area( aAt, VECTOR2I( 0, 0 ) );
    area.Merge( end );
    area.Inflate( near + aHalfWidth );

    auto consider = [&]( BOARD_CONNECTED_ITEM* aItem, nlohmann::json aWhat )
    {
        if( aItem->GetNetCode() == aNetCode || !aItem->IsOnLayer( aLayer ) || !area.Intersects( aItem->GetBoundingBox() ) )
            return;

        std::shared_ptr<SHAPE> shape = aItem->GetEffectiveShape( aLayer );
        int                    actual = 0;

        if( shape->Collide( &path, 0 ) || shape->Collide( &probe, near ) )
        {
            shape->Collide( &probe, std::numeric_limits<int>::max() / 2, &actual );
            aWhat["net"] = str( UnescapeString( aItem->GetNetname() ) );
            aWhat["distance_mm"] = toMm( actual );
            found.emplace_back( actual, aWhat );
        }
    };

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            consider( pad, { { "pad", str( fp->GetReference() ) + "." + str( pad->GetNumber() ) } } );
    }

    for( PCB_TRACK* track : aBoard->Tracks() )
        consider( track, { { "type", track->Type() == PCB_VIA_T ? "via" : "track" }, { "uuid", str( track->m_Uuid.AsString() ) } } );

    std::stable_sort( found.begin(), found.end(), []( const auto& x, const auto& y ) { return x.first < y.first; } );

    nlohmann::json out = nlohmann::json::array();

    for( size_t k = 0; k < found.size() && k < 3; ++k )
        out.push_back( found[k].second );

    return out;
}


/// @brief routeOneImpl, and for a route that stopped short what stands there (blocked_by)
bool routeOne( PCB_CONTEXT& aContext, const ROUTE_REQUEST& aReq, nlohmann::json& aOut, std::vector<KIID>& aAdded )
{
    const bool ok = routeOneImpl( aContext, aReq, aOut, aAdded );

    auto point = []( const nlohmann::json& aMm )
    { return VECTOR2I( pcbIUScale.mmToIU( aMm[0].get<double>() ), pcbIUScale.mmToIU( aMm[1].get<double>() ) ); };

    if( !ok && aOut.contains( "stopped_at_mm" ) && aReq.from )
    {
        const VECTOR2I at = point( aOut["stopped_at_mm"] );
        const VECTOR2I aim = aOut.contains( "toward_mm" ) ? point( aOut["toward_mm"] )
                             : aOut.contains( "target_mm" ) ? point( aOut["target_mm"] ) : at;
        const int      half = pcbIUScale.mmToIU( aOut.value( "track_width_mm", 0.2 ) ) / 2
                         + pcbIUScale.mmToIU( aOut.value( "clearance_mm", 0.2 ) );

        aOut["blocked_by"] = blockersAt( aContext.GetBoard(), at, aim, aReq.layer, aReq.from->GetNetCode(), half );
    }

    return ok;
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


/// @brief The board's pours (filled, not rule areas) refilled in one undo step: after copper was
/// laid through fills that gave way
void refillPours( PCB_CONTEXT& aContext, BOARD* aBoard )
{
    std::vector<ZONE*> zones;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone->IsFilled() && !zone->GetIsRuleArea() && !zone->IsTeardropArea() )
            zones.push_back( zone );
    }

    if( zones.empty() )
        return;

    BOARD_COMMIT                fillCommit( aContext.GetToolManager() );
    ZONE_FILLER                 filler( aBoard, &fillCommit );
    KOPENAPI_KEEPALIVE_REPORTER reporter;
    filler.SetProgressReporter( &reporter );

    if( filler.Fill( zones ) )
        fillCommit.Push( _( "Fill zones (API)" ) );
    else
        fillCommit.Revert();

    aBoard->BuildConnectivity();
    aBoard->GetConnectivity()->RecalculateRatsnest();
}


/// @brief Pour fills out of the way for one route (the router's world takes a fill as copper of its
/// net: nothing would cross it), put back after - refilled when copper was laid, as they were
/// otherwise (a dry run, a failure: no undo step, nothing changes)
class FILL_HOLD
{
public:
    FILL_HOLD( BOARD* aBoard, bool aActive ) : m_board( aBoard )
    {
        if( !aActive )
            return;

        for( ZONE* zone : aBoard->Zones() )
        {
            if( !zone->IsFilled() || zone->GetIsRuleArea() || zone->IsTeardropArea() )
                continue;

            SAVED saved{ zone, {} };

            for( PCB_LAYER_ID layer : zone->GetLayerSet().Seq() )
            {
                if( zone->HasFilledPolysForLayer( layer ) )
                    saved.fills.emplace_back( layer, *zone->GetFilledPolysList( layer ) );
            }

            m_saved.push_back( std::move( saved ) );
            zone->UnFill();
        }

        if( !m_saved.empty() )
            aBoard->BuildConnectivity();
    }

    ~FILL_HOLD() { Restore(); }

    bool Active() const { return !m_saved.empty(); }

    void Restore()
    {
        for( SAVED& saved : m_saved )
        {
            for( const auto& [layer, fill] : saved.fills )
                saved.zone->SetFilledPolysList( layer, fill );

            saved.zone->SetIsFilled( true );
        }

        if( !m_saved.empty() )
            m_board->BuildConnectivity();

        m_saved.clear();
    }

    /// @brief The old fills back first (the fill step's undo copy is the board as it was), then
    /// KiCad's filler around the new copper
    void Refill( PCB_CONTEXT& aContext )
    {
        const bool any = Active();
        Restore();

        if( any )
            refillPours( aContext, m_board );
    }

private:
    struct SAVED
    {
        ZONE*                                               zone;
        std::vector<std::pair<PCB_LAYER_ID, SHAPE_POLY_SET>> fills;
    };

    BOARD*             m_board;
    std::vector<SAVED> m_saved;
};


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

        const VECTOR2I point( pcbIUScale.mmToIU( at[0].get<double>() ), pcbIUScale.mmToIU( at[1].get<double>() ) );

        // a via at the corner just before it: the via's leg reaches that point anyway
        if( viaLayer && !req.vias.empty() && !req.vias.back().second && req.vias.back().first == point )
            req.vias.pop_back();

        // the same point twice: once
        if( !req.vias.empty() && req.vias.back().first == point && req.vias.back().second == viaLayer )
            continue;

        req.vias.emplace_back( point, viaLayer );
    }

    const std::string zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (pour fills give way, refilled after) or keep" );

    if( aArgs.contains( "via_diameter_mm" ) )
        req.viaDiameter = pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.0 ) );

    if( aArgs.contains( "via_drill_mm" ) )
        req.viaDrill = pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.0 ) );

    {
        const BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();

        if( ( req.viaDiameter && req.viaDiameter < bds.m_ViasMinSize ) || ( req.viaDrill && req.viaDrill < bds.m_MinThroughDrill ) )
            return KOPENAPI_RESULT::Error( 422, "via_diameter_mm / via_drill_mm below the board's minimum via" );
    }

    nlohmann::json    result = { { "from", aArgs["from"] } };
    std::vector<KIID> added;

    if( aArgs.contains( "to" ) )
        result["to"] = aArgs["to"];

    {
        FILL_HOLD hold( board, zoneMode == "refill" );

        routeOne( *context, req, result, added );

        if( result.value( "applied", false ) && hold.Active() )
        {
            hold.Refill( *context );
            result["zones_refilled"] = true;
        }
    }

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
namespace
{
std::vector<PCB_TRACK*> copperBetween( BOARD* aBoard, PAD* aFrom, PAD* aTo );
}


/// @brief The corner points of a route's segments (as routeOne answers them) in order from aStart
static std::vector<VECTOR2I> segmentPath( const nlohmann::json& aSegments, const VECTOR2I& aStart )
{
    std::vector<std::pair<VECTOR2I, VECTOR2I>> left;

    for( const nlohmann::json& seg : aSegments )
    {
        left.emplace_back( VECTOR2I( pcbIUScale.mmToIU( seg["from"][0].get<double>() ), pcbIUScale.mmToIU( seg["from"][1].get<double>() ) ),
                           VECTOR2I( pcbIUScale.mmToIU( seg["to"][0].get<double>() ), pcbIUScale.mmToIU( seg["to"][1].get<double>() ) ) );
    }

    std::vector<VECTOR2I> path;
    VECTOR2I              at = aStart;
    const int             near = pcbIUScale.mmToIU( 0.01 );

    while( !left.empty() )
    {
        auto it = std::find_if( left.begin(), left.end(), [&]( const auto& s )
                                { return ( s.first - at ).EuclideanNorm() <= near || ( s.second - at ).EuclideanNorm() <= near; } );

        if( it == left.end() )
            break;

        at = ( it->first - at ).EuclideanNorm() <= near ? it->second : it->first;
        path.push_back( at );
        left.erase( it );
    }

    if( !path.empty() )
        path.pop_back();   // the target itself

    return path;
}


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

    // plane layers (type power in the stack-up) carry no signals: a track there would cut the plane
    nlohmann::json planeLayers = nlohmann::json::array();

    if( !aArgs.value( "plane_layers", false ) )
    {
        // a plane: type power in the stack-up, or a layer more than half of which one pour covers
        double boardArea = 0;
        {
            SHAPE_POLY_SET outline;

            if( board->GetBoardPolygonOutlines( outline, true ) )
                boardArea = outline.Area();
        }

        auto isPlane = [&]( PCB_LAYER_ID aLayer )
        {
            if( board->GetLayerType( aLayer ) == LT_POWER )
                return true;

            if( IsExternalCopperLayer( aLayer ) || boardArea <= 0 )
                return false;   // outer layers with a pour still carry signals

            std::map<int, double> covered;   // net -> pour area on this layer

            for( ZONE* zone : board->Zones() )
            {
                if( !zone->GetIsRuleArea() && zone->IsOnLayer( aLayer ) )
                    covered[zone->GetNetCode()] += zone->Outline()->Area();
            }

            return std::any_of( covered.begin(), covered.end(),
                                [&]( const auto& c ) { return c.second > 0.5 * boardArea; } );
        };

        std::erase_if( layers, [&]( PCB_LAYER_ID aLayer )
                       {
                           if( !isPlane( aLayer ) )
                               return false;

                           planeLayers.push_back( str( board->GetLayerName( aLayer ) ) );
                           return true;
                       } );

        if( layers.empty() )
            return KOPENAPI_RESULT::Error( 422, "every given layer is a plane layer (type power): give signal layers, or "
                                                "plane_layers: true" );
    }

    const std::string order = aArgs.value( "order", std::string( "short_first" ) );
    const bool        dryRun = aArgs.value( "dry_run", false );
    const int         stepMs = std::clamp( aArgs.value( "step_ms", 0 ), 0, 5000 );
    const std::string style = aArgs.value( "style", std::string( "direct" ) );

    if( style != "direct" && style != "rail" )
        return KOPENAPI_RESULT::Error( 400, "style: direct or rail" );

    // attempts per connection and pass: a count, not a time (the same board routes the same way);
    // on a big board an attempt costs about a second
    const int maxTries = std::clamp( aArgs.value( "max_tries", 120 ), 1, 10000 );

    const std::string diffPairs = aArgs.value( "diff_pairs", std::string( "auto" ) );

    if( aArgs.value( "neck_down", std::string( "auto" ) ) != "auto" && aArgs.value( "neck_down", std::string( "auto" ) ) != "off" )
        return KOPENAPI_RESULT::Error( 400, "neck_down: auto (connections at fine-pitch pads at the Default width) or off" );

    if( diffPairs != "auto" && diffPairs != "off" )
        return KOPENAPI_RESULT::Error( 400, "diff_pairs: auto (nets named as a pair routed coupled) or off" );
    size_t            glowFrom = 0;

    // Pours are filled around the tracks afterwards: their fills come out for the routing (they
    // would block every crossing) and are refilled at the end
    const std::string zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (fills out while routing, refilled after) or keep" );

    std::vector<ZONE*> unfilled;

    if( zoneMode == "refill" && !dryRun )
    {
        BOARD_COMMIT unfill( context->GetToolManager() );

        for( ZONE* zone : board->Zones() )
        {
            if( zone->IsFilled() && !zone->GetIsRuleArea() && !zone->IsTeardropArea() )
            {
                unfill.Modify( zone );
                zone->UnFill();
                unfilled.push_back( zone );
            }
        }

        if( unfilled.empty() )
            unfill.Revert();
        else
            unfill.Push( _( "Unfill zones for routing (API)" ) );
    }

    // Under the parts (avoid): inside the ring of pads of a quad package (QFP, QFN, LGA) no track and
    // no via of this call - such tracks can wall its other pins in.  Temporary rule areas on the
    // part's side for the call; parts with pads inside the ring (exposed pad, BGA) are left open
    const std::string underParts = aArgs.value( "under_parts", std::string( "allow" ) );

    if( underParts != "avoid" && underParts != "allow" )
        return KOPENAPI_RESULT::Error( 400, "under_parts: avoid (no tracks inside a part's ring of pads) or allow" );

    struct TEMP_KEEPOUTS
    {
        BOARD*             board;
        std::vector<ZONE*> zones;

        ~TEMP_KEEPOUTS()
        {
            for( ZONE* zone : zones )
            {
                board->Remove( zone );
                delete zone;
            }
        }
    } keepouts{ board, {} };

    nlohmann::json keptOut = nlohmann::json::array();

    if( underParts == "avoid" )
    {
        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( fp->Pads().size() < 8 )
                continue;

            BOX2I centres;
            int   padLength = 0;
            bool  first = true;

            for( PAD* pad : fp->Pads() )
            {
                if( first )
                    centres = BOX2I( pad->GetPosition(), VECTOR2I( 0, 0 ) );
                else
                    centres.Merge( pad->GetPosition() );

                first = false;
                const BOX2I pb = pad->GetBoundingBox();
                padLength = std::max( { padLength, (int) pb.GetWidth(), (int) pb.GetHeight() } );
            }

            BOX2I inner = centres;
            inner.Inflate( -( padLength / 2 + pcbIUScale.mmToIU( 0.1 ) ) );

            if( inner.GetWidth() <= 0 || inner.GetHeight() <= 0 )
                continue;

            // pads on all four sides (a quad package); two rows (SOIC, headers) leave room between
            int sides[4] = { 0, 0, 0, 0 };

            for( PAD* pad : fp->Pads() )
            {
                const VECTOR2I c = pad->GetPosition();
                const bool     spanX = c.x >= inner.GetLeft() && c.x <= inner.GetRight();
                const bool     spanY = c.y >= inner.GetTop() && c.y <= inner.GetBottom();
                sides[0] += c.x < inner.GetLeft() && spanY;
                sides[1] += c.x > inner.GetRight() && spanY;
                sides[2] += c.y < inner.GetTop() && spanX;
                sides[3] += c.y > inner.GetBottom() && spanX;
            }

            if( std::any_of( std::begin( sides ), std::end( sides ), []( int n ) { return n < 2; } ) )
                continue;

            if( std::any_of( fp->Pads().begin(), fp->Pads().end(),
                             [&]( PAD* pad ) { return inner.Contains( pad->GetPosition() ); } ) )
            {
                continue;
            }

            auto* zone = new ZONE( board );
            zone->SetIsRuleArea( true );
            zone->SetDoNotAllowTracks( true );
            zone->SetDoNotAllowVias( true );
            zone->SetDoNotAllowPads( false );
            zone->SetDoNotAllowFootprints( false );
            zone->SetDoNotAllowZoneFills( false );
            zone->SetLayerSet( LSET( { fp->GetLayer() == B_Cu ? B_Cu : F_Cu } ) );

            SHAPE_LINE_CHAIN rect;
            rect.Append( inner.GetLeft(), inner.GetTop() );
            rect.Append( inner.GetRight(), inner.GetTop() );
            rect.Append( inner.GetRight(), inner.GetBottom() );
            rect.Append( inner.GetLeft(), inner.GetBottom() );
            rect.SetClosed( true );
            zone->AddPolygon( rect );

            board->Add( zone );
            keepouts.zones.push_back( zone );
            keptOut.push_back( str( fp->GetReference() ) );
        }
    }

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
        bool                  trunk = false;   ///< a differential pair's trunk, not an airwire
    };

    std::vector<JOB> jobs;

    struct GAP
    {
        KIID        a;
        KIID        b;
        VECTOR2I    aPos;
        VECTOR2I    bPos;
        std::string net;
    };

    std::vector<GAP> gaps;

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

        // a fanned-out ball is reached at its via (any layer), not at the ball (its own layer only)
        std::vector<PCB_VIA*> netVias;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() == PCB_VIA_T && t->GetNetCode() == code )
                netVias.push_back( static_cast<PCB_VIA*>( t ) );
        }

        auto fannedVia = [&]( BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aAnchor ) -> PCB_VIA*
        {
            // a track with an end on a via (the stub of a dog-bone): the via
            if( aItem->Type() == PCB_TRACE_T || aItem->Type() == PCB_ARC_T )
            {
                const PCB_TRACK* t = static_cast<PCB_TRACK*>( aItem );

                for( PCB_VIA* v : netVias )
                {
                    for( const VECTOR2I& end : { aAnchor, t->GetStart(), t->GetEnd() } )
                    {
                        if( ( v->GetPosition() - end ).EuclideanNorm() <= pcbIUScale.mmToIU( 0.01 ) )
                            return v;
                    }
                }

                return nullptr;
            }

            if( aItem->Type() != PCB_PAD_T || static_cast<PAD*>( aItem )->GetAttribute() != PAD_ATTRIB::SMD )
                return nullptr;

            const VECTOR2I at = aItem->GetPosition();
            const BOX2I    pad = aItem->GetBoundingBox();
            const int      reach = std::max( pad.GetWidth(), pad.GetHeight() ) + pcbIUScale.mmToIU( 0.45 );

            for( PCB_VIA* v : netVias )
            {
                if( ( v->GetPosition() - at ).EuclideanNorm() > reach )
                    continue;

                const auto joined = connectivity->GetConnectedItems( v );

                if( std::find( joined.begin(), joined.end(), aItem ) != joined.end() )
                    return v;
            }

            return nullptr;
        };

        for( const CN_EDGE& edge : rn->GetEdges() )
        {
            std::shared_ptr<const CN_ANCHOR> a = edge.GetSourceNode(), b = edge.GetTargetNode();

            if( !a || !b )
                continue;

            BOARD_CONNECTED_ITEM* pa = a->Parent();
            BOARD_CONNECTED_ITEM* pb = b->Parent();
            VECTOR2I              posA = a->Pos(), posB = b->Pos();

            if( PCB_VIA* v = fannedVia( pa, posA ) )
                pa = v, posA = v->GetPosition();

            if( PCB_VIA* v = fannedVia( pb, posB ) )
                pb = v, posB = v->GetPosition();

            if( pa->Type() != PCB_PAD_T )
                std::swap( pa, pb ), std::swap( posA, posB );

            if( pa->Type() != PCB_PAD_T )
            {
                // two pieces of the net's copper with a gap between (tracks / vias, a fanned-out ball's
                // via): routed after the pad connections, from the end of one to the other
                if( ( pa->Type() == PCB_TRACE_T || pa->Type() == PCB_ARC_T || pa->Type() == PCB_VIA_T )
                    && ( pb->Type() == PCB_TRACE_T || pb->Type() == PCB_ARC_T || pb->Type() == PCB_VIA_T ) )
                {
                    gaps.push_back( { pa->m_Uuid, pb->m_Uuid, posA, posB, name } );
                }

                continue;
            }

            jobs.push_back( { static_cast<PAD*>( pa ), pb, pa->m_Uuid, pb->m_Uuid, posB,
                              (double) ( posB - posA ).EuclideanNorm(), name, power } );
        }
    }

    std::stable_sort( jobs.begin(), jobs.end(),
                      [&]( const JOB& x, const JOB& y )
                      {
                          if( order == "power_first" && x.power != y.power )
                              return x.power;

                          return x.length < y.length;
                      } );

    // dense_first: connections at fine-pitch pads (0.65 mm or less to the next pad) before the rest
    auto pitchOf = [&]( PAD* aPad )
    {
        double best = std::numeric_limits<double>::max();

        for( PAD* other : aPad->GetParentFootprint()->Pads() )
        {
            if( other != aPad )
                best = std::min( best, double( ( other->GetPosition() - aPad->GetPosition() ).EuclideanNorm() ) );
        }

        return best;
    };

    // Neck-down: a connection with an end on a fine-pitch pad (0.65 mm or closer to the next) is
    // routed at the Default class width (not below the board's minimum) - a wide power class does
    // not fit between such pads; the rest of the net keeps its width
    const std::string neckDown = aArgs.value( "neck_down", std::string( "auto" ) );
    const int         neckWidth = std::max( board->GetDesignSettings().m_TrackMinWidth,
                                            board->GetDesignSettings().m_NetSettings->GetDefaultNetclass()->HasTrackWidth()
                                                    ? board->GetDesignSettings().m_NetSettings->GetDefaultNetclass()->GetTrackWidth()
                                                    : 0 );

    auto neckFor = [&]( PAD* aFrom, BOARD_CONNECTED_ITEM* aTo ) -> int
    {
        const double fine = pcbIUScale.mmToIU( 0.65 );

        if( neckDown != "auto" )
            return 0;

        if( pitchOf( aFrom ) <= fine || ( aTo && aTo->Type() == PCB_PAD_T && pitchOf( static_cast<PAD*>( aTo ) ) <= fine ) )
            return neckWidth;

        return 0;
    };

    if( order == "dense_first" )
    {
        const double fine = pcbIUScale.mmToIU( 0.65 );

        std::stable_sort( jobs.begin(), jobs.end(),
                          [&]( const JOB& x, const JOB& y )
                          {
                              const bool fx = pitchOf( x.from ) <= fine
                                              || ( x.to->Type() == PCB_PAD_T && pitchOf( static_cast<PAD*>( x.to ) ) <= fine );
                              const bool fy = pitchOf( y.from ) <= fine
                                              || ( y.to->Type() == PCB_PAD_T && pitchOf( static_cast<PAD*>( y.to ) ) <= fine );

                              return fx != fy ? fx : x.length < y.length;
                          } );
    }

    const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();

    // hole-to-hole rule against every hole already there (earlier vias, THT pads)
    auto holeClash = [&]( const VECTOR2I& aAt )
    {
        const int drill = ds.GetCurrentViaDrill();

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

    // Via spots to leave an end: out of the package first (away from its body, past the pad's end,
    // a little to either side), then around the end; nearest the other end first within each kind
    auto spots = [&]( BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aPos, const VECTOR2I& aToward )
    {
        std::vector<VECTOR2I> outward, around;
        const int             grid = pcbIUScale.mmToIU( 0.05 );
        auto                  snap = [&]( const VECTOR2D& p )
        {
            return VECTOR2I( KiROUND( p.x / grid ) * grid, KiROUND( p.y / grid ) * grid );
        };

        if( aEnd->Type() == PCB_PAD_T && static_cast<PAD*>( aEnd )->GetParentFootprint() )
        {
            PAD*           pad = static_cast<PAD*>( aEnd );
            FOOTPRINT*     fp = pad->GetParentFootprint();
            const VECTOR2D centre = fp->GetBoundingBox( false ).Centre();
            VECTOR2D       dir = VECTOR2D( aPos ) - centre;

            // a pad on a side of a package leaves square to that side
            if( std::abs( dir.x ) >= std::abs( dir.y ) )
                dir = VECTOR2D( dir.x >= 0 ? 1 : -1, 0 );
            else
                dir = VECTOR2D( 0, dir.y >= 0 ? 1 : -1 );

            const BOX2I    box = pad->GetBoundingBox();
            const double   half = std::abs( dir.x ) > 0 ? box.GetWidth() / 2.0 : box.GetHeight() / 2.0;
            const VECTOR2D across( -dir.y, dir.x );

            for( double d : { 0.45, 0.75, 1.1, 1.6 } )
            {
                for( double side : { 0.0, 0.4, -0.4, 0.8, -0.8 } )
                {
                    outward.push_back( snap( VECTOR2D( aPos ) + dir * ( half + pcbIUScale.mmToIU( d ) )
                                             + across * pcbIUScale.mmToIU( side ) ) );
                }
            }
        }

        const BOX2I box = aEnd->GetBoundingBox();
        const int   reach = std::max( box.GetWidth(), box.GetHeight() ) / 2 + pcbIUScale.mmToIU( 1.3 );

        for( int k = 0; k < 8; ++k )
        {
            const double a = k * M_PI / 4;
            around.push_back( snap( VECTOR2D( aPos ) + VECTOR2D( reach * std::cos( a ), reach * std::sin( a ) ) ) );
        }

        auto byTarget = [&]( const VECTOR2I& x, const VECTOR2I& y )
        { return ( x - aToward ).EuclideanNorm() < ( y - aToward ).EuclideanNorm(); };

        outward.erase( std::remove_if( outward.begin(), outward.end(), holeClash ), outward.end() );
        around.erase( std::remove_if( around.begin(), around.end(), holeClash ), around.end() );
        std::stable_sort( around.begin(), around.end(), byTarget );

        std::vector<VECTOR2I> out;

        // the outward spots in their order (the closest escape first), at most 6, then 3 around
        for( size_t i = 0; i < outward.size() && out.size() < 6; ++i )
            out.push_back( outward[i] );

        for( size_t i = 0; i < around.size() && i < 3; ++i )
            out.push_back( around[i] );

        return out;
    };

    std::vector<KIID> added;
    const bool rail = style == "rail";
    const bool viasAllowed = aArgs.value( "vias", true ) && layers.size() >= 2;

    // Every way to route one connection: on a layer both ends are on (walkaround, then shove),
    // then out through a via near each end and across on another layer.  aShove false keeps every
    // other route as it is (rip-up attempts must be revertible).
    auto tryRoute = [&]( JOB& aJob, nlohmann::json& aAttempt, int& aTries, bool aShove, bool aBothWays ) -> bool
    {
        bool ok = false;

        // both ways (retries) when the far end is a pad too: the walkaround finds other paths from the
        // other end; the first pass goes one way only, the other end's detours crowd later routes
        struct WAY
        {
            PAD*                  from;
            BOARD_CONNECTED_ITEM* to;
            VECTOR2I              toPos;
        };

        std::vector<WAY> ways = { { aJob.from, aJob.to, aJob.toPos } };

        if( aBothWays && aJob.to->Type() == PCB_PAD_T )
            ways.push_back( { static_cast<PAD*>( aJob.to ), aJob.from, aJob.from->GetPosition() } );

        for( PNS::PNS_MODE mode : { PNS::RM_Walkaround, PNS::RM_Shove } )
        {
            if( mode == PNS::RM_Shove && !aShove )
                continue;

            for( const WAY& way : ways )
            {
                for( PCB_LAYER_ID layer : layers )
                {
                    if( ok || aTries >= maxTries || rail || !way.from->IsOnLayer( layer ) || !way.to->IsOnLayer( layer ) )
                        continue;

                    ROUTE_REQUEST req{ way.from, way.to, way.toPos, layer, mode, dryRun, {} };
                    req.neckWidth = neckFor( way.from, way.to );
                    aAttempt = nlohmann::json::object();
                    aTries++;
                    ok = routeOne( *context, req, aAttempt, added );
                }
            }
        }

        if( ok || !viasAllowed )
            return ok;

        for( const WAY& way : ways )
        {
            // leave on the start's layer, cross on each other layer in order of preference
            PCB_LAYER_ID start = layers[0];

            for( PCB_LAYER_ID layer : layers )
            {
                if( way.from->IsOnLayer( layer ) )
                {
                    start = layer;
                    break;
                }
            }

            PCB_LAYER_ID arrive = start;

            if( !way.to->IsOnLayer( arrive ) )
            {
                for( PCB_LAYER_ID layer : layers )
                {
                    if( way.to->IsOnLayer( layer ) )
                    {
                        arrive = layer;
                        break;
                    }
                }
            }

            const VECTOR2I              from = way.from->GetPosition();
            const std::vector<VECTOR2I> starts = spots( way.from, from, way.toPos );
            const std::vector<VECTOR2I> ends = spots( way.to, way.toPos, from );

            for( PCB_LAYER_ID cross : layers )
            {
                if( cross == start )
                    continue;

                int budget = 24;   // via pairs per crossing layer

                for( const VECTOR2I& va : starts )
                {
                    for( const VECTOR2I& vb : ends )
                    {
                        if( ok || aTries >= maxTries || budget-- <= 0 )
                            break;

                        ROUTE_REQUEST req{ way.from, way.to, way.toPos, start, PNS::RM_Walkaround, dryRun, {} };
                        req.neckWidth = neckFor( way.from, way.to );
                        req.vias = { { va, cross }, { vb, arrive } };
                        aAttempt = nlohmann::json::object();
                        aTries++;
                        ok = routeOne( *context, req, aAttempt, added );
                    }
                }

                if( ok )
                    break;
            }

            if( ok )
                break;
        }

        if( !ok && aTries >= maxTries )
            aAttempt["reason"] = "gave up after " + std::to_string( aTries ) + " tries (max_tries); last: "
                                 + aAttempt.value( "reason", std::string( "-" ) );

        return ok;
    };

    // Differential pairs: for a net named as the P side of a pair (+ or P suffix) whose N net is
    // routed too, a trunk between each two footprints next in a chain over the footprints with pads of
    // both nets (connector - ESD - MCU), from the P and N pads nearest each other there.  The trunks
    // go first and coupled; airwires their copper joins afterwards are not routed again.
    std::vector<JOB> trunks;

    if( diffPairs == "auto" && !rail )
    {
        std::set<std::string> jobNets;

        for( const JOB& job : jobs )
            jobNets.insert( job.net );

        for( const std::string& netP : jobNets )
        {
            wxString other;

            if( board->MatchDpSuffix( wxString::FromUTF8( netP ), other ) != 1 || !jobNets.count( str( other ) ) )
                continue;

            NETINFO_ITEM* infoP = board->FindNet( wxString::FromUTF8( netP ) );
            NETINFO_ITEM* infoN = board->FindNet( other );

            if( !infoP || !infoN )
                continue;

            // per footprint with pads of both nets: the P and N pads nearest each other
            std::vector<std::pair<PAD*, PAD*>> ends;

            for( FOOTPRINT* fp : board->Footprints() )
            {
                std::optional<std::pair<PAD*, PAD*>> best;
                double                               bestGap = std::numeric_limits<double>::max();

                for( PAD* p : fp->Pads() )
                {
                    if( p->GetNetCode() != infoP->GetNetCode() )
                        continue;

                    for( PAD* n : fp->Pads() )
                    {
                        if( n->GetNetCode() != infoN->GetNetCode() || !( p->GetLayerSet() & n->GetLayerSet() & LSET::AllCuMask() ).any() )
                            continue;

                        const double gapPN = ( p->GetPosition() - n->GetPosition() ).EuclideanNorm();

                        if( gapPN < bestGap )
                        {
                            bestGap = gapPN;
                            best = std::make_pair( p, n );
                        }
                    }
                }

                if( best )
                    ends.push_back( *best );
            }

            if( ends.size() < 2 )
                continue;

            // a chain: from the end farthest from the middle, each time to the nearest left
            VECTOR2I middle( 0, 0 );

            for( const auto& [p, n] : ends )
                middle += p->GetPosition() / (int) ends.size();

            std::sort( ends.begin(), ends.end(), [&]( const auto& x, const auto& y )
                       { return ( x.first->GetPosition() - middle ).EuclideanNorm() > ( y.first->GetPosition() - middle ).EuclideanNorm(); } );

            for( size_t k = 1; k < ends.size(); ++k )
            {
                std::sort( ends.begin() + k, ends.end(), [&]( const auto& x, const auto& y )
                           { return ( x.first->GetPosition() - ends[k - 1].first->GetPosition() ).EuclideanNorm()
                                    < ( y.first->GetPosition() - ends[k - 1].first->GetPosition() ).EuclideanNorm(); } );
            }

            for( size_t k = 1; k < ends.size(); ++k )
            {
                const auto& [p0, n0] = ends[k - 1];
                const auto& [p1, n1] = ends[k];
                trunks.push_back( { p0, p1, p0->m_Uuid, p1->m_Uuid, p1->GetPosition(),
                                    (double) ( p1->GetPosition() - p0->GetPosition() ).EuclideanNorm(), netP, false, true } );
                trunks.push_back( { n0, n1, n0->m_Uuid, n1->m_Uuid, n1->GetPosition(),
                                    (double) ( n1->GetPosition() - n0->GetPosition() ).EuclideanNorm(), str( other ), false, true } );
            }
        }
    }

    jobs.insert( jobs.begin(), trunks.begin(), trunks.end() );

    std::vector<int> partner( jobs.size(), -1 );

    for( size_t k = 0; k + 1 < trunks.size(); k += 2 )
    {
        partner[k] = (int) k + 1;
        partner[k + 1] = (int) k;
    }

    nlohmann::json    results = nlohmann::json::array();
    std::vector<int>  rowOf( jobs.size(), -1 );
    std::vector<bool> done( jobs.size(), false );

    for( size_t index = 0; index < jobs.size(); ++index )
    {
        JOB& job = jobs[index];
        KopenapiKeepUiAlive();

        job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
        job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

        if( !job.from || !job.to )
        {
            rowOf[index] = (int) results.size();
            results.push_back( { { "net", job.net }, { "routed", false }, { "tries", 0 },
                                 { "reason", "an end of the connection is gone (removed or replaced by an earlier route)" } } );
            continue;
        }

        if( rowOf[index] >= 0 )
            continue;   // routed with its pair

        if( partner[index] >= 0 )
        {
            const size_t   mate = partner[index];
            JOB&           other = jobs[mate];
            nlohmann::json pairAttempt;
            int            pairTries = 0;
            bool           pairOk = false;

            other.from = dynamic_cast<PAD*>( board->ResolveItem( other.fromId, true ) );
            other.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( other.toId, true ) );

            // from either end of the trunk; the pair follows the corridor a single route finds first
            // (tried, not kept)
            PAD* const trunkEnds[2][2] = { { job.from, static_cast<PAD*>( job.to ) },
                                           { static_cast<PAD*>( job.to ), job.from } };

            for( PNS::PNS_MODE mode : { PNS::RM_Walkaround, PNS::RM_Shove } )
            {
                for( PCB_LAYER_ID layer : layers )
                {
                    for( const auto& [a, b] : trunkEnds )
                    {
                        if( pairOk || !other.from || !other.to || !a->IsOnLayer( layer ) || !b->IsOnLayer( layer ) )
                            continue;

                        ROUTE_REQUEST req{ a, b, b->GetPosition(), layer, mode, dryRun, {} };
                        req.diffPair = true;

                        ROUTE_REQUEST     probe{ a, b, b->GetPosition(), layer, PNS::RM_Walkaround, true, {} };
                        nlohmann::json    probeOut;
                        std::vector<KIID> none;

                        if( routeOne( *context, probe, probeOut, none ) )
                            req.guide = segmentPath( probeOut.value( "segments", nlohmann::json::array() ), a->GetPosition() );

                        pairAttempt = nlohmann::json::object();
                        pairTries++;
                        pairOk = routeOne( *context, req, pairAttempt, added );
                    }
                }
            }

            if( pairOk )
            {
                const double length = pairAttempt["metrics"].value( "length_mm", 0.0 );

                for( const auto& [k, mine] : { std::pair<size_t, JOB*>{ index, &job }, std::pair<size_t, JOB*>{ mate, &other } } )
                {
                    nlohmann::json row = { { "net", mine->net },
                                           { "from", str( mine->from->GetParentFootprint()->GetReference() ) + "."
                                                             + str( mine->from->GetNumber() ) },
                                           { "to", str( static_cast<PAD*>( mine->to )->GetParentFootprint()->GetReference() ) + "."
                                                           + str( static_cast<PAD*>( mine->to )->GetNumber() ) },
                                           { "routed", true },
                                           { "trunk", true },
                                           { "tries", k == index ? pairTries : 0 },
                                           { "diff_pair", { { "with", k == index ? other.net : job.net } } },
                                           { "layer", pairAttempt["layer"] },
                                           { "mode", pairAttempt["mode"] },
                                           { "length_mm", length / 2 },
                                           { "vias", k == index ? pairAttempt["metrics"].value( "vias", 0 ) : 0 } };

                    row["diff_pair"].update( pairAttempt["diff_pair"] );
                    done[k] = true;
                    rowOf[k] = (int) results.size();
                    results.push_back( row );
                }

                continue;
            }

            // the airwires of both nets are routed one by one instead; the trunk rows tell why
            for( const auto& [k, mine] : { std::pair<size_t, JOB*>{ index, &job }, std::pair<size_t, JOB*>{ mate, &other } } )
            {
                rowOf[k] = (int) results.size();
                results.push_back( { { "net", mine->net },
                                     { "routed", false },
                                     { "trunk", true },
                                     { "tries", k == index ? pairTries : 0 },
                                     { "diff_pair", { { "with", k == index ? other.net : job.net } } },
                                     { "reason", "the pair could not be routed coupled ("
                                                         + pairAttempt.value( "reason", std::string( "no common layer" ) )
                                                         + "): its airwires are routed one by one" } } );
            }

            continue;
        }

        // an airwire an earlier route (a pair's trunk, a path through a pad) already joined
        if( !dryRun && job.to->Type() == PCB_PAD_T )
        {
            const auto joined = board->GetConnectivity()->GetConnectedItems( job.from );

            if( std::find( joined.begin(), joined.end(), job.to ) != joined.end() )
            {
                done[index] = true;
                rowOf[index] = (int) results.size();
                results.push_back( { { "net", job.net },
                                     { "from", str( job.from->GetParentFootprint()->GetReference() ) + "." + str( job.from->GetNumber() ) },
                                     { "to", str( static_cast<PAD*>( job.to )->GetParentFootprint()->GetReference() ) + "."
                                                     + str( static_cast<PAD*>( job.to )->GetNumber() ) },
                                     { "routed", true },
                                     { "tries", 0 },
                                     { "joined_by_earlier_route", true },
                                     { "length_mm", 0.0 } } );
                continue;
            }
        }

        nlohmann::json attempt;
        int            tries = 0;
        const bool     ok = tryRoute( job, attempt, tries, true, false );

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
            done[index] = true;
            row["layer"] = attempt["layer"];
            row["mode"] = attempt["mode"];
            row["width_mm"] = attempt["track_width_mm"];
            row["length_mm"] = attempt["metrics"]["length_mm"];
            row["vias"] = attempt["metrics"].value( "vias", 0 );
            row["detour"] = attempt["metrics"]["detour"];

            if( attempt.contains( "necked_from_mm" ) )
                row["necked_from_mm"] = attempt["necked_from_mm"];
        }
        else
        {
            row["reason"] = attempt.value( "reason", std::string() );

            for( const char* key : { "violations", "stopped_at_mm", "blocked_by" } )
            {
                if( attempt.contains( key ) )
                    row[key] = attempt[key];
            }
        }

        rowOf[index] = (int) results.size();
        results.push_back( row );
    }

    // Rip-up and retry: for a connection that failed, take out the routes of this call that cross
    // its corridor, route it, route them again; if anything of that fails, everything goes back
    // as it was (walkaround only: nothing else is pushed aside, so the restore is exact)
    const int ripRounds = dryRun ? 0 : std::clamp( aArgs.value( "rip_up", 2 ), 0, 6 );
    int       rippedOk = 0, rippedTried = 0;

    auto chainOf = [&]( JOB& aJob ) -> std::vector<PCB_TRACK*>
    {
        PAD* a = dynamic_cast<PAD*>( board->ResolveItem( aJob.fromId, true ) );
        PAD* b = dynamic_cast<PAD*>( board->ResolveItem( aJob.toId, true ) );

        if( !a || !b )
            return {};

        return copperBetween( board, a, b );
    };

    for( int round = 0; round < ripRounds; ++round )
    {
        bool progress = false;

        for( size_t index = 0; index < jobs.size(); ++index )
        {
            if( done[index] || rowOf[index] < 0 || jobs[index].trunk )
                continue;

            JOB& job = jobs[index];
            job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
            job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

            if( !job.from || !job.to )
                continue;

            KopenapiKeepUiAlive();

            // first from both ends, nothing taken out
            if( round == 0 )
            {
                nlohmann::json attempt;
                int            tries = 0;

                if( tryRoute( job, attempt, tries, false, true ) )
                {
                    progress = true;
                    done[index] = true;

                    nlohmann::json& row = results[rowOf[index]];
                    row["routed"] = true;
                    row.erase( "reason" );
                    row.erase( "violations" );
                    row.erase( "stopped_at_mm" );
                    row.erase( "blocked_by" );
                    row["retried"] = true;
                    row["layer"] = attempt["layer"];
                    row["mode"] = attempt["mode"];
                    row["length_mm"] = attempt["metrics"]["length_mm"];
                    row["vias"] = attempt["metrics"].value( "vias", 0 );
                    row["detour"] = attempt["metrics"]["detour"];
                    continue;
                }
            }

            // copper of the pairs' trunks is never taken out
            std::set<PCB_TRACK*> trunkCopper;

            for( size_t t = 0; t < trunks.size(); ++t )
            {
                if( done[t] )
                {
                    for( PCB_TRACK* track : chainOf( jobs[t] ) )
                        trunkCopper.insert( track );
                }
            }

            // routes of this call in the corridor of the failed connection, the nearest three
            BOX2I corridor( job.from->GetPosition(), VECTOR2I( 0, 0 ) );
            corridor.Merge( job.toPos );
            corridor.Inflate( pcbIUScale.mmToIU( 1.5 ) );

            std::vector<std::pair<double, size_t>> crossing;

            for( size_t other = 0; other < jobs.size(); ++other )
            {
                if( !done[other] || partner[other] >= 0 || jobs[other].to->Type() != PCB_PAD_T
                    || results[rowOf[other]].value( "joined_by_earlier_route", false ) )
                {
                    continue;   // pairs stay; a joined airwire has no copper of its own
                }

                const std::vector<PCB_TRACK*> chain = chainOf( jobs[other] );

                if( std::any_of( chain.begin(), chain.end(), [&]( PCB_TRACK* t ) { return trunkCopper.count( t ); } ) )
                    continue;

                for( PCB_TRACK* t : chain )
                {
                    if( !t->IsLocked() && t->GetBoundingBox().Intersects( corridor ) )
                    {
                        crossing.emplace_back( ( t->GetBoundingBox().Centre() - corridor.Centre() ).EuclideanNorm(), other );
                        break;
                    }
                }
            }

            if( crossing.empty() )
                continue;

            std::sort( crossing.begin(), crossing.end() );

            if( crossing.size() > 3 )
                crossing.resize( 3 );

            rippedTried++;

            // take them out, keeping copies to put back
            std::vector<size_t>                     victims;
            std::vector<std::unique_ptr<PCB_TRACK>> saved;
            BOARD_COMMIT                            rip( context->GetToolManager() );

            std::set<PCB_TRACK*> taken;   // victims of one net can share copper: each piece once

            for( const auto& [d, other] : crossing )
            {
                victims.push_back( other );

                for( PCB_TRACK* t : chainOf( jobs[other] ) )
                {
                    if( !taken.insert( t ).second )
                        continue;

                    saved.emplace_back( static_cast<PCB_TRACK*>( t->Clone() ) );
                    rip.Remove( t );
                }
            }

            rip.Push( _( "Rip up (API)" ) );
            const size_t firstNew = added.size();

            nlohmann::json attempt;
            int            tries = 0;
            bool           ok = tryRoute( job, attempt, tries, false, true );
            nlohmann::json victimAttempts = nlohmann::json::array();

            for( size_t v : victims )
            {
                if( !ok )
                    break;

                JOB& victim = jobs[v];
                victim.from = dynamic_cast<PAD*>( board->ResolveItem( victim.fromId, true ) );
                victim.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( victim.toId, true ) );
                nlohmann::json again;
                int            againTries = 0;

                ok = victim.from && victim.to && tryRoute( victim, again, againTries, false, true );
                victimAttempts.push_back( again );
            }

            if( ok )
            {
                rippedOk++;
                progress = true;
                done[index] = true;

                nlohmann::json& row = results[rowOf[index]];
                row["routed"] = true;
                row.erase( "reason" );
                row.erase( "violations" );
                row.erase( "stopped_at_mm" );
                row.erase( "blocked_by" );
                row["after_rip_up"] = true;
                row["layer"] = attempt["layer"];
                row["mode"] = attempt["mode"];
                row["length_mm"] = attempt["metrics"]["length_mm"];
                row["vias"] = attempt["metrics"].value( "vias", 0 );
                row["detour"] = attempt["metrics"]["detour"];

                for( size_t k = 0; k < victims.size(); ++k )
                {
                    nlohmann::json& vrow = results[rowOf[victims[k]]];
                    vrow["rerouted"] = true;
                    vrow["length_mm"] = victimAttempts[k]["metrics"]["length_mm"];
                    vrow["vias"] = victimAttempts[k]["metrics"].value( "vias", 0 );
                    vrow["layer"] = victimAttempts[k]["layer"];
                }

                continue;
            }

            // put everything back: the new copper out, the copies in
            BOARD_COMMIT undo( context->GetToolManager() );

            for( size_t k = firstNew; k < added.size(); ++k )
            {
                if( BOARD_ITEM* item = board->ResolveItem( added[k], true ) )
                    undo.Remove( item );
            }

            added.resize( firstNew );

            for( std::unique_ptr<PCB_TRACK>& t : saved )
                undo.Add( t.release() );

            undo.Push( _( "Rip up undone (API)" ) );
            board->GetConnectivity()->RecalculateRatsnest();
        }

        if( !progress )
            break;
    }

    // Gaps between two pieces of a net's copper: from the end of one piece to the other
    for( const GAP& gap : gaps )
    {
        KopenapiKeepUiAlive();

        auto* a = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( gap.a, true ) );
        auto* b = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( gap.b, true ) );

        auto kind = []( BOARD_ITEM* aItem ) -> std::string
        {
            return !aItem ? "gone" : aItem->Type() == PCB_VIA_T ? "via" : aItem->Type() == PCB_PAD_T ? "pad" : "track";
        };

        nlohmann::json row = { { "net", gap.net },
                               { "gap", { { "from_mm", { toMm( gap.aPos.x ), toMm( gap.aPos.y ) } },
                                          { "to_mm", { toMm( gap.bPos.x ), toMm( gap.bPos.y ) } },
                                          { "from", kind( board->ResolveItem( gap.a, true ) ) },
                                          { "to", kind( board->ResolveItem( gap.b, true ) ) } } },
                               { "routed", false },
                               { "tries", 0 } };

        if( !a || !b )
        {
            row["reason"] = "an end of the gap is gone (taken out by a route of this call)";
            results.push_back( row );
            continue;
        }

        const auto joined = board->GetConnectivity()->GetConnectedItems( a );

        if( std::find( joined.begin(), joined.end(), b ) != joined.end() )
        {
            row["routed"] = true;
            row["joined_by_earlier_route"] = true;
            row["length_mm"] = 0.0;
            results.push_back( row );
            continue;
        }

        nlohmann::json attempt;
        int            tries = 0;
        bool           ok = false;

        for( PNS::PNS_MODE mode : { PNS::RM_Walkaround, PNS::RM_Shove } )
        {
            for( PCB_LAYER_ID layer : layers )
            {
                if( ok || tries >= maxTries || !a->IsOnLayer( layer ) || !b->IsOnLayer( layer ) )
                    continue;

                ROUTE_REQUEST req{ a, b, gap.bPos, layer, mode, dryRun, {} };
                req.fromPos = gap.aPos;
                attempt = nlohmann::json::object();
                tries++;
                ok = routeOne( *context, req, attempt, added );
            }
        }

        row["routed"] = ok;
        row["tries"] = tries;

        if( ok )
        {
            row["layer"] = attempt["layer"];
            row["mode"] = attempt["mode"];
            row["width_mm"] = attempt["track_width_mm"];
            row["length_mm"] = attempt["metrics"]["length_mm"];
            row["vias"] = attempt["metrics"].value( "vias", 0 );
        }
        else
        {
            row["reason"] = tries ? attempt.value( "reason", std::string() ) : "the two pieces share no routing layer";

            for( const char* key : { "violations", "stopped_at_mm", "blocked_by" } )
            {
                if( attempt.contains( key ) )
                    row[key] = attempt[key];
            }
        }

        results.push_back( row );
    }

    if( !unfilled.empty() )
    {
        BOARD_COMMIT fillCommit( context->GetToolManager() );
        ZONE_FILLER  filler( board, &fillCommit );
        KOPENAPI_KEEPALIVE_REPORTER reporter;
        filler.SetProgressReporter( &reporter );

        if( filler.Fill( unfilled ) )
            fillCommit.Push( _( "Fill zones (API)" ) );
        else
            fillCommit.Revert();

        board->BuildConnectivity();
    }

    // copper of this call that leads nowhere (left by rip-ups, router slivers) taken out
    int stubs = 0;

    for( bool again = !dryRun; again; )
    {
        again = false;
        board->BuildConnectivity();
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
        BOARD_COMMIT                       sweep( context->GetToolManager() );

        for( const KIID& id : added )
        {
            PCB_TRACK* track = dynamic_cast<PCB_TRACK*>( board->ResolveItem( id, true ) );

            if( !track || track->IsLocked() )
                continue;

            if( conn->TestTrackEndpointDangling( track, true ) )
            {
                sweep.Remove( track );
                stubs++;
                again = true;
            }
        }

        if( again )
            sweep.Push( _( "Remove dangling copper (API)" ) );
        else
            sweep.Revert();
    }

    // the rows as the copper is now: a routed one still joined (a later rip-up can cut a chain it
    // shared), a failed one perhaps joined by a later route
    if( !dryRun )
    {
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();

        for( size_t index = 0; index < jobs.size(); ++index )
        {
            if( rowOf[index] < 0 )
                continue;

            nlohmann::json& row = results[rowOf[index]];
            BOARD_ITEM*     from = board->ResolveItem( jobs[index].fromId, true );
            BOARD_ITEM*     to = board->ResolveItem( jobs[index].toId, true );

            if( jobs[index].trunk || !from || !to || from->Type() != PCB_PAD_T || to->Type() != PCB_PAD_T )
                continue;

            const auto joined = conn->GetConnectedItems( static_cast<BOARD_CONNECTED_ITEM*>( from ) );
            const bool isJoined = std::find( joined.begin(), joined.end(), to ) != joined.end();

            if( row.value( "routed", false ) && !isJoined )
            {
                row["routed"] = false;
                row["reason"] = "cut by a later rip-up";
            }
            else if( !row.value( "routed", false ) && isJoined )
            {
                // a later route of the net (through these pads) joined it
                row["routed"] = true;
                row["joined_by_later_route"] = true;
                row["length_mm"] = 0.0;
                row["vias"] = 0;
                row.erase( "reason" );
            }
        }
    }

    // connections are the airwires; the trunks add their copper
    int    routed = 0, vias = 0;
    double length = 0;

    for( const nlohmann::json& row : results )
    {
        if( row.value( "routed", false ) )
        {
            routed += row.value( "trunk", false ) ? 0 : 1;
            length += row.value( "length_mm", 0.0 );
            vias += row.value( "vias", 0 );
        }
    }

    const int connections = (int) ( jobs.size() - trunks.size() + gaps.size() );

    if( stepMs == 0 )
        showRoutes( aCtx, added );

    connectivity = board->GetConnectivity();
    connectivity->RecalculateRatsnest();

    return KOPENAPI_RESULT::Ok( { { "connections", connections },
                                  { "routed", routed },
                                  { "failed", connections - routed },
                                  { "length_mm", std::round( length * 1000.0 ) / 1000.0 },
                                  { "vias", vias },
                                  { "rip_up", { { "tried", rippedTried }, { "won", rippedOk } } },
                                  { "stubs_removed", stubs },
                                  { "kept_out_under", keptOut },
                                  { "plane_layers_skipped", planeLayers },
                                  { "dry_run", dryRun },
                                  { "unrouted_left", dryRun ? connections - routed : (int) connectivity->GetUnconnectedCount( false ) },
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

    // Passes: place, refill, then take out the vias the fill does not reach (a plane island too small
    // to keep, a knockout); their pads are tried again at other spots
    struct STITCHED
    {
        BOARD_CONNECTED_ITEM* item;
        KIID                  via;
        KIID                  track;
        VECTOR2I              at;
    };

    std::set<std::pair<int, int>> bad;
    std::map<BOARD_CONNECTED_ITEM*, nlohmann::json> misses;   // ends still without a via, and why

    for( int pass = 0; pass < 3; ++pass )
    {
        std::vector<STITCHED> fresh_vias;

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

                    if( !guard( board, fresh, {}, true ).empty() )
                        continue;

                    BOARD_COMMIT commit( context->GetToolManager() );
                    const bool   hasTrack = !!track;
                    added.push_back( via->m_Uuid );
                    commit.Add( via.release() );

                    if( track )
                    {
                        added.push_back( track->m_Uuid );
                        commit.Add( track.release() );
                    }

                    commit.Push( _( "Stitching via (API)" ) );
                    fresh_vias.push_back( { item, added.end()[hasTrack ? -2 : -1], hasTrack ? added.back() : niluuid, at } );
                    placed.push_back( { { "via", str( fresh_vias.back().via.AsString() ) },
                                        { "at_mm", { toMm( at.x ), toMm( at.y ) } },
                                        { "for", item->Type() == PCB_PAD_T
                                                         ? str( static_cast<PAD*>( item )->GetParentFootprint()->GetReference() ) + "."
                                                                   + str( static_cast<PAD*>( item )->GetNumber() )
                                                         : std::string( "zone island" ) } } );
                    done.insert( item );
                    ok = true;
                }
            }

            if( !ok )
            {
                misses[item] = { { "for", item->Type() == PCB_PAD_T
                                              ? str( static_cast<PAD*>( item )->GetParentFootprint()->GetReference() ) + "."
                                                        + str( static_cast<PAD*>( item )->GetNumber() )
                                              : std::string( "zone island" ) },
                                 { "at_mm", { toMm( pos.x ), toMm( pos.y ) } },
                                 { "reason", candidates.empty() ? "no spot to try (the island is too small for a via)"
                                                                : "no free spot within reach: every spot near it touches "
                                                                  "other copper, a hole or the board edge" } };
            }
        }

        refill();

        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
        BOARD_COMMIT                       drop( context->GetToolManager() );
        bool                               dropped = false;

        for( const STITCHED& st : fresh_vias )
        {
            PCB_VIA* via = dynamic_cast<PCB_VIA*>( board->ResolveItem( st.via, true ) );

            if( !via )
                continue;

            const std::vector<BOARD_CONNECTED_ITEM*> joined = conn->GetConnectedItems( via );

            if( std::any_of( joined.begin(), joined.end(), []( BOARD_CONNECTED_ITEM* i ) { return i->Type() == PCB_ZONE_T; } ) )
                continue;

            drop.Remove( via );

            if( BOARD_ITEM* track = board->ResolveItem( st.track, true ) )
                drop.Remove( track );

            bad.insert( { st.at.x, st.at.y } );
            done.erase( st.item );
            misses[st.item] = { { "for", st.item->Type() == PCB_PAD_T
                                             ? str( static_cast<PAD*>( st.item )->GetParentFootprint()->GetReference() ) + "."
                                                       + str( static_cast<PAD*>( st.item )->GetNumber() )
                                             : std::string( "zone island" ) },
                                { "at_mm", { toMm( st.at.x ), toMm( st.at.y ) } },
                                { "reason", "the via there was off the plane after the refill (an island too small to keep)" } };
            for( auto it = placed.begin(); it != placed.end(); ++it )
            {
                if( ( *it )["via"] == str( st.via.AsString() ) )
                {
                    placed.erase( it );
                    break;
                }
            }
            std::erase( added, st.via );
            std::erase( added, st.track );
            dropped = true;
        }

        if( !dropped )
        {
            drop.Revert();
            break;
        }

        drop.Push( _( "Stitching vias off the plane removed (API)" ) );
        refill();
    }

    // vias of the net that a refill left touching one layer only (an island cut off by later routing),
    // with the stubs that led to them
    int danglingRemoved = 0;

    {
        board->BuildConnectivity();
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
        BOARD_COMMIT                       tidy( context->GetToolManager() );
        std::set<std::pair<int, int>>      gone;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() == PCB_VIA_T && t->GetNetCode() == net->GetNetCode() && !t->IsLocked()
                && conn->TestTrackEndpointDangling( t, true ) )
            {
                tidy.Remove( t );
                gone.insert( { t->GetPosition().x, t->GetPosition().y } );
                danglingRemoved++;
            }
        }

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() != PCB_VIA_T && t->GetNetCode() == net->GetNetCode() && !t->IsLocked()
                && ( gone.count( { t->GetStart().x, t->GetStart().y } ) || gone.count( { t->GetEnd().x, t->GetEnd().y } ) ) )
            {
                tidy.Remove( t );
            }
        }

        if( danglingRemoved )
        {
            tidy.Push( _( "Remove dangling vias (API)" ) );
            refill();
        }
        else
        {
            tidy.Revert();
        }
    }

    showRoutes( aCtx, added );

    return KOPENAPI_RESULT::Ok( { { "net", str( netName ) },
                                  { "vias", placed },
                                  { "dangling_vias_removed", danglingRemoved },
                                  { "not_reached", [&]()
                                    {
                                        nlohmann::json out = nlohmann::json::array();

                                        for( const auto& [item, why] : misses )
                                        {
                                            if( !done.count( item ) )
                                                out.push_back( why );
                                        }

                                        std::sort( out.begin(), out.end(), []( const nlohmann::json& x, const nlohmann::json& y )
                                                   { return x.dump() < y.dump(); } );
                                        return out;
                                    }() },
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

    if( aNeedSelector && nets.empty() && !between && !region && !aArgs.value( "all", false ) )
    {
        aError = KOPENAPI_RESULT::Error( 400, "say which copper: uuids, net / nets, between [\"REF.PAD\", \"REF.PAD\"], "
                                              "region [x0, y0, x1, y1] or all: true (layer and kinds narrow it further)" );
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

    if( steps.size() < 2 && !( steps.size() == 1 && steps[0].via ) )
        return KOPENAPI_RESULT::Error( 400, "a track needs two points at least: from / points / to (or one point with "
                                            "via: a via alone, pcb_via_add)" );

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
    const int  viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.0 ) )
                        : netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : pcbIUScale.mmToIU( 0.8 );
    const int  viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.0 ) )
                          : netclass && netclass->HasViaDrill() ? netclass->GetViaDrill() : pcbIUScale.mmToIU( 0.4 );

    if( viaDia < bds.m_ViasMinSize || viaDrill < bds.m_MinThroughDrill || viaDrill >= viaDia )
        return KOPENAPI_RESULT::Error( 422, "via " + std::to_string( toMm( viaDia ) ).substr( 0, 5 ) + " / "
                                                    + std::to_string( toMm( viaDrill ) ).substr( 0, 5 )
                                                    + " mm: below the board's minimum via diameter / drill or a drill "
                                                      "not smaller than the via" );

    if( width < bds.m_TrackMinWidth )
        return KOPENAPI_RESULT::Error( 422, "width_mm is below the board's minimum track width "
                                                    + std::to_string( toMm( bds.m_TrackMinWidth ) ).substr( 0, 5 ) + " mm" );

    std::vector<std::unique_ptr<PCB_TRACK>> made;
    std::vector<BOARD_CONNECTED_ITEM*>      changed;
    double                                  length = 0;
    int                                     vias = 0;
    nlohmann::json                          segments = nlohmann::json::array();

    auto dropVia = [&]( const VECTOR2I& aAt )
    {
        auto via = std::make_unique<PCB_VIA>( board );
        via->SetViaType( VIATYPE::THROUGH );
        via->SetPosition( aAt );
        via->SetWidth( PADSTACK::ALL_LAYERS, viaDia );
        via->SetDrill( viaDrill );
        via->SetLayerPair( F_Cu, B_Cu );
        via->SetNet( net );
        vias++;
        changed.push_back( via.get() );
        made.push_back( std::move( via ) );
    };

    // a via on the first point (a via alone, or a start on the other side)
    if( !steps.empty() && steps[0].via )
    {
        dropVia( steps[0].at );
        layer = *steps[0].via;
    }

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
            dropVia( steps[i].at );
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

    const std::string zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (pour fills give way, refilled after) or keep" );

    const nlohmann::json violations = guard( board, changed, {}, zoneMode == "refill" );

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

    if( zoneMode == "refill" )
    {
        refillPours( *context, board );
        out["zones_refilled"] = std::any_of( board->Zones().begin(), board->Zones().end(),
                                             []( ZONE* z ) { return z->IsFilled() && !z->GetIsRuleArea(); } );
    }

    showRoutes( aCtx, added );

    out["applied"] = true;
    out["uuids"] = uuids;
    out["unrouted_left"] = board->GetConnectivity()->GetUnconnectedCount( false );
    return KOPENAPI_RESULT::Ok( out );
}


/// @brief A via alone (a point on a net), or with a short track from a pad: pcb_track_add's path of
/// one point with a via, or of the pad and that point
static KOPENAPI_RESULT h_pcb_via_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const nlohmann::json at = aArgs.value( "at_mm", nlohmann::json() );

    if( !at.is_array() || at.size() != 2 || !at[0].is_number() || !at[1].is_number() )
        return KOPENAPI_RESULT::Error( 400, "at_mm: [x_mm, y_mm]" );

    nlohmann::json args = nlohmann::json::object();

    for( const char* key : { "net", "from", "dry_run", "zones", "width_mm", "layer" } )
    {
        if( aArgs.contains( key ) )
            args[key] = aArgs[key];
    }

    if( aArgs.contains( "diameter_mm" ) )
        args["via_diameter_mm"] = aArgs["diameter_mm"];

    if( aArgs.contains( "drill_mm" ) )
        args["via_drill_mm"] = aArgs["drill_mm"];

    // the via's other side: B.Cu from a top track (or a lone via), F.Cu from a bottom one
    const std::string other = aArgs.value( "layer", std::string( "F.Cu" ) ) == "B.Cu" ? "F.Cu" : "B.Cu";
    args["points"] = nlohmann::json::array( { { { "at", at }, { "via", other } } } );

    return h_pcb_track_add( aCtx, args );
}


/// @brief Fan an area-array package (BGA, LGA) out as the reference designs do: a dog-bone per ball -
/// a short track from the ball to a through via on the diagonal between four balls, pointing away
/// from the package centre (quadrant fan-out), another diagonal when that spot is taken.  Balls of
/// single-pad nets are left alone, and so are the outer rings that leave on the ball's layer.  Via and
/// stub sizes follow the pitch unless given; the guard and the board's minimums apply; pours give way
/// and are refilled; one undo step.
static KOPENAPI_RESULT h_pcb_fanout( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    const wxString ref = wxString::FromUTF8( aArgs.value( "ref", std::string() ) );
    FOOTPRINT*     fp = nullptr;

    for( FOOTPRINT* f : board->Footprints() )
    {
        if( f->GetReference() == ref )
            fp = f;
    }

    if( !fp )
        return KOPENAPI_RESULT::Error( 404, "ref: no footprint " + aArgs.value( "ref", std::string() ) );

    std::vector<PAD*> balls;

    for( PAD* pad : fp->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::SMD )
            balls.push_back( pad );
    }

    if( balls.size() < 9 )
        return KOPENAPI_RESULT::Error( 422, "not an area-array package (fewer than 9 SMD pads)" );

    // the pitch: the median distance to the nearest ball
    std::vector<double> nearest;

    for( PAD* a : balls )
    {
        double best = std::numeric_limits<double>::max();

        for( PAD* b : balls )
        {
            if( a != b )
                best = std::min( best, double( ( a->GetPosition() - b->GetPosition() ).EuclideanNorm() ) );
        }

        nearest.push_back( best );
    }

    std::nth_element( nearest.begin(), nearest.begin() + nearest.size() / 2, nearest.end() );
    const int pitch = KiROUND( nearest[nearest.size() / 2] );

    BOX2I span( balls.front()->GetPosition(), VECTOR2I( 0, 0 ) );

    for( PAD* b : balls )
        span.Merge( b->GetPosition() );

    const VECTOR2I centre = span.Centre();
    const int      cols = KiROUND( double( span.GetWidth() ) / pitch );
    const int      rows = KiROUND( double( span.GetHeight() ) / pitch );

    // an array: balls on most points of a grid of three or more rows and columns (DDR packages leave a
    // corridor in the middle), and inside its outer ring too (a small QFN / QFP ring fills half its grid)
    int inner = 0;

    for( PAD* b : balls )
    {
        const VECTOR2I d = b->GetPosition() - span.GetOrigin();

        if( d.x > pitch / 2 && d.y > pitch / 2 && span.GetWidth() - d.x > pitch / 2 && span.GetHeight() - d.y > pitch / 2 )
            inner++;
    }

    if( cols < 2 || rows < 2 || double( balls.size() ) / ( ( cols + 1 ) * ( rows + 1 ) ) < 0.4
        || double( inner ) / ( ( cols - 1 ) * ( rows - 1 ) ) < 0.25 )
        return KOPENAPI_RESULT::Error( 422, "not an area-array package (pads on a ring, or too few rows): a QFP / QFN fans out "
                                            "with pcb_route" );

    // sizes from the pitch (the reference designs: 0.5 mm 0.28 / 0.15, 0.8 mm 0.4 - 0.5 / 0.2 - 0.25,
    // 1.0 mm 0.5 / 0.2), the board's minimums as a floor
    const BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    const double                 pitchMm = pitch / pcbIUScale.IU_PER_MM;
    const double defDia = pitchMm < 0.6 ? 0.3 : pitchMm < 0.7 ? 0.35 : pitchMm < 0.9 ? 0.45 : 0.5;
    const double defDrill = pitchMm < 0.6 ? 0.15 : pitchMm < 0.9 ? 0.2 : 0.25;
    // the board's rules at a via between the balls (custom rules under a BGA count): smallest via,
    // drill and ring, and the clearance a via of one net keeps to a ball of another
    int minDia = bds.m_ViasMinSize, minDrill = bds.m_MinThroughDrill, minRing = bds.m_ViasMinAnnularWidth;
    int clearance = std::max( bds.m_MinClearance, 1 );
    int ballRadius = 0;

    for( PAD* b : balls )
        ballRadius = std::max( ballRadius, (int) std::max( b->GetBoundingBox().GetWidth(), b->GetBoundingBox().GetHeight() ) / 2 );

    if( std::shared_ptr<DRC_ENGINE> drc = bds.m_DRCEngine )
    {
        PAD*    a = balls.front();
        PAD*    other = nullptr;

        for( PAD* b : balls )
        {
            if( b->GetNetCode() != a->GetNetCode() )
                other = b;
        }

        PCB_VIA probe( board );
        probe.SetViaType( VIATYPE::THROUGH );
        probe.SetLayerPair( F_Cu, B_Cu );
        probe.SetPosition( a->GetPosition() + VECTOR2I( pitch / 2, pitch / 2 ) );
        probe.SetNet( a->GetNet() );

        auto minOf = [&]( DRC_CONSTRAINT_T aType, const BOARD_ITEM* aOther, int aFallback )
        {
            DRC_CONSTRAINT c = drc->EvalRules( aType, &probe, aOther, F_Cu );
            return c.GetValue().HasMin() ? c.GetValue().Min() : aFallback;
        };

        minDia = minOf( VIA_DIAMETER_CONSTRAINT, nullptr, minDia );
        minDrill = minOf( HOLE_SIZE_CONSTRAINT, nullptr, minDrill );
        minRing = minOf( ANNULAR_WIDTH_CONSTRAINT, nullptr, minRing );

        if( other )
            clearance = std::max( 1, minOf( CLEARANCE_CONSTRAINT, other, clearance ) );
    }

    // the room between four balls: half the diagonal holds half the via, a ball and the clearance
    const int room = KiROUND( pitch / M_SQRT2 );
    const int fitDia = 2 * ( room - ballRadius - clearance );

    int viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", defDia ) )
                                                   : std::max( minDia, std::min( pcbIUScale.mmToIU( defDia ), fitDia ) );
    int viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", defDrill ) )
                                                    : std::max( minDrill, std::min( pcbIUScale.mmToIU( defDrill ), viaDia - 2 * std::max( minRing, 1 ) ) );

    // vias already under the package: the design's own size (the most common one), when its rules allow it
    if( !aArgs.contains( "via_diameter_mm" ) && !aArgs.contains( "via_drill_mm" ) )
    {
        std::map<std::pair<int, int>, int> sizes;
        BOX2I                              under = span;
        under.Inflate( pitch );

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() == PCB_VIA_T && under.Contains( t->GetPosition() ) )
            {
                PCB_VIA* v = static_cast<PCB_VIA*>( t );
                sizes[{ v->GetWidth( F_Cu ), v->GetDrillValue() }]++;
            }
        }

        if( !sizes.empty() )
        {
            const auto common = std::max_element( sizes.begin(), sizes.end(),
                                                  []( const auto& x, const auto& y ) { return x.second < y.second; } );

            const auto [dia, drill] = common->first;

            if( dia <= fitDia && dia >= minDia && drill >= minDrill && drill + 2 * minRing <= dia )
            {
                viaDia = dia;
                viaDrill = drill;
            }
        }
    }

    if( viaDia > fitDia )
        return KOPENAPI_RESULT::Error( 422, "a via of " + std::to_string( toMm( viaDia ) ).substr( 0, 5 ) + " mm does not fit between the "
                                                    "balls (pitch " + std::to_string( toMm( pitch ) ).substr( 0, 5 ) + " mm, ball "
                                                    + std::to_string( toMm( 2 * ballRadius ) ).substr( 0, 5 ) + " mm, clearance "
                                                    + std::to_string( toMm( clearance ) ).substr( 0, 5 ) + " mm: at most "
                                                    + std::to_string( toMm( std::max( 0, fitDia ) ) ).substr( 0, 5 ) + " mm), the board's "
                                                    "rules allow down to " + std::to_string( toMm( minDia ) ).substr( 0, 5 ) + " mm: a rule for "
                                                    "smaller vias / clearance under the package, or via-in-pad (not built yet)" );

    if( viaDia < minDia || viaDrill < minDrill || viaDrill + 2 * minRing > viaDia )
        return KOPENAPI_RESULT::Error( 422, "via " + std::to_string( toMm( viaDia ) ).substr( 0, 5 ) + " / "
                                                    + std::to_string( toMm( viaDrill ) ).substr( 0, 5 )
                                                    + " mm is below the board's rules (diameter / drill / ring): lower "
                                                      "constraints in pcb_rules_set, or give via_diameter_mm / via_drill_mm" );

    const int defaultDirect = pitchMm >= 0.75 ? 1 : 0;
    const int direct = std::clamp( aArgs.value( "outer_rings_direct", defaultDirect ), 0, 20 );
    const bool dryRun = aArgs.value( "dry_run", false );
    const std::string zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (pour fills give way, refilled after) or keep" );

    std::set<std::string> onlyNets;

    for( const nlohmann::json& n : aArgs.value( "nets", nlohmann::json::array() ) )
    {
        if( n.is_string() )
            onlyNets.insert( n.get<std::string>() );
    }

    // pads per net on the board: a ball on a net of its own has nothing to reach
    std::map<int, int> netPads;

    for( FOOTPRINT* f : board->Footprints() )
    {
        for( PAD* pad : f->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    std::set<std::pair<int, int>> spots;   // via spots taken (this call and vias already there)

    for( PCB_TRACK* t : board->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
            spots.insert( { t->GetPosition().x, t->GetPosition().y } );
    }

    BOARD_COMMIT              commit( context->GetToolManager() );
    std::vector<KIID>         added;
    std::vector<BOARD_CONNECTED_ITEM*> staged;   // vias and stubs of this call, owned by the commit
    std::vector<std::unique_ptr<BOARD_CONNECTED_ITEM>> planned;   // the same in a dry run
    nlohmann::json            placed = nlohmann::json::array();
    nlohmann::json            skipped = nlohmann::json::array();
    std::map<std::string, int> why;
    const int                 half = pitch / 2;

    // inner rings first: they have the fewest choices
    std::vector<std::pair<int, PAD*>> order;

    for( PAD* b : balls )
    {
        const int col = KiROUND( double( b->GetPosition().x - span.GetLeft() ) / pitch );
        const int row = KiROUND( double( b->GetPosition().y - span.GetTop() ) / pitch );
        order.emplace_back( std::min( { col, row, cols - col, rows - row } ), b );
    }

    std::stable_sort( order.begin(), order.end(), []( const auto& x, const auto& y ) { return x.first > y.first; } );

    auto skip = [&]( PAD* aBall, int aRing, const std::string& aWhy )
    {
        skipped.push_back( { { "ball", str( fp->GetReference() ) + "." + str( aBall->GetNumber() ) }, { "ring", aRing }, { "reason", aWhy } } );
        why[aWhy]++;
    };

    for( const auto& [ring, ball] : order )
    {
        const std::string net = str( UnescapeString( ball->GetNetname() ) );

        if( ball->GetNetCode() <= 0 || netPads[ball->GetNetCode()] < 2 )
        {
            skip( ball, ring, "no other pad on its net" );
            continue;
        }

        if( !onlyNets.empty() && !onlyNets.count( net ) )
            continue;

        if( ring < direct )
        {
            skip( ball, ring, "outer ring: leaves on its layer (outer_rings_direct)" );
            continue;
        }

        const PCB_LAYER_ID layer = ball->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
        const VECTOR2I     at = ball->GetPosition();
        const int          sx = at.x >= centre.x ? 1 : -1;
        const int          sy = at.y >= centre.y ? 1 : -1;
        const VECTOR2I     dirs[4] = { { sx, sy }, { sx, -sy }, { -sx, sy }, { -sx, -sy } };
        bool               done = false;
        std::string        last = "every diagonal spot taken";

        NETCLASS* netclass = ball->GetEffectiveNetClass();
        int       width = aArgs.contains( "width_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "width_mm", 0.1 ) )
                                                       : ( netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : bds.m_TrackMinWidth );
        width = std::max( bds.m_TrackMinWidth, std::min( width, fitDia ) );   // the stub passes two balls as the via does

        // a rule of the net's own (impedance-controlled pairs) may want a wider track
        if( std::shared_ptr<DRC_ENGINE> drc = bds.m_DRCEngine; drc && !aArgs.contains( "width_mm" ) )
        {
            PCB_TRACK probe( board );
            probe.SetStart( ball->GetPosition() );
            probe.SetEnd( ball->GetPosition() + VECTOR2I( half, half ) );
            probe.SetLayer( ball->IsOnLayer( F_Cu ) ? F_Cu : B_Cu );
            probe.SetNet( ball->GetNet() );
            DRC_CONSTRAINT c = drc->EvalRules( TRACK_WIDTH_CONSTRAINT, &probe, nullptr, probe.GetLayer() );

            if( c.GetValue().HasMin() )
                width = std::max( width, c.GetValue().Min() );
        }

        for( const VECTOR2I& d : dirs )
        {
            const VECTOR2I spot = at + VECTOR2I( d.x * half, d.y * half );

            if( spots.count( { spot.x, spot.y } ) )
                continue;

            auto via = std::make_unique<PCB_VIA>( board );
            via->SetViaType( VIATYPE::THROUGH );
            via->SetPosition( spot );
            via->SetWidth( PADSTACK::ALL_LAYERS, viaDia );
            via->SetDrill( viaDrill );
            via->SetLayerPair( F_Cu, B_Cu );
            via->SetNet( ball->GetNet() );

            auto stub = std::make_unique<PCB_TRACK>( board );
            stub->SetStart( at );
            stub->SetEnd( spot );
            stub->SetWidth( width );
            stub->SetLayer( layer );
            stub->SetNet( ball->GetNet() );

            const nlohmann::json violations = guard( board, { via.get(), stub.get() }, {}, zoneMode == "refill", &staged );

            if( !violations.empty() )
            {
                last = "the spots near it touch other copper (" + violations[0]["other"].dump() + ")";
                continue;
            }

            spots.insert( { spot.x, spot.y } );
            placed.push_back( { { "ball", str( fp->GetReference() ) + "." + str( ball->GetNumber() ) },
                                { "net", net },
                                { "ring", ring },
                                { "via_mm", { toMm( spot.x ), toMm( spot.y ) } },
                                { "outward", d.x == sx && d.y == sy } } );

            staged.push_back( via.get() );
            staged.push_back( stub.get() );

            if( !dryRun )
            {
                added.push_back( via->m_Uuid );
                added.push_back( stub->m_Uuid );
                commit.Add( via.release() );
                commit.Add( stub.release() );
            }
            else
            {
                planned.push_back( std::move( via ) );
                planned.push_back( std::move( stub ) );
            }

            done = true;
            break;
        }

        if( !done )
            skip( ball, ring, last.substr( 0, 400 ) );
    }

    if( !dryRun && !added.empty() )
    {
        commit.Push( _( "Fan out (API)" ) );

        if( zoneMode == "refill" )
            refillPours( *context, board );

        board->BuildConnectivity();
        board->GetConnectivity()->RecalculateRatsnest();
        showRoutes( aCtx, added );
    }
    else
    {
        commit.Revert();
    }

    nlohmann::json reasons = nlohmann::json::object();

    for( const auto& [w, n] : why )
        reasons[w] = n;

    int outward = 0;

    for( const nlohmann::json& p : placed )
        outward += p["outward"].get<bool>() ? 1 : 0;

    return KOPENAPI_RESULT::Ok( { { "ref", str( fp->GetReference() ) },
                                  { "pitch_mm", toMm( pitch ) },
                                  { "via", { { "diameter_mm", toMm( viaDia ) }, { "drill_mm", toMm( viaDrill ) } } },
                                  { "outer_rings_direct", direct },
                                  { "fanned", placed.size() },
                                  { "outward", outward },
                                  { "skipped", skipped.size() },
                                  { "skipped_by_reason", reasons },
                                  { "applied", !dryRun && !added.empty() },
                                  { "balls", placed },
                                  { "skipped_balls", skipped } } );
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
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills give way while routing and are refilled around the route; keep: the route must clear the fills"},
                        "via_diameter_mm":{"type":"number","description":"via size of this route (default the net class's)"},"via_drill_mm":{"type":"number"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_route_connection, 120 );

KOPENAPI_REGISTER( "pcb_route",
                   "EXPERIMENTAL. Route the board's airwires (all, or of given nets) with KiCad's "
                   "push-and-shove router, one connection after another (order short_first, power_first or "
                   "dense_first); a failed connection is tried on the next layer, then in shove mode, then "
                   "out of its package through vias (fan-out spots past the pads first) across every other "
                   "given layer; then retry rounds: first from both ends, then rip-up (the routes of this "
                   "call in its way are taken out, it is routed, they are routed again - or all goes back); "
                   "nets named as a differential pair (+/-, P/N suffix) between the same two footprints are "
                   "routed coupled by KiCad's diff-pair router (width / gap from the rules, else track width "
                   "and clearance), one by one if that fails (row diff_pair tells); "
                   "every route passes the net-merge guard; copper of this call that ends nowhere is removed "
                   "(stubs_removed); a row still routed at the end is joined by copper; per connection "
                   "routed / reason, totals (routed, length, vias, unrouted left); dry_run routes nothing",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"string"}},
                        "layers":{"type":"array","items":{"type":"string"},"default":["F.Cu","B.Cu"]},
                        "order":{"type":"string","enum":["short_first","power_first","dense_first"],"default":"short_first","description":"dense_first: connections at fine-pitch pads (<= 0.65 mm) first"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills out while routing (they would block every crossing), refilled after; keep: routes must clear the fills"},
                        "rip_up":{"type":"integer","default":2,"minimum":0,"maximum":6,"description":"rounds of rip-up and retry for failed connections (routes of this call in their way taken out and routed again; all back if that fails)"},
                        "vias":{"type":"boolean","default":true,"description":"connections blocked on their side: escape through vias near both ends"},
                        "diff_pairs":{"type":"string","enum":["auto","off"],"default":"auto","description":"auto: pairs (USB_D+ / USB_D-, LVDS_P / LVDS_N) routed coupled; off: every net alone"},
                        "plane_layers":{"type":"boolean","default":false,"description":"route on plane layers too (type power in the stack-up, or an inner layer more than half of which one net's pours cover): skipped by default, a track would cut the plane; the answer lists plane_layers_skipped"},
                        "under_parts":{"type":"string","enum":["allow","avoid"],"default":"allow","description":"avoid: no track or via of this call inside the ring of pads of a quad package (QFP / QFN / LGA, pads on four sides) on its side - tracks there can wall its other pins in (blocked_by names them); parts with pads inside the ring (exposed pad, BGA) stay open; the answer lists kept_out_under. Costs routes on a crowded board (TC-13: 77 -> 70 of 81)"},
                        "neck_down":{"type":"string","enum":["auto","off"],"default":"auto","description":"auto: a connection with an end on a fine-pitch pad (0.65 mm or closer) is routed at the Default class width (not below the board minimum), the rest of its net at its class width; the row says necked_from_mm"},
                        "max_tries":{"type":"integer","default":120,"minimum":1,"description":"router attempts per connection and pass (layers x modes x escape spots); a big board pays about a second per attempt"},
                        "style":{"type":"string","enum":["direct","rail"],"default":"direct","description":"rail (power nets): every connection leaves its pads through a via next to them and runs on the second layer"},
                        "step_ms":{"type":"integer","default":0,"description":"GUI: draw each route and wait this long before the next (for watching / recording)"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_route, 600 );


KOPENAPI_REGISTER( "pcb_stitch",
                   "Stitch a net (default GND) to its pours and planes: every open end on one side only (SMD "
                   "pad, zone island) gets a through via at a free spot next to it (with a short track from a "
                   "pad), guarded against touching other nets (other nets' fills give way: they are refilled), "
                   "inside the board; after the refill a via the fill does not reach is taken out and its pad "
                   "tried at another spot (up to 3 passes); unlocked vias of the net left touching one layer "
                   "only are removed. On inner planes stitch before routing the signals (the spots next to the "
                   "pads are still free), with outer-layer pours both before and after; answers the vias, "
                   "dangling_vias_removed, not_reached (pads / islands left without a via, and why) and the "
                   "connections still open",
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
                        "all":{"type":"boolean","description":"every track and via (unlocked) - with layer / kinds to narrow"},
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
                        "via_diameter_mm":{"type":"number","description":"via size of this call (default the net class's)"},"via_drill_mm":{"type":"number"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills of other nets give way (the guard does not count them) and are refilled around the track; keep: the track must clear the fills"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_track_add );

KOPENAPI_REGISTER( "pcb_via_add",
                   "Add a via: alone on a net at a point, or with a short track from a pad (from: REF.PAD); "
                   "size from the net class unless diameter_mm / drill_mm; the router's guard applies (422 with "
                   "the violations); pour fills of other nets give way and are refilled (zones: keep to clear "
                   "them); one undo step; dry_run checks only",
                   R"json({"type":"object","required":["at_mm"],"properties":{
                        "at_mm":{"type":"array","items":{"type":"number"},"description":"[x_mm, y_mm]"},
                        "net":{"type":"string","description":"needed without 'from'"},
                        "from":{"type":"string","description":"REF.PAD: a track from its centre to the via"},
                        "layer":{"type":"string","description":"layer of that track (default F.Cu, B.Cu for a bottom pad)"},
                        "width_mm":{"type":"number"},"diameter_mm":{"type":"number"},"drill_mm":{"type":"number"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_via_add );

KOPENAPI_REGISTER( "pcb_fanout",
                   "Fan an area-array package (BGA / LGA) out before routing, as the reference designs do: per "
                   "ball a short track to a through via on the diagonal between four balls, pointing away from "
                   "the package centre (quadrant dog-bone; another diagonal when that spot is taken). Balls of "
                   "nets with no other pad are left alone, so are the outer rings that leave on their layer "
                   "(outer_rings_direct: default 1 at 0.75 mm pitch and coarser, 0 finer). Via size: given, or "
                   "the one most used under the package when the rules allow it, or from the pitch (0.5 mm "
                   "0.3 / 0.15, 0.65 mm 0.35 / 0.2, 0.8 mm 0.45 / 0.2, 1.0 mm 0.5 / 0.25); stub width the net class's (width_mm to set); the guard and the board's minimums "
                   "apply; pours give way and are refilled; one undo step; dry_run plans only. Answers per ball "
                   "(via spot, ring, outward) and the skipped ones with the reason; then pcb_route from the vias",
                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string"},
                        "nets":{"type":"array","items":{"type":"string"},"description":"only these nets"},
                        "outer_rings_direct":{"type":"integer","minimum":0},
                        "via_diameter_mm":{"type":"number"},"via_drill_mm":{"type":"number"},
                        "width_mm":{"type":"number"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_fanout, 300 );

KOPENAPI_MARK_EDITING( "pcb_route_connection", "pcb_route", "pcb_stitch", "pcb_tracks_delete", "pcb_track_add", "pcb_via_add", "pcb_fanout" );
