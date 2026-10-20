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
#include "route_grid.h"
#include "route_timeline.h"

#include <kicadopenapi_history.h>
#include "route_plan.h"

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_memory.h>
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
#include <pcb_shape.h>
#include <pcb_track.h>
#include <router/pns_kicad_iface.h>
#include <regex>
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
#include <tuple>
#include <thread>
#include <cmath>
#include <optional>
#include <set>

#include <kicadopenapi_jobs.h>


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

        if( aItem->Type() == PCB_VIA_T )
        {
            d["at_mm"] = { toMm( aItem->GetPosition().x ), toMm( aItem->GetPosition().y ) };
        }
        else if( PCB_TRACK* track = dynamic_cast<PCB_TRACK*>( aItem ) )
        {
            d["from_mm"] = { toMm( track->GetStart().x ), toMm( track->GetStart().y ) };
            d["to_mm"] = { toMm( track->GetEnd().x ), toMm( track->GetEnd().y ) };
        }

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


/// @brief The way out of a fine-pitch pad on the edge of its package whose connection is still to be
/// routed: room for a track and a via in front of the pad; copper of another net across it walls
/// the pad in (a short connection around the pad row is the usual culprit)
struct ESCAPE_FRONT
{
    PAD*                pad = nullptr;
    PCB_LAYER_ID        layer = F_Cu;
    SHAPE_SEGMENT       shape;
    std::vector<size_t> jobs;          ///< the connections at the pad
    bool                open = true;   ///< false once they are all routed
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

    /// @brief The points come from the global plan (PlanRoute)
    bool planned = false;

    /// @brief Ways out of pads kept open: a route putting copper of another net across one is refused
    const std::vector<ESCAPE_FRONT>* fronts = nullptr;
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


/// @brief Where a pcb_route call's time goes, over every attempt (the failed ones too)
struct ROUTE_PROFILE
{
    int    attempts = 0;
    int    deadLegsSkipped = 0;   ///< tries not made: their first leg had failed to reach that via
    int    viaSpotsBlocked = 0;   ///< via spots left out: a via there would touch another net
    int    ripUpsSkipped = 0;     ///< rip-ups not repeated: nothing changed since the same one failed
    int    ripUpsWalled = 0;      ///< rip-ups not made: no plan even through the routes of this call
    int    ripUpsPlanned = 0;     ///< rip-ups whose routes to take out a plan chose (the rest: the nearest in the corridor)
    std::map<std::string, int> ripUpsFailedAt;   ///< failed rip-ups by what failed: the route, a victim, a cut one
    int    ripUpsSecond = 0;      ///< victims with no way back that took out routes in turn
    int    pairsStuck = 0;        ///< pair tries ended early: nothing placed after the first steps
    int    plans = 0;             ///< global plans made (PlanRoute)
    int    plansFound = 0;        ///< ... that found a way
    double planMs = 0;
    double syncMs = 0;    ///< the router's world built from the board
    double moveMs = 0;    ///< the router's path search (every Move of the head)
    double totalMs = 0;   ///< whole attempts (world, routing, guard, commit)

    struct TALLY
    {
        int    tries = 0;
        int    routed = 0;
        double ms = 0;
    };

    std::map<std::string, TALLY> byKind;     ///< walkaround / shove / one_via / two_vias
    std::map<std::string, TALLY> byFailure;  ///< failed attempts by reason (places cut off)
    std::map<std::string, TALLY> byPass;     ///< first pass, retry from both ends, rip-up
    std::string                  pass = "first";
    std::set<int>                shovedNets;   ///< nets of other copper a committed route moved
};

static ROUTE_PROFILE s_routeProfile;


static nlohmann::json tallies( const std::map<std::string, ROUTE_PROFILE::TALLY>& aTallies )
{
    nlohmann::json out = nlohmann::json::object();

    for( const auto& [name, t] : aTallies )
        out[name] = { { "tries", t.tries }, { "routed", t.routed }, { "ms", std::round( t.ms ) } };

    return out;
}


static double msSince( std::chrono::steady_clock::time_point aFrom,
                       std::chrono::steady_clock::time_point aTo = std::chrono::steady_clock::now() )
{
    return std::chrono::duration<double, std::milli>( aTo - aFrom ).count();
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
    // lifted for the call, its iteration limits bound the work (a shove through a dense row can take
    // 50 ms an iteration: the editor's 250 would run past any clock)
    settings.SetShoveTimeLimit( 60000 );
    settings.SetShoveIterationLimit( 100 );

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

    std::vector<std::unique_ptr<PCB_SHAPE>> frontItems;   // the ways out kept open, as the router sees them

    auto router = std::make_unique<PNS::ROUTER>();
    router->SetInterface( iface.get() );
    router->LoadSettings( &settings );
    router->SyncWorld();

    // a way out of another net's pad kept open is that net's copper to the router: it walks around
    if( aReq.fronts && !aReq.diffPair )
    {
        for( const ESCAPE_FRONT& front : *aReq.fronts )
        {
            if( !front.open || front.pad->GetNetCode() == aReq.from->GetNetCode() )
                continue;

            auto item = std::make_unique<PCB_SHAPE>( board, SHAPE_T::SEGMENT );
            item->SetStart( front.shape.GetSeg().A );
            item->SetEnd( front.shape.GetSeg().B );
            item->SetWidth( front.shape.GetWidth() );
            item->SetLayer( front.layer );
            item->SetNet( front.pad->GetNet() );

            auto solid = std::make_unique<PNS::SOLID>();
            solid->SetLayer( iface->GetPNSLayerFromBoardLayer( front.layer ) );
            solid->SetNet( front.pad->GetNet() );
            solid->SetParent( item.get() );
            solid->SetShape( front.shape.Clone() );
            solid->SetPos( front.shape.GetSeg().Center() );
            solid->SetRoutable( false );
            router->GetWorld()->Add( std::move( solid ) );
            frontItems.push_back( std::move( item ) );
        }
    }

    const auto synced = std::chrono::steady_clock::now();
    s_routeProfile.syncMs += msSince( started, synced );
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

    auto move = [&]( const VECTOR2I& aAt, PNS::ITEM* aItem )
    {
        const auto from = std::chrono::steady_clock::now();
        router->Move( aAt, aItem );
        s_routeProfile.moveMs += msSince( from );
    };

    // Vias on the way: route to the point, drop a through via there, continue on the next layer
    PNS::PLACEMENT_ALGO* placer = router->Placer();

    for( const auto& [at, viaLayer] : aReq.vias )
    {
        if( !viaLayer )
        {
            // a corner: route to the point and fix what is there, on the same layer
            move( at, nullptr );
            move( at, nullptr );

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

        move( at, nullptr );
        move( at, nullptr );

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

        // a pair that has not left its start pads after a few steps does not fit there: each further
        // step only searches the same gateways again (seconds each on a dense board)
        constexpr int STUCK_STEPS = 3;
        int           empty = 0;

        for( size_t leg = 1; leg < path.size() && empty < STUCK_STEPS; ++leg )
        {
            const VECTOR2I from = path[leg - 1];
            const VECTOR2I to = path[leg];
            const int      steps = std::max( 1, (int) ( ( to - from ).EuclideanNorm() / pcbIUScale.mmToIU( 1.0 ) ) );

            for( int k = 1; k <= steps && empty < STUCK_STEPS; ++k )
            {
                if( leg + 1 < path.size() || k < steps )
                {
                    move( from + ( to - from ) * k / steps, nullptr );
                    empty = placer->HasPlacedAnything() ? 0 : empty + 1;
                }
            }
        }

        if( empty >= STUCK_STEPS )
        {
            router->StopRouting();
            iface->Staged().Revert();
            s_routeProfile.pairsStuck++;
            aOut["routed"] = false;
            aOut["reason"] = "the pair did not leave its start pads";
            return false;
        }
    }

    // Move towards the target until the head stops changing (as the editor's Attempt Finish)
    VECTOR2I             previous;
    int                  tries = 8;

    do
    {
        previous = placer->CurrentEnd();
        move( target, targetItem );
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
        const bool     wrongLayer = !fixed && !aReq.diffPair && end == target && !targetLayers.Overlaps( router->GetCurrentLayer() );

        if( router->RoutingInProgress() )
            router->StopRouting();

        iface->Staged().Revert();
        aOut["routed"] = false;
        aOut["reason"] = aReq.diffPair ? "the pair did not reach the target pads"
                         : reached     ? "the router could not fix the route"
                         : wrongLayer  ? "at the target but on another layer than its pad: a via is needed in front of it"
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

    if( aReq.fronts && !aReq.diffPair )
    {
        for( BOARD_CONNECTED_ITEM* item : changed )
        {
            for( const ESCAPE_FRONT& front : *aReq.fronts )
            {
                if( !front.open || front.pad->GetNetCode() == item->GetNetCode() || !item->IsOnLayer( front.layer )
                    || !item->GetEffectiveShape( front.layer )->Collide( &front.shape, 0 ) )
                {
                    continue;
                }

                iface->Staged().Revert();
                aOut["routed"] = false;
                aOut["reason"] = "refused: the route would wall in the way out of "
                                 + str( front.pad->GetParentFootprint()->GetReference() ) + "." + str( front.pad->GetNumber() );
                return false;
            }
        }
    }
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

    const NETINFO_ITEM* coupled = aReq.diffPair ? board->DpCoupledNet( aReq.from->GetNet() ) : nullptr;

    auto otherNet = [&]( const BOARD_ITEM* aItem ) -> int
    {
        const BOARD_CONNECTED_ITEM* connected = dynamic_cast<const BOARD_CONNECTED_ITEM*>( aItem );

        if( !connected || connected->GetNetCode() <= 0 || connected->GetNetCode() == aReq.from->GetNetCode()
            || ( coupled && connected->GetNetCode() == coupled->GetNetCode() ) )
        {
            return 0;
        }

        return connected->GetNetCode();
    };

    for( BOARD_ITEM* item : iface->m_added )
        aAdded.push_back( item->m_Uuid );

    for( BOARD_ITEM* item : iface->m_added )
    {
        if( int net = otherNet( item ) )
            s_routeProfile.shovedNets.insert( net );
    }

    for( const auto* items : { &iface->m_modified, &iface->m_removed } )
    {
        for( BOARD_ITEM* item : *items )
        {
            if( int net = otherNet( item ) )
                s_routeProfile.shovedNets.insert( net );
        }
    }

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
    const auto started = std::chrono::steady_clock::now();
    const bool ok = routeOneImpl( aContext, aReq, aOut, aAdded );

    const double ms = msSince( started );
    s_routeProfile.attempts++;
    s_routeProfile.totalMs += ms;

    const size_t      viaCount = std::count_if( aReq.vias.begin(), aReq.vias.end(), []( const auto& v ) { return v.second.has_value(); } );
    const std::string kind = aReq.planned ? "planned"
                             : viaCount >= 2 ? "two_vias"
                             : viaCount == 1 ? "one_via"
                             : aReq.mode == PNS::RM_Shove ? "shove" : "walkaround";
    for( ROUTE_PROFILE::TALLY* t : { &s_routeProfile.byKind[kind], &s_routeProfile.byPass[s_routeProfile.pass] } )
    {
        t->tries++;
        t->routed += ok;
        t->ms += ms;
    }

    if( !ok )
    {
        std::string reason = aOut.value( "reason", std::string( "-" ) );
        size_t      at = reason.find( " at " );

        if( at != std::string::npos )
            reason.resize( at );

        ROUTE_PROFILE::TALLY& f = s_routeProfile.byFailure[reason];
        f.tries++;
        f.ms += ms;
    }

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

    // the start pad's layer unless given (an SMD pad on the back starts on B.Cu)
    const std::string defaultLayer = ( *from )->IsOnLayer( F_Cu ) || !( *from )->IsOnLayer( B_Cu ) ? "F.Cu" : "B.Cu";
    const std::string           layerName = aArgs.value( "layer", defaultLayer );
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

    // a target that is not on the layer the route arrives on (an SMD pad on the other side): a via
    // in front of it, on the line from the last point, the route ends on the pad's layer
    std::optional<VECTOR2I> autoVia;

    if( req.to )
    {
        PCB_LAYER_ID arrive = req.layer;

        for( const auto& [point, viaLayer] : req.vias )
        {
            if( viaLayer )
                arrive = *viaLayer;
        }

        if( !req.to->IsOnLayer( arrive ) )
        {
            std::optional<PCB_LAYER_ID> padLayer;

            for( PCB_LAYER_ID l : board->GetEnabledLayers().CuStack() )
            {
                if( req.to->IsOnLayer( l ) )
                {
                    padLayer = l;
                    break;
                }
            }

            if( padLayer )
            {
                const VECTOR2I last = req.vias.empty() ? req.from->GetPosition() : req.vias.back().first;
                const VECTOR2I dir = last - req.toPos;
                const VECTOR2I size = static_cast<PAD*>( req.to )->GetSize( *padLayer );
                const int      reach = std::max( size.x, size.y ) / 2 + board->GetDesignSettings().GetCurrentViaSize() / 2
                                  + board->GetDesignSettings().m_MinClearance + pcbIUScale.mmToIU( 0.1 );
                const double   len = std::max( 1.0, double( dir.EuclideanNorm() ) );
                const VECTOR2I at = req.toPos + VECTOR2I( KiROUND( dir.x * reach / len ), KiROUND( dir.y * reach / len ) );

                req.vias.emplace_back( at, *padLayer );
                autoVia = at;
            }
        }
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

    if( autoVia )
    {
        result["via_near_target_mm"] = { toMm( autoVia->x ), toMm( autoVia->y ) };

        if( !result.value( "routed", false ) )
            result["reason"] = result.value( "reason", std::string() ) + " ('to' is on the other side: a via was put in front of it;"
                                                                        " a waypoint {at, layer} chooses where)";
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


/// @brief pcb_route {from}: a recorded lane goes on after one of its steps
struct ROUTE_BRANCH
{
    std::shared_ptr<GRID_TIMELINE> timeline;
    int                            lane = -1;
    size_t                         step = 0;
    std::string                    board;   ///< how the board was brought back to the run's start
};


static KOPENAPI_RESULT pcbRoute( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, const ROUTE_BRANCH* aBranch );


static KOPENAPI_RESULT h_pcb_route( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    if( !aArgs.contains( "from" ) )
        return pcbRoute( aCtx, aArgs, nullptr );

    const nlohmann::json& from = aArgs["from"];

    if( !from.is_object() || !from.contains( "step" ) || !from["step"].is_number_integer() )
        return KOPENAPI_RESULT::Error( 400, "from: {run, lane (index or variant), step}" );

    std::shared_ptr<GRID_TIMELINE> t = GridTimelineLast();

    if( !t )
        return KOPENAPI_RESULT::Error( 404, "no recorded route (pcb_route {engine: 'grid', record: true})" );

    if( from.value( "run", t->run ) != t->run )
        return KOPENAPI_RESULT::Error( 404, "only the last recorded run is kept: run " + std::to_string( t->run ) );

    if( t->live )
        return KOPENAPI_RESULT::Error( 409, "run " + std::to_string( t->run ) + " is still routing" );

    ROUTE_BRANCH          branch{ t, -1, 0 };
    const nlohmann::json& lane = from.value( "lane", nlohmann::json( t->appliedLane >= 0 ? t->appliedLane : t->winner ) );

    for( size_t k = 0; k < t->LaneCount(); ++k )
    {
        if( ( lane.is_number_integer() && lane.get<int>() == (int) k ) || ( lane.is_string() && lane.get<std::string>() == t->Lane( k ).name ) )
            branch.lane = (int) k;
    }

    if( branch.lane < 0 )
        return KOPENAPI_RESULT::Error( 404, "from.lane: no such lane in run " + std::to_string( t->run ) );

    const long long step = from["step"].get<long long>();
    const size_t    steps = t->Lane( branch.lane ).steps.size();

    if( step < 0 || step >= (long long) steps )
        return KOPENAPI_RESULT::Error( 400, "from.step: 0.." + std::to_string( (long long) steps - 1 ) );

    branch.step = (size_t) step;

    nlohmann::json args = nlohmann::json::parse( t->args, nullptr, false );

    if( !args.is_object() )
        return KOPENAPI_RESULT::Error( 500, "the run's arguments are not kept" );

    // the board as the run found it: the copper of the run (or of its last branch) taken back when
    // that is the last edit; else the board must already be so
    KOPENAPI_HISTORY& history = KOPENAPI_HISTORY::Get();

    if( t->appliedSeq > 0 && history.LastSeq() == t->appliedSeq )
    {
        KOPENAPI_RESULT back = history.RollbackLastInCall( aCtx );

        if( back.status != 200 )
            return back;

        t->appliedSeq = 0;
        branch.board = "the route's copper rolled back";
    }
    else
    {
        branch.board = t->appliedSeq > 0 ? "the route is not the last edit (edit_log): the board was left as it is"
                                         : "nothing of the run on the board to roll back";
    }

    return pcbRoute( aCtx, args, &branch );
}


static KOPENAPI_RESULT pcbRoute( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, const ROUTE_BRANCH* aBranch )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD* board = context->GetBoard();
    const auto started = std::chrono::steady_clock::now();
    s_routeProfile = {};

    // each net's copper before the call (pieces, length): what shove moved of other nets is told
    auto copperOf = [&]()
    {
        std::map<int, std::pair<int, double>> copper;

        for( PCB_TRACK* track : board->Tracks() )
        {
            std::pair<int, double>& net = copper[track->GetNetCode()];
            net.first++;
            net.second += track->Type() == PCB_VIA_T ? 0.0 : track->GetLength();
        }

        return copper;
    };

    const std::map<int, std::pair<int, double>> copperBefore = copperOf();

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

    // grid: every net together on one grid of the board, negotiated (PathFinder); pns: one connection
    // at a time with the push-and-shove router
    const std::string engine = aArgs.value( "engine", std::string( "pns" ) );

    if( engine != "pns" && engine != "grid" )
        return KOPENAPI_RESULT::Error( 400, "engine: pns or grid" );

    const bool grid = engine == "grid" && !dryRun;

    // attempts per connection and pass: a count, not a time (the same board routes the same way);
    // on a big board an attempt costs about a second
    const int maxTries = std::clamp( aArgs.value( "max_tries", 120 ), 1, 10000 );

    const std::string diffPairs = aArgs.value( "diff_pairs", std::string( "auto" ) );

    // neck_down: "auto" | "off", or per net {"VBUS": "off", "*": "auto"}
    std::map<wxString, bool> neckPerNet;
    bool                     neckDefault = true;
    const nlohmann::json     neckArg = aArgs.value( "neck_down", nlohmann::json( "auto" ) );

    auto neckMode = [&]( const nlohmann::json& aValue, bool& aOut ) -> bool
    {
        if( !aValue.is_string() || ( aValue != "auto" && aValue != "off" ) )
            return false;

        aOut = aValue == "auto";
        return true;
    };

    if( neckArg.is_object() )
    {
        for( const auto& [net, mode] : neckArg.items() )
        {
            bool on = true;

            if( !neckMode( mode, on ) )
                return KOPENAPI_RESULT::Error( 400, "neck_down: each net (or \"*\" for the rest) is auto or off" );

            if( net == "*" )
                neckDefault = on;
            else
                neckPerNet[wxString::FromUTF8( net )] = on;
        }
    }
    else if( !neckMode( neckArg, neckDefault ) )
    {
        return KOPENAPI_RESULT::Error( 400, "neck_down: auto, off, or per net {\"VBUS\": \"off\", \"*\": \"auto\"}" );
    }

    if( diffPairs != "auto" && diffPairs != "off" )
        return KOPENAPI_RESULT::Error( 400, "diff_pairs: auto (nets named as a pair routed coupled) or off" );
    size_t            glowFrom = 0;

    // Pours are filled around the tracks afterwards: their fills come out for the routing (they
    // would block every crossing) and are refilled at the end
    const std::string zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (fills out while routing, refilled after) or keep" );

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
        double                ms = 0;          ///< time of every try for it, rip-up re-routes too
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

    // the fills are taken out after the connections are read: what a zone joins needs no track
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

    // equal keys by where the ends are: the board's item order follows uuids, random for a board
    // built by pcb_netlist_apply
    auto byPlace = []( const JOB& x, const JOB& y )
    {
        const VECTOR2I xf = x.from->GetPosition(), yf = y.from->GetPosition();

        if( xf != yf )
            return xf.x != yf.x ? xf.x < yf.x : xf.y < yf.y;

        return x.toPos.x != y.toPos.x ? x.toPos.x < y.toPos.x : x.toPos.y < y.toPos.y;
    };

    std::stable_sort( jobs.begin(), jobs.end(),
                      [&]( const JOB& x, const JOB& y )
                      {
                          if( order == "power_first" && x.power != y.power )
                              return x.power;

                          if( x.length != y.length )
                              return x.length < y.length;

                          return byPlace( x, y );
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
    // routed no wider than that pad's narrow side (a track that wide fits wherever the pad does),
    // not below the Default class width or the board's minimum; the rest of the net keeps its width
    const int         neckWidth = std::max( board->GetDesignSettings().m_TrackMinWidth,
                                            board->GetDesignSettings().m_NetSettings->GetDefaultNetclass()->HasTrackWidth()
                                                    ? board->GetDesignSettings().m_NetSettings->GetDefaultNetclass()->GetTrackWidth()
                                                    : 0 );

    auto neckFor = [&]( PAD* aFrom, BOARD_CONNECTED_ITEM* aTo ) -> int
    {
        const double fine = pcbIUScale.mmToIU( 0.65 );
        auto         it = neckPerNet.find( aFrom->GetNetname() );

        if( !( it == neckPerNet.end() ? neckDefault : it->second ) )
            return 0;

        int cap = INT_MAX;

        for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( aFrom ), aTo } )
        {
            if( end && end->Type() == PCB_PAD_T && pitchOf( static_cast<PAD*>( end ) ) <= fine )
            {
                const VECTOR2I size = static_cast<PAD*>( end )->GetSize( PADSTACK::ALL_LAYERS );
                cap = std::min( cap, std::max( neckWidth, std::min( size.x, size.y ) ) );
            }
        }

        return cap == INT_MAX ? 0 : cap;
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

                              if( fx != fy )
                                  return fx;

                              return x.length != y.length ? x.length < y.length : byPlace( x, y );
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

    // a via of the end's net (the smallest the router could use there) would touch copper of
    // another net (pads, tracks, vias; the board's rule for each pair): the router's walkaround cannot
    // place it, a try through it is wasted
    auto viaBlocked = [&]( BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aAt )
    {
        int       dia = ds.GetCurrentViaSize();
        int       drill = ds.GetCurrentViaDrill();
        NETCLASS* netclass = aEnd->GetEffectiveNetClass();

        if( netclass && netclass->HasViaDiameter() )
            dia = std::min( dia, netclass->GetViaDiameter() );

        if( netclass && netclass->HasViaDrill() )
            drill = std::min( drill, netclass->GetViaDrill() );

        PCB_VIA via( board );
        via.SetPosition( aAt );
        via.SetViaType( VIATYPE::THROUGH );
        via.SetLayerPair( F_Cu, B_Cu );
        via.SetWidth( PADSTACK::ALL_LAYERS, dia );
        via.SetDrill( drill );
        via.SetNet( aEnd->GetNet() );

        return !guard( board, { &via }, {}, true ).empty();
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

        auto unusable = [&]( const VECTOR2I& aAt )
        {
            if( holeClash( aAt ) )
                return true;

            if( viaBlocked( aEnd, aAt ) )
            {
                s_routeProfile.viaSpotsBlocked++;
                return true;
            }

            return false;
        };

        outward.erase( std::remove_if( outward.begin(), outward.end(), unusable ), outward.end() );
        around.erase( std::remove_if( around.begin(), around.end(), unusable ), around.end() );
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
    const bool planning = aArgs.value( "plan", true );

    // ways out of fine-pitch pads kept open while their connections wait (filled once the jobs are
    // final, below); open while a connection at the pad is not routed
    std::vector<ESCAPE_FRONT> fronts;
    const std::vector<bool>*  doneOf = nullptr;

    auto refreshFronts = [&]()
    {
        for( ESCAPE_FRONT& front : fronts )
            front.open = std::any_of( front.jobs.begin(), front.jobs.end(), [&]( size_t k ) { return !( *doneOf )[k]; } );
    };

    auto planRequest = [&]( PAD* aFrom, BOARD_CONNECTED_ITEM* aTo, const VECTOR2I& aToPos )
    {
        NETCLASS* netclass = aFrom->GetEffectiveNetClass();
        int       width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : ds.GetCurrentTrackWidth();
        int       dia = ds.GetCurrentViaSize();
        int       drill = ds.GetCurrentViaDrill();

        if( netclass && netclass->HasViaDiameter() )
            dia = std::min( dia, netclass->GetViaDiameter() );

        if( netclass && netclass->HasViaDrill() )
            drill = std::min( drill, netclass->GetViaDrill() );

        if( const int neck = neckFor( aFrom, aTo ); neck > 0 && neck < width )
            width = neck;

        ROUTE_PLAN_REQUEST req{ board, aFrom, aTo, aToPos, layers, width, viasAllowed ? dia : 0, drill, {}, {} };

        for( const ESCAPE_FRONT& front : fronts )
        {
            if( front.open && front.pad->GetNetCode() != aFrom->GetNetCode() )
                req.keepOut.emplace_back( front.layer, front.shape );
        }

        return req;
    };

    auto makePlan = [&]( const ROUTE_PLAN_REQUEST& aReq )
    {
        const ROUTE_PLAN plan = PlanRoute( aReq );

        s_routeProfile.plans++;
        s_routeProfile.plansFound += plan.found;
        s_routeProfile.planMs += plan.ms;
        return plan;
    };

    // Every way to route one connection: on a layer both ends are on (walkaround, then shove),
    // then out through a via near each end and across on another layer.  aShove false keeps every
    // other route as it is (rip-up attempts must be revertible).
    auto tryRoute = [&]( JOB& aJob, nlohmann::json& aAttempt, int& aTries, bool aShove, bool aBothWays,
                         bool aForward = true ) -> bool
    {
        struct TIMED
        {
            JOB&                                  job;
            std::chrono::steady_clock::time_point from = std::chrono::steady_clock::now();
            ~TIMED() { job.ms += msSince( from ); }
        } timed{ aJob };

        bool ok = false;

        if( doneOf )
            refreshFronts();

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

        // aForward false: the forward way is known to fail
        if( !aForward )
            ways.erase( ways.begin() );

        // the global plan: a way over the free space of the layers, the router follows its corners
        int  planned = -1;   // -1 no plan made, 0 none found, 1 found
        auto planTry = [&]()
        {
            if( ok || !planning || rail || ways.empty() || aTries >= maxTries )
                return;

            const WAY&       way = ways.front();
            const ROUTE_PLAN plan = makePlan( planRequest( way.from, way.to, way.toPos ) );

            planned = plan.found ? 1 : 0;

            if( plan.found )
            {
                ROUTE_REQUEST req{ way.from, way.to, way.toPos, plan.startLayer, PNS::RM_Walkaround, dryRun, {} };
                req.fronts = &fronts;
                req.neckWidth = neckFor( way.from, way.to );
                req.vias = plan.points;
                req.planned = true;
                aAttempt = nlohmann::json::object();
                aTries++;
                ok = routeOne( *context, req, aAttempt, added );
            }
        };

        // on a layer both ends are on: walkaround, then the plan, then shove (slow when it fails: last)
        auto onLayers = [&]( PNS::PNS_MODE aMode )
        {
            for( const WAY& way : ways )
            {
                for( PCB_LAYER_ID layer : layers )
                {
                    if( ok || aTries >= maxTries || rail || !way.from->IsOnLayer( layer ) || !way.to->IsOnLayer( layer ) )
                        continue;

                    ROUTE_REQUEST req{ way.from, way.to, way.toPos, layer, aMode, dryRun, {} };
                    req.fronts = &fronts;
                    req.neckWidth = neckFor( way.from, way.to );
                    aAttempt = nlohmann::json::object();
                    aTries++;
                    ok = routeOne( *context, req, aAttempt, added );
                }
            }
        };

        onLayers( PNS::RM_Walkaround );

        if( !ok && aAttempt.is_null() )
        {
            aAttempt = { { "routed", false },
                         { "reason", std::string( "no layer of this call has both ends" )
                                             + ( viasAllowed ? "" : " (vias: false; a via is needed)" ) } };
        }

        if( !ok )
            planTry();

        if( !ok && aShove )
        {
            const nlohmann::json before = aAttempt;   // what the plan's try (or the walkaround) found
            onLayers( PNS::RM_Shove );

            if( !ok && !before.is_null() )
                aAttempt = before;
        }

        if( ok || !viasAllowed )
            return ok;

        // the plan searched every way over the free space around the connection and found none:
        // the escape spots below find none either (they only try fewer ways)
        if( planned == 0 )
        {
            aAttempt["plan"] = "no way over the free space around the connection";
            return false;
        }

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

            // a route that stopped short of its first via: that leg (from the pad, on the start
            // layer) fails whatever comes after the via, so no other try through that spot is made;
            // one that passed every via and stopped short of the target: the last leg (from the last
            // via, on the arriving layer) fails whatever came before it
            std::set<std::pair<int, int>> deadFirstVia;
            std::set<std::pair<int, int>> deadLastVia;

            auto stoppedBefore = [&]( const VECTOR2I& aVia )
            {
                if( !aAttempt.contains( "toward_mm" ) || aAttempt.value( "routed", false ) )
                    return false;

                const nlohmann::json& t = aAttempt["toward_mm"];
                return std::abs( pcbIUScale.mmToIU( t[0].get<double>() ) - aVia.x ) <= 1
                       && std::abs( pcbIUScale.mmToIU( t[1].get<double>() ) - aVia.y ) <= 1;
            };

            auto stoppedAfter = [&]()
            {
                return !aAttempt.value( "routed", false ) && aAttempt.contains( "target_mm" )
                       && aAttempt.contains( "stopped_at_mm" ) && !aAttempt.contains( "toward_mm" );
            };

            // ends on different layers: one via, near the far end first (the start leaves on its
            // own layer), then near the start
            if( arrive != start )
            {
                for( const std::vector<VECTOR2I>* spotList : { &ends, &starts } )
                {
                    int budget = 9;

                    for( const VECTOR2I& v : *spotList )
                    {
                        if( ok || aTries >= maxTries || budget-- <= 0 )
                            break;

                        if( deadFirstVia.count( { v.x, v.y } ) || deadLastVia.count( { v.x, v.y } ) )
                        {
                            s_routeProfile.deadLegsSkipped++;
                            continue;
                        }

                        ROUTE_REQUEST req{ way.from, way.to, way.toPos, start, PNS::RM_Walkaround, dryRun, {} };
                        req.fronts = &fronts;
                        req.neckWidth = neckFor( way.from, way.to );
                        req.vias = { { v, arrive } };
                        aAttempt = nlohmann::json::object();
                        aTries++;
                        ok = routeOne( *context, req, aAttempt, added );

                        if( !ok && stoppedBefore( v ) )
                            deadFirstVia.insert( { v.x, v.y } );
                        else if( !ok && stoppedAfter() )
                            deadLastVia.insert( { v.x, v.y } );
                    }
                }

                if( ok )
                    break;
            }

            for( PCB_LAYER_ID cross : layers )
            {
                if( cross == start || ( cross == arrive && arrive != start ) )
                    continue;

                int budget = 24;   // via pairs per crossing layer

                for( const VECTOR2I& va : starts )
                {
                    for( const VECTOR2I& vb : ends )
                    {
                        if( deadFirstVia.count( { va.x, va.y } ) )
                        {
                            s_routeProfile.deadLegsSkipped++;
                            break;
                        }

                        if( ok || aTries >= maxTries || budget <= 0 )
                            break;

                        if( deadLastVia.count( { vb.x, vb.y } ) )
                        {
                            s_routeProfile.deadLegsSkipped++;
                            continue;
                        }

                        budget--;

                        ROUTE_REQUEST req{ way.from, way.to, way.toPos, start, PNS::RM_Walkaround, dryRun, {} };
                        req.fronts = &fronts;
                        req.neckWidth = neckFor( way.from, way.to );
                        req.vias = { { va, cross }, { vb, arrive } };
                        aAttempt = nlohmann::json::object();
                        aTries++;
                        ok = routeOne( *context, req, aAttempt, added );

                        if( !ok && stoppedBefore( va ) )
                            deadFirstVia.insert( { va.x, va.y } );
                        else if( !ok && stoppedAfter() )
                            deadLastVia.insert( { vb.x, vb.y } );
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

    // Ways out of the fine-pitch pads on the edges of their packages (pads inside the ring of an area
    // array are left to the fan-out): from the pad's end outward, room for a track or a via
    if( aArgs.value( "escapes", true ) && !rail )
    {
        const double         fine = pcbIUScale.mmToIU( 0.65 );
        const int            depth = 2 * std::max( ds.m_MinClearance, 0 ) + ds.GetCurrentViaSize();
        std::map<PAD*, std::vector<size_t>> frontsOf;

        for( size_t k = 0; k < jobs.size(); ++k )
        {
            if( partner[k] >= 0 )
                continue;

            for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( jobs[k].from ), jobs[k].to } )
            {
                PAD* pad = dynamic_cast<PAD*>( end );

                if( !pad || !pad->GetParentFootprint() )
                    continue;

                if( auto it = frontsOf.find( pad ); it != frontsOf.end() )
                {
                    for( size_t f : it->second )
                        fronts[f].jobs.push_back( k );

                    continue;
                }

                frontsOf[pad];
                const double pitch = pitchOf( pad );

                if( pitch > fine )
                    continue;

                FOOTPRINT* fp = pad->GetParentFootprint();
                BOX2I      ring = fp->Pads().front()->GetBoundingBox();

                for( PAD* q : fp->Pads() )
                    ring.Merge( q->GetBoundingBox() );

                const BOX2I    box = pad->GetBoundingBox();
                const VECTOR2I d = pad->GetPosition() - ring.Centre();
                const bool     horizontal = (double) std::abs( d.x ) * ring.GetHeight() >= (double) std::abs( d.y ) * ring.GetWidth();
                const VECTOR2I dir = horizontal ? VECTOR2I( d.x >= 0 ? 1 : -1, 0 ) : VECTOR2I( 0, d.y >= 0 ? 1 : -1 );
                const int      half = horizontal ? box.GetWidth() / 2 : box.GetHeight() / 2;
                const int      across = horizontal ? box.GetHeight() : box.GetWidth();
                const int      toEdge = horizontal ? ( dir.x > 0 ? ring.GetRight() - box.GetRight() : box.GetLeft() - ring.GetLeft() )
                                                   : ( dir.y > 0 ? ring.GetBottom() - box.GetBottom() : box.GetTop() - ring.GetTop() );

                if( toEdge > pitch / 2 || depth <= across )
                    continue;

                const VECTOR2I tip = box.Centre() + dir * half;

                for( PCB_LAYER_ID layer : layers )
                {
                    if( !pad->IsOnLayer( layer ) || !pad->FlashLayer( layer ) )
                        continue;

                    ESCAPE_FRONT front;
                    front.pad = pad;
                    front.layer = layer;
                    front.shape = SHAPE_SEGMENT( tip + dir * ( across / 2 ), tip + dir * ( depth - across / 2 ), across );
                    front.jobs.push_back( k );
                    frontsOf[pad].push_back( fronts.size() );
                    fronts.push_back( std::move( front ) );
                }
            }
        }

        doneOf = &done;
    }

    // a job's progress: connections routed so far (rip-up and gaps add to them)
    auto progressCounts = [&]()
    {
        return nlohmann::json{ { "routed", std::count( done.begin(), done.end(), true ) } };
    };

    for( size_t index = 0; index < jobs.size(); ++index )
    {
        JOB& job = jobs[index];

        if( grid && partner[index] < 0 )
            continue;   // on the grid below

        if( !KopenapiJobStep( "route", (int) index, (int) jobs.size(), job.net, progressCounts() ) )
            break;

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

            // the guiding single route is the same whatever the pair's mode: found once per end and layer
            std::map<std::tuple<PAD*, PAD*, int>, std::vector<VECTOR2I>> guides;

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

                        const auto key = std::make_tuple( a, b, (int) layer );
                        auto       guide = guides.find( key );

                        if( guide == guides.end() )
                        {
                            ROUTE_REQUEST     probe{ a, b, b->GetPosition(), layer, PNS::RM_Walkaround, true, {} };
                            nlohmann::json    probeOut;
                            std::vector<KIID> none;
                            std::vector<VECTOR2I> path;

                            if( routeOne( *context, probe, probeOut, none ) )
                                path = segmentPath( probeOut.value( "segments", nlohmann::json::array() ), a->GetPosition() );

                            guide = guides.emplace( key, path ).first;
                        }

                        req.guide = guide->second;

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

            for( const char* key : { "violations", "stopped_at_mm", "blocked_by", "plan" } )
            {
                if( attempt.contains( key ) )
                    row[key] = attempt[key];
            }
        }

        rowOf[index] = (int) results.size();
        results.push_back( row );
    }

    // The grid engine: the connections left, every net at once
    nlohmann::json gridOut;

    if( grid )
    {
        board->BuildConnectivity();
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
        std::vector<GRID_NET>              gridNets;
        std::map<int, size_t>              gridOf;      // net code -> index in gridNets
        std::vector<std::vector<size_t>>   jobsOfNet;
        std::vector<std::pair<int, int>>   endsOfJob( jobs.size(), { -1, -1 } );

        auto endIndex = [&]( GRID_NET& aNet, BOARD_CONNECTED_ITEM* aItem )
        {
            auto it = std::find( aNet.ends.begin(), aNet.ends.end(), aItem );

            if( it != aNet.ends.end() )
                return (int) ( it - aNet.ends.begin() );

            aNet.ends.push_back( aItem );
            return (int) aNet.ends.size() - 1;
        };

        for( size_t index = 0; index < jobs.size(); ++index )
        {
            JOB& job = jobs[index];

            if( rowOf[index] >= 0 || job.trunk )
                continue;

            job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
            job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

            if( !job.from || !job.to )
            {
                rowOf[index] = (int) results.size();
                results.push_back( { { "net", job.net }, { "routed", false }, { "tries", 0 },
                                     { "reason", "an end of the connection is gone (removed or replaced by an earlier route)" } } );
                continue;
            }

            const auto joined = conn->GetConnectedItems( job.from );

            if( std::find( joined.begin(), joined.end(), job.to ) != joined.end() )
            {
                rowOf[index] = (int) results.size();
                done[index] = true;
                results.push_back( { { "net", job.net }, { "routed", true }, { "joined_by_earlier_route", true },
                                     { "length_mm", 0.0 }, { "vias", 0 }, { "tries", 0 } } );
                continue;
            }

            const int code = job.from->GetNetCode();
            auto [it, fresh] = gridOf.try_emplace( code, gridNets.size() );

            if( fresh )
            {
                NETCLASS* nc = job.from->GetEffectiveNetClass();
                GRID_NET  n;
                n.netCode = code;
                n.width = nc && nc->HasTrackWidth() ? nc->GetTrackWidth() : ds.GetCurrentTrackWidth();
                n.clearance = netClearance( job.from, ds.m_MinClearance );
                n.viaDiameter = nc && nc->HasViaDiameter() ? nc->GetViaDiameter() : ds.GetCurrentViaSize();
                n.viaDrill = nc && nc->HasViaDrill() ? nc->GetViaDrill() : ds.GetCurrentViaDrill();
                gridNets.push_back( n );
                jobsOfNet.emplace_back();
            }

            GRID_NET& n = gridNets[it->second];

            if( const int neck = neckFor( job.from, job.to ); neck > 0 )
                n.width = std::min( n.width, neck );

            endsOfJob[index] = { endIndex( n, job.from ), endIndex( n, job.to ) };
            jobsOfNet[it->second].push_back( index );
        }

        // bundles: from pcb_net_roles (auto), given, or none; members by their order, nets of this call only
        nlohmann::json bundlesUsed = nlohmann::json::array();
        const nlohmann::json bundlesArg = aArgs.contains( "bundles" ) ? aArgs["bundles"] : nlohmann::json( "auto" );
        std::vector<std::pair<std::string, std::vector<std::vector<std::string>>>> bundleList;

        if( bundlesArg.is_string() && bundlesArg.get<std::string>() == "auto" )
        {
            if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
            {
                const KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

                if( r.status < 400 && r.body.contains( "bundles" ) )
                {
                    for( const auto& b : r.body["bundles"] )
                    {
                        std::vector<std::vector<std::string>> members;

                        for( const auto& m : b.value( "members", nlohmann::json::array() ) )
                            members.push_back( m.value( "nets", std::vector<std::string>() ) );

                        bundleList.emplace_back( b.value( "name", std::string() ), std::move( members ) );
                    }
                }
            }
        }
        else if( bundlesArg.is_array() )
        {
            for( const auto& b : bundlesArg )
            {
                std::vector<std::vector<std::string>> members;

                for( const auto& n : b.value( "nets", nlohmann::json::array() ) )
                    members.push_back( { n.get<std::string>() } );

                bundleList.emplace_back( b.value( "name", std::string() ), std::move( members ) );
            }
        }
        else if( !( bundlesArg.is_string() && bundlesArg.get<std::string>() == "off" ) )
        {
            return KOPENAPI_RESULT::Error( 400, "bundles: \"auto\", \"off\" or [{name, nets}]" );
        }

        // the parts a net joins: only nets between the same parts run side by side
        auto partsOf = [&]( const GRID_NET& aNet )
        {
            std::set<std::string> refs;

            for( BOARD_CONNECTED_ITEM* end : aNet.ends )
            {
                if( auto* fp = end->Type() == PCB_PAD_T ? static_cast<PAD*>( end )->GetParentFootprint() : nullptr )
                    refs.insert( str( fp->GetReference() ) );
            }

            return refs;
        };

        for( const auto& [name, members] : bundleList )
        {
            std::vector<std::tuple<size_t, int, std::set<std::string>>> found;   // grid net, rank, parts
            std::map<std::string, int>                                    netsAt;   // part -> nets of the bundle

            for( size_t m = 0; m < members.size(); ++m )
            {
                for( const std::string& netName : members[m] )
                {
                    NETINFO_ITEM* info = board->FindNet( wxString::FromUTF8( netName ) );

                    if( !info )
                        continue;

                    if( auto it = gridOf.find( info->GetNetCode() ); it != gridOf.end() && gridNets[it->second].group < 0 )
                    {
                        found.emplace_back( it->second, (int) m, partsOf( gridNets[it->second] ) );

                        for( const std::string& ref : std::get<2>( found.back() ) )
                            netsAt[ref]++;
                    }
                }
            }

            // a part only one net of the bundle reaches (a pull-up, a test point) is a branch, not a run's end
            std::map<std::string, std::vector<std::pair<size_t, int>>> runs;   // parts -> grid net, rank

            for( const auto& [k, rank, parts] : found )
            {
                std::string key;

                for( const std::string& ref : parts )
                {
                    if( netsAt[ref] > 1 )
                        key += ( key.empty() ? "" : "-" ) + ref;
                }

                runs[key].emplace_back( k, rank );
            }

            for( const auto& [parts, found] : runs )
            {
                if( found.size() < 2 )
                    continue;

                const int      group = (int) bundlesUsed.size();
                nlohmann::json nets = nlohmann::json::array();

                for( const auto& [k, rank] : found )
                {
                    gridNets[k].group = group;
                    gridNets[k].rank = rank;
                    nets.push_back( str( UnescapeString( board->FindNet( gridNets[k].netCode )->GetNetname() ) ) );
                }

                bundlesUsed.push_back( { { "name", name }, { "parts", parts }, { "nets", nets } } );
            }
        }

        std::atomic<bool> cancel{ false };
        GRID_PARAMS       params;
        params.layers = layers;
        params.cancel = &cancel;
        params.maxRounds = std::clamp( aArgs.value( "max_rounds", 40 ), 1, 500 );
        params.variants = std::clamp( aArgs.value( "variants", 0 ), 0, 6 );
        params.cell = pcbIUScale.mmToIU( std::clamp( aArgs.value( "cell_mm", 0.0 ), 0.0, 1.0 ) );
        params.viaKeepOut = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_keepout_mm", 5.0 ), 0.0, 20.0 ) );
        params.viaAway = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_away_mm", 7.0 ), 0.0, 30.0 ) );
        params.viaAwayCost = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_away_cost_mm", 8.0 ), 0.0, 100.0 ) );
        params.hug = (float) std::clamp( aArgs.value( "hug", 1.0 ), 0.05, 1.0 );
        params.astar = (float) std::clamp( aArgs.value( "astar", 1.2 ), 1.0, 3.0 );
        params.pfStart = (float) std::clamp( aArgs.value( "pf_start", 1.0 ), 0.01, 100.0 );
        params.pfGrowth = (float) std::clamp( aArgs.value( "pf_growth", 1.6 ), 1.01, 10.0 );
        params.record = aArgs.value( "record", false );

        if( aBranch )
        {
            params.branchOf = aBranch->timeline;
            params.branchLane = aBranch->lane;
            params.branchStep = aBranch->step;
        }
        params.histDecay = (float) std::clamp( aArgs.value( "hist_decay", 1.0 ), 0.1, 1.0 );
        params.recent = std::clamp( aArgs.value( "reroute_recent", 2 ), 0, 100 );

        auto scheduleOf = []( const std::string& aName ) { return aName == "adaptive" ? 1 : aName == "adaptive_hist" ? 2 : 0; };

        if( aArgs.contains( "schedule" ) )
        {
            if( !aArgs["schedule"].is_string() || ( aArgs["schedule"] != "fixed" && scheduleOf( aArgs["schedule"] ) == 0 ) )
                return KOPENAPI_RESULT::Error( 400, "schedule: fixed, adaptive or adaptive_hist" );

            params.schedule = scheduleOf( aArgs["schedule"] );
        }

        // "short_first_hv@1.0x1.6/a1.5/d0.9/r2", "fine_first@adaptive" or {order, directions, pf_start, pf_growth, astar, schedule, hist_decay, reroute_recent}
        for( const nlohmann::json& js : aArgs.value( "strategies", nlohmann::json::array() ) )
        {
            GRID_STRATEGY st;
            st.pfStart = params.pfStart;
            st.pfGrowth = params.pfGrowth;
            st.astar = params.astar;
            st.schedule = params.schedule;
            st.histDecay = params.histDecay;
            st.recent = params.recent;

            if( js.is_string() )
            {
                std::smatch m;
                const std::string text = js.get<std::string>();
                static const std::regex re( R"(^(short_first|fine_first|long_first)(?:_(hv|vh))?(?:@(?:([0-9.]+)x([0-9.]+)|(adaptive|adaptive_hist)))?(?:/a([0-9.]+))?(?:/d([0-9.]+))?(?:/r([0-9]+))?$)" );

                if( !std::regex_match( text, m, re ) )
                    return KOPENAPI_RESULT::Error( 400, "strategies: '" + text + "' is not ORDER[_hv|_vh][@START x GROWTH|@adaptive|@adaptive_hist][/aASTAR][/dDECAY][/rROUNDS]" );

                st.order = m[1];
                st.directions = m[2];

                if( m[3].matched )
                {
                    st.pfStart = std::stof( m[3] );
                    st.pfGrowth = std::stof( m[4] );
                    st.schedule = 0;
                }

                if( m[5].matched )
                    st.schedule = scheduleOf( m[5] );

                if( m[6].matched )
                    st.astar = std::stof( m[6] );

                if( m[7].matched )
                    st.histDecay = std::stof( m[7] );

                if( m[8].matched )
                    st.recent = std::stoi( m[8] );
            }
            else if( js.is_object() )
            {
                st.order = js.value( "order", st.order );
                st.directions = js.value( "directions", st.directions );
                st.pfStart = js.value( "pf_start", st.pfStart );
                st.pfGrowth = js.value( "pf_growth", st.pfGrowth );
                st.astar = js.value( "astar", st.astar );
                st.schedule = scheduleOf( js.value( "schedule", std::string( "fixed" ) ) );
                st.histDecay = js.value( "hist_decay", st.histDecay );
                st.recent = js.value( "reroute_recent", st.recent );

                if( st.order != "short_first" && st.order != "fine_first" && st.order != "long_first" )
                    return KOPENAPI_RESULT::Error( 400, "strategies[].order: short_first, fine_first or long_first" );
            }
            else
            {
                return KOPENAPI_RESULT::Error( 400, "strategies: names or objects" );
            }

            st.pfStart = std::clamp( st.pfStart, 0.01f, 100.0f );
            st.pfGrowth = std::clamp( st.pfGrowth, 1.01f, 10.0f );
            st.astar = std::clamp( st.astar, 1.0f, 3.0f );
            st.histDecay = std::clamp( st.histDecay, 0.1f, 1.0f );
            st.recent = std::clamp( st.recent, 0, 100 );
            params.strategies.push_back( st );
        }

        if( params.strategies.size() > 32 )
            return KOPENAPI_RESULT::Error( 400, "strategies: at most 32" );
        params.progress = [&]( const std::vector<GRID_VARIANT_PROGRESS>& aVariants )
        {
            if( params.record )
                GridReplayAdvance();

            nlohmann::json variants = nlohmann::json::array();
            int            finished = 0, crowded = INT_MAX;

            for( const GRID_VARIANT_PROGRESS& v : aVariants )
            {
                finished += v.stage == "done" || v.stage == "abandoned";
                crowded = std::min( crowded, v.crowded );
                variants.push_back( { { "variant", v.name }, { "stage", v.stage }, { "round", v.round },
                                      { "crowded", v.crowded }, { "net", v.net }, { "of", v.nets },
                                      { "expanded", v.expanded } } );
            }

            // done / total: variants finished; each variant's own stage below
            return KopenapiJobStep( "grid", finished, (int) aVariants.size(), std::string(),
                                    nlohmann::json{ { "nets", gridNets.size() }, { "crowded", crowded },
                                                    { "variants", variants } } );
        };

        params.held = []( uint64_t& aSerial, std::string& aBy, int& aN ) { return KopenapiJobHeld( &aSerial, &aBy, &aN ); };

        // the timeline readable (pcb_route_timeline, the replay) while the variants add to it
        params.timelineStarted = [&]( const std::shared_ptr<GRID_TIMELINE>& aTimeline )
        {
            static int runs = 0;

            if( aBranch )
            {
                GridTimelineKeep( aTimeline );
                return;
            }

            aTimeline->run = ++runs;
            aTimeline->args = aArgs.dump();

            for( int code : aTimeline->netCodes )
            {
                NETINFO_ITEM* info = board->FindNet( code );
                aTimeline->netNames.push_back( info ? str( UnescapeString( info->GetNetname() ) ) : std::to_string( code ) );
            }

            GridTimelineKeep( aTimeline );
        };

        const GRID_RESULT gr = RouteOnGrid( board, gridNets, params );
        nlohmann::json    timelineOut;

        // the replay sees the route end with its copper on the board
        struct LIVE_END
        {
            std::shared_ptr<GRID_TIMELINE> timeline;
            ~LIVE_END()
            {
                if( timeline )
                {
                    timeline->live = false;
                    KopenapiEventNotify( "pcb_route_timeline" );
                }
            }
        } liveEnd{ gr.timeline };

        if( !gr.error.empty() )
            return KOPENAPI_RESULT::Error( 409, gr.error + ( aBranch ? "; " + aBranch->board : std::string() ) );

        if( gr.timeline )
            timelineOut = { { "run", gr.timeline->run }, { "bytes", gr.timeline->Bytes() },
                            { "print_ms", std::round( gr.printMs * 10 ) / 10 } };

        if( aBranch && gr.timeline )
        {
            const GRID_LANE& lane = gr.timeline->Lane( gr.branchLane );
            timelineOut["branch"] = { { "lane", gr.branchLane }, { "variant", lane.name }, { "parent", lane.parent },
                                      { "parent_step", lane.parentStep }, { "steps", lane.steps.size() },
                                      { "restore_ms", std::round( gr.restoreMs * 10 ) / 10 } };
        }

        // each net's copper staged and guarded against the rest; a net the guard rejects stays out
        BOARD_COMMIT                        commit( context->GetToolManager() );
        std::vector<BOARD_CONNECTED_ITEM*>  staged;
        nlohmann::json                      rejected = nlohmann::json::array();
        std::vector<bool>                   netOk( gridNets.size(), false );
        std::vector<int>                    netVias( gridNets.size(), 0 );
        const auto                          guardFrom = std::chrono::steady_clock::now();

        for( size_t k = 0; k < gr.nets.size(); ++k )
        {
            const GRID_NET&                         n = gridNets[k];
            const GRID_NET_RESULT&                  r = gr.nets[k];
            NETINFO_ITEM*                           net = board->FindNet( n.netCode );
            std::vector<std::unique_ptr<PCB_TRACK>> made;
            std::vector<BOARD_CONNECTED_ITEM*>      mine;

            for( const GRID_TRACK& t : r.tracks )
            {
                for( size_t p = 1; p < t.points.size(); ++p )
                {
                    auto track = std::make_unique<PCB_TRACK>( board );
                    track->SetStart( t.points[p - 1] );
                    track->SetEnd( t.points[p] );
                    track->SetWidth( n.width );
                    track->SetLayer( t.layer );
                    track->SetNet( net );
                    mine.push_back( track.get() );
                    made.push_back( std::move( track ) );
                }
            }

            for( const VECTOR2I& at : r.vias )
            {
                auto via = std::make_unique<PCB_VIA>( board );
                via->SetPosition( at );
                via->SetViaType( VIATYPE::THROUGH );
                via->SetLayerPair( F_Cu, B_Cu );
                via->SetWidth( PADSTACK::ALL_LAYERS, n.viaDiameter );
                via->SetDrill( n.viaDrill );
                via->SetNet( net );
                mine.push_back( via.get() );
                made.push_back( std::move( via ) );
            }

            if( made.empty() )
                continue;

            const nlohmann::json violations = guard( board, mine, {}, zoneMode == "refill", &staged );

            if( !violations.empty() )
            {
                rejected.push_back( { { "net", str( UnescapeString( net->GetNetname() ) ) }, { "violations", violations } } );
                continue;
            }

            netOk[k] = true;
            netVias[k] = (int) r.vias.size();
            staged.insert( staged.end(), mine.begin(), mine.end() );

            for( std::unique_ptr<PCB_TRACK>& item : made )
            {
                added.push_back( item->m_Uuid );
                commit.Add( item.release() );
            }
        }

        if( staged.empty() )
            commit.Revert();
        else
            commit.Push( _( "Route on the grid (API)" ) );

        if( gr.timeline && !staged.empty() )
        {
            gr.timeline->appliedSeq = KOPENAPI_HISTORY::Get().PendingSeq();
            gr.timeline->appliedLane = aBranch ? gr.branchLane : gr.timeline->winner;
        }

        for( size_t k = 0; k < gridNets.size(); ++k )
        {
            const GRID_NET_RESULT* r = k < gr.nets.size() ? &gr.nets[k] : nullptr;
            bool                   first = true;

            for( size_t index : jobsOfNet[k] )
            {
                const auto [a, b] = endsOfJob[index];
                const bool ok = r && netOk[k] && r->reached[a] && r->reached[b];
                nlohmann::json row = { { "net", jobs[index].net }, { "routed", ok }, { "tries", 1 }, { "engine", "grid" } };

                if( ok )
                {
                    row["length_mm"] = first ? toMm( r->length ) : 0.0;
                    row["vias"] = first ? netVias[k] : 0;
                    row["width_mm"] = toMm( gridNets[k].width );
                    first = false;
                }
                else
                {
                    row["reason"] = !r || !r->reached[a] || !r->reached[b]
                                            ? ( KopenapiJobCancelled() ? "stopped" : "no way on the grid (or dropped to keep the rest legal)" )
                                            : "rejected by the guard: the route would touch another net";
                }

                done[index] = ok;
                rowOf[index] = (int) results.size();
                results.push_back( row );
            }
        }

        nlohmann::json crowdedNets = nlohmann::json::array();

        auto netName = [&]( int aCode ) -> std::string
        {
            NETINFO_ITEM* info = board->FindNet( aCode );
            return info ? str( UnescapeString( info->GetNetname() ) ) : std::to_string( aCode );
        };

        for( const CROWDING& c : gr.crowdedAtEnd )
        {
            nlohmann::json by = nlohmann::json::array();

            for( int code : c.by )
                by.push_back( netName( code ) );

            crowdedNets.push_back( { { "net", netName( c.net ) },
                                     { "at_mm", { toMm( c.at.x ), toMm( c.at.y ) } },
                                     { "layer", c.layer >= 0 ? str( board->GetLayerName( layers[c.layer] ) ) : std::string( "via" ) },
                                     { "by", by } } );
        }

        nlohmann::json costliest = nlohmann::json::array();

        for( const auto& [code, states] : gr.costliest )
            costliest.push_back( { { "net", netName( code ) }, { "expanded", states } } );

        gridOut = { { "variant", gr.variant }, { "variants_tried", gr.variantsTried },
                    { "lane_mb", std::round( gr.laneBytes / 1048576.0 ) },
                    { "lanes_memory_allows", gr.lanesMemoryAllows },
                    { "variants_abandoned", gr.variantsAbandoned },
                    { "variant_stats", [&]
                      {
                          nlohmann::json rows = nlohmann::json::array();

                          for( const GRID_RESULT::VARIANT_STATS& v : gr.variantStats )
                          {
                              rows.push_back( { { "variant", v.name }, { "ms", std::round( v.ms ) },
                                                { "expanded_by_stage", v.expandedByStage }, { "reached", v.reached },
                                                { "vias", v.vias }, { "abandoned", v.abandoned } } );
                          }

                          return rows;
                      }() }, { "rounds", gr.rounds },
                    { "crowded_by_round", gr.crowdedByRound }, { "crowded_at_end", crowdedNets },
                    { "dropped", gr.conflictsLeft }, { "cell_mm", toMm( gr.cell ) }, { "expanded", gr.expanded },
                    { "build_ms", std::round( gr.buildMs ) }, { "route_ms", std::round( gr.routeMs ) },
                    { "guard_ms", std::round( msSince( guardFrom ) ) }, { "rejected", rejected },
                    { "costliest", costliest }, { "expanded_by_stage", gr.expandedByStage }, { "bundles", bundlesUsed },
                    { "timeline", timelineOut } };
        board->BuildConnectivity();
    }

    // Rip-up and retry: for a connection that failed, take out the routes of this call that cross
    // its corridor, route it, route them again; if anything of that fails, everything goes back
    // as it was (walkaround only: no other net is pushed aside; the copper of the nets it touched
    // is put back from copies)
    const int ripRounds = dryRun || grid ? 0 : std::clamp( aArgs.value( "rip_up", 2 ), 0, 6 );
    int       rippedOk = 0, rippedTried = 0;

    // the rip-ups that failed, by board version and the routes taken out; how often each route has
    // been taken out (its copper costs a plan more each time)
    int                                                        boardVersion = 0;
    std::vector<std::set<std::pair<int, std::vector<size_t>>>> ripFailed( jobs.size() );
    std::vector<int>                                           ripHistory( jobs.size(), 0 );
    constexpr int                                              RIP_CELL_COST = 30;
    constexpr size_t                                           MAX_VICTIMS = 4;
    constexpr int                                              RIP_PICKS = 3;

    auto chainOf = [&]( JOB& aJob ) -> std::vector<PCB_TRACK*>
    {
        PAD* a = dynamic_cast<PAD*>( board->ResolveItem( aJob.fromId, true ) );
        PAD* b = dynamic_cast<PAD*>( board->ResolveItem( aJob.toId, true ) );

        if( !a || !b )
            return {};

        return copperBetween( board, a, b );
    };

    for( int round = 0; round < ripRounds && !KopenapiJobCancelled(); ++round )
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

            if( !KopenapiJobStep( "rip_up " + std::to_string( round + 1 ), (int) index, (int) jobs.size(), job.net,
                                  progressCounts() ) )
            {
                break;
            }

            // first from both ends, nothing taken out
            if( round == 0 )
            {
                nlohmann::json attempt;
                int            tries = 0;

                s_routeProfile.pass = "retry";

                // until a rip-up has moved routes, the board has only gained copper since the first
                // pass: the forward way failed there already
                if( tryRoute( job, attempt, tries, false, true, rippedOk > 0 ) )
                {
                    progress = true;
                    boardVersion++;
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

            // The routes to take out: the ones a plan crosses when their copper may be crossed at a
            // cost that rises each time a route has been taken out (it gives way less readily); else
            // (no such plan on the grid, which is stricter than the router; a plan through free space
            // alone, which the router missed; no planning) the nearest three in the corridor
            bool corridorTaken = false;

            // routes of this call that can be taken out for aTarget and routed again, with their
            // copper; aKeep stay
            auto movableFor = [&]( const JOB& aTarget, const std::set<size_t>& aKeep )
            {
                // copper of the pairs' trunks is never taken out (looked up for each pick: a failed
                // rip-up puts copies back)
                std::set<PCB_TRACK*> trunkCopper;

                for( size_t t = 0; t < trunks.size(); ++t )
                {
                    if( done[t] )
                    {
                        for( PCB_TRACK* track : chainOf( jobs[t] ) )
                            trunkCopper.insert( track );
                    }
                }

                std::vector<std::pair<size_t, std::vector<PCB_TRACK*>>> movable;

                for( size_t other = 0; other < jobs.size(); ++other )
                {
                    if( !done[other] || aKeep.count( other ) || partner[other] >= 0 || jobs[other].to->Type() != PCB_PAD_T
                        || results[rowOf[other]].value( "joined_by_earlier_route", false )
                        || jobs[other].from->GetNetCode() == aTarget.from->GetNetCode() )
                    {
                        continue;   // pairs stay; a joined airwire has no copper of its own; the own net is no obstacle
                    }

                    std::vector<PCB_TRACK*> chain = chainOf( jobs[other] );

                    if( chain.empty()
                        || std::any_of( chain.begin(), chain.end(),
                                        [&]( PCB_TRACK* t ) { return trunkCopper.count( t ) || t->IsLocked(); } ) )
                    {
                        continue;
                    }

                    movable.emplace_back( other, std::move( chain ) );
                }

                return movable;
            };

            // the routes a plan for aTarget crosses when their copper costs more each time it was
            // taken out; nullopt: walled in even through them
            auto planVictims = [&]( const JOB& aTarget, const std::vector<std::pair<size_t, std::vector<PCB_TRACK*>>>& aMovable )
                    -> std::optional<std::vector<size_t>>
            {
                ROUTE_PLAN_REQUEST req = planRequest( aTarget.from, aTarget.to, aTarget.toPos );

                for( const auto& [other, chain] : aMovable )
                {
                    ROUTE_PLAN_REQUEST::SOFT soft;
                    soft.copper.assign( chain.begin(), chain.end() );
                    soft.cellCost = RIP_CELL_COST * ( 1 + 2 * ripHistory[other] );
                    req.soft.push_back( std::move( soft ) );
                }

                const ROUTE_PLAN plan = makePlan( req );

                if( !plan.found )
                    return std::nullopt;

                std::vector<size_t> out;

                if( plan.crossed.size() <= MAX_VICTIMS )
                {
                    for( int k : plan.crossed )
                        out.push_back( aMovable[k].first );
                }

                return out;
            };

            auto pickVictims = [&]() -> std::vector<size_t>
            {
                const std::vector<std::pair<size_t, std::vector<PCB_TRACK*>>> movable = movableFor( job, {} );

                if( movable.empty() )
                    return {};

                if( planning && !rail && !corridorTaken )
                {
                    const std::optional<std::vector<size_t>> planned = planVictims( job, movable );

                    // walled in by copper that cannot move
                    if( !planned )
                    {
                        s_routeProfile.ripUpsWalled++;
                        return {};
                    }

                    if( !planned->empty() )
                    {
                        s_routeProfile.ripUpsPlanned++;
                        return *planned;
                    }
                }
                else if( corridorTaken )
                {
                    return {};
                }

                corridorTaken = true;

                BOX2I corridor( job.from->GetPosition(), VECTOR2I( 0, 0 ) );
                corridor.Merge( job.toPos );
                corridor.Inflate( pcbIUScale.mmToIU( 1.5 ) );

                std::vector<std::pair<double, size_t>> crossing;

                for( const auto& [other, chain] : movable )
                {
                    for( PCB_TRACK* t : chain )
                    {
                        if( t->GetBoundingBox().Intersects( corridor ) )
                        {
                            crossing.emplace_back( ( t->GetBoundingBox().Centre() - corridor.Centre() ).EuclideanNorm(), other );
                            break;
                        }
                    }
                }

                std::sort( crossing.begin(), crossing.end() );

                if( crossing.size() > 3 )
                    crossing.resize( 3 );

                std::vector<size_t> out;

                for( const auto& [d, other] : crossing )
                    out.push_back( other );

                return out;
            };

            for( int pick = 0; pick < RIP_PICKS && !done[index]; ++pick )
            {
                std::vector<size_t> victims = pickVictims();

                if( victims.empty() )
                    break;

                std::sort( victims.begin(), victims.end() );

                // a failed rip-up puts everything back exactly: until another route changes the
                // board, the same one fails again
                if( ripFailed[index].count( { boardVersion, victims } ) )
                {
                    s_routeProfile.ripUpsSkipped++;
                    break;
                }

                rippedTried++;

                for( size_t v : victims )
                    ripHistory[v]++;

                // the copper of every net the rip-up touches as it is now, to put back: the router
                // also merges and trims the copper of the net it routes, not only adds to it
                std::set<int> touched = { job.from->GetNetCode() };

                for( size_t other : victims )
                    touched.insert( jobs[other].from->GetNetCode() );

                std::vector<std::unique_ptr<PCB_TRACK>> saved;

                for( PCB_TRACK* t : board->Tracks() )
                {
                    if( touched.count( t->GetNetCode() ) )
                        saved.emplace_back( static_cast<PCB_TRACK*>( t->Clone() ) );
                }

                // take them out
                BOARD_COMMIT         rip( context->GetToolManager() );
                std::set<PCB_TRACK*> taken;   // victims of one net can share copper: each piece once

                for( size_t other : victims )
                {
                    for( PCB_TRACK* t : chainOf( jobs[other] ) )
                    {
                        if( taken.insert( t ).second )
                            rip.Remove( t );
                    }
                }

                rip.Push( _( "Rip up (API)" ) );
                const size_t firstNew = added.size();

                nlohmann::json attempt;
                int            tries = 0;
                s_routeProfile.pass = "rip_up";
                bool           ok = tryRoute( job, attempt, tries, false, true );
                nlohmann::json victimAttempts = nlohmann::json::array();
                const char*    failedAt = ok ? nullptr : "route";

                // every route taken out by this rip-up, the second level's too
                std::vector<size_t> allTaken = victims;
                std::vector<size_t> secondTaken;

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

                    // a victim with no way back: a plan for it chooses routes it may move in turn (not
                    // the new route, not the others taken out); they go back after it
                    if( !ok && victim.from && victim.to && planning && !rail )
                    {
                        std::set<size_t> keep( allTaken.begin(), allTaken.end() );
                        keep.insert( index );

                        const auto movable = movableFor( victim, keep );
                        const auto second = movable.empty() ? std::nullopt : planVictims( victim, movable );

                        if( second && !second->empty() )
                        {
                            s_routeProfile.ripUpsSecond++;

                            BOARD_COMMIT         secondRip( context->GetToolManager() );
                            std::set<PCB_TRACK*> pieces;

                            for( size_t other : *second )
                            {
                                ripHistory[other]++;

                                if( touched.insert( jobs[other].from->GetNetCode() ).second )
                                {
                                    for( PCB_TRACK* t : board->Tracks() )
                                    {
                                        if( t->GetNetCode() == jobs[other].from->GetNetCode() )
                                            saved.emplace_back( static_cast<PCB_TRACK*>( t->Clone() ) );
                                    }
                                }

                                for( PCB_TRACK* t : chainOf( jobs[other] ) )
                                {
                                    if( pieces.insert( t ).second )
                                        secondRip.Remove( t );
                                }
                            }

                            secondRip.Push( _( "Rip up (API)" ) );
                            allTaken.insert( allTaken.end(), second->begin(), second->end() );

                            ok = tryRoute( victim, again, againTries, false, true );

                            for( size_t other : *second )
                            {
                                if( !ok )
                                    break;

                                JOB& moved = jobs[other];
                                moved.from = dynamic_cast<PAD*>( board->ResolveItem( moved.fromId, true ) );
                                moved.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( moved.toId, true ) );
                                nlohmann::json movedAttempt;
                                int            movedTries = 0;

                                ok = moved.from && moved.to && tryRoute( moved, movedAttempt, movedTries, false, true );
                                secondTaken.push_back( other );
                            }
                        }
                    }

                    victimAttempts.push_back( again );

                    if( !ok )
                        failedAt = "victim";
                }

                // a victim's copper can be shared with other connections of its net: one left cut is
                // routed again too
                std::vector<size_t> cut;

                for( size_t k = 0; ok && k < jobs.size(); ++k )
                {
                    if( !done[k] || k == index || jobs[k].trunk || partner[k] >= 0 || rowOf[k] < 0
                        || std::find( allTaken.begin(), allTaken.end(), k ) != allTaken.end()
                        || !results[rowOf[k]].value( "routed", false ) )
                    {
                        continue;
                    }

                    if( std::none_of( allTaken.begin(), allTaken.end(),
                                      [&]( size_t v ) { return jobs[v].net == jobs[k].net; } ) )
                    {
                        continue;
                    }

                    auto* a = dynamic_cast<PAD*>( board->ResolveItem( jobs[k].fromId, true ) );
                    auto* b = dynamic_cast<PAD*>( board->ResolveItem( jobs[k].toId, true ) );

                    if( !a || !b )
                        continue;

                    const auto joined = board->GetConnectivity()->GetConnectedItems( a );

                    if( std::find( joined.begin(), joined.end(), b ) != joined.end() )
                        continue;

                    JOB& again = jobs[k];
                    again.from = a;
                    again.to = b;
                    nlohmann::json attemptAgain;
                    int            triesAgain = 0;

                    ok = tryRoute( again, attemptAgain, triesAgain, false, true );
                    cut.push_back( k );

                    if( !ok )
                        failedAt = "cut";
                }

                if( ok )
                {
                    rippedOk++;
                    progress = true;
                    boardVersion++;
                    done[index] = true;

                    nlohmann::json& row = results[rowOf[index]];
                    row["routed"] = true;
                    row.erase( "reason" );
                    row.erase( "violations" );
                    row.erase( "stopped_at_mm" );
                    row.erase( "blocked_by" );
                    row.erase( "rip_ups_failed" );
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

                    for( size_t k : cut )
                        results[rowOf[k]]["rerouted"] = true;

                    for( size_t k : secondTaken )
                        results[rowOf[k]]["rerouted"] = true;

                    break;
                }

                // put everything back: the copper of those nets out, the copies in
                BOARD_COMMIT undo( context->GetToolManager() );

                for( PCB_TRACK* t : board->Tracks() )
                {
                    if( touched.count( t->GetNetCode() ) )
                        undo.Remove( t );
                }

                added.resize( firstNew );

                for( std::unique_ptr<PCB_TRACK>& t : saved )
                    undo.Add( t.release() );

                undo.Push( _( "Rip up undone (API)" ) );
                board->GetConnectivity()->RecalculateRatsnest();
                ripFailed[index].insert( { boardVersion, victims } );
                s_routeProfile.ripUpsFailedAt[failedAt]++;

                nlohmann::json takenOut = nlohmann::json::array();

                for( size_t v : victims )
                    takenOut.push_back( jobs[v].net + " " + str( jobs[v].from->GetParentFootprint()->GetReference() ) + "."
                                     + str( jobs[v].from->GetNumber() ) );

                results[rowOf[index]]["rip_ups_failed"].push_back( { { "taken_out", takenOut }, { "failed_at", failedAt } } );
            }
        }

        if( !progress )
            break;
    }

    // Gaps between two pieces of a net's copper: from the end of one piece to the other
    s_routeProfile.pass = "gaps";

    for( size_t gapIndex = 0; gapIndex < gaps.size(); ++gapIndex )
    {
        const GAP& gap = gaps[gapIndex];

        if( !KopenapiJobStep( "gaps", (int) gapIndex, (int) gaps.size(), gap.net, progressCounts() ) )
            break;

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

            for( const char* key : { "violations", "stopped_at_mm", "blocked_by", "plan" } )
            {
                if( attempt.contains( key ) )
                    row[key] = attempt[key];
            }
        }

        results.push_back( row );
    }

    // Clean-up: a routed connection with vias or a detour is routed again alone on a layer both
    // ends are on (walkaround: it moves nothing else); kept when it has fewer vias, or as many and
    // is 5 % shorter, and no connection of its net is left cut; else put back as it was
    int cleanTried = 0, cleanSkipped = 0, cleanKept = 0, viasRemoved = 0;
    double shortenedIU = 0;

    if( !dryRun && !rail && !grid && aArgs.value( "clean_up", true ) )
    {
        s_routeProfile.pass = "clean_up";

        struct CLEAN_TRY
        {
            size_t index;
            int    vias;
            double length;
        };

        std::vector<CLEAN_TRY> candidates;
        std::set<PCB_TRACK*>   trunkCopper;   // pairs stay as they are

        for( size_t t = 0; t < trunks.size(); ++t )
        {
            if( done[t] )
            {
                for( PCB_TRACK* track : chainOf( jobs[t] ) )
                    trunkCopper.insert( track );
            }
        }

        for( size_t k = 0; k < jobs.size(); ++k )
        {
            if( !done[k] || jobs[k].trunk || partner[k] >= 0 || rowOf[k] < 0 || jobs[k].to->Type() != PCB_PAD_T
                || !results[rowOf[k]].value( "routed", false ) || results[rowOf[k]].value( "joined_by_earlier_route", false ) )
            {
                continue;
            }

            const std::vector<PCB_TRACK*> chain = chainOf( jobs[k] );

            if( chain.empty()
                || std::any_of( chain.begin(), chain.end(),
                                [&]( PCB_TRACK* t ) { return t->IsLocked() || trunkCopper.count( t ); } ) )
            {
                continue;
            }

            int    vias = 0;
            double length = 0;

            for( PCB_TRACK* t : chain )
            {
                if( t->Type() == PCB_VIA_T )
                    vias++;
                else
                    length += t->GetLength();
            }

            const double direct = ( jobs[k].toPos - jobs[k].from->GetPosition() ).EuclideanNorm();

            if( vias > 0 || length > 1.3 * direct )
                candidates.push_back( { k, vias, length } );
        }

        std::stable_sort( candidates.begin(), candidates.end(),
                          []( const CLEAN_TRY& x, const CLEAN_TRY& y ) { return x.vias > y.vias; } );

        for( size_t cleanIndex = 0; cleanIndex < candidates.size(); ++cleanIndex )
        {
            const CLEAN_TRY& c = candidates[cleanIndex];

            if( !KopenapiJobStep( "clean_up", (int) cleanIndex, (int) candidates.size(), jobs[c.index].net ) )
                break;

            JOB& job = jobs[c.index];
            job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
            job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

            if( !job.from || !job.to )
                continue;

            std::vector<PCB_LAYER_ID> shared;

            for( PCB_LAYER_ID layer : layers )
            {
                if( job.from->IsOnLayer( layer ) && job.to->IsOnLayer( layer ) )
                    shared.push_back( layer );
            }

            const std::vector<PCB_TRACK*> chain = chainOf( job );

            if( shared.empty() || chain.empty() )
                continue;

            // the free space must hold a way on those layers that could win (the net's own copper
            // is no obstacle to the plan, so the chain stays until then)
            if( planning )
            {
                ROUTE_PLAN_REQUEST req = planRequest( job.from, job.to, job.toPos );
                req.layers = shared;
                req.viaDiameter = 0;

                const ROUTE_PLAN plan = makePlan( req );

                if( !plan.found || ( c.vias == 0 && plan.length >= c.length ) )
                {
                    cleanSkipped++;
                    continue;
                }
            }

            cleanTried++;

            const int netCode = job.from->GetNetCode();
            std::vector<std::unique_ptr<PCB_TRACK>> saved;

            for( PCB_TRACK* t : board->Tracks() )
            {
                if( t->GetNetCode() == netCode )
                    saved.emplace_back( static_cast<PCB_TRACK*>( t->Clone() ) );
            }

            BOARD_COMMIT out( context->GetToolManager() );

            for( PCB_TRACK* t : chain )
                out.Remove( t );

            out.Push( _( "Clean up (API)" ) );
            const size_t firstNew = added.size();

            nlohmann::json attempt;
            bool           ok = false;

            for( PCB_LAYER_ID layer : shared )
            {
                for( bool reverse : { false, true } )
                {
                    if( ok )
                        break;

                    PAD*                  from = job.from;
                    BOARD_CONNECTED_ITEM* to = job.to;
                    VECTOR2I              toPos = job.toPos;

                    if( reverse )
                    {
                        from = static_cast<PAD*>( job.to );
                        to = job.from;
                        toPos = job.from->GetPosition();
                    }

                    ROUTE_REQUEST req{ from, to, toPos, layer, PNS::RM_Walkaround, false, {} };
                    req.fronts = &fronts;
                    req.neckWidth = neckFor( from, to );
                    attempt = nlohmann::json::object();
                    ok = routeOne( *context, req, attempt, added );
                }
            }

            int    vias = 0;
            double length = 0;

            if( ok )
            {
                for( PCB_TRACK* t : chainOf( job ) )
                {
                    if( t->Type() == PCB_VIA_T )
                        vias++;
                    else
                        length += t->GetLength();
                }

                ok = vias < c.vias || ( vias == c.vias && length < 0.95 * c.length );
            }

            // every routed connection of the net still joined
            for( size_t k = 0; ok && k < jobs.size(); ++k )
            {
                if( k == c.index || !done[k] || rowOf[k] < 0 || jobs[k].from->GetNetCode() != netCode
                    || !results[rowOf[k]].value( "routed", false ) )
                {
                    continue;
                }

                auto* a = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( jobs[k].fromId, true ) );
                auto* b = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( jobs[k].toId, true ) );

                if( a && b )
                {
                    const auto joined = board->GetConnectivity()->GetConnectedItems( a );
                    ok = std::find( joined.begin(), joined.end(), b ) != joined.end();
                }
            }

            if( ok )
            {
                cleanKept++;
                viasRemoved += c.vias - vias;
                shortenedIU += c.length - length;

                nlohmann::json& row = results[rowOf[c.index]];
                row["cleaned"] = true;
                row["layer"] = attempt["layer"];
                row["mode"] = attempt["mode"];
                row["length_mm"] = attempt["metrics"]["length_mm"];
                row["vias"] = vias;
                row["detour"] = attempt["metrics"]["detour"];
                continue;
            }

            BOARD_COMMIT undo( context->GetToolManager() );

            for( PCB_TRACK* t : board->Tracks() )
            {
                if( t->GetNetCode() == netCode )
                    undo.Remove( t );
            }

            added.resize( firstNew );

            for( std::unique_ptr<PCB_TRACK>& t : saved )
                undo.Add( t.release() );

            undo.Push( _( "Clean up undone (API)" ) );
            board->GetConnectivity()->RecalculateRatsnest();
        }
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

    // nets of the call: copper of other nets that a shove laid again is theirs and stays
    std::set<int> callNets;

    for( const JOB& job : jobs )
    {
        callNets.insert( job.from->GetNetCode() );

        if( NETINFO_ITEM* coupled = job.trunk ? board->DpCoupledNet( job.from->GetNet() ) : nullptr )
            callNets.insert( coupled->GetNetCode() );
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

            if( !track || track->IsLocked() || !callNets.count( track->GetNetCode() ) )
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

    for( size_t k = 0; k < jobs.size(); k++ )
    {
        if( rowOf[k] >= 0 && results[rowOf[k]].is_object() )
            results[rowOf[k]]["ms"] = std::round( jobs[k].ms );
    }

    // other nets whose copper the routes pushed aside and that did not come back as they were
    nlohmann::json                              moved = nlohmann::json::array();
    const std::map<int, std::pair<int, double>> copperAfter = copperOf();

    for( int net : s_routeProfile.shovedNets )
    {
        auto before = copperBefore.find( net );
        auto after = copperAfter.find( net );
        const std::pair<int, double> b = before != copperBefore.end() ? before->second : std::pair<int, double>( 0, 0.0 );
        const std::pair<int, double> a = after != copperAfter.end() ? after->second : std::pair<int, double>( 0, 0.0 );

        if( callNets.count( net ) || ( b.first == a.first && std::abs( a.second - b.second ) < 1.0 ) )
            continue;

        NETINFO_ITEM* info = board->FindNet( net );
        moved.push_back( { { "net", info ? str( info->GetNetname() ) : std::to_string( net ) },
                           { "pieces", { b.first, a.first } },
                           { "length_mm", { toMm( b.second ), toMm( a.second ) } } } );
    }

    return KOPENAPI_RESULT::Ok( { { "connections", connections },
                                  { "other_nets_moved", moved },
                                  { "routed", routed },
                                  { "failed", connections - routed },
                                  { "length_mm", std::round( length * 1000.0 ) / 1000.0 },
                                  { "vias", vias },
                                  { "rip_up", { { "tried", rippedTried }, { "won", rippedOk } } },
                                  { "clean_up", { { "tried", cleanTried }, { "skipped", cleanSkipped }, { "kept", cleanKept }, { "vias_removed", viasRemoved },
                                                  { "shortened_mm", std::round( pcbIUScale.IUTomm( shortenedIU ) * 1000.0 ) / 1000.0 } } },
                                  { "stubs_removed", stubs },
                                  { "kept_out_under", keptOut },
                                  { "grid", gridOut },
                                  { "plane_layers_skipped", planeLayers },
                                  { "dry_run", dryRun },
                                  { "stopped", KopenapiJobCancelled() },
                                  { "unrouted_left", dryRun ? connections - routed : (int) connectivity->GetUnconnectedCount( false ) },
                                  { "ms", std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() },
                                  { "profile", { { "attempts", s_routeProfile.attempts },
                                                 { "dead_legs_skipped", s_routeProfile.deadLegsSkipped },
                                                 { "via_spots_blocked", s_routeProfile.viaSpotsBlocked },
                                                 { "rip_ups_skipped", s_routeProfile.ripUpsSkipped },
                                                 { "rip_ups_planned", s_routeProfile.ripUpsPlanned },
                                                 { "rip_ups_walled", s_routeProfile.ripUpsWalled },
                                                 { "rip_ups_failed_at", s_routeProfile.ripUpsFailedAt },
                                                 { "rip_ups_second", s_routeProfile.ripUpsSecond },
                                                 { "pairs_stuck", s_routeProfile.pairsStuck },
                                                 { "plans", s_routeProfile.plans },
                                                 { "plans_found", s_routeProfile.plansFound },
                                                 { "plan_ms", std::round( s_routeProfile.planMs ) },
                                                 { "sync_ms", std::round( s_routeProfile.syncMs ) },
                                                 { "move_ms", std::round( s_routeProfile.moveMs ) },
                                                 { "route_ms", std::round( s_routeProfile.totalMs - s_routeProfile.syncMs ) },
                                                 { "by_kind", tallies( s_routeProfile.byKind ) },
                                                 { "failed_by", tallies( s_routeProfile.byFailure ) },
                                                 { "by_pass", tallies( s_routeProfile.byPass ) } } },
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

        for( size_t endIndex = 0; endIndex < ends.size(); ++endIndex )
        {
            const auto& [item, pos] = ends[endIndex];

            if( !KopenapiJobStep( "stitch", (int) endIndex, (int) ends.size() ) )
                break;

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
                                  { "islands_left", [&]()
                                    {
                                        // pieces of the net's pours no via reached: cut off from the rest
                                        nlohmann::json out = nlohmann::json::array();

                                        for( ZONE* zone : board->Zones() )
                                        {
                                            if( zone->GetNetCode() != net->GetNetCode() || zone->GetIsRuleArea() )
                                                continue;

                                            for( nlohmann::json& piece : KopenapiZoneIslands( board, zone ) )
                                            {
                                                if( piece.value( "vias", 0 ) == 0 )
                                                {
                                                    piece["zone"] = str( zone->GetZoneName() );
                                                    out.push_back( piece );
                                                }
                                            }
                                        }

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
    const std::string          regionMode = aArgs.value( "region_mode", std::string( "inside" ) );
    const bool                 touching = regionMode == "touching" || regionMode == "cut";
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

        // the copper itself against the region (a diagonal's box may cross it where the track does not)
        if( region && !track->HitTest( *region, !touching, 0 ) )
            continue;

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
    const std::optional<BOX2I> region = regionOf( aArgs );
    const bool               cut = region && aArgs.value( "region_mode", std::string() ) == "cut";
    std::vector<PCB_TRACK*>  doomed;
    std::vector<std::pair<VECTOR2I, VECTOR2I>> keptPieces;   // the parts of cut tracks outside the region
    std::vector<PCB_TRACK*>  keptFrom;
    int                      cutCount = 0;
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
            double gone = track->GetLength();

            // region_mode cut: a straight track crossing the region's edge loses only its part inside
            if( cut && track->Type() == PCB_TRACE_T && !region->Contains( track->GetBoundingBox() ) )
            {
                const VECTOR2D a = track->GetStart(), b = track->GetEnd(), d = b - a;
                double         t0 = 0, t1 = 1;
                const double   lo[2] = { (double) region->GetLeft(), (double) region->GetTop() };
                const double   hi[2] = { (double) region->GetRight(), (double) region->GetBottom() };
                const double   p0[2] = { a.x, a.y }, dd[2] = { d.x, d.y };

                for( int k = 0; k < 2; ++k )
                {
                    if( dd[k] == 0 )
                        continue;

                    double e = ( lo[k] - p0[k] ) / dd[k], l = ( hi[k] - p0[k] ) / dd[k];

                    if( e > l )
                        std::swap( e, l );

                    t0 = std::max( t0, e );
                    t1 = std::min( t1, l );
                }

                auto at = [&]( double t ) { return VECTOR2I( KiROUND( a.x + d.x * t ), KiROUND( a.y + d.y * t ) ); };

                if( t0 > 0 )
                {
                    keptPieces.emplace_back( track->GetStart(), at( t0 ) );
                    keptFrom.push_back( track );
                }

                if( t1 < 1 )
                {
                    keptPieces.emplace_back( at( t1 ), track->GetEnd() );
                    keptFrom.push_back( track );
                }

                gone = std::max( 0.0, t1 - t0 ) * track->GetLength();
                cutCount++;
            }

            tracks++;
            length += gone;
            net["tracks"] = net["tracks"].get<int>() + 1;
            net["length_mm"] = net["length_mm"].get<double>() + toMm( gone );
        }
    }

    nlohmann::json out = { { "deleted", { { "tracks", tracks }, { "vias", vias }, { "length_mm", toMm( length ) } } },
                           { "by_net", byNet }, { "missing", missing }, { "locked_skipped", locked },
                           { "applied", false } };

    if( cut )
        out["cut"] = cutCount;

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

    for( size_t i = 0; i < keptPieces.size(); ++i )
    {
        PCB_TRACK* piece = static_cast<PCB_TRACK*>( keptFrom[i]->Duplicate( IGNORE_PARENT_GROUP ) );
        piece->SetStart( keptPieces[i].first );
        piece->SetEnd( keptPieces[i].second );
        commit.Add( piece );
    }

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
                        "layer":{"type":"string","description":"start layer; default the 'from' pad's (F.Cu, B.Cu for a pad on the back only). A 'to' on another layer than the route arrives on gets a via in front of it (via_near_target_mm)"},
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
                   "call a plan crosses - each costs the plan more every time it was taken out - else the nearest "
                   "in its corridor, are taken out, it is routed, they are routed again - or all goes back; none "
                   "when no plan exists even through them); "
                   "nets named as a differential pair (+/-, P/N suffix) between the same two footprints are "
                   "routed coupled by KiCad's diff-pair router (width / gap from the rules, else track width "
                   "and clearance), one by one if that fails (row diff_pair tells); "
                   "every route passes the net-merge guard; copper of this call that ends nowhere is removed "
                   "(stubs_removed); copper of nets outside the call that shove pushed aside is listed "
                   "(other_nets_moved: pieces and length before / after); a row still routed at the end is "
                   "joined by copper; per connection "
                   "routed / reason and ms (every try for it), totals (routed, length, vias, unrouted left) and "
                   "profile (attempts; dead_legs_skipped / via_spots_blocked / rip_ups_skipped / rip_ups_walled: tries not made; "
                   "rip_ups_planned: rip-ups whose routes a plan chose; rip_ups_second: routes taken out that had no way back "
                   "and moved others in turn; rip_ups_failed_at: failed rip-ups by what failed (route, victim, cut); a row left "
                   "failed lists its rip_ups_failed (taken_out, failed_at); "
                   "pairs_stuck: pair tries ended when nothing was placed after 3 steps; plans / plans_found / plan_ms: global plans; sync_ms: building the router's world for each; move_ms: the path search; "
                   "route_ms: all but the world; by_kind: tries / routed / ms on a layer (walkaround, shove), "
                   "through one via, two vias; failed_by: failed tries by reason; by_pass: first, retry from both ends, "
                   "rip_up, gaps); "
                   "dry_run routes nothing",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"string"}},
                        "layers":{"type":"array","items":{"type":"string"},"default":["F.Cu","B.Cu"]},
                        "order":{"type":"string","enum":["short_first","power_first","dense_first"],"default":"short_first","description":"dense_first: connections at fine-pitch pads (<= 0.65 mm) first"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills out while routing (they would block every crossing), refilled after; keep: routes must clear the fills"},
                        "clean_up":{"type":"boolean","default":true,"description":"after routing, each connection with vias or a detour (over 1.3 x the straight line) is routed again alone on a layer both ends are on, without moving anything; kept with fewer vias, or as many and 5 % shorter (a plan over the free space of those layers first: none, or no shorter, skips it; clean_up: tried / skipped / kept / vias_removed / shortened_mm; rows cleaned)"},
                        "rip_up":{"type":"integer","default":2,"minimum":0,"maximum":6,"description":"rounds of rip-up and retry for failed connections (routes of this call in their way taken out and routed again, up to 3 choices of them per connection and round; all back if that fails)"},
                        "vias":{"type":"boolean","default":true,"description":"connections blocked on their side: escape through vias near both ends"},
                        "diff_pairs":{"type":"string","enum":["auto","off"],"default":"auto","description":"auto: pairs (USB_D+ / USB_D-, LVDS_P / LVDS_N) routed coupled; off: every net alone"},
                        "engine":{"type":"string","enum":["pns","grid"],"default":"pns","description":"grid: every net at once on one grid of the board, negotiated congestion (PathFinder) in parallel variants, the best legal result committed (diff-pair trunks still by pns; no rip_up / clean_up passes); pns: one connection at a time with the push-and-shove router"},
                        "max_rounds":{"type":"integer","default":40,"description":"grid: negotiation rounds before the nets still crowding each other are routed apart or left out"},
                        "cell_mm":{"type":"number","default":0,"description":"grid: the cell, 0 a sixth of the finest track pitch (width + clearance) on a 0.025 mm step"},
                        "via_keepout_mm":{"type":"number","default":5,"description":"grid: only a chip's own nets put vias this far around it (pads on four sides, or an array outside its balls) while the board routes; other nets left short and, once all are joined, the via-saving pass may use it; 0 off"},
                        "bundles":{"description":"grid: \"auto\" (pcb_net_roles' bundles), \"off\", or [{name, nets}] in member order: a bundle's nets between the same parts are routed first, side by side (the answer's grid.bundles: name, parts, nets)","default":"auto"},
                        "via_away_mm":{"type":"number","default":7,"description":"grid: a signal via nearer a chip's pads (not an array's) than this costs more, linearly to the pads; 0 off"},
                        "via_away_cost_mm":{"type":"number","default":8,"description":"grid: a via at a chip's pads costs as much more as this length of track"},
                        "hug":{"type":"number","default":1,"description":"grid: cost factor of a cell beside the copper of the net's own bundle (1: no pull; below 1 so far lowers cohesion, adds vias and time)"},
                        "astar":{"type":"number","default":1.2,"description":"grid: weight of the distance left in the search (1: cheapest paths; more: fewer states searched, paths a little longer)"},
                        "pf_start":{"type":"number","default":1.0,"description":"grid: price of sharing a cell in the first negotiation round"},
                        "pf_growth":{"type":"number","default":1.6,"description":"grid: how much that price grows each round (more: fewer rounds, longer detours)"},
                        "record":{"type":"boolean","default":false,"description":"grid: keep every variant's steps for pcb_route_timeline"},
                        "from":{"type":"object","description":"grid, a branch: the recorded lane goes on after a step as a new lane of the same run (pcb_route_timeline status: parent, parent_step), with the run's own arguments; the copper the run (or its last branch) put on the board is rolled back first when that is the last edit, else the board must be as the run found it (409). Up to the step the recorded nets are taken without searching, so the branch's steps equal the recorded ones until it searches on","properties":{"run":{"type":"integer"},"lane":{"description":"index or variant name; default the lane on the board"},"step":{"type":"integer"}},"required":["step"]},
                        "schedule":{"type":"string","enum":["fixed","adaptive","adaptive_hist"],"default":"fixed","description":"grid, EXPERIMENTAL: negotiation schedule of the built-in variants (adaptive: growth 1.3 while the crowded nets fall, 2.0 when they stall; adaptive_hist: also twice the history on places fought over 3 rounds running)"},
                        "strategies":{"type":"array","description":"grid, EXPERIMENTAL: the variants to race instead of the built-ins: 'short_first_hv@1.0x1.6/a1.5/d0.9/r2', 'fine_first@adaptive', or {order, directions, pf_start, pf_growth, astar, schedule, hist_decay, reroute_recent}","items":{}},
                        "hist_decay":{"type":"number","default":1.0,"description":"grid, EXPERIMENTAL: the history cost of places not fought over in a round is multiplied by this (1: kept for good)"},
                        "reroute_recent":{"type":"integer","default":2,"description":"grid: a net that crowds nobody is routed again only through places fought over in the last this many rounds (0: through any place ever fought over)"},
                        "variants":{"type":"integer","default":0,"description":"grid: how many of the built-in variants (pcb_route_strategies) run in parallel; 0 one per two cores, as many as half the available memory holds (grid.lane_mb, grid.lanes_memory_allows)"},
                        "plane_layers":{"type":"boolean","default":false,"description":"route on plane layers too (type power in the stack-up, or an inner layer more than half of which one net's pours cover): skipped by default, a track would cut the plane; the answer lists plane_layers_skipped"},
                        "under_parts":{"type":"string","enum":["allow","avoid"],"default":"allow","description":"avoid: no track or via of this call inside the ring of pads of a quad package (QFP / QFN / LGA, pads on four sides) on its side - tracks there can wall its other pins in (blocked_by names them); parts with pads inside the ring (exposed pad, BGA) stay open; the answer lists kept_out_under. Costs routes on a crowded board (TC-13: 77 -> 70 of 81)"},
                        "neck_down":{"oneOf":[{"type":"string","enum":["auto","off"]},{"type":"object","additionalProperties":{"type":"string","enum":["auto","off"]}}],"default":"auto","description":"auto: a connection with an end on a fine-pitch pad (0.65 mm or closer) is routed no wider than that pad's narrow side (not below the Default class width or the board minimum) when its class is wider; the rest of its net at its class width; the row says necked_from_mm. Per net: {\"VBUS\": \"off\", \"*\": \"auto\"}"},
                        "escapes":{"type":"boolean","default":true,"description":"the way out of each fine-pitch pad (0.65 mm or closer) on the edge of its package is kept open while its connection waits: room for a track and a via in front of it; a route of another net across it is refused (reason names the pad). false: no such room kept"},
                        "plan":{"type":"boolean","default":true,"description":"a connection the router does not find on a layer is planned over the free space (grid search with vias, the board's clearances), the router follows the plan; no plan found: no escape-spot tries either (row plan tells)"},
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
                   "dangling_vias_removed, not_reached (pads / islands left without a via, and why), islands_left (pieces of the "
                   "net's pours with no via: layer, box, pads on them) and the "
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
                        "region_mode":{"type":"string","enum":["inside","touching","cut"],"default":"inside","description":"inside: wholly in the region; touching: any part in it (the copper, not its box); cut (pcb_tracks_delete): straight tracks crossing the edge lose only their part inside, the rest stays (answer: cut)"},
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

static KOPENAPI_RESULT h_pcb_route_strategies( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    GRID_PARAMS params;
    params.pfStart = (float) std::clamp( aArgs.value( "pf_start", 1.0 ), 0.01, 100.0 );
    params.pfGrowth = (float) std::clamp( aArgs.value( "pf_growth", 1.6 ), 1.01, 10.0 );
    params.astar = (float) std::clamp( aArgs.value( "astar", 1.2 ), 1.0, 3.0 );
    params.recent = std::clamp( aArgs.value( "reroute_recent", 2 ), 0, 100 );

    nlohmann::json rows = nlohmann::json::array();
    const std::vector<GRID_STRATEGY> builtins = GridBuiltinStrategies( params );

    for( const GRID_STRATEGY& st : builtins )
    {
        rows.push_back( { { "name", st.Name() }, { "order", st.order }, { "directions", st.directions },
                          { "pf_start", st.pfStart }, { "pf_growth", st.pfGrowth }, { "astar", st.astar },
                          { "schedule", st.schedule == 0 ? "fixed" : st.schedule == 1 ? "adaptive" : "adaptive_hist" },
                          { "hist_decay", st.histDecay }, { "reroute_recent", st.recent } } );
    }

    const int     cores = (int) std::thread::hardware_concurrency();
    const int64_t available = KOPENAPI_MEMORY::SystemAvailable();

    return KOPENAPI_RESULT::Ok( { { "builtin", rows }, { "cores", cores },
                                  { "lanes_by_cores", std::clamp( cores / 2, 1, (int) builtins.size() ) },
                                  { "memory_available_mb", available > 0 ? available / 1048576 : -1 } } );
}


KOPENAPI_REGISTER( "pcb_route_strategies",
                   "The grid engine's built-in variants (lanes) raced by pcb_route {engine: 'grid'} when no "
                   "strategies are given, in the order lanes are dropped from the end: net order, layer "
                   "directions, first-round price and growth, A* weight, recent rip-up. pcb_route runs one "
                   "per two cores, as many as half the available memory holds; its answer gives the lane "
                   "size of that board (grid.lane_mb) and how many lanes fit (grid.lanes_memory_allows). "
                   "pf_start / pf_growth / astar / reroute_recent: as for pcb_route",
                   R"json({"type":"object","properties":{
                        "pf_start":{"type":"number","default":1.0},
                        "pf_growth":{"type":"number","default":1.6},
                        "astar":{"type":"number","default":1.2},
                        "reroute_recent":{"type":"integer","default":2}}})json"_json,
                   false, h_pcb_route_strategies );

KOPENAPI_MARK_EDITING( "pcb_route_connection", "pcb_route", "pcb_stitch", "pcb_tracks_delete", "pcb_track_add", "pcb_via_add", "pcb_fanout" );
