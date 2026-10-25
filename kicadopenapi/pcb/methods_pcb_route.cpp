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
#include "board_delta.h"
#include "variant_tree.h"

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
#include <cleanup_item.h>
#include <tracks_cleaner.h>

#include <chrono>
#include <wx/thread.h>
#include <limits>
#include <deque>
#include <map>
#include <mutex>
#include <tuple>
#include <thread>
#include <cmath>
#include <optional>
#include <set>

#include <kicadopenapi_jobs.h>
#include <model_thread.h>


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
    // a tool builds wx menus in the GUI and takes an id from an unguarded counter: owner thread only
    wxASSERT( !KopenapiInComputeWorker() );

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


/// @brief A name for copper that stays the same from run to run (a uuid of new copper does not):
/// net/kind/layer/x,y[/x,y] in mm to 0.01, a track's ends in one order
static std::string copperKey( const BOARD_ITEM* aItem )
{
    auto at = []( const VECTOR2I& p )
    {
        char text[48];
        std::snprintf( text, sizeof( text ), "%.2f,%.2f", toMm( p.x ), toMm( p.y ) );
        return std::string( text );
    };

    const BOARD_CONNECTED_ITEM* c = dynamic_cast<const BOARD_CONNECTED_ITEM*>( aItem );
    const BOARD*                board = aItem->GetBoard();
    const std::string           net = c ? str( UnescapeString( c->GetNetname() ) ) : std::string();
    auto layerName = [&]( PCB_LAYER_ID l ) { return board ? str( board->GetLayerName( l ) ) : std::to_string( (int) l ); };

    if( aItem->Type() == PCB_VIA_T )
    {
        const PCB_VIA* via = static_cast<const PCB_VIA*>( aItem );
        return net + "/via/" + layerName( via->TopLayer() ) + "-" + layerName( via->BottomLayer() ) + "/" + at( via->GetPosition() );
    }

    if( aItem->Type() == PCB_TRACE_T || aItem->Type() == PCB_ARC_T )
    {
        const PCB_TRACK* t = static_cast<const PCB_TRACK*>( aItem );
        std::string      a = at( t->GetStart() ), b = at( t->GetEnd() );

        if( std::tie( t->GetEnd().x, t->GetEnd().y ) < std::tie( t->GetStart().x, t->GetStart().y ) )
            std::swap( a, b );

        return net + "/" + ( aItem->Type() == PCB_ARC_T ? "arc" : "track" ) + "/" + layerName( t->GetLayer() ) + "/" + a + "/" + b;
    }

    if( aItem->Type() == PCB_ZONE_T )
    {
        const ZONE* zone = static_cast<const ZONE*>( aItem );
        const BOX2I box = zone->GetBoundingBox();
        return ( zone->GetIsRuleArea() ? "keepout/" + str( zone->GetZoneName() ) : net + "/zone" ) + "/" + at( box.GetOrigin() ) + "/"
               + at( box.GetEnd() );
    }

    return net + "/item/" + at( aItem->GetPosition() );
}


/// @brief Gross errors in a staged route: new or moved copper (aChanged) within the clearance of copper
/// of another net - pads, tracks, vias, filled zones - or inside a keep-out that bars it; removed
/// items do not count.  The clearance is the board's rule for the pair (DRC engine: custom rules,
/// net classes, board minimum), as DRC and the router see it.  aZonesYield: the caller refills the zones afterwards, so other nets' fills give
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

        if( aItem->Type() != PCB_PAD_T )
        {
            d["uuid"] = str( aItem->m_Uuid.AsString() );
            d["key"] = copperKey( aItem );
        }

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

    // keep-outs with their outlines on the board (a footprint's zone keeps its own in the
    // footprint's coordinates)
    std::vector<std::pair<ZONE*, SHAPE_POLY_SET>> ruleAreas;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone->GetIsRuleArea() )
            ruleAreas.emplace_back( zone, zone->GetBoardOutline() );
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( ZONE* zone : fp->Zones() )
        {
            if( zone->GetIsRuleArea() )
                ruleAreas.emplace_back( zone, zone->GetBoardOutline() );
        }
    }

    for( BOARD_CONNECTED_ITEM* item : aChanged )
    {
        BOX2I reach = item->GetBoundingBox();
        reach.Inflate( widest );
        std::set<ZONE*> barredBy;

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

            for( const auto& [zone, outline] : ruleAreas )
            {
                if( barredBy.count( zone ) || !zone->IsOnLayer( layer ) || !reach.Intersects( zone->GetBoundingBox() ) )
                {
                    continue;
                }

                const bool barred = item->Type() == PCB_VIA_T   ? zone->GetDoNotAllowVias()
                                    : item->Type() == PCB_PAD_T ? zone->GetDoNotAllowPads()
                                                                : zone->GetDoNotAllowTracks();

                if( barred && outline.Collide( shape.get(), 0 ) )
                {
                    barredBy.insert( zone );
                    nlohmann::json keepout = { { "keepout", str( zone->GetZoneName() ) }, { "uuid", str( zone->m_Uuid.AsString() ) },
                                                { "key", copperKey( zone ) } };

                    if( FOOTPRINT* owner = zone->GetParentFootprint() )
                        keepout["footprint"] = str( owner->GetReference() );

                    violations.push_back( { { "layer", str( aBoard->GetLayerName( layer ) ) },
                                            { "new", describe( item ) },
                                            { "other", keepout } } );
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

    static void Brighten( FRAME* aFrame, EDA_ITEM* aItem, bool aOn ) { KopenapiPcbBrighten( aFrame, aItem, aOn ); }

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

    /// @brief The track width of this route instead of the net class's (> 0)
    int width = 0;

    /// @brief Where on `from` the route starts (a track's end); its position when not given
    std::optional<VECTOR2I> fromPos;

    /// @brief No item at the target: the route ends at toPos on this layer (a free spot)
    std::optional<PCB_LAYER_ID> toFreeLayer;
    bool                        noDoubleBack = false;   // a forced corner: refuse a line that runs back over itself
    std::optional<PCB_LAYER_ID> endViaLayer;   // a free spot on another layer: the route ends in a via there

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
    int    viaSpotsOffBoard = 0;  ///< via spots left out: on or past the board's edge
    int    ripUpsSkipped = 0;     ///< rip-ups not repeated: nothing changed since the same one failed
    int    ripUpsWalled = 0;      ///< rip-ups not made: no plan even through the routes of this call
    int    ripUpsPlanned = 0;     ///< rip-ups whose routes to take out a plan chose (the rest: the nearest in the corridor)
    std::map<std::string, int> ripUpsFailedAt;   ///< failed rip-ups by what failed: the route, a victim, a cut one
    int    ripUpsSecond = 0;      ///< victims with no way back that took out routes in turn
    int    pairsStuck = 0;        ///< pair tries ended early: nothing placed after the first steps
    int    padExitsTried = 0;     ///< starts stuck in their pad that tried a corner out of a free side
    int    padExitsWon = 0;       ///< ... routed that way
    int    plans = 0;             ///< global plans made (PlanRoute)
    int    plansFound = 0;        ///< ... that found a way
    int    plansWholeBoard = 0;   ///< ... searched again over the whole board (the box round the ends cut the way)
    double planMs = 0;
    double syncMs = 0;    ///< the router's world built from the board
    double moveMs = 0;    ///< the router's path search (every Move of the head)
    double totalMs = 0;   ///< whole attempts (world, routing, guard, commit)
    int64_t clearanceLookups = 0;   ///< the router's clearances looked up past its caches
    int64_t ruleEvaluations = 0;    ///< ... and the design rule evaluations they took

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

    /// @brief Who laid a piece of copper: the pass, the job (>= 0) or gap (-2 - index), the attempt
    struct LAID
    {
        std::string pass;
        long        job = -1;
        int         attempt = 0;
    };

    std::map<KIID, LAID> laid;
    long                 job = -1;   ///< the job or gap the routes are laid for now
};

/// @brief The profile of the pcb_route call running on a board; a scratch one outside such a call
class ROUTE_PROFILES
{
public:
    class BIND
    {
    public:
        BIND( const BOARD* aBoard, ROUTE_PROFILE& aProfile ) : m_board( aBoard )
        {
            std::lock_guard<std::mutex> lock( mutex() );
            map()[aBoard] = &aProfile;
        }

        ~BIND()
        {
            std::lock_guard<std::mutex> lock( mutex() );
            map().erase( m_board );
        }

    private:
        const BOARD* m_board;
    };

    static ROUTE_PROFILE& Of( const BOARD* aBoard, ROUTE_PROFILE& aScratch )
    {
        std::lock_guard<std::mutex> lock( mutex() );
        auto                        it = map().find( aBoard );
        return it != map().end() ? *it->second : aScratch;
    }

private:
    static std::mutex& mutex()
    {
        static std::mutex m;
        return m;
    }

    static std::map<const BOARD*, ROUTE_PROFILE*>& map()
    {
        static std::map<const BOARD*, ROUTE_PROFILE*> m;
        return m;
    }
};


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


/// @brief The route's tracks at the width the rules give where each piece lies, as DRC sees it:
/// split where they cross a courtyard, a piece under a custom track_width rule at its opt (else its
/// min, not below the board minimum), else at aBase, or at the neck width inside the courtyards of
/// the route's ends.  A piece made wider than routed stays as routed when that would break a
/// clearance (guard).  Answers widths_mm when more than one, narrow_kept_mm
static void resolveRouteWidths( KOPENAPI_PNS_IFACE& aIface, BOARD* aBoard, const ROUTE_REQUEST& aReq, int aBase,
                                nlohmann::json& aOut )
{
    std::shared_ptr<DRC_ENGINE> drc = aBoard->GetDesignSettings().m_DRCEngine;
    const int                   netCode = aReq.from->GetNetCode();
    const int                   boardMin = aBoard->GetDesignSettings().m_TrackMinWidth;

    if( !drc || ( !drc->HasGeometryDependentRules() && aReq.neckWidth <= 0 ) )
        return;

    std::vector<PCB_TRACK*> mine;
    BOX2I                   box;

    for( BOARD_ITEM* item : aIface.m_added )
    {
        if( item->Type() == PCB_TRACE_T && static_cast<PCB_TRACK*>( item )->GetNetCode() == netCode )
        {
            mine.push_back( static_cast<PCB_TRACK*>( item ) );
            box.Merge( item->GetBoundingBox() );
        }
    }

    if( mine.empty() )
        return;

    std::vector<SHAPE_POLY_SET> courtyards;
    std::vector<SHAPE_POLY_SET> neckAreas;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        const bool atEnd = aReq.neckWidth > 0
                           && ( fp == aReq.from->GetParentFootprint() || ( aReq.to && fp == aReq.to->GetParentFootprint() ) );

        for( PCB_LAYER_ID side : { F_CrtYd, B_CrtYd } )
        {
            const SHAPE_POLY_SET& c = fp->GetCourtyard( side );

            if( c.OutlineCount() == 0 || !c.BBox().Intersects( box ) )
                continue;

            courtyards.push_back( c );

            if( atEnd )
                neckAreas.push_back( c );
        }
    }

    std::vector<PCB_TRACK*> pieces;

    for( PCB_TRACK* track : mine )
    {
        const SEG             whole( track->GetStart(), track->GetEnd() );
        std::vector<VECTOR2I> cuts = { whole.A, whole.B };

        for( const SHAPE_POLY_SET& c : courtyards )
        {
            for( int o = 0; o < c.OutlineCount(); ++o )
            {
                SHAPE_LINE_CHAIN::INTERSECTIONS hits;
                c.COutline( o ).Intersect( whole, hits );

                for( const SHAPE_LINE_CHAIN::INTERSECTION& hit : hits )
                    cuts.push_back( hit.p );
            }
        }

        std::sort( cuts.begin(), cuts.end(),
                   [&]( const VECTOR2I& a, const VECTOR2I& b )
                   { return ( a - whole.A ).SquaredEuclideanNorm() < ( b - whole.A ).SquaredEuclideanNorm(); } );
        cuts.erase( std::unique( cuts.begin(), cuts.end() ), cuts.end() );

        track->SetEnd( cuts[1] );
        pieces.push_back( track );

        for( size_t c = 2; c < cuts.size(); ++c )
        {
            PCB_TRACK* piece = static_cast<PCB_TRACK*>( track->Duplicate( false ) );
            piece->SetStart( cuts[c - 1] );
            piece->SetEnd( cuts[c] );
            aIface.Staged().Add( piece );
            aIface.m_added.push_back( piece );
            pieces.push_back( piece );
        }
    }

    std::vector<BOARD_CONNECTED_ITEM*> staged;

    for( BOARD_ITEM* item : aIface.m_added )
    {
        if( BOARD_CONNECTED_ITEM* c = dynamic_cast<BOARD_CONNECTED_ITEM*>( item ) )
            staged.push_back( c );
    }

    int64_t       narrowKept = 0;
    std::set<int> widths;

    for( PCB_TRACK* piece : pieces )
    {
        const int      routed = piece->GetWidth();
        const VECTOR2I mid = ( piece->GetStart() + piece->GetEnd() ) / 2;
        int            want = aBase;

        // the rule that holds on the piece itself: a point probe at its middle (the piece's round
        // ends touch the courtyard outline it was cut at)
        PCB_TRACK probe( aBoard );
        probe.SetFlags( ROUTER_TRANSIENT );
        probe.SetNet( piece->GetNet() );
        probe.SetLayer( piece->GetLayer() );
        probe.SetWidth( 1 );
        probe.SetStart( mid );
        probe.SetEnd( mid );

        const DRC_CONSTRAINT c = drc->EvalRules( TRACK_WIDTH_CONSTRAINT, &probe, nullptr, piece->GetLayer() );

        if( c.GetParentRule() && !c.GetParentRule()->IsImplicit() && ( c.GetValue().HasOpt() || c.GetValue().HasMin() ) )
        {
            want = std::max( c.GetValue().HasOpt() ? c.GetValue().Opt() : c.GetValue().Min(), boardMin );
        }
        else if( aReq.neckWidth > 0 )
        {
            for( const SHAPE_POLY_SET& area : neckAreas )
            {
                if( area.Contains( mid ) )
                {
                    want = std::min( want, aReq.neckWidth );
                    break;
                }
            }
        }

        if( want > 0 && want != routed )
        {
            piece->SetWidth( want );

            if( want > routed && !guard( aBoard, { piece }, aIface.m_removed, false, &staged ).empty() )
            {
                piece->SetWidth( routed );
                narrowKept += piece->GetLength();
            }
        }

        widths.insert( piece->GetWidth() );
    }

    if( widths.size() > 1 )
    {
        aOut["widths_mm"] = nlohmann::json::array();

        for( int w : widths )
            aOut["widths_mm"].push_back( toMm( w ) );
    }

    if( narrowKept > 0 )
        aOut["narrow_kept_mm"] = toMm( narrowKept );
}


/// @brief New straight segments laid along copper of their net that is there already (as wide,
/// same layer) lose the covered part: PNS links a new segment to an equal one only, so a line that
/// leaves a pad along an earlier route doubles it (and a cut at a courtyard edge makes an exact
/// twin).  A segment covered whole is not added; a covered head or tail is cut back to the end of
/// the copper there (the two meet); copper in the middle stays doubled.  A shove's unchanged copy
/// of a segment is covered whole too.
static void trimOverOwnCopper( KOPENAPI_PNS_IFACE& aIface, BOARD* aBoard )
{
    for( auto it = aIface.m_added.begin(); it != aIface.m_added.end(); )
    {
        if( ( *it )->Type() != PCB_TRACE_T )
        {
            ++it;
            continue;
        }

        PCB_TRACK* fresh = static_cast<PCB_TRACK*>( *it );
        bool       covered = false;

        for( bool cut = true; cut && !covered; )
        {
            cut = false;
            const VECTOR2I a = fresh->GetStart();
            const VECTOR2I d = fresh->GetEnd() - a;
            const double   len = d.EuclideanNorm();

            if( len <= 0 )
            {
                covered = true;
                break;
            }

            const SEG line( a, fresh->GetEnd() );

            for( PCB_TRACK* other : aBoard->Tracks() )
            {
                if( other->Type() != PCB_TRACE_T || aIface.m_removed.count( other ) || other->GetNetCode() != fresh->GetNetCode()
                    || other->GetLayer() != fresh->GetLayer() || other->GetWidth() < fresh->GetWidth()
                    || line.LineDistance( other->GetStart() ) > 1 || line.LineDistance( other->GetEnd() ) > 1 )
                {
                    continue;
                }

                auto t = [&]( const VECTOR2I& p ) { return double( ( p - a ).Dot( d ) ) / len; };
                const bool     startFirst = t( other->GetStart() ) <= t( other->GetEnd() );
                const VECTOR2I lo = startFirst ? other->GetStart() : other->GetEnd();
                const VECTOR2I hi = startFirst ? other->GetEnd() : other->GetStart();

                if( t( hi ) <= 1 || t( lo ) >= len - 1 )
                    continue;

                if( t( lo ) <= 1 && t( hi ) >= len - 1 )
                {
                    covered = true;
                    break;
                }

                if( t( lo ) <= 1 )
                {
                    fresh->SetStart( hi );
                    cut = true;
                    break;
                }

                if( t( hi ) >= len - 1 )
                {
                    fresh->SetEnd( lo );
                    cut = true;
                    break;
                }
            }
        }

        if( covered )
        {
            aIface.Staged().Unstage( fresh, nullptr );
            it = aIface.m_added.erase( it );
        }
        else
        {
            ++it;
        }
    }
}


bool routeOneImpl( PCB_CONTEXT& aContext, const ROUTE_REQUEST& aReq, nlohmann::json& aOut, std::vector<KIID>& aAdded )
{
    BOARD*         board = aContext.GetBoard();
    ROUTE_PROFILE  scratch;
    ROUTE_PROFILE& profile = ROUTE_PROFILES::Of( board, scratch );
    const auto started = std::chrono::steady_clock::now();

    auto iface = std::make_unique<KOPENAPI_PNS_IFACE>();
    iface->SetBoard( board );
    // no SetView: without a view the interface draws no previews (SetView( nullptr ) would
    // create a preview group that EraseView then updates on a null view)
    iface->SetHostTool( routeHost( aContext.GetToolManager() ) );

    // The search only reads the board: in the GUI it runs on a worker while the main thread paints
    // and navigates, and readers read alongside until the route is committed
    PNS::ROUTING_SETTINGS                   settings( nullptr, "" );
    std::vector<std::unique_ptr<PCB_SHAPE>> frontItems;   // the ways out kept open, as the router sees them
    std::optional<PNS::ROUTER::THREAD_SCOPE> commitScope;   // outlives the router: current while it is
    std::unique_ptr<PNS::ROUTER>            router;
    PNS::PLACEMENT_ALGO*                    placer = nullptr;
    std::chrono::steady_clock::time_point   synced;
    VECTOR2I                                start;
    VECTOR2I                                target;
    PNS_LAYER_RANGE                         targetLayers;
    bool                                    reached = false;
    bool                                    fixed = false;

    auto search = [&]() -> std::optional<bool>
    {
        settings.SetMode( aReq.mode );

        // The same result on every run: the router's wall-clock limits (made for a hand on the mouse) are
        // lifted for the call, its iteration limits bound the work (a shove through a dense row can take
        // 50 ms an iteration: the editor's 250 would run past any clock)
        settings.SetShoveTimeLimit( 60000 );
        settings.SetShoveIterationLimit( 100 );
        settings.SetClusterTimeout( 10000 );

        // our router is current on this thread only: the editor's router stays current for the editor
        PNS::ROUTER::THREAD_SCOPE routerScope;

        router = std::make_unique<PNS::ROUTER>();
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

        synced = std::chrono::steady_clock::now();
        profile.syncMs += msSince( started, synced );
        router->SetMode( aReq.diffPair ? PNS::PNS_MODE_ROUTE_DIFF_PAIR : PNS::PNS_MODE_ROUTE_SINGLE );

        PNS::ITEM* startItem = router->GetWorld()->FindItemByParent( aReq.from );

        if( !startItem )
        {
            aOut["routed"] = false;
            aOut["reason"] = "the router does not know the start pad";
            return false;
        }

        start = aReq.fromPos.value_or( aReq.from->GetPosition() );
        PNS::SIZES_SETTINGS sizes( router->Sizes() );
        iface->SetStartLayerFromPCBNew( aReq.layer );
        iface->ImportSizes( sizes, startItem, nullptr, start );
        sizes.AddLayerPair( iface->GetPNSLayerFromBoardLayer( F_Cu ), iface->GetPNSLayerFromBoardLayer( B_Cu ) );

        if( aReq.viaDiameter > 0 )
            sizes.SetViaDiameter( aReq.viaDiameter );

        if( aReq.viaDrill > 0 )
            sizes.SetViaDrill( aReq.viaDrill );

        if( aReq.width > 0 )
            sizes.SetTrackWidth( aReq.width );

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
            profile.moveMs += msSince( from );
        };

        // Vias on the way: route to the point, drop a through via there, continue on the next layer
        placer = router->Placer();

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

        PNS::ITEM* targetItem = nullptr;

        if( aReq.to )
        {
            target = aReq.toPos;
            targetItem = router->GetWorld()->FindItemByParent( aReq.to );

            if( targetItem )
                targetLayers = targetItem->Layers();
        }
        else if( aReq.toFreeLayer )
        {
            target = aReq.toPos;
            const int l = iface->GetPNSLayerFromBoardLayer( *aReq.toFreeLayer );
            targetLayers = PNS_LAYER_RANGE( l, l );
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
                profile.pairsStuck++;
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
        reached = aReq.diffPair || ( placer->CurrentEnd() == target && targetLayers.Overlaps( router->GetCurrentLayer() ) );

        if( reached && aReq.endViaLayer )
        {
            PNS::SIZES_SETTINGS viaSizes( router->Sizes() );
            viaSizes.SetViaType( VIATYPE::THROUGH );
            viaSizes.ClearLayerPairs();
            viaSizes.AddLayerPair( router->GetCurrentLayer(), iface->GetPNSLayerFromBoardLayer( *aReq.endViaLayer ) );
            router->UpdateSizes( viaSizes );

            if( !router->IsPlacingVia() )
                router->ToggleViaPlacement();

            move( target, nullptr );
            move( target, nullptr );
            reached = placer->CurrentEnd() == target;
        }

        // Fix the line in the router, then commit it to the board (as the router tool does); our
        // interface stages the commit instead of pushing it
        // a free spot has no item to end on: the line is finished there, as the editor's End does
        fixed = reached && router->FixRoute( target, targetItem, !aReq.to && aReq.toFreeLayer.has_value(), false );

        return std::nullopt;
    };

    std::optional<bool> early;
    KopenapiCompute( [&]() { early = search(); } );

    profile.clearanceLookups += iface->ClearanceLookups();
    profile.ruleEvaluations += iface->RuleEvaluations();

    if( early )
        return *early;

    // the commit runs on this thread: our router is current here for it
    commitScope.emplace();
    PNS::ROUTER::SetInstance( router.get() );

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

    if( aReq.noDoubleBack )
    {
        std::vector<PCB_TRACK*> laid;

        for( BOARD_ITEM* item : iface->m_added )
        {
            if( item->Type() == PCB_TRACE_T )
                laid.push_back( static_cast<PCB_TRACK*>( item ) );
        }

        for( size_t i = 0; i < laid.size(); ++i )
        {
            for( size_t j = i + 1; j < laid.size(); ++j )
            {
                if( laid[i]->GetLayer() != laid[j]->GetLayer() )
                    continue;

                const SEG     a( laid[i]->GetStart(), laid[i]->GetEnd() );
                const SEG     b( laid[j]->GetStart(), laid[j]->GetEnd() );
                const VECTOR2I d = a.B - a.A;
                const double  len = d.EuclideanNorm();

                if( len <= 0 || a.LineDistance( b.A ) > 1 || a.LineDistance( b.B ) > 1 )
                    continue;

                auto t = [&]( const VECTOR2I& p ) { return double( ( p - a.A ).Dot( d ) ) / len; };
                const double lo = std::max( 0.0, std::min( t( b.A ), t( b.B ) ) );
                const double hi = std::min( len, std::max( t( b.A ), t( b.B ) ) );

                if( hi - lo > laid[i]->GetWidth() / 2 )
                {
                    iface->Staged().Revert();
                    aOut["routed"] = false;
                    aOut["reason"] = "the way by the corner runs back over itself";
                    return false;
                }
            }
        }
    }

    if( !aReq.diffPair )
    {
        NETCLASS* nc = aReq.from->GetEffectiveNetClass();
        const int base = aReq.width > 0 ? aReq.width
                         : nc && nc->HasTrackWidth() ? nc->GetTrackWidth()
                                                     : board->GetDesignSettings().GetCurrentTrackWidth();
        resolveRouteWidths( *iface, board, aReq, base, aOut );
    }

    trimOverOwnCopper( *iface, board );

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
            profile.shovedNets.insert( net );
    }

    for( const auto* items : { &iface->m_modified, &iface->m_removed } )
    {
        for( BOARD_ITEM* item : *items )
        {
            if( int net = otherNet( item ) )
                profile.shovedNets.insert( net );
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
    const int      nearby = pcbIUScale.mmToIU( 1.0 );
    const VECTOR2I dir = aAim - aAt;
    const double   len = dir.EuclideanNorm();
    const int      look = pcbIUScale.mmToIU( 5.0 );
    const VECTOR2I end = len > look ? aAt + VECTOR2I( KiROUND( (double) dir.x * look / len ), KiROUND( (double) dir.y * look / len ) )
                                    : aAim;
    const SHAPE_SEGMENT path( aAt, end, 2 * aHalfWidth );
    const SHAPE_CIRCLE  probe( aAt, 0 );

    BOX2I area( aAt, VECTOR2I( 0, 0 ) );
    area.Merge( end );
    area.Inflate( nearby + aHalfWidth );

    auto consider = [&]( BOARD_CONNECTED_ITEM* aItem, nlohmann::json aWhat )
    {
        if( aItem->GetNetCode() == aNetCode || !aItem->IsOnLayer( aLayer ) || !area.Intersects( aItem->GetBoundingBox() ) )
            return;

        std::shared_ptr<SHAPE> shape = aItem->GetEffectiveShape( aLayer );
        int                    actual = 0;

        if( shape->Collide( &path, 0 ) || shape->Collide( &probe, nearby ) )
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
        consider( track, { { "type", track->Type() == PCB_VIA_T ? "via" : "track" }, { "uuid", str( track->m_Uuid.AsString() ) }, { "key", copperKey( track ) } } );

    std::stable_sort( found.begin(), found.end(), []( const auto& x, const auto& y ) { return x.first < y.first; } );

    nlohmann::json out = nlohmann::json::array();

    for( size_t k = 0; k < found.size() && k < 3; ++k )
        out.push_back( found[k].second );

    return out;
}


/// @brief A grid end no search reached without a wall round it: the other nets' copper next to it on
/// its routing layers (blockersAt toward the other end), each with the end and layer, at most four
static nlohmann::json gridEndBlockers( BOARD* aBoard, BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aAim,
                                       const std::vector<PCB_LAYER_ID>& aLayers, int aNetCode, int aHalfWidth,
                                       const std::string& aEndName )
{
    nlohmann::json        out = nlohmann::json::array();
    std::set<std::string> named;

    if( !aEnd )
        return out;

    for( PCB_LAYER_ID layer : aLayers )
    {
        if( !aEnd->IsOnLayer( layer ) )
            continue;

        for( nlohmann::json b : blockersAt( aBoard, aEnd->GetPosition(), aAim, layer, aNetCode, aHalfWidth ) )
        {
            if( out.size() >= 4 || !named.insert( b.value( "uuid", b.value( "pad", std::string() ) ) ).second )
                continue;

            b["end"] = aEndName;
            b["layer"] = str( aBoard->GetLayerName( layer ) );
            out.push_back( b );
        }
    }

    return out;
}


static std::string mmText( double aMm )
{
    char text[32];
    std::snprintf( text, sizeof( text ), "%g", std::round( aMm * 1000.0 ) / 1000.0 );
    return text;
}


/// @brief A route end by name: REF.PAD for a pad, else its kind and where it is
static std::string endName( BOARD_CONNECTED_ITEM* aItem )
{
    if( !aItem )
        return "?";

    if( aItem->Type() == PCB_PAD_T )
    {
        PAD* pad = static_cast<PAD*>( aItem );
        return str( pad->GetParentFootprint() ? pad->GetParentFootprint()->GetReference() : wxString( "?" ) ) + "." + str( pad->GetNumber() );
    }

    const VECTOR2I at = aItem->GetPosition();
    const std::string kind = aItem->Type() == PCB_VIA_T ? "via" : aItem->Type() == PCB_ZONE_T ? "pour" : "track";
    return kind + " at (" + mmText( toMm( at.x ) ) + ", " + mmText( toMm( at.y ) ) + ")";
}


/// @brief The board item behind a blocked grid cell: the nearest copper of aOwner's net (aOwner -1:
/// of any net but aNet, or any pad on the via plane, a hole, a keep-out area, the board edge) to
/// aAt; aLayer UNDEFINED_LAYER for the via plane
nlohmann::json wallItem( BOARD* aBoard, int aOwner, const VECTOR2I& aAt, PCB_LAYER_ID aLayer, int aNet )
{
    const SHAPE_CIRCLE probe( aAt, 0 );
    const int          reach = pcbIUScale.mmToIU( 1.5 );
    int                best = std::numeric_limits<int>::max();
    nlohmann::json     out;

    auto consider = [&]( BOARD_ITEM* aItem, const std::shared_ptr<SHAPE>& aShape, nlohmann::json aWhat )
    {
        int actual = 0;

        if( !aShape || !aShape->Collide( &probe, reach, &actual ) || actual >= best )
            return;

        best = actual;
        aWhat["distance_mm"] = toMm( actual );
        out = aWhat;
    };

    // no owner on a track plane: whatever copper is not the routed net's
    auto netOk = [&]( BOARD_CONNECTED_ITEM* aItem ) { return aOwner > 0 ? aItem->GetNetCode() == aOwner : aItem->GetNetCode() != aNet; };
    BOX2I area( aAt, VECTOR2I( 0, 0 ) );
    area.Inflate( reach );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            const bool any = aOwner < 0 && aLayer == UNDEFINED_LAYER;   // no via in a pad, whoever's

            if( !area.Intersects( pad->GetBoundingBox() ) || !( any || netOk( pad ) ) )
                continue;

            const PCB_LAYER_ID layer = aLayer == UNDEFINED_LAYER ? pad->GetPrincipalLayer() : aLayer;

            if( pad->IsOnLayer( layer ) )
            {
                consider( pad, pad->GetEffectiveShape( layer ),
                          { { "pad", str( fp->GetReference() ) + "." + str( pad->GetNumber() ) },
                            { "net", str( UnescapeString( pad->GetNetname() ) ) } } );
            }

            if( pad->HasHole() && aOwner < 0 )
                consider( pad, pad->GetEffectiveHoleShape(), { { "hole", str( fp->GetReference() ) + "." + str( pad->GetNumber() ) } } );
        }
    }

    for( PCB_TRACK* t : aBoard->Tracks() )
    {
        if( !area.Intersects( t->GetBoundingBox() ) || !netOk( t ) )
            continue;

        const PCB_LAYER_ID layer = aLayer == UNDEFINED_LAYER ? ( t->IsOnLayer( F_Cu ) ? F_Cu : t->GetLayer() ) : aLayer;

        if( t->IsOnLayer( layer ) )
        {
            nlohmann::json what = { { "type", t->Type() == PCB_VIA_T ? "via" : "track" },
                                    { "net", str( UnescapeString( t->GetNetname() ) ) },
                                    { "uuid", str( t->m_Uuid.AsString() ) }, { "key", copperKey( t ) } };

            if( t->Type() == PCB_VIA_T )
                what["at_mm"] = { toMm( t->GetPosition().x ), toMm( t->GetPosition().y ) };
            else
                what["from_mm"] = { toMm( t->GetStart().x ), toMm( t->GetStart().y ) },
                what["to_mm"] = { toMm( t->GetEnd().x ), toMm( t->GetEnd().y ) },
                what["width_mm"] = toMm( t->GetWidth() );

            consider( t, t->GetEffectiveShape( layer ), what );
        }
    }

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone->GetIsRuleArea() )
        {
            if( aOwner < 0 && zone->GetBoardOutline().Contains( aAt, -1, reach ) )
                consider( zone, std::make_shared<SHAPE_POLY_SET>( zone->GetBoardOutline() ),
                          { { "keep_out", str( zone->GetZoneName() ) } } );

            continue;
        }

        if( aLayer == UNDEFINED_LAYER || !zone->IsOnLayer( aLayer ) || !netOk( zone ) )
            continue;

        if( std::shared_ptr<SHAPE_POLY_SET> fill = zone->GetFilledPolysList( aLayer ); fill && !fill->IsEmpty() )
            consider( zone, std::make_shared<SHAPE_POLY_SET>( *fill ),
                      { { "zone", str( zone->GetZoneName() ) }, { "net", str( UnescapeString( zone->GetNetname() ) ) } } );
    }

    if( out.is_null() && aOwner < 0 )
    {
        for( BOARD_ITEM* item : aBoard->Drawings() )
        {
            if( item->IsOnLayer( Edge_Cuts ) )
                consider( item, item->GetEffectiveShape( Edge_Cuts ), { { "edge", true } } );
        }
    }

    if( out.is_null() && aOwner > 0 )
    {
        if( NETINFO_ITEM* net = aBoard->FindNet( aOwner ) )
            out = { { "net", str( UnescapeString( net->GetNetname() ) ) } };
    }

    return out;
}


/// @brief What closes a walled-in end: the copper of other nets that each closed cell round its room
/// lies within aReach of (the track's half width plus the clearance plus a cell), most cells first
nlohmann::json wallClosers( BOARD* aBoard, const GRID_WALL& aWall, const std::vector<PCB_LAYER_ID>& aLayers, int aNet,
                            int aReach )
{
    BOX2I area;
    bool  any = false;

    for( const auto& [at, l] : aWall.closed )
    {
        if( l < 0 )
            continue;

        area = any ? area.Merge( BOX2I( at, VECTOR2I( 0, 0 ) ) ) : BOX2I( at, VECTOR2I( 0, 0 ) );
        any = true;
    }

    nlohmann::json out = nlohmann::json::array();

    if( !any )
        return out;

    area.Inflate( aReach );

    struct CAND
    {
        BOARD_ITEM*    item;
        nlohmann::json what;
        int            cells = 0;
        int            nearest = std::numeric_limits<int>::max();
    };

    std::vector<CAND> cands;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() != aNet && area.Intersects( pad->GetBoundingBox() ) )
                cands.push_back( { pad, { { "pad", str( fp->GetReference() ) + "." + str( pad->GetNumber() ) },
                                          { "net", str( UnescapeString( pad->GetNetname() ) ) } } } );
        }
    }

    for( PCB_TRACK* t : aBoard->Tracks() )
    {
        if( t->GetNetCode() != aNet && area.Intersects( t->GetBoundingBox() ) )
            cands.push_back( { t, { { "type", t->Type() == PCB_VIA_T ? "via" : "track" },
                                    { "net", str( UnescapeString( t->GetNetname() ) ) },
                                    { "uuid", str( t->m_Uuid.AsString() ) }, { "key", copperKey( t ) } } } );
    }

    for( ZONE* zone : aBoard->Zones() )
    {
        if( !zone->GetIsRuleArea() && zone->GetNetCode() != aNet && area.Intersects( zone->GetBoundingBox() ) )
            cands.push_back( { zone, { { "zone", str( zone->GetZoneName() ) }, { "net", str( UnescapeString( zone->GetNetname() ) ) } } } );
    }

    for( const auto& [at, l] : aWall.closed )
    {
        if( l < 0 || l >= (int) aLayers.size() )
            continue;

        const SHAPE_CIRCLE probe( at, 0 );

        for( CAND& c : cands )
        {
            if( !c.item->IsOnLayer( aLayers[l] ) )
                continue;

            std::shared_ptr<SHAPE> shape;

            if( c.item->Type() == PCB_ZONE_T )
            {
                std::shared_ptr<SHAPE_POLY_SET> fill = static_cast<ZONE*>( c.item )->GetFilledPolysList( aLayers[l] );

                if( !fill || fill->IsEmpty() )
                    continue;

                shape = std::make_shared<SHAPE_POLY_SET>( *fill );
            }
            else
            {
                shape = c.item->GetEffectiveShape( aLayers[l] );
            }

            int actual = 0;

            if( shape && shape->Collide( &probe, aReach, &actual ) )
            {
                c.cells++;
                c.nearest = std::min( c.nearest, actual );
                c.what["layer"] = str( aBoard->GetLayerName( aLayers[l] ) );
            }
        }
    }

    std::stable_sort( cands.begin(), cands.end(), []( const CAND& a, const CAND& b ) { return a.cells > b.cells; } );

    for( const CAND& c : cands )
    {
        if( c.cells == 0 || out.size() >= 5 )
            break;

        nlohmann::json w = c.what;
        w["cells"] = c.cells;
        w["distance_mm"] = toMm( c.nearest );
        out.push_back( w );
    }

    return out;
}


/// @brief routeOneImpl, and for a route that stopped short what stands there (blocked_by)
bool routeOne( PCB_CONTEXT& aContext, const ROUTE_REQUEST& aReq, nlohmann::json& aOut, std::vector<KIID>& aAdded )
{
    const auto   started = std::chrono::steady_clock::now();
    const size_t before = aAdded.size();
    const bool   ok = routeOneImpl( aContext, aReq, aOut, aAdded );
    ROUTE_PROFILE  scratch;
    ROUTE_PROFILE& profile = ROUTE_PROFILES::Of( aContext.GetBoard(), scratch );

    const double ms = msSince( started );
    profile.attempts++;

    for( size_t i = before; ok && !aReq.dryRun && i < aAdded.size(); ++i )
        profile.laid[aAdded[i]] = { profile.pass, profile.job, profile.attempts };
    profile.totalMs += ms;

    const size_t      viaCount = std::count_if( aReq.vias.begin(), aReq.vias.end(), []( const auto& v ) { return v.second.has_value(); } );
    const std::string kind = aReq.planned ? "planned"
                             : viaCount >= 2 ? "two_vias"
                             : viaCount == 1 ? "one_via"
                             : aReq.mode == PNS::RM_Shove ? "shove" : "walkaround";
    for( ROUTE_PROFILE::TALLY* t : { &profile.byKind[kind], &profile.byPass[profile.pass] } )
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

        ROUTE_PROFILE::TALLY& f = profile.byFailure[reason];
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

    BOARD* board = context->GetBoard();

    // a spot: [x_mm, y_mm] or {at: [x_mm, y_mm], layer}
    auto spotOf = [&]( const nlohmann::json& aSpot, VECTOR2I& aAt, std::optional<PCB_LAYER_ID>& aLayer ) -> std::string
    {
        const nlohmann::json at = aSpot.is_object() ? aSpot.value( "at", nlohmann::json() ) : aSpot;

        if( !at.is_array() || at.size() != 2 || !at[0].is_number() || !at[1].is_number() )
            return "a spot is [x_mm, y_mm] or {at: [x_mm, y_mm], layer: \"F.Cu\"}";

        aAt = VECTOR2I( pcbIUScale.mmToIU( at[0].get<double>() ), pcbIUScale.mmToIU( at[1].get<double>() ) );

        if( aSpot.is_object() && aSpot.contains( "layer" ) )
        {
            aLayer = aSpot["layer"].is_string() ? parseLayer( board, aSpot["layer"].get<std::string>() ) : std::nullopt;

            if( !aLayer )
                return "layer: an enabled copper layer";
        }

        return std::string();
    };

    // copper of a net at a spot: a track's or arc's end there, a via there, a pad over it
    auto copperAt = [&]( const VECTOR2I& aAt, std::optional<PCB_LAYER_ID> aLayer, int aNet ) -> BOARD_CONNECTED_ITEM*
    {
        const int tol = pcbIUScale.mmToIU( 0.005 );

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( ( aNet > 0 && t->GetNetCode() != aNet ) || ( aLayer && !t->IsOnLayer( *aLayer ) ) )
                continue;

            if( t->Type() == PCB_VIA_T ? ( t->GetPosition() - aAt ).EuclideanNorm() <= tol
                                       : ( t->GetStart() - aAt ).EuclideanNorm() <= tol || ( t->GetEnd() - aAt ).EuclideanNorm() <= tol )
            {
                return t;
            }
        }

        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
            {
                if( ( aNet > 0 && pad->GetNetCode() != aNet ) || ( aLayer && !pad->IsOnLayer( *aLayer ) ) )
                    continue;

                const PCB_LAYER_ID l = aLayer.value_or( pad->GetPrincipalLayer() );

                if( pad->IsOnLayer( l ) && pad->HitTest( aAt ) )
                    return pad;
            }
        }

        return nullptr;
    };

    ROUTE_REQUEST req;
    req.dryRun = aArgs.value( "dry_run", false );

    std::optional<PCB_LAYER_ID> fromLayer;

    if( aArgs.contains( "from" ) && !aArgs["from"].is_string() )
    {
        VECTOR2I          at;
        const std::string bad = spotOf( aArgs["from"], at, fromLayer );

        if( !bad.empty() )
            return KOPENAPI_RESULT::Error( 400, "from: REF.PAD, or " + bad );

        req.from = copperAt( at, fromLayer, 0 );

        if( !req.from || req.from->GetNetCode() <= 0 )
            return KOPENAPI_RESULT::Error( 404, "from: no copper of a net there (a track end, a via or a pad)" );

        if( req.from->Type() != PCB_PAD_T )
            req.fromPos = at;

        if( !fromLayer )
            fromLayer = req.from->Type() == PCB_PAD_T || req.from->Type() == PCB_VIA_T
                                ? ( req.from->IsOnLayer( F_Cu ) || !req.from->IsOnLayer( B_Cu ) ? F_Cu : B_Cu )
                                : static_cast<PCB_TRACK*>( req.from )->GetLayer();
    }
    else
    {
        std::optional<PAD*> from = findPad( board, aArgs.value( "from", std::string() ) );

        if( !from )
            return KOPENAPI_RESULT::Error( 404, "give 'from' as REF.PAD of an existing pad, or a spot on copper of its net "
                                                "([x_mm, y_mm] or {at, layer}: a track end, a via)" );

        req.from = *from;
    }

    std::optional<VECTOR2I> freeTarget;
    PAD*                    fromPad = req.from->Type() == PCB_PAD_T ? static_cast<PAD*>( req.from ) : nullptr;

    if( aArgs.contains( "to" ) && aArgs["to"].is_string() )
    {
        std::optional<PAD*> to = findPad( board, aArgs.value( "to", std::string() ), fromPad, fromPad );

        if( !to )
            return KOPENAPI_RESULT::Error( 404, "'to' pad not found" );

        if( ( *to )->GetNetCode() != req.from->GetNetCode() || req.from->GetNetCode() <= 0 )
            return KOPENAPI_RESULT::Error( 422, "'from' and 'to' are not on one net" );

        req.to = *to;
        req.toPos = ( *to )->GetPosition();
    }
    else if( aArgs.contains( "to" ) )
    {
        VECTOR2I                    at;
        std::optional<PCB_LAYER_ID> toLayer;
        const std::string           bad = spotOf( aArgs["to"], at, toLayer );

        if( !bad.empty() )
            return KOPENAPI_RESULT::Error( 400, "to: REF.PAD, or " + bad );

        req.toPos = at;

        // the net's own copper there: the route ends on it, else at the free spot
        if( BOARD_CONNECTED_ITEM* onto = copperAt( at, toLayer, req.from->GetNetCode() ); onto && onto != req.from )
        {
            req.to = onto;
        }
        else
        {
            req.toFreeLayer = toLayer;
            freeTarget = at;
        }
    }

    // the start's layer unless given (an SMD pad on the back starts on B.Cu)
    const std::string defaultLayer = fromLayer ? str( board->GetLayerName( *fromLayer ) )
                                     : req.from->IsOnLayer( F_Cu ) || !req.from->IsOnLayer( B_Cu ) ? "F.Cu" : "B.Cu";
    const std::string           layerName = aArgs.value( "layer", defaultLayer );
    std::optional<PCB_LAYER_ID> layer = parseLayer( board, layerName );

    if( !layer )
        return KOPENAPI_RESULT::Error( 400, "layer: an enabled copper layer, e.g. F.Cu, B.Cu" );

    if( !req.from->IsOnLayer( *layer ) )
        return KOPENAPI_RESULT::Error( 422, "'from' is not on " + layerName + " (SMD pad on the other side?)" );

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

    // a free spot: on the layer the route arrives on, or a via at the spot to the layer asked
    if( freeTarget )
    {
        PCB_LAYER_ID arrive = req.layer;

        for( const auto& [point, viaLayer] : req.vias )
        {
            if( viaLayer )
                arrive = *viaLayer;
        }

        if( req.toFreeLayer && *req.toFreeLayer != arrive )
            req.endViaLayer = req.toFreeLayer;

        req.toFreeLayer = arrive;
    }

    // a target that is not on the layer the route arrives on (an SMD pad on the other side): a via
    // in front of it, on the line from the last point, the route ends on the pad's layer
    std::optional<VECTOR2I> autoVia;

    if( req.to && req.to->Type() == PCB_PAD_T )
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

    if( aArgs.contains( "width_mm" ) )
        req.width = pcbIUScale.mmToIU( aArgs.value( "width_mm", 0.0 ) );

    {
        const BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();

        if( aArgs.contains( "width_mm" ) && req.width < std::max( 1, bds.m_TrackMinWidth ) )
            return KOPENAPI_RESULT::Error( 422, "width_mm below the board's minimum track width" );

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
        result.update( KopenapiUnroutedJson( board ) );
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
    const int             nearby = pcbIUScale.mmToIU( 0.01 );

    while( !left.empty() )
    {
        auto it = std::find_if( left.begin(), left.end(), [&]( const auto& s )
                                { return ( s.first - at ).EuclideanNorm() <= nearby || ( s.second - at ).EuclideanNorm() <= nearby; } );

        if( it == left.end() )
            break;

        at = ( it->first - at ).EuclideanNorm() <= nearby ? it->second : it->first;
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
    bool                           warm = false;      ///< over the board as it is now (edited since)
    GRID_PARAMS::WARM_HISTORY      history = GRID_PARAMS::KEEP;
    int                            removed = -1;      ///< warm: the run's tracks and vias taken out (-1: none were)
};


static KOPENAPI_RESULT pcbRoute( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, const ROUTE_BRANCH* aBranch );

// rows of pcb_route listed by default (failed ones first when cut)
constexpr size_t ROUTE_LIST_CAP = 100;


/// @brief Under a quad package's body (QFP / QFN / LGA, pads on four sides): the area inside its
/// ring of pads on its side, where a track or via of another net walls its pins in.  With an
/// exposed pad the area is the ring between the pad rows and the exposed pad (the exposed pad and
/// its clearance stay open: its own vias go there); parts with an array of pads inside (BGA) have
/// no such area
struct UNDER_PART
{
    FOOTPRINT*     fp;
    PCB_LAYER_ID   layer;
    SHAPE_POLY_SET area;
    bool           exposedPad;
};


static std::vector<UNDER_PART> underPartAreas( BOARD* aBoard )
{
    std::vector<UNDER_PART>      out;
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    std::shared_ptr<NETCLASS>    defaultClass = bds.m_NetSettings ? bds.m_NetSettings->GetDefaultNetclass() : nullptr;
    const int clearance = std::max( { bds.m_MinClearance, defaultClass && defaultClass->HasClearance() ? defaultClass->GetClearance() : 0, 0 } );

    for( FOOTPRINT* fp : aBoard->Footprints() )
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
        }

        // the pad rows: the pads on the box of the centres; their length sets how far in the ring starts
        for( PAD* pad : fp->Pads() )
        {
            const VECTOR2I c = pad->GetPosition();

            if( c.x == centres.GetLeft() || c.x == centres.GetRight() || c.y == centres.GetTop() || c.y == centres.GetBottom() )
            {
                const BOX2I pb = pad->GetBoundingBox();
                padLength = std::max( { padLength, (int) pb.GetWidth(), (int) pb.GetHeight() } );
            }
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

        // pads inside: an exposed pad (one or two numbers, its paste / thermal pieces) or an array
        std::vector<PAD*>     insidePads;
        std::set<wxString>    insideNumbers;

        for( PAD* pad : fp->Pads() )
        {
            if( inner.Contains( pad->GetPosition() ) )
            {
                insidePads.push_back( pad );
                insideNumbers.insert( pad->GetNumber() );
            }
        }

        if( insideNumbers.size() > 2 )
            continue;

        SHAPE_POLY_SET area;
        area.NewOutline();
        area.Append( inner.GetLeft(), inner.GetTop() );
        area.Append( inner.GetRight(), inner.GetTop() );
        area.Append( inner.GetRight(), inner.GetBottom() );
        area.Append( inner.GetLeft(), inner.GetBottom() );

        if( !insidePads.empty() )
        {
            SHAPE_POLY_SET open;

            for( PAD* pad : insidePads )
            {
                for( PCB_LAYER_ID l : pad->GetLayerSet().CuStack() )
                {
                    if( l == ( fp->GetLayer() == B_Cu ? B_Cu : F_Cu ) )
                        pad->TransformShapeToPolygon( open, l, clearance, ARC_LOW_DEF, ERROR_OUTSIDE );
                }
            }

            BOX2I openBox = open.BBox();
            openBox.Inflate( clearance );
            SHAPE_POLY_SET hole;
            hole.NewOutline();
            hole.Append( openBox.GetLeft(), openBox.GetTop() );
            hole.Append( openBox.GetRight(), openBox.GetTop() );
            hole.Append( openBox.GetRight(), openBox.GetBottom() );
            hole.Append( openBox.GetLeft(), openBox.GetBottom() );
            area.BooleanSubtract( hole );

            if( area.OutlineCount() == 0 )
                continue;
        }

        out.push_back( { fp, fp->GetLayer() == B_Cu ? B_Cu : F_Cu, std::move( area ), !insidePads.empty() } );
    }

    return out;
}


static KOPENAPI_RESULT routeOrBranch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );


static KOPENAPI_RESULT h_pcb_route( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_RESULT result = routeOrBranch( aCtx, aArgs );

    // job_cancel {apply: false} / {keep: true}: what was laid so far goes, whichever pass laid it (pair
    // trunks and pns passes write as they go, around a grid stage that wrote nothing)
    if( result.status == 200 && !aArgs.value( "dry_run", false ) && KopenapiJobDiscard() )
    {
        KOPENAPI_RESULT back = KOPENAPI_HISTORY::Get().RestoreBeforePending( aCtx );

        if( back.status != 200 )
            return back;

        result.body["applied"] = false;
        result.body["taken_back"] = true;
        result.body["taken_back_documents"] = back.body.value( "documents", nlohmann::json::object() );

        // the grid's timeline stays to branch from, no longer as the board's copper
        const nlohmann::json run = result.body.contains( "grid" ) && result.body["grid"].is_object()
                                           ? result.body["grid"].value( "timeline", nlohmann::json() )
                                           : nlohmann::json();

        if( run.is_object() && run.contains( "run" ) && run["run"].is_number_integer() )
        {
            if( std::shared_ptr<GRID_TIMELINE> t = GridTimelineFind( run["run"].get<int>() ) )
            {
                t->appliedSeq = 0;
                t->appliedItems.clear();
                t->appliedBySwitch = false;
            }
        }
    }

    return result;
}


static KOPENAPI_RESULT routeOrBranch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    if( !aArgs.contains( "from" ) )
        return pcbRoute( aCtx, aArgs, nullptr );

    if( aArgs.value( "dry_run", false ) )
        return KOPENAPI_RESULT::Error( 400, "dry_run with from: a branch goes on from copper on the board; route it for real "
                                            "(rollback undoes it)" );

    nlohmann::json from = aArgs["from"];
    bool           warmByNode = false;
    nlohmann::json nodeBoard;

    if( from.is_object() && from.contains( "node" ) )
    {
        int         run = 0, lane = -1;
        size_t      step = 0;
        std::string error;

        if( !from["node"].is_number_integer() )
            return KOPENAPI_RESULT::Error( 400, "from.node: a node id (pcb_variant_tree list)" );

        if( !VariantTreeNodeAt( from["node"].get<int>(), run, lane, step, error ) )
            return KOPENAPI_RESULT::Error( 404, error );

        from = { { "run", run }, { "lane", lane }, { "step", step } };

        if( !GridTimelineFind( run ) )
            return KOPENAPI_RESULT::Error( 409, "node " + aArgs["from"]["node"].dump() + ": run " + std::to_string( run )
                                                + "'s timeline was evicted (pcb_variant_tree budget)" );

        warmByNode = GridTimelineFind( run )->Lane( lane ).world != 0;

        // the board the node's lane ran over, unless asked to go on warm over the board as it is
        if( !aArgs.value( "warm", false ) )
        {
            std::shared_ptr<const BOARD_STATE> state;
            std::shared_ptr<PCB_CONTEXT>       context = KopenapiPcbContext( aCtx );

            if( !context )
                return KopenapiNoBoard();

            if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
                return *busy;

            if( !VariantTreeLaneBoard( aArgs["from"]["node"].get<int>(), state, error ) )
                return KOPENAPI_RESULT::Error( 409, error );

            BOARD_COMMIT              commit( context->GetToolManager() );
            const BOARD_STATE_APPLIED applied = BoardStateApply( context->GetBoard(), commit, *state );

            if( !applied.error.empty() )
            {
                commit.Revert();
                return KOPENAPI_RESULT::Error( 500, applied.error );
            }

            if( applied.removed + applied.added + applied.replaced > 0 )
                commit.Push( _( "Switch to a variant (API)" ) );
            else
                commit.Revert();

            GridTimelineFind( run )->appliedSeq = 0;
            GridTimelineFind( run )->appliedItems.clear();
            GridTimelineFind( run )->appliedBySwitch = false;
            nodeBoard = { { "removed", applied.removed }, { "added", applied.added }, { "replaced", applied.replaced } };
        }
    }

    if( !from.is_object() || !from.contains( "step" ) || !from["step"].is_number_integer() )
        return KOPENAPI_RESULT::Error( 400, "from: {node} or {run, lane (index or variant), step}" );

    if( from.contains( "run" ) && !from["run"].is_number_integer() )
        return KOPENAPI_RESULT::Error( 400, "from.run: a run id" );

    std::shared_ptr<GRID_TIMELINE> t = from.contains( "run" ) ? GridTimelineFind( from["run"].get<int>() ) : GridTimelineLast();

    if( !t )
        return KOPENAPI_RESULT::Error( 404, from.contains( "run" ) ? "run " + from["run"].dump() + " is not kept (pcb_variant_tree list: timelines)"
                                                                   : "no recorded route (pcb_route {engine: 'grid', record: true})" );

    if( t->live )
        return KOPENAPI_RESULT::Error( 409, "run " + std::to_string( t->run ) + " is still routing" );

    ROUTE_BRANCH          branch{ t, -1, 0 };
    const nlohmann::json& lane = from.value( "lane", nlohmann::json( t->appliedLane >= 0 ? t->appliedLane : t->winner ) );

    for( size_t k = 0; k < t->LaneCount(); ++k )
    {
        if( ( lane.is_number_integer() && lane.get<int>() == (int) k ) || ( lane.is_string() && lane.get<std::string>() == t->Lane( k ).name ) )
            branch.lane = (int) k;
    }

    if( branch.lane < 0 || t->Lane( branch.lane ).deleted )
        return KOPENAPI_RESULT::Error( 404, "from.lane: no such lane in run " + std::to_string( t->run ) );

    const long long step = from["step"].get<long long>();
    const size_t    steps = t->Lane( branch.lane ).steps.size();

    if( step < 0 || step >= (long long) steps )
        return KOPENAPI_RESULT::Error( 400, "from.step: 0.." + std::to_string( (long long) steps - 1 ) );

    branch.step = (size_t) step;
    branch.warm = aArgs.value( "warm", false ) || warmByNode;

    if( aArgs.contains( "history" ) )
    {
        const std::string h = aArgs["history"].is_string() ? aArgs["history"].get<std::string>() : std::string();

        if( h != "keep" && h != "clear" && h != "clear_edited" )
            return KOPENAPI_RESULT::Error( 400, "history: keep, clear or clear_edited" );

        branch.history = h == "clear" ? GRID_PARAMS::CLEAR : h == "clear_edited" ? GRID_PARAMS::CLEAR_EDITED : GRID_PARAMS::KEEP;
    }

    if( !branch.warm && t->Lane( branch.lane ).world )
        return KOPENAPI_RESULT::Error( 409, "lane " + t->Lane( branch.lane ).name + " went on over an edited board: branch it with warm: true" );

    nlohmann::json args = nlohmann::json::parse( t->args, nullptr, false );

    if( !args.is_object() )
        return KOPENAPI_RESULT::Error( 500, "the run's arguments are not kept" );

    // the board as the run found it: the copper of the run (or of its last branch) taken back when
    // that is the last edit; else the board must already be so
    KOPENAPI_HISTORY& history = KOPENAPI_HISTORY::Get();

    if( t->appliedSeq > 0 && history.LastSeq() == t->appliedSeq && !t->appliedBySwitch )
    {
        KOPENAPI_RESULT back = history.RollbackLastInCall( aCtx );

        if( back.status != 200 )
            return back;

        t->appliedSeq = 0;
        branch.board = "the route's copper rolled back";
    }
    else if( t->appliedSeq > 0 && ( branch.warm || t->appliedBySwitch ) && history.Holds( t->appliedSeq, "pcb" ) )
    {
        // under later edits, or switched to: the run's own tracks and vias taken out, the edits kept
        std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

        if( !context )
            return KopenapiNoBoard();

        if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
            return *busy;

        BOARD*       board = context->GetBoard();
        BOARD_COMMIT commit( context->GetToolManager() );
        branch.removed = 0;

        for( const KIID& id : t->appliedItems )
        {
            BOARD_ITEM* item = board->ResolveItem( id, true );

            if( item && ( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T || item->Type() == PCB_VIA_T ) )
            {
                commit.Remove( item );
                branch.removed++;
            }
        }

        if( branch.removed > 0 )
            commit.Push( _( "Take a route's copper out (API)" ) );
        else
            commit.Revert();

        // the fills still give way to the copper taken out: pads they joined before the run would
        // read as connections to route (other nets than the run's)
        if( branch.removed > 0 && args.value( "zones", std::string( "refill" ) ) == "refill" )
            refillPours( *context, board );

        t->appliedSeq = 0;
        t->appliedItems.clear();
        branch.board = t->appliedBySwitch ? "the copper of the node switched to taken out, the edits after it kept"
                                          : "the run's copper taken out, the edits after it kept";
        t->appliedBySwitch = false;
    }
    else
    {
        branch.board = t->appliedSeq > 0 && history.Holds( t->appliedSeq, "pcb" )
                               ? "the route is not the last edit (edit_log): the board was left as it is"
                               : "nothing of the run on the board to roll back";
    }

    if( !nodeBoard.is_null() )
    {
        branch.board = "the board of node " + aArgs["from"]["node"].dump() + " (items removed " + nodeBoard["removed"].dump()
                       + ", added " + nodeBoard["added"].dump() + ", replaced " + nodeBoard["replaced"].dump() + ")";
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
    ROUTE_PROFILE              profile;
    ROUTE_PROFILES::BIND       bindProfile( board, profile );

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
        {
            if( !KopenapiFindNet( board, n.get<std::string>() ) )
                return KopenapiNoSuchNet( board, n.get<std::string>(), "nets: " );

            nets.insert( str( UnescapeString( KopenapiFindNet( board, n.get<std::string>() )->GetNetname() ) ) );
        }
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

    const bool grid = engine == "grid";

    // attempts per connection and pass: a count, not a time (the same board routes the same way);
    // on a big board an attempt costs about a second
    const int maxTries = std::clamp( aArgs.value( "max_tries", 120 ), 1, 10000 );

    // the retry / rip-up pass gives each connection this long; a cancelled job stops between tries
    const double ripBudgetS = std::clamp( aArgs.value( "rip_up_budget_s", 30.0 ), 0.0, 3600.0 );
    std::optional<std::chrono::steady_clock::time_point> tryDeadline;

    auto overTime = [&]()
    {
        return KopenapiJobCancelled() || ( tryDeadline && std::chrono::steady_clock::now() > *tryDeadline );
    };

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

    // the board as the route found it, for the variant tree's nodes: before its temporary keep-outs and the fills taken out
    VARIANT_FOUND foundBoard;

    if( grid && ( aArgs.value( "record", false ) || aBranch ) )
        foundBoard = VariantTreeRouteFound( board );

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
        for( const UNDER_PART& under : underPartAreas( board ) )
        {
            auto* zone = new ZONE( board );
            zone->SetIsRuleArea( true );
            zone->SetDoNotAllowTracks( true );
            zone->SetDoNotAllowVias( true );
            zone->SetDoNotAllowPads( false );
            zone->SetDoNotAllowFootprints( false );
            zone->SetDoNotAllowZoneFills( false );
            zone->SetLayerSet( LSET( { under.layer } ) );
            *zone->Outline() = under.area;
            zone->UnHatchBorder();

            board->Add( zone );
            keepouts.zones.push_back( zone );
            keptOut.push_back( str( under.fp->GetReference() ) );
        }
    }

    // The airwires to route: pad-to-pad (or pad-to-track) connections of the ratsnest
    board->BuildConnectivity();
    std::shared_ptr<CONNECTIVITY_DATA> connectivity = board->GetConnectivity();
    connectivity->RecalculateRatsnest();

    // the board's open connections with the zones still filled: while the fills are out the
    // ratsnest also counts what the planes join, so the progress counts from here
    const KOPENAPI_UNROUTED openAtStart = KopenapiUnrouted( board, false );
    std::optional<KOPENAPI_UNROUTED> openAtEnd;

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
            if( KopenapiPadNeighbours( other, aPad ) )
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

    // A custom rule's track width where a pad is (a fine-pitch escape rule inside a courtyard): its
    // opt, else its min; 0 when only the class says
    auto ruleWidthAt = [&]( PAD* aPad, int aNetCode ) -> int
    {
        std::shared_ptr<DRC_ENGINE> drc = board->GetDesignSettings().m_DRCEngine;

        if( !drc )
            return 0;

        PCB_TRACK probe( board );
        probe.SetFlags( ROUTER_TRANSIENT );
        probe.SetNetCode( aNetCode );
        probe.SetStart( aPad->GetPosition() );
        probe.SetEnd( aPad->GetPosition() );
        probe.SetLayer( aPad->GetPrincipalLayer() );

        const DRC_CONSTRAINT c = drc->EvalRules( TRACK_WIDTH_CONSTRAINT, &probe, nullptr, aPad->GetPrincipalLayer() );

        if( !c.GetParentRule() || c.GetParentRule()->IsImplicit() )
            return 0;

        if( c.GetValue().HasOpt() )
            return c.GetValue().Opt();

        return c.GetValue().HasMin() ? c.GetValue().Min() : 0;
    };

    // Why a track of aWidth cannot leave a fine-pitch pad between its neighbours: centred on the
    // pad it needs aWidth / 2 + clearance up to the nearest pad's edge; "" when it fits
    auto padEntryProblem = [&]( PAD* aPad, int aNetCode, int aWidth ) -> std::string
    {
        PAD*   nearby = nullptr;
        double dist = std::numeric_limits<double>::max();

        for( PAD* other : aPad->GetParentFootprint()->Pads() )
        {
            const double d = ( other->GetPosition() - aPad->GetPosition() ).EuclideanNorm();

            if( KopenapiPadNeighbours( other, aPad ) && d < dist )
            {
                dist = d;
                nearby = other;
            }
        }

        if( !nearby )
            return std::string();

        const VECTOR2I sa = aPad->GetSize( PADSTACK::ALL_LAYERS );
        const VECTOR2I sb = nearby->GetSize( PADSTACK::ALL_LAYERS );
        const int      na = std::min( sa.x, sa.y ), nb = std::min( sb.x, sb.y );
        const double   gap = dist - na / 2.0 - nb / 2.0;
        int            clearance = std::max( board->GetDesignSettings().m_MinClearance, 0 );

        if( std::shared_ptr<DRC_ENGINE> drc = board->GetDesignSettings().m_DRCEngine )
        {
            PCB_TRACK probe( board );
            probe.SetFlags( ROUTER_TRANSIENT );
            probe.SetNetCode( aNetCode );
            probe.SetWidth( aWidth );
            probe.SetStart( aPad->GetPosition() );
            probe.SetEnd( aPad->GetPosition() );
            probe.SetLayer( aPad->GetPrincipalLayer() );

            const DRC_CONSTRAINT c = drc->EvalRules( CLEARANCE_CONSTRAINT, &probe, nearby, aPad->GetPrincipalLayer() );

            if( c.GetValue().HasMin() )
                clearance = std::max( clearance, c.GetValue().Min() );
        }

        const double room = gap + ( na - aWidth ) / 2.0;

        // pads that overlap or touch are not a row a track leaves between
        if( aWidth / 2.0 + clearance <= dist - nb / 2.0 || gap <= 0 )
            return std::string();

        auto mm = []( double aIU )
        {
            char text[32];
            std::snprintf( text, sizeof( text ), "%g", std::round( pcbIUScale.IUTomm( aIU ) * 1000.0 ) / 1000.0 );
            return std::string( text );
        };

        const std::string pad = str( aPad->GetParentFootprint()->GetReference() ) + "." + str( aPad->GetNumber() );

        if( aWidth > na )
        {
            return "pad " + pad + ": track width " + mm( aWidth ) + " wider than the pad (" + mm( na ) + ") and clearance "
                   + mm( clearance ) + " vs pad gap " + mm( gap ) + ": neck-down needed (neck_down auto, or a track_width rule "
                   "inside the courtyard)";
        }

        return "pad " + pad + ": clearance " + mm( clearance ) + " > pad gap " + mm( room ) + " at track width " + mm( aWidth )
               + " (pitch " + mm( dist ) + "): a clearance / track_width rule inside the courtyard is needed";
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

    // a via there would stand on or past the board's edge (escape spots of an edge connector's
    // fingers point out of the board)
    SHAPE_POLY_SET boardArea;
    const bool     hasBoardArea = board->GetBoardPolygonOutlines( boardArea, true ) && boardArea.OutlineCount() > 0;

    auto offBoard = [&]( BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aAt )
    {
        if( !hasBoardArea )
            return false;

        int       dia = ds.GetCurrentViaSize();
        NETCLASS* netclass = aEnd->GetEffectiveNetClass();

        if( netclass && netclass->HasViaDiameter() )
            dia = std::min( dia, netclass->GetViaDiameter() );

        const SEG::ecoord keep = dia / 2 + std::max( ds.m_CopperEdgeClearance, 0 );

        if( !boardArea.Contains( aAt ) )
            return true;

        // SHAPE_POLY_SET::SquaredDistance is 0 inside a polygon: measure to the edges themselves
        for( int i = 0; i < boardArea.OutlineCount(); ++i )
        {
            if( boardArea.COutline( i ).SquaredDistance( aAt, true ) < keep * keep )
                return true;

            for( int h = 0; h < boardArea.HoleCount( i ); ++h )
            {
                if( boardArea.CHole( i, h ).SquaredDistance( aAt, true ) < keep * keep )
                    return true;
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

        auto unusable = [&]( const VECTOR2I& aAt )
        {
            if( holeClash( aAt ) )
                return true;

            if( offBoard( aEnd, aAt ) )
            {
                profile.viaSpotsOffBoard++;
                return true;
            }

            if( viaBlocked( aEnd, aAt ) )
            {
                profile.viaSpotsBlocked++;
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
        // the plan only reads the board: off the GUI thread, readers read alongside
        ROUTE_PLAN plan;
        KopenapiCompute( [&]() { plan = PlanRoute( aReq ); } );

        profile.plans++;
        profile.plansFound += plan.found;
        profile.plansWholeBoard += plan.wholeBoard;
        profile.planMs += plan.ms;
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
            ROUTE_PROFILE&                        prof;
            long                                  outer = prof.job;
            std::chrono::steady_clock::time_point from = std::chrono::steady_clock::now();
            ~TIMED()
            {
                job.ms += msSince( from );
                prof.job = outer;
            }
        } timed{ aJob, profile };

        profile.job = &aJob - jobs.data();

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
        int  boxed = 0;      // the plan's end shut in a room of its own: 1 the start, 2 the target
        auto planTry = [&]()
        {
            if( ok || !planning || rail || ways.empty() || aTries >= maxTries )
                return;

            const WAY&       way = ways.front();
            const ROUTE_PLAN plan = makePlan( planRequest( way.from, way.to, way.toPos ) );

            planned = plan.found ? 1 : 0;

            if( !plan.found && plan.boxedEnd )
            {
                boxed = plan.boxedEnd;
                BOARD_CONNECTED_ITEM* end = plan.boxedEnd == 1 ? static_cast<BOARD_CONNECTED_ITEM*>( way.from ) : way.to;
                const VECTOR2I        at = plan.boxedEnd == 1 ? way.from->GetPosition() : way.toPos;
                PCB_LAYER_ID          layer = end->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
                nlohmann::json        by = nlohmann::json::array();
                std::string           list;

                // other nets' tracks and vias nearest the end's copper (its own part's pads are its
                // neighbours anyway), then the pads
                {
                    std::shared_ptr<SHAPE>                      own = end->GetEffectiveShape( layer );
                    std::vector<std::pair<int, nlohmann::json>> nearby;
                    BOX2I                                       area = end->GetBoundingBox();
                    area.Inflate( pcbIUScale.mmToIU( 1.0 ) );

                    for( PCB_TRACK* t : board->Tracks() )
                    {
                        int actual = 0;

                        if( !own || t->GetNetCode() == way.from->GetNetCode() || !t->IsOnLayer( layer )
                            || !area.Intersects( t->GetBoundingBox() )
                            || !t->GetEffectiveShape( layer )->Collide( own.get(), pcbIUScale.mmToIU( 1.0 ), &actual ) )
                        {
                            continue;
                        }

                        nearby.emplace_back( actual, nlohmann::json{ { "type", t->Type() == PCB_VIA_T ? "via" : "track" },
                                                                   { "net", str( UnescapeString( t->GetNetname() ) ) },
                                                                   { "uuid", str( t->m_Uuid.AsString() ) }, { "key", copperKey( t ) },
                                                                   { "gap_mm", toMm( actual ) } } );
                    }

                    std::stable_sort( nearby.begin(), nearby.end(), []( const auto& x, const auto& y ) { return x.first < y.first; } );

                    for( size_t k = 0; k < nearby.size() && k < 3; ++k )
                        by.push_back( nearby[k].second );

                    for( const nlohmann::json& b : blockersAt( board, at, at, layer, way.from->GetNetCode(),
                                                               std::max( 1, neckFor( way.from, way.to ) / 2 ) ) )
                    {
                        if( b.contains( "pad" ) && by.size() < 5 )
                            by.push_back( b );
                    }
                }

                for( const nlohmann::json& b : by )
                {
                    std::string what = b.contains( "pad" ) ? "pad " + b["pad"].get<std::string>() + " ("
                                                                     + mmText( b.value( "distance_mm", 0.0 ) ) + " mm from its centre)"
                                                           : b.value( "net", std::string() ) + " " + b.value( "type", std::string( "copper" ) ) + " "
                                                                     + b.value( "uuid", std::string() ) + " [" + b.value( "key", std::string() ) + "] ("
                                                                     + mmText( b.value( "gap_mm", 0.0 ) ) + " mm gap)";
                    list += ( list.empty() ? "" : ", " ) + what;
                }

                aAttempt["boxed_in"] = { { "end", endName( end ) },
                                         { "which", plan.boxedEnd == 1 ? "start" : "target" },
                                         { "room_cells", plan.boxedRoom },
                                         { "by", by } };
                aAttempt["boxed_reason"] = std::string( plan.boxedEnd == 1 ? "start " : "target " ) + endName( end )
                                           + " is boxed in: no way out of it at this width and clearance"
                                           + ( list.empty() ? std::string() : ", by " + list );
            }

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

        // a walkaround that never got away from the start pad (it heads for the target and the pad's
        // side that way is shut): a corner 1 mm out of each free side of the pad, the outward side first
        auto exitTry = [&]()
        {
            if( ok || rail || ways.empty() || boxed == 2 || !aAttempt.contains( "stopped_at_mm" ) )
                return;

            const WAY&     way = ways.front();
            PAD*           pad = way.from;
            const VECTOR2I at = pad->GetPosition();
            const VECTOR2I stop( pcbIUScale.mmToIU( aAttempt["stopped_at_mm"][0].get<double>() ),
                                 pcbIUScale.mmToIU( aAttempt["stopped_at_mm"][1].get<double>() ) );
            const VECTOR2I size = pad->GetSize( PADSTACK::ALL_LAYERS );
            const int      reach = pcbIUScale.mmToIU( 1.0 );   // how far out of the pad the corners stand

            // stopped on the pad or nearer it than the corners: they give the walk another start
            if( ( stop - at ).EuclideanNorm() > std::max( size.x, size.y ) / 2.0 + reach )
                return;

            const VECTOR2I centre = pad->GetParentFootprint() ? pad->GetParentFootprint()->GetPosition() : at;
            const VECTOR2D aim = VECTOR2D( way.toPos - at ).Resize( 1.0 );
            VECTOR2D       outward = VECTOR2D( at - centre );
            outward = outward.EuclideanNorm() > 0 ? outward.Resize( 1.0 ) : aim;

            std::vector<std::pair<double, VECTOR2I>> exits;

            for( int axis = 0; axis < 4; ++axis )
            {
                VECTOR2D     dir( axis == 0 ? 1 : axis == 1 ? -1 : 0, axis == 2 ? 1 : axis == 3 ? -1 : 0 );
                const double half = ( axis < 2 ? size.x : size.y ) / 2.0;
                RotatePoint( dir, pad->GetOrientation() );

                const double out = dir.Dot( outward );

                // into the package: no way out there
                if( out < -0.5 )
                    continue;

                const double   d = half + reach;
                const VECTOR2I corner = at + VECTOR2I( KiROUND( dir.x * d ), KiROUND( dir.y * d ) );
                exits.emplace_back( -( 2.0 * out + dir.Dot( aim ) ), corner );
            }

            std::sort( exits.begin(), exits.end(), []( const auto& x, const auto& y ) { return x.first < y.first; } );

            const nlohmann::json before = aAttempt;
            nlohmann::json       tried = nlohmann::json::array();
            const PCB_LAYER_ID   layer = pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;

            if( !exits.empty() )
                profile.padExitsTried++;

            for( const auto& [score, corner] : exits )
            {
                if( ok || tried.size() >= 3 )
                    break;

                ROUTE_REQUEST req{ way.from, way.to, way.toPos, layer, PNS::RM_Walkaround, dryRun, {} };
                req.fronts = &fronts;
                req.neckWidth = neckFor( way.from, way.to );
                req.vias = { { corner, std::nullopt } };
                req.noDoubleBack = true;
                aAttempt = nlohmann::json::object();
                aTries++;
                ok = routeOne( *context, req, aAttempt, added );
                tried.push_back( { toMm( corner.x ), toMm( corner.y ) } );
            }

            if( ok )
            {
                profile.padExitsWon++;
                aAttempt["left_pad_by"] = tried.back();
            }
            else
            {
                aAttempt = before;
                aAttempt["pad_exits_tried_mm"] = tried;
            }
        };

        exitTry();

        if( !ok && aAttempt.contains( "boxed_reason" ) )
            aAttempt["reason"] = aAttempt["boxed_reason"];

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
                        if( ok || aTries >= maxTries || budget-- <= 0 || overTime() )
                            break;

                        if( deadFirstVia.count( { v.x, v.y } ) || deadLastVia.count( { v.x, v.y } ) )
                        {
                            profile.deadLegsSkipped++;
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
                            profile.deadLegsSkipped++;
                            break;
                        }

                        if( ok || aTries >= maxTries || budget <= 0 || overTime() )
                            break;

                        if( deadLastVia.count( { vb.x, vb.y } ) )
                        {
                            profile.deadLegsSkipped++;
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
        else if( !ok && overTime() )
            aAttempt["reason"] = std::string( KopenapiJobCancelled() ? "stopped (job_cancel)" : "the connection's rip_up_budget_s ran out" )
                                 + " after " + std::to_string( aTries ) + " tries; last: " + aAttempt.value( "reason", std::string( "-" ) );

        return ok;
    };

    // Differential pairs: for a net named as the P side of a pair (+ or P suffix) whose N net is
    // routed too, a trunk between each two footprints next in a chain over the footprints with pads of
    // both nets (connector - ESD - MCU), from the P and N pads nearest each other there.  The trunks
    // go first and coupled; airwires their copper joins afterwards are not routed again.
    std::vector<JOB> trunks;
    nlohmann::json   pairsSkipped = nlohmann::json::array();

    if( diffPairs == "auto" && !rail )
    {
        std::set<std::string> jobNets;

        for( const JOB& job : jobs )
            jobNets.insert( job.net );

        for( const std::string& netP : jobNets )
        {
            wxString other;

            if( board->MatchDpSuffix( wxString::FromUTF8( netP ), other ) != 1 || !jobNets.count( str( other ) ) )
            {
                // USB's DP / DM naming (USB_DP, USB_DP_MCU): KiCad's pairing knows +/- and P/N only
                static const std::regex dp( R"(^(.*D)P((?:_[^/]*)?)$)" );
                std::smatch             m;

                if( !std::regex_match( netP, m, dp ) || !jobNets.count( m[1].str() + "M" + m[2].str() ) )
                    continue;

                other = wxString::FromUTF8( m[1].str() + "M" + m[2].str() );
            }

            NETINFO_ITEM* infoP = KopenapiFindNet( board, netP );
            NETINFO_ITEM* infoN = KopenapiFindNet( board, str( other ) );

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

            // a pair through two parts side by side (series resistors, one per line): a P pad and the
            // nearest N pad on another footprint that has no pad of the P net, each the other's nearest
            {
                std::vector<PAD*> lonePs, loneNs;

                for( FOOTPRINT* fp : board->Footprints() )
                {
                    bool hasP = false, hasN = false;

                    for( PAD* pad : fp->Pads() )
                    {
                        hasP |= pad->GetNetCode() == infoP->GetNetCode();
                        hasN |= pad->GetNetCode() == infoN->GetNetCode();
                    }

                    if( hasP == hasN )
                        continue;

                    for( PAD* pad : fp->Pads() )
                    {
                        if( pad->GetNetCode() == infoP->GetNetCode() )
                            lonePs.push_back( pad );
                        else if( pad->GetNetCode() == infoN->GetNetCode() )
                            loneNs.push_back( pad );
                    }
                }

                auto nearest = [&]( PAD* aFrom, const std::vector<PAD*>& aAmong ) -> PAD*
                {
                    PAD*   best = nullptr;
                    double bestGap = std::numeric_limits<double>::max();

                    for( PAD* pad : aAmong )
                    {
                        const double gap = ( pad->GetPosition() - aFrom->GetPosition() ).EuclideanNorm();

                        if( pad->GetParentFootprint() != aFrom->GetParentFootprint() && gap < bestGap
                            && ( pad->GetLayerSet() & aFrom->GetLayerSet() & LSET::AllCuMask() ).any() )
                        {
                            bestGap = gap;
                            best = pad;
                        }
                    }

                    return best;
                };

                const double reach = pcbIUScale.mmToIU( 5.0 );

                for( PAD* p : lonePs )
                {
                    PAD* n = nearest( p, loneNs );

                    if( n && nearest( n, lonePs ) == p && ( n->GetPosition() - p->GetPosition() ).EuclideanNorm() <= reach )
                        ends.push_back( { p, n } );
                }
            }

            if( ends.size() < 2 )
            {
                pairsSkipped.push_back( { { "nets", { netP, str( other ) } },
                                          { "reason", "fewer than two places where the P and N pads sit side by side "
                                                      "(a footprint with both, or two parts within 5 mm of each other): "
                                                      "the nets route alone" } } );
                continue;
            }

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

    // live progress: this call's connections (airwires, then gaps) summed from the rows as the
    // totals are, and the board's open connections now; at most every 200 ms within a pass
    std::string                           progressPass;
    nlohmann::json                        progressLast;
    std::chrono::steady_clock::time_point progressAt;

    auto progressCounts = [&]( const std::string& aPass, int aRound = 0 ) -> nlohmann::json
    {
        const auto now = std::chrono::steady_clock::now();

        if( aPass == progressPass && now - progressAt < std::chrono::milliseconds( 200 ) )
            return progressLast;

        int    routed = 0, failed = 0, vias = 0;
        double length = 0;

        for( const nlohmann::json& row : results )
        {
            if( !row.is_object() )
                continue;

            const bool ok = row.value( "routed", false );

            if( ok )
            {
                length += row.value( "length_mm", 0.0 );
                vias += row.value( "vias", 0 );
            }

            if( !row.value( "trunk", false ) )
                ( ok ? routed : failed )++;
        }

        const int connections = (int) ( jobs.size() - trunks.size() + gaps.size() );

        nlohmann::json counts = { { "pass", aPass },
                                  { "connections", connections },
                                  { "done", routed + failed },
                                  { "routed", routed },
                                  { "failed", failed },
                                  { "vias", vias },
                                  { "length_mm", std::round( length * 1000.0 ) / 1000.0 } };

        if( openAtEnd )
        {
            counts["unrouted_connections"] = openAtEnd->connections;
            counts["unrouted_nets"] = openAtEnd->nets;
            counts["zone_fills"] = openAtEnd->zoneFills;
            counts["unrouted_basis"] = "board";
        }
        else
        {
            counts["unrouted_connections"] = std::max( 0, openAtStart.connections - routed );
            counts["zone_fills"] = openAtStart.zoneFills;
            counts["unrouted_basis"] = "start_minus_routed";
        }

        counts["unrouted_left"] = counts["unrouted_connections"];

        if( aRound > 0 )
            counts["rip_up_round"] = aRound;

        // the first pass settles every connection once: the share of the call done
        if( aPass == "first" && connections > 0 )
            counts["fraction"] = std::round( 1000.0 * ( routed + failed ) / connections ) / 1000.0;

        progressPass = aPass;
        progressAt = now;
        progressLast = counts;
        return counts;
    };

    for( size_t index = 0; index < jobs.size(); ++index )
    {
        JOB& job = jobs[index];

        if( grid && partner[index] < 0 )
            continue;   // on the grid below

        if( !KopenapiJobStep( "route", (int) index, (int) jobs.size(), job.net, progressCounts( "first" ) ) )
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

        // a grid dry run leaves the trunks out: the grid search could not see their uncommitted
        // copper, so the pair's airwires are in the grid search instead
        if( partner[index] >= 0 && grid && dryRun )
        {
            const size_t mate = partner[index];

            for( size_t k : { index, mate } )
            {
                rowOf[k] = (int) results.size();
                results.push_back( { { "net", jobs[k].net },
                                     { "routed", false },
                                     { "trunk", true },
                                     { "engine", "pns" },
                                     { "not_simulated", true },
                                     { "tries", 0 },
                                     { "diff_pair", { { "with", jobs[k == index ? mate : index].net } } },
                                     { "reason", "a grid dry run does not route diff-pair trunks: a real run routes them coupled with pns "
                                                 "and commits them before the grid search, which a dry run cannot do; the pair's "
                                                 "airwires are in the grid search instead" } } );
            }

            continue;
        }

        if( partner[index] >= 0 )
        {
            const size_t   mate = partner[index];
            JOB&           other = jobs[mate];
            nlohmann::json pairAttempt = nlohmann::json::object();
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
                                           { "engine", "pns" },
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
                                     { "engine", "pns" },
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

            if( attempt.contains( "left_pad_by" ) )
                row["left_pad_by_mm"] = attempt["left_pad_by"];
        }
        else
        {
            row["reason"] = attempt.value( "reason", std::string() );

            for( const char* key : { "violations", "stopped_at_mm", "blocked_by", "plan", "boxed_in", "pad_exits_tried_mm" } )
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

    // the end node keeps the board as the call leaves it (fills refilled, stubs out)
    struct
    {
        std::shared_ptr<GRID_TIMELINE> timeline;
        int                            lane = -1;
        nlohmann::json                 metrics;
    } endNode;

    std::vector<bool> gapTaken( gaps.size(), false );   // the grid took the gap: no pns pass for it

    if( grid )
    {
        board->BuildConnectivity();
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
        std::vector<GRID_NET>              gridNets;
        std::map<int, size_t>              gridOf;      // net code -> index in gridNets
        std::vector<std::vector<size_t>>   jobsOfNet;
        std::vector<std::pair<int, int>>   endsOfJob( jobs.size(), { -1, -1 } );
        std::vector<int>                   gridClassWidth;   // per grid net: its class width before any neck-down

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

            if( fresh )
                gridClassWidth.push_back( n.width );

            if( const int neck = neckFor( job.from, job.to ); neck > 0 )
            {
                n.width = std::min( n.width, neck );

                for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( job.from ), job.to } )
                {
                    if( end && end->Type() == PCB_PAD_T && pitchOf( static_cast<PAD*>( end ) ) <= pcbIUScale.mmToIU( 0.65 ) )
                    {
                        if( const int w = ruleWidthAt( static_cast<PAD*>( end ), code ); w > 0 )
                            n.width = std::min( n.width, std::max( w, ds.m_TrackMinWidth ) );
                    }
                }
            }

            endsOfJob[index] = { endIndex( n, job.from ), endIndex( n, job.to ) };
            jobsOfNet[it->second].push_back( index );
        }

        // gaps between pieces of a net's copper (stubs, fanned vias): their ends are the grid's too
        struct GRID_GAP
        {
            size_t gap;
            size_t net;
            int    a, b;
        };

        std::vector<GRID_GAP> gridGaps;

        for( size_t g = 0; g < gaps.size(); ++g )
        {
            auto* a = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( gaps[g].a, true ) );
            auto* b = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( gaps[g].b, true ) );

            if( !a || !b || a->GetNetCode() <= 0 )
                continue;

            const auto joined = conn->GetConnectedItems( a );

            if( std::find( joined.begin(), joined.end(), b ) != joined.end() )
                continue;

            const int code = a->GetNetCode();
            auto [it, fresh] = gridOf.try_emplace( code, gridNets.size() );

            if( fresh )
            {
                NETCLASS* nc = a->GetEffectiveNetClass();
                GRID_NET  n;
                n.netCode = code;
                n.width = nc && nc->HasTrackWidth() ? nc->GetTrackWidth() : ds.GetCurrentTrackWidth();
                n.clearance = netClearance( a, ds.m_MinClearance );
                n.viaDiameter = nc && nc->HasViaDiameter() ? nc->GetViaDiameter() : ds.GetCurrentViaSize();
                n.viaDrill = nc && nc->HasViaDrill() ? nc->GetViaDrill() : ds.GetCurrentViaDrill();
                gridNets.push_back( n );
                jobsOfNet.emplace_back();
                gridClassWidth.push_back( n.width );
            }

            GRID_NET& n = gridNets[it->second];

            // a stub as narrow as the stub: it leaves the pin field the way it was drawn
            for( BOARD_CONNECTED_ITEM* end : { a, b } )
            {
                if( end->Type() == PCB_TRACE_T || end->Type() == PCB_ARC_T )
                    n.width = std::min( n.width, std::max( static_cast<PCB_TRACK*>( end )->GetWidth(), ds.m_TrackMinWidth ) );
            }

            gridGaps.push_back( { g, it->second, endIndex( n, a ), endIndex( n, b ) } );
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
                    NETINFO_ITEM* info = KopenapiFindNet( board, netName );

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
        params.fillsOut = dryRun && zoneMode == "refill";
        params.maxRounds = std::clamp( aArgs.value( "max_rounds", 40 ), 1, 500 );
        params.variants = std::clamp( aArgs.value( "variants", 0 ), 0, 6 );
        params.cell = pcbIUScale.mmToIU( std::clamp( aArgs.value( "cell_mm", 0.0 ), 0.0, 1.0 ) );
        params.viaKeepOut = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_keepout_mm", 5.0 ), 0.0, 20.0 ) );
        params.viaAway = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_away_mm", 7.0 ), 0.0, 30.0 ) );
        params.viaAwayCost = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_away_cost_mm", 8.0 ), 0.0, 100.0 ) );
        params.rowsCost = (float) std::clamp( aArgs.value( "rows_cost", 6.0 ), 1.0, 100.0 );
        params.viaNearHole = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_near_hole_mm", 2.5 ), 0.0, 30.0 ) );
        params.viaNearHoleCost = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_near_hole_cost_mm", 6.0 ), 0.0, 100.0 ) );
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
            params.warm = aBranch->warm;
            params.warmHistory = aBranch->history;
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
            nlohmann::json counts = wxThread::IsMain() ? progressCounts( "grid" ) : nlohmann::json::object();
            counts["nets"] = gridNets.size();
            counts["crowded"] = crowded;
            counts["variants"] = variants;

            return KopenapiJobStep( "grid", finished, (int) aVariants.size(), std::string(), counts );
        };

        // a cancelled job stops every variant at its next net, not only at the next progress report
        params.held = [&cancel]( uint64_t& aSerial, std::string& aBy, int& aN )
        {
            if( KopenapiJobCancelled() )
            {
                cancel = true;
                return false;
            }

            return KopenapiJobHeld( &aSerial, &aBy, &aN );
        };

        // the timeline readable (pcb_route_timeline, the replay) while the variants add to it
        params.timelineStarted = [&]( const std::shared_ptr<GRID_TIMELINE>& aTimeline )
        {
            static int runs = 0;

            if( aBranch && aTimeline == aBranch->timeline )
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

        // the grid search only reads the board: off the GUI thread, readers read alongside
        GRID_RESULT gr;
        KopenapiCompute( [&]() { gr = RouteOnGrid( board, gridNets, params ); } );
        nlohmann::json    timelineOut;

        if( gr.timeline && ( !aBranch || gr.timeline != aBranch->timeline ) )
            VariantTreeRunStarted( gr.timeline, foundBoard );

        if( gr.timeline && gr.branchLane >= 0 && aBranch && aBranch->warm )
            VariantTreeWarmLane( gr.timeline, gr.branchLane, foundBoard );

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

        // job_cancel {keep} / {apply: false}: nothing on the board, the lanes stay where they stopped
        // to branch from
        if( KopenapiJobDiscard() )
        {
            nlohmann::json lanes = nlohmann::json::array();
            int            best = -1;

            if( gr.timeline )
            {
                // the pause node: the stopped lane with the fewest nets crowded, then the furthest
                auto rank = [&]( int aLane )
                {
                    const GRID_LANE& l = gr.timeline->Lane( aLane );
                    return std::make_pair( l.steps.empty() ? INT_MAX : l.steps.back().crowded, -(long long) l.steps.size() );
                };

                for( size_t k = 0; k < gr.timeline->LaneCount(); ++k )
                {
                    const GRID_LANE& lane = gr.timeline->Lane( k );

                    if( !lane.stopped || lane.deleted )
                        continue;

                    lanes.push_back( { { "lane", k }, { "variant", lane.name }, { "steps", lane.steps.size() } } );

                    if( !lane.steps.empty() && ( best < 0 || rank( (int) k ) < rank( best ) ) )
                        best = (int) k;
                }
            }

            nlohmann::json out = { { "stopped", true }, { "applied", false } };

            if( gr.timeline )
            {
                out["timeline"] = { { "run", gr.timeline->run }, { "stopped_lanes", lanes } };

                if( best >= 0 )
                {
                    out["timeline"]["node"] = VariantTreeAutoNode( gr.timeline, best, gr.timeline->Lane( best ).steps.size() - 1,
                                                                   "pause", nlohmann::json::object(), board );
                }
            }

            return KOPENAPI_RESULT::Ok( out );
        }

        if( !gr.error.empty() )
            return KOPENAPI_RESULT::Error( 409, gr.error + ( aBranch ? "; " + aBranch->board : std::string() ) );

        if( gr.timeline )
            timelineOut = { { "run", gr.timeline->run }, { "bytes", gr.timeline->Bytes() },
                            { "print_ms", std::round( gr.printMs * 10 ) / 10 } };

        if( gr.branchLane >= 0 && gr.timeline )
        {
            const GRID_LANE& lane = gr.timeline->Lane( gr.branchLane );
            timelineOut["branch"] = { { "lane", gr.branchLane }, { "variant", lane.name }, { "parent", lane.parent },
                                      { "parent_step", lane.parentStep }, { "steps", lane.steps.size() },
                                      { "restore_ms", std::round( gr.restoreMs * 10 ) / 10 } };

            if( aBranch && aBranch->warm )
            {
                nlohmann::json affected = nlohmann::json::array();

                for( const auto& [code, why] : gr.warmAffected )
                {
                    NETINFO_ITEM* info = board->FindNet( code );
                    affected.push_back( { { "net", info ? str( UnescapeString( info->GetNetname() ) ) : std::to_string( code ) },
                                          { "why", why } } );
                }

                timelineOut["branch"]["warm"] = { { "round", gr.warmRound }, { "affected", affected },
                                                  { "edited_tiles", gr.warmEditedTiles }, { "board", aBranch->board } };

                if( aBranch->removed >= 0 )
                    timelineOut["branch"]["warm"]["copper_removed"] = aBranch->removed;
            }
        }
        else if( aBranch && !gr.cold.empty() )
        {
            timelineOut["cold"] = gr.cold;
        }

        // each net's copper staged and guarded against the rest; a net the guard rejects stays out
        BOARD_COMMIT                        commit( context->GetToolManager() );
        std::vector<KIID>                   gridItems;
        std::vector<BOARD_CONNECTED_ITEM*>  staged;
        nlohmann::json                      rejected = nlohmann::json::array();
        std::vector<std::unique_ptr<PCB_TRACK>> dryRunCopper;
        std::vector<bool>                   netOk( gridNets.size(), false );
        std::vector<int>                    netVias( gridNets.size(), 0 );
        const auto                          guardFrom = std::chrono::steady_clock::now();

        // a necked net is narrow only inside the courtyards of its fine-pitch ends (the pad
        // entry); its tracks are split there and the parts outside are widened to the class
        // width once every net is staged
        std::vector<std::vector<PCB_TRACK*>> widenable( gridNets.size() );
        std::vector<int64_t>                 neckLength( gridNets.size(), 0 );

        auto neckZones = [&]( size_t k )
        {
            std::vector<SHAPE_POLY_SET> zones;
            std::set<FOOTPRINT*>        seen;

            for( size_t index : jobsOfNet[k] )
            {
                for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( jobs[index].from ), jobs[index].to } )
                {
                    if( !end || end->Type() != PCB_PAD_T || pitchOf( static_cast<PAD*>( end ) ) > pcbIUScale.mmToIU( 0.65 ) )
                        continue;

                    FOOTPRINT* fp = static_cast<PAD*>( end )->GetParentFootprint();

                    if( !fp || !seen.insert( fp ).second )
                        continue;

                    SHAPE_POLY_SET zone = fp->GetCourtyard( fp->IsFlipped() ? B_CrtYd : F_CrtYd );

                    if( zone.OutlineCount() == 0 )
                    {
                        BOX2I box;

                        for( PAD* pad : fp->Pads() )
                            box.Merge( pad->GetBoundingBox() );

                        box.Inflate( pitchOf( static_cast<PAD*>( end ) ) );
                        zone.NewOutline();
                        zone.Append( box.GetLeft(), box.GetTop() );
                        zone.Append( box.GetRight(), box.GetTop() );
                        zone.Append( box.GetRight(), box.GetBottom() );
                        zone.Append( box.GetLeft(), box.GetBottom() );
                    }

                    zones.push_back( std::move( zone ) );
                }
            }

            return zones;
        };

        for( size_t k = 0; k < gr.nets.size(); ++k )
        {
            const GRID_NET&                         n = gridNets[k];
            const GRID_NET_RESULT&                  r = gr.nets[k];
            NETINFO_ITEM*                           net = board->FindNet( n.netCode );
            std::vector<std::unique_ptr<PCB_TRACK>> made;
            std::vector<BOARD_CONNECTED_ITEM*>      mine;
            const bool                              necked = n.width < gridClassWidth[k];
            const std::vector<SHAPE_POLY_SET>       zones = necked ? neckZones( k ) : std::vector<SHAPE_POLY_SET>();
            std::vector<PCB_TRACK*>                 outside;

            auto inZone = [&]( const VECTOR2I& aPt )
            {
                for( const SHAPE_POLY_SET& zone : zones )
                {
                    if( zone.Contains( aPt ) )
                        return true;
                }

                return false;
            };

            for( const GRID_TRACK& t : r.tracks )
            {
                for( size_t p = 1; p < t.points.size(); ++p )
                {
                    const SEG         whole( t.points[p - 1], t.points[p] );
                    std::vector<VECTOR2I> cuts = { whole.A, whole.B };

                    for( const SHAPE_POLY_SET& zone : zones )
                    {
                        for( int o = 0; o < zone.OutlineCount(); ++o )
                        {
                            SHAPE_LINE_CHAIN::INTERSECTIONS hits;
                            zone.COutline( o ).Intersect( whole, hits );

                            for( const SHAPE_LINE_CHAIN::INTERSECTION& hit : hits )
                                cuts.push_back( hit.p );
                        }
                    }

                    std::sort( cuts.begin(), cuts.end(),
                               [&]( const VECTOR2I& a, const VECTOR2I& b )
                               { return ( a - whole.A ).SquaredEuclideanNorm() < ( b - whole.A ).SquaredEuclideanNorm(); } );
                    cuts.erase( std::unique( cuts.begin(), cuts.end() ), cuts.end() );

                    for( size_t c = 1; c < cuts.size(); ++c )
                    {
                        auto track = std::make_unique<PCB_TRACK>( board );
                        track->SetStart( cuts[c - 1] );
                        track->SetEnd( cuts[c] );
                        track->SetWidth( n.width );
                        track->SetLayer( t.layer );
                        track->SetNet( net );

                        if( necked && !inZone( ( cuts[c - 1] + cuts[c] ) / 2 ) )
                            outside.push_back( track.get() );

                        mine.push_back( track.get() );
                        made.push_back( std::move( track ) );
                    }
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
            widenable[k] = outside;
            staged.insert( staged.end(), mine.begin(), mine.end() );

            // a dry run keeps the staged copper off the board, alive until the guard is done
            for( std::unique_ptr<PCB_TRACK>& item : made )
            {
                if( dryRun )
                {
                    dryRunCopper.push_back( std::move( item ) );
                    continue;
                }

                added.push_back( item->m_Uuid );
                gridItems.push_back( item->m_Uuid );
                commit.Add( item.release() );
            }
        }

        // the necked nets' copper outside the pad entry at the class width where the guard lets it
        std::vector<int64_t> narrowOutside( gridNets.size(), 0 );
        std::vector<int>     widened( gridNets.size(), 0 );

        for( size_t k = 0; k < gridNets.size(); ++k )
        {
            for( PCB_TRACK* track : widenable[k] )
            {
                int wide = gridClassWidth[k];

                if( std::shared_ptr<DRC_ENGINE> drc = ds.m_DRCEngine )
                {
                    track->SetWidth( wide );
                    const DRC_CONSTRAINT c = drc->EvalRules( TRACK_WIDTH_CONSTRAINT, track, nullptr, track->GetLayer() );

                    if( c.GetParentRule() && !c.GetParentRule()->IsImplicit() && c.GetValue().HasMax() )
                        wide = std::min( wide, c.GetValue().Max() );
                }

                if( wide <= gridNets[k].width )
                {
                    track->SetWidth( gridNets[k].width );
                    narrowOutside[k] += track->GetLength();
                    continue;
                }

                track->SetWidth( wide );

                if( !guard( board, { track }, {}, zoneMode == "refill", &staged ).empty() )
                {
                    track->SetWidth( gridNets[k].width );
                    narrowOutside[k] += track->GetLength();
                    continue;
                }

                ++widened[k];
            }
        }

        for( size_t k = 0; k < gridNets.size(); ++k )
        {
            if( gridNets[k].width >= gridClassWidth[k] )
                continue;

            for( BOARD_CONNECTED_ITEM* item : staged )
            {
                if( item->Type() == PCB_TRACE_T && item->GetNetCode() == gridNets[k].netCode
                    && static_cast<PCB_TRACK*>( item )->GetWidth() == gridNets[k].width )
                {
                    neckLength[k] += static_cast<PCB_TRACK*>( item )->GetLength();
                }
            }
        }

        if( staged.empty() || dryRun )
            commit.Revert();
        else
            commit.Push( _( "Route on the grid (API)" ) );

        if( gr.timeline && !staged.empty() && !dryRun )
        {
            gr.timeline->appliedSeq = KOPENAPI_HISTORY::Get().PendingSeq();
            gr.timeline->appliedLane = gr.branchLane >= 0 ? gr.branchLane : gr.timeline->winner;
            gr.timeline->appliedItems = gridItems;
            gr.timeline->appliedBySwitch = false;
        }

        for( size_t k = 0; k < gridNets.size(); ++k )
        {
            const GRID_NET_RESULT* r = k < gr.nets.size() ? &gr.nets[k] : nullptr;
            bool                   first = true;

            for( size_t index : jobsOfNet[k] )
            {
                const auto [a, b] = endsOfJob[index];
                const bool reached = r && a < r->reached.size() && b < r->reached.size() && r->reached[a] && r->reached[b];
                const bool ok = netOk[k] && reached;
                nlohmann::json row = { { "net", jobs[index].net }, { "routed", ok }, { "tries", 1 }, { "engine", "grid" } };

                if( ok )
                {
                    row["length_mm"] = first ? toMm( r->length ) : 0.0;
                    row["vias"] = first ? netVias[k] : 0;
                    row["width_mm"] = toMm( widened[k] > 0 ? gridClassWidth[k] : gridNets[k].width );

                    for( int e : r->halfCellExits )
                    {
                        if( ( e == (int) a || e == (int) b ) && e < (int) gridNets[k].ends.size() )
                            row["half_cell_exits"].push_back( endName( gridNets[k].ends[e] ) );
                    }

                    if( gridNets[k].width < gridClassWidth[k] )
                    {
                        row["neck_width_mm"] = toMm( gridNets[k].width );
                        row["neck_length_mm"] = toMm( neckLength[k] );

                        if( narrowOutside[k] > 0 )
                        {
                            row["narrow_outside_mm"] = toMm( narrowOutside[k] );
                            row["narrow_outside_reason"] = "kept at the neck width outside the pad entry: the class width would break a clearance or a width rule there";
                        }
                    }

                    first = false;
                }
                else
                {
                    row["reason"] = !reached
                                            ? ( KopenapiJobCancelled() ? "stopped" : "no way on the grid (or dropped to keep the rest legal)" )
                                            : "rejected by the guard: the route would touch another net";

                    // an end fixed copper walls in before any search: which end and what closes it
                    std::string wallWhy;

                    for( const GRID_WALL& w : r && !reached ? r->walls : std::vector<GRID_WALL>() )
                    {
                        if( w.end != a && w.end != b )
                            continue;

                        BOARD_CONNECTED_ITEM* end = w.end == a ? static_cast<BOARD_CONNECTED_ITEM*>( jobs[index].from ) : jobs[index].to;
                        std::string           name = endName( end );
                        nlohmann::json        by = nlohmann::json::array();
                        std::string           list;

                        std::set<std::string>      named;
                        std::vector<GRID_WALL::BY> order = w.by;   // the track planes first: the via plane is mostly the pad itself
                        const int                  reach = gridNets[k].width / 2 + std::max( gridNets[k].clearance, ds.m_MinClearance )
                                                           + gr.cell;

                        std::stable_partition( order.begin(), order.end(), []( const GRID_WALL::BY& c ) { return c.layer >= 0; } );

                        // the copper that closes the most cells round the room
                        for( const nlohmann::json& item : wallClosers( board, w, params.layers, gridNets[k].netCode, reach ) )
                        {
                            const std::string what = item.contains( "pad" )    ? "pad " + item["pad"].get<std::string>()
                                                     : item.contains( "type" ) ? item["net"].get<std::string>() + " " + item["type"].get<std::string>()
                                                                                         + " " + item["uuid"].get<std::string>() + " [" + item.value( "key", std::string() ) + "]"
                                                                               : item["net"].get<std::string>() + " pour";

                            list += ( list.empty() ? "" : ", " ) + what + " (" + mmText( item["distance_mm"].get<double>() ) + " mm)";
                            by.push_back( item );
                        }

                        if( !by.empty() )
                            order.clear();

                        for( const GRID_WALL::BY& c : order )
                        {
                            if( by.size() >= 4 )
                                break;

                            const PCB_LAYER_ID layer = c.layer >= 0 && c.layer < (int) params.layers.size()
                                                               ? params.layers[c.layer]
                                                               : UNDEFINED_LAYER;
                            nlohmann::json item = wallItem( board, c.owner, c.at, layer, gridNets[k].netCode );
                            const std::string id = item.value( "uuid", item.value( "pad", item.value( "zone", item.value( "net", std::string() ) ) ) )
                                                   + ( layer == UNDEFINED_LAYER ? "/via" : "" );

                            if( !named.insert( id ).second )
                                continue;
                            item["blocked_at_mm"] = { toMm( c.at.x ), toMm( c.at.y ) };
                            item["plane"] = layer == UNDEFINED_LAYER ? std::string( "via" ) : str( board->GetLayerName( layer ) );
                            item["cells"] = c.cells;

                            std::string what = item.contains( "pad" )        ? "pad " + item["pad"].get<std::string>()
                                               : item.contains( "type" )     ? item["net"].get<std::string>() + " " + item["type"].get<std::string>()
                                               : item.contains( "zone" )     ? item["net"].get<std::string>() + " pour"
                                               : item.contains( "keep_out" ) ? "keep-out " + item["keep_out"].get<std::string>()
                                               : item.contains( "hole" )     ? "hole " + item["hole"].get<std::string>()
                                               : item.contains( "edge" )     ? std::string( "board edge" )
                                               : item.contains( "net" )      ? item["net"].get<std::string>() + " copper"
                                                                             : std::string( "a keep-out" );

                            if( item.contains( "distance_mm" ) && c.layer >= 0 )
                                what += " (" + mmText( item["distance_mm"].get<double>() ) + " mm off on " + item["plane"].get<std::string>() + ")";
                            else if( c.layer < 0 )
                                what += " (via plane)";

                            list += ( list.empty() ? "" : ", " ) + what;
                            by.push_back( item );
                        }

                        wallWhy = "end " + name + " is walled in before any search: its track has room for " + std::to_string( w.room )
                                  + " grid states only, closed by " + ( list.empty() ? std::string( "nothing named" ) : list )
                                  + " at track width " + mmText( toMm( gridNets[k].width ) ) + " and its clearances";
                        row["blocked_end"] = { { "end", name }, { "room_states", w.room }, { "by", by } };
                        row["blocked_by"] = by;   // the key pns failures use

                        std::set<std::string> movable;

                        for( const nlohmann::json& item : by )
                        {
                            if( item.contains( "type" ) && item.contains( "net" ) && !item["net"].get<std::string>().empty() )
                                movable.insert( item["net"].get<std::string>() );
                        }

                        if( !movable.empty() )
                        {
                            std::string nets;

                            for( const std::string& n : movable )
                                nets += ( nets.empty() ? "" : ", " ) + n;

                            row["blocked_end"]["hint"] = "the tracks of " + nets + " close it: pcb_tracks_delete them and route "
                                                         "them in the same pcb_route call, or reroute them first";
                        }

                        break;
                    }

                    if( !wallWhy.empty() )
                        row["reason"] = wallWhy;

                    if( !reached && wallWhy.empty() && !KopenapiJobCancelled() )
                    {
                        nlohmann::json by = nlohmann::json::array();

                        for( const auto& [e, end, aim] : { std::tuple{ a, static_cast<BOARD_CONNECTED_ITEM*>( jobs[index].from ), jobs[index].to },
                                                           std::tuple{ b, jobs[index].to, static_cast<BOARD_CONNECTED_ITEM*>( jobs[index].from ) } } )
                        {
                            if( r && (size_t) e < r->reached.size() && r->reached[e] )
                                continue;

                            for( const nlohmann::json& item : gridEndBlockers( board, end, aim ? aim->GetPosition() : end->GetPosition(),
                                                                               params.layers, gridNets[k].netCode, gridNets[k].width / 2,
                                                                               endName( end ) ) )
                            {
                                by.push_back( item );
                            }
                        }

                        if( !by.empty() )
                            row["blocked_by"] = by;
                    }

                    // the reason a fine-pitch end cannot be left, with its numbers (before the wall's)
                    if( !reached && !KopenapiJobCancelled() )
                    {
                        for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( jobs[index].from ), jobs[index].to } )
                        {
                            if( !end || end->Type() != PCB_PAD_T || pitchOf( static_cast<PAD*>( end ) ) > pcbIUScale.mmToIU( 0.65 ) )
                                continue;

                            if( const std::string why = padEntryProblem( static_cast<PAD*>( end ), gridNets[k].netCode, gridNets[k].width );
                                !why.empty() )
                            {
                                row["reason"] = why;
                                break;
                            }
                        }
                    }
                }

                if( gridNets[k].width < gridClassWidth[k] )
                    row["necked_from_mm"] = toMm( gridClassWidth[k] );

                done[index] = ok;
                rowOf[index] = (int) results.size();
                results.push_back( row );
            }
        }

        for( const GRID_GAP& gg : gridGaps )
        {
            const GAP&             gap = gaps[gg.gap];
            const GRID_NET_RESULT* r = gg.net < gr.nets.size() ? &gr.nets[gg.net] : nullptr;
            const bool             reached = r && (size_t) std::max( gg.a, gg.b ) < r->reached.size() && r->reached[gg.a] && r->reached[gg.b];
            const bool             ok = reached && netOk[gg.net];
            nlohmann::json         row = { { "net", gap.net }, { "routed", ok }, { "tries", 1 }, { "engine", "grid" },
                                           { "gap", { { "from_mm", { toMm( gap.aPos.x ), toMm( gap.aPos.y ) } },
                                                      { "to_mm", { toMm( gap.bPos.x ), toMm( gap.bPos.y ) } } } } };

            if( ok )
            {
                row["width_mm"] = toMm( gridNets[gg.net].width );
            }
            else
            {
                row["reason"] = !reached ? "no way on the grid between the two pieces of copper"
                                         : "rejected by the guard: the route would touch another net";

                for( const GRID_WALL& w : r && !reached ? r->walls : std::vector<GRID_WALL>() )
                {
                    if( w.end != gg.a && w.end != gg.b )
                        continue;

                    BOARD_CONNECTED_ITEM* end = gridNets[gg.net].ends[w.end];
                    const int             reach = gridNets[gg.net].width / 2 + std::max( gridNets[gg.net].clearance, ds.m_MinClearance ) + gr.cell;
                    const nlohmann::json  by = wallClosers( board, w, params.layers, gridNets[gg.net].netCode, reach );
                    std::string           list;

                    for( const nlohmann::json& item : by )
                        list += ( list.empty() ? "" : ", " ) + ( item.contains( "pad" ) ? "pad " + item["pad"].get<std::string>()
                                                                                       : item.value( "net", std::string() ) + " "
                                                                                                 + item.value( "type", std::string( "pour" ) ) );

                    row["reason"] = "end " + endName( end ) + " is walled in before any search"
                                    + ( list.empty() ? std::string() : ", closed by " + list );
                    row["blocked_end"] = { { "end", endName( end ) }, { "room_states", w.room }, { "by", by } };
                    row["blocked_by"] = by;
                    break;
                }

                if( !reached && !row.contains( "blocked_by" ) && !KopenapiJobCancelled() )
                {
                    nlohmann::json by = nlohmann::json::array();

                    for( const auto& [e, end, aim] : { std::tuple{ gg.a, gridNets[gg.net].ends[gg.a], gap.bPos },
                                                       std::tuple{ gg.b, gridNets[gg.net].ends[gg.b], gap.aPos } } )
                    {
                        if( r && (size_t) e < r->reached.size() && r->reached[e] )
                            continue;

                        for( const nlohmann::json& item : gridEndBlockers( board, end, aim, params.layers, gridNets[gg.net].netCode,
                                                                           gridNets[gg.net].width / 2, endName( end ) ) )
                        {
                            by.push_back( item );
                        }
                    }

                    if( !by.empty() )
                        row["blocked_by"] = by;
                }
            }

            gapTaken[gg.gap] = true;
            results.push_back( row );
        }

        if( gr.timeline )
        {
            const int lane = gr.branchLane >= 0 ? gr.branchLane : gr.timeline->winner;

            if( lane >= 0 && !gr.timeline->Lane( lane ).steps.empty() )
            {
                int    joined = 0, connections = 0, vias = 0;
                double length = 0;

                for( size_t k = 0; k < gridNets.size(); ++k )
                {
                    for( size_t index : jobsOfNet[k] )
                    {
                        connections++;
                        joined += done[index];
                    }

                    if( netOk[k] && k < gr.nets.size() )
                    {
                        vias += netVias[k];
                        length += gr.nets[k].length;
                    }
                }

                const nlohmann::json metrics = { { "routed", joined }, { "connections", connections }, { "vias", vias },
                                                 { "length_mm", toMm( length ) }, { "route_ms", std::round( gr.routeMs ) },
                                                 { "applied", !staged.empty() && !dryRun } };

                if( gr.branchLane >= 0 )
                {
                    const GRID_LANE& b = gr.timeline->Lane( gr.branchLane );
                    timelineOut["nodes"]["branch"] = VariantTreeAutoNode( gr.timeline, b.parent, b.parentStep, "branch",
                                                                          nlohmann::json::object(), nullptr );
                }

                endNode = { gr.timeline, lane, metrics };
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
                    { "half_cell", { { "tried", gr.halfCellTried }, { "won", gr.halfCellWon }, { "ms", std::round( gr.halfCellMs * 10 ) / 10 } } },
                    { "guard_ms", std::round( msSince( guardFrom ) ) }, { "rejected", rejected },
                    { "costliest", costliest }, { "expanded_by_stage", gr.expandedByStage }, { "bundles", bundlesUsed },
                    { "timeline", timelineOut } };
        board->BuildConnectivity();
    }

    // Rip-up and retry: for a connection that failed, take out the routes of this call that cross
    // its corridor, route it, route them again; if anything of that fails, everything goes back
    // as it was (walkaround only: no other net is pushed aside; the copper of the nets it touched
    // is put back from copies)
    const int ripRounds = dryRun ? 0 : std::clamp( aArgs.value( "rip_up", 2 ), 0, 6 );
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
                                  round == 0 ? progressCounts( "retry" ) : progressCounts( "rip_up", round ) ) )
            {
                break;
            }

            if( ripBudgetS > 0 )
                tryDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds( (long) ( ripBudgetS * 1000 ) );

            // first from both ends, nothing taken out
            if( round == 0 )
            {
                nlohmann::json attempt;
                int            tries = 0;

                profile.pass = "retry";

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
                    row.erase( "blocked_end" );
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
                        profile.ripUpsWalled++;
                        return {};
                    }

                    if( !planned->empty() )
                    {
                        profile.ripUpsPlanned++;
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
                // a failed pick puts copies back: the ends may be new objects
                job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
                job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

                if( !job.from || !job.to )
                    break;

                std::vector<size_t> victims = pickVictims();

                if( victims.empty() )
                    break;

                std::sort( victims.begin(), victims.end() );

                // a failed rip-up puts everything back exactly: until another route changes the
                // board, the same one fails again
                if( ripFailed[index].count( { boardVersion, victims } ) )
                {
                    profile.ripUpsSkipped++;
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
                profile.pass = "rip_up";
                bool           ok = tryRoute( job, attempt, tries, false, true );
                nlohmann::json victimAttempts = nlohmann::json::array();
                const char*    failedAt = ok ? nullptr : "route";

                // every route taken out by this rip-up, the second level's too
                std::vector<size_t> allTaken = victims;
                std::vector<size_t> secondTaken;

                for( size_t v : victims )
                {
                    if( ok && overTime() )
                        ok = false, failedAt = KopenapiJobCancelled() ? "stopped" : "time_budget";

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
                            profile.ripUpsSecond++;

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
                    row.erase( "blocked_end" );
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
                profile.ripUpsFailedAt[failedAt]++;

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

    tryDeadline.reset();

    // Gaps between two pieces of a net's copper: from the end of one piece to the other
    profile.pass = "gaps";
    std::vector<long> gapRowOf( gaps.size(), -1 );

    for( size_t gapIndex = 0; gapIndex < gaps.size(); ++gapIndex )
    {
        const GAP& gap = gaps[gapIndex];

        if( !KopenapiJobStep( "gaps", (int) gapIndex, (int) gaps.size(), gap.net, progressCounts( "gaps" ) ) )
            break;

        if( gapTaken[gapIndex] )
            continue;

        gapRowOf[gapIndex] = (long) results.size();
        profile.job = -2 - (long) gapIndex;

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
        profile.pass = "clean_up";
        profile.job = -1;

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

            if( !KopenapiJobStep( "clean_up", (int) cleanIndex, (int) candidates.size(), jobs[c.index].net,
                                  progressCounts( "clean_up" ) ) )
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
            profile.job = (long) c.index;

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

    // a segment of this call laid over another one end to end (two routes of a net, or a rip-up's
    // way back, over the same piece): one copy stays
    int            twins = 0;
    nlohmann::json doubled = nlohmann::json::array();

    if( !dryRun )
    {
        std::map<std::tuple<int, int, int, int64_t, int64_t, int64_t, int64_t>, PCB_TRACK*> seen;
        auto key = []( PCB_TRACK* t )
        {
            VECTOR2I a = t->GetStart(), b = t->GetEnd();

            if( std::tie( b.x, b.y ) < std::tie( a.x, a.y ) )
                std::swap( a, b );

            return std::make_tuple( t->GetNetCode(), (int) t->GetLayer(), t->GetWidth(), (int64_t) a.x, (int64_t) a.y,
                                    (int64_t) b.x, (int64_t) b.y );
        };
        std::set<KIID> mine( added.begin(), added.end() );
        BOARD_COMMIT   drop( context->GetToolManager() );

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() == PCB_TRACE_T && !mine.count( t->m_Uuid ) )
                seen.emplace( key( t ), t );
        }

        for( const KIID& id : added )
        {
            PCB_TRACK* t = dynamic_cast<PCB_TRACK*>( board->ResolveItem( id, true ) );

            if( !t || t->Type() != PCB_TRACE_T || t->IsLocked() )
                continue;

            auto [first, fresh] = seen.emplace( key( t ), t );

            if( !fresh )
            {
                auto who = [&]( PCB_TRACK* aTrack ) -> nlohmann::json
                {
                    auto it = profile.laid.find( aTrack->m_Uuid );

                    if( it == profile.laid.end() )
                        return { { "uuid", aTrack->m_Uuid.AsStdString() }, { "key", copperKey( aTrack ) } };

                    return { { "uuid", aTrack->m_Uuid.AsStdString() }, { "key", copperKey( aTrack ) }, { "pass", it->second.pass },
                             { "attempt", it->second.attempt }, { "from_mm", { toMm( aTrack->GetStart().x ), toMm( aTrack->GetStart().y ) } } };
                };

                doubled.push_back( { { "net", str( t->GetNetname() ) },
                                     { "layer", str( board->GetLayerName( t->GetLayer() ) ) },
                                     { "kept", who( first->second ) },
                                     { "removed", who( t ) } } );
                drop.Remove( t );
                twins++;
            }
        }

        if( twins )
            drop.Push( _( "Remove doubled copper (API)" ) );
        else
            drop.Revert();
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

    for( int net : profile.shovedNets )
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

    if( dryRun )
        openAtEnd = KOPENAPI_UNROUTED{ std::max( 0, openAtStart.connections - routed ), openAtStart.nets, openAtStart.zoneFills };
    else
        openAtEnd = KopenapiUnrouted( board );

    progressPass.clear();
    KopenapiJobProgress( "done", 1, 1, std::string(), progressCounts( "done" ) );

    if( endNode.timeline )
    {
        gridOut["timeline"]["nodes"]["end"] = VariantTreeAutoNode( endNode.timeline, endNode.lane,
                                                                   endNode.timeline->Lane( endNode.lane ).steps.size() - 1,
                                                                   "end", endNode.metrics, board );
    }

    const KOPENAPI_LIST_CAP cap( aArgs, ROUTE_LIST_CAP );

    // full: the copper each row's routes left on the board, with the pass that laid it
    if( cap.Full() && !dryRun )
    {
        std::map<long, nlohmann::json> copperOfJob;

        for( PCB_TRACK* t : board->Tracks() )
        {
            auto it = profile.laid.find( t->m_Uuid );

            if( it == profile.laid.end() || it->second.job == -1 )
                continue;

            nlohmann::json seg = { { "kind", t->Type() == PCB_VIA_T ? "via" : "track" },
                                   { "layer", str( board->GetLayerName( t->GetLayer() ) ) },
                                   { "pass", it->second.pass },
                                   { "attempt", it->second.attempt },
                                   { "uuid", t->m_Uuid.AsStdString() },
                                   { "key", copperKey( t ) } };

            if( t->Type() == PCB_VIA_T )
            {
                seg["at_mm"] = { toMm( t->GetStart().x ), toMm( t->GetStart().y ) };
            }
            else
            {
                seg["from_mm"] = { toMm( t->GetStart().x ), toMm( t->GetStart().y ) };
                seg["to_mm"] = { toMm( t->GetEnd().x ), toMm( t->GetEnd().y ) };
                seg["width_mm"] = toMm( t->GetWidth() );
            }

            copperOfJob[it->second.job].push_back( seg );
        }

        auto rowSegments = [&]( long aRow, long aJob )
        {
            if( aRow < 0 || aRow >= (long) results.size() )
                return;

            auto it = copperOfJob.find( aJob );
            results[aRow]["segments"] = it != copperOfJob.end() ? it->second : nlohmann::json::array();
        };

        for( size_t k = 0; k < jobs.size(); ++k )
            rowSegments( rowOf[k], (long) k );

        for( size_t g = 0; g < gapRowOf.size(); ++g )
            rowSegments( gapRowOf[g], -2 - (long) g );
    }

    if( gridOut.is_object() )
    {
        // each rejected net's violations too (violations_total keeps the count)
        if( gridOut.contains( "rejected" ) && gridOut["rejected"].is_array() )
        {
            for( nlohmann::json& r : gridOut["rejected"] )
            {
                if( r.contains( "violations" ) && r["violations"].is_array() )
                    cap.Put( r, "violations", r["violations"] );
            }
        }

        for( const char* key : { "crowded_at_end", "rejected", "costliest", "bundles" } )
        {
            if( gridOut.contains( key ) )
                cap.Put( gridOut, key, gridOut[key] );
        }
    }

    nlohmann::json answer = { { "connections", connections },
                              { "routed", routed },
                              { "failed", connections - routed },
                              { "length_mm", std::round( length * 1000.0 ) / 1000.0 },
                              { "vias", vias },
                              { "rip_up", { { "tried", rippedTried }, { "won", rippedOk } } },
                              { "clean_up", { { "tried", cleanTried }, { "skipped", cleanSkipped }, { "kept", cleanKept }, { "vias_removed", viasRemoved },
                                              { "shortened_mm", std::round( pcbIUScale.IUTomm( shortenedIU ) * 1000.0 ) / 1000.0 } } },
                              { "stubs_removed", stubs },
                              { "doubled_removed", twins },
                              { "kept_out_under", keptOut },
                              { "grid", gridOut },
                              { "plane_layers_skipped", planeLayers },
                              { "diff_pairs_skipped", pairsSkipped },
                              { "dry_run", dryRun },
                              { "stopped", KopenapiJobCancelled() },
                              { "unrouted_left", openAtEnd->connections },
                              { "unrouted_connections", openAtEnd->connections },
                              { "unrouted_nets", openAtEnd->nets },
                              { "zone_fills", openAtEnd->zoneFills },
                              { "ms", std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() },
                              { "profile", { { "attempts", profile.attempts },
                                             { "dead_legs_skipped", profile.deadLegsSkipped },
                                             { "clearance_lookups", profile.clearanceLookups },
                                             { "rule_evaluations", profile.ruleEvaluations },
                                             { "via_spots_blocked", profile.viaSpotsBlocked },
                                             { "via_spots_off_board", profile.viaSpotsOffBoard },
                                             { "rip_ups_skipped", profile.ripUpsSkipped },
                                             { "rip_ups_planned", profile.ripUpsPlanned },
                                             { "rip_ups_walled", profile.ripUpsWalled },
                                             { "rip_ups_failed_at", profile.ripUpsFailedAt },
                                             { "rip_ups_second", profile.ripUpsSecond },
                                             { "pairs_stuck", profile.pairsStuck },
                                             { "pad_exits", { { "tried", profile.padExitsTried }, { "won", profile.padExitsWon } } },
                                             { "plans", profile.plans },
                                             { "plans_found", profile.plansFound },
                                             { "plans_whole_board", profile.plansWholeBoard },
                                             { "plan_ms", std::round( profile.planMs ) },
                                             { "sync_ms", std::round( profile.syncMs ) },
                                             { "move_ms", std::round( profile.moveMs ) },
                                             { "route_ms", std::round( profile.totalMs - profile.syncMs ) },
                                             { "by_kind", tallies( profile.byKind ) },
                                             { "failed_by", tallies( profile.byFailure ) },
                                             { "by_pass", tallies( profile.byPass ) } } } };

    cap.Put( answer, "other_nets_moved", std::move( moved ) );

    if( !doubled.empty() )
        cap.Put( answer, "doubled", std::move( doubled ) );
    cap.PutPreferring( answer, "results", std::move( results ),
                       []( const nlohmann::json& aRow )
                       {
                           return !aRow.value( "routed", false );
                       } );
    return KOPENAPI_RESULT::Ok( std::move( answer ) );
}


/// @brief Stitch a net's loose ends to its pour on another layer: for every open connection of the net,
/// an end on one side only (an SMD pad, an island of a zone) gets a through via at a free spot
/// next to it (and a short track from a pad); each via passes the net-merge guard and must land
/// inside the board; zones are refilled at the end.
// What a stitching via serves, for the answer: REF.PAD, the track end of an island, or a zone island
static std::string stitchedFor( BOARD_CONNECTED_ITEM* aItem )
{
    if( aItem->Type() == PCB_PAD_T )
        return str( static_cast<PAD*>( aItem )->GetParentFootprint()->GetReference() ) + "."
               + str( static_cast<PAD*>( aItem )->GetNumber() );

    if( aItem->Type() == PCB_TRACE_T || aItem->Type() == PCB_ARC_T )
        return "track island";

    return "zone island";
}


static KOPENAPI_RESULT h_pcb_stitch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*        board = context->GetBoard();
    const wxString netName = wxString::FromUTF8( aArgs.value( "net", std::string( "GND" ) ) );
    NETINFO_ITEM* net = KopenapiFindNet( board, aArgs.value( "net", std::string( "GND" ) ) );

    if( !net )
        return KopenapiNoSuchNet( board, str( netName ) );

    NETCLASS*  netclass = net->GetNetClass();
    const int  viaDia = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : pcbIUScale.mmToIU( 0.8 );
    const int  viaDrill = netclass && netclass->HasViaDrill() ? netclass->GetViaDrill() : pcbIUScale.mmToIU( 0.4 );
    const int  width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : pcbIUScale.mmToIU( 0.3 );

    // under_parts avoid: no via or stub inside a quad package's ring of pads (its exposed pad stays open)
    const std::string underParts = aArgs.value( "under_parts", std::string( "avoid" ) );

    if( underParts != "avoid" && underParts != "allow" )
        return KOPENAPI_RESULT::Error( 400, "under_parts: avoid (no via inside a quad package's ring of pads) or allow" );

    const std::vector<UNDER_PART> underAreas = underParts == "avoid" ? underPartAreas( board ) : std::vector<UNDER_PART>();

    // the part whose body a via at aAt (with its stub from aFrom on aLayer) would sit under, or none
    auto underWhich = [&]( const VECTOR2I& aAt, const std::optional<VECTOR2I>& aFrom, PCB_LAYER_ID aLayer ) -> FOOTPRINT*
    {
        for( const UNDER_PART& under : underAreas )
        {
            if( under.area.Collide( aAt, viaDia / 2 ) )
                return under.fp;

            if( aFrom && aLayer == under.layer && under.area.Collide( SEG( *aFrom, aAt ), width / 2 ) )
                return under.fp;
        }

        return nullptr;
    };

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

        KopenapiKeepUiAlive();
        KopenapiReadPoint();
        board->BuildConnectivity();
        KopenapiKeepUiAlive();
        board->GetConnectivity()->RecalculateRatsnest();
        KopenapiKeepUiAlive();
    };

    refill();

    // the net's planes per layer with priorities resolved (a higher-priority pour of another net
    // over them is not theirs), shrunk so a via's copper lies wholly on the plane
    std::map<PCB_LAYER_ID, SHAPE_POLY_SET> planes;

    for( PCB_LAYER_ID l : board->GetEnabledLayers().CuStack() )
    {
        SHAPE_POLY_SET domain;
        KopenapiNetDomain( board, net->GetNetCode(), l, domain );

        if( !domain.OutlineCount() )
            continue;

        domain.Deflate( viaDia / 2 + board->GetDesignSettings().m_MinClearance, CORNER_STRATEGY::ROUND_ALL_CORNERS,
                        ARC_HIGH_DEF );

        if( domain.OutlineCount() )
            planes[l] = std::move( domain );
    }

    std::vector<KIID> added;
    nlohmann::json    placed = nlohmann::json::array();
    std::set<BOARD_CONNECTED_ITEM*> done;
    int               offPlaneSkipped = 0;

    // Passes: place, refill, then take out the vias the fill does not reach (a plane island too small
    // to keep, a knockout); their pads are tried again at other spots
    struct STITCHED
    {
        BOARD_CONNECTED_ITEM* item;
        KIID                  via;
        KIID                  track;
        VECTOR2I              at;
        PCB_LAYER_ID          layer;   ///< the end's own layer
    };

    // a via stitches when the refilled plane of another layer holds it in a piece that is a plane,
    // not a sliver left between other nets' pours
    const double kMinPlanePiece = pcbIUScale.mmToIU( 1.0 ) * (double) pcbIUScale.mmToIU( 1.0 ) * 2.0;

    auto onPlanePiece = [&]( PCB_VIA* aVia, PCB_LAYER_ID aOwn )
    {
        for( ZONE* zone : board->Zones() )
        {
            if( zone->GetIsRuleArea() || zone->GetNetCode() != net->GetNetCode() )
                continue;

            for( PCB_LAYER_ID l : zone->GetLayerSet().CuStack() )
            {
                if( l == aOwn || !aVia->IsOnLayer( l ) || !zone->HasFilledPolysForLayer( l ) )
                    continue;

                const std::shared_ptr<SHAPE_POLY_SET>& fill = zone->GetFilledPolysList( l );

                for( int o = 0; o < fill->OutlineCount(); ++o )
                {
                    if( !fill->COutline( o ).PointInside( aVia->GetPosition() ) )
                        continue;

                    SHAPE_POLY_SET piece;
                    piece.AddOutline( fill->COutline( o ) );

                    for( int h = 0; h < fill->HoleCount( o ); ++h )
                        piece.AddHole( fill->CHole( o, h ) );

                    if( piece.Area() >= kMinPlanePiece )
                        return true;
                }
            }
        }

        return false;
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

            // between ends the board is consistent: paint, and let waiting readers in
            KopenapiKeepUiAlive();
            KopenapiReadPoint();

            const int            open = KopenapiUnrouted( board, false ).connections;
            const nlohmann::json counts = { { "pass", pass + 1 },
                                            { "vias", placed.size() },
                                            { "not_reached", std::count_if( misses.begin(), misses.end(),
                                                                            [&]( const auto& m ) { return !done.count( m.first ); } ) },
                                            { "unrouted_connections", open },
                                            { "unrouted_left", open } };

            if( !KopenapiJobStep( "stitch", (int) endIndex, (int) ends.size(), stitchedFor( item ), counts ) )
                break;

            if( done.count( item ) && item->Type() != PCB_ZONE_T )
                continue;

            const PCB_LAYER_ID layer = item->Type() == PCB_ZONE_T ? item->GetLayerSet().CuStack().front()
                                                                    : item->GetLayerSet().CuStack().front();
            const bool  onTrack = item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T;
            const BOX2I box = item->Type() == PCB_ZONE_T || onTrack ? BOX2I( pos, VECTOR2I( 0, 0 ) ) : item->GetBoundingBox();
            bool        ok = false;

            // Candidate spots, nearest first: around a pad; inside a zone island (on a grid over the
            // island, the via's copper fully in it)
            std::vector<VECTOR2I>                         candidates;
            std::map<std::pair<int, int>, VECTOR2I>       stubFrom;   // where the stub to a spot starts

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
                // around the pad; for an island of tracks around every pad and track end of it on this layer
                std::vector<std::pair<VECTOR2I, int>> bases = { { pos, std::max( box.GetWidth(), box.GetHeight() ) / 2 } };

                if( onTrack )
                {
                    for( BOARD_CONNECTED_ITEM* other : board->GetConnectivity()->GetConnectedItems( item ) )
                    {
                        if( other->GetNetCode() != net->GetNetCode() || !other->IsOnLayer( layer ) )
                            continue;

                        if( other->Type() == PCB_PAD_T )
                        {
                            const BOX2I pb = other->GetBoundingBox();
                            bases.emplace_back( other->GetPosition(), std::max( pb.GetWidth(), pb.GetHeight() ) / 2 );
                        }
                        else if( other->Type() == PCB_TRACE_T || other->Type() == PCB_ARC_T )
                        {
                            bases.emplace_back( static_cast<PCB_TRACK*>( other )->GetStart(), 0 );
                            bases.emplace_back( static_cast<PCB_TRACK*>( other )->GetEnd(), 0 );
                        }
                    }
                }

                for( const auto& [base, half] : bases )
                {
                    for( int ring = 0; ring < 6; ++ring )
                    {
                        const int reach = half + viaDia / 2 + pcbIUScale.mmToIU( 0.4 ) + ring * pcbIUScale.mmToIU( 0.5 );

                        for( int k = 0; k < 16; ++k )
                        {
                            const double a = k * M_PI / 8;
                            VECTOR2I     at = base + VECTOR2I( KiROUND( reach * std::cos( a ) ), KiROUND( reach * std::sin( a ) ) );
                            at = VECTOR2I( KiROUND( (double) at.x / step ) * step, KiROUND( (double) at.y / step ) * step );
                            candidates.push_back( at );
                            stubFrom[{ at.x, at.y }] = base;
                        }
                    }
                }
            }

            // nearest to the anchor first; for an island the stub length decides (a spot near any of its pads)
            auto stubLength = [&]( const VECTOR2I& at )
            {
                auto it = stubFrom.find( { at.x, at.y } );
                return ( at - ( it == stubFrom.end() ? pos : it->second ) ).EuclideanNorm();
            };

            // a via only where the net's plane is on another layer (priorities resolved): inside a
            // higher-priority pour of another net it would reach nothing or a sliver of the plane
            auto onPlane = [&]( const VECTOR2I& at )
            {
                for( const auto& [l, plane] : planes )
                {
                    if( l != layer && plane.Contains( at ) )
                        return true;
                }

                return false;
            };

            std::map<std::pair<int, int>, bool> planeAt;

            // the 400 best spots (on the plane first, then the shortest stub; ties in candidate
            // order), keys computed once: a zone island yields tens of thousands of spots
            struct SPOT_KEY
            {
                bool   plane;
                double stub;
                size_t index;
            };

            std::vector<SPOT_KEY> keys;
            keys.reserve( candidates.size() );

            for( size_t i = 0; i < candidates.size(); ++i )
            {
                const VECTOR2I& at = candidates[i];
                const bool      plane = planeAt.emplace( std::make_pair( at.x, at.y ), onPlane( at ) ).first->second;
                keys.push_back( { plane, static_cast<double>( stubLength( at ) ), i } );
            }

            const size_t keep = std::min<size_t>( keys.size(), 400 );
            std::partial_sort( keys.begin(), keys.begin() + keep, keys.end(),
                               []( const SPOT_KEY& x, const SPOT_KEY& y )
                               {
                                   if( x.plane != y.plane )
                                       return x.plane;

                                   if( x.stub != y.stub )
                                       return x.stub < y.stub;

                                   return x.index < y.index;
                               } );

            std::vector<VECTOR2I> best;
            best.reserve( keep );

            for( size_t i = 0; i < keep; ++i )
                best.push_back( candidates[keys[i].index] );

            candidates = std::move( best );

            const int offPlaneBefore = offPlaneSkipped;
            std::map<std::string, int> underSkipped;   // spots under a part's body, per part

            {
                for( const VECTOR2I& at : candidates )
                {
                    if( ok )
                        break;

                    if( inner.OutlineCount() && !inner.Contains( at ) )
                        continue;   // too close to the board edge

                    if( !planes.empty() && !planeAt[{ at.x, at.y }] )
                    {
                        offPlaneSkipped++;
                        continue;
                    }

                    {
                        const auto        from = stubFrom.find( { at.x, at.y } );
                        const bool        stub = item->Type() == PCB_PAD_T || onTrack;
                        const FOOTPRINT*  body = underWhich( at, stub ? std::optional<VECTOR2I>( from == stubFrom.end() ? pos : from->second )
                                                                      : std::nullopt, layer );

                        if( body )
                        {
                            underSkipped[str( body->GetReference() )]++;
                            continue;
                        }
                    }

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

                    // a pad, or the end of a track that joins pads into an island without the plane
                    if( item->Type() == PCB_PAD_T || onTrack )
                    {
                        track = std::make_unique<PCB_TRACK>( board );
                        auto from = stubFrom.find( { at.x, at.y } );
                        track->SetStart( from == stubFrom.end() ? pos : from->second );
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
                    KopenapiKeepUiAlive();
                    fresh_vias.push_back( { item, added.end()[hasTrack ? -2 : -1], hasTrack ? added.back() : niluuid, at, layer } );
                    placed.push_back( { { "via", str( fresh_vias.back().via.AsString() ) },
                                        { "at_mm", { toMm( at.x ), toMm( at.y ) } },
                                        { "for", stitchedFor( item ) } } );
                    done.insert( item );
                    ok = true;
                }
            }

            if( !ok )
            {
                int underCount = 0;
                std::string underNames;

                for( const auto& [ref, n] : underSkipped )
                {
                    underCount += n;
                    underNames += ( underNames.empty() ? "" : ", " ) + ref;
                }

                misses[item] = { { "for", stitchedFor( item ) },
                                 { "at_mm", { toMm( pos.x ), toMm( pos.y ) } },
                                 { "reason", !candidates.empty() && underCount > 0 && underCount + offPlaneSkipped - offPlaneBefore >= (int) candidates.size()
                                                     ? "every spot near it lies under the body of " + underNames
                                                               + " (inside its ring of pads; under_parts: allow to place there)"
                                             : candidates.empty() ? "no spot to try (the island is too small for a via)"
                                             : offPlaneSkipped - offPlaneBefore == (int) candidates.size()
                                                     ? "no spot near it over the net's plane on another layer (a "
                                                       "higher-priority pour of another net or a keep-out covers it there)"
                                                     : "no free spot within reach: every spot near it touches "
                                                       "other copper, a hole, a keep-out, the board edge or lies off the net's plane" } };
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

            if( std::any_of( joined.begin(), joined.end(), []( BOARD_CONNECTED_ITEM* i ) { return i->Type() == PCB_ZONE_T; } )
                && ( planes.empty() || onPlanePiece( via, st.layer ) ) )
            {
                continue;
            }

            drop.Remove( via );

            if( BOARD_ITEM* track = board->ResolveItem( st.track, true ) )
                drop.Remove( track );

            bad.insert( { st.at.x, st.at.y } );
            done.erase( st.item );
            misses[st.item] = { { "for", stitchedFor( st.item ) },
                                { "at_mm", { toMm( st.at.x ), toMm( st.at.y ) } },
                                { "reason", "the via there was off the plane after the refill (an island too small to keep, "
                                            "or a sliver of it under 2 mm2)" } };
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
        KopenapiKeepUiAlive();
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

    KopenapiKeepUiAlive();
    showRoutes( aCtx, added );
    KopenapiKeepUiAlive();

    nlohmann::json answer = { { "net", str( netName ) },
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
                                        // the pieces of the net's pours beside the main one per zone and layer, as
                                        // pcb_zone_list / pcb_zones_fill island_list shows them: no via (cut
                                        // off), or a sliver its vias reach
                                        nlohmann::json out = nlohmann::json::array();

                                        for( ZONE* zone : board->Zones() )
                                        {
                                            if( zone->GetNetCode() != net->GetNetCode() || zone->GetIsRuleArea() )
                                                continue;

                                            KopenapiKeepUiAlive();
                                            nlohmann::json            pieces = KopenapiZoneIslands( board, zone );
                                            std::map<std::string, size_t> mainOf;

                                            for( size_t i = 0; i < pieces.size(); ++i )
                                            {
                                                const std::string l = pieces[i].value( "layer", std::string() );
                                                auto              it = mainOf.find( l );

                                                if( it == mainOf.end() || pieces[i].value( "area_mm2", 0.0 ) > pieces[it->second].value( "area_mm2", 0.0 ) )
                                                    mainOf[l] = i;
                                            }

                                            for( size_t i = 0; i < pieces.size(); ++i )
                                            {
                                                if( mainOf[pieces[i].value( "layer", std::string() )] == i )
                                                    continue;

                                                pieces[i]["zone"] = str( zone->GetZoneName() );
                                                pieces[i]["kind"] = pieces[i].value( "vias", 0 ) == 0 ? "cut_off" : "sliver";
                                                out.push_back( pieces[i] );
                                            }
                                        }

                                        return out;
                                    }() } };

    const KOPENAPI_LIST_CAP cap( aArgs );
    nlohmann::json          out = { { "net", answer["net"] },
                                    { "vias_added", answer["vias"].size() },
                                    { "dangling_vias_removed", danglingRemoved },
                                    { "not_reached_count", answer["not_reached"].size() },
                                    { "islands_left_count", answer["islands_left"].size() },
                                    { "spots_off_plane", offPlaneSkipped } };

    for( const char* key : { "vias", "not_reached", "islands_left" } )
        cap.Put( out, key, answer[key] );

    KopenapiKeepUiAlive();
    out.update( KopenapiUnroutedJson( board ) );
    return KOPENAPI_RESULT::Ok( out );
}


namespace
{

/// @brief Compact figures of a bulk change of tracks / vias: per net (most items first, capped), per
/// layer, and the box around them
void bulkSummary( nlohmann::json& aOut, const KOPENAPI_LIST_CAP& aCap, const std::vector<PCB_TRACK*>& aItems,
                  const std::map<std::string, nlohmann::json>& aByNet )
{
    std::vector<std::pair<std::string, nlohmann::json>> nets( aByNet.begin(), aByNet.end() );

    auto items = []( const nlohmann::json& n ) { return n.value( "tracks", 0 ) + n.value( "vias", 0 ); };

    std::stable_sort( nets.begin(), nets.end(),
                      [&]( const auto& a, const auto& b ) { return items( a.second ) > items( b.second ); } );

    for( auto& [name, n] : nets )
    {
        if( n.contains( "length_mm" ) )
            n["length_mm"] = std::round( n["length_mm"].get<double>() * 1000 ) / 1000;
    }

    aCap.PutObject( aOut, "by_net", nets );

    nlohmann::json byLayer = nlohmann::json::object();
    BOX2I          box;
    bool           first = true;

    for( PCB_TRACK* t : aItems )
    {
        const std::string layer = t->Type() == PCB_VIA_T ? std::string( "via" )
                                                         : str( t->GetBoard()->GetLayerName( t->GetLayer() ) );
        nlohmann::json&   l = byLayer[layer];

        if( l.is_null() )
            l = t->Type() == PCB_VIA_T ? nlohmann::json{ { "count", 0 } }
                                       : nlohmann::json{ { "count", 0 }, { "length_mm", 0.0 } };

        l["count"] = l["count"].get<int>() + 1;

        if( t->Type() != PCB_VIA_T )
            l["length_mm"] = std::round( ( l["length_mm"].get<double>() + toMm( t->GetLength() ) ) * 1000 ) / 1000;

        if( first )
            box = t->GetBoundingBox();
        else
            box.Merge( t->GetBoundingBox() );

        first = false;
    }

    aOut["by_layer"] = byLayer;

    if( !first )
        aOut["bbox_mm"] = { toMm( box.GetLeft() ), toMm( box.GetTop() ), toMm( box.GetRight() ), toMm( box.GetBottom() ) };
}


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

        const std::vector<BOARD_ITEM*> tracks( aBoard->Tracks().begin(), aBoard->Tracks().end() );

        for( const nlohmann::json& u : aArgs["uuids"] )
        {
            std::string error;
            BOARD_ITEM* hit = u.is_string() ? KopenapiMatchUuid( tracks, u.get<std::string>(), error ) : nullptr;

            if( !hit && error.find( "ambiguous" ) != std::string::npos )
            {
                aError = KOPENAPI_RESULT::Error( 400, "uuids: " + error );
                return {};
            }

            if( !hit )
                aMissing.push_back( u );
            else
                out.push_back( static_cast<PCB_TRACK*>( hit ) );
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
        NETINFO_ITEM* net = n.is_string() ? KopenapiFindNet( aBoard, n.get<std::string>() ) : nullptr;

        if( !net )
        {
            aError = n.is_string() ? KopenapiNoSuchNet( aBoard, n.get<std::string>() )
                                   : KOPENAPI_RESULT::Error( 400, "nets: net names" );
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
    const KOPENAPI_LIST_CAP  cap( aArgs );
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
                           { "nets", byNet.size() }, { "applied", false } };
    bulkSummary( out, cap, doomed, byNet );
    cap.Put( out, "missing", missing );
    out["locked_skipped_count"] = locked.size();
    cap.Put( out, "locked_skipped", locked );

    if( cut )
        out["cut"] = cutCount;

    if( doomed.empty() )
    {
        out["reason"] = !locked.empty() ? "everything selected is locked (override_locks: true)"
                        : aArgs.contains( "between" ) ? "no copper joins these two pads"
                                                      : "nothing matches";
        return KOPENAPI_RESULT::Ok( out );
    }

    {
        nlohmann::json uuids = nlohmann::json::array();

        for( PCB_TRACK* track : doomed )
            uuids.push_back( str( track->m_Uuid.AsString() ) );

        cap.Put( out, "uuids", std::move( uuids ) );
    }

    if( dryRun )
        return KOPENAPI_RESULT::Ok( out );

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
    out.update( KopenapiUnroutedJson( board ) );
    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_tracks_cleanup( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD*             board = context->GetBoard();
    std::set<int>      netCodes;
    nlohmann::json     unknown = nlohmann::json::array();

    for( const char* key : { "net", "nets" } )
    {
        if( !aArgs.contains( key ) )
            continue;

        std::vector<std::string> names = aArgs[key].is_array() ? aArgs[key].get<std::vector<std::string>>()
                                                               : std::vector<std::string>{ aArgs[key].get<std::string>() };

        for( const std::string& name : names )
        {
            if( NETINFO_ITEM* net = KopenapiFindNet( board, name ) )
                netCodes.insert( net->GetNetCode() );
            else
                unknown.push_back( name );
        }
    }

    if( !unknown.empty() )
        return KopenapiNoSuchNet( board, unknown[0].get<std::string>() );

    const bool dryRun = aArgs.value( "dry_run", false );

    // dangling ends are judged by the connectivity, and the fills are part of it: a zone changed
    // since its fill (or never filled) would leave vias it will reach looking dangling
    bool refilledFirst = false;

    if( aArgs.value( "dangling_vias", true ) || aArgs.value( "dangling_tracks", true ) )
    {
        nlohmann::json    stale = nlohmann::json::array();
        const std::string fills = KopenapiZoneFillState( board, &stale );

        if( fills == "unfilled" || fills == "stale" )
        {
            const bool refill = aArgs.value( "refill_first", !dryRun );

            if( !refill )
            {
                return KOPENAPI_RESULT::Error( 409, "zone fills are not current (" + stale.dump()
                                                            + "): dangling copper would be judged against them; "
                                                              "pcb_zones_fill first, or refill_first: true" );
            }

            BOARD_COMMIT fillCommit( context->GetToolManager() );
            ZONE_FILLER  filler( board, &fillCommit );
            KOPENAPI_KEEPALIVE_REPORTER reporter;
            filler.SetProgressReporter( &reporter );
            std::vector<ZONE*> zones( board->Zones().begin(), board->Zones().end() );

            if( filler.Fill( zones ) )
                fillCommit.Push( _( "Fill zones (API)" ) );
            else
                fillCommit.Revert();

            refilledFirst = true;
        }
    }

    board->BuildConnectivity();

    BOARD_COMMIT                               commit( context->GetToolManager() );
    TRACKS_CLEANER                             cleaner( board, commit );
    std::vector<std::shared_ptr<CLEANUP_ITEM>> found;

    if( !netCodes.empty() )
    {
        cleaner.SetFilter( [&]( BOARD_CONNECTED_ITEM* aItem ) { return !netCodes.count( aItem->GetNetCode() ); } );
    }

    cleaner.CleanupBoard( dryRun, &found, aArgs.value( "shorting", false ), aArgs.value( "redundant_vias", true ),
                          aArgs.value( "merge_segments", false ), aArgs.value( "dangling_tracks", true ),
                          aArgs.value( "tracks_in_pads", false ), aArgs.value( "dangling_vias", true ) );

    static const std::map<int, std::string> kinds = {
        { CLEANUP_SHORTING_TRACK, "shorting_track" }, { CLEANUP_SHORTING_VIA, "shorting_via" },
        { CLEANUP_REDUNDANT_VIA, "redundant_via" },   { CLEANUP_DUPLICATE_TRACK, "duplicate_track" },
        { CLEANUP_MERGE_TRACKS, "merged_tracks" },    { CLEANUP_DANGLING_TRACK, "dangling_track" },
        { CLEANUP_DANGLING_VIA, "dangling_via" },     { CLEANUP_ZERO_LENGTH_TRACK, "zero_length_track" },
        { CLEANUP_TRACK_IN_PAD, "track_in_pad" } };

    nlohmann::json counts = nlohmann::json::object();
    nlohmann::json items = nlohmann::json::array();
    std::vector<PCB_TRACK*>               touched;
    std::map<std::string, nlohmann::json> byNet;

    for( const std::shared_ptr<CLEANUP_ITEM>& item : found )
    {
        auto        it = kinds.find( item->GetErrorCode() );
        std::string kind = it != kinds.end() ? it->second : str( item->GetErrorText( -1, false ) );
        counts[kind] = counts.value( kind, 0 ) + 1;
        nlohmann::json entry = { { "kind", kind }, { "uuid", str( item->GetMainItemID().AsString() ) } };

        if( BOARD_ITEM* b = board->ResolveItem( item->GetMainItemID(), true ) )
        {
            const std::string netName = b->IsConnected() ? str( static_cast<BOARD_CONNECTED_ITEM*>( b )->GetNetname() ) : "";
            entry["net"] = netName;
            entry["at_mm"] = { pcbIUScale.IUTomm( b->GetPosition().x ), pcbIUScale.IUTomm( b->GetPosition().y ) };

            if( PCB_TRACK* t = dynamic_cast<PCB_TRACK*>( b ) )
            {
                touched.push_back( t );
                nlohmann::json& n = byNet[str( UnescapeString( t->GetNetname() ) )];

                if( n.is_null() )
                    n = { { "tracks", 0 }, { "vias", 0 }, { "length_mm", 0.0 } };

                if( t->Type() == PCB_VIA_T )
                    n["vias"] = n["vias"].get<int>() + 1;
                else
                {
                    n["tracks"] = n["tracks"].get<int>() + 1;
                    n["length_mm"] = n["length_mm"].get<double>() + toMm( t->GetLength() );
                }
            }
        }

        items.push_back( entry );
    }

    const KOPENAPI_LIST_CAP cap( aArgs );
    nlohmann::json          out = { { "dry_run", dryRun }, { "counts", counts }, { "found", found.size() } };

    if( refilledFirst )
        out["zones_refilled_first"] = true;
    bulkSummary( out, cap, touched, byNet );
    cap.Put( out, "items", std::move( items ) );

    if( dryRun || commit.Empty() )
        return KOPENAPI_RESULT::Ok( out );

    commit.Push( _( "Board Cleanup (API)" ) );

    if( aArgs.value( "refill_zones", false ) )
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

    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();
    showRoutes( aCtx, {} );

    out["applied"] = true;
    out.update( KopenapiUnroutedJson( board ) );
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
        NETINFO_ITEM* named = aArgs["net"].is_string() ? KopenapiFindNet( board, aArgs["net"].get<std::string>() )
                                                       : nullptr;

        if( !named )
            return aArgs["net"].is_string() ? KopenapiNoSuchNet( board, aArgs["net"].get<std::string>() )
                                            : KOPENAPI_RESULT::Error( 400, "net: a net name" );

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

        // a via alone keeps its net (connectivity would give it the net of a pour it stands in)
        if( steps.size() == 1 && !fromPad )
            static_cast<PCB_VIA*>( made.back().get() )->SetIsFree();

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
        KOPENAPI_RESULT rejected = KOPENAPI_RESULT::Error( 422, "rejected by the guard: this copper would touch another net or enter a keep-out" );
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
    out.update( KopenapiUnroutedJson( board ) );
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
/// @brief Fan-out of a package with its pads on a ring (QFN / QFP / SOIC / LGA): per pin a short stub
/// straight out from its side and a via just outside the pad row.  Where the vias do not fit side by
/// side at the pitch they stagger over rows; on each side the pin nearest the way its far ends lie
/// takes the nearest row, so the tracks leaving the vias towards them do not cross the other vias.
/// Exposed pads are left to their own vias.  vias false: the stubs only, out past the courtyard.
static KOPENAPI_RESULT peripheralFanout( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, BOARD* aBoard, FOOTPRINT* aFp,
                                         const nlohmann::json& aArgs )
{
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    const bool                   dryRun = aArgs.value( "dry_run", false );
    const bool                   withVias = aArgs.value( "vias", true );
    const std::string            zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (pour fills give way, refilled after) or keep" );

    std::set<int> onlyNets;

    for( const nlohmann::json& n : aArgs.value( "nets", nlohmann::json::array() ) )
    {
        if( !n.is_string() )
            continue;

        NETINFO_ITEM* net = KopenapiFindNet( aBoard, n.get<std::string>() );

        if( !net )
            return KopenapiNoSuchNet( aBoard, n.get<std::string>(), "nets: " );

        onlyNets.insert( net->GetNetCode() );
    }

    std::vector<PAD*> pads;

    for( PAD* pad : aFp->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::SMD && pad->IsOnCopperLayer() )
            pads.push_back( pad );
    }

    if( pads.size() < 4 )
        return KOPENAPI_RESULT::Error( 422, "fewer than 4 SMD pads: nothing to fan out" );

    BOX2I centres( pads.front()->GetPosition(), VECTOR2I( 0, 0 ) );

    for( PAD* pad : pads )
        centres.Merge( pad->GetPosition() );

    // the side of each pad: the edge of the centres' box it lies on (pads not on an edge: inside)
    enum SIDE { LEFT, RIGHT, TOP, BOTTOM, INSIDE };
    const int tol = pcbIUScale.mmToIU( 0.05 );

    auto sideOf = [&]( PAD* aPad ) -> SIDE
    {
        const VECTOR2I c = aPad->GetPosition();
        const BOX2I    pb = aPad->GetBoundingBox();
        const bool     tall = pb.GetHeight() < pb.GetWidth();   // a pad long in x sits on a left / right row

        if( std::abs( c.x - centres.GetLeft() ) <= tol && tall )
            return LEFT;
        if( std::abs( c.x - centres.GetRight() ) <= tol && tall )
            return RIGHT;
        if( std::abs( c.y - centres.GetTop() ) <= tol && !tall )
            return TOP;
        if( std::abs( c.y - centres.GetBottom() ) <= tol && !tall )
            return BOTTOM;
        if( std::abs( c.x - centres.GetLeft() ) <= tol )
            return LEFT;
        if( std::abs( c.x - centres.GetRight() ) <= tol )
            return RIGHT;
        if( std::abs( c.y - centres.GetTop() ) <= tol )
            return TOP;
        if( std::abs( c.y - centres.GetBottom() ) <= tol )
            return BOTTOM;

        return INSIDE;
    };

    static const VECTOR2I out[4] = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
    static const char*    sideName[5] = { "left", "right", "top", "bottom", "inside" };

    std::map<int, std::vector<PAD*>> bySide;

    for( PAD* pad : pads )
        bySide[sideOf( pad )].push_back( pad );

    // pitch along the rows: the median gap to the next pad on the same side
    std::vector<int> gaps;

    for( auto& [side, list] : bySide )
    {
        if( side == INSIDE )
            continue;

        const bool alongX = side == TOP || side == BOTTOM;
        std::sort( list.begin(), list.end(), [&]( PAD* a, PAD* b )
                   { return alongX ? a->GetPosition().x < b->GetPosition().x : a->GetPosition().y < b->GetPosition().y; } );

        for( size_t k = 1; k < list.size(); ++k )
            gaps.push_back( (int) ( list[k]->GetPosition() - list[k - 1]->GetPosition() ).EuclideanNorm() );
    }

    if( gaps.empty() )
        return KOPENAPI_RESULT::Error( 422, "no pad rows found (the pads do not line up on the package's sides)" );

    std::nth_element( gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end() );
    const int pitch = std::max( gaps[gaps.size() / 2], 1 );

    NETCLASS* defaultClass = bds.m_NetSettings ? bds.m_NetSettings->GetDefaultNetclass().get() : nullptr;
    const int clearance = std::max( { bds.m_MinClearance, defaultClass && defaultClass->HasClearance() ? defaultClass->GetClearance() : 0, 1 } );
    const int classDia = defaultClass && defaultClass->HasViaDiameter() ? defaultClass->GetViaDiameter() : pcbIUScale.mmToIU( 0.6 );
    const int classDrill = defaultClass && defaultClass->HasViaDrill() ? defaultClass->GetViaDrill() : pcbIUScale.mmToIU( 0.3 );
    const int viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.6 ) )
                                                           : std::max( bds.m_ViasMinSize, classDia );
    const int viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.3 ) )
                                                          : std::max( bds.m_MinThroughDrill, classDrill );
    // two via lines take half the spread one line needs, when a via plus clearance is wider than the pitch
    const int autoRows = viaDia + clearance > pitch ? 2 : 1;
    const int rows = aArgs.contains( "stagger" ) ? std::clamp( aArgs.value( "stagger", autoRows ), 1, 4 ) : autoRows;

    // the far end of a pad: the nearest pad of its net on another footprint
    auto farEnd = [&]( PAD* aPad ) -> std::optional<VECTOR2I>
    {
        std::optional<VECTOR2I> best;
        double                  bestGap = std::numeric_limits<double>::max();

        for( FOOTPRINT* other : aBoard->Footprints() )
        {
            if( other == aFp )
                continue;

            for( PAD* p : other->Pads() )
            {
                if( p->GetNetCode() != aPad->GetNetCode() )
                    continue;

                const double gap = ( p->GetPosition() - aPad->GetPosition() ).EuclideanNorm();

                if( gap < bestGap )
                {
                    bestGap = gap;
                    best = p->GetPosition();
                }
            }
        }

        return best;
    };

    const SHAPE_POLY_SET courtyard = aFp->GetCourtyard( aFp->GetLayer() == B_Cu ? B_CrtYd : F_CrtYd );

    BOARD_COMMIT                                       commit( aContext.GetToolManager() );
    std::vector<KIID>                                  added;
    std::vector<BOARD_CONNECTED_ITEM*>                 staged;
    std::vector<std::unique_ptr<BOARD_CONNECTED_ITEM>> planned;
    nlohmann::json                                     placed = nlohmann::json::array();
    nlohmann::json                                     skipped = nlohmann::json::array();
    std::map<std::string, int>                         why;

    auto skip = [&]( PAD* aPad, const std::string& aWhy )
    {
        skipped.push_back( { { "pad", str( aFp->GetReference() ) + "." + str( aPad->GetNumber() ) }, { "reason", aWhy } } );
        why[aWhy]++;
    };

    for( auto& [sideKey, list] : bySide )
    {
        const SIDE side = (SIDE) sideKey;

        if( side == INSIDE )
        {
            for( PAD* pad : list )
                skip( pad, "inside the ring (exposed pad): its own vias" );

            continue;
        }

        const VECTOR2I dir = out[side];
        const VECTOR2I tangent( dir.y != 0 ? 1 : 0, dir.x != 0 ? 1 : 0 );

        auto along = [&]( const VECTOR2I& aAt ) { return (int64_t) aAt.x * tangent.x + (int64_t) aAt.y * tangent.y; };
        auto across = [&]( const VECTOR2I& aAt ) { return (int64_t) aAt.x * dir.x + (int64_t) aAt.y * dir.y; };

        std::vector<PAD*>                       todo;
        std::map<PAD*, std::optional<VECTOR2I>> fars;

        for( PAD* pad : list )
        {
            if( pad->GetNetCode() <= 0 )
            {
                skip( pad, "no net" );
                continue;
            }

            if( !onlyNets.empty() && !onlyNets.count( pad->GetNetCode() ) )
                continue;

            fars[pad] = farEnd( pad );

            if( !fars[pad] )
            {
                skip( pad, "no other pad on its net" );
                continue;
            }

            todo.push_back( pad );
        }

        if( todo.empty() )
            continue;

        // the stubs leave as a fan of rays from the pads' outer ends, in pin order (a bus leaves
        // uncrossed), spreading the vias to a pitch where they fit; with several via lines the
        // neighbours alternate lines and the spread only has to pass a stub between two vias
        const int64_t mid = ( along( list.front()->GetPosition() ) + along( list.back()->GetPosition() ) ) / 2;
        int64_t       tipLine = std::numeric_limits<int64_t>::min();
        int64_t       maxOffset = 0;
        NETCLASS*     netclass = todo.front()->GetEffectiveNetClass();
        const int     classWidth = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : bds.m_TrackMinWidth;
        int           width = aArgs.contains( "width_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "width_mm", 0.15 ) )
                                                           : std::min( classWidth, (int) ( 0.95 * pitch ) - clearance );

        width = std::max( width, std::max( bds.m_TrackMinWidth, 1 ) );

        for( PAD* pad : list )
        {
            const BOX2I pb = pad->GetBoundingBox();
            tipLine = std::max( tipLine, across( pad->GetPosition() ) + ( dir.x != 0 ? pb.GetWidth() : pb.GetHeight() ) / 2 );
            maxOffset = std::max( maxOffset, std::abs( along( pad->GetPosition() ) - mid ) );
        }

        const double need = withVias ? ( rows > 1 ? viaDia / 2.0 + clearance + width / 2.0 : double( viaDia + clearance ) )
                                     : double( width + clearance );
        const double spread = std::max( 1.0, need / pitch );
        const double cosMax = std::min( 0.999, std::max( 0.2, double( width + clearance ) / pitch ) );
        const double tanMax = std::sqrt( 1.0 - cosMax * cosMax ) / cosMax;
        const int64_t rayLength = std::max<int64_t>( clearance, KiROUND( maxOffset * ( spread - 1.0 ) / tanMax ) );
        const int64_t rayEnd = tipLine + rayLength;

        int64_t toCourtyard = rayEnd;

        if( courtyard.OutlineCount() )
        {
            const BOX2I cb = courtyard.BBox();
            toCourtyard = std::max( toCourtyard, std::max( across( cb.GetOrigin() ), across( cb.GetEnd() ) ) + clearance + width / 2 );
        }

        auto point = [&]( int64_t aAlong, int64_t aAcross )
        {
            return VECTOR2I( (int) ( aAlong * tangent.x + aAcross * dir.x ), (int) ( aAlong * tangent.y + aAcross * dir.y ) );
        };

        for( PAD* pad : todo )
        {
            const VECTOR2I     at = pad->GetPosition();
            const PCB_LAYER_ID layer = pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
            const int64_t      offset = along( at ) - mid;
            const int64_t      fanned = mid + KiROUND( offset * spread );
            const size_t       index = std::find( list.begin(), list.end(), pad ) - list.begin();
            const int          first = (int) ( index % rows );
            std::string        last = "every row's spot touches other copper";
            bool               done = false;

            const VECTOR2I knee = point( along( at ), tipLine );
            const VECTOR2I ray = point( fanned, rayEnd );

            // the preferred line first, then the others, then further out
            std::vector<int> tries;

            for( int r = 0; r < rows + 2; ++r )
                tries.push_back( r < rows ? ( first + r ) % rows : r );

            for( int r : tries )
            {
                const int64_t  reach = withVias ? rayEnd + r * ( viaDia + clearance ) : toCourtyard;
                const VECTOR2I spot = point( fanned, reach );

                std::unique_ptr<PCB_VIA> via;

                if( withVias )
                {
                    via = std::make_unique<PCB_VIA>( aBoard );
                    via->SetViaType( VIATYPE::THROUGH );
                    via->SetPosition( spot );
                    via->SetWidth( PADSTACK::ALL_LAYERS, viaDia );
                    via->SetDrill( viaDrill );
                    via->SetLayerPair( F_Cu, B_Cu );
                    via->SetNet( pad->GetNet() );
                }

                std::vector<std::unique_ptr<PCB_TRACK>> legs;

                for( const auto& [p0, p1] : { std::pair{ at, knee }, std::pair{ knee, ray }, std::pair{ ray, spot } } )
                {
                    if( p0 == p1 )
                        continue;

                    auto leg = std::make_unique<PCB_TRACK>( aBoard );
                    leg->SetStart( p0 );
                    leg->SetEnd( p1 );
                    leg->SetWidth( width );
                    leg->SetLayer( layer );
                    leg->SetNet( pad->GetNet() );
                    legs.push_back( std::move( leg ) );
                }

                std::vector<BOARD_CONNECTED_ITEM*> fresh;

                for( const auto& leg : legs )
                    fresh.push_back( leg.get() );

                if( via )
                    fresh.push_back( via.get() );

                const nlohmann::json violations = guard( aBoard, fresh, {}, zoneMode == "refill", &staged );

                if( !violations.empty() )
                {
                    last = "the spots out from it touch other copper (" + violations[0]["other"].dump() + ")";

                    if( !withVias )
                        break;

                    continue;
                }

                nlohmann::json row = { { "pad", str( aFp->GetReference() ) + "." + str( pad->GetNumber() ) },
                                       { "net", str( UnescapeString( pad->GetNetname() ) ) },
                                       { "side", sideName[side] },
                                       { "end_mm", { toMm( spot.x ), toMm( spot.y ) } },
                                       { "far_end_mm", { toMm( fars[pad]->x ), toMm( fars[pad]->y ) } } };

                if( withVias )
                    row["row"] = r;

                placed.push_back( row );

                for( BOARD_CONNECTED_ITEM* item : fresh )
                    staged.push_back( item );

                for( auto& leg : legs )
                {
                    if( !dryRun )
                    {
                        added.push_back( leg->m_Uuid );
                        commit.Add( leg.release() );
                    }
                    else
                    {
                        planned.push_back( std::move( leg ) );
                    }
                }

                if( via )
                {
                    if( !dryRun )
                    {
                        added.push_back( via->m_Uuid );
                        commit.Add( via.release() );
                    }
                    else
                    {
                        planned.push_back( std::move( via ) );
                    }
                }

                done = true;
                break;
            }

            if( !done )
                skip( pad, last.substr( 0, 400 ) );
        }
    }

    if( !dryRun && !added.empty() )
    {
        commit.Push( _( "Fan out (API)" ) );

        if( zoneMode == "refill" )
            refillPours( aContext, aBoard );

        aBoard->BuildConnectivity();
        aBoard->GetConnectivity()->RecalculateRatsnest();
        showRoutes( aCtx, added );
    }
    else
    {
        commit.Revert();
    }

    nlohmann::json reasons = nlohmann::json::object();

    for( const auto& [w, n] : why )
        reasons[w] = n;

    nlohmann::json answer = { { "ref", str( aFp->GetReference() ) },
                              { "package", "peripheral" },
                              { "pitch_mm", toMm( pitch ) },
                              { "pattern", "fan" },
                              { "stagger_rows", withVias ? rows : 0 },
                              { "fanned", placed.size() },
                              { "skipped", skipped.size() },
                              { "skipped_by_reason", reasons },
                              { "dry_run", dryRun },
                              { "applied", !dryRun && !added.empty() } };

    if( withVias )
        answer["via"] = { { "diameter_mm", toMm( viaDia ) }, { "drill_mm", toMm( viaDrill ) } };

    const KOPENAPI_LIST_CAP cap( aArgs, 20 );
    cap.Put( answer, "pins", placed );
    cap.Put( answer, "skipped_pins", skipped );
    return KOPENAPI_RESULT::Ok( answer );
}


/// @brief The way into the board from a connector whose pads sit on the board edge (card-edge
/// fingers): the axis direction away from the edge most of its pads are next to, or none.
static std::optional<VECTOR2I> edgeInward( BOARD* aBoard, const std::vector<PAD*>& aPads )
{
    SHAPE_POLY_SET outline;

    if( aPads.empty() || !aBoard->GetBoardPolygonOutlines( outline, false ) || !outline.OutlineCount() )
        return std::nullopt;

    const int                  nearby = pcbIUScale.mmToIU( 1.0 );
    std::map<std::pair<int, int>, int> votes;
    int                        touching = 0, connectorPads = 0;

    for( PAD* pad : aPads )
    {
        const VECTOR2I c = pad->GetPosition();
        VECTOR2I       best;
        int64_t        bestDist = std::numeric_limits<int64_t>::max();

        if( pad->GetAttribute() == PAD_ATTRIB::CONN )
            connectorPads++;

        for( int o = 0; o < outline.OutlineCount(); ++o )
        {
            const VECTOR2I p = outline.Outline( o ).NearestPoint( c );
            const int64_t  d = ( p - c ).SquaredEuclideanNorm();

            if( d < bestDist )
                bestDist = d, best = p;
        }

        const VECTOR2I d = c - best;
        const BOX2I    pb = pad->GetBoundingBox();
        // to the edge from the pad's side facing it, along the axis the edge lies on
        const bool     alongY = std::abs( d.y ) >= std::abs( d.x );
        const int      gap = ( alongY ? std::abs( d.y ) - pb.GetHeight() / 2 : std::abs( d.x ) - pb.GetWidth() / 2 );

        if( gap > nearby )
            continue;

        touching++;
        votes[alongY ? std::pair{ 0, d.y >= 0 ? 1 : -1 } : std::pair{ d.x >= 0 ? 1 : -1, 0 }]++;
    }

    if( votes.empty() || ( touching * 2 < (int) aPads.size() && connectorPads * 2 < (int) aPads.size() ) || touching < 2 )
        return std::nullopt;

    auto best = std::max_element( votes.begin(), votes.end(), []( const auto& a, const auto& b ) { return a.second < b.second; } );
    return VECTOR2I( best->first.first, best->first.second );
}


/// @brief Fan a single row of pads out (card-edge fingers, a header): per pad a straight stub from its
/// centre in one direction (inward from the board edge by default), on the pad's own layer, past
/// the pad end; an optional via at its end (one line, or two lines the neighbours alternate over).
/// A stub or via that would touch other copper (the twin finger's via on the other side, a keep-out)
/// tries the other line, then a half-pitch jog.
static KOPENAPI_RESULT edgeFanout( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, BOARD* aBoard, FOOTPRINT* aFp,
                                   const std::vector<PAD*>& aPads, VECTOR2I aDir, const nlohmann::json& aArgs )
{
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    const bool                   dryRun = aArgs.value( "dry_run", false );
    const std::string            zoneMode = aArgs.value( "zones", std::string( "refill" ) );
    std::string                  viaMode = aArgs.value( "via", std::string( "none" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (pour fills give way, refilled after) or keep" );

    if( aArgs.contains( "vias" ) && !aArgs.value( "vias", true ) )
        viaMode = "none";

    if( viaMode != "none" && viaMode != "always" && viaMode != "stagger" )
        return KOPENAPI_RESULT::Error( 400, "via: none (stubs only), always (a via at every stub end) or stagger "
                                            "(the neighbours alternate two via lines)" );

    std::set<int> onlyNets;

    for( const nlohmann::json& n : aArgs.value( "nets", nlohmann::json::array() ) )
    {
        if( !n.is_string() )
            continue;

        NETINFO_ITEM* net = KopenapiFindNet( aBoard, n.get<std::string>() );

        if( !net )
            return KopenapiNoSuchNet( aBoard, n.get<std::string>(), "nets: " );

        onlyNets.insert( net->GetNetCode() );
    }

    const VECTOR2I tangent( aDir.y != 0 ? 1 : 0, aDir.x != 0 ? 1 : 0 );

    auto along = [&]( const VECTOR2I& aAt ) { return (int64_t) aAt.x * tangent.x + (int64_t) aAt.y * tangent.y; };

    // pitch: the median gap to the next pad along the row on the same layer
    std::vector<PAD*> row = aPads;
    std::sort( row.begin(), row.end(), [&]( PAD* a, PAD* b ) { return along( a->GetPosition() ) < along( b->GetPosition() ); } );
    std::vector<int64_t> gaps;

    for( size_t i = 0; i < row.size(); ++i )
    {
        for( size_t j = i + 1; j < row.size(); ++j )
        {
            if( KopenapiPadNeighbours( row[i], row[j] ) )
            {
                gaps.push_back( std::abs( along( row[j]->GetPosition() ) - along( row[i]->GetPosition() ) ) );
                break;
            }
        }
    }

    std::nth_element( gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end() );
    const int pitch = gaps.empty() ? pcbIUScale.mmToIU( 2.54 ) : (int) std::max<int64_t>( gaps[gaps.size() / 2], 1 );

    NETCLASS* defaultClass = bds.m_NetSettings ? bds.m_NetSettings->GetDefaultNetclass().get() : nullptr;
    const int classDia = defaultClass && defaultClass->HasViaDiameter() ? defaultClass->GetViaDiameter() : pcbIUScale.mmToIU( 0.6 );
    const int classDrill = defaultClass && defaultClass->HasViaDrill() ? defaultClass->GetViaDrill() : pcbIUScale.mmToIU( 0.3 );
    const int viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.6 ) )
                                                           : std::max( bds.m_ViasMinSize, classDia );
    const int viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.3 ) )
                                                          : std::max( bds.m_MinThroughDrill, classDrill );
    const std::optional<int> stubGiven = aArgs.contains( "stub_mm" )
                                                 ? std::optional<int>( pcbIUScale.mmToIU( aArgs.value( "stub_mm", 0.0 ) ) )
                                                 : std::nullopt;

    if( stubGiven && *stubGiven <= 0 )
        return KOPENAPI_RESULT::Error( 400, "stub_mm: a length from the pad centre, > 0" );

    BOARD_COMMIT                                       commit( aContext.GetToolManager() );
    std::vector<KIID>                                  added;
    std::vector<BOARD_CONNECTED_ITEM*>                 staged;
    std::vector<std::unique_ptr<BOARD_CONNECTED_ITEM>> planned;
    nlohmann::json                                     placed = nlohmann::json::array();
    nlohmann::json                                     skipped = nlohmann::json::array();
    std::map<std::string, int>                         why;
    std::shared_ptr<CONNECTIVITY_DATA>                 conn = aBoard->GetConnectivity();

    auto padName = [&]( PAD* aPad ) { return str( aFp->GetReference() ) + "." + str( aPad->GetNumber() ); };

    auto skip = [&]( PAD* aPad, const std::string& aWhy, const std::string& aKind )
    {
        skipped.push_back( { { "pad", padName( aPad ) }, { "reason", aWhy } } );
        why[aKind]++;
    };

    auto otherPad = [&]( PAD* aPad )
    {
        for( FOOTPRINT* other : aBoard->Footprints() )
        {
            if( other == aFp )
                continue;

            for( PAD* p : other->Pads() )
            {
                if( p->GetNetCode() == aPad->GetNetCode() )
                    return true;
            }
        }

        return false;
    };

    std::map<PCB_LAYER_ID, int> indexOnLayer;

    for( PAD* pad : row )
    {
        const PCB_LAYER_ID layer = pad->IsOnLayer( F_Cu ) ? F_Cu : pad->IsOnLayer( B_Cu ) ? B_Cu : pad->GetPrincipalLayer();
        const int          index = indexOnLayer[layer]++;

        if( pad->GetNetCode() <= 0 )
        {
            skip( pad, "no net", "no net" );
            continue;
        }

        if( !onlyNets.empty() && !onlyNets.count( pad->GetNetCode() ) )
            continue;

        if( !otherPad( pad ) )
        {
            skip( pad, "no other pad on its net", "no other pad on its net" );
            continue;
        }

        bool hasCopper = false;

        for( BOARD_CONNECTED_ITEM* item : conn->GetConnectedItems( pad ) )
            hasCopper |= item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T || item->Type() == PCB_VIA_T;

        if( hasCopper )
        {
            skip( pad, "a track or via already leaves it", "already has copper" );
            continue;
        }

        NETCLASS*      nc = pad->GetEffectiveNetClass();
        const int      clearance = netClearance( pad, std::max( bds.m_MinClearance, 1 ) );
        const int      width = aArgs.contains( "width_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "width_mm", 0.25 ) )
                                                            : std::max( nc && nc->HasTrackWidth() ? nc->GetTrackWidth() : bds.m_TrackMinWidth,
                                                                        std::max( bds.m_TrackMinWidth, 1 ) );
        const VECTOR2I at = pad->GetPosition();
        const BOX2I    pb = pad->GetBoundingBox();
        const int      half = ( aDir.x != 0 ? pb.GetWidth() : pb.GetHeight() ) / 2;
        const bool     withVia = viaMode != "none";
        const int      stub = stubGiven ? *stubGiven : half + clearance + ( withVia ? viaDia / 2 : width / 2 );
        const int      step = viaDia + clearance;
        const int      jog = pitch / 2;
        const int      first = viaMode == "stagger" ? index % 2 : 0;

        // (via line, sideways jog): the preferred line straight, the other line, then a half-pitch jog
        std::vector<std::pair<int, int>> tries;

        if( !withVia )
            tries = { { 0, 0 } };
        else
            tries = { { first, 0 }, { 1 - first, 0 }, { first, jog }, { first, -jog }, { 1 - first, jog }, { 1 - first, -jog }, { 2, 0 } };

        std::string last;
        bool        done = false;

        for( const auto& [line, side] : tries )
        {
            // a jog leaves the pad centre on a 45 degree line to half a pitch aside (clear of the twin's
            // via on the other side), then goes straight on
            if( side != 0 && stub <= std::abs( side ) )
                continue;

            const VECTOR2I spot = at + tangent * side + aDir * ( stub + line * step );
            const VECTOR2I knee = side != 0 ? at + tangent * side + aDir * std::abs( side ) : spot;

            std::vector<std::unique_ptr<BOARD_CONNECTED_ITEM>> items;

            for( const auto& [p0, p1] : { std::pair{ at, knee }, std::pair{ knee, spot } } )
            {
                if( p0 == p1 )
                    continue;

                auto leg = std::make_unique<PCB_TRACK>( aBoard );
                leg->SetStart( p0 );
                leg->SetEnd( p1 );
                leg->SetWidth( width );
                leg->SetLayer( layer );
                leg->SetNet( pad->GetNet() );
                items.push_back( std::move( leg ) );
            }

            if( withVia )
            {
                auto via = std::make_unique<PCB_VIA>( aBoard );
                via->SetViaType( VIATYPE::THROUGH );
                via->SetPosition( spot );
                via->SetWidth( PADSTACK::ALL_LAYERS, viaDia );
                via->SetDrill( viaDrill );
                via->SetLayerPair( F_Cu, B_Cu );
                via->SetNet( pad->GetNet() );
                items.push_back( std::move( via ) );
            }

            std::vector<BOARD_CONNECTED_ITEM*> fresh;

            for( const auto& item : items )
                fresh.push_back( item.get() );

            const nlohmann::json violations = guard( aBoard, fresh, {}, zoneMode == "refill", &staged );

            if( !violations.empty() )
            {
                const nlohmann::json& o = violations[0]["other"];
                last = o.contains( "pad" )       ? "pad " + o["pad"].get<std::string>()
                       : o.contains( "keepout" ) ? "keep-out " + ( o["keepout"].get<std::string>().empty() ? o.value( "key", std::string() )
                                                                                                           : o["keepout"].get<std::string>() )
                       : o.contains( "zone" )    ? o.value( "net", std::string() ) + " pour"
                                                 : o.value( "net", std::string() ) + " " + o.value( "type", std::string( "copper" ) );

                if( o.contains( "from_mm" ) || o.contains( "at_mm" ) )
                    last += " " + o.value( "uuid", std::string() ) + " [" + o.value( "key", std::string() ) + "]";

                continue;
            }

            nlohmann::json out = { { "pad", padName( pad ) },
                                   { "net", str( UnescapeString( pad->GetNetname() ) ) },
                                   { "layer", str( aBoard->GetLayerName( layer ) ) },
                                   { "stub_end_mm", { toMm( spot.x ), toMm( spot.y ) } },
                                   { "width_mm", toMm( width ) } };

            if( withVia )
            {
                out["via_mm"] = { toMm( spot.x ), toMm( spot.y ) };
                out["row"] = line;
            }

            if( side != 0 )
                out["jog_mm"] = toMm( side );

            placed.push_back( out );

            for( BOARD_CONNECTED_ITEM* item : fresh )
                staged.push_back( item );

            for( auto& item : items )
            {
                if( !dryRun )
                {
                    added.push_back( item->m_Uuid );
                    commit.Add( item.release() );
                }
                else
                {
                    planned.push_back( std::move( item ) );
                }
            }

            done = true;
            break;
        }

        if( !done )
            skip( pad, "the stub" + std::string( withVia ? " or its via" : "" ) + " would touch " + last.substr( 0, 300 ),
                  withVia ? "the stub or its via touches other copper or a keep-out" : "the stub touches other copper or a keep-out" );
    }

    if( !dryRun && !added.empty() )
    {
        commit.Push( _( "Fan out (API)" ) );

        if( zoneMode == "refill" )
            refillPours( aContext, aBoard );

        aBoard->BuildConnectivity();
        aBoard->GetConnectivity()->RecalculateRatsnest();
        showRoutes( aCtx, added );
    }
    else
    {
        commit.Revert();
    }

    nlohmann::json reasons = nlohmann::json::object();

    for( const auto& [w, n] : why )
        reasons[w] = n;

    static const std::map<std::pair<int, int>, const char*> dirName = {
        { { 0, -1 }, "up" }, { { 0, 1 }, "down" }, { { -1, 0 }, "left" }, { { 1, 0 }, "right" } };

    nlohmann::json answer = { { "ref", str( aFp->GetReference() ) },
                              { "package", "edge" },
                              { "direction", dirName.at( { aDir.x, aDir.y } ) },
                              { "pitch_mm", toMm( pitch ) },
                              { "pattern", "stub" },
                              { "via", viaMode },
                              { "fanned", placed.size() },
                              { "skipped", skipped.size() },
                              { "skipped_by_reason", reasons },
                              { "dry_run", dryRun },
                              { "applied", !dryRun && !added.empty() } };

    if( viaMode != "none" )
        answer["via_size"] = { { "diameter_mm", toMm( viaDia ) }, { "drill_mm", toMm( viaDrill ) } };

    const KOPENAPI_LIST_CAP cap( aArgs, 20 );
    cap.Put( answer, "pins", placed );
    cap.Put( answer, "skipped_pins", skipped );
    return KOPENAPI_RESULT::Ok( answer );
}


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

    // a row of pads on the board edge (card-edge fingers), or a direction given: straight stubs
    std::vector<PAD*> copperPads;

    for( PAD* pad : fp->Pads() )
    {
        if( pad->GetAttribute() != PAD_ATTRIB::NPTH && pad->IsOnCopperLayer() )
            copperPads.push_back( pad );
    }

    const std::string direction = aArgs.value( "direction", std::string() );
    std::optional<VECTOR2I> inward;

    if( direction == "up" )
        inward = VECTOR2I( 0, -1 );
    else if( direction == "down" )
        inward = VECTOR2I( 0, 1 );
    else if( direction == "left" )
        inward = VECTOR2I( -1, 0 );
    else if( direction == "right" )
        inward = VECTOR2I( 1, 0 );
    else if( !direction.empty() && direction != "inward" )
        return KOPENAPI_RESULT::Error( 400, "direction: up, down, left, right or inward (away from the board edge the pads sit on)" );
    else
        inward = edgeInward( board, copperPads );

    if( direction == "inward" && !inward )
        return KOPENAPI_RESULT::Error( 422, "direction inward: the pads of " + str( fp->GetReference() )
                                                    + " are not next to the board edge; give up, down, left or right" );

    if( inward )
        return edgeFanout( aCtx, *context, board, fp, copperPads, *inward, aArgs );

    std::vector<PAD*> balls;

    for( PAD* pad : fp->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::SMD )
            balls.push_back( pad );
    }

    if( balls.size() < 9 )
        return peripheralFanout( aCtx, *context, board, fp, aArgs );

    // the pitch: the median distance to the nearest ball
    std::vector<double> nearest;

    for( PAD* a : balls )
    {
        double best = std::numeric_limits<double>::max();

        for( PAD* b : balls )
        {
            if( KopenapiPadNeighbours( a, b ) )
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
        return peripheralFanout( aCtx, *context, board, fp, aArgs );

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
        probe.SetFlags( ROUTER_TRANSIENT );
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
        {
            if( !KopenapiFindNet( board, n.get<std::string>() ) )
                return KopenapiNoSuchNet( board, n.get<std::string>(), "nets: " );

            onlyNets.insert( str( UnescapeString( KopenapiFindNet( board, n.get<std::string>() )->GetNetname() ) ) );
        }
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
            probe.SetFlags( ROUTER_TRANSIENT );
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

    nlohmann::json answer = { { "ref", str( fp->GetReference() ) },
                              { "pitch_mm", toMm( pitch ) },
                              { "via", { { "diameter_mm", toMm( viaDia ) }, { "drill_mm", toMm( viaDrill ) } } },
                              { "outer_rings_direct", direct },
                              { "fanned", placed.size() },
                              { "outward", outward },
                              { "skipped", skipped.size() },
                              { "skipped_by_reason", reasons },
                              { "applied", !dryRun && !added.empty() } };
    const KOPENAPI_LIST_CAP cap( aArgs, 20 );
    cap.Put( answer, "balls", placed );
    cap.Put( answer, "skipped_balls", skipped );
    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_route_connection",
                   "EXPERIMENTAL. Route one connection with KiCad's push-and-shove router, as a person "
                   "does in the editor: from a pad (REF.PAD) or a spot on its net's copper (a track end, a via) "
                   "to a pad of the same net, a spot (its net's copper there, else a free spot: one leg out of a "
                   "crowded pin field to a chosen place), or the nearest unconnected point of its net; layer, mode walkaround / shove; track width (width_mm to set) and "
                   "clearance from the board's rules; waypoints steer it (corners, vias with a layer change). A guard "
                   "rejects routes whose copper would touch another net. Undo one route: pcb_tracks_delete "
                   "(between its pads). "
                   "dry_run: measure only (geometry, length, detour, vias, shoved items), nothing changes",
                   R"json({"type":"object","required":["from"],"properties":{
                        "from":{"description":"REF.PAD, or a spot on copper of a net: [x_mm, y_mm] or {at: [x_mm, y_mm], layer} (a track end, a via, a pad)"},
                        "to":{"description":"REF.PAD on the same net, or a spot [x_mm, y_mm] / {at, layer}: the net's copper there (track end, via, pad) or a free spot the route ends at (with layer: a via there when the route arrives on another); default: nearest unconnected"},
                        "layer":{"type":"string","description":"start layer; default the 'from' pad's (F.Cu, B.Cu for a pad on the back only). A 'to' on another layer than the route arrives on gets a via in front of it (via_near_target_mm)"},
                        "mode":{"type":"string","enum":["walkaround","shove"],"default":"walkaround"},
                        "waypoints":{"type":"array","description":"points to pass in order: [x_mm, y_mm] fixes a corner there; {at: [x_mm, y_mm], layer: 'B.Cu'} drops a through via there and continues on that layer","items":{}},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills give way while routing and are refilled around the route; keep: the route must clear the fills"},
                        "width_mm":{"type":"number","description":"track width of this route (default the net class's), e.g. narrower to leave a fine-pitch pad"},
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
                   "job_cancel stops it and keeps what is done (pns: the connections routed so far; grid: the "
                   "nets legal in the round it stopped in, even an unconverged one, committed as one step); "
                   "job_cancel {apply: false} (or keep: true) leaves the board as the call found it, in whatever pass "
                   "it stops (applied: false; taken_back when copper already written, the grid's included, was "
                   "taken out again; the grid timeline stays to branch from); the retry / rip-up pass gives each "
                   "connection rip_up_budget_s and a cancel stops it between router tries; "
                   "every route passes the net-merge guard; copper of this call that ends nowhere is removed "
                   "(stubs_removed); copper of nets outside the call that shove pushed aside is listed "
                   "(other_nets_moved: pieces and length before / after); a row still routed at the end is "
                   "joined by copper; per connection "
                   "routed / reason and ms (every try for it), totals (routed, length, vias, unrouted left) and "
                   "profile (attempts; clearance_lookups / rule_evaluations: the router's clearances past its caches and the design "
                   "rule evaluations they took; dead_legs_skipped / via_spots_blocked / rip_ups_skipped / rip_ups_walled: tries not made; "
                   "rip_ups_planned: rip-ups whose routes a plan chose; rip_ups_second: routes taken out that had no way back "
                   "and moved others in turn; rip_ups_failed_at: failed rip-ups by what failed (route, victim, cut); a row left "
                   "failed lists its rip_ups_failed (taken_out, failed_at); "
                   "pairs_stuck: pair tries ended when nothing was placed after 3 steps; pad_exits: starts stuck in their pad that tried / won a corner out of a free side (row left_pad_by_mm); plans / plans_found / plan_ms: global plans (plans_whole_board: searched again over the whole board when the box round the ends cut the way); sync_ms: building the router's world for each; move_ms: the path search; "
                   "route_ms: all but the world; by_kind: tries / routed / ms on a layer (walkaround, shove), "
                   "through one via, two vias; failed_by: failed tries by reason; by_pass: first, retry from both ends, "
                   "rip_up, gaps); "
                   "dry_run routes nothing; rows (results, failed ones first when cut), other_nets_moved and the grid lists capped at list_limit (counts whole), full: true for all",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"string"}},
                        "layers":{"type":"array","items":{"type":"string"},"default":["F.Cu","B.Cu"]},
                        "order":{"type":"string","enum":["short_first","power_first","dense_first"],"default":"short_first","description":"dense_first: connections at fine-pitch pads (<= 0.65 mm) first"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills out while routing (they would block every crossing), refilled after; keep: routes must clear the fills"},
                        "clean_up":{"type":"boolean","default":true,"description":"after routing, each connection with vias or a detour (over 1.3 x the straight line) is routed again alone on a layer both ends are on, without moving anything; kept with fewer vias, or as many and 5 % shorter (a plan over the free space of those layers first: none, or no shorter, skips it; clean_up: tried / skipped / kept / vias_removed / shortened_mm; rows cleaned)"},
                        "rip_up_budget_s":{"type":"number","default":30,"minimum":0,"description":"seconds the retry / rip-up pass gives one connection (its tries and the routes it took out going back); spent: the rip-up fails and what it took out goes back; 0 no limit"},
                        "rip_up":{"type":"integer","default":2,"minimum":0,"maximum":6,"description":"rounds of rip-up and retry for failed connections (routes of this call in their way taken out and routed again, up to 3 choices of them per connection and round; all back if that fails)"},
                        "vias":{"type":"boolean","default":true,"description":"connections blocked on their side: escape through vias near both ends"},
                        "diff_pairs":{"type":"string","enum":["auto","off"],"default":"auto","description":"auto: pairs (USB_D+ / USB_D-, LVDS_P / LVDS_N) routed coupled; off: every net alone"},
                        "engine":{"type":"string","enum":["pns","grid"],"default":"pns","description":"grid: every net at once on one grid of the board, negotiated congestion (PathFinder) in parallel variants, the best legal result committed (diff-pair trunks still by pns, their rows say engine pns; no rip_up / clean_up passes; with dry_run the same search and guard, rows / totals / grid answered, nothing put on the board, the fills counted out as zones: refill would, the trunks not routed: their rows say not_simulated with the reason and the pairs' airwires are in the grid search; with from: 400; a fine-pitch end the grid walls in tries a lattice half a cell off at its pad: row half_cell_exits, grid.half_cell {tried, won, ms}); pns: one connection at a time with the push-and-shove router"},
                        "max_rounds":{"type":"integer","default":40,"description":"grid: negotiation rounds before the nets still crowding each other are routed apart or left out"},
                        "cell_mm":{"type":"number","default":0,"description":"grid: the cell, 0 a third of the finest track pitch (width + clearance) on a 0.025 mm step"},
                        "via_keepout_mm":{"type":"number","default":5,"description":"grid: only a chip's own nets put vias this far around it (pads on four sides, or an array outside its balls) while the board routes; other nets left short and, once all are joined, the via-saving pass may use it; 0 off"},
                        "bundles":{"description":"grid: \"auto\" (pcb_net_roles' bundles), \"off\", or [{name, nets}] in member order: a bundle's nets between the same parts are routed first, side by side (the answer's grid.bundles: name, parts, nets)","default":"auto"},
                        "via_away_mm":{"type":"number","default":7,"description":"grid: a signal via nearer a chip's pads (not an array's) than this costs more, linearly to the pads; 0 off"},
                        "via_away_cost_mm":{"type":"number","default":8,"description":"grid: a via at a chip's pads costs as much more as this length of track"},
                        "rows_cost":{"type":"number","default":6,"description":"grid: a track between the two pin rows of a 2-row connector (a header) costs this many times more: pins leave the rows outward, rails run outside the row; 1 off"},
                        "via_near_hole_mm":{"type":"number","default":2.5,"description":"grid: a via nearer a through-hole pad or via of its own net than this costs more, linearly to the hole (the hole is on every layer: start on the layer needed there); 0 off"},
                        "via_near_hole_cost_mm":{"type":"number","default":6,"description":"grid: a via at its own net's through hole costs as much more as this length of track"},
                        "hug":{"type":"number","default":1,"description":"grid: cost factor of a cell beside the copper of the net's own bundle (1: no pull; below 1 so far lowers cohesion, adds vias and time)"},
                        "astar":{"type":"number","default":1.2,"description":"grid: weight of the distance left in the search (1: cheapest paths; more: fewer states searched, paths a little longer)"},
                        "pf_start":{"type":"number","default":1.0,"description":"grid: price of sharing a cell in the first negotiation round"},
                        "pf_growth":{"type":"number","default":1.6,"description":"grid: how much that price grows each round (more: fewer rounds, longer detours)"},
                        "record":{"type":"boolean","default":false,"description":"grid: keep every variant's steps for pcb_route_timeline"},
                        "from":{"type":"object","description":"grid, a branch: the recorded lane goes on after a step as a new lane of the same run (pcb_route_timeline status: parent, parent_step), with the run's own arguments; the copper the run (or its last branch) put on the board is rolled back first when that is the last edit, else the board must be as the run found it (409). Up to the step the recorded nets are taken without searching, so the branch's steps equal the recorded ones until it searches on. A grid route stopped with job_cancel {keep: true} goes on from a stopped lane's last step. With warm: true over the board as edited since. {node}: from a pcb_variant_tree node (its run, lane and step; warm when its lane went on over an edited board); the answer's timeline.nodes names the branch and end nodes","properties":{"node":{"type":"integer","description":"a pcb_variant_tree node instead of run / lane / step"},"run":{"type":"integer","description":"a kept run; default the one recorded or branched last"},"lane":{"description":"index or variant name; default the lane on the board"},"step":{"type":"integer"}}},
                        "warm":{"type":"boolean","description":"with from: go on over the board as edited since (a part moved, a keep-out drawn) instead of the run's: the step's copper, history and rounds are taken as they are, the nets whose copper the edit broke (over new copper or keep-outs, pads moved) or that are not joined lose it and are routed first in a new round, the rest keep theirs (timeline.branch.warm: round, affected nets and why). The grid, layers, sizes and nets must be the run's, else a new run is routed cold (timeline.cold: why). The run's copper on the board under later edits is taken out first, the edits kept (warm.copper_removed)"},
                        "history":{"type":"string","enum":["keep","clear","clear_edited"],"description":"warm: the history of places fought over kept (default), started empty, or cleared where the edit changed the board (and 1 mm around; timeline.branch.warm.edited_tiles)"},
                        "schedule":{"type":"string","enum":["fixed","adaptive","adaptive_hist"],"default":"fixed","description":"grid, EXPERIMENTAL: negotiation schedule of the built-in variants (adaptive: growth 1.3 while the crowded nets fall, 2.0 when they stall; adaptive_hist: also twice the history on places fought over 3 rounds running)"},
                        "strategies":{"type":"array","description":"grid, EXPERIMENTAL: the variants to race instead of the built-ins: 'short_first_hv@1.0x1.6/a1.5/d0.9/r2', 'fine_first@adaptive', or {order, directions, pf_start, pf_growth, astar, schedule, hist_decay, reroute_recent}","items":{}},
                        "hist_decay":{"type":"number","default":1.0,"description":"grid, EXPERIMENTAL: the history cost of places not fought over in a round is multiplied by this (1: kept for good)"},
                        "reroute_recent":{"type":"integer","default":2,"description":"grid: a net that crowds nobody is routed again only through places fought over in the last this many rounds (0: through any place ever fought over)"},
                        "variants":{"type":"integer","default":0,"description":"grid: how many of the built-in variants (pcb_route_strategies) run in parallel; 0 one per two cores, as many as half the available memory holds (grid.lane_mb, grid.lanes_memory_allows)"},
                        "plane_layers":{"type":"boolean","default":false,"description":"route on plane layers too (type power in the stack-up, or an inner layer more than half of which one net's pours cover): skipped by default, a track would cut the plane; the answer lists plane_layers_skipped"},
                        "under_parts":{"type":"string","enum":["allow","avoid"],"default":"allow","description":"avoid: no track or via of this call (grid and pns) inside the ring of pads of a quad package (QFP / QFN / LGA, pads on four sides) on its side - tracks there can wall its other pins in (blocked_by names them); with an exposed pad the ring between the pad rows and the exposed pad is kept out (the exposed pad and its clearance stay open); BGAs stay open; the answer lists kept_out_under. Costs routes on a crowded board (TC-13: 77 -> 70 of 81)"},
                        "neck_down":{"oneOf":[{"type":"string","enum":["auto","off"]},{"type":"object","additionalProperties":{"type":"string","enum":["auto","off"]}}],"default":"auto","description":"auto: a connection with an end on a fine-pitch pad (0.65 mm or closer) is routed no wider than that pad's narrow side (not below the Default class width or the board minimum) when its class is wider; the rest of its net at its class width (grid: narrow only inside the courtyard of the fine-pitch footprint (the pad entry), the rest of the net widened to its class width where clearance allows (else neck_width_mm stays there and the row says narrow_outside_mm); a custom track_width rule at the pad sets the neck lower, its opt else min, not below the board minimum); the row says necked_from_mm. Per net: {\"VBUS\": \"off\", \"*\": \"auto\"}"},
                        "escapes":{"type":"boolean","default":true,"description":"the way out of each fine-pitch pad (0.65 mm or closer) on the edge of its package is kept open while its connection waits: room for a track and a via in front of it; a route of another net across it is refused (reason names the pad). false: no such room kept"},
                        "plan":{"type":"boolean","default":true,"description":"a connection the router does not find on a layer is planned over the free space (grid search with vias, the board's clearances), the router follows the plan; no plan found: no escape-spot tries either (row plan tells)"},
                        "max_tries":{"type":"integer","default":120,"minimum":1,"description":"router attempts per connection and pass (layers x modes x escape spots); a big board pays about a second per attempt"},
                        "style":{"type":"string","enum":["direct","rail"],"default":"direct","description":"rail (power nets): every connection leaves its pads through a via next to them and runs on the second layer"},
                        "step_ms":{"type":"integer","default":0,"description":"GUI: draw each route and wait this long before the next (for watching / recording)"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json, ROUTE_LIST_CAP ),
                   false, h_pcb_route, 600 );


KOPENAPI_REGISTER( "pcb_stitch",
                   "Stitch a net (default GND) to its pours and planes: every open end on one side only (SMD "
                   "pad, zone island, the track end of pads joined by tracks) gets a through via at a free spot "
                   "next to it (with a short track from a pad or track end), guarded against touching other nets (other nets' fills give way: they are refilled), "
                   "inside the board; after the refill a via the fill does not reach is taken out and its pad "
                   "tried at another spot (up to 3 passes); unlocked vias of the net left touching one layer "
                   "only are removed. On inner planes stitch before routing the signals (the spots next to the "
                   "pads are still free), with outer-layer pours both before and after; answers the vias, "
                   "dangling_vias_removed, not_reached (pads / islands left without a via, and why), islands_left (pieces of the "
                   "net's pours beside the main one per zone and layer, as island_list shows them: kind cut_off (no via) or sliver; "
                   "vias go only where the net's plane is on another layer with zone priorities resolved, and a via left on a "
                   "piece under 2 mm2 after the refill is taken out; footprint and board keep-outs are honoured; spots_off_plane) and the "
                   "connections still open (unrouted); lists capped at list_limit (counts whole), full: true for all",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{"net":{"type":"string","default":"GND"},
                        "under_parts":{"type":"string","enum":["avoid","allow"],"default":"avoid","description":"avoid: no via or stub inside the ring of pads of a quad package (QFP / QFN / LGA) on its side - the exposed pad and its clearance stay open for its own vias; BGAs are not affected; not_reached says when only such spots were near"}}})json"_json ),
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
                   "(override_locks). dry_run tells what would go. Answers counts (tracks, vias, length), per net "
                   "(most first) and per layer, the box, the first uuids (list_limit, full: true for all) and the "
                   "connections now open (unrouted); route again with pcb_route / pcb_route_connection / "
                   "pcb_track_add",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
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
                        "dry_run":{"type":"boolean","default":false}}})json"_json ),
                   false, h_pcb_tracks_delete, 300 );

KOPENAPI_REGISTER( "pcb_tracks_cleanup",
                   "Board cleanup as KiCad's Cleanup Tracks & Vias, in one undo step: dangling tracks and vias "
                   "(ends that reach nothing, repeated until none is left), vias inside a pad or another via, "
                   "and on request shorting copper, collinear segments merged, tracks inside pads; net / nets "
                   "limit it; zone fills that are not current are refilled first (refill_first). Answers counts per kind, per net and per layer, the box, the first items (uuid, "
                   "net, where; list_limit, full: true for all); dry_run changes nothing",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "net":{"type":"string"},"nets":{"type":"array","items":{"type":"string"}},
                        "dangling_tracks":{"type":"boolean","default":true},
                        "dangling_vias":{"type":"boolean","default":true},
                        "redundant_vias":{"type":"boolean","default":true},
                        "shorting":{"type":"boolean","default":false,"description":"remove tracks and vias joining two nets"},
                        "merge_segments":{"type":"boolean","default":false},
                        "tracks_in_pads":{"type":"boolean","default":false},
                        "refill_zones":{"type":"boolean","default":false},
                        "refill_first":{"type":"boolean","description":"zones not filled or changed since their fill are refilled before dangling copper is judged (default true; with dry_run default false: 409 naming the zones)"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json ),
                   false, h_pcb_tracks_cleanup, 300 );

KOPENAPI_REGISTER( "pcb_track_add",
                   "Lay a track by hand along given points (no router): from a pad and / or to a pad (their "
                   "centres) through points in mm; a point with via continues on that layer through a via. "
                   "Width and via size from the net's class unless width_mm is given. The same guard as the "
                   "router: copper that would touch another net or enter a keep-out is refused (422 with the "
                   "violations), nothing is added then. One undo step; dry_run checks only",
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
                   "(via spot, ring, outward) and the skipped ones with the reason (lists cut to list_limit, "
                   "default 20; full: true for all); then pcb_route from the vias. A package with its pads on a ring "
                   "(QFN / QFP / SOIC / LGA): per side a fan of stubs, in pin order (a bus leaves uncrossed), out "
                   "from the pad tips to through vias spread to a pitch where they fit; with stagger 2+ the "
                   "neighbours alternate via lines (auto 2 when a via plus clearance is wider than the pitch); "
                   "a blocked spot tries the other lines, then further out; exposed pads are skipped (their own "
                   "vias); vias false: stubs only, out past the courtyard (escapes for pcb_route); answers "
                   "package peripheral, pattern fan, pins (pad, net, side, row, end, far end). A row of pads on the "
                   "board edge (card-edge fingers on F.Cu / B.Cu, found by pads next to Edge.Cuts) or any part "
                   "with direction given: per pad of a net with other pads a straight stub from the pad centre "
                   "inward (or up / down / left / right) on the pad's own layer, stub_mm long (default: past the "
                   "pad end plus clearance), via none (default) / always / stagger (neighbours alternate two "
                   "lines); a blocked spot tries the other line, then a half-pitch jog; pads that already have "
                   "a track or via are left alone; answers package edge, pattern stub, pins (pad, net, layer, "
                   "stub end, via, row, jog)",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string"},
                        "nets":{"type":"array","items":{"type":"string"},"description":"only these nets"},
                        "outer_rings_direct":{"type":"integer","minimum":0},
                        "via_diameter_mm":{"type":"number"},"via_drill_mm":{"type":"number"},
                        "width_mm":{"type":"number"},
                        "stagger":{"type":"integer","minimum":1,"maximum":4,"description":"ring packages: via lines the neighbours alternate over (default 2 when a via plus clearance is wider than the pitch, else 1)"},
                        "vias":{"type":"boolean","default":true,"description":"ring packages: false gives the stubs only, ending just outside the courtyard"},
                        "direction":{"type":"string","enum":["inward","up","down","left","right"],"description":"stubs straight this way (edge mode); inward: away from the board edge the pads sit on (found by itself for card-edge fingers)"},
                        "stub_mm":{"type":"number","description":"edge mode: stub length from the pad centre"},
                        "via":{"type":"string","enum":["none","always","stagger"],"default":"none","description":"edge mode: a via at each stub end, stagger: on two alternating lines"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json, 20 ),
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

KOPENAPI_MARK_EDITING( "pcb_route_connection", "pcb_route", "pcb_stitch", "pcb_tracks_delete", "pcb_tracks_cleanup", "pcb_track_add", "pcb_via_add", "pcb_fanout" );
KOPENAPI_MARK_CONCURRENT_READS( "pcb_track_list" );
