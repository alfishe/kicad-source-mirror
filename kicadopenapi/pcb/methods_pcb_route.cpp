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
#include "rail_plan.h"

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_memory.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <project/net_settings.h>
#include <geometry/direction45.h>
#include <geometry/shape_circle.h>
#include <geometry/shape_segment.h>
#include <connectivity/connectivity_algo.h>
#include <connectivity/connectivity_data.h>
#include <length_delay_calculation/length_delay_calculation.h>
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
#include <router/pns_debug_decorator.h>
#include <router/pns_line.h>
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

#include <atomic>
#include <cctype>
#include <chrono>
#include <wx/thread.h>
#include <limits>
#include <deque>
#include <map>
#include <mutex>
#include <numeric>
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


/// @brief The router's steps of one try as text, no addresses: where (file:line), what, and the
/// items and shapes by kind, net, layers, box and points (trace_attempt)
class TRACE_DECORATOR : public PNS::DEBUG_DECORATOR
{
public:
    TRACE_DECORATOR( const PNS::ROUTER_IFACE& aIface, size_t aCap ) : m_iface( aIface ), m_cap( aCap ) { SetDebugEnabled( true ); }

    void Message( const wxString& aMsg, const SRC_LOCATION_INFO& aSrc ) override { add( aSrc, "msg " + str( aMsg ) ); }
    void NewStage( const wxString& aName, int aIter, const SRC_LOCATION_INFO& aSrc ) override
    {
        add( aSrc, "stage " + str( aName ) + " " + std::to_string( aIter ) );
    }
    void BeginGroup( const wxString& aName, int aLevel, const SRC_LOCATION_INFO& aSrc ) override { add( aSrc, "begin " + str( aName ) ); }
    void EndGroup( const SRC_LOCATION_INFO& aSrc ) override { add( aSrc, "end" ); }
    void AddPoint( const VECTOR2I& aP, const KIGFX::COLOR4D&, int, const wxString& aName, const SRC_LOCATION_INFO& aSrc ) override
    {
        add( aSrc, "point " + str( aName ) + " " + pt( aP ) );
    }
    void AddItem( const PNS::ITEM* aItem, const KIGFX::COLOR4D&, int, const wxString& aName, const SRC_LOCATION_INFO& aSrc ) override
    {
        add( aSrc, "item " + str( aName ) + " " + itemKey( aItem ) );
    }
    void AddShape( const SHAPE* aShape, const KIGFX::COLOR4D&, int, const wxString& aName, const SRC_LOCATION_INFO& aSrc ) override
    {
        add( aSrc, "shape " + str( aName ) + " " + shapeKey( aShape ) );
    }

    std::string itemKey( const PNS::ITEM* aItem ) const
    {
        if( !aItem )
            return "-";

        std::string key = aItem->KindStr() + " net " + std::to_string( m_iface.GetNetCode( aItem->Net() ) ) + " L" + std::to_string( aItem->Layers().Start() )
                          + "-" + std::to_string( aItem->Layers().End() );

        if( const PNS::LINKED_ITEM* linked = dynamic_cast<const PNS::LINKED_ITEM*>( aItem ) )
            key += " uid " + std::to_string( linked->Uid() );

        if( aItem->Kind() == PNS::ITEM::LINE_T )
            return key + " " + chain( static_cast<const PNS::LINE*>( aItem )->CLine() );

        return key + " " + shapeKey( aItem->Shape( aItem->Layers().Start() ) );
    }

    nlohmann::json Json() const
    {
        return { { "events", m_events }, { "total", m_total }, { "capped", m_total > m_events.size() }, { "other_threads", m_otherThreads.load() } };
    }

private:
    static std::string pt( const VECTOR2I& aP ) { return std::to_string( aP.x ) + "," + std::to_string( aP.y ); }

    // a message or a name may print a pointer (%p): it differs from run to run
    static std::string noAddresses( std::string aText )
    {
        for( size_t at = aText.find( "0x" ); at != std::string::npos; at = aText.find( "0x", at + 2 ) )
        {
            size_t end = at + 2;

            while( end < aText.size() && std::isxdigit( (unsigned char) aText[end] ) )
                end++;

            aText.replace( at + 2, end - at - 2, "_" );
        }

        // MinGW / MSVC print %p without the prefix, zero-padded to the pointer width
        const size_t width = 2 * sizeof( void* );

        for( size_t at = 0; at < aText.size(); )
        {
            if( !std::isxdigit( (unsigned char) aText[at] ) || ( at > 0 && std::isalnum( (unsigned char) aText[at - 1] ) ) )
            {
                at++;
                continue;
            }

            size_t end = at;

            while( end < aText.size() && std::isxdigit( (unsigned char) aText[end] ) )
                end++;

            if( end - at == width && ( end == aText.size() || !std::isalnum( (unsigned char) aText[end] ) ) )
            {
                aText.replace( at, width, "0x_" );
                end = at + 3;
            }

            at = end;
        }

        return aText;
    }

    static std::string chain( const SHAPE_LINE_CHAIN& aChain )
    {
        std::string out = "chain " + std::to_string( aChain.PointCount() ) + ":";

        for( int i = 0; i < aChain.PointCount(); ++i )
            out += " " + pt( aChain.CPoint( i ) );

        return out;
    }

    static std::string shapeKey( const SHAPE* aShape )
    {
        if( !aShape )
            return "-";

        if( aShape->Type() == SH_LINE_CHAIN )
            return chain( *static_cast<const SHAPE_LINE_CHAIN*>( aShape ) );

        const BOX2I box = aShape->BBox();
        return "type " + std::to_string( (int) aShape->Type() ) + " box " + pt( box.GetOrigin() ) + " " + pt( box.GetEnd() );
    }

    // the router's pool threads (nearest obstacle) report too: in no fixed order, only counted
    void add( const SRC_LOCATION_INFO& aSrc, std::string aText )
    {
        if( std::this_thread::get_id() != m_thread )
        {
            m_otherThreads++;
            return;
        }

        if( m_events.size() < m_cap )
        {
            const size_t slash = aSrc.fileName.find_last_of( "/\\" );
            m_events.push_back( ( slash == std::string::npos ? aSrc.fileName : aSrc.fileName.substr( slash + 1 ) ) + ":" + std::to_string( aSrc.line ) + " "
                                + noAddresses( std::move( aText ) ) );
        }

        m_total++;
    }

    const PNS::ROUTER_IFACE& m_iface;
    const std::thread::id    m_thread = std::this_thread::get_id();
    std::atomic<size_t>      m_otherThreads{ 0 };
    size_t                   m_cap;
    std::vector<std::string> m_events;
    size_t                   m_total = 0;
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


/// @brief The other net of a differential pair: one of aPairs (P net code -> N), else the board's name rule
static NETINFO_ITEM* coupledNet( BOARD* aBoard, const NETINFO_ITEM* aNet, const std::map<int, int>* aPairs )
{
    if( !aNet )
        return nullptr;

    if( aPairs )
    {
        for( const auto& [p, n] : *aPairs )
        {
            if( aNet->GetNetCode() == p || aNet->GetNetCode() == n )
                return aBoard->FindNet( aNet->GetNetCode() == p ? n : p );
        }
    }

    return aBoard->DpCoupledNet( aNet );
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

    // the rule caches (intersectsCourtyard and the like) keep a board item's answers until the next
    // commit: copper moved in place and not committed yet would get its old place's answer
    std::vector<BOARD_ITEM*> unflag;

    for( const std::vector<BOARD_CONNECTED_ITEM*>* list : { &aChanged, aStaged } )
    {
        if( !list )
            continue;

        for( BOARD_CONNECTED_ITEM* item : *list )
        {
            if( item && !item->HasFlag( ROUTER_TRANSIENT ) )
            {
                item->SetFlags( ROUTER_TRANSIENT );
                unflag.push_back( item );
            }
        }
    }

    struct UNFLAG
    {
        std::vector<BOARD_ITEM*>& items;
        ~UNFLAG()
        {
            for( BOARD_ITEM* item : items )
                item->ClearFlags( ROUTER_TRANSIENT );
        }
    } unflagAtEnd{ unflag };

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


/// @brief A stub's neck at a pad: the width it may have there and why it is narrower than asked
struct STUB_NECK
{
    int         width = 0;
    std::string why;   ///< empty when the stub keeps the width it was asked for
};


/// @brief A custom track_width rule where aPad is (a fine-pitch escape rule in a courtyard), for a
/// track of aNetCode on aLayer: {opt or min, min}; {0, 0} when only the class says
std::pair<int, int> ruleTrackWidthAt( BOARD* aBoard, PAD* aPad, int aNetCode, PCB_LAYER_ID aLayer )
{
    std::shared_ptr<DRC_ENGINE> drc = aBoard->GetDesignSettings().m_DRCEngine;

    if( !drc || !aPad )
        return { 0, 0 };

    PCB_TRACK probe( aBoard );
    probe.SetFlags( ROUTER_TRANSIENT );
    probe.SetNetCode( aNetCode );
    probe.SetStart( aPad->GetPosition() );
    probe.SetEnd( aPad->GetPosition() );
    probe.SetLayer( aLayer );

    const DRC_CONSTRAINT c = drc->EvalRules( TRACK_WIDTH_CONSTRAINT, &probe, nullptr, aLayer );

    if( !c.GetParentRule() || c.GetParentRule()->IsImplicit() )
        return { 0, 0 };

    const int min = c.GetValue().HasMin() ? c.GetValue().Min() : 0;
    return { c.GetValue().HasOpt() ? c.GetValue().Opt() : min, min };
}


/// @brief The narrowest a stub at aPad may be: the board minimum, or a custom track_width rule's
/// min there
int stubMinWidth( BOARD* aBoard, PAD* aPad, int aNetCode, PCB_LAYER_ID aLayer )
{
    return std::max( { aBoard->GetDesignSettings().m_TrackMinWidth, ruleTrackWidthAt( aBoard, aPad, aNetCode, aLayer ).second,
                       1 } );
}


} // namespace


KOPENAPI_RULE_TRACK KopenapiRuleTrack( BOARD* aBoard, int aNetCode, const VECTOR2I& aAt, PCB_LAYER_ID aLayer )
{
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    KOPENAPI_RULE_TRACK          out{ std::max( bds.m_TrackMinWidth, 0 ), bds.GetCurrentTrackWidth() };

    if( NETINFO_ITEM* net = aBoard->FindNet( aNetCode ) )
    {
        if( NETCLASS* nc = net->GetNetClass(); nc && nc->HasTrackWidth() )
            out.opt = nc->GetTrackWidth();
    }

    if( std::shared_ptr<DRC_ENGINE> drc = bds.m_DRCEngine )
    {
        PCB_TRACK probe( aBoard );
        probe.SetFlags( ROUTER_TRANSIENT );
        probe.SetNetCode( aNetCode );
        probe.SetStart( aAt );
        probe.SetEnd( aAt );
        probe.SetLayer( aLayer );

        const DRC_CONSTRAINT c = drc->EvalRules( TRACK_WIDTH_CONSTRAINT, &probe, nullptr, aLayer );

        if( c.GetValue().HasMin() )
            out.min = std::max( out.min, c.GetValue().Min() );

        if( c.GetValue().HasOpt() )
            out.opt = c.GetValue().Opt();
    }

    out.opt = std::max( out.opt, out.min );
    return out;
}


KOPENAPI_RULE_VIA KopenapiRuleVia( BOARD* aBoard, int aNetCode, const VECTOR2I& aAt, PCB_LAYER_ID aLayer, BOARD_ITEM* aOther )
{
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    KOPENAPI_RULE_VIA            out{ bds.GetCurrentViaSize(), bds.GetCurrentViaDrill(), std::max( bds.m_MinClearance, 0 ) };
    NETCLASS*                    nc = nullptr;

    if( NETINFO_ITEM* net = aBoard->FindNet( aNetCode ) )
        nc = net->GetNetClass();

    if( nc && nc->HasViaDiameter() )
        out.dia = nc->GetViaDiameter();

    if( nc && nc->HasViaDrill() )
        out.drill = nc->GetViaDrill();

    if( nc && nc->HasClearance() )
        out.clearance = std::max( out.clearance, nc->GetClearance() );

    if( std::shared_ptr<DRC_ENGINE> drc = bds.m_DRCEngine )
    {
        PCB_VIA probe( aBoard );
        probe.SetFlags( ROUTER_TRANSIENT );
        probe.SetNetCode( aNetCode );
        probe.SetPosition( aAt );
        probe.SetLayerPair( F_Cu, B_Cu );
        probe.SetWidth( PADSTACK::ALL_LAYERS, out.dia );

        const DRC_CONSTRAINT d = drc->EvalRules( VIA_DIAMETER_CONSTRAINT, &probe, nullptr, aLayer );

        if( d.GetValue().HasOpt() )
            out.dia = d.GetValue().Opt();

        if( d.GetValue().HasMin() )
            out.dia = std::max( out.dia, d.GetValue().Min() );

        probe.SetWidth( PADSTACK::ALL_LAYERS, out.dia );

        const DRC_CONSTRAINT h = drc->EvalRules( HOLE_SIZE_CONSTRAINT, &probe, nullptr, aLayer );

        if( h.GetValue().HasOpt() )
            out.drill = h.GetValue().Opt();

        if( h.GetValue().HasMin() )
            out.drill = std::max( out.drill, h.GetValue().Min() );

        probe.SetDrill( out.drill );

        const DRC_CONSTRAINT c = drc->EvalRules( CLEARANCE_CONSTRAINT, &probe, aOther, aLayer );

        if( c.GetValue().HasMin() )
            out.clearance = std::max( std::max( bds.m_MinClearance, 0 ), c.GetValue().Min() );
    }

    return out;
}


bool KopenapiPadFinePitch( BOARD* aBoard, PAD* aPad )
{
    if( !aPad || !aPad->GetParentFootprint() )
        return false;

    PAD*   nearest = nullptr;
    double pitch = std::numeric_limits<double>::max();

    for( PAD* other : aPad->GetParentFootprint()->Pads() )
    {
        const double d = ( other->GetPosition() - aPad->GetPosition() ).EuclideanNorm();

        if( KopenapiPadNeighbours( other, aPad ) && d < pitch )
        {
            pitch = d;
            nearest = other;
        }
    }

    if( !nearest )
        return false;

    const PCB_LAYER_ID        layer = aPad->GetPrincipalLayer();
    const KOPENAPI_RULE_TRACK track = KopenapiRuleTrack( aBoard, aPad->GetNetCode(), aPad->GetPosition(), layer );
    const KOPENAPI_RULE_VIA   via = KopenapiRuleVia( aBoard, aPad->GetNetCode(), aPad->GetPosition(), layer, nearest );
    int                       gap = 0;

    aPad->GetEffectiveShape( layer )->Collide( nearest->GetEffectiveShape( layer ).get(), KiROUND( pitch ), &gap );
    return gap < track.opt + 2 * via.clearance || pitch < via.dia + via.clearance;
}


namespace
{

/// @brief The widest track of aNetCode, centred on aPad, that leaves it between its row neighbours of
/// other nets at the clearance DRC asks to each (5 µm steps); INT_MAX when no neighbour bounds it
int padEntryFit( BOARD* aBoard, PAD* aPad, int aNetCode )
{
    if( !aPad || !aPad->GetParentFootprint() )
        return INT_MAX;

    std::shared_ptr<DRC_ENGINE> drc = aBoard->GetDesignSettings().m_DRCEngine;
    const int                   step = pcbIUScale.mmToIU( 0.005 );
    int                         fit = INT_MAX;

    for( PAD* other : aPad->GetParentFootprint()->Pads() )
    {
        if( !KopenapiPadNeighbours( other, aPad ) || ( aNetCode > 0 && other->GetNetCode() == aNetCode ) )
            continue;

        int clearance = std::max( aBoard->GetDesignSettings().m_MinClearance, 0 );

        if( drc )
        {
            PCB_TRACK probe( aBoard );
            probe.SetFlags( ROUTER_TRANSIENT );
            probe.SetNetCode( aNetCode );
            probe.SetStart( aPad->GetPosition() );
            probe.SetEnd( aPad->GetPosition() );
            probe.SetLayer( aPad->GetPrincipalLayer() );

            const DRC_CONSTRAINT c = drc->EvalRules( CLEARANCE_CONSTRAINT, &probe, other, aPad->GetPrincipalLayer() );

            if( c.GetValue().HasMin() )
                clearance = std::max( clearance, c.GetValue().Min() );
        }

        const VECTOR2I size = other->GetSize( PADSTACK::ALL_LAYERS );
        const double   dist = ( other->GetPosition() - aPad->GetPosition() ).EuclideanNorm();
        const double   room = 2.0 * ( dist - std::min( size.x, size.y ) / 2.0 - clearance );

        if( room > 0 )
            fit = std::min( fit, int( room ) / step * step );
    }

    return fit;
}


/// @brief The router's neck-down for a stub leaving aPad (neck_down auto of pcb_route): at a fine-pitch
/// pad (KopenapiPadFinePitch) no wider than leaves its neighbours the clearance the rules ask
/// (padEntryFit), not below the narrowest the rules allow there (stubMinWidth); a custom track_width
/// rule at the pad sets it lower (its opt, else min)
STUB_NECK padNeck( BOARD* aBoard, PAD* aPad, int aWidth, PCB_LAYER_ID aLayer )
{
    STUB_NECK out{ aWidth, std::string() };

    if( !aPad || !KopenapiPadFinePitch( aBoard, aPad ) )
        return out;

    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    int                          cap = INT_MAX;
    bool                         byFit = false;

    if( const int fit = padEntryFit( aBoard, aPad, aPad->GetNetCode() ); fit < INT_MAX )
    {
        cap = std::max( stubMinWidth( aBoard, aPad, aPad->GetNetCode(), aLayer ), fit );
        byFit = true;
    }

    if( const int rule = ruleTrackWidthAt( aBoard, aPad, aPad->GetNetCode(), aLayer ).first; rule > 0 && rule < cap )
    {
        cap = std::max( bds.m_TrackMinWidth, rule );
        byFit = false;
    }

    if( cap < aWidth )
    {
        double pitch = std::numeric_limits<double>::max();

        for( PAD* other : aPad->GetParentFootprint()->Pads() )
        {
            if( KopenapiPadNeighbours( other, aPad ) )
                pitch = std::min( pitch, double( ( other->GetPosition() - aPad->GetPosition() ).EuclideanNorm() ) );
        }

        const VECTOR2I size = aPad->GetSize( PADSTACK::ALL_LAYERS );
        char           text[200];
        std::snprintf( text, sizeof( text ), "fine-pitch pad %s.%s (pitch %.3g mm, pad %.3g mm wide)%s",
                       str( aPad->GetParentFootprint()->GetReference() ).c_str(), str( aPad->GetNumber() ).c_str(),
                       toMm( pitch ), toMm( std::min( size.x, size.y ) ),
                       byFit ? ": the width that leaves its neighbours their clearance" : "" );
        out = { cap, text };
    }

    return out;
}


/// @brief Narrows aTrack (a stub not on the board yet) until the guard passes it: the widest width
/// from its own down to aMin (5 µm steps) clear of other nets' copper and keep-outs.  aWith: copper
/// of the same call (its via).  False, aTrack unchanged, when even aMin does not pass
bool neckToFit( BOARD* aBoard, PCB_TRACK* aTrack, const std::vector<BOARD_CONNECTED_ITEM*>& aWith, bool aZonesYield,
                int aMin )
{
    const int step = pcbIUScale.mmToIU( 0.005 );
    const int was = aTrack->GetWidth();

    auto passes = [&]( int aWidth )
    {
        aTrack->SetWidth( aWidth );
        return guard( aBoard, { aTrack }, {}, aZonesYield, &aWith ).empty();
    };

    if( aMin <= 0 || aMin > was || !passes( aMin ) )
    {
        aTrack->SetWidth( was );
        return false;
    }

    int lo = aMin, hi = was;   // lo passes; hi is the width asked for

    if( passes( hi ) )
        return true;

    while( hi - lo > step )
    {
        const int mid = lo + ( hi - lo ) / 2;

        if( passes( mid ) )
            lo = mid;
        else
            hi = mid;
    }

    aTrack->SetWidth( lo / step * step >= aMin ? lo / step * step : lo );
    return true;
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
    std::vector<size_t> jobs;          ///< the connections at the pad; none: a pad of another net, open all the call
    bool                open = true;   ///< false once they are all routed
    bool                port = false;  ///< the layout plan's port of a signal pin (ports)
    std::shared_ptr<const std::set<int>> only;   ///< the nets it walls out; none: every other net

    bool walls( int aNet ) const { return open && pad->GetNetCode() != aNet && ( !only || only->count( aNet ) ); }
};


/// @brief A signal pin's port from the layout plan (pcb_fanout_ports): its way out on its side, to past
/// the courtyard on the pad's layer, or to a via onto the other layer
struct PORT_GOAL
{
    PCB_LAYER_ID                layer = F_Cu;   ///< the pad's
    std::optional<PCB_LAYER_ID> otherLayer;
    VECTOR2I                    dir;
    VECTOR2I                    start;          ///< where the way out leaves the pad
    VECTOR2I                    tip;            ///< past the courtyard; at its edge for a pin planned round the part
    int                         width = 0;
    bool                        other = false;  ///< the plan moves it to the other layer
    std::optional<VECTOR2I>     via;            ///< ... by a via here
    bool                        viaPlanned = false;   ///< ... at the plan's spot (a pin yielding to lanes): it leaves by it
    std::string                 side, to;
    bool                        round = false;  ///< planned round the part: it turns as soon as it is out
    std::vector<VECTOR2I>       lane;           ///< lanes: its planned lane from the pad's end, out past its turn
    int                         rank = -1;      ///< ... 0 innermost
    std::vector<KIID>           ring;           ///< ... laid on the board before the routes: its tracks, the last at the lane's end
    std::vector<VECTOR2I>       leg;            ///< ... both lanes laid, their fronts face to face: the leg's 45 degree corners
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
    /// net are found next to them by KiCad's diff-pair router (net names ending in +/- or P/N, or a
    /// pair of `pairs`)
    bool diffPair = false;

    /// @brief Pairs the router takes as differential pairs besides its name rule (P net code -> N)
    const std::map<int, int>* pairs = nullptr;

    /// @brief A narrower track than the net's class gives (> 0): the connection meets a fine-pitch pad
    int neckWidth = 0;

    /// @brief A net's tail from its pad to its pair's line split back: the pair's own way, not held to
    /// other pads' ways out (as the pair is not)
    bool pairTail = false;

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

    /// @brief Laid as given, straight through the points (corners only), no search
    bool exact = false;

    /// @brief Ways out of pads kept open: a route putting copper of another net across one is refused
    const std::vector<ESCAPE_FRONT>* fronts = nullptr;

    /// @brief The ways out are obstacles in the search too (a try again after a refused one)
    bool frontsInWorld = false;

    /// @brief A longer route is not taken (> 0)
    double maxLength = 0;

    /// @brief Each try appended here (pcb_route attempts: true)
    nlohmann::json* tried = nullptr;
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
    int    viaSpotsInEscapes = 0; ///< via spots left out: in the way out kept open for another net's pad
    int    ripUpsSkipped = 0;     ///< rip-ups not repeated: nothing changed since the same one failed
    int    ripUpsWalled = 0;      ///< rip-ups not made: no plan even through the routes of this call
    int    ripUpsPlanned = 0;     ///< rip-ups whose routes to take out a plan chose (the rest: the nearest in the corridor)
    std::map<std::string, int> ripUpsFailedAt;   ///< failed rip-ups by what failed: the route, a victim, a cut one
    int    ripUpsSecond = 0;      ///< victims with no way back that took out routes in turn
    int    pairsStuck = 0;        ///< pair tries ended early: nothing placed after the first steps
    int    frontRetries = 0;      ///< tries refused for walling in a port, made again last with the ways out as obstacles
    int    frontRetriesWon = 0;   ///< ... routed so

    /// @brief While a connection is tried: its first try refused for walling in a port, to make again last
    std::optional<ROUTE_REQUEST>* wallRefused = nullptr;
    nlohmann::json*               wallRefusedAt = nullptr;
    int    padExitsTried = 0;     ///< starts stuck in their pad that tried a corner out of a free side
    int    padExitsWon = 0;       ///< ... routed that way
    int    plans = 0;             ///< global plans made (PlanRoute)
    int    plansFound = 0;        ///< ... that found a way
    int    plansWholeBoard = 0;   ///< ... searched again over the whole board (the box round the ends cut the way)
    int    plansStopped = 0;      ///< ... ended by job_cancel, a budget or the plateau
    double planMs = 0;
    double syncMs = 0;    ///< the router's world built from the board
    double moveMs = 0;    ///< the router's path search (every Move of the head)
    double totalMs = 0;   ///< whole attempts (world, routing, guard, commit)
    int    triesStopped = 0;   ///< router tries ended inside the router (job_cancel, the connection's budget)
    std::map<std::string, int> budgetSpent;   ///< connections whose time budget ran out, by pass
    std::vector<std::string>   overFirstBudget;   ///< the connections the first pass left on its budget (net REF.PAD)
    int64_t clearanceLookups = 0;   ///< the router's clearances looked up past its caches
    int64_t ruleEvaluations = 0;    ///< ... and the design rule evaluations they took

    /// @brief Asked inside a router try, between its walkaround / shove steps: true ends the try as
    /// failed (job_cancel, the connection's rip_up_budget_s); unset: job_cancel
    std::function<bool()> stop;

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

    int            traceAttempt = 0;   ///< trace_attempt: the first try (1 = the first) traced
    int            traceLast = 0;      ///< trace_last: the last one
    int            tracedAttempt = 0;  ///< the try being traced (a try inside a try is not)
    nlohmann::json traces = nlohmann::json::array();
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
/// clearance (guard) or run over a way out kept open for another net's pad (split where it comes
/// near one).  Answers widths_mm when more than one, narrow_kept_mm
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

    std::vector<SHAPE_POLY_SET>      courtyards;
    std::vector<SHAPE_POLY_SET>      neckAreas;
    std::vector<const ESCAPE_FRONT*> fronts;
    std::vector<std::pair<PCB_LAYER_ID, SHAPE_POLY_SET>> frontZones;
    const int                        clearance = mine.front()->GetOwnClearance( mine.front()->GetLayer() );

    if( aReq.fronts )
    {
        for( const ESCAPE_FRONT& front : *aReq.fronts )
        {
            BOX2I nearBox = front.shape.BBox();
            nearBox.Inflate( aBase + clearance );

            if( !front.walls( netCode ) || !nearBox.Intersects( box ) )
                continue;

            fronts.push_back( &front );

            // past this outline a track aBase wide keeps its clearance from the way out
            SHAPE_POLY_SET zone;
            SHAPE_SEGMENT( front.shape.GetSeg(), front.shape.GetWidth() + aBase + 2 * clearance )
                    .TransformToPolygon( zone, aBoard->GetDesignSettings().m_MaxError, ERROR_OUTSIDE );
            frontZones.emplace_back( front.layer, std::move( zone ) );
        }
    }

    auto overFront = [&]( PCB_TRACK* aPiece )
    {
        for( const ESCAPE_FRONT* front : fronts )
        {
            if( aPiece->IsOnLayer( front->layer )
                && aPiece->GetEffectiveShape( front->layer )->Collide( &front->shape, std::max( aPiece->GetOwnClearance( front->layer ) - 1, 0 ) ) )
            {
                return true;
            }
        }

        return false;
    };

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

    struct JOINT
    {
        PCB_TRACK*            before;
        PCB_TRACK*            after;
        const SHAPE_POLY_SET* courtyard;
    };

    std::vector<PCB_TRACK*>   pieces;
    std::vector<JOINT>        joints;
    std::map<PCB_TRACK*, int> routedWidth;

    for( PCB_TRACK* track : mine )
    {
        const SEG whole( track->GetStart(), track->GetEnd() );
        std::vector<std::pair<VECTOR2I, const SHAPE_POLY_SET*>> cuts = { { whole.A, nullptr }, { whole.B, nullptr } };

        auto cutAt = [&]( const SHAPE_POLY_SET& c, bool aCourtyard )
        {
            for( int o = 0; o < c.OutlineCount(); ++o )
            {
                SHAPE_LINE_CHAIN::INTERSECTIONS hits;
                c.COutline( o ).Intersect( whole, hits );

                for( const SHAPE_LINE_CHAIN::INTERSECTION& hit : hits )
                    cuts.emplace_back( hit.p, aCourtyard ? &c : nullptr );
            }
        };

        for( const SHAPE_POLY_SET& c : courtyards )
            cutAt( c, true );

        for( const auto& [layer, zone] : frontZones )
        {
            if( layer == track->GetLayer() )
                cutAt( zone, false );
        }

        std::stable_sort( cuts.begin(), cuts.end(),
                          [&]( const auto& a, const auto& b )
                          {
                              return ( a.first - whole.A ).SquaredEuclideanNorm() < ( b.first - whole.A ).SquaredEuclideanNorm();
                          } );
        cuts.erase( std::unique( cuts.begin(), cuts.end(), []( const auto& a, const auto& b ) { return a.first == b.first; } ),
                    cuts.end() );

        routedWidth[track] = track->GetWidth();
        track->SetEnd( cuts[1].first );
        pieces.push_back( track );

        for( size_t c = 2; c < cuts.size(); ++c )
        {
            PCB_TRACK* piece = static_cast<PCB_TRACK*>( track->Duplicate( false ) );
            piece->SetStart( cuts[c - 1].first );
            piece->SetEnd( cuts[c].first );
            aIface.Staged().Add( piece );
            aIface.m_added.push_back( piece );

            if( cuts[c - 1].second )
                joints.push_back( { pieces.back(), piece, cuts[c - 1].second } );

            routedWidth[piece] = routedWidth[track];
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

            if( want > routed && ( overFront( piece ) || !guard( aBoard, { piece }, aIface.m_removed, false, &staged ).empty() ) )
            {
                piece->SetWidth( routed );
                narrowKept += piece->GetLength();
            }
        }

    }

    // DRC tests a piece's whole shape against a courtyard: the piece outside one ends where its
    // round end clears it, the piece inside takes the rest
    for( const JOINT& joint : joints )
    {
        auto inside = [&]( PCB_TRACK* aPiece )
        {
            return joint.courtyard->Contains( ( aPiece->GetStart() + aPiece->GetEnd() ) / 2 );
        };

        const bool beforeIn = inside( joint.before );

        if( beforeIn == inside( joint.after ) )
            continue;

        PCB_TRACK*     out = beforeIn ? joint.after : joint.before;
        PCB_TRACK*     in = beforeIn ? joint.before : joint.after;
        const VECTOR2I at = joint.before->GetEnd();
        const VECTOR2I end = beforeIn ? out->GetEnd() : out->GetStart();
        const VECTOR2I dir = end - at;
        const int64_t  len = dir.EuclideanNorm();

        auto pointAt = [&]( int64_t aT ) { return at + dir.Resize( aT ); };
        auto clearAt = [&]( int64_t aT )
        {
            SHAPE_SEGMENT shape( SEG( pointAt( aT ), end ), out->GetWidth() );
            return !joint.courtyard->Collide( &shape );
        };

        if( len <= 0 || clearAt( 0 ) || !clearAt( len - 1 ) )
            continue;

        int64_t lo = 0, hi = len - 1;

        while( hi - lo > 1 )
        {
            const int64_t m = ( lo + hi ) / 2;
            ( clearAt( m ) ? hi : lo ) = m;
        }

        const VECTOR2I cut = pointAt( hi );
        joint.before->SetEnd( cut );
        joint.after->SetStart( cut );

        if( in->GetWidth() > routedWidth[in]
            && ( overFront( in ) || !guard( aBoard, { in }, aIface.m_removed, false, &staged ).empty() ) )
        {
            in->SetWidth( routedWidth[in] );
            narrowKept += in->GetLength();
        }
    }

    for( PCB_TRACK* piece : pieces )
        widths.insert( piece->GetWidth() );

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

    // trace_attempt: outlives the router and its interface, which report to it
    struct TRACED
    {
        ROUTE_PROFILE&                   profile;
        nlohmann::json&                  out;
        std::unique_ptr<TRACE_DECORATOR> tracer;
        nlohmann::json                   trace;
        ~TRACED()
        {
            if( !tracer )
                return;

            nlohmann::json pns = tracer->Json();
            // uids are counted by the process: kept apart from the steps
            std::string steps;

            for( const nlohmann::json& event : pns["events"] )
            {
                const std::string& text = event.get_ref<const std::string&>();
                size_t             from = 0;

                for( size_t at = text.find( " uid " ); at != std::string::npos; at = text.find( " uid ", from ) )
                {
                    steps.append( text, from, at - from );
                    from = at + 5;

                    while( from < text.size() && std::isdigit( (unsigned char) text[from] ) )
                        from++;
                }

                steps.append( text, from, std::string::npos );
                steps += '\n';
            }

            trace["pns_hash"] = std::to_string( std::hash<std::string>()( steps ) );
            trace["pns_uid_hash"] = std::to_string( std::hash<std::string>()( pns["events"].dump() ) );
            trace["routed"] = out.value( "routed", false );

            // one try: all of it; a range: what tells two runs' tries apart
            if( profile.traceLast == profile.traceAttempt )
            {
                trace["pns"] = std::move( pns );
                trace["out"] = out;
            }
            else
            {
                trace.erase( "board_tracks" );
                trace["pns_total"] = pns["total"];
            }

            profile.traces.push_back( std::move( trace ) );
            profile.tracedAttempt = 0;
        }
    } traced{ profile, aOut, nullptr, nullptr };

    auto iface = std::make_unique<KOPENAPI_PNS_IFACE>();
    iface->SetBoard( board );

    // the interface deletes its decorator: the tracer is taken back before
    struct DETACH
    {
        KOPENAPI_PNS_IFACE& iface;
        ~DETACH() { iface.SetDebugDecorator( nullptr ); }
    } detach{ *iface };

    if( aReq.pairs )
        iface->SetExplicitDiffPairs( *aReq.pairs );
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
    bool                                    stopped = false;

    auto search = [&]() -> std::optional<bool>
    {
        settings.SetMode( aReq.mode );

        // The same result on every run: the router's wall-clock limits (made for a hand on the mouse) are
        // lifted for the call, its iteration and work limits bound the work (a shove through a dense row
        // can take 50 ms an iteration: the editor's 250 would run past any clock; the largest walkaround
        // cluster seen, 5800 items around a 600-point line, is 22M work)
        settings.SetShoveTimeLimit( 60000 );
        settings.SetShoveIterationLimit( 100 );
        settings.SetClusterTimeout( 60000 );
        settings.SetClusterWorkLimit( 100'000'000 );

        // a cancel or the connection's budget ends the try inside the router, not after it
        settings.SetStopCheck(
                [&]()
                {
                    if( !stopped && ( profile.stop ? profile.stop() : KopenapiJobCancelled() ) )
                        stopped = true;

                    return stopped;
                } );

        // our router is current on this thread only: the editor's router stays current for the editor
        PNS::ROUTER::THREAD_SCOPE routerScope;

        router = std::make_unique<PNS::ROUTER>();
        router->SetInterface( iface.get() );
        router->LoadSettings( &settings );

        if( profile.traceAttempt > 0 && profile.attempts + 1 >= profile.traceAttempt && profile.attempts + 1 <= profile.traceLast
            && !profile.tracedAttempt )
        {
            nlohmann::json tracks = nlohmann::json::array();

            for( PCB_TRACK* track : board->Tracks() )
                tracks.push_back( copperKey( track ) );

            nlohmann::json fronts = nlohmann::json::array();

            for( size_t k = 0; aReq.fronts && k < aReq.fronts->size(); ++k )
            {
                const ESCAPE_FRONT& front = ( *aReq.fronts )[k];

                if( front.walls( aReq.from->GetNetCode() ) )
                    fronts.push_back( str( front.pad->GetParentFootprint()->GetReference() ) + "." + str( front.pad->GetNumber() ) + " "
                                      + std::to_string( front.shape.GetSeg().A.x ) + "," + std::to_string( front.shape.GetSeg().A.y ) );
            }

            nlohmann::json vias = nlohmann::json::array();

            for( const auto& [at, layer] : aReq.vias )
                vias.push_back( { at.x, at.y, layer ? (int) *layer : -1 } );

            std::string trackKeys;

            for( const nlohmann::json& key : tracks )
                trackKeys += key.get<std::string>() + "\n";

            traced.trace = { { "attempt", profile.attempts + 1 },
                              { "from", copperKey( aReq.from ) },
                              { "to", aReq.to ? copperKey( aReq.to ) : std::string() },
                              { "to_pos", { aReq.toPos.x, aReq.toPos.y } },
                              { "layer", (int) aReq.layer },
                              { "mode", (int) aReq.mode },
                              { "neck", aReq.neckWidth },
                              { "vias", vias },
                              { "fronts", fronts },
                              { "board_tracks", tracks },
                              { "board_tracks_hash", std::to_string( std::hash<std::string>()( trackKeys ) ) } };
            profile.tracedAttempt = profile.attempts + 1;
            traced.tracer = std::make_unique<TRACE_DECORATOR>( *iface, 400000 );
            iface->SetDebugDecorator( traced.tracer.get() );
        }

        router->SyncWorld();

        // a way out of another net's pad kept open is that net's copper to the router when asked: it walks around
        if( aReq.fronts && aReq.frontsInWorld && !aReq.diffPair )
        {
            for( const ESCAPE_FRONT& front : *aReq.fronts )
            {
                if( !front.walls( aReq.from->GetNetCode() ) )
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
                // SetPos moves the shape by the change of position: before the shape, not after it
                solid->SetPos( front.shape.GetSeg().Center() );
                solid->SetShape( front.shape.Clone() );
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
            if( settings.StopRequested() )
                return;

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
        reached = !stopped && ( aReq.diffPair || ( placer->CurrentEnd() == target && targetLayers.Overlaps( router->GetCurrentLayer() ) ) );

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
            reached = !stopped && placer->CurrentEnd() == target;
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
        if( stopped )
            profile.triesStopped++;

        aOut["reason"] = stopped       ? "stopped inside the router try (job_cancel, the connection's time budget: first_pass_budget_s / rip_up_budget_s, or stop_on_plateau_s ending its turn)"
                         : aReq.diffPair ? "the pair did not reach the target pads"
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
                    aOut["overlap_mm"] = { { toMm( a.A.x ), toMm( a.A.y ), toMm( a.B.x ), toMm( a.B.y ) },
                                           { toMm( b.A.x ), toMm( b.A.y ), toMm( b.B.x ), toMm( b.B.y ) } };
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

    if( aReq.fronts && !aReq.diffPair && !aReq.pairTail )
    {
        for( BOARD_CONNECTED_ITEM* item : changed )
        {
            for( const ESCAPE_FRONT& front : *aReq.fronts )
            {
                // the way out is the pad's copper: another net keeps its clearance from it
                if( !front.walls( item->GetNetCode() ) || !item->IsOnLayer( front.layer )
                    || !item->GetEffectiveShape( front.layer )->Collide( &front.shape, std::max( item->GetOwnClearance( front.layer ) - 1, 0 ) ) )
                {
                    continue;
                }

                const std::string key = copperKey( item );
                const VECTOR2I    at = item->GetEffectiveShape( front.layer )->Centre();

                aOut["routed"] = false;
                aOut["reason"] = "refused: the route would wall in the way out of "
                                 + str( front.pad->GetParentFootprint()->GetReference() ) + "." + str( front.pad->GetNumber() );
                aOut["walls_in"] = { { "pad", str( front.pad->GetParentFootprint()->GetReference() ) + "." + str( front.pad->GetNumber() ) },
                                     { "layer", str( board->GetLayerName( front.layer ) ) },
                                     { "by", key },
                                     { "way_out_mm", { { toMm( front.shape.GetSeg().A.x ), toMm( front.shape.GetSeg().A.y ) },
                                                       { toMm( front.shape.GetSeg().B.x ), toMm( front.shape.GetSeg().B.y ) } } },
                                     { "at_mm", { toMm( at.x ), toMm( at.y ) } },
                                     { "route_mm", toMm( length ) } };

                if( front.port )
                    aOut["walls_in"]["port"] = true;

                iface->Staged().Revert();
                return false;
            }
        }
    }
    const double         direct = ( target - start ).EuclideanNorm();

    if( aReq.maxLength > 0 && length > aReq.maxLength )
    {
        iface->Staged().Revert();
        aOut["routed"] = false;
        aOut["reason"] = "too long round the ways out kept open";
        aOut["length_mm"] = toMm( length );
        aOut["max_length_mm"] = toMm( aReq.maxLength );
        return false;
    }

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

    const NETINFO_ITEM* coupled = aReq.diffPair ? coupledNet( board, aReq.from->GetNet(), aReq.pairs ) : nullptr;

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

    // rule areas barring tracks on the layer: no net, and the start point inside one fails the router at once
    std::vector<ZONE*> ruleAreas( aBoard->Zones().begin(), aBoard->Zones().end() );

    for( FOOTPRINT* fp : aBoard->Footprints() )
        ruleAreas.insert( ruleAreas.end(), fp->Zones().begin(), fp->Zones().end() );

    for( ZONE* zone : ruleAreas )
    {
        if( !zone->GetIsRuleArea() || !zone->GetDoNotAllowTracks() || !zone->IsOnLayer( aLayer )
            || !area.Intersects( zone->GetBoundingBox() ) )
        {
            continue;
        }

        std::shared_ptr<SHAPE> shape = zone->GetEffectiveShape( aLayer );
        int                    actual = 0;

        if( !shape->Collide( &path, 0 ) && !shape->Collide( &probe, nearby ) )
            continue;

        shape->Collide( &probe, std::numeric_limits<int>::max() / 2, &actual );

        nlohmann::json what = { { "type", "rule_area" }, { "rule_area", str( zone->GetZoneName() ) },
                                { "uuid", str( zone->m_Uuid.AsString() ) }, { "layer", str( aBoard->GetLayerName( aLayer ) ) },
                                { "distance_mm", toMm( actual ) } };

        if( const FOOTPRINT* owner = zone->GetParentFootprint() )
            what["footprint"] = str( owner->GetReference() );

        found.emplace_back( actual, what );
    }

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
/// @brief The route laid as given (ROUTE_REQUEST::exact): straight from its start through its corners to
/// toPos on its layer, no search; refused as a searched one is (ways out kept open, the guard)
bool layExact( PCB_CONTEXT& aContext, const ROUTE_REQUEST& aReq, nlohmann::json& aOut, std::vector<KIID>& aAdded )
{
    BOARD*                board = aContext.GetBoard();
    std::vector<VECTOR2I> pts = { aReq.fromPos.value_or( aReq.from->GetPosition() ) };

    for( const auto& [at, layer] : aReq.vias )
    {
        if( layer )
        {
            aOut["routed"] = false;
            aOut["reason"] = "laid as given: a layer change is not laid";
            return false;
        }

        pts.push_back( at );
    }

    pts.push_back( aReq.toPos );

    NETCLASS*                               netclass = aReq.from->GetEffectiveNetClass();
    const int                               width = aReq.width > 0 ? aReq.width
                                                    : netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth()
                                                                                            : board->GetDesignSettings().GetCurrentTrackWidth();
    std::vector<std::unique_ptr<PCB_TRACK>> made;
    std::vector<BOARD_CONNECTED_ITEM*>      mine;
    nlohmann::json                          segments = nlohmann::json::array();
    double                                  length = 0;

    for( size_t i = 1; i < pts.size(); ++i )
    {
        if( pts[i] == pts[i - 1] )
            continue;

        auto track = std::make_unique<PCB_TRACK>( board );
        track->SetStart( pts[i - 1] );
        track->SetEnd( pts[i] );
        track->SetWidth( width );
        track->SetLayer( aReq.layer );
        track->SetNet( aReq.from->GetNet() );
        length += track->GetLength();
        segments.push_back( { { "from", { toMm( pts[i - 1].x ), toMm( pts[i - 1].y ) } },
                              { "to", { toMm( pts[i].x ), toMm( pts[i].y ) } },
                              { "layer", str( board->GetLayerName( aReq.layer ) ) },
                              { "width_mm", toMm( width ) } } );
        mine.push_back( track.get() );
        made.push_back( std::move( track ) );
    }

    for( BOARD_CONNECTED_ITEM* item : aReq.fronts ? mine : std::vector<BOARD_CONNECTED_ITEM*>() )
    {
        for( const ESCAPE_FRONT& front : *aReq.fronts )
        {
            if( !front.walls( item->GetNetCode() ) || !item->IsOnLayer( front.layer )
                || !item->GetEffectiveShape( front.layer )->Collide( &front.shape, std::max( item->GetOwnClearance( front.layer ) - 1, 0 ) ) )
            {
                continue;
            }

            const std::string at = str( front.pad->GetParentFootprint()->GetReference() ) + "." + str( front.pad->GetNumber() );
            aOut["routed"] = false;
            aOut["reason"] = "refused: the route would wall in the way out of " + at;
            aOut["walls_in"] = { { "pad", at },
                                 { "layer", str( board->GetLayerName( front.layer ) ) },
                                 { "by", copperKey( item ) },
                                 { "way_out_mm", { { toMm( front.shape.GetSeg().A.x ), toMm( front.shape.GetSeg().A.y ) },
                                                   { toMm( front.shape.GetSeg().B.x ), toMm( front.shape.GetSeg().B.y ) } } },
                                 { "route_mm", toMm( length ) } };

            if( front.port )
                aOut["walls_in"]["port"] = true;

            return false;
        }
    }

    const nlohmann::json violations = guard( board, mine, {} );
    aOut["segments"] = segments;
    aOut["metrics"] = { { "length_mm", toMm( length ) }, { "segments", segments.size() }, { "vias", 0 } };

    if( !violations.empty() )
    {
        aOut["routed"] = false;
        aOut["reason"] = "rejected by the guard: the route would touch another net";
        aOut["violations"] = violations;
        return false;
    }

    if( aReq.dryRun )
    {
        aOut["routed"] = true;
        aOut["applied"] = false;
        return true;
    }

    BOARD_COMMIT commit( aContext.GetToolManager() );

    for( std::unique_ptr<PCB_TRACK>& track : made )
    {
        aAdded.push_back( track->m_Uuid );
        commit.Add( track.release() );
    }

    commit.Push( _( "Route (API)" ) );
    board->GetConnectivity()->RecalculateRatsnest();
    aOut["routed"] = true;
    aOut["applied"] = true;
    return true;
}


bool routeOne( PCB_CONTEXT& aContext, const ROUTE_REQUEST& aReq, nlohmann::json& aOut, std::vector<KIID>& aAdded )
{
    const auto   started = std::chrono::steady_clock::now();
    const size_t before = aAdded.size();
    // the router may replace a track the request names (a route going on from it): what is read of the
    // request's items is read before
    const PAD*   fromPad = aReq.tried ? dynamic_cast<const PAD*>( aReq.from ) : nullptr;
    const std::string fromName = fromPad && fromPad->GetParentFootprint()
                                         ? str( fromPad->GetParentFootprint()->GetReference() ) + "." + str( fromPad->GetNumber() )
                                         : std::string( "copper" );
    const bool     padToPad = aReq.from && aReq.from->Type() == PCB_PAD_T && aReq.to && aReq.to->Type() == PCB_PAD_T;
    const int      fromNet = aReq.from ? aReq.from->GetNetCode() : 0;
    const VECTOR2I fromAt = aReq.from ? aReq.fromPos.value_or( aReq.from->GetPosition() ) : VECTOR2I();
    const bool   ok = aReq.exact ? layExact( aContext, aReq, aOut, aAdded ) : routeOneImpl( aContext, aReq, aOut, aAdded );
    ROUTE_PROFILE  scratch;
    ROUTE_PROFILE& profile = ROUTE_PROFILES::Of( aContext.GetBoard(), scratch );

    if( !ok && profile.wallRefused && !*profile.wallRefused && aReq.fronts && !aReq.frontsInWorld && !aReq.diffPair
        && padToPad
        && aOut.contains( "walls_in" ) && aOut["walls_in"].value( "port", false ) )
    {
        ROUTE_REQUEST again = aReq;
        again.frontsInWorld = true;
        again.maxLength = 1.5 * pcbIUScale.mmToIU( aOut["walls_in"].value( "route_mm", 0.0 ) );
        profile.wallRefused->emplace( std::move( again ) );
        *profile.wallRefusedAt = aOut["walls_in"];
    }

    const double ms = msSince( started );
    profile.attempts++;

    for( size_t i = before; ok && !aReq.dryRun && i < aAdded.size(); ++i )
        profile.laid[aAdded[i]] = { profile.pass, profile.job, profile.attempts };
    profile.totalMs += ms;

    const size_t      viaCount = std::count_if( aReq.vias.begin(), aReq.vias.end(), []( const auto& v ) { return v.second.has_value(); } );
    const std::string kind = aReq.exact     ? "laid"
                             : aReq.planned ? "planned"
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

        aOut["blocked_by"] = blockersAt( aContext.GetBoard(), at, aim, aReq.layer, fromNet, half );
    }
    else if( !ok && aReq.from && aOut.value( "reason", std::string() ).rfind( "could not start", 0 ) == 0 )
    {
        const VECTOR2I at = fromAt;
        const VECTOR2I aim = !aReq.vias.empty()             ? aReq.vias.front().first
                             : aReq.to || aReq.toFreeLayer ? aReq.toPos : at;
        const int      half = pcbIUScale.mmToIU( aOut.value( "track_width_mm", 0.2 ) ) / 2
                         + pcbIUScale.mmToIU( aOut.value( "clearance_mm", 0.2 ) );

        aOut["blocked_by"] = blockersAt( aContext.GetBoard(), at, aim, aReq.layer, fromNet, half );
    }

    if( aReq.tried && aReq.tried->size() < 40 )
    {
        BOARD*         board = aContext.GetBoard();
        nlohmann::json vias = nlohmann::json::array();

        for( const auto& [pos, layer] : aReq.vias )
        {
            vias.push_back( { { "at_mm", { toMm( pos.x ), toMm( pos.y ) } },
                              { "layer", layer ? nlohmann::json( str( board->GetLayerName( *layer ) ) ) : nlohmann::json() } } );
        }

        nlohmann::json t = { { "kind", kind }, { "pass", profile.pass }, { "layer", str( board->GetLayerName( aReq.layer ) ) },
                             { "from", fromName },
                             { "routed", ok }, { "ms", std::round( ms ) } };

        if( !vias.empty() )
            t["points"] = vias;

        if( !ok )
        {
            t["reason"] = aOut.value( "reason", std::string() );

            for( const char* key : { "stopped_at_mm", "toward_mm", "walls_in", "overlap_mm" } )
            {
                if( aOut.contains( key ) )
                    t[key] = aOut[key];
            }

            if( aOut.contains( "blocked_by" ) && !aOut["blocked_by"].empty() )
                t["blocked_by"] = aOut["blocked_by"][0];

            if( aOut.contains( "violations" ) && !aOut["violations"].empty() )
                t["violation"] = aOut["violations"][0];
        }

        aReq.tried->push_back( std::move( t ) );
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

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

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
            return KOPENAPI_RESULT::Error( 400, "waypoints: [x_mm, y_mm] (a corner) or {at: [x_mm, y_mm], via: \"B.Cu\"} "
                                                "(a via there, the route continues on that layer; layer is the same key)" );

        std::optional<PCB_LAYER_ID> viaLayer;

        // via and layer name the same thing (as pcb_track_add's points): the layer after the via
        if( w.is_object() && ( w.contains( "via" ) || w.contains( "layer" ) ) )
        {
            if( w.contains( "via" ) && w.contains( "layer" ) && w["via"] != w["layer"] )
                return KOPENAPI_RESULT::Error( 400, "waypoints: via and layer differ (" + w["via"].dump() + " vs " + w["layer"].dump()
                                                    + "): both name the layer the route continues on after the via; give one" );

            const nlohmann::json& name = w.contains( "via" ) ? w["via"] : w["layer"];
            viaLayer = name.is_string() ? parseLayer( board, name.get<std::string>() ) : std::nullopt;

            if( !viaLayer )
                return KOPENAPI_RESULT::Error( 400, "waypoints: via (or layer) is an enabled copper layer, the one the route "
                                                    "continues on after the via there" );
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
                KOPENAPI_RULE_VIA rule = KopenapiRuleVia( board, req.to->GetNetCode(), req.toPos, *padLayer, nullptr );

                if( aArgs.contains( "via_diameter_mm" ) )
                    rule.dia = pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.0 ) );

                const int      reach = std::max( size.x, size.y ) / 2 + rule.dia / 2 + rule.clearance;
                const double   len = std::max( 1.0, double( dir.EuclideanNorm() ) );
                const VECTOR2I at = req.toPos + VECTOR2I( KiROUND( double( dir.x ) * reach / len ), KiROUND( double( dir.y ) * reach / len ) );

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


/// @brief Cut a pair's two lines back from their target ends to where each has room to turn to its own
/// pad alone: by the farther line end's distance to its pad plus one pair pitch (width + gap), along each
/// line on its layer (not past a via or an arc).  aLines: each net's code, its start pad's and its target
/// pad's positions; aEnds gets each net's new open end.
static double splitPairBack( PCB_CONTEXT& aContext, BOARD* aBoard, const std::vector<KIID>& aAdded, size_t aFrom,
                           const std::vector<std::tuple<int, VECTOR2I, VECTOR2I>>& aLines, const nlohmann::json& aAttempt,
                           std::map<int, std::pair<PCB_TRACK*, VECTOR2I>>& aEnds )
{
    const nlohmann::json dp = aAttempt.value( "diff_pair", nlohmann::json::object() );
    const double         pitch = pcbIUScale.mmToIU( dp.value( "width_mm", 0.0 ) + dp.value( "gap_mm", 0.0 ) );

    std::map<int, std::vector<PCB_TRACK*>>      lines;   // net code -> the pair's tracks of that net
    std::map<int, std::pair<PCB_TRACK*, VECTOR2I>> ends;
    double                                      cut = pitch;

    for( const auto& [net, start, target] : aLines )
    {
        VECTOR2I from;
        bool     any = false;

        for( size_t i = aFrom; i < aAdded.size(); ++i )
        {
            if( PCB_TRACK* t = dynamic_cast<PCB_TRACK*>( aBoard->ResolveItem( aAdded[i], true ) );
                t && t->Type() == PCB_TRACE_T && t->GetNetCode() == net )
            {
                lines[net].push_back( t );

                for( const VECTOR2I& at : { t->GetStart(), t->GetEnd() } )
                {
                    if( !any || ( at - start ).SquaredEuclideanNorm() < ( from - start ).SquaredEuclideanNorm() )
                        from = at;

                    any = true;
                }
            }
        }

        // the line's end: the point farthest along it from its start pad (a short stub off the line,
        // a segment laid twice or a bend's corner is no end)
        std::set<PCB_TRACK*>                     walked;
        std::vector<std::pair<VECTOR2I, double>> open = { { from, 0.0 } };
        double                                   farthest = -1.0;

        while( !open.empty() )
        {
            const auto [at, along] = open.back();
            open.pop_back();

            for( PCB_TRACK* t : lines[net] )
            {
                if( walked.count( t ) || ( t->GetStart() != at && t->GetEnd() != at ) )
                    continue;

                walked.insert( t );
                const VECTOR2I next = t->GetStart() == at ? t->GetEnd() : t->GetStart();
                const double   reach = along + ( next - at ).EuclideanNorm();

                if( reach > farthest )
                {
                    farthest = reach;
                    ends[net] = std::make_pair( t, next );
                }

                open.emplace_back( next, reach );
            }
        }

        if( ends.count( net ) )
            cut = std::max( cut, ( ends[net].second - target ).EuclideanNorm() + pitch );
    }

    if( ends.size() != 2 )
        return 0.0;

    BOARD_COMMIT commit( aContext.GetToolManager() );
    std::set<PCB_TRACK*> removed;

    for( auto& [net, end] : ends )
    {
        auto [track, at] = end;
        double left = cut;

        while( track && left > 0 )
        {
            const bool     atStart = track->GetStart() == at;
            const VECTOR2I other = atStart ? track->GetEnd() : track->GetStart();
            const double   length = ( other - at ).EuclideanNorm();

            if( length > left )
            {
                const VECTOR2I to = at + VECTOR2I( KiROUND( ( other.x - at.x ) * left / length ),
                                                   KiROUND( ( other.y - at.y ) * left / length ) );
                commit.Modify( track );
                atStart ? track->SetStart( to ) : track->SetEnd( to );
                at = to;
                break;
            }

            PCB_TRACK* next = nullptr;

            for( PCB_TRACK* t : lines[net] )
            {
                if( t == track || removed.count( t ) || t->GetLayer() != track->GetLayer()
                    || ( t->GetStart() != other && t->GetEnd() != other ) )
                {
                    continue;
                }

                if( t->GetStart() == at || t->GetEnd() == at )
                {
                    commit.Remove( t );   // the same segment laid twice
                    removed.insert( t );
                    continue;
                }

                next = t;
            }

            if( !next )
                break;   // the line's start, a via or an arc: keep the rest

            commit.Remove( track );
            removed.insert( track );
            left -= length;
            track = next;
            at = other;
        }

        aEnds[net] = std::make_pair( track, at );
    }

    commit.Push( _( "Split a pair back to its pads (API)" ) );
    return cut;
}


static KOPENAPI_RESULT pcbRoute( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, const ROUTE_BRANCH* aBranch,
                                 bool aQuiet = false );

// rows of pcb_route listed by default (failed ones first when cut)
constexpr size_t ROUTE_LIST_CAP = 100;




static KOPENAPI_RESULT routeOrBranch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );


/// @brief pns dry run: the call routed for real, every pass (rip-up, clean-up, stub sweep, refill),
/// then the board taken back to where the call found it; the answer is what applying would give
static KOPENAPI_RESULT pnsDryRun( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_HISTORY& history = KOPENAPI_HISTORY::Get();
    KOPENAPI_KEEP_UNSAVED keep;   // the edits are taken back: the board stays as saved as it was

    if( !history.TrialBegin( aCtx, "pcb" ) )
        return KOPENAPI_RESULT::Error( 409, "dry_run: no copy of the board before the call could be made to take the routes "
                                            "back; route for real and roll back (rollback)" );

    nlohmann::json args = aArgs;
    args["dry_run"] = false;

    KOPENAPI_RESULT result = pcbRoute( aCtx, args, nullptr, true );
    KOPENAPI_RESULT back = history.TrialRestore( aCtx );

    if( back.status != 200 )
    {
        return KOPENAPI_RESULT::Error( 500, "dry_run: the routes could not be taken back (" + back.body.dump()
                                                    + "): the board holds them; rollback 1 step takes them out" );
    }

    if( result.status == 200 )
    {
        result.body["dry_run"] = true;
        result.body["applied"] = false;
        result.body["simulated"] = "the whole call (first pass, retries, rip-up, clean-up, stubs, refill) was routed on the board "
                                   "and taken back: applying the same call on the same board gives these rows; uuids in them "
                                   "are gone";
    }

    return result;
}


static KOPENAPI_RESULT h_pcb_route( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    if( aArgs.value( "dry_run", false ) && aArgs.value( "engine", std::string( "pns" ) ) == "pns" && !aArgs.contains( "from" ) )
        return pnsDryRun( aCtx, aArgs );

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


static KOPENAPI_RESULT pcbRoute( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, const ROUTE_BRANCH* aBranch,
                                 bool aQuiet )
{

    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD* board = context->GetBoard();

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

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

    // stage global (engine grid): each connection's way on a coarse grid of the board, nothing laid
    const std::string stage = aArgs.value( "stage", std::string( "full" ) );

    if( stage != "full" && stage != "global" && stage != "detail" )
        return KOPENAPI_RESULT::Error( 400, "stage: full, global or detail" );

    const bool        globalStage = stage == "global";
    const bool        detailStage = stage == "detail";

    if( detailStage && aArgs.value( "dry_run", false ) )
        return KOPENAPI_RESULT::Error( 400, "stage detail lays each route before the next: no dry_run (stage global shows the corridors)" );

    const bool        dryRun = aArgs.value( "dry_run", false ) || globalStage;
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

    if( ( globalStage || detailStage ) && !grid )
        return KOPENAPI_RESULT::Error( 400, "stage global / detail: engine grid" );

    // attempts per connection and pass: a count, not a time (the same board routes the same way);
    // on a big board an attempt costs about a second
    const int maxTries = std::clamp( aArgs.value( "max_tries", 120 ), 1, 10000 );

    // the retry / rip-up pass gives each connection this long, the first pass firstBudgetS (a connection
    // over it waits for the retry pass); a cancelled job stops between tries
    const double ripBudgetS = std::clamp( aArgs.value( "rip_up_budget_s", 30.0 ), 0.0, 3600.0 );
    const double firstBudgetS = std::clamp( aArgs.value( "first_pass_budget_s", 30.0 ), 0.0, 3600.0 );
    // stage detail, a time policy: a branch is one of many and the grid's copper is its fallback, so one
    // branch gets a tenth of a connection's first-pass time (3 s when that is unbounded)
    const double branchBudgetS = std::clamp( aArgs.value( "detail_branch_budget_s", firstBudgetS > 0 ? firstBudgetS / 10 : 3.0 ), 0.0, 3600.0 );
    // a time policy: a corridor's try along it is a soft goal (the member keeps its own turn and whole budget
    // after), so it gets the same share
    const double corridorTryBudgetS = std::clamp( aArgs.value( "corridor_try_budget_s", firstBudgetS > 0 ? firstBudgetS / 10 : 3.0 ), 0.0, 3600.0 );
    std::optional<std::chrono::steady_clock::time_point> tryDeadline;
    std::string budgetName = "first_pass_budget_s";

    // budgets work: the budgets (first pass, rip-up, plateau) counted in the router's work, its stop
    // checks (each walkaround / shove step, each try), at work_per_s a second, so that the same board
    // routes the same under any load; the wall clock only as a hard cap, wall_cap_x times a budget
    const std::string budgetMode = aArgs.value( "budgets", std::string( "time" ) );

    if( budgetMode != "time" && budgetMode != "work" )
        return KOPENAPI_RESULT::Error( 400, "budgets: time (seconds on the wall clock) or work (the router's work, the same under any load)" );

    profile.traceAttempt = std::max( 0, aArgs.value( "trace_attempt", 0 ) );
    profile.traceLast = std::max( profile.traceAttempt, aArgs.value( "trace_last", profile.traceAttempt ) );

    const bool   workBudgets = budgetMode == "work";
    const double workPerS = std::clamp( aArgs.value( "work_per_s", 5000.0 ), 1.0, 1e9 );
    const double wallCapX = std::clamp( aArgs.value( "wall_cap_x", 10.0 ), 1.0, 1000.0 );
    const auto   clockFrom = std::chrono::steady_clock::now();
    int64_t      work = 0;
    int          wallCaps = 0;
    bool         wallCapped = false;
    std::optional<std::chrono::steady_clock::time_point> wallDeadline;

    auto clockNow = [&]()
    {
        if( !workBudgets )
            return std::chrono::steady_clock::now();

        return clockFrom + std::chrono::nanoseconds( (int64_t) ( (double) work * 1e9 / workPerS ) );
    };

    auto setDeadline = [&]( double aBudgetS )
    {
        tryDeadline = clockNow() + std::chrono::milliseconds( (long) ( aBudgetS * 1000 ) );
        wallCapped = false;

        if( workBudgets )
            wallDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds( (long) ( aBudgetS * 1000 * wallCapX ) );
    };

    // stop_on_plateau_s: no connection more routed for that long stops the call as job_cancel does
    // (what is routed is kept)
    const double plateauS = std::clamp( aArgs.value( "stop_on_plateau_s", 0.0 ), 0.0, 86400.0 );
    int          plateauBest = -1;
    auto         plateauSince = clockNow();
    bool         plateauStopped = false;
    size_t       plateauHeld = 0;   ///< connections still to have their turn (first pass, retry pass)
    bool         plateauTurnCut = false;

    // while connections still wait for their turn, a plateau ends the turn of the one routing, not the call
    auto plateauCheck = [&]( const std::chrono::steady_clock::time_point& aNow )
    {
        if( plateauS > 0 && !plateauStopped && std::chrono::duration<double>( aNow - plateauSince ).count() > plateauS )
        {
            if( plateauHeld > 0 )
            {
                plateauTurnCut = true;
                return;
            }

            plateauStopped = true;
            KopenapiJobStopSelf( "stop_on_plateau_s: no connection routed for " + mmText( plateauS ) + " s" );
        }
    };

    // a held turn has the whole plateau for itself
    auto plateauTurn = [&]()
    {
        if( plateauHeld > 0 )
            plateauSince = std::max( plateauSince, clockNow() );

        plateauTurnCut = false;
    };

    auto overTime = [&]()
    {
        ++work;
        const auto now = clockNow();
        plateauCheck( now );

        if( workBudgets && tryDeadline && wallDeadline && !wallCapped && now <= *tryDeadline
            && std::chrono::steady_clock::now() > *wallDeadline )
        {
            wallCapped = true;
            ++wallCaps;
        }

        return KopenapiJobCancelled() || plateauTurnCut || ( tryDeadline && ( now > *tryDeadline || wallCapped ) );
    };

    profile.stop = overTime;

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
        bool                  foreign = false; ///< copper of an earlier call (rip_up_scope unlocked), no row
        bool                  toPairEnd = false;   ///< to the open end of a pair's line that ended short
        int                   bus = -1;        ///< its bus (index in busOut), its place in the bus's order
        int                   busIndex = -1;
        std::optional<VECTOR2I> busVia = std::nullopt;   ///< its spot in the bus's via fan (bus_layer)
        std::vector<VECTOR2I> corridor = {};   ///< bus_paths: its corners along the bus's corridor
        std::vector<std::optional<PCB_LAYER_ID>> corridorVia = {};   ///< per corner: a via there, the layer on
        int                   corridorOf = -1; ///< the corridor (index in corridorOut), its place across it
        int                   corridorRank = -1;   ///< the order it is routed in
        int                   corridorPlace = -1;  ///< from the left of the way
        bool                  corridorRouted = false;
        bool                  laneRouted = false;   ///< lanes: along its laid lane, a planned detour clean_up leaves as it is
        std::optional<VECTOR2I> fanStart, fanEnd;   ///< bus_paths: the fan-out tips at its corridor's ends
        bool                  laneFrom = false, laneTo = false;   ///< bus_paths with lanes: its corridor from / to its laid lane's end
        double                ms = 0;          ///< time of every try for it, rip-up re-routes too
        nlohmann::json        tried = nlohmann::json::array();   ///< its tries (attempts: true)
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

        std::vector<PAD*> netPads;

        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
            {
                if( pad->GetNetCode() == code )
                    netPads.push_back( pad );
            }
        }

        // a track leaving a pad (an escape stub, one segment or a chain of them) is reached at its
        // free end: the ratsnest may anchor it next to the pad, where the pad's neighbours wall it in
        auto stubEnd = [&]( BOARD_CONNECTED_ITEM*& aItem, VECTOR2I& aPos )
        {
            if( aItem->Type() != PCB_TRACE_T && aItem->Type() != PCB_ARC_T )
                return;

            const int eps = pcbIUScale.mmToIU( 0.01 );

            // what else of the net touches point aAt of track aTrack: pads and vias, other tracks
            auto touching = [&]( const PCB_TRACK* aTrack, const VECTOR2I& aAt, int& aPadsVias,
                                 std::vector<PCB_TRACK*>& aTracks )
            {
                aPadsVias = 0;
                aTracks.clear();

                for( PAD* pad : netPads )
                {
                    if( pad->IsOnLayer( aTrack->GetLayer() ) && pad->HitTest( aAt, eps ) )
                        aPadsVias++;
                }

                for( PCB_TRACK* t : board->Tracks() )
                {
                    if( t == aTrack || t->GetNetCode() != code )
                        continue;

                    if( t->Type() == PCB_VIA_T )
                    {
                        const PCB_VIA* via = static_cast<const PCB_VIA*>( t );

                        if( ( via->GetPosition() - aAt ).EuclideanNorm() <= via->GetWidth( aTrack->GetLayer() ) / 2 )
                            aPadsVias++;
                    }
                    else if( t->GetLayer() == aTrack->GetLayer()
                             && ( ( t->GetStart() - aAt ).EuclideanNorm() <= eps
                                  || ( t->GetEnd() - aAt ).EuclideanNorm() <= eps ) )
                    {
                        aTracks.push_back( t );
                    }
                }
            };

            PCB_TRACK*              cur = static_cast<PCB_TRACK*>( aItem );
            int                     padsVias[2];
            std::vector<PCB_TRACK*> tracks[2];

            touching( cur, cur->GetStart(), padsVias[0], tracks[0] );
            touching( cur, cur->GetEnd(), padsVias[1], tracks[1] );

            const bool free0 = !padsVias[0] && tracks[0].empty();
            const bool free1 = !padsVias[1] && tracks[1].empty();

            if( free0 != free1 )
            {
                aPos = free0 ? cur->GetStart() : cur->GetEnd();
                return;
            }

            // a chain: from the end on a pad along single joins to an end nothing else touches
            if( free0 || ( !padsVias[0] && !padsVias[1] ) )
                return;

            VECTOR2I at = padsVias[0] ? cur->GetEnd() : cur->GetStart();
            int      side = padsVias[0] ? 1 : 0;

            if( padsVias[side] )
                return;

            std::vector<PCB_TRACK*> next = tracks[side];

            for( int step = 0; step < 64; ++step )
            {
                if( next.empty() )
                {
                    aItem = cur;
                    aPos = at;
                    return;
                }

                if( next.size() > 1 )
                    return;

                PCB_TRACK* t = next.front();
                at = ( t->GetStart() - at ).EuclideanNorm() <= eps ? t->GetEnd() : t->GetStart();
                cur = t;

                int pv = 0;
                touching( cur, at, pv, next );

                if( pv )
                    return;
            }
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
            else
                stubEnd( pa, posA );

            if( PCB_VIA* v = fannedVia( pb, posB ) )
                pb = v, posB = v->GetPosition();
            else
                stubEnd( pb, posB );

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

    // fine-pitch pads (KopenapiPadFinePitch), once per pad in this call
    std::map<PAD*, bool> fineOf;

    auto finePad = [&]( PAD* aPad ) -> bool
    {
        auto it = fineOf.find( aPad );
        return it != fineOf.end() ? it->second : ( fineOf[aPad] = KopenapiPadFinePitch( board, aPad ) );
    };

    // Neck-down: a connection with an end on a fine-pitch pad (finePad) is routed no wider than leaves the pad's neighbours the clearance the rules ask (padEntryFit), not
    // below the narrowest track the rules allow the net there; the rest of the net keeps its width
    // a pair's lines split back to their pads end beside each other at the pair's pitch: a net's
    // tail leaves its line no wider than the line
    std::set<KIID> pairLineEnds;

    auto neckFor = [&]( BOARD_CONNECTED_ITEM* aFrom, BOARD_CONNECTED_ITEM* aTo ) -> int
    {
        if( aTo && aTo->Type() == PCB_TRACE_T && pairLineEnds.count( aTo->m_Uuid ) )
            return static_cast<PCB_TRACK*>( aTo )->GetWidth();

        auto it = neckPerNet.find( aFrom->GetNetname() );

        if( !( it == neckPerNet.end() ? neckDefault : it->second ) )
            return 0;

        int cap = INT_MAX;

        // the widest track that leaves the pad's neighbours the clearance the rules ask, not below the
        // narrowest the rules allow the net there
        for( BOARD_CONNECTED_ITEM* end : { aFrom, aTo } )
        {
            if( end && end->Type() == PCB_PAD_T && finePad( static_cast<PAD*>( end ) ) )
            {
                PAD* pad = static_cast<PAD*>( end );

                if( const int fit = padEntryFit( board, pad, aFrom->GetNetCode() ); fit < INT_MAX )
                    cap = std::min( cap, std::max( stubMinWidth( board, pad, aFrom->GetNetCode(), pad->GetPrincipalLayer() ), fit ) );
            }
        }

        return cap == INT_MAX ? 0 : cap;
    };

    auto pairTail = [&]( BOARD_CONNECTED_ITEM* aTo )
    {
        return aTo && aTo->Type() == PCB_TRACE_T && pairLineEnds.count( aTo->m_Uuid ) > 0;
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

    // dense_first: connections at fine-pitch pads (finePad) before the rest
    if( order == "dense_first" )
    {
        std::stable_sort( jobs.begin(), jobs.end(),
                          [&]( const JOB& x, const JOB& y )
                          {
                              const bool fx = finePad( x.from ) || ( x.to->Type() == PCB_PAD_T && finePad( static_cast<PAD*>( x.to ) ) );
                              const bool fy = finePad( y.from ) || ( y.to->Type() == PCB_PAD_T && finePad( static_cast<PAD*>( y.to ) ) );

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

    // ways out of fine-pitch pads kept open while their connections wait (filled once the jobs are
    // final, below); open while a connection at the pad is not routed
    std::vector<ESCAPE_FRONT> fronts;
    const std::vector<bool>*  doneOf = nullptr;

    // the planned ports of signal pins (ports), filled once the jobs are final
    std::map<const PAD*, PORT_GOAL> portGoals;

    // of two parts joined by planned lanes, the one whose lanes set the order of the connections between them:
    // summed over those connections, the lanes that turn more round their part (their ranks nest in the ring),
    // then the longer; a straight lane follows any order. One end per pair of parts, so a bus keeps one order
    std::map<std::pair<const FOOTPRINT*, const FOOTPRINT*>, bool> setsOrderCache;

    auto setsOrder = [&]( const FOOTPRINT* aPart, const FOOTPRINT* aOther ) -> bool
    {
        if( auto it = setsOrderCache.find( { aPart, aOther } ); it != setsOrderCache.end() )
            return it->second;

        auto tally = [&]( const FOOTPRINT* aFp, const FOOTPRINT* aFar )
        {
            std::pair<size_t, double> sum( 0, 0.0 );

            for( const auto& [pad, goal] : portGoals )
            {
                if( pad->GetParentFootprint() != aFp || goal.lane.empty() )
                    continue;

                bool joined = false;

                for( const PAD* q : aFar->Pads() )
                    joined = joined || ( q->GetNetCode() > 0 && q->GetNetCode() == pad->GetNetCode() );

                if( !joined )
                    continue;

                sum.first += goal.lane.size();

                for( size_t i = 1; i < goal.lane.size(); ++i )
                    sum.second += ( goal.lane[i] - goal.lane[i - 1] ).EuclideanNorm();
            }

            return sum;
        };

        const std::pair<size_t, double> mine = tally( aPart, aOther ), theirs = tally( aOther, aPart );
        const bool sets = mine.first != theirs.first ? mine.first > theirs.first : mine.second > theirs.second;
        setsOrderCache[{ aPart, aOther }] = sets;
        setsOrderCache[{ aOther, aPart }] = !sets && ( mine != theirs );
        return sets;
    };
    bool                            lanesOn = false;   ///< lanes: ports follow their planned lanes
    int                             lanesLaid = 0, lanesRefused = 0, lanesTakenOut = 0;
    int                             portTries = 0, portWon = 0;
    double                          portMs = 0;
    std::set<size_t>                portTried;   // jobs whose port try was made: once per call

    // a via of the end's net there would stand in the way out kept open for another net's pad: the
    // router walks around that, a try through it is wasted
    auto frontBlocked = [&]( BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aAt )
    {
        int       dia = ds.GetCurrentViaSize();
        NETCLASS* netclass = aEnd->GetEffectiveNetClass();

        if( netclass && netclass->HasViaDiameter() )
            dia = std::min( dia, netclass->GetViaDiameter() );

        const int        clearance = std::max( std::max( ds.m_MinClearance, 0 ), netclass && netclass->HasClearance() ? netclass->GetClearance() : 0 );
        const SHAPE_CIRCLE via( aAt, dia / 2 );

        for( const ESCAPE_FRONT& front : fronts )
        {
            if( front.walls( aEnd->GetNetCode() ) && front.shape.Collide( &via, clearance - 1 ) )
                return true;
        }

        return false;
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

    // inward_escapes: parts whose row pads may also leave inward, through a via in the ring between
    // the row and the exposed pad (the largest pad, inside the row) where the ring is wide enough
    std::map<const FOOTPRINT*, PAD*> inwardParts;

    if( aArgs.contains( "inward_escapes" ) )
    {
        if( !aArgs["inward_escapes"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "inward_escapes: [footprint references]" );

        for( const nlohmann::json& r : aArgs["inward_escapes"] )
        {
            FOOTPRINT* fp = r.is_string() ? board->FindFootprintByReference( wxString::FromUTF8( r.get<std::string>() ) ) : nullptr;

            if( !fp )
                return KOPENAPI_RESULT::Error( 404, "inward_escapes: no footprint " + ( r.is_string() ? r.get<std::string>() : r.dump() ) );

            PAD*   ep = nullptr;
            double epArea = 0, rowArea = 0;

            for( PAD* q : fp->Pads() )
            {
                const double area = (double) q->GetBoundingBox().GetWidth() * q->GetBoundingBox().GetHeight();

                if( area > epArea )
                {
                    rowArea = std::max( rowArea, epArea );
                    epArea = area;
                    ep = q;
                }
                else
                {
                    rowArea = std::max( rowArea, area );
                }
            }

            BOX2I ring;

            for( PAD* q : fp->Pads() )
            {
                if( q != ep )
                    ring.Merge( q->GetBoundingBox() );
            }

            if( !ep || fp->Pads().size() < 5 || epArea < 4 * rowArea || !ring.Contains( ep->GetBoundingBox() ) )
            {
                return KOPENAPI_RESULT::Error( 400, "inward_escapes: " + str( fp->GetReference() )
                                                            + " has no exposed pad inside its pad row" );
            }

            inwardParts[fp] = ep;
        }
    }

    std::set<std::pair<int, int>> inwardSpots;   // via spots in a ring, to name them in the rows

    // spots in the ring between aPad's inner end and the exposed pad aEp (aDir: aPad's way out of the
    // package), nearest the pad first, in front of the pad and between it and each neighbour; none in
    // the straight way from a pad of the exposed pad's net to it (its ground connection)
    auto ringSpots = [&]( PAD* aPad, PAD* aEp, const VECTOR2D& aDir )
    {
        std::vector<VECTOR2I> out;

        if( aEp->GetNetCode() > 0 && aPad->GetNetCode() == aEp->GetNetCode() )
            return out;

        const int grid = pcbIUScale.mmToIU( 0.05 );
        const int clear = std::max( ds.m_MinClearance, 0 );
        int       dia = ds.GetCurrentViaSize();
        NETCLASS* netclass = aPad->GetEffectiveNetClass();

        if( netclass && netclass->HasViaDiameter() )
            dia = std::min( dia, netclass->GetViaDiameter() );

        const BOX2I epBox = aEp->GetBoundingBox();

        // a pad's inner end, the way into the package and the gap from there to the exposed pad
        auto innerOf = [&]( PAD* aQ, VECTOR2D& aIn, double& aGap )
        {
            const BOX2I    box = aQ->GetBoundingBox();
            const VECTOR2D d = VECTOR2D( box.Centre() ) - VECTOR2D( epBox.Centre() );
            const bool     x = std::abs( d.x ) * epBox.GetHeight() >= std::abs( d.y ) * epBox.GetWidth();

            aIn = x ? VECTOR2D( d.x >= 0 ? -1 : 1, 0 ) : VECTOR2D( 0, d.y >= 0 ? -1 : 1 );

            const VECTOR2D end = VECTOR2D( box.Centre() ) + aIn * ( x ? box.GetWidth() / 2.0 : box.GetHeight() / 2.0 );
            aGap = x ? ( aIn.x > 0 ? epBox.GetLeft() - end.x : end.x - epBox.GetRight() )
                     : ( aIn.y > 0 ? epBox.GetTop() - end.y : end.y - epBox.GetBottom() );
            return end;
        };

        std::vector<std::pair<SEG, int>> ground;   // a ground pad's way to the exposed pad, its half width

        for( PAD* q : aPad->GetParentFootprint()->Pads() )
        {
            if( q == aEp || aEp->GetNetCode() <= 0 || q->GetNetCode() != aEp->GetNetCode() )
                continue;

            VECTOR2D     in;
            double       gap = 0;
            const VECTOR2D end = innerOf( q, in, gap );
            const BOX2I  qb = q->GetBoundingBox();
            const int    across = in.x != 0 ? qb.GetHeight() : qb.GetWidth();

            if( gap > 0 )
                ground.emplace_back( SEG( VECTOR2I( end ), VECTOR2I( end + in * gap ) ), across / 2 );
        }

        VECTOR2D       in;
        double         gap = 0;
        const VECTOR2D end = innerOf( aPad, in, gap );
        const VECTOR2D across( -in.y, in.x );
        const double   pitch = pitchOf( aPad );

        // the opposite of the outward way the caller found means the same side
        if( in.x != -aDir.x || in.y != -aDir.y )
            return out;

        for( double d = dia / 2.0 + clear; d <= gap - dia / 2.0 - clear; d += grid )
        {
            for( double side : { 0.0, pitch / 2, -pitch / 2 } )
            {
                const VECTOR2D p = end + in * d + across * side;
                const VECTOR2I at( KiROUND( p.x / grid ) * grid, KiROUND( p.y / grid ) * grid );
                const bool     cuts = std::any_of( ground.begin(), ground.end(), [&]( const std::pair<SEG, int>& g )
                                                   { return g.first.Distance( at ) < g.second + clear + dia / 2; } );

                if( !cuts )
                    out.push_back( at );
            }
        }

        return out;
    };

    // a routed connection through a ring spot names it
    auto noteInward = [&]( const ROUTE_REQUEST& aReq, nlohmann::json& aAttempt )
    {
        for( const auto& [at, layer] : aReq.vias )
        {
            if( inwardSpots.count( { at.x, at.y } ) )
                aAttempt["inward_via_mm"] = { pcbIUScale.IUTomm( at.x ), pcbIUScale.IUTomm( at.y ) };
        }
    };

    // Via spots to leave an end: inward into the ring of an inward_escapes part first, out of the
    // package next (away from its body, past the pad's end, a little to either side), then around the
    // end; nearest the other end first within each kind
    auto spots = [&]( BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aPos, const VECTOR2I& aToward )
    {
        std::vector<VECTOR2I> inward, outward, around;
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

            if( auto it = inwardParts.find( fp ); it != inwardParts.end() && pad != it->second )
                inward = ringSpots( pad, it->second, dir );
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

            if( frontBlocked( aEnd, aAt ) )
            {
                profile.viaSpotsInEscapes++;
                return true;
            }

            return false;
        };

        inward.erase( std::remove_if( inward.begin(), inward.end(), unusable ), inward.end() );
        outward.erase( std::remove_if( outward.begin(), outward.end(), unusable ), outward.end() );
        around.erase( std::remove_if( around.begin(), around.end(), unusable ), around.end() );
        std::stable_sort( around.begin(), around.end(), byTarget );

        std::vector<VECTOR2I> out;

        // the ring spots nearest the pad, at most 3
        for( size_t i = 0; i < inward.size() && i < 3; ++i )
        {
            out.push_back( inward[i] );
            inwardSpots.insert( { inward[i].x, inward[i].y } );
        }

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
    const bool keepTries = aArgs.value( "attempts", false );

    auto refreshFronts = [&]()
    {
        for( ESCAPE_FRONT& front : fronts )
        {
            if( !front.jobs.empty() )
                front.open = std::any_of( front.jobs.begin(), front.jobs.end(), [&]( size_t k ) { return !( *doneOf )[k]; } );
        }
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
        req.stop = overTime;

        for( const ESCAPE_FRONT& front : fronts )
        {
            if( front.walls( aFrom->GetNetCode() ) && !pairTail( aTo ) )
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
        profile.plansStopped += plan.stopped;
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
        aJob.laneRouted = false;

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

        // a try refused for walling in another pin's port: made again as the last try, with the ways out as
        // obstacles to the search (made at once, its detour round the part shuts the ports after it)
        std::optional<ROUTE_REQUEST> portRefused, wallRefused;
        nlohmann::json               portRefusedAs, portRefusedWall, wallRefusedAt;

        struct WATCH
        {
            ROUTE_PROFILE&                prof;
            std::optional<ROUTE_REQUEST>* outer = prof.wallRefused;
            nlohmann::json*               outerAt = prof.wallRefusedAt;
            ~WATCH()
            {
                prof.wallRefused = outer;
                prof.wallRefusedAt = outerAt;
            }
        } watch{ profile };

        profile.wallRefused = &wallRefused;
        profile.wallRefusedAt = &wallRefusedAt;

        // a planned port at an end (ports): the route leaves that pin through its way out, to the tip past
        // the courtyard or through its planned via, and arrives through the far pin's port when the layers
        // allow; from the port's end when only the far end has one
        auto portTry = [&]()
        {
            if( ok || rail || portGoals.empty() || aTries >= maxTries || overTime()
                || !portTried.insert( &aJob - jobs.data() ).second )
            {
                return;
            }

            auto goalOf = [&]( BOARD_CONNECTED_ITEM* aEnd ) -> PORT_GOAL*
            {
                if( !aEnd || aEnd->Type() != PCB_PAD_T )
                    return nullptr;

                auto it = portGoals.find( static_cast<PAD*>( aEnd ) );
                return it == portGoals.end() ? nullptr : &it->second;
            };

            PAD*                  a = aJob.from;
            BOARD_CONNECTED_ITEM* b = aJob.to;
            VECTOR2I              bPos = aJob.toPos;
            PORT_GOAL*            ga = goalOf( a );
            PORT_GOAL*            gb = goalOf( b );

            if( !ga && !gb )
                return;

            if( !ga )
            {
                a = static_cast<PAD*>( aJob.to );
                b = aJob.from;
                bPos = aJob.from->GetPosition();
                std::swap( ga, gb );
            }

            // from the end whose pin yields to lanes (it must leave by the via the plan gives), else with lanes
            // at both ends from the end whose lanes set the order
            if( gb && b->Type() == PCB_PAD_T
                && ( ( gb->viaPlanned && gb->otherLayer && !ga->via )
                     || ( lanesOn && !ga->via && !gb->via && !ga->lane.empty() && !gb->lane.empty()
                          && setsOrder( static_cast<PAD*>( b )->GetParentFootprint(), a->GetParentFootprint() ) ) ) )
            {
                PAD* other = static_cast<PAD*>( b );
                b = a;
                bPos = a->GetPosition();
                a = other;
                std::swap( ga, gb );
            }

            if( !a->IsOnLayer( ga->layer ) )
                return;

            using POINTS = std::vector<std::pair<VECTOR2I, std::optional<PCB_LAYER_ID>>>;
            POINTS       points;
            bool         nearLaid = false;
            PCB_LAYER_ID layer = ga->layer;

            if( ga->via && ga->otherLayer )
            {
                points.push_back( { *ga->via, ga->otherLayer } );
                layer = *ga->otherLayer;
            }
            else if( !ga->ring.empty() && board->ResolveItem( ga->ring.back(), true ) )
            {
                // its lane is laid: the route leaves from the lane's end
                nearLaid = true;
            }
            else if( ga->lane.size() >= 2 )
            {
                for( size_t i = 1; i < ga->lane.size(); ++i )
                    points.push_back( { ga->lane[i], std::nullopt } );
            }
            else
            {
                points.push_back( { ga->tip, std::nullopt } );
            }

            // the far lane laid: the route ends at its end
            PCB_TRACK* farRing = gb && !gb->via && !gb->ring.empty()
                                         ? dynamic_cast<PCB_TRACK*>( board->ResolveItem( gb->ring.back(), true ) )
                                         : nullptr;
            const std::string farName = endName( b );

            if( farRing && farRing->GetLayer() == layer )
            {
                b = farRing;
                bPos = farRing->GetEnd();
            }
            else if( gb && gb->via && gb->otherLayer == layer && b->IsOnLayer( gb->layer ) )
            {
                points.push_back( { *gb->via, gb->layer } );
                layer = gb->layer;
            }
            else if( gb && !gb->via && gb->layer == layer )
            {
                points.push_back( { gb->tip, std::nullopt } );
            }

            std::vector<POINTS> plans;
            const POINTS*       legPlan = nullptr;   // the leg as computed: laid as given
            plans.reserve( 4 );

            if( b->IsOnLayer( layer ) )
            {
                if( nearLaid && b == farRing && !ga->leg.empty() )
                {
                    POINTS jogged = points;

                    for( const VECTOR2I& v : ga->leg )
                        jogged.push_back( { v, std::nullopt } );

                    plans.push_back( std::move( jogged ) );
                    legPlan = &plans.front();
                }

                plans.push_back( points );
            }
            else if( viasAllowed )
            {
                std::optional<PCB_LAYER_ID> arrive;

                for( PCB_LAYER_ID l : layers )
                {
                    if( !arrive && b->IsOnLayer( l ) )
                        arrive = l;
                }

                if( arrive )
                {
                    for( const VECTOR2I& v : spots( b, bPos, points.back().first ) )
                    {
                        if( plans.size() >= 3 )
                            break;

                        POINTS p = points;
                        p.push_back( { v, *arrive } );
                        plans.push_back( std::move( p ) );
                    }
                }
            }

            for( const POINTS& p : plans )
            {
                if( ok || aTries >= maxTries || overTime() )
                    break;

                ROUTE_REQUEST req{ a, b, bPos, ga->layer, PNS::RM_Walkaround, dryRun, {} };
                req.fronts = &fronts;
                req.tried = keepTries ? &aJob.tried : nullptr;
                req.neckWidth = neckFor( a, b );
                req.pairTail = pairTail( b );
                req.vias = p;
                req.noDoubleBack = true;
                req.exact = &p == legPlan;

                if( req.exact )
                    req.width = ga->width;

                if( PCB_TRACK* end = !ga->ring.empty() && !ga->via
                                             ? dynamic_cast<PCB_TRACK*>( board->ResolveItem( ga->ring.back(), true ) )
                                             : nullptr )
                {
                    req.from = end;
                    req.fromPos = end->GetEnd();
                }
                aAttempt = nlohmann::json::object();
                aTries++;
                portTries++;
                const auto portStarted = std::chrono::steady_clock::now();
                ok = routeOne( *context, req, aAttempt, added );
                portMs += msSince( portStarted );

                nlohmann::json as = { { "pad", endName( a ) }, { "side", ga->side }, { "layer", ga->via ? "other" : "pad" } };

                if( ga->via )
                    as["via_mm"] = { toMm( ga->via->x ), toMm( ga->via->y ) };

                if( gb )
                    as["far"] = farName;

                if( b == farRing )
                    as["far_ring_laid"] = true;

                if( ga->round )
                    as["round_the_part"] = true;

                if( !ga->via && ga->lane.size() >= 2 )
                    as["lane"] = ga->rank;

                if( req.from != a )
                    as["ring_laid"] = true;

                if( ok )
                {
                    portWon++;
                    aJob.laneRouted = req.from != a || ( farRing && b == farRing );

                    for( const PORT_GOAL* g : { ga, gb } )
                    {
                        const bool used = g == ga ? req.from != a : b == farRing && farRing;

                        for( const KIID& id : used ? g->ring : std::vector<KIID>() )
                        {
                            profile.laid[id] = { profile.pass, profile.job, profile.attempts };
                            added.push_back( id );
                        }
                    }

                    // the connection's copper now: ripped up with it
                    for( PORT_GOAL* g : { ga, gb } )
                    {
                        if( g )
                            g->ring.clear();
                    }

                    aAttempt["port"] = std::move( as );
                }
                else if( !portRefused && !req.diffPair && aAttempt.contains( "walls_in" )
                         && aAttempt["walls_in"].value( "port", false ) )
                {
                    portRefused = req;
                    portRefused->frontsInWorld = true;
                    portRefused->maxLength = 1.5 * pcbIUScale.mmToIU( aAttempt["walls_in"].value( "route_mm", 0.0 ) );
                    portRefusedAs = std::move( as );
                    portRefusedWall = aAttempt["walls_in"];
                }
            }
        };

        portTry();

        // a laid lane its route did not leave from: taken out again, the connection routed as any other
        for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( aJob.from ), aJob.to } )
        {
            auto it = !ok && end && end->Type() == PCB_PAD_T ? portGoals.find( static_cast<PAD*>( end ) ) : portGoals.end();

            if( it == portGoals.end() || it->second.ring.empty() )
                continue;

            BOARD_COMMIT commit( context->GetToolManager() );

            for( const KIID& id : it->second.ring )
            {
                BOARD_ITEM* t = board->ResolveItem( id, true );

                if( portRefused && ( portRefused->from == t || portRefused->to == t ) )
                    portRefused.reset();

                if( t )
                    commit.Remove( t );
            }

            commit.Push( _( "Lane taken out (API)" ) );
            it->second.ring.clear();
            lanesTakenOut++;
        }

        // the global plan: a way over the free space of the layers, the router follows its corners
        int  planned = -1;   // -1 no plan made, 0 none found, 1 found
        int  boxed = 0;      // the plan's end shut in a room of its own: 1 the start, 2 the target
        auto planTry = [&]()
        {
            if( ok || !planning || rail || ways.empty() || aTries >= maxTries )
                return;

            const WAY&               way = ways.front();
            const ROUTE_PLAN_REQUEST asked = planRequest( way.from, way.to, way.toPos );
            const ROUTE_PLAN         plan = makePlan( asked );

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

                        // copper a track of the plan's width passes beside at its clearance does not shut the end in
                        if( actual >= asked.trackWidth + std::max( netClearance( way.from, ds.m_MinClearance ),
                                                             netClearance( t, ds.m_MinClearance ) ) )
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

                // the cause, when one can be named: the clearance to its own row neighbours, else the ways
                // out kept open for other nets' fine-pitch pads (the plan finds a way without them)
                std::string cause;
                std::string causeKind;

                if( end->Type() == PCB_PAD_T && finePad( static_cast<PAD*>( end ) ) )
                {
                    if( const int fit = padEntryFit( board, static_cast<PAD*>( end ), way.from->GetNetCode() );
                        fit < ds.m_TrackMinWidth )
                    {
                        causeKind = "pad_clearance";
                        cause = "the clearance to its neighbour pads leaves " + ( fit == INT_MAX ? std::string( "no" ) : mmText( toMm( std::max( fit, 0 ) ) ) + " mm" )
                                + " for a track, under the board minimum " + mmText( toMm( ds.m_TrackMinWidth ) ) + " mm";
                    }
                }

                if( cause.empty() )
                {
                    ROUTE_PLAN_REQUEST open = asked;

                    if( !open.keepOut.empty() )
                    {
                        open.keepOut.clear();
                        const ROUTE_PLAN freed = makePlan( open );

                        if( freed.found || freed.boxedEnd != plan.boxedEnd )
                        {
                            std::string pads;
                            int         shown = 0, count = 0;
                            BOX2I       nearBox = end->GetBoundingBox();
                            nearBox.Inflate( pcbIUScale.mmToIU( 1.5 ) );

                            for( const ESCAPE_FRONT& front : fronts )
                            {
                                if( !front.walls( way.from->GetNetCode() ) || !nearBox.Intersects( front.shape.BBox() ) )
                                {
                                    continue;
                                }

                                count++;

                                if( shown++ < 6 )
                                    pads += ( pads.empty() ? "" : ", " ) + endName( front.pad );
                            }

                            causeKind = "escapes_kept";
                            cause = "the ways out kept open for other nets' fine-pitch pads shut it in"
                                    + ( pads.empty() ? std::string() : " (" + pads + ( count > shown ? ", ..." : "" ) + ")" )
                                    + "; escapes: false or routing those pads first frees it";
                        }
                    }
                }

                aAttempt["boxed_in"] = { { "end", endName( end ) },
                                         { "which", plan.boxedEnd == 1 ? "start" : "target" },
                                         { "room_cells", plan.boxedRoom },
                                         { "by", by } };

                if( !causeKind.empty() )
                    aAttempt["boxed_in"]["cause"] = causeKind;

                aAttempt["boxed_reason"] = std::string( plan.boxedEnd == 1 ? "start " : "target " ) + endName( end )
                                           + " is boxed in: no way out of it at this width and clearance"
                                           + ( cause.empty() ? std::string() : ": " + cause )
                                           + ( list.empty() ? std::string() : ", by " + list );
            }

            if( plan.found )
            {
                ROUTE_REQUEST req{ way.from, way.to, way.toPos, plan.startLayer, PNS::RM_Walkaround, dryRun, {} };
                req.fronts = &fronts;
                req.tried = keepTries ? &aJob.tried : nullptr;
                req.neckWidth = neckFor( way.from, way.to );
                req.pairTail = pairTail( way.to );
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
                    req.tried = keepTries ? &aJob.tried : nullptr;
                    req.neckWidth = neckFor( way.from, way.to );
                    req.pairTail = pairTail( way.to );
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
                req.tried = keepTries ? &aJob.tried : nullptr;
                req.neckWidth = neckFor( way.from, way.to );
                req.pairTail = pairTail( way.to );
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

            auto stopOf = [&]()
            {
                const nlohmann::json& t = aAttempt["stopped_at_mm"];
                return VECTOR2I( pcbIUScale.mmToIU( t[0].get<double>() ), pcbIUScale.mmToIU( t[1].get<double>() ) );
            };

            auto stoppedAt = [&]( const VECTOR2I& aVia )
            {
                if( !aAttempt.contains( "stopped_at_mm" ) || aAttempt.value( "routed", false ) )
                    return false;

                const VECTOR2I d = stopOf() - aVia;
                return std::abs( d.x ) <= pcbIUScale.mmToIU( 0.05 ) && std::abs( d.y ) <= pcbIUScale.mmToIU( 0.05 );
            };

            auto stoppedShort = [&]( const VECTOR2I& aVia )
            {
                return aAttempt.contains( "stopped_at_mm" ) && !aAttempt.value( "routed", false )
                       && ( stopOf() - aVia ).EuclideanNorm() > pcbIUScale.mmToIU( 3.0 );
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
                        req.tried = keepTries ? &aJob.tried : nullptr;
                        req.neckWidth = neckFor( way.from, way.to );
                        req.pairTail = pairTail( way.to );
                        req.vias = { { v, arrive } };
                        aAttempt = nlohmann::json::object();
                        aTries++;
                        ok = routeOne( *context, req, aAttempt, added );

                        if( ok )
                            noteInward( req, aAttempt );

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

                // a crossing leg that stopped at its first via or well short of the second (the second
                // vias all lie near the far end): that first via fails there whatever the second via
                // is; two first vias stopped at the same spot: the layer is shut there for every one
                std::set<std::pair<int, int>> deadCrossing;
                std::vector<VECTOR2I>         crossStops;
                bool                          crossShut = false;

                for( const VECTOR2I& va : starts )
                {
                    for( const VECTOR2I& vb : ends )
                    {
                        if( deadFirstVia.count( { va.x, va.y } ) || deadCrossing.count( { va.x, va.y } ) )
                        {
                            profile.deadLegsSkipped++;
                            break;
                        }

                        if( ok || crossShut || aTries >= maxTries || budget <= 0 || overTime() )
                            break;

                        if( deadLastVia.count( { vb.x, vb.y } ) )
                        {
                            profile.deadLegsSkipped++;
                            continue;
                        }

                        budget--;

                        ROUTE_REQUEST req{ way.from, way.to, way.toPos, start, PNS::RM_Walkaround, dryRun, {} };
                        req.fronts = &fronts;
                        req.tried = keepTries ? &aJob.tried : nullptr;
                        req.neckWidth = neckFor( way.from, way.to );
                        req.pairTail = pairTail( way.to );
                        req.vias = { { va, cross }, { vb, arrive } };
                        aAttempt = nlohmann::json::object();
                        aTries++;
                        ok = routeOne( *context, req, aAttempt, added );

                        if( ok )
                            noteInward( req, aAttempt );

                        if( !ok && stoppedBefore( va ) )
                            deadFirstVia.insert( { va.x, va.y } );
                        else if( !ok && stoppedBefore( vb ) && ( stoppedAt( va ) || stoppedShort( vb ) ) )
                        {
                            const VECTOR2I at = stopOf();
                            crossShut = std::any_of( crossStops.begin(), crossStops.end(), [&]( const VECTOR2I& p )
                                                     { return ( p - at ).EuclideanNorm() <= pcbIUScale.mmToIU( 0.5 ); } );
                            crossStops.push_back( at );
                            deadCrossing.insert( { va.x, va.y } );
                        }
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

        profile.wallRefused = nullptr;

        if( !ok && ( portRefused || wallRefused ) && aTries < maxTries && !overTime() )
        {
            const bool     port = portRefused.has_value();
            nlohmann::json again = nlohmann::json::object();
            aTries++;
            portTries += port;
            profile.frontRetries++;
            const auto started = std::chrono::steady_clock::now();
            ok = routeOne( *context, port ? *portRefused : *wallRefused, again, added );

            if( port )
                portMs += msSince( started );

            again["walls_in_first"] = port ? portRefusedWall : wallRefusedAt;

            if( ok )
            {
                portWon += port;
                profile.frontRetriesWon++;

                if( port )
                    again["port"] = std::move( portRefusedAs );

                aAttempt = std::move( again );
            }
        }

        if( !ok && overTime() )
        {
            const std::string budget = plateauTurnCut ? std::string( "stop_on_plateau_s" ) : budgetName;

            if( !KopenapiJobCancelled() )
            {
                profile.budgetSpent[profile.pass]++;
                aAttempt["budget"] = budget;
            }

            aAttempt["reason"] = std::string( KopenapiJobCancelled() ? ( plateauStopped ? "stopped (stop_on_plateau_s)" : "stopped (job_cancel)" ) : "the connection's " + budget + " ran out" )
                                 + " after " + std::to_string( aTries ) + " tries; last: " + aAttempt.value( "reason", std::string( "-" ) );
        }
        else if( !ok && aTries >= maxTries )
            aAttempt["reason"] = "gave up after " + std::to_string( aTries ) + " tries (max_tries); last: "
                                 + aAttempt.value( "reason", std::string( "-" ) );

        return ok;
    };

    // Differential pairs: for a net named as the P side of a pair (+ or P suffix) whose N net is
    // routed too, a trunk between each two footprints next in a chain over the footprints with pads of
    // both nets (connector - ESD - MCU), from the P and N pads nearest each other there.  The trunks
    // go first and coupled; airwires their copper joins afterwards are not routed again.
    std::vector<JOB>   trunks;
    nlohmann::json     pairsSkipped = nlohmann::json::array();
    std::map<int, int> explicitPairs;   // the pairs handed to the router (P net code -> N)

    if( diffPairs == "auto" && !rail )
    {
        std::set<std::string> jobNets;

        for( const JOB& job : jobs )
            jobNets.insert( job.net );

        // the pairs: KiCad's name rule (+/-, P/N), USB's DP / DM naming (USB_DP, USB_DP_MCU), and the
        // pairs pcb_net_roles finds; each net in one pair, both nets routed by this call
        std::vector<std::pair<std::string, std::string>> pairNames;
        std::set<std::string>                            inPair;

        auto addPair = [&]( const std::string& aP, const std::string& aN )
        {
            if( aP == aN || inPair.count( aP ) || inPair.count( aN ) || !jobNets.count( aP ) || !jobNets.count( aN ) )
                return;

            pairNames.emplace_back( aP, aN );
            inPair.insert( aP );
            inPair.insert( aN );
        };

        for( const std::string& netP : jobNets )
        {
            const std::string netN = KopenapiPairPartner( board, netP,
                    [&]( const std::string& aName ) { return jobNets.count( aName ) > 0; } );

            if( !netN.empty() )
                addPair( netP, netN );
        }

        if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
        {
            const KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

            if( r.status < 400 && r.body.contains( "pairs" ) && r.body["pairs"].is_array() )
            {
                for( const nlohmann::json& pr : r.body["pairs"] )
                {
                    if( pr.is_object() && pr.value( "p", nlohmann::json() ).is_string() && pr.value( "n", nlohmann::json() ).is_string() )
                        addPair( pr["p"].get<std::string>(), pr["n"].get<std::string>() );
                }
            }
        }

        for( const auto& [netP, otherName] : pairNames )
        {
            const wxString other = wxString::FromUTF8( otherName );
            NETINFO_ITEM* infoP = KopenapiFindNet( board, netP );
            NETINFO_ITEM* infoN = KopenapiFindNet( board, str( other ) );

            if( !infoP || !infoN )
                continue;

            explicitPairs[infoP->GetNetCode()] = infoN->GetNetCode();

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

                        if( pad->GetParentFootprint() != aFrom->GetParentFootprint()
                            && ( gap < bestGap
                                 || ( gap == bestGap && best
                                      && std::make_tuple( pad->GetPosition().x, pad->GetPosition().y )
                                                 < std::make_tuple( best->GetPosition().x, best->GetPosition().y ) ) )
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

            // a chain: from the end farthest from the middle, each time to the nearest left; ties (two
            // ends are always as far from their middle) go to the package with more pads, then by place
            VECTOR2I middle( 0, 0 );

            for( const auto& [p, n] : ends )
                middle += p->GetPosition() / (int) ends.size();

            auto chainKey = [&]( const std::pair<PAD*, PAD*>& aEnd, const VECTOR2I& aFrom, bool aFar )
            {
                const VECTOR2I at = aEnd.first->GetPosition();
                const int64_t  um = KiROUND( ( at - aFrom ).EuclideanNorm() / pcbIUScale.mmToIU( 0.001 ) );
                const FOOTPRINT* fp = aEnd.first->GetParentFootprint();
                const int64_t  pads = fp ? (int64_t) fp->Pads().size() : 0;

                return std::make_tuple( aFar ? -um : um, -pads, at.x, at.y );
            };

            std::sort( ends.begin(), ends.end(), [&]( const auto& x, const auto& y )
                       { return chainKey( x, middle, true ) < chainKey( y, middle, true ); } );

            for( size_t k = 1; k < ends.size(); ++k )
            {
                const VECTOR2I last = ends[k - 1].first->GetPosition();

                std::sort( ends.begin() + k, ends.end(), [&]( const auto& x, const auto& y )
                           { return chainKey( x, last, false ) < chainKey( y, last, false ); } );
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

    // bus: the members of a bundle leaving the same side of a package one after another in pin order
    // (the pin on the inside of the turn to their far ends first: each next one runs beside the last,
    // uncrossed); bus_layer: each member drops a through via on a diagonal fan in front of the row
    // (via pitch apart, the vias of the later members kept free while they wait) and goes on on that
    // layer; a member the fan does not route is routed as any connection
    nlohmann::json            busOut = nlohmann::json::array();
    std::optional<PCB_LAYER_ID> busLayer;

    // ports: the packages whose signal pins follow the layout plan; true: every package with 8 SMD pads or more
    std::optional<std::vector<FOOTPRINT*>> portRefs;

    if( aArgs.contains( "ports" ) )
    {
        const nlohmann::json& arg = aArgs["ports"];

        if( arg.is_boolean() )
        {
            if( arg.get<bool>() )
            {
                portRefs.emplace();

                for( FOOTPRINT* fp : board->Footprints() )
                {
                    const size_t smd = std::count_if( fp->Pads().begin(), fp->Pads().end(),
                                                      []( PAD* p ) { return p->GetAttribute() == PAD_ATTRIB::SMD; } );

                    if( smd >= 8 )
                        portRefs->push_back( fp );
                }
            }
        }
        else if( arg.is_array() )
        {
            portRefs.emplace();

            for( const nlohmann::json& r : arg )
            {
                FOOTPRINT* fp = r.is_string() ? board->FindFootprintByReference( wxString::FromUTF8( r.get<std::string>() ) ) : nullptr;

                if( !fp )
                    return KOPENAPI_RESULT::Error( 404, "ports: no footprint " + ( r.is_string() ? r.get<std::string>() : r.dump() ) );

                portRefs->push_back( fp );
            }
        }
        else
        {
            return KOPENAPI_RESULT::Error( 400, "ports: true (every package with 8 SMD pads or more) or [footprint references]" );
        }
    }

    // lanes: with ports, each pin planned on its pad's layer runs the lane pcb_fanout_ports lanes plans round its
    // package, the outermost first
    lanesOn = portRefs.has_value() && aArgs.value( "lanes", false );

    if( aArgs.contains( "bus_layer" ) )
    {
        busLayer = aArgs["bus_layer"].is_string() ? parseLayer( board, aArgs["bus_layer"].get<std::string>() ) : std::nullopt;

        if( !busLayer || std::find( layers.begin(), layers.end(), *busLayer ) == layers.end() )
            return KOPENAPI_RESULT::Error( 400, "bus_layer: one of the call's layers" );
    }

    {
        const nlohmann::json busArg = aArgs.contains( "bus" ) ? aArgs["bus"] : nlohmann::json( "off" );
        std::vector<std::pair<std::string, std::vector<std::string>>> busList;

        if( busArg.is_string() && busArg.get<std::string>() == "auto" )
        {
            if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
            {
                const KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

                if( r.status < 400 && r.body.contains( "bundles" ) && r.body["bundles"].is_array() )
                {
                    for( const auto& b : r.body["bundles"] )
                    {
                        std::vector<std::string> members;

                        for( const auto& m : b.value( "members", nlohmann::json::array() ) )
                        {
                            for( const std::string& n : m.value( "nets", std::vector<std::string>() ) )
                                members.push_back( n );
                        }

                        busList.emplace_back( b.value( "name", std::string() ), std::move( members ) );
                    }
                }
            }
        }
        else if( busArg.is_array() )
        {
            for( const auto& b : busArg )
            {
                std::vector<std::string> members;

                for( const auto& n : b.value( "nets", nlohmann::json::array() ) )
                {
                    if( !n.is_string() )
                        return KOPENAPI_RESULT::Error( 400, "bus: [{name, nets: [net names]}]" );

                    if( !KopenapiFindNet( board, n.get<std::string>() ) )
                        return KopenapiNoSuchNet( board, n.get<std::string>(), "bus: " );

                    members.push_back( n.get<std::string>() );
                }

                busList.emplace_back( b.value( "name", std::string( "bus" ) + std::to_string( busList.size() + 1 ) ), std::move( members ) );
            }
        }
        else if( !( busArg.is_string() && busArg.get<std::string>() == "off" ) )
        {
            return KOPENAPI_RESULT::Error( 400, "bus: \"off\", \"auto\" (pcb_net_roles' bundles) or [{name, nets}]" );
        }

        if( grid && !busList.empty() )
        {
            busOut.push_back( { { "skipped", "engine grid: its bundles (the bundles argument) route the members side by side" } } );
            busList.clear();
        }

        std::set<size_t> taken;   // jobs already in a bus

        for( const auto& [name, members] : busList )
        {
            std::set<int> codes;

            for( const std::string& n : members )
            {
                if( NETINFO_ITEM* info = KopenapiFindNet( board, n ) )
                    codes.insert( info->GetNetCode() );
            }

            // the part the members leave: the package with the most of their ends (the finer pitch on a tie)
            std::map<FOOTPRINT*, int> endsAt;

            for( size_t k = 0; k < jobs.size(); ++k )
            {
                if( taken.count( k ) || !codes.count( jobs[k].from->GetNetCode() ) )
                    continue;

                for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( jobs[k].from ), jobs[k].to } )
                {
                    if( FOOTPRINT* fp = end->Type() == PCB_PAD_T ? static_cast<PAD*>( end )->GetParentFootprint() : nullptr;
                        fp && fp->Pads().size() >= 4 )
                    {
                        endsAt[fp]++;
                    }
                }
            }

            FOOTPRINT* source = nullptr;

            for( const auto& [fp, n] : endsAt )
            {
                if( !source || n > endsAt[source]
                    || ( n == endsAt[source] && pitchOf( fp->Pads().front() ) < pitchOf( source->Pads().front() ) )
                    || ( n == endsAt[source] && pitchOf( fp->Pads().front() ) == pitchOf( source->Pads().front() )
                         && fp->GetReference() < source->GetReference() ) )
                {
                    source = fp;
                }
            }

            nlohmann::json out = { { "name", name } };

            if( !source || endsAt[source] < 2 )
            {
                out["skipped"] = "fewer than two of its connections leave one package";
                busOut.push_back( out );
                continue;
            }

            // one connection per member net, leaving the source part (its end there made the start)
            std::map<int, size_t> memberJob;

            for( size_t k = 0; k < jobs.size(); ++k )
            {
                JOB& j = jobs[k];

                if( taken.count( k ) || !codes.count( j.from->GetNetCode() ) || memberJob.count( j.from->GetNetCode() ) )
                    continue;

                if( j.from->GetParentFootprint() != source )
                {
                    PAD* farPad = dynamic_cast<PAD*>( j.to );

                    if( !farPad || farPad->GetParentFootprint() != source )
                        continue;

                    PAD* nearPad = j.from;
                    j.from = farPad;
                    j.to = nearPad;
                    std::swap( j.fromId, j.toId );
                    j.toPos = nearPad->GetPosition();
                }

                memberJob[j.from->GetNetCode()] = k;
            }

            BOX2I centres( source->Pads().front()->GetPosition(), VECTOR2I( 0, 0 ) );

            for( PAD* pad : source->Pads() )
                centres.Merge( pad->GetPosition() );

            // per side of the package: the rows the members leave
            std::map<std::pair<int, int>, std::vector<size_t>> bySide;

            for( const auto& [code, k] : memberJob )
            {
                if( std::optional<VECTOR2I> dir = outwardOf( jobs[k].from, centres ) )
                    bySide[{ dir->x, dir->y }].push_back( k );
            }

            for( auto& [dirKey, list] : bySide )
            {
                if( list.size() < 2 )
                    continue;

                const VECTOR2I dir( dirKey.first, dirKey.second );
                const VECTOR2I tangent( dir.y != 0 ? 1 : 0, dir.x != 0 ? 1 : 0 );
                auto along = [&]( const VECTOR2I& aAt ) { return (int64_t) aAt.x * tangent.x + (int64_t) aAt.y * tangent.y; };
                auto across = [&]( const VECTOR2I& aAt ) { return (int64_t) aAt.x * dir.x + (int64_t) aAt.y * dir.y; };

                int64_t mid = 0, farSide = 0;

                for( size_t k : list )
                {
                    mid += along( jobs[k].from->GetPosition() ) / (int64_t) list.size();
                    farSide += ( along( jobs[k].toPos ) - along( jobs[k].from->GetPosition() ) ) / (int64_t) list.size();
                }

                // the inside of the turn first; ties by the pin's place
                std::sort( list.begin(), list.end(), [&]( size_t x, size_t y )
                           {
                               const int64_t ax = along( jobs[x].from->GetPosition() ), ay = along( jobs[y].from->GetPosition() );
                               return farSide >= 0 ? ax > ay : ax < ay;
                           } );

                const int busIndexOut = (int) busOut.size();
                nlohmann::json sideOut = { { "name", name },
                                           { "part", str( source->GetReference() ) },
                                           { "side", dir.x < 0 ? "left" : dir.x > 0 ? "right" : dir.y < 0 ? "top" : "bottom" },
                                           { "nets", nlohmann::json::array() } };

                for( size_t m = 0; m < list.size(); ++m )
                {
                    JOB& j = jobs[list[m]];
                    j.bus = busIndexOut;
                    j.busIndex = (int) m;
                    taken.insert( list[m] );
                    sideOut["nets"].push_back( j.net );
                }

                PAD* first = jobs[list.front()].from;
                const PCB_LAYER_ID padLayer = first->IsOnLayer( F_Cu ) ? F_Cu : first->IsOnLayer( B_Cu ) ? B_Cu
                                                                                                        : first->GetLayerSet().CuStack().front();

                if( busLayer && *busLayer != padLayer && viasAllowed )
                {
                    NETCLASS* netclass = first->GetEffectiveNetClass();
                    const int width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : ds.GetCurrentTrackWidth();
                    const int dia = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : ds.GetCurrentViaSize();
                    const int drill = netclass && netclass->HasViaDrill() ? netclass->GetViaDrill() : ds.GetCurrentViaDrill();
                    const int clearance = std::max( { ds.m_MinClearance, netclass && netclass->HasClearance() ? netclass->GetClearance() : 0, 1 } );
                    const double viaPitch = std::max( double( dia + clearance ), double( drill + std::max( ds.m_HoleToHoleMin, 0 ) ) );
                    const double need = dia / 2.0 + clearance + width / 2.0;   // a track beside a via

                    int64_t tipLine = std::numeric_limits<int64_t>::min();
                    double  minGap = std::numeric_limits<double>::max();
                    int64_t maxOffset = 0;

                    for( size_t m = 0; m < list.size(); ++m )
                    {
                        PAD*        pad = jobs[list[m]].from;
                        const BOX2I pb = pad->GetBoundingBox();
                        tipLine = std::max( tipLine, across( pad->GetPosition() ) + ( dir.x != 0 ? pb.GetWidth() : pb.GetHeight() ) / 2 );
                        maxOffset = std::max( maxOffset, std::abs( along( pad->GetPosition() ) - mid ) );

                        if( m > 0 )
                            minGap = std::min( minGap, (double) std::abs( along( pad->GetPosition() ) - along( jobs[list[m - 1]].from->GetPosition() ) ) );
                    }

                    // the lines spread (45 degrees at most) until a line passes the via before it
                    const double  spread = std::max( 1.0, need / std::max( 1.0, minGap ) );
                    const int64_t rayLength = KiROUND( maxOffset * ( spread - 1.0 ) );
                    double        fanAcross = (double) tipLine + rayLength + dia / 2.0 + clearance;
                    nlohmann::json fanSpots = nlohmann::json::array();

                    for( size_t m = 0; m < list.size(); ++m )
                    {
                        JOB&          j = jobs[list[m]];
                        const int64_t fanned = mid + KiROUND( ( along( j.from->GetPosition() ) - mid ) * spread );

                        if( m > 0 )
                        {
                            const double df = std::abs( (double) fanned
                                                        - ( mid + KiROUND( ( along( jobs[list[m - 1]].from->GetPosition() ) - mid ) * spread ) ) );
                            fanAcross += std::max( need, std::sqrt( std::max( 0.0, viaPitch * viaPitch - df * df ) ) );
                        }

                        const int64_t c = KiROUND( fanAcross );
                        j.busVia = VECTOR2I( (int) ( fanned * tangent.x + c * dir.x ), (int) ( fanned * tangent.y + c * dir.y ) );
                        fanSpots.push_back( { toMm( j.busVia->x ), toMm( j.busVia->y ) } );
                    }

                    sideOut["via_layer"] = str( board->GetLayerName( *busLayer ) );
                    sideOut["vias_mm"] = fanSpots;
                    sideOut["via_pitch_mm"] = toMm( KiROUND( viaPitch ) );
                }
                else if( busLayer && *busLayer != padLayer )
                {
                    sideOut["no_via_fan"] = "vias: false or one layer";
                }

                busOut.push_back( sideOut );
            }
        }

        // the members one after another where the first of them was in the order
        if( !taken.empty() )
        {
            std::vector<JOB> ordered;
            std::set<int>    emitted;

            for( size_t k = 0; k < jobs.size(); ++k )
            {
                if( jobs[k].bus < 0 )
                {
                    ordered.push_back( jobs[k] );
                    continue;
                }

                if( !emitted.insert( jobs[k].bus ).second )
                    continue;

                std::vector<const JOB*> members;

                for( const JOB& j : jobs )
                {
                    if( j.bus == jobs[k].bus )
                        members.push_back( &j );
                }

                std::sort( members.begin(), members.end(), []( const JOB* x, const JOB* y ) { return x->busIndex < y->busIndex; } );

                for( const JOB* j : members )
                    ordered.push_back( *j );
            }

            jobs = std::move( ordered );
        }
    }

    // a package's signal pins' ports from its plan (pcb_fanout_ports' ports): the way out on the pin's
    // side to past the courtyard on the pad's layer, or to the other layer (its nets in aViaNets); an
    // edge connector's fingers (CONN pads) as its SMD pads, on the side away from the board edge
    auto goalsFrom = [&]( FOOTPRINT* fp, const nlohmann::json& aPorts, std::map<const PAD*, PORT_GOAL>& aGoals,
                          std::vector<std::string>& aViaNets )
    {
        const BOX2I courtyard = fp->GetCourtyard( fp->GetLayer() == B_Cu ? B_CrtYd : F_CrtYd ).OutlineCount()
                                        ? fp->GetCourtyard( fp->GetLayer() == B_Cu ? B_CrtYd : F_CrtYd ).BBox()
                                        : fp->GetBoundingBox( false );
        int         planned = 0;

        for( const auto& [side, list] : aPorts.items() )
        {
            const VECTOR2I dir = side == "west" ? VECTOR2I( -1, 0 ) : side == "east" ? VECTOR2I( 1, 0 )
                                 : side == "north" ? VECTOR2I( 0, -1 ) : side == "south" ? VECTOR2I( 0, 1 ) : VECTOR2I( 0, 0 );

            if( dir == VECTOR2I( 0, 0 ) || !list.is_array() )
                continue;

            for( const nlohmann::json& port : list )
            {
                if( port.value( "kind", std::string() ) != "signal" )
                    continue;

                const wxString number = wxString::FromUTF8( port.value( "pad", std::string() ) );

                for( PAD* pad : fp->Pads() )
                {
                    if( pad->GetNumber() != number || pad->GetNetCode() <= 0
                        || ( pad->GetAttribute() != PAD_ATTRIB::SMD && pad->GetAttribute() != PAD_ATTRIB::CONN ) )
                        continue;

                    PORT_GOAL goal;
                    goal.layer = pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
                    goal.dir = dir;
                    goal.side = side;
                    goal.to = port.value( "to", std::string() );
                    goal.other = port.value( "layer", std::string() ) == "other";
                    goal.round = port.value( "turn", std::string() ) == "round_the_part";

                    for( PCB_LAYER_ID l : layers )
                    {
                        if( l != goal.layer )
                        {
                            goal.otherLayer = l;
                            break;
                        }
                    }

                    if( goal.other && !( viasAllowed && goal.otherLayer ) )
                        goal.other = false;

                    // the plan's own spot for the via (lanes: the spot they keep clear of)
                    if( const nlohmann::json& at = port.contains( "via_mm" ) ? port["via_mm"] : nlohmann::json();
                        goal.other && at.is_array() && at.size() == 2 && at[0].is_number() && at[1].is_number() )
                    {
                        goal.via = VECTOR2I( pcbIUScale.mmToIU( at[0].get<double>() ), pcbIUScale.mmToIU( at[1].get<double>() ) );
                        goal.viaPlanned = true;
                    }

                    NETCLASS*      netclass = pad->GetEffectiveNetClass();
                    const int      clearance = std::max( { ds.m_MinClearance, netclass && netclass->HasClearance() ? netclass->GetClearance() : 0, 1 } );
                    const BOX2I    box = pad->GetBoundingBox();
                    const int      half = dir.x != 0 ? box.GetWidth() / 2 : box.GetHeight() / 2;
                    const int      across = dir.x != 0 ? box.GetHeight() : box.GetWidth();
                    const int      width = std::min( across, netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : ds.GetCurrentTrackWidth() );
                    const VECTOR2I end = box.Centre() + dir * half;
                    const int64_t  edge = dir.x != 0 ? ( dir.x > 0 ? courtyard.GetRight() : -(int64_t) courtyard.GetLeft() )
                                                     : ( dir.y > 0 ? courtyard.GetBottom() : -(int64_t) courtyard.GetTop() );
                    const int64_t  at = (int64_t) end.x * dir.x + (int64_t) end.y * dir.y;
                    // round the part: kept open only to the courtyard, the neighbours turn right past it
                    const int      reach = (int) std::max<int64_t>( width, goal.round ? edge - at : edge - at + clearance + width / 2 );

                    goal.width = width;
                    goal.start = end + dir * ( width / 2 );
                    goal.tip = end + dir * reach;

                    // its lane: the way out reaches the lane's offset, the route follows the lane's corners
                    static const nlohmann::json none;

                    if( const nlohmann::json& lane = port.contains( "lane" ) ? port["lane"] : none;
                        !goal.other && lane.is_object() && lane.contains( "polyline_mm" ) && lane["polyline_mm"].is_array()
                        && lane["polyline_mm"].size() >= 2 )
                    {
                        for( const nlohmann::json& pt : lane["polyline_mm"] )
                        {
                            if( pt.is_array() && pt.size() == 2 && pt[0].is_number() && pt[1].is_number() )
                                goal.lane.emplace_back( pcbIUScale.mmToIU( pt[0].get<double>() ), pcbIUScale.mmToIU( pt[1].get<double>() ) );
                        }

                        goal.rank = lane.value( "rank", 0 );

                        if( goal.lane.size() >= 2 )
                            goal.tip = goal.lane[1];
                        else
                            goal.lane.clear();
                    }

                    aGoals[pad] = goal;
                    planned++;

                    if( goal.other )
                        aViaNets.push_back( str( UnescapeString( pad->GetNetname() ) ) );
                }
            }
        }

        return planned;
    };

    // ports: the layout's plan for the signal pins of the packages asked (pcb_fanout_ports): each pin's
    // way out runs on its side to past the courtyard on the pad's layer, or to the via spot the plan
    // gives (layer other, as pcb_fanout ports would lay it); its routes leave through it
    nlohmann::json portsOut = nlohmann::json::array();

    if( portRefs && !rail )
    {
        std::optional<KOPENAPI_METHOD> planner = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout_ports" );
        std::optional<KOPENAPI_METHOD> fanout = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout" );

        for( FOOTPRINT* fp : *portRefs )
        {
            nlohmann::json pkg = { { "ref", str( fp->GetReference() ) } };

            if( !planner )
            {
                pkg["error"] = "pcb_fanout_ports is not registered";
                portsOut.push_back( pkg );
                continue;
            }

            const KOPENAPI_RESULT plan = planner->handler( aCtx, { { "ref", str( fp->GetReference() ) }, { "lanes", lanesOn } } );

            if( plan.status != 200 || !plan.body.contains( "ports" ) || !plan.body["ports"].is_object() )
            {
                pkg["error"] = plan.body.value( "error", std::string( "no plan" ) );
                portsOut.push_back( pkg );
                continue;
            }

            std::vector<std::string> viaNets;
            const int                planned = goalsFrom( fp, plan.body["ports"], portGoals, viaNets );

            // the via spots the plan's fan-out would lay (dry run): the spot of each moved pin
            int fanSpots = 0;

            if( !viaNets.empty() && fanout )
            {
                const KOPENAPI_RESULT fan = fanout->handler( aCtx, { { "ref", str( fp->GetReference() ) }, { "ports", true },
                                                                     { "stubs", false }, { "dry_run", true }, { "nets", viaNets },
                                                                     { "full", true }, { "zones", "keep" } } );

                if( fan.status == 200 && fan.body.contains( "pins" ) && fan.body["pins"].is_array() )
                {
                    for( const nlohmann::json& row : fan.body["pins"] )
                    {
                        const std::string name = row.value( "pad", std::string() );
                        const std::string number = name.substr( name.rfind( '.' ) + 1 );

                        if( !row.contains( "row" ) || !row.contains( "end_mm" ) )
                            continue;

                        for( auto& [pad, goal] : portGoals )
                        {
                            if( pad->GetParentFootprint() == fp && goal.other && !goal.via && str( pad->GetNumber() ) == number )
                            {
                                goal.via = VECTOR2I( pcbIUScale.mmToIU( row["end_mm"][0].get<double>() ),
                                                     pcbIUScale.mmToIU( row["end_mm"][1].get<double>() ) );
                                fanSpots++;
                            }
                        }
                    }
                }
            }

            // a moved pin with no spot from the fan-out: a via past the tips of its neighbours' ways out
            for( auto& [pad, goal] : portGoals )
            {
                if( pad->GetParentFootprint() != fp || !goal.other || goal.via )
                    continue;

                NETCLASS* netclass = pad->GetEffectiveNetClass();
                const int clearance = std::max( { ds.m_MinClearance, netclass && netclass->HasClearance() ? netclass->GetClearance() : 0, 1 } );
                const int dia = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : ds.GetCurrentViaSize();
                goal.via = goal.tip + goal.dir * ( goal.width / 2 + clearance + dia / 2 );
            }

            pkg["pins"] = planned;
            pkg["vias"] = viaNets.size();
            pkg["fan_spots"] = fanSpots;
            portsOut.push_back( pkg );
        }
    }

    // the connections leaving one side of a package for one part through planned ports on its layer, one after
    // another from the inside of their turn (as a bus), each in the place one of them had: a later one
    // runs beside the last, not across the ways out still waiting
    if( !portGoals.empty() )
    {
        std::map<std::tuple<const FOOTPRINT*, std::string, std::string>, std::vector<size_t>> bySide;
        std::vector<const PAD*>                                                 portEnd( jobs.size(), nullptr );

        // across a lane's way out of the ring, towards its package
        auto laneInward = []( const PAD* aPad, const PORT_GOAL& aGoal )
        {
            const VECTOR2I e = aGoal.lane.back() - aGoal.lane[aGoal.lane.size() - 2];
            const VECTOR2I across( e.y != 0 ? 1 : 0, e.x != 0 ? 1 : 0 );
            const VECTOR2I toPart = aPad->GetParentFootprint()->GetPosition() - aGoal.lane.back();
            return (int64_t) toPart.x * across.x + (int64_t) toPart.y * across.y >= 0 ? across : across * -1;
        };

        for( size_t k = 0; k < jobs.size(); ++k )
        {
            if( jobs[k].bus >= 0 )
                continue;

            for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( jobs[k].from ), jobs[k].to } )
            {
                const PAD* pad = end && end->Type() == PCB_PAD_T ? static_cast<const PAD*>( end ) : nullptr;
                auto       it = pad ? portGoals.find( pad ) : portGoals.end();

                if( it == portGoals.end() || it->second.via )
                    continue;

                if( !portEnd[k]
                    || ( lanesOn ? !it->second.lane.empty()
                                           && ( portGoals[portEnd[k]].lane.empty()
                                                || setsOrder( pad->GetParentFootprint(), portEnd[k]->GetParentFootprint() ) )
                                 : pad->GetParentFootprint()->Pads().size()
                                           > portEnd[k]->GetParentFootprint()->Pads().size() ) )
                {
                    portEnd[k] = pad;
                }
            }

            // lanes turning out of the ring: by the corner they leave and the way they leave it
            if( const PORT_GOAL& g = portEnd[k] ? portGoals[portEnd[k]] : PORT_GOAL(); lanesOn && g.lane.size() >= 4 )
            {
                const VECTOR2I e = g.lane.back() - g.lane[g.lane.size() - 2];
                const VECTOR2I way( ( e.x > 0 ) - ( e.x < 0 ), ( e.y > 0 ) - ( e.y < 0 ) );
                const VECTOR2I inward = laneInward( portEnd[k], g );
                bySide[{ portEnd[k]->GetParentFootprint(), "lane",
                         std::to_string( way.x ) + "," + std::to_string( way.y ) + "," + std::to_string( inward.x ) + ","
                                 + std::to_string( inward.y ) }].push_back( k );
            }
            else if( portEnd[k] )
                bySide[{ portEnd[k]->GetParentFootprint(), portGoals[portEnd[k]].side, portGoals[portEnd[k]].to }].push_back( k );
        }

        for( auto& [key, list] : bySide )
        {
            if( list.size() < 2 )
                continue;

            // lanes leaving one corner one way: their ring is fixed, the legs after it are not: the lane on the
            // inside of the legs' turn first (the innermost when they turn back towards the package, else the
            // outermost), so no leg runs across the end of a lane still waiting
            if( std::get<1>( key ) == "lane" )
            {
                int64_t towards = 0;

                for( size_t k : list )
                {
                    const PORT_GOAL& g = portGoals[portEnd[k]];
                    const PAD*       farPad = static_cast<const PAD*>( portEnd[k] == jobs[k].from ? nullptr : jobs[k].from );
                    VECTOR2I         farAt = portEnd[k] == jobs[k].from ? jobs[k].toPos : jobs[k].from->GetPosition();

                    if( portEnd[k] == jobs[k].from && jobs[k].to && jobs[k].to->Type() == PCB_PAD_T )
                        farPad = static_cast<const PAD*>( jobs[k].to );

                    if( auto it = farPad ? portGoals.find( farPad ) : portGoals.end(); it != portGoals.end() && !it->second.lane.empty() )
                        farAt = it->second.lane.back();

                    const VECTOR2I inward = laneInward( portEnd[k], g );
                    towards += ( (int64_t) ( farAt - g.lane.back() ).x * inward.x + (int64_t) ( farAt - g.lane.back() ).y * inward.y )
                               / (int64_t) list.size();
                }

                std::vector<size_t> sorted = list;
                std::stable_sort( sorted.begin(), sorted.end(),
                                  [&]( size_t x, size_t y )
                                  {
                                      return towards > 0 ? portGoals[portEnd[x]].rank < portGoals[portEnd[y]].rank
                                                         : portGoals[portEnd[x]].rank > portGoals[portEnd[y]].rank;
                                  } );
                std::vector<JOB> moved;

                for( size_t k : sorted )
                    moved.push_back( jobs[k] );

                for( size_t m = 0; m < list.size(); ++m )
                    jobs[list[m]] = moved[m];

                continue;
            }

            const VECTOR2I dir = portGoals[portEnd[list.front()]].dir;
            const VECTOR2I tangent( dir.y != 0 ? 1 : 0, dir.x != 0 ? 1 : 0 );
            auto along = [&]( const VECTOR2I& aAt ) { return (int64_t) aAt.x * tangent.x + (int64_t) aAt.y * tangent.y; };
            int64_t farSide = 0;

            for( size_t k : list )
            {
                const VECTOR2I farPos = portEnd[k] == jobs[k].from ? jobs[k].toPos : jobs[k].from->GetPosition();
                farSide += ( along( farPos ) - along( portEnd[k]->GetPosition() ) ) / (int64_t) list.size();
            }

            std::vector<size_t> sorted = list;
            std::stable_sort( sorted.begin(), sorted.end(), [&]( size_t x, size_t y )
                              {
                                  const int64_t ax = along( portEnd[x]->GetPosition() ), ay = along( portEnd[y]->GetPosition() );
                                  return farSide >= 0 ? ax > ay : ax < ay;
                              } );

            std::vector<JOB> moved;

            for( size_t k : sorted )
                moved.push_back( jobs[k] );

            for( size_t m = 0; m < list.size(); ++m )
                jobs[list[m]] = moved[m];
        }
    }

    // lanes: the rings laid before any route, each lane as planned (from its pad out to its offset, its
    // corners, out to its front) where the guard lets it, so no route takes the place of one still waiting;
    // the connection's route then leaves from its lane's end
    if( lanesOn && !dryRun && !portGoals.empty() )
    {
        std::vector<BOARD_CONNECTED_ITEM*>      staged;
        std::vector<std::unique_ptr<PCB_TRACK>> made;

        for( JOB& job : jobs )
        {
            for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( job.from ), job.to } )
            {
                PAD* pad = end && end->Type() == PCB_PAD_T ? static_cast<PAD*>( end ) : nullptr;
                auto it = pad ? portGoals.find( pad ) : portGoals.end();

                if( it == portGoals.end() || it->second.via || it->second.lane.size() < 4 || !it->second.ring.empty() )
                    continue;

                PORT_GOAL&                         g = it->second;
                std::vector<VECTOR2I>              pts = g.lane;
                std::vector<BOARD_CONNECTED_ITEM*> mine;
                std::vector<std::unique_ptr<PCB_TRACK>> ring;
                pts[0] = pad->GetPosition();

                for( size_t i = 1; i < pts.size(); ++i )
                {
                    auto track = std::make_unique<PCB_TRACK>( board );
                    track->SetStart( pts[i - 1] );
                    track->SetEnd( pts[i] );
                    track->SetWidth( g.width );
                    track->SetLayer( g.layer );
                    track->SetNet( pad->GetNet() );
                    mine.push_back( track.get() );
                    ring.push_back( std::move( track ) );
                }

                if( !guard( board, mine, {}, false, &staged ).empty() )
                {
                    lanesRefused++;
                    continue;
                }

                staged.insert( staged.end(), mine.begin(), mine.end() );

                for( std::unique_ptr<PCB_TRACK>& t : ring )
                {
                    g.ring.push_back( t->m_Uuid );
                    made.push_back( std::move( t ) );
                }
            }
        }

        if( !made.empty() )
        {
            BOARD_COMMIT commit( context->GetToolManager() );

            for( std::unique_ptr<PCB_TRACK>& t : made )
                commit.Add( t.release() );

            commit.Push( _( "Lanes (API)" ) );
            board->BuildConnectivity();
            lanesLaid = (int) staged.size();
        }
    }

    // legs between two laid lanes whose fronts face each other (their last runs snap to opposite directions
    // of the 45 degree grid): a straight run, one 45 degree jog across, a straight run; the jogs of a bus
    // staggered by (sqrt 2 - 1) pitch per track from the inside of the turn, so the legs lie side by side at
    // their pitch through the jog too
    if( lanesOn && lanesLaid > 0 )
    {
        struct LEG
        {
            PORT_GOAL* goal;
            PAD*       pad;
            VECTOR2I   farEnd;
            VECTOR2D   u, n;
            double     across, along, shift;
        };

        std::map<std::array<long long, 5>, std::vector<LEG>> groups;

        for( const JOB& job : jobs )
        {
            PAD* a = job.from;
            PAD* b = job.to && job.to->Type() == PCB_PAD_T ? static_cast<PAD*>( job.to ) : nullptr;

            if( !a || !b || !a->GetParentFootprint() || !b->GetParentFootprint() )
                continue;

            auto ia = portGoals.find( a );
            auto ib = portGoals.find( b );

            if( ia == portGoals.end() || ib == portGoals.end() )
                continue;

            if( setsOrder( b->GetParentFootprint(), a->GetParentFootprint() ) )
            {
                std::swap( a, b );
                std::swap( ia, ib );
            }

            PORT_GOAL& ga = ia->second;
            PORT_GOAL& gb = ib->second;

            if( ga.ring.empty() || gb.ring.empty() || ga.layer != gb.layer )
                continue;

            const VECTOR2D N( ga.lane.back() ), F( gb.lane.back() );
            VECTOR2D       u = N - VECTOR2D( ga.lane[ga.lane.size() - 2] );
            VECTOR2D       w = F - VECTOR2D( gb.lane[gb.lane.size() - 2] );

            if( u.EuclideanNorm() <= 0 || w.EuclideanNorm() <= 0 )
                continue;

            const DIRECTION_45 du( VECTOR2I( KiROUND( u.x ), KiROUND( u.y ) ) );
            const DIRECTION_45 dw( VECTOR2I( KiROUND( w.x ), KiROUND( w.y ) ) );

            if( !du.IsDefined() || dw != du.Opposite() )
                continue;

            const double octant = std::round( std::atan2( u.y, u.x ) / ( M_PI / 4 ) ) * ( M_PI / 4 );
            u = VECTOR2D( std::cos( octant ), std::sin( octant ) );

            const VECTOR2D n( -u.y, u.x );
            const double   along = ( F - N ).Dot( u );
            const double   shift = ( F - N ).Dot( n );

            if( along <= 0 || std::abs( shift ) < 1 )
                continue;

            const int sign = shift > 0 ? 1 : -1;
            groups[{ std::llround( u.x ), std::llround( u.y ), std::llround( N.Dot( u ) / 1000 ),
                     std::llround( F.Dot( u ) / 1000 ), sign }]
                    .push_back( { &ga, a, gb.lane.back(), u, n, N.Dot( n ) * sign, along, shift } );
        }

        // the group's jogs slid along the run together, to the first place where all its legs pass the guard
        const double slide = pcbIUScale.mmToIU( 0.25 );

        for( const auto& [key, legs] : groups )
        {
            double inner = legs.front().across;

            for( const LEG& l : legs )
                inner = std::max( inner, l.across );

            for( double extra = 0;; extra += slide )
            {
                std::vector<std::vector<VECTOR2I>>      corners;
                std::vector<std::unique_ptr<PCB_TRACK>> made;
                std::vector<BOARD_CONNECTED_ITEM*>      staged;
                bool                                    fits = true;

                for( const LEG& l : legs )
                {
                    const double   run = ( std::sqrt( 2.0 ) - 1 ) * ( inner - l.across ) + extra;
                    const VECTOR2D c1 = VECTOR2D( l.goal->lane.back() ) + l.u * run;
                    const VECTOR2D c2 = c1 + l.u * std::abs( l.shift ) + l.n * l.shift;
                    std::vector<VECTOR2I>& c = corners.emplace_back();

                    fits = fits && run + std::abs( l.shift ) <= l.along;

                    if( run >= 1 )
                        c.emplace_back( KiROUND( c1.x ), KiROUND( c1.y ) );

                    c.emplace_back( KiROUND( c2.x ), KiROUND( c2.y ) );

                    std::vector<VECTOR2I> pts = { l.goal->lane.back() };
                    pts.insert( pts.end(), c.begin(), c.end() );
                    pts.push_back( l.farEnd );

                    for( size_t i = 1; i < pts.size(); ++i )
                    {
                        if( pts[i] == pts[i - 1] )
                            continue;

                        auto track = std::make_unique<PCB_TRACK>( board );
                        track->SetStart( pts[i - 1] );
                        track->SetEnd( pts[i] );
                        track->SetWidth( l.goal->width );
                        track->SetLayer( l.goal->layer );
                        track->SetNet( l.pad->GetNet() );
                        staged.push_back( track.get() );
                        made.push_back( std::move( track ) );
                    }
                }

                if( !fits )
                    break;

                if( !guard( board, staged, {}, false ).empty() )
                    continue;

                for( size_t k = 0; k < legs.size(); ++k )
                    legs[k].goal->leg = corners[k];

                break;
            }
        }
    }

    // the corridor type (docs corridor.md): its gates as read, and the lanes its members take along it;
    // bus_paths and stage detail read corridors so
    struct CROSSING
    {
        VECTOR2I at;
        int      axis = -1;   ///< the portal along x (0) or y (1); -1 a free point
        int      lo = 0, hi = 0;
        std::optional<BOX2I> into;   ///< the region the path enters there
        std::vector<BOX2I>   rooms;  ///< the regions to the next gate (corridor.md); more than one: a straight run
        std::optional<PCB_LAYER_ID> layer;            ///< the run from it to the next gate on that layer
        std::optional<PCB_LAYER_ID> viaFrom, viaTo;   ///< a via gate: the layer changes there
        std::map<std::string, std::vector<std::pair<int, int>>> free;   ///< layer -> runs no copper is in
    };

    struct CORRIDOR
    {
        std::string              name;
        std::vector<std::string> nets;
        std::vector<CROSSING>    points;
        std::set<std::string>    aParts, bParts;   ///< floorplan: the parts of the blocks it joins
        std::string              a, b;
        nlohmann::json           from, to;         ///< one connection's ends: "REF.PAD" or {at_mm, layer}
    };

    // a connection along a corridor, put the corridor's way round (from its a end)
    struct MEMBER
    {
        size_t                job;
        bool                  forward;
        VECTOR2I              start, end;
        double                lateral = 0;
        BOARD_CONNECTED_ITEM* item = nullptr;   ///< its first end (the class's pitch, the layer it starts on)
        VECTOR2I              padStart, padEnd;   ///< its pads, when fanned out from them
        bool                  fanStart = false, fanEnd = false;
    };

    const int bigIU = 1 << 30;

    auto crossingOf = [&]( const nlohmann::json& aPoint, CROSSING& aOut )
    {
        const nlohmann::json at = aPoint.is_array() ? aPoint : aPoint.is_object() ? aPoint.value( "at_mm", nlohmann::json() )
                                                                                  : nlohmann::json();

        if( !at.is_array() || at.size() != 2 || !at[0].is_number() || !at[1].is_number() )
            return false;

        aOut.at = VECTOR2I( pcbIUScale.mmToIU( at[0].get<double>() ), pcbIUScale.mmToIU( at[1].get<double>() ) );
        aOut.lo = -bigIU;
        aOut.hi = bigIU;

        if( aPoint.is_object() && aPoint.contains( "layer" ) )
        {
            aOut.layer = aPoint["layer"].is_string() ? parseLayer( board, aPoint["layer"].get<std::string>() ) : std::nullopt;

            if( !aOut.layer )
                return false;
        }

        if( aPoint.is_object() && aPoint.contains( "via" ) )
        {
            const nlohmann::json v = aPoint["via"];

            if( !v.is_object() || aPoint.contains( "along" ) || !v.value( "from_layer", nlohmann::json() ).is_string()
                || !v.value( "to_layer", nlohmann::json() ).is_string() )
            {
                return false;
            }

            aOut.viaFrom = parseLayer( board, v["from_layer"].get<std::string>() );
            aOut.viaTo = parseLayer( board, v["to_layer"].get<std::string>() );

            if( !aOut.viaFrom || !aOut.viaTo || *aOut.viaFrom == *aOut.viaTo )
                return false;

            aOut.layer = aOut.viaTo;
        }

        auto boxOf = []( const nlohmann::json& aBox ) -> std::optional<BOX2I>
        {
            if( !aBox.is_array() || aBox.size() != 4
                || !std::all_of( aBox.begin(), aBox.end(), []( const nlohmann::json& v ) { return v.is_number(); } ) )
            {
                return std::nullopt;
            }

            const VECTOR2I p0( pcbIUScale.mmToIU( aBox[0].get<double>() ), pcbIUScale.mmToIU( aBox[1].get<double>() ) );
            const VECTOR2I p1( pcbIUScale.mmToIU( aBox[2].get<double>() ), pcbIUScale.mmToIU( aBox[3].get<double>() ) );
            return BOX2I( p0, p1 - p0 );
        };

        // the rooms to the next gate: any gate's (a portal's, a via gate's, a free point's)
        const nlohmann::json roomsArg = aPoint.is_object() ? aPoint.value( "rooms_mm", nlohmann::json() ) : nlohmann::json();

        if( roomsArg.is_array() )
        {
            for( const nlohmann::json& box : roomsArg )
            {
                std::optional<BOX2I> room = boxOf( box );

                if( !room )
                    return false;

                aOut.rooms.push_back( *room );
            }
        }

        if( aPoint.is_object() && aPoint.contains( "along" ) )
        {
            const std::string along = aPoint["along"].is_string() ? aPoint["along"].get<std::string>() : std::string();

            if( along != "x" && along != "y" )
                return false;

            aOut.axis = along == "x" ? 0 : 1;
            aOut.lo = aPoint.contains( "lo_mm" ) ? pcbIUScale.mmToIU( aPoint.value( "lo_mm", 0.0 ) ) : -bigIU;
            aOut.hi = aPoint.contains( "hi_mm" ) ? pcbIUScale.mmToIU( aPoint.value( "hi_mm", 0.0 ) ) : bigIU;
            const nlohmann::json r = aPoint.value( "into_mm", nlohmann::json() );

            if( std::optional<BOX2I> room = boxOf( r ) )
                aOut.into = room;
            else if( !aOut.rooms.empty() )
                aOut.into = aOut.rooms.front();

            if( aOut.rooms.empty() && aOut.into )
                aOut.rooms.push_back( *aOut.into );

            const nlohmann::json freeArg = aPoint.value( "free_mm", nlohmann::json() );

            if( freeArg.is_object() )
            {
                for( auto it = freeArg.begin(); it != freeArg.end(); ++it )
                {
                    if( !it.value().is_array() )
                        return false;

                    std::vector<std::pair<int, int>>& runs = aOut.free[it.key()];

                    for( const nlohmann::json& run : it.value() )
                    {
                        if( !run.is_array() || run.size() != 2 || !run[0].is_number() || !run[1].is_number() )
                            return false;

                        runs.emplace_back( pcbIUScale.mmToIU( run[0].get<double>() ), pcbIUScale.mmToIU( run[1].get<double>() ) );
                    }
                }
            }
        }

        return true;
    };

    auto pitchOfNet = [&]( BOARD_CONNECTED_ITEM* aPad )
    {
        NETCLASS* netclass = aPad->GetEffectiveNetClass();
        const int width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : ds.GetCurrentTrackWidth();
        const int clearance = std::max( { ds.m_MinClearance, netclass && netclass->HasClearance() ? netclass->GetClearance() : 0, 1 } );
        return width + clearance;
    };

    // a corridor's members laid along it (bus_paths, stage detail): the centre line through its gates,
    // each member's corners beside it at the pitch, a via gate's layers at its corners; the members
    // sorted across the way. aNear: only the copper there is looked at (none: the whole board)
    struct LANES
    {
        std::vector<std::vector<VECTOR2I>>                                 corners;   ///< per member, along the corridor
        std::vector<std::optional<std::pair<PCB_LAYER_ID, PCB_LAYER_ID>>> vias;      ///< per corner: a via gate there (from, to)
        bool                                                               insideLast = false;
        nlohmann::json                                                     out = nlohmann::json::object();
    };

    auto laneAlong = [&]( const CORRIDOR& c, const std::set<int>& codes, std::vector<MEMBER>& members,
                          const std::optional<BOX2I>& aNear ) -> LANES
    {
        int pitch = 0;
        VECTOR2I startMid( 0, 0 ), endMid( 0, 0 );

        for( const MEMBER& m : members )
        {
            pitch = std::max( pitch, pitchOfNet( m.item ) );
            startMid += m.start / (int) members.size();
            endMid += m.end / (int) members.size();
        }

        const int n = (int) members.size();
        const int half = KiROUND( ( n - 1 ) / 2.0 * pitch );
        const int margin = pitch;
        int       squeezed = 0, copperNarrowed = 0;
        std::vector<CROSSING> pts = c.points;

        // an end's own boundary at or behind its members' fan-out tips: those crossings stay where the
        // floorplan put them (the tips pass them; the copper there is the end's own pads)
        std::vector<char> behindTips( pts.size(), 0 );

        for( bool atStart : { true, false } )
        {
            VECTOR2D padMid( 0, 0 ), tipMid( 0, 0 );
            int      fanned = 0;

            for( const MEMBER& m : members )
            {
                if( !( atStart ? m.fanStart : m.fanEnd ) )
                    continue;

                padMid += VECTOR2D( atStart ? m.padStart : m.padEnd );
                tipMid += VECTOR2D( atStart ? m.start : m.end );
                fanned++;
            }

            if( !fanned )
                continue;

            padMid = padMid / fanned;
            tipMid = tipMid / fanned;
            const VECTOR2D way = tipMid - padMid;

            if( way.EuclideanNorm() <= 0 )
                continue;

            // a portal across the way out (the end's side its tips leave by), not one along it
            auto behind = [&]( const CROSSING& aX )
            {
                const bool across = aX.axis == 0 ? std::abs( way.y ) > std::abs( way.x )
                                    : aX.axis == 1 && std::abs( way.x ) > std::abs( way.y );
                return across && ( VECTOR2D( aX.at ) - tipMid ).Dot( way ) <= 0;
            };

            for( size_t k = 0; k < pts.size(); ++k )
            {
                const size_t at = atStart ? k : pts.size() - 1 - k;

                if( !behind( pts[at] ) )
                    break;

                behindTips[at] = 1;
            }
        }

        BOARD_CONNECTED_ITEM* first = members.front().item;
        const PCB_LAYER_ID    startLayer = first->IsOnLayer( F_Cu ) ? F_Cu : first->IsOnLayer( B_Cu ) ? B_Cu
                                                                                               : first->GetLayerSet().CuStack().front();
        const std::string     startLayerName = str( board->GetLayerName( startLayer ) );

        // copper on the members' layer: each crossing in the run of its stretch no copper is in that
        // is wide enough and nearest to where it was (the widest when none is)
        for( size_t k = 0; k < pts.size(); ++k )
        {
            CROSSING& x = pts[k];
            auto      runs = x.free.find( x.layer ? str( board->GetLayerName( *x.layer ) ) : startLayerName );

            if( x.axis < 0 || runs == x.free.end() || behindTips[k] )
                continue;

            const int u = x.axis == 0 ? x.at.x : x.at.y;
            const std::pair<int, int>* pick = nullptr;
            int64_t pickCost = std::numeric_limits<int64_t>::max();

            for( const std::pair<int, int>& run : runs->second )
            {
                const int lo = run.first + margin + half, hi = run.second - margin - half;
                const int64_t cost = lo <= hi ? int64_t( u < lo ? lo - u : u > hi ? u - hi : 0 )
                                              : int64_t( bigIU ) * 4 - ( run.second - run.first );

                if( cost < pickCost )
                {
                    pickCost = cost;
                    pick = &run;
                }
            }

            if( pick && ( pick->first != x.lo || pick->second != x.hi ) )
            {
                x.lo = pick->first;
                x.hi = pick->second;
                copperNarrowed++;
            }
        }

        // the ribbon inside each portal's free stretch, a pitch off its ends, straight on from the
        // portal before where that is in line
        for( size_t k = 0; k < pts.size(); ++k )
        {
            CROSSING& x = pts[k];

            if( x.axis < 0 )
                continue;

            int&      u = x.axis == 0 ? x.at.x : x.at.y;
            const int lo = x.lo + margin + half, hi = x.hi - margin - half;

            if( k > 0 && pts[k - 1].axis == x.axis && pts[k - 1].rooms.size() <= 1 )
                u = x.axis == 0 ? pts[k - 1].at.x : pts[k - 1].at.y;

            if( lo <= hi )
            {
                u = std::clamp( u, lo, hi );
            }
            else
            {
                u = int( ( int64_t( x.lo ) + x.hi ) / 2 );
                squeezed++;
            }
        }

        // the way across a crossing, into the region it enters
        auto travelAt = [&]( size_t aIndex ) -> VECTOR2D
        {
            const CROSSING& x = pts[aIndex];

            if( x.axis >= 0 && x.into )
            {
                return x.axis == 1 ? VECTOR2D( x.into->Centre().x > x.at.x ? 1.0 : -1.0, 0.0 )
                                   : VECTOR2D( 0.0, x.into->Centre().y > x.at.y ? 1.0 : -1.0 );
            }

            const VECTOR2D way( ( aIndex + 1 < pts.size() ? pts[aIndex + 1].at : endMid )
                                - ( aIndex > 0 ? pts[aIndex - 1].at : startMid ) );
            return way.EuclideanNorm() > 0 ? way.Resize( 1.0 ) : VECTOR2D( 1.0, 0.0 );
        };

        // the centre line: the crossings, joined inside each region by axis-parallel runs (a turn's
        // corner where the two crossings' lines meet; a jog half way; a way back out through the same
        // side as deep as the ribbon needs), the ribbon a pitch off the region's sides where it fits
        std::vector<VECTOR2I>     line;
        std::vector<PCB_LAYER_ID> lineLayer;   ///< per vertex: the layer of the run from it on
        int                       runsMoved = 0;

        // per gate the layer of the run from it to the next (its own, else the one before's)
        std::vector<PCB_LAYER_ID> gateLayer( pts.size(), startLayer );

        for( size_t k = 0; k < pts.size(); ++k )
            gateLayer[k] = pts[k].layer ? *pts[k].layer : k > 0 ? gateLayer[k - 1] : startLayer;

        PCB_LAYER_ID onLayer = startLayer;   ///< the layer the run looked at is on

        // copper of other nets per layer (read when first asked), for the runs inside the regions
        // (and the pulls); and the rule areas tracks may not cross there
        std::map<PCB_LAYER_ID, std::vector<std::pair<BOX2I, std::shared_ptr<SHAPE>>>> copperOf;

        auto copperOn = [&]( PCB_LAYER_ID aLayer ) -> const std::vector<std::pair<BOX2I, std::shared_ptr<SHAPE>>>&
        {
            auto it = copperOf.find( aLayer );

            if( it != copperOf.end() )
                return it->second;

            std::vector<std::pair<BOX2I, std::shared_ptr<SHAPE>>>& copper = copperOf[aLayer];

            auto addCopper = [&]( BOARD_CONNECTED_ITEM* aItem )
            {
                if( aNear && !aNear->Intersects( aItem->GetBoundingBox() ) )
                    return;

                if( aItem->IsOnLayer( aLayer ) && !codes.count( aItem->GetNetCode() ) )
                    copper.emplace_back( aItem->GetBoundingBox(), aItem->GetEffectiveShape( aLayer ) );
            };

            for( PCB_TRACK* track : board->Tracks() )
                addCopper( track );

            for( FOOTPRINT* fp : board->Footprints() )
            {
                for( PAD* pad : fp->Pads() )
                    addCopper( pad );
            }

            for( ZONE* zone : board->Zones() )
            {
                if( zone->GetIsRuleArea() && zone->GetDoNotAllowTracks() && zone->IsOnLayer( aLayer )
                    && ( !aNear || aNear->Intersects( zone->Outline()->BBox() ) ) )
                    copper.emplace_back( zone->Outline()->BBox(), std::make_shared<SHAPE_POLY_SET>( *zone->Outline() ) );
            }

            return copper;
        };

        auto ribbonClear = [&]( const VECTOR2I& aFrom, const VECTOR2I& aTo )
        {
            const int clearance = half + margin;
            BOX2I     span = BOX2I::ByCorners( aFrom, aTo );
            span.Inflate( clearance );

            for( const auto& [box, shape] : copperOn( onLayer ) )
            {
                if( box.Intersects( span ) && shape->Collide( SEG( aFrom, aTo ), clearance ) )
                    return false;
            }

            return true;
        };

        // a via gate's point: fixed, the line keeps it as a corner (never pulled past, never dropped)
        auto viaGateAt = [&]( const VECTOR2I& aAt ) -> const CROSSING*
        {
            for( const CROSSING& x : pts )
            {
                if( x.viaTo && x.at == aAt )
                    return &x;
            }

            return nullptr;
        };

        auto keep = [&]( const VECTOR2I& aAt )
        {
            if( line.empty() || line.back() != aAt )
            {
                line.push_back( aAt );
                lineLayer.push_back( onLayer );
            }
        };

        // a crossing moved along its stretch (inside it at the ribbon's room), the shifts to try: none
        // first, then outward both ways a pitch at a time (24 pitches at most)
        auto movedAlong = [&]( const CROSSING& aC, int aBy )
        {
            return aC.axis == 0 ? VECTOR2I( aC.at.x + aBy, aC.at.y ) : VECTOR2I( aC.at.x, aC.at.y + aBy );
        };

        auto shiftsOf = [&]( const CROSSING& aC )
        {
            const int        u = aC.axis == 0 ? aC.at.x : aC.at.y;
            const int        lo = aC.lo + margin + half - u, hi = aC.hi - margin - half - u;
            const int        step = std::max( 1, pitch );
            std::vector<int> out = { 0 };

            for( int d = step; d <= std::max( hi, -lo ) && out.size() < 49; d += step )
            {
                if( d <= hi )
                    out.push_back( d );

                if( -d >= lo )
                    out.push_back( -d );
            }

            return out;
        };

        int crossingsMoved = 0;

        for( size_t k = 0; k < pts.size(); ++k )
        {
            CROSSING& x = pts[k];
            onLayer = gateLayer[k];
            keep( x.at );

            if( k + 1 == pts.size() || x.axis < 0 || pts[k + 1].axis < 0 || !x.into || x.rooms.size() > 1 )
                continue;

            CROSSING& y = pts[k + 1];

            // a jog of less than two pitches between two crossings in line: the second moved in line
            // with the first where its stretch has room
            if( x.axis == y.axis )
            {
                int&      v = x.axis == 0 ? y.at.x : y.at.y;
                const int u = x.axis == 0 ? x.at.x : x.at.y;

                if( v != u && std::abs( v - u ) < 2 * pitch && u >= y.lo + margin + half && u <= y.hi - margin - half )
                    v = u;
            }

            if( x.axis != y.axis )
            {
                auto cornerOf = [&]( const VECTOR2I& aX, const VECTOR2I& aY )
                {
                    return x.axis == 1 ? VECTOR2I( aY.x, aX.y ) : VECTOR2I( aX.x, aY.y );
                };

                auto legsClear = [&]( const VECTOR2I& aX, const VECTOR2I& aY )
                {
                    const VECTOR2I c = cornerOf( aX, aY );
                    return ribbonClear( aX, c ) && ribbonClear( c, aY );
                };

                // the turn and its two legs clear of the copper: both crossings moved along their
                // stretches, the least moved pair that clears
                if( !copperOn( onLayer ).empty() && !legsClear( x.at, y.at ) )
                {
                    const std::vector<int> dx = shiftsOf( x ), dy = shiftsOf( y );
                    std::optional<std::pair<VECTOR2I, VECTOR2I>> best;
                    int64_t                                      bestCost = std::numeric_limits<int64_t>::max();

                    for( int u : dx )
                    {
                        for( int v : dy )
                        {
                            const int64_t cost = int64_t( std::abs( u ) ) + std::abs( v );

                            if( cost >= bestCost )
                                continue;

                            const VECTOR2I px = movedAlong( x, u ), py = movedAlong( y, v );

                            if( legsClear( px, py ) )
                            {
                                best = std::make_pair( px, py );
                                bestCost = cost;
                            }
                        }
                    }

                    if( best )
                    {
                        if( line.back() == x.at )
                            line.back() = best->first;

                        x.at = best->first;
                        y.at = best->second;
                        crossingsMoved++;
                    }
                }

                keep( cornerOf( x.at, y.at ) );
                continue;
            }

            const bool inX = x.axis == 1;
            const int  a0 = inX ? x.at.x : x.at.y, a1 = inX ? y.at.x : y.at.y;
            const int  b0 = inX ? x.at.y : x.at.x;
            int        b1 = inX ? y.at.y : y.at.x;

            // in line: straight on where that is clear of the copper, else a jog round it
            if( b0 == b1 && ( copperOn( onLayer ).empty() || ribbonClear( x.at, y.at ) ) )
                continue;

            // through the region with the crossings less than a pitch apart along the way: one straight
            // (or diagonal) run, no jog past the second and back; out through the side it came in by:
            // as deep as the ribbon needs
            const VECTOR2D way = travelAt( k );
            const bool     back = way.Dot( travelAt( k + 1 ) ) < 0;

            if( !back && std::abs( a1 - a0 ) < pitch )
                continue;

            int            mid = back && std::abs( a1 - a0 ) < pitch ? a0 + KiROUND( ( inX ? way.x : way.y ) * ( margin + half ) )
                                                                     : int( ( int64_t( a0 ) + a1 ) / 2 );
            const int      rlo = ( inX ? x.into->GetLeft() : x.into->GetTop() ) + margin + half;
            const int      rhi = ( inX ? x.into->GetRight() : x.into->GetBottom() ) - margin - half;

            if( rlo <= rhi )
                mid = std::clamp( mid, rlo, rhi );

            // the run (and its two legs from the crossings) clear of the copper there: the nearest
            // depth in the region where it is
            auto runClear = [&]( int aMid )
            {
                const VECTOR2I p0 = inX ? VECTOR2I( a0, b0 ) : VECTOR2I( b0, a0 );
                const VECTOR2I p1 = inX ? VECTOR2I( aMid, b0 ) : VECTOR2I( b0, aMid );
                const VECTOR2I p2 = inX ? VECTOR2I( aMid, b1 ) : VECTOR2I( b1, aMid );
                const VECTOR2I p3 = inX ? VECTOR2I( a1, b1 ) : VECTOR2I( b1, a1 );
                return ribbonClear( p0, p1 ) && ribbonClear( p1, p2 ) && ribbonClear( p2, p3 );
            };

            // the nearest clear depth, the second crossing where it is or else moved along its
            // stretch (the least moved that clears)
            if( rlo <= rhi && !copperOn( onLayer ).empty() && !runClear( mid ) )
            {
                const int step = std::max( 1, pitch / 4 );
                const int b1From = b1;

                for( int v : shiftsOf( y ) )
                {
                    b1 = b1From + v;
                    std::optional<int> clearAt;

                    if( v != 0 && runClear( mid ) )
                        clearAt = mid;

                    for( int d = step; !clearAt && d <= rhi - rlo; d += step )
                    {
                        if( mid + d <= rhi && runClear( mid + d ) )
                            clearAt = mid + d;
                        else if( mid - d >= rlo && runClear( mid - d ) )
                            clearAt = mid - d;
                    }

                    if( clearAt )
                    {
                        runsMoved += *clearAt != mid;
                        mid = *clearAt;

                        if( v != 0 )
                        {
                            y.at = movedAlong( y, v );
                            crossingsMoved++;
                        }

                        break;
                    }
                }

                if( b1 != b1From && ( inX ? y.at.y : y.at.x ) != b1 )
                    b1 = b1From;
            }

            keep( inX ? VECTOR2I( mid, b0 ) : VECTOR2I( b0, mid ) );
            keep( inX ? VECTOR2I( mid, b1 ) : VECTOR2I( b1, mid ) );
        }

        auto unit = []( const VECTOR2I& aD )
        {
            const VECTOR2D v( aD );
            return v.EuclideanNorm() > 0 ? v.Resize( 1.0 ) : VECTOR2D( 0.0, 0.0 );
        };

        // the line pulled tight: from each vertex on to the farthest one a straight run (axis-parallel
        // or else diagonal) or one turn reaches inside the regions crossed and clear of the copper, so
        // the corners are the turns and the way never goes back
        const size_t lineFrom = line.size();

        auto inRegions = [&]( const VECTOR2I& aFrom, const VECTOR2I& aTo )
        {
            const double length = ( aTo - aFrom ).EuclideanNorm();
            const int    steps = std::max( 1, (int) std::ceil( length / std::max( pitch, 1 ) ) );
            const VECTOR2D across = length > 0 ? VECTOR2D( -( aTo.y - aFrom.y ) / length, ( aTo.x - aFrom.x ) / length )
                                               : VECTOR2D( 0.0, 0.0 );

            for( int t = 0; t <= steps; ++t )
            {
                const double   f = double( t ) / steps;
                const VECTOR2D at( aFrom.x + ( aTo.x - aFrom.x ) * f, aFrom.y + ( aTo.y - aFrom.y ) * f );

                // the ribbon's middle and both its sides, a pitch off them
                for( double side : { 0.0, 1.0, -1.0 } )
                {
                    const VECTOR2D p = at + across * ( side * ( half + margin ) );
                    const VECTOR2I q( KiROUND( p.x ), KiROUND( p.y ) );
                    bool           inside = false;

                    for( size_t g = 0; g < pts.size() && !inside; ++g )
                    {
                        const CROSSING& x = pts[g];

                        if( gateLayer[g] != onLayer )
                            continue;

                        for( const BOX2I& room : x.rooms )
                            inside = inside || BOX2I( room ).Inflate( 10 ).Contains( q );

                        if( inside )
                            break;
                    }

                    if( !inside )
                        return false;
                }
            }

            return true;
        };

        auto clear = [&]( const VECTOR2I& aFrom, const VECTOR2I& aTo )
        {
            return inRegions( aFrom, aTo ) && ribbonClear( aFrom, aTo );
        };

        // the first and the last run kept: the ways out of the end blocks and into them
        if( line.size() > 4 )
        {
            std::vector<VECTOR2I> tight = { line[0], line[1] };
            const size_t          last = line.size() - 2;

            for( size_t i = 1; i < last; )
            {
                size_t                  next = i + 1;
                std::optional<VECTOR2I> turn;

                for( size_t j = last; j > i + 1; --j )
                {
                    bool overVia = false;

                    for( size_t v = i + 1; v < j && !overVia; ++v )
                        overVia = viaGateAt( line[v] ) != nullptr || lineLayer[v] != lineLayer[i];

                    if( overVia )
                        continue;

                    onLayer = lineLayer[i];

                    const VECTOR2I a = line[i], b = line[j];
                    const VECTOR2I c0( b.x, a.y ), c1( a.x, b.y );

                    if( ( a.x == b.x || a.y == b.y ) && clear( a, b ) )
                    {
                        next = j;
                        break;
                    }

                    if( clear( a, c0 ) && clear( c0, b ) )
                    {
                        next = j;
                        turn = c0;
                        break;
                    }

                    if( clear( a, c1 ) && clear( c1, b ) )
                    {
                        next = j;
                        turn = c1;
                        break;
                    }

                    if( clear( a, b ) )
                    {
                        next = j;
                        break;
                    }
                }

                if( turn )
                    tight.push_back( *turn );

                tight.push_back( line[next] );
                i = next;
            }

            tight.push_back( line.back() );
            line = std::move( tight );
        }

        // corners only where the line turns: a vertex in line with its neighbours, or one the line
        // comes back from (a way back along the same line), is dropped
        for( bool dropped = true; dropped && line.size() > 2; )
        {
            dropped = false;

            for( size_t v = 1; v + 1 < line.size(); ++v )
            {
                const VECTOR2D a = unit( line[v] - line[v - 1] ), b = unit( line[v + 1] - line[v] );

                if( viaGateAt( line[v] ) )
                    continue;

                if( a.EuclideanNorm() == 0 || b.EuclideanNorm() == 0 || std::abs( a.x * b.y - a.y * b.x ) < 1e-6 )
                {
                    line.erase( line.begin() + (long) v );
                    dropped = true;
                    break;
                }
            }
        }

        const VECTOR2D firstWay = travelAt( 0 );
        const VECTOR2D lastWay = pts.back().axis >= 0 && pts.back().into ? travelAt( pts.size() - 1 ) : unit( endMid - line.back() );
        const VECTOR2D left0( firstWay.y, -firstWay.x );

        // per vertex the way a ribbon line one unit to the left is moved (mitred at the turns)
        std::vector<VECTOR2D> shift( line.size() );

        for( size_t k = 0; k < line.size(); ++k )
        {
            VECTOR2D a = k == 0 ? firstWay : unit( line[k] - line[k - 1] );
            VECTOR2D b = k + 1 == line.size() ? lastWay : unit( line[k + 1] - line[k] );

            if( a.EuclideanNorm() == 0 )
                a = b;

            if( b.EuclideanNorm() == 0 )
                b = a;

            const VECTOR2D na( a.y, -a.x ), nb( b.y, -b.x );
            const double   denom = 1.0 + a.Dot( b );
            shift[k] = denom > 0.2 ? ( na + nb ) * ( 1.0 / denom ) : na;
        }

        for( MEMBER& m : members )
            m.lateral = VECTOR2D( m.start - line.front() ).Dot( left0 );

        std::stable_sort( members.begin(), members.end(), []( const MEMBER& x, const MEMBER& y ) { return x.lateral > y.lateral; } );

        // the corners: the line's vertices but its ends on the end blocks' sides (the members leave
        // their pads for the first turn, so their pitch closes up over the way there)
        // the corners: the line's vertices but those near the end pads, where the members still close
        // up from their pads' pitch to the ribbon's (as far as that takes at 45 degrees, plus two pitches)
        std::vector<std::vector<VECTOR2I>> corners( n );

        auto closeUp = [&]( bool aStart )
        {
            const VECTOR2D left = aStart ? left0 : VECTOR2D( lastWay.y, -lastWay.x );
            double         lo = std::numeric_limits<double>::max(), hi = -lo;

            for( const MEMBER& m : members )
            {
                const double u = VECTOR2D( aStart ? m.start : m.end ).Dot( left );
                lo = std::min( lo, u );
                hi = std::max( hi, u );
            }

            return std::max( 0.0, ( hi - lo - 2.0 * half ) / 2.0 ) + 2.0 * pitch + half;
        };

        const double reachStart = closeUp( true ), reachEnd = closeUp( false );

        auto nearEnd = [&]( const VECTOR2I& aAt )
        {
            for( const MEMBER& m : members )
            {
                if( ( m.start - aAt ).EuclideanNorm() < reachStart || ( m.end - aAt ).EuclideanNorm() < reachEnd )
                    return true;
            }

            return false;
        };

        std::vector<size_t> kept;

        for( size_t v = 0; v < line.size(); ++v )
        {
            if( !nearEnd( line[v] ) || viaGateAt( line[v] ) )
                kept.push_back( v );
        }

        // at a via gate the members' vias lie across the way at the via pitch (a via and the board's
        // clearance), not the tracks'
        const int viaPitch = std::max( pitch, ds.GetCurrentViaSize() + ds.m_MinClearance );
        nlohmann::json viasOut = nlohmann::json::array();

        for( size_t v : kept )
        {
            if( const CROSSING* g = viaGateAt( line[v] ) )
            {
                viasOut.push_back( { { "at_mm", { toMm( line[v].x ), toMm( line[v].y ) } },
                                     { "from_layer", str( board->GetLayerName( *g->viaFrom ) ) },
                                     { "to_layer", str( board->GetLayerName( *g->viaTo ) ) } } );
            }
        }

        for( int k = 0; k < n; ++k )
        {
            for( size_t v : kept )
            {
                const bool   via = viaGateAt( line[v] ) != nullptr;
                const double offset = ( n - 1 ) / 2.0 * ( via ? viaPitch : pitch ) - double( k ) * ( via ? viaPitch : pitch );
                corners[k].push_back( line[v] + VECTOR2I( KiROUND( shift[v].x * offset ), KiROUND( shift[v].y * offset ) ) );
            }
        }

        // routed from the inside of the first turn out (y down: a clockwise turn has its inside on
        // the right, the last of the order)
        bool insideLast = false;

        for( size_t v = 0; v < line.size(); ++v )
        {
            const VECTOR2D a = v == 0 ? firstWay : unit( line[v] - line[v - 1] );
            const VECTOR2D b = v + 1 == line.size() ? lastWay : unit( line[v + 1] - line[v] );
            const double   turn = a.x * b.y - a.y * b.x;

            if( std::abs( turn ) > 0.1 )
            {
                insideLast = turn > 0;
                break;
            }
        }


        LANES lanes;
        lanes.corners = std::move( corners );
        lanes.insideLast = insideLast;

        for( size_t v : kept )
        {
            const CROSSING* g = viaGateAt( line[v] );
            lanes.vias.push_back( g ? std::make_optional( std::make_pair( *g->viaFrom, *g->viaTo ) ) : std::nullopt );
        }

        nlohmann::json& out = lanes.out;
        out["corners"] = kept.size();

        if( !viasOut.empty() )
            out["vias"] = viasOut;

        out["pitch_mm"] = toMm( pitch );
        out["line_mm"] = nlohmann::json::array();

        for( const VECTOR2I& v : line )
            out["line_mm"].push_back( { toMm( v.x ), toMm( v.y ) } );

        if( squeezed )
            out["portals_narrower"] = squeezed;

        if( copperNarrowed )
            out["copper_narrowed"] = copperNarrowed;

        if( const int kept = (int) std::count( behindTips.begin(), behindTips.end(), 1 ) )
            out["behind_tips"] = kept;

        if( runsMoved )
            out["runs_moved"] = runsMoved;

        if( crossingsMoved )
            out["crossings_moved"] = crossingsMoved;

        if( line.size() < lineFrom )
            out["pulled_from"] = lineFrom;

        return lanes;
    };

    // bus_paths: a bus along a corridor (pcb_floorplan's path for the bus between two blocks, or points
    // given): its members side by side at their pitch across every portal of the path, in the order
    // their pads lie across the way in at the start, as fixed corners; a member routed so is left as it
    // is by clean_up, one the corridor does not route is routed as any other connection (row corridor)
    nlohmann::json corridorOut = nlohmann::json::array();
    int            corridorRoutedCount = 0, corridorMembers = 0;
    {
        std::vector<CORRIDOR> corridors;
        const nlohmann::json  pathsArg = aArgs.contains( "bus_paths" ) ? aArgs["bus_paths"] : nlohmann::json( "off" );

        if( ( pathsArg.is_string() && pathsArg.get<std::string>() == "floorplan" )
            || ( pathsArg.is_object() && pathsArg.contains( "floorplan" ) ) )
        {
            std::optional<KOPENAPI_METHOD> fp = KOPENAPI_REGISTRY::Get().Find( "pcb_floorplan" );

            if( !fp )
                return KOPENAPI_RESULT::Error( 500, "pcb_floorplan is not registered" );

            nlohmann::json fpArgs = pathsArg.is_object() ? pathsArg["floorplan"] : nlohmann::json::object();

            if( !fpArgs.is_object() )
                return KOPENAPI_RESULT::Error( 400, "bus_paths: {floorplan: {pcb_floorplan arguments}}" );

            fpArgs["list_limit"] = 100000;

            if( !fpArgs.contains( "copper_stretches" ) )
                fpArgs["copper_stretches"] = true;

            const KOPENAPI_RESULT r = fp->handler( aCtx, fpArgs );

            if( r.status >= 400 )
                return KOPENAPI_RESULT::Error( r.status, "bus_paths: pcb_floorplan: " + r.body.value( "error", nlohmann::json() ).dump() );

            std::map<std::string, std::set<std::string>> partsOf;

            for( const nlohmann::json& blk : r.body.value( "blocks", nlohmann::json::array() ) )
            {
                for( const nlohmann::json& ref : blk.value( "parts", nlohmann::json::array() ) )
                {
                    if( ref.is_string() )
                        partsOf[blk.value( "name", std::string() )].insert( ref.get<std::string>() );
                }
            }

            for( const nlohmann::json& l : r.body.value( "links", nlohmann::json::array() ) )
            {
                if( !l.contains( "bus" ) || !l["bus"].is_string() || !l.value( "found", false ) )
                    continue;

                CORRIDOR c;
                c.name = l["bus"].get<std::string>();
                c.a = l.value( "a", std::string() );
                c.b = l.value( "b", std::string() );
                c.aParts = partsOf[c.a];
                c.bParts = partsOf[c.b];

                for( const nlohmann::json& n : l.value( "net_names", nlohmann::json::array() ) )
                {
                    if( n.is_string() )
                        c.nets.push_back( n.get<std::string>() );
                }

                for( const nlohmann::json& pt : l.value( "corridor", nlohmann::json::array() ) )
                {
                    CROSSING x;

                    if( crossingOf( pt, x ) )
                        c.points.push_back( x );
                }

                if( !c.points.empty() && !c.nets.empty() )
                    corridors.push_back( std::move( c ) );
            }
        }
        else if( pathsArg.is_array() )
        {
            for( const nlohmann::json& b : pathsArg )
            {
                CORRIDOR c;
                c.name = b.is_object() ? b.value( "name", std::string( "bus" ) + std::to_string( corridors.size() + 1 ) ) : std::string();

                const nlohmann::json netsArg = b.is_object() && b.contains( "net" ) && b["net"].is_string()
                                                       ? nlohmann::json::array( { b["net"] } )
                                                       : b.is_object() ? b.value( "nets", nlohmann::json() ) : nlohmann::json();

                if( !b.is_object() || !netsArg.is_array() || !b.contains( "points_mm" ) || !b["points_mm"].is_array() )
                    return KOPENAPI_RESULT::Error( 400, "bus_paths: [{name, nets: [net names] (or net), from, to, points_mm: [[x, y] or {at_mm, along, lo_mm, hi_mm}]}]" );

                for( const char* key : { "from", "to" } )
                {
                    if( !b.contains( key ) )
                        continue;

                    const nlohmann::json& e = b[key];
                    const bool padEnd = e.is_string() && e.get<std::string>().find( '.' ) != std::string::npos;
                    const bool atEnd = e.is_object() && e.value( "at_mm", nlohmann::json() ).is_array() && e["at_mm"].size() == 2;

                    if( !padEnd && !atEnd )
                        return KOPENAPI_RESULT::Error( 400, std::string( "bus_paths: " ) + key + " is \"REF.PAD\" or {at_mm: [x, y], layer}" );

                    ( std::string( key ) == "from" ? c.from : c.to ) = e;
                }

                if( c.from.is_null() != c.to.is_null() )
                    return KOPENAPI_RESULT::Error( 400, "bus_paths: from and to go together" );

                for( const nlohmann::json& n : netsArg )
                {
                    if( !n.is_string() )
                        return KOPENAPI_RESULT::Error( 400, "bus_paths: nets are net names" );

                    if( !KopenapiFindNet( board, n.get<std::string>() ) )
                        return KopenapiNoSuchNet( board, n.get<std::string>(), "bus_paths: " );

                    c.nets.push_back( n.get<std::string>() );
                }

                for( const nlohmann::json& pt : b["points_mm"] )
                {
                    CROSSING x;

                    if( !crossingOf( pt, x ) )
                        return KOPENAPI_RESULT::Error( 400, "bus_paths: a point is [x_mm, y_mm], {at_mm: [x, y], along: \"x\" | \"y\", lo_mm, hi_mm, "
                                                            "rooms_mm, layer} or a via gate {at_mm, via: {from_layer, to_layer}} (layers enabled copper, not the same)" );

                    c.points.push_back( x );
                }

                if( c.points.empty() )
                    return KOPENAPI_RESULT::Error( 400, "bus_paths: points_mm: one point at least" );

                corridors.push_back( std::move( c ) );
            }
        }
        else if( !( pathsArg.is_string() && pathsArg.get<std::string>() == "off" ) )
        {
            return KOPENAPI_RESULT::Error( 400, "bus_paths: \"off\", \"floorplan\" (or {floorplan: {its arguments}}) or "
                                                "[{name, nets, points_mm}]" );
        }

        if( grid && !corridors.empty() )
        {
            corridorOut.push_back( { { "skipped", "engine grid: its bundles route the members side by side" } } );
            corridors.clear();
        }

        auto refOf = []( BOARD_CONNECTED_ITEM* aItem )
        {
            FOOTPRINT* fp = aItem && aItem->Type() == PCB_PAD_T ? static_cast<PAD*>( aItem )->GetParentFootprint() : nullptr;
            return fp ? str( fp->GetReference() ) : std::string();
        };

        // per package of 8 SMD pads (or edge fingers) or more its plan's ports, asked once; a pin's tip on
        // its pad's layer
        std::map<const FOOTPRINT*, std::map<const PAD*, PORT_GOAL>> fanPlans;
        std::optional<KOPENAPI_METHOD>                               fanPlanner = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout_ports" );
        int                                                          fannedOut = 0;

        // lanes: the last track of a pin's laid lane (its end the lane's front), when its lane is laid
        auto laneEndOf = [&]( BOARD_CONNECTED_ITEM* aEnd ) -> PCB_TRACK*
        {
            const PAD* pad = lanesOn && aEnd && aEnd->Type() == PCB_PAD_T ? static_cast<const PAD*>( aEnd ) : nullptr;
            auto       it = pad ? portGoals.find( pad ) : portGoals.end();

            if( it == portGoals.end() || it->second.via || it->second.ring.empty() )
                return nullptr;

            return dynamic_cast<PCB_TRACK*>( board->ResolveItem( it->second.ring.back(), true ) );
        };

        auto fanTip = [&]( const PAD* aPad ) -> std::optional<VECTOR2I>
        {
            if( auto it = portGoals.find( aPad ); it != portGoals.end() )
                return it->second.other ? std::nullopt : std::optional<VECTOR2I>( it->second.tip );

            FOOTPRINT* fp = aPad->GetParentFootprint();

            if( !fp || !fanPlanner )
                return std::nullopt;

            auto plan = fanPlans.find( fp );

            if( plan == fanPlans.end() )
            {
                plan = fanPlans.emplace( fp, std::map<const PAD*, PORT_GOAL>() ).first;
                const size_t smd = std::count_if( fp->Pads().begin(), fp->Pads().end(),
                                                  []( PAD* p ) { return p->GetAttribute() == PAD_ATTRIB::SMD
                                                                        || p->GetAttribute() == PAD_ATTRIB::CONN; } );

                if( smd >= 8 )
                {
                    const KOPENAPI_RESULT r = fanPlanner->handler( aCtx, { { "ref", str( fp->GetReference() ) } } );
                    std::vector<std::string> viaNets;

                    if( r.status == 200 && r.body.contains( "ports" ) && r.body["ports"].is_object() )
                        goalsFrom( fp, r.body["ports"], plan->second, viaNets );
                }
            }

            auto it = plan->second.find( aPad );
            return it == plan->second.end() || it->second.other ? std::nullopt : std::optional<VECTOR2I>( it->second.tip );
        };

        for( CORRIDOR& c : corridors )
        {
            std::set<int> codes;

            for( const std::string& n : c.nets )
            {
                if( NETINFO_ITEM* info = KopenapiFindNet( board, n ) )
                    codes.insert( info->GetNetCode() );
            }

            std::vector<MEMBER> members;
            const VECTOR2I      front = c.points.front().at, back = c.points.back().at;
            int                 partEnds = 0;   ///< members ending on copper of their net already there

            // the block an end is in: its part's; a track or via end (a net partly routed), the block
            // whose parts' pads are nearer
            std::vector<VECTOR2I> padsOf[2];

            for( FOOTPRINT* fp : board->Footprints() )
            {
                const std::string ref = str( fp->GetReference() );

                for( int side = 0; side < 2; ++side )
                {
                    if( ( side == 0 ? c.aParts : c.bParts ).count( ref ) )
                    {
                        for( PAD* pad : fp->Pads() )
                            padsOf[side].push_back( pad->GetPosition() );
                    }
                }
            }

            auto sideOf = [&]( BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aPos )
            {
                const std::string ref = refOf( aItem );

                if( !ref.empty() )
                    return c.aParts.count( ref ) ? 0 : c.bParts.count( ref ) ? 1 : -1;

                double nearest[2] = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };

                for( int side = 0; side < 2; ++side )
                {
                    for( const VECTOR2I& at : padsOf[side] )
                        nearest[side] = std::min( nearest[side], double( ( at - aPos ).EuclideanNorm() ) );
                }

                return nearest[0] <= nearest[1] ? 0 : 1;
            };

            // a connection's end as a corridor names it: "REF.PAD", or a copper item's point (and layer)
            auto endIs = [&]( const nlohmann::json& aEnd, BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aPos )
            {
                if( aEnd.is_string() )
                {
                    const PAD* pad = aItem && aItem->Type() == PCB_PAD_T ? static_cast<const PAD*>( aItem ) : nullptr;
                    return pad && refOf( aItem ) + "." + str( pad->GetNumber() ) == aEnd.get<std::string>();
                }

                const VECTOR2I at( pcbIUScale.mmToIU( aEnd["at_mm"][0].get<double>() ), pcbIUScale.mmToIU( aEnd["at_mm"][1].get<double>() ) );
                const std::optional<PCB_LAYER_ID> layer = aEnd.value( "layer", nlohmann::json() ).is_string()
                                                                  ? parseLayer( board, aEnd["layer"].get<std::string>() ) : std::nullopt;
                return ( aPos - at ).EuclideanNorm() <= pcbIUScale.mmToIU( 0.05 ) && ( !layer || ( aItem && aItem->IsOnLayer( *layer ) ) );
            };

            for( size_t k = 0; k < jobs.size(); ++k )
            {
                const JOB& j = jobs[k];

                if( j.trunk || j.foreign || j.busVia || j.corridorOf >= 0 || !codes.count( j.from->GetNetCode() ) )
                    continue;

                bool forward = true;

                if( !c.from.is_null() )
                {
                    const bool there = endIs( c.from, j.from, j.from->GetPosition() ) && endIs( c.to, j.to, j.toPos );
                    const bool back = endIs( c.to, j.from, j.from->GetPosition() ) && endIs( c.from, j.to, j.toPos );

                    if( !there && !back )
                        continue;

                    forward = there;
                }
                else if( !c.aParts.empty() || !c.bParts.empty() )
                {
                    const int from = sideOf( j.from, j.from->GetPosition() ), to = sideOf( j.to, j.toPos );

                    if( from < 0 || to < 0 || from == to )
                        continue;

                    forward = from == 0;
                    partEnds += !refOf( j.to ).empty() ? 0 : 1;
                }
                else
                {
                    const double fwd = ( j.from->GetPosition() - front ).EuclideanNorm() + ( j.toPos - back ).EuclideanNorm();
                    const double rev = ( j.from->GetPosition() - back ).EuclideanNorm() + ( j.toPos - front ).EuclideanNorm();
                    forward = fwd <= rev;
                }

                members.push_back( { k, forward, forward ? j.from->GetPosition() : j.toPos, forward ? j.toPos : j.from->GetPosition(), 0, j.from } );
            }

            // the fan-out from packages of 8 SMD pads or more, the plan's (pcb_fanout_ports): a member leaves
            // its pin straight out on the pin's side to past the courtyard, then closes up to the ribbon
            for( MEMBER& m : members )
            {
                JOB& j = jobs[m.job];

                for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( j.from ), j.to } )
                {
                    const PAD* pad = end && end->Type() == PCB_PAD_T ? static_cast<const PAD*>( end ) : nullptr;
                    const auto tip = pad ? fanTip( pad ) : std::nullopt;

                    if( !tip )
                        continue;

                    const bool atStart = ( end == j.from ) == m.forward;
                    ( atStart ? m.padStart : m.padEnd ) = atStart ? m.start : m.end;
                    ( atStart ? m.fanStart : m.fanEnd ) = true;
                    ( atStart ? m.start : m.end ) = *tip;
                    ( atStart ? j.fanStart : j.fanEnd ) = *tip;
                    fannedOut++;
                }
            }

            // lanes: at an end where every member's pin has its lane laid, the members start (end) at the lanes'
            // ends; the rings are their way round the package
            int laneEnds[2] = { 0, 0 };

            for( const MEMBER& m : members )
            {
                const JOB& j = jobs[m.job];

                for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( j.from ), j.to } )
                {
                    if( laneEndOf( end ) )
                        laneEnds[( ( end == j.from ) == m.forward ) ? 0 : 1]++;
                }
            }

            for( int side = 0; side < 2; ++side )
            {
                if( laneEnds[side] != (int) members.size() )
                {
                    laneEnds[side] = 0;
                    continue;
                }

                for( MEMBER& m : members )
                {
                    JOB&                  j = jobs[m.job];
                    BOARD_CONNECTED_ITEM* end = ( side == 0 ) == m.forward ? static_cast<BOARD_CONNECTED_ITEM*>( j.from ) : j.to;
                    const PCB_TRACK*      last = laneEndOf( end );

                    ( side == 0 ? m.start : m.end ) = last->GetEnd();
                    ( side == 0 ? m.fanStart : m.fanEnd ) = false;
                    ( end == j.from ? j.fanStart : j.fanEnd ) = std::nullopt;
                    ( end == j.from ? j.laneFrom : j.laneTo ) = true;
                }
            }

            nlohmann::json out = { { "name", c.name }, { "points", c.points.size() } };

            if( !c.a.empty() )
            {
                out["a"] = c.a;
                out["b"] = c.b;
            }

            if( members.size() < ( c.from.is_null() ? 2u : 1u ) )
            {
                out["skipped"] = members.empty() ? "no connection of its nets runs between its ends"
                                                 : "one connection of its nets runs between its ends";
                corridorOut.push_back( out );
                continue;
            }

            // every member leaving (reaching) by its lane: the way starts at the gate that makes the way from
            // the lane ends shortest, the gates round the package (the ring's part) dropped
            CORRIDOR along = c;
            int      laneTrimmed = 0;

            for( bool atStart : { true, false } )
            {
                std::vector<CROSSING>& pts = along.points;

                if( laneEnds[atStart ? 0 : 1] != (int) members.size() || pts.size() < 2 )
                    continue;

                VECTOR2D mid( 0, 0 ), farMid( 0, 0 );

                for( const MEMBER& m : members )
                {
                    mid += VECTOR2D( atStart ? m.start : m.end ) / (double) members.size();
                    farMid += VECTOR2D( atStart ? m.end : m.start ) / (double) members.size();
                }

                const size_t        count = pts.size();
                std::vector<double> rest( count, 0 );
                auto                at = [&]( size_t k ) { return VECTOR2D( pts[atStart ? k : count - 1 - k].at ); };

                rest[count - 1] = ( farMid - at( count - 1 ) ).EuclideanNorm();

                for( size_t k = count - 1; k-- > 0; )
                    rest[k] = rest[k + 1] + ( at( k + 1 ) - at( k ) ).EuclideanNorm();

                size_t best = 0;

                for( size_t k = 1; k < count; ++k )
                {
                    if( pts[atStart ? k : count - 1 - k].viaTo )
                        break;

                    if( ( at( k ) - mid ).EuclideanNorm() + rest[k] < ( at( best ) - mid ).EuclideanNorm() + rest[best] - 1 )
                        best = k;
                }

                if( best == 0 )
                    continue;

                if( atStart )
                    pts.erase( pts.begin(), pts.begin() + best );
                else
                    pts.erase( pts.end() - best, pts.end() );

                laneTrimmed += (int) best;
            }

            if( laneEnds[0] + laneEnds[1] )
            {
                out["lane_ends"] = laneEnds[0] + laneEnds[1];
                out["lane_gates_dropped"] = laneTrimmed;
            }

            LANES          lanes = laneAlong( along, codes, members, std::nullopt );
            const int      n = (int) members.size();
            const bool     insideLast = lanes.insideLast;
            const int      corridorIndex = (int) corridorOut.size();
            nlohmann::json across = nlohmann::json::array();

            for( int k = 0; k < n; ++k )
            {
                JOB& j = jobs[members[k].job];
                j.corridor = lanes.corners[k];
                j.corridorVia.clear();

                for( const auto& g : lanes.vias )
                    j.corridorVia.push_back( g ? std::make_optional( members[k].forward ? g->second : g->first ) : std::nullopt );

                if( j.fanStart )
                {
                    j.corridor.insert( j.corridor.begin(), *j.fanStart );
                    j.corridorVia.insert( j.corridorVia.begin(), std::nullopt );
                }

                if( j.fanEnd )
                {
                    j.corridor.push_back( *j.fanEnd );
                    j.corridorVia.push_back( std::nullopt );
                }

                if( !members[k].forward )
                {
                    std::reverse( j.corridor.begin(), j.corridor.end() );
                    std::reverse( j.corridorVia.begin(), j.corridorVia.end() );
                }

                j.corridorOf = corridorIndex;
                j.corridorRank = insideLast ? n - 1 - k : k;
                j.corridorPlace = k;
                across.push_back( j.net );
            }

            corridorMembers += n;
            out.update( lanes.out );
            out["nets"] = across;
            out["routed_from"] = insideLast ? "the last (inside of the first turn)" : "the first";

            if( partEnds )
                out["ends_on_copper"] = partEnds;

            if( fannedOut )
                out["fanned_out"] = fannedOut;

            fannedOut = 0;

            corridorOut.push_back( out );
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
    std::vector<bool> overFirstBudget( jobs.size(), false );   // the first pass ran out of time for it

    // Ways out of the fine-pitch pads on the edges of their packages (pads inside the ring of an area
    // array are left to the fan-out): from the pad's end outward, room for a track or a via.  The
    // call's pads, and the pads of other nets with an open connection: a later call routes them
    const bool escapesOn = aArgs.value( "escapes", true );

    if( ( escapesOn || !portGoals.empty() ) && !rail )
    {
        std::map<PAD*, std::vector<size_t>> frontsOf;

        // A signal pad of a fine-pitch row next to a rail pin whose via sits inward of the row: the row
        // pins on that side leave inward around the via, the way in front of this pad's inner end is
        // its own (as deep as the via reaches, then room for a track); a route of a neighbour on the row
        // (within 3.5 pitches) crossing it there walls the pad in, other nets may pass
        std::vector<PCB_VIA*> rowVias;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() == PCB_VIA_T )
                rowVias.push_back( static_cast<PCB_VIA*>( t ) );
        }

        auto inwardFront = [&]( PAD* aPad, std::optional<size_t> aJob, std::vector<size_t>& aOf )
        {
            if( !aPad->GetParentFootprint() || !finePad( aPad ) )
                return;

            const double pitch = pitchOf( aPad );

            FOOTPRINT* fp = aPad->GetParentFootprint();
            BOX2I      ring = fp->Pads().front()->GetBoundingBox();

            for( PAD* q : fp->Pads() )
                ring.Merge( q->GetBoundingBox() );

            const BOX2I    box = aPad->GetBoundingBox();
            const VECTOR2I d = aPad->GetPosition() - ring.Centre();
            const bool     horizontal = (double) std::abs( d.x ) * ring.GetHeight() >= (double) std::abs( d.y ) * ring.GetWidth();
            const VECTOR2I in = horizontal ? VECTOR2I( d.x >= 0 ? -1 : 1, 0 ) : VECTOR2I( 0, d.y >= 0 ? -1 : 1 );
            const int      half = horizontal ? box.GetWidth() / 2 : box.GetHeight() / 2;
            const VECTOR2I end = box.Centre() + in * half;   // the pad's inner end
            NETCLASS*      netclass = aPad->GetEffectiveNetClass();
            const int      clearance = std::max( std::max( ds.m_MinClearance, 0 ), netclass && netclass->HasClearance() ? netclass->GetClearance() : 0 );
            const int      width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : ds.GetCurrentTrackWidth();
            int            depth = 0;

            for( PCB_VIA* via : rowVias )
            {
                const VECTOR2I rel = via->GetPosition() - end;
                const int      along = horizontal ? rel.x * in.x : rel.y * in.y;
                const int      aside = std::abs( horizontal ? rel.y : rel.x );
                const int      r = via->GetWidth( via->TopLayer() ) / 2;

                // a via in the band just inward of the row, beside the pad (the next pin or the one after)
                if( along <= 0 || along > 3 * r + 2 * clearance || aside > KiROUND( 2.5 * pitch ) )
                    continue;

                // a rail pin with its own via there leaves by it
                if( via->GetNetCode() == aPad->GetNetCode() )
                    return;

                if( aside < r )
                    continue;

                bool railNeighbour = false;

                for( PAD* q : fp->Pads() )
                {
                    if( q != aPad && q->GetNetCode() == via->GetNetCode()
                        && ( q->GetPosition() - aPad->GetPosition() ).EuclideanNorm() < 2.5 * pitch )
                    {
                        railNeighbour = true;
                    }
                }

                if( railNeighbour )
                    depth = std::max( depth, along + r + 2 * clearance + width );
            }

            if( depth <= 0 )
                return;

            // only the neighbours on the row could cross it there
            auto nets = std::make_shared<std::set<int>>();

            for( PAD* q : fp->Pads() )
            {
                if( q->GetNetCode() > 0 && q->GetNetCode() != aPad->GetNetCode()
                    && ( q->GetPosition() - aPad->GetPosition() ).EuclideanNorm() < 3.5 * pitch )
                {
                    nets->insert( q->GetNetCode() );
                }
            }

            for( PCB_LAYER_ID layer : layers )
            {
                if( !aPad->IsOnLayer( layer ) || !aPad->FlashLayer( layer ) )
                    continue;

                ESCAPE_FRONT front;
                front.pad = aPad;
                front.layer = layer;
                front.shape = SHAPE_SEGMENT( end + in * ( width / 2 ), end + in * ( depth - width / 2 ), width );
                front.only = nets;

                if( aJob )
                    front.jobs.push_back( *aJob );

                aOf.push_back( fronts.size() );
                fronts.push_back( std::move( front ) );
            }
        };

        auto addFronts = [&]( PAD* aPad, std::optional<size_t> aJob )
        {
            if( auto it = frontsOf.find( aPad ); it != frontsOf.end() )
            {
                for( size_t f : it->second )
                {
                    if( aJob )
                        fronts[f].jobs.push_back( *aJob );
                }

                return;
            }

            frontsOf[aPad];

            // a bus member's way out is its fan via (kept below)
            if( aJob && jobs[*aJob].busVia && busLayer )
                return;

            inwardFront( aPad, aJob, frontsOf[aPad] );

            // a planned port: its way out to past the courtyard, or to its via and the via's spot
            if( auto port = portGoals.find( aPad ); port != portGoals.end() )
            {
                const PORT_GOAL& goal = port->second;
                NETCLASS*        netclass = aPad->GetEffectiveNetClass();
                const int        dia = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : ds.GetCurrentViaSize();

                auto add = [&]( PCB_LAYER_ID aLayer, const SHAPE_SEGMENT& aShape )
                {
                    ESCAPE_FRONT front;
                    front.pad = aPad;
                    front.layer = aLayer;
                    front.shape = aShape;
                    front.port = true;

                    if( aJob )
                        front.jobs.push_back( *aJob );

                    frontsOf[aPad].push_back( fronts.size() );
                    fronts.push_back( std::move( front ) );
                };

                if( goal.via )
                {
                    add( goal.layer, SHAPE_SEGMENT( goal.start, *goal.via, goal.width ) );

                    for( PCB_LAYER_ID l : layers )
                        add( l, SHAPE_SEGMENT( *goal.via, *goal.via, dia ) );
                }
                else
                {
                    add( goal.layer, SHAPE_SEGMENT( goal.start, goal.tip, goal.width ) );
                }

                return;
            }

            if( !escapesOn )
                return;

            if( !finePad( aPad ) )
                return;

            const double pitch = pitchOf( aPad );

            // a via of the pad's net with the clearance the rules ask to its row neighbours on both
            // sides; where two vias do not fit side by side at the pitch, room for the neighbours' vias on
            // a second, staggered line
            FOOTPRINT*   owner = aPad->GetParentFootprint();
            const PCB_LAYER_ID padLayer = aPad->GetPrincipalLayer();
            KOPENAPI_RULE_VIA rule = KopenapiRuleVia( board, aPad->GetNetCode(), aPad->GetPosition(), padLayer, nullptr );

            for( PAD* q : owner->Pads() )
            {
                if( q != aPad && q->GetNetCode() != aPad->GetNetCode() && KopenapiPadNeighbours( q, aPad ) )
                    rule.clearance = std::max( rule.clearance, KopenapiRuleVia( board, aPad->GetNetCode(), aPad->GetPosition(), padLayer, q ).clearance );
            }

            const int    clearance = rule.clearance;
            const int    via = rule.dia;
            const double apart = via + clearance;
            const int    depth = 2 * clearance + via + ( pitch < apart ? KiROUND( std::sqrt( apart * apart - pitch * pitch ) ) : 0 );

            FOOTPRINT* fp = aPad->GetParentFootprint();
            BOX2I      ring = fp->Pads().front()->GetBoundingBox();

            for( PAD* q : fp->Pads() )
                ring.Merge( q->GetBoundingBox() );

            const BOX2I    box = aPad->GetBoundingBox();
            const VECTOR2I d = aPad->GetPosition() - ring.Centre();
            const bool     horizontal = (double) std::abs( d.x ) * ring.GetHeight() >= (double) std::abs( d.y ) * ring.GetWidth();
            const VECTOR2I dir = horizontal ? VECTOR2I( d.x >= 0 ? 1 : -1, 0 ) : VECTOR2I( 0, d.y >= 0 ? 1 : -1 );
            const int      half = horizontal ? box.GetWidth() / 2 : box.GetHeight() / 2;
            const int      across = horizontal ? box.GetHeight() : box.GetWidth();
            const int      toEdge = horizontal ? ( dir.x > 0 ? ring.GetRight() - box.GetRight() : box.GetLeft() - ring.GetLeft() )
                                               : ( dir.y > 0 ? ring.GetBottom() - box.GetBottom() : box.GetTop() - ring.GetTop() );

            if( toEdge > pitch / 2 || depth <= across )
                return;

            const VECTOR2I tip = box.Centre() + dir * half;

            // as wide as the net's track, not the pad: the neighbours' tracks leave between the rooms
            const int width = std::min( across, KopenapiRuleTrack( board, aPad->GetNetCode(), tip, padLayer ).opt );

            for( PCB_LAYER_ID layer : layers )
            {
                if( !aPad->IsOnLayer( layer ) || !aPad->FlashLayer( layer ) )
                    continue;

                ESCAPE_FRONT front;
                front.pad = aPad;
                front.layer = layer;
                front.shape = SHAPE_SEGMENT( tip + dir * ( width / 2 ), tip + dir * ( depth - width / 2 ), width );

                if( aJob )
                    front.jobs.push_back( *aJob );

                frontsOf[aPad].push_back( fronts.size() );
                fronts.push_back( std::move( front ) );
            }
        };

        for( size_t k = 0; k < jobs.size(); ++k )
        {
            if( partner[k] >= 0 )
                continue;

            for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( jobs[k].from ), jobs[k].to } )
            {
                if( PAD* pad = dynamic_cast<PAD*>( end ); pad && pad->GetParentFootprint() )
                    addFronts( pad, k );
            }
        }

        std::set<int> callNets;

        for( const JOB& job : jobs )
            callNets.insert( job.from ? job.from->GetNetCode() : 0 );

        for( NETINFO_ITEM* net : board->GetNetInfo() )
        {
            if( net->GetNetCode() <= 0 || callNets.count( net->GetNetCode() ) )
                continue;

            RN_NET* rn = connectivity->GetRatsnestForNet( net->GetNetCode() );

            if( !rn )
                continue;

            for( const CN_EDGE& edge : rn->GetEdges() )
            {
                for( const std::shared_ptr<const CN_ANCHOR>& node : { edge.GetSourceNode(), edge.GetTargetNode() } )
                {
                    if( !node || node->Dirty() || !node->Parent() || node->Parent()->Type() != PCB_PAD_T )
                        continue;

                    PAD* pad = static_cast<PAD*>( node->Parent() );

                    if( pad->GetParentFootprint() )
                        addFronts( pad, std::nullopt );
                }
            }
        }

        doneOf = &done;
    }

    // a bus member's via spot is kept free while it waits
    for( size_t k = 0; k < jobs.size(); ++k )
    {
        if( !jobs[k].busVia || !busLayer )
            continue;

        NETCLASS* netclass = jobs[k].from->GetEffectiveNetClass();
        const int dia = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter() : ds.GetCurrentViaSize();

        for( PCB_LAYER_ID l : layers )
        {
            ESCAPE_FRONT front;
            front.pad = jobs[k].from;
            front.layer = l;
            front.shape = SHAPE_SEGMENT( *jobs[k].busVia, *jobs[k].busVia, dia );
            front.jobs.push_back( k );
            fronts.push_back( std::move( front ) );
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

        // with work budgets the plateau's clock is the work: every call counts
        if( !workBudgets && aPass == progressPass && now - progressAt < std::chrono::milliseconds( 200 ) )
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

        if( routed > plateauBest )
        {
            plateauBest = routed;
            plateauSince = clockNow();
        }

        plateauCheck( clockNow() );

        const int connections = (int) ( jobs.size() - trunks.size() + gaps.size()
                                        - std::count_if( jobs.begin(), jobs.end(), []( const JOB& j ) { return j.foreign; } ) );

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

    // corridors as soft goals: at the turn of a corridor's first connection, each of its members one try
    // along it, across it in order, in corridor_try_budget_s; a member it
    // routes is kept for its own turn, the others keep their turn and whole budget as without the corridor
    std::vector<int>            alongTried( jobs.size(), 0 );   // 1: routed along, -1: failed along
    std::vector<nlohmann::json> alongAttempt( jobs.size() ), alongRow( jobs.size() );
    std::set<int>               corridorsTried;

    auto tryAlong = [&]( size_t aIndex, int aOrder )
    {
        JOB& job = jobs[aIndex];
        job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
        job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

        if( !job.from || !job.to )
            return;

        const PCB_LAYER_ID padLayer = job.from->IsOnLayer( F_Cu ) ? F_Cu : job.from->IsOnLayer( B_Cu ) ? B_Cu
                                                                                                     : job.from->GetLayerSet().CuStack().front();
        const auto alongStarted = std::chrono::steady_clock::now();
        const long outer = profile.job;
        profile.job = (long) aIndex;

        if( doneOf )
            refreshFronts();

        ROUTE_REQUEST req{ job.from, job.to, job.toPos, padLayer, PNS::RM_Walkaround, dryRun, {} };
        req.fronts = &fronts;
        req.neckWidth = neckFor( job.from, job.to );
        req.pairTail = pairTail( job.to );

        for( size_t k = 0; k < job.corridor.size(); ++k )
            req.vias.emplace_back( job.corridor[k], k < job.corridorVia.size() ? job.corridorVia[k] : std::nullopt );

        // its corridor runs from / to its laid lanes' ends: the route goes on from that copper
        PORT_GOAL* laneGoal[2] = { nullptr, nullptr };

        for( int side = 0; side < 2; ++side )
        {
            BOARD_CONNECTED_ITEM* end = side == 0 ? static_cast<BOARD_CONNECTED_ITEM*>( job.from ) : job.to;
            auto                  it = ( side == 0 ? job.laneFrom : job.laneTo ) && end && end->Type() == PCB_PAD_T
                                               ? portGoals.find( static_cast<PAD*>( end ) ) : portGoals.end();
            PCB_TRACK*            last = it != portGoals.end() && !it->second.ring.empty()
                                                 ? dynamic_cast<PCB_TRACK*>( board->ResolveItem( it->second.ring.back(), true ) )
                                                 : nullptr;

            if( !last )
                continue;

            laneGoal[side] = &it->second;

            if( side == 0 )
            {
                req.from = last;
                req.fromPos = last->GetEnd();
                req.layer = last->GetLayer();
            }
            else
            {
                req.to = last;
                req.toPos = last->GetEnd();
            }
        }

        setDeadline( corridorTryBudgetS );
        nlohmann::json attempt = nlohmann::json::object();
        const bool     ok = routeOne( *context, req, attempt, added );

        // the lanes it went on from are its copper now: ripped up with it
        for( PORT_GOAL* g : laneGoal )
        {
            if( !ok || !g )
                continue;

            for( const KIID& id : g->ring )
            {
                profile.laid[id] = { profile.pass, profile.job, profile.attempts };
                added.push_back( id );
            }

            g->ring.clear();
            job.laneRouted = true;
        }
        tryDeadline.reset();
        job.ms += msSince( alongStarted );
        profile.job = outer;

        job.corridorRouted = ok;
        corridorRoutedCount += ok;
        done[aIndex] = done[aIndex] || ok;
        alongTried[aIndex] = ok ? 1 : -1;
        alongAttempt[aIndex] = attempt;
        alongRow[aIndex] = { { "name", corridorOut[job.corridorOf].value( "name", std::string() ) },
                             { "index", job.corridorPlace }, { "order", aOrder }, { "corners", job.corridor.size() }, { "routed", ok } };

        if( !ok )
            alongRow[aIndex]["failed"] = attempt.value( "reason", std::string() );
    };

    // a pair line's end track gone since: the net's track through the old end now (a route onto it
    // merged with it), else its track end nearest the old place off the start's island (shoved: new
    // segments)
    auto pairEndAgain = [&]( JOB& aJob )
    {
        if( !aJob.toPairEnd || !aJob.from )
            return;

        const bool split = pairLineEnds.count( aJob.toId ) > 0;
        const auto joined = board->GetConnectivity()->GetConnectedItems( aJob.from );
        double     best = std::numeric_limits<double>::max();

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() == PCB_TRACE_T && t->GetNetCode() == aJob.from->GetNetCode() && t->HitTest( aJob.toPos, 0 ) )
            {
                aJob.to = t;
                aJob.toId = t->m_Uuid;
                best = -1.0;
                break;
            }
        }

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( best < 0 || t->Type() != PCB_TRACE_T || t->GetNetCode() != aJob.from->GetNetCode()
                || std::find( joined.begin(), joined.end(), t ) != joined.end() )
            {
                continue;
            }

            for( const VECTOR2I& at : { t->GetStart(), t->GetEnd() } )
            {
                if( const double d = ( at - aJob.toPos ).EuclideanNorm(); d < best )
                {
                    best = d;
                    aJob.to = t;
                    aJob.toId = t->m_Uuid;
                    aJob.toPos = at;
                }
            }
        }

        if( split && aJob.to )
            pairLineEnds.insert( aJob.toId );
    };

    // a first pass bounded per connection: the plateau waits until each connection has had its try
    plateauHeld = firstBudgetS > 0 ? 1 : 0;

    for( size_t index = 0; index < jobs.size(); ++index )
    {
        JOB& job = jobs[index];

        if( job.corridorOf >= 0 && !job.corridor.empty() && job.bus < 0 && !grid && corridorsTried.insert( job.corridorOf ).second )
        {
            std::vector<size_t> members;

            for( size_t k = 0; k < jobs.size(); ++k )
            {
                if( jobs[k].corridorOf == job.corridorOf && !jobs[k].corridor.empty() && jobs[k].bus < 0 && rowOf[k] < 0 )
                    members.push_back( k );
            }

            std::stable_sort( members.begin(), members.end(),
                              [&]( size_t x, size_t y ) { return jobs[x].corridorRank < jobs[y].corridorRank; } );

            for( size_t k = 0; k < members.size(); ++k )
                tryAlong( members[k], (int) k );
        }

        if( grid && partner[index] < 0 )
            continue;   // on the grid below

        if( !KopenapiJobStep( "route", (int) index, (int) jobs.size(), job.net, progressCounts( "first" ) ) )
            break;

        job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
        job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

        if( !job.to )
            pairEndAgain( job );

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
            const size_t   pairFrom = added.size();

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
                        req.pairs = &explicitPairs;

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

            // the pair's placer fixes what it laid when it cannot snap onto both target pads (pads far
            // apart: two parts not side by side): the coupled part stays, each net's airwire is then
            // routed alone from its far pad to the end of the pair's line
            std::map<int, std::pair<PCB_TRACK*, VECTOR2I>> pairEnds;   // net code -> its line's open end

            if( pairOk && !dryRun )
            {
                for( JOB* mine : { &job, &other } )
                {
                    const auto joined = board->GetConnectivity()->GetConnectedItems( mine->from );

                    if( std::find( joined.begin(), joined.end(), mine->to ) != joined.end() )
                        continue;

                    std::optional<std::pair<PCB_TRACK*, VECTOR2I>> end;

                    for( size_t i = pairFrom; i < added.size(); ++i )
                    {
                        PCB_TRACK* t = dynamic_cast<PCB_TRACK*>( board->ResolveItem( added[i], true ) );

                        if( !t || t->Type() != PCB_TRACE_T || t->GetNetCode() != mine->from->GetNetCode() )
                            continue;

                        for( const VECTOR2I& at : { t->GetStart(), t->GetEnd() } )
                        {
                            if( !end || ( at - mine->to->GetPosition() ).SquaredEuclideanNorm()
                                                < ( end->second - mine->to->GetPosition() ).SquaredEuclideanNorm() )
                            {
                                end = std::make_pair( t, at );
                            }
                        }
                    }

                    if( end )
                        pairEnds[mine->from->GetNetCode()] = *end;
                }
            }

            // to two parts side by side the placer bends one line onto the pad it reaches and stops the
            // other short beside it, closer to its partner than a single route may come: both lines are cut
            // back to where each has room to turn to its own pad, and each net goes on alone from there
            double pairSplit = 0.0;

            if( !pairEnds.empty() )
                pairSplit = splitPairBack( *context, board, added, pairFrom,
                               { { job.from->GetNetCode(), job.from->GetPosition(), job.to->GetPosition() },
                                 { other.from->GetNetCode(), other.from->GetPosition(), other.to->GetPosition() } },
                               pairAttempt, pairEnds );

            if( !pairEnds.empty() )
            {
                for( size_t k = 0; k < jobs.size(); ++k )
                {
                    JOB& wire = jobs[k];

                    if( wire.trunk || rowOf[k] >= 0 || done[k] || !wire.from || !wire.to
                        || !pairEnds.count( wire.from->GetNetCode() ) )
                    {
                        continue;
                    }

                    const auto& [track, at] = pairEnds[wire.from->GetNetCode()];
                    const auto  joined = board->GetConnectivity()->GetConnectedItems( track );
                    const bool  fromOn = std::find( joined.begin(), joined.end(), wire.from ) != joined.end();
                    const bool  toOn = std::find( joined.begin(), joined.end(), wire.to ) != joined.end();
                    PAD*        farPad = !fromOn ? wire.from : dynamic_cast<PAD*>( wire.to );

                    if( fromOn == toOn || !farPad )
                        continue;

                    wire.from = farPad;
                    wire.fromId = farPad->m_Uuid;
                    wire.to = track;
                    wire.toId = track->m_Uuid;
                    wire.toPos = at;
                    wire.length = (double) ( at - farPad->GetPosition() ).EuclideanNorm();
                    wire.toPairEnd = true;

                    if( pairSplit > 0 )
                        pairLineEnds.insert( track->m_Uuid );
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

                    if( pairEnds.count( mine->from->GetNetCode() ) )
                    {
                        const VECTOR2I at = pairEnds[mine->from->GetNetCode()].second;
                        row["diff_pair"]["ends_short_mm"] = { toMm( at.x ), toMm( at.y ) };
                    }

                    if( pairSplit > 0 )
                        row["diff_pair"]["split_back_mm"] = toMm( pairSplit );

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
        if( !dryRun && alongTried[index] <= 0 && job.to->Type() == PCB_PAD_T )
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

        plateauTurn();

        if( firstBudgetS > 0 )
            setDeadline( firstBudgetS );

        // a bus member with its via spot: from its pad to the via, on to its far end on bus_layer
        nlohmann::json busRow;

        if( job.bus >= 0 )
            busRow = { { "name", busOut[job.bus].value( "name", std::string() ) }, { "index", job.busIndex } };

        bool ok = false;

        if( job.busVia && busLayer )
        {
            const PCB_LAYER_ID padLayer = job.from->IsOnLayer( F_Cu ) ? F_Cu : job.from->IsOnLayer( B_Cu ) ? B_Cu
                                                                                                         : job.from->GetLayerSet().CuStack().front();
            const auto fanStarted = std::chrono::steady_clock::now();
            const long outer = profile.job;
            profile.job = (long) index;

            if( doneOf )
                refreshFronts();

            ROUTE_REQUEST req{ job.from, job.to, job.toPos, padLayer, PNS::RM_Walkaround, dryRun, {} };
            req.fronts = &fronts;
            req.neckWidth = neckFor( job.from, job.to );
            req.pairTail = pairTail( job.to );
            req.vias = { { *job.busVia, *busLayer } };
            attempt = nlohmann::json::object();
            tries++;
            ok = routeOne( *context, req, attempt, added );
            job.ms += msSince( fanStarted );
            profile.job = outer;
            busRow["via_mm"] = { toMm( job.busVia->x ), toMm( job.busVia->y ) };
            busRow["fan"] = ok;

            if( !ok )
                busRow["fan_failed"] = attempt.value( "reason", std::string() );
        }

        nlohmann::json corridorRow;

        if( !ok && alongTried[index] != 0 )
        {
            corridorRow = alongRow[index];

            if( alongTried[index] > 0 )
            {
                ok = true;
                attempt = alongAttempt[index];
            }
        }

        if( !ok )
            ok = tryRoute( job, attempt, tries, true, false );

        tryDeadline.reset();

        // the budget ran out: the retry pass tries it again, the forward way included
        if( !ok && attempt.contains( "budget" ) )
        {
            overFirstBudget[index] = true;
            profile.overFirstBudget.push_back( job.net + " " + str( job.from->GetParentFootprint()->GetReference() ) + "."
                                               + str( job.from->GetNumber() ) );
        }

        nlohmann::json row = { { "net", job.net },
                               { "from", str( job.from->GetParentFootprint()->GetReference() ) + "." + str( job.from->GetNumber() ) },
                               { "routed", ok }, { "tries", tries } };

        if( job.to->Type() == PCB_PAD_T )
            row["to"] = str( static_cast<PAD*>( job.to )->GetParentFootprint()->GetReference() ) + "."
                        + str( static_cast<PAD*>( job.to )->GetNumber() );

        if( busRow.is_object() )
            row["bus"] = busRow;

        if( corridorRow.is_object() )
            row["corridor"] = corridorRow;

        // GUI, for watching: each accepted route is drawn before the next one starts
        if( ok && stepMs > 0 && !aQuiet && !aCtx.headless && aCtx.kiway )
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

            if( attempt.contains( "inward_via_mm" ) )
                row["inward_via_mm"] = attempt["inward_via_mm"];

            if( attempt.contains( "necked_from_mm" ) )
                row["necked_from_mm"] = attempt["necked_from_mm"];

            if( attempt.contains( "left_pad_by" ) )
                row["left_pad_by_mm"] = attempt["left_pad_by"];

            if( attempt.contains( "port" ) )
                row["port"] = attempt["port"];
        }
        else
        {
            row["reason"] = attempt.value( "reason", std::string() );

            for( const char* key : { "violations", "stopped_at_mm", "blocked_by", "plan", "boxed_in", "pad_exits_tried_mm", "budget" } )
            {
                if( attempt.contains( key ) )
                    row[key] = attempt[key];
            }
        }

        rowOf[index] = (int) results.size();
        results.push_back( row );
    }

    plateauHeld = 0;
    plateauTurnCut = false;

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
                    if( end && end->Type() == PCB_PAD_T && finePad( static_cast<PAD*>( end ) ) )
                    {
                        if( const int w = ruleWidthAt( static_cast<PAD*>( end ), code ); w > 0 )
                            n.width = std::min( n.width, std::max( w, ds.m_TrackMinWidth ) );
                    }
                }
            }

            endsOfJob[index] = { endIndex( n, job.from ), endIndex( n, job.to ) };

            // a pair's line cut short: joined at its open end, not anywhere along it
            if( job.toPairEnd )
            {
                n.endAt.resize( n.ends.size() );
                n.endAt[endIndex( n, job.to )] = job.toPos;
            }
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
        params.coarse = ( globalStage || detailStage ) && params.cell == 0;
        params.space = globalStage || detailStage;
        params.raceWait = (float) std::clamp( aArgs.value( "race_wait", globalStage || detailStage ? 0.1 : 0.25 ), 0.0, 4.0 );
        params.raceWaitMinS = (float) std::clamp( aArgs.value( "race_wait_min_s", 2.0 ), 0.0, 600.0 );
        params.viaKeepOut = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_keepout_mm", 5.0 ), 0.0, 20.0 ) );

        for( const ESCAPE_FRONT& front : fronts )
        {
            if( front.jobs.empty() && !front.only )
                params.keepOpen.push_back( { front.layer, front.shape, front.pad->GetNetCode() } );
        }
        params.viaAway = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_away_mm", 7.0 ), 0.0, 30.0 ) );
        params.viaAwayCost = pcbIUScale.mmToIU( std::clamp( aArgs.value( "via_away_cost_mm", 8.0 ), 0.0, 100.0 ) );
        params.rowsCost = (float) std::clamp( aArgs.value( "rows_cost", 6.0 ), 1.0, 100.0 );
        params.bodyCost = (float) std::clamp( aArgs.value( "body_cost", 1.0 ), 1.0, 100.0 );
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
        auto gridCounts = [&]( const std::vector<GRID_VARIANT_PROGRESS>& aVariants, int& aFinished )
        {
            nlohmann::json variants = nlohmann::json::array();
            int            crowded = INT_MAX;
            aFinished = 0;

            for( const GRID_VARIANT_PROGRESS& v : aVariants )
            {
                aFinished += v.stage == "done" || v.stage == "abandoned";
                crowded = std::min( crowded, v.crowded );
                variants.push_back( { { "variant", v.name }, { "stage", v.stage }, { "round", v.round },
                                      { "crowded", v.crowded }, { "net", v.net }, { "of", v.nets },
                                      { "expanded", v.expanded }, { "held", v.held }, { "step", v.step } } );
            }

            // done / total: variants finished; each variant's own stage below
            nlohmann::json counts = wxThread::IsMain() ? progressCounts( "grid" ) : nlohmann::json::object();
            counts["nets"] = gridNets.size();
            counts["crowded"] = crowded;
            counts["variants"] = variants;
            return counts;
        };

        params.progress = [&]( const std::vector<GRID_VARIANT_PROGRESS>& aVariants )
        {
            if( params.record )
                GridReplayAdvance();

            int                  finished = 0;
            const nlohmann::json counts = gridCounts( aVariants, finished );
            return KopenapiJobStep( "grid", finished, (int) aVariants.size(), std::string(), counts );
        };

        params.live = [&]( std::function<std::vector<GRID_VARIANT_PROGRESS>()> aNow )
        {
            if( !aNow )
            {
                KopenapiJobRefresh( nullptr );
                return;
            }

            KopenapiJobRefresh( [&gridCounts, aNow]()
                                {
                                    int finished = 0;
                                    return gridCounts( aNow(), finished );
                                } );
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

        // a way through a net's grid copper as a corridor (docs corridor.md): a gate at each corner (zero
        // wide, on the run's layer, the rooms of the run to the next), a via gate where the layer changes
        // at a via, the last point a free point; the rooms widened from the run's cells into the net's
        // share of the free space
        GRID_ROOMS rooms( gr.space, gr.nets );

        auto cornerEnd = [&]( BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aAt ) -> nlohmann::json
        {
            if( aItem && aItem->Type() == PCB_PAD_T )
                return endName( aItem );

            nlohmann::json end = { { "at_mm", { toMm( aAt.x ), toMm( aAt.y ) } } };

            if( aItem && aItem->Type() != PCB_VIA_T )
                end["layer"] = str( board->GetLayerName( aItem->GetLayer() ) );

            return end;
        };

        auto corridorOfWay = [&]( size_t aNet, const std::string& aNetName, const GRID_CORRIDOR& aWay, nlohmann::json aFrom,
                                  nlohmann::json aTo ) -> nlohmann::json
        {
            nlohmann::json points = nlohmann::json::array();

            auto boxes = []( const std::vector<BOX2I>& aRooms )
            {
                nlohmann::json out = nlohmann::json::array();

                for( const BOX2I& r : aRooms )
                    out.push_back( { toMm( r.GetLeft() ), toMm( r.GetTop() ), toMm( r.GetRight() ), toMm( r.GetBottom() ) } );

                return out;
            };

            for( size_t r = 0; r < aWay.runs.size(); ++r )
            {
                const std::vector<VECTOR2I>& pts = aWay.runs[r].points;
                const std::string            layer = str( board->GetLayerName( aWay.runs[r].layer ) );

                for( size_t q = 0; q < pts.size(); ++q )
                {
                    if( q + 1 == pts.size() && r + 1 < aWay.runs.size() )
                        continue;

                    nlohmann::json gate = { { "at_mm", { toMm( pts[q].x ), toMm( pts[q].y ) } }, { "layer", layer } };

                    if( q + 1 < pts.size() )
                    {
                        const VECTOR2I d = pts[q + 1] - pts[q];
                        gate["rooms_mm"] = boxes( rooms.Rooms( (int) aNet, aWay.runs[r].layer, pts[q], pts[q + 1] ) );

                        if( r > 0 && q == 0 && std::find( aWay.vias.begin(), aWay.vias.end(), pts[q] ) != aWay.vias.end() )
                        {
                            gate["via"] = { { "from_layer", str( board->GetLayerName( aWay.runs[r - 1].layer ) ) }, { "to_layer", layer } };
                        }
                        else
                        {
                            const bool acrossY = std::abs( d.x ) >= std::abs( d.y );
                            const double u = toMm( acrossY ? pts[q].y : pts[q].x );
                            gate["along"] = acrossY ? "y" : "x";
                            gate["lo_mm"] = u;
                            gate["hi_mm"] = u;
                        }
                    }

                    points.push_back( gate );
                }
            }

            return { { "name", aNetName }, { "net", aNetName }, { "from", aFrom }, { "to", aTo }, { "points_mm", points } };
        };

        // the global stage: each connection's way through its net's grid copper, nothing laid
        if( globalStage )
        {
            nlohmann::json rows = results;
            int            covered = 0, connections = 0, vias = 0;

            auto corridorRow = [&]( size_t aNet, BOARD_CONNECTED_ITEM* aFrom, BOARD_CONNECTED_ITEM* aTo, const VECTOR2I& aToPos,
                                   const std::string& aName )
            {
                connections++;
                nlohmann::json      row = { { "net", aName }, { "from", endName( aFrom ) }, { "to", endName( aTo ) }, { "engine", "grid" } };
                const GRID_CORRIDOR c = aNet < gr.nets.size() ? GridCorridor( gridNets[aNet], gr.nets[aNet], aFrom, aTo, gr.cell ) : GRID_CORRIDOR();
                row["found"] = c.found;

                if( !c.found )
                {
                    const bool joined = aNet < gr.nets.size() && gr.nets[aNet].complete;
                    row["reason"] = KopenapiJobCancelled() ? "stopped"
                                    : joined ? "its net joined on the grid, but not these two ends in its copper"
                                             : "no way on the grid (or its net dropped to keep the rest legal)";
                    rows.push_back( row );
                    return;
                }

                covered++;
                vias += (int) c.vias.size();
                std::set<std::string> on;

                for( const GRID_CORRIDOR::RUN& run : c.runs )
                    on.insert( str( board->GetLayerName( run.layer ) ) );

                nlohmann::json viaSites = nlohmann::json::array();

                for( const VECTOR2I& v : c.vias )
                    viaSites.push_back( { toMm( v.x ), toMm( v.y ) } );

                row["corridor"] = corridorOfWay( aNet, aName, c, cornerEnd( aFrom, aFrom->GetPosition() ), cornerEnd( aTo, aToPos ) );
                row["vias_mm"] = viaSites;
                row["layers"] = on;
                row["length_mm"] = std::round( toMm( c.length ) * 100 ) / 100;
                rows.push_back( row );
            };

            for( size_t k = 0; k < gridNets.size(); ++k )
            {
                for( size_t index : jobsOfNet[k] )
                    corridorRow( k, jobs[index].from, jobs[index].to, jobs[index].toPos, jobs[index].net );
            }

            for( const GRID_GAP& gg : gridGaps )
            {
                NETINFO_ITEM* info = board->FindNet( gridNets[gg.net].netCode );
                corridorRow( gg.net, gridNets[gg.net].ends[gg.a], gridNets[gg.net].ends[gg.b], gridNets[gg.net].ends[gg.b]->GetPosition(),
                             info ? str( UnescapeString( info->GetNetname() ) ) : std::string() );
            }

            nlohmann::json variantStats = nlohmann::json::array();

            for( const GRID_RESULT::VARIANT_STATS& v : gr.variantStats )
            {
                variantStats.push_back( { { "variant", v.name }, { "ms", std::round( v.ms ) }, { "reached", v.reached },
                                          { "vias", v.vias }, { "abandoned", v.abandoned } } );
            }

            return KOPENAPI_RESULT::Ok( { { "stage", "global" }, { "applied", false }, { "connections", connections },
                                          { "covered", covered }, { "vias", vias }, { "results", rows },
                                          { "grid", { { "variant", gr.variant }, { "cell_mm", toMm( gr.cell ) },
                                                      { "rounds", gr.rounds }, { "crowded_by_round", gr.crowdedByRound },
                                                      { "dropped", gr.conflictsLeft }, { "expanded", gr.expanded },
                                                      { "build_ms", std::round( gr.buildMs ) }, { "route_ms", std::round( gr.routeMs ) },
                                                      { "variant_stats", variantStats } } },
                                          { "ms", std::round( msSince( started ) ) } } );
        }

        // the detail stage: the grid's copper of each net cut into branches between its junctions (its
        // ends, points where three ways meet), each branch by pns along its corridor (the lane bus_paths
        // would lay along it: its corners and via gates as waypoints); bundles first, then the branches
        // between two ends, then those onto a junction, shorter first. A branch that does not route so
        // tries its via gates alone, then the grid's copper of it goes through the guard below
        std::vector<int>         detailOf( jobs.size(), 0 );   // 1 joined by pns, 2 left to the grid's copper
        std::vector<std::string> detailWhy( jobs.size() );
        nlohmann::json           detailOut;

        if( detailStage )
        {
            struct DETAIL
            {
                size_t      net;
                GRID_BRANCH branch;
            };

            std::vector<DETAIL> order;

            for( size_t k = 0; k < gridNets.size() && k < gr.nets.size(); ++k )
            {
                for( GRID_BRANCH& branch : GridBranches( gridNets[k], gr.nets[k], gr.cell ) )
                    order.push_back( { k, std::move( branch ) } );
            }

            std::stable_sort( order.begin(), order.end(),
                              [&]( const DETAIL& x, const DETAIL& y )
                              {
                                  const GRID_NET& nx = gridNets[x.net];
                                  const GRID_NET& ny = gridNets[y.net];

                                  if( ( nx.group >= 0 ) != ( ny.group >= 0 ) )
                                      return nx.group >= 0;

                                  if( nx.group != ny.group )
                                      return nx.group < ny.group;

                                  if( nx.rank != ny.rank )
                                      return nx.rank < ny.rank;

                                  if( ( x.branch.a >= 0 ) != ( y.branch.a >= 0 ) )
                                      return x.branch.a >= 0;

                                  if( ( x.branch.b >= 0 ) != ( y.branch.b >= 0 ) )
                                      return x.branch.b >= 0;

                                  return x.branch.way.length < y.branch.way.length;
                              } );

            // the net's copper nearest a junction point, in the net's own space round it (its share of the free
            // space, where the rooms its copper was laid in lie), else within a grid cell and the copper's width
            // of it (the junction is a cell's centre on the grid's way): what a branch onto it ends on
            auto copperAt = [&]( size_t aNet, const VECTOR2I& aAt, PCB_LAYER_ID aLayer, VECTOR2I& aPos ) -> BOARD_CONNECTED_ITEM*
            {
                const int                  aNetCode = gridNets[aNet].netCode;
                const std::optional<BOX2I> space = rooms.SpaceAt( (int) aNet, aLayer, aAt );
                BOARD_CONNECTED_ITEM*      best = nullptr;
                int64_t                    bestD = std::numeric_limits<int64_t>::max();

                for( PCB_TRACK* track : board->Tracks() )
                {
                    if( track->GetNetCode() != aNetCode || ( track->Type() != PCB_VIA_T && track->GetLayer() != aLayer ) )
                        continue;

                    const VECTOR2I at = track->Type() == PCB_VIA_T ? track->GetPosition()
                                                                   : SEG( track->GetStart(), track->GetEnd() ).NearestPoint( aAt );
                    const int64_t  d = ( at - aAt ).EuclideanNorm();

                    if( d < bestD && ( d <= gr.cell + track->GetWidth() || ( space && space->Contains( at ) ) ) )
                    {
                        bestD = d;
                        best = track;
                        aPos = at;
                    }
                }

                return best;
            };

            int    byCorridor = 0, byViaSites = 0, toGrid = 0, branchesJoined = 0, pnsVias = 0;
            double pnsLength = 0;
            std::vector<GRID_NET_RESULT> left( gr.nets.size() );
            std::vector<std::string>     netWhy( gridNets.size() );

            for( size_t k = 0; k < left.size(); ++k )
            {
                left[k].walls = gr.nets[k].walls;
                left[k].halfCellExits = gr.nets[k].halfCellExits;
                left[k].reached.assign( gridNets[k].ends.size(), gr.nets[k].complete );
            }

            auto toLeft = [&]( const DETAIL& d, const std::string& aWhy )
            {
                toGrid++;

                if( netWhy[d.net].empty() )
                    netWhy[d.net] = aWhy;

                GRID_NET_RESULT& r = left[d.net];

                for( const GRID_CORRIDOR::RUN& run : d.branch.way.runs )
                {
                    if( run.points.size() > 1 )
                        r.tracks.push_back( { run.layer, run.points } );
                }

                r.vias.insert( r.vias.end(), d.branch.way.vias.begin(), d.branch.way.vias.end() );
                r.length += d.branch.way.length;
            };

            for( size_t step = 0; step < order.size(); ++step )
            {
                const DETAIL&   d = order[step];
                const GRID_NET& n = gridNets[d.net];

                if( !KopenapiJobStep( "detail", (int) step, (int) order.size(), std::to_string( n.netCode ), progressCounts( "detail" ) ) )
                    break;

                if( d.branch.way.runs.empty() || d.branch.way.runs.front().points.empty() )
                    continue;

                // between two junctions: from the copper laid at the first one
                VECTOR2I              fromPos;
                BOARD_CONNECTED_ITEM* from = d.branch.a >= 0 ? n.ends[d.branch.a]
                                                             : copperAt( d.net, d.branch.way.runs.front().points.front(),
                                                                         d.branch.way.runs.front().layer, fromPos );

                if( !from )
                {
                    toLeft( d, "a branch between two junctions of its copper, no copper at the first yet" );
                    continue;
                }

                BOARD_CONNECTED_ITEM* to = nullptr;
                VECTOR2I              toPos;
                GRID_CORRIDOR         way = d.branch.way;
                int                   b = d.branch.b;

                // onto a junction no copper has reached yet: on through it to the end of another branch
                // there
                if( b < 0 && !copperAt( d.net, d.branch.bAt, d.branch.bLayer, toPos ) )
                {
                    for( size_t o = step + 1; o < order.size(); ++o )
                    {
                        const GRID_BRANCH& other = order[o].branch;

                        if( order[o].net != d.net || other.b >= 0 || other.a < 0 || other.a == d.branch.a
                            || other.bAt != d.branch.bAt || other.bLayer != d.branch.bLayer )
                        {
                            continue;
                        }

                        std::vector<GRID_CORRIDOR::RUN> back( other.way.runs.rbegin(), other.way.runs.rend() );

                        for( GRID_CORRIDOR::RUN& run : back )
                            std::reverse( run.points.begin(), run.points.end() );

                        if( !back.empty() && !way.runs.empty() && back.front().layer == way.runs.back().layer )
                        {
                            way.runs.back().points.insert( way.runs.back().points.end(), back.front().points.begin() + 1,
                                                           back.front().points.end() );
                            back.erase( back.begin() );
                        }

                        way.runs.insert( way.runs.end(), back.begin(), back.end() );
                        way.vias.insert( way.vias.end(), other.way.vias.rbegin(), other.way.vias.rend() );
                        way.length += other.way.length;
                        b = other.a;
                        break;
                    }
                }

                if( b >= 0 )
                {
                    to = n.ends[b];
                    toPos = to->Type() == PCB_PAD_T || to->Type() == PCB_VIA_T ? to->GetPosition() : way.runs.back().points.back();

                    // onto a track the way stops just short of or past its end at: that end (a
                    // joint inside the track leaves its end over, a sliver the sweep takes out)
                    if( PCB_TRACK* line = dynamic_cast<PCB_TRACK*>( to ); line && line->Type() == PCB_TRACE_T )
                    {
                        for( const VECTOR2I& end : { line->GetStart(), line->GetEnd() } )
                        {
                            if( ( end - toPos ).EuclideanNorm() <= line->GetWidth() )
                                toPos = end;
                        }
                    }

                    if( from->GetNetCode() > 0 )
                    {
                        const auto joined = board->GetConnectivity()->GetConnectedItems( from );

                        if( std::find( joined.begin(), joined.end(), to ) != joined.end() )
                        {
                            branchesJoined++;
                            continue;
                        }
                    }
                }
                else
                {
                    to = copperAt( d.net, d.branch.bAt, d.branch.bLayer, toPos );

                    if( !to )
                    {
                        toLeft( d, "its junction has no copper of the net near it" );
                        continue;
                    }

                    const auto joined = board->GetConnectivity()->GetConnectedItems( from );

                    if( std::find( joined.begin(), joined.end(), to ) != joined.end() )
                    {
                        branchesJoined++;
                        continue;
                    }
                }

                const VECTOR2I start = from->Type() == PCB_PAD_T || from->Type() == PCB_VIA_T ? from->GetPosition()
                                       : d.branch.a < 0                                         ? fromPos
                                                                                                : way.runs.front().points.front();
                const bool     viaAtEnd = !way.vias.empty() && ( way.vias.front() == start || way.vias.back() == toPos );
                bool           ok = false;
                nlohmann::json attempt = nlohmann::json::object();
                std::string    why;

                // its corridor, read as bus_paths reads one: the corners and via gates its lane keeps
                NETINFO_ITEM*       info = board->FindNet( n.netCode );
                const std::string   netName = info ? str( UnescapeString( info->GetNetname() ) ) : std::string();
                const nlohmann::json corridor = corridorOfWay( d.net, netName, way, cornerEnd( from, start ), cornerEnd( to, toPos ) );
                CORRIDOR            c;
                BOX2I               around = BOX2I::ByCorners( start, toPos );

                c.name = netName;
                c.nets = { netName };
                c.from = corridor["from"];
                c.to = corridor["to"];

                for( const nlohmann::json& gate : corridor["points_mm"] )
                {
                    CROSSING x;

                    if( !crossingOf( gate, x ) )
                        continue;

                    around.Merge( x.at );

                    for( const BOX2I& room : x.rooms )
                        around.Merge( room );

                    c.points.push_back( x );
                }

                // what can touch the lane: copper within a track and the board's largest clearance of its rooms
                {
                    const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
                    NETCLASS*                    nc = from->GetEffectiveNetClass();
                    const int width = nc && nc->HasTrackWidth() ? nc->GetTrackWidth() : ds.GetCurrentTrackWidth();
                    around.Inflate( width + std::max( { ds.GetBiggestClearanceValue(), ds.m_MinClearance, 1 } ) );
                }

                std::vector<MEMBER> lane = { { 0, true, start, toPos, 0, from } };
                const LANES         lanes = c.points.empty() ? LANES() : laneAlong( c, { n.netCode }, lane, around );

                for( int widen = 0; widen < 2 && !ok && way.vias.size() + 1 == way.runs.size() && !viaAtEnd; ++widen )
                {
                    PAD*          pad = dynamic_cast<PAD*>( from );
                    ROUTE_REQUEST req{ from, to, toPos, way.runs.front().layer, PNS::RM_Walkaround, false, {} };

                    // the grid planned every net's way out: the ways kept open matter only off the corridor
                    req.fronts = widen == 0 ? nullptr : &fronts;
                    req.neckWidth = pad ? neckFor( pad, to ) : 0;
                    req.pairTail = pairTail( to );

                    for( size_t v = 0; !lanes.corners.empty() && v < lanes.corners[0].size(); ++v )
                    {
                        if( widen == 0 || lanes.vias[v] )
                            req.vias.emplace_back( lanes.corners[0][v], lanes.vias[v] ? std::make_optional( lanes.vias[v]->second ) : std::nullopt );
                    }

                    req.planned = true;

                    if( !pad )
                        req.fromPos = start;

                    setDeadline( branchBudgetS );
                    attempt = nlohmann::json::object();
                    ok = routeOne( *context, req, attempt, added );
                    tryDeadline.reset();

                    if( !ok )
                        why += ( why.empty() ? "" : "; " ) + std::string( widen == 0 ? "along the corridor: " : "by its via sites: " )
                               + attempt.value( "reason", std::string() );

                    if( ok )
                    {
                        ( widen == 0 ? byCorridor : byViaSites )++;
                        const nlohmann::json metrics = attempt.value( "metrics", nlohmann::json::object() );
                        pnsLength += metrics.value( "length_mm", 0.0 );
                        pnsVias += metrics.value( "vias", 0 );
                    }
                }

                if( !ok )
                {
                    toLeft( d, viaAtEnd                                                    ? "its corridor changes layer where an end is"
                               : way.vias.size() + 1 != way.runs.size() ? "its corridor changes layer through a pad"
                               : why.empty() ? std::string( "the corridor's way does not route" ) : why );
                }
            }

            // the connections pns joined; the rest go to the grid's copper of the branches it left
            board->BuildConnectivity();

            for( size_t k = 0; k < gridNets.size(); ++k )
            {
                for( size_t index : jobsOfNet[k] )
                {
                    JOB& job = jobs[index];
                    job.from = dynamic_cast<PAD*>( board->ResolveItem( job.fromId, true ) );
                    job.to = dynamic_cast<BOARD_CONNECTED_ITEM*>( board->ResolveItem( job.toId, true ) );

                    if( !job.to )
                        pairEndAgain( job );

                    const auto joined = job.from ? board->GetConnectivity()->GetConnectedItems( job.from )
                                                 : std::vector<BOARD_CONNECTED_ITEM*>();

                    if( !job.to || std::find( joined.begin(), joined.end(), job.to ) == joined.end() )
                    {
                        detailOf[index] = 2;
                        detailWhy[index] = netWhy[k].empty() ? std::string( "pns did not join it" ) : netWhy[k];
                        continue;
                    }

                    detailOf[index] = 1;
                    done[index] = true;
                    rowOf[index] = (int) results.size();
                    results.push_back( { { "net", job.net }, { "from", endName( job.from ) }, { "to", endName( job.to ) },
                                         { "routed", true }, { "tries", 1 }, { "engine", "pns" }, { "detail", "branches" } } );
                }
            }

            gr.nets = std::move( left );
            detailOut = { { "branches", order.size() }, { "by_corridor", byCorridor }, { "by_via_sites", byViaSites },
                          { "already_joined", branchesJoined }, { "to_grid_copper", toGrid },
                          { "pns_length_mm", std::round( pnsLength * 100 ) / 100 }, { "pns_vias", pnsVias } };
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
                    if( !end || end->Type() != PCB_PAD_T || !finePad( static_cast<PAD*>( end ) ) )
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

        // the detail stage: what the grid's copper joined, read from the board
        if( detailStage )
            board->BuildConnectivity();

        auto joinedNow = [&]( size_t aIndex )
        {
            const JOB& job = jobs[aIndex];

            if( !job.from || !job.to )
                return false;

            const auto joined = board->GetConnectivity()->GetConnectedItems( job.from );
            return std::find( joined.begin(), joined.end(), job.to ) != joined.end();
        };

        for( size_t k = 0; k < gridNets.size(); ++k )
        {
            const GRID_NET_RESULT* r = k < gr.nets.size() ? &gr.nets[k] : nullptr;
            bool                   first = true;

            for( size_t index : jobsOfNet[k] )
            {
                if( detailOf[index] == 1 )
                    continue;

                const auto [a, b] = endsOfJob[index];
                const bool reached = r && a < r->reached.size() && b < r->reached.size() && r->reached[a] && r->reached[b];
                const bool ok = detailStage ? netOk[k] && joinedNow( index ) : netOk[k] && reached;
                nlohmann::json row = { { "net", jobs[index].net }, { "from", endName( jobs[index].from ) }, { "to", endName( jobs[index].to ) },
                                       { "routed", ok }, { "tries", 1 }, { "engine", "grid" } };

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
                            if( !end || end->Type() != PCB_PAD_T || !finePad( static_cast<PAD*>( end ) ) )
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

                if( !detailWhy[index].empty() )
                    row["pns_detail"] = detailWhy[index];

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
            bool                   ok = reached && netOk[gg.net];

            if( detailStage )
            {
                BOARD_CONNECTED_ITEM* a = gridNets[gg.net].ends[gg.a];
                BOARD_CONNECTED_ITEM* b = gridNets[gg.net].ends[gg.b];
                const auto            joined = board->GetConnectivity()->GetConnectedItems( a );
                ok = std::find( joined.begin(), joined.end(), b ) != joined.end();
            }
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
                    { "dropped", gr.conflictsLeft }, { "crowded_left_out", gr.crowdedLeftOut }, { "cell_mm", toMm( gr.cell ) }, { "expanded", gr.expanded },
                    { "build_ms", std::round( gr.buildMs ) }, { "route_ms", std::round( gr.routeMs ) },
                    { "half_cell", { { "tried", gr.halfCellTried }, { "won", gr.halfCellWon }, { "ms", std::round( gr.halfCellMs * 10 ) / 10 } } },
                    { "guard_ms", std::round( msSince( guardFrom ) ) }, { "rejected", rejected },
                    { "costliest", costliest }, { "expanded_by_stage", gr.expandedByStage }, { "bundles", bundlesUsed },
                    { "timeline", timelineOut } };

        if( detailStage )
            gridOut["detail"] = detailOut;

        board->BuildConnectivity();
    }

    // Rip-up and retry: for a connection that failed, take out the routes of this call that cross
    // its corridor, route it, route them again; if anything of that fails, everything goes back
    // as it was (walkaround only: no other net is pushed aside; the copper of the nets it touched
    // is put back from copies)
    const int ripRounds = dryRun ? 0 : std::clamp( aArgs.value( "rip_up", 2 ), 0, 6 );
    int       rippedOk = 0, rippedTried = 0;

    budgetName = "rip_up_budget_s";

    // rip_up_scope unlocked: the unlocked copper of other nets (earlier calls, laid by hand) near the
    // connections left may be taken out too; each piece between two pads of its net (a spanning tree
    // over the pads it joins) is a route that goes back when the rip-up is over, or all goes back
    const std::string ripScope = aArgs.value( "rip_up_scope", std::string( "call" ) );

    if( ripScope != "call" && ripScope != "unlocked" )
        return KOPENAPI_RESULT::Error( 400, "rip_up_scope: call (only this call's routes) or unlocked (any unlocked copper)" );

    size_t         foreignJobs = 0;
    nlohmann::json foreignMoved = nlohmann::json::array();

    if( ripScope == "unlocked" && ripRounds > 0 && !grid )
    {
        BOX2I           leftArea;
        bool            anyLeft = false;
        std::set<int>   own;

        for( size_t k = 0; k < jobs.size(); ++k )
        {
            if( PAD* from = dynamic_cast<PAD*>( board->ResolveItem( jobs[k].fromId, true ) ) )
                own.insert( from->GetNetCode() );

            if( done[k] || rowOf[k] < 0 || jobs[k].trunk )
                continue;

            BOX2I box( jobs[k].from->GetPosition(), VECTOR2I( 0, 0 ) );
            box.Merge( jobs[k].toPos );
            leftArea = anyLeft ? leftArea.Merge( box ) : box;
            anyLeft = true;
        }

        for( const auto& [p, n] : explicitPairs )
            own.insert( p ), own.insert( n );

        leftArea.Inflate( pcbIUScale.mmToIU( 5.0 ) );
        std::map<int, std::vector<PAD*>> padsOfNet;

        if( anyLeft )
        {
            std::set<int> closeNets;

            for( PCB_TRACK* t : board->Tracks() )
            {
                if( t->GetNetCode() > 0 && !own.count( t->GetNetCode() ) && t->GetBoundingBox().Intersects( leftArea ) )
                    closeNets.insert( t->GetNetCode() );
            }

            for( int code : closeNets )
            {
                // a pair's net is not taken apart from its partner
                if( coupledNet( board, board->FindNet( code ), &explicitPairs ) )
                    continue;

                padsOfNet[code];
            }

            for( FOOTPRINT* fp : board->Footprints() )
            {
                for( PAD* pad : fp->Pads() )
                {
                    if( auto it = padsOfNet.find( pad->GetNetCode() ); it != padsOfNet.end() )
                        it->second.push_back( pad );
                }
            }
        }

        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();

        for( auto& [code, pads] : padsOfNet )
        {
            // the pads its copper joins, a spanning tree (shortest first) over each joined group
            std::vector<int> group( pads.size(), -1 );

            for( size_t i = 0; i < pads.size(); ++i )
            {
                if( group[i] >= 0 )
                    continue;

                group[i] = (int) i;
                const auto joined = conn->GetConnectedItems( pads[i] );

                for( size_t j = i + 1; j < pads.size(); ++j )
                {
                    if( group[j] < 0 && std::find( joined.begin(), joined.end(), pads[j] ) != joined.end() )
                        group[j] = (int) i;
                }
            }

            std::vector<bool> inTree( pads.size(), false );

            for( size_t root = 0; root < pads.size(); ++root )
            {
                if( group[root] != (int) root )
                    continue;

                inTree[root] = true;

                for( ;; )
                {
                    double best = std::numeric_limits<double>::max();
                    size_t a = 0, b = 0;

                    for( size_t i = 0; i < pads.size(); ++i )
                    {
                        if( !inTree[i] || group[i] != (int) root )
                            continue;

                        for( size_t j = 0; j < pads.size(); ++j )
                        {
                            if( inTree[j] || group[j] != (int) root )
                                continue;

                            const double d = ( pads[i]->GetPosition() - pads[j]->GetPosition() ).EuclideanNorm();

                            if( d < best )
                                best = d, a = i, b = j;
                        }
                    }

                    if( best == std::numeric_limits<double>::max() )
                        break;

                    inTree[b] = true;
                    const std::vector<PCB_TRACK*> chain = copperBetween( board, pads[a], pads[b] );

                    if( chain.empty() || std::any_of( chain.begin(), chain.end(), []( PCB_TRACK* t ) { return t->IsLocked(); } ) )
                        continue;

                    JOB j{ pads[a], pads[b], pads[a]->m_Uuid, pads[b]->m_Uuid, pads[b]->GetPosition(), best,
                           str( UnescapeString( pads[a]->GetNetname() ) ), false };
                    j.foreign = true;
                    jobs.push_back( j );
                    foreignJobs++;
                }
            }
        }

        done.resize( jobs.size(), true );
        rowOf.resize( jobs.size(), -1 );
        partner.resize( jobs.size(), -1 );
        overFirstBudget.resize( jobs.size(), false );
    }

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

        // the connections with fewest tries first: one that failed fast is not left without a turn
        // behind the slow ones; in the first round the plateau waits until each has had its turn
        std::vector<size_t> order;

        for( size_t index = 0; index < jobs.size(); ++index )
        {
            if( !done[index] && rowOf[index] >= 0 && !jobs[index].trunk )
                order.push_back( index );
        }

        std::stable_sort( order.begin(), order.end(),
                          [&]( size_t a, size_t b )
                          { return results[rowOf[a]].value( "tries", 0 ) < results[rowOf[b]].value( "tries", 0 ); } );

        if( round == 0 )
            plateauHeld = order.size();

        struct TURN
        {
            size_t& held;
            bool    first;
            ~TURN()
            {
                if( first && held > 0 )
                    held--;
            }
        };

        for( size_t index : order )
        {
            TURN turn{ plateauHeld, round == 0 };

            if( done[index] )
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

            plateauTurn();

            if( ripBudgetS > 0 )
                setDeadline( ripBudgetS );

            // first from both ends, nothing taken out
            if( round == 0 )
            {
                nlohmann::json attempt;
                int            tries = 0;

                profile.pass = "retry";

                // until a rip-up has moved routes, the board has only gained copper since the first
                // pass: the forward way failed there already
                if( tryRoute( job, attempt, tries, false, true, rippedOk > 0 || overFirstBudget[index] ) )
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
                    row.erase( "budget" );
                    row["retried"] = true;
                    row["layer"] = attempt["layer"];
                    row["mode"] = attempt["mode"];
                    row["length_mm"] = attempt["metrics"]["length_mm"];
                    row["vias"] = attempt["metrics"].value( "vias", 0 );
                    row["detour"] = attempt["metrics"]["detour"];

                    if( attempt.contains( "inward_via_mm" ) )
                        row["inward_via_mm"] = attempt["inward_via_mm"];
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
                        || ( rowOf[other] >= 0 && results[rowOf[other]].value( "joined_by_earlier_route", false ) )
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
                std::vector<nlohmann::json> secondAttempts;

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
                                secondAttempts.push_back( std::move( movedAttempt ) );
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
                    if( !done[k] || k == index || jobs[k].trunk || partner[k] >= 0 || ( rowOf[k] < 0 && !jobs[k].foreign )
                        || std::find( allTaken.begin(), allTaken.end(), k ) != allTaken.end()
                        || ( rowOf[k] >= 0 && !results[rowOf[k]].value( "routed", false ) ) )
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
                    row.erase( "budget" );
                    row.erase( "boxed_in" );
                    row.erase( "pad_exits_tried_mm" );
                    row["after_rip_up"] = true;
                    row["layer"] = attempt["layer"];
                    row["mode"] = attempt["mode"];
                    row["length_mm"] = attempt["metrics"]["length_mm"];
                    row["vias"] = attempt["metrics"].value( "vias", 0 );
                    row["detour"] = attempt["metrics"]["detour"];

                    if( attempt.contains( "inward_via_mm" ) )
                        row["inward_via_mm"] = attempt["inward_via_mm"];

                    // copper of an earlier call laid again: listed, and its net is one this call moved
                    auto earlier = [&]( size_t aJob, const nlohmann::json* aAttempt )
                    {
                        const JOB& j = jobs[aJob];
                        profile.shovedNets.insert( j.from->GetNetCode() );

                        auto name = []( const BOARD_CONNECTED_ITEM* aItem ) -> std::string
                        {
                            const PAD* pad = dynamic_cast<const PAD*>( aItem );

                            return pad && pad->GetParentFootprint()
                                           ? str( pad->GetParentFootprint()->GetReference() ) + "." + str( pad->GetNumber() )
                                           : std::string();
                        };

                        nlohmann::json e = { { "net", j.net }, { "from", name( j.from ) }, { "to", name( j.to ) }, { "for", job.net } };

                        if( aAttempt && aAttempt->contains( "metrics" ) )
                        {
                            e["length_mm"] = ( *aAttempt )["metrics"]["length_mm"];
                            e["vias"] = ( *aAttempt )["metrics"].value( "vias", 0 );
                        }

                        foreignMoved.push_back( std::move( e ) );
                    };

                    for( size_t k = 0; k < victims.size(); ++k )
                    {
                        if( rowOf[victims[k]] < 0 )
                        {
                            earlier( victims[k], &victimAttempts[k] );
                            continue;
                        }

                        nlohmann::json& vrow = results[rowOf[victims[k]]];
                        vrow["rerouted"] = true;
                        vrow["length_mm"] = victimAttempts[k]["metrics"]["length_mm"];
                        vrow["vias"] = victimAttempts[k]["metrics"].value( "vias", 0 );
                        vrow["layer"] = victimAttempts[k]["layer"];
                    }

                    for( size_t k : cut )
                    {
                        if( rowOf[k] < 0 )
                            earlier( k, nullptr );
                        else
                            results[rowOf[k]]["rerouted"] = true;
                    }

                    for( size_t k = 0; k < secondTaken.size(); ++k )
                    {
                        if( rowOf[secondTaken[k]] < 0 )
                        {
                            earlier( secondTaken[k], &secondAttempts[k] );
                            continue;
                        }

                        nlohmann::json& vrow = results[rowOf[secondTaken[k]]];
                        vrow["rerouted"] = true;
                        vrow["length_mm"] = secondAttempts[k]["metrics"]["length_mm"];
                        vrow["vias"] = secondAttempts[k]["metrics"].value( "vias", 0 );
                        vrow["layer"] = secondAttempts[k]["layer"];
                    }

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

        plateauHeld = 0;
        plateauTurnCut = false;

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

        if( ripBudgetS > 0 )
            setDeadline( ripBudgetS );

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

    tryDeadline.reset();

    // Clean-up: a routed connection with vias or a detour is routed again alone on a layer both
    // ends are on (walkaround: it moves nothing else); kept when it has fewer vias, or as many and
    // is 5 % shorter, and no connection of its net is left cut; else put back as it was.  A long
    // detour (over 1.6 x the straight line) follows a plan through vias instead when that plan is a
    // quarter shorter (a via counted as 2 mm of track); kept when it is a fifth shorter so counted
    int cleanTried = 0, cleanSkipped = 0, cleanKept = 0, viasRemoved = 0, viasAdded = 0, viaWays = 0;
    double shortenedIU = 0;
    const double viaCost = pcbIUScale.mmToIU( 2.0 );

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
            if( !done[k] || jobs[k].trunk || jobs[k].busVia || jobs[k].corridorRouted || jobs[k].laneRouted || partner[k] >= 0 || rowOf[k] < 0 || jobs[k].to->Type() != PCB_PAD_T
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

            if( chain.empty() )
                continue;

            const double direct = ( job.toPos - job.from->GetPosition() ).EuclideanNorm();
            ROUTE_PLAN   viaPlan;

            if( planning && viasAllowed && c.length > 1.6 * direct )
            {
                ROUTE_PLAN plan = makePlan( planRequest( job.from, job.to, job.toPos ) );

                if( plan.found && plan.vias > 0
                    && plan.length + viaCost * plan.vias < 0.75 * ( c.length + viaCost * c.vias ) )
                {
                    viaPlan = std::move( plan );
                }
            }

            if( !viaPlan.found && shared.empty() )
                continue;

            // the free space must hold a way on those layers that could win (the net's own copper
            // is no obstacle to the plan, so the chain stays until then)
            if( planning && !viaPlan.found )
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

            if( viaPlan.found )
            {
                ROUTE_REQUEST req{ job.from, job.to, job.toPos, viaPlan.startLayer, PNS::RM_Walkaround, false, {} };
                req.fronts = &fronts;
                req.neckWidth = neckFor( job.from, job.to );
                req.pairTail = pairTail( job.to );
                req.vias = viaPlan.points;
                req.planned = true;
                ok = routeOne( *context, req, attempt, added );
            }

            for( PCB_LAYER_ID layer : viaPlan.found ? std::vector<PCB_LAYER_ID>() : shared )
            {
                for( bool reverse : { false, true } )
                {
                    if( ok )
                        break;

                    BOARD_CONNECTED_ITEM* from = job.from;
                    BOARD_CONNECTED_ITEM* to = job.to;
                    VECTOR2I              toPos = job.toPos;

                    if( reverse )
                    {
                        from = job.to;
                        to = job.from;
                        toPos = job.from->GetPosition();
                    }

                    ROUTE_REQUEST req{ from, to, toPos, layer, PNS::RM_Walkaround, false, {} };
                    req.fronts = &fronts;
                    req.neckWidth = neckFor( from, to );
                    req.pairTail = pairTail( to );
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

                ok = viaPlan.found ? length + viaCost * vias < 0.8 * ( c.length + viaCost * c.vias )
                                   : vias < c.vias || ( vias == c.vias && length < 0.95 * c.length );
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
                viasRemoved += std::max( c.vias - vias, 0 );
                viasAdded += std::max( vias - c.vias, 0 );
                viaWays += viaPlan.found;
                shortenedIU += c.length - length;

                nlohmann::json& row = results[rowOf[c.index]];
                row["cleaned"] = true;
                row["layer"] = attempt["layer"];
                row["mode"] = attempt["mode"];
                row["length_mm"] = attempt["metrics"]["length_mm"];
                row["vias"] = vias;
                row["detour"] = attempt["metrics"]["detour"];

                if( attempt.contains( "inward_via_mm" ) )
                    row["inward_via_mm"] = attempt["inward_via_mm"];
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
        if( job.foreign )
            continue;

        callNets.insert( job.from->GetNetCode() );

        if( NETINFO_ITEM* coupled = job.trunk ? coupledNet( board, job.from->GetNet(), &explicitPairs ) : nullptr )
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

    const int connections = (int) ( jobs.size() - trunks.size() + gaps.size() - foreignJobs );

    if( stepMs == 0 && !aQuiet )
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

    // each pair's two nets as laid: their lengths (as DRC's skew test takes them: tracks, vias through
    // the stackup, pad to die) and skew, against the board's skew rule for them
    nlohmann::json pairsOut = nlohmann::json::array();

    for( const auto& [codeP, codeN] : explicitPairs )
    {
        double     lengthOf[2] = { 0.0, 0.0 };
        PCB_TRACK* anyOf[2] = { nullptr, nullptr };

        for( int side : { 0, 1 } )
        {
            const int                                  code = side ? codeN : codeP;
            std::vector<LENGTH_DELAY_CALCULATION_ITEM> items;
            LENGTH_DELAY_CALCULATION*                  calc = board->GetLengthCalculation();

            for( BOARD_CONNECTED_ITEM* item : board->GetConnectivity()->GetNetItems( code, { PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_PAD_T } ) )
            {
                if( PCB_TRACK* t = dynamic_cast<PCB_TRACK*>( item ); t && t->Type() != PCB_VIA_T && !anyOf[side] )
                    anyOf[side] = t;

                if( LENGTH_DELAY_CALCULATION_ITEM it = calc->GetLengthCalculationItem( item );
                    it.Type() != LENGTH_DELAY_CALCULATION_ITEM::TYPE::UNKNOWN )
                {
                    items.emplace_back( it );
                }
            }

            constexpr PATH_OPTIMISATIONS opts = { .OptimiseVias = true, .MergeTracks = true,
                                                  .OptimiseTracesInPads = true, .InferViaInPad = false };
            const LENGTH_DELAY_STATS     stats = calc->CalculateLengthDetails( items, opts, nullptr, nullptr,
                                                                               LENGTH_DELAY_LAYER_OPT::NO_LAYER_DETAIL,
                                                                               LENGTH_DELAY_DOMAIN_OPT::NO_DELAY_DETAIL );
            lengthOf[side] = (double) stats.TrackLength + stats.ViaLength + stats.PadToDieLength;
        }

        const double   skew = std::abs( lengthOf[0] - lengthOf[1] );
        nlohmann::json row = { { "p", str( board->FindNet( codeP )->GetNetname() ) },
                               { "n", str( board->FindNet( codeN )->GetNetname() ) },
                               { "length_mm", { toMm( lengthOf[0] ), toMm( lengthOf[1] ) } },
                               { "skew_mm", toMm( skew ) } };

        if( std::shared_ptr<DRC_ENGINE> drc = board->GetDesignSettings().m_DRCEngine; drc && anyOf[0] )
        {
            const DRC_CONSTRAINT c = drc->EvalRules( SKEW_CONSTRAINT, anyOf[0], nullptr, anyOf[0]->GetLayer() );

            if( c.GetParentRule() && !c.GetParentRule()->IsImplicit() && c.GetValue().HasMax() )
            {
                row["skew_max_mm"] = toMm( c.GetValue().Max() );
                row["skew_rule"] = str( c.GetParentRule()->m_Name );
                row["skew_ok"] = skew <= c.GetValue().Max();
            }
        }

        pairsOut.push_back( row );
    }

    nlohmann::json answer = { { "connections", connections },
                              { "routed", routed },
                              { "failed", connections - routed },
                              { "length_mm", std::round( length * 1000.0 ) / 1000.0 },
                              { "vias", vias },
                              { "bus", busOut },
                              { "bus_paths", { { "corridors", corridorOut }, { "members", corridorMembers },
                                               { "routed_along", corridorRoutedCount } } },
                              { "rip_up", { { "tried", rippedTried }, { "won", rippedOk }, { "scope", ripScope },
                                            { "earlier_routes_movable", foreignJobs },
                                            { "earlier_routes_rerouted", foreignMoved } } },
                              { "clean_up", { { "tried", cleanTried }, { "skipped", cleanSkipped }, { "kept", cleanKept }, { "vias_removed", viasRemoved }, { "via_ways", viaWays }, { "vias_added", viasAdded },
                                              { "shortened_mm", std::round( pcbIUScale.IUTomm( shortenedIU ) * 1000.0 ) / 1000.0 } } },
                              { "stubs_removed", stubs },
                              { "doubled_removed", twins },
                              { "kept_out_under", keptOut },
                              { "grid", gridOut },
                              { "plane_layers_skipped", planeLayers },
                              { "diff_pairs_skipped", pairsSkipped },
                              { "diff_pair_lengths", pairsOut },
                              { "dry_run", dryRun },
                              { "stopped", KopenapiJobCancelled() },
                              { "stopped_on_plateau", plateauStopped },
                              { "unrouted_left", openAtEnd->connections },
                              { "unrouted_connections", openAtEnd->connections },
                              { "unrouted_nets", openAtEnd->nets },
                              { "zone_fills", openAtEnd->zoneFills },
                              { "ms", std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() },
                              { "profile", { { "attempts", profile.attempts },
                                             { "dead_legs_skipped", profile.deadLegsSkipped },
                                             { "tries_stopped", profile.triesStopped },
                                             { "budget", { { "first_pass_s", firstBudgetS }, { "rip_up_s", ripBudgetS },
                                                           { "mode", budgetMode }, { "work", work },
                                                           { "work_per_s", workPerS }, { "wall_caps", wallCaps },
                                                           { "spent", profile.budgetSpent },
                                                           { "first_pass_left", profile.overFirstBudget } } },
                                             { "clearance_lookups", profile.clearanceLookups },
                                             { "rule_evaluations", profile.ruleEvaluations },
                                             { "via_spots_blocked", profile.viaSpotsBlocked },
                                             { "via_spots_off_board", profile.viaSpotsOffBoard },
                                             { "via_spots_in_escapes", profile.viaSpotsInEscapes },
                                             { "rip_ups_skipped", profile.ripUpsSkipped },
                                             { "rip_ups_planned", profile.ripUpsPlanned },
                                             { "rip_ups_walled", profile.ripUpsWalled },
                                             { "rip_ups_failed_at", profile.ripUpsFailedAt },
                                             { "rip_ups_second", profile.ripUpsSecond },
                                             { "pairs_stuck", profile.pairsStuck },
                                             { "front_retries", { { "tried", profile.frontRetries }, { "won", profile.frontRetriesWon } } },
                                             { "pad_exits", { { "tried", profile.padExitsTried }, { "won", profile.padExitsWon } } },
                                             { "plans", profile.plans },
                                             { "plans_found", profile.plansFound },
                                             { "plans_whole_board", profile.plansWholeBoard }, { "plans_stopped", profile.plansStopped },
                                             { "plan_ms", std::round( profile.planMs ) },
                                             { "sync_ms", std::round( profile.syncMs ) },
                                             { "move_ms", std::round( profile.moveMs ) },
                                             { "route_ms", std::round( profile.totalMs - profile.syncMs ) },
                                             { "by_kind", tallies( profile.byKind ) },
                                             { "failed_by", tallies( profile.byFailure ) },
                                             { "by_pass", tallies( profile.byPass ) } } } };

    if( portRefs )
    {
        int portVias = 0;

        for( const auto& [pad, goal] : portGoals )
            portVias += goal.via.has_value();

        answer["ports"] = { { "packages", portsOut }, { "pins", portGoals.size() }, { "vias", portVias }, { "tries", portTries }, { "won", portWon },
                            { "ms", std::round( portMs ) } };

        if( lanesOn )
            answer["ports"]["lanes"] = { { "laid_tracks", lanesLaid }, { "refused", lanesRefused }, { "taken_out", lanesTakenOut } };
    }

    if( profile.traceAttempt > 0 && profile.traceLast == profile.traceAttempt )
        answer["trace"] = profile.traces.empty() ? nlohmann::json::object() : std::move( profile.traces[0] );
    else if( profile.traceAttempt > 0 )
        answer["traces"] = std::move( profile.traces );

    cap.Put( answer, "other_nets_moved", std::move( moved ) );

    if( !doubled.empty() )
        cap.Put( answer, "doubled", std::move( doubled ) );
    for( size_t k = 0; keepTries && k < jobs.size(); ++k )
    {
        if( rowOf[k] >= 0 && !jobs[k].tried.empty() )
            results[rowOf[k]]["attempts"] = std::move( jobs[k].tried );
    }

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


/// @brief The net a stitch or rail plan is for: the one given, else the one net pcb_net_roles calls
/// ground; aError set (400 / 404) where there is none or more than one
static NETINFO_ITEM* stitchNetOf( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, const nlohmann::json& aArgs, KOPENAPI_RESULT& aError )
{
    if( aArgs.contains( "net" ) )
    {
        const std::string text = aArgs.value( "net", std::string() );
        NETINFO_ITEM*     net = KopenapiFindNet( aBoard, text );

        if( !net )
            aError = KopenapiNoSuchNet( aBoard, text );

        return net;
    }

    std::vector<NETINFO_ITEM*> grounds;

    if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
    {
        KOPENAPI_RESULT res = roles->handler( aCtx, nlohmann::json::object() );
        const nlohmann::json rows = res.status == 200 ? res.body.value( "nets", nlohmann::json::array() ) : nlohmann::json::array();

        for( const nlohmann::json& row : rows )
        {
            if( row.value( "role", std::string() ) != "ground" )
                continue;

            if( NETINFO_ITEM* net = KopenapiFindNet( aBoard, row.value( "net", std::string() ) ) )
                grounds.push_back( net );
        }
    }

    if( grounds.size() == 1 )
        return grounds.front();

    nlohmann::json names = nlohmann::json::array();

    for( NETINFO_ITEM* net : grounds )
        names.push_back( str( UnescapeString( net->GetNetname() ) ) );

    aError = KOPENAPI_RESULT::Error( 400, grounds.empty() ? std::string( "net: none given and pcb_net_roles calls no net ground: give net" )
                                                          : "net: none given and pcb_net_roles calls " + std::to_string( grounds.size() )
                                                                    + " nets ground " + names.dump() + ": give net" );
    return nullptr;
}


static KOPENAPI_RESULT pcbStitch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, bool aQuiet )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*        board = context->GetBoard();

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

    KOPENAPI_RESULT netError;
    NETINFO_ITEM*   net = stitchNetOf( aCtx, board, aArgs, netError );

    if( !net )
        return netError;

    const wxString netName = net->GetNetname();

    // the net's via and track as KiCad's rules size them
    const KOPENAPI_RULE_VIA ruleVia = KopenapiRuleVia( board, net->GetNetCode(), VECTOR2I(), F_Cu );
    const int               classViaDia = ruleVia.dia;
    const int               classViaDrill = ruleVia.drill;
    const int               width = KopenapiRuleTrack( board, net->GetNetCode(), VECTOR2I(), F_Cu ).opt;

    // the via being tried: the rules', or the board's smallest at a fine-pitch row pin where the
    // rules' does not fit inward
    int viaDia = classViaDia;
    int viaDrill = classViaDrill;
    int minViaDia = 0;
    int minViaDrill = 0;

    boardMinVia( board, net, classViaDia, classViaDrill, minViaDia, minViaDrill );

    // under_parts avoid: no via or stub inside a quad package's ring of pads (its exposed pad stays open)
    const std::string underParts = aArgs.value( "under_parts", std::string( "avoid" ) );

    if( underParts != "avoid" && underParts != "allow" )
        return KOPENAPI_RESULT::Error( 400, "under_parts: avoid (no via inside a quad package's ring of pads) or allow" );

    const std::vector<UNDER_PART> underAreas = underParts == "avoid" ? underPartAreas( board ) : std::vector<UNDER_PART>();

    // the ring under a quad package between its pad rows and its exposed pad: with under_parts
    // allow, where its own pads' vias go first; avoid keeps every via out of it
    const std::vector<UNDER_PART> ringAreas = underParts == "avoid" ? underAreas : underPartAreas( board );

    // the part whose body a via at aAt (with its stub from aFrom on aLayer) would sit under, or none;
    // the pads of a package's own rows may go under it (aOwn)
    auto underWhich = [&]( const VECTOR2I& aAt, const std::optional<VECTOR2I>& aFrom, PCB_LAYER_ID aLayer,
                           const FOOTPRINT* aOwn ) -> FOOTPRINT*
    {
        for( const UNDER_PART& under : underAreas )
        {
            if( under.fp == aOwn )
                continue;

            if( under.area.Collide( aAt, viaDia / 2 ) )
                return under.fp;

            if( aFrom && aLayer == under.layer && under.area.Collide( SEG( *aFrom, aAt ), width / 2 ) )
                return under.fp;
        }

        return nullptr;
    };

    // pads: only these ends (REF.PAD of the net), and the islands of tracks they are on
    std::set<const PAD*> only;

    if( aArgs.contains( "pads" ) )
    {
        if( !aArgs["pads"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "pads: a list of REF.PAD of the net, e.g. [\"U18.68\"]" );

        for( const nlohmann::json& name : aArgs["pads"] )
        {
            const std::string text = name.is_string() ? name.get<std::string>() : std::string();
            const size_t      dot = text.rfind( '.' );
            FOOTPRINT*        fp = dot == std::string::npos ? nullptr
                                                            : board->FindFootprintByReference( wxString::FromUTF8( text.substr( 0, dot ) ) );
            PAD*              pad = nullptr;

            if( fp )
            {
                for( PAD* p : fp->Pads() )
                {
                    if( str( p->GetNumber() ) == text.substr( dot + 1 ) )
                        pad = p;
                }
            }

            if( !pad )
                return KOPENAPI_RESULT::Error( 400, "pads: no pad " + ( text.empty() ? name.dump() : text ) + " (REF.PAD)" );

            if( pad->GetNetCode() != net->GetNetCode() )
            {
                return KOPENAPI_RESULT::Error( 400, "pads: " + text + " is on " + str( pad->GetNetname() ) + ", not on "
                                                            + str( net->GetNetname() ) );
            }

            only.insert( pad );
        }
    }

    // escape_keep_mm: the straight way out of a fine-pitch pin (KopenapiPadFinePitch) and of a
    // pair's or clock's pin stays clear of stitching vias and stubs for that length; by default past its
    // part's courtyard by one via pitch (the rules' via and clearance), where its first via can go
    const std::optional<double> escapeKeepGiven = aArgs.contains( "escape_keep_mm" ) ? std::optional<double>( std::clamp(
                                                          aArgs.value( "escape_keep_mm", 0.0 ), 0.0, 20.0 ) )
                                                                                     : std::nullopt;
    const bool                  escapeKeepOn = !escapeKeepGiven || *escapeKeepGiven > 0;
    const int    minClearance = board->GetDesignSettings().m_MinClearance;

    struct ESCAPE
    {
        const PAD*    pad;
        PCB_LAYER_ID  layer;
        SHAPE_SEGMENT shape;
    };

    std::vector<ESCAPE>              escapes;
    std::set<const PAD*>             finePads;
    std::map<const PAD*, VECTOR2I>   outward;
    nlohmann::json                   pairLanes = nlohmann::json::array();       ///< pin, to
    nlohmann::json                   pairLanesNone = nlohmann::json::array();   ///< pin, why

    {
        std::set<int> special;
        std::set<int> pairNets;

        if( escapeKeepOn )
        {
            if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
            {
                const KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

                if( r.status < 400 && r.body.contains( "nets" ) && r.body["nets"].is_array() )
                {
                    for( const nlohmann::json& row : r.body["nets"] )
                    {
                        const std::string role = row.value( "role", std::string() );

                        if( role != "pair" && role != "clock" )
                            continue;

                        if( NETINFO_ITEM* info = KopenapiFindNet( board, row.value( "net", std::string() ) ) )
                        {
                            special.insert( info->GetNetCode() );

                            if( role == "pair" )
                                pairNets.insert( info->GetNetCode() );
                        }
                    }
                }
            }
        }

        // a pair's lane runs to its partner pad in the pin's own block (pcb_blocks: the part and its
        // members); beyond it the way is the router's
        std::map<std::string, int> blockOfRef;

        if( !pairNets.empty() )
        {
            if( std::optional<KOPENAPI_METHOD> blocksMethod = KOPENAPI_REGISTRY::Get().Find( "pcb_blocks" ) )
            {
                const KOPENAPI_RESULT r = blocksMethod->handler( aCtx, nlohmann::json{ { "heights", false } } );
                const nlohmann::json  list = r.status == 200 ? r.body.value( "blocks", nlohmann::json::array() ) : nlohmann::json::array();

                for( size_t b = 0; b < list.size(); ++b )
                {
                    for( const nlohmann::json& ref : list[b].value( "parts", nlohmann::json::array() ) )
                    {
                        if( ref.is_string() )
                            blockOfRef.emplace( ref.get<std::string>(), (int) b );
                    }
                }
            }
        }

        auto blockOf = [&]( const FOOTPRINT* aFp )
        {
            auto it = blockOfRef.find( str( aFp->GetReference() ) );
            return it == blockOfRef.end() ? -1 : it->second;
        };

        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( fp->Pads().size() < 4 )
                continue;

            BOX2I centres( fp->Pads().front()->GetPosition(), VECTOR2I( 0, 0 ) );

            for( PAD* pad : fp->Pads() )
                centres.Merge( pad->GetPosition() );

            for( PAD* pad : fp->Pads() )
            {
                const bool fine = KopenapiPadFinePitch( board, pad );

                if( fine )
                    finePads.insert( pad );

                const std::optional<VECTOR2I> dir = outwardOf( pad, centres );

                if( !dir )
                    continue;

                outward[pad] = *dir;

                if( !escapeKeepOn || pad->GetNetCode() == net->GetNetCode() || pad->GetNetCode() <= 0
                    || ( !fine && !special.count( pad->GetNetCode() ) ) )
                {
                    continue;
                }

                const BOX2I    pb = pad->GetBoundingBox();
                const int      half = dir->x ? pb.GetWidth() / 2 : pb.GetHeight() / 2;
                const int      across = dir->x ? pb.GetHeight() : pb.GetWidth();
                const VECTOR2I tip = pad->GetPosition() + *dir * half;
                int            keep = 0;

                if( escapeKeepGiven )
                {
                    keep = pcbIUScale.mmToIU( *escapeKeepGiven );
                }
                else
                {
                    const SHAPE_POLY_SET& cy = fp->GetCourtyard( fp->GetLayer() == B_Cu ? B_Cu : F_Cu );
                    const BOX2I           body = cy.OutlineCount() ? cy.BBox() : fp->GetBoundingBox( false );
                    const int             toEdge = dir->x > 0 ? body.GetRight() - tip.x : dir->x < 0 ? tip.x - body.GetLeft()
                                                   : dir->y > 0 ? body.GetBottom() - tip.y : tip.y - body.GetTop();
                    keep = std::max( toEdge, 0 ) + classViaDia + ruleVia.clearance;
                }

                const VECTOR2I end = tip + *dir * keep;

                for( PCB_LAYER_ID l : pad->GetLayerSet().CuStack() )
                    escapes.push_back( { pad, l, SHAPE_SEGMENT( tip, end, across + 2 * minClearance ) } );

                // a pair's pin: on to the pad of its net where the pair goes (the nearest on another
                // part), a line of the pair's width wide: the pair leaves coupled between the two
                if( !pairNets.count( pad->GetNetCode() ) )
                    continue;

                const PAD* partner = nullptr;
                const int  block = blockOf( fp );
                const std::string pin = str( fp->GetReference() ) + "." + str( pad->GetNumber() );

                for( FOOTPRINT* other : board->Footprints() )
                {
                    if( other == fp || block < 0 || blockOf( other ) != block )
                        continue;

                    for( PAD* p : other->Pads() )
                    {
                        if( p->GetNetCode() == pad->GetNetCode() && ( p->GetLayerSet() & pad->GetLayerSet() & LSET::AllCuMask() ).any()
                            && ( !partner
                                 || ( p->GetPosition() - end ).SquaredEuclideanNorm() < ( partner->GetPosition() - end ).SquaredEuclideanNorm() ) )
                        {
                            partner = p;
                        }
                    }
                }

                if( !partner )
                {
                    pairLanesNone.push_back( { { "pin", pin },
                                               { "why", block < 0 ? "its part is in no block (pcb_blocks)"
                                                                  : "no pad of its net on another part of its block (pcb_blocks): the way on is the router's" } } );
                    continue;
                }

                pairLanes.push_back( { { "pin", pin }, { "to", str( partner->GetParentFootprint()->GetReference() ) + "." + str( partner->GetNumber() ) } } );

                NETCLASS* nc = pad->GetNet() ? pad->GetNet()->GetNetClass() : nullptr;
                const int line = nc && nc->HasDiffPairWidth() ? nc->GetDiffPairWidth()
                                 : nc && nc->HasTrackWidth()  ? nc->GetTrackWidth()
                                                              : board->GetDesignSettings().GetCurrentTrackWidth();
                const int clear = std::max( minClearance, nc && nc->HasClearance() ? nc->GetClearance() : 0 );

                for( PCB_LAYER_ID l : pad->GetLayerSet().CuStack() )
                    escapes.push_back( { pad, l, SHAPE_SEGMENT( end, partner->GetPosition(), line + 2 * clear ) } );
            }
        }
    }

    // the pin whose escape a via at aAt (with its stub from aFrom on aLayer) would sit in, or none;
    // the stub counts from where it leaves aFromPad
    auto inEscape = [&]( const VECTOR2I& aAt, const std::optional<VECTOR2I>& aFrom, PCB_LAYER_ID aLayer,
                         const PAD* aFromPad ) -> const PAD*
    {
        std::optional<VECTOR2I> from = aFrom;

        if( aFrom && aFromPad )
        {
            const BOX2I    pb = aFromPad->GetBoundingBox();
            const VECTOR2I d = aAt - *aFrom;
            double         t = 1.0;

            if( d.x != 0 )
                t = std::min( t, pb.GetWidth() / 2.0 / std::abs( (double) d.x ) );

            if( d.y != 0 )
                t = std::min( t, pb.GetHeight() / 2.0 / std::abs( (double) d.y ) );

            from = *aFrom + VECTOR2I( KiROUND( d.x * t ), KiROUND( d.y * t ) );
        }

        for( const ESCAPE& e : escapes )
        {
            if( e.shape.Collide( aAt, viaDia / 2 ) )
                return e.pad;

            if( from && aLayer == e.layer && e.shape.Collide( SEG( *from, aAt ), width / 2 ) )
                return e.pad;
        }

        return nullptr;
    };

    int escapeSkipped = 0;

    // the pad rows of fine-pitch quad packages: no via on them (it narrows the pins' ways out)
    std::vector<std::pair<FOOTPRINT*, SHAPE_POLY_SET>> rowBands;

    for( const UNDER_PART& u : ringAreas )
    {
        BOX2I centres;
        int   padHalf = 0;
        bool  first = true, fine = false;

        for( PAD* pad : u.fp->Pads() )
        {
            if( !outward.count( pad ) )
                continue;

            const BOX2I pb = pad->GetBoundingBox();
            padHalf = std::max( { padHalf, (int) pb.GetWidth() / 2, (int) pb.GetHeight() / 2 } );
            fine |= finePads.count( pad ) > 0;

            if( first )
                centres = BOX2I( pad->GetPosition(), VECTOR2I( 0, 0 ) );
            else
                centres.Merge( pad->GetPosition() );

            first = false;
        }

        if( first || !fine )
            continue;

        BOX2I outer = centres, inner = centres;
        outer.Inflate( padHalf );
        inner.Inflate( -padHalf );

        SHAPE_POLY_SET band;
        band.NewOutline();
        band.Append( outer.GetLeft(), outer.GetTop() );
        band.Append( outer.GetRight(), outer.GetTop() );
        band.Append( outer.GetRight(), outer.GetBottom() );
        band.Append( outer.GetLeft(), outer.GetBottom() );

        if( inner.GetWidth() > 0 && inner.GetHeight() > 0 )
        {
            SHAPE_POLY_SET hole;
            hole.NewOutline();
            hole.Append( inner.GetLeft(), inner.GetTop() );
            hole.Append( inner.GetRight(), inner.GetTop() );
            hole.Append( inner.GetRight(), inner.GetBottom() );
            hole.Append( inner.GetLeft(), inner.GetBottom() );
            band.BooleanSubtract( hole );
        }

        rowBands.emplace_back( u.fp, std::move( band ) );
    }

    // the exposed pads of quad packages (pads inside the rows): their vias go in them
    std::set<PAD*> exposedPads;

    for( const UNDER_PART& u : ringAreas )
    {
        for( PAD* pad : u.fp->Pads() )
        {
            if( u.exposedPad && !outward.count( pad ) && pad->IsOnLayer( u.layer ) )
                exposedPads.insert( pad );
        }
    }

    // a fine-pitch pad of a quad package's row: without a plane of the net its via goes inward all the
    // same, as the pin's way to the other layers for the router (a port)
    auto rowPin = [&]( const BOARD_CONNECTED_ITEM* aItem )
    {
        if( aItem->Type() != PCB_PAD_T )
            return false;

        const PAD* pad = static_cast<const PAD*>( aItem );
        return finePads.count( pad ) && outward.count( pad )
               && std::any_of( ringAreas.begin(), ringAreas.end(),
                               [&]( const UNDER_PART& u ) { return u.fp == pad->GetParentFootprint(); } );
    };

    // a via joined to a pin of a quad package's row (directly or by its stub; laid here, at the
    // decaps' placement or by hand): the pin's way to the other layers, kept and used as it is
    auto viaOfRowPin = [&]( const PAD* aPad ) -> PCB_TRACK*
    {
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();

        for( BOARD_CONNECTED_ITEM* c : conn->GetConnectedItems( aPad ) )
        {
            if( c->Type() == PCB_VIA_T )
                return static_cast<PCB_TRACK*>( c );

            if( c->Type() == PCB_TRACE_T || c->Type() == PCB_ARC_T )
            {
                for( BOARD_CONNECTED_ITEM* o : conn->GetConnectedItems( c ) )
                {
                    if( o->Type() == PCB_VIA_T )
                        return static_cast<PCB_TRACK*>( o );
                }
            }
        }

        return nullptr;
    };

    // Rail pins of a fine-pitch row: each run of them side by side gets its via pattern once, from
    // the board alone (planRowRailVias), so every call sees the same plan. A planned spot of another
    // net not yet on another layer is kept free, as is the way inward of a signal pin
    // pcb_fanout_ports moves to the other layer
    struct PLANNED
    {
        VECTOR2I spot;
        int      dia;
        int      drill;
    };

    struct LANE
    {
        const PAD* pad;
        SEG        stub;
        VECTOR2I   spot;
        int        dia;
        int        drill;
    };

    std::map<const PAD*, PLANNED> plannedVia;
    std::set<const PAD*>          runSkipped;
    std::set<const PAD*>          railPins;
    std::vector<LANE>             lanes;
    const int                     holeClearance = board->GetDesignSettings().m_HoleClearance;
    const int                     holeToHole = board->GetDesignSettings().m_HoleToHoleMin;
    const int                     stubMin = board->GetDesignSettings().m_TrackMinWidth;

    auto clearOf = [&]( const PAD* aPad )
    {
        NETCLASS* nc = aPad->GetNet() ? aPad->GetNet()->GetNetClass() : nullptr;
        return std::max( minClearance, nc && nc->HasClearance() ? nc->GetClearance() : 0 );
    };

    // the centre distance two vias of other nets keep; a via's centre to a stub's axis
    auto viaApart = [&]( int aDia, int aDrill, int bDia, int bDrill, int aClear )
    {
        return std::max( { ( aDia + bDia ) / 2 + aClear, aDrill / 2 + holeClearance + bDia / 2, bDrill / 2 + holeClearance + aDia / 2,
                           ( aDrill + bDrill ) / 2 + holeToHole } );
    };

    auto stubApart = [&]( int aDia, int aDrill, int aClear )
    {
        return stubMin / 2 + std::max( aDia / 2 + aClear, aDrill / 2 + holeClearance );
    };

    {
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();

        auto reached = [&]( const PAD* aPad )
        {
            for( BOARD_CONNECTED_ITEM* c : conn->GetConnectedItems( aPad ) )
            {
                if( c->Type() == PCB_VIA_T || c->Type() == PCB_ZONE_T )
                    return true;

                if( c->Type() == PCB_TRACE_T || c->Type() == PCB_ARC_T )
                {
                    for( BOARD_CONNECTED_ITEM* o : conn->GetConnectedItems( c ) )
                    {
                        if( o->Type() == PCB_VIA_T )
                            return true;
                    }
                }
            }

            return false;
        };

        RAIL_PLAN_OPTIONS options;
        options.classViaDia = classViaDia;
        options.classViaDrill = classViaDrill;
        options.minViaDia = minViaDia;
        options.minViaDrill = minViaDrill;

        std::set<const FOOTPRINT*> packages;

        for( const UNDER_PART& u : ringAreas )
        {
            if( std::any_of( u.fp->Pads().begin(), u.fp->Pads().end(), [&]( const PAD* aPad ) { return rowPin( aPad ); } ) )
                packages.insert( u.fp );

            for( const RAIL_PIN_PLAN& p : planRowRailVias( board, u.fp, options ) )
            {
                railPins.insert( p.pad );

                if( p.planned )
                    plannedVia[p.pad] = { p.spot, p.dia, p.drill };
                else if( p.skipped )
                    runSkipped.insert( p.pad );
            }
        }

        for( const auto& [pad, plan] : plannedVia )
        {
            if( !reached( pad ) )
                lanes.push_back( { pad, SEG( pad->GetPosition(), plan.spot ), plan.spot, plan.dia, plan.drill } );
        }

        // signal pins that leave by a via (pcb_fanout_ports layer other; without its plan, every
        // fine-pitch row pin with an open connection): their straight way in and a via there
        const int signalDia = minViaDia ? minViaDia : classViaDia;
        const int signalDrill = minViaDia ? minViaDrill : classViaDrill;

        for( const FOOTPRINT* fp : packages )
        {
            std::set<std::string> moved;
            bool                  havePlan = false;

            if( std::optional<KOPENAPI_METHOD> ports = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout_ports" ) )
            {
                const KOPENAPI_RESULT r = ports->handler( aCtx, { { "ref", str( fp->GetReference() ) } } );

                if( r.status == 200 && r.body.contains( "ports" ) && r.body["ports"].is_object() )
                {
                    havePlan = true;

                    for( const auto& [side, list] : r.body["ports"].items() )
                    {
                        for( const nlohmann::json& port : list )
                        {
                            if( port.value( "kind", std::string() ) != "rail" && port.value( "layer", std::string() ) == "other" )
                                moved.insert( port.value( "pad", std::string() ) );
                        }
                    }
                }
            }

            for( const auto& [pad, dir] : outward )
            {
                if( pad->GetParentFootprint() != fp || !rowPin( pad ) || railPins.count( pad ) || pad->GetNetCode() <= 0 || reached( pad ) )
                    continue;

                if( havePlan && !moved.count( str( pad->GetNumber() ) ) )
                    continue;

                const BOX2I    pb = pad->GetBoundingBox();
                const int      padHalf = dir.x ? pb.GetWidth() / 2 : pb.GetHeight() / 2;
                const VECTOR2I spot = pad->GetPosition() - dir * ( padHalf + signalDia / 2 + minClearance );
                lanes.push_back( { pad, SEG( pad->GetPosition(), spot ), spot, signalDia, signalDrill } );
            }
        }
    }

    // the pin whose planned way inward a via at aAt (aVia) or a stub from aFrom would shut
    auto inLane = [&]( const VECTOR2I& aAt, const std::optional<VECTOR2I>& aFrom, bool aVia ) -> const PAD*
    {
        const int own = std::max( minClearance, ruleVia.clearance );

        for( const LANE& l : lanes )
        {
            if( l.pad->GetNetCode() == net->GetNetCode() )
                continue;

            const int clear = std::max( own, clearOf( l.pad ) );

            if( aVia && ( ( aAt - l.spot ).EuclideanNorm() < viaApart( viaDia, viaDrill, l.dia, l.drill, clear )
                          || l.stub.Distance( aAt ) < stubApart( viaDia, viaDrill, clear ) ) )
            {
                return l.pad;
            }

            if( aFrom )
            {
                const SEG stub( *aFrom, aAt );

                if( stub.Distance( l.spot ) < stubApart( l.dia, l.drill, clear ) || stub.Distance( l.stub ) < stubMin + clear )
                    return l.pad;
            }
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

    const bool    planeless = planes.empty();
    std::set<KIID> ports;   // vias of a net without a plane at its row pins, kept for the router

    std::vector<KIID> added;
    nlohmann::json    placed = nlohmann::json::array();
    nlohmann::json    joined = nlohmann::json::array();   // pins joined to their exposed pad by a stub
    nlohmann::json    reused = nlohmann::json::array();   // row pins whose via was there already
    std::set<BOARD_CONNECTED_ITEM*> done;
    int               offPlaneSkipped = 0;
    int               neckedStubs = 0;
    int               minVias = 0;   // vias of the board's smallest size at fine-pitch row pins

    // a pad of the net centred at aAt (where an island's stub starts)
    auto padAt = [&]( const VECTOR2I& aAt ) -> PAD*
    {
        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( !fp->GetBoundingBox().Contains( aAt ) )
                continue;

            for( PAD* pad : fp->Pads() )
            {
                if( pad->GetNetCode() == net->GetNetCode() && pad->GetPosition() == aAt )
                    return pad;
            }
        }

        return nullptr;
    };

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

    // a via stitches when the refilled plane of another layer holds it in a piece that is a plane, not
    // a sliver left between other nets' pours: the zone's largest piece on that layer, or one that
    // holds another pad or via of the net
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
                int                                    main = -1;
                double                                 mainArea = -1;

                for( int o = 0; o < fill->OutlineCount(); ++o )
                {
                    if( const double a = std::abs( fill->COutline( o ).Area() ); a > mainArea )
                    {
                        mainArea = a;
                        main = o;
                    }
                }

                for( int o = 0; o < fill->OutlineCount(); ++o )
                {
                    if( !fill->COutline( o ).PointInside( aVia->GetPosition() ) )
                        continue;

                    if( o == main )
                        return true;

                    auto inPiece = [&]( BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aAt )
                    {
                        if( aItem == aVia || aItem->GetNetCode() != net->GetNetCode() || !aItem->IsOnLayer( l )
                            || !fill->COutline( o ).PointInside( aAt ) )
                        {
                            return false;
                        }

                        for( int h = 0; h < fill->HoleCount( o ); ++h )
                        {
                            if( fill->CHole( o, h ).PointInside( aAt ) )
                                return false;
                        }

                        return true;
                    };

                    for( PCB_TRACK* t : board->Tracks() )
                    {
                        if( t->Type() == PCB_VIA_T && inPiece( t, t->GetPosition() ) )
                            return true;
                    }

                    for( FOOTPRINT* fp : board->Footprints() )
                    {
                        for( PAD* pad : fp->Pads() )
                        {
                            if( inPiece( pad, pad->GetPosition() ) )
                                return true;
                        }
                    }
                }
            }
        }

        return false;
    };

    std::set<std::pair<int, int>> bad;
    std::map<BOARD_CONNECTED_ITEM*, nlohmann::json> misses;   // ends still without a via, and why

    std::set<const PAD*> reusedPins;

    for( const auto& [pad, dir] : outward )
    {
        if( pad->GetNetCode() != net->GetNetCode() || ( !only.empty() && !only.count( pad ) ) )
            continue;

        if( PCB_TRACK* via = viaOfRowPin( pad ) )
        {
            reusedPins.insert( pad );
            reused.push_back( { { "for", stitchedFor( const_cast<PAD*>( pad ) ) },
                                { "via", str( via->m_Uuid.AsString() ) },
                                { "at_mm", { toMm( via->GetPosition().x ), toMm( via->GetPosition().y ) } } } );
        }
    }

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

            if( !only.empty() )
            {
                bool chosen = item->Type() == PCB_PAD_T && only.count( static_cast<PAD*>( item ) );

                if( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T )
                {
                    for( BOARD_CONNECTED_ITEM* other : board->GetConnectivity()->GetConnectedItems( item ) )
                        chosen |= other->Type() == PCB_PAD_T && only.count( static_cast<PAD*>( other ) );
                }

                if( !chosen )
                    continue;
            }

            if( item->Type() == PCB_PAD_T && reusedPins.count( static_cast<PAD*>( item ) ) )
            {
                done.insert( item );
                continue;
            }

            if( item->Type() == PCB_PAD_T && runSkipped.count( static_cast<PAD*>( item ) ) && planeless )
            {
                misses[item] = { { "for", stitchedFor( item ) },
                                 { "at_mm", { toMm( pos.x ), toMm( pos.y ) } },
                                 { "reason", "a pin of a run of rail pins too tight for a via at each (pitch, via and hole "
                                             "clearance): its neighbours of poured nets got them, it is left to the router" } };
                continue;
            }

            if( planeless && !rowPin( item ) )
            {
                misses[item] = { { "for", stitchedFor( item ) },
                                 { "at_mm", { toMm( pos.x ), toMm( pos.y ) } },
                                 { "reason", "the net has no plane: left to the router (only the fine-pitch pins of a quad "
                                             "package get a via, inward, as their way to the other layers)" } };
                continue;
            }

            const PCB_LAYER_ID layer = item->Type() == PCB_ZONE_T ? item->GetLayerSet().CuStack().front()
                                                                    : item->GetLayerSet().CuStack().front();
            const bool  onTrack = item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T;
            const BOX2I box = item->Type() == PCB_ZONE_T || onTrack ? BOX2I( pos, VECTOR2I( 0, 0 ) ) : item->GetBoundingBox();
            bool        ok = false;
            viaDia = classViaDia;
            viaDrill = classViaDrill;

            // a pin of a run takes its planned via (or none: left out of a tight run); another row pin
            // the rules' via, the board's smallest where that does not fit
            const PAD*  rowPad = rowPin( item ) ? static_cast<const PAD*>( item ) : nullptr;
            const auto  plan = rowPad ? plannedVia.find( rowPad ) : plannedVia.end();

            for( int attempt = 0; attempt < 2 && !ok; ++attempt )
            {
                const bool smallVia = attempt == 1;

                if( smallVia && ( !minViaDia || !rowPad || plan != plannedVia.end() ) )
                    break;

                viaDia = plan != plannedVia.end() ? plan->second.dia : smallVia ? minViaDia : classViaDia;
                viaDrill = plan != plannedVia.end() ? plan->second.drill : smallVia ? minViaDrill : classViaDrill;

                // Candidate spots, nearest first: around a pad; inside a zone island (on a grid over the
                // island, the via's copper fully in it)
                std::vector<VECTOR2I>                         candidates;
                std::map<std::pair<int, int>, VECTOR2I>       stubFrom;   // where the stub to a spot starts
                std::map<std::pair<int, int>, PAD*>           stubPadAt;  // the pad a stub leaves, where not at its centre
                std::set<std::pair<int, int>>                 inwardAt;   // spots inward of a pad of a quad package's row
                std::set<std::pair<int, int>>                 inPad;      // spots in an exposed pad: no stub
                std::map<std::pair<int, int>, BOARD_CONNECTED_ITEM*> joinAt;   // a stub into the exposed pad or a via of the net: no via
                std::set<std::pair<int, int>>                 slantAt;    // inward spots off the pad's axis
                std::set<std::pair<int, int>>                 plannedAt;  // the run's planned spot of the pin

                if( plan != plannedVia.end() )
                {
                    const VECTOR2I at = plan->second.spot;
                    candidates.push_back( at );
                    stubFrom[{ at.x, at.y }] = rowPad->GetPosition();
                    stubPadAt[{ at.x, at.y }] = const_cast<PAD*>( rowPad );
                    inwardAt.insert( { at.x, at.y } );
                    plannedAt.insert( { at.x, at.y } );
                }

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

                            if( !island.Contains( pos, -1, board->GetDesignSettings().m_MaxError ) )
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
                        // inward from a pad of a quad package's row: spots in the ring before its exposed pad
                        PAD* basePad = item->Type() == PCB_PAD_T && base == pos ? static_cast<PAD*>( item ) : padAt( base );

                        if( auto dir = basePad ? outward.find( basePad ) : outward.end(); dir != outward.end() )
                        {
                            const bool quad = std::any_of( ringAreas.begin(), ringAreas.end(),
                                                           [&]( const UNDER_PART& u ) { return u.fp == basePad->GetParentFootprint(); } );
                            const BOX2I    pb = basePad->GetBoundingBox();
                            const int      padHalf = dir->second.x ? pb.GetWidth() / 2 : pb.GetHeight() / 2;
                            const int      narrow = dir->second.x ? pb.GetHeight() : pb.GetWidth();
                            const VECTOR2I across( -dir->second.y, dir->second.x );

                            // straight in from the pad's centre, or slanted from its inner end past the
                            // vias of its row neighbours
                            const VECTOR2I tip = base - dir->second * std::max( padHalf - narrow / 2, 0 );

                            // a pin of the exposed pad's net: a stub into it, no via
                            for( PAD* ep : exposedPads )
                            {
                                if( ep->GetParentFootprint() != basePad->GetParentFootprint() || ep->GetNetCode() != net->GetNetCode() )
                                    continue;

                                BOX2I eb = ep->GetBoundingBox();
                                eb.Inflate( -narrow );

                                if( eb.GetWidth() <= 0 || eb.GetHeight() <= 0 )
                                    continue;

                                const VECTOR2I at( std::clamp( tip.x, eb.GetLeft(), eb.GetRight() ),
                                                   std::clamp( tip.y, eb.GetTop(), eb.GetBottom() ) );
                                candidates.push_back( at );
                                stubFrom[{ at.x, at.y }] = at.x == tip.x || at.y == tip.y ? base : tip;
                                stubPadAt[{ at.x, at.y }] = basePad;
                                joinAt[{ at.x, at.y }] = ep;
                            }

                            // a via of the net already in the ring next to it (a row neighbour's): a stub to it
                            for( PCB_TRACK* t : board->Tracks() )
                            {
                                if( !quad || t->Type() != PCB_VIA_T || t->GetNetCode() != net->GetNetCode() )
                                    continue;

                                const VECTOR2I at = t->GetPosition();
                                const VECTOR2I d = at - tip;

                                if( (double) d.EuclideanNorm() > pcbIUScale.mmToIU( 1.2 ) || d.Dot( dir->second ) >= 0
                                    || !std::any_of( ringAreas.begin(), ringAreas.end(),
                                                     [&]( const UNDER_PART& u )
                                                     { return u.fp == basePad->GetParentFootprint() && u.area.Contains( at ); } ) )
                                {
                                    continue;
                                }

                                candidates.push_back( at );
                                stubFrom[{ at.x, at.y }] = tip;
                                stubPadAt[{ at.x, at.y }] = basePad;
                                joinAt[{ at.x, at.y }] = t;
                            }

                            for( int k = 0; quad && k < 20; ++k )
                            {
                                for( int side : { 0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5 } )
                                {
                                    const VECTOR2I at = base - dir->second * ( padHalf + viaDia / 2 + minClearance + k * pcbIUScale.mmToIU( 0.1 ) )
                                                        + across * ( side * pcbIUScale.mmToIU( 0.2 ) );
                                    candidates.push_back( at );
                                    stubFrom[{ at.x, at.y }] = side ? tip : base;
                                    stubPadAt[{ at.x, at.y }] = basePad;
                                    inwardAt.insert( { at.x, at.y } );

                                    if( side )
                                        slantAt.insert( { at.x, at.y } );
                                }
                            }
                        }
                        else if( basePad && base == pos && exposedPads.count( basePad ) )
                        {
                            // an exposed pad: a via in it, nearest its centre first
                            const BOX2I pb = basePad->GetBoundingBox();
                            SHAPE_POLY_SET room;
                            basePad->TransformShapeToPolygon( room, layer, 0, ARC_HIGH_DEF, ERROR_INSIDE );
                            room.Deflate( viaDia / 2, CORNER_STRATEGY::ROUND_ALL_CORNERS, ARC_HIGH_DEF );

                            for( int x = pb.GetLeft(); x <= pb.GetRight(); x += step )
                            {
                                for( int y = pb.GetTop(); y <= pb.GetBottom(); y += step )
                                {
                                    const VECTOR2I at( KiROUND( (double) x / step ) * step, KiROUND( (double) y / step ) * step );

                                    if( room.OutlineCount() && room.Contains( at ) && !inPad.count( { at.x, at.y } ) )
                                    {
                                        candidates.push_back( at );
                                        stubFrom[{ at.x, at.y }] = base;
                                        stubPadAt[{ at.x, at.y }] = basePad;
                                        inPad.insert( { at.x, at.y } );
                                    }
                                }
                            }
                        }

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

                if( planeless )
                    std::erase_if( candidates, [&]( const VECTOR2I& at ) { return !inwardAt.count( { at.x, at.y } ) && !joinAt.count( { at.x, at.y } ); } );

                if( smallVia )
                    std::erase_if( candidates, [&]( const VECTOR2I& at ) { return !inwardAt.count( { at.x, at.y } ); } );

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
                    int    rank;
                    double stub;
                    size_t index;
                };

                // the pad a spot's stub leaves (none for a zone island's spot)
                auto stubPadOf = [&]( const VECTOR2I& at ) -> PAD*
                {
                    if( item->Type() == PCB_ZONE_T )
                        return nullptr;

                    if( auto own = stubPadAt.find( { at.x, at.y } ); own != stubPadAt.end() )
                        return own->second;

                    auto       it = stubFrom.find( { at.x, at.y } );
                    const bool fromPos = it == stubFrom.end() || it->second == pos;

                    if( fromPos && item->Type() == PCB_PAD_T )
                        return static_cast<PAD*>( item );

                    return padAt( fromPos ? pos : it->second );
                };

                // inward under the pad's own package first, then beside a pad of coarse pitch (a cap), a
                // fine-pitch pin's surroundings last
                auto rankOf = [&]( const VECTOR2I& at )
                {
                    PAD* from = stubPadOf( at );

                    if( !from )
                        return 1;

                    if( joinAt.count( { at.x, at.y } ) )
                        return -3;

                    if( plannedAt.count( { at.x, at.y } ) )
                        return -2;

                    if( inPad.count( { at.x, at.y } ) || ( inwardAt.count( { at.x, at.y } ) && !slantAt.count( { at.x, at.y } ) ) )
                        return -1;

                    for( const UNDER_PART& u : ringAreas )
                    {
                        if( u.fp == from->GetParentFootprint() && u.area.Contains( at ) )
                            return 0;
                    }

                    return finePads.count( from ) ? 2 : 1;
                };

                std::vector<SPOT_KEY> keys;
                keys.reserve( candidates.size() );

                for( size_t i = 0; i < candidates.size(); ++i )
                {
                    const VECTOR2I& at = candidates[i];
                    const bool      plane = planeAt.emplace( std::make_pair( at.x, at.y ), onPlane( at ) ).first->second
                                            || joinAt.count( { at.x, at.y } );
                    keys.push_back( { plane, item->Type() == PCB_ZONE_T ? 1 : rankOf( at ), static_cast<double>( stubLength( at ) ), i } );
                }

                const size_t keep = std::min<size_t>( keys.size(), 400 );
                std::partial_sort( keys.begin(), keys.begin() + keep, keys.end(),
                                   []( const SPOT_KEY& x, const SPOT_KEY& y )
                                   {
                                       if( x.plane != y.plane )
                                           return x.plane;

                                       if( x.rank != y.rank )
                                           return x.rank < y.rank;

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
                std::map<std::string, int> escapeOf;       // spots in a pin's escape, per pin
                std::map<std::string, int> laneOf;         // spots in a planned way inward of another pin

                {
                    for( const VECTOR2I& at : candidates )
                    {
                        if( ok )
                            break;

                        if( inner.OutlineCount() && !inner.Contains( at ) )
                            continue;   // too close to the board edge

                        const auto join = joinAt.find( { at.x, at.y } );

                        if( !planes.empty() && !planeAt[{ at.x, at.y }] )
                        {
                            offPlaneSkipped++;
                            continue;
                        }

                        {
                            const auto        from = stubFrom.find( { at.x, at.y } );
                            const bool        stub = item->Type() == PCB_PAD_T || onTrack;
                            const std::optional<VECTOR2I> start = stub ? std::optional<VECTOR2I>( from == stubFrom.end() ? pos : from->second )
                                                                       : std::nullopt;
                            const PAD*        own = stubPadOf( at );
                            const FOOTPRINT*  body = underWhich( at, start, layer, own ? own->GetParentFootprint() : nullptr );

                            if( !body && join == joinAt.end() )
                            {
                                for( const auto& [fp, band] : rowBands )
                                {
                                    if( band.Collide( at, viaDia / 2 ) )
                                        body = fp;
                                }
                            }

                            if( body )
                            {
                                underSkipped[str( body->GetReference() )]++;
                                continue;
                            }

                            if( const PAD* pin = inLane( at, start, join == joinAt.end() ) )
                            {
                                laneOf[str( pin->GetParentFootprint()->GetReference() ) + "." + str( pin->GetNumber() )]++;
                                continue;
                            }

                            if( const PAD* pin = inEscape( at, start, layer, own ) )
                            {
                                escapeOf[str( pin->GetParentFootprint()->GetReference() ) + "." + str( pin->GetNumber() )]++;
                                escapeSkipped++;
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

                        if( join != joinAt.end() )
                        {
                            auto stubTrack = std::make_unique<PCB_TRACK>( board );
                            stubTrack->SetStart( stubFrom[{ at.x, at.y }] );
                            stubTrack->SetEnd( at );
                            stubTrack->SetLayer( layer );
                            stubTrack->SetNet( net );
                            PAD* from = stubPadAt[{ at.x, at.y }];
                            stubTrack->SetWidth( padNeck( board, from, width, layer ).width );
                            std::vector<BOARD_CONNECTED_ITEM*> fresh = { stubTrack.get() };

                            if( !guard( board, fresh, {}, true ).empty()
                                && ( !neckToFit( board, stubTrack.get(), {}, true, stubMinWidth( board, from, net->GetNetCode(), layer ) )
                                     || !guard( board, fresh, {}, true ).empty() ) )
                            {
                                continue;
                            }

                            BOARD_COMMIT commit( context->GetToolManager() );
                            added.push_back( stubTrack->m_Uuid );
                            commit.Add( stubTrack.release() );
                            commit.Push( _( "Stitching stub (API)" ) );
                            KopenapiKeepUiAlive();
                            const PAD* toPad = dynamic_cast<const PAD*>( join->second );
                            joined.push_back( { { "for", stitchedFor( item ) },
                                                { "to", toPad ? str( toPad->GetParentFootprint()->GetReference() ) + "." + str( toPad->GetNumber() )
                                                              : std::string( "via" ) },
                                                { "at_mm", { toMm( at.x ), toMm( at.y ) } } } );
                            done.insert( item );
                            ok = true;
                            continue;
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
                        PAD*                       stubPad = nullptr;
                        STUB_NECK                  neck{ width, std::string() };

                        // a pad, or the end of a track that joins pads into an island without the plane
                        if( ( item->Type() == PCB_PAD_T || onTrack ) && !inPad.count( { at.x, at.y } ) )
                        {
                            track = std::make_unique<PCB_TRACK>( board );
                            auto from = stubFrom.find( { at.x, at.y } );
                            track->SetStart( from == stubFrom.end() ? pos : from->second );
                            track->SetEnd( at );
                            track->SetLayer( layer );
                            track->SetNet( net );

                            // the pad the stub leaves: necked as the router does at fine-pitch pads
                            auto own = stubPadAt.find( { at.x, at.y } );
                            stubPad = own != stubPadAt.end() ? own->second
                                      : item->Type() == PCB_PAD_T ? static_cast<PAD*>( item ) : padAt( track->GetStart() );
                            neck = padNeck( board, stubPad, width, layer );
                            track->SetWidth( neck.width );
                        }

                        std::vector<BOARD_CONNECTED_ITEM*> fresh = { via.get() };

                        if( track )
                            fresh.push_back( track.get() );

                        if( !guard( board, fresh, {}, true ).empty() )
                        {
                            // the stub may still fit narrower: the via alone has to pass first
                            std::vector<BOARD_CONNECTED_ITEM*> viaOnly = { via.get() };
                            const int                          before = track ? track->GetWidth() : 0;

                            if( !track || !stubPad || !guard( board, viaOnly, {}, true, nullptr ).empty()
                                || !neckToFit( board, track.get(), viaOnly, true,
                                               stubMinWidth( board, stubPad, net->GetNetCode(), layer ) )
                                || !guard( board, fresh, {}, true ).empty() )
                            {
                                continue;
                            }

                            if( track->GetWidth() < before )
                            {
                                neck.why += std::string( neck.why.empty() ? "" : "; " ) + "clearance to the copper next to "
                                            + str( stubPad->GetParentFootprint()->GetReference() ) + "."
                                            + str( stubPad->GetNumber() );
                            }
                        }

                        const int stubWidth = track ? track->GetWidth() : 0;

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

                        if( inwardAt.count( { at.x, at.y } ) )
                            placed.back()["inward"] = true;

                        if( viaDia != classViaDia )
                        {
                            placed.back()["via_mm"] = { toMm( viaDia ), toMm( viaDrill ) };
                            placed.back()["board_min_via"] = true;
                            minVias++;
                        }

                        if( planeless )
                        {
                            placed.back()["port"] = true;
                            ports.insert( fresh_vias.back().via );
                        }

                        if( hasTrack && stubWidth < width )
                        {
                            placed.back()["stub_width_mm"] = toMm( stubWidth );
                            placed.back()["necked_from_mm"] = toMm( width );
                            placed.back()["necked_why"] = neck.why;
                            neckedStubs++;
                        }
                        done.insert( item );
                        ok = true;
                    }
                }

                if( !ok )
                {
                    int underCount = 0, escapeCount = 0;
                    std::string underNames, escapeNames;

                    for( const auto& [ref, n] : underSkipped )
                    {
                        underCount += n;
                        underNames += ( underNames.empty() ? "" : ", " ) + ref;
                    }

                    for( const auto& [pin, n] : escapeOf )
                    {
                        escapeCount += n;

                        if( escapeNames.size() < 120 )
                            escapeNames += ( escapeNames.empty() ? "" : ", " ) + pin;
                    }

                    const int offPlaneHere = offPlaneSkipped - offPlaneBefore;

                    misses[item] = { { "for", stitchedFor( item ) },
                                     { "at_mm", { toMm( pos.x ), toMm( pos.y ) } },
                                     { "reason", !candidates.empty() && escapeCount > 0 && escapeCount + underCount + offPlaneHere >= (int) candidates.size()
                                                         ? "every spot near it lies in the straight escape of a fine-pitch, pair or clock pin ("
                                                                   + escapeNames + "; escape_keep_mm to shorten) or under a part's body"
                                                 : !candidates.empty() && underCount > 0 && underCount + offPlaneSkipped - offPlaneBefore >= (int) candidates.size()
                                                         ? "every spot near it lies under the body of " + underNames
                                                                   + " (inside its ring of pads; under_parts: allow to place there)"
                                                 : candidates.empty() ? "no spot to try (the island is too small for a via)"
                                                 : offPlaneSkipped - offPlaneBefore == (int) candidates.size()
                                                         ? "no spot near it over the net's plane on another layer (a "
                                                           "higher-priority pour of another net or a keep-out covers it there)"
                                                         : "no free spot within reach: every spot near it touches "
                                                           "other copper, a hole, a keep-out, the board edge or lies off the net's plane" } };

                    if( !laneOf.empty() )
                    {
                        nlohmann::json kept = nlohmann::json::array();

                        for( const auto& [pin, n] : laneOf )
                            kept.push_back( pin );

                        misses[item]["kept_free_for"] = kept;
                    }
                }
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

            if( ports.count( st.via ) )
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
                                            "or a sliver: not the zone's largest piece on that layer and no other pad or via of the net on it)" } };
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

        // a via an earlier call left as a row pin's port: joined to the pin by its stub
        auto ofRowPin = [&]( const BOARD_CONNECTED_ITEM* aItem )
        { return aItem->Type() == PCB_PAD_T && outward.count( static_cast<const PAD*>( aItem ) ); };

        auto portOfRowPin = [&]( const std::shared_ptr<CONNECTIVITY_DATA>& aConn, PCB_TRACK* aVia )
        {
            for( BOARD_CONNECTED_ITEM* item : aConn->GetConnectedItems( aVia ) )
            {
                if( ofRowPin( item ) )
                    return true;

                if( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T )
                {
                    for( BOARD_CONNECTED_ITEM* other : aConn->GetConnectedItems( item ) )
                    {
                        if( ofRowPin( other ) )
                            return true;
                    }
                }
            }

            return false;
        };
        std::set<std::pair<int, int>>      gone;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() == PCB_VIA_T && t->GetNetCode() == net->GetNetCode() && !t->IsLocked() && !ports.count( t->m_Uuid )
                && !portOfRowPin( conn, t ) && conn->TestTrackEndpointDangling( t, true ) )
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

    if( !aQuiet )
    {
        KopenapiKeepUiAlive();
        showRoutes( aCtx, added );
    }

    KopenapiKeepUiAlive();

    nlohmann::json answer = { { "net", str( netName ) },
                                  { "pair_lanes", pairLanes },
                                  { "pair_lanes_none", pairLanesNone },
                                  { "vias", placed },
                                  { "joined", joined },
                                  { "reused", reused },
                                  { "necked_stubs", neckedStubs },
                                  { "board_min_vias", minVias },
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
                                    { "joined_count", answer["joined"].size() },
                                    { "reused_count", answer["reused"].size() },
                                    { "necked_stubs", neckedStubs },
                                    { "board_min_vias", minVias },
                                    { "dangling_vias_removed", danglingRemoved },
                                    { "not_reached_count", answer["not_reached"].size() },
                                    { "islands_left_count", answer["islands_left"].size() },
                                    { "spots_off_plane", offPlaneSkipped },
                                    { "escape_keep", escapeKeepGiven ? nlohmann::json( *escapeKeepGiven ) : nlohmann::json( "courtyard_via_pitch" ) },
                                    { "spots_in_escapes", escapeSkipped } };

    if( !only.empty() )
        out["pads"] = aArgs["pads"];

    for( const char* key : { "vias", "joined", "reused", "not_reached", "islands_left", "pair_lanes", "pair_lanes_none" } )
        cap.Put( out, key, answer[key] );

    KopenapiKeepUiAlive();
    out.update( KopenapiUnroutedJson( board ) );
    return KOPENAPI_RESULT::Ok( out );
}


/// @brief pcb_stitch; dry_run stitches on the board and takes it all back (the vias and stubs it
/// reports are what the same call places)
static KOPENAPI_RESULT h_pcb_stitch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    if( !aArgs.value( "dry_run", false ) )
        return pcbStitch( aCtx, aArgs, false );

    KOPENAPI_HISTORY& history = KOPENAPI_HISTORY::Get();
    KOPENAPI_KEEP_UNSAVED keep;   // the edits are taken back: the board stays as saved as it was

    if( !history.TrialBegin( aCtx, "pcb" ) )
        return KOPENAPI_RESULT::Error( 409, "dry_run: no copy of the board before the call could be made to take the vias "
                                            "back; stitch for real and roll back (rollback)" );

    KOPENAPI_RESULT result = pcbStitch( aCtx, aArgs, true );
    KOPENAPI_RESULT back = history.TrialRestore( aCtx );

    if( back.status != 200 )
    {
        return KOPENAPI_RESULT::Error( 500, "dry_run: the vias could not be taken back (" + back.body.dump()
                                                    + "): the board holds them; rollback 1 step takes them out" );
    }

    if( result.status == 200 )
    {
        result.body["dry_run"] = true;
        result.body["applied"] = false;
        result.body["simulated"] = "the whole call (vias, stubs, refills, the vias the fill left off the plane taken out) ran "
                                   "on the board and was taken back: the same call places these vias; uuids in them are gone";
    }

    return result;
}


static KOPENAPI_RESULT h_pcb_rail_plan( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string ref = aArgs.value( "ref", std::string() );
    FOOTPRINT*        fp = ref.empty() ? nullptr : board->FindFootprintByReference( wxString::FromUTF8( ref ) );

    if( !fp )
        return KOPENAPI_RESULT::Error( 404, "ref: no footprint '" + ref + "' (the quad package whose rail pins to plan)" );

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

    KOPENAPI_RESULT netError;
    NETINFO_ITEM*   net = stitchNetOf( aCtx, board, aArgs, netError );

    if( !net )
        return netError;

    const RAIL_PLAN_OPTIONS options = railPlanOptions( board, net, fp );

    nlohmann::json pins = nlohmann::json::array();
    int            planned = 0, skipped = 0, fixed = 0;

    for( const RAIL_PIN_PLAN& p : planRowRailVias( board, fp, options ) )
    {
        nlohmann::json row = { { "pad", str( fp->GetReference() ) + "." + str( p.pad->GetNumber() ) },
                               { "net", str( UnescapeString( p.pad->GetNetname() ) ) },
                               { "outward", { p.outward.x, p.outward.y } },
                               { "planned", p.planned },
                               { "skipped", p.skipped },
                               { "pattern", p.pattern },
                               { "reason", p.reason } };

        if( p.planned )
        {
            row["at_mm"] = { toMm( p.spot.x ), toMm( p.spot.y ) };
            row["via_mm"] = toMm( p.dia );
            row["drill_mm"] = toMm( p.drill );
            row["stub_mm"] = { { toMm( p.stub.A.x ), toMm( p.stub.A.y ) }, { toMm( p.stub.B.x ), toMm( p.stub.B.y ) } };
        }

        planned += p.planned;
        skipped += p.skipped;
        fixed += p.pattern == "fixed";
        pins.push_back( std::move( row ) );
    }

    nlohmann::json body = { { "ref", ref }, { "net", str( UnescapeString( net->GetNetname() ) ) }, { "pins", std::move( pins ) }, { "planned_count", planned },
                            { "skipped_count", skipped }, { "fixed_count", fixed },
                            { "class_via_mm", toMm( options.classViaDia ) }, { "class_drill_mm", toMm( options.classViaDrill ) } };

    if( options.minViaDia )
        body["board_min_via_mm"] = { toMm( options.minViaDia ), toMm( options.minViaDrill ) };

    return KOPENAPI_RESULT::Ok( body );
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
        row["start_mm"] = row["from_mm"];
        row["end_mm"] = row["to_mm"];
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

    const bool     full = KopenapiFull( aArgs );
    const size_t   limit = full ? tracks.size() : size_t( std::clamp( aArgs.value( "limit", 500 ), 1, 5000 ) );
    std::string    pageError;
    const size_t   offset = full ? 0 : KopenapiPageStart( aArgs, pageError );

    if( !pageError.empty() )
        return KOPENAPI_RESULT::Error( 400, pageError );

    nlohmann::json items = nlohmann::json::array();
    double         length = 0;
    int            vias = 0;

    std::shared_ptr<CONNECTIVITY_DATA> conn = full ? board->GetConnectivity() : nullptr;

    for( size_t i = 0; i < tracks.size(); i++ )
    {
        if( tracks[i]->Type() == PCB_VIA_T )
            vias++;
        else
            length += tracks[i]->GetLength();

        if( i < offset || items.size() >= limit )
            continue;

        nlohmann::json row = trackJson( board, tracks[i] );

        if( full )
        {
            PCB_TRACK* t = tracks[i];
            NETCLASS*  netclass = t->GetEffectiveNetClass();

            row["net_code"] = t->GetNetCode();
            row["netclass"] = netclass ? str( netclass->GetName() ) : std::string();
            row["locked"] = t->IsLocked();
            row["dangling"] = conn && conn->TestTrackEndpointDangling( t, true );

            wxString source;
            t->GetOwnClearance( t->GetLayer(), &source );
            row["clearance_mm"] = toMm( t->GetOwnClearance( t->GetLayer() ) );
            row["clearance_source"] = str( source );

            if( t->Type() == PCB_VIA_T )
            {
                PCB_VIA*       via = static_cast<PCB_VIA*>( t );
                nlohmann::json span = nlohmann::json::array();

                for( PCB_LAYER_ID l : via->GetLayerSet().CuStack() )
                    span.push_back( str( board->GetLayerName( l ) ) );

                row["layer_span"] = span;
                row["via_type"] = via->GetViaType() == VIATYPE::THROUGH       ? "through"
                                  : via->GetViaType() == VIATYPE::MICROVIA    ? "micro"
                                  : via->GetViaType() == VIATYPE::BLIND       ? "blind"
                                  : via->GetViaType() == VIATYPE::BURIED      ? "buried"
                                                                              : "other";
            }
            else if( t->Type() == PCB_ARC_T )
            {
                const PCB_ARC* arc = static_cast<const PCB_ARC*>( t );
                row["mid_mm"] = { toMm( arc->GetMid().x ), toMm( arc->GetMid().y ) };
            }
        }

        items.push_back( std::move( row ) );
    }

    nlohmann::json out = { { "items", items }, { "total", tracks.size() }, { "vias", vias },
                           { "length_mm", toMm( length ) }, { "missing", missing } };

    KopenapiPageNext( out, offset, items.size(), tracks.size() );

    return KOPENAPI_RESULT::Ok( out );
}


static const char* viaTypeName( VIATYPE aType )
{
    switch( aType )
    {
    case VIATYPE::THROUGH:  return "through";
    case VIATYPE::BLIND:    return "blind";
    case VIATYPE::BURIED:   return "buried";
    case VIATYPE::MICROVIA: return "micro";
    default:                return "undefined";
    }
}


static KOPENAPI_RESULT h_pcb_via_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    std::set<std::string> types;

    for( const nlohmann::json& t : aArgs.value( "via_types", nlohmann::json::array() ) )
    {
        if( !t.is_string() || !std::set<std::string>{ "through", "blind", "buried", "micro" }.count( t.get<std::string>() ) )
            return KOPENAPI_RESULT::Error( 400, "via_types: through, blind, buried, micro" );

        types.insert( t.get<std::string>() );
    }

    BOARD*                         board = context->GetBoard();
    nlohmann::json                 select = aArgs;
    nlohmann::json                 missing = nlohmann::json::array();
    std::optional<KOPENAPI_RESULT> error;

    select["kinds"] = nlohmann::json::array( { "via" } );
    std::vector<PCB_TRACK*> tracks = selectTracks( board, select, false, missing, error );

    if( error )
        return *error;

    const bool     full = KopenapiFull( aArgs );
    const size_t   limit = full ? std::numeric_limits<size_t>::max()
                                : size_t( std::clamp( aArgs.value( "limit", 1000 ), 1, 100000 ) );
    std::string    pageError;
    const size_t   offset = KopenapiPageStart( aArgs, pageError );

    if( !pageError.empty() )
        return KOPENAPI_RESULT::Error( 400, pageError );

    nlohmann::json items = nlohmann::json::array();
    std::map<std::string, int>                     byType;
    std::map<std::pair<double, double>, int>       bySize;
    size_t                                         total = 0;

    for( PCB_TRACK* track : tracks )
    {
        if( track->Type() != PCB_VIA_T )
            continue;

        PCB_VIA*          via = static_cast<PCB_VIA*>( track );
        const std::string type = viaTypeName( via->GetViaType() );

        if( !types.empty() && !types.count( type ) )
            continue;

        const double dia = toMm( via->GetWidth( via->TopLayer() ) );
        const double drill = toMm( via->GetDrillValue() );

        byType[type]++;
        bySize[{ dia, drill }]++;

        if( total++ < offset || items.size() >= limit )
            continue;

        nlohmann::json span = nlohmann::json::array();

        for( PCB_LAYER_ID layer : LSET( via->GetLayerSet() & board->GetEnabledLayers() & LSET::AllCuMask() ).CuStack() )
            span.push_back( str( board->GetLayerName( layer ) ) );

        items.push_back( { { "uuid", str( via->m_Uuid.AsString() ) },
                           { "at_mm", { toMm( via->GetPosition().x ), toMm( via->GetPosition().y ) } },
                           { "net", str( UnescapeString( via->GetNetname() ) ) },
                           { "net_code", via->GetNetCode() },
                           { "diameter_mm", dia },
                           { "drill_mm", drill },
                           { "via_type", type },
                           { "layer_span", { str( board->GetLayerName( via->TopLayer() ) ),
                                             str( board->GetLayerName( via->BottomLayer() ) ) } },
                           { "layers", span },
                           { "unused_layers", KopenapiUnusedLayersMode( via->Padstack() ) },
                           { "locked", via->IsLocked() } } );
    }

    nlohmann::json sizes = nlohmann::json::array();

    for( const auto& [size, count] : bySize )
        sizes.push_back( { { "diameter_mm", size.first }, { "drill_mm", size.second }, { "count", count } } );

    nlohmann::json out = { { "items", items }, { "total", total }, { "by_type", byType }, { "sizes", sizes },
                           { "missing", missing } };

    KopenapiPageNext( out, offset, items.size(), total );

    return KOPENAPI_RESULT::Ok( out );
}


/// @brief Copper of one net as a graph of board items (pads, tracks, vias, zones) by what
/// connectivity says touches what
class NET_COPPER_GRAPH
{
public:
    NET_COPPER_GRAPH( BOARD* aBoard, int aNetCode )
    {
        std::shared_ptr<CONNECTIVITY_DATA>    conn = aBoard->GetConnectivity();
        std::shared_ptr<CN_CONNECTIVITY_ALGO> algo = conn ? conn->GetConnectivityAlgo() : nullptr;

        if( !algo )
            return;

        for( BOARD_CONNECTED_ITEM* item : conn->GetNetItems( aNetCode, { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_ZONE_T } ) )
        {
            m_next[item];

            if( item->Type() == PCB_PAD_T )
                m_pads.push_back( static_cast<PAD*>( item ) );

            if( !algo->ItemExists( item ) )
                continue;

            for( CN_ITEM* cn : algo->ItemEntry( item ).GetItems() )
            {
                for( CN_ITEM* other : cn->ConnectedItems() )
                {
                    if( other->Valid() && other->Parent() != item && other->Parent()->GetNetCode() == aNetCode )
                        m_next[item].insert( other->Parent() );
                }
            }
        }
    }

    const std::vector<PAD*>& Pads() const { return m_pads; }

    /// @brief Component id of every item when aGone are taken out (gone items get -1)
    std::map<const BOARD_CONNECTED_ITEM*, int> Components( const std::set<const BOARD_CONNECTED_ITEM*>& aGone ) const
    {
        std::map<const BOARD_CONNECTED_ITEM*, int> id;
        int                                        next = 0;

        for( const auto& [start, unused] : m_next )
        {
            if( aGone.count( start ) )
            {
                id[start] = -1;
                continue;
            }

            if( id.count( start ) )
                continue;

            std::deque<const BOARD_CONNECTED_ITEM*> queue{ start };
            id[start] = next;

            while( !queue.empty() )
            {
                const BOARD_CONNECTED_ITEM* at = queue.front();
                queue.pop_front();

                for( const BOARD_CONNECTED_ITEM* n : m_next.at( at ) )
                {
                    if( !aGone.count( n ) && !id.count( n ) )
                    {
                        id[n] = next;
                        queue.push_back( n );
                    }
                }
            }

            next++;
        }

        return id;
    }

    /// @brief Pads that touch copper (tracks, vias, zones) now and touch none once aGone are taken out
    std::vector<PAD*> PadsCut( const std::set<const BOARD_CONNECTED_ITEM*>& aGone ) const
    {
        std::vector<PAD*> out;

        for( PAD* pad : m_pads )
        {
            int copper = 0, left = 0;

            for( const BOARD_CONNECTED_ITEM* n : m_next.at( pad ) )
            {
                if( n->Type() == PCB_PAD_T )
                    continue;

                copper++;
                left += !aGone.count( n );
            }

            if( copper > 0 && left == 0 )
                out.push_back( pad );
        }

        return out;
    }

    /// @brief Do the pads other than aSkip stay joined as they are when aGone are taken out
    bool OthersKeepJoined( const std::set<const BOARD_CONNECTED_ITEM*>& aGone, const std::set<const PAD*>& aSkip ) const
    {
        const std::map<const BOARD_CONNECTED_ITEM*, int> before = Components( {} );
        const std::map<const BOARD_CONNECTED_ITEM*, int> after = Components( aGone );
        std::map<int, int>                               mapped;   ///< component before -> after

        for( const PAD* pad : m_pads )
        {
            if( aSkip.count( pad ) )
                continue;

            const int b = before.at( pad ), a = after.at( pad );

            if( auto it = mapped.find( b ); it == mapped.end() )
                mapped[b] = a;
            else if( it->second != a )
                return false;
        }

        return true;
    }

private:
    std::map<const BOARD_CONNECTED_ITEM*, std::set<const BOARD_CONNECTED_ITEM*>> m_next;
    std::vector<PAD*>                                                          m_pads;
};


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

    board->BuildConnectivity();
    board->GetConnectivity()->RecalculateRatsnest();   // the item links the graphs below read

    // between: only the copper the two pads use alone; what other pads of the net need stays
    std::set<const PAD*> betweenPads;
    nlohmann::json       keptShared = nlohmann::json::array();

    if( aArgs.contains( "between" ) && !aArgs.contains( "uuids" ) && !selected.empty() )
    {
        const nlohmann::json& b = aArgs["between"];
        PAD*                  a = *findPad( board, b[0].get<std::string>() );
        PAD*                  z = *findPad( board, b[1].get<std::string>(), a, a );
        NET_COPPER_GRAPH      graph( board, a->GetNetCode() );
        std::set<const BOARD_CONNECTED_ITEM*> gone;
        std::vector<PCB_TRACK*>               exclusive;

        betweenPads = { a, z };

        for( PCB_TRACK* track : selected )
        {
            gone.insert( track );

            if( graph.OthersKeepJoined( gone, betweenPads ) )
            {
                exclusive.push_back( track );
            }
            else
            {
                gone.erase( track );
                keptShared.push_back( str( track->m_Uuid.AsString() ) );
            }
        }

        selected = std::move( exclusive );
    }

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

    if( aArgs.contains( "between" ) && !aArgs.contains( "uuids" ) )
    {
        out["kept_shared_count"] = keptShared.size();
        cap.Put( out, "kept_shared", keptShared );
    }

    // pads left with no copper of their net that had some (a via over a neighbouring pad of the
    // net is that pad's copper too); the pads of between are meant to lose theirs
    nlohmann::json padsCut = nlohmann::json::array();

    if( !doomed.empty() && !cut )
    {
        std::set<const BOARD_CONNECTED_ITEM*> gone( doomed.begin(), doomed.end() );
        std::set<int>                         netCodes;

        for( PCB_TRACK* track : doomed )
            netCodes.insert( track->GetNetCode() );

        for( int code : netCodes )
        {
            if( code <= 0 )
                continue;

            for( PAD* pad : NET_COPPER_GRAPH( board, code ).PadsCut( gone ) )
            {
                if( !betweenPads.count( pad ) )
                    padsCut.push_back( str( pad->GetParentFootprint()->GetReference() ) + "." + str( pad->GetNumber() ) );
            }
        }
    }

    out["pads_cut_count"] = padsCut.size();
    cap.Put( out, "pads_cut", padsCut );

    if( doomed.empty() )
    {
        out["reason"] = !locked.empty() ? "everything selected is locked (override_locks: true)"
                        : !keptShared.empty() ? "the copper joining these two pads is shared with other pads of the net "
                                                "(kept_shared): delete between one of them and the pad it serves"
                        : aArgs.contains( "between" ) ? "no copper joins these two pads"
                                                      : "nothing matches";
        return KOPENAPI_RESULT::Ok( out );
    }

    if( !padsCut.empty() && aArgs.value( "keep_pads", false ) )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "the deletion would leave pads with no copper of their net that "
                                                               "have some now (pads_cut); nothing deleted" );
        refused.body["pads_cut"] = padsCut;
        return refused;
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


static KOPENAPI_RESULT h_pcb_tracks_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( std::optional<KOPENAPI_RESULT> busy = guiRouting() )
        return *busy;

    BOARD* board = context->GetBoard();

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

    auto positiveMm = [&]( const char* aKey, std::optional<int>& aOut ) -> std::optional<KOPENAPI_RESULT>
    {
        if( !aArgs.contains( aKey ) )
            return std::nullopt;

        if( !aArgs[aKey].is_number() || aArgs[aKey].get<double>() <= 0 || aArgs[aKey].get<double>() > 100 )
            return KOPENAPI_RESULT::Error( 400, std::string( aKey ) + ": a size in mm above 0" );

        aOut = pcbIUScale.mmToIU( aArgs[aKey].get<double>() );
        return std::nullopt;
    };

    std::optional<int> width, viaDiameter, viaDrill;

    for( auto [key, value] : { std::pair<const char*, std::optional<int>*>{ "width_mm", &width },
                               { "via_diameter_mm", &viaDiameter },
                               { "via_drill_mm", &viaDrill } } )
    {
        if( std::optional<KOPENAPI_RESULT> bad = positiveMm( key, *value ) )
            return *bad;
    }

    std::optional<PCB_LAYER_ID> layer;

    if( aArgs.contains( "to_layer" ) )
    {
        layer = aArgs["to_layer"].is_string() ? parseLayer( board, aArgs["to_layer"].get<std::string>() ) : std::nullopt;

        if( !layer )
            return KOPENAPI_RESULT::Error( 400, "to_layer: an enabled copper layer (pcb_layer_list), e.g. B.Cu" );
    }

    const bool fromRules = aArgs.value( "from_rules", false );

    if( fromRules && ( width || viaDiameter || viaDrill ) )
        return KOPENAPI_RESULT::Error( 400, "from_rules sets the sizes from the design rules: give it without width_mm / via_*_mm" );

    if( !fromRules && !width && !viaDiameter && !viaDrill && !layer )
        return KOPENAPI_RESULT::Error( 400, "nothing to change: width_mm, via_diameter_mm, via_drill_mm, to_layer or from_rules" );

    const std::string zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (pour fills give way, refilled after) or keep" );

    nlohmann::json                 missing = nlohmann::json::array();
    std::optional<KOPENAPI_RESULT> error;
    std::vector<PCB_TRACK*>        selected = selectTracks( board, aArgs, true, missing, error );

    if( error )
        return *error;

    struct CHANGE
    {
        PCB_TRACK*   item;
        int          width;
        int          drill;
        PCB_LAYER_ID layer;
        std::string  rule;
    };

    const bool              overrideLocks = aArgs.value( "override_locks", false );
    const bool              dryRun = aArgs.value( "dry_run", false );
    const KOPENAPI_LIST_CAP cap( aArgs );
    std::vector<CHANGE>     changes;
    nlohmann::json          locked = nlohmann::json::array();
    nlohmann::json          badVias = nlohmann::json::array();
    int                     unchanged = 0;

    for( PCB_TRACK* track : selected )
    {
        PCB_VIA*  via = track->Type() == PCB_VIA_T ? static_cast<PCB_VIA*>( track ) : nullptr;
        const int now = via ? via->GetWidth( PADSTACK::ALL_LAYERS ) : track->GetWidth();
        CHANGE    c{ track, now, via ? via->GetDrillValue() : 0, via ? UNDEFINED_LAYER : track->GetLayer(), {} };

        if( fromRules )
        {
            wxString       source;
            MINOPTMAX<int> w = track->GetWidthConstraint( &source );

            if( w.HasOpt() )
                c.width = w.PinnedOpt();
            else if( w.Min() > 0 )
                c.width = w.Min();

            c.rule = str( source );

            if( via )
            {
                MINOPTMAX<int> d = via->GetDrillConstraint();

                if( d.HasOpt() )
                    c.drill = d.PinnedOpt();
                else if( d.Min() > 0 )
                    c.drill = d.Min();
            }
        }
        else if( via )
        {
            c.width = viaDiameter.value_or( c.width );
            c.drill = viaDrill.value_or( c.drill );
        }
        else
        {
            c.width = width.value_or( c.width );
            c.layer = layer.value_or( c.layer );
        }

        if( via && c.drill >= c.width )
        {
            badVias.push_back( str( track->m_Uuid.AsString() ) );
            continue;
        }

        if( c.width == now && ( !via || c.drill == via->GetDrillValue() ) && ( via || c.layer == track->GetLayer() ) )
        {
            unchanged++;
            continue;
        }

        if( track->IsLocked() && !overrideLocks )
        {
            locked.push_back( str( track->m_Uuid.AsString() ) );
            continue;
        }

        changes.push_back( c );
    }

    if( !badVias.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 400, "the drill would not be smaller than the via diameter; nothing changed" );
        refused.body["vias"] = badVias;
        return refused;
    }

    std::map<std::string, nlohmann::json> byNet;
    std::vector<PCB_TRACK*>               items;
    std::map<std::string, int>            rules;
    int                                   tracks = 0, vias = 0;

    for( const CHANGE& c : changes )
    {
        nlohmann::json& net = byNet[str( UnescapeString( c.item->GetNetname() ) )];

        if( net.is_null() )
            net = { { "tracks", 0 }, { "vias", 0 } };

        net[c.item->Type() == PCB_VIA_T ? "vias" : "tracks"] = net[c.item->Type() == PCB_VIA_T ? "vias" : "tracks"].get<int>() + 1;
        ( c.item->Type() == PCB_VIA_T ? vias : tracks )++;
        items.push_back( c.item );

        if( !c.rule.empty() )
            rules[c.rule]++;
    }

    nlohmann::json out = { { "changed", { { "tracks", tracks }, { "vias", vias } } },
                           { "unchanged", unchanged }, { "applied", false } };
    bulkSummary( out, cap, items, byNet );
    cap.Put( out, "missing", missing );
    out["locked_skipped_count"] = locked.size();
    cap.Put( out, "locked_skipped", locked );

    if( fromRules )
        out["rules"] = rules;

    if( changes.empty() )
    {
        out["reason"] = !locked.empty() ? "everything to change is locked (override_locks: true)"
                        : unchanged > 0 ? "everything selected has these values already"
                                        : "nothing matches";
        return KOPENAPI_RESULT::Ok( out );
    }

    {
        nlohmann::json uuids = nlohmann::json::array();

        for( const CHANGE& c : changes )
            uuids.push_back( str( c.item->m_Uuid.AsString() ) );

        cap.Put( out, "uuids", std::move( uuids ) );
    }

    BOARD_COMMIT                       commit( context->GetToolManager() );
    std::vector<BOARD_CONNECTED_ITEM*> changed;

    for( const CHANGE& c : changes )
    {
        commit.Modify( c.item );

        if( PCB_VIA* via = dynamic_cast<PCB_VIA*>( c.item ) )
        {
            via->SetWidth( PADSTACK::ALL_LAYERS, c.width );
            via->SetDrill( c.drill );
        }
        else
        {
            c.item->SetWidth( c.width );
            c.item->SetLayer( c.layer );
        }

        changed.push_back( c.item );
    }

    const nlohmann::json violations = guard( board, changed, {}, zoneMode == "refill" );
    const bool           force = aArgs.value( "force", false );

    if( !violations.empty() )
    {
        out["violations_count"] = violations.size();
        cap.Put( out, "violations", violations );
    }

    if( dryRun || ( !violations.empty() && !force ) )
    {
        commit.Revert();

        if( dryRun )
            return KOPENAPI_RESULT::Ok( out );

        KOPENAPI_RESULT rejected = KOPENAPI_RESULT::Error( 422, "rejected by the guard: the changed copper would touch another net "
                                                                "or enter a keep-out (violations); nothing changed; force: true "
                                                                "applies it anyway, as the editor's dialog does" );
        rejected.body.update( out );
        return rejected;
    }

    commit.Push( _( "Edit track and via properties (API)" ) );

    if( zoneMode == "refill" )
    {
        bool filled = false;

        for( ZONE* zone : board->Zones() )
            filled |= !zone->GetIsRuleArea() && zone->IsFilled();

        if( filled )
        {
            refillPours( *context, board );
            out["zones_refilled"] = true;
        }
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

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

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

        if( p.is_object() && ( p.contains( "via" ) || p.contains( "layer" ) ) )
        {
            const nlohmann::json& name = p.contains( "via" ) ? p["via"] : p["layer"];

            if( p.contains( "via" ) && p.contains( "layer" ) && p["via"] != p["layer"] )
                return KOPENAPI_RESULT::Error( 400, "points: via and layer name the same thing (the layer the track continues on); give one" );

            step.via = name.is_string() ? parseLayer( board, name.get<std::string>() ) : std::nullopt;

            if( !step.via )
                return KOPENAPI_RESULT::Error( 400, "points: via is the enabled copper layer the track continues on, e.g. {at: [x_mm, y_mm], via: \"B.Cu\"}" );
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
    const VECTOR2I          ruleAt = steps.empty() ? VECTOR2I() : steps.front().at;
    const KOPENAPI_RULE_VIA ruleVia = KopenapiRuleVia( board, net->GetNetCode(), ruleAt, layer );
    const int  width = aArgs.contains( "width_mm" ) && aArgs["width_mm"].is_number()
                               ? pcbIUScale.mmToIU( aArgs["width_mm"].get<double>() )
                               : KopenapiRuleTrack( board, net->GetNetCode(), ruleAt, layer ).opt;
    const int  viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.0 ) ) : ruleVia.dia;
    const int  viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.0 ) ) : ruleVia.drill;

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
    const bool                              widthGiven = aArgs.contains( "width_mm" );

    // stubs at the end pads: necked as the router does (fine-pitch pads), then to what the
    // clearances leave (unless width_mm says)
    struct PAD_STUB
    {
        PCB_TRACK*  track;
        PAD*        pad;
        std::string why;
    };

    std::vector<PAD_STUB> padStubs;
    std::map<const PCB_TRACK*, size_t> pieceOf;   ///< the pieces a segment was cut into at its pad stubs

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
            const VECTOR2I a = steps[i - 1].at;
            const VECTOR2I b = steps[i].at;
            PAD*           startPad = i == 1 && fromPad && !steps[0].via && !widthGiven ? fromPad : nullptr;
            PAD*           endPad = i + 1 == steps.size() && toPad && !widthGiven ? toPad : nullptr;

            // only the stub that leaves a pad may neck: the rest of a long segment keeps the width
            auto reach = [&]( PAD* aPad )
            {
                const int clearance = netclass && netclass->HasClearance() ? netclass->GetClearance() : 0;
                return KiROUND( aPad->GetBoundingBox().GetSize().EuclideanNorm() / 2.0 ) + 2 * std::max( width, clearance );
            };

            const double   span = ( b - a ).EuclideanNorm();
            const VECTOR2I dir = b - a;

            if( startPad && endPad && reach( startPad ) + reach( endPad ) >= span )
                endPad = nullptr;

            // the part between the stubs: [from, to]
            const VECTOR2I from = !startPad ? a : reach( startPad ) < span ? a + dir.Resize( reach( startPad ) ) : b;
            const VECTOR2I to = !endPad ? b : reach( endPad ) < span ? b - dir.Resize( reach( endPad ) ) : a;

            auto addTrack = [&]( const VECTOR2I& aStart, const VECTOR2I& aEnd, PAD* aStubPad )
            {
                auto track = std::make_unique<PCB_TRACK>( board );
                track->SetStart( aStart );
                track->SetEnd( aEnd );
                track->SetWidth( width );
                track->SetLayer( layer );
                track->SetNet( net );

                if( aStubPad )
                {
                    const STUB_NECK neck = padNeck( board, aStubPad, width, layer );
                    track->SetWidth( neck.width );
                    padStubs.push_back( { track.get(), aStubPad, neck.why } );
                }

                pieceOf[track.get()] = i;
                changed.push_back( track.get() );
                made.push_back( std::move( track ) );
            };

            length += span;

            if( startPad )
                addTrack( a, from, startPad );

            if( from != to )
                addTrack( from, to, nullptr );

            if( endPad )
                addTrack( to, b, endPad );
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

    const std::string zoneMode = aArgs.value( "zones", std::string( "refill" ) );

    if( zoneMode != "refill" && zoneMode != "keep" )
        return KOPENAPI_RESULT::Error( 400, "zones: refill (pour fills give way, refilled after) or keep" );

    nlohmann::json violations = guard( board, changed, {}, zoneMode == "refill" );

    // a pad stub too wide for the clearances there narrows to the widest that keeps them
    if( !violations.empty() && !padStubs.empty() )
    {
        for( PAD_STUB& stub : padStubs )
        {
            std::vector<BOARD_CONNECTED_ITEM*> rest;

            for( BOARD_CONNECTED_ITEM* item : changed )
            {
                if( item != stub.track )
                    rest.push_back( item );
            }

            const int before = stub.track->GetWidth();

            if( !guard( board, { stub.track }, {}, zoneMode == "refill", &rest ).empty()
                && neckToFit( board, stub.track, rest, zoneMode == "refill",
                              stubMinWidth( board, stub.pad, net->GetNetCode(), stub.track->GetLayer() ) )
                && stub.track->GetWidth() < before )
            {
                stub.why += std::string( stub.why.empty() ? "" : "; " ) + "clearance to the copper next to "
                            + str( stub.pad->GetParentFootprint()->GetReference() ) + "." + str( stub.pad->GetNumber() );
            }
        }

        violations = guard( board, changed, {}, zoneMode == "refill" );
    }

    // a stub that kept the full width joins the rest of its segment again
    for( size_t k = 1; k < made.size(); )
    {
        PCB_TRACK* prev = made[k - 1].get();
        PCB_TRACK* cur = made[k].get();

        if( pieceOf.count( prev ) && pieceOf.count( cur ) && pieceOf[prev] == pieceOf[cur] && prev->GetEnd() == cur->GetStart()
            && prev->GetLayer() == cur->GetLayer() && prev->GetWidth() == cur->GetWidth()
            && SEG( prev->GetStart(), cur->GetEnd() ).LineDistance( prev->GetEnd() ) <= 2
            && ( prev->GetEnd() - prev->GetStart() ).Dot( cur->GetEnd() - cur->GetStart() ) > 0 )
        {
            prev->SetEnd( cur->GetEnd() );

            for( PAD_STUB& stub : padStubs )
            {
                if( stub.track == cur )
                    stub.track = prev;
            }

            changed.erase( std::remove( changed.begin(), changed.end(), cur ), changed.end() );
            made.erase( made.begin() + k );
            continue;
        }

        k++;
    }

    nlohmann::json necked = nlohmann::json::array();

    for( const std::unique_ptr<PCB_TRACK>& item : made )
    {
        if( item->Type() != PCB_TRACE_T )
            continue;

        segments.push_back( { { "from", { toMm( item->GetStart().x ), toMm( item->GetStart().y ) } },
                              { "to", { toMm( item->GetEnd().x ), toMm( item->GetEnd().y ) } },
                              { "layer", str( board->GetLayerName( item->GetLayer() ) ) },
                              { "width_mm", toMm( item->GetWidth() ) } } );

        for( const PAD_STUB& stub : padStubs )
        {
            if( stub.track == item.get() && item->GetWidth() < width )
            {
                segments.back()["necked_from_mm"] = toMm( width );
                necked.push_back( { { "pad", str( stub.pad->GetParentFootprint()->GetReference() ) + "."
                                                     + str( stub.pad->GetNumber() ) },
                                    { "width_mm", toMm( item->GetWidth() ) },
                                    { "necked_from_mm", toMm( width ) },
                                    { "why", stub.why } } );
            }
        }
    }

    nlohmann::json out = { { "net", str( UnescapeString( net->GetNetname() ) ) }, { "segments", segments },
                           { "length_mm", toMm( length ) }, { "vias", vias }, { "width_mm", toMm( width ) },
                           { "applied", false } };

    if( !necked.empty() )
    {
        out["necked"] = necked;
        out["note"] = "the stub at the pad is narrower than the net class width (" + std::to_string( toMm( width ) ).substr( 0, 5 )
                      + " mm): see necked";
    }

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

    // ports: the layout's plan for each pin (pcb_fanout_ports): a via only for rails and for pins the
    // order at their far end moves to the other layer; the rest leave on the pad's layer
    struct PORT_PLAN
    {
        std::string kind, layer, to;
    };

    const bool                 withPorts = aArgs.value( "ports", false );
    const bool                 portStubs = aArgs.value( "stubs", true );
    std::map<std::string, PORT_PLAN> plan;

    if( withPorts )
    {
        std::optional<KOPENAPI_METHOD> ports = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout_ports" );

        if( !ports )
            return KOPENAPI_RESULT::Error( 500, "pcb_fanout_ports is not registered" );

        KOPENAPI_RESULT r = ports->handler( aCtx, { { "ref", str( aFp->GetReference() ) } } );

        if( r.status != 200 )
            return r;

        for( const auto& [side, list] : r.body["ports"].items() )
        {
            for( const nlohmann::json& port : list )
            {
                plan[port.value( "pad", std::string() )] = { port.value( "kind", std::string() ), port.value( "layer", std::string() ),
                                                              port.value( "to", std::string() ) };
            }
        }
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

    // the part's via as KiCad's rules size it (no net: the default class's and the rules for the part)
    const KOPENAPI_RULE_VIA ruleVia = KopenapiRuleVia( aBoard, 0, aFp->GetPosition(), aFp->GetLayer() == B_Cu ? B_Cu : F_Cu );
    const int clearance = std::max( ruleVia.clearance, 1 );
    const int viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.0 ) )
                                                           : std::max( bds.m_ViasMinSize, ruleVia.dia );
    const int viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.0 ) )
                                                          : std::max( bds.m_MinThroughDrill, ruleVia.drill );
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
        // the rules' track, narrowed so two side by side keep the clearance at the pitch
        const int     ruleWidth = KopenapiRuleTrack( aBoard, todo.front()->GetNetCode(), todo.front()->GetPosition(),
                                                     todo.front()->IsOnLayer( F_Cu ) ? F_Cu : B_Cu ).opt;
        int           width = aArgs.contains( "width_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "width_mm", 0.0 ) )
                                                           : std::min( ruleWidth, pitch - clearance );

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

            const auto     portOf = plan.find( str( pad->GetNumber() ) );
            const bool     padVia = !withPorts ? withVias
                                    : portOf != plan.end() && ( portOf->second.kind == "rail" || portOf->second.layer == "other" );

            if( withPorts && !portStubs && !padVia )
                continue;

            const VECTOR2I knee = point( along( at ), tipLine );
            const VECTOR2I ray = point( fanned, rayEnd );

            // the preferred line first, then the others, then further out
            std::vector<int> tries;

            for( int r = 0; r < rows + 2; ++r )
                tries.push_back( r < rows ? ( first + r ) % rows : r );

            for( int r : tries )
            {
                const int64_t  reach = padVia ? rayEnd + r * ( viaDia + clearance ) : toCourtyard;
                const VECTOR2I spot = point( fanned, reach );

                std::unique_ptr<PCB_VIA> via;

                if( padVia )
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

                    if( !padVia )
                        break;

                    continue;
                }

                nlohmann::json row = { { "pad", str( aFp->GetReference() ) + "." + str( pad->GetNumber() ) },
                                       { "net", str( UnescapeString( pad->GetNetname() ) ) },
                                       { "side", sideName[side] },
                                       { "end_mm", { toMm( spot.x ), toMm( spot.y ) } },
                                       { "far_end_mm", { toMm( fars[pad]->x ), toMm( fars[pad]->y ) } } };

                if( padVia )
                    row["row"] = r;

                if( withPorts && portOf != plan.end() )
                {
                    row["port"] = { { "kind", portOf->second.kind }, { "layer", portOf->second.layer } };

                    if( !portOf->second.to.empty() )
                        row["port"]["to"] = portOf->second.to;
                }

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

    if( withPorts )
    {
        int planVias = 0;

        for( const nlohmann::json& row : placed )
            planVias += row.contains( "row" );

        answer["ports"] = { { "planned", plan.size() }, { "vias", planVias } };
    }

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

    // at the edge: no track of the board's narrowest fits between the pad and the edge at the rules' clearances
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    const int nearby = std::max( bds.m_CopperEdgeClearance, 0 ) + std::max( bds.m_TrackMinWidth, 1 ) + std::max( bds.m_MinClearance, 0 );
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

    // the part's via as KiCad's rules size it (no net: the default class's and the rules for the part)
    const KOPENAPI_RULE_VIA ruleVia = KopenapiRuleVia( aBoard, 0, aFp->GetPosition(), aFp->GetLayer() == B_Cu ? B_Cu : F_Cu );
    const int viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.0 ) )
                                                           : std::max( bds.m_ViasMinSize, ruleVia.dia );
    const int viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.0 ) )
                                                          : std::max( bds.m_MinThroughDrill, ruleVia.drill );

    // without a neighbour on its layer a pin's jog aside is half a via pitch
    std::nth_element( gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end() );
    const int pitch = gaps.empty() ? viaDia + ruleVia.clearance : (int) std::max<int64_t>( gaps[gaps.size() / 2], 1 );
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

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

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

    // an array: balls on the pitch lattice inside its outer ring, in two rows and two columns at least
    // (a QFN / QFP ring holds at most its exposed pad inside; a DDR package's corridor still leaves rows)
    const int          tolerance = std::max( board->GetDesignSettings().m_MaxError, 1 );
    std::set<int>      innerRows, innerCols;

    for( PAD* b : balls )
    {
        const VECTOR2I d = b->GetPosition() - span.GetOrigin();
        const int      i = KiROUND( double( d.x ) / pitch ), j = KiROUND( double( d.y ) / pitch );

        if( std::abs( d.x - i * pitch ) > tolerance || std::abs( d.y - j * pitch ) > tolerance )
            continue;

        if( i > 0 && j > 0 && i < cols && j < rows )
        {
            innerCols.insert( i );
            innerRows.insert( j );
        }
    }

    if( cols < 2 || rows < 2 || innerRows.size() < 2 || innerCols.size() < 2 )
        return peripheralFanout( aCtx, *context, board, fp, aArgs );

    const BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
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

    // the via the rules size for the balls' nets where it fits between them, else the largest that fits
    // (its drill keeping the minimum ring); the board's minimums as a floor
    const VECTOR2I          spot = balls.front()->GetPosition() + VECTOR2I( pitch / 2, pitch / 2 );
    const KOPENAPI_RULE_VIA ruleVia = KopenapiRuleVia( board, balls.front()->GetNetCode(), spot, F_Cu );
    const int               fitted = std::max( minDia, std::min( ruleVia.dia, fitDia ) );

    int viaDia = aArgs.contains( "via_diameter_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_diameter_mm", 0.0 ) ) : fitted;
    int viaDrill = aArgs.contains( "via_drill_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "via_drill_mm", 0.0 ) )
                                                    : std::max( minDrill, std::min( ruleVia.drill, viaDia - 2 * std::max( minRing, 1 ) ) );

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

    // the outer rings that leave on their own layer: ring n's track passes between the balls of the n - 1
    // rings outside it, so n - 1 tracks of the rules' width fit in the gap between two balls at the rules'
    // clearances; none when not even the outer ring's tracks run side by side at the pitch
    const int ruleWidth = KopenapiRuleTrack( board, balls.front()->GetNetCode(), balls.front()->GetPosition(), F_Cu ).opt;
    int       defaultDirect = 0;

    if( ruleWidth + clearance <= pitch )
    {
        defaultDirect = 1;

        while( defaultDirect < 20 && defaultDirect * ruleWidth + ( defaultDirect + 1 ) * clearance <= pitch - 2 * ballRadius )
            defaultDirect++;
    }

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
                   "A failed route names blocked_by (pads, other nets' copper, rule areas barring tracks: rule_area, uuid, layer) "
                   "where it stopped or could not start. "
                   "dry_run: measure only (geometry, length, detour, vias, shoved items), nothing changes",
                   R"json({"type":"object","required":["from"],"properties":{
                        "from":{"description":"REF.PAD, or a spot on copper of a net: [x_mm, y_mm] or {at: [x_mm, y_mm], layer} (a track end, a via, a pad)"},
                        "to":{"description":"REF.PAD on the same net, or a spot [x_mm, y_mm] / {at, layer}: the net's copper there (track end, via, pad) or a free spot the route ends at (with layer: a via there when the route arrives on another); default: nearest unconnected"},
                        "layer":{"type":"string","description":"start layer; default the 'from' pad's (F.Cu, B.Cu for a pad on the back only). A 'to' on another layer than the route arrives on gets a via in front of it (via_near_target_mm)"},
                        "mode":{"type":"string","enum":["walkaround","shove"],"default":"walkaround"},
                        "waypoints":{"type":"array","description":"points to pass in order: [x_mm, y_mm] fixes a corner there; {at: [x_mm, y_mm], via: 'B.Cu'} drops a through via there and continues on that layer (layer: the same key, as in pcb_track_add points; both given must agree)","items":{"oneOf":[
                            {"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2,"description":"[x_mm, y_mm]: a corner"},
                            {"type":"object","required":["at"],"properties":{"at":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},
                                "via":{"type":"string","description":"layer after a through via at this point"},
                                "layer":{"type":"string","description":"the same as via"}}}]}},
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
                   "connection rip_up_budget_s; a cancel or a spent budget stops a router try within its walkaround / shove "
                   "steps (the row's reason says so; profile tries_stopped); "
                   "every route passes the net-merge guard; copper of this call that ends nowhere is removed "
                   "(stubs_removed); copper of nets outside the call that shove pushed aside is listed "
                   "(other_nets_moved: pieces and length before / after); a row still routed at the end is "
                   "joined by copper; per connection "
                   "routed / reason and ms (every try for it), a failed row's blocked_by (what stands where it stopped or "
                   "could not start: pads, other nets' tracks / vias, rule areas barring tracks by rule_area name, uuid, layer), "
                   "totals (routed, length, vias, unrouted left) and "
                   "profile (attempts; clearance_lookups / rule_evaluations: the router's clearances past its caches and the design "
                   "rule evaluations they took; dead_legs_skipped / via_spots_blocked / via_spots_in_escapes / rip_ups_skipped / rip_ups_walled: tries not made; "
                   "rip_ups_planned: rip-ups whose routes a plan chose; rip_ups_second: routes taken out that had no way back "
                   "and moved others in turn; rip_ups_failed_at: failed rip-ups by what failed (route, victim, cut); a row left "
                   "failed lists its rip_ups_failed (taken_out, failed_at); "
                   "a row whose end is shut in has boxed_in (end, which, by: other nets' copper nearer than a track of its width needs, and pads; cause pad_clearance: the clearance to its own row neighbours leaves no track of the board minimum, escapes_kept: the ways out kept open for other nets' fine-pitch pads shut it, the reason names them); "
                   "pairs_stuck: pair tries ended when nothing was placed after 3 steps; front_retries: tries refused for walling in another pin's port (a port try's first, else the first), made again as the connection's last try with the ways out as obstacles to the search (tried / won; the row's try says walls_in_first); pad_exits: starts stuck in their pad that tried / won a corner out of a free side (row left_pad_by_mm); plans / plans_found / plan_ms: global plans (plans_whole_board: searched again over the whole board when the box round the ends cut the way; plans_stopped: ended by job_cancel, a budget or stop_on_plateau_s); sync_ms: building the router's world for each; move_ms: the path search; "
                   "route_ms: all but the world; by_kind: tries / routed / ms on a layer (walkaround, shove), "
                   "through one via, two vias; failed_by: failed tries by reason; by_pass: first, retry from both ends, "
                   "rip_up, gaps); "
                   "nets a pcb_net_roles pair names (USB_DP_MCU / USB_DM_MCU too) are routed coupled as well (diff_pairs); a pair whose target pads lie apart is coupled up to where "
                   "its line ends (row diff_pair.ends_short_mm), each net goes on alone from there; to two parts side by side both lines are cut back first "
                   "(diff_pair.split_back_mm) and each tail leaves its line at the line's width, the pair's own way (not held to other pads' ways out); diff_pair_lengths: each pair's two "
                   "net lengths and skew, against a custom skew rule (skew_max_mm, skew_rule, skew_ok); "
                   "bus routes a bundle's members one after another in pin order, bus_layer through a diagonal via fan (answer bus, row bus); "
                   "bus_paths routes a bus along a corridor, pcb_floorplan's path between two blocks or points given: its members side by "
                   "side at their pitch across each portal of the path as fixed corners, in the order they lie across the way in, one "
                   "after another; its line pulled tight inside the regions crossed, clear of copper and track keep-outs (a corner per "
                   "turn, corridor pulled_from; a turn or run and its legs blocked: the crossings moved along their stretches, "
                   "corridor crossings_moved), each member first out through its pin's port tip (pcb_fanout_ports, packages of 8 SMD "
                   "pads or more, edge fingers too, away from the board edge: corridor fanned_out; an end's own crossings at or "
                   "behind its tips kept where they are, not moved off its pads' copper: behind_tips) (answer bus_paths: corridors, members, routed_along; row corridor: name, index, order (its place in the corridor's tries), corners, routed, failed; "
                   "a member the corridor does not route is routed as any other, one routed along it is left out of clean_up); "
                   "rip_up_scope unlocked lets the rip-up take out unlocked copper of earlier calls too (rip_up.earlier_routes_rerouted, "
                   "other_nets_moved); the first pass gives each connection first_pass_budget_s (row budget, profile budget); "
                   "dry_run (pns) routes the whole call on the board and takes it back: its rows are what applying gives (simulated says so); "
                   "rows (results, failed ones first when cut), other_nets_moved and the grid lists capped at list_limit (counts whole), full: true for all",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"string"}},
                        "layers":{"type":"array","items":{"type":"string"},"default":["F.Cu","B.Cu"]},
                        "order":{"type":"string","enum":["short_first","power_first","dense_first"],"default":"short_first","description":"dense_first: connections at fine-pitch pads first (no track of the net as wide as the rules ask passes between the pad and its neighbour at the rules' clearance, or two of its vias do not fit side by side at the pitch)"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills out while routing (they would block every crossing), refilled after; keep: routes must clear the fills"},
                        "clean_up":{"type":"boolean","default":true,"description":"after routing, each connection with vias or a detour (over 1.3 x the straight line) is routed again alone on a layer both ends are on, without moving anything; kept with fewer vias, or as many and 5 % shorter (a plan over the free space of those layers first: none, or no shorter, skips it); a detour over 1.6 x follows a plan through vias instead when that is a quarter shorter, a via counted as 2 mm (kept when a fifth shorter); clean_up: tried / skipped / kept / vias_removed / via_ways / vias_added / shortened_mm; rows cleaned"},
                        "rip_up_budget_s":{"type":"number","default":30,"minimum":0,"description":"seconds the retry / rip-up pass gives one connection (its tries and the routes it took out going back), and the gaps pass one gap between two pieces of a net; spent: the rip-up fails and what it took out goes back; 0 no limit"},
                        "attempts":{"type":"boolean","default":false,"description":"pns: each row lists its tries (at most 40): kind, pass, layer, from, points (vias / corners), routed, ms; a failed one its reason, stopped_at_mm, toward_mm, walls_in, the nearest blocker"},"stop_on_plateau_s":{"type":"number","default":0,"description":"pns: stop the call when no connection more was routed for this long (seconds; 0 never), as job_cancel does; while connections still wait for their first-pass try (first_pass_budget_s bounding it) or, failed there, for their turn in the retry pass (fewest tries first), it ends the turn of the one routing instead (budget stop_on_plateau_s): what is routed is kept; the answer says stopped_on_plateau, stop_reason in job_get"},"first_pass_budget_s":{"type":"number","default":30,"minimum":0,"description":"seconds the first pass gives one connection; spent: it is left to the retry / rip-up pass (row budget: first_pass_budget_s, profile budget.first_pass_left); 0 no limit"},
                        "budgets":{"type":"string","enum":["time","work"],"default":"time","description":"time: first_pass_budget_s, rip_up_budget_s and stop_on_plateau_s on the wall clock; work: counted in the router's work (its stop checks: each walkaround / shove step and try) at work_per_s a second, so the same board routes the same under any load (A/B comparisons), the wall clock only a hard cap at wall_cap_x times a budget (profile budget: mode, work, work_per_s, wall_caps)"},
                        "trace_attempt":{"type":"integer","minimum":0,"default":0,"description":"debug: trace the router's try of this number (1 = the first; the segments' attempt): trace {attempt, from, to, layer, mode, vias, fronts, board_tracks (the board's track order), board_tracks_hash, pns {events: the router's steps as text without addresses, file:line first, total, capped, other_threads}, pns_hash, routed, out}; two runs' traces show where they part"},
                        "trace_last":{"type":"integer","minimum":0,"description":"debug: with trace_attempt, trace every try up to this one: traces [{attempt, from, to, ..., board_tracks_hash, pns_hash, pns_total, routed}] (no steps, no track list): the first try whose hashes differ between two runs is the one to trace alone"},
                        "work_per_s":{"type":"number","default":5000,"minimum":1,"description":"budgets work: the work counted as one second (5000: about what U18 routes a second on the Mac Studio under load 15..20)"},
                        "wall_cap_x":{"type":"number","default":10,"minimum":1,"description":"budgets work: a budget also ends after this many times its seconds on the wall clock (wall_caps counts them; a capped run is no longer load-independent)"},
                        "rip_up_scope":{"type":"string","enum":["call","unlocked"],"default":"call","description":"call: the rip-up takes out only routes of this call; unlocked: also unlocked copper of other nets near the connections left (earlier calls, laid by hand; pairs and locked copper never), routed again after (rip_up.earlier_routes_rerouted, other_nets_moved) or all put back"},
                        "bus":{"description":"pns: \"off\", \"auto\" (pcb_net_roles' bundles) or [{name, nets}]: the members of a bus leaving the same side of a package are routed one after another in pin order, the pin on the inside of the turn first (answer bus: name, part, side, nets in order; row bus)","default":"off"},
                        "ports":{"oneOf":[{"type":"boolean"},{"type":"array","items":{"type":"string"}}],"description":"layout level 1 goals: the signal pins of these packages (true: every package with 8 SMD pads or more) follow their port from pcb_fanout_ports: the way out on the pin's side is kept open to past the courtyard on the pad's layer (a pin planned round_the_part: to the courtyard's edge, it turns right past it; layer other: to the via spot the plan's fan-out would lay, the spot kept free on every layer), and the pin's routes leave through it first (the tip as a corner, or the via), arriving through the far pin's port when the layers allow; a connection that does not route that way is routed as any other. Rows routed so name port (round_the_part when planned so); ports answers per package the pins and vias planned, tries, won and ms"},
                        "lanes":{"type":"boolean","default":false,"description":"with ports: each pin kept on its pad's layer runs its lane from pcb_fanout_ports lanes. The lanes round a package are laid as tracks before any route (where the guard lets them, at both ends of a connection); the leg between two lanes whose ends face each other is a straight run, one 45 degree jog and a straight run, the jogs of a bus staggered so its legs keep their pitch, laid as computed where it is clean, else routed from lane end to lane end. A lane its route did not use is taken out again. Routes so made are left as they are by clean_up. Rows routed so name port.lane (the rank), ring_laid, far_ring_laid; ports.lanes counts laid_tracks, refused, taken_out"},
                        "bus_paths":{"description":"pns: \"off\", \"floorplan\" (or {floorplan: {pcb_floorplan arguments}}): every bus link pcb_floorplan finds, along its corridor, the connections of its nets between the two blocks' parts; or [{name, nets, points_mm: [[x_mm, y_mm] (a free point) or {at_mm, along: x | y, lo_mm, hi_mm, into_mm: [x0, y0, x1, y1]} (a portal crossed into that region: the ribbon inside lo..hi, joined to the next portal by axis-parallel runs inside the region; rooms_mm: [[x0, y0, x1, y1], ...] the regions to the next gate, more than one: joined by the straight run, which lies inside them (docs corridor.md))]}]: the connections of those nets, from the end nearer the first point. A corridor is a soft goal: at the turn of its first connection each member gets one try along it, across it in order, in a tenth of first_pass_budget_s; the members it does not route keep their turn and their usual tries with their whole budget. The answer's corridors give the centre line (line_mm) the members run beside at their pitch","default":"off"},
                        "bus_layer":{"type":"string","description":"with bus: the members drop a through via each on a diagonal fan in front of the pad row (via pitch apart, the later members' spots kept free) and go on on this layer; a member the fan does not route is routed as any other (row bus.fan, fan_failed)"},
                        "rip_up":{"type":"integer","default":2,"minimum":0,"maximum":6,"description":"rounds of rip-up and retry for failed connections (routes of this call in their way taken out and routed again, up to 3 choices of them per connection and round; all back if that fails)"},
                        "vias":{"type":"boolean","default":true,"description":"connections blocked on their side: escape through vias near both ends"},
                        "diff_pairs":{"type":"string","enum":["auto","off"],"default":"auto","description":"auto: pairs (USB_D+ / USB_D-, LVDS_P / LVDS_N) routed coupled; off: every net alone"},
                        "engine":{"type":"string","enum":["pns","grid"],"default":"pns","description":"grid: every net at once on one grid of the board, negotiated congestion (PathFinder) in parallel variants, the best legal result committed (diff-pair trunks still by pns, their rows say engine pns; no rip_up / clean_up passes; with dry_run the same search and guard, rows / totals / grid answered, nothing put on the board, the fills counted out as zones: refill would, the trunks not routed: their rows say not_simulated with the reason and the pairs' airwires are in the grid search; with from: 400; a fine-pitch end the grid walls in tries a lattice half a cell off at its pad: row half_cell_exits, grid.half_cell {tried, won, ms}); pns: one connection at a time with the push-and-shove router"},
                        "max_rounds":{"type":"integer","default":40,"description":"grid: negotiation rounds before the nets still crowding each other are routed apart or left out"},
                        "cell_mm":{"type":"number","default":0,"description":"grid: the cell, 0 a third of the finest track pitch (width + clearance) on a 0.025 mm step (stage global: half the pitch)"},
                        "stage":{"type":"string","enum":["full","global","detail"],"default":"full","description":"engine grid: full lays the grid's copper; detail: the global stage, then the grid's copper of each net cut into branches between its junctions (ends, points where three ways meet), each branch by pns along its corridor (read as bus_paths reads one: the lane's corners and via gates as waypoints; bundles first, then branches between two ends, then onto a junction, shorter first; one that does not route tries its via sites alone, then the grid's copper of it through the guard): rows engine pns with detail branches, or engine grid with pns_detail (why pns did not); grid.detail {branches, by_corridor, by_via_sites, already_joined, to_grid_copper}; global only finds each connection's way on a coarse grid (cell half a track pitch), nothing laid: per connection from, to, found, corridor (the shared corridor type, docs corridor.md: {name, net, from, to, points_mm: gates at the corners {at_mm, along, lo_mm = hi_mm, layer, rooms_mm}, via gates {at_mm, via: {from_layer, to_layer}, layer}, the last a free point}; rooms are runs of free cells widened up to 1 mm, other nets' never overlap; bus_paths takes it as given), vias_mm (where it changes layer), layers, length_mm; answer connections, covered, vias, grid (variant, cell_mm, rounds, build_ms, route_ms, variant_stats)"},
                        "race_wait":{"type":"number","default":0.25,"description":"grid: once the first variant is done, the others get this share of its time again (race_wait_min_s at least); stages global and detail 0.1 (a time policy: their grid result is a plan the detail stage refines)"},"race_wait_min_s":{"type":"number","default":2,"minimum":0,"description":"grid: the least time the other variants get once the first is done (seconds)"},"corridor_try_budget_s":{"type":"number","minimum":0,"description":"seconds one member's try along its corridor gets at the turn of the corridor's first connection (a time policy: the try is a soft goal, the member keeps its own turn and budget after); default first_pass_budget_s / 10, 3 when that is 0"},"detail_branch_budget_s":{"type":"number","minimum":0,"description":"stage detail: seconds pns gets for one branch along its corridor (a time policy: a branch is one of many and has the grid's copper as its fallback); default first_pass_budget_s / 10, 3 when that is 0"},
                        "via_keepout_mm":{"type":"number","default":5,"description":"grid: only a chip's own nets put vias this far around it (pads on four sides, or an array outside its balls) while the board routes; other nets left short and, once all are joined, the via-saving pass may use it; 0 off"},
                        "bundles":{"description":"grid: \"auto\" (pcb_net_roles' bundles), \"off\", or [{name, nets}] in member order: a bundle's nets between the same parts are routed first, side by side (the answer's grid.bundles: name, parts, nets)","default":"auto"},
                        "via_away_mm":{"type":"number","default":7,"description":"grid: a signal via nearer a chip's pads (not an array's) than this costs more, linearly to the pads; 0 off"},
                        "via_away_cost_mm":{"type":"number","default":8,"description":"grid: a via at a chip's pads costs as much more as this length of track"},
                        "body_cost":{"type":"number","default":1,"description":"grid: a track inside a fine-pitch package's ring of pins (between the pin rows and the exposed pad, on any layer) costs this many times more: the pins' ways in and their fan-out vias keep their room; 1 off"},
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
                        "neck_down":{"oneOf":[{"type":"string","enum":["auto","off"]},{"type":"object","additionalProperties":{"type":"string","enum":["auto","off"]}}],"default":"auto","description":"auto: a connection with an end on a fine-pitch pad (as dense_first says) is routed no wider than a track centred on the pad leaving between its neighbours at the clearance DRC asks to each, not below the narrowest track KiCad's rules allow the net at the pad (track_width min, the board minimum); the rest of its net at its class width (grid: narrow only inside the courtyard of the fine-pitch footprint (the pad entry), the rest of the net widened to its class width where clearance allows (else neck_width_mm stays there and the row says narrow_outside_mm); a custom track_width rule at the pad sets the neck lower, its opt else min, not below the board minimum); the row says necked_from_mm. Per net: {\"VBUS\": \"off\", \"*\": \"auto\"}"},
                        "inward_escapes":{"type":"array","items":{"type":"string"},"description":"footprint references of packages with an exposed pad whose row pads may also leave inward: a via in the ring between the pad row and the exposed pad, tried before the outward spots, only where the ring holds it with clearance and never in the straight way from a pad of the exposed pad's net to it; a row through one names it (inward_via_mm). 404 an unknown reference, 400 a part without an exposed pad inside its row"},"escapes":{"type":"boolean","default":true,"description":"the way out of each fine-pitch pad (as dense_first says) on the edge of its package is kept open while its connection waits, and so is that of such a pad of a net outside the call with an open connection (a later call routes it): room for a track and a via in front of it; a route of another net across it is refused (reason names the pad). false: no such room kept"},
                        "plan":{"type":"boolean","default":true,"description":"a connection the router does not find on a layer is planned over the free space (grid search with vias, the board's clearances), the router follows the plan; no plan found: no escape-spot tries either (row plan tells)"},
                        "max_tries":{"type":"integer","default":120,"minimum":1,"description":"router attempts per connection and pass (layers x modes x escape spots); a big board pays about a second per attempt"},
                        "style":{"type":"string","enum":["direct","rail"],"default":"direct","description":"rail (power nets): every connection leaves its pads through a via next to them and runs on the second layer"},
                        "step_ms":{"type":"integer","default":0,"description":"GUI: draw each route and wait this long before the next (for watching / recording)"},
                        "dry_run":{"type":"boolean","default":false,"description":"pns: the whole call (first pass, retries, rip-up, clean-up, stubs, refill) is routed on the board and taken back - rows and totals are what applying gives (409 when no copy of the board could be made); grid: the search and guard, nothing put on the board"}}})json"_json, ROUTE_LIST_CAP ),
                   false, h_pcb_route, 600 );


KOPENAPI_REGISTER( "pcb_stitch",
                   "Stitch a net (default: the one net pcb_net_roles calls ground; 400 when none or several) to its pours and planes: every open end on one side only (SMD "
                   "pad, zone island, the track end of pads joined by tracks) gets a through via at a free spot "
                   "next to it (with a short track from a pad or track end), guarded against touching other nets (other nets' fills give way: they are refilled), "
                   "the stub necked as the router does at a fine-pitch pad (no wider than leaves its neighbours the rules' clearance) and further "
                   "to what the clearances there leave (a via row's stub_width_mm / necked_from_mm / necked_why; necked_stubs counts them), "
                   "inside the board; after the refill a via the fill does not reach is taken out and its pad "
                   "tried at another spot (up to 3 passes); unlocked vias of the net left touching one layer "
                   "only are removed. On inner planes stitch before routing the signals (the spots next to the "
                   "pads are still free), with outer-layer pours both before and after; answers the vias, "
                   "dangling_vias_removed, not_reached (pads / islands left without a via, and why), islands_left (pieces of the "
                   "net's pours beside the main one per zone and layer, as island_list shows them: kind cut_off (no via) or sliver; "
                   "vias go only where the net's plane is on another layer with zone priorities resolved, and a via left on a "
                   "sliver after the refill (not the zone's largest piece on the layer, no other pad or via of the net on it) is taken out; footprint and board keep-outs are honoured; spots_off_plane) and the "
                   "connections still open (unrouted); a pad of a quad package's row first joins the package's exposed pad of its net by a stub "
                   "(joined: for, to), else its via goes inward first (under its own package, between the pad row and the exposed pad, "
                   "straight in or slanted from the pad's inner end; the row says inward; where the rules' via for the net does not fit there, the "
                   "board's smallest via within its rules: board_min_via, via_mm, count board_min_vias), an exposed pad's via goes in the pad, an island's "
                   "beside a coarse-pitch pad (a cap) before a fine-pitch pin; a net without a plane gets vias only at the fine-pitch pins of "
                   "quad packages, inward, kept as their way to the other layers for the router (port: true; the other ends are left "
                   "to the router, not_reached says so); no via or stub in the "
                   "straight escape of another net's fine-pitch, pair or clock pin (escape_keep_mm; spots_in_escapes), nor in a pair pin's lane on to "
                   "the nearest pad of its net on another part of its block (pcb_blocks; pair_lanes: pin, to; pair_lanes_none: pin, why); pads picks the ends; "
                   "dry_run stitches and takes it back; lists capped at list_limit (counts whole), full: true for all",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{"net":{"type":"string","description":"default: the one net pcb_net_roles calls ground (400 when none or several)"},
                        "pads":{"type":"array","items":{"type":"string"},"description":"only these ends: REF.PAD of the net (and the islands of tracks they are on); 400 for a pad not found or on another net"},
                        "escape_keep_mm":{"type":"number","minimum":0,"description":"no via or stub this far out in front of a fine-pitch (no track of the rules' width passes beside it at their clearance, or two of its vias do not fit side by side at the pitch), pair or clock pin of another net on a package's row; default past its part's courtyard by one via pitch (the rules' via and clearance), where its first via can go; 0 off; the answer's escape_keep is the value given or courtyard_via_pitch"},
                        "dry_run":{"type":"boolean","default":false,"description":"the whole call (vias, stubs, refills) runs on the board and is taken back: the answer is what it places (409 when no copy of the board could be made)"},
                        "under_parts":{"type":"string","enum":["avoid","allow"],"default":"avoid","description":"avoid: no via or stub of another part inside the ring of pads of a quad package (QFP / QFN / LGA) on its side - the ring is the package's own pins' (their vias go inward) and the exposed pad and its clearance stay open for its own vias; BGAs are not affected; not_reached says when only such spots were near. allow: other parts' vias may go there too"}}})json"_json ),
                   false, h_pcb_stitch, 300 );

KOPENAPI_REGISTER( "pcb_rail_plan",
                   "The vias pcb_stitch plans inward of the rail pins (power pins, pins of a poured net or of the exposed pad's net) "
                   "of a fine-pitch quad package's rows, without laying any: per pin the spot (at_mm), via and drill, the stub from "
                   "the pad, the pattern of its run (one_depth; alternating depths; every_other where two vias a pin apart do not fit, "
                   "poured nets first; fixed: a via joined to the pin already, kept, its neighbours planned around it) and why; "
                   "skipped pins are left to the router (a tight run, a spot too near a fixed via, a spot or stub in a rule area that "
                   "forbids it). A pin beside one of its own net joins that one's via (planned false). Sizes as pcb_stitch of net takes "
                   "them: the via KiCad's rules size for the net first (a via_diameter / hole_size rule's opt, else the net class's), the board's smallest within its rules where it does not fit. Read only",
                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string","description":"the package"},
                        "net":{"type":"string","description":"the net the rules size the vias for (as pcb_stitch net; default the one net pcb_net_roles calls ground)"}}})json"_json,
                   false, h_pcb_rail_plan, 60 );

KOPENAPI_REGISTER( "pcb_track_list",
                   "Tracks and vias with their uuids: all, or of nets, in a region, on a layer, or the copper "
                   "joining two pads (between); per item kind, net, ends (start_mm / end_mm, also as from_mm / to_mm) / "
                   "position, layer(s), width, length (vias in detail: pcb_via_list); "
                   "totals (count, vias, length). Paged: limit / cursor (offset: the same as a number); "
                   "full: every item (no limit / cursor) with net code, net class, "
                   "locked, dangling, a via's type and layer span, an arc's mid point",
                   R"json({"type":"object","properties":{
                        "net":{"type":"string"},"nets":{"type":"array","items":{"type":"string"}},
                        "between":{"type":"array","items":{"type":"string"},"description":"[\"REF.PAD\", \"REF.PAD\"]: the chain of tracks / vias joining these two pads of one net"},
                        "region":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm]"},
                        "region_mode":{"type":"string","enum":["inside","touching"],"default":"inside"},
                        "layer":{"type":"string"},
                        "kinds":{"type":"array","items":{"type":"string","enum":["track","via"]}},
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "limit":{"type":"integer","default":500,"description":"items per page, 1..5000 (ignored with full)"},
                        "cursor":{"type":"string","description":"next_cursor of the previous page"},
                        "offset":{"type":"integer","default":0,"description":"the same as cursor, as a number (next_offset)"},
                        "full":{"type":"boolean","default":false,"description":"everything: every item (no limit or cursor) with all fields: net_code, netclass, locked, dangling, clearance_mm / clearance_source (its own clearance as the clearance outline and the router use it, and the rule it comes from); vias: via_type, layer_span; arcs: mid_mm; large on a big design"}}})json"_json,
                   false, h_pcb_track_list );

KOPENAPI_REGISTER( "pcb_via_list",
                   "Vias of the board with their uuids: all, or of nets, in a region, through a layer, of a via type; "
                   "per via position (at_mm), net and net_code, diameter, drill, via_type (through / blind / buried / "
                   "micro), layer_span [top, bottom] and the copper layers it passes (layers), unused_layers (keep / remove / keep_outer: pcb_pads_unused_layers), locked; totals by type "
                   "and by size (diameter / drill: count). Paged: limit / cursor (offset: the same as a number), "
                   "full: true for all",
                   R"json({"type":"object","properties":{
                        "net":{"type":"string"},"nets":{"type":"array","items":{"type":"string"}},
                        "region":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm]"},
                        "region_mode":{"type":"string","enum":["inside","touching"],"default":"inside"},
                        "layer":{"type":"string","description":"vias passing this copper layer"},
                        "via_types":{"type":"array","items":{"type":"string","enum":["through","blind","buried","micro"]}},
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "full":{"type":"boolean","default":false,"description":"everything: every item (no page limit) and every entry of every list in the answer (no list_limit); large on a big design"},
                        "limit":{"type":"integer","default":1000},
                        "cursor":{"type":"string","description":"next_cursor of the previous page"},
                        "offset":{"type":"integer","default":0,"description":"the same as cursor, as a number (next_offset)"}}})json"_json,
                   false, h_pcb_via_list );

KOPENAPI_REGISTER( "pcb_tracks_delete",
                   "Remove tracks and vias in one undo step: by uuids, or the ones passing every filter given - "
                   "net / nets (unroute them), between two pads (undo one connection: the copper joining them that "
                   "no other pad of the net needs; the shared rest stays, kept_shared), region, layer, kinds. Pads, "
                   "zones and footprints are never touched; locked copper is kept (override_locks). dry_run tells "
                   "what would go. Answers counts (tracks, vias, length), per net (most first) and per layer, the "
                   "box, the first uuids (list_limit, full: true for all), pads_cut (pads that touch copper of "
                   "their net now and would touch none, e.g. a via sitting on a neighbouring pad; keep_pads: true "
                   "refuses then, 409) and the connections now open (unrouted); route again with pcb_route / "
                   "pcb_route_connection / pcb_track_add",
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
                        "keep_pads":{"type":"boolean","default":false,"description":"refuse (409, nothing deleted) when a pad would lose all copper of its net (pads_cut); the pads of between excepted"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json ),
                   false, h_pcb_tracks_delete, 300 );

KOPENAPI_REGISTER( "pcb_tracks_update",
                   "Change routed tracks and vias in one undo step, as KiCad's Edit Track & Via Properties: "
                   "width_mm (tracks and arcs), via_diameter_mm / via_drill_mm (vias), to_layer (tracks onto "
                   "another copper layer), or from_rules: each item to the size its design rules give it (net "
                   "class, custom rules; answer: rules used and how often). The items: the selectors of "
                   "pcb_tracks_delete (uuids, net / nets, all, between, region, layer, kinds). A guard refuses "
                   "(422, violations, nothing changed) when the changed copper would touch another net or enter "
                   "a keep-out; force: true applies anyway, as the dialog does. Locked copper is kept "
                   "(override_locks); dry_run tells what would change and the violations. Pour fills give way "
                   "and are refilled after (zones: keep leaves them). Answers counts, per net and per layer, "
                   "unchanged (already so), the first uuids (list_limit, full: true for all)",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "uuids":{"type":"array","items":{"type":"string"},"description":"exactly these (pcb_track_list); the filters below are ignored"},
                        "net":{"type":"string"},"nets":{"type":"array","items":{"type":"string"}},
                        "all":{"type":"boolean","description":"every track and via - with layer / kinds to narrow"},
                        "between":{"type":"array","items":{"type":"string"},"description":"[\"REF.PAD\", \"REF.PAD\"]: the copper joining them"},
                        "region":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm]"},
                        "region_mode":{"type":"string","enum":["inside","touching"],"default":"inside"},
                        "layer":{"type":"string","description":"select the items on this layer"},
                        "kinds":{"type":"array","items":{"type":"string","enum":["track","via"]}},
                        "width_mm":{"type":"number","description":"new width of the tracks"},
                        "via_diameter_mm":{"type":"number"},"via_drill_mm":{"type":"number"},
                        "to_layer":{"type":"string","description":"move the tracks to this copper layer"},
                        "from_rules":{"type":"boolean","default":false,"description":"sizes from the design rules (net class, custom rules), as the dialog's 'Set to net class / custom rule values'"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill"},
                        "force":{"type":"boolean","default":false,"description":"apply despite the guard's violations"},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json ),
                   false, h_pcb_tracks_update, 300 );

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
                   "Width and via size as KiCad's rules size them for the net (a track_width / via_diameter / hole_size rule's opt, else the net class's) unless given; without width_mm the "
                   "segments at the from / to pads are necked as in pcb_via_add (answer necked). The same guard as the "
                   "router: copper that would touch another net or enter a keep-out is refused (422 with the "
                   "violations), nothing is added then. One undo step; dry_run checks only",
                   R"json({"type":"object","properties":{
                        "from":{"type":"string","description":"REF.PAD: start at its centre, on its net"},
                        "to":{"type":"string","description":"REF.PAD of the same net: end at its centre"},
                        "net":{"type":"string","description":"needed without 'from'"},
                        "points":{"type":"array","description":"[x_mm, y_mm] (a corner), or {at: [x_mm, y_mm], via: 'B.Cu'}: a through via there and the track continues on that layer ('layer' is the same key, as in pcb_route waypoints)","items":{"oneOf":[{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},{"type":"object","required":["at"],"properties":{"at":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},"via":{"description":"copper layer to continue on, e.g. B.Cu"},"layer":{"description":"the same as via"}}}]}},
                        "layer":{"type":"string","description":"start layer; default F.Cu (B.Cu from a bottom SMD pad)"},
                        "width_mm":{"type":"number"},
                        "via_diameter_mm":{"type":"number","description":"via size of this call (default the rules' for the net)"},"via_drill_mm":{"type":"number"},
                        "zones":{"type":"string","enum":["refill","keep"],"default":"refill","description":"refill: pour fills of other nets give way (the guard does not count them) and are refilled around the track; keep: the track must clear the fills"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_track_add );

KOPENAPI_REGISTER( "pcb_via_add",
                   "Add a via: alone on a net at a point, or with a short track from a pad (from: REF.PAD); "
                   "size from the net class unless diameter_mm / drill_mm; the track at the class width unless "
                   "width_mm, necked at a fine-pitch pad as the router does (no wider than leaves its neighbours the rules' clearance, "
                   "a custom track_width rule there sets it lower) and further to what the clearances to the "
                   "neighbouring copper leave (not below the board / rule minimum): the answer's necked [{pad, "
                   "width_mm, necked_from_mm, why}] and note say so; the router's guard applies (422 with "
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
                   "(outer_rings_direct: default as many rings as the rules' tracks pass between two outer balls - ring n "
                   "needs n - 1 tracks in the gap at the clearances - plus the outer one, 0 when not even its tracks run "
                   "side by side at the pitch). Via size: given, or the one most used under the package when the rules "
                   "allow it, or the via the rules size for the net where it fits between four balls, else the largest "
                   "that fits (drill keeping the minimum ring); stub width the net class's (width_mm to set); the guard and the board's minimums "
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
                        "ports":{"type":"boolean","default":false,"description":"ring packages: follow the layout plan (pcb_fanout_ports): a via only for rails, for pins whose far end's order moves them to the other layer and for pins that yield to lanes (at the plan's via_mm, the route leaving from that end); the rest stubs out past the courtyard on the pad's layer; each pin answers its port (kind, layer, to)"},
                        "stubs":{"type":"boolean","default":true,"description":"with ports: false lays only the planned vias (rails, pins to the other layer); the other pins are left to pcb_route"},
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

KOPENAPI_MARK_EDITING( "pcb_route_connection", "pcb_route", "pcb_stitch", "pcb_tracks_delete", "pcb_tracks_update", "pcb_tracks_cleanup", "pcb_track_add", "pcb_via_add", "pcb_fanout" );
KOPENAPI_MARK_CONCURRENT_READS( "pcb_track_list", "pcb_via_list" );
