/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * pcb_route_oracle: whether a connection can be routed at all on the board as it is, decided apart from
 * the router. Two grids over the whole board bound the free space a track's centre (and a via's) may use
 * under KiCad's rules: in the optimistic one a cell is shut only when every point of it is too near
 * other copper, so no path there proves no route exists; in the pessimistic one a cell is open only
 * when every point of it is clear, so a path there is a route (its corners are cell centres).
 */

#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <string_utils.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <drc/drc_engine.h>
#include <drc/drc_rule.h>
#include <footprint.h>
#include <convert_basic_shapes_to_polygon.h>
#include <geometry/approximation.h>
#include <geometry/corner_strategy.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_segment.h>
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_util.h>
#include <pad.h>
#include <pcb_track.h>
#include <zone.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <functional>
#include <queue>
#include <set>


namespace
{

/// A min-queue of (cost, state) that pops in the same order as a binary heap of the pairs. Entries go to
/// buckets by the leading bits of the cost (monotone in a non-negative float), and only the lowest bucket
/// in use is kept as a heap, so most pushes are appends.
class COST_QUEUE
{
public:
    using ENTRY = std::pair<float, uint32_t>;

    COST_QUEUE() : m_buckets( LAST + 1 ) {}

    bool empty() const { return m_size == 0; }

    void push( float aCost, uint32_t aState )
    {
        const uint32_t b = bucket( aCost );

        if( m_size == 0 )
            m_cur = m_hi = b;

        if( b <= m_cur )
        {
            m_heap.emplace_back( aCost, aState );
            std::push_heap( m_heap.begin(), m_heap.end(), std::greater<ENTRY>() );
        }
        else
        {
            m_buckets[b].emplace_back( aCost, aState );
            m_hi = std::max( m_hi, b );
        }

        m_size++;
    }

    ENTRY pop()
    {
        if( m_heap.empty() )
        {
            while( m_buckets[++m_cur].empty() )
                ;

            m_heap.swap( m_buckets[m_cur] );
            std::make_heap( m_heap.begin(), m_heap.end(), std::greater<ENTRY>() );
        }

        std::pop_heap( m_heap.begin(), m_heap.end(), std::greater<ENTRY>() );
        const ENTRY e = m_heap.back();
        m_heap.pop_back();
        m_size--;
        return e;
    }

    void clear()
    {
        if( m_size )
        {
            for( uint32_t b = m_cur + 1; b <= m_hi; ++b )
                m_buckets[b].clear();
        }

        m_heap.clear();
        m_size = 0;
    }

private:
    static constexpr int      SHIFT = 12;
    static constexpr uint32_t LAST = 0x7F800000u >> SHIFT;

    static uint32_t bucket( float aCost )
    {
        uint32_t bits;
        std::memcpy( &bits, &aCost, sizeof( bits ) );
        return std::min( bits >> SHIFT, LAST );
    }

    std::vector<std::vector<ENTRY>> m_buckets;
    std::vector<ENTRY>              m_heap;
    size_t                          m_size = 0;
    uint32_t                        m_cur = 0;
    uint32_t                        m_hi = 0;
};

enum : uint8_t
{
    OPT = 1,          ///< shut in the optimistic grid: every point of the cell is too near
    PES = 2,          ///< shut in the pessimistic grid: some point of the cell is too near
    START_OPT = 4,    ///< the ends' cells, per grid
    GOAL_OPT = 8,
    START_PES = 16,
    GOAL_PES = 32
};


struct ORACLE_GRID
{
    VECTOR2I origin;
    int      cell = 0;
    int      nx = 0, ny = 0;

    size_t   Cells() const { return (size_t) nx * ny; }
    VECTOR2I Centre( int aX, int aY ) const { return origin + VECTOR2I( aX * cell + cell / 2, aY * cell + cell / 2 ); }

    /// Calls aFn( index ) for each cell whose centre lies inside aPoly (even-odd per polygon of the set)
    template <typename FN>
    void Fill( const SHAPE_POLY_SET& aPoly, FN aFn ) const
    {
        for( int p = 0; p < aPoly.OutlineCount(); ++p )
        {
            const SHAPE_POLY_SET::POLYGON& poly = aPoly.CPolygon( p );
            const BOX2I                    box = poly[0].BBox();
            const int y0 = std::max( 0, ( box.GetTop() - origin.y ) / cell - 1 );
            const int y1 = std::min( ny - 1, ( box.GetBottom() - origin.y ) / cell + 1 );

            std::vector<std::pair<VECTOR2I, VECTOR2I>> edges;

            for( const SHAPE_LINE_CHAIN& chain : poly )
            {
                for( int i = 0; i < chain.PointCount(); ++i )
                    edges.emplace_back( chain.CPoint( i ), chain.CPoint( ( i + 1 ) % chain.PointCount() ) );
            }

            std::vector<double> xs;

            for( int y = y0; y <= y1; ++y )
            {
                const double cy = origin.y + (double) y * cell + cell / 2.0;
                xs.clear();

                for( const auto& [a, b] : edges )
                {
                    if( ( a.y <= cy ) != ( b.y <= cy ) )
                        xs.push_back( a.x + ( cy - a.y ) * double( b.x - a.x ) / double( b.y - a.y ) );
                }

                std::sort( xs.begin(), xs.end() );

                for( size_t i = 0; i + 1 < xs.size(); i += 2 )
                {
                    const int xa = std::max( 0, (int) std::ceil( ( xs[i] - origin.x - cell / 2.0 ) / cell ) );
                    const int xb = std::min( nx - 1, (int) std::floor( ( xs[i + 1] - origin.x - cell / 2.0 ) / cell ) );

                    for( int x = xa; x <= xb; ++x )
                        aFn( (size_t) y * nx + x );
                }
            }
        }
    }
};


std::string str( const wxString& aText )
{
    return std::string( aText.ToUTF8() );
}


std::string itemName( BOARD_ITEM* aItem )
{
    if( aItem->Type() == PCB_PAD_T )
    {
        PAD* pad = static_cast<PAD*>( aItem );
        return ( pad->GetParentFootprint() ? str( pad->GetParentFootprint()->GetReference() ) : std::string( "?" ) ) + "."
               + str( pad->GetNumber() );
    }

    if( aItem->Type() == PCB_VIA_T )
        return "via";

    if( aItem->Type() == PCB_ZONE_T )
        return "zone";

    if( aItem->Type() == PCB_FOOTPRINT_T )
        return "hole of " + str( static_cast<FOOTPRINT*>( aItem )->GetReference() );

    return "track";
}


struct OBSTACLE
{
    BOARD_ITEM*  item;
    PCB_LAYER_ID layer;     // UNDEFINED_LAYER: a hole (every layer)
    int          trackReach;
    int          viaReach;
    bool         anyNet = false;    // shuts every net, its own too
};


/// A track and a through via whose room the grids bound; the clearances are the ones KiCad's rules give
/// a probe of this net
struct ORACLE_PROBE
{
    int  net = 0;
    int  width = 0;
    int  viaDia = 0;
    int  viaDrill = 0;
    bool vias = false;
};


/// The board's obstacles to aProbe's track centre and via centre, rasterised on aGrid in aModes (OPT: under
/// the true shape by the cell's reach, PES: over it). aMark( owner, layer, cell, mode ) per shut cell:
/// owner the net of the copper that shuts it (-1: shut for every net), layer an index in aLayers or -1 for
/// the via grid. Copper of aSkipNet is left out (hole to hole is not). aZonesYield: zone copper shuts no
/// via cell (a refill clears round a new via). Returns the obstacles.
template <typename FN>
std::vector<OBSTACLE> rasteriseObstacles( BOARD* aBoard, const std::vector<PCB_LAYER_ID>& aLayers, const ORACLE_GRID& aGrid,
                                          const SHAPE_POLY_SET& aOutline, int aDiag, int aMaxError, const ORACLE_PROBE& aProbe,
                                          int aSkipNet, const std::vector<uint8_t>& aModes, bool aZonesYield, FN aMark )
{
    const BOARD_DESIGN_SETTINGS& ds = aBoard->GetDesignSettings();
    std::shared_ptr<DRC_ENGINE>  drc = ds.m_DRCEngine;
    const int                    L = (int) aLayers.size();
    const size_t                 cells = aGrid.Cells();
    const int                    halfWidth = aProbe.width / 2, viaRadius = aProbe.viaDia / 2;
    const bool                   vias = aProbe.vias;

    // the board edge: centres within the edge clearance (+ the half track / via) of the outline, or outside
    {
        const int edgeClr = std::max( ds.m_CopperEdgeClearance, 0 );

        for( const auto& [reach, isVia] : { std::pair( edgeClr + halfWidth, false ), std::pair( edgeClr + viaRadius, true ) } )
        {
            if( isVia && !vias )
                continue;

            for( const uint8_t mode : aModes )
            {
                SHAPE_POLY_SET inside = aOutline;
                const int      by = mode == OPT ? reach - aDiag : reach + aDiag;

                if( by > 0 )
                    inside.Deflate( by, CORNER_STRATEGY::ROUND_ALL_CORNERS, aMaxError );
                else if( by < 0 )
                    inside.Inflate( -by, CORNER_STRATEGY::ROUND_ALL_CORNERS, aMaxError );

                std::vector<uint8_t> in( cells, 0 );
                aGrid.Fill( inside, [&]( size_t i ) { in[i] = 1; } );

                for( size_t i = 0; i < cells; ++i )
                {
                    if( in[i] )
                        continue;

                    if( isVia )
                    {
                        aMark( -1, -1, i, mode );
                    }
                    else
                    {
                        for( int l = 0; l < L; ++l )
                            aMark( -1, l, i, mode );
                    }
                }
            }
        }
    }

    PCB_TRACK probeTrack( aBoard );
    probeTrack.SetNetCode( aProbe.net );
    probeTrack.SetWidth( std::max( aProbe.width, 1 ) );

    PCB_VIA probeVia( aBoard );
    probeVia.SetNetCode( aProbe.net );
    probeVia.SetViaType( VIATYPE::THROUGH );
    probeVia.SetLayerPair( F_Cu, B_Cu );
    probeVia.SetWidth( PADSTACK::ALL_LAYERS, std::max( aProbe.viaDia, 1 ) );
    probeVia.SetDrill( std::max( aProbe.viaDrill, 1 ) );

    auto clearance = [&]( BOARD_ITEM* aProbeItem, BOARD_ITEM* aItem, PCB_LAYER_ID aLayer, DRC_CONSTRAINT_T aType, int aFloor )
    {
        int c = aFloor;

        if( drc )
        {
            const VECTOR2I at = aItem->GetBoundingBox().Centre();

            if( aProbeItem == &probeTrack )
            {
                probeTrack.SetStart( at );
                probeTrack.SetEnd( at );
                probeTrack.SetLayer( aLayer );
            }
            else
            {
                probeVia.SetPosition( at );
            }

            DRC_CONSTRAINT rule = drc->EvalRules( aType, aProbeItem, aItem, aLayer );

            if( rule.GetValue().HasMin() )
                c = std::max( c, rule.GetValue().Min() );
        }

        return c;
    };

    auto ownerOf = [&]( BOARD_ITEM* aItem )
    {
        BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( aItem );
        return bci && bci->GetNetCode() > 0 ? bci->GetNetCode() : -1;
    };

    // an item's copper on a layer, grown by aReach, under (OPT) or over (PES) the true shape
    auto mark = [&]( int aOwner, int aLayer, int aReach, const std::function<void( SHAPE_POLY_SET&, int, ERROR_LOC )>& aShape )
    {
        for( const uint8_t mode : aModes )
        {
            const int      grow = mode == OPT ? aReach - aDiag : aReach + aDiag;
            SHAPE_POLY_SET poly;
            aShape( poly, std::max( grow, 0 ), mode == OPT ? ERROR_INSIDE : ERROR_OUTSIDE );

            if( grow < 0 )
                poly.Deflate( -grow, CORNER_STRATEGY::ROUND_ALL_CORNERS, aMaxError );

            aGrid.Fill( poly, [&]( size_t i ) { aMark( aOwner, aLayer, i, mode ); } );
        }
    };

    std::vector<OBSTACLE> obstacles;

    // an item farther from the grid than any reach marks no cell of it
    BOX2I reachBox( aGrid.origin, VECTOR2I( (int64_t) aGrid.nx * aGrid.cell, (int64_t) aGrid.ny * aGrid.cell ) );
    reachBox.Inflate( ds.GetBiggestClearanceValue() + std::max( halfWidth, viaRadius ) + aProbe.viaDrill + 2 * aDiag
                      + pcbIUScale.mmToIU( 0.1 ) );

    auto far = [&]( BOARD_ITEM* aItem ) { return !aItem->GetBoundingBox().Intersects( reachBox ); };

    auto addCopper = [&]( BOARD_CONNECTED_ITEM* aItem )
    {
        if( ( aItem->GetNetCode() == aSkipNet && aSkipNet > 0 ) || far( aItem ) )
            return;

        for( int l = 0; l < L; ++l )
        {
            const PCB_LAYER_ID layer = aLayers[l];

            if( !aItem->IsOnLayer( layer ) )
                continue;

            if( aItem->Type() == PCB_PAD_T && !static_cast<PAD*>( aItem )->FlashLayer( layer ) )
                continue;

            if( aItem->Type() == PCB_VIA_T && !static_cast<PCB_VIA*>( aItem )->FlashLayer( layer ) )
                continue;

            const int clr = clearance( &probeTrack, aItem, layer, CLEARANCE_CONSTRAINT, ds.m_MinClearance );
            int viaReach = 0;

            if( vias )
            {
                // the via's ring by copper clearance, its hole by hole clearance
                const int vclr = clearance( &probeVia, aItem, layer, CLEARANCE_CONSTRAINT, ds.m_MinClearance );
                const int hclr = clearance( &probeVia, aItem, layer, HOLE_CLEARANCE_CONSTRAINT, ds.m_HoleClearance );
                viaReach = std::max( vclr + viaRadius, hclr + aProbe.viaDrill / 2 );
            }

            obstacles.push_back( { aItem, layer, clr + halfWidth, viaReach } );
        }
    };

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            addCopper( pad );

            // no via in a pad without a hole, of its own net neither
            if( vias && !pad->HasHole() && ( aSkipNet <= 0 || pad->GetNetCode() == aSkipNet ) )
            {
                for( const PCB_LAYER_ID layer : aLayers )
                {
                    if( pad->IsOnLayer( layer ) && pad->FlashLayer( layer ) )
                        obstacles.push_back( { pad, layer, -1, viaRadius, true } );
                }
            }

            if( pad->HasHole() && !far( pad ) )
                obstacles.push_back( { pad, UNDEFINED_LAYER, 0, 0 } );
        }
    }

    for( PCB_TRACK* t : aBoard->Tracks() )
    {
        addCopper( t );

        if( t->Type() == PCB_VIA_T && !far( t ) )
            obstacles.push_back( { t, UNDEFINED_LAYER, 0, 0 } );
    }

    std::vector<ZONE*> zones( aBoard->Zones().begin(), aBoard->Zones().end() );

    for( FOOTPRINT* fp : aBoard->Footprints() )
        zones.insert( zones.end(), fp->Zones().begin(), fp->Zones().end() );

    for( ZONE* zone : zones )
    {
        if( zone->GetIsRuleArea() )
        {
            if( far( zone ) )
                continue;

            for( int l = 0; l < L; ++l )
            {
                if( zone->IsOnLayer( aLayers[l] ) )
                    obstacles.push_back( { zone, aLayers[l], zone->GetDoNotAllowTracks() ? halfWidth : -1,
                                           zone->GetDoNotAllowVias() ? viaRadius : -1 } );
            }
        }
        else
        {
            addCopper( zone );
        }
    }

    for( OBSTACLE& o : obstacles )
    {
        if( o.layer == UNDEFINED_LAYER )
        {
            // a hole: the hole clearance to a track on every layer, hole to hole to a via
            std::shared_ptr<SHAPE_SEGMENT> hole = o.item->Type() == PCB_PAD_T
                                                          ? static_cast<PAD*>( o.item )->GetEffectiveHoleShape()
                                                          : static_cast<PCB_VIA*>( o.item )->GetEffectiveHoleShape();

            if( !hole )
                continue;

            auto shape = [&]( SHAPE_POLY_SET& aPoly, int aGrow, ERROR_LOC aLoc )
            {
                TransformOvalToPolygon( aPoly, hole->GetSeg().A, hole->GetSeg().B, hole->GetWidth() + 2 * aGrow, aMaxError, aLoc );
            };

            const int owner = ownerOf( o.item );

            if( !( owner == aSkipNet && aSkipNet > 0 ) )
            {
                const int hc = clearance( &probeTrack, o.item, aLayers[0], HOLE_CLEARANCE_CONSTRAINT, ds.m_HoleClearance );
                o.trackReach = hc + halfWidth;

                for( int l = 0; l < L; ++l )
                    mark( owner, l, hc + halfWidth, shape );
            }

            if( vias )
            {
                // hole to hole for any net; another net's hole also keeps hole clearance from the via's ring
                const int h2h = clearance( &probeVia, o.item, aLayers[0], HOLE_TO_HOLE_CONSTRAINT, ds.m_HoleToHoleMin );
                o.viaReach = h2h + aProbe.viaDrill / 2;
                mark( -1, -1, h2h + aProbe.viaDrill / 2, shape );

                if( !( owner == aSkipNet && aSkipNet > 0 ) )
                {
                    const int hv = clearance( &probeVia, o.item, aLayers[0], HOLE_CLEARANCE_CONSTRAINT, ds.m_HoleClearance );
                    o.viaReach = std::max( o.viaReach, hv + aProbe.viaDia / 2 );
                    mark( owner, -1, hv + aProbe.viaDia / 2, shape );
                }
            }

            continue;
        }

        const int l = (int) ( std::find( aLayers.begin(), aLayers.end(), o.layer ) - aLayers.begin() );
        const bool ruleArea = o.item->Type() == PCB_ZONE_T && static_cast<ZONE*>( o.item )->GetIsRuleArea();

        auto shape = [&]( SHAPE_POLY_SET& aPoly, int aGrow, ERROR_LOC aLoc )
        {
            if( ruleArea )
            {
                aPoly = static_cast<ZONE*>( o.item )->GetBoardOutline();

                if( aGrow > 0 )
                    aPoly.Inflate( aGrow, CORNER_STRATEGY::ROUND_ALL_CORNERS, aMaxError );

                return;
            }

            o.item->TransformShapeToPolygon( aPoly, o.layer, aGrow, aMaxError, aLoc );
        };

        const int owner = ruleArea || o.anyNet ? -1 : ownerOf( o.item );

        if( o.trackReach >= 0 )
            mark( owner, l, o.trackReach, shape );

        if( vias && o.viaReach >= 0 && !( aZonesYield && !ruleArea && o.item->Type() == PCB_ZONE_T ) )
            mark( owner, -1, o.viaReach, shape );
    }

    return obstacles;
}

} // namespace


/// A finer search over a window of a connection the whole-board grids leave undecided: the coarse grid
/// and its pessimistic rooms of the start and of the goal (per state) seed the window's ends; path: the
/// window's path, corners and layer indices, when found
struct ORACLE_WINDOW
{
    BOX2I                                         box;
    int                                           cell = 0;
    ORACLE_GRID                                   coarse;
    const std::vector<uint8_t>*                   startRoom = nullptr;
    const std::vector<uint8_t>*                   goalRoom = nullptr;
    mutable std::vector<std::pair<VECTOR2I, int>> path;
};


static KOPENAPI_RESULT oracleRun( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, const ORACLE_WINDOW* aWindow )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

    const auto started = std::chrono::steady_clock::now();

    auto padOf = [&]( const std::string& aName ) -> PAD*
    {
        const size_t dot = aName.rfind( '.' );

        if( dot == std::string::npos )
            return nullptr;

        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( str( fp->GetReference() ) != aName.substr( 0, dot ) )
                continue;

            for( PAD* pad : fp->Pads() )
            {
                if( str( pad->GetNumber() ) == aName.substr( dot + 1 ) )
                    return pad;
            }
        }

        return nullptr;
    };

    std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
    PAD* from = padOf( aArgs.value( "from", std::string() ) );
    PAD* to = aArgs.contains( "to" ) ? padOf( aArgs.value( "to", std::string() ) ) : nullptr;

    // a net alone: its first pad whose island is not the whole net
    if( !from && aArgs.contains( "net" ) && !aArgs.contains( "from" ) )
    {
        const NETINFO_ITEM* info = KopenapiFindNet( board, aArgs["net"].get<std::string>() );

        if( !info || info->GetNetCode() <= 0 )
            return KOPENAPI_RESULT::Error( 404, "no net " + aArgs["net"].get<std::string>() );

        const std::vector<BOARD_CONNECTED_ITEM*> netItems =
                conn->GetNetItems( info->GetNetCode(), { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T } );

        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
            {
                if( from || pad->GetNetCode() != info->GetNetCode() )
                    continue;

                std::set<const BOARD_CONNECTED_ITEM*> island = { pad };

                for( BOARD_CONNECTED_ITEM* item : conn->GetConnectedItems( pad ) )
                    island.insert( item );

                for( BOARD_CONNECTED_ITEM* item : netItems )
                {
                    if( !island.count( item ) )
                    {
                        from = pad;
                        break;
                    }
                }
            }
        }

        if( !from )
            return KOPENAPI_RESULT::Ok( { { "verdict", "connected" }, { "reason", "the net is one island" } } );
    }

    if( !from || ( aArgs.contains( "to" ) && !to ) )
        return KOPENAPI_RESULT::Error( 404, "from / to: pads as REF.NUMBER (or net alone)" );

    if( from->GetNetCode() <= 0 || ( to && from->GetNetCode() != to->GetNetCode() ) )
        return KOPENAPI_RESULT::Error( 400, "from and to must be pads of the same net" );

    const int                    net = from->GetNetCode();
    const LSET                   copper = LSET::AllCuMask( board->GetCopperLayerCount() );
    const std::vector<PCB_LAYER_ID> layers = copper.Seq();
    const int                    L = (int) layers.size();
    // layers the path may use: the rest stay obstacles to a via, shut to a track
    std::vector<bool>            shutLayer( L, false );

    if( aArgs.contains( "layers" ) )
    {
        int open = 0;

        for( int l = 0; l < L; ++l )
        {
            bool asked = false;

            for( const nlohmann::json& name : aArgs["layers"] )
                asked |= name.is_string() && board->GetLayerID( wxString::FromUTF8( name.get<std::string>() ) ) == layers[l];

            shutLayer[l] = !asked;
            open += asked;
        }

        if( !open )
            return KOPENAPI_RESULT::Error( 400, "layers: copper layers of the board" );
    }

    const KOPENAPI_RULE_TRACK rt = KopenapiRuleTrack( board, net, from->GetPosition(), from->GetPrincipalLayer() );
    const KOPENAPI_RULE_VIA   rv = KopenapiRuleVia( board, net, from->GetPosition(), from->GetPrincipalLayer() );
    const int width = aArgs.contains( "width_mm" ) ? pcbIUScale.mmToIU( aArgs["width_mm"].get<double>() ) : rt.min;
    const int viaDia = aArgs.contains( "via_mm" ) ? pcbIUScale.mmToIU( aArgs["via_mm"].get<double>() ) : rv.dia;
    const int viaDrill = aArgs.contains( "drill_mm" ) ? pcbIUScale.mmToIU( aArgs["drill_mm"].get<double>() ) : rv.drill;
    const bool vias = aArgs.value( "vias", true ) && L > 1;

    if( width <= 0 )
        return KOPENAPI_RESULT::Error( 400, "width_mm > 0" );

    SHAPE_POLY_SET outline;

    if( !board->GetBoardPolygonOutlines( outline, true ) || outline.OutlineCount() == 0 )
        return KOPENAPI_RESULT::Error( 422, "the board has no outline" );

    // the grid over the whole board: the asked cell, coarser when it would pass the cell budget
    ORACLE_GRID grid;
    const BOX2I edges = aWindow ? aWindow->box : outline.BBox();
    grid.cell = aWindow ? aWindow->cell : pcbIUScale.mmToIU( std::max( 0.005, aArgs.value( "cell_mm", 0.025 ) ) );
    const double budget = aArgs.value( "max_cells_m", 20.0 ) * 1e6;

    while( (double) edges.GetWidth() / grid.cell * edges.GetHeight() / grid.cell > budget )
        grid.cell = grid.cell * 5 / 4 + 1;

    grid.origin = edges.GetOrigin();
    grid.nx = edges.GetWidth() / grid.cell + 1;
    grid.ny = edges.GetHeight() / grid.cell + 1;

    if( aWindow && grid.cell >= aWindow->coarse.cell )
        return KOPENAPI_RESULT::Ok( { { "verdict", "unknown" } } );

    const size_t cells = grid.Cells();
    const int    maxError = std::max( 1, grid.cell / 8 );
    // a cell's centre to its farthest point, and the polygons' error on top
    const int    diag = (int) std::ceil( grid.cell * M_SQRT1_2 ) + maxError + 1;

    std::vector<uint8_t> track( cells * L, 0 );   // per layer: a track's centre
    std::vector<uint8_t> via( cells, 0 );         // a through via's centre

    // the islands the connection joins, and the net's other copper: none of them is in the way
    std::set<const BOARD_ITEM*> islandA, islandB;

    islandA.insert( from );

    for( BOARD_CONNECTED_ITEM* item : conn->GetConnectedItems( from ) )
        islandA.insert( item );

    if( to )
    {
        islandB.insert( to );

        for( BOARD_CONNECTED_ITEM* item : conn->GetConnectedItems( to ) )
            islandB.insert( item );
    }
    else
    {
        // no target named: any copper of the net off the start's island
        for( BOARD_CONNECTED_ITEM* item : conn->GetNetItems( net, { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T } ) )
        {
            if( !islandA.count( item ) )
                islandB.insert( item );
        }

        if( islandB.empty() )
            return KOPENAPI_RESULT::Ok( { { "verdict", "connected" }, { "reason", "the net is one island" } } );
    }

    if( to && islandA.count( to ) )
        return KOPENAPI_RESULT::Ok( { { "verdict", "connected" }, { "reason", "from and to are joined already" } } );

    ORACLE_PROBE probe;
    probe.net = net;
    probe.width = width;
    probe.viaDia = viaDia;
    probe.viaDrill = viaDrill;
    probe.vias = vias;

    const std::vector<OBSTACLE> obstacles = rasteriseObstacles( board, layers, grid, outline, diag, maxError, probe, net, { OPT, PES },
                                                                aArgs.value( "zones_yield", true ),
            [&]( int, int aLayer, size_t aCell, uint8_t aMode )
            {
                if( aLayer < 0 )
                    via[aCell] |= aMode;
                else
                    track[(size_t) aLayer * cells + aCell] |= aMode;
            } );

    for( int l = 0; l < L; ++l )
    {
        if( shutLayer[l] )
            std::fill( track.begin() + (size_t) l * cells, track.begin() + (size_t) ( l + 1 ) * cells, (uint8_t) ( OPT | PES ) );
    }

    // boxes no via may overlap (a chip's ring of pins the router keeps for its pins' ways out)
    for( const nlohmann::json& b : aArgs.value( "no_via_mm", nlohmann::json::array() ) )
    {
        if( !b.is_array() || b.size() != 4 )
            continue;

        const VECTOR2I a( pcbIUScale.mmToIU( b[0].get<double>() ), pcbIUScale.mmToIU( b[1].get<double>() ) );
        const VECTOR2I c( pcbIUScale.mmToIU( b[2].get<double>() ), pcbIUScale.mmToIU( b[3].get<double>() ) );
        BOX2I          box( a, c - a );
        box.Normalize();

        for( const uint8_t mode : { OPT, PES } )
        {
            BOX2I grown = box;
            grown.Inflate( mode == PES ? viaDia / 2 + diag : std::max( viaDia / 2 - diag, 0 ) );
            SHAPE_POLY_SET poly;
            poly.NewOutline();
            poly.Append( grown.GetLeft(), grown.GetTop() );
            poly.Append( grown.GetRight(), grown.GetTop() );
            poly.Append( grown.GetRight(), grown.GetBottom() );
            poly.Append( grown.GetLeft(), grown.GetBottom() );
            grid.Fill( poly, [&]( size_t i ) { via[i] |= mode; } );
        }
    }

    // ways out kept open for other pads: segments [layer, a, b, width, clearance] no copper of this net comes near
    for( const nlohmann::json& w : aArgs.value( "kept_ways", nlohmann::json::array() ) )
    {
        if( !w.is_object() || !w.contains( "a" ) || !w.contains( "b" ) )
            continue;

        const int l = (int) ( std::find_if( layers.begin(), layers.end(),
                                            [&]( PCB_LAYER_ID aId ) { return str( board->GetLayerName( aId ) ) == w.value( "layer", std::string() ); } )
                              - layers.begin() );

        if( l >= L )
            continue;

        const VECTOR2I a( pcbIUScale.mmToIU( w["a"][0].get<double>() ), pcbIUScale.mmToIU( w["a"][1].get<double>() ) );
        const VECTOR2I b( pcbIUScale.mmToIU( w["b"][0].get<double>() ), pcbIUScale.mmToIU( w["b"][1].get<double>() ) );
        const int      half = pcbIUScale.mmToIU( w.value( "width_mm", 0.0 ) ) / 2;
        const int      clr = pcbIUScale.mmToIU( w.value( "clearance_mm", 0.0 ) );

        for( const uint8_t mode : { OPT, PES } )
        {
            const int by = mode == PES ? diag : -diag;

            for( const auto& [reach, isVia] : { std::pair( half + clr + width / 2, false ), std::pair( half + clr + viaDia / 2, true ) } )
            {
                if( isVia && !vias )
                    continue;

                SHAPE_POLY_SET poly;
                TransformOvalToPolygon( poly, a, b, 2 * std::max( reach + by, 1 ), maxError, mode == PES ? ERROR_OUTSIDE : ERROR_INSIDE );

                if( isVia )
                    grid.Fill( poly, [&]( size_t i ) { via[i] |= mode; } );
                else
                    grid.Fill( poly, [&]( size_t i ) { track[(size_t) l * cells + i] |= mode; } );
            }
        }
    }

    // the ends: cells whose centre lies on the island's copper on a layer (optimistic: within the cell's
    // reach of it; pessimistic: inside it by that reach, so a track from that centre touches it)
    auto seeds = [&]( const std::set<const BOARD_ITEM*>& aIsland, uint8_t aMode, uint8_t aBit )
    {
        for( const BOARD_ITEM* item : aIsland )
        {
            if( item->Type() != PCB_PAD_T && item->Type() != PCB_TRACE_T && item->Type() != PCB_ARC_T && item->Type() != PCB_VIA_T )
                continue;

            for( int l = 0; l < L; ++l )
            {
                if( !item->IsOnLayer( layers[l] ) )
                    continue;

                SHAPE_POLY_SET poly;
                item->TransformShapeToPolygon( poly, layers[l], aMode == OPT ? diag : 0, maxError, aMode == OPT ? ERROR_OUTSIDE : ERROR_INSIDE );

                if( aMode == PES )
                    poly.Deflate( diag, CORNER_STRATEGY::ROUND_ALL_CORNERS, maxError );

                grid.Fill( poly, [&]( size_t i ) { track[(size_t) l * cells + i] |= aBit; } );
            }
        }
    };

    // a search from aStart's cells to aGoal's: 8 neighbours on a layer, a via where its cell is open
    struct SEARCH
    {
        bool                 found = false;
        size_t               reached = 0;
        size_t               goalState = 0;
        std::vector<uint8_t> from;   // 0: not reached; 1..8 the step in; 10 a seed; 11 + layer: a via from that layer
    };

    const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    const int DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

    // aFewestVias: in rounds, round k a breadth-first search on the layers from the states k vias reach,
    // so the path found has the fewest vias, the shortest way per round. aViaCost >= 0: the least cost,
    // a straight step 5, a diagonal 7, a via aViaCost (a bucket queue)
    auto search = [&]( uint8_t aMode, uint8_t aStart, uint8_t aGoal, bool aFewestVias = false, int aViaCost = -1 )
    {
        SEARCH             s;
        std::deque<size_t> open;
        std::vector<std::pair<size_t, uint8_t>> next;   // a via's far state, its step in
        s.from.assign( cells * L, 0 );

        if( aViaCost >= 0 )
        {
            const uint32_t                   viaStep = (uint32_t) std::max( aViaCost, 1 );
            const size_t                     ring = std::max<size_t>( 7, viaStep ) + 1;
            std::vector<uint32_t>            dist( cells * L, UINT32_MAX );
            std::vector<std::vector<size_t>> buckets( ring );
            size_t                           pending = 0;

            for( size_t k = 0; k < cells * L; ++k )
            {
                if( ( track[k] & aStart ) && !( track[k] & aMode ) )
                {
                    dist[k] = 0;
                    s.from[k] = 10;
                    buckets[0].push_back( k );
                    pending++;
                }
            }

            auto relax = [&]( size_t aState, uint32_t aCost, uint8_t aStep )
            {
                if( aCost < dist[aState] && s.from[aState] != 10 )
                {
                    dist[aState] = aCost;
                    s.from[aState] = aStep;
                    buckets[aCost % ring].push_back( aState );
                    pending++;
                }
            };

            for( uint32_t cost = 0; pending; ++cost )
            {
                std::vector<size_t>& bucket = buckets[cost % ring];

                while( !bucket.empty() )
                {
                    const size_t k = bucket.back();
                    bucket.pop_back();
                    pending--;

                    if( dist[k] != cost )
                        continue;

                    s.reached++;

                    if( track[k] & aGoal )
                    {
                        s.found = true;
                        s.goalState = k;
                        return s;
                    }

                    const int    l = (int) ( k / cells );
                    const size_t i = k % cells;
                    const int    x = (int) ( i % grid.nx ), y = (int) ( i / grid.nx );

                    for( int d = 0; d < 8; ++d )
                    {
                        const int nx = x + DX[d], ny = y + DY[d];

                        if( nx < 0 || ny < 0 || nx >= grid.nx || ny >= grid.ny )
                            continue;

                        const size_t n = (size_t) l * cells + (size_t) ny * grid.nx + nx;

                        if( !( track[n] & aMode ) )
                            relax( n, cost + ( d < 4 ? 5 : 7 ), (uint8_t) ( d + 1 ) );
                    }

                    if( vias && !( via[i] & aMode ) )
                    {
                        for( int m = 0; m < L; ++m )
                        {
                            const size_t n = (size_t) m * cells + i;

                            if( m != l && !( track[n] & aMode ) )
                                relax( n, cost + viaStep, (uint8_t) ( 11 + l ) );
                        }
                    }
                }
            }

            return s;
        }

        for( size_t k = 0; k < cells * L; ++k )
        {
            if( ( track[k] & aStart ) && !( track[k] & aMode ) )
            {
                s.from[k] = 10;
                open.push_back( k );
            }
        }

        while( !open.empty() || !next.empty() )
        {
            if( open.empty() )
            {
                for( const auto& [n, step] : next )
                {
                    if( !s.from[n] )
                    {
                        s.from[n] = step;
                        open.push_back( n );
                    }
                }

                next.clear();
                continue;
            }

            const size_t k = open.front();
            open.pop_front();
            s.reached++;

            if( track[k] & aGoal )
            {
                s.found = true;
                s.goalState = k;
                return s;
            }

            const int    l = (int) ( k / cells );
            const size_t i = k % cells;
            const int    x = (int) ( i % grid.nx ), y = (int) ( i / grid.nx );

            for( int d = 0; d < 8; ++d )
            {
                const int nx = x + DX[d], ny = y + DY[d];

                if( nx < 0 || ny < 0 || nx >= grid.nx || ny >= grid.ny )
                    continue;

                const size_t n = (size_t) l * cells + (size_t) ny * grid.nx + nx;

                if( !s.from[n] && !( track[n] & aMode ) )
                {
                    s.from[n] = (uint8_t) ( d + 1 );
                    open.push_back( n );
                }
            }

            if( vias && !( via[i] & aMode ) )
            {
                for( int m = 0; m < L; ++m )
                {
                    const size_t n = (size_t) m * cells + i;

                    if( m != l && !s.from[n] && !( track[n] & aMode ) )
                    {
                        if( aFewestVias )
                        {
                            next.emplace_back( n, (uint8_t) ( 11 + l ) );
                        }
                        else
                        {
                            s.from[n] = (uint8_t) ( 11 + l );
                            open.push_back( n );
                        }
                    }
                }
            }
        }

        return s;
    };

    seeds( islandA, OPT, START_OPT );
    seeds( islandB, OPT, GOAL_OPT );
    seeds( islandA, PES, START_PES );
    seeds( islandB, PES, GOAL_PES );

    // a window: its cells inside the coarse grid's rooms are ends too (a coarse pessimistic cell is clear
    // at every point, so a fine cell in it is)
    if( aWindow )
    {
        const ORACLE_GRID& c = aWindow->coarse;
        const size_t       coarseCells = c.Cells();

        for( int l = 0; l < L; ++l )
        {
            for( int y = 0; y < grid.ny; ++y )
            {
                for( int x = 0; x < grid.nx; ++x )
                {
                    const VECTOR2I at = grid.Centre( x, y ) - c.origin;

                    if( at.x < 0 || at.y < 0 || at.x / c.cell >= c.nx || at.y / c.cell >= c.ny )
                        continue;

                    const size_t ck = (size_t) l * coarseCells + (size_t) ( at.y / c.cell ) * c.nx + at.x / c.cell;
                    const size_t k = (size_t) l * cells + (size_t) y * grid.nx + x;

                    if( ( *aWindow->startRoom )[ck] )
                        track[k] |= START_PES;

                    if( ( *aWindow->goalRoom )[ck] )
                        track[k] |= GOAL_PES;
                }
            }
        }
    }

    nlohmann::json out = { { "net", str( UnescapeString( from->GetNetname() ) ) },
                           { "from", itemName( from ) },
                           { "to", to ? nlohmann::json( itemName( to ) ) : nlohmann::json( "the net's other islands" ) },
                           { "width_mm", pcbIUScale.IUTomm( width ) },
                           { "rules_width_mm", { { "min", pcbIUScale.IUTomm( rt.min ) }, { "opt", pcbIUScale.IUTomm( rt.opt ) } } },
                           { "via_mm", vias ? nlohmann::json( { pcbIUScale.IUTomm( viaDia ), pcbIUScale.IUTomm( viaDrill ) } ) : nlohmann::json() },
                           { "cell_mm", pcbIUScale.IUTomm( grid.cell ) },
                           { "obstacles", obstacles.size() } };

    // where a path of the optimistic grid crosses cells the pessimistic one shuts: per stretch its narrowest gap,
    // and the items nearest it with their exact room there (distance less the reach the rules ask)
    auto edgeDistance = []( const SHAPE_POLY_SET& aPoly, const VECTOR2I& aAt )
    {
        if( !aPoly.Contains( aAt ) )
            return 0;

        SEG::ecoord best = VECTOR2I::ECOORD_MAX;

        for( auto it = aPoly.CIterateSegmentsWithHoles(); it; ++it )
            best = std::min( best, ( *it ).SquaredDistance( aAt ) );

        return (int) std::sqrt( (double) best );
    };

    // aFromStart / aFromGoal: the pessimistic grid's rooms of the start and of the goal; only the part of
    // the path from where it last leaves the start's room to where it first enters the goal's is a gap
    auto pinches = [&]( const SEARCH& aSearch, const SEARCH& aFromStart, const SEARCH& aFromGoal, std::optional<BOX2I>& aGap )
    {
        struct STEP { size_t state; bool via; bool shut; };
        struct ROOM { int room; std::string name; VECTOR2I toward; const BOARD_ITEM* item; };
        std::vector<STEP> walk;

        for( size_t k = aSearch.goalState;; )
        {
            const uint8_t step = aSearch.from[k];
            const size_t  i = k % cells;
            const bool    isVia = step >= 11;
            walk.push_back( { k, isVia, isVia ? ( via[i] & PES ) != 0 : ( track[k] & PES ) != 0 } );

            if( step == 10 )
                break;

            if( isVia )
            {
                k = (size_t) ( step - 11 ) * cells + i;
                continue;
            }

            const int d = step - 1, x = (int) ( i % grid.nx ), y = (int) ( i / grid.nx );
            k = ( k / cells ) * cells + (size_t) ( y - DY[d] ) * grid.nx + ( x - DX[d] );
        }

        std::reverse( walk.begin(), walk.end() );

        size_t leave = 0, enter = walk.size();

        for( size_t j = 0; j < walk.size(); ++j )
        {
            if( aFromStart.from[walk[j].state] )
                leave = j;
        }

        for( size_t j = leave; j < walk.size(); ++j )
        {
            if( aFromGoal.from[walk[j].state] )
            {
                enter = j;
                break;
            }
        }

        for( size_t j = 0; j < walk.size(); ++j )
        {
            walk[j].shut = walk[j].shut && j > leave && ( j < enter || ( j == enter && walk[j].via ) );

            if( j >= leave && j <= std::min( enter, walk.size() - 1 ) )
            {
                const size_t   i = walk[j].state % cells;
                const VECTOR2I at = grid.Centre( (int) ( i % grid.nx ), (int) ( i / grid.nx ) );

                if( aGap )
                    aGap->Merge( at );
                else
                    aGap = BOX2I( at, VECTOR2I( 0, 0 ) );
            }
        }

        const int edgeClr = std::max( board->GetDesignSettings().m_CopperEdgeClearance, 0 );
        const int halfWidth = width / 2, viaRadius = viaDia / 2;
        const bool zonesYield = aArgs.value( "zones_yield", true );
        nlohmann::json list = nlohmann::json::array();
        int stretches = 0;

        for( size_t a = 0; a < walk.size(); )
        {
            if( !walk[a].shut )
            {
                ++a;
                continue;
            }

            size_t b = a;

            while( b + 1 < walk.size() && walk[b + 1].shut && walk[b + 1].via == walk[a].via )
                ++b;

            ++stretches;

            if( list.size() < 8 )
            {
                // the items round a cell of the path and their room there, tightest first
                auto roomAt = [&]( const STEP& mid )
                {
                    const size_t   i = mid.state % cells;
                    const int      l = (int) ( mid.state / cells );
                    const VECTOR2I at = grid.Centre( (int) ( i % grid.nx ), (int) ( i / grid.nx ) );
                    std::vector<ROOM> room;

                    {
                        const int reach = edgeClr + ( mid.via ? viaRadius : halfWidth );
                        VECTOR2I  near = at;
                        SEG::ecoord best = VECTOR2I::ECOORD_MAX;

                        for( auto it = outline.CIterateSegmentsWithHoles(); it; ++it )
                        {
                            if( ( *it ).SquaredDistance( at ) < best )
                            {
                                best = ( *it ).SquaredDistance( at );
                                near = ( *it ).NearestPoint( at );
                            }
                        }

                        room.push_back( { edgeDistance( outline, at ) - reach, "board edge", near - at, nullptr } );
                    }

                    for( const OBSTACLE& o : obstacles )
                    {
                        const int reach = mid.via ? o.viaReach : o.trackReach;
                        const bool hole = o.layer == UNDEFINED_LAYER;

                        if( reach < 0 || ( !mid.via && !hole && o.layer != layers[l] ) )
                            continue;

                        const BOARD_CONNECTED_ITEM* bci = dynamic_cast<const BOARD_CONNECTED_ITEM*>( o.item );
                        const bool ruleArea = o.item->Type() == PCB_ZONE_T && static_cast<ZONE*>( o.item )->GetIsRuleArea();

                        if( !ruleArea && !o.anyNet && bci && bci->GetNetCode() == net && !( hole && mid.via ) )
                            continue;

                        if( mid.via && zonesYield && !ruleArea && o.item->Type() == PCB_ZONE_T )
                            continue;

                        BOX2I box = o.item->GetBoundingBox();
                        box.Inflate( reach + pcbIUScale.mmToIU( 1.0 ) );

                        if( !box.Contains( at ) )
                            continue;

                        int      dist = 0;
                        VECTOR2I near = at;

                        if( ruleArea )
                        {
                            const SHAPE_POLY_SET area = static_cast<ZONE*>( o.item )->GetBoardOutline();
                            dist = area.Contains( at ) ? 0 : (int) std::sqrt( (double) area.SquaredDistance( at, false, &near ) );
                        }
                        else if( hole )
                        {
                            std::shared_ptr<SHAPE_SEGMENT> h = o.item->Type() == PCB_PAD_T ? static_cast<PAD*>( o.item )->GetEffectiveHoleShape()
                                                                                           : static_cast<PCB_VIA*>( o.item )->GetEffectiveHoleShape();
                            if( !h )
                                continue;

                            h->Collide( at, std::numeric_limits<int>::max() / 2, &dist, &near );
                        }
                        else
                        {
                            const PCB_LAYER_ID layer = mid.via ? ( o.layer == UNDEFINED_LAYER ? layers[0] : o.layer ) : layers[l];
                            o.item->GetEffectiveShape( layer )->Collide( at, std::numeric_limits<int>::max() / 2, &dist, &near );
                        }

                        std::string name = itemName( o.item ) + ( hole ? " hole" : ruleArea ? " rule area" : "" ) + " ("
                                           + ( hole ? std::string( "all" ) : str( board->GetLayerName( o.layer ) ) ) + ")";

                        if( bci && !ruleArea )
                            name += " " + str( UnescapeString( bci->GetNetname() ) );

                        room.push_back( { dist - reach, name, near - at, o.item } );
                    }

                    std::sort( room.begin(), room.end(), []( const ROOM& x, const ROOM& y ) { return x.room < y.room; } );
                    return room;
                };

                // a gap: two items on opposite sides of the cell, its room the sum of theirs
                auto gapAt = [&]( const std::vector<ROOM>& aRoom ) -> std::optional<std::pair<size_t, size_t>>
                {
                    std::optional<std::pair<size_t, size_t>> best;

                    for( size_t x = 0; x < aRoom.size() && x < 24; ++x )
                    {
                        for( size_t y = x + 1; y < aRoom.size() && y < 24; ++y )
                        {
                            const VECTOR2L dx = VECTOR2L( aRoom[x].toward ), dy = VECTOR2L( aRoom[y].toward );

                            if( aRoom[x].item && aRoom[x].item == aRoom[y].item )
                                continue;

                            if( dx.Dot( dy ) >= 0 && dx != VECTOR2L( 0, 0 ) && dy != VECTOR2L( 0, 0 ) )
                                continue;

                            if( !best || aRoom[x].room + aRoom[y].room < aRoom[best->first].room + aRoom[best->second].room )
                                best = std::pair( x, y );
                        }
                    }

                    return best;
                };

                // the stretch's narrowest gap
                size_t pick = ( a + b ) / 2;
                int    best = std::numeric_limits<int>::max();
                const size_t stride = std::max<size_t>( 1, ( b - a + 1 ) / 200 );

                for( size_t j = a; j <= b; j += stride )
                {
                    const std::vector<ROOM> r = roomAt( walk[j] );

                    if( const auto g = gapAt( r ); g && r[g->first].room + r[g->second].room < best )
                    {
                        best = r[g->first].room + r[g->second].room;
                        pick = j;
                    }
                }

                const STEP&    mid = walk[pick];
                const size_t   i = mid.state % cells;
                const int      l = (int) ( mid.state / cells );
                const VECTOR2I at = grid.Centre( (int) ( i % grid.nx ), (int) ( i / grid.nx ) );
                const std::vector<ROOM> room = roomAt( mid );
                nlohmann::json nearest = nlohmann::json::array();

                for( size_t j = 0; j < room.size() && j < 4; ++j )
                    nearest.push_back( { { "item", room[j].name }, { "room_mm", pcbIUScale.IUTomm( room[j].room ) } } );

                nlohmann::json between;

                if( const auto g = gapAt( room ) )
                    between = { { "items", { room[g->first].name, room[g->second].name } },
                                { "room_mm", pcbIUScale.IUTomm( room[g->first].room + room[g->second].room ) } };

                list.push_back( { { "at", { pcbIUScale.IUTomm( at.x ), pcbIUScale.IUTomm( at.y ) } },
                                  { "layer", mid.via ? std::string( "via" ) : str( board->GetLayerName( layers[l] ) ) },
                                  { "cells", b - a + 1 }, { "nearest", nearest }, { "gap", between } } );
            }

            a = b + 1;
        }

        return nlohmann::json( { { "stretches", stretches }, { "first", list } } );
    };

    const int    viaCost = aArgs.contains( "via_cost_mm" )
                               ? (int) std::lround( aArgs["via_cost_mm"].get<double>() * pcbIUScale.mmToIU( 1 ) / grid.cell * 5 )
                               : -1;
    const SEARCH pes = search( PES, START_PES, GOAL_PES, aArgs.value( "fewest_vias", false ), viaCost );

    // the path from a seed to aState in aSearch's tree: corners where the step changes, a via where the
    // layer does
    auto trace = [&]( const SEARCH& aSearch, size_t aState )
    {
        std::vector<std::pair<VECTOR2I, int>> points;
        size_t k = aState;
        int    lastStep = -1;

        while( true )
        {
            const int     l = (int) ( k / cells );
            const size_t  i = k % cells;
            const int     x = (int) ( i % grid.nx ), y = (int) ( i / grid.nx );
            const uint8_t step = aSearch.from[k];

            if( step != lastStep || step >= 10 )
                points.emplace_back( grid.Centre( x, y ), l );

            if( step == 10 )
                break;

            if( step >= 11 )
            {
                const int prev = step - 11;
                k = (size_t) prev * cells + i;
                lastStep = step;
                points.emplace_back( grid.Centre( x, y ), prev );
                continue;
            }

            const int d = step - 1;
            k = (size_t) l * cells + (size_t) ( y - DY[d] ) * grid.nx + ( x - DX[d] );
            lastStep = step;
        }

        std::reverse( points.begin(), points.end() );
        return points;
    };

    auto pathJson = [&]( const std::vector<std::pair<VECTOR2I, int>>& aPoints )
    {
        nlohmann::json path = nlohmann::json::array();

        for( size_t j = 0; j < aPoints.size(); ++j )
        {
            if( j > 0 && aPoints[j] == aPoints[j - 1] )
                continue;

            path.push_back( { { "at", { pcbIUScale.IUTomm( aPoints[j].first.x ), pcbIUScale.IUTomm( aPoints[j].first.y ) } },
                              { "layer", str( board->GetLayerName( layers[aPoints[j].second] ) ) } } );
        }

        return path;
    };

    if( pes.found )
    {
        const std::vector<std::pair<VECTOR2I, int>> points = trace( pes, pes.goalState );

        if( aWindow )
            aWindow->path = points;

        out["verdict"] = "routable";
        out["path"] = pathJson( points );
        out["reached_cells"] = pes.reached;
    }
    else if( aWindow )
    {
        out["verdict"] = "unknown";
    }
    else
    {
        const SEARCH opt = search( OPT, START_OPT, GOAL_OPT );

        if( opt.found )
        {
            out["verdict"] = "unknown";
            out["reason"] = "a way exists in the optimistic grid only: a finer cell_mm decides";
            out["reached_cells"] = opt.reached;
            const SEARCH         back = search( PES, GOAL_PES, START_PES );
            std::optional<BOX2I> gap;
            out["pinches"] = pinches( opt, pes, back, gap );

            // the gap searched again on a finer grid over a window round it, its ends the coarse rooms; a
            // window with no way through has its uncertain cells shut in the optimistic grid and the next
            // optimistic way's gap is tried, a few at most. Only a path found decides
            SEARCH       way = opt;
            const int    ways = aArgs.value( "refine", true ) ? 6 : 0;
            int          tried = 0;
            nlohmann::json windows = nlohmann::json::array();

            for( ; gap && tried < ways; ++tried )
            {
                ORACLE_WINDOW window;
                window.coarse = grid;
                window.startRoom = &pes.from;
                window.goalRoom = &back.from;
                nlohmann::json tries = nlohmann::json::array();

                // a window 8 cells round the gap, then wider: a way round it may leave the first one
                for( const int round : { 8, 24, 56 } )
                {
                    window.box = *gap;
                    window.box.Inflate( grid.cell * round + viaDia );
                    window.box = window.box.Intersect( outline.BBox() );
                    window.cell = grid.cell / 4;

                    // finer by 4 each try while undecided, down to 0.001 mm or the cell budget
                    while( window.cell >= pcbIUScale.mmToIU( 0.001 ) )
                    {
                        const KOPENAPI_RESULT fine = oracleRun( aCtx, aArgs, &window );
                        const double          ran = fine.body.value( "cell_mm", 0.0 );
                        tries.push_back( { { "cell_mm", ran }, { "verdict", fine.body.value( "verdict", std::string() ) } } );

                        if( !window.path.empty() || ran <= 0 || pcbIUScale.mmToIU( ran ) > window.cell )
                            break;

                        window.cell /= 4;
                    }

                    if( !window.path.empty() )
                        break;
                }

                windows.push_back( { { "window_mm", { pcbIUScale.IUTomm( window.box.GetLeft() ), pcbIUScale.IUTomm( window.box.GetTop() ),
                                                      pcbIUScale.IUTomm( window.box.GetRight() ), pcbIUScale.IUTomm( window.box.GetBottom() ) } },
                                     { "tries", tries } } );

                if( !window.path.empty() )
                {
                    // the coarse way to the window's entry, the window's path, the coarse way on to the goal
                    auto coarseState = [&]( const std::pair<VECTOR2I, int>& aAt ) -> std::optional<size_t>
                    {
                        const VECTOR2I d = aAt.first - grid.origin;

                        if( d.x < 0 || d.y < 0 || d.x / grid.cell >= grid.nx || d.y / grid.cell >= grid.ny )
                            return std::nullopt;

                        return (size_t) aAt.second * cells + (size_t) ( d.y / grid.cell ) * grid.nx + d.x / grid.cell;
                    };

                    const std::optional<size_t> entry = coarseState( window.path.front() ), exit = coarseState( window.path.back() );
                    std::vector<std::pair<VECTOR2I, int>> points;

                    if( entry && pes.from[*entry] )
                        points = trace( pes, *entry );

                    points.insert( points.end(), window.path.begin(), window.path.end() );

                    if( exit && back.from[*exit] )
                    {
                        const std::vector<std::pair<VECTOR2I, int>> rest = trace( back, *exit );
                        points.insert( points.end(), rest.rbegin(), rest.rend() );
                    }

                    out["verdict"] = "routable";
                    out["reason"] = "decided on a finer grid over the gap";
                    out["path"] = pathJson( points );
                    break;
                }

                const int x0 = std::max( 0, ( window.box.GetLeft() - grid.origin.x ) / grid.cell );
                const int y0 = std::max( 0, ( window.box.GetTop() - grid.origin.y ) / grid.cell );
                const int x1 = std::min( grid.nx - 1, ( window.box.GetRight() - grid.origin.x ) / grid.cell );
                const int y1 = std::min( grid.ny - 1, ( window.box.GetBottom() - grid.origin.y ) / grid.cell );

                for( int y = y0; y <= y1; ++y )
                {
                    for( int x = x0; x <= x1; ++x )
                    {
                        const size_t i = (size_t) y * grid.nx + x;

                        if( via[i] & PES )
                            via[i] |= OPT;

                        for( int l = 0; l < L; ++l )
                        {
                            uint8_t& t = track[(size_t) l * cells + i];

                            if( ( t & PES ) && !( t & ( START_OPT | GOAL_OPT ) ) )
                                t |= OPT;
                        }
                    }
                }

                way = search( OPT, START_OPT, GOAL_OPT );
                gap.reset();

                if( way.found )
                    pinches( way, pes, back, gap );
            }

            if( !windows.empty() )
            {
                out["refined"] = windows.back();
                out["refined"]["gaps_tried"] = (int) windows.size();
            }
        }
        else
        {
            // no route: the room the start reaches and the one the target reaches, and what shuts the smaller
            const SEARCH back = search( OPT, GOAL_OPT, START_OPT );
            const bool   startSmaller = opt.reached <= back.reached;
            const SEARCH& room = startSmaller ? opt : back;

            out["verdict"] = "unroutable";
            out["reason"] = std::string( "no way in the free space even with every obstacle shrunk by a cell: the " )
                            + ( startSmaller ? "start" : "target" ) + " is shut in";
            out["start_room_cells"] = opt.reached;
            out["target_room_cells"] = back.reached;
            out["room_mm2"] = room.reached * pcbIUScale.IUTomm( grid.cell ) * pcbIUScale.IUTomm( grid.cell );

            // the shut cells round that room, and the obstacles over them
            std::vector<std::pair<int, size_t>> border;   // layer, cell

            for( size_t k = 0; k < cells * L; ++k )
            {
                if( !room.from[k] )
                    continue;

                const int    l = (int) ( k / cells );
                const size_t i = k % cells;
                const int    x = (int) ( i % grid.nx ), y = (int) ( i / grid.nx );

                for( int d = 0; d < 4; ++d )
                {
                    const int nx = x + DX[d], ny = y + DY[d];

                    if( nx < 0 || ny < 0 || nx >= grid.nx || ny >= grid.ny )
                        continue;

                    const size_t n = (size_t) l * cells + (size_t) ny * grid.nx + nx;

                    if( track[n] & OPT && !room.from[n] )
                        border.emplace_back( l, (size_t) ny * grid.nx + nx );
                }
            }

            std::map<std::string, int> by;
            std::map<std::string, std::string> byNet;
            BOX2I box;

            for( const auto& [l, i] : border )
                box.Merge( grid.Centre( (int) ( i % grid.nx ), (int) ( i / grid.nx ) ) );

            for( const OBSTACLE& o : obstacles )
            {
                if( o.layer == UNDEFINED_LAYER || o.trackReach < 0 )
                    continue;

                BOX2I ob = o.item->GetBoundingBox();
                ob.Inflate( o.trackReach + diag );

                if( !ob.Intersects( box ) )
                    continue;

                const int l = (int) ( std::find( layers.begin(), layers.end(), o.layer ) - layers.begin() );
                std::vector<uint8_t> mine( cells, 0 );
                SHAPE_POLY_SET poly;

                if( o.item->Type() == PCB_ZONE_T && static_cast<ZONE*>( o.item )->GetIsRuleArea() )
                    poly = static_cast<ZONE*>( o.item )->GetBoardOutline();
                else
                    o.item->TransformShapeToPolygon( poly, o.layer, std::max( o.trackReach - diag, 0 ), maxError, ERROR_INSIDE );

                int hits = 0;
                grid.Fill( poly, [&]( size_t i ) { mine[i] = 1; } );

                for( const auto& [bl, i] : border )
                {
                    if( bl == l && mine[i] )
                        hits++;
                }

                if( hits )
                {
                    const std::string key = itemName( o.item ) + " (" + str( board->GetLayerName( o.layer ) ) + ")";
                    by[key] += hits;

                    if( BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( o.item ) )
                        byNet[key] = str( UnescapeString( bci->GetNetname() ) );
                }
            }

            std::vector<std::pair<int, std::string>> ranked;

            for( const auto& [k, v] : by )
                ranked.emplace_back( -v, k );

            std::sort( ranked.begin(), ranked.end() );
            nlohmann::json shut = nlohmann::json::array();

            for( size_t j = 0; j < ranked.size() && j < 12; ++j )
                shut.push_back( { { "item", ranked[j].second }, { "net", byNet[ranked[j].second] }, { "border_cells", -ranked[j].first } } );

            out["shut_by"] = shut;
        }
    }

    out["ms"] = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();
    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_route_oracle( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return oracleRun( aCtx, aArgs, nullptr );
}


KOPENAPI_REGISTER( "pcb_route_oracle",
                   "Whether a connection can be routed at all on the board as it is, decided apart from the router: "
                   "two grids over the whole board bound the free space under KiCad's rules (clearance, hole "
                   "clearance, hole to hole, edge clearance, rule areas) for a track of width_mm (default the rules' "
                   "narrowest for the net) and a through via (default the net's). verdict: routable (a path in the "
                   "pessimistic grid, every point of its cells clear: path [{at, layer}], corners at cell centres, a "
                   "layer change is a via), unroutable (no path even in the optimistic grid, every obstacle shrunk "
                   "by a cell: no route exists at that width and via; start / target room in cells, the room's "
                   "area, shut_by: the obstacles round the smaller room), unknown (the grids disagree: a finer "
                   "cell_mm decides; with refine (default) the gap is searched again over a window round it on a "
                   "grid 4x finer each try, down to 0.001 mm or the cell budget, the window 8, 24 then 56 cells round "
                   "the gap; a window with no way through has its "
                   "uncertain cells shut on the optimistic grid and the next optimistic way's gap is tried, 6 at most: "
                   "refined {window_mm, tries [{cell_mm, verdict}], gaps_tried} of the last, and a path found there is the routable verdict's path; pinches: where the optimistic path, between the pessimistic grid's room of the start and the "
                   "goal's, crosses cells the pessimistic grid shuts, "
                   "per stretch its narrowest gap: at, the nearest items with room_mm (their distance less the "
                   "reach the rules ask, negative: too near there) and gap {items on opposite sides, room_mm the "
                   "sum of theirs}), connected. Other copper of the net is no obstacle; with zones_yield (default) a zone fill shuts no "
                   "via, as a refill clears round it; "
                   "rules_width_mm: the rules' min and opt width for the net",
                   R"json({"type":"object","properties":{
                        "from":{"type":"string","description":"pad REF.NUMBER"},
                        "to":{"type":"string","description":"pad REF.NUMBER of the same net; without it any copper of the net off from's island"},
                        "net":{"type":"string","description":"without from / to: the net's first pad whose island is not the whole net, to the rest"},
                        "width_mm":{"type":"number","exclusiveMinimum":0},
                        "via_mm":{"type":"number","exclusiveMinimum":0},
                        "drill_mm":{"type":"number","exclusiveMinimum":0},
                        "vias":{"type":"boolean","default":true},
                        "layers":{"type":"array","items":{"type":"string"},"description":"copper layers the path may use (default all)"},
                        "fewest_vias":{"type":"boolean","default":false,"description":"the path with the fewest vias (else the fewest cells, a via one)"},
                        "via_cost_mm":{"type":"number","minimum":0,"description":"the path of the least length plus this per via (a diagonal step its length); before fewest_vias"},
                        "no_via_mm":{"type":"array","items":{"type":"array","items":{"type":"number"},"minItems":4,"maxItems":4},"description":"boxes [x0, y0, x1, y1] no via may overlap"},
                        "kept_ways":{"type":"array","items":{"type":"object","properties":{"layer":{"type":"string"},"a":{"type":"array","items":{"type":"number"}},"b":{"type":"array","items":{"type":"number"}},"width_mm":{"type":"number"},"clearance_mm":{"type":"number"}}},"description":"ways out kept open for other pads: segments a to b of that width on that layer, which no copper of this net may come within clearance_mm of"},
                        "cell_mm":{"type":"number","default":0.025,"minimum":0.005},
                        "max_cells_m":{"type":"number","default":20,"description":"millions of cells per layer at most: the cell grows to fit"},
                        "zones_yield":{"type":"boolean","default":true,"description":"zone fills shut no via (a refill clears round it); tracks keep out of them"},
                        "refine":{"type":"boolean","default":true,"description":"an undecided gap searched again over a window round it, 4x finer each try"}},
                        "required":[]})json"_json,
                   false, h_pcb_route_oracle, 300 );


/*
 * pcb_route_oracle_joint: whether every connection still open on the board fits at once. Each net's open
 * islands are joined on the pessimistic grid (its own copper open to it), the nets negotiating for room:
 * a cell another net's new track or via keeps out costs more each round it stays shared, and its history
 * grows (PathFinder). When no cell is shared, the paths are a joint route: laid together they keep the
 * rules' clearances, each track and via kept out of the others by its own and the widest one's room.
 */
/// @brief An order of copper items that does not depend on where they lie in memory or on uuids made at
/// load (a file may hold none for pads): kind, then a pad's footprint reference and number, then place
/// and layers, the uuid last
static bool stableItemOrder( const BOARD_CONNECTED_ITEM* a, const BOARD_CONNECTED_ITEM* b )
{
    auto before = []( const VECTOR2I& p, const VECTOR2I& q )
    {
        return p.x != q.x ? p.x < q.x : p.y < q.y;
    };

    if( a->Type() != b->Type() )
        return a->Type() < b->Type();

    if( a->Type() == PCB_PAD_T )
    {
        const PAD*       pa = static_cast<const PAD*>( a );
        const PAD*       pb = static_cast<const PAD*>( b );
        const FOOTPRINT* fa = pa->GetParentFootprint();
        const FOOTPRINT* fb = pb->GetParentFootprint();
        const int        byRef = ( fa ? fa->GetReference() : wxString() ).Cmp( fb ? fb->GetReference() : wxString() );

        if( byRef != 0 )
            return byRef < 0;

        if( const int byNumber = pa->GetNumber().Cmp( pb->GetNumber() ); byNumber != 0 )
            return byNumber < 0;
    }

    if( const PCB_TRACK* ta = dynamic_cast<const PCB_TRACK*>( a ) )
    {
        const PCB_TRACK* tb = static_cast<const PCB_TRACK*>( b );

        if( ta->GetStart() != tb->GetStart() )
            return before( ta->GetStart(), tb->GetStart() );

        if( ta->GetEnd() != tb->GetEnd() )
            return before( ta->GetEnd(), tb->GetEnd() );
    }
    else if( a->GetPosition() != b->GetPosition() )
    {
        return before( a->GetPosition(), b->GetPosition() );
    }

    if( a->GetLayerSet() != b->GetLayerSet() )
        return a->GetLayerSet().FmtHex() < b->GetLayerSet().FmtHex();

    return a->m_Uuid < b->m_Uuid;
}


static KOPENAPI_RESULT h_pcb_route_oracle_joint( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( std::string rulesError; !KopenapiDesignRulesFromDisk( board, &rulesError ) )
        return KOPENAPI_RESULT::Error( 422, rulesError );

    const auto                         started = std::chrono::steady_clock::now();
    std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
    const BOARD_DESIGN_SETTINGS&       ds = board->GetDesignSettings();
    const std::vector<PCB_LAYER_ID>    layers = LSET::AllCuMask( board->GetCopperLayerCount() ).Seq();
    const int                          L = (int) layers.size();
    const bool                         vias = aArgs.value( "vias", true ) && L > 1;

    std::set<int> only;

    if( aArgs.contains( "nets" ) )
    {
        for( const nlohmann::json& n : aArgs["nets"] )
        {
            const NETINFO_ITEM* info = KopenapiFindNet( board, n.get<std::string>() );

            if( !info || info->GetNetCode() <= 0 )
                return KOPENAPI_RESULT::Error( 404, "no net " + n.get<std::string>() );

            only.insert( info->GetNetCode() );
        }
    }

    SHAPE_POLY_SET outline;

    if( !board->GetBoardPolygonOutlines( outline, true ) || outline.OutlineCount() == 0 )
        return KOPENAPI_RESULT::Error( 422, "the board has no outline" );

    // region: the grid covers the boxes only; the copper outside is fixed, a net joins its islands that
    // reach a box, inside the boxes
    std::vector<BOX2I> region;

    for( const nlohmann::json& b : aArgs.value( "region", nlohmann::json::array() ) )
    {
        if( !b.is_array() || b.size() != 4 )
            return KOPENAPI_RESULT::Error( 400, "region: boxes as [x0, y0, x1, y1] in mm" );

        const VECTOR2I a( pcbIUScale.mmToIU( b[0].get<double>() ), pcbIUScale.mmToIU( b[1].get<double>() ) );
        const VECTOR2I c( pcbIUScale.mmToIU( b[2].get<double>() ), pcbIUScale.mmToIU( b[3].get<double>() ) );
        BOX2I          box( a, c - a );
        box.Normalize();
        box = box.Intersect( outline.BBox() );

        if( box.GetWidth() <= 0 || box.GetHeight() <= 0 )
            return KOPENAPI_RESULT::Error( 422, "region: a box " + b.dump() + " lies off the board" );

        region.push_back( box );
    }

    ORACLE_GRID grid;
    BOX2I       edges = outline.BBox();

    if( !region.empty() )
    {
        edges = region[0];

        for( const BOX2I& box : region )
            edges.Merge( box );
    }

    grid.cell = pcbIUScale.mmToIU( std::max( 0.005, aArgs.value( "cell_mm", 0.05 ) ) );
    const double cellBudget = aArgs.value( "max_cells_m", 6.0 ) * 1e6;

    while( (double) edges.GetWidth() / grid.cell * edges.GetHeight() / grid.cell > cellBudget )
        grid.cell = grid.cell * 5 / 4 + 1;

    if( !region.empty() )
    {
        // the region's cells are the whole board's cells: the same rasterising, the same answer where the
        // ways stay inside
        const VECTOR2I o = outline.BBox().GetOrigin();
        const VECTOR2I a( o.x + ( edges.GetLeft() - o.x ) / grid.cell * grid.cell, o.y + ( edges.GetTop() - o.y ) / grid.cell * grid.cell );
        edges.SetSize( edges.GetRight() - a.x, edges.GetBottom() - a.y );
        edges.SetOrigin( a );
    }

    grid.origin = edges.GetOrigin();
    grid.nx = edges.GetWidth() / grid.cell + 1;
    grid.ny = edges.GetHeight() / grid.cell + 1;

    const size_t cells = grid.Cells();
    const int    maxError = std::max( 1, grid.cell / 8 );
    const int    diag = (int) std::ceil( grid.cell * M_SQRT1_2 ) + maxError + 1;

    // the nets with more than one island, each with the track and via the rules give it
    struct JOB
    {
        int                                             net = 0;
        int                                             cls = 0;
        ORACLE_PROBE                                    probe;
        int                                             clearance = 0;
        std::vector<std::vector<BOARD_CONNECTED_ITEM*>> islands;
        BOX2I                                           box;
        bool                                            wide = false;    // searched over the whole board
        std::vector<std::vector<size_t>>                paths;           // per joined island: layer * cells + cell
        std::vector<size_t>                             trackStates;
        std::vector<size_t>                             viaCells;
        int                                             joined = 0;
        int                                             outside = 0;     // islands that reach no box of the region
        size_t                                          conflicts = 0;
    };

    std::vector<JOB>                                          jobs;
    std::map<std::tuple<std::string, int, int, int>, int>     classOf;
    std::vector<ORACLE_PROBE>                                 classes;

    for( NETINFO_ITEM* info : board->GetNetInfo() )
    {
        const int code = info->GetNetCode();

        if( code <= 0 || ( !only.empty() && !only.count( code ) ) )
            continue;

        std::vector<BOARD_CONNECTED_ITEM*> items =
                conn->GetNetItems( code, { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_ZONE_T } );

        if( items.size() < 2 )
            continue;

        // the connectivity's order follows memory addresses: in a fixed order the islands and the order
        // they are joined in are the same on every load of the board
        std::sort( items.begin(), items.end(), stableItemOrder );

        JOB                                     job;
        std::map<const BOARD_CONNECTED_ITEM*, int> islandOf;

        for( BOARD_CONNECTED_ITEM* item : items )
        {
            if( islandOf.count( item ) )
                continue;

            const int k = (int) job.islands.size();
            job.islands.emplace_back();
            islandOf[item] = k;
            job.islands[k].push_back( item );

            for( BOARD_CONNECTED_ITEM* other : conn->GetConnectedItems( item ) )
            {
                if( other->GetNetCode() == code && !islandOf.count( other ) )
                {
                    islandOf[other] = k;
                    job.islands[k].push_back( other );
                }
            }
        }

        if( !region.empty() )
        {
            auto reaches = [&]( const std::vector<BOARD_CONNECTED_ITEM*>& aIsland )
            {
                return std::any_of( aIsland.begin(), aIsland.end(), [&]( BOARD_CONNECTED_ITEM* aItem )
                                    { return std::any_of( region.begin(), region.end(), [&]( const BOX2I& aBox )
                                                          { return aBox.Intersects( aItem->GetBoundingBox() ); } ); } );
            };

            const size_t had = job.islands.size();
            job.islands.erase( std::remove_if( job.islands.begin(), job.islands.end(),
                                               [&]( const auto& aIsland ) { return !reaches( aIsland ); } ),
                               job.islands.end() );
            job.outside = (int) ( had - job.islands.size() );
            items.clear();

            for( const std::vector<BOARD_CONNECTED_ITEM*>& island : job.islands )
                items.insert( items.end(), island.begin(), island.end() );
        }

        if( job.islands.size() < 2 )
            continue;

        for( std::vector<BOARD_CONNECTED_ITEM*>& island : job.islands )
        {
            std::sort( island.begin(), island.end(), stableItemOrder );
        }

        job.net = code;
        job.probe.net = code;
        job.probe.width = ds.m_TrackMinWidth;
        job.probe.vias = vias;
        bool viaSet = false;

        for( BOARD_CONNECTED_ITEM* item : items )
        {
            job.box.Merge( item->GetBoundingBox() );

            if( item->Type() != PCB_PAD_T )
                continue;

            PAD* pad = static_cast<PAD*>( item );
            job.probe.width = std::max( job.probe.width, KopenapiRuleTrack( board, code, pad->GetPosition(), pad->GetPrincipalLayer() ).min );

            if( !viaSet )
            {
                const KOPENAPI_RULE_VIA rv = KopenapiRuleVia( board, code, pad->GetPosition(), pad->GetPrincipalLayer() );
                job.probe.viaDia = rv.dia;
                job.probe.viaDrill = rv.drill;
                viaSet = true;
            }
        }

        if( !viaSet )
        {
            const KOPENAPI_RULE_VIA rv = KopenapiRuleVia( board, code, job.box.Centre(), layers[0] );
            job.probe.viaDia = rv.dia;
            job.probe.viaDrill = rv.drill;
        }

        const NETCLASS* nc = info->GetNetClass();
        job.clearance = std::max( ds.m_MinClearance, nc ? nc->GetClearance() : 0 );

        const auto key = std::make_tuple( nc ? str( nc->GetName() ) : std::string(), job.probe.width, job.probe.viaDia,
                                          job.probe.viaDrill );

        if( !classOf.count( key ) )
        {
            classOf[key] = (int) classes.size();
            classes.push_back( job.probe );
        }

        job.cls = classOf[key];
        jobs.push_back( std::move( job ) );
    }

    nlohmann::json out = { { "cell_mm", pcbIUScale.IUTomm( grid.cell ) }, { "nets", jobs.size() }, { "classes", classes.size() } };

    if( jobs.empty() )
    {
        out["converged"] = true;
        out["reason"] = "every net is one island";
        return KOPENAPI_RESULT::Ok( out );
    }

    // per class, the pessimistic grid: who may put a track centre (per layer) or a via centre in a cell.
    // 0 every net, a net code that net only (its own copper), 0xFFFF none
    std::vector<std::vector<uint16_t>> ownTrack( classes.size() ), ownVia( classes.size() );

    for( size_t c = 0; c < classes.size(); ++c )
    {
        ownTrack[c].assign( cells * L, 0 );
        ownVia[c].assign( vias ? cells : 0, 0 );

        rasteriseObstacles( board, layers, grid, outline, diag, maxError, classes[c], 0, { PES }, aArgs.value( "zones_yield", true ),
                [&]( int aOwner, int aLayer, size_t aCell, uint8_t )
                {
                    uint16_t& o = aLayer < 0 ? ownVia[c][aCell] : ownTrack[c][(size_t) aLayer * cells + aCell];
                    const uint16_t v = aOwner > 0 && aOwner < 0xFFFF ? (uint16_t) aOwner : 0xFFFF;
                    o = o == 0 ? v : ( o == v ? o : 0xFFFF );
                } );
    }

    if( region.size() > 1 )
    {
        std::vector<uint8_t> in( cells, 0 );

        for( size_t i = 0; i < cells; ++i )
        {
            const VECTOR2I at = grid.Centre( (int) ( i % grid.nx ), (int) ( i / grid.nx ) );
            in[i] = std::any_of( region.begin(), region.end(), [&]( const BOX2I& aBox ) { return aBox.Contains( at ); } );
        }

        for( size_t c = 0; c < classes.size(); ++c )
        {
            for( size_t i = 0; i < cells; ++i )
            {
                if( in[i] )
                    continue;

                for( int l = 0; l < L; ++l )
                    ownTrack[c][(size_t) l * cells + i] = 0xFFFF;

                if( vias )
                    ownVia[c][i] = 0xFFFF;
            }
        }
    }

    const double buildMs = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();

    // per class: the largest clearance of its nets
    std::vector<int> classClearance( classes.size(), ds.m_MinClearance );

    for( const JOB& job : jobs )
        classClearance[job.cls] = std::max( classClearance[job.cls], job.clearance );

    const size_t C = classes.size();

    const int holeClr = std::max( ds.m_HoleClearance, 0 ), h2h = std::max( ds.m_HoleToHoleMin, 0 );
    // two centres a cell's half diagonal off each true line (claim_slack_mm: less, DRC of the laid paths
    // then decides)
    const int slack = aArgs.contains( "claim_slack_mm" ) ? pcbIUScale.mmToIU( aArgs["claim_slack_mm"].get<double>() )
                                                         : (int) std::ceil( grid.cell * M_SQRT2 ) + 1;

    std::map<int, std::vector<std::pair<int, int>>> discs;

    auto disc = [&]( int aRadius ) -> const std::vector<std::pair<int, int>>&
    {
        const int r = ( aRadius + grid.cell - 1 ) / grid.cell;
        auto      it = discs.find( r );

        if( it != discs.end() )
            return it->second;

        std::vector<std::pair<int, int>>& d = discs[r];

        for( int dy = -r; dy <= r; ++dy )
        {
            for( int dx = -r; dx <= r; ++dx )
            {
                if( (int64_t) dx * dx + (int64_t) dy * dy <= (int64_t) r * r )
                    d.emplace_back( dx, dy );
            }
        }

        return d;
    };

    // per class: how many nets keep that class's track (via) centre out of a cell
    std::vector<std::vector<uint8_t>> claimTrack( C ), claimVia( C );

    for( size_t c = 0; c < C; ++c )
    {
        claimTrack[c].assign( cells * L, 0 );
        claimVia[c].assign( vias ? cells * L : 0, 0 );
    }

    std::vector<uint32_t> stamp( cells * L, 0 );
    std::vector<float>    history( cells * L, 0.f ), historyVia( vias ? cells : 0, 0.f );
    uint32_t              epoch = 0;

    // adds (aDelta 1) or takes (-1) a net's claims: the room its new tracks and vias keep each class's
    // centres out of (both halves, the larger clearance, a cell's diagonal), each cell once per net
    auto claim = [&]( const JOB& aJob, int aDelta )
    {
        const int halfW = aJob.probe.width / 2, viaR = aJob.probe.viaDia / 2, drill = aJob.probe.viaDrill / 2;

        auto stampDisc = [&]( std::vector<uint8_t>& aClaim, std::vector<uint32_t>& aStamp, int aLayer, size_t aCell, int aRadius )
        {
            const int x = (int) ( aCell % grid.nx ), y = (int) ( aCell / grid.nx );

            for( const auto& [dx, dy] : disc( aRadius ) )
            {
                const int nx = x + dx, ny = y + dy;

                if( nx < 0 || ny < 0 || nx >= grid.nx || ny >= grid.ny )
                    continue;

                const size_t k = (size_t) aLayer * cells + (size_t) ny * grid.nx + nx;

                if( aStamp[k] == epoch )
                    continue;

                aStamp[k] = epoch;
                aClaim[k] = (uint8_t) std::clamp( (int) aClaim[k] + aDelta, 0, 255 );
            }
        };

        for( size_t c = 0; c < C; ++c )
        {
            const int oHalf = classes[c].width / 2, oVia = classes[c].viaDia / 2, oDrill = classes[c].viaDrill / 2;
            const int clr = std::max( classClearance[c], aJob.clearance );
            const int rTT = halfW + oHalf + clr + slack;
            const int rTV = std::max( halfW + oVia + clr, halfW + oDrill + holeClr ) + slack;
            const int rVT = std::max( viaR + oHalf + clr, drill + oHalf + holeClr ) + slack;
            const int rVV = std::max( { viaR + oVia + clr, drill + oDrill + h2h, drill + oVia + holeClr, viaR + oDrill + holeClr } ) + slack;

            ++epoch;

            for( size_t k : aJob.trackStates )
                stampDisc( claimTrack[c], stamp, (int) ( k / cells ), k % cells, rTT );

            for( size_t i : aJob.viaCells )
            {
                for( int l = 0; l < L; ++l )
                    stampDisc( claimTrack[c], stamp, l, i, rVT );
            }

            if( !vias )
                continue;

            ++epoch;

            for( size_t k : aJob.trackStates )
                stampDisc( claimVia[c], stamp, (int) ( k / cells ), k % cells, rTV );

            for( size_t i : aJob.viaCells )
            {
                for( int l = 0; l < L; ++l )
                    stampDisc( claimVia[c], stamp, l, i, rVV );
            }
        }
    };

    // the cells where another net's claim lies on this net's track or via
    auto conflicts = [&]( const JOB& aJob, std::vector<size_t>* aTrack, std::vector<size_t>* aVia )
    {
        size_t n = 0;

        const std::vector<uint8_t>& ct = claimTrack[aJob.cls];
        const std::vector<uint8_t>& cv = claimVia[aJob.cls];

        for( size_t k : aJob.trackStates )
        {
            if( ct[k] >= 2 )
            {
                n++;

                if( aTrack )
                    aTrack->push_back( k );
            }
        }

        for( size_t i : aJob.viaCells )
        {
            for( int l = 0; l < L; ++l )
            {
                if( cv[(size_t) l * cells + i] >= 2 )
                {
                    n++;

                    if( aVia )
                        aVia->push_back( i );

                    break;
                }
            }
        }

        return n;
    };

    const int    DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    const int    DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    const float  STEP[8] = { 1.f, 1.f, 1.f, 1.f, (float) M_SQRT2, (float) M_SQRT2, (float) M_SQRT2, (float) M_SQRT2 };
    const float  viaCost = (float) ( pcbIUScale.mmToIU( aArgs.value( "via_cost_mm", 1.0 ) ) / (double) grid.cell );
    const int    margin = std::max( 1, (int) ( pcbIUScale.mmToIU( aArgs.value( "margin_mm", 3.0 ) ) / grid.cell ) );
    const double workCap = aArgs.value( "max_work_m", 20000.0 ) * 1e6;
    double       work = 0;

    COST_QUEUE open;

    // joins a net's islands one by one, from the tree joined so far to the nearest island left
    auto route = [&]( JOB& aJob, float aPresent ) -> bool
    {
        const std::vector<uint16_t>& own = ownTrack[aJob.cls];
        const std::vector<uint16_t>& ownV = vias ? ownVia[aJob.cls] : own;
        const uint16_t               me = (uint16_t) aJob.net;
        const std::vector<uint8_t>&  claimT = claimTrack[aJob.cls];
        const std::vector<uint8_t>&  claimV = claimVia[aJob.cls];

        int x0 = 0, y0 = 0, x1 = grid.nx - 1, y1 = grid.ny - 1;

        if( !aJob.wide )
        {
            x0 = std::max( 0, ( aJob.box.GetLeft() - grid.origin.x ) / grid.cell - margin );
            y0 = std::max( 0, ( aJob.box.GetTop() - grid.origin.y ) / grid.cell - margin );
            x1 = std::min( grid.nx - 1, ( aJob.box.GetRight() - grid.origin.x ) / grid.cell + margin );
            y1 = std::min( grid.ny - 1, ( aJob.box.GetBottom() - grid.origin.y ) / grid.cell + margin );
        }

        const int    wx = x1 - x0 + 1, wy = y1 - y0 + 1;
        const size_t wc = (size_t) wx * wy;

        auto local = [&]( size_t aGlobal ) -> size_t
        {
            const int    l = (int) ( aGlobal / cells );
            const size_t i = aGlobal % cells;
            return (size_t) l * wc + (size_t) ( (int) ( i / grid.nx ) - y0 ) * wx + ( (int) ( i % grid.nx ) - x0 );
        };

        auto global = [&]( size_t aLocal ) -> size_t
        {
            const int    l = (int) ( aLocal / wc );
            const size_t i = aLocal % wc;
            return (size_t) l * cells + (size_t) ( (int) ( i / wx ) + y0 ) * grid.nx + ( (int) ( i % wx ) + x0 );
        };

        // island cells: a centre on the island's copper
        std::vector<int16_t> island( wc * L, -1 );
        std::vector<BOX2I>   islandBox( aJob.islands.size() );
        std::vector<uint8_t> hasCells( aJob.islands.size(), 0 );

        for( size_t k = 0; k < aJob.islands.size(); ++k )
        {
            for( BOARD_CONNECTED_ITEM* item : aJob.islands[k] )
            {
                for( int l = 0; l < L; ++l )
                {
                    if( !item->IsOnLayer( layers[l] ) )
                        continue;

                    if( item->Type() == PCB_PAD_T && !static_cast<PAD*>( item )->FlashLayer( layers[l] ) )
                        continue;

                    SHAPE_POLY_SET poly;
                    item->TransformShapeToPolygon( poly, layers[l], 0, maxError, ERROR_INSIDE );
                    poly.Deflate( diag, CORNER_STRATEGY::ROUND_ALL_CORNERS, maxError );

                    grid.Fill( poly, [&]( size_t i )
                    {
                        const int x = (int) ( i % grid.nx ), y = (int) ( i / grid.nx );

                        if( x < x0 || x > x1 || y < y0 || y > y1 )
                            return;

                        // an end only where a track centre of this net may be
                        const uint16_t o = own[(size_t) l * cells + i];

                        if( o != 0 && o != me )
                            return;

                        island[(size_t) l * wc + (size_t) ( y - y0 ) * wx + ( x - x0 )] = (int16_t) k;

                        if( !hasCells[k] )
                            islandBox[k] = BOX2I( VECTOR2I( x - x0, y - y0 ), VECTOR2I( 0, 0 ) );
                        else
                            islandBox[k].Merge( VECTOR2I( x - x0, y - y0 ) );

                        hasCells[k] = 1;
                    } );
                }
            }
        }

        aJob.paths.clear();
        aJob.trackStates.clear();
        aJob.viaCells.clear();
        aJob.joined = 1;

        std::vector<uint8_t> joined( aJob.islands.size(), 0 ), tree( wc * L, 0 );
        // cells a new via of this net may not take: too near (hole to hole) one of its vias already placed
        std::vector<uint8_t> viaNear( wc, 0 );
        std::vector<float>   g( wc * L );
        std::vector<int32_t> prev( wc * L );
        joined[0] = 1;

        for( size_t s = 0; s < wc * L; ++s )
        {
            if( island[s] == 0 )
                tree[s] = 1;
        }

        bool all = true;

        for( size_t round = 1; round < aJob.islands.size(); ++round )
        {
            BOX2I target;
            bool  anyTarget = false;

            for( size_t k = 0; k < aJob.islands.size(); ++k )
            {
                if( joined[k] || !hasCells[k] )
                    continue;

                if( !anyTarget )
                    target = islandBox[k];
                else
                    target.Merge( islandBox[k] );

                anyTarget = true;
            }

            if( !anyTarget )
            {
                all = false;
                break;
            }

            // a path whose own vias come nearer each other than hole to hole is searched again with no
            // other via round the first of them
            const int           viaGap = ( aJob.probe.viaDrill + h2h + slack + grid.cell - 1 ) / grid.cell;
            std::vector<size_t> path;
            int64_t             reached = -1;

            for( int attempt = 0;; ++attempt )
            {
                std::fill( g.begin(), g.end(), std::numeric_limits<float>::max() );
                std::fill( prev.begin(), prev.end(), -1 );

                open.clear();

                auto h = [&]( size_t s )
                {
                    const size_t i = s % wc;
                    const int    x = (int) ( i % wx ), y = (int) ( i / wx );
                    const int    dx = std::max( { 0, target.GetLeft() - x, x - target.GetRight() } );
                    const int    dy = std::max( { 0, target.GetTop() - y, y - target.GetBottom() } );
                    return (float) ( std::max( dx, dy ) + ( M_SQRT2 - 1 ) * std::min( dx, dy ) );
                };

                for( size_t s = 0; s < wc * L; ++s )
                {
                    if( tree[s] )
                    {
                        g[s] = 0;
                        open.push( h( s ), (uint32_t) s );
                    }
                }

                reached = -1;

                while( !open.empty() )
                {
                    const auto [f, s] = open.pop();
                    work++;

                    if( f - h( s ) > g[s] + 1e-3f )
                        continue;

                    if( island[s] >= 0 && !joined[island[s]] )
                    {
                        reached = s;
                        break;
                    }

                    const int    l = (int) ( s / wc );
                    const size_t i = s % wc;
                    const int    x = (int) ( i % wx ), y = (int) ( i / wx );

                    for( int d = 0; d < 8; ++d )
                    {
                        const int nx = x + DX[d], ny = y + DY[d];

                        if( nx < 0 || ny < 0 || nx >= wx || ny >= wy )
                            continue;

                        const size_t n = (size_t) l * wc + (size_t) ny * wx + nx;
                        const size_t k = global( n );

                        if( own[k] != 0 && own[k] != me )
                            continue;

                        const float c = g[s] + STEP[d] * ( 1.f + history[k] ) * ( 1.f + aPresent * claimT[k] );

                        if( c < g[n] )
                        {
                            g[n] = c;
                            prev[n] = (int32_t) s;
                            open.push( c + h( n ), (uint32_t) n );
                        }
                    }

                    if( !vias )
                        continue;

                    const size_t gi = global( s ) % cells;

                    if( ( ownV[gi] != 0 && ownV[gi] != me ) || viaNear[i] )
                        continue;

                    int share = 0;

                    for( int m = 0; m < L; ++m )
                        share += claimV[(size_t) m * cells + gi];

                    for( int m = 0; m < L; ++m )
                    {
                        if( m == l )
                            continue;

                        const size_t n = (size_t) m * wc + i;
                        const size_t k = (size_t) m * cells + gi;

                        if( own[k] != 0 && own[k] != me )
                            continue;

                        const float c = g[s] + viaCost * ( 1.f + historyVia[gi] ) * ( 1.f + aPresent * share );

                        if( c < g[n] )
                        {
                            g[n] = c;
                            prev[n] = (int32_t) s;
                            open.push( c + h( n ), (uint32_t) n );
                        }
                    }
                }

                if( reached < 0 )
                    break;

                // the path back to the tree, then the island it reached joins the tree
                path.clear();

                for( int64_t s = reached; s >= 0; s = prev[s] )
                {
                    path.push_back( global( (size_t) s ) );

                    if( tree[s] )
                        break;
                }

                std::reverse( path.begin(), path.end() );

                if( reached < 0 || !vias || attempt >= 8 )
                    break;

                std::vector<size_t> pv;

                for( size_t j = 1; j < path.size(); ++j )
                {
                    if( path[j] % cells == path[j - 1] % cells )
                        pv.push_back( local( path[j] ) % wc );
                }

                int64_t tooNear = -1;

                for( size_t u = 0; u < pv.size() && tooNear < 0; ++u )
                {
                    for( size_t v = u + 1; v < pv.size(); ++v )
                    {
                        const int ddx = (int) ( pv[u] % wx ) - (int) ( pv[v] % wx );
                        const int ddy = (int) ( pv[u] / wx ) - (int) ( pv[v] / wx );

                        if( pv[u] != pv[v] && (int64_t) ddx * ddx + (int64_t) ddy * ddy <= (int64_t) viaGap * viaGap )
                        {
                            tooNear = (int64_t) pv[u];
                            break;
                        }
                    }
                }

                if( tooNear < 0 )
                    break;

                const int     vx = (int) ( tooNear % wx ), vy = (int) ( tooNear / wx );
                const uint8_t keep = viaNear[tooNear];

                for( const auto& [dx, dy] : disc( aJob.probe.viaDrill + h2h + slack ) )
                {
                    if( vx + dx >= 0 && vy + dy >= 0 && vx + dx < wx && vy + dy < wy )
                        viaNear[(size_t) ( vy + dy ) * wx + vx + dx] = 1;
                }

                viaNear[tooNear] = keep;
            }

            if( reached < 0 )
            {
                all = false;
                break;
            }

            // new copper: the path off the islands' copper, and its vias
            for( size_t j = 0; j < path.size(); ++j )
            {
                tree[local( path[j] )] = 1;

                if( island[local( path[j] )] < 0 )
                    aJob.trackStates.push_back( path[j] );

                if( j > 0 && path[j] % cells == path[j - 1] % cells )
                {
                    aJob.viaCells.push_back( path[j] % cells );

                    const size_t li = local( path[j] ) % wc;
                    const int    vx = (int) ( li % wx ), vy = (int) ( li / wx );

                    for( const auto& [dx, dy] : disc( aJob.probe.viaDrill + h2h + slack ) )
                    {
                        if( vx + dx >= 0 && vy + dy >= 0 && vx + dx < wx && vy + dy < wy )
                            viaNear[(size_t) ( vy + dy ) * wx + vx + dx] = 1;
                    }
                }
            }

            const int16_t reachedIsland = island[reached];
            joined[reachedIsland] = 1;
            aJob.joined++;

            for( size_t s = 0; s < wc * L; ++s )
            {
                if( island[s] == reachedIsland )
                    tree[s] = 1;
            }

            aJob.paths.push_back( std::move( path ) );

            if( work > workCap )
            {
                all = aJob.joined == (int) aJob.islands.size();
                break;
            }
        }

        return all;
    };

    const int maxRounds = aArgs.value( "rounds", 40 );
    float     present = (float) aArgs.value( "present", 0.5 );
    const float presentGrowth = (float) aArgs.value( "present_growth", 1.6 );
    // capped: beyond it a cell step of 1 is lost in a float cost and the search stops telling ways apart
    const float presentMax = (float) aArgs.value( "present_max", 1000.0 );
    const float historyStep = (float) aArgs.value( "history", 1.0 );
    // every net each round, so one in the way of another moves too; else only the nets that share
    const bool  rerouteAll = aArgs.value( "reroute", std::string( "all" ) ) == "all";

    nlohmann::json rounds = nlohmann::json::array();
    bool           converged = false;
    bool           stopped = false;
    int            round = 0;
    size_t         lastShared = 0;

    // the best round seen (fewest nets unjoined, then fewest cells shared): the answer when none converges
    struct KEPT
    {
        std::vector<std::vector<size_t>> paths;
        std::vector<size_t>              trackStates, viaCells;
        int                              joined = 0;
        bool                             wide = false;
    };

    std::vector<KEPT>             best;
    std::pair<size_t, size_t>     bestScore{ SIZE_MAX, SIZE_MAX };
    int                           bestRound = -1;

    // the first round: every net alone in its window, the whole board where the window is too small
    for( size_t j = 0; j < jobs.size() && !stopped; ++j )
    {
        JOB& job = jobs[j];

        if( !KopenapiJobStep( "round 0", (int) j, (int) jobs.size(), str( UnescapeString( board->FindNet( job.net )->GetNetname() ) ),
                              { { "work_m", work / 1e6 } } ) )
        {
            stopped = true;
            break;
        }

        if( !route( job, 0.f ) && !job.wide && work <= workCap )
        {
            job.wide = true;
            route( job, 0.f );
        }

        claim( job, 1 );
    }

    for( ; round <= maxRounds && work <= workCap && !stopped; ++round )
    {
        size_t shared = 0, nets = 0;

        for( JOB& job : jobs )
        {
            std::vector<size_t> t, v;
            job.conflicts = conflicts( job, &t, &v );

            if( !job.conflicts )
                continue;

            shared += job.conflicts;
            nets++;

            for( size_t k : t )
                history[k] += historyStep;

            for( size_t i : v )
                historyVia[i] += historyStep;
        }

        size_t unjoined = 0;

        for( const JOB& job : jobs )
            unjoined += job.joined < (int) job.islands.size();

        rounds.push_back( { { "round", round }, { "nets_sharing", nets }, { "cells_shared", shared }, { "nets_unjoined", unjoined },
                            { "work_m", work / 1e6 } } );

        if( std::make_pair( unjoined, shared ) < bestScore )
        {
            bestScore = { unjoined, shared };
            bestRound = round;
            best.clear();

            for( const JOB& job : jobs )
                best.push_back( { job.paths, job.trackStates, job.viaCells, job.joined, job.wide } );
        }

        if( !nets )
        {
            converged = unjoined == 0;
            break;
        }

        if( round == maxRounds )
            break;

        lastShared = shared;
        int done = 0;
        const int todo = rerouteAll ? (int) jobs.size() : (int) nets;

        for( JOB& job : jobs )
        {
            if( !job.conflicts && !rerouteAll )
                continue;

            if( !KopenapiJobStep( "round " + std::to_string( round + 1 ), done++, todo,
                                  str( UnescapeString( board->FindNet( job.net )->GetNetname() ) ),
                                  { { "cells_shared", lastShared }, { "work_m", work / 1e6 } } ) )
            {
                stopped = true;
                break;
            }

            claim( job, -1 );
            route( job, present );
            claim( job, 1 );
        }

        // finite: an infinite factor times a free cell's zero claim is NaN, and no step would be taken
        present = std::min( present * presentGrowth, presentMax );
    }

    const int lastRound = rounds.empty() ? -1 : rounds.back().value( "round", -1 );

    if( !converged && bestRound >= 0 && bestRound != lastRound && best.size() == jobs.size() )
    {
        for( size_t j = 0; j < jobs.size(); ++j )
        {
            claim( jobs[j], -1 );
            jobs[j].paths = std::move( best[j].paths );
            jobs[j].trackStates = std::move( best[j].trackStates );
            jobs[j].viaCells = std::move( best[j].viaCells );
            jobs[j].joined = best[j].joined;
            jobs[j].wide = best[j].wide;
            claim( jobs[j], 1 );
        }
    }

    // the paths as the single oracle gives them: corners where the step changes, a via where the layer does
    auto points = [&]( const std::vector<size_t>& aPath )
    {
        nlohmann::json pts = nlohmann::json::array();
        int64_t        lastDx = 9, lastDy = 9;

        for( size_t j = 0; j < aPath.size(); ++j )
        {
            const size_t i = aPath[j] % cells;
            const int    l = (int) ( aPath[j] / cells );
            const VECTOR2I at = grid.Centre( (int) ( i % grid.nx ), (int) ( i / grid.nx ) );
            bool         keep = j == 0 || j + 1 == aPath.size();

            if( j > 0 && j + 1 < aPath.size() )
            {
                const size_t a = aPath[j - 1], b = aPath[j + 1];
                const int64_t dx = (int64_t) ( b % cells % grid.nx ) - (int64_t) ( i % grid.nx );
                const int64_t dy = (int64_t) ( b % cells / grid.nx ) - (int64_t) ( i / grid.nx );
                keep = dx != lastDx || dy != lastDy || a / cells != aPath[j] / cells || b / cells != aPath[j] / cells;
                lastDx = dx;
                lastDy = dy;
            }

            if( keep )
            {
                pts.push_back( { { "at", { pcbIUScale.IUTomm( at.x ), pcbIUScale.IUTomm( at.y ) } },
                                 { "layer", str( board->GetLayerName( layers[l] ) ) } } );
            }
        }

        return pts;
    };

    const double   negotiateMs = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count() - buildMs;
    nlohmann::json nets = nlohmann::json::array();
    size_t         unjoined = 0, sharing = 0;

    for( JOB& job : jobs )
    {
        job.conflicts = conflicts( job, nullptr, nullptr );
        const bool whole = job.joined == (int) job.islands.size();
        unjoined += !whole;
        sharing += job.conflicts > 0;

        nlohmann::json n = { { "net", str( UnescapeString( board->FindNet( job.net )->GetNetname() ) ) },
                             { "islands", job.islands.size() },
                             { "joined", whole },
                             { "cells_shared", job.conflicts },
                             { "islands_outside", job.outside },
                             { "width_mm", pcbIUScale.IUTomm( job.probe.width ) },
                             { "via_mm", vias ? nlohmann::json( { pcbIUScale.IUTomm( job.probe.viaDia ), pcbIUScale.IUTomm( job.probe.viaDrill ) } )
                                              : nlohmann::json() } };

        if( aArgs.value( "paths", true ) )
        {
            nlohmann::json paths = nlohmann::json::array();

            for( const std::vector<size_t>& p : job.paths )
                paths.push_back( points( p ) );

            n["paths"] = paths;
        }

        nets.push_back( n );
    }

    // where nets still share: 1 mm squares by shared cells, with the nets there
    std::vector<VECTOR2I> hotAt;

    {
        std::map<std::pair<int, int>, std::pair<size_t, std::set<int>>> spots;
        const int bin = pcbIUScale.mmToIU( 1.0 );

        for( const JOB& job : jobs )
        {
            std::vector<size_t> t, v;
            conflicts( job, &t, &v );

            for( size_t k : t )
                v.push_back( k % cells );

            for( size_t i : v )
            {
                const VECTOR2I at = grid.Centre( (int) ( i % grid.nx ), (int) ( i / grid.nx ) );
                auto& spot = spots[{ at.x / bin, at.y / bin }];
                spot.first++;
                spot.second.insert( job.net );
            }
        }

        std::vector<std::pair<size_t, std::pair<int, int>>> ranked;

        for( const auto& [key, spot] : spots )
            ranked.emplace_back( spot.first, key );

        std::sort( ranked.rbegin(), ranked.rend() );
        nlohmann::json hot = nlohmann::json::array();

        for( size_t j = 0; j < ranked.size() && j < 25; ++j )
        {
            nlohmann::json netsThere = nlohmann::json::array();

            for( int n : spots[ranked[j].second].second )
                netsThere.push_back( str( UnescapeString( board->FindNet( n )->GetNetname() ) ) );

            hotAt.emplace_back( ranked[j].second.first * bin + bin / 2, ranked[j].second.second * bin + bin / 2 );
            hot.push_back( { { "at_mm", { ranked[j].second.first + 0.5, ranked[j].second.second + 0.5 } },
                             { "cells", ranked[j].first }, { "nets", netsThere } } );
        }

        out["hot"] = hot;
        out["hot_squares"] = spots.size();
    }

    out["converged"] = converged && !unjoined && !sharing;
    out["best_round"] = bestRound;

    // a proof that not all fit: a rectangle round a hot square whose boundary more nets must cross (copper
    // inside and outside, none of it on the boundary) than tracks fit along it. The room is an upper
    // bound: the optimistic grid for the narrowest track of these nets, crossings at least that track and
    // the board's least clearance apart along the boundary (on every copper layer, planes as filled)
    if( !out["converged"].get<bool>() && aArgs.value( "cuts", true ) && !hotAt.empty() )
    {
        ORACLE_PROBE thin;
        thin.width = std::numeric_limits<int>::max();

        for( const JOB& job : jobs )
            thin.width = std::min( thin.width, job.probe.width );

        const int pitch = thin.width + std::max( ds.m_MinClearance, 0 );
        std::vector<uint8_t> shut( cells * L, 0 );

        rasteriseObstacles( board, layers, grid, outline, diag, maxError, thin, 0, { OPT }, true,
                [&]( int, int aLayer, size_t aCell, uint8_t )
                {
                    if( aLayer >= 0 )
                        shut[(size_t) aLayer * cells + aCell] = 1;
                } );

        std::vector<std::vector<BOX2I>> islandBoxes( jobs.size() );

        for( size_t j = 0; j < jobs.size(); ++j )
        {
            for( const auto& isl : jobs[j].islands )
            {
                BOX2I b = isl.front()->GetBoundingBox();

                for( BOARD_CONNECTED_ITEM* item : isl )
                    b.Merge( item->GetBoundingBox() );

                islandBoxes[j].push_back( b );
            }
        }

        struct CUT
        {
            BOX2I            box;
            int              demand = 0;
            int              room = 0;
            std::vector<int> perLayer;
            std::vector<int> nets;
        };

        CUT       best;
        bool      have = false;
        const int step = pcbIUScale.mmToIU( 0.5 );
        const int maxHalf = pcbIUScale.mmToIU( aArgs.value( "cut_max_mm", 15.0 ) );

        for( const VECTOR2I& c : hotAt )
        {
            const int cx = std::clamp( ( c.x - grid.origin.x ) / grid.cell, 0, grid.nx - 1 );
            const int cy = std::clamp( ( c.y - grid.origin.y ) / grid.cell, 0, grid.ny - 1 );

            for( int hx = step; hx <= maxHalf; hx += step )
            {
                for( int hy = step; hy <= maxHalf; hy += step )
                {
                    const int x0 = std::max( 0, cx - hx / grid.cell ), x1 = std::min( grid.nx - 1, cx + hx / grid.cell );
                    const int y0 = std::max( 0, cy - hy / grid.cell ), y1 = std::min( grid.ny - 1, cy + hy / grid.cell );

                    if( x1 - x0 < 2 || y1 - y0 < 2 )
                        continue;

                    const VECTOR2I a = grid.Centre( x0, y0 ), b = grid.Centre( x1, y1 );
                    const BOX2I    box( a, VECTOR2I( b.x - a.x, b.y - a.y ) );
                    CUT            cut;
                    cut.box = box;

                    for( size_t j = 0; j < jobs.size(); ++j )
                    {
                        bool in = false, outside = false, across = false;

                        for( const BOX2I& ib : islandBoxes[j] )
                        {
                            if( ib.GetLeft() > box.GetLeft() && ib.GetRight() < box.GetRight() && ib.GetTop() > box.GetTop()
                                && ib.GetBottom() < box.GetBottom() )
                                in = true;
                            else if( !ib.Intersects( box ) )
                                outside = true;
                            else
                                across = true;
                        }

                        if( in && outside && !across )
                        {
                            cut.demand++;
                            cut.nets.push_back( (int) j );
                        }
                    }

                    if( cut.demand == 0 )
                        continue;

                    // the boundary as one ring of cells; per layer the open runs, each fitting len / pitch + 1
                    std::vector<size_t> ring;

                    for( int x = x0; x < x1; ++x )
                        ring.push_back( (size_t) y0 * grid.nx + x );

                    for( int y = y0; y < y1; ++y )
                        ring.push_back( (size_t) y * grid.nx + x1 );

                    for( int x = x1; x > x0; --x )
                        ring.push_back( (size_t) y1 * grid.nx + x );

                    for( int y = y1; y > y0; --y )
                        ring.push_back( (size_t) y * grid.nx + x0 );

                    for( int l = 0; l < L; ++l )
                    {
                        const uint8_t* sh = shut.data() + (size_t) l * cells;
                        size_t         first = 0;

                        while( first < ring.size() && !sh[ring[first]] )
                            first++;

                        int fit = 0;

                        if( first == ring.size() )
                        {
                            fit = (int) ( (int64_t) ring.size() * grid.cell / pitch );
                        }
                        else
                        {
                            size_t run = 0;

                            for( size_t k = 1; k <= ring.size(); ++k )
                            {
                                const size_t i = ring[( first + k ) % ring.size()];

                                if( !sh[i] && k < ring.size() )
                                {
                                    run++;
                                    continue;
                                }

                                if( run )
                                    fit += (int) ( (int64_t) run * grid.cell / pitch ) + 1;

                                run = 0;
                            }
                        }

                        cut.perLayer.push_back( fit );
                        cut.room += fit;
                    }

                    if( !have || cut.demand - cut.room > best.demand - best.room )
                    {
                        best = cut;
                        have = true;
                    }
                }
            }
        }

        if( have )
        {
            nlohmann::json perLayer = nlohmann::json::object(), demanded = nlohmann::json::array();

            for( int l = 0; l < L; ++l )
                perLayer[str( board->GetLayerName( layers[l] ) )] = best.perLayer[l];

            for( int j : best.nets )
                demanded.push_back( str( UnescapeString( board->FindNet( jobs[j].net )->GetNetname() ) ) );

            out["cut"] = { { "box_mm", { pcbIUScale.IUTomm( best.box.GetLeft() ), pcbIUScale.IUTomm( best.box.GetTop() ),
                                         pcbIUScale.IUTomm( best.box.GetRight() ), pcbIUScale.IUTomm( best.box.GetBottom() ) } },
                           { "must_cross", best.demand },
                           { "room", best.room },
                           { "room_per_layer", perLayer },
                           { "pitch_mm", pcbIUScale.IUTomm( pitch ) },
                           { "proves_impossible", best.demand > best.room },
                           { "nets", demanded } };
        }
    }
    out["nets_unjoined"] = unjoined;
    out["nets_sharing"] = sharing;
    out["rounds"] = rounds;
    out["stopped"] = stopped;
    out["work_m"] = work / 1e6;
    out["result"] = nets;
    out["ms"] = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();
    out["build_ms"] = buildMs;
    out["negotiate_ms"] = negotiateMs;

    if( !region.empty() )
        out["region"] = { { "boxes", region.size() }, { "cells", cells } };

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_route_oracle_joint",
                   "Whether every connection still open on the board fits at once, decided apart from the router: "
                   "each net's islands are joined on the pessimistic grid of pcb_route_oracle (a track of the "
                   "rules' narrowest width for the net, its via), all nets negotiating for room round by round "
                   "(a cell another net's new track or via keeps out costs more each round it stays shared). "
                   "converged: every net joined and no cell shared, so the paths laid together keep the rules' "
                   "clearances (lay them and run DRC to prove it). Otherwise per net: joined, cells_shared. "
                   "result[]: net, islands, joined, cells_shared, width_mm, via_mm, paths (per joined island: "
                   "[{at, layer}], a layer change is a via); rounds[]: nets_sharing, cells_shared, nets_unjoined; "
                   "build_ms (the grids), negotiate_ms",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"string"},"description":"only these nets (default every net with more than one island)"},
                        "region":{"type":"array","items":{"type":"array","items":{"type":"number"},"minItems":4,"maxItems":4},"description":"boxes [x0, y0, x1, y1] mm: the grid covers them only, the copper outside stays fixed; a net joins its islands that reach a box (result islands_outside: those that do not), its paths inside the boxes; answer region {boxes, cells}"},
                        "vias":{"type":"boolean","default":true},
                        "zones_yield":{"type":"boolean","default":true,"description":"zone fills shut no via (a refill clears round it); tracks keep out of them"},
                        "cell_mm":{"type":"number","default":0.05,"minimum":0.005},
                        "max_cells_m":{"type":"number","default":6,"description":"millions of cells per layer at most: the cell grows to fit"},
                        "margin_mm":{"type":"number","default":3,"description":"a net's search window round its copper (the whole board when it fails)"},
                        "via_cost_mm":{"type":"number","default":1},
                        "rounds":{"type":"integer","default":40},
                        "present":{"type":"number","default":0.5},
                        "present_growth":{"type":"number","default":1.6},
                        "present_max":{"type":"number","default":1000,"description":"the present factor grows to this at most"},
                        "history":{"type":"number","default":1},
                        "reroute":{"type":"string","enum":["all","sharing"],"default":"all","description":"each round reroute every net, or only those that share"},
                        "max_work_m":{"type":"number","default":20000,"description":"millions of search steps at most"},
                        "paths":{"type":"boolean","default":true},
                        "claim_slack_mm":{"type":"number","minimum":0,"description":"the margin on each claim for centres off the true line (default a cell's diagonal: converged paths then keep the clearances; less: only DRC of the laid paths proves it)"},
                        "cuts":{"type":"boolean","default":true,"description":"when not converged: the rectangle round a hot square whose boundary most outnumbers the tracks that fit (cut.proves_impossible)"},
                        "cut_max_mm":{"type":"number","default":15}},
                        "required":[]})json"_json,
                   false, h_pcb_route_oracle_joint, 1800 );
