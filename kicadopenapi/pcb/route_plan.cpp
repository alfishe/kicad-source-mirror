#include "route_plan.h"
#include "route_raster.h"
#include "kopenapi_pcb.h"

#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <footprint.h>
#include <geometry/shape.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_rect.h>
#include <geometry/shape_segment.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <trigo.h>
#include <zone.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <optional>
#include <functional>
#include <queue>
#include <set>
#include <unordered_map>

namespace
{

/// @brief Soft copper over the cells of a grid: the first owner of a cell here, any others in more
struct OWNERS
{
    std::vector<int32_t>                             first;
    std::unordered_map<size_t, std::vector<int32_t>> more;

    void Add( size_t aIndex, int32_t aOwner )
    {
        int32_t& o = first[aIndex];

        if( o < 0 )
        {
            o = aOwner;
        }
        else if( o != aOwner )
        {
            std::vector<int32_t>& m = more[aIndex];

            if( std::find( m.begin(), m.end(), aOwner ) == m.end() )
                m.push_back( aOwner );
        }
    }

    template <typename F>
    void ForEach( size_t aIndex, F aF ) const
    {
        if( first[aIndex] < 0 )
            return;

        aF( first[aIndex] );

        if( auto it = more.find( aIndex ); it != more.end() )
        {
            for( int32_t o : it->second )
                aF( o );
        }
    }
};


/// @brief The A* open list: entries leave in the order of ( f, state ), one bucket of states per f
class OPEN_LIST
{
public:
    bool Empty() const { return m_count == 0; }

    void Push( int aF, int aState )
    {
        if( m_buckets.empty() )
            m_base = aF;

        if( aF < m_base )
        {
            m_buckets.insert( m_buckets.begin(), (size_t) ( m_base - aF ), {} );
            m_first += (size_t) ( m_base - aF );
            m_base = aF;
        }

        const size_t i = (size_t) ( aF - m_base );

        if( i >= m_buckets.size() )
            m_buckets.resize( i + 1 );

        std::vector<int>& bucket = m_buckets[i];
        bucket.push_back( aState );
        std::push_heap( bucket.begin(), bucket.end(), std::greater<int>() );
        m_first = std::min( m_first, i );
        m_count++;
    }

    /// @brief The least entry, taken off the list; the list must not be empty
    std::pair<int, int> Pop()
    {
        while( m_buckets[m_first].empty() )
            m_first++;

        std::vector<int>& bucket = m_buckets[m_first];
        std::pop_heap( bucket.begin(), bucket.end(), std::greater<int>() );
        const int state = bucket.back();
        bucket.pop_back();
        m_count--;
        return { m_base + (int) m_first, state };
    }

private:
    std::vector<std::vector<int>> m_buckets;   ///< a min-heap of states per f, from m_base on
    int                           m_base = 0;
    size_t                        m_first = 0;   ///< no entry below this bucket
    size_t                        m_count = 0;
};


constexpr int DX[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
constexpr int DY[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

} // namespace


ROUTE_PLAN PlanRoute( const ROUTE_PLAN_REQUEST& aReq )
{
    const auto started = std::chrono::steady_clock::now();
    ROUTE_PLAN plan;

    BOARD*                       board = aReq.board;
    const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
    std::shared_ptr<DRC_ENGINE>  drc = ds.m_DRCEngine;
    const int                    net = aReq.from->GetNetCode();
    const int                    halfWidth = aReq.trackWidth / 2;
    const int                    viaRadius = aReq.viaDiameter / 2;
    const bool                   vias = aReq.viaDiameter > 0 && aReq.layers.size() > 1;
    const int                    L = (int) aReq.layers.size();

    auto finish = [&]()
    {
        plan.ms = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();
        return plan;
    };

    if( L == 0 || aReq.trackWidth <= 0 )
        return finish();

    // The box: both ends with room around them for a detour, inside the board
    const VECTOR2I fromPos = aReq.from->GetPosition();
    BOX2I          region( fromPos, VECTOR2I( 0, 0 ) );
    region.Merge( aReq.toPos );

    const int span = std::max( region.GetWidth(), region.GetHeight() );
    region.Inflate( std::max( pcbIUScale.mmToIU( 3.0 ), span * 3 / 10 ) );

    const BOX2I edges = board->GetBoardEdgesBoundingBox();
    const bool  hasEdges = edges.GetWidth() > 0 && edges.GetHeight() > 0;

    if( hasEdges && aReq.wholeBoard )
        region = edges;
    else if( hasEdges )
        region = region.Intersect( edges );

    plan.wholeBoard = aReq.wholeBoard;

    // A cell of about a quarter of a track and its clearance (a fine-pitch pad's way out between its
    // neighbours stays open), on a 0.05 mm step, coarser when the box is large
    const int step = pcbIUScale.mmToIU( 0.05 );
    int       cell = std::clamp( ( aReq.trackWidth + std::max( ds.m_MinClearance, 1 ) ) / 4 / step * step, step,
                                 pcbIUScale.mmToIU( 0.25 ) );

    while( (double) region.GetWidth() / cell * region.GetHeight() / cell * L > 6e6 )
        cell *= 2;

    PLAN_GRID grid;
    grid.origin = region.GetOrigin();
    grid.cell = cell;
    grid.nx = region.GetWidth() / cell + 1;
    grid.ny = region.GetHeight() / cell + 1;
    plan.cell = cell;

    // the box's sides that cut the board (beyond them the board goes on)
    const bool cutSide[4] = { hasEdges && region.GetLeft() > edges.GetLeft() + cell, hasEdges && region.GetTop() > edges.GetTop() + cell,
                              hasEdges && region.GetRight() < edges.GetRight() - cell,
                              hasEdges && region.GetBottom() < edges.GetBottom() - cell };
    bool       reachedCut = false;

    const size_t         cells = (size_t) grid.nx * grid.ny;
    std::vector<uint8_t> track( cells * L, 0 );   // a track's centre may not be here, per layer
    std::vector<uint8_t> via( cells, 0 );         // a via's centre may not be here
    std::vector<uint8_t> rowsVia( cells, 0 );     // over a chip's rows of pins (the ends' own cells cleared)
    const int            slack = cell / 4;        // a straight run between cell centres
    // within 0.8 mm of another net's pad: that pad's way out, which a plan passing there would take
    std::vector<uint8_t> nearPad( cells * L, 0 );
    const int            escape = pcbIUScale.mmToIU( 0.8 );
    constexpr int        ESCAPE_COST = 30;

    // soft copper marks owners instead of blocking
    const bool                                 anySoft = !aReq.soft.empty();
    OWNERS                                     trackSoft{ std::vector<int32_t>( anySoft ? cells * L : 0, -1 ), {} };
    OWNERS                                     viaSoft{ std::vector<int32_t>( anySoft ? cells : 0, -1 ), {} };
    std::unordered_map<const BOARD_ITEM*, int> softOf;
    int                                        softNow = -1;   // the owner of the copper being marked, -1 hard

    for( size_t k = 0; k < aReq.soft.size(); ++k )
    {
        for( BOARD_CONNECTED_ITEM* item : aReq.soft[k].copper )
            softOf[item] = (int) k;
    }

    auto put = [&]( const SHAPE& aShape, int aInflate, std::vector<uint8_t>& aHard, OWNERS& aSoft, size_t aOffset )
    {
        if( softNow < 0 )
        {
            grid.Mark( aShape, aInflate, aHard, aOffset );
            return;
        }

        grid.Each( aShape, aInflate, aOffset, [&]( size_t i ) { return aHard[i] != 0; },
                   [&]( size_t i ) { aSoft.Add( i, softNow ); } );
    };

    auto softCost = [&]( const OWNERS& aSoft, size_t aIndex )
    {
        int c = 0;

        if( anySoft )
            aSoft.ForEach( aIndex, [&]( int32_t o ) { c += aReq.soft[o].cellCost; } );

        return c;
    };

    // the board's rule for our net against an item, as DRC and the router see it
    PCB_TRACK probeTrack( board );
    probeTrack.SetFlags( ROUTER_TRANSIENT );
    probeTrack.SetNetCode( net );
    probeTrack.SetWidth( aReq.trackWidth );

    PCB_VIA probeVia( board );
    probeVia.SetFlags( ROUTER_TRANSIENT );
    probeVia.SetNetCode( net );
    probeVia.SetViaType( VIATYPE::THROUGH );
    probeVia.SetLayerPair( F_Cu, B_Cu );
    probeVia.SetWidth( PADSTACK::ALL_LAYERS, std::max( aReq.viaDiameter, 1 ) );
    probeVia.SetDrill( std::max( aReq.viaDrill, 1 ) );

    auto rule = [&]( DRC_CONSTRAINT_T aType, BOARD_ITEM* aProbe, BOARD_ITEM* aItem, PCB_LAYER_ID aLayer, int aFloor )
    {
        if( drc )
        {
            // the probe where the item is: rules by area (courtyards, rule areas) see it there
            if( PCB_TRACK* t = dynamic_cast<PCB_TRACK*>( aProbe ) )
            {
                const VECTOR2I at = aItem->GetBoundingBox().Centre();
                t->SetStart( at );
                t->SetEnd( at );
            }

            DRC_CONSTRAINT c = drc->EvalRules( aType, aProbe, aItem, aLayer );

            if( c.GetValue().HasMin() )
                return std::max( c.GetValue().Min(), aFloor );
        }

        return aFloor;
    };

    // the floor under the rules' answer: the board minimum; the net classes' clearance only without
    // a rules engine (a custom rule may ask less than the class, e.g. inside a fine-pitch courtyard)
    auto netFloor = [&]( BOARD_ITEM* aItem )
    {
        int floor = ds.m_MinClearance;

        if( drc )
            return floor;

        for( BOARD_ITEM* item : { aItem, static_cast<BOARD_ITEM*>( aReq.from ) } )
        {
            if( BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( item ) )
            {
                if( NETCLASS* nc = bci->GetEffectiveNetClass(); nc && nc->HasClearance() )
                    floor = std::max( floor, nc->GetClearance() );
            }
        }

        return floor;
    };

    const LSET copper = LSET::AllCuMask( board->GetCopperLayerCount() );

    // copper of other nets, kept with its clearance for the exact way out of each end
    struct NEAR_END
    {
        std::shared_ptr<SHAPE> shape;
        int                    clearance;
        int                    layer;
        int                    owner;   ///< soft copper's owner, -1 hard
    };

    std::vector<NEAR_END> nearEnds;
    // no way out of an end runs further than the other end
    const int reach = std::max( (int) ( fromPos - aReq.toPos ).EuclideanNorm(), 1 );

    // copper of another net (or of none) on the routing layers and, for vias, on every copper layer
    auto copperObstacle = [&]( BOARD_ITEM* aItem, const std::function<std::shared_ptr<SHAPE>( PCB_LAYER_ID )>& aShape )
    {
        const int floor = netFloor( aItem );

        for( int l = 0; l < L; ++l )
        {
            const PCB_LAYER_ID layer = aReq.layers[l];

            if( !aItem->IsOnLayer( layer ) )
                continue;

            if( std::shared_ptr<SHAPE> s = aShape( layer ) )
            {
                probeTrack.SetLayer( layer );
                const int clearance = rule( CLEARANCE_CONSTRAINT, &probeTrack, aItem, layer, floor );
                put( *s, clearance + halfWidth + slack, track, trackSoft, (size_t) l * cells );

                nearEnds.push_back( { s, clearance, l, softNow } );
            }
        }

        if( !vias )
            return;

        for( PCB_LAYER_ID layer : copper.Seq() )
        {
            if( !aItem->IsOnLayer( layer ) )
                continue;

            if( std::shared_ptr<SHAPE> s = aShape( layer ) )
            {
                // the via's copper keeps the clearance, its hole the hole clearance
                const int reach = std::max( rule( CLEARANCE_CONSTRAINT, &probeVia, aItem, layer, floor ) + viaRadius,
                                            rule( HOLE_CLEARANCE_CONSTRAINT, &probeVia, aItem, layer, ds.m_HoleClearance )
                                                    + aReq.viaDrill / 2 );
                put( *s, reach + slack, via, viaSoft, 0 );
            }
        }
    };

    auto holeObstacle = [&]( BOARD_ITEM* aItem, const SHAPE_SEGMENT& aHole, bool aNpth )
    {
        if( aNpth )
        {
            for( int l = 0; l < L; ++l )
            {
                probeTrack.SetLayer( aReq.layers[l] );
                grid.Mark( aHole, rule( HOLE_CLEARANCE_CONSTRAINT, &probeTrack, aItem, aReq.layers[l], ds.m_HoleClearance )
                                          + halfWidth + slack,
                           track, (size_t) l * cells );
            }
        }

        if( vias )
            grid.Mark( aHole, rule( HOLE_TO_HOLE_CONSTRAINT, &probeVia, aItem, F_Cu, ds.m_HoleToHoleMin ) + aReq.viaDrill / 2 + slack,
                       via );
    };

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( !fp->GetBoundingBox( false ).Intersects( region ) )
            continue;

        // no via inside a chip's ring of pins nor among its rows (as the escape spots): the pins' ways
        // out and in and their fan-out vias keep that room; an end inside the ring (an exposed pad) or
        // a part the call opens keeps it open
        if( vias && !aReq.ringOpen.count( fp ) )
        {
            int edge = INT_MAX;

            for( PAD* pad : fp->Pads() )
                edge = std::min( edge, (int) std::min( pad->GetBoundingBox().GetWidth(), pad->GetBoundingBox().GetHeight() ) );

            if( const std::optional<BOX2I> inside = KopenapiPadRingInside( fp, std::max( edge / 2, 1 ) ) )
            {
                BOX2I rows = fp->Pads().front()->GetBoundingBox();

                for( PAD* pad : fp->Pads() )
                    rows.Merge( pad->GetBoundingBox() );

                if( !inside->Contains( fromPos ) && !inside->Contains( aReq.toPos ) )
                    grid.Mark( SHAPE_RECT( rows ), viaRadius, rowsVia );
            }
        }

        for( PAD* pad : fp->Pads() )
        {
            const bool mine = pad->GetNetCode() == net && net > 0;

            if( !mine && pad->GetNetCode() > 0 )
            {
                for( int l = 0; l < L; ++l )
                {
                    if( pad->IsOnLayer( aReq.layers[l] ) && pad->FlashLayer( aReq.layers[l] ) )
                        grid.Mark( *pad->GetEffectiveShape( aReq.layers[l] ), escape, nearPad, (size_t) l * cells );
                }
            }

            if( !mine )
            {
                copperObstacle( pad,
                                [&]( PCB_LAYER_ID aLayer ) -> std::shared_ptr<SHAPE>
                                {
                                    if( !pad->FlashLayer( aLayer ) )
                                        return nullptr;

                                    return pad->GetEffectiveShape( aLayer );
                                } );
            }

            if( pad->HasHole() )
            {
                std::shared_ptr<SHAPE_SEGMENT> hole = pad->GetEffectiveHoleShape();
                holeObstacle( pad, *hole, pad->GetAttribute() == PAD_ATTRIB::NPTH );
            }
        }

        for( BOARD_ITEM* item : fp->GraphicalItems() )
        {
            if( item->Type() == PCB_SHAPE_T && ( item->GetLayerSet() & copper ).any() )
                copperObstacle( item, [&]( PCB_LAYER_ID aLayer ) { return item->GetEffectiveShape( aLayer ); } );
        }
    }

    for( PCB_TRACK* t : board->Tracks() )
    {
        if( !t->GetBoundingBox().Intersects( region ) )
            continue;

        if( auto it = softOf.find( t ); it != softOf.end() && ( t->GetNetCode() != net || net <= 0 ) )
            softNow = it->second;

        if( t->GetNetCode() != net || net <= 0 )
        {
            copperObstacle( t,
                            [&]( PCB_LAYER_ID aLayer ) -> std::shared_ptr<SHAPE>
                            {
                                if( t->Type() == PCB_VIA_T && !static_cast<PCB_VIA*>( t )->FlashLayer( aLayer ) )
                                    return nullptr;

                                return t->GetEffectiveShape( aLayer );
                            } );
        }

        if( t->Type() == PCB_VIA_T && vias )
        {
            std::shared_ptr<SHAPE_SEGMENT> hole = static_cast<PCB_VIA*>( t )->GetEffectiveHoleShape();
            put( *hole, ds.m_HoleToHoleMin + aReq.viaDrill / 2 + slack, via, viaSoft, 0 );
        }

        softNow = -1;
    }

    for( BOARD_ITEM* item : board->Drawings() )
    {
        if( item->Type() != PCB_SHAPE_T || !item->GetBoundingBox().Intersects( region ) )
            continue;

        if( item->IsOnLayer( Edge_Cuts ) )
        {
            const std::shared_ptr<SHAPE> s = item->GetEffectiveShape( Edge_Cuts );
            const int                    edge = std::max( ds.m_CopperEdgeClearance, 0 );

            for( int l = 0; l < L; ++l )
                grid.Mark( *s, edge + halfWidth + slack, track, (size_t) l * cells );

            if( vias )
                grid.Mark( *s, edge + viaRadius + slack, via );
        }
        else if( ( item->GetLayerSet() & copper ).any() )
        {
            copperObstacle( item, [&]( PCB_LAYER_ID aLayer ) { return item->GetEffectiveShape( aLayer ); } );
        }
    }

    std::vector<ZONE*> zones( board->Zones().begin(), board->Zones().end() );

    for( FOOTPRINT* fp : board->Footprints() )
        zones.insert( zones.end(), fp->Zones().begin(), fp->Zones().end() );

    for( ZONE* zone : zones )
    {
        if( !zone->GetBoundingBox().Intersects( region ) )
            continue;

        if( zone->GetIsRuleArea() )
        {
            // a footprint's zone keeps its outline in the footprint's own coordinates
            const SHAPE_POLY_SET area = zone->GetBoardOutline();

            for( int l = 0; l < L; ++l )
            {
                if( zone->GetDoNotAllowTracks() && zone->IsOnLayer( aReq.layers[l] ) )
                    grid.Mark( area, halfWidth + slack, track, (size_t) l * cells );
            }

            if( vias && zone->GetDoNotAllowVias() && ( zone->GetLayerSet() & copper ).any() )
                grid.Mark( area, viaRadius + slack, via );

            continue;
        }

        if( zone->GetNetCode() == net && net > 0 )
            continue;

        copperObstacle( zone,
                        [&]( PCB_LAYER_ID aLayer ) -> std::shared_ptr<SHAPE>
                        {
                            std::shared_ptr<SHAPE_POLY_SET> fill = zone->GetFilledPolysList( aLayer );

                            if( !fill || fill->IsEmpty() )
                                return nullptr;

                            return std::make_shared<SHAPE_POLY_SET>( *fill );
                        } );
    }

    for( const auto& [layer, shape] : aReq.keepOut )
    {
        for( int l = 0; l < L; ++l )
        {
            if( aReq.layers[l] != layer || !shape.BBox().Intersects( region ) )
                continue;

            grid.Mark( shape, halfWidth + slack, track, (size_t) l * cells );

            nearEnds.push_back( { std::make_shared<SHAPE_SEGMENT>( shape ), 0, l, -1 } );
        }

        // a through via stands in the way out on every layer, at the clearance the rules ask of our
        // net's via there
        if( vias && shape.BBox().Intersects( region ) )
        {
            const int clearance = KopenapiRuleVia( board, net, shape.BBox().Centre(), layer ).clearance;
            grid.Mark( shape, clearance + viaRadius + slack, via );
        }
    }

    // no via on the end pads themselves: the route leaves a pad on a track
    if( vias )
    {
        for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( aReq.from ), aReq.to } )
        {
            for( PCB_LAYER_ID layer : LSET( end->GetLayerSet() & copper ).Seq() )
            {
                if( std::shared_ptr<SHAPE> s = end->GetEffectiveShape( layer ) )
                    grid.Mark( *s, viaRadius + ds.m_MinClearance, via );
            }
        }
    }

    // outside the board outline
    SHAPE_POLY_SET outline;

    if( board->GetBoardPolygonOutlines( outline, true ) && outline.OutlineCount() > 0 )
    {
        std::vector<uint8_t> inside( cells, 0 );
        grid.Mark( outline, 0, inside );

        for( size_t i = 0; i < cells; ++i )
        {
            if( inside[i] )
                continue;

            for( int l = 0; l < L; ++l )
                track[(size_t) l * cells + i] = 1;

            via[i] = 1;
        }
    }

    // Ends: the cell of each end on each of its layers (free whatever its neighbours' clearance says:
    // the router leaves a pad along its own copper)
    std::vector<int> starts, goals;
    int              sx, sy, gx, gy;

    if( !grid.CellOf( fromPos, sx, sy ) || !grid.CellOf( aReq.toPos, gx, gy ) )
        return finish();

    std::vector<uint8_t> isGoal( cells * L, 0 );

    // the ends' own copper may take their via, the rest of the rows not
    {
        std::vector<uint8_t> own( cells, 0 );

        for( BOARD_CONNECTED_ITEM* end : { static_cast<BOARD_CONNECTED_ITEM*>( aReq.from ), aReq.to } )
        {
            const PAD*         pad = dynamic_cast<const PAD*>( end );
            const PCB_LAYER_ID layer = pad ? pad->GetPrincipalLayer() : end->GetLayer();

            if( std::shared_ptr<SHAPE> s = end->GetEffectiveShape( layer ) )
                grid.Mark( *s, 0, own );
        }

        own[grid.Index( sx, sy )] = 1;
        own[grid.Index( gx, gy )] = 1;

        for( size_t c = 0; c < cells; ++c )
            via[c] |= rowsVia[c] && !own[c];
    }

    for( int l = 0; l < L; ++l )
    {
        if( aReq.from->IsOnLayer( aReq.layers[l] ) )
        {
            starts.push_back( l * (int) cells + grid.Index( sx, sy ) );
            track[(size_t) l * cells + grid.Index( sx, sy )] = 0;
        }

        if( aReq.to->IsOnLayer( aReq.layers[l] ) )
        {
            goals.push_back( l * (int) cells + grid.Index( gx, gy ) );
            track[(size_t) l * cells + grid.Index( gx, gy )] = 0;
            isGoal[(size_t) l * cells + grid.Index( gx, gy )] = 1;
        }
    }

    // The way out of an end on the grid can be shut by the grid alone (a fine-pitch pad's way out
    // between its neighbours, a track ending between two others at a track and its clearance, off the
    // cell centres): along each axis of a pad, or on along a track past its end, the first free cell a
    // straight run from the end reaches with its clearance to every other net's copper
    struct ESCAPE
    {
        VECTOR2I         at;
        int              cost;
        std::vector<int> owners;
    };

    auto escapesOf = [&]( BOARD_CONNECTED_ITEM* aEnd, const VECTOR2I& aPos )
    {
        std::vector<std::pair<int, ESCAPE>> out;   // state, escape
        PAD*                                pad = dynamic_cast<PAD*>( aEnd );

        // a track's end: the way out goes on along the track past its end
        auto runFrom = [&]( int aLayer, const VECTOR2D& aDir, double aFrom )
        {
            // the copper beside the whole run, once; each step then checks the piece it adds
            const VECTOR2I runEnd = aPos + VECTOR2I( KiROUND( aDir.x * ( aFrom + reach ) ), KiROUND( aDir.y * ( aFrom + reach ) ) );
            BOX2I          lane( aPos, VECTOR2I( 0, 0 ) );
            lane.Merge( runEnd );
            lane.Inflate( aReq.trackWidth / 2 + 1 );
            std::vector<const NEAR_END*> beside;

            for( const NEAR_END& n : nearEnds )
            {
                // not BBox( clearance ): a compound shape (a pad) leaves its clearance out
                BOX2I box = n.shape->BBox();
                box.Inflate( n.clearance );

                if( n.layer == aLayer && box.Intersects( lane ) )
                    beside.push_back( &n );
            }

            std::vector<int> owners;
            VECTOR2I         last = aPos;

            for( double d = aFrom; d <= aFrom + reach; d += cell )
            {
                const VECTOR2I at = aPos + VECTOR2I( KiROUND( aDir.x * d ), KiROUND( aDir.y * d ) );
                int            x, y;

                if( !grid.CellOf( at, x, y ) )
                    break;

                const SHAPE_SEGMENT run( last, at, aReq.trackWidth );
                bool                clean = true;
                last = at;

                for( const NEAR_END* np : beside )
                {
                    const NEAR_END& n = *np;

                    if( !n.shape->Collide( &run, n.clearance ) )
                        continue;

                    if( n.owner < 0 )
                    {
                        clean = false;
                        break;
                    }

                    if( std::find( owners.begin(), owners.end(), n.owner ) == owners.end() )
                        owners.push_back( n.owner );
                }

                if( !clean )
                    break;   // a longer run holds this one

                const size_t state = (size_t) aLayer * cells + grid.Index( x, y );

                if( track[state] )
                    continue;

                int cost = (int) ( 10.0 * d / cell );

                for( int o : owners )
                    cost += aReq.soft[o].cellCost * 8;

                out.push_back( { (int) state, ESCAPE{ at, cost, owners } } );
                break;
            }
        };

        if( !pad )
        {
            if( aEnd->Type() != PCB_TRACE_T )
                return out;

            const PCB_TRACK* t = static_cast<const PCB_TRACK*>( aEnd );
            const bool       atEnd = ( t->GetEnd() - aPos ).EuclideanNorm() <= ( t->GetStart() - aPos ).EuclideanNorm();
            const VECTOR2I   along = atEnd ? t->GetEnd() - t->GetStart() : t->GetStart() - t->GetEnd();

            if( along == VECTOR2I( 0, 0 ) )
                return out;

            const VECTOR2D dir = VECTOR2D( along ) / along.EuclideanNorm();

            for( int l = 0; l < L; ++l )
            {
                if( aReq.layers[l] == t->GetLayer() )
                    runFrom( l, dir, cell );
            }

            return out;
        }

        for( int l = 0; l < L; ++l )
        {
            const PCB_LAYER_ID layer = aReq.layers[l];

            if( !pad->IsOnLayer( layer ) || !pad->FlashLayer( layer ) )
                continue;

            const VECTOR2I size = pad->GetSize( layer );

            for( int axis = 0; axis < 4; ++axis )
            {
                VECTOR2D     dir( axis == 0 ? 1 : axis == 1 ? -1 : 0, axis == 2 ? 1 : axis == 3 ? -1 : 0 );
                const double half = ( axis < 2 ? size.x : size.y ) / 2.0;
                RotatePoint( dir, pad->GetOrientation() );

                runFrom( l, dir, half );
            }
        }

        return out;
    };

    std::unordered_map<int, ESCAPE> startEscapes, goalEscapes;

    for( auto& [state, esc] : escapesOf( aReq.from, fromPos ) )
    {
        if( auto it = startEscapes.find( state ); it == startEscapes.end() || it->second.cost > esc.cost )
            startEscapes[state] = esc;
    }

    for( auto& [state, esc] : escapesOf( aReq.to, aReq.toPos ) )
    {
        if( auto it = goalEscapes.find( state ); it == goalEscapes.end() || it->second.cost > esc.cost )
            goalEscapes[state] = esc;

        isGoal[state] = 1;
    }

    if( ( starts.empty() && startEscapes.empty() ) || ( goals.empty() && goalEscapes.empty() ) )
        return finish();

    // A*: a straight step 10, a diagonal 14 (in cells), a turn 6, beside another net's pad 30 more,
    // a via as much as the track the request prices it at (at least a step)
    const double         viaTrack = aReq.viaCost > 0 ? aReq.viaCost : pcbIUScale.mmToIU( KopenapiViaCostMm( board, {}, net ) );
    const int            viaCost = std::max( 10, (int) ( viaTrack * 10 / cell ) );
    const size_t         states = cells * L;
    std::vector<int>     g( states, INT_MAX );
    std::vector<int>     parent( states, -1 );
    std::vector<int8_t>  dirIn( states, -1 );

    auto heuristicAt = [&]( int aX, int aY )
    {
        const int dx = std::abs( aX - gx );
        const int dy = std::abs( aY - gy );
        return 10 * std::max( dx, dy ) + 4 * std::min( dx, dy );
    };

    auto heuristic = [&]( int aState )
    {
        const int c = aState % (int) cells;
        return heuristicAt( c % grid.nx, c / grid.nx );
    };

    OPEN_LIST open;

    for( int s : starts )
    {
        g[s] = 0;
        open.Push( heuristic( s ), s );
    }

    for( const auto& [s, esc] : startEscapes )
    {
        if( esc.cost < g[s] )
        {
            g[s] = esc.cost;
            open.Push( g[s] + heuristic( s ), s );
        }
    }

    int       reached = -1;
    const long limit = (long) states * 4;

    while( !open.Empty() && plan.expanded < limit )
    {
        if( aReq.stop && ( plan.expanded & 4095 ) == 0 && aReq.stop() )
        {
            plan.stopped = true;
            break;
        }

        const std::pair<int, int> top = open.Pop();
        const int                 f = top.first;
        const int                 s = top.second;

        const int l = s / (int) cells;
        const int c = s % (int) cells;
        const int x = c % grid.nx;
        const int y = c / grid.nx;

        if( f - heuristicAt( x, y ) > g[s] )
            continue;   // a stale entry

        plan.expanded++;

        if( isGoal[s] )
        {
            reached = s;
            break;
        }

        if( ( x == 0 && cutSide[0] ) || ( y == 0 && cutSide[1] ) || ( x == grid.nx - 1 && cutSide[2] )
            || ( y == grid.ny - 1 && cutSide[3] ) )
        {
            reachedCut = true;
        }

        // aNext lies at cell ( aX, aY )
        auto relax = [&]( int aNext, int aX, int aY, int aCost, int8_t aDir )
        {
            if( g[s] + aCost < g[aNext] )
            {
                g[aNext] = g[s] + aCost;
                parent[aNext] = s;
                dirIn[aNext] = aDir;
                open.Push( g[aNext] + heuristicAt( aX, aY ), aNext );
            }
        };

        for( int d = 0; d < 8; ++d )
        {
            const int x2 = x + DX[d];
            const int y2 = y + DY[d];

            if( x2 < 0 || y2 < 0 || x2 >= grid.nx || y2 >= grid.ny )
                continue;

            const size_t base = (size_t) l * cells;

            if( track[base + grid.Index( x2, y2 )] )
                continue;

            // a diagonal step only where both cells beside it are free: the run keeps its clearance
            if( ( d & 1 ) && ( track[base + grid.Index( x2, y )] || track[base + grid.Index( x, y2 )] ) )
                continue;

            const int turn = ( dirIn[s] >= 0 && dirIn[s] != d ) ? 6 : 0;
            const int nearby = nearPad[base + grid.Index( x2, y2 )] ? ESCAPE_COST : 0;
            const int cross = softCost( trackSoft, base + grid.Index( x2, y2 ) );
            relax( l * (int) cells + grid.Index( x2, y2 ), x2, y2, ( ( d & 1 ) ? 14 : 10 ) + turn + nearby + cross,
                   (int8_t) d );
        }

        if( vias && !via[c] )
        {
            for( int l2 = 0; l2 < L; ++l2 )
            {
                if( l2 != l && !track[(size_t) l2 * cells + c] )
                {
                    relax( l2 * (int) cells + c, x, y,
                           viaCost + softCost( viaSoft, c ) + softCost( trackSoft, (size_t) l2 * cells + c ), -1 );
                }
            }
        }
    }

    if( reached < 0 )
    {
        // the room of the target in the box (the states a track or a via of the net reaches from it),
        // searched as far as the start's: closed inside the box, the target is shut in, also over the
        // whole board
        constexpr size_t ROOM = 20000;
        const size_t     cap = std::max( ROOM, (size_t) plan.expanded );
        size_t           room = 0;
        bool             roomCut = false;

        if( open.Empty() )
        {
            std::vector<uint8_t> seen( states, 0 );
            std::vector<int>     todo;

            for( int s : goals )
                todo.push_back( s );

            for( const auto& [s, esc] : goalEscapes )
                todo.push_back( s );

            for( int s : todo )
                seen[s] = 1;

            while( !todo.empty() && room <= cap && !roomCut )
            {
                const int s = todo.back();
                todo.pop_back();
                room++;

                const int l = s / (int) cells, c = s % (int) cells, x = c % grid.nx, y = c / grid.nx;

                if( ( x == 0 && cutSide[0] ) || ( y == 0 && cutSide[1] ) || ( x == grid.nx - 1 && cutSide[2] )
                    || ( y == grid.ny - 1 && cutSide[3] ) )
                {
                    roomCut = true;
                }

                for( int d = 0; d < 8; d += 2 )
                {
                    const int x2 = x + DX[d], y2 = y + DY[d];

                    if( x2 < 0 || y2 < 0 || x2 >= grid.nx || y2 >= grid.ny )
                        continue;

                    const int n = l * (int) cells + grid.Index( x2, y2 );

                    if( !track[n] && !seen[n] )
                    {
                        seen[n] = 1;
                        todo.push_back( n );
                    }
                }

                if( vias && !via[c] )
                {
                    for( int l2 = 0; l2 < L; ++l2 )
                    {
                        const int n = l2 * (int) cells + c;

                        if( !track[n] && !seen[n] )
                        {
                            seen[n] = 1;
                            todo.push_back( n );
                        }
                    }
                }
            }
        }

        const bool targetShut = open.Empty() && room <= cap && !roomCut;

        // the start's room goes on past the box: once more over the whole board
        if( open.Empty() && reachedCut && !aReq.wholeBoard && !targetShut )
        {
            ROUTE_PLAN_REQUEST wide = aReq;
            wide.wholeBoard = true;

            ROUTE_PLAN again = PlanRoute( wide );
            again.ms += std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();
            return again;
        }

        // all the start reaches searched: is it the start's room or the target's that is shut
        if( open.Empty() )
        {
            if( targetShut && (size_t) plan.expanded > room )
            {
                plan.boxedEnd = 2;
                plan.boxedRoom = room;
            }
            else if( (size_t) plan.expanded <= ROOM )
            {
                plan.boxedEnd = 1;
                plan.boxedRoom = (size_t) plan.expanded;
            }
        }

        return finish();
    }

    // The path, start first: a corner where the direction changes, a via where the layer does
    std::vector<int> path;

    for( int s = reached; s >= 0; s = parent[s] )
        path.push_back( s );

    std::reverse( path.begin(), path.end() );

    plan.found = true;
    plan.startLayer = aReq.layers[path.front() / (int) cells];

    // the run out of an end, when the path starts or ends at the end of one
    const auto  startEscape = startEscapes.find( path.front() );
    const auto  goalEscape = goalEscapes.find( reached );
    const bool  fromEscape = startEscape != startEscapes.end()
                            && std::find( starts.begin(), starts.end(), path.front() ) == starts.end();
    const bool  toEscape = goalEscape != goalEscapes.end() && std::find( goals.begin(), goals.end(), reached ) == goals.end();

    if( fromEscape )
    {
        plan.points.emplace_back( startEscape->second.at, std::nullopt );
        plan.length += ( startEscape->second.at - fromPos ).EuclideanNorm();
    }

    if( anySoft )
    {
        std::set<int> crossed;
        auto          take = [&]( int32_t o ) { crossed.insert( o ); };

        for( const auto& [escape, used] : { std::pair( startEscape, fromEscape ), std::pair( goalEscape, toEscape ) } )
        {
            if( used )
                crossed.insert( escape->second.owners.begin(), escape->second.owners.end() );
        }

        for( size_t k = 0; k < path.size(); ++k )
        {
            trackSoft.ForEach( path[k], take );

            if( k > 0 && path[k] / (int) cells != path[k - 1] / (int) cells )
                viaSoft.ForEach( path[k] % (int) cells, take );
        }

        plan.crossed.assign( crossed.begin(), crossed.end() );
    }

    int8_t runDir = -1;

    for( size_t k = 1; k < path.size(); ++k )
    {
        const int      prev = path[k - 1];
        const int      cur = path[k];
        const VECTOR2I at = grid.Centre( prev % (int) cells );

        if( cur / (int) cells != prev / (int) cells )
        {
            plan.points.emplace_back( at, aReq.layers[cur / (int) cells] );
            plan.vias++;
            runDir = -1;
            continue;
        }

        if( runDir >= 0 && dirIn[cur] != runDir && k > 1 )
            plan.points.emplace_back( at, std::nullopt );

        runDir = dirIn[cur];
        plan.length += ( grid.Centre( cur % (int) cells ) - at ).EuclideanNorm();
    }

    // a corner at the goal's cell is the target itself
    while( !plan.points.empty() && !plan.points.back().second
           && ( plan.points.back().first - aReq.toPos ).EuclideanNorm() < cell )
    {
        plan.points.pop_back();
    }

    if( toEscape )
    {
        plan.points.emplace_back( goalEscape->second.at, std::nullopt );
        plan.length += ( goalEscape->second.at - aReq.toPos ).EuclideanNorm();
    }

    return finish();
}
