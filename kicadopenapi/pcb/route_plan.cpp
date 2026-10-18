#include "route_plan.h"

#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <footprint.h>
#include <geometry/shape.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_segment.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <zone.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>

namespace
{

/// @brief Cells of a box of the board, row by row
struct PLAN_GRID
{
    VECTOR2I origin;
    int      cell = 1;
    int      nx = 0;
    int      ny = 0;

    int      Index( int aX, int aY ) const { return aY * nx + aX; }
    VECTOR2I Centre( int aX, int aY ) const { return origin + VECTOR2I( aX * cell, aY * cell ); }
    VECTOR2I Centre( int aIndex ) const { return Centre( aIndex % nx, aIndex / nx ); }

    bool CellOf( const VECTOR2I& aP, int& aX, int& aY ) const
    {
        aX = (int) std::lround( (double) ( aP.x - origin.x ) / cell );
        aY = (int) std::lround( (double) ( aP.y - origin.y ) / cell );
        return aX >= 0 && aY >= 0 && aX < nx && aY < ny;
    }

    /// @brief Every cell whose centre is within aInflate of the shape
    void Mark( const SHAPE& aShape, int aInflate, std::vector<uint8_t>& aCells, size_t aOffset = 0 ) const
    {
        BOX2I bb = aShape.BBox();   // BBox( clearance ) of a compound shape (a pad) leaves the clearance out
        bb.Inflate( aInflate );
        const int   x0 = std::max( 0, (int) std::floor( (double) ( bb.GetLeft() - origin.x ) / cell ) );
        const int   y0 = std::max( 0, (int) std::floor( (double) ( bb.GetTop() - origin.y ) / cell ) );
        const int   x1 = std::min( nx - 1, (int) std::ceil( (double) ( bb.GetRight() - origin.x ) / cell ) );
        const int   y1 = std::min( ny - 1, (int) std::ceil( (double) ( bb.GetBottom() - origin.y ) / cell ) );

        for( int y = y0; y <= y1; ++y )
        {
            for( int x = x0; x <= x1; ++x )
            {
                uint8_t& c = aCells[aOffset + Index( x, y )];

                if( !c && aShape.Collide( Centre( x, y ), aInflate ) )
                    c = 1;
            }
        }
    }
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

    BOX2I edges = board->GetBoardEdgesBoundingBox();

    if( edges.GetWidth() > 0 && edges.GetHeight() > 0 )
        region = region.Intersect( edges );

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

    const size_t         cells = (size_t) grid.nx * grid.ny;
    std::vector<uint8_t> track( cells * L, 0 );   // a track's centre may not be here, per layer
    std::vector<uint8_t> via( cells, 0 );         // a via's centre may not be here
    const int            slack = cell / 4;        // a straight run between cell centres
    // within 0.8 mm of another net's pad: that pad's way out, which a plan passing there would take
    std::vector<uint8_t> nearPad( cells * L, 0 );
    const int            escape = pcbIUScale.mmToIU( 0.8 );
    constexpr int        ESCAPE_COST = 30;

    // the board's rule for our net against an item, as DRC and the router see it
    PCB_TRACK probeTrack( board );
    probeTrack.SetNetCode( net );
    probeTrack.SetWidth( aReq.trackWidth );

    PCB_VIA probeVia( board );
    probeVia.SetNetCode( net );
    probeVia.SetViaType( VIATYPE::THROUGH );
    probeVia.SetLayerPair( F_Cu, B_Cu );
    probeVia.SetWidth( PADSTACK::ALL_LAYERS, std::max( aReq.viaDiameter, 1 ) );
    probeVia.SetDrill( std::max( aReq.viaDrill, 1 ) );

    auto rule = [&]( DRC_CONSTRAINT_T aType, BOARD_ITEM* aProbe, BOARD_ITEM* aItem, PCB_LAYER_ID aLayer, int aFloor )
    {
        if( drc )
        {
            DRC_CONSTRAINT c = drc->EvalRules( aType, aProbe, aItem, aLayer );

            if( c.GetValue().HasMin() )
                return std::max( c.GetValue().Min(), aFloor );
        }

        return aFloor;
    };

    auto netFloor = [&]( BOARD_ITEM* aItem )
    {
        int floor = ds.m_MinClearance;

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
                grid.Mark( *s, rule( CLEARANCE_CONSTRAINT, &probeTrack, aItem, layer, floor ) + halfWidth + slack, track,
                           (size_t) l * cells );
            }
        }

        if( !vias )
            return;

        for( PCB_LAYER_ID layer : copper.Seq() )
        {
            if( !aItem->IsOnLayer( layer ) )
                continue;

            if( std::shared_ptr<SHAPE> s = aShape( layer ) )
                grid.Mark( *s, rule( CLEARANCE_CONSTRAINT, &probeVia, aItem, layer, floor ) + viaRadius + slack, via );
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
            grid.Mark( *hole, ds.m_HoleToHoleMin + aReq.viaDrill / 2 + slack, via );
        }
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

    for( ZONE* zone : board->Zones() )
    {
        if( !zone->GetBoundingBox().Intersects( region ) )
            continue;

        if( zone->GetIsRuleArea() )
        {
            for( int l = 0; l < L; ++l )
            {
                if( zone->GetDoNotAllowTracks() && zone->IsOnLayer( aReq.layers[l] ) )
                    grid.Mark( *zone->Outline(), halfWidth + slack, track, (size_t) l * cells );
            }

            if( vias && zone->GetDoNotAllowVias() && ( zone->GetLayerSet() & copper ).any() )
                grid.Mark( *zone->Outline(), viaRadius + slack, via );

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
        for( int y = 0; y < grid.ny; ++y )
        {
            for( int x = 0; x < grid.nx; ++x )
            {
                if( outline.Contains( grid.Centre( x, y ) ) )
                    continue;

                for( int l = 0; l < L; ++l )
                    track[(size_t) l * cells + grid.Index( x, y )] = 1;

                via[grid.Index( x, y )] = 1;
            }
        }
    }

    // Ends: the cell of each end on each of its layers (free whatever its neighbours' clearance says:
    // the router leaves a pad along its own copper)
    std::vector<int> starts, goals;
    int              sx, sy, gx, gy;

    if( !grid.CellOf( fromPos, sx, sy ) || !grid.CellOf( aReq.toPos, gx, gy ) )
        return finish();

    std::vector<uint8_t> isGoal( cells * L, 0 );

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

    if( starts.empty() || goals.empty() )
        return finish();

    // A*: a straight step 10, a diagonal 14 (in cells), a turn 6, beside another net's pad 30 more,
    // a via about 3 mm of track
    const int            viaCost = std::max( 80, (int) ( (double) pcbIUScale.mmToIU( 3.0 ) * 10 / cell ) );
    const size_t         states = cells * L;
    std::vector<int>     g( states, INT_MAX );
    std::vector<int>     parent( states, -1 );
    std::vector<int8_t>  dirIn( states, -1 );

    auto heuristic = [&]( int aState )
    {
        const int c = aState % (int) cells;
        const int dx = std::abs( c % grid.nx - gx );
        const int dy = std::abs( c / grid.nx - gy );
        return 10 * std::max( dx, dy ) + 4 * std::min( dx, dy );
    };

    using ENTRY = std::pair<int, int>;   // f, state
    std::priority_queue<ENTRY, std::vector<ENTRY>, std::greater<ENTRY>> open;

    for( int s : starts )
    {
        g[s] = 0;
        open.emplace( heuristic( s ), s );
    }

    int       reached = -1;
    const long limit = (long) states * 4;

    while( !open.empty() && plan.expanded < limit )
    {
        const auto [f, s] = open.top();
        open.pop();

        if( f - heuristic( s ) > g[s] )
            continue;   // a stale entry

        plan.expanded++;

        if( isGoal[s] )
        {
            reached = s;
            break;
        }

        const int l = s / (int) cells;
        const int c = s % (int) cells;
        const int x = c % grid.nx;
        const int y = c / grid.nx;

        auto relax = [&]( int aNext, int aCost, int8_t aDir )
        {
            if( g[s] + aCost < g[aNext] )
            {
                g[aNext] = g[s] + aCost;
                parent[aNext] = s;
                dirIn[aNext] = aDir;
                open.emplace( g[aNext] + heuristic( aNext ), aNext );
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
            const int near = nearPad[base + grid.Index( x2, y2 )] ? ESCAPE_COST : 0;
            relax( l * (int) cells + grid.Index( x2, y2 ), ( ( d & 1 ) ? 14 : 10 ) + turn + near, (int8_t) d );
        }

        if( vias && !via[c] )
        {
            for( int l2 = 0; l2 < L; ++l2 )
            {
                if( l2 != l && !track[(size_t) l2 * cells + c] )
                    relax( l2 * (int) cells + c, viaCost, -1 );
            }
        }
    }

    if( reached < 0 )
        return finish();

    // The path, start first: a corner where the direction changes, a via where the layer does
    std::vector<int> path;

    for( int s = reached; s >= 0; s = parent[s] )
        path.push_back( s );

    std::reverse( path.begin(), path.end() );

    plan.found = true;
    plan.startLayer = aReq.layers[path.front() / (int) cells];

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

    return finish();
}
