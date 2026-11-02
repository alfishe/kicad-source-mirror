#include "pcb_escape_estimate.h"
#include "rail_plan.h"

#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <netclass.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <zone.h>
#include <lset.h>
#include <project/net_settings.h>
#include <geometry/shape.h>
#include <geometry/shape_poly_set.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <vector>


namespace
{

struct ESCAPE_GRID
{
    BOX2I                     box;
    int                       cell = 1;
    int                       cols = 0;
    int                       rows = 0;
    std::vector<PCB_LAYER_ID> layers;
    std::vector<int>          owner;   ///< per layer and cell: 0 free, -1 nobody, > 0 only that net
    int                       viaReach = 0;   ///< cells round a via's centre it takes

    int      Cells() const { return cols * rows; }
    int      Node( int aLayer, int aCol, int aRow ) const { return ( aLayer * rows + aRow ) * cols + aCol; }
    VECTOR2I Centre( int aCol, int aRow ) const
    {
        return VECTOR2I( box.GetLeft() + aCol * cell + cell / 2, box.GetTop() + aRow * cell + cell / 2 );
    }

    bool Inside( const VECTOR2I& aAt ) const { return box.Contains( aAt ); }

    void Mark( int aLayer, const BOX2I& aBox, int aNet, const std::function<bool( const VECTOR2I& )>& aHit )
    {
        const int c0 = std::max( 0, ( aBox.GetLeft() - box.GetLeft() ) / cell );
        const int c1 = std::min( cols - 1, ( aBox.GetRight() - box.GetLeft() ) / cell );
        const int r0 = std::max( 0, ( aBox.GetTop() - box.GetTop() ) / cell );
        const int r1 = std::min( rows - 1, ( aBox.GetBottom() - box.GetTop() ) / cell );

        for( int r = r0; r <= r1; ++r )
        {
            for( int c = c0; c <= c1; ++c )
            {
                if( !aHit( Centre( c, r ) ) )
                    continue;

                int& o = owner[Node( aLayer, c, r )];
                o = ( aNet <= 0 || ( o != 0 && o != aNet ) ) ? -1 : aNet;
            }
        }
    }

    bool Passable( int aNode, int aNet ) const { return owner[aNode] == 0 || owner[aNode] == aNet; }

    bool ViaFits( int aCol, int aRow, int aNet ) const
    {
        for( int l = 0; l < (int) layers.size(); ++l )
        {
            for( int r = aRow - viaReach; r <= aRow + viaReach; ++r )
            {
                for( int c = aCol - viaReach; c <= aCol + viaReach; ++c )
                {
                    if( c < 0 || r < 0 || c >= cols || r >= rows || !Passable( Node( l, c, r ), aNet ) )
                        return false;
                }
            }
        }

        return true;
    }
};


struct ESCAPE_CONNECTION
{
    int              net = 0;
    PAD*             from = nullptr;
    PAD*             to = nullptr;
    bool             toInside = true;
    std::vector<int> path;
    std::vector<int> viaNodes;   ///< the cells its vias take, on every layer
    int              vias = 0;
    double           lengthMm = 0;
    bool             found = false;
};


/// the cells of a pad on the grid's layers: its own area, else the cell under its centre
std::vector<int> padNodes( const ESCAPE_GRID& aGrid, PAD* aPad )
{
    std::vector<int> nodes;

    for( int l = 0; l < (int) aGrid.layers.size(); ++l )
    {
        if( !aPad->IsOnLayer( aGrid.layers[l] ) )
            continue;

        std::shared_ptr<SHAPE> shape = aPad->GetEffectiveShape( aGrid.layers[l] );
        const BOX2I            bb = aPad->GetBoundingBox();
        bool                   any = false;

        for( int r = std::max( 0, ( bb.GetTop() - aGrid.box.GetTop() ) / aGrid.cell );
             r <= std::min( aGrid.rows - 1, ( bb.GetBottom() - aGrid.box.GetTop() ) / aGrid.cell ); ++r )
        {
            for( int c = std::max( 0, ( bb.GetLeft() - aGrid.box.GetLeft() ) / aGrid.cell );
                 c <= std::min( aGrid.cols - 1, ( bb.GetRight() - aGrid.box.GetLeft() ) / aGrid.cell ); ++c )
            {
                if( shape->Collide( aGrid.Centre( c, r ), 0 ) )
                {
                    nodes.push_back( aGrid.Node( l, c, r ) );
                    any = true;
                }
            }
        }

        if( !any && aGrid.Inside( aPad->GetPosition() ) )
        {
            const int c = ( aPad->GetPosition().x - aGrid.box.GetLeft() ) / aGrid.cell;
            const int r = ( aPad->GetPosition().y - aGrid.box.GetTop() ) / aGrid.cell;
            nodes.push_back( aGrid.Node( l, std::clamp( c, 0, aGrid.cols - 1 ), std::clamp( r, 0, aGrid.rows - 1 ) ) );
        }
    }

    return nodes;
}

} // namespace


ESCAPE_ESTIMATE EstimateEscape( BOARD* aBoard, FOOTPRINT* aPart, const std::set<int>& aRails, double aMarginMm )
{
    const auto      started = std::chrono::steady_clock::now();
    ESCAPE_ESTIMATE out;

    if( !aBoard || !aPart )
        return out;

    BOARD_DESIGN_SETTINGS&    ds = aBoard->GetDesignSettings();
    std::shared_ptr<NETCLASS> defaults = ds.m_NetSettings ? ds.m_NetSettings->GetDefaultNetclass() : nullptr;
    const int width = std::max( defaults && defaults->HasTrackWidth() ? defaults->GetTrackWidth() : ds.m_TrackMinWidth,
                                ds.m_TrackMinWidth );
    const int clearance = std::max( defaults && defaults->HasClearance() ? defaults->GetClearance() : ds.m_MinClearance,
                                    ds.m_MinClearance );
    const int via = std::max( defaults && defaults->HasViaDiameter() ? defaults->GetViaDiameter() : ds.m_ViasMinSize,
                              ds.m_ViasMinSize );

    // the cell: one track pitch, or the part's pad pitch when that is coarser (up to two track pitches):
    // its pads then sit on cell centres, one way out in front of each
    const int pitch = std::clamp( width + clearance, pcbIUScale.mmToIU( 0.2 ), pcbIUScale.mmToIU( 1.0 ) );
    int       padPitch = std::numeric_limits<int>::max();
    const SHAPE_POLY_SET& partCy = aPart->GetCourtyard( aPart->IsFlipped() ? B_CrtYd : F_CrtYd );
    const BOX2I           partBb = partCy.OutlineCount() ? partCy.BBox() : aPart->GetBoundingBox( false );
    std::vector<PAD*>     rowPads;

    // the pads of the rows round the edge (an exposed pad's parts in the middle left out)
    for( PAD* pad : aPart->Pads() )
    {
        const VECTOR2I d = pad->GetPosition() - partBb.Centre();

        if( !pad->GetNumber().IsEmpty()
            && ( std::abs( d.x ) > 0.6 * partBb.GetWidth() / 2 || std::abs( d.y ) > 0.6 * partBb.GetHeight() / 2 ) )
        {
            rowPads.push_back( pad );
        }
    }

    for( PAD* a : rowPads )
    {
        for( PAD* b : rowPads )
        {
            const VECTOR2I d = b->GetPosition() - a->GetPosition();

            if( a != b && std::min( std::abs( d.x ), std::abs( d.y ) ) < pcbIUScale.mmToIU( 0.01 ) )
                padPitch = std::min( padPitch, std::max( std::abs( d.x ), std::abs( d.y ) ) );
        }
    }

    ESCAPE_GRID grid;
    grid.cell = padPitch >= pitch && padPitch <= 2 * pitch ? padPitch : pitch;
    grid.viaReach = (int) std::floor( double( via - width ) / 2.0 / grid.cell + 0.5 );

    // the box: the part's courtyard grown by the margin, on the cell lines through the part's centre
    const SHAPE_POLY_SET& cy = aPart->GetCourtyard( aPart->IsFlipped() ? B_CrtYd : F_CrtYd );
    BOX2I                 partBox = cy.OutlineCount() ? cy.BBox() : aPart->GetBoundingBox( false );
    const VECTOR2I        centre = partBox.Centre();
    const int             halfW = partBox.GetWidth() / 2 + pcbIUScale.mmToIU( aMarginMm );
    const int             halfH = partBox.GetHeight() / 2 + pcbIUScale.mmToIU( aMarginMm );
    const int             nx = ( halfW + grid.cell - 1 ) / grid.cell;
    const int             ny = ( halfH + grid.cell - 1 ) / grid.cell;

    // cell centres on the pads of the part's rows: a column through the pad nearest the courtyard's top
    // edge, a row through the one nearest its left edge
    VECTOR2I align = centre - VECTOR2I( grid.cell / 2, grid.cell / 2 );
    PAD*     top = nullptr;
    PAD*     left = nullptr;

    for( PAD* pad : aPart->Pads() )
    {
        if( !top || pad->GetPosition().y < top->GetPosition().y )
            top = pad;

        if( !left || pad->GetPosition().x < left->GetPosition().x )
            left = pad;
    }

    if( top && left )
        align = VECTOR2I( top->GetPosition().x, left->GetPosition().y );

    auto snap = [&]( int aFrom, int aOn )
    {
        const long long k = (long long) std::floor( double( aFrom - aOn ) / grid.cell );
        return int( aOn + k * grid.cell - grid.cell / 2 );
    };

    grid.cols = 2 * nx + 1;
    grid.rows = 2 * ny + 1;
    grid.box = BOX2I( VECTOR2I( snap( centre.x - nx * grid.cell, align.x ), snap( centre.y - ny * grid.cell, align.y ) ),
                      VECTOR2I( grid.cols * grid.cell, grid.rows * grid.cell ) );

    // routing layers: the outer ones and inner ones without a pour of a rail (planes)
    for( PCB_LAYER_ID layer : LSET::AllCuMask( aBoard->GetCopperLayerCount() ).CuStack() )
    {
        bool plane = false;

        if( layer != F_Cu && layer != B_Cu )
        {
            for( ZONE* zone : aBoard->Zones() )
            {
                if( !zone->GetIsRuleArea() && zone->IsOnLayer( layer ) && aRails.count( zone->GetNetCode() ) )
                    plane = true;
            }
        }

        if( !plane )
            grid.layers.push_back( layer );
    }

    grid.owner.assign( grid.layers.size() * grid.Cells(), 0 );
    out.cellMm = pcbIUScale.IUTomm( grid.cell );
    out.cells = grid.Cells();

    const int  reach = clearance + width / 2 - 1;
    BOX2I      wide = grid.box;
    wide.Inflate( reach );

    SHAPE_POLY_SET outline;

    if( aBoard->GetBoardPolygonOutlines( outline, true ) && outline.OutlineCount() > 0 )
    {
        for( int l = 0; l < (int) grid.layers.size(); ++l )
        {
            for( int r = 0; r < grid.rows; ++r )
            {
                for( int c = 0; c < grid.cols; ++c )
                {
                    if( !outline.Contains( grid.Centre( c, r ) ) )
                        grid.owner[grid.Node( l, c, r )] = -1;
                }
            }
        }
    }

    std::map<int, std::vector<PAD*>> netPads;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 )
                netPads[pad->GetNetCode()].push_back( pad );

            BOX2I bb = pad->GetBoundingBox();
            bb.Inflate( reach );

            if( !bb.Intersects( grid.box ) )
                continue;

            for( int l = 0; l < (int) grid.layers.size(); ++l )
            {
                if( !pad->IsOnLayer( grid.layers[l] ) )
                    continue;

                std::shared_ptr<SHAPE> shape = pad->GetEffectiveShape( grid.layers[l] );
                grid.Mark( l, bb, pad->GetNetCode(), [&]( const VECTOR2I& p ) { return shape->Collide( p, reach ); } );
            }
        }
    }

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        BOX2I bb = track->GetBoundingBox();
        bb.Inflate( reach );

        if( !bb.Intersects( grid.box ) )
            continue;

        for( int l = 0; l < (int) grid.layers.size(); ++l )
        {
            if( !track->IsOnLayer( grid.layers[l] ) )
                continue;

            std::shared_ptr<SHAPE> shape = track->GetEffectiveShape( grid.layers[l] );
            grid.Mark( l, bb, track->GetNetCode(), [&]( const VECTOR2I& p ) { return shape->Collide( p, reach ); } );
        }
    }

    // the rails' stitching vias, through every layer (a rail with a plane on another layer): the runs of
    // the anchor's rail pins as pcb_stitch plans them inward, every other rail pad one at the nearest
    // spot beside it
    std::map<int, std::set<PCB_LAYER_ID>> planeLayers;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( !zone->GetIsRuleArea() && aRails.count( zone->GetNetCode() ) )
        {
            for( PCB_LAYER_ID layer : zone->GetLayerSet().CuStack() )
                planeLayers[zone->GetNetCode()].insert( layer );
        }
    }

    auto reserveVia = [&]( const VECTOR2I& aAt, int aDia, int aNet )
    {
        BOX2I bb( aAt, VECTOR2I( 0, 0 ) );
        bb.Inflate( aDia / 2 + reach );

        for( int l = 0; l < (int) grid.layers.size(); ++l )
            grid.Mark( l, bb, aNet, [&]( const VECTOR2I& p ) { return ( p - aAt ).EuclideanNorm() <= aDia / 2 + reach; } );

        out.railVias++;
        out.railViaSpots.push_back( { pcbIUScale.IUTomm( aAt.x ), pcbIUScale.IUTomm( aAt.y ) } );
    };

    auto hasPlane = [&]( int aNet, PCB_LAYER_ID aBesides )
    {
        const auto planes = planeLayers.find( aNet );
        return planes != planeLayers.end()
               && std::any_of( planes->second.begin(), planes->second.end(), [&]( PCB_LAYER_ID l ) { return l != aBesides; } );
    };

    std::set<const PAD*> planned;

    for( PAD* pad : aPart->Pads() )
    {
        if( !aRails.count( pad->GetNetCode() ) )
            continue;

        const RAIL_PLAN_OPTIONS options = railPlanOptions( aBoard, pad->GetNet(), aPart );

        for( const RAIL_PIN_PLAN& plan : planRowRailVias( aBoard, aPart, options ) )
        {
            if( !plan.planned || planned.count( plan.pad ) || !hasPlane( plan.pad->GetNetCode(), plan.pad->GetPrincipalLayer() ) )
                continue;

            planned.insert( plan.pad );
            reserveVia( plan.spot, plan.dia, plan.pad->GetNetCode() );

            for( int l = 0; l < (int) grid.layers.size(); ++l )
            {
                if( !plan.pad->IsOnLayer( grid.layers[l] ) )
                    continue;

                BOX2I bb( plan.stub.A, VECTOR2I( 0, 0 ) );
                bb.Merge( plan.stub.B );
                bb.Inflate( reach );
                grid.Mark( l, bb, plan.pad->GetNetCode(), [&]( const VECTOR2I& p ) { return plan.stub.Distance( p ) <= reach; } );
            }
        }

        break;
    }

    BOX2I partCentres( aPart->GetPosition(), VECTOR2I( 0, 0 ) );

    for( PAD* pad : aPart->Pads() )
        partCentres.Merge( pad->GetPosition() );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            const int netCode = pad->GetNetCode();

            if( !aRails.count( netCode ) || !grid.Inside( pad->GetPosition() ) || pad->HasHole() )
                continue;

            if( fp == aPart && ( planned.count( pad ) || std::find( rowPads.begin(), rowPads.end(), pad ) == rowPads.end() ) )
                continue;

            const PCB_LAYER_ID padLayer = pad->GetPrincipalLayer();

            if( !hasPlane( netCode, padLayer ) )
                continue;

            // an anchor's pin: inward, as pcb_stitch puts it, its way out left open
            const std::optional<VECTOR2I> outward = fp == aPart ? outwardOf( pad, partCentres ) : std::nullopt;
            std::shared_ptr<SHAPE>        shape = pad->GetEffectiveShape( padLayer );
            const BOX2I                   padBox = pad->GetBoundingBox();
            const int              span = std::max( padBox.GetWidth(), padBox.GetHeight() ) / 2 + via + clearance;
            const int              c0 = ( pad->GetPosition().x - grid.box.GetLeft() ) / grid.cell;
            const int              r0 = ( pad->GetPosition().y - grid.box.GetTop() ) / grid.cell;
            const int              k = span / grid.cell + 1;
            int                    bestC = -1, bestR = -1;
            double                 best = std::numeric_limits<double>::max();

            for( int r = std::max( 0, r0 - k ); r <= std::min( grid.rows - 1, r0 + k ); ++r )
            {
                for( int c = std::max( 0, c0 - k ); c <= std::min( grid.cols - 1, c0 + k ); ++c )
                {
                    const VECTOR2I at = grid.Centre( c, r );
                    const double   d = ( at - pad->GetPosition() ).EuclideanNorm();

                    if( outward && ( (int64_t) ( at - pad->GetPosition() ).x * outward->x
                                     + (int64_t) ( at - pad->GetPosition() ).y * outward->y ) > 0 )
                    {
                        continue;
                    }

                    if( d < best && d <= span && !shape->Collide( at, via / 2 ) && grid.ViaFits( c, r, netCode ) )
                    {
                        best = d;
                        bestC = c;
                        bestR = r;
                    }
                }
            }

            if( bestC >= 0 )
                reserveVia( grid.Centre( bestC, bestR ), via, netCode );
        }
    }

    // the connections: per net with a pad in the box, the shortest tree over its pads, its edges with
    // an end in the box (an end outside: the other one is taken as the start)
    std::vector<ESCAPE_CONNECTION> conns;

    for( auto& [net, pads] : netPads )
    {
        if( aRails.count( net ) || pads.size() < 2 || pads.size() > 64 )
            continue;

        if( std::none_of( pads.begin(), pads.end(), [&]( PAD* p ) { return grid.Inside( p->GetPosition() ); } ) )
            continue;

        std::vector<bool>   in( pads.size(), false );
        std::vector<double> best( pads.size(), std::numeric_limits<double>::max() );
        std::vector<int>    parent( pads.size(), -1 );
        best[0] = 0;

        for( size_t round = 0; round < pads.size(); ++round )
        {
            int u = -1;

            for( size_t i = 0; i < pads.size(); ++i )
            {
                if( !in[i] && ( u < 0 || best[i] < best[u] ) )
                    u = (int) i;
            }

            in[u] = true;

            if( parent[u] >= 0 )
            {
                PAD* a = pads[parent[u]];
                PAD* b = pads[u];

                if( !grid.Inside( a->GetPosition() ) )
                    std::swap( a, b );

                if( grid.Inside( a->GetPosition() ) && a->GetParentFootprint() != b->GetParentFootprint() )
                    conns.push_back( { net, a, b, grid.Inside( b->GetPosition() ) } );
            }

            for( size_t i = 0; i < pads.size(); ++i )
            {
                const double d = ( pads[i]->GetPosition() - pads[u]->GetPosition() ).EuclideanNorm();

                if( !in[i] && d < best[i] )
                {
                    best[i] = d;
                    parent[i] = u;
                }
            }
        }
    }

    out.connections = (int) conns.size();

    // negotiated congestion: every connection routed again each round round the others' cells, the
    // price of a shared cell growing
    const int                      nodes = (int) grid.owner.size();
    std::vector<std::vector<int>>  users( nodes );
    std::vector<float>             history( nodes, 0.0f );
    std::vector<double>            dist( nodes );
    std::vector<int>               from( nodes );
    const double                   step = 10.0, diagonal = 14.142, viaCost = 40.0;
    const int                      dc[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    const int                      dr[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

    auto others = [&]( int aNode, int aNet )
    {
        int n = 0;

        for( int u : users[aNode] )
            n += u != aNet;

        return n;
    };

    // a via's cells on every layer
    auto viaBlock = [&]( int aCol, int aRow )
    {
        std::vector<int> block;

        for( int l = 0; l < (int) grid.layers.size(); ++l )
        {
            for( int r = std::max( 0, aRow - grid.viaReach ); r <= std::min( grid.rows - 1, aRow + grid.viaReach ); ++r )
            {
                for( int c = std::max( 0, aCol - grid.viaReach ); c <= std::min( grid.cols - 1, aCol + grid.viaReach ); ++c )
                    block.push_back( grid.Node( l, c, r ) );
            }
        }

        return block;
    };

    auto route = [&]( ESCAPE_CONNECTION& aConn, double aPresent, bool aHard )
    {
        std::fill( dist.begin(), dist.end(), std::numeric_limits<double>::max() );
        std::fill( from.begin(), from.end(), -1 );

        using ITEM = std::pair<double, int>;
        std::priority_queue<ITEM, std::vector<ITEM>, std::greater<ITEM>> open;
        std::set<int> targets;

        for( int n : padNodes( grid, aConn.from ) )
        {
            dist[n] = 0;
            open.push( { 0.0, n } );
        }

        if( aConn.toInside )
        {
            for( int n : padNodes( grid, aConn.to ) )
                targets.insert( n );
        }

        const VECTOR2I farEnd = aConn.to->GetPosition();
        double         bestTotal = std::numeric_limits<double>::max();
        int            bestNode = -1;

        while( !open.empty() )
        {
            auto [d, n] = open.top();
            open.pop();

            if( d > dist[n] || d >= bestTotal )
                continue;

            const int l = n / grid.Cells();
            const int r = ( n % grid.Cells() ) / grid.cols;
            const int c = n % grid.cols;

            if( aConn.toInside ? targets.count( n ) > 0 : ( c == 0 || r == 0 || c == grid.cols - 1 || r == grid.rows - 1 ) )
            {
                const double rest = aConn.toInside ? 0.0 : step * ( grid.Centre( c, r ) - farEnd ).EuclideanNorm() / grid.cell;

                if( d + rest < bestTotal )
                {
                    bestTotal = d + rest;
                    bestNode = n;
                }

                if( aConn.toInside )
                    break;
            }

            auto relax = [&]( int aNext, double aBase )
            {
                const double price = aBase * ( 1.0 + history[aNext] ) * ( 1.0 + aPresent * others( aNext, aConn.net ) );

                if( d + price < dist[aNext] )
                {
                    dist[aNext] = d + price;
                    from[aNext] = n;
                    open.push( { dist[aNext], aNext } );
                }
            };

            for( int k = 0; k < 8; ++k )
            {
                const int cc = c + dc[k], rr = r + dr[k];

                if( cc < 0 || rr < 0 || cc >= grid.cols || rr >= grid.rows )
                    continue;

                const int next = grid.Node( l, cc, rr );

                if( ( !grid.Passable( next, aConn.net ) || ( aHard && others( next, aConn.net ) > 0 ) ) && !targets.count( next ) )
                    continue;

                // no corner cut between two walls
                if( k >= 4 && !grid.Passable( grid.Node( l, cc, r ), aConn.net )
                    && !grid.Passable( grid.Node( l, c, rr ), aConn.net ) )
                {
                    continue;
                }

                relax( next, k < 4 ? step : diagonal );
            }

            if( grid.layers.size() > 1 && grid.ViaFits( c, r, aConn.net ) )
            {
                int crowd = 0;

                for( int b : viaBlock( c, r ) )
                    crowd = std::max( crowd, others( b, aConn.net ) );

                if( !( aHard && crowd > 0 ) )
                {
                    const double price = viaCost * ( 1.0 + history[n] ) * ( 1.0 + aPresent * crowd );

                    for( int ll = 0; ll < (int) grid.layers.size(); ++ll )
                    {
                        const int next = grid.Node( ll, c, r );

                        if( ll != l && d + price < dist[next] )
                        {
                            dist[next] = d + price;
                            from[next] = n;
                            open.push( { dist[next], next } );
                        }
                    }
                }
            }
        }

        aConn.path.clear();
        aConn.viaNodes.clear();
        aConn.vias = 0;
        aConn.lengthMm = 0;
        aConn.found = bestNode >= 0;

        if( !aConn.found )
            return;

        for( int n = bestNode; n >= 0; n = from[n] )
            aConn.path.push_back( n );

        for( size_t k = 1; k < aConn.path.size(); ++k )
        {
            const int a = aConn.path[k - 1], b = aConn.path[k];

            if( a / grid.Cells() != b / grid.Cells() )
            {
                aConn.vias++;

                for( int v : viaBlock( a % grid.cols, ( a % grid.Cells() ) / grid.cols ) )
                {
                    if( std::find( aConn.path.begin(), aConn.path.end(), v ) == aConn.path.end() )
                        aConn.viaNodes.push_back( v );
                }

                continue;
            }

            const int ca = a % grid.cols, cb = b % grid.cols;
            const int ra = ( a % grid.Cells() ) / grid.cols, rb = ( b % grid.Cells() ) / grid.cols;
            aConn.lengthMm += out.cellMm * ( ( ca != cb && ra != rb ) ? 1.4142 : 1.0 );
        }

        if( !aConn.toInside )
        {
            const int n = bestNode;
            aConn.lengthMm += pcbIUScale.IUTomm( ( grid.Centre( n % grid.cols, ( n % grid.Cells() ) / grid.cols ) - farEnd ).EuclideanNorm() );
        }
    };

    auto take = [&]( ESCAPE_CONNECTION& aConn, int aSign )
    {
        std::vector<int> cells = aConn.path;
        cells.insert( cells.end(), aConn.viaNodes.begin(), aConn.viaNodes.end() );
        std::sort( cells.begin(), cells.end() );
        cells.erase( std::unique( cells.begin(), cells.end() ), cells.end() );

        for( int n : cells )
        {
            if( aSign > 0 )
            {
                users[n].push_back( aConn.net );
            }
            else if( auto it = std::find( users[n].begin(), users[n].end(), aConn.net ); it != users[n].end() )
            {
                users[n].erase( it );
            }
        }
    };

    // shortest first, as pcb_route takes them
    std::sort( conns.begin(), conns.end(), []( const ESCAPE_CONNECTION& a, const ESCAPE_CONNECTION& b )
               { return ( a.from->GetPosition() - a.to->GetPosition() ).EuclideanNorm()
                        < ( b.from->GetPosition() - b.to->GetPosition() ).EuclideanNorm(); } );

    for( ESCAPE_CONNECTION& conn : conns )
    {
        route( conn, 0.0, false );

        if( !conn.found )
        {
            out.boxedIn++;

            if( out.boxedPads.size() < 12 )
                out.boxedPads.push_back( std::string( conn.from->GetParentFootprint()->GetReference().utf8_str() ) + "."
                                         + std::string( conn.from->GetNumber().utf8_str() ) );
        }
    }

    // one pass as a router without rip-up takes them: each round the cells the ones before it took
    for( ESCAPE_CONNECTION& conn : conns )
    {
        if( !conn.found )
            continue;

        route( conn, 0.0, true );

        if( conn.found )
        {
            take( conn, +1 );
            continue;
        }

        out.firstPassFailed++;
        route( conn, 0.0, false );
        take( conn, +1 );
    }

    double present = 0.5;
    int    overflowing = 0;

    for( int round = 0; round < 12; ++round )
    {
        out.rounds = round + 1;

        for( ESCAPE_CONNECTION& conn : conns )
        {
            if( !conn.found )
                continue;

            take( conn, -1 );
            route( conn, present, false );
            take( conn, +1 );
        }

        overflowing = 0;

        for( ESCAPE_CONNECTION& conn : conns )
        {
            auto crowded = [&]( int n ) { return others( n, conn.net ) > 0; };

            if( conn.found
                && ( std::any_of( conn.path.begin(), conn.path.end(), crowded )
                     || std::any_of( conn.viaNodes.begin(), conn.viaNodes.end(), crowded ) ) )
                overflowing++;
        }

        for( int n = 0; n < nodes; ++n )
        {
            std::set<int> distinct( users[n].begin(), users[n].end() );

            if( distinct.size() > 1 )
                history[n] += 0.5f * float( distinct.size() - 1 );
        }

        if( overflowing == 0 )
            break;

        present *= 1.8;
    }

    out.overflowing = overflowing;

    for( const ESCAPE_CONNECTION& conn : conns )
    {
        out.vias += conn.vias;
        out.lengthMm += conn.lengthMm;
    }

    out.lengthMm = std::round( out.lengthMm * 10 ) / 10;
    out.ms = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();
    return out;
}
