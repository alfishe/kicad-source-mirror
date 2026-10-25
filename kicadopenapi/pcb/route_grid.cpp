#include "route_grid.h"
#include "route_raster.h"
#include "route_timeline.h"
#include "kopenapi_pcb.h"

#include <kicadopenapi_memory.h>

#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <footprint.h>
#include <geometry/shape_segment.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <zone.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <climits>
#include <cstdio>
#include <future>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <deque>
#include <queue>
#include <set>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace
{

using CLOCK = std::chrono::steady_clock;

double msSince( CLOCK::time_point aFrom )
{
    return std::chrono::duration<double, std::milli>( CLOCK::now() - aFrom ).count();
}

constexpr int DX[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
constexpr int DY[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
constexpr int NO_DIR = 8;

// a cell fought over in a round costs this much more for good
constexpr float HIST_STEP = 1.0f;

// rounds with no fewer nets crowded than before them: negotiation is over
constexpr size_t STALL_ROUNDS = 12;
constexpr int    STALL_AFTER = 12;   ///< before that sharing is still cheap: nobody had to move yet

// the guide: tiles of about this size, open to a class of track when this share of their cells is
// free; nets reaching more than GUIDE_SPAN tiles are searched toward the tiles' distance
constexpr double TILE_MM = 1.0;
constexpr float  GUIDE_OPEN = 0.05f;
constexpr int    GUIDE_SPAN = 4;

// a search toward more ends than this finds the nearest through buckets of cells
constexpr size_t END_BUCKETS_FROM = 256;

// the guides one variant keeps (recomputed when dropped: the same values)
constexpr size_t GUIDE_CACHE_BYTES = 128u << 20;

// an end whose free cells run out within this many states is in a pocket of fixed copper
constexpr size_t POCKET_STATES = 20000;

/// @brief Sizes shared by nets: one obstacle map each
struct SIZES
{
    int halfWidth = 0;
    int clearance = 0;
    int viaRadius = 0;
    int viaDrill = 0;

    auto Key() const { return std::make_tuple( halfWidth, clearance, viaRadius, viaDrill ); }
};

/// @brief Cells within a radius of a centre, as a half width per row
struct DISK
{
    int              r = 0;
    std::vector<int> half;   ///< per dy + r: the farthest dx inside, -1 none

    /// aSlack (cells squared) widens the test: a diagonal step between two cells just outside
    /// passes as close as sqrt( d * d - 0.5 ) to the centre
    explicit DISK( double aRadiusCells, double aSlack = 0.0 )
    {
        r = std::max( 0, (int) std::ceil( std::sqrt( aRadiusCells * aRadiusCells + aSlack ) ) );
        const double rr = aRadiusCells * aRadiusCells + aSlack;
        half.assign( 2 * r + 1, -1 );

        for( int dy = -r; dy <= r; ++dy )
        {
            for( int dx = r; dx >= 0; --dx )
            {
                if( (double) dx * dx + (double) dy * dy < rr )
                {
                    half[dy + r] = dx;
                    break;
                }
            }
        }
    }
};

/// @brief A shape a map was made of: copper of an owner (-1 none may pass) and how far its cells reach
struct MARK
{
    std::shared_ptr<const SHAPE> shape;
    int                          inflate;
    int32_t                      owner;
};

/// @brief The board as the search sees it: read once, shared by every variant, never written
struct WORLD
{
    PLAN_GRID grid;
    int       L = 0;
    size_t    cells = 0;
    int       h2h = 0;
    int       holeClearance = 0;

    std::vector<SIZES>                sizes;
    std::vector<std::vector<int32_t>> track;   ///< per sizes: L * cells; 0 free, net: only that net, -1 none
    std::vector<std::vector<int32_t>> via;     ///< per sizes: cells
    std::vector<uint8_t>              fine;    ///< cells near a fine-pitch package
    std::vector<uint8_t>              rows;    ///< cells between the two pin rows of a connector
    std::vector<uint32_t>             keep;    ///< cells around chips (a bit each) where only their own nets' vias go
    std::vector<uint16_t>             away;    ///< cells: distance in cells to the nearest chip pad, up to viaAway; empty none
    std::vector<int32_t>              endOf;   ///< states a net's track there stays inside its end copper: that net

    /// @brief Deep inside its own end's copper a path crowds nobody: the end's copper is there anyway
    bool InOwnEnd( uint32_t aState, int aNet ) const { return endOf[aState] == aNet; }

    /// @brief While the world is built: the shapes of each map (sizes x plane: the track layers, then
    /// the vias), their reach over the exact distance (slack) and the board's outline
    std::vector<std::vector<MARK>> marks;
    int                            planes = 0;
    int                            slack = 0;
    SHAPE_POLY_SET                 outline;

    using EXIT = GRID_EXIT;

    struct NET
    {
        int                                code = 0;
        int                                sizes = 0;
        bool                               vias = false;
        std::vector<std::vector<uint32_t>> ends;      ///< states of each end
        std::vector<std::array<int, 4>>    endBox;    ///< cells: x0 y0 x1 y1
        double                             span = 0;  ///< half perimeter of the ends' box, cells
        bool                               finePitch = false;
        std::vector<bool>                  walled;    ///< per end: fixed copper parts it from the other ends
        std::vector<GRID_WALL>             walls;     ///< the walled ends: room and what closes it
        uint32_t                           chips = 0; ///< the chips (keep bits) it has a pad on
        std::vector<std::pair<int, int>>   holes;     ///< cells of its through-hole ends (on every layer already)
        int                                group = -1;
        int                                rank = 0;
        std::vector<EXIT>                  exits;
    };

    std::vector<NET> nets;

    // the coarse grid for the guide: tiles of tile x tile cells
    int                               tile = 0;
    int                               tnx = 0, tny = 0;
    std::vector<std::vector<float>>   tileFree;   ///< per sizes: L * tiles: share of cells any net's track may use
    std::vector<std::vector<uint8_t>> tileVia;    ///< per sizes: tiles: a via fits somewhere inside

    int TileOf( int aCell ) const { return ( aCell / grid.nx / tile ) * tnx + aCell % grid.nx / tile; }

    bool TrackOk( int aSizes, uint32_t aState, int aNet ) const
    {
        const int32_t id = track[aSizes][aState];
        return id == 0 || id == aNet;
    }

    bool ViaOk( int aSizes, uint32_t aCell, int aNet ) const
    {
        const int32_t id = via[aSizes][aCell];
        return id == 0 || id == aNet;
    }
};


/// @brief Each end's free cells flooded a little way: an end that runs out of room before it meets
/// all the other ends is walled in by fixed copper, and no search needs to find that out again
void findPockets( WORLD& aWorld )
{
    const PLAN_GRID& g = aWorld.grid;
    const size_t     cells = aWorld.cells;

    auto check = [&]( WORLD::NET& aNet )
    {
        const int k = aNet.sizes;
        aNet.walled.assign( aNet.ends.size(), false );
        aNet.walls.clear();

        auto open = [&]( uint32_t s ) { return aWorld.TrackOk( k, s, aNet.code ) || aWorld.InOwnEnd( s, aNet.code ); };

        for( size_t e = 0; e < aNet.ends.size(); ++e )
        {
            if( aNet.ends[e].size() > POCKET_STATES )
                continue;

            std::unordered_set<uint32_t> seen( aNet.ends[e].begin(), aNet.ends[e].end() );
            std::vector<uint32_t>        todo( aNet.ends[e].begin(), aNet.ends[e].end() );
            // (owner, plane, octant round the end) -> nearest closed state, count: overlapping halos
            // of several nets leave no owner (-1), the octants still part what closes it
            std::map<std::tuple<int32_t, bool, int>, std::pair<uint32_t, int>> closedBy;
            std::unordered_set<uint64_t>                                       closedCells;

            VECTOR2I centre( 0, 0 );

            for( uint32_t s : aNet.ends[e] )
                centre += g.Centre( (int) ( s % cells ) ) / (int) aNet.ends[e].size();

            auto closed = [&]( uint32_t n, bool aVia )
            {
                const int32_t  owner = aVia ? aWorld.via[k][n % cells] : aWorld.track[k][n];
                const VECTOR2I d = g.Centre( (int) ( n % cells ) ) - centre;
                const int      octant = (int) std::floor( ( std::atan2( (double) d.y, (double) d.x ) + M_PI ) / ( M_PI / 4 ) ) % 8;
                auto [it, fresh] = closedBy.try_emplace( { owner, aVia, octant }, n | ( aVia ? 0x80000000u : 0 ), 0 );

                if( closedCells.size() < 1024 )
                    closedCells.insert( ( (uint64_t) aVia << 32 ) | ( aVia ? n % cells : n ) );
                it->second.second++;

                if( !fresh )
                {
                    const uint32_t was = it->second.first & 0x7fffffffu;

                    if( ( g.Centre( (int) ( n % cells ) ) - centre ).SquaredEuclideanNorm()
                        < ( g.Centre( (int) ( was % cells ) ) - centre ).SquaredEuclideanNorm() )
                    {
                        it->second.first = n | ( aVia ? 0x80000000u : 0 );
                    }
                }
            };

            while( !todo.empty() && seen.size() <= POCKET_STATES )
            {
                const uint32_t s = todo.back();
                todo.pop_back();
                const int l = (int) ( s / cells ), i = (int) ( s % cells );
                const int x = i % g.nx, y = i / g.nx;

                auto visit = [&]( uint32_t n )
                {
                    if( !open( n ) )
                        closed( n, false );
                    else if( seen.insert( n ).second )
                        todo.push_back( n );
                };

                for( int d = 0; d < 8; ++d )
                {
                    const int nx = x + DX[d], ny = y + DY[d];

                    if( nx < 0 || ny < 0 || nx >= g.nx || ny >= g.ny )
                        continue;

                    if( ( d & 1 ) && !open( (uint32_t) ( (size_t) l * cells + g.Index( nx, y ) ) )
                        && !open( (uint32_t) ( (size_t) l * cells + g.Index( x, ny ) ) ) )
                    {
                        continue;
                    }

                    visit( (uint32_t) ( (size_t) l * cells + g.Index( nx, ny ) ) );
                }

                if( aNet.vias && aWorld.L > 1 )
                {
                    if( aWorld.ViaOk( k, (uint32_t) i, aNet.code ) )
                    {
                        for( int to = 0; to < aWorld.L; ++to )
                            visit( (uint32_t) ( (size_t) to * cells + i ) );
                    }
                    else
                    {
                        closed( (uint32_t) i, true );
                    }
                }
            }

            if( seen.size() > POCKET_STATES )
                continue;

            for( size_t o = 0; o < aNet.ends.size() && !aNet.walled[e]; ++o )
            {
                aNet.walled[e] = o != e && !aNet.ends[o].empty()
                                 && std::none_of( aNet.ends[o].begin(), aNet.ends[o].end(),
                                                  [&]( uint32_t s ) { return seen.count( s ) > 0; } );
            }

            if( !aNet.walled[e] )
                continue;

            GRID_WALL wall;
            wall.end = (int) e;
            wall.room = seen.size();
            std::set<int> layers;

            for( uint32_t s : aNet.ends[e] )
                layers.insert( (int) ( s / cells ) );

            wall.layers = (int) layers.size();

            for( const auto& [key, nearby] : closedBy )
            {
                const int32_t  owner = std::get<0>( key );
                const bool     via = nearby.first & 0x80000000u;
                const uint32_t n = nearby.first & 0x7fffffffu;
                wall.by.push_back( { owner, g.Centre( (int) ( n % cells ) ), via ? -1 : (int) ( n / cells ), nearby.second } );
            }

            for( uint64_t c : closedCells )
            {
                const bool     via = c >> 32;
                const uint32_t n = (uint32_t) c;
                wall.closed.emplace_back( g.Centre( (int) ( n % cells ) ), via ? -1 : (int) ( n / cells ) );
            }

            // the nearest first
            std::sort( wall.by.begin(), wall.by.end(),
                       [&]( const GRID_WALL::BY& a, const GRID_WALL::BY& b )
                       { return ( a.at - centre ).SquaredEuclideanNorm() < ( b.at - centre ).SquaredEuclideanNorm(); } );
            aNet.walls.push_back( std::move( wall ) );
        }
    };

    const size_t threads = std::max( 1u, std::thread::hardware_concurrency() );
    std::vector<std::future<void>> jobs;

    for( size_t t = 0; t < threads; ++t )
    {
        jobs.push_back( std::async( std::launch::async,
                                    [&, t]
                                    {
                                        for( size_t n = t; n < aWorld.nets.size(); n += threads )
                                            check( aWorld.nets[n] );
                                    } ) );
    }

    for( auto& job : jobs )
        job.get();
}


inline void own( int32_t& aCell, int32_t aOwner );


/// @brief A second lattice half a cell off the first, for fine-pitch pads the first walls in: a
/// pad whose way out runs between the cell centres (its part off the grid's phase) leaves along
/// it.  The copper from inside the pad to a free cell of the grid is fixed before the search (an
/// exit) and the end starts on that cell; other nets keep their clearance from it.  Only walled ends
/// at fine-pitch parts are tried, each in a window round its pad, so the cost stays local; the
/// distances there are exact (the shapes the maps were made of, without their slack)
std::pair<int, int> halfCellExits( WORLD& aWorld, const std::vector<GRID_NET>& aNets, const GRID_PARAMS& aParams )
{
    int tried = 0, won = 0;

    const PLAN_GRID& g = aWorld.grid;
    const size_t     cells = aWorld.cells;
    const int        half = g.cell / 2;
    const bool       outlined = aWorld.outline.OutlineCount() > 0;

    for( size_t n = 0; n < aWorld.nets.size() && n < aNets.size(); ++n )
    {
        WORLD::NET& net = aWorld.nets[n];
        const int   k = net.sizes;
        const int   hw = aWorld.sizes[k].halfWidth;

        for( size_t e = 0; e < net.ends.size() && e < aNets[n].ends.size(); ++e )
        {
            PAD* pad = dynamic_cast<PAD*>( aNets[n].ends[e] );
            int  cx = 0, cy = 0;

            if( !net.walled[e] || !pad || !g.CellOf( pad->GetPosition(), cx, cy ) || !aWorld.fine[g.Index( cx, cy )] )
                continue;

            tried++;

            const VECTOR2I size = pad->GetBoundingBox().GetSize();
            const int      R = std::min( 40, (int) std::ceil( ( std::max( size.x, size.y ) / 2.0 + pcbIUScale.mmToIU( 1.0 ) ) / g.cell ) );
            const int      W = 2 * R + 1;
            const BOX2I    window( g.Centre( cx - R, cy - R ) - VECTOR2I( g.cell, g.cell ),
                                   VECTOR2I( ( W + 1 ) * g.cell, ( W + 1 ) * g.cell ) );

            std::set<uint32_t> others;

            for( size_t o = 0; o < net.ends.size(); ++o )
            {
                if( o != e )
                    others.insert( net.ends[o].begin(), net.ends[o].end() );
            }

            // a cell of the grid with room: it meets another end, or the room is too big to be a pocket
            std::map<uint32_t, bool> roomy;

            auto hasRoom = [&]( uint32_t aStart )
            {
                auto [it, fresh] = roomy.try_emplace( aStart, false );

                if( !fresh )
                    return it->second;

                auto open = [&]( uint32_t s ) { return aWorld.TrackOk( k, s, net.code ) || aWorld.InOwnEnd( s, net.code ); };
                std::unordered_set<uint32_t> seen = { aStart };
                std::vector<uint32_t>        todo = { aStart };

                while( !todo.empty() && !it->second )
                {
                    const uint32_t s = todo.back();
                    todo.pop_back();

                    if( others.count( s ) || seen.size() > POCKET_STATES )
                    {
                        it->second = true;
                        break;
                    }

                    const int l = (int) ( s / cells ), i = (int) ( s % cells );
                    const int x = i % g.nx, y = i / g.nx;

                    for( int d = 0; d < 8; ++d )
                    {
                        const int nx = x + DX[d], ny = y + DY[d];

                        if( nx < 0 || ny < 0 || nx >= g.nx || ny >= g.ny )
                            continue;

                        if( ( d & 1 ) && !open( (uint32_t) ( (size_t) l * cells + g.Index( nx, y ) ) )
                            && !open( (uint32_t) ( (size_t) l * cells + g.Index( x, ny ) ) ) )
                        {
                            continue;
                        }

                        const uint32_t to = (uint32_t) ( (size_t) l * cells + g.Index( nx, ny ) );

                        if( open( to ) && seen.insert( to ).second )
                            todo.push_back( to );
                    }

                    if( net.vias && aWorld.L > 1 && aWorld.ViaOk( k, (uint32_t) i, net.code ) )
                    {
                        for( int to = 0; to < aWorld.L; ++to )
                        {
                            const uint32_t t = (uint32_t) ( (size_t) to * cells + i );

                            if( open( t ) && seen.insert( t ).second )
                                todo.push_back( t );
                        }
                    }
                }

                return it->second;
            };

            std::optional<WORLD::EXIT> found;

            for( int l = 0; l < aWorld.L && !found; ++l )
            {
                const PCB_LAYER_ID layer = aParams.layers[l];

                if( std::none_of( net.ends[e].begin(), net.ends[e].end(), [&]( uint32_t s ) { return (int) ( s / cells ) == l; } )
                    || !pad->FlashLayer( layer ) )
                {
                    continue;
                }

                const std::shared_ptr<SHAPE> copper = pad->GetEffectiveShape( layer );
                std::vector<const MARK*>     nearby;

                for( const MARK& m : aWorld.marks[(size_t) k * aWorld.planes + l] )
                {
                    BOX2I bb = m.shape->BBox();
                    bb.Inflate( m.inflate );

                    if( m.owner != net.code && bb.Intersects( window ) )
                        nearby.push_back( &m );
                }

                auto freeAt = [&]( const VECTOR2I& p )
                {
                    if( outlined && !aWorld.outline.Contains( p ) )
                        return false;

                    return std::none_of( nearby.begin(), nearby.end(),
                                         [&]( const MARK* m ) { return m->shape->Collide( p, m->inflate - aWorld.slack ); } );
                };

                auto freeSeg = [&]( const VECTOR2I& a, const VECTOR2I& b )
                {
                    if( outlined && ( !aWorld.outline.Contains( a ) || !aWorld.outline.Contains( b ) ) )
                        return false;

                    const SEG sg( a, b );
                    return std::none_of( nearby.begin(), nearby.end(),
                                         [&]( const MARK* m ) { return m->shape->Collide( sg, m->inflate - aWorld.slack ); } );
                };

                for( const VECTOR2I& phase : { VECTOR2I( half, half ), VECTOR2I( half, 0 ), VECTOR2I( 0, half ) } )
                {
                    if( found )
                        break;

                    auto at = [&]( int aCell ) { return g.Centre( cx - R + aCell % W, cy - R + aCell / W ) + phase; };

                    std::vector<int> parent( (size_t) W * W, -2 );   // -2 not seen, -1 a start inside the pad
                    std::deque<int>  todo;

                    for( int c = 0; c < W * W; ++c )
                    {
                        if( copper && copper->Collide( at( c ), 0 ) && freeAt( at( c ) ) )
                        {
                            parent[c] = -1;
                            todo.push_back( c );
                        }
                    }

                    while( !todo.empty() && !found )
                    {
                        const int      c = todo.front();
                        const VECTOR2I p = at( c );
                        const int      bx = cx - R + c % W, by = cy - R + c / W;
                        todo.pop_front();

                        // onto the grid: the cells half a cell away
                        for( int hy = 0; hy <= ( phase.y ? 1 : 0 ) && !found; ++hy )
                        {
                            for( int hx = 0; hx <= ( phase.x ? 1 : 0 ) && !found; ++hx )
                            {
                                const int x = bx + hx, y = by + hy;

                                if( x < 0 || y < 0 || x >= g.nx || y >= g.ny )
                                    continue;

                                const uint32_t state = (uint32_t) ( (size_t) l * cells + g.Index( x, y ) );

                                if( !aWorld.TrackOk( k, state, net.code ) || !freeSeg( p, g.Centre( x, y ) ) || !hasRoom( state ) )
                                    continue;

                                WORLD::EXIT exit;
                                exit.end = (int) e;
                                exit.state = state;
                                exit.track.layer = layer;
                                exit.track.points.push_back( g.Centre( x, y ) );

                                for( int q = c; q >= 0; q = parent[q] )
                                    exit.track.points.push_back( at( q ) );

                                std::reverse( exit.track.points.begin(), exit.track.points.end() );

                                // corners only where the direction changes
                                std::vector<VECTOR2I> corners;

                                for( const VECTOR2I& pt : exit.track.points )
                                {
                                    if( corners.size() >= 2 )
                                    {
                                        const VECTOR2I d1 = corners.back() - corners[corners.size() - 2], d2 = pt - corners.back();

                                        if( (int64_t) d1.x * d2.y == (int64_t) d1.y * d2.x && (int64_t) d1.x * d2.x + (int64_t) d1.y * d2.y > 0 )
                                        {
                                            corners.back() = pt;
                                            continue;
                                        }
                                    }

                                    corners.push_back( pt );
                                }

                                exit.track.points = std::move( corners );
                                found = std::move( exit );
                            }
                        }

                        for( int d = 0; d < 8 && !found; ++d )
                        {
                            const int nx = c % W + DX[d], ny = c / W + DY[d];

                            if( nx < 0 || ny < 0 || nx >= W || ny >= W || parent[ny * W + nx] != -2 )
                                continue;

                            if( freeSeg( p, at( ny * W + nx ) ) )
                            {
                                parent[ny * W + nx] = c;
                                todo.push_back( ny * W + nx );
                            }
                        }
                    }
                }
            }

            if( !found )
                continue;

            const int i = (int) ( found->state % cells );
            net.ends[e] = { found->state };
            net.endBox[e] = { i % g.nx, i / g.nx, i % g.nx, i / g.nx };
            net.walled[e] = false;
            net.walls.erase( std::remove_if( net.walls.begin(), net.walls.end(), [&]( const GRID_WALL& w ) { return w.end == (int) e; } ),
                             net.walls.end() );

            // the exit's copper: the other nets keep their clearance from it
            const int    l = (int) ( found->state / cells );
            const size_t offset = (size_t) l * cells;

            for( size_t z = 0; z < aWorld.sizes.size(); ++z )
            {
                const SIZES& sz = aWorld.sizes[z];
                const int    clearance = std::max( sz.clearance, aWorld.sizes[k].clearance );

                for( size_t q = 1; q < found->track.points.size(); ++q )
                {
                    const SHAPE_SEGMENT seg( found->track.points[q - 1], found->track.points[q], 2 * hw );
                    std::vector<int32_t>& map = aWorld.track[z];
                    g.Each( seg, clearance + sz.halfWidth + aWorld.slack, offset, [&]( size_t c ) { return map[c] < 0; },
                            [&]( size_t c ) { own( map[c], net.code ); } );

                    if( sz.viaRadius > 0 )
                    {
                        std::vector<int32_t>& vias = aWorld.via[z];
                        g.Each( seg, clearance + sz.viaRadius + aWorld.slack, 0, [&]( size_t c ) { return vias[c] < 0; },
                                [&]( size_t c ) { own( vias[c], net.code ); } );
                    }
                }
            }

            net.exits.push_back( std::move( *found ) );
            won++;
        }
    }

    return { tried, won };
}


/// @brief The coarse grid: how open each tile is to each class of track, where a via fits
void buildTiles( WORLD& aWorld )
{
    const PLAN_GRID& g = aWorld.grid;
    aWorld.tile = std::max( 4, (int) std::lround( pcbIUScale.mmToIU( TILE_MM ) / g.cell ) );
    aWorld.tnx = ( g.nx + aWorld.tile - 1 ) / aWorld.tile;
    aWorld.tny = ( g.ny + aWorld.tile - 1 ) / aWorld.tile;

    const size_t tiles = (size_t) aWorld.tnx * aWorld.tny;
    const size_t K = aWorld.sizes.size();
    aWorld.tileFree.assign( K, std::vector<float>( tiles * aWorld.L, 0.0f ) );
    aWorld.tileVia.assign( K, std::vector<uint8_t>( tiles, 0 ) );

    std::vector<std::future<void>> jobs;

    for( size_t k = 0; k < K; ++k )
    {
        jobs.push_back( std::async( std::launch::async,
                                    [&aWorld, k, tiles]
                                    {
                                        std::vector<int> count( tiles * aWorld.L, 0 ), area( tiles, 0 );

                                        for( int i = 0; i < (int) aWorld.cells; ++i )
                                        {
                                            const int t = aWorld.TileOf( i );
                                            area[t]++;

                                            for( int l = 0; l < aWorld.L; ++l )
                                                count[l * tiles + t] += aWorld.track[k][(size_t) l * aWorld.cells + i] == 0;

                                            if( aWorld.via[k][i] >= 0 )
                                                aWorld.tileVia[k][t] = 1;
                                        }

                                        for( size_t t = 0; t < tiles * aWorld.L; ++t )
                                            aWorld.tileFree[k][t] = area[t % tiles] ? (float) count[t] / area[t % tiles] : 0.0f;
                                    } ) );
    }

    for( auto& job : jobs )
        job.get();
}


/// @brief Put an owner on a cell: a second owner closes it to every net
inline void own( int32_t& aCell, int32_t aOwner )
{
    if( aCell == 0 )
        aCell = aOwner;
    else if( aCell != aOwner )
        aCell = -1;
}


/// @brief A chip by its reference: not a passive, a connector, a module or a test point
bool isPackage( const FOOTPRINT* aFp )
{
    static const std::set<std::string> others = { "R", "RN", "RA", "RP", "C", "CN", "L", "FB", "D", "LED", "Y", "X",
                                                  "J", "P", "A", "SW", "TP", "F", "BT", "H", "MH", "JP" };
    std::string prefix;

    for( wxUniChar c : aFp->GetReference() )
    {
        if( !wxIsalpha( c ) )
            break;

        prefix += (char) wxToupper( c );
    }

    return !prefix.empty() && !others.count( prefix );
}


/// @brief Pads in rows on the outline and four or more inside it (a ball or land array)
bool isArray( const std::vector<BOX2I>& aPads )
{
    BOX2I               box = aPads[0];
    std::vector<double> areas;

    for( const BOX2I& b : aPads )
    {
        box.Merge( b );
        areas.push_back( (double) b.GetArea() );
    }

    std::nth_element( areas.begin(), areas.begin() + areas.size() / 2, areas.end() );
    const double median = areas[areas.size() / 2];
    int          inner = 0;

    for( const BOX2I& b : aPads )
    {
        const int edge = std::min( b.GetWidth(), b.GetHeight() );

        if( (double) b.GetArea() <= 4 * median && b.GetLeft() - box.GetLeft() > edge && box.GetRight() - b.GetRight() > edge
            && b.GetTop() - box.GetTop() > edge && box.GetBottom() - b.GetBottom() > edge )
        {
            inner++;
        }
    }

    return inner >= 4;
}


void buildWorld( BOARD* aBoard, const std::vector<GRID_NET>& aNets, const GRID_PARAMS& aParams, WORLD& aWorld )
{
    const BOARD_DESIGN_SETTINGS& ds = aBoard->GetDesignSettings();
    std::shared_ptr<DRC_ENGINE>  drc = ds.m_DRCEngine;
    const int                    L = (int) aParams.layers.size();
    const LSET                   copper = LSET::AllCuMask( aBoard->GetCopperLayerCount() );

    aWorld.L = L;
    aWorld.h2h = std::max( ds.m_HoleToHoleMin, 0 );
    aWorld.holeClearance = std::max( ds.m_HoleClearance, 0 );

    std::map<std::tuple<int, int, int, int>, int> sizesOf;
    std::vector<int>                              sample;   // a net of each sizes for the board's rules

    for( const GRID_NET& n : aNets )
    {
        SIZES s{ n.width / 2, std::max( n.clearance, ds.m_MinClearance ), n.viaDiameter / 2, n.viaDrill };

        if( !sizesOf.count( s.Key() ) )
        {
            sizesOf[s.Key()] = (int) aWorld.sizes.size();
            aWorld.sizes.push_back( s );
            sample.push_back( n.netCode );
        }
    }

    // a cell of a third of the finest pitch of track and clearance, on a 0.025 mm step
    const int step = pcbIUScale.mmToIU( 0.025 );
    int       cell = aParams.cell;

    if( cell <= 0 )
    {
        int finest = INT_MAX;

        for( const SIZES& s : aWorld.sizes )
            finest = std::min( finest, 2 * s.halfWidth + s.clearance );

        cell = std::clamp( finest / 3 / step * step, step, 4 * step );
    }

    BOX2I region = aBoard->GetBoardEdgesBoundingBox();

    if( region.GetWidth() <= 0 || region.GetHeight() <= 0 )
        region = aBoard->ComputeBoundingBox( false );

    while( (double) region.GetWidth() / cell * region.GetHeight() / cell * L > 3e7 )
        cell += step;

    // the grid's phase: on the most pad centres, so a track can end on a pad's centre line
    std::map<int, int> phaseX, phaseY;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            phaseX[( ( pad->GetPosition().x - region.GetX() ) % cell + cell ) % cell]++;
            phaseY[( ( pad->GetPosition().y - region.GetY() ) % cell + cell ) % cell]++;
        }
    }

    auto mode = []( const std::map<int, int>& aCount )
    {
        auto best = std::max_element( aCount.begin(), aCount.end(), []( const auto& a, const auto& b ) { return a.second < b.second; } );
        return best == aCount.end() ? 0 : best->first;
    };

    region.SetOrigin( region.GetOrigin() + VECTOR2I( mode( phaseX ), mode( phaseY ) ) - VECTOR2I( cell, cell ) );
    region.SetSize( region.GetSize() + VECTOR2I( cell, cell ) );

    PLAN_GRID& grid = aWorld.grid;
    grid.origin = region.GetOrigin();
    grid.cell = cell;
    grid.nx = region.GetWidth() / cell + 1;
    grid.ny = region.GetHeight() / cell + 1;
    aWorld.cells = (size_t) grid.nx * grid.ny;

    const size_t cells = aWorld.cells;
    // a step between two free centres comes at most ~0.06 cell closer to a corner than its ends
    const int    slack = cell / 8;

    aWorld.track.assign( aWorld.sizes.size(), std::vector<int32_t>( cells * L, 0 ) );
    aWorld.via.assign( aWorld.sizes.size(), std::vector<int32_t>( cells, 0 ) );

    // The board is read here, on the calling thread: each shape with its reach per map; the cells
    // are filled afterwards, one map plane (sizes x layer, or the via plane) per worker
    const int                      planes = L + 1;   // the track layers, then the vias
    std::vector<std::vector<MARK>> marks( aWorld.sizes.size() * planes );
    size_t                         k = 0;

    auto mark = [&]( std::shared_ptr<const SHAPE> aShape, int aInflate, int aPlane, int32_t aOwner )
    {
        marks[k * planes + aPlane].push_back( { std::move( aShape ), aInflate, aOwner } );
    };

    for( k = 0; k < aWorld.sizes.size(); ++k )
    {
        const SIZES& sz = aWorld.sizes[k];
        const bool   vias = sz.viaRadius > 0;

        PCB_TRACK probeTrack( aBoard );
        probeTrack.SetFlags( ROUTER_TRANSIENT );
        probeTrack.SetNetCode( sample[k] );
        probeTrack.SetWidth( 2 * sz.halfWidth );

        PCB_VIA probeVia( aBoard );
        probeVia.SetFlags( ROUTER_TRANSIENT );
        probeVia.SetNetCode( sample[k] );
        probeVia.SetViaType( VIATYPE::THROUGH );
        probeVia.SetLayerPair( F_Cu, B_Cu );
        probeVia.SetWidth( PADSTACK::ALL_LAYERS, std::max( 2 * sz.viaRadius, 1 ) );
        probeVia.SetDrill( std::max( sz.viaDrill, 1 ) );

        // the probe stands at the item: rules conditioned on a place (inside a courtyard) apply
        // where the item is; a custom rule may go below the class clearance, not below the board's
        auto rule = [&]( DRC_CONSTRAINT_T aType, BOARD_ITEM* aProbe, BOARD_ITEM* aItem, PCB_LAYER_ID aLayer, int aFloor )
        {
            if( drc )
            {
                const VECTOR2I at = aItem->GetPosition();

                if( aProbe == &probeTrack )
                {
                    probeTrack.SetStart( at );
                    probeTrack.SetEnd( at );
                }
                else if( aProbe == &probeVia )
                {
                    probeVia.SetPosition( at );
                }

                DRC_CONSTRAINT c = drc->EvalRules( aType, aProbe, aItem, aLayer );

                if( c.GetValue().HasMin() )
                {
                    const bool custom = c.GetParentRule() && !c.GetParentRule()->IsImplicit();

                    if( custom && aType == CLEARANCE_CONSTRAINT )
                        return std::max( c.GetValue().Min(), std::max( ds.m_MinClearance, 0 ) );

                    return std::max( c.GetValue().Min(), aFloor );
                }
            }

            return aFloor;
        };

        auto floorOf = [&]( BOARD_ITEM* aItem )
        {
            int floor = sz.clearance;

            if( BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( aItem ) )
            {
                if( NETCLASS* nc = bci->GetEffectiveNetClass(); nc && nc->HasClearance() )
                    floor = std::max( floor, nc->GetClearance() );
            }

            return floor;
        };

        // copper: its net may pass, no other
        auto copperItem = [&]( BOARD_ITEM* aItem, int32_t aOwner, bool aViaHard,
                               const std::function<std::shared_ptr<SHAPE>( PCB_LAYER_ID )>& aShape )
        {
            const int floor = floorOf( aItem );

            for( int l = 0; l < L; ++l )
            {
                const PCB_LAYER_ID layer = aParams.layers[l];

                if( !aItem->IsOnLayer( layer ) )
                    continue;

                if( std::shared_ptr<SHAPE> s = aShape( layer ) )
                {
                    probeTrack.SetLayer( layer );
                    mark( s, rule( CLEARANCE_CONSTRAINT, &probeTrack, aItem, layer, floor ) + sz.halfWidth + slack, l,
                          aOwner );
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
                    const int reach = std::max( rule( CLEARANCE_CONSTRAINT, &probeVia, aItem, layer, floor ) + sz.viaRadius,
                                                rule( HOLE_CLEARANCE_CONSTRAINT, &probeVia, aItem, layer, ds.m_HoleClearance )
                                                        + sz.viaDrill / 2 );
                    mark( s, reach + slack, L, aViaHard ? -1 : aOwner );
                }
            }
        };

        // another net's copper keeps the hole clearance from a hole, plated or not
        auto hole = [&]( BOARD_ITEM* aItem, std::shared_ptr<const SHAPE> aHole, int32_t aOwner )
        {
            for( int l = 0; l < L; ++l )
            {
                probeTrack.SetLayer( aParams.layers[l] );
                mark( aHole, rule( HOLE_CLEARANCE_CONSTRAINT, &probeTrack, aItem, aParams.layers[l], ds.m_HoleClearance )
                                     + sz.halfWidth + slack,
                      l, aOwner );
            }

            if( vias )
            {
                mark( aHole, rule( HOLE_TO_HOLE_CONSTRAINT, &probeVia, aItem, F_Cu, aWorld.h2h ) + sz.viaDrill / 2 + slack,
                      L, -1 );
                mark( aHole, rule( HOLE_CLEARANCE_CONSTRAINT, &probeVia, aItem, F_Cu, ds.m_HoleClearance ) + sz.viaRadius + slack,
                      L, aOwner );
            }
        };

        auto ownerOf = []( BOARD_CONNECTED_ITEM* aItem ) { return aItem->GetNetCode() > 0 ? aItem->GetNetCode() : -1; };

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
            {
                // no via in a pad, whoever's
                copperItem( pad, ownerOf( pad ), true,
                            [&]( PCB_LAYER_ID aLayer ) -> std::shared_ptr<SHAPE>
                            {
                                if( !pad->FlashLayer( aLayer ) )
                                    return nullptr;

                                return pad->GetEffectiveShape( aLayer );
                            } );

                if( pad->HasHole() )
                    hole( pad, pad->GetEffectiveHoleShape(), pad->GetAttribute() == PAD_ATTRIB::NPTH ? -1 : ownerOf( pad ) );
            }

            for( BOARD_ITEM* item : fp->GraphicalItems() )
            {
                if( item->Type() == PCB_SHAPE_T && ( item->GetLayerSet() & copper ).any() )
                {
                    const int32_t o = ownerOf( static_cast<PCB_SHAPE*>( item ) );
                    copperItem( item, o, false, [&]( PCB_LAYER_ID aLayer ) { return item->GetEffectiveShape( aLayer ); } );
                }
            }
        }

        for( PCB_TRACK* t : aBoard->Tracks() )
        {
            copperItem( t, ownerOf( t ), false,
                        [&]( PCB_LAYER_ID aLayer ) -> std::shared_ptr<SHAPE>
                        {
                            if( t->Type() == PCB_VIA_T && !static_cast<PCB_VIA*>( t )->FlashLayer( aLayer ) )
                                return nullptr;

                            return t->GetEffectiveShape( aLayer );
                        } );

            if( t->Type() == PCB_VIA_T )
            {
                std::shared_ptr<SHAPE_SEGMENT> h = static_cast<PCB_VIA*>( t )->GetEffectiveHoleShape();

                for( int l = 0; l < L; ++l )
                {
                    probeTrack.SetLayer( aParams.layers[l] );
                    mark( h, rule( HOLE_CLEARANCE_CONSTRAINT, &probeTrack, t, aParams.layers[l], ds.m_HoleClearance )
                                     + sz.halfWidth + slack,
                          l, ownerOf( t ) );
                }

                if( vias )
                {
                    mark( h, aWorld.h2h + sz.viaDrill / 2 + slack, L, -1 );
                    mark( h, rule( HOLE_CLEARANCE_CONSTRAINT, &probeVia, t, F_Cu, ds.m_HoleClearance ) + sz.viaRadius + slack, L,
                          ownerOf( t ) );
                }
            }
        }

        for( BOARD_ITEM* item : aBoard->Drawings() )
        {
            if( item->Type() != PCB_SHAPE_T )
                continue;

            if( item->IsOnLayer( Edge_Cuts ) )
            {
                const std::shared_ptr<SHAPE> s = item->GetEffectiveShape( Edge_Cuts );
                const int                    edge = std::max( ds.m_CopperEdgeClearance, 0 );

                for( int l = 0; l < L; ++l )
                    mark( s, edge + sz.halfWidth + slack, l, -1 );

                if( vias )
                    mark( s, edge + sz.viaRadius + slack, L, -1 );
            }
            else if( ( item->GetLayerSet() & copper ).any() )
            {
                const int32_t o = ownerOf( static_cast<PCB_SHAPE*>( item ) );
                copperItem( item, o, false, [&]( PCB_LAYER_ID aLayer ) { return item->GetEffectiveShape( aLayer ); } );
            }
        }

        std::vector<ZONE*> zones( aBoard->Zones().begin(), aBoard->Zones().end() );

        for( FOOTPRINT* fp : aBoard->Footprints() )
            zones.insert( zones.end(), fp->Zones().begin(), fp->Zones().end() );

        for( ZONE* zone : zones )
        {
            if( zone->GetIsRuleArea() )
            {
                // a footprint's zone keeps its outline in the footprint's own coordinates
                auto area = std::make_shared<const SHAPE_POLY_SET>( zone->GetBoardOutline() );

                for( int l = 0; l < L; ++l )
                {
                    if( zone->GetDoNotAllowTracks() && zone->IsOnLayer( aParams.layers[l] ) )
                        mark( area, sz.halfWidth + slack, l, -1 );
                }

                if( vias && zone->GetDoNotAllowVias() && ( zone->GetLayerSet() & copper ).any() )
                    mark( area, sz.viaRadius + slack, L, -1 );

                continue;
            }

            if( aParams.fillsOut && !zone->IsTeardropArea() )
                continue;

            copperItem( zone, ownerOf( zone ), false,
                        [&]( PCB_LAYER_ID aLayer ) -> std::shared_ptr<SHAPE>
                        {
                            std::shared_ptr<SHAPE_POLY_SET> fill = zone->GetFilledPolysList( aLayer );

                            if( !fill || fill->IsEmpty() )
                                return nullptr;

                            return std::make_shared<SHAPE_POLY_SET>( *fill );
                        } );
        }
    }

    {
        std::vector<std::future<void>> fill;

        for( size_t p = 0; p < marks.size(); ++p )
        {
            fill.push_back( std::async( std::launch::async,
                                        [&, p]()
                                        {
                                            const size_t          sizes = p / planes;
                                            const int             plane = (int) ( p % planes );
                                            std::vector<int32_t>& map = plane < L ? aWorld.track[sizes] : aWorld.via[sizes];
                                            const size_t          offset = plane < L ? (size_t) plane * cells : 0;

                                            for( const MARK& m : marks[p] )
                                            {
                                                grid.Each( *m.shape, m.inflate, offset, [&]( size_t i ) { return map[i] < 0; },
                                                           [&]( size_t i ) { own( map[i], m.owner ); } );
                                            }
                                        } ) );
        }

        for( std::future<void>& f : fill )
            f.get();
    }

    aWorld.marks = std::move( marks );
    aWorld.planes = planes;
    aWorld.slack = slack;

    // outside the outline
    SHAPE_POLY_SET& outline = aWorld.outline;

    if( !aBoard->GetBoardPolygonOutlines( outline, true ) )
        outline.RemoveAllContours();

    if( outline.OutlineCount() > 0 )
    {
        std::vector<uint8_t> inside( cells, 0 );
        grid.Mark( outline, 0, inside );

        for( size_t z = 0; z < aWorld.sizes.size(); ++z )
        {
            for( size_t i = 0; i < cells; ++i )
            {
                if( inside[i] )
                    continue;

                for( int l = 0; l < L; ++l )
                    aWorld.track[z][(size_t) l * cells + i] = -1;

                aWorld.via[z][i] = -1;
            }
        }
    }

    // around a chip (pads on all four sides, or an array) only its own nets' vias while the board
    // routes: the way out of its pads stays open; an array keeps the vias between its balls
    aWorld.keep.assign( cells, 0 );
    std::map<FOOTPRINT*, uint32_t> chipBit;

    auto markChip = [&]( FOOTPRINT* aFp, const BOX2I& aBox, double aPitch )
    {
        if( aParams.viaKeepOut <= 0 || aPitch > pcbIUScale.mmToIU( 1.0 ) )
            return;

        std::vector<double> areas;

        for( PAD* pad : aFp->Pads() )
            areas.push_back( (double) pad->GetBoundingBox().GetArea() );

        std::nth_element( areas.begin(), areas.begin() + areas.size() / 2, areas.end() );
        const double median = areas[areas.size() / 2];
        const int    edge = (int) ( aPitch / 2 );
        int          sides[4] = { 0, 0, 0, 0 };
        int          inner = 0;

        for( PAD* pad : aFp->Pads() )
        {
            const BOX2I pb = pad->GetBoundingBox();

            if( (double) pb.GetArea() > 4 * median )
                continue;

            const bool l = pb.GetLeft() - aBox.GetLeft() <= edge, r = aBox.GetRight() - pb.GetRight() <= edge;
            const bool t = pb.GetTop() - aBox.GetTop() <= edge, b = aBox.GetBottom() - pb.GetBottom() <= edge;
            sides[0] += l, sides[1] += r, sides[2] += t, sides[3] += b;
            inner += !( l || r || t || b );
        }

        const bool array = inner >= 4;

        if( !array && std::count_if( sides, sides + 4, []( int n ) { return n >= 2; } ) < 4 )
            return;

        if( chipBit.size() >= 32 )
            return;

        const uint32_t bit = 1u << chipBit.size();
        chipBit[aFp] = bit;

        BOX2I out = aBox;
        out.Inflate( aParams.viaKeepOut );
        int x0, y0, x1, y1, ix0, iy0, ix1, iy1;
        grid.CellOf( out.GetOrigin(), x0, y0 );
        grid.CellOf( out.GetEnd(), x1, y1 );
        grid.CellOf( aBox.GetOrigin(), ix0, iy0 );
        grid.CellOf( aBox.GetEnd(), ix1, iy1 );

        for( int y = std::max( 0, y0 ); y <= std::min( grid.ny - 1, y1 ); ++y )
        {
            for( int x = std::max( 0, x0 ); x <= std::min( grid.nx - 1, x1 ); ++x )
            {
                if( !array || x <= ix0 || x >= ix1 || y <= iy0 || y >= iy1 )
                    aWorld.keep[grid.Index( x, y )] |= bit;
            }
        }
    };

    // around fine-pitch packages a via takes the room of the package's ways out
    aWorld.fine.assign( cells, 0 );
    const int fineAround = pcbIUScale.mmToIU( 0.8 );
    std::map<FOOTPRINT*, bool> finePart;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( fp->Pads().size() < 8 )
            continue;

        double pitch = std::numeric_limits<double>::max();
        BOX2I  box;
        bool   first = true;

        for( PAD* a : fp->Pads() )
        {
            box = first ? a->GetBoundingBox() : box.Merge( a->GetBoundingBox() );
            first = false;

            for( PAD* b : fp->Pads() )
            {
                if( KopenapiPadNeighbours( a, b ) )
                    pitch = std::min( pitch, (double) ( a->GetPosition() - b->GetPosition() ).EuclideanNorm() );
            }
        }

        markChip( fp, box, pitch );

        if( pitch > pcbIUScale.mmToIU( 0.65 ) )
            continue;

        finePart[fp] = true;
        box.Inflate( fineAround );
        int x0, y0, x1, y1;
        grid.CellOf( box.GetOrigin(), x0, y0 );
        grid.CellOf( box.GetEnd(), x1, y1 );

        for( int y = std::max( 0, y0 ); y <= std::min( grid.ny - 1, y1 ); ++y )
        {
            for( int x = std::max( 0, x0 ); x <= std::min( grid.nx - 1, x1 ); ++x )
                aWorld.fine[grid.Index( x, y )] = 1;
        }
    }

    // between the two pin rows of a connector (a 2-row header): a way out for its pins, not a channel
    if( aParams.rowsCost > 1.0f )
    {
        aWorld.rows.assign( cells, 0 );

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( isPackage( fp ) )
                continue;

            std::vector<VECTOR2I> at;

            for( PAD* pad : fp->Pads() )
            {
                if( pad->HasHole() && pad->GetAttribute() == PAD_ATTRIB::PTH )
                    at.push_back( pad->GetPosition() );
            }

            if( at.size() < 6 )
                continue;

            for( int axis = 0; axis < 2; ++axis )
            {
                const int                 tol = pcbIUScale.mmToIU( 0.05 );
                std::map<int, int>        lines;   // row coordinate across the rows -> pins
                int                       lo = INT_MAX, hi = INT_MIN;

                for( const VECTOR2I& p : at )
                {
                    const int across = axis == 0 ? p.x : p.y;
                    const int along = axis == 0 ? p.y : p.x;
                    auto      it = std::find_if( lines.begin(), lines.end(), [&]( const auto& l ) { return std::abs( l.first - across ) <= tol; } );

                    if( it == lines.end() )
                        lines[across] = 1;
                    else
                        it->second++;

                    lo = std::min( lo, along );
                    hi = std::max( hi, along );
                }

                if( lines.size() != 2 || lines.begin()->second < 3 || lines.rbegin()->second < 3 )
                    continue;

                const int a = lines.begin()->first, b = lines.rbegin()->first;

                for( int y = 0; y < grid.ny; ++y )
                {
                    for( int x = 0; x < grid.nx; ++x )
                    {
                        const VECTOR2I c = grid.Centre( x, y );
                        const int      across = axis == 0 ? c.x : c.y;
                        const int      along = axis == 0 ? c.y : c.x;

                        if( across > a && across < b && along >= lo && along <= hi )
                            aWorld.rows[grid.Index( x, y )] = 1;
                    }
                }
            }
        }
    }

    // around packages (not arrays, whose vias sit between the balls): how far each cell is from
    // the nearest pad, for the price of a via there
    if( aParams.viaAway > 0 && L > 1 )
    {
        aWorld.away.assign( cells, UINT16_MAX );

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( fp->Pads().size() < 8 || !isPackage( fp ) )
                continue;

            std::vector<BOX2I> pads;

            for( PAD* pad : fp->Pads() )
            {
                if( pad->GetAttribute() != PAD_ATTRIB::NPTH )
                    pads.push_back( pad->GetBoundingBox() );
            }

            if( pads.empty() || isArray( pads ) )
                continue;

            BOX2I box = pads[0];

            for( const BOX2I& b : pads )
                box.Merge( b );

            box.Inflate( aParams.viaAway );
            int x0, y0, x1, y1;
            grid.CellOf( box.GetOrigin(), x0, y0 );
            grid.CellOf( box.GetEnd(), x1, y1 );

            for( int y = std::max( 0, y0 ); y <= std::min( grid.ny - 1, y1 ); ++y )
            {
                for( int x = std::max( 0, x0 ); x <= std::min( grid.nx - 1, x1 ); ++x )
                {
                    const VECTOR2I c = grid.Centre( grid.Index( x, y ) );
                    double         d = std::numeric_limits<double>::max();

                    for( const BOX2I& b : pads )
                    {
                        const double dx = std::max( { (double) b.GetLeft() - c.x, 0.0, (double) c.x - b.GetRight() } );
                        const double dy = std::max( { (double) b.GetTop() - c.y, 0.0, (double) c.y - b.GetBottom() } );
                        d = std::min( d, std::hypot( dx, dy ) );
                    }

                    uint16_t& at = aWorld.away[grid.Index( x, y )];
                    at = std::min<uint16_t>( at, (uint16_t) std::min( d / cell, 65534.0 ) );
                }
            }
        }
    }

    // the nets' ends: the cells inside each end's copper on the routing layers
    aWorld.endOf.assign( cells * L, 0 );

    for( const GRID_NET& n : aNets )
    {
        WORLD::NET net;
        net.code = n.netCode;
        net.sizes = sizesOf[SIZES{ n.width / 2, std::max( n.clearance, ds.m_MinClearance ), n.viaDiameter / 2, n.viaDrill }.Key()];
        net.vias = n.viaDiameter > 0 && L > 1;
        net.group = n.group;
        net.rank = n.rank;

        int                   bx0 = INT_MAX, by0 = INT_MAX, bx1 = INT_MIN, by1 = INT_MIN;
        const int             hw = n.width / 2;
        std::vector<uint32_t> deep;   // where the net's track adds no copper to its ends'

        for( BOARD_CONNECTED_ITEM* end : n.ends )
        {
            std::vector<uint32_t> states;
            std::array<int, 4>    box = { INT_MAX, INT_MAX, INT_MIN, INT_MIN };

            if( PAD* pad = dynamic_cast<PAD*>( end ); pad && pad->GetParentFootprint()
                                                      && finePart.count( pad->GetParentFootprint() ) )
            {
                net.finePitch = true;
            }

            if( PAD* pad = dynamic_cast<PAD*>( end ); pad && chipBit.count( pad->GetParentFootprint() ) )
                net.chips |= chipBit[pad->GetParentFootprint()];

            if( PAD* pad = dynamic_cast<PAD*>( end ); ( pad && pad->HasHole() && pad->GetAttribute() == PAD_ATTRIB::PTH )
                                                      || end->Type() == PCB_VIA_T )
            {
                int hx = 0, hy = 0;

                if( grid.CellOf( end->GetPosition(), hx, hy ) )
                    net.holes.emplace_back( hx, hy );
            }

            for( int l = 0; l < L; ++l )
            {
                const PCB_LAYER_ID layer = aParams.layers[l];

                if( !end->IsOnLayer( layer ) )
                    continue;

                if( end->Type() == PCB_PAD_T && !static_cast<PAD*>( end )->FlashLayer( layer ) )
                    continue;

                const size_t offset = (size_t) l * cells;
                size_t       before = states.size();
                const size_t deepBefore = deep.size();

                // a pour: its whole outline, as its fill is redone around the routes
                if( end->Type() == PCB_ZONE_T )
                {
                    grid.EachInPoly( static_cast<ZONE*>( end )->GetBoardOutline(), 0, offset, []( size_t ) { return false; },
                                     [&]( size_t i )
                                     {
                                         if( aWorld.TrackOk( net.sizes, (uint32_t) i, n.netCode ) )
                                             states.push_back( (uint32_t) i );
                                     } );
                    continue;
                }

                std::shared_ptr<SHAPE> s = end->GetEffectiveShape( layer );

                if( !s )
                    continue;

                grid.Each( *s, 0, offset, []( size_t ) { return false; },
                           [&]( size_t i )
                           {
                               if( !aWorld.TrackOk( net.sizes, (uint32_t) i, n.netCode ) )
                                   return;

                               states.push_back( (uint32_t) i );

                               // a track centred here stays inside the end's copper all round
                               const VECTOR2I c = grid.Centre( (int) ( i - offset ) );
                               bool           inside = true;

                               for( int k = 0; k < 12 && inside; ++k )
                               {
                                   const double a = k * M_PI / 6;
                                   inside = s->Collide( c + VECTOR2I( KiROUND( hw * std::cos( a ) ), KiROUND( hw * std::sin( a ) ) ), 0 );
                               }

                               if( inside )
                                   deep.push_back( (uint32_t) i );
                           } );

                // no free cell inside (a small pad off the cell centres): the cell at its centre
                if( states.size() == before )
                {
                    int cx, cy;

                    if( grid.CellOf( end->GetPosition(), cx, cy ) )
                        states.push_back( (uint32_t) ( offset + grid.Index( cx, cy ) ) );
                }

                // none deep enough (a pad hardly wider than the track): its centre cell
                if( deep.size() == deepBefore && states.size() > before )
                {
                    int cx, cy;

                    if( grid.CellOf( end->GetPosition(), cx, cy )
                        && std::count( states.begin() + before, states.end(), offset + grid.Index( cx, cy ) ) )
                    {
                        deep.push_back( (uint32_t) ( offset + grid.Index( cx, cy ) ) );
                    }
                }
            }

            for( uint32_t s : states )
            {
                const int i = (int) ( s % cells );
                box[0] = std::min( box[0], i % grid.nx );
                box[1] = std::min( box[1], i / grid.nx );
                box[2] = std::max( box[2], i % grid.nx );
                box[3] = std::max( box[3], i / grid.nx );
            }

            if( !states.empty() )
            {
                bx0 = std::min( bx0, box[0] );
                by0 = std::min( by0, box[1] );
                bx1 = std::max( bx1, box[2] );
                by1 = std::max( by1, box[3] );
            }

            net.ends.push_back( std::move( states ) );
            net.endBox.push_back( box );
        }

        net.span = bx0 <= bx1 ? (double) ( bx1 - bx0 ) + ( by1 - by0 ) : 0;
        for( uint32_t s : deep )
            aWorld.endOf[s] = n.netCode;

        aWorld.nets.push_back( std::move( net ) );
    }
}


/// @brief How a variant routes: the order of the nets and a direction per layer
struct VARIANT
{
    std::string      name;
    std::vector<int> order;
    std::vector<int> direction;   ///< per layer: 0 any, 1 horizontal, 2 vertical
    GRID_STRATEGY    strategy;
};


/// @brief One variant's routing: its own congestion and search state over the shared world
class NEGOTIATION
{
public:
    NEGOTIATION( const WORLD& aWorld, const VARIANT& aVariant, const GRID_PARAMS& aParams, std::atomic<bool>& aStop ) :
            m_w( aWorld ), m_v( aVariant ), m_params( aParams ), m_stop( aStop )
    {
        const size_t S = (size_t) m_w.L * m_w.cells;
        const size_t K = m_w.sizes.size();

        m_occTrack.assign( K, std::vector<uint16_t>( S, 0 ) );
        m_occVia.assign( K, std::vector<uint16_t>( m_w.cells, 0 ) );
        m_hist.assign( S, 0.0f );
        m_histVia.assign( m_w.cells, 0.0f );
        m_node.assign( S, NODE() );
        m_dir.assign( S, NO_DIR );
        m_targetEnd.assign( S, -1 );
        m_nets.resize( m_w.nets.size() );

        for( const WORLD::NET& n : m_w.nets )
        {
            m_dead.emplace_back( n.ends.size(), false );
            m_keepDead.emplace_back( n.ends.size(), false );
        }

        m_expandedBy.assign( m_w.nets.size(), 0 );

        const double c = m_w.grid.cell;

        // the reach of one net's copper into another's: centres closer than this crowd each other
        // (exactly at it is legal: parallel runs at the minimum pitch)
        m_trackDisk.resize( K );
        m_viaDisk.resize( K );

        for( size_t a = 0; a < K; ++a )
        {
            for( size_t b = 0; b < K; ++b )
            {
                const SIZES& x = m_w.sizes[a];
                const SIZES& y = m_w.sizes[b];
                const int    clr = std::max( x.clearance, y.clearance );
                const double viaVia = std::max( { x.viaRadius + y.viaRadius + clr, ( x.viaDrill + y.viaDrill ) / 2 + m_w.h2h,
                                                  x.viaRadius + y.viaDrill / 2 + m_w.holeClearance,
                                                  y.viaRadius + x.viaDrill / 2 + m_w.holeClearance } );
                // a via's reach to another net's track: its copper's clearance or its hole's
                const double xVia = std::max( x.viaRadius + clr, x.viaDrill / 2 + m_w.holeClearance );
                const double yVia = std::max( y.viaRadius + clr, y.viaDrill / 2 + m_w.holeClearance );

                // a's track / via onto b's track map and b's via map
                // a via against a track: room for the track's diagonal steps past it (track against
                // track stays exact, parallel runs at the minimum pitch)
                m_trackDisk[a].push_back( { DISK( ( x.halfWidth + clr + y.halfWidth ) / c ),
                                            DISK( ( xVia + y.halfWidth ) / c, 0.5 ) } );
                m_viaDisk[a].push_back( { DISK( ( x.halfWidth + yVia ) / c, 0.5 ), DISK( viaVia / c ) } );
            }
        }

        m_viaCost = 10.0f * (float) ( pcbIUScale.mmToIU( 2.0 ) / c );
        m_fineCost = 10.0f * (float) ( pcbIUScale.mmToIU( 3.0 ) / c );
        m_awayCells = (float) ( m_params.viaAway / c );
        m_awayCost = 10.0f * (float) ( m_params.viaAwayCost / c );
        m_nearHoleCells = (float) ( m_params.viaNearHole / c );
        m_nearHoleCost = 10.0f * (float) ( m_params.viaNearHoleCost / c );

        if( m_params.hug < 1.0f && std::any_of( m_w.nets.begin(), m_w.nets.end(), []( const WORLD::NET& n ) { return n.group >= 0; } ) )
        {
            m_band.assign( S, 0 );
            m_bandGroup.assign( S, 0 );

            for( size_t a = 0; a < K; ++a )
            {
                const SIZES& x = m_w.sizes[a];
                m_bandDisk.emplace_back( ( 2.0 * x.halfWidth + x.clearance ) / c + std::max( 2.0, pcbIUScale.mmToIU( 0.1 ) / c ) );
            }
        }
    }

    void Run()
    {
        if( !m_warm )
            m_pf = m_v.strategy.pfStart;

        float&       pf = m_pf;
        const size_t base = m_byRound.size();

        for( m_round = m_firstRound; m_round < m_firstRound + m_params.maxRounds && !stopped(); ++m_round )
        {
            // the crowded nets, and the nets through places fought over before: a net that crowds
            // nobody may still have to make room
            std::vector<int> todo;

            if( m_round == m_firstRound && !m_firstTodo.empty() )
            {
                todo = m_firstTodo;
            }
            else
            {
                for( int n : m_v.order )
                {
                    if( ( m_round == 1 && !m_warm ) || conflicts( n ) > 0
                        || ( m_v.strategy.recent > 0 ? throughRecentHistory( n ) : throughHistory( n ) ) )
                        todo.push_back( n );
                }
            }

            int taken = 0;

            for( int n : todo )
            {
                if( stopped() )
                    return;

                at( STAGE_ROUNDS, taken++, (int) todo.size() );
                unmark( n );
                route( n, pf, false );
                markNet( n );
            }

            // places fought over cost more from now on
            int          left = 0;
            const size_t S = (size_t) m_w.L * m_w.cells;

            for( size_t n = 0; n < m_nets.size(); ++n )
            {
                const int k = m_w.nets[n].sizes;
                bool      any = false;

                for( const auto& path : m_nets[n].paths )
                {
                    for( uint32_t s : path )
                    {
                        if( m_occTrack[k][s] >= 2 && !m_w.InOwnEnd( s, m_w.nets[n].code ) )
                        {
                            m_hist[s] += histStep( s );
                            any = true;

                            if( m_lane )
                                m_histTaken.push_back( s );
                        }
                    }
                }

                for( uint32_t i : m_nets[n].vias )
                {
                    if( m_occVia[k][i] >= 2 )
                    {
                        m_histVia[i] += histStep( S + i );
                        any = true;

                        if( m_lane )
                            m_histTaken.push_back( i | GRID_LANE::VIA );
                    }
                }

                left += any;
            }

            if( m_v.strategy.histDecay < 1.0f )
                decayHistory();

            m_conflicting = left;
            m_byRound.push_back( left );
            recordRound();
            pf = std::min( pf * growth(), 1e4f );

            if( left == 0 )
                break;

            // no fewer for a while: the rest is left to the legalisation
            const size_t r = m_byRound.size();

            if( m_round - m_firstRound + 1 > STALL_AFTER && r - base > STALL_ROUNDS
                && *std::min_element( m_byRound.end() - STALL_ROUNDS, m_byRound.end() )
                           >= *std::min_element( m_byRound.begin() + (long) base, m_byRound.end() - STALL_ROUNDS ) )
            {
                break;
            }
        }

        m_byStage.push_back( m_expanded );

        for( size_t n = 0; n < m_nets.size(); ++n )
        {
            if( conflicts( (int) n ) > 0 )
                m_crowded.push_back( describeCrowding( (int) n ) );
        }

        // still crowded: the worst net routed again where nobody is, until no two nets crowd each other
        int legalised = 0;

        for( std::vector<bool> redone( m_nets.size(), false ); !stopped(); )
        {
            at( STAGE_LEGALISE, legalised++, (int) m_nets.size() );

            int worst = -1, most = 0;

            for( size_t n = 0; n < m_nets.size(); ++n )
            {
                const int c = conflicts( (int) n );

                if( c > most || ( c == most && c > 0 && redone[worst] && !redone[n] ) )
                    most = c, worst = (int) n;
            }

            if( worst < 0 )
                break;

            if( redone[worst] )
            {
                // routed apart once and still crowded (an end inside another's reach): it goes
                unmark( worst );
                m_nets[worst] = NET_STATE();
                m_nets[worst].reached.assign( m_w.nets[worst].ends.size(), false );
                m_dropped++;
                record( worst, (int) std::size( GRID_TIMELINE::STAGES ) - 1 );
                continue;
            }

            redone[worst] = true;
            unmark( worst );
            route( worst, 0, true );
            markNet( worst );
        }

        m_conflicting = 0;
        m_byStage.push_back( m_expanded );

        // the nets left short: once more where nobody is, now that the rest has settled, with vias
        // allowed around the chips
        m_keepOut = false;
        int taken = 0;

        for( int n : m_v.order )
        {
            if( stopped() )
                break;

            at( STAGE_SHORT_AGAIN, taken++, (int) m_v.order.size() );
            NET_STATE& net = m_nets[n];

            if( std::find( net.reached.begin(), net.reached.end(), false ) == net.reached.end() )
                continue;

            NET_STATE  old = net;
            const long oldReached = std::count( old.reached.begin(), old.reached.end(), true );

            unmark( n );
            route( n, 0, true );

            if( std::count( net.reached.begin(), net.reached.end(), true ) < oldReached )
            {
                net.paths = std::move( old.paths );
                net.vias = std::move( old.vias );
                net.reached = std::move( old.reached );
            }

            markNet( n );
        }

        m_byStage.push_back( m_expanded );

        // Fewer vias: each net with vias routed again among the others with vias dear; kept when it
        // joins as much with fewer vias and no more than 8 mm longer per via saved.  Vias move near
        // the chips only once every net is joined
        const double perVia = pcbIUScale.mmToIU( 8.0 ) / m_w.grid.cell;

        m_keepOut = std::any_of( m_nets.begin(), m_nets.end(),
                                 []( const NET_STATE& aNet )
                                 { return std::find( aNet.reached.begin(), aNet.reached.end(), false ) != aNet.reached.end(); } );

        taken = 0;

        for( int n : m_v.order )
        {
            if( stopped() )
                break;

            at( STAGE_FEWER_VIAS, taken++, (int) m_v.order.size() );

            if( m_nets[n].vias.empty() )
                continue;

            NET_STATE    old = m_nets[n];
            const double oldLength = length( old );
            const long   oldReached = std::count( old.reached.begin(), old.reached.end(), true );

            unmark( n );
            m_viaScale = 6.0f;
            m_oneWindow = true;
            route( n, 0, true );
            m_oneWindow = false;
            m_viaScale = 1.0f;

            const NET_STATE& now = m_nets[n];
            const size_t     saved = old.vias.size() > now.vias.size() ? old.vias.size() - now.vias.size() : 0;

            if( saved == 0 || std::count( now.reached.begin(), now.reached.end(), true ) < oldReached
                || length( now ) - oldLength > perVia * saved )
            {
                m_nets[n].paths = std::move( old.paths );
                m_nets[n].vias = std::move( old.vias );
                m_nets[n].reached = std::move( old.reached );
            }

            markNet( n );
        }

        m_byStage.push_back( m_expanded );
        taken = 0;

        for( int n : m_v.order )
        {
            if( stopped() )
                break;

            at( STAGE_PULL_TIGHT, taken++, (int) m_v.order.size() );
            unmark( n );

            if( replaying() )
            {
                takeRecorded( n );
            }
            else
            {
                // where the net's paths branch off one another: kept on the copper
                std::unordered_set<uint32_t> branches;

                for( const auto& path : m_nets[n].paths )
                {
                    if( !path.empty() )
                        branches.insert( path.front() );
                }

                for( auto& path : m_nets[n].paths )
                    pullTight( n, path, branches );
            }

            markNet( n );
        }

        for( size_t k = m_byStage.size() - 1; k > 0; --k )
            m_byStage[k] -= m_byStage[k - 1];

        at( STAGE_DONE, (int) m_v.order.size(), (int) m_v.order.size() );
    }

    /// @brief Keep the steps in aLane (before Run); a branch's lane already holds its parent's
    void Record( const GRID_TIMELINE& aTimeline, GRID_LANE& aLane, CLOCK::time_point aStarted, bool aFresh = true )
    {
        m_timeline = &aTimeline;
        m_lane = &aLane;
        m_started = aStarted;

        if( aFresh )
            aLane.Start( m_nets.size() );
    }

    /// @brief A branch (before Run): up to aStep of aFrom the run takes each net's recorded copper
    /// instead of searching, so it reaches that step's state exactly, then goes on searching
    void Replay( const GRID_LANE& aFrom, size_t aStep )
    {
        m_replay = &aFrom;
        m_replayEnd = aStep + 1;
        m_replayAt = 0;
        m_replayDead = 0;
        m_replayStarted = CLOCK::now();
    }

    /// @brief A warm branch (before Run): the state after aStep of aFrom taken as it is over this
    /// world (each net's copper, the history of the finished rounds, the present factor); the nets
    /// whose copper the world no longer allows, or that are not joined, lose it and go first in a new
    /// round
    void WarmStart( const GRID_LANE& aFrom, size_t aStep, GRID_PARAMS::WARM_HISTORY aHistory,
                    const std::vector<bool>& aEdited )
    {
        const auto                            started = CLOCK::now();
        std::lock_guard<std::recursive_mutex> lock( aFrom.Mutex() );
        const GRID_STEP&                      at = aFrom.steps[aStep];
        const size_t rounds = std::min( at.stage == 0 ? (size_t) at.round - 1 : aFrom.histByRound.size(), aFrom.histByRound.size() );
        const size_t S = (size_t) m_w.L * m_w.cells;

        // nets crowded after each finished round: what the steps of the next one saw
        std::vector<int> byRound;

        for( size_t k = 0; k <= aStep && byRound.size() < rounds; ++k )
        {
            const GRID_STEP& st = aFrom.steps[k];
            const size_t     done = st.stage == 0 ? (size_t) st.round - 1 : rounds;

            while( byRound.size() < std::min( done, rounds ) )
                byRound.push_back( st.crowded );
        }

        byRound.resize( rounds, at.crowded );
        m_warm = true;
        m_pf = m_v.strategy.pfStart;

        for( size_t r = 0; r < rounds; ++r )
        {
            m_round = (int) r + 1;

            for( uint32_t v : aHistory != GRID_PARAMS::CLEAR ? aFrom.histByRound[r] : std::vector<uint32_t>() )
            {
                if( v & GRID_LANE::VIA )
                    m_histVia[v & ~GRID_LANE::VIA] += histStep( S + ( v & ~GRID_LANE::VIA ) );
                else
                    m_hist[v] += histStep( v );
            }

            if( aHistory != GRID_PARAMS::CLEAR && m_v.strategy.histDecay < 1.0f )
                decayHistory();

            m_byRound.push_back( byRound[r] );
            m_pf = std::min( m_pf * growth(), 1e4f );
        }

        if( aHistory == GRID_PARAMS::CLEAR_EDITED && m_timeline )
        {
            for( size_t c = 0; c < m_w.cells; ++c )
            {
                const auto [x, y] = tileOf( (uint32_t) c );

                if( !aEdited[(size_t) y * m_timeline->tnx + x] )
                    continue;

                for( int l = 0; l < m_w.L; ++l )
                {
                    const size_t s = (size_t) l * m_w.cells + c;
                    m_hist[s] = 0;

                    if( !m_fought.empty() )
                        m_fought[s] = 0, m_streak[s] = 0;
                }

                m_histVia[c] = 0;

                if( !m_fought.empty() )
                    m_fought[S + c] = 0, m_streak[S + c] = 0;
            }
        }

        std::vector<uint32_t> table;
        aFrom.TableAt( aStep, table );
        m_restoring = true;

        for( int n : m_v.order )
        {
            const uint32_t id = n < (int) table.size() ? table[n] : GRID_STEP::NONE;
            NET_STATE&     net = m_nets[n];
            std::string    why = "not joined";

            if( id < aFrom.paths.size() )
            {
                net.paths = aFrom.paths[id].paths;
                net.reached = aFrom.paths[id].reached;
                net.vias.clear();

                for( const auto& path : net.paths )
                {
                    for( size_t k = 1; k < path.size(); ++k )
                    {
                        if( path[k] % m_w.cells == path[k - 1] % m_w.cells )
                            net.vias.push_back( (uint32_t) ( path[k] % m_w.cells ) );
                    }
                }

                why = brokenBy( n );
            }

            if( why.empty() )
            {
                markNet( n );
                continue;
            }

            net = NET_STATE();
            net.reached.assign( m_w.nets[n].ends.size(), false );
            m_firstTodo.push_back( n );
            m_affected.emplace_back( n, why );
        }

        m_restoring = false;

        if( m_lane )
        {
            m_netConflicts.assign( m_nets.size(), {} );
            m_conflictNets.clear();

            for( size_t n = 0; n < m_nets.size(); ++n )
            {
                crowdedTiles( (int) n, m_netConflicts[n] );

                if( !m_netConflicts[n].empty() )
                    m_conflictNets.push_back( (int) n );
            }

            m_changed = { INT_MAX, INT_MAX, INT_MIN, INT_MIN };
        }

        m_firstRound = (int) rounds + 1;
        m_round = m_firstRound;
        m_conflicting = byRound.empty() ? 0 : byRound.back();
        m_expanded = at.expanded;
        m_restoreMs = msSince( started );
    }

    /// @brief A warm branch: the nets that lost their copper at the step, and why
    const std::vector<std::pair<int, std::string>>& Affected() const { return m_affected; }
    int                                             FirstRound() const { return m_firstRound; }

    /// @brief The replay met a net other than the recorded one: the board or the router differ
    bool   Diverged() const { return m_diverged; }
    size_t ReplayedSteps() const { return m_replayAt; }
    double RestoreMs() const { return m_restoreMs; }

    /// @brief Another variant has joined everything while this one still negotiates (or, aAny,
    /// this one is too far behind the first finished): it stops
    void Abandon( bool aAny = false )
    {
        if( aAny || m_stage == STAGE_ROUNDS )
            m_abandoned = true;
    }

    bool Abandoned() const { return m_abandoned; }

    /// @brief Run ended before its stages did, not by another variant's win (a cancelled job)
    bool Halted() const { return m_stage != STAGE_DONE && !m_abandoned; }

    /// @brief After Run: every end joined, no net dropped
    bool Complete() const
    {
        if( m_abandoned || m_dropped > 0 )
            return false;

        return std::all_of( m_nets.begin(), m_nets.end(),
                            []( const NET_STATE& aNet )
                            { return std::find( aNet.reached.begin(), aNet.reached.end(), false ) == aNet.reached.end(); } );
    }

    GRID_VARIANT_PROGRESS Progress() const
    {
        static const char* const names[] = { "rounds", "legalise", "short_again", "fewer_vias", "pull_tight", "done" };
        GRID_VARIANT_PROGRESS    p;
        p.name = m_v.name;
        p.stage = m_abandoned ? "abandoned" : names[m_stage.load()];
        p.round = std::min<int>( m_round, m_firstRound - 1 + m_params.maxRounds );
        p.crowded = m_conflicting;
        p.net = m_taken;
        p.nets = m_takenOf;
        p.expanded = m_expandedShown;
        return p;
    }

    struct NET_STATE
    {
        std::vector<std::vector<uint32_t>> paths;   ///< each from the tree to an end, in order
        std::vector<uint32_t>              vias;    ///< cells
        std::vector<bool>                  reached;
        std::vector<std::vector<uint32_t>> markedTrack;   ///< per sizes
        std::vector<std::vector<uint32_t>> markedVia;     ///< per sizes
        std::vector<uint32_t>              markedBand;    ///< beside its copper, for its bundle
    };

    const std::vector<NET_STATE>& Nets() const { return m_nets; }
    int                           Round() const { return m_round; }
    int                           Conflicting() const { return m_conflicting; }
    int                           Dropped() const { return m_dropped; }
    int64_t                       Expanded() const { return m_expanded; }
    const std::vector<int64_t>&   ExpandedBy() const { return m_expandedBy; }
    const std::vector<int64_t>&   ExpandedByStage() const { return m_byStage; }
    const std::vector<int>&       ByRound() const { return m_byRound; }
    const std::vector<CROWDING>&  Crowded() const { return m_crowded; }

private:
    enum STAGE { STAGE_ROUNDS, STAGE_LEGALISE, STAGE_SHORT_AGAIN, STAGE_FEWER_VIAS, STAGE_PULL_TIGHT, STAGE_DONE };

    void at( STAGE aStage, int aTaken, int aOf )
    {
        m_stage = aStage;
        m_taken = aTaken;
        m_takenOf = aOf;
        m_expandedShown = m_expanded;
    }

    /// @brief The net was taken (routed, kept, pulled tight or dropped): one step; a paused job
    /// waits here
    void record( int aNet, int aStage )
    {
        if( m_restoring )
            return;

        if( replaying() )
        {
            if( m_replay->steps[m_replayAt].net != aNet )
            {
                m_diverged = true;
                return;
            }

            if( m_lane )
                recordConflicts( aNet );

            if( ++m_replayAt == m_replayEnd )
                m_restoreMs = msSince( m_replayStarted );

            return;
        }

        recordStep( aNet, aStage );
        gate();
    }

    /// @brief While the job is paused: wait, or go on as far as its last job_step allows (n nets,
    /// or to the n-th round from here; after the rounds to the next stage)
    void gate()
    {
        if( !m_params.held )
            return;

        for( ;; )
        {
            uint64_t    serial = 0;
            std::string by;
            int         n = 0;

            if( stopped() || !m_params.held( serial, by, n ) )
                return;

            if( serial != m_stepSeen )
            {
                m_stepSeen = serial;
                m_stepLeft = by == "round" ? 0 : std::max( n, 1 );
                m_stepRound = by == "round" ? m_round + std::max( n, 1 ) : -1;
                m_stepStage = m_stage;
            }

            if( m_stepLeft > 0 )
            {
                --m_stepLeft;
                return;
            }

            if( m_stepRound >= 0 && m_stage == m_stepStage && ( m_stage != STAGE_ROUNDS || m_round < m_stepRound ) )
                return;

            std::this_thread::sleep_for( std::chrono::milliseconds( 2 ) );
        }
    }

    void recordStep( int aNet, int aStage )
    {
        if( !m_lane )
            return;

        GRID_STEP st;
        st.stage = (uint8_t) aStage;
        st.round = (uint16_t) std::min( m_round.load(), 65535 );
        st.net = aNet;
        st.crowded = m_conflicting;
        st.expanded = m_expanded;
        std::copy( m_box.begin(), m_box.end(), st.box );
        st.ms = (float) msSince( m_started );
        st.conflicts = recordConflicts( aNet );

        const NET_STATE& net = m_nets[aNet];
        GRID_PATH        path{ net.paths, net.reached };
        m_lane->Add( st, &path );
    }

    /// @brief A tile of the timeline (x, y) holding a cell
    std::pair<int, int> tileOf( uint32_t aCell ) const
    {
        return { (int) ( aCell % m_w.grid.nx ) / m_timeline->tile, (int) ( aCell / m_w.grid.nx ) / m_timeline->tile };
    }

    /// @brief Widens the tiles changed since the last step by a net's copper (and a tile around it)
    void touchChanged( int aNet )
    {
        auto add = [&]( uint32_t aCell )
        {
            const auto [x, y] = tileOf( aCell );
            m_changed = { std::min( m_changed[0], x - 1 ), std::min( m_changed[1], y - 1 ), std::max( m_changed[2], x + 1 ),
                          std::max( m_changed[3], y + 1 ) };
        };

        for( const auto& path : m_nets[aNet].paths )
        {
            for( uint32_t s : path )
                add( (uint32_t) ( s % m_w.cells ) );
        }

        for( uint32_t i : m_nets[aNet].vias )
            add( i );
    }

    /// @brief The tiles where a net crowds another now, one entry per crowded state
    void crowdedTiles( int aNet, std::vector<uint32_t>& aOut ) const
    {
        aOut.clear();
        const int k = m_w.nets[aNet].sizes;

        auto add = [&]( uint32_t aCell )
        {
            const auto [x, y] = tileOf( aCell );
            aOut.push_back( (uint32_t) ( y * m_timeline->tnx + x ) );
        };

        for( const auto& path : m_nets[aNet].paths )
        {
            for( uint32_t s : path )
            {
                if( m_occTrack[k][s] >= 2 && !m_w.InOwnEnd( s, m_w.nets[aNet].code ) )
                    add( (uint32_t) ( s % m_w.cells ) );
            }
        }

        for( uint32_t i : m_nets[aNet].vias )
        {
            if( m_occVia[k][i] >= 2 )
                add( i );
        }
    }

    /// @brief The places crowded after a net was taken: the net's own, and again those of the nets
    /// crowded before where the copper changed; returns the set's id in the lane
    uint32_t recordConflicts( int aNet )
    {
        if( m_netConflicts.empty() )
            m_netConflicts.resize( m_nets.size() );

        touchChanged( aNet );
        crowdedTiles( aNet, m_netConflicts[aNet] );

        std::vector<int> still;

        for( int n : m_conflictNets )
        {
            if( n == aNet )
                continue;

            for( uint32_t t : m_netConflicts[n] )
            {
                const int x = (int) ( t % m_timeline->tnx ), y = (int) ( t / m_timeline->tnx );

                if( x >= m_changed[0] && x <= m_changed[2] && y >= m_changed[1] && y <= m_changed[3] )
                {
                    crowdedTiles( n, m_netConflicts[n] );
                    break;
                }
            }

            if( !m_netConflicts[n].empty() )
                still.push_back( n );
        }

        if( !m_netConflicts[aNet].empty() )
            still.push_back( aNet );

        m_conflictNets.swap( still );
        m_changed = { INT_MAX, INT_MAX, INT_MIN, INT_MIN };

        std::vector<uint32_t> all;

        for( int n : m_conflictNets )
            all.insert( all.end(), m_netConflicts[n].begin(), m_netConflicts[n].end() );

        std::sort( all.begin(), all.end() );
        std::vector<uint32_t> set;

        for( size_t i = 0; i < all.size(); )
        {
            size_t j = i;

            while( j < all.size() && all[j] == all[i] )
                ++j;

            set.push_back( all[i] << 8 | (uint32_t) std::min<size_t>( j - i, 255 ) );
            i = j;
        }

        return m_lane->AddConflicts( set );
    }

    /// @brief The present factor's growth after this round: fixed, or slow while the crowded nets
    /// fall (by a tenth over two rounds) and fast when they stall
    float growth() const
    {
        if( m_v.strategy.schedule == 0 )
            return m_v.strategy.pfGrowth;

        const size_t r = m_byRound.size();

        if( r < 3 )
            return 1.3f;

        return m_byRound[r - 1] <= 0.9 * m_byRound[r - 3] ? 1.3f : 2.0f;
    }

    /// @brief History added to a state (track states, then S + via cell) fought over this round:
    /// twice the step once it was fought over in three rounds running (schedule 2)
    float histStep( size_t aState )
    {
        if( !stampsFought() )
            return HIST_STEP;

        if( m_fought.empty() )
        {
            const size_t S = (size_t) m_w.L * m_w.cells;
            m_fought.assign( S + m_w.cells, 0 );
            m_streak.assign( S + m_w.cells, 0 );
        }

        const uint16_t round = (uint16_t) m_round.load();

        if( m_fought[aState] != round )
        {
            m_streak[aState] = m_fought[aState] + 1 == round ? (uint8_t) std::min( 255, m_streak[aState] + 1 ) : 1;
            m_fought[aState] = round;
        }

        return m_v.strategy.schedule >= 2 && m_streak[aState] >= 3 ? 2.0f * HIST_STEP : HIST_STEP;
    }

    /// @brief Whether the round each state was last fought over is kept
    bool stampsFought() const
    {
        const GRID_STRATEGY& st = m_v.strategy;
        return st.schedule >= 2 || st.histDecay < 1.0f || st.recent > 0;
    }

    /// @brief History of the places not fought over this round fades; what is left under a tenth of
    /// a step is gone
    void decayHistory()
    {
        const float  d = m_v.strategy.histDecay;
        const size_t S = (size_t) m_w.L * m_w.cells;
        const uint16_t round = (uint16_t) m_round.load();

        auto fade = [&]( float& aHist, size_t aState )
        {
            if( aHist > 0 && ( m_fought.empty() || m_fought[aState] != round ) )
            {
                aHist *= d;

                if( aHist < 0.1f * HIST_STEP )
                    aHist = 0;
            }
        };

        for( size_t s = 0; s < S; ++s )
            fade( m_hist[s], s );

        for( size_t i = 0; i < m_histVia.size(); ++i )
            fade( m_histVia[i], S + i );
    }

    /// @brief Whether the net passes where nets were fought over in the last rounds (strategy recent)
    bool throughRecentHistory( int aNet ) const
    {
        if( m_fought.empty() )
            return false;

        const size_t S = (size_t) m_w.L * m_w.cells;
        const int    since = m_round - m_v.strategy.recent;

        auto recent = [&]( size_t aState ) { return m_fought[aState] && (int) m_fought[aState] >= since; };

        for( const auto& path : m_nets[aNet].paths )
        {
            for( uint32_t s : path )
            {
                if( recent( s ) )
                    return true;
            }
        }

        for( uint32_t i : m_nets[aNet].vias )
        {
            if( recent( S + i ) )
                return true;
        }

        return false;
    }

    /// @brief After a round: its history increments and the heat per tile
    void recordRound()
    {
        if( !m_lane )
            return;

        // in a replay the round is the parent's, already in the lane
        if( replaying() )
        {
            m_histTaken.clear();
            return;
        }

        const GRID_TIMELINE& t = *m_timeline;
        const PLAN_GRID&     g = m_w.grid;
        std::vector<uint16_t> heat( (size_t) t.tnx * t.tny, 0 );

        for( int l = 0; l < m_w.L; ++l )
        {
            const float* hist = m_hist.data() + (size_t) l * m_w.cells;

            for( int y = 0; y < g.ny; ++y )
            {
                uint16_t* row = heat.data() + (size_t) ( y / t.tile ) * t.tnx;

                for( int x = 0; x < g.nx; ++x )
                {
                    const float v = hist[(size_t) y * g.nx + x];

                    if( v > 0 )
                    {
                        uint16_t& h = row[x / t.tile];
                        h = std::max<uint16_t>( h, (uint16_t) std::min( v, 65535.0f ) );
                    }
                }
            }
        }

        std::lock_guard<std::recursive_mutex> lock( m_lane->Mutex() );
        m_lane->histByRound.push_back( std::move( m_histTaken ) );
        m_histTaken.clear();
        m_lane->heatByRound.push_back( std::move( heat ) );
    }

    bool replaying() const { return m_replay && m_replayAt < m_replayEnd; }

    /// @brief Why a net's copper does not hold in this world (empty: it holds): over copper or a
    /// keep-out not its own, its ends moved, or ends left open
    std::string brokenBy( int aNet ) const
    {
        const WORLD::NET& wn = m_w.nets[aNet];
        const NET_STATE&  net = m_nets[aNet];

        if( net.reached.size() != wn.ends.size() )
            return "its pads changed";

        std::unordered_map<uint32_t, int> endAt;

        for( size_t e = 0; e < wn.ends.size(); ++e )
        {
            for( uint32_t s : wn.ends[e] )
                endAt.emplace( s, (int) e );
        }

        size_t first = 0;

        while( first < wn.ends.size() && ( wn.ends[first].empty() || wn.walled[first] ) )
            first++;

        if( first == wn.ends.size() || !net.reached[first] )
            return "its pads changed";

        std::unordered_set<uint32_t> tree( wn.ends[first].begin(), wn.ends[first].end() );
        std::vector<bool>            reached( wn.ends.size(), false );
        reached[first] = true;

        for( const auto& path : net.paths )
        {
            if( path.empty() )
                continue;

            auto end = endAt.find( path.back() );

            if( !tree.count( path.front() ) || end == endAt.end() )
                return "its pads moved";

            for( size_t k = 0; k < path.size(); ++k )
            {
                const uint32_t s = path[k];

                if( !m_w.TrackOk( wn.sizes, s, wn.code ) && !m_w.InOwnEnd( s, wn.code ) )
                    return "crosses copper or a keep-out";

                if( k > 0 && s % m_w.cells == path[k - 1] % m_w.cells && !m_w.ViaOk( wn.sizes, (uint32_t) ( s % m_w.cells ), wn.code ) )
                    return "a via over copper or a keep-out";
            }

            reached[end->second] = true;
            tree.insert( path.begin(), path.end() );
            tree.insert( wn.ends[end->second].begin(), wn.ends[end->second].end() );
        }

        for( size_t e = 0; e < wn.ends.size(); ++e )
        {
            if( !reached[e] && !wn.ends[e].empty() && !wn.walled[e] )
                return "not joined";
        }

        return std::string();
    }

    /// @brief In a replay: the recorded copper of the step it is at, for aNet
    void takeRecorded( int aNet )
    {
        const GRID_STEP& st = m_replay->steps[m_replayAt];

        if( st.net != aNet )
        {
            m_diverged = true;
            return;
        }

        for( ; m_replayDead < m_replay->dead.size() && m_replay->dead[m_replayDead].step <= m_replayAt; ++m_replayDead )
        {
            const GRID_LANE::DEAD& d = m_replay->dead[m_replayDead];
            ( d.keepOut ? m_keepDead : m_dead )[d.net][d.end] = true;
        }

        const WORLD::NET& wn = m_w.nets[aNet];
        NET_STATE&        net = m_nets[aNet];

        if( st.path != GRID_STEP::NONE )
        {
            net.paths = m_replay->paths[st.path].paths;
            net.reached = m_replay->paths[st.path].reached;
        }
        else
        {
            // no copper: the first end alone, as the search leaves it
            net.paths.clear();
            net.reached.assign( wn.ends.size(), false );

            for( size_t e = 0; e < wn.ends.size(); ++e )
            {
                if( !wn.ends[e].empty() && !wn.walled[e] )
                {
                    net.reached[e] = true;
                    break;
                }
            }
        }

        // pulled tight a net keeps its vias
        if( st.stage != STAGE_PULL_TIGHT )
        {
            net.vias.clear();

            for( const auto& path : net.paths )
            {
                for( size_t k = 1; k < path.size(); ++k )
                {
                    if( path[k] % m_w.cells == path[k - 1] % m_w.cells )
                        net.vias.push_back( (uint32_t) ( path[k] % m_w.cells ) );
                }
            }
        }

        m_expanded = st.expanded;
        std::copy( std::begin( st.box ), std::end( st.box ), m_box.begin() );
    }

    bool stopped() const
    {
        return m_stop.load( std::memory_order_relaxed ) || m_abandoned.load( std::memory_order_relaxed )
               || m_diverged.load( std::memory_order_relaxed )
               || ( m_params.cancel && m_params.cancel->load( std::memory_order_relaxed ) );
    }

    bool throughHistory( int aNet ) const
    {
        for( const auto& path : m_nets[aNet].paths )
        {
            for( uint32_t s : path )
            {
                if( m_hist[s] > 0 )
                    return true;
            }
        }

        for( uint32_t i : m_nets[aNet].vias )
        {
            if( m_histVia[i] > 0 )
                return true;
        }

        return false;
    }

    /// @brief Where a crowded net is crowded and by whom (the first place), for the report
    CROWDING describeCrowding( int aNet ) const
    {
        CROWDING  out;
        const int k = m_w.nets[aNet].sizes;
        out.net = m_w.nets[aNet].code;

        auto by = [&]( uint32_t aState, bool aVia )
        {
            for( size_t o = 0; o < m_nets.size(); ++o )
            {
                if( (int) o == aNet || m_nets[o].markedTrack.empty() )
                    continue;

                const std::vector<uint32_t>& marked = aVia ? m_nets[o].markedVia[k] : m_nets[o].markedTrack[k];

                if( std::binary_search( marked.begin(), marked.end(), aState ) )
                    out.by.push_back( m_w.nets[o].code );
            }
        };

        for( const auto& path : m_nets[aNet].paths )
        {
            for( uint32_t s : path )
            {
                if( out.by.empty() && m_occTrack[k][s] >= 2 && !m_w.InOwnEnd( s, out.net ) )
                {
                    out.at = m_w.grid.Centre( (int) ( s % m_w.cells ) );
                    out.layer = (int) ( s / m_w.cells );
                    by( s, false );
                }
            }
        }

        for( uint32_t i : m_nets[aNet].vias )
        {
            if( out.by.empty() && m_occVia[k][i] >= 2 )
            {
                out.at = m_w.grid.Centre( (int) i );
                out.layer = -1;
                by( i, true );
            }
        }

        return out;
    }

    /// @brief Cells along the paths (a diagonal step counts 1.41)
    double length( const NET_STATE& aNet ) const
    {
        double len = 0;

        for( const auto& path : aNet.paths )
        {
            for( size_t k = 1; k < path.size(); ++k )
            {
                const int a = (int) ( path[k - 1] % m_w.cells ), b = (int) ( path[k] % m_w.cells );

                if( a != b )
                    len += ( a % m_w.grid.nx != b % m_w.grid.nx && a / m_w.grid.nx != b / m_w.grid.nx ) ? 1.41421 : 1.0;
            }
        }

        return len;
    }

    /// @brief A path drawn tight: each run on a layer as few straight or two-leg (45 / 90 degree)
    /// pieces as stay clear of the fixed copper and of the other nets, turns of 90 degrees at most;
    /// a piece ends at each of aBranches it passes
    void pullTight( int aNet, std::vector<uint32_t>& aPath, const std::unordered_set<uint32_t>& aBranches ) const
    {
        const WORLD::NET& wn = m_w.nets[aNet];
        const PLAN_GRID&  g = m_w.grid;
        const size_t      cells = m_w.cells;
        const int         k = wn.sizes;

        // between a connector's pin rows only where the path went already
        std::unordered_set<uint32_t> inRows;

        if( !m_w.rows.empty() )
        {
            for( uint32_t s : aPath )
            {
                if( m_w.rows[s % cells] )
                    inRows.insert( s );
            }
        }

        auto free = [&]( uint32_t s )
        {
            if( m_w.InOwnEnd( s, wn.code ) )
                return true;

            if( !m_w.rows.empty() && m_w.rows[s % cells] && !inRows.count( s ) )
                return false;

            return m_w.TrackOk( k, s, wn.code ) && m_occTrack[k][s] == 0;
        };

        auto dirOf = []( int dx, int dy )
        {
            for( int d = 0; d < 8; ++d )
            {
                if( DX[d] == dx && DY[d] == dy )
                    return d;
            }

            return (int) NO_DIR;
        };

        auto sgn = []( int v ) { return ( v > 0 ) - ( v < 0 ); };

        // the cells after aFrom up to aTo on layer aLayer, straight leg first or diagonal leg first;
        // empty when a cell is taken, a diagonal cuts a closed corner or a turn is sharper than 90
        auto legs = [&]( uint32_t aFrom, uint32_t aTo, bool aStraightFirst, int aLastDir, std::vector<uint32_t>& aOut, int& aEndDir )
        {
            aOut.clear();
            const size_t off = aFrom / cells * cells;
            int          x = (int) ( aFrom % cells % g.nx ), y = (int) ( aFrom % cells / g.nx );
            const int    tx = (int) ( aTo % cells % g.nx ), ty = (int) ( aTo % cells / g.nx );
            const int    adx = std::abs( tx - x ), ady = std::abs( ty - y );
            const int    diag = std::min( adx, ady );
            const int    sx = sgn( tx - x ), sy = sgn( ty - y );
            const int    dDiag = dirOf( sx, sy );
            const int    dStraight = adx > ady ? dirOf( sx, 0 ) : dirOf( 0, sy );
            const int    nStraight = std::max( adx, ady ) - diag;
            int          last = aLastDir;

            for( int leg = 0; leg < 2; ++leg )
            {
                const bool straight = ( leg == 0 ) == aStraightFirst;
                const int  d = straight ? dStraight : dDiag;
                const int  count = straight ? nStraight : diag;

                if( count == 0 )
                    continue;

                if( last != NO_DIR )
                {
                    int turn = std::abs( d - last );

                    if( std::min( turn, 8 - turn ) > 2 )
                        return false;
                }

                for( int c = 0; c < count; ++c )
                {
                    if( d & 1 )
                    {
                        const uint32_t a = (uint32_t) ( off + g.Index( x + DX[d], y ) );
                        const uint32_t b = (uint32_t) ( off + g.Index( x, y + DY[d] ) );

                        if( !m_w.TrackOk( k, a, wn.code ) && !m_w.TrackOk( k, b, wn.code ) )
                            return false;
                    }

                    x += DX[d];
                    y += DY[d];
                    const uint32_t s = (uint32_t) ( off + g.Index( x, y ) );

                    if( !free( s ) )
                        return false;

                    aOut.push_back( s );
                }

                last = d;
            }

            aEndDir = last;
            return true;
        };

        std::vector<uint32_t> out;
        std::vector<uint32_t> piece, best;
        int                   lastDir = NO_DIR;
        size_t                i = 0;

        if( !aPath.empty() )
            out.push_back( aPath[0] );

        while( i + 1 < aPath.size() )
        {
            // a via: kept where it is
            if( aPath[i + 1] / cells != aPath[i] / cells )
            {
                out.push_back( aPath[++i] );
                lastDir = NO_DIR;
                continue;
            }

            size_t j = i + 1, bestJ = i;
            int    bestDir = NO_DIR;

            for( ; j < aPath.size() && aPath[j] / cells == aPath[i] / cells; ++j )
            {
                int  endDir = NO_DIR;
                bool ok = legs( aPath[i], aPath[j], true, lastDir, piece, endDir )
                          || legs( aPath[i], aPath[j], false, lastDir, piece, endDir );

                if( ok )
                {
                    bestJ = j;
                    bestDir = endDir;
                    best.swap( piece );
                }
                else if( j > bestJ + 8 )
                {
                    break;
                }

                if( aBranches.count( aPath[j] ) )
                    break;
            }

            if( bestJ == i )
            {
                // not even the next cell by the rules: the path as it was from here
                out.insert( out.end(), aPath.begin() + i + 1, aPath.end() );
                break;
            }

            out.insert( out.end(), best.begin(), best.end() );
            lastDir = bestDir;
            i = bestJ;
        }

        aPath.swap( out );
    }

    int conflicts( int aNet ) const
    {
        const int k = m_w.nets[aNet].sizes;
        int       c = 0;

        for( const auto& path : m_nets[aNet].paths )
        {
            for( uint32_t s : path )
                c += m_occTrack[k][s] >= 2 && !m_w.InOwnEnd( s, m_w.nets[aNet].code );
        }

        for( uint32_t i : m_nets[aNet].vias )
            c += m_occVia[k][i] >= 2;

        return c;
    }

    /// @brief The cells within the disks around the centres, each once
    void dilate( const std::vector<std::pair<const std::vector<uint32_t>*, const DISK*>>& aGroups, size_t aOffset,
                 std::vector<uint32_t>& aOut )
    {
        const PLAN_GRID& g = m_w.grid;
        m_spans.clear();

        for( const auto& [centres, disk] : aGroups )
        {
            for( uint32_t i : *centres )
            {
                const int cx = (int) ( i % g.nx ), cy = (int) ( i / g.nx );

                for( int dy = -disk->r; dy <= disk->r; ++dy )
                {
                    const int h = disk->half[dy + disk->r];
                    const int y = cy + dy;

                    if( h < 0 || y < 0 || y >= g.ny )
                        continue;

                    m_spans.push_back( { y, std::max( 0, cx - h ), std::min( g.nx - 1, cx + h ) } );
                }
            }
        }

        std::sort( m_spans.begin(), m_spans.end() );

        for( size_t k = 0; k < m_spans.size(); )
        {
            const int y = m_spans[k][0];
            int       x0 = m_spans[k][1], x1 = m_spans[k][2];

            for( ++k; k < m_spans.size() && m_spans[k][0] == y && m_spans[k][1] <= x1 + 1; ++k )
                x1 = std::max( x1, m_spans[k][2] );

            for( int x = x0; x <= x1; ++x )
                aOut.push_back( (uint32_t) ( aOffset + g.Index( x, y ) ) );
        }
    }

    void markNet( int aNet )
    {
        NET_STATE&   net = m_nets[aNet];
        const int    a = m_w.nets[aNet].sizes;
        const size_t K = m_w.sizes.size();
        const size_t cells = m_w.cells;

        std::vector<std::vector<uint32_t>> onLayer( m_w.L );
        std::vector<uint32_t>              all;

        for( const auto& path : net.paths )
        {
            for( uint32_t s : path )
            {
                if( m_w.InOwnEnd( s, m_w.nets[aNet].code ) )
                    continue;

                onLayer[s / cells].push_back( (uint32_t) ( s % cells ) );
                all.push_back( (uint32_t) ( s % cells ) );
            }
        }

        for( auto& l : onLayer )
        {
            std::sort( l.begin(), l.end() );
            l.erase( std::unique( l.begin(), l.end() ), l.end() );
        }

        std::sort( all.begin(), all.end() );
        all.erase( std::unique( all.begin(), all.end() ), all.end() );

        net.markedTrack.assign( K, {} );
        net.markedVia.assign( K, {} );

        for( size_t b = 0; b < K; ++b )
        {
            for( int l = 0; l < m_w.L; ++l )
            {
                dilate( { { &onLayer[l], &m_trackDisk[a][b][0] }, { &net.vias, &m_trackDisk[a][b][1] } }, (size_t) l * cells,
                        net.markedTrack[b] );
            }

            dilate( { { &all, &m_viaDisk[a][b][0] }, { &net.vias, &m_viaDisk[a][b][1] } }, 0, net.markedVia[b] );

            for( uint32_t s : net.markedTrack[b] )
                m_occTrack[b][s]++;

            for( uint32_t i : net.markedVia[b] )
                m_occVia[b][i]++;
        }

        if( !m_band.empty() && m_w.nets[aNet].group >= 0 )
        {
            net.markedBand.clear();

            for( int l = 0; l < m_w.L; ++l )
                dilate( { { &onLayer[l], &m_bandDisk[a] } }, (size_t) l * cells, net.markedBand );

            const uint16_t g = (uint16_t) ( m_w.nets[aNet].group + 1 );

            for( uint32_t s : net.markedBand )
            {
                m_band[s]++;
                m_bandGroup[s] = g;
            }
        }

        record( aNet, m_stage );
    }

    void unmark( int aNet )
    {
        NET_STATE& net = m_nets[aNet];

        for( size_t b = 0; b < net.markedTrack.size(); ++b )
        {
            for( uint32_t s : net.markedTrack[b] )
                m_occTrack[b][s]--;

            for( uint32_t i : net.markedVia[b] )
                m_occVia[b][i]--;
        }

        for( uint32_t s : net.markedBand )
        {
            if( --m_band[s] == 0 )
                m_bandGroup[s] = 0;
        }

        net.markedTrack.clear();
        net.markedVia.clear();
        net.markedBand.clear();
        m_box = { -1, -1, -1, -1 };

        if( m_lane )
            touchChanged( aNet );
    }

    void route( int aNet, float aPf, bool aHard )
    {
        const int64_t before = m_expanded;

        if( replaying() )
            takeRecorded( aNet );
        else
            routeTree( aNet, aPf, aHard );

        m_expandedBy[aNet] += m_expanded - before;
    }

    /// @brief The net as a tree: from its first end, each time to the nearest end left
    void routeTree( int aNet, float aPf, bool aHard )
    {
        const WORLD::NET& wn = m_w.nets[aNet];
        NET_STATE&        net = m_nets[aNet];
        net.paths.clear();
        net.vias.clear();
        net.reached.assign( wn.ends.size(), false );

        size_t first = 0;

        while( first < wn.ends.size() && ( wn.ends[first].empty() || wn.walled[first] ) )
            first++;

        if( first == wn.ends.size() )
            return;

        std::vector<uint32_t> tree = wn.ends[first];
        net.reached[first] = true;

        std::vector<int> left;

        for( size_t e = 0; e < wn.ends.size(); ++e )
        {
            if( e != first && !wn.ends[e].empty() && !wn.walled[e] && !m_dead[aNet][e]
                && !( m_keepOut && m_keepDead[aNet][e] ) )
                left.push_back( (int) e );
        }

        while( !left.empty() && !stopped() )
        {
            std::vector<uint32_t> path;
            const int             end = search( aNet, tree, left, aPf, aHard, path );

            if( end < 0 )
            {
                // nothing but fixed copper in the way: no round will reach them
                if( m_walledIn || m_keepWalledIn )
                {
                    for( int e : left )
                    {
                        ( m_walledIn ? m_dead : m_keepDead )[aNet][e] = true;

                        if( m_lane )
                            m_lane->AddDead( aNet, e, !m_walledIn );
                    }
                }

                break;
            }

            net.reached[end] = true;
            std::erase( left, end );

            for( size_t k = 1; k < path.size(); ++k )
            {
                if( path[k] % m_w.cells == path[k - 1] % m_w.cells )
                    net.vias.push_back( (uint32_t) ( path[k] % m_w.cells ) );
            }

            tree.insert( tree.end(), path.begin(), path.end() );
            tree.insert( tree.end(), wn.ends[end].begin(), wn.ends[end].end() );
            net.paths.push_back( std::move( path ) );
        }
    }

    struct OPEN
    {
        float    f;
        float    g;
        uint32_t s;

        bool operator<( const OPEN& o ) const { return f != o.f ? f > o.f : g < o.g; }
    };

    /// @brief A* from the tree to the nearest of the ends left, in a window that grows when it fails
    int search( int aNet, const std::vector<uint32_t>& aTree, const std::vector<int>& aLeft, float aPf, bool aHard,
                std::vector<uint32_t>& aPath )
    {
        const WORLD::NET& wn = m_w.nets[aNet];
        const PLAN_GRID&  g = m_w.grid;
        const size_t      cells = m_w.cells;

        m_stamp++;

        for( int e : aLeft )
        {
            for( uint32_t s : wn.ends[e] )
            {
                m_node[s].target = m_stamp;
                m_targetEnd[s] = e;
            }
        }

        int bx0 = INT_MAX, by0 = INT_MAX, bx1 = INT_MIN, by1 = INT_MIN;

        auto grow = [&]( int x, int y )
        {
            bx0 = std::min( bx0, x );
            by0 = std::min( by0, y );
            bx1 = std::max( bx1, x );
            by1 = std::max( by1, y );
        };

        for( uint32_t s : aTree )
            grow( (int) ( s % cells % g.nx ), (int) ( s % cells / g.nx ) );

        for( int e : aLeft )
        {
            grow( wn.endBox[e][0], wn.endBox[e][1] );
            grow( wn.endBox[e][2], wn.endBox[e][3] );
        }

        const int span = std::max( bx1 - bx0, by1 - by0 );
        m_walledIn = false;
        m_keepWalledIn = false;
        m_keepRefused = false;
        int                   bestEnd = -1;
        float                 bestCost = std::numeric_limits<float>::max();
        std::vector<uint32_t> best;
        bool                  widened = false;
        bool                  flooded = false;
        int       margin = std::max( (int) ( pcbIUScale.mmToIU( 2.0 ) / g.cell ), m_oneWindow ? span : span * 3 / 10 );

        // far apart: the tiles' distance to the ends guides the search round what is in the way
        m_guide = span > GUIDE_SPAN * m_w.tile ? &guide( aNet, aLeft ) : nullptr;

        for( ;; )
        {
            const int  x0 = std::max( 0, bx0 - margin ), y0 = std::max( 0, by0 - margin );
            const int  x1 = std::min( g.nx - 1, bx1 + margin ), y1 = std::min( g.ny - 1, by1 + margin );
            const bool whole = x0 == 0 && y0 == 0 && x1 == g.nx - 1 && y1 == g.ny - 1;

            const int end = searchIn( aNet, aTree, aLeft, aPf, aHard, x0, y0, x1, y1, aPath );

            if( end >= 0 && m_lastCost < bestCost )
            {
                bestEnd = end;
                bestCost = m_lastCost;
                best = aPath;
            }

            // a path through vias may only be the window's: a detour around it is tried once (not
            // when the search never met the window's edge: a wider one finds the same)
            bool through = false;

            for( size_t k = 1; end >= 0 && k < aPath.size() && !through; ++k )
                through = aPath[k] % cells == aPath[k - 1] % cells;

            // a search that never reached the window's edge is the whole board's
            const bool closed = !m_hitEdge && !m_overBudget;

            if( ( end >= 0 && ( !through || widened || !m_hitEdge ) ) || whole || closed || m_oneWindow || stopped() )
            {
                if( bestEnd >= 0 )
                {
                    aPath = std::move( best );
                    return bestEnd;
                }

                // a via the keep-out refused is no fixed copper: the stage without it tries again
                m_walledIn = ( whole || closed ) && !aHard && !m_overBudget && !m_keepRefused && !stopped();

                // only the keep-out in the way: no search reaches them while it holds
                m_keepWalledIn = ( whole || closed ) && !aHard && !m_overBudget && m_keepRefused && m_keepOut && !stopped();
                return -1;
            }

            // a wider window helps only if the ends can be reached at all: a flood over the board's
            // open states says when they cannot (each wider window would search all it holds)
            if( bestEnd < 0 && !flooded )
            {
                flooded = true;
                bool keepRefused = false;

                if( !reachable( aNet, aTree, aHard, keepRefused ) )
                {
                    // as the whole board's search would have found it: fixed copper alone, or with the keep-out
                    m_walledIn = !aHard && !keepRefused;
                    m_keepWalledIn = !aHard && keepRefused && m_keepOut;
                    return -1;
                }
            }

            widened = widened || end >= 0;

            margin = margin * 2 + 1;
            m_stamp++;   // the window's search marks out; targets marked again

            for( int e : aLeft )
            {
                for( uint32_t s : wn.ends[e] )
                {
                    m_node[s].target = m_stamp;
                    m_targetEnd[s] = e;
                }
            }
        }
    }

    /// @brief Each tile's distance to the nearest end left (per layer, over open tiles, vias where one
    /// fits): a lower bound with what is in the way; kept per net and end set, the tiles never change
    const std::vector<uint16_t>& guide( int aNet, const std::vector<int>& aLeft )
    {
        std::vector<int> key = aLeft;
        std::sort( key.begin(), key.end() );
        key.push_back( (int) m_viaScale );
        key.push_back( aNet );

        if( auto it = m_guides.find( key ); it != m_guides.end() )
            return it->second;

        const WORLD::NET& wn = m_w.nets[aNet];
        const size_t      cells = m_w.cells;
        const int         T = m_w.tile, tnx = m_w.tnx, tny = m_w.tny, L = m_w.L, k = wn.sizes;
        const size_t      tiles = (size_t) tnx * tny;
        std::vector<float> dist( tiles * L, std::numeric_limits<float>::max() );
        std::vector<uint8_t> own( tiles * L, 0 );
        std::priority_queue<OPEN> open;

        for( int e = 0; e < (int) wn.ends.size(); ++e )
        {
            const bool target = std::find( aLeft.begin(), aLeft.end(), e ) != aLeft.end();

            for( uint32_t s : wn.ends[e] )
            {
                const uint32_t n = (uint32_t) ( ( s / cells ) * tiles + m_w.TileOf( (int) ( s % cells ) ) );
                own[n] = 1;

                if( target && dist[n] > 0 )
                {
                    dist[n] = 0;
                    open.push( { 0, 0, n } );
                }
            }
        }

        while( !open.empty() )
        {
            const OPEN top = open.top();
            open.pop();

            if( top.g > dist[top.s] )
                continue;

            const int l = (int) ( top.s / tiles ), t = (int) ( top.s % tiles );
            const int x = t % tnx, y = t / tnx;

            auto relax = [&]( uint32_t n, float cost )
            {
                if( top.g + cost < dist[n] )
                {
                    dist[n] = top.g + cost;
                    open.push( { dist[n], dist[n], n } );
                }
            };

            for( int d = 0; d < 8; ++d )
            {
                const int nx = x + DX[d], ny = y + DY[d];

                if( nx < 0 || ny < 0 || nx >= tnx || ny >= tny )
                    continue;

                const uint32_t n = (uint32_t) ( (size_t) l * tiles + ny * tnx + nx );

                if( own[n] || m_w.tileFree[k][n] >= GUIDE_OPEN )
                    relax( n, ( ( d & 1 ) ? 14.0f : 10.0f ) * T );
            }

            if( wn.vias && L > 1 && m_w.tileVia[k][t] )
            {
                for( int to = 0; to < L; ++to )
                {
                    if( to != l )
                        relax( (uint32_t) ( (size_t) to * tiles + t ), m_viaCost * m_viaScale );
                }
            }
        }

        // in tile units, less a tile's diagonal: a cell may sit anywhere in its tile
        std::vector<uint16_t> out( tiles * L );

        for( size_t n = 0; n < out.size(); ++n )
        {
            const float d = dist[n] == std::numeric_limits<float>::max() ? 0.0f : std::max( 0.0f, dist[n] / T - 14.0f );
            out[n] = (uint16_t) std::min( d, 65535.0f );
        }

        // a net of hundreds of ends makes a guide per search: the cache stays within its budget
        if( m_guideBytes + out.size() * sizeof( uint16_t ) > GUIDE_CACHE_BYTES )
        {
            m_guides.clear();
            m_guideBytes = 0;
        }

        m_guideBytes += out.size() * sizeof( uint16_t );
        return m_guides.emplace( std::move( key ), std::move( out ) ).first->second;
    }

    /// @brief Whether any end marked as a target of the current search can be reached from aTree over
    /// the whole board (aHard: other nets' copper closed).  Its moves are a superset of the A*'s (the
    /// same cells, corners and vias, no turn rule), so false means no window of the search reaches one.  aKeepRefused: it met
    /// a via spot only the chips' keep-out closed
    bool reachable( int aNet, const std::vector<uint32_t>& aTree, bool aHard, bool& aKeepRefused )
    {
        const WORLD::NET& wn = m_w.nets[aNet];
        const PLAN_GRID&  g = m_w.grid;
        const size_t      cells = m_w.cells;
        const int         k = wn.sizes;
        const int         L = m_w.L;
        const uint32_t    stamp = m_stamp;

        m_flooded.assign( (size_t) L * cells, false );
        m_floodQueue.clear();

        auto open = [&]( uint32_t n )
        {
            if( m_node[n].target == stamp )
                return true;

            if( !m_w.TrackOk( k, n, wn.code ) )
                return false;

            return !aHard || m_w.InOwnEnd( n, wn.code ) || !m_occTrack[k][n];
        };

        for( uint32_t s : aTree )
        {
            if( !m_flooded[s] )
            {
                m_flooded[s] = true;
                m_floodQueue.push_back( s );
            }
        }

        int64_t visited = 0;

        for( size_t q = 0; q < m_floodQueue.size(); ++q )
        {
            const uint32_t s = m_floodQueue[q];

            if( m_node[s].target == stamp )
            {
                m_expanded += visited;
                return true;
            }

            if( ( ++visited & 65535 ) == 0 && stopped() )
                break;

            const int l = (int) ( s / cells );
            const int i = (int) ( s % cells );
            const int x = i % g.nx, y = i / g.nx;

            for( int d = 0; d < 8; ++d )
            {
                const int nx = x + DX[d], ny = y + DY[d];

                if( nx < 0 || ny < 0 || nx >= g.nx || ny >= g.ny )
                    continue;

                const uint32_t n = (uint32_t) ( (size_t) l * cells + g.Index( nx, ny ) );

                if( m_flooded[n] )
                    continue;

                // as the search: a diagonal step may not cut the corner between two closed cells
                if( d & 1 )
                {
                    const uint32_t a = (uint32_t) ( (size_t) l * cells + g.Index( nx, y ) );
                    const uint32_t b = (uint32_t) ( (size_t) l * cells + g.Index( x, ny ) );

                    if( !m_w.TrackOk( k, a, wn.code ) && !m_w.TrackOk( k, b, wn.code ) )
                        continue;
                }

                if( open( n ) )
                {
                    m_flooded[n] = true;
                    m_floodQueue.push_back( n );
                }
            }

            if( !wn.vias || L < 2 || !m_w.ViaOk( k, (uint32_t) i, wn.code ) )
                continue;

            if( m_keepOut && ( m_w.keep[i] & ~wn.chips ) )
            {
                aKeepRefused = true;
                continue;
            }

            if( aHard && m_occVia[k][i] )
                continue;

            for( int to = 0; to < L; ++to )
            {
                const uint32_t n = (uint32_t) ( (size_t) to * cells + i );

                if( to != l && !m_flooded[n] && open( n ) )
                {
                    m_flooded[n] = true;
                    m_floodQueue.push_back( n );
                }
            }
        }

        m_expanded += visited;

        // stopped: as if reachable, the search itself stops
        return stopped();
    }

    int searchIn( int aNet, const std::vector<uint32_t>& aTree, const std::vector<int>& aLeft, float aPf, bool aHard,
                  int x0, int y0, int x1, int y1, std::vector<uint32_t>& aPath )
    {
        const WORLD::NET& wn = m_w.nets[aNet];
        const PLAN_GRID&  g = m_w.grid;
        const size_t      cells = m_w.cells;
        const int         k = wn.sizes;
        const int         L = m_w.L;
        const uint32_t    stamp = m_stamp;

        std::priority_queue<OPEN> open;

        const size_t tiles = (size_t) m_w.tnx * m_w.tny;

        if( m_lane )
        {
            if( m_box[0] < 0 )
                m_box = { x0, y0, x1, y1 };
            else
                m_box = { std::min( m_box[0], x0 ), std::min( m_box[1], y0 ), std::max( m_box[2], x1 ), std::max( m_box[3], y1 ) };
        }

        auto boxDistance = [&]( int e, int x, int y )
        {
            const auto& b = wn.endBox[e];
            const int   dx = std::max( { b[0] - x, 0, x - b[2] } );
            const int   dy = std::max( { b[1] - y, 0, y - b[3] } );
            return 10 * std::max( dx, dy ) + 4 * std::min( dx, dy );
        };

        // many ends: looked up by buckets of cells, nearest rings first (the same minimum)
        const bool bucketed = aLeft.size() > END_BUCKETS_FROM;

        if( bucketed )
            m_endIndex.Build( wn.endBox, aLeft, g.nx, g.ny );

        auto h = [&]( int x, int y, int l )
        {
            int best = INT_MAX;

            if( bucketed )
            {
                const END_INDEX& ix = m_endIndex;
                const int        bx = x / ix.B, by = y / ix.B;
                const int        rings = std::max( ix.nx, ix.ny );

                for( int r = 0; r <= rings; ++r )
                {
                    // a box in ring r is at least (r - 1) * B + 1 cells away on one axis
                    if( r > 0 && best <= 10 * ( ( r - 1 ) * ix.B + 1 ) )
                        break;

                    for( int cy = by - r; cy <= by + r; ++cy )
                    {
                        if( cy < 0 || cy >= ix.ny )
                            continue;

                        const bool edgeRow = cy == by - r || cy == by + r;

                        for( int cx = bx - r; cx <= bx + r; cx += ( edgeRow || r == 0 ) ? 1 : 2 * r )
                        {
                            if( cx < 0 || cx >= ix.nx )
                                continue;

                            const size_t c = (size_t) cy * ix.nx + cx;

                            for( int q = ix.start[c]; q < ix.start[c + 1]; ++q )
                                best = std::min( best, boxDistance( ix.items[q], x, y ) );
                        }
                    }
                }
            }
            else
            {
                for( int e : aLeft )
                    best = std::min( best, boxDistance( e, x, y ) );
            }

            if( m_guide )
                best = std::max( best, (int) ( *m_guide )[(size_t) l * tiles + ( y / m_w.tile ) * m_w.tnx + x / m_w.tile] * m_w.tile );

            return m_v.strategy.astar * (float) best;
        };

        for( uint32_t s : aTree )
        {
            if( m_node[s].seen == stamp && m_node[s].g == 0 )
                continue;

            const int i = (int) ( s % cells );
            const int x = i % g.nx, y = i / g.nx;

            if( x < x0 || x > x1 || y < y0 || y > y1 )
                continue;

            m_node[s].seen = stamp;
            m_node[s].g = 0;
            m_node[s].parent = -1;
            m_dir[s] = NO_DIR;
            open.push( { h( x, y, (int) ( s / cells ) ), 0, s } );
        }

        const int64_t budget = 4LL * ( x1 - x0 + 1 ) * ( y1 - y0 + 1 ) * L;
        int64_t       taken = 0;
        m_overBudget = false;
        m_hitEdge = false;

        auto cellCost = [&]( uint32_t s ) -> float
        {
            if( m_w.InOwnEnd( s, wn.code ) )
                return 1.0f;

            const uint16_t occ = m_occTrack[k][s];
            const float    hug = !occ && wn.group >= 0 && !m_band.empty() && m_band[s] && m_bandGroup[s] == wn.group + 1
                                         ? m_params.hug
                                         : 1.0f;

            const float rows = !m_w.rows.empty() && m_w.rows[s % cells] ? m_params.rowsCost : 1.0f;

            if( aHard )
                return occ ? -1.0f : hug * rows;

            return ( 1.0f + m_hist[s] ) * ( 1.0f + aPf * occ ) * hug * rows;
        };

        while( !open.empty() )
        {
            const OPEN top = open.top();
            open.pop();

            if( top.g > m_node[top.s].g )
                continue;

            if( m_node[top.s].target == stamp )
            {
                aPath.clear();

                for( int64_t s = top.s; s >= 0; s = m_node[s].parent )
                    aPath.push_back( (uint32_t) s );

                std::reverse( aPath.begin(), aPath.end() );
                m_lastCost = top.g;
                m_expanded += taken;
                return m_targetEnd[top.s];
            }

            if( ++taken > budget || ( ( taken & 4095 ) == 0 && stopped() ) )
            {
                m_overBudget = taken > budget;
                break;
            }

            const uint32_t s = top.s;
            const int      l = (int) ( s / cells );
            const int      i = (int) ( s % cells );
            const int      x = i % g.nx, y = i / g.nx;
            const int      dir = m_dir[s];
            const int      pref = m_v.direction.empty() ? 0 : m_v.direction[l];

            auto relax = [&]( uint32_t n, float cost, int nd, int nx, int ny, int nl )
            {
                const float ng = top.g + cost;

                if( m_node[n].seen == stamp && ng >= m_node[n].g )
                    return;

                m_node[n].seen = stamp;
                m_node[n].g = ng;
                m_node[n].parent = (int32_t) s;
                m_dir[n] = (uint8_t) nd;
                open.push( { ng + h( nx, ny, nl ), ng, n } );
            };

            for( int d = 0; d < 8; ++d )
            {
                const int nx = x + DX[d], ny = y + DY[d];

                if( nx < x0 || nx > x1 || ny < y0 || ny > y1 )
                {
                    m_hitEdge = m_hitEdge || ( nx >= 0 && ny >= 0 && nx < g.nx && ny < g.ny );
                    continue;
                }

                // no sharp turns
                int turn = 0;

                if( dir != NO_DIR )
                {
                    turn = std::abs( d - dir );
                    turn = std::min( turn, 8 - turn );

                    if( turn > 2 )
                        continue;
                }

                const uint32_t n = (uint32_t) ( (size_t) l * cells + g.Index( nx, ny ) );
                const bool     target = m_node[n].target == stamp;

                if( !target && !m_w.TrackOk( k, n, wn.code ) )
                    continue;

                // a diagonal step may not cut the corner between two closed cells
                if( d & 1 )
                {
                    const uint32_t a = (uint32_t) ( (size_t) l * cells + g.Index( x + DX[d], y ) );
                    const uint32_t b = (uint32_t) ( (size_t) l * cells + g.Index( x, y + DY[d] ) );

                    if( !m_w.TrackOk( k, a, wn.code ) && !m_w.TrackOk( k, b, wn.code ) )
                        continue;
                }

                const float c = target ? 1.0f : cellCost( n );

                if( c < 0 )
                    continue;

                float step = ( d & 1 ) ? 14.0f : 10.0f;

                if( pref == 1 && ( d & 3 ) != 0 )
                    step *= ( d & 1 ) ? 1.3f : 1.6f;
                else if( pref == 2 && d != 2 && d != 6 )
                    step *= ( d & 1 ) ? 1.3f : 1.6f;

                relax( n, step * c + ( turn == 1 ? 3.0f : turn == 2 ? 8.0f : 0.0f ), d, nx, ny, l );
            }

            if( wn.vias && L > 1 && m_keepOut && ( m_w.keep[i] & ~wn.chips ) )
            {
                m_keepRefused = m_keepRefused || m_w.ViaOk( k, (uint32_t) i, wn.code );
            }
            else if( wn.vias && L > 1 && m_w.ViaOk( k, (uint32_t) i, wn.code ) )
            {
                const uint16_t occ = m_occVia[k][i];

                if( aHard && occ )
                    continue;

                const float base = m_viaCost * m_viaScale * ( aHard ? 1.0f : ( 1.0f + m_histVia[i] ) * ( 1.0f + aPf * occ ) )
                                   + ( m_w.fine[i] ? m_fineCost : 0.0f )
                                   + ( !m_w.away.empty() && m_w.away[i] < m_awayCells ? m_awayCost * ( 1.0f - m_w.away[i] / m_awayCells ) : 0.0f )
                                   + nearHole( wn, x, y );

                for( int to = 0; to < L; ++to )
                {
                    if( to == l )
                        continue;

                    const uint32_t n = (uint32_t) ( (size_t) to * cells + i );

                    if( m_node[n].target != stamp && !m_w.TrackOk( k, n, wn.code ) )
                        continue;

                    const float c = m_node[n].target == stamp ? 1.0f : cellCost( n );

                    if( c < 0 )
                        continue;

                    relax( n, base * c, NO_DIR, x, y, to );
                }
            }
        }

        m_expanded += taken;
        return -1;
    }

    const WORLD&       m_w;
    const VARIANT&     m_v;
    const GRID_PARAMS& m_params;
    std::atomic<bool>& m_stop;

    std::vector<std::vector<uint16_t>> m_occTrack;
    std::vector<std::vector<uint16_t>> m_occVia;
    std::vector<float>                 m_hist;
    std::vector<float>                 m_histVia;

    /// @brief A state's search data, together: one cache line holds four states' worth
    struct NODE
    {
        uint32_t seen = 0;     ///< stamp of the search that reached it
        uint32_t target = 0;   ///< stamp of the search it is an end of
        float    g = 0;
        int32_t  parent = -1;
    };

    /// @brief The ends left of a search by buckets of B x B cells (each end in every bucket its box meets)
    struct END_INDEX
    {
        static constexpr int B = 32;

        int              nx = 0, ny = 0;
        std::vector<int> start;   ///< per bucket: its first item; one more at the end
        std::vector<int> items;

        void Build( const std::vector<std::array<int, 4>>& aBoxes, const std::vector<int>& aEnds, int aCellsX, int aCellsY )
        {
            nx = ( aCellsX + B - 1 ) / B;
            ny = ( aCellsY + B - 1 ) / B;
            start.assign( (size_t) nx * ny + 1, 0 );

            auto each = [&]( int aEnd, auto&& aFn )
            {
                const auto& b = aBoxes[aEnd];

                for( int cy = std::max( 0, b[1] / B ); cy <= std::min( ny - 1, b[3] / B ); ++cy )
                {
                    for( int cx = std::max( 0, b[0] / B ); cx <= std::min( nx - 1, b[2] / B ); ++cx )
                        aFn( (size_t) cy * nx + cx );
                }
            };

            for( int e : aEnds )
                each( e, [&]( size_t c ) { start[c + 1]++; } );

            for( size_t c = 0; c + 1 < start.size(); ++c )
                start[c + 1] += start[c];

            items.assign( start.back(), 0 );
            std::vector<int> fill( start.begin(), start.end() - 1 );

            for( int e : aEnds )
                each( e, [&]( size_t c ) { items[fill[c]++] = e; } );
        }
    };

    END_INDEX             m_endIndex;
    std::vector<NODE>     m_node;
    std::vector<bool>     m_flooded;      ///< reachable(): states reached
    std::vector<uint32_t> m_floodQueue;
    std::vector<uint8_t>  m_dir;
    std::vector<int>      m_targetEnd;
    uint32_t              m_stamp = 0;
    bool                  m_overBudget = false;
    bool                  m_hitEdge = false;    ///< the last window's search tried to step out of it
    float                 m_lastCost = 0;
    bool                  m_walledIn = false;   ///< the last search failed over the whole board on fixed copper alone
    bool                  m_keepRefused = false;   ///< the last search met a via spot only the chips' keep-out closed
    bool                  m_keepWalledIn = false;  ///< the last search failed over the whole board on fixed copper and the keep-out

    std::vector<std::vector<bool>> m_dead;   ///< per net, per end: walled in by fixed copper
    std::vector<std::vector<bool>> m_keepDead;   ///< per net, per end: walled in while the chips' via keep-out holds

    std::vector<std::vector<std::array<DISK, 2>>> m_trackDisk;   ///< [from sizes][onto sizes]: track, via
    std::vector<std::vector<std::array<DISK, 2>>> m_viaDisk;
    std::vector<std::array<int, 3>>               m_spans;

    float m_viaCost = 0;
    float m_viaScale = 1.0f;
    bool  m_keepOut = true;   ///< no vias around the chips but their own nets'
    bool  m_oneWindow = false;   ///< the first window only: no detour around it
    float m_fineCost = 0;
    float m_awayCells = 0;
    float m_awayCost = 0;
    float m_nearHoleCells = 0;
    float m_nearHoleCost = 0;

    /// @brief A via beside a through hole of its own net: the hole is on every layer already
    float nearHole( const WORLD::NET& aNet, int aX, int aY ) const
    {
        float worst = 0;

        for( const auto& [hx, hy] : aNet.holes )
        {
            const float d = std::hypot( (float) ( aX - hx ), (float) ( aY - hy ) );

            if( d < m_nearHoleCells )
                worst = std::max( worst, m_nearHoleCost * ( 1.0f - d / m_nearHoleCells ) );
        }

        return worst;
    }

    std::vector<uint16_t> m_band;        ///< states beside bundle copper: how many members' copper
    std::vector<uint16_t> m_bandGroup;   ///< and of which bundle (+1)
    std::vector<DISK>     m_bandDisk;    ///< per sizes: a member's track and the place beside it

    std::vector<NET_STATE> m_nets;
    std::atomic<int>       m_round{ 0 };
    std::atomic<int>       m_conflicting{ 0 };
    std::atomic<int>       m_stage{ STAGE_ROUNDS };
    uint64_t               m_stepSeen = 0;    ///< the job_step this variant went on for
    int                    m_stepLeft = 0;
    int                    m_stepRound = -1;
    int                    m_stepStage = -1;
    std::atomic<bool>      m_abandoned{ false };
    std::atomic<int>       m_taken{ 0 };
    std::atomic<int>       m_takenOf{ 0 };
    std::atomic<int64_t>   m_expandedShown{ 0 };
    int                    m_dropped = 0;
    std::vector<int>       m_byRound;   ///< nets still crowded after each round
    std::vector<int64_t>   m_expandedBy;   ///< search states per net, all rounds
    std::map<std::vector<int>, std::vector<uint16_t>> m_guides;   ///< per end set, via scale, net
    size_t                                            m_guideBytes = 0;
    const std::vector<uint16_t>*                      m_guide = nullptr;
    std::vector<int64_t>   m_byStage;      ///< search states: rounds, legalising, the short again, fewer vias
    std::vector<CROWDING>  m_crowded;   ///< nets still crowded when the rounds ended
    int64_t                m_expanded = 0;

    const GRID_TIMELINE*   m_timeline = nullptr;
    GRID_LANE*             m_lane = nullptr;
    const GRID_LANE*       m_replay = nullptr;   ///< a branch's parent, taken up to m_replayEnd steps
    size_t                 m_replayEnd = 0;
    size_t                 m_replayAt = 0;
    size_t                 m_replayDead = 0;
    std::atomic<bool>      m_diverged{ false };
    CLOCK::time_point      m_replayStarted;
    double                 m_restoreMs = 0;
    bool                   m_restoring = false;   ///< a warm start marks the copper it took: no steps
    bool                   m_warm = false;
    int                    m_firstRound = 1;
    float                  m_pf = 1.0f;           ///< the present factor of the round
    std::vector<int>       m_firstTodo;           ///< a warm start's first round: the nets it broke
    std::vector<std::pair<int, std::string>> m_affected;
    CLOCK::time_point      m_started;
    std::array<int, 4>     m_box{ -1, -1, -1, -1 };   ///< cells searched in since the net was taken
    std::vector<uint32_t>  m_histTaken;   ///< this round's history increments
    std::array<int, 4>     m_changed{ INT_MAX, INT_MAX, INT_MIN, INT_MIN };   ///< tiles changed since the last step
    std::vector<std::vector<uint32_t>> m_netConflicts;   ///< per net: the tiles of its crowded states
    std::vector<int>       m_conflictNets;   ///< the nets crowded at the last step
    std::vector<uint16_t>  m_fought;      ///< per state: the last round it was fought over
    std::vector<uint8_t>   m_streak;      ///< and in how many rounds running
};


GRID_NET_RESULT toGeometry( const WORLD& aWorld, const GRID_PARAMS& aParams, const NEGOTIATION::NET_STATE& aNet, size_t aIndex )
{
    GRID_NET_RESULT out = GridPathGeometry( aWorld.grid, aWorld.cells, aParams.layers, aNet.paths, aNet.reached );

    if( aIndex < aWorld.nets.size() )
        GridAddExits( out, aWorld.nets[aIndex].exits, aNet.paths );

    return out;
}

} // namespace


/// @brief The board as the search sees it, hashed: a branch must route over the same. Nets by name:
/// a board read again numbers them anew
uint64_t worldPrint( const WORLD& aWorld, const BOARD* aBoard )
{
    uint64_t h = 1469598103934665603ull;

    std::unordered_map<int32_t, uint64_t> byCode;

    auto net = [&]( int32_t aCode ) -> uint64_t
    {
        if( aCode <= 0 )
            return (uint64_t) (int64_t) aCode;

        auto [it, added] = byCode.try_emplace( aCode, 0 );

        if( added )
        {
            const NETINFO_ITEM* info = aBoard->FindNet( aCode );
            it->second = std::hash<std::string>()( info ? info->GetNetname().ToStdString() : std::string() ) | 1ull << 63;
        }

        return it->second;
    };

    auto mix = [&]( uint64_t v )
    {
        h ^= v;
        h *= 1099511628211ull;
    };

    auto mixAll = [&]( const auto& aValues )
    {
        mix( aValues.size() );

        for( const auto& v : aValues )
            mix( (uint64_t) v );
    };

    mix( (uint64_t) aWorld.grid.origin.x );
    mix( (uint64_t) aWorld.grid.origin.y );
    mix( (uint64_t) aWorld.grid.cell );
    mix( (uint64_t) aWorld.grid.nx );
    mix( (uint64_t) aWorld.grid.ny );
    mix( (uint64_t) aWorld.L );
    mix( (uint64_t) aWorld.h2h );
    mix( (uint64_t) aWorld.holeClearance );

    for( const SIZES& sz : aWorld.sizes )
    {
        mix( (uint64_t) sz.halfWidth );
        mix( (uint64_t) sz.clearance );
        mix( (uint64_t) sz.viaRadius );
        mix( (uint64_t) sz.viaDrill );
    }

    auto mixNets = [&]( const std::vector<int32_t>& aOwners )
    {
        mix( aOwners.size() );

        for( int32_t v : aOwners )
            mix( net( v ) );
    };

    for( const auto& t : aWorld.track )
        mixNets( t );

    for( const auto& v : aWorld.via )
        mixNets( v );

    mixAll( aWorld.fine );
    mixAll( aWorld.keep );
    mixAll( aWorld.away );
    mixNets( aWorld.endOf );

    for( const WORLD::NET& n : aWorld.nets )
    {
        mix( net( n.code ) );
        mix( (uint64_t) n.sizes );
        mix( (uint64_t) n.group );
        mix( (uint64_t) n.rank );
        mixAll( n.walled );

        for( const auto& e : n.ends )
            mixAll( e );
    }

    return h;
}


/// @brief Per tile of aTile cells (aTnx across): what the search sees there hashed, nets by name, so
/// two worlds on the same grid tell where they differ
std::vector<uint64_t> tilePrints( const WORLD& aWorld, const BOARD* aBoard, int aTile, int aTnx, int aTny )
{
    std::vector<uint64_t> h( (size_t) aTnx * aTny, 1469598103934665603ull );
    std::unordered_map<int32_t, uint64_t> byCode;

    auto net = [&]( int32_t aCode ) -> uint64_t
    {
        if( aCode <= 0 )
            return (uint64_t) (int64_t) aCode;

        auto [it, added] = byCode.try_emplace( aCode, 0 );

        if( added )
        {
            const NETINFO_ITEM* info = aBoard->FindNet( aCode );
            it->second = std::hash<std::string>()( info ? info->GetNetname().ToStdString() : std::string() ) | 1ull << 63;
        }

        return it->second;
    };

    const PLAN_GRID& g = aWorld.grid;

    for( int y = 0; y < g.ny; ++y )
    {
        for( int x = 0; x < g.nx; ++x )
        {
            const size_t c = (size_t) y * g.nx + x;
            uint64_t&    t = h[(size_t) ( y / aTile ) * aTnx + x / aTile];

            auto mix = [&]( uint64_t v )
            {
                t ^= v;
                t *= 1099511628211ull;
            };

            for( size_t k = 0; k < aWorld.sizes.size(); ++k )
            {
                for( int l = 0; l < aWorld.L; ++l )
                    mix( net( aWorld.track[k][(size_t) l * aWorld.cells + c] ) );

                mix( net( aWorld.via[k][c] ) );
            }

            for( int l = 0; l < aWorld.L; ++l )
                mix( net( aWorld.endOf[(size_t) l * aWorld.cells + c] ) );

            mix( aWorld.keep.empty() ? 0 : aWorld.keep[c] );
            mix( aWorld.fine.empty() ? 0 : aWorld.fine[c] );
            mix( aWorld.away.empty() ? 0 : aWorld.away[c] );
        }
    }

    return h;
}


/// @brief What a warm branch needs to be the same: the grid, layers and sizes (aNets false), or the
/// nets routed by name in order (aNets true)
uint64_t rulesPrint( const WORLD& aWorld, const BOARD* aBoard, bool aNets )
{
    uint64_t h = 1469598103934665603ull;

    auto mix = [&]( uint64_t v )
    {
        h ^= v;
        h *= 1099511628211ull;
    };

    if( aNets )
    {
        for( const WORLD::NET& n : aWorld.nets )
        {
            const NETINFO_ITEM* info = aBoard->FindNet( n.code );
            mix( std::hash<std::string>()( info ? info->GetNetname().ToStdString() : std::string() ) );
        }

        return h;
    }

    mix( (uint64_t) aWorld.grid.origin.x );
    mix( (uint64_t) aWorld.grid.origin.y );
    mix( (uint64_t) aWorld.grid.cell );
    mix( (uint64_t) aWorld.grid.nx );
    mix( (uint64_t) aWorld.grid.ny );
    mix( (uint64_t) aWorld.L );
    mix( (uint64_t) aWorld.h2h );
    mix( (uint64_t) aWorld.holeClearance );

    for( const SIZES& sz : aWorld.sizes )
    {
        mix( (uint64_t) sz.halfWidth );
        mix( (uint64_t) sz.clearance );
        mix( (uint64_t) sz.viaRadius );
        mix( (uint64_t) sz.viaDrill );
    }

    return h;
}


GRID_RESULT RouteOnGrid( BOARD* aBoard, const std::vector<GRID_NET>& aNets, const GRID_PARAMS& aParams )
{
    GRID_RESULT result;
    const auto  started = CLOCK::now();

    if( aNets.empty() || aParams.layers.empty() )
    {
        if( aParams.branchOf )
            result.error = "the board is not the one run " + std::to_string( aParams.branchOf->run )
                           + " started from (none of its nets is left to route)";

        return result;
    }

    // a warm branch routes the nets in the run's order (paths and lanes go by net index): the request
    // may list them otherwise once parts moved
    std::vector<GRID_NET> runOrder;
    std::vector<size_t>   asAsked;   ///< world net -> request net

    if( aParams.warm && aParams.branchOf && aParams.branchOf->netKeys.size() == aNets.size() )
    {
        std::unordered_map<std::string, size_t> byName;

        for( size_t k = 0; k < aNets.size(); ++k )
        {
            const NETINFO_ITEM* info = aBoard->FindNet( aNets[k].netCode );
            byName.emplace( info ? info->GetNetname().ToStdString() : std::string(), k );
        }

        for( const std::string& name : aParams.branchOf->netKeys )
        {
            auto it = byName.find( name );

            if( it == byName.end() )
                break;

            runOrder.push_back( aNets[it->second] );
            asAsked.push_back( it->second );
            byName.erase( it );
        }

        if( runOrder.size() != aNets.size() )
        {
            runOrder.clear();
            asAsked.clear();
        }
    }

    const std::vector<GRID_NET>& nets = runOrder.empty() ? aNets : runOrder;

    WORLD world;
    buildWorld( aBoard, nets, aParams, world );
    findPockets( world );
    const auto halfFrom = CLOCK::now();
    std::tie( result.halfCellTried, result.halfCellWon ) = halfCellExits( world, nets, aParams );
    result.halfCellMs = msSince( halfFrom );
    std::vector<std::vector<MARK>>().swap( world.marks );
    buildTiles( world );
    result.buildMs = msSince( started );
    result.cell = world.grid.cell;

    std::shared_ptr<GRID_TIMELINE> branchOf = aParams.branchOf;
    const auto                     printFrom = CLOCK::now();
    const uint64_t                 print = aParams.record || branchOf ? worldPrint( world, aBoard ) : 0;
    const uint64_t                 rules = rulesPrint( world, aBoard, false );
    const uint64_t                 netsPrint = rulesPrint( world, aBoard, true );
    const int                      tile = std::max( 1, (int) std::lround( pcbIUScale.mmToIU( 1.0 ) / (double) world.grid.cell ) );
    const int                      tnx = ( world.grid.nx + tile - 1 ) / tile, tny = ( world.grid.ny + tile - 1 ) / tile;
    const std::vector<uint64_t>    tiles = aParams.record || branchOf ? tilePrints( world, aBoard, tile, tnx, tny )
                                                                     : std::vector<uint64_t>();
    result.printMs = msSince( printFrom );

    if( branchOf )
    {
        if( aParams.branchLane < 0 || aParams.branchLane >= (int) branchOf->LaneCount()
            || aParams.branchStep >= branchOf->Lane( aParams.branchLane ).steps.size() )
        {
            result.error = "no such lane or step";
            return result;
        }

        const GRID_LANE& from = branchOf->Lane( aParams.branchLane );

        if( aParams.warm )
        {
            const PLAN_GRID& g = branchOf->grid;

            if( g.origin != world.grid.origin || g.cell != world.grid.cell || g.nx != world.grid.nx || g.ny != world.grid.ny
                || branchOf->layers != aParams.layers )
                result.cold = "the board's outline, layers or grid cell changed";
            else if( netsPrint != branchOf->nets )
                result.cold = "the nets to route changed";
            else if( rules != branchOf->rules )
                result.cold = "track or via sizes or clearances changed";

            // cold: a new run over the board as it is
            if( !result.cold.empty() )
                branchOf = nullptr;
        }
        else if( print != ( from.world ? from.world : branchOf->world ) || from.world )
        {
            result.error = from.world ? "lane " + from.name + " went on over an edited board: branch it with warm: true"
                                      : "the board is not the one run " + std::to_string( branchOf->run )
                                                + " started from (its copper, parts, rules or nets changed)";
            return result;
        }
    }

    const auto routing = CLOCK::now();

    // variants: net orders and layer directions
    const int N = (int) world.nets.size();

    auto order = [&]( auto aLess )
    {
        std::vector<int> o( N );
        std::iota( o.begin(), o.end(), 0 );
        std::stable_sort( o.begin(), o.end(), aLess );
        return o;
    };

    const auto shortFirst = order( [&]( int a, int b ) { return world.nets[a].span < world.nets[b].span; } );
    const auto longFirst = order( [&]( int a, int b ) { return world.nets[a].span > world.nets[b].span; } );
    const auto fineFirst = order( [&]( int a, int b )
                                  {
                                      if( world.nets[a].finePitch != world.nets[b].finePitch )
                                          return world.nets[a].finePitch;

                                      return world.nets[a].span < world.nets[b].span;
                                  } );

    // bundles first, the largest first, each member by member; the rest in the variant's order
    std::map<int, int> groupSize;

    for( const WORLD::NET& n : world.nets )
    {
        if( n.group >= 0 )
            groupSize[n.group]++;
    }

    auto bundlesFirst = [&]( std::vector<int> aOrder )
    {
        std::stable_sort( aOrder.begin(), aOrder.end(),
                          [&]( int a, int b )
                          {
                              const WORLD::NET& x = world.nets[a];
                              const WORLD::NET& y = world.nets[b];

                              if( ( x.group >= 0 ) != ( y.group >= 0 ) )
                                  return x.group >= 0;

                              if( x.group < 0 )
                                  return false;

                              if( x.group != y.group )
                              {
                                  const int sx = groupSize[x.group], sy = groupSize[y.group];
                                  return sx != sy ? sx > sy : x.group < y.group;
                              }

                              return x.rank < y.rank;
                          } );
        return aOrder;
    };

    std::vector<int> hv( world.L, 0 ), vh( world.L, 0 );

    for( int l = 0; l < world.L; ++l )
    {
        hv[l] = l % 2 ? 2 : 1;
        vh[l] = l % 2 ? 1 : 2;
    }

    std::vector<GRID_STRATEGY> strategies = aParams.strategies;

    // one lane's search state per state: occupancy per sizes, history, node, direction, end, fights,
    // bundle band, the reach flood's queue; per cell: via occupancy per sizes, via history, fights
    const int64_t states = (int64_t) world.L * world.cells;
    const int64_t K = (int64_t) world.sizes.size();
    result.laneBytes = states * ( 2 * K + 4 + 16 + 1 + 4 + 3 + 4 + 4 ) + (int64_t) world.cells * ( 2 * K + 4 + 3 );

    int64_t budget = aParams.memoryBudget;

    if( budget <= 0 )
    {
        const int64_t available = KOPENAPI_MEMORY::SystemAvailable();
        budget = available > 0 ? available / 2 : std::max<int64_t>( KOPENAPI_MEMORY::SystemTotal() / 4, 1LL << 30 );
    }

    result.lanesMemoryAllows = (int) std::clamp<int64_t>( budget / std::max<int64_t>( result.laneBytes, 1 ), 1, 64 );

    if( strategies.empty() )
    {
        strategies = GridBuiltinStrategies( aParams );
        int count = aParams.variants;

        if( count <= 0 )
            count = std::min( (int) std::thread::hardware_concurrency() / 2, result.lanesMemoryAllows );

        strategies.resize( (size_t) std::clamp( count, 1, (int) strategies.size() ) );
    }

    std::vector<VARIANT> variants;

    if( branchOf )
    {
        const GRID_LANE& from = branchOf->Lane( aParams.branchLane );
        strategies = { from.strategy };
        variants.push_back( { from.name, from.order, from.direction, from.strategy } );
    }

    for( const GRID_STRATEGY& st : branchOf ? std::vector<GRID_STRATEGY>() : strategies )
    {
        VARIANT v;
        v.order = st.order == "long_first" ? longFirst : st.order == "fine_first" ? fineFirst : shortFirst;
        v.direction = st.directions == "hv" ? hv : st.directions == "vh" ? vh : std::vector<int>();
        v.strategy = st;
        v.name = st.Name();

        if( !groupSize.empty() )
            v.order = bundlesFirst( v.order );

        variants.push_back( std::move( v ) );
    }

    // the same order and directions route the same: run once
    for( size_t k = variants.size(); k-- > 1; )
    {
        for( size_t j = 0; j < k; ++j )
        {
            const GRID_STRATEGY& a = variants[j].strategy;
            const GRID_STRATEGY& b = variants[k].strategy;

            if( variants[j].order == variants[k].order && variants[j].direction == variants[k].direction
                && a.pfStart == b.pfStart && a.pfGrowth == b.pfGrowth && a.astar == b.astar && a.schedule == b.schedule
                && a.histDecay == b.histDecay && a.recent == b.recent )
            {
                variants.erase( variants.begin() + (long) k );
                break;
            }
        }
    }

    std::atomic<bool>                        stop{ false };
    std::vector<std::unique_ptr<NEGOTIATION>> runs;

    for( const VARIANT& v : variants )
        runs.push_back( std::make_unique<NEGOTIATION>( world, v, aParams, stop ) );

    if( branchOf )
    {
        const GRID_LANE& from = branchOf->Lane( aParams.branchLane );
        const size_t     k = branchOf->AddLane();
        GRID_LANE&       lane = branchOf->Lane( k );

        lane.ForkFrom( from, aParams.branchStep );

        for( size_t n = 0; n < world.nets.size() && n < branchOf->netCodes.size(); ++n )
            branchOf->netCodes[n] = world.nets[n].code;

        lane.name = from.name + ( aParams.warm ? "/w" : "/b" ) + std::to_string( k );
        lane.parent = aParams.branchLane;
        lane.parentStep = aParams.branchStep;
        lane.running = true;
        variants[0].name = lane.name;

        const auto since = std::chrono::duration_cast<CLOCK::duration>(
                std::chrono::duration<double, std::milli>( from.steps[aParams.branchStep].ms ) );
        runs[0]->Record( *branchOf, lane, routing - since, false );

        if( aParams.warm )
        {
            lane.world = print;

            for( const WORLD::NET& n : world.nets )
                lane.exits.push_back( n.exits );

            // the tiles the edit changed, and one around each
            const std::vector<uint64_t>& now = tiles;
            std::vector<bool>           edited( now.size(), false );

            for( int ty = 0; ty < branchOf->tny; ++ty )
            {
                for( int tx = 0; tx < branchOf->tnx; ++tx )
                {
                    const size_t t = (size_t) ty * branchOf->tnx + tx;

                    if( t >= branchOf->tilePrints.size() || now[t] == branchOf->tilePrints[t] )
                        continue;

                    result.warmEditedTiles++;

                    for( int dy = -1; dy <= 1; ++dy )
                    {
                        for( int dx = -1; dx <= 1; ++dx )
                        {
                            const int x = tx + dx, y = ty + dy;

                            if( x >= 0 && y >= 0 && x < branchOf->tnx && y < branchOf->tny )
                                edited[(size_t) y * branchOf->tnx + x] = true;
                        }
                    }
                }
            }

            runs[0]->WarmStart( from, aParams.branchStep, aParams.warmHistory, edited );
            result.warmRound = runs[0]->FirstRound();

            for( const auto& [n, why] : runs[0]->Affected() )
                result.warmAffected.emplace_back( world.nets[n].code, why );
        }
        else
        {
            runs[0]->Replay( from, aParams.branchStep );
        }

        result.timeline = branchOf;
        result.branchLane = (int) k;
        branchOf->live = true;

        if( aParams.timelineStarted )
            aParams.timelineStarted( branchOf );
    }
    else if( aParams.record || aParams.branchOf )
    {
        auto tl = std::make_shared<GRID_TIMELINE>();
        tl->grid = world.grid;
        tl->cells = world.cells;
        tl->layers = aParams.layers;
        tl->tile = tile;
        tl->tnx = tnx;
        tl->tny = tny;
        tl->lanes.resize( runs.size() );

        for( const WORLD::NET& n : world.nets )
        {
            tl->netCodes.push_back( n.code );
            tl->exits.push_back( n.exits );
        }

        tl->world = print;
        tl->rules = rules;
        tl->nets = netsPrint;
        tl->tilePrints = tiles;

        for( const WORLD::NET& n : world.nets )
        {
            const NETINFO_ITEM* info = aBoard->FindNet( n.code );
            tl->netKeys.push_back( info ? info->GetNetname().ToStdString() : std::string() );
        }

        for( size_t k = 0; k < runs.size(); ++k )
        {
            GRID_LANE& lane = tl->lanes[k];
            lane.name = variants[k].name;
            lane.strategy = variants[k].strategy;
            lane.order = variants[k].order;
            lane.direction = variants[k].direction;
            lane.running = true;
            runs[k]->Record( *tl, lane, routing );
        }

        result.timeline = tl;
        tl->live = true;

        if( aParams.timelineStarted )
            aParams.timelineStarted( tl );
    }

    std::vector<std::future<void>> done;

    for( auto& run : runs )
        done.push_back( std::async( std::launch::async, [r = run.get()]() { r->Run(); } ) );

    // once one variant has joined everything, the variants still negotiating stop: those past
    // their rounds finish (they may still need fewer vias or less length).  Once the first has
    // finished at all, the others get a quarter of its time again (2 s at least)
    std::vector<bool>   finished( runs.size(), false );
    std::vector<double> finishedMs( runs.size(), 0 );
    bool                complete = false;
    double              firstMs = -1;

    for( size_t left = runs.size(); left > 0; )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );

        for( size_t k = 0; k < runs.size(); ++k )
        {
            if( finished[k] || done[k].wait_for( std::chrono::seconds( 0 ) ) != std::future_status::ready )
                continue;

            done[k].get();
            finished[k] = true;
            finishedMs[k] = msSince( routing );
            --left;

            if( firstMs < 0 && !runs[k]->Halted() )
                firstMs = finishedMs[k];

            if( !complete && runs[k]->Complete() )
            {
                complete = true;

                for( size_t j = 0; j < runs.size(); ++j )
                {
                    if( !finished[j] )
                        runs[j]->Abandon();
                }
            }
        }

        if( firstMs >= 0 && left > 0 && msSince( routing ) > firstMs + std::max( 2000.0, firstMs / 4 ) )
        {
            for( size_t j = 0; j < runs.size(); ++j )
            {
                if( !finished[j] )
                    runs[j]->Abandon( true );
            }
        }

        if( aParams.progress && left > 0 )
        {
            std::vector<GRID_VARIANT_PROGRESS> states;

            for( auto& run : runs )
                states.push_back( run->Progress() );

            if( !aParams.progress( states ) )
                stop = true;
        }
    }

    // the best: most ends joined, then fewest vias, then shortest; ties to the earlier variant
    int    best = -1;
    long   bestReached = -1, bestVias = 0;
    double bestLength = 0;

    for( size_t k = 0; k < runs.size(); ++k )
    {
        long   reached = 0, vias = 0;
        double length = 0;

        for( const auto& net : runs[k]->Nets() )
        {
            reached += std::count( net.reached.begin(), net.reached.end(), true );
            vias += (long) net.vias.size();

            for( const auto& path : net.paths )
                length += (double) path.size();
        }

        result.variantsTried.push_back( variants[k].name );
        result.expanded += runs[k]->Expanded();
        result.variantStats.push_back( { variants[k].name, finishedMs[k], runs[k]->ExpandedByStage(), reached, vias,
                                         runs[k]->Abandoned() } );

        if( runs[k]->Abandoned() )
        {
            result.variantsAbandoned.push_back( variants[k].name );
            continue;
        }

        if( reached > bestReached || ( reached == bestReached && ( vias < bestVias || ( vias == bestVias && length < bestLength ) ) ) )
        {
            best = (int) k;
            bestReached = reached;
            bestVias = vias;
            bestLength = length;
        }
    }

    if( branchOf )
    {
        GRID_LANE& lane = branchOf->Lane( result.branchLane );
        lane.running = false;
        lane.stopped = runs[0]->Halted();
        result.restoreMs = runs[0]->RestoreMs();

        if( runs[0]->Diverged() )
        {
            result.error = "the replay of " + branchOf->Lane( aParams.branchLane ).name + " went another way at step "
                           + std::to_string( runs[0]->ReplayedSteps() ) + ": the router differs from the one that recorded it";
            lane.abandoned = true;
            return result;
        }
    }
    else if( result.timeline )
    {
        result.timeline->winner = best;

        for( size_t k = 0; k < runs.size(); ++k )
        {
            result.timeline->lanes[k].abandoned = runs[k]->Abandoned();
            result.timeline->lanes[k].running = false;
            result.timeline->lanes[k].stopped = runs[k]->Halted();
        }
    }

    if( best < 0 )
    {
        result.error = "stopped";
        return result;
    }

    const NEGOTIATION& win = *runs[best];
    result.variant = variants[best].name;

    result.rounds = win.Round();
    result.conflictsLeft = win.Dropped();
    result.crowdedByRound = win.ByRound();
    result.crowdedAtEnd = win.Crowded();

    result.expandedByStage = win.ExpandedByStage();

    for( size_t n = 0; n < world.nets.size(); ++n )
        result.costliest.emplace_back( world.nets[n].code, win.ExpandedBy()[n] );

    std::stable_sort( result.costliest.begin(), result.costliest.end(),
                      []( const auto& a, const auto& b ) { return a.second > b.second; } );
    result.costliest.resize( std::min<size_t>( result.costliest.size(), 10 ) );

    for( size_t n = 0; n < win.Nets().size(); ++n )
    {
        result.nets.push_back( toGeometry( world, aParams, win.Nets()[n], n ) );

        if( n < world.nets.size() )
            result.nets.back().walls = world.nets[n].walls;
    }

    if( !asAsked.empty() )
    {
        std::vector<GRID_NET_RESULT> asked( result.nets.size() );

        for( size_t k = 0; k < asAsked.size(); ++k )
            asked[asAsked[k]] = std::move( result.nets[k] );

        result.nets = std::move( asked );
    }

    result.routeMs = msSince( routing );
    return result;
}


std::vector<GRID_STRATEGY> GridBuiltinStrategies( const GRID_PARAMS& aParams )
{
    std::vector<GRID_STRATEGY> out;

    for( const auto& [o, d, start] : std::vector<std::tuple<const char*, const char*, float>>{ { "short_first", "hv", 1.0f },
                                                                                              { "fine_first", "hv", 1.0f },
                                                                                              { "short_first", "", 1.0f },
                                                                                              { "short_first", "hv", 2.0f },
                                                                                              { "fine_first", "hv", 2.0f },
                                                                                              { "short_first", "hv", 0.75f } } )
    {
        GRID_STRATEGY st;
        st.order = o;
        st.directions = d;
        st.pfStart = std::clamp( aParams.pfStart * start, 0.01f, 100.0f );
        st.pfGrowth = aParams.pfGrowth;
        st.astar = aParams.astar;
        st.schedule = aParams.schedule;
        st.histDecay = aParams.histDecay;
        st.recent = aParams.recent;
        out.push_back( st );
    }

    return out;
}


std::string GRID_STRATEGY::Name() const
{
    auto num = []( float v )
    {
        char buf[16];
        snprintf( buf, sizeof( buf ), "%g", v );
        return std::string( buf );
    };

    std::string n = order + ( directions.empty() ? "" : "_" + directions );

    if( schedule == 0 )
        n += "@" + num( pfStart ) + "x" + num( pfGrowth );
    else
        n += schedule == 1 ? "@adaptive" : "@adaptive_hist";

    if( astar != 1.2f )
        n += "/a" + num( astar );

    if( histDecay != 1.0f )
        n += "/d" + num( histDecay );

    if( recent > 0 )
        n += "/r" + std::to_string( recent );

    return n;
}
