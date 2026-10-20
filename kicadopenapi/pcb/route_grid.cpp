#include "route_grid.h"
#include "route_raster.h"
#include "route_timeline.h"

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
#include <queue>
#include <set>
#include <thread>
#include <tuple>
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

    explicit DISK( double aRadiusCells )
    {
        r = std::max( 0, (int) std::ceil( aRadiusCells ) );
        const double rr = aRadiusCells * aRadiusCells;
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

/// @brief The board as the search sees it: read once, shared by every variant, never written
struct WORLD
{
    PLAN_GRID grid;
    int       L = 0;
    size_t    cells = 0;
    int       h2h = 0;

    std::vector<SIZES>                sizes;
    std::vector<std::vector<int32_t>> track;   ///< per sizes: L * cells; 0 free, net: only that net, -1 none
    std::vector<std::vector<int32_t>> via;     ///< per sizes: cells
    std::vector<uint8_t>              fine;    ///< cells near a fine-pitch package
    std::vector<uint32_t>             keep;    ///< cells around chips (a bit each) where only their own nets' vias go
    std::vector<uint16_t>             away;    ///< cells: distance in cells to the nearest chip pad, up to viaAway; empty none
    std::vector<int32_t>              endOf;   ///< states a net's track there stays inside its end copper: that net

    /// @brief Deep inside its own end's copper a path crowds nobody: the end's copper is there anyway
    bool InOwnEnd( uint32_t aState, int aNet ) const { return endOf[aState] == aNet; }

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
        uint32_t                           chips = 0; ///< the chips (keep bits) it has a pad on
        int                                group = -1;
        int                                rank = 0;
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

        auto open = [&]( uint32_t s ) { return aWorld.TrackOk( k, s, aNet.code ) || aWorld.InOwnEnd( s, aNet.code ); };

        for( size_t e = 0; e < aNet.ends.size(); ++e )
        {
            if( aNet.ends[e].size() > POCKET_STATES )
                continue;

            std::unordered_set<uint32_t> seen( aNet.ends[e].begin(), aNet.ends[e].end() );
            std::vector<uint32_t>        todo( aNet.ends[e].begin(), aNet.ends[e].end() );

            while( !todo.empty() && seen.size() <= POCKET_STATES )
            {
                const uint32_t s = todo.back();
                todo.pop_back();
                const int l = (int) ( s / cells ), i = (int) ( s % cells );
                const int x = i % g.nx, y = i / g.nx;

                auto visit = [&]( uint32_t n )
                {
                    if( open( n ) && seen.insert( n ).second )
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

                if( aNet.vias && aWorld.L > 1 && aWorld.ViaOk( k, (uint32_t) i, aNet.code ) )
                {
                    for( int to = 0; to < aWorld.L; ++to )
                        visit( (uint32_t) ( (size_t) to * cells + i ) );
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

    // a cell of a sixth of the finest pitch of track and clearance, on a 0.025 mm step
    const int step = pcbIUScale.mmToIU( 0.025 );
    int       cell = aParams.cell;

    if( cell <= 0 )
    {
        int finest = INT_MAX;

        for( const SIZES& s : aWorld.sizes )
            finest = std::min( finest, 2 * s.halfWidth + s.clearance );

        cell = std::clamp( finest / 6 / step * step, step, 4 * step );
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
    struct MARK
    {
        std::shared_ptr<const SHAPE> shape;
        int                          inflate;
        int32_t                      owner;
    };

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
        probeTrack.SetNetCode( sample[k] );
        probeTrack.SetWidth( 2 * sz.halfWidth );

        PCB_VIA probeVia( aBoard );
        probeVia.SetNetCode( sample[k] );
        probeVia.SetViaType( VIATYPE::THROUGH );
        probeVia.SetLayerPair( F_Cu, B_Cu );
        probeVia.SetWidth( PADSTACK::ALL_LAYERS, std::max( 2 * sz.viaRadius, 1 ) );
        probeVia.SetDrill( std::max( sz.viaDrill, 1 ) );

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
                    mark( s, rule( CLEARANCE_CONSTRAINT, &probeVia, aItem, layer, floor ) + sz.viaRadius + slack, L,
                          aViaHard ? -1 : aOwner );
                }
            }
        };

        auto hole = [&]( BOARD_ITEM* aItem, std::shared_ptr<const SHAPE> aHole, bool aNpth )
        {
            if( aNpth )
            {
                for( int l = 0; l < L; ++l )
                {
                    probeTrack.SetLayer( aParams.layers[l] );
                    mark( aHole, rule( HOLE_CLEARANCE_CONSTRAINT, &probeTrack, aItem, aParams.layers[l], ds.m_HoleClearance )
                                         + sz.halfWidth + slack,
                          l, -1 );
                }
            }

            if( vias )
                mark( aHole, rule( HOLE_TO_HOLE_CONSTRAINT, &probeVia, aItem, F_Cu, aWorld.h2h ) + sz.viaDrill / 2 + slack,
                      L, -1 );
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
                    hole( pad, pad->GetEffectiveHoleShape(), pad->GetAttribute() == PAD_ATTRIB::NPTH );
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

            if( t->Type() == PCB_VIA_T && vias )
            {
                std::shared_ptr<SHAPE_SEGMENT> h = static_cast<PCB_VIA*>( t )->GetEffectiveHoleShape();
                mark( h, aWorld.h2h + sz.viaDrill / 2 + slack, L, -1 );
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

    // outside the outline
    SHAPE_POLY_SET outline;

    if( aBoard->GetBoardPolygonOutlines( outline, true ) && outline.OutlineCount() > 0 )
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
                if( a != b )
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
            m_dead.emplace_back( n.ends.size(), false );

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
                const double viaVia = std::max( x.viaRadius + y.viaRadius + clr, ( x.viaDrill + y.viaDrill ) / 2 + m_w.h2h );

                // a's track / via onto b's track map and b's via map
                m_trackDisk[a].push_back( { DISK( ( x.halfWidth + clr + y.halfWidth ) / c ),
                                            DISK( ( x.viaRadius + clr + y.halfWidth ) / c ) } );
                m_viaDisk[a].push_back( { DISK( ( x.halfWidth + clr + y.viaRadius ) / c ), DISK( viaVia / c ) } );
            }
        }

        m_viaCost = 10.0f * (float) ( pcbIUScale.mmToIU( 2.0 ) / c );
        m_fineCost = 10.0f * (float) ( pcbIUScale.mmToIU( 3.0 ) / c );
        m_awayCells = (float) ( m_params.viaAway / c );
        m_awayCost = 10.0f * (float) ( m_params.viaAwayCost / c );

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
        float pf = m_v.strategy.pfStart;

        for( m_round = 1; m_round <= m_params.maxRounds && !stopped(); ++m_round )
        {
            // the crowded nets, and the nets through places fought over before: a net that crowds
            // nobody may still have to make room
            std::vector<int> todo;

            for( int n : m_v.order )
            {
                if( m_round == 1 || conflicts( n ) > 0
                    || ( m_v.strategy.recent > 0 ? throughRecentHistory( n ) : throughHistory( n ) ) )
                    todo.push_back( n );
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

            if( m_round > STALL_AFTER && r > STALL_ROUNDS && *std::min_element( m_byRound.end() - STALL_ROUNDS, m_byRound.end() )
                                            >= *std::min_element( m_byRound.begin(), m_byRound.end() - STALL_ROUNDS ) )
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

            // where the net's paths branch off one another: kept on the copper
            std::unordered_set<uint32_t> branches;

            for( const auto& path : m_nets[n].paths )
            {
                if( !path.empty() )
                    branches.insert( path.front() );
            }

            for( auto& path : m_nets[n].paths )
                pullTight( n, path, branches );

            markNet( n );
        }

        for( size_t k = m_byStage.size() - 1; k > 0; --k )
            m_byStage[k] -= m_byStage[k - 1];

        at( STAGE_DONE, (int) m_v.order.size(), (int) m_v.order.size() );
    }

    /// @brief Keep the steps in aLane (before Run)
    void Record( const GRID_TIMELINE& aTimeline, GRID_LANE& aLane, CLOCK::time_point aStarted )
    {
        m_timeline = &aTimeline;
        m_lane = &aLane;
        m_started = aStarted;
        aLane.Start( m_nets.size() );
    }

    /// @brief Another variant has joined everything while this one still negotiates: it stops
    void Abandon()
    {
        if( m_stage == STAGE_ROUNDS )
            m_abandoned = true;
    }

    bool Abandoned() const { return m_abandoned; }

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
        p.round = std::min<int>( m_round, m_params.maxRounds );
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
    long                          Expanded() const { return m_expanded; }
    const std::vector<long>&      ExpandedBy() const { return m_expandedBy; }
    const std::vector<long>&      ExpandedByStage() const { return m_byStage; }
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

    /// @brief The net was taken (routed, kept, pulled tight or dropped): one step
    void record( int aNet, int aStage )
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

        m_lane->histByRound.push_back( std::move( m_histTaken ) );
        m_histTaken.clear();

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

        m_lane->heatByRound.push_back( std::move( heat ) );
    }

    bool stopped() const
    {
        return m_stop.load( std::memory_order_relaxed ) || m_abandoned.load( std::memory_order_relaxed )
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

        auto free = [&]( uint32_t s )
        {
            return m_w.InOwnEnd( s, wn.code ) || ( m_w.TrackOk( k, s, wn.code ) && m_occTrack[k][s] == 0 );
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
        const long before = m_expanded;
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
            if( e != first && !wn.ends[e].empty() && !wn.walled[e] && !m_dead[aNet][e] )
                left.push_back( (int) e );
        }

        while( !left.empty() && !stopped() )
        {
            std::vector<uint32_t> path;
            const int             end = search( aNet, tree, left, aPf, aHard, path );

            if( end < 0 )
            {
                // nothing but fixed copper in the way: no round will reach them
                if( m_walledIn )
                {
                    for( int e : left )
                        m_dead[aNet][e] = true;
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
        int                   bestEnd = -1;
        float                 bestCost = std::numeric_limits<float>::max();
        std::vector<uint32_t> best;
        bool                  widened = false;
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

            // a path through vias may only be the window's: a detour around it is tried once
            bool through = false;

            for( size_t k = 1; end >= 0 && k < aPath.size() && !through; ++k )
                through = aPath[k] % cells == aPath[k - 1] % cells;

            if( ( end >= 0 && ( !through || widened ) ) || whole || m_oneWindow || stopped() )
            {
                if( bestEnd >= 0 )
                {
                    aPath = std::move( best );
                    return bestEnd;
                }

                m_walledIn = whole && !aHard && !m_overBudget && !stopped();
                return -1;
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

        return m_guides.emplace( std::move( key ), std::move( out ) ).first->second;
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

        auto h = [&]( int x, int y, int l )
        {
            int best = INT_MAX;

            for( int e : aLeft )
            {
                const auto& b = wn.endBox[e];
                const int   dx = std::max( { b[0] - x, 0, x - b[2] } );
                const int   dy = std::max( { b[1] - y, 0, y - b[3] } );
                best = std::min( best, 10 * std::max( dx, dy ) + 4 * std::min( dx, dy ) );
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

        const long budget = 4L * ( x1 - x0 + 1 ) * ( y1 - y0 + 1 ) * L;
        long       taken = 0;
        m_overBudget = false;

        auto cellCost = [&]( uint32_t s ) -> float
        {
            if( m_w.InOwnEnd( s, wn.code ) )
                return 1.0f;

            const uint16_t occ = m_occTrack[k][s];
            const float    hug = !occ && wn.group >= 0 && !m_band.empty() && m_band[s] && m_bandGroup[s] == wn.group + 1
                                         ? m_params.hug
                                         : 1.0f;

            if( aHard )
                return occ ? -1.0f : hug;

            return ( 1.0f + m_hist[s] ) * ( 1.0f + aPf * occ ) * hug;
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

            auto relax = [&]( uint32_t n, float cost, int nd )
            {
                const float ng = top.g + cost;

                if( m_node[n].seen == stamp && ng >= m_node[n].g )
                    return;

                m_node[n].seen = stamp;
                m_node[n].g = ng;
                m_node[n].parent = (int32_t) s;
                m_dir[n] = (uint8_t) nd;
                const int ni = (int) ( n % cells );
                open.push( { ng + h( ni % g.nx, ni / g.nx, (int) ( n / cells ) ), ng, n } );
            };

            for( int d = 0; d < 8; ++d )
            {
                const int nx = x + DX[d], ny = y + DY[d];

                if( nx < x0 || nx > x1 || ny < y0 || ny > y1 )
                    continue;

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

                relax( n, step * c + ( turn == 1 ? 3.0f : turn == 2 ? 8.0f : 0.0f ), d );
            }

            if( wn.vias && L > 1 && m_w.ViaOk( k, (uint32_t) i, wn.code ) && !( m_keepOut && ( m_w.keep[i] & ~wn.chips ) ) )
            {
                const uint16_t occ = m_occVia[k][i];

                if( aHard && occ )
                    continue;

                const float base = m_viaCost * m_viaScale * ( aHard ? 1.0f : ( 1.0f + m_histVia[i] ) * ( 1.0f + aPf * occ ) )
                                   + ( m_w.fine[i] ? m_fineCost : 0.0f )
                                   + ( !m_w.away.empty() && m_w.away[i] < m_awayCells ? m_awayCost * ( 1.0f - m_w.away[i] / m_awayCells ) : 0.0f );

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

                    relax( n, base * c, NO_DIR );
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

    std::vector<NODE>     m_node;
    std::vector<uint8_t>  m_dir;
    std::vector<int>      m_targetEnd;
    uint32_t              m_stamp = 0;
    bool                  m_overBudget = false;
    float                 m_lastCost = 0;
    bool                  m_walledIn = false;   ///< the last search failed over the whole board on fixed copper alone

    std::vector<std::vector<bool>> m_dead;   ///< per net, per end: walled in by fixed copper

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

    std::vector<uint16_t> m_band;        ///< states beside bundle copper: how many members' copper
    std::vector<uint16_t> m_bandGroup;   ///< and of which bundle (+1)
    std::vector<DISK>     m_bandDisk;    ///< per sizes: a member's track and the place beside it

    std::vector<NET_STATE> m_nets;
    std::atomic<int>       m_round{ 0 };
    std::atomic<int>       m_conflicting{ 0 };
    std::atomic<int>       m_stage{ STAGE_ROUNDS };
    std::atomic<bool>      m_abandoned{ false };
    std::atomic<int>       m_taken{ 0 };
    std::atomic<int>       m_takenOf{ 0 };
    std::atomic<long>      m_expandedShown{ 0 };
    int                    m_dropped = 0;
    std::vector<int>       m_byRound;   ///< nets still crowded after each round
    std::vector<long>      m_expandedBy;   ///< search states per net, all rounds
    std::map<std::vector<int>, std::vector<uint16_t>> m_guides;   ///< per end set, via scale, net
    const std::vector<uint16_t>*                      m_guide = nullptr;
    std::vector<long>      m_byStage;      ///< search states: rounds, legalising, the short again, fewer vias
    std::vector<CROWDING>  m_crowded;   ///< nets still crowded when the rounds ended
    long                   m_expanded = 0;

    const GRID_TIMELINE*   m_timeline = nullptr;
    GRID_LANE*             m_lane = nullptr;
    CLOCK::time_point      m_started;
    std::array<int, 4>     m_box{ -1, -1, -1, -1 };   ///< cells searched in since the net was taken
    std::vector<uint32_t>  m_histTaken;   ///< this round's history increments
    std::array<int, 4>     m_changed{ INT_MAX, INT_MAX, INT_MIN, INT_MIN };   ///< tiles changed since the last step
    std::vector<std::vector<uint32_t>> m_netConflicts;   ///< per net: the tiles of its crowded states
    std::vector<int>       m_conflictNets;   ///< the nets crowded at the last step
    std::vector<uint16_t>  m_fought;      ///< per state: the last round it was fought over
    std::vector<uint8_t>   m_streak;      ///< and in how many rounds running
};


GRID_NET_RESULT toGeometry( const WORLD& aWorld, const GRID_PARAMS& aParams, const NEGOTIATION::NET_STATE& aNet )
{
    return GridPathGeometry( aWorld.grid, aWorld.cells, aParams.layers, aNet.paths, aNet.reached );
}

} // namespace


GRID_RESULT RouteOnGrid( BOARD* aBoard, const std::vector<GRID_NET>& aNets, const GRID_PARAMS& aParams )
{
    GRID_RESULT result;
    const auto  started = CLOCK::now();

    if( aNets.empty() || aParams.layers.empty() )
        return result;

    WORLD world;
    buildWorld( aBoard, aNets, aParams, world );
    findPockets( world );
    buildTiles( world );
    result.buildMs = msSince( started );
    result.cell = world.grid.cell;

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
    // bundle band; per cell: via occupancy per sizes, via history, fights
    const int64_t states = (int64_t) world.L * world.cells;
    const int64_t K = (int64_t) world.sizes.size();
    result.laneBytes = states * ( 2 * K + 4 + 16 + 1 + 4 + 3 + 4 ) + (int64_t) world.cells * ( 2 * K + 4 + 3 );

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

    for( const GRID_STRATEGY& st : strategies )
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

    if( aParams.record )
    {
        auto tl = std::make_shared<GRID_TIMELINE>();
        tl->grid = world.grid;
        tl->cells = world.cells;
        tl->layers = aParams.layers;
        tl->tile = std::max( 1, (int) std::lround( pcbIUScale.mmToIU( 1.0 ) / (double) world.grid.cell ) );
        tl->tnx = ( world.grid.nx + tl->tile - 1 ) / tl->tile;
        tl->tny = ( world.grid.ny + tl->tile - 1 ) / tl->tile;
        tl->lanes.resize( runs.size() );

        for( const WORLD::NET& n : world.nets )
            tl->netCodes.push_back( n.code );

        for( size_t k = 0; k < runs.size(); ++k )
        {
            tl->lanes[k].name = variants[k].name;
            runs[k]->Record( *tl, tl->lanes[k], routing );
        }

        result.timeline = tl;
    }

    std::vector<std::future<void>> done;

    for( auto& run : runs )
        done.push_back( std::async( std::launch::async, [r = run.get()]() { r->Run(); } ) );

    // once one variant has joined everything, the variants still negotiating stop: those past
    // their rounds finish (they may still need fewer vias or less length)
    std::vector<bool>   finished( runs.size(), false );
    std::vector<double> finishedMs( runs.size(), 0 );
    bool                complete = false;

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

    const NEGOTIATION& win = *runs[best];
    result.variant = variants[best].name;

    if( result.timeline )
    {
        result.timeline->winner = best;

        for( size_t k = 0; k < runs.size(); ++k )
            result.timeline->lanes[k].abandoned = runs[k]->Abandoned();
    }
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

    for( const auto& net : win.Nets() )
        result.nets.push_back( toGeometry( world, aParams, net ) );

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
