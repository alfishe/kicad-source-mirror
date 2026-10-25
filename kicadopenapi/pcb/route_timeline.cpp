#include "route_timeline.h"

#include <kicadopenapi_registry.h>

#include <algorithm>


void GRID_LANE::Start( size_t aNets )
{
    m_table.assign( aNets, GRID_STEP::NONE );
    conflictSets.assign( 1, {} );
}


uint32_t GRID_LANE::AddConflicts( const std::vector<uint32_t>& aSet )
{
    std::lock_guard<std::recursive_mutex> lock( *m_mutex );
    if( aSet.empty() )
        return 0;

    uint64_t h = 1469598103934665603ull;

    for( uint32_t v : aSet )
    {
        h ^= v;
        h *= 1099511628211ull;
    }

    std::vector<uint32_t>& same = m_conflictsByHash[h];

    for( uint32_t id : same )
    {
        if( conflictSets[id] == aSet )
            return id;
    }

    same.push_back( (uint32_t) conflictSets.size() );
    conflictSets.push_back( aSet );
    return same.back();
}


void GRID_LANE::Add( GRID_STEP aStep, const GRID_PATH* aPath )
{
    struct NOTIFY
    {
        ~NOTIFY() { KopenapiEventNotify( "pcb_route_timeline" ); }
    } notify;

    std::lock_guard<std::recursive_mutex> lock( *m_mutex );
    if( steps.size() % KEY == 0 )
        m_keys.push_back( m_table );

    if( aPath && !aPath->paths.empty() )
    {
        // FNV-1a over the states and the path breaks
        uint64_t h = 1469598103934665603ull;

        auto mix = [&]( uint32_t v )
        {
            h ^= v;
            h *= 1099511628211ull;
        };

        for( const auto& path : aPath->paths )
        {
            mix( UINT32_MAX );

            for( uint32_t s : path )
                mix( s );
        }

        for( bool r : aPath->reached )
            mix( r );

        std::vector<uint32_t>& same = m_byHash[h];
        aStep.path = GRID_STEP::NONE;

        for( uint32_t id : same )
        {
            if( paths[id].paths == aPath->paths && paths[id].reached == aPath->reached )
            {
                aStep.path = id;
                break;
            }
        }

        if( aStep.path == GRID_STEP::NONE )
        {
            aStep.path = (uint32_t) paths.size();
            paths.push_back( *aPath );
            same.push_back( aStep.path );
        }
    }
    else
    {
        aStep.path = GRID_STEP::NONE;
    }

    m_table[aStep.net] = aStep.path;
    steps.push_back( aStep );
}


void GRID_LANE::AddDead( int aNet, int aEnd, bool aKeepOut )
{
    std::lock_guard<std::recursive_mutex> lock( *m_mutex );
    dead.push_back( { (uint32_t) steps.size(), aNet, aEnd, aKeepOut } );
}


void GRID_LANE::ForkFrom( const GRID_LANE& aParent, size_t aStep )
{
    std::lock_guard<std::recursive_mutex> lock( aParent.Mutex() );

    if( aParent.steps.empty() )
        return;

    aStep = std::min( aStep, aParent.steps.size() - 1 );

    strategy = aParent.strategy;
    order = aParent.order;
    direction = aParent.direction;
    world = aParent.world;
    exits = aParent.exits;
    steps.assign( aParent.steps.begin(), aParent.steps.begin() + (long) aStep + 1 );

    // ids are given in the order first seen: the prefix uses the first ones only
    uint32_t pathsUsed = 0, setsUsed = 1;

    for( const GRID_STEP& st : steps )
    {
        if( st.path != GRID_STEP::NONE )
            pathsUsed = std::max( pathsUsed, st.path + 1 );

        setsUsed = std::max( setsUsed, st.conflicts + 1 );
    }

    paths.assign( aParent.paths.begin(), aParent.paths.begin() + pathsUsed );
    conflictSets.assign( aParent.conflictSets.begin(), aParent.conflictSets.begin() + setsUsed );

    auto keep = []( const std::unordered_map<uint64_t, std::vector<uint32_t>>& aFrom, uint32_t aBelow,
                    std::unordered_map<uint64_t, std::vector<uint32_t>>& aTo )
    {
        aTo.clear();

        for( const auto& [h, ids] : aFrom )
        {
            for( uint32_t id : ids )
            {
                if( id < aBelow )
                    aTo[h].push_back( id );
            }
        }
    };

    keep( aParent.m_byHash, pathsUsed, m_byHash );
    keep( aParent.m_conflictsByHash, setsUsed, m_conflictsByHash );

    m_keys.assign( aParent.m_keys.begin(), aParent.m_keys.begin() + (long) ( aStep / KEY + 1 ) );
    aParent.TableAt( aStep, m_table );

    // the rounds finished before the step; a round 0 step (none finished) or a lane that kept fewer
    // rounds than its step's number (a warm lane numbers on from its parent's) takes what there is
    const GRID_STEP& last = steps.back();
    const size_t     rounds = last.stage != 0 ? aParent.histByRound.size() : last.round > 0 ? (size_t) last.round - 1 : 0;

    histByRound.assign( aParent.histByRound.begin(),
                        aParent.histByRound.begin() + (long) std::min( rounds, aParent.histByRound.size() ) );
    heatByRound.assign( aParent.heatByRound.begin(),
                        aParent.heatByRound.begin() + (long) std::min( rounds, aParent.heatByRound.size() ) );

    dead.clear();

    for( const DEAD& d : aParent.dead )
    {
        if( d.step <= aStep )
            dead.push_back( d );
    }
}


void GRID_LANE::TableAt( size_t aStep, std::vector<uint32_t>& aTable ) const
{
    if( m_keys.empty() )
    {
        aTable = m_table;
        return;
    }

    if( aStep == SIZE_MAX )
    {
        aTable = m_keys.front();
        return;
    }

    aStep = std::min( aStep, steps.size() - 1 );
    aTable = m_keys[aStep / KEY];

    for( size_t k = aStep / KEY * KEY; k <= aStep; ++k )
        aTable[steps[k].net] = steps[k].path;
}


size_t GRID_LANE::Bytes() const
{
    std::lock_guard<std::recursive_mutex> lock( *m_mutex );
    size_t b = steps.capacity() * sizeof( GRID_STEP ) + m_table.capacity() * 4;

    for( const GRID_PATH& p : paths )
    {
        b += sizeof( GRID_PATH ) + p.reached.size() / 8;

        for( const auto& path : p.paths )
            b += path.capacity() * 4 + sizeof( path );
    }

    for( const auto& k : m_keys )
        b += k.capacity() * 4;

    for( const auto& r : histByRound )
        b += r.capacity() * 4;

    for( const auto& r : heatByRound )
        b += r.capacity() * 2;

    for( const auto& c : conflictSets )
        b += c.capacity() * 4 + sizeof( c );

    for( const auto& e : exits )
        b += e.size() * ( sizeof( GRID_EXIT ) + 2 * sizeof( VECTOR2I ) ) + sizeof( e );

    return b + m_byHash.size() * 48;
}


void GRID_LANE::Release()
{
    std::lock_guard<std::recursive_mutex> lock( *m_mutex );
    deleted = true;
    std::vector<GRID_STEP>().swap( steps );
    std::vector<GRID_PATH>().swap( paths );
    std::vector<DEAD>().swap( dead );
    std::vector<std::vector<uint32_t>>().swap( histByRound );
    std::vector<std::vector<uint16_t>>().swap( heatByRound );
    std::vector<std::vector<uint32_t>>().swap( conflictSets );
    std::vector<int>().swap( order );
    std::vector<int>().swap( direction );
    std::vector<std::vector<GRID_EXIT>>().swap( exits );
    std::vector<uint32_t>().swap( m_table );
    std::vector<std::vector<uint32_t>>().swap( m_keys );
    std::unordered_map<uint64_t, std::vector<uint32_t>>().swap( m_byHash );
    std::unordered_map<uint64_t, std::vector<uint32_t>>().swap( m_conflictsByHash );
}


GRID_NET_RESULT GRID_TIMELINE::Geometry( const GRID_LANE& aLane, int aNet, const GRID_PATH& aPath ) const
{
    GRID_NET_RESULT out = GridPathGeometry( grid, cells, layers, aPath.paths, aPath.reached );
    const std::vector<std::vector<GRID_EXIT>>& byNet = aLane.world ? aLane.exits : exits;

    if( aNet >= 0 && (size_t) aNet < byNet.size() )
        GridAddExits( out, byNet[aNet], aPath.paths );

    return out;
}


void GridAddExits( GRID_NET_RESULT& aOut, const std::vector<GRID_EXIT>& aExits,
                   const std::vector<std::vector<uint32_t>>& aPaths )
{
    for( const GRID_EXIT& exit : aExits )
    {
        const bool used = std::any_of( aPaths.begin(), aPaths.end(),
                                       [&]( const std::vector<uint32_t>& p )
                                       {
                                           return std::find( p.begin(), p.end(), exit.state ) != p.end();
                                       } );

        if( !used )
            continue;

        for( size_t k = 1; k < exit.track.points.size(); ++k )
            aOut.length += ( exit.track.points[k] - exit.track.points[k - 1] ).EuclideanNorm();

        aOut.tracks.push_back( exit.track );
        aOut.halfCellExits.push_back( exit.end );
    }
}


int GRID_TIMELINE::HeatRound( const GRID_LANE& aLane, size_t aStep ) const
{
    const GRID_STEP& st = aLane.steps[aStep];
    return st.stage == 0 ? st.round - 1 : (int) aLane.heatByRound.size();
}


std::vector<uint8_t> GRID_TIMELINE::FightAge( const GRID_LANE& aLane, int aRound ) const
{
    const size_t         last = std::min<size_t>( std::max( aRound, 0 ), aLane.histByRound.size() );
    std::vector<uint8_t> age( (size_t) tnx * tny, 255 );

    for( size_t r = 0; r < last; ++r )
    {
        const uint8_t a = (uint8_t) std::min<size_t>( last - 1 - r, 254 );

        for( uint32_t v : aLane.histByRound[r] )
        {
            const uint32_t cell = v & GRID_LANE::VIA ? v & ~GRID_LANE::VIA : (uint32_t) ( v % cells );
            const int      x = (int) ( cell % grid.nx ) / tile, y = (int) ( cell / grid.nx ) / tile;
            age[(size_t) y * tnx + x] = a;
        }
    }

    return age;
}


VECTOR2I GRID_TIMELINE::TileCentre( int aTile ) const
{
    const int x = aTile % tnx, y = aTile / tnx;
    const VECTOR2I a = grid.Centre( x * tile, y * tile );
    return a + VECTOR2I( ( tile - 1 ) * grid.cell / 2, ( tile - 1 ) * grid.cell / 2 );
}


size_t GRID_TIMELINE::Bytes() const
{
    std::lock_guard<std::mutex> lock( m_lanesMutex );
    size_t                      b = 0;

    for( const GRID_LANE& lane : lanes )
        b += lane.Bytes();

    return b;
}


size_t GRID_TIMELINE::LaneCount() const
{
    std::lock_guard<std::mutex> lock( m_lanesMutex );
    return lanes.size();
}


const GRID_LANE& GRID_TIMELINE::Lane( size_t aLane ) const
{
    std::lock_guard<std::mutex> lock( m_lanesMutex );
    return lanes[aLane];
}


GRID_LANE& GRID_TIMELINE::Lane( size_t aLane )
{
    std::lock_guard<std::mutex> lock( m_lanesMutex );
    return lanes[aLane];
}


int GRID_TIMELINE::DefaultLane() const
{
    const int shown = shownLane;

    if( shown >= 0 && shown < (int) LaneCount() && !Lane( shown ).deleted )
        return shown;

    return live ? 0 : std::max( winner, 0 );
}


size_t GRID_TIMELINE::AddLane()
{
    std::lock_guard<std::mutex> lock( m_lanesMutex );
    lanes.emplace_back();
    return lanes.size() - 1;
}


GRID_NET_RESULT GridPathGeometry( const PLAN_GRID& aGrid, size_t aCells, const std::vector<PCB_LAYER_ID>& aLayers,
                                  const std::vector<std::vector<uint32_t>>& aPaths, const std::vector<bool>& aReached )
{
    GRID_NET_RESULT out;
    out.reached = aReached;
    out.complete = std::all_of( aReached.begin(), aReached.end(), []( bool r ) { return r; } );

    for( const auto& path : aPaths )
    {
        GRID_TRACK run;

        auto flush = [&]()
        {
            if( run.points.size() >= 2 )
            {
                for( size_t k = 1; k < run.points.size(); ++k )
                    out.length += ( run.points[k] - run.points[k - 1] ).EuclideanNorm();

                out.tracks.push_back( run );
            }

            run.points.clear();
        };

        for( size_t k = 0; k < path.size(); ++k )
        {
            const int      l = (int) ( path[k] / aCells );
            const VECTOR2I p = aGrid.Centre( (int) ( path[k] % aCells ) );

            if( k > 0 && l != (int) ( path[k - 1] / aCells ) )
            {
                flush();
                out.vias.push_back( p );
            }

            run.layer = aLayers[l];

            // a corner only where the direction changes
            if( run.points.size() >= 2 )
            {
                const VECTOR2I a = run.points[run.points.size() - 2], b = run.points.back();
                const VECTOR2I d1 = b - a, d2 = p - b;

                if( (int64_t) d1.x * d2.y == (int64_t) d1.y * d2.x && (int64_t) d1.x * d2.x + (int64_t) d1.y * d2.y > 0 )
                {
                    run.points.back() = p;
                    continue;
                }
            }

            run.points.push_back( p );
        }

        flush();
    }

    return out;
}
