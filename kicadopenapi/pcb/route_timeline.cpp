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

    return b + m_byHash.size() * 48;
}


GRID_NET_RESULT GRID_TIMELINE::Geometry( const GRID_PATH& aPath ) const
{
    return GridPathGeometry( grid, cells, layers, aPath.paths, aPath.reached );
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
    size_t b = 0;

    for( const GRID_LANE& lane : lanes )
        b += lane.Bytes();

    return b;
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
