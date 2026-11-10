#include "part_fanout.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>


PART_FAN_ASSIGNMENT PartFanAssign( const std::vector<PART_FAN_SLOT>& aSlots, int64_t aLo, int64_t aHi, int aStep )
{
    PART_FAN_ASSIGNMENT out;
    const size_t        n = aSlots.size();
    out.at.assign( n, std::nullopt );

    if( n == 0 )
        return out;

    const int64_t step = std::max( aStep, 1 );
    const size_t  cells = aHi < aLo ? 0 : (size_t) ( ( aHi - aLo ) / step ) + 1;
    const int64_t inf = std::numeric_limits<int64_t>::max() / 4;
    // one slot left out costs more than every slot moved across the whole line
    const int64_t miss = ( std::max<int64_t>( aHi - aLo, 0 ) + 1 ) * (int64_t) ( n + 1 );

    auto pos = [&]( size_t j ) { return aLo + (int64_t) j * step; };
    auto open = [&]( size_t k ) { return aSlots[k].laneHalf > 0; };

    // A slot's states: placed at a cell, placed on its pin (a fan's slot that does not move), or left out on its
    // lane (its pin).  Every slot that keeps a way out is in the chain; the others may be skipped
    enum STATE { CELL, STAY, LANE };

    auto halfIn = [&]( size_t k, STATE aState ) { return aState == LANE ? aSlots[k].laneHalf : aSlots[k].half; };

    auto gap = [&]( size_t i, STATE iState, size_t k, STATE kState ) -> int64_t
    {
        const PART_FAN_SLOT& p = aSlots[i];
        const PART_FAN_SLOT& s = aSlots[k];
        const int            clearance = std::max( p.clearance, s.clearance );
        int64_t              g = std::max( (int64_t) halfIn( i, iState ) + halfIn( k, kState ) + clearance, s.pin - p.pin );

        // parallel 45 degree legs: their tracks and clearance across, sqrt 2 times that along the line
        if( iState == CELL && kState == CELL && p.way != 0 && p.way == s.way )
            g = std::max( g, (int64_t) std::ceil( std::sqrt( 2.0 ) * ( p.laneHalf + s.laneHalf + clearance ) ) );

        return g;
    };

    auto fits = [&]( size_t k, int64_t x )
    {
        const PART_FAN_SLOT& s = aSlots[k];
        const int64_t        move = x - s.pin;
        const int64_t        down = std::max( s.reach, s.way < 0 ? s.fanReach : 0 );
        const int64_t        up = std::max( s.reach, s.way > 0 ? s.fanReach : 0 );

        return move >= -down && move <= up
               && !std::any_of( s.blocked.begin(), s.blocked.end(), [&]( const auto& b ) { return x > b.first && x < b.second; } );
    };

    // best[k][j]: the least cost of slots 0..k with k at cell j; low[k][j] the least of best[k][0..j] and where;
    // stay[k], lane[k]: with k on its pin placed / left out
    std::vector<std::vector<int64_t>> best( n, std::vector<int64_t>( cells, inf ) );
    std::vector<std::vector<int64_t>> low( n, std::vector<int64_t>( cells, inf ) );
    std::vector<std::vector<int>>     lowAt( n, std::vector<int>( cells, -1 ) );
    std::vector<std::vector<int>>     fromSlot( n, std::vector<int>( cells, -1 ) );
    std::vector<std::vector<int>>     fromCell( n, std::vector<int>( cells, -1 ) );
    std::vector<int64_t>              stay( n, inf ), lane( n, inf );
    std::vector<int>                  stayFromSlot( n, -1 ), stayFromCell( n, -1 ), laneFromSlot( n, -1 ), laneFromCell( n, -1 );
    constexpr int                     onLane = -2, onStay = -3;

    // the least cost to come to slot k at x in aState, from the slots before it
    auto arrive = [&]( size_t k, int64_t x, STATE aState, int& aSlot, int& aCell )
    {
        int64_t cost = inf;
        aSlot = aCell = -1;

        auto offer = [&]( int64_t aCost, size_t i, int aFrom )
        {
            if( aCost < cost )
            {
                cost = aCost;
                aSlot = (int) i;
                aCell = aFrom;
            }
        };

        for( size_t i = k; i-- > 0; )
        {
            const int64_t skipped = miss * (int64_t) ( k - i - 1 );

            if( cells > 0 )
            {
                const int64_t lastX = x - gap( i, CELL, k, aState );

                if( lastX >= aLo )
                {
                    const size_t jj = std::min( (size_t) ( ( lastX - aLo ) / step ), cells - 1 );

                    if( low[i][jj] < inf )
                        offer( low[i][jj] + skipped, i, lowAt[i][jj] );
                }
            }

            if( stay[i] < inf && x - aSlots[i].pin >= gap( i, STAY, k, aState ) )
                offer( stay[i] + skipped, i, onStay );

            // two lanes are the pins themselves
            if( lane[i] < inf && ( aState == LANE || x - aSlots[i].pin >= gap( i, LANE, k, aState ) ) )
                offer( lane[i] + skipped, i, onLane );

            if( open( i ) )
                break;
        }

        // the first in the chain: every one before it skipped
        bool allSkipped = true;

        for( size_t i = 0; i < k && allSkipped; ++i )
            allSkipped = !open( i );

        if( allSkipped && miss * (int64_t) k < cost )
        {
            cost = miss * (int64_t) k;
            aSlot = aCell = -1;
        }

        return cost;
    };

    for( size_t k = 0; k < n; ++k )
    {
        const PART_FAN_SLOT& s = aSlots[k];

        for( size_t j = 0; j < cells; ++j )
        {
            const int64_t x = pos( j );

            if( !fits( k, x ) )
                continue;

            int           fs, fc;
            const int64_t cost = arrive( k, x, CELL, fs, fc );

            if( cost >= inf )
                continue;

            best[k][j] = cost + std::abs( x - s.pin );
            fromSlot[k][j] = fs;
            fromCell[k][j] = fc;
        }

        if( s.way != 0 && s.pin >= aLo && s.pin <= aHi && fits( k, s.pin ) )
        {
            int           fs, fc;
            const int64_t cost = arrive( k, s.pin, STAY, fs, fc );

            if( cost < inf )
            {
                stay[k] = cost;
                stayFromSlot[k] = fs;
                stayFromCell[k] = fc;
            }
        }

        if( open( k ) )
        {
            int           fs, fc;
            const int64_t cost = arrive( k, s.pin, LANE, fs, fc );

            if( cost < inf )
            {
                lane[k] = cost + miss;
                laneFromSlot[k] = fs;
                laneFromCell[k] = fc;
            }
        }

        for( size_t j = 0; j < cells; ++j )
        {
            low[k][j] = best[k][j];
            lowAt[k][j] = best[k][j] < inf ? (int) j : -1;

            if( j > 0 && low[k][j - 1] <= low[k][j] )
            {
                low[k][j] = low[k][j - 1];
                lowAt[k][j] = lowAt[k][j - 1];
            }
        }
    }

    int64_t total = inf;
    int     lastSlot = -1, lastCell = -1;
    bool    noneOpen = true;

    for( size_t k = n; k-- > 0; )
    {
        const int64_t after = miss * (int64_t) ( n - 1 - k );

        auto offer = [&]( int64_t aCost, int aFrom )
        {
            if( aCost < inf && aCost + after < total )
            {
                total = aCost + after;
                lastSlot = (int) k;
                lastCell = aFrom;
            }
        };

        if( cells > 0 && low[k][cells - 1] < inf )
            offer( low[k][cells - 1], lowAt[k][cells - 1] );

        offer( stay[k], onStay );
        offer( lane[k], onLane );

        if( open( k ) )
        {
            noneOpen = false;
            break;
        }
    }

    if( noneOpen && miss * (int64_t) n < total )
        lastSlot = -1;

    for( int k = lastSlot, j = lastCell; k >= 0; )
    {
        int pk, pj;

        if( j == onLane )
        {
            pk = laneFromSlot[k];
            pj = laneFromCell[k];
        }
        else if( j == onStay )
        {
            out.at[k] = aSlots[k].pin;
            pk = stayFromSlot[k];
            pj = stayFromCell[k];
        }
        else
        {
            out.at[k] = pos( (size_t) j );
            pk = fromSlot[k][j];
            pj = fromCell[k][j];
        }

        k = pk;
        j = pj;
    }

    for( size_t k = 0; k < n; ++k )
    {
        if( out.at[k] )
            out.moved += std::abs( *out.at[k] - aSlots[k].pin );
        else
            out.left++;
    }

    return out;
}
