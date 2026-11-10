#include "part_fanout.h"

#include <algorithm>
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

    if( aHi < aLo )
    {
        out.left = (int) n;
        return out;
    }

    const size_t  cells = (size_t) ( ( aHi - aLo ) / step ) + 1;
    const int64_t inf = std::numeric_limits<int64_t>::max() / 4;
    // one slot left out costs more than every slot moved across the whole line
    const int64_t miss = ( aHi - aLo + 1 ) * (int64_t) ( n + 1 );

    auto pos = [&]( size_t j ) { return aLo + (int64_t) j * step; };

    // best[k][j]: the least cost of slots 0..k with k placed at cell j as the last placed one; low[k][j] the least
    // of best[k][0..j] and where it is
    std::vector<std::vector<int64_t>> best( n, std::vector<int64_t>( cells, inf ) );
    std::vector<std::vector<int64_t>> low( n, std::vector<int64_t>( cells, inf ) );
    std::vector<std::vector<int>>     lowAt( n, std::vector<int>( cells, -1 ) );
    std::vector<std::vector<int>>     fromSlot( n, std::vector<int>( cells, -1 ) );
    std::vector<std::vector<int>>     fromCell( n, std::vector<int>( cells, -1 ) );

    for( size_t k = 0; k < n; ++k )
    {
        const PART_FAN_SLOT& s = aSlots[k];

        for( size_t j = 0; j < cells; ++j )
        {
            const int64_t x = pos( j );

            if( std::abs( x - s.pin ) > s.reach )
                continue;

            if( std::any_of( s.blocked.begin(), s.blocked.end(), [&]( const auto& b ) { return x > b.first && x < b.second; } ) )
                continue;

            // the first placed: the ones before it left out
            int64_t cost = miss * (int64_t) k;
            int     fs = -1, fc = -1;

            for( size_t i = 0; i < k; ++i )
            {
                const PART_FAN_SLOT& p = aSlots[i];
                const int64_t        gap = std::max( (int64_t) p.half + s.half + std::max( p.clearance, s.clearance ), s.pin - p.pin );
                const int64_t        lastX = x - gap;

                if( lastX < aLo )
                    continue;

                const size_t jj = std::min( (size_t) ( ( lastX - aLo ) / step ), cells - 1 );

                if( low[i][jj] >= inf )
                    continue;

                const int64_t c = low[i][jj] + miss * (int64_t) ( k - i - 1 );

                if( c < cost )
                {
                    cost = c;
                    fs = (int) i;
                    fc = lowAt[i][jj];
                }
            }

            best[k][j] = cost + std::abs( x - s.pin );
            fromSlot[k][j] = fs;
            fromCell[k][j] = fc;
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

    int64_t total = miss * (int64_t) n;
    int     lastSlot = -1, lastCell = -1;

    for( size_t k = 0; k < n; ++k )
    {
        if( low[k][cells - 1] >= inf )
            continue;

        const int64_t c = low[k][cells - 1] + miss * (int64_t) ( n - 1 - k );

        if( c < total )
        {
            total = c;
            lastSlot = (int) k;
            lastCell = lowAt[k][cells - 1];
        }
    }

    for( int k = lastSlot, j = lastCell; k >= 0 && j >= 0; )
    {
        out.at[k] = pos( (size_t) j );
        const int pk = fromSlot[k][j], pj = fromCell[k][j];
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
