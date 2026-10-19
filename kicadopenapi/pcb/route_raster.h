#pragma once

#include <geometry/shape.h>
#include <geometry/shape_poly_set.h>
#include <math/vector2d.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

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

    /// @brief Visit every cell whose centre is within aInflate of the shape and aSkip does not pass over
    template <typename SKIP, typename VISIT>
    void Each( const SHAPE& aShape, int aInflate, size_t aOffset, SKIP aSkip, VISIT aVisit ) const
    {
        if( aShape.Type() == SH_POLY_SET )
        {
            EachInPoly( static_cast<const SHAPE_POLY_SET&>( aShape ), aInflate, aOffset, aSkip, aVisit );
            return;
        }

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
                const size_t i = aOffset + Index( x, y );

                if( !aSkip( i ) && aShape.Collide( Centre( x, y ), aInflate ) )
                    aVisit( i );
            }
        }
    }

    /// @brief Each() for polygons (zone fills, outlines: thousands of corners): the polygon grown by
    /// aInflate, filled row by row between its edge crossings
    template <typename SKIP, typename VISIT>
    void EachInPoly( const SHAPE_POLY_SET& aPoly, int aInflate, size_t aOffset, SKIP aSkip, VISIT aVisit ) const
    {
        SHAPE_POLY_SET grown = aPoly.CloneDropTriangulation();

        if( aInflate > 0 )
            grown.Inflate( aInflate, CORNER_STRATEGY::ROUND_ALL_CORNERS, std::max( cell / 8, 1 ) );

        std::vector<std::vector<double>> crossings( ny );

        for( auto seg = grown.CIterateSegmentsWithHoles(); seg; ++seg )
        {
            VECTOR2I a = seg.Get().A - origin;
            VECTOR2I b = seg.Get().B - origin;

            if( a.y == b.y )
                continue;

            if( a.y > b.y )
                std::swap( a, b );

            // rows whose centre line y is in [a.y, b.y)
            const int r0 = std::max( 0, (int) std::ceil( (double) a.y / cell ) );
            const int r1 = std::min( ny - 1, (int) std::ceil( (double) b.y / cell ) - 1 );

            for( int r = r0; r <= r1; ++r )
            {
                const double y = (double) r * cell;
                crossings[r].push_back( a.x + ( y - a.y ) * ( b.x - a.x ) / ( b.y - a.y ) );
            }
        }

        for( int y = 0; y < ny; ++y )
        {
            std::vector<double>& xs = crossings[y];
            std::sort( xs.begin(), xs.end() );

            for( size_t k = 0; k + 1 < xs.size(); k += 2 )
            {
                const int x0 = std::max( 0, (int) std::ceil( xs[k] / cell ) );
                const int x1 = std::min( nx - 1, (int) std::floor( xs[k + 1] / cell ) );

                for( int x = x0; x <= x1; ++x )
                {
                    const size_t i = aOffset + Index( x, y );

                    if( !aSkip( i ) )
                        aVisit( i );
                }
            }
        }
    }

    /// @brief Every cell whose centre is within aInflate of the shape
    void Mark( const SHAPE& aShape, int aInflate, std::vector<uint8_t>& aCells, size_t aOffset = 0 ) const
    {
        Each( aShape, aInflate, aOffset, [&]( size_t i ) { return aCells[i] != 0; },
              [&]( size_t i ) { aCells[i] = 1; } );
    }
};
