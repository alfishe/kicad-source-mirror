#pragma once

#include <geometry/shape.h>
#include <geometry/shape_circle.h>
#include <geometry/shape_compound.h>
#include <geometry/shape_rect.h>
#include <geometry/shape_segment.h>
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

        std::vector<PIECE> pieces;
        const bool         closed = Pieces( aShape, aInflate, pieces );

        if( closed && pieces.size() == 1 )
        {
            // one piece: the cells between its may and sure spans are tested, the sure ones taken
            const PIECE& piece = pieces[0];

            for( int y = y0; y <= y1; ++y )
            {
                int mayLo, mayHi, sureLo, sureHi;
                RowSpan( piece, y, piece.radius + SURE, x0, x1, mayLo, mayHi );
                RowSpan( piece, y, piece.radius - SURE, x0, x1, sureLo, sureHi );

                for( int x = mayLo; x <= mayHi; ++x )
                {
                    const size_t i = aOffset + Index( x, y );

                    if( !aSkip( i ) && ( ( x >= sureLo && x <= sureHi ) || aShape.Collide( Centre( x, y ), aInflate ) ) )
                        aVisit( i );
                }
            }

            return;
        }

        std::vector<uint8_t> row( (size_t) std::max( x1 - x0 + 1, 0 ) );

        for( int y = y0; y <= y1; ++y )
        {
            // per cell of the row: 2 the shape surely covers it, 1 it may (its own test decides), 0 not
            std::fill( row.begin(), row.end(), closed ? 0 : 1 );

            for( const PIECE& piece : pieces )
            {
                int lo, hi;
                RowSpan( piece, y, piece.radius + SURE, x0, x1, lo, hi );

                for( int x = lo; x <= hi; ++x )
                    row[x - x0] = std::max<uint8_t>( row[x - x0], 1 );

                RowSpan( piece, y, piece.radius - SURE, x0, x1, lo, hi );

                for( int x = lo; x <= hi; ++x )
                    row[x - x0] = 2;
            }

            for( int x = x0; x <= x1; ++x )
            {
                const uint8_t cover = row[x - x0];

                if( cover == 0 )
                    continue;

                const size_t i = aOffset + Index( x, y );

                if( !aSkip( i ) && ( cover == 2 || aShape.Collide( Centre( x, y ), aInflate ) ) )
                    aVisit( i );
            }
        }
    }

    /// @brief A part of a shape whose cells per row are found in closed form: a round-ended
    /// segment (a circle when its ends meet) or an axis-aligned box, grown by a radius
    struct PIECE
    {
        bool   box = false;
        double ax = 0, ay = 0, bx = 0, by = 0;   ///< the segment's ends, or the box's corners, from the origin
        double radius = 0;                       ///< a point nearer than this collides (the shapes square a negative one)
    };

    static constexpr double SURE = 2.0;   ///< far above the rounding of the shapes' integer distances

    /// @return false when a part of the shape has no closed form (every cell is tested), else the
    /// pieces of the shape in aPieces
    bool Pieces( const SHAPE& aShape, int aInflate, std::vector<PIECE>& aPieces ) const
    {
        auto add = [&]( const SHAPE* aPart ) -> bool
        {
            PIECE p;

            switch( aPart->Type() )
            {
            case SH_SEGMENT:
            {
                const SHAPE_SEGMENT* s = static_cast<const SHAPE_SEGMENT*>( aPart );
                p.ax = (double) s->GetSeg().A.x - origin.x;
                p.ay = (double) s->GetSeg().A.y - origin.y;
                p.bx = (double) s->GetSeg().B.x - origin.x;
                p.by = (double) s->GetSeg().B.y - origin.y;
                p.radius = std::abs( (double) ( ( s->GetWidth() + 1 ) / 2 + aInflate ) );
                break;
            }
            case SH_CIRCLE:
            {
                const SHAPE_CIRCLE* c = static_cast<const SHAPE_CIRCLE*>( aPart );
                p.ax = p.bx = (double) c->GetCenter().x - origin.x;
                p.ay = p.by = (double) c->GetCenter().y - origin.y;
                p.radius = std::abs( (double) ( c->GetRadius() + aInflate ) );
                break;
            }
            case SH_RECT:
            {
                const SHAPE_RECT* r = static_cast<const SHAPE_RECT*>( aPart );

                if( r->GetRadius() > 0 || r->GetWidth() < 0 || r->GetHeight() < 0 )
                    return false;

                p.box = true;
                p.ax = (double) r->GetPosition().x - origin.x;
                p.ay = (double) r->GetPosition().y - origin.y;
                p.bx = p.ax + r->GetWidth();
                p.by = p.ay + r->GetHeight();
                p.radius = std::abs( (double) aInflate );
                break;
            }
            default:
                return false;
            }

            aPieces.push_back( p );
            return true;
        };

        if( aShape.Type() == SH_COMPOUND )
        {
            for( const SHAPE* part : static_cast<const SHAPE_COMPOUND&>( aShape ).Shapes() )
            {
                if( !add( part ) )
                {
                    aPieces.clear();
                    return false;
                }
            }

            return true;
        }

        return add( &aShape );
    }

    /// @brief The cells [aLo, aHi] of row aY within aX0..aX1 whose centres are within aRadius of the
    /// piece (a box takes its inside too); aLo > aHi when there are none
    void RowSpan( const PIECE& aPiece, int aY, double aRadius, int aX0, int aX1, int& aLo, int& aHi ) const
    {
        aLo = aX0;
        aHi = aX0 - 1;

        const double py = (double) aY * cell;
        double       lo = HUGE_VAL;
        double       hi = -HUGE_VAL;

        auto disc = [&]( double aCx, double aCy, double aR )
        {
            const double dy = py - aCy;

            if( aR < 0 || std::abs( dy ) > aR )
                return;

            const double h = std::sqrt( aR * aR - dy * dy );
            lo = std::min( lo, aCx - h );
            hi = std::max( hi, aCx + h );
        };

        if( aPiece.box )
        {
            if( aRadius < 0 )
                aRadius = 0;   // the box's inside collides at any clearance

            const double dy = std::max( { aPiece.ay - py, 0.0, py - aPiece.by } );

            if( dy > aRadius )
                return;

            const double h = std::sqrt( aRadius * aRadius - dy * dy );
            lo = aPiece.ax - h;
            hi = aPiece.bx + h;
        }
        else
        {
            if( aRadius < 0 )
                return;

            disc( aPiece.ax, aPiece.ay, aRadius );
            disc( aPiece.bx, aPiece.by, aRadius );

            // the band beside the segment: 0 <= ( P - A ).u <= |u|^2 and |u x ( P - A )| <= r |u|
            const double ux = aPiece.bx - aPiece.ax;
            const double uy = aPiece.by - aPiece.ay;
            const double len2 = ux * ux + uy * uy;

            if( len2 > 0 )
            {
                const double dy = py - aPiece.ay;
                const double side = aRadius * std::sqrt( len2 );
                double       bandLo = -HUGE_VAL;
                double       bandHi = HUGE_VAL;
                bool         empty = false;

                // aMin <= aK * t + aC <= aMax with t = x - A.x
                auto clip = [&]( double aK, double aC, double aMin, double aMax )
                {
                    if( aK == 0 )
                    {
                        empty = empty || aC < aMin || aC > aMax;
                        return;
                    }

                    double t0 = ( aMin - aC ) / aK;
                    double t1 = ( aMax - aC ) / aK;

                    if( t0 > t1 )
                        std::swap( t0, t1 );

                    bandLo = std::max( bandLo, t0 );
                    bandHi = std::min( bandHi, t1 );
                };

                clip( ux, uy * dy, 0, len2 );
                clip( -uy, ux * dy, -side, side );

                if( !empty && bandLo <= bandHi )
                {
                    lo = std::min( lo, aPiece.ax + bandLo );
                    hi = std::max( hi, aPiece.ax + bandHi );
                }
            }
        }

        if( lo > hi )
            return;

        aLo = std::max( aX0, (int) std::ceil( lo / cell ) );
        aHi = std::min( aX1, (int) std::floor( hi / cell ) );
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
