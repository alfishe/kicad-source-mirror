/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2015-2016 Mario Luzeiro <mrluzeiro@ua.pt>
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file triangle_2d.cpp
 */

#include "triangle_2d.h"
#include "../ray.h"
#include <wx/debug.h>
#include <geometry/polygon_triangulation.h>
#include <thread_pool.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <vector>
#include "../../../3d_fastmath.h"


TRIANGLE_2D::TRIANGLE_2D( const SFVEC2F& aV1, const SFVEC2F& aV2, const SFVEC2F& aV3,
                          const BOARD_ITEM& aBoardItem ) :
        OBJECT_2D( OBJECT_2D_TYPE::TRIANGLE, aBoardItem )
{
    p1 = aV1;
    p2 = aV2;
    p3 = aV3;

    // Pre-Calc values
    m_inv_denominator = 1.0f / ( ( p2.y - p3.y ) * ( p1.x - p3.x ) +
                                 ( p3.x - p2.x ) * ( p1.y - p3.y ) );
    m_p2y_minus_p3y = ( p2.y - p3.y );
    m_p3x_minus_p2x = ( p3.x - p2.x );
    m_p3y_minus_p1y = ( p3.y - p1.y );
    m_p1x_minus_p3x = ( p1.x - p3.x );

    m_bbox.Reset();
    m_bbox.Union( aV1 );
    m_bbox.Union( aV2 );
    m_bbox.Union( aV3 );
    m_bbox.ScaleNextUp();
    m_centroid = m_bbox.GetCenter();

    wxASSERT( m_bbox.IsInitialized() );
}


bool TRIANGLE_2D::Intersects( const BBOX_2D& aBBox ) const
{
    if( !m_bbox.Intersects( aBBox ) )
        return false;

    //!TODO: Optimize
    return true;
}


bool TRIANGLE_2D::Overlaps( const BBOX_2D& aBBox ) const
{
    // NOT IMPLEMENTED
    return false;
}


bool TRIANGLE_2D::Intersect( const RAYSEG2D& aSegRay, float* aOutT, SFVEC2F* aNormalOut ) const
{
    return false;
}


INTERSECTION_RESULT TRIANGLE_2D::IsBBoxInside( const BBOX_2D& aBBox ) const
{
    if( !m_bbox.Intersects( aBBox ) )
        return INTERSECTION_RESULT::MISSES;

    // !TODO:
    return INTERSECTION_RESULT::MISSES;
}


bool TRIANGLE_2D::IsPointInside( const SFVEC2F& aPoint ) const
{
    // http://totologic.blogspot.co.uk/2014/01/accurate-point-in-triangle-test.html

    SFVEC2F point_minus_p3 = aPoint - p3;

    // barycentric coordinate system
    const float a = ( m_p2y_minus_p3y * point_minus_p3.x +
                      m_p3x_minus_p2x * point_minus_p3.y ) * m_inv_denominator;

    if( 0.0f > a || a > 1.0f )
        return false;

    const float b = ( m_p3y_minus_p1y * point_minus_p3.x +
                      m_p1x_minus_p3x * point_minus_p3.y ) * m_inv_denominator;

    if( 0.0f > b || b > 1.0f )
        return false;

    const float c = 1.0f - a - b;

    return 0.0f <= c && c <= 1.0f;
}


namespace
{

// Joining holes to their outline costs holes x vertices, so a board body with thousands of drill
// holes took seconds. Cut into tiles, each piece has few holes; tiles run on the thread pool. The
// seams add coplanar triangles only, which a 3D view cannot show.
constexpr int TILED_MIN_HOLES = 256;
constexpr int HOLES_PER_TILE = 48;

struct TRIANGLE_PTS
{
    VECTOR2I a, b, c;
};


bool appendTriangulation( SHAPE_POLY_SET& aPoly, std::vector<TRIANGLE_PTS>& aOut )
{
    if( aPoly.OutlineCount() == 0 )
        return true;

    aPoly.CacheTriangulation();

    if( aPoly.TriangulatedPolyCount() == 0 )
        return false;

    for( unsigned int j = 0; j < aPoly.TriangulatedPolyCount(); j++ )
    {
        const SHAPE_POLY_SET::TRIANGULATED_POLYGON* tri = aPoly.TriangulatedPolygon( j );

        for( size_t i = 0; i < tri->GetTriangleCount(); i++ )
        {
            TRIANGLE_PTS t;
            tri->GetTriangle( i, t.a, t.b, t.c );
            aOut.push_back( t );
        }
    }

    return true;
}


/// @return false when the polygon has too few holes to gain, or a tile failed (use the whole polygon)
bool triangulateTiled( const SHAPE_POLY_SET& aPolyList, std::vector<TRIANGLE_PTS>& aOut )
{
    int holes = 0;

    for( int i = 0; i < aPolyList.OutlineCount(); ++i )
        holes += aPolyList.HoleCount( i );

    if( holes < TILED_MIN_HOLES )
        return false;

    const BOX2I box = aPolyList.BBox();
    const int   n = std::clamp( (int) std::ceil( std::sqrt( holes / (double) HOLES_PER_TILE ) ), 2, 32 );
    const int64_t w = box.GetWidth(), h = box.GetHeight();

    std::vector<std::vector<TRIANGLE_PTS>> parts( (size_t) n * n );
    std::vector<char>                      ok( parts.size(), 0 );

    // each tile clips only the contours whose box reaches into it (not the whole board each time)
    std::vector<std::vector<BOX2I>> boxes( aPolyList.OutlineCount() );

    for( int i = 0; i < aPolyList.OutlineCount(); ++i )
    {
        for( const SHAPE_LINE_CHAIN& chain : aPolyList.CPolygon( i ) )
            boxes[i].push_back( chain.BBox() );
    }

    auto tile = [&]( int aIndex )
    {
        const int     tx = aIndex % n, ty = aIndex / n;
        const int64_t x0 = box.GetX() + w * tx / n, x1 = box.GetX() + w * ( tx + 1 ) / n;
        const int64_t y0 = box.GetY() + h * ty / n, y1 = box.GetY() + h * ( ty + 1 ) / n;

        SHAPE_POLY_SET rect;
        rect.NewOutline();
        rect.Append( (int) x0, (int) y0 );
        rect.Append( (int) x1, (int) y0 );
        rect.Append( (int) x1, (int) y1 );
        rect.Append( (int) x0, (int) y1 );

        const BOX2I    tileBox( VECTOR2I( (int) x0, (int) y0 ), VECTOR2I( (int) ( x1 - x0 ), (int) ( y1 - y0 ) ) );
        SHAPE_POLY_SET touching;

        for( int i = 0; i < aPolyList.OutlineCount(); ++i )
        {
            if( !boxes[i][0].Intersects( tileBox ) )
                continue;

            const SHAPE_POLY_SET::POLYGON& poly = aPolyList.CPolygon( i );
            SHAPE_POLY_SET::POLYGON        kept{ poly[0] };

            for( size_t h = 1; h < poly.size(); ++h )
            {
                if( boxes[i][h].Intersects( tileBox ) )
                    kept.push_back( poly[h] );
            }

            touching.AddPolygon( kept );
        }

        SHAPE_POLY_SET piece;
        piece.BooleanIntersection( touching, rect );
        ok[aIndex] = appendTriangulation( piece, parts[aIndex] );
    };

    // a pool worker waiting for its own pool's tasks could stall it: tiles run in line there
    if( BS::this_thread::get_pool() )
    {
        for( int i = 0; i < (int) parts.size(); ++i )
            tile( i );
    }
    else
    {
        thread_pool&                   tp = GetKiCadThreadPool();
        std::vector<std::future<void>> done;

        for( int i = 0; i < (int) parts.size(); ++i )
            done.push_back( tp.submit_task( [&tile, i]() { tile( i ); } ) );

        for( std::future<void>& f : done )
            f.get();
    }

    if( std::find( ok.begin(), ok.end(), 0 ) != ok.end() )
        return false;

    for( std::vector<TRIANGLE_PTS>& part : parts )
        aOut.insert( aOut.end(), part.begin(), part.end() );

    return true;
}

} // namespace


void ConvertPolygonToTriangles( const SHAPE_POLY_SET& aPolyList, CONTAINER_2D_BASE& aDstContainer,
                                float aBiuTo3dUnitsScale, const BOARD_ITEM& aBoardItem )
{
    VECTOR2I a;
    VECTOR2I b;
    VECTOR2I c;

    const double conver_d = (double)aBiuTo3dUnitsScale;

    if( !aPolyList.IsTriangulationUpToDate() )
    {
        std::vector<TRIANGLE_PTS> tiled;

        if( triangulateTiled( aPolyList, tiled ) )
        {
            for( const TRIANGLE_PTS& t : tiled )
            {
                aDstContainer.Add( new TRIANGLE_2D( SFVEC2F( t.a.x * conver_d, -t.a.y * conver_d ),
                                                    SFVEC2F( t.b.x * conver_d, -t.b.y * conver_d ),
                                                    SFVEC2F( t.c.x * conver_d, -t.c.y * conver_d ),
                                                    aBoardItem ) );
            }

            return;
        }
    }

    const_cast<SHAPE_POLY_SET&>( aPolyList ).CacheTriangulation();

    for( unsigned int j = 0; j < aPolyList.TriangulatedPolyCount(); j++ )
    {
        const SHAPE_POLY_SET::TRIANGULATED_POLYGON* triPoly = aPolyList.TriangulatedPolygon( j );

        for( size_t i = 0; i < triPoly->GetTriangleCount(); i++ )
        {
            triPoly->GetTriangle( i, a, b, c );

            aDstContainer.Add( new TRIANGLE_2D( SFVEC2F( a.x * conver_d, -a.y * conver_d ),
                                                SFVEC2F( b.x * conver_d, -b.y * conver_d ),
                                                SFVEC2F( c.x * conver_d, -c.y * conver_d ),
                                                aBoardItem ) );
        }
    }
}
