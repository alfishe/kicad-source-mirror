/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
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

#include <polygon_corner_edit.h>

#include <climits>

#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <constraints/constraint_builder.h>
#include <geometry/corner_operations.h>
#include <geometry/seg.h>
#include <geometry/shape_arc.h>
#include <geometry/shape_poly_set.h>
#include <pcb_shape.h>
#include <zone.h>
#include <wx/intl.h>


SHAPE_POLY_SET* EditableCornerOutline( EDA_ITEM* aItem )
{
    if( !aItem )
        return nullptr;

    if( aItem->Type() == PCB_ZONE_T )
        return static_cast<ZONE*>( aItem )->Outline();

    if( aItem->Type() == PCB_SHAPE_T && static_cast<PCB_SHAPE*>( aItem )->GetShape() == SHAPE_T::POLY )
        return &static_cast<PCB_SHAPE*>( aItem )->GetPolyShape();

    return nullptr;
}


bool CanAddShapeCorner( const EDA_ITEM& aItem )
{
    if( aItem.Type() == PCB_ZONE_T )
        return true;

    if( aItem.Type() == PCB_SHAPE_T )
    {
        const SHAPE_T shapeType = static_cast<const PCB_SHAPE&>( aItem ).GetShape();
        return shapeType == SHAPE_T::SEGMENT || shapeType == SHAPE_T::POLY || shapeType == SHAPE_T::ARC;
    }

    return false;
}


bool CanChamferShapeCorner( const EDA_ITEM& aItem )
{
    if( aItem.Type() == PCB_ZONE_T )
        return true;

    return aItem.Type() == PCB_SHAPE_T && static_cast<const PCB_SHAPE&>( aItem ).GetShape() == SHAPE_T::POLY;
}


bool CanRemoveShapeCorner( EDA_ITEM* aItem, int aGlobalIndex )
{
    SHAPE_POLY_SET*              polyset = EditableCornerOutline( aItem );
    SHAPE_POLY_SET::VERTEX_INDEX idx;

    if( !polyset || !polyset->GetRelativeIndices( aGlobalIndex, &idx ) )
        return false;

    // Holes may lose every corner (a hole of two corners goes away), outlines keep three
    return !( idx.m_contour == 0 && polyset->Polygon( idx.m_polygon )[idx.m_contour].PointCount() <= 3 );
}


int NearestShapeCorner( EDA_ITEM* aItem, const VECTOR2I& aPos )
{
    SHAPE_POLY_SET* polygon = EditableCornerOutline( aItem );

    if( !polygon || polygon->OutlineCount() == 0 )
        return -1;

    unsigned int nearestIdx = 0;
    unsigned int nearestDist = INT_MAX;
    int          curr_idx = 0;

    for( SHAPE_POLY_SET::ITERATOR it = polygon->Iterate( 0, polygon->OutlineCount() - 1, true ); it; it++, curr_idx++ )
    {
        unsigned int distance = polygon->CVertex( curr_idx ).Distance( aPos );

        if( distance < nearestDist )
        {
            nearestDist = distance;
            nearestIdx = curr_idx;
        }
    }

    return (int) nearestIdx;
}


std::optional<wxString> AddShapeCorner( BOARD* aBoard, BOARD_ITEM* aItem, const VECTOR2I& aPos, BOARD_COMMIT& aCommit )
{
    if( !aItem || !CanAddShapeCorner( *aItem ) )
        return std::nullopt;

    PCB_SHAPE* graphicItem = dynamic_cast<PCB_SHAPE*>( aItem );

    if( SHAPE_POLY_SET* zoneOutline = EditableCornerOutline( aItem ) )
    {
        unsigned int nearestIdx = 0;
        unsigned int nextNearestIdx = 0;
        unsigned int nearestDist = INT_MAX;
        unsigned int firstPointInContour = 0;

        if( aItem->Type() == PCB_ZONE_T )
            static_cast<ZONE*>( aItem )->SetNeedRefill( true );

        aCommit.Modify( aItem );

        // The outline side (main contour or hole) nearest aPos is split in two
        SHAPE_POLY_SET::ITERATOR iterator = zoneOutline->Iterate( 0, zoneOutline->OutlineCount() - 1, true );
        int                      curr_idx = 0;

        for( ; iterator; iterator++, curr_idx++ )
        {
            int jj = curr_idx + 1;

            if( iterator.IsEndContour() )
            {
                jj = firstPointInContour;
                firstPointInContour = curr_idx + 1;
            }

            SEG          curr_segment( zoneOutline->CVertex( curr_idx ), zoneOutline->CVertex( jj ) );
            unsigned int distance = curr_segment.Distance( aPos );

            if( distance < nearestDist )
            {
                nearestDist = distance;
                nearestIdx = curr_idx;
                nextNearestIdx = jj;
            }
        }

        const VECTOR2I& sideOrigin = zoneOutline->CVertex( nearestIdx );
        const VECTOR2I& sideEnd = zoneOutline->CVertex( nextNearestIdx );
        SEG             nearestSide( sideOrigin, sideEnd );
        VECTOR2I        nearestPoint = nearestSide.NearestPoint( aPos );

        // No corner on top of an existing one: the side's middle instead
        if( nearestPoint == sideOrigin || nearestPoint == sideEnd )
            nearestPoint = ( sideOrigin + sideEnd ) / 2;

        zoneOutline->InsertVertex( nextNearestIdx, nearestPoint );

        // Only hole-free shape polygons carry vertex constraints; the ordinals past the new one shift
        if( graphicItem && nextNearestIdx < (unsigned) zoneOutline->COutline( 0 ).PointCount() )
        {
            RemapPolygonVertexMembers( aBoard, graphicItem->m_Uuid, (int) nextNearestIdx, 1,
                                       [&]( BOARD_ITEM* aConstraint ) { aCommit.Modify( aConstraint ); },
                                       [&]( BOARD_ITEM* aConstraint ) { aCommit.Remove( aConstraint ); } );
        }

        if( aItem->Type() == PCB_ZONE_T )
            static_cast<ZONE*>( aItem )->HatchBorder();

        return _( "Add Zone Corner" );
    }

    if( graphicItem && graphicItem->GetShape() == SHAPE_T::SEGMENT )
    {
        aCommit.Modify( graphicItem );

        SEG      seg( graphicItem->GetStart(), graphicItem->GetEnd() );
        VECTOR2I nearestPoint = seg.NearestPoint( aPos );

        graphicItem->SetEnd( nearestPoint );

        PCB_SHAPE* newSegment = static_cast<PCB_SHAPE*>( graphicItem->Duplicate( true, &aCommit ) );
        newSegment->ClearSelected();
        newSegment->SetStart( nearestPoint );
        newSegment->SetEnd( VECTOR2I( seg.B.x, seg.B.y ) );

        aCommit.Add( newSegment );
        return _( "Split Segment" );
    }

    if( graphicItem && graphicItem->GetShape() == SHAPE_T::ARC )
    {
        aCommit.Modify( graphicItem );

        const SHAPE_ARC arc( graphicItem->GetStart(), graphicItem->GetArcMid(), graphicItem->GetEnd(), 0 );
        const VECTOR2I  nearestPoint = arc.NearestPoint( aPos );

        graphicItem->SetEnd( nearestPoint );

        PCB_SHAPE* newArc = static_cast<PCB_SHAPE*>( graphicItem->Duplicate( true, &aCommit ) );
        newArc->ClearSelected();
        newArc->SetEnd( arc.GetP1() );
        newArc->SetStart( nearestPoint );

        aCommit.Add( newArc );
        return _( "Split Arc" );
    }

    return std::nullopt;
}


std::optional<wxString> RemoveShapeCorner( BOARD* aBoard, BOARD_ITEM* aItem, int aGlobalIndex, BOARD_COMMIT& aCommit,
                                           CORNER_REMOVAL* aRemoval )
{
    SHAPE_POLY_SET*              polygon = EditableCornerOutline( aItem );
    SHAPE_POLY_SET::VERTEX_INDEX vertexIdx;

    if( !polygon || !polygon->GetRelativeIndices( aGlobalIndex, &vertexIdx ) )
        return std::nullopt;

    if( aItem->Type() == PCB_ZONE_T )
        static_cast<ZONE*>( aItem )->SetNeedRefill( true );

    SHAPE_LINE_CHAIN& outline = polygon->Polygon( vertexIdx.m_polygon )[vertexIdx.m_contour];
    CORNER_REMOVAL    removal = CORNER_REMOVAL::CORNER;

    if( outline.PointCount() > 3 )
    {
        aCommit.Modify( aItem );
        polygon->RemoveVertex( vertexIdx );

        // Vertex constraints live on hole-free shape polygons: the removed vertex retires its
        // constraint, later ordinals shift down
        if( aItem->Type() == PCB_SHAPE_T && vertexIdx.m_contour == 0 )
        {
            RemapPolygonVertexMembers( aBoard, aItem->m_Uuid, vertexIdx.m_vertex, -1,
                                       [&]( BOARD_ITEM* aConstraint ) { aCommit.Modify( aConstraint ); },
                                       [&]( BOARD_ITEM* aConstraint ) { aCommit.Remove( aConstraint ); } );
        }
    }
    else if( vertexIdx.m_contour > 0 )
    {
        aCommit.Modify( aItem );
        polygon->RemoveContour( vertexIdx.m_contour, vertexIdx.m_polygon );
        removal = CORNER_REMOVAL::HOLE;
    }
    else
    {
        aCommit.Remove( aItem );
        removal = CORNER_REMOVAL::ITEM;
    }

    if( removal != CORNER_REMOVAL::ITEM && aItem->Type() == PCB_ZONE_T )
        static_cast<ZONE*>( aItem )->HatchBorder();

    if( aRemoval )
        *aRemoval = removal;

    return aItem->Type() == PCB_ZONE_T ? _( "Remove Zone Corner" ) : _( "Remove Polygon Corner" );
}


std::optional<wxString> ChamferShapeCorner( BOARD* aBoard, BOARD_ITEM* aItem, const VECTOR2I& aPos, BOARD_COMMIT& aCommit,
                                            std::optional<int> aSetback, bool* aDone )
{
    SHAPE_POLY_SET* polygon = EditableCornerOutline( aItem );

    if( aDone )
        *aDone = false;

    if( !polygon )
        return std::nullopt;

    if( aItem->Type() == PCB_ZONE_T )
        static_cast<ZONE*>( aItem )->SetNeedRefill( true );

    const int nearestIdx = NearestShapeCorner( aItem, aPos );
    int       prevIdx, nextIdx;

    if( nearestIdx >= 0 && polygon->GetNeighbourIndexes( nearestIdx, &prevIdx, &nextIdx ) )
    {
        const SEG segA{ polygon->CVertex( prevIdx ), polygon->CVertex( nearestIdx ) };
        const SEG segB{ polygon->CVertex( nextIdx ), polygon->CVertex( nearestIdx ) };

        int setback = 0;

        if( aSetback )
        {
            setback = *aSetback;
        }
        else
        {
            // A plausible setback that won't consume a whole edge
            setback = pcbIUScale.mmToIU( 5 );
            setback = std::min( setback, (int) ( segA.Length() * 0.25 ) );
            setback = std::min( setback, (int) ( segB.Length() * 0.25 ) );
        }

        CHAMFER_PARAMS                chamferParams{ setback, setback };
        std::optional<CHAMFER_RESULT> chamferResult = ComputeChamferPoints( segA, segB, chamferParams );

        if( chamferResult && chamferResult->m_updated_seg_a && chamferResult->m_updated_seg_b )
        {
            aCommit.Modify( aItem );
            polygon->RemoveVertex( nearestIdx );

            // The two end points of the chamfer are the new corners
            polygon->InsertVertex( nearestIdx, chamferResult->m_updated_seg_b->B );
            polygon->InsertVertex( nearestIdx, chamferResult->m_updated_seg_a->B );

            // The chamfered corner's constraints retire, the ones past it shift up by one: the
            // insert pass runs first (from nearestIdx + 1), then the removal
            if( aItem->Type() == PCB_SHAPE_T && nearestIdx < polygon->COutline( 0 ).PointCount() )
            {
                auto modify = [&]( BOARD_ITEM* aConstraint ) { aCommit.Modify( aConstraint ); };
                auto remove = [&]( BOARD_ITEM* aConstraint ) { aCommit.Remove( aConstraint ); };

                RemapPolygonVertexMembers( aBoard, aItem->m_Uuid, nearestIdx + 1, 2, modify, remove );
                RemapPolygonVertexMembers( aBoard, aItem->m_Uuid, nearestIdx, -1, modify, remove );
            }

            if( aDone )
                *aDone = true;
        }
    }

    if( aItem->Type() == PCB_ZONE_T )
        static_cast<ZONE*>( aItem )->HatchBorder();

    return aItem->Type() == PCB_ZONE_T ? _( "Break Zone Corner" ) : _( "Break Polygon Corner" );
}
