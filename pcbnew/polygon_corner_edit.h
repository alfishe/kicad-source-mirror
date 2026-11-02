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

#ifndef POLYGON_CORNER_EDIT_H
#define POLYGON_CORNER_EDIT_H

#include <optional>

#include <math/vector2d.h>
#include <wx/string.h>

class BOARD;
class BOARD_ITEM;
class BOARD_COMMIT;
class EDA_ITEM;
class SHAPE_POLY_SET;

/**
 * Corner edits of the point editor (Add Corner, Remove Corner, Chamfer Corner) on zones and
 * graphic polygons; Add Corner also splits graphic lines and arcs.  Each stages its change in
 * \a aCommit and answers the commit description, or nothing when there was nothing to do; the
 * caller pushes the commit.  Constraints on the polygon's vertices follow the new numbering.
 */

/// The editable outline of a zone or a graphic polygon, nullptr for anything else
SHAPE_POLY_SET* EditableCornerOutline( EDA_ITEM* aItem );

bool CanAddShapeCorner( const EDA_ITEM& aItem );
bool CanChamferShapeCorner( const EDA_ITEM& aItem );

/// The corner at \a aGlobalIndex can go without leaving an outline of fewer than three corners
bool CanRemoveShapeCorner( EDA_ITEM* aItem, int aGlobalIndex );

/// The corner nearest \a aPos (global index), -1 when the item has no editable outline
int NearestShapeCorner( EDA_ITEM* aItem, const VECTOR2I& aPos );

/**
 * A corner on the outline side nearest \a aPos (the side's middle when \a aPos projects onto
 * one of its ends); a line or an arc is split there into two.
 */
std::optional<wxString> AddShapeCorner( BOARD* aBoard, BOARD_ITEM* aItem, const VECTOR2I& aPos, BOARD_COMMIT& aCommit );

enum class CORNER_REMOVAL
{
    CORNER, ///< the corner removed
    HOLE,   ///< a hole of three corners removed
    ITEM    ///< an outline of three corners: the whole item removed
};

/**
 * Remove the corner at \a aGlobalIndex.  A hole of three corners goes away; an outline of three
 * corners removes the item.
 */
std::optional<wxString> RemoveShapeCorner( BOARD* aBoard, BOARD_ITEM* aItem, int aGlobalIndex, BOARD_COMMIT& aCommit,
                                           CORNER_REMOVAL* aRemoval = nullptr );

/**
 * Chamfer the corner nearest \a aPos by \a aSetback, by default 5 mm but at most a quarter of
 * either side.  The description is answered even when the corner could not be chamfered
 * (\a aDone false).
 */
std::optional<wxString> ChamferShapeCorner( BOARD* aBoard, BOARD_ITEM* aItem, const VECTOR2I& aPos, BOARD_COMMIT& aCommit,
                                            std::optional<int> aSetback = std::nullopt, bool* aDone = nullptr );

#endif // POLYGON_CORNER_EDIT_H
