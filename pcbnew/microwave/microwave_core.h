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

#ifndef MICROWAVE_CORE_H
#define MICROWAVE_CORE_H

#include <geometry/eda_angle.h>
#include <math/vector2d.h>
#include <wx/string.h>

#include <vector>

class BOARD;
class FOOTPRINT;

/// Microwave shapes that are created as board footprints
enum class MICROWAVE_FOOTPRINT_SHAPE
{
    GAP,
    STUB,
    STUB_ARC,
    FUNCTION_SHAPE,
};

/// Parameters for construction of a microwave inductor
struct MICROWAVE_INDUCTOR_PATTERN
{
    VECTOR2I m_Start;
    VECTOR2I m_End;
    int      m_Length;       ///< full length of the trace
    int      m_Width;        ///< trace width
};

/// How the outline of a shape description file becomes a polygon
enum class MICROWAVE_POLYGON_MODE
{
    NORMAL,
    SYMMETRICAL,   ///< the outline and its mirror about the X axis
    MIRRORED,      ///< the outline mirrored about the X axis
};

/// A shape description file: normalized points (0..1) and the file's scale in IU
struct MICROWAVE_SHAPE_FILE
{
    std::vector<VECTOR2D> points;
    VECTOR2D              scale = { 1.0, 1.0 };
};

/**
 * A footprint for microwave applications with @p aPadCount pads: SMD, rectangular, the board's
 * current track width, on F.Cu, excluded from position files and BOM.  @p aTextSize sizes the
 * reference and value (<= 0 keeps the board defaults).
 */
FOOTPRINT* CreateMicrowaveBaseFootprint( BOARD* aBoard, const wxString& aValue, int aTextSize, int aPadCount );

/**
 * A gap (two pads @p aSize apart), a stub (pad 2 @p aSize long) or an arc stub (radius @p aSize,
 * swept @p aArcAngle, clamped to 0..180 degrees).  The pads are the board's current track width.
 * @return nullptr with @p aError set for a non-positive size or another shape.
 */
FOOTPRINT* CreateMicrowaveFootprint( BOARD* aBoard, MICROWAVE_FOOTPRINT_SHAPE aShape, int aSize,
                                     const EDA_ANGLE& aArcAngle, wxString* aError );

/// Why the coil of @p aPattern cannot be built; empty when it can
wxString CheckMicrowaveInductor( const MICROWAVE_INDUCTOR_PATTERN& aPattern );

/**
 * An S-shaped coil footprint named @p aValue from m_End to m_Start of @p aPattern, m_Length long.
 * @return nullptr with @p aError set when the length cannot be built.
 */
FOOTPRINT* CreateMicrowaveInductor( BOARD* aBoard, const MICROWAVE_INDUCTOR_PATTERN& aPattern,
                                    const wxString& aValue, wxString* aError );

/**
 * Read a shape description file:
 *
 *  Unit=MM
 *  XScale=271.501
 *  YScale=1.00133
 *  $COORD
 *  0                      0.6112600148417837
 *  0.001851851851851852   0.6104800531118608
 *  $ENDCOORD
 *
 * Each coordinate line is X Y in normalized units from 0 to 1.
 */
bool ReadMicrowaveShapeFile( const wxString& aPath, MICROWAVE_SHAPE_FILE& aShape, wxString* aError );

/**
 * A two-pad footprint with a filled F.Cu polygon from normalized @p aPoints scaled to @p aSize
 * (IU), its anchor in the middle of the shape length.
 * @return nullptr with @p aError set for a null size or no points.
 */
FOOTPRINT* CreateMicrowavePolygon( BOARD* aBoard, const std::vector<VECTOR2D>& aPoints, const VECTOR2I& aSize,
                                   MICROWAVE_POLYGON_MODE aMode, wxString* aError );

#endif
