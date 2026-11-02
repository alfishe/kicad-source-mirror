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

#ifndef SCH_GLOBAL_EDIT_TEXT_AND_GRAPHICS_H
#define SCH_GLOBAL_EDIT_TEXT_AND_GRAPHICS_H

#include <functional>
#include <optional>
#include <vector>

#include <font/text_attributes.h>
#include <gal/color4d.h>
#include <sch_label.h>
#include <sch_sheet_path.h>
#include <stroke_params.h>
#include <wx/string.h>

class SCH_COMMIT;
class SCH_ITEM;
class SCHEMATIC;

namespace KIFONT
{
class FONT;
}


/**
 * The schematic editor's Edit Text and Graphics: which items, which filters, and what to set.
 * Unset optionals leave the property as it is.
 */
struct SCH_GLOBAL_EDIT_OPTIONS
{
    bool references = false;
    bool values = false;
    bool otherFields = false;
    bool wires = false;
    bool buses = false;
    bool globalLabels = false;
    bool hierLabels = false;
    bool labelFields = false;
    bool sheetTitles = false;
    bool sheetFields = false;
    bool sheetPins = false;
    bool sheetBorders = false;
    bool textAndGraphics = false;

    bool                selectedOnly = false;
    wxString            fieldNameFilter;    ///< wildcards; empty: any
    wxString            referenceFilter;    ///< wildcards; empty: any
    wxString            symbolFilter;       ///< library id wildcards; empty: any
    wxString            netFilter;          ///< wildcards; empty: any
    std::optional<bool> powerSymbols;       ///< true: power symbols only, false: others only

    std::optional<int>               textSize;
    std::optional<KIGFX::COLOR4D>    textColor;
    std::optional<GR_TEXT_H_ALIGN_T> hAlign;
    std::optional<GR_TEXT_V_ALIGN_T> vAlign;
    std::optional<bool>              italic;
    std::optional<bool>              bold;
    std::optional<bool>              visible;
    std::optional<bool>              showFieldNames;
    std::optional<SPIN_STYLE::SPIN>  labelOrientation;

    /// the font for an item's bold / italic; unset keeps the font
    std::function<KIFONT::FONT*( bool aBold, bool aItalic )> font;

    std::optional<int>            lineWidth;
    std::optional<LINE_STYLE>     lineStyle;
    std::optional<KIGFX::COLOR4D> lineColor;
    std::optional<KIGFX::COLOR4D> fillColor;     ///< UNSPECIFIED: no fill
    std::optional<int>            junctionSize;
    std::optional<KIGFX::COLOR4D> junctionColor;
};


struct SCH_GLOBAL_EDIT_CHANGE
{
    SCH_ITEM*      m_Item;
    SCH_SHEET_PATH m_Sheet;
};


/**
 * Apply @a aOptions over every sheet of the hierarchy, as the Edit Text and Graphics dialog does.
 * @a aOnSheet, when set, runs before each sheet is visited.
 *
 * @return the items staged in @a aCommit, in visiting order
 */
std::vector<SCH_GLOBAL_EDIT_CHANGE>
SchGlobalEditTextAndGraphics( SCHEMATIC& aSchematic, SCH_COMMIT& aCommit, const SCH_GLOBAL_EDIT_OPTIONS& aOptions,
                              const std::function<void( const SCH_SHEET_PATH& )>& aOnSheet = {} );

#endif
