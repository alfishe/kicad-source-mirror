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

#ifndef GLOBAL_EDIT_TEXT_AND_GRAPHICS_H
#define GLOBAL_EDIT_TEXT_AND_GRAPHICS_H

#include <functional>
#include <optional>
#include <vector>

#include <layer_ids.h>
#include <wx/string.h>

class BOARD;
class BOARD_COMMIT;
class BOARD_DESIGN_SETTINGS;
class BOARD_ITEM;

namespace KIFONT
{
class FONT;
}


/**
 * Edit Text and Graphics: which items, which filters, and what to set.  Unset optionals leave
 * the property as it is.
 */
struct GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS
{
    bool references = false;
    bool values = false;
    bool otherFootprintFields = false;
    bool footprintTexts = false;
    bool footprintGraphics = false;
    bool footprintDimensions = false;
    bool boardTexts = false;
    bool boardGraphics = false;
    bool boardDimensions = false;

    bool                        selectedOnly = false;
    std::optional<PCB_LAYER_ID> layerFilter;
    wxString                    referenceFilter;    ///< wildcards; empty: any
    wxString                    footprintFilter;    ///< library id wildcards; empty: any

    /// true: each item takes its layer class defaults (Board Setup), the rest below is ignored
    bool setToLayerDefaults = false;

    std::optional<PCB_LAYER_ID> layer;
    std::optional<int>          lineWidth;
    std::optional<int>          textWidth;
    std::optional<int>          textHeight;
    std::optional<int>          textThickness;
    bool                        autoTextThickness = false;
    std::optional<bool>         bold;
    std::optional<bool>         italic;
    std::optional<bool>         keepUpright;
    std::optional<bool>         visible;
    bool                        centerOnFootprint = false;

    /// the font for an item's bold / italic; unset keeps the font
    std::function<KIFONT::FONT*( bool aBold, bool aItalic )> font;
};


/**
 * Apply @a aOptions to the board's footprint texts, fields, graphics and dimensions and, in the
 * board editor, to its own drawings, as the Edit Text and Graphics dialog does.
 *
 * @return the items changed, in visiting order
 */
std::vector<BOARD_ITEM*> GlobalEditTextAndGraphics( BOARD& aBoard, BOARD_COMMIT& aCommit,
                                                    const BOARD_DESIGN_SETTINGS& aSettings,
                                                    const GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS& aOptions,
                                                    bool aBoardEditor );

#endif
