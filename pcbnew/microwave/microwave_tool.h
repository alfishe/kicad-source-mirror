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

#ifndef TOOLS_MICROWAVE_TOOL_H
#define TOOLS_MICROWAVE_TOOL_H

#include <tools/pcb_tool_base.h>

#include <tool/tool_menu.h>
#include <microwave/microwave_core.h>

/**
 * Tool responsible for adding microwave features to PCBs
 */
class MICROWAVE_TOOL : public PCB_TOOL_BASE
{
public:
    MICROWAVE_TOOL();
    ~MICROWAVE_TOOL();

    /// React to model/view changes
    void Reset( RESET_REASON aReason ) override;

    /// Bind handlers to corresponding TOOL_ACTIONs
    void setTransitions() override;

private:
    /// Main interactive tool
    int addMicrowaveFootprint( const TOOL_EVENT& aEvent );

    /// Create an inductor between the two points
    void createInductorBetween( const VECTOR2I& aStart, const VECTOR2I& aEnd );

    /// Draw a microwave inductor interactively
    int drawMicrowaveInductor( const TOOL_EVENT& aEvent );

    /// Ask for the size (and the arc angle) of a gap, stub or arc stub and create it
    FOOTPRINT* createFootprint( MICROWAVE_FOOTPRINT_SHAPE aFootprintShape );

    /// Ask for the shape description file, size and mode and create the polygon footprint
    FOOTPRINT* createPolygonShape();
};


#endif // TOOLS_MICROWAVE_TOOL_H
