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
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef STEP_MATERIALS_H
#define STEP_MATERIALS_H

#include <optional>
#include <string>
#include <vector>

/**
 * Physically based appearance of a model surface for mesh exports (glTF metallic-roughness).
 */
struct STEP_PBR
{
    double metallic = 0.0;
    double roughness = 0.55;
    double alpha = 1.0;

    /// Set by PbrFromColour: what the colour was taken for ("tin_by_colour", "gold_by_colour")
    std::string guess;
};

/**
 * A named colour of a STEP file, linear RGB (as OpenCascade stores it).
 */
struct STEP_NAMED_COLOUR
{
    double      red;
    double      green;
    double      blue;
    std::string name;
};

/**
 * Appearance from the name of a STEP colour ("tin_bright", "epoxy_black_matte", "gold",
 * "led_lens_green"): a base material word (tin, nickel, gold, epoxy, ceramic, glass ...) plus
 * finish words (bright, matte, glossy, dull, brushed ...).
 *
 * @return nothing when the name names no known material (a generic "Body color" or "").
 */
std::optional<STEP_PBR> PbrFromColourName( const std::string& aName );

/**
 * Appearance guessed from a linear RGB colour, for unnamed colours of vendor models: neutral
 * mid greys read as tin, saturated yellow-orange as gold.
 *
 * @return nothing for anything else (left to the exporter's dielectric default).
 */
std::optional<STEP_PBR> PbrFromColour( double aRed, double aGreen, double aBlue );

#endif // STEP_MATERIALS_H
