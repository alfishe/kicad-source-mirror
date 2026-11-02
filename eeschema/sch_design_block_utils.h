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

#pragma once

#include <memory>
#include <vector>

class EDA_ITEM;
class SCHEMATIC;
class SCH_SHEET;


/**
 * Make the sheet a design block's schematic is saved from: copies of \a aItems (a group with its
 * members, a symbol with its fields) on a new screen.
 */
std::unique_ptr<SCH_SHEET> MakeDesignBlockSheet( SCHEMATIC* aSchematic, const std::vector<EDA_ITEM*>& aItems );
