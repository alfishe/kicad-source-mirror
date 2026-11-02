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

#ifndef NEW_FOOTPRINT_H
#define NEW_FOOTPRINT_H

#include <wx/string.h>

class BOARD;
class FOOTPRINT;

/**
 * A new footprint named @p aName (its LIB_ID item name, and reference / value when the board's
 * default footprint texts leave them empty) carrying the board's default footprint texts and text
 * styles.  Owned by the caller; its parent is @p aBoard but it is not added to it.
 */
FOOTPRINT* NewFootprintFromDefaults( BOARD* aBoard, const wxString& aName, int aAttributes );

#endif
