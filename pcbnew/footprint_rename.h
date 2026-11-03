/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FOOTPRINT_RENAME_H
#define FOOTPRINT_RENAME_H

#include <functional>
#include <wx/string.h>

class FOOTPRINT;


/// What renaming a library footprint to a name meets (Change Footprint Name)
enum class FOOTPRINT_RENAME_CHECK
{
    OK,
    UNCHANGED,   ///< the same name: nothing to do
    EMPTY,       ///< refused: a footprint must have a name
    EXISTS       ///< another footprint of the library has the name: renaming overwrites it
};


/**
 * Check a new name for the footprint @p aOldName of a library (name trimmed by the caller).
 * @param aExists whether the library has a footprint of that name.
 */
FOOTPRINT_RENAME_CHECK CheckFootprintRename( const wxString& aOldName, const wxString& aNewName,
                                             const std::function<bool( const wxString& )>& aExists );

/// The editor's words for a check: empty for OK and UNCHANGED
wxString FootprintRenameMessage( FOOTPRINT_RENAME_CHECK aCheck, const wxString& aNewName,
                                 const wxString& aLibrary );

/// The footprint takes the new name: its library id's item name, and its value when that was the old name
void ApplyFootprintRename( FOOTPRINT& aFootprint, const wxString& aOldName, const wxString& aNewName );

#endif
