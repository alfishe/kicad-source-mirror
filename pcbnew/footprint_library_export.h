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

#ifndef FOOTPRINT_LIBRARY_EXPORT_H
#define FOOTPRINT_LIBRARY_EXPORT_H

#include <vector>

#include <libraries/library_table.h>
#include <wx/string.h>

class BOARD;
class FOOTPRINT;
class FOOTPRINT_LIBRARY_ADAPTER;
class PROJECT;


/// The plugin type Add Library gives the library at @a aLibPath (KiCad when it cannot tell)
wxString FootprintLibraryType( const wxString& aLibPath );

/// The nickname Add Library gives the library at @a aLibPath
wxString FootprintLibraryNickname( const wxString& aLibPath );


/**
 * Add Library without the file dialog: the library at @a aLibPath goes into the global or project
 * footprint library table (nickname from its file name, path normalised to an environment
 * variable, or the project for the project table) through EditLibraryTable, and the library
 * loaded.
 *
 * @return false when there is no library name (@a aError empty), or with @a aError from
 *         EditLibraryTable
 */
bool AddFootprintLibraryToTable( PROJECT& aProject, const wxString& aLibPath, LIBRARY_TABLE_SCOPE aScope,
                                 wxString& aNickname, wxString& aError );


struct FOOTPRINT_EXPORT_RESULT
{
    FOOTPRINT* m_Footprint = nullptr;
    bool       m_Saved = false;
    wxString   m_Error;     ///< empty when saved or skipped (no library name)
};


/**
 * Export Footprints to Library without the dialogs: each board footprint with a library name is
 * saved into the library @a aNickname in library form (REF**, no group, zones relative to the
 * footprint), overwriting a footprint of that name.  Relinking the board is left to the caller.
 */
std::vector<FOOTPRINT_EXPORT_RESULT> ExportFootprintsToLibrary( BOARD& aBoard, FOOTPRINT_LIBRARY_ADAPTER& aAdapter,
                                                                const wxString& aNickname );

#endif
