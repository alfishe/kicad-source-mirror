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

#ifndef LIBRARY_TABLE_EDIT_H
#define LIBRARY_TABLE_EDIT_H

#include <functional>

#include <kicommon.h>
#include <libraries/library_table.h>
#include <wx/string.h>


/**
 * The Library Tables dialog's checks on a table: rows without a URI dropped, nickname and URI
 * trimmed, every nickname present, legal and unique; the hidden flag cleared where the table
 * kind has no visibility column (footprints).
 *
 * @return the first problem, empty when the table is valid
 */
KICOMMON_API wxString VerifyLibraryTable( LIBRARY_TABLE& aTable );


/**
 * Edit the global or project library table of @a aType as the Library Tables dialog does: a copy
 * is changed by @a aEdit (which answers an error or empty), verified, put in place and saved, then
 * the tables of that type are reloaded.  The project table is created when it does not exist.
 *
 * @return false with @a aError when there is no such table, @a aEdit or the checks refused, or the
 *         file could not be saved
 */
KICOMMON_API bool EditLibraryTable( LIBRARY_TABLE_TYPE aType, LIBRARY_TABLE_SCOPE aScope,
                                    const std::function<wxString( LIBRARY_TABLE& )>& aEdit, wxString& aError );

#endif
