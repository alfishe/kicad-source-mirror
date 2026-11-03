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

#ifndef SCH_AUTOSAVE_RECOVERY_H
#define SCH_AUTOSAVE_RECOVERY_H

#include <local_history.h>
#include <wx/string.h>

#include <vector>

class SCHEMATIC;


/// The auto-saved copies of the project's schematic files (LOCAL_HISTORY autosave files), newest first
std::vector<AUTOSAVE_FILE_INFO> FindSchematicAutosaves( LOCAL_HISTORY& aHistory, const SCHEMATIC& aSchematic );


/**
 * Write the schematic's sheet files into @p aDir under their paths relative to the root sheet's
 * directory: each from its copy in @p aAutosaves when it has one, else from its saved file (copies
 * of sheets the open hierarchy does not have, e.g. one added before a crash, too).  Sheets outside
 * the root's directory keep their place relative to it.
 * @return the root sheet's copy; empty on failure, with @p aError.
 */
wxString StageSchematicAutosaves( const SCHEMATIC& aSchematic, const std::vector<AUTOSAVE_FILE_INFO>& aAutosaves,
                                  const wxString& aDir, wxString& aError );


/**
 * A schematic loaded from a copy in another directory (StageSchematicAutosaves, an edit history
 * snapshot) gets its own files back: every sheet file under @p aFromDir moves to the same relative
 * place under @p aToDir, top-level sheets named relative to the project again, all marked modified
 * when @p aMarkModified.
 */
void RetargetSchematicFiles( SCHEMATIC& aSchematic, const wxString& aFromDir, const wxString& aToDir,
                             bool aMarkModified );


/// @p aPath moved from under @p aFromDir to the same relative place under @p aToDir
wxString MoveSchematicPath( const wxString& aPath, const wxString& aFromDir, const wxString& aToDir );


/// A new empty directory for StageSchematicAutosaves in the system's temporary directory
wxString MakeSchematicStagingDir();

#endif
