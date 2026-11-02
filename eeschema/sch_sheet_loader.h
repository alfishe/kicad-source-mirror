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

#include <wx/string.h>

class SCHEMATIC;
class SCH_SHEET;
class SCH_SHEET_PATH;


/**
 * The questions and failures of a sheet load: the schematic editor answers them with dialogs,
 * a caller without a window decides them itself.
 */
class SCH_SHEET_LOAD_HANDLER
{
public:
    virtual ~SCH_SHEET_LOAD_HANDLER() = default;

    /**
     * The load can go on but the result may be incomplete (partial file, old instance data,
     * library links that may break).
     *
     * @return true to go on, false to stop the load.
     */
    virtual bool Continue( const wxString& aTitle, const wxString& aMessage, const wxString& aDetail,
                           const wxString& aContinueLabel ) = 0;

    /// The load stopped, or a step it went past failed.
    virtual void Error( const wxString& aMessage, const wxString& aDetail ) = 0;

    /// The load stopped for a reason that is no failure of the file (unremapped library links).
    virtual void Info( const wxString& aMessage ) { Error( aMessage, wxEmptyString ); }

    /// The file could not be read.
    virtual void LoadFailed( const wxString& aFileName ) {}
};


/**
 * Load a schematic file into a sheet: SCH_EDIT_FRAME::LoadSheetFromFile without a frame.
 *
 * @see SCH_EDIT_FRAME::LoadSheetFromFile for the arguments.
 */
bool SchLoadSheetFromFile( SCHEMATIC& aSchematic, SCH_SHEET* aSheet, SCH_SHEET_PATH* aCurrentSheet,
                           const wxString& aFileName, bool aSkipRecursionCheck, bool aSkipLibCheck,
                           SCH_SHEET_LOAD_HANDLER& aHandler );
