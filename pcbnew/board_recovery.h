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

#ifndef BOARD_RECOVERY_H
#define BOARD_RECOVERY_H

#include <kiid.h>
#include <local_history.h>
#include <wx/string.h>

#include <optional>
#include <vector>

class BOARD;
class BOARD_ITEM;


/// What Repair Board found (and fixed, unless a dry run)
struct BOARD_REPAIR_RESULT
{
    struct REPLACED_ID
    {
        BOARD_ITEM* item;
        KIID        oldId;
        KIID        newId;   ///< niluuid in a dry run
    };

    std::vector<REPLACED_ID> duplicateIds;   ///< items whose id another item had claimed first
    std::vector<wxString>    orphanedNets;   ///< nets of pads and tracks missing from the board's list

    int Count() const { return (int) ( duplicateIds.size() + orphanedNets.size() ); }

    /// The editor's lines: "N duplicate IDs replaced.", "Orphaned net X re-parented."
    std::vector<wxString> Details() const;

    /// "N potential problems repaired." or "No board problems found."
    wxString Summary() const;
};


/**
 * Repair Board: give items with an id already taken a new one (footprints first, then pads,
 * tracks, footprint children, the rest) and add the nets of pads and tracks the board's net
 * list lost.  Not an undoable commit; the caller marks the board modified when Count() > 0.
 */
BOARD_REPAIR_RESULT RepairBoard( BOARD& aBoard, bool aDryRun = false );


/// The auto-saved copies of the board file (LOCAL_HISTORY autosave files), newest first
std::vector<AUTOSAVE_FILE_INFO> FindBoardAutosaves( LOCAL_HISTORY& aHistory, const BOARD& aBoard );

#endif
