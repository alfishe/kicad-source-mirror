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

#include "board_recovery.h"

#include <board.h>
#include <footprint.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <project.h>
#include <wildcards_and_files_ext.h>
#include <wx/filename.h>
#include <wx/intl.h>

#include <algorithm>
#include <set>


std::vector<wxString> BOARD_REPAIR_RESULT::Details() const
{
    std::vector<wxString> lines;

    if( !duplicateIds.empty() )
        lines.push_back( wxString::Format( _( "%d duplicate IDs replaced." ), (int) duplicateIds.size() ) );

    for( const wxString& net : orphanedNets )
        lines.push_back( wxString::Format( _( "Orphaned net %s re-parented." ), net ) );

    return lines;
}


wxString BOARD_REPAIR_RESULT::Summary() const
{
    if( Count() == 0 )
        return _( "No board problems found." );

    return wxString::Format( _( "%d potential problems repaired." ), Count() );
}


BOARD_REPAIR_RESULT RepairBoard( BOARD& aBoard, bool aDryRun )
{
    BOARD_REPAIR_RESULT result;
    std::vector<std::pair<BOARD_ITEM*, KIID>> replaced;

    aBoard.RepairDuplicateItemUuids( &replaced, aDryRun );

    for( const auto& [item, oldId] : replaced )
        result.duplicateIds.push_back( { item, oldId, aDryRun ? niluuid : item->m_Uuid } );

    std::set<wxString> seen;

    auto checkNet = [&]( BOARD_CONNECTED_ITEM* aItem )
    {
        if( !aItem->GetNetCode() )
            return;

        NETINFO_ITEM* netinfo = aItem->GetNet();

        if( !netinfo || aBoard.FindNet( netinfo->GetNetname() ) || seen.count( netinfo->GetNetname() ) )
            return;

        seen.insert( netinfo->GetNetname() );

        if( !aDryRun )
            aBoard.Add( netinfo );

        result.orphanedNets.push_back( netinfo->GetNetname() );
    };

    for( FOOTPRINT* footprint : aBoard.Footprints() )
    {
        for( PAD* pad : footprint->Pads() )
            checkNet( pad );
    }

    for( PCB_TRACK* track : aBoard.Tracks() )
        checkNet( track );

    return result;
}


std::vector<AUTOSAVE_FILE_INFO> FindBoardAutosaves( LOCAL_HISTORY& aHistory, const BOARD& aBoard )
{
    std::vector<AUTOSAVE_FILE_INFO> found;
    PROJECT*                        project = aBoard.GetProject();

    if( !project || aBoard.GetFileName().IsEmpty() )
        return found;

    const wxFileName board( project->AbsolutePath( aBoard.GetFileName() ) );

    for( AUTOSAVE_FILE_INFO& info :
         aHistory.ListAutosaveFiles( project->GetProjectPath(), { FILEEXT::KiCadPcbFileExtension } ) )
    {
        if( wxFileName( info.sourcePath ).SameAs( board ) )
            found.push_back( std::move( info ) );
    }

    std::sort( found.begin(), found.end(),
               []( const AUTOSAVE_FILE_INFO& a, const AUTOSAVE_FILE_INFO& b )
               {
                   return a.autosaveTime.IsLaterThan( b.autosaveTime );
               } );

    return found;
}
