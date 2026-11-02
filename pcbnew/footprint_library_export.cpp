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

#include <footprint_library_export.h>

#include <board.h>
#include <env_paths.h>
#include <footprint.h>
#include <footprint_library_adapter.h>
#include <ki_exception.h>
#include <libraries/library_manager.h>
#include <pcb_io/pcb_io_mgr.h>
#include <pgm_base.h>
#include <project.h>
#include <project_pcb.h>
#include <wildcards_and_files_ext.h>
#include <zone.h>

#include <wx/filename.h>

#include <memory>


bool AddFootprintLibraryToTable( PROJECT& aProject, const wxString& aLibPath, LIBRARY_TABLE_SCOPE aScope,
                                 wxString& aNickname, wxString& aError )
{
    FOOTPRINT_LIBRARY_ADAPTER* adapter = PROJECT_PCB::FootprintLibAdapter( &aProject );
    LIBRARY_MANAGER&           manager = Pgm().GetLibraryManager();
    wxFileName                 fn( aLibPath );
    wxString                   libPath = fn.GetFullPath();
    wxString                   libName = fn.GetName();

    if( libName.IsEmpty() )
        return false;

    PCB_IO_MGR::PCB_FILE_T lib_type = PCB_IO_MGR::GuessPluginTypeFromLibPath( libPath );

    if( lib_type == PCB_IO_MGR::FILE_TYPE_NONE )
        lib_type = PCB_IO_MGR::KICAD_SEXP;

    wxString type = PCB_IO_MGR::ShowType( lib_type );

    // KiCad lib is our default guess.  So it might not have the .pretty extension
    // In this case, the extension is part of the library name
    if( lib_type == PCB_IO_MGR::KICAD_SEXP && fn.GetExt() != FILEEXT::KiCadFootprintLibPathExtension )
        libName = fn.GetFullName();

    // try to use path normalized to an environmental variable or project path
    wxString normalizedPath = NormalizePath( libPath, &Pgm().GetLocalEnvVariables(), &aProject );
    bool     success = true;

    try
    {
        std::optional<LIBRARY_TABLE*> optTable = manager.Table( LIBRARY_TABLE_TYPE::FOOTPRINT, aScope );

        if( !optTable )
            return false;

        LIBRARY_TABLE* table = optTable.value();

        LIBRARY_TABLE_ROW& row = table->InsertRow();

        row.SetNickname( libName );
        row.SetURI( normalizedPath );
        row.SetType( type );

        table->Save().map_error(
                [&]( const LIBRARY_ERROR& aLibError )
                {
                    aError = _( "Error saving library table:\n\n" ) + aLibError.message;
                    success = false;
                } );
    }
    catch( const IO_ERROR& ioe )
    {
        aError = ioe.What();
        return false;
    }

    if( !success )
        return false;

    manager.ReloadTables( aScope, { LIBRARY_TABLE_TYPE::FOOTPRINT } );
    adapter->LoadOne( libName );
    aNickname = libName;
    return true;
}


std::vector<FOOTPRINT_EXPORT_RESULT> ExportFootprintsToLibrary( BOARD& aBoard, FOOTPRINT_LIBRARY_ADAPTER& aAdapter,
                                                                const wxString& aNickname )
{
    std::vector<FOOTPRINT_EXPORT_RESULT> results;

    for( FOOTPRINT* footprint : aBoard.Footprints() )
    {
        FOOTPRINT_EXPORT_RESULT& result = results.emplace_back();
        result.m_Footprint = footprint;

        try
        {
            if( !footprint->GetFPID().GetLibItemName().empty() )    // Handle old boards.
            {
                std::unique_ptr<FOOTPRINT> fpCopy(
                        static_cast<FOOTPRINT*>( footprint->Duplicate( IGNORE_PARENT_GROUP ) ) );

                // Reset reference designator, group membership, and zone offset before saving

                fpCopy->SetReference( "REF**" );
                fpCopy->SetParentGroup( nullptr );

                for( ZONE* zone : fpCopy->Zones() )
                    zone->Move( -fpCopy->GetPosition() );

                result.m_Saved = aAdapter.SaveFootprint( aNickname, fpCopy.get(), true )
                                 == FOOTPRINT_LIBRARY_ADAPTER::SAVE_OK;
            }
        }
        catch( const IO_ERROR& ioe )
        {
            result.m_Error = ioe.What();
        }
    }

    return results;
}
