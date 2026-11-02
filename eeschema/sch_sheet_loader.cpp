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

#include "sch_sheet_loader.h"

#include <core/utf8.h>
#include <io/io_mgr.h>
#include <kiid.h>
#include <libraries/library_manager.h>
#include <libraries/library_table.h>
#include <libraries/symbol_library_adapter.h>
#include <project.h>
#include <project_sch.h>
#include <sch_io/sch_io.h>
#include <sch_io/sch_io_mgr.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <schematic.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>
#include <wx/intl.h>

#include <map>
#include <memory>
#include <unordered_set>


bool SchLoadSheetFromFile( SCHEMATIC& aSchematic, SCH_SHEET* aSheet, SCH_SHEET_PATH* aCurrentSheet,
                           const wxString& aFileName, bool aSkipRecursionCheck, bool aSkipLibCheck,
                           SCH_SHEET_LOAD_HANDLER& aHandler )
{
    wxASSERT( aSheet && aCurrentSheet );

    wxString    msg;
    wxFileName  currentSheetFileName;
    bool        libTableChanged = false;

    SCH_IO_MGR::SCH_FILE_T schFileType = SCH_IO_MGR::GuessPluginTypeFromSchPath( aFileName );

    if( schFileType == SCH_IO_MGR::SCH_FILE_UNKNOWN )
        schFileType = SCH_IO_MGR::SCH_KICAD;

    IO_RELEASER<SCH_IO> pi( SCH_IO_MGR::FindPlugin( schFileType ) );
    std::unique_ptr< SCH_SHEET> tmpSheet = std::make_unique<SCH_SHEET>( &aSchematic );

    // This will cause the sheet UUID to be set to the UUID of the aSheet argument.  This is
    // required to ensure all of the sheet paths in any sub-sheets are correctly generated when
    // using the temporary SCH_SHEET object that the file is loaded into..
    const_cast<KIID&>( tmpSheet->m_Uuid ) = aSheet->m_Uuid;

    wxFileName fileName( aFileName );

    if( !fileName.IsAbsolute() && !fileName.MakeAbsolute() )
    {
        wxFAIL_MSG( wxString::Format( "Cannot make file name '%s' path absolute.", aFileName ) );
        return false;
    }

    wxString fullFilename = fileName.GetFullPath();

    // The caller owns the sheet a plugin returns here, so name the context. An importer that can
    // only adopt a whole document into the schematic cannot satisfy that and refuses instead.
    std::map<std::string, UTF8> loadProps;
    loadProps["hierarchical_sheet_load"] = "1";

    try
    {
        if( aSheet->GetScreen() != nullptr )
        {
            tmpSheet.reset( pi->LoadSchematicFile( fullFilename, &aSchematic, nullptr, &loadProps ) );
        }
        else
        {
            tmpSheet->SetFileName( fullFilename );
            pi->LoadSchematicFile( fullFilename, &aSchematic, tmpSheet.get(), &loadProps );
        }

        if( !pi->GetError().IsEmpty() )
        {
            msg = _( "The entire schematic could not be loaded.  Errors occurred attempting "
                     "to load hierarchical sheet schematics." );

            if( !aHandler.Continue( _( "Schematic Load Error" ), msg, pi->GetError(),
                                    _( "Use partial schematic" ) ) )
            {
                return false;
            }
        }
    }
    catch( const IO_ERROR& ioe )
    {
        msg.Printf( _( "Error loading schematic '%s'." ), fullFilename );
        aHandler.Error( msg, ioe.What() );
        aHandler.LoadFailed( fullFilename );

        return false;
    }

    // If the loaded schematic is in a different folder from the current project and
    // it contains hierarchical sheets, the hierarchical sheet paths need to be updated.
    //
    // Additionally, we need to make all backing screens absolute paths be in the current project
    // path not the source path.
    if( fileName.GetPathWithSep() != aSchematic.Project().GetProjectPath() )
    {
        SCH_SHEET_LIST loadedSheets( tmpSheet.get() );

        for( const SCH_SHEET_PATH& sheetPath : loadedSheets )
        {
            // Skip the loaded sheet since the user already determined if the file path should
            // be relative or absolute.
            if( sheetPath.size() == 1 )
                continue;

            wxString lastSheetPath = aSchematic.Project().GetProjectPath();

            for( unsigned i = 1; i < sheetPath.size(); i++ )
            {
                SCH_SHEET* sheet = sheetPath.at( i );
                wxCHECK2( sheet, continue );

                SCH_SCREEN* screen = sheet->GetScreen();
                wxCHECK2( screen, continue );

                // Use the screen file name which should always be absolute.
                wxFileName loadedSheetFileName = screen->GetFileName();
                wxCHECK2( loadedSheetFileName.IsAbsolute(), continue );

                wxFileName tmp = loadedSheetFileName;
                wxString sheetFileName;

                if( tmp.MakeRelativeTo( lastSheetPath ) )
                    sheetFileName = tmp.GetFullPath();
                else
                    sheetFileName = loadedSheetFileName.GetFullPath();

                sheetFileName.Replace( wxT( "\\" ), wxT( "/" ) );
                sheet->SetFileName( sheetFileName );
                lastSheetPath = loadedSheetFileName.GetPath();
            }
        }
    }

    SYMBOL_LIBRARY_ADAPTER* adapter = PROJECT_SCH::SymbolLibAdapter( &aSchematic.Project() );
    LIBRARY_TABLE* projectTable = adapter->ProjectTable().value_or( nullptr );

    SCH_SHEET_LIST loadedSheets( tmpSheet.get() );
    aSchematic.RefreshHierarchy();
    SCH_SHEET_LIST schematicSheets = aSchematic.Hierarchy();

    // Make sure any new sheet changes do not cause any recursion issues.
    if( !aSkipRecursionCheck )
    {
        wxString destFilePath = aCurrentSheet->LastScreen()->GetFileName();

        // An unsaved destination cannot be part of a recursion yet
        if( !destFilePath.IsEmpty() && schematicSheets.TestForRecursion( loadedSheets, destFilePath ) )
        {
            msg.Printf( _( "The sheet changes cannot be made because the destination sheet already "
                           "has the sheet '%s' or one of its subsheets as a parent somewhere in the "
                           "schematic hierarchy." ),
                        destFilePath );
            aHandler.Error( msg, wxEmptyString );
            return false;
        }
    }

    if( SCH_SCREENS( tmpSheet.get() ).HasNoFullyDefinedLibIds() )
    {
        msg.Printf( _( "The schematic '%s' has not had its symbol library links remapped "
                       "to the symbol library table.  The project this schematic belongs to "
                       "must first be remapped before it can be imported into the current "
                       "project." ),
                    tmpSheet->GetScreen()->GetFileName() );
        aHandler.Info( msg );
        return false;
    }

    // Make a valiant attempt to warn the user of all possible scenarios where there could
    // be broken symbol library links.
    wxArrayString    names;
    wxArrayString    newLibNames;
    SCH_SCREENS      newScreens( tmpSheet.get() );   // All screens associated with the import.
    SCH_SCREENS      prjScreens( &aSchematic.Root() );

    newScreens.GetLibNicknames( names );

    const wxString continueLabel = _( "Continue Load" );
    const wxString continueTitle = _( "Continue Load Schematic" );

    // Prior to schematic file format 20221002, all symbol instance data was saved in the root
    // sheet so loading a hierarchical sheet that is not the root sheet will have no symbol
    // instance data.  Give the user a chance to go back and save the project that contains this
    // hierarchical sheet so the symbol instance data will be correct on load.
    if( ( tmpSheet->GetScreen()->GetFileFormatVersionAtLoad() < 20221002 )
      && tmpSheet->GetScreen()->GetSymbolInstances().empty() )
    {
        msg = _( "There are hierarchical sheets in the loaded schematic file from an older "
                 "file version resulting in  missing symbol instance data.  This will "
                 "result in all of the symbols in the loaded schematic to use either the "
                 "default instance setting or fall back to the library symbol settings.  "
                 "Loading the project that uses this schematic file and saving to the "
                 "latest file version will resolve this issue.\n\n"
                 "Do you wish to continue?" );
        if( !aHandler.Continue( continueTitle, msg, wxEmptyString, continueLabel ) )
            return false;
    }

    if( !aSkipLibCheck && !prjScreens.HasSchematic( fullFilename ) )
    {
        if( fileName.GetPathWithSep() == aSchematic.Project().GetProjectPath() )
        {
            // A schematic in the current project path that isn't part of the current project.
            // It's possible the user copied this schematic from another project so the library
            // links may not be available.  Even this is check is no guarantee that all symbol
            // library links are valid but it's better than nothing.
            for( const wxString& name : names )
            {
                if( !PROJECT_SCH::SymbolLibAdapter( &aSchematic.Project() )->HasLibrary( name ) )
                    newLibNames.Add( name );
            }

            if( !newLibNames.IsEmpty() )
            {
                msg = _( "There are library names in the selected schematic that are missing "
                         "from the current project library table.  This may result in broken "
                         "symbol library references for the loaded schematic.\n\n"
                         "Do you wish to continue?" );
                if( !aHandler.Continue( continueTitle, msg, wxEmptyString, continueLabel ) )
                    return false;
            }
        }
        else if( fileName.GetPathWithSep() != aSchematic.Project().GetProjectPath() && projectTable )
        {
            // A schematic loaded from a path other than the current project path.

            // If there are symbol libraries in the imported schematic that are not in the
            // symbol library table of this project, there could be a lot of broken symbol
            // library links.  Attempt to add the missing libraries to the project symbol
            // library table.
            wxArrayString    duplicateLibNames;

            for( const wxString& name : names )
            {
                if( !PROJECT_SCH::SymbolLibAdapter( &aSchematic.Project() )->HasLibrary( name ) )
                    newLibNames.Add( name );
                else
                    duplicateLibNames.Add( name );
            }

            wxFileName symLibTableFn( fileName.GetPath(), FILEEXT::SymbolLibraryTableFileName );
            LIBRARY_TABLE table( symLibTableFn, LIBRARY_TABLE_SCOPE::PROJECT );

            // If there are any new or duplicate libraries, check to see if it's possible that
            // there could be any missing libraries that would cause broken symbol library links.
            if( !newLibNames.IsEmpty() || !duplicateLibNames.IsEmpty() )
            {
                if( !symLibTableFn.Exists() || !symLibTableFn.IsFileReadable() )
                {
                    msg = _( "The selected file was created as part of a different project.  "
                             "Linking the file to this project may result in missing or "
                             "incorrect symbol library references.\n\n"
                             "Do you wish to continue?" );
                    if( !aHandler.Continue( continueTitle, msg, wxEmptyString, continueLabel ) )
                        return false;
                }
                else
                {
                    if( !table.IsOk() )
                    {
                        msg.Printf( _( "Error loading the symbol library table '%s'." ),
                                    symLibTableFn.GetFullPath() );
                        aHandler.Error( msg, table.ErrorDescription() );
                        return false;
                    }
                }
            }

            // Check to see if any of the symbol libraries found in the appended schematic do
            // not exist in the current project are missing from the appended project symbol
            // library table.
            if( !newLibNames.IsEmpty() )
            {
                bool missingLibNames = table.Rows().empty();

                if( !missingLibNames )
                {
                    for( const wxString& newLibName : newLibNames )
                    {
                        if( !table.HasRow( newLibName ) )
                        {
                            missingLibNames = true;
                            break;
                        }
                    }
                }

                if( missingLibNames )
                {
                    msg = _( "There are symbol library names in the selected schematic that "
                             "are missing from the selected schematic project library table.  "
                             "This may result in broken symbol library references.\n\n"
                             "Do you wish to continue?" );
                    if( !aHandler.Continue( continueTitle, msg, wxEmptyString, continueLabel ) )
                        return false;
                }
            }

            // The library name already exists in the current project.  Check to see if the
            // duplicate name is the same library in the current project.  If it's not, it's
            // most likely that the symbol library links will be broken.
            if( !duplicateLibNames.IsEmpty() && !table.Rows().empty() )
            {
                bool libNameConflict = false;

                for( const wxString& duplicateLibName : duplicateLibNames )
                {
                    const LIBRARY_TABLE_ROW* thisRow = nullptr;
                    const LIBRARY_TABLE_ROW* otherRow = nullptr;

                    if( adapter->HasLibrary( duplicateLibName ) )
                        thisRow = *adapter->GetRow( duplicateLibName );

                    if( table.HasRow( duplicateLibName ) )
                        otherRow = *table.Row( duplicateLibName );

                    // It's in the global library table so there is no conflict.
                    if( thisRow && !otherRow )
                        continue;

                    if( !thisRow || !otherRow )
                        continue;

                    wxFileName otherUriFileName;
                    wxString thisURI = LIBRARY_MANAGER::GetFullURI( thisRow, true );;
                    wxString otherURI = LIBRARY_MANAGER::GetFullURI( otherRow, false );

                    if( otherURI.Contains( "${KIPRJMOD}" ) || otherURI.Contains( "$(KIPRJMOD)" ) )
                    {
                        // Cannot use relative paths here, "${KIPRJMOD}../path-to-cache-lib" does
                        // not expand to a valid symbol library path.
                        otherUriFileName.SetPath( fileName.GetPath() );
                        otherUriFileName.SetFullName( otherURI.AfterLast( '}' ) );
                        otherURI = otherUriFileName.GetFullPath();
                    }

                    if( thisURI != otherURI )
                    {
                        libNameConflict = true;
                        break;
                    }
                }

                if( libNameConflict )
                {
                    msg = _( "A duplicate library name that references a different library exists "
                             "in the current library table.  This conflict cannot be resolved and "
                             "may result in broken symbol library references.\n\n"
                             "Do you wish to continue?" );
                    if( !aHandler.Continue( continueTitle, msg, wxEmptyString, continueLabel ) )
                        return false;
                }
            }

            // All (most?) of the possible broken symbol library link cases are covered.  Map the
            // new appended schematic project symbol library table entries to the current project
            // symbol library table.
            if( !newLibNames.IsEmpty() && !table.Rows().empty() )
            {
                for( const wxString& libName : newLibNames )
                {
                    if( !table.HasRow( libName ) || adapter->HasLibrary( libName ) )
                    {
                        continue;
                    }

                    LIBRARY_TABLE_ROW* row = *table.Row( libName );

                    // Don't expand environment variable because KIPRJMOD will not be correct
                    // for a different project.
                    wxString uri = LIBRARY_MANAGER::GetFullURI( row, false );
                    wxFileName newLib;

                    if( uri.Contains( "${KIPRJMOD}" ) || uri.Contains( "$(KIPRJMOD)" ) )
                    {
                        // Cannot use relative paths here, "${KIPRJMOD}../path-to-cache-lib" does
                        // not expand to a valid symbol library path.
                        newLib.SetPath( fileName.GetPath() );
                        newLib.SetFullName( uri.AfterLast( '}' ) );
                        uri = newLib.GetFullPath();
                    }
                    else
                    {
                        uri = LIBRARY_MANAGER::GetFullURI( row, true );
                    }

                    // Add the library from the imported project to the current project
                    // symbol library table.

                    LIBRARY_TABLE_ROW& newRow = projectTable->InsertRow();

                    newRow.SetNickname( libName );
                    newRow.SetURI( uri );
                    newRow.SetType( row->Type() );
                    newRow.SetDescription( row->Description() );
                    newRow.SetOptions( row->Options() );

                    libTableChanged = true;
                }
            }
        }
    }

    SCH_SCREEN* newScreen = tmpSheet->GetScreen();
    wxCHECK_MSG( newScreen, false, "No screen defined for sheet." );

    if( libTableChanged && projectTable )
    {
        projectTable->Save().map_error(
                [&]( const LIBRARY_ERROR& aError )
                {
                    aHandler.Error( _( "Error saving library table." ), aError.message );
                } );
    }

    // Make the best attempt to set the symbol instance data for the loaded schematic.
    if( newScreen->GetFileFormatVersionAtLoad() < 20221002 )
    {
        if( !newScreen->GetSymbolInstances().empty() )
        {
            // If the loaded schematic is a root sheet for another project, update the symbol
            // instances.
            loadedSheets.UpdateSymbolInstanceData( newScreen->GetSymbolInstances());
        }
    }

    newScreen->MigrateSimModels();

    // Attempt to create new symbol instances using the instance data loaded above.
    loadedSheets.AddNewSymbolInstances( *aCurrentSheet, aSchematic.Project().GetProjectName() );

    // Add new sheet instance data.
    loadedSheets.AddNewSheetInstances( *aCurrentSheet, schematicSheets.GetLastVirtualPageNumber() );

    // It is finally safe to add or append the imported schematic.
    if( aSheet->GetScreen() == nullptr )
    {
        aSheet->SetScreen( newScreen );
    }
    else
    {
        // Appended items that repeat identities already in the schematic (a design block placed
        // where it was saved from, or twice) take new ones, with their pins and fields, so the
        // items already there keep theirs and no two pins share one
        std::unordered_set<KIID> taken;
        SCH_SCREENS              existingScreens( &aSchematic.Root() );

        for( SCH_SCREEN* screen = existingScreens.GetFirst(); screen; screen = existingScreens.GetNext() )
        {
            for( SCH_ITEM* item : screen->Items() )
            {
                taken.insert( item->m_Uuid );
                item->RunOnChildren( [&]( SCH_ITEM* aChild ) { taken.insert( aChild->m_Uuid ); },
                                     RECURSE_MODE::NO_RECURSE );
            }
        }

        for( SCH_ITEM* item : newScreen->Items() )
        {
            if( item->Type() == SCH_SHEET_T )
                continue;

            bool renew = taken.contains( item->m_Uuid );

            item->RunOnChildren( [&]( SCH_ITEM* aChild ) { renew |= taken.contains( aChild->m_Uuid ); },
                                 RECURSE_MODE::NO_RECURSE );

            if( !renew )
                continue;

            const_cast<KIID&>( item->m_Uuid ) = KIID();
            item->RunOnChildren( []( SCH_ITEM* aChild ) { const_cast<KIID&>( aChild->m_Uuid ) = KIID(); },
                                 RECURSE_MODE::NO_RECURSE );
        }

        aSheet->GetScreen()->Append( newScreen );
    }

    SCH_SCREENS allProjectScreens( &aSchematic.Root() );
    allProjectScreens.ReplaceDuplicateTimeStamps();

    return true;
}
