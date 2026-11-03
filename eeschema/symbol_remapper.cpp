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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <symbol_remapper.h>

#include <core/kicad_algo.h>
#include <env_paths.h>
#include <libraries/legacy_symbol_library.h>
#include <libraries/library_manager.h>
#include <libraries/symbol_library_adapter.h>
#include <pgm_base.h>
#include <project.h>
#include <project_sch.h>
#include <reporter.h>
#include <sch_io/sch_io_mgr.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <wildcards_and_files_ext.h>
#include <wx_filename.h>

#include <wx/datetime.h>
#include <wx/filefn.h>
#include <wx/intl.h>


SYMBOL_REMAPPER::SYMBOL_REMAPPER( PROJECT& aProject, SCHEMATIC& aSchematic, bool aQuiet ) :
        m_project( aProject ),
        m_schematic( aSchematic ),
        m_quiet( aQuiet )
{
}


const wxString& SYMBOL_REMAPPER::BackupFolder()
{
    static const wxString folder = wxS( "rescue-backup" );
    return folder;
}


wxString SYMBOL_REMAPPER::CheckWritable() const
{
    wxString projectPath = m_project.GetProjectPath();

    if( wxFileName::IsDirWritable( projectPath ) )
        return wxEmptyString;

    return wxString::Format( _( "Remapping is not possible because you have insufficient privileges to the "
                                "project folder '%s'." ),
                             projectPath );
}


LEGACY_SYMBOL_LIBS* SYMBOL_REMAPPER::legacyLibs()
{
    return PROJECT_SCH::LegacySchLibs( &m_project, m_quiet ? &m_loadErrors : nullptr );
}


void SYMBOL_REMAPPER::Remap( REPORTER& aReporter )
{
    // The schematic is fully loaded, any legacy library symbols have been rescued.  Now
    // check to see if the schematic has not been converted to the symbol library table
    // method for looking up symbols.

    wxFileName prjSymLibTableFileName( m_project.GetProjectPath(), FILEEXT::SymbolLibraryTableFileName );

    // Delete the existing project symbol library table.
    if( prjSymLibTableFileName.FileExists() )
        wxRemoveFile( prjSymLibTableFileName.GetFullPath() );

    createProjectSymbolLibTable( aReporter );

    remapSymbolsToLibTable( aReporter );

    // Remove all of the libraries from the legacy library list.
    wxString paths;
    wxArrayString libNames;

    LEGACY_SYMBOL_LIBS::SetLibNamesAndPaths( &m_project, paths, libNames );

    // Reload the cache symbol library.
    m_project.SetElem( PROJECT::ELEM::LEGACY_SYMBOL_LIBS, nullptr );
    legacyLibs();
}


size_t SYMBOL_REMAPPER::getLibsNotInGlobalSymbolLibTable( std::vector< LEGACY_SYMBOL_LIB* >& aLibs )
{
    LIBRARY_MANAGER& mgr = Pgm().GetLibraryManager();

    for( LEGACY_SYMBOL_LIB& lib : *legacyLibs() )
    {
        // Ignore the cache library.
        if( lib.IsCache() )
            continue;

        // Check for the obvious library name.
        wxString libFileName = lib.GetFullFileName();

        if( !mgr.FindRowByURI( LIBRARY_TABLE_TYPE::SYMBOL, libFileName ) )
            aLibs.push_back( &lib );
    }

    return aLibs.size();
}


void SYMBOL_REMAPPER::createProjectSymbolLibTable( REPORTER& aReporter )
{
    SYMBOL_LIBRARY_ADAPTER* adapter = PROJECT_SCH::SymbolLibAdapter( &m_project );
    LIBRARY_MANAGER&        manager = Pgm().GetLibraryManager();

    std::optional<LIBRARY_TABLE*> optTable = manager.Table( LIBRARY_TABLE_TYPE::SYMBOL, LIBRARY_TABLE_SCOPE::PROJECT );
    wxCHECK( optTable, /* void */ );
    LIBRARY_TABLE* projectTable = *optTable;

    std::vector<LEGACY_SYMBOL_LIB*> libs;

    if( getLibsNotInGlobalSymbolLibTable( libs ) )
    {
        std::vector<wxString> libNames = adapter->GetLibraryNames();

        for( LEGACY_SYMBOL_LIB* lib : libs )
        {
            wxString libName = lib->GetName();
            int libNameInc = 1;
            int libNameLen = libName.Length();

            // Spaces in the file name will break the symbol name because they are not
            // quoted in the symbol library file format.
            libName.Replace( wxS( " " ), wxS( "-" ) );

            // Don't create duplicate table entries.
            while( alg::contains( libNames, libName ) )
            {
                libName = libName.Left( libNameLen );
                libName << libNameInc;
                libNameInc++;
            }

            wxString   type = SCH_IO_MGR::ShowType( SCH_IO_MGR::SCH_LEGACY );
            wxFileName fn( lib->GetFullFileName() );

            // Use environment variable substitution where possible.  This is based solely
            // on the internal user environment variable list.  Checking against all of the
            // system wide environment variables is probably not a good idea.
            wxString   normalizedPath = NormalizePath( fn, &Pgm().GetLocalEnvVariables(), &m_project );
            wxFileName normalizedFn( normalizedPath );

            // Don't add symbol libraries that do not exist.
            if( normalizedFn.Normalize(FN_NORMALIZE_FLAGS | wxPATH_NORM_ENV_VARS )
                    && normalizedFn.FileExists() )
            {
                aReporter.Report( wxString::Format( _( "Adding library '%s', file '%s' to project "
                                                       "symbol library table." ),
                                                    libName,
                                                    normalizedPath ),
                                  RPT_SEVERITY_INFO );

                LIBRARY_TABLE_ROW& newRow = projectTable->InsertRow();
                newRow.SetNickname( libName );
                newRow.SetURI( normalizedPath );
                newRow.SetType( type );
                m_addedLibraries.push_back( libName );
            }
            else
            {
                aReporter.Report( wxString::Format( _( "Library '%s' not found." ), normalizedPath ),
                                  RPT_SEVERITY_WARNING );
            }
        }

        // Don't save empty project symbol library table.
        if( !projectTable->Rows().empty() )
        {
            projectTable->Save().map_error(
                    [&aReporter]( const LIBRARY_ERROR& aError )
                    {
                        aReporter.ReportTail( _( "Error saving library table:\n\n" ) + aError.message );
                    } );

            // Trigger a reload of the table and cancel an in-progress background load
            Pgm().GetLibraryManager().ProjectChanged();

            aReporter.ReportTail( _( "Created project symbol library table.\n" ), RPT_SEVERITY_INFO );
        }
    }
}


void SYMBOL_REMAPPER::remapSymbolsToLibTable( REPORTER& aReporter )
{
    SCH_SCREENS  schematic( m_schematic.Root() );
    SCH_SYMBOL*  symbol;
    SCH_SCREEN*  screen;

    for( screen = schematic.GetFirst(); screen; screen = schematic.GetNext() )
    {
        for( EDA_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            symbol = dynamic_cast<SCH_SYMBOL*>( item );

            wxCHECK2( symbol, continue );

            if( !remapSymbolToLibTable( symbol ) )
            {
                m_unmapped.push_back( symbol );

                aReporter.Report( wxString::Format( _( "No symbol %s found in symbol library "
                                                       "table." ),
                                                    symbol->GetLibId().GetLibItemName().wx_str() ),
                                  RPT_SEVERITY_WARNING );
            }
            else
            {
                aReporter.Report( wxString::Format( _( "Symbol %s mapped to symbol library '%s'." ),
                                            symbol->GetLibId().GetLibItemName().wx_str(),
                                            symbol->GetLibId().GetLibNickname().wx_str() ),
                                  RPT_SEVERITY_ACTION );

                m_remapped.push_back( symbol );
                screen->SetContentModified();
            }
        }
    }

    aReporter.Report( _( "Symbol library table mapping complete!" ), RPT_SEVERITY_INFO );
    schematic.UpdateSymbolLinks();
}


bool SYMBOL_REMAPPER::remapSymbolToLibTable( SCH_SYMBOL* aSymbol )
{
    wxCHECK_MSG( aSymbol != nullptr, false, "Null pointer passed to remapSymbolToLibTable." );
    wxCHECK_MSG( aSymbol->GetLibId().GetLibNickname().empty(), false,
                 "Cannot remap symbol that is already mapped." );
    wxCHECK_MSG( !aSymbol->GetLibId().GetLibItemName().empty(), false,
                 "The symbol LIB_ID name is empty." );

    for( LEGACY_SYMBOL_LIB& lib : *legacyLibs() )
    {
        // Ignore the cache library.
        if( lib.IsCache() )
            continue;

        LIB_SYMBOL* alias = lib.FindSymbol( aSymbol->GetLibId().GetLibItemName().wx_str() );

        // Found in the same library as the old look up method assuming the user didn't
        // change the libraries or library ordering since the last time the schematic was
        // loaded.
        if( alias )
        {
            // Find the same library in the symbol library table using the full path and file name.
            wxString libFileName = lib.GetFullFileName();

            if( std::optional<wxString> nickname =
                    PROJECT_SCH::SymbolLibAdapter( &m_project )->FindLibraryByURI( libFileName ) )
            {
                LIB_ID id = aSymbol->GetLibId();

                id.SetLibNickname( *nickname );

                // Don't resolve symbol library links now.
                aSymbol->SetLibId( id );
                return true;
            }
        }
    }

    return false;
}


bool SYMBOL_REMAPPER::BackupProject( REPORTER& aReporter, wxString& aErrors )
{
    wxString    errorMsg;
    wxFileName  srcFileName;
    wxFileName  destFileName;
    wxFileName  backupPath;
    SCH_SCREENS schematic( m_schematic.Root() );

    // Copy backup files to different folder so as not to pollute the project folder.
    destFileName.SetPath( m_project.GetProjectPath() );
    destFileName.AppendDir( BackupFolder() );
    backupPath = destFileName;

    if( !destFileName.DirExists() )
    {
        if( !destFileName.Mkdir() )
        {
            aErrors.Printf( _( "Cannot create project remap back up folder '%s'." ), destFileName.GetPath() );
            return false;
        }
    }

    {
        // Time stamp to append to file name in case multiple remappings are performed.
        wxString timeStamp = wxDateTime::Now().Format( "-%Y-%m-%d-%H-%M-%S" );

        // Back up symbol library table.
        srcFileName.SetPath( m_project.GetProjectPath() );
        srcFileName.SetName( FILEEXT::SymbolLibraryTableFileName );
        destFileName = srcFileName;
        destFileName.AppendDir( BackupFolder() );
        destFileName.SetName( destFileName.GetName() + timeStamp );

        aReporter.Report( wxString::Format( _( "Backing up file '%s' to '%s'." ),
                                            srcFileName.GetFullPath(),
                                            destFileName.GetFullPath() ),
                          RPT_SEVERITY_INFO );

        if( wxFileName::Exists( srcFileName.GetFullPath() )
                && !wxCopyFile( srcFileName.GetFullPath(), destFileName.GetFullPath() ) )
        {
            errorMsg += wxString::Format( _( "Failed to back up file '%s'.\n" ),
                                          srcFileName.GetFullPath() );
        }

        // Back up the schematic files.
        for( SCH_SCREEN* screen = schematic.GetFirst(); screen; screen = schematic.GetNext() )
        {
            destFileName = screen->GetFileName();
            destFileName.SetName( destFileName.GetName() + timeStamp );

            // Check for nest hierarchical schematic paths.
            if( destFileName.GetPath() != backupPath.GetPath() )
            {
                destFileName.SetPath( backupPath.GetPath() );

                wxArrayString srcDirs = wxFileName( screen->GetFileName() ).GetDirs();
                wxArrayString destDirs = wxFileName( m_project.GetProjectPath() ).GetDirs();

                for( size_t i = destDirs.GetCount(); i < srcDirs.GetCount(); i++ )
                    destFileName.AppendDir( srcDirs[i] );
            }
            else
            {
                destFileName.AppendDir( BackupFolder() );
            }

            aReporter.Report( wxString::Format( _( "Backing up file '%s' to '%s'." ),
                                                screen->GetFileName(),
                                                destFileName.GetFullPath() ),
                              RPT_SEVERITY_INFO );

            if( !destFileName.DirExists()
             && !destFileName.Mkdir( wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
            {
                errorMsg += wxString::Format( _( "Failed to create backup folder '%s'.\n" ),
                                              destFileName.GetPath() );
                continue;
            }

            if( wxFileName::Exists( screen->GetFileName() )
                    && !wxCopyFile( screen->GetFileName(), destFileName.GetFullPath() ) )
            {
                errorMsg += wxString::Format( _( "Failed to back up file '%s'.\n" ),
                                              screen->GetFileName() );
            }
        }

        // Back up the project file.
        destFileName = m_project.GetProjectFullName();
        destFileName.SetName( destFileName.GetName() + timeStamp );
        destFileName.AppendDir( BackupFolder() );

        aReporter.Report( wxString::Format( _( "Backing up file '%s' to '%s'." ),
                                            m_project.GetProjectFullName(),
                                            destFileName.GetFullPath() ),
                          RPT_SEVERITY_INFO );

        if( wxFileName::Exists( m_project.GetProjectFullName() )
                && !wxCopyFile( m_project.GetProjectFullName(), destFileName.GetFullPath() ) )
        {
            errorMsg += wxString::Format( _( "Failed to back up file '%s'.\n" ),
                                          m_project.GetProjectFullName() );
        }

        // Back up the cache library.
        srcFileName.SetPath( m_project.GetProjectPath() );
        srcFileName.SetName( m_project.GetProjectName() + wxS( "-cache" ) );
        srcFileName.SetExt( FILEEXT::LegacySymbolLibFileExtension );

        destFileName = srcFileName;
        destFileName.SetName( destFileName.GetName() + timeStamp );
        destFileName.AppendDir( BackupFolder() );

        aReporter.Report( wxString::Format( _( "Backing up file '%s' to '%s'." ),
                                            srcFileName.GetFullPath(),
                                            destFileName.GetFullPath() ),
                          RPT_SEVERITY_INFO );

        if( srcFileName.Exists()
                && !wxCopyFile( srcFileName.GetFullPath(), destFileName.GetFullPath() ) )
        {
            errorMsg += wxString::Format( _( "Failed to back up file '%s'.\n" ),
                                          srcFileName.GetFullPath() );
        }

        // Back up the old (2007) cache library.
        srcFileName.SetPath( m_project.GetProjectPath() );
        srcFileName.SetName( m_project.GetProjectName() + wxS( ".cache" ) );
        srcFileName.SetExt( FILEEXT::LegacySymbolLibFileExtension );

        destFileName = srcFileName;
        destFileName.SetName( destFileName.GetName() + timeStamp );
        destFileName.AppendDir( BackupFolder() );

        aReporter.Report( wxString::Format( _( "Backing up file '%s' to '%s'." ),
                                            srcFileName.GetFullPath(),
                                            destFileName.GetFullPath() ),
                          RPT_SEVERITY_INFO );

        if( srcFileName.Exists()
                && !wxCopyFile( srcFileName.GetFullPath(), destFileName.GetFullPath() ) )
        {
            errorMsg += wxString::Format( _( "Failed to back up file '%s'.\n" ),
                                          srcFileName.GetFullPath() );
        }

        // Back up the rescue symbol library if it exists.
        srcFileName.SetName( m_project.GetProjectName() + wxS( "-rescue" ) );
        destFileName.SetName( srcFileName.GetName() + timeStamp );

        aReporter.Report( wxString::Format( _( "Backing up file '%s' to '%s'." ),
                                            srcFileName.GetFullPath(),
                                            destFileName.GetFullPath() ),
                          RPT_SEVERITY_INFO );

        if( srcFileName.Exists()
                && !wxCopyFile( srcFileName.GetFullPath(), destFileName.GetFullPath() ) )
        {
            errorMsg += wxString::Format( _( "Failed to back up file '%s'.\n" ),
                                          srcFileName.GetFullPath() );
        }

        // Back up the rescue symbol library document file if it exists.
        srcFileName.SetExt( FILEEXT::LegacySymbolDocumentFileExtension );
        destFileName.SetExt( srcFileName.GetExt() );

        aReporter.Report( wxString::Format( _( "Backing up file '%s' to '%s'." ),
                                            srcFileName.GetFullPath(),
                                            destFileName.GetFullPath() ),
                          RPT_SEVERITY_INFO );

        if( srcFileName.Exists()
                && !wxCopyFile( srcFileName.GetFullPath(), destFileName.GetFullPath() ) )
        {
            errorMsg += wxString::Format( _( "Failed to back up file '%s'.\n" ),
                                          srcFileName.GetFullPath() );
        }
    }

    aErrors = errorMsg.Trim();
    return true;
}
