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

#include <sch_autosave_recovery.h>

#include <project.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <schematic.h>
#include <wildcards_and_files_ext.h>

#include <wx/filefn.h>
#include <wx/filename.h>
#include <wx/intl.h>
#include <wx/utils.h>

#include <algorithm>
#include <map>


std::vector<AUTOSAVE_FILE_INFO> FindSchematicAutosaves( LOCAL_HISTORY& aHistory, const SCHEMATIC& aSchematic )
{
    std::vector<AUTOSAVE_FILE_INFO> found;

    if( !aSchematic.IsValid() || aSchematic.GetFileName().IsEmpty() )
        return found;

    const wxString projectPath = aSchematic.Project().GetProjectPath();

    if( projectPath.IsEmpty() )
        return found;

    found = aHistory.ListAutosaveFiles( projectPath, { FILEEXT::KiCadSchematicFileExtension } );

    std::sort( found.begin(), found.end(),
               []( const AUTOSAVE_FILE_INFO& a, const AUTOSAVE_FILE_INFO& b )
               {
                   return a.autosaveTime.IsLaterThan( b.autosaveTime );
               } );

    return found;
}


static wxString normalized( const wxString& aPath )
{
    wxFileName fn( aPath );
    fn.Normalize( wxPATH_NORM_DOTS | wxPATH_NORM_ABSOLUTE | wxPATH_NORM_TILDE );
    return fn.GetFullPath();
}


wxString StageSchematicAutosaves( const SCHEMATIC& aSchematic, const std::vector<AUTOSAVE_FILE_INFO>& aAutosaves,
                                  const wxString& aDir, wxString& aError )
{
    const wxFileName root( aSchematic.GetFileName() );

    if( !aSchematic.IsValid() || !root.IsAbsolute() )
    {
        aError = _( "The schematic has no file." );
        return wxEmptyString;
    }

    // source file -> the file its copy is made from
    std::map<wxString, wxString> sources;
    SCH_SCREENS                  screens( aSchematic.Root() );

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        if( !screen->GetFileName().IsEmpty() )
            sources[normalized( screen->GetFileName() )] = screen->GetFileName();
    }

    for( const AUTOSAVE_FILE_INFO& info : aAutosaves )
        sources[normalized( info.sourcePath )] = info.autosavePath;

    // sheets above the root's directory ("../x"): the root goes as deep under aDir
    int up = 0;

    for( const auto& [source, from] : sources )
    {
        wxFileName rel( source );
        rel.MakeRelativeTo( root.GetPath() );

        int depth = 0;

        for( const wxString& dir : rel.GetDirs() )
        {
            if( dir != wxS( ".." ) )
                break;

            depth++;
        }

        up = std::max( up, depth );
    }

    wxFileName base = wxFileName::DirName( aDir );

    for( int i = 0; i < up; i++ )
        base.AppendDir( wxS( "_" ) );

    for( const auto& [source, from] : sources )
    {
        wxFileName target( source );
        target.MakeRelativeTo( root.GetPath() );
        target.MakeAbsolute( base.GetPath() );
        target.Normalize( wxPATH_NORM_DOTS | wxPATH_NORM_ABSOLUTE );

        if( !wxFileName::DirExists( target.GetPath() )
            && !wxFileName::Mkdir( target.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
        {
            aError = wxString::Format( _( "Cannot create the folder '%s'." ), target.GetPath() );
            return wxEmptyString;
        }

        // a sheet with neither a saved file nor a copy: its load reports it
        if( !wxFileExists( from ) )
            continue;

        if( !wxCopyFile( from, target.GetFullPath(), true ) )
        {
            aError = wxString::Format( _( "Cannot copy '%s' to '%s'." ), from, target.GetFullPath() );
            return wxEmptyString;
        }
    }

    wxFileName rootCopy( base.GetPath(), root.GetFullName() );
    return rootCopy.GetFullPath();
}


void RetargetSchematicFiles( SCHEMATIC& aSchematic, const wxString& aFromDir, const wxString& aToDir,
                             bool aMarkModified )
{
    SCH_SCREENS screens( aSchematic.Root() );

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        // the virtual root's screen has no file
        if( screen->GetFileName().IsEmpty() )
            continue;

        screen->SetFileName( MoveSchematicPath( screen->GetFileName(), aFromDir, aToDir ) );

        if( aMarkModified )
            screen->SetContentModified( true );
    }

    // the loader names top-level sheets relative to the project, from where it read them
    for( SCH_SHEET* sheet : aSchematic.GetTopLevelSheets() )
    {
        if( !sheet || !sheet->GetScreen() || sheet->GetScreen()->GetFileName().IsEmpty() )
            continue;

        wxFileName rel( sheet->GetScreen()->GetFileName() );
        rel.MakeRelativeTo( aSchematic.Project().GetProjectPath() );
        sheet->SetFileName( rel.GetFullPath() );
    }
}


wxString MoveSchematicPath( const wxString& aPath, const wxString& aFromDir, const wxString& aToDir )
{
    wxFileName fn( aPath );
    fn.MakeRelativeTo( aFromDir );
    fn.MakeAbsolute( aToDir );
    fn.Normalize( wxPATH_NORM_DOTS | wxPATH_NORM_ABSOLUTE );
    return fn.GetFullPath();
}


wxString MakeSchematicStagingDir()
{
    static int s_count = 0;

    for( int attempt = 0; attempt < 100; attempt++ )
    {
        wxFileName dir = wxFileName::DirName( wxFileName::GetTempDir() );
        dir.AppendDir( wxString::Format( wxS( "kicad-sch-rescue-%lu-%d" ), wxGetProcessId(), ++s_count ) );

        if( !dir.DirExists() && dir.Mkdir( wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
            return dir.GetPath();
    }

    return wxEmptyString;
}
