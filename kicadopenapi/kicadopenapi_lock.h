/// @file kicadopenapi_lock.h
/// @brief Who else holds a document or its project open (KiCad's lock files), checked before an
/// editor opens it so an API call refuses cleanly instead of KiCad asking in a dialog.
#ifndef KICADOPENAPI_LOCK_H
#define KICADOPENAPI_LOCK_H

#include <lockfile.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>

#include <string>


/// @brief "user@host" of another process holding aDocument or its project open, else empty.
/// aOwnProject: the project this process has open (its lock is ours, not another's)
inline std::string KopenapiLockedBy( const wxFileName& aDocument, const wxString& aOwnProject = wxEmptyString )
{
    wxFileName project( aDocument );
    project.SetExt( FILEEXT::ProjectFileExtension );

    for( const wxFileName& file : { project, aDocument } )
    {
        if( !file.FileExists() )
            continue;

        if( !aOwnProject.IsEmpty() && wxFileName( aOwnProject ).SameAs( project ) && file == project )
            continue;

        LOCKFILE lock = LOCKFILE::Inspect( file.GetFullPath() );

        if( !lock.Valid() )
            return ( lock.GetUsername() + wxS( "@" ) + lock.GetHostname() ).ToStdString( wxConvUTF8 );
    }

    return {};
}

#endif
