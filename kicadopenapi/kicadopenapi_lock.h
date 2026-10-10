/// @file kicadopenapi_lock.h
/// @brief Who else holds a document or its project open (KiCad's lock files), checked before an
/// editor opens it so an API call refuses cleanly instead of KiCad asking in a dialog.
#ifndef KICADOPENAPI_LOCK_H
#define KICADOPENAPI_LOCK_H

#include <lockfile.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>

#include <string>


/// @brief "user@host" of another process holding aDocument or its project open, else empty
inline std::string KopenapiLockedBy( const wxFileName& aDocument )
{
    wxFileName project( aDocument );
    project.SetExt( FILEEXT::ProjectFileExtension );

    for( const wxFileName& file : { project, aDocument } )
    {
        if( !file.FileExists() )
            continue;

        LOCKFILE lock = LOCKFILE::Inspect( file.GetFullPath() );

        if( !lock.Valid() )
            return ( lock.GetUsername() + wxS( "@" ) + lock.GetHostname() ).ToStdString( wxConvUTF8 );
    }

    return {};
}

#endif
