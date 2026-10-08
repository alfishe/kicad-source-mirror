/*
 * Project/document management methods for kicadopenapi (PGM + editor frames).
 * Open/close go through KIWAY::ProcessApiOpenDocument / ProcessApiCloseDocument —
 * the same path upstream's OpenDocument command uses — so the GUI opens the
 * proper editor window for the file.
 */
#include "kicadopenapi_registry.h"

#include <kiway.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>

static KIWAY::FACE_T faceForPath( const wxString& aPath )
{
    wxFileName fn( aPath );

    if( fn.GetExt() == FILEEXT::KiCadSchematicFileExtension )
        return KIWAY::FACE_SCH;
    if( fn.GetExt() == FILEEXT::KiCadPcbFileExtension )
        return KIWAY::FACE_PCB;
    if( fn.GetExt() == FILEEXT::ProjectFileExtension )
        return KIWAY::KIWAY_FACE_COUNT;  // .kicad_pro opens via the project manager

    return KIWAY::KIWAY_FACE_COUNT;
}


static std::string h_open_document( KOPENAPI_HOST& aHost, const std::string& aArgs )
{
    if( aArgs.find( "path" ) == std::string::npos )
        return "{\"error\":\"missing path\"}";

    // crude JSON string value extraction for the single "path" argument
    size_t key = aArgs.find( "\"path\"" );
    size_t colon = aArgs.find( ':', key );
    size_t q1 = aArgs.find( '"', colon );
    size_t q2 = aArgs.find( '"', q1 + 1 );
    std::string path = aArgs.substr( q1 + 1, q2 - q1 - 1 );

    KIWAY::FACE_T face = faceForPath( path );
    if( face == KIWAY::KIWAY_FACE_COUNT )
        return "{\"error\":\"unsupported file type\"}";

    KIFACE::DOCUMENT_SPEC spec;
    spec.kind = KIFACE::DOCUMENT_SPEC::KIND::FILE_KIND;
    spec.path = path;

    wxString error;

    if( !aHost.Ki()->ProcessApiOpenDocument( face, spec, nullptr, &error ) )
        return "{\"error\":\"" + error.ToStdString() + "\"}";

    return "{\"opened\":\"" + path + "\"}";
}


static std::string h_close_document( KOPENAPI_HOST& aHost, const std::string& aArgs )
{
    size_t key = aArgs.find( "\"path\"" );
    if( key == std::string::npos )
        return "{\"error\":\"missing path\"}";

    size_t colon = aArgs.find( ':', key );
    size_t q1 = aArgs.find( '"', colon );
    size_t q2 = aArgs.find( '"', q1 + 1 );
    std::string path = aArgs.substr( q1 + 1, q2 - q1 - 1 );

    KIWAY::FACE_T face = faceForPath( path );
    if( face == KIWAY::KIWAY_FACE_COUNT )
        return "{\"error\":\"unsupported file type\"}";

    KIFACE::DOCUMENT_SPEC spec;
    spec.kind = KIFACE::DOCUMENT_SPEC::KIND::FILE_KIND;
    spec.path = path;

    wxString error;

    if( !aHost.Ki()->ProcessApiCloseDocument( face, spec, nullptr, &error ) )
        return "{\"error\":\"" + error.ToStdString() + "\"}";

    return "{\"closed\":\"" + path + "\"}";
}


static std::string h_open_project( KOPENAPI_HOST& aHost, const std::string& aArgs )
{
    // .kicad_pro has no editor kiface; opening a project switches the PGM.
    (void) aHost;
    (void) aArgs;
    return "{\"error\":\"open .kicad_sch/.kicad_pcb instead; project switch runs in the "
           "project manager\"}";
}


KOPENAPI_REGISTER( "open_schematic", "Open a .kicad_sch in the schematic editor",
                   h_open_document );
KOPENAPI_REGISTER( "open_pcb", "Open a .kicad_pcb in the PCB editor", h_open_document );
KOPENAPI_REGISTER( "open_project", "Open a .kicad_pro project", h_open_project );
KOPENAPI_REGISTER( "close_document", "Close an open document by path", h_close_document );
