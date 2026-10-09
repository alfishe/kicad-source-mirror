/*
 * Document window methods for kicadopenapi (GUI only).
 *
 * Open/close go through the KIWAY player path — the same one the project manager uses — so
 * the real editor window opens.  Handlers run on the main thread (the service marshals).
 */
#include "kicadopenapi_registry.h"

#include <kiway.h>
#include <kiway_player.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>


static const nlohmann::json PATH_SCHEMA = R"({
    "type": "object",
    "required": [ "path" ],
    "properties": {
        "path": { "type": "string", "description": "Absolute or cwd-relative file path" }
    }
})"_json;


/// Resolves "path" to an absolute file name; empty on bad input.
static wxString absolutePath( const nlohmann::json& aArgs )
{
    if( !aArgs["path"].is_string() || aArgs["path"].get<std::string>().empty() )
        return wxEmptyString;

    wxFileName fn( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
    fn.MakeAbsolute();
    return fn.GetFullPath();
}


static KOPENAPI_RESULT openInEditor( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs,
                                     FRAME_T aFrame, const wxString& aExt )
{
    wxString path = absolutePath( aArgs );

    if( path.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "'path' must be a non-empty string" );

    if( wxFileName( path ).GetExt() != aExt )
        return KOPENAPI_RESULT::Error( 400, "expected a ." + aExt.ToStdString() + " file" );

    if( !wxFileName::FileExists( path ) )
        return KOPENAPI_RESULT::Error( 404, "file not found: " + path.ToStdString() );

    KIWAY_PLAYER* player = aCtx.kiway->Player( aFrame, true );

    if( !player )
        return KOPENAPI_RESULT::Error( 500, "could not create the editor window" );

    if( !player->OpenProjectFiles( { path } ) )
        return KOPENAPI_RESULT::Error( 422, "editor could not open the document" );

    // OpenProjectFiles does not show the frame; mirror KICAD_MANAGER_CONTROL::ShowPlayer
    player->Iconize( false );
    player->Show( true );
    player->Raise();

    return KOPENAPI_RESULT::Ok( { { "path", path.ToStdString() }, { "opened", true } } );
}


static KOPENAPI_RESULT h_open_schematic( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return openInEditor( aCtx, aArgs, FRAME_SCH, FILEEXT::KiCadSchematicFileExtension );
}


static KOPENAPI_RESULT h_open_pcb( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return openInEditor( aCtx, aArgs, FRAME_PCB_EDITOR, FILEEXT::KiCadPcbFileExtension );
}


static KOPENAPI_RESULT h_close_document( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    wxString path = absolutePath( aArgs );

    if( path.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "'path' must be a non-empty string" );

    wxString ext = wxFileName( path ).GetExt();
    FRAME_T  frame;

    if( ext == FILEEXT::KiCadSchematicFileExtension )
        frame = FRAME_SCH;
    else if( ext == FILEEXT::KiCadPcbFileExtension )
        frame = FRAME_PCB_EDITOR;
    else
        return KOPENAPI_RESULT::Error( 400, "expected a .kicad_sch or .kicad_pcb file" );

    KIWAY_PLAYER* player = aCtx.kiway->Player( frame, false );

    if( !player || !wxFileName( player->GetCurrentFileName() ).SameAs( wxFileName( path ) ) )
        return KOPENAPI_RESULT::Error( 404, "document is not open: " + path.ToStdString() );

    if( !player->Close( true ) )
        return KOPENAPI_RESULT::Error( 409, "editor refused to close" );

    return KOPENAPI_RESULT::Ok( { { "path", path.ToStdString() }, { "closed", true } } );
}


KOPENAPI_REGISTER( "open_schematic", "Open a .kicad_sch in the schematic editor window",
                   PATH_SCHEMA, true, h_open_schematic );

KOPENAPI_REGISTER( "open_pcb", "Open a .kicad_pcb in the PCB editor window",
                   PATH_SCHEMA, true, h_open_pcb );

KOPENAPI_REGISTER( "close_document", "Close the editor window showing a .kicad_sch/.kicad_pcb",
                   PATH_SCHEMA, true, h_close_document );
