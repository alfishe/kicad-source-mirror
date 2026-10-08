/*
 * Project/document management methods for kicadopenapi (PGM + editor frames).
 * Document open/close go through the KIWAY player path — the same one the
 * project manager uses — so the real editor window opens in the GUI. Handlers
 * run on the webserver thread and marshal the GUI work to the main loop via
 * the host frame's CallAfter, waiting on a future for the result.
 */
#include "kicadopenapi_registry.h"

#include <kiway.h>
#include <kiway_player.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>

#include <chrono>
#include <future>

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


static bool parse_path_arg( const std::string& aArgs, std::string& aPath )
{
    size_t key = aArgs.find( "\"path\"" );

    if( key == std::string::npos )
        return false;

    size_t colon = aArgs.find( ':', key );
    size_t q1 = aArgs.find( '"', colon );
    size_t q2 = aArgs.find( '"', q1 + 1 );

    if( colon == std::string::npos || q1 == std::string::npos || q2 == std::string::npos )
        return false;

    aPath = aArgs.substr( q1 + 1, q2 - q1 - 1 );
    return true;
}


static std::string h_open_document( KOPENAPI_HOST& aHost, const std::string& aArgs )
{
    std::string path;

    if( !parse_path_arg( aArgs, path ) )
        return "{\"error\":\"missing path\"}";

    KIWAY::FACE_T face = faceForPath( path );

    if( face == KIWAY::KIWAY_FACE_COUNT )
        return "{\"error\":\"unsupported file type\"}";

    wxFileName fn( path );
    fn.MakeAbsolute();
    wxString abs = fn.GetFullPath();

    auto promise = std::make_shared<std::promise<bool>>();
    auto result = promise->get_future();

    aHost.Window()->CallAfter(
            [ki = aHost.Ki(), face, abs, promise]()
            {
                FRAME_T frame = ( face == KIWAY::FACE_SCH ) ? FRAME_SCH : FRAME_PCB_EDITOR;
                KIWAY_PLAYER* player = ki->Player( frame, true );

                if( !player )
                {
                    promise->set_value( false );
                    return;
                }

                // OpenProjectFiles does not show the player frame; mirror
                // KICAD_MANAGER_CONTROL::ShowPlayer so the editor becomes visible.
                bool opened = player->OpenProjectFiles( { abs } );

                if( opened )
                {
                    player->Iconize( false );
                    player->Show( true );
                    player->Raise();

                    if( wxWindow::FindFocus() != player )
                        player->SetFocus();
                }

                promise->set_value( opened );
            } );

    if( result.wait_for( std::chrono::seconds( 15 ) ) != std::future_status::ready )
        return "{\"error\":\"timed out opening document\"}";

    if( !result.get() )
        return "{\"error\":\"editor could not open the document\"}";

    return "{\"opened\":\"" + abs.ToStdString() + "\"}";
}


static std::string h_close_document( KOPENAPI_HOST& aHost, const std::string& aArgs )
{
    std::string path;

    if( !parse_path_arg( aArgs, path ) )
        return "{\"error\":\"missing path\"}";

    KIWAY::FACE_T face = faceForPath( path );

    if( face == KIWAY::KIWAY_FACE_COUNT )
        return "{\"error\":\"unsupported file type\"}";

    wxFileName fn( path );
    fn.MakeAbsolute();
    wxString abs = fn.GetFullPath();

    auto promise = std::make_shared<std::promise<bool>>();
    auto result = promise->get_future();

    aHost.Window()->CallAfter(
            [ki = aHost.Ki(), face, abs, promise]()
            {
                FRAME_T frame = ( face == KIWAY::FACE_SCH ) ? FRAME_SCH : FRAME_PCB_EDITOR;
                KIWAY_PLAYER* player = ki->Player( frame, false );

                if( !player )
                {
                    promise->set_value( false );
                    return;
                }

                bool closed = player->Close( true );
                promise->set_value( closed );
            } );

    if( result.wait_for( std::chrono::seconds( 15 ) ) != std::future_status::ready )
        return "{\"error\":\"timed out closing document\"}";

    if( !result.get() )
        return "{\"error\":\"no open editor for that document\"}";

    return "{\"closed\":\"" + abs.ToStdString() + "\"}";
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
