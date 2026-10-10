/// @file methods_pcb_documents.cpp
/// @brief kicadopenapi PCB document methods: pcb_open, pcb_close, pcb_save, pcb_revert, and the
/// "pcb" document provider.  Compiled into the pcbnew kiface; registered when it loads.
#include <kicadopenapi_lock.h>
#include <kicadopenapi_service.h>
#include "kopenapi_pcb.h"

#include <wx/log.h>

#include <api/headless_pcb_context.h>
#include <api/pcb_context.h>
#include <board.h>
#include <board_loader.h>
#include <kiway.h>
#include <tool/tool_manager.h>
#include <tool/actions.h>
#include <kicadopenapi_history.h>
#include <pcb_edit_frame.h>
#include <pcb_io/pcb_io_mgr.h>
#include <pcbnew_settings.h>
#include <pgm_base.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>


/// @brief Headless board; GUI hosts use the editor frame instead
static std::shared_ptr<HEADLESS_PCB_CONTEXT> s_headless;


static PCB_EDIT_FRAME* guiFrame( KOPENAPI_CONTEXT& aCtx, bool aCreate )
{
    if( aCtx.headless || !aCtx.kiway )
        return nullptr;

    return static_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, aCreate ) );
}


std::shared_ptr<PCB_CONTEXT> KopenapiPcbContext( KOPENAPI_CONTEXT& aCtx )
{
    if( aCtx.headless )
        return s_headless;

    PCB_EDIT_FRAME* frame = guiFrame( aCtx, false );

    if( !frame )
        return nullptr;

    std::shared_ptr<PCB_CONTEXT> context = CreatePcbFrameContext( frame );
    return context->GetCurrentFileName().IsEmpty() ? nullptr : context;
}


KOPENAPI_RESULT KopenapiNoBoard()
{
    return KOPENAPI_RESULT::Error( 409, "no board is open (pcb_open)" );
}


static nlohmann::json documentStatus( KOPENAPI_CONTEXT& aCtx )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return nullptr;

    return { { "domain", "pcb" },
             { "path", context->GetCurrentFileName().ToStdString() },
             { "project", context->Prj().GetProjectFullName().ToStdString() },
             { "unsaved", context->IsContentModified() },
             { "mode", aCtx.headless ? "headless" : "gui" } };
}


/// @brief Another open document of this process belongs to a different project
static std::string projectConflict( KOPENAPI_CONTEXT& aCtx, const wxFileName& aProject )
{
    for( const nlohmann::json& doc : KOPENAPI_REGISTRY::Get().Documents( aCtx ) )
    {
        if( doc.value( "domain", "" ) == "pcb" )
            continue;

        wxFileName other( wxString::FromUTF8( doc.value( "project", "" ) ) );

        if( !other.GetFullPath().IsEmpty() && !other.SameAs( aProject ) )
            return doc.value( "project", "" );
    }

    return {};
}


static KOPENAPI_RESULT openHeadless( KOPENAPI_CONTEXT& aCtx, const wxFileName& aBoard )
{
    wxFileName projectFile( aBoard );
    projectFile.SetExt( FILEEXT::ProjectFileExtension );

    if( std::string other = projectConflict( aCtx, projectFile ); !other.empty() )
    {
        return KOPENAPI_RESULT::Error( 409, "this process serves project " + other
                                                    + "; start another instance for a different project" );
    }

    // Drop the current board first: activating another project destroys the old PROJECT,
    // which the old board still references (unsaved changes were checked by the caller)
    s_headless.reset();

    SETTINGS_MANAGER& settings = Pgm().GetSettingsManager();
    PROJECT*          project = settings.GetProject( projectFile.GetFullPath() );

    if( !project )
    {
        settings.LoadProject( projectFile.GetFullPath(), true );
        project = settings.GetProject( projectFile.GetFullPath() );
    }

    if( !project )
        return KOPENAPI_RESULT::Error( 500, "could not load the project for this board" );

    PCB_IO_MGR::PCB_FILE_T type =
            PCB_IO_MGR::FindPluginTypeFromBoardPath( aBoard.GetFullPath(), KICTL_KICAD_ONLY );

    if( type == PCB_IO_MGR::FILE_TYPE_NONE )
        return KOPENAPI_RESULT::Error( 415, "not a KiCad board file" );

    std::unique_ptr<BOARD> board;

    try
    {
        board = BOARD_LOADER::Load( aBoard.GetFullPath(), type, project );
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 422, "board could not be loaded: " + ioe.What().ToStdString() );
    }

    if( !board )
        return KOPENAPI_RESULT::Error( 422, "board could not be loaded" );

    s_headless = std::make_shared<HEADLESS_PCB_CONTEXT>(
            std::move( board ), project, GetAppSettings<PCBNEW_SETTINGS>( "pcbnew" ), aCtx.kiway );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT openGui( KOPENAPI_CONTEXT& aCtx, const wxFileName& aBoard )
{
    PCB_EDIT_FRAME* frame = guiFrame( aCtx, true );

    if( !frame )
        return KOPENAPI_RESULT::Error( 500, "could not create the PCB editor window" );

    // another process holds it (or its project) open: refused here, never asked in a dialog
    if( !KICAD_OPENAPI_SERVICE::OverrideLock() && frame->GetCurrentFileName() != aBoard.GetFullPath() )
    {
        if( const std::string owner = KopenapiLockedBy( aBoard ); !owner.empty() )
        {
            return KOPENAPI_RESULT::Error( 409, "the board or its project is open in another process (" + owner
                                                        + "): close it there, or open with override_lock: true" );
        }
    }

    if( !frame->OpenProjectFiles( { aBoard.GetFullPath() } ) )
        return KOPENAPI_RESULT::Error( 422, "the PCB editor could not open the board" );

    // OpenProjectFiles does not show the frame; mirror KICAD_MANAGER_CONTROL::ShowPlayer
    frame->Iconize( false );
    frame->Show( true );
    frame->Raise();

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT h_pcb_open( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_HISTORY::Get().Reset( "pcb", "pcb_open" );   // earlier marks belong to another document state

    if( !aArgs["path"].is_string() || aArgs["path"].get<std::string>().empty() )
        return KOPENAPI_RESULT::Error( 400, "'path' must be a non-empty string" );

    wxFileName board( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
    board.MakeAbsolute();

    if( board.GetExt() == FILEEXT::ProjectFileExtension )
        board.SetExt( FILEEXT::KiCadPcbFileExtension );

    if( board.GetExt() != FILEEXT::KiCadPcbFileExtension )
        return KOPENAPI_RESULT::Error( 400, "expected a .kicad_pcb or .kicad_pro file" );

    if( !board.FileExists() )
        return KOPENAPI_RESULT::Error( 404, "file not found: " + board.GetFullPath().ToStdString() );

    // Never let a "save changes?" dialog or a silent drop happen: unsaved work blocks
    if( std::shared_ptr<PCB_CONTEXT> current = KopenapiPcbContext( aCtx ) )
    {
        if( current->IsContentModified() )
        {
            if( !aArgs.value( "discard", false ) )
                return KOPENAPI_RESULT::Error( 409, "the open board has unsaved changes; pcb_save, "
                                                    "pcb_revert, or pass discard: true" );

            current->SetContentModified( false );
        }
    }

    if( aCtx.headless )
        return openHeadless( aCtx, board );

    // a file another process has open: opened only when asked (no modal question)
    KICAD_OPENAPI_SERVICE::SetOverrideLock( aArgs.value( "override_lock", false ) );
    KOPENAPI_RESULT result = openGui( aCtx, board );
    KICAD_OPENAPI_SERVICE::SetOverrideLock( false );

    if( result.status != 200 )
        result.body["hint"] = "another process may have the file open (errors: journal); override_lock: true opens it anyway";

    return result;
}


static KOPENAPI_RESULT h_pcb_close( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_HISTORY::Get().Reset( "pcb", "pcb_close" );   // earlier marks belong to another document state

    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( context->IsContentModified() )
    {
        if( !aArgs.value( "discard", false ) )
            return KOPENAPI_RESULT::Error( 409, "the board has unsaved changes; pcb_save or pass discard: true" );

        wxLogWarning( "Unsaved changes to %s discarded (pcb_close discard)", context->GetCurrentFileName() );
        context->SetContentModified( false );
    }

    nlohmann::json closed = documentStatus( aCtx );

    if( aCtx.headless )
        s_headless.reset();
    else if( PCB_EDIT_FRAME* frame = guiFrame( aCtx, false ) )
        frame->Close( true );

    return KOPENAPI_RESULT::Ok( { { "closed", closed } } );
}


static KOPENAPI_RESULT h_pcb_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    bool ok = false;

    if( aArgs.contains( "path" ) && aArgs["path"].is_string() )
    {
        wxFileName target( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
        target.MakeAbsolute();

        if( target.GetExt() != FILEEXT::KiCadPcbFileExtension )
            return KOPENAPI_RESULT::Error( 400, "expected a .kicad_pcb path" );

        // Save As switches the document (headless); GUI hosts write a copy and keep editing
        ok = aCtx.headless ? context->SaveBoardAs( target.GetFullPath() )
                           : context->SavePcbCopy( target.GetFullPath(), false, false );
    }
    else
    {
        ok = context->SaveBoard();
    }

    if( !ok )
        return KOPENAPI_RESULT::Error( 500, "saving the board failed" );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT h_pcb_revert( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    KOPENAPI_HISTORY::Get().Reset( "pcb", "pcb_revert" );   // earlier marks belong to another document state

    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( !context->RevertToSaved() )
        return KOPENAPI_RESULT::Error( 422, "reverting failed (no saved file?)" );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


// ---- edit history (kicadopenapi_history.h) --------------------------------------------------

static KOPENAPI_DOC_HISTORY history()
{
    KOPENAPI_DOC_HISTORY h;

    h.undoDepth = []( KOPENAPI_CONTEXT& aCtx ) -> std::optional<int>
    {
        PCB_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame || !KopenapiPcbContext( aCtx ) )
            return std::nullopt;

        return frame->GetUndoCommandCount();
    };

    h.undoTo = []( KOPENAPI_CONTEXT& aCtx, int aDepth )
    {
        PCB_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame )
            return false;

        while( frame->GetUndoCommandCount() > aDepth )
        {
            const int before = frame->GetUndoCommandCount();
            frame->GetToolManager()->RunAction( ACTIONS::undo );

            if( frame->GetUndoCommandCount() >= before )
                return false;   // nothing undone: stop rather than loop
        }

        return frame->GetUndoCommandCount() == aDepth;
    };

    h.snapshot = []( KOPENAPI_CONTEXT& aCtx, const std::string& aDir )
    {
        if( !aCtx.headless || !s_headless )
            return false;

        const wxFileName target( wxString::FromUTF8( aDir ), wxFileName( s_headless->GetCurrentFileName() ).GetFullName() );
        return s_headless->SavePcbCopy( target.GetFullPath(), false, true );
    };

    h.restore = []( KOPENAPI_CONTEXT& aCtx, const std::string& aDir )
    {
        if( !aCtx.headless || !s_headless )
            return false;

        const wxString   original = s_headless->GetCurrentFileName();
        const wxFileName copy( wxString::FromUTF8( aDir ), wxFileName( original ).GetFullName() );
        PROJECT*         project = s_headless->GetBoard()->GetProject();
        std::unique_ptr<BOARD> board;

        try
        {
            board = BOARD_LOADER::Load( copy.GetFullPath(), PCB_IO_MGR::KICAD_SEXP, project );
        }
        catch( const IO_ERROR& )
        {
            return false;
        }

        if( !board )
            return false;

        // Back under the document's own path, unsaved
        board->SetFileName( original );
        s_headless.reset();
        s_headless = std::make_shared<HEADLESS_PCB_CONTEXT>( std::move( board ), project,
                                                             GetAppSettings<PCBNEW_SETTINGS>( "pcbnew" ), aCtx.kiway );
        s_headless->SetContentModified( true );
        return true;
    };

    return h;
}


KOPENAPI_REGISTER_HISTORY( "pcb", history() );

KOPENAPI_REGISTER_DOCUMENTS( "pcb", documentStatus,
                             []()
                             {
                                 // Stopping a headless server with unsaved work loses it: say so
                                 if( s_headless && s_headless->IsContentModified() )
                                     wxLogWarning( "Unsaved changes to %s discarded (headless server released the board)",
                                                   s_headless->GetCurrentFileName() );

                                 s_headless.reset();
                             } );

KOPENAPI_REGISTER( "pcb_open",
                   "Open a board (.kicad_pcb or its .kicad_pro): editor window in the GUI, in memory headless",
                   R"json({"type":"object","required":["path"],"properties":{
                        "path":{"type":"string"},
                        "discard":{"type":"boolean","default":false,
                                   "description":"Drop unsaved changes of the currently open board"},
                        "override_lock":{"type":"boolean","default":false,
                                   "description":"GUI: open even when another process has the file open (never asked in a dialog; refused with a journal error otherwise)"}}})json"_json,
                   false, h_pcb_open, 120 );

KOPENAPI_REGISTER( "pcb_close", "Close the open board: refuses with unsaved changes unless discard: true, which drops "
                   "them without any dialog (a warning goes to the journal); GUI: the editor window closes",
                   R"json({"type":"object","properties":{"discard":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_close );

KOPENAPI_REGISTER( "pcb_save",
                   "Save the open board; with 'path' save as (headless switches to it, GUI writes a copy)",
                   R"json({"type":"object","properties":{"path":{"type":"string"}}})json"_json, false, h_pcb_save );

KOPENAPI_REGISTER( "pcb_revert", "Reload the open board from disk, dropping unsaved changes",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_revert, 120 );
