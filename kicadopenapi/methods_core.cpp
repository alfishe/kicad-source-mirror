/// @file methods_core.cpp
/// @brief Core kicadopenapi methods available in every host (GUI and headless).
#include "kicadopenapi_registry.h"
#include "kicadopenapi_journal.h"
#include "kicadopenapi_service.h"
#include "platform/platform.h"

#include <eda_base_frame.h>
#include <frame_type.h>
#include <pgm_base.h>
#include <settings/kicad_settings.h>
#include <settings/settings_manager.h>
#include <kiway.h>
#include <kiway_player.h>
#include <project.h>

#include <wx/toplevel.h>
#include <wx/app.h>
#include <wx/log.h>
#include <wx/stdpaths.h>
#include <wx/utils.h>

#include <algorithm>
#include <cstring>
#include <typeinfo>


static KOPENAPI_RESULT h_ping( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    // Reaching here proves the main thread is responsive (the service marshals every call)
    return KOPENAPI_RESULT::Ok( { { "pong", true }, { "headless", aCtx.headless } } );
}


KOPENAPI_REGISTER( "ping", "Round trip through the main thread (liveness and latency check)",
                   R"json({"type":"object","properties":{}})json"_json, false, h_ping );


static KOPENAPI_RESULT h_documents( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    return KOPENAPI_RESULT::Ok( { { "documents", KOPENAPI_REGISTRY::Get().Documents( aCtx ) } } );
}


KOPENAPI_REGISTER( "documents", "List open documents (schematic, PCB) with path, project and unsaved state",
                   R"json({"type":"object","properties":{}})json"_json, false, h_documents );


static KOPENAPI_RESULT h_errors( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    const uint64_t    since = aArgs.value( "since", static_cast<uint64_t>( 0 ) );
    const size_t      limit = std::clamp( aArgs.value( "limit", 100 ), 1, 1000 );
    const std::string level = aArgs.value( "level", std::string( "warning" ) );

    nlohmann::json out = KOPENAPI_JOURNAL::Summary();
    out["entries"] = KOPENAPI_JOURNAL::Entries( since, limit, level );

    if( aArgs.value( "clear", false ) )
        KOPENAPI_JOURNAL::Clear();

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "errors",
                   "Errors and warnings KiCad reported (never shown as dialogs): operation that caused "
                   "them, message, time, source location; poll with since=last_seq",
                   R"json({"type":"object","properties":{
                        "since":{"type":"integer","default":0,"description":"Only records with seq greater than this"},
                        "limit":{"type":"integer","default":100,"minimum":1,"maximum":1000},
                        "level":{"type":"string","enum":["warning","error"],"default":"warning",
                                 "description":"Minimum severity"},
                        "clear":{"type":"boolean","default":false,"description":"Drop collected records after reading"}}})json"_json,
                   false, h_errors );


/// @brief Quit the GUI process without any dialog.  Unsaved documents block unless discard: true; then
/// each document is closed through its domain's *_close {discard: true} (a journal warning each)
/// and the top window closes after this answer has gone out.  Library editors with unsaved
/// library changes always block: KiCad can only ask about those, and a question would hang the
/// agent.
static KOPENAPI_RESULT h_app_quit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const bool     discard = aArgs.value( "discard", false );
    nlohmann::json documents = KOPENAPI_REGISTRY::Get().Documents( aCtx );
    nlohmann::json unsaved = nlohmann::json::array();

    for( const nlohmann::json& doc : documents )
    {
        if( doc.value( "unsaved", false ) )
            unsaved.push_back( doc );
    }

    nlohmann::json libraries = nlohmann::json::array();

    for( const auto& [type, name] : { std::pair{ FRAME_SCH_SYMBOL_EDITOR, "symbol editor" },
                                      std::pair{ FRAME_FOOTPRINT_EDITOR, "footprint editor" } } )
    {
        if( KIWAY_PLAYER* editor = aCtx.kiway ? aCtx.kiway->Player( type, false ) : nullptr;
            editor && editor->IsContentModified() )
        {
            libraries.push_back( name );
        }
    }

    if( !libraries.empty() )
    {
        return KOPENAPI_RESULT::Error( 409, "library editors have unsaved library changes (" + libraries.dump()
                                                    + "): save or close them first" );
    }

    if( !unsaved.empty() && !discard )
        return KOPENAPI_RESULT::Error( 409, "unsaved documents: " + unsaved.dump() + "; save them or pass discard: true" );

    nlohmann::json closed = nlohmann::json::array();

    for( const nlohmann::json& doc : documents )
    {
        const std::string domain = doc.value( "domain", std::string() );

        if( std::optional<KOPENAPI_METHOD> close = KOPENAPI_REGISTRY::Get().Find( domain + "_close" ) )
        {
            KOPENAPI_RESULT result = close->handler( aCtx, { { "discard", true } } );

            if( result.status == 200 )
                closed.push_back( doc );
        }
    }

    // After the answer has been written: closing the top window ends the process
    wxTheApp->CallAfter(
            []()
            {
                if( wxWindow* top = wxTheApp->GetTopWindow() )
                    top->Close( true );
            } );

    return KOPENAPI_RESULT::Ok( { { "quitting", true }, { "closed", closed }, { "discarded", discard ? unsaved : nlohmann::json::array() } } );
}


KOPENAPI_REGISTER( "app_quit",
                   "Quit KiCad (GUI) without any dialog: refuses while documents are unsaved unless "
                   "discard: true, which drops the changes (journal warnings) and closes the editors; "
                   "library editors with unsaved library changes always refuse. Headless: POST "
                   "/api/v1/shutdown (unsaved work is lost with a journal warning)",
                   R"json({"type":"object","properties":{"discard":{"type":"boolean","default":false}}})json"_json,
                   true, h_app_quit );


/// @brief The project manager window when this process has one (project mode), else nullptr
static EDA_BASE_FRAME* projectManagerFrame( KOPENAPI_CONTEXT& aCtx )
{
    if( aCtx.headless || !aCtx.kiway )
        return nullptr;

    // static_cast as in COMMON_CONTROL::ShowProjectManager: the top is always an EDA_BASE_FRAME
    EDA_BASE_FRAME* top = static_cast<EDA_BASE_FRAME*>( aCtx.kiway->GetTop() );

    return top && top->GetFrameType() == KICAD_MAIN_FRAME_T ? top : nullptr;
}


static KOPENAPI_RESULT h_app_restart( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_BASE_FRAME* manager = projectManagerFrame( aCtx );

    // what is open now: documents (schematic first), else the project; extra editor windows
    std::vector<std::string> files;
    std::vector<std::string> editors;

    if( aArgs.value( "reopen", true ) )
    {
        for( const char* domain : { "sch", "pcb" } )
        {
            for( const nlohmann::json& doc : KOPENAPI_REGISTRY::Get().Documents( aCtx ) )
            {
                std::string path = doc.value( "path", std::string() );

                if( doc.value( "domain", std::string() ) == domain && !path.empty()
                    && std::find( files.begin(), files.end(), path ) == files.end() )
                {
                    files.push_back( path );
                }
            }
        }

        if( files.empty() && aCtx.kiway && !aCtx.kiway->Prj().GetProjectFullName().IsEmpty() )
            files.push_back( aCtx.kiway->Prj().GetProjectFullName().ToStdString( wxConvUTF8 ) );

        // the 3D viewer is not in the kiway's player table: found by its type among the windows
        for( wxWindow* window : wxTopLevelWindows )
        {
            if( window->IsShown() && std::strstr( typeid( *window ).name(), "EDA_3D_VIEWER_FRAME" ) )
            {
                editors.push_back( "3d" );
                break;
            }
        }

        for( const auto& [type, name] : { std::pair{ FRAME_SCH_SYMBOL_EDITOR, "symbol_editor" },
                                          std::pair{ FRAME_FOOTPRINT_EDITOR, "footprint_editor" } } )
        {
            if( KIWAY_PLAYER* player = aCtx.kiway ? aCtx.kiway->Player( type, false ) : nullptr;
                player && player->IsShown() )
            {
                editors.push_back( name );
            }
        }
    }

    // the caller's set replaces the captured one
    if( aArgs.contains( "documents" ) && aArgs["documents"].is_array() )
    {
        files.clear();

        for( const nlohmann::json& path : aArgs["documents"] )
        {
            if( path.is_string() )
                files.push_back( path.get<std::string>() );
        }
    }

    if( aArgs.contains( "editors" ) && aArgs["editors"].is_array() )
    {
        editors.clear();

        for( const nlohmann::json& name : aArgs["editors"] )
        {
            if( name.is_string() )
                editors.push_back( name.get<std::string>() );
        }
    }

    std::string managerMode = aArgs.value( "manager", std::string( "keep" ) );

    if( managerMode == "keep" )
        managerMode = manager && !manager->IsShown() ? "hide" : "show";

    if( managerMode != "show" && managerMode != "hide" )
        return KOPENAPI_RESULT::Error( 400, "manager: show, hide or keep" );

    const std::string exe = wxStandardPaths::Get().GetExecutablePath().ToStdString( wxConvUTF8 );
    const int         port = KICAD_OPENAPI_SERVICE::CurrentPort();

    // same checks as app_quit (unsaved documents, library editors) before anything starts
    KOPENAPI_RESULT quit = h_app_quit( aCtx, aArgs );

    if( quit.status != 200 )
        return quit;

    // the successor waits for this process to exit, then takes the same port
    std::vector<std::string> argv = { exe };

    if( manager )
    {
        std::string list;

        for( const std::string& name : editors )
            list += ( list.empty() ? "" : "," ) + name;

        argv.insert( argv.end(), { "--manager", managerMode, "--editors", list.empty() ? "none" : list } );
        argv.insert( argv.end(), files.begin(), files.end() );
    }
    else if( !files.empty() )
    {
        // a stand-alone editor opens one document
        argv.push_back( files.front() );
    }

    wxSetEnv( wxS( "KICAD_OPENAPI_WAIT_PID" ), wxString::Format( wxS( "%ld" ), kopenapi::platform::CurrentPid() ) );
    wxSetEnv( wxS( "KICAD_OPENAPI_AGENT" ), wxS( "1" ) );   // restarted for an agent: no modal questions

    // the successor announces itself once these documents are open again (or its opening is done)
    wxSetEnv( wxS( "KICAD_OPENAPI_REOPEN" ), wxString::FromUTF8( nlohmann::json( files ).dump() ) );

    if( port > 0 )
        wxSetEnv( wxS( "KICAD_OPENAPI_PORT" ), wxString::Format( wxS( "%d" ), port ) );

    std::string error;
    const auto  logDir = kopenapi::platform::TempRoot() / "kicad" / "openapi-logs";
    const long  pid = kopenapi::platform::SpawnDetached( argv, logDir / "restart.log", error );

    wxUnsetEnv( wxS( "KICAD_OPENAPI_WAIT_PID" ) );
    wxUnsetEnv( wxS( "KICAD_OPENAPI_REOPEN" ) );

    nlohmann::json answer = quit.body;
    answer["restarting"] = pid != 0;
    answer["new_pid"] = pid;
    answer["reopen"] = files.empty() ? std::string() : files.front();
    answer["documents"] = files;
    answer["editors"] = manager ? nlohmann::json( editors ) : nlohmann::json::array();
    answer["manager"] = manager ? nlohmann::json( managerMode ) : nlohmann::json( nullptr );
    answer["port"] = port;

    if( !pid )
        answer["error"] = error;

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "app_restart",
                   "Restart KiCad (GUI) to pick up a new build: quits like app_quit (refuses on unsaved "
                   "documents unless discard: true), starts the same executable again on the same API "
                   "port and reopens exactly what was open (schematic, board, 3D viewer, symbol / footprint "
                   "editor, project manager window shown or hidden) or the set given in documents / editors / "
                   "manager; paths that no longer exist are reported in the new process's journal (errors). "
                   "The new process is announced (instance list, bridge) only once its documents are open",
                   R"json({"type":"object","properties":{
                        "discard":{"type":"boolean","default":false},
                        "reopen":{"type":"boolean","default":true,
                                  "description":"false: start with nothing open (documents / editors still apply)"},
                        "documents":{"type":"array","items":{"type":"string"},
                                     "description":"replaces the open set: .kicad_sch / .kicad_pcb / .kicad_pro paths of one project ([] opens none)"},
                        "editors":{"type":"array","items":{"type":"string",
                                   "enum":["sch","pcb","3d","symbol_editor","footprint_editor"]},
                                   "description":"replaces the extra windows: sch / pcb open the project's main schematic / board"},
                        "manager":{"type":"string","enum":["keep","show","hide"],"default":"keep",
                                   "description":"project manager window after the restart"}}})json"_json,
                   true, h_app_restart );


static nlohmann::json projectManagerSettingsJson( const KICAD_SETTINGS& aSettings )
{
    return { { "show_on_start", KICAD_SETTINGS::ToString( aSettings.m_ProjectManager.show_on_start ) },
             { "open_project_shows", KICAD_SETTINGS::ToString( aSettings.m_ProjectManager.open_project_shows ) },
             { "quit_with_last_editor", KICAD_SETTINGS::ToString( aSettings.m_ProjectManager.quit_with_last_editor ) } };
}


static KOPENAPI_RESULT h_app_project_manager( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_BASE_FRAME* manager = projectManagerFrame( aCtx );
    KICAD_SETTINGS* settings = Pgm().GetSettingsManager().GetAppSettings<KICAD_SETTINGS>( "kicad" );

    if( aArgs.contains( "settings" ) )
    {
        if( !settings )
            return KOPENAPI_RESULT::Error( 503, "no KiCad settings in this process" );

        const nlohmann::json&                       in = aArgs["settings"];
        KICAD_SETTINGS::PROJECT_MANAGER_BEHAVIOUR   next = settings->m_ProjectManager;
        bool                                        ok = in.is_object();

        if( ok && in.contains( "show_on_start" ) )
            ok = in["show_on_start"].is_string() && KICAD_SETTINGS::FromString( in["show_on_start"].get<std::string>(), next.show_on_start );

        if( ok && in.contains( "open_project_shows" ) )
            ok = in["open_project_shows"].is_string() && KICAD_SETTINGS::FromString( in["open_project_shows"].get<std::string>(), next.open_project_shows );

        if( ok && in.contains( "quit_with_last_editor" ) )
            ok = in["quit_with_last_editor"].is_string() && KICAD_SETTINGS::FromString( in["quit_with_last_editor"].get<std::string>(), next.quit_with_last_editor );

        if( !ok )
        {
            return KOPENAPI_RESULT::Error( 400, "settings: show_on_start always|without_document|never, open_project_shows "
                                                "manager|editors, quit_with_last_editor when_hidden|always|never" );
        }

        settings->m_ProjectManager = next;
        settings->SaveToFile( Pgm().GetSettingsManager().GetPathForSettingsFile( settings ) );
    }

    if( aArgs.contains( "show" ) )
    {
        if( !manager )
            return KOPENAPI_RESULT::Error( 409, "no project manager window in this process (headless or stand-alone editor)" );

        if( aArgs.value( "show", true ) )
        {
            manager->Iconize( false );
            manager->Show( true );
            manager->Raise();
        }
        else
        {
            manager->Show( false );
        }
    }

    wxString       env;
    nlohmann::json answer = { { "available", manager != nullptr },
                              { "shown", manager && manager->IsShown() },
                              { "env_override", wxGetEnv( wxS( "KICAD_PROJECT_MANAGER" ), &env )
                                                        ? nlohmann::json( env.ToStdString( wxConvUTF8 ) )
                                                        : nlohmann::json( nullptr ) } };

    if( settings )
        answer["settings"] = projectManagerSettingsJson( *settings );

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "app_project_manager",
                   "Project manager (launcher) window: show or hide it (show: true / false; KiCad keeps running "
                   "hidden, closing the last editor then quits unless quit_with_last_editor is never) and read / "
                   "set its persistent settings (kicad.json project_manager: show_on_start always | "
                   "without_document | never, open_project_shows manager | editors, quit_with_last_editor "
                   "when_hidden | always | never; env KICAD_PROJECT_MANAGER=show|hide overrides show_on_start)",
                   R"json({"type":"object","properties":{
                        "show":{"type":"boolean","description":"true shows and raises the window, false hides it"},
                        "settings":{"type":"object","properties":{
                            "show_on_start":{"type":"string","enum":["always","without_document","never"]},
                            "open_project_shows":{"type":"string","enum":["manager","editors"]},
                            "quit_with_last_editor":{"type":"string","enum":["when_hidden","always","never"]}},
                            "description":"changes are saved to kicad.json at once"}}})json"_json,
                   false, h_app_project_manager );


static KOPENAPI_RESULT h_api_trace( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    // the service answers api_trace itself, on the HTTP thread (see invokeParsed)
    return KOPENAPI_RESULT::Error( 500, "api_trace is served by the service" );
}


KOPENAPI_REGISTER( "api_trace",
                   "Trace of the API gate (every REST / MCP call passes it), off until asked (no cost "
                   "then): enabled: true starts it; the last calls (method, "
                   "when, arguments, time queued for the main thread, time running there, status, error) "
                   "and the call holding the main thread now; answers even while KiCad is busy. Kept in a "
                   "buffer (buffer_bytes, default 1 MB) and optionally appended to a file as JSON lines "
                   "(file; null stops). Env KICAD_OPENAPI_TRACE=1 also logs every call to the journal",
                   R"json({"type":"object","properties":{
                        "enabled":{"type":"boolean","description":"true starts tracing, false stops it and frees the buffer"},
                        "limit":{"type":"integer","default":50},
                        "with_args":{"type":"boolean","default":false},
                        "buffer_bytes":{"type":"integer","description":"set the buffer size (default 1048576)"},
                        "file":{"type":["string","null"],"description":"append every call to this file (JSON lines); null: stop"},
                        "method":{"type":"string","description":"glob on the method name"},
                        "slower_ms":{"type":"number","description":"only calls that took longer (queued + run)"}}})json"_json,
                   false, h_api_trace );
