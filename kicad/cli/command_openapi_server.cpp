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

#include "command_openapi_server.h"
#include <string_utils.h>
#include <kicadopenapi_registry.h>
#include <wx/filename.h>
#include <wildcards_and_files_ext.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

#include <cli/exit_codes.h>
#include <kicadopenapi_journal.h>
#include <kicadopenapi_service.h>
#include <wx/app.h>
#include <wx/crt.h>
#include <wx/log.h>
#include <wx/process.h>

#define ARG_PATH "path"
#define ARG_PORT "--port"
#define ARG_PARENT_PID "--parent-pid"


/// Upper bound for noticing signals and a dead parent while idle; requests wake the loop at once
static constexpr std::chrono::milliseconds IDLE_WAIT( 100 );

static std::atomic_bool g_openApiExitRequested{ false };
static std::atomic_int  g_openApiStopSignals{ 0 };

static void openApiStopRequested()
{
    g_openApiStopSignals.fetch_add( 1 );
    g_openApiExitRequested.store( true );
}


/// How long a stop waits for the running call to reach a safe point (KICAD_OPENAPI_STOP_GRACE_S)
static std::chrono::milliseconds stopGrace()
{
    const char*  env = std::getenv( "KICAD_OPENAPI_STOP_GRACE_S" );
    const double sec = env ? std::atof( env ) : 0;
    return std::chrono::milliseconds( static_cast<long long>( ( sec > 0 ? sec : 10 ) * 1000 ) );
}


/// @brief Stop() with a deadline: past the grace period, or on another stop signal, the process
/// exits without waiting for the call that does not stop (the journal says which)
static void stopWithin( KICAD_OPENAPI_SERVICE& aService, std::chrono::milliseconds aGrace )
{
    std::mutex              mutex;
    std::condition_variable stoppedCv;
    bool                    stopped = false;
    const int               signalsBefore = g_openApiStopSignals.load();
    const auto              deadline = std::chrono::steady_clock::now() + aGrace;

    std::thread watchdog(
            [&]()
            {
                std::unique_lock<std::mutex> lock( mutex );

                while( !stopped )
                {
                    stoppedCv.wait_for( lock, std::chrono::milliseconds( 100 ) );

                    if( stopped )
                        return;

                    const bool again = g_openApiStopSignals.load() > signalsBefore;

                    if( !again && std::chrono::steady_clock::now() < deadline )
                        continue;

                    wxLog::SetThreadActiveTarget( KOPENAPI_JOURNAL::ThreadTarget() );
                    const nlohmann::json busy = aService.Busy();
                    wxLogError( "kicadopenapi: not stopped %s; exiting without waiting for %s",
                                again ? "before a second stop request"
                                      : wxString::Format( "within %.1f s", aGrace.count() / 1000.0 ),
                                busy.is_null() ? std::string( "the service" ) : busy.dump() );
                    aService.Abandon();
                    std::fflush( nullptr );
                    std::_Exit( CLI::EXIT_CODES::ERR_UNKNOWN );
                }
            } );

    aService.Stop();

    {
        std::lock_guard<std::mutex> lock( mutex );
        stopped = true;
    }

    stoppedCv.notify_all();
    watchdog.join();
}


/// Event-driven main loop: sleeps until the service queues main-thread work or exit is due
class MAIN_LOOP_WAKER
{
public:
    void Wake()
    {
        {
            std::lock_guard<std::mutex> lock( m_mutex );
            m_pending = true;
        }
        m_cv.notify_one();
    }

    void WaitForWork()
    {
        std::unique_lock<std::mutex> lock( m_mutex );
        m_cv.wait_for( lock, IDLE_WAIT,
                       [this]() { return m_pending || g_openApiExitRequested.load(); } );
        m_pending = false;
    }

private:
    std::mutex              m_mutex;
    std::condition_variable m_cv;
    bool                    m_pending = false;
};


CLI::OPENAPI_SERVER_COMMAND::OPENAPI_SERVER_COMMAND() :
        COMMAND( "openapi-server" )
{
    m_argParser.add_description(
            UTF8STDSTR( _( "Run the kicadopenapi Web API headless (no windows, no IPC)" ) ) );

    m_argParser.add_argument( ARG_PATH )
            .default_value( std::string() )
            .nargs( argparse::nargs_pattern::optional )
            .help( UTF8STDSTR( _( "Document to pre-load (not supported yet)" ) ) )
            .metavar( "PROJECT_OR_FILE" );

    m_argParser.add_argument( ARG_PORT )
            .default_value( -1 )
            .scan<'i', int>()
            .help( UTF8STDSTR( _( "First port to try (default: KICAD_OPENAPI_PORT or 4242)" ) ) )
            .metavar( "PORT" );

    m_argParser.add_argument( ARG_PARENT_PID )
            .default_value( 0 )
            .scan<'i', int>()
            .help( UTF8STDSTR( _( "Exit when this process ends (set by kicad-mcp-bridge)" ) ) )
            .metavar( "PID" );
}


int CLI::OPENAPI_SERVER_COMMAND::doPerform( KIWAY& aKiway )
{
    // Errors/warnings go to the journal (API + log file + stderr)
    KOPENAPI_JOURNAL::Install( "kicad-cli-openapi" );

    const wxString preload = From_UTF8( m_argParser.get<std::string>( ARG_PATH ).c_str() );

    if( !preload.IsEmpty() && !wxFileName::FileExists( preload ) )
    {
        wxFprintf( stderr, _( "File not found: %s\n" ), preload );
        return EXIT_CODES::ERR_INVALID_INPUT_FILE;
    }

    const long parentPid = m_argParser.get<int>( ARG_PARENT_PID );

    if( parentPid > 0 && !wxProcess::Exists( parentPid ) )
    {
        wxFprintf( stderr, _( "Parent process %ld is not running\n" ), parentPid );
        return EXIT_CODES::ERR_ARGS;
    }

    MAIN_LOOP_WAKER       waker;
    KICAD_OPENAPI_SERVICE service( &aKiway, true );

    service.SetMainLoopWaker( [&waker]() { waker.Wake(); } );
    service.SetShutdownHandler(
            [&waker]()
            {
                g_openApiExitRequested.store( true );
                waker.Wake();
            } );

    // armed before the service starts: a /shutdown or a signal during the preload below is kept
    g_openApiExitRequested.store( false );
    KICAD_OPENAPI_SERVICE::InstallStopRequestHandler( openApiStopRequested );

    if( !service.Start( m_argParser.get<int>( ARG_PORT ) ) )
        return EXIT_CODES::ERR_UNKNOWN;

    // a document given on the command line opens before the first request is served: a board or
    // schematic, or both of a project
    if( !preload.IsEmpty() )
    {
        wxFileName           fn( preload );
        KOPENAPI_CONTEXT     ctx{ &aKiway, true };
        std::vector<std::pair<std::string, wxString>> opens;

        fn.MakeAbsolute();

        if( fn.GetExt() == FILEEXT::KiCadPcbFileExtension )
            opens.emplace_back( "pcb_open", fn.GetFullPath() );
        else if( fn.GetExt() == FILEEXT::KiCadSchematicFileExtension )
            opens.emplace_back( "sch_open", fn.GetFullPath() );
        else if( fn.GetExt() == FILEEXT::ProjectFileExtension )
        {
            for( const auto& [method, ext] : { std::pair{ "sch_open", FILEEXT::KiCadSchematicFileExtension },
                                               std::pair{ "pcb_open", FILEEXT::KiCadPcbFileExtension } } )
            {
                wxFileName doc( fn );
                doc.SetExt( ext );

                if( doc.FileExists() )
                    opens.emplace_back( method, doc.GetFullPath() );
            }
        }
        else
        {
            wxFprintf( stderr, _( "Cannot open %s: give a .kicad_pcb, .kicad_sch or .kicad_pro\n" ), preload );
            service.Stop();
            return EXIT_CODES::ERR_ARGS;
        }

        service.RunOnOwner(
                [&]()
                {
                    for( KIWAY::FACE_T face : { KIWAY::FACE_SCH, KIWAY::FACE_PCB } )
                        aKiway.KiFACE( face );

                    for( const auto& [method, path] : opens )
                    {
                        std::optional<KOPENAPI_METHOD> open = KOPENAPI_REGISTRY::Get().Find( method );
                        KOPENAPI_RESULT result =
                                open ? open->handler( ctx, { { "path", path.ToStdString( wxConvUTF8 ) } } )
                                     : KOPENAPI_RESULT::Error( 500, method + std::string( " not available" ) );

                        if( result.status != 200 )
                        {
                            wxFprintf( stderr, _( "Cannot open %s: %s\n" ), path,
                                       From_UTF8( result.body.dump().c_str() ) );
                        }
                    }
                } );
    }

    service.PreloadKifaces();

    // Readiness for humans and scripts; the bridge uses the discovery file instead
    std::printf( "kicadopenapi headless server listening at http://127.0.0.1:%d\n", service.Port() );
    std::fflush( stdout );

    std::string why = "shutdown request";

    while( !g_openApiExitRequested.load() )
    {
        wxTheApp->ProcessPendingEvents();
        waker.WaitForWork();

        if( parentPid > 0 && !wxProcess::Exists( parentPid ) )
        {
            why = "parent process ended";
            break;
        }
    }

    if( g_openApiStopSignals.load() > 0 )
        why = "stop signal";

    service.RequestStop( why );
    stopWithin( service, stopGrace() );
    return EXIT_CODES::OK;
}
