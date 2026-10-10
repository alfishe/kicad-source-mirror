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
#include <csignal>
#include <cstdio>
#include <mutex>

#include <cli/exit_codes.h>
#include <kicadopenapi_journal.h>
#include <kicadopenapi_service.h>
#include <wx/app.h>
#include <wx/crt.h>
#include <wx/process.h>

#define ARG_PATH "path"
#define ARG_PORT "--port"
#define ARG_PARENT_PID "--parent-pid"


/// Upper bound for noticing signals and a dead parent while idle; requests wake the loop at once
static constexpr std::chrono::milliseconds IDLE_WAIT( 100 );

static std::atomic_bool g_openApiExitRequested{ false };

static void openApiSignalHandler( int )
{
    g_openApiExitRequested.store( true );
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

        for( KIWAY::FACE_T face : { KIWAY::FACE_SCH, KIWAY::FACE_PCB } )
            aKiway.KiFACE( face );

        for( const auto& [method, path] : opens )
        {
            std::optional<KOPENAPI_METHOD> open = KOPENAPI_REGISTRY::Get().Find( method );
            KOPENAPI_RESULT result = open ? open->handler( ctx, { { "path", path.ToStdString( wxConvUTF8 ) } } )
                                          : KOPENAPI_RESULT::Error( 500, method + std::string( " not available" ) );

            if( result.status != 200 )
                wxFprintf( stderr, _( "Cannot open %s: %s\n" ), path, From_UTF8( result.body.dump().c_str() ) );
        }
    }

    service.PreloadKifaces();

    // Readiness for humans and scripts; the bridge uses the discovery file instead
    std::printf( "kicadopenapi headless server listening at http://127.0.0.1:%d\n", service.Port() );
    std::fflush( stdout );

    g_openApiExitRequested.store( false );
    auto oldSigInt = std::signal( SIGINT, openApiSignalHandler );
    auto oldSigTerm = std::signal( SIGTERM, openApiSignalHandler );

    while( !g_openApiExitRequested.load() )
    {
        wxTheApp->ProcessPendingEvents();
        waker.WaitForWork();

        if( parentPid > 0 && !wxProcess::Exists( parentPid ) )
            break;
    }

    std::signal( SIGINT, oldSigInt );
    std::signal( SIGTERM, oldSigTerm );

    service.Stop();
    return EXIT_CODES::OK;
}
