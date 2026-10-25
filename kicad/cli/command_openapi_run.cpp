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

#include "command_openapi_run.h"

#include <cli/exit_codes.h>
#include <kicadopenapi_journal.h>
#include <kicadopenapi_service.h>
#include <string_utils.h>

#include <wx/app.h>
#include <wx/crt.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <thread>

#define ARG_BATCH "batch"
#define ARG_PORT "--port"
#define ARG_KEEP_GOING "--keep-going"


CLI::OPENAPI_RUN_COMMAND::OPENAPI_RUN_COMMAND() :
        COMMAND( "openapi-run" )
{
    m_argParser.add_description( UTF8STDSTR(
            _( "Run a batch of kicadopenapi calls headless: a JSON array of { \"method\", \"arguments\", "
               "\"job\" } (job: true runs the method as a job and waits for it); prints one JSON line "
               "per call" ) ) );

    m_argParser.add_argument( ARG_BATCH )
            .help( UTF8STDSTR( _( "Batch file (JSON)" ) ) )
            .metavar( "BATCH_JSON" );

    m_argParser.add_argument( ARG_PORT )
            .default_value( -1 )
            .scan<'i', int>()
            .help( UTF8STDSTR( _( "First port to try for the API while the batch runs (default: "
                                  "KICAD_OPENAPI_PORT or 4242)" ) ) )
            .metavar( "PORT" );

    m_argParser.add_argument( ARG_KEEP_GOING )
            .flag()
            .help( UTF8STDSTR( _( "Run the remaining calls after a failed one" ) ) );
}


namespace
{

/// @brief A step as a job: job_start, then job_get until it ends; the job's own status and result
KOPENAPI_RESULT runAsJob( KICAD_OPENAPI_SERVICE& aService, const std::string& aMethod, const nlohmann::json& aArgs )
{
    KOPENAPI_RESULT started = aService.Invoke( "job_start", { { "method", aMethod }, { "arguments", aArgs } } );

    if( started.status != 200 )
        return started;

    const std::string id = started.body.value( "job_id", std::string() );

    for( ;; )
    {
        KOPENAPI_RESULT got = aService.Invoke( "job_get", { { "job_id", id }, { "wait_s", 60 } } );

        if( got.status != 200 )
            return got;

        const std::string state = got.body.value( "state", std::string() );

        if( state == "queued" || state == "running" )
            continue;

        return { got.body.value( "status", 200 ), got.body };
    }
}

} // namespace


int CLI::OPENAPI_RUN_COMMAND::doPerform( KIWAY& aKiway )
{
    KOPENAPI_JOURNAL::Install( "kicad-cli-openapi-run" );

    const std::string path = m_argParser.get<std::string>( ARG_BATCH );
    std::ifstream     in( path );

    if( !in )
    {
        wxFprintf( stderr, _( "File not found: %s\n" ), From_UTF8( path.c_str() ) );
        return EXIT_CODES::ERR_INVALID_INPUT_FILE;
    }

    nlohmann::json batch = nlohmann::json::parse( in, nullptr, false );

    if( batch.is_object() && batch.contains( "calls" ) )
        batch = batch["calls"];

    if( !batch.is_array() )
    {
        wxFprintf( stderr, _( "%s: expected a JSON array of calls\n" ), From_UTF8( path.c_str() ) );
        return EXIT_CODES::ERR_INVALID_INPUT_FILE;
    }

    for( const nlohmann::json& call : batch )
    {
        if( !call.is_object() || !call.contains( "method" ) || !call["method"].is_string() )
        {
            wxFprintf( stderr, _( "%s: every call needs a \"method\"\n" ), From_UTF8( path.c_str() ) );
            return EXIT_CODES::ERR_INVALID_INPUT_FILE;
        }
    }

    const bool keepGoing = m_argParser.get<bool>( ARG_KEEP_GOING );

    std::mutex              mutex;
    std::condition_variable wake;
    bool                    pending = false;
    std::atomic<bool>       done{ false };

    auto wakeMain = [&]()
    {
        {
            std::lock_guard<std::mutex> lock( mutex );
            pending = true;
        }

        wake.notify_one();
    };

    KICAD_OPENAPI_SERVICE service( &aKiway, true );
    service.SetMainLoopWaker( wakeMain );

    if( !service.Start( m_argParser.get<int>( ARG_PORT ) ) )
        return EXIT_CODES::ERR_UNKNOWN;

    std::atomic<bool> failed{ false };

    // the calls wait for their answers; the main thread keeps serving what the service queues on it
    std::thread runner(
            [&]()
            {
                int index = 0;

                for( const nlohmann::json& call : batch )
                {
                    const std::string    method = call["method"].get<std::string>();
                    const nlohmann::json args = call.value( "arguments", nlohmann::json::object() );
                    const auto           t0 = std::chrono::steady_clock::now();

                    KOPENAPI_RESULT result = call.value( "job", false ) ? runAsJob( service, method, args )
                                                                       : service.Invoke( method, args );

                    const double ms = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now()
                                                                                 - t0 )
                                              .count();

                    nlohmann::json line = { { "call", index++ },
                                            { "method", method },
                                            { "status", result.status },
                                            { "ms", std::round( ms * 10 ) / 10 },
                                            { "result", result.body } };

                    std::printf( "%s\n", line.dump().c_str() );
                    std::fflush( stdout );

                    if( result.status >= 400 )
                    {
                        failed.store( true );

                        if( !keepGoing )
                            break;
                    }
                }

                done.store( true );
                wakeMain();
            } );

    while( !done.load() )
    {
        wxTheApp->ProcessPendingEvents();

        std::unique_lock<std::mutex> lock( mutex );
        wake.wait_for( lock, std::chrono::milliseconds( 100 ), [&]() { return pending || done.load(); } );
        pending = false;
    }

    runner.join();
    service.Stop();
    return failed.load() ? EXIT_CODES::ERR_JOBS_RUN_FAILED : EXIT_CODES::OK;
}
