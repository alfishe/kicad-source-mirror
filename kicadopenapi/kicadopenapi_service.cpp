#include "kicadopenapi_service.h"
#include "kicadopenapi_history.h"
#include "kicadopenapi_registry.h"
#include "recording/kicadopenapi_recorder.h"
#include "kicadopenapi_mcp.h"
#include "kicadopenapi_journal.h"
#include "kicadopenapi_libraries.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <wx/utils.h>
#include <fstream>
#include <future>
#include <cmath>
#include <mutex>
#include <deque>
#include <thread>

#include <httplib.h>

#include <kiway.h>
#include <wx/app.h>
#include <wx/log.h>
#include <kicadopenapi_util.h>
#include <wx/datetime.h>

#include <platform.h>

static const char* const KOPENAPI_VERSION = "0.2.0";
static const char* const KOPENAPI_HOST_ADDR = "127.0.0.1";

/// @brief Default upper bound for one main-thread call (methods may declare more); long operations
/// will move to async jobs.
static constexpr std::chrono::seconds CALL_TIMEOUT( 15 );
static constexpr std::chrono::milliseconds WAIT_SLICE( 20 );

namespace fs = std::filesystem;


/// @brief Drops discovery files left behind by crashed processes
static std::atomic<int>  s_apiCalls{ 0 };       ///< API calls running on the main thread now
static std::atomic<bool> s_overrideLock{ false };


static void removeStaleDiscoveryFiles( const fs::path& aDir )
{
    std::error_code ec;

    for( const fs::directory_entry& entry : fs::directory_iterator( aDir, ec ) )
    {
        if( entry.path().extension() != ".json" )
            continue;

        long pid = std::atol( entry.path().stem().string().c_str() );

        if( pid > 0 && !kopenapi::platform::ProcessAlive( pid ) )
            fs::remove( entry.path(), ec );
    }
}


static std::string toJson( const nlohmann::json& aJson )
{
    // Replace invalid UTF-8 rather than throwing from inside an HTTP handler
    return aJson.dump( -1, ' ', false, nlohmann::json::error_handler_t::replace );
}


static void reply( httplib::Response& aRes, const KOPENAPI_RESULT& aResult )
{
    aRes.status = aResult.status;
    aRes.set_content( toJson( aResult.body ), "application/json" );
}


/// @brief The main-thread end of the gate: API work arrives as thread events (category THREAD),
/// so a call that keeps the UI alive (KopenapiKeepUiAlive: paint and timers only) never starts
/// another call inside itself.
class MAIN_GATE_HANDLER : public wxEvtHandler
{
public:
    MAIN_GATE_HANDLER()
    {
        Bind( wxEVT_THREAD,
              []( wxThreadEvent& aEvent )
              {
                  if( auto job = aEvent.GetPayload<std::shared_ptr<std::function<void()>>>() )
                      ( *job )();
              } );
    }

    static MAIN_GATE_HANDLER& Get()
    {
        static MAIN_GATE_HANDLER handler;
        return handler;
    }
};


/// @brief Run aFn on the main (GUI/event-loop) thread and wait for its result on the calling HTTP
/// worker.  The main thread never waits on HTTP threads.  If the service stops or the call
/// exceeds CALL_TIMEOUT, the worker returns an error; a late closure still completes safely
/// because it only touches shared state.
static KOPENAPI_RESULT runInMain( const std::shared_ptr<std::atomic<bool>>& aAlive,
                                  const std::function<void()>&             aWaker,
                                  std::function<KOPENAPI_RESULT()>         aFn,
                                  std::chrono::seconds                     aTimeout = CALL_TIMEOUT )
{
    if( !wxTheApp )
        return KOPENAPI_RESULT::Error( 503, "no application event loop" );

    auto promise = std::make_shared<std::promise<KOPENAPI_RESULT>>();
    std::future<KOPENAPI_RESULT> future = promise->get_future();

    auto job = std::make_shared<std::function<void()>>(
            [aAlive, promise, fn = std::move( aFn )]()
            {
                if( !aAlive->load() )
                {
                    promise->set_value( KOPENAPI_RESULT::Error( 503, "service stopping" ) );
                    return;
                }

                try
                {
                    promise->set_value( fn() );
                }
                catch( const std::exception& e )
                {
                    promise->set_value( KOPENAPI_RESULT::Error( 500, e.what() ) );
                }
                catch( ... )
                {
                    promise->set_value( KOPENAPI_RESULT::Error( 500, "unknown exception" ) );
                }
            } );

    auto* event = new wxThreadEvent();
    event->SetPayload( job );
    MAIN_GATE_HANDLER::Get().QueueEvent( event );

    if( aWaker )
        aWaker();

    const auto deadline = std::chrono::steady_clock::now() + aTimeout;

    while( future.wait_for( WAIT_SLICE ) != std::future_status::ready )
    {
        if( !aAlive->load() )
            return KOPENAPI_RESULT::Error( 503, "service stopping" );

        if( std::chrono::steady_clock::now() > deadline )
            return KOPENAPI_RESULT::Error( 504, "main thread did not complete the call in time" );
    }

    return future.get();
}


struct KICAD_OPENAPI_SERVICE::IMPL
{
    KOPENAPI_CONTEXT                  ctx;
    std::unique_ptr<httplib::Server>  server;
    std::thread                       thread;
    int                               port = 0;
    std::string                       appName;
    std::shared_ptr<std::atomic<bool>> alive = std::make_shared<std::atomic<bool>>( false );
    fs::path                          discoveryFile;
    std::function<void()>             waker;
    std::function<void()>             shutdownHandler;

    nlohmann::json statusJson() const
    {
        return { { "service", "kicadopenapi" },
                 { "version", KOPENAPI_VERSION },
                 { "app", appName },
                 { "pid", kopenapi::platform::CurrentPid() },
                 { "port", port },
                 { "headless", ctx.headless },
                 { "busy", busyJson() } };
    }

    /// @brief The call holding the main thread (method, running_ms), or null; any thread
    nlohmann::json busyJson() const
    {
        std::lock_guard<std::mutex> lock( traceMutex );

        if( busyMethod.empty() )
            return nullptr;

        return { { "method", busyMethod },
                 { "running_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now() - busySince ).count() } };
    }

    nlohmann::json openApiJson() const;

    /// @brief statusJson() + open documents and the unsaved flag (main-thread round trip)
    KOPENAPI_RESULT liveStatus() const;

    /// @brief Load the editor kifaces (eeschema, pcbnew) once, on the main thread, so their methods
    /// and document providers are registered.  Lazy: paid by the first API use, not at start.
    void ensureKifaces() const;

    mutable std::atomic<bool> kifacesLoaded{ false };

    /// @brief One API call through the gate, for api_trace
    struct TRACE
    {
        long long   seq = 0;
        std::string method;
        size_t      argBytes = 0;
        double      queuedMs = 0;   ///< waiting for the main thread
        double      runMs = 0;      ///< running there
        int         status = 0;
        std::string at;             ///< wall clock when it was received
        std::string args;           ///< arguments, cut at 2 KB
        std::string error;          ///< error message of a failed call

        nlohmann::json Json() const
        {
            nlohmann::json j = { { "seq", seq }, { "method", method }, { "at", at }, { "arg_bytes", argBytes },
                                 { "queued_ms", queuedMs }, { "run_ms", runMs }, { "status", status },
                                 { "args", args } };

            if( !error.empty() )
                j["error"] = error;

            return j;
        }
    };

    mutable std::atomic<bool> traceOn{ false };  ///< off: the gate records nothing (no buffer, no cost)
    mutable std::mutex        traceMutex;
    mutable std::deque<TRACE> traces;            ///< newest last, at most traceLimit bytes
    mutable std::deque<size_t> traceSizes;       ///< bytes of each entry (its JSON line)
    mutable size_t            traceBytes = 0;
    mutable size_t            traceLimit = 1 << 20;
    mutable std::string       traceFile;         ///< JSON lines appended per call, empty: off
    mutable std::ofstream     traceStream;
    bool                      traceToJournal = false;   ///< env KICAD_OPENAPI_TRACE
    mutable long long         traceSeq = 0;
    mutable std::string       busyMethod;        ///< the call running on the main thread now
    mutable std::chrono::steady_clock::time_point busySince;
    mutable nlohmann::json    lastDocuments = nlohmann::json::array();

    nlohmann::json traceJson( const nlohmann::json& aArgs ) const;
    std::thread               preloadThread;   ///< joined in Stop()
    std::thread               publishThread;   ///< restart: publishes once the reopened document is open

    void writeDiscoveryFile();

    KOPENAPI_RESULT invoke( const std::string& aName, const std::string& aBody ) const;

    /// @brief Same as invoke() with already parsed arguments (REST and MCP share this path)
    KOPENAPI_RESULT invokeParsed( const std::string& aName, const nlohmann::json& aArgs ) const;

    void handleMcp( const httplib::Request& aReq, httplib::Response& aRes ) const;

    void registerRoutes();
};


void KICAD_OPENAPI_SERVICE::IMPL::ensureKifaces() const
{
    if( kifacesLoaded.load() || !ctx.kiway )
        return;

    KIWAY* kiway = ctx.kiway;

    runInMain( alive, waker,
               [kiway]()
               {
                   for( KIWAY::FACE_T face : { KIWAY::FACE_SCH, KIWAY::FACE_PCB } )
                   {
                       try
                       {
                           kiway->KiFACE( face );
                       }
                       catch( ... )
                       {
                           // A missing kiface only means its methods are unavailable
                       }
                   }

                   return KOPENAPI_RESULT::Ok( nlohmann::json::object() );
               } );

    kifacesLoaded.store( true );
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::IMPL::liveStatus() const
{
    ensureKifaces();

    // while a call holds the main thread, answer with the documents seen last
    if( !busyJson().is_null() )
    {
        nlohmann::json status = statusJson();
        std::lock_guard<std::mutex> lock( traceMutex );
        status["documents"] = lastDocuments;
        status["documents_stale"] = true;
        status["journal"] = KOPENAPI_JOURNAL::Summary();
        return KOPENAPI_RESULT::Ok( status );
    }

    KOPENAPI_CONTEXT ctxCopy = ctx;
    KOPENAPI_RESULT  docs = runInMain( alive, waker,
                                       [ctxCopy]() mutable
                                       {
                                           return KOPENAPI_RESULT::Ok(
                                                   KOPENAPI_REGISTRY::Get().Documents( ctxCopy ) );
                                       } );

    if( docs.status != 200 )
        return docs;

    {
        std::lock_guard<std::mutex> lock( traceMutex );
        lastDocuments = docs.body;
    }

    nlohmann::json status = statusJson();
    bool           unsaved = false;

    for( const nlohmann::json& doc : docs.body )
        unsaved |= doc.value( "unsaved", false );

    status["documents"] = docs.body;
    status["unsaved"] = unsaved;
    status["journal"] = KOPENAPI_JOURNAL::Summary();
    return KOPENAPI_RESULT::Ok( status );
}


nlohmann::json KICAD_OPENAPI_SERVICE::IMPL::openApiJson() const
{
    ensureKifaces();

    using nlohmann::json;

    const json errorRef = { { "$ref", "#/components/schemas/Error" } };
    const json errorResponse = {
        { "description", "Error" },
        { "content", { { "application/json", { { "schema", errorRef } } } } }
    };

    json paths = json::object();

    paths["/api/v1/status"] = {
        { "get",
          { { "operationId", "status" },
            { "summary", "Service and process information" },
            { "responses",
              { { "200",
                  { { "description", "Status" },
                    { "content",
                      { { "application/json",
                          { { "schema", { { "$ref", "#/components/schemas/Status" } } } } } } } } } } } } }
    };

    paths["/api/v1/openapi.json"] = {
        { "get",
          { { "operationId", "openapi" },
            { "summary", "This OpenAPI manifest" },
            { "responses", { { "200", { { "description", "OpenAPI 3.0 document" } } } } } } }
    };

    paths["/mcp"] = {
        { "post",
          { { "operationId", "mcp" },
            { "summary", "MCP endpoint (JSON-RPC 2.0, Streamable HTTP): tools search + invoke" },
            { "requestBody",
              { { "required", true },
                { "content",
                  { { "application/json",
                      { { "schema", { { "type", "object" } } } } } } } } },
            { "responses",
              { { "200", { { "description", "JSON-RPC response" } } },
                { "202", { { "description", "Notification accepted" } } } } } } }
    };

    if( shutdownHandler )
    {
        paths["/api/v1/shutdown"] = {
            { "post",
              { { "operationId", "shutdown" },
                { "summary", "Stop this headless process" },
                { "responses", { { "200", { { "description", "Shutdown started" } } } } } } }
        };
    }

    for( const KOPENAPI_METHOD& m : KOPENAPI_REGISTRY::Get().Snapshot() )
    {
        if( shutdownHandler && m.name == "shutdown" )
            continue;   // the built-in route wins

        json op = {
            { "operationId", m.name },
            { "summary", m.summary },
            { "x-gui-only", m.guiOnly },
            { "requestBody",
              { { "required", true },
                { "content", { { "application/json", { { "schema", m.inputSchema } } } } } } },
            { "responses",
              { { "200",
                  { { "description", "Success" },
                    { "content",
                      { { "application/json",
                          { { "schema", { { "type", "object" } } } } } } } } },
                { "default", errorResponse } } }
        };

        paths["/api/v1/" + m.name] = { { "post", op } };
    }

    return {
        { "openapi", "3.0.3" },
        { "info",
          { { "title", "kicadopenapi" },
            { "version", KOPENAPI_VERSION },
            { "description", "In-process Web API of a KiCad process (" + appName + ")" } } },
        { "servers", json::array( { { { "url", "http://" + std::string( KOPENAPI_HOST_ADDR ) + ":"
                                                     + std::to_string( port ) } } } ) },
        { "paths", paths },
        { "components",
          { { "schemas",
              { { "Error",
                  { { "type", "object" },
                    { "required", { "error" } },
                    { "properties",
                      { { "error",
                          { { "type", "object" },
                            { "required", { "code", "message" } },
                            { "properties",
                              { { "code", { { "type", "integer" } } },
                                { "message", { { "type", "string" } } } } } } } } } } },
                { "Status",
                  { { "type", "object" },
                    { "properties",
                      { { "service", { { "type", "string" } } },
                        { "version", { { "type", "string" } } },
                        { "app", { { "type", "string" } } },
                        { "pid", { { "type", "integer" } } },
                        { "port", { { "type", "integer" } } },
                        { "headless", { { "type", "boolean" } } },
                        { "unsaved", { { "type", "boolean" } } },
                        { "journal", { { "type", "object" } } },
                        { "documents", { { "type", "array" }, { "items", { { "type", "object" } } } } } } } } } } } } }
    };
}


void KICAD_OPENAPI_SERVICE::IMPL::writeDiscoveryFile()
{
    std::error_code ec;
    fs::path        dir = kopenapi::platform::DiscoveryDir();

    fs::create_directories( dir, ec );
    removeStaleDiscoveryFiles( dir );

    const std::string base = "http://" + std::string( KOPENAPI_HOST_ADDR ) + ":"
                             + std::to_string( port );
    nlohmann::json info = statusJson();
    info["url"] = base;
    info["mcp"] = base + "/mcp";

    // Write-then-rename so readers never see a partial file
    fs::path file = dir / ( std::to_string( kopenapi::platform::CurrentPid() ) + ".json" );
    fs::path tmp = file;
    tmp += ".tmp";

    {
        std::ofstream out( tmp, std::ios::trunc );
        out << toJson( info ) << "\n";
    }

    fs::rename( tmp, file, ec );

    if( ec )
        fs::remove( tmp, ec );
    else
        discoveryFile = file;
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::IMPL::invoke( const std::string& aName,
                                                     const std::string& aBody ) const
{
    nlohmann::json args = nlohmann::json::object();

    if( !aBody.empty() )
    {
        args = nlohmann::json::parse( aBody, nullptr, false );

        if( args.is_discarded() || !args.is_object() )
            return KOPENAPI_RESULT::Error( 400, "request body must be a JSON object" );
    }

    return invokeParsed( aName, args );
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::IMPL::invokeParsed( const std::string&    aName,
                                                           const nlohmann::json& aArgs ) const
{
    ensureKifaces();

    std::optional<KOPENAPI_METHOD> method = KOPENAPI_REGISTRY::Get().Find( aName );

    if( !method )
        return KOPENAPI_RESULT::Error( 404, "unknown method '" + aName + "'" );

    if( method->guiOnly && ctx.headless )
        return KOPENAPI_RESULT::Error( 501, "'" + aName + "' requires the GUI" );

    if( method->inputSchema.contains( "required" ) )
    {
        for( const nlohmann::json& field : method->inputSchema["required"] )
        {
            if( field.is_string() && !aArgs.contains( field.get<std::string>() ) )
                return KOPENAPI_RESULT::Error( 400, "missing required field '"
                                                            + field.get<std::string>() + "'" );
        }
    }

    // answered on the HTTP thread: works while the main thread is busy
    if( aName == "api_trace" )
        return KOPENAPI_RESULT::Ok( traceJson( aArgs ) );

    KOPENAPI_CONTEXT ctxCopy = ctx;
    KOPENAPI_HANDLER handler = method->handler;
    nlohmann::json   args = aArgs;

    const std::string operation = "api:" + aName;
    const bool        tracing = traceOn.load( std::memory_order_relaxed );
    const auto        received = tracing ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point();
    auto              started = tracing ? std::make_shared<std::chrono::steady_clock::time_point>( received ) : nullptr;
    const IMPL*       self = this;

    KOPENAPI_RESULT outcome = runInMain( alive, waker,
                      [ctxCopy, handler, args, operation, started, self]() mutable
                      {
                          const auto now = std::chrono::steady_clock::now();

                          if( started )
                              *started = now;

                          {
                              std::lock_guard<std::mutex> lock( self->traceMutex );
                              self->busyMethod = operation.substr( 4 );
                              self->busySince = now;
                          }

                          s_apiCalls++;

                          struct BUSY_END
                          {
                              const IMPL* impl;
                              ~BUSY_END()
                              {
                                  s_apiCalls--;
                                  std::lock_guard<std::mutex> lock( impl->traceMutex );
                                  impl->busyMethod.clear();
                              }
                          } busyEnd{ self };

                          // Log records during the call are attributed to this method
                          KOPENAPI_JOURNAL::SCOPED_OPERATION scope( operation );

                          // Editing calls: the documents are marked before, the call is logged
                          const std::string name = operation.substr( 4 );
                          const bool        editing = KOPENAPI_REGISTRY::Get().IsEditing( name );

                          if( editing )
                              KOPENAPI_HISTORY::Get().BeforeEdit( ctxCopy, name, args );

                          KOPENAPI_RESULT result = handler( ctxCopy, args );

                          if( editing )
                              KOPENAPI_HISTORY::Get().AfterEdit( ctxCopy, result );

                          // Failed operations are journaled too (4xx warning, 5xx error)
                          if( result.status >= 400 )
                          {
                              const std::string message = result.body.contains( "error" )
                                      ? result.body["error"].value( "message", std::string() )
                                      : std::string();
                              const wxString text = wxString::FromUTF8(
                                      operation + " failed (" + std::to_string( result.status )
                                      + "): " + message );

                              if( result.status >= 500 )
                                  wxLogError( "%s", text );
                              else
                                  wxLogWarning( "%s", text );
                          }

                          return result;
                      },
                      std::chrono::seconds( std::max( method->timeoutSec, 1 ) ) );

    if( !tracing )
        return outcome;

    const auto finished = std::chrono::steady_clock::now();
    auto       ms = []( auto d ) { return std::round( std::chrono::duration<double, std::milli>( d ).count() * 10 ) / 10; };
    TRACE      t;
    t.method = aName;
    t.args = aArgs.dump();
    t.argBytes = t.args.size();

    if( t.args.size() > 2048 )
        t.args = t.args.substr( 0, 2048 ) + "...";

    if( outcome.status >= 400 && outcome.body.contains( "error" ) )
        t.error = outcome.body["error"].value( "message", std::string() );
    t.queuedMs = ms( *started - received );
    t.runMs = ms( finished - *started );
    t.status = outcome.status;
    t.at = wxDateTime::Now().FormatISOCombined( ' ' ).ToStdString();

    {
        std::lock_guard<std::mutex> lock( traceMutex );
        t.seq = ++traceSeq;
        const std::string line = t.Json().dump();

        traces.push_back( t );
        traceSizes.push_back( line.size() + 1 );
        traceBytes += line.size() + 1;

        while( traceBytes > traceLimit && traces.size() > 1 )
        {
            traceBytes -= traceSizes.front();
            traceSizes.pop_front();
            traces.pop_front();
        }

        if( traceStream.is_open() )
            traceStream << line << '\n' << std::flush;
    }

    if( traceToJournal )
        wxLogMessage( "api #%lld %s %zu B: queued %.1f ms, ran %.1f ms -> %d", t.seq, aName, t.argBytes, t.queuedMs,
                      t.runMs, t.status );

    return outcome;
}


nlohmann::json KICAD_OPENAPI_SERVICE::IMPL::traceJson( const nlohmann::json& aArgs ) const
{
    // configuration: on / off, buffer size, file sink
    if( aArgs.contains( "enabled" ) || aArgs.contains( "buffer_bytes" ) || aArgs.contains( "file" ) )
    {
        std::lock_guard<std::mutex> lock( traceMutex );
        const bool on = aArgs.value( "enabled", true );

        traceOn = on;

        if( !on )
        {
            traces.clear();
            traces.shrink_to_fit();
            traceSizes.clear();
            traceSizes.shrink_to_fit();
            traceBytes = 0;
            traceStream.close();
            traceFile.clear();
            return { { "enabled", false } };
        }

        if( aArgs.contains( "buffer_bytes" ) )
            traceLimit = std::clamp<size_t>( aArgs["buffer_bytes"].get<size_t>(), 4096, size_t( 1 ) << 30 );

        if( aArgs.contains( "file" ) )
        {
            traceStream.close();
            traceFile = aArgs["file"].is_string() ? aArgs["file"].get<std::string>() : std::string();

            if( !traceFile.empty() )
            {
                traceStream.open( traceFile, std::ios::app );

                if( !traceStream )
                {
                    const std::string bad = traceFile;
                    traceFile.clear();
                    return { { "error", "cannot open " + bad } };
                }
            }
        }

        while( traceBytes > traceLimit && traces.size() > 1 )
        {
            traceBytes -= traceSizes.front();
            traceSizes.pop_front();
            traces.pop_front();
        }
    }

    if( !traceOn )
        return { { "enabled", false }, { "hint", "api_trace {enabled: true, buffer_bytes?, file?} starts tracing" } };

    const size_t   limit = std::max( aArgs.value( "limit", 50 ), 1 );
    const std::string only = aArgs.value( "method", std::string() );
    const double   slowerMs = aArgs.value( "slower_ms", 0.0 );
    nlohmann::json calls = nlohmann::json::array();

    std::lock_guard<std::mutex> lock( traceMutex );

    for( auto it = traces.rbegin(); it != traces.rend() && calls.size() < limit; ++it )
    {
        if( ( !only.empty() && !KopenapiGlob( only, it->method ) ) || it->runMs + it->queuedMs < slowerMs )
            continue;

        nlohmann::json row = it->Json();

        if( !aArgs.value( "with_args", false ) )
            row.erase( "args" );

        calls.push_back( row );
    }

    nlohmann::json busy = nullptr;

    if( !busyMethod.empty() )
    {
        busy = { { "method", busyMethod },
                 { "running_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now() - busySince ).count() } };
    }

    return { { "enabled", true }, { "calls", calls }, { "total", traceSeq }, { "busy", busy }, { "kept", traces.size() },
             { "buffer_bytes", traceLimit }, { "used_bytes", traceBytes },
             { "file", traceFile.empty() ? nlohmann::json() : nlohmann::json( traceFile ) } };
}


void KICAD_OPENAPI_SERVICE::IMPL::handleMcp( const httplib::Request& aReq,
                                             httplib::Response&      aRes ) const
{
    KOPENAPI_MCP_CONTEXT mcpCtx;
    mcpCtx.headless = ctx.headless;
    mcpCtx.version = KOPENAPI_VERSION;
    ensureKifaces();   // search must see kiface methods
    mcpCtx.invoke = [this]( const std::string& aName, const nlohmann::json& aArgs )
    {
        return invokeParsed( aName, aArgs );
    };

    const nlohmann::json msg = nlohmann::json::parse( aReq.body, nullptr, false );

    if( msg.is_discarded() )
    {
        aRes.status = 400;
        aRes.set_content( toJson( { { "jsonrpc", "2.0" },
                                    { "id", nullptr },
                                    { "error", { { "code", -32700 }, { "message", "parse error" } } } } ),
                          "application/json" );
        return;
    }

    nlohmann::json response;

    if( msg.is_array() )   // JSON-RPC batch
    {
        response = nlohmann::json::array();

        for( const nlohmann::json& item : msg )
        {
            if( std::optional<nlohmann::json> r = KopenapiMcpHandle( mcpCtx, item ) )
                response.push_back( *r );
        }

        if( response.empty() )
            response = nullptr;
    }
    else if( std::optional<nlohmann::json> r = KopenapiMcpHandle( mcpCtx, msg ) )
    {
        response = *r;
    }

    if( response.is_null() )
    {
        aRes.status = 202;   // notifications only
        return;
    }

    aRes.status = 200;
    aRes.set_content( toJson( response ), "application/json" );
}


void KICAD_OPENAPI_SERVICE::IMPL::registerRoutes()
{
    // Exclusive listener (per OS, see platform layer): httplib's default SO_REUSEPORT would
    // let two KiCad processes silently share one port
    server->set_socket_options(
            []( socket_t aSock )
            {
                kopenapi::platform::ConfigureListenSocket( static_cast<std::uintptr_t>( aSock ) );
            } );

    server->Get( "/",
                 []( const httplib::Request&, httplib::Response& res )
                 {
                     // swagger-ui (assets from CDN) over the generated manifest
                     res.set_content(
                             "<!DOCTYPE html><html><head><title>kicadopenapi</title>"
                             "<meta charset='utf-8'/>"
                             "<link rel='stylesheet' "
                             "href='https://unpkg.com/swagger-ui-dist@5/swagger-ui.css'>"
                             "</head><body><div id='swagger-ui'></div>"
                             "<script "
                             "src='https://unpkg.com/swagger-ui-dist@5/swagger-ui-bundle.js'>"
                             "</script><script>window.onload=()=>{SwaggerUIBundle({"
                             "url:'/api/v1/openapi.json',dom_id:'#swagger-ui'})};</script>"
                             "</body></html>",
                             "text/html" );
                 } );

    server->Get( "/api/v1/status",
                 [this]( const httplib::Request&, httplib::Response& res )
                 {
                     reply( res, liveStatus() );
                 } );

    server->Get( "/api/v1/openapi.json",
                 [this]( const httplib::Request&, httplib::Response& res )
                 {
                     reply( res, KOPENAPI_RESULT::Ok( openApiJson() ) );
                 } );

    server->Post( "/mcp",
                  [this]( const httplib::Request& req, httplib::Response& res )
                  {
                      handleMcp( req, res );
                  } );

    if( shutdownHandler )
    {
        // Registered before the generic method route so it takes precedence
        server->Post( "/api/v1/shutdown",
                      [this]( const httplib::Request&, httplib::Response& res )
                      {
                          reply( res, KOPENAPI_RESULT::Ok( { { "stopping", true } } ) );
                          shutdownHandler();
                      } );
    }

    server->Post( R"(/api/v1/([a-z0-9_]+))",
                  [this]( const httplib::Request& req, httplib::Response& res )
                  {
                      reply( res, invoke( req.matches[1], req.body ) );
                  } );

    server->set_error_handler(
            []( const httplib::Request&, httplib::Response& res )
            {
                // Keep error bodies JSON (unknown paths, wrong verbs)
                if( res.body.empty() )
                {
                    res.set_content( toJson( KOPENAPI_RESULT::Error(
                                                     res.status, httplib::status_message( res.status ) )
                                                     .body ),
                                     "application/json" );
                }
            } );
}


KICAD_OPENAPI_SERVICE::KICAD_OPENAPI_SERVICE( KIWAY* aKiway, bool aHeadless ) :
        m_impl( std::make_unique<IMPL>() )
{
    m_impl->ctx.kiway = aKiway;
    m_impl->ctx.headless = aHeadless;
}


KICAD_OPENAPI_SERVICE::~KICAD_OPENAPI_SERVICE()
{
    Stop();
}


void KICAD_OPENAPI_SERVICE::PreloadKifaces()
{
    if( !wxTheApp || !m_impl->ctx.kiway || m_impl->kifacesLoaded.load() )
        return;

    // ensureKifaces() on a worker: it marshals to the main loop and returns when the kifaces
    // are in (or the service stops); requests arriving meanwhile queue behind it
    if( !m_impl->preloadThread.joinable() )
        m_impl->preloadThread = std::thread( [impl = m_impl.get()]() { impl->ensureKifaces(); } );
}


void KICAD_OPENAPI_SERVICE::SetMainLoopWaker( std::function<void()> aWaker )
{
    m_impl->waker = std::move( aWaker );
}


void KICAD_OPENAPI_SERVICE::SetShutdownHandler( std::function<void()> aHandler )
{
    m_impl->shutdownHandler = std::move( aHandler );
}


namespace
{
std::atomic<int> s_currentPort{ 0 };
}


int KICAD_OPENAPI_SERVICE::CurrentPort()
{
    return s_currentPort.load();
}


int KICAD_OPENAPI_SERVICE::DefaultPort()
{
    if( const char* env = std::getenv( "KICAD_OPENAPI_PORT" ) )
    {
        int port = std::atoi( env );

        if( port > 0 && port < 65536 )
            return port;
    }

    return DEFAULT_PORT;
}


bool KICAD_OPENAPI_SERVICE::Start( int aPort )
{
    if( Running() )
        return true;

    const int basePort = aPort < 0 ? DefaultPort() : aPort;

    // app_restart: the process we replace still holds the port until it exits
    if( const char* env = std::getenv( "KICAD_OPENAPI_WAIT_PID" ) )
    {
        const long pid = std::atol( env );

        for( int i = 0; i < 300 && kopenapi::platform::ProcessAlive( pid ); ++i )
            std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

        wxUnsetEnv( wxS( "KICAD_OPENAPI_WAIT_PID" ) );
    }

    m_impl->appName = wxTheApp ? wxTheApp->GetAppName().ToStdString() : std::string( "kicad" );
    m_impl->port = 0;

    for( int i = 0; i < PORT_PROBE_COUNT; ++i )
    {
        // A fresh server per attempt: httplib decommissions a server after a failed bind
        m_impl->server = std::make_unique<httplib::Server>();
        m_impl->registerRoutes();

        if( m_impl->server->bind_to_port( KOPENAPI_HOST_ADDR, basePort + i ) )
        {
            m_impl->port = basePort + i;
            s_currentPort = m_impl->port;
            break;
        }
    }

    if( !m_impl->port )
    {
        std::fprintf( stderr, "kicadopenapi: failed to bind 127.0.0.1:%d..%d\n", basePort,
                      basePort + PORT_PROBE_COUNT - 1 );
        m_impl->server.reset();
        return false;
    }

    m_impl->alive->store( true );
    MAIN_GATE_HANDLER::Get();   // created on the main thread

    if( std::getenv( "KICAD_OPENAPI_TRACE" ) )
    {
        m_impl->traceOn = true;
        m_impl->traceToJournal = true;
    }

    // Startup ends when the main loop first runs; later records not caused by an API call
    // are attributed to "background"
    if( wxTheApp )
        wxTheApp->CallAfter( []() { KOPENAPI_JOURNAL::SetOperation( "background" ); } );

    // The socket is already bound, so clients can connect as soon as Start() returns;
    // listen_after_bind() blocks in this thread until Stop().
    m_impl->thread = std::thread( [srv = m_impl->server.get()]() { srv->listen_after_bind(); } );

    // After app_restart clients find this process only once the document it reopens is open:
    // their next call (e.g. a 3D view of the board) must not race the loading
    std::string reopen;

    if( const char* env = std::getenv( "KICAD_OPENAPI_REOPEN" ) )
    {
        reopen = env;
        wxUnsetEnv( wxS( "KICAD_OPENAPI_REOPEN" ) );
    }

    if( reopen.empty() )
    {
        m_impl->writeDiscoveryFile();
    }
    else
    {
        m_impl->publishThread = std::thread(
                [impl = m_impl.get(), reopen]()
                {
                    namespace fs = std::filesystem;
                    std::error_code ec;
                    const fs::path  want = fs::weakly_canonical( fs::path( reopen ), ec );
                    const auto      deadline = std::chrono::steady_clock::now() + std::chrono::seconds( 120 );

                    while( impl->alive->load() && std::chrono::steady_clock::now() < deadline )
                    {
                        KOPENAPI_CONTEXT ctx = impl->ctx;
                        KOPENAPI_RESULT  docs = runInMain( impl->alive, impl->waker,
                                                           [ctx]() mutable
                                                           {
                                                               return KOPENAPI_RESULT::Ok(
                                                                       KOPENAPI_REGISTRY::Get().Documents( ctx ) );
                                                           } );
                        bool open = false;

                        for( const nlohmann::json& d : docs.status == 200 ? docs.body : nlohmann::json::array() )
                        {
                            for( const char* key : { "path", "project" } )
                            {
                                std::error_code e;

                                if( d.contains( key ) && fs::weakly_canonical( fs::path( d.value( key, std::string() ) ), e ) == want )
                                    open = true;
                            }
                        }

                        if( open )
                            break;

                        std::this_thread::sleep_for( std::chrono::milliseconds( 200 ) );
                    }

                    if( impl->alive->load() )
                        impl->writeDiscoveryFile();
                } );
    }

    // A process without libraries is a silent failure for an agent: say so in the journal
    KopenapiCheckGlobalLibraryTables();

    std::fprintf( stderr, "kicadopenapi listening at http://%s:%d\n", KOPENAPI_HOST_ADDR,
                  m_impl->port );
    return true;
}


void KICAD_OPENAPI_SERVICE::Stop()
{
    if( !m_impl->server )
        return;

    KopenapiRecorderShutdown();   // a recording file must be finished while its window exists

    // Unblock workers waiting on main-thread calls first, then stop and join the listener
    // (which joins the worker pool).
    m_impl->alive->store( false );
    m_impl->server->stop();

    if( m_impl->preloadThread.joinable() )
        m_impl->preloadThread.join();

    if( m_impl->publishThread.joinable() )
        m_impl->publishThread.join();

    if( m_impl->thread.joinable() )
        m_impl->thread.join();

    m_impl->server.reset();
    m_impl->port = 0;
    s_currentPort = 0;

    // Headless documents live in the kifaces; drop them while the project/settings still exist
    KOPENAPI_REGISTRY::Get().ReleaseDocuments();

    if( !m_impl->discoveryFile.empty() )
    {
        std::error_code ec;
        fs::remove( m_impl->discoveryFile, ec );
        m_impl->discoveryFile.clear();
    }
}


bool KICAD_OPENAPI_SERVICE::NoModalUi()
{
    static const bool agent = std::getenv( "KICAD_OPENAPI_AGENT" ) != nullptr;
    return CurrentPort() > 0 && ( agent || s_apiCalls.load() > 0 );
}


void KICAD_OPENAPI_SERVICE::SetOverrideLock( bool aOverride )
{
    s_overrideLock = aOverride;
}


bool KICAD_OPENAPI_SERVICE::OverrideLock()
{
    return s_overrideLock;
}


bool KICAD_OPENAPI_SERVICE::Running() const
{
    return m_impl->server && m_impl->alive->load();
}


int KICAD_OPENAPI_SERVICE::Port() const
{
    return Running() ? m_impl->port : 0;
}
