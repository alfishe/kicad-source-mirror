#include "kicadopenapi_service.h"
#include "kicadopenapi_registry.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <thread>

#include <httplib.h>

#include <wx/app.h>

#include <platform.h>

static const char* const KOPENAPI_VERSION = "0.2.0";
static const char* const KOPENAPI_HOST_ADDR = "127.0.0.1";

/// Upper bound for one main-thread call; long operations will move to async jobs.
static constexpr std::chrono::seconds CALL_TIMEOUT( 15 );
static constexpr std::chrono::milliseconds WAIT_SLICE( 20 );

namespace fs = std::filesystem;


/// Drops discovery files left behind by crashed processes
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


/**
 * Run aFn on the main (GUI/event-loop) thread and wait for its result on the calling HTTP
 * worker.  The main thread never waits on HTTP threads.  If the service stops or the call
 * exceeds CALL_TIMEOUT, the worker returns an error; a late closure still completes safely
 * because it only touches shared state.
 */
static KOPENAPI_RESULT runInMain( const std::shared_ptr<std::atomic<bool>>& aAlive,
                                  const std::function<void()>&             aWaker,
                                  std::function<KOPENAPI_RESULT()>         aFn )
{
    if( !wxTheApp )
        return KOPENAPI_RESULT::Error( 503, "no application event loop" );

    auto promise = std::make_shared<std::promise<KOPENAPI_RESULT>>();
    std::future<KOPENAPI_RESULT> future = promise->get_future();

    wxTheApp->CallAfter(
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

    if( aWaker )
        aWaker();

    const auto deadline = std::chrono::steady_clock::now() + CALL_TIMEOUT;

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
                 { "headless", ctx.headless } };
    }

    nlohmann::json openApiJson() const;

    void writeDiscoveryFile();

    KOPENAPI_RESULT invoke( const std::string& aName, const std::string& aBody ) const;

    void registerRoutes();
};


nlohmann::json KICAD_OPENAPI_SERVICE::IMPL::openApiJson() const
{
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
                        { "headless", { { "type", "boolean" } } } } } } } } } } }
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
    std::optional<KOPENAPI_METHOD> method = KOPENAPI_REGISTRY::Get().Find( aName );

    if( !method )
        return KOPENAPI_RESULT::Error( 404, "unknown method '" + aName + "'" );

    if( method->guiOnly && ctx.headless )
        return KOPENAPI_RESULT::Error( 501, "'" + aName + "' requires the GUI" );

    nlohmann::json args = nlohmann::json::object();

    if( !aBody.empty() )
    {
        args = nlohmann::json::parse( aBody, nullptr, false );

        if( args.is_discarded() || !args.is_object() )
            return KOPENAPI_RESULT::Error( 400, "request body must be a JSON object" );
    }

    if( method->inputSchema.contains( "required" ) )
    {
        for( const nlohmann::json& field : method->inputSchema["required"] )
        {
            if( field.is_string() && !args.contains( field.get<std::string>() ) )
                return KOPENAPI_RESULT::Error( 400, "missing required field '"
                                                            + field.get<std::string>() + "'" );
        }
    }

    KOPENAPI_CONTEXT ctxCopy = ctx;
    KOPENAPI_HANDLER handler = method->handler;

    return runInMain( alive, waker,
                      [ctxCopy, handler, args]() mutable { return handler( ctxCopy, args ); } );
}


void KICAD_OPENAPI_SERVICE::IMPL::registerRoutes()
{
    // Only SO_REUSEADDR: httplib's default SO_REUSEPORT would let two KiCad processes
    // silently share one port.
    server->set_socket_options(
            []( socket_t aSock )
            {
                int yes = 1;
                setsockopt( aSock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>( &yes ),
                            sizeof( yes ) );
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
                     reply( res, KOPENAPI_RESULT::Ok( statusJson() ) );
                 } );

    server->Get( "/api/v1/openapi.json",
                 [this]( const httplib::Request&, httplib::Response& res )
                 {
                     reply( res, KOPENAPI_RESULT::Ok( openApiJson() ) );
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


void KICAD_OPENAPI_SERVICE::SetMainLoopWaker( std::function<void()> aWaker )
{
    m_impl->waker = std::move( aWaker );
}


void KICAD_OPENAPI_SERVICE::SetShutdownHandler( std::function<void()> aHandler )
{
    m_impl->shutdownHandler = std::move( aHandler );
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

    m_impl->appName = wxTheApp ? wxTheApp->GetAppName().ToStdString() : std::string( "kicad" );
    m_impl->server = std::make_unique<httplib::Server>();
    m_impl->registerRoutes();
    m_impl->port = 0;

    for( int i = 0; i < PORT_PROBE_COUNT; ++i )
    {
        if( m_impl->server->bind_to_port( KOPENAPI_HOST_ADDR, basePort + i ) )
        {
            m_impl->port = basePort + i;
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

    // The socket is already bound, so clients can connect as soon as Start() returns;
    // listen_after_bind() blocks in this thread until Stop().
    m_impl->thread = std::thread( [srv = m_impl->server.get()]() { srv->listen_after_bind(); } );

    m_impl->writeDiscoveryFile();

    std::fprintf( stderr, "kicadopenapi listening at http://%s:%d\n", KOPENAPI_HOST_ADDR,
                  m_impl->port );
    return true;
}


void KICAD_OPENAPI_SERVICE::Stop()
{
    if( !m_impl->server )
        return;

    // Unblock workers waiting on main-thread calls first, then stop and join the listener
    // (which joins the worker pool).
    m_impl->alive->store( false );
    m_impl->server->stop();

    if( m_impl->thread.joinable() )
        m_impl->thread.join();

    m_impl->server.reset();
    m_impl->port = 0;

    if( !m_impl->discoveryFile.empty() )
    {
        std::error_code ec;
        fs::remove( m_impl->discoveryFile, ec );
        m_impl->discoveryFile.clear();
    }
}


bool KICAD_OPENAPI_SERVICE::Running() const
{
    return m_impl->server && m_impl->alive->load();
}


int KICAD_OPENAPI_SERVICE::Port() const
{
    return Running() ? m_impl->port : 0;
}
