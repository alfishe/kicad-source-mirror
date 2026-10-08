#include "kicadopenapi_service.h"

#include "kicadopenapi_registry.h"

#include <wx/log.h>

KICAD_OPENAPI_SERVICE::KICAD_OPENAPI_SERVICE( const std::string& aUtilityName, int aPort ) :
        m_utilityName( aUtilityName ),
        m_port( aPort ),
        m_server( std::make_unique<httplib::Server>() )
{
    register_builtin_endpoints();
}


KICAD_OPENAPI_SERVICE::~KICAD_OPENAPI_SERVICE()
{
    stop();
}


void KICAD_OPENAPI_SERVICE::add_endpoint( const std::string& aMethod, const std::string& aPath,
                                          const std::string& aSummary, JsonHandler aHandler )
{
    JsonHandler handler = std::move( aHandler );

    if( aMethod == "GET" )
        m_server->Get( aPath, [handler]( const httplib::Request& req, httplib::Response& res ) {
            res.set_content( handler( req.body ), "application/json" );
        } );
    else if( aMethod == "POST" )
        m_server->Post( aPath, [handler]( const httplib::Request& req, httplib::Response& res ) {
            res.set_content( handler( req.body ), "application/json" );
        } );

    m_endpoints.push_back( { aMethod, aPath, aSummary } );
}


void KICAD_OPENAPI_SERVICE::set_host( KOPENAPI_HOST* aHost )
{
    m_host = aHost;
}


void KICAD_OPENAPI_SERVICE::register_builtin_endpoints()
{
    m_server->Get( "/", []( const httplib::Request&, httplib::Response& res ) {
        // Embedded swagger-ui webapp (static; assets from CDN) over /openapi.json.
        res.set_content(
                "<!DOCTYPE html><html><head><title>kicadopenapi</title>"
                "<meta charset='utf-8'/>"
                "<link rel='stylesheet' href='https://unpkg.com/swagger-ui-dist@5/swagger-ui.css'>"
                "</head><body><div id='swagger-ui'></div>"
                "<script src='https://unpkg.com/swagger-ui-dist@5/swagger-ui-bundle.js'></script>"
                "<script>window.onload=()=>{SwaggerUIBundle({url:'/openapi.json',"
                "dom_id:'#swagger-ui'})};</script></body></html>",
                "text/html" );
    } );

    m_server->Get( "/openapi.json", [&]( const httplib::Request&, httplib::Response& res ) {
        res.set_content( openapi_json(), "application/json" );
    } );

    m_server->Get( "/status", [&]( const httplib::Request&, httplib::Response& res ) {
        res.set_content(
                std::string( "{\"utility\":\"" ) + m_utilityName + "\",\"port\":" +
                        std::to_string( m_port ) + "}",
                "application/json" );
    } );

    m_server->Get( "/mcp/search", [&]( const httplib::Request& req, httplib::Response& res ) {
        std::string q = req.has_param( "q" ) ? req.get_param_value( "q" ) : "";
        std::string out = "[";
        size_t      n = 0;

        for( const KOPENAPI_METHOD& m : KOPENAPI_REGISTRY::Get().Search( q, 20 ) )
        {
            if( n++ )
                out += ",";
            out += "{\"name\":\"" + m.name + "\",\"summary\":\"" + m.summary + "\"}";
        }
        res.set_content( out + "]", "application/json" );
    } );

    // MCP invoke: POST /mcp/invoke/{name} with the JSON arguments as body.
    m_server->Post( R"(/mcp/invoke/([A-Za-z0-9_]+))",
                    [&]( const httplib::Request& req, httplib::Response& res )
                    {
                        const std::string name = req.matches[1];
                        const KOPENAPI_METHOD* method = KOPENAPI_REGISTRY::Get().Find( name );

                        if( !method )
                        {
                            res.status = 404;
                            res.set_content( "{\"error\":\"unknown_method\",\"name\":\"" + name +
                                                     "\"}",
                                             "application/json" );
                            return;
                        }

                        if( !m_host )
                        {
                            res.status = 503;
                            res.set_content( "{\"error\":\"no host bound\"}",
                                             "application/json" );
                            return;
                        }

                        res.set_content( method->handler( *m_host, req.body ),
                                         "application/json" );
                    } );
}


std::string KICAD_OPENAPI_SERVICE::openapi_json() const
{
    std::string paths;

    for( const auto& [name, method] : KOPENAPI_REGISTRY::Get().Methods() )
    {
        std::string escaped = name;
        paths += std::string( "\"/api/" ) + escaped + "\": { \"post\": { \"summary\": \"" +
                 method.summary + "\" } },";
    }

    for( const EndpointSpec& ep : m_endpoints )
    {
        paths += std::string( "\"" ) + ep.path + "\": { \"" + ep.method + "\": { \"summary\": \"" +
                 ep.summary + "\" } },";
    }
    if( !paths.empty() )
        paths.pop_back();

    return std::string( "{\"openapi\":\"3.0.3\",\"info\":{\"title\":\"kicadopenapi " ) +
           m_utilityName + "\",\"version\":\"0.1.0\"},\"paths\":{" + paths + "}}";
}


bool KICAD_OPENAPI_SERVICE::start()
{
    if( m_running )
        return true;

    m_thread = std::thread(
            [this]()
            {
                m_running = m_server->bind_to_port( "127.0.0.1", m_port ) &&
                            m_server->listen_after_bind();
            } );

    for( int i = 0; i < 50 && !m_running; ++i )
        std::this_thread::sleep_for( std::chrono::milliseconds( 20 ) );

    wxLogInfo( "kicadopenapi(%s): %s", m_utilityName,
               m_running ? "listening" : "failed to bind" );
    return m_running;
}


void KICAD_OPENAPI_SERVICE::stop()
{
    if( !m_running )
        return;

    m_server->stop();
    if( m_thread.joinable() )
        m_thread.join();
    m_running = false;
}
