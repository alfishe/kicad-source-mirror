#include "kicad_web_service.h"

#include <wx/log.h>

KICAD_WEB_SERVICE::KICAD_WEB_SERVICE( const std::string& aUtilityName, int aPort ) :
        m_utilityName( aUtilityName ),
        m_port( aPort ),
        m_server( std::make_unique<httplib::Server>() )
{
    register_builtin_endpoints();
}


KICAD_WEB_SERVICE::~KICAD_WEB_SERVICE()
{
    stop();
}


void KICAD_WEB_SERVICE::add_endpoint( const std::string& aMethod, const std::string& aPath,
                                      const std::string& aSummary, Handler aHandler )
{
    Handler handler = std::move( aHandler );

    if( aMethod == "GET" )
        m_server->Get( aPath, [handler]( const httplib::Request& req, httplib::Response& res ) {
            res.set_content( handler( req.body ), "application/json" );
        } );
    else if( aMethod == "POST" )
        m_server->Post( aPath, [handler]( const httplib::Request& req, httplib::Response& res ) {
            res.set_content( handler( req.body ), "application/json" );
        } );

    m_endpoints.push_back( { aMethod, aPath, aSummary, nullptr } );
}


void KICAD_WEB_SERVICE::register_builtin_endpoints()
{
    m_server->Get( "/", []( httplib::Request&, httplib::Response& res ) {
        res.set_content(
                "<html><head><title>kicadweb</title></head><body>"
                "<h1>kicadweb</h1><p>See <a href='/openapi.json'>/openapi.json</a></p>"
                "</body></html>",
                "text/html" );
    } );

    m_server->Get( "/openapi.json", [&]( httplib::Request&, httplib::Response& res ) {
        res.set_content( openapi_json(), "application/json" );
    } );

    m_server->Get( "/status", [&]( httplib::Request&, httplib::Response& res ) {
        res.set_content(
                std::string( "{\"utility\":\"" ) + m_utilityName + "\",\"port\":" +
                        std::to_string( m_port ) + "}",
                "application/json" );
    } );
}


std::string KICAD_WEB_SERVICE::openapi_json() const
{
    std::string paths;

    for( const EndpointSpec& ep : m_endpoints )
    {
        paths += std::string( "\"" ) + ep.path + "\": { \"" + ep.method + "\": { \"summary\": \"" +
                 ep.summary + "\" } },";
    }
    if( !paths.empty() )
        paths.pop_back();

    return std::string( "{\"openapi\":\"3.0.3\",\"info\":{\"title\":\"kicadweb " ) + m_utilityName +
           "\",\"version\":\"0.1.0\"},\"paths\":{" + paths + "}}";
}


bool KICAD_WEB_SERVICE::start()
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

    wxLogInfo( "kicadweb(%s): %s", m_utilityName,
               m_running ? "listening" : "failed to bind" );
    return m_running;
}


void KICAD_WEB_SERVICE::stop()
{
    if( !m_running )
        return;

    m_server->stop();
    if( m_thread.joinable() )
        m_thread.join();
    m_running = false;
}
