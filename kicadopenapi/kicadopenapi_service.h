/*
 * In-process HTTP service embedded into KiCad utilities (eeschema, pcbnew, PGM).
 *
 * Serves a generated OpenAPI spec, a static test webui, and REST endpoints with
 * direct access to the owning frame's object model. Runs on 127.0.0.1 only.
 */
#ifndef KICAD_OPENAPI_SERVICE_H
#define KICAD_OPENAPI_SERVICE_H

#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <thread>

#include <httplib.h>

class KICAD_OPENAPI_SERVICE
{
public:
    using Handler = std::function<std::string( const std::string& aBody )>;

    KICAD_OPENAPI_SERVICE( const std::string& aUtilityName, int aPort );
    ~KICAD_OPENAPI_SERVICE();

    void add_endpoint( const std::string& aMethod, const std::string& aPath,
                       const std::string& aSummary, Handler aHandler );

    bool start();
    void stop();

    int port() const { return m_port; }

private:
    void register_builtin_endpoints();
    std::string openapi_json() const;

    std::string m_utilityName;
    int m_port;
    std::atomic<bool> m_running{ false };
    std::thread m_thread;
    std::unique_ptr<httplib::Server> m_server;

    struct EndpointSpec
    {
        std::string method;
        std::string path;
        std::string summary;
        Handler handler;
    };
    std::vector<EndpointSpec> m_endpoints;
};

#endif
