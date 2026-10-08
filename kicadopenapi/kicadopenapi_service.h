/*
 * kicadopenapi — in-process REST/OpenAPI/webui service embedded into KiCad
 * utilities (eeschema, pcbnew, PGM).
 *
 * Serves the method registry as OpenAPI + swagger-ui webapp, an MCP-style
 * /mcp/search and /mcp/invoke/{name} surface, and REST endpoints with direct
 * access to the owning frame's object model. Runs on 127.0.0.1 only.
 */
#ifndef KICAD_OPENAPI_SERVICE_H
#define KICAD_OPENAPI_SERVICE_H

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <httplib.h>

#include "kicadopenapi_registry.h"

class KICAD_OPENAPI_SERVICE
{
public:
    using JsonHandler = std::function<std::string( const std::string& aBody )>;

    KICAD_OPENAPI_SERVICE( const std::string& aUtilityName, int aPort );
    ~KICAD_OPENAPI_SERVICE();

    void add_endpoint( const std::string& aMethod, const std::string& aPath,
                       const std::string& aSummary, JsonHandler aHandler );

    /** Bind the object-model host; handlers receive it on every invoke. */
    void set_host( KOPENAPI_HOST* aHost );

    bool start();
    void stop();

    int port() const { return m_port; }

private:
    void register_builtin_endpoints();
    std::string openapi_json() const;

    std::string m_utilityName;
    int m_port;
    KOPENAPI_HOST* m_host = nullptr;
    std::atomic<bool> m_running{ false };
    std::thread m_thread;
    std::unique_ptr<httplib::Server> m_server;

    struct EndpointSpec
    {
        std::string method;
        std::string path;
        std::string summary;
    };
    std::vector<EndpointSpec> m_endpoints;
};

#endif
