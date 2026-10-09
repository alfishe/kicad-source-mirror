/*
 * kicadopenapi MCP dispatcher (JSON-RPC 2.0 over Streamable HTTP, POST /mcp).
 *
 * Advertises exactly two tools, `search` and `invoke` (shared/mcp_tools.h); methods are
 * discovered through the registry, never listed as tools.  Transport-agnostic: the service
 * feeds parsed messages and gets responses back.
 */
#ifndef KICADOPENAPI_MCP_H
#define KICADOPENAPI_MCP_H

#include <functional>
#include <optional>
#include <string>

#include "kicadopenapi_registry.h"


struct KOPENAPI_MCP_CONTEXT
{
    bool        headless = false;
    std::string version;

    /// Runs a registry method exactly like POST /api/v1/{name}
    std::function<KOPENAPI_RESULT( const std::string&, const nlohmann::json& )> invoke;
};


/// One JSON-RPC message → response, or nullopt for notifications (answered with HTTP 202)
std::optional<nlohmann::json> KopenapiMcpHandle( const KOPENAPI_MCP_CONTEXT& aCtx,
                                                 const nlohmann::json&       aMessage );

#endif
