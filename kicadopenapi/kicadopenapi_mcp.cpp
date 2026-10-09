#include "kicadopenapi_mcp.h"

#include <mcp_tools.h>

using nlohmann::json;

namespace mcp = kopenapi::mcp;


static json rpcResult( const json& aId, json aResult )
{
    return { { "jsonrpc", "2.0" }, { "id", aId }, { "result", std::move( aResult ) } };
}


static json rpcError( const json& aId, int aCode, const std::string& aMessage )
{
    return { { "jsonrpc", "2.0" },
             { "id", aId },
             { "error", { { "code", aCode }, { "message", aMessage } } } };
}


/// MCP tool result: JSON text for any client plus structuredContent for typed clients
static json toolResult( json aData, bool aIsError )
{
    if( !aData.is_object() )
        aData = { { "result", std::move( aData ) } };

    std::string text = aData.dump( 2, ' ', false, json::error_handler_t::replace );

    return { { "content", json::array( { { { "type", "text" }, { "text", text } } } ) },
             { "structuredContent", std::move( aData ) },
             { "isError", aIsError } };
}


static json searchTool( const KOPENAPI_MCP_CONTEXT& aCtx, const json& aArgs )
{
    const std::string query = aArgs.value( "query", std::string() );
    const int         limit = std::clamp( aArgs.value( "limit", 20 ), 1, 100 );

    json methods = json::array();

    // gui-only methods are hidden headless: they could only fail there
    for( const KOPENAPI_METHOD& m :
         KOPENAPI_REGISTRY::Get().Search( query, (size_t) limit, !aCtx.headless ) )
    {
        methods.push_back( { { "name", m.name },
                             { "summary", m.summary },
                             { "inputSchema", m.inputSchema },
                             { "gui_only", m.guiOnly } } );
    }

    return toolResult( { { "methods", methods }, { "count", methods.size() } }, false );
}


static json invokeTool( const KOPENAPI_MCP_CONTEXT& aCtx, const json& aArgs )
{
    if( !aArgs.contains( "name" ) || !aArgs["name"].is_string() )
        return toolResult( { { "error", "'name' (string) is required; find names with search" } }, true );

    json args = aArgs.value( "arguments", json::object() );

    if( args.is_null() )
        args = json::object();

    if( !args.is_object() )
        return toolResult( { { "error", "'arguments' must be an object" } }, true );

    KOPENAPI_RESULT result = aCtx.invoke( aArgs["name"].get<std::string>(), args );
    return toolResult( result.body, result.status >= 400 );
}


std::optional<json> KopenapiMcpHandle( const KOPENAPI_MCP_CONTEXT& aCtx, const json& aMessage )
{
    if( !aMessage.is_object() || aMessage.value( "jsonrpc", std::string() ) != "2.0"
        || !aMessage.contains( "method" ) || !aMessage["method"].is_string() )
    {
        return rpcError( aMessage.is_object() ? aMessage.value( "id", json() ) : json(), -32600,
                         "invalid request" );
    }

    const std::string method = aMessage["method"].get<std::string>();

    if( !aMessage.contains( "id" ) )
        return std::nullopt;   // notification

    const json  id = aMessage["id"];
    const json& params = aMessage.contains( "params" ) && aMessage["params"].is_object()
                                 ? aMessage["params"]
                                 : json::object();

    if( method == "initialize" )
    {
        std::string version = params.value( "protocolVersion", std::string() );
        bool        supported = false;

        for( const char* v : mcp::kSupportedVersions )
            supported |= version == v;

        return rpcResult( id, { { "protocolVersion", supported ? version : mcp::kProtocolVersion },
                                { "capabilities", { { "tools", json::object() } } },
                                { "serverInfo",
                                  { { "name", "kicadopenapi" }, { "version", aCtx.version } } },
                                { "instructions",
                                  "Use search to find KiCad methods, then invoke to call them." } } );
    }

    if( method == "ping" )
        return rpcResult( id, json::object() );

    if( method == "tools/list" )
        return rpcResult( id, { { "tools", json::parse( mcp::kToolsJson ) } } );

    if( method == "tools/call" )
    {
        const std::string name = params.value( "name", std::string() );
        json              args = params.value( "arguments", json::object() );

        if( name == mcp::kSearchTool )
            return rpcResult( id, searchTool( aCtx, args.is_object() ? args : json::object() ) );

        if( name == mcp::kInvokeTool )
            return rpcResult( id, invokeTool( aCtx, args.is_object() ? args : json::object() ) );

        return rpcError( id, -32602, "unknown tool '" + name + "' (tools: search, invoke)" );
    }

    return rpcError( id, -32601, "method not found: " + method );
}
