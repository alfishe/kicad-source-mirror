#include "kicadopenapi_mcp.h"

#include <mcp_search.h>
#include <mcp_tools.h>

#include <algorithm>
#include <limits>
#include <optional>

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

    // A method returning an image (image_base64 + mime_type) becomes MCP image content, so the
    // agent sees a picture; the base64 is not repeated in the text / structured parts
    json image;

    if( aData.contains( "image_base64" ) && aData["image_base64"].is_string() )
    {
        image = { { "type", "image" },
                  { "data", aData["image_base64"] },
                  { "mimeType", aData.value( "mime_type", std::string( "image/png" ) ) } };
        aData.erase( "image_base64" );
        aData["image"] = "returned as MCP image content";
    }

    // Several images (e.g. one per board layer): "images": [{ image_base64, mime_type, ... }]
    json images = json::array();

    if( aData.contains( "images" ) && aData["images"].is_array() )
    {
        for( json& item : aData["images"] )
        {
            if( item.is_object() && item.contains( "image_base64" ) && item["image_base64"].is_string() )
            {
                images.push_back( { { "type", "image" },
                                    { "data", item["image_base64"] },
                                    { "mimeType", item.value( "mime_type", std::string( "image/png" ) ) } } );
                item.erase( "image_base64" );
                item["image"] = "returned as MCP image content #" + std::to_string( images.size() );
            }
        }
    }

    std::string text = aData.dump( 2, ' ', false, json::error_handler_t::replace );
    json        content = json::array( { { { "type", "text" }, { "text", text } } } );

    if( !image.is_null() )
        content.push_back( std::move( image ) );

    for( json& item : images )
        content.push_back( std::move( item ) );

    return { { "content", std::move( content ) },
             { "structuredContent", std::move( aData ) },
             { "isError", aIsError } };
}


static json searchTool( const KOPENAPI_MCP_CONTEXT& aCtx, const json& aArgs )
{
    const std::string query = aArgs.value( "query", std::string() );
    const int         limit = std::clamp( aArgs.value( "limit", 20 ), 1, 100 );
    const bool        full = aArgs.value( "detail", std::string( "brief" ) ) == "full";

    // names: exact methods, described in full
    if( aArgs.contains( "names" ) && aArgs["names"].is_array() )
    {
        json methods = json::array(), missing = json::array();

        for( const json& name : aArgs["names"] )
        {
            std::optional<KOPENAPI_METHOD> m = name.is_string() ? KOPENAPI_REGISTRY::Get().Find( name.get<std::string>() )
                                                                : std::nullopt;

            if( m && ( !m->guiOnly || !aCtx.headless ) )
                methods.push_back( kopenapi::mcp::FormatMethod( m->name, m->summary, m->inputSchema, m->guiOnly, true ) );
            else
                missing.push_back( name );
        }

        return toolResult( { { "methods", methods }, { "count", methods.size() }, { "not_found", missing } }, false );
    }

    json methods = json::array();
    json more = json::array();

    // gui-only methods are hidden headless: they could only fail there.  Matches beyond the
    // limit are not dropped silently: they come back in "more" as name + summary only.
    const std::vector<KOPENAPI_METHOD> found =
            KOPENAPI_REGISTRY::Get().Search( query, std::numeric_limits<size_t>::max(), !aCtx.headless );

    for( const KOPENAPI_METHOD& m : found )
    {
        if( methods.size() < (size_t) limit )
            methods.push_back( kopenapi::mcp::FormatMethod( m.name, m.summary, m.inputSchema, m.guiOnly, full ) );
        else
            more.push_back( { { "name", m.name }, { "summary", m.summary } } );
    }

    return toolResult( { { "methods", methods }, { "count", methods.size() }, { "total", found.size() }, { "more", more } },
                       false );
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
