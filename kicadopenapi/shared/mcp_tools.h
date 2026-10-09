#pragma once

// The MCP tool surface of kicadopenapi, shared verbatim by the in-process /mcp endpoint and
// kicad-mcp-bridge. Only two tools are ever advertised: methods are discovered with
// `search` and called with `invoke`, so adding a method never changes the tool list.
// Header-only, no dependencies.

namespace kopenapi::mcp
{

inline constexpr const char* kProtocolVersion = "2025-03-26";

/// Protocol versions accepted from clients (anything else is answered with kProtocolVersion)
inline constexpr const char* kSupportedVersions[] = { "2025-06-18", "2025-03-26", "2024-11-05" };

inline constexpr const char* kSearchTool = "search";
inline constexpr const char* kInvokeTool = "invoke";

/// tools/list result "tools" array
inline constexpr const char* kToolsJson = R"json([
  {
    "name": "search",
    "description": "Find KiCad methods by keywords (e.g. 'open pcb', 'symbol', 'drc', 'instance'). Returns each method's name, summary, inputSchema and whether it needs the GUI. Empty query lists all methods. Call invoke with a returned name.",
    "inputSchema": {
      "type": "object",
      "properties": {
        "query": { "type": "string", "description": "Keywords; empty lists everything" },
        "limit": { "type": "integer", "minimum": 1, "maximum": 100, "default": 20 }
      }
    }
  },
  {
    "name": "invoke",
    "description": "Call a KiCad method found with search. 'arguments' must match the method's inputSchema. Returns the method's JSON result; failures come back with isError=true and an error message.",
    "inputSchema": {
      "type": "object",
      "required": [ "name" ],
      "properties": {
        "name": { "type": "string", "description": "Method name from search" },
        "arguments": { "type": "object", "description": "Method arguments", "default": {} }
      }
    }
  }
])json";

} // namespace kopenapi::mcp
