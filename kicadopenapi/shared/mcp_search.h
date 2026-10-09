#pragma once

// Formatting of MCP `search` results, shared by the in-process /mcp endpoint and
// kicad-mcp-bridge.  Default output is compact — name, summary, one line of parameters — so a
// search costs few tokens; the full inputSchema comes with detail: "full" or for the methods
// named in `names`.  Header-only; needs nlohmann::json.

#include <nlohmann/json.hpp>

#include <string>

namespace kopenapi::mcp
{

/// "path*: string, side: left|right = \"auto\", limit: integer = 20" (* = required)
inline std::string BriefParams( const nlohmann::json& aSchema )
{
    if( !aSchema.is_object() || !aSchema.contains( "properties" ) || !aSchema["properties"].is_object() )
        return std::string();

    nlohmann::json required = aSchema.value( "required", nlohmann::json::array() );
    std::string    out;

    for( const auto& [name, prop] : aSchema["properties"].items() )
    {
        std::string type = "any";

        if( prop.contains( "enum" ) && prop["enum"].is_array() )
        {
            type.clear();

            for( const nlohmann::json& value : prop["enum"] )
                type += ( type.empty() ? "" : "|" ) + ( value.is_string() ? value.get<std::string>() : value.dump() );
        }
        else if( prop.contains( "type" ) && prop["type"].is_string() )
        {
            type = prop["type"].get<std::string>();

            if( type == "array" && prop.contains( "items" ) && prop["items"].is_object() && prop["items"].contains( "type" ) )
                type = prop["items"]["type"].get<std::string>() + "[]";
        }

        bool isRequired = false;

        for( const nlohmann::json& r : required )
            isRequired |= r == name;

        out += ( out.empty() ? "" : ", " ) + name + ( isRequired ? "*" : "" ) + ": " + type;

        if( prop.contains( "default" ) )
            out += " = " + prop["default"].dump();
    }

    return out;
}


/// One method entry: aDetail "full" keeps inputSchema, else params as one line
inline nlohmann::json FormatMethod( const std::string& aName, const std::string& aSummary,
                                    const nlohmann::json& aSchema, bool aGuiOnly, bool aFull )
{
    nlohmann::json entry = { { "name", aName }, { "summary", aSummary } };

    if( aFull )
        entry["inputSchema"] = aSchema;
    else
        entry["params"] = BriefParams( aSchema );

    if( aGuiOnly )
        entry["gui_only"] = true;

    return entry;
}

} // namespace kopenapi::mcp
