/*
 * kicadopenapi method registry.
 *
 * Every Web API method is declared once, next to its handler, and registers itself at
 * static initialization:
 *
 *   static KOPENAPI_RESULT h_open_pcb( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );
 *
 *   KOPENAPI_REGISTER( "open_pcb", "Open a .kicad_pcb in the PCB editor window",
 *                      R"({"type":"object","required":["path"],
 *                          "properties":{"path":{"type":"string"}}})"_json,
 *                      true, h_open_pcb );
 *
 * The registry is the single source of truth: the service routes POST /api/v1/{name}
 * through it and generates /api/v1/openapi.json from it, so the manifest can only list
 * what is actually served.
 *
 * The registry lives in kicommon so that methods registered from kiface DSOs land in the
 * same (process-wide) instance.
 */
#ifndef KICADOPENAPI_REGISTRY_H
#define KICADOPENAPI_REGISTRY_H

#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <json_common.h>
#include <kicommon.h>

class KIWAY;


/// Process-level surface handed to every handler; handlers always run on the main thread.
struct KOPENAPI_CONTEXT
{
    KIWAY* kiway = nullptr;
    bool   headless = false;
};


/// Handler outcome: HTTP status + JSON body.  Errors use { "error": { code, message } }.
struct KOPENAPI_RESULT
{
    int            status = 200;
    nlohmann::json body = nlohmann::json::object();

    static KOPENAPI_RESULT Ok( nlohmann::json aBody ) { return { 200, std::move( aBody ) }; }

    static KOPENAPI_RESULT Error( int aStatus, const std::string& aMessage )
    {
        return { aStatus, { { "error", { { "code", aStatus }, { "message", aMessage } } } } };
    }
};


using KOPENAPI_HANDLER =
        std::function<KOPENAPI_RESULT( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )>;


struct KOPENAPI_METHOD
{
    std::string      name;         ///< [a-z0-9_]+, served at POST /api/v1/{name}
    std::string      summary;
    nlohmann::json   inputSchema;  ///< JSON Schema of the request body (type: object)
    bool             guiOnly;      ///< refused (501) when the process is headless
    KOPENAPI_HANDLER handler;
};


/// Thread-safe: kiface DSOs register while HTTP workers read; readers get copies.
class KICOMMON_API KOPENAPI_REGISTRY
{
public:
    static KOPENAPI_REGISTRY& Get();

    static bool Add( KOPENAPI_METHOD aMethod );

    std::vector<KOPENAPI_METHOD> Snapshot() const;

    std::optional<KOPENAPI_METHOD> Find( const std::string& aName ) const;

private:
    KOPENAPI_REGISTRY() = default;

    mutable std::mutex                     m_mutex;
    std::map<std::string, KOPENAPI_METHOD> m_methods;
};


#define KOPENAPI_REGISTER_IMPL2( line, ... )                                             \
    static const bool kopenapi_reg_##line = KOPENAPI_REGISTRY::Add( KOPENAPI_METHOD{ __VA_ARGS__ } )
#define KOPENAPI_REGISTER_IMPL( line, ... ) KOPENAPI_REGISTER_IMPL2( line, __VA_ARGS__ )
#define KOPENAPI_REGISTER( ... ) KOPENAPI_REGISTER_IMPL( __LINE__, __VA_ARGS__ )

#endif
