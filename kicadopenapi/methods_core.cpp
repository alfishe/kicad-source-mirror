/*
 * Core kicadopenapi methods available in every host (GUI and headless).
 */
#include "kicadopenapi_registry.h"


static KOPENAPI_RESULT h_ping( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    // Reaching here proves the main thread is responsive (the service marshals every call)
    return KOPENAPI_RESULT::Ok( { { "pong", true }, { "headless", aCtx.headless } } );
}


KOPENAPI_REGISTER( "ping", "Round trip through the main thread (liveness and latency check)",
                   R"({"type":"object","properties":{}})"_json, false, h_ping );
