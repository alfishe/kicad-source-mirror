/*
 * Core kicadopenapi methods available in every host (GUI and headless).
 */
#include "kicadopenapi_registry.h"
#include "kicadopenapi_journal.h"

#include <algorithm>


static KOPENAPI_RESULT h_ping( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    // Reaching here proves the main thread is responsive (the service marshals every call)
    return KOPENAPI_RESULT::Ok( { { "pong", true }, { "headless", aCtx.headless } } );
}


KOPENAPI_REGISTER( "ping", "Round trip through the main thread (liveness and latency check)",
                   R"json({"type":"object","properties":{}})json"_json, false, h_ping );


static KOPENAPI_RESULT h_documents( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    return KOPENAPI_RESULT::Ok( { { "documents", KOPENAPI_REGISTRY::Get().Documents( aCtx ) } } );
}


KOPENAPI_REGISTER( "documents", "List open documents (schematic, PCB) with path, project and unsaved state",
                   R"json({"type":"object","properties":{}})json"_json, false, h_documents );


static KOPENAPI_RESULT h_errors( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    const uint64_t    since = aArgs.value( "since", static_cast<uint64_t>( 0 ) );
    const size_t      limit = std::clamp( aArgs.value( "limit", 100 ), 1, 1000 );
    const std::string level = aArgs.value( "level", std::string( "warning" ) );

    nlohmann::json out = KOPENAPI_JOURNAL::Summary();
    out["entries"] = KOPENAPI_JOURNAL::Entries( since, limit, level );

    if( aArgs.value( "clear", false ) )
        KOPENAPI_JOURNAL::Clear();

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "errors",
                   "Errors and warnings KiCad reported (never shown as dialogs): operation that caused "
                   "them, message, time, source location; poll with since=last_seq",
                   R"json({"type":"object","properties":{
                        "since":{"type":"integer","default":0,"description":"Only records with seq greater than this"},
                        "limit":{"type":"integer","default":100,"minimum":1,"maximum":1000},
                        "level":{"type":"string","enum":["warning","error"],"default":"warning",
                                 "description":"Minimum severity"},
                        "clear":{"type":"boolean","default":false,"description":"Drop collected records after reading"}}})json"_json,
                   false, h_errors );
