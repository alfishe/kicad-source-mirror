/*
 * Core kicadopenapi methods available in every host (GUI and headless).
 */
#include "kicadopenapi_registry.h"
#include "kicadopenapi_journal.h"

#include <frame_type.h>
#include <kiway.h>
#include <kiway_player.h>

#include <wx/app.h>
#include <wx/log.h>

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


/**
 * Quit the GUI process without any dialog.  Unsaved documents block unless discard: true; then
 * each document is closed through its domain's *_close {discard: true} (a journal warning each)
 * and the top window closes after this answer has gone out.  Library editors with unsaved
 * library changes always block: KiCad can only ask about those, and a question would hang the
 * agent.
 */
static KOPENAPI_RESULT h_app_quit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const bool     discard = aArgs.value( "discard", false );
    nlohmann::json documents = KOPENAPI_REGISTRY::Get().Documents( aCtx );
    nlohmann::json unsaved = nlohmann::json::array();

    for( const nlohmann::json& doc : documents )
    {
        if( doc.value( "unsaved", false ) )
            unsaved.push_back( doc );
    }

    nlohmann::json libraries = nlohmann::json::array();

    for( const auto& [type, name] : { std::pair{ FRAME_SCH_SYMBOL_EDITOR, "symbol editor" },
                                      std::pair{ FRAME_FOOTPRINT_EDITOR, "footprint editor" } } )
    {
        if( KIWAY_PLAYER* editor = aCtx.kiway ? aCtx.kiway->Player( type, false ) : nullptr;
            editor && editor->IsContentModified() )
        {
            libraries.push_back( name );
        }
    }

    if( !libraries.empty() )
    {
        return KOPENAPI_RESULT::Error( 409, "library editors have unsaved library changes (" + libraries.dump()
                                                    + "): save or close them first" );
    }

    if( !unsaved.empty() && !discard )
        return KOPENAPI_RESULT::Error( 409, "unsaved documents: " + unsaved.dump() + "; save them or pass discard: true" );

    nlohmann::json closed = nlohmann::json::array();

    for( const nlohmann::json& doc : documents )
    {
        const std::string domain = doc.value( "domain", std::string() );

        if( std::optional<KOPENAPI_METHOD> close = KOPENAPI_REGISTRY::Get().Find( domain + "_close" ) )
        {
            KOPENAPI_RESULT result = close->handler( aCtx, { { "discard", true } } );

            if( result.status == 200 )
                closed.push_back( doc );
        }
    }

    // After the answer has been written: closing the top window ends the process
    wxTheApp->CallAfter(
            []()
            {
                if( wxWindow* top = wxTheApp->GetTopWindow() )
                    top->Close( true );
            } );

    return KOPENAPI_RESULT::Ok( { { "quitting", true }, { "closed", closed }, { "discarded", discard ? unsaved : nlohmann::json::array() } } );
}


KOPENAPI_REGISTER( "app_quit",
                   "Quit KiCad (GUI) without any dialog: refuses while documents are unsaved unless "
                   "discard: true, which drops the changes (journal warnings) and closes the editors; "
                   "library editors with unsaved library changes always refuse. Headless: POST "
                   "/api/v1/shutdown (unsaved work is lost with a journal warning)",
                   R"json({"type":"object","properties":{"discard":{"type":"boolean","default":false}}})json"_json,
                   true, h_app_quit );
