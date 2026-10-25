/// @file methods_pcb_query.cpp
/// @brief pcb_query: select board items with a KiCad DRC condition, shape them into rows / groups.

#include "kopenapi_pcb.h"
#include "kopenapi_pcb_query.h"

#include <api/pcb_context.h>
#include <board.h>


static KOPENAPI_RESULT h_pcb_query( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    try
    {
        return KopenapiPcbQuery( context->GetBoard(), aArgs );
    }
    catch( const nlohmann::json::exception& e )
    {
        return KOPENAPI_RESULT::Error( 400, std::string( "malformed request: " ) + e.what() );
    }
    catch( const std::exception& e )
    {
        return KOPENAPI_RESULT::Error( 400, std::string( "malformed request: " ) + e.what() );
    }
}


KOPENAPI_REGISTER( "pcb_query",
                   "Query board items with a KiCad DRC rule condition (A.<Property> with spaces as underscores, "
                   "==, !=, <, >, &&, ||, !, numbers need units (mm / mil / in / deg), A.NetName for the net name "
                   "(A.Net is the net code), KiCad's functions such as "
                   "A.intersectsCourtyard('U1'), A.enclosedByArea('name')) plus query functions: A.side() "
                   "('top' / 'bottom' / 'both' / 'inner'), A.mount() ('smd' / 'tht' / 'other'), A.parentRef(), "
                   "A.courtyard() ('x0 y0 x1 y1' mm), A.inRect(x0, y0, x1, y1) / A.touchesRect(...) (mm, or "
                   "numbers with units), A.inPolygon('x0 y0, x1 y1, ...'), A.inArea('name'); a call with no item "
                   "(inRect(...)) applies to A. Read-only. "
                   "Answer: count, items (select fields; properties as in KiCad's Properties panel, lengths in mm) "
                   "or groups (group_by + aggregate: count, sum(x), min(x), max(x), avg(x), distinct(x)); "
                   "zones are the board's and the footprints' (each zone row's owner: the footprint's reference, or "
                   "'board'; A.parentRef() filters by it), "
                   "next_cursor, timing_ms, scanned; explain: true gives each top-level && term per item, "
                   "'misses' adds items failing exactly one term. Errors 400 carry the offset and close names",
                   R"json({"type":"object","properties":{
                        "from":{"oneOf":[{"type":"string"},{"type":"array","items":{"type":"string","enum":["footprints","pads","tracks","vias","zones","drawings","texts","groups","items"]}}],"default":"footprints"},
                        "where":{"type":"string","description":"KiCad DRC condition, e.g. A.side() == 'bottom' && A.mount() == 'smd'"},
                        "select":{"type":"array","items":{"type":"string"},"description":"property names (Reference, Value, Layer, Net, Width, Position_X ...) or functions (A.side(); a bare parentRef / side / mount / courtyard is the same); a name that is neither for the queried kinds is a 400 with close names (as group_by / order_by / aggregate)"},
                        "group_by":{"type":"array","items":{"type":"string"}},
                        "aggregate":{"type":"object","additionalProperties":{"type":"string"},"description":"name: count | sum(x) | min(x) | max(x) | avg(x) | distinct(x)"},
                        "order_by":{"type":"array","items":{"type":"string"},"description":"fields, '-' prefix for descending; references sort naturally"},
                        "limit":{"type":"integer","default":1000},
                        "cursor":{"type":"string"},
                        "explain":{"oneOf":[{"type":"boolean"},{"type":"string","enum":["misses"]}],"default":false},
                        "deadline_ms":{"type":"number","description":"stop the scan after this time (answer 504)"}}})json"_json,
                   false, h_pcb_query );

KOPENAPI_MARK_CONCURRENT_READS( "pcb_query" );
