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
                   "A.courtyard() ('x0 y0 x1 y1' mm), A.Uuid (the item's uuid; '1f2e*' matches a prefix), A.inRect(x0, y0, x1, y1) / A.touchesRect(...) (mm, or "
                   "numbers with units), A.inPolygon('x0 y0, x1 y1, ...'), A.inArea('name'); a call with no item "
                   "(inRect(...)) applies to A. Read-only. "
                   "from picks the kinds: tracks are segments and arcs (A.Type 'Track'), vias are from 'vias' "
                   "(A.Type 'Via'), pads 'Pad', footprints 'Footprint', zones 'Zone', drawings 'Graphic', texts "
                   "'Text' / 'Text Box', groups 'Group'; to tell vias from tracks query from ['tracks', 'vias'] "
                   "with A.Type == 'Via' (or the row's kind: 'track' / 'arc' / 'via'); an A.Type no kind in from "
                   "has answers a hint. "
                   "Answer: count, items (select fields; properties as in KiCad's Properties panel, lengths in mm) "
                   "or groups (group_by: field names; aggregate: {output name: count | sum(x) | min(x) | max(x) | "
                   "avg(x) | distinct(x)}; e.g. tracks per net: {from: 'tracks', group_by: 'Net', aggregate: "
                   "{n: 'count', length: 'sum(Length)'}} gives groups [{key, count, n, length}]; Length is A.length(), "
                   "a track's or arc's length in mm); "
                   "zones are the board's and the footprints' (each zone row's owner: the footprint's reference, or "
                   "'board'; A.parentRef() filters by it), "
                   "next_cursor, timing_ms, scanned; explain: true gives each top-level && term per item, "
                   "'misses' adds items failing exactly one term. Errors 400 carry the offset and close names",
                   R"json({"type":"object","properties":{
                        "from":{"oneOf":[{"type":"string"},{"type":"array","items":{"type":"string","enum":["footprints","pads","tracks","vias","zones","drawings","texts","groups","items"]}}],"default":"footprints"},
                        "where":{"type":"string","description":"KiCad DRC condition, e.g. A.side() == 'bottom' && A.mount() == 'smd'; also X in ['a', 'b'] / X not in [...] (e.g. A.NetName in ['GND', '+3V3*'])"},
                        "select":{"type":"array","items":{"type":"string"},"description":"property names (Reference, Value, Layer, Net, Width, Position_X ...), uuid / kind / owner (also in group_by / order_by), or functions (A.side(); a bare parentRef / side / mount / courtyard / length is the same); a name that is neither for the queried kinds is a 400 with close names (as group_by / order_by / aggregate)"},
                        "group_by":{"oneOf":[{"type":"string"},{"type":"array","items":{"type":"string"}}],"description":"field name(s) the rows are grouped by, e.g. 'Net' or ['Net', 'Layer']"},
                        "aggregate":{"type":"object","additionalProperties":{"type":"string"},"description":"{output name: count | sum(x) | min(x) | max(x) | avg(x) | distinct(x)} per group, e.g. {n: 'count', length: 'sum(Length)'} with group_by 'Net'"},
                        "order_by":{"type":"array","items":{"type":"string"},"description":"fields, '-' prefix for descending; references sort naturally"},
                        "limit":{"type":"integer","default":1000},
                        "cursor":{"type":"string"},
                        "explain":{"oneOf":[{"type":"boolean"},{"type":"string","enum":["misses"]}],"default":false},
                        "deadline_ms":{"type":"number","description":"stop the scan after this time (answer 504)"}}})json"_json,
                   false, h_pcb_query );

KOPENAPI_MARK_CONCURRENT_READS( "pcb_query" );
