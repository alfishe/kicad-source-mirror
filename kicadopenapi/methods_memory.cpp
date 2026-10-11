/// @file methods_memory.cpp
/// @brief memory_summary (answered on the HTTP thread by the service) and memory_trim.
#include "kicadopenapi_memory.h"
#include "kicadopenapi_registry.h"


static KOPENAPI_RESULT h_memory_summary( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    // the service answers memory_summary itself, on the HTTP thread (see invokeParsed)
    return KOPENAPI_RESULT::Ok( KOPENAPI_MEMORY::Summary() );
}


static KOPENAPI_RESULT h_memory_trim( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    return KOPENAPI_RESULT::Ok( KOPENAPI_MEMORY::TrimNow( "api" ) );
}


KOPENAPI_REGISTER( "memory_summary",
                   "Memory of the KiCad process now: resident, footprint (what memory pressure counts; "
                   "footprint_kind says which OS figure), peaks, virtual, compressed / swapped, threads; "
                   "the allocator's live bytes and what it holds from the OS (mapped, retained); unused = "
                   "footprint - live (what a trim can give back); caches (embedded files decoded in memory "
                   "now, decodes, releases); the automatic "
                   "trim (enabled, count, last result). Answers even while KiCad is busy",
                   R"json({"type":"object","properties":{}})json"_json,
                   false, h_memory_summary );

KOPENAPI_REGISTER( "memory_trim",
                   "Give memory the allocator retains back to the OS now (malloc_zone_pressure_relief / "
                   "malloc_trim / HeapCompact); answers footprint, live, unused and retained before and after and "
                   "the pause (ms). KiCad also does this by itself when idle after documents close "
                   "(memory_summary auto_trim; env KICAD_OPENAPI_AUTO_TRIM=0 turns that off)",
                   R"json({"type":"object","properties":{}})json"_json,
                   false, h_memory_trim, 120 );
