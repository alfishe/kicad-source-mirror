/// @file methods_view_bench.cpp
/// @brief kicadopenapi view_benchmark: a fixed, repeatable render benchmark on the schematic or board
/// editor's canvas. The kifaces run it (KOPENAPI_REGISTER_VIEW_BENCHMARK, the driver is in
/// kicadopenapi_view_bench.h); this file picks the editor and dispatches.
#include <kicadopenapi_registry.h>

#include <frame_type.h>
#include <kiway.h>
#include <kiway_player.h>


namespace
{

/// @brief Whether the editor of the domain has a window on screen
bool editorShown( KOPENAPI_CONTEXT& aCtx, const std::string& aDomain )
{
    if( !aCtx.kiway )
        return false;

    KIWAY_PLAYER* frame = aCtx.kiway->Player( aDomain == "pcb" ? FRAME_PCB_EDITOR : FRAME_SCH, false );
    return frame && frame->IsShown();
}

} // namespace


static KOPENAPI_RESULT h_view_benchmark( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::string editor = aArgs.value( "editor", std::string() );

    if( editor.empty() )
    {
        const bool pcb = editorShown( aCtx, "pcb" );
        const bool sch = editorShown( aCtx, "sch" );

        if( pcb == sch )
            return KOPENAPI_RESULT::Error( 400, pcb ? "both editors are open: editor: pcb or sch"
                                                    : "no schematic or board editor window open (sch_open / pcb_open)" );

        editor = pcb ? "pcb" : "sch";
    }

    if( editor != "pcb" && editor != "sch" )
        return KOPENAPI_RESULT::Error( 400, "editor: pcb or sch" );

    const std::map<std::string, KOPENAPI_HANDLER> benches = KOPENAPI_REGISTRY::Get().ViewBenchmarks();
    auto                                          it = benches.find( editor );

    if( it == benches.end() || !editorShown( aCtx, editor ) )
        return KOPENAPI_RESULT::Error( 409, editor == "pcb" ? "no board editor window open (pcb_open)"
                                                            : "no schematic editor window open (sch_open)" );

    return it->second( aCtx, aArgs );
}


KOPENAPI_REGISTER( "view_benchmark",
                   "Render performance benchmark of the schematic or board editor canvas (GAL / OpenGL): "
                   "fixed repeatable scenarios (static redraw, compose only, pan sweep, zoom sweep, full "
                   "recache, display option, layer visibility toggles, selection, net highlight, small edit "
                   "+ undo); per scenario frames, mean / p50 / p95 / max frame time, stage breakdown "
                   "(update, redraw, cached, non-cached, overlay, composite, swap), item / vertex / draw "
                   "call counts, slowest layers; view, selection and document restored afterwards; GUI only",
                   R"json({"type":"object","properties":{
                        "editor":{"type":"string","enum":["pcb","sch"],"description":"default: the one open"},
                        "sheet":{"type":"string","default":"current","description":"sch: current, busiest (most items), or a sheet path / file name glob; switched back afterwards"},
                        "scenarios":{"type":"array","items":{"type":"string","enum":["static","compose","pan","zoom","recache","option","layers","select","highlight","edit"]},
                            "description":"default: all that apply (layers: board only)"},
                        "frames":{"type":"integer","default":60,"minimum":4,"maximum":2000,"description":"frames of static / compose / pan / zoom; the other scenarios run fewer (cycles)"},
                        "cycles":{"type":"integer","default":6,"minimum":1,"maximum":200,"description":"on / off cycles of recache, option, layers, select, highlight, edit"},
                        "fit":{"type":"boolean","default":true,"description":"start every scenario from the whole drawing fitted"},
                        "vsync":{"type":"boolean","default":false,"description":"false: swap interval 0 during the run (frame times not capped by the display)"},
                        "gpu_sync":{"type":"boolean","default":false,"description":"glFinish after each stage: the GPU's time lands in its stage (slower overall)"},
                        "warmup":{"type":"integer","default":3,"minimum":0,"maximum":100,"description":"frames drawn before measuring"},
                        "layers_top":{"type":"integer","default":8,"minimum":0,"maximum":100,"description":"slowest view layers listed per scenario"},
                        "frames_detail":{"type":"boolean","default":false,"description":"also answer every frame's numbers"}}})json"_json,
                   true, h_view_benchmark, 600 );
