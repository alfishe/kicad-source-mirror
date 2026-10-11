/// @file methods_sch_input.cpp
/// @brief sch_action, sch_input: the schematic editor's tools driven as by a user (see
/// kicadopenapi_editor_input.h).
#include <kicadopenapi_editor_input.h>
#include <kicadopenapi_registry.h>

#include <base_units.h>
#include <frame_type.h>
#include <kiway.h>
#include <sch_edit_frame.h>


static SCH_EDIT_FRAME* schematicEditor( KOPENAPI_CONTEXT& aCtx )
{
    return aCtx.kiway ? dynamic_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;
}


static KOPENAPI_RESULT h_sch_action( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return KopenapiEditorAction( schematicEditor( aCtx ), schIUScale, aArgs, {} );
}


static KOPENAPI_RESULT h_sch_input( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return KopenapiEditorInput( schematicEditor( aCtx ), schIUScale, aArgs, {} );
}


KOPENAPI_REGISTER( "sch_action",
                   "Run a schematic editor tool action by name as the user would (menu / hotkey): e.g. "
                   "eeschema.InteractiveDrawing.drawWire; at: [x, y] mm puts the pointer there first; list: "
                   "glob finds action names, labels and hotkeys. Answers the active tool, pointer, held "
                   "buttons. Drive the tool then with sch_input; GUI only",
                   KopenapiEditorActionSchema(), true, h_sch_action );

KOPENAPI_REGISTER( "sch_input",
                   "Pointer and keyboard input to the schematic editor's canvas, taking the same path as the "
                   "user's (click / drag thresholds, double clicks, hotkeys, Escape cancels): steps of move, "
                   "click, dclick, down, up at [x, y] mm on the current sheet (the point must be visible), "
                   "key, mods, wait_ms. No OS events or permissions involved. Answers the active tool, "
                   "pointer, held buttons; GUI only",
                   KopenapiEditorInputSchema(), true, h_sch_input );
