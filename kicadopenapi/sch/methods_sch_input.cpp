/// @file methods_sch_input.cpp
/// @brief sch_action, sch_input: the schematic editor's tools driven as by a user (see
/// kicadopenapi_editor_input.h).
#include "kopenapi_sch.h"

#include <kicadopenapi_editor_input.h>
#include <kicadopenapi_registry.h>

#include <base_units.h>
#include <frame_type.h>
#include <kiway.h>
#include <sch_edit_frame.h>
#include <tool/tool_manager.h>
#include <tools/sch_selection_tool.h>


static SCH_EDIT_FRAME* schematicEditor( KOPENAPI_CONTEXT& aCtx )
{
    return aCtx.kiway ? dynamic_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;
}


/// The schematic editor's side: state, item lookup on the shown sheet, selecting exactly one item
static KOPENAPI_EDITOR_HOST schematicHost( SCH_EDIT_FRAME* aFrame )
{
    KOPENAPI_EDITOR_HOST host;

    host.state = [aFrame]( nlohmann::json& aOut )
    {
        if( SCH_SELECTION_TOOL* tool = aFrame->GetToolManager()->GetTool<SCH_SELECTION_TOOL>() )
            aOut["selected"] = tool->GetSelection().Size();
    };

    host.resolve = [aFrame]( const std::string& aRef ) -> std::optional<BOX2I>
    {
        SCH_ITEM* item = KopenapiFindSchematicItem( aFrame, aRef );
        return item ? std::optional<BOX2I>( item->GetBoundingBox() ) : std::nullopt;
    };

    host.select = [aFrame]( const std::string& aRef ) -> bool
    {
        SCH_ITEM*           item = KopenapiFindSchematicItem( aFrame, aRef );
        SCH_SELECTION_TOOL* tool = aFrame->GetToolManager()->GetTool<SCH_SELECTION_TOOL>();

        if( !item || !tool )
            return false;

        tool->ClearSelection();
        tool->AddItemToSel( item );
        return true;
    };

    return host;
}


static KOPENAPI_RESULT h_sch_action( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    SCH_EDIT_FRAME* frame = schematicEditor( aCtx );
    return KopenapiEditorAction( frame, schIUScale, aArgs, schematicHost( frame ) );
}


static KOPENAPI_RESULT h_sch_input( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    SCH_EDIT_FRAME* frame = schematicEditor( aCtx );
    return KopenapiEditorInput( frame, schIUScale, aArgs, schematicHost( frame ) );
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
                   "click, dclick, down, up at [x, y] mm or an item (R5, U1.3, uuid) on the shown sheet "
                   "(visible on the canvas), ms to glide there; intents select (exactly that item) and drag "
                   "(select it and move it to / by); key, mods, wait_ms. No OS events or permissions involved. "
                   "Answers the active tool, pointer, held buttons, selection size; GUI only",
                   KopenapiEditorInputSchema(), true, h_sch_input );

// Tool actions and input can change the document (drag, delete, a placed wire): each call
// that does is a step in edit_log; calls that change nothing leave no entry (see AfterEdit)
KOPENAPI_MARK_EDITING( "sch_action", "sch_input" );
