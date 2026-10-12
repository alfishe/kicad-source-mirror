/// @file methods_pcb_input.cpp
/// @brief pcb_action, pcb_input: the board editor's tools driven as by a user (see
/// kicadopenapi_editor_input.h).
#include "kopenapi_pcb.h"

#include <kicadopenapi_editor_input.h>
#include <kicadopenapi_registry.h>

#include <base_units.h>
#include <board.h>
#include <frame_type.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <router/pns_router.h>
#include <tool/tool_manager.h>
#include <tools/pcb_selection_tool.h>


static PCB_EDIT_FRAME* boardEditor( KOPENAPI_CONTEXT& aCtx )
{
    return aCtx.kiway ? dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;
}


/// The routing state and board counts after the input
static void boardState( PCB_EDIT_FRAME* aFrame, nlohmann::json& aOut )
{
    const PNS::ROUTER* router = PNS::ROUTER::GetInstance();
    aOut["routing"] = router && router->RoutingInProgress();

    if( BOARD* board = aFrame->GetBoard() )
        aOut["tracks"] = board->Tracks().size();

    if( PCB_SELECTION_TOOL* selection = aFrame->GetToolManager()->GetTool<PCB_SELECTION_TOOL>() )
        aOut["selected"] = selection->GetSelection().Size();
}


/// The board editor's side: state, item lookup, selecting exactly one item
static KOPENAPI_EDITOR_HOST boardHost( PCB_EDIT_FRAME* aFrame )
{
    KOPENAPI_EDITOR_HOST host;

    host.state = [aFrame]( nlohmann::json& aOut ) { boardState( aFrame, aOut ); };

    host.resolve = [aFrame]( const std::string& aRef ) -> std::optional<BOX2I>
    {
        BOARD_ITEM* item = KopenapiFindBoardItem( aFrame->GetBoard(), aRef );
        return item ? std::optional<BOX2I>( KopenapiBoardItemBox( item ) ) : std::nullopt;
    };

    host.select = [aFrame]( const std::string& aRef ) -> bool
    {
        BOARD_ITEM*         item = KopenapiFindBoardItem( aFrame->GetBoard(), aRef );
        PCB_SELECTION_TOOL* tool = aFrame->GetToolManager()->GetTool<PCB_SELECTION_TOOL>();

        if( !item || !tool )
            return false;

        tool->ClearSelection();
        tool->AddItemToSel( item );
        return true;
    };

    return host;
}


static KOPENAPI_RESULT h_pcb_action( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    PCB_EDIT_FRAME* frame = boardEditor( aCtx );
    return KopenapiEditorAction( frame, pcbIUScale, aArgs, boardHost( frame ) );
}


static KOPENAPI_RESULT h_pcb_input( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    PCB_EDIT_FRAME* frame = boardEditor( aCtx );
    return KopenapiEditorInput( frame, pcbIUScale, aArgs, boardHost( frame ) );
}


KOPENAPI_REGISTER( "pcb_action",
                   "Run a board editor tool action by name as the user would (menu / hotkey): e.g. "
                   "pcbnew.InteractiveRouter.SingleTrack, common.Interactive.zoomFitScreen; at: [x, y] mm puts "
                   "the pointer there first; list: glob finds action names, labels and hotkeys. Answers the "
                   "active tool, pointer, held buttons, routing in progress, track count, selection size. Drive the tool "
                   "then with pcb_input; GUI only",
                   KopenapiEditorActionSchema(), true, h_pcb_action );

KOPENAPI_REGISTER( "pcb_input",
                   "Pointer and keyboard input to the board editor's canvas, taking the same path as the "
                   "user's (click / drag thresholds, double clicks, hotkeys, Escape cancels): steps of move, "
                   "click, dclick, down, up at [x, y] mm or an item (U1, U1.3, uuid; visible on the canvas), "
                   "ms to glide there; intents select (exactly that item, even under a track) and drag "
                   "(select it and move it to / by); key (hotkey letters, Escape, Enter, ...), mods, wait_ms. "
                   "No OS events or permissions involved. Answers the "
                   "active tool, pointer, held buttons, routing in progress, track count, selection size; GUI only",
                   KopenapiEditorInputSchema(), true, h_pcb_input );

// Tool actions and input can change the document (drag, delete, a finished route): each call
// that does is a step in edit_log; calls that change nothing leave no entry (see AfterEdit)
KOPENAPI_MARK_EDITING( "pcb_action", "pcb_input" );
