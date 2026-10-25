/// @file methods_pcb_select.cpp
/// @brief kicadopenapi board editor selection: pcb_select (GUI)
#include "kopenapi_pcb.h"

#include <board.h>
#include <footprint.h>
#include <frame_type.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <pcb_base_frame.h>
#include <tool/tool_manager.h>
#include <tools/pcb_selection_tool.h>


static KOPENAPI_RESULT h_pcb_select( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;

    if( !frame || !frame->GetCanvas() || !frame->IsShown() || !frame->GetBoard() )
        return KOPENAPI_RESULT::Error( 409, "no board editor window open" );

    PCB_SELECTION_TOOL* tool = frame->GetToolManager()->GetTool<PCB_SELECTION_TOOL>();

    if( !tool )
        return KOPENAPI_RESULT::Error( 409, "the board editor has no selection tool" );

    BOARD* board = frame->GetBoard();

    if( aArgs.value( "clear", true ) )
        tool->ClearSelection();

    int selected = 0;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
    {
        if( FOOTPRINT* fp = board->FindFootprintByReference( wxString::FromUTF8( r.get<std::string>() ) ) )
        {
            tool->AddItemToSel( fp );
            ++selected;
        }
    }

    for( const nlohmann::json& u : aArgs.value( "uuids", nlohmann::json::array() ) )
    {
        std::string error;

        if( BOARD_ITEM* item = u.is_string() ? KopenapiResolveUuid( board, u.get<std::string>(), error ) : nullptr )
        {
            tool->AddItemToSel( item );
            ++selected;
        }
    }

    frame->GetCanvas()->Refresh();
    return KOPENAPI_RESULT::Ok( { { "selected", selected } } );
}


KOPENAPI_REGISTER( "pcb_select",
                   "Select items in the board editor as a user would (footprints by reference, any item by "
                   "uuid) - for screenshots, recordings and checks; clear (default) replaces the selection, "
                   "no arguments clears it; GUI only",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "clear":{"type":"boolean","default":true}}})json"_json,
                   true, h_pcb_select );
