/// @file methods_pcb_paste.cpp
/// @brief kicadopenapi pcb_paste: Edit > Paste of an explicit text instead of the system clipboard.
///
/// The editor's own paste reads it (board content or a text item), so its checks are the same: a long
/// text item is confirmed by confirm (the editor asks in its infobar).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <clipboard.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_service.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <tools/pcb_control.h>


static KOPENAPI_RESULT h_pcb_paste( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( !aArgs.contains( "text" ) || !aArgs["text"].is_string() || aArgs["text"].get<std::string>().empty() )
        return KOPENAPI_RESULT::Error( 400, "text: the clipboard text to paste (board content or plain text)" );

    const std::string text = aArgs["text"].get<std::string>();
    const bool        content = PCB_CONTROL::IsBoardContent( text );
    const bool        confirm = aArgs.value( "confirm", false );

    if( !content && IsLongPasteText( text ) && !confirm )
    {
        return KOPENAPI_RESULT::Error( 409, std::string( LongPasteQuestion().ToUTF8() )
                                                    + " (the editor asks; confirm: true pastes it)" );
    }

    if( !aCtx.headless && aCtx.kiway )
    {
        PCB_EDIT_FRAME* frame = static_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) );

        if( !frame )
            return KopenapiNoBoard();

        // the editor's Paste, reading this text: the pasted items follow the pointer to be placed
        SetClipboardTextOverride( text );
        KICAD_OPENAPI_SERVICE::SetConfirmPrompts( confirm );
        const bool ran = frame->GetToolManager()->RunAction( ACTIONS::paste );
        KICAD_OPENAPI_SERVICE::SetConfirmPrompts( false );
        SetClipboardTextOverride( std::nullopt );

        return KOPENAPI_RESULT::Ok( { { "pasted", content ? "content" : "text" },
                                      { "ran", ran },
                                      { "placing", true } } );
    }

    if( content )
    {
        return KOPENAPI_RESULT::Error( 409, "pasting board content needs the editor (GUI); headless: pcb_append "
                                            "a file, or pcb_design_block_place" );
    }

    // the editor's text paste: a text item of the whole text
    std::optional<KOPENAPI_METHOD> add = KOPENAPI_REGISTRY::Get().Find( "pcb_text_add" );

    if( !add )
        return KOPENAPI_RESULT::Error( 500, "pcb_text_add is not available in this process" );

    nlohmann::json args = { { "text", text },
                            { "layer", aArgs.value( "layer", std::string( "F.SilkS" ) ) },
                            { "x_mm", aArgs.value( "x_mm", 0.0 ) },
                            { "y_mm", aArgs.value( "y_mm", 0.0 ) } };

    KOPENAPI_RESULT result = add->handler( aCtx, args );

    if( result.status == 200 )
        result.body["pasted"] = "text";

    return result;
}


KOPENAPI_REGISTER( "pcb_paste",
                   "Edit > Paste of this text instead of the system clipboard: board content (what Copy puts there: "
                   "items, a footprint) or plain text, pasted as a text item. A long text item is confirmed by "
                   "confirm: true (the editor asks in its infobar; 409 with the question otherwise). GUI: the "
                   "editor's paste runs, its items follow the pointer to be placed (pcb_input click). Headless: "
                   "a text item at x_mm / y_mm on layer (content needs the editor)",
                   R"json({"type":"object","required":["text"],"properties":{
                        "text":{"type":"string"},
                        "confirm":{"type":"boolean","default":false,"description":"answer yes to the long text question"},
                        "layer":{"type":"string","default":"F.SilkS","description":"headless text item"},
                        "x_mm":{"type":"number","default":0},"y_mm":{"type":"number","default":0}}})json"_json,
                   false, h_pcb_paste );

KOPENAPI_MARK_EDITING( "pcb_paste" );
