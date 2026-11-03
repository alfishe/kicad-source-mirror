/// @file methods_sch_paste.cpp
/// @brief kicadopenapi sch_paste: Edit > Paste of an explicit text instead of the system clipboard.
///
/// The editor's own paste reads it (schematic content or a text item), so its checks are the same: a
/// long text item is confirmed by confirm (the editor asks in its infobar).
#include "kopenapi_sch.h"

#include <api/sch_context.h>
#include <base_units.h>
#include <clipboard.h>
#include <kicadopenapi_service.h>
#include <kiway.h>
#include <sch_commit.h>
#include <sch_edit_frame.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_text.h>
#include <schematic.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <tools/sch_editor_control.h>


static KOPENAPI_RESULT h_sch_paste( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context || !context->GetSchematic() )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "text" ) || !aArgs["text"].is_string() || aArgs["text"].get<std::string>().empty() )
        return KOPENAPI_RESULT::Error( 400, "text: the clipboard text to paste (schematic content or plain text)" );

    const std::string text = aArgs["text"].get<std::string>();
    const bool        content = SCH_EDITOR_CONTROL::IsSchematicContent( *context->GetSchematic(), text );
    const bool        confirm = aArgs.value( "confirm", false );

    if( !content && IsLongPasteText( text ) && !confirm )
    {
        return KOPENAPI_RESULT::Error( 409, std::string( LongPasteQuestion().ToUTF8() )
                                                    + " (the editor asks; confirm: true pastes it)" );
    }

    if( !aCtx.headless && aCtx.kiway )
    {
        SCH_EDIT_FRAME* frame = static_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) );

        if( !frame )
            return KopenapiNoSchematic();

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
        return KOPENAPI_RESULT::Error( 409, "pasting schematic content needs the editor (GUI); headless: "
                                            "sch_design_block_place" );
    }

    std::optional<SCH_SHEET_PATH> sheet = KopenapiSchTargetSheet( *context, aArgs );

    if( !sheet || !sheet->LastScreen() )
        return KOPENAPI_RESULT::Error( 404, "sheet: no such sheet instance (sch_sheet_list)" );

    // the editor's text paste: a text item of the whole text
    const VECTOR2I at( schIUScale.mmToIU( aArgs.value( "x_mm", 0.0 ) ), schIUScale.mmToIU( aArgs.value( "y_mm", 0.0 ) ) );
    SCH_TEXT*      item = new SCH_TEXT( at, wxString::FromUTF8( text ) );

    SCH_COMMIT commit( context->GetToolManager() );
    commit.Add( item, sheet->LastScreen() );
    KopenapiSchPushEdit( commit, context->GetSchematic(), _( "Paste (API)" ), aCtx.kiway, { item->m_Uuid } );

    return KOPENAPI_RESULT::Ok( { { "pasted", "text" },
                                  { "uuid", item->m_Uuid.AsStdString() },
                                  { "sheet", std::string( sheet->PathHumanReadable().ToUTF8() ) } } );
}


KOPENAPI_REGISTER( "sch_paste",
                   "Edit > Paste of this text instead of the system clipboard: schematic content (what Copy puts "
                   "there) or plain text, pasted as a text item. A long text item is confirmed by confirm: true "
                   "(the editor asks in its infobar; 409 with the question otherwise). GUI: the editor's paste "
                   "runs, its items follow the pointer to be placed (sch_input click). Headless: a text item at "
                   "x_mm / y_mm on sheet (default the root; content needs the editor)",
                   R"json({"type":"object","required":["text"],"properties":{
                        "text":{"type":"string"},
                        "confirm":{"type":"boolean","default":false,"description":"answer yes to the long text question"},
                        "sheet":{"type":"string","description":"headless: the sheet instance path (sch_sheet_list)"},
                        "x_mm":{"type":"number","default":0},"y_mm":{"type":"number","default":0}}})json"_json,
                   false, h_sch_paste );

KOPENAPI_MARK_EDITING( "sch_paste" );
