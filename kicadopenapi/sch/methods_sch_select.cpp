/// @file methods_sch_select.cpp
/// @brief kicadopenapi schematic editor selection: sch_select (GUI)
#include "kopenapi_sch.h"

#include <frame_type.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <sch_edit_frame.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <tool/tool_manager.h>
#include <tools/sch_selection_tool.h>

#include <set>


static KOPENAPI_RESULT h_sch_select( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;

    if( !frame || !frame->GetCanvas() || !frame->IsShown() || !frame->GetScreen() )
        return KOPENAPI_RESULT::Error( 409, "no schematic editor window open" );

    SCH_SELECTION_TOOL* tool = frame->GetToolManager()->GetTool<SCH_SELECTION_TOOL>();

    if( !tool )
        return KOPENAPI_RESULT::Error( 409, "the schematic editor has no selection tool" );

    std::set<std::string> refs, uuids;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
        refs.insert( r.get<std::string>() );

    for( const nlohmann::json& u : aArgs.value( "uuids", nlohmann::json::array() ) )
        uuids.insert( u.get<std::string>() );

    if( !uuids.empty() )
    {
        std::vector<SCH_ITEM*> items;

        for( SCH_ITEM* item : frame->GetScreen()->Items() )
            items.push_back( item );

        if( std::string error; !KopenapiSchExpandUuids( items, uuids, error ) )
            return KOPENAPI_RESULT::Error( 400, error );
    }

    if( aArgs.value( "clear", true ) )
        tool->ClearSelection();

    int selected = 0;

    for( SCH_ITEM* item : frame->GetScreen()->Items() )
    {
        bool want = uuids.count( item->m_Uuid.AsStdString() ) > 0;

        if( !want && item->Type() == SCH_SYMBOL_T && !refs.empty() )
        {
            auto* symbol = static_cast<SCH_SYMBOL*>( item );
            want = refs.count( symbol->GetRef( &frame->GetCurrentSheet() ).ToStdString( wxConvUTF8 ) ) > 0;
        }

        if( want )
        {
            tool->AddItemToSel( item );
            ++selected;
        }
    }

    frame->GetCanvas()->Refresh();
    return KOPENAPI_RESULT::Ok( { { "selected", selected } } );
}


KOPENAPI_REGISTER( "sch_select",
                   "Select items in the schematic editor as a user would (symbols by reference, any item by "
                   "uuid) on the current sheet - for screenshots, recordings and checks; clear (default) "
                   "replaces the selection, no arguments clears it; GUI only",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "uuids":{"type":"array","items":{"type":"string"},"description":"whole uuids or unique prefixes (4+ characters)"},
                        "clear":{"type":"boolean","default":true}}})json"_json,
                   true, h_sch_select );
