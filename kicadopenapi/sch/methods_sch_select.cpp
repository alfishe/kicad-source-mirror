/// @file methods_sch_select.cpp
/// @brief kicadopenapi schematic editor selection: sch_select (the editor's selection tool, or the
/// items' selection flags headless, which the selection-scoped tools read)
#include "kopenapi_sch.h"

#include <api/sch_context.h>
#include <sch_sheet_path.h>
#include <schematic.h>

#include <frame_type.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <sch_edit_frame.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <tool/tool_manager.h>
#include <tools/sch_selection_tool.h>

#include <set>


namespace
{

/// The items of aScreen wanted by refs / uuids (uuids expanded from prefixes); "" or an error
std::string wantedItems( SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aSheet, const nlohmann::json& aArgs,
                         std::vector<SCH_ITEM*>& aOut )
{
    std::set<std::string> refs, uuids;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
    {
        if( !r.is_string() )
            return "refs: strings";

        refs.insert( r.get<std::string>() );
    }

    for( const nlohmann::json& u : aArgs.value( "uuids", nlohmann::json::array() ) )
    {
        if( !u.is_string() )
            return "uuids: strings";

        uuids.insert( u.get<std::string>() );
    }

    if( !uuids.empty() )
    {
        std::vector<SCH_ITEM*> items;

        for( SCH_ITEM* item : aScreen->Items() )
            items.push_back( item );

        if( std::string error; !KopenapiSchExpandUuids( items, uuids, error ) )
            return error;
    }

    for( SCH_ITEM* item : aScreen->Items() )
    {
        bool want = uuids.count( item->m_Uuid.AsStdString() ) > 0;

        if( !want && item->Type() == SCH_SYMBOL_T && !refs.empty() )
        {
            auto* symbol = static_cast<SCH_SYMBOL*>( item );
            want = refs.count( symbol->GetRef( &aSheet ).ToStdString( wxConvUTF8 ) ) > 0;
        }

        if( want )
            aOut.push_back( item );
    }

    return std::string();
}


void clearFlags( SCH_ITEM* aItem )
{
    aItem->ClearSelected();
    aItem->RunOnChildren( []( SCH_ITEM* aChild ) { aChild->ClearSelected(); }, RECURSE_MODE::RECURSE );
}

} // namespace


static KOPENAPI_RESULT h_sch_select( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;

    if( frame && frame->GetCanvas() && frame->IsShown() && frame->GetScreen() )
    {
        SCH_SELECTION_TOOL* tool = frame->GetToolManager()->GetTool<SCH_SELECTION_TOOL>();

        if( !tool )
            return KOPENAPI_RESULT::Error( 409, "the schematic editor has no selection tool" );

        if( aArgs.contains( "sheet" ) )
            return KOPENAPI_RESULT::Error( 400, "sheet: the editor selects on its current sheet" );

        std::vector<SCH_ITEM*> items;

        if( std::string error = wantedItems( frame->GetScreen(), frame->GetCurrentSheet(), aArgs, items );
            !error.empty() )
        {
            return KOPENAPI_RESULT::Error( 400, error );
        }

        if( aArgs.value( "clear", true ) )
            tool->ClearSelection();

        for( SCH_ITEM* item : items )
            tool->AddItemToSel( item );

        frame->GetCanvas()->Refresh();
        return KOPENAPI_RESULT::Ok( { { "selected", items.size() }, { "selection", tool->GetSelection().Size() } } );
    }

    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::optional<SCH_SHEET_PATH> sheet = KopenapiSchTargetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet: no such sheet (sch_sheet_list)" );

    std::vector<SCH_ITEM*> items;

    if( std::string error = wantedItems( sheet->LastScreen(), *sheet, aArgs, items ); !error.empty() )
        return KOPENAPI_RESULT::Error( 400, error );

    SCH_SCREENS screens( context->GetSchematic()->Root() );

    if( aArgs.value( "clear", true ) )
    {
        for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
        {
            for( SCH_ITEM* item : screen->Items() )
                clearFlags( item );
        }
    }

    for( SCH_ITEM* item : items )
        item->SetSelected();

    int selection = 0;

    for( SCH_ITEM* item : sheet->LastScreen()->Items() )
        selection += item->IsSelected() ? 1 : 0;

    return KOPENAPI_RESULT::Ok( { { "selected", items.size() }, { "selection", selection } } );
}


KOPENAPI_REGISTER( "sch_select",
                   "Select items as a user would (symbols by reference, any item by uuid) - for the "
                   "selection-scoped tools (selected_only), screenshots and checks. In the editor: its "
                   "selection on the current sheet; headless: the items of sheet (default the current one, "
                   "sch_sheet_list) are marked selected. clear (default) replaces the selection (every sheet), "
                   "no refs / uuids clears it. Answers selected (matched now) and selection (selected items on "
                   "the sheet)",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "uuids":{"type":"array","items":{"type":"string"},"description":"whole uuids or unique prefixes (4+ characters)"},
                        "sheet":{"type":"string","description":"headless: the sheet path (sch_sheet_list)"},
                        "clear":{"type":"boolean","default":true}}})json"_json,
                   false, h_sch_select );
