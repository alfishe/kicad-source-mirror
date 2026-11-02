/// @file methods_pcb_select.cpp
/// @brief kicadopenapi board editor selection: pcb_select (the editor's selection tool, or the
/// items' selection flags headless, which the selection-scoped tools read)
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <pcb_generator.h>
#include <pcb_group.h>
#include <pcb_track.h>
#include <zone.h>

#include <board.h>
#include <footprint.h>
#include <frame_type.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <pcb_base_frame.h>
#include <tool/tool_manager.h>
#include <tools/pcb_selection_tool.h>


namespace
{

/// The items wanted by refs / uuids; the ones not found in aUnmatched
std::vector<BOARD_ITEM*> wantedItems( BOARD* aBoard, const nlohmann::json& aArgs, nlohmann::json& aUnmatched )
{
    std::vector<BOARD_ITEM*> items;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
    {
        FOOTPRINT* fp = r.is_string() ? aBoard->FindFootprintByReference( wxString::FromUTF8( r.get<std::string>() ) )
                                      : nullptr;

        if( fp )
            items.push_back( fp );
        else
            aUnmatched.push_back( { { "ref", r }, { "error", "no footprint with this reference" } } );
    }

    for( const nlohmann::json& u : aArgs.value( "uuids", nlohmann::json::array() ) )
    {
        std::string error = "not a string";

        if( BOARD_ITEM* item = u.is_string() ? KopenapiResolveUuid( aBoard, u.get<std::string>(), error ) : nullptr )
            items.push_back( item );
        else
            aUnmatched.push_back( { { "uuid", u }, { "error", error } } );
    }

    return items;
}


/// Every item of the board with its children
void forEachItem( BOARD* aBoard, const std::function<void( BOARD_ITEM* )>& aFunction )
{
    auto withChildren = [&]( BOARD_ITEM* aItem )
    {
        aFunction( aItem );
        aItem->RunOnChildren( aFunction, RECURSE_MODE::RECURSE );
    };

    for( FOOTPRINT* item : aBoard->Footprints() )
        withChildren( item );

    for( PCB_TRACK* item : aBoard->Tracks() )
        withChildren( item );

    for( BOARD_ITEM* item : aBoard->Drawings() )
        withChildren( item );

    for( ZONE* item : aBoard->Zones() )
        withChildren( item );

    for( PCB_GROUP* item : aBoard->Groups() )
        aFunction( item );

    for( PCB_GENERATOR* item : aBoard->Generators() )
        aFunction( item );
}

} // namespace


static KOPENAPI_RESULT h_pcb_select( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;
    nlohmann::json unmatched = nlohmann::json::array();
    nlohmann::json out;

    if( frame && frame->GetCanvas() && frame->IsShown() && frame->GetBoard() )
    {
        PCB_SELECTION_TOOL* tool = frame->GetToolManager()->GetTool<PCB_SELECTION_TOOL>();

        if( !tool )
            return KOPENAPI_RESULT::Error( 409, "the board editor has no selection tool" );

        std::vector<BOARD_ITEM*> items = wantedItems( frame->GetBoard(), aArgs, unmatched );

        if( aArgs.value( "clear", true ) )
            tool->ClearSelection();

        for( BOARD_ITEM* item : items )
            tool->AddItemToSel( item );

        frame->GetCanvas()->Refresh();
        out = { { "selected", items.size() }, { "selection", tool->GetSelection().Size() } };
    }
    else
    {
        std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

        if( !context )
            return KopenapiNoBoard();

        BOARD*                   board = context->GetBoard();
        std::vector<BOARD_ITEM*> items = wantedItems( board, aArgs, unmatched );

        if( aArgs.value( "clear", true ) )
            forEachItem( board, []( BOARD_ITEM* aItem ) { aItem->ClearSelected(); } );

        for( BOARD_ITEM* item : items )
            item->SetSelected();

        int selection = 0;
        forEachItem( board, [&]( BOARD_ITEM* aItem ) { selection += aItem->IsSelected() ? 1 : 0; } );
        out = { { "selected", items.size() }, { "selection", selection } };
    }

    if( !unmatched.empty() )
        out["unmatched"] = unmatched;

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_select",
                   "Select items as a user would (footprints by reference, any item by uuid or uuid prefix, as "
                   "pcb_query's uuid) - for the selection-scoped tools (selected_only), screenshots and checks. "
                   "In the editor: its selection tool; headless: the items are marked selected. clear (default) "
                   "replaces the selection, no refs / uuids clears it; refs / uuids not found are listed in "
                   "unmatched with the reason. Answers selected (matched now) and selection (selected items, "
                   "with children headless)",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "clear":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_select );
