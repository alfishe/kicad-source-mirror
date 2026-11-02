/// @file methods_pcb_design_block.cpp
/// @brief kicadopenapi design blocks on the board, as the editor's Design Blocks pane: list the
/// libraries and their blocks, Save Board / Selection as Design Block, Place Design Block (its
/// layout appended, placed, grouped and linked to the block).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <design_block.h>
#include <footprint.h>
#include <io/io_mgr.h>
#include <kicadopenapi_design_blocks.h>
#include <kicadopenapi_util.h>
#include <locale_io.h>
#include <netinfo.h>
#include <pcb_design_block_utils.h>
#include <pcb_group.h>
#include <pcb_io/pcb_io.h>
#include <pcb_io/pcb_io_mgr.h>
#include <project.h>
#include <tool/tool_manager.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>

#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


wxString wx( const std::string& aText )
{
    return wxString::FromUTF8( aText );
}


const std::string LIST = "pcb_design_block_list";


KOPENAPI_RESULT h_pcb_design_block_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    return KopenapiDesignBlockList( context->GetBoard()->GetProject(), aArgs, LIST );
}


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


/// @brief A group by uuid (or prefix) or name
PCB_GROUP* findGroup( BOARD* aBoard, const std::string& aSpec )
{
    for( PCB_GROUP* group : aBoard->Groups() )
    {
        if( str( group->GetName() ) == aSpec )
            return group;
    }

    BOARD_ITEM* item = KopenapiFindBoardItem( aBoard, aSpec );
    return item && item->Type() == PCB_GROUP_T ? static_cast<PCB_GROUP*>( item ) : nullptr;
}


KOPENAPI_RESULT h_pcb_design_block_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*   board = context->GetBoard();
    PROJECT* project = board->GetProject();

    std::string                        error;
    int                                status = 400;
    std::optional<KOPENAPI_DB_LIBRARY> lib = KopenapiDesignBlockLibrary( project, aArgs, false, LIST, error, status );

    if( !lib )
        return KOPENAPI_RESULT::Error( status, error );

    if( !KopenapiDesignBlockLibraryWritable( project, *lib ) )
        return KOPENAPI_RESULT::Error( 409, "library: '" + str( lib->nickname ) + "' is read-only" );

    // what to save: the whole board, one group (its members; the group gets the link) or items
    // (they get a new linked group)
    PCB_GROUP*             group = nullptr;
    std::vector<EDA_ITEM*> items;
    const bool             wholeBoard = !aArgs.contains( "items" ) && !aArgs.contains( "group" );

    if( aArgs.contains( "group" ) )
    {
        group = findGroup( board, aArgs.value( "group", std::string() ) );

        if( !group )
            return KOPENAPI_RESULT::Error( 404, "group: no group " + aArgs["group"].dump() + " (name or uuid)" );

        group->RunOnChildren( [&]( BOARD_ITEM* aItem ) { items.push_back( aItem ); }, RECURSE_MODE::NO_RECURSE );
    }

    if( aArgs.contains( "items" ) )
    {
        if( group )
            return KOPENAPI_RESULT::Error( 400, "items or group, not both" );

        std::set<EDA_ITEM*> seen;

        for( const nlohmann::json& spec : aArgs["items"] )
        {
            BOARD_ITEM* item = spec.is_string() ? KopenapiFindBoardItem( board, spec.get<std::string>() ) : nullptr;

            if( !item )
                return KOPENAPI_RESULT::Error( 404, "items: no item " + spec.dump() + " on the board" );

            if( item->GetParentFootprint() )
                return KOPENAPI_RESULT::Error( 400, "items: " + spec.dump() + " is part of a footprint: give the footprint" );

            if( seen.insert( item ).second )
                items.push_back( item );
        }
    }

    if( !wholeBoard && items.empty() )
        return KOPENAPI_RESULT::Error( 400, "nothing to save: the group is empty / no items" );

    wxString name = wx( aArgs.value( "name", std::string() ) );

    if( name.IsEmpty() )
        name = group && !group->GetName().IsEmpty() ? group->GetName() : wxFileName( board->GetFileName() ).GetName();

    if( name.IsEmpty() || name.find_first_of( wxS( "/\\:" ) ) != wxString::npos )
        return KOPENAPI_RESULT::Error( 400, "name: the design block's name (no / \\ :)" );

    const bool exists = KopenapiDesignBlockExists( project, *lib, name );

    if( exists && !aArgs.value( "overwrite", false ) )
        return KOPENAPI_RESULT::Error( 409, "design block '" + str( name ) + "' already exists in "
                                                    + lib->Name()
                                                    + ": overwrite: true replaces its layout" );

    // overwriting keeps the block's description, keywords, fields and schematic, as Update Design
    // Block from Board / Selection does
    DESIGN_BLOCK block;

    if( exists )
    {
        std::string                   loadError;
        std::unique_ptr<DESIGN_BLOCK> old = KopenapiLoadDesignBlock( project, *lib, name, loadError );

        if( old )
        {
            block.SetLibDescription( old->GetLibDescription() );
            block.SetKeywords( old->GetKeywords() );
            for( const auto& [key, value] : old->GetFields() )
                block.GetFields()[key] = value;
        }
    }

    block.SetLibId( LIB_ID( lib->nickname, name ) );

    if( aArgs.contains( "description" ) )
        block.SetLibDescription( wx( aArgs.value( "description", std::string() ) ) );

    if( aArgs.contains( "keywords" ) )
        block.SetKeywords( wx( aArgs.value( "keywords", std::string() ) ) );

    const wxString tempFile = wxFileName::CreateTempFileName( "design_block" );

    try
    {
        LOCALE_IO                    toggle;
        IO_RELEASER<PCB_IO>          pi( PCB_IO_MGR::FindPlugin( PCB_IO_MGR::KICAD_SEXP ) );
        std::unique_ptr<BOARD>       blockBoard;

        if( !wholeBoard )
            blockBoard = MakeDesignBlockBoard( *board, project, items );

        pi->SaveBoard( tempFile, wholeBoard ? *board : *blockBoard, nullptr );
        block.SetBoardFile( tempFile );
        KopenapiSaveDesignBlock( project, *lib, block );
    }
    catch( const IO_ERROR& ioe )
    {
        wxRemoveFile( tempFile );
        return KOPENAPI_RESULT::Error( 422, "the design block could not be saved: " + str( ioe.What() ) );
    }

    wxRemoveFile( tempFile );

    nlohmann::json out = { { "lib_id", str( block.GetLibId().Format() ) },
                           { "name", str( name ) },
                           { "overwritten", exists },
                           { "items", wholeBoard ? nlohmann::json( "board" ) : nlohmann::json( items.size() ) } };

    if( lib->IsPath() )
        out["path"] = str( lib->path + wxFileName::GetPathSeparator() + name + wxS( "." )
                           + wxString( FILEEXT::KiCadDesignBlockPathExtension ) );

    // the saved items join the block: a new group linked to it, or the link on the group given
    if( !wholeBoard )
    {
        BOARD_COMMIT commit( context->GetToolManager() );

        if( group )
        {
            if( !group->HasDesignBlockLink() )
            {
                commit.Modify( group, nullptr, RECURSE_MODE::NO_RECURSE );
                group->SetDesignBlockLibId( block.GetLibId() );
                commit.Push( _( "Set Group Design Block Link" ) );
            }

            out["group"] = { { "uuid", str( group->m_Uuid.AsString() ) },
                             { "name", str( group->GetName() ) },
                             { "design_block", str( group->GetDesignBlockLibId().Format() ) } };
        }
        else
        {
            PCB_GROUP* newGroup = new PCB_GROUP( board );
            newGroup->SetName( block.GetLibId().GetUniStringLibItemName() );
            newGroup->SetDesignBlockLibId( block.GetLibId() );
            int added = 0;

            for( EDA_ITEM* edaItem : items )
            {
                BOARD_ITEM* item = static_cast<BOARD_ITEM*>( edaItem );

                if( item->GetParentFootprint() || !item->IsGroupableType() )
                    continue;

                if( EDA_GROUP* existing = item->GetParentGroup() )
                    commit.Modify( existing->AsEdaItem(), nullptr, RECURSE_MODE::NO_RECURSE );

                commit.Modify( item, nullptr, RECURSE_MODE::NO_RECURSE );
                newGroup->AddItem( item );
                added++;
            }

            if( added > 0 )
            {
                commit.Add( newGroup );
                commit.Push( _( "Group Items" ) );
                out["group"] = { { "uuid", str( newGroup->m_Uuid.AsString() ) },
                                 { "name", str( newGroup->GetName() ) },
                                 { "design_block", str( newGroup->GetDesignBlockLibId().Format() ) } };
            }
            else
            {
                newGroup->RemoveAll();
                delete newGroup;
            }
        }

        context->SetContentModified();
    }

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT h_pcb_design_block_place( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*   brd = context->GetBoard();
    PROJECT* project = brd->GetProject();

    std::string                        error;
    int                                status = 400;
    std::optional<KOPENAPI_DB_LIBRARY> lib = KopenapiDesignBlockLibrary( project, aArgs, true, LIST, error, status );

    if( !lib )
        return KOPENAPI_RESULT::Error( status, error );

    const wxString name = wx( aArgs.value( "name", std::string() ) );

    if( name.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "name: the design block (pcb_design_block_list)" );

    std::optional<VECTOR2I> at;

    if( aArgs.contains( "at_mm" ) )
    {
        const nlohmann::json& a = aArgs["at_mm"];

        if( !a.is_array() || a.size() != 2 || !a[0].is_number() || !a[1].is_number() )
            return KOPENAPI_RESULT::Error( 400, "at_mm: [x, y]" );

        at = VECTOR2I( pcbIUScale.mmToIU( a[0].get<double>() ), pcbIUScale.mmToIU( a[1].get<double>() ) );
    }

    if( !KopenapiDesignBlockExists( project, *lib, name ) )
        return KOPENAPI_RESULT::Error( 404, "name: no design block '" + str( name ) + "' in "
                                                    + lib->Name() );

    std::unique_ptr<DESIGN_BLOCK> block = KopenapiLoadDesignBlock( project, *lib, name, error );

    if( !block )
        return KOPENAPI_RESULT::Error( 422, "name: " + error );

    if( block->GetBoardFile().IsEmpty() || !wxFileName::FileExists( block->GetBoardFile() ) )
        return KOPENAPI_RESULT::Error( 422, "design block '" + str( name ) + "' has no layout to place" );

    // As the editor's Append Board: the block's board read into this one, the items it brings
    // marked by what was there before
    BOARD_COMMIT   commit( context->GetToolManager() );
    BOARD_ITEM_SET existingItems = brd->GetItemSet();
    std::set<NETINFO_ITEM*> existingNets;

    for( NETINFO_ITEM* net : brd->GetNetInfo() )
        existingNets.insert( net );

    for( BOARD_ITEM* item : existingItems )
        item->SetFlags( SKIP_STRUCT );

    auto clearSkipStruct = [&]()
    {
        for( BOARD_ITEM* item : existingItems )
            item->ClearFlags( SKIP_STRUCT );
    };

    std::map<wxString, wxString> oldProperties = brd->GetProperties();
    PAGE_INFO                    oldPageInfo = brd->GetPageSettings();
    TITLE_BLOCK                  oldTitleBlock = brd->GetTitleBlock();
    const int                    initialCopperLayerCount = brd->GetCopperLayerCount();
    const LSET                   initialEnabledLayers = brd->GetEnabledLayers();
    nlohmann::json               warnings = nlohmann::json::array();

    try
    {
        std::map<std::string, UTF8> props;
        const VECTOR2I              page = brd->GetPageSettings().GetSizeIU( pcbIUScale.IU_PER_MILS );

        props["page_width"] = std::to_string( page.x );
        props["page_height"] = std::to_string( page.y );
        props[PCB_IO_LOAD_PROPERTIES::APPEND_PRESERVE_DESTINATION_STACKUP] = "";

        IO_RELEASER<PCB_IO> pi( PCB_IO_MGR::FindPlugin( PCB_IO_MGR::KICAD_SEXP ) );

        // the editor asks; the API goes on and says what it was asked
        pi->SetQueryUserCallback(
                [&]( wxString aTitle, int, wxString aMessage, wxString ) -> bool
                {
                    warnings.push_back( str( aTitle + wxS( ": " ) + aMessage ) );
                    return true;
                } );

        pi->LoadAndAppendBoard( block->GetBoardFile(), *brd, &props, nullptr );
    }
    catch( const IO_ERROR& ioe )
    {
        clearSkipStruct();
        return KOPENAPI_RESULT::Error( 422, "the design block's layout could not be read: " + str( ioe.What() ) );
    }

    std::map<wxString, wxString> newProperties = brd->GetProperties();

    for( const auto& [key, value] : oldProperties )
        newProperties[key] = value;

    brd->SetProperties( newProperties );
    brd->SetPageSettings( oldPageInfo );
    brd->SetTitleBlock( oldTitleBlock );

    brd->BuildListOfNets();
    brd->SynchronizeNetsAndNetClasses( true );
    brd->BuildConnectivity();

    int netsAdded = 0;

    for( NETINFO_ITEM* net : brd->GetNetInfo() )
    {
        if( !existingNets.contains( net ) )
        {
            commit.Added( net );
            netsAdded++;
        }
    }

    if( brd->GetCopperLayerCount() > initialCopperLayerCount )
        brd->SetCopperLayerCount( brd->GetCopperLayerCount() );

    LSET enabledLayers = brd->GetEnabledLayers() | initialEnabledLayers;
    brd->SetEnabledLayers( enabledLayers );
    brd->SetVisibleLayers( enabledLayers );
    brd->GetDesignSettings().GetStackupDescriptor().SynchronizeWithBoard( &brd->GetDesignSettings() );

    if( brd->GetCopperLayerCount() != initialCopperLayerCount )
        warnings.push_back( "the board changed from " + std::to_string( initialCopperLayerCount ) + " to "
                            + std::to_string( brd->GetCopperLayerCount() ) + " copper layers, stackup updated" );

    std::vector<BOARD_ITEM*> added;

    for( BOARD_ITEM* item : brd->GetItemSet() )
    {
        if( item->Type() == PCB_MARKER_T || ( item->GetFlags() & SKIP_STRUCT ) )
            continue;

        added.push_back( item );
    }

    clearSkipStruct();

    if( added.empty() )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Error( 422, "design block '" + str( name ) + "' has an empty layout" );
    }

    // the items a selection would hold: not inside a group that came with them
    std::set<EDA_ITEM*>      addedSet( added.begin(), added.end() );
    std::vector<BOARD_ITEM*> top;

    for( BOARD_ITEM* item : added )
    {
        if( !item->GetParentGroup() || !addedSet.count( item->GetParentGroup()->AsEdaItem() ) )
            top.push_back( item );

        commit.Added( item );
    }

    // where the editor's move starts: the top-left item's position (smallest x, then y)
    BOARD_ITEM* topLeft = nullptr;

    for( BOARD_ITEM* item : top )
    {
        const VECTOR2I p = item->GetPosition();

        if( !topLeft || p.x < topLeft->GetPosition().x
            || ( p.x == topLeft->GetPosition().x && p.y < topLeft->GetPosition().y ) )
        {
            topLeft = item;
        }
    }

    const VECTOR2I anchor = topLeft->GetPosition();
    const VECTOR2I delta = at ? *at - anchor : VECTOR2I( 0, 0 );

    if( delta != VECTOR2I( 0, 0 ) )
    {
        for( BOARD_ITEM* item : top )
            item->Move( delta );
    }

    nlohmann::json out = { { "lib_id", str( block->GetLibId().Format() ) },
                           { "items", added.size() },
                           { "nets_added", netsAdded },
                           { "anchor_mm", { toMm( anchor.x + delta.x ), toMm( anchor.y + delta.y ) } },
                           { "moved_by_mm", { toMm( delta.x ), toMm( delta.y ) } } };

    // a group linked to the block, as the editor's "place as group" (its default)
    if( aArgs.value( "group", true ) )
    {
        std::vector<BOARD_ITEM*> groupable;

        for( BOARD_ITEM* item : top )
        {
            if( !item->GetParentFootprint() )
                groupable.push_back( item );
        }

        if( groupable.size() >= 2 )
        {
            PCB_GROUP* group = new PCB_GROUP( brd );
            group->SetName( block->GetLibId().GetLibItemName() );
            group->SetDesignBlockLibId( block->GetLibId() );

            for( BOARD_ITEM* item : top )
            {
                if( item->IsLocked() )
                    group->SetLocked( true );
            }

            commit.Add( group );

            for( BOARD_ITEM* item : groupable )
            {
                commit.Modify( item );
                group->AddItem( item );
            }

            out["group"] = { { "uuid", str( group->m_Uuid.AsString() ) },
                             { "name", str( group->GetName() ) },
                             { "design_block", str( group->GetDesignBlockLibId().Format() ) } };
        }
    }

    commit.Push( _( "Place Design Block" ) );
    brd->BuildConnectivity();
    context->SetContentModified();

    // the editor does not reannotate an appended block: say which references now repeat
    std::map<wxString, int> refCount;
    nlohmann::json          refs = nlohmann::json::array();

    for( FOOTPRINT* fp : brd->Footprints() )
        refCount[fp->GetReference()]++;

    std::set<wxString> duplicates;

    for( BOARD_ITEM* item : added )
    {
        if( item->Type() != PCB_FOOTPRINT_T )
            continue;

        const wxString ref = static_cast<FOOTPRINT*>( item )->GetReference();
        refs.push_back( str( ref ) );

        if( refCount[ref] > 1 )
            duplicates.insert( ref );
    }

    out["footprints"] = refs;

    if( !duplicates.empty() )
    {
        nlohmann::json dup = nlohmann::json::array();

        for( const wxString& ref : duplicates )
            dup.push_back( str( ref ) );

        out["duplicate_refs"] = dup;
    }

    if( !warnings.empty() )
        out["warnings"] = warnings;

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_design_block_list",
                   "Design block libraries and their blocks, as the editor's Design Blocks pane: without library "
                   "the libraries of the global and project tables (nickname, description, blocks); with library "
                   "(a nickname, or the absolute path of a .kicad_blocks directory) its blocks: name, lib_id, "
                   "description, keywords, has_layout, has_schematic",
                   KopenapiDesignBlockListSchema(), false, h_pcb_design_block_list );

KOPENAPI_REGISTER( "pcb_design_block_save",
                   "Save as Design Block, as the editor's: without items / group the whole board (Save Board as "
                   "Design Block); with group (name or uuid) its members, and the group gets the block's link; with "
                   "items (REF, uuids of tracks, vias, drawings, zones, groups) those, and they join a new group "
                   "linked to the block (Save Selection as Design Block). library: a nickname of the design block "
                   "library tables, or the absolute path of a .kicad_blocks directory (created if missing); name "
                   "default the group's / board's name; description, keywords. An existing block is refused (409) "
                   "unless overwrite: true, which replaces its layout and keeps its description, keywords, fields "
                   "and schematic unless given",
                   R"json({"type":"object","required":["library"],"properties":{
                        "library":{"type":"string"},"name":{"type":"string"},
                        "items":{"type":"array","items":{"type":"string"}},
                        "group":{"type":"string"},
                        "description":{"type":"string"},"keywords":{"type":"string"},
                        "overwrite":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_design_block_save );

KOPENAPI_REGISTER( "pcb_design_block_place",
                   "Place Design Block, as the editor's: the block's layout appended to the board (nets by name, "
                   "the board's stackup kept, layers it uses enabled), its top-left item (smallest x, then y) at "
                   "at_mm (default where the block has it), and with group (default true, the editor's \"place as "
                   "group\") one group named after the block and linked to it. References are not renumbered, "
                   "as in the editor: duplicate_refs names the ones now on the board twice. One undo step",
                   R"json({"type":"object","required":["library","name"],"properties":{
                        "library":{"type":"string"},"name":{"type":"string"},
                        "at_mm":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},
                        "group":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_design_block_place );

KOPENAPI_MARK_EDITING( "pcb_design_block_save", "pcb_design_block_place" );
