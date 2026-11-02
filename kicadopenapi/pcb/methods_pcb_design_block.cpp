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
#include <pcb_io/common/plugin_common_layer_mapping.h>
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


/// @brief What Append Board brought into the board
struct APPENDED
{
    std::vector<BOARD_ITEM*> added;      ///< every new item, footprints' children excluded
    std::vector<BOARD_ITEM*> top;        ///< the ones a selection would hold (not in a group that came along)
    int                      netsAdded = 0;
    VECTOR2I                 anchor;     ///< the top-left item's position before any move
    VECTOR2I                 delta;
    nlohmann::json           warnings = nlohmann::json::array();
    nlohmann::json           layers = nlohmann::json::object();   ///< foreign layers mapped
    nlohmann::json           unmapped = nlohmann::json::array();  ///< foreign layers not brought in
};


/// @brief As the editor's Append Board: aFile read into the board through its plugin, nets by
/// name, the board's stackup kept, the layers it uses enabled, the new items staged in aCommit
/// and moved so the top-left one is at aAt. Foreign layers map by aLayerMap (name -> KiCad
/// layer name), else the plugin's best guess. nullopt with aError / aStatus on failure; the
/// board is then as before
std::optional<APPENDED> appendBoard( BOARD* aBoard, BOARD_COMMIT& aCommit, const wxString& aFile,
                                     PCB_IO_MGR::PCB_FILE_T aType, const std::optional<VECTOR2I>& aAt,
                                     const nlohmann::json& aLayerMap, std::string& aError, int& aStatus )
{
    APPENDED                out;
    BOARD_ITEM_SET          existingItems = aBoard->GetItemSet();
    std::set<NETINFO_ITEM*> existingNets;

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
        existingNets.insert( net );

    for( BOARD_ITEM* item : existingItems )
        item->SetFlags( SKIP_STRUCT );

    auto clearSkipStruct = [&]()
    {
        for( BOARD_ITEM* item : existingItems )
            item->ClearFlags( SKIP_STRUCT );
    };

    std::map<wxString, wxString> oldProperties = aBoard->GetProperties();
    PAGE_INFO                    oldPageInfo = aBoard->GetPageSettings();
    TITLE_BLOCK                  oldTitleBlock = aBoard->GetTitleBlock();
    const int                    initialCopperLayerCount = aBoard->GetCopperLayerCount();
    const LSET                   initialEnabledLayers = aBoard->GetEnabledLayers();
    std::string                  mapError;

    try
    {
        std::map<std::string, UTF8> props;
        const VECTOR2I              page = aBoard->GetPageSettings().GetSizeIU( pcbIUScale.IU_PER_MILS );

        props["page_width"] = std::to_string( page.x );
        props["page_height"] = std::to_string( page.y );
        props[PCB_IO_LOAD_PROPERTIES::APPEND_PRESERVE_DESTINATION_STACKUP] = "";

        IO_RELEASER<PCB_IO> pi( PCB_IO_MGR::FindPlugin( aType ) );

        if( !pi )
        {
            clearSkipStruct();
            aError = "no reader for this file";
            aStatus = 400;
            return std::nullopt;
        }

        // the editor asks; the API goes on and says what it was asked
        pi->SetQueryUserCallback(
                [&]( wxString aTitle, int, wxString aMessage, wxString ) -> bool
                {
                    out.warnings.push_back( str( aTitle + wxS( ": " ) + aMessage ) );
                    return true;
                } );

        // the editor's Map Layers dialog: the given mapping, else the plugin's guess
        if( LAYER_MAPPABLE_PLUGIN* mappable = dynamic_cast<LAYER_MAPPABLE_PLUGIN*>( pi.get() ) )
        {
            mappable->RegisterCallback(
                    [&]( const std::vector<INPUT_LAYER_DESC>& aDescs )
                    {
                        std::map<wxString, PCB_LAYER_ID> map;

                        for( const INPUT_LAYER_DESC& desc : aDescs )
                        {
                            PCB_LAYER_ID layer = desc.AutoMapLayer;
                            const std::string name = str( desc.Name );

                            if( aLayerMap.is_object() && aLayerMap.contains( name ) )
                            {
                                const std::string target = aLayerMap[name].is_string() ? aLayerMap[name].get<std::string>()
                                                                                        : std::string();
                                const int id = target.empty() ? UNDEFINED_LAYER : aBoard->GetLayerID( wx( target ) );

                                if( !target.empty() && ( id < 0 || !desc.PermittedLayers.Contains( (PCB_LAYER_ID) id ) ) )
                                    mapError = "layer_map: '" + name + "' cannot go on '" + target + "'";
                                else
                                    layer = (PCB_LAYER_ID) id;
                            }

                            map[desc.Name] = layer;
                            if( layer < 0 )
                                out.unmapped.push_back( name );
                            else
                                out.layers[name] = str( LayerName( layer ) );
                        }

                        return map;
                    } );
        }

        pi->LoadAndAppendBoard( aFile, *aBoard, &props, nullptr );
    }
    catch( const IO_ERROR& ioe )
    {
        clearSkipStruct();
        aError = "could not be read: " + str( ioe.What() );
        aStatus = 422;
        return std::nullopt;
    }

    std::map<wxString, wxString> newProperties = aBoard->GetProperties();

    for( const auto& [key, value] : oldProperties )
        newProperties[key] = value;

    aBoard->SetProperties( newProperties );
    aBoard->SetPageSettings( oldPageInfo );
    aBoard->SetTitleBlock( oldTitleBlock );

    aBoard->BuildListOfNets();
    aBoard->SynchronizeNetsAndNetClasses( true );
    aBoard->BuildConnectivity();

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
    {
        if( !existingNets.contains( net ) )
        {
            aCommit.Added( net );
            out.netsAdded++;
        }
    }

    if( aBoard->GetCopperLayerCount() > initialCopperLayerCount )
        aBoard->SetCopperLayerCount( aBoard->GetCopperLayerCount() );

    LSET enabledLayers = aBoard->GetEnabledLayers() | initialEnabledLayers;
    aBoard->SetEnabledLayers( enabledLayers );
    aBoard->SetVisibleLayers( enabledLayers );
    aBoard->GetDesignSettings().GetStackupDescriptor().SynchronizeWithBoard( &aBoard->GetDesignSettings() );

    if( aBoard->GetCopperLayerCount() != initialCopperLayerCount )
        out.warnings.push_back( "the board changed from " + std::to_string( initialCopperLayerCount ) + " to "
                                + std::to_string( aBoard->GetCopperLayerCount() ) + " copper layers, stackup updated" );

    for( BOARD_ITEM* item : aBoard->GetItemSet() )
    {
        if( item->Type() == PCB_MARKER_T || ( item->GetFlags() & SKIP_STRUCT ) )
            continue;

        out.added.push_back( item );
    }

    clearSkipStruct();

    for( BOARD_ITEM* item : out.added )
        aCommit.Added( item );

    if( !mapError.empty() || out.added.empty() )
    {
        aCommit.Revert();
        aError = mapError.empty() ? "nothing to append" : mapError;
        aStatus = mapError.empty() ? 422 : 400;
        return std::nullopt;
    }

    std::set<EDA_ITEM*> addedSet( out.added.begin(), out.added.end() );

    for( BOARD_ITEM* item : out.added )
    {
        if( !item->GetParentGroup() || !addedSet.count( item->GetParentGroup()->AsEdaItem() ) )
            out.top.push_back( item );
    }

    // where the editor's move starts: the top-left item's position (smallest x, then y)
    BOARD_ITEM* topLeft = nullptr;

    for( BOARD_ITEM* item : out.top )
    {
        const VECTOR2I p = item->GetPosition();

        if( !topLeft || p.x < topLeft->GetPosition().x
            || ( p.x == topLeft->GetPosition().x && p.y < topLeft->GetPosition().y ) )
        {
            topLeft = item;
        }
    }

    out.anchor = topLeft->GetPosition();
    out.delta = aAt ? *aAt - out.anchor : VECTOR2I( 0, 0 );

    if( out.delta != VECTOR2I( 0, 0 ) )
    {
        for( BOARD_ITEM* item : out.top )
            item->Move( out.delta );
    }

    return out;
}


/// @brief The editor's "place as group": the appended top-level items (not footprint children)
/// in one new group; nullptr when fewer than two
PCB_GROUP* groupAppended( BOARD* aBoard, BOARD_COMMIT& aCommit, const APPENDED& aAppended, const wxString& aName )
{
    std::vector<BOARD_ITEM*> groupable;

    for( BOARD_ITEM* item : aAppended.top )
    {
        if( !item->GetParentFootprint() )
            groupable.push_back( item );
    }

    if( groupable.size() < 2 )
        return nullptr;

    PCB_GROUP* group = new PCB_GROUP( aBoard );
    group->SetName( aName );

    for( BOARD_ITEM* item : aAppended.top )
    {
        if( item->IsLocked() )
            group->SetLocked( true );
    }

    aCommit.Add( group );

    for( BOARD_ITEM* item : groupable )
    {
        aCommit.Modify( item );
        group->AddItem( item );
    }

    return group;
}


/// @brief The appended footprints' references, and (duplicate_refs) the ones now on the board twice
void reportRefs( BOARD* aBoard, const APPENDED& aAppended, nlohmann::json& aOut )
{
    std::map<wxString, int> refCount;
    nlohmann::json          refs = nlohmann::json::array();
    std::set<wxString>      duplicates;

    for( FOOTPRINT* fp : aBoard->Footprints() )
        refCount[fp->GetReference()]++;

    for( BOARD_ITEM* item : aAppended.added )
    {
        if( item->Type() != PCB_FOOTPRINT_T )
            continue;

        const wxString ref = static_cast<FOOTPRINT*>( item )->GetReference();
        refs.push_back( str( ref ) );

        if( refCount[ref] > 1 )
            duplicates.insert( ref );
    }

    aOut["footprints"] = refs;

    if( !duplicates.empty() )
    {
        nlohmann::json dup = nlohmann::json::array();

        for( const wxString& ref : duplicates )
            dup.push_back( str( ref ) );

        aOut["duplicate_refs"] = dup;
    }

    if( !aAppended.warnings.empty() )
        aOut["warnings"] = aAppended.warnings;

    if( !aAppended.layers.empty() )
        aOut["layer_map"] = aAppended.layers;

    if( !aAppended.unmapped.empty() )
        aOut["unmapped_layers"] = aAppended.unmapped;
}


std::optional<VECTOR2I> atMm( const nlohmann::json& aArgs, bool& aBad )
{
    aBad = false;

    if( !aArgs.contains( "at_mm" ) )
        return std::nullopt;

    const nlohmann::json& a = aArgs["at_mm"];

    if( !a.is_array() || a.size() != 2 || !a[0].is_number() || !a[1].is_number() )
    {
        aBad = true;
        return std::nullopt;
    }

    return VECTOR2I( pcbIUScale.mmToIU( a[0].get<double>() ), pcbIUScale.mmToIU( a[1].get<double>() ) );
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

    bool                    badAt = false;
    std::optional<VECTOR2I> at = atMm( aArgs, badAt );

    if( badAt )
        return KOPENAPI_RESULT::Error( 400, "at_mm: [x, y]" );

    if( !KopenapiDesignBlockExists( project, *lib, name ) )
        return KOPENAPI_RESULT::Error( 404, "name: no design block '" + str( name ) + "' in "
                                                    + lib->Name() );

    std::unique_ptr<DESIGN_BLOCK> block = KopenapiLoadDesignBlock( project, *lib, name, error );

    if( !block )
        return KOPENAPI_RESULT::Error( 422, "name: " + error );

    if( block->GetBoardFile().IsEmpty() || !wxFileName::FileExists( block->GetBoardFile() ) )
        return KOPENAPI_RESULT::Error( 422, "design block '" + str( name ) + "' has no layout to place" );

    BOARD_COMMIT            commit( context->GetToolManager() );
    std::optional<APPENDED> appended = appendBoard( brd, commit, block->GetBoardFile(), PCB_IO_MGR::KICAD_SEXP, at,
                                                    nlohmann::json(), error, status );

    if( !appended )
    {
        if( error == "nothing to append" )
            return KOPENAPI_RESULT::Error( 422, "design block '" + str( name ) + "' has an empty layout" );

        return KOPENAPI_RESULT::Error( status, "the design block's layout " + error );
    }

    nlohmann::json out = { { "lib_id", str( block->GetLibId().Format() ) },
                           { "items", appended->added.size() },
                           { "nets_added", appended->netsAdded },
                           { "anchor_mm", { toMm( appended->anchor.x + appended->delta.x ),
                                            toMm( appended->anchor.y + appended->delta.y ) } },
                           { "moved_by_mm", { toMm( appended->delta.x ), toMm( appended->delta.y ) } } };

    // a group linked to the block, as the editor's "place as group" (its default)
    if( aArgs.value( "group", true ) )
    {
        if( PCB_GROUP* group = groupAppended( brd, commit, *appended, block->GetLibId().GetLibItemName() ) )
        {
            group->SetDesignBlockLibId( block->GetLibId() );
            out["group"] = { { "uuid", str( group->m_Uuid.AsString() ) },
                             { "name", str( group->GetName() ) },
                             { "design_block", str( group->GetDesignBlockLibId().Format() ) } };
        }
    }

    commit.Push( _( "Place Design Block" ) );
    brd->BuildConnectivity();
    context->SetContentModified();

    // the editor does not reannotate an appended block: say which references now repeat
    reportRefs( brd, *appended, out );

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT h_pcb_append( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         brd = context->GetBoard();
    const wxString path = wx( aArgs.value( "path", std::string() ) );

    if( path.IsEmpty() || !wxFileName( path ).IsAbsolute() )
        return KOPENAPI_RESULT::Error( 400, "path: the absolute path of a board file" );

    if( !wxFileName::FileExists( path ) )
        return KOPENAPI_RESULT::Error( 404, "path: no such file" );

    bool                    badAt = false;
    std::optional<VECTOR2I> at = atMm( aArgs, badAt );

    if( badAt )
        return KOPENAPI_RESULT::Error( 400, "at_mm: [x, y]" );

    if( aArgs.contains( "layer_map" ) && !aArgs["layer_map"].is_object() )
        return KOPENAPI_RESULT::Error( 400, "layer_map: {\"source layer\": \"KiCad layer\" or null}" );

    // KiCad boards and the importers File > Import > Non-KiCad Board offers
    const PCB_IO_MGR::PCB_FILE_T type = PCB_IO_MGR::FindPluginTypeFromBoardPath( path );

    if( type == PCB_IO_MGR::FILE_TYPE_NONE )
        return KOPENAPI_RESULT::Error( 400, "path: not a board format KiCad reads" );

    std::string             error;
    int                     status = 400;
    BOARD_COMMIT            commit( context->GetToolManager() );
    std::optional<APPENDED> appended = appendBoard( brd, commit, path, type, at, aArgs.value( "layer_map", nlohmann::json() ),
                                                    error, status );

    if( !appended )
        return KOPENAPI_RESULT::Error( status, "path: " + error );

    nlohmann::json out = { { "format", str( PCB_IO_MGR::ShowType( type ) ) },
                           { "items", appended->added.size() },
                           { "nets_added", appended->netsAdded },
                           { "anchor_mm", { toMm( appended->anchor.x + appended->delta.x ),
                                            toMm( appended->anchor.y + appended->delta.y ) } },
                           { "moved_by_mm", { toMm( appended->delta.x ), toMm( appended->delta.y ) } } };

    if( aArgs.value( "group", false ) )
    {
        if( PCB_GROUP* group = groupAppended( brd, commit, *appended, wxFileName( path ).GetName() ) )
            out["group"] = { { "uuid", str( group->m_Uuid.AsString() ) }, { "name", str( group->GetName() ) } };
    }

    commit.Push( _( "Append Board" ) );
    brd->BuildConnectivity();
    context->SetContentModified();
    reportRefs( brd, *appended, out );

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

KOPENAPI_REGISTER( "pcb_append",
                   "Append Board, as the editor's (also File > Import > Non-KiCad Board into this board): a KiCad "
                   "board or one KiCad imports (Altium, Eagle, CADSTAR, EasyEDA, PADS, ...) read into this board in "
                   "one undo step: nets by name, this board's stackup kept, the layers it uses enabled; its top-left "
                   "item at at_mm (default where the file has it); group (default false) puts it in one group named "
                   "after the file. A foreign board's layers map by layer_map (source name -> KiCad layer, null to "
                   "drop) else the importer's guess; the answer says the mapping (layer_map, and unmapped_layers left out). References are not renumbered: "
                   "duplicate_refs names the ones now on the board twice",
                   R"json({"type":"object","required":["path"],"properties":{
                        "path":{"type":"string","description":"absolute path"},
                        "at_mm":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},
                        "group":{"type":"boolean","default":false},
                        "layer_map":{"type":"object","additionalProperties":{"type":["string","null"]}}}})json"_json,
                   false, h_pcb_append, 600 );

KOPENAPI_MARK_EDITING( "pcb_design_block_save", "pcb_design_block_place", "pcb_append" );
