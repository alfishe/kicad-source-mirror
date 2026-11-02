/// @file methods_sch_design_block.cpp
/// @brief kicadopenapi design blocks in the schematic, as the editor's Design Blocks pane: list the
/// libraries and their blocks, Save Sheet / Selection as Design Block, Place Design Block (its
/// schematic loaded into a sheet, annotated, grouped and linked to the block).
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <base_units.h>
#include <design_block.h>
#include <kicadopenapi_design_blocks.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <locale_io.h>
#include <project.h>
#include <sch_commit.h>
#include <sch_design_block_utils.h>
#include <sch_field.h>
#include <sch_group.h>
#include <sch_io/sch_io.h>
#include <sch_io/sch_io_mgr.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_loader.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <tools/sch_tool_utils.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

using namespace kopenapi_sch;


namespace
{

const std::string LIST = "sch_design_block_list";
constexpr double  GRID_MM = 1.27;


std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


wxString wx( const std::string& aText )
{
    return wxString::FromUTF8( aText );
}


double toMm( int aIU )
{
    return std::round( schIUScale.IUTomm( aIU ) * 10000.0 ) / 10000.0;
}


nlohmann::json groupJson( const SCH_GROUP* aGroup )
{
    return { { "uuid", str( aGroup->m_Uuid.AsString() ) },
             { "name", str( aGroup->GetName() ) },
             { "design_block", str( aGroup->GetDesignBlockLibId().Format() ) } };
}


/// @brief A group of aScreen by name, or uuid (prefix)
SCH_GROUP* findGroup( SCH_SCREEN* aScreen, const std::string& aSpec, std::string& aError )
{
    std::vector<SCH_ITEM*> groups;

    for( SCH_ITEM* item : aScreen->Items().OfType( SCH_GROUP_T ) )
    {
        if( str( static_cast<SCH_GROUP*>( item )->GetName() ) == aSpec )
            return static_cast<SCH_GROUP*>( item );

        groups.push_back( item );
    }

    return static_cast<SCH_GROUP*>( KopenapiSchMatchUuid( groups, aSpec, aError ) );
}


/// @brief An item of the sheet: a symbol by reference, else any item by uuid (prefix)
SCH_ITEM* findItem( const SCH_SHEET_PATH& aPath, const std::string& aSpec, std::string& aError )
{
    std::vector<SCH_ITEM*> scope;

    for( SCH_ITEM* item : aPath.LastScreen()->Items() )
    {
        if( item->Type() == SCH_SYMBOL_T
            && str( static_cast<SCH_SYMBOL*>( item )->GetRef( &aPath, false ) ) == aSpec )
        {
            return item;
        }

        scope.push_back( item );
    }

    return KopenapiSchMatchUuid( scope, aSpec, aError );
}


/// @brief Save aSheet as the schematic file aPath, as the editor saves the block's temporary copy
void writeSchematic( SCHEMATIC* aSchematic, SCH_SHEET* aSheet, const wxString& aPath )
{
    LOCALE_IO           toggle;
    IO_RELEASER<SCH_IO> pi( SCH_IO_MGR::FindPlugin( SCH_IO_MGR::SCH_KICAD ) );
    pi->SaveSchematicFile( aPath, aSheet, aSchematic );
}


KOPENAPI_RESULT h_sch_design_block_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    return KopenapiDesignBlockList( &context->GetSchematic()->Project(), aArgs, LIST );
}


KOPENAPI_RESULT h_sch_design_block_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC* schematic = context->GetSchematic();
    PROJECT*   project = &schematic->Project();

    std::string                        error;
    int                                status = 400;
    std::optional<KOPENAPI_DB_LIBRARY> lib = KopenapiDesignBlockLibrary( project, aArgs, false, LIST, error, status );

    if( !lib )
        return KOPENAPI_RESULT::Error( status, error );

    if( !KopenapiDesignBlockLibraryWritable( project, *lib ) )
        return KOPENAPI_RESULT::Error( 409, "library: '" + str( lib->nickname ) + "' is read-only" );

    std::optional<SCH_SHEET_PATH> path = KopenapiSchTargetSheet( *context, aArgs );

    if( !path )
        return KOPENAPI_RESULT::Error( 404, "sheet: not found (sch_sheet_list)" );

    if( aArgs.contains( "items" ) && aArgs.contains( "group" ) )
        return KOPENAPI_RESULT::Error( 400, "items or group, not both" );

    // what to save: a whole sheet (Save Sheet as Design Block), one group (its members; the group
    // gets the link) or items (they join a new linked group)
    SCH_SCREEN*            screen = path->LastScreen();
    SCH_GROUP*             group = nullptr;
    std::vector<EDA_ITEM*> items;
    std::optional<SCH_SHEET_PATH> sheetSource;

    if( aArgs.contains( "group" ) )
    {
        group = findGroup( screen, aArgs.value( "group", std::string() ), error );

        if( !group )
            return KOPENAPI_RESULT::Error( 404, "group: no group " + aArgs["group"].dump() + " on the sheet (name or uuid)"
                                                        + ( error.empty() ? "" : ": " + error ) );

        group->RunOnChildren( [&]( EDA_ITEM* aItem ) { items.push_back( aItem ); }, RECURSE_MODE::NO_RECURSE );

        if( items.empty() )
            return KOPENAPI_RESULT::Error( 400, "group: the group is empty" );
    }
    else if( aArgs.contains( "items" ) )
    {
        if( !aArgs["items"].is_array() || aArgs["items"].empty() )
            return KOPENAPI_RESULT::Error( 400, "items: references and uuids of the sheet's items" );

        std::set<EDA_ITEM*> seen;

        for( const nlohmann::json& spec : aArgs["items"] )
        {
            SCH_ITEM* item = spec.is_string() ? findItem( *path, spec.get<std::string>(), error ) : nullptr;

            if( !item )
                return KOPENAPI_RESULT::Error( 404, "items: no item " + spec.dump() + " on the sheet"
                                                            + ( error.empty() ? "" : ": " + error ) );

            if( seen.insert( item ).second )
                items.push_back( item );
        }

        bool hasSheet = std::any_of( items.begin(), items.end(),
                                     []( EDA_ITEM* aItem ) { return aItem->Type() == SCH_SHEET_T; } );

        if( hasSheet && items.size() > 1 )
            return KOPENAPI_RESULT::Error( 400, "items: design blocks with nested sheets are not supported: give one "
                                                "sheet alone (its content is saved) or no sheet" );

        // a sheet alone saves its content, as the editor does for a selected sheet
        if( hasSheet )
        {
            sheetSource = *path;
            sheetSource->push_back( static_cast<SCH_SHEET*>( items.front() ) );
            items.clear();
        }
    }
    else
    {
        sheetSource = *path;
    }

    if( sheetSource )
    {
        std::vector<SCH_ITEM*> nested;
        sheetSource->LastScreen()->GetSheets( &nested );

        if( !nested.empty() )
            return KOPENAPI_RESULT::Error( 400, "the sheet holds sheets: design blocks with nested sheets are not "
                                                "supported (save its content as items instead)" );
    }

    wxString name = wx( aArgs.value( "name", std::string() ) );

    if( name.IsEmpty() )
    {
        if( group && !group->GetName().IsEmpty() )
            name = group->GetName();
        else if( sheetSource && !sheetSource->Last()->GetName().IsEmpty() )
            name = wxFileName( sheetSource->Last()->GetName() ).GetName();
        else
            name = wxFileName( screen->GetFileName() ).GetName();
    }

    if( name.IsEmpty() || name.find_first_of( wxS( "/\\:" ) ) != wxString::npos )
        return KOPENAPI_RESULT::Error( 400, "name: the design block's name (no / \\ :)" );

    const bool exists = KopenapiDesignBlockExists( project, *lib, name );

    if( exists && !aArgs.value( "overwrite", false ) )
        return KOPENAPI_RESULT::Error( 409, "design block '" + str( name ) + "' already exists in " + lib->Name()
                                                    + ": overwrite: true replaces its schematic" );

    // overwriting keeps the block's description, keywords, fields and layout, as Update Design
    // Block from Sheet / Selection does
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

    // a sheet's fields go to the block, as the editor does
    if( sheetSource )
    {
        for( SCH_FIELD& field : sheetSource->Last()->GetFields() )
        {
            if( field.GetId() == FIELD_T::SHEET_NAME || field.GetId() == FIELD_T::SHEET_FILENAME )
                continue;

            block.GetFields()[field.GetUntranslatedName()] = field.GetText();
        }
    }

    if( aArgs.contains( "description" ) )
        block.SetLibDescription( wx( aArgs.value( "description", std::string() ) ) );

    if( aArgs.contains( "keywords" ) )
        block.SetKeywords( wx( aArgs.value( "keywords", std::string() ) ) );

    const wxString tempFile = wxFileName::CreateTempFileName( "design_block" );

    try
    {
        if( sheetSource )
        {
            writeSchematic( schematic, sheetSource->Last(), tempFile );
        }
        else
        {
            std::unique_ptr<SCH_SHEET> blockSheet = MakeDesignBlockSheet( schematic, items );
            writeSchematic( schematic, blockSheet.get(), tempFile );
        }

        block.SetSchematicFile( tempFile );
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
                           { "sheet", sheetPath( sheetSource ? *sheetSource : *path ) },
                           { "items", sheetSource ? nlohmann::json( "sheet" ) : nlohmann::json( items.size() ) } };

    if( lib->IsPath() )
        out["path"] = str( lib->path + wxFileName::GetPathSeparator() + name + wxS( "." )
                           + wxString( FILEEXT::KiCadDesignBlockPathExtension ) );

    // the saved items join the block: a new group linked to it, or the link on the group given
    if( !sheetSource )
    {
        SCH_COMMIT commit( context->GetToolManager() );

        if( group )
        {
            if( !group->HasDesignBlockLink() )
            {
                commit.Modify( group, screen, RECURSE_MODE::NO_RECURSE );
                group->SetDesignBlockLibId( block.GetLibId() );
                KopenapiSchPushEdit( commit, schematic, _( "Set Group Design Block Link" ), aCtx.kiway, {} );
            }

            out["group"] = groupJson( group );
        }
        else
        {
            SCH_GROUP* newGroup = new SCH_GROUP( screen );
            newGroup->SetName( block.GetLibId().GetUniStringLibItemName() );
            newGroup->SetDesignBlockLibId( block.GetLibId() );
            int added = 0;

            for( EDA_ITEM* edaItem : items )
            {
                SCH_ITEM* item = static_cast<SCH_ITEM*>( edaItem );

                if( item->GetParentSymbol() || !item->IsGroupableType() )
                    continue;

                if( EDA_GROUP* existing = item->GetParentGroup() )
                    commit.Modify( existing->AsEdaItem(), screen, RECURSE_MODE::NO_RECURSE );

                commit.Modify( item, screen, RECURSE_MODE::NO_RECURSE );
                newGroup->AddItem( item );
                added++;
            }

            if( added > 0 )
            {
                commit.Add( newGroup, screen );
                KopenapiSchPushEdit( commit, schematic, _( "Group Items" ), aCtx.kiway, { newGroup->m_Uuid } );
                out["group"] = groupJson( newGroup );
            }
            else
            {
                newGroup->RemoveAll();
                delete newGroup;
            }
        }
    }

    return KOPENAPI_RESULT::Ok( out );
}


/// The API's answers to a sheet load: go on, and say what the editor would have asked
class API_SHEET_LOAD_HANDLER : public SCH_SHEET_LOAD_HANDLER
{
public:
    bool Continue( const wxString& aTitle, const wxString& aMessage, const wxString& aDetail,
                   const wxString& aContinueLabel ) override
    {
        m_warnings.push_back( str( aMessage + ( aDetail.IsEmpty() ? wxString() : wxS( " " ) + aDetail ) ) );
        return true;
    }

    void Error( const wxString& aMessage, const wxString& aDetail ) override
    {
        m_errors.push_back( str( aMessage + ( aDetail.IsEmpty() ? wxString() : wxS( " " ) + aDetail ) ) );
    }

    nlohmann::json m_warnings = nlohmann::json::array();
    std::vector<std::string> m_errors;
};


/// @brief "U3" -> "U", "#PWR01" -> "#PWR", "R?" -> "R"
wxString refPrefix( const wxString& aRef )
{
    wxString prefix = aRef;

    while( !prefix.IsEmpty() && ( wxIsdigit( prefix.Last() ) || prefix.Last() == '?' ) )
        prefix.RemoveLast();

    return prefix;
}


KOPENAPI_RESULT h_sch_design_block_place( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC* schematic = context->GetSchematic();
    PROJECT*   project = &schematic->Project();

    std::string                        error;
    int                                status = 400;
    std::optional<KOPENAPI_DB_LIBRARY> lib = KopenapiDesignBlockLibrary( project, aArgs, true, LIST, error, status );

    if( !lib )
        return KOPENAPI_RESULT::Error( status, error );

    const wxString name = wx( aArgs.value( "name", std::string() ) );

    if( name.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "name: the design block (" + LIST + ")" );

    const std::string annotate = aArgs.value( "annotate", std::string( "auto" ) );

    if( annotate != "auto" && annotate != "keep" && annotate != "clear" )
        return KOPENAPI_RESULT::Error( 400, "annotate: auto (first free references), keep or clear" );

    std::optional<VECTOR2I> at;

    if( aArgs.contains( "at_mm" ) )
    {
        const nlohmann::json& a = aArgs["at_mm"];

        if( !a.is_array() || a.size() != 2 || !a[0].is_number() || !a[1].is_number() )
            return KOPENAPI_RESULT::Error( 400, "at_mm: [x, y]" );

        // on the connection grid, as the editor's cursor
        at = VECTOR2I( schIUScale.mmToIU( std::round( a[0].get<double>() / GRID_MM ) * GRID_MM ),
                       schIUScale.mmToIU( std::round( a[1].get<double>() / GRID_MM ) * GRID_MM ) );
    }

    std::optional<SCH_SHEET_PATH> path = KopenapiSchTargetSheet( *context, aArgs );

    if( !path )
        return KOPENAPI_RESULT::Error( 404, "sheet: not found (sch_sheet_list)" );

    if( !KopenapiDesignBlockExists( project, *lib, name ) )
        return KOPENAPI_RESULT::Error( 404, "name: no design block '" + str( name ) + "' in " + lib->Name() );

    std::unique_ptr<DESIGN_BLOCK> block = KopenapiLoadDesignBlock( project, *lib, name, error );

    if( !block )
        return KOPENAPI_RESULT::Error( 422, "name: " + error );

    if( block->GetSchematicFile().IsEmpty() || !wxFileName::FileExists( block->GetSchematicFile() ) )
        return KOPENAPI_RESULT::Error( 422, "design block '" + str( name ) + "' has no schematic to place" );

    // As the editor's Place Design Block: the block's schematic loaded into the sheet, the items
    // it brings marked by what was there before
    SCH_SCREEN*                   screen = path->LastScreen();
    std::unordered_set<EDA_ITEM*> existing;

    for( EDA_ITEM* item : screen->Items() )
        existing.insert( item );

    API_SHEET_LOAD_HANDLER handler;

    if( !SchLoadSheetFromFile( *schematic, path->Last(), &*path, block->GetSchematicFile(), true, true, handler ) )
    {
        std::string why;

        for( const std::string& e : handler.m_errors )
            why += ( why.empty() ? "" : "; " ) + e;

        return KOPENAPI_RESULT::Error( 422, "the design block's schematic could not be loaded"
                                                    + ( why.empty() ? std::string() : ": " + why ) );
    }

    std::vector<SCH_ITEM*> added;
    std::vector<SCH_ITEM*> top;

    for( SCH_ITEM* item : screen->Items() )
    {
        if( existing.contains( item ) )
            continue;

        added.push_back( item );

        if( !item->GetParentGroup() )
            top.push_back( item );
    }

    if( added.empty() )
        return KOPENAPI_RESULT::Error( 422, "design block '" + str( name ) + "' has an empty schematic" );

    SCH_COMMIT commit( context->GetToolManager() );

    for( SCH_ITEM* item : added )
        commit.Added( item, screen );

    // references: the block's kept, cleared ("R?"), or the first free ones, the units of one
    // symbol in the block sharing theirs
    std::vector<SCH_SYMBOL*> symbols;

    for( SCH_ITEM* item : added )
    {
        if( item->Type() == SCH_SYMBOL_T )
            symbols.push_back( static_cast<SCH_SYMBOL*>( item ) );
    }

    std::map<SCH_SYMBOL*, wxString> was;

    for( SCH_SYMBOL* symbol : symbols )
        was[symbol] = symbol->GetRef( &*path, false );

    if( annotate != "keep" )
    {
        for( SCH_SYMBOL* symbol : symbols )
            symbol->ClearAnnotation( &*path, false );
    }

    if( annotate == "auto" )
    {
        std::set<wxString>           taken;
        std::map<wxString, wxString> renamed;

        for( SCH_SYMBOL* symbol : symbols )
        {
            const wxString& old = was[symbol];
            const bool      annotated = !old.IsEmpty() && !old.EndsWith( wxS( "?" ) );

            if( annotated && renamed.count( old ) )
            {
                symbol->SetRef( &*path, renamed[old] );
                continue;
            }

            wxString ref = KopenapiSchNextFreeRef( schematic, refPrefix( old ), taken );
            symbol->SetRef( &*path, ref );

            if( annotated )
                renamed[old] = ref;
        }
    }

    // the block's anchor (its top-left connection point) at at_mm
    std::vector<SCH_ITEM*> flat = FlattenGroups( EDA_ITEMS( top.begin(), top.end() ) );
    std::optional<VECTOR2I> anchor = DesignBlockAnchor( flat );
    const VECTOR2I          delta = at && anchor ? *at - *anchor : VECTOR2I( 0, 0 );

    if( delta != VECTOR2I( 0, 0 ) )
    {
        for( SCH_ITEM* item : top )
            item->Move( delta );
    }

    nlohmann::json out = { { "lib_id", str( block->GetLibId().Format() ) },
                           { "sheet", sheetPath( *path ) },
                           { "items", added.size() },
                           { "moved_by_mm", { toMm( delta.x ), toMm( delta.y ) } } };

    if( anchor )
        out["anchor_mm"] = { toMm( anchor->x + delta.x ), toMm( anchor->y + delta.y ) };

    std::vector<KIID> glow;

    // a group linked to the block, as the editor's "place as group" (its default)
    if( aArgs.value( "group", true ) )
    {
        SCH_GROUP* group = new SCH_GROUP( screen );
        group->SetName( UniqueGroupName( screen, block->GetLibId().GetLibItemName().wx_str() ) );
        group->SetDesignBlockLibId( block->GetLibId() );

        for( SCH_ITEM* item : top )
            group->AddItem( item );

        commit.Add( group, screen );
        glow.push_back( group->m_Uuid );
        out["group"] = groupJson( group );
    }
    else
    {
        for( SCH_ITEM* item : top )
            glow.push_back( item->m_Uuid );
    }

    KopenapiSchPushEdit( commit, schematic, _( "Add Design Block" ), aCtx.kiway, glow );

    nlohmann::json refs = nlohmann::json::array();
    std::map<wxString, std::set<SCH_SYMBOL*>> byRef;

    for( const SCH_SHEET_PATH& sheet : schematic->Hierarchy() )
    {
        for( SCH_ITEM* item : sheet.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );
            byRef[symbol->GetRef( &sheet, false )].insert( symbol );
        }
    }

    std::set<wxString> duplicates;

    for( SCH_SYMBOL* symbol : symbols )
    {
        const wxString ref = symbol->GetRef( &*path, false );
        nlohmann::json entry = { { "ref", str( ref ) }, { "was", str( was[symbol] ) }, { "unit", symbol->GetUnit() } };
        refs.push_back( entry );

        // the units of one multi-unit symbol share a reference: a repeat is another symbol's
        bool repeated = false;

        for( SCH_SYMBOL* other : byRef[ref] )
        {
            if( other != symbol && ( other->GetLibId() != symbol->GetLibId() || other->GetUnit() == symbol->GetUnit() ) )
                repeated = true;
        }

        if( repeated && !ref.EndsWith( wxS( "?" ) ) )
            duplicates.insert( ref );
    }

    out["symbols"] = refs;

    if( !duplicates.empty() )
    {
        nlohmann::json dup = nlohmann::json::array();

        for( const wxString& ref : duplicates )
            dup.push_back( str( ref ) );

        out["duplicate_refs"] = dup;
    }

    if( !handler.m_warnings.empty() )
        out["warnings"] = handler.m_warnings;

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "sch_design_block_list",
                   "Design block libraries and their blocks, as the schematic editor's Design Blocks pane: without "
                   "library the libraries of the global and project tables (nickname, description, blocks); with "
                   "library (a nickname, or the absolute path of a .kicad_blocks directory) its blocks: name, lib_id, "
                   "description, keywords, has_layout, has_schematic. The same libraries pcb_design_block_list lists",
                   KopenapiDesignBlockListSchema(), false, h_sch_design_block_list );

KOPENAPI_REGISTER( "sch_design_block_save",
                   "Save as Design Block, as the schematic editor's: without items / group the sheet (default the "
                   "current one, or sheet: a path of sch_sheet_list) with its fields (Save Sheet as Design Block); "
                   "with group (name or uuid) its members, and the group gets the block's link; with items (REF, "
                   "uuids of the sheet's wires, labels, texts, groups) those, and they join a new group linked to "
                   "the block (Save Selection as Design Block); one sheet symbol as items saves that sheet's "
                   "content. Sheets holding sheets are refused, as in the editor. library: a nickname of the design "
                   "block library tables, or the absolute path of a .kicad_blocks directory (created if missing); "
                   "name default the group's / sheet's name; description, keywords. An existing block is refused "
                   "(409) unless overwrite: true, which replaces its schematic and keeps its description, keywords, "
                   "fields and layout unless given",
                   R"json({"type":"object","required":["library"],"properties":{
                        "library":{"type":"string"},"name":{"type":"string"},"sheet":{"type":"string"},
                        "items":{"type":"array","items":{"type":"string"}},
                        "group":{"type":"string"},
                        "description":{"type":"string"},"keywords":{"type":"string"},
                        "overwrite":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_design_block_save );

KOPENAPI_REGISTER( "sch_design_block_place",
                   "Place Design Block, as the schematic editor's: the block's schematic loaded into the sheet "
                   "(default the current one, or sheet: a path of sch_sheet_list), its anchor (the top-left "
                   "connection point) at at_mm on the 1.27 mm grid (default where the block has it), with group "
                   "(default true, the editor's \"place as group\") one group named after the block and linked to "
                   "it. annotate: auto (default; the first free references, the units of one symbol sharing theirs), "
                   "keep (the block's; duplicate_refs names repeats) or clear (R?). symbols: ref, was, unit. "
                   "warnings: what the editor would have asked (partial file, old format). One undo step",
                   R"json({"type":"object","required":["library","name"],"properties":{
                        "library":{"type":"string"},"name":{"type":"string"},"sheet":{"type":"string"},
                        "at_mm":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},
                        "annotate":{"type":"string","enum":["auto","keep","clear"],"default":"auto"},
                        "group":{"type":"boolean","default":true}}})json"_json,
                   false, h_sch_design_block_place );

KOPENAPI_MARK_EDITING( "sch_design_block_save", "sch_design_block_place" );
