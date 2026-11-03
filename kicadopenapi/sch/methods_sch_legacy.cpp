/// @file methods_sch_legacy.cpp
/// @brief kicadopenapi legacy schematic upkeep: sch_rescue_symbols, sch_remap_symbols and
/// sch_migrate_buses, on the cores of the Rescue Symbols wizard, the Remap Symbols dialog and the
/// Migrate Buses dialog.
///
/// The editor's dialogs ask; the API takes the answers as arguments.
#include "kopenapi_sch.h"

#include <api/sch_context.h>
#include <connection_graph.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_util.h>
#include <libraries/symbol_library_adapter.h>
#include <libraries/legacy_symbol_library.h>
#include <kiway.h>
#include <project_rescue.h>
#include <project_sch.h>
#include <reporter.h>
#include <sch_bus_migration.h>
#include <sch_commit.h>
#include <sch_edit_frame.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <symbol_remapper.h>
#include <tool/tool_manager.h>
#include <tools/sch_editor_control.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>


namespace
{

std::string str( const wxString& aText )
{
    return std::string( aText.ToUTF8() );
}


SCH_EDIT_FRAME* editorFrame( KOPENAPI_CONTEXT& aCtx )
{
    if( aCtx.headless || !aCtx.kiway )
        return nullptr;

    return static_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) );
}


/// @brief "names": absent (nullopt), else a list of strings; false and aError otherwise
bool namesArg( const nlohmann::json& aArgs, std::optional<std::vector<wxString>>& aNames, std::string& aError )
{
    if( !aArgs.contains( "names" ) || aArgs["names"].is_null() )
        return true;

    if( !aArgs["names"].is_array() )
    {
        aError = "names: an array of strings";
        return false;
    }

    aNames.emplace();

    for( const nlohmann::json& name : aArgs["names"] )
    {
        if( !name.is_string() )
        {
            aError = "names: an array of strings";
            return false;
        }

        aNames->push_back( wxString::FromUTF8( name.get<std::string>() ) );
    }

    return true;
}


/// @brief The project's symbol libraries loaded (the rescue compares the cache with them)
void waitLibraries( PROJECT& aProject )
{
    SYMBOL_LIBRARY_ADAPTER* adapter = PROJECT_SCH::SymbolLibAdapter( &aProject );
    adapter->AsyncLoad();
    KopenapiWaitLibraries( adapter );
}


/// @brief Every sheet of the open schematic marked unsaved (headless; the editor's OnModify in the GUI)
void markModified( SCHEMATIC& aSchematic )
{
    SCH_SCREENS screens( aSchematic.Root() );

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        if( !screen->GetFileName().IsEmpty() )
            screen->SetContentModified( true );
    }
}


nlohmann::json candidateJson( const RESCUE_CANDIDATE& aCandidate )
{
    return { { "name", str( aCandidate.GetRequestedName() ) },
             { "new_name", str( aCandidate.GetNewName() ) },
             { "action", str( aCandidate.GetActionDescription() ) },
             { "in_cache", aCandidate.GetCacheCandidate() != nullptr },
             { "in_library", aCandidate.GetLibCandidate() != nullptr } };
}


KOPENAPI_RESULT h_sch_rescue_symbols( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context || !context->GetSchematic() )
        return KopenapiNoSchematic();

    std::optional<std::vector<wxString>> names;
    std::string                          argError;

    if( !namesArg( aArgs, names, argError ) )
        return KOPENAPI_RESULT::Error( 400, argError );

    SCHEMATIC*  schematic = context->GetSchematic();
    SCH_SCREENS screens( schematic->Root() );

    waitLibraries( context->Prj() );
    const bool  legacy = screens.HasNoFullyDefinedLibIds();

    // as Rescue Symbols picks it: symbols without a library nickname rescue against the project's
    // legacy libraries, the others against the symbol library table
    std::unique_ptr<RESCUER> rescuer;

    if( legacy )
        rescuer = std::make_unique<LEGACY_RESCUER>( context->Prj(), schematic, nullptr,
                                                    EDA_DRAW_PANEL_GAL::GAL_TYPE_NONE );
    else
        rescuer = std::make_unique<SYMBOL_LIB_TABLE_RESCUER>( context->Prj(), schematic, nullptr,
                                                              EDA_DRAW_PANEL_GAL::GAL_TYPE_NONE );

    rescuer->FindUniqueCandidates();

    nlohmann::json candidates = nlohmann::json::array();

    for( const RESCUE_CANDIDATE& candidate : rescuer->GetCandidates() )
        candidates.push_back( candidateJson( candidate ) );

    wxFileName library( schematic->GetFileName() );
    library.SetName( library.GetName() + wxS( "-rescue" ) );
    library.SetExt( legacy ? FILEEXT::LegacySymbolLibFileExtension : FILEEXT::KiCadSymbolLibFileExtension );

    nlohmann::json out = { { "kind", legacy ? "legacy_libraries" : "library_table" },
                           { "candidates", candidates },
                           { "library", str( library.GetFullPath() ) } };

    if( names )
    {
        for( const wxString& name : *names )
        {
            const auto& all = rescuer->GetCandidates();

            if( std::none_of( all.begin(), all.end(),
                              [&]( const RESCUE_CANDIDATE& aCandidate )
                              {
                                  return aCandidate.GetRequestedName() == name;
                              } ) )
            {
                return KOPENAPI_RESULT::Error( 404, "no rescue candidate named '" + str( name )
                                                            + "' (dry_run: true lists them)" );
            }
        }
    }

    if( aArgs.value( "dry_run", false ) )
        return KOPENAPI_RESULT::Ok( out );

    wxString error;

    if( !RESCUER::RescueChosen( *rescuer, names, error ) )
        return KOPENAPI_RESULT::Error( 500, "the rescue failed: " + str( error ) );

    nlohmann::json rescued = nlohmann::json::array();

    for( const RESCUE_CANDIDATE* candidate : rescuer->GetChosenCandidates() )
        rescued.push_back( candidateJson( *candidate ) );

    out["rescued"] = rescued;

    if( !rescued.empty() )
    {
        if( SCH_EDIT_FRAME* frame = editorFrame( aCtx ) )
            frame->GetToolManager()->GetTool<SCH_EDITOR_CONTROL>()->ShowRescuedSymbols( true );
        else
            markModified( *schematic );
    }

    out["unsaved"] = schematic->HasHierarchy() && schematic->Hierarchy().IsModified();
    return KOPENAPI_RESULT::Ok( out );
}



/// @brief The REPORTER lines a core writes, for the reply
class LINES_REPORTER : public REPORTER
{
public:
    REPORTER& Report( const wxString& aText, SEVERITY aSeverity = RPT_SEVERITY_UNDEFINED ) override
    {
        m_lines.push_back( str( aText ) );
        return *this;
    }

    bool HasMessage() const override { return !m_lines.empty(); }

    nlohmann::json Lines() const { return m_lines; }

private:
    std::vector<std::string> m_lines;
};


std::vector<SCH_SYMBOL*> symbolsWithoutLibrary( SCHEMATIC& aSchematic )
{
    std::vector<SCH_SYMBOL*> symbols;
    SCH_SCREENS              screens( aSchematic.Root() );

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        for( EDA_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( symbol->GetLibId().GetLibNickname().empty() )
                symbols.push_back( symbol );
        }
    }

    return symbols;
}


nlohmann::json symbolJson( SCH_SYMBOL* aSymbol )
{
    return { { "uuid", aSymbol->m_Uuid.AsStdString() },
             { "reference", str( aSymbol->GetField( FIELD_T::REFERENCE )->GetText() ) },
             { "lib_id", str( aSymbol->GetLibId().Format().wx_str() ) } };
}


KOPENAPI_RESULT h_sch_remap_symbols( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context || !context->GetSchematic() )
        return KopenapiNoSchematic();

    SCHEMATIC& schematic = *context->GetSchematic();
    PROJECT&   project = context->Prj();

    if( !SCH_SCREENS( schematic.Root() ).HasNoFullyDefinedLibIds() )
    {
        return KOPENAPI_RESULT::Error( 409, "every symbol names its library already (the symbol library table); "
                                            "nothing to remap" );
    }

    SYMBOL_REMAPPER remapper( project, schematic, true );

    if( aArgs.value( "dry_run", false ) )
    {
        // where the remap finds each symbol: the first legacy library (project order) that has it
        wxString            loadErrors;
        LEGACY_SYMBOL_LIBS* libs = PROJECT_SCH::LegacySchLibs( &project, &loadErrors );
        nlohmann::json      symbols = nlohmann::json::array();
        nlohmann::json      libraries = nlohmann::json::array();

        for( LEGACY_SYMBOL_LIB& lib : *libs )
        {
            if( !lib.IsCache() )
                libraries.push_back( { { "name", str( lib.GetName() ) }, { "path", str( lib.GetFullFileName() ) } } );
        }

        for( SCH_SYMBOL* symbol : symbolsWithoutLibrary( schematic ) )
        {
            nlohmann::json entry = symbolJson( symbol );
            entry["library"] = nullptr;

            for( LEGACY_SYMBOL_LIB& lib : *libs )
            {
                if( !lib.IsCache() && lib.FindSymbol( symbol->GetLibId().GetLibItemName().wx_str() ) )
                {
                    entry["library"] = str( lib.GetName() );
                    break;
                }
            }

            symbols.push_back( entry );
        }

        return KOPENAPI_RESULT::Ok( { { "symbols", symbols },
                                      { "libraries", libraries },
                                      { "load_errors", str( loadErrors ) },
                                      { "writable", remapper.CheckWritable().IsEmpty() } } );
    }

    if( wxString notWritable = remapper.CheckWritable(); !notWritable.IsEmpty() )
        return KOPENAPI_RESULT::Error( 409, str( notWritable ) );

    LINES_REPORTER report;
    nlohmann::json out = nlohmann::json::object();

    if( aArgs.value( "backup", true ) )
    {
        wxString   errors;
        const bool created = remapper.BackupProject( report, errors );

        if( !errors.IsEmpty() && !aArgs.value( "ignore_backup_errors", false ) )
        {
            return KOPENAPI_RESULT::Error( 409, ( created ? "some project files could not be backed up: "
                                                          : std::string() )
                                                        + str( errors )
                                                        + " (the editor asks; ignore_backup_errors: true goes on)" );
        }

        wxFileName folder( project.GetProjectPath(), wxEmptyString );
        folder.AppendDir( SYMBOL_REMAPPER::BackupFolder() );
        out["backup"] = str( folder.GetPath() );
        out["backup_errors"] = str( errors );
    }

    // one last rescue of the legacy library symbols first (Remap Symbols always runs it)
    nlohmann::json rescued = nlohmann::json::array();

    if( aArgs.value( "rescue", true ) )
    {
        waitLibraries( project );

        LEGACY_RESCUER rescuer( project, &schematic, nullptr, EDA_DRAW_PANEL_GAL::GAL_TYPE_NONE );
        wxString       error;

        if( !RESCUER::RescueChosen( rescuer, std::nullopt, error ) )
            return KOPENAPI_RESULT::Error( 500, "the rescue before the remap failed: " + str( error ) );

        for( const RESCUE_CANDIDATE* candidate : rescuer.GetChosenCandidates() )
            rescued.push_back( candidateJson( *candidate ) );
    }

    remapper.Remap( report );

    nlohmann::json remapped = nlohmann::json::array();
    nlohmann::json unmapped = nlohmann::json::array();

    for( SCH_SYMBOL* symbol : remapper.Remapped() )
        remapped.push_back( symbolJson( symbol ) );

    for( SCH_SYMBOL* symbol : remapper.Unmapped() )
        unmapped.push_back( symbolJson( symbol ) );

    if( SCH_EDIT_FRAME* frame = editorFrame( aCtx ) )
        frame->GetToolManager()->GetTool<SCH_EDITOR_CONTROL>()->ShowRescuedSymbols( true );
    else
        markModified( schematic );

    nlohmann::json added = nlohmann::json::array();

    for( const wxString& name : remapper.AddedLibraries() )
        added.push_back( str( name ) );

    out["rescued"] = rescued;
    out["remapped"] = remapped;
    out["unmapped"] = unmapped;
    out["added_libraries"] = added;
    out["library_table"] = str( wxFileName( project.GetProjectPath(), FILEEXT::SymbolLibraryTableFileName ).GetFullPath() );
    out["load_errors"] = str( remapper.LoadErrors() );
    out["report"] = report.Lines();
    out["unsaved"] = schematic.HasHierarchy() && schematic.Hierarchy().IsModified();
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT h_sch_migrate_buses( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context || !context->GetSchematic() )
        return KopenapiNoSchematic();

    SCHEMATIC& schematic = *context->GetSchematic();

    std::vector<BUS_MIGRATION_STATUS> items = SCH_BUS_MIGRATION::Find( schematic );
    nlohmann::json                    migrations = nlohmann::json::array();

    for( size_t i = 0; i < items.size(); ++i )
    {
        nlohmann::json labels = nlohmann::json::array();
        nlohmann::json proposals = nlohmann::json::array();

        for( const wxString& label : items[i].labels )
            labels.push_back( str( label ) );

        for( const wxString& label : items[i].possible_labels )
            proposals.push_back( str( label ) );

        migrations.push_back( { { "index", i },
                                { "sheet", str( items[i].subgraph->GetSheet().PathHumanReadable() ) },
                                { "labels", labels },
                                { "proposals", proposals } } );
    }

    // the chosen name per migration: choices, else the first proposal (the dialog's default)
    std::vector<wxString> chosen;

    for( const BUS_MIGRATION_STATUS& item : items )
        chosen.push_back( item.possible_labels.empty() ? wxString() : item.possible_labels.front() );

    if( aArgs.contains( "choices" ) )
    {
        if( !aArgs["choices"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "choices: an array of {index, label}" );

        std::vector<bool> given( items.size(), false );

        for( const nlohmann::json& choice : aArgs["choices"] )
        {
            if( !choice.is_object() || !choice.contains( "index" ) || !choice["index"].is_number_integer()
                || !choice.contains( "label" ) || !choice["label"].is_string()
                || choice["label"].get<std::string>().empty() )
            {
                return KOPENAPI_RESULT::Error( 400, "choices: an array of {index, label}" );
            }

            const int index = choice["index"].get<int>();

            if( index < 0 || index >= static_cast<int>( items.size() ) )
            {
                return KOPENAPI_RESULT::Error( 404, "choices: no migration " + std::to_string( index )
                                                            + " (dry_run: true lists them)" );
            }

            chosen[index] = wxString::FromUTF8( choice["label"].get<std::string>() );
            given[index] = true;
        }

        // only the migrations named are applied when choices is given
        for( size_t i = 0; i < items.size(); ++i )
        {
            if( !given[i] )
                chosen[i].clear();
        }
    }

    const int      version = schematic.RootScreen() ? schematic.RootScreen()->GetFileFormatVersionAtLoad() : 0;
    nlohmann::json out = { { "migrations", migrations }, { "file_format_version", version } };

    // the search built the connection graph: connectivity as before when nothing changes
    auto restoreConnectivity = [&]()
    {
        if( SCH_EDIT_FRAME* frame = editorFrame( aCtx ) )
            frame->RecalculateConnections( nullptr, NO_CLEANUP );
        else
            schematic.RebuildConnectivity();
    };

    if( aArgs.value( "dry_run", false ) || items.empty() )
    {
        restoreConnectivity();
        out["migrated"] = nlohmann::json::array();
        return KOPENAPI_RESULT::Ok( out );
    }

    SCH_COMMIT     commit( context->GetToolManager() );
    nlohmann::json migrated = nlohmann::json::array();

    for( size_t i = 0; i < items.size(); ++i )
    {
        if( chosen[i].IsEmpty() )
            continue;

        SCH_BUS_MIGRATION::Apply( items[i], chosen[i], &commit );
        migrated.push_back( { { "index", i }, { "label", str( chosen[i] ) } } );
    }

    if( !migrated.empty() )
        KopenapiSchPushEdit( commit, &schematic, _( "Migrate Buses (API)" ), aCtx.kiway, {} );
    else
        restoreConnectivity();

    out["migrated"] = migrated;
    out["unsaved"] = schematic.HasHierarchy() && schematic.Hierarchy().IsModified();
    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "sch_rescue_symbols",
                   "Rescue Symbols (Tools menu; the editor offers it when a legacy project loads): symbols whose "
                   "library version differs from the project's cache (pins), or that only the cache still has, "
                   "get a copy of the cached symbol in <schematic>-rescue (added to the project's symbol library "
                   "table; a .lib for projects without one) and are relinked to it. candidates lists every one "
                   "(name, new_name, action, in_cache, in_library); dry_run: true changes nothing; names picks the "
                   "ones to rescue (default all; an unknown name 404). The schematic is then unsaved",
                   R"json({"type":"object","properties":{
                        "dry_run":{"type":"boolean","default":false},
                        "names":{"type":"array","items":{"type":"string"},
                                 "description":"requested names (candidates[].name) to rescue; default every candidate"}}})json"_json,
                   false, h_sch_rescue_symbols );

KOPENAPI_REGISTER( "sch_remap_symbols",
                   "Remap Symbols (Tools > Remap Legacy Library Symbols...; the editor offers it when a project "
                   "whose symbols name no library loads): the legacy library list becomes a project symbol "
                   "library table and every symbol is relinked through it. Like the dialog it backs up the "
                   "project files to rescue-backup (backup: false skips it; a failed backup 409s unless "
                   "ignore_backup_errors) and runs one last rescue of the legacy symbols (rescue: false skips "
                   "it). dry_run: true lists each symbol with the legacy library that has it. 409 when every "
                   "symbol names its library already. The schematic is then unsaved",
                   R"json({"type":"object","properties":{
                        "dry_run":{"type":"boolean","default":false},
                        "backup":{"type":"boolean","default":true},
                        "ignore_backup_errors":{"type":"boolean","default":false},
                        "rescue":{"type":"boolean","default":true}}})json"_json,
                   false, h_sch_remap_symbols );

KOPENAPI_REGISTER( "sch_migrate_buses",
                   "Migrate Buses (the editor asks when a pre-6.0 schematic loads): a bus with several differently "
                   "named vector labels (joined by index before KiCad 6) gets one name spanning all of them. "
                   "migrations lists each (index, sheet, labels, proposals); dry_run: true changes nothing; "
                   "choices [{index, label}] names the ones to migrate and their new label (default: every "
                   "migration, its first proposal). One undo step; the schematic is then unsaved",
                   R"json({"type":"object","properties":{
                        "dry_run":{"type":"boolean","default":false},
                        "choices":{"type":"array","items":{"type":"object","required":["index","label"],"properties":{
                            "index":{"type":"integer"},"label":{"type":"string"}}}}}})json"_json,
                   false, h_sch_migrate_buses );

KOPENAPI_MARK_EDITING( "sch_rescue_symbols", "sch_remap_symbols", "sch_migrate_buses" );
