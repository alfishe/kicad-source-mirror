/// @file methods_sch_symbol_lib.cpp
/// @brief kicadopenapi Update Symbols from Library / Change Symbols: placed symbols get their
/// library symbol again (or another one), through SYMBOL_CHANGER as the editor's dialog
#include "kopenapi_sch.h"

#include <api/sch_context.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <ki_exception.h>
#include <lib_symbol.h>
#include <libraries/symbol_library_adapter.h>
#include <project_sch.h>
#include <sch_commit.h>
#include <sch_field.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <string_utils.h>
#include <symbol_changer.h>
#include <template_fieldnames.h>
#include <tool/tool_manager.h>

#include <map>
#include <set>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


std::vector<std::string> globs( const nlohmann::json& aValue )
{
    std::vector<std::string> out;

    if( aValue.is_string() )
        out.push_back( aValue.get<std::string>() );
    else if( aValue.is_array() )
    {
        for( const nlohmann::json& v : aValue )
        {
            if( v.is_string() )
                out.push_back( v.get<std::string>() );
        }
    }

    return out;
}


bool anyGlob( const std::vector<std::string>& aGlobs, const wxString& aText )
{
    for( const std::string& g : aGlobs )
    {
        if( WildCompareString( wxString::FromUTF8( g ), aText, false ) )
            return true;
    }

    return false;
}


/// The references of every instance of a symbol in this schematic, sorted: a symbol on a sheet
/// used twice changes for both, whichever instance matched
std::vector<std::string> refsOf( SCHEMATIC* aSchematic, SCH_SYMBOL* aSymbol, const SYMBOL_CHANGE_INFO& aInfo )
{
    std::set<std::string> refs;
    const SCH_SCREEN*     screen = aInfo.m_Instances.empty() ? nullptr : aInfo.m_Instances[0].LastScreen();

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        if( path.LastScreen() == screen )
            refs.insert( str( aSymbol->GetRef( &path, false ) ) );
    }

    return { refs.begin(), refs.end() };
}

} // namespace


static KOPENAPI_RESULT h_sch_symbol_lib_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC*  schematic = context->GetSchematic();
    const bool  change = aArgs.contains( "lib_id" );
    LIB_ID      newId;

    if( change )
    {
        if( !aArgs["lib_id"].is_string() || newId.Parse( wxString::FromUTF8( aArgs["lib_id"].get<std::string>() ) ) >= 0
            || newId.GetLibNickname().empty() || !newId.IsValid() )
        {
            return KOPENAPI_RESULT::Error( 400, "lib_id: the new symbol, LIBRARY:SYMBOL" );
        }
    }

    const std::vector<std::string> refGlobs = globs( aArgs.value( "refs", nlohmann::json() ) );
    const std::vector<std::string> valueGlobs = globs( aArgs.value( "values", nlohmann::json() ) );
    const std::vector<std::string> idGlobs = globs( aArgs.value( "lib_ids", nlohmann::json() ) );

    if( change && refGlobs.empty() && valueGlobs.empty() && idGlobs.empty() )
        return KOPENAPI_RESULT::Error( 400, "say which symbols to change: refs, values or lib_ids (globs)" );

    for( const char* key : { "refs", "values", "lib_ids" } )
    {
        if( aArgs.contains( key ) && globs( aArgs[key] ).empty() )
            return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": a glob or a list of globs" );
    }

    SYMBOL_LIBRARY_ADAPTER* adapter = PROJECT_SCH::SymbolLibAdapter( &schematic->Project() );
    adapter->AsyncLoad();
    KopenapiWaitLibraries( adapter );

    std::map<LIB_ID, LIB_SYMBOL*> loaded;

    auto lookup = [&]( const LIB_ID& aId ) -> LIB_SYMBOL*
    {
        if( auto it = loaded.find( aId ); it != loaded.end() )
            return it->second;

        LIB_SYMBOL* symbol = nullptr;

        try
        {
            symbol = adapter->LoadSymbol( aId );
        }
        catch( const IO_ERROR& )
        {
        }

        loaded[aId] = symbol;
        return symbol;
    };

    if( change && !lookup( newId ) )
        return KOPENAPI_RESULT::Error( 404, "lib_id: " + str( newId.Format() ) + " is not in the libraries (sch_lib_symbol_search)" );

    // The symbols: every instance that matches (a symbol shared by sheets once, its instances listed)
    std::map<SCH_SYMBOL*, SYMBOL_CHANGE_INFO> symbols;

    for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( !refGlobs.empty() && !anyGlob( refGlobs, UnescapeString( symbol->GetRef( &path, false ) ) ) )
                continue;

            if( !valueGlobs.empty() && !anyGlob( valueGlobs, UnescapeString( symbol->GetField( FIELD_T::VALUE )->GetText() ) ) )
                continue;

            if( !idGlobs.empty() && !anyGlob( idGlobs, UnescapeString( symbol->GetLibId().Format() ) ) )
                continue;

            SYMBOL_CHANGE_INFO& info = symbols[symbol];
            info.m_Instances.push_back( path );
            info.m_LibId = change ? newId : symbol->GetLibId();
        }
    }

    if( symbols.empty() )
        return KOPENAPI_RESULT::Error( 404, "no placed symbol matches (sch_symbol_list)" );

    if( !aArgs.value( "override_locks", false ) )
    {
        nlohmann::json locked = nlohmann::json::array();

        for( const auto& [symbol, info] : symbols )
        {
            if( symbol->IsLocked() )
            {
                for( const std::string& ref : refsOf( schematic, symbol, info ) )
                    locked.push_back( ref );
            }
        }

        if( !locked.empty() )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "locked symbols match (sch_unlock, or override_locks)" );
            r.body["locked"] = locked;
            return r;
        }
    }

    // The dialog's choices; its defaults per mode (Change resets more)
    SYMBOL_CHANGE_OPTIONS options;
    options.m_RemoveExtraFields = aArgs.value( "remove_extra_fields", false );
    options.m_ResetEmptyFields = aArgs.value( "reset_empty_fields", false );
    options.m_ResetFieldText = aArgs.value( "reset_field_text", true );
    options.m_ResetFieldVisibilities = aArgs.value( "reset_field_visibility", change );
    options.m_ResetFieldEffects = aArgs.value( "reset_field_effects", change );
    options.m_ResetFieldPositions = aArgs.value( "reset_field_positions", change );
    options.m_ResetAttributes = aArgs.value( "reset_attributes", change );
    options.m_ResetPinTextVisibility = aArgs.value( "reset_pin_text_visibility", change );
    options.m_ResetAlternatePins = aArgs.value( "reset_alternate_pins", change );
    options.m_ResetCustomPower = aArgs.value( "reset_custom_power", false );
    options.m_ResetPinMapOverrides = aArgs.value( "reset_pin_map_overrides", change );
    options.m_AutoplaceNewFields = aArgs.value( "autoplace_fields", true );

    if( aArgs.contains( "fields" ) )
    {
        if( !aArgs["fields"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "fields: names of the fields to update (Reference, Value, Footprint, ...)" );

        for( const nlohmann::json& f : aArgs["fields"] )
        {
            if( f.is_string() )
                options.m_UpdateFields.insert( wxString::FromUTF8( f.get<std::string>() ) );
        }
    }
    else
    {
        // as the dialog opens: every field but Reference and Value (update_references / update_values)
        for( FIELD_T id : MANDATORY_FIELDS )
        {
            if( ( id == FIELD_T::REFERENCE && !aArgs.value( "update_references", false ) )
                || ( id == FIELD_T::VALUE && !aArgs.value( "update_values", false ) ) )
            {
                continue;
            }

            options.m_UpdateFields.insert( GetDefaultFieldName( id, UNTRANSLATED ) );
        }

        for( const auto& [symbol, info] : symbols )
        {
            for( SCH_FIELD& field : symbol->GetFields() )
            {
                if( !field.IsMandatory() && !field.IsPrivate() )
                    options.m_UpdateFields.insert( field.GetName() );
            }

            if( LIB_SYMBOL* libSymbol = lookup( info.m_LibId ) )
            {
                std::unique_ptr<LIB_SYMBOL> flat = libSymbol->Flatten();
                std::vector<SCH_FIELD*>     libFields;
                flat->GetFields( libFields );

                for( SCH_FIELD* libField : libFields )
                {
                    if( !libField->IsMandatory() && !libField->IsPrivate() )
                        options.m_UpdateFields.insert( libField->GetName() );
                }
            }
        }
    }

    // Which placed symbols differ from their library symbol now (out of date)
    std::map<SCH_SYMBOL*, bool> differed;

    for( const auto& [symbol, info] : symbols )
    {
        LIB_SYMBOL* libSymbol = lookup( info.m_LibId );
        differed[symbol] = !libSymbol || !symbol->GetLibSymbolRef()
                           || symbol->GetLibSymbolRef()->Compare( *libSymbol->Flatten(),
                                                                  ~( SCH_ITEM::COMPARE_FLAGS::UUID
                                                                     | SCH_ITEM::COMPARE_FLAGS::IDENTITY ) ) != 0
                           || symbol->GetLibId() != info.m_LibId;
    }

    const bool dryRun = aArgs.value( "dry_run", false );
    std::map<SCH_SYMBOL*, std::vector<std::string>> refs;

    for( const auto& [symbol, info] : symbols )
        refs[symbol] = refsOf( schematic, symbol, info );

    SCH_COMMIT                        commit( context->GetToolManager() );
    SYMBOL_CHANGER                    changer( lookup, options );
    std::vector<SYMBOL_CHANGE_RESULT> results = changer.Apply( &commit, symbols );

    nlohmann::json items = nlohmann::json::array();
    int            changed = 0;
    int            failed = 0;

    for( const SYMBOL_CHANGE_RESULT& result : results )
    {
        nlohmann::json row = { { "refs", refs[result.m_Symbol] },
                               { "uuid", str( result.m_Symbol->m_Uuid.AsString() ) },
                               { "lib_id", str( result.m_NewLibId.Format() ) },
                               { "was_out_of_date", differed[result.m_Symbol] } };

        if( result.m_OldLibId != result.m_NewLibId )
            row["old_lib_id"] = str( result.m_OldLibId.Format() );

        if( result.m_Error.IsEmpty() )
        {
            row["status"] = dryRun ? "would_change" : "changed";
            changed++;
        }
        else
        {
            row["status"] = "failed";
            row["error"] = str( result.m_Error );
            failed++;
        }

        items.push_back( row );
    }

    if( dryRun || changed == 0 )
    {
        commit.Revert();
    }
    else
    {
        commit.Push( change ? _( "Change Symbols (API)" ) : _( "Update Symbols (API)" ) );
        schematic->RebuildConnectivity();
    }

    nlohmann::json answer = { { "mode", change ? "change" : "update" }, { "matched", symbols.size() },
                              { "changed", changed }, { "failed", failed }, { "dry_run", dryRun } };

    std::vector<std::string> fieldList;

    for( const wxString& f : options.m_UpdateFields )
        fieldList.push_back( str( f ) );

    answer["fields"] = fieldList;
    KOPENAPI_LIST_CAP( aArgs, 100 ).Put( answer, "items", items );

    if( changed && !dryRun )
        answer["next"] = "the board follows with design_update_board (footprints, values, fields)";

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "sch_symbol_lib_update",
                   "Update Symbols from Library / Change Symbols: placed symbols get their library symbol again "
                   "(update, the default) or another one (lib_id: change), as the editor's dialogs. Which: refs, "
                   "values, lib_ids (globs; every one given must match; none: all placed symbols, update only). "
                   "Fields updated: every field but Reference and Value (update_references / update_values), or "
                   "fields: [names]; text from the library (reset_field_text, reset_empty_fields), "
                   "remove_extra_fields, and the resets (visibility, effects, positions, attributes, pin text "
                   "visibility, alternate pins, pin map overrides, custom power value) off for update, on for change. "
                   "Answers per symbol: refs, lib_id, was_out_of_date, status (changed / failed with the reason: "
                   "not found, too few units); locked ones refused (409) unless override_locks; dry_run; one undo step",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "lib_id":{"type":"string","description":"change to this LIBRARY:SYMBOL"},
                        "refs":{"type":["string","array"],"items":{"type":"string"}},
                        "values":{"type":["string","array"],"items":{"type":"string"}},
                        "lib_ids":{"type":["string","array"],"items":{"type":"string"}},
                        "fields":{"type":"array","items":{"type":"string"}},
                        "update_references":{"type":"boolean","default":false},
                        "update_values":{"type":"boolean","default":false},
                        "remove_extra_fields":{"type":"boolean","default":false},
                        "reset_empty_fields":{"type":"boolean","default":false},
                        "reset_field_text":{"type":"boolean","default":true},
                        "reset_field_visibility":{"type":"boolean"},
                        "reset_field_effects":{"type":"boolean"},
                        "reset_field_positions":{"type":"boolean"},
                        "reset_attributes":{"type":"boolean"},
                        "reset_pin_text_visibility":{"type":"boolean"},
                        "reset_alternate_pins":{"type":"boolean"},
                        "reset_custom_power":{"type":"boolean","default":false},
                        "reset_pin_map_overrides":{"type":"boolean"},
                        "autoplace_fields":{"type":"boolean","default":true},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json, 100 ),
                   false, h_sch_symbol_lib_update );

KOPENAPI_MARK_EDITING( "sch_symbol_lib_update" );
