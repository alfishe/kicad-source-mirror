/// @file methods_sch_library.cpp
/// @brief kicadopenapi symbol libraries: sch_lib_list, sch_lib_symbol_search, sch_lib_symbol_get.
///
/// Libraries come from the same tables KiCad uses (global sym-lib-table + the project's), via
/// the project's SYMBOL_LIBRARY_ADAPTER; a library that fails to load is reported with its
/// error, never silently skipped.  No document needs to be open: without one, the global
/// tables of the current (default) project apply.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kicadopenapi_libraries.h>
#include <base_units.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <lib_symbol.h>
#include <libraries/library_table.h>
#include <libraries/symbol_library_adapter.h>
#include <project_sch.h>
#include <sch_field.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <schematic.h>

#include <algorithm>
#include <cmath>
#include <map>

using namespace kopenapi_sch;


namespace
{

/// @brief The project whose library tables apply: the open schematic's, else KiCad's current one
PROJECT* libraryProject( KOPENAPI_CONTEXT& aCtx )
{
    if( std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx ) )
        return &context->GetSchematic()->Project();

    return aCtx.kiway ? &aCtx.kiway->Prj() : nullptr;
}


SYMBOL_LIBRARY_ADAPTER* loadedAdapter( KOPENAPI_CONTEXT& aCtx )
{
    PROJECT* project = libraryProject( aCtx );

    if( !project )
        return nullptr;

    SYMBOL_LIBRARY_ADAPTER* adapter = PROJECT_SCH::SymbolLibAdapter( project );
    adapter->AsyncLoad();
    adapter->BlockUntilLoaded();
    return adapter;
}


const char* scopeName( LIBRARY_TABLE_SCOPE aScope )
{
    switch( aScope )
    {
    case LIBRARY_TABLE_SCOPE::GLOBAL:  return "global";
    case LIBRARY_TABLE_SCOPE::PROJECT: return "project";
    default:                           return "unknown";
    }
}


/// @brief Symbol usage in the open schematic, by lib_id
std::map<std::string, int> usage( KOPENAPI_CONTEXT& aCtx )
{
    std::map<std::string, int> used;

    if( std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx ) )
    {
        for( const SCH_SHEET_PATH& path : context->GetSchematic()->Hierarchy() )
        {
            for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
                used[str( static_cast<SCH_SYMBOL*>( item )->GetLibId().Format() )]++;
        }
    }

    return used;
}


nlohmann::json symbolRow( const wxString& aNickname, const LIB_SYMBOL* aSymbol )
{
    nlohmann::json filters = nlohmann::json::array();

    for( const wxString& f : aSymbol->GetFPFilters() )
        filters.push_back( str( f ) );

    return { { "lib_id", str( aNickname ) + ":" + str( aSymbol->GetName() ) },
             { "description", str( aSymbol->GetDescription() ) },
             { "keywords", str( aSymbol->GetKeyWords() ) },
             { "pins", aSymbol->GetPinCount() },
             { "units", aSymbol->GetUnitCount() },
             { "power", aSymbol->IsPower() },
             { "value", str( aSymbol->GetValueField().GetText() ) },
             { "footprint", str( aSymbol->GetFootprintField().GetText() ) },
             { "footprint_filters", filters },
             { "derived_from", aSymbol->IsDerived() && aSymbol->GetRootSymbol()
                                       ? nlohmann::json( str( aSymbol->GetRootSymbol()->GetName() ) )
                                       : nlohmann::json() } };
}

} // namespace


static KOPENAPI_RESULT h_sch_lib_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    SYMBOL_LIBRARY_ADAPTER* adapter = loadedAdapter( aCtx );

    if( !adapter )
        return KOPENAPI_RESULT::Error( 503, "no project to read library tables from" );

    const std::string           glob = aArgs.value( "name", std::string() );
    std::vector<nlohmann::json> rows;
    int                         errors = 0;

    for( const LIBRARY_TABLE_ROW* row : adapter->Rows( LIBRARY_TABLE_SCOPE::BOTH, true ) )
    {
        const std::string nickname = str( row->Nickname() );

        if( !KopenapiGlob( glob, nickname ) )
            continue;

        nlohmann::json out = { { "name", nickname },
                               { "scope", scopeName( row->Scope() ) },
                               { "type", str( row->Type() ) },
                               { "uri", str( row->URI() ) },
                               { "description", str( row->Description() ) },
                               { "enabled", !row->Disabled() } };

        std::optional<LIB_STATUS> status = adapter->GetLibraryStatus( row->Nickname() );

        if( !row->IsOk() )
        {
            out["status"] = "error";
            out["error"] = str( row->ErrorDescription() );
        }
        else if( status && status->load_status == LOAD_STATUS::LOAD_ERROR )
        {
            out["status"] = "error";
            out["error"] = status->error ? str( status->error->message ) : std::string();
        }
        else if( status && status->load_status == LOAD_STATUS::LOADED )
        {
            out["status"] = "loaded";
            out["symbols"] = adapter->GetSymbolNames( row->Nickname() ).size();
        }
        else
        {
            out["status"] = row->Disabled() ? "disabled" : "not_loaded";
        }

        errors += out["status"] == "error" ? 1 : 0;
        rows.push_back( std::move( out ) );
    }

    nlohmann::json result = KopenapiPage( rows, aArgs );
    result["tables"] = KopenapiLibraryTables( *adapter, errors );
    result["errors"] = errors;
    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_sch_lib_symbol_search( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string query = aArgs.value( "query", std::string() );
    const std::string libGlob = aArgs.value( "lib", std::string() );
    const bool        powerOnly = aArgs.value( "power", false );
    const int         minPins = aArgs.value( "min_pins", 0 );
    const int         maxPins = aArgs.value( "max_pins", 0 );

    if( query.empty() && libGlob.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'query' (words) and/or 'lib' (glob)" );

    SYMBOL_LIBRARY_ADAPTER* adapter = loadedAdapter( aCtx );

    if( !adapter )
        return KOPENAPI_RESULT::Error( 503, "no project to read library tables from" );

    const std::map<std::string, int>           used = usage( aCtx );
    std::vector<std::pair<KOPENAPI_TEXT_MATCH, nlohmann::json>> scored;

    for( const wxString& nickname : adapter->GetLibraryNames() )
    {
        if( !KopenapiGlob( libGlob, str( nickname ) ) )
            continue;

        for( LIB_SYMBOL* symbol : adapter->GetSymbols( nickname, powerOnly ? SYMBOL_LIBRARY_ADAPTER::SYMBOL_TYPE::POWER_ONLY
                                                                           : SYMBOL_LIBRARY_ADAPTER::SYMBOL_TYPE::ALL_SYMBOLS ) )
        {
            if( !symbol )
                continue;

            const int pins = symbol->GetPinCount();

            if( pins < minPins || ( maxPins > 0 && pins > maxPins ) )
                continue;

            const KOPENAPI_TEXT_MATCH match = KopenapiTextMatch( query, str( symbol->GetName() ), str( symbol->GetKeyWords() ), str( symbol->GetDescription() ) );

            if( match.terms > 0 && match.matched == 0 )
                continue;

            nlohmann::json row = symbolRow( nickname, symbol );

            auto u = used.find( row["lib_id"] );
            row["used_in_design"] = u == used.end() ? 0 : u->second;
            scored.emplace_back( match, std::move( row ) );
        }
    }

    return KOPENAPI_RESULT::Ok( KopenapiRankedPage( std::move( scored ), aArgs ) );
}


static KOPENAPI_RESULT h_sch_lib_symbol_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string id = aArgs.value( "lib_id", std::string() );
    LIB_ID            libId;

    if( id.empty() || libId.Parse( wxString::FromUTF8( id ) ) >= 0 || libId.GetLibNickname().empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'lib_id' as LIBRARY:SYMBOL" );

    SYMBOL_LIBRARY_ADAPTER* adapter = loadedAdapter( aCtx );

    if( !adapter )
        return KOPENAPI_RESULT::Error( 503, "no project to read library tables from" );

    LIB_SYMBOL* symbol = nullptr;

    try
    {
        symbol = adapter->LoadSymbol( libId );
    }
    catch( const IO_ERROR& error )
    {
        return KOPENAPI_RESULT::Error( 422, "library error: " + str( error.What() ) );
    }

    if( !symbol )
        return KOPENAPI_RESULT::Error( 404, "symbol not found: " + id );

    std::unique_ptr<LIB_SYMBOL> flat = symbol->Flatten();
    nlohmann::json              card = symbolRow( libId.GetLibNickname(), flat.get() );

    // The flattened copy no longer knows its parent: take it from the library symbol
    card["derived_from"] = symbolRow( libId.GetLibNickname(), symbol )["derived_from"];

    nlohmann::json fields = nlohmann::json::object();
    std::vector<SCH_FIELD*> list;
    flat->GetFields( list );

    for( const SCH_FIELD* field : list )
        fields[str( field->GetName() )] = str( field->GetText() );

    // Pins per unit (body style 1; De Morgan alternates draw the same pins)
    nlohmann::json pins = nlohmann::json::array();

    for( const SCH_PIN* pin : flat->GetGraphicalPins( 0, 0 ) )
    {
        if( pin->GetBodyStyle() > 1 )
            continue;

        // Where the pin sits on the body, for planning a placement before placing: the
        // connection point relative to the symbol origin (rotation 0, y down as on the sheet)
        // and the side of the body it leaves from
        const char* side = "left";

        switch( pin->GetOrientation() )
        {
        case PIN_ORIENTATION::PIN_RIGHT: side = "left";   break;   // drawn rightwards into the body
        case PIN_ORIENTATION::PIN_LEFT:  side = "right";  break;
        case PIN_ORIENTATION::PIN_UP:    side = "bottom"; break;
        case PIN_ORIENTATION::PIN_DOWN:  side = "top";    break;
        default:                                          break;
        }

        pins.push_back( { { "number", str( pin->GetNumber() ) },
                          { "name", str( pin->GetShownName() ) },
                          { "type", str( pin->GetElectricalTypeName() ) },
                          { "unit", pin->GetUnit() },
                          { "hidden", !pin->IsVisible() },
                          { "side", side },
                          { "x_mm", std::round( schIUScale.IUTomm( pin->GetPosition().x ) * 1000.0 ) / 1000.0 },
                          { "y_mm", std::round( schIUScale.IUTomm( pin->GetPosition().y ) * 1000.0 ) / 1000.0 } } );
    }

    std::sort( pins.begin(), pins.end(),
               []( const nlohmann::json& a, const nlohmann::json& b )
               {
                   if( a["unit"] != b["unit"] )
                       return a["unit"].get<int>() < b["unit"].get<int>();

                   return KopenapiNaturalLess( a["number"], b["number"] );
               } );

    nlohmann::json units = nlohmann::json::array();

    for( int unit = 1; unit <= flat->GetUnitCount(); ++unit )
        units.push_back( { { "unit", unit }, { "name", str( flat->GetUnitDisplayName( unit, false ) ) } } );

    const std::map<std::string, int> used = usage( aCtx );
    auto                             u = used.find( card["lib_id"] );

    // Body outline of unit 1 in the same frame as the pins: mm from the symbol origin at
    // rotation 0, y growing downwards as on the sheet
    const BOX2I body = flat->GetBodyBoundingBox( 1, 1, false, false );
    auto        mm3 = []( int aIU ) { return std::round( schIUScale.IUTomm( aIU ) * 1000.0 ) / 1000.0; };

    card["coordinates"] = "mm from the symbol origin, rotation 0, y grows downwards (as on the sheet)";
    card["body_mm"] = { mm3( body.GetLeft() ), mm3( body.GetTop() ), mm3( body.GetRight() ), mm3( body.GetBottom() ) };
    card["fields"] = fields;
    card["pin_list"] = pins;
    card["unit_list"] = units;
    card["body_styles"] = flat->GetBodyStyleCount();
    card["datasheet"] = str( flat->GetDatasheetField().GetText() );
    card["used_in_design"] = u == used.end() ? 0 : u->second;
    return KOPENAPI_RESULT::Ok( card );
}


KOPENAPI_REGISTER( "sch_lib_list",
                   "List symbol libraries from KiCad's global and project sym-lib-tables: name, scope, "
                   "type, uri, status (loaded / error with message / disabled), symbol count; errors "
                   "counted; works without an open document; paginated",
                   KopenapiPagedSchema( R"json({"name":{"type":"string","description":"glob on the library name"}})json"_json ),
                   false, h_sch_lib_list, 600 );

KOPENAPI_REGISTER( "sch_lib_symbol_search",
                   "Search library symbols (parts) by words in name, keywords and description, ranked "
                   "(e.g. 'ldo 3.3', 'usb uart', 'opamp dual'); filter by library glob, power symbols, "
                   "pin count; returns lib_id, description, pins, units, default footprint, footprint "
                   "filters, used_in_design; paginated",
                   KopenapiPagedSchema( R"json({
                        "query":{"type":"string","description":"words; items matching all of them, else best partial matches (partial, unmatched_terms)"},
                        "lib":{"type":"string","description":"glob on the library name"},
                        "power":{"type":"boolean","default":false,"description":"power symbols only"},
                        "min_pins":{"type":"integer"},"max_pins":{"type":"integer"}})json"_json ),
                   false, h_sch_lib_symbol_search, 600 );

KOPENAPI_REGISTER( "sch_lib_symbol_get",
                   "Library symbol card by lib_id (LIBRARY:SYMBOL): fields, every pin (number, name, "
                   "electrical type, unit, hidden, side of the body, position from the origin in mm — to plan "
                   "a placement), units with names, body styles, default footprint "
                   "and footprint filters, datasheet, used_in_design",
                   R"json({"type":"object","required":["lib_id"],"properties":{
                        "lib_id":{"type":"string","description":"e.g. Regulator_Linear:AMS1117-3.3"}}})json"_json,
                   false, h_sch_lib_symbol_get, 600 );
