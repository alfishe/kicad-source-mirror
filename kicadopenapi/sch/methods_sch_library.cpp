/// @file methods_sch_library.cpp
/// @brief kicadopenapi symbol libraries: sch_lib_list, sch_lib_symbol_search, sch_lib_symbol_get,
/// sch_lib_symbol_save.
///
/// Libraries come from the same tables KiCad uses (global sym-lib-table + the project's), via
/// the project's SYMBOL_LIBRARY_ADAPTER; a library that fails to load is reported with its
/// error, never silently skipped.  No document needs to be open: without one, the global
/// tables of the current (default) project apply.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_libraries.h>
#include <base_units.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <lib_symbol.h>
#include <libraries/library_table.h>
#include <libraries/symbol_library_adapter.h>
#include <project_sch.h>
#include <sch_field.h>
#include <sch_io/kicad_sexpr/sch_io_kicad_sexpr.h>
#include <sch_io/sch_io_mgr.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <wildcards_and_files_ext.h>
#include <wx_filename.h>

#include <wx/filename.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

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
    KopenapiWaitLibraries( adapter );
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


/// @brief A size word ("3.5mm", "6.35mm", "5x5mm"): it describes the footprint, a symbol rarely names it
static bool isDimensionWord( const std::string& aWord )
{
    std::string w = aWord;

    for( char& ch : w )
        ch = static_cast<char>( std::tolower( static_cast<unsigned char>( ch ) ) );

    if( w.size() < 3 || !w.ends_with( "mm" ) || !std::isdigit( static_cast<unsigned char>( w[0] ) ) )
        return false;

    return std::all_of( w.begin(), w.end() - 2, []( char c ) { return std::isdigit( static_cast<unsigned char>( c ) ) || c == '.' || c == 'x'; } );
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

    // Size words rank but do not filter: "audio jack 3.5mm" finds AudioJack3, whose text has no size
    std::string hard, soft;

    {
        std::istringstream in( query );
        std::string        word;

        while( in >> word )
        {
            std::string& to = isDimensionWord( word ) ? soft : hard;
            to += ( to.empty() ? "" : " " ) + word;
        }
    }

    if( hard.empty() )
        std::swap( hard, soft );

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

            KOPENAPI_TEXT_MATCH match = KopenapiTextMatch( hard, str( symbol->GetName() ), str( symbol->GetKeyWords() ), str( symbol->GetDescription() ) );

            if( !soft.empty() )
                match.score += KopenapiTextMatch( soft, str( symbol->GetName() ), str( symbol->GetKeyWords() ), str( symbol->GetDescription() ) ).score / 4;

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


namespace
{

/// @brief A symbol to save: its cached library symbol and the lib_id it came from
struct SAVE_ITEM
{
    const LIB_SYMBOL* symbol = nullptr;
    std::string       libId;
    wxString          name;
};


/// @brief The cached library symbol of each placed symbol, by the placed symbol's lib_id
std::map<std::string, const LIB_SYMBOL*> placedLibSymbols( SCHEMATIC* aSchematic, std::vector<std::string>* aDiffering )
{
    std::map<std::string, const LIB_SYMBOL*> out;

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            const SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );
            const LIB_SYMBOL* cached = symbol->GetLibSymbolRef().get();

            if( !cached )
                continue;

            const std::string id = str( symbol->GetLibId().Format() );
            auto [it, inserted] = out.emplace( id, cached );

            if( !inserted && aDiffering && it->second != cached
                && it->second->Compare( *cached, ~( SCH_ITEM::COMPARE_FLAGS::UUID | SCH_ITEM::COMPARE_FLAGS::IDENTITY ) ) != 0
                && std::find( aDiffering->begin(), aDiffering->end(), id ) == aDiffering->end() )
            {
                aDiffering->push_back( id );
            }
        }
    }

    return out;
}

} // namespace


static KOPENAPI_RESULT h_sch_lib_symbol_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string library = aArgs.value( "library", std::string() );
    const bool        allMissing = aArgs.value( "all_missing", false );
    const bool        overwrite = aArgs.value( "overwrite", false );
    const int         sources = ( aArgs.contains( "lib_id" ) ? 1 : 0 ) + ( aArgs.contains( "ref" ) ? 1 : 0 ) + ( allMissing ? 1 : 0 );

    if( library.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'library': a sym-lib-table nickname or a .kicad_sym path" );

    if( sources != 1 )
        return KOPENAPI_RESULT::Error( 400, "give exactly one of 'lib_id', 'ref', 'all_missing: true'" );

    if( allMissing && aArgs.contains( "name" ) )
        return KOPENAPI_RESULT::Error( 400, "'name' applies to one symbol (lib_id / ref), not to all_missing" );

    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC*              schematic = context->GetSchematic();
    PROJECT*                project = &schematic->Project();
    SYMBOL_LIBRARY_ADAPTER* adapter = loadedAdapter( aCtx );

    if( !adapter )
        return KOPENAPI_RESULT::Error( 503, "no project to read library tables from" );

    // Target: a table row by nickname, else a file path (relative to the project)
    const LIBRARY_TABLE_ROW* row = nullptr;
    wxString                 path;

    if( std::optional<LIBRARY_TABLE_ROW*> found = adapter->GetRow( wxString::FromUTF8( library ) ); found && *found )
    {
        row = *found;

        if( SCH_IO_MGR::EnumFromStr( row->Type() ) != SCH_IO_MGR::SCH_KICAD )
            return KOPENAPI_RESULT::Error( 422, "library '" + library + "' is of type " + str( row->Type() ) + ", not a KiCad .kicad_sym library" );

        path = LIBRARY_MANAGER::GetFullURI( row, true, project );
    }
    else
    {
        wxFileName fn( wxString::FromUTF8( library ) );

        if( fn.GetExt() != wxString::FromUTF8( FILEEXT::KiCadSymbolLibFileExtension ) )
            return KOPENAPI_RESULT::Error( 400, "library '" + library + "' is neither a sym-lib-table nickname nor a ."
                                                        + FILEEXT::KiCadSymbolLibFileExtension + " path" );

        if( !fn.IsAbsolute() )
            fn.MakeAbsolute( project->GetProjectPath() );

        fn.Normalize( FN_NORMALIZE_FLAGS );
        path = fn.GetFullPath();

        if( std::optional<LIBRARY_TABLE_ROW*> byUri = adapter->FindRowByURI( path ); byUri && *byUri )
            row = *byUri;
    }

    // Symbols to save
    std::vector<SAVE_ITEM> items;
    nlohmann::json         skipped = nlohmann::json::array();
    int                    inLibrary = 0;

    if( allMissing )
    {
        std::vector<std::string> differing;

        for( const auto& [id, cached] : placedLibSymbols( schematic, &differing ) )
        {
            LIB_ID libId;

            if( libId.Parse( wxString::FromUTF8( id ) ) >= 0 || libId.GetLibItemName().empty() )
            {
                skipped.push_back( { { "lib_id", id }, { "reason", "lib_id does not parse" } } );
                continue;
            }

            LIB_SYMBOL* existing = nullptr;

            try
            {
                if( !libId.GetLibNickname().empty() )
                    existing = adapter->LoadSymbol( libId );
            }
            catch( const IO_ERROR& )
            {
                // a broken library lacks the symbol as much as a missing one
            }

            if( existing )
            {
                inLibrary++;
                continue;
            }

            if( std::find( differing.begin(), differing.end(), id ) != differing.end() )
                skipped.push_back( { { "lib_id", id }, { "reason", "placed symbols of this lib_id carry differing cached copies; the first one is saved" } } );

            items.push_back( { cached, id, libId.GetLibItemName().wx_str() } );
        }
    }
    else
    {
        SAVE_ITEM item;

        if( aArgs.contains( "ref" ) )
        {
            const wxString ref = wxString::FromUTF8( aArgs.value( "ref", std::string() ) );

            for( const SCH_SHEET_PATH& sheetPath : schematic->Hierarchy() )
            {
                for( SCH_ITEM* schItem : sheetPath.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
                {
                    SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( schItem );

                    if( !item.symbol && symbol->GetRef( &sheetPath, false ) == ref && symbol->GetLibSymbolRef() )
                    {
                        item.symbol = symbol->GetLibSymbolRef().get();
                        item.libId = str( symbol->GetLibId().Format() );
                        item.name = symbol->GetLibId().GetLibItemName().wx_str();
                    }
                }
            }

            if( !item.symbol )
                return KOPENAPI_RESULT::Error( 404, "no placed symbol with ref " + str( ref ) );
        }
        else
        {
            const std::string id = aArgs.value( "lib_id", std::string() );
            LIB_ID            libId;

            if( id.empty() || libId.Parse( wxString::FromUTF8( id ) ) >= 0 || libId.GetLibNickname().empty()
                || libId.GetLibItemName().empty() )
                return KOPENAPI_RESULT::Error( 400, "give 'lib_id' as LIBRARY:SYMBOL" );

            const std::map<std::string, const LIB_SYMBOL*> placed = placedLibSymbols( schematic, nullptr );

            if( auto it = placed.find( id ); it != placed.end() )
                item.symbol = it->second;

            // A cache entry no placed symbol uses
            SCH_SCREENS screens( schematic->Root() );

            for( SCH_SCREEN* screen = screens.GetFirst(); screen && !item.symbol; screen = screens.GetNext() )
            {
                auto it = screen->GetLibSymbols().find( wxString::FromUTF8( id ) );

                if( it != screen->GetLibSymbols().end() )
                    item.symbol = it->second;
            }

            if( !item.symbol )
                return KOPENAPI_RESULT::Error( 404, "lib_id not in the schematic's symbol cache: " + id );

            item.libId = id;
            item.name = libId.GetLibItemName().wx_str();
        }

        if( aArgs.contains( "name" ) )
            item.name = wxString::FromUTF8( aArgs.value( "name", std::string() ) );

        if( item.name.IsEmpty() )
            return KOPENAPI_RESULT::Error( 400, "'name' is empty" );

        items.push_back( item );
    }

    // Write through KiCad's own s-expression library IO: existing symbols of the file stay
    SCH_IO_KICAD_SEXPR io;
    const bool         created = !wxFileName::FileExists( path );
    nlohmann::json     saved = nlohmann::json::array();
    const int          compareFlags = ~( SCH_ITEM::COMPARE_FLAGS::UUID | SCH_ITEM::COMPARE_FLAGS::IDENTITY );

    try
    {
        if( created )
        {
            if( !wxFileName::DirExists( wxFileName( path ).GetPath() ) )
                wxFileName::Mkdir( wxFileName( path ).GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL );

            if( !items.empty() )
                io.CreateLibrary( path );
        }
        else
        {
            wxArrayString names;
            io.EnumerateSymbolLib( names, path );
        }
    }
    catch( const IO_ERROR& e )
    {
        return KOPENAPI_RESULT::Error( created ? 500 : 422, ( created ? "cannot create " : "not a symbol library: " ) + str( path ) + ": " + str( e.What() ) );
    }

    for( const SAVE_ITEM& item : items )
    {
        std::unique_ptr<LIB_SYMBOL> copy = item.symbol->Flatten();
        LIB_ID                      id;

        id.SetLibItemName( item.name );
        copy->SetName( item.name );
        copy->SetLibId( id );

        const LIB_SYMBOL* existing = created ? nullptr : io.LoadSymbol( path, item.name );
        const char*       status = "added";

        if( existing )
        {
            std::unique_ptr<LIB_SYMBOL> flatExisting = existing->Flatten();

            if( flatExisting->Compare( *copy, compareFlags ) == 0 )
            {
                saved.push_back( { { "name", str( item.name ) }, { "lib_id", item.libId }, { "status", "kept" } } );
                continue;
            }

            if( !overwrite )
            {
                const std::string why = "library already has a different symbol '" + str( item.name ) + "' (overwrite: true replaces it)";

                if( !allMissing )
                    return KOPENAPI_RESULT::Error( 409, why );

                skipped.push_back( { { "lib_id", item.libId }, { "reason", why } } );
                continue;
            }

            status = "replaced";
        }

        try
        {
            io.SaveSymbol( path, std::move( copy ) );
        }
        catch( const IO_ERROR& e )
        {
            return KOPENAPI_RESULT::Error( 500, "cannot save " + str( item.name ) + " to " + str( path ) + ": " + str( e.What() ) );
        }

        saved.push_back( { { "name", str( item.name ) }, { "lib_id", item.libId }, { "status", status } } );
    }

    // The library manager caches the library: reload it so lookups see the new symbols at once
    if( row )
    {
        adapter->ReloadLibraryEntry( row->Nickname(), row->Scope() );
        adapter->LoadLibraryEntry( row->Nickname() );
    }

    nlohmann::json result = { { "library", { { "nickname", row ? nlohmann::json( str( row->Nickname() ) ) : nlohmann::json() },
                                             { "path", str( path ) },
                                             { "created", created && !items.empty() } } },
                              { "saved", saved },
                              { "skipped", skipped } };

    if( allMissing )
        result["in_library"] = inLibrary;

    if( !row )
        result["note"] = "the file is in no sym-lib-table: add it to a sym-lib-table (the project one) to use its symbols by lib_id";

    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "sch_lib_list",
                   "List symbol libraries from KiCad's global and project sym-lib-tables: name, scope, "
                   "type, uri, status (loaded / error with message / disabled), symbol count; errors "
                   "counted; works without an open document; paginated",
                   KopenapiPagedSchema( R"json({"name":{"type":"string","description":"glob on the library name"}})json"_json ),
                   false, h_sch_lib_list, 600 );

KOPENAPI_REGISTER( "sch_lib_symbol_search",
                   "Search library symbols (parts) by words in name, keywords and description, ranked "
                   "(e.g. 'ldo 3.3', 'usb uart', 'opamp dual'; a 74-series number finds every logic family of it: "
                   "74LVC257 -> 74LS257); filter by library glob, power symbols, "
                   "pin count; returns lib_id, description, pins, units, default footprint, footprint "
                   "filters, used_in_design; paginated",
                   KopenapiPagedSchema( R"json({
                        "query":{"type":"string","description":"words; items matching all of them, else best partial matches (partial, unmatched_terms)"},
                        "lib":{"type":"string","description":"glob on the library name"},
                        "power":{"type":"boolean","default":false,"description":"power symbols only"},
                        "min_pins":{"type":"integer"},"max_pins":{"type":"integer"}})json"_json, KOPENAPI_RANKED_PAGE ),
                   false, h_sch_lib_symbol_search, 600 );

KOPENAPI_REGISTER( "sch_lib_symbol_get",
                   "Library symbol card by lib_id (LIBRARY:SYMBOL): fields, every pin (number, name, "
                   "electrical type, unit, hidden, side of the body, position from the origin in mm — to plan "
                   "a placement), units with names, body styles, default footprint "
                   "and footprint filters, datasheet, used_in_design",
                   R"json({"type":"object","required":["lib_id"],"properties":{
                        "lib_id":{"type":"string","description":"e.g. Regulator_Linear:AMS1117-3.3"}}})json"_json,
                   false, h_sch_lib_symbol_get, 600 );

KOPENAPI_REGISTER( "sch_lib_symbol_save",
                   "Save library symbols cached in the open schematic (lib_symbols) into a symbol library "
                   "(the API counterpart of File > Export > Symbols to Library): one by lib_id, the one of a "
                   "placed symbol by ref, or all_missing (every placed symbol whose library lacks it, power "
                   "symbols included). The library is a sym-lib-table nickname or a .kicad_sym path (created "
                   "if missing); its other symbols stay; it is reloaded at once, so sch_symbol_add / sch_connect "
                   "power_lib_id / sch_lib_symbol_search see the symbols. An identical symbol of that name is "
                   "kept; a different one is a 409 unless overwrite (all_missing: listed in skipped)",
                   R"json({"type":"object","required":["library"],"properties":{
                        "library":{"type":"string","description":"sym-lib-table nickname, or a .kicad_sym path (relative: to the project)"},
                        "lib_id":{"type":"string","description":"a symbol of the schematic's cache, e.g. 'PiGUS library:AGND'"},
                        "ref":{"type":"string","description":"a placed symbol: its cached library symbol"},
                        "all_missing":{"type":"boolean","default":false,"description":"every cached symbol whose own library lacks it"},
                        "name":{"type":"string","description":"symbol name in the library; default the lib_id's item name (lib_id / ref only)"},
                        "overwrite":{"type":"boolean","default":false,"description":"replace a different symbol of that name"}}})json"_json,
                   false, h_sch_lib_symbol_save, 600 );
