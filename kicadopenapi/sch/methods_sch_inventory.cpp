/*
 * kicadopenapi schematic statistics, inventories and net cards (DESIGN-ANALYSIS-API.md):
 * sch_stats, sch_sheet_list, sch_symbol_list, sch_net_list, sch_net_get.
 *
 * Symbols and nets are reported per sheet *instance* (a reused sheet yields distinct
 * references and net members per instance), using the same connection graph KiCad's
 * netlist exporter uses.  Drawing items (wires, labels, ...) are counted per file.
 */
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <advanced_config.h>
#include <api/sch_context.h>
#include <connectivity/conn_facade.h>
#include <connection_graph.h>
#include <kicadopenapi_util.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <schematic.h>

#include <algorithm>
#include <map>
#include <set>


using namespace kopenapi_sch;


static KOPENAPI_RESULT h_sch_stats( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC*           schematic = context->GetSchematic();
    const SCH_SHEET_LIST hierarchy = schematic->Hierarchy();

    // ---- drawing items, per file: unique screens of the real hierarchy (the virtual root
    // that holds top-level sheets has no content of its own) ----
    std::map<std::string, int> items;
    std::vector<SCH_SCREEN*>   screens;

    for( const SCH_SHEET_PATH& path : hierarchy )
    {
        if( std::find( screens.begin(), screens.end(), path.LastScreen() ) == screens.end() )
            screens.push_back( path.LastScreen() );
    }

    for( SCH_SCREEN* screen : screens )
    {
        for( SCH_ITEM* item : screen->Items() )
        {
            switch( item->Type() )
            {
            case SCH_LINE_T:
            {
                SCH_LINE* line = static_cast<SCH_LINE*>( item );
                items[line->IsWire() ? "wires" : line->IsBus() ? "buses" : "graphic_lines"]++;
                break;
            }
            case SCH_BUS_WIRE_ENTRY_T: items["bus_wire_entries"]++; break;
            case SCH_BUS_BUS_ENTRY_T:  items["bus_bus_entries"]++;  break;
            case SCH_JUNCTION_T:       items["junctions"]++;        break;
            case SCH_NO_CONNECT_T:     items["no_connects"]++;      break;
            case SCH_TEXT_T:           items["texts"]++;            break;
            case SCH_TEXTBOX_T:        items["text_boxes"]++;       break;
            case SCH_TABLE_T:          items["tables"]++;           break;
            case SCH_BITMAP_T:         items["images"]++;           break;
            case SCH_SHAPE_T:          items["shapes"]++;           break;
            case SCH_SHEET_T:
                items["sheet_symbols"]++;
                items["sheet_pins"] += static_cast<int>( static_cast<SCH_SHEET*>( item )->GetPins().size() );
                break;
            default:
                if( const char* kind = labelKind( item->Type() ) )
                    items[std::string( "labels_" ) + kind]++;
                break;
            }
        }
    }

    // ---- symbols, per instance ----
    int                        instances = 0, power = 0, unannotated = 0, dnp = 0, notInBom = 0, notOnBoard = 0;
    int                        multiUnit = 0, pinsTotal = 0;
    std::set<std::string>      components;
    std::map<std::string, int> libraries;
    size_t                     depth = 0;

    for( const SCH_SHEET_PATH& path : hierarchy )
    {
        depth = std::max( depth, path.size() );

        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* sym = static_cast<SCH_SYMBOL*>( item );
            instances++;

            if( sym->IsPower() )
            {
                power++;
                continue;
            }

            const std::string ref = str( sym->GetRef( &path, false ) );
            components.insert( ref );
            unannotated += ( !ref.empty() && ref.back() == '?' ) ? 1 : 0;
            dnp += sym->GetDNP( &path ) ? 1 : 0;
            notInBom += sym->GetExcludedFromBOM( &path ) ? 1 : 0;
            notOnBoard += sym->GetExcludedFromBoard( &path ) ? 1 : 0;
            multiUnit += sym->IsMultiUnit() ? 1 : 0;
            pinsTotal += static_cast<int>( sym->GetPins( &path ).size() );
            libraries[str( sym->GetLibId().GetLibNickname() )]++;
        }
    }

    // ---- nets, from the connection graph ----
    int netsTotal = 0, withPins = 0, singlePin = 0, powerNets = 0, noPins = 0;

    for( const NET_ENTRY& net : collectNets( schematic ) )
    {
        netsTotal++;
        auto pins = netPins( net );
        int  realPins = 0;
        bool isPower = false;

        for( const auto& [pin, path] : pins )
        {
            SCH_SYMBOL* sym = pinSymbol( pin );

            if( sym && sym->IsPower() )
                isPower = true;
            else
                realPins++;
        }

        if( pins.empty() )
            noPins++;
        else
            withPins++;

        singlePin += realPins == 1 ? 1 : 0;
        powerNets += isPower ? 1 : 0;
    }

    nlohmann::json libs = nlohmann::json::object();

    for( const auto& [lib, count] : libraries )
        libs[lib.empty() ? "(none)" : lib] = count;

    return KOPENAPI_RESULT::Ok(
            { { "document", str( context->GetCurrentFileName() ) },
              { "sheets",
                { { "files", screens.size() },
                  { "instances", hierarchy.size() },
                  { "top_level", schematic->GetTopLevelSheets().size() },
                  { "depth", depth } } },
              { "symbols",
                { { "instances", instances },
                  { "power", power },
                  { "components", components.size() },
                  { "pins", pinsTotal },
                  { "multi_unit", multiUnit },
                  { "unannotated", unannotated },
                  { "dnp", dnp },
                  { "excluded_from_bom", notInBom },
                  { "excluded_from_board", notOnBoard },
                  { "by_library", libs } } },
              { "items_per_file", items },
              { "nets",
                { { "total", netsTotal },
                  { "with_pins", withPins },
                  { "without_pins", noPins },
                  { "single_pin", singlePin },
                  { "power", powerNets } } } } );
}


static KOPENAPI_RESULT h_sch_sheet_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string glob = aArgs.value( "path", std::string() );
    std::vector<nlohmann::json> rows;

    for( const SCH_SHEET_PATH& path : context->GetSchematic()->Hierarchy() )
    {
        const std::string human = sheetPath( path );

        if( !KopenapiGlob( glob, human ) )
            continue;

        int symbols = 0;

        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
            symbols += static_cast<SCH_SYMBOL*>( item )->IsPower() ? 0 : 1;

        nlohmann::json ports = nlohmann::json::array();
        const SCH_SHEET* sheet = path.Last();

        if( path.size() > 1 )
        {
            for( const SCH_SHEET_PIN* pin : sheet->GetPins() )
                ports.push_back( { { "name", str( pin->GetText() ) }, { "direction", portDirection( pin ) } } );
        }

        rows.push_back( { { "path", human },
                          { "name", path.size() > 1 ? str( sheet->GetName() ) : std::string( "/" ) },
                          { "file", str( path.LastScreen()->GetFileName() ) },
                          { "page", str( path.GetPageNumber() ) },
                          { "symbols", symbols },
                          { "ports", ports },
                          { "uuid_path", str( path.Path().AsString() ) } } );
    }

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


static KOPENAPI_RESULT h_sch_symbol_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string refGlob = aArgs.value( "ref", std::string() );
    const std::string valueGlob = aArgs.value( "value", std::string() );
    const std::string libGlob = aArgs.value( "lib_id", std::string() );
    const std::string sheetGlob = aArgs.value( "sheet", std::string() );
    const bool        includePower = aArgs.value( "include_power", false );

    std::vector<nlohmann::json> rows;

    for( const SCH_SHEET_PATH& path : context->GetSchematic()->Hierarchy() )
    {
        const std::string sheet = sheetPath( path );

        if( !KopenapiGlob( sheetGlob, sheet ) )
            continue;

        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* sym = static_cast<SCH_SYMBOL*>( item );

            if( sym->IsPower() && !includePower )
                continue;

            nlohmann::json row = { { "ref", str( sym->GetRef( &path, false ) ) },
                                   { "value", str( sym->GetValue( &path, FOR_GUI ) ) },
                                   { "lib_id", str( sym->GetLibId().Format() ) },
                                   { "footprint", str( sym->GetFootprintFieldText( &path, FOR_GUI ) ) },
                                   { "unit", sym->GetUnit() },
                                   { "units", sym->GetUnitCount() },
                                   { "sheet", sheet },
                                   { "power", sym->IsPower() },
                                   { "dnp", sym->GetDNP( &path ) },
                                   { "in_bom", !sym->GetExcludedFromBOM( &path ) },
                                   { "on_board", !sym->GetExcludedFromBoard( &path ) },
                                   { "uuid", str( sym->m_Uuid.AsString() ) } };

            if( !KopenapiGlob( refGlob, row["ref"] ) || !KopenapiGlob( valueGlob, row["value"] )
                || !KopenapiGlob( libGlob, row["lib_id"] ) )
            {
                continue;
            }

            rows.push_back( std::move( row ) );
        }
    }

    std::sort( rows.begin(), rows.end(),
               []( const nlohmann::json& a, const nlohmann::json& b )
               {
                   if( a["ref"] != b["ref"] )
                       return KopenapiNaturalLess( a["ref"], b["ref"] );

                   return a["unit"].get<int>() < b["unit"].get<int>();
               } );

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


/// Summary of one net for lists
static nlohmann::json netRow( const NET_ENTRY& aNet )
{
    int                   pins = 0, labels = 0;
    bool                  power = false;
    std::set<std::string> sheets;

    for( const auto& [pin, path] : netPins( aNet ) )
    {
        SCH_SYMBOL* sym = pinSymbol( pin );

        if( sym && sym->IsPower() )
            power = true;
        else
            pins++;
    }

    for( const NET_INSTANCE& inst : aNet.instances )
    {
        sheets.insert( sheetPath( inst.path ) );

        for( SCH_ITEM* item : inst.items )
            labels += labelKind( item->Type() ) ? 1 : 0;
    }

    return { { "name", aNet.name },
             { "code", aNet.code },
             { "pins", pins },
             { "sheets", sheets.size() },
             { "labels", labels },
             { "power", power } };
}


static KOPENAPI_RESULT h_sch_net_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string glob = aArgs.value( "name", std::string() );
    const int         minPins = aArgs.value( "min_pins", 0 );
    const bool        hasPower = aArgs.contains( "power" ) && aArgs["power"].is_boolean();

    std::vector<nlohmann::json> rows;

    for( const NET_ENTRY& net : collectNets( context->GetSchematic() ) )
    {
        nlohmann::json row = netRow( net );

        if( !KopenapiGlob( glob, row["name"] ) || row["pins"].get<int>() < minPins
            || ( hasPower && row["power"] != aArgs["power"] ) )
        {
            continue;
        }

        rows.push_back( std::move( row ) );
    }

    std::sort( rows.begin(), rows.end(),
               []( const nlohmann::json& a, const nlohmann::json& b )
               { return KopenapiNaturalLess( a["name"], b["name"] ); } );

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


static KOPENAPI_RESULT h_sch_net_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::vector<NET_ENTRY> nets = collectNets( context->GetSchematic() );
    const NET_ENTRY*             found = nullptr;

    const std::string name = aArgs.value( "name", std::string() );
    const int         code = aArgs.value( "code", -1 );
    const std::string pin = aArgs.value( "pin", std::string() );

    if( name.empty() && code < 0 && pin.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'name', 'code' or 'pin' (REF.PIN)" );

    for( const NET_ENTRY& entry : nets )
    {
        if( ( !name.empty() && entry.name == name ) || ( code >= 0 && entry.code == code ) )
        {
            found = &entry;
            break;
        }

        if( !pin.empty() )
        {
            for( const auto& [p, path] : netPins( entry ) )
            {
                SCH_SYMBOL* sym = pinSymbol( p );

                if( sym && str( sym->GetRef( &path, false ) ) + "." + str( p->GetNumber() ) == pin )
                {
                    found = &entry;
                    break;
                }
            }

            if( found )
                break;
        }
    }

    if( !found )
        return KOPENAPI_RESULT::Error( 404, "net not found" );

    nlohmann::json        sheets = nlohmann::json::array();
    nlohmann::json        allPins = nlohmann::json::array();
    std::set<std::string> names = { found->name };
    std::map<std::string, nlohmann::json> byType;
    bool                  powerSymbol = false;
    std::set<std::string> seenPins;

    for( const NET_INSTANCE& inst : found->instances )
    {
        const SCH_SHEET_PATH& path = inst.path;
        nlohmann::json        pins = nlohmann::json::array();
        nlohmann::json        labels = nlohmann::json::array();
        nlohmann::json        ports = nlohmann::json::array();
        int                   wires = 0, junctions = 0;

        for( SCH_ITEM* item : inst.items )
        {
            if( item->Type() == SCH_PIN_T )
            {
                // Shared pins of multi-unit symbols appear once per unit: list them once
                if( !seenPins.insert( pinId( static_cast<SCH_PIN*>( item ), path ) ).second )
                    continue;

                nlohmann::json p = pinJson( static_cast<SCH_PIN*>( item ), path );
                powerSymbol |= p["power_symbol"].get<bool>();

                if( !p["power_symbol"].get<bool>() )
                {
                    byType[p["type"].get<std::string>()].push_back( p["pin"] );
                    allPins.push_back( p );
                }

                pins.push_back( p["pin"] );
            }
            else if( const char* kind = labelKind( item->Type() ) )
            {
                const std::string text = str( static_cast<SCH_LABEL_BASE*>( item )->GetText() );
                labels.push_back( { { "kind", kind }, { "text", text } } );
                names.insert( text );
            }
            else if( item->Type() == SCH_SHEET_PIN_T )
            {
                SCH_SHEET_PIN* sp = static_cast<SCH_SHEET_PIN*>( item );
                ports.push_back( { { "sheet", str( sp->GetParent() ? static_cast<SCH_SHEET*>( sp->GetParent() )->GetName() : wxString() ) },
                                   { "port", str( sp->GetText() ) } } );
            }
            else if( item->Type() == SCH_LINE_T )
            {
                wires++;
            }
            else if( item->Type() == SCH_JUNCTION_T )
            {
                junctions++;
            }
        }

        if( !inst.localName.empty() )
            names.insert( inst.localName );

        sheets.push_back( { { "sheet", sheetPath( path ) },
                            { "local_name", inst.localName },
                            { "pins", pins },
                            { "labels", labels },
                            { "sheet_ports", ports },
                            { "wires", wires },
                            { "junctions", junctions } } );
    }

    std::sort( allPins.begin(), allPins.end(),
               []( const nlohmann::json& a, const nlohmann::json& b )
               { return KopenapiNaturalLess( a["pin"], b["pin"] ); } );

    // Role: name heuristics + facts from the graph
    nlohmann::json inferred = KopenapiNetRoleFromName( found->name );

    if( powerSymbol )
    {
        if( inferred["role"] == "signal" )
            inferred["role"] = "power";

        inferred["basis"].push_back( "driven by a power symbol" );
    }

    if( byType.count( "Power output" ) && inferred["role"] == "signal" )
    {
        inferred["role"] = "power";
        inferred["basis"].push_back( "has power output pins" );
    }

    // Drivers vs loads by electrical type
    nlohmann::json drivers = nlohmann::json::array(), loads = nlohmann::json::array(), passive = nlohmann::json::array();

    for( const auto& [type, list] : byType )
    {
        for( const nlohmann::json& p : list )
        {
            if( type == "Output" || type == "Power output" || type == "Open collector" || type == "Open emitter"
                || type == "Bidirectional" || type == "Tri-state" )
                drivers.push_back( p );
            else if( type == "Input" || type == "Power input" )
                loads.push_back( p );
            else
                passive.push_back( p );
        }
    }

    nlohmann::json nameList = nlohmann::json::array();

    for( const std::string& n : names )
    {
        if( !n.empty() )
            nameList.push_back( n );
    }

    return KOPENAPI_RESULT::Ok( { { "name", found->name },
                                  { "code", found->code },
                                  { "names", nameList },
                                  { "pins", allPins },
                                  { "drivers", drivers },
                                  { "loads", loads },
                                  { "other_pins", passive },
                                  { "sheets", sheets },
                                  { "inferred", inferred } } );
}


KOPENAPI_REGISTER( "sch_stats",
                   "Schematic statistics in one call: sheets (files, instances, depth), symbols (instances, "
                   "components, power, pins, DNP, unannotated, by library), wires, buses, bus entries, "
                   "junctions, labels/ports per kind, nets (total, with pins, single-pin, power)",
                   R"json({"type":"object","properties":{}})json"_json, false, h_sch_stats );

KOPENAPI_REGISTER( "sch_sheet_list",
                   "List sheet instances of the hierarchy: path, name, file, page, symbol count, ports "
                   "(sheet pins with direction); filter by path glob; paginated",
                   KopenapiPagedSchema( R"json({"path":{"type":"string","description":"glob on the human path, e.g. /CPU*"}})json"_json ),
                   false, h_sch_sheet_list );

KOPENAPI_REGISTER( "sch_symbol_list",
                   "List schematic symbols per sheet instance: ref, value, lib_id, footprint, unit, sheet, "
                   "DNP/BOM/board flags; filter by ref/value/lib_id/sheet glob, include_power; paginated",
                   KopenapiPagedSchema( R"json({
                        "ref":{"type":"string"},"value":{"type":"string"},"lib_id":{"type":"string"},
                        "sheet":{"type":"string","description":"glob on the sheet path"},
                        "include_power":{"type":"boolean","default":false}})json"_json ),
                   false, h_sch_symbol_list );

KOPENAPI_REGISTER( "sch_net_list",
                   "List schematic nets: name, code, pin count (without power symbols), sheets spanned, "
                   "labels, power flag; filter by name glob, min_pins, power; paginated",
                   KopenapiPagedSchema( R"json({
                        "name":{"type":"string","description":"glob, e.g. /CPU/* or *CLK*"},
                        "min_pins":{"type":"integer","default":0},
                        "power":{"type":"boolean"}})json"_json ),
                   false, h_sch_net_list );

KOPENAPI_REGISTER( "sch_net_get",
                   "Schematic net card: every pin (REF.PIN, pin name, electrical type, value, sheet), drivers "
                   "vs loads, all names along the hierarchy, per sheet instance the labels, sheet ports, wires "
                   "and junctions, inferred role with basis; look up by name, code or pin REF.PIN",
                   R"json({"type":"object","properties":{
                        "name":{"type":"string"},"code":{"type":"integer"},
                        "pin":{"type":"string","description":"REF.PIN, e.g. U3.14"}}})json"_json,
                   false, h_sch_net_get );
