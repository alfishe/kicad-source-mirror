/// @file methods_sch_expect.cpp
/// @brief kicadopenapi schematic intent check: sch_nets_expect.
///
/// The nets a design means ({net: [REF.PIN, ...]}) against the schematic's connectivity: a net split
/// over several nets, joined with pins of the listed parts that no listed net names, pins that are
/// on no net or do not exist.  The check a drawing session runs after every batch of edits, so a
/// move, a delete or two touching pins cannot change the circuit unnoticed.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kicadopenapi_util.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>

#include <algorithm>
#include <map>
#include <set>

using namespace kopenapi_sch;


static KOPENAPI_RESULT h_sch_nets_expect( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const nlohmann::json expected = aArgs.value( "nets", nlohmann::json::object() );

    if( !expected.is_object() || expected.empty() )
        return KOPENAPI_RESULT::Error( 400, "nets: an object {net name: [REF.PIN, ...]} with at least one net" );

    SCHEMATIC* schematic = context->GetSchematic();

    // every pin of the design, and the parts the expected nets name
    std::set<std::string> known;

    for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( symbol->IsPower() )
                continue;

            for( SCH_PIN* pin : symbol->GetPins( &path ) )
                known.insert( pinId( pin, path ) );
        }
    }

    std::map<std::string, std::string> netOf;   // REF.PIN -> net name
    std::map<std::string, int>         netSize;

    for( const NET_ENTRY& net : collectNets( schematic ) )
    {
        for( const auto& [pin, path] : netPins( net ) )
        {
            SCH_SYMBOL* symbol = pinSymbol( pin );

            if( symbol && !symbol->IsPower() )
            {
                netOf[pinId( pin, path )] = net.name;
                netSize[net.name]++;
            }
        }
    }

    std::set<std::string> listed, parts;

    for( const auto& [name, pins] : expected.items() )
    {
        if( !pins.is_array() )
            return KOPENAPI_RESULT::Error( 400, "nets." + name + ": an array of REF.PIN" );

        for( const nlohmann::json& p : pins )
        {
            const std::string id = p.get<std::string>();
            listed.insert( id );
            parts.insert( id.substr( 0, id.find( '.' ) ) );
        }
    }

    // the real nets that hold pins of the listed parts, with those pins
    std::map<std::string, std::set<std::string>> partPinsOn;

    for( const auto& [id, net] : netOf )
    {
        if( parts.count( id.substr( 0, id.find( '.' ) ) ) )
            partPinsOn[net].insert( id );
    }

    nlohmann::json problems = nlohmann::json::array();
    nlohmann::json matched = nlohmann::json::object();

    for( const auto& [name, pins] : expected.items() )
    {
        std::vector<std::string> unknown, unconnected;
        std::map<std::string, std::vector<std::string>> byNet;

        for( const nlohmann::json& p : pins )
        {
            const std::string id = p.get<std::string>();

            if( !known.count( id ) )
                unknown.push_back( id );
            else if( auto it = netOf.find( id ); it == netOf.end() || ( pins.size() > 1 && netSize[it->second] == 1 ) )
                unconnected.push_back( id );
            else
                byNet[it->second].push_back( id );
        }

        if( !unknown.empty() )
            problems.push_back( { { "net", name }, { "kind", "unknown_pins" }, { "pins", unknown } } );

        if( !unconnected.empty() )
            problems.push_back( { { "net", name }, { "kind", "unconnected" }, { "pins", unconnected } } );

        if( byNet.size() > 1 )
        {
            nlohmann::json pieces = nlohmann::json::object();

            for( const auto& [net, ids] : byNet )
                pieces[net] = ids;

            problems.push_back( { { "net", name }, { "kind", "split" }, { "nets", pieces } } );
        }
        else if( byNet.size() == 1 )
        {
            const std::string& real = byNet.begin()->first;
            matched[name] = real;
            std::vector<std::string> stray;

            for( const std::string& id : partPinsOn[real] )
            {
                if( std::find( pins.begin(), pins.end(), id ) == pins.end() )
                    stray.push_back( id );
            }

            if( !stray.empty() )
                problems.push_back( { { "net", name }, { "kind", "joined" }, { "real_net", real }, { "extra_pins", stray } } );
        }
    }

    // two expected nets that ended on one real net
    std::map<std::string, std::vector<std::string>> sharing;

    for( const auto& [name, real] : matched.items() )
        sharing[real.get<std::string>()].push_back( name );

    for( const auto& [real, names] : sharing )
    {
        if( names.size() > 1 )
            problems.push_back( { { "net", names.front() }, { "kind", "merged" }, { "real_net", real }, { "expected_nets", names } } );
    }

    return KOPENAPI_RESULT::Ok( { { "ok", problems.empty() },
                                  { "checked_nets", expected.size() },
                                  { "checked_pins", listed.size() },
                                  { "problems", problems },
                                  { "nets", matched } } );
}


KOPENAPI_REGISTER( "sch_nets_expect",
                   "Check the schematic against the nets meant ({net: [REF.PIN, ...]}, any sheets): per net "
                   "split (its pins on several nets), joined (its net also holds other pins of the listed "
                   "parts that this net does not list), merged (two listed nets are one), unconnected (a pin "
                   "on no net), unknown_pins (no such pin); ok flag and the real net name of each. Run it "
                   "after every batch of drawing edits: moves, deletes and touching pins change circuits "
                   "silently",
                   R"json({"type":"object","required":["nets"],"properties":{
                       "nets":{"type":"object","description":"net name -> array of REF.PIN","additionalProperties":{"type":"array","items":{"type":"string"}}}}})json"_json,
                   false, h_sch_nets_expect );
