/*
 * kicadopenapi schematic traversal (DESIGN-ANALYSIS-API.md §5):
 * sch_net_trace, sch_symbol_neighbors, sch_path_find, sch_power_tree, sch_interface_map,
 * sch_net_search, sch_subcircuit.
 *
 * All methods work on one connectivity graph built per call: components (by reference,
 * units merged) connected through nets via their pins — the same nets and pins as
 * sch_net_get / the netlist exporter.
 */
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kicadopenapi_util.h>
#include <sch_label.h>
#include <sch_pin.h>
#include <sch_sheet.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <string_utils.h>

#include <algorithm>
#include <deque>
#include <map>
#include <set>

using namespace kopenapi_sch;


namespace
{

struct PIN_NODE
{
    std::string id;       ///< REF.PIN
    std::string ref;
    std::string number;
    std::string name;
    std::string type;     ///< electrical type name
    int         net = -1; ///< index into GRAPH::nets
};


struct COMPONENT
{
    std::string      ref;
    std::string      value;
    std::string      libId;
    std::vector<int> pins;      ///< indices into GRAPH::pins
};


struct GRAPH
{
    std::vector<NET_ENTRY>             nets;
    std::vector<std::vector<int>>      netPins;      ///< per net: pin indices (power symbols excluded)
    std::vector<bool>                  netIsPower;   ///< power/ground by name or power symbol
    std::vector<std::string>           netRole;
    std::vector<PIN_NODE>              pins;
    std::map<std::string, COMPONENT>   components;
    std::map<std::string, int>         pinIndex;     ///< REF.PIN -> pin
    std::map<std::string, int>         netIndex;     ///< name -> net
};


GRAPH buildGraph( SCHEMATIC* aSchematic )
{
    GRAPH g;
    g.nets = collectNets( aSchematic );
    g.netPins.resize( g.nets.size() );
    g.netIsPower.resize( g.nets.size(), false );
    g.netRole.resize( g.nets.size() );

    for( size_t n = 0; n < g.nets.size(); ++n )
    {
        g.netIndex[g.nets[n].name] = static_cast<int>( n );
        g.netRole[n] = KopenapiNetRoleFromName( g.nets[n].name )["role"];
        g.netIsPower[n] = g.netRole[n] == "power" || g.netRole[n] == "ground";

        for( const auto& [pin, path] : netPins( g.nets[n] ) )
        {
            SCH_SYMBOL* sym = pinSymbol( pin );

            if( !sym || sym->IsPower() )
            {
                g.netIsPower[n] = g.netIsPower[n] || isRailSymbol( sym );
                continue;
            }

            PIN_NODE node;
            node.ref = str( sym->GetRef( &path, false ) );
            node.number = str( pin->GetNumber() );
            node.id = node.ref + "." + node.number;
            node.name = str( pin->GetShownName() );
            node.type = str( pin->GetElectricalTypeName() );
            node.net = static_cast<int>( n );

            const int index = static_cast<int>( g.pins.size() );
            g.pins.push_back( node );
            g.pinIndex[node.id] = index;
            g.netPins[n].push_back( index );

            COMPONENT& comp = g.components[node.ref];

            if( comp.ref.empty() )
            {
                comp.ref = node.ref;
                comp.value = str( sym->GetValue( &path, FOR_GUI ) );
                comp.libId = str( sym->GetLibId().Format() );
            }

            comp.pins.push_back( index );
        }
    }

    return g;
}


nlohmann::json pinBrief( const GRAPH& g, int aPin )
{
    const PIN_NODE& p = g.pins[aPin];
    return { { "pin", p.id }, { "name", p.name }, { "type", p.type } };
}


/// A net is not walked through when it is a power net (optional) or too big (fan-out)
bool blocked( const GRAPH& g, int aNet, bool aSkipPower, int aMaxFanout )
{
    if( aSkipPower && g.netIsPower[aNet] )
        return true;

    return aMaxFanout > 0 && static_cast<int>( g.netPins[aNet].size() ) > aMaxFanout;
}


/// Two-terminal parts (R, C, L, diodes, ferrites...) are what signals pass "through"
bool isPassive( const COMPONENT& aComp )
{
    return aComp.pins.size() <= 2;
}


/// Distance between two sheet instances in the hierarchy tree
size_t treeDistance( const SCH_SHEET_PATH& aA, const SCH_SHEET_PATH& aB )
{
    size_t common = 0;

    while( common < aA.size() && common < aB.size() && aA.at( common ) == aB.at( common ) )
        ++common;

    return ( aA.size() - common ) + ( aB.size() - common );
}

} // namespace


static KOPENAPI_RESULT h_sch_net_trace( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::vector<NET_ENTRY> nets = collectNets( context->GetSchematic() );
    const std::string            pin = aArgs.value( "pin", std::string() );
    const std::string            name = aArgs.value( "net", std::string() );
    const NET_ENTRY*             net = nullptr;
    const SCH_SHEET_PATH*        start = nullptr;

    for( const NET_ENTRY& entry : nets )
    {
        if( !name.empty() && ( entry.name == name || entry.name == str( UnescapeString( name ) ) ) )
        {
            net = &entry;
            break;
        }

        for( const NET_INSTANCE& inst : entry.instances )
        {
            for( SCH_ITEM* item : inst.items )
            {
                if( !pin.empty() && item->Type() == SCH_PIN_T && pinId( static_cast<SCH_PIN*>( item ), inst.path ) == pin )
                {
                    net = &entry;
                    start = &inst.path;
                }
            }
        }

        if( net )
            break;
    }

    if( pin.empty() && name.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'pin' (REF.PIN) or 'net' (name)" );

    if( !net )
        return KOPENAPI_RESULT::Error( 404, "net or pin not found" );

    if( !start && !net->instances.empty() )
        start = &net->instances.front().path;

    // Order sheet instances by distance from the start sheet in the hierarchy tree
    std::vector<const NET_INSTANCE*> order;

    for( const NET_INSTANCE& inst : net->instances )
        order.push_back( &inst );

    std::stable_sort( order.begin(), order.end(),
                      [&]( const NET_INSTANCE* a, const NET_INSTANCE* b )
                      { return treeDistance( *start, a->path ) < treeDistance( *start, b->path ); } );

    nlohmann::json route = nlohmann::json::array();

    for( const NET_INSTANCE* inst : order )
    {
        nlohmann::json pins = nlohmann::json::array(), via = nlohmann::json::array();
        std::set<std::string> seen;

        for( SCH_ITEM* item : inst->items )
        {
            if( item->Type() == SCH_PIN_T )
            {
                SCH_SYMBOL* sym = pinSymbol( static_cast<SCH_PIN*>( item ) );
                const std::string id = pinId( static_cast<SCH_PIN*>( item ), inst->path );

                if( seen.insert( id ).second )
                    pins.push_back( sym && sym->IsPower() ? "power symbol " + str( sym->GetValue( &inst->path, FOR_GUI ) ) : id );
            }
            else if( const char* kind = labelKind( item->Type() ) )
            {
                via.push_back( { { "label", kind }, { "text", str( UnescapeString( static_cast<SCH_LABEL_BASE*>( item )->GetText() ) ) } } );
            }
            else if( item->Type() == SCH_SHEET_PIN_T )
            {
                SCH_SHEET_PIN* sp = static_cast<SCH_SHEET_PIN*>( item );
                via.push_back( { { "sheet_port", str( UnescapeString( sp->GetText() ) ) },
                                 { "into_sheet", str( static_cast<SCH_SHEET*>( sp->GetParent() )->GetName() ) },
                                 { "direction", portDirection( sp ) } } );
            }
        }

        route.push_back( { { "sheet", sheetPath( inst->path ) },
                           { "distance", treeDistance( *start, inst->path ) },
                           { "local_name", inst->localName },
                           { "via", via },
                           { "pins", pins } } );
    }

    return KOPENAPI_RESULT::Ok( { { "net", net->name }, { "from", pin.empty() ? nlohmann::json() : nlohmann::json( pin ) },
                                  { "start_sheet", sheetPath( *start ) }, { "route", route } } );
}


static KOPENAPI_RESULT h_sch_symbol_neighbors( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const GRAPH       g = buildGraph( context->GetSchematic() );
    const std::string ref = aArgs.value( "ref", std::string() );
    const int         depth = std::clamp( aArgs.value( "depth", 1 ), 1, 6 );
    const bool        skipPower = aArgs.value( "skip_power", true );
    const int         maxFanout = aArgs.value( "max_fanout", 30 );

    auto it = g.components.find( ref );

    if( it == g.components.end() )
        return KOPENAPI_RESULT::Error( 404, "component not found (or not connected): " + ref );

    std::map<std::string, int>        level = { { ref, 0 } };
    std::deque<std::string>           queue = { ref };
    nlohmann::json                    neighbors = nlohmann::json::array();
    std::set<std::string>             skippedNets;

    while( !queue.empty() )
    {
        const std::string current = queue.front();
        queue.pop_front();

        if( level[current] >= depth )
            continue;

        // links of `current` grouped by neighbour
        std::map<std::string, nlohmann::json> links;

        for( int pin : g.components.at( current ).pins )
        {
            const int net = g.pins[pin].net;

            if( blocked( g, net, skipPower, maxFanout ) )
            {
                skippedNets.insert( g.nets[net].name );
                continue;
            }

            for( int other : g.netPins[net] )
            {
                const std::string& otherRef = g.pins[other].ref;

                if( otherRef == current )
                    continue;

                links[otherRef].push_back( { { "net", g.nets[net].name },
                                             { "from_pin", g.pins[pin].id },
                                             { "to_pin", g.pins[other].id } } );
            }
        }

        for( auto& [otherRef, via] : links )
        {
            if( level.count( otherRef ) )
                continue;

            level[otherRef] = level[current] + 1;
            queue.push_back( otherRef );

            const COMPONENT& comp = g.components.at( otherRef );
            neighbors.push_back( { { "ref", otherRef },
                                   { "value", comp.value },
                                   { "lib_id", comp.libId },
                                   { "depth", level[otherRef] },
                                   { "reached_from", current },
                                   { "via", via } } );
        }
    }

    return KOPENAPI_RESULT::Ok( { { "ref", ref },
                                  { "value", it->second.value },
                                  { "neighbors", neighbors },
                                  { "skipped_nets", skippedNets } } );
}


static KOPENAPI_RESULT h_sch_path_find( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const GRAPH       g = buildGraph( context->GetSchematic() );
    const std::string from = aArgs.value( "from", std::string() );
    const std::string to = aArgs.value( "to", std::string() );
    const bool        skipPower = aArgs.value( "skip_power", true );
    const bool        anyPart = aArgs.value( "through", std::string( "passive" ) ) == "any";
    const int         maxHops = std::clamp( aArgs.value( "max_parts", 6 ), 1, 20 );

    // Endpoints: a pin (REF.PIN) or a whole component (REF)
    auto endpointPins = [&]( const std::string& aSpec ) -> std::vector<int>
    {
        if( auto p = g.pinIndex.find( aSpec ); p != g.pinIndex.end() )
            return { p->second };

        if( auto c = g.components.find( aSpec ); c != g.components.end() )
            return c->second.pins;

        return {};
    };

    const std::vector<int> sources = endpointPins( from );
    const std::vector<int> targets = endpointPins( to );

    if( sources.empty() || targets.empty() )
        return KOPENAPI_RESULT::Error( 404, "'from' / 'to' must be a connected REF.PIN or REF" );

    std::set<int> targetNets;

    for( int pin : targets )
        targetNets.insert( g.pins[pin].net );

    // A power net is walked only when an endpoint was given as a pin sitting on it
    std::set<int> endpointNets;

    for( const std::string& spec : { from, to } )
    {
        if( auto p = g.pinIndex.find( spec ); p != g.pinIndex.end() )
            endpointNets.insert( g.pins[p->second].net );
    }

    auto skipNet = [&]( int aNet ) { return skipPower && g.netIsPower[aNet] && !endpointNets.count( aNet ); };

    const std::string targetRef = g.pins[targets.front()].ref;

    // BFS over nets; a hop crosses one intermediate part (entering at one pin, leaving by another)
    struct STEP
    {
        int prevNet = -1;
        int inPin = -1;   ///< pin of the intermediate part on prevNet
        int outPin = -1;  ///< pin of the intermediate part on this net
        int parts = 0;
    };

    std::map<int, STEP> visited;
    std::deque<int>     queue;

    for( int pin : sources )
    {
        const int net = g.pins[pin].net;

        if( !visited.count( net ) && !skipNet( net ) )
        {
            visited[net] = STEP{ -1, -1, pin, 0 };
            queue.push_back( net );
        }
    }

    int found = -1;

    while( !queue.empty() && found < 0 )
    {
        const int net = queue.front();
        queue.pop_front();

        if( targetNets.count( net ) )
        {
            found = net;
            break;
        }

        if( visited[net].parts >= maxHops || skipNet( net ) )
            continue;

        for( int pin : g.netPins[net] )
        {
            const COMPONENT& part = g.components.at( g.pins[pin].ref );

            if( part.ref == targetRef || ( !anyPart && !isPassive( part ) ) )
                continue;

            for( int out : part.pins )
            {
                const int next = g.pins[out].net;

                if( out == pin || visited.count( next ) )
                    continue;

                if( skipNet( next ) )
                    continue;

                visited[next] = STEP{ net, pin, out, visited[net].parts + 1 };
                queue.push_back( next );
            }
        }
    }

    if( found < 0 )
        return KOPENAPI_RESULT::Ok( { { "found", false }, { "from", from }, { "to", to } } );

    // Walk back: net, part, net, part, ...
    nlohmann::json chain = nlohmann::json::array();

    for( int net = found; net >= 0; net = visited[net].prevNet )
    {
        chain.insert( chain.begin(), nlohmann::json::object( { { "net", g.nets[net].name } } ) );

        const STEP& step = visited[net];

        if( step.prevNet >= 0 )
        {
            const COMPONENT& part = g.components.at( g.pins[step.inPin].ref );
            chain.insert( chain.begin(), nlohmann::json::object( { { "part", part.ref },
                                                                   { "value", part.value },
                                                                   { "in_pin", g.pins[step.inPin].id },
                                                                   { "out_pin", g.pins[step.outPin].id } } ) );
        }
    }

    return KOPENAPI_RESULT::Ok( { { "found", true },
                                  { "from", from },
                                  { "to", to },
                                  { "parts_between", visited[found].parts },
                                  { "chain", chain } } );
}


static KOPENAPI_RESULT h_sch_power_tree( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const GRAPH g = buildGraph( context->GetSchematic() );
    const int   listLimit = std::clamp( aArgs.value( "list_limit", 50 ), 0, 1000 );

    nlohmann::json rails = nlohmann::json::array();

    for( size_t n = 0; n < g.nets.size(); ++n )
    {
        if( !g.netIsPower[n] )
            continue;

        nlohmann::json sources = nlohmann::json::array(), consumers = nlohmann::json::array(),
                       connectors = nlohmann::json::array(), other = nlohmann::json::array();
        int            consumerCount = 0, otherCount = 0;

        for( int pin : g.netPins[n] )
        {
            const PIN_NODE& p = g.pins[pin];

            if( p.type == "Power output" )
                sources.push_back( pinBrief( g, pin ) );
            else if( KopenapiIsConnectorRef( p.ref ) )
                connectors.push_back( pinBrief( g, pin ) );
            else if( p.type == "Power input" )
            {
                if( consumerCount++ < listLimit )
                    consumers.push_back( p.id );
            }
            else if( otherCount++ < listLimit )
            {
                other.push_back( p.id );
            }
        }

        rails.push_back( { { "net", g.nets[n].name },
                           { "role", g.netRole[n] == "signal" ? std::string( "power" ) : g.netRole[n] },
                           { "pins", g.netPins[n].size() },
                           { "sources", sources },
                           { "connectors", connectors },
                           { "consumers", consumers },
                           { "consumer_count", consumerCount },
                           { "other_pins", other },
                           { "other_count", otherCount } } );
    }

    return KOPENAPI_RESULT::Ok( { { "rails", rails } } );
}


static KOPENAPI_RESULT h_sch_interface_map( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const GRAPH       g = buildGraph( context->GetSchematic() );
    const std::string refGlob = aArgs.value( "ref", std::string() );
    const int         listLimit = std::clamp( aArgs.value( "list_limit", 10 ), 0, 1000 );

    std::vector<std::string> refs;

    for( const auto& [ref, comp] : g.components )
    {
        const bool connectorLib = comp.libId.find( "Conn" ) != std::string::npos;

        if( ( KopenapiIsConnectorRef( ref ) || connectorLib ) && KopenapiGlob( refGlob, ref ) )
            refs.push_back( ref );
    }

    std::sort( refs.begin(), refs.end(), KopenapiNaturalLess );

    nlohmann::json connectors = nlohmann::json::array();

    for( const std::string& ref : refs )
    {
        const COMPONENT& comp = g.components.at( ref );
        std::vector<int> pins = comp.pins;
        std::sort( pins.begin(), pins.end(),
                   [&]( int a, int b ) { return KopenapiNaturalLess( g.pins[a].number, g.pins[b].number ); } );

        nlohmann::json pinList = nlohmann::json::array();

        for( int pin : pins )
        {
            const int      net = g.pins[pin].net;
            nlohmann::json far = nlohmann::json::array();
            int            farCount = 0;

            for( int other : g.netPins[net] )
            {
                if( g.pins[other].ref != ref && farCount++ < listLimit )
                    far.push_back( g.pins[other].id );
            }

            pinList.push_back( { { "pin", g.pins[pin].number },
                                 { "name", g.pins[pin].name },
                                 { "net", g.nets[net].name },
                                 { "role", g.netRole[net] == "signal" && g.netIsPower[net] ? std::string( "power" ) : g.netRole[net] },
                                 { "far_end", far },
                                 { "far_end_count", farCount } } );
        }

        connectors.push_back( { { "ref", ref }, { "value", comp.value }, { "lib_id", comp.libId }, { "pins", pinList } } );
    }

    return KOPENAPI_RESULT::Ok( { { "connectors", connectors } } );
}


static KOPENAPI_RESULT h_sch_net_search( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string pattern = aArgs.value( "pattern", std::string() );
    const std::string by = aArgs.value( "by", std::string( "any" ) );
    const std::string roleFilter = aArgs.value( "role", std::string() );

    static const std::set<std::string> kinds = { "any", "name", "pin_name", "value", "ref" };

    if( !kinds.count( by ) )
        return KOPENAPI_RESULT::Error( 400, "by must be any, name, pin_name, value or ref" );

    if( pattern.empty() && roleFilter.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'pattern' (glob) and/or 'role'" );

    const GRAPH g = buildGraph( context->GetSchematic() );
    const bool  all = by == "any";
    const int   listLimit = std::clamp( aArgs.value( "list_limit", 10 ), 1, 1000 );

    std::vector<nlohmann::json> rows;

    for( size_t n = 0; n < g.nets.size(); ++n )
    {
        const NET_ENTRY& net = g.nets[n];

        if( !roleFilter.empty() && g.netRole[n] != roleFilter
            && !( roleFilter == "power" && g.netIsPower[n] && g.netRole[n] != "ground" ) )
        {
            continue;
        }

        nlohmann::json matched = nlohmann::json::array();
        int            matchCount = 0;
        auto           hit = [&]( const char* aWhat, const std::string& aText, const std::string& aWhere )
        {
            if( matchCount++ < listLimit )
                matched.push_back( { { "by", aWhat }, { "text", aText }, { "at", aWhere } } );
        };

        if( pattern.empty() )
        {
            hit( "role", g.netRole[n], net.name );
        }
        else
        {
            if( all || by == "name" )
            {
                std::set<std::string> names = { net.name };

                for( const NET_INSTANCE& inst : net.instances )
                {
                    if( !inst.localName.empty() )
                        names.insert( inst.localName );
                }

                for( const std::string& name : names )
                {
                    if( KopenapiGlob( pattern, name ) )
                        hit( "name", name, net.name );
                }
            }

            std::set<std::string> refsHit;

            for( int pin : g.netPins[n] )
            {
                const PIN_NODE&  p = g.pins[pin];
                const COMPONENT& comp = g.components.at( p.ref );

                if( ( all || by == "pin_name" ) && KopenapiGlob( pattern, p.name ) )
                    hit( "pin_name", p.name, p.id );

                if( by == "value" && KopenapiGlob( pattern, comp.value ) && refsHit.insert( p.ref ).second )
                    hit( "value", comp.value, p.ref );

                if( by == "ref" && KopenapiGlob( pattern, p.ref ) && refsHit.insert( p.ref ).second )
                    hit( "ref", p.ref, p.id );
            }
        }

        if( matchCount == 0 )
            continue;

        rows.push_back( { { "name", net.name },
                          { "code", net.code },
                          { "role", g.netRole[n] },
                          { "pins", g.netPins[n].size() },
                          { "matched", matched },
                          { "match_count", matchCount } } );
    }

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


static KOPENAPI_RESULT h_sch_subcircuit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const GRAPH       g = buildGraph( context->GetSchematic() );
    const std::string ref = aArgs.value( "ref", std::string() );
    const int         depth = std::clamp( aArgs.value( "depth", 2 ), 1, 6 );
    const int         maxPartPins = std::clamp( aArgs.value( "max_part_pins", 3 ), 2, 64 );
    const int         maxFanout = aArgs.value( "max_fanout", 30 );

    auto center = g.components.find( ref );

    if( center == g.components.end() )
        return KOPENAPI_RESULT::Error( 404, "component not found (or not connected): " + ref );

    // Members grow through small parts (<= max_part_pins: R, C, L, diodes, transistors);
    // connectors and bigger parts (other ICs) are the boundary: listed, not expanded.
    std::map<std::string, int>         level = { { ref, 0 } };
    std::map<std::string, std::string> kind = { { ref, "center" } };
    std::deque<std::string>            queue = { ref };
    std::set<int>                      rails, crowded;

    while( !queue.empty() )
    {
        const std::string current = queue.front();
        queue.pop_front();

        if( level[current] >= depth )
            continue;

        for( int pin : g.components.at( current ).pins )
        {
            const int net = g.pins[pin].net;

            if( g.netIsPower[net] )
            {
                rails.insert( net );
                continue;
            }

            if( maxFanout > 0 && static_cast<int>( g.netPins[net].size() ) > maxFanout )
            {
                crowded.insert( net );
                continue;
            }

            for( int other : g.netPins[net] )
            {
                const std::string& otherRef = g.pins[other].ref;

                if( level.count( otherRef ) )
                    continue;

                const COMPONENT& comp = g.components.at( otherRef );
                level[otherRef] = level[current] + 1;

                if( KopenapiIsConnectorRef( otherRef ) )
                {
                    kind[otherRef] = "boundary_connector";
                }
                else if( static_cast<int>( comp.pins.size() ) > maxPartPins )
                {
                    kind[otherRef] = "boundary_part";
                }
                else
                {
                    kind[otherRef] = "member";
                    queue.push_back( otherRef );
                }
            }
        }
    }

    // Nets of the block: internal when every pin belongs to the center or members
    nlohmann::json parts = nlohmann::json::array(), internal = nlohmann::json::array(),
                   external = nlohmann::json::array(), railList = nlohmann::json::array();
    std::set<int>  nets;

    std::vector<std::string> refs;

    for( const auto& [r, k] : kind )
        refs.push_back( r );

    std::sort( refs.begin(), refs.end(),
               [&]( const std::string& a, const std::string& b )
               {
                   if( level[a] != level[b] )
                       return level[a] < level[b];

                   return KopenapiNaturalLess( a, b );
               } );

    for( const std::string& r : refs )
    {
        const COMPONENT& comp = g.components.at( r );
        parts.push_back( { { "ref", r }, { "value", comp.value }, { "lib_id", comp.libId },
                           { "kind", kind[r] }, { "pins", comp.pins.size() }, { "distance", level[r] } } );

        if( kind[r] == "center" || kind[r] == "member" )
        {
            for( int pin : comp.pins )
            {
                if( !g.netIsPower[g.pins[pin].net] )
                    nets.insert( g.pins[pin].net );
            }
        }
    }

    for( int net : nets )
    {
        nlohmann::json outside = nlohmann::json::array();
        int            outsideCount = 0;

        for( int pin : g.netPins[net] )
        {
            auto k = kind.find( g.pins[pin].ref );

            if( k == kind.end() || ( k->second != "center" && k->second != "member" ) )
            {
                if( outsideCount++ < 10 )
                    outside.push_back( g.pins[pin].id );
            }
        }

        if( outsideCount == 0 )
            internal.push_back( g.nets[net].name );
        else
            external.push_back( { { "net", g.nets[net].name }, { "role", g.netRole[net] },
                                  { "outside_pins", outside }, { "outside_count", outsideCount },
                                  { "crowded", crowded.count( net ) > 0 } } );
    }

    for( int net : rails )
        railList.push_back( { { "net", g.nets[net].name }, { "role", g.netRole[net] } } );

    return KOPENAPI_RESULT::Ok( { { "ref", ref },
                                  { "value", center->second.value },
                                  { "parts", parts },
                                  { "internal_nets", internal },
                                  { "external_nets", external },
                                  { "rails", railList } } );
}


KOPENAPI_REGISTER( "sch_net_trace",
                   "Trace a signal through the hierarchy: from a pin (REF.PIN) or net, every sheet instance "
                   "it reaches ordered by distance, with local name, labels and sheet ports it passes and "
                   "the pins on each sheet",
                   R"json({"type":"object","properties":{
                        "pin":{"type":"string","description":"REF.PIN start point"},
                        "net":{"type":"string","description":"net name (alternative to pin)"}}})json"_json,
                   false, h_sch_net_trace );

KOPENAPI_REGISTER( "sch_symbol_neighbors",
                   "What a component is connected to: neighbouring parts level by level (depth), with the "
                   "nets and pins linking them; power/ground and huge nets skipped by default",
                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string"},
                        "depth":{"type":"integer","default":1,"minimum":1,"maximum":6},
                        "skip_power":{"type":"boolean","default":true},
                        "max_fanout":{"type":"integer","default":30,"description":"skip nets with more pins; 0 = no limit"}}})json"_json,
                   false, h_sch_symbol_neighbors );

KOPENAPI_REGISTER( "sch_path_find",
                   "Shortest connection chain between two pins or parts (REF.PIN or REF): nets and the "
                   "parts in between, through passive two-terminal parts by default",
                   R"json({"type":"object","required":["from","to"],"properties":{
                        "from":{"type":"string"},"to":{"type":"string"},
                        "through":{"type":"string","enum":["passive","any"],"default":"passive"},
                        "skip_power":{"type":"boolean","default":true},
                        "max_parts":{"type":"integer","default":6}}})json"_json,
                   false, h_sch_path_find );

KOPENAPI_REGISTER( "sch_power_tree",
                   "Power and ground rails: per rail the sources (power output pins), connectors feeding "
                   "it, consumers (power input pins) and other pins",
                   R"json({"type":"object","properties":{
                        "list_limit":{"type":"integer","default":50,"description":"max listed pins per group"}}})json"_json,
                   false, h_sch_power_tree );

KOPENAPI_REGISTER( "sch_interface_map",
                   "Board interfaces: every connector with each pin's net, role and the far-end pins the "
                   "net reaches; filter connectors by ref glob",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string","description":"connector ref glob"},
                        "list_limit":{"type":"integer","default":10}}})json"_json,
                   false, h_sch_interface_map );

KOPENAPI_REGISTER( "sch_net_search",
                   "Find nets by glob: by net name (incl. local names along the hierarchy), pin name "
                   "(e.g. all nets touching a pin named *RESET*), value of a part on the net (e.g. Z84C15*), "
                   "part reference; or by role (power/ground/clock/reset/signal/unconnected); shows what "
                   "matched; paginated",
                   KopenapiPagedSchema( R"json({
                        "pattern":{"type":"string","description":"glob, case-insensitive"},
                        "by":{"type":"string","enum":["any","name","pin_name","value","ref"],"default":"any",
                              "description":"any = net names and pin names"},
                        "role":{"type":"string","enum":["power","ground","clock","reset","signal","unconnected"]},
                        "list_limit":{"type":"integer","default":10,"description":"max matches listed per net"}})json"_json ),
                   false, h_sch_net_search );

KOPENAPI_REGISTER( "sch_subcircuit",
                   "Functional block around a part (e.g. a regulator with its capacitors, an oscillator "
                   "with crystal and load caps): grows through small parts, stops at power rails, "
                   "connectors and bigger parts (listed as boundary); internal nets, nets leaving the "
                   "block with their outside pins, power rails used",
                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string"},
                        "depth":{"type":"integer","default":2,"minimum":1,"maximum":6},
                        "max_part_pins":{"type":"integer","default":3,"description":"parts with more pins are boundary, not expanded"},
                        "max_fanout":{"type":"integer","default":30,"description":"do not expand through nets with more pins; 0 = no limit"}}})json"_json,
                   false, h_sch_subcircuit );
