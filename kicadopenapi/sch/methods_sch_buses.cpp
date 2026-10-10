/// @file methods_sch_buses.cpp
/// @brief kicadopenapi schematic buses (docs/api/design-analysis-api.md §3 sch_buses): sch_bus_list, sch_bus_get.
///
/// A bus is reported as a *drawn bus group*: bus segments connected end-to-segment on one sheet
/// instance.  Its members are what actually enters it — the nets of the wires attached through
/// bus entries — so graphical buses without a bus label (designs converted from P-CAD and other
/// tools) still tell which signals run in them.  When the bus carries a bus label ("D[0..7]",
/// "PCI{AD[0..31] CBE[0..3]}"), the declared members are expanded too and compared.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <advanced_config.h>
#include <api/sch_context.h>
#include <base_units.h>
#include <connectivity/conn_bus.h>
#include <connectivity/conn_facade.h>
#include <kicadopenapi_util.h>
#include <project/net_settings.h>
#include <sch_bus_entry.h>
#include <sch_connection.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_sheet_pin.h>
#include <schematic.h>
#include <string_utils.h>

#include <algorithm>
#include <map>
#include <numeric>
#include <set>

using namespace kopenapi_sch;


namespace
{

/// @brief Name without the hierarchical prefix; a trailing '/' (active-low) is kept
std::string shortName( const std::string& aName )
{
    const size_t slash = aName.size() > 1 ? aName.find_last_of( '/', aName.size() - 2 ) : std::string::npos;
    return slash == std::string::npos ? aName : aName.substr( slash + 1 );
}


bool onSegment( const VECTOR2I& aP, const VECTOR2I& aA, const VECTOR2I& aB )
{
    const int64_t cross = int64_t( aB.x - aA.x ) * ( aP.y - aA.y ) - int64_t( aB.y - aA.y ) * ( aP.x - aA.x );

    return cross == 0 && aP.x >= std::min( aA.x, aB.x ) && aP.x <= std::max( aA.x, aB.x )
           && aP.y >= std::min( aA.y, aB.y ) && aP.y <= std::max( aA.y, aB.y );
}


struct MEMBER
{
    std::string net;        ///< global net name
    std::string localName;  ///< name on this sheet
    int         entries = 0;
};


struct BUS_GROUP
{
    std::string                   id;
    SCH_SHEET_PATH                path;
    std::vector<SCH_LINE*>        segments;
    int                           wireEntries = 0;
    int                           busEntries = 0;
    int                           unattachedEntries = 0;   ///< wire side touches no net
    std::set<std::string>         labels;                  ///< bus labels / bus sheet pins on it
    std::map<std::string, MEMBER> members;                 ///< by net name
    std::vector<std::string>      declared;                ///< expanded from the labels
    std::map<std::string, std::string> declaredNet;        ///< declared member -> net on this sheet ("" = none)
    bool                          fromEngine = false;      ///< members resolved by KiCad's connectivity
    double                        lengthMm = 0;
    BOX2I                         bbox;

    bool touches( const VECTOR2I& aPoint ) const
    {
        for( SCH_LINE* seg : segments )
        {
            if( onSegment( aPoint, seg->GetStartPoint(), seg->GetEndPoint() ) )
                return true;
        }

        return false;
    }
};


std::vector<BUS_GROUP> collectBuses( SCHEMATIC* aSchematic )
{
    // (sheet instance, item) -> net, for wires and bus entries
    std::map<std::pair<std::string, const SCH_ITEM*>, std::pair<std::string, std::string>> netOfItem;

    // (sheet instance, short local name) -> net: resolves declared bus members on that sheet
    std::map<std::pair<std::string, std::string>, std::string> netByLocalName;

    for( const NET_ENTRY& net : collectNets( aSchematic ) )
    {
        for( const NET_INSTANCE& inst : net.instances )
        {
            const std::string key = str( inst.path.Path().AsString() );
            netByLocalName.emplace( std::make_pair( key, shortName( inst.localName ) ), net.name );
            netByLocalName.emplace( std::make_pair( key, shortName( net.name ) ), net.name );

            for( SCH_ITEM* item : inst.items )
            {
                if( item->Type() == SCH_LINE_T || item->Type() == SCH_BUS_WIRE_ENTRY_T )
                    netOfItem[{ key, item }] = { net.name, inst.localName };
            }
        }
    }

    // KiCad's own bus members (connectivity engine): (sheet instance, bus segment) -> member
    // name as declared on that bus -> member net.  Authoritative: it follows renames across
    // sheets and aliases, which matching label text against net names cannot.
    std::map<std::pair<std::string, const SCH_ITEM*>, std::vector<std::pair<std::string, std::string>>> engineMembers;

    if( ADVANCED_CFG::GetCfg().m_ConnectivityEngine )
    {
        std::map<KIID_PATH, std::string> keys;

        for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
            keys.emplace( path.Path(), str( path.Path().AsString() ) );

        for( const SCH_CONNECTIVITY::NET_GROUP& group : aSchematic->Connectivity().GetNetMap() )
        {
            for( const SCH_CONNECTIVITY::NET_VIEW& view : group.instances )
            {
                if( !view.IsBus() )
                    continue;

                auto key = keys.find( view.Instance() );

                if( key == keys.end() )
                    continue;

                const SCH_CONNECTIVITY::BUS_MEMBERS members = view.Members();
                std::vector<std::pair<std::string, std::string>> leaves;   // in declaration order

                if( members.schema && members.schema->leaves.size() == members.leaves.size() )
                {
                    for( size_t i = 0; i < members.leaves.size(); ++i )
                    {
                        leaves.emplace_back( str( UnescapeString( members.schema->leaves[i].localName ) ),
                                             str( UnescapeString( members.leaves[i].Name() ) ) );
                    }
                }

                for( SCH_ITEM* item : view.Items() )
                {
                    if( item->Type() == SCH_LINE_T && !leaves.empty() )
                        engineMembers[{ key->second, item }] = leaves;
                }
            }
        }
    }

    std::vector<BUS_GROUP> groups;

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        SCH_SCREEN*            screen = path.LastScreen();
        const std::string      pathKey = str( path.Path().AsString() );
        std::vector<SCH_LINE*> buses, wires;

        for( SCH_ITEM* item : screen->Items().OfType( SCH_LINE_T ) )
        {
            SCH_LINE* line = static_cast<SCH_LINE*>( item );

            if( line->IsBus() )
                buses.push_back( line );
            else if( line->IsWire() )
                wires.push_back( line );
        }

        if( buses.empty() )
            continue;

        // Union bus segments whose end lies on another segment
        std::vector<size_t> parent( buses.size() );
        std::iota( parent.begin(), parent.end(), 0 );

        std::function<size_t( size_t )> root = [&]( size_t i ) { return parent[i] == i ? i : parent[i] = root( parent[i] ); };

        for( size_t i = 0; i < buses.size(); ++i )
        {
            for( size_t j = i + 1; j < buses.size(); ++j )
            {
                const SCH_LINE* a = buses[i];
                const SCH_LINE* b = buses[j];

                if( onSegment( a->GetStartPoint(), b->GetStartPoint(), b->GetEndPoint() )
                    || onSegment( a->GetEndPoint(), b->GetStartPoint(), b->GetEndPoint() )
                    || onSegment( b->GetStartPoint(), a->GetStartPoint(), a->GetEndPoint() )
                    || onSegment( b->GetEndPoint(), a->GetStartPoint(), a->GetEndPoint() ) )
                {
                    parent[root( i )] = root( j );
                }
            }
        }

        std::map<size_t, BUS_GROUP> byRoot;

        for( size_t i = 0; i < buses.size(); ++i )
        {
            BUS_GROUP& g = byRoot[root( i )];
            g.path = path;
            g.segments.push_back( buses[i] );
            g.lengthMm += schIUScale.IUTomm( ( buses[i]->GetEndPoint() - buses[i]->GetStartPoint() ).EuclideanNorm() );

            if( g.bbox.GetWidth() == 0 && g.bbox.GetHeight() == 0 && g.segments.size() == 1 )
                g.bbox = buses[i]->GetBoundingBox();
            else
                g.bbox.Merge( buses[i]->GetBoundingBox() );
        }

        std::vector<BUS_GROUP*> sheetGroups;

        for( auto& [r, g] : byRoot )
            sheetGroups.push_back( &g );

        auto groupAt = [&]( const VECTOR2I& aPoint ) -> BUS_GROUP*
        {
            for( BUS_GROUP* g : sheetGroups )
            {
                if( g->touches( aPoint ) )
                    return g;
            }

            return nullptr;
        };

        // Bus entries: the end on a bus picks the group, the other end the member net
        for( SCH_ITEM* item : screen->Items().OfType( SCH_BUS_WIRE_ENTRY_T ) )
        {
            SCH_BUS_WIRE_ENTRY* entry = static_cast<SCH_BUS_WIRE_ENTRY*>( item );
            VECTOR2I            ends[2] = { entry->GetPosition(), entry->GetEnd() };

            for( int side = 0; side < 2; ++side )
            {
                BUS_GROUP* g = groupAt( ends[side] );

                if( !g )
                    continue;

                g->wireEntries++;
                const VECTOR2I wireEnd = ends[1 - side];

                std::pair<std::string, std::string> net;
                auto it = netOfItem.find( { pathKey, entry } );

                if( it != netOfItem.end() )
                {
                    net = it->second;
                }
                else
                {
                    for( SCH_LINE* wire : wires )
                    {
                        if( onSegment( wireEnd, wire->GetStartPoint(), wire->GetEndPoint() ) )
                        {
                            if( auto w = netOfItem.find( { pathKey, wire } ); w != netOfItem.end() )
                                net = w->second;

                            break;
                        }
                    }
                }

                if( net.first.empty() )
                {
                    g->unattachedEntries++;
                }
                else
                {
                    MEMBER& m = g->members[net.first];
                    m.net = net.first;
                    m.localName = net.second;
                    m.entries++;
                }

                break;
            }
        }

        for( SCH_ITEM* item : screen->Items().OfType( SCH_BUS_BUS_ENTRY_T ) )
        {
            SCH_BUS_BUS_ENTRY* entry = static_cast<SCH_BUS_BUS_ENTRY*>( item );

            if( BUS_GROUP* g = groupAt( entry->GetPosition() ) )
                g->busEntries++;
        }

        // Bus labels and bus sheet pins name the group and declare its members
        auto addLabel = [&]( const wxString& aText, const VECTOR2I& aPos )
        {
            if( !SCH_CONNECTION::IsBusLabel( aText ) )
                return;

            if( BUS_GROUP* g = groupAt( aPos ) )
                g->labels.insert( str( UnescapeString( aText ) ) );
        };

        for( SCH_ITEM* item : screen->Items() )
        {
            if( labelKind( item->Type() ) )
                addLabel( static_cast<SCH_LABEL_BASE*>( item )->GetText(), item->GetPosition() );
            else if( item->Type() == SCH_SHEET_T )
            {
                for( SCH_SHEET_PIN* pin : static_cast<SCH_SHEET*>( item )->GetPins() )
                    addLabel( pin->GetText(), pin->GetPosition() );
            }
        }

        // Stable ids: sheet path + order by position (top, then left)
        std::sort( sheetGroups.begin(), sheetGroups.end(),
                   []( const BUS_GROUP* a, const BUS_GROUP* b )
                   {
                       if( a->bbox.GetTop() != b->bbox.GetTop() )
                           return a->bbox.GetTop() < b->bbox.GetTop();

                       return a->bbox.GetLeft() < b->bbox.GetLeft();
                   } );

        for( size_t i = 0; i < sheetGroups.size(); ++i )
        {
            BUS_GROUP* g = sheetGroups[i];
            g->id = sheetPath( path ) + "#" + std::to_string( i + 1 );

            // KiCad's members of this bus, in declaration order of the driving label
            std::vector<std::pair<std::string, std::string>> engine;

            for( SCH_LINE* seg : g->segments )
            {
                if( auto it = engineMembers.find( { pathKey, seg } ); it != engineMembers.end() )
                {
                    engine = it->second;
                    break;
                }
            }

            g->fromEngine = !engine.empty();

            // Every label's members resolve to nets: by ordinal against KiCad's members (vector
            // buses align by ordinal, so a second label or a name from the parent sheet is an
            // alias of the same lines), else by name on this sheet
            std::map<std::string, std::string> engineByName( engine.begin(), engine.end() );
            std::set<std::string>              declared;

            for( const std::string& label : g->labels )
            {
                std::vector<std::string> expanded;
                NET_SETTINGS::ForEachBusMember( wxString::FromUTF8( label ),
                                                [&]( const wxString& aMember ) { expanded.push_back( str( aMember ) ); } );

                for( size_t k = 0; k < expanded.size(); ++k )
                {
                    const std::string& member = expanded[k];
                    declared.insert( member );

                    if( expanded.size() == engine.size() )
                        g->declaredNet[member] = engine[k].second;
                    else if( auto e = engineByName.find( member ); e != engineByName.end() )
                        g->declaredNet[member] = e->second;
                    else if( auto n = netByLocalName.find( { pathKey, member } ); n != netByLocalName.end() )
                        g->declaredNet[member] = n->second;
                    else
                        g->declaredNet[member] = std::string();
                }
            }

            // A bus without a label of its own on this sheet (named through a sheet pin elsewhere)
            for( const auto& [member, net] : engine )
            {
                if( declared.insert( member ).second )
                    g->declaredNet[member] = net;
            }

            g->declared.assign( declared.begin(), declared.end() );
            std::sort( g->declared.begin(), g->declared.end(), KopenapiNaturalLess );
            groups.push_back( std::move( *g ) );
        }
    }

    return groups;
}


std::vector<std::string> memberNames( const BUS_GROUP& aGroup )
{
    std::vector<std::string> names;

    for( const auto& [net, m] : aGroup.members )
        names.push_back( net );

    std::sort( names.begin(), names.end(), KopenapiNaturalLess );
    return names;
}


/// @brief Declared members (expanded bus labels) resolved to the nets of this sheet, and the two real
/// findings: a declared member no net carries here, and an entry whose net the label does not
/// declare.  A labelled bus often has no entries at all (a stub between a sheet pin and a
/// label): its members are connected through the label, so "declared but not entered" is no
/// finding.
void compareDeclared( const BUS_GROUP& aGroup, nlohmann::json& aOut, bool aWithNets )
{
    if( aGroup.declared.empty() )
        return;

    nlohmann::json unresolved = nlohmann::json::array(), undeclared = nlohmann::json::array();
    nlohmann::json resolved = nlohmann::json::array();
    std::set<std::string> declaredNets;

    for( const std::string& d : aGroup.declared )
    {
        const std::string& net = aGroup.declaredNet.at( d );

        if( net.empty() )
            unresolved.push_back( d );
        else
            declaredNets.insert( net );

        if( aWithNets )
            resolved.push_back( { { "member", d }, { "net", net.empty() ? nlohmann::json() : nlohmann::json( net ) } } );
    }

    for( const auto& [net, m] : aGroup.members )
    {
        if( !declaredNets.count( net ) )
            undeclared.push_back( net );
    }

    // Several labels on one bus name the same lines: list which names meet on each net
    if( aGroup.labels.size() > 1 )
    {
        std::map<std::string, std::vector<std::string>> byNet;

        for( const std::string& d : aGroup.declared )
        {
            if( const std::string& net = aGroup.declaredNet.at( d ); !net.empty() )
                byNet[net].push_back( d );
        }

        nlohmann::json aliases = nlohmann::json::array();

        for( auto& [net, names] : byNet )
        {
            if( names.size() > 1 && aliases.size() < 8 )
                aliases.push_back( { { "net", net }, { "names", names } } );
        }

        aOut["label_aliases"] = aliases;
    }

    aOut["declared_count"] = aGroup.declared.size();
    aOut["declared_source"] = aGroup.fromEngine ? "engine" : "label";
    aOut["declared_without_net"] = unresolved;
    aOut["entered_not_declared"] = undeclared;

    if( aWithNets )
        aOut["declared"] = resolved;
}

} // namespace


static KOPENAPI_RESULT h_sch_bus_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string sheetGlob = aArgs.value( "sheet", std::string() );
    const std::string nameGlob = aArgs.value( "name", std::string() );
    const std::string memberGlob = aArgs.value( "member", std::string() );
    const int         listLimit = std::clamp( aArgs.value( "list_limit", 16 ), 0, 10000 );

    std::vector<nlohmann::json> rows;

    for( const BUS_GROUP& g : collectBuses( context->GetSchematic() ) )
    {
        if( !KopenapiGlob( sheetGlob, sheetPath( g.path ) ) )
            continue;

        if( !nameGlob.empty()
            && std::none_of( g.labels.begin(), g.labels.end(), [&]( const std::string& l ) { return KopenapiGlob( nameGlob, l ); } ) )
        {
            continue;
        }

        const std::vector<std::string> members = memberNames( g );

        if( !memberGlob.empty()
            && std::none_of( members.begin(), members.end(), [&]( const std::string& m ) { return KopenapiGlob( memberGlob, m ); } ) )
        {
            continue;
        }

        nlohmann::json shown = nlohmann::json::array();

        for( size_t i = 0; i < members.size() && i < (size_t) listLimit; ++i )
            shown.push_back( members[i] );

        nlohmann::json row = { { "id", g.id },
                               { "sheet", sheetPath( g.path ) },
                               { "labels", g.labels },
                               { "segments", g.segments.size() },
                               { "length_mm", std::round( g.lengthMm * 100 ) / 100 },
                               { "wire_entries", g.wireEntries },
                               { "member_count", members.size() },
                               { "members", shown } };

        compareDeclared( g, row, false );
        rows.push_back( std::move( row ) );
    }

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


static KOPENAPI_RESULT h_sch_bus_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string id = aArgs.value( "id", std::string() );
    const std::string member = aArgs.value( "member", std::string() );

    if( id.empty() && member.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'id' (from sch_bus_list) or 'member' (a net name)" );

    std::vector<BUS_GROUP> groups = collectBuses( context->GetSchematic() );
    std::vector<nlohmann::json> found;

    // Roles of member nets
    std::map<std::string, std::string> roles;
    std::map<std::string, int>         pins;

    for( const NET_ENTRY& net : collectNets( context->GetSchematic() ) )
    {
        roles[net.name] = netRole( net );
        int n = 0;

        for( const auto& [pin, path] : netPins( net ) )
            n += pinSymbol( pin ) && !pinSymbol( pin )->IsPower() ? 1 : 0;

        pins[net.name] = n;
    }

    for( const BUS_GROUP& g : groups )
    {
        if( !id.empty() && g.id != id )
            continue;

        if( !member.empty() && !g.members.count( member ) && !g.members.count( str( UnescapeString( wxString::FromUTF8( member ) ) ) ) )
            continue;

        nlohmann::json members = nlohmann::json::array();

        for( const std::string& name : memberNames( g ) )
        {
            const MEMBER& m = g.members.at( name );
            members.push_back( { { "net", name }, { "local_name", m.localName }, { "entries", m.entries },
                                 { "role", roles[name] }, { "pins", pins[name] } } );
        }

        nlohmann::json bus = { { "id", g.id },
                               { "sheet", sheetPath( g.path ) },
                               { "labels", g.labels },
                               { "segments", g.segments.size() },
                               { "length_mm", std::round( g.lengthMm * 100 ) / 100 },
                               { "bbox_mm",
                                 { schIUScale.IUTomm( g.bbox.GetLeft() ), schIUScale.IUTomm( g.bbox.GetTop() ),
                                   schIUScale.IUTomm( g.bbox.GetRight() ), schIUScale.IUTomm( g.bbox.GetBottom() ) } },
                               { "wire_entries", g.wireEntries },
                               { "bus_bus_entries", g.busEntries },
                               { "entries_without_net", g.unattachedEntries },
                               { "members", members } };

        compareDeclared( g, bus, true );
        found.push_back( std::move( bus ) );
    }

    if( found.empty() )
        return KOPENAPI_RESULT::Error( 404, "bus not found" );

    if( !id.empty() )
        return KOPENAPI_RESULT::Ok( found.front() );

    return KOPENAPI_RESULT::Ok( { { "member", member }, { "buses", found } } );
}


KOPENAPI_REGISTER( "sch_bus_list",
                   "List drawn buses per sheet instance: id, bus labels (e.g. D[0..7]), segments, length, "
                   "wire entries, member nets that actually enter the bus (works for graphical buses "
                   "without labels), declared members without a net, entries not declared by the label; "
                   "filter by sheet, label name or member net glob; paginated",
                   KopenapiPagedSchema( R"json({
                        "sheet":{"type":"string","description":"glob on the sheet path"},
                        "name":{"type":"string","description":"glob on a bus label"},
                        "member":{"type":"string","description":"glob on a member net: buses carrying it"},
                        "list_limit":{"type":"integer","default":16,"description":"members listed per bus"}})json"_json ),
                   false, h_sch_bus_list );

KOPENAPI_REGISTER( "sch_bus_get",
                   "Bus card: every member net entering the bus (local name, entries, role, pins), bus "
                   "labels with their expanded declared members resolved to nets on the sheet, declared "
                   "members without a net, entries not declared by the label, geometry; by id from "
                   "sch_bus_list, or all buses a member net runs in",
                   R"json({"type":"object","properties":{
                        "id":{"type":"string","description":"bus id, e.g. sp-dx/Sheet2#3"},
                        "member":{"type":"string","description":"net name: every bus it enters"}}})json"_json,
                   false, h_sch_bus_get );
