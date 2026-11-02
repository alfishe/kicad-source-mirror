/// @file methods_sch_netclass.cpp
/// @brief kicadopenapi net classes from the schematic: Assign Netclass (a net name pattern gets a
/// class, as the editor's dialog) and the classes themselves (Schematic Setup > Net Classes), in
/// the project's net settings the board uses too
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <eda_pattern_match.h>
#include <kicadopenapi_netclasses.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <netclass.h>
#include <project.h>
#include <project/net_settings.h>
#include <project/project_file.h>
#include <sch_item.h>
#include <sch_pin.h>
#include <sch_symbol.h>
#include <schematic.h>

#include <algorithm>
#include <map>
#include <set>
#include <vector>

using namespace kopenapi_sch;


namespace
{

/// The editor assigns a net class to labelled nets only: a label, a sheet pin or a power symbol
/// names the net (a directive label names none), not a pin alone
bool isLabelled( const NET_ENTRY& aNet )
{
    for( const NET_INSTANCE& inst : aNet.instances )
    {
        for( SCH_ITEM* item : inst.items )
        {
            if( ( labelKind( item->Type() ) && item->Type() != SCH_DIRECTIVE_LABEL_T ) || item->Type() == SCH_SHEET_PIN_T )
                return true;

            if( item->Type() == SCH_PIN_T )
            {
                if( SCH_SYMBOL* symbol = pinSymbol( static_cast<SCH_PIN*>( item ) ) )
                {
                    if( isRailSymbol( symbol ) )
                        return true;
                }
            }
        }
    }

    return false;
}

} // namespace


static KOPENAPI_RESULT h_sch_netclass_assign( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC*                     schematic = context->GetSchematic();
    std::shared_ptr<NET_SETTINGS>& netSettings = schematic->Project().GetProjectFile().NetSettings();
    const std::string              netclass = aArgs.value( "netclass", std::string() );
    const bool                     remove = aArgs.value( "remove", false );
    const bool                     dryRun = aArgs.value( "dry_run", false );
    const int                      given = aArgs.contains( "pattern" ) + aArgs.contains( "nets" ) + aArgs.contains( "pins" );

    if( netclass.empty() || given != 1 )
        return KOPENAPI_RESULT::Error( 400, "give 'netclass' and one of 'pattern' (a net name pattern, as the dialog's), "
                                            "'nets' (net names) or 'pins' (REF.PIN: the labelled net there)" );

    const wxString className = wxString::FromUTF8( netclass );
    const bool     known = className == NETCLASS::Default || netSettings->HasNetclass( className );

    if( !known && !remove && !aArgs.value( "create", false ) )
    {
        nlohmann::json classes = nlohmann::json::array( { std::string( NETCLASS::Default ) } );

        for( const auto& [name, cls] : netSettings->GetNetclasses() )
            classes.push_back( str( name ) );

        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 404, "no net class " + netclass + " (create: true adds it; sizes: sch_netclass_set)" );
        r.body["netclasses"] = classes;
        return r;
    }

    // The patterns: as typed, or one per selected net (a bus: its members, as the dialog)
    std::vector<wxString>  patterns;
    std::vector<NET_ENTRY> nets = collectNets( schematic );

    if( aArgs.contains( "pattern" ) )
    {
        if( !aArgs["pattern"].is_string() || aArgs["pattern"].get<std::string>().empty() )
            return KOPENAPI_RESULT::Error( 400, "pattern: a net name, wildcard (* ?) or /regex/" );

        patterns.push_back( wxString::FromUTF8( aArgs["pattern"].get<std::string>() ) );
    }
    else
    {
        const char*           key = aArgs.contains( "nets" ) ? "nets" : "pins";
        const nlohmann::json& list = aArgs[key];

        if( !list.is_array() || list.empty() )
            return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": a non-empty list" );

        nlohmann::json unlabelled = nlohmann::json::array();

        for( const nlohmann::json& entry : list )
        {
            if( !entry.is_string() )
                return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": strings" );

            const std::string want = entry.get<std::string>();
            const NET_ENTRY*  found = nullptr;

            for( const NET_ENTRY& net : nets )
            {
                if( std::string( key ) == "nets" ? net.name == want
                                                 : std::ranges::any_of( netPins( net ),
                                                                        [&]( const auto& aPin )
                                                                        {
                                                                            return pinId( aPin.first, aPin.second ) == want;
                                                                        } ) )
                {
                    found = &net;
                    break;
                }
            }

            if( !found )
                return KOPENAPI_RESULT::Error( 404, std::string( key == std::string( "nets" ) ? "net" : "pin" ) + " not found: " + want
                                                    + " (sch_net_list)" );

            if( !isLabelled( *found ) )
                unlabelled.push_back( found->name );
            else
                patterns.push_back( wxString::FromUTF8( found->name ) );
        }

        if( !unlabelled.empty() )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 422, "all nets must be labelled to assign a net class (as the editor): "
                                                             "name them first (sch_label_add), or give a pattern" );
            r.body["unlabelled"] = unlabelled;
            return r;
        }
    }

    std::sort( patterns.begin(), patterns.end() );
    patterns.erase( std::unique( patterns.begin(), patterns.end() ), patterns.end() );

    // Nets the patterns name now
    std::vector<std::string> matching;

    for( const NET_ENTRY& net : nets )
    {
        for( const wxString& pattern : patterns )
        {
            EDA_COMBINED_MATCHER matcher( pattern, CTX_NETCLASS );

            if( matcher.StartsWith( wxString::FromUTF8( net.name ) ) )
            {
                matching.push_back( net.name );
                break;
            }
        }
    }

    int removed = 0;

    if( !dryRun )
    {
        if( remove )
        {
            auto& assignments = netSettings->GetNetclassPatternAssignments();
            std::vector<std::pair<std::unique_ptr<EDA_COMBINED_MATCHER>, wxString>> kept;

            for( auto& [matcher, cls] : assignments )
            {
                if( cls == className && std::ranges::find( patterns, matcher->GetPattern() ) != patterns.end() )
                    removed++;
                else
                    kept.emplace_back( std::move( matcher ), cls );
            }

            netSettings->SetNetclassPatternAssignments( std::move( kept ) );
            netSettings->ClearAllCaches();
        }
        else
        {
            if( !known )
            {
                std::shared_ptr<NETCLASS> created = std::make_shared<NETCLASS>( className );
                netSettings->SetNetclass( className, created );
            }

            for( const wxString& pattern : patterns )
                netSettings->SetNetclassPatternAssignment( pattern, className );
        }
    }

    nlohmann::json patternList = nlohmann::json::array();

    for( const wxString& pattern : patterns )
        patternList.push_back( str( pattern ) );

    nlohmann::json netList = nlohmann::json::array();

    for( const std::string& name : matching )
    {
        netList.push_back( { { "net", name },
                             { "net_class", str( netSettings->GetEffectiveNetClass( wxString::FromUTF8( name ) )->GetName() ) } } );
    }

    nlohmann::json answer = { { "netclass", netclass },
                              { "patterns", patternList },
                              { "matching", matching.size() },
                              { "dry_run", dryRun },
                              { "created", !known && !remove && !dryRun } };

    if( remove )
        answer["removed"] = removed;

    KOPENAPI_LIST_CAP( aArgs, 100 ).Put( answer, "nets", netList );

    if( !dryRun )
        answer["note"] = "a project setting, as in the editor: no undo step; saved with the project (sch_save); remove: true takes a pattern back";

    if( matching.empty() && !remove )
        answer["warning"] = "no net matches the pattern now";

    return KOPENAPI_RESULT::Ok( answer );
}


/// Classes, patterns and the nets each class holds now (a net in several classes under each,
/// under Default only when it has no other)
static nlohmann::json netclassesAnswer( SCHEMATIC* aSchematic, const nlohmann::json& aArgs )
{
    NET_SETTINGS&  settings = *aSchematic->Project().GetProjectFile().NetSettings();
    nlohmann::json answer = KopenapiNetclassesJson( settings, true );
    std::map<std::string, nlohmann::json> members;

    for( const NET_ENTRY& net : collectNets( aSchematic ) )
    {
        std::shared_ptr<NETCLASS> effective = settings.GetEffectiveNetClass( wxString::FromUTF8( net.name ) );
        std::vector<std::string>  names;

        for( NETCLASS* part : effective->GetConstituentNetclasses() )
        {
            if( part->GetName() != NETCLASS::Default )
                names.push_back( str( part->GetName() ) );
        }

        if( names.empty() )
            names.push_back( NETCLASS::Default );

        for( const std::string& name : names )
            members[name].push_back( net.name );
    }

    KOPENAPI_LIST_CAP cap( aArgs, 50 );
    nlohmann::json    classes = nlohmann::json::array();

    for( nlohmann::json c : answer["netclasses"] )
    {
        nlohmann::json nets = members.count( c["name"] ) ? members[c["name"]] : nlohmann::json::array();
        c["net_count"] = nets.size();
        cap.Put( c, "nets", std::move( nets ) );
        classes.push_back( std::move( c ) );
    }

    answer["netclasses"] = classes;
    return answer;
}


static KOPENAPI_RESULT h_sch_netclass_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    return KOPENAPI_RESULT::Ok( netclassesAnswer( context->GetSchematic(), aArgs ) );
}


static KOPENAPI_RESULT h_sch_netclass_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "netclasses" ) )
        return KOPENAPI_RESULT::Error( 400, "give 'netclasses': [{name, values, nets, delete}]" );

    if( const std::string why = KopenapiNetclassesCheck( aArgs["netclasses"] ); !why.empty() )
        return KOPENAPI_RESULT::Error( 400, why );

    SCHEMATIC* schematic = context->GetSchematic();
    KopenapiNetclassesApply( *schematic->Project().GetProjectFile().NetSettings(), aArgs["netclasses"] );

    nlohmann::json answer = netclassesAnswer( schematic, aArgs );
    answer["note"] = "project settings, as Schematic Setup > Net Classes: no undo step; saved with the project (sch_save); "
                     "the board reads the same classes";
    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "sch_netclass_assign",
                   "Assign Netclass (schematic): nets by name pattern get a net class in the project's net "
                   "settings, as the editor's dialog. pattern (a net name, wildcard * ?, or /regex/), nets "
                   "(names) or pins (REF.PIN: the net there); nets and pins must be labelled nets (a label, "
                   "sheet pin or power symbol names them; 422 else, as the editor), a bus gives its members. "
                   "The class must exist (404 with the list) unless create: true (a new class with default "
                   "sizes). dry_run answers the matching nets only; remove: true takes the patterns back. "
                   "Answers the nets the patterns match with the class now in force (net_class: label "
                   "assignments by directive labels win, as in KiCad). A net class by a flag on the wire: "
                   "sch_label_add kind directive. The classes and their sizes: sch_netclass_set",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","required":["netclass"],"properties":{
                        "netclass":{"type":"string"},
                        "pattern":{"type":"string"},
                        "nets":{"type":"array","items":{"type":"string"}},
                        "pins":{"type":"array","items":{"type":"string"}},
                        "create":{"type":"boolean","default":false},
                        "remove":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json, 100 ),
                   false, h_sch_netclass_assign );

static nlohmann::json netclassSetSchema()
{
    nlohmann::json item = { { "type", "object" }, { "required", { "name" } } };
    nlohmann::json props = KopenapiNetclassBoardSchema();
    props.update( KopenapiNetclassSchematicSchema() );
    props["name"] = { { "type", "string" } };
    props["nets"] = { { "type", "array" }, { "items", { { "type", "string" } } },
                      { "description", "the class's nets / patterns: replaces its earlier ones, and the nets leave other classes; [] empties it" } };
    props["delete"] = { { "type", "boolean" }, { "description", "remove this class and its net patterns" } };
    item["properties"] = props;

    return KOPENAPI_LIST_CAP::Schema( { { "type", "object" }, { "required", { "netclasses" } },
                                        { "properties", { { "netclasses", { { "type", "array" }, { "items", item } } } } } }, 50 );
}


KOPENAPI_REGISTER( "sch_netclass_list",
                   "Net classes of the project (Schematic Setup > Net Classes; the board uses the same): each class "
                   "with the values it sets (board: clearance, track, via, diff pair; schematic: wire / bus width, "
                   "color, line style), the nets it holds now (patterns and directive labels; net_count, nets "
                   "capped at list_limit), and the pattern assignments",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{}})json"_json, 50 ),
                   false, h_sch_netclass_list );

KOPENAPI_REGISTER( "sch_netclass_set",
                   "Define net classes from the schematic, without a board (Schematic Setup > Net Classes): "
                   "netclasses [{name ('Default' edits the default), clearance_mm, track_width_mm, "
                   "via_diameter_mm, via_drill_mm, diff_pair_width_mm, diff_pair_gap_mm, wire_width_mm, "
                   "bus_width_mm, color, line_style, nets: [names or patterns], delete}] - the same project net "
                   "settings and rules as pcb_rules_set netclasses, so the board gets these classes. All or "
                   "nothing (400 names the bad value). Answers as sch_netclass_list",
                   netclassSetSchema(), false, h_sch_netclass_set );

KOPENAPI_MARK_EDITING( "sch_netclass_assign", "sch_netclass_set" );
