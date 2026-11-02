/// @file methods_pcb_multichannel.cpp
/// @brief kicadopenapi multichannel layout, as the editor's Generate Placement Rule Areas and Repeat
/// Layout (KiCad's MULTICHANNEL_TOOL, without its dialogs): placement rule areas per sheet, component
/// class or group, and one channel's placement and routing copied onto the others.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_util.h>
#include <tool/tool_event.h>
#include <tool/tool_manager.h>
#include <tools/multichannel_tool.h>
#include <zone.h>

#include <cmath>
#include <map>
#include <set>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


/// @brief The editor's tool, or one registered on a headless manager (no Init: it builds no menus)
MULTICHANNEL_TOOL* multichannelTool( TOOL_MANAGER* aManager )
{
    wxASSERT( !KopenapiInComputeWorker() );

    if( MULTICHANNEL_TOOL* tool = aManager->GetTool<MULTICHANNEL_TOOL>() )
        return tool;

    aManager->RegisterTool( new MULTICHANNEL_TOOL() );
    return aManager->GetTool<MULTICHANNEL_TOOL>();
}


const char* sourceName( PLACEMENT_SOURCE_T aType )
{
    switch( aType )
    {
    case PLACEMENT_SOURCE_T::SHEETNAME: return "sheet";
    case PLACEMENT_SOURCE_T::COMPONENT_CLASS: return "component_class";
    case PLACEMENT_SOURCE_T::GROUP_PLACEMENT: return "group";
    case PLACEMENT_SOURCE_T::DESIGN_BLOCK: return "design_block";
    }

    return "sheet";
}


wxString sourceOf( const RULE_AREA& aArea )
{
    switch( aArea.m_sourceType )
    {
    case PLACEMENT_SOURCE_T::SHEETNAME: return aArea.m_sheetPath;
    case PLACEMENT_SOURCE_T::COMPONENT_CLASS: return aArea.m_componentClass;
    case PLACEMENT_SOURCE_T::GROUP_PLACEMENT: return aArea.m_groupName;
    default: return wxEmptyString;
    }
}


nlohmann::json refsJson( const std::set<FOOTPRINT*>& aFootprints )
{
    std::vector<std::string> refs;

    for( FOOTPRINT* fp : aFootprints )
        refs.push_back( str( fp->GetReference() ) );

    std::sort( refs.begin(), refs.end() );
    return refs;
}


nlohmann::json areaJson( const RULE_AREA& aArea )
{
    nlohmann::json j = { { "source_type", sourceName( aArea.m_sourceType ) },
                         { "source", str( sourceOf( aArea ) ) },
                         { "footprints", refsJson( aArea.m_components ) } };

    if( aArea.m_zone )
    {
        const BOX2I box = aArea.m_zone->GetBoundingBox();
        j["name"] = str( aArea.m_zone->GetZoneName() );
        j["uuid"] = str( aArea.m_zone->m_Uuid.AsString() );
        j["bbox_mm"] = { toMm( box.GetLeft() ), toMm( box.GetTop() ), toMm( box.GetRight() ), toMm( box.GetBottom() ) };
    }

    return j;
}


std::optional<PLACEMENT_SOURCE_T> parseSource( const std::string& aName )
{
    if( aName == "sheet" )
        return PLACEMENT_SOURCE_T::SHEETNAME;
    if( aName == "component_class" )
        return PLACEMENT_SOURCE_T::COMPONENT_CLASS;
    if( aName == "group" )
        return PLACEMENT_SOURCE_T::GROUP_PLACEMENT;

    return std::nullopt;
}


KOPENAPI_RESULT h_pcb_rule_areas_generate( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    MULTICHANNEL_TOOL* tool = multichannelTool( context->GetToolManager() );
    RULE_AREAS_DATA*   data = tool->GetData();

    tool->GeneratePotentialRuleAreas();

    // the dialog's list: every candidate, ticked or not
    if( aArgs.value( "list", false ) )
    {
        nlohmann::json candidates = nlohmann::json::array();

        for( const RULE_AREA& ra : data->m_areas )
        {
            if( !ra.m_components.empty() )
                candidates.push_back( areaJson( ra ) );
        }

        return KOPENAPI_RESULT::Ok( { { "candidates", candidates } } );
    }

    const std::string sourceArg = aArgs.value( "source", std::string( "sheet" ) );
    std::optional<PLACEMENT_SOURCE_T> source = parseSource( sourceArg );

    if( !source )
        return KOPENAPI_RESULT::Error( 400, "source: sheet, component_class or group" );

    std::set<std::string> names;

    if( aArgs.contains( "names" ) )
    {
        for( const nlohmann::json& n : aArgs["names"] )
        {
            if( n.is_string() )
                names.insert( n.get<std::string>() );
        }
    }

    std::set<std::string> found;
    size_t                enabled = 0;

    for( RULE_AREA& ra : data->m_areas )
    {
        const std::string src = str( sourceOf( ra ) );
        ra.m_generateEnabled = ra.m_sourceType == *source && !ra.m_components.empty()
                               && ( names.empty() || names.count( src ) );

        if( ra.m_generateEnabled )
        {
            found.insert( src );
            ++enabled;
        }
    }

    nlohmann::json unknown = nlohmann::json::array();

    for( const std::string& n : names )
    {
        if( !found.count( n ) )
            unknown.push_back( n );
    }

    if( !unknown.empty() )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 404, "names: no " + sourceArg + " with footprints by these names "
                                                         "(list: true shows the candidates)" );
        r.body["unknown"] = unknown;
        return r;
    }

    if( !enabled )
        return KOPENAPI_RESULT::Error( 404, "no " + sourceArg + " with footprints on this board (list: true shows the candidates)" );

    data->m_replaceExisting = aArgs.value( "replace_existing", false );
    data->m_options.m_groupItems = aArgs.value( "group_items", false );

    std::set<KIID> before;

    for( ZONE* zone : context->GetBoard()->Zones() )
        before.insert( zone->m_Uuid );

    tool->GenerateRuleAreas();
    context->SetContentModified();

    nlohmann::json created = nlohmann::json::array();
    nlohmann::json kept = nlohmann::json::array();

    for( const RULE_AREA& ra : data->m_areas )
    {
        if( !ra.m_generateEnabled )
            continue;

        if( ra.m_zone )
            created.push_back( areaJson( ra ) );
        else
            kept.push_back( str( sourceOf( ra ) ) );
    }

    return KOPENAPI_RESULT::Ok( { { "created", created },
                                  { "existing_kept", kept },
                                  { "replace_existing", data->m_replaceExisting } } );
}


KOPENAPI_RESULT h_pcb_repeat_layout( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*             board = context->GetBoard();
    MULTICHANNEL_TOOL* tool = multichannelTool( context->GetToolManager() );
    RULE_AREAS_DATA*   data = tool->GetData();

    tool->FindExistingRuleAreas();

    auto matches = [&]( const RULE_AREA& aArea, const std::string& aSpec )
    {
        return str( aArea.m_ruleName ) == aSpec || ( aArea.m_zone && str( aArea.m_zone->m_Uuid.AsString() ) == aSpec )
               || ( !aSpec.empty() && str( sourceOf( aArea ) ) == aSpec );
    };

    if( aArgs.value( "list", false ) || !aArgs.contains( "from" ) )
    {
        if( !aArgs.value( "list", false ) )
            return KOPENAPI_RESULT::Error( 400, "from: the reference placement rule area (name, uuid or its sheet / "
                                                "class / group); list: true lists them" );

        nlohmann::json areas = nlohmann::json::array();

        for( const RULE_AREA& ra : data->m_areas )
            areas.push_back( areaJson( ra ) );

        return KOPENAPI_RESULT::Ok( { { "rule_areas", areas } } );
    }

    const std::string from = aArgs.value( "from", std::string() );
    RULE_AREA*        ref = nullptr;

    for( RULE_AREA& ra : data->m_areas )
    {
        if( matches( ra, from ) )
            ref = &ra;
    }

    if( !ref )
        return KOPENAPI_RESULT::Error( 404, "from: no placement rule area '" + from + "' (list: true)" );

    std::vector<RULE_AREA*> targets;

    if( aArgs.contains( "to" ) )
    {
        for( const nlohmann::json& t : aArgs["to"] )
        {
            const std::string spec = t.is_string() ? t.get<std::string>() : t.dump();
            RULE_AREA*        target = nullptr;

            for( RULE_AREA& ra : data->m_areas )
            {
                if( &ra != ref && matches( ra, spec ) )
                    target = &ra;
            }

            if( !target )
                return KOPENAPI_RESULT::Error( 404, "to: no other placement rule area '" + spec + "' (list: true)" );

            targets.push_back( target );
        }
    }
    else
    {
        for( RULE_AREA& ra : data->m_areas )
        {
            if( &ra != ref )
                targets.push_back( &ra );
        }
    }

    if( targets.empty() )
        return KOPENAPI_RESULT::Error( 404, "to: no other placement rule area to repeat the layout to" );

    REPEAT_LAYOUT_OPTIONS options;
    options.m_copyRouting = aArgs.value( "copy_routing", true );
    options.m_connectedRoutingOnly = aArgs.value( "connected_routing_only", false );
    options.m_copyPlacement = aArgs.value( "copy_placement", true );
    options.m_copyOtherItems = aArgs.value( "copy_other_items", true );
    options.m_groupItems = aArgs.value( "group_items", false );
    options.m_includeLockedItems = aArgs.value( "include_locked", true );

    if( aArgs.contains( "anchor" ) )
    {
        const std::string anchor = aArgs.value( "anchor", std::string() );

        for( FOOTPRINT* fp : ref->m_components )
        {
            if( str( fp->GetReference() ) == anchor )
                options.m_anchorFp = fp;
        }

        if( !options.m_anchorFp )
            return KOPENAPI_RESULT::Error( 400, "anchor: '" + anchor + "' is no footprint of the reference area "
                                                + refsJson( ref->m_components ).dump() );
    }

    const bool     dryRun = aArgs.value( "dry_run", false );
    BOARD_COMMIT   commit( context->GetToolManager() );
    TOOL_EVENT     event( TC_COMMAND, TA_ACTION );
    nlohmann::json results = nlohmann::json::array();
    size_t         copied = 0;

    std::map<FOOTPRINT*, std::pair<VECTOR2I, EDA_ANGLE>> was;

    for( RULE_AREA* target : targets )
    {
        for( FOOTPRINT* fp : target->m_components )
            was[fp] = { fp->GetPosition(), fp->GetOrientation() };
    }

    const size_t tracksBefore = board->Tracks().size();

    for( RULE_AREA* target : targets )
    {
        wxString       error;
        nlohmann::json r = areaJson( *target );

        if( tool->RepeatLayout( event, *ref, *target, options, &commit, &error ) == 0 )
        {
            r["ok"] = true;
            ++copied;
        }
        else
        {
            r["ok"] = false;
            r["reason"] = str( error );
        }

        results.push_back( r );
    }

    // what moved, before the commit is pushed or reverted
    for( size_t i = 0; i < targets.size(); ++i )
    {
        if( !results[i].value( "ok", false ) )
            continue;

        size_t moved = 0;

        for( FOOTPRINT* fp : targets[i]->m_components )
        {
            const auto& pose = was[fp];

            if( fp->GetPosition() != pose.first || fp->GetOrientation() != pose.second )
                ++moved;
        }

        results[i]["footprints_moved"] = moved;
    }

    nlohmann::json out = { { "from", areaJson( *ref ) }, { "targets", results }, { "copied", copied }, { "dry_run", dryRun } };

    if( dryRun || !copied )
    {
        commit.Revert();
    }
    else
    {
        commit.Push( _( "Repeat Layout (API)" ) );
        context->SetContentModified();
        out["tracks_added"] = (int64_t) board->Tracks().size() - (int64_t) tracksBefore;
    }

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_rule_areas_generate",
                   "Generate Placement Rule Areas, as the editor's (Placement > Generate Placement Rule Areas): one "
                   "placement rule area around each schematic sheet's, component class's or named group's "
                   "footprints (source: sheet (default), component_class, group; names: only these sheets "
                   "(paths) / classes / groups). Existing areas of a source are kept unless replace_existing; "
                   "group_items groups each area with its footprints. list: true answers the candidates only. "
                   "Then pcb_repeat_layout copies one channel onto the others",
                   R"json({"type":"object","properties":{
                        "source":{"type":"string","enum":["sheet","component_class","group"],"default":"sheet"},
                        "names":{"type":"array","items":{"type":"string"}},
                        "replace_existing":{"type":"boolean","default":false},
                        "group_items":{"type":"boolean","default":false},
                        "list":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_rule_areas_generate );

KOPENAPI_REGISTER( "pcb_repeat_layout",
                   "Repeat Layout, as the editor's (Placement > Repeat Layout): the footprints' placement, the "
                   "routing and other items of the placement rule area from (name, uuid or its sheet / class / "
                   "group) copied onto the areas to (default: every other one), matched by connection topology. "
                   "copy_placement, copy_routing (connected_routing_only: only copper connected to the area's "
                   "pads), copy_other_items, include_locked, group_items; anchor: a footprint of from that the "
                   "copies are aligned on (default the area's position). Each target answers ok or the "
                   "mismatch reason; one undo step; dry_run checks and answers without changing the board. list: "
                   "true lists the placement rule areas with their footprints",
                   R"json({"type":"object","properties":{
                        "from":{"type":"string"},
                        "to":{"type":"array","items":{"type":"string"}},
                        "copy_placement":{"type":"boolean","default":true},
                        "copy_routing":{"type":"boolean","default":true},
                        "connected_routing_only":{"type":"boolean","default":false},
                        "copy_other_items":{"type":"boolean","default":true},
                        "include_locked":{"type":"boolean","default":true},
                        "group_items":{"type":"boolean","default":false},
                        "anchor":{"type":"string"},
                        "dry_run":{"type":"boolean","default":false},
                        "list":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_repeat_layout );

KOPENAPI_MARK_EDITING( "pcb_rule_areas_generate", "pcb_repeat_layout" );
