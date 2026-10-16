/// @file methods_pcb_checks.cpp
/// @brief kicadopenapi board checks: pcb_drc.
///
/// Runs KiCad's own DRC engine (as `kicad-cli pcb drc` does) on the live board and returns the
/// drc.v1 report KiCad writes (DRC_REPORT::GetJsonReport), plus a summary and a flat,
/// filterable, paginated violation list for agents.  Markers land on the board (in the GUI
/// they show in the editor), without an undo step and without marking the board modified;
/// zones are refilled only when asked (that does change the board).
#include "kopenapi_pcb.h"

#include <map>
#include <set>

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
#include <board.h>
#include <tool/actions.h>
#include <tools/drc_tool.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <drc/drc_item.h>
#include <drc/drc_report.h>
#include <footprint.h>
#include <footprint_library_adapter.h>
#include <kicadopenapi_checks.h>
#include <pcb_marker.h>
#include <project_pcb.h>
#include <tool/tool_manager.h>
#include <tools/zone_filler_tool.h>
#include <zone.h>


static KOPENAPI_RESULT h_pcb_drc( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    int severities = 0;

    if( !KopenapiSeverityMask( aArgs, severities ) )
        return KOPENAPI_RESULT::Error( 400, "severity: list of error, warning, exclusion" );

    BOARD*        board = context->GetBoard();
    TOOL_MANAGER* toolManager = context->GetToolManager();

    // the run replaces every marker: nothing may keep pointing at the old ones (the selection,
    // the DRC dialog's lists)
    if( DRC_TOOL* drcTool = toolManager->GetTool<DRC_TOOL>() )
    {
        if( drcTool->IsDRCDialogShown() )
            return KOPENAPI_RESULT::Error( 409, "the DRC dialog is open: close it first" );

        drcTool->DestroyDRCDialog();
    }

    toolManager->RunAction( ACTIONS::selectionClear );

    // DRC needs the footprint libraries (library footprint mismatch checks)
    FOOTPRINT_LIBRARY_ADAPTER* adapter = PROJECT_PCB::FootprintLibAdapter( board->GetProject() );
    adapter->AsyncLoad();
    KopenapiWaitLibraries( adapter );

    KOPENAPI_KEEPALIVE_REPORTER reporter;

    std::shared_ptr<DRC_ENGINE> engine = board->GetDesignSettings().m_DRCEngine;

    if( !engine->RulesValid() )
    {
        try
        {
            engine->InitEngine( board->GetDesignRulesPath() );
        }
        catch( const PARSE_ERROR& error )
        {
            return KOPENAPI_RESULT::Error( 422, "custom design rules do not compile: " + error.What().ToStdString( wxConvUTF8 ) );
        }
    }

    if( aArgs.value( "refill_zones", false ) )
    {
        if( !toolManager->FindTool( ZONE_FILLER_TOOL_NAME ) )
            toolManager->RegisterTool( new ZONE_FILLER_TOOL );

        toolManager->GetTool<ZONE_FILLER_TOOL>()->FillAllZones( nullptr, &reporter, true );
    }

    BOARD_COMMIT commit( toolManager );

    engine->SetViolationHandler(
            [&]( const std::shared_ptr<DRC_ITEM>& aItem, const VECTOR2I& aPos, int aLayer,
                 const std::function<void( PCB_MARKER* )>& aPathGenerator )
            {
                PCB_MARKER* marker = new PCB_MARKER( aItem, aPos, aLayer );
                aPathGenerator( marker );
                commit.Add( marker );
            } );

    board->RecordDRCExclusions();
    board->DeleteMARKERs( true, true );
    // Every track error by default: with one error per track KiCad keeps whichever segment its
    // threads report first, so repeated runs differ (seen on Sprinter DX: 2..4 clearance errors)
    engine->SetProgressReporter( &reporter );
    engine->RunTests( EDA_UNITS::MM, aArgs.value( "all_track_errors", true ), false );
    engine->SetProgressReporter( nullptr );
    engine->ClearViolationHandler();

    commit.Push( _( "DRC" ), SKIP_UNDO | SKIP_SET_DIRTY );
    board->ResolveDRCExclusions( false );

    auto markers = std::make_shared<DRC_ITEMS_PROVIDER>( board, MARKER_BASE::MARKER_DRC, MARKER_BASE::MARKER_DRAWING_SHEET );
    auto unconnected = std::make_shared<DRC_ITEMS_PROVIDER>( board, MARKER_BASE::MARKER_RATSNEST );
    auto parity = std::make_shared<DRC_ITEMS_PROVIDER>( board, MARKER_BASE::MARKER_PARITY );

    markers->SetSeverities( severities );
    unconnected->SetSeverities( severities );
    parity->SetSeverities( severities );

    DRC_REPORT     report( board, EDA_UNITS::MM, markers, unconnected, parity );
    nlohmann::json full = report.GetJsonReport();

    // The R-trees and result caches DRC built stay until the next DRC otherwise (~230 MB on a
    // large board); every other user rebuilds or falls back
    board->ReleaseDRCCaches();

    // The editor draws the solder mask bridges after a DRC; headless nothing does
    if( !toolManager->GetView() && board->m_SolderMaskBridges )
    {
        for( PCB_LAYER_ID layer : { F_Mask, B_Mask } )
            board->m_SolderMaskBridges->GetFill( layer )->RemoveAllContours();
    }

    std::vector<nlohmann::json> violations;

    for( const nlohmann::json& v : full.value( "violations", nlohmann::json::array() ) )
        violations.push_back( v );

    // where the marker is (the report gives the items' positions only: a zone island's item is at
    // the zone's anchor, its marker on the island)
    for( int i = 0; i < markers->GetCount() && i < (int) violations.size(); ++i )
    {
        if( const MARKER_BASE* marker = markers->GetItem( i )->GetParent() )
            violations[i]["at_mm"] = { pcbIUScale.IUTomm( marker->GetPos().x ), pcbIUScale.IUTomm( marker->GetPos().y ) };
    }

    // a footprint's own violations (all its items in one footprint: a library part's pads against its
    // own pegs) left out when asked: true for every footprint, or a list of references
    const nlohmann::json within = aArgs.value( "exclude_within_footprint", nlohmann::json( false ) );
    nlohmann::json       excluded = nlohmann::json::array();

    if( within.is_array() || ( within.is_boolean() && within.get<bool>() ) )
    {
        std::set<wxString> refs;

        if( within.is_array() )
        {
            for( const nlohmann::json& r : within )
            {
                if( r.is_string() )
                    refs.insert( wxString::FromUTF8( r.get<std::string>() ) );
            }
        }

        std::map<std::string, FOOTPRINT*> owner;

        for( FOOTPRINT* fp : board->Footprints() )
        {
            owner[fp->m_Uuid.AsStdString()] = fp;
            fp->RunOnChildren( [&]( BOARD_ITEM* aChild ) { owner[aChild->m_Uuid.AsStdString()] = fp; }, RECURSE_MODE::RECURSE );
        }

        std::vector<nlohmann::json> kept;

        for( nlohmann::json& v : violations )
        {
            FOOTPRINT* fp = nullptr;
            bool       one = !v.value( "items", nlohmann::json::array() ).empty();

            for( const nlohmann::json& item : v.value( "items", nlohmann::json::array() ) )
            {
                auto it = owner.find( item.value( "uuid", std::string() ) );

                if( it == owner.end() || ( fp && fp != it->second ) )
                {
                    one = false;
                    break;
                }

                fp = it->second;
            }

            if( one && ( refs.empty() || refs.count( fp->GetReference() ) ) )
                excluded.push_back( { { "ref", fp->GetReference().ToStdString() }, { "type", v.value( "type", std::string() ) } } );
            else
                kept.push_back( std::move( v ) );
        }

        violations.swap( kept );
    }

    std::vector<nlohmann::json> unrouted;

    for( const nlohmann::json& v : full.value( "unconnected_items", nlohmann::json::array() ) )
        unrouted.push_back( v );

    nlohmann::json result = KopenapiCheckResult( violations, unrouted, full.value( "ignored_checks", nlohmann::json::array() ), aArgs );

    if( !excluded.empty() )
        result["excluded_within_footprint"] = excluded;

    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "pcb_drc",
                   "Run DRC (design rules check) on the open board with KiCad's own engine, like "
                   "`kicad-cli pcb drc`: summary by severity and type, violations (type, severity, "
                   "description, items with uuid and position in mm), unconnected items, ignored checks; "
                   "filter by severity and type glob; paginated; refill_zones refills copper first "
                   "(changes the board). For schematic parity use design_parity",
                   KopenapiCheckSchema( R"json({
                        "refill_zones":{"type":"boolean","default":false,"description":"refill all zones first (modifies the board)"},
                        "all_track_errors":{"type":"boolean","default":true,"description":"report every track error (deterministic); false = one per track like the CLI default, which varies between runs"},
                        "exclude_within_footprint":{"oneOf":[{"type":"boolean"},{"type":"array","items":{"type":"string"}}],"default":false,"description":"leave out violations whose items all belong to one footprint (a library part against itself, e.g. pads near its own pegs): true for every footprint or a list of references; they are listed in excluded_within_footprint"}})json"_json ),
                   false, h_pcb_drc, 600 );
