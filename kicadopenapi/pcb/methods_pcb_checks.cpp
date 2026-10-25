/// @file methods_pcb_checks.cpp
/// @brief kicadopenapi board checks: pcb_drc.
///
/// Runs KiCad's own DRC engine (as `kicad-cli pcb drc` does) on the live board and returns the
/// drc.v1 report KiCad writes (DRC_REPORT::GetJsonReport), plus a summary and a flat,
/// filterable, paginated violation list for agents.  Markers land on the board (in the GUI
/// they show in the editor), without an undo step and without marking the board modified;
/// zones are refilled only when asked (that does change the board).
#include "kopenapi_pcb.h"

#include <atomic>
#include <map>
#include <mutex>
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
#include <kicadopenapi_jobs.h>
#include <pcb_marker.h>
#include <project_pcb.h>
#include <tool/tool_manager.h>
#include <tools/zone_filler_tool.h>
#include <zone.h>

#include <wx/filename.h>
#include <wx/log.h>


bool KopenapiDesignRulesFromDisk( BOARD* aBoard, std::string* aError )
{
    struct STAMP
    {
        wxString   path;
        bool       exists = false;
        wxLongLong size = 0;
        wxDateTime modified;

        bool operator==( const STAMP& aOther ) const
        {
            return path == aOther.path && exists == aOther.exists && size == aOther.size
                   && ( !exists || modified == aOther.modified );
        }
    };

    static std::mutex                     s_mutex;
    static std::map<const BOARD*, STAMP>  s_loaded;

    std::shared_ptr<DRC_ENGINE> engine = aBoard ? aBoard->GetDesignSettings().m_DRCEngine : nullptr;

    if( !engine )
        return true;

    STAMP      now;
    wxFileName file( aBoard->GetDesignRulesPath() );
    now.path = file.GetFullPath();
    now.exists = file.FileExists();

    if( now.exists )
    {
        now.size = file.GetSize();
        now.modified = file.GetModificationTime();
    }

    std::lock_guard<std::mutex> lock( s_mutex );
    auto                        it = s_loaded.find( aBoard );

    if( it != s_loaded.end() && it->second == now && engine->RulesValid() )
        return true;

    KOPENAPI_KEEP_UNSAVED keep;

    try
    {
        engine->InitEngine( file );
    }
    catch( const PARSE_ERROR& error )
    {
        if( aError )
            *aError = "custom design rules do not compile: " + error.What().ToStdString( wxConvUTF8 );

        return false;
    }

    if( it != s_loaded.end() )
        wxLogMessage( "kicadopenapi: design rules %s changed on disk: read again", now.path );

    s_loaded[aBoard] = now;
    return true;
}


namespace
{

/// @brief The DRC engine's log: which test provider runs now (any thread)
class DRC_PROVIDER_WATCH : public REPORTER
{
public:
    using REPORTER::Report;

    REPORTER& Report( const wxString& aText, SEVERITY ) override
    {
        static const wxString run = wxS( "Run DRC provider: '" );

        if( aText.StartsWith( run ) )
        {
            std::lock_guard<std::mutex> lock( m_mutex );
            m_provider = aText.Mid( run.length() ).BeforeLast( '\'' ).ToStdString( wxConvUTF8 );
            m_index++;
        }

        return *this;
    }

    std::pair<std::string, int> Current() const
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        return { m_provider, m_index };
    }

private:
    mutable std::mutex m_mutex;
    std::string        m_provider;
    int                m_index = 0;
};

} // namespace


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
    KopenapiFootprintLibraries( board->GetProject() );

    std::shared_ptr<DRC_ENGINE> engine = board->GetDesignSettings().m_DRCEngine;

    if( std::string error; !KopenapiDesignRulesFromDisk( board, &error ) )
        return KOPENAPI_RESULT::Error( 422, error );

    if( aArgs.value( "refill_zones", false ) )
    {
        if( !toolManager->FindTool( ZONE_FILLER_TOOL_NAME ) )
            toolManager->RegisterTool( new ZONE_FILLER_TOOL );

        KOPENAPI_PROGRESS_REPORTER fillReporter(
                []( const std::string& aMessage, int aPhase, double aFraction )
                {
                    return KopenapiJobProgress( "zones_fill " + std::to_string( aPhase ), (int) std::lround( aFraction * 1000 ),
                                                1000, aMessage, { { "step", aPhase } } );
                } );

        toolManager->GetTool<ZONE_FILLER_TOOL>()->FillAllZones( nullptr, &fillReporter, true );
    }

    // progress: the test provider at work, its share done, violations so far by severity (before
    // exclusions)
    std::vector<SEVERITY> severityOf( DRCE_LAST + 1, RPT_SEVERITY_ERROR );

    for( int code = DRCE_FIRST; code <= DRCE_LAST; ++code )
        severityOf[code] = board->GetDesignSettings().GetSeverity( code );

    const int                    providers = (int) engine->GetTestProviders().size();
    std::atomic<int>             errors{ 0 }, warnings{ 0 };
    DRC_PROVIDER_WATCH           watch;

    KOPENAPI_PROGRESS_REPORTER reporter(
            [&]( const std::string& aMessage, int, double aFraction )
            {
                const auto [provider, index] = watch.Current();
                const double fraction = providers > 0 ? std::clamp( ( std::max( index - 1, 0 ) + ( index > 0 ? aFraction : 0.0 ) )
                                                                    / providers, 0.0, 1.0 ) : 0.0;
                const nlohmann::json counts = { { "provider", provider },
                                                { "provider_index", index },
                                                { "providers", providers },
                                                { "fraction", std::round( fraction * 1000 ) / 1000 },
                                                { "violations", { { "error", errors.load() },
                                                                  { "warning", warnings.load() },
                                                                  { "total", errors.load() + warnings.load() } } } };

                return KopenapiJobProgress( index > 0 ? "drc " + provider : std::string( "drc setup" ),
                                            (int) std::lround( aFraction * 1000 ), 1000, aMessage, counts );
            } );

    BOARD_COMMIT commit( toolManager );

    engine->SetViolationHandler(
            [&]( const std::shared_ptr<DRC_ITEM>& aItem, const VECTOR2I& aPos, int aLayer,
                 const std::function<void( PCB_MARKER* )>& aPathGenerator )
            {
                PCB_MARKER* marker = new PCB_MARKER( aItem, aPos, aLayer );
                aPathGenerator( marker );
                commit.Add( marker );

                const int      code = aItem->GetErrorCode();
                const SEVERITY severity = code >= 0 && code <= DRCE_LAST ? severityOf[code] : RPT_SEVERITY_ERROR;
                ( severity == RPT_SEVERITY_ERROR ? errors : warnings ) += severity == RPT_SEVERITY_IGNORE ? 0 : 1;
            } );

    board->RecordDRCExclusions();
    board->DeleteMARKERs( true, true );
    // Every track error by default: with one error per track KiCad keeps whichever segment its
    // threads report first, so repeated runs differ (seen on Sprinter DX: 2..4 clearance errors)
    engine->SetProgressReporter( &reporter );

    // The caches the tests read (connectivity, R-trees) are built here, by the owner; the tests
    // only read the board (off the GUI thread, readers read alongside); markers reach the board in
    // the commit below
    engine->SetLogReporter( &watch );

    if( engine->PrepareTests( EDA_UNITS::MM, aArgs.value( "all_track_errors", true ), false ) )
        KopenapiCompute( [&]() { engine->RunProviders( EDA_UNITS::MM ); } );

    engine->SetLogReporter( nullptr );

    engine->SetProgressReporter( nullptr );

    const bool stopped = reporter.IsCancelled();
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

    // the ratsnest's count beside DRC's unconnected_items (which leaves out excluded ones)
    const KOPENAPI_UNROUTED open = KopenapiUnrouted( board );
    result["summary"]["unrouted_connections"] = open.connections;
    result["summary"]["unrouted_nets"] = open.nets;
    result["summary"]["zone_fills"] = open.zoneFills;

    // a job stopped (job_cancel): the providers run so far, the rest not
    result["stopped"] = stopped;

    // 3D model files and their metadata (KiCad's DRC has no such test; the library side,
    // models included, is its lib_footprint_mismatch)
    if( aArgs.value( "model_checks", true ) )
    {
        nlohmann::json             items = nlohmann::json::array();
        std::map<std::string, int> counts;

        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( nlohmann::json& f : KopenapiModelFindings( board, fp ) )
            {
                if( f["severity"] == "info" )
                    continue;

                counts[f["kind"].get<std::string>()]++;
                f["ref"] = fp->GetReference().ToStdString( wxConvUTF8 );

                if( items.size() < 100 )
                    items.push_back( std::move( f ) );
            }
        }

        result["model_findings"] = { { "counts", counts }, { "items", items } };
    }

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_drc_exclude( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const nlohmann::json rules = aArgs.value( "exclusions", nlohmann::json::array() );

    if( !rules.is_array() || rules.empty() )
        return KOPENAPI_RESULT::Error( 400, "exclusions: [{type, refs, comment}] (a pcb_drc run first places the markers)" );

    for( const nlohmann::json& r : rules )
    {
        if( !r.is_object() || !r.contains( "type" ) || !r["type"].is_string() || !r.contains( "refs" ) || !r["refs"].is_array()
            || r["refs"].empty() )
        {
            return KOPENAPI_RESULT::Error( 400, "every exclusion: type (a pcb_drc violation type) and refs (the footprints it is about)" );
        }
    }

    BOARD*     board = context->GetBoard();
    const bool remove = aArgs.value( "remove", false );

    // the footprint an item is or belongs to
    auto ownerRef = [&]( const KIID& aId ) -> wxString
    {
        if( aId == niluuid )
            return wxEmptyString;

        BOARD_ITEM* item = board->ResolveItem( aId, true );

        if( !item )
            return wxEmptyString;

        if( FOOTPRINT* fp = dynamic_cast<FOOTPRINT*>( item ) )
            return fp->GetReference();

        if( FOOTPRINT* fp = item->GetParentFootprint() )
            return fp->GetReference();

        return wxEmptyString;
    };

    nlohmann::json changed = nlohmann::json::array();
    int            markers = 0;

    for( PCB_MARKER* marker : board->Markers() )
    {
        std::shared_ptr<RC_ITEM> rc = marker->GetRCItem();

        if( !rc || marker->GetMarkerType() != MARKER_BASE::MARKER_DRC )
            continue;

        markers++;
        const std::string type = rc->GetSettingsKey().ToStdString();
        const wxString    main = ownerRef( rc->GetMainItemID() );
        const wxString    aux = rc->GetAuxItemID() == niluuid ? main : ownerRef( rc->GetAuxItemID() );

        for( const nlohmann::json& r : rules )
        {
            if( r["type"].get<std::string>() != type || main.IsEmpty() || main != aux )
                continue;

            bool named = false;

            for( const nlohmann::json& ref : r["refs"] )
                named |= ref.is_string() && wxString::FromUTF8( ref.get<std::string>() ) == main;

            if( !named || marker->IsExcluded() != remove )
                continue;

            if( remove )
                marker->SetExcluded( false );
            else
                marker->SetExcluded( true, wxString::FromUTF8( r.value( "comment", std::string() ) ) );

            changed.push_back( { { "type", type }, { "ref", main.ToStdString() },
                                 { "at_mm", { pcbIUScale.IUTomm( marker->GetPos().x ), pcbIUScale.IUTomm( marker->GetPos().y ) } } } );
            break;
        }
    }

    if( !markers )
        return KOPENAPI_RESULT::Error( 409, "no DRC markers on the board: run pcb_drc first" );

    if( !changed.empty() )
    {
        board->RecordDRCExclusions();
        context->SetContentModified();
    }

    return KOPENAPI_RESULT::Ok( { { remove ? "included_again" : "excluded", changed },
                                  { "note", "exclusions live in the project file (pcb_save); pcb_drc lists them with severity exclusion" } } );
}


KOPENAPI_REGISTER( "pcb_drc",
                   "Run DRC (design rules check) on the open board with KiCad's own engine, like "
                   "`kicad-cli pcb drc`: summary by severity and type, violations (type, severity, "
                   "description, items with uuid and position in mm), unconnected items, ignored checks; "
                   "filter by severity and type glob; paginated; refill_zones refills copper first "
                   "(changes the board); the footprint libraries the board uses read again when their files changed "
                   "on disk (lib_footprint_mismatch against the current files, 3D models included); model_findings: 3D model "
                   "files not found and stale / mismatching <model>.meta.json (pcb_parts_check). For schematic parity use design_parity",
                   KopenapiCheckSchema( R"json({
                        "refill_zones":{"type":"boolean","default":false,"description":"refill all zones first (modifies the board)"},
                        "model_checks":{"type":"boolean","default":true,"description":"model_findings: 3D model files and metadata"},
                        "all_track_errors":{"type":"boolean","default":true,"description":"report every track error (deterministic); false = one per track like the CLI default, which varies between runs"},
                        "exclude_within_footprint":{"oneOf":[{"type":"boolean"},{"type":"array","items":{"type":"string"}}],"default":false,"description":"leave out violations whose items all belong to one footprint (a library part against itself, e.g. pads near its own pegs): true for every footprint or a list of references; they are listed in excluded_within_footprint"}})json"_json ),
                   false, h_pcb_drc, 600 );

KOPENAPI_REGISTER( "pcb_drc_exclude",
                   "Exclude DRC violations for good, as the DRC dialog's \"Exclude this violation\" (with a "
                   "comment): the markers of the last pcb_drc whose type matches and whose items all belong to "
                   "one of the named footprints (a part against itself, or the footprint as a whole, e.g. "
                   "footprint_type_mismatch of an SMD part with thermal-via pads); kept in the project; "
                   "remove: true includes them again",
                   R"json({"type":"object","required":["exclusions"],"properties":{
                        "exclusions":{"type":"array","items":{"type":"object","required":["type","refs"],"properties":{
                            "type":{"type":"string","description":"a pcb_drc violation type"},
                            "refs":{"type":"array","items":{"type":"string"}},
                            "comment":{"type":"string","description":"why it is not a problem"}}}},
                        "remove":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_drc_exclude );

KOPENAPI_MARK_EDITING( "pcb_drc_exclude" );
