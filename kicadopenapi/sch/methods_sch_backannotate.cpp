/// @file methods_sch_backannotate.cpp
/// @brief kicadopenapi Update Schematic from PCB: the board's netlist (pcb_netlist_kicad) applied to
/// the open schematic, as the editor's Tools > Update Schematic from PCB
#include "kopenapi_sch.h"

#include <api/sch_context.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>
#include <reporter.h>
#include <sch_sheet_path.h>
#include <schematic.h>
#include <tool/tool_manager.h>
#include <tools/backannotate.h>

#include <vector>


namespace
{

/// @brief The messages BACK_ANNOTATE reports, in order, with their severity
class JSON_REPORTER : public REPORTER
{
public:
    REPORTER& Report( const wxString& aText, SEVERITY aSeverity = RPT_SEVERITY_UNDEFINED ) override
    {
        REPORTER::Report( aText, aSeverity );

        const char* severity = aSeverity == RPT_SEVERITY_ERROR     ? "error"
                               : aSeverity == RPT_SEVERITY_WARNING ? "warning"
                               : aSeverity == RPT_SEVERITY_ACTION  ? "action"
                                                                   : "info";

        m_messages.push_back( { { "severity", severity }, { "text", aText.ToStdString( wxConvUTF8 ) } } );
        return *this;
    }

    const std::vector<nlohmann::json>& Messages() const { return m_messages; }

private:
    std::vector<nlohmann::json> m_messages;
};


KOPENAPI_RESULT h_sch_update_from_pcb( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "netlist" ) || !aArgs["netlist"].is_string() || aArgs["netlist"].get<std::string>().empty() )
        return KOPENAPI_RESULT::Error( 400, "netlist: the board's netlist from pcb_netlist_kicad (or use design_update_schematic)" );

    const bool     dryRun = aArgs.value( "dry_run", false );
    JSON_REPORTER  reporter;
    BACK_ANNOTATE  backAnno( context->GetSchematic(), context->GetToolManager(), reporter, false,
                             aArgs.value( "footprints", true ), aArgs.value( "values", true ),
                             aArgs.value( "references", true ), aArgs.value( "net_names", true ),
                             aArgs.value( "attributes", true ), aArgs.value( "other_fields", true ),
                             aArgs.value( "prefer_unit_swaps", true ), aArgs.value( "prefer_pin_swaps", true ),
                             dryRun );

    bool ran = false;

    try
    {
        ran = backAnno.BackAnnotateSymbols( aArgs["netlist"].get<std::string>() );
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 422, "netlist: " + ioe.What().ToStdString( wxConvUTF8 ) );
    }

    if( !ran )
        return KOPENAPI_RESULT::Error( 400, "nothing to update: every property is off" );

    if( !dryRun )
    {
        for( SCH_SHEET_PATH& sheet : context->GetSchematic()->Hierarchy() )
            sheet.UpdateAllScreenReferences();

        context->GetSchematic()->RebuildConnectivity();
    }

    const size_t   limit = aArgs.value( "list_limit", 200 );
    nlohmann::json messages = nlohmann::json::array();
    int            errors = 0, warnings = 0;

    for( const nlohmann::json& m : reporter.Messages() )
    {
        errors += m["severity"] == "error";
        warnings += m["severity"] == "warning";

        if( aArgs.value( "full", false ) || messages.size() < limit )
            messages.push_back( m );
    }

    return KOPENAPI_RESULT::Ok( { { "changes", backAnno.GetChangesCount() },
                                  { "errors", errors },
                                  { "warnings", warnings },
                                  { "messages", messages },
                                  { "messages_total", reporter.Messages().size() },
                                  { "dry_run", dryRun } } );
}

} // namespace


KOPENAPI_REGISTER( "sch_update_from_pcb",
                   "Update Schematic from PCB with a board netlist given (pcb_netlist_kicad; design_update_schematic "
                   "does both): symbols matched to footprints by their path; references, footprints, values, "
                   "net_names (pad net changes as label renames or, preferred, symbol unit / pin swaps), "
                   "attributes (dnp, exclusions) and other_fields, each default true as in the editor. Answers "
                   "the number of changes and the editor's report (severity, text; list_limit default 200, full); "
                   "dry_run previews. One undo step",
                   R"json({"type":"object","required":["netlist"],"properties":{
                        "netlist":{"type":"string"},
                        "references":{"type":"boolean","default":true},"footprints":{"type":"boolean","default":true},
                        "values":{"type":"boolean","default":true},"net_names":{"type":"boolean","default":true},
                        "attributes":{"type":"boolean","default":true},"other_fields":{"type":"boolean","default":true},
                        "prefer_unit_swaps":{"type":"boolean","default":true},
                        "prefer_pin_swaps":{"type":"boolean","default":true},
                        "dry_run":{"type":"boolean","default":false},
                        "list_limit":{"type":"integer","default":200},"full":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_update_from_pcb );

KOPENAPI_MARK_EDITING( "sch_update_from_pcb" );
