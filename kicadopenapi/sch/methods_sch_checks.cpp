/// @file methods_sch_checks.cpp
/// @brief kicadopenapi schematic checks: sch_erc.
///
/// Runs KiCad's own ERC (ERC_TESTER, as `kicad-cli sch erc` does) on the live schematic and
/// returns the erc.v1 report KiCad writes (ERC_REPORT::GetJsonReport), plus a summary and a
/// flat, filterable, paginated violation list for agents.  In the GUI the markers appear in the
/// open editor, like running ERC from its dialog.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kicadopenapi_keepalive.h>
#include <erc/erc.h>
#include <erc/erc_item.h>
#include <erc/erc_report.h>
#include <erc/erc_settings.h>
#include <kicadopenapi_checks.h>
#include <kicadopenapi_libraries.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <project_sch.h>
#include <sch_edit_frame.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <schematic.h>
#include <libraries/symbol_library_adapter.h>

#include <map>


static KOPENAPI_RESULT h_sch_erc( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    int severities = 0;

    if( !KopenapiSeverityMask( aArgs, severities ) )
        return KOPENAPI_RESULT::Error( 400, "severity: list of error, warning, exclusion" );

    SCHEMATIC* schematic = context->GetSchematic();

    // ERC needs the symbol libraries (library symbol mismatch checks)
    SYMBOL_LIBRARY_ADAPTER* adapter = PROJECT_SCH::SymbolLibAdapter( &schematic->Project() );
    adapter->AsyncLoad();
    KopenapiWaitLibraries( adapter );

    // ...and the footprint libraries (footprint link checks go through cvpcb)
    KopenapiEnsureFootprintLibraries( aCtx.kiway );

    // GUI: the editor showing this schematic gets the markers, as with its own ERC dialog
    SCH_EDIT_FRAME* frame = nullptr;

    if( !aCtx.headless && aCtx.kiway )
    {
        frame = static_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) );

        if( frame && &frame->Schematic() != schematic )
            frame = nullptr;
    }

    kopenapi_sch::ensureConnectivity( schematic );

    // The editor only drops its selection (it may hold old markers); the run itself is the
    // frame-less one in both modes, so GUI and headless ERC never differ (the frame run also
    // cleans up wires)
    if( frame )
        frame->GetToolManager()->RunAction( ACTIONS::selectionClear );

    KOPENAPI_KEEPALIVE_REPORTER reporter;
    ERC_TESTER tester( schematic );
    tester.RunTests( nullptr, nullptr, aCtx.kiway ? aCtx.kiway->KiFACE( KIWAY::FACE_CVPCB ) : nullptr,
                     &schematic->Project(), &reporter );

    if( frame )
        frame->RefreshErcMarkers();

    auto markers = std::make_shared<SHEETLIST_ERC_ITEMS_PROVIDER>( schematic );
    markers->SetSeverities( severities );

    ERC_REPORT     report( schematic, EDA_UNITS::MM, markers );
    nlohmann::json full = report.GetJsonReport();

    // Flatten: one row per violation, with the sheet it is on
    std::vector<nlohmann::json> violations;

    for( const nlohmann::json& sheet : full.value( "sheets", nlohmann::json::array() ) )
    {
        for( nlohmann::json v : sheet.value( "violations", nlohmann::json::array() ) )
        {
            v["sheet"] = sheet.value( "path", std::string() );
            violations.push_back( std::move( v ) );
        }
    }

    nlohmann::json result = KopenapiCheckResult( violations, {}, full.value( "ignored_checks", nlohmann::json::array() ), aArgs );

    // The usual fixes for the usual findings, so an agent need not know KiCad's conventions
    static const std::map<std::string, std::string> hints = {
        { "power_pin_not_driven",
          "a power input pin's net has no driver: if the rail comes in through a connector (passive pins), "
          "place power:PWR_FLAG on that net once (sch_symbol_add, then connect it to the rail)" },
        { "pin_not_connected", "intentionally unused pins: sch_no_connect" },
        { "pin_not_driven", "an input pin's net has no output driving it: check the connection or the pin types" },
        { "label_dangling", "a label touches no wire or pin: move it onto a wire end / pin end (sch_label_add answers the net)" },
        { "unconnected_wire_endpoint", "a wire end touches nothing: sch_item_list types [wire] near the point, delete or extend it" },
        { "pin_to_pin", "two pins of incompatible types share a net: often fine for passives / power outputs on one rail; "
                        "check the pin types in the symbol cards" } };

    nlohmann::json advice = nlohmann::json::array();

    for( const auto& [type, count] : result["summary"]["by_type"].items() )
    {
        if( auto hint = hints.find( type ); hint != hints.end() )
            advice.push_back( { { "type", type }, { "hint", hint->second } } );
    }

    if( !advice.empty() )
        result["hints"] = advice;

    // symbol pins <-> library footprint pads (sch_footprint_check), problems only
    if( aArgs.value( "footprint_check", true ) )
    {
        if( std::optional<KOPENAPI_METHOD> check = KOPENAPI_REGISTRY::Get().Find( "sch_footprint_check" ) )
        {
            KOPENAPI_RESULT parts = check->handler( aCtx, { { "problems_only", true }, { "limit", 50 } } );

            if( parts.status == 200 )
            {
                result["footprint_check"] = { { "parts", parts.body["parts"] },
                                              { "problems", parts.body["problems"] },
                                              { "items", parts.body.value( "items", nlohmann::json::array() ) } };
            }
        }
    }

    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "sch_erc",
                   "Run ERC (electrical rules check) on the open schematic with KiCad's own checker, like "
                   "`kicad-cli sch erc`: summary by severity and type, violations (type, severity, "
                   "description, sheet, items with uuid and position in mm), ignored checks; filter by "
                   "severity and type glob; paginated. GUI: markers appear in the editor. The symbol "
                   "libraries are read as they are on disk now (lib_symbol_mismatch); footprint_check: "
                   "parts whose symbol pins find no pad on the library footprint, or whose footprint is "
                   "missing (sch_footprint_check, problems only)",
                   KopenapiCheckSchema( R"json({
                        "footprint_check":{"type":"boolean","default":true,"description":"include sch_footprint_check problems"}})json"_json ),
                   false, h_sch_erc, 600 );
