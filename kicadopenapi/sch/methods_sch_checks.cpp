/*
 * kicadopenapi schematic checks: sch_erc.
 *
 * Runs KiCad's own ERC (ERC_TESTER, as `kicad-cli sch erc` does) on the live schematic and
 * returns the erc.v1 report KiCad writes (ERC_REPORT::GetJsonReport), plus a summary and a
 * flat, filterable, paginated violation list for agents.  In the GUI the markers appear in the
 * open editor, like running ERC from its dialog.
 */
#include "kopenapi_sch.h"

#include <api/sch_context.h>
#include <erc/erc.h>
#include <erc/erc_item.h>
#include <erc/erc_report.h>
#include <erc/erc_settings.h>
#include <kicadopenapi_checks.h>
#include <kiway.h>
#include <project_sch.h>
#include <sch_edit_frame.h>
#include <schematic.h>
#include <libraries/symbol_library_adapter.h>


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
    adapter->BlockUntilLoaded();

    // GUI: the editor showing this schematic gets the markers, as with its own ERC dialog
    SCH_EDIT_FRAME* frame = nullptr;

    if( !aCtx.headless && aCtx.kiway )
    {
        frame = static_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) );

        if( frame && &frame->Schematic() != schematic )
            frame = nullptr;
    }

    if( frame )
        frame->ClearErcMarkers();

    ERC_TESTER tester( schematic );
    tester.RunTests( nullptr, frame, aCtx.kiway ? aCtx.kiway->KiFACE( KIWAY::FACE_CVPCB ) : nullptr,
                     &schematic->Project(), nullptr );

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

    return KOPENAPI_RESULT::Ok( KopenapiCheckResult( violations, {}, full.value( "ignored_checks", nlohmann::json::array() ),
                                                     aArgs ) );
}


KOPENAPI_REGISTER( "sch_erc",
                   "Run ERC (electrical rules check) on the open schematic with KiCad's own checker, like "
                   "`kicad-cli sch erc`: summary by severity and type, violations (type, severity, "
                   "description, sheet, items with uuid and position in mm), ignored checks; filter by "
                   "severity and type glob; paginated. GUI: markers appear in the editor",
                   KopenapiCheckSchema(), false, h_sch_erc, 600 );
