/*
 * kicadopenapi schematic -> board: sch_netlist_kicad.
 *
 * The KiCad netlist of the open schematic, built in memory the way the editor builds it for
 * "Update PCB from Schematic" (NETLIST_EXPORTER_KICAD, GNL_ALL | GNL_OPT_KICAD), so the board can
 * follow unsaved schematic edits.  Consumed by design_update_board -> pcb_netlist_apply.
 */
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <netlist_exporters/netlist_exporter_kicad.h>
#include <richio.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>

using namespace kopenapi_sch;


static KOPENAPI_RESULT h_sch_netlist_kicad( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC* schematic = context->GetSchematic();

    // The board matches parts by reference: every part needs one
    nlohmann::json unannotated = nlohmann::json::array();

    for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( !symbol->IsPower() && !symbol->GetExcludedFromBoard() && !symbol->IsAnnotated( &path ) )
                unannotated.push_back( str( symbol->m_Uuid.AsString() ) );
        }
    }

    if( !unannotated.empty() )
        return KOPENAPI_RESULT::Error( 409, "parts without a reference (sch_annotate first): " + unannotated.dump() );

    schematic->RebuildConnectivity();

    NETLIST_EXPORTER_KICAD exporter( schematic, aCtx.kiway );
    STRING_FORMATTER       formatter;
    exporter.Format( &formatter, GNL_ALL | GNL_OPT_KICAD );

    return KOPENAPI_RESULT::Ok( { { "netlist", formatter.GetString() },
                                  { "schematic", str( schematic->Project().GetProjectFullName() ) } } );
}


KOPENAPI_REGISTER( "sch_netlist_kicad",
                   "KiCad netlist text (s-expression) of the open schematic, unsaved edits included, as "
                   "the editor builds it for Update PCB from Schematic; 409 when parts lack references. "
                   "To update the board use design_update_board",
                   R"json({"type":"object","properties":{}})json"_json, false, h_sch_netlist_kicad );
