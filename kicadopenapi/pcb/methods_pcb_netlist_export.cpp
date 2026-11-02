/// @file methods_pcb_netlist_export.cpp
/// @brief pcb_export_netlist: the board editor's File > Export > Netlist (board netlist file), on the core
/// function the editor calls.

#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_netlist_export.h>
#include <ctl_flags.h>
#include <wx/filename.h>


namespace
{

KOPENAPI_RESULT h_pcb_export_netlist( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*     board = context->GetBoard();
    wxFileName fn;

    if( aArgs.contains( "path" ) && aArgs["path"].is_string() )
    {
        fn = wxFileName( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
    }
    else
    {
        // the editor proposes <board>.pcb_net beside the board
        fn = wxFileName( board->GetFileName() );
        fn.SetExt( wxT( "pcb_net" ) );
    }

    if( !fn.IsAbsolute() || fn.GetFullName().IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "path: an absolute file path (default: the board's .pcb_net)" );

    int ctl = 0;

    if( aArgs.value( "omit_extras", false ) )
        ctl |= CTL_OMIT_EXTRA;

    if( aArgs.value( "omit_nets", false ) )
        ctl |= CTL_OMIT_NETS;

    if( aArgs.value( "omit_fp_uuids", false ) )
        ctl |= CTL_OMIT_FP_UUID;

    if( wxString error = ExportBoardNetlist( *board, fn.GetFullPath(), ctl ); !error.IsEmpty() )
        return KOPENAPI_RESULT::Error( 422, error.ToStdString( wxConvUTF8 ) );

    return KOPENAPI_RESULT::Ok( { { "path", fn.GetFullPath().ToStdString( wxConvUTF8 ) },
                                  { "footprints", board->Footprints().size() } } );
}

} // namespace


KOPENAPI_REGISTER( "pcb_export_netlist",
                   "File > Export > Netlist of the board editor: each footprint with its pad nets and fields as a "
                   "KiCad board netlist file (path, default the board's .pcb_net; overwritten). omit_extras, "
                   "omit_nets, omit_fp_uuids: the dialog's options. An unwritable folder or a write error is 422 "
                   "with the editor's message",
                   R"json({"type":"object","properties":{
                        "path":{"type":"string"},
                        "omit_extras":{"type":"boolean","default":false},
                        "omit_nets":{"type":"boolean","default":false},
                        "omit_fp_uuids":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_export_netlist );

KOPENAPI_MARK_CONCURRENT_READS( "pcb_export_netlist" );
