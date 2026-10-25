#include "kopenapi_board_load.h"

#include <board.h>
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_keepalive.h>
#include <model_thread.h>
#include <pcb_io/pcb_io.h>
#include <wx/log.h>
#include <wx/thread.h>


std::unique_ptr<BOARD> KopenapiLoadBoard( const wxString& aFileName, PCB_IO_MGR::PCB_FILE_T aFormat,
                                          PROJECT* aProject, const BOARD_LOADER::OPTIONS& aOptions )
{
    if( aFormat != PCB_IO_MGR::KICAD_SEXP || !wxThread::IsMain() || !KopenapiCallActive() )
        return BOARD_LOADER::Load( aFileName, aFormat, aProject, aOptions );

    BOARD_LOADER::OPTIONS options = aOptions;
    options.progress_reporter = nullptr;   // a dialog's reporter: main thread only
    options.plugin_configurator = [&aOptions]( PCB_IO& aPlugin )
    {
        if( aOptions.plugin_configurator )
            aOptions.plugin_configurator( aPlugin );

        aPlugin.SetQueryUserCallback(
                []( wxString aTitle, int, wxString aMessage, wxString ) -> bool
                {
                    wxLogWarning( "%s: %s (loading goes on)", aTitle, aMessage );
                    return true;
                } );
    };

    std::unique_ptr<BOARD> board;
    KopenapiRunOffMain(
            [&]()
            {
                // the board is this thread's alone until it returns: it writes it freely
                MODEL_THREAD::SCOPE own( MODEL_THREAD::KIND::WRITER );

                board = BOARD_LOADER::Load( aFileName, aFormat, aProject, options );

                // the zone fills the editor tessellates when it shows the board, done here first
                if( board )
                    board->CacheTriangulation();
            } );
    return board;
}
