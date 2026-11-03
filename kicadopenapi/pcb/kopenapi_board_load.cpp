#include "kopenapi_board_load.h"

#include <board.h>
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_keepalive.h>
#include <model_thread.h>
#include <pcb_io/pcb_io.h>
#include <wx/log.h>
#include <wx/thread.h>


KOPENAPI_BOARD_QUERIES& KopenapiBoardQueries()
{
    static KOPENAPI_BOARD_QUERIES s_queries;
    return s_queries;
}


void KopenapiAnswerQueries( PCB_IO& aPlugin, KOPENAPI_BOARD_QUERIES& aQueries )
{
    aPlugin.SetQueryUserCallback(
            [&aQueries]( wxString aTitle, int, wxString aMessage, wxString ) -> bool
            {
                const std::string question( ( aTitle + wxS( ": " ) + aMessage ).ToUTF8() );

                if( !aQueries.accept )
                {
                    if( aQueries.declined.empty() )
                        aQueries.declined = question;

                    wxLogWarning( "%s: %s (answered no, loading stops)", aTitle, aMessage );
                    return false;
                }

                aQueries.asked.push_back( question );
                wxLogWarning( "%s: %s", aTitle, aMessage );
                return true;
            } );
}


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

        KopenapiAnswerQueries( aPlugin, KopenapiBoardQueries() );
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
