/// @file kopenapi_board_load.h
/// @brief The board parse of PCB_EDIT_FRAME::OpenProjectFiles off the GUI main thread during an
/// API call (pcb_open, pcb_revert), so the main loop keeps painting while the file is read.
#pragma once

#include <board_loader.h>

#include <memory>
#include <string>
#include <vector>


/// @brief How a board read answers the reader's questions (items on undefined layers): accept says
/// yes and records each in asked; otherwise the first is declined (the read stops) and kept
struct KOPENAPI_BOARD_QUERIES
{
    bool                     accept = true;
    std::vector<std::string> asked;
    std::string              declined;
};


/// @brief The answers of the board reads of this API call (pcb_open accept_queries; yes by default)
KOPENAPI_BOARD_QUERIES& KopenapiBoardQueries();

/// @brief The plugin's questions answered as @p aQueries says, recorded there
void KopenapiAnswerQueries( PCB_IO& aPlugin, KOPENAPI_BOARD_QUERIES& aQueries );


/// @brief BOARD_LOADER::Load as OpenProjectFiles calls it.  During an API call or job, on the main
/// thread, a KiCad board file is parsed on a worker while the UI paints (no progress bar; a question
/// the parser would ask in a dialog is answered by KopenapiBoardQueries and logged), its zone fills tessellated there
/// too; otherwise right here.  The board parsed is not shared with anyone until it is returned.
/// Rethrows what the loader throws
std::unique_ptr<BOARD> KopenapiLoadBoard( const wxString& aFileName, PCB_IO_MGR::PCB_FILE_T aFormat,
                                          PROJECT* aProject, const BOARD_LOADER::OPTIONS& aOptions );
