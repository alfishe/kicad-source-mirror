/// @file kopenapi_board_load.h
/// @brief The board parse of PCB_EDIT_FRAME::OpenProjectFiles off the GUI main thread during an
/// API call (pcb_open, pcb_revert), so the main loop keeps painting while the file is read.
#pragma once

#include <board_loader.h>

#include <memory>


/// @brief BOARD_LOADER::Load as OpenProjectFiles calls it.  During an API call or job, on the main
/// thread, a KiCad board file is parsed on a worker while the UI paints (no progress bar; a question
/// the parser would ask in a dialog is answered "go on" and logged), its zone fills tessellated there
/// too; otherwise right here.  The board parsed is not shared with anyone until it is returned.
/// Rethrows what the loader throws
std::unique_ptr<BOARD> KopenapiLoadBoard( const wxString& aFileName, PCB_IO_MGR::PCB_FILE_T aFormat,
                                          PROJECT* aProject, const BOARD_LOADER::OPTIONS& aOptions );
