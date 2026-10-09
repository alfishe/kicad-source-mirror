/*
 * kicadopenapi PCB access (compiled into the pcbnew kiface).
 *
 * One board per process.  GUI hosts reach the board through the PCB editor frame, headless
 * hosts through a HEADLESS_PCB_CONTEXT owned here; both behind PCB_CONTEXT, so every pcb_*
 * method has a single code path.
 */
#ifndef KOPENAPI_PCB_H
#define KOPENAPI_PCB_H

#include <memory>

#include <kicadopenapi_registry.h>

class PCB_CONTEXT;


/// The open board's context, or nullptr when no board is open
std::shared_ptr<PCB_CONTEXT> KopenapiPcbContext( KOPENAPI_CONTEXT& aCtx );

/// 409 "no board open" result for methods that need one
KOPENAPI_RESULT KopenapiNoBoard();

#endif
