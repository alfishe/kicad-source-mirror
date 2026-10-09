/*
 * kicadopenapi schematic access (compiled into the eeschema kiface).
 *
 * One schematic per process.  GUI hosts reach it through the schematic editor frame,
 * headless hosts through a HEADLESS_SCH_CONTEXT owned here; both behind SCH_CONTEXT.
 */
#ifndef KOPENAPI_SCH_H
#define KOPENAPI_SCH_H

#include <memory>

#include <kicadopenapi_registry.h>

class SCH_CONTEXT;


/// The open schematic's context, or nullptr when no schematic is open
std::shared_ptr<SCH_CONTEXT> KopenapiSchContext( KOPENAPI_CONTEXT& aCtx );

/// 409 "no schematic open" result for methods that need one
KOPENAPI_RESULT KopenapiNoSchematic();

#endif
