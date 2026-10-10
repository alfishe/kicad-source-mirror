/// @file kopenapi_pcb.h
/// @brief kicadopenapi PCB access (compiled into the pcbnew kiface).
///
/// One board per process.  GUI hosts reach the board through the PCB editor frame, headless
/// hosts through a HEADLESS_PCB_CONTEXT owned here; both behind PCB_CONTEXT, so every pcb_*
/// method has a single code path.
#ifndef KOPENAPI_PCB_H
#define KOPENAPI_PCB_H

#include <memory>
#include <string>

#include <kicadopenapi_registry.h>

class FOOTPRINT;
class PCB_CONTEXT;


/// @brief The open board's context, or nullptr when no board is open
std::shared_ptr<PCB_CONTEXT> KopenapiPcbContext( KOPENAPI_CONTEXT& aCtx );

/// @brief 409 "no board open" result for methods that need one
KOPENAPI_RESULT KopenapiNoBoard();

/// @brief An edge connector's panel line for footprint cards.
/// @param aMethod auto (marker, else inferred from the body), marker, infer
/// @return line ends, facing (left/right/up/down or degrees), depth behind the line, overhang
/// past it, source, confidence; null when no edge is known
nlohmann::json KopenapiPanelEdgeJson( FOOTPRINT* aFootprint, const std::string& aMethod = "auto" );

#endif
