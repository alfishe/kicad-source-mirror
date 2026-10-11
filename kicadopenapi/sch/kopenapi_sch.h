/// @file kopenapi_sch.h
/// @brief kicadopenapi schematic access (compiled into the eeschema kiface).
///
/// One schematic per process.  GUI hosts reach it through the schematic editor frame,
/// headless hosts through a HEADLESS_SCH_CONTEXT owned here; both behind SCH_CONTEXT.
#ifndef KOPENAPI_SCH_H
#define KOPENAPI_SCH_H

#include <memory>
#include <string>

#include <kicadopenapi_registry.h>

class SCH_CONTEXT;


/// @brief The open schematic's context, or nullptr when no schematic is open
std::shared_ptr<SCH_CONTEXT> KopenapiSchContext( KOPENAPI_CONTEXT& aCtx );

/// @brief 409 "no schematic open" result for methods that need one
KOPENAPI_RESULT KopenapiNoSchematic();

class SCH_EDIT_FRAME;
class SCH_ITEM;

/// @brief An item of the shown sheet by uuid, a symbol by reference, a pin as REF.NUMBER; nullptr if none
SCH_ITEM* KopenapiFindSchematicItem( SCH_EDIT_FRAME* aFrame, const std::string& aRef );

#endif
