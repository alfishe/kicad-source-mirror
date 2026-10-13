/// @file kopenapi_pcb_query.h
/// @brief pcb_query: board items selected by a KiCad DRC condition, shaped into rows / groups.
///
/// Conditions use KiCad's own expression engine (LIBEVAL + the DRC property bindings) with a few
/// query-only functions (A.side(), A.mount(), A.inRect(...), ...); read-only by construction.
/// Design: docs/inprogress/2026-10-12-project-query/design.md.
#ifndef KOPENAPI_PCB_QUERY_H
#define KOPENAPI_PCB_QUERY_H

#include <kicadopenapi_registry.h>

class BOARD;

/// @brief Run one query against a board (main thread).
/// @param aArgs from, where, select, group_by, aggregate, order_by, limit, cursor, explain,
/// deadline_ms (see the method's schema)
KOPENAPI_RESULT KopenapiPcbQuery( BOARD* aBoard, const nlohmann::json& aArgs );

#endif // KOPENAPI_PCB_QUERY_H
