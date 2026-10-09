/*
 * kicadopenapi helpers shared by the library methods (sch_lib_list, pcb_lib_list).
 */
#ifndef KICADOPENAPI_LIBRARIES_H
#define KICADOPENAPI_LIBRARIES_H

#include <json_common.h>
#include <kicommon.h>

class LIBRARY_MANAGER_ADAPTER;


/**
 * The library tables behind an adapter — global and project — with their path, whether they
 * loaded, the error if not, and nested "Table" rows with their own status.  A broken table
 * otherwise only shows as "no libraries".  aErrors is incremented per broken table / row.
 */
KICOMMON_API nlohmann::json KopenapiLibraryTables( const LIBRARY_MANAGER_ADAPTER& aAdapter, int& aErrors );

/**
 * Warn (wxLogWarning -> the error journal) when KiCad's global symbol / footprint library
 * tables would leave this process without libraries: table file missing, unreadable or empty,
 * a nested "Table" row naming a missing file, or library paths (e.g. ${KICAD10_SYMBOL_DIR})
 * resolving to a missing directory.  Seen with development builds sharing one settings
 * directory: a table pointing into another build's bundle that no longer exists.
 */
KICOMMON_API void KopenapiCheckGlobalLibraryTables();

#endif
