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

#endif
