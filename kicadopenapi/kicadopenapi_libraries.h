/*
 * kicadopenapi helpers shared by the library methods (sch_lib_list, pcb_lib_list).
 */
#ifndef KICADOPENAPI_LIBRARIES_H
#define KICADOPENAPI_LIBRARIES_H

#include <json_common.h>
#include <kicommon.h>

class KIWAY;
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

/**
 * Make sure the footprint libraries of the current project are loaded (from any kiface; the
 * adapter lives in the shared library manager).  A project switch aborts running loads but
 * leaves a per-library status behind, and cvpcb's lookups (ERC footprint links, netlist pad
 * resolution) then skip loading: every footprint looks missing (seen: 161 vs 3 ERC
 * footprint_link_issues after a DRC in the previous project).
 *
 * With aKiway, the footprint adapter is first created by pcbnew if nobody has created it yet:
 * it is one object per process, made by whichever kiface asks first, and its footprint cache
 * lives in statics of that kiface's copy of pcbcommon.  Made by cvpcb (ERC), pcbnew's library
 * search would see empty libraries.
 */
KICOMMON_API void KopenapiEnsureFootprintLibraries( KIWAY* aKiway = nullptr );

#endif
