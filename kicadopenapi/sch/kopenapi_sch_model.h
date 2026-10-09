/*
 * kicadopenapi schematic model helpers shared by the sch_* methods (eeschema kiface):
 * nets from the same source KiCad's netlist exporter uses, pins per sheet instance.
 */
#ifndef KOPENAPI_SCH_MODEL_H
#define KOPENAPI_SCH_MODEL_H

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <json_common.h>
#include <core/typeinfo.h>
#include <sch_sheet_path.h>
#include <wx/string.h>

class KIWAY;
class PROJECT;
class SCHEMATIC;
class SCH_ITEM;
class SCH_PIN;
class SCH_SHEET_PIN;
class SCH_SYMBOL;

namespace kopenapi_sch
{

/// One sheet instance of a net
struct NET_INSTANCE
{
    SCH_SHEET_PATH         path;
    std::vector<SCH_ITEM*> items;
    std::string            localName;
};


/// A net with its instances, from the same source KiCad's netlist exporter uses
struct NET_ENTRY
{
    std::string               name;
    int                       code = 0;   ///< 1-based, in name order; valid for this model state
    std::vector<NET_INSTANCE> instances;
};


std::string str( const wxString& aText );

/// "local" / "global" / "hierarchical" / "directive", or nullptr for non-labels
const char* labelKind( KICAD_T aType );

const char* portDirection( const SCH_SHEET_PIN* aPin );

std::string sheetPath( const SCH_SHEET_PATH& aPath );

SCH_SYMBOL* pinSymbol( const SCH_PIN* aPin );

/// REF.PIN of a pin on a sheet instance
std::string pinId( const SCH_PIN* aPin, const SCH_SHEET_PATH& aPath );

/**
 * Footprint pads of symbol pins, resolved exactly like KiCad's netlist exporter
 * (NETLIST_EXPORTER_BASE::resolvePadNumbers): the symbol's footprint is looked up through the
 * cvpcb kiface and the pin's pad map applied.  A pin without a pad in its footprint is
 * "unmapped" and the exporter leaves it out of the netlist.  One instance per call (caches
 * footprint pad sets).
 */
class PAD_RESOLVER
{
public:
    PAD_RESOLVER( KIWAY* aKiway, PROJECT* aProject ) : m_kiway( aKiway ), m_project( aProject ) {}

    /// aStatus: mapped | unmapped | no_footprint | footprint_not_found
    std::vector<std::string> Resolve( const SCH_PIN* aPin, const SCH_SHEET_PATH& aPath, std::string& aStatus );

private:
    const std::set<wxString>& footprintPads( const wxString& aFootprintId );

    KIWAY*                                  m_kiway;
    PROJECT*                                m_project;
    std::map<wxString, std::set<wxString>> m_cache;
};

/// Pin card; with a resolver also its footprint pads and pad_status
nlohmann::json pinJson( const SCH_PIN* aPin, const SCH_SHEET_PATH& aPath, PAD_RESOLVER* aPads = nullptr );

/// All nets, natural name order, code = 1-based position
std::vector<NET_ENTRY> collectNets( SCHEMATIC* aSchematic );

/**
 * Role of a net: name heuristics (KopenapiNetRoleFromName) plus graph facts — a power symbol on
 * a net with a signal-like name makes it "power".  Hidden power_in pins alone do not (designs
 * converted from other tools put them on every package).
 */
std::string netRole( const NET_ENTRY& aNet );

/// Pins of a net, one per REF.PIN (multi-unit symbols repeat shared pins per unit)
std::vector<std::pair<const SCH_PIN*, SCH_SHEET_PATH>> netPins( const NET_ENTRY& aNet );

} // namespace kopenapi_sch

#endif
