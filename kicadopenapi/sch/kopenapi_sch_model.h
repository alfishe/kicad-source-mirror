/*
 * kicadopenapi schematic model helpers shared by the sch_* methods (eeschema kiface):
 * nets from the same source KiCad's netlist exporter uses, pins per sheet instance.
 */
#ifndef KOPENAPI_SCH_MODEL_H
#define KOPENAPI_SCH_MODEL_H

#include <string>
#include <utility>
#include <vector>

#include <json_common.h>
#include <core/typeinfo.h>
#include <sch_sheet_path.h>
#include <wx/string.h>

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

nlohmann::json pinJson( const SCH_PIN* aPin, const SCH_SHEET_PATH& aPath );

/// All nets, natural name order, code = 1-based position
std::vector<NET_ENTRY> collectNets( SCHEMATIC* aSchematic );

/// Pins of a net, one per REF.PIN (multi-unit symbols repeat shared pins per unit)
std::vector<std::pair<const SCH_PIN*, SCH_SHEET_PATH>> netPins( const NET_ENTRY& aNet );

} // namespace kopenapi_sch

#endif
