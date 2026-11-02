/// @file kicadopenapi_netclasses.h
/// @brief kicadopenapi net classes in the project's net settings, shared by pcb_rules_set and the
/// schematic's sch_netclass_set / sch_netclass_list: one source, as Board Setup and Schematic Setup
/// edit the same classes.
#ifndef KICADOPENAPI_NETCLASSES_H
#define KICADOPENAPI_NETCLASSES_H

#include <json_common.h>
#include <kicommon.h>

#include <string>

class NETCLASS;
class NET_SETTINGS;


/// @brief Class values the board uses (mm): clearance_mm, track_width_mm, via_diameter_mm,
/// via_drill_mm, diff_pair_width_mm, diff_pair_gap_mm
KICOMMON_API const nlohmann::json& KopenapiNetclassBoardSchema();

/// @brief Class values the schematic uses: wire_width_mm, bus_width_mm, color (#rrggbb, "" for
/// none), line_style (solid, dash, dot, dash_dot, dash_dot_dot)
KICOMMON_API const nlohmann::json& KopenapiNetclassSchematicSchema();

/// @brief "" when aList (the netclasses argument: [{name, values, nets, delete}]) is valid, else why
KICOMMON_API std::string KopenapiNetclassesCheck( const nlohmann::json& aList );

/// @brief Apply a checked netclasses list: values set, a missing class created, nets (patterns)
/// replace the class's patterns and leave other classes, delete removes a class and its patterns
KICOMMON_API void KopenapiNetclassesApply( NET_SETTINGS& aSettings, const nlohmann::json& aList );

/// @brief A class with the values it sets (board ones; with aSchematic also the schematic ones)
KICOMMON_API nlohmann::json KopenapiNetclassJson( NETCLASS& aClass, bool aSchematic );

/// @brief Every class (Default first) and the pattern assignments {pattern, netclass}
KICOMMON_API nlohmann::json KopenapiNetclassesJson( NET_SETTINGS& aSettings, bool aSchematic );

#endif
