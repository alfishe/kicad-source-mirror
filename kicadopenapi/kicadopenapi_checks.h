/// @file kicadopenapi_checks.h
/// @brief kicadopenapi helpers shared by the check methods (sch_erc, pcb_drc): severity selection,
/// input schema, and the agent-facing result (summary + filtered, paginated violations).
#ifndef KICADOPENAPI_CHECKS_H
#define KICADOPENAPI_CHECKS_H

#include <json_common.h>
#include <kicommon.h>

#include <vector>


/// @brief Severity mask (RPT_SEVERITY_*) from args["severity"] (list of "error", "warning",
/// "exclusion"); default error + warning.  False on an unknown name.
KICOMMON_API bool KopenapiSeverityMask( const nlohmann::json& aArgs, int& aMask );

/// @brief Input schema of a check method: severity, type glob, list limits, plus aExtra properties
KICOMMON_API nlohmann::json KopenapiCheckSchema( nlohmann::json aExtra = nlohmann::json::object() );

/// @brief { summary: {errors, warnings, excluded, by_type, unconnected_items}, violations: page,
///   unconnected: first items, ignored_checks }.  Violations are KiCad's report entries
/// (type, severity, description, items, excluded); filtered by args "type" glob, paginated.
KICOMMON_API nlohmann::json KopenapiCheckResult( const std::vector<nlohmann::json>& aViolations,
                                                 const std::vector<nlohmann::json>& aUnconnected,
                                                 const nlohmann::json& aIgnoredChecks, const nlohmann::json& aArgs );

#endif
