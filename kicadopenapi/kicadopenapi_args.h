/// @file kicadopenapi_args.h
/// @brief Arguments of a call checked against the method's inputSchema before the main thread sees
/// them: unknown top-level names are refused (with the nearest known name), and the whole schema
/// is applied (nested types, enums, ranges, items).  A typo that would otherwise fall back to a
/// default ("layers" for "layer") comes back as an error the caller can act on.
#ifndef KICADOPENAPI_ARGS_H
#define KICADOPENAPI_ARGS_H

#include <json_common.h>
#include <kicommon.h>

#include <string>
#include <vector>


/// @brief Names a method takes at the top level; empty when its schema does not say (no
/// "properties"), or allows others ("additionalProperties" not false)
KICOMMON_API std::vector<std::string> KopenapiKnownArgs( const nlohmann::json& aSchema );

/// @brief The first problem with aArgs under aSchema, or empty when there is none.  aMethod names
/// the cached validator (one per method, compiled on first use; thread-safe).  A schema that does
/// not compile is reported once to the journal and its calls are not validated beyond the names.
KICOMMON_API std::string KopenapiValidateArgs( const std::string& aMethod, const nlohmann::json& aSchema,
                                               const nlohmann::json& aArgs );

#endif
