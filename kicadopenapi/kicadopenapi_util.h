/*
 * kicadopenapi helpers shared by method implementations (lists, filters, pagination).
 */
#ifndef KICADOPENAPI_UTIL_H
#define KICADOPENAPI_UTIL_H

#include <string>

#include <json_common.h>
#include <kicommon.h>


/// Case-insensitive glob with '*' and '?'; an empty pattern matches everything
KICOMMON_API bool KopenapiGlob( const std::string& aPattern, const std::string& aText );

/// Natural order: "R2" < "R10", case-insensitive
KICOMMON_API bool KopenapiNaturalLess( const std::string& aA, const std::string& aB );

/**
 * One page of aItems: args "limit" (default aDefaultLimit, max 1000) and "cursor" (opaque,
 * from the previous page).  Returns { items, total, next_cursor (null on the last page) }.
 */
KICOMMON_API nlohmann::json KopenapiPage( const nlohmann::json& aItems, const nlohmann::json& aArgs,
                                          size_t aDefaultLimit = 100 );

/**
 * Role of a net guessed from its name only: { role: ground|power|clock|reset|signal,
 * basis: [...] }.  Hierarchical prefixes ("/CPU/") are ignored.
 */
KICOMMON_API nlohmann::json KopenapiNetRoleFromName( const std::string& aNetName );

/// JSON schema properties shared by all paginated methods (limit, cursor)
KICOMMON_API nlohmann::json KopenapiPageSchema();

#endif
