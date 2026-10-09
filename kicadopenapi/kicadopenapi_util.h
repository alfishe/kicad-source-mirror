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
 * Role of a net guessed from its name only: { role: ground|power|clock|reset|signal|unconnected,
 * basis: [...] }.  Hierarchical prefixes ("/CPU/") are ignored, a trailing '/' (active-low,
 * "RESET/") is kept; KiCad auto names ("Net-(...)", "unconnected-(...)") are not guessed from.
 */
KICOMMON_API nlohmann::json KopenapiNetRoleFromName( const std::string& aNetName );

/// Connector-like reference designator (J1, P2, X3, CN4, XS1, XP1, CON5): where nets leave the board
KICOMMON_API bool KopenapiIsConnectorRef( const std::string& aRef );

/**
 * Relevance of a library item for a free-text query: every whitespace-separated term must
 * occur (case-insensitive) in the name, keywords or description, else 0.  Name hits weigh
 * most (exact name > name prefix > name substring), then keywords, then description.
 */
KICOMMON_API int KopenapiTextScore( const std::string& aQuery, const std::string& aName,
                                    const std::string& aKeywords, const std::string& aDescription );

/// JSON schema properties shared by all paginated methods (limit, cursor)
KICOMMON_API nlohmann::json KopenapiPageSchema();

/// Object schema with aProperties plus limit/cursor
KICOMMON_API nlohmann::json KopenapiPagedSchema( nlohmann::json aProperties );

#endif
