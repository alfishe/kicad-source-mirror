/// @file kicadopenapi_util.h
/// @brief kicadopenapi helpers shared by method implementations (lists, filters, pagination).
#ifndef KICADOPENAPI_UTIL_H
#define KICADOPENAPI_UTIL_H

#include <functional>
#include <string>
#include <vector>
#include <wx/string.h>

#include <json_common.h>
#include <kicommon.h>


/// @brief Case-insensitive glob with '*', '?' and brace alternation ("{J8,U37,R*}", nested, one or
/// more groups); a brace pair without a comma is literal; an empty pattern matches everything
KICOMMON_API bool KopenapiGlob( const std::string& aPattern, const std::string& aText );

/// @brief The plain globs a pattern stands for, braces expanded ("U{1,2}*" -> "U1*", "U2*")
KICOMMON_API std::vector<std::string> KopenapiGlobAlternatives( const std::string& aPattern );

/// @brief Reference filter of list / bulk methods: a glob under aGlobKey ("ref", braces allowed)
/// and / or an array of references or globs under aListKey ("refs"); an item passes when any
/// pattern matches.  Every pattern that matched nothing is reported (Unmatched), so a filter
/// never yields an unexplained empty answer.
class KICOMMON_API KOPENAPI_REF_FILTER
{
public:
    KOPENAPI_REF_FILTER( const nlohmann::json& aArgs, const char* aGlobKey = "ref", const char* aListKey = "refs" );

    /// @brief Bad argument types; empty when valid
    const std::string& Error() const { return m_error; }

    /// @brief No pattern given: everything passes
    bool Empty() const { return m_patterns.empty(); }

    /// @brief Does aRef pass; records which patterns hit
    bool Match( const std::string& aRef );

    /// @brief Patterns (braces expanded) that matched no reference passed to Match
    nlohmann::json Unmatched() const;

private:
    std::vector<std::string> m_patterns;
    std::vector<char>        m_hit;
    std::string              m_error;
};

/// @brief Index into aUuids of the one uuid aId names: the whole uuid, or a prefix of 4 characters
/// or more (case and spaces ignored); a uuid listed more than once counts once.  -1 when none or
/// several match, aError then says which ("no item with uuid ..." / "uuid prefix ... is ambiguous:
/// ..." naming up to 8 of them, with aLabel( index ) when given)
KICOMMON_API long KopenapiUuidMatch( const std::vector<std::string>& aUuids, const std::string& aId, std::string& aError,
                                     const std::function<std::string( size_t )>& aLabel = nullptr );

/// @brief The file's absolute path with symbolic links and "." / ".." resolved (as far as it
/// exists): one spelling per file, so a project reached through another path is the same project
KICOMMON_API wxString KopenapiCanonicalPath( const wxString& aPath );

/// @brief Natural order: "R2" < "R10", case-insensitive
KICOMMON_API bool KopenapiNaturalLess( const std::string& aA, const std::string& aB );

/// @brief One page of aItems: args "limit" (default aDefaultLimit, max 1000) and "cursor" (opaque,
/// from the previous page).  Returns { items, total, next_cursor (null on the last page) }.
KICOMMON_API nlohmann::json KopenapiPage( const nlohmann::json& aItems, const nlohmann::json& aArgs,
                                          size_t aDefaultLimit = 100 );

/// @brief Role of a net guessed from its name only: { role: ground|power|clock|reset|signal|unconnected,
/// basis: [...] }.  Hierarchical prefixes ("/CPU/") are ignored, a trailing '/' (active-low,
/// "RESET/") is kept; KiCad auto names ("Net-(...)", "unconnected-(...)") are not guessed from.
KICOMMON_API nlohmann::json KopenapiNetRoleFromName( const std::string& aNetName );

/// @brief Connector-like reference designator (J1, P2, X3, CN4, XS1, XP1, CON5): where nets leave the board
KICOMMON_API bool KopenapiIsConnectorRef( const std::string& aRef );

/// @brief Relevance of a library item for a free-text query: every whitespace-separated term must
/// occur (case-insensitive) in the name, keywords or description, else 0.  Name hits weigh
/// most (exact name > name prefix > name substring), then keywords, then description.
KICOMMON_API int KopenapiTextScore( const std::string& aQuery, const std::string& aName,
                                    const std::string& aKeywords, const std::string& aDescription );

/// @brief The function of a 74 / 54 series logic part number without vendor prefix, logic family
/// and suffix: "SN74LVC257APW", "74HC257", "74LS257" -> "74257", "74LVC1G04" -> "741g04" (lower
/// case); empty when aText is not such a number.  Equal cores share the function and pinout.
KICOMMON_API std::string KopenapiLogicCore( const std::string& aText );

/// @brief Per-term outcome of a free-text query against a library item
struct KOPENAPI_TEXT_MATCH
{
    int                      score = 0;     ///< relevance of the matched terms
    int                      matched = 0;   ///< terms found
    int                      terms = 0;     ///< terms in the query
    std::vector<std::string> unmatched;     ///< terms not found
};

KICOMMON_API KOPENAPI_TEXT_MATCH KopenapiTextMatch( const std::string& aQuery, const std::string& aName,
                                                    const std::string& aKeywords, const std::string& aDescription );

/// @brief Page of ranked search rows: rows matching every query term if there are any; otherwise the
/// best partial matches (most terms first), flagged "partial" with the terms nothing matched, so
/// a query with one unlucky word still returns something useful.  Default page: 20 rows.
KICOMMON_API nlohmann::json KopenapiRankedPage( std::vector<std::pair<KOPENAPI_TEXT_MATCH, nlohmann::json>> aRows,
                                                const nlohmann::json& aArgs );

/// @brief Lists in the answers of bulk methods (one entry per track, via, part, connection): at
/// most "list_limit" entries each (default aDefault, 0 = counts only), every entry with "full":
/// true.  A cut list keeps its first entries; "<key>_total" holds its length and the key is
/// named in the answer's "truncated".
class KICOMMON_API KOPENAPI_LIST_CAP
{
public:
    explicit KOPENAPI_LIST_CAP( const nlohmann::json& aArgs, size_t aDefault = 10 );

    size_t Limit() const { return m_limit; }
    bool   Full() const { return m_full; }

    /// @brief aOut[aKey] = aList (an array) cut to the limit
    void Put( nlohmann::json& aOut, const std::string& aKey, nlohmann::json aList ) const;

    /// @brief As Put, the entries aFirst picks kept ahead of the rest when the list is cut (e.g.
    /// failed rows before routed ones); the kept entries stay in their order
    void PutPreferring( nlohmann::json& aOut, const std::string& aKey, nlohmann::json aList,
                        const std::function<bool( const nlohmann::json& )>& aFirst ) const;

    /// @brief aOut[aKey] = an object of the entries given, first ones kept (callers order them,
    /// e.g. most items first)
    void PutObject( nlohmann::json& aOut, const std::string& aKey,
                    const std::vector<std::pair<std::string, nlohmann::json>>& aEntries ) const;

    /// @brief aSchema (an object schema) with list_limit and full added
    static nlohmann::json Schema( nlohmann::json aSchema, size_t aDefault = 10 );

private:
    void mark( nlohmann::json& aOut, const std::string& aKey, size_t aTotal ) const;

    size_t m_limit;
    bool   m_full;
};

/// @brief JSON schema properties shared by all paginated methods (limit, cursor)
KICOMMON_API nlohmann::json KopenapiPageSchema( int aDefaultLimit = 100 );

/// @brief Object schema with aProperties plus limit/cursor
KICOMMON_API nlohmann::json KopenapiPagedSchema( nlohmann::json aProperties, int aDefaultLimit = 100 );

/// @brief Default page size of ranked searches (KopenapiRankedPage)
constexpr int KOPENAPI_RANKED_PAGE = 20;

#endif
