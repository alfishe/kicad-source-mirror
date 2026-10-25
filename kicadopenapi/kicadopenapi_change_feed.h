/// @file kicadopenapi_change_feed.h
/// @brief The change sets of each document, one per commit, numbered by document version.
///
/// The documents' listeners record every commit (API call, job batch or GUI edit) on the owner
/// thread as one KOPENAPI_CHANGE_SET of item ids, tagged with the operation that made it
/// ("api:<method>", "job:<id>" or "gui").  Readers ask for what changed since a version they
/// saw (`doc_changes`) and look the items up in the live document under a read lease: ids only,
/// never item copies.  The last few hundred change sets per document are kept; an older
/// `since` answers `truncated` and the reader re-reads the document.
#ifndef KICADOPENAPI_CHANGE_FEED_H
#define KICADOPENAPI_CHANGE_FEED_H

#include <json_common.h>
#include <kicommon.h>

#include <cstdint>
#include <string>
#include <vector>


class KICOMMON_API KOPENAPI_CHANGE_FEED
{
public:
    /// @brief Records one commit of a document; empty lists record nothing (owner thread)
    static void Record( const std::string& aDomain, const std::vector<std::string>& aAdded,
                        const std::vector<std::string>& aChanged, const std::vector<std::string>& aRemoved );

    /// @brief The document was loaded, replaced or closed: readers re-read it
    static void Reset( const std::string& aDomain );

    static uint64_t Version( const std::string& aDomain );

    /// @param aSince the version the reader last saw
    /// @param aLimit change sets returned at most (the oldest first)
    /// @param aFold fold the returned change sets into one
    static nlohmann::json Since( const std::string& aDomain, uint64_t aSince, size_t aLimit, bool aFold );

    /// @brief Versions per document, for /status
    static nlohmann::json Status();

    static constexpr size_t KEEP = 512;
};

#endif
