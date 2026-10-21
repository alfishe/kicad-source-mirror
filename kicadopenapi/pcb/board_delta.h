#pragma once

#include <kiid.h>
#include <nlohmann/json.hpp>

#include <map>
#include <memory>
#include <set>
#include <string>

class BOARD;
class BOARD_COMMIT;

/// @brief The board's items as their s-expression by KIID (footprints, tracks, vias, zones,
/// drawings, points; not groups, generators or constraints), the reference the states differ from
struct BOARD_BASE
{
    std::string                                         file;
    std::map<KIID, std::shared_ptr<const std::string>> items;
};

/// @brief A board as the base plus the items added or changed and the items removed
struct BOARD_STATE
{
    std::shared_ptr<const BOARD_BASE>                   base;
    std::map<KIID, std::shared_ptr<const std::string>> items;
    std::set<KIID>                                      removed;

    size_t Bytes() const;
};

/// @brief The board now against aBase (made from it when null). Main thread
std::shared_ptr<const BOARD_STATE> BoardStateTake( const BOARD* aBoard, std::shared_ptr<const BOARD_BASE>& aBase );

struct BOARD_STATE_APPLIED
{
    int         removed = 0, added = 0, replaced = 0, netsMissing = 0;
    std::string error;
};

/// @brief The board made aState's: items differing taken out, the state's put in (parsed, their
/// nets by name), staged on aCommit. Main thread
BOARD_STATE_APPLIED BoardStateApply( BOARD* aBoard, BOARD_COMMIT& aCommit, const BOARD_STATE& aState );

/// @brief How the board now differs from aState: items to add, to remove and changed, a few KIIDs each
nlohmann::json BoardStateDiff( const BOARD* aBoard, const BOARD_STATE& aState );
