#pragma once

#include <nlohmann/json.hpp>

#include <functional>
#include <set>
#include <string>
#include <vector>

class BOARD;
class PAD;

/// @brief A bus link between two parts (a corridor): its nets and the parts of the blocks at its ends
struct TIER_LINK
{
    std::string              name;
    std::vector<std::string> nets;
    std::set<std::string>    aParts, bParts;
    size_t                   id = 0;   ///< the caller's, given back as "id"
};

/// @brief The board's via and track sizes for the stacks' vias and slots
struct TIER_RULES
{
    int viaDiameter = 0;
    int clearance = 0;
    int trackWidth = 0;
    std::set<std::string> routed;   ///< the nets being routed: the facing rows' other ones among them are passengers
};

/// @brief The tier plan of the bus links (docs: router-architecture/tier-planner.md).
/// Links are taken shortest first. A link runs straight when every part in its band between its
/// ends has lanes left under it (its pad rows along the run without a signal pin, less the links
/// already through); else it detours beside them all on one side: the side the
/// end part's other detours along that run already took, else the one whose band crosses the fewest
/// other connections and detours.  Its layer is F.Cu when it passes under nothing and every pin at
/// both ends is entered directly from the side it arrives at, else B.Cu.
/// A link on B.Cu gets its pins' vias and slots per stack (a part's side, every link leaving there):
/// a facing-row pin a via just outside its pad, a far-row pin a via under the body with a leg across
/// on B.Cu at its row, or the nearest free row when a facing via stands at it; vias closer than the
/// rules allow are staggered; the slots stand in a column past the vias.  Only two-row SMD parts left
/// across their rows get them; a link with slots at both ends is "covered" and has a centreline.
/// @param aPitch a pad's track width plus clearance
/// @return {links: [...], parts: [...]}: per link its route, side, the parts it passes round or
/// under, the side it leaves each end by, its layer, pin entries (with via, bends, slot when
/// covered) and centreline; per part its lanes and stacks
nlohmann::json PlanTiers( BOARD* aBoard, const std::vector<TIER_LINK>& aLinks,
                          const std::function<int( const PAD* )>& aPitch, const TIER_RULES& aRules );
