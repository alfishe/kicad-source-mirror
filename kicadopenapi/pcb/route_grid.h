#pragma once

#include <layer_ids.h>
#include <math/vector2d.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class BOARD;
class BOARD_CONNECTED_ITEM;
struct GRID_TIMELINE;

/// @brief A net to join on the grid: its ends (pads, tracks, vias of the net) and its sizes
struct GRID_NET
{
    int                                netCode = 0;
    std::vector<BOARD_CONNECTED_ITEM*> ends;
    int                                width = 0;
    int                                clearance = 0;
    int                                viaDiameter = 0;   ///< 0: no vias for this net
    int                                viaDrill = 0;
    int                                group = -1;   ///< bundle: routed first, along its other members
    int                                rank = 0;     ///< member order within the bundle
};

/// @brief Where each variant is while the variants run
struct GRID_VARIANT_PROGRESS
{
    std::string name;
    std::string stage;      ///< rounds, legalise, short_again, fewer_vias, pull_tight, done, abandoned
    int         round = 0;
    int         crowded = 0;
    int         net = 0;    ///< nets taken in this stage (of nets)
    int         nets = 0;
    long        expanded = 0;
};

/// @brief How one variant searches: net order, layer directions, negotiation schedule, A* weight
struct GRID_STRATEGY
{
    std::string order = "short_first";   ///< short_first, fine_first, long_first
    std::string directions;              ///< "" any, "hv", "vh" (per layer from the first)
    float       pfStart = 1.0f;
    float       pfGrowth = 1.6f;
    float       astar = 1.2f;
    int         schedule = 0;            ///< 0 fixed, 1 adaptive growth, 2 adaptive + history on cells fought over again
    float       histDecay = 1.0f;        ///< history of places not fought over in a round is multiplied by this; 1 kept
    int         recent = 0;              ///< rounds: a net is routed again through history this recent only; 0 any history

    std::string Name() const;
};

struct GRID_PARAMS
{
    std::vector<PCB_LAYER_ID> layers;          ///< routing layers
    int                       cell = 0;        ///< 0: from the finest class
    int                       variants = 0;    ///< 0: half the cores, up to the built-ins, as many as the memory allows
    int64_t                   memoryBudget = 0; ///< bytes the lanes may take; 0: half the memory available now
    int                       maxRounds = 40;
    int                       viaKeepOut = 0;  ///< only the chip's own nets' vias this far around it while routing; 0 none
    int                       viaAway = 0;     ///< a via nearer a chip's pads than this costs more, the nearer the more; 0 none
    int                       viaAwayCost = 0; ///< at the pads: as much as this length of track
    float                     hug = 1.0f;      ///< cost factor of a cell beside the copper of the net's own bundle; 1 none
    float                     astar = 1.2f;    ///< the distance left counts this much more than it costs at least (VPR's astar_fac)
    float                     pfStart = 1.0f;  ///< the price of sharing a cell in the first round (VPR's initial_pres_fac)
    float                     pfGrowth = 1.6f; ///< and how much it grows each round (VPR's pres_fac_mult)
    bool                      record = false;  ///< keep each variant's timeline (GRID_RESULT::timeline)
    /// @brief A paused job: true while the variants wait between two nets; aSerial / aBy / aN the last
    /// job_step (KopenapiJobHeld).  Any thread
    std::function<bool( uint64_t& aSerial, std::string& aBy, int& aN )> held;
    /// @brief With record: the timeline before the variants start (calling thread); it grows while they run
    std::function<void( const std::shared_ptr<GRID_TIMELINE>& )> timelineStarted;
    /// @brief A branch: the lane branchLane of this timeline goes on after its step branchStep as a new
    /// lane of the same timeline (the board must be the one the run started from)
    std::shared_ptr<GRID_TIMELINE> branchOf;
    int                       branchLane = -1;
    size_t                    branchStep = 0;
    int                       schedule = 0;    ///< GRID_STRATEGY::schedule of the built-in variants
    float                     histDecay = 1.0f; ///< GRID_STRATEGY::histDecay of the built-in variants
    int                       recent = 2;      ///< GRID_STRATEGY::recent of the built-in variants
    std::vector<GRID_STRATEGY> strategies;     ///< the variants to race; empty: the built-ins
    std::atomic<bool>*        cancel = nullptr;

    /// @brief Called on the calling thread while the variants run, about every 50 ms; false stops them
    std::function<bool( const std::vector<GRID_VARIANT_PROGRESS>& )> progress;
};

/// @brief A run of track on one layer: its corners from end to end
struct GRID_TRACK
{
    PCB_LAYER_ID          layer = UNDEFINED_LAYER;
    std::vector<VECTOR2I> points;
};

struct GRID_NET_RESULT
{
    std::vector<GRID_TRACK> tracks;
    std::vector<VECTOR2I>   vias;
    std::vector<bool>       reached;   ///< per end: joined to the first end
    bool                    complete = false;
    double                  length = 0;   ///< board units
};

/// @brief A net still crowded when negotiation ended: the first place and the nets there
struct CROWDING
{
    int              net = 0;
    VECTOR2I         at;
    int              layer = -1;   ///< index into the routing layers, -1 a via
    std::vector<int> by;
};

struct GRID_RESULT
{
    std::vector<GRID_NET_RESULT> nets;   ///< in the order of the request
    std::string                  variant;
    int                          rounds = 0;
    int                          conflictsLeft = 0;   ///< nets dropped to make the rest legal
    int                          cell = 0;
    long                         expanded = 0;
    double                       buildMs = 0;
    double                       routeMs = 0;
    std::vector<std::string>     variantsTried;
    int64_t                      laneBytes = 0;       ///< one lane's search state (estimate)
    int                          lanesMemoryAllows = 0; ///< within GRID_PARAMS::memoryBudget
    std::vector<std::string>     variantsAbandoned;   ///< stopped in their rounds once another joined everything
    std::string                  error;               ///< nothing routed: why (a branch over another board)
    int                          branchLane = -1;     ///< the lane a branch was recorded as
    double                       restoreMs = 0;       ///< a branch: replaying its parent up to the step
    double                       printMs = 0;         ///< hashing the board as the search sees it (recorded / a branch)

    /// @brief Per variant tried: when it finished, its search states per stage, ends joined, vias
    struct VARIANT_STATS
    {
        std::string       name;
        double            ms = 0;
        std::vector<long> expandedByStage;
        long              reached = 0;
        long              vias = 0;
        bool              abandoned = false;
    };

    std::vector<VARIANT_STATS> variantStats;
    std::vector<int>             crowdedByRound;   ///< the winner's nets still crowded after each round
    std::vector<CROWDING>        crowdedAtEnd;     ///< still crowded when its rounds ended
    std::vector<long>            expandedByStage;   ///< the winner's: rounds, legalising, the short again, fewer vias
    std::vector<std::pair<int, long>> costliest;   ///< the winner's nets that took the most search states
    std::shared_ptr<GRID_TIMELINE>    timeline;    ///< with GRID_PARAMS::record: every variant's steps
};

/**
 * Join each net's ends over one grid of the board, all nets together: negotiated congestion
 * (PathFinder: every net routes through the cheapest cells, sharing costs more each round and a
 * place fought over keeps a history cost) until no two nets crowd each other.  The obstacles are
 * read from the board once on the calling thread (the board's rule for each pair, holes, keep-outs,
 * the edge); the search runs in variants on worker threads (net order, layer directions) that never
 * touch the board, and the best legal result is returned (fewest nets left out, then vias, then
 * length; ties to the first variant).  Vias cost more near fine-pitch packages; around chips (pads on
 * four sides, or an array outside its balls) only the chip's own nets put vias while the board
 * routes: the other nets only when left short, and once every net is joined in the pass that saves
 * vias.  Signal vias cost more the nearer they are to a chip's pads (viaAway).  Bundles (GRID_NET::group)
 * are routed first in each variant, member by member, and a cell beside a member's copper costs less
 * to the other members (hug): the bundle runs side by side.
 */
GRID_RESULT RouteOnGrid( BOARD* aBoard, const std::vector<GRID_NET>& aNets, const GRID_PARAMS& aParams );

/// @brief The variants raced when GRID_PARAMS::strategies is empty, in the order lanes are dropped from
/// the end: mixed net orders, layer directions and first-round prices (relative to aParams.pfStart)
std::vector<GRID_STRATEGY> GridBuiltinStrategies( const GRID_PARAMS& aParams );
