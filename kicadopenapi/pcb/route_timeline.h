#pragma once

#include "route_grid.h"
#include "route_raster.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <kiid.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

/// @brief A net's copper at some step: its paths over the grid's states (layer * cells + cell)
struct GRID_PATH
{
    std::vector<std::vector<uint32_t>> paths;
    std::vector<bool>                  reached;
};

/// @brief One net taken by a variant: routed in a round, legalised, routed short again, with fewer
/// vias, pulled tight, or dropped
struct GRID_STEP
{
    static constexpr uint32_t NONE = UINT32_MAX;

    uint8_t  stage = 0;          ///< GRID_TIMELINE::STAGES
    uint16_t round = 0;
    int32_t  net = 0;            ///< index into GRID_TIMELINE::netCodes
    uint32_t path = NONE;        ///< the net's path after the step (GRID_LANE::paths), NONE: no copper
    int32_t  crowded = 0;        ///< nets crowded after the last finished round
    int64_t  expanded = 0;       ///< the variant's search states so far
    int32_t  box[4] = { -1, -1, -1, -1 };   ///< cells searched in: x0, y0, x1, y1 (-1: no search)
    float    ms = 0;             ///< since the variants started
    uint32_t conflicts = 0;      ///< the places crowded after the step (GRID_LANE::conflictSets)
};

/// @brief One variant's steps; the state at any step is the path of every net, kept as a full table
/// every KEY steps and replayed from there
class GRID_LANE
{
public:
    static constexpr size_t KEY = 64;

    std::string            name;
    std::vector<GRID_STEP> steps;
    std::vector<GRID_PATH> paths;
    bool                   abandoned = false;

    /// @brief The variant: its strategy, net order and layer directions
    GRID_STRATEGY          strategy;
    std::vector<int>       order;
    std::vector<int>       direction;

    /// @brief A branch: the lane it goes on from and the step after which its own steps begin (its
    /// steps before are the parent's); -1 a variant of the run
    int                    parent = -1;
    size_t                 parentStep = 0;
    bool                   running = false;   ///< steps are being added
    bool                   stopped = false;   ///< the route was stopped before the lane's stages ended
    bool                   deleted = false;   ///< a branch taken out (branch_delete): its index stays, its data gone
    /// @brief Routed on from its parent_step over an edited board (pcb_route {warm}): the board it
    /// went on over (worldPrint); 0 the run's
    uint64_t               world = 0;

    /// @brief Ends found walled in by fixed copper: from the search before step `step`
    struct DEAD
    {
        uint32_t step;
        int32_t  net;
        int32_t  end;
    };

    std::vector<DEAD>      dead;

    /// @brief Per finished round: the history cost increments of the round (track states, then
    /// cells + VIA for vias), and the heat of the places fought over (max history per tile)
    static constexpr uint32_t VIA = 1u << 31;
    std::vector<std::vector<uint32_t>> histByRound;
    std::vector<std::vector<uint16_t>> heatByRound;

    /// @brief Places where nets crowd each other: per tile (GRID_TIMELINE::tile) its crowded states,
    /// tile << 8 | min( count, 255 ), sorted; the first set is empty
    std::vector<std::vector<uint32_t>> conflictSets;

    void Start( size_t aNets );

    /// @brief The id of a set of crowded places, deduplicated by content
    uint32_t AddConflicts( const std::vector<uint32_t>& aSet );

    /// @brief After a net was taken: its copper now, deduplicated by content
    void Add( GRID_STEP aStep, const GRID_PATH* aPath );

    void AddDead( int aNet, int aEnd );

    /// @brief A branch's start: the parent's steps up to aStep (inclusive) with their paths, crowded
    /// places, keys and finished rounds
    void ForkFrom( const GRID_LANE& aParent, size_t aStep );

    /// @brief Every net's path id after step aStep (inclusive); before the first step with SIZE_MAX
    void TableAt( size_t aStep, std::vector<uint32_t>& aTable ) const;

    size_t Bytes() const;

    /// @brief Frees everything the lane holds (a branch deleted)
    void Release();

    /// @brief Held by the lane's variant while it adds a step or a round, and by a reader while the
    /// route runs (GRID_TIMELINE::live)
    std::recursive_mutex& Mutex() const { return *m_mutex; }

private:
    std::unique_ptr<std::recursive_mutex>          m_mutex = std::make_unique<std::recursive_mutex>();
    std::vector<uint32_t>                          m_table;
    std::vector<std::vector<uint32_t>>             m_keys;   ///< table before step k * KEY
    std::unordered_map<uint64_t, std::vector<uint32_t>> m_byHash;
    std::unordered_map<uint64_t, std::vector<uint32_t>> m_conflictsByHash;
};

/// @brief What the variants did, step by step, for replay
struct GRID_TIMELINE
{
    static constexpr const char* STAGES[] = { "rounds", "legalise", "short_again", "fewer_vias", "pull_tight", "dropped" };

    PLAN_GRID                 grid;
    size_t                    cells = 0;
    std::vector<PCB_LAYER_ID> layers;
    std::vector<int>          netCodes;
    std::vector<std::string>  netNames;      ///< filled by the caller
    int                       run = 0;       ///< filled by the caller
    int                       tile = 1;      ///< heat tile, cells
    int                       tnx = 0, tny = 0;
    std::deque<GRID_LANE>     lanes;         ///< a branch adds one while readers hold others: Lane() / LaneCount()
    int                       winner = -1;   ///< set when the route ends
    uint64_t                  world = 0;     ///< the board as the router saw it (a branch needs the same)
    uint64_t                  rules = 0;     ///< the grid, layers and sizes (a warm branch needs the same)
    uint64_t                  nets = 0;      ///< the nets routed, by name in order (a warm branch needs the same)
    std::vector<std::string>  netKeys;       ///< the nets' board names in order (a warm branch routes them so)
    std::vector<uint64_t>     tilePrints;    ///< per tile (tile, tnx, tny): the world there hashed (what an edit changed)
    std::string               args;          ///< the route call's arguments (JSON), a branch's too
    long                      appliedSeq = 0; ///< the edit history entry that put a lane's copper on the board
    int                       appliedLane = -1;
    std::vector<KIID>         appliedItems;  ///< the tracks and vias that put it there
    std::atomic<int>          shownLane{ -1 }; ///< branch_switch: the lane frame / the replay show by default
    std::atomic<bool>         live{ false }; ///< the route is running: steps are added; the caller clears it
                                             ///< once the route's copper is on the board

    size_t           LaneCount() const;
    const GRID_LANE& Lane( size_t aLane ) const;
    GRID_LANE&       Lane( size_t aLane );
    size_t           AddLane();

    /// @brief The lane shown when none is asked for: the switched-to one, else the winner (lane 0
    /// while live)
    int              DefaultLane() const;

    GRID_NET_RESULT Geometry( const GRID_PATH& aPath ) const;
    size_t          Bytes() const;

    /// @brief The last round finished at a step of a lane (0: none yet)
    int HeatRound( const GRID_LANE& aLane, size_t aStep ) const;

    /// @brief Per tile: rounds since it was last fought over by the end of round aRound (0: that
    /// round), 255 never
    std::vector<uint8_t> FightAge( const GRID_LANE& aLane, int aRound ) const;

    /// @brief The board point (internal units) of a tile's centre
    VECTOR2I TileCentre( int aTile ) const;

private:
    mutable std::mutex m_lanesMutex;
};

/// @brief Grid paths as tracks and vias: corners only where the direction changes
GRID_NET_RESULT GridPathGeometry( const PLAN_GRID& aGrid, size_t aCells, const std::vector<PCB_LAYER_ID>& aLayers,
                                  const std::vector<std::vector<uint32_t>>& aPaths, const std::vector<bool>& aReached );

/// @brief The recorded grid routes of this process: the last runs within a memory budget, least
/// recently used out first (never the last one or a live one). Keep: a run recorded or branched now
void                           GridTimelineKeep( std::shared_ptr<GRID_TIMELINE> aTimeline );
std::shared_ptr<GRID_TIMELINE> GridTimelineLast();
/// @brief A kept run by id (used: it goes to the end of the eviction order); null when not kept
std::shared_ptr<GRID_TIMELINE> GridTimelineFind( int aRun );
std::vector<std::shared_ptr<GRID_TIMELINE>> GridTimelinesKept();
size_t                         GridTimelineBudget();
/// @brief The runs evicted to fit the new budget
std::vector<int>               GridTimelineSetBudget( size_t aBytes );

/// @brief The editor's replay of a running route moves on (from the route's progress, main thread):
/// a plain call holds the main loop, so the replay's own timer does not run meanwhile
void                           GridReplayAdvance();

/// @brief The editor's replay after branch_switch / branch_delete: it shows the timeline's default
/// lane when its own is gone or another was switched to (main thread)
void                           GridReplayFollow( const std::shared_ptr<GRID_TIMELINE>& aTimeline );
