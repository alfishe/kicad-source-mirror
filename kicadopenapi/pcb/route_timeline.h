#pragma once

#include "route_grid.h"
#include "route_raster.h"

#include <atomic>
#include <cstdint>
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

    /// @brief Every net's path id after step aStep (inclusive); before the first step with SIZE_MAX
    void TableAt( size_t aStep, std::vector<uint32_t>& aTable ) const;

    size_t Bytes() const;

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
    std::vector<GRID_LANE>    lanes;
    int                       winner = -1;   ///< set when the route ends
    std::atomic<bool>         live{ false }; ///< the route is running: steps are added; the caller clears it
                                             ///< once the route's copper is on the board

    GRID_NET_RESULT Geometry( const GRID_PATH& aPath ) const;
    size_t          Bytes() const;

    /// @brief The last round finished at a step of a lane (0: none yet)
    int HeatRound( const GRID_LANE& aLane, size_t aStep ) const;

    /// @brief Per tile: rounds since it was last fought over by the end of round aRound (0: that
    /// round), 255 never
    std::vector<uint8_t> FightAge( const GRID_LANE& aLane, int aRound ) const;

    /// @brief The board point (internal units) of a tile's centre
    VECTOR2I TileCentre( int aTile ) const;
};

/// @brief Grid paths as tracks and vias: corners only where the direction changes
GRID_NET_RESULT GridPathGeometry( const PLAN_GRID& aGrid, size_t aCells, const std::vector<PCB_LAYER_ID>& aLayers,
                                  const std::vector<std::vector<uint32_t>>& aPaths, const std::vector<bool>& aReached );

/// @brief The last recorded grid route of this process (main thread)
void                           GridTimelineKeep( std::shared_ptr<GRID_TIMELINE> aTimeline );
std::shared_ptr<GRID_TIMELINE> GridTimelineLast();

/// @brief The editor's replay of a running route moves on (from the route's progress, main thread):
/// a plain call holds the main loop, so the replay's own timer does not run meanwhile
void                           GridReplayAdvance();
