#pragma once

#include <layer_ids.h>
#include <math/vector2d.h>

#include <atomic>
#include <functional>
#include <string>
#include <vector>

class BOARD;
class BOARD_CONNECTED_ITEM;

/// @brief A net to join on the grid: its ends (pads, tracks, vias of the net) and its sizes
struct GRID_NET
{
    int                                netCode = 0;
    std::vector<BOARD_CONNECTED_ITEM*> ends;
    int                                width = 0;
    int                                clearance = 0;
    int                                viaDiameter = 0;   ///< 0: no vias for this net
    int                                viaDrill = 0;
};

struct GRID_PARAMS
{
    std::vector<PCB_LAYER_ID> layers;          ///< routing layers
    int                       cell = 0;        ///< 0: from the finest class
    int                       variants = 0;    ///< 0: one per core, up to 8
    int                       maxRounds = 40;
    int                       viaKeepOut = 0;  ///< only the chip's own nets' vias this far around it while routing; 0 none
    std::atomic<bool>*        cancel = nullptr;

    /// @brief Called on the calling thread while the variants run, about every 50 ms; false stops them
    std::function<bool( int aRound, int aConflicts )> progress;
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
    std::vector<int>             crowdedByRound;   ///< the winner's nets still crowded after each round
    std::vector<CROWDING>        crowdedAtEnd;     ///< still crowded when its rounds ended
    std::vector<long>            expandedByStage;   ///< the winner's: rounds, legalising, the short again, fewer vias
    std::vector<std::pair<int, long>> costliest;   ///< the winner's nets that took the most search states
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
 * vias.
 */
GRID_RESULT RouteOnGrid( BOARD* aBoard, const std::vector<GRID_NET>& aNets, const GRID_PARAMS& aParams );
