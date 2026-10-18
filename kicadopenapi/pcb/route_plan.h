#pragma once

#include <layer_ids.h>
#include <math/vector2d.h>
#include <optional>
#include <vector>

class BOARD;
class BOARD_CONNECTED_ITEM;
class PAD;

/// @brief A connection to plan: from a pad to a pad (or other item of the net) over the given layers
struct ROUTE_PLAN_REQUEST
{
    BOARD*                    board = nullptr;
    PAD*                      from = nullptr;
    BOARD_CONNECTED_ITEM*     to = nullptr;
    VECTOR2I                  toPos;
    std::vector<PCB_LAYER_ID> layers;              ///< routing layers, in order of preference
    int                       trackWidth = 0;
    int                       viaDiameter = 0;     ///< 0: no layer changes
    int                       viaDrill = 0;

    /// @brief Copper the plan may cross at a cost: routes that can be taken out and routed again
    struct SOFT
    {
        std::vector<BOARD_CONNECTED_ITEM*> copper;
        int                                cellCost = 0;   ///< per cell of its clearance crossed (a straight step is 10)
    };

    std::vector<SOFT> soft;
};

/// @brief A route through free space: corners on a layer, and through vias where it changes layer
struct ROUTE_PLAN
{
    bool         found = false;
    PCB_LAYER_ID startLayer = UNDEFINED_LAYER;

    /// @brief The points after the start pad: a corner (no layer) or a via to that layer
    std::vector<std::pair<VECTOR2I, std::optional<PCB_LAYER_ID>>> points;

    std::vector<int> crossed;   ///< the soft copper the path runs through (indexes into the request's soft)

    int    vias = 0;
    double length = 0;     ///< board units, along the cells
    int    cell = 0;       ///< grid pitch used
    long   expanded = 0;   ///< search states taken
    double ms = 0;
};

/**
 * Plan a connection over a grid of the free space around it (the clearance to other nets' copper,
 * holes, keep-out areas and the board edge from the board's rules for each pair; through vias where
 * one fits on every copper layer), A* by length with a cost for each via and each turn. The path
 * runs in steps of 0, 45 and 90 degrees between its corners, so the push-and-shove router can
 * follow it point by point.
 */
ROUTE_PLAN PlanRoute( const ROUTE_PLAN_REQUEST& aReq );
