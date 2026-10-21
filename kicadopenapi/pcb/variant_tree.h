#pragma once

#include <nlohmann/json.hpp>

#include <memory>
#include <string>

class BOARD;
struct BOARD_STATE;
struct GRID_TIMELINE;

/// @brief A recorded run starts from the board as it is (pcb_route, main thread): the node it starts
/// from, the current one or else a new 'start' node with the board now
int VariantTreeRunStarted( const std::shared_ptr<GRID_TIMELINE>& aTimeline, const BOARD* aBoard );

/// @brief A warm branch's lane goes on over the board as it is now: its steps' nodes keep that board
void VariantTreeWarmLane( const std::shared_ptr<GRID_TIMELINE>& aTimeline, int aLane, const BOARD* aBoard );

/// @brief An automatic node at a lane's step (kind end, branch or pause); an existing node at the
/// same place is returned instead. aCurrent: the board after the running call, the node's
int VariantTreeAutoNode( const std::shared_ptr<GRID_TIMELINE>& aTimeline, int aLane, size_t aStep,
                         const std::string& aKind, const nlohmann::json& aMetrics, const BOARD* aCurrent );

/// @brief pcb_route {from: {node}}: where the node is in its run; false with the reason
bool VariantTreeNodeAt( int aId, int& aRun, int& aLane, size_t& aStep, std::string& aError );

/// @brief The board a node keeps (a run's step: the board its lane ran over)
bool VariantTreeLaneBoard( int aId, std::shared_ptr<const BOARD_STATE>& aState, std::string& aError );
