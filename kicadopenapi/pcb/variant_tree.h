#pragma once

#include <nlohmann/json.hpp>

#include <memory>
#include <string>

struct GRID_TIMELINE;

/// @brief A recorded run starts from the board as it is (pcb_route, main thread): the node it starts
/// from, the current one or else a new 'start' node
int VariantTreeRunStarted( const std::shared_ptr<GRID_TIMELINE>& aTimeline );

/// @brief An automatic node at a lane's step (kind end, branch or pause); an existing node at the
/// same place is returned instead. aCurrent: the board after the running call is the node's
int VariantTreeAutoNode( const std::shared_ptr<GRID_TIMELINE>& aTimeline, int aLane, size_t aStep,
                         const std::string& aKind, const nlohmann::json& aMetrics, bool aCurrent );

/// @brief pcb_route {from: {node}}: where the node is in its run; false with the reason
bool VariantTreeNodeAt( int aId, int& aRun, int& aLane, size_t& aStep, std::string& aError );
