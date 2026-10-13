/// @file kopenapi_model_warmup.h
/// @brief Background warm-up of the 3D model cache after a board is opened in the GUI, so the 3D
/// viewer finds its models loaded.
#pragma once

#include <nlohmann/json.hpp>

class BOARD;


/// @brief Starts warming the board's 3D models (GUI only; stops any earlier warm-up). The list is
/// taken now (main thread); the loading runs on two low-priority threads after a short delay,
/// largest files first, and stops under memory pressure or once its budget is spent.
void KopenapiWarmModels( BOARD* aBoard );

/// @brief Stops the warm-up and waits for it (at most the model being read)
void KopenapiStopWarmModels();

/// @brief State for memory_summary: running, models done / queued, bytes, why it stopped
nlohmann::json KopenapiWarmModelsStatus();
