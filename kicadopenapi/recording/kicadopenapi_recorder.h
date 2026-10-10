/// @file kicadopenapi_recorder.h
/// @brief Video recording of KiCad windows (record_* methods).
#ifndef KICADOPENAPI_RECORDER_H
#define KICADOPENAPI_RECORDER_H

#include <kicommon.h>

/// @brief Finishes a running recording; the service calls it when it stops (KiCad closing)
KICOMMON_API void KopenapiRecorderShutdown();

/// @brief A recording runs (not paused): animations step on video time, one frame per step.
/// @param aFps receives the recording's frame rate (optional)
KICOMMON_API bool KopenapiRecordingSync( int* aFps = nullptr );

/// @brief Records the window as it is now as the next video frame (video time advances by exactly
/// one frame); waits while the encoder is behind instead of dropping. Main thread.
KICOMMON_API void KopenapiRecordFrame();

/// @brief Screen frames per video frame during a synced recording: the display's refresh rate over
/// the video's frame rate (at least 1), so the window animates at the display's rate
KICOMMON_API int KopenapiRecordingScreenSteps();

#endif
