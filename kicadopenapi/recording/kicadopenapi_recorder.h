/// @file kicadopenapi_recorder.h
/// @brief Video recording of KiCad windows (record_* methods).
#ifndef KICADOPENAPI_RECORDER_H
#define KICADOPENAPI_RECORDER_H

/// @brief Finishes a running recording; the service calls it when it stops (KiCad closing)
void KopenapiRecorderShutdown();

#endif
