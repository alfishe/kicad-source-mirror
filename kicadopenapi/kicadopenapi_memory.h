/// @file kicadopenapi_memory.h
/// @brief Process and allocator memory for the API (memory_summary, memory_trim) and the automatic
/// allocator trim: memory freed by closing documents is given back to the OS on a background
/// thread once KiCad is idle (policy measured in POC-001: idle >= 1 s, footprint - live grown by
/// >= 64 MB since the last trim, >= 30 s between trims, never during user input or an API call). Env
/// KICAD_OPENAPI_AUTO_TRIM=0 turns the automatic trim off.
#ifndef KICADOPENAPI_MEMORY_H
#define KICADOPENAPI_MEMORY_H

#include <json_common.h>
#include <kicommon.h>

#include <string>


class KICOMMON_API KOPENAPI_MEMORY
{
public:
    /// @brief Starts the trim thread and the user-input filter (idempotent). Main thread.
    static void StartAutoTrim();

    /// @brief Stops both; safe to call more than once. Main thread.
    static void StopAutoTrim();

    /// @brief { os, process, allocator, auto_trim }; any thread
    static nlohmann::json Summary();

    /// @brief Trims now on the calling thread.
    /// @param aTrigger is recorded with the result ("api", "auto").
    /// @return { before, after, released, allocator_reported, ms }
    static nlohmann::json TrimNow( const std::string& aTrigger );
};

#endif
