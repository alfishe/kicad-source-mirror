/// @file kicadopenapi_progress.h
/// @brief Lifetime of KiCad's progress windows (title, shown, closed) for the API: what a user waits on.
#pragma once

#include <kicommon.h>
#include <nlohmann/json.hpp>
#include <wx/string.h>


class KICOMMON_API KOPENAPI_PROGRESS
{
public:
    /// @brief A progress window appeared (any thread); aKey identifies it until Closed()
    static void Opened( const void* aKey, const wxString& aTitle );
    static void Closed( const void* aKey );

    /// @brief Open windows and the recent history: title, opened / closed (ms since process
    /// start and wall time), duration
    static nlohmann::json Snapshot( size_t aLimit );
};
