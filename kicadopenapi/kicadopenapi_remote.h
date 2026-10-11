/// @file kicadopenapi_remote.h
/// @brief Remote control sessions (as in unreal-ng's automation sessions): a client takes control
/// explicitly (remote_session take); while it holds it, KiCad's windows are read-only for the user -
/// looking (scrolling, zooming, panning, window moves and resizes) stays, changing (clicks, keys,
/// menu and toolbar commands) is refused - and a bar over every KiCad window says who controls it,
/// with Take Back Control. The client is identified by the X-Kicad-Session header (the MCP bridge
/// sends one per bridge process). Its calls renew a lease; with no call for the lease the session
/// ends. When the user takes control back, that client's calls get 409 revoked_by_user until the
/// user allows it again or a cooldown passes. The API's own work and injected input are not
/// affected. Process-wide; the state is safe on any thread, the UI is main-thread.
#ifndef KICADOPENAPI_REMOTE_H
#define KICADOPENAPI_REMOTE_H

#include <kicadopenapi_registry.h>
#include <kicommon.h>

#include <optional>
#include <string>


class KICOMMON_API KOPENAPI_REMOTE
{
public:
    /// @brief Every API call: refuses a revoked client (409), renews the holder's lease
    static std::optional<KOPENAPI_RESULT> Gate( const std::string& aCaller, const std::string& aMethod );

    /// @brief remote_session: take, release, status, allow_again, take_back
    static KOPENAPI_RESULT Handle( const std::string& aCaller, const nlohmann::json& aArgs );

    /// @brief Ends the session and removes the UI (service stop)
    static void Shutdown();

    /// @brief A session is held now
    static bool Held();
};

#endif
