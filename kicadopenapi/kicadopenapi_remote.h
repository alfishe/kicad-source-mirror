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


class wxTopLevelWindow;
class wxWindow;


/// @brief One step of the remote lock cascade: what a part of KiCad switches off while the user's
/// input is refused, and how it puts back exactly what it changed. Main thread.
class KICOMMON_API KOPENAPI_REMOTE_PARTICIPANT
{
public:
    virtual ~KOPENAPI_REMOTE_PARTICIPANT() = default;

    virtual const char* Name() const = 0;

    /// @brief The lock starts: switch off, remembering the state found
    virtual void Engage() = 0;

    /// @brief The lock ends: restore the remembered state (not "everything on"); a state someone
    /// else changed meanwhile is left as it is
    virtual void Release() = 0;

    /// @brief A window appeared while engaged
    virtual void WindowAdded( wxTopLevelWindow* aWindow ) {}
};


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

    /// @brief The user's input is refused now (main thread; cheap, for UI update handlers)
    static bool UiLocked();

    /// @brief Adds a step to the lock cascade (engaged in ascending aOrder, released in reverse);
    /// engaged at once when the lock is on. Main thread; remove before destroying it.
    static void AddParticipant( KOPENAPI_REMOTE_PARTICIPANT* aParticipant, int aOrder );

    static void RemoveParticipant( KOPENAPI_REMOTE_PARTICIPANT* aParticipant );

    /// @brief Which command ids (menu, toolbar) of a window only look: allowed and kept enabled
    /// under the lock. Per window, as action ids are numbered per module; nullptr removes it.
    static void SetLookingCommands( wxWindow* aFrame, bool ( *aIsLooking )( int aId ) );

    /// @brief aId only looks in aWindow's top-level window (wx's own help / copy / find ids anywhere)
    static bool IsLookingCommand( wxWindow* aWindow, int aId );

    /// @brief Participants and their state, for remote_session status
    static nlohmann::json Cascade();
};

#endif
