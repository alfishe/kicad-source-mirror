/// @file kicadopenapi_remote.cpp
/// @brief Remote control sessions (see kicadopenapi_remote.h).
#include <kicadopenapi_remote.h>
#include <kicadopenapi_service.h>

#include <kiplatform/ui.h>

#include <wx/app.h>
#include <wx/button.h>
#include <wx/eventfilter.h>
#include <wx/frame.h>
#include <wx/intl.h>
#include <wx/log.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/timer.h>
#include <wx/toplevel.h>
#include <wx/weakref.h>

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <random>


namespace
{
using CLOCK = std::chrono::steady_clock;

constexpr int DEFAULT_LEASE_S = 60;
constexpr int COOLDOWN_S = 300;


struct SESSION_STATE
{
    std::mutex                                 mutex;
    bool                                       held = false;
    std::string                                holder;        ///< caller key
    std::string                                client;        ///< display name
    std::string                                peer;
    std::chrono::system_clock::time_point      since;
    CLOCK::time_point                          lastCall;
    int                                        leaseS = DEFAULT_LEASE_S;
    std::map<std::string, CLOCK::time_point>   revoked;       ///< caller key -> cooldown end
    std::string                                lastRevokedClient;
    std::atomic<bool>                          lockedUi{ false };
};


SESSION_STATE& state()
{
    static SESSION_STATE* s_state = new SESSION_STATE();   // process lifetime: used from HTTP threads
    return *s_state;
}


std::string newToken()
{
    std::random_device rd;
    std::mt19937_64    gen( rd() );
    char               buf[33];
    std::snprintf( buf, sizeof( buf ), "%016llx%016llx", (unsigned long long) gen(), (unsigned long long) gen() );
    return std::string( "rs-" ) + buf;
}


void updateUiLater();


/// @brief Ends an idle session (lock held)
bool expireLocked( SESSION_STATE& s )
{
    if( s.held && s.leaseS > 0 && CLOCK::now() - s.lastCall > std::chrono::seconds( s.leaseS ) )
    {
        s.held = false;
        return true;
    }

    return false;
}


// ---------------------------------------------------------------------------------------------
// UI: the read-only filter and the bar over every KiCad window (main thread)

class REMOTE_BAR;


/// @brief The bar's frame and its parent's id, so that input to it stays allowed
bool isBarWindow( wxWindow* aWindow );


/// @brief Keys that only look: navigation, zoom, copy, find, escape
bool isLookingKey( const wxKeyEvent& aEvent )
{
    switch( aEvent.GetKeyCode() )
    {
    case WXK_LEFT: case WXK_RIGHT: case WXK_UP: case WXK_DOWN:
    case WXK_PAGEUP: case WXK_PAGEDOWN: case WXK_HOME: case WXK_END:
    case WXK_TAB: case WXK_ESCAPE: case WXK_SHIFT: case WXK_CONTROL: case WXK_ALT:
    case WXK_RAW_CONTROL: case WXK_F1: case WXK_F2: case WXK_F3: case WXK_F4:
        return true;
    default:
        break;
    }

    // copy and find
    if( aEvent.ControlDown() && ( aEvent.GetKeyCode() == 'C' || aEvent.GetKeyCode() == 'F' ) )
        return true;

    return false;
}


class READONLY_FILTER : public wxEventFilter
{
public:
    int FilterEvent( wxEvent& aEvent ) override
    {
        if( !state().lockedUi.load( std::memory_order_relaxed ) )
            return Event_Skip;

        // the API's own work (and the input it injects) is never filtered
        if( KICAD_OPENAPI_SERVICE::InMainThreadCall() )
            return Event_Skip;

        const wxEventType type = aEvent.GetEventType();
        wxWindow*         window = dynamic_cast<wxWindow*>( aEvent.GetEventObject() );

        if( window && isBarWindow( window ) )
            return Event_Skip;

        // looking: pointer moves, wheel (zoom / scroll), middle button (pan)
        if( type == wxEVT_MOTION || type == wxEVT_MOUSEWHEEL || type == wxEVT_MAGNIFY || type == wxEVT_MIDDLE_DOWN
            || type == wxEVT_MIDDLE_UP || type == wxEVT_ENTER_WINDOW || type == wxEVT_LEAVE_WINDOW )
        {
            return Event_Skip;
        }

        const bool click = type == wxEVT_LEFT_DOWN || type == wxEVT_LEFT_UP || type == wxEVT_LEFT_DCLICK
                           || type == wxEVT_RIGHT_DOWN || type == wxEVT_RIGHT_UP || type == wxEVT_RIGHT_DCLICK
                           || type == wxEVT_AUX1_DOWN || type == wxEVT_AUX2_DOWN;

        const bool key = type == wxEVT_KEY_DOWN || type == wxEVT_CHAR || type == wxEVT_CHAR_HOOK;

        // menu and toolbar commands of the user (the API's run inside a call, above)
        const bool command = type == wxEVT_MENU || type == wxEVT_TOOL;

        if( key && isLookingKey( static_cast<wxKeyEvent&>( aEvent ) ) )
            return Event_Skip;

        if( click || key || command )
        {
            m_refused++;
            return Event_Processed;
        }

        return Event_Skip;
    }

    std::atomic<long> m_refused{ 0 };
};


READONLY_FILTER* g_filter = nullptr;


/// @brief The bar over one KiCad window: a small frame floating on it, at the top centre
class REMOTE_BAR : public wxFrame
{
public:
    explicit REMOTE_BAR( wxTopLevelWindow* aParent ) :
            wxFrame( aParent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,
                     wxFRAME_FLOAT_ON_PARENT | wxFRAME_NO_TASKBAR | wxBORDER_NONE | wxFRAME_TOOL_WINDOW ),
            m_parent( aParent )
    {
        m_panel = new wxPanel( this );
        auto* sizer = new wxBoxSizer( wxHORIZONTAL );
        m_text = new wxStaticText( m_panel, wxID_ANY, wxEmptyString );
        m_text->SetFont( m_text->GetFont().Bold().Scaled( 1.1f ) );
        m_action = new wxButton( m_panel, wxID_ANY, _( "Take Back Control" ) );
        m_dismiss = new wxButton( m_panel, wxID_ANY, _( "Dismiss" ) );
        sizer->Add( m_text, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, 14 );
        sizer->Add( m_action, 0, wxALIGN_CENTER_VERTICAL | wxTOP | wxBOTTOM | wxRIGHT, 6 );
        sizer->Add( m_dismiss, 0, wxALIGN_CENTER_VERTICAL | wxTOP | wxBOTTOM | wxRIGHT, 6 );
        m_panel->SetSizer( sizer );

        m_action->Bind( wxEVT_BUTTON, [this]( wxCommandEvent& ) { onAction(); } );
        m_dismiss->Bind( wxEVT_BUTTON, [this]( wxCommandEvent& ) { Hide(); m_dismissed = true; } );

        aParent->Bind( wxEVT_MOVE, &REMOTE_BAR::onParentMoved, this );
        aParent->Bind( wxEVT_SIZE, &REMOTE_BAR::onParentMoved, this );
    }

    ~REMOTE_BAR() override
    {
        if( m_parent )
        {
            m_parent->Unbind( wxEVT_MOVE, &REMOTE_BAR::onParentMoved, this );
            m_parent->Unbind( wxEVT_SIZE, &REMOTE_BAR::onParentMoved, this );
        }
    }

    /// @param aHeld the session runs (else it was taken back)
    void Present( const wxString& aText, bool aHeld )
    {
        if( m_dismissed && !aHeld )
            return;

        if( aHeld )
            m_dismissed = false;

        m_held = aHeld;
        m_text->SetLabel( aText );
        m_action->SetLabel( aHeld ? _( "Take Back Control" ) : _( "Allow Automation Again" ) );
        m_dismiss->Show( !aHeld );

        const wxColour bg = aHeld ? wxColour( 0xD9, 0x6A, 0x00 ) : wxColour( 0x3A, 0x45, 0x52 );
        m_panel->SetBackgroundColour( bg );
        m_text->SetForegroundColour( *wxWHITE );
        m_panel->Layout();
        m_panel->GetSizer()->Fit( this );
        place();
        wxFrame::Show( true );
        Refresh();
    }

    wxTopLevelWindow* Parent() const { return m_parent; }

private:
    void place()
    {
        if( !m_parent )
            return;

        const wxRect r = m_parent->GetScreenRect();
        const wxSize s = GetSize();
        SetPosition( wxPoint( r.x + ( r.width - s.x ) / 2, r.y + 64 ) );
    }

    void onParentMoved( wxEvent& aEvent )
    {
        aEvent.Skip();

        if( IsShown() )
            place();
    }

    void onAction();

    wxWeakRef<wxTopLevelWindow> m_parent;
    wxPanel*                    m_panel = nullptr;
    wxStaticText*               m_text = nullptr;
    wxButton*                   m_action = nullptr;
    wxButton*                   m_dismiss = nullptr;
    bool                        m_held = true;
    bool                        m_dismissed = false;
};


/// @brief One bar per KiCad window; a timer keeps the clock running, catches new windows and expiry
class UI : public wxEvtHandler
{
public:
    UI()
    {
        m_timer.SetOwner( this );
        Bind( wxEVT_TIMER, [this]( wxTimerEvent& ) { Update(); }, m_timer.GetId() );
    }

    void Update();

    bool IsBar( wxWindow* aWindow ) const
    {
        wxWindow* top = wxGetTopLevelParent( aWindow );

        for( const auto& [parent, bar] : m_bars )
        {
            if( bar && top == bar.get() )
                return true;
        }

        return false;
    }

    void Clear()
    {
        m_timer.Stop();

        for( auto& [parent, bar] : m_bars )
        {
            if( bar )
                bar->Destroy();
        }

        m_bars.clear();
    }

private:
    wxTimer                                            m_timer;
    std::map<wxTopLevelWindow*, wxWeakRef<REMOTE_BAR>> m_bars;
};


UI& ui()
{
    static UI* s_ui = new UI();
    return *s_ui;
}


bool isBarWindow( wxWindow* aWindow )
{
    return ui().IsBar( aWindow );
}


wxString elapsed( std::chrono::system_clock::time_point aSince )
{
    const long s = (long) std::chrono::duration_cast<std::chrono::seconds>( std::chrono::system_clock::now() - aSince ).count();
    return wxString::Format( "%02ld:%02ld", s / 60, s % 60 );
}


void UI::Update()
{
    SESSION_STATE& s = state();
    bool           held;
    wxString       text;
    bool           takenBack;

    {
        std::lock_guard<std::mutex> lock( s.mutex );

        if( expireLocked( s ) )
            wxLogMessage( "remote control: the session of '%s' expired (no call for %d s)", s.client, s.leaseS );

        held = s.held;
        takenBack = !held && !s.lastRevokedClient.empty();

        if( held )
        {
            text = wxString::Format( _( "Remote control: %s%s · %s" ), wxString::FromUTF8( s.client ),
                                     s.peer.empty() ? wxString() : wxString::FromUTF8( " · " + s.peer ),
                                     elapsed( s.since ) );
        }
        else if( takenBack )
        {
            text = wxString::Format( _( "Control taken back from %s" ), wxString::FromUTF8( s.lastRevokedClient ) );
        }
    }

    state().lockedUi.store( held );

    if( held && !g_filter )
    {
        g_filter = new READONLY_FILTER();
        wxEvtHandler::AddFilter( g_filter );
    }

    // drop bars of closed windows
    for( auto it = m_bars.begin(); it != m_bars.end(); )
    {
        if( !it->second || !it->second->Parent() )
        {
            if( it->second )
                it->second->Destroy();

            it = m_bars.erase( it );
        }
        else
        {
            ++it;
        }
    }

    if( !held && !takenBack )
    {
        Clear();

        if( g_filter )
        {
            wxEvtHandler::RemoveFilter( g_filter );
            delete g_filter;
            g_filter = nullptr;
        }

        return;
    }

    // every KiCad window (frames, not dialogs or our bars) gets a bar
    for( wxWindow* window : wxTopLevelWindows )
    {
        auto* frame = dynamic_cast<wxFrame*>( window );

        // KiCad's windows, not our bars or small floating panels
        if( !frame || dynamic_cast<REMOTE_BAR*>( frame ) || !frame->IsShown() || frame->IsIconized()
            || frame->GetSize().x < 600 )
        {
            continue;
        }

        wxWeakRef<REMOTE_BAR>& bar = m_bars[frame];

        if( !bar )
            bar = new REMOTE_BAR( frame );

        bar->Present( text, held );
    }

    if( !m_timer.IsRunning() )
        m_timer.Start( 1000 );
}


void updateUiLater()
{
    if( wxTheApp )
        wxTheApp->CallAfter( []() { ui().Update(); } );
}


/// @brief The user takes control back (main thread)
void revoke()
{
    SESSION_STATE& s = state();

    {
        std::lock_guard<std::mutex> lock( s.mutex );

        if( !s.held )
            return;

        s.held = false;
        s.revoked[s.holder] = CLOCK::now() + std::chrono::seconds( COOLDOWN_S );
        s.lastRevokedClient = s.client;
        wxLogMessage( "remote control: taken back from '%s' by the user", s.client );
    }

    // nothing the automation was pressing stays pressed
    KIPLATFORM::UI::SetInjectedPointer( nullptr );
    ui().Update();
}


void allowAgain()
{
    SESSION_STATE& s = state();

    {
        std::lock_guard<std::mutex> lock( s.mutex );
        s.revoked.clear();
        s.lastRevokedClient.clear();
    }

    ui().Update();
}


void REMOTE_BAR::onAction()
{
    if( m_held )
        revoke();
    else
        allowAgain();
}


nlohmann::json statusLocked( const SESSION_STATE& s, const std::string& aCaller )
{
    nlohmann::json cooldowns = nlohmann::json::array();

    for( const auto& [key, until] : s.revoked )
    {
        const long left = (long) std::chrono::duration_cast<std::chrono::seconds>( until - CLOCK::now() ).count();

        if( left > 0 )
            cooldowns.push_back( { { "session", key }, { "seconds_left", left } } );
    }

    const bool mine = s.held && !aCaller.empty() && s.holder == aCaller;

    return { { "held", s.held },
             { "yours", mine },
             { "client", s.held ? nlohmann::json( s.client ) : nlohmann::json() },
             { "peer", s.held && !s.peer.empty() ? nlohmann::json( s.peer ) : nlohmann::json() },
             { "since_s", s.held ? nlohmann::json( (long) std::chrono::duration_cast<std::chrono::seconds>(
                                                           std::chrono::system_clock::now() - s.since ).count() )
                                 : nlohmann::json() },
             { "lease_s", s.held ? nlohmann::json( s.leaseS ) : nlohmann::json() },
             { "revoked_cooldowns", cooldowns } };
}


nlohmann::json revokedBody()
{
    return { { "error", { { "code", 409 },
                          { "reason", "revoked_by_user" },
                          { "retry", false },
                          { "message", "the user took control of KiCad back: stop, do not retry, and ask the user "
                                       "before taking control again (remote_session take)" } } } };
}
} // namespace


std::optional<KOPENAPI_RESULT> KOPENAPI_REMOTE::Gate( const std::string& aCaller, const std::string& aMethod )
{
    if( aCaller.empty() )
        return std::nullopt;

    SESSION_STATE& s = state();
    std::lock_guard<std::mutex> lock( s.mutex );

    if( auto it = s.revoked.find( aCaller ); it != s.revoked.end() )
    {
        if( CLOCK::now() >= it->second )
        {
            s.revoked.erase( it );
        }
        else if( aMethod != "remote_session" )
        {
            KOPENAPI_RESULT refused;
            refused.status = 409;
            refused.body = revokedBody();
            return refused;
        }
    }

    if( s.held && s.holder == aCaller )
        s.lastCall = CLOCK::now();

    return std::nullopt;
}


KOPENAPI_RESULT KOPENAPI_REMOTE::Handle( const std::string& aCaller, const nlohmann::json& aArgs )
{
    const std::string action = aArgs.value( "action", std::string( "status" ) );
    SESSION_STATE&    s = state();
    nlohmann::json    out;

    {
        std::lock_guard<std::mutex> lock( s.mutex );
        expireLocked( s );

        if( action == "status" )
            return KOPENAPI_RESULT::Ok( statusLocked( s, aCaller ) );

        if( action == "take" )
        {
            const std::string key = aCaller.empty() ? newToken() : aCaller;

            if( auto it = s.revoked.find( key ); it != s.revoked.end() && CLOCK::now() < it->second )
            {
                KOPENAPI_RESULT refused;
                refused.status = 409;
                refused.body = revokedBody();
                refused.body["error"]["reason"] = "revoked_cooldown";
                return refused;
            }

            if( s.held && s.holder != key )
            {
                KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "KiCad is controlled by '" + s.client
                                                                            + "'; it must release it first" );
                refused.body["error"]["reason"] = "held_by_other";
                refused.body["error"]["retry"] = false;
                return refused;
            }

            const bool again = s.held;
            s.held = true;
            s.holder = key;
            s.client = aArgs.value( "client", std::string( "agent" ) ).substr( 0, 60 );
            s.peer = aArgs.value( "peer", std::string() ).substr( 0, 80 );
            s.leaseS = std::clamp( aArgs.value( "lease_s", DEFAULT_LEASE_S ), 0, 3600 );
            s.lastCall = CLOCK::now();

            if( !again )
                s.since = std::chrono::system_clock::now();

            s.lastRevokedClient.clear();
            out = statusLocked( s, key );
            out["session"] = key;
            out["header"] = "X-Kicad-Session";
        }
        else if( action == "release" )
        {
            const bool mine = s.held && ( s.holder == aCaller || aArgs.value( "session", std::string() ) == s.holder );

            if( mine )
                s.held = false;

            out = { { "released", mine } };
        }
        else if( action == "allow_again" || action == "take_back" )
        {
            // what the user does with the bar's buttons, for tests and recordings
        }
        else
        {
            return KOPENAPI_RESULT::Error( 400, "action is take, release, status, take_back or allow_again" );
        }
    }

    if( action == "take_back" || action == "allow_again" )
    {
        if( !wxTheApp )
            return KOPENAPI_RESULT::Error( 501, "remote control needs the GUI" );

        wxTheApp->CallAfter( [action]() { action == "take_back" ? revoke() : allowAgain(); } );
        out = { { "done", true } };
    }

    updateUiLater();
    return KOPENAPI_RESULT::Ok( out );
}


void KOPENAPI_REMOTE::Shutdown()
{
    {
        std::lock_guard<std::mutex> lock( state().mutex );
        state().held = false;
        state().lastRevokedClient.clear();
    }

    state().lockedUi.store( false );

    if( wxTheApp && wxThread::IsMain() )
    {
        ui().Clear();

        if( g_filter )
        {
            wxEvtHandler::RemoveFilter( g_filter );
            delete g_filter;
            g_filter = nullptr;
        }
    }
}


bool KOPENAPI_REMOTE::Held()
{
    std::lock_guard<std::mutex> lock( state().mutex );
    return state().held;
}


static KOPENAPI_RESULT h_remote_session( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    // the service answers remote_session itself, on the HTTP thread (see invokeParsed)
    return KOPENAPI_RESULT::Error( 500, "remote_session is served by the service" );
}


KOPENAPI_REGISTER( "remote_session",
                   "Remote control session: take shows the user a bar over every KiCad window (\"Remote control: "
                   "<client>\") and makes the UI read-only for them (looking stays: scroll, zoom, pan, window "
                   "moves; clicks, keys, menu and toolbar commands are refused) while your calls drive it. "
                   "Identified by the X-Kicad-Session header (the MCP bridge sends one; else take answers a "
                   "session to send). Your calls renew the lease (lease_s, default 60, 0 = none); release ends "
                   "it. If the user presses Take Back Control your calls get 409 revoked_by_user: stop and ask "
                   "the user. status, take, release; take_back / allow_again do what the bar's buttons do",
                   R"json({"type":"object","properties":{
                        "action":{"type":"string","enum":["take","release","status","take_back","allow_again"],"default":"status"},
                        "client":{"type":"string","description":"name shown to the user, e.g. the agent's name"},
                        "peer":{"type":"string","description":"where it runs, shown with the name"},
                        "lease_s":{"type":"integer","default":60,"description":"the session ends after this long without a call; 0 = never"},
                        "session":{"type":"string","description":"release: the session from take, when no header is sent"}}})json"_json,
                   false, h_remote_session );
