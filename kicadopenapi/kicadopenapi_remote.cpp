/// @file kicadopenapi_remote.cpp
/// @brief Remote control sessions (see kicadopenapi_remote.h).
#include <kicadopenapi_remote.h>
#include <kicadopenapi_service.h>

#include <kiplatform/ui.h>

#include <wx/app.h>
#include <wx/aui/auibar.h>
#include <wx/aui/framemanager.h>
#include <wx/button.h>
#include <wx/eventfilter.h>
#include <wx/frame.h>
#include <wx/menu.h>
#include <wx/intl.h>
#include <wx/log.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/timer.h>
#include <wx/toplevel.h>
#include <wx/weakref.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <unordered_set>
#include <vector>


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
    std::atomic<long long>                     lastRefusedMs{ 0 };   ///< steady clock, user input refused
    std::atomic<bool>                          hintPending{ false };
    nlohmann::json                             cascade = nlohmann::json::array();  ///< published by the main thread
};


constexpr int REFUSED_HINT_MS = 3000;


long long nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>( CLOCK::now().time_since_epoch() ).count();
}


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


wxWindow* commandWindow( wxEvent& aEvent );


/// @brief Keys that only look: navigation, zoom, copy, find, escape
bool isLookingKey( const wxKeyEvent& aEvent )
{
    // WXK_RAW_CONTROL equals WXK_CONTROL outside macOS: not a case label
    if( aEvent.GetKeyCode() == WXK_RAW_CONTROL )
        return true;

    switch( aEvent.GetKeyCode() )
    {
    case WXK_LEFT: case WXK_RIGHT: case WXK_UP: case WXK_DOWN:
    case WXK_PAGEUP: case WXK_PAGEDOWN: case WXK_HOME: case WXK_END:
    case WXK_TAB: case WXK_ESCAPE: case WXK_SHIFT: case WXK_CONTROL: case WXK_ALT:
    case WXK_F1: case WXK_F2: case WXK_F3: case WXK_F4:
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

        // looking controls (the toolbars' grid and zoom choices) and what they are made of
        for( wxWindow* w = window; w && !w->IsTopLevel(); w = w->GetParent() )
        {
            if( KOPENAPI_REMOTE::IsLookingCommand( w, w->GetId() ) )
                return Event_Skip;
        }

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
        const bool command = ( type == wxEVT_MENU || type == wxEVT_TOOL )
                             && !KOPENAPI_REMOTE::IsLookingCommand( commandWindow( aEvent ), aEvent.GetId() );

        if( key && isLookingKey( static_cast<wxKeyEvent&>( aEvent ) ) )
            return Event_Skip;

        if( click || key || command )
        {
            m_refused++;
            state().lastRefusedMs.store( nowMs() );

            if( !state().hintPending.exchange( true ) )
                updateUiLater();

            return Event_Processed;
        }

        return Event_Skip;
    }

    std::atomic<long> m_refused{ 0 };
};


READONLY_FILTER* g_filter = nullptr;


const std::unordered_set<int>& wxLookingIds()
{
    static const std::unordered_set<int> s_ids = { wxID_ABOUT, wxID_HELP, wxID_HELP_CONTENTS, wxID_HELP_INDEX,
                                                   wxID_EXIT, wxID_COPY, wxID_FIND, wxID_ZOOM_IN, wxID_ZOOM_OUT,
                                                   wxID_ZOOM_FIT, wxID_ZOOM_100 };
    return s_ids;
}


/// @brief Per window: its module's "only looks" test (main thread)
std::map<wxWindow*, bool ( * )( int )>& lookingResolvers()
{
    static std::map<wxWindow*, bool ( * )( int )> s_resolvers;
    return s_resolvers;
}


/// @brief The window a command event is for: the event's window, a menu's window, else the focus'
wxWindow* commandWindow( wxEvent& aEvent )
{
    wxObject* object = aEvent.GetEventObject();

    if( auto* window = dynamic_cast<wxWindow*>( object ) )
        return window;

    if( auto* menu = dynamic_cast<wxMenu*>( object ) )
    {
        if( wxWindow* window = menu->GetWindow() )
            return window;
    }

    if( wxWindow* focus = wxWindow::FindFocus() )
        return focus;

    return wxTheApp ? wxTheApp->GetTopWindow() : nullptr;
}


/// @brief The lock cascade: participants engaged in order when the user's input gets refused,
/// released in reverse when it ends; each step guarded, so one failing step never stops the others
class CASCADE
{
public:
    void Add( KOPENAPI_REMOTE_PARTICIPANT* aParticipant, int aOrder )
    {
        Remove( aParticipant );
        STEP step{ aParticipant, aOrder, false, 0 };
        auto it = std::upper_bound( m_steps.begin(), m_steps.end(), aOrder,
                                    []( int aO, const STEP& aS ) { return aO < aS.order; } );
        it = m_steps.insert( it, step );

        if( m_locked )
            engage( *it );

        publish();
    }

    void Remove( KOPENAPI_REMOTE_PARTICIPANT* aParticipant )
    {
        for( auto it = m_steps.begin(); it != m_steps.end(); ++it )
        {
            if( it->participant == aParticipant )
            {
                if( it->engaged )
                    release( *it );

                m_steps.erase( it );
                publish();
                return;
            }
        }
    }

    /// @brief Switches the lock: flag first, then the steps (engage ascending, release descending)
    void SetLocked( bool aLocked )
    {
        if( aLocked == m_locked )
        {
            if( m_locked )
                scanWindows();

            return;
        }

        m_locked = aLocked;
        state().lockedUi.store( aLocked );

        if( aLocked )
        {
            m_windows.clear();

            for( wxWindow* window : wxTopLevelWindows )
                m_windows.emplace_back( window );

            for( STEP& step : m_steps )
                engage( step );
        }
        else
        {
            for( auto it = m_steps.rbegin(); it != m_steps.rend(); ++it )
                release( *it );

            m_windows.clear();
        }

        publish();
    }

    bool Locked() const { return m_locked; }

    nlohmann::json Json() const
    {
        nlohmann::json out = nlohmann::json::array();

        for( const STEP& step : m_steps )
        {
            out.push_back( { { "name", step.participant->Name() }, { "order", step.order },
                             { "engaged", step.engaged }, { "failures", step.failures } } );
        }

        return out;
    }

private:
    void publish()
    {
        nlohmann::json json = Json();
        std::lock_guard<std::mutex> lock( state().mutex );
        state().cascade = std::move( json );
    }

    struct STEP
    {
        KOPENAPI_REMOTE_PARTICIPANT* participant;
        int                          order;
        bool                         engaged;
        int                          failures;
    };

    template <typename FN>
    bool guarded( STEP& aStep, const char* aWhat, FN&& aFn )
    {
        try
        {
            aFn();
            return true;
        }
        catch( const std::exception& e )
        {
            aStep.failures++;
            wxLogMessage( "remote control: %s of '%s' failed: %s", aWhat, aStep.participant->Name(), e.what() );
        }
        catch( ... )
        {
            aStep.failures++;
            wxLogMessage( "remote control: %s of '%s' failed", aWhat, aStep.participant->Name() );
        }

        return false;
    }

    void engage( STEP& aStep )
    {
        aStep.engaged = guarded( aStep, "engage", [&]() { aStep.participant->Engage(); } );
    }

    void release( STEP& aStep )
    {
        if( aStep.engaged )
            guarded( aStep, "release", [&]() { aStep.participant->Release(); } );

        aStep.engaged = false;
    }

    /// @brief Windows that appeared while locked are handed to the engaged steps
    void scanWindows()
    {
        m_windows.erase( std::remove_if( m_windows.begin(), m_windows.end(),
                                         []( const wxWeakRef<wxWindow>& aW ) { return !aW; } ),
                         m_windows.end() );

        for( wxWindow* window : wxTopLevelWindows )
        {
            auto* top = dynamic_cast<wxTopLevelWindow*>( window );

            if( !top || isBarWindow( top ) )
                continue;

            bool known = false;

            for( const wxWeakRef<wxWindow>& w : m_windows )
                known = known || w.get() == window;

            if( known )
                continue;

            m_windows.emplace_back( window );

            for( STEP& step : m_steps )
            {
                if( step.engaged )
                    guarded( step, "window", [&]() { step.participant->WindowAdded( top ); } );
            }
        }
    }

    std::vector<STEP>                m_steps;
    std::vector<wxWeakRef<wxWindow>> m_windows;
    bool                             m_locked = false;
};


CASCADE& cascade()
{
    static CASCADE* s_cascade = new CASCADE();
    return *s_cascade;
}


/// @brief Step: the application event filter refusing the user's clicks, keys and commands
class FILTER_STEP : public KOPENAPI_REMOTE_PARTICIPANT
{
public:
    const char* Name() const override { return "input_filter"; }

    void Engage() override
    {
        if( !g_filter )
        {
            g_filter = new READONLY_FILTER();
            wxEvtHandler::AddFilter( g_filter );
        }
    }

    void Release() override
    {
        if( g_filter )
        {
            wxEvtHandler::RemoveFilter( g_filter );
            delete g_filter;
            g_filter = nullptr;
        }
    }
};


/// @brief Step: KiCad's UI conditions (toolbars, menus of actions) re-evaluated. They add "not under
/// remote control" to every editing action themselves, so releasing restores each control's own state.
class CONDITIONS_STEP : public KOPENAPI_REMOTE_PARTICIPANT
{
public:
    const char* Name() const override { return "ui_conditions"; }

    void Engage() override { refresh(); }

    void Release() override { refresh(); }

    void WindowAdded( wxTopLevelWindow* aWindow ) override { aWindow->UpdateWindowUI( wxUPDATE_UI_RECURSE ); }

private:
    static void refresh()
    {
        for( wxWindow* window : wxTopLevelWindows )
        {
            if( !isBarWindow( window ) )
                window->UpdateWindowUI( wxUPDATE_UI_RECURSE );
        }
    }
};


/// @brief Step: menu items. Disables the enabled ones that do not only look and remembers exactly
/// those; release re-enables an item only if it is still the same item, disabled as we left it.
/// Items with UI conditions are re-evaluated afterwards (CONDITIONS_STEP releases later).
class MENUS_STEP : public KOPENAPI_REMOTE_PARTICIPANT
{
public:
    const char* Name() const override { return "menus"; }

    void Engage() override
    {
        m_items.clear();

        for( wxWindow* window : wxTopLevelWindows )
        {
            if( auto* top = dynamic_cast<wxTopLevelWindow*>( window ) )
                WindowAdded( top );
        }
    }

    void Release() override
    {
        for( ITEM& item : m_items )
        {
            if( !item.frame )
                continue;

            wxMenuBar* bar = item.frame->GetMenuBar();

            // the menu bar may have been rebuilt meanwhile: only the same item, still as we left it
            if( bar && bar->FindItem( item.id ) == item.item && !item.item->IsEnabled() )
                item.item->Enable( true );
        }

        m_items.clear();
    }

    void WindowAdded( wxTopLevelWindow* aWindow ) override
    {
        auto* frame = dynamic_cast<wxFrame*>( aWindow );

        if( !frame || isBarWindow( frame ) || !frame->GetMenuBar() )
            return;

        wxMenuBar* bar = frame->GetMenuBar();

        for( size_t i = 0; i < bar->GetMenuCount(); ++i )
            addMenu( frame, bar->GetMenu( i ) );
    }

private:
    struct ITEM
    {
        wxWeakRef<wxFrame> frame;
        wxMenuItem*        item;
        int                id;
    };

    void addMenu( wxFrame* aFrame, wxMenu* aMenu )
    {
        for( wxMenuItem* item : aMenu->GetMenuItems() )
        {
            if( item->IsSubMenu() )
            {
                addMenu( aFrame, item->GetSubMenu() );
                continue;
            }

            if( item->IsSeparator() || !item->IsEnabled()
                || KOPENAPI_REMOTE::IsLookingCommand( aFrame, item->GetId() ) )
            {
                continue;
            }

            item->Enable( false );
            m_items.push_back( { aFrame, item, item->GetId() } );
        }
    }

    std::vector<ITEM> m_items;
};


/// @brief Step: toolbar tools and the controls embedded in toolbars (track width, via size, layer;
/// grid and zoom choices only look and stay). Same snapshot rule as the menus.
class TOOLBARS_STEP : public KOPENAPI_REMOTE_PARTICIPANT
{
public:
    const char* Name() const override { return "toolbars"; }

    void Engage() override
    {
        m_tools.clear();
        m_controls.clear();

        for( wxWindow* window : wxTopLevelWindows )
        {
            if( auto* top = dynamic_cast<wxTopLevelWindow*>( window ) )
                WindowAdded( top );
        }
    }

    void Release() override
    {
        std::set<wxAuiToolBar*> changed;

        for( wxWeakRef<wxWindow>& control : m_controls )
        {
            if( control && !control->IsEnabled() )
                control->Enable( true );
        }

        m_controls.clear();

        for( TOOL& tool : m_tools )
        {
            if( !tool.bar )
                continue;

            wxAuiToolBarItem* item = tool.bar->FindTool( tool.id );

            if( item && !item->GetWindow() && ( item->GetState() & wxAUI_BUTTON_STATE_DISABLED ) )
            {
                tool.bar->EnableTool( tool.id, true );
                changed.insert( tool.bar.get() );
            }
        }

        for( wxAuiToolBar* bar : changed )
            bar->Refresh();

        m_tools.clear();
    }

    void WindowAdded( wxTopLevelWindow* aWindow ) override
    {
        if( !isBarWindow( aWindow ) )
            addBars( aWindow );
    }

private:
    struct TOOL
    {
        wxWeakRef<wxAuiToolBar> bar;
        int                     id;
    };

    void addBars( wxWindow* aWindow )
    {
        for( wxWindow* child : aWindow->GetChildren() )
        {
            auto* bar = dynamic_cast<wxAuiToolBar*>( child );

            if( !bar )
            {
                addBars( child );
                continue;
            }

            bool changed = false;

            for( int i = 0; i < bar->GetToolCount(); ++i )
            {
                wxAuiToolBarItem* item = bar->FindToolByIndex( i );

                if( wxWindow* control = item ? item->GetWindow() : nullptr )
                {
                    if( control->IsEnabled() && !KOPENAPI_REMOTE::IsLookingCommand( bar, item->GetId() )
                        && !KOPENAPI_REMOTE::IsLookingCommand( bar, control->GetId() ) )
                    {
                        control->Enable( false );
                        m_controls.emplace_back( control );
                    }

                    continue;
                }

                if( !item || item->GetKind() == wxITEM_SEPARATOR || item->GetId() == wxID_ANY
                    || ( item->GetState() & wxAUI_BUTTON_STATE_DISABLED )
                    || KOPENAPI_REMOTE::IsLookingCommand( bar, item->GetId() ) )
                {
                    continue;
                }

                bar->EnableTool( item->GetId(), false );
                m_tools.push_back( { bar, item->GetId() } );
                changed = true;
            }

            if( changed )
                bar->Refresh();
        }
    }

    std::vector<TOOL>                m_tools;
    std::vector<wxWeakRef<wxWindow>> m_controls;
};


void installBuiltinSteps()
{
    static FILTER_STEP     s_filter;
    static CONDITIONS_STEP s_conditions;
    static MENUS_STEP      s_menus;
    static TOOLBARS_STEP   s_toolbars;
    static bool            s_done = false;

    if( s_done )
        return;

    s_done = true;
    cascade().Add( &s_filter, 100 );
    cascade().Add( &s_conditions, 200 );
    cascade().Add( &s_menus, 300 );
    cascade().Add( &s_toolbars, 400 );
}


/// @brief The bar over one KiCad window: a small frame floating on it, at the top centre
class REMOTE_BAR : public wxFrame
{
public:
    explicit REMOTE_BAR( wxTopLevelWindow* aParent ) :
            wxFrame( aParent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,
                     KIPLATFORM::UI::DecorationWindowStyle() ),
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
        m_dismiss->Bind( wxEVT_BUTTON,
                         [this]( wxCommandEvent& )
                         {
                             KIPLATFORM::UI::DetachChildWindow( this );
                             Hide();
                             m_dismissed = true;
                         } );

        aParent->Bind( wxEVT_MOVE, &REMOTE_BAR::onParentMoved, this );
        aParent->Bind( wxEVT_SIZE, &REMOTE_BAR::onParentMoved, this );
    }

    ~REMOTE_BAR() override
    {
        KIPLATFORM::UI::DetachChildWindow( this );

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

        // Only when hidden: showing a shown window brings it to the front (macOS orders it
        // above every window), the bar must stay just above its own window
        if( !IsShown() )
        {
            wxFrame::Show( true );

            // A child window moves with its parent by the window system itself (macOS): the
            // parent's move events arrive throttled during a drag, so following them lags
            if( m_parent )
                KIPLATFORM::UI::AttachChildWindow( this, m_parent.get() );
        }

        Refresh();
    }

    wxTopLevelWindow* Parent() const { return m_parent; }

    /// @brief Hide and destroy: detached from its parent first, else the window system shows a
    /// hidden child window again when the parent comes to the front (macOS)
    void Dispose()
    {
        KIPLATFORM::UI::DetachChildWindow( this );
        Hide();
        Destroy();
    }

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
                bar->Dispose();
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

    s.hintPending.store( false );
    const bool refusedRecently = nowMs() - s.lastRefusedMs.load() < REFUSED_HINT_MS;

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

            if( refusedRecently )
                text += wxS( "  —  " ) + _( "take control back to edit" );
        }
        else if( takenBack )
        {
            text = wxString::Format( _( "Control taken back from %s" ), wxString::FromUTF8( s.lastRevokedClient ) );
        }
    }

    installBuiltinSteps();
    cascade().SetLocked( held );

    // drop bars of closed windows
    for( auto it = m_bars.begin(); it != m_bars.end(); )
    {
        if( !it->second || !it->second->Parent() )
        {
            if( it->second )
                it->second->Dispose();

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
             { "revoked_cooldowns", cooldowns },
             { "ui_locked", s.lockedUi.load() },
             { "cascade", s.cascade } };
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

    if( wxTheApp && wxThread::IsMain() )
    {
        cascade().SetLocked( false );
        ui().Clear();
    }
    else
    {
        state().lockedUi.store( false );
    }
}


void KOPENAPI_REMOTE::AddParticipant( KOPENAPI_REMOTE_PARTICIPANT* aParticipant, int aOrder )
{
    installBuiltinSteps();
    cascade().Add( aParticipant, aOrder );
}


void KOPENAPI_REMOTE::RemoveParticipant( KOPENAPI_REMOTE_PARTICIPANT* aParticipant )
{
    cascade().Remove( aParticipant );
}


nlohmann::json KOPENAPI_REMOTE::Cascade()
{
    installBuiltinSteps();
    return cascade().Json();
}


void KOPENAPI_REMOTE::SetLookingCommands( wxWindow* aFrame, bool ( *aIsLooking )( int aId ) )
{
    if( aIsLooking )
        lookingResolvers()[aFrame] = aIsLooking;
    else
        lookingResolvers().erase( aFrame );
}


bool KOPENAPI_REMOTE::IsLookingCommand( wxWindow* aWindow, int aId )
{
    if( wxLookingIds().count( aId ) )
        return true;

    if( !aWindow )
        return false;

    auto& resolvers = lookingResolvers();
    auto  it = resolvers.find( wxGetTopLevelParent( aWindow ) );

    return it != resolvers.end() && it->second( aId );
}


bool KOPENAPI_REMOTE::UiLocked()
{
    return state().lockedUi.load( std::memory_order_relaxed );
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
