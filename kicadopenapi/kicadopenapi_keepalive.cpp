/// @file kicadopenapi_keepalive.cpp
/// @brief UI keep-alive during API calls (see kicadopenapi_keepalive.h).
#include <kicadopenapi_keepalive.h>

#include <libraries/library_manager.h>

#include <wx/app.h>
#include <wx/evtloop.h>
#include <wx/thread.h>
#include <wx/eventfilter.h>
#include <wx/toplevel.h>
#include <wx/weakref.h>

#include <map>

#include <atomic>
#include <chrono>
#include <future>
#include <optional>
#include <thread>


namespace
{
std::chrono::steady_clock::time_point g_lastPump;
bool                                  g_pumping = false;
} // namespace


void KopenapiKeepUiAlive()
{
    if( !wxTheApp || !wxThread::IsMain() || g_pumping )
        return;

    wxEventLoopBase* loop = wxEventLoopBase::GetActive();

    if( !loop || !loop->IsRunning() )
        return;   // headless servers run their own loop and have nothing to show

    const auto now = std::chrono::steady_clock::now();

    if( now - g_lastPump < std::chrono::milliseconds( 30 ) )
        return;

    g_pumping = true;

    // paint, size, timers: yes; mouse / keyboard and the API's own calls (thread events): no
    loop->YieldFor( wxEVT_CATEGORY_UI | wxEVT_CATEGORY_TIMER );

    g_pumping = false;
    g_lastPump = std::chrono::steady_clock::now();
}


void KopenapiRunOffMain( const std::function<void()>& aWork )
{
    std::future<void> done = std::async( std::launch::async, aWork );

    while( done.wait_for( std::chrono::milliseconds( 15 ) ) != std::future_status::ready )
        KopenapiKeepUiAlive();

    done.get();   // rethrows
}


void KopenapiWaitLibraries( LIBRARY_MANAGER_ADAPTER* aAdapter )
{
    if( !aAdapter )
        return;

    for( std::optional<float> p = aAdapter->AsyncLoadProgress(); p && *p < 1.0f; p = aAdapter->AsyncLoadProgress() )
    {
        KopenapiKeepUiAlive();
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    }

    aAdapter->BlockUntilLoaded();   // the workers retire right after the last library
}


namespace
{

/// @brief Drops user input aimed at windows under API control
class REMOTE_FILTER : public wxEventFilter
{
public:
    int FilterEvent( wxEvent& aEvent ) override
    {
        if( m_windows.empty() || aEvent.GetEventCategory() != wxEVT_CATEGORY_USER_INPUT )
            return Event_Skip;

        auto* window = dynamic_cast<wxWindow*>( aEvent.GetEventObject() );

        if( window && m_windows.count( wxGetTopLevelParent( window ) ) )
            return Event_Ignore;

        return Event_Skip;
    }

    /// window -> (guards, title before)
    std::map<wxWindow*, std::pair<int, wxString>> m_windows;
};


REMOTE_FILTER& remoteFilter()
{
    static REMOTE_FILTER* filter = []()
    {
        auto* f = new REMOTE_FILTER();
        wxEvtHandler::AddFilter( f );
        return f;
    }();

    return *filter;
}

} // namespace


KOPENAPI_REMOTE_GUARD::KOPENAPI_REMOTE_GUARD( wxTopLevelWindow* aWindow ) : m_window( aWindow )
{
    if( !m_window || !wxThread::IsMain() )
        return;

    auto& entry = remoteFilter().m_windows[m_window];

    if( entry.first++ == 0 )
    {
        entry.second = m_window->GetTitle();
        m_window->SetTitle( entry.second + wxS( " \u2014 remote controlled (API)" ) );
    }
}


KOPENAPI_REMOTE_GUARD::~KOPENAPI_REMOTE_GUARD()
{
    if( !m_window || !wxThread::IsMain() )
        return;

    auto& windows = remoteFilter().m_windows;
    auto  it = windows.find( m_window );

    if( it == windows.end() )
        return;

    if( --it->second.first == 0 )
    {
        m_window->SetTitle( it->second.second );
        windows.erase( it );
    }
}
