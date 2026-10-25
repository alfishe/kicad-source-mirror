/// @file kicadopenapi_keepalive.cpp
/// @brief UI keep-alive during API calls (see kicadopenapi_keepalive.h).
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_journal.h>
#include <model_thread.h>
#include <platform.h>

#include <libraries/library_manager.h>

#include <wx/app.h>
#include <wx/evtloop.h>
#include <wx/log.h>
#include <wx/thread.h>
#include <wx/eventfilter.h>
#include <wx/toplevel.h>
#include <wx/weakref.h>

#include <map>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <exception>
#include <future>
#include <mutex>
#include <optional>
#include <thread>


namespace
{
std::chrono::steady_clock::time_point g_lastPump;
bool                                  g_pumping = false;
thread_local bool                     tlComputeWorker = false;

/// KiCad code is written for the main thread's stack (8 MB); the router recurses deeply
constexpr size_t COMPUTE_STACK_BYTES = 16 * 1024 * 1024;
} // namespace


void KopenapiKeepUiAlive()
{
    if( !wxTheApp || !wxThread::IsMain() || g_pumping )
        return;

    wxEventLoopBase* loop = wxEventLoopBase::GetActive();

    if( !loop || !loop->IsRunning() )
        return;   // headless servers run their own loop and have nothing to show

    const auto now = std::chrono::steady_clock::now();

    if( now - g_lastPump < std::chrono::milliseconds( 15 ) )
        return;

    g_pumping = true;

    // paint, size, timers: yes; the API's own calls (thread events): no; mouse / keyboard only
    // during a job (its input filter lets navigation through)
    loop->YieldFor( wxEVT_CATEGORY_UI | wxEVT_CATEGORY_TIMER
                    | ( KopenapiJobRunning() ? wxEVT_CATEGORY_USER_INPUT : 0 ) );

    g_pumping = false;
    g_lastPump = std::chrono::steady_clock::now();
}


void KopenapiYieldAll()
{
    if( !wxTheApp || !KopenapiOnDispatchThread() || g_pumping )
        return;

    wxEventLoopBase* loop = wxEventLoopBase::GetActive();

    if( !loop || !loop->IsRunning() )
    {
        KopenapiServeQueuedCalls();   // headless: no UI, the API calls only
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    if( now - g_lastPump < std::chrono::milliseconds( 30 ) )
        return;

    g_pumping = true;
    loop->YieldFor( wxEVT_CATEGORY_ALL );
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


void KopenapiReadPoint()
{
    // ending the write admits the readers waiting through it; taking it again waits for them
    MODEL_THREAD::READ_WINDOW window;
}


bool KopenapiInComputeWorker()
{
    return tlComputeWorker;
}


void KopenapiCompute( const std::function<void()>& aWork )
{
    MODEL_THREAD::READ_WINDOW window;

    wxEventLoopBase* loop = wxTheApp && wxThread::IsMain() ? wxEventLoopBase::GetActive() : nullptr;

    // KICAD_OPENAPI_MAIN_DISPATCH: the model before the rework, everything on the main thread (A/B)
    static const bool onMain = std::getenv( "KICAD_OPENAPI_MAIN_DISPATCH" ) != nullptr;

    if( !loop || !loop->IsRunning() || g_pumping || onMain )
    {
        aWork();
        return;
    }

    std::mutex              mutex;
    std::condition_variable finished;
    bool                    done = false;
    std::exception_ptr      error;
    const std::string       operation = KOPENAPI_JOURNAL::Operation();

    kopenapi::platform::THREAD worker( COMPUTE_STACK_BYTES,
                                       [&]()
                                       {
                                           wxLog::SetThreadActiveTarget( KOPENAPI_JOURNAL::ThreadTarget() );
                                           tlComputeWorker = true;

                                           {
                                               KOPENAPI_JOURNAL::SCOPED_OPERATION scope( operation );
                                               MODEL_THREAD::SCOPE reader( MODEL_THREAD::KIND::READER );

                                               try
                                               {
                                                   aWork();
                                               }
                                               catch( ... )
                                               {
                                                   error = std::current_exception();
                                               }
                                           }

                                           tlComputeWorker = false;
                                           wxLog::SetThreadActiveTarget( nullptr );

                                           {
                                               std::lock_guard<std::mutex> lock( mutex );
                                               done = true;
                                           }

                                           finished.notify_all();
                                       } );

    if( !worker.Started() )
    {
        aWork();
        return;
    }

    // paint, size, timers; during a job also the navigation its input filter lets through
    const long categories = wxEVT_CATEGORY_UI | wxEVT_CATEGORY_TIMER
                            | ( KopenapiJobRunning() ? wxEVT_CATEGORY_USER_INPUT : 0 );

    g_pumping = true;

    for( ;; )
    {
        loop->YieldFor( categories );

        std::unique_lock<std::mutex> lock( mutex );

        if( finished.wait_for( lock, std::chrono::milliseconds( 8 ), [&]() { return done; } ) )
            break;
    }

    g_pumping = false;
    g_lastPump = std::chrono::steady_clock::now();

    worker.Join();

    if( error )
        std::rethrow_exception( error );
}


void KopenapiWaitLibraries( LIBRARY_MANAGER_ADAPTER* aAdapter, int aTimeoutS )
{
    if( !aAdapter )
        return;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds( aTimeoutS );

    for( std::optional<float> p = aAdapter->AsyncLoadProgress(); p && *p < 1.0f; p = aAdapter->AsyncLoadProgress() )
    {
        if( std::chrono::steady_clock::now() > deadline )
        {
            wxLogMessage( "libraries: the background load is at %.0f %% after %d s; going on without the rest",
                          *p * 100.0, aTimeoutS );
            return;   // BlockUntilLoaded would wait for it on the main thread
        }

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


KOPENAPI_REMOTE_GUARD::KOPENAPI_REMOTE_GUARD( wxTopLevelWindow* aWindow ) :
        m_key( aWindow ),
        m_window( aWindow )
{
    if( !m_window || !wxThread::IsMain() )
        return;

    auto& entry = remoteFilter().m_windows[m_key];

    if( entry.first++ == 0 )
    {
        entry.second = m_window->GetTitle();
        m_window->SetTitle( entry.second + wxS( " \u2014 remote controlled (API)" ) );
    }
}


KOPENAPI_REMOTE_GUARD::~KOPENAPI_REMOTE_GUARD()
{
    if( !m_key || !wxThread::IsMain() )
        return;

    auto& windows = remoteFilter().m_windows;
    auto  it = windows.find( m_key );

    if( it == windows.end() )
        return;

    // the window closed while held: its entry goes (a new window at the same address must not
    // inherit it), there is no title to restore
    if( !m_window )
    {
        windows.erase( it );
        return;
    }

    if( --it->second.first == 0 )
    {
        m_window->SetTitle( it->second.second );
        windows.erase( it );
    }
}
