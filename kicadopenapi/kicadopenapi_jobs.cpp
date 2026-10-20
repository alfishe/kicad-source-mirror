/// @file kicadopenapi_jobs.cpp
/// @brief Long API calls as jobs (see kicadopenapi_jobs.h).
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_keepalive.h>

#include <wx/app.h>
#include <wx/event.h>
#include <wx/eventfilter.h>
#include <wx/log.h>
#include <wx/thread.h>
#include <wx/toplevel.h>
#include <wx/weakref.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>


namespace
{

using CLOCK = std::chrono::steady_clock;

struct JOB
{
    uint64_t          seq = 0;
    std::string       method;
    std::string       state = "queued";   ///< queued, running, done, failed, cancelled
    std::string       phase;
    std::string       item;
    int               done = 0;
    int               total = 0;
    nlohmann::json    counts;
    std::string       stopReason;
    CLOCK::time_point created = CLOCK::now();
    CLOCK::time_point started;
    CLOCK::time_point phaseStarted;
    CLOCK::time_point finished;
    std::optional<KOPENAPI_RESULT> result;
    std::atomic<bool> cancel{ false };

    std::string Id() const { return "job-" + std::to_string( seq ); }
    bool        Finished() const { return state == "done" || state == "failed" || state == "cancelled"; }
};

std::mutex                                g_mutex;
std::condition_variable                   g_changed;
std::map<uint64_t, std::shared_ptr<JOB>>  g_jobs;
uint64_t                                  g_seq = 0;
std::shared_ptr<JOB>                      g_current;    ///< the running job (main thread)
std::shared_ptr<JOB>                      g_call;       ///< the running outermost call, outside jobs
CLOCK::time_point                         g_callServed;
bool                                      g_yielding = false;
constexpr size_t                          KEEP = 32;


double seconds( CLOCK::duration aD )
{
    return std::round( std::chrono::duration<double>( aD ).count() * 10 ) / 10;
}


uint64_t idOf( const std::string& aId )
{
    return aId.rfind( "job-", 0 ) == 0 ? std::strtoull( aId.c_str() + 4, nullptr, 10 ) : 0;
}


/// @brief Under g_mutex
nlohmann::json describe( const JOB& aJob, bool aWithResult )
{
    const CLOCK::time_point now = CLOCK::now();
    nlohmann::json          j = { { "job_id", aJob.Id() }, { "method", aJob.method }, { "state", aJob.state } };

    if( aJob.state == "queued" )
    {
        j["queued_s"] = seconds( now - aJob.created );
        return j;
    }

    const CLOCK::time_point end = aJob.Finished() ? aJob.finished : now;
    j["elapsed_s"] = seconds( end - aJob.started );

    nlohmann::json progress = { { "phase", aJob.phase }, { "done", aJob.done }, { "total", aJob.total } };

    if( !aJob.item.empty() )
        progress["item"] = aJob.item;

    if( !aJob.counts.is_null() )
        progress["counts"] = aJob.counts;

    // the phase's own pace; phases after it are not known
    if( aJob.state == "running" && aJob.done > 0 && aJob.done < aJob.total )
    {
        const double spent = std::chrono::duration<double>( now - aJob.phaseStarted ).count();
        progress["phase_eta_s"] = std::round( spent / aJob.done * ( aJob.total - aJob.done ) * 10 ) / 10;
    }

    j["progress"] = progress;

    if( !aJob.stopReason.empty() )
        j["stop_reason"] = aJob.stopReason;

    if( aWithResult && aJob.result )
    {
        j["status"] = aJob.result->status;
        j["result"] = aJob.result->body;
    }

    return j;
}


void requestStop( const std::string& aWhy )
{
    std::lock_guard<std::mutex> lock( g_mutex );

    if( g_current && !g_current->cancel.exchange( true ) )
    {
        g_current->stopReason = aWhy;
        wxLogMessage( "api job %s (%s): stopping: %s", g_current->Id(), g_current->method, aWhy );
    }
}


/// @brief While a job runs: view navigation passes, other input is dropped, Esc stops the job, a
/// window close is refused and stops it (the window closes when asked again afterwards)
class JOB_FILTER : public wxEventFilter
{
public:
    int FilterEvent( wxEvent& aEvent ) override
    {
        const wxEventType type = aEvent.GetEventType();

        if( type == wxEVT_CLOSE_WINDOW || type == wxEVT_QUERY_END_SESSION )
        {
            auto& close = static_cast<wxCloseEvent&>( aEvent );

            if( close.CanVeto() )
            {
                requestStop( "a window close was asked (close again once the job has stopped)" );
                close.Veto();
                return Event_Processed;
            }

            requestStop( "the session ends" );
            return Event_Skip;
        }

        if( aEvent.GetEventCategory() != wxEVT_CATEGORY_USER_INPUT )
            return Event_Skip;

        if( type == wxEVT_CHAR_HOOK || type == wxEVT_KEY_DOWN )
        {
            if( static_cast<wxKeyEvent&>( aEvent ).GetKeyCode() == WXK_ESCAPE )
            {
                requestStop( "Esc pressed" );
                return Event_Processed;
            }

            return Event_Ignore;
        }

        if( type == wxEVT_MOUSEWHEEL || type == wxEVT_MAGNIFY || type == wxEVT_MOTION || type == wxEVT_MIDDLE_DOWN
            || type == wxEVT_MIDDLE_UP || type == wxEVT_MIDDLE_DCLICK || type == wxEVT_ENTER_WINDOW
            || type == wxEVT_LEAVE_WINDOW )
        {
            return Event_Skip;
        }

        return Event_Ignore;
    }
};


/// @brief The titles of the shown top-level windows say a job runs and how far it is
class JOB_TITLES
{
public:
    void Show( const std::string& aText )
    {
        if( !wxThread::IsMain() )
            return;

        for( wxWindow* w : wxTopLevelWindows )
        {
            auto* tlw = dynamic_cast<wxTopLevelWindow*>( w );

            if( !tlw || !tlw->IsShown() )
                continue;

            auto it = std::find_if( m_saved.begin(), m_saved.end(), [&]( const auto& s ) { return s.first.get() == tlw; } );

            if( it == m_saved.end() )
            {
                m_saved.emplace_back( wxWeakRef<wxTopLevelWindow>( tlw ), tlw->GetTitle() );
                it = std::prev( m_saved.end() );
            }

            tlw->SetTitle( it->second + wxS( " — " ) + wxString::FromUTF8( aText ) );
        }
    }

    ~JOB_TITLES()
    {
        for( auto& [window, title] : m_saved )
        {
            if( window )
                window->SetTitle( title );
        }
    }

private:
    std::deque<std::pair<wxWeakRef<wxTopLevelWindow>, wxString>> m_saved;
};

JOB_TITLES* g_titles = nullptr;
CLOCK::time_point g_titleShown;

} // namespace


namespace
{

/// @brief Under g_mutex
void record( JOB& aJob, const std::string& aPhase, int aDone, int aTotal, const std::string& aItem,
             const nlohmann::json& aCounts )
{
    if( aJob.phase != aPhase )
        aJob.phaseStarted = CLOCK::now();

    aJob.phase = aPhase;
    aJob.done = aDone;
    aJob.total = aTotal;
    aJob.item = aItem;

    if( !aCounts.is_null() )
        aJob.counts = aCounts;
}

} // namespace


bool KopenapiJobStep( const std::string& aPhase, int aDone, int aTotal, const std::string& aItem,
                      const nlohmann::json& aCounts )
{
    std::string title;

    {
        std::lock_guard<std::mutex> lock( g_mutex );

        if( g_yielding || !wxThread::IsMain() )
            return true;

        if( !g_current && !g_call )
            return true;

        if( !g_current )
        {
            record( *g_call, aPhase, aDone, aTotal, aItem, aCounts );

            if( CLOCK::now() - g_callServed < std::chrono::milliseconds( 30 ) )
                return true;

            g_yielding = true;
        }
    }

    // a plain call: reading API calls queued meanwhile run here (the documents are consistent),
    // edits get 409; no user input, no stop
    if( g_yielding )
    {
        KopenapiServeQueuedCalls();

        std::lock_guard<std::mutex> lock( g_mutex );
        g_yielding = false;
        g_callServed = CLOCK::now();
        return true;
    }

    {
        std::lock_guard<std::mutex> lock( g_mutex );

        JOB& job = *g_current;
        record( job, aPhase, aDone, aTotal, aItem, aCounts );

        if( job.cancel )
            return false;

        if( g_titles && CLOCK::now() - g_titleShown > std::chrono::milliseconds( 250 ) )
        {
            title = "API job " + job.method + ": " + aPhase
                    + ( aTotal > 0 ? " " + std::to_string( aDone ) + "/" + std::to_string( aTotal ) : std::string() )
                    + " (Esc stops)";
            g_titleShown = CLOCK::now();
        }

        g_yielding = true;
    }

    g_changed.notify_all();

    if( !title.empty() )
        g_titles->Show( title );

    KopenapiYieldAll();

    std::lock_guard<std::mutex> lock( g_mutex );
    g_yielding = false;
    return !g_current->cancel;
}


void KopenapiCallBegin( const std::string& aMethod )
{
    auto call = std::make_shared<JOB>();
    call->method = aMethod;
    call->state = "running";
    call->started = call->phaseStarted = CLOCK::now();

    std::lock_guard<std::mutex> lock( g_mutex );
    g_call = std::move( call );
}


void KopenapiCallEnd()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    g_call.reset();
}


nlohmann::json KopenapiCallProgress()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    const JOB*                  job = g_current ? g_current.get() : g_call.get();

    if( !job || job->phase.empty() )
        return nullptr;

    const nlohmann::json j = describe( *job, false );
    nlohmann::json       progress = j["progress"];
    progress["elapsed_s"] = j["elapsed_s"];
    return progress;
}


bool KopenapiJobCancelled()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    return g_current && g_current->cancel;
}


bool KopenapiJobYielding()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    return g_yielding;
}


KOPENAPI_JOBS& KOPENAPI_JOBS::Get()
{
    static KOPENAPI_JOBS jobs;
    return jobs;
}


std::string KOPENAPI_JOBS::Add( const std::string& aMethod, std::string& aWhy )
{
    std::lock_guard<std::mutex> lock( g_mutex );

    for( const auto& [seq, job] : g_jobs )
    {
        if( !job->Finished() )
        {
            aWhy = "job " + job->Id() + " (" + job->method + ") is " + job->state
                   + ": one job at a time (job_get, job_cancel)";
            return std::string();
        }
    }

    auto job = std::make_shared<JOB>();
    job->seq = ++g_seq;
    job->method = aMethod;
    g_jobs[job->seq] = job;

    while( g_jobs.size() > KEEP )
        g_jobs.erase( g_jobs.begin() );   // finished: only one is ever unfinished, the newest

    return job->Id();
}


void KOPENAPI_JOBS::Run( const std::string& aId, const std::function<KOPENAPI_RESULT()>& aCall )
{
    std::shared_ptr<JOB> job;

    {
        std::lock_guard<std::mutex> lock( g_mutex );
        auto                        it = g_jobs.find( idOf( aId ) );

        if( it == g_jobs.end() )
            return;

        job = it->second;

        if( job->cancel )
        {
            job->state = "cancelled";
            job->started = job->finished = CLOCK::now();
            g_changed.notify_all();
            return;
        }

        job->state = "running";
        job->started = job->phaseStarted = CLOCK::now();
        g_current = job;
    }

    g_changed.notify_all();
    wxLogMessage( "api job %s (%s): started", aId, job->method );

    JOB_FILTER filter;
    JOB_TITLES titles;
    wxEvtHandler::AddFilter( &filter );
    g_titles = &titles;

    KOPENAPI_RESULT result;

    try
    {
        result = aCall();
    }
    catch( const std::exception& e )
    {
        result = KOPENAPI_RESULT::Error( 500, e.what() );
    }
    catch( ... )
    {
        result = KOPENAPI_RESULT::Error( 500, "unknown exception" );
    }

    g_titles = nullptr;
    wxEvtHandler::RemoveFilter( &filter );

    {
        std::lock_guard<std::mutex> lock( g_mutex );
        job->finished = CLOCK::now();
        job->state = result.status >= 400 ? "failed" : job->cancel ? "cancelled" : "done";
        job->result = std::move( result );
        g_current.reset();
    }

    g_changed.notify_all();
    wxLogMessage( "api job %s (%s): %s", aId, job->method, job->state );
}


void KOPENAPI_JOBS::Fail( const std::string& aId, const KOPENAPI_RESULT& aResult )
{
    {
        std::lock_guard<std::mutex> lock( g_mutex );

        if( auto it = g_jobs.find( idOf( aId ) ); it != g_jobs.end() && !it->second->Finished() )
        {
            it->second->state = "failed";
            it->second->started = it->second->finished = CLOCK::now();
            it->second->result = aResult;
        }
    }

    g_changed.notify_all();
}


KOPENAPI_RESULT KOPENAPI_JOBS::Status( const nlohmann::json& aArgs )
{
    const std::string id = aArgs.value( "job_id", std::string() );
    const int         waitS = std::clamp( aArgs.value( "wait_s", 0 ), 0, 600 );

    std::unique_lock<std::mutex> lock( g_mutex );
    auto                         it = g_jobs.find( idOf( id ) );

    if( it == g_jobs.end() )
        return KOPENAPI_RESULT::Error( 404, "no job '" + id + "' (the last " + std::to_string( KEEP ) + " are kept)" );

    std::shared_ptr<JOB> job = it->second;
    g_changed.wait_for( lock, std::chrono::seconds( waitS ), [&] { return job->Finished(); } );

    return KOPENAPI_RESULT::Ok( describe( *job, true ) );
}


KOPENAPI_RESULT KOPENAPI_JOBS::Cancel( const nlohmann::json& aArgs )
{
    const std::string id = aArgs.value( "job_id", std::string() );

    {
        std::lock_guard<std::mutex> lock( g_mutex );
        auto                        it = g_jobs.find( idOf( id ) );

        if( it == g_jobs.end() )
            return KOPENAPI_RESULT::Error( 404, "no job '" + id + "'" );

        JOB& job = *it->second;

        if( job.Finished() )
            return KOPENAPI_RESULT::Ok( describe( job, false ) );

        if( !job.cancel.exchange( true ) )
            job.stopReason = "job_cancel";
    }

    // the job stops at its next safe point and still gives its result (what it did so far)
    return KOPENAPI_RESULT::Ok( { { "job_id", id }, { "stopping", true } } );
}


KOPENAPI_RESULT KOPENAPI_JOBS::List() const
{
    std::lock_guard<std::mutex> lock( g_mutex );
    nlohmann::json              list = nlohmann::json::array();

    for( auto it = g_jobs.rbegin(); it != g_jobs.rend(); ++it )
        list.push_back( describe( *it->second, false ) );

    return KOPENAPI_RESULT::Ok( { { "jobs", list } } );
}


std::string KOPENAPI_JOBS::Busy() const
{
    std::lock_guard<std::mutex> lock( g_mutex );

    for( const auto& [seq, job] : g_jobs )
    {
        if( !job->Finished() )
            return job->Id() + " (" + job->method + ")";
    }

    return std::string();
}
