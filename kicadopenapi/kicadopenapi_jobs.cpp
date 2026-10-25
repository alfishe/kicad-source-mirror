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
#include <thread>


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
    bool              keep = false;   ///< job_cancel {keep}: stop, apply nothing, keep the state
    bool              paused = false;
    uint64_t          stepSerial = 0;      ///< job_step requests so far
    std::string       stepBy;
    int               stepN = 0;
    uint64_t          stepServed = 0;      ///< the last request the main thread went on for
    uint64_t          rev = 0;             ///< grows with every change of state or progress (job_get since)

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
    nlohmann::json          j = { { "job_id", aJob.Id() }, { "method", aJob.method }, { "state", aJob.state },
                                  { "rev", aJob.rev } };

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

    progress["elapsed_s"] = j["elapsed_s"];

    // the whole call's pace when it tells the share of its work done (counts.fraction), else the phase's
    if( aJob.state == "running" )
    {
        const nlohmann::json& f = aJob.counts.is_object() && aJob.counts.contains( "fraction" ) ? aJob.counts["fraction"]
                                                                                                 : nlohmann::json();
        const double fraction = f.is_number() ? f.get<double>() : -1;

        if( fraction > 0 && fraction < 1 )
        {
            const double spent = std::chrono::duration<double>( now - aJob.started ).count();
            progress["eta_s"] = std::round( spent / fraction * ( 1 - fraction ) * 10 ) / 10;
        }
        else if( progress.contains( "phase_eta_s" ) )
        {
            progress["eta_s"] = progress["phase_eta_s"];
        }
    }

    j["progress"] = progress;

    if( !aJob.stopReason.empty() )
        j["stop_reason"] = aJob.stopReason;

    if( !aJob.Finished() )
        j["paused"] = aJob.paused;

    if( aJob.stepSerial > 0 )
        j["step"] = { { "serial", aJob.stepSerial }, { "by", aJob.stepBy }, { "n", aJob.stepN } };

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
    aJob.rev++;

    if( !aCounts.is_null() )
        aJob.counts = aCounts;

    KopenapiEventNotify( "job" );
    KopenapiEventNotify( "progress" );
}


/// @brief Under g_mutex: the title of the windows at a job's progress, at most every 250 ms; else empty
std::string jobTitle( const JOB& aJob, const std::string& aPhase, int aDone, int aTotal )
{
    if( !g_titles || !wxThread::IsMain() || CLOCK::now() - g_titleShown < std::chrono::milliseconds( 250 ) )
        return std::string();

    g_titleShown = CLOCK::now();
    return "API job " + aJob.method + ": " + aPhase
           + ( aTotal > 0 ? " " + std::to_string( aDone ) + "/" + std::to_string( aTotal ) : std::string() )
           + " (Esc stops)";
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

        title = jobTitle( job, aPhase, aDone, aTotal );
        g_yielding = true;
    }

    g_changed.notify_all();
    KopenapiEventNotify( "job" );

    if( !title.empty() )
        g_titles->Show( title );

    KopenapiYieldAll();

    // paused: the UI and reading calls run until job_resume, a job_step or a stop
    for( bool shown = false;; )
    {
        {
            std::lock_guard<std::mutex> lock( g_mutex );
            JOB&                        job = *g_current;

            if( !job.paused || job.cancel )
                break;

            if( job.stepSerial != job.stepServed )
            {
                job.stepServed = job.stepSerial;
                break;
            }
        }

        if( !shown && g_titles )
        {
            g_titles->Show( "API job " + g_current->method + " paused (job_resume, job_step; Esc stops)" );
            shown = true;
        }

        KopenapiYieldAll();
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    }

    std::lock_guard<std::mutex> lock( g_mutex );
    g_yielding = false;
    return !g_current->cancel;
}


bool KopenapiJobProgress( const std::string& aPhase, int aDone, int aTotal, const std::string& aItem,
                          const nlohmann::json& aCounts )
{
    std::string title;
    bool        cancelled = false;

    {
        std::lock_guard<std::mutex> lock( g_mutex );
        JOB*                        job = g_current ? g_current.get() : g_call.get();

        if( g_yielding || !job )
            return true;

        record( *job, aPhase, aDone, aTotal, aItem, aCounts );

        if( g_current )
        {
            cancelled = g_current->cancel;
            title = jobTitle( *g_current, aPhase, aDone, aTotal );
        }
    }

    g_changed.notify_all();

    if( !title.empty() )
        g_titles->Show( title );

    KopenapiKeepUiAlive();
    return !cancelled;
}


bool KopenapiJobHeld( uint64_t* aSerial, std::string* aBy, int* aN )
{
    std::lock_guard<std::mutex> lock( g_mutex );

    if( !g_current || !g_current->paused || g_current->cancel )
        return false;

    if( aSerial )
        *aSerial = g_current->stepSerial;

    if( aBy )
        *aBy = g_current->stepBy;

    if( aN )
        *aN = g_current->stepN;

    return true;
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
    {
        std::lock_guard<std::mutex> lock( g_mutex );
        g_call.reset();
    }

    KopenapiEventNotify( "progress" );
}


nlohmann::json KopenapiCallProgress()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    const JOB*                  job = g_current ? g_current.get() : g_call.get();

    if( !job || job->phase.empty() )
        return nullptr;

    return describe( *job, false )["progress"];
}


nlohmann::json KopenapiRunningProgress()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    const JOB*                  job = g_current ? g_current.get() : g_call.get();

    if( !job || job->phase.empty() )
        return nullptr;

    const nlohmann::json j = describe( *job, false );
    nlohmann::json       out = { { "method", job->method }, { "rev", job->rev }, { "progress", j["progress"] } };

    if( g_current )
    {
        out["job_id"] = job->Id();
        out["paused"] = job->paused;
    }

    return out;
}


bool KopenapiJobCancelled()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    return g_current && g_current->cancel;
}


bool KopenapiJobKeep()
{
    std::lock_guard<std::mutex> lock( g_mutex );
    return g_current && g_current->cancel && g_current->keep;
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
            job->rev++;
            g_changed.notify_all();
            KopenapiEventNotify( "job" );
            return;
        }

        job->state = "running";
        job->started = job->phaseStarted = CLOCK::now();
        job->rev++;
        g_current = job;
    }

    g_changed.notify_all();
    KopenapiEventNotify( "job" );
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
    catch( const KOPENAPI_RETRY_LATER& e )
    {
        result = e.Result();
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
        job->rev++;
        g_current.reset();
    }

    g_changed.notify_all();
    KopenapiEventNotify( "job" );
    KopenapiEventNotify( "progress" );
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
            it->second->rev++;
        }
    }

    g_changed.notify_all();
    KopenapiEventNotify( "job" );
}


KOPENAPI_RESULT KOPENAPI_JOBS::Status( const nlohmann::json& aArgs )
{
    const std::string id = aArgs.value( "job_id", std::string() );
    const int         waitS = std::clamp( aArgs.value( "wait_s", 0 ), 0, 600 );
    const bool        bySince = aArgs.contains( "since" ) && aArgs["since"].is_number_integer();
    const int64_t     since = bySince ? aArgs["since"].get<int64_t>() : 0;

    std::unique_lock<std::mutex> lock( g_mutex );
    auto                         it = g_jobs.find( idOf( id ) );

    if( it == g_jobs.end() )
        return KOPENAPI_RESULT::Error( 404, "no job '" + id + "' (the last " + std::to_string( KEEP ) + " are kept)" );

    std::shared_ptr<JOB> job = it->second;
    // since: returns as soon as anything changed after that rev (long poll), else at the end
    g_changed.wait_for( lock, std::chrono::seconds( waitS ),
                        [&] { return job->Finished() || ( bySince && (int64_t) job->rev > since ); } );

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
        {
            job.keep = aArgs.value( "keep", false );
            job.stopReason = job.keep ? "job_cancel {keep}" : "job_cancel";
            job.rev++;
        }
    }

    g_changed.notify_all();

    // the job stops at its next safe point and still gives its result (what it did so far)
    return KOPENAPI_RESULT::Ok( { { "job_id", id }, { "stopping", true } } );
}


KOPENAPI_RESULT KOPENAPI_JOBS::Pause( const nlohmann::json& aArgs, bool aPause )
{
    const std::string           id = aArgs.value( "job_id", std::string() );
    std::lock_guard<std::mutex> lock( g_mutex );
    auto                        it = g_jobs.find( idOf( id ) );

    if( it == g_jobs.end() )
        return KOPENAPI_RESULT::Error( 404, "no job '" + id + "'" );

    JOB& job = *it->second;

    if( !job.Finished() && job.paused != aPause )
    {
        job.paused = aPause;
        job.rev++;
        g_changed.notify_all();
        KopenapiEventNotify( "job" );
        wxLogMessage( "api job %s (%s): %s", job.Id(), job.method, aPause ? "paused" : "resumed" );
    }

    return KOPENAPI_RESULT::Ok( describe( job, false ) );
}


KOPENAPI_RESULT KOPENAPI_JOBS::Step( const nlohmann::json& aArgs )
{
    const std::string id = aArgs.value( "job_id", std::string() );
    const std::string by = aArgs.value( "by", std::string( "net" ) );

    if( by != "net" && by != "round" )
        return KOPENAPI_RESULT::Error( 400, "by: net or round" );

    std::lock_guard<std::mutex> lock( g_mutex );
    auto                        it = g_jobs.find( idOf( id ) );

    if( it == g_jobs.end() )
        return KOPENAPI_RESULT::Error( 404, "no job '" + id + "'" );

    JOB& job = *it->second;

    if( job.Finished() || !job.paused )
        return KOPENAPI_RESULT::Error( 409, "job '" + id + "' is not paused (job_pause first)" );

    job.stepBy = by;
    job.stepN = std::clamp( aArgs.value( "n", 1 ), 1, 100000 );
    ++job.stepSerial;
    job.rev++;
    g_changed.notify_all();
    KopenapiEventNotify( "job" );
    return KOPENAPI_RESULT::Ok( describe( job, false ) );
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


namespace
{

KOPENAPI_RESULT openJobEvents( const nlohmann::json& aArgs, KOPENAPI_EVENT_POLL& aPoll )
{
    const std::string id = aArgs.value( "job_id", nlohmann::json() ).is_string() ? aArgs["job_id"].get<std::string>() : std::string();
    std::shared_ptr<JOB> job;

    {
        std::lock_guard<std::mutex> lock( g_mutex );
        auto                        it = g_jobs.find( idOf( id ) );

        if( it == g_jobs.end() )
            return KOPENAPI_RESULT::Error( 404, "no job '" + id + "' (the last " + std::to_string( KEEP ) + " are kept)" );

        job = it->second;
    }

    auto last = std::make_shared<nlohmann::json>();

    aPoll = [job, last]( std::vector<KOPENAPI_EVENT>& aOut )
    {
        std::lock_guard<std::mutex> lock( g_mutex );

        if( job->Finished() )
        {
            aOut.push_back( { "end", describe( *job, true ) } );
            return false;
        }

        nlohmann::json j = describe( *job, false );
        nlohmann::json key = j;
        key.erase( "elapsed_s" );
        key.erase( "queued_s" );

        key.erase( "rev" );

        if( key.contains( "progress" ) )
        {
            for( const char* volatileKey : { "phase_eta_s", "eta_s", "elapsed_s" } )
                key["progress"].erase( volatileKey );
        }

        if( key != *last )
        {
            *last = std::move( key );
            aOut.push_back( { "progress", std::move( j ) } );
        }

        return true;
    };

    return KOPENAPI_RESULT::Ok( nlohmann::json::object() );
}

} // namespace


KOPENAPI_REGISTER_EVENT_TOPIC( "job",
                               "a job's progress (job_id): 'progress' events as job_get without the result when "
                               "the phase, counts, pause or step change, then 'end' with the result",
                               openJobEvents );

