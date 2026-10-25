#include "kicadopenapi_service.h"
#include "kicadopenapi_history.h"
#include "kicadopenapi_registry.h"
#include "recording/kicadopenapi_recorder.h"
#include "kicadopenapi_mcp.h"
#include "kicadopenapi_journal.h"
#include "kicadopenapi_memory.h"
#include "kicadopenapi_remote.h"
#include "kicadopenapi_libraries.h"
#include "kicadopenapi_jobs.h"
#include "threading/kicadopenapi_doc_gate.h"
#include "kicadopenapi_change_feed.h"

#include <algorithm>
#include <atomic>
#include <functional>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <wx/utils.h>
#include <fstream>
#include <future>

#include "kicadopenapi_args.h"
#include "kicadopenapi_call_ticket.h"
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <thread>

// A body sent without "Content-Type: application/json" (curl -d, Python urllib) is taken as a form,
// whose limit would be 8 KB: a netlist or a long query would get 413.  The same limit as JSON.
#define CPPHTTPLIB_FORM_URL_ENCODED_PAYLOAD_MAX_LENGTH ( 100 * 1024 * 1024 )
#include <httplib.h>

#include <kiway.h>
#include <wx/app.h>
#include <wx/log.h>
#include <wx/timer.h>
#include <kicadopenapi_util.h>
#include <wx/datetime.h>

#include <platform.h>
#include <model_thread.h>

static const char* const KOPENAPI_VERSION = "0.2.0";
static const char* const KOPENAPI_HOST_ADDR = "127.0.0.1";

/// @brief Default upper bound for one main-thread call (methods may declare more); long operations
/// will move to async jobs.
static constexpr std::chrono::seconds CALL_TIMEOUT( 15 );
static constexpr std::chrono::milliseconds WAIT_SLICE( 20 );


/// @brief A method's timeout; env KICAD_OPENAPI_CALL_TIMEOUT_S lowers every one of them (tests of
/// the late-call path with ordinary methods)
static std::chrono::seconds callTimeout( int aMethodSec )
{
    static const int cap = []
    {
        const char* env = std::getenv( "KICAD_OPENAPI_CALL_TIMEOUT_S" );
        return env ? std::atoi( env ) : 0;
    }();

    const int sec = std::max( aMethodSec, 1 );
    return std::chrono::seconds( cap > 0 ? std::min( sec, cap ) : sec );
}


/// @brief Calls that outlived their timeout and run on: the 504 names them (call_id), their result
/// waits here for api_call_result.  The newest LATE_KEEP are kept, finished ones dropped first.
struct LATE_CALL
{
    std::string                           method;
    std::chrono::steady_clock::time_point since;
    std::shared_future<KOPENAPI_RESULT>   result;
};

static std::mutex                         s_lateMutex;
static std::map<uint64_t, LATE_CALL>      s_lateCalls;
static uint64_t                           s_lateSeq = 0;
static constexpr size_t                   LATE_KEEP = 64;


static std::string keepLateCall( const std::string& aMethod, std::shared_future<KOPENAPI_RESULT> aResult )
{
    std::lock_guard<std::mutex> lock( s_lateMutex );
    const uint64_t              id = ++s_lateSeq;

    s_lateCalls[id] = { aMethod, std::chrono::steady_clock::now(), std::move( aResult ) };

    while( s_lateCalls.size() > LATE_KEEP )
    {
        auto drop = std::find_if( s_lateCalls.begin(), s_lateCalls.end(),
                                  []( const auto& aEntry )
                                  {
                                      return aEntry.second.result.wait_for( std::chrono::seconds( 0 ) )
                                             == std::future_status::ready;
                                  } );
        s_lateCalls.erase( drop != s_lateCalls.end() ? drop : s_lateCalls.begin() );
    }

    return "call-" + std::to_string( id );
}


/// @brief api_call_result: the result of a call that outlived its timeout (once; then forgotten),
/// or that it still runs.  wait_s waits for it on this HTTP thread.
static KOPENAPI_RESULT lateCallResult( const nlohmann::json& aArgs )
{
    const std::string callId = aArgs.value( "call_id", std::string() );
    uint64_t          id = 0;

    if( callId.rfind( "call-", 0 ) == 0 )
        id = std::strtoull( callId.c_str() + 5, nullptr, 10 );

    std::shared_future<KOPENAPI_RESULT> result;
    std::string                         method;
    std::chrono::steady_clock::time_point since;

    {
        std::lock_guard<std::mutex> lock( s_lateMutex );
        auto                        it = s_lateCalls.find( id );

        if( it == s_lateCalls.end() )
            return KOPENAPI_RESULT::Error( 404, "no call '" + callId + "': unknown, or its result was already taken" );

        result = it->second.result;
        method = it->second.method;
        since = it->second.since;
    }

    const int waitS = std::clamp( aArgs.value( "wait_s", 0 ), 0, 600 );

    if( result.wait_for( std::chrono::seconds( waitS ) ) != std::future_status::ready )
    {
        const double running = std::chrono::duration<double>( std::chrono::steady_clock::now() - since ).count();
        return KOPENAPI_RESULT::Ok( { { "call_id", callId }, { "method", method }, { "running", true },
                                      { "running_s", std::round( running * 10 ) / 10 } } );
    }

    {
        std::lock_guard<std::mutex> lock( s_lateMutex );
        s_lateCalls.erase( id );
    }

    KOPENAPI_RESULT done = result.get();

    if( done.body.is_object() )
    {
        done.body["call_id"] = callId;
        done.body["method"] = method;
        done.body["running"] = false;
    }

    return done;
}

namespace fs = std::filesystem;


/// @brief Drops discovery files left behind by crashed processes
static std::atomic<int>  s_apiCalls{ 0 };       ///< API calls running on the main thread now


/// @brief Arguments a handler can take as they are: top-level types as the method's schema says,
/// every number finite and in range (millimetre values — keys ending in _mm, "points" — within
/// +-2000 mm, which still fits nanometres in 32 bits; other numbers within 32-bit integers), and
/// no nesting deeper than 64. Anything else is answered 400 before the main thread sees it.
static std::string checkArgs( const nlohmann::json& aSchema, const nlohmann::json& aArgs )
{
    if( aSchema.contains( "properties" ) && aSchema["properties"].is_object() )
    {
        for( const auto& [key, prop] : aSchema["properties"].items() )
        {
            if( !aArgs.contains( key ) || !prop.is_object() || !prop.contains( "type" ) || !prop["type"].is_string() )
                continue;

            const std::string     type = prop["type"].get<std::string>();
            const nlohmann::json& v = aArgs[key];
            const bool            ok = type == "string"    ? v.is_string()
                                       : type == "number"  ? v.is_number()
                                       : type == "integer" ? v.is_number()
                                       : type == "boolean" ? v.is_boolean()
                                       : type == "array"   ? v.is_array()
                                       : type == "object"  ? v.is_object()
                                                           : true;

            if( !ok )
                return "'" + key + "' must be " + ( type == "integer" ? "an integer" : "a " + type );
        }
    }

    std::string error;

    std::function<void( const nlohmann::json&, const std::string&, bool, int )> walk =
            [&]( const nlohmann::json& aValue, const std::string& aKey, bool aMm, int aDepth )
    {
        if( !error.empty() )
            return;

        if( aDepth > 64 )
        {
            error = "arguments nested too deep";
            return;
        }

        if( aValue.is_number_float() || aValue.is_number_integer() || aValue.is_number_unsigned() )
        {
            const double v = aValue.get<double>();

            if( !std::isfinite( v ) )
                error = "'" + aKey + "' is not a finite number";
            else if( aMm && std::abs( v ) > 2000.0 )
                error = "'" + aKey + "' is out of range (millimetres within +-2000)";
            else if( !aMm && std::abs( v ) > 2147483647.0 )
                error = "'" + aKey + "' is out of range";

            return;
        }

        if( aValue.is_object() )
        {
            for( const auto& [k, v] : aValue.items() )
            {
                const bool mm = k.size() > 3 && k.compare( k.size() - 3, 3, "_mm" ) == 0;
                walk( v, k, mm || k == "points", aDepth + 1 );
            }
        }
        else if( aValue.is_array() )
        {
            for( const nlohmann::json& v : aValue )
                walk( v, aKey, aMm, aDepth + 1 );
        }
    };

    walk( aArgs, "arguments", false, 0 );
    return error;
}
static std::atomic<bool> s_overrideLock{ false };


static void removeStaleDiscoveryFiles( const fs::path& aDir )
{
    std::error_code ec;

    for( const fs::directory_entry& entry : fs::directory_iterator( aDir, ec ) )
    {
        if( entry.path().extension() != ".json" )
            continue;

        long pid = std::atol( entry.path().stem().string().c_str() );

        if( pid > 0 && !kopenapi::platform::ProcessAlive( pid ) )
            fs::remove( entry.path(), ec );
    }
}


static std::string toJson( const nlohmann::json& aJson )
{
    // Replace invalid UTF-8 rather than throwing from inside an HTTP handler
    return aJson.dump( -1, ' ', false, nlohmann::json::error_handler_t::replace );
}


static void reply( httplib::Response& aRes, const KOPENAPI_RESULT& aResult )
{
    aRes.status = aResult.status;

    if( aResult.status == 503 && aResult.body.contains( "retry_after_s" ) && aResult.body["retry_after_s"].is_number() )
        aRes.set_header( "Retry-After", std::to_string( aResult.body["retry_after_s"].get<int>() ) );

    aRes.set_content( toJson( aResult.body ), "application/json" );
}


/// @brief The main-thread end of the gate: API work arrives as thread events (category THREAD),
/// so a call that keeps the UI alive (KopenapiKeepUiAlive: paint and timers only) never starts
/// another call inside itself.
class MAIN_GATE_HANDLER : public wxEvtHandler
{
public:
    MAIN_GATE_HANDLER()
    {
        Bind( wxEVT_THREAD,
              [this]( wxThreadEvent& aEvent )
              {
                  m_queued--;

                  if( auto job = aEvent.GetPayload<std::shared_ptr<std::function<void()>>>() )
                      ( *job )();
              } );
    }

    /// @brief Any thread; counted once queued, so the count never exceeds what is in the queue
    void Queue( wxThreadEvent* aEvent )
    {
        QueueEvent( aEvent );
        m_queued++;
    }

    /// @brief Calls queued and not started (ProcessPendingEvents asserts on an empty queue)
    int Queued() const { return m_queued; }

    /// @brief API calls: also run at the safe points of a long call
    static MAIN_GATE_HANDLER& Get()
    {
        static MAIN_GATE_HANDLER handler;
        return handler;
    }

    /// @brief Jobs: never started inside another call
    static MAIN_GATE_HANDLER& Jobs()
    {
        static MAIN_GATE_HANDLER handler;
        return handler;
    }

private:
    std::atomic<int> m_queued{ 0 };
};


using DISPATCH_TASK = std::shared_ptr<std::function<void()>>;


/// @brief Where API calls and jobs run.  Calls go before jobs; a job's safe point serves the
/// calls queued so far (ServeQueuedCalls).
class CALL_DISPATCH
{
public:
    virtual ~CALL_DISPATCH() = default;

    /// @brief Any thread
    virtual void QueueCall( DISPATCH_TASK aTask ) = 0;
    virtual void QueueJob( DISPATCH_TASK aTask ) = 0;

    /// @brief On the dispatch thread: runs the calls queued now, none of what arrives meanwhile
    virtual void ServeQueuedCalls() = 0;

    virtual bool OnDispatchThread() const = 0;
};


/// @brief GUI: the wx main loop, by thread events
class MAIN_LOOP_DISPATCH : public CALL_DISPATCH
{
public:
    void QueueCall( DISPATCH_TASK aTask ) override { queue( MAIN_GATE_HANDLER::Get(), std::move( aTask ) ); }

    void QueueJob( DISPATCH_TASK aTask ) override { queue( MAIN_GATE_HANDLER::Jobs(), std::move( aTask ) ); }

    void ServeQueuedCalls() override
    {
        MAIN_GATE_HANDLER& gate = MAIN_GATE_HANDLER::Get();

        // one event per ProcessPendingEvents
        for( int left = gate.Queued(); left > 0 && gate.Queued() > 0; --left )
            gate.ProcessPendingEvents();
    }

    bool OnDispatchThread() const override { return wxThread::IsMain(); }

private:
    static void queue( MAIN_GATE_HANDLER& aGate, DISPATCH_TASK aTask )
    {
        auto* event = new wxThreadEvent();
        event->SetPayload( std::move( aTask ) );
        aGate.Queue( event );
    }
};


/// @brief Headless: the document owner strand, one std::thread running queued closures in a
/// MODEL_THREAD writer scope, with no wx event dispatch.  The main thread never touches the
/// documents while it runs.
class OWNER_STRAND_DISPATCH : public CALL_DISPATCH
{
public:
    OWNER_STRAND_DISPATCH() : m_thread( STACK_BYTES, [this]() { run(); } ) {}

    ~OWNER_STRAND_DISPATCH() override { Stop(); }

    void QueueCall( DISPATCH_TASK aTask ) override { push( m_calls, std::move( aTask ) ); }

    void QueueJob( DISPATCH_TASK aTask ) override { push( m_jobs, std::move( aTask ) ); }

    void ServeQueuedCalls() override
    {
        size_t left;

        {
            std::lock_guard<std::mutex> lock( m_mutex );
            left = m_calls.size();
        }

        for( ; left > 0; --left )
        {
            DISPATCH_TASK task;

            {
                std::lock_guard<std::mutex> lock( m_mutex );

                if( m_calls.empty() )
                    return;

                task = std::move( m_calls.front() );
                m_calls.pop_front();
            }

            ( *task )();
        }
    }

    bool OnDispatchThread() const override { return std::this_thread::get_id() == m_threadId.load(); }

    /// @brief Runs what is still queued (the tasks see the service stopping) and joins
    void Stop()
    {
        {
            std::lock_guard<std::mutex> lock( m_mutex );
            m_stop = true;
        }

        m_cv.notify_all();

        if( !OnDispatchThread() )
            m_thread.Join();
    }

private:
    void push( std::deque<DISPATCH_TASK>& aQueue, DISPATCH_TASK aTask )
    {
        {
            std::lock_guard<std::mutex> lock( m_mutex );
            aQueue.push_back( std::move( aTask ) );
        }

        m_cv.notify_one();
    }

    void run()
    {
        m_threadId = std::this_thread::get_id();
        wxLog::SetThreadActiveTarget( KOPENAPI_JOURNAL::ThreadTarget() );
        MODEL_THREAD::SCOPE owner( MODEL_THREAD::KIND::OWNER );

        for( ;; )
        {
            DISPATCH_TASK task;

            {
                std::unique_lock<std::mutex> lock( m_mutex );
                m_cv.wait( lock, [this]() { return m_stop || !m_calls.empty() || !m_jobs.empty(); } );

                std::deque<DISPATCH_TASK>& queue = !m_calls.empty() ? m_calls : m_jobs;

                if( queue.empty() )
                    break;

                task = std::move( queue.front() );
                queue.pop_front();
            }

            ( *task )();
        }

        wxLog::SetThreadActiveTarget( nullptr );
    }

    std::mutex                m_mutex;
    std::condition_variable   m_cv;
    std::deque<DISPATCH_TASK> m_calls;
    std::deque<DISPATCH_TASK> m_jobs;
    bool                      m_stop = false;
    std::atomic<std::thread::id> m_threadId;

    /// KiCad code is written for the main thread's stack (8 MB); handlers keep large frames
    static constexpr size_t   STACK_BYTES = 16 * 1024 * 1024;

    kopenapi::platform::THREAD m_thread;
};


static MAIN_LOOP_DISPATCH                 s_mainLoopDispatch;
static std::atomic<CALL_DISPATCH*>        s_dispatch{ &s_mainLoopDispatch };


static CALL_DISPATCH& dispatch()
{
    return *s_dispatch.load();
}


void KopenapiServeQueuedCalls()
{
    dispatch().ServeQueuedCalls();
}


bool KopenapiOnDispatchThread()
{
    return dispatch().OnDispatchThread();
}


/// @brief How long a lease read waits for the owner's write before it queues on the strand
static constexpr std::chrono::milliseconds LEASE_WAIT{ 200 };

/// @brief A lease read past this while a write waits is journaled (revoked, then stuck)
static constexpr std::chrono::milliseconds LEASE_BUDGET{ 2000 };


/// @brief The owner strand's write through MODEL_THREAD::WRITE_SECTION
static thread_local std::optional<KOPENAPI_WRITE_SCOPE> tlOwnerWrite;


/// @brief Run aFn on the main (GUI/event-loop) thread and wait for its result on the calling HTTP
/// worker.  The main thread never waits on HTTP threads.  If the service stops or the call
/// exceeds its timeout, the worker returns an error, and what the caller is told matches what
/// happens: a call that had not started is withdrawn and never runs ("not started"); one that had
/// started runs to its end and its effect stands ("still running", see edit_log).  A late
/// closure only touches shared state.
static KOPENAPI_RESULT runInMain( const std::shared_ptr<std::atomic<bool>>& aAlive,
                                  const std::function<void()>&             aWaker,
                                  std::function<KOPENAPI_RESULT()>         aFn,
                                  std::chrono::seconds                     aTimeout = CALL_TIMEOUT,
                                  const std::string&                       aMethod = std::string() )
{
    if( !wxTheApp )
        return KOPENAPI_RESULT::Error( 503, "no application event loop" );

    auto promise = std::make_shared<std::promise<KOPENAPI_RESULT>>();
    std::shared_future<KOPENAPI_RESULT> future = promise->get_future().share();

    auto ticket = std::make_shared<KOPENAPI_CALL_TICKET>();

    auto job = std::make_shared<std::function<void()>>(
            [aAlive, promise, ticket, fn = std::move( aFn )]()
            {
                // The caller gave up before the call started: it must not run now
                if( !ticket->Start() )
                {
                    promise->set_value( KOPENAPI_RESULT::Error( 504, "withdrawn before it started" ) );
                    return;
                }

                if( !aAlive->load() )
                {
                    promise->set_value( KOPENAPI_RESULT::Error( 503, "service stopping" ) );
                    return;
                }

                try
                {
                    promise->set_value( fn() );
                }
                catch( const KOPENAPI_RETRY_LATER& e )
                {
                    promise->set_value( e.Result() );
                }
                catch( const std::exception& e )
                {
                    promise->set_value( KOPENAPI_RESULT::Error( 500, e.what() ) );
                }
                catch( ... )
                {
                    promise->set_value( KOPENAPI_RESULT::Error( 500, "unknown exception" ) );
                }
            } );

    dispatch().QueueCall( job );

    if( aWaker )
        aWaker();

    const auto deadline = std::chrono::steady_clock::now() + aTimeout;

    while( future.wait_for( WAIT_SLICE ) != std::future_status::ready )
    {
        const bool stopping = !aAlive->load();

        if( !stopping && std::chrono::steady_clock::now() <= deadline )
            continue;

        if( ticket->Abandon() )
        {
            return stopping ? KOPENAPI_RESULT::Error( 503, "service stopping; the call did not start and will not run" )
                            : KOPENAPI_RESULT::Error( 504, "the main thread was busy for "
                                                                   + std::to_string( aTimeout.count() )
                                                                   + " s; the call did not start and will not run "
                                                                     "(safe to retry)" );
        }

        // Started: it runs to its end on the main thread and its changes stand
        if( stopping )
            return KOPENAPI_RESULT::Error( 503, "service stopping while the call was running; it may "
                                                "have completed (edit_log)" );

        const std::string callId = keepLateCall( aMethod, future );
        KOPENAPI_RESULT   late = KOPENAPI_RESULT::Error( 504, "the call started but did not finish within "
                                                                  + std::to_string( aTimeout.count() )
                                                                  + " s; it is still running and its changes "
                                                                    "will stand: api_call_result {call_id: \""
                                                                  + callId + "\"} gives its result" );
        late.body["call_id"] = callId;
        return late;
    }

    return future.get();
}


/// @brief How late the GUI main thread serves its native events (KICAD_OPENAPI_MAIN_PROBE=1): a
/// 20 ms timer on the main thread; each sample is how much later than due it fired, the wait a
/// paint or a wheel event has too.  GET /api/v1/main_loop reads it (?reset=1 clears).
/// (A posted call, CallAfter, would measure wx's own queue, which yields for paint and input skip.)
class MAIN_LOOP_PROBE : public wxTimer
{
public:
    MAIN_LOOP_PROBE()
    {
        m_last = std::chrono::steady_clock::now();
        Start( INTERVAL_MS );
    }

    ~MAIN_LOOP_PROBE() override { Stop(); }

    void Notify() override
    {
        const auto   now = std::chrono::steady_clock::now();
        const double late = std::chrono::duration<double, std::milli>( now - m_last ).count() - INTERVAL_MS;
        m_last = now;

        std::lock_guard<std::mutex> lock( m_mutex );

        if( m_samples.size() < MAX_SAMPLES )
            m_samples.push_back( std::max( 0.0, late ) );
    }

    nlohmann::json Json( bool aReset )
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        std::vector<double>         v = m_samples;
        nlohmann::json              out = { { "n", v.size() }, { "interval_ms", INTERVAL_MS } };

        if( !v.empty() )
        {
            std::sort( v.begin(), v.end() );
            auto at = [&]( double q ) { return v[(size_t) ( q * ( v.size() - 1 ) )]; };
            out["p50_ms"] = at( 0.5 );
            out["p95_ms"] = at( 0.95 );
            out["p99_ms"] = at( 0.99 );
            out["max_ms"] = v.back();
            out["over_50ms"] = std::count_if( v.begin(), v.end(), []( double x ) { return x > 50; } );
            out["over_100ms"] = std::count_if( v.begin(), v.end(), []( double x ) { return x > 100; } );
            out["over_250ms"] = std::count_if( v.begin(), v.end(), []( double x ) { return x > 250; } );
        }

        if( aReset )
            m_samples.clear();

        return out;
    }

private:
    static constexpr int    INTERVAL_MS = 20;
    static constexpr size_t MAX_SAMPLES = 1 << 20;

    std::chrono::steady_clock::time_point m_last;
    std::mutex                            m_mutex;
    std::vector<double>                   m_samples;
};


struct KICAD_OPENAPI_SERVICE::IMPL
{
    KOPENAPI_CONTEXT                  ctx;
    std::unique_ptr<httplib::Server>  server;
    std::thread                       thread;
    int                               port = 0;
    std::string                       appName;
    std::shared_ptr<std::atomic<bool>> alive = std::make_shared<std::atomic<bool>>( false );
    fs::path                          discoveryFile;
    std::function<void()>             waker;
    std::function<void()>             shutdownHandler;
    std::unique_ptr<OWNER_STRAND_DISPATCH> strand;
    std::unique_ptr<KOPENAPI_DOC_GATE>     gate;   ///< every document; the owner (strand or main thread) writes
    mutable std::atomic<uint64_t>          leaseReadsServed{ 0 };
    mutable std::atomic<uint64_t>          leaseReadsToOwner{ 0 };   ///< leased, then answered by the owner

    /// GUI: the main thread owns the documents and holds the write except in KopenapiCompute
    std::optional<MODEL_THREAD::SCOPE>         mainOwner;
    std::optional<MODEL_THREAD::WRITE_SECTION> mainWrite;

    /// How long a lease read waits for the owner's write: headless the strand's writes are short;
    /// GUI the main thread writes until it computes, so a read that finds it writing queues at once
    std::chrono::milliseconds leaseWait{ 0 };

    std::unique_ptr<MAIN_LOOP_PROBE> mainProbe;

    /// @brief A lease read on this (HTTP) thread; nullopt: not granted in time or retry on the strand
    std::optional<KOPENAPI_RESULT> leaseRead( const std::string& aName, const nlohmann::json& aArgs ) const;

    nlohmann::json gateJson() const;

    /// @brief GET /api/v1/progress: the running call's progress without touching the main thread
    nlohmann::json progressJson() const
    {
        const std::string job = KOPENAPI_JOBS::Get().Busy();

        return { { "busy", busyJson() },
                 { "running", KopenapiRunningProgress() },
                 { "job", job.empty() ? nlohmann::json() : nlohmann::json( job ) } };
    }

    nlohmann::json statusJson() const
    {
        return { { "service", "kicadopenapi" },
                 { "version", KOPENAPI_VERSION },
                 { "app", appName },
                 { "pid", kopenapi::platform::CurrentPid() },
                 { "port", port },
                 { "headless", ctx.headless },
                 { "busy", busyJson() },
                 { "model_gate", gateJson() },
                 { "claim", claimJson() },
                 { "changes", KOPENAPI_CHANGE_FEED::Status() },
                 { "job", KOPENAPI_JOBS::Get().Busy().empty() ? nlohmann::json() : nlohmann::json( KOPENAPI_JOBS::Get().Busy() ) } };
    }

    /// @brief Who holds the documents as their only writer, or null; any thread.
    /// long: a queued or running job, or a call that serves others at its safe points: other
    /// edits are refused (409 with this claim), reads are answered.  short: a call that runs
    /// to its end: edits posted meanwhile wait and run after it, in order.
    nlohmann::json claimJson() const
    {
        if( const std::string job = KOPENAPI_JOBS::Get().Busy(); !job.empty() )
            return { { "kind", "long" }, { "source", "job:" + job } };

        std::lock_guard<std::mutex> lock( traceMutex );

        if( busyMethod.empty() )
            return nullptr;

        return { { "kind", KopenapiJobYielding() ? "long" : "short" }, { "source", "api:" + busyMethod } };
    }

    /// @brief A 409 for an edit refused by a long claim
    KOPENAPI_RESULT refusedByClaim( const std::string& aWhy ) const
    {
        KOPENAPI_RESULT result = KOPENAPI_RESULT::Error( 409, aWhy );

        if( result.body.contains( "error" ) && result.body["error"].is_object() )
            result.body["error"]["claim"] = claimJson();

        return result;
    }

    /// @brief The call holding the main thread (method, running_ms, progress), or null; any thread
    nlohmann::json busyJson() const
    {
        std::lock_guard<std::mutex> lock( traceMutex );
        return busyLocked();
    }

    /// @brief busyJson() under traceMutex
    nlohmann::json busyLocked() const
    {
        if( busyMethod.empty() )
            return nullptr;

        nlohmann::json busy = { { "method", busyMethod },
                                { "running_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
                                                        std::chrono::steady_clock::now() - busySince ).count() } };

        if( nlohmann::json progress = KopenapiCallProgress(); !progress.is_null() )
            busy["progress"] = progress;

        return busy;
    }

    nlohmann::json openApiJson() const;

    /// @brief statusJson() + open documents and the unsaved flag (main-thread round trip)
    KOPENAPI_RESULT liveStatus() const;

    /// @brief Load the editor kifaces (eeschema, pcbnew) once, on the main thread, so their methods
    /// and document providers are registered.  Lazy: paid by the first API use, not at start.
    void ensureKifaces() const;

    mutable std::atomic<bool> kifacesLoaded{ false };

    /// @brief One API call through the gate, for api_trace
    struct TRACE
    {
        long long   seq = 0;
        std::string method;
        size_t      argBytes = 0;
        double      queuedMs = 0;   ///< waiting for the main thread
        double      runMs = 0;      ///< running there
        int         status = 0;
        std::string at;             ///< wall clock when it was received
        std::string args;           ///< arguments, cut at 2 KB
        std::string error;          ///< error message of a failed call

        nlohmann::json Json() const
        {
            nlohmann::json j = { { "seq", seq }, { "method", method }, { "at", at }, { "arg_bytes", argBytes },
                                 { "queued_ms", queuedMs }, { "run_ms", runMs }, { "status", status },
                                 { "args", args } };

            if( !error.empty() )
                j["error"] = error;

            return j;
        }
    };

    mutable std::atomic<bool> traceOn{ false };  ///< off: the gate records nothing (no buffer, no cost)
    mutable std::mutex        traceMutex;
    mutable std::deque<TRACE> traces;            ///< newest last, at most traceLimit bytes
    mutable std::deque<size_t> traceSizes;       ///< bytes of each entry (its JSON line)
    mutable size_t            traceBytes = 0;
    mutable size_t            traceLimit = 1 << 20;
    mutable std::string       traceFile;         ///< JSON lines appended per call, empty: off
    mutable std::ofstream     traceStream;
    bool                      traceToJournal = false;   ///< env KICAD_OPENAPI_TRACE
    mutable long long         traceSeq = 0;
    mutable std::string       busyMethod;        ///< the call running on the main thread now
    mutable std::chrono::steady_clock::time_point busySince;
    mutable nlohmann::json    lastDocuments = nlohmann::json::array();

    nlohmann::json traceJson( const nlohmann::json& aArgs ) const;
    std::thread               preloadThread;   ///< joined in Stop()
    std::thread               publishThread;   ///< restart: publishes once the reopened documents are open
    std::atomic<bool>         startupOpened{ false };   ///< the host finished its start-up opening

    void writeDiscoveryFile();

    KOPENAPI_RESULT invoke( const std::string& aName, const std::string& aBody, const std::string& aCaller = {} ) const;

    /// @brief Same as invoke() with already parsed arguments (REST and MCP share this path)
    /// @param aCaller the X-Kicad-Session header: the client's identity for remote control sessions
    KOPENAPI_RESULT invokeParsed( const std::string& aName, const nlohmann::json& aArgs,
                                  const std::string& aCaller = {} ) const;

    /// @brief The method exists, may run here, and its arguments match its schema; else the error
    std::optional<KOPENAPI_RESULT> checkCall( const std::string& aName, const nlohmann::json& aArgs,
                                              std::optional<KOPENAPI_METHOD>& aMethod ) const;

    /// @brief The call as it runs on the main thread (history marks, journal, busy state);
    /// aStarted (tracing) gets the time it started
    std::function<KOPENAPI_RESULT()> mainCall( const std::string& aName, const nlohmann::json& aArgs,
                                               std::shared_ptr<std::chrono::steady_clock::time_point> aStarted ) const;

    /// @brief job_start: checks the call, queues it as a job, answers its job_id at once
    KOPENAPI_RESULT startJob( const nlohmann::json& aArgs, const std::string& aCaller ) const;

    /// @brief Queues a checked call as a job; its id, else empty and aRefused (409 / 503)
    std::string queueJob( const std::string& aName, const nlohmann::json& aArgs,
                          std::shared_ptr<std::chrono::steady_clock::time_point> aStarted,
                          KOPENAPI_RESULT& aRefused, bool aPlainCall = false ) const;

    void traceCall( const std::string& aName, const nlohmann::json& aArgs, const KOPENAPI_RESULT& aOutcome,
                    std::chrono::steady_clock::time_point aReceived,
                    std::chrono::steady_clock::time_point aStarted ) const;

    void handleMcp( const httplib::Request& aReq, httplib::Response& aRes ) const;

    void registerRoutes();
};


void KICAD_OPENAPI_SERVICE::IMPL::ensureKifaces() const
{
    if( kifacesLoaded.load() || !ctx.kiway )
        return;

    KIWAY* kiway = ctx.kiway;

    runInMain( alive, waker,
               [kiway]()
               {
                   for( KIWAY::FACE_T face : { KIWAY::FACE_SCH, KIWAY::FACE_PCB } )
                   {
                       try
                       {
                           kiway->KiFACE( face );
                       }
                       catch( ... )
                       {
                           // A missing kiface only means its methods are unavailable
                       }
                   }

                   return KOPENAPI_RESULT::Ok( nlohmann::json::object() );
               } );

    kifacesLoaded.store( true );
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::IMPL::liveStatus() const
{
    ensureKifaces();

    std::optional<KOPENAPI_RESULT> leased;

    if( gate )
        leased = leaseRead( "documents", nlohmann::json::object() );

    // while a call holds the main thread, answer with the documents seen last
    if( !leased && !busyJson().is_null() )
    {
        nlohmann::json status = statusJson();
        std::lock_guard<std::mutex> lock( traceMutex );
        status["documents"] = lastDocuments;
        status["documents_stale"] = true;
        status["journal"] = KOPENAPI_JOURNAL::Summary();
        return KOPENAPI_RESULT::Ok( status );
    }

    KOPENAPI_CONTEXT ctxCopy = ctx;
    KOPENAPI_RESULT  docs = leased && leased->status == 200
                                    ? KOPENAPI_RESULT::Ok( std::move( leased->body["documents"] ) )
                                    : runInMain( alive, waker,
                                                 [ctxCopy]() mutable
                                                 {
                                                     return KOPENAPI_RESULT::Ok(
                                                             KOPENAPI_REGISTRY::Get().Documents( ctxCopy ) );
                                                 } );

    if( docs.status != 200 )
        return docs;

    {
        std::lock_guard<std::mutex> lock( traceMutex );
        lastDocuments = docs.body;
    }

    nlohmann::json status = statusJson();
    bool           unsaved = false;

    for( const nlohmann::json& doc : docs.body )
        unsaved |= doc.value( "unsaved", false );

    status["documents"] = docs.body;
    status["unsaved"] = unsaved;
    status["journal"] = KOPENAPI_JOURNAL::Summary();
    return KOPENAPI_RESULT::Ok( status );
}


nlohmann::json KICAD_OPENAPI_SERVICE::IMPL::openApiJson() const
{
    ensureKifaces();

    using nlohmann::json;

    const json errorRef = { { "$ref", "#/components/schemas/Error" } };
    const json errorResponse = {
        { "description", "Error" },
        { "content", { { "application/json", { { "schema", errorRef } } } } }
    };

    json paths = json::object();

    paths["/api/v1/status"] = {
        { "get",
          { { "operationId", "status" },
            { "summary", "Service and process information" },
            { "responses",
              { { "200",
                  { { "description", "Status" },
                    { "content",
                      { { "application/json",
                          { { "schema", { { "$ref", "#/components/schemas/Status" } } } } } } } } } } } } }
    };

    paths["/api/v1/openapi.json"] = {
        { "get",
          { { "operationId", "openapi" },
            { "summary", "This OpenAPI manifest" },
            { "responses", { { "200", { { "description", "OpenAPI 3.0 document" } } } } } } }
    };

    paths["/mcp"] = {
        { "post",
          { { "operationId", "mcp" },
            { "summary", "MCP endpoint (JSON-RPC 2.0, Streamable HTTP): tools search + invoke" },
            { "requestBody",
              { { "required", true },
                { "content",
                  { { "application/json",
                      { { "schema", { { "type", "object" } } } } } } } } },
            { "responses",
              { { "200", { { "description", "JSON-RPC response" } } },
                { "202", { { "description", "Notification accepted" } } } } } } }
    };

    {
        json topics = json::array();
        std::string described;

        for( const auto& [name, topic] : KOPENAPI_REGISTRY::Get().EventTopics() )
        {
            topics.push_back( name );
            described += "; " + name + ": " + topic.summary;
        }

        paths["/api/v1/events"] = {
            { "get",
              { { "operationId", "events" },
                { "summary", "Server-Sent Events (REST only, not MCP): a topic's changes pushed as they "
                             "happen, coalesced to the latest state at most ~30 per second; the other query "
                             "parameters are the topic's (JSON values, else strings)" + described },
                { "parameters",
                  { { { "name", "topic" }, { "in", "query" }, { "required", true },
                      { "schema", { { "type", "string" }, { "enum", topics } } } } } },
                { "responses",
                  { { "200", { { "description", "text/event-stream" },
                               { "content", { { "text/event-stream", { { "schema", { { "type", "string" } } } } } } } } },
                    { "404", errorResponse } } } } }
        };
    }

    paths["/api/v1/progress"] = {
        { "get",
          { { "operationId", "progress" },
            { "summary", "The running call's progress, answered on the HTTP thread (never waits for KiCad): "
                         "busy (method, running_ms, progress), running ({method, rev, progress, job_id + paused "
                         "for a job} of the call reporting progress, else null), job (the queued or running "
                         "job). Pushed instead: GET /api/v1/events?topic=progress; a job: job_get {since}" },
            { "responses", { { "200", { { "description", "progress" } } } } } } }
    };

    if( shutdownHandler )
    {
        paths["/api/v1/shutdown"] = {
            { "post",
              { { "operationId", "shutdown" },
                { "summary", "Stop this headless process" },
                { "responses", { { "200", { { "description", "Shutdown started" } } } } } } }
        };
    }

    for( const KOPENAPI_METHOD& m : KOPENAPI_REGISTRY::Get().Snapshot() )
    {
        if( shutdownHandler && m.name == "shutdown" )
            continue;   // the built-in route wins

        json op = {
            { "operationId", m.name },
            { "summary", m.summary },
            { "x-gui-only", m.guiOnly },
            { "requestBody",
              { { "required", true },
                { "content", { { "application/json", { { "schema", m.inputSchema } } } } } } },
            { "responses",
              { { "200",
                  { { "description", "Success" },
                    { "content",
                      { { "application/json",
                          { { "schema", { { "type", "object" } } } } } } } } },
                { "default", errorResponse } } }
        };

        paths["/api/v1/" + m.name] = { { "post", op } };
    }

    return {
        { "openapi", "3.0.3" },
        { "info",
          { { "title", "kicadopenapi" },
            { "version", KOPENAPI_VERSION },
            { "description", "In-process Web API of a KiCad process (" + appName + ")" } } },
        { "servers", json::array( { { { "url", "http://" + std::string( KOPENAPI_HOST_ADDR ) + ":"
                                                     + std::to_string( port ) } } } ) },
        { "paths", paths },
        { "components",
          { { "schemas",
              { { "Error",
                  { { "type", "object" },
                    { "required", { "error" } },
                    { "properties",
                      { { "error",
                          { { "type", "object" },
                            { "required", { "code", "message" } },
                            { "properties",
                              { { "code", { { "type", "integer" } } },
                                { "message", { { "type", "string" } } } } } } } } } } },
                { "Status",
                  { { "type", "object" },
                    { "properties",
                      { { "service", { { "type", "string" } } },
                        { "version", { { "type", "string" } } },
                        { "app", { { "type", "string" } } },
                        { "pid", { { "type", "integer" } } },
                        { "port", { { "type", "integer" } } },
                        { "headless", { { "type", "boolean" } } },
                        { "unsaved", { { "type", "boolean" } } },
                        { "journal", { { "type", "object" } } },
                        { "documents", { { "type", "array" }, { "items", { { "type", "object" } } } } } } } } } } } } }
    };
}


void KICAD_OPENAPI_SERVICE::IMPL::writeDiscoveryFile()
{
    std::error_code ec;
    fs::path        dir = kopenapi::platform::DiscoveryDir();

    fs::create_directories( dir, ec );
    removeStaleDiscoveryFiles( dir );

    const std::string base = "http://" + std::string( KOPENAPI_HOST_ADDR ) + ":"
                             + std::to_string( port );
    nlohmann::json info = statusJson();
    info["url"] = base;
    info["mcp"] = base + "/mcp";

    // Write-then-rename so readers never see a partial file
    fs::path file = dir / ( std::to_string( kopenapi::platform::CurrentPid() ) + ".json" );
    fs::path tmp = file;
    tmp += ".tmp";

    {
        std::ofstream out( tmp, std::ios::trunc );
        out << toJson( info ) << "\n";
    }

    fs::rename( tmp, file, ec );

    if( ec )
        fs::remove( tmp, ec );
    else
        discoveryFile = file;
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::IMPL::invoke( const std::string& aName,
                                                     const std::string& aBody,
                                                     const std::string& aCaller ) const
{
    nlohmann::json args = nlohmann::json::object();

    if( !aBody.empty() )
    {
        args = nlohmann::json::parse( aBody, nullptr, false );

        if( args.is_discarded() || !args.is_object() )
            return KOPENAPI_RESULT::Error( 400, "request body must be a JSON object" );
    }

    return invokeParsed( aName, args, aCaller );
}


/// Every point of an answer also as at_mm [x, y] (the routing and DRC answers' shape): an object
/// with numeric x_mm and y_mm gets it, unless it is a rectangle (width_mm) or has one already
static void addPoints( nlohmann::json& aJson, int aDepth )
{
    if( aDepth > 64 )
        return;

    if( aJson.is_object() )
    {
        if( !aJson.contains( "at_mm" ) && !aJson.contains( "width_mm" ) && aJson.contains( "x_mm" )
            && aJson.contains( "y_mm" ) && aJson["x_mm"].is_number() && aJson["y_mm"].is_number() )
        {
            aJson["at_mm"] = { aJson["x_mm"], aJson["y_mm"] };
        }

        for( auto& [key, value] : aJson.items() )
        {
            if( value.is_structured() )
                addPoints( value, aDepth + 1 );
        }
    }
    else if( aJson.is_array() )
    {
        for( nlohmann::json& value : aJson )
        {
            if( value.is_structured() )
                addPoints( value, aDepth + 1 );
        }
    }
}


std::optional<KOPENAPI_RESULT> KICAD_OPENAPI_SERVICE::IMPL::checkCall( const std::string&              aName,
                                                                      const nlohmann::json&           aArgs,
                                                                      std::optional<KOPENAPI_METHOD>& aMethod ) const
{
    std::optional<KOPENAPI_METHOD>& method = aMethod;
    method = KOPENAPI_REGISTRY::Get().Find( aName );

    if( !method )
    {
        const std::vector<std::string> similar = KOPENAPI_REGISTRY::Get().Similar( aName, 6, !ctx.headless );
        std::string                    hint;

        for( const std::string& name : similar )
            hint += ( hint.empty() ? " (did you mean " : ", " ) + name;

        KOPENAPI_RESULT result = KOPENAPI_RESULT::Error( 404, "unknown method '" + aName + "'" + ( hint.empty() ? "" : hint + "?)" ) );
        result.body["error"]["similar"] = similar;
        return result;
    }

    if( method->guiOnly && ctx.headless )
        return KOPENAPI_RESULT::Error( 501, "'" + aName + "' requires the GUI"
                                                    + std::string( aName == "app_quit" ? "; headless: POST /api/v1/shutdown" : "" ) );

    // unknown names first (a misspelt required name reads better as "did you mean"), then the
    // required ones and the plain top-level types (short messages), then the whole inputSchema
    // (nested types, enums, ranges)
    if( const std::vector<std::string> known = KopenapiKnownArgs( method->inputSchema ); !known.empty() )
    {
        for( const auto& [key, value] : aArgs.items() )
        {
            if( std::find( known.begin(), known.end(), key ) == known.end() )
                return KOPENAPI_RESULT::Error( 400, KopenapiValidateArgs( aName, method->inputSchema, aArgs ) );
        }
    }

    if( method->inputSchema.contains( "required" ) )
    {
        for( const nlohmann::json& field : method->inputSchema["required"] )
        {
            if( field.is_string() && !aArgs.contains( field.get<std::string>() ) )
                return KOPENAPI_RESULT::Error( 400, "missing required field '"
                                                            + field.get<std::string>() + "'" );
        }
    }

    if( const std::string bad = checkArgs( method->inputSchema, aArgs ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad );

    // unknown names, nested types, enums, ranges: the whole inputSchema
    if( const std::string bad = KopenapiValidateArgs( aName, method->inputSchema, aArgs ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad );

    return std::nullopt;
}


/// @brief How long a plain call of a method that may run longer waits for it before it answers the
/// job it runs as; env KICAD_OPENAPI_SYNC_JOB_S sets it (tests)
static std::chrono::seconds syncJobWait()
{
    static const int sec = []
    {
        const char* env = std::getenv( "KICAD_OPENAPI_SYNC_JOB_S" );
        const int   v = env ? std::atoi( env ) : 0;
        return v > 0 ? v : 240;
    }();

    return std::chrono::seconds( sec );
}


/// @brief The job's own result once it ends within aWait, else {job_id, running: true, hint}
static KOPENAPI_RESULT awaitJob( const std::shared_ptr<std::atomic<bool>>& aAlive, const std::string& aId,
                                 const std::string& aMethod, std::chrono::seconds aWait )
{
    const auto      deadline = std::chrono::steady_clock::now() + aWait;
    KOPENAPI_RESULT result;

    while( !KOPENAPI_JOBS::Get().Wait( aId, std::chrono::milliseconds( 250 ), result ) )
    {
        if( !aAlive->load() )
            return KOPENAPI_RESULT::Error( 503, "service stopping while " + aId + " (" + aMethod
                                                        + ") was running; it may have completed (edit_log)" );

        if( std::chrono::steady_clock::now() < deadline )
            continue;

        KOPENAPI_RESULT status = KOPENAPI_JOBS::Get().Status( { { "job_id", aId } } );
        nlohmann::json  body = { { "job_id", aId },
                                 { "method", aMethod },
                                 { "running", true },
                                 { "waited_s", aWait.count() },
                                 { "hint", "not done within " + std::to_string( aWait.count() )
                                                   + " s, it runs on as a job and its changes will stand: "
                                                     "job_get {job_id: \"" + aId + "\", wait_s: 60} gives its "
                                                     "result, job_cancel stops it (edits get 409 meanwhile)" } };

        if( status.body.is_object() )
        {
            for( const char* key : { "state", "progress", "rev" } )
            {
                if( status.body.contains( key ) )
                    body[key] = status.body[key];
            }
        }

        return KOPENAPI_RESULT::Ok( body );
    }

    return result;
}


/// @brief Methods the service answers itself on the HTTP thread: no main thread, no job
static bool servedByService( const std::string& aName )
{
    static const std::set<std::string> names = { "api_trace",  "memory_summary", "api_call_result", "remote_session",
                                                 "job_start",  "job_get",        "job_cancel",      "job_list",
                                                 "job_pause",  "job_resume",     "job_step" };
    return names.count( aName ) > 0;
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::IMPL::startJob( const nlohmann::json& aArgs, const std::string& aCaller ) const
{
    const std::string    name = aArgs.value( "method", std::string() );
    const nlohmann::json args = aArgs.value( "arguments", nlohmann::json::object() );

    if( servedByService( name ) )
        return KOPENAPI_RESULT::Error( 400, "'" + name + "' answers at once: call it directly" );

    if( std::optional<KOPENAPI_RESULT> refused = KOPENAPI_REMOTE::Gate( aCaller, name ) )
        return *refused;

    std::optional<KOPENAPI_METHOD> method;

    if( std::optional<KOPENAPI_RESULT> bad = checkCall( name, args, method ) )
        return *bad;

    KOPENAPI_RESULT   refused;
    const std::string id = queueJob( name, args, nullptr, refused );

    if( id.empty() )
        return refused;

    return KOPENAPI_RESULT::Ok( { { "job_id", id }, { "method", name }, { "state", "queued" } } );
}


std::string KICAD_OPENAPI_SERVICE::IMPL::queueJob( const std::string& aName, const nlohmann::json& aArgs,
                                                   std::shared_ptr<std::chrono::steady_clock::time_point> aStarted,
                                                   KOPENAPI_RESULT& aRefused, bool aPlainCall ) const
{
    std::string       why;
    const std::string id = KOPENAPI_JOBS::Get().Add( aName, why, aPlainCall );

    if( id.empty() )
    {
        aRefused = KOPENAPI_RESULT::Error( 409, why );
        return id;
    }

    if( !wxTheApp )
    {
        aRefused = KOPENAPI_RESULT::Error( 503, "no application event loop" );
        KOPENAPI_JOBS::Get().Fail( id, aRefused );
        return std::string();
    }

    auto job = std::make_shared<std::function<void()>>(
            [id, call = mainCall( aName, aArgs, std::move( aStarted ) ), aliveFlag = alive]()
            {
                if( !aliveFlag->load() )
                    KOPENAPI_JOBS::Get().Fail( id, KOPENAPI_RESULT::Error( 503, "service stopping" ) );
                else
                    KOPENAPI_JOBS::Get().Run( id, call );
            } );

    dispatch().QueueJob( job );

    if( waker )
        waker();

    return id;
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::IMPL::invokeParsed( const std::string&    aName,
                                                           const nlohmann::json& aArgs,
                                                           const std::string&    aCaller ) const
{
    ensureKifaces();

    // a client the user took control back from is stopped; the holder's calls renew its lease
    if( std::optional<KOPENAPI_RESULT> refused = KOPENAPI_REMOTE::Gate( aCaller, aName ) )
        return *refused;

    if( aName == "remote_session" )
        return KOPENAPI_REMOTE::Handle( aCaller, aArgs );

    std::optional<KOPENAPI_METHOD> method;

    if( std::optional<KOPENAPI_RESULT> bad = checkCall( aName, aArgs, method ) )
        return *bad;

    // answered on the HTTP thread: works while the main thread is busy
    if( aName == "api_trace" )
        return KOPENAPI_RESULT::Ok( traceJson( aArgs ) );

    if( aName == "memory_summary" )
        return KOPENAPI_RESULT::Ok( KOPENAPI_MEMORY::Summary() );

    if( aName == "api_call_result" )
        return lateCallResult( aArgs );

    if( aName == "job_start" )
        return startJob( aArgs, aCaller );

    if( aName == "job_get" )
        return KOPENAPI_JOBS::Get().Status( aArgs );

    if( aName == "job_cancel" )
        return KOPENAPI_JOBS::Get().Cancel( aArgs );

    if( aName == "job_list" )
        return KOPENAPI_JOBS::Get().List();

    if( aName == "job_pause" || aName == "job_resume" )
        return KOPENAPI_JOBS::Get().Pause( aArgs, aName == "job_pause" );

    if( aName == "job_step" )
        return KOPENAPI_JOBS::Get().Step( aArgs );

    if( KOPENAPI_REGISTRY::Get().IsEditing( aName ) )
    {
        if( const std::string job = KOPENAPI_JOBS::Get().Busy(); !job.empty() )
            return refusedByClaim( "job " + job + " is queued or running and owns the documents: "
                                   "job_get waits for it, job_cancel stops it" );
    }

    const bool tracing = traceOn.load( std::memory_order_relaxed );
    const auto received = tracing ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point();
    auto       started = tracing ? std::make_shared<std::chrono::steady_clock::time_point>( received ) : nullptr;

    if( gate && KOPENAPI_REGISTRY::Get().IsConcurrentRead( aName ) )
    {
        if( std::optional<KOPENAPI_RESULT> read = leaseRead( aName, aArgs ) )
        {
            if( tracing )
                traceCall( aName, aArgs, *read, received, received );

            return *read;
        }
    }

    const std::chrono::seconds timeout = callTimeout( method->timeoutSec );
    std::optional<KOPENAPI_RESULT> outcomeAsJob;

    // may outlast the client's wait (bridge: kPostReceiveTimeoutSeconds): runs as a job and answers
    // its job_id when it is not done by then
    if( timeout > syncJobWait() )
    {
        KOPENAPI_RESULT   refused;
        const std::string id = queueJob( aName, aArgs, started, refused, true );

        if( !id.empty() )
            outcomeAsJob = awaitJob( alive, id, aName, syncJobWait() );
        else if( KOPENAPI_REGISTRY::Get().IsEditing( aName ) || refused.status != 409 )
            return refused;
    }

    KOPENAPI_RESULT outcome = outcomeAsJob ? std::move( *outcomeAsJob )
                                           : runInMain( alive, waker, mainCall( aName, aArgs, started ), timeout, aName );

    if( !tracing )
        return outcome;

    traceCall( aName, aArgs, outcome, received, *started );
    return outcome;
}


std::optional<KOPENAPI_RESULT> KICAD_OPENAPI_SERVICE::IMPL::leaseRead( const std::string&    aName,
                                                                     const nlohmann::json& aArgs ) const
{
    const std::string   holder = "api:" + aName;
    KOPENAPI_READ_LEASE lease = gate->Acquire( holder, std::nullopt,
                                               std::chrono::duration_cast<std::chrono::microseconds>( leaseWait ) );

    // GUI: while a call or job holds the main thread, it lends the documents at its next compute
    // (a queued call would wait for the whole call); an idle main thread answers at once
    while( !lease && !ctx.headless && alive->load()
           && ( !busyJson().is_null() || !KOPENAPI_JOBS::Get().Busy().empty() ) )
    {
        lease = gate->Acquire( holder, std::nullopt, std::chrono::milliseconds( 100 ) );
    }

    if( !lease )
        return std::nullopt;

    // wxLog keeps a thread's records for the main loop unless the thread has its own target
    thread_local const bool logTarget = ( wxLog::SetThreadActiveTarget( KOPENAPI_JOURNAL::ThreadTarget() ), true );
    (void) logTarget;

    MODEL_THREAD::SCOPE                reader( MODEL_THREAD::KIND::READER );
    MODEL_THREAD::REFUSE_WRITES        readOnly;
    KOPENAPI_JOURNAL::SCOPED_OPERATION operation( "api:" + aName );
    KOPENAPI_CONTEXT                   ctxCopy = ctx;

    try
    {
        std::optional<KOPENAPI_METHOD> method = KOPENAPI_REGISTRY::Get().Find( aName );

        if( !method )
            return std::nullopt;

        KOPENAPI_RESULT result = method->handler( ctxCopy, aArgs );
        leaseReadsServed++;

        if( result.status < 400 )
        {
            addPoints( result.body, 0 );
        }
        else
        {
            const std::string message = result.body.contains( "error" ) && result.body["error"].is_object()
                                                ? result.body["error"].value( "message", std::string() )
                                                : std::string();
            wxLogWarning( "%s", wxString::FromUTF8( "api:" + aName + " failed (" + std::to_string( result.status )
                                                    + "): " + message ) );
        }

        return result;
    }
    catch( const KOPENAPI_RETRY_LATER& )
    {
        leaseReadsToOwner++;
        return std::nullopt;
    }
    catch( const MODEL_THREAD::WRITE_REFUSED& )
    {
        leaseReadsToOwner++;
        return std::nullopt;   // a read that would write: the owner answers it
    }
    catch( const std::exception& e )
    {
        return KOPENAPI_RESULT::Error( 500, e.what() );
    }
}


nlohmann::json KICAD_OPENAPI_SERVICE::IMPL::gateJson() const
{
    if( !gate )
        return nullptr;

    const KOPENAPI_DOC_GATE::STATS st = gate->Stats();
    auto ms = []( std::chrono::microseconds d ) { return std::round( d.count() / 100.0 ) / 10; };

    return { { "version", gate->Version() },
             { "lease_reads", leaseReadsServed.load() },
             { "lease_reads_to_owner", leaseReadsToOwner.load() },
             { "leases_granted", st.leasesGranted },
             { "leases_active", st.leasesActive },
             { "writes", st.writes },
             { "revoked", st.revoked },
             { "stuck", st.stuck },
             { "max_lease_hold_ms", ms( st.maxLeaseHold ) },
             { "mean_lease_hold_ms", st.leasesGranted ? ms( st.totalLeaseHold / (long long) st.leasesGranted ) : 0.0 },
             { "max_write_wait_ms", ms( st.maxWriteWait ) },
             { "write_violations", MODEL_THREAD::WriteViolations() },
             { "writes_refused", MODEL_THREAD::WritesRefused() } };
}


std::function<KOPENAPI_RESULT()>
KICAD_OPENAPI_SERVICE::IMPL::mainCall( const std::string& aName, const nlohmann::json& aArgs,
                                       std::shared_ptr<std::chrono::steady_clock::time_point> aStarted ) const
{
    KOPENAPI_CONTEXT ctxCopy = ctx;
    KOPENAPI_HANDLER handler = KOPENAPI_REGISTRY::Get().Find( aName )->handler;
    nlohmann::json   args = aArgs;

    const std::string operation = "api:" + aName;
    const IMPL*       self = this;

    return [ctxCopy, handler, args, operation, started = std::move( aStarted ), self]() mutable
                      {
                          const auto now = std::chrono::steady_clock::now();

                          if( started )
                              *started = now;

                          // Editing calls: the documents are marked before, the call is logged
                          const std::string name = operation.substr( 4 );
                          const bool        editing = KOPENAPI_REGISTRY::Get().IsEditing( name );

                          // queued during a long call and run at one of its safe points
                          if( editing && KopenapiJobYielding() )
                          {
                              const std::string job = KOPENAPI_JOBS::Get().Busy();
                              std::string       owner;

                              {
                                  std::lock_guard<std::mutex> lock( self->traceMutex );
                                  owner = self->busyMethod;
                              }

                              return self->refusedByClaim( !job.empty()
                                                                   ? "a job owns the documents: job_list, job_cancel"
                                                                   : owner + " is running and owns the documents (reading calls "
                                                                             "answer meanwhile); retry once it ends" );
                          }

                          std::string busyBefore;

                          {
                              std::lock_guard<std::mutex> lock( self->traceMutex );
                              busyBefore = self->busyMethod;
                              self->busyMethod = name;
                              self->busySince = now;
                          }

                          if( busyBefore.empty() )
                              KopenapiCallBegin( name );

                          s_apiCalls++;

                          struct BUSY_END
                          {
                              const IMPL* impl;
                              std::string before;
                              ~BUSY_END()
                              {
                                  s_apiCalls--;

                                  if( before.empty() )
                                      KopenapiCallEnd();

                                  std::lock_guard<std::mutex> lock( impl->traceMutex );
                                  impl->busyMethod = before;
                              }
                          } busyEnd{ self, busyBefore };

                          // headless: the call holds the documents exclusively (readers under leases wait)
                          MODEL_THREAD::WRITE_SECTION writeSection;

                          // Log records during the call are attributed to this method
                          KOPENAPI_JOURNAL::SCOPED_OPERATION scope( operation );

                          // A method not marked editing must not change a document: in the GUI that
                          // is checked on the undo stacks, so a method that edits without the mark
                          // (no history step, no rollback) is reported instead of going unnoticed
                          const std::string undoBefore = editing ? std::string()
                                                                 : KOPENAPI_HISTORY::Get().UndoFingerprint( ctxCopy );

                          if( editing )
                              KOPENAPI_HISTORY::Get().BeforeEdit( ctxCopy, name, args );

                          const uint64_t  logBefore = KOPENAPI_JOURNAL::Summary().value( "last_seq", uint64_t( 0 ) );
                          KOPENAPI_RESULT result;

                          try
                          {
                              result = handler( ctxCopy, args );
                          }
                          catch( const KOPENAPI_RETRY_LATER& e )
                          {
                              result = e.Result();
                          }

                          // a failure carries what KiCad logged during the call: the cause it would
                          // have shown in a dialog
                          if( result.status >= 400 && result.body.contains( "error" ) && result.body["error"].is_object() )
                          {
                              nlohmann::json log = nlohmann::json::array();

                              for( const nlohmann::json& entry : KOPENAPI_JOURNAL::Entries( logBefore, 20, "warning" ) )
                              {
                                  if( entry.value( "operation", std::string() ) == operation )
                                      log.push_back( entry.value( "message", std::string() ) );
                              }

                              if( !log.empty() )
                                  result.body["error"]["log"] = log;
                          }

                          if( result.status < 400 )
                              addPoints( result.body, 0 );

                          // the change feed follows a document opened or replaced by the call
                          KOPENAPI_REGISTRY::Get().Documents( ctxCopy );

                          if( editing )
                              KOPENAPI_HISTORY::Get().AfterEdit( ctxCopy, result );
                          else if( const std::string undoAfter = KOPENAPI_HISTORY::Get().UndoFingerprint( ctxCopy );
                                   undoAfter != undoBefore
                                   && undoAfter.substr( 0, undoAfter.find( '|' ) )
                                              == undoBefore.substr( 0, undoBefore.find( '|' ) ) )
                              wxLogWarning( "api %s changed a document but is not marked editing "
                                            "(KOPENAPI_MARK_EDITING): it is not in edit_log and cannot be "
                                            "rolled back by step", name );

                          // Failed operations are journaled too (4xx warning, 5xx error)
                          if( result.status >= 400 )
                          {
                              const std::string message = result.body.contains( "error" )
                                      ? result.body["error"].value( "message", std::string() )
                                      : std::string();
                              const wxString text = wxString::FromUTF8(
                                      operation + " failed (" + std::to_string( result.status )
                                      + "): " + message );

                              if( result.status >= 500 )
                                  wxLogError( "%s", text );
                              else
                                  wxLogWarning( "%s", text );
                          }

                          return result;
                      };
}


void KICAD_OPENAPI_SERVICE::IMPL::traceCall( const std::string& aName, const nlohmann::json& aArgs,
                                             const KOPENAPI_RESULT& aOutcome, std::chrono::steady_clock::time_point aReceived,
                                             std::chrono::steady_clock::time_point aStarted ) const
{
    const auto finished = std::chrono::steady_clock::now();
    auto       ms = []( auto d ) { return std::round( std::chrono::duration<double, std::milli>( d ).count() * 10 ) / 10; };
    TRACE      t;
    t.method = aName;
    t.args = aArgs.dump();
    t.argBytes = t.args.size();

    if( t.args.size() > 2048 )
        t.args = t.args.substr( 0, 2048 ) + "...";

    if( aOutcome.status >= 400 && aOutcome.body.contains( "error" ) )
        t.error = aOutcome.body["error"].value( "message", std::string() );
    t.queuedMs = ms( aStarted - aReceived );
    t.runMs = ms( finished - aStarted );
    t.status = aOutcome.status;
    t.at = wxDateTime::Now().FormatISOCombined( ' ' ).ToStdString();

    {
        std::lock_guard<std::mutex> lock( traceMutex );
        t.seq = ++traceSeq;
        const std::string line = t.Json().dump();

        traces.push_back( t );
        traceSizes.push_back( line.size() + 1 );
        traceBytes += line.size() + 1;

        while( traceBytes > traceLimit && traces.size() > 1 )
        {
            traceBytes -= traceSizes.front();
            traceSizes.pop_front();
            traces.pop_front();
        }

        if( traceStream.is_open() )
            traceStream << line << '\n' << std::flush;
    }

    if( traceToJournal )
        wxLogMessage( "api #%lld %s %zu B: queued %.1f ms, ran %.1f ms -> %d", t.seq, aName, t.argBytes, t.queuedMs,
                      t.runMs, t.status );
}


nlohmann::json KICAD_OPENAPI_SERVICE::IMPL::traceJson( const nlohmann::json& aArgs ) const
{
    // configuration: on / off, buffer size, file sink
    if( aArgs.contains( "enabled" ) || aArgs.contains( "buffer_bytes" ) || aArgs.contains( "file" ) )
    {
        std::lock_guard<std::mutex> lock( traceMutex );
        const bool on = aArgs.value( "enabled", true );

        traceOn = on;

        if( !on )
        {
            traces.clear();
            traces.shrink_to_fit();
            traceSizes.clear();
            traceSizes.shrink_to_fit();
            traceBytes = 0;
            traceStream.close();
            traceFile.clear();
            return { { "enabled", false } };
        }

        if( aArgs.contains( "buffer_bytes" ) )
            traceLimit = std::clamp<size_t>( aArgs["buffer_bytes"].get<size_t>(), 4096, size_t( 1 ) << 30 );

        if( aArgs.contains( "file" ) )
        {
            traceStream.close();
            traceFile = aArgs["file"].is_string() ? aArgs["file"].get<std::string>() : std::string();

            if( !traceFile.empty() )
            {
                traceStream.open( traceFile, std::ios::app );

                if( !traceStream )
                {
                    const std::string bad = traceFile;
                    traceFile.clear();
                    return { { "error", "cannot open " + bad } };
                }
            }
        }

        while( traceBytes > traceLimit && traces.size() > 1 )
        {
            traceBytes -= traceSizes.front();
            traceSizes.pop_front();
            traces.pop_front();
        }
    }

    if( !traceOn )
        return { { "enabled", false }, { "busy", busyJson() },
                 { "hint", "api_trace {enabled: true, buffer_bytes?, file?} starts tracing" } };

    const size_t   limit = std::max( aArgs.value( "limit", 50 ), 1 );
    const std::string only = aArgs.value( "method", std::string() );
    const double   slowerMs = aArgs.value( "slower_ms", 0.0 );
    nlohmann::json calls = nlohmann::json::array();

    std::lock_guard<std::mutex> lock( traceMutex );

    for( auto it = traces.rbegin(); it != traces.rend() && calls.size() < limit; ++it )
    {
        if( ( !only.empty() && !KopenapiGlob( only, it->method ) ) || it->runMs + it->queuedMs < slowerMs )
            continue;

        nlohmann::json row = it->Json();

        if( !aArgs.value( "with_args", false ) )
            row.erase( "args" );

        calls.push_back( row );
    }

    return { { "enabled", true }, { "calls", calls }, { "total", traceSeq }, { "busy", busyLocked() }, { "kept", traces.size() },
             { "buffer_bytes", traceLimit }, { "used_bytes", traceBytes },
             { "file", traceFile.empty() ? nlohmann::json() : nlohmann::json( traceFile ) } };
}


void KICAD_OPENAPI_SERVICE::IMPL::handleMcp( const httplib::Request& aReq,
                                             httplib::Response&      aRes ) const
{
    KOPENAPI_MCP_CONTEXT mcpCtx;
    mcpCtx.headless = ctx.headless;
    mcpCtx.version = KOPENAPI_VERSION;
    ensureKifaces();   // search must see kiface methods
    const std::string caller = aReq.get_header_value( "X-Kicad-Session" );

    mcpCtx.invoke = [this, caller]( const std::string& aName, const nlohmann::json& aArgs )
    {
        return invokeParsed( aName, aArgs, caller );
    };

    const nlohmann::json msg = nlohmann::json::parse( aReq.body, nullptr, false );

    if( msg.is_discarded() )
    {
        aRes.status = 400;
        aRes.set_content( toJson( { { "jsonrpc", "2.0" },
                                    { "id", nullptr },
                                    { "error", { { "code", -32700 }, { "message", "parse error" } } } } ),
                          "application/json" );
        return;
    }

    nlohmann::json response;

    if( msg.is_array() )   // JSON-RPC batch
    {
        response = nlohmann::json::array();

        for( const nlohmann::json& item : msg )
        {
            if( std::optional<nlohmann::json> r = KopenapiMcpHandle( mcpCtx, item ) )
                response.push_back( *r );
        }

        if( response.empty() )
            response = nullptr;
    }
    else if( std::optional<nlohmann::json> r = KopenapiMcpHandle( mcpCtx, msg ) )
    {
        response = *r;
    }

    if( response.is_null() )
    {
        aRes.status = 202;   // notifications only
        return;
    }

    aRes.status = 200;
    aRes.set_content( toJson( response ), "application/json" );
}


/// @brief Open event streams, each holding an HTTP worker while open: the worker pool has room for
/// this many beyond the calls' own, and one more is refused
static constexpr int    MAX_EVENT_STREAMS = 64;
static std::atomic<int> s_eventStreams{ 0 };


/// @brief GET /api/v1/events?topic=…: Server-Sent Events.  The worker sleeps until the topic changes
/// (KopenapiEventNotify), then polls at most ~30 times a second, so a burst of changes reaches the
/// client as its latest state; idle it polls once a second for the topic's deadlines, and a comment
/// every 15 s keeps idle proxies and clients from timing out
static void serveEvents( const httplib::Request& aReq, httplib::Response& aRes,
                         const std::shared_ptr<std::atomic<bool>>& aAlive )
{
    const std::string                                 name = aReq.get_param_value( "topic" );
    const std::map<std::string, KOPENAPI_EVENT_TOPIC> topics = KOPENAPI_REGISTRY::Get().EventTopics();
    auto                                              it = topics.find( name );

    if( it == topics.end() )
    {
        std::string known;

        for( const auto& [topic, unused] : topics )
            known += ( known.empty() ? "" : ", " ) + topic;

        reply( aRes, KOPENAPI_RESULT::Error( 404, "topic: one of " + known ) );
        return;
    }

    nlohmann::json args = nlohmann::json::object();

    for( const auto& [key, value] : aReq.params )
    {
        if( key == "topic" )
            continue;

        nlohmann::json v = nlohmann::json::parse( value, nullptr, false );
        args[key] = v.is_discarded() ? nlohmann::json( value ) : v;
    }

    if( s_eventStreams.load() >= MAX_EVENT_STREAMS )
    {
        reply( aRes, KOPENAPI_RESULT::Error( 503, std::to_string( MAX_EVENT_STREAMS )
                                                          + " event streams are open; close one" ) );
        return;
    }

    KOPENAPI_EVENT_POLL poll;
    KOPENAPI_RESULT     opened = it->second.open( args, poll );

    if( opened.status != 200 || !poll )
    {
        reply( aRes, opened.status != 200 ? opened : KOPENAPI_RESULT::Error( 500, "the topic opened no stream" ) );
        return;
    }

    using CLOCK = std::chrono::steady_clock;

    struct STREAM
    {
        std::string                 topic;
        KOPENAPI_EVENT_POLL         poll;
        std::vector<KOPENAPI_EVENT> events;
        uint64_t                    seen = 0;
        CLOCK::time_point           next;
        CLOCK::time_point           wrote = CLOCK::now();
        bool                        first = true;

        STREAM() { s_eventStreams++; }
        ~STREAM() { s_eventStreams--; }
    };

    auto stream = std::make_shared<STREAM>();
    stream->topic = name;
    stream->poll = std::move( poll );

    aRes.set_header( "Cache-Control", "no-cache" );
    aRes.set_chunked_content_provider(
            "text/event-stream",
            [stream, aAlive]( size_t, httplib::DataSink& aSink ) -> bool
            {
                STREAM& st = *stream;
                auto    send = [&]( const std::string& aText )
                {
                    st.wrote = CLOCK::now();
                    return aSink.write( aText.data(), aText.size() );
                };

                if( st.first )
                {
                    st.first = false;

                    if( !send( ": open\n\n" ) )
                        return false;
                }
                else
                {
                    std::this_thread::sleep_until( st.next );
                    KopenapiEventWait( st.topic, st.seen,
                                       std::min( st.wrote + std::chrono::seconds( 15 ),
                                                 CLOCK::now() + std::chrono::seconds( 1 ) ) );
                }

                if( !aAlive->load() || !aSink.is_writable() )
                {
                    aSink.done();
                    return true;
                }

                bool more = false;
                st.events.clear();
                st.seen = KopenapiEventSeq( st.topic );

                try
                {
                    more = st.poll( st.events );
                }
                catch( const std::exception& e )
                {
                    st.events.push_back( { "error", { { "error", e.what() } } } );
                }

                for( const KOPENAPI_EVENT& e : st.events )
                {
                    if( !send( "event: " + e.name + "\ndata: " + toJson( e.data ) + "\n\n" ) )
                        return false;
                }

                st.next = CLOCK::now() + std::chrono::milliseconds( 33 );

                if( !more )
                    aSink.done();
                else if( CLOCK::now() - st.wrote > std::chrono::seconds( 15 ) && !send( ": keep-alive\n\n" ) )
                    return false;

                return true;
            } );
}



namespace
{

/// @brief The running call's progress, a job's or a plain call's: 'progress' events when it changes,
/// 'idle' when none reports any
KOPENAPI_RESULT openProgressEvents( const nlohmann::json&, KOPENAPI_EVENT_POLL& aPoll )
{
    auto last = std::make_shared<std::optional<nlohmann::json>>();

    aPoll = [last]( std::vector<KOPENAPI_EVENT>& aOut )
    {
        nlohmann::json now = KopenapiRunningProgress();
        nlohmann::json key = now;

        if( key.is_object() )
        {
            key.erase( "rev" );

            for( const char* volatileKey : { "phase_eta_s", "eta_s", "elapsed_s" } )
                key["progress"].erase( volatileKey );
        }

        if( !*last || key != **last )
        {
            *last = std::move( key );
            aOut.push_back( { now.is_null() ? "idle" : "progress", now.is_null() ? nlohmann::json::object() : now } );
        }

        return true;
    };

    return KOPENAPI_RESULT::Ok( nlohmann::json::object() );
}

} // namespace


KOPENAPI_REGISTER_EVENT_TOPIC( "progress",
                               "the progress of the running call, a job or a plain call (pcb_route, pcb_drc, "
                               "pcb_zones_fill, pcb_stitch): 'progress' events {method, job_id (jobs), progress} "
                               "when it changes, 'idle' when no call reports progress; the stream stays open",
                               openProgressEvents );


void KICAD_OPENAPI_SERVICE::IMPL::registerRoutes()
{
    // Exclusive listener (per OS, see platform layer): httplib's default SO_REUSEPORT would
    // let two KiCad processes silently share one port
    server->set_socket_options(
            []( socket_t aSock )
            {
                kopenapi::platform::ConfigureListenSocket( static_cast<std::uintptr_t>( aSock ) );
            } );

    server->Get( "/",
                 []( const httplib::Request&, httplib::Response& res )
                 {
                     // swagger-ui (assets from CDN) over the generated manifest
                     res.set_content(
                             "<!DOCTYPE html><html><head><title>kicadopenapi</title>"
                             "<meta charset='utf-8'/>"
                             "<link rel='stylesheet' "
                             "href='https://unpkg.com/swagger-ui-dist@5/swagger-ui.css'>"
                             "</head><body><div id='swagger-ui'></div>"
                             "<script "
                             "src='https://unpkg.com/swagger-ui-dist@5/swagger-ui-bundle.js'>"
                             "</script><script>window.onload=()=>{SwaggerUIBundle({"
                             "url:'/api/v1/openapi.json',dom_id:'#swagger-ui'})};</script>"
                             "</body></html>",
                             "text/html" );
                 } );

    server->Get( "/api/v1/status",
                 [this]( const httplib::Request&, httplib::Response& res )
                 {
                     reply( res, liveStatus() );
                 } );

    server->Get( "/api/v1/main_loop",
                 [this]( const httplib::Request& req, httplib::Response& res )
                 {
                     if( !mainProbe )
                         reply( res, KOPENAPI_RESULT::Error( 404, "start the GUI with KICAD_OPENAPI_MAIN_PROBE=1" ) );
                     else
                         reply( res, KOPENAPI_RESULT::Ok( mainProbe->Json( req.has_param( "reset" ) ) ) );
                 } );

    server->Get( "/api/v1/progress",
                 [this]( const httplib::Request&, httplib::Response& res )
                 {
                     reply( res, KOPENAPI_RESULT::Ok( progressJson() ) );
                 } );

    server->Get( "/api/v1/openapi.json",
                 [this]( const httplib::Request&, httplib::Response& res )
                 {
                     reply( res, KOPENAPI_RESULT::Ok( openApiJson() ) );
                 } );

    server->Get( "/api/v1/events",
                 [this]( const httplib::Request& req, httplib::Response& res )
                 {
                     serveEvents( req, res, alive );
                 } );

    server->Post( "/mcp",
                  [this]( const httplib::Request& req, httplib::Response& res )
                  {
                      handleMcp( req, res );
                  } );

    if( shutdownHandler )
    {
        // Registered before the generic method route so it takes precedence
        server->Post( "/api/v1/shutdown",
                      [this]( const httplib::Request&, httplib::Response& res )
                      {
                          reply( res, KOPENAPI_RESULT::Ok( { { "stopping", true } } ) );
                          shutdownHandler();
                      } );
    }

    server->Post( R"(/api/v1/([a-z0-9_]+))",
                  [this]( const httplib::Request& req, httplib::Response& res )
                  {
                      reply( res, invoke( req.matches[1], req.body, req.get_header_value( "X-Kicad-Session" ) ) );
                  } );

    server->set_error_handler(
            []( const httplib::Request&, httplib::Response& res )
            {
                // Keep error bodies JSON (unknown paths, wrong verbs)
                if( res.body.empty() )
                {
                    res.set_content( toJson( KOPENAPI_RESULT::Error(
                                                     res.status, httplib::status_message( res.status ) )
                                                     .body ),
                                     "application/json" );
                }
            } );
}


KICAD_OPENAPI_SERVICE::KICAD_OPENAPI_SERVICE( KIWAY* aKiway, bool aHeadless ) :
        m_impl( std::make_unique<IMPL>() )
{
    m_impl->ctx.kiway = aKiway;
    m_impl->ctx.headless = aHeadless;
}


KICAD_OPENAPI_SERVICE::~KICAD_OPENAPI_SERVICE()
{
    Stop();
}


void KICAD_OPENAPI_SERVICE::PreloadKifaces()
{
    if( !wxTheApp || !m_impl->ctx.kiway || m_impl->kifacesLoaded.load() )
        return;

    // ensureKifaces() on a worker: it marshals to the main loop and returns when the kifaces
    // are in (or the service stops); requests arriving meanwhile queue behind it
    if( !m_impl->preloadThread.joinable() )
        m_impl->preloadThread = std::thread( [impl = m_impl.get()]() { impl->ensureKifaces(); } );
}


void KICAD_OPENAPI_SERVICE::RunOnOwner( const std::function<void()>& aFn )
{
    if( !m_impl->strand || m_impl->strand->OnDispatchThread() )
    {
        aFn();
        return;
    }

    auto done = std::make_shared<std::promise<void>>();
    std::future<void> finished = done->get_future();

    m_impl->strand->QueueCall( std::make_shared<std::function<void()>>(
            [&aFn, done]()
            {
                try
                {
                    MODEL_THREAD::WRITE_SECTION writeSection;
                    aFn();
                    done->set_value();
                }
                catch( ... )
                {
                    done->set_exception( std::current_exception() );
                }
            } ) );

    finished.get();
}


void KICAD_OPENAPI_SERVICE::SetMainLoopWaker( std::function<void()> aWaker )
{
    m_impl->waker = std::move( aWaker );
}


void KICAD_OPENAPI_SERVICE::SetShutdownHandler( std::function<void()> aHandler )
{
    m_impl->shutdownHandler = std::move( aHandler );
}


namespace
{
std::atomic<int> s_currentPort{ 0 };
}


int KICAD_OPENAPI_SERVICE::CurrentPort()
{
    return s_currentPort.load();
}


int KICAD_OPENAPI_SERVICE::DefaultPort()
{
    if( const char* env = std::getenv( "KICAD_OPENAPI_PORT" ) )
    {
        int port = std::atoi( env );

        if( port > 0 && port < 65536 )
            return port;
    }

    return DEFAULT_PORT;
}


bool KICAD_OPENAPI_SERVICE::Start( int aPort )
{
    KOPENAPI_MEMORY::StartAutoTrim();

    for( const std::string& name : KOPENAPI_REGISTRY::Get().Duplicates() )
        wxLogError( "kicadopenapi: method '%s' is registered twice; the first registration is kept", name );

    if( Running() )
        return true;

    const int basePort = aPort < 0 ? DefaultPort() : aPort;

    // app_restart: the process we replace still holds the port until it exits
    if( const char* env = std::getenv( "KICAD_OPENAPI_WAIT_PID" ) )
    {
        const long pid = std::atol( env );

        for( int i = 0; i < 300 && kopenapi::platform::ProcessAlive( pid ); ++i )
            std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

        wxUnsetEnv( wxS( "KICAD_OPENAPI_WAIT_PID" ) );
    }

    m_impl->appName = wxTheApp ? wxTheApp->GetAppName().ToStdString() : std::string( "kicad" );
    m_impl->port = 0;

    auto poolReady = std::make_shared<std::promise<void>>();
    std::future<void> poolBuilt = poolReady->get_future();

    for( int i = 0; i < PORT_PROBE_COUNT; ++i )
    {
        // A fresh server per attempt: httplib decommissions a server after a failed bind
        m_impl->server = std::make_unique<httplib::Server>();
        m_impl->server->new_task_queue = [poolReady]
        {
            // standing threads for them: the pool adds one only when none is idle as a connection
            // comes, so a burst of streams could otherwise wait behind each other
            auto* pool = new httplib::ThreadPool( CPPHTTPLIB_THREAD_POOL_COUNT + MAX_EVENT_STREAMS,
                                                  CPPHTTPLIB_THREAD_POOL_MAX_COUNT + MAX_EVENT_STREAMS );
            poolReady->set_value();
            return pool;
        };
        m_impl->registerRoutes();

        if( m_impl->server->bind_to_port( KOPENAPI_HOST_ADDR, basePort + i ) )
        {
            m_impl->port = basePort + i;
            s_currentPort = m_impl->port;
            break;
        }
    }

    if( !m_impl->port )
    {
        std::fprintf( stderr, "kicadopenapi: failed to bind 127.0.0.1:%d..%d\n", basePort,
                      basePort + PORT_PROBE_COUNT - 1 );
        m_impl->server.reset();
        return false;
    }

    m_impl->alive->store( true );
    MAIN_GATE_HANDLER::Get();   // created on the main thread
    MAIN_GATE_HANDLER::Jobs();

    // headless: calls and jobs run on the owner strand; GUI: on the main thread, which lends the
    // documents to readers while it computes.  KICAD_OPENAPI_MAIN_DISPATCH keeps the main loop
    // without leases (A/B comparison)
    if( !std::getenv( "KICAD_OPENAPI_MAIN_DISPATCH" ) )
    {
        KOPENAPI_DOC_GATE::OPTIONS options;
        options.leaseBudget = LEASE_BUDGET;
        m_impl->gate = std::make_unique<KOPENAPI_DOC_GATE>( options );
        m_impl->gate->SetViolationSink(
                []( const KOPENAPI_LEASE_VIOLATION& aViolation )
                {
                    wxLogWarning( "model gate: read lease of %s %s (held %lld ms, budget %lld ms)",
                                  aViolation.holder,
                                  aViolation.kind == KOPENAPI_LEASE_VIOLATION::KIND::REVOKED ? "revoked" : "stuck",
                                  (long long) std::chrono::duration_cast<std::chrono::milliseconds>( aViolation.held ).count(),
                                  (long long) std::chrono::duration_cast<std::chrono::milliseconds>( aViolation.budget ).count() );
                } );

        KOPENAPI_DOC_GATE* gate = m_impl->gate.get();
        MODEL_THREAD::SetWriteHooks( [gate]() { tlOwnerWrite.emplace( gate->BeginWrite() ); },
                                     []()
                                     {
                                         tlOwnerWrite->Commit();
                                         tlOwnerWrite.reset();
                                     } );

        if( m_impl->ctx.headless )
        {
            m_impl->leaseWait = LEASE_WAIT;
            m_impl->strand = std::make_unique<OWNER_STRAND_DISPATCH>();
            s_dispatch = m_impl->strand.get();
        }
        else
        {
            m_impl->mainOwner.emplace( MODEL_THREAD::KIND::OWNER );
            m_impl->mainWrite.emplace();
        }
    }

    if( !m_impl->ctx.headless && std::getenv( "KICAD_OPENAPI_MAIN_PROBE" ) )
        m_impl->mainProbe = std::make_unique<MAIN_LOOP_PROBE>();

    if( std::getenv( "KICAD_OPENAPI_TRACE" ) )
    {
        m_impl->traceOn = true;
        m_impl->traceToJournal = true;
    }

    // Startup ends when the main loop first runs; later records not caused by an API call
    // are attributed to "background"
    if( wxTheApp )
        wxTheApp->CallAfter( []() { KOPENAPI_JOURNAL::SetOperation( "background" ); } );

    // The socket is already bound, so clients can connect as soon as Start() returns;
    // listen_after_bind() blocks in this thread until Stop().
    m_impl->thread = std::thread( [srv = m_impl->server.get()]() { srv->listen_after_bind(); } );

    // The pool's threads exist before Start() returns, so they are never created while the
    // kifaces load (see upstream-issues A50: GCC 13 ASan allocates thread contexts and a loaded
    // library's globals from one unlocked allocator)
    poolBuilt.wait_for( std::chrono::seconds( 10 ) );

    // After app_restart clients find this process only once the documents it reopens are open
    // (or the host reports its start-up opening finished): their next call (e.g. a 3D view of
    // the board) must not race the loading.  The variable holds a JSON array of paths or one path.
    std::vector<std::string> reopen;
    bool                     restarted = false;

    if( const char* env = std::getenv( "KICAD_OPENAPI_REOPEN" ) )
    {
        restarted = true;
        nlohmann::json list = nlohmann::json::parse( env, nullptr, false );

        if( list.is_array() )
        {
            for( const nlohmann::json& path : list )
            {
                if( path.is_string() )
                    reopen.push_back( path.get<std::string>() );
            }
        }
        else if( *env )
        {
            reopen.push_back( env );
        }

        wxUnsetEnv( wxS( "KICAD_OPENAPI_REOPEN" ) );
    }

    if( !restarted )
    {
        m_impl->writeDiscoveryFile();
    }
    else
    {
        m_impl->publishThread = std::thread(
                [impl = m_impl.get(), reopen]()
                {
                    namespace fs = std::filesystem;
                    std::vector<fs::path> want;

                    // a path that no longer exists is reported by the host and never waited for
                    for( const std::string& path : reopen )
                    {
                        std::error_code ec;

                        if( fs::exists( fs::path( path ), ec ) )
                            want.push_back( fs::weakly_canonical( fs::path( path ), ec ) );
                    }

                    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds( 120 );

                    while( impl->alive->load() && !impl->startupOpened.load()
                           && std::chrono::steady_clock::now() < deadline )
                    {
                        if( !want.empty() )
                        {
                            KOPENAPI_CONTEXT ctx = impl->ctx;
                            KOPENAPI_RESULT  docs = runInMain( impl->alive, impl->waker,
                                                               [ctx]() mutable
                                                               {
                                                                   return KOPENAPI_RESULT::Ok(
                                                                           KOPENAPI_REGISTRY::Get().Documents( ctx ) );
                                                               } );
                            size_t open = 0;

                            for( const fs::path& path : want )
                            {
                                bool found = false;

                                for( const nlohmann::json& d : docs.status == 200 ? docs.body : nlohmann::json::array() )
                                {
                                    for( const char* key : { "path", "project" } )
                                    {
                                        std::error_code e;

                                        if( d.contains( key )
                                            && fs::weakly_canonical( fs::path( d.value( key, std::string() ) ), e ) == path )
                                        {
                                            found = true;
                                        }
                                    }
                                }

                                open += found ? 1 : 0;
                            }

                            if( open == want.size() )
                                break;
                        }

                        std::this_thread::sleep_for( std::chrono::milliseconds( 200 ) );
                    }

                    if( impl->alive->load() )
                        impl->writeDiscoveryFile();
                } );
    }

    // A process without libraries is a silent failure for an agent: say so in the journal
    KopenapiCheckGlobalLibraryTables();

    // a throttled background KiCad answers an agent's call in tens of seconds on a loaded host
    kopenapi::platform::KeepProcessResponsive();

    std::fprintf( stderr, "kicadopenapi listening at http://%s:%d\n", KOPENAPI_HOST_ADDR,
                  m_impl->port );
    return true;
}


void KICAD_OPENAPI_SERVICE::StartupOpened()
{
    m_impl->startupOpened.store( true );
}


bool KICAD_OPENAPI_SERVICE::InMainThreadCall()
{
    return s_apiCalls.load() > 0;
}


void KICAD_OPENAPI_SERVICE::Stop()
{
    KOPENAPI_MEMORY::StopAutoTrim();
    KOPENAPI_REMOTE::Shutdown();

    if( !m_impl->server )
        return;

    KopenapiRecorderShutdown();   // a recording file must be finished while its window exists

    // Unblock workers waiting on main-thread calls first, then stop and join the listener
    // (which joins the worker pool).
    m_impl->alive->store( false );
    KopenapiEventWakeAll();
    m_impl->server->stop();

    if( m_impl->preloadThread.joinable() )
        m_impl->preloadThread.join();

    if( m_impl->publishThread.joinable() )
        m_impl->publishThread.join();

    if( m_impl->thread.joinable() )
        m_impl->thread.join();

    m_impl->server.reset();
    m_impl->mainProbe.reset();
    m_impl->port = 0;
    s_currentPort = 0;

    // Headless documents live in the kifaces; drop them while the project/settings still exist
    RunOnOwner( []() { KOPENAPI_REGISTRY::Get().ReleaseDocuments(); } );

    if( m_impl->strand )
    {
        m_impl->strand->Stop();
        s_dispatch = &s_mainLoopDispatch;
        m_impl->strand.reset();
    }

    m_impl->mainWrite.reset();
    m_impl->mainOwner.reset();

    if( m_impl->gate )
    {
        MODEL_THREAD::SetWriteHooks( nullptr, nullptr );
        m_impl->gate.reset();
    }

    if( !m_impl->discoveryFile.empty() )
    {
        std::error_code ec;
        fs::remove( m_impl->discoveryFile, ec );
        m_impl->discoveryFile.clear();
    }
}


bool KICAD_OPENAPI_SERVICE::NoModalUi()
{
    static const bool agent = std::getenv( "KICAD_OPENAPI_AGENT" ) != nullptr;
    return CurrentPort() > 0 && ( agent || s_apiCalls.load() > 0 );
}


void KICAD_OPENAPI_SERVICE::SetOverrideLock( bool aOverride )
{
    s_overrideLock = aOverride;
}


bool KICAD_OPENAPI_SERVICE::OverrideLock()
{
    return s_overrideLock;
}


KOPENAPI_RESULT KICAD_OPENAPI_SERVICE::Invoke( const std::string& aName, const nlohmann::json& aArgs )
{
    return m_impl->invokeParsed( aName, aArgs.is_null() ? nlohmann::json::object() : aArgs );
}


bool KICAD_OPENAPI_SERVICE::Running() const
{
    return m_impl->server && m_impl->alive->load();
}


int KICAD_OPENAPI_SERVICE::Port() const
{
    return Running() ? m_impl->port : 0;
}
