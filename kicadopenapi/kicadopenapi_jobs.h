/// @file kicadopenapi_jobs.h
/// @brief Long API calls as jobs.  job_start answers at once with a job_id; the call runs on the
/// main thread like any other, and at its safe points (KopenapiJobStep: the documents consistent,
/// between two connections of the router) it reports progress and gives way: the UI runs (paint,
/// timers, view navigation; other input is dropped, Esc stops the job, closing a window is
/// refused and stops it) and so do queued API calls that only read.  Calls that edit are refused
/// (409) while a job is queued or running.  job_get, job_cancel and job_list answer on the HTTP
/// thread.  One job at a time.
#ifndef KICADOPENAPI_JOBS_H
#define KICADOPENAPI_JOBS_H

#include <kicommon.h>
#include <kicadopenapi_registry.h>

#include <chrono>
#include <functional>
#include <string>


/// @brief At a safe point of a call that runs as a job: records the progress (phase, aDone of
/// aTotal, the item at work, counts so far) and lets the UI and reading API calls run.  False once
/// the job is asked to stop: the caller finishes with what it has done.  Outside a job: true
/// (a plain call's safe point, KopenapiCallBegin).
KICOMMON_API bool KopenapiJobStep( const std::string& aPhase, int aDone, int aTotal,
                                   const std::string& aItem = std::string(),
                                   const nlohmann::json& aCounts = nlohmann::json() );

/// @brief Progress inside work whose worker threads may be running (DRC tests, zone fills): records
/// it like KopenapiJobStep and keeps the windows painted, but runs no API calls and never pauses.
/// False once the job is asked to stop.  Any thread records; the UI only on the main thread
KICOMMON_API bool KopenapiJobProgress( const std::string& aPhase, int aDone, int aTotal,
                                       const std::string& aItem = std::string(),
                                       const nlohmann::json& aCounts = nlohmann::json() );

/// @brief The gate: an outermost API call starts / ends on the main thread.  KopenapiJobStep outside
/// a job records that call's progress and runs the API calls queued meanwhile (edits get 409); no
/// user input, no stop
KICOMMON_API void KopenapiCallBegin( const std::string& aMethod );
KICOMMON_API void KopenapiCallEnd();

/// @brief The progress of the running job, else of the running call ({phase, done, total, item,
/// counts, phase_eta_s, eta_s, elapsed_s}); null when it has reported none.  eta_s: from
/// counts.fraction (the share of the whole call done) when given, else the phase's pace.  Any thread
KICOMMON_API nlohmann::json KopenapiCallProgress();

/// @brief {method, rev, progress, job_id + paused for a job} of the call reporting progress now (the
/// running job, else the plain call); null when none does.  Any thread
KICOMMON_API nlohmann::json KopenapiRunningProgress();

/// @brief The running job was asked to stop (job_cancel, Esc, a window close)
KICOMMON_API bool KopenapiJobCancelled();

/// @brief The service stops: every queued or running job is cancelled (as job_cancel) and the
/// running plain call is asked to stop at its next safe point (KopenapiJobStep / KopenapiJobProgress
/// / KopenapiJobCancelled answer stop).  Any thread
KICOMMON_API void KopenapiStopAll( const std::string& aWhy );

/// @brief A job runs now (its input filter is installed).  Any thread
KICOMMON_API bool KopenapiJobRunning();

/// @brief An API call or job runs now (no one to answer a question in a dialog).  Any thread
KICOMMON_API bool KopenapiCallActive();

/// @brief Stopped by job_cancel {keep: true}: the job applies nothing and keeps its state for later
/// (a grid route: its timeline, every lane stopped at its last step).  Any thread
KICOMMON_API bool KopenapiJobKeep();

/// @brief Stopped by job_cancel {apply: false} or {keep: true}: whatever the job did is not applied
/// (taken back when it was written as it went).  Any thread
KICOMMON_API bool KopenapiJobDiscard();

/// @brief The running job is paused (job_pause) and not asked to stop.  aSerial / aBy / aN: the last
/// job_step (the serial grows with each), so a worker thread of the job can go that far and wait
/// again.  Any thread
KICOMMON_API bool KopenapiJobHeld( uint64_t* aSerial = nullptr, std::string* aBy = nullptr, int* aN = nullptr );

/// @brief Main thread: runs the API calls queued for it now (none of what arrives meanwhile);
/// GUI and headless (kicadopenapi_service.cpp)
KICOMMON_API void KopenapiServeQueuedCalls();

/// @brief True on the thread API calls run on: the main thread in GUI, the owner strand headless
KICOMMON_API bool KopenapiOnDispatchThread();

/// @brief The main thread is at a safe point of a job or a plain call: a call running now is nested in the job
KICOMMON_API bool KopenapiJobYielding();


class KICOMMON_API KOPENAPI_JOBS
{
public:
    static KOPENAPI_JOBS& Get();

    /// @brief HTTP thread: a new queued job; empty and aWhy when one is queued or running.
    /// aPlainCall: a plain call run as a job (plain_call in its status)
    std::string Add( const std::string& aMethod, std::string& aWhy, bool aPlainCall = false );

    /// @brief Main thread: runs aCall as the job aId (nothing when it was cancelled while queued)
    void Run( const std::string& aId, const std::function<KOPENAPI_RESULT()>& aCall );

    /// @brief The job could not be started (service stopping): it fails with aResult
    void Fail( const std::string& aId, const KOPENAPI_RESULT& aResult );

    /// @brief HTTP thread: waits up to aFor for the job to end; true and its result when it has
    bool Wait( const std::string& aId, std::chrono::milliseconds aFor, KOPENAPI_RESULT& aResult );

    /// @brief job_get / job_cancel / job_list (HTTP thread)
    KOPENAPI_RESULT Status( const nlohmann::json& aArgs );
    KOPENAPI_RESULT Cancel( const nlohmann::json& aArgs );

    /// @brief HTTP thread: pause / resume a job at its next safe point (a queued one starts paused)
    KOPENAPI_RESULT Pause( const nlohmann::json& aArgs, bool aPause );

    /// @brief HTTP thread: a paused job goes on by {by: net | round, n} and pauses again
    KOPENAPI_RESULT Step( const nlohmann::json& aArgs );
    KOPENAPI_RESULT List() const;

    /// @brief "job-N (method)" of the queued or running job; empty when none.
    /// aExceptRunning: not the job running now (a handler run as that job asks for others)
    std::string Busy( bool aExceptRunning = false ) const;

private:
    KOPENAPI_JOBS() = default;
};

#endif
