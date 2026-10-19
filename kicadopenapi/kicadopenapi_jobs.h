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

#include <functional>
#include <string>


/// @brief At a safe point of a call that runs as a job: records the progress (phase, aDone of
/// aTotal, the item at work, counts so far) and lets the UI and reading API calls run.  False once
/// the job is asked to stop: the caller finishes with what it has done.  Outside a job: true.
KICOMMON_API bool KopenapiJobStep( const std::string& aPhase, int aDone, int aTotal,
                                   const std::string& aItem = std::string(),
                                   const nlohmann::json& aCounts = nlohmann::json() );

/// @brief The running job was asked to stop (job_cancel, Esc, a window close)
KICOMMON_API bool KopenapiJobCancelled();

/// @brief The main thread is at a job's safe point: a call running now is nested in the job
KICOMMON_API bool KopenapiJobYielding();


class KICOMMON_API KOPENAPI_JOBS
{
public:
    static KOPENAPI_JOBS& Get();

    /// @brief HTTP thread: a new queued job; empty and aWhy when one is queued or running
    std::string Add( const std::string& aMethod, std::string& aWhy );

    /// @brief Main thread: runs aCall as the job aId (nothing when it was cancelled while queued)
    void Run( const std::string& aId, const std::function<KOPENAPI_RESULT()>& aCall );

    /// @brief The job could not be started (service stopping): it fails with aResult
    void Fail( const std::string& aId, const KOPENAPI_RESULT& aResult );

    /// @brief job_get / job_cancel / job_list (HTTP thread)
    KOPENAPI_RESULT Status( const nlohmann::json& aArgs );
    KOPENAPI_RESULT Cancel( const nlohmann::json& aArgs );
    KOPENAPI_RESULT List() const;

    /// @brief "job-N (method)" of the queued or running job; empty when none
    std::string Busy() const;

private:
    KOPENAPI_JOBS() = default;
};

#endif
