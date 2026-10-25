/// @file kicadopenapi_keepalive.h
/// @brief Keeps KiCad's UI alive while an API call runs on the main thread: long operations call
/// KopenapiKeepUiAlive() now and then, which repaints and runs timers (glow, recordings) but
/// leaves user input and further API calls queued until the call ends. GUI only; no-op headless
/// and off the main thread.
#ifndef KICADOPENAPI_KEEPALIVE_H
#define KICADOPENAPI_KEEPALIVE_H

#include <kicommon.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <widgets/progress_reporter_base.h>


/// @brief Lets the UI repaint and timers run (at most every 30 ms); cheap to call often
KICOMMON_API void KopenapiKeepUiAlive();

/// @brief A job's safe point (kicadopenapi_jobs.h): every waiting event runs, API calls queued for
/// the main thread too (at most every 30 ms); input goes through the job's filter
KICOMMON_API void KopenapiYieldAll();

/// @brief Runs aWork on a worker thread and keeps the UI alive until it finishes; for work that
/// does not touch the KiCad model (file conversions, geometry kernels)
KICOMMON_API void KopenapiRunOffMain( const std::function<void()>& aWork );


/// @brief Work that only reads the documents (a route search, DRC tests); API readers may read
/// meanwhile.  On the GUI main thread it runs on a worker while the main loop keeps painting and,
/// during a job, navigating (the job's input filter); elsewhere (the headless owner) right here.
/// Either way inside a MODEL_THREAD::READ_WINDOW.  aWork must not write the documents nor touch
/// windows; it may call KopenapiJobStep (progress, stop).  Rethrows what aWork throws.
/// KICAD_OPENAPI_MAIN_DISPATCH runs it right here on the main thread too (A/B)
KICOMMON_API void KopenapiCompute( const std::function<void()>& aWork );

/// @brief A point in a long edit where the documents are consistent: API readers waiting for them
/// read now, before the edit goes on (the owner's write ends and is taken again)
KICOMMON_API void KopenapiReadPoint();

/// @brief True on a worker of KopenapiCompute
KICOMMON_API bool KopenapiInComputeWorker();

#include <wx/toplevel.h>
#include <wx/weakref.h>


class LIBRARY_MANAGER_ADAPTER;
class wxTopLevelWindow;


/// @brief While it lives, a window is under API control: mouse / keyboard input to it is dropped
/// (not queued for later) and its title bar says "remote controlled (API)". Restores both when it
/// goes. Nested guards on one window are fine. GUI, main thread.
class KICOMMON_API KOPENAPI_REMOTE_GUARD
{
public:
    explicit KOPENAPI_REMOTE_GUARD( wxTopLevelWindow* aWindow );
    ~KOPENAPI_REMOTE_GUARD();

    KOPENAPI_REMOTE_GUARD( const KOPENAPI_REMOTE_GUARD& ) = delete;
    KOPENAPI_REMOTE_GUARD& operator=( const KOPENAPI_REMOTE_GUARD& ) = delete;

private:
    wxTopLevelWindow*            m_key;     ///< the filter's key (never dereferenced)
    wxWeakRef<wxTopLevelWindow>  m_window;  ///< null once the window is gone (closed while held)
};

/// @brief Waits for an adapter's background library load with the UI alive
/// (at most aTimeoutS: a load that stops progressing is left behind with a journal entry, the call
/// goes on with the libraries loaded so far instead of the UI waiting forever)
KICOMMON_API void KopenapiWaitLibraries( LIBRARY_MANAGER_ADAPTER* aAdapter, int aTimeoutS = 300 );


/// @brief Progress reporter for KiCad algorithms that accept one (zone filler, DRC): no dialog,
/// keeps the UI alive while they run
class KICOMMON_API KOPENAPI_KEEPALIVE_REPORTER : public PROGRESS_REPORTER_BASE
{
public:
    KOPENAPI_KEEPALIVE_REPORTER() : PROGRESS_REPORTER_BASE( 1 ) {}

    /// Algorithms that only step phases (ERC) never ask for a refresh: each phase is a pump
    void AdvancePhase() override
    {
        PROGRESS_REPORTER_BASE::AdvancePhase();
        KopenapiKeepUiAlive();
    }

    void AdvancePhase( const wxString& aMessage ) override
    {
        PROGRESS_REPORTER_BASE::AdvancePhase( aMessage );
        KopenapiKeepUiAlive();
    }

protected:
    bool updateUI() override
    {
        KopenapiKeepUiAlive();
        return true;
    }
};


/// @brief Progress reporter for KiCad algorithms that accept one (DRC, zone filler) in a call that
/// reports progress: no dialog; hands aReport the algorithm's message, its phase (1-based) and the
/// share of that phase done, at most every 50 ms and at each new phase or message (the UI kept
/// alive in between); false from aReport cancels the algorithm
class KICOMMON_API KOPENAPI_PROGRESS_REPORTER : public PROGRESS_REPORTER_BASE
{
public:
    using REPORT = std::function<bool( const std::string& aMessage, int aPhase, double aPhaseFraction )>;

    explicit KOPENAPI_PROGRESS_REPORTER( REPORT aReport ) :
            PROGRESS_REPORTER_BASE( 1 ),
            m_report( std::move( aReport ) )
    {
    }

protected:
    bool updateUI() override
    {
        wxString message;

        {
            std::lock_guard<std::mutex> lock( m_mutex );
            message = m_rptMessage;
        }

        const int  phase = m_phase.load();
        const auto now = std::chrono::steady_clock::now();

        if( phase == m_lastPhase && message == m_lastMessage && now - m_lastReport < std::chrono::milliseconds( 50 ) )
        {
            KopenapiKeepUiAlive();
            return true;
        }

        m_lastPhase = phase;
        m_lastMessage = message;
        m_lastReport = now;

        const int    max = m_maxProgress.load();
        const double fraction = max > 0 ? std::clamp( (double) m_progress.load() / max, 0.0, 1.0 ) : 0.0;

        return m_report( message.ToStdString( wxConvUTF8 ), phase, fraction );
    }

private:
    REPORT                                m_report;
    int                                   m_lastPhase = -1;
    wxString                              m_lastMessage;
    std::chrono::steady_clock::time_point m_lastReport;
};

#endif
