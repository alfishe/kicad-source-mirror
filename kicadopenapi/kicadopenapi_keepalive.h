/// @file kicadopenapi_keepalive.h
/// @brief Keeps KiCad's UI alive while an API call runs on the main thread: long operations call
/// KopenapiKeepUiAlive() now and then, which repaints and runs timers (glow, recordings) but
/// leaves user input and further API calls queued until the call ends. GUI only; no-op headless
/// and off the main thread.
#ifndef KICADOPENAPI_KEEPALIVE_H
#define KICADOPENAPI_KEEPALIVE_H

#include <kicommon.h>

#include <functional>
#include <widgets/progress_reporter_base.h>


/// @brief Lets the UI repaint and timers run (at most every 30 ms); cheap to call often
KICOMMON_API void KopenapiKeepUiAlive();

/// @brief Runs aWork on a worker thread and keeps the UI alive until it finishes; for work that
/// does not touch the KiCad model (file conversions, geometry kernels)
KICOMMON_API void KopenapiRunOffMain( const std::function<void()>& aWork );


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
    wxTopLevelWindow* m_window;
};

/// @brief Waits for an adapter's background library load with the UI alive
KICOMMON_API void KopenapiWaitLibraries( LIBRARY_MANAGER_ADAPTER* aAdapter );


/// @brief Progress reporter for KiCad algorithms that accept one (zone filler, DRC): no dialog,
/// keeps the UI alive while they run
class KICOMMON_API KOPENAPI_KEEPALIVE_REPORTER : public PROGRESS_REPORTER_BASE
{
public:
    KOPENAPI_KEEPALIVE_REPORTER() : PROGRESS_REPORTER_BASE( 1 ) {}

protected:
    bool updateUI() override
    {
        KopenapiKeepUiAlive();
        return true;
    }
};

#endif
