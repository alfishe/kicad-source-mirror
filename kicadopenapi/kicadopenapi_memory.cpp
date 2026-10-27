/// @file kicadopenapi_memory.cpp
/// @brief Memory numbers and the automatic allocator trim (see kicadopenapi_memory.h).
#include <kicadopenapi_journal.h>
#include <kicadopenapi_memory.h>
#include <kicadopenapi_service.h>

#include <embedded_files.h>

#include "platform/platform.h"

#include <wx/event.h>
#include <wx/eventfilter.h>
#include <wx/log.h>
#include <wx/utils.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <thread>


namespace
{
using CLOCK = std::chrono::steady_clock;

constexpr int64_t MB = 1024 * 1024;
constexpr int64_t MIN_FREED = 64 * MB;           ///< growth of footprint - live since the last trim
constexpr auto    MIN_INTERVAL = std::chrono::seconds( 2 );
constexpr double  MAX_DUTY = 0.02;                 ///< trim pauses may take this share of the time between trims
constexpr auto    IDLE = std::chrono::milliseconds( 1000 );
constexpr auto    IDLE_LONG_PAUSE = std::chrono::milliseconds( 3000 );
constexpr double  LONG_PAUSE_MS = 150.0;


int64_t msSince( CLOCK::time_point aWhen )
{
    return std::chrono::duration_cast<std::chrono::milliseconds>( CLOCK::now() - aWhen ).count();
}


/// @brief Expected pause of a trim releasing aBytes (POC-001 fits)
double predictedPauseMs( int64_t aBytes )
{
    const double mb = static_cast<double>( aBytes ) / MB;
#if defined( __APPLE__ )
    return 20.0 + 0.12 * mb;
#else
    return 1.0 + 0.15 * mb;
#endif
}


nlohmann::json orNull( int64_t aValue )
{
    return aValue < 0 ? nlohmann::json() : nlohmann::json( aValue );
}


nlohmann::json processJson( const kopenapi::platform::ProcessMemoryInfo& aInfo )
{
    return { { "rss", orNull( aInfo.rss ) },
             { "footprint", orNull( aInfo.footprint ) },
             { "footprint_kind", aInfo.footprintKind },
             { "peak_rss", orNull( aInfo.peakRss ) },
             { "peak_footprint", orNull( aInfo.peakFootprint ) },
             { "virtual", orNull( aInfo.virtualSize ) },
             { "compressed", orNull( aInfo.compressed ) },
             { "swapped", orNull( aInfo.swapped ) },
             { "threads", aInfo.threads < 0 ? nlohmann::json() : nlohmann::json( aInfo.threads ) } };
}


nlohmann::json allocatorJson( const kopenapi::platform::AllocatorInfo& aInfo )
{
    const bool known = aInfo.live >= 0 && aInfo.mapped >= 0;

    return { { "allocator", aInfo.allocator },
             { "live", orNull( aInfo.live ) },
             { "mapped", orNull( aInfo.mapped ) },
             { "retained", known ? nlohmann::json( aInfo.mapped - aInfo.live ) : nlohmann::json() },
             { "peak_live", orNull( aInfo.peakLive ) } };
}


/// @brief Records the time of the last keyboard / mouse input (main thread, every event: kept trivial)
class INPUT_FILTER : public wxEventFilter
{
public:
    int FilterEvent( wxEvent& aEvent ) override;
};


struct STATE
{
    std::mutex              mutex;            ///< guards the fields below, waits of the thread
    std::condition_variable wake;
    std::thread             thread;
    bool                    stop = false;
    bool                    running = false;
    INPUT_FILTER*           filter = nullptr;

    int                     trims = 0;
    int                     autoTrims = 0;
    nlohmann::json          last;             ///< result of the last trim
    CLOCK::time_point       lastTrim{};
    int64_t                 unusedAfterTrim = 0;   ///< footprint - live after the last trim
    CLOCK::time_point       nextStats{};           ///< earliest next allocator walk (rated by its cost)

    std::atomic<int64_t>    lastInputTicks{ 0 };   ///< CLOCK ticks of the last user input

    ~STATE()
    {
        std::thread finished;

        {
            std::lock_guard<std::mutex> lock( mutex );
            stop = true;
            finished = std::move( thread );
        }

        wake.notify_all();

        if( finished.joinable() )
            finished.join();
    }
};


STATE& state()
{
    static STATE s_state;
    return s_state;
}


int INPUT_FILTER::FilterEvent( wxEvent& aEvent )
{
    const wxEventType type = aEvent.GetEventType();

    if( type == wxEVT_KEY_DOWN || type == wxEVT_MOTION || type == wxEVT_MOUSEWHEEL
            || type == wxEVT_LEFT_DOWN || type == wxEVT_LEFT_UP || type == wxEVT_RIGHT_DOWN
            || type == wxEVT_MIDDLE_DOWN )
    {
        state().lastInputTicks.store( CLOCK::now().time_since_epoch().count(), std::memory_order_relaxed );
    }

    return Event_Skip;
}


/// @brief Idle long enough for a trim of this size: no input for IDLE (IDLE_LONG_PAUSE when the
/// pause would be noticeable) and no API call on the main thread
bool idleFor( int64_t aUnused )
{
    if( KICAD_OPENAPI_SERVICE::InMainThreadCall() )
        return false;

    const CLOCK::time_point lastInput{ CLOCK::duration( state().lastInputTicks.load( std::memory_order_relaxed ) ) };
    const auto              needed = predictedPauseMs( aUnused ) > LONG_PAUSE_MS ? IDLE_LONG_PAUSE : IDLE;

    return CLOCK::now() - lastInput >= needed;
}


void trimLoop()
{
    kopenapi::platform::LowerCurrentThreadPriority();

    // logged at once into the journal; wx's per-thread buffer is flushed at exit into a message box
    wxLog::SetThreadActiveTarget( KOPENAPI_JOURNAL::ThreadTarget() );

    STATE& s = state();
    std::unique_lock<std::mutex> lock( s.mutex );

    while( !s.stop )
    {
        s.wake.wait_for( lock, std::chrono::seconds( 1 ) );

        if( s.stop )
            break;

        if( s.trims > 0 && CLOCK::now() - s.lastTrim < MIN_INTERVAL )
            continue;

        if( CLOCK::now() < s.nextStats )
            continue;

        const int64_t floor = s.unusedAfterTrim;
        const int64_t sinceTrimMs = s.trims > 0 ? msSince( s.lastTrim ) : std::numeric_limits<int64_t>::max();
        lock.unlock();

        // The allocator numbers walk the heap under its lock on Windows (HeapSummary): every
        // allocation of every thread waits meanwhile, at this thread's lowered priority. Taken only
        // when idle and at most MAX_DUTY of the time.
        if( !idleFor( 0 ) )
        {
            lock.lock();
            continue;
        }

        // Memory the OS charges but nothing uses: what a trim can give back. The allocator's own
        // "retained" (mapped - live) is no measure on macOS, where zones keep released ranges mapped.
        const CLOCK::time_point                     statsStart = CLOCK::now();
        const kopenapi::platform::ProcessMemoryInfo process = kopenapi::platform::ProcessMemory();
        const kopenapi::platform::AllocatorInfo     allocator = kopenapi::platform::AllocatorStats();
        const CLOCK::duration                       statsCost = CLOCK::now() - statsStart;
        const int64_t                               unused = process.footprint - allocator.live;
        bool                                        trimmed = false;

        // rate by cost: a short cycle of loads and closes gets a trim per cycle, never a pause
        // taking more than MAX_DUTY of the time
        const bool affordable = predictedPauseMs( unused ) <= MAX_DUTY * static_cast<double>( sinceTrimMs );

        if( process.footprint >= 0 && allocator.live >= 0 && unused >= floor + MIN_FREED && affordable
                && idleFor( unused ) )
        {
            KOPENAPI_MEMORY::TrimNow( "auto" );
            trimmed = true;
        }

        lock.lock();

        s.nextStats = CLOCK::now() + std::chrono::duration_cast<CLOCK::duration>( statsCost / MAX_DUTY );

        if( trimmed )
            s.autoTrims++;
    }
}
} // namespace


void KOPENAPI_MEMORY::StartAutoTrim()
{
    if( const char* env = std::getenv( "KICAD_OPENAPI_AUTO_TRIM" ); env && std::strcmp( env, "0" ) == 0 )
        return;

    STATE& s = state();
    std::lock_guard<std::mutex> lock( s.mutex );

    if( s.running )
        return;

    s.stop = false;
    s.running = true;
    s.lastInputTicks.store( CLOCK::now().time_since_epoch().count() );
    s.filter = new INPUT_FILTER;
    wxEvtHandler::AddFilter( s.filter );
    s.thread = std::thread( trimLoop );
}


void KOPENAPI_MEMORY::StopAutoTrim()
{
    STATE&      s = state();
    std::thread finished;

    {
        std::lock_guard<std::mutex> lock( s.mutex );

        if( !s.running )
            return;

        s.stop = true;
        s.running = false;
        finished = std::move( s.thread );

        if( s.filter )
        {
            wxEvtHandler::RemoveFilter( s.filter );
            delete s.filter;
            s.filter = nullptr;
        }
    }

    s.wake.notify_all();

    if( finished.joinable() )
        finished.join();
}


bool KOPENAPI_MEMORY::UnderPressure( double aProcessShare )
{
    const kopenapi::platform::SystemMemoryInfo system = kopenapi::platform::SystemMemory();

    if( system.pressure > 0 )
        return true;

    return system.total > 0
           && kopenapi::platform::ProcessMemory().footprint > (int64_t) ( (double) system.total * aProcessShare );
}


int64_t KOPENAPI_MEMORY::SystemTotal()
{
    return kopenapi::platform::SystemMemory().total;
}


int64_t KOPENAPI_MEMORY::SystemAvailable()
{
    return kopenapi::platform::SystemMemory().available;
}


void KOPENAPI_MEMORY::LowerThreadPriority()
{
    kopenapi::platform::LowerCurrentThreadPriority();
}


nlohmann::json KOPENAPI_MEMORY::Summary()
{
    const kopenapi::platform::ProcessMemoryInfo process = kopenapi::platform::ProcessMemory();
    const kopenapi::platform::AllocatorInfo     allocator = kopenapi::platform::AllocatorStats();

    // footprint - live: charged by the OS but used by nothing, what a trim can give back
    nlohmann::json out = { { "process", processJson( process ) },
                           { "allocator", allocatorJson( allocator ) },
                           { "unused", process.footprint >= 0 && allocator.live >= 0
                                               ? nlohmann::json( process.footprint - allocator.live )
                                               : nlohmann::json() } };

    const EMBEDDED_FILES::DATA_STATS embedded = EMBEDDED_FILES::GetDataStats();

    const kopenapi::platform::SystemMemoryInfo system = kopenapi::platform::SystemMemory();
    out["system"] = { { "total", orNull( system.total ) },
                      { "available", orNull( system.available ) },
                      { "pressure", system.pressure == 2 ? "critical" : system.pressure == 1 ? "warning" : "normal" } };

    out["caches"] = { { "embedded_files", { { "resident_bytes", embedded.residentBytes },
                                            { "resident_files", embedded.residentFiles },
                                            { "decodes", embedded.decodes },
                                            { "releases", embedded.releases } } } };

#if defined( _WIN32 )
    out["os"] = "windows";
#elif defined( __APPLE__ )
    out["os"] = "macos";
#elif defined( __linux__ )
    out["os"] = "linux";
#else
    out["os"] = "other";
#endif

    STATE& s = state();
    std::lock_guard<std::mutex> lock( s.mutex );

    nlohmann::json last = s.last;

    if( !last.is_null() )
        last["age_ms"] = msSince( s.lastTrim );

    out["auto_trim"] = { { "enabled", s.running },
                         { "trims", s.trims },
                         { "auto_trims", s.autoTrims },
                         { "last", std::move( last ) } };
    return out;
}


nlohmann::json KOPENAPI_MEMORY::TrimNow( const std::string& aTrigger )
{
    namespace kp = kopenapi::platform;

    const kp::ProcessMemoryInfo processBefore = kp::ProcessMemory();
    const kp::AllocatorInfo     before = kp::AllocatorStats();
    const auto                  start = CLOCK::now();
    const int64_t               reported = kp::AllocatorTrim();
    const double                ms = std::chrono::duration<double, std::milli>( CLOCK::now() - start ).count();
    const kp::ProcessMemoryInfo processAfter = kp::ProcessMemory();
    const kp::AllocatorInfo     after = kp::AllocatorStats();

    auto side = []( const kp::ProcessMemoryInfo& aProcess, const kp::AllocatorInfo& aAllocator )
    {
        return nlohmann::json{ { "footprint", orNull( aProcess.footprint ) },
                               { "live", orNull( aAllocator.live ) },
                               { "unused", aProcess.footprint >= 0 && aAllocator.live >= 0
                                                   ? nlohmann::json( aProcess.footprint - aAllocator.live )
                                                   : nlohmann::json() },
                               { "retained", aAllocator.live >= 0 ? nlohmann::json( aAllocator.mapped - aAllocator.live )
                                                                  : nlohmann::json() } };
    };

    nlohmann::json result = { { "trigger", aTrigger },
                              { "before", side( processBefore, before ) },
                              { "after", side( processAfter, after ) },
                              { "released", processBefore.footprint >= 0 && processAfter.footprint >= 0
                                                    ? nlohmann::json( processBefore.footprint - processAfter.footprint )
                                                    : nlohmann::json() },
                              { "allocator_reported", orNull( reported ) },
                              { "ms", std::round( ms * 10 ) / 10 } };

    {
        STATE& s = state();
        std::lock_guard<std::mutex> lock( s.mutex );
        s.trims++;
        s.last = result;
        s.lastTrim = CLOCK::now();
        s.unusedAfterTrim = processAfter.footprint >= 0 && after.live >= 0 ? processAfter.footprint - after.live : 0;
    }

    wxLogMessage( "memory trim (%s): footprint %.1f -> %.1f MB, %.1f ms", wxString::FromUTF8( aTrigger ),
                  processBefore.footprint / double( MB ), processAfter.footprint / double( MB ), ms );

    return result;
}
