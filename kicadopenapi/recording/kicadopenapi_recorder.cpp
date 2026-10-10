/// @file kicadopenapi_recorder.cpp
/// @brief Video recording of a KiCad window or of its canvas: record_start, record_stop,
/// record_status, record_pause, record_resume, record_marker.
///
/// A timer on the main thread captures frames (offscreen, see kicadopenapi_capture.h) into a short
/// queue; an encoder thread scales them to the video size and encodes. When capture or encoding
/// falls behind, frames are dropped (counted) and the previous frame stays on screen, so the video
/// keeps real time. GUI only (registered GUI-only: headless answers 501).
#include "kicadopenapi_recorder.h"
#include "frame_scaler.h"
#include "video_encoder.h"

#include <wx/display.h>

#include <kicadopenapi_capture.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>

#include <kiway.h>
#include <project.h>

#include <wx/datetime.h>
#include <wx/image.h>
#include <wx/timer.h>
#include <wx/toplevel.h>
#include <wx/weakref.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>


namespace
{

using CLOCK = std::chrono::steady_clock;


struct FRAME
{
    int64_t              index = 0;
    int                  width = 0;
    int                  height = 0;
    std::vector<uint8_t> rgb;
};


class RECORDER;


class RECORD_TIMER : public wxTimer
{
public:
    explicit RECORD_TIMER( RECORDER& aRecorder ) : m_recorder( aRecorder ) {}
    void Notify() override;

private:
    RECORDER& m_recorder;
};


class RECORDER
{
public:
    bool Active() const { return m_active; }
    int  Fps() const { return m_settings.fps; }

    KOPENAPI_RESULT Start( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
    {
        if( m_active )
            return KOPENAPI_RESULT::Error( 409, "a recording is running (record_stop)" );

        const std::string source = aArgs.value( "source", std::string( "window" ) );

        if( std::string error; !SetSteady( aArgs.value( "steadycam", nlohmann::json( true ) ), error ) )
            return KOPENAPI_RESULT::Error( 400, error );

        // how frames are taken: the GPU (composed and scaled there), the CPU (one read, SIMD
        // scaling on the encoder thread) or whichever works (auto)
        m_capturePath = aArgs.value( "capture", std::string( "auto" ) );

        if( m_capturePath != "auto" && m_capturePath != "gpu" && m_capturePath != "cpu" )
            return KOPENAPI_RESULT::Error( 400, "capture: auto, gpu or cpu" );

        // the clock: video time (every frame the next one, never faster than real time; when
        // capture is slower the video runs longer instead of skipping) or wall time (frames the
        // wall clock passed by are skipped)
        m_clockMode = aArgs.value( "clock", std::string( "auto" ) );

        if( m_clockMode != "auto" && m_clockMode != "video" && m_clockMode != "wall" )
            return KOPENAPI_RESULT::Error( 400, "clock: auto, video or wall" );

        m_videoClock = m_clockMode != "wall";
        m_lastFrameWall = CLOCK::time_point();
        m_videoEpoch = CLOCK::time_point();
        m_videoFrames = 0;
        m_clockMs.clear();
        m_clockSkipped = 0;
        m_clockLog = nlohmann::json::array();

        m_steadyLast = CLOCK::time_point();
        m_chrome = wxImage();
        m_windowMs = m_pasteMs = 0;
        m_windowFrames = m_pasteFrames = 0;
        m_chromeMs = std::clamp( aArgs.value( "chrome_ms", 200 ), 0, 5000 );

        if( source != "window" && source != "canvas" )
            return KOPENAPI_RESULT::Error( 400, "source: window or canvas" );

        wxTopLevelWindow* window = KopenapiFindWindow( aArgs.value( "window", std::string() ) );

        if( !window )
            return KOPENAPI_RESULT::Error( 404, "window not found (see window_list: id or title glob)" );

        if( window->IsIconized() )
            return KOPENAPI_RESULT::Error( 409, "the window is minimized" );

        m_canvas = nullptr;

        if( source == "canvas" && !( m_canvas = KopenapiFindCanvas( window ) ) )
            return KOPENAPI_RESULT::Error( 409, "the window has no canvas; record source: window" );

        KOPENAPI_VIDEO_SETTINGS settings;
        settings.format = aArgs.value( "format", std::string( "mp4" ) );
        settings.codec = aArgs.value( "codec", std::string( "h264" ) );
        settings.fps = std::clamp( aArgs.value( "fps", source == "canvas" ? 30 : 20 ), 1, 60 );
        settings.quality = std::clamp( aArgs.value( "quality", 5 ), 0, 10 );

        if( settings.format != "mp4" && settings.format != "gif" )
            return KOPENAPI_RESULT::Error( 400, "format: mp4 or gif" );

        if( settings.codec != "h264" && settings.codec != "hevc" )
            return KOPENAPI_RESULT::Error( 400, "codec: h264 or hevc" );

        if( aArgs.contains( "path" ) )
        {
            settings.path = std::filesystem::path( aArgs["path"].get<std::string>() );
        }
        else
        {
            const wxString dir = aCtx.kiway ? aCtx.kiway->Prj().GetProjectPath() : wxString();

            if( dir.IsEmpty() )
                return KOPENAPI_RESULT::Error( 400, "no project open: give path" );

            const std::string stamp = wxDateTime::Now().Format( wxS( "%Y%m%d-%H%M%S" ) ).ToStdString();
            settings.path = std::filesystem::path( dir.ToStdString( wxConvUTF8 ) ) / "recordings"
                            / ( "kicad-" + stamp + "." + settings.format );
        }

        if( !settings.path.is_absolute() )
            return KOPENAPI_RESULT::Error( 400, "path must be absolute" );

        std::error_code ec;
        std::filesystem::create_directories( settings.path.parent_path(), ec );

        m_source = source;
        m_renderSize = wxSize();

        // first frame decides the video size
        wxImage first;

        if( !capture( window, first ) )
            return KOPENAPI_RESULT::Error( 501, "this platform cannot draw windows offscreen (e.g. Wayland)" );

        // profile: a fixed frame (frames fitted in, letterboxed) or the source size (capped)
        const std::string profile = aArgs.value( "profile", std::string( "source" ) );

        if( profile == "4k" || profile == "1080p" || profile == "custom" )
        {
            settings.width = profile == "4k" ? 3840 : profile == "1080p" ? 1920 : aArgs.value( "width", 0 ) & ~1;
            settings.height = profile == "4k" ? 2160 : profile == "1080p" ? 1080 : aArgs.value( "height", 0 ) & ~1;

            if( settings.width < 16 || settings.height < 16 || settings.width > 7680 || settings.height > 4320 )
                return KOPENAPI_RESULT::Error( 400, "profile custom: width and height (16..7680 x 16..4320)" );

            // frames of another size are fitted on the encoder thread (SIMD), so the encoder gets
            // the video's size
            settings.inputWidth = settings.width;
            settings.inputHeight = settings.height;
            m_letterbox = true;

            // a canvas that can draw itself at the video's size (the 3D viewer) is rendered there:
            // sharp, no scaling, no bars
            wxImage sharp;

            m_qualityMode = aArgs.value( "quality_mode", std::string( "adaptive" ) );

            if( m_capturePath != "auto" )
                m_qualityMode = "max";   // the path asked for stays

            m_tier = m_capturePath == "cpu" || m_qualityMode == "fast" ? TIER_READBACK
                     : aArgs.value( "antialias", std::string( "ssaa2" ) ) == "none" ? TIER_GPU : TIER_GPU_SSAA;
            m_tierLog = nlohmann::json::array();
            m_tierMs.clear();
            m_tierFrames = 0;

            if( m_capturePath == "gpu" && !( m_canvas && KopenapiRenderCanvas( m_canvas, settings.width, settings.height, sharp ) ) )
                return KOPENAPI_RESULT::Error( 409, "capture gpu: this canvas cannot be rendered on the GPU here (source canvas, a "
                                                    "4k / 1080p / custom profile, OpenGL)" );

            if( m_canvas && m_tier != TIER_READBACK && KopenapiRenderCanvas( m_canvas, settings.width, settings.height, sharp ) )
            {
                m_renderSize = wxSize( settings.width, settings.height );
                renderSharp( sharp );
                settings.inputWidth = settings.width;   // readback frames are fitted to this size
                settings.inputHeight = settings.height;
                first = sharp;
            }
        }
        else if( profile == "source" )
        {
            const int maxWidth = std::clamp( aArgs.value( "max_width", 1920 ), 160, 7680 );
            double    scale = std::min( 1.0, double( maxWidth ) / first.GetWidth() );
            settings.width = std::max( 2, int( first.GetWidth() * scale ) & ~1 );
            settings.height = std::max( 2, int( first.GetHeight() * scale ) & ~1 );
            m_letterbox = false;
        }
        else
        {
            return KOPENAPI_RESULT::Error( 400, "profile: 4k, 1080p, custom or source" );
        }

        const std::string encoder = aArgs.value( "encoder", std::string( "auto" ) );
        std::string       error;

        m_encoder.reset();

        if( encoder == "auto" || encoder == "native" )
            m_encoder = KopenapiNativeVideoEncoder( settings );

        if( !m_encoder && encoder == "native" )
            return KOPENAPI_RESULT::Error( 409, "no native encoder for this format on this OS (encoder: ffmpeg)" );

        if( !m_encoder )
            m_encoder = KopenapiFfmpegVideoEncoder( error );

        if( !m_encoder )
            return KOPENAPI_RESULT::Error( 409, error );

        if( !m_encoder->Open( settings, error ) )
        {
            m_encoder.reset();
            return KOPENAPI_RESULT::Error( 500, error );
        }

        m_settings = settings;
        m_window = window;
        m_windowTitle = window->GetTitle().ToStdString( wxConvUTF8 );
        m_maxSeconds = std::clamp( aArgs.value( "max_seconds", 600.0 ), 1.0, 4.0 * 3600 );
        m_start = CLOCK::now();
        m_pausedTotal = CLOCK::duration::zero();
        m_paused = false;
        m_lastIndex = -1;
        m_captured = m_written = m_dropped = 0;
        m_captureMsTotal = m_captureMsMax = 0;
        m_error.clear();
        m_stopReason.clear();
        m_markers = nlohmann::json::array();
        m_finishing = false;
        m_active = true;
        m_startedUnixMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::system_clock::now().time_since_epoch() ).count();

        m_thread = std::thread( [this]() { encodeLoop(); } );
        enqueue( first, 0 );

        if( !m_timer )
            m_timer = std::make_unique<RECORD_TIMER>( *this );

        // twice per frame: a timer firing late does not skip a frame
        m_timer->Start( std::max( 1, 500 / settings.fps ) );

        nlohmann::json answer = status();
        answer["encoder"] = m_encoder->Name();
        return KOPENAPI_RESULT::Ok( answer );
    }

    KOPENAPI_RESULT Stop( const std::string& aReason = "requested" )
    {
        if( !m_active )
            return KOPENAPI_RESULT::Error( 409, "no recording is running" );

        m_timer->Stop();
        m_stopReason = aReason;
        const int64_t end = m_rebase ? m_lastIndex + 1 : std::max<int64_t>( currentIndex() + 1, m_lastIndex + 1 );

        {
            std::lock_guard<std::mutex> lock( m_mutex );
            m_finishing = true;
            m_endIndex = end;
        }

        m_cv.notify_all();
        m_thread.join();
        m_active = false;

        nlohmann::json answer = status();

        if( !m_markers.empty() )
        {
            const std::filesystem::path sidecar = m_settings.path.string() + ".markers.json";
            std::ofstream( sidecar ) << nlohmann::json( { { "video", m_settings.path.filename().string() },
                                                          { "fps", m_settings.fps },
                                                          { "markers", m_markers } } ).dump( 2 );
            answer["markers_file"] = sidecar.string();
        }

        std::error_code ec;
        answer["file_bytes"] = (int64_t) std::filesystem::file_size( m_settings.path, ec );
        m_encoder.reset();

        if( !m_error.empty() )
            return KOPENAPI_RESULT::Error( 500, m_error + " (" + answer.dump() + ")" );

        return KOPENAPI_RESULT::Ok( answer );
    }

    KOPENAPI_RESULT Pause( bool aPause )
    {
        if( !m_active )
            return KOPENAPI_RESULT::Error( 409, "no recording is running" );

        if( aPause && !m_paused )
        {
            m_paused = true;
            m_pauseStart = CLOCK::now();
        }
        else if( !aPause && m_paused )
        {
            m_paused = false;
            m_pausedTotal += CLOCK::now() - m_pauseStart;
        }

        return KOPENAPI_RESULT::Ok( status() );
    }

    KOPENAPI_RESULT Marker( const std::string& aText )
    {
        if( !m_active )
            return KOPENAPI_RESULT::Error( 409, "no recording is running" );

        const int64_t index = currentIndex();
        m_markers.push_back( { { "time_s", double( index ) / m_settings.fps }, { "frame", index }, { "text", aText } } );
        return KOPENAPI_RESULT::Ok( { { "time_s", double( index ) / m_settings.fps }, { "markers", m_markers.size() } } );
    }

    nlohmann::json status()
    {
        std::lock_guard<std::mutex> lock( m_mutex );

        return { { "recording", m_active && !m_finishing },
                 { "paused", m_paused },
                 { "path", m_settings.path.string() },
                 { "source", m_source },
                 { "window", m_windowTitle },
                 { "format", m_settings.format },
                 { "size", { m_settings.width, m_settings.height } },
                 { "fps", m_settings.fps },
                 { "encoder", m_encoder ? m_encoder->Name() : "" },
                 { "duration_s", double( std::max<int64_t>( currentIndex(), 0 ) ) / std::max( 1, m_settings.fps ) },
                 { "frames_captured", m_captured },
                 { "frames_encoded", m_written },
                 { "frames_dropped", m_dropped },
                 { "capture_ms_avg", m_captured ? std::round( m_captureMsTotal / m_captured * 10 ) / 10 : 0.0 },
                 { "capture_ms_max", std::round( m_captureMsMax * 10 ) / 10 },
                 { "stop_reason", m_stopReason },
                 { "started_unix_ms", m_startedUnixMs },
                 { "steadycam", steadyJson() },
                 { "capture", m_capturePath },
                 { "window_ms_avg", m_windowFrames ? std::round( m_windowMs / m_windowFrames * 10 ) / 10 : 0.0 },
                 { "canvases_ms_avg", m_pasteFrames ? std::round( m_pasteMs / m_pasteFrames * 10 ) / 10 : 0.0 },
                 { "clock", m_videoClock ? "video" : "wall" },
                 { "clock_changes", m_clockLog },
                 { "quality_tier", m_renderSize.x > 0 ? nlohmann::json( tierName( m_tier ) ) : nlohmann::json() },
                 { "quality_changes", m_tierLog },
                 { "error", m_error } };
    }

    void Tick()
    {
        if( !m_active || m_paused )
            return;

        // an animation is feeding frames on video time: the timer stays out of its way, then
        // continues the video clock from the last synced frame
        if( CLOCK::now() - m_lastSync < std::chrono::milliseconds( 150 ) )
            return;

        if( m_rebase )
        {
            m_start = CLOCK::now() - m_pausedTotal
                      - std::chrono::duration_cast<CLOCK::duration>( std::chrono::duration<double>( double( m_lastIndex + 1 ) / m_settings.fps ) );
            m_rebase = false;
        }

        if( !m_window || !m_window->IsShown() || ( m_source == "canvas" && !m_canvas ) )
        {
            Stop( "window closed" );
            return;
        }

        if( std::chrono::duration<double>( CLOCK::now() - m_start - m_pausedTotal ).count() > m_maxSeconds )
        {
            Stop( "max_seconds" );
            return;
        }

        {
            std::lock_guard<std::mutex> lock( m_mutex );

            if( !m_error.empty() )
            {
                m_timer->Stop();   // the encoder failed: record_stop reports it
                return;
            }
        }

        if( m_window->IsIconized() )
            return;

        if( m_videoClock )
        {
            advanceVideo();
            return;
        }

        const int64_t index = currentIndex();

        if( index <= m_lastIndex )
            return;

        m_clockSkipped += int( std::max<int64_t>( 0, index - m_lastIndex - 1 ) );
        advance( index );
    }

    /// @brief Capture and enqueue frame aIndex; keeps the clock decision up to date
    bool advance( int64_t aIndex )
    {
        const auto t0 = CLOCK::now();
        wxImage    image;

        if( !capture( m_window, image ) )
            return false;

        const double ms = std::chrono::duration<double, std::milli>( CLOCK::now() - t0 ).count();
        m_captureMsTotal += ms;
        m_captureMsMax = std::max( m_captureMsMax, ms );

        // the encoder keeps up: on video time wait for it instead of skipping
        if( m_videoClock )
        {
            std::unique_lock<std::mutex> lock( m_mutex );
            m_cv.wait( lock, [this]() { return m_queue.size() < 3 || !m_error.empty(); } );
        }

        enqueue( image, aIndex );
        m_lastFrameWall = CLOCK::now();
        clockStep( ms );
        return true;
    }

    /// @brief Wall clock: notes in clock_changes when frames start being skipped (capture slower
    /// than real time; clock auto / video would not skip)
    void clockStep( double aMs )
    {
        if( m_videoClock )
            return;

        const double budget = 1000.0 / m_settings.fps;

        // frames the wall clock passed by since the last one count as time taken too
        aMs += m_clockSkipped * budget;
        m_clockSkipped = 0;
        m_clockMs.push_back( aMs );

        if( m_clockMs.size() > 30 )
            m_clockMs.pop_front();

        double avg = 0;

        for( double v : m_clockMs )
            avg += v;

        avg /= m_clockMs.size();

        if( m_clockMs.size() >= 10 && avg > budget * 0.95 && m_clockLog.empty() )
        {
            m_clockLog.push_back( { { "time_s", double( m_lastIndex ) / m_settings.fps },
                                    { "clock", "wall" },
                                    { "why", "frames skipped: they take " + std::to_string( int( avg ) ) + " ms (a frame is "
                                                     + std::to_string( int( budget ) ) + "); clock auto would not skip" } } );
        }
    }

    /// @brief Video time: the next frame, once a frame's time has passed since the last one (never
    /// faster than real time; slower when capture is)
    bool advanceVideo()
    {
        if( !m_active || m_paused || !m_videoClock || !m_window )
            return false;

        // frame n is due at epoch + n periods: a late timer is caught up, never ahead of real time;
        // more than a quarter second behind (a long call held the main thread) the schedule moves
        // on and the video runs longer than real time instead of repeating one picture
        const auto now = CLOCK::now();
        const auto period = std::chrono::duration_cast<CLOCK::duration>( std::chrono::duration<double>( 1.0 / m_settings.fps ) );

        if( m_videoEpoch == CLOCK::time_point() )
        {
            m_videoEpoch = now;
            m_videoFrames = 0;
        }

        const auto due = m_videoEpoch + period * m_videoFrames;

        if( now < due )
            return false;

        if( now - due > std::chrono::milliseconds( 250 ) )
        {
            m_videoEpoch = now - period * m_videoFrames;
        }

        if( !advance( m_lastIndex + 1 ) )
            return false;

        m_videoFrames++;
        return true;
    }

    int64_t LastIndex() const { return m_lastIndex; }
    bool    VideoClock() const { return m_videoClock; }

    bool Synced( int* aFps )
    {
        std::lock_guard<std::mutex> lock( m_mutex );

        if( !m_active || m_paused || m_finishing || !m_error.empty() )
            return false;

        if( aFps )
            *aFps = m_settings.fps;

        return true;
    }

    bool RenderSize( wxWindow* aCanvas, int* aWidth, int* aHeight )
    {
        if( !m_active || m_canvas != aCanvas || m_renderSize.x <= 0 || m_tier == TIER_READBACK )
            return false;

        *aWidth = m_renderSize.x;
        *aHeight = m_renderSize.y;
        return true;
    }

    int ScreenSteps()
    {
        int fps = 0;

        if( !Synced( &fps ) || fps <= 0 || !m_window )
            return 1;

        const int display = wxDisplay::GetFromWindow( m_window );
        int       hz = display != wxNOT_FOUND ? wxDisplay( display ).GetCurrentMode().GetRefresh() : 0;

        if( hz <= 0 )
            hz = 60;

        return std::clamp( int( std::lround( double( hz ) / fps ) ), 1, 8 );
    }

    void RecordFrame()
    {
        if( !Synced( nullptr ) || !m_window )
            return;

        wxImage    image;
        const auto t0 = CLOCK::now();

        if( !capture( m_window, image ) )
            return;

        const double ms = std::chrono::duration<double, std::milli>( CLOCK::now() - t0 ).count();
        m_captureMsTotal += ms;
        m_captureMsMax = std::max( m_captureMsMax, ms );

        // the encoder keeps up: wait instead of dropping (this is video time, not wall time)
        {
            std::unique_lock<std::mutex> lock( m_mutex );
            m_cv.wait( lock, [this]() { return m_queue.size() < 3 || !m_error.empty(); } );
        }

        enqueue( image, m_lastIndex + 1 );
        m_lastSync = CLOCK::now();
        m_rebase = true;
    }

    void Shutdown()
    {
        if( m_active )
            Stop( "KiCad closing" );
    }

private:
    /// Quality ladder for canvases that render offscreen (the 3D viewer): full quality first, then
    /// down a step whenever frames take longer than the budget (or the GPU refuses), back up when
    /// there is plenty of room.
    enum TIER
    {
        TIER_GPU_SSAA = 0,   ///< drawn 2x on the GPU and averaged down there
        TIER_GPU = 1,        ///< drawn at the video size on the GPU
        TIER_READBACK = 2,   ///< the canvas as shown, scaled up by the encoder thread (SIMD)
    };

    static const char* tierName( int aTier )
    {
        return aTier == TIER_GPU_SSAA ? "gpu 2x supersampled" : aTier == TIER_GPU ? "gpu at video size" : "window readback, scaled";
    }

    void setTier( int aTier, const std::string& aWhy )
    {
        if( aTier == m_tier )
            return;

        m_tierLog.push_back( { { "time_s", double( std::max<int64_t>( m_lastIndex, 0 ) ) / m_settings.fps },
                               { "from", tierName( m_tier ) }, { "to", tierName( aTier ) }, { "why", aWhy } } );
        m_tier = aTier;
        m_tierMs.clear();
        m_tierFrames = 0;
    }

    /// @brief One frame of an offscreen-capable canvas on the current tier; adapts the tier
    bool renderSharp( wxImage& aImage )
    {
        const auto t0 = CLOCK::now();
        bool       ok = false;

        while( !ok )
        {
            if( m_tier == TIER_READBACK )
            {
                ok = KopenapiCaptureCanvas( m_canvas, aImage );
                break;
            }

            ok = KopenapiRenderCanvas( m_canvas, m_renderSize.x, m_renderSize.y, aImage, m_tier == TIER_GPU_SSAA ? 2 : 1 );

            if( !ok )
                setTier( m_tier + 1, "the GPU could not render this size" );
        }

        if( !ok || m_qualityMode != "adaptive" )
            return ok;

        // budget: most of a frame's time (the UI and the encoder need the rest)
        const double ms = std::chrono::duration<double, std::milli>( CLOCK::now() - t0 ).count();
        const double budget = 800.0 / m_settings.fps;
        m_tierMs.push_back( ms );
        m_tierFrames++;

        if( m_tierMs.size() > 20 )
            m_tierMs.pop_front();

        double avg = 0;

        for( double v : m_tierMs )
            avg += v;

        avg /= m_tierMs.size();

        if( m_tierMs.size() >= 10 && avg > budget && m_tier < TIER_READBACK )
            setTier( m_tier + 1, "frames took " + std::to_string( int( avg ) ) + " ms (budget " + std::to_string( int( budget ) ) + ")" );
        else if( m_tierFrames >= 90 && avg < budget * 0.3 && m_tier > TIER_GPU_SSAA )
            setTier( m_tier - 1, "room to spare (" + std::to_string( int( avg ) ) + " ms)" );

        return true;
    }

public:
    /// @brief Steadycam settings from true / false / "filter" / "timed" / {mode, smooth_ms, duration_ms}
    bool SetSteady( const nlohmann::json& aSpec, std::string& aError )
    {
        KOPENAPI_STEADY steady;
        std::string     mode = "timed";

        if( aSpec.is_boolean() )
            mode = aSpec.get<bool>() ? "timed" : "off";
        else if( aSpec.is_string() )
            mode = aSpec.get<std::string>();
        else if( aSpec.is_object() )
            mode = aSpec.value( "enabled", true ) ? aSpec.value( "mode", std::string( "timed" ) ) : "off";

        if( mode != "off" && mode != "filter" && mode != "timed" )
        {
            aError = "steadycam mode: off, filter or timed";
            return false;
        }

        steady.mode = mode == "off" ? KOPENAPI_STEADY::OFF : mode == "filter" ? KOPENAPI_STEADY::FILTER : KOPENAPI_STEADY::TIMED;

        if( aSpec.is_object() )
        {
            steady.smoothSeconds = std::clamp( aSpec.value( "smooth_ms", 450.0 ), 50.0, 5000.0 ) / 1000.0;
            steady.durationSeconds = std::clamp( aSpec.value( "duration_ms", 600.0 ), 50.0, 10000.0 ) / 1000.0;
        }

        m_steady = steady;
        return true;
    }

    nlohmann::json steadyJson() const
    {
        return { { "mode", m_steady.mode == KOPENAPI_STEADY::OFF ? "off" : m_steady.mode == KOPENAPI_STEADY::FILTER ? "filter" : "timed" },
                 { "smooth_ms", std::lround( m_steady.smoothSeconds * 1000 ) },
                 { "duration_ms", std::lround( m_steady.durationSeconds * 1000 ) } };
    }

private:
    /// @brief Steadycam: the recorded view eases to where it was put instead of jumping
    void steady( wxTopLevelWindow* aWindow, std::function<void()>& aRestore )
    {
        wxWindow* canvas = m_canvas ? m_canvas.get() : KopenapiFindCanvas( aWindow );

        if( !canvas )
            return;

        const auto now = CLOCK::now();
        double     dt = m_steadyLast == CLOCK::time_point() ? 1.0
                                                            : std::chrono::duration<double>( now - m_steadyLast ).count();
        m_steadyLast = now;

        if( m_videoClock && dt < 1.0 )
            dt = 1.0 / m_settings.fps;   // video time: one frame per step

        for( const KOPENAPI_CANVAS_STEADY& step : KOPENAPI_REGISTRY::Get().CanvasSteadies() )
        {
            if( step( canvas, std::min( dt, 1.0 ), m_steady, KopenapiAnimating(), aRestore ) )
                break;
        }
    }

    bool capture( wxTopLevelWindow* aWindow, wxImage& aImage )
    {
        // steadycam: this frame is drawn from the eased view, the real one comes back after it
        std::function<void()> restore;
        steady( aWindow, restore );

        struct RESTORE
        {
            std::function<void()>& fn;
            ~RESTORE()
            {
                if( fn )
                    fn();
            }
        } restoreAfter{ restore };

        if( m_source == "canvas" && m_canvas && m_renderSize.x > 0 )
            return renderSharp( aImage );

        if( m_source == "canvas" )
            return m_canvas && KopenapiCaptureCanvas( m_canvas, aImage );

        // the whole window: toolbars and panels change rarely, the canvases every frame — the
        // window is drawn again every m_chromeMs (or on a new size), the canvases pasted fresh
        const auto now = CLOCK::now();
        const bool stale = !m_chrome.IsOk() || aWindow->GetClientSize() != m_chromeSize
                           || now - m_chromeAt >= std::chrono::milliseconds( m_chromeMs );

        if( stale )
        {
            if( !KopenapiCaptureWindow( aWindow, m_chrome ) )
                return false;

            m_chromeAt = now;
            m_chromeSize = aWindow->GetClientSize();
            aImage = m_chrome.Copy();
            m_windowMs += std::chrono::duration<double, std::milli>( CLOCK::now() - now ).count();
            m_windowFrames++;
            return true;
        }

        aImage = m_chrome.Copy();
        KopenapiPasteCanvases( aWindow, aImage );
        m_pasteMs += std::chrono::duration<double, std::milli>( CLOCK::now() - now ).count();
        m_pasteFrames++;
        return true;
    }

    int64_t currentIndex() const
    {
        if( m_videoClock )
            return m_lastIndex + 1;   // every frame the next one

        CLOCK::duration elapsed = ( m_paused ? m_pauseStart : CLOCK::now() ) - m_start - m_pausedTotal;
        return int64_t( std::chrono::duration<double>( elapsed ).count() * m_settings.fps );
    }

    void enqueue( const wxImage& aImage, int64_t aIndex )
    {
        // raw copy here: wxImage data must not be shared across threads
        FRAME frame;
        frame.index = aIndex;
        frame.width = aImage.GetWidth();
        frame.height = aImage.GetHeight();
        frame.rgb.assign( aImage.GetData(), aImage.GetData() + size_t( frame.width ) * frame.height * 3 );
        m_lastIndex = aIndex;

        {
            std::lock_guard<std::mutex> lock( m_mutex );
            m_captured++;

            if( m_queue.size() >= 3 )
            {
                m_queue.pop_front();
                m_dropped++;
            }

            m_queue.push_back( std::move( frame ) );
        }

        m_cv.notify_one();
    }

    void encodeLoop()
    {
        const size_t outBytes = size_t( m_settings.width ) * m_settings.height * 3;

        for( ;; )
        {
            FRAME frame;

            {
                std::unique_lock<std::mutex> lock( m_mutex );
                m_cv.wait( lock, [this]() { return !m_queue.empty() || m_finishing; } );

                if( m_queue.empty() )
                    break;

                frame = std::move( m_queue.front() );
                m_queue.pop_front();
            }

            m_cv.notify_all();   // a synced frame may be waiting for room

            std::string error;
            bool        ok = true;

            if( !m_letterbox && frame.width == m_settings.width && frame.height == m_settings.height )
            {
                ok = m_encoder->Write( frame.rgb.data(), frame.index, error );
            }
            else if( m_letterbox )
            {
                // the encoder letterboxes; only frames whose size changed (window resized) are
                // scaled here, to the input size it was opened with
                const int inW = m_settings.inputWidth, inH = m_settings.inputHeight;

                if( frame.width == inW && frame.height == inH )
                {
                    ok = m_encoder->Write( frame.rgb.data(), frame.index, error );
                }
                else
                {
                    m_fitted.resize( size_t( inW ) * inH * 3 );
                    m_scaler.FitRgb( frame.rgb.data(), frame.width, frame.height, frame.width * 3, m_fitted.data(), inW, inH,
                                     inW * 3, KOPENAPI_SCALE_QUALITY::HIGH, 0x121216 );
                    ok = m_encoder->Write( m_fitted.data(), frame.index, error );
                }
            }
            else
            {
                m_fitted.resize( outBytes );
                m_scaler.ScaleRgb( frame.rgb.data(), frame.width, frame.height, frame.width * 3, m_fitted.data(),
                                   m_settings.width, m_settings.height, m_settings.width * 3, KOPENAPI_SCALE_QUALITY::HIGH );
                ok = m_encoder->Write( m_fitted.data(), frame.index, error );
            }

            std::lock_guard<std::mutex> lock( m_mutex );

            if( ok )
            {
                m_written++;
            }
            else if( m_error.empty() )
            {
                m_error = error;
                m_queue.clear();
            }
        }

        std::string error;

        if( !m_encoder->Close( m_endIndex, error ) )
        {
            std::lock_guard<std::mutex> lock( m_mutex );

            if( m_error.empty() )
                m_error = error;
        }
    }

    std::unique_ptr<KOPENAPI_VIDEO_ENCODER> m_encoder;
    std::unique_ptr<RECORD_TIMER>           m_timer;
    KOPENAPI_VIDEO_SETTINGS                 m_settings;
    std::string                             m_source;
    std::string                             m_windowTitle;
    wxWeakRef<wxTopLevelWindow>             m_window;
    wxWeakRef<wxWindow>                     m_canvas;
    double                                  m_maxSeconds = 600;
    bool                                    m_letterbox = false;   ///< fixed profile frame
    wxSize                                  m_renderSize;          ///< canvas rendered offscreen at this size
    int64_t                                 m_startedUnixMs = 0;   ///< wall clock of frame 0 (aligning parallel recordings)
    int                                     m_tier = 0;            ///< TIER of the offscreen canvas path
    std::string                             m_qualityMode = "adaptive";
    std::string                             m_capturePath = "auto"; ///< auto / gpu / cpu
    KOPENAPI_FRAME_SCALER                   m_scaler;               ///< encoder thread: fits frames (tables kept)
    wxImage                                 m_chrome;               ///< source window: the last whole-window picture
    CLOCK::time_point                       m_chromeAt;
    wxSize                                  m_chromeSize;
    int                                     m_chromeMs = 200;       ///< source window: redraw the window this often
    double                                  m_windowMs = 0, m_pasteMs = 0;   ///< source window: time drawing the window / pasting canvases
    int                                     m_windowFrames = 0, m_pasteFrames = 0;
    std::vector<uint8_t>                    m_fitted;               ///< encoder thread: the fitted frame
    std::string                             m_clockMode = "auto";   ///< auto / video / wall
    bool                                    m_videoClock = false;   ///< frames on video time now
    int                                     m_clockSkipped = 0;     ///< wall-clock frames passed by since the last one
    CLOCK::time_point                       m_lastFrameWall;        ///< when the last frame was taken
    CLOCK::time_point                       m_videoEpoch;           ///< video clock: when frame 0 of the schedule was due
    int64_t                                 m_videoFrames = 0;      ///< video clock: frames taken on this schedule
    std::deque<double>                      m_clockMs;              ///< recent capture times (auto clock)
    nlohmann::json                          m_clockLog = nlohmann::json::array();
    KOPENAPI_STEADY                         m_steady;              ///< steadycam mode and timing
    CLOCK::time_point                       m_steadyLast;
    std::deque<double>                      m_tierMs;              ///< recent frame times on this tier
    int                                     m_tierFrames = 0;
    nlohmann::json                          m_tierLog = nlohmann::json::array();
    CLOCK::time_point                       m_lastSync;            ///< last frame fed on video time
    bool                                    m_rebase = false;      ///< timer continues the video clock
    CLOCK::time_point                       m_start;
    CLOCK::time_point                       m_pauseStart;
    CLOCK::duration                         m_pausedTotal{};
    bool                                    m_paused = false;
    bool                                    m_active = false;
    int64_t                                 m_lastIndex = -1;
    int64_t                                 m_endIndex = 0;
    double                                  m_captureMsTotal = 0;
    double                                  m_captureMsMax = 0;
    nlohmann::json                          m_markers = nlohmann::json::array();
    std::string                             m_stopReason;

    std::thread                             m_thread;
    std::mutex                              m_mutex;
    std::condition_variable                 m_cv;
    std::deque<FRAME>                       m_queue;      ///< guarded by m_mutex
    bool                                    m_finishing = false;
    int64_t                                 m_captured = 0;
    int64_t                                 m_written = 0;
    int64_t                                 m_dropped = 0;
    std::string                             m_error;
};


void RECORD_TIMER::Notify()
{
    m_recorder.Tick();
}



/// Recordings running at the same time (e.g. the board editor and the 3D viewer for a
/// side-by-side scene); a synced animation step is captured by every one of them
class RECORDINGS
{
public:
    static RECORDINGS& Get()
    {
        static RECORDINGS recordings;
        return recordings;
    }

    std::vector<RECORDER*> Active()
    {
        std::vector<RECORDER*> out;

        for( auto& [id, r] : m_list )
        {
            if( r->Active() )
                out.push_back( r.get() );
        }

        return out;
    }

    KOPENAPI_RESULT Start( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
    {
        // finished recordings make room; their last state was answered by record_stop
        for( auto it = m_list.begin(); it != m_list.end(); )
            it = it->second->Active() ? std::next( it ) : m_list.erase( it );

        std::string id = aArgs.value( "id", std::string() );

        if( id.empty() )
            id = "rec" + std::to_string( ++m_counter );

        if( m_list.count( id ) )
            return KOPENAPI_RESULT::Error( 409, "a recording with id '" + id + "' is running" );

        // one video clock for synced animations: every running recording at the same frame rate
        if( !Active().empty() && aArgs.contains( "fps" ) && aArgs["fps"].get<int>() != Active().front()->Fps() )
        {
            return KOPENAPI_RESULT::Error( 409, "fps must match the running recording(s): "
                                                        + std::to_string( Active().front()->Fps() ) );
        }

        auto            recorder = std::make_unique<RECORDER>();
        nlohmann::json  args = aArgs;

        if( !Active().empty() && !aArgs.contains( "fps" ) )
            args["fps"] = Active().front()->Fps();

        KOPENAPI_RESULT result = recorder->Start( aCtx, args );

        if( result.status != 200 )
            return result;

        result.body["id"] = id;
        m_list[id] = std::move( recorder );
        m_last = id;
        return result;
    }

    /// @brief By id, else the only running one; aAll: every running one when no id is given
    std::vector<std::pair<std::string, RECORDER*>> Pick( const nlohmann::json& aArgs, bool aAll, std::string& aError )
    {
        std::vector<std::pair<std::string, RECORDER*>> out;
        const std::string                               id = aArgs.value( "id", std::string() );

        if( !id.empty() )
        {
            if( m_list.count( id ) )
                out.push_back( { id, m_list[id].get() } );
            else
                aError = "no recording with id '" + id + "'";

            return out;
        }

        for( auto& [rid, r] : m_list )
        {
            if( r->Active() )
                out.push_back( { rid, r.get() } );
        }

        if( out.empty() && m_list.count( m_last ) )
            out.push_back( { m_last, m_list[m_last].get() } );

        if( out.size() > 1 && !aAll )
        {
            aError = "several recordings are running: give id";
            out.clear();
        }

        return out;
    }

    bool Synced( int* aFps )
    {
        for( RECORDER* r : Active() )
        {
            if( r->Synced( aFps ) )
                return true;
        }

        return false;
    }

    void RecordFrame()
    {
        for( RECORDER* r : Active() )
            r->RecordFrame();
    }

    /// @brief Let aSeconds of video pass in every running recording: on video time the frames are
    /// taken here one after another (as fast as capture goes), on wall time the timer takes them;
    /// the UI keeps running. Without a recording: a plain pause with the UI alive.
    nlohmann::json Wait( double aSeconds )
    {
        const auto started = CLOCK::now();
        auto       recorders = Active();

        if( recorders.empty() )
        {
            while( std::chrono::duration<double>( CLOCK::now() - started ).count() < aSeconds )
            {
                KopenapiKeepUiAlive();
                std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
            }

            return { { "recording", false }, { "wall_s", aSeconds } };
        }

        std::vector<std::pair<RECORDER*, int64_t>> targets;

        for( RECORDER* r : recorders )
            targets.push_back( { r, r->LastIndex() + std::max<int64_t>( 1, std::llround( aSeconds * r->Fps() ) ) } );

        // a guard: a recording that stops taking frames (window closed) ends the wait
        const double limit = std::max( 30.0, aSeconds * 30 );

        for( ;; )
        {
            bool pending = false;

            for( auto& [r, target] : targets )
            {
                if( !r->Active() || r->LastIndex() >= target )
                    continue;

                pending = true;

                if( r->VideoClock() )
                    r->advanceVideo();
            }

            if( !pending || std::chrono::duration<double>( CLOCK::now() - started ).count() > limit )
                break;

            KopenapiKeepUiAlive();
            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        }

        return { { "recording", true }, { "video_s", aSeconds },
                 { "wall_s", std::chrono::duration<double>( CLOCK::now() - started ).count() } };
    }

    int ScreenSteps()
    {
        int steps = 1;

        for( RECORDER* r : Active() )
            steps = std::max( steps, r->ScreenSteps() );

        return steps;
    }

    void Shutdown()
    {
        for( auto& [id, r] : m_list )
            r->Shutdown();
    }

private:
    std::map<std::string, std::unique_ptr<RECORDER>> m_list;
    std::string                                      m_last;
    int                                              m_counter = 0;
};

} // namespace


static std::atomic<bool> s_animating{ false };


void KopenapiSetAnimating( bool aAnimating )
{
    s_animating = aAnimating;
}


bool KopenapiAnimating()
{
    return s_animating;
}


void KopenapiRecorderShutdown()
{
    RECORDINGS::Get().Shutdown();
}


bool KopenapiRecordingSync( int* aFps )
{
    return RECORDINGS::Get().Synced( aFps );
}


void KopenapiRecordFrame()
{
    RECORDINGS::Get().RecordFrame();
}


int KopenapiRecordingScreenSteps()
{
    return RECORDINGS::Get().ScreenSteps();
}


bool KopenapiRecordingRenderSize( wxWindow* aCanvas, int* aWidth, int* aHeight )
{
    for( RECORDER* r : RECORDINGS::Get().Active() )
    {
        if( r->RenderSize( aCanvas, aWidth, aHeight ) )
            return true;
    }

    return false;
}


static KOPENAPI_RESULT h_record_start( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return RECORDINGS::Get().Start( aCtx, aArgs );
}


/// @brief Runs aDo on the picked recording(s): one answer as is, several as {recordings: [...]}
static KOPENAPI_RESULT forPicked( const nlohmann::json& aArgs, bool aAll,
                                  const std::function<KOPENAPI_RESULT( RECORDER& )>& aDo )
{
    std::string error;
    auto        picked = RECORDINGS::Get().Pick( aArgs, aAll, error );

    if( picked.empty() )
        return KOPENAPI_RESULT::Error( error.empty() ? 409 : 400, error.empty() ? "no recording is running" : error );

    if( picked.size() == 1 )
    {
        KOPENAPI_RESULT r = aDo( *picked.front().second );

        if( r.status == 200 && r.body.is_object() )
            r.body["id"] = picked.front().first;

        return r;
    }

    nlohmann::json all = nlohmann::json::array();
    int            status = 200;

    for( auto& [id, rec] : picked )
    {
        KOPENAPI_RESULT r = aDo( *rec );
        nlohmann::json  body = r.body.is_object() ? r.body : nlohmann::json::object();
        body["id"] = id;
        all.push_back( body );

        if( r.status != 200 )
            status = r.status;
    }

    KOPENAPI_RESULT out = KOPENAPI_RESULT::Ok( { { "recordings", all } } );
    out.status = status;
    return out;
}


static KOPENAPI_RESULT h_record_stop( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    return forPicked( aArgs, true, []( RECORDER& r ) { return r.Stop(); } );
}


static KOPENAPI_RESULT h_record_status( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    return forPicked( aArgs, true, []( RECORDER& r ) { return KOPENAPI_RESULT::Ok( r.status() ); } );
}


static KOPENAPI_RESULT h_record_pause( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    return forPicked( aArgs, true, []( RECORDER& r ) { return r.Pause( true ); } );
}


static KOPENAPI_RESULT h_record_resume( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    return forPicked( aArgs, true, []( RECORDER& r ) { return r.Pause( false ); } );
}


static KOPENAPI_RESULT h_record_wait( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    const double seconds = aArgs.value( "seconds", 1.0 );

    if( seconds < 0 || seconds > 600 )
        return KOPENAPI_RESULT::Error( 400, "seconds: 0..600" );

    return KOPENAPI_RESULT::Ok( RECORDINGS::Get().Wait( seconds ) );
}


static KOPENAPI_RESULT h_record_steadycam( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    if( RECORDINGS::Get().Active().empty() )
        return KOPENAPI_RESULT::Error( 409, "no recording is running" );

    nlohmann::json spec = aArgs;
    spec.erase( "id" );

    return forPicked( aArgs, true,
                      [&]( RECORDER& r )
                      {
                          std::string error;

                          if( !r.SetSteady( spec.empty() ? nlohmann::json( true ) : spec, error ) )
                              return KOPENAPI_RESULT::Error( 400, error );

                          return KOPENAPI_RESULT::Ok( r.status() );
                      } );
}


static KOPENAPI_RESULT h_record_marker( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    const std::string text = aArgs.value( "text", std::string() );
    return forPicked( aArgs, true, [&]( RECORDER& r ) { return r.Marker( text ); } );
}


KOPENAPI_REGISTER( "record_start",
                   "Start a video recording (screen capture, screencast, demo video) of a KiCad window or "
                   "of its canvas alone: offscreen, no screen-recording permission, covered windows still "
                   "record; several at once (answers an id; started_unix_ms aligns them); MP4 (native encoder: macOS VideoToolbox; else ffmpeg) or GIF; GUI only",
                   R"json({"type":"object","properties":{
                        "source":{"type":"string","enum":["window","canvas"],"default":"window","description":"the whole window (toolbars, panels) or the editor's drawing area only"},
                        "window":{"type":"string","description":"window id or title glob from window_list; default the active window"},
                        "path":{"type":"string","description":"absolute .mp4 / .gif path; default <project>/recordings/kicad-<time>.<format>"},
                        "format":{"type":"string","enum":["mp4","gif"],"default":"mp4"},
                        "codec":{"type":"string","enum":["h264","hevc"],"default":"h264"},
                        "encoder":{"type":"string","enum":["auto","native","ffmpeg"],"default":"auto"},
                        "fps":{"type":"integer","minimum":1,"maximum":60,"description":"default 20 for a window, 30 for a canvas"},
                        "antialias":{"type":"string","enum":["ssaa2","none"],"default":"ssaa2","description":"canvases rendered at the video size (3D viewer): 2x supersampling on the GPU"},
                        "chrome_ms":{"type":"integer","default":200,"minimum":0,"maximum":5000,"description":"source window: the window itself (toolbars, panels) is drawn again this often, the canvases every frame; 0 = every frame"},
                        "capture":{"type":"string","enum":["auto","gpu","cpu"],"default":"auto","description":"how frames are taken: gpu (composed and scaled on the GPU, read once; canvas source with a 4k / 1080p / custom profile), cpu (one read of the composed frame, SIMD scaling on the encoder thread), auto (gpu where it works)"},
                        "clock":{"type":"string","enum":["auto","video","wall"],"default":"auto","description":"auto / video: video time — every frame the next one, never faster than real time; when capture is slower the video runs longer instead of skipping frames (smooth); record_wait paces scripts on it. wall: frames follow the wall clock, skipped when capture is slower"},
                        "steadycam":{"description":"how the recorded view goes where it is put (API jumps, view changes): timed (default: a smooth path that arrives exactly duration_ms later), filter (eases through a smoothing filter, arrives later), off; true = timed, false = off, or {mode, smooth_ms (filter, default 450), duration_ms (timed, default 600)}; API animations pass as they are","oneOf":[{"type":"boolean"},{"type":"string","enum":["timed","filter","off"]},{"type":"object","properties":{"mode":{"type":"string","enum":["timed","filter","off"]},"enabled":{"type":"boolean"},"smooth_ms":{"type":"number"},"duration_ms":{"type":"number"}}}]},
                        "quality_mode":{"type":"string","enum":["adaptive","max","fast"],"default":"adaptive","description":"3D viewer canvas: adaptive starts at full quality and steps down (gpu 2x -> gpu -> window readback scaled) when frames exceed the budget or the GPU refuses, back up with room to spare; max keeps the best the GPU can do; fast uses the readback"},
                        "profile":{"type":"string","enum":["source","1080p","4k","custom"],"default":"source","description":"4k (3840x2160) / 1080p (1920x1080) / custom (width x height, e.g. one half of a side-by-side): every frame scaled (bicubic up, high-quality down) to fit, centred; the 3D viewer renders at that size; source: the window's size capped by max_width"},
                        "width":{"type":"integer","description":"profile custom"},
                        "height":{"type":"integer","description":"profile custom"},
                        "max_width":{"type":"integer","default":1920,"description":"source profile: scale down wider frames"},
                        "quality":{"type":"integer","minimum":0,"maximum":10,"default":5},
                        "max_seconds":{"type":"number","default":600,"description":"stops by itself after this long"},
                        "id":{"type":"string","description":"name for this recording (default rec<N>); several can run at once, e.g. the board editor and the 3D viewer for a side-by-side scene: same fps, one video clock for animations"}}})json"_json,
                   true, h_record_start, 60 );

KOPENAPI_REGISTER( "record_stop",
                   "Stop the video recording and finish the file: path, duration, frames captured / "
                   "encoded / dropped, capture time, file size, markers file; id, else every running one",
                   R"json({"type":"object","properties":{"id":{"type":"string","description":"default: every running recording"}}})json"_json, true, h_record_stop, 120 );

KOPENAPI_REGISTER( "record_status",
                   "Video recording state: running / paused, path, size, fps, encoder, duration, frames "
                   "captured / encoded / dropped, capture time per frame, stop reason, error; id, else every "
                   "running one (several: {recordings: [...]})",
                   R"json({"type":"object","properties":{"id":{"type":"string","description":"default: every running recording"}}})json"_json, true, h_record_status );

KOPENAPI_REGISTER( "record_pause", "Pause the video recording(s) (the paused time is cut out); id, else all",
                   R"json({"type":"object","properties":{"id":{"type":"string","description":"default: every running recording"}}})json"_json, true, h_record_pause );

KOPENAPI_REGISTER( "record_resume", "Resume paused video recording(s); id, else all",
                   R"json({"type":"object","properties":{"id":{"type":"string","description":"default: every running recording"}}})json"_json, true, h_record_resume );

KOPENAPI_REGISTER( "record_wait",
                   "Pause a script by video time: returns once every running recording has taken this many "
                   "seconds of video (on video time — capture slower than real time — the frames are taken "
                   "here one after another, so the video keeps its pace); without a recording a plain pause; "
                   "the UI keeps running. Answers the video and wall seconds",
                   R"json({"type":"object","properties":{
                        "seconds":{"type":"number","default":1,"minimum":0,"maximum":600}}})json"_json,
                   true, h_record_wait, 660 );

KOPENAPI_REGISTER( "record_steadycam",
                   "Change a running recording's steadycam (how the recorded view goes where it is put): "
                   "mode timed (a smooth path that arrives exactly duration_ms later), filter (eases "
                   "through a smoothing filter, arrives later) or off; id, else every running recording",
                   R"json({"type":"object","properties":{
                        "id":{"type":"string"},
                        "mode":{"type":"string","enum":["timed","filter","off"],"default":"timed"},
                        "smooth_ms":{"type":"number","default":450},
                        "duration_ms":{"type":"number","default":600}}})json"_json,
                   true, h_record_steadycam );

KOPENAPI_REGISTER( "record_marker",
                   "Mark the current time of the video recording with a text (chapters / captions for "
                   "editing a demo; written next to the video as <video>.markers.json)",
                   R"json({"type":"object","required":["text"],"properties":{"text":{"type":"string"},
                        "id":{"type":"string","description":"default: every running recording"}}})json"_json,
                   true, h_record_marker );
