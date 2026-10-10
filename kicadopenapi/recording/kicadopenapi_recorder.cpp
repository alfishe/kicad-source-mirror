/// @file kicadopenapi_recorder.cpp
/// @brief Video recording of a KiCad window or of its canvas: record_start, record_stop,
/// record_status, record_pause, record_resume, record_marker.
///
/// A timer on the main thread captures frames (offscreen, see kicadopenapi_capture.h) into a short
/// queue; an encoder thread scales them to the video size and encodes. When capture or encoding
/// falls behind, frames are dropped (counted) and the previous frame stays on screen, so the video
/// keeps real time. GUI only (registered GUI-only: headless answers 501).
#include "kicadopenapi_recorder.h"
#include "video_encoder.h"

#include <kicadopenapi_capture.h>
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
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
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
    static RECORDER& Get()
    {
        static RECORDER recorder;
        return recorder;
    }

    bool Active() const { return m_active; }

    KOPENAPI_RESULT Start( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
    {
        if( m_active )
            return KOPENAPI_RESULT::Error( 409, "a recording is running (record_stop)" );

        const std::string source = aArgs.value( "source", std::string( "window" ) );

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

        // first frame decides the video size
        wxImage first;

        if( !capture( window, first ) )
            return KOPENAPI_RESULT::Error( 501, "this platform cannot draw windows offscreen (e.g. Wayland)" );

        const int maxWidth = std::clamp( aArgs.value( "max_width", 1920 ), 160, 7680 );
        double    scale = std::min( 1.0, double( maxWidth ) / first.GetWidth() );
        settings.width = std::max( 2, int( first.GetWidth() * scale ) & ~1 );
        settings.height = std::max( 2, int( first.GetHeight() * scale ) & ~1 );

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

        m_thread = std::thread( [this]() { encodeLoop(); } );
        enqueue( first, 0 );

        if( !m_timer )
            m_timer = std::make_unique<RECORD_TIMER>( *this );

        m_timer->Start( std::max( 1, 1000 / settings.fps ) );

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
        const int64_t end = std::max<int64_t>( currentIndex() + 1, m_lastIndex + 1 );

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
                 { "error", m_error } };
    }

    void Tick()
    {
        if( !m_active || m_paused )
            return;

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

        const int64_t index = currentIndex();

        if( index <= m_lastIndex || m_window->IsIconized() )
            return;

        const auto t0 = CLOCK::now();
        wxImage    image;

        if( !capture( m_window, image ) )
            return;

        const double ms = std::chrono::duration<double, std::milli>( CLOCK::now() - t0 ).count();
        m_captureMsTotal += ms;
        m_captureMsMax = std::max( m_captureMsMax, ms );
        enqueue( image, index );
    }

    void Shutdown()
    {
        if( m_active )
            Stop( "KiCad closing" );
    }

private:
    bool capture( wxTopLevelWindow* aWindow, wxImage& aImage )
    {
        if( m_source == "canvas" )
            return m_canvas && KopenapiCaptureCanvas( m_canvas, aImage );

        return KopenapiCaptureWindow( aWindow, aImage );
    }

    int64_t currentIndex() const
    {
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

            std::string error;
            bool        ok = true;

            if( frame.width == m_settings.width && frame.height == m_settings.height )
            {
                ok = m_encoder->Write( frame.rgb.data(), frame.index, error );
            }
            else
            {
                wxImage image( frame.width, frame.height, frame.rgb.data(), true );
                wxImage scaled = image.Scale( m_settings.width, m_settings.height, wxIMAGE_QUALITY_BILINEAR );
                std::vector<uint8_t> rgb( scaled.GetData(), scaled.GetData() + outBytes );
                ok = m_encoder->Write( rgb.data(), frame.index, error );
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



} // namespace


void KopenapiRecorderShutdown()
{
    RECORDER::Get().Shutdown();
}


static KOPENAPI_RESULT h_record_start( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return RECORDER::Get().Start( aCtx, aArgs );
}


static KOPENAPI_RESULT h_record_stop( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    return RECORDER::Get().Stop();
}


static KOPENAPI_RESULT h_record_status( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    return KOPENAPI_RESULT::Ok( RECORDER::Get().status() );
}


static KOPENAPI_RESULT h_record_pause( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    return RECORDER::Get().Pause( true );
}


static KOPENAPI_RESULT h_record_resume( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    return RECORDER::Get().Pause( false );
}


static KOPENAPI_RESULT h_record_marker( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    return RECORDER::Get().Marker( aArgs.value( "text", std::string() ) );
}


KOPENAPI_REGISTER( "record_start",
                   "Start a video recording (screen capture, screencast, demo video) of a KiCad window or "
                   "of its canvas alone: offscreen, no screen-recording permission, covered windows still "
                   "record; MP4 (native encoder: macOS VideoToolbox; else ffmpeg) or GIF; GUI only",
                   R"json({"type":"object","properties":{
                        "source":{"type":"string","enum":["window","canvas"],"default":"window","description":"the whole window (toolbars, panels) or the editor's drawing area only"},
                        "window":{"type":"string","description":"window id or title glob from window_list; default the active window"},
                        "path":{"type":"string","description":"absolute .mp4 / .gif path; default <project>/recordings/kicad-<time>.<format>"},
                        "format":{"type":"string","enum":["mp4","gif"],"default":"mp4"},
                        "codec":{"type":"string","enum":["h264","hevc"],"default":"h264"},
                        "encoder":{"type":"string","enum":["auto","native","ffmpeg"],"default":"auto"},
                        "fps":{"type":"integer","minimum":1,"maximum":60,"description":"default 20 for a window, 30 for a canvas"},
                        "max_width":{"type":"integer","default":1920,"description":"scale down wider frames"},
                        "quality":{"type":"integer","minimum":0,"maximum":10,"default":5},
                        "max_seconds":{"type":"number","default":600,"description":"stops by itself after this long"}}})json"_json,
                   true, h_record_start, 60 );

KOPENAPI_REGISTER( "record_stop",
                   "Stop the video recording and finish the file: path, duration, frames captured / "
                   "encoded / dropped, capture time, file size, markers file",
                   R"json({"type":"object","properties":{}})json"_json, true, h_record_stop, 120 );

KOPENAPI_REGISTER( "record_status",
                   "Video recording state: running / paused, path, size, fps, encoder, duration, frames "
                   "captured / encoded / dropped, capture time per frame, stop reason, error",
                   R"json({"type":"object","properties":{}})json"_json, true, h_record_status );

KOPENAPI_REGISTER( "record_pause", "Pause the video recording (the paused time is cut out)",
                   R"json({"type":"object","properties":{}})json"_json, true, h_record_pause );

KOPENAPI_REGISTER( "record_resume", "Resume a paused video recording",
                   R"json({"type":"object","properties":{}})json"_json, true, h_record_resume );

KOPENAPI_REGISTER( "record_marker",
                   "Mark the current time of the video recording with a text (chapters / captions for "
                   "editing a demo; written next to the video as <video>.markers.json)",
                   R"json({"type":"object","required":["text"],"properties":{"text":{"type":"string"}}})json"_json,
                   true, h_record_marker );
