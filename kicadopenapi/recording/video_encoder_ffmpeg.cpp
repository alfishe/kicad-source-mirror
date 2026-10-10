/// @file video_encoder_ffmpeg.cpp
/// @brief Recording through an external ffmpeg: raw RGB24 frames on its stdin, constant frame
/// rate (a frame that stays on screen is repeated). Works on every OS where ffmpeg is installed.
#include "video_encoder.h"

#include "../platform/platform.h"

#include <algorithm>
#include <vector>

namespace kp = kopenapi::platform;


namespace
{

class FFMPEG_ENCODER : public KOPENAPI_VIDEO_ENCODER
{
public:
    explicit FFMPEG_ENCODER( std::filesystem::path aExe ) : m_exe( std::move( aExe ) ) {}

    ~FFMPEG_ENCODER() override
    {
        std::string ignored;
        finish( ignored );
    }

    std::string Name() const override { return "ffmpeg"; }

    bool Open( const KOPENAPI_VIDEO_SETTINGS& aSettings, std::string& aError ) override
    {
        m_settings = aSettings;

        const int inW = aSettings.inputWidth ? aSettings.inputWidth : aSettings.width;
        const int inH = aSettings.inputHeight ? aSettings.inputHeight : aSettings.height;
        m_frameBytes = size_t( inW ) * inH * 3;

        const std::string size = std::to_string( inW ) + "x" + std::to_string( inH );
        const std::string W = std::to_string( aSettings.width ), H = std::to_string( aSettings.height );

        // a fixed output frame: ffmpeg fits the input in (lanczos), letterboxed on a dark field
        const std::string fit = inW == aSettings.width && inH == aSettings.height
                                        ? std::string()
                                        : "scale=" + W + ":" + H + ":force_original_aspect_ratio=decrease:flags=lanczos,pad="
                                                  + W + ":" + H + ":(ow-iw)/2:(oh-ih)/2:color=0x121216";
        const std::string fps = std::to_string( aSettings.fps );
        std::vector<std::string> argv = { m_exe.string(), "-hide_banner", "-loglevel", "error", "-y",
                                          "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", size, "-r", fps, "-i", "-" };

        if( aSettings.format == "gif" )
        {
            argv.insert( argv.end(), { "-vf", ( fit.empty() ? "" : fit + "," )
                                                       + "split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=none",
                                       "-loop", "0" } );
        }
        else
        {
            // quality 0..10 -> crf 34..18
            const int crf = 34 - std::clamp( aSettings.quality, 0, 10 ) * 16 / 10;

            if( !fit.empty() )
                argv.insert( argv.end(), { "-vf", fit } );
            argv.insert( argv.end(), { "-c:v", aSettings.codec == "hevc" ? "libx265" : "libx264", "-preset", "veryfast",
                                       "-crf", std::to_string( crf ), "-pix_fmt", "yuv420p", "-movflags", "+faststart" } );

            if( aSettings.codec == "hevc" )
                argv.insert( argv.end(), { "-tag:v", "hvc1" } );
        }

        argv.push_back( aSettings.path.string() );

        const std::filesystem::path logDir = kp::TempRoot() / "kicad" / "openapi-logs";
        std::error_code             ec;
        std::filesystem::create_directories( logDir, ec );
        m_log = logDir / ( "ffmpeg-" + std::to_string( kp::CurrentPid() ) + ".log" );

        m_process = kp::SpawnWithStdinPipe( argv, m_log, aError );
        return m_process.pid != 0;
    }

    bool Write( const uint8_t* aRgb, int64_t aIndex, std::string& aError ) override
    {
        // constant frame rate: the previous frame fills the slots up to this one
        if( !m_last.empty() )
        {
            while( m_written < aIndex )
            {
                if( !put( m_last.data(), aError ) )
                    return false;
            }
        }

        m_last.assign( aRgb, aRgb + m_frameBytes );
        return put( m_last.data(), aError );
    }

    bool Close( int64_t aEndIndex, std::string& aError ) override
    {
        while( !m_last.empty() && m_written < aEndIndex )
        {
            if( !put( m_last.data(), aError ) )
                break;
        }

        return finish( aError );
    }

private:
    bool put( const uint8_t* aFrame, std::string& aError )
    {
        if( !kp::WritePipe( m_process.stdinHandle, aFrame, m_frameBytes ) )
        {
            aError = "ffmpeg stopped taking frames (log: " + m_log.string() + ")";
            return false;
        }

        m_written++;
        return true;
    }

    bool finish( std::string& aError )
    {
        if( m_process.pid == 0 )
            return true;

        kp::ClosePipe( m_process.stdinHandle );
        const int code = kp::WaitProcess( m_process.pid, 60000 );

        if( code == -1 )
            kp::KillProcess( m_process.pid );

        m_process = {};

        if( code != 0 )
        {
            aError = "ffmpeg failed (exit " + std::to_string( code ) + ", log: " + m_log.string() + ")";
            return false;
        }

        return true;
    }

    std::filesystem::path   m_exe;
    std::filesystem::path   m_log;
    KOPENAPI_VIDEO_SETTINGS m_settings;
    kp::PipeProcess         m_process;
    size_t                  m_frameBytes = 0;
    int64_t                 m_written = 0;
    std::vector<uint8_t>    m_last;
};

} // namespace


std::unique_ptr<KOPENAPI_VIDEO_ENCODER> KopenapiFfmpegVideoEncoder( std::string& aError )
{
    const std::filesystem::path exe = kp::FindExecutable( "ffmpeg" );

    if( exe.empty() )
    {
        aError = "ffmpeg not found on PATH (install it, or use the native encoder: format mp4)";
        return nullptr;
    }

    return std::make_unique<FFMPEG_ENCODER>( exe );
}
