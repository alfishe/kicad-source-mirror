/// @file video_encoder.h
/// @brief Video encoders for recordings: RGB24 frames in, a file out. One implementation per
/// backend: the OS's native encoder (video_encoder_apple.mm; none elsewhere yet) and an
/// external ffmpeg fed through a pipe (every OS). No wx dependency; called from one thread.
#ifndef KICADOPENAPI_VIDEO_ENCODER_H
#define KICADOPENAPI_VIDEO_ENCODER_H

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>


struct KOPENAPI_VIDEO_SETTINGS
{
    std::filesystem::path path;
    int                   width = 0;            ///< even
    int                   height = 0;           ///< even
    int                   fps = 30;
    std::string           format = "mp4";       ///< mp4 | gif
    std::string           codec = "h264";       ///< h264 | hevc (mp4)
    int                   quality = 5;          ///< 0 (smallest) .. 10 (best)
};


class KOPENAPI_VIDEO_ENCODER
{
public:
    virtual ~KOPENAPI_VIDEO_ENCODER() = default;

    /// @brief Short name for status answers ("videotoolbox", "ffmpeg")
    virtual std::string Name() const = 0;

    /// @return false with aError set when the file cannot be started
    virtual bool Open( const KOPENAPI_VIDEO_SETTINGS& aSettings, std::string& aError ) = 0;

    /// @brief Adds a frame shown from frame slot aIndex (time = aIndex / fps) until the next one.
    /// @param aRgb width x height packed RGB24
    /// @param aIndex increasing; gaps mean the previous frame stays on screen
    /// @return false when the encoder failed (aError set)
    virtual bool Write( const uint8_t* aRgb, int64_t aIndex, std::string& aError ) = 0;

    /// @brief Finishes the file.
    /// @param aEndIndex the slot after the last one (the last frame lasts until then)
    /// @return false with aError set when the file could not be finished
    virtual bool Close( int64_t aEndIndex, std::string& aError ) = 0;
};


/// @brief The OS's native encoder for aSettings, or nullptr (none on this OS / format)
std::unique_ptr<KOPENAPI_VIDEO_ENCODER> KopenapiNativeVideoEncoder( const KOPENAPI_VIDEO_SETTINGS& aSettings );

/// @brief An encoder running an external ffmpeg, or nullptr with aError when ffmpeg is not found
std::unique_ptr<KOPENAPI_VIDEO_ENCODER> KopenapiFfmpegVideoEncoder( std::string& aError );

#endif
