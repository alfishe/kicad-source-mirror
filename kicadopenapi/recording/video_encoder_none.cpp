/// @file video_encoder_none.cpp
/// @brief No native video encoder on this OS yet (Windows: Media Foundation planned); recordings
/// go through ffmpeg.
#if !defined( __APPLE__ )

#include "video_encoder.h"


std::unique_ptr<KOPENAPI_VIDEO_ENCODER> KopenapiNativeVideoEncoder( const KOPENAPI_VIDEO_SETTINGS& )
{
    return nullptr;
}

#endif
