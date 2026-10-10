/// @file video_encoder_apple.mm
/// @brief Native macOS recording: AVAssetWriter compresses H.264 / HEVC through VideoToolbox, MP4
/// written by the system, no external tools. Frames carry their own time (variable frame rate).
#if defined( __APPLE__ )

#include "video_encoder.h"

#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

#include <algorithm>
#include <unistd.h>


namespace
{

class APPLE_ENCODER : public KOPENAPI_VIDEO_ENCODER
{
public:
    ~APPLE_ENCODER() override
    {
        std::string ignored;
        Close( m_lastIndex + 1, ignored );
    }

    std::string Name() const override { return "videotoolbox"; }

    bool Open( const KOPENAPI_VIDEO_SETTINGS& aSettings, std::string& aError ) override
    {
        m_settings = aSettings;

        @autoreleasepool
        {
            NSURL* url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:aSettings.path.c_str()]];
            [[NSFileManager defaultManager] removeItemAtURL:url error:nil];   // the writer refuses to overwrite

            NSError* error = nil;
            AVAssetWriter* writer = [[AVAssetWriter alloc] initWithURL:url fileType:AVFileTypeMPEG4 error:&error];

            if( !writer )
            {
                aError = std::string( "AVAssetWriter: " ) + ( error ? [[error localizedDescription] UTF8String] : "failed" );
                return false;
            }

            // bits per pixel per frame 0.04 .. 0.24 by quality; UI content compresses well
            const double bpp = 0.04 + 0.02 * std::clamp( aSettings.quality, 0, 10 );
            const double bitrate = bpp * aSettings.width * aSettings.height * aSettings.fps;

            NSDictionary* compression = @{ AVVideoAverageBitRateKey: @( (long long) bitrate ),
                                           AVVideoMaxKeyFrameIntervalKey: @( aSettings.fps * 2 ),
                                           AVVideoAllowFrameReorderingKey: @NO };
            NSDictionary* color = @{ AVVideoColorPrimariesKey: AVVideoColorPrimaries_ITU_R_709_2,
                                     AVVideoTransferFunctionKey: AVVideoTransferFunction_ITU_R_709_2,
                                     AVVideoYCbCrMatrixKey: AVVideoYCbCrMatrix_ITU_R_709_2 };
            NSDictionary* video = @{ AVVideoCodecKey: aSettings.codec == "hevc" ? AVVideoCodecTypeHEVC : AVVideoCodecTypeH264,
                                     AVVideoWidthKey: @( aSettings.width ),
                                     AVVideoHeightKey: @( aSettings.height ),
                                     AVVideoCompressionPropertiesKey: compression,
                                     AVVideoColorPropertiesKey: color };

            AVAssetWriterInput* input = [[AVAssetWriterInput alloc] initWithMediaType:AVMediaTypeVideo outputSettings:video];
            input.expectsMediaDataInRealTime = YES;

            NSDictionary* buffers = @{ (id) kCVPixelBufferPixelFormatTypeKey: @( kCVPixelFormatType_32BGRA ),
                                       (id) kCVPixelBufferWidthKey: @( aSettings.width ),
                                       (id) kCVPixelBufferHeightKey: @( aSettings.height ) };
            AVAssetWriterInputPixelBufferAdaptor* adaptor =
                    [[AVAssetWriterInputPixelBufferAdaptor alloc] initWithAssetWriterInput:input
                                                               sourcePixelBufferAttributes:buffers];

            if( ![writer canAddInput:input] )
            {
                aError = "AVAssetWriter cannot take this video format";
                [adaptor release];
                [input release];
                [writer release];
                return false;
            }

            [writer addInput:input];

            if( ![writer startWriting] )
            {
                aError = std::string( "AVAssetWriter: " )
                         + ( writer.error ? [[writer.error localizedDescription] UTF8String] : "cannot start" );
                [adaptor release];
                [input release];
                [writer release];
                return false;
            }

            [writer startSessionAtSourceTime:kCMTimeZero];
            m_writer = writer;
            m_input = input;
            m_adaptor = adaptor;
        }

        return true;
    }

    bool Write( const uint8_t* aRgb, int64_t aIndex, std::string& aError ) override
    {
        if( !m_writer || aIndex <= m_lastIndex )
            return true;   // a frame for a slot already taken: the earlier one stays

        @autoreleasepool
        {
            for( int waited = 0; !m_input.readyForMoreMediaData && waited < 200; ++waited )
                usleep( 1000 );

            if( m_writer.status != AVAssetWriterStatusWriting )
            {
                aError = std::string( "AVAssetWriter failed: " )
                         + ( m_writer.error ? [[m_writer.error localizedDescription] UTF8String] : "unknown" );
                return false;
            }

            if( !m_input.readyForMoreMediaData )
                return true;   // encoder busy: drop this frame, the previous one stays on screen

            CVPixelBufferRef buffer = nullptr;

            if( !m_adaptor.pixelBufferPool
                || CVPixelBufferPoolCreatePixelBuffer( kCFAllocatorDefault, m_adaptor.pixelBufferPool, &buffer ) != kCVReturnSuccess )
            {
                aError = "no pixel buffer from the encoder";
                return false;
            }

            CVPixelBufferLockBaseAddress( buffer, 0 );
            uint8_t*     dst = static_cast<uint8_t*>( CVPixelBufferGetBaseAddress( buffer ) );
            const size_t stride = CVPixelBufferGetBytesPerRow( buffer );

            for( int y = 0; y < m_settings.height; ++y )
            {
                const uint8_t* s = aRgb + size_t( y ) * m_settings.width * 3;
                uint8_t*       d = dst + size_t( y ) * stride;

                for( int x = 0; x < m_settings.width; ++x, s += 3, d += 4 )
                {
                    d[0] = s[2];
                    d[1] = s[1];
                    d[2] = s[0];
                    d[3] = 255;
                }
            }

            CVPixelBufferUnlockBaseAddress( buffer, 0 );

            const bool ok = [m_adaptor appendPixelBuffer:buffer withPresentationTime:CMTimeMake( aIndex, m_settings.fps )];
            CVPixelBufferRelease( buffer );

            if( !ok )
            {
                aError = std::string( "appendPixelBuffer: " )
                         + ( m_writer.error ? [[m_writer.error localizedDescription] UTF8String] : "failed" );
                return false;
            }

            m_lastIndex = aIndex;
        }

        return true;
    }

    bool Close( int64_t aEndIndex, std::string& aError ) override
    {
        if( !m_writer )
            return true;

        bool ok = true;

        @autoreleasepool
        {
            if( m_writer.status == AVAssetWriterStatusWriting )
            {
                [m_input markAsFinished];
                [m_writer endSessionAtSourceTime:CMTimeMake( std::max<int64_t>( aEndIndex, m_lastIndex + 1 ), m_settings.fps )];

                dispatch_semaphore_t done = dispatch_semaphore_create( 0 );
                [m_writer finishWritingWithCompletionHandler:^{ dispatch_semaphore_signal( done ); }];
                dispatch_semaphore_wait( done, dispatch_time( DISPATCH_TIME_NOW, 60 * NSEC_PER_SEC ) );
                dispatch_release( done );
            }

            if( m_writer.status != AVAssetWriterStatusCompleted )
            {
                aError = std::string( "AVAssetWriter did not finish: " )
                         + ( m_writer.error ? [[m_writer.error localizedDescription] UTF8String] : "unknown" );
                ok = false;
            }

            [m_adaptor release];
            [m_input release];
            [m_writer release];
            m_adaptor = nil;
            m_input = nil;
            m_writer = nil;
        }

        return ok;
    }

private:
    KOPENAPI_VIDEO_SETTINGS               m_settings;
    AVAssetWriter*                        m_writer = nil;
    AVAssetWriterInput*                   m_input = nil;
    AVAssetWriterInputPixelBufferAdaptor* m_adaptor = nil;
    int64_t                               m_lastIndex = -1;
};

} // namespace


std::unique_ptr<KOPENAPI_VIDEO_ENCODER> KopenapiNativeVideoEncoder( const KOPENAPI_VIDEO_SETTINGS& aSettings )
{
    if( aSettings.format != "mp4" )
        return nullptr;

    return std::make_unique<APPLE_ENCODER>();
}

#endif
