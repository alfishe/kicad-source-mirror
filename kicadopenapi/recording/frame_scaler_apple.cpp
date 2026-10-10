/// @file frame_scaler_apple.cpp
/// @brief Frame scaling through Accelerate / vImage (macOS): RGB24 is widened to 4 channels,
/// resampled by vImageScale_ARGB8888 (Lanczos 3; Lanczos 5 for HIGH) and narrowed back, or
/// written straight into a BGRA frame. Work buffers live in the scaler state.
#if defined(__APPLE__)

#include "frame_scaler_impl.h"

#include <Accelerate/Accelerate.h>


namespace
{

vImage_Buffer buffer( const void* aData, int aW, int aH, size_t aStride )
{
    vImage_Buffer b;
    b.data = const_cast<void*>( aData );
    b.width = vImagePixelCount( aW );
    b.height = vImagePixelCount( aH );
    b.rowBytes = aStride;
    return b;
}


uint8_t* work( std::vector<uint8_t>& aBuffer, size_t aBytes )
{
    if( aBuffer.size() < aBytes )
        aBuffer.resize( aBytes );

    return aBuffer.data();
}

} // namespace


bool KopenapiScaleApple( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                         int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, bool aBgra,
                         bool aMultiThreaded, KOPENAPI_SCALER_STATE& aState )
{
    const vImage_Flags threads = aMultiThreaded ? kvImageNoFlags : kvImageDoNotTile;
    const vImage_Flags scaleFlags = threads | kvImageEdgeExtend
                                    | ( aQuality == KOPENAPI_SCALE_QUALITY::HIGH ? kvImageHighQualityResampling
                                                                                 : kvImageNoFlags );

    // 4-channel source in the destination's channel order: the scaler is order-agnostic
    const size_t        src4Stride = size_t( aSrcW ) * 4;
    const vImage_Buffer rgb = buffer( aSrc, aSrcW, aSrcH, size_t( aSrcStride ) );
    const vImage_Buffer src4 = buffer( work( aState.nativeSrc, src4Stride * aSrcH ), aSrcW, aSrcH, src4Stride );

    const vImage_Error widened = aBgra ? vImageConvert_RGB888toBGRA8888( &rgb, nullptr, 255, &src4, false, threads )
                                       : vImageConvert_RGB888toRGBA8888( &rgb, nullptr, 255, &src4, false, threads );

    if( widened != kvImageNoError )
        return false;

    const size_t  dst4Stride = size_t( aDstW ) * 4;
    vImage_Buffer dst4 = aBgra ? buffer( aDst, aDstW, aDstH, size_t( aDstStride ) )
                               : buffer( work( aState.nativeDst, dst4Stride * aDstH ), aDstW, aDstH, dst4Stride );

    const vImage_Error tempSize = vImageScale_ARGB8888( &src4, &dst4, nullptr, scaleFlags | kvImageGetTempBufferSize );

    if( tempSize < 0 )
        return false;

    void* temp = tempSize > 0 ? work( aState.nativeTemp, size_t( tempSize ) ) : nullptr;

    if( vImageScale_ARGB8888( &src4, &dst4, temp, scaleFlags ) != kvImageNoError )
        return false;

    if( aBgra )
        return true;

    const vImage_Buffer rgbOut = buffer( aDst, aDstW, aDstH, size_t( aDstStride ) );
    return vImageConvert_RGBA8888toRGB888( &dst4, &rgbOut, threads ) == kvImageNoError;
}


bool KopenapiRgbToBgraApple( const uint8_t* aSrc, int aW, int aH, int aSrcStride, uint8_t* aDst, int aDstStride )
{
    const vImage_Buffer rgb = buffer( aSrc, aW, aH, size_t( aSrcStride ) );
    const vImage_Buffer bgra = buffer( aDst, aW, aH, size_t( aDstStride ) );
    return vImageConvert_RGB888toBGRA8888( &rgb, nullptr, 255, &bgra, false, kvImageNoFlags ) == kvImageNoError;
}

#endif
