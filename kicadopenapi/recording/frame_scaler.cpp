/// @file frame_scaler.cpp
/// @brief Frame scaler entry points: argument checks, letterboxing, and the choice between the
/// OS's image library (frame_scaler_apple.cpp) and the portable resampler
/// (frame_scaler_generic.cpp), which also takes over on every native failure.
#include "frame_scaler.h"

#include "frame_scaler_impl.h"

#include <cstdint>


namespace
{

bool validFrame( const uint8_t* aData, int aW, int aH, int aStride, int aBytesPerPixel )
{
    return aData && aW > 0 && aH > 0 && int64_t( aStride ) >= int64_t( aW ) * aBytesPerPixel;
}

} // namespace


KOPENAPI_FIT_RECT KopenapiFitRect( int aSrcW, int aSrcH, int aDstW, int aDstH )
{
    KOPENAPI_FIT_RECT rect;

    if( aSrcW <= 0 || aSrcH <= 0 || aDstW <= 0 || aDstH <= 0 )
        return rect;

    if( int64_t( aSrcW ) * aDstH <= int64_t( aSrcH ) * aDstW )
    {
        rect.height = aDstH;
        rect.width = int( ( int64_t( aSrcW ) * aDstH + aSrcH / 2 ) / aSrcH );
    }
    else
    {
        rect.width = aDstW;
        rect.height = int( ( int64_t( aSrcH ) * aDstW + aSrcW / 2 ) / aSrcW );
    }

    rect.width = rect.width < 1 ? 1 : ( rect.width > aDstW ? aDstW : rect.width );
    rect.height = rect.height < 1 ? 1 : ( rect.height > aDstH ? aDstH : rect.height );
    rect.x = ( aDstW - rect.width ) / 2;
    rect.y = ( aDstH - rect.height ) / 2;
    return rect;
}


KOPENAPI_FRAME_SCALER::KOPENAPI_FRAME_SCALER( KOPENAPI_SCALER_BACKEND aBackend, bool aMultiThreaded ) :
        m_state( std::make_unique<KOPENAPI_SCALER_STATE>() ),
        m_backend( aBackend ),
        m_multiThreaded( aMultiThreaded )
{
}


KOPENAPI_FRAME_SCALER::~KOPENAPI_FRAME_SCALER() = default;
KOPENAPI_FRAME_SCALER::KOPENAPI_FRAME_SCALER( KOPENAPI_FRAME_SCALER&& ) noexcept = default;
KOPENAPI_FRAME_SCALER& KOPENAPI_FRAME_SCALER::operator=( KOPENAPI_FRAME_SCALER&& ) noexcept = default;


bool KOPENAPI_FRAME_SCALER::scale( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst,
                                   int aDstW, int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality,
                                   bool aBgra )
{
    if( !m_state )
        m_state = std::make_unique<KOPENAPI_SCALER_STATE>();

#if defined(__APPLE__)
    if( m_backend == KOPENAPI_SCALER_BACKEND::NATIVE
        && KopenapiScaleApple( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality, aBgra,
                               m_multiThreaded, *m_state ) )
    {
        return true;
    }
#endif

    return KopenapiScaleGeneric( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality, aBgra,
                                 *m_state );
}


bool KOPENAPI_FRAME_SCALER::ScaleRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst,
                                      int aDstW, int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality )
{
    if( !validFrame( aSrc, aSrcW, aSrcH, aSrcStride, 3 ) || !validFrame( aDst, aDstW, aDstH, aDstStride, 3 ) )
        return false;

    return scale( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality, false );
}


bool KOPENAPI_FRAME_SCALER::fit( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst,
                                 int aDstW, int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality,
                                 uint32_t aFillRgb, bool aBgra )
{
    const int bpp = aBgra ? 4 : 3;

    if( !validFrame( aSrc, aSrcW, aSrcH, aSrcStride, 3 ) || !validFrame( aDst, aDstW, aDstH, aDstStride, bpp ) )
        return false;

    const KOPENAPI_FIT_RECT r = KopenapiFitRect( aSrcW, aSrcH, aDstW, aDstH );
    auto                    at = [&]( int aX, int aY ) { return aDst + size_t( aY ) * aDstStride + size_t( aX ) * bpp; };

    // bars: the bands above and below, then the strips left and right of the picture
    KopenapiFillRectGeneric( at( 0, 0 ), aDstW, r.y, aDstStride, aFillRgb, aBgra );
    KopenapiFillRectGeneric( at( 0, r.y + r.height ), aDstW, aDstH - r.y - r.height, aDstStride, aFillRgb, aBgra );
    KopenapiFillRectGeneric( at( 0, r.y ), r.x, r.height, aDstStride, aFillRgb, aBgra );
    KopenapiFillRectGeneric( at( r.x + r.width, r.y ), aDstW - r.x - r.width, r.height, aDstStride, aFillRgb,
                             aBgra );

    return scale( aSrc, aSrcW, aSrcH, aSrcStride, at( r.x, r.y ), r.width, r.height, aDstStride, aQuality, aBgra );
}


bool KOPENAPI_FRAME_SCALER::FitRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst,
                                    int aDstW, int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality,
                                    uint32_t aFillRgb )
{
    return fit( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality, aFillRgb, false );
}


bool KOPENAPI_FRAME_SCALER::FitRgbToBgra( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst,
                                          int aDstW, int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality,
                                          uint32_t aFillRgb )
{
    return fit( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality, aFillRgb, true );
}


bool KopenapiScaleRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                       int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality )
{
    KOPENAPI_FRAME_SCALER scaler;
    return scaler.ScaleRgb( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality );
}


bool KopenapiFitRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                     int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, uint32_t aFillRgb )
{
    KOPENAPI_FRAME_SCALER scaler;
    return scaler.FitRgb( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality, aFillRgb );
}


bool KopenapiFitRgbToBgra( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                           int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, uint32_t aFillRgb )
{
    KOPENAPI_FRAME_SCALER scaler;
    return scaler.FitRgbToBgra( aSrc, aSrcW, aSrcH, aSrcStride, aDst, aDstW, aDstH, aDstStride, aQuality,
                                aFillRgb );
}


void KopenapiRgbToBgra( const uint8_t* aSrc, int aW, int aH, int aSrcStride, uint8_t* aDst, int aDstStride )
{
    if( !validFrame( aSrc, aW, aH, aSrcStride, 3 ) || !validFrame( aDst, aW, aH, aDstStride, 4 ) )
        return;

#if defined(__APPLE__)
    if( KopenapiRgbToBgraApple( aSrc, aW, aH, aSrcStride, aDst, aDstStride ) )
        return;
#endif

    KopenapiRgbToBgraGeneric( aSrc, aW, aH, aSrcStride, aDst, aDstStride );
}
