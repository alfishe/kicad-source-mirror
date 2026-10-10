/// @file frame_scaler_generic.cpp
/// @brief Portable resampler: separable, fixed point (14-bit weights), one filter table per axis
/// computed once per size and reused. Rows are streamed: when the height grows each source row
/// is filtered horizontally once into a ring of rows the vertical filter combines; when it
/// shrinks the vertical filter runs first on the source rows and the horizontal one on its
/// result, so the costlier horizontal pass runs on the fewer rows. The horizontal filter works
/// on an RGBX copy of the row (one pixel per 32-bit lane group); the hot loops have NEON / SSE2
/// versions, the scalar ones serve every other target.
#include "frame_scaler_impl.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#if defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(_M_ARM64)
#include <arm_neon.h>
#define KOPENAPI_SCALER_NEON 1
#elif defined(__SSE2__) || defined(_M_X64) || ( defined(_M_IX86_FP) && _M_IX86_FP >= 2 )
#include <emmintrin.h>
#define KOPENAPI_SCALER_SSE2 1
#endif


namespace
{

constexpr int32_t ONE = 1 << KOPENAPI_SCALER_SHIFT;
constexpr int32_t HALF = ONE >> 1;


inline uint8_t clamp8( int32_t aValue )
{
    return uint8_t( aValue < 0 ? 0 : ( aValue > 255 ? 255 : aValue ) );
}


/// @brief Catmull-Rom cubic (a = -0.5): sharp, interpolating, little ringing on screen content
double cubic( double aX )
{
    aX = std::fabs( aX );

    if( aX < 1.0 )
        return ( 1.5 * aX - 2.5 ) * aX * aX + 1.0;

    if( aX < 2.0 )
        return ( ( -0.5 * aX + 2.5 ) * aX - 4.0 ) * aX + 2.0;

    return 0.0;
}


double triangle( double aX )
{
    aX = std::fabs( aX );
    return aX < 1.0 ? 1.0 - aX : 0.0;
}


/// @brief Filter table of one axis. Pixel centres map as (i + 0.5) * src / dst - 0.5; taps outside the
/// axis fold onto the edge pixel; HIGH shrinking is the exact area average.
void buildAxis( KOPENAPI_SCALE_AXIS& aAxis, int aSrc, int aDst, KOPENAPI_SCALE_QUALITY aQuality )
{
    const int quality = int( aQuality );

    if( aAxis.srcLen == aSrc && aAxis.dstLen == aDst && aAxis.quality == quality )
        return;

    aAxis.srcLen = aSrc;
    aAxis.dstLen = aDst;
    aAxis.quality = quality;
    aAxis.identity = aSrc == aDst;
    aAxis.taps = 0;
    aAxis.start.clear();
    aAxis.weights.clear();

    if( aAxis.identity )
        return;

    const double scale = double( aSrc ) / aDst;
    const bool   area = aQuality == KOPENAPI_SCALE_QUALITY::HIGH && aDst < aSrc;
    const bool   bicubic = aQuality == KOPENAPI_SCALE_QUALITY::HIGH;

    std::vector<int>                 first( aDst );
    std::vector<std::vector<double>> real( aDst );
    int                              maxTaps = 1;

    for( int i = 0; i < aDst; ++i )
    {
        std::vector<double>& w = real[i];
        int                  lo = 0;

        if( area )
        {
            const double a = i * scale;
            const double b = ( i + 1 ) * scale;
            lo = int( std::floor( a ) );
            const int hi = std::min( aSrc, int( std::ceil( b ) ) );

            for( int j = lo; j < hi; ++j )
                w.push_back( ( std::min( b, j + 1.0 ) - std::max( a, double( j ) ) ) / scale );
        }
        else
        {
            const double centre = ( i + 0.5 ) * scale - 0.5;
            const double support = bicubic ? 2.0 : 1.0;
            const int    j0 = int( std::floor( centre - support ) ) + 1;
            const int    j1 = int( std::floor( centre + support ) );
            lo = std::clamp( j0, 0, aSrc - 1 );
            const int hi = std::clamp( j1, 0, aSrc - 1 );
            w.assign( size_t( hi - lo + 1 ), 0.0 );

            for( int j = j0; j <= j1; ++j )
                w[size_t( std::clamp( j, 0, aSrc - 1 ) - lo )] += bicubic ? cubic( j - centre ) : triangle( j - centre );
        }

        while( w.size() > 1 && std::fabs( w.front() ) < 1e-9 )
        {
            w.erase( w.begin() );
            ++lo;
        }

        while( w.size() > 1 && std::fabs( w.back() ) < 1e-9 )
            w.pop_back();

        double sum = 0.0;

        for( double v : w )
            sum += v;

        for( double& v : w )
            v /= sum;

        first[i] = lo;
        maxTaps = std::max( maxTaps, int( w.size() ) );
    }

    // even: the SIMD loops take the taps in pairs
    const int taps = ( maxTaps + 1 ) & ~1;
    aAxis.taps = taps;
    aAxis.start.resize( size_t( aDst ) );
    aAxis.weights.assign( size_t( aDst ) * taps, 0 );

    for( int i = 0; i < aDst; ++i )
    {
        const std::vector<double>& w = real[i];

        // the window stays inside the axis when it can; inputs it covers beyond w get weight 0
        const int start = std::min( first[i], std::max( 0, aSrc - taps ) );
        const int offset = first[i] - start;

        int16_t* out = aAxis.weights.data() + size_t( i ) * taps;
        int32_t  sum = 0;
        int      largest = offset;

        for( size_t k = 0; k < w.size(); ++k )
        {
            const int32_t q = int32_t( std::lround( w[k] * ONE ) );
            out[offset + k] = int16_t( q );
            sum += q;

            if( std::abs( q ) > std::abs( out[largest] ) )
                largest = offset + int( k );
        }

        out[largest] = int16_t( out[largest] + ( ONE - sum ) );
        aAxis.start[size_t( i )] = start;
    }
}


// ---- row kernels ----------------------------------------------------------------------------

/// @brief RGB24 to RGBX (X = 255)
void expandRow( const uint8_t* aRgb, int aW, uint8_t* aRgbx )
{
    int x = 0;

#if defined(KOPENAPI_SCALER_NEON)
    for( ; x + 16 <= aW; x += 16 )
    {
        const uint8x16x3_t s = vld3q_u8( aRgb + 3 * size_t( x ) );
        uint8x16x4_t       d;
        d.val[0] = s.val[0];
        d.val[1] = s.val[1];
        d.val[2] = s.val[2];
        d.val[3] = vdupq_n_u8( 255 );
        vst4q_u8( aRgbx + 4 * size_t( x ), d );
    }
#endif

    for( ; x < aW; ++x )
    {
        aRgbx[4 * size_t( x ) + 0] = aRgb[3 * size_t( x ) + 0];
        aRgbx[4 * size_t( x ) + 1] = aRgb[3 * size_t( x ) + 1];
        aRgbx[4 * size_t( x ) + 2] = aRgb[3 * size_t( x ) + 2];
        aRgbx[4 * size_t( x ) + 3] = 255;
    }
}


/// @brief RGB24 to BGRA, alpha 255
void bgraRow( const uint8_t* aRgb, int aW, uint8_t* aBgra )
{
    int x = 0;

#if defined(KOPENAPI_SCALER_NEON)
    for( ; x + 16 <= aW; x += 16 )
    {
        const uint8x16x3_t s = vld3q_u8( aRgb + 3 * size_t( x ) );
        uint8x16x4_t       d;
        d.val[0] = s.val[2];
        d.val[1] = s.val[1];
        d.val[2] = s.val[0];
        d.val[3] = vdupq_n_u8( 255 );
        vst4q_u8( aBgra + 4 * size_t( x ), d );
    }
#endif

    for( ; x < aW; ++x )
    {
        const uint8_t* s = aRgb + 3 * size_t( x );
        uint8_t*       d = aBgra + 4 * size_t( x );
        d[0] = s[2];
        d[1] = s[1];
        d[2] = s[0];
        d[3] = 255;
    }
}


/// @brief One output pixel of the horizontal filter: RGBX bytes in memory order in a uint32_t
template <int TAPS>
inline uint32_t horizontalPixel( const uint8_t* aRgbx, const int16_t* aW, int aTaps )
{
    const int taps = TAPS ? TAPS : aTaps;

#if defined(KOPENAPI_SCALER_NEON)
    int32x4_t acc = vdupq_n_s32( HALF );

    for( int k = 0; k < taps; k += 2 )
    {
        const int16x8_t v = vreinterpretq_s16_u16( vmovl_u8( vld1_u8( aRgbx + 4 * k ) ) );
        acc = vmlal_n_s16( acc, vget_low_s16( v ), aW[k] );
        acc = vmlal_n_s16( acc, vget_high_s16( v ), aW[k + 1] );
    }

    const uint16x4_t n16 = vqshrun_n_s32( acc, KOPENAPI_SCALER_SHIFT );
    const uint8x8_t  n8 = vqmovn_u16( vcombine_u16( n16, n16 ) );
    return vget_lane_u32( vreinterpret_u32_u8( n8 ), 0 );
#elif defined(KOPENAPI_SCALER_SSE2)
    const __m128i zero = _mm_setzero_si128();
    __m128i       acc = _mm_set1_epi32( HALF );

    for( int k = 0; k < taps; k += 2 )
    {
        // pixels k and k + 1 as int16 pairs (R0 R1 G0 G1 B0 B1 X0 X1) against (w0 w1) pairs
        const __m128i v = _mm_loadl_epi64( reinterpret_cast<const __m128i*>( aRgbx + 4 * k ) );
        const __m128i pairs = _mm_unpacklo_epi8( _mm_unpacklo_epi8( v, _mm_srli_si128( v, 4 ) ), zero );
        int32_t       w2;
        std::memcpy( &w2, aW + k, 4 );
        acc = _mm_add_epi32( acc, _mm_madd_epi16( pairs, _mm_set1_epi32( w2 ) ) );
    }

    const __m128i s = _mm_srai_epi32( acc, KOPENAPI_SCALER_SHIFT );
    const __m128i p16 = _mm_packs_epi32( s, s );
    return uint32_t( _mm_cvtsi128_si32( _mm_packus_epi16( p16, p16 ) ) );
#else
    int32_t r = HALF, g = HALF, b = HALF;

    for( int k = 0; k < taps; ++k )
    {
        r += aW[k] * aRgbx[4 * k + 0];
        g += aW[k] * aRgbx[4 * k + 1];
        b += aW[k] * aRgbx[4 * k + 2];
    }

    const uint8_t bytes[4] = { clamp8( r >> KOPENAPI_SCALER_SHIFT ), clamp8( g >> KOPENAPI_SCALER_SHIFT ),
                               clamp8( b >> KOPENAPI_SCALER_SHIFT ), 0 };
    uint32_t      pixel;
    std::memcpy( &pixel, bytes, 4 );
    return pixel;
#endif
}


/// @brief Horizontal filter of one row: padded RGBX in, exactly aAxis.dstLen RGB24 pixels out
template <int TAPS>
void horizontalRowT( const uint8_t* aRgbx, const KOPENAPI_SCALE_AXIS& aAxis, uint8_t* aOut )
{
    const int      taps = aAxis.taps;
    const int      n = aAxis.dstLen;
    const int32_t* start = aAxis.start.data();
    const int16_t* w = aAxis.weights.data();

    // 4-byte stores overlapping the next pixel; the last pixel stores its 3 bytes only
    for( int x = 0; x < n - 1; ++x, w += taps )
    {
        const uint32_t pixel = horizontalPixel<TAPS>( aRgbx + 4 * size_t( start[x] ), w, taps );
        std::memcpy( aOut + 3 * size_t( x ), &pixel, 4 );
    }

    const uint32_t last = horizontalPixel<TAPS>( aRgbx + 4 * size_t( start[n - 1] ), w, taps );
    std::memcpy( aOut + 3 * size_t( n - 1 ), &last, 3 );
}


void horizontalRow( const uint8_t* aRgbx, const KOPENAPI_SCALE_AXIS& aAxis, uint8_t* aOut )
{
    switch( aAxis.taps )
    {
    case 2: horizontalRowT<2>( aRgbx, aAxis, aOut ); break;
    case 4: horizontalRowT<4>( aRgbx, aAxis, aOut ); break;
    case 6: horizontalRowT<6>( aRgbx, aAxis, aOut ); break;
    default: horizontalRowT<0>( aRgbx, aAxis, aOut ); break;
    }
}


/// @brief Vertical filter: aBytes bytes of aOut from the same bytes of aTaps rows (aTaps even)
void verticalRow( const uint8_t* const* aRows, const int16_t* aW, int aTaps, uint8_t* aOut, size_t aBytes )
{
    size_t i = 0;

#if defined(KOPENAPI_SCALER_NEON)
    for( ; i + 16 <= aBytes; i += 16 )
    {
        int32x4_t a0 = vdupq_n_s32( HALF ), a1 = a0, a2 = a0, a3 = a0;

        for( int k = 0; k < aTaps; ++k )
        {
            const uint8x16_t v = vld1q_u8( aRows[k] + i );
            const int16x8_t  lo = vreinterpretq_s16_u16( vmovl_u8( vget_low_u8( v ) ) );
            const int16x8_t  hi = vreinterpretq_s16_u16( vmovl_u8( vget_high_u8( v ) ) );
            const int16_t    w = aW[k];
            a0 = vmlal_n_s16( a0, vget_low_s16( lo ), w );
            a1 = vmlal_n_s16( a1, vget_high_s16( lo ), w );
            a2 = vmlal_n_s16( a2, vget_low_s16( hi ), w );
            a3 = vmlal_n_s16( a3, vget_high_s16( hi ), w );
        }

        const uint16x8_t l = vcombine_u16( vqshrun_n_s32( a0, KOPENAPI_SCALER_SHIFT ),
                                           vqshrun_n_s32( a1, KOPENAPI_SCALER_SHIFT ) );
        const uint16x8_t h = vcombine_u16( vqshrun_n_s32( a2, KOPENAPI_SCALER_SHIFT ),
                                           vqshrun_n_s32( a3, KOPENAPI_SCALER_SHIFT ) );
        vst1q_u8( aOut + i, vcombine_u8( vqmovn_u16( l ), vqmovn_u16( h ) ) );
    }
#elif defined(KOPENAPI_SCALER_SSE2)
    const __m128i zero = _mm_setzero_si128();

    for( ; i + 16 <= aBytes; i += 16 )
    {
        __m128i a0 = _mm_set1_epi32( HALF ), a1 = a0, a2 = a0, a3 = a0;

        for( int k = 0; k < aTaps; k += 2 )
        {
            // rows k and k + 1 interleaved as int16 pairs against (w0 w1) pairs
            const __m128i r0 = _mm_loadu_si128( reinterpret_cast<const __m128i*>( aRows[k] + i ) );
            const __m128i r1 = _mm_loadu_si128( reinterpret_cast<const __m128i*>( aRows[k + 1] + i ) );
            int32_t       w2;
            std::memcpy( &w2, aW + k, 4 );
            const __m128i w = _mm_set1_epi32( w2 );
            const __m128i lo = _mm_unpacklo_epi8( r0, r1 );
            const __m128i hi = _mm_unpackhi_epi8( r0, r1 );
            a0 = _mm_add_epi32( a0, _mm_madd_epi16( _mm_unpacklo_epi8( lo, zero ), w ) );
            a1 = _mm_add_epi32( a1, _mm_madd_epi16( _mm_unpackhi_epi8( lo, zero ), w ) );
            a2 = _mm_add_epi32( a2, _mm_madd_epi16( _mm_unpacklo_epi8( hi, zero ), w ) );
            a3 = _mm_add_epi32( a3, _mm_madd_epi16( _mm_unpackhi_epi8( hi, zero ), w ) );
        }

        const __m128i l = _mm_packs_epi32( _mm_srai_epi32( a0, KOPENAPI_SCALER_SHIFT ),
                                           _mm_srai_epi32( a1, KOPENAPI_SCALER_SHIFT ) );
        const __m128i h = _mm_packs_epi32( _mm_srai_epi32( a2, KOPENAPI_SCALER_SHIFT ),
                                           _mm_srai_epi32( a3, KOPENAPI_SCALER_SHIFT ) );
        _mm_storeu_si128( reinterpret_cast<__m128i*>( aOut + i ), _mm_packus_epi16( l, h ) );
    }
#endif

    for( ; i < aBytes; ++i )
    {
        int32_t acc = HALF;

        for( int k = 0; k < aTaps; ++k )
            acc += aW[k] * aRows[k][i];

        aOut[i] = clamp8( acc >> KOPENAPI_SCALER_SHIFT );
    }
}

} // namespace


bool KopenapiScaleGeneric( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                           int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, bool aBgra,
                           KOPENAPI_SCALER_STATE& aState )
{
    KOPENAPI_SCALE_AXIS& h = aState.horizontal;
    KOPENAPI_SCALE_AXIS& v = aState.vertical;
    buildAxis( h, aSrcW, aDstW, aQuality );
    buildAxis( v, aSrcH, aDstH, aQuality );

    const size_t srcRowBytes = size_t( aSrcW ) * 3;
    const size_t dstRowBytes = size_t( aDstW ) * 3;

    auto srcAt = [&]( int aY ) { return aSrc + size_t( aY ) * size_t( aSrcStride ); };
    auto dstAt = [&]( int aY ) { return aDst + size_t( aY ) * size_t( aDstStride ); };

    // a BGRA frame gets each RGB24 row through a one-row buffer
    std::vector<uint8_t>& rgbOut = aState.rowC;

    if( aBgra && rgbOut.size() < dstRowBytes )
        rgbOut.resize( dstRowBytes );

    auto target = [&]( int aY ) { return aBgra ? rgbOut.data() : dstAt( aY ); };

    auto commit = [&]( int aY )
    {
        if( aBgra )
            bgraRow( rgbOut.data(), aDstW, dstAt( aY ) );
    };

    // RGBX copy of the row the horizontal filter reads, padded for its paired 8-byte loads
    std::vector<uint8_t>& rgbx = aState.rowA;
    const size_t          rgbxBytes = ( size_t( std::max( aSrcW, h.taps ) ) + 2 ) * 4;

    if( !h.identity && rgbx.size() < rgbxBytes )
        rgbx.resize( rgbxBytes, 0 );

    auto horizontal = [&]( const uint8_t* aRgb, uint8_t* aOut )
    {
        if( h.identity )
        {
            if( aOut != aRgb )
                std::memcpy( aOut, aRgb, dstRowBytes );

            return;
        }

        expandRow( aRgb, aSrcW, rgbx.data() );
        horizontalRow( rgbx.data(), h, aOut );
    };

    if( v.identity )
    {
        for( int y = 0; y < aDstH; ++y )
        {
            horizontal( srcAt( y ), target( y ) );
            commit( y );
        }

        return true;
    }

    const int                   taps = v.taps;
    std::vector<const uint8_t*> rows( size_t( taps ), nullptr );

    if( aDstH > aSrcH )
    {
        // taps consecutive source rows are live at a time: row r lives in slot r % taps
        if( !h.identity )
        {
            aState.ring.resize( size_t( taps ) * dstRowBytes );
            aState.ringRow.assign( size_t( taps ), -1 );
        }

        for( int y = 0; y < aDstH; ++y )
        {
            const int start = v.start[size_t( y )];

            for( int k = 0; k < taps; ++k )
            {
                const int sy = std::min( start + k, aSrcH - 1 );

                if( h.identity )
                {
                    rows[size_t( k )] = srcAt( sy );
                    continue;
                }

                const int slot = sy % taps;
                uint8_t*  ringRow = aState.ring.data() + size_t( slot ) * dstRowBytes;

                if( aState.ringRow[size_t( slot )] != sy )
                {
                    horizontal( srcAt( sy ), ringRow );
                    aState.ringRow[size_t( slot )] = sy;
                }

                rows[size_t( k )] = ringRow;
            }

            verticalRow( rows.data(), v.weights.data() + size_t( y ) * taps, taps, target( y ), dstRowBytes );
            commit( y );
        }

        return true;
    }

    std::vector<uint8_t>& vertical = aState.rowB;

    if( !h.identity && vertical.size() < srcRowBytes )
        vertical.resize( srcRowBytes );

    for( int y = 0; y < aDstH; ++y )
    {
        const int start = v.start[size_t( y )];

        for( int k = 0; k < taps; ++k )
            rows[size_t( k )] = srcAt( std::min( start + k, aSrcH - 1 ) );

        const int16_t* w = v.weights.data() + size_t( y ) * taps;

        if( h.identity )
        {
            verticalRow( rows.data(), w, taps, target( y ), dstRowBytes );
        }
        else
        {
            verticalRow( rows.data(), w, taps, vertical.data(), srcRowBytes );
            horizontal( vertical.data(), target( y ) );
        }

        commit( y );
    }

    return true;
}


void KopenapiRgbToBgraGeneric( const uint8_t* aSrc, int aW, int aH, int aSrcStride, uint8_t* aDst,
                               int aDstStride )
{
    for( int y = 0; y < aH; ++y )
        bgraRow( aSrc + size_t( y ) * size_t( aSrcStride ), aW, aDst + size_t( y ) * size_t( aDstStride ) );
}


void KopenapiFillRectGeneric( uint8_t* aDst, int aW, int aH, int aDstStride, uint32_t aFillRgb, bool aBgra )
{
    if( aW <= 0 || aH <= 0 )
        return;

    const uint8_t r = uint8_t( aFillRgb >> 16 );
    const uint8_t g = uint8_t( aFillRgb >> 8 );
    const uint8_t b = uint8_t( aFillRgb );
    const size_t  bpp = aBgra ? 4 : 3;
    const size_t  rowBytes = size_t( aW ) * bpp;

    // the first row pixel by pixel, the others copy it
    for( int x = 0; x < aW; ++x )
    {
        uint8_t* p = aDst + size_t( x ) * bpp;

        if( aBgra )
        {
            p[0] = b;
            p[1] = g;
            p[2] = r;
            p[3] = 255;
        }
        else
        {
            p[0] = r;
            p[1] = g;
            p[2] = b;
        }
    }

    for( int y = 1; y < aH; ++y )
        std::memcpy( aDst + size_t( y ) * size_t( aDstStride ), aDst, rowBytes );
}
