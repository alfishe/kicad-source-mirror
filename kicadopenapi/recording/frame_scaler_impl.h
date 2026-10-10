/// @file frame_scaler_impl.h
/// @brief Internals of frame_scaler.cpp: the scaler state and the per-implementation entry
/// points (portable in frame_scaler_generic.cpp, one file per OS library). Arguments are
/// validated by the dispatcher; destinations are written whole, never read.
#ifndef KICADOPENAPI_FRAME_SCALER_IMPL_H
#define KICADOPENAPI_FRAME_SCALER_IMPL_H

#include "frame_scaler.h"

#include <cstddef>
#include <cstdint>
#include <vector>


/// @brief Fixed-point filter of one axis: every output takes `taps` consecutive inputs from
/// start[i] with weights[i * taps ...] (sum 1 << KOPENAPI_SCALER_SHIFT). `taps` is even; inputs
/// past the end of the axis carry weight 0.
struct KOPENAPI_SCALE_AXIS
{
    int                  srcLen = 0;
    int                  dstLen = 0;
    int                  quality = -1;
    bool                 identity = false;  ///< same length: a copy, the tables are empty
    int                  taps = 0;
    std::vector<int32_t> start;
    std::vector<int16_t> weights;
};

constexpr int KOPENAPI_SCALER_SHIFT = 14;


/// @brief Buffers and tables one KOPENAPI_FRAME_SCALER reuses between frames
struct KOPENAPI_SCALER_STATE
{
    KOPENAPI_SCALE_AXIS  horizontal;
    KOPENAPI_SCALE_AXIS  vertical;
    std::vector<uint8_t> ring;          ///< horizontally scaled rows (vertical enlarging)
    std::vector<int>     ringRow;       ///< source row held by each ring slot, -1 none
    std::vector<uint8_t> rowA;          ///< one-row work buffers
    std::vector<uint8_t> rowB;
    std::vector<uint8_t> rowC;
    std::vector<uint8_t> nativeSrc;     ///< OS library work buffers
    std::vector<uint8_t> nativeDst;
    std::vector<uint8_t> nativeTemp;
};


/// @brief Portable resampler; aBgra: aDst is BGRA (4 bytes per pixel), else RGB24
bool KopenapiScaleGeneric( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                           int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, bool aBgra,
                           KOPENAPI_SCALER_STATE& aState );

/// @brief Portable RGB24 to BGRA (alpha 255)
void KopenapiRgbToBgraGeneric( const uint8_t* aSrc, int aW, int aH, int aSrcStride, uint8_t* aDst,
                               int aDstStride );

/// @brief Fills a aW x aH rectangle of an RGB24 (aBgra false) or BGRA frame with aFillRgb (0xRRGGBB)
void KopenapiFillRectGeneric( uint8_t* aDst, int aW, int aH, int aDstStride, uint32_t aFillRgb, bool aBgra );


#if defined(__APPLE__)

/// @brief vImage resampler (Lanczos); false when vImage fails (the caller falls back)
bool KopenapiScaleApple( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                         int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, bool aBgra,
                         bool aMultiThreaded, KOPENAPI_SCALER_STATE& aState );

/// @brief vImage RGB24 to BGRA; false when vImage fails
bool KopenapiRgbToBgraApple( const uint8_t* aSrc, int aW, int aH, int aSrcStride, uint8_t* aDst, int aDstStride );

#endif

#endif
