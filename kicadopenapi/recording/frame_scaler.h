/// @file frame_scaler.h
/// @brief Frame scaling and pixel conversion for recordings: packed RGB24 frames resampled to
/// another size, fitted into an output frame with the aspect kept (letterbox), converted to
/// BGRA for the native encoders. Plain C++17, no wx dependency, no global state: every call
/// works on its own buffers, a KOPENAPI_FRAME_SCALER keeps them between frames (one object per
/// thread). Scaling runs the portable fixed-point resampler unless the OS's image library
/// (Accelerate / vImage on macOS) is asked for; RGB to BGRA uses the OS library when there is one.
#ifndef KICADOPENAPI_FRAME_SCALER_H
#define KICADOPENAPI_FRAME_SCALER_H

#include <cstddef>
#include <cstdint>
#include <memory>


/// @brief Resampling filter
enum class KOPENAPI_SCALE_QUALITY
{
    FAST,   ///< bilinear (2 taps per axis)
    HIGH    ///< area average when shrinking, bicubic (Catmull-Rom) when enlarging; vImage: Lanczos
};


/// @brief Which implementation a KOPENAPI_FRAME_SCALER uses
enum class KOPENAPI_SCALER_BACKEND
{
    AUTO,       ///< the fastest measured on this OS: the portable resampler (on one thread it beats
                ///< vImage 1.3x to 4x at 4K), the same output on every OS
    GENERIC,    ///< always the portable resampler
    NATIVE      ///< the OS's image library (vImage Lanczos on macOS; may use its worker threads),
                ///< the portable resampler where there is none or when it fails
};


/// @brief Where a fitted picture lands in the output frame
struct KOPENAPI_FIT_RECT
{
    int x = 0;          ///< left bar width
    int y = 0;          ///< top bar height
    int width = 0;      ///< picture width
    int height = 0;     ///< picture height
};


/// @brief The largest aSrcW x aSrcH-shaped rectangle inside aDstW x aDstH, centred.
/// @return all zero when any size is not positive
KOPENAPI_FIT_RECT KopenapiFitRect( int aSrcW, int aSrcH, int aDstW, int aDstH );


struct KOPENAPI_SCALER_STATE;


/// @brief A scaler that keeps its filter tables and work buffers between frames. Not shared
/// between threads; any number of scalers may run in parallel.
class KOPENAPI_FRAME_SCALER
{
public:
    /// @param aMultiThreaded the native library may split a frame over its worker threads
    /// (vImage tiling); the portable resampler always runs on the calling thread
    explicit KOPENAPI_FRAME_SCALER( KOPENAPI_SCALER_BACKEND aBackend = KOPENAPI_SCALER_BACKEND::AUTO,
                                    bool aMultiThreaded = true );
    ~KOPENAPI_FRAME_SCALER();

    KOPENAPI_FRAME_SCALER( KOPENAPI_FRAME_SCALER&& ) noexcept;
    KOPENAPI_FRAME_SCALER& operator=( KOPENAPI_FRAME_SCALER&& ) noexcept;
    KOPENAPI_FRAME_SCALER( const KOPENAPI_FRAME_SCALER& ) = delete;
    KOPENAPI_FRAME_SCALER& operator=( const KOPENAPI_FRAME_SCALER& ) = delete;

    /// @brief Resamples packed RGB24 aSrc (aSrcW x aSrcH, rows aSrcStride bytes apart) into the
    /// whole of aDst (aDstW x aDstH RGB24, rows aDstStride bytes apart). Source and destination
    /// must not overlap.
    /// @return false for a null pointer, a size that is not positive or a stride shorter than a row
    bool ScaleRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                   int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality );

    /// @brief Fits aSrc into the RGB24 frame aDst keeping the aspect (KopenapiFitRect), centred;
    /// the bars are filled with aFillRgb (0xRRGGBB). Every byte of the aDstW x aDstH frame is written.
    /// @return false on the arguments ScaleRgb rejects
    bool FitRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW, int aDstH,
                 int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, uint32_t aFillRgb = 0x000000 );

    /// @brief FitRgb with a BGRA output frame (B, G, R, 255 per pixel), e.g. a CVPixelBuffer of
    /// kCVPixelFormatType_32BGRA: saves the separate KopenapiRgbToBgra pass.
    bool FitRgbToBgra( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                       int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, uint32_t aFillRgb = 0x000000 );

private:
    bool fit( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW, int aDstH,
              int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, uint32_t aFillRgb, bool aBgra );

    bool scale( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW, int aDstH,
                int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, bool aBgra );

    std::unique_ptr<KOPENAPI_SCALER_STATE> m_state;
    KOPENAPI_SCALER_BACKEND                m_backend;
    bool                                   m_multiThreaded;
};


/// @brief One-shot KOPENAPI_FRAME_SCALER::ScaleRgb (allocates its work buffers per call)
bool KopenapiScaleRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                       int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality );

/// @brief One-shot KOPENAPI_FRAME_SCALER::FitRgb
bool KopenapiFitRgb( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                     int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality, uint32_t aFillRgb = 0x000000 );

/// @brief One-shot KOPENAPI_FRAME_SCALER::FitRgbToBgra
bool KopenapiFitRgbToBgra( const uint8_t* aSrc, int aSrcW, int aSrcH, int aSrcStride, uint8_t* aDst, int aDstW,
                           int aDstH, int aDstStride, KOPENAPI_SCALE_QUALITY aQuality,
                           uint32_t aFillRgb = 0x000000 );

/// @brief Packed RGB24 to BGRA (B, G, R, 255), aW x aH pixels; rows aSrcStride / aDstStride bytes
/// apart. Does nothing on a null pointer or a size that is not positive.
void KopenapiRgbToBgra( const uint8_t* aSrc, int aW, int aH, int aSrcStride, uint8_t* aDst, int aDstStride );

#endif
