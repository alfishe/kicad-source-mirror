/// @file kicadopenapi_capture.h
/// @brief Images of KiCad windows without reading the screen: whole windows drawn offscreen by the
/// toolkit with their OpenGL canvases read back in place, or a window's main canvas alone.
/// Main thread only.
#ifndef KICADOPENAPI_CAPTURE_H
#define KICADOPENAPI_CAPTURE_H

#include <kicommon.h>

#include <string>

class wxImage;
class wxTopLevelWindow;
class wxWindow;


/// @brief Stable id of a window for window_list / window_capture ("w" + address)
KICOMMON_API std::string KopenapiWindowId( const wxWindow* aWindow );

/// @brief A shown top-level window by id or title glob; empty: the active one, else the first shown
KICOMMON_API wxTopLevelWindow* KopenapiFindWindow( const std::string& aWanted );

/// @brief The whole window with its canvases pasted in, in device pixels.
/// @param aCanvases receives the number of canvases pasted (optional)
/// @return false when the platform cannot draw windows offscreen
KICOMMON_API bool KopenapiCaptureWindow( wxTopLevelWindow* aWindow, wxImage& aImage, int* aCanvases = nullptr );

/// @brief The largest readable canvas inside the window (the editor's drawing area); probes by
/// reading canvases back, so resolve it once and reuse it
KICOMMON_API wxWindow* KopenapiFindCanvas( wxTopLevelWindow* aWindow );

/// @brief A canvas read back alone (from KopenapiFindCanvas)
KICOMMON_API bool KopenapiCaptureCanvas( wxWindow* aCanvas, wxImage& aImage );

/// @brief A canvas rendered offscreen at aWidth x aHeight pixels (sharp at any video size), when its
/// kiface supports it (the 3D viewer); false otherwise
KICOMMON_API bool KopenapiRenderCanvas( wxWindow* aCanvas, int aWidth, int aHeight, wxImage& aImage, int aSupersample = 1 );

#endif
