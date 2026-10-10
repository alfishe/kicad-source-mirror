/// @file kicadopenapi_capture.cpp
/// @brief Window and canvas images for window_capture and recordings (see kicadopenapi_capture.h).
#include <kicadopenapi_capture.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>

#include <kiplatform/ui.h>

#include <map>

#include <wx/app.h>
#include <wx/image.h>
#include <wx/toplevel.h>
#include <wx/window.h>

#include <cmath>
#include <cstdint>
#include <sstream>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


int pasteCanvases( wxWindow* aRoot, wxWindow* aWindow, wxImage& aImage, double aScale,
                   const std::vector<KOPENAPI_CANVAS_CAPTURE>& aCaptures )
{
    int pasted = 0;

    for( wxWindow* child : aWindow->GetChildren() )
    {
        if( !child->IsShownOnScreen() || child->IsTopLevel() )
            continue;

        // a buffer per canvas, kept between frames: the capture reads straight into it
        static std::map<wxWindow*, wxImage> buffers;

        wxImage& canvas = buffers[child];
        bool     captured = false;

        for( const KOPENAPI_CANVAS_CAPTURE& capture : aCaptures )
        {
            if( capture( child, canvas ) && canvas.IsOk() )
            {
                captured = true;
                break;
            }
        }

        if( captured )
        {
            const wxPoint origin = child->GetScreenPosition() - aRoot->ClientToScreen( wxPoint( 0, 0 ) );
            const wxSize  size = child->GetClientSize();
            const int     w = std::lround( size.x * aScale );
            const int     h = std::lround( size.y * aScale );

            if( w > 0 && h > 0 )
            {
                if( canvas.GetWidth() != w || canvas.GetHeight() != h )
                    canvas.Rescale( w, h, wxIMAGE_QUALITY_HIGH );

                canvas.ClearAlpha();
                aImage.Paste( canvas, std::lround( origin.x * aScale ), std::lround( origin.y * aScale ) );
                pasted++;
            }

            continue;   // a canvas has no children worth composing
        }

        pasted += pasteCanvases( aRoot, child, aImage, aScale, aCaptures );
    }

    return pasted;
}


/// @brief The biggest shown child the canvas captures accept
wxWindow* largestCanvas( wxWindow* aWindow, const std::vector<KOPENAPI_CANVAS_CAPTURE>& aCaptures, long& aArea )
{
    wxWindow* best = nullptr;

    for( wxWindow* child : aWindow->GetChildren() )
    {
        if( !child->IsShownOnScreen() || child->IsTopLevel() )
            continue;

        const wxSize size = child->GetClientSize();
        wxImage      probe;
        bool         isCanvas = false;

        if( long( size.x ) * size.y > aArea )
        {
            for( const KOPENAPI_CANVAS_CAPTURE& capture : aCaptures )
            {
                if( capture( child, probe ) && probe.IsOk() )
                {
                    isCanvas = true;
                    break;
                }
            }
        }

        if( isCanvas )
        {
            best = child;
            aArea = long( size.x ) * size.y;
            continue;
        }

        if( wxWindow* inner = largestCanvas( child, aCaptures, aArea ) )
            best = inner;
    }

    return best;
}

} // namespace


std::string KopenapiWindowId( const wxWindow* aWindow )
{
    std::ostringstream id;
    id << "w" << std::hex << reinterpret_cast<std::uintptr_t>( aWindow );
    return id.str();
}


wxTopLevelWindow* KopenapiFindWindow( const std::string& aWanted )
{
    for( wxWindow* window : wxTopLevelWindows )
    {
        auto* tlw = dynamic_cast<wxTopLevelWindow*>( window );

        if( !tlw || !tlw->IsShown() )
            continue;

        if( aWanted.empty() ? tlw->IsActive()
                            : ( KopenapiWindowId( tlw ) == aWanted || KopenapiGlob( aWanted, str( tlw->GetTitle() ) ) ) )
        {
            return tlw;
        }
    }

    // No active window (the agent's terminal has the focus): the first shown one
    if( aWanted.empty() )
    {
        for( wxWindow* window : wxTopLevelWindows )
        {
            if( auto* tlw = dynamic_cast<wxTopLevelWindow*>( window ); tlw && tlw->IsShown() )
                return tlw;
        }
    }

    return nullptr;
}


bool KopenapiCaptureWindow( wxTopLevelWindow* aWindow, wxImage& aImage, int* aCanvases )
{
    if( !KIPLATFORM::UI::CaptureWindow( aWindow, aImage ) || !aImage.IsOk() )
        return false;

    // Device pixels per window unit (Retina / HiDPI)
    const wxSize client = aWindow->GetClientSize();
    const double scale = client.x > 0 ? double( aImage.GetWidth() ) / client.x : 1.0;
    const int    pasted = pasteCanvases( aWindow, aWindow, aImage, scale, KOPENAPI_REGISTRY::Get().CanvasCaptures() );

    if( aCanvases )
        *aCanvases = pasted;

    return true;
}


int KopenapiPasteCanvases( wxTopLevelWindow* aWindow, wxImage& aImage )
{
    const wxSize client = aWindow->GetClientSize();
    const double scale = client.x > 0 ? double( aImage.GetWidth() ) / client.x : 1.0;
    return pasteCanvases( aWindow, aWindow, aImage, scale, KOPENAPI_REGISTRY::Get().CanvasCaptures() );
}


wxWindow* KopenapiFindCanvas( wxTopLevelWindow* aWindow )
{
    long area = 0;
    return largestCanvas( aWindow, KOPENAPI_REGISTRY::Get().CanvasCaptures(), area );
}


bool KopenapiCaptureCanvas( wxWindow* aCanvas, wxImage& aImage )
{
    for( const KOPENAPI_CANVAS_CAPTURE& capture : KOPENAPI_REGISTRY::Get().CanvasCaptures() )
    {
        if( capture( aCanvas, aImage ) && aImage.IsOk() )
        {
            aImage.ClearAlpha();
            return true;
        }
    }

    return false;
}


bool KopenapiRenderCanvas( wxWindow* aCanvas, int aWidth, int aHeight, wxImage& aImage, int aSupersample )
{
    for( const KOPENAPI_CANVAS_RENDER& render : KOPENAPI_REGISTRY::Get().CanvasRenders() )
    {
        if( render( aCanvas, aWidth, aHeight, aSupersample, aImage ) && aImage.IsOk() )
            return true;
    }

    return false;
}
