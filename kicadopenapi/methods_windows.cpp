/*
 * kicadopenapi windows of the process (GUI): window_list, window_capture.
 *
 * window_capture renders a whole window — toolbars, panels, dialogs, the canvas — into an image
 * without reading the screen: the toolkit draws the window offscreen (KIPLATFORM::UI::
 * CaptureWindow, per OS), and the OpenGL canvases inside it, which that leaves blank, are read
 * back by the kifaces that own them (KOPENAPI_REGISTER_CANVAS_CAPTURE) and pasted in place.
 * No screen-recording permission, covered windows still capture.
 */
#include <kicadopenapi_image.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>

#include <kiplatform/ui.h>

#include <wx/app.h>
#include <wx/image.h>
#include <wx/toplevel.h>
#include <wx/window.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>


namespace
{

std::string windowId( const wxWindow* aWindow )
{
    std::ostringstream id;
    id << "w" << std::hex << reinterpret_cast<std::uintptr_t>( aWindow );
    return id.str();
}


std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


nlohmann::json windowJson( wxTopLevelWindow* aWindow )
{
    const wxRect rect = aWindow->GetScreenRect();

    return { { "id", windowId( aWindow ) },
             { "title", str( aWindow->GetTitle() ) },
             { "class", str( aWindow->GetClassInfo() ? wxString( aWindow->GetClassInfo()->GetClassName() ) : wxString() ) },
             { "shown", aWindow->IsShown() },
             { "active", aWindow->IsActive() },
             { "iconized", aWindow->IsIconized() },
             { "x", rect.x },
             { "y", rect.y },
             { "width", rect.width },
             { "height", rect.height } };
}


/// Paste every canvas a kiface can read on top of the window image
int pasteCanvases( wxWindow* aRoot, wxWindow* aWindow, wxImage& aImage, double aScale,
                   const std::vector<KOPENAPI_CANVAS_CAPTURE>& aCaptures )
{
    int pasted = 0;

    for( wxWindow* child : aWindow->GetChildren() )
    {
        if( !child->IsShownOnScreen() || child->IsTopLevel() )
            continue;

        wxImage canvas;
        bool    captured = false;

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

} // namespace


static KOPENAPI_RESULT h_window_list( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    const bool     all = aArgs.value( "include_hidden", false );
    nlohmann::json windows = nlohmann::json::array();

    for( wxWindow* window : wxTopLevelWindows )
    {
        auto* tlw = dynamic_cast<wxTopLevelWindow*>( window );

        if( tlw && ( all || tlw->IsShown() ) )
            windows.push_back( windowJson( tlw ) );
    }

    return KOPENAPI_RESULT::Ok( { { "windows", windows } } );
}


static KOPENAPI_RESULT h_window_capture( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    const std::string wanted = aArgs.value( "window", std::string() );
    wxTopLevelWindow* target = nullptr;

    for( wxWindow* window : wxTopLevelWindows )
    {
        auto* tlw = dynamic_cast<wxTopLevelWindow*>( window );

        if( !tlw || !tlw->IsShown() )
            continue;

        if( wanted.empty() ? tlw->IsActive() : ( windowId( tlw ) == wanted || KopenapiGlob( wanted, str( tlw->GetTitle() ) ) ) )
        {
            target = tlw;
            break;
        }
    }

    // No active window (the agent's terminal has the focus): take the first shown one
    if( !target && wanted.empty() )
    {
        for( wxWindow* window : wxTopLevelWindows )
        {
            if( auto* tlw = dynamic_cast<wxTopLevelWindow*>( window ); tlw && tlw->IsShown() )
            {
                target = tlw;
                break;
            }
        }
    }

    if( !target )
        return KOPENAPI_RESULT::Error( 404, "window not found (see window_list: id or title glob)" );

    if( target->IsIconized() )
        return KOPENAPI_RESULT::Error( 409, "the window is minimized" );

    wxImage image;

    if( !KIPLATFORM::UI::CaptureWindow( target, image ) || !image.IsOk() )
        return KOPENAPI_RESULT::Error( 501, "this platform cannot draw windows offscreen (e.g. Wayland); "
                                            "sch_view_capture / pcb_view_capture still read the canvases" );

    // Device pixels per window unit (Retina / HiDPI)
    const wxSize client = target->GetClientSize();
    const double scale = client.x > 0 ? double( image.GetWidth() ) / client.x : 1.0;
    const int    canvases = pasteCanvases( target, target, image, scale, KOPENAPI_REGISTRY::Get().CanvasCaptures() );

    nlohmann::json result = KopenapiImageResult( image, std::clamp( aArgs.value( "max_width", 1600 ), 200, 8000 ) );
    result["window"] = windowJson( target );
    result["canvases"] = canvases;
    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "window_list",
                   "List the windows of this KiCad process (project manager, editors, 3D viewer, dialogs): "
                   "id, title, class, shown / active / minimized, position and size; GUI only",
                   R"json({"type":"object","properties":{
                        "include_hidden":{"type":"boolean","default":false}}})json"_json,
                   true, h_window_list );

KOPENAPI_REGISTER( "window_capture",
                   "Screenshot of a whole KiCad window (toolbars, panels, dialogs and the drawing canvas as "
                   "shown) rendered by the process itself — no OS screen capture or permission; covered "
                   "windows work. Window by id or title glob from window_list (default: the active one). "
                   "Over MCP an image; GUI only",
                   R"json({"type":"object","properties":{
                        "window":{"type":"string","description":"id or title glob, e.g. *Schematic Editor*"},
                        "max_width":{"type":"integer","default":1600}}})json"_json,
                   true, h_window_capture, 60 );
