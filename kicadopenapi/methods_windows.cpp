/// @file methods_windows.cpp
/// @brief kicadopenapi windows of the process (GUI): window_list, window_capture.
///
/// window_capture renders a whole window — toolbars, panels, dialogs, the canvas — into an image
/// without reading the screen: the toolkit draws the window offscreen (KIPLATFORM::UI::
/// CaptureWindow, per OS), and the OpenGL canvases inside it, which that leaves blank, are read
/// back by the kifaces that own them (KOPENAPI_REGISTER_CANVAS_CAPTURE) and pasted in place.
/// No screen-recording permission, covered windows still capture.
#include <kicadopenapi_capture.h>
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



std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


nlohmann::json windowJson( wxTopLevelWindow* aWindow )
{
    const wxRect rect = aWindow->GetScreenRect();

    return { { "id", KopenapiWindowId( aWindow ) },
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


/// @brief Paste every canvas a kiface can read on top of the window image

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
    wxTopLevelWindow* target = KopenapiFindWindow( aArgs.value( "window", std::string() ) );

    if( !target )
        return KOPENAPI_RESULT::Error( 404, "window not found (see window_list: id or title glob)" );

    if( target->IsIconized() )
        return KOPENAPI_RESULT::Error( 409, "the window is minimized" );

    wxImage image;
    int     canvases = 0;

    if( !KopenapiCaptureWindow( target, image, &canvases ) )
        return KOPENAPI_RESULT::Error( 501, "this platform cannot draw windows offscreen (e.g. Wayland); "
                                            "sch_view_capture / pcb_view_capture still read the canvases" );

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


static KOPENAPI_RESULT h_window_set( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    wxTopLevelWindow* target = KopenapiFindWindow( aArgs.value( "window", std::string() ) );

    if( !target )
        return KOPENAPI_RESULT::Error( 404, "window not found (see window_list: id or title glob)" );

    if( target->IsIconized() )
        target->Iconize( false );

    if( target->IsMaximized() && ( aArgs.contains( "width" ) || aArgs.contains( "x" ) ) )
        target->Maximize( false );

    wxPoint pos = target->GetPosition();
    wxSize  size = target->GetSize();
    pos.x = aArgs.value( "x", pos.x );
    pos.y = aArgs.value( "y", pos.y );
    size.x = std::max( 200, aArgs.value( "width", size.x ) );
    size.y = std::max( 150, aArgs.value( "height", size.y ) );
    target->SetSize( wxRect( pos, size ) );

    if( aArgs.value( "raise", false ) )
        target->Raise();

    return KOPENAPI_RESULT::Ok( windowJson( target ) );
}


KOPENAPI_REGISTER( "window_set",
                   "Move / resize a KiCad window (screen points): x, y, width, height, raise; e.g. the same "
                   "editor size for every scene of a recording; GUI only",
                   R"json({"type":"object","required":["window"],"properties":{
                        "window":{"type":"string","description":"id or title glob from window_list"},
                        "x":{"type":"integer"}, "y":{"type":"integer"},
                        "width":{"type":"integer"}, "height":{"type":"integer"},
                        "raise":{"type":"boolean","default":false}}})json"_json,
                   true, h_window_set );
