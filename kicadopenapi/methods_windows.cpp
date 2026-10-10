/// @file methods_windows.cpp
/// @brief kicadopenapi windows of the process (GUI): window_list (which app each window is and
/// what it has loaded), window_capture, window_set.
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

#include <frame_type.h>
#include <kiplatform/ui.h>
#include <kiway.h>
#include <kiway_player.h>
#include <project.h>

#include <wx/app.h>
#include <wx/dialog.h>
#include <wx/image.h>
#include <wx/toplevel.h>
#include <wx/window.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <sstream>
#include <typeinfo>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


/// @brief The KiCad application a frame type stands for
const char* appOf( FRAME_T aType )
{
    switch( aType )
    {
    case FRAME_SCH:                return "schematic_editor";
    case FRAME_SCH_SYMBOL_EDITOR:  return "symbol_editor";
    case FRAME_SCH_VIEWER:         return "symbol_viewer";
    case FRAME_SIMULATOR:          return "simulator";
    case FRAME_PCB_EDITOR:         return "board_editor";
    case FRAME_FOOTPRINT_EDITOR:   return "footprint_editor";
    case FRAME_FOOTPRINT_VIEWER:   return "footprint_viewer";
    case FRAME_PCB_DISPLAY3D:      return "3d_viewer";
    case FRAME_CVPCB:              return "footprint_assignment";
    case FRAME_GERBER:             return "gerber_viewer";
    case FRAME_PL_EDITOR:          return "drawing_sheet_editor";
    case FRAME_BM2CMP:             return "image_converter";
    case FRAME_CALC:               return "calculator";
    case KICAD_MAIN_FRAME_T:       return "project_manager";
    default:                       return "other";
    }
}


/// @brief The KiCad frames of this process by window: the kiway's players, its top frame (the
/// project manager) and the 3D viewer (recognised by its type, owned by the board editor)
std::map<wxWindow*, std::pair<EDA_BASE_FRAME*, FRAME_T>> kicadFrames( KIWAY* aKiway )
{
    std::map<wxWindow*, std::pair<EDA_BASE_FRAME*, FRAME_T>> frames;

    if( !aKiway )
        return frames;

    for( int t = 0; t < KIWAY_PLAYER_COUNT; ++t )
    {
        if( KIWAY_PLAYER* player = aKiway->Player( (FRAME_T) t, false ) )
            frames[player] = { player, (FRAME_T) t };
    }

    if( wxFrame* top = aKiway->GetTop() )
    {
        if( !frames.count( top ) )
            frames[top] = { static_cast<EDA_BASE_FRAME*>( top ), KICAD_MAIN_FRAME_T };
    }

    for( wxWindow* window : wxTopLevelWindows )
    {
        // the 3D viewer is a KIWAY_PLAYER (an EDA_BASE_FRAME) but not in the kiway's table
        if( !frames.count( window ) && std::strstr( typeid( *window ).name(), "EDA_3D_VIEWER_FRAME" ) )
            frames[window] = { static_cast<EDA_BASE_FRAME*>( static_cast<wxFrame*>( window ) ), FRAME_PCB_DISPLAY3D };
    }

    return frames;
}


/// @brief What a KiCad frame is and has loaded: app, document (the 3D viewer: its board's),
/// unsaved changes, project
void describeFrame( EDA_BASE_FRAME* aFrame, FRAME_T aType, KIWAY* aKiway, nlohmann::json& aOut )
{
    aOut["app"] = appOf( aType );

    wxString document = aFrame->GetCurrentFileName();

    if( aType == FRAME_PCB_DISPLAY3D )
    {
        if( KIWAY_PLAYER* board = aKiway->Player( FRAME_PCB_EDITOR, false ) )
        {
            document = board->GetCurrentFileName();
            aOut["shows_window"] = KopenapiWindowId( board );
        }
    }

    aOut["document"] = document.IsEmpty() ? nlohmann::json() : nlohmann::json( str( document ) );

    if( aType != FRAME_PCB_DISPLAY3D && aType != KICAD_MAIN_FRAME_T )
        aOut["unsaved"] = aFrame->IsContentModified();

    const wxString project = aKiway->Prj().GetProjectFullName();
    aOut["project"] = project.IsEmpty() ? nlohmann::json() : nlohmann::json( str( project ) );
}


nlohmann::json windowJson( wxTopLevelWindow* aWindow, KIWAY* aKiway )
{
    const wxRect   rect = aWindow->GetScreenRect();
    nlohmann::json out = { { "app", "other" } };
    const auto     frames = kicadFrames( aKiway );

    if( auto it = frames.find( aWindow ); it != frames.end() )
    {
        describeFrame( it->second.first, it->second.second, aKiway, out );
    }
    else if( dynamic_cast<wxDialog*>( aWindow ) )
    {
        out["app"] = "dialog";

        if( wxWindow* parent = aWindow->GetParent() )
            out["parent"] = KopenapiWindowId( wxGetTopLevelParent( parent ) );
    }

    out.update( nlohmann::json{ { "id", KopenapiWindowId( aWindow ) },
             { "title", str( aWindow->GetTitle() ) },
             { "class", str( aWindow->GetClassInfo() ? wxString( aWindow->GetClassInfo()->GetClassName() ) : wxString() ) },
             { "shown", aWindow->IsShown() },
             { "active", aWindow->IsActive() },
             { "iconized", aWindow->IsIconized() },
             { "x", rect.x },
             { "y", rect.y },
             { "width", rect.width },
             { "height", rect.height } } );
    return out;
}

} // namespace


static KOPENAPI_RESULT h_window_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const bool     all = aArgs.value( "include_hidden", false );
    nlohmann::json windows = nlohmann::json::array();

    for( wxWindow* window : wxTopLevelWindows )
    {
        auto* tlw = dynamic_cast<wxTopLevelWindow*>( window );

        if( tlw && ( all || tlw->IsShown() ) )
            windows.push_back( windowJson( tlw, aCtx.kiway ) );
    }

    return KOPENAPI_RESULT::Ok( { { "windows", windows } } );
}


static KOPENAPI_RESULT h_window_capture( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
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
    result["window"] = windowJson( target, aCtx.kiway );
    result["canvases"] = canvases;
    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "window_list",
                   "List the windows of this KiCad process and what each one is: app (project_manager, "
                   "schematic_editor, board_editor, 3d_viewer, symbol_editor, footprint_editor, ..., dialog), "
                   "the document loaded (the 3D viewer: its board, shows_window), unsaved changes, project, a "
                   "dialog's parent; id, title, class, shown / active / minimized, position and size; GUI only",
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


static KOPENAPI_RESULT h_window_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
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

    return KOPENAPI_RESULT::Ok( windowJson( target, aCtx.kiway ) );
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
