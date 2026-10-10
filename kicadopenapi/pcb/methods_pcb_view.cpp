/// @file methods_pcb_view.cpp
/// @brief kicadopenapi board-side captures: pcb_view_capture (board editor, footprint editor, footprint
/// viewer, 3D viewer canvases as shown) and the canvas read-back window_capture composes.
#include "kopenapi_pcb.h"

#include <3d_canvas/eda_3d_canvas.h>
#include <3d_viewer/eda_3d_viewer_frame.h>
#include <class_draw_panel_gal.h>
#include <kicadopenapi_image.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <pcb_base_frame.h>

#include <wx/image.h>

#include <algorithm>


static KOPENAPI_RESULT h_pcb_view_capture( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string editor = aArgs.value( "editor", std::string( "board" ) );
    wxImage           image;

    if( editor == "3d" )
    {
        auto* frame = aCtx.kiway ? dynamic_cast<EDA_3D_VIEWER_FRAME*>( aCtx.kiway->Player( FRAME_PCB_DISPLAY3D, false ) )
                                 : nullptr;

        // open: true shows the board's 3D viewer first (View > 3D Viewer)
        if( !frame && aArgs.value( "open", false ) && aCtx.kiway )
        {
            if( auto* board = dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
                frame = board->CreateAndShow3D_Frame();
        }

        if( !frame || !frame->GetCanvas() || !frame->IsShown() )
            return KOPENAPI_RESULT::Error( 409, "no 3D viewer window open (open: true opens it from the board editor)" );

        // Draw now (a just-opened viewer may not have painted yet), then read the frame buffer
        frame->GetCanvas()->DoRePaint();
        frame->GetCanvas()->GetScreenshot( image );
    }
    else
    {
        FRAME_T type;

        if( editor == "board" )
            type = FRAME_PCB_EDITOR;
        else if( editor == "footprint" )
            type = FRAME_FOOTPRINT_EDITOR;
        else if( editor == "footprint_viewer" )
            type = FRAME_FOOTPRINT_VIEWER;
        else
            return KOPENAPI_RESULT::Error( 400, "editor must be board, footprint, footprint_viewer or 3d" );

        auto* frame = aCtx.kiway ? dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( type, false ) ) : nullptr;

        if( !frame || !frame->GetCanvas() || !frame->IsShown() )
            return KOPENAPI_RESULT::Error( 409, "no " + editor + " window open" );

        frame->GetCanvas()->GetScreenshot( image );
    }

    if( !image.IsOk() )
        return KOPENAPI_RESULT::Error( 500, "the canvas could not be read (not OpenGL, or not drawn yet)" );

    nlohmann::json result = KopenapiImageResult( image, std::clamp( aArgs.value( "max_width", 1600 ), 200, 8000 ) );
    result["editor"] = editor;
    return KOPENAPI_RESULT::Ok( result );
}


/// @brief window_capture: board-side OpenGL canvases (2D and 3D) read back in place
KOPENAPI_REGISTER_CANVAS_CAPTURE(
        []( wxWindow* aWindow, wxImage& aImage ) -> bool
        {
            if( auto* canvas = dynamic_cast<EDA_DRAW_PANEL_GAL*>( aWindow ) )
                return canvas->GetScreenshot( aImage ) && aImage.IsOk();

            if( auto* canvas3d = dynamic_cast<EDA_3D_CANVAS*>( aWindow ) )
            {
                canvas3d->GetScreenshot( aImage );
                return aImage.IsOk();
            }

            return false;
        } );


KOPENAPI_REGISTER( "pcb_view_capture",
                   "Capture what a board-side window shows right now (its canvas: zoom, pan, selection as "
                   "on screen): board editor, footprint editor, footprint viewer or 3D viewer; PNG; GUI "
                   "only; no OS screen capture. For the whole window with toolbars use window_capture",
                   R"json({"type":"object","properties":{
                        "editor":{"type":"string","enum":["board","footprint","footprint_viewer","3d"],"default":"board"},
                        "open":{"type":"boolean","default":false,"description":"3d: open the 3D viewer if it is not open"},
                        "max_width":{"type":"integer","default":1600}}})json"_json,
                   true, h_pcb_view_capture );
