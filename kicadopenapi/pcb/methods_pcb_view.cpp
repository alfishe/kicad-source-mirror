/// @file methods_pcb_view.cpp
/// @brief kicadopenapi board-side views: pcb_view_capture (board editor, footprint editor, footprint
/// viewer, 3D viewer canvases as shown), the canvas read-back window_capture composes,
/// pcb_view_zoom, view3d_camera, view3d_orbit.
#include "kopenapi_pcb.h"

#include <3d_canvas/eda_3d_canvas.h>
#include <3d_viewer/eda_3d_viewer_frame.h>
#include <3d_viewer/eda_3d_viewer_settings.h>
#include <class_draw_panel_gal.h>
#include <view/view.h>
#include <kicadopenapi_image.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <pcb_base_frame.h>
#include <board.h>
#include <footprint.h>
#include <pad.h>
#include <board_connected_item.h>
#include <netinfo.h>
#include <frame_type.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_gal_render.h>
#include <kicadopenapi_steadycam.h>
#include <kicadopenapi_view_motion.h>
#include <gal/3d/camera.h>
#include <glm/gtc/quaternion.hpp>
#include <3d_enums.h>
#include <3d_canvas/board_adapter.h>
#include <3d_rendering/raytracing/shapes3D/bbox_3d.h>
#include <3d_rendering/render_3d_base.h>
#include <3d_rendering/lead_clip.h>
#include <3d_cache/3d_cache.h>
#include <3d_math.h>
#include <filename_resolver.h>
#include <footprint_library_adapter.h>
#include <project.h>
#include <project_pcb.h>
#include <plugins/3dapi/c3dmodel.h>
#include <dialogs/appearance_controls_3D.h>
#include <3d_viewer/tools/eda_3d_actions.h>
#include <tool/tool_manager.h>
#include <functional>
#include <settings/settings_manager.h>
#include <pgm_base.h>
#include <3d_viewer/tools/eda_3d_controller.h>
#include <layer_ids.h>

#include <chrono>
#include <map>
#include <optional>
#include <set>
#include <cmath>
#include <thread>

#include <wx/image.h>

#include <algorithm>


static EDA_3D_VIEWER_FRAME* viewer3d( KOPENAPI_CONTEXT& aCtx, bool aOpen );


static KOPENAPI_RESULT h_pcb_view_capture( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string editor = aArgs.value( "editor", std::string( "board" ) );
    wxImage           image;

    if( editor == "3d" )
    {
        // open: true shows the board's 3D viewer first (View > 3D Viewer)
        EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", false ) );

        if( !frame )
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
            // the composed frame in one read (no second buffer, no CPU blend); else KiCad's screenshot
            if( auto* canvas = dynamic_cast<EDA_DRAW_PANEL_GAL*>( aWindow ) )
                return ( canvas->CaptureComposed( aImage ) || canvas->GetScreenshot( aImage ) ) && aImage.IsOk();

            // the 3D canvas drawn offscreen at its own size in its own GL context (its screenshot
            // reads whatever context is current, which may be another editor's)
            if( auto* canvas3d = dynamic_cast<EDA_3D_CANVAS*>( aWindow ) )
            {
                const wxSize size = canvas3d->GetNativePixelSize();

                if( size.x <= 0 || size.y <= 0 )
                    return false;

                aImage.Create( size.x, size.y, false );
                return canvas3d->RenderToImage( aImage.GetData(), size.x, size.y, 1 ) && aImage.IsOk();
            }

            return false;
        } );


/// @brief 3D camera pose for the steadycam: rotation and log zoom
struct STEADY_3D_POSE
{
    glm::quat rotation{ 1, 0, 0, 0 };
    double    logZoom = 0;
};


struct STEADY_3D_OPS
{
    /// @brief The turn from a to b as axis * angle, with the zoom change as the fourth part
    static glm::dvec4 delta( const STEADY_3D_POSE& a, const STEADY_3D_POSE& b )
    {
        glm::quat d = b.rotation * glm::inverse( a.rotation );

        if( d.w < 0 )
            d = -d;   // the short way round

        const double angle = 2 * std::acos( std::clamp( (double) d.w, -1.0, 1.0 ) );
        const double sinHalf = std::sqrt( std::max( 0.0, 1.0 - (double) d.w * d.w ) );
        const glm::dvec3 axis = sinHalf > 1e-9 ? glm::dvec3( d.x, d.y, d.z ) / sinHalf : glm::dvec3( 0 );
        return glm::dvec4( axis * angle, b.logZoom - a.logZoom );
    }

    STEADY_3D_POSE lerp( const STEADY_3D_POSE& a, const STEADY_3D_POSE& b, double s ) const
    {
        glm::quat target = b.rotation;

        if( glm::dot( a.rotation, target ) < 0 )
            target = -target;

        return { glm::normalize( glm::slerp( a.rotation, target, (float) s ) ), a.logZoom + ( b.logZoom - a.logZoom ) * s };
    }

    double distance( const STEADY_3D_POSE& a, const STEADY_3D_POSE& b ) const { return glm::length( delta( a, b ) ); }

    double rate( const STEADY_3D_POSE& aFrom, const STEADY_3D_POSE& aTo, const STEADY_3D_POSE& aShown,
                 const STEADY_3D_POSE& aTarget ) const
    {
        const glm::dvec4 v = delta( aFrom, aTo ), d = delta( aShown, aTarget );
        const double     len2 = glm::dot( d, d );
        return len2 > 1e-12 ? glm::dot( v, d ) / len2 : 0.0;
    }
};


/// @brief Recordings' steadycam: board canvas (GAL) and the 3D viewer's camera
static bool steadyBoard( wxWindow* aWindow, double aDt, const KOPENAPI_STEADY& aSteady, bool aExact,
                         std::function<void()>& aRestore )
{
    if( KopenapiSteadyGal( aWindow, aDt, aSteady, aExact, aRestore ) )
        return true;

    auto* canvas = dynamic_cast<EDA_3D_CANVAS*>( aWindow );
    auto* viewer = canvas ? dynamic_cast<EDA_3D_VIEWER_FRAME*>( wxGetTopLevelParent( canvas ) ) : nullptr;

    if( !viewer )
        return false;

    static std::map<wxWindow*, KOPENAPI_STEADY_TRACK<STEADY_3D_POSE>> tracks;

    CAMERA&              camera = viewer->GetCurrentCamera();
    const STEADY_3D_POSE now{ glm::normalize( glm::quat_cast( glm::mat3( camera.GetRotationMatrix() ) ) ),
                              std::log( std::max( 1e-6f, camera.GetZoom() ) ) };
    auto                 it = tracks.find( aWindow );

    if( it == tracks.end() || aDt > 0.5 || aExact || aSteady.mode == KOPENAPI_STEADY::OFF )
    {
        tracks[aWindow].reset( now );
        return true;
    }

    const STEADY_3D_POSE show = it->second.step( now, aDt, aSteady, STEADY_3D_OPS() );

    if( STEADY_3D_OPS().distance( show, now ) < 1e-9 )
        return true;

    // the camera's real state comes back after the frame: KiCad's own presets and animations keep
    // working on it undisturbed
    const glm::mat4 realRotation = camera.GetRotationMatrix();
    const float     realZoom = camera.GetZoom();

    // RotateX( 0 ) refreshes the view matrix (SetRotationMatrix alone does not)
    camera.SetRotationMatrix( glm::mat4_cast( show.rotation ) );
    camera.RotateX( 0.0f );
    camera.Zoom( (float) ( camera.GetZoom() / std::exp( show.logZoom ) ) );

    aRestore = [&camera, realRotation, realZoom]()
    {
        camera.SetRotationMatrix( realRotation );
        camera.RotateX( 0.0f );
        camera.Zoom( camera.GetZoom() / realZoom );
    };

    return true;
}

KOPENAPI_REGISTER_CANVAS_STEADY( steadyBoard );


/// @brief Recordings: board canvas (GAL, composed frame scaled on the GPU) and the 3D viewer drawn
/// offscreen at the video's size from its loaded scene
KOPENAPI_REGISTER_CANVAS_RENDER(
        []( wxWindow* aWindow, int aWidth, int aHeight, int aSupersample, wxImage& aImage ) -> bool
        {
            if( KopenapiRenderGal( aWindow, aWidth, aHeight, aSupersample, aImage ) )
                return true;

            auto* canvas = dynamic_cast<EDA_3D_CANVAS*>( aWindow );

            if( !canvas || aWidth <= 0 || aHeight <= 0 )
                return false;

            // like KiCad's own image export: no axis navigator, no rollover highlight in the frame
            auto*                                     viewer = dynamic_cast<EDA_3D_VIEWER_FRAME*>( wxGetTopLevelParent( canvas ) );
            EDA_3D_VIEWER_SETTINGS::RENDER_SETTINGS* render = viewer && viewer->GetAdapter().m_Cfg
                                                                      ? &viewer->GetAdapter().m_Cfg->m_Render
                                                                      : nullptr;
            const bool navigator = render && render->show_navigator;
            const bool rollover = render && render->highlight_on_rollover;

            if( render )
            {
                render->show_navigator = false;
                render->highlight_on_rollover = false;
            }

            // drawn on the GPU at the asked size (supersampled and averaged there), read as RGB; the
            // window keeps its own on-screen rendering
            aImage.Create( aWidth, aHeight, false );
            const bool ok = canvas->RenderToImage( aImage.GetData(), aWidth, aHeight, std::max( 1, aSupersample ) );

            if( render )
            {
                render->show_navigator = navigator;
                render->highlight_on_rollover = rollover;
            }

            return ok;
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


static KOPENAPI_RESULT h_pcb_view_zoom( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;

    if( !frame || !frame->GetCanvas() || !frame->IsShown() || !frame->GetBoard() )
        return KOPENAPI_RESULT::Error( 409, "no board editor window open" );

    KOPENAPI_REMOTE_GUARD remote( frame );
    BOARD*       board = frame->GetBoard();
    const double margin = pcbIUScale.mmToIU( aArgs.value( "margin_mm", 2.0 ) );
    BOX2I        area;

    if( aArgs.contains( "area_mm" ) )
    {
        const nlohmann::json& a = aArgs["area_mm"];

        if( !a.is_array() || a.size() != 4 )
            return KOPENAPI_RESULT::Error( 400, "area_mm: [x0, y0, x1, y1]" );

        const VECTOR2I p0( pcbIUScale.mmToIU( a[0].get<double>() ), pcbIUScale.mmToIU( a[1].get<double>() ) );
        const VECTOR2I p1( pcbIUScale.mmToIU( a[2].get<double>() ), pcbIUScale.mmToIU( a[3].get<double>() ) );
        area = BOX2I( p0, p1 - p0 );
        area.Normalize();
    }
    else if( aArgs.contains( "refs" ) )
    {
        bool any = false;

        for( const nlohmann::json& r : aArgs["refs"] )
        {
            for( FOOTPRINT* fp : board->Footprints() )
            {
                if( fp->GetReference().ToStdString( wxConvUTF8 ) != r.get<std::string>() )
                    continue;

                const BOX2I b = fp->GetBoundingBox( false );
                area = any ? area.Merge( b ) : b;
                any = true;
            }
        }

        if( !any )
            return KOPENAPI_RESULT::Error( 404, "none of refs is on the board" );
    }
    else
    {
        // fit: the board outline, else everything
        area = board->GetBoardEdgesBoundingBox();

        if( area.GetWidth() <= 0 )
            area = board->GetBoundingBox();
    }

    // the side looked at (View > Flip Board View): back = from below, for bottom-side work
    if( aArgs.contains( "side" ) )
    {
        const std::string side = aArgs["side"].get<std::string>();
        const bool        flipped = frame->GetDisplayOptions().m_FlipBoardView;
        const bool        want = side == "flip" ? !flipped : side == "back";

        if( side != "front" && side != "back" && side != "flip" )
            return KOPENAPI_RESULT::Error( 400, "side: front, back or flip" );

        if( want != flipped )
        {
            PCB_DISPLAY_OPTIONS opts = frame->GetDisplayOptions();
            opts.m_FlipBoardView = want;
            frame->SetDisplayOptions( opts );
        }
    }

    area.Inflate( margin );
    if( !KopenapiMoveView( frame->GetCanvas(), BOX2D( area.GetOrigin(), area.GetSize() ),
                           std::clamp( aArgs.value( "animate_ms", 600 ), 0, 5000 ) ) )
    {
        return KOPENAPI_RESULT::Error( 409, "the editor was closed during the move" );
    }

    const BOX2D v = frame->GetCanvas()->GetView()->GetViewport();
    auto        mm = []( double x ) { return std::round( x / pcbIUScale.IU_PER_MM * 100 ) / 100; };
    return KOPENAPI_RESULT::Ok( { { "viewport_mm", { mm( v.GetLeft() ), mm( v.GetTop() ), mm( v.GetRight() ), mm( v.GetBottom() ) } },
                                  { "side", frame->GetDisplayOptions().m_FlipBoardView ? "back" : "front" } } );
}


void KopenapiRefresh3D( PCB_BASE_FRAME* aFrame )
{
    EDA_3D_VIEWER_FRAME* viewer = aFrame ? aFrame->Get3DViewerFrame() : nullptr;

    if( !viewer )
        return;

    if( EDA_3D_CANVAS* canvas = viewer->GetCanvas() )
        canvas->HoldFrameUntilLoaded();

    aFrame->Update3DView( true, true );
}


void KopenapiPcbBrighten( PCB_BASE_FRAME* aFrame, EDA_ITEM* aItem, bool aOn )
{
    KIGFX::VIEW* view = aFrame->GetCanvas()->GetView();

    auto one = [&]( EDA_ITEM* aOne )
    {
        if( aOn )
            aOne->SetBrightened();
        else
            aOne->ClearBrightened();

        // the painter frames brightened markers, images and drill maps; everything else only
        // changes colour, and a recolour keeps the cached geometry (text shadows, pad shapes)
        const KICAD_T type = aOne->Type();
        const bool    reshaped = type == PCB_MARKER_T || type == PCB_REFERENCE_IMAGE_T || type == PCB_DRILL_MAP_T;

        view->Update( aOne, reshaped ? KIGFX::REPAINT : KIGFX::COLOR );
    };

    one( aItem );

    // a footprint's pads, texts and graphics are drawn on their own
    if( aItem->Type() == PCB_FOOTPRINT_T )
        static_cast<FOOTPRINT*>( aItem )->RunOnChildren( [&]( BOARD_ITEM* aChild ) { one( aChild ); },
                                                         RECURSE_MODE::RECURSE );
}


/// @brief The board's 3D viewer, opened when asked
static EDA_3D_VIEWER_FRAME* viewer3d( KOPENAPI_CONTEXT& aCtx, bool aOpen )
{
    auto* frame = aCtx.kiway ? dynamic_cast<EDA_3D_VIEWER_FRAME*>( aCtx.kiway->Player( FRAME_PCB_DISPLAY3D, false ) )
                             : nullptr;
    auto* board = aCtx.kiway ? dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) )
                             : nullptr;

    // the board editor's viewer is its child window, not a KIWAY player
    if( !frame && board )
        frame = board->Get3DViewerFrame();

    if( !frame && aOpen && board )
        frame = board->CreateAndShow3D_Frame();

    return frame && frame->GetCanvas() && frame->IsShown() ? frame : nullptr;
}


/// @brief One camera move: rotation (degrees about x, y, z) and zoom factor over some seconds
struct CAMERA_MOVE
{
    double rotate[3] = { 0, 0, 0 };
    double zoom = 1.0;
    double seconds = 1.0;

    std::optional<SFVEC3F> lookAt;        ///< where the camera looks at the end (world), if it pans
    SFVEC3F                shiftLookAt{ 0 };   ///< the pan over the move (resolveLookAt)
    SFVEC3F                shiftPos{ 0 };      ///< the camera's offset across the view taken out
};


/// @brief Board mm -> the 3D viewer's world, through the board's own 3D box (however the adapter
/// places and scales it); the board's top surface is z 0 in mm
static std::optional<SFVEC3F> boardToWorld( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard, double aX, double aY,
                                            double aZ )
{
    const BOX2I outline = aBoard ? aBoard->GetBoardEdgesBoundingBox() : BOX2I();

    if( outline.GetWidth() <= 0 || outline.GetHeight() <= 0 )
        return std::nullopt;

    const BOARD_ADAPTER& adapter = aFrame->GetAdapter();
    const SFVEC3F        bmin = adapter.GetBBox().Min();
    const SFVEC3F        bmax = adapter.GetBBox().Max();
    const double         ox = outline.GetLeft() / pcbIUScale.IU_PER_MM;
    const double         oy = outline.GetTop() / pcbIUScale.IU_PER_MM;
    const double         sx = ( bmax.x - bmin.x ) / ( outline.GetWidth() / pcbIUScale.IU_PER_MM );
    const double         sy = ( bmax.y - bmin.y ) / ( outline.GetHeight() / pcbIUScale.IU_PER_MM );

    return SFVEC3F( (float) ( bmin.x + ( aX - ox ) * sx ),
                    (float) ( bmax.y - ( aY - oy ) * sy ),   // board y runs down, the world's up
                    (float) ( bmax.z + aZ * sx ) );
}


/// @brief A look_at argument in the world: "board" (its middle), references (the middle of their
/// boxes) or [x_mm, y_mm]; empty with aError set when it does not resolve
static std::optional<SFVEC3F> lookAtOf( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard, const nlohmann::json& aArg,
                                        std::string& aError )
{
    if( !aBoard )
    {
        aError = "look_at: no board open";
        return std::nullopt;
    }

    BOX2I area;

    if( aArg.is_string() && aArg.get<std::string>() == "board" )
    {
        area = aBoard->GetBoardEdgesBoundingBox();
    }
    else if( aArg.is_array() && aArg.size() == 2 && aArg[0].is_number() && aArg[1].is_number() )
    {
        const VECTOR2I p( pcbIUScale.mmToIU( aArg[0].get<double>() ), pcbIUScale.mmToIU( aArg[1].get<double>() ) );
        area = BOX2I( p, VECTOR2I( 0, 0 ) );
    }
    else if( aArg.is_array() && !aArg.empty() )
    {
        bool any = false;

        for( const nlohmann::json& r : aArg )
        {
            if( !r.is_string() )
            {
                aError = "look_at: \"board\", [x_mm, y_mm] or references";
                return std::nullopt;
            }

            for( FOOTPRINT* fp : aBoard->Footprints() )
            {
                if( fp->GetReference().ToStdString( wxConvUTF8 ) != r.get<std::string>() )
                    continue;

                const BOX2I b = fp->GetBoundingBox( false );
                area = any ? area.Merge( b ) : b;
                any = true;
            }
        }

        if( !any )
        {
            aError = "look_at: none of the references is on the board";
            return std::nullopt;
        }
    }
    else
    {
        aError = "look_at: \"board\", [x_mm, y_mm] or references";
        return std::nullopt;
    }

    const VECTOR2I c = area.GetCenter();
    std::optional<SFVEC3F> w = boardToWorld( aFrame, aBoard, c.x / pcbIUScale.IU_PER_MM, c.y / pcbIUScale.IU_PER_MM, 0 );

    if( !w )
        aError = "look_at: the board has no outline";

    return w;
}


/// @brief Moves the point the camera turns about and looks at, and its offset across the view, by
/// these amounts (KiCad's own interpolation, rotation and zoom kept)
static void shiftCamera( CAMERA& aCamera, const SFVEC3F& aLookAt, const SFVEC3F& aPos )
{
    if( aLookAt == SFVEC3F( 0 ) && aPos == SFVEC3F( 0 ) )
        return;

    aCamera.SetT0_and_T1_current_T();
    aCamera.SetLookAtPos_T1( aCamera.GetLookAtPos() + aLookAt );
    aCamera.Pan_T1( aPos );
    aCamera.Interpolate( 1.0f );
}


/// @brief The pans of the moves from where the camera looks now: each move with a look_at ends
/// looking there, centred in the view (a manual pan across the view taken out)
static void resolveLookAt( const CAMERA& aCamera, std::vector<CAMERA_MOVE>& aMoves )
{
    SFVEC3F at = aCamera.GetLookAtPos();
    SFVEC3F pos = aCamera.GetCameraPos();

    for( CAMERA_MOVE& m : aMoves )
    {
        if( !m.lookAt )
            continue;

        m.shiftLookAt = *m.lookAt - at;
        m.shiftPos = SFVEC3F( -pos.x, -pos.y, 0 );
        at = *m.lookAt;
        pos.x = pos.y = 0;
    }
}


/// @brief Zoom correction along the moves (follow framing): (time into the moves, factor)
using FRAMING = std::vector<std::pair<double, double>>;

static nlohmann::json fitTrajectory( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard, const std::vector<CAMERA_MOVE>& aMoves,
                                    double aSeconds, FRAMING* aFollow = nullptr );
static void stepMove( CAMERA& aCamera, const CAMERA_MOVE& aMove, double aFrom, double aTo );


/// @brief The correction at time aT (linear between samples)
static double framingAt( const FRAMING& aFraming, double aT )
{
    if( aFraming.empty() )
        return 1.0;

    if( aT <= aFraming.front().first )
        return aFraming.front().second;

    for( size_t i = 1; i < aFraming.size(); ++i )
    {
        if( aT <= aFraming[i].first )
        {
            const auto& [t0, k0] = aFraming[i - 1];
            const auto& [t1, k1] = aFraming[i];
            return t1 > t0 ? k0 + ( k1 - k0 ) * ( aT - t0 ) / ( t1 - t0 ) : k1;
        }
    }

    return aFraming.back().second;
}


/// @brief The board shown in the board editor
static BOARD* boardOf( KOPENAPI_CONTEXT& aCtx )
{
    auto* frame = aCtx.kiway ? dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;
    return frame ? frame->GetBoard() : nullptr;
}


/// @brief Repaints the 3D canvas and lets the UI (recordings) see it
static void paint3d( EDA_3D_VIEWER_FRAME* aFrame )
{
    aFrame->GetCanvas()->Request_refresh( true );
    KopenapiKeepUiAlive();
}


static const std::map<std::string, int>& view3dLayers()
{
    static const std::map<std::string, int> layers = {
        { "board", LAYER_3D_BOARD },                     { "copper_top", LAYER_3D_COPPER_TOP },
        { "copper_bottom", LAYER_3D_COPPER_BOTTOM },     { "plated_barrels", LAYER_3D_PLATED_BARRELS },
        { "silkscreen_top", LAYER_3D_SILKSCREEN_TOP },   { "silkscreen_bottom", LAYER_3D_SILKSCREEN_BOTTOM },
        { "soldermask_top", LAYER_3D_SOLDERMASK_TOP },   { "soldermask_bottom", LAYER_3D_SOLDERMASK_BOTTOM },
        { "solderpaste", LAYER_3D_SOLDERPASTE },         { "adhesive", LAYER_3D_ADHESIVE },
        { "user_comments", LAYER_3D_USER_COMMENTS },     { "user_drawings", LAYER_3D_USER_DRAWINGS },
        { "user_eco1", LAYER_3D_USER_ECO1 },             { "user_eco2", LAYER_3D_USER_ECO2 },
        { "fab_top", LAYER_3D_F_FAB },                   { "fab_bottom", LAYER_3D_B_FAB },
        { "courtyard_top", LAYER_3D_F_COURTYARD },       { "courtyard_bottom", LAYER_3D_B_COURTYARD },
        { "fp_text", LAYER_FP_TEXT },                    { "fp_references", LAYER_FP_REFERENCES },
        { "fp_values", LAYER_FP_VALUES },                { "th_models", LAYER_3D_TH_MODELS },
        { "smd_models", LAYER_3D_SMD_MODELS },           { "virtual_models", LAYER_3D_VIRTUAL_MODELS },
        { "models_not_in_pos", LAYER_3D_MODELS_NOT_IN_POS }, { "models_dnp", LAYER_3D_MODELS_MARKED_DNP },
        { "bounding_boxes", LAYER_3D_BOUNDING_BOXES },   { "off_board_silk", LAYER_3D_OFF_BOARD_SILK },
        { "navigator", LAYER_3D_NAVIGATOR } };
    return layers;
}


/// @brief Layers with a colour swatch in the viewer: the shown layers and the two backgrounds
static const std::map<std::string, int>& view3dColorLayers()
{
    static const std::map<std::string, int> layers = []()
    {
        std::map<std::string, int> all = view3dLayers();
        all["background_top"] = LAYER_3D_BACKGROUND_TOP;
        all["background_bottom"] = LAYER_3D_BACKGROUND_BOTTOM;
        return all;
    }();
    return layers;
}


static KOPENAPI_RESULT h_view3d_layers( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", true ) );

    if( !frame || !frame->GetAppearanceManager() )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer (open: true opens it from the board editor)" );

    static const std::map<std::string, std::pair<MATERIAL_MODE, const TOOL_ACTION*>> materials = {
        { "realistic", { MATERIAL_MODE::NORMAL, &EDA_3D_ACTIONS::materialNormal } },
        { "diffuse", { MATERIAL_MODE::DIFFUSE_ONLY, &EDA_3D_ACTIONS::materialDiffuse } },
        { "cad", { MATERIAL_MODE::CAD_MODE, &EDA_3D_ACTIONS::materialCAD } } };

    APPEARANCE_CONTROLS_3D* panel = frame->GetAppearanceManager();
    EDA_3D_VIEWER_SETTINGS* cfg = frame->GetAdapter().m_Cfg;
    const nlohmann::json    set = aArgs.value( "set", nlohmann::json::object() );
    const std::string       material = aArgs.value( "material", std::string() );
    const nlohmann::json    colorArgs = aArgs.value( "colors", nlohmann::json::object() );
    std::map<int, COLOR4D>  colors;

    for( const auto& [name, visible] : set.items() )
    {
        if( !view3dLayers().count( name ) || !visible.is_boolean() )
            return KOPENAPI_RESULT::Error( 400, "set: {layer: true|false}, layers: see the answer's 'layers'" );
    }

    if( !material.empty() && !materials.count( material ) )
        return KOPENAPI_RESULT::Error( 400, "material: realistic | diffuse | cad" );

    for( const auto& [name, value] : colorArgs.items() )
    {
        wxColour colour( wxString::FromUTF8( value.is_string() ? value.get<std::string>() : std::string() ) );

        if( !view3dColorLayers().count( name ) || !colour.IsOk() )
            return KOPENAPI_RESULT::Error( 400, "colors: {layer: '#rrggbb' or '#rrggbbaa'}, layers as in the answer's "
                                                "'colors'" );

        colors[view3dColorLayers().at( name )] = COLOR4D( colour );
    }

    if( aArgs.contains( "preset" ) )
    {
        const wxString preset = wxString::FromUTF8( aArgs["preset"].get<std::string>() );

        if( !cfg->FindPreset( preset ) && preset != FOLLOW_PCB && preset != FOLLOW_PLOT_SETTINGS )
            return KOPENAPI_RESULT::Error( 400, "preset: an unknown name; the answer's 'presets' lists them" );

        panel->ApplyLayerPreset( preset );
    }

    // the same paths as the viewer's Layers panel and menus (presets, follow-board, reload)
    for( const auto& [name, visible] : set.items() )
        panel->OnLayerVisibilityChanged( view3dLayers().at( name ), visible.get<bool>() );

    if( aArgs.contains( "stackup_colors" ) )
        panel->SetUseStackupColors( aArgs["stackup_colors"].get<bool>() );

    if( aArgs.contains( "editor_copper_colors" ) )
        panel->SetUseEditorCopperColors( aArgs["editor_copper_colors"].get<bool>() );

    for( const auto& [layer, color] : colors )
    {
        if( !panel->SetLayerColor( layer, color ) )
            return KOPENAPI_RESULT::Error( 409, "colors: that layer has no colour of its own here (or board stackup "
                                                "colours are in use: stackup_colors false)" );
    }

    if( !material.empty() )
        frame->GetToolManager()->RunAction( *materials.at( material ).second );

    const std::bitset<LAYER_3D_END> visibleLayers = frame->GetAdapter().GetVisibleLayers();
    nlohmann::json                  layers = nlohmann::json::object();

    for( const auto& [name, id] : view3dLayers() )
        layers[name] = visibleLayers.test( id );

    const std::map<int, COLOR4D> layerColors = frame->GetAdapter().GetLayerColors();
    nlohmann::json               colorsNow = nlohmann::json::object();

    for( const auto& [name, id] : view3dColorLayers() )
    {
        auto it = layerColors.find( id );

        if( it == layerColors.end() )
            continue;

        const wxColour c = it->second.ToColour();
        colorsNow[name] = wxString::Format( "#%02x%02x%02x%02x", c.Red(), c.Green(), c.Blue(), c.Alpha() )
                                  .ToStdString();
    }

    std::string materialNow;

    for( const auto& [name, mode] : materials )
    {
        if( mode.first == cfg->m_Render.material_mode )
            materialNow = name;
    }

    nlohmann::json presets = nlohmann::json::array();

    for( const LAYER_PRESET_3D& preset : cfg->m_LayerPresets )
        presets.push_back( preset.name.ToStdString() );

    const RENDER_3D_BASE* render = frame->GetCanvas()->GetCurrentRender();

    return KOPENAPI_RESULT::Ok( { { "layers", layers },
                                  { "colors", colorsNow },
                                  { "material", materialNow },
                                  { "stackup_colors", cfg->m_UseStackupColors },
                                  { "editor_copper_colors", cfg->m_Render.use_board_editor_copper_colors },
                                  { "preset", cfg->m_CurrentPreset.ToStdString() },
                                  { "presets", presets },
                                  { "rebuild", frame->GetCanvas()->IsReloadRequestPending()
                                                       || ( render && render->IsSceneLoading() ) } } );
}


/// @brief How a 3D viewer option takes effect: as the viewer's menus and Preferences apply it
enum class VIEW3D_APPLY
{
    REDRAW,     ///< read on every paint
    SCENE,      ///< read while the scene is built: rebuild
    RAYTRACE,   ///< raytracing settings: a new picture when raytracing is shown
    CAMERA,     ///< canvas / camera settings
    ENGINE,     ///< OpenGL or raytracing
    REOPEN      ///< fixed when the GL canvas is made: the next time the viewer opens
};


/// @brief One option of view3d_options: where it sits in the answer and how it is read / set
struct VIEW3D_OPTION
{
    std::string  path;     ///< json pointer, e.g. /raytracing/shadows
    VIEW3D_APPLY apply;
    std::function<nlohmann::json( EDA_3D_VIEWER_SETTINGS& )>                     get;
    /// error text or ""; the value stored only when aStore
    std::function<std::string( EDA_3D_VIEWER_SETTINGS&, const nlohmann::json&, bool aStore )> set;
};


static std::string colorText( const COLOR4D& aColor )
{
    const wxColour c = aColor.ToColour();
    return wxString::Format( "#%02x%02x%02x%02x", c.Red(), c.Green(), c.Blue(), c.Alpha() ).ToStdString();
}


static const std::vector<VIEW3D_OPTION>& view3dOptions()
{
    using CFG = EDA_3D_VIEWER_SETTINGS;

    static const std::vector<VIEW3D_OPTION> options = []()
    {
        std::vector<VIEW3D_OPTION> o;

        auto flag = [&]( const char* aPath, VIEW3D_APPLY aApply, std::function<bool&( CFG& )> aField )
        {
            o.push_back( { aPath, aApply, [=]( CFG& c ) { return nlohmann::json( aField( c ) ); },
                           [=]( CFG& c, const nlohmann::json& v, bool aStore ) -> std::string
                           {
                               if( !v.is_boolean() )
                                   return "true or false";

                               if( aStore )
                                   aField( c ) = v.get<bool>();
                               return "";
                           } } );
        };

        auto number = [&]( const char* aPath, VIEW3D_APPLY aApply, std::function<double( CFG& )> aGet,
                           std::function<void( CFG&, double )> aSet, double aMin, double aMax, bool aInteger )
        {
            o.push_back( { aPath, aApply,
                           [=]( CFG& c )
                           {
                               return aInteger ? nlohmann::json( std::lround( aGet( c ) ) ) : nlohmann::json( aGet( c ) );
                           },
                           [=]( CFG& c, const nlohmann::json& v, bool aStore ) -> std::string
                           {
                               if( !v.is_number() || v.get<double>() < aMin || v.get<double>() > aMax
                                   || ( aInteger && v.get<double>() != std::floor( v.get<double>() ) ) )
                               {
                                   return wxString::Format( "%s from %g to %g", aInteger ? "a whole number" : "a number",
                                                            aMin, aMax ).ToStdString();
                               }

                               if( aStore )
                                   aSet( c, v.get<double>() );
                               return "";
                           } } );
        };

        auto integer = [&]( const char* aPath, VIEW3D_APPLY aApply, std::function<int&( CFG& )> aField, int aMin,
                            int aMax )
        {
            number( aPath, aApply, [=]( CFG& c ) { return double( aField( c ) ); },
                    [=]( CFG& c, double v ) { aField( c ) = int( v ); }, aMin, aMax, true );
        };

        auto color = [&]( const std::string& aPath, VIEW3D_APPLY aApply, std::function<COLOR4D&( CFG& )> aField )
        {
            o.push_back( { aPath, aApply, [=]( CFG& c ) { return nlohmann::json( colorText( aField( c ) ) ); },
                           [=]( CFG& c, const nlohmann::json& v, bool aStore ) -> std::string
                           {
                               wxColour colour( wxString::FromUTF8( v.is_string() ? v.get<std::string>() : "" ) );

                               if( !colour.IsOk() )
                                   return "'#rrggbb', '#rrggbbaa' or a colour name";

                               if( aStore )
                                   aField( c ) = COLOR4D( colour );
                               return "";
                           } } );
        };

        auto choice = [&]( const char* aPath, VIEW3D_APPLY aApply, std::function<int( CFG& )> aGet,
                           std::function<void( CFG&, int )> aSet, std::vector<const char*> aNames )
        {
            o.push_back( { aPath, aApply,
                           [=]( CFG& c )
                           {
                               const int i = aGet( c );
                               return nlohmann::json( i >= 0 && i < int( aNames.size() ) ? aNames[i] : "" );
                           },
                           [=]( CFG& c, const nlohmann::json& v, bool aStore ) -> std::string
                           {
                               std::string all;

                               for( size_t i = 0; i < aNames.size(); ++i )
                               {
                                   if( v.is_string() && v.get<std::string>() == aNames[i] )
                                   {
                                       if( aStore )
                                           aSet( c, int( i ) );

                                       return "";
                                   }

                                   all += ( i ? " | " : "" ) + std::string( aNames[i] );
                               }

                               return all;
                           } } );
        };

        using A = VIEW3D_APPLY;

        choice( "/engine", A::ENGINE, []( CFG& c ) { return int( c.m_Render.engine ); },
                []( CFG& c, int v ) { c.m_Render.engine = RENDER_ENGINE( v ); }, { "opengl", "raytracing" } );
        choice( "/grid", A::REDRAW, []( CFG& c ) { return int( c.m_Render.grid_type ); },
                []( CFG& c, int v ) { c.m_Render.grid_type = GRID3D_TYPE( v ); },
                { "none", "1mm", "2.5mm", "5mm", "10mm" } );
        choice( "/antialiasing", A::REOPEN, []( CFG& c ) { return int( c.m_Render.opengl_AA_mode ); },
                []( CFG& c, int v ) { c.m_Render.opengl_AA_mode = ANTIALIASING_MODE( v ); },
                { "off", "2x", "4x", "8x" } );
        flag( "/show_missing_models", A::SCENE, []( CFG& c ) -> bool& { return c.m_Render.show_missing_models; } );
        flag( "/clip_tht_leads", A::SCENE, []( CFG& c ) -> bool& { return c.m_Render.clip_tht_leads; } );
        flag( "/model_instancing", A::REDRAW, []( CFG& c ) -> bool& { return c.m_Render.opengl_model_instancing; } );
        flag( "/highlight_on_rollover", A::REDRAW,
              []( CFG& c ) -> bool& { return c.m_Render.highlight_on_rollover; } );
        color( "/selection_color", A::REDRAW, []( CFG& c ) -> COLOR4D& { return c.m_Render.opengl_selection_color; } );
        flag( "/copper_thickness", A::SCENE, []( CFG& c ) -> bool& { return c.m_Render.opengl_copper_thickness; } );
        flag( "/show_zones", A::SCENE, []( CFG& c ) -> bool& { return c.m_Render.show_zones; } );
        flag( "/subtract_mask_from_silk", A::SCENE,
              []( CFG& c ) -> bool& { return c.m_Render.subtract_mask_from_silk; } );
        flag( "/clip_silk_on_via_annuli", A::SCENE,
              []( CFG& c ) -> bool& { return c.m_Render.clip_silk_on_via_annuli; } );
        flag( "/differentiate_plated_copper", A::SCENE,
              []( CFG& c ) -> bool& { return c.m_Render.differentiate_plated_copper; } );
        flag( "/while_moving/no_antialiasing", A::REDRAW,
              []( CFG& c ) -> bool& { return c.m_Render.opengl_AA_disableOnMove; } );
        flag( "/while_moving/no_copper_thickness", A::REDRAW,
              []( CFG& c ) -> bool& { return c.m_Render.opengl_thickness_disableOnMove; } );
        flag( "/while_moving/no_vias", A::REDRAW,
              []( CFG& c ) -> bool& { return c.m_Render.opengl_microvias_disableOnMove; } );
        flag( "/while_moving/no_holes", A::REDRAW,
              []( CFG& c ) -> bool& { return c.m_Render.opengl_holes_disableOnMove; } );
        flag( "/while_moving/boxes_only", A::REDRAW,
              []( CFG& c ) -> bool& { return c.m_Render.opengl_render_bbox_only_OnMove; } );
        flag( "/camera/animation", A::CAMERA, []( CFG& c ) -> bool& { return c.m_Camera.animation_enabled; } );
        integer( "/camera/animation_speed", A::CAMERA,
                 []( CFG& c ) -> int& { return c.m_Camera.moving_speed_multiplier; }, 1, 5 );
        number( "/camera/rotation_increment_deg", A::CAMERA, []( CFG& c ) { return c.m_Camera.rotation_increment; },
                []( CFG& c, double v ) { c.m_Camera.rotation_increment = v; }, 0, 359, false );
        choice( "/camera/projection", A::CAMERA, []( CFG& c ) { return c.m_Camera.projection_mode; },
                []( CFG& c, int v ) { c.m_Camera.projection_mode = v; }, { "orthographic", "perspective" } );

        flag( "/raytracing/shadows", A::RAYTRACE, []( CFG& c ) -> bool& { return c.m_Render.raytrace_shadows; } );
        flag( "/raytracing/back_floor", A::RAYTRACE, []( CFG& c ) -> bool& { return c.m_Render.raytrace_backfloor; } );
        flag( "/raytracing/refractions", A::RAYTRACE,
              []( CFG& c ) -> bool& { return c.m_Render.raytrace_refractions; } );
        flag( "/raytracing/reflections", A::RAYTRACE,
              []( CFG& c ) -> bool& { return c.m_Render.raytrace_reflections; } );
        flag( "/raytracing/post_processing", A::RAYTRACE,
              []( CFG& c ) -> bool& { return c.m_Render.raytrace_post_processing; } );
        flag( "/raytracing/anti_aliasing", A::RAYTRACE,
              []( CFG& c ) -> bool& { return c.m_Render.raytrace_anti_aliasing; } );
        flag( "/raytracing/procedural_textures", A::RAYTRACE,
              []( CFG& c ) -> bool& { return c.m_Render.raytrace_procedural_textures; } );
        integer( "/raytracing/samples/shadows", A::RAYTRACE,
                 []( CFG& c ) -> int& { return c.m_Render.raytrace_nrsamples_shadows; }, 1, 64 );
        integer( "/raytracing/samples/reflections", A::RAYTRACE,
                 []( CFG& c ) -> int& { return c.m_Render.raytrace_nrsamples_reflections; }, 1, 32 );
        integer( "/raytracing/samples/refractions", A::RAYTRACE,
                 []( CFG& c ) -> int& { return c.m_Render.raytrace_nrsamples_refractions; }, 1, 5 );
        number( "/raytracing/spread/shadows", A::RAYTRACE, []( CFG& c ) { return c.m_Render.raytrace_spread_shadows; },
                []( CFG& c, double v ) { c.m_Render.raytrace_spread_shadows = float( v ); }, 0, 1, false );
        number( "/raytracing/spread/reflections", A::RAYTRACE,
                []( CFG& c ) { return c.m_Render.raytrace_spread_reflections; },
                []( CFG& c, double v ) { c.m_Render.raytrace_spread_reflections = float( v ); }, 0, 1, false );
        number( "/raytracing/spread/refractions", A::RAYTRACE,
                []( CFG& c ) { return c.m_Render.raytrace_spread_refractions; },
                []( CFG& c, double v ) { c.m_Render.raytrace_spread_refractions = float( v ); }, 0, 1, false );
        integer( "/raytracing/recursion/reflections", A::RAYTRACE,
                 []( CFG& c ) -> int& { return c.m_Render.raytrace_recursivelevel_reflections; }, 1, 5 );
        integer( "/raytracing/recursion/refractions", A::RAYTRACE,
                 []( CFG& c ) -> int& { return c.m_Render.raytrace_recursivelevel_refractions; }, 1, 7 );
        color( "/raytracing/light_camera", A::RAYTRACE,
               []( CFG& c ) -> COLOR4D& { return c.m_Render.raytrace_lightColorCamera; } );
        color( "/raytracing/light_top", A::RAYTRACE, []( CFG& c ) -> COLOR4D& { return c.m_Render.raytrace_lightColorTop; } );
        color( "/raytracing/light_bottom", A::RAYTRACE,
               []( CFG& c ) -> COLOR4D& { return c.m_Render.raytrace_lightColorBottom; } );

        for( size_t i = 0; i < 8; ++i )
        {
            const std::string light = "/raytracing/lights/" + std::to_string( i );

            color( light + "/color", A::RAYTRACE, [i]( CFG& c ) -> COLOR4D& { return c.m_Render.raytrace_lightColor[i]; } );
            integer( ( light + "/elevation_deg" ).c_str(), A::RAYTRACE,
                     [i]( CFG& c ) -> int& { return c.m_Render.raytrace_lightElevation[i]; }, -90, 90 );
            integer( ( light + "/azimuth_deg" ).c_str(), A::RAYTRACE,
                     [i]( CFG& c ) -> int& { return c.m_Render.raytrace_lightAzimuth[i]; }, 0, 359 );
        }

        return o;
    }();

    return options;
}


/// @brief Sets 3D viewer options (the view3d_options shape) as the viewer's Preferences and menus do,
/// checked first, all or nothing; aChanged gets the kinds of change made
/// @return the error, empty when set
static std::string setView3dOptions( EDA_3D_VIEWER_FRAME* aFrame, EDA_3D_VIEWER_SETTINGS& aCfg,
                                     const nlohmann::json& aSet, std::set<VIEW3D_APPLY>& aChanged,
                                     bool aApply = true )
{
    std::map<std::string, const VIEW3D_OPTION*> byPath;

    for( const VIEW3D_OPTION& option : view3dOptions() )
        byPath[option.path] = &option;

    // checked first, all or nothing
    const nlohmann::json flat = aSet.empty() ? nlohmann::json::object() : aSet.flatten();

    for( const auto& [path, value] : flat.items() )
    {
        auto it = byPath.find( path );

        if( it == byPath.end() )
            return "set: no option " + path + " (the answer lists them)";

        if( std::string error = it->second->set( aCfg, value, false ); !error.empty() )
            return "set: " + path + ": " + error;
    }

    if( !aApply )
        return "";

    for( const auto& [path, value] : flat.items() )
    {
        const VIEW3D_OPTION* option = byPath.at( path );
        const nlohmann::json before = option->get( aCfg );

        option->set( aCfg, value, true );

        if( option->get( aCfg ) != before )
            aChanged.insert( option->apply );
    }

    if( aFrame )
    {
        EDA_3D_CANVAS* canvas = aFrame->GetCanvas();

        if( aChanged.count( VIEW3D_APPLY::CAMERA ) )
        {
            canvas->SetAnimationEnabled( aCfg.m_Camera.animation_enabled );
            canvas->SetMovingSpeedMultiplier( aCfg.m_Camera.moving_speed_multiplier );

            if( canvas->GetProjectionMode() != aCfg.m_Camera.projection_mode )
                aFrame->GetCurrentCamera().ToggleProjection();

            if( EDA_3D_CONTROLLER* ctrl = aFrame->GetToolManager()->GetTool<EDA_3D_CONTROLLER>() )
                ctrl->SetRotationIncrement( aCfg.m_Camera.rotation_increment );
        }

        // the same calls as the viewer's menu items
        if( aChanged.count( VIEW3D_APPLY::ENGINE ) )
            canvas->RenderEngineChanged();
        else if( aChanged.count( VIEW3D_APPLY::SCENE )
                 || ( aChanged.count( VIEW3D_APPLY::RAYTRACE ) && aCfg.m_Render.engine == RENDER_ENGINE::RAYTRACING ) )
            aFrame->NewDisplay( true );
        else if( aChanged.count( VIEW3D_APPLY::REDRAW ) || aChanged.count( VIEW3D_APPLY::CAMERA ) )
        {
            if( aCfg.m_Render.engine == RENDER_ENGINE::OPENGL )
                canvas->Request_refresh();
            else
                canvas->RenderRaytracingRequest();
        }
    }

    return "";
}


static nlohmann::json view3dOptionsJson( EDA_3D_VIEWER_SETTINGS& aCfg )
{
    nlohmann::json answer = nlohmann::json::object();

    for( const VIEW3D_OPTION& option : view3dOptions() )
        answer[nlohmann::json::json_pointer( option.path )] = option.get( aCfg );

    return answer;
}


static KOPENAPI_RESULT h_view3d_options( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME*    frame = viewer3d( aCtx, aArgs.value( "open", false ) );
    EDA_3D_VIEWER_SETTINGS* cfg = frame ? frame->GetAdapter().m_Cfg
                                        : Pgm().GetSettingsManager().GetAppSettings<EDA_3D_VIEWER_SETTINGS>( "3d_viewer" );

    if( !cfg )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer settings" );

    const nlohmann::json set = aArgs.value( "set", nlohmann::json::object() );

    if( !set.is_object() )
        return KOPENAPI_RESULT::Error( 400, "set: options as in the answer, e.g. {highlight_on_rollover: false, "
                                            "raytracing: {shadows: false}}" );

    std::set<VIEW3D_APPLY> changed;

    if( std::string error = setView3dOptions( frame, *cfg, set, changed ); !error.empty() )
        return KOPENAPI_RESULT::Error( 400, error );

    nlohmann::json answer = view3dOptionsJson( *cfg );

    const RENDER_3D_BASE* render = frame ? frame->GetCanvas()->GetCurrentRender() : nullptr;

    answer["viewer_open"] = frame != nullptr;
    answer["rebuild"] = frame && ( frame->GetCanvas()->IsReloadRequestPending() || ( render && render->IsSceneLoading() ) );
    answer["reopen_needed"] = changed.count( VIEW3D_APPLY::REOPEN ) > 0 && frame != nullptr;
    return KOPENAPI_RESULT::Ok( answer );
}


static KOPENAPI_RESULT h_view3d_status( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", false ) );

    if( !frame || !frame->GetCanvas() || !frame->IsShown() )
        return KOPENAPI_RESULT::Ok( { { "open", false }, { "ready", false } } );

    EDA_3D_CANVAS*                         canvas = frame->GetCanvas();
    const EDA_3D_CANVAS::RENDER_PROGRESS&  progress = canvas->GetRenderProgress();
    const RENDER_3D_BASE*                  render = canvas->GetCurrentRender();
    const bool reloadPending = canvas->IsReloadRequestPending();
    const bool loading = render && render->IsSceneLoading();
    const bool raytracing = canvas->GetBoardAdapter().m_Cfg->m_Render.engine == RENDER_ENGINE::RAYTRACING;

    return KOPENAPI_RESULT::Ok( { { "open", true },
                                  { "engine", raytracing ? "raytracing" : "opengl" },
                                  { "reload_pending", reloadPending },
                                  { "scene_loading", loading },
                                  { "more_passes", progress.moreRequested },
                                  { "frames", progress.frames },
                                  { "last_frame_ms", progress.lastFrameMs },
                                  { "ready", progress.frames > 0 && !reloadPending && !loading
                                                     && !progress.moreRequested },
                                  { "hit_test_ready", !raytracing && !reloadPending && canvas->IsHitTestReady() },
                                  { "model_instancing", render && render->IsModelInstancing() } } );
}


/// @brief The 3D viewer's world -> board mm (inverse of boardToWorld)
static std::optional<SFVEC3F> worldToBoard( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard, const SFVEC3F& aWorld )
{
    const BOX2I outline = aBoard ? aBoard->GetBoardEdgesBoundingBox() : BOX2I();

    if( outline.GetWidth() <= 0 || outline.GetHeight() <= 0 )
        return std::nullopt;

    const BOARD_ADAPTER& adapter = aFrame->GetAdapter();
    const SFVEC3F        bmin = adapter.GetBBox().Min();
    const SFVEC3F        bmax = adapter.GetBBox().Max();
    const double         sx = ( bmax.x - bmin.x ) / ( outline.GetWidth() / pcbIUScale.IU_PER_MM );
    const double         sy = ( bmax.y - bmin.y ) / ( outline.GetHeight() / pcbIUScale.IU_PER_MM );

    return SFVEC3F( (float) ( outline.GetLeft() / pcbIUScale.IU_PER_MM + ( aWorld.x - bmin.x ) / sx ),
                    (float) ( outline.GetTop() / pcbIUScale.IU_PER_MM + ( bmax.y - aWorld.y ) / sy ),
                    (float) ( ( aWorld.z - bmax.z ) / sx ) );
}


static KOPENAPI_RESULT h_view3d_pick( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const bool byBoardPoint = aArgs.contains( "at_mm" );

    if( byBoardPoint )
    {
        const nlohmann::json& at = aArgs["at_mm"];

        if( !at.is_array() || at.size() < 2 || !at[0].is_number() || !at[1].is_number() )
            return KOPENAPI_RESULT::Error( 400, "at_mm must be [x_mm, y_mm] or [x_mm, y_mm, z_mm]" );
    }
    else if( !( aArgs.contains( "x" ) && aArgs.contains( "y" ) && aArgs["x"].is_number() && aArgs["y"].is_number() ) )
    {
        return KOPENAPI_RESULT::Error( 400, "give x and y (pixels of a capture, from the top left) or at_mm" );
    }

    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, false );

    if( !frame || !frame->GetCanvas() || !frame->IsShown() || !frame->GetAdapter().GetBoard() )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer window open (view3d_status open: true opens it)" );

    EDA_3D_CANVAS* canvas = frame->GetCanvas();
    BOARD*         board = const_cast<BOARD*>( frame->GetAdapter().GetBoard() );

    if( frame->GetAdapter().m_Cfg->m_Render.engine != RENDER_ENGINE::OPENGL )
        return KOPENAPI_RESULT::Error( 409, "picking needs the OpenGL engine (raytracing has no hit-test scene)" );

    if( canvas->IsReloadRequestPending() || !canvas->IsHitTestReady() )
        return KOPENAPI_RESULT::Error( 409, "hit-test scene being built: wait for view3d_status hit_test_ready" );

    const wxSize   size = canvas->GetNativePixelSize();
    const CAMERA&  camera = frame->GetCurrentCamera();
    SFVEC2F        pixel;

    if( byBoardPoint )
    {
        const nlohmann::json& at = aArgs["at_mm"];
        std::optional<SFVEC3F> world = boardToWorld( frame, board, at[0].get<double>(), at[1].get<double>(),
                                                     at.size() > 2 && at[2].is_number() ? at[2].get<double>() : 0.0 );

        if( !world )
            return KOPENAPI_RESULT::Error( 409, "the board has no outline" );

        const glm::vec4 clip = camera.GetProjectionMatrix() * camera.GetViewMatrix() * glm::vec4( *world, 1.0f );

        if( clip.w <= 0.0f )
            return KOPENAPI_RESULT::Error( 400, "at_mm is behind the camera" );

        pixel = SFVEC2F( ( clip.x / clip.w * 0.5f + 0.5f ) * size.x, ( 0.5f - clip.y / clip.w * 0.5f ) * size.y );
    }
    else
    {
        pixel = SFVEC2F( aArgs["x"].get<float>(), aArgs["y"].get<float>() );
    }

    nlohmann::json result = { { "pixel", { pixel.x, pixel.y } }, { "canvas", { size.x, size.y } } };

    if( pixel.x < 0 || pixel.y < 0 || pixel.x >= size.x - 1 || pixel.y >= size.y - 1 )
    {
        result["hit"] = false;
        result["outside"] = true;
        return KOPENAPI_RESULT::Ok( result );
    }

    SFVEC3F     hitWorld;
    BOARD_ITEM* item = canvas->PickBoardItem( pixel, &hitWorld );

    result["hit"] = item != nullptr;

    if( !item )
        return KOPENAPI_RESULT::Ok( result );

    if( std::optional<SFVEC3F> mm = worldToBoard( frame, board, hitWorld ) )
        result["point_mm"] = { mm->x, mm->y, mm->z };

    nlohmann::json desc = { { "type", item->GetClass().ToStdString() },
                            { "uuid", item->m_Uuid.AsStdString() } };

    if( item->GetLayer() != UNDEFINED_LAYER )
        desc["layer"] = board->GetLayerName( item->GetLayer() ).ToStdString();

    if( const FOOTPRINT* fp = item->Type() == PCB_FOOTPRINT_T ? static_cast<const FOOTPRINT*>( item )
                                                               : item->GetParentFootprint() )
    {
        desc["reference"] = fp->GetReference().ToStdString();
    }

    if( item->Type() == PCB_PAD_T )
        desc["pad"] = static_cast<PAD*>( item )->GetNumber().ToStdString();

    if( BOARD_CONNECTED_ITEM* conn = dynamic_cast<BOARD_CONNECTED_ITEM*>( item ); conn && conn->GetNet() )
        desc["net"] = conn->GetNetname().ToStdString();

    result["item"] = desc;
    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_view3d_lead_clips( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", true ) );

    if( !frame || !frame->GetCanvas() || !frame->GetAdapter().GetBoard() )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer window open (open: true opens it from the board editor)" );

    BOARD_ADAPTER& adapter = frame->GetAdapter();
    S3D_CACHE*     cache = adapter.Get3dCacheManager();
    const BOARD*   board = adapter.GetBoard();

    if( !cache )
        return KOPENAPI_RESULT::Error( 409, "no 3D model cache" );

    const bool all = aArgs.value( "all", false );
    std::set<wxString> refs;

    if( aArgs.contains( "refs" ) )
    {
        for( const auto& r : aArgs["refs"] )
            refs.insert( wxString::FromUTF8( r.get<std::string>() ) );
    }

    std::map<const S3DMODEL*, std::vector<SFVEC3F>> points;
    nlohmann::json                                 out = nlohmann::json::array();
    std::map<std::string, int>                     counts;

    for( const FOOTPRINT* fp : board->Footprints() )
    {
        if( !refs.empty() && !refs.contains( fp->GetReference() ) )
            continue;

        wxString basePath;

        if( board->GetProject() )
        {
            try
            {
                if( std::optional<LIBRARY_TABLE_ROW*> row =
                            PROJECT_PCB::FootprintLibAdapter( board->GetProject() )->GetRow( fp->GetFPID().GetLibNickname() ) )
                {
                    basePath = LIBRARY_MANAGER::GetFullURI( *row, true );
                }
            }
            catch( ... )
            {
            }
        }

        const glm::mat4 fpMatrix = adapter.GetFootprintMatrix( *fp );

        for( const FP_3DMODEL& model : fp->Models() )
        {
            if( !model.m_Show || model.m_Filename.empty() )
                continue;

            std::vector<const EMBEDDED_FILES*> embedded{ fp->GetEmbeddedFiles(), board->GetEmbeddedFiles() };
            const wxString  path = cache->GetResolver()->ResolvePath( model.m_Filename, basePath, embedded );
            const S3DMODEL* s3d = cache->GetModel( model.m_Filename, basePath, std::move( embedded ) );

            if( !s3d )
                continue;

            auto pts = points.find( s3d );

            if( pts == points.end() )
                pts = points.emplace( s3d, ModelSamplePoints( *s3d ) ).first;

            const glm::mat4 world = fpMatrix
                                    * CalcModelMatrix( SFVEC3F( model.m_Offset.x, model.m_Offset.y, model.m_Offset.z ),
                                                       SFVEC3F( model.m_Rotation.x, model.m_Rotation.y, model.m_Rotation.z ),
                                                       SFVEC3F( model.m_Scale.x, model.m_Scale.y, model.m_Scale.z ) );
            const MODEL_MOUNT_META meta = ReadModelMountMeta( path );
            const LEAD_CLIP_RESULT r = ComputeLeadClip( adapter, *fp, world, pts->second, meta );

            static const char* const names[] = { "clipped", "within_limits", "below_board_by_design",
                                                 "body_below_board", "pegs_below_board", "no_holes" };
            const std::string status = names[(int) r.m_status];
            counts[status]++;

            if( !all && ( r.m_status == LEAD_CLIP_STATUS::NO_HOLES || r.m_status == LEAD_CLIP_STATUS::WITHIN_LIMITS ) )
                continue;

            nlohmann::json item = { { "ref", fp->GetReference().ToStdString() },
                                    { "model", wxFileName( model.m_Filename ).GetFullName().ToStdString() },
                                    { "status", status },
                                    { "depth_mm", std::round( r.m_depthMm * 100.0 ) / 100.0 },
                                    { "peg_depth_mm", std::round( r.m_pegDepthMm * 100.0 ) / 100.0 },
                                    { "cut_to_mm", r.m_targetMm },
                                    { "meta", meta.found } };

            if( meta.found )
                item["seat"] = meta.seat;

            out.push_back( item );
        }
    }

    return KOPENAPI_RESULT::Ok( { { "enabled", adapter.m_Cfg->m_Render.clip_tht_leads },
                                  { "counts", counts },
                                  { "models", out } } );
}


static KOPENAPI_RESULT h_view3d_camera( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", true ) );

    if( !frame )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer (open: true opens it from the board editor)" );

    KOPENAPI_REMOTE_GUARD remote( frame );   // manual camera off, title says so, for this call

    EDA_3D_CANVAS* canvas = frame->GetCanvas();

    if( aArgs.contains( "preset" ) )
    {
        static const std::map<std::string, VIEW3D_TYPE> presets = {
            { "top", VIEW3D_TYPE::VIEW3D_TOP },     { "bottom", VIEW3D_TYPE::VIEW3D_BOTTOM },
            { "left", VIEW3D_TYPE::VIEW3D_LEFT },   { "right", VIEW3D_TYPE::VIEW3D_RIGHT },
            { "front", VIEW3D_TYPE::VIEW3D_FRONT }, { "back", VIEW3D_TYPE::VIEW3D_BACK },
            { "fit", VIEW3D_TYPE::VIEW3D_FIT_SCREEN } };

        auto it = presets.find( aArgs["preset"].get<std::string>() );

        if( it == presets.end() )
            return KOPENAPI_RESULT::Error( 400, "preset: top, bottom, left, right, front, back or fit" );

        // the user's animation setting is theirs: set for this call only
        const bool userAnimation = canvas->GetAnimationEnabled();
        canvas->SetAnimationEnabled( aArgs.value( "animate", true ) );
        canvas->SetView3D( it->second );

        // the preset animates through the canvas' own timer: let it run
        const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds( aArgs.value( "animate", true ) ? 800 : 50 );

        while( std::chrono::steady_clock::now() < until )
        {
            KopenapiKeepUiAlive();
            std::this_thread::sleep_for( std::chrono::milliseconds( 8 ) );
        }

        canvas->SetAnimationEnabled( userAnimation );
    }

    CAMERA& camera = frame->GetCurrentCamera();

    if( aArgs.contains( "rotate_deg" ) )
    {
        const nlohmann::json& r = aArgs["rotate_deg"];

        if( !r.is_array() || r.size() != 3 )
            return KOPENAPI_RESULT::Error( 400, "rotate_deg: [x, y, z]" );

        camera.RotateX( glm::radians( r[0].get<float>() ) );
        camera.RotateY( glm::radians( r[1].get<float>() ) );
        camera.RotateZ( glm::radians( r[2].get<float>() ) );
    }

    wxWeakRef<EDA_3D_VIEWER_FRAME> alive( frame );

    if( aArgs.contains( "look_at" ) )
    {
        std::string                  error;
        const std::optional<SFVEC3F> at = lookAtOf( frame, boardOf( aCtx ), aArgs["look_at"], error );

        if( !at )
            return KOPENAPI_RESULT::Error( error.find( "none" ) != std::string::npos ? 404 : 400, error );

        std::vector<CAMERA_MOVE> pan( 1 );
        pan[0].lookAt = at;
        resolveLookAt( camera, pan );
        double done = 0;

        KopenapiAnimate( aArgs.value( "animate", true ) ? 0.6 : 0.0,
                         [&]( double e )
                         {
                             if( !alive )
                                 return;

                             stepMove( camera, pan[0], done, e );
                             done = e;
                             frame->GetCanvas()->DoRePaint();
                         } );

        if( !alive )
            return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the animation" );

        if( done < 1.0 )
            stepMove( camera, pan[0], done, 1.0 );
    }

    if( aArgs.contains( "zoom" ) )
        camera.Zoom( aArgs["zoom"].get<float>() );

    // frame the whole assembly in this pose (unless a zoom was asked for)
    nlohmann::json fit;

    if( aArgs.value( "fit_assembly", true ) && !aArgs.contains( "zoom" ) )
        fit = fitTrajectory( frame, boardOf( aCtx ), {}, aArgs.value( "animate", true ) ? 0.5 : 0.0 );

    if( !alive )
        return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the animation" );

    paint3d( frame );
    return KOPENAPI_RESULT::Ok( { { "camera", "set" }, { "fit", fit }, { "zoom", camera.GetZoom() } } );
}




/// @brief Applies part of a move: the rotation (x, then y, then z) and zoom for a progress step
static void stepMove( CAMERA& aCamera, const CAMERA_MOVE& aMove, double aFrom, double aTo )
{
    const double de = aTo - aFrom;
    aCamera.RotateX( (float) glm::radians( aMove.rotate[0] * de ) );
    aCamera.RotateY( (float) glm::radians( aMove.rotate[1] * de ) );
    aCamera.RotateZ( (float) glm::radians( aMove.rotate[2] * de ) );

    if( aMove.zoom != 1.0 )
        aCamera.Zoom( (float) std::pow( aMove.zoom, de ) );

    shiftCamera( aCamera, aMove.shiftLookAt * (float) de, aMove.shiftPos * (float) de );
}


/// @brief Plays camera moves in order, each eased in and out, repainting as it goes (on video
/// time while a recording runs)
/// @return frames drawn, -1 when the viewer was closed meanwhile (the UI runs between the steps)
/// @brief Paint time per frame (ms): median, p95, max
static nlohmann::json frameStats( std::vector<double> aMs )
{
    if( aMs.empty() )
        return nullptr;

    std::sort( aMs.begin(), aMs.end() );
    const auto at = [&]( double q ) { return aMs[std::min( aMs.size() - 1, size_t( q * ( aMs.size() - 1 ) + 0.5 ) )]; };

    return { { "median", at( 0.5 ) }, { "p95", at( 0.95 ) }, { "max", aMs.back() } };
}


static int playMoves( EDA_3D_VIEWER_FRAME* aFrame, const std::vector<CAMERA_MOVE>& aMoves,
                      const FRAMING* aFollow = nullptr, std::vector<double>* aFrameMs = nullptr,
                      bool* aStopped = nullptr )
{
    wxWeakRef<EDA_3D_VIEWER_FRAME> alive( aFrame );
    CAMERA& camera = aFrame->GetCurrentCamera();
    int     frames = 0;
    double  start = 0;                                            // time into the moves at this move
    double  applied = aFollow ? framingAt( *aFollow, 0 ) : 1.0;   // correction in the camera now

    for( const CAMERA_MOVE& move : aMoves )
    {
        double done = 0;

        if( !alive )
            return -1;

        const bool finished = KopenapiAnimate( move.seconds,
                                               [&]( double e )
                                               {
                                                   if( !alive )
                                                       return;

                                                   stepMove( camera, move, done, e );
                                                   done = e;

                                                   if( aFollow )
                                                   {
                                                       const double k = framingAt( *aFollow, start + e * move.seconds );
                                                       camera.Zoom( (float) ( applied / k ) );
                                                       applied = k;
                                                   }

                                                   const auto paintStart = std::chrono::steady_clock::now();

                                                   aFrame->GetCanvas()->DoRePaint();

                                                   if( aFrameMs )
                                                   {
                                                       const std::chrono::duration<double, std::milli> paint =
                                                               std::chrono::steady_clock::now() - paintStart;
                                                       aFrameMs->push_back( paint.count() );
                                                   }

                                                   frames++;
                                               } );

        if( !finished )
        {
            if( aStopped )
                *aStopped = true;

            break;
        }

        start += move.seconds;
    }

    return alive ? frames : -1;
}


/// @brief The assembly's box corners (board + every shown model) in the 3D viewer's world units
static std::vector<glm::vec4> assemblyCorners( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard )
{
    double lo[3], hi[3];

    if( !aBoard || !KopenapiAssemblyBox( aBoard, lo, hi ) )
        return {};

    std::vector<glm::vec4> corners;

    for( int c = 0; c < 8; ++c )
    {
        const std::optional<SFVEC3F> w = boardToWorld( aFrame, aBoard, ( c & 1 ) ? hi[0] : lo[0],
                                                       ( c & 2 ) ? hi[1] : lo[1], ( c & 4 ) ? hi[2] : lo[2] );

        if( !w )
            return {};

        corners.emplace_back( *w, 1.0f );
    }

    return corners;
}


/// @brief How far the assembly reaches out of the view now (1 = touches the edge), in NDC
static double reach( CAMERA& aCamera, const std::vector<glm::vec4>& aCorners )
{
    const glm::mat4 pv = aCamera.GetProjectionMatrix() * aCamera.GetViewMatrix();
    double          r = 0;

    for( const glm::vec4& p : aCorners )
    {
        const glm::vec4 c = pv * p;

        if( std::abs( c.w ) < 1e-9 )
            continue;

        r = std::max( { r, std::abs( double( c.x / c.w ) ), std::abs( double( c.y / c.w ) ) } );
    }

    return r;
}


/// @brief Frames the whole assembly (board + models over its edges) for the coming moves: the
/// camera runs through the trajectory without drawing, the widest reach of the assembly's box
/// decides the zoom, the camera comes back and zooms there smoothly. Only as much room as the
/// moves need (a half turn needs less than a full tumble).
static nlohmann::json fitTrajectory( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard, const std::vector<CAMERA_MOVE>& aMoves,
                                    double aSeconds, FRAMING* aFollow )
{
    const std::vector<glm::vec4> corners = assemblyCorners( aFrame, aBoard );

    if( corners.empty() )
        return { { "skipped", "no assembly box" } };

    CAMERA&     camera = aFrame->GetCurrentCamera();
    const float zoom0 = camera.GetZoom();

    // a recording renders this view at its own size: frame for that picture
    int          recW = 0, recH = 0;
    const wxSize windowSize = aFrame->GetCanvas()->GetNativePixelSize();
    const bool   recorded = KopenapiRecordingRenderSize( aFrame->GetCanvas(), &recW, &recH );

    if( recorded )
        camera.SetCurWindowSize( wxSize( recW, recH ) );

    double worst = reach( camera, corners );

    // dry run (no painting), then back along the exact inverse steps; follow framing keeps the
    // reach at every moment: (time, zoom factor that brings it to the margin)
    const double                                margin = 0.93;
    std::vector<std::pair<CAMERA_MOVE, double>> done;
    FRAMING                                     need = { { 0.0, reach( camera, corners ) / margin } };
    double                                      t = 0;

    for( const CAMERA_MOVE& move : aMoves )
    {
        const int samples = std::max( 24, int( move.seconds * 30 ) );

        for( int i = 1; i <= samples; ++i )
        {
            stepMove( camera, move, double( i - 1 ) / samples, double( i ) / samples );
            done.emplace_back( move, 1.0 / samples );
            worst = std::max( worst, reach( camera, corners ) * zoom0 / camera.GetZoom() );   // at the start zoom
            need.emplace_back( t + move.seconds * i / samples, reach( camera, corners ) / margin );
        }

        t += move.seconds;
    }

    for( auto it = done.rbegin(); it != done.rend(); ++it )
    {
        const CAMERA_MOVE& m = it->first;
        const double       de = it->second;

        shiftCamera( camera, m.shiftLookAt * (float) -de, m.shiftPos * (float) -de );

        if( m.zoom != 1.0 )
            camera.Zoom( (float) std::pow( m.zoom, -de ) );

        camera.RotateZ( (float) glm::radians( -m.rotate[2] * de ) );
        camera.RotateY( (float) glm::radians( -m.rotate[1] * de ) );
        camera.RotateX( (float) glm::radians( -m.rotate[0] * de ) );
    }

    camera.Zoom( camera.GetZoom() / zoom0 );   // exact start zoom
    const double reachNow = reach( camera, corners );

    if( recorded )
        camera.SetCurWindowSize( windowSize );

    // follow: the envelope (max within +-w) averaged over +-w never falls below the need, and
    // changes smoothly
    if( aFollow )
    {
        const double w = 0.7;
        FRAMING      envelope;

        for( const auto& [ti, ki] : need )
        {
            double m = ki;

            for( const auto& [tj, kj] : need )
            {
                if( std::abs( tj - ti ) <= w )
                    m = std::max( m, kj );
            }

            envelope.emplace_back( ti, m );
        }

        aFollow->clear();

        for( const auto& [ti, ki] : envelope )
        {
            double sum = 0;
            int    n = 0;

            for( const auto& [tj, kj] : envelope )
            {
                if( std::abs( tj - ti ) <= w )
                {
                    sum += kj;
                    n++;
                }
            }

            aFollow->emplace_back( ti, n ? sum / n : ki );
        }
    }

    // a larger zoom value moves the camera away; the view shrinks with it
    const double target = aFollow ? zoom0 * framingAt( *aFollow, 0 ) : zoom0 * worst / 0.94;
    nlohmann::json info = { { "zoom_from", zoom0 }, { "zoom_to", target }, { "reach_at_start_zoom", worst },
                            { "reach_now", reachNow }, { "framing", aFollow ? "follow" : "path" },
                            { "framed_for", recorded ? "recording" : "window" } };

    if( std::abs( target - zoom0 ) / zoom0 < 0.01 )
        return info;

    wxWeakRef<EDA_3D_VIEWER_FRAME> alive( aFrame );

    KopenapiAnimate( aSeconds,
                     [&]( double e )
                     {
                         if( !alive )
                             return;

                         const double want = zoom0 + ( target - zoom0 ) * e;
                         camera.Zoom( (float) ( camera.GetZoom() / want ) );
                         aFrame->GetCanvas()->DoRePaint();
                     } );

    if( !alive )
        return info;

    info["zoom_after"] = camera.GetZoom();
    info["reach_after"] = reach( camera, corners );
    return info;
}


/// @brief Built-in camera choreographies for demos
static std::vector<CAMERA_MOVE> effect( const std::string& aName, double aSpeed )
{
    auto m = [aSpeed]( double x, double y, double z, double zoom, double seconds )
    {
        CAMERA_MOVE c;
        c.rotate[0] = x;
        c.rotate[1] = y;
        c.rotate[2] = z;
        c.zoom = zoom;
        c.seconds = seconds / aSpeed;
        return c;
    };

    if( aName == "turntable" )   // a full turn around the board normal
        return { m( 0, 0, 360, 1, 12 ) };

    if( aName == "tumble" )      // tilt, half turn, tilt back, half turn
        return { m( -35, 0, 0, 1, 2.5 ), m( 0, 0, 180, 1, 6 ), m( 35, 0, 0, 1, 2.5 ), m( 0, 0, 180, 1, 6 ) };

    if( aName == "flyover" )     // dive in at a low angle, sweep along, rise back out
        return { m( -50, 0, 0, 1.8, 3.5 ), m( 0, 0, 120, 1, 6 ), m( 50, 0, -120, 1 / 1.8, 4.5 ) };

    if( aName == "showcase" )    // tilt, full orbit, closer look, flip to the bottom and back, settle
        return { m( -40, 0, 0, 1, 2.5 ),   m( 0, 0, 360, 1, 12 ),   m( 0, 0, 0, 1.5, 2 ),
                 m( 0, 180, 0, 1, 4 ),     m( 0, 180, 0, 1, 4 ),    m( 40, 0, 0, 1 / 1.5, 3 ) };

    return {};
}


static KOPENAPI_RESULT h_view3d_animate( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", true ) );

    if( !frame )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer (open: true opens it from the board editor)" );

    KOPENAPI_REMOTE_GUARD remote( frame );   // manual camera off, title says so, for this call

    std::vector<CAMERA_MOVE> moves;

    if( aArgs.contains( "effect" ) )
    {
        moves = effect( aArgs["effect"].get<std::string>(), std::clamp( aArgs.value( "speed", 1.0 ), 0.1, 10.0 ) );

        if( moves.empty() )
            return KOPENAPI_RESULT::Error( 400, "effect: turntable, tumble, flyover or showcase" );
    }

    for( const nlohmann::json& k : aArgs.value( "moves", nlohmann::json::array() ) )
    {
        CAMERA_MOVE move;
        const nlohmann::json r = k.value( "rotate_deg", nlohmann::json::array( { 0, 0, 0 } ) );

        if( !r.is_array() || r.size() != 3 )
            return KOPENAPI_RESULT::Error( 400, "moves[].rotate_deg: [x, y, z]" );

        for( int i = 0; i < 3; ++i )
            move.rotate[i] = r[i].get<double>();

        move.zoom = k.value( "zoom", 1.0 );
        move.seconds = std::clamp( k.value( "seconds", 1.0 ), 0.05, 120.0 );

        if( k.contains( "look_at" ) )
        {
            std::string error;
            move.lookAt = lookAtOf( frame, boardOf( aCtx ), k["look_at"], error );

            if( !move.lookAt )
                return KOPENAPI_RESULT::Error( error.find( "none" ) != std::string::npos ? 404 : 400, "moves[]." + error );
        }

        moves.push_back( move );
    }

    if( moves.empty() )
        return KOPENAPI_RESULT::Error( 400, "give effect or moves" );

    resolveLookAt( frame->GetCurrentCamera(), moves );

    nlohmann::json fit;
    FRAMING        follow;
    const bool     following = aArgs.value( "framing", std::string( "follow" ) ) == "follow";

    wxWeakRef<EDA_3D_VIEWER_FRAME> alive( frame );

    if( aArgs.value( "fit_assembly", true ) )
        fit = fitTrajectory( frame, boardOf( aCtx ), moves, 0.7, following ? &follow : nullptr );

    if( !alive )
        return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the animation" );

    double total = 0;

    for( const CAMERA_MOVE& m : moves )
        total += m.seconds;

    std::vector<double> frameMs;
    bool                stopped = false;
    const int           frames = playMoves( frame, moves, follow.empty() ? nullptr : &follow, &frameMs, &stopped );

    if( frames < 0 )
        return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the animation" );

    nlohmann::json result = { { "moves", moves.size() }, { "seconds", total }, { "frames", frames }, { "frame_ms", frameStats( frameMs ) }, { "fit", fit },
                              { "stopped", stopped } };

    for( auto it = moves.rbegin(); it != moves.rend(); ++it )
    {
        if( !it->lookAt )
            continue;

        // where the last look_at target ends up on screen: [0, 0] is the centre, +-1 the edges
        CAMERA&         camera = frame->GetCurrentCamera();
        const glm::vec4 clip = camera.GetProjectionMatrix() * camera.GetViewMatrix() * glm::vec4( *it->lookAt, 1.0f );
        result["look_at_screen"] = { clip.x / clip.w, clip.y / clip.w };
        break;
    }

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_view3d_orbit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", true ) );

    if( !frame )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer (open: true opens it from the board editor)" );

    KOPENAPI_REMOTE_GUARD remote( frame );   // manual camera off, title says so, for this call

    const std::string axis = aArgs.value( "axis", std::string( "z" ) );
    CAMERA_MOVE       move;
    move.rotate[axis == "x" ? 0 : axis == "y" ? 1 : 2] = aArgs.value( "degrees", 360.0 );
    move.seconds = std::clamp( aArgs.value( "seconds", 8.0 ), 0.5, 120.0 );

    FRAMING follow;

    wxWeakRef<EDA_3D_VIEWER_FRAME> alive( frame );

    if( aArgs.value( "fit_assembly", true ) )
        fitTrajectory( frame, boardOf( aCtx ), { move }, 0.7,
                       aArgs.value( "framing", std::string( "follow" ) ) == "follow" ? &follow : nullptr );

    if( !alive )
        return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the animation" );

    std::vector<double> frameMs;
    bool                stopped = false;
    const int           frames = playMoves( frame, { move }, follow.empty() ? nullptr : &follow, &frameMs, &stopped );

    if( frames < 0 )
        return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the animation" );

    return KOPENAPI_RESULT::Ok( { { "degrees", aArgs.value( "degrees", 360.0 ) }, { "seconds", move.seconds },
                                  { "frames", frames }, { "frame_ms", frameStats( frameMs ) },
                                  { "stopped", stopped } } );
}


/// @brief Draws the 3D viewer until its scene is built and the final picture is shown
/// @return false when the viewer was closed or the call was stopped meanwhile
static bool settle3d( EDA_3D_VIEWER_FRAME* aFrame, double aLimitS = 300 )
{
    wxWeakRef<EDA_3D_VIEWER_FRAME> alive( aFrame );
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>( aLimitS );

    while( alive && std::chrono::steady_clock::now() < deadline )
    {
        EDA_3D_CANVAS*        canvas = aFrame->GetCanvas();
        const RENDER_3D_BASE* render = canvas->GetCurrentRender();

        if( !canvas->IsReloadRequestPending() && !( render && render->IsSceneLoading() )
            && !canvas->GetRenderProgress().moreRequested && canvas->GetRenderProgress().frames > 0 )
        {
            return true;
        }

        canvas->DoRePaint();
        KopenapiKeepUiAlive();

        if( !KopenapiJobStep( "scene", 0, 0 ) )
            return false;

        std::this_thread::sleep_for( std::chrono::milliseconds( 8 ) );
    }

    return (bool) alive;
}


/// @brief median, p95, min, mean of ms values
static nlohmann::json benchStats( std::vector<double> aMs )
{
    if( aMs.empty() )
        return nullptr;

    std::sort( aMs.begin(), aMs.end() );

    double sum = 0;

    for( double v : aMs )
        sum += v;

    auto at = [&]( double aQ ) { return aMs[std::min( aMs.size() - 1, (size_t) std::lround( aQ * ( aMs.size() - 1 ) ) )]; };
    auto r3 = []( double v ) { return std::round( v * 1000 ) / 1000; };

    return { { "median", r3( at( 0.5 ) ) }, { "p95", r3( at( 0.95 ) ) }, { "min", r3( aMs.front() ) },
             { "mean", r3( sum / aMs.size() ) } };
}


static KOPENAPI_RESULT h_view3d_benchmark( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    EDA_3D_VIEWER_FRAME* frame = viewer3d( aCtx, aArgs.value( "open", true ) );

    if( !frame )
        return KOPENAPI_RESULT::Error( 409, "no 3D viewer (open: true opens it from the board editor)" );

    EDA_3D_VIEWER_SETTINGS* cfg = frame->GetAdapter().m_Cfg;

    if( !cfg || cfg->m_Render.engine != RENDER_ENGINE::OPENGL )
        return KOPENAPI_RESULT::Error( 409, "the OpenGL engine draws the benchmark (view3d_options {engine: opengl})" );

    const int    frames = std::clamp( aArgs.value( "frames", 120 ), 10, 5000 );
    const int    warmup = std::clamp( aArgs.value( "warmup", 10 ), 0, 1000 );
    const int    rounds = std::clamp( aArgs.value( "rounds", 3 ), 1, 50 );
    const double degrees = aArgs.value( "degrees", 90.0 );
    const bool   finish = aArgs.value( "finish", false );
    const bool   vsync = aArgs.value( "vsync", false );
    const std::string axis = aArgs.value( "axis", std::string( "z" ) );

    nlohmann::json variants = aArgs.value( "variants", nlohmann::json::array( { nlohmann::json::object() } ) );

    if( !variants.is_array() || variants.empty() )
        return KOPENAPI_RESULT::Error( 400, "variants: a list of view3d_options sets, e.g. [{model_instancing: false}, "
                                            "{model_instancing: true}]" );

    // every variant checked first; the options they touch are put back afterwards
    nlohmann::json       restore = nlohmann::json::object();
    const nlohmann::json before = view3dOptionsJson( *cfg ).flatten();

    for( const nlohmann::json& set : variants )
    {
        std::set<VIEW3D_APPLY> unused;

        if( !set.is_object() )
            return KOPENAPI_RESULT::Error( 400, "variants: each one a view3d_options set (an object)" );

        if( std::string error = setView3dOptions( frame, *cfg, set, unused, false ); !error.empty() )
            return KOPENAPI_RESULT::Error( 400, "variants: " + error );

        const nlohmann::json flat = set.empty() ? nlohmann::json::object() : set.flatten();

        for( const auto& [path, value] : flat.items() )
            restore[path] = before.at( path );
    }

    KOPENAPI_REMOTE_GUARD          remote( frame );   // manual camera off, title says so, for this call
    wxWeakRef<EDA_3D_VIEWER_FRAME> alive( frame );
    CAMERA&                        camera = frame->GetCurrentCamera();

    CAMERA_MOVE sweep;
    sweep.rotate[axis == "x" ? 0 : axis == "y" ? 1 : 2] = degrees;

    struct SAMPLES
    {
        std::vector<double> paint, redraw, layers, collect, opaque, board, transparent, overlay, finish, rest;
        std::vector<double> roundPaint;
        RENDER_3D_FRAME_STATS last;
    };

    std::vector<SAMPLES> samples( variants.size() );
    bool                 stopped = false;
    int                  done = 0;
    const int            total = rounds * (int) variants.size() * frames;

    for( int round = 0; round < rounds && !stopped; ++round )
    {
        for( size_t v = 0; v < variants.size() && !stopped; ++v )
        {
            std::set<VIEW3D_APPLY> changed;
            setView3dOptions( frame, *cfg, variants[v], changed );

            if( !settle3d( frame ) )
            {
                if( !alive )
                    return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the benchmark" );

                stopped = true;
                break;
            }

            RENDER_3D_BASE* render = frame->GetCanvas()->GetCurrentRender();
            render->SetFinishFrames( finish );
            frame->GetCanvas()->SetVsync( vsync );

            for( int w = 0; w < warmup; ++w )
                frame->GetCanvas()->DoRePaint();

            std::vector<double> paints;
            double              swept = 0;

            for( int k = 1; k <= frames; ++k )
            {
                const double e = double( k ) / frames;
                stepMove( camera, sweep, swept, e );
                swept = e;

                const auto start = std::chrono::steady_clock::now();
                frame->GetCanvas()->DoRePaint();
                const std::chrono::duration<double, std::milli> paint = std::chrono::steady_clock::now() - start;

                if( !alive )
                    return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the benchmark" );

                const RENDER_3D_FRAME_STATS st = frame->GetCanvas()->GetCurrentRender()->GetFrameStats();
                SAMPLES&                    out = samples[v];

                out.paint.push_back( paint.count() );
                out.redraw.push_back( st.redraw );
                out.layers.push_back( st.layers );
                out.collect.push_back( st.collect );
                out.opaque.push_back( st.opaqueModels );
                out.board.push_back( st.board );
                out.transparent.push_back( st.transparentModels );
                out.overlay.push_back( st.overlay );
                out.finish.push_back( st.finish );
                out.rest.push_back( std::max( 0.0, paint.count() - st.redraw ) );
                out.last = st;
                paints.push_back( paint.count() );

                const bool go = KopenapiJobStep( "benchmark", ++done, total );

                if( !alive )
                    return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the benchmark" );

                if( !go )
                {
                    stopped = true;
                    break;
                }
            }

            stepMove( camera, sweep, swept, 0.0 );   // back to the start pose
            frame->GetCanvas()->GetCurrentRender()->SetFinishFrames( false );
            frame->GetCanvas()->SetVsync( true );
            samples[v].roundPaint.push_back( benchStats( paints )["median"].get<double>() );
        }
    }

    if( alive )
    {
        std::set<VIEW3D_APPLY> changed;
        setView3dOptions( frame, *cfg, restore.empty() ? restore : restore.unflatten(), changed );
        frame->GetCanvas()->GetCurrentRender()->SetFinishFrames( false );
        settle3d( frame );
        paint3d( frame );
    }

    if( !alive )
        return KOPENAPI_RESULT::Error( 409, "the 3D viewer was closed during the benchmark" );

    const wxSize   size = frame->GetCanvas()->GetClientSize() * frame->GetCanvas()->GetContentScaleFactor();
    nlohmann::json out = nlohmann::json::array();

    for( size_t v = 0; v < variants.size(); ++v )
    {
        const SAMPLES& sm = samples[v];

        if( sm.paint.empty() )
            continue;

        out.push_back( { { "set", variants[v] },
                         { "frames", sm.paint.size() },
                         { "paint_ms", benchStats( sm.paint ) },
                         { "round_paint_median_ms", sm.roundPaint },
                         { "stages_ms", { { "redraw", benchStats( sm.redraw ) },
                                          { "layers", benchStats( sm.layers ) },
                                          { "collect", benchStats( sm.collect ) },
                                          { "opaque_models", benchStats( sm.opaque ) },
                                          { "board", benchStats( sm.board ) },
                                          { "transparent_models", benchStats( sm.transparent ) },
                                          { "overlay", benchStats( sm.overlay ) },
                                          { "finish", benchStats( sm.finish ) },
                                          { "swap_and_rest", benchStats( sm.rest ) } } },
                         { "models", { { "instances", sm.last.modelInstances },
                                       { "groups", sm.last.modelGroups },
                                       { "draw_calls", sm.last.modelDrawCalls },
                                       { "instanced", sm.last.instanced } } } } );
    }

    nlohmann::json answer = { { "variants", out }, { "rounds", rounds }, { "frames_per_round", frames },
                              { "degrees", degrees }, { "axis", axis }, { "finish", finish }, { "vsync", vsync },
                              { "canvas", { size.x, size.y } }, { "stopped", stopped } };

    // each variant against the first: how many times faster (median)
    if( out.size() > 1 )
    {
        auto median = [&]( size_t aV, const char* aStage ) -> double
        {
            const nlohmann::json& st = aStage ? out[aV]["stages_ms"][aStage] : out[aV]["paint_ms"];
            return st.is_null() ? 0.0 : st["median"].get<double>();
        };

        nlohmann::json speedup = nlohmann::json::array();

        for( size_t v = 1; v < out.size(); ++v )
        {
            auto ratio = [&]( const char* aStage )
            {
                const double b = median( v, aStage );
                return b > 0 ? std::round( median( 0, aStage ) / b * 100 ) / 100 : 0.0;
            };

            speedup.push_back( { { "paint", ratio( nullptr ) }, { "redraw", ratio( "redraw" ) },
                                 { "opaque_models", ratio( "opaque_models" ) } } );
        }

        answer["speedup_vs_first"] = speedup;
    }

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_view_zoom",
                   "Move the board editor's view (camera, zoom, pan): fit the board, an area in mm, or "
                   "parts by reference; margin; animate_ms for a smooth camera move (recordings); side front / back / "
                   "flip to look from above or below; GUI only",
                   R"json({"type":"object","properties":{
                        "area_mm":{"type":"array","items":{"type":"number"},"description":"[x0, y0, x1, y1]"},
                        "refs":{"type":"array","items":{"type":"string"}},
                        "margin_mm":{"type":"number","default":2},
                        "animate_ms":{"type":"integer","default":600,"maximum":5000,"description":"smooth camera move; 0 jumps"},
                        "side":{"type":"string","enum":["front","back","flip"],"description":"the side looked at: front (from above), back (from below, Flip Board View; bottom-side work), flip toggles; unchanged when absent"}}})json"_json,
                   true, h_pcb_view_zoom, 300 );   // animations record on video time: slow while recording

KOPENAPI_REGISTER( "view3d_status",
                   "3D viewer progress: open, engine, scene reload pending / loading (models), whether the "
                   "renderer wants more passes (raytracing), frames shown and the last frame's time; "
                   "ready = the final picture is on screen; hit_test_ready = view3d_pick sees the current "
                   "scene and visibility. open: true opens the viewer; GUI only",
                   R"json({"type":"object","properties":{
                        "open":{"type":"boolean","default":false}}})json"_json,
                   true, h_view3d_status );

KOPENAPI_REGISTER( "view3d_pick",
                   "What the 3D viewer shows at a point, as a mouse click picks it (OpenGL hit-test scene: "
                   "copper with pads, tracks, vias, zones, and the models; hidden layers and models are not "
                   "picked): x / y (pixels of a pcb_view_capture of the 3D view, from the top left) or at_mm "
                   "(a board point, projected through the camera). Answer: hit, item (type, reference, pad, "
                   "net, layer, uuid), point_mm (where the ray hits, z above the board's top surface), pixel, "
                   "canvas size. 409 while the hit-test scene is rebuilt (view3d_status hit_test_ready); GUI only",
                   R"json({"type":"object","properties":{
                        "x":{"type":"number"},"y":{"type":"number"},
                        "at_mm":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":3}}})json"_json,
                   true, h_view3d_pick );

KOPENAPI_REGISTER( "view3d_lead_clips",
                   "Through-hole lead cut of the 3D viewer per footprint model: leads past the maximum protrusion "
                   "(IPC-A-610 Class 2: 2.5 mm; footprint field protrusion_mm) are cut to 1.0 mm unless something other than leads "
                   "hangs below the board or the model's <model>.meta.json says mount.below_board. Lists clipped "
                   "and left-whole models (all: every model) with the depth past the board; GUI only",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "all":{"type":"boolean","default":false},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_lead_clips );

KOPENAPI_REGISTER( "view3d_layers",
                   "3D viewer appearance: visibility of board, copper, silkscreen, soldermask, paste, user and fab "
                   "layers, footprint texts, model kinds, model bounding boxes and the navigator; set: {layer: "
                   "true|false} applies them like clicks in the viewer's Layers panel; also its colour swatches "
                   "(colors), the stackup / editor copper colour checkboxes, layer presets and the material mode "
                   "(View menu). Answers every layer's visibility and colour (#rrggbbaa, backgrounds too), the "
                   "material, the checkboxes, the presets and "
                   "rebuild: whether the change rebuilds the scene (view3d_status ready when done); GUI only",
                   R"json({"type":"object","properties":{
                        "set":{"type":"object","additionalProperties":{"type":"boolean"}},
                        "colors":{"type":"object","additionalProperties":{"type":"string"},"description":"layer: '#rrggbb' or '#rrggbbaa', as picked in its swatch; layers as in the answer's colors"},
                        "stackup_colors":{"type":"boolean","description":"'Use board stackup colors'"},
                        "editor_copper_colors":{"type":"boolean","description":"'Use PCB editor copper colors'"},
                        "preset":{"type":"string","description":"a layer preset by name"},
                        "material":{"type":"string","enum":["realistic","diffuse","cad"]},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_layers );

KOPENAPI_REGISTER( "view3d_options",
                   "3D viewer preferences and menu options: engine (opengl / raytracing), grid, antialiasing "
                   "(takes effect when the viewer opens next: reopen_needed), placeholders for missing models, "
                   "through-hole lead clipping, rollover highlight, selection colour, copper thickness, zones, "
                   "silkscreen clipping, plated copper, what is left out while moving, camera (animation, speed, "
                   "rotation increment, projection) and the raytracing settings (lights included); set: the "
                   "same shape, only what changes (e.g. {highlight_on_rollover: false, raytracing: {shadows: "
                   "false}}), checked first, all or nothing, applied as the viewer's Preferences / menus do. "
                   "Without an open viewer the stored settings are read and set (used when it opens). Answer: "
                   "every option, rebuild (the change rebuilds the scene: view3d_status ready when done), "
                   "viewer_open, reopen_needed; GUI only",
                   R"json({"type":"object","properties":{
                        "set":{"type":"object","description":"options as in the answer (nested), e.g. {grid: '5mm', camera: {projection: 'orthographic'}}"},
                        "open":{"type":"boolean","default":false,"description":"open the viewer from the board editor first"}}})json"_json,
                   true, h_view3d_options );

KOPENAPI_REGISTER( "view3d_camera",
                   "3D viewer camera: preset view (top, bottom, left, right, front, back, fit; animated), "
                   "then rotate_deg [x, y, z], look_at (turn about and look at parts or a point of the "
                   "board) and zoom factor; opens the viewer if needed; GUI only",
                   R"json({"type":"object","properties":{
                        "preset":{"type":"string","enum":["top","bottom","left","right","front","back","fit"]},
                        "animate":{"type":"boolean","default":true},
                        "rotate_deg":{"type":"array","items":{"type":"number"}},
                        "look_at":{"description":"the point the camera looks at and turns about, centred in the view: references (the middle of their parts), [x_mm, y_mm] on the board, or \"board\" (its middle, the default view)","oneOf":[{"type":"string","enum":["board"]},{"type":"array","items":{"type":["string","number"]}}]},
                        "zoom":{"type":"number","description":"> 1 closer"},
                        "fit_assembly":{"type":"boolean","default":true,"description":"frame the whole assembly (board + models over its edges) for any rotation"},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_camera, 300 );   // animations record on video time: slow while recording

KOPENAPI_REGISTER( "view3d_benchmark",
                   "Render benchmark of the 3D viewer (OpenGL): from the current camera, rounds of frames swept "
                   "over degrees about an axis by a fixed step per frame (no clock: the same poses each time), "
                   "for each variant (a view3d_options set, e.g. [{model_instancing: false}, {model_instancing: "
                   "true}]) in turn per round so a load change hits all alike; per variant paint ms (median, p95, "
                   "min, mean), the median of each round, the renderer's stages (redraw, layers, collect, "
                   "opaque_models, board, transparent_models, overlay, finish = glFinish when finish: true, the "
                   "GPU catching up; swap_and_rest = paint - redraw; vsync false by default: frames are not held to "
                   "the display rate), the models' instances, groups, draw calls "
                   "and whether they were instanced; speedup_vs_first. Options and camera put back afterwards. "
                   "Reading calls answer between frames, job_cancel stops it (stopped: true); GUI only",
                   R"json({"type":"object","properties":{
                        "variants":{"type":"array","items":{"type":"object"},"description":"view3d_options sets; default: [{}] (as it is)"},
                        "frames":{"type":"integer","default":120,"minimum":10,"maximum":5000},
                        "warmup":{"type":"integer","default":10,"minimum":0,"maximum":1000,"description":"unmeasured frames after a variant is set"},
                        "rounds":{"type":"integer","default":3,"minimum":1,"maximum":50},
                        "degrees":{"type":"number","default":90},
                        "axis":{"type":"string","enum":["x","y","z"],"default":"z"},
                        "vsync":{"type":"boolean","default":false,"description":"swap in step with the display, as the viewer does"},
                        "finish":{"type":"boolean","default":false,"description":"glFinish each frame: the GPU time shows (stages finish)"},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_benchmark, 1800 );

KOPENAPI_REGISTER( "view3d_orbit",
                   "Orbit the 3D viewer's camera smoothly (turntable for demos / recordings): degrees "
                   "over seconds around an axis (z = around the board normal); answers frames, frame_ms (paint time "
                   "per frame: median, p95, max) and stopped (job_cancel ended it early; the camera stays where it "
                   "got to). Reading calls (view3d_status, …) answer between the frames; GUI only",
                   R"json({"type":"object","properties":{
                        "degrees":{"type":"number","default":360},
                        "seconds":{"type":"number","default":8},
                        "axis":{"type":"string","enum":["x","y","z"],"default":"z"},
                        "fit_assembly":{"type":"boolean","default":true},
                        "framing":{"type":"string","enum":["follow","path"],"default":"follow","description":"follow: the zoom follows the assembly's reach along the way (close where it lies flat, wider while tilted, smooth, never clipped); path: one zoom for the whole way"},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_orbit, 180 );

KOPENAPI_REGISTER( "view3d_animate",
                   "Smooth 3D camera choreography (fly-around, rotation about all three axes, zoom; eased): "
                   "built-in effects turntable, tumble, flyover, showcase (speed factor), or your own moves "
                   "[{rotate_deg: [x, y, z], zoom, look_at, seconds}] played in order (look_at pans the "
                   "camera to parts or a point along the move: a tour of the board); for demos and recordings; the "
                   "viewer's manual behaviour is untouched; GUI only",
                   R"json({"type":"object","properties":{
                        "effect":{"type":"string","enum":["turntable","tumble","flyover","showcase"]},
                        "speed":{"type":"number","default":1,"description":"> 1 faster"},
                        "moves":{"type":"array","items":{"type":"object","properties":{
                            "rotate_deg":{"type":"array","items":{"type":"number"}},
                            "zoom":{"type":"number","default":1},
                            "look_at":{"description":"pan along the move to look at (and turn about) references, [x_mm, y_mm] or \"board\"","oneOf":[{"type":"string","enum":["board"]},{"type":"array","items":{"type":["string","number"]}}]},
                            "seconds":{"type":"number","default":1}}}},
                        "fit_assembly":{"type":"boolean","default":true,"description":"first frame the whole assembly (board + models over its edges) so nothing leaves the view while turning"},
                        "framing":{"type":"string","enum":["follow","path"],"default":"follow","description":"follow: the zoom follows the assembly's reach along the way (close where it lies flat, wider while tilted, smooth, never clipped); path: one zoom for the whole way"},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_animate, 300 );
