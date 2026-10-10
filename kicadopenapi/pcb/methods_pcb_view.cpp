/// @file methods_pcb_view.cpp
/// @brief kicadopenapi board-side views: pcb_view_capture (board editor, footprint editor, footprint
/// viewer, 3D viewer canvases as shown), the canvas read-back window_capture composes,
/// pcb_view_zoom, view3d_camera, view3d_orbit.
#include "kopenapi_pcb.h"

#include <3d_canvas/eda_3d_canvas.h>
#include <3d_viewer/eda_3d_viewer_frame.h>
#include <3d_viewer/eda_3d_viewer_settings.h>
#include <class_draw_panel_gal.h>
#include <kicadopenapi_image.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <pcb_base_frame.h>
#include <board.h>
#include <footprint.h>
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

#include <chrono>
#include <map>
#include <cmath>
#include <thread>

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
    KopenapiMoveView( frame->GetCanvas(), BOX2D( area.GetOrigin(), area.GetSize() ),
                      std::clamp( aArgs.value( "animate_ms", 600 ), 0, 5000 ) );

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


/// @brief The board's 3D viewer, opened when asked
static EDA_3D_VIEWER_FRAME* viewer3d( KOPENAPI_CONTEXT& aCtx, bool aOpen )
{
    auto* frame = aCtx.kiway ? dynamic_cast<EDA_3D_VIEWER_FRAME*>( aCtx.kiway->Player( FRAME_PCB_DISPLAY3D, false ) )
                             : nullptr;

    if( !frame && aOpen && aCtx.kiway )
    {
        if( auto* board = dynamic_cast<PCB_BASE_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
            frame = board->CreateAndShow3D_Frame();
    }

    return frame && frame->GetCanvas() && frame->IsShown() ? frame : nullptr;
}


/// @brief One camera move: rotation (degrees about x, y, z) and zoom factor over some seconds
struct CAMERA_MOVE
{
    double rotate[3] = { 0, 0, 0 };
    double zoom = 1.0;
    double seconds = 1.0;
};


/// @brief Zoom correction along the moves (follow framing): (time into the moves, factor)
using FRAMING = std::vector<std::pair<double, double>>;

static nlohmann::json fitTrajectory( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard, const std::vector<CAMERA_MOVE>& aMoves,
                                    double aSeconds, FRAMING* aFollow = nullptr );


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

    if( aArgs.contains( "zoom" ) )
        camera.Zoom( aArgs["zoom"].get<float>() );

    // frame the whole assembly in this pose (unless a zoom was asked for)
    nlohmann::json fit;

    if( aArgs.value( "fit_assembly", true ) && !aArgs.contains( "zoom" ) )
        fit = fitTrajectory( frame, boardOf( aCtx ), {}, aArgs.value( "animate", true ) ? 0.5 : 0.0 );

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
}


/// @brief Plays camera moves in order, each eased in and out, repainting as it goes (on video
/// time while a recording runs)
static int playMoves( EDA_3D_VIEWER_FRAME* aFrame, const std::vector<CAMERA_MOVE>& aMoves,
                      const FRAMING* aFollow = nullptr )
{
    CAMERA& camera = aFrame->GetCurrentCamera();
    int     frames = 0;
    double  start = 0;                                            // time into the moves at this move
    double  applied = aFollow ? framingAt( *aFollow, 0 ) : 1.0;   // correction in the camera now

    for( const CAMERA_MOVE& move : aMoves )
    {
        double done = 0;

        KopenapiAnimate( move.seconds,
                         [&]( double e )
                         {
                             stepMove( camera, move, done, e );
                             done = e;

                             if( aFollow )
                             {
                                 const double k = framingAt( *aFollow, start + e * move.seconds );
                                 camera.Zoom( (float) ( applied / k ) );
                                 applied = k;
                             }

                             aFrame->GetCanvas()->DoRePaint();

                             frames++;
                         } );

        start += move.seconds;
    }

    return frames;
}


/// @brief The assembly's box corners (board + every shown model) in the 3D viewer's world units
static std::vector<glm::vec4> assemblyCorners( EDA_3D_VIEWER_FRAME* aFrame, BOARD* aBoard )
{
    double lo[3], hi[3];

    if( !aBoard || !KopenapiAssemblyBox( aBoard, lo, hi ) )
        return {};

    // board mm -> the viewer's world: through the board's own 3D box (however the adapter places
    // and scales it); the board's top surface is z 0 in mm
    const BOARD_ADAPTER& adapter = aFrame->GetAdapter();
    const BOX2I          outline = aBoard->GetBoardEdgesBoundingBox();
    const SFVEC3F        bmin = adapter.GetBBox().Min();
    const SFVEC3F        bmax = adapter.GetBBox().Max();

    if( outline.GetWidth() <= 0 || outline.GetHeight() <= 0 )
        return {};

    const double ox = outline.GetLeft() / pcbIUScale.IU_PER_MM;
    const double oy = outline.GetTop() / pcbIUScale.IU_PER_MM;
    const double sx = ( bmax.x - bmin.x ) / ( outline.GetWidth() / pcbIUScale.IU_PER_MM );
    const double sy = ( bmax.y - bmin.y ) / ( outline.GetHeight() / pcbIUScale.IU_PER_MM );
    std::vector<glm::vec4> corners;

    for( int c = 0; c < 8; ++c )
    {
        const double x = ( c & 1 ) ? hi[0] : lo[0];
        const double y = ( c & 2 ) ? hi[1] : lo[1];
        const double z = ( c & 4 ) ? hi[2] : lo[2];
        corners.emplace_back( (float) ( bmin.x + ( x - ox ) * sx ),
                              (float) ( bmax.y - ( y - oy ) * sy ),   // board y runs down, the world's up
                              (float) ( bmax.z + z * sx ), 1.0f );
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

    KopenapiAnimate( aSeconds,
                     [&]( double e )
                     {
                         const double want = zoom0 + ( target - zoom0 ) * e;
                         camera.Zoom( (float) ( camera.GetZoom() / want ) );
                         aFrame->GetCanvas()->DoRePaint();
                     } );

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
        moves.push_back( move );
    }

    if( moves.empty() )
        return KOPENAPI_RESULT::Error( 400, "give effect or moves" );

    nlohmann::json fit;
    FRAMING        follow;
    const bool     following = aArgs.value( "framing", std::string( "follow" ) ) == "follow";

    if( aArgs.value( "fit_assembly", true ) )
        fit = fitTrajectory( frame, boardOf( aCtx ), moves, 0.7, following ? &follow : nullptr );

    double total = 0;

    for( const CAMERA_MOVE& m : moves )
        total += m.seconds;

    const int frames = playMoves( frame, moves, follow.empty() ? nullptr : &follow );
    return KOPENAPI_RESULT::Ok( { { "moves", moves.size() }, { "seconds", total }, { "frames", frames }, { "fit", fit } } );
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

    if( aArgs.value( "fit_assembly", true ) )
        fitTrajectory( frame, boardOf( aCtx ), { move }, 0.7,
                       aArgs.value( "framing", std::string( "follow" ) ) == "follow" ? &follow : nullptr );

    const int frames = playMoves( frame, { move }, follow.empty() ? nullptr : &follow );
    return KOPENAPI_RESULT::Ok( { { "degrees", aArgs.value( "degrees", 360.0 ) }, { "seconds", move.seconds },
                                  { "frames", frames } } );
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

KOPENAPI_REGISTER( "view3d_camera",
                   "3D viewer camera: preset view (top, bottom, left, right, front, back, fit; animated), "
                   "then rotate_deg [x, y, z] and zoom factor; opens the viewer if needed; GUI only",
                   R"json({"type":"object","properties":{
                        "preset":{"type":"string","enum":["top","bottom","left","right","front","back","fit"]},
                        "animate":{"type":"boolean","default":true},
                        "rotate_deg":{"type":"array","items":{"type":"number"}},
                        "zoom":{"type":"number","description":"> 1 closer"},
                        "fit_assembly":{"type":"boolean","default":true,"description":"frame the whole assembly (board + models over its edges) for any rotation"},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_camera, 300 );   // animations record on video time: slow while recording

KOPENAPI_REGISTER( "view3d_orbit",
                   "Orbit the 3D viewer's camera smoothly (turntable for demos / recordings): degrees "
                   "over seconds around an axis (z = around the board normal); GUI only",
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
                   "[{rotate_deg: [x, y, z], zoom, seconds}] played in order; for demos and recordings; the "
                   "viewer's manual behaviour is untouched; GUI only",
                   R"json({"type":"object","properties":{
                        "effect":{"type":"string","enum":["turntable","tumble","flyover","showcase"]},
                        "speed":{"type":"number","default":1,"description":"> 1 faster"},
                        "moves":{"type":"array","items":{"type":"object","properties":{
                            "rotate_deg":{"type":"array","items":{"type":"number"}},
                            "zoom":{"type":"number","default":1},
                            "seconds":{"type":"number","default":1}}}},
                        "fit_assembly":{"type":"boolean","default":true,"description":"first frame the whole assembly (board + models over its edges) so nothing leaves the view while turning"},
                        "framing":{"type":"string","enum":["follow","path"],"default":"follow","description":"follow: the zoom follows the assembly's reach along the way (close where it lies flat, wider while tilted, smooth, never clipped); path: one zoom for the whole way"},
                        "open":{"type":"boolean","default":true}}})json"_json,
                   true, h_view3d_animate, 300 );
