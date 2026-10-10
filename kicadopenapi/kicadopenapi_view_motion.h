/// @file kicadopenapi_view_motion.h
/// @brief Moves an editor's view (GAL canvas) to an area, optionally animated, with the UI alive
/// during the motion (recordings see a camera move instead of a jump). Header-only, used by the
/// board and schematic kifaces.
#ifndef KICADOPENAPI_VIEW_MOTION_H
#define KICADOPENAPI_VIEW_MOTION_H

#include <class_draw_panel_gal.h>
#include <kicadopenapi_keepalive.h>
#include <recording/kicadopenapi_recorder.h>
#include <view/view.h>

#include <functional>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>


/// @brief Runs an animation: aStep( e ) shows the state at eased progress e (0..1, smoothstep) and
/// repaints. While a recording runs it steps on video time (every video frame recorded, never
/// dropped), with extra on-screen steps in between at the display's rate; otherwise on the wall
/// clock with the UI alive.
inline void KopenapiAnimate( double aSeconds, const std::function<void( double )>& aStep )
{
    auto ease = []( double t ) { return t * t * ( 3 - 2 * t ); };
    int  fps = 0;

    if( aSeconds <= 0 )
    {
        aStep( 1.0 );
        KopenapiKeepUiAlive();
        return;
    }

    // the motion is smooth already: a recording's steadycam follows it exactly
    KopenapiSetAnimating( true );

    struct ANIMATING_END
    {
        ~ANIMATING_END() { KopenapiSetAnimating( false ); }
    } animatingEnd;

    if( KopenapiRecordingSync( &fps ) && fps > 0 )
    {
        const int frames = std::max( 1, int( std::lround( aSeconds * fps ) ) );
        const int  screen = KopenapiRecordingScreenSteps();
        const auto start = std::chrono::steady_clock::now();
        const auto period = std::chrono::duration<double>( 1.0 / ( double( fps ) * screen ) );

        for( int i = 1; i <= frames; ++i )
        {
            for( int s = 1; s <= screen; ++s )
            {
                // not ahead of real time on screen; when slower the video still gets every frame
                std::this_thread::sleep_until( start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                               period * ( ( i - 1 ) * screen + s - 1 ) ) );
                aStep( ease( ( i - 1 + double( s ) / screen ) / frames ) );
                KopenapiKeepUiAlive();
            }

            KopenapiRecordFrame();
        }

        return;
    }

    const auto start = std::chrono::steady_clock::now();

    for( ;; )
    {
        const double t = std::min( 1.0, std::chrono::duration<double>( std::chrono::steady_clock::now() - start ).count()
                                                / aSeconds );
        aStep( ease( t ) );
        KopenapiKeepUiAlive();

        if( t >= 1.0 )
            break;

        std::this_thread::sleep_for( std::chrono::milliseconds( 8 ) );
    }
}


/// @brief Shows aTarget (internal units) in the canvas; over aAnimateMs with an ease-in-out motion
inline void KopenapiMoveView( EDA_DRAW_PANEL_GAL* aCanvas, const BOX2D& aTarget, int aAnimateMs )
{
    KIGFX::VIEW* view = aCanvas->GetView();
    const BOX2D  from = view->GetViewport();

    KopenapiAnimate( aAnimateMs / 1000.0,
                     [&]( double e )
                     {
                         const auto mix = [e]( double a, double b ) { return a + ( b - a ) * e; };
                         view->SetViewport( BOX2D( VECTOR2D( mix( from.GetX(), aTarget.GetX() ), mix( from.GetY(), aTarget.GetY() ) ),
                                                   VECTOR2D( mix( from.GetWidth(), aTarget.GetWidth() ),
                                                             mix( from.GetHeight(), aTarget.GetHeight() ) ) ) );
                         aCanvas->Refresh();
                     } );

    view->SetViewport( aTarget );
    aCanvas->Refresh();
    KopenapiKeepUiAlive();
}

#endif
