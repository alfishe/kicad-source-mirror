/// @file kicadopenapi_steadycam.h
/// @brief Steadycam for recordings: a view put somewhere new eases there in the recorded picture
/// instead of jumping. Only the frame is drawn from the eased pose; the view's real state is put
/// back right after, so the editor (and KiCad's own camera state) never sees it.
///
/// FILTER follows the target through two exponential stages (critically damped feel; the motion
/// ends later than the jump). TIMED plans a path to the target and moves along it with a smooth
/// start and stop (cubic Hermite in the path's progress) that arrives exactly the move's duration
/// later; a new target mid-way starts from the current speed. Generic over the view's pose
/// (GAL: centre + log scale; 3D: rotation quaternion + log zoom). Header-only, used by the board
/// and schematic kifaces.
#ifndef KICADOPENAPI_STEADYCAM_H
#define KICADOPENAPI_STEADYCAM_H

#include <class_draw_panel_gal.h>
#include <kicadopenapi_registry.h>
#include <view/view.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>


/// @brief Progress along a TIMED move: s( 0 ) = 0, s( 1 ) = 1, ds/dtau( 0 ) = aStartSlope, ds/dtau( 1 ) = 0
inline double KopenapiTimedProgress( double aTau, double aStartSlope )
{
    const double t = std::clamp( aTau, 0.0, 1.0 );
    return ( -2 * t * t * t + 3 * t * t ) + ( t * t * t - 2 * t * t + t ) * aStartSlope;
}


/// @brief d(progress)/dtau of KopenapiTimedProgress
inline double KopenapiTimedSlope( double aTau, double aStartSlope )
{
    const double t = std::clamp( aTau, 0.0, 1.0 );
    return ( -6 * t * t + 6 * t ) + ( 3 * t * t - 4 * t + 1 ) * aStartSlope;
}


/// @brief Steadycam state for one view: POSE has lerp( a, b, s ), distance( a, b ) and
/// rate( from, to, velocityDelta ) (the part of a pose change per second along from -> to)
template <typename POSE>
struct KOPENAPI_STEADY_TRACK
{
    POSE   target, s1, s2;        ///< FILTER stages; s2 is shown
    POSE   from, to;              ///< TIMED move
    double elapsed = 0, slope = 0;
    POSE   shown;                 ///< the pose of the last recorded frame
    POSE   seen;                  ///< the view's real pose at the last step

    void reset( const POSE& aPose )
    {
        target = s1 = s2 = from = to = shown = seen = aPose;
        elapsed = 1e9;
        slope = 0;
    }

    /// @brief One step: aCurrent is the view's real pose now; answers the pose to record
    template <typename OPS>
    POSE step( const POSE& aCurrent, double aDt, const KOPENAPI_STEADY& aSteady, const OPS& aOps )
    {
        const bool moved = aOps.distance( aCurrent, seen ) > 1e-7;
        seen = aCurrent;

        if( aSteady.mode == KOPENAPI_STEADY::FILTER )
        {
            if( moved )
                target = aCurrent;

            const double a = 1.0 - std::exp( -aDt / ( std::max( 0.02, aSteady.smoothSeconds ) / 2 ) );
            s1 = aOps.lerp( s1, target, a );
            s2 = aOps.lerp( s2, s1, a );
            from = to = shown = s2;
            elapsed = 1e9;
            return s2;
        }

        const double T = std::max( 0.05, aSteady.durationSeconds );

        if( moved )
        {
            // carry the speed we have into the new move (along its direction, no backward start)
            double speed = 0;

            if( elapsed < T )
                speed = aOps.rate( from, to, shown, aCurrent ) * KopenapiTimedSlope( elapsed / T, slope ) / T;

            from = shown;
            to = aCurrent;
            slope = std::clamp( speed * T, 0.0, 2.0 );
            elapsed = 0;
        }

        elapsed += aDt;
        shown = elapsed >= T ? to : aOps.lerp( from, to, KopenapiTimedProgress( elapsed / T, slope ) );
        s1 = s2 = target = shown;
        return shown;
    }
};


/// @brief GAL pose: centre (in view widths, so position and scale weigh alike) and log scale
struct KOPENAPI_GAL_POSE
{
    VECTOR2D centre;
    double   logScale = 0;
};


struct KOPENAPI_GAL_OPS
{
    double width = 1;   ///< the view's width in world units (normalises centre distances)

    KOPENAPI_GAL_POSE lerp( const KOPENAPI_GAL_POSE& a, const KOPENAPI_GAL_POSE& b, double s ) const
    {
        return { a.centre + ( b.centre - a.centre ) * s, a.logScale + ( b.logScale - a.logScale ) * s };
    }

    double distance( const KOPENAPI_GAL_POSE& a, const KOPENAPI_GAL_POSE& b ) const
    {
        const VECTOR2D d = ( b.centre - a.centre ) / width;
        return std::sqrt( d.x * d.x + d.y * d.y + std::pow( b.logScale - a.logScale, 2 ) );
    }

    /// @brief How much of the new move (aShown -> aTarget) one unit of the old move's progress covers
    double rate( const KOPENAPI_GAL_POSE& aFrom, const KOPENAPI_GAL_POSE& aTo, const KOPENAPI_GAL_POSE& aShown,
                 const KOPENAPI_GAL_POSE& aTarget ) const
    {
        const VECTOR2D v = ( aTo.centre - aFrom.centre ) / width, d = ( aTarget.centre - aShown.centre ) / width;
        const double   vs = aTo.logScale - aFrom.logScale, ds = aTarget.logScale - aShown.logScale;
        const double   len2 = d.x * d.x + d.y * d.y + ds * ds;
        return len2 > 1e-12 ? ( v.x * d.x + v.y * d.y + vs * ds ) / len2 : 0.0;
    }
};


/// @brief One steadycam step for a GAL canvas; false when aWindow is not one
inline bool KopenapiSteadyGal( wxWindow* aWindow, double aDt, const KOPENAPI_STEADY& aSteady, bool aExact,
                               std::function<void()>& aRestore )
{
    auto* canvas = dynamic_cast<EDA_DRAW_PANEL_GAL*>( aWindow );

    if( !canvas || !canvas->GetView() )
        return false;

    static std::map<wxWindow*, KOPENAPI_STEADY_TRACK<KOPENAPI_GAL_POSE>> tracks;

    KIGFX::VIEW*            view = canvas->GetView();
    const KOPENAPI_GAL_POSE now{ view->GetCenter(), std::log( view->GetScale() ) };
    KOPENAPI_GAL_OPS        ops;
    ops.width = std::max( 1.0, view->GetViewport().GetWidth() );
    auto it = tracks.find( aWindow );

    // first frame, a pause, steadycam off or an API animation: follow exactly
    if( it == tracks.end() || aDt > 0.5 || aExact || aSteady.mode == KOPENAPI_STEADY::OFF )
    {
        tracks[aWindow].reset( now );
        return true;
    }

    const KOPENAPI_GAL_POSE show = it->second.step( now, aDt, aSteady, ops );

    if( ops.distance( show, now ) < 1e-9 )
        return true;

    view->SetScale( std::exp( show.logScale ) );
    view->SetCenter( show.centre );

    aRestore = [canvas, view, now]()
    {
        view->SetScale( std::exp( now.logScale ) );
        view->SetCenter( now.centre );
        canvas->Refresh();
    };

    return true;
}

#endif
