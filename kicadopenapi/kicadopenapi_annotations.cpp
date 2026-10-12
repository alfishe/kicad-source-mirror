/// @file kicadopenapi_annotations.cpp
/// @brief The annotation layer (see kicadopenapi_annotations.h).
#include <kicadopenapi_annotations.h>
#include <kicadopenapi_model_events.h>

#include <base_units.h>
#include <class_draw_panel_gal.h>
#include <eda_draw_frame.h>
#include <gal/color4d.h>
#include <geometry/eda_angle.h>
#include <view/view.h>
#include <view/view_overlay.h>

#include <wx/event.h>
#include <wx/timer.h>
#include <wx/utils.h>
#include <wx/weakref.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <numbers>
#include <vector>


namespace
{
using CLOCK = std::chrono::steady_clock;
using KIGFX::COLOR4D;

constexpr size_t MAX_ANNOTATIONS = 500;
constexpr size_t MAX_TEXT = 200;
constexpr int    FAST_TICK_MS = 33;    ///< animations: pointer, pulses, fading
constexpr int    FOLLOW_TICK_MS = 150; ///< item anchors following edits / drags
constexpr double TRAIL_S = 0.7;
constexpr double RIPPLE_S = 0.45;
constexpr double KEY_S = 1.4;
constexpr double POINTER_IDLE_S = 4.0; ///< the cursor fades after this long without input


VECTOR2D rotated( const VECTOR2D& aVector, double aRadians )
{
    const double c = std::cos( aRadians );
    const double s = std::sin( aRadians );
    return VECTOR2D( aVector.x * c - aVector.y * s, aVector.x * s + aVector.y * c );
}


double seconds( CLOCK::duration aDuration )
{
    return std::chrono::duration<double>( aDuration ).count();
}


COLOR4D parseColour( const nlohmann::json& aValue, const COLOR4D& aDefault )
{
    if( !aValue.is_string() )
        return aDefault;

    const std::string hex = aValue.get<std::string>();

    if( hex.size() != 7 || hex[0] != '#' )
        return aDefault;

    const unsigned long rgb = std::strtoul( hex.c_str() + 1, nullptr, 16 );
    return COLOR4D( ( ( rgb >> 16 ) & 0xFF ) / 255.0, ( ( rgb >> 8 ) & 0xFF ) / 255.0, ( rgb & 0xFF ) / 255.0, 1.0 );
}


struct ANCHOR
{
    bool        set = false;
    bool        isItem = false;
    std::string ref;      ///< uuid or reference designator
    VECTOR2D    point;    ///< internal units, when not an item
};


struct ANNOTATION
{
    std::string       id;
    std::string       kind;
    std::string       text;
    std::string       group;
    std::string       onOrphan = "keep";   ///< keep (greyed at the last place), hide, remove
    ANCHOR            target;
    ANCHOR            to;
    std::optional<BOX2D> area;              ///< internal units
    COLOR4D           colour{ 1.0, 0.55, 0.0, 1.0 };
    double            widthPx = 3.0;
    double            ttl = 0.0;            ///< seconds, 0 = until removed
    bool              pulse = false;
    int               number = 0;
    CLOCK::time_point created = CLOCK::now();
    bool              orphaned = false;
    std::optional<BOX2D> lastTarget;        ///< resolved boxes from the last redraw
    std::optional<BOX2D> lastTo;
    std::string       attach;               ///< item the point anchors and the area move with
    VECTOR2D          attachOrigin;         ///< that item's centre when attached
    VECTOR2D          attachShift;          ///< its move since then, as of the last redraw
    std::array<double, 4> attachLinear{ 1, 0, 0, 1 };   ///< the item's linear map when attached
    std::array<double, 4> attachTurn{ 1, 0, 0, 1 };     ///< its change since then (now * inverse( attached ))
    VECTOR2D          attachCentre;         ///< its centre as of the last redraw

    /// @brief Where aPoint (as placed at attach time) is now: turned with the item about its centre
    VECTOR2D Attached( const VECTOR2D& aPoint ) const
    {
        const VECTOR2D v = aPoint - attachOrigin;
        const auto&    m = attachTurn;
        return attachCentre + VECTOR2D( m[0] * v.x + m[1] * v.y, m[2] * v.x + m[3] * v.y );
    }
};


/// @brief a * inverse( b ) of 2x2 maps { xx, xy, yx, yy }; identity when b is singular
std::array<double, 4> turnSince( const std::array<double, 4>& a, const std::array<double, 4>& b )
{
    const double det = b[0] * b[3] - b[1] * b[2];

    if( std::abs( det ) < 1e-12 )
        return { 1, 0, 0, 1 };

    const std::array<double, 4> inv{ b[3] / det, -b[1] / det, -b[2] / det, b[0] / det };
    return { a[0] * inv[0] + a[1] * inv[2], a[0] * inv[1] + a[1] * inv[3],
             a[2] * inv[0] + a[3] * inv[2], a[2] * inv[1] + a[3] * inv[3] };
}


struct POINTER
{
    bool                                         shown = false;
    VECTOR2D                                     position;
    CLOCK::time_point                            lastInput;
    std::deque<std::pair<CLOCK::time_point, VECTOR2D>> trail;
    struct RIPPLE
    {
        CLOCK::time_point start;
        VECTOR2D          at;
        int               rings;
    };
    std::deque<RIPPLE>                           ripples;
    wxString                                     key;
    CLOCK::time_point                            keyTime;
    bool                                         held = false;
};


class LAYER : public wxEvtHandler
{
public:
    explicit LAYER( EDA_DRAW_FRAME* aFrame ) : m_frame( aFrame )
    {
        m_timer.SetOwner( this );
        Bind( wxEVT_TIMER, [this]( wxTimerEvent& ) { Redraw(); }, m_timer.GetId() );
    }

    ~LAYER() override
    {
        m_timer.Stop();
        dropOverlay();
    }

    bool Alive() const { return m_frame && m_frame->GetCanvas(); }

    void SetHost( const KOPENAPI_ANNOTATION_HOST& aHost )
    {
        const bool newDomain = m_host.domain != aHost.domain;
        m_host = aHost;

        if( newDomain || !m_token )
        {
            m_token = KOPENAPI_MODEL_EVENTS::Subscribe( "annotations:" + aHost.domain, aHost.domain,
                                                        [this]( const KOPENAPI_MODEL_EVENTS::EVENT& )
                                                        {
                                                            Redraw();
                                                        } );
        }
    }

    std::vector<ANNOTATION>& Annotations() { return m_annotations; }
    POINTER&                 Pointer() { return m_pointer; }
    const KOPENAPI_ANNOTATION_HOST& Host() const { return m_host; }

    /// @brief Resolve every anchor, drop expired / orphan-removed annotations, draw, schedule
    void Redraw();

private:
    std::optional<BOX2D> resolve( const ANCHOR& aAnchor ) const
    {
        if( !aAnchor.set )
            return std::nullopt;

        if( !aAnchor.isItem )
            return BOX2D( aAnchor.point, VECTOR2D( 0, 0 ) );

        if( !m_host.resolve )
            return std::nullopt;

        if( std::optional<BOX2I> box = m_host.resolve( aAnchor.ref ) )
            return BOX2D( VECTOR2D( box->GetOrigin() ), VECTOR2D( box->GetSize() ) );

        return std::nullopt;
    }

    void ensureOverlay( KIGFX::VIEW* aView )
    {
        if( m_overlay && m_overlayCanvas && m_overlayCanvas.get() == m_frame->GetCanvas() && m_overlayView == aView )
            return;

        dropOverlay();
        m_overlay = aView->MakeOverlay();
        m_overlayView = aView;
        m_overlayCanvas = m_frame->GetCanvas();
    }

    /// @brief As the glow does: an overlay whose view is gone is abandoned, not destroyed
    void dropOverlay()
    {
        if( m_overlay && !( m_overlayCanvas && m_overlayCanvas->GetView() == m_overlayView ) )
            new std::shared_ptr<KIGFX::VIEW_OVERLAY>( std::move( m_overlay ) );

        m_overlay.reset();
        m_overlayView = nullptr;
        m_overlayCanvas = nullptr;
    }

    void drawLabel( KIGFX::VIEW_OVERLAY& aOverlay, double aPx, const VECTOR2D& aAt, const wxString& aText,
                    const COLOR4D& aColour, double aAlpha );
    void drawAnnotation( KIGFX::VIEW_OVERLAY& aOverlay, double aPx, const BOX2D& aViewport, ANNOTATION& aNote,
                         double aAlpha, CLOCK::time_point aNow );
    void drawPointer( KIGFX::VIEW_OVERLAY& aOverlay, double aPx, const BOX2D& aViewport, CLOCK::time_point aNow );

    wxWeakRef<EDA_DRAW_FRAME>                       m_frame;
    KOPENAPI_ANNOTATION_HOST                        m_host;
    std::unique_ptr<KOPENAPI_MODEL_EVENTS::TOKEN>   m_token;
    std::vector<ANNOTATION>                         m_annotations;
    POINTER                                         m_pointer;
    wxTimer                                         m_timer;
    wxWeakRef<EDA_DRAW_PANEL_GAL>                   m_overlayCanvas;
    std::shared_ptr<KIGFX::VIEW_OVERLAY>            m_overlay;
    KIGFX::VIEW*                                    m_overlayView = nullptr;
};


/// @brief Stacking of the kinds, top first: texts over arrows over marked areas over dimming
int zBand( const std::string& aKind )
{
    if( aKind == "key_hint" )
        return 0;

    if( aKind == "label" || aKind == "step" )
        return 1;

    if( aKind == "arrow" )
        return 2;

    if( aKind == "highlight" || aKind == "region" )
        return 3;

    return 4;   // spotlight: dims everything else
}


void LAYER::drawLabel( KIGFX::VIEW_OVERLAY& aOverlay, double aPx, const VECTOR2D& aAt, const wxString& aText,
                       const COLOR4D& aColour, double aAlpha )
{
    const double glyph = 15 * aPx;
    const double width = aText.length() * glyph * 0.8 + 18 * aPx;
    const double height = glyph * 1.7;
    const VECTOR2D topLeft( aAt.x, aAt.y - height / 2 );

    // the GAL keeps the first of equally deep shapes: what is on top is drawn first
    aOverlay.SetIsFill( false );
    aOverlay.SetIsStroke( true );
    aOverlay.SetStrokeColor( COLOR4D( 1, 1, 1, aAlpha ) );
    aOverlay.SetLineWidth( glyph * 0.16 );
    aOverlay.SetGlyphSize( VECTOR2I( KiROUND( glyph ), KiROUND( glyph ) ) );
    aOverlay.SetTextJustify( GR_TEXT_H_ALIGN_LEFT, GR_TEXT_V_ALIGN_CENTER );
    aOverlay.StrokeText( aText, VECTOR2I( KiROUND( aAt.x + 8 * aPx ), KiROUND( aAt.y ) ), ANGLE_0 );

    aOverlay.SetIsFill( true );
    aOverlay.SetLineWidth( 2 * aPx );
    // opaque: the layer is always above the document, nothing of it shows through a label
    aOverlay.SetFillColor( COLOR4D( 0.08, 0.08, 0.10, aAlpha ) );
    aOverlay.SetStrokeColor( aColour.WithAlpha( aAlpha ) );
    aOverlay.Rectangle( topLeft, topLeft + VECTOR2D( width, height ) );
}


void LAYER::drawAnnotation( KIGFX::VIEW_OVERLAY& aOverlay, double aPx, const BOX2D& aViewport, ANNOTATION& aNote,
                            double aAlpha, CLOCK::time_point aNow )
{
    std::optional<BOX2D> target = resolve( aNote.target );
    std::optional<BOX2D> to = resolve( aNote.to );

    // points and the area attached to an item move with it
    if( !aNote.attach.empty() )
    {
        ANCHOR item;
        item.set = true;
        item.isItem = true;
        item.ref = aNote.attach;

        if( std::optional<BOX2D> box = resolve( item ) )
        {
            aNote.attachCentre = box->Centre();
            aNote.attachShift = aNote.attachCentre - aNote.attachOrigin;

            if( m_host.linear )
            {
                if( std::optional<std::array<double, 4>> now = m_host.linear( aNote.attach ) )
                    aNote.attachTurn = turnSince( *now, aNote.attachLinear );
            }
        }
        else
        {
            aNote.orphaned = true;
        }

        if( target && !aNote.target.isItem )
            *target = BOX2D( aNote.Attached( aNote.target.point ), VECTOR2D( 0, 0 ) );

        if( to && !aNote.to.isItem )
            *to = BOX2D( aNote.Attached( aNote.to.point ), VECTOR2D( 0, 0 ) );
    }

    if( aNote.target.set && aNote.target.isItem && !target )
        aNote.orphaned = true;
    else if( target )
        aNote.lastTarget = target;

    if( aNote.to.set && aNote.to.isItem && !to )
        aNote.orphaned = true;
    else if( to )
        aNote.lastTo = to;

    COLOR4D colour = aNote.colour;

    if( aNote.orphaned )
    {
        if( aNote.onOrphan == "hide" )
            return;

        target = aNote.lastTarget;
        to = aNote.lastTo;
        colour = COLOR4D( 0.55, 0.55, 0.55, 1.0 );
    }

    if( aNote.pulse )
        aAlpha *= 0.65 + 0.35 * std::sin( seconds( aNow - aNote.created ) * 2 * std::numbers::pi / 1.2 );

    const double width = aNote.widthPx * aPx;
    const wxString text = wxString::FromUTF8( aNote.text );

    aOverlay.SetLineWidth( width );
    aOverlay.SetStrokeColor( colour.WithAlpha( aAlpha ) );

    if( aNote.kind == "highlight" && target )
    {
        const double m = 6 * aPx;
        const VECTOR2D a = target->GetOrigin() - VECTOR2D( m, m );
        const VECTOR2D b = target->GetEnd() + VECTOR2D( m, m );

        if( !text.IsEmpty() )
            drawLabel( aOverlay, aPx, VECTOR2D( b.x + 10 * aPx, a.y ), text, colour, aAlpha );

        aOverlay.SetLineWidth( width );
        aOverlay.SetStrokeColor( colour.WithAlpha( aAlpha ) );
        aOverlay.SetIsFill( true );
        aOverlay.SetIsStroke( true );
        aOverlay.SetFillColor( colour.WithAlpha( 0.12 * aAlpha ) );
        aOverlay.Rectangle( a, b );
    }
    else if( aNote.kind == "arrow" && target && to )
    {
        VECTOR2D       start = target->Centre();
        VECTOR2D       end = to->Centre();
        const VECTOR2D dir = ( end - start ).Resize( 1.0 );

        // stop short of an item's box so the head points at it
        if( aNote.to.isItem )
        {
            const double half = std::max( to->GetWidth(), to->GetHeight() ) / 2 + 8 * aPx;
            end = end - dir * half;
        }

        if( aNote.target.isItem )
            start = start + dir * ( std::max( target->GetWidth(), target->GetHeight() ) / 2 + 8 * aPx );

        if( !text.IsEmpty() )
            drawLabel( aOverlay, aPx, start + VECTOR2D( 8 * aPx, -18 * aPx ), text, colour, aAlpha );

        aOverlay.SetStrokeColor( colour.WithAlpha( aAlpha ) );
        aOverlay.SetIsStroke( true );
        aOverlay.SetIsFill( false );
        aOverlay.SetFillColor( colour.WithAlpha( aAlpha ) );

        const double   head = 16 * aPx;
        const VECTOR2D back = dir * -head;
        aOverlay.Segment( end, end + rotated( back, 0.45 ), width );
        aOverlay.Segment( end, end + rotated( back, -0.45 ), width );
        aOverlay.Segment( start, end, width );
    }
    else if( aNote.kind == "label" && target )
    {
        const VECTOR2D at( target->GetRight() + 24 * aPx, target->GetTop() - 24 * aPx );

        drawLabel( aOverlay, aPx, at, text, colour, aAlpha );
        aOverlay.SetLineWidth( width );
        aOverlay.SetStrokeColor( colour.WithAlpha( aAlpha ) );
        aOverlay.SetIsStroke( true );
        aOverlay.Line( VECTOR2D( target->GetRight(), target->GetTop() ), at );
    }
    else if( ( aNote.kind == "region" || aNote.kind == "spotlight" ) && ( aNote.area || target ) )
    {
        BOX2D box = aNote.area ? *aNote.area : *target;

        // an attached area: its corners turned with the item, the box around them
        if( aNote.area && !aNote.attach.empty() )
        {
            BOX2D turned( aNote.Attached( aNote.area->GetOrigin() ), VECTOR2D( 0, 0 ) );

            for( const VECTOR2D& c : { VECTOR2D( aNote.area->GetRight(), aNote.area->GetTop() ),
                                       VECTOR2D( aNote.area->GetLeft(), aNote.area->GetBottom() ), aNote.area->GetEnd() } )
            {
                turned.Merge( aNote.Attached( c ) );
            }

            box = turned;
        }

        if( !aNote.area )
            box.Inflate( 10 * aPx );

        if( !text.IsEmpty() )
            drawLabel( aOverlay, aPx, VECTOR2D( box.GetLeft(), box.GetTop() - 20 * aPx ), text, colour, aAlpha );

        aOverlay.SetLineWidth( width );
        aOverlay.SetStrokeColor( colour.WithAlpha( aAlpha ) );
        aOverlay.SetIsFill( aNote.kind == "region" );
        aOverlay.SetFillColor( colour.WithAlpha( 0.08 * aAlpha ) );
        aOverlay.SetIsStroke( true );
        aOverlay.Rectangle( box.GetOrigin(), box.GetEnd() );

        if( aNote.kind == "spotlight" )
        {
            BOX2D outer = aViewport;
            outer.Merge( box );
            aOverlay.SetIsStroke( false );
            aOverlay.SetIsFill( true );
            aOverlay.SetFillColor( COLOR4D( 0, 0, 0, 0.55 * aAlpha ) );
            aOverlay.Rectangle( outer.GetOrigin(), VECTOR2D( outer.GetRight(), box.GetTop() ) );
            aOverlay.Rectangle( VECTOR2D( outer.GetLeft(), box.GetBottom() ), outer.GetEnd() );
            aOverlay.Rectangle( VECTOR2D( outer.GetLeft(), box.GetTop() ), VECTOR2D( box.GetLeft(), box.GetBottom() ) );
            aOverlay.Rectangle( VECTOR2D( box.GetRight(), box.GetTop() ), VECTOR2D( outer.GetRight(), box.GetBottom() ) );
        }
    }
    else if( aNote.kind == "step" && target )
    {
        const VECTOR2D at( target->GetLeft() - 4 * aPx, target->GetTop() - 4 * aPx );
        const double   r = 13 * aPx;

        if( !text.IsEmpty() )
            drawLabel( aOverlay, aPx, at + VECTOR2D( r + 8 * aPx, 0 ), text, colour, aAlpha );

        aOverlay.SetIsFill( false );
        aOverlay.SetIsStroke( true );
        aOverlay.SetStrokeColor( COLOR4D( 1, 1, 1, aAlpha ) );
        aOverlay.SetLineWidth( 2.6 * aPx );
        aOverlay.SetGlyphSize( VECTOR2I( KiROUND( 15 * aPx ), KiROUND( 15 * aPx ) ) );
        aOverlay.SetTextJustify( GR_TEXT_H_ALIGN_CENTER, GR_TEXT_V_ALIGN_CENTER );
        aOverlay.StrokeText( wxString::Format( "%d", aNote.number ), VECTOR2I( KiROUND( at.x ), KiROUND( at.y ) ), ANGLE_0 );

        aOverlay.SetIsFill( true );
        aOverlay.SetIsStroke( false );
        aOverlay.SetFillColor( colour.WithAlpha( aAlpha ) );
        aOverlay.Circle( at, r );
    }
    else if( aNote.kind == "key_hint" )
    {
        drawLabel( aOverlay, aPx, VECTOR2D( aViewport.GetLeft() + 16 * aPx, aViewport.GetBottom() - 30 * aPx ), text,
                   colour, aAlpha );
    }
}


void LAYER::drawPointer( KIGFX::VIEW_OVERLAY& aOverlay, double aPx, const BOX2D& aViewport, CLOCK::time_point aNow )
{
    POINTER& p = m_pointer;

    while( !p.trail.empty() && seconds( aNow - p.trail.front().first ) > TRAIL_S )
        p.trail.pop_front();

    while( !p.ripples.empty() && seconds( aNow - p.ripples.front().start ) > RIPPLE_S )
        p.ripples.pop_front();

    const double idle = seconds( aNow - p.lastInput );

    if( !p.shown || idle > POINTER_IDLE_S + 1.0 )
        return;

    const double alpha = idle > POINTER_IDLE_S ? std::max( 0.0, 1.0 - ( idle - POINTER_IDLE_S ) ) : 1.0;
    const COLOR4D accent( 0.10, 0.65, 1.0, 1.0 );

    // the last key, at the bottom left of the canvas
    if( !p.key.IsEmpty() && seconds( aNow - p.keyTime ) < KEY_S )
    {
        const double fade = std::min( 1.0, ( KEY_S - seconds( aNow - p.keyTime ) ) / 0.3 );
        drawLabel( aOverlay, aPx, VECTOR2D( aViewport.GetLeft() + 16 * aPx, aViewport.GetBottom() - 70 * aPx ),
                   wxS( "key  " ) + p.key, accent, fade );
    }

    // the cursor: an arrow, tip at the position (screen-size constant)
    static const VECTOR2D shape[] = { { 0, 0 }, { 0, 20 }, { 5, 15.5 }, { 9, 24 }, { 12.5, 22.5 }, { 8.5, 14 }, { 15, 14 } };
    std::deque<VECTOR2D> points;

    for( const VECTOR2D& s : shape )
        points.push_back( p.position + s * aPx );

    aOverlay.SetIsFill( true );
    aOverlay.SetIsStroke( true );
    aOverlay.SetFillColor( p.held ? accent.WithAlpha( alpha ) : COLOR4D( 1, 1, 1, alpha ) );
    aOverlay.SetStrokeColor( COLOR4D( 0, 0, 0, alpha ) );
    aOverlay.SetLineWidth( 1.5 * aPx );
    aOverlay.Polygon( points );

    // drawn after the cursor: the GAL keeps the first of equally deep shapes
    // click ripples
    aOverlay.SetIsFill( false );
    aOverlay.SetIsStroke( true );

    for( const POINTER::RIPPLE& ripple : p.ripples )
    {
        const double t = seconds( aNow - ripple.start ) / RIPPLE_S;

        for( int ring = 0; ring < ripple.rings; ++ring )
        {
            const double tr = std::clamp( t - ring * 0.25, 0.0, 1.0 );
            aOverlay.SetLineWidth( 3 * aPx );
            aOverlay.SetStrokeColor( accent.WithAlpha( ( 1.0 - tr ) * alpha ) );
            aOverlay.Circle( ripple.at, ( 6 + 26 * tr ) * aPx );
        }
    }

    // trail: fading segments
    aOverlay.SetIsFill( false );
    aOverlay.SetIsStroke( true );

    for( size_t i = 1; i < p.trail.size(); ++i )
    {
        const double age = seconds( aNow - p.trail[i].first ) / TRAIL_S;
        aOverlay.SetStrokeColor( accent.WithAlpha( 0.6 * ( 1.0 - age ) * alpha ) );
        aOverlay.Segment( p.trail[i - 1].second, p.trail[i].second, 4 * aPx );
    }

}


void LAYER::Redraw()
{
    if( !Alive() )
    {
        m_timer.Stop();
        return;
    }

    KIGFX::VIEW*    view = m_frame->GetCanvas()->GetView();
    const CLOCK::time_point now = CLOCK::now();
    const double    px = view->ToWorld( 1.0 );
    const BOX2D     viewport = view->GetViewport();

    ensureOverlay( view );
    m_overlay->Clear();

    bool animating = false;
    bool following = false;

    // the GAL keeps the first of equally deep shapes, so what is on top is drawn first: the
    // pointer, then by kind (texts over arrows over marked areas), the newest first within a kind
    drawPointer( *m_overlay, px, viewport, now );

    std::erase_if( m_annotations, [&]( const ANNOTATION& n ) { return n.ttl > 0 && seconds( now - n.created ) >= n.ttl; } );

    std::vector<ANNOTATION*> order;

    for( auto it = m_annotations.rbegin(); it != m_annotations.rend(); ++it )
        order.push_back( &*it );

    std::stable_sort( order.begin(), order.end(),
                      []( const ANNOTATION* a, const ANNOTATION* b ) { return zBand( a->kind ) < zBand( b->kind ); } );

    for( ANNOTATION* notePtr : order )
    {
        ANNOTATION& note = *notePtr;
        double      alpha = 1.0;

        if( note.ttl > 0 )
        {
            alpha = std::min( 1.0, ( note.ttl - seconds( now - note.created ) ) / 0.4 );
            animating = true;
        }

        // one failing annotation never stops the others
        try
        {
            drawAnnotation( *m_overlay, px, viewport, note, alpha, now );
        }
        catch( ... )
        {
            note.orphaned = true;
        }

        animating |= note.pulse;
        following |= note.target.isItem || note.to.isItem || !note.attach.empty();
    }

    std::erase_if( m_annotations, []( const ANNOTATION& n ) { return n.orphaned && n.onOrphan == "remove"; } );

    const double pointerAge = seconds( now - m_pointer.lastInput );
    animating |= m_pointer.shown && pointerAge < POINTER_IDLE_S + 1.2;
    animating |= !m_pointer.ripples.empty() || !m_pointer.trail.empty();

    view->Update( m_overlay.get() );
    m_frame->GetCanvas()->Refresh();

    const int tick = animating ? FAST_TICK_MS : following ? FOLLOW_TICK_MS : 0;

    if( !tick )
        m_timer.Stop();
    else if( !m_timer.IsRunning() || m_timer.GetInterval() != tick )
        m_timer.Start( tick );
}


/// Process-lifetime registry (the timers belong to their layers; layers of closed editors are
/// dropped on the next access)
std::map<EDA_DRAW_FRAME*, std::unique_ptr<LAYER>>& registry()
{
    static auto* s_layers = new std::map<EDA_DRAW_FRAME*, std::unique_ptr<LAYER>>();
    return *s_layers;
}


LAYER* layerFor( EDA_DRAW_FRAME* aFrame )
{
    auto& layers = registry();

    for( auto it = layers.begin(); it != layers.end(); )
    {
        if( !it->second->Alive() )
            it = layers.erase( it );
        else
            ++it;
    }

    if( !aFrame || !aFrame->GetCanvas() )
        return nullptr;

    std::unique_ptr<LAYER>& layer = layers[aFrame];

    if( !layer )
        layer = std::make_unique<LAYER>( aFrame );

    return layer.get();
}


std::string parseAnchor( const KOPENAPI_ANNOTATION_HOST& aHost, const nlohmann::json& aValue, ANCHOR& aAnchor )
{
    if( aValue.is_string() )
    {
        aAnchor.set = true;
        aAnchor.isItem = true;
        aAnchor.ref = aValue.get<std::string>();

        if( !aHost.resolve || !aHost.resolve( aAnchor.ref ) )
            return "no item '" + aAnchor.ref + "' (a reference or a uuid on the shown sheet / board)";

        return {};
    }

    if( aValue.is_array() && aValue.size() == 2 && aValue[0].is_number() && aValue[1].is_number() )
    {
        aAnchor.set = true;
        aAnchor.isItem = false;
        aAnchor.point = VECTOR2D( aHost.scale->mmToIU( aValue[0].get<double>() ),
                                  aHost.scale->mmToIU( aValue[1].get<double>() ) );
        return {};
    }

    return "an anchor is a reference / uuid or [x, y] mm";
}


/// @brief Points given now are where they are relative to the item as it is now
void attachBaseline( const KOPENAPI_ANNOTATION_HOST& aHost, ANNOTATION& aNote, const BOX2I& aItemBox )
{
    aNote.attachOrigin = VECTOR2D( aItemBox.Centre() );
    aNote.attachCentre = aNote.attachOrigin;
    aNote.attachShift = VECTOR2D( 0, 0 );
    aNote.attachTurn = { 1, 0, 0, 1 };
    aNote.attachLinear = { 1, 0, 0, 1 };

    if( aHost.linear )
    {
        if( std::optional<std::array<double, 4>> m = aHost.linear( aNote.attach ) )
            aNote.attachLinear = *m;
    }
}


std::string applyFields( const KOPENAPI_ANNOTATION_HOST& aHost, const nlohmann::json& aSpec, ANNOTATION& aNote )
{
    static const std::vector<std::string> kinds = { "highlight", "arrow", "label", "region", "spotlight", "step",
                                                    "key_hint" };

    if( aSpec.contains( "kind" ) )
    {
        aNote.kind = aSpec.value( "kind", std::string() );

        if( std::find( kinds.begin(), kinds.end(), aNote.kind ) == kinds.end() )
            return "kind is one of highlight, arrow, label, region, spotlight, step, key_hint";
    }

    if( aSpec.contains( "target" ) )
    {
        if( std::string bad = parseAnchor( aHost, aSpec["target"], aNote.target ); !bad.empty() )
            return bad;
    }

    if( aSpec.contains( "to" ) )
    {
        if( std::string bad = parseAnchor( aHost, aSpec["to"], aNote.to ); !bad.empty() )
            return bad;
    }

    if( aSpec.contains( "area" ) )
    {
        const nlohmann::json& a = aSpec["area"];

        if( !a.is_array() || a.size() != 4 )
            return "area is [x0, y0, x1, y1] mm";

        const VECTOR2D p0( aHost.scale->mmToIU( a[0].get<double>() ), aHost.scale->mmToIU( a[1].get<double>() ) );
        const VECTOR2D p1( aHost.scale->mmToIU( a[2].get<double>() ), aHost.scale->mmToIU( a[3].get<double>() ) );
        BOX2D          box( p0, p1 - p0 );
        box.Normalize();
        aNote.area = box;
    }

    if( aSpec.contains( "attach" ) )
    {
        const std::string ref = aSpec["attach"].is_string() ? aSpec["attach"].get<std::string>() : std::string();

        // detached or attached elsewhere: what the old item moved stays where it is now
        if( !aNote.attach.empty() && !aSpec.contains( "target" ) && !aSpec.contains( "to" ) && !aSpec.contains( "area" ) )
        {
            if( aNote.target.set && !aNote.target.isItem )
                aNote.target.point = aNote.Attached( aNote.target.point );

            if( aNote.to.set && !aNote.to.isItem )
                aNote.to.point = aNote.Attached( aNote.to.point );

            if( aNote.area )
            {
                BOX2D turned( aNote.Attached( aNote.area->GetOrigin() ), VECTOR2D( 0, 0 ) );

                for( const VECTOR2D& c : { VECTOR2D( aNote.area->GetRight(), aNote.area->GetTop() ),
                                           VECTOR2D( aNote.area->GetLeft(), aNote.area->GetBottom() ), aNote.area->GetEnd() } )
                {
                    turned.Merge( aNote.Attached( c ) );
                }

                aNote.area = turned;
            }

            aNote.attachShift = VECTOR2D( 0, 0 );
        }

        if( ref.empty() )
        {
            aNote.attach.clear();
        }
        else
        {
            std::optional<BOX2I> box = aHost.resolve ? aHost.resolve( ref ) : std::nullopt;

            if( !box )
                return "attach: no item '" + ref + "' (a reference or a uuid on the shown sheet / board)";

            aNote.attach = ref;
            attachBaseline( aHost, aNote, *box );
        }
    }

    // new absolute points of an attached annotation: they keep their offset from where the item
    // is now
    if( !aSpec.contains( "attach" ) && !aNote.attach.empty()
        && ( aSpec.contains( "target" ) || aSpec.contains( "to" ) || aSpec.contains( "area" ) ) )
    {
        if( std::optional<BOX2I> box = aHost.resolve ? aHost.resolve( aNote.attach ) : std::nullopt )
            attachBaseline( aHost, aNote, *box );
    }

    if( aSpec.contains( "text" ) )
        aNote.text = aSpec.value( "text", std::string() ).substr( 0, MAX_TEXT );

    aNote.group = aSpec.value( "group", aNote.group );
    aNote.onOrphan = aSpec.value( "on_orphan", aNote.onOrphan );
    aNote.colour = parseColour( aSpec.value( "color", nlohmann::json() ), aNote.colour );
    aNote.widthPx = std::clamp( aSpec.value( "width_px", aNote.widthPx ), 0.5, 20.0 );
    aNote.pulse = aSpec.value( "pulse", aNote.pulse );
    aNote.number = aSpec.value( "number", aNote.number );

    if( aSpec.contains( "ttl_s" ) )
    {
        aNote.ttl = std::clamp( aSpec.value( "ttl_s", 0.0 ), 0.0, 3600.0 );
        aNote.created = CLOCK::now();
    }

    if( aNote.kind == "arrow" && !( aNote.target.set && aNote.to.set ) )
        return "an arrow needs target and to";

    if( ( aNote.kind == "highlight" || aNote.kind == "label" || aNote.kind == "step" ) && !aNote.target.set )
        return aNote.kind + " needs a target";

    if( ( aNote.kind == "region" || aNote.kind == "spotlight" ) && !aNote.area && !aNote.target.set )
        return aNote.kind + " needs an area or a target";

    return {};
}


nlohmann::json describe( const KOPENAPI_ANNOTATION_HOST& aHost, const ANNOTATION& aNote )
{
    auto box = [&]( const std::optional<BOX2D>& aBox ) -> nlohmann::json
    {
        if( !aBox )
            return nullptr;

        return { aHost.scale->IUTomm( aBox->GetLeft() ), aHost.scale->IUTomm( aBox->GetTop() ),
                 aHost.scale->IUTomm( aBox->GetRight() ), aHost.scale->IUTomm( aBox->GetBottom() ) };
    };

    return { { "id", aNote.id },
             { "kind", aNote.kind },
             { "text", aNote.text },
             { "group", aNote.group },
             { "orphaned", aNote.orphaned },
             { "attach", aNote.attach.empty() ? nlohmann::json() : nlohmann::json( aNote.attach ) },
             { "target_box_mm", box( aNote.lastTarget ) },
             { "to_box_mm", box( aNote.lastTo ) } };
}
} // namespace


KOPENAPI_RESULT KopenapiAnnotate( const KOPENAPI_ANNOTATION_HOST& aHost, const nlohmann::json& aArgs )
{
    LAYER* layer = layerFor( aHost.frame );

    if( !layer || !aHost.frame->IsShown() )
        return KOPENAPI_RESULT::Error( 409, "no editor window open" );

    layer->SetHost( aHost );

    std::vector<ANNOTATION>& notes = layer->Annotations();
    static int               s_nextId = 1;
    nlohmann::json           added = nlohmann::json::array();

    if( aArgs.contains( "clear" ) )
    {
        const std::string group = aArgs.value( "clear", std::string( "*" ) );
        std::erase_if( notes, [&]( const ANNOTATION& n ) { return group == "*" || n.group == group; } );
    }

    for( const nlohmann::json& id : aArgs.value( "remove", nlohmann::json::array() ) )
    {
        const std::string want = id.is_string() ? id.get<std::string>() : std::string();
        std::erase_if( notes, [&]( const ANNOTATION& n ) { return n.id == want; } );
    }

    for( const nlohmann::json& spec : aArgs.value( "update", nlohmann::json::array() ) )
    {
        const std::string id = spec.value( "id", std::string() );
        auto it = std::find_if( notes.begin(), notes.end(), [&]( const ANNOTATION& n ) { return n.id == id; } );

        if( it == notes.end() )
            return KOPENAPI_RESULT::Error( 404, "no annotation '" + id + "'" );

        if( std::string bad = applyFields( aHost, spec, *it ); !bad.empty() )
            return KOPENAPI_RESULT::Error( 400, "update " + id + ": " + bad );
    }

    for( const nlohmann::json& spec : aArgs.value( "add", nlohmann::json::array() ) )
    {
        if( notes.size() >= MAX_ANNOTATIONS )
            return KOPENAPI_RESULT::Error( 409, "at most 500 annotations per editor" );

        ANNOTATION note;
        note.kind = "highlight";

        if( std::string bad = applyFields( aHost, spec, note ); !bad.empty() )
            return KOPENAPI_RESULT::Error( 400, "add: " + bad );

        note.id = spec.value( "id", "a" + std::to_string( s_nextId++ ) );
        std::erase_if( notes, [&]( const ANNOTATION& n ) { return n.id == note.id; } );
        notes.push_back( note );
        added.push_back( note.id );
    }

    layer->Redraw();

    nlohmann::json list = nlohmann::json::array();

    for( const ANNOTATION& note : layer->Annotations() )
        list.push_back( describe( aHost, note ) );

    return KOPENAPI_RESULT::Ok( { { "added", added }, { "annotations", list } } );
}


void KopenapiAnnotatePointer( EDA_DRAW_FRAME* aFrame, const VECTOR2D& aWorld, KOPENAPI_POINTER_EVENT aEvent,
                              const wxString& aKey )
{
    static const bool enabled = []()
    {
        wxString value;
        return !( wxGetEnv( wxS( "KOPENAPI_SHOW_POINTER" ), &value ) && value == wxS( "0" ) );
    }();

    LAYER* layer = enabled ? layerFor( aFrame ) : nullptr;

    if( !layer )
        return;

    POINTER&                p = layer->Pointer();
    const CLOCK::time_point now = CLOCK::now();

    p.lastInput = now;

    if( aEvent == KOPENAPI_POINTER_EVENT::KEY )
    {
        p.key = aKey;
        p.keyTime = now;
    }
    else
    {
        if( !p.shown || p.position != aWorld )
            p.trail.emplace_back( now, aWorld );

        p.shown = true;
        p.position = aWorld;

        if( aEvent == KOPENAPI_POINTER_EVENT::DOWN )
        {
            p.held = true;
            p.ripples.push_back( { now, aWorld, 1 } );
        }
        else if( aEvent == KOPENAPI_POINTER_EVENT::UP )
        {
            p.held = false;
        }
        else if( aEvent == KOPENAPI_POINTER_EVENT::DCLICK )
        {
            p.ripples.push_back( { now, aWorld, 2 } );
        }
    }

    layer->Redraw();
}


const nlohmann::json& KopenapiAnnotateSchema()
{
    static const nlohmann::json schema = R"json({"type":"object","properties":{
        "add":{"type":"array","items":{"type":"object","properties":{
            "id":{"type":"string","description":"optional; reusing an id replaces that annotation"},
            "kind":{"type":"string","enum":["highlight","arrow","label","region","spotlight","step","key_hint"],"default":"highlight"},
            "target":{"description":"anchor: a reference (U1, R5), a uuid, or [x, y] mm; items are followed when they move"},
            "to":{"description":"arrow end: an anchor"},
            "area":{"type":"array","items":{"type":"number"},"description":"region / spotlight: [x0, y0, x1, y1] mm"},
            "attach":{"type":"string","description":"a reference or uuid: [x, y] anchors and the area keep their offset from this item and move with it (drags, moves); empty detaches; the item deleted orphans the annotation"},
            "text":{"type":"string"},
            "number":{"type":"integer","description":"step number"},
            "color":{"type":"string","description":"#RRGGBB (default orange)"},
            "width_px":{"type":"number","default":3},
            "pulse":{"type":"boolean","default":false},
            "ttl_s":{"type":"number","description":"fade out after this many seconds; 0 = stays"},
            "group":{"type":"string","description":"for clear"},
            "on_orphan":{"type":"string","enum":["keep","hide","remove"],"default":"keep","description":"when the anchored item is deleted"}}}},
        "update":{"type":"array","items":{"type":"object"},"description":"objects with id and the fields to change"},
        "remove":{"type":"array","items":{"type":"string"}},
        "clear":{"type":"string","description":"a group, or * for all"}}})json"_json;
    return schema;
}
