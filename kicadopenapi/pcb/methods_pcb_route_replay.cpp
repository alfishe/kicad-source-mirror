/// @file methods_pcb_route_replay.cpp
/// @brief pcb_route_replay: the last recorded grid route (route_timeline.h) replayed over the
/// board editor's canvas: every net's copper at a step in its layer's colour, the net being taken
/// bright with the area searched, the places crowded right after the step, the places fought over
/// in the last rounds, and a timeline strip with the crowded places over the run. The board's own
/// copper of those nets is hidden from the view while the replay is shown (the board is unchanged).
/// Keys while shown: space play / pause, left / right a step (shift ten), home / end, up / down the
/// speed, escape closes; dragging on the strip seeks. GUI, main thread.
#include "kopenapi_pcb.h"
#include "route_timeline.h"

#include <kicadopenapi_editor_input.h>
#include <kicadopenapi_registry.h>

#include <base_units.h>
#include <board.h>
#include <class_draw_panel_gal.h>
#include <frame_type.h>
#include <geometry/shape_line_chain.h>
#include <kiway.h>
#include <netclass.h>
#include <netinfo.h>
#include <pcb_edit_frame.h>
#include <pcb_track.h>
#include <settings/color_settings.h>
#include <tool/tool_event.h>
#include <view/view.h>
#include <view/view_overlay.h>

#include <wx/timer.h>
#include <wx/weakref.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <unordered_set>


namespace
{
using CLOCK = std::chrono::steady_clock;

constexpr int    TICK_MS = 16;
constexpr double STRIP_PX = 64;

const COLOR4D CROWDED( 1.0, 0.0, 0.78, 1.0 );
const COLOR4D FOUGHT( 1.0, 0.55, 0.0, 1.0 );


class REPLAY : public wxEvtHandler
{
public:
    REPLAY()
    {
        m_timer.SetOwner( this );
        Bind( wxEVT_TIMER, [this]( wxTimerEvent& ) { tick(); }, m_timer.GetId() );
    }

    ~REPLAY() override { Hide(); }

    bool Shown() const { return m_frame && m_timeline; }

    std::string Show( PCB_EDIT_FRAME* aFrame, std::shared_ptr<GRID_TIMELINE> aTimeline, int aLane )
    {
        if( m_frame && m_frame.get() != aFrame )
            Hide();

        const bool same = m_timeline == aTimeline && m_lane == aLane;

        if( m_timeline != aTimeline )
            restoreBoard();

        m_frame = aFrame;
        m_timeline = std::move( aTimeline );
        m_lane = aLane;

        if( !same )
        {
            m_geometry.clear();
            m_at = (double) lane().steps.size() - 1;
            m_rate = std::max( 5.0, lane().steps.size() / 20.0 );
            m_playing = false;
            buildSeries();
        }

        hideBoardCopper();
        bindInput();
        Redraw();
        return {};
    }

    void Hide()
    {
        m_timer.Stop();
        m_playing = false;
        unbindInput();
        restoreBoard();

        if( m_overlay )
        {
            m_overlay->Clear();

            if( m_overlayView && m_frame && m_frame->GetCanvas() && m_frame->GetCanvas()->GetView() == m_overlayView )
            {
                m_overlayView->Update( m_overlay.get() );
                m_frame->GetCanvas()->Refresh();
            }
            else
            {
                // the view is gone: abandoned, as the annotation layer does
                new std::shared_ptr<KIGFX::VIEW_OVERLAY>( std::move( m_overlay ) );
            }
        }

        m_overlay.reset();
        m_overlayView = nullptr;
        m_timeline.reset();
        m_frame = nullptr;
    }

    void Seek( double aStep )
    {
        m_at = std::clamp( aStep, 0.0, (double) lane().steps.size() - 1 );
        Redraw();
    }

    void Play( bool aPlay )
    {
        if( aPlay && m_at >= lane().steps.size() - 1 )
            m_at = 0;

        m_playing = aPlay;
        m_last = CLOCK::now();

        if( aPlay )
            m_timer.Start( TICK_MS );
        else
            m_timer.Stop();

        Redraw();
    }

    void SetRate( double aRate ) { m_rate = std::clamp( aRate, 0.25, 1e5 ); }

    nlohmann::json Status() const
    {
        if( !Shown() )
            return { { "shown", false } };

        const GRID_LANE& L = lane();
        const GRID_STEP& s = L.steps[step()];
        return { { "shown", true },
                 { "run", m_timeline->run },
                 { "lane", m_lane },
                 { "variant", L.name },
                 { "at", step() },
                 { "steps", L.steps.size() },
                 { "stage", GRID_TIMELINE::STAGES[s.stage] },
                 { "round", s.round },
                 { "net", m_timeline->netNames[s.net] },
                 { "crowded_places", m_series[step()] },
                 { "playing", m_playing },
                 { "rate", m_rate },
                 { "hidden_board_items", m_hidden.size() },
                 { "draw_us", m_drawUs } };
    }

    void Redraw();

private:
    const GRID_LANE& lane() const { return m_timeline->lanes[m_lane]; }
    size_t           step() const { return (size_t) std::lround( std::floor( m_at ) ); }

    void tick()
    {
        if( !Shown() || !m_frame->GetCanvas() )
        {
            Hide();
            return;
        }

        const CLOCK::time_point now = CLOCK::now();
        const double            dt = std::chrono::duration<double>( now - m_last ).count();
        m_last = now;
        const size_t before = step();
        m_at = std::min( m_at + m_rate * dt, (double) lane().steps.size() - 1 );

        if( m_at >= lane().steps.size() - 1 )
        {
            m_playing = false;
            m_timer.Stop();
        }

        if( step() != before || !m_playing )
            Redraw();
    }

    void buildSeries()
    {
        const GRID_LANE& L = lane();
        m_series.assign( L.steps.size(), 0 );
        m_top = 1;

        for( size_t k = 0; k < L.steps.size(); ++k )
        {
            m_series[k] = L.conflictSets.empty() ? 0 : (int) L.conflictSets[L.steps[k].conflicts].size();
            m_top = std::max( m_top, m_series[k] );
        }
    }

    void hideBoardCopper()
    {
        BOARD*        board = m_frame->GetBoard();
        KIGFX::VIEW*  view = m_frame->GetCanvas()->GetView();
        std::unordered_set<int> codes( m_timeline->netCodes.begin(), m_timeline->netCodes.end() );

        for( PCB_TRACK* track : board->Tracks() )
        {
            if( codes.count( track->GetNetCode() ) && !m_hidden.count( track ) )
            {
                view->Hide( track, true );
                m_hidden.insert( track );
            }
        }

        m_board = board;
    }

    void restoreBoard()
    {
        if( m_frame && m_board && m_frame->GetBoard() == m_board && m_frame->GetCanvas() )
        {
            KIGFX::VIEW* view = m_frame->GetCanvas()->GetView();

            // only what is still on the board
            for( PCB_TRACK* track : m_board->Tracks() )
            {
                if( m_hidden.count( track ) )
                    view->Hide( track, false );
            }

            m_frame->GetCanvas()->Refresh();
        }

        m_hidden.clear();
        m_board = nullptr;
    }

    void bindInput()
    {
        if( m_bound == m_frame.get() )
            return;

        unbindInput();
        m_frame->Bind( wxEVT_CHAR_HOOK, &REPLAY::onKey, this );
        KopenapiSetKeyHandler( m_frame.get(), [this]( int aKey, int aMods ) { return key( aKey, aMods & MD_SHIFT ); } );
        m_canvas = m_frame->GetCanvas();
        m_canvas->Bind( wxEVT_LEFT_DOWN, &REPLAY::onMouse, this );
        m_canvas->Bind( wxEVT_MOTION, &REPLAY::onMouse, this );
        m_canvas->Bind( wxEVT_LEFT_UP, &REPLAY::onMouse, this );
        m_bound = m_frame.get();
    }

    void unbindInput()
    {
        if( m_bound && m_frame && m_frame.get() == m_bound )
        {
            m_bound->Unbind( wxEVT_CHAR_HOOK, &REPLAY::onKey, this );
            KopenapiSetKeyHandler( m_bound, nullptr );

            if( m_canvas && m_frame->GetCanvas() == m_canvas )
            {
                m_canvas->Unbind( wxEVT_LEFT_DOWN, &REPLAY::onMouse, this );
                m_canvas->Unbind( wxEVT_MOTION, &REPLAY::onMouse, this );
                m_canvas->Unbind( wxEVT_LEFT_UP, &REPLAY::onMouse, this );
            }
        }

        m_bound = nullptr;
        m_canvas = nullptr;
        m_dragging = false;
    }

    void onKey( wxKeyEvent& aEvent )
    {
        if( !key( aEvent.GetKeyCode(), aEvent.ShiftDown() ) )
            aEvent.Skip();
    }

    /// @brief A key from the keyboard or injected: true when it is the replay's
    bool key( int aKeyCode, bool aShift )
    {
        if( !Shown() )
            return false;

        const int n = (int) lane().steps.size();
        const int by = aShift ? 10 : 1;

        switch( aKeyCode )
        {
        case WXK_SPACE: Play( !m_playing ); return true;
        case WXK_LEFT: Play( false ); Seek( (double) step() - by ); return true;
        case WXK_RIGHT: Play( false ); Seek( (double) step() + by ); return true;
        case WXK_HOME: Seek( 0 ); return true;
        case WXK_END: Seek( n - 1 ); return true;
        case WXK_UP: SetRate( m_rate * 2 ); Redraw(); return true;
        case WXK_DOWN: SetRate( m_rate / 2 ); Redraw(); return true;
        case WXK_ESCAPE: CallAfter( [this]() { Hide(); } ); return true;   // not inside the handler being removed
        default: return false;
        }
    }

    /// @brief The strip's step under a canvas point, or -1 outside the strip
    double stripStep( const wxPoint& aScreen ) const
    {
        const wxSize size = m_canvas->GetClientSize();

        if( aScreen.y < size.y - STRIP_PX )
            return -1;

        const double f = std::clamp( ( aScreen.x - 10.0 ) / std::max( 1.0, size.x - 20.0 ), 0.0, 1.0 );
        return f * ( lane().steps.size() - 1 );
    }

    void onMouse( wxMouseEvent& aEvent )
    {
        if( !Shown() || !m_canvas )
        {
            aEvent.Skip();
            return;
        }

        const double at = stripStep( aEvent.GetPosition() );

        if( aEvent.LeftDown() && at >= 0 )
        {
            m_dragging = true;
            Play( false );
            Seek( at );
            return;
        }

        if( m_dragging && aEvent.Dragging() )
        {
            Seek( std::max( 0.0, at < 0 ? m_at : at ) );
            return;
        }

        if( m_dragging && aEvent.LeftUp() )
        {
            m_dragging = false;
            return;
        }

        aEvent.Skip();
    }

    const GRID_NET_RESULT& geometry( uint32_t aPath )
    {
        auto it = m_geometry.find( aPath );

        if( it == m_geometry.end() )
            it = m_geometry.emplace( aPath, m_timeline->Geometry( lane().paths[aPath] ) ).first;

        return it->second;
    }

    void netSizes( int aNet, double& aWidth, double& aVia ) const
    {
        aWidth = pcbIUScale.mmToIU( 0.25 );
        aVia = pcbIUScale.mmToIU( 0.6 );
        const NETINFO_ITEM* net = m_board ? m_board->FindNet( m_timeline->netCodes[aNet] ) : nullptr;

        if( net && net->GetNetClass() )
        {
            if( net->GetNetClass()->GetTrackWidth() > 0 )
                aWidth = net->GetNetClass()->GetTrackWidth();

            if( net->GetNetClass()->GetViaDiameter() > 0 )
                aVia = net->GetNetClass()->GetViaDiameter();
        }
    }

    void drawStrip( KIGFX::VIEW_OVERLAY& aOv, double aPx, const BOX2D& aVp );

    wxWeakRef<PCB_EDIT_FRAME>                        m_frame;
    std::shared_ptr<GRID_TIMELINE>                   m_timeline;
    int                                              m_lane = 0;
    double                                           m_at = 0;
    double                                           m_rate = 10;
    bool                                             m_playing = false;
    CLOCK::time_point                                m_last;
    wxTimer                                          m_timer;
    std::vector<int>                                 m_series;
    int                                              m_top = 1;
    std::unordered_map<uint32_t, GRID_NET_RESULT>    m_geometry;
    std::vector<uint32_t>                            m_table;
    BOARD*                                           m_board = nullptr;
    std::unordered_set<const BOARD_ITEM*>            m_hidden;
    std::shared_ptr<KIGFX::VIEW_OVERLAY>             m_overlay;
    KIGFX::VIEW*                                     m_overlayView = nullptr;
    PCB_EDIT_FRAME*                                  m_bound = nullptr;
    EDA_DRAW_PANEL_GAL*                              m_canvas = nullptr;
    bool                                             m_dragging = false;
    long                                             m_drawUs = 0;
};


void REPLAY::drawStrip( KIGFX::VIEW_OVERLAY& aOv, double aPx, const BOX2D& aVp )
{
    const GRID_LANE& L = lane();
    const GRID_STEP& s = L.steps[step()];
    const size_t     n = L.steps.size();
    const double     x0 = aVp.GetLeft() + 10 * aPx, x1 = aVp.GetRight() - 10 * aPx;
    const double     yBottom = aVp.GetBottom(), yTop = yBottom - STRIP_PX * aPx;
    const double     plotTop = yTop + 22 * aPx, plotBottom = yBottom - 4 * aPx;

    auto xOf = [&]( double k ) { return x0 + ( x1 - x0 ) * k / std::max<size_t>( 1, n - 1 ); };

    const GRID_TIMELINE& t = *m_timeline;
    wxString text = wxString::Format( wxS( "%s   %s   round %d   step %zu / %zu   %s   crowded places %d   %.1f s   "
                                           "%s %.0f steps/s" ),
                                      wxString::FromUTF8( L.name ), GRID_TIMELINE::STAGES[s.stage], (int) s.round,
                                      step() + 1, n, wxString::FromUTF8( t.netNames[s.net] ), m_series[step()],
                                      s.ms / 1000.0, m_playing ? wxS( "playing" ) : wxS( "paused" ), m_rate );

    // the GAL keeps the first of equally deep shapes: the top first
    const double glyph = 13 * aPx;
    aOv.SetIsFill( false );
    aOv.SetIsStroke( true );
    aOv.SetStrokeColor( COLOR4D( 1, 1, 1, 1 ) );
    aOv.SetLineWidth( glyph * 0.15 );
    aOv.SetGlyphSize( VECTOR2I( KiROUND( glyph ), KiROUND( glyph ) ) );
    aOv.SetTextJustify( GR_TEXT_H_ALIGN_LEFT, GR_TEXT_V_ALIGN_CENTER );
    aOv.StrokeText( text, VECTOR2I( KiROUND( x0 ), KiROUND( yTop + 11 * aPx ) ), ANGLE_0 );

    aOv.SetStrokeColor( COLOR4D( 1, 1, 1, 1 ) );
    aOv.Segment( VECTOR2D( xOf( (double) step() ), yTop + 20 * aPx ), VECTOR2D( xOf( (double) step() ), yBottom ),
                 2 * aPx );

    // crowded places over the run, one value per pixel column (the most in it)
    const int cols = std::max( 2, (int) ( ( x1 - x0 ) / aPx ) );
    std::vector<int> peak( cols, 0 );

    for( size_t k = 0; k < n; ++k )
    {
        int& c = peak[std::min<int>( cols - 1, (int) ( (double) k / std::max<size_t>( 1, n - 1 ) * ( cols - 1 ) ) )];
        c = std::max( c, m_series[k] );
    }

    SHAPE_LINE_CHAIN line;

    for( int c = 0; c < cols; ++c )
    {
        line.Append( VECTOR2I( KiROUND( x0 + c * ( x1 - x0 ) / ( cols - 1 ) ),
                               KiROUND( plotBottom - ( plotBottom - plotTop ) * peak[c] / m_top ) ) );
    }

    aOv.SetStrokeColor( CROWDED );
    aOv.SetLineWidth( 2 * aPx );
    aOv.Polyline( line );

    aOv.SetStrokeColor( COLOR4D( 0.35, 0.35, 0.45, 1 ) );
    aOv.SetLineWidth( aPx );

    for( size_t k = 1; k < n; ++k )
    {
        if( L.steps[k].stage != L.steps[k - 1].stage )
            aOv.Line( VECTOR2D( xOf( (double) k ), plotTop ), VECTOR2D( xOf( (double) k ), plotBottom ) );
    }

    aOv.SetIsFill( true );
    aOv.SetIsStroke( false );
    aOv.SetFillColor( COLOR4D( 0.02, 0.02, 0.05, 0.92 ) );
    aOv.Rectangle( VECTOR2D( aVp.GetLeft(), yTop ), VECTOR2D( aVp.GetRight(), yBottom ) );
}


void REPLAY::Redraw()
{
    if( !Shown() || !m_frame->GetCanvas() )
        return;

    const CLOCK::time_point started = CLOCK::now();
    KIGFX::VIEW*            view = m_frame->GetCanvas()->GetView();

    if( !m_overlay || m_overlayView != view )
    {
        m_overlay = view->MakeOverlay();
        m_overlayView = view;
    }

    KIGFX::VIEW_OVERLAY& ov = *m_overlay;
    ov.Clear();

    const double          px = view->ToWorld( 1.0 );
    const BOX2D           vp = view->GetViewport();
    const GRID_TIMELINE&  t = *m_timeline;
    const GRID_LANE&      L = lane();
    const size_t          at = step();
    const GRID_STEP&      s = L.steps[at];
    COLOR_SETTINGS*       colours = m_frame->GetColorSettings();

    drawStrip( ov, px, vp );

    L.TableAt( at, m_table );

    auto drawNet = [&]( int aNet, bool aHot )
    {
        if( m_table[aNet] == GRID_STEP::NONE )
            return;

        double width, via;
        netSizes( aNet, width, via );
        const GRID_NET_RESULT& g = geometry( m_table[aNet] );

        ov.SetIsFill( false );
        ov.SetIsStroke( true );

        for( const GRID_TRACK& run : g.tracks )
        {
            ov.SetStrokeColor( aHot ? COLOR4D( 1, 1, 1, 1 ) : colours->GetColor( run.layer ).WithAlpha( 0.9 ) );

            for( size_t i = 1; i < run.points.size(); ++i )
                ov.Segment( run.points[i - 1], run.points[i], aHot ? width * 1.6 : width );
        }

        ov.SetIsFill( true );
        ov.SetFillColor( aHot ? COLOR4D( 1, 1, 1, 1 ) : colours->GetColor( LAYER_VIA_THROUGH ).WithAlpha( 0.95 ) );
        ov.SetIsStroke( false );

        for( const VECTOR2I& v : g.vias )
            ov.Circle( v, via / 2 );
    };

    // the net being taken, the area searched, then every other net
    drawNet( s.net, true );

    if( s.box[0] >= 0 )
    {
        ov.SetIsFill( false );
        ov.SetIsStroke( true );
        ov.SetStrokeColor( COLOR4D( 1.0, 1.0, 0.45, 0.9 ) );
        ov.SetLineWidth( 2 * px );
        ov.Rectangle( t.grid.Centre( s.box[0], s.box[1] ), t.grid.Centre( s.box[2], s.box[3] ) );
    }

    for( int n = 0; n < (int) m_table.size(); ++n )
    {
        if( n != s.net )
            drawNet( n, false );
    }

    // crowded right after the step, then the last rounds' fights (none once the rounds are over)
    const double half = t.tile * t.grid.cell / 2.0;

    auto tileBox = [&]( int aTile, const COLOR4D& aFill, bool aOutline )
    {
        const VECTOR2D c = t.TileCentre( aTile );
        ov.SetIsFill( true );
        ov.SetFillColor( aFill );
        ov.SetIsStroke( aOutline );
        ov.SetStrokeColor( CROWDED.WithAlpha( 0.9 ) );
        ov.SetLineWidth( 1.5 * px );
        ov.Rectangle( c - VECTOR2D( half, half ), c + VECTOR2D( half, half ) );
    };

    if( !L.conflictSets.empty() )
    {
        for( uint32_t v : L.conflictSets[s.conflicts] )
            tileBox( (int) ( v >> 8 ), CROWDED.WithAlpha( 0.2 + 0.4 * std::min( 1.0, ( v & 0xff ) / 40.0 ) ), true );
    }

    const int round = t.HeatRound( L, at );

    if( s.stage == 0 && round > 0 )
    {
        static const double fade[] = { 0.28, 0.14, 0.06 };
        const std::vector<uint8_t> age = t.FightAge( L, round );

        for( size_t i = 0; i < age.size(); ++i )
        {
            if( age[i] < 3 )
                tileBox( (int) i, FOUGHT.WithAlpha( fade[age[i]] ), false );
        }
    }

    view->Update( m_overlay.get() );
    m_frame->GetCanvas()->Refresh();
    m_drawUs = (long) std::chrono::duration_cast<std::chrono::microseconds>( CLOCK::now() - started ).count();
}


REPLAY& replay()
{
    static REPLAY* s_replay = new REPLAY();   // bound to frames and timers: lives as long as the process
    return *s_replay;
}


KOPENAPI_RESULT h_pcb_route_replay( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string verb = aArgs.value( "verb", "show" );
    REPLAY&           r = replay();

    if( verb == "status" )
        return KOPENAPI_RESULT::Ok( r.Status() );

    if( verb == "hide" )
    {
        r.Hide();
        return KOPENAPI_RESULT::Ok( r.Status() );
    }

    auto* frame = aCtx.kiway ? dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;

    if( !frame || !frame->GetBoard() || !frame->GetCanvas() )
        return KOPENAPI_RESULT::Error( 409, "no board editor window open" );

    if( verb == "show" || !r.Shown() )
    {
        std::shared_ptr<GRID_TIMELINE> t = GridTimelineLast();

        if( !t || t->lanes.empty() )
            return KOPENAPI_RESULT::Error( 404, "no recorded grid route in this process (pcb_route {engine: 'grid', record: true})" );

        int lane = t->winner >= 0 ? t->winner : 0;

        if( aArgs.contains( "lane" ) )
        {
            const nlohmann::json& l = aArgs["lane"];
            lane = -1;

            for( size_t i = 0; i < t->lanes.size(); ++i )
            {
                if( ( l.is_number_integer() && l.get<int>() == (int) i ) || ( l.is_string() && l.get<std::string>() == t->lanes[i].name ) )
                    lane = (int) i;
            }

            if( lane < 0 )
                return KOPENAPI_RESULT::Error( 404, "no lane " + l.dump() );
        }

        if( t->lanes[lane].steps.empty() )
            return KOPENAPI_RESULT::Error( 404, "the lane has no steps" );

        r.Show( frame, t, lane );
    }

    if( aArgs.contains( "rate" ) )
        r.SetRate( aArgs["rate"].get<double>() );

    if( aArgs.contains( "at" ) )
    {
        const long steps = r.Status()["steps"].get<long>();
        long       at = aArgs["at"].get<long>();

        if( at < 0 )
            at += steps;

        if( at < 0 || at >= steps )
            return KOPENAPI_RESULT::Error( 400, "at: 0 to " + std::to_string( steps - 1 ) + " (negative from the end)" );

        r.Play( false );
        r.Seek( (double) at );
    }

    if( aArgs.contains( "delta" ) )
    {
        r.Play( false );
        r.Seek( r.Status()["at"].get<double>() + aArgs["delta"].get<double>() );
    }

    if( verb == "play" )
        r.Play( true );
    else if( verb == "pause" )
        r.Play( false );
    else if( verb != "show" && verb != "seek" )
        return KOPENAPI_RESULT::Error( 400, "verb: show, seek, play, pause, hide or status" );

    return KOPENAPI_RESULT::Ok( r.Status() );
}
} // namespace


KOPENAPI_REGISTER( "pcb_route_replay",
                   "EXPERIMENTAL. The last recorded grid route (pcb_route {engine: 'grid', record: true}) replayed "
                   "over the board editor's canvas: every net's copper at the step in its layer's colour, the "
                   "net being taken white with the area searched, the places crowded right after the step "
                   "(magenta), the places fought over in the last two rounds (orange, fading), and a strip "
                   "with the crowded places over the run. The board's copper of those nets is hidden from the "
                   "view while shown; the board is not changed. verb show (lane, at), seek (at | delta), "
                   "play (rate: steps per second), pause, hide, status. Keys while shown: space play / pause, "
                   "left / right a step (shift: 10), home / end, up / down speed, escape closes; drag on the "
                   "strip to seek. GUI only",
                   R"json({"type":"object","properties":{
                        "verb":{"type":"string","enum":["show","seek","play","pause","hide","status"],"default":"show"},
                        "lane":{"description":"lane index or variant name; default the winner"},
                        "at":{"type":"integer","description":"step; negative from the end"},
                        "delta":{"type":"number","description":"steps to move from the current one"},
                        "rate":{"type":"number","description":"steps per second while playing; default the run in 20 s"}}})json"_json,
                   true, h_pcb_route_replay );
