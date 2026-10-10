/// @file kicadopenapi_view_bench.h
/// @brief view_benchmark driver for GAL canvases (schematic and board editors): fixed scenarios drawn
/// frame by frame through EDA_DRAW_PANEL_GAL::DoRePaint, measured by KIGFX::RENDER_STATS, summarised
/// per scenario. The editors add their own scenarios through KOPENAPI_BENCH_HOOKS. Header-only, used
/// by the board and schematic kifaces; main thread, GUI.
#ifndef KICADOPENAPI_VIEW_BENCH_H
#define KICADOPENAPI_VIEW_BENCH_H

#include <class_draw_panel_gal.h>
#include <gal/graphics_abstraction_layer.h>
#include <gal/render_stats.h>
#include <kicadopenapi_registry.h>
#include <layer_ids.h>
#include <view/view.h>

#include <wx/glcanvas.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>


/// @brief What an editor adds to the generic scenarios; a scenario whose hook is unset is skipped
struct KOPENAPI_BENCH_HOOKS
{
    BOX2D fitBox;   ///< what "fit" shows (the board outline, the drawing sheet); empty: the current view

    std::string                     option;      ///< what the option scenario toggles
    std::function<void( bool aOn )> setOption;   ///< option on / back to the user's value

    /// @brief Layer visibility scenario: name, set visible (true puts the user's state back)
    std::vector<std::pair<std::string, std::function<void( bool aVisible )>>> layers;

    std::string                     selectWhat;   ///< what the select scenario selects
    std::function<void( bool aOn )> select;       ///< the fixed set selected / selection cleared

    std::string                     highlightWhat;   ///< the net highlighted
    std::function<void( bool aOn )> highlight;       ///< net highlight on / off

    std::string            editWhat;   ///< what the edit scenario changes
    std::function<bool()>  edit;       ///< one small change as one undo step
    std::function<bool()>  undo;       ///< undoes it

    std::function<void()> restore;   ///< after the run: the user's selection, highlight, modified flag
};


/// @brief Name of a view layer: board / schematic layer names, board virtual layers by their base layer
inline std::string KopenapiViewLayerName( int aLayer )
{
    auto name = []( int aId ) -> std::string
    {
        const std::string n = LayerName( aId ).ToStdString( wxConvUTF8 );

        // LayerName falls back to board layer names for ids it does not know
        const bool fallback = aId >= PCB_LAYER_ID_COUNT && ( n.rfind( "In", 0 ) == 0 || n.rfind( "User.", 0 ) == 0 );
        return n.empty() || fallback ? "layer " + std::to_string( aId ) : n;
    };

    static const std::vector<std::tuple<int, int, const char*>> ranges = {
        { NETNAMES_LAYER_ID_START, NETNAMES_LAYER_ID_START + PCB_LAYER_ID_COUNT, "net names" },
        { LAYER_ZONE_START, LAYER_ZONE_END, "zones" },
        { LAYER_PAD_COPPER_START, LAYER_PAD_COPPER_END, "pads" },
        { LAYER_VIA_COPPER_START, LAYER_VIA_COPPER_END, "vias" },
        { LAYER_CLEARANCE_START, LAYER_CLEARANCE_END, "clearance" },
        { LAYER_BITMAP_START, LAYER_BITMAP_END, "bitmaps" },
        { LAYER_POINT_START, LAYER_POINT_END, "points" },
        { LAYER_DRILL_SYMBOL_START, LAYER_DRILL_SYMBOL_END, "drill symbols" } };

    for( const auto& [start, end, what] : ranges )
    {
        if( aLayer >= start && aLayer < end )
            return name( aLayer - start ) + " " + what;
    }

    return name( aLayer );
}


namespace KOPENAPI_BENCH
{

/// @brief One measured step: the scenario's action (if any) and the frame drawn after it
struct STEP
{
    double                    action = 0;   ///< ms of the action before the frame
    double                    step = 0;     ///< ms of action + frame
    bool                      drawn = false;
    int                       frames = 0;   ///< frames drawn during the step (the action may draw its own)
    KIGFX::RENDER_FRAME_STATS frame;        ///< every frame drawn during the step, summed
};


/// @brief Adds aFrame to aSum (times and counts summed, cache capacity: the largest)
inline void accumulate( KIGFX::RENDER_FRAME_STATS& aSum, const KIGFX::RENDER_FRAME_STATS& aFrame )
{
    aSum.total += aFrame.total;
    aSum.update += aFrame.update;
    aSum.rtree += aFrame.rtree;
    aSum.redraw += aFrame.redraw;
    aSum.begin += aFrame.begin;
    aSum.end += aFrame.end;
    aSum.noncached += aFrame.noncached;
    aSum.cached += aFrame.cached;
    aSum.overlay += aFrame.overlay;
    aSum.composite += aFrame.composite;
    aSum.swap += aFrame.swap;
    aSum.layers += aFrame.layers;
    aSum.itemsCached += aFrame.itemsCached;
    aSum.itemsImmediate += aFrame.itemsImmediate;
    aSum.itemsRecached += aFrame.itemsRecached;
    aSum.itemsUpdated += aFrame.itemsUpdated;
    aSum.cachedIndices += aFrame.cachedIndices;
    aSum.noncachedVertices += aFrame.noncachedVertices;
    aSum.overlayVertices += aFrame.overlayVertices;
    aSum.drawCalls += aFrame.drawCalls;
    aSum.cacheVertices = std::max( aSum.cacheVertices, aFrame.cacheVertices );
}


inline nlohmann::json distribution( std::vector<double> aValues )
{
    if( aValues.empty() )
        return nullptr;

    std::sort( aValues.begin(), aValues.end() );

    auto at = [&]( double aQ )
    {
        const size_t i = std::min( aValues.size() - 1, (size_t) std::floor( aQ * ( aValues.size() - 1 ) + 0.5 ) );
        return aValues[i];
    };

    double sum = 0;

    for( double v : aValues )
        sum += v;

    auto r = []( double x ) { return std::round( x * 1000 ) / 1000; };

    return { { "mean", r( sum / aValues.size() ) }, { "p50", r( at( 0.5 ) ) }, { "p95", r( at( 0.95 ) ) },
             { "max", r( aValues.back() ) } };
}


/// @brief Summary of a scenario's steps and the per-layer sums collected during it
inline nlohmann::json summarise( const std::string& aName, const std::string& aWhat, const std::vector<STEP>& aSteps,
                                 const KIGFX::RENDER_STATS& aStats, int aLayersTop, bool aDetail )
{
    std::vector<double> steps, frames;
    KIGFX::RENDER_FRAME_STATS sum;
    double   action = 0;
    int      drawn = 0;
    int      frameCount = 0;

    for( const STEP& s : aSteps )
    {
        steps.push_back( s.step );
        action += s.action;

        if( !s.drawn )
            continue;

        drawn++;
        frames.push_back( s.frame.total );

        accumulate( sum, s.frame );
        frameCount += s.frames;
    }

    const double n = std::max( 1, drawn );
    auto         ms = [&]( double x ) { return std::round( x / n * 1000 ) / 1000; };
    auto         cnt = [&]( uint64_t x ) { return (uint64_t) std::llround( x / n ); };

    // what DoRePaint spends outside the measured stages (scrollbars, grid, cursor, context switch)
    const double other = sum.total - sum.update - sum.redraw - sum.begin - sum.end;

    nlohmann::json out = {
        { "name", aName },
        { "frames", drawn },
        { "step_ms", distribution( steps ) },
        { "frame_ms", distribution( frames ) },
        { "stages_ms",
          { { "action", std::round( action / std::max<size_t>( 1, aSteps.size() ) * 1000 ) / 1000 },
            { "update", ms( sum.update ) },
            { "rtree", ms( sum.rtree ) },
            { "redraw", ms( sum.redraw ) },
            { "begin", ms( sum.begin ) },
            { "noncached", ms( sum.noncached ) },
            { "cached", ms( sum.cached ) },
            { "overlay", ms( sum.overlay ) },
            { "composite", ms( sum.composite ) },
            { "swap", ms( sum.swap ) },
            { "end", ms( sum.end ) },
            { "other", ms( other ) } } },
        { "counts",
          { { "layers", cnt( sum.layers ) },
            { "items_cached", cnt( sum.itemsCached ) },
            { "items_immediate", cnt( sum.itemsImmediate ) },
            { "items_recached", cnt( sum.itemsRecached ) },
            { "items_updated", cnt( sum.itemsUpdated ) },
            { "cached_indices", cnt( sum.cachedIndices ) },
            { "noncached_vertices", cnt( sum.noncachedVertices ) },
            { "overlay_vertices", cnt( sum.overlayVertices ) },
            { "triangles", cnt( ( sum.cachedIndices + sum.noncachedVertices + sum.overlayVertices ) / 3 ) },
            { "draw_calls", cnt( sum.drawCalls ) },
            { "cache_vertices", sum.cacheVertices } } },
        { "frames_drawn", frameCount } };

    if( !aWhat.empty() )
        out["what"] = aWhat;

    if( aLayersTop > 0 && !aStats.layers.empty() )
    {
        std::vector<std::pair<int, KIGFX::RENDER_LAYER_STATS>> layers( aStats.layers.begin(), aStats.layers.end() );

        std::sort( layers.begin(), layers.end(),
                   []( const auto& a, const auto& b ) { return a.second.ms > b.second.ms; } );

        nlohmann::json top = nlohmann::json::array();

        for( size_t i = 0; i < layers.size() && (int) i < aLayersTop; ++i )
        {
            top.push_back( { { "layer", KopenapiViewLayerName( layers[i].first ) },
                             { "id", layers[i].first },
                             { "ms", ms( layers[i].second.ms ) },
                             { "items", cnt( layers[i].second.items ) } } );
        }

        out["layers_top"] = top;
    }

    if( aDetail )
    {
        nlohmann::json rows = nlohmann::json::array();

        for( const STEP& s : aSteps )
        {
            const KIGFX::RENDER_FRAME_STATS& f = s.frame;
            rows.push_back( { s.step, s.action, f.total, f.update, f.redraw, f.noncached, f.cached, f.overlay,
                              f.composite, f.swap, f.itemsImmediate, f.itemsRecached, f.drawCalls } );
        }

        out["frames_detail"] = { { "columns", { "step", "action", "total", "update", "redraw", "noncached", "cached",
                                                "overlay", "composite", "swap", "items_immediate",
                                                "items_recached", "draw_calls" } },
                                 { "rows", rows } };
    }

    return out;
}

} // namespace KOPENAPI_BENCH


/// @brief Runs the benchmark on aCanvas and answers per scenario (see view_benchmark's schema).
/// The view (centre, scale), the swap interval and whatever the hooks change are put back.
/// @param aInfo document facts merged into the answer (editor, document, item counts)
inline KOPENAPI_RESULT KopenapiViewBenchmark( EDA_DRAW_PANEL_GAL* aCanvas, KOPENAPI_BENCH_HOOKS& aHooks,
                                              const nlohmann::json& aArgs, nlohmann::json aInfo )
{
    using namespace KOPENAPI_BENCH;

    KIGFX::VIEW* view = aCanvas ? aCanvas->GetView() : nullptr;
    KIGFX::GAL*  gal = aCanvas ? aCanvas->GetGAL() : nullptr;

    if( !view || !gal || !aCanvas->IsShownOnScreen() )
        return KOPENAPI_RESULT::Error( 409, "the editor's canvas is not on screen" );

    if( !gal->IsOpenGlEngine() )
        return KOPENAPI_RESULT::Error( 409, "the canvas does not draw with OpenGL (Preferences > Common > Graphics)" );

    static const std::vector<std::string> all = { "static", "compose", "pan",    "zoom",      "recache",
                                                   "option", "layers",  "select", "highlight", "edit" };
    std::vector<std::string> scenarios;

    for( const nlohmann::json& s : aArgs.value( "scenarios", nlohmann::json::array() ) )
    {
        const std::string name = s.get<std::string>();

        if( std::find( all.begin(), all.end(), name ) == all.end() )
            return KOPENAPI_RESULT::Error( 400, "unknown scenario: " + name );

        scenarios.push_back( name );
    }

    if( scenarios.empty() )
        scenarios = all;

    const int  frames = std::clamp( aArgs.value( "frames", 60 ), 4, 2000 );
    const int  cycles = std::clamp( aArgs.value( "cycles", 6 ), 1, 200 );
    const int  warmup = std::clamp( aArgs.value( "warmup", 3 ), 0, 100 );
    const int  layersTop = std::clamp( aArgs.value( "layers_top", 8 ), 0, 100 );
    const bool fit = aArgs.value( "fit", true );
    const bool vsync = aArgs.value( "vsync", false );
    const bool detail = aArgs.value( "frames_detail", false );

    const VECTOR2D centre = view->GetCenter();
    const double   scale = view->GetScale();

    KIGFX::RENDER_STATS stats;
    stats.gpuSync = aArgs.value( "gpu_sync", false );

    // the display's refresh must not cap the frame times: swap interval 0 for the run
    wxGLCanvas* glCanvas = dynamic_cast<wxGLCanvas*>( gal );
    int         swapInterval = 0;
    bool        swapChanged = false;

#if wxCHECK_VERSION( 3, 3, 3 )
    if( glCanvas && !vsync )
    {
        KIGFX::GAL_CONTEXT_LOCKER lock( gal );
        swapInterval = glCanvas->GetSwapInterval();
        swapChanged = glCanvas->SetSwapInterval( 0 ) == wxGLCanvas::SwapInterval::Set;
    }
#endif

    // a step's numbers are every frame drawn during it: actions such as a commit or an undo may
    // repaint the canvas themselves before the step's own frame
    auto paint = [&]( const std::function<void()>& aAction ) -> STEP
    {
        STEP         s;
        const size_t first = stats.frames.size();
        const auto   t0 = std::chrono::steady_clock::now();

        if( aAction )
            aAction();

        const double actionWall = KIGFX::RENDER_STATS::Since( t0 );
        double       inAction = 0;

        for( size_t i = first; i < stats.frames.size(); ++i )
            inAction += stats.frames[i].total;

        aCanvas->DoRePaint( false );
        s.step = KIGFX::RENDER_STATS::Since( t0 );
        s.action = actionWall - inAction;

        for( size_t i = first; i < stats.frames.size(); ++i )
            accumulate( s.frame, stats.frames[i] );

        s.frames = (int) ( stats.frames.size() - first );
        s.drawn = s.frames > 0;
        return s;
    };

    auto home = [&]()
    {
        if( fit && aHooks.fitBox.GetWidth() > 0 && aHooks.fitBox.GetHeight() > 0 )
            view->SetViewport( aHooks.fitBox );
        else
        {
            view->SetScale( scale );
            view->SetCenter( centre );
        }

        view->MarkDirty();
    };

    const VECTOR2D homeCentre = [&]()
    {
        home();
        return view->GetCenter();
    }();
    const double   homeScale = view->GetScale();
    const BOX2D    homeViewport = view->GetViewport();

    nlohmann::json results = nlohmann::json::array();
    nlohmann::json skipped = nlohmann::json::array();

    KIGFX::RENDER_STATS::SetActive( &stats );

    for( const std::string& name : scenarios )
    {
        std::vector<STEP> steps;
        std::string       what;

        home();

        for( int i = 0; i < warmup; ++i )
        {
            view->MarkDirty();
            aCanvas->DoRePaint( false );
        }

        // a settled frame first so the scenario starts from a drawn view
        view->MarkDirty();
        aCanvas->DoRePaint( false );
        stats.Clear();

        if( name == "static" )
        {
            what = "full redraw from the cache (all targets dirty)";

            for( int i = 0; i < frames; ++i )
                steps.push_back( paint( [&]() { view->MarkDirty(); } ) );
        }
        else if( name == "compose" )
        {
            what = "nothing changed: buffers composited and swapped only";

            for( int i = 0; i < frames; ++i )
                steps.push_back( paint( nullptr ) );
        }
        else if( name == "pan" )
        {
            what = "4x the fit zoom, centre on a Lissajous path over the fitted area";
            view->SetScale( homeScale * 4 );

            for( int i = 0; i < frames; ++i )
            {
                const double t = 2 * 3.14159265358979323846 * i / frames;
                const VECTOR2D c( homeCentre.x + homeViewport.GetWidth() * 0.4 * std::sin( t ),
                                  homeCentre.y + homeViewport.GetHeight() * 0.4 * std::sin( 2 * t ) );
                steps.push_back( paint( [&]() { view->SetCenter( c ); } ) );
            }
        }
        else if( name == "zoom" )
        {
            what = "zoom into the fitted centre up to 32x and back out";

            for( int i = 0; i < frames; ++i )
            {
                const double u = (double) i / ( frames - 1 );
                const double tri = 1 - std::abs( 2 * u - 1 );
                const double s = homeScale * std::pow( 2.0, 5 * tri );
                steps.push_back( paint( [&]() { view->SetScale( s, homeCentre ); } ) );
            }
        }
        else if( name == "recache" )
        {
            what = "every item re-tessellated into the cache (VIEW::RecacheAllItems), like a geometry option change";

            for( int i = 0; i < cycles; ++i )
            {
                steps.push_back( paint(
                        [&]()
                        {
                            view->RecacheAllItems();
                            view->MarkDirty();
                        } ) );
            }
        }
        else if( name == "option" && aHooks.setOption )
        {
            what = aHooks.option;

            for( int i = 0; i < cycles; ++i )
            {
                steps.push_back( paint( [&]() { aHooks.setOption( true ); } ) );
                steps.push_back( paint( [&]() { aHooks.setOption( false ); } ) );
            }
        }
        else if( name == "layers" && !aHooks.layers.empty() )
        {
            for( size_t l = 0; l < aHooks.layers.size(); ++l )
            {
                what += ( l ? ", " : "hidden and shown again: " ) + aHooks.layers[l].first;
                steps.push_back( paint( [&]() { aHooks.layers[l].second( false ); } ) );
                steps.push_back( paint( [&]() { aHooks.layers[l].second( true ); } ) );
            }
        }
        else if( name == "select" && aHooks.select )
        {
            what = aHooks.selectWhat;

            for( int i = 0; i < cycles; ++i )
            {
                steps.push_back( paint( [&]() { aHooks.select( true ); } ) );
                steps.push_back( paint( [&]() { aHooks.select( false ); } ) );
            }
        }
        else if( name == "highlight" && aHooks.highlight )
        {
            what = aHooks.highlightWhat;

            for( int i = 0; i < cycles; ++i )
            {
                steps.push_back( paint( [&]() { aHooks.highlight( true ); } ) );
                steps.push_back( paint( [&]() { aHooks.highlight( false ); } ) );
            }
        }
        else if( name == "edit" && aHooks.edit && aHooks.undo )
        {
            what = aHooks.editWhat;
            bool ok = true;

            for( int i = 0; i < cycles && ok; ++i )
            {
                steps.push_back( paint( [&]() { ok = aHooks.edit(); } ) );
                steps.push_back( paint( [&]() { ok = aHooks.undo() && ok; } ) );
            }

            if( !ok )
                what += " (an edit or undo failed: stopped)";
        }
        else
        {
            skipped.push_back( name );
            continue;
        }

        results.push_back( summarise( name, what, steps, stats, layersTop, detail ) );
    }

    KIGFX::RENDER_STATS::SetActive( nullptr );

#if wxCHECK_VERSION( 3, 3, 3 )
    if( swapChanged )
    {
        KIGFX::GAL_CONTEXT_LOCKER lock( gal );
        glCanvas->SetSwapInterval( swapInterval );
    }
#endif

    view->SetScale( scale );
    view->SetCenter( centre );
    view->MarkDirty();

    // after the view: a hook may switch the document's page back, which brings its own view
    if( aHooks.restore )
        aHooks.restore();

    aCanvas->DoRePaint( false );

    const VECTOR2I px = gal->GetScreenPixelSize();
    const wxSize   pt = aCanvas->GetClientSize();

    nlohmann::json out = std::move( aInfo );
    out["canvas_px"] = { px.x, px.y };
    out["canvas_pt"] = { pt.x, pt.y };
    out["content_scale"] = aCanvas->GetContentScaleFactor();
    out["gl_renderer"] = stats.glRenderer;
    out["gl_version"] = stats.glVersion;
    out["vsync"] = !swapChanged;
    out["gpu_sync"] = stats.gpuSync;
    out["fit"] = fit;
    out["frames_requested"] = frames;
    out["cycles"] = cycles;
    out["scenarios"] = results;

    if( !skipped.empty() )
        out["skipped"] = skipped;

    return KOPENAPI_RESULT::Ok( out );
}

#endif
