/// @file methods_pcb_render.cpp
/// @brief kicadopenapi board rendering: pcb_render (layers plotted to PNG) and pcb_render_3d (the 3D
/// raytracer).  Both run KiCad's own jobs — the code behind `kicad-cli pcb export png` and
/// `kicad-cli pcb render` — on the live board (unsaved edits included) through
/// PCBNEW_JOBS_HANDLER::SetBoardOverride, in the GUI and headless.  Over MCP the result is an
/// image.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_design_settings.h>
#include <page_info.h>
#include <zone.h>
#include <lset.h>
#include <jobs/job_export_pcb_png.h>
#include <jobs/job_pcb_render.h>
#include <kicadopenapi_image.h>
#include <kiway.h>
#include <pcbnew_jobs_handler.h>
#include <reporter.h>

#include <wx/filename.h>
#include <wx/image.h>

#include <algorithm>
#include <climits>
#include <cstdlib>
#include <cmath>
#include <map>
#include <optional>


namespace
{

/// @brief Board jobs on the live board for the duration of a call
struct BOARD_OVERRIDE
{
    explicit BOARD_OVERRIDE( BOARD* aBoard ) { PCBNEW_JOBS_HANDLER::SetBoardOverride( aBoard ); }
    ~BOARD_OVERRIDE() { PCBNEW_JOBS_HANDLER::SetBoardOverride( nullptr ); }
};


/// @brief The plot page as a window on the board for the duration of a call: the plotter's image
/// covers this box only, at any resolution (the drawing sheet is not drawn then)
struct PAGE_WINDOW
{
    PAGE_WINDOW( BOARD* aBoard, const BOX2I& aBox ) :
            board( aBoard ),
            page( aBoard->GetPageSettings() ),
            origin( aBoard->GetDesignSettings().GetAuxOrigin() )
    {
        PAGE_INFO window( PAGE_SIZE_TYPE::User );
        window.SetWidthMils( aBox.GetWidth() / pcbIUScale.IU_PER_MILS );
        window.SetHeightMils( aBox.GetHeight() / pcbIUScale.IU_PER_MILS );
        board->SetPageSettings( window );
        board->GetDesignSettings().SetAuxOrigin( aBox.GetOrigin() );
    }

    ~PAGE_WINDOW()
    {
        board->SetPageSettings( page );
        board->GetDesignSettings().SetAuxOrigin( origin );
    }

    BOARD*    board;
    PAGE_INFO page;
    VECTOR2I  origin;
};


/// @brief What one plotted image costs: the plotter draws into a surface of the window (a page
/// is at least an inch a side), the image is read back, cut and encoded
struct RENDER_COST
{
    int    width = 0;    ///< the plotter's surface, pixels
    int    height = 0;
    double megapixels = 0;
    double memoryMb = 0;
    double cpuS = 0;

    nlohmann::json Json() const
    {
        return { { "surface_px", { width, height } },
                 { "megapixels", std::round( megapixels * 10 ) / 10 },
                 { "memory_mb", std::lround( memoryMb ) },
                 { "cpu_s", std::round( cpuS * 10 ) / 10 },
                 { "cpu_reference", "one core of an Apple M1 Ultra; the wall time grows with the machine's load" } };
    }
};

// CPU seconds of one core of an M1 Ultra (TC-13, vme-wren): plotting, the image read back, cut and
// encoded; memory: the plotter's ARGB surface, then the image read back and its cut-out, 4 bytes a
// pixel each
constexpr double RENDER_BYTES_PER_PIXEL = 8.0;
constexpr double RENDER_S_PER_MEGAPIXEL = 0.06;
constexpr double RENDER_S_PER_KILO_ITEM = 0.002;
constexpr double RENDER_MAX_MEGAPIXELS = 64.0;
constexpr int    RENDER_MAX_SIDE = 32767;   // Cairo's image surface
constexpr int    RENDER_MAX_DPI = 25400;    // a micrometre a pixel


RENDER_COST renderCost( const BOARD* aBoard, const BOX2I& aWindow, int aDpi, int aLayers )
{
    const double inchIU = pcbIUScale.IU_PER_MILS * 1000.0;
    RENDER_COST  cost;

    cost.width = KiROUND( std::max( aWindow.GetWidth() / inchIU, 1.0 ) * aDpi );
    cost.height = KiROUND( std::max( aWindow.GetHeight() / inchIU, 1.0 ) * aDpi );
    cost.megapixels = (double) cost.width * cost.height / 1e6;

    const size_t items = aBoard->Tracks().size() + aBoard->Drawings().size() + aBoard->Zones().size()
                         + aBoard->Footprints().size() * 20;

    cost.memoryMb = cost.megapixels * RENDER_BYTES_PER_PIXEL;
    cost.cpuS = aLayers * ( cost.megapixels * RENDER_S_PER_MEGAPIXEL + items / 1000.0 * RENDER_S_PER_KILO_ITEM );
    return cost;
}


/// @brief A 422 for a render over the limits: what it would cost, and what fits
KOPENAPI_RESULT tooBig( const RENDER_COST& aCost, const BOX2I& aWindow, int aDpi )
{
    const double inchIU = pcbIUScale.IU_PER_MILS * 1000.0;
    const double wIn = std::max( aWindow.GetWidth() / inchIU, 1.0 );
    const double hIn = std::max( aWindow.GetHeight() / inchIU, 1.0 );
    const int    maxDpi = (int) std::min( { std::sqrt( RENDER_MAX_MEGAPIXELS * 1e6 / ( wIn * hIn ) ),
                                            RENDER_MAX_SIDE / wIn, RENDER_MAX_SIDE / hIn } );
    const double fitMm = std::sqrt( RENDER_MAX_MEGAPIXELS * 1e6 ) / aDpi * 25.4;

    KOPENAPI_RESULT result = KOPENAPI_RESULT::Error( 422, "the image would be too large: render a smaller region "
                                                          "or at a lower dpi (estimate: what it would cost)" );
    nlohmann::json& error = result.body["error"];
    error["estimate"] = aCost.Json();
    error["limits"] = { { "megapixels", RENDER_MAX_MEGAPIXELS }, { "side_px", RENDER_MAX_SIDE } };
    error["max_dpi_for_this_window"] = maxDpi;
    error["max_square_region_mm_at_this_dpi"] = std::round( std::min( fitMm, RENDER_MAX_SIDE * 25.4 / aDpi ) * 10 ) / 10;
    return result;
}


wxString tempPng()
{
    wxFileName fn( wxFileName::CreateTempFileName( wxS( "kopenapi-pcb" ) ) );
    wxRemoveFile( fn.GetFullPath() );
    fn.SetExt( wxS( "png" ) );
    return fn.GetFullPath();
}


KOPENAPI_RESULT readImage( const wxString& aFile, int aStatus, const wxString& aMessages, bool aCrop, int aMargin,
                           wxImage& aImage )
{
    if( !wxImage::FindHandler( wxBITMAP_TYPE_PNG ) )
        wxImage::AddHandler( new wxPNGHandler );

    const bool ok = aStatus == 0 && wxFileName::FileExists( aFile ) && aImage.LoadFile( aFile, wxBITMAP_TYPE_PNG );
    wxRemoveFile( aFile );

    if( !ok )
        return KOPENAPI_RESULT::Error( 500, "KiCad's job failed: " + aMessages.ToStdString( wxConvUTF8 ) );

    if( aCrop )
        aImage = KopenapiCropToContent( aImage, aMargin );

    return KOPENAPI_RESULT::Ok( nlohmann::json::object() );
}

} // namespace


static KOPENAPI_RESULT h_pcb_render( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    static const std::map<std::string, std::string> presets = {
        { "front", "F.Cu,F.Mask,F.SilkS,Edge.Cuts" },
        { "back", "B.Cu,B.Mask,B.SilkS,Edge.Cuts" },
        { "copper", "F.Cu,B.Cu,Edge.Cuts" },
        { "assembly", "F.Fab,F.SilkS,Edge.Cuts" } };

    std::string layers = aArgs.value( "layers", std::string( "front" ) );

    if( auto preset = presets.find( layers ); preset != presets.end() )
        layers = preset->second;

    const int dpi = aArgs.value( "dpi", 150 );
    BOARD*    board = context->GetBoard();

    if( dpi < 30 || dpi > RENDER_MAX_DPI )
        return KOPENAPI_RESULT::Error( 400, "dpi: 30 to " + std::to_string( RENDER_MAX_DPI ) );

    // what is drawn: a region of the board, else the board with a margin, else (drawing sheet) the page
    const nlohmann::json region = aArgs.value( "region", nlohmann::json() );
    const bool           byRegion = region.is_array() && region.size() == 4
                                    && std::all_of( region.begin(), region.end(), []( const nlohmann::json& v ) { return v.is_number(); } );
    const bool           sheet = aArgs.value( "drawing_sheet", false );

    if( !region.is_null() && !byRegion )
        return KOPENAPI_RESULT::Error( 400, "region: [x0_mm, y0_mm, x1_mm, y1_mm]" );

    if( byRegion && sheet )
        return KOPENAPI_RESULT::Error( 400, "region draws no drawing sheet: drawing_sheet false" );

    BOX2I window;

    if( byRegion )
    {
        const VECTOR2I a( pcbIUScale.mmToIU( region[0].get<double>() ), pcbIUScale.mmToIU( region[1].get<double>() ) );
        const VECTOR2I b( pcbIUScale.mmToIU( region[2].get<double>() ), pcbIUScale.mmToIU( region[3].get<double>() ) );
        window = BOX2I( a, b - a );
        window.Normalize();

        if( window.GetWidth() <= 0 || window.GetHeight() <= 0 )
            return KOPENAPI_RESULT::Error( 400, "region: an empty box" );
    }
    else if( !sheet )
    {
        window = board->ComputeBoundingBox( false, false );
        window.Inflate( pcbIUScale.mmToIU( 25.4 / 6 ) );   // the crop's margin (a sixth of an inch)
    }

    if( sheet || window.GetWidth() <= 0 || window.GetHeight() <= 0 )
    {
        const PAGE_INFO& page = board->GetPageSettings();
        window = BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( page.GetWidthIU( pcbIUScale.IU_PER_MILS ),
                                                     page.GetHeightIU( pcbIUScale.IU_PER_MILS ) ) );
    }

    const RENDER_COST cost = renderCost( board, window, dpi, 1 );

    if( cost.megapixels > RENDER_MAX_MEGAPIXELS || cost.width > RENDER_MAX_SIDE || cost.height > RENDER_MAX_SIDE )
        return tooBig( cost, window, dpi );

    if( aArgs.value( "estimate_only", false ) )
        return KOPENAPI_RESULT::Ok( { { "estimate", cost.Json() }, { "dpi", dpi } } );

    JOB_EXPORT_PCB_PNG job;
    job.m_filename = board->GetFileName();
    job.m_genMode = JOB_EXPORT_PCB_PNG::GEN_MODE::SINGLE;
    job.m_dpi = dpi;
    job.m_antialias = true;
    job.m_useBackgroundColor = true;
    job.m_argLayers = wxString::FromUTF8( layers );
    job.m_plotDrawingSheet = aArgs.value( "drawing_sheet", false );
    job.m_blackAndWhite = aArgs.value( "black_and_white", false );
    job.m_mirror = aArgs.value( "mirror", false );
    job.m_checkZonesBeforePlot = false;

    if( aArgs.contains( "theme" ) )
        job.m_colorTheme = wxString::FromUTF8( aArgs["theme"].get<std::string>() );

    const wxString out = tempPng();
    job.SetConfiguredOutputPath( out );

    // zones: filled (as plotted), outline (each fill drawn as its edge: what is under a pour shows),
    // hidden; the fills are swapped for the render only and put back
    const std::string zoneMode = aArgs.value( "zones", std::string( "filled" ) );

    if( zoneMode != "filled" && zoneMode != "outline" && zoneMode != "hidden" )
        return KOPENAPI_RESULT::Error( 400, "zones: filled, outline or hidden" );

    std::vector<std::tuple<ZONE*, PCB_LAYER_ID, SHAPE_POLY_SET>> saved;

    if( zoneMode != "filled" )
    {
        const int edge = std::max( pcbIUScale.mmToIU( 0.1 ), KiROUND( 2.0 * 25.4 / dpi * pcbIUScale.IU_PER_MM ) );

        for( ZONE* zone : board->Zones() )
        {
            for( PCB_LAYER_ID layer : zone->GetLayerSet().Seq() )
            {
                if( !zone->HasFilledPolysForLayer( layer ) )
                    continue;

                const SHAPE_POLY_SET fill = *zone->GetFilledPolysList( layer );
                SHAPE_POLY_SET       shown;

                if( zoneMode == "outline" )
                {
                    // fills are stored fractured (holes joined to the outline): deflate the real shape
                    SHAPE_POLY_SET whole = fill;
                    whole.Unfracture();
                    SHAPE_POLY_SET inner = whole;
                    inner.Deflate( edge, CORNER_STRATEGY::ROUND_ALL_CORNERS, ARC_HIGH_DEF );
                    shown = whole;
                    shown.BooleanSubtract( inner );
                    shown.Fracture();
                }

                saved.emplace_back( zone, layer, fill );
                zone->SetFilledPolysList( layer, shown );
            }
        }
    }

    WX_STRING_REPORTER reporter;
    int                status;

    {
        BOARD_OVERRIDE live( board );
        std::optional<PAGE_WINDOW> page;

        if( !sheet )
        {
            page.emplace( board, window );
            job.m_useDrillOrigin = true;
        }

        status = aCtx.kiway->ProcessJob( KIWAY::FACE_PCB, &job, &reporter );
    }

    for( auto& [zone, layer, fill] : saved )
        zone->SetFilledPolysList( layer, fill );

    wxImage         image;
    KOPENAPI_RESULT read = readImage( out, status, reporter.GetMessages(), false, 0, image );

    if( read.status != 200 )
        return read;

    // a page is at least an inch a side: the window is the image's top left
    const double pxPerIU = dpi / ( pcbIUScale.IU_PER_MILS * 1000.0 );
    wxRect       want( 0, 0, KiROUND( window.GetWidth() * pxPerIU ), KiROUND( window.GetHeight() * pxPerIU ) );
    want.Intersect( wxRect( 0, 0, image.GetWidth(), image.GetHeight() ) );

    if( !sheet && !want.IsEmpty() && want.GetSize() != image.GetSize() )
        image = image.GetSubImage( want );

    if( !byRegion && aArgs.value( "crop", true ) )
        image = KopenapiCropToContent( image, dpi / 6 );

    nlohmann::json result = KopenapiImageResult( image );
    result["layers"] = layers;
    result["dpi"] = dpi;
    result["estimate"] = cost.Json();

    if( byRegion )
        result["region_mm"] = region;

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_render_3d( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    static const std::map<std::string, JOB_PCB_RENDER::SIDE> sides = {
        { "top", JOB_PCB_RENDER::SIDE::TOP },     { "bottom", JOB_PCB_RENDER::SIDE::BOTTOM },
        { "left", JOB_PCB_RENDER::SIDE::LEFT },   { "right", JOB_PCB_RENDER::SIDE::RIGHT },
        { "front", JOB_PCB_RENDER::SIDE::FRONT }, { "back", JOB_PCB_RENDER::SIDE::BACK } };

    std::string sideName = aArgs.value( "side", std::string( "top" ) );
    const bool  iso = sideName == "iso";

    // "iso": the top view tilted and turned, the usual look at a populated board
    auto side = sides.find( iso ? std::string( "top" ) : sideName );

    if( side == sides.end() )
        return KOPENAPI_RESULT::Error( 400, "side must be top, bottom, left, right, front, back or iso" );

    BOARD*         board = context->GetBoard();
    JOB_PCB_RENDER job;
    job.m_filename = board->GetFileName();
    job.m_format = JOB_PCB_RENDER::FORMAT::PNG;
    job.m_quality = aArgs.value( "quality", std::string( "basic" ) ) == "high" ? JOB_PCB_RENDER::QUALITY::HIGH
                                                                             : JOB_PCB_RENDER::QUALITY::BASIC;
    job.m_bgStyle = JOB_PCB_RENDER::BG_STYLE::OPAQUE;
    job.m_width = std::clamp( aArgs.value( "width", 1600 ), 64, 6000 );
    job.m_height = std::clamp( aArgs.value( "height", 1000 ), 64, 6000 );
    job.m_side = side->second;
    job.m_zoom = std::clamp( aArgs.value( "zoom", 1.0 ), 0.1, 20.0 );
    job.m_perspective = aArgs.value( "perspective", false );   // like kicad-cli: tall parts do not fill the view
    job.m_floor = aArgs.value( "floor", false );

    if( iso )
        job.m_rotation = VECTOR3D( -45.0, 0.0, 45.0 );

    if( aArgs.contains( "rotate" ) && aArgs["rotate"].is_array() && aArgs["rotate"].size() == 3 )
        job.m_rotation = VECTOR3D( aArgs["rotate"][0].get<double>(), aArgs["rotate"][1].get<double>(), aArgs["rotate"][2].get<double>() );

    const wxString out = tempPng();
    job.SetConfiguredOutputPath( out );

    WX_STRING_REPORTER reporter;
    int                status;

    {
        BOARD_OVERRIDE live( board );
        status = aCtx.kiway->ProcessJob( KIWAY::FACE_PCB, &job, &reporter );
    }

    wxImage         image;
    KOPENAPI_RESULT read = readImage( out, status, reporter.GetMessages(), false, 0, image );

    if( read.status != 200 )
        return read;

    nlohmann::json result = KopenapiImageResult( image );
    result["side"] = sideName;
    return KOPENAPI_RESULT::Ok( result );
}


namespace
{

const char* layerKind( PCB_LAYER_ID aLayer, LAYER_T aType )
{
    if( IsCopperLayer( aLayer ) )
        return aType == LT_POWER ? "copper_plane" : "copper";

    switch( aLayer )
    {
    case F_Mask: case B_Mask:     return "solder_mask";
    case F_Paste: case B_Paste:   return "paste";
    case F_SilkS: case B_SilkS:   return "silkscreen";
    case F_Fab: case B_Fab:       return "fabrication";
    case F_CrtYd: case B_CrtYd:   return "courtyard";
    case F_Adhes: case B_Adhes:   return "adhesive";
    case Edge_Cuts:               return "board_outline";
    case Margin:                  return "margin";
    case Dwgs_User: case Cmts_User:
    case Eco1_User: case Eco2_User: return "drawing";
    default:                      return "user";
    }
}

} // namespace


static KOPENAPI_RESULT h_pcb_layer_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    nlohmann::json layers = nlohmann::json::array();

    for( PCB_LAYER_ID layer : board->GetEnabledLayers().UIOrder() )
    {
        layers.push_back( { { "name", LSET::Name( layer ).ToStdString( wxConvUTF8 ) },
                            { "user_name", board->GetLayerName( layer ).ToStdString( wxConvUTF8 ) },
                            { "kind", layerKind( layer, board->GetLayerType( layer ) ) },
                            { "side", IsFrontLayer( layer ) ? "front" : IsBackLayer( layer ) ? "back" : "none" },
                            { "visible", board->IsLayerVisible( layer ) } } );
    }

    return KOPENAPI_RESULT::Ok( { { "copper_layers", board->GetCopperLayerCount() }, { "layers", layers } } );
}


static KOPENAPI_RESULT h_pcb_render_layers( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    // Which layers: names, or "all" / "copper" (enabled ones)
    std::vector<PCB_LAYER_ID> wanted;
    const nlohmann::json      spec = aArgs.value( "layers", nlohmann::json( "copper" ) );

    if( spec.is_string() && ( spec == "all" || spec == "copper" ) )
    {
        for( PCB_LAYER_ID layer : board->GetEnabledLayers().UIOrder() )
        {
            if( spec == "all" || IsCopperLayer( layer ) )
                wanted.push_back( layer );
        }
    }
    else if( spec.is_array() )
    {
        for( const nlohmann::json& name : spec )
        {
            wxString  layerName = name.is_string() ? wxString::FromUTF8( name.get<std::string>() ) : wxString();
            const int id = layerName.IsEmpty() ? -1 : LSET::NameToLayer( layerName );

            if( id < 0 || !board->IsLayerEnabled( (PCB_LAYER_ID) id ) )
                return KOPENAPI_RESULT::Error( 400, "unknown or disabled layer: " + name.dump() + " (see pcb_layer_list)" );

            wanted.push_back( (PCB_LAYER_ID) id );
        }
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "layers: \"all\", \"copper\" or a list of layer names" );
    }

    if( wanted.empty() || wanted.size() > 64 )
        return KOPENAPI_RESULT::Error( 400, "no layers to render" );

    const int  dpi = std::clamp( aArgs.value( "dpi", 100 ), 30, 600 );
    const bool outline = aArgs.value( "outline", true );

    // the board with the crop's margin, not the page
    BOX2I window = board->ComputeBoundingBox( false, false );
    window.Inflate( pcbIUScale.mmToIU( 25.4 / 6 ) );

    if( window.GetWidth() <= 0 || window.GetHeight() <= 0 )
    {
        const PAGE_INFO& page = board->GetPageSettings();
        window = BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( page.GetWidthIU( pcbIUScale.IU_PER_MILS ),
                                                     page.GetHeightIU( pcbIUScale.IU_PER_MILS ) ) );
    }

    const RENDER_COST cost = renderCost( board, window, dpi, (int) wanted.size() );

    if( cost.width > RENDER_MAX_SIDE || cost.height > RENDER_MAX_SIDE
        || cost.megapixels * wanted.size() > RENDER_MAX_MEGAPIXELS * 4 )
    {
        return tooBig( cost, window, dpi );
    }

    if( aArgs.value( "estimate_only", false ) )
        return KOPENAPI_RESULT::Ok( { { "estimate", cost.Json() }, { "dpi", dpi } } );

    const double pxPerIU = dpi / ( pcbIUScale.IU_PER_MILS * 1000.0 );

    std::vector<wxImage>     images;
    std::vector<std::string> names;

    for( PCB_LAYER_ID layer : wanted )
    {
        JOB_EXPORT_PCB_PNG job;
        job.m_filename = board->GetFileName();
        job.m_genMode = JOB_EXPORT_PCB_PNG::GEN_MODE::SINGLE;
        job.m_dpi = dpi;
        job.m_antialias = true;
        job.m_useBackgroundColor = true;
        job.m_argLayers = LSET::Name( layer );

        if( outline && layer != Edge_Cuts )
            job.m_argCommonLayers = wxS( "Edge.Cuts" );

        job.m_plotDrawingSheet = false;
        job.m_checkZonesBeforePlot = false;

        const wxString out = tempPng();
        job.SetConfiguredOutputPath( out );

        WX_STRING_REPORTER reporter;
        int                status;

        {
            BOARD_OVERRIDE live( board );
            PAGE_WINDOW    page( board, window );
            job.m_useDrillOrigin = true;
            status = aCtx.kiway->ProcessJob( KIWAY::FACE_PCB, &job, &reporter );
        }

        wxImage         image;
        KOPENAPI_RESULT read = readImage( out, status, reporter.GetMessages(), false, 0, image );

        if( read.status != 200 )
            return read;

        wxRect want( 0, 0, KiROUND( window.GetWidth() * pxPerIU ), KiROUND( window.GetHeight() * pxPerIU ) );
        want.Intersect( wxRect( 0, 0, image.GetWidth(), image.GetHeight() ) );

        if( !want.IsEmpty() && want.GetSize() != image.GetSize() )
            image = image.GetSubImage( want );

        images.push_back( image );
        names.push_back( LSET::Name( layer ).ToStdString( wxConvUTF8 ) );
    }

    // One crop for all, so the images lie exactly on top of each other
    if( aArgs.value( "crop", true ) )
    {
        // Union of the content boxes of all layers
        int left = INT_MAX, top = INT_MAX, right = -1, bottom = -1;

        for( const wxImage& image : images )
        {
            const unsigned char* data = image.GetData();
            const int            w = image.GetWidth(), h = image.GetHeight();

            for( int y = 0; y < h; ++y )
            {
                for( int x = 0; x < w; ++x )
                {
                    const unsigned char* px = data + 3 * ( (size_t) y * w + x );

                    if( std::abs( px[0] - data[0] ) + std::abs( px[1] - data[1] ) + std::abs( px[2] - data[2] ) > 24 )
                    {
                        left = std::min( left, x );
                        right = std::max( right, x );
                        top = std::min( top, y );
                        bottom = std::max( bottom, y );
                    }
                }
            }
        }

        if( right >= 0 )
        {
            const int margin = dpi / 6;
            left = std::max( 0, left - margin );
            top = std::max( 0, top - margin );
            right = std::min( images.front().GetWidth() - 1, right + margin );
            bottom = std::min( images.front().GetHeight() - 1, bottom + margin );

            for( wxImage& image : images )
                image = image.GetSubImage( wxRect( left, top, right - left + 1, bottom - top + 1 ) );
        }
    }

    nlohmann::json items = nlohmann::json::array();

    for( size_t i = 0; i < images.size(); ++i )
    {
        nlohmann::json item = KopenapiImageResult( images[i] );
        item["layer"] = names[i];
        items.push_back( std::move( item ) );
    }

    return KOPENAPI_RESULT::Ok( { { "dpi", dpi }, { "outline", outline }, { "images", items }, { "estimate", cost.Json() } } );
}


KOPENAPI_REGISTER( "pcb_layer_list",
                   "List the board's enabled layers in display order: KiCad name (F.Cu), user name, kind "
                   "(copper / solder_mask / silkscreen / board_outline / courtyard / fabrication / ...), side, "
                   "visibility; copper layer count",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_layer_list );

KOPENAPI_REGISTER( "pcb_render_layers",
                   "Render board layers one by one to PNG images (one image per layer, the board outline "
                   "drawn on each for orientation, all cropped alike so they overlay exactly): \"copper\", "
                   "\"all\" or a list of layer names from pcb_layer_list; unsaved edits included; over MCP "
                   "several images",
                   R"json({"type":"object","properties":{
                        "layers":{"description":"\"copper\" (default), \"all\" or [\"F.Cu\",\"In1.Cu\",...]"},
                        "dpi":{"type":"integer","default":100,"minimum":30,"maximum":600},
                        "outline":{"type":"boolean","default":true},
                        "crop":{"type":"boolean","default":true},
                        "estimate_only":{"type":"boolean","default":false,"description":"render nothing: the surface, memory and CPU it would take"}}})json"_json,
                   false, h_pcb_render_layers, 600 );

KOPENAPI_REGISTER( "pcb_render",
                   "Render the open board (unsaved edits included) to a PNG image with KiCad's own plotter, "
                   "like `kicad-cli pcb export png`: layers as a list (F.Cu,F.SilkS,Edge.Cuts...) or a preset "
                   "(front, back, copper, assembly); the whole board cropped to the drawing, or a region drawn "
                   "alone at a high dpi for detail; an image over 64 megapixels is refused (422) with what it "
                   "would cost (memory, CPU) and the dpi / region that fit; GUI and headless; over MCP an image",
                   R"json({"type":"object","properties":{
                        "layers":{"type":"string","default":"front","description":"preset front/back/copper/assembly or layer list"},
                        "dpi":{"type":"integer","default":150,"minimum":30,"maximum":25400,"description":"the whole board: keep it low (150); a region: as high as the detail needs (only the region is drawn)"},
                        "crop":{"type":"boolean","default":true},
                        "region":{"type":"array","items":{"type":"number"},"description":"[x0_mm, y0_mm, x1_mm, y1_mm]: only this part of the board is drawn (memory and time follow the region's size at this dpi)"},
                        "estimate_only":{"type":"boolean","default":false,"description":"render nothing: the surface, memory and CPU it would take"},
                        "drawing_sheet":{"type":"boolean","default":false,"description":"the whole page with its drawing sheet (not with region)"},
                        "mirror":{"type":"boolean","default":false},
                        "black_and_white":{"type":"boolean","default":false},
                        "theme":{"type":"string"},
                        "zones":{"type":"string","enum":["filled","outline","hidden"],"default":"filled","description":"outline: each pour drawn as its edge (the tracks and pads under it show); hidden: no pours"}}})json"_json,
                   false, h_pcb_render, 300 );

KOPENAPI_REGISTER( "pcb_render_3d",
                   "Render the open board in 3D (raytraced, with component models), like `kicad-cli pcb "
                   "render`: side top/bottom/left/right/front/back or iso (tilted), size, zoom, rotation, perspective, "
                   "quality basic/high; unsaved edits included; GUI and headless; over MCP an image",
                   R"json({"type":"object","properties":{
                        "side":{"type":"string","enum":["top","bottom","left","right","front","back","iso"],"default":"top"},
                        "width":{"type":"integer","default":1600},"height":{"type":"integer","default":1000},
                        "zoom":{"type":"number","default":1.0},
                        "rotate":{"type":"array","items":{"type":"number"},"description":"[x, y, z] degrees"},
                        "perspective":{"type":"boolean","default":false},
                        "floor":{"type":"boolean","default":false},
                        "quality":{"type":"string","enum":["basic","high"],"default":"basic"}}})json"_json,
                   false, h_pcb_render_3d, 600 );
