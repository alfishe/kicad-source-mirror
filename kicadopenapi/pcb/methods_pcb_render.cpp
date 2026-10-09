/*
 * kicadopenapi board rendering: pcb_render (layers plotted to PNG) and pcb_render_3d (the 3D
 * raytracer).  Both run KiCad's own jobs — the code behind `kicad-cli pcb export png` and
 * `kicad-cli pcb render` — on the live board (unsaved edits included) through
 * PCBNEW_JOBS_HANDLER::SetBoardOverride, in the GUI and headless.  Over MCP the result is an
 * image.
 */
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
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
#include <map>


namespace
{

/// Board jobs on the live board for the duration of a call
struct BOARD_OVERRIDE
{
    explicit BOARD_OVERRIDE( BOARD* aBoard ) { PCBNEW_JOBS_HANDLER::SetBoardOverride( aBoard ); }
    ~BOARD_OVERRIDE() { PCBNEW_JOBS_HANDLER::SetBoardOverride( nullptr ); }
};


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

    const int dpi = std::clamp( aArgs.value( "dpi", 150 ), 30, 1200 );
    BOARD*    board = context->GetBoard();

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

    WX_STRING_REPORTER reporter;
    int                status;

    {
        BOARD_OVERRIDE live( board );
        status = aCtx.kiway->ProcessJob( KIWAY::FACE_PCB, &job, &reporter );
    }

    wxImage         image;
    KOPENAPI_RESULT read = readImage( out, status, reporter.GetMessages(), aArgs.value( "crop", true ), dpi / 6, image );

    if( read.status != 200 )
        return read;

    nlohmann::json result = KopenapiImageResult( image );
    result["layers"] = layers;
    result["dpi"] = dpi;
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
            status = aCtx.kiway->ProcessJob( KIWAY::FACE_PCB, &job, &reporter );
        }

        wxImage         image;
        KOPENAPI_RESULT read = readImage( out, status, reporter.GetMessages(), false, 0, image );

        if( read.status != 200 )
            return read;

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

    return KOPENAPI_RESULT::Ok( { { "dpi", dpi }, { "outline", outline }, { "images", items } } );
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
                        "crop":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_render_layers, 600 );

KOPENAPI_REGISTER( "pcb_render",
                   "Render the open board (unsaved edits included) to a PNG image with KiCad's own plotter, "
                   "like `kicad-cli pcb export png`: layers as a list (F.Cu,F.SilkS,Edge.Cuts...) or a preset "
                   "(front, back, copper, assembly); cropped to the drawing; GUI and headless; over MCP an image",
                   R"json({"type":"object","properties":{
                        "layers":{"type":"string","default":"front","description":"preset front/back/copper/assembly or layer list"},
                        "dpi":{"type":"integer","default":150,"minimum":30,"maximum":1200},
                        "crop":{"type":"boolean","default":true},
                        "drawing_sheet":{"type":"boolean","default":false},
                        "mirror":{"type":"boolean","default":false},
                        "black_and_white":{"type":"boolean","default":false},
                        "theme":{"type":"string"}}})json"_json,
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
