/*
 * kicadopenapi schematic rendering: sch_render.
 *
 * Draws a sheet of the *live* schematic (unsaved edits included) with KiCad's own plotter to
 * PNG — the same renderer as File > Plot / `kicad-cli sch export` — so agents and people can look
 * at the result without screen capture (no OS screen-recording permission involved), in the GUI
 * and headless alike.  By default the page frame is left out and the image is cropped to the
 * drawing.  Over MCP the image is returned as image content.
 */
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kiway.h>
#include <sch_draw_panel.h>
#include <sch_edit_frame.h>
#include <lib_symbol.h>
#include <sch_painter.h>
#include <sch_plotter.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <schematic.h>
#include <schematic_settings.h>
#include <settings/color_settings.h>
#include <settings/settings_manager.h>
#include <plotters/plotter.h>

#include <wx/base64.h>
#include <wx/ffile.h>
#include <wx/filename.h>
#include <wx/image.h>
#include <wx/mstream.h>

#include <algorithm>

using namespace kopenapi_sch;


namespace
{

/// Crop to the bounding box of pixels that differ from the corner (background) colour
wxImage cropToContent( const wxImage& aImage, int aMargin )
{
    const int           w = aImage.GetWidth(), h = aImage.GetHeight();
    const unsigned char* data = aImage.GetData();
    const unsigned char  r = data[0], g = data[1], b = data[2];
    int                  left = w, top = h, right = -1, bottom = -1;

    for( int y = 0; y < h; ++y )
    {
        const unsigned char* row = data + 3 * y * w;

        for( int x = 0; x < w; ++x )
        {
            const unsigned char* px = row + 3 * x;

            if( std::abs( px[0] - r ) + std::abs( px[1] - g ) + std::abs( px[2] - b ) > 24 )
            {
                left = std::min( left, x );
                right = std::max( right, x );
                top = std::min( top, y );
                bottom = std::max( bottom, y );
            }
        }
    }

    if( right < 0 )
        return aImage;

    left = std::max( 0, left - aMargin );
    top = std::max( 0, top - aMargin );
    right = std::min( w - 1, right + aMargin );
    bottom = std::min( h - 1, bottom + aMargin );

    return aImage.GetSubImage( wxRect( left, top, right - left + 1, bottom - top + 1 ) );
}

} // namespace


static KOPENAPI_RESULT h_sch_render( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC*           schematic = context->GetSchematic();
    const SCH_SHEET_LIST hierarchy = schematic->Hierarchy();

    std::optional<SCH_SHEET_PATH> sheet;
    const std::string             wanted = aArgs.value( "sheet", std::string() );

    for( const SCH_SHEET_PATH& path : hierarchy )
    {
        if( wanted.empty() ? !sheet.has_value() : sheetPath( path ) == wanted )
            sheet = path;
    }

    if( !wanted.empty() && !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    if( !sheet && context->GetCurrentSheet() )
        sheet = context->GetCurrentSheet();

    const int  dpi = std::clamp( aArgs.value( "dpi", 150 ), 30, 600 );
    const bool crop = aArgs.value( "crop", true );
    const bool frame = aArgs.value( "drawing_sheet", false );

    // Plot to a temporary file in the OS temp directory
    wxFileName out( wxFileName::CreateTempFileName( wxS( "kopenapi-render" ) ) );
    wxRemoveFile( out.GetFullPath() );
    out.SetExt( wxS( "png" ) );

    auto renderSettings = std::make_unique<SCH_RENDER_SETTINGS>();
    renderSettings->LoadColors( ::GetColorSettings( wxString::FromUTF8( aArgs.value( "theme", std::string( "_builtin_default" ) ) ) ) );
    renderSettings->m_ShowHiddenPins = false;
    renderSettings->m_ShowHiddenFields = false;
    renderSettings->m_ShowPinAltIcons = false;
    renderSettings->SetDefaultPenWidth( schematic->Settings().m_DefaultLineWidth );
    renderSettings->m_LabelSizeRatio = schematic->Settings().m_LabelSizeRatio;
    renderSettings->m_TextOffsetRatio = schematic->Settings().m_TextOffsetRatio;
    renderSettings->m_PinSymbolSize = schematic->Settings().m_PinSymbolSize;
    renderSettings->m_ShowDNPMarkers = schematic->Settings().m_ShowDNPMarkers;
    renderSettings->SetDashLengthRatio( schematic->Settings().m_DashedLineDashRatio );
    renderSettings->SetGapLengthRatio( schematic->Settings().m_DashedLineGapRatio );
    renderSettings->SetDefaultFont( wxEmptyString );

    // Text extents may be cached from another font setup: recompute them for the plot
    SCH_SCREENS screens( schematic->Root() );

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        for( SCH_ITEM* item : screen->Items() )
            item->ClearCaches();

        for( const auto& [name, libSymbol] : screen->GetLibSymbols() )
            libSymbol->ClearCaches();
    }

    SCH_PLOT_OPTS opts;
    opts.m_plotAll = false;
    opts.m_plotDrawingSheet = frame;
    opts.m_sheetPath = sheet;
    opts.m_outputFile = out.GetFullPath();
    opts.m_useBackgroundColor = true;
    opts.m_blackAndWhite = aArgs.value( "black_and_white", false );
    opts.m_pngDPI = dpi;
    opts.m_pngAntialias = true;

    SCH_PLOTTER plotter( schematic );
    plotter.Plot( PLOT_FORMAT::PNG, opts, renderSettings.get(), nullptr );

    wxString file = plotter.GetLastOutputFilePath();

    if( file.IsEmpty() || !wxFileName::FileExists( file ) )
        file = out.GetFullPath();

    if( !wxImage::FindHandler( wxBITMAP_TYPE_PNG ) )
        wxImage::AddHandler( new wxPNGHandler );

    wxImage image;

    if( !wxFileName::FileExists( file ) || !image.LoadFile( file, wxBITMAP_TYPE_PNG ) )
    {
        wxRemoveFile( file );
        return KOPENAPI_RESULT::Error( 500, "plotting the sheet to PNG failed" );
    }

    wxRemoveFile( file );

    if( crop )
        image = cropToContent( image, dpi / 6 );

    wxMemoryOutputStream png;
    image.SaveFile( png, wxBITMAP_TYPE_PNG );

    std::vector<char> bytes( png.GetSize() );
    png.CopyTo( bytes.data(), bytes.size() );

    return KOPENAPI_RESULT::Ok( { { "sheet", sheet ? sheetPath( *sheet ) : std::string() },
                                  { "width", image.GetWidth() },
                                  { "height", image.GetHeight() },
                                  { "dpi", dpi },
                                  { "mime_type", "image/png" },
                                  { "image_base64", str( wxBase64Encode( bytes.data(), bytes.size() ) ) } } );
}


static KOPENAPI_RESULT h_sch_view_capture( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    SCH_EDIT_FRAME* frame = aCtx.kiway ? static_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;

    if( !frame || !frame->GetCanvas() )
        return KOPENAPI_RESULT::Error( 409, "no schematic editor window" );

    // The canvas's own OpenGL buffer: what the editor shows, glow included, without OS screen
    // capture (no screen-recording permission involved)
    wxImage image;

    if( !frame->GetCanvas()->GetScreenshot( image ) || !image.IsOk() )
        return KOPENAPI_RESULT::Error( 500, "the canvas could not be read (not OpenGL, or not drawn yet)" );

    const int maxWidth = std::clamp( aArgs.value( "max_width", 1600 ), 200, 8000 );

    if( image.GetWidth() > maxWidth )
        image.Rescale( maxWidth, image.GetHeight() * maxWidth / image.GetWidth(), wxIMAGE_QUALITY_HIGH );

    if( !wxImage::FindHandler( wxBITMAP_TYPE_PNG ) )
        wxImage::AddHandler( new wxPNGHandler );

    wxMemoryOutputStream png;
    image.SaveFile( png, wxBITMAP_TYPE_PNG );

    std::vector<char> bytes( png.GetSize() );
    png.CopyTo( bytes.data(), bytes.size() );

    return KOPENAPI_RESULT::Ok( { { "width", image.GetWidth() },
                                  { "height", image.GetHeight() },
                                  { "mime_type", "image/png" },
                                  { "image_base64", str( wxBase64Encode( bytes.data(), bytes.size() ) ) } } );
}


KOPENAPI_REGISTER( "sch_view_capture",
                   "Capture what the schematic editor window shows right now (its own canvas: zoom, "
                   "pan, selection and glow as on screen), as a PNG image; GUI only; no OS screen "
                   "capture or permission involved. Over MCP an image",
                   R"json({"type":"object","properties":{
                        "max_width":{"type":"integer","default":1600}}})json"_json,
                   true, h_sch_view_capture );

KOPENAPI_REGISTER( "sch_render",
                   "Render a sheet of the open schematic (unsaved edits included) to a PNG image with "
                   "KiCad's own plotter — look at the schematic without screen capture; works in the "
                   "GUI and headless. Cropped to the drawing without the page frame by default. Over "
                   "MCP the result is an image; over REST base64 in image_base64",
                   R"json({"type":"object","properties":{
                        "sheet":{"type":"string","description":"sheet path from sch_sheet_list; default current/root"},
                        "dpi":{"type":"integer","default":150,"minimum":30,"maximum":600},
                        "crop":{"type":"boolean","default":true,"description":"crop to the drawing"},
                        "drawing_sheet":{"type":"boolean","default":false,"description":"include the page frame and title block"},
                        "black_and_white":{"type":"boolean","default":false},
                        "theme":{"type":"string","description":"color theme name"}}})json"_json,
                   false, h_sch_render, 120 );
