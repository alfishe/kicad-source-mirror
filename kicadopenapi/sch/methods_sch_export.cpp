/// @file methods_sch_export.cpp
/// @brief kicadopenapi sch_export: the open schematic (unsaved edits included) plotted to PDF / SVG /
/// DXF / PostScript and written as the netlist formats, with KiCad's own plotter and exporters, as
/// File > Plot, File > Export > Netlist and `kicad-cli sch export ...`.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <eeschema_jobs_handler.h>
#include <kicadopenapi_registry.h>
#include <lib_symbol.h>
#include <netlist_exporters/netlist_exporter_allegro.h>
#include <netlist_exporters/netlist_exporter_cadstar.h>
#include <netlist_exporters/netlist_exporter_kicad.h>
#include <netlist_exporters/netlist_exporter_orcadpcb2.h>
#include <netlist_exporters/netlist_exporter_pads.h>
#include <netlist_exporters/netlist_exporter_spice.h>
#include <netlist_exporters/netlist_exporter_spice_model.h>
#include <netlist_exporters/netlist_exporter_xml.h>
#include <plotters/plotter.h>
#include <project.h>
#include <reporter.h>
#include <sch_painter.h>
#include <sch_plotter.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <schematic.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <vector>

using namespace kopenapi_sch;


namespace
{

const std::map<std::string, PLOT_FORMAT>& plotFormats()
{
    static const std::map<std::string, PLOT_FORMAT> formats = { { "pdf", PLOT_FORMAT::PDF },
                                                                { "svg", PLOT_FORMAT::SVG },
                                                                { "dxf", PLOT_FORMAT::DXF },
                                                                { "ps", PLOT_FORMAT::POST } };
    return formats;
}


struct NETLIST_FORMAT
{
    const char* ext;
    unsigned    options;
    std::function<std::unique_ptr<NETLIST_EXPORTER_BASE>( SCHEMATIC*, KIWAY* )> make;
};


template <typename T>
std::function<std::unique_ptr<NETLIST_EXPORTER_BASE>( SCHEMATIC*, KIWAY* )> maker()
{
    return []( SCHEMATIC* aSch, KIWAY* aKiway ) { return std::make_unique<T>( aSch, aKiway ); };
}


/// The netlist formats of File > Export > Netlist and `kicad-cli sch export netlist`
const std::map<std::string, NETLIST_FORMAT>& netlistFormats()
{
    static const std::map<std::string, NETLIST_FORMAT> formats = {
        { "netlist_kicad", { "net", 0, maker<NETLIST_EXPORTER_KICAD>() } },
        { "netlist_xml", { "xml", 0, maker<NETLIST_EXPORTER_XML>() } },
        { "netlist_orcadpcb2", { "net", 0, maker<NETLIST_EXPORTER_ORCADPCB2>() } },
        { "netlist_cadstar", { "frp", 0, maker<NETLIST_EXPORTER_CADSTAR>() } },
        { "netlist_spice", { "cir", NETLIST_EXPORTER_SPICE::OPTION_SIM_COMMAND, maker<NETLIST_EXPORTER_SPICE>() } },
        { "netlist_spice_model", { "cir", 0, maker<NETLIST_EXPORTER_SPICE_MODEL>() } },
        { "netlist_pads", { "asc", 0, maker<NETLIST_EXPORTER_PADS>() } },
        { "netlist_allegro", { "txt", 0, maker<NETLIST_EXPORTER_ALLEGRO>() } } };
    return formats;
}


/// KiCad's messages of one format, the errors marked
class COLLECT_REPORTER : public REPORTER
{
public:
    REPORTER& Report( const wxString& aText, SEVERITY aSeverity = RPT_SEVERITY_UNDEFINED ) override
    {
        wxString text = aText;
        text.Trim();

        if( !text.IsEmpty() )
            m_messages.push_back( text.ToStdString( wxConvUTF8 ) );

        if( aSeverity == RPT_SEVERITY_ERROR )
            m_error = true;

        return *this;
    }

    bool HasMessage() const override { return !m_messages.empty(); }

    nlohmann::json m_messages = nlohmann::json::array();
    bool           m_error = false;
};


nlohmann::json fileJson( const wxString& aPath )
{
    wxFileName fn( aPath );
    return { { "path", str( fn.GetFullPath() ) },
             { "bytes", fn.FileExists() ? static_cast<long long>( fn.GetSize().GetValue() ) : 0LL } };
}


KOPENAPI_RESULT h_sch_export( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC* schematic = context->GetSchematic();

    std::vector<std::string> wanted;

    if( aArgs.contains( "formats" ) && aArgs["formats"].is_string() )
        wanted.push_back( aArgs["formats"].get<std::string>() );
    else if( aArgs.contains( "formats" ) && aArgs["formats"].is_array() )
    {
        for( const nlohmann::json& f : aArgs["formats"] )
            wanted.push_back( f.is_string() ? f.get<std::string>() : f.dump() );
    }

    if( wanted.empty() )
        return KOPENAPI_RESULT::Error( 400, "formats: pdf, svg, dxf, ps, netlist_kicad, netlist_xml, netlist_orcadpcb2, "
                                            "netlist_cadstar, netlist_spice, netlist_spice_model, netlist_pads, "
                                            "netlist_allegro (one or a list)" );

    for( const std::string& f : wanted )
    {
        if( !plotFormats().count( f ) && !netlistFormats().count( f ) )
            return KOPENAPI_RESULT::Error( 400, "formats: unknown format '" + f + "'" );
    }

    std::optional<SCH_SHEET_PATH> sheet;

    if( aArgs.contains( "sheet" ) )
    {
        sheet = KopenapiSchTargetSheet( *context, aArgs );

        if( !sheet )
            return KOPENAPI_RESULT::Error( 404, "sheet: not found (sch_sheet_list)" );
    }

    const wxFileName schFile( schematic->Project().AbsolutePath( schematic->Root().GetFileName() ) );
    wxFileName       dir = wxFileName::DirName( wxString::FromUTF8( aArgs.value( "dir", std::string( "export" ) ) ) );

    if( !dir.IsAbsolute() )
        dir.MakeAbsolute( wxFileName( schematic->Project().GetProjectFullName() ).GetPath() );

    if( !dir.DirExists() && !wxFileName::Mkdir( dir.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
        return KOPENAPI_RESULT::Error( 422, "dir: cannot create " + str( dir.GetPath() ) );

    const wxString baseName = wxFileName( schematic->Project().GetProjectFullName() ).GetName();
    nlohmann::json results = nlohmann::json::object();
    bool           failed = false;
    bool           connected = false;

    for( const std::string& format : wanted )
    {
        COLLECT_REPORTER reporter;
        nlohmann::json   files = nlohmann::json::array();

        if( auto plot = plotFormats().find( format ); plot != plotFormats().end() )
        {
            auto renderSettings = std::make_unique<SCH_RENDER_SETTINGS>();
            EESCHEMA_JOBS_HANDLER::InitPlotRenderSettings(
                    renderSettings.get(), wxString::FromUTF8( aArgs.value( "theme", std::string() ) ), schematic,
                    wxEmptyString, reporter );
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
            opts.m_plotAll = !sheet;
            opts.m_sheetPath = sheet;
            opts.m_plotDrawingSheet = aArgs.value( "drawing_sheet", true );
            opts.m_blackAndWhite = aArgs.value( "black_and_white", false );
            opts.m_useBackgroundColor = aArgs.value( "background", false );
            opts.m_PDFHierarchicalLinks = aArgs.value( "pdf_hierarchical_links", true );
            opts.m_PDFPropertyPopups = aArgs.value( "pdf_property_popups", true );
            opts.m_PDFMetadata = aArgs.value( "pdf_metadata", true );
            opts.m_theme = wxString::FromUTF8( aArgs.value( "theme", std::string() ) );
            opts.m_DXF_File_Unit = DXF_UNITS::MM;
            opts.m_outputDirectory = dir.GetPath();

            SCH_PLOTTER plotter( schematic );
            plotter.Plot( plot->second, opts, renderSettings.get(), &reporter );

            for( const wxString& path : plotter.GetOutputFilePaths() )
                files.push_back( fileJson( path ) );

            if( files.empty() )
                reporter.m_error = true;
        }
        else
        {
            if( !connected )
            {
                schematic->RebuildConnectivity();
                ensureConnectivity( schematic );
                connected = true;
            }

            const NETLIST_FORMAT& netlist = netlistFormats().at( format );
            wxFileName            out( dir.GetPath(), baseName, wxString::FromUTF8( netlist.ext ) );

            // formats sharing an extension with the main one get the format in the name
            if( format == "netlist_orcadpcb2" || format == "netlist_spice_model" )
                out.SetName( baseName + wxS( "-" ) + wxString::FromUTF8( format.substr( 8 ) ) );

            std::unique_ptr<NETLIST_EXPORTER_BASE> exporter = netlist.make( schematic, aCtx.kiway );

            if( exporter->WriteNetlist( out.GetFullPath(), netlist.options, reporter ) && out.FileExists() )
                files.push_back( fileJson( out.GetFullPath() ) );
            else
                reporter.m_error = true;
        }

        nlohmann::json entry = { { "ok", !reporter.m_error }, { "files", files } };

        if( !reporter.m_messages.empty() )
            entry["messages"] = reporter.m_messages;

        failed |= reporter.m_error;
        results[format] = entry;
    }

    nlohmann::json out = { { "dir", str( dir.GetPath() ) }, { "formats", results } };

    if( sheet )
        out["sheet"] = sheetPath( *sheet );

    if( failed )
    {
        KOPENAPI_RESULT error = KOPENAPI_RESULT::Error( 422, "a format failed (see formats)" );
        error.body["dir"] = out["dir"];
        error.body["formats"] = results;
        return error;
    }

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "sch_export",
                   "Documentation outputs of the open schematic (unsaved edits included) with KiCad's own plotter "
                   "and netlist exporters, as File > Plot, File > Export > Netlist and `kicad-cli sch export ...`: "
                   "plots pdf (one file, every sheet; hierarchical links, property popups and metadata by default), "
                   "svg / dxf / ps (one file per sheet), with sheet only that sheet; netlists netlist_kicad, "
                   "netlist_xml, netlist_orcadpcb2, netlist_cadstar, netlist_spice, netlist_spice_model, "
                   "netlist_pads, netlist_allegro. Into dir (default 'export' beside the project; relative to the "
                   "project's folder), files named after the project / sheets. theme: a color theme name "
                   "(default the editor's), black_and_white, background, drawing_sheet (the page frame, default "
                   "true). Answers per format ok, the files with their size and KiCad's messages; 422 when a format "
                   "failed (the others are written). sch_render gives a PNG image instead",
                   R"json({"type":"object","required":["formats"],"properties":{
                        "formats":{"type":["string","array"],"items":{"type":"string"},"description":"pdf, svg, dxf, ps, netlist_kicad, netlist_xml, netlist_orcadpcb2, netlist_cadstar, netlist_spice, netlist_spice_model, netlist_pads, netlist_allegro"},
                        "dir":{"type":"string","default":"export"},
                        "sheet":{"type":"string","description":"plots: this sheet only (a path of sch_sheet_list)"},
                        "theme":{"type":"string"},
                        "black_and_white":{"type":"boolean","default":false},
                        "background":{"type":"boolean","default":false},
                        "drawing_sheet":{"type":"boolean","default":true},
                        "pdf_hierarchical_links":{"type":"boolean","default":true},
                        "pdf_property_popups":{"type":"boolean","default":true},
                        "pdf_metadata":{"type":"boolean","default":true}}})json"_json,
                   false, h_sch_export, 600 );
