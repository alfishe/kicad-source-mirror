/// @file methods_pcb_export.cpp
/// @brief kicadopenapi fabrication outputs: pcb_export (Gerbers, drill, positions, IPC-2581, ODB++,
/// IPC-D-356, STEP / GLB, GenCAD, IDF) runs KiCad's own jobs - the code behind `kicad-cli pcb
/// export ...` - on the live board (unsaved edits included) through
/// PCBNEW_JOBS_HANDLER::SetBoardOverride; pcb_origin_set sets the drill / place file origin and the
/// grid origin.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <exporters/export_footprint_associations.h>
#include <exporters/export_hyperlynx.h>
#include <frame_type.h>
#include <gal/graphics_abstraction_layer.h>
#include <geometry/shape_poly_set.h>
#include <jobs/job_export_pcb_3d.h>
#include <jobs/job_export_pcb_drill.h>
#include <jobs/job_export_pcb_gencad.h>
#include <jobs/job_export_pcb_gerbers.h>
#include <jobs/job_export_pcb_idf.h>
#include <jobs/job_export_pcb_ipc2581.h>
#include <jobs/job_export_pcb_ipcd356.h>
#include <jobs/job_export_pcb_odb.h>
#include <jobs/job_export_pcb_pos.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <pcbnew_jobs_handler.h>
#include <reporter.h>
#include <tool/tool_manager.h>
#include <wildcards_and_files_ext.h>
#include <zone.h>
#include <zone_filler.h>

#include <wx/filename.h>
#include <wx/tokenzr.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


struct BOARD_OVERRIDE
{
    explicit BOARD_OVERRIDE( BOARD* aBoard ) { PCBNEW_JOBS_HANDLER::SetBoardOverride( aBoard ); }
    ~BOARD_OVERRIDE() { PCBNEW_JOBS_HANDLER::SetBoardOverride( nullptr ); }
};


const std::vector<std::string> EXPORT_FORMATS = { "gerbers", "drill", "pos",    "ipc2581", "odb",  "ipcd356",
                                                  "step",    "stepz", "glb",    "brep",    "xao",  "ply",
                                                  "stl",     "u3d",   "pdf_3d", "vrml",    "gencad", "idf",
                                                  "hyperlynx", "cmp" };


/// 3D formats of the STEP / VRML exporters: job format, file extension
const std::map<std::string, std::pair<JOB_EXPORT_PCB_3D::FORMAT, std::string>> FORMATS_3D = {
    { "step", { JOB_EXPORT_PCB_3D::FORMAT::STEP, "step" } },
    { "stepz", { JOB_EXPORT_PCB_3D::FORMAT::STEPZ, "stpz" } },
    { "glb", { JOB_EXPORT_PCB_3D::FORMAT::GLB, "glb" } },
    { "brep", { JOB_EXPORT_PCB_3D::FORMAT::BREP, "brep" } },
    { "xao", { JOB_EXPORT_PCB_3D::FORMAT::XAO, "xao" } },
    { "ply", { JOB_EXPORT_PCB_3D::FORMAT::PLY, "ply" } },
    { "stl", { JOB_EXPORT_PCB_3D::FORMAT::STL, "stl" } },
    { "u3d", { JOB_EXPORT_PCB_3D::FORMAT::U3D, "u3d" } },
    { "pdf_3d", { JOB_EXPORT_PCB_3D::FORMAT::PDF, "pdf" } },
    { "vrml", { JOB_EXPORT_PCB_3D::FORMAT::VRML, "wrl" } }
};


/// @brief The copper pours filled again (unfilled ones too) in one undo step
void fillCopperZones( PCB_CONTEXT& aContext, BOARD* aBoard )
{
    std::vector<ZONE*> zones;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( !zone->GetIsRuleArea() && !zone->IsTeardropArea() && zone->IsOnCopperLayer() )
            zones.push_back( zone );
    }

    if( zones.empty() )
        return;

    BOARD_COMMIT                commit( aContext.GetToolManager() );
    ZONE_FILLER                 filler( aBoard, &commit );
    KOPENAPI_KEEPALIVE_REPORTER reporter;
    filler.SetProgressReporter( &reporter );

    if( filler.Fill( zones ) )
        commit.Push( _( "Fill zones (API)" ) );
    else
        commit.Revert();
}


/// @brief The job of one format, set up for aDir / aBase; nullptr for an unknown format
std::unique_ptr<JOB> makeJob( const std::string& aFormat, const nlohmann::json& aArgs, const wxString& aDir,
                              const wxString& aBase, BOARD* aBoard, std::string& aError )
{
    const bool     drillOrigin = aArgs.value( "use_drill_origin", false );
    const wxString file = aDir + wxFileName::GetPathSeparator() + aBase;

    if( aFormat == "gerbers" )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_GERBERS>();
        job->m_useDrillOrigin = drillOrigin;
        job->m_createJobsFile = true;
        job->m_precision = 6;

        if( aArgs.contains( "layers" ) )
        {
            wxString layers;

            for( const nlohmann::json& name : aArgs["layers"] )
            {
                if( aBoard->GetLayerID( wxString::FromUTF8( name.get<std::string>() ) ) < 0 )
                {
                    aError = "layers: no layer " + name.get<std::string>() + " on this board (pcb_layer_list)";
                    return nullptr;
                }

                layers += ( layers.IsEmpty() ? "" : "," ) + wxString::FromUTF8( name.get<std::string>() );
            }

            job->m_argLayers = layers;
        }

        job->SetConfiguredOutputPath( aDir );
        return job;
    }

    if( aFormat == "drill" )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_DRILL>();
        const std::string format = aArgs.value( "drill_format", std::string( "excellon" ) );
        job->m_format = format == "gerber" ? JOB_EXPORT_PCB_DRILL::DRILL_FORMAT::GERBER
                                           : JOB_EXPORT_PCB_DRILL::DRILL_FORMAT::EXCELLON;
        job->m_drillOrigin = drillOrigin ? JOB_EXPORT_PCB_DRILL::DRILL_ORIGIN::PLOT : JOB_EXPORT_PCB_DRILL::DRILL_ORIGIN::ABS;
        job->m_drillUnits = aArgs.value( "units", std::string( "mm" ) ) == "in" ? JOB_EXPORT_PCB_DRILL::DRILL_UNITS::INCH
                                                                              : JOB_EXPORT_PCB_DRILL::DRILL_UNITS::MM;
        job->m_excellonCombinePTHNPTH = aArgs.value( "drill_merge_pth_npth", false );
        job->m_generateMap = aArgs.value( "drill_map", false );
        job->m_mapFormat = JOB_EXPORT_PCB_DRILL::MAP_FORMAT::PDF;
        job->SetConfiguredOutputPath( aDir );
        return job;
    }

    if( aFormat == "pos" )
    {
        auto              job = std::make_unique<JOB_EXPORT_PCB_POS>();
        const std::string format = aArgs.value( "pos_format", std::string( "csv" ) );
        const std::string side = aArgs.value( "pos_side", std::string( "both" ) );
        job->m_format = format == "ascii"    ? JOB_EXPORT_PCB_POS::FORMAT::ASCII
                        : format == "gerber" ? JOB_EXPORT_PCB_POS::FORMAT::GERBER
                                             : JOB_EXPORT_PCB_POS::FORMAT::CSV;
        job->m_side = side == "front" ? JOB_EXPORT_PCB_POS::SIDE::FRONT
                      : side == "back" ? JOB_EXPORT_PCB_POS::SIDE::BACK
                                       : JOB_EXPORT_PCB_POS::SIDE::BOTH;
        job->m_units = aArgs.value( "units", std::string( "mm" ) ) == "in" ? JOB_EXPORT_PCB_POS::UNITS::INCH
                                                                         : JOB_EXPORT_PCB_POS::UNITS::MM;
        job->m_useDrillPlaceFileOrigin = drillOrigin;
        job->m_smdOnly = aArgs.value( "pos_smd_only", false );
        job->m_excludeDNP = aArgs.value( "pos_exclude_dnp", false );
        job->m_singleFile = aArgs.value( "pos_single_file", false );
        job->SetConfiguredOutputPath( file + "."
                                      + ( format == "ascii"    ? wxString( FILEEXT::FootprintPlaceFileExtension )
                                          : format == "gerber" ? wxString( FILEEXT::GerberFileExtension )
                                                               : wxString( FILEEXT::CsvFileExtension ) ) );
        return job;
    }

    if( aFormat == "ipc2581" )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_IPC2581>();
        job->SetConfiguredOutputPath( file + "." + FILEEXT::Ipc2581FileExtension );
        return job;
    }

    if( aFormat == "odb" )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_ODB>();
        job->m_compressionMode = JOB_EXPORT_PCB_ODB::ODB_COMPRESSION::ZIP;
        job->m_origin = drillOrigin ? JOB_EXPORT_PCB_ODB::ORIGIN::AUX : JOB_EXPORT_PCB_ODB::ORIGIN::ABSOLUTE_COORDS;
        job->SetConfiguredOutputPath( file + "-odb." + FILEEXT::ArchiveFileExtension );
        return job;
    }

    if( aFormat == "ipcd356" )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_IPCD356>();
        job->SetConfiguredOutputPath( file + "." + FILEEXT::IpcD356FileExtension );
        return job;
    }

    if( auto it3d = FORMATS_3D.find( aFormat ); it3d != FORMATS_3D.end() )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_3D>();
        job->m_format = it3d->second.first;
        job->m_3dparams.m_Overwrite = true;
        job->m_3dparams.m_UseDrillOrigin = drillOrigin;
        job->m_3dparams.m_ExportComponents = aArgs.value( "step_components", true );
        job->m_3dparams.m_IncludeDNP = aArgs.value( "step_include_dnp", false );
        job->m_3dparams.m_IncludeUnspecified = aArgs.value( "include_unspecified", true );

        if( aFormat == "vrml" )
        {
            static const std::map<std::string, JOB_EXPORT_PCB_3D::VRML_UNITS> units = {
                { "mm", JOB_EXPORT_PCB_3D::VRML_UNITS::MM },
                { "m", JOB_EXPORT_PCB_3D::VRML_UNITS::METERS },
                { "in", JOB_EXPORT_PCB_3D::VRML_UNITS::INCH },
                { "tenths", JOB_EXPORT_PCB_3D::VRML_UNITS::TENTHS }
            };

            auto unit = units.find( aArgs.value( "vrml_units", std::string( "m" ) ) );

            if( unit == units.end() )
            {
                aError = "vrml_units: mm, m, in or tenths";
                return nullptr;
            }

            job->m_vrmlUnits = unit->second;
            job->m_vrmlRelativePaths = aArgs.value( "vrml_relative_paths", false );

            if( aArgs.contains( "vrml_models_dir" ) )
            {
                wxFileName models = wxFileName::DirName( wxString::FromUTF8( aArgs.value( "vrml_models_dir",
                                                                                          std::string() ) ) );

                if( !models.IsAbsolute() )
                    models.MakeAbsolute( aDir );

                job->m_vrmlModelDir = models.GetPath();
            }

            // the dialog's origin: the board's centre, the drill / place file origin, or a point
            if( aArgs.contains( "vrml_origin_mm" ) )
            {
                const nlohmann::json& o = aArgs["vrml_origin_mm"];

                if( !o.is_array() || o.size() != 2 || !o[0].is_number() || !o[1].is_number() )
                {
                    aError = "vrml_origin_mm: [x, y]";
                    return nullptr;
                }

                job->m_hasUserOrigin = true;
                job->m_3dparams.m_Origin = VECTOR2D( pcbIUScale.mmToIU( o[0].get<double>() ),
                                                     pcbIUScale.mmToIU( o[1].get<double>() ) );
            }
            else if( drillOrigin )
            {
                job->m_hasUserOrigin = true;
                job->m_3dparams.m_Origin = VECTOR2D( aBoard->GetDesignSettings().GetAuxOrigin() );
            }
        }

        job->SetConfiguredOutputPath( file + "." + wxString::FromUTF8( it3d->second.second ) );
        return job;
    }

    if( aFormat == "gencad" )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_GENCAD>();
        job->m_useDrillOrigin = drillOrigin;
        job->SetConfiguredOutputPath( file + "." + FILEEXT::GencadFileExtension );
        return job;
    }

    if( aFormat == "idf" )
    {
        auto job = std::make_unique<JOB_EXPORT_PCB_IDF>();
        job->SetConfiguredOutputPath( file + "." + FILEEXT::IdfV3BoardFileExtension );
        return job;
    }

    aError = "formats: unknown format " + aFormat;
    return nullptr;
}


/// @brief Files a job wrote: its outputs, else the file it was told to write and its siblings by
/// name (IDF writes .emn and .emp)
std::vector<wxString> writtenFiles( JOB& aJob, const std::string& aFormat )
{
    std::vector<wxString> out;

    for( const JOB_OUTPUT& output : aJob.GetOutputs() )
        out.push_back( output.m_outputPath );

    if( out.empty() )
    {
        wxFileName fn( aJob.GetConfiguredOutputPath() );

        if( fn.FileExists() )
            out.push_back( fn.GetFullPath() );

        if( aFormat == "idf" )
        {
            fn.SetExt( wxS( "emp" ) );

            if( fn.FileExists() )
                out.push_back( fn.GetFullPath() );
        }
    }

    return out;
}


KOPENAPI_RESULT h_pcb_export( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    std::vector<std::string> formats;

    if( !aArgs.contains( "formats" ) )
        return KOPENAPI_RESULT::Error( 400, "formats: e.g. [\"gerbers\", \"drill\", \"pos\"] or \"fab\" for those three" );

    for( const nlohmann::json& f : aArgs["formats"].is_array() ? aArgs["formats"] : nlohmann::json::array( { aArgs["formats"] } ) )
    {
        const std::string name = f.is_string() ? f.get<std::string>() : std::string();

        if( name == "fab" )
        {
            formats.insert( formats.end(), { "gerbers", "drill", "pos" } );
            continue;
        }

        if( std::find( EXPORT_FORMATS.begin(), EXPORT_FORMATS.end(), name ) == EXPORT_FORMATS.end() )
        {
            std::string known;

            for( const std::string& k : EXPORT_FORMATS )
                known += ( known.empty() ? "" : ", " ) + k;

            return KOPENAPI_RESULT::Error( 400, "formats: unknown format '" + name + "' (" + known + ", fab)" );
        }

        if( std::find( formats.begin(), formats.end(), name ) == formats.end() )
            formats.push_back( name );
    }

    if( formats.empty() )
        return KOPENAPI_RESULT::Error( 400, "formats: none given" );

    wxFileName boardFile( board->GetFileName() );

    if( boardFile.GetFullPath().IsEmpty() )
        return KOPENAPI_RESULT::Error( 409, "the board has no file yet: pcb_save it with a path first" );

    wxFileName dir = wxFileName::DirName( wxString::FromUTF8( aArgs.value( "dir", std::string( "fab" ) ) ) );

    if( !dir.IsAbsolute() )
        dir.MakeAbsolute( boardFile.GetPath() );

    const wxString outDir = dir.GetPath();

    if( !wxFileName::DirExists( outDir ) && !wxFileName::Mkdir( outDir, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
        return KOPENAPI_RESULT::Error( 422, "dir: cannot create " + str( outDir ) );

    // fab data from stale or missing pour fills would be wrong copper
    nlohmann::json    stale = nlohmann::json::array();
    const std::string fills = KopenapiZoneFillState( board, &stale );
    nlohmann::json    out = { { "dir", str( outDir ) }, { "zone_fills", fills } };

    if( fills == "unfilled" || fills == "stale" )
    {
        if( !aArgs.value( "refill_zones", true ) )
        {
            KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "zone fills are not current: the outputs would not show the "
                                                                   "copper the board has; pcb_zones_fill first, or "
                                                                   "refill_zones: true" );
            refused.body["stale_zones"] = stale;
            return refused;
        }

        fillCopperZones( *context, board );
        out["zones_refilled"] = true;
        out["zone_fills"] = KopenapiZoneFillState( board );
    }

    if( aArgs.value( "use_drill_origin", false ) )
    {
        const VECTOR2I origin = board->GetDesignSettings().GetAuxOrigin();
        out["origin_mm"] = { toMm( origin.x ), toMm( origin.y ) };
    }

    nlohmann::json results = nlohmann::json::object();
    nlohmann::json failed = nlohmann::json::array();
    int            fileCount = 0;

    for( const std::string& format : formats )
    {
        if( format == "hyperlynx" || format == "cmp" )
        {
            // File > Export > Hyperlynx / Footprint Association (.cmp): the editor's exporters
            wxFileName fn( outDir, boardFile.GetName(),
                           format == "cmp" ? wxString( FILEEXT::FootprintAssignmentFileExtension ) : wxString( wxS( "hyp" ) ) );
            const bool done = format == "cmp" ? RecreateCmpFile( board, fn.GetFullPath() )
                                              : ExportBoardToHyperlynxFile( board, fn.GetFullPath() );
            nlohmann::json files = nlohmann::json::array();

            if( fn.FileExists() )
                files.push_back( { { "path", str( fn.GetFullPath() ) }, { "bytes", (long long) fn.GetSize().GetValue() } } );

            const bool ok = done && !files.empty();
            fileCount += files.size();
            results[format] = { { "ok", ok }, { "files", files } };

            if( !ok )
                failed.push_back( format );

            continue;
        }

        std::string          error;
        std::unique_ptr<JOB> job = makeJob( format, aArgs, outDir, boardFile.GetName(), board, error );

        if( !job )
            return KOPENAPI_RESULT::Error( 400, error );

        WX_STRING_REPORTER reporter;
        int                status;

        {
            BOARD_OVERRIDE live( board );
            status = aCtx.kiway->ProcessJob( KIWAY::FACE_PCB, job.get(), &reporter );
        }

        nlohmann::json files = nlohmann::json::array();

        for( const wxString& path : writtenFiles( *job, format ) )
        {
            wxFileName fn( path );

            if( !fn.FileExists() )
                continue;

            files.push_back( { { "path", str( fn.GetFullPath() ) }, { "bytes", (long long) fn.GetSize().GetValue() } } );
        }

        nlohmann::json messages = nlohmann::json::array();
        wxStringTokenizer lines( reporter.GetMessages(), "\n" );

        while( lines.HasMoreTokens() && messages.size() < 20 )
        {
            wxString line = lines.GetNextToken().Trim().Trim( false );

            if( !line.IsEmpty() )
                messages.push_back( str( line ) );
        }

        const bool ok = status == 0 && !files.empty();
        fileCount += files.size();
        results[format] = { { "ok", ok }, { "files", files } };

        if( !messages.empty() )
            results[format]["messages"] = messages;

        if( !ok )
        {
            results[format]["exit_code"] = status;
            failed.push_back( format );
        }
    }

    out["formats"] = results;
    out["file_count"] = fileCount;

    if( !failed.empty() )
    {
        KOPENAPI_RESULT partial = KOPENAPI_RESULT::Error( 422, "export failed for " + failed.dump() + " (see formats.<name>.messages)" );
        partial.body.update( out );
        partial.body["failed"] = failed;
        return partial;
    }

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT h_pcb_origin_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                 board = context->GetBoard();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();

    auto point = [&]( const char* aKey, std::optional<VECTOR2I>& aOut ) -> std::optional<KOPENAPI_RESULT>
    {
        if( !aArgs.contains( aKey ) )
            return std::nullopt;

        const nlohmann::json& p = aArgs[aKey];

        if( p.is_null() )
        {
            aOut = VECTOR2I( 0, 0 );
            return std::nullopt;
        }

        if( !p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number() )
            return KOPENAPI_RESULT::Error( 400, std::string( aKey ) + ": [x, y] in mm, or null for 0, 0" );

        aOut = VECTOR2I( pcbIUScale.mmToIU( p[0].get<double>() ), pcbIUScale.mmToIU( p[1].get<double>() ) );
        return std::nullopt;
    };

    std::optional<VECTOR2I> drill, grid;

    if( std::optional<KOPENAPI_RESULT> bad = point( "drill_mm", drill ) )
        return *bad;

    if( std::optional<KOPENAPI_RESULT> bad = point( "grid_mm", grid ) )
        return *bad;

    if( aArgs.value( "drill_at_outline", false ) )
    {
        if( drill )
            return KOPENAPI_RESULT::Error( 400, "give drill_mm or drill_at_outline, not both" );

        SHAPE_POLY_SET outline;

        if( !board->GetBoardPolygonOutlines( outline, false ) || outline.OutlineCount() == 0 )
            return KOPENAPI_RESULT::Error( 422, "drill_at_outline: the board has no closed outline (pcb_outline_set)" );

        const BOX2I edges = outline.BBox();

        drill = VECTOR2I( edges.GetLeft(), edges.GetBottom() );
    }

    if( drill && *drill != bds.GetAuxOrigin() )
    {
        bds.SetAuxOrigin( *drill );
        context->SetContentModified();
    }

    if( grid && *grid != bds.GetGridOrigin() )
    {
        bds.SetGridOrigin( *grid );
        context->SetContentModified();
    }

    if( PCB_EDIT_FRAME* frame = static_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
    {
        if( grid )
            frame->GetCanvas()->GetGAL()->SetGridOrigin( VECTOR2D( *grid ) );

        frame->GetCanvas()->Refresh();
    }

    const VECTOR2I aux = bds.GetAuxOrigin(), gridOrigin = bds.GetGridOrigin();
    return KOPENAPI_RESULT::Ok( { { "drill_mm", { toMm( aux.x ), toMm( aux.y ) } },
                                  { "grid_mm", { toMm( gridOrigin.x ), toMm( gridOrigin.y ) } } } );
}

} // namespace


KOPENAPI_REGISTER( "pcb_export",
                   "Fabrication outputs of the open board (unsaved edits included) with KiCad's own exporters, as "
                   "File > Fabrication Outputs / Export and `kicad-cli pcb export ...`: formats gerbers (every "
                   "enabled layer, or layers; with the job file), drill (excellon or gerber, drill_map PDF), pos "
                   "(component placement: csv / ascii / gerber, pos_side, pos_smd_only, pos_exclude_dnp), ipc2581, "
                   "odb (ODB++ zip), ipcd356, the 3D exports step, stepz, glb, brep, xao, ply, stl, u3d, pdf_3d "
                   "and vrml (vrml_units, default m; vrml_models_dir: copy the models there, vrml_relative_paths; "
                   "origin the board centre, use_drill_origin or vrml_origin_mm), gencad, idf, hyperlynx (.hyp), "
                   "cmp (footprint associations); \"fab\" = gerbers + drill + pos. Into dir "
                   "(default 'fab' beside the board; relative to the board's folder), files named after the board. "
                   "use_drill_origin: coordinates from the drill / place file origin (pcb_origin_set). Stale or "
                   "unfilled pours are refilled first (refill_zones: false refuses, 409). Answers per format ok, "
                   "the files with their size and KiCad's messages; 422 when a format failed (the others are written)",
                   R"json({"type":"object","required":["formats"],"properties":{
                        "formats":{"type":["string","array"],"items":{"type":"string"},"description":"a format or a list: gerbers, drill, pos, ipc2581, odb, ipcd356, step, stepz, glb, brep, xao, ply, stl, u3d, pdf_3d, vrml, gencad, idf, hyperlynx, cmp, fab"},
                        "dir":{"type":"string","default":"fab"},
                        "use_drill_origin":{"type":"boolean","default":false},
                        "layers":{"type":"array","items":{"type":"string"},"description":"gerbers: these layers only (F.Cu, F.Mask, Edge.Cuts...)"},
                        "units":{"type":"string","enum":["mm","in"],"default":"mm","description":"drill and pos"},
                        "drill_format":{"type":"string","enum":["excellon","gerber"],"default":"excellon"},
                        "drill_map":{"type":"boolean","default":false},
                        "drill_merge_pth_npth":{"type":"boolean","default":false},
                        "pos_format":{"type":"string","enum":["csv","ascii","gerber"],"default":"csv"},
                        "pos_side":{"type":"string","enum":["both","front","back"],"default":"both"},
                        "pos_single_file":{"type":"boolean","default":false},
                        "pos_smd_only":{"type":"boolean","default":false},
                        "pos_exclude_dnp":{"type":"boolean","default":false},
                        "step_components":{"type":"boolean","default":true},
                        "step_include_dnp":{"type":"boolean","default":false},
                        "include_unspecified":{"type":"boolean","default":true,"description":"3D: footprints of unspecified type"},
                        "vrml_units":{"type":"string","enum":["mm","m","in","tenths"],"default":"m"},
                        "vrml_models_dir":{"type":"string","description":"copy the 3D models into this folder (relative to dir)"},
                        "vrml_relative_paths":{"type":"boolean","default":false},
                        "vrml_origin_mm":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},
                        "refill_zones":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_export, 600 );

KOPENAPI_REGISTER( "pcb_origin_set",
                   "Set the drill / place file origin (Place > Drill/Place File Origin; the origin pcb_export "
                   "use_drill_origin and the fab outputs measure from) and the grid origin: drill_mm / grid_mm [x, y], "
                   "null resets to 0, 0; drill_at_outline: the bottom-left corner of the board outline. Saved with "
                   "the board. Answers both origins",
                   R"json({"type":"object","properties":{
                        "drill_mm":{"type":["array","null"],"items":{"type":"number"}},
                        "grid_mm":{"type":["array","null"],"items":{"type":"number"}},
                        "drill_at_outline":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_origin_set );

KOPENAPI_MARK_EDITING( "pcb_export", "pcb_origin_set" );
