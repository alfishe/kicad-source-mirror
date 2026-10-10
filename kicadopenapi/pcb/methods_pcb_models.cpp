/// @file methods_pcb_models.cpp
/// @brief kicadopenapi 3D models of board footprints: pcb_footprint_model_set, pcb_models_inspect,
/// pcb_model_seat.
#include "kopenapi_pcb.h"
#include "kopenapi_occ.h"

#include <3d_math.h>
#include <board_design_settings.h>
#include <common.h>
#include <kicadopenapi_keepalive.h>
#include <pad.h>
#include <project.h>
#include <trigo.h>

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <frame_type.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <tool/tool_manager.h>

#include <cmath>
#include <filesystem>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <future>
#include <set>
#include <sstream>
#include <thread>
#include <tuple>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


bool vec3( const nlohmann::json& aModel, const char* aKey, VECTOR3D& aOut )
{
    if( !aModel.contains( aKey ) )
        return true;

    const nlohmann::json& v = aModel[aKey];

    if( !v.is_array() || v.size() != 3 || !v[0].is_number() || !v[1].is_number() || !v[2].is_number() )
        return false;

    aOut = VECTOR3D( v[0].get<double>(), v[1].get<double>(), v[2].get<double>() );
    return true;
}


nlohmann::json modelsJson( const FOOTPRINT* aFootprint )
{
    nlohmann::json models = nlohmann::json::array();

    for( const FP_3DMODEL& m : aFootprint->Models() )
    {
        models.push_back( { { "file", str( m.m_Filename ) },
                            { "offset_mm", { m.m_Offset.x, m.m_Offset.y, m.m_Offset.z } },
                            { "rotation_deg", { m.m_Rotation.x, m.m_Rotation.y, m.m_Rotation.z } },
                            { "scale", { m.m_Scale.x, m.m_Scale.y, m.m_Scale.z } },
                            { "show", m.m_Show } } );
    }

    return models;
}


/// @brief A model's file on disk (variables expanded, relative to the project, .wrl -> .step)
std::filesystem::path resolveModel( const FP_3DMODEL& aModel, PROJECT* aProject )
{
    namespace fs = std::filesystem;
    fs::path p( str( ExpandEnvVarSubstitutions( aModel.m_Filename, aProject ) ) );

    if( p.is_relative() && aProject )
        p = fs::path( str( aProject->GetProjectPath() ) ) / p;

    if( p.extension() == ".wrl" || p.extension() == ".WRL" )
    {
        fs::path step = p;
        step.replace_extension( ".step" );

        if( fs::exists( step ) )
            p = step;
    }

    return p;
}


glm::mat4 modelMatrix( const FP_3DMODEL& aModel )
{
    return CalcModelMatrix( SFVEC3F( aModel.m_Offset.x, aModel.m_Offset.y, aModel.m_Offset.z ),
                            SFVEC3F( aModel.m_Rotation.x, aModel.m_Rotation.y, aModel.m_Rotation.z ),
                            SFVEC3F( aModel.m_Scale.x, aModel.m_Scale.y, aModel.m_Scale.z ) );
}


/// @brief A point of the footprint's 3D frame (mm, y up) on the board (mm, y down, z up from the
/// board's top surface)
std::array<double, 3> toBoard( const FOOTPRINT* aFp, double aX, double aY, double aZ, double aThickness )
{
    double x = aFp->IsFlipped() ? -aX : aX;
    double y = -aY;
    double z = aFp->IsFlipped() ? -aThickness - aZ : aZ;

    VECTOR2D v( x * pcbIUScale.IU_PER_MM, y * pcbIUScale.IU_PER_MM );
    RotatePoint( v, aFp->GetOrientation() );
    const VECTOR2I pos = aFp->GetPosition();

    return { ( v.x + pos.x ) / pcbIUScale.IU_PER_MM, ( v.y + pos.y ) / pcbIUScale.IU_PER_MM, z };
}


bool isTht( const FOOTPRINT* aFp )
{
    for( const PAD* pad : aFp->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::PTH )
            return true;
    }

    return false;
}


double r2( double v )
{
    return std::round( v * 100 ) / 100;
}

} // namespace


static KOPENAPI_RESULT h_pcb_footprint_model_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( !aArgs.contains( "models" ) || !aArgs["models"].is_array() )
        return KOPENAPI_RESULT::Error( 400, "models: [{file, offset_mm?, rotation_deg?, scale?, show?}] ([] removes them)" );

    std::vector<FP_3DMODEL> models;

    for( const nlohmann::json& m : aArgs["models"] )
    {
        FP_3DMODEL model;

        if( !m.is_object() || !m.contains( "file" ) || !m["file"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "every model needs a file (path or ${KICAD10_3DMODEL_DIR}/... / ${KIPRJMOD}/...)" );

        model.m_Filename = wxString::FromUTF8( m["file"].get<std::string>() );

        if( !vec3( m, "offset_mm", model.m_Offset ) || !vec3( m, "rotation_deg", model.m_Rotation )
            || !vec3( m, "scale", model.m_Scale ) )
        {
            return KOPENAPI_RESULT::Error( 400, "offset_mm / rotation_deg / scale: [x, y, z]" );
        }

        model.m_Show = m.value( "show", true );
        models.push_back( model );
    }

    // targets: refs, or every footprint of a library footprint
    std::set<FOOTPRINT*> targets;
    const std::string    libId = aArgs.value( "lib_id", std::string() );
    std::set<std::string> refs;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
        refs.insert( r.get<std::string>() );

    if( aArgs.contains( "ref" ) )
        refs.insert( aArgs["ref"].get<std::string>() );

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( refs.count( str( fp->GetReference() ) ) || ( !libId.empty() && str( fp->GetFPID().Format() ) == libId ) )
            targets.insert( fp );
    }

    if( targets.empty() )
        return KOPENAPI_RESULT::Error( 404, "no footprint matches ref / refs / lib_id (pcb_footprint_list)" );

    BOARD_COMMIT   commit( context->GetToolManager() );
    nlohmann::json changed = nlohmann::json::array();

    for( FOOTPRINT* fp : targets )
    {
        commit.Modify( fp );
        fp->Models() = models;
        changed.push_back( str( fp->GetReference() ) );
    }

    commit.Push( _( "Set 3D models (API)" ) );

    if( !aCtx.headless )
    {
        if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
            frame->Update3DView( true, true );
    }

    std::sort( changed.begin(), changed.end(),
               []( const nlohmann::json& a, const nlohmann::json& b ) { return KopenapiNaturalLess( a, b ); } );

    nlohmann::json answer = { { "footprints", changed }, { "models", modelsJson( *targets.begin() ) } };

    if( nlohmann::json trim = KopenapiLeadTrimRefresh( aCtx, *context ); !trim.is_null() )
        answer["lead_trim"] = trim["trimmed"].size();

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_footprint_model_set",
                   "Set the 3D models of board footprints (all instances of a lib_id, or refs): file "
                   "(${KIPRJMOD}/..., ${KICAD10_3DMODEL_DIR}/..., absolute), offset_mm, rotation_deg, "
                   "scale, show; replaces the footprint's model list; the 3D view refreshes",
                   R"json({"type":"object","required":["models"],"properties":{
                        "lib_id":{"type":"string","description":"every footprint of this library footprint"},
                        "ref":{"type":"string"},
                        "refs":{"type":"array","items":{"type":"string"}},
                        "models":{"type":"array","items":{"type":"object","required":["file"],"properties":{
                            "file":{"type":"string"},
                            "offset_mm":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},
                            "rotation_deg":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},
                            "scale":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},
                            "show":{"type":"boolean","default":true}}}}}})json"_json,
                   false, h_pcb_footprint_model_set );

KOPENAPI_MARK_EDITING( "pcb_footprint_model_set", "pcb_model_seat" );


static KOPENAPI_RESULT h_pcb_models_inspect( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    PROJECT*       project = board->GetProject();
    const double   thickness = board->GetDesignSettings().GetBoardThickness() / pcbIUScale.IU_PER_MM;
    const double   tolerance = aArgs.value( "tolerance_mm", 0.1 );
    const bool     geometry = aArgs.value( "geometry", true );
    std::set<std::string> refs;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
        refs.insert( r.get<std::string>() );

    const std::string libId = aArgs.value( "lib_id", std::string() );
    nlohmann::json    rows = nlohmann::json::array();
    int               problems = 0;
    double            seconds = 0;

    // measure every shown model in parallel first (geometry only, off the main thread); the loop
    // below then reads the cache
    if( geometry )
    {
        // cut copies need only their box (seating is measured on the generic model)
        std::vector<std::tuple<std::filesystem::path, glm::mat4, bool>> jobs;
        std::set<std::string>                                           queued;   // one job per file and placement

        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( ( !refs.empty() && !refs.count( str( fp->GetReference() ) ) )
                || ( !libId.empty() && str( fp->GetFPID().Format() ) != libId ) )
            {
                continue;
            }

            for( const FP_3DMODEL& m : fp->Models() )
            {
                const std::filesystem::path file = resolveModel( m, project );
                const glm::mat4             matrix = modelMatrix( m );
                std::ostringstream          key;
                key << file.string();

                for( int i = 0; i < 16; ++i )
                    key << '|' << matrix[i / 4][i % 4];

                if( std::filesystem::exists( file ) && queued.insert( key.str() ).second )
                    jobs.emplace_back( file, matrix, !m.m_Filename.Contains( wxS( "/.trimmed/" ) ) );
            }
        }

        const auto t0 = std::chrono::steady_clock::now();

        KopenapiRunOffMain(
                [&]()
                {
                    // workers take the next job: a slow model does not hold the others back
                    std::atomic<size_t>            next{ 0 };
                    std::vector<std::future<void>> workers;
                    const size_t width = std::max( 2u, std::thread::hardware_concurrency() / 2 );

                    for( size_t w = 0; w < std::min( width, jobs.size() ); ++w )
                    {
                        workers.push_back( std::async( std::launch::async,
                                                       [&]()
                                                       {
                                                           for( size_t i = next++; i < jobs.size(); i = next++ )
                                                           {
                                                               const auto& [file, matrix, body] = jobs[i];
                                                               KopenapiModelGeometry( file, matrix, body );
                                                           }
                                                       } ) );
                    }

                    for( std::future<void>& w : workers )
                        w.get();
                } );

        seconds = std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();
    }

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( ( !refs.empty() && !refs.count( str( fp->GetReference() ) ) )
            || ( !libId.empty() && str( fp->GetFPID().Format() ) != libId ) )
        {
            continue;
        }

        nlohmann::json models = nlohmann::json::array();
        nlohmann::json issues = nlohmann::json::array();
        bool           shown = false;

        for( const FP_3DMODEL& m : fp->Models() )
        {
            const std::filesystem::path file = resolveModel( m, project );
            nlohmann::json row = { { "file", str( m.m_Filename ) }, { "resolved", file.string() },
                                   { "exists", std::filesystem::exists( file ) }, { "show", m.m_Show },
                                   { "trimmed", m.m_Filename.Contains( wxS( "/.trimmed/" ) ) },
                                   { "offset_mm", { m.m_Offset.x, m.m_Offset.y, m.m_Offset.z } },
                                   { "rotation_deg", { m.m_Rotation.x, m.m_Rotation.y, m.m_Rotation.z } },
                                   { "scale", { m.m_Scale.x, m.m_Scale.y, m.m_Scale.z } } };

            if( !m.m_Show )
            {
                models.push_back( row );
                continue;
            }

            shown = true;

            if( !row["exists"].get<bool>() )
            {
                issues.push_back( "model file missing: " + str( m.m_Filename ) );
                models.push_back( row );
                continue;
            }

            if( geometry )
            {
                // a cut copy (trimmed leads) is seated like its generic model: measure that one
                std::filesystem::path measured = file;

                if( m.m_Filename.Contains( wxS( "/.trimmed/" ) ) )
                {
                    for( const FP_3DMODEL& other : fp->Models() )
                    {
                        if( !other.m_Filename.Contains( wxS( "/.trimmed/" ) ) && other.m_Offset == m.m_Offset
                            && other.m_Rotation == m.m_Rotation && other.m_Scale == m.m_Scale )
                        {
                            measured = resolveModel( other, project );
                            break;
                        }
                    }
                }

                KOPENAPI_MODEL_GEOMETRY g, shown;
                KopenapiRunOffMain(
                        [&]()
                        {
                            g = KopenapiModelGeometry( measured, modelMatrix( m ) );
                            shown = measured == file ? g : KopenapiModelGeometry( file, modelMatrix( m ), false );
                        } );
                seconds += g.seconds + ( measured == file ? 0 : shown.seconds );   // prefetch misses only

                if( !g.ok )
                {
                    issues.push_back( g.error );
                }
                else
                {
                    // board-frame box from the eight corners
                    double lo[3] = { 1e9, 1e9, 1e9 }, hi[3] = { -1e9, -1e9, -1e9 };

                    for( int i = 0; i < 8; ++i )
                    {
                        auto p = toBoard( fp, ( i & 1 ) ? shown.max[0] : shown.min[0], ( i & 2 ) ? shown.max[1] : shown.min[1],
                                          ( i & 4 ) ? shown.max[2] : shown.min[2], thickness );

                        for( int a = 0; a < 3; ++a )
                        {
                            lo[a] = std::min( lo[a], p[a] );
                            hi[a] = std::max( hi[a], p[a] );
                        }
                    }

                    // seating: THT bodies rest on the board, SMD parts' lowest point on the pads
                    const bool   tht = isTht( fp ) && g.hasLeads;
                    const double rest = tht ? g.bodyBottom : g.min[2];
                    const double gap = rest;   // the board's top surface is z = 0 in the footprint frame

                    row["bbox_board_mm"] = { { "min", { r2( lo[0] ), r2( lo[1] ), r2( lo[2] ) } },
                                             { "max", { r2( hi[0] ), r2( hi[1] ), r2( hi[2] ) } } };
                    row["height_mm"] = r2( shown.max[2] - std::max( 0.0, shown.min[2] ) );
                    row["seat"] = { { "rests_on", tht ? "body" : "lowest point" },
                                    { "body_bottom_mm", r2( g.bodyBottom ) },
                                    { "gap_mm", r2( gap ) },
                                    { "leads_below_board_mm", tht ? r2( std::max( 0.0, -shown.min[2] - thickness ) ) : 0.0 } };

                    if( measured != file )
                        row["seat"]["measured_on"] = "generic model (this is a cut copy)";

                    if( gap > tolerance )
                        issues.push_back( "floating " + std::to_string( r2( gap ) ).substr( 0, 4 ) + " mm above the board (pcb_model_seat)" );
                    else if( gap < -tolerance )
                        issues.push_back( "sunk " + std::to_string( r2( -gap ) ).substr( 0, 4 ) + " mm into the board (pcb_model_seat)" );
                }
            }

            models.push_back( row );
        }

        if( !shown )
            issues.push_back( "no 3D model shown" );

        problems += issues.empty() ? 0 : 1;

        if( aArgs.value( "only_problems", false ) && issues.empty() )
            continue;

        rows.push_back( { { "ref", str( fp->GetReference() ) }, { "footprint", str( fp->GetFPID().Format() ) },
                          { "side", fp->IsFlipped() ? "bottom" : "top" },
                          { "position_mm", { r2( fp->GetPosition().x / pcbIUScale.IU_PER_MM ),
                                             r2( fp->GetPosition().y / pcbIUScale.IU_PER_MM ) } },
                          { "rotation_deg", fp->GetOrientation().AsDegrees() },
                          { "tht", isTht( fp ) }, { "models", models }, { "problems", issues } } );
    }

    return KOPENAPI_RESULT::Ok( { { "footprints", rows }, { "with_problems", problems },
                                  { "board_thickness_mm", thickness }, { "geometry_s", r2( seconds ) } } );
}


static KOPENAPI_RESULT h_pcb_model_seat( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    PROJECT*          project = board->GetProject();
    const std::string libId = aArgs.value( "lib_id", std::string() );
    std::set<std::string> refs;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
        refs.insert( r.get<std::string>() );

    if( refs.empty() && libId.empty() )
        return KOPENAPI_RESULT::Error( 400, "give refs or lib_id (seating changes model offsets; check pcb_models_inspect first)" );

    const double   target = aArgs.value( "gap_mm", 0.0 );
    BOARD_COMMIT   commit( context->GetToolManager() );
    nlohmann::json changed = nlohmann::json::array();

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( !refs.count( str( fp->GetReference() ) ) && ( libId.empty() || str( fp->GetFPID().Format() ) != libId ) )
            continue;

        bool modified = false;

        for( FP_3DMODEL& m : fp->Models() )
        {
            if( m.m_Filename.Contains( wxS( "/.trimmed/" ) ) )
                continue;   // cut copies follow their generic model (re-cut below)

            const std::filesystem::path file = resolveModel( m, project );
            KOPENAPI_MODEL_GEOMETRY     g;
            KopenapiRunOffMain( [&]() { g = KopenapiModelGeometry( file, modelMatrix( m ) ); } );

            if( !g.ok )
                continue;

            const double rest = ( isTht( fp ) && g.hasLeads ) ? g.bodyBottom : g.min[2];
            const double dz = target - rest;

            if( std::abs( dz ) < 0.005 )
                continue;

            if( !modified )
                commit.Modify( fp );

            modified = true;
            m.m_Offset.z += dz;
            changed.push_back( { { "ref", str( fp->GetReference() ) }, { "file", str( m.m_Filename ) },
                                 { "moved_z_mm", r2( dz ) }, { "offset_z_mm", r2( m.m_Offset.z ) } } );
        }

        // cut copies go (the lead trim re-cuts them at the new placement); the generic models
        // they hid are shown again
        if( modified )
        {
            const bool hadCut = std::any_of( fp->Models().begin(), fp->Models().end(),
                                             []( const FP_3DMODEL& m ) { return m.m_Filename.Contains( wxS( "/.trimmed/" ) ); } );
            std::vector<FP_3DMODEL> kept;

            for( FP_3DMODEL m : fp->Models() )
            {
                if( m.m_Filename.Contains( wxS( "/.trimmed/" ) ) )
                    continue;

                if( hadCut )
                    m.m_Show = true;

                kept.push_back( m );
            }

            fp->Models() = kept;
        }
    }

    if( changed.empty() )
        return KOPENAPI_RESULT::Ok( { { "changed", changed } } );

    commit.Push( _( "Seat 3D models (API)" ) );

    nlohmann::json answer = { { "changed", changed } };

    if( nlohmann::json trim = KopenapiLeadTrimRefresh( aCtx, *context ); !trim.is_null() )
        answer["lead_trim"] = trim["trimmed"].size();

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_models_inspect",
                   "3D models of the board's footprints for inspection: per model the file (resolved, "
                   "exists), shown, trimmed, offset / rotation / scale, bounding box on the board (mm, z "
                   "up from the top surface), height, seating (THT: body bottom; SMD: lowest point) with "
                   "the gap to the board and lead length below it; problems: missing file, no model, "
                   "floating, sunk. Geometry by OpenCascade, cached",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "lib_id":{"type":"string"},
                        "only_problems":{"type":"boolean","default":false},
                        "geometry":{"type":"boolean","default":true,"description":"false: files and placement only (fast)"},
                        "tolerance_mm":{"type":"number","default":0.1}}})json"_json,
                   false, h_pcb_models_inspect, 600 );

KOPENAPI_REGISTER( "pcb_model_seat",
                   "Seat 3D models on the board: moves the model's z offset so a THT body rests on the "
                   "board (above its leads) and an SMD part's lowest point on the pads (gap_mm to leave a "
                   "standoff); refs or lib_id required; trimmed leads are re-cut",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "lib_id":{"type":"string"},
                        "gap_mm":{"type":"number","default":0}}})json"_json,
                   false, h_pcb_model_seat, 600 );


bool KopenapiAssemblyBox( BOARD* aBoard, double aLo[3], double aHi[3] )
{
    const double thickness = aBoard->GetDesignSettings().GetBoardThickness() / pcbIUScale.IU_PER_MM;
    BOX2I        outline = aBoard->GetBoardEdgesBoundingBox();

    if( outline.GetWidth() <= 0 )
        outline = aBoard->GetBoundingBox();

    aLo[0] = outline.GetLeft() / pcbIUScale.IU_PER_MM;
    aLo[1] = outline.GetTop() / pcbIUScale.IU_PER_MM;
    aLo[2] = -thickness;
    aHi[0] = outline.GetRight() / pcbIUScale.IU_PER_MM;
    aHi[1] = outline.GetBottom() / pcbIUScale.IU_PER_MM;
    aHi[2] = 0;

    // every shown model, box only (cached)
    std::vector<std::tuple<FOOTPRINT*, std::filesystem::path, glm::mat4>> jobs;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( const FP_3DMODEL& m : fp->Models() )
        {
            if( !m.m_Show )
                continue;

            std::filesystem::path file = KopenapiModelFile( fp, m, aBoard->GetProject() );

            if( std::filesystem::exists( file ) )
                jobs.emplace_back( fp, file, modelMatrix( m ) );
        }
    }

    std::vector<KOPENAPI_MODEL_GEOMETRY> boxes( jobs.size() );
    KopenapiRunOffMain(
            [&]()
            {
                for( size_t i = 0; i < jobs.size(); ++i )
                    boxes[i] = KopenapiModelGeometry( std::get<1>( jobs[i] ), std::get<2>( jobs[i] ), false );
            } );

    for( size_t i = 0; i < jobs.size(); ++i )
    {
        const KOPENAPI_MODEL_GEOMETRY& g = boxes[i];

        if( !g.ok )
            continue;

        for( int c = 0; c < 8; ++c )
        {
            auto p = toBoard( std::get<0>( jobs[i] ), ( c & 1 ) ? g.max[0] : g.min[0], ( c & 2 ) ? g.max[1] : g.min[1],
                              ( c & 4 ) ? g.max[2] : g.min[2], thickness );

            for( int a = 0; a < 3; ++a )
            {
                aLo[a] = std::min( aLo[a], p[a] );
                aHi[a] = std::max( aHi[a], p[a] );
            }
        }
    }

    return true;
}
