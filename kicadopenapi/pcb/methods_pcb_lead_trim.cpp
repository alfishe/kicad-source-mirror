/// @file methods_pcb_lead_trim.cpp
/// @brief kicadopenapi trimmed THT leads in 3D: pcb_lead_trim_get, pcb_lead_trim_set.
///
/// Stock 3D models show leads at factory length. Assembled boards have them cut to a short
/// protrusion past the solder side: p = clamp( k * lead diameter, p_min, p_max ), the lead
/// diameter estimated from the pad drill. The project stores the rule (kicadopenapi.lead_trim in
/// the .kicad_pro); applying it shortens each THT model's leads with OpenCascade so they end at
/// z = -( board thickness + p ) in the footprint's frame (a slab under the board is removed and the
/// tip moves up: tip shape, colours and names kept), caches the result in
/// <project>/3dmodels/.trimmed/, shows the cut model and hides the generic one. Turning the rule
/// off shows the generic models again. The cut is made in the model's own frame, so the cut model
/// keeps the generic model's offset / rotation / scale.
#include "kopenapi_pcb.h"

#include <3d_math.h>
#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <common.h>
#include <footprint.h>
#include <frame_type.h>
#include <kicadopenapi_keepalive.h>
#include <kiway.h>
#include <pad.h>
#include <pcb_edit_frame.h>
#include <pgm_base.h>
#include <project.h>
#include <project/project_file.h>
#include <settings/settings_manager.h>
#include <tool/tool_manager.h>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Bnd_Box.hxx>
#include <gp_GTrsf.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <NCollection_DataMap.hxx>
#include <Quantity_ColorRGBA.hxx>

#include <wx/filename.h>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <set>
#include <sstream>

namespace fs = std::filesystem;


namespace
{

const char*    TRIMMED_DIR = "3dmodels/.trimmed";
const wxString TRIMMED_PREFIX = wxS( "${KIPRJMOD}/3dmodels/.trimmed/" );


std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


struct TRIM_RULE
{
    bool           enabled = false;
    double         k = 1.5;          ///< protrusion per mm of lead diameter
    double         pMin = 0.5;       ///< mm
    double         pMax = 1.5;       ///< mm
    nlohmann::json overrides = nlohmann::json::object();   ///< lib_id -> {protrusion_mm} | {trim: false}

    nlohmann::json Json() const
    {
        return { { "enabled", enabled }, { "k", k }, { "p_min_mm", pMin }, { "p_max_mm", pMax },
                 { "overrides", overrides } };
    }

    void From( const nlohmann::json& aJson )
    {
        enabled = aJson.value( "enabled", enabled );
        k = aJson.value( "k", k );
        pMin = aJson.value( "p_min_mm", pMin );
        pMax = aJson.value( "p_max_mm", pMax );

        if( aJson.contains( "overrides" ) && aJson["overrides"].is_object() )
            overrides = aJson["overrides"];
    }
};


TRIM_RULE loadRule( PROJECT& aProject )
{
    TRIM_RULE rule;

    if( aProject.GetProjectFile().Contains( "kicadopenapi.lead_trim" ) )
        rule.From( aProject.GetProjectFile().At( "kicadopenapi.lead_trim" ) );

    return rule;
}


bool isTrimmed( const FP_3DMODEL& aModel )
{
    return aModel.m_Filename.StartsWith( TRIMMED_PREFIX );
}


/// @brief Lead diameter from the largest plated drill of the footprint (lead ≈ drill - 0.3 mm)
double leadDiameter( const FOOTPRINT* aFootprint )
{
    double drill = 0;

    for( const PAD* pad : aFootprint->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::PTH )
            drill = std::max( drill, (double) std::min( pad->GetDrillSize().x, pad->GetDrillSize().y ) );
    }

    return drill > 0 ? std::max( 0.4, drill / pcbIUScale.IU_PER_MM - 0.3 ) : 0.0;
}


/// @brief Shortens every solid of a STEP file that reaches below aTarget (footprint frame, the
/// model frame mapped by aMatrix): a slab of the lead under the board is removed and the tip below
/// it moves up, so the lead ends at aTarget with its own tip shape and colours; nothing is drawn
/// new. Colours and names are kept; writes aOut.
/// @param aBoardBottom z of the board's bottom surface (the slab stays below it)
bool shortenStep( const fs::path& aIn, const fs::path& aOut, const glm::mat4& aMatrix, double aTarget,
                  double aBoardBottom, std::string& aError )
{
    Handle( XCAFApp_Application ) app = XCAFApp_Application::GetApplication();
    Handle( TDocStd_Document ) doc;
    app->NewDocument( "MDTV-XCAF", doc );

    STEPCAFControl_Reader reader;
    reader.SetColorMode( true );
    reader.SetNameMode( true );
    reader.SetLayerMode( true );

    if( reader.ReadFile( aIn.string().c_str() ) != IFSelect_RetDone || !reader.Transfer( doc ) )
    {
        aError = "cannot read " + aIn.string();
        app->Close( doc );
        return false;
    }

    // footprint frame -> model frame
    const glm::mat4 inv = glm::inverse( aMatrix );
    gp_GTrsf        toModel;

    for( int r = 0; r < 3; ++r )
    {
        for( int c = 0; c < 3; ++c )
            toModel.SetValue( r + 1, c + 1, inv[c][r] );   // glm is column-major

        toModel.SetValue( r + 1, 4, inv[3][r] );
    }

    Handle( XCAFDoc_ShapeTool ) shapes = XCAFDoc_DocumentTool::ShapeTool( doc->Main() );
    Handle( XCAFDoc_ColorTool ) colors = XCAFDoc_DocumentTool::ColorTool( doc->Main() );
    std::set<int>               done;
    const double                big = 1000.0;

    auto faceColour = [&]( const TDF_Label& aLabel, const TopoDS_Shape& aFace, Quantity_ColorRGBA& aColour ) -> bool
    {
        TDF_Label sub;

        if( shapes->FindSubShape( aLabel, aFace, sub ) )
        {
            for( XCAFDoc_ColorType type : { XCAFDoc_ColorSurf, XCAFDoc_ColorGen } )
            {
                if( colors->GetColor( sub, type, aColour ) )
                    return true;
            }
        }

        return colors->GetColor( aFace, XCAFDoc_ColorSurf, aColour );
    };

    using COLOURS = NCollection_DataMap<TopoDS_Shape, Quantity_ColorRGBA, TopTools_ShapeMapHasher>;

    // a face map carried through one operation's history
    auto carry = [&]( const COLOURS& aIn, BRepBuilderAPI_MakeShape& aOp, COLOURS& aOut )
    {
        for( COLOURS::Iterator it( aIn ); it.More(); it.Next() )
        {
            const TopTools_ListOfShape& modified = aOp.Modified( it.Key() );

            if( modified.IsEmpty() )
            {
                if( !aOp.IsDeleted( it.Key() ) )
                    aOut.Bind( it.Key(), it.Value() );
            }
            else
            {
                for( const TopoDS_Shape& m : modified )
                    aOut.Bind( m, it.Value() );
            }
        }
    };

    std::function<void( const TDF_Label&, const gp_Trsf& )> walk = [&]( const TDF_Label& aLabel, const gp_Trsf& aPlace )
    {
        if( shapes->IsAssembly( aLabel ) )
        {
            TDF_LabelSequence components;
            shapes->GetComponents( aLabel, components );

            for( int i = 1; i <= components.Length(); ++i )
            {
                TDF_Label proto;

                if( shapes->GetReferredShape( components.Value( i ), proto ) )
                    walk( proto, aPlace * shapes->GetLocation( components.Value( i ) ).Transformation() );
            }

            return;
        }

        if( !shapes->IsSimpleShape( aLabel ) || done.count( aLabel.Tag() ) )
            return;

        done.insert( aLabel.Tag() );

        // footprint frame -> this prototype's frame
        gp_GTrsf toProto = toModel;
        toProto.PreMultiply( gp_GTrsf( aPlace.Inverted() ) );

        gp_GTrsf toFootprint = toProto;
        toFootprint.Invert();

        const TopoDS_Shape shape = shapes->GetShape( aLabel );

        // lowest point in the footprint frame
        Bnd_Box box;
        BRepBndLib::AddOptimal( BRepBuilderAPI_GTransform( shape, toFootprint, true ).Shape(), box, false, false );

        if( box.IsVoid() )
            return;

        double x0, y0, z0, x1, y1, z1;
        box.Get( x0, y0, z0, x1, y1, z1 );

        const double shift = aTarget - z0;   // how much shorter

        if( shift <= 0.01 || z1 < aBoardBottom )
            return;   // already short enough, or entirely under the board (not a lead)

        // keep the tip (up to half the remaining protrusion), drop the slab above it
        const double tip = std::min( 0.5, ( aBoardBottom - aTarget ) / 2 );
        const double slabLow = z0 + tip;
        const double slabHigh = slabLow + shift;

        auto below = [&]( double aZ )
        {
            TopoDS_Shape b = BRepPrimAPI_MakeBox( gp_Pnt( -big, -big, -big ), gp_Pnt( big, big, aZ ) ).Shape();
            return BRepBuilderAPI_GTransform( b, toProto, true ).Shape();
        };

        COLOURS colours;

        for( TopExp_Explorer f( shape, TopAbs_FACE ); f.More(); f.Next() )
        {
            Quantity_ColorRGBA c;

            if( faceColour( aLabel, f.Current(), c ) )
                colours.Bind( f.Current(), c );
        }

        BRepAlgoAPI_Cut    upper( shape, below( slabHigh ) );
        BRepAlgoAPI_Common tipPart( shape, below( slabLow ) );

        if( !upper.IsDone() || !tipPart.IsDone() )
            return;

        // the tip moves up by the slab's height (along the footprint's z, seen from the prototype)
        gp_XYZ from( 0, 0, 0 ), to( 0, 0, shift );
        toProto.Transforms( from );
        toProto.Transforms( to );
        gp_Trsf lift;
        lift.SetTranslation( gp_Vec( to - from ) );
        BRepBuilderAPI_Transform moved( tipPart.Shape(), lift, true );

        COLOURS upperColours, tipColours, movedColours, joined;
        carry( colours, upper, upperColours );
        carry( colours, tipPart, tipColours );
        carry( tipColours, moved, movedColours );

        BRepAlgoAPI_Fuse fuse( upper.Shape(), moved.Shape() );

        if( !fuse.IsDone() )
            return;

        carry( upperColours, fuse, joined );
        carry( movedColours, fuse, joined );

        const TopoDS_Shape result = fuse.Shape();
        shapes->SetShape( aLabel, result );

        for( TopExp_Explorer f( result, TopAbs_FACE ); f.More(); f.Next() )
        {
            if( !joined.IsBound( f.Current() ) )
                continue;

            TDF_Label sub = shapes->AddSubShape( aLabel, f.Current() );

            if( !sub.IsNull() )
                colors->SetColor( sub, joined.Find( f.Current() ), XCAFDoc_ColorSurf );
        }
    };

    TDF_LabelSequence roots;
    shapes->GetFreeShapes( roots );

    for( int i = 1; i <= roots.Length(); ++i )
        walk( roots.Value( i ), gp_Trsf() );

    shapes->UpdateAssemblies();

    std::error_code ec;
    fs::create_directories( aOut.parent_path(), ec );

    STEPCAFControl_Writer writer;
    writer.SetColorMode( true );
    writer.SetNameMode( true );
    writer.SetLayerMode( true );
    Interface_Static::SetCVal( "write.step.unit", "MM" );

    const bool ok = writer.Transfer( doc, STEPControl_AsIs ) && writer.Write( aOut.string().c_str() ) == IFSelect_RetDone;

    app->Close( doc );

    if( !ok )
        aError = "cannot write " + aOut.string();

    return ok;
}


/// @brief Applies the project's rule to every THT footprint (or restores the generic models)
nlohmann::json apply( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const TRIM_RULE& aRule )
{
    BOARD*            board = aContext.GetBoard();
    PROJECT&          project = board->GetProject() ? *board->GetProject() : aCtx.kiway->Prj();
    const fs::path    projectDir = fs::path( str( project.GetProjectPath() ) );
    const double      thickness = board->GetDesignSettings().GetBoardThickness() / pcbIUScale.IU_PER_MM;
    BOARD_COMMIT      commit( aContext.GetToolManager() );
    nlohmann::json    rows = nlohmann::json::array();
    nlohmann::json    errors = nlohmann::json::array();
    std::map<std::string, std::string> made;   // cache key -> file, within this call

    for( FOOTPRINT* fp : board->Footprints() )
    {
        const double lead = leadDiameter( fp );

        if( lead <= 0 || fp->Models().empty() )
            continue;

        const std::string    libId = str( fp->GetFPID().Format() );
        const nlohmann::json over = aRule.overrides.value( libId, nlohmann::json::object() );
        const bool           trim = aRule.enabled && over.value( "trim", true );
        const double         p = over.contains( "protrusion_mm" ) ? over["protrusion_mm"].get<double>()
                                                                  : std::clamp( aRule.k * lead, aRule.pMin, aRule.pMax );

        // generic models only; the ones hidden by an earlier cut are shown again first
        const bool hadCut = std::any_of( fp->Models().begin(), fp->Models().end(), isTrimmed );
        std::vector<FP_3DMODEL> models;

        for( FP_3DMODEL m : fp->Models() )
        {
            if( isTrimmed( m ) )
                continue;

            if( hadCut )
                m.m_Show = true;

            models.push_back( m );
        }

        std::vector<FP_3DMODEL> result;

        for( FP_3DMODEL m : models )
        {
            if( !trim || !m.m_Show )
            {
                result.push_back( m );
                continue;
            }

            const fs::path source = KopenapiModelFile( fp, m, &project );

            if( !fs::exists( source ) )
            {
                errors.push_back( { { "ref", str( fp->GetReference() ) }, { "model", str( m.m_Filename ) },
                                    { "error", "model file not found" } } );
                result.push_back( m );
                continue;
            }

            const glm::mat4 matrix = CalcModelMatrix( SFVEC3F( m.m_Offset.x, m.m_Offset.y, m.m_Offset.z ),
                                                      SFVEC3F( m.m_Rotation.x, m.m_Rotation.y, m.m_Rotation.z ),
                                                      SFVEC3F( m.m_Scale.x, m.m_Scale.y, m.m_Scale.z ) );
            const double    plane = -( thickness + p );   // where the lead ends

            std::error_code ec;
            std::ostringstream key;
            key << source.string() << '|' << (long long) std::chrono::duration_cast<std::chrono::seconds>( fs::last_write_time( source, ec ).time_since_epoch() ).count() << '|';

            for( int i = 0; i < 16; ++i )
                key << std::round( glm::value_ptr( matrix )[i] * 1e4 ) << ',';

            key << std::round( plane * 1e3 );

            std::ostringstream name;
            name << source.stem().string() << '-' << std::hex << std::hash<std::string>{}( key.str() ) << ".step";
            const fs::path out = projectDir / TRIMMED_DIR / name.str();

            std::string error;

            bool shortened = made.count( key.str() ) || fs::exists( out );

            if( !shortened )
            {
                // geometry only (files): runs off the main thread, the UI stays alive meanwhile
                KopenapiRunOffMain( [&]() { shortened = shortenStep( source, out, matrix, plane, -thickness, error ); } );
            }

            if( !shortened )
            {
                errors.push_back( { { "ref", str( fp->GetReference() ) }, { "model", str( m.m_Filename ) }, { "error", error } } );
                result.push_back( m );
                continue;
            }

            made[key.str()] = out.string();

            FP_3DMODEL cutModel = m;
            cutModel.m_Filename = TRIMMED_PREFIX + wxString::FromUTF8( name.str() );
            m.m_Show = false;   // the generic model stays, hidden: switching the rule off restores it
            result.push_back( m );
            result.push_back( cutModel );

            rows.push_back( { { "ref", str( fp->GetReference() ) }, { "lead_mm", std::round( lead * 100 ) / 100 },
                              { "protrusion_mm", std::round( p * 100 ) / 100 },
                              { "source", over.contains( "protrusion_mm" ) ? "override" : "rule" } } );
        }

        bool same = result.size() == fp->Models().size();

        for( size_t i = 0; same && i < result.size(); ++i )
            same = result[i].m_Filename == fp->Models()[i].m_Filename && result[i].m_Show == fp->Models()[i].m_Show;

        if( !same )
        {
            commit.Modify( fp );
            fp->Models() = result;
        }
    }

    commit.Push( _( "Trim THT leads in 3D (API)" ) );

    if( !aCtx.headless && aCtx.kiway )
    {
        if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
            KopenapiRefresh3D( frame );
    }

    return { { "rule", aRule.Json() }, { "board_thickness_mm", thickness }, { "trimmed", rows }, { "errors", errors } };
}

} // namespace


nlohmann::json KopenapiLeadTrimRefresh( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext )
{
    BOARD* board = aContext.GetBoard();

    if( !board->GetProject() )
        return nullptr;

    const TRIM_RULE rule = loadRule( *board->GetProject() );
    return rule.enabled ? apply( aCtx, aContext, rule ) : nlohmann::json();
}


static KOPENAPI_RESULT h_pcb_lead_trim_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    PROJECT&       project = board->GetProject() ? *board->GetProject() : aCtx.kiway->Prj();
    nlohmann::json rows = nlohmann::json::array();

    for( FOOTPRINT* fp : board->Footprints() )
    {
        for( const FP_3DMODEL& m : fp->Models() )
        {
            if( isTrimmed( m ) && m.m_Show )
                rows.push_back( { { "ref", str( fp->GetReference() ) }, { "model", str( m.m_Filename ) } } );
        }
    }

    return KOPENAPI_RESULT::Ok( { { "rule", loadRule( project ).Json() }, { "trimmed", rows } } );
}


static KOPENAPI_RESULT h_pcb_lead_trim_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*   board = context->GetBoard();
    PROJECT& project = board->GetProject() ? *board->GetProject() : aCtx.kiway->Prj();

    if( project.GetProjectPath().IsEmpty() )
        return KOPENAPI_RESULT::Error( 409, "the board has no project (save it in a project first)" );

    TRIM_RULE rule = loadRule( project );
    rule.From( aArgs );

    if( aArgs.contains( "class" ) )
    {
        // IPC-A-610 lead protrusion limits: class 1-2 up to 2.5 mm, class 3 up to 1.5 mm
        const int cls = aArgs["class"].get<int>();

        if( cls < 1 || cls > 3 )
            return KOPENAPI_RESULT::Error( 400, "class: 1, 2 or 3" );

        rule.pMax = cls == 3 ? 1.5 : 2.5;
    }

    if( rule.pMin <= 0 || rule.pMax < rule.pMin || rule.k <= 0 )
        return KOPENAPI_RESULT::Error( 400, "need 0 < p_min_mm <= p_max_mm and k > 0" );

    project.GetProjectFile().Set( "kicadopenapi.lead_trim", rule.Json() );
    project.GetProjectFile().SaveToFile( project.GetProjectPath(), true );

    return KOPENAPI_RESULT::Ok( apply( aCtx, *context, rule ) );
}


KOPENAPI_REGISTER( "pcb_lead_trim_get",
                   "Trimmed THT leads in 3D: the project's rule (enabled, k, p_min_mm, p_max_mm, per "
                   "footprint overrides) and which footprints show cut models",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_lead_trim_get );

KOPENAPI_REGISTER( "pcb_lead_trim_set",
                   "Cut THT leads of the 3D models to an assembled length (protrusion past the solder "
                   "side = clamp(k x lead diameter, p_min, p_max)), stored in the project and applied to "
                   "every THT footprint: cut STEP models cached in <project>/3dmodels/.trimmed, generic "
                   "models kept hidden; enabled: false shows the generic ones again; renders, 3D viewer "
                   "and STEP export use the cut models",
                   R"json({"type":"object","properties":{
                        "enabled":{"type":"boolean","default":true},
                        "class":{"type":"integer","enum":[1,2,3],"description":"IPC-A-610 preset for p_max_mm (3: 1.5 mm, 1-2: 2.5 mm)"},
                        "k":{"type":"number","default":1.5,"description":"protrusion per mm of lead diameter"},
                        "p_min_mm":{"type":"number","default":0.5},
                        "p_max_mm":{"type":"number","default":1.5},
                        "overrides":{"type":"object","description":"per lib_id: {protrusion_mm: 2.0} or {trim: false}"}}})json"_json,
                   false, h_pcb_lead_trim_set, 600 );

KOPENAPI_MARK_EDITING( "pcb_lead_trim_set" );
