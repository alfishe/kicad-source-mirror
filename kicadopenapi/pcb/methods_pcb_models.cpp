/// @file methods_pcb_models.cpp
/// @brief kicadopenapi 3D models of board footprints: pcb_footprint_model_set.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <frame_type.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <tool/tool_manager.h>

#include <set>


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

    return KOPENAPI_RESULT::Ok( { { "footprints", changed }, { "models", modelsJson( *targets.begin() ) } } );
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

KOPENAPI_MARK_EDITING( "pcb_footprint_model_set" );
