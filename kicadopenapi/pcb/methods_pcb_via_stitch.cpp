/// @file methods_pcb_via_stitch.cpp
/// @brief kicadopenapi via stitching (the zone tool's Add Via Stitching generator): areas filled with a via
/// pattern or a guard ring around a net, their properties, Exclude Via From Stitching and Clear Via
/// Stitching Exclusions (PCB_VIA_STITCH, shared with the editor's tools)
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <pcb_track.h>
#include <generators/pcb_via_stitch.h>
#include <geometry/shape_poly_set.h>
#include <kicadopenapi_util.h>
#include <netinfo.h>
#include <string_utils.h>
#include <tool/tool_manager.h>
#include <tools/generator_tool.h>
#include <zone.h>

#include <cmath>
#include <map>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( long long aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1e4 ) / 1e4;
}


const std::map<std::string, PCB_VIA_STITCH_LAYOUT>& layouts()
{
    static const std::map<std::string, PCB_VIA_STITCH_LAYOUT> names = { { "plain", PCB_VIA_STITCH_LAYOUT::PLAIN },
                                                                         { "staggered", PCB_VIA_STITCH_LAYOUT::STAGGERED },
                                                                         { "poisson", PCB_VIA_STITCH_LAYOUT::POISSON } };
    return names;
}


const std::map<std::string, PCB_VIA_STITCH_MODE>& modes()
{
    static const std::map<std::string, PCB_VIA_STITCH_MODE> names = { { "stitch", PCB_VIA_STITCH_MODE::STITCH },
                                                                       { "guard", PCB_VIA_STITCH_MODE::GUARD } };
    return names;
}


template <typename T>
std::string nameOf( const std::map<std::string, T>& aNames, T aValue )
{
    for( const auto& [name, value] : aNames )
    {
        if( value == aValue )
            return name;
    }

    return std::string();
}


std::string netName( BOARD* aBoard, int aCode )
{
    NETINFO_ITEM* net = aCode > 0 ? aBoard->FindNet( aCode ) : nullptr;
    return net ? str( UnescapeString( net->GetNetname() ) ) : std::string();
}


nlohmann::json stitchJson( BOARD* aBoard, PCB_VIA_STITCH* aStitch, size_t aViaLimit = 500 )
{
    nlohmann::json vias = nlohmann::json::array();
    size_t         count = 0;

    for( BOARD_ITEM* item : aStitch->GetBoardItems() )
    {
        if( item->Type() != PCB_VIA_T )
            continue;

        if( count++ < aViaLimit )
        {
            vias.push_back( { { "uuid", str( item->m_Uuid.AsString() ) },
                              { "at_mm", { toMm( item->GetPosition().x ), toMm( item->GetPosition().y ) } } } );
        }
    }

    const BOX2I    box = aStitch->GetBoundingBox();
    nlohmann::json answer = { { "uuid", str( aStitch->m_Uuid.AsString() ) },
                              { "net", netName( aBoard, aStitch->GetNetCode() ) },
                              { "mode", nameOf( modes(), aStitch->GetMode() ) },
                              { "layout", nameOf( layouts(), aStitch->GetLayout() ) },
                              { "pitch_mm", toMm( aStitch->GetPitch() ) },
                              { "via_size_mm", toMm( aStitch->GetViaSize() ) },
                              { "drill_mm", toMm( aStitch->GetViaDrill() ) },
                              { "seed", aStitch->GetSeed() },
                              { "bbox_mm", { toMm( box.GetLeft() ), toMm( box.GetTop() ), toMm( box.GetRight() ), toMm( box.GetBottom() ) } },
                              { "contours_mm", KopenapiContoursMm( aStitch->Outline() ) },
                              { "vias", count },
                              { "via_list", vias },
                              { "exclusions", aStitch->ExclusionCount() },
                              { "locked", aStitch->IsLocked() } };

    if( aStitch->GetMode() == PCB_VIA_STITCH_MODE::GUARD )
        answer["guarded_net"] = netName( aBoard, aStitch->GetGuardedNetCode() );

    if( count > aViaLimit )
        answer["via_list_truncated"] = true;

    return answer;
}


std::vector<PCB_VIA_STITCH*> stitchings( BOARD* aBoard )
{
    std::vector<PCB_VIA_STITCH*> out;

    for( PCB_GENERATOR* generator : aBoard->Generators() )
    {
        if( PCB_VIA_STITCH* stitch = dynamic_cast<PCB_VIA_STITCH*>( generator ) )
            out.push_back( stitch );
    }

    return out;
}


PCB_VIA_STITCH* findStitching( BOARD* aBoard, const std::string& aUuid )
{
    for( PCB_VIA_STITCH* stitch : stitchings( aBoard ) )
    {
        if( str( stitch->m_Uuid.AsString() ) == aUuid )
            return stitch;
    }

    return nullptr;
}


/// @brief The properties of the stitching dialog from aArgs onto aStitch; an error text when one is
/// not valid (nothing is changed then)
std::string applySettings( BOARD* aBoard, PCB_VIA_STITCH* aStitch, const nlohmann::json& aArgs )
{
    NETINFO_ITEM* net = nullptr;
    NETINFO_ITEM* guarded = nullptr;

    for( const char* key : { "net", "guarded_net" } )
    {
        if( !aArgs.contains( key ) )
            continue;

        NETINFO_ITEM* found = aArgs[key].is_string() ? KopenapiFindNet( aBoard, aArgs[key].get<std::string>() ) : nullptr;

        if( !found || found->GetNetCode() <= 0 )
            return std::string( key ) + ": no net " + aArgs[key].dump();

        ( std::string( key ) == "net" ? net : guarded ) = found;
    }

    if( aArgs.contains( "layout" ) && !layouts().count( aArgs.value( "layout", std::string() ) ) )
        return "layout: plain, staggered or poisson";

    if( aArgs.contains( "mode" ) && !modes().count( aArgs.value( "mode", std::string() ) ) )
        return "mode: stitch or guard";

    for( const char* key : { "pitch_mm", "via_size_mm", "drill_mm" } )
    {
        if( aArgs.contains( key ) && ( !aArgs[key].is_number() || aArgs[key].get<double>() <= 0 ) )
            return std::string( key ) + ": a number > 0 (mm)";
    }

    if( aArgs.contains( "seed" ) && ( !aArgs["seed"].is_number_integer() || aArgs["seed"].get<long long>() < 0 ) )
        return "seed: an integer >= 0";

    SHAPE_LINE_CHAIN outline;

    if( aArgs.contains( "points" ) || aArgs.contains( "rect" ) )
    {
        std::string error;
        outline = KopenapiOutlineArg( aArgs, error );

        if( !error.empty() )
            return error;
    }

    const int size = aArgs.contains( "via_size_mm" ) ? KiROUND( aArgs["via_size_mm"].get<double>() * pcbIUScale.IU_PER_MM ) : aStitch->GetViaSize();
    const int drill = aArgs.contains( "drill_mm" ) ? KiROUND( aArgs["drill_mm"].get<double>() * pcbIUScale.IU_PER_MM ) : aStitch->GetViaDrill();

    if( drill >= size )
        return "drill_mm: smaller than the via size (" + std::to_string( toMm( size ) ) + " mm)";

    if( net )
        aStitch->SetNetCode( net->GetNetCode() );

    if( guarded )
        aStitch->SetGuardedNetCode( guarded->GetNetCode() );

    if( aArgs.contains( "mode" ) )
        aStitch->SetMode( modes().at( aArgs["mode"].get<std::string>() ) );

    if( aArgs.contains( "layout" ) )
        aStitch->SetLayout( layouts().at( aArgs["layout"].get<std::string>() ) );

    if( aArgs.contains( "pitch_mm" ) )
        aStitch->SetPitch( KiROUND( aArgs["pitch_mm"].get<double>() * pcbIUScale.IU_PER_MM ) );

    aStitch->SetViaSize( size );
    aStitch->SetViaDrill( drill );

    if( aArgs.contains( "seed" ) )
        aStitch->SetSeed( static_cast<uint32_t>( aArgs["seed"].get<long long>() ) );

    if( outline.PointCount() >= 3 )
    {
        SHAPE_POLY_SET poly;
        poly.AddOutline( outline );
        aStitch->SetOutline( poly );
    }

    return std::string();
}


nlohmann::json refillJson( PCB_VIA_STITCH* aStitch )
{
    nlohmann::json zones = nlohmann::json::array();

    for( ZONE* zone : aStitch->GetZonesNeedingRefillAfterUpdate() )
        zones.push_back( str( zone->m_Uuid.AsString() ) );

    return zones;
}


KOPENAPI_RESULT h_pcb_via_stitch_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    nlohmann::json items = nlohmann::json::array();

    for( PCB_VIA_STITCH* stitch : stitchings( board ) )
    {
        if( !aArgs.contains( "uuid" ) || aArgs.value( "uuid", std::string() ) == str( stitch->m_Uuid.AsString() ) )
            items.push_back( stitchJson( board, stitch ) );
    }

    if( aArgs.contains( "uuid" ) && items.empty() )
        return KOPENAPI_RESULT::Error( 404, "no via stitching " + aArgs.value( "uuid", std::string() ) + " (pcb_via_stitch_list)" );

    return KOPENAPI_RESULT::Ok( { { "items", items }, { "total", items.size() } } );
}


KOPENAPI_RESULT h_pcb_via_stitch( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string op = aArgs.value( "op", std::string() );

    static const std::set<std::string> ops = { "add", "update", "exclude", "clear_exclusions", "remove" };

    if( !ops.count( op ) )
        return KOPENAPI_RESULT::Error( 400, "op: add, update, exclude, clear_exclusions or remove (pcb_via_stitch_list lists)" );

    GENERATOR_TOOL* tool = KopenapiGeneratorTool( context->GetToolManager() );

    if( !tool )
        return KOPENAPI_RESULT::Error( 503, "the generator tool is not available" );

    BOARD_COMMIT commit( context->GetToolManager() );

    if( op == "add" )
    {
        if( !aArgs.contains( "points" ) && !aArgs.contains( "rect" ) )
            return KOPENAPI_RESULT::Error( 400, "add: the area as points [[x_mm, y_mm], ...] or rect [x0, y0, x1, y1]" );

        if( !aArgs.contains( "net" ) )
            return KOPENAPI_RESULT::Error( 400, "net: the vias' net (pcb_net_list)" );

        if( aArgs.value( "mode", std::string() ) == "guard" && !aArgs.contains( "guarded_net" ) )
            return KOPENAPI_RESULT::Error( 400, "guarded_net: the net a guard ring surrounds" );

        std::string            error;
        const SHAPE_LINE_CHAIN chain = KopenapiOutlineArg( aArgs, error );

        if( !error.empty() )
            return KOPENAPI_RESULT::Error( 400, error );

        SHAPE_POLY_SET outline;
        outline.AddOutline( chain );

        std::unique_ptr<PCB_VIA_STITCH> stitch = PCB_VIA_STITCH::CreateForOutline( board, outline );

        if( std::string bad = applySettings( board, stitch.get(), aArgs ); !bad.empty() )
            return KOPENAPI_RESULT::Error( 400, bad );

        PCB_VIA_STITCH* added = stitch.release();
        added->Generate( tool, board, &commit );
        commit.Push( _( "Create Via Stitch" ) );
        context->SetContentModified();

        nlohmann::json answer = stitchJson( board, added );
        answer["zones_to_refill"] = refillJson( added );
        answer["description"] = "Create Via Stitch";
        return KOPENAPI_RESULT::Ok( answer );
    }

    if( op == "exclude" )
    {
        if( !aArgs.contains( "vias" ) || !aArgs["vias"].is_array() || aArgs["vias"].empty() )
            return KOPENAPI_RESULT::Error( 400, "vias: uuids of stitched vias (pcb_via_stitch_list via_list)" );

        std::vector<PCB_VIA*> vias;
        nlohmann::json        skipped = nlohmann::json::array();

        for( const nlohmann::json& id : aArgs["vias"] )
        {
            PCB_VIA* via = nullptr;

            if( id.is_string() )
                via = dynamic_cast<PCB_VIA*>( board->ResolveItem( KIID( wxString::FromUTF8( id.get<std::string>() ) ), true ) );

            if( !via )
                return KOPENAPI_RESULT::Error( 404, "vias: no via " + id.dump() );

            if( PCB_VIA_STITCH::StitchingOf( via ) )
                vias.push_back( via );
            else
                skipped.push_back( id );
        }

        if( vias.empty() )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 422, "none of the vias belongs to a via stitching" );
            r.body["skipped"] = skipped;
            return r;
        }

        std::set<PCB_VIA_STITCH*> changed = PCB_VIA_STITCH::ExcludeVias( vias, tool, board, commit );
        commit.Push( _( "Exclude Via From Stitching" ) );
        context->SetContentModified();

        nlohmann::json items = nlohmann::json::array();

        for( PCB_VIA_STITCH* stitch : changed )
            items.push_back( stitchJson( board, stitch ) );

        return KOPENAPI_RESULT::Ok( { { "items", items }, { "excluded", vias.size() }, { "skipped", skipped },
                                      { "description", "Exclude Via From Stitching" } } );
    }

    std::vector<PCB_VIA_STITCH*> chosen;
    const nlohmann::json         ids = aArgs.contains( "uuids" ) ? aArgs["uuids"]
                                       : aArgs.contains( "uuid" ) ? nlohmann::json::array( { aArgs["uuid"] } )
                                                                  : nlohmann::json::array();

    if( !ids.is_array() || ids.empty() || ( op != "clear_exclusions" && ids.size() != 1 ) )
        return KOPENAPI_RESULT::Error( 400, op == "clear_exclusions" ? "uuid or uuids: via stitchings" : "uuid: one via stitching" );

    for( const nlohmann::json& id : ids )
    {
        PCB_VIA_STITCH* stitch = id.is_string() ? findStitching( board, id.get<std::string>() ) : nullptr;

        if( !stitch )
            return KOPENAPI_RESULT::Error( 404, "no via stitching " + id.dump() + " (pcb_via_stitch_list)" );

        if( stitch->IsLocked() && op != "clear_exclusions" && !aArgs.value( "override_locks", false ) )
            return KOPENAPI_RESULT::Error( 409, "via stitching " + id.get<std::string>() + " is locked (override_locks)" );

        chosen.push_back( stitch );
    }

    if( op == "clear_exclusions" )
    {
        std::set<PCB_VIA_STITCH*> changed = PCB_VIA_STITCH::ClearExclusions( chosen, tool, board, commit );

        if( changed.empty() )
            return KOPENAPI_RESULT::Error( 422, "no exclusions to clear" );

        commit.Push( _( "Clear Via Stitching Exclusions" ) );
        context->SetContentModified();

        nlohmann::json items = nlohmann::json::array();

        for( PCB_VIA_STITCH* stitch : changed )
            items.push_back( stitchJson( board, stitch ) );

        return KOPENAPI_RESULT::Ok( { { "items", items }, { "description", "Clear Via Stitching Exclusions" } } );
    }

    PCB_VIA_STITCH* stitch = chosen.front();

    if( op == "remove" )
    {
        const size_t vias = stitchJson( board, stitch, 0 )["vias"].get<size_t>();
        stitch->Remove( tool, board, &commit );
        commit.Push( _( "Remove Via Stitching" ) );
        context->SetContentModified();
        return KOPENAPI_RESULT::Ok( { { "removed", ids[0] }, { "vias_removed", vias } } );
    }

    // update: the properties dialog's OK
    commit.Modify( stitch );

    if( std::string bad = applySettings( board, stitch, aArgs ); !bad.empty() )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Error( 400, bad );
    }

    stitch->Generate( tool, board, &commit );
    commit.Push( _( "Edit Via Stitching" ) );
    context->SetContentModified();

    nlohmann::json answer = stitchJson( board, stitch );
    answer["zones_to_refill"] = refillJson( stitch );
    answer["description"] = "Edit Via Stitching";
    return KOPENAPI_RESULT::Ok( answer );
}

} // namespace


KOPENAPI_REGISTER( "pcb_via_stitch_list",
                   "Via stitching areas (the zone tool's Add Via Stitching), uuid for one: each area's net, mode (stitch: "
                   "a via pattern over the area; guard: a ring around guarded_net inside it), layout (plain, staggered, "
                   "poisson), pitch, via size / drill, seed, outline, its vias (via_list) and exclusions",
                   R"json({"type":"object","properties":{"uuid":{"type":"string"}}})json"_json,
                   false, h_pcb_via_stitch_list );

KOPENAPI_REGISTER( "pcb_via_stitch",
                   "Edit via stitching areas (the zone tool's Add Via Stitching; pcb_via_stitch_list lists them): op add: "
                   "an area (points / rect) with net and the settings; op update (uuid): the properties dialog's "
                   "settings or a new outline, regenerated; op exclude: vias (uuids of stitched vias) left out of "
                   "their stitching (Exclude Via From Stitching); op clear_exclusions (uuid / uuids); op remove (uuid) "
                   "with its vias. One undo step each; zones_to_refill names the pours the vias cross (pcb_zones_fill)",
                   R"json({"type":"object","required":["op"],"properties":{
                        "op":{"type":"string","enum":["add","update","exclude","clear_exclusions","remove"]},
                        "uuid":{"type":"string"},"uuids":{"type":"array","items":{"type":"string"}},
                        "points":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"the area [[x_mm, y_mm], ...] (or rect)"},
                        "rect":{"type":"array","items":{"type":"number"},"description":"the area [x0_mm, y0_mm, x1_mm, y1_mm]"},
                        "net":{"type":"string"},"guarded_net":{"type":"string","description":"mode guard: the net surrounded"},
                        "mode":{"type":"string","enum":["stitch","guard"]},
                        "layout":{"type":"string","enum":["plain","staggered","poisson"]},
                        "pitch_mm":{"type":"number"},"via_size_mm":{"type":"number"},"drill_mm":{"type":"number"},
                        "seed":{"type":"integer","description":"poisson layout: the tile shift"},
                        "vias":{"type":"array","items":{"type":"string"},"description":"exclude: stitched vias' uuids"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_via_stitch );

KOPENAPI_MARK_EDITING( "pcb_via_stitch" );
