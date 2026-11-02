/// @file methods_pcb_convert.cpp
/// @brief kicadopenapi pcb_convert: the board editor's Convert menu (Create Polygon / Zone / Rule
/// Area / Lines / Tracks / Arc from the given items, Outset) through CONVERT_TOOL's and
/// OUTSET_ROUTINE's own code, one undo step per call.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <tool/tool_manager.h>
#include <tools/convert_tool.h>
#include <tools/item_modification_routine.h>
#include <zone.h>
#include <zone_settings.h>

#include <deque>
#include <map>
#include <optional>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


std::optional<PCB_LAYER_ID> namedLayer( BOARD* aBoard, const nlohmann::json& aName )
{
    if( !aName.is_string() )
        return std::nullopt;

    const int id = aBoard->GetLayerID( wxString::FromUTF8( aName.get<std::string>() ) );

    if( id < 0 || !aBoard->IsLayerEnabled( (PCB_LAYER_ID) id ) )
        return std::nullopt;

    return (PCB_LAYER_ID) id;
}


nlohmann::json describe( BOARD* aBoard, BOARD_ITEM* aItem )
{
    nlohmann::json row = { { "uuid", str( aItem->m_Uuid.AsString() ) },
                           { "kind", str( aItem->GetClass() ) },
                           { "layer", str( aBoard->GetLayerName( aItem->GetLayer() ) ) } };

    if( aItem->Type() == PCB_SHAPE_T )
        row["shape"] = str( static_cast<PCB_SHAPE*>( aItem )->ShowShape() );

    if( BOARD_CONNECTED_ITEM* connected = dynamic_cast<BOARD_CONNECTED_ITEM*>( aItem ) )
    {
        if( connected->GetNetCode() > 0 )
            row["net"] = str( connected->GetNetname() );
    }

    return row;
}


/// Lines / Tracks take what the editor's selection filter for them keeps
bool lineSource( BOARD_ITEM* aItem, bool aTracks )
{
    if( aItem->Type() == PCB_ZONE_T )
        return true;

    if( aItem->Type() != PCB_SHAPE_T )
        return false;

    switch( static_cast<PCB_SHAPE*>( aItem )->GetShape() )
    {
    case SHAPE_T::POLY:
    case SHAPE_T::RECTANGLE:
    case SHAPE_T::ELLIPSE:
    case SHAPE_T::ELLIPSE_ARC: return true;
    case SHAPE_T::SEGMENT:
    case SHAPE_T::ARC: return aTracks;
    default: return false;
    }
}


const std::map<std::string, CONVERT_STRATEGY>& strategies()
{
    static const std::map<std::string, CONVERT_STRATEGY> s = { { "centerline", CENTERLINE },
                                                               { "copy_line_width", COPY_LINEWIDTH },
                                                               { "bounding_hull", BOUNDING_HULL } };
    return s;
}

} // namespace


static KOPENAPI_RESULT h_pcb_convert( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                 board = context->GetBoard();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    const std::string      to = aArgs.value( "to", std::string() );

    static const std::set<std::string> targets = { "poly", "zone", "keepout", "lines", "tracks", "arc", "outset" };

    if( !targets.count( to ) )
        return KOPENAPI_RESULT::Error( 400, "to: poly, zone, keepout, lines, tracks, arc or outset" );

    if( !aArgs.contains( "uuids" ) || !aArgs["uuids"].is_array() || aArgs["uuids"].empty() )
        return KOPENAPI_RESULT::Error( 400, "uuids: the items to convert" );

    std::deque<EDA_ITEM*>    items;
    std::vector<BOARD_ITEM*> sources;

    for( const nlohmann::json& id : aArgs["uuids"] )
    {
        if( !id.is_string() )
            return KOPENAPI_RESULT::Error( 400, "uuids: strings" );

        BOARD_ITEM* item = board->ResolveItem( KIID( wxString::FromUTF8( id.get<std::string>() ) ), true );

        if( !item )
            return KOPENAPI_RESULT::Error( 404, "no item " + id.get<std::string>() );

        if( std::find( sources.begin(), sources.end(), item ) == sources.end() )
        {
            sources.push_back( item );
            items.push_back( item );
        }
    }

    std::optional<PCB_LAYER_ID> layer;

    if( aArgs.contains( "layer" ) )
    {
        layer = namedLayer( board, aArgs["layer"] );

        if( !layer )
            return KOPENAPI_RESULT::Error( 400, "layer: an enabled layer name" );
    }

    const bool deleteOriginals = aArgs.value( "delete_originals", to != "outset" && to != "arc" );

    BOARD_COMMIT             commit( context->GetToolManager() );
    std::vector<BOARD_ITEM*> created;
    std::vector<BOARD_ITEM*> removed;
    nlohmann::json           out = nlohmann::json::object();
    wxString                 message;

    if( to == "poly" || to == "zone" || to == "keepout" )
    {
        const std::string strategyName = aArgs.value( "strategy", std::string( "centerline" ) );
        auto              it = strategies().find( strategyName );

        if( it == strategies().end() )
            return KOPENAPI_RESULT::Error( 400, "strategy: centerline, copy_line_width or bounding_hull" );

        if( it->second == COPY_LINEWIDTH && ( to != "poly" || sources.front()->Type() == PCB_PAD_T ) )
            return KOPENAPI_RESULT::Error( 400, "strategy copy_line_width: for poly from graphics only" );

        const double gapMm = aArgs.value( "gap_mm", 0.0 );
        const double widthMm = aArgs.value( "line_width_mm", 0.0 );

        if( gapMm < 0 || widthMm < 0 )
            return KOPENAPI_RESULT::Error( 400, "gap_mm, line_width_mm: >= 0" );

        CONVERT_SETTINGS settings;
        settings.m_Strategy = it->second;
        settings.m_Gap = pcbIUScale.mmToIU( gapMm );
        settings.m_LineWidth = pcbIUScale.mmToIU( widthMm );
        settings.m_DeleteOriginals = deleteOriginals;

        const PCB_LAYER_ID destLayer = layer ? *layer : sources.front()->GetLayer();
        std::vector<SHAPE_POLY_SET> polys;

        if( to == "poly" )
        {
            const CONVERT_SETTINGS resolved = CONVERT_TOOL::ResolvePolySettings( settings, bds, destLayer );

            if( !CONVERT_TOOL::MakePolys( items, resolved, bds.m_MaxError, polys ) )
            {
                return KOPENAPI_RESULT::Error( 422, resolved.m_Strategy == BOUNDING_HULL
                                                            ? "resulting polygon would be empty"
                                                            : "objects must form a closed shape" );
            }

            for( PCB_SHAPE* shape : CONVERT_TOOL::AddPolyShapes( commit, items, polys, resolved,
                                                                 sources.front()->GetParentFootprint(),
                                                                 destLayer ) )
            {
                created.push_back( shape );
            }

            message = deleteOriginals ? _( "Convert to Polygon" ) : _( "Create Polygon" );
        }
        else
        {
            ZONE_SETTINGS zoneInfo = bds.GetDefaultZoneSettings();
            zoneInfo.m_Layers.reset().set( destLayer );
            zoneInfo.m_Name = wxString::FromUTF8( aArgs.value( "name", std::string() ) );
            zoneInfo.SetIsRuleArea( to == "keepout" );

            if( to == "keepout" )
            {
                if( !IsCopperLayer( destLayer ) )
                    return KOPENAPI_RESULT::Error( 400, "layer: a keep-out goes on a copper layer" );

                zoneInfo.SetDoNotAllowTracks( aArgs.value( "no_tracks", zoneInfo.GetDoNotAllowTracks() ) );
                zoneInfo.SetDoNotAllowVias( aArgs.value( "no_vias", zoneInfo.GetDoNotAllowVias() ) );
                zoneInfo.SetDoNotAllowPads( aArgs.value( "no_pads", zoneInfo.GetDoNotAllowPads() ) );
                zoneInfo.SetDoNotAllowZoneFills( aArgs.value( "no_copper_pour",
                                                              zoneInfo.GetDoNotAllowZoneFills() ) );
                zoneInfo.SetDoNotAllowFootprints( aArgs.value( "no_footprints",
                                                               zoneInfo.GetDoNotAllowFootprints() ) );
            }
            else if( aArgs.contains( "net" ) )
            {
                if( !IsCopperLayer( destLayer ) )
                    return KOPENAPI_RESULT::Error( 400, "net: for a zone on a copper layer" );

                NETINFO_ITEM* net = aArgs["net"].is_string()
                                            ? board->FindNet( wxString::FromUTF8( aArgs["net"].get<std::string>() ) )
                                            : nullptr;

                if( !net )
                    return KOPENAPI_RESULT::Error( 404, "no net " + aArgs["net"].dump() );

                zoneInfo.m_Netcode = net->GetNetCode();
            }

            if( !CONVERT_TOOL::MakePolys( items, settings, bds.m_MaxError, polys ) )
                return KOPENAPI_RESULT::Error( 422, "objects must form a closed shape" );

            for( ZONE* zone : CONVERT_TOOL::AddZones( commit, board, polys, zoneInfo ) )
                created.push_back( zone );

            message = deleteOriginals ? _( "Convert to Zone" ) : _( "Create Zone" );
        }

        if( deleteOriginals )
        {
            for( BOARD_ITEM* item : sources )
            {
                if( item->GetFlags() & SKIP_STRUCT )
                    removed.push_back( item );
            }
        }

        for( BOARD_ITEM* item : sources )
            item->ClearTempFlags();
    }
    else if( to == "lines" || to == "tracks" )
    {
        const bool tracks = to == "tracks";

        for( BOARD_ITEM* item : sources )
        {
            if( !lineSource( item, tracks ) )
            {
                return KOPENAPI_RESULT::Error( 400, "item " + str( item->m_Uuid.AsString() ) + " (" + str( item->GetClass() )
                                                            + ") is not a polygon, rectangle, ellipse or zone"
                                                            + ( tracks ? std::string( ", line or arc" ) : std::string() ) );
            }
        }

        const PCB_LAYER_ID destLayer = layer ? *layer : sources.front()->GetLayer();

        if( tracks && !IsCopperLayer( destLayer ) )
            return KOPENAPI_RESULT::Error( 400, "layer: tracks go on a copper layer" );

        created = CONVERT_TOOL::AddLines( commit, items, !tracks, destLayer, board, nullptr );

        if( deleteOriginals )
            removed = sources;

        message = _( "Create Lines" );
    }
    else if( to == "arc" )
    {
        for( BOARD_ITEM* item : sources )
        {
            BOARD_ITEM* arc = CONVERT_TOOL::AddArc( commit, item, board );

            if( !arc )
            {
                return KOPENAPI_RESULT::Error( 400, "item " + str( item->m_Uuid.AsString() )
                                                            + ": a line, an arc, a track or a track arc" );
            }

            created.push_back( arc );
        }

        if( deleteOriginals )
            removed = sources;

        message = _( "Create Arc" );
    }
    else
    {
        for( BOARD_ITEM* item : sources )
        {
            if( item->Type() != PCB_PAD_T && item->Type() != PCB_SHAPE_T )
                return KOPENAPI_RESULT::Error( 400, "item " + str( item->m_Uuid.AsString() ) + ": a pad or a shape" );

            if( item->IsLocked() && !aArgs.value( "override_locks", false ) )
            {
                return KOPENAPI_RESULT::Error( 409, "item " + str( item->m_Uuid.AsString() )
                                                            + " is locked (pcb_unlock, or override_locks)" );
            }
        }

        const double outsetMm = aArgs.value( "outset_mm", 1.0 );
        const PCB_LAYER_ID outLayer = layer ? *layer : Edge_Cuts;

        OUTSET_ROUTINE::PARAMETERS params{
            pcbIUScale.mmToIU( outsetMm ),
            aArgs.value( "round_corners", true ),
            aArgs.value( "use_source_layers", true ),
            aArgs.value( "use_source_widths", true ),
            outLayer,
            aArgs.contains( "line_width_mm" ) ? pcbIUScale.mmToIU( aArgs.value( "line_width_mm", 0.0 ) )
                                              : bds.GetLineThickness( outLayer ),
            std::nullopt,
            deleteOriginals,
        };

        if( aArgs.contains( "grid_rounding_mm" ) )
        {
            const double grid = aArgs.value( "grid_rounding_mm", 0.0 );

            if( grid <= 0 )
                return KOPENAPI_RESULT::Error( 400, "grid_rounding_mm: > 0" );

            params.gridRounding = pcbIUScale.mmToIU( grid );
        }

        if( params.outsetDistance <= 0 || params.lineWidth < 0 )
            return KOPENAPI_RESULT::Error( 400, "outset_mm: > 0, line_width_mm: >= 0" );

        ITEM_MODIFICATION_ROUTINE::CALLABLE_BASED_HANDLER handler(
                [&]( std::unique_ptr<BOARD_ITEM> aItem )
                {
                    created.push_back( aItem.get() );
                    commit.Add( aItem.release() );
                },
                []( BOARD_ITEM& ) {},
                [&]( BOARD_ITEM& aItem )
                {
                    if( !aItem.GetParentFootprint() )
                        removed.push_back( &aItem );
                } );

        OUTSET_ROUTINE routine( board, handler, params );

        for( BOARD_ITEM* item : sources )
            routine.ProcessItem( *item );

        out["outset"] = routine.GetSuccesses();
        out["failed"] = routine.GetFailures();
        message = routine.GetCommitDescription();
    }

    std::vector<nlohmann::json> createdRows;
    nlohmann::json              removedIds = nlohmann::json::array();

    for( BOARD_ITEM* item : removed )
    {
        removedIds.push_back( str( item->m_Uuid.AsString() ) );
        commit.Remove( item );
    }

    if( !commit.Empty() )
        commit.Push( message );

    for( BOARD_ITEM* item : created )
        createdRows.push_back( describe( board, item ) );

    out["to"] = to;
    out["created"] = createdRows;
    out["removed"] = removedIds;
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_convert",
                   "The board editor's Convert menu on the given items (uuids), in one undo step. to: poly (Create "
                   "Polygon from closed chains of lines / arcs, closed shapes, pads, texts; strategy centerline "
                   "(filled), copy_line_width or bounding_hull with gap_mm; line_width_mm, 0: the layer's default), "
                   "zone / keepout (Create Zone / Rule Area from the same polygons with Board Setup's zone defaults; "
                   "zone: net, name; keepout: name and no_tracks / no_vias / no_pads / no_copper_pour / "
                   "no_footprints; more with pcb_zone_update), lines / tracks (Create Lines / Tracks from polygons, "
                   "rectangles, ellipses, zones; tracks also from lines and arcs, keeping the source's net), arc "
                   "(Create Arc: a line becomes an arc, an arc changes graphic <-> track), outset (Outset Items: pads "
                   "and shapes grown by outset_mm, round_corners, use_source_layers / use_source_widths or layer "
                   "(default Edge.Cuts) and line_width_mm, grid_rounding_mm; locked items 409 unless "
                   "override_locks). layer: the target layer, default the first item's (the editor uses the active "
                   "layer). delete_originals: default true, false for arc and outset. Answers created (uuid, kind, "
                   "layer, shape, net) and removed uuids; 422 when no polygon forms",
                   R"json({"type":"object","required":["uuids","to"],"properties":{
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "to":{"type":"string","enum":["poly","zone","keepout","lines","tracks","arc","outset"]},
                        "layer":{"type":"string"},
                        "delete_originals":{"type":"boolean"},
                        "strategy":{"type":"string","enum":["centerline","copy_line_width","bounding_hull"],"default":"centerline"},
                        "gap_mm":{"type":"number","default":0},
                        "line_width_mm":{"type":"number"},
                        "net":{"type":"string"},
                        "name":{"type":"string"},
                        "no_tracks":{"type":"boolean"},"no_vias":{"type":"boolean"},"no_pads":{"type":"boolean"},
                        "no_copper_pour":{"type":"boolean"},"no_footprints":{"type":"boolean"},
                        "outset_mm":{"type":"number","default":1},
                        "round_corners":{"type":"boolean","default":true},
                        "use_source_layers":{"type":"boolean","default":true},
                        "use_source_widths":{"type":"boolean","default":true},
                        "grid_rounding_mm":{"type":"number"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_convert );

KOPENAPI_MARK_EDITING( "pcb_convert" );
