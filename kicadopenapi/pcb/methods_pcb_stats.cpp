/// @file methods_pcb_stats.cpp
/// @brief kicadopenapi pcb_stats: level-0 board statistics in one call (docs/api/design-analysis-api.md §2).
///
/// The default pass is linear in the board size.  `slow: true` adds upstream's full board
/// statistics (copper areas, minimum track clearance — an O(n²) track scan), which is what
/// makes `kicad-cli pcb export stats` take tens of seconds on large boards.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <board_statistics_report.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <geometry/shape_poly_set.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_group.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <pcb_track.h>
#include <project/net_settings.h>
#include <zone.h>

#include <chrono>
#include <cmath>
#include <map>


/// @brief IU → mm at nanometre resolution; in double (IUTomm() takes int and overflows on sums)
static double mm( double aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1e6 ) / 1e6;
}


static double mm2( double aIU2 )
{
    return std::round( aIU2 / ( pcbIUScale.IU_PER_MM * pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


static nlohmann::json sideCounts( int aFront, int aBack )
{
    return { { "front", aFront }, { "back", aBack }, { "total", aFront + aBack } };
}


static nlohmann::json footprintStats( BOARD* aBoard )
{
    int tht[2] = {}, smd[2] = {}, other[2] = {};
    int dnp = 0, excludedFromBom = 0, excludedFromPos = 0, boardOnly = 0, withoutPads = 0;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        const int side = fp->GetSide() == B_Cu ? 1 : 0;
        const int attrs = fp->GetAttributes();

        // Same classification as upstream board statistics (THT wins over SMD)
        if( attrs & FP_THROUGH_HOLE )
            tht[side]++;
        else if( attrs & FP_SMD )
            smd[side]++;
        else
            other[side]++;

        dnp += fp->IsDNP() ? 1 : 0;
        excludedFromBom += fp->IsExcludedFromBOM() ? 1 : 0;
        excludedFromPos += ( attrs & FP_EXCLUDE_FROM_POS_FILES ) ? 1 : 0;
        boardOnly += fp->IsBoardOnly() ? 1 : 0;
        withoutPads += fp->Pads().empty() ? 1 : 0;
    }

    return { { "total", aBoard->Footprints().size() },
             { "tht", sideCounts( tht[0], tht[1] ) },
             { "smd", sideCounts( smd[0], smd[1] ) },
             { "unspecified", sideCounts( other[0], other[1] ) },
             { "front", tht[0] + smd[0] + other[0] },
             { "back", tht[1] + smd[1] + other[1] },
             { "dnp", dnp },
             { "excluded_from_bom", excludedFromBom },
             { "excluded_from_pos", excludedFromPos },
             { "board_only", boardOnly },
             { "without_pads", withoutPads } };
}


static nlohmann::json padStats( BOARD* aBoard )
{
    std::map<std::string, int> byType = { { "through_hole", 0 }, { "smd", 0 }, { "connector", 0 }, { "npth", 0 } };
    std::map<std::string, int> byProperty;
    int                        total = 0, unconnectedToNet = 0;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            total++;

            switch( pad->GetAttribute() )
            {
            case PAD_ATTRIB::PTH:  byType["through_hole"]++; break;
            case PAD_ATTRIB::SMD:  byType["smd"]++;          break;
            case PAD_ATTRIB::CONN: byType["connector"]++;    break;
            case PAD_ATTRIB::NPTH: byType["npth"]++;         break;
            default:                                         break;
            }

            switch( pad->GetProperty() )
            {
            case PAD_PROP::BGA:            byProperty["bga"]++;            break;
            case PAD_PROP::FIDUCIAL_GLBL:  byProperty["fiducial_global"]++; break;
            case PAD_PROP::FIDUCIAL_LOCAL: byProperty["fiducial_local"]++; break;
            case PAD_PROP::TESTPOINT:      byProperty["testpoint"]++;      break;
            case PAD_PROP::HEATSINK:       byProperty["heatsink"]++;       break;
            case PAD_PROP::CASTELLATED:    byProperty["castellated"]++;    break;
            case PAD_PROP::MECHANICAL:     byProperty["mechanical"]++;     break;
            case PAD_PROP::PRESSFIT:       byProperty["press_fit"]++;      break;
            default:                                                       break;
            }

            if( pad->GetAttribute() != PAD_ATTRIB::NPTH && pad->GetNetCode() <= 0 )
                unconnectedToNet++;
        }
    }

    nlohmann::json out = byType;
    out["total"] = total;
    out["by_property"] = byProperty;
    out["without_net"] = unconnectedToNet;
    return out;
}


static nlohmann::json trackStats( BOARD* aBoard, nlohmann::json& aVias )
{
    std::map<std::string, double> lengthByLayer;
    std::map<std::string, int>    segmentsByLayer;
    std::map<std::string, int>    vias = { { "through", 0 }, { "blind", 0 }, { "buried", 0 }, { "micro", 0 } };
    int    segments = 0, arcs = 0;
    double totalLength = 0.0;
    int    minWidth = 0;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->Type() == PCB_VIA_T )
        {
            switch( static_cast<PCB_VIA*>( track )->GetViaType() )
            {
            case VIATYPE::THROUGH:  vias["through"]++; break;
            case VIATYPE::BLIND:    vias["blind"]++;   break;
            case VIATYPE::BURIED:   vias["buried"]++;  break;
            case VIATYPE::MICROVIA: vias["micro"]++;   break;
            default:                                   break;
            }

            continue;
        }

        if( track->Type() == PCB_ARC_T )
            arcs++;
        else
            segments++;

        const std::string layer = aBoard->GetLayerName( track->GetLayer() ).ToStdString();
        const double      length = track->GetLength();

        lengthByLayer[layer] += length;
        segmentsByLayer[layer]++;
        totalLength += length;

        if( minWidth == 0 || track->GetWidth() < minWidth )
            minWidth = track->GetWidth();
    }

    nlohmann::json layers = nlohmann::json::object();

    for( const auto& [layer, length] : lengthByLayer )
        layers[layer] = { { "length_mm", mm( length ) }, { "items", segmentsByLayer[layer] } };

    int viaTotal = 0;

    for( const auto& [type, count] : vias )
        viaTotal += count;

    aVias = vias;
    aVias["total"] = viaTotal;

    return { { "segments", segments },
             { "arcs", arcs },
             { "length_mm", mm( totalLength ) },
             { "min_width_mm", mm( minWidth ) },
             { "by_layer", layers } };
}


static nlohmann::json zoneStats( BOARD* aBoard )
{
    int copper = 0, filled = 0, ruleAreas = 0;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone->GetIsRuleArea() )
        {
            ruleAreas++;
            continue;
        }

        copper++;
        filled += zone->IsFilled() ? 1 : 0;
    }

    return { { "copper", copper }, { "filled", filled }, { "rule_areas", ruleAreas } };
}


static nlohmann::json netStats( BOARD* aBoard )
{
    std::map<int, int> padsPerNet;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 )
                padsPerNet[pad->GetNetCode()]++;
        }
    }

    int total = 0, singlePad = 0, withoutPads = 0;

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
    {
        if( net->GetNetCode() <= 0 )
            continue;

        total++;

        auto it = padsPerNet.find( net->GetNetCode() );

        if( it == padsPerNet.end() )
            withoutPads++;
        else if( it->second == 1 )
            singlePad++;
    }

    nlohmann::json out = { { "total", total }, { "single_pad", singlePad }, { "without_pads", withoutPads } };

    if( std::shared_ptr<CONNECTIVITY_DATA> conn = aBoard->GetConnectivity() )
        out["unrouted_connections"] = conn->GetUnconnectedCount( false );

    if( std::shared_ptr<NET_SETTINGS> settings = aBoard->GetDesignSettings().m_NetSettings )
        out["net_classes"] = settings->GetNetclasses().size() + 1;   // + Default

    return out;
}


static nlohmann::json otherStats( BOARD* aBoard )
{
    int shapes = 0, texts = 0, edgeCuts = 0;

    for( BOARD_ITEM* item : aBoard->Drawings() )
    {
        if( item->Type() == PCB_SHAPE_T )
        {
            shapes++;
            edgeCuts += item->GetLayer() == Edge_Cuts ? 1 : 0;
        }
        else if( item->Type() == PCB_TEXT_T || item->Type() == PCB_TEXTBOX_T )
        {
            texts++;
        }
    }

    return { { "drawings", aBoard->Drawings().size() },
             { "shapes", shapes },
             { "edge_cut_shapes", edgeCuts },
             { "texts", texts },
             { "groups", aBoard->Groups().size() } };
}


/// @brief Board outline as upstream measures it: bbox and area of the outline polygons
static nlohmann::json outlineStats( BOARD* aBoard )
{
    SHAPE_POLY_SET outlines;

    if( !aBoard->GetBoardPolygonOutlines( outlines, false ) || outlines.OutlineCount() == 0 )
        return { { "has_outline", false } };

    double area = 0.0;

    for( int i = 0; i < outlines.OutlineCount(); ++i )
        area += outlines.Outline( i ).Area();

    const BOX2I bbox = outlines.BBox();

    return { { "has_outline", true },
             { "width_mm", mm( bbox.GetWidth() ) },
             { "height_mm", mm( bbox.GetHeight() ) },
             { "area_mm2", mm2( area ) },
             { "outlines", outlines.OutlineCount() } };
}


static KOPENAPI_RESULT h_pcb_stats( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const auto start = std::chrono::steady_clock::now();
    BOARD*     board = context->GetBoard();

    nlohmann::json layers = nlohmann::json::array();

    for( PCB_LAYER_ID layer : board->GetEnabledLayers().CuStack() )
        layers.push_back( board->GetLayerName( layer ).ToStdString() );

    nlohmann::json boardInfo = outlineStats( board );
    boardInfo["copper_layers"] = board->GetCopperLayerCount();
    boardInfo["copper_layer_names"] = layers;
    boardInfo["thickness_mm"] = mm( board->GetDesignSettings().GetBoardThickness() );

    nlohmann::json vias;
    nlohmann::json out = {
        { "document", context->GetCurrentFileName().ToStdString() },
        { "board", boardInfo },
        { "footprints", footprintStats( board ) },
        { "pads", padStats( board ) },
        { "tracks", trackStats( board, vias ) },
        { "zones", zoneStats( board ) },
        { "nets", netStats( board ) },
        { "other", otherStats( board ) }
    };

    out["vias"] = vias;

    out["elapsed_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - start )
                                .count();
    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_clearance_stats( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const auto               start = std::chrono::steady_clock::now();
    BOARD_STATISTICS_OPTIONS options;
    BOARD_STATISTICS_DATA    data;

    // Upstream's full computation: copper areas and the O(n^2) track-to-track clearance scan
    InitializeBoardStatisticsData( data );
    ComputeBoardStatistics( context->GetBoard(), options, data );

    return KOPENAPI_RESULT::Ok(
            { { "document", context->GetCurrentFileName().ToStdString() },
              { "front_copper_area_mm2", mm2( data.frontCopperArea ) },
              { "back_copper_area_mm2", mm2( data.backCopperArea ) },
              { "min_track_clearance_mm", mm( data.minClearanceTrackToTrack ) },
              { "min_track_width_mm", mm( data.minTrackWidth ) },
              { "min_drill_mm", mm( data.minDrillSize ) },
              { "drill_kinds", data.drillEntries.size() },
              { "elapsed_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - start )
                                      .count() } } );
}


KOPENAPI_REGISTER( "pcb_stats",
                   "Board statistics in one call (milliseconds): outline size/area, layers, footprints "
                   "(THT/SMD/side/DNP), pads, tracks per layer, vias, zones, nets, unrouted connections",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_stats );

KOPENAPI_REGISTER( "pcb_clearance_stats",
                   "Slow board metrics: copper areas, minimum track-to-track clearance, minimum drill; "
                   "O(n^2) in track count, can take tens of seconds on large boards",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_clearance_stats, 600 );
