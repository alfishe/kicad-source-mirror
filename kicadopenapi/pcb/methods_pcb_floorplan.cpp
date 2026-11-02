/// @file methods_pcb_floorplan.cpp
/// @brief kicadopenapi floorplan analysis (L3): the channels between the blocks of the board, their
/// capacity per copper layer and the demand of the buses and links between blocks on them.  Read only.
///
/// Blocks (their members' courtyards merged into a few boxes), all-layer keep-outs and the board outside
/// are obstacles.  Their box edges (edges closer than snap_mm on one line) cut the board into a non-uniform
/// grid of cells; a channel is a maximal free run of cell boundaries along one grid line (a cross-section
/// of the free space between two obstacles).  Cells alike merge into rectangular regions; regions meet at
/// portals.  Every bundle (the nets of one bus between two blocks, or the single nets between them) takes
/// one path portal to portal from its pads' boxes in one block to those in the other; each channel it
/// crosses (portals, and the inner grid lines of the free regions it goes through) carries its width
/// (track + clearance of each net's class).  Overflows are negotiated over channels (PathFinder costs:
/// present overuse and history).  A bus leaves and enters its end parts' boxes by the side their pads
/// face (pcb_layout_graph); a box side's free facing length per layer is its port capacity.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <geometry/shape_poly_set.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <netclass.h>
#include <netinfo.h>
#include <pad.h>
#include <project/net_settings.h>
#include <string_utils.h>
#include <zone.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


int iu( double aMm )
{
    return pcbIUScale.mmToIU( aMm );
}


double toMm( double aIU )
{
    return aIU / pcbIUScale.IU_PER_MM;
}


double r2( double aV )
{
    return std::round( aV * 100.0 ) / 100.0;
}


double r3( double aV )
{
    return std::round( aV * 1000.0 ) / 1000.0;
}


nlohmann::json boxMm( const BOX2I& aBox )
{
    return { r2( toMm( aBox.GetLeft() ) ), r2( toMm( aBox.GetTop() ) ), r2( toMm( aBox.GetRight() ) ),
             r2( toMm( aBox.GetBottom() ) ) };
}


std::optional<BOX2I> boxArg( const nlohmann::json& aV )
{
    if( !aV.is_array() || aV.size() != 4
        || !std::all_of( aV.begin(), aV.end(), []( const nlohmann::json& v ) { return v.is_number(); } ) )
    {
        return std::nullopt;
    }

    const double x0 = std::min( aV[0].get<double>(), aV[2].get<double>() );
    const double x1 = std::max( aV[0].get<double>(), aV[2].get<double>() );
    const double y0 = std::min( aV[1].get<double>(), aV[3].get<double>() );
    const double y1 = std::max( aV[1].get<double>(), aV[3].get<double>() );

    if( x1 - x0 <= 0 || y1 - y0 <= 0 )
        return std::nullopt;

    return BOX2I( VECTOR2I( iu( x0 ), iu( y0 ) ), VECTOR2I( iu( x1 - x0 ), iu( y1 - y0 ) ) );
}


struct FP_BLOCK
{
    std::string              name;
    BOX2I                    box;     ///< all of its boxes
    std::vector<std::string> parts;
    std::vector<int>         boxes;   ///< obstacle boxes (global ids)
};


/// @brief One obstacle box of a block: the courtyards of some of its members, merged
struct FP_BOX
{
    int                      block = -1;
    BOX2I                    box;
    std::vector<std::string> parts;
};


struct FP_KEEPOUT
{
    std::string name;
    BOX2I       box;
    uint64_t    layers = 0;   ///< bit per routing layer
    std::string source;       ///< board, footprint, argument
};


struct FP_CLASS
{
    std::string name;
    int         width = 0;
    int         clearance = 0;
    int         via = 0;
};


enum CELL_KIND : char
{
    CELL_FREE,
    CELL_WALL,    ///< a keep-out on every routing layer
    CELL_OUT,     ///< outside the usable outline
    CELL_BLOCK
};


struct CELL
{
    char             kind = CELL_FREE;
    bool             inside = true;
    uint64_t         blocked = 0;   ///< routing layers a keep-out takes
    int              wall = -1;
    std::vector<int> blocks;
    std::vector<int> boxes;
};


struct CUT
{
    bool                vertical = true;   ///< along y at x = at (crossed moving in x)
    int                 at = 0, from = 0, to = 0;
    std::vector<int>    free;              ///< free length per routing layer, IU
    long long           total = 0;         ///< sum of free
    int                 allLayers = 0;     ///< length free on every routing layer (vias)
    std::string         low, high;         ///< obstacles at its ends
    double              use = 0;           ///< mm
    double              hist = 0;
    std::vector<int>    links;
};


struct LINK
{
    int              a = -1, b = -1;
    std::string      bus;
    std::vector<int> nets;
    double           demand = 0;   ///< mm
    std::set<std::string> classes;
    std::set<int>    aBoxes, bBoxes;   ///< the boxes its nets leave from at a and at b
    int              aPort = -1, bPort = -1;           ///< pcb_layout_graph: the end part's box ...
    int              aSide = -1, bSide = -1;           ///< ... and the side its pads face (SIDES)
    bool             sidesKept = true;                 ///< false: no way out by those sides
    int              aExit = -1, bExit = -1;           ///< the boxes and sides its path leaves and enters by
    int              aExitSide = -1, bExitSide = -1;
    std::vector<int> cuts;
    std::set<int>    crosses;
    std::vector<int> topologyOnly;   ///< nets in the bus by their ends only, not by their name
    std::vector<std::array<double, 4>> corridor;   ///< portal crossings in path order: x, y (mm), portal, region entered
    double           length = 0;
    bool             found = false;
};


const char* const SIDES[4] = { "west", "east", "north", "south" };


int sideIndex( const std::string& aName )
{
    for( int k = 0; k < 4; ++k )
    {
        if( aName == SIDES[k] )
            return k;
    }

    return -1;
}


std::string obstacleName( const CELL& aCell, const std::vector<FP_BLOCK>& aBlocks, const std::vector<FP_KEEPOUT>& aKeepouts )
{
    if( aCell.kind == CELL_BLOCK )
        return aBlocks[aCell.blocks[0]].name;

    if( aCell.kind == CELL_WALL )
        return aKeepouts[aCell.wall].name;

    if( aCell.kind == CELL_OUT )
        return "board edge";

    return std::string();
}


void overlayShow( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aAdd )
{
    if( aCtx.headless )
        return;

    std::optional<KOPENAPI_METHOD> m = KOPENAPI_REGISTRY::Get().Find( "pcb_overlay" );

    if( !m )
        return;

    m->handler( aCtx, { { "clear", "floorplan" } } );

    if( !aAdd.empty() )
        m->handler( aCtx, { { "add", aAdd } } );
}

} // namespace


static KOPENAPI_RESULT h_pcb_floorplan( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    using clock = std::chrono::steady_clock;
    const auto started = clock::now();
    BOARD*     board = context->GetBoard();

    SHAPE_POLY_SET outline;

    if( !board->GetBoardPolygonOutlines( outline, true ) || outline.OutlineCount() == 0 )
        return KOPENAPI_RESULT::Error( 409, "the board has no outline" );

    const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
    const int edgeClearance = iu( aArgs.value( "edge_clearance_mm", toMm( ds.m_CopperEdgeClearance ) ) );
    SHAPE_POLY_SET usable = outline;

    if( edgeClearance > 0 )
        usable.Deflate( edgeClearance, CORNER_STRATEGY::ROUND_ALL_CORNERS, ds.m_MaxError );

    if( usable.OutlineCount() == 0 )
        return KOPENAPI_RESULT::Error( 409, "nothing of the board is left inside its edge clearance" );

    const int maxIterations = std::clamp( aArgs.value( "iterations", 8 ), 1, 64 );
    const bool negotiate = aArgs.value( "negotiate", true );
    const double overflowCost = std::max( 0.0, aArgs.value( "overflow_cost_mm", 5.0 ) );
    const double foreignPenalty = std::max( 1.0, aArgs.value( "through_block_factor", 10.0 ) );
    const int listLimit = KopenapiListLimit( aArgs, 40, 1 );
    const bool allChannels = aArgs.value( "all_channels", false );
    const std::string reroute = aArgs.value( "reroute", std::string( "overflowed" ) );

    if( reroute != "overflowed" && reroute != "all" )
        return KOPENAPI_RESULT::Error( 400, "reroute: overflowed or all" );

    const bool rerouteAll = reroute == "all";

    // routing layers: every copper layer, inner layers mostly covered by a rail pour are planes
    std::vector<PCB_LAYER_ID> copper;

    for( PCB_LAYER_ID l : LSET::AllCuMask( board->GetCopperLayerCount() ).CuStack() )
        copper.push_back( l );

    // blocks and rails from pcb_blocks, buses from pcb_layout_graph
    nlohmann::json pass = nlohmann::json::object();

    for( const char* key : { "by", "blocks", "max_part_pins", "depth", "max_fanout" } )
    {
        if( aArgs.contains( key ) )
            pass[key] = aArgs[key];
    }

    std::optional<KOPENAPI_METHOD> blocksMethod = KOPENAPI_REGISTRY::Get().Find( "pcb_blocks" );
    std::optional<KOPENAPI_METHOD> graphMethod = KOPENAPI_REGISTRY::Get().Find( "pcb_layout_graph" );

    if( !blocksMethod || !graphMethod )
        return KOPENAPI_RESULT::Error( 500, "pcb_blocks / pcb_layout_graph not registered" );

    nlohmann::json blocksArgs = pass;
    blocksArgs["heights"] = false;
    KOPENAPI_RESULT blocksResult = blocksMethod->handler( aCtx, blocksArgs );

    if( blocksResult.status != 200 )
        return blocksResult;

    nlohmann::json graphArgs = pass;
    graphArgs["list_limit"] = 1;

    if( aArgs.contains( "min_bus" ) )
        graphArgs["min_bus"] = aArgs["min_bus"];

    KOPENAPI_RESULT graphResult = graphMethod->handler( aCtx, graphArgs );

    if( graphResult.status != 200 )
        return graphResult;

    const auto analysed = clock::now();

    std::vector<FP_BLOCK>      blocks;
    std::map<std::string, int> blockOfRef;
    std::set<int>              rails;

    for( const nlohmann::json& j : blocksResult.body.value( "blocks", nlohmann::json::array() ) )
    {
        std::optional<BOX2I> box = boxArg( j.value( "bbox_mm", nlohmann::json() ) );

        if( !box )
            continue;

        const int id = (int) blocks.size();
        blocks.push_back( { j.value( "name", std::string() ), *box, {}, {} } );

        for( const nlohmann::json& r : j.value( "parts", nlohmann::json::array() ) )
        {
            if( r.is_string() && blockOfRef.emplace( r.get<std::string>(), id ).second )
                blocks[id].parts.push_back( r.get<std::string>() );
        }

        for( const nlohmann::json& r : j.value( "rails", nlohmann::json::array() ) )
        {
            if( NETINFO_ITEM* net = r.is_string() ? KopenapiFindNet( board, r.get<std::string>() ) : nullptr )
                rails.insert( net->GetNetCode() );
        }
    }

    std::map<int, std::string> busOfNet;
    std::set<int>              topologyOnly;   ///< bus nets whose name stem is not the bus's (topology bundles)
    const bool                 splitTopology = aArgs.value( "split_topology_only", false );
    std::map<std::string, std::map<std::string, std::pair<std::string, int>>> busEnds;   ///< bus -> block -> part, side

    for( const nlohmann::json& bus : graphResult.body.value( "buses", nlohmann::json::array() ) )
    {
        const std::string name = bus.value( "name", std::string() );

        for( const nlohmann::json& e : bus.value( "ends", nlohmann::json::array() ) )
        {
            busEnds[name][e.value( "block", std::string() )] = {
                e.value( "part", std::string() ), sideIndex( e.value( "exit", e.value( "side", std::string() ) ) ) };
        }

        const nlohmann::json netSource = bus.value( "net_source", nlohmann::json::object() );

        for( const nlohmann::json& n : bus.value( "nets", nlohmann::json::array() ) )
        {
            NETINFO_ITEM* net = n.is_string() ? KopenapiFindNet( board, n.get<std::string>() ) : nullptr;

            if( !net )
                continue;

            const bool alone = netSource.is_object() && netSource.value( n.get<std::string>(), std::string() ) == "topology";

            if( alone )
                topologyOnly.insert( net->GetNetCode() );

            if( !alone || !splitTopology )
                busOfNet.emplace( net->GetNetCode(), name );
        }
    }

    // layers
    std::vector<PCB_LAYER_ID> routing;
    nlohmann::json            layerList = nlohmann::json::array();
    const double              usableArea = usable.Area();

    if( aArgs.contains( "layers" ) )
    {
        if( !aArgs["layers"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "layers: copper layer names to route on" );

        for( const nlohmann::json& n : aArgs["layers"] )
        {
            const PCB_LAYER_ID l = n.is_string() ? board->GetLayerID( wxString::FromUTF8( n.get<std::string>() ) ) : UNDEFINED_LAYER;

            if( std::find( copper.begin(), copper.end(), l ) == copper.end() )
                return KOPENAPI_RESULT::Error( 400, "layers: " + n.dump() + " is not a copper layer of the board" );

            if( std::find( routing.begin(), routing.end(), l ) == routing.end() )
                routing.push_back( l );
        }

        for( PCB_LAYER_ID l : copper )
        {
            const bool on = std::find( routing.begin(), routing.end(), l ) != routing.end();
            layerList.push_back( { { "layer", str( board->GetLayerName( l ) ) }, { "routing", on },
                                   { "why", on ? "given" : "not given" } } );
        }
    }
    else
    {
        for( PCB_LAYER_ID l : copper )
        {
            double poured = 0;

            if( l != F_Cu && l != B_Cu )
            {
                for( ZONE* zone : board->Zones() )
                {
                    if( !zone->GetIsRuleArea() && zone->IsOnLayer( l ) && rails.count( zone->GetNetCode() ) )
                        poured += zone->Outline()->Area();
                }
            }

            const bool plane = usableArea > 0 && poured >= 0.5 * usableArea;

            if( !plane )
                routing.push_back( l );

            layerList.push_back( { { "layer", str( board->GetLayerName( l ) ) }, { "routing", !plane },
                                   { "why", plane ? "plane (a rail pour over half the board or more)" : "signal" } } );
        }
    }

    if( routing.empty() )
        return KOPENAPI_RESULT::Error( 409, "no routing layer" );

    if( routing.size() > 64 )
        routing.resize( 64 );

    const int      nl = (int) routing.size();
    const uint64_t allBits = nl >= 64 ? ~uint64_t( 0 ) : ( ( uint64_t( 1 ) << nl ) - 1 );

    auto layerBits = [&]( const LSET& aSet )
    {
        uint64_t bits = 0;

        for( int k = 0; k < nl; ++k )
        {
            if( aSet.Contains( routing[k] ) )
                bits |= uint64_t( 1 ) << k;
        }

        return bits;
    };

    // keep-outs that bar tracks: rule areas of the board and of footprints, and given ones
    std::vector<FP_KEEPOUT> keepouts;

    auto ruleArea = [&]( ZONE* aZone, const std::string& aSource )
    {
        if( !aZone->GetIsRuleArea() || !aZone->GetDoNotAllowTracks() )
            return;

        const uint64_t bits = layerBits( aZone->GetLayerSet() );

        if( !bits )
            return;

        std::string name = str( aZone->GetZoneName() );

        if( name.empty() )
            name = "keep-out " + std::to_string( keepouts.size() + 1 );

        keepouts.push_back( { name, aZone->GetBoardOutline().BBox(), bits, aSource } );
    };

    for( ZONE* zone : board->Zones() )
        ruleArea( zone, "board" );

    for( FOOTPRINT* fp : board->Footprints() )
    {
        for( ZONE* zone : fp->Zones() )
            ruleArea( zone, "footprint " + str( fp->GetReference() ) );
    }

    for( const nlohmann::json& k : aArgs.value( "keepouts", nlohmann::json::array() ) )
    {
        std::optional<BOX2I> box = k.is_object() ? boxArg( k.value( "rect", nlohmann::json() ) ) : std::nullopt;

        if( !box )
            return KOPENAPI_RESULT::Error( 400, "keepouts: [{name, rect: [x0, y0, x1, y1] mm, layers}]" );

        uint64_t bits = allBits;

        if( k.contains( "layers" ) )
        {
            if( !k["layers"].is_array() )
                return KOPENAPI_RESULT::Error( 400, "keepouts: layers is a list of copper layer names" );

            bits = 0;

            for( const nlohmann::json& n : k["layers"] )
            {
                const PCB_LAYER_ID l = n.is_string() ? board->GetLayerID( wxString::FromUTF8( n.get<std::string>() ) ) : UNDEFINED_LAYER;

                if( std::find( copper.begin(), copper.end(), l ) == copper.end() )
                    return KOPENAPI_RESULT::Error( 400, "keepouts: " + n.dump() + " is not a copper layer of the board" );

                for( int i = 0; i < nl; ++i )
                {
                    if( routing[i] == l )
                        bits |= uint64_t( 1 ) << i;
                }
            }
        }

        keepouts.push_back( { k.value( "name", "keep-out " + std::to_string( keepouts.size() + 1 ) ), *box, bits, "argument" } );
    }

    // net classes
    std::shared_ptr<NET_SETTINGS> netSettings = ds.m_NetSettings;
    std::shared_ptr<NETCLASS>     defaultClass = netSettings ? netSettings->GetDefaultNetclass() : nullptr;

    auto classRule = [&]( const NETCLASS* aClass, const std::string& aName )
    {
        FP_CLASS c;
        c.name = aName;
        const NETCLASS* d = defaultClass.get();
        c.width = aClass && aClass->HasTrackWidth() ? aClass->GetTrackWidth()
                  : d && d->HasTrackWidth()         ? d->GetTrackWidth()
                                                    : ds.m_TrackMinWidth;
        c.clearance = aClass && aClass->HasClearance() ? aClass->GetClearance()
                      : d && d->HasClearance()         ? d->GetClearance()
                                                       : ds.m_MinClearance;
        c.via = aClass && aClass->HasViaDiameter() ? aClass->GetViaDiameter()
                : d && d->HasViaDiameter()         ? d->GetViaDiameter()
                                                   : ds.m_ViasMinSize;
        c.width = std::max( c.width, ds.m_TrackMinWidth );
        c.clearance = std::max( c.clearance, ds.m_MinClearance );
        return c;
    };

    std::map<std::string, FP_CLASS> classes;
    classes["Default"] = classRule( defaultClass.get(), "Default" );

    if( netSettings )
    {
        for( const auto& [name, nc] : netSettings->GetNetclasses() )
            classes[str( name )] = classRule( nc.get(), str( name ) );
    }

    const FP_CLASS& defaults = classes["Default"];
    const double    defaultPitch = std::max( 0.01, toMm( defaults.width + defaults.clearance ) );

    // obstacles: per block its members' courtyards, boxes merged while their union stays full
    // (obstacles: bbox, one box per block as pcb_blocks gives it)
    const std::string obstacles = aArgs.value( "obstacles", std::string( "parts" ) );

    if( obstacles != "parts" && obstacles != "bbox" )
        return KOPENAPI_RESULT::Error( 400, "obstacles: parts or bbox" );

    const int    mergeGap = iu( std::max( 0.0, aArgs.value( "merge_gap_mm", toMm( defaults.width + defaults.clearance ) ) ) );
    const double minFill = std::clamp( aArgs.value( "merge_fill", 0.7 ), 0.0, 1.0 );
    const int    maxBoxes = std::clamp( aArgs.value( "max_boxes", 8 ), 1, 64 );

    std::map<std::string, FOOTPRINT*> fpOfRef;

    for( FOOTPRINT* fp : board->Footprints() )
        fpOfRef.emplace( str( fp->GetReference() ), fp );

    auto courtyardBox = []( FOOTPRINT* aFp )
    {
        const SHAPE_POLY_SET& cy = aFp->GetCourtyard( aFp->IsFlipped() ? B_CrtYd : F_CrtYd );
        return cy.OutlineCount() > 0 ? cy.BBox() : aFp->GetBoundingBox( false );
    };

    std::vector<FP_BOX>        boxes;
    std::map<std::string, int> boxOfRef;

    auto boxArea = []( const BOX2I& aB ) { return double( aB.GetWidth() ) * aB.GetHeight(); };

    for( size_t b = 0; b < blocks.size(); ++b )
    {
        std::vector<FP_BOX> own;

        for( const std::string& ref : blocks[b].parts )
        {
            auto it = fpOfRef.find( ref );

            if( it != fpOfRef.end() )
                own.push_back( { (int) b, courtyardBox( it->second ), { ref } } );
        }

        if( own.empty() )
            own.push_back( { (int) b, blocks[b].box, {} } );

        auto join = [&]( size_t i, size_t j )
        {
            own[i].box.Merge( own[j].box );
            own[i].parts.insert( own[i].parts.end(), own[j].parts.begin(), own[j].parts.end() );
            own.erase( own.begin() + (long) j );
        };

        // the fill of two boxes' union: their areas (overlap once) over the union's
        auto fillOf = [&]( size_t i, size_t j )
        {
            BOX2I u = own[i].box;
            u.Merge( own[j].box );
            const BOX2I  o = own[i].box.Intersect( own[j].box );
            const double overlap = o.GetWidth() > 0 && o.GetHeight() > 0 ? boxArea( o ) : 0;
            return boxArea( u ) > 0 ? ( boxArea( own[i].box ) + boxArea( own[j].box ) - overlap ) / boxArea( u ) : 1.0;
        };

        if( obstacles == "bbox" )
        {
            while( own.size() > 1 )
                join( 0, 1 );
        }
        else
        {
            for( bool merged = true; merged; )
            {
                merged = false;

                for( size_t i = 0; i < own.size() && !merged; ++i )
                {
                    for( size_t j = i + 1; j < own.size() && !merged; ++j )
                    {
                        BOX2I grown = own[i].box;
                        grown.Inflate( mergeGap );

                        if( grown.Intersects( own[j].box ) && fillOf( i, j ) >= minFill )
                        {
                            join( i, j );
                            merged = true;
                        }
                    }
                }
            }

            while( (int) own.size() > maxBoxes )
            {
                size_t bi = 0, bj = 1;
                double best = -1;

                for( size_t i = 0; i < own.size(); ++i )
                {
                    for( size_t j = i + 1; j < own.size(); ++j )
                    {
                        if( const double f = fillOf( i, j ); f > best )
                        {
                            best = f;
                            bi = i;
                            bj = j;
                        }
                    }
                }

                join( bi, bj );
            }
        }

        bool first = true;

        for( FP_BOX& fb : own )
        {
            for( const std::string& ref : fb.parts )
                boxOfRef[ref] = (int) boxes.size();

            if( first )
                blocks[b].box = fb.box;
            else
                blocks[b].box.Merge( fb.box );

            first = false;
            blocks[b].boxes.push_back( (int) boxes.size() );
            boxes.push_back( std::move( fb ) );
        }
    }

    // links: per signal net its blocks joined by a shortest tree; the nets of one bus between two
    // blocks, and the single nets between them, are one bundle each
    std::map<int, std::set<int>>                netBlocks;
    std::map<int, std::map<int, std::set<int>>> netBoxes;   ///< net -> block -> boxes of its pads

    for( FOOTPRINT* fp : board->Footprints() )
    {
        const std::string ref = str( fp->GetReference() );
        auto              it = blockOfRef.find( ref );

        if( it == blockOfRef.end() )
            continue;

        auto bx = boxOfRef.find( ref );

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 && !rails.count( pad->GetNetCode() ) )
            {
                netBlocks[pad->GetNetCode()].insert( it->second );

                if( bx != boxOfRef.end() )
                    netBoxes[pad->GetNetCode()][it->second].insert( bx->second );
            }
        }
    }

    std::map<std::tuple<int, int, std::string>, LINK> linkMap;

    for( const auto& [net, set] : netBlocks )
    {
        if( set.size() < 2 )
            continue;

        std::vector<int>  members( set.begin(), set.end() );
        std::vector<bool> in( members.size(), false );
        in[0] = true;

        NETINFO_ITEM*   info = board->FindNet( net );
        const NETCLASS* nc = info ? info->GetNetClass() : nullptr;
        std::string     className = nc ? str( nc->GetName() ) : "Default";

        if( !classes.count( className ) )
            classes[className] = classRule( nc, className );

        const FP_CLASS& rule = classes[className];
        const double    width = toMm( rule.width + rule.clearance );
        auto            bus = busOfNet.find( net );

        for( size_t added = 1; added < members.size(); ++added )
        {
            double best = std::numeric_limits<double>::max();
            int    bi = -1, bj = -1;

            for( size_t i = 0; i < members.size(); ++i )
            {
                if( !in[i] )
                    continue;

                for( size_t j = 0; j < members.size(); ++j )
                {
                    if( in[j] )
                        continue;

                    const VECTOR2I d = blocks[members[i]].box.Centre() - blocks[members[j]].box.Centre();
                    const double   len = std::abs( double( d.x ) ) + std::abs( double( d.y ) );

                    if( len < best )
                    {
                        best = len;
                        bi = (int) i;
                        bj = (int) j;
                    }
                }
            }

            in[bj] = true;
            const int a = std::min( members[bi], members[bj] );
            const int b = std::max( members[bi], members[bj] );
            LINK&     l = linkMap[{ a, b, bus == busOfNet.end() ? std::string() : bus->second }];
            l.a = a;
            l.b = b;
            l.bus = bus == busOfNet.end() ? std::string() : bus->second;
            l.nets.push_back( net );
            l.demand += width;

            if( topologyOnly.count( net ) && !l.bus.empty() )
                l.topologyOnly.push_back( net );

            for( int x : netBoxes[net][a] )
                l.aBoxes.insert( x );

            for( int x : netBoxes[net][b] )
                l.bBoxes.insert( x );
            l.classes.insert( className );
        }
    }

    std::vector<LINK> links;

    for( auto& [key, l] : linkMap )
        links.push_back( std::move( l ) );

    // a bus leaves (enters) a block by the side of its end part that its pads face
    const bool portSides = aArgs.value( "port_sides", true );

    for( LINK& l : links )
    {
        auto be = portSides && !l.bus.empty() ? busEnds.find( l.bus ) : busEnds.end();

        if( be == busEnds.end() )
            continue;

        for( auto [blk, port, side] : { std::tie( l.a, l.aPort, l.aSide ), std::tie( l.b, l.bPort, l.bSide ) } )
        {
            auto e = be->second.find( blocks[blk].name );

            if( e == be->second.end() || e->second.second < 0 )
                continue;

            auto bx = boxOfRef.find( e->second.first );

            if( bx != boxOfRef.end() && boxes[bx->second].block == blk )
            {
                port = bx->second;
                side = e->second.second;
            }
        }
    }

    // the grid: lines at the obstacles' edges
    BOX2I extent = usable.BBox();

    for( const FP_BLOCK& b : blocks )
        extent.Merge( b.box );

    std::vector<int> xs = { extent.GetLeft(), extent.GetRight() };
    std::vector<int> ys = { extent.GetTop(), extent.GetBottom() };
    const BOX2I      ub = usable.BBox();

    auto addLines = [&]( const BOX2I& aBox )
    {
        for( int x : { aBox.GetLeft(), aBox.GetRight() } )
            xs.push_back( std::clamp( x, extent.GetLeft(), extent.GetRight() ) );

        for( int y : { aBox.GetTop(), aBox.GetBottom() } )
            ys.push_back( std::clamp( y, extent.GetTop(), extent.GetBottom() ) );
    };

    // lines closer than snap_mm are one line, except the two edges of an obstacle thinner than that
    const int          snapTol = iu( std::max( 0.0, aArgs.value( "snap_mm", defaultPitch ) ) );
    std::map<int, int> thinX, thinY;   ///< far edge -> the largest near edge of a thin obstacle

    auto thin = [&]( std::map<int, int>& aMap, int aLow, int aHigh )
    {
        if( aHigh - aLow >= snapTol || aHigh <= aLow )
            return;

        auto [it, added] = aMap.emplace( aHigh, aLow );

        if( !added )
            it->second = std::max( it->second, aLow );
    };

    auto addObstacle = [&]( const BOX2I& aBox )
    {
        addLines( aBox );
        thin( thinX, std::clamp( aBox.GetLeft(), extent.GetLeft(), extent.GetRight() ),
              std::clamp( aBox.GetRight(), extent.GetLeft(), extent.GetRight() ) );
        thin( thinY, std::clamp( aBox.GetTop(), extent.GetTop(), extent.GetBottom() ),
              std::clamp( aBox.GetBottom(), extent.GetTop(), extent.GetBottom() ) );
    };

    addLines( ub );

    for( const FP_BOX& b : boxes )
        addObstacle( b.box );

    for( const FP_KEEPOUT& k : keepouts )
        addObstacle( k.box );

    std::vector<int> rawX, rawY, repX, repY;
    int              rawLines = 0;

    auto snapLines = [&]( std::vector<int>& aLines, const std::map<int, int>& aThin, std::vector<int>& aRaw,
                          std::vector<int>& aRep )
    {
        std::sort( aLines.begin(), aLines.end() );
        aLines.erase( std::unique( aLines.begin(), aLines.end() ), aLines.end() );
        aRaw = aLines;
        rawLines += (int) aRaw.size();
        aRep.assign( aRaw.size(), aRaw.front() );
        aLines.assign( 1, aRaw.front() );

        for( size_t k = 1; k < aRaw.size(); ++k )
        {
            auto       t = aThin.find( aRaw[k] );
            const bool keep = t != aThin.end() && t->second >= aLines.back();

            if( aRaw[k] - aLines.back() >= snapTol || keep )
                aLines.push_back( aRaw[k] );

            aRep[k] = aLines.back();
        }

        // the extent's far edge stays where it is
        if( aLines.back() != aRaw.back() )
        {
            const int old = aLines.back();
            aLines.back() = aRaw.back();
            std::replace( aRep.begin(), aRep.end(), old, aRaw.back() );
        }
    };

    snapLines( xs, thinX, rawX, repX );
    snapLines( ys, thinY, rawY, repY );

    auto snapTo = []( const std::vector<int>& aRaw, const std::vector<int>& aRep, int aV )
    {
        const auto it = std::lower_bound( aRaw.begin(), aRaw.end(), aV );
        const size_t k = std::min( size_t( it - aRaw.begin() ), aRaw.size() - 1 );
        return aRep[k];
    };

    const int nx = (int) xs.size() - 1, ny = (int) ys.size() - 1;

    if( nx < 1 || ny < 1 )
        return KOPENAPI_RESULT::Error( 409, "the board outline has no area" );

    std::vector<CELL> cells( size_t( nx ) * ny );
    auto at = [&]( int i, int j ) -> CELL& { return cells[size_t( j ) * nx + i]; };

    // the cells a box takes (i0 .. i1 - 1); one at least when it has an area
    auto span = [&]( const std::vector<int>& aLines, const std::vector<int>& aRaw, const std::vector<int>& aRep,
                     int aLow, int aHigh )
    {
        const int last = (int) aLines.size() - 1;
        const int low = snapTo( aRaw, aRep, std::clamp( aLow, aRaw.front(), aRaw.back() ) );
        const int high = snapTo( aRaw, aRep, std::clamp( aHigh, aRaw.front(), aRaw.back() ) );
        const int i0 = std::min( int( std::lower_bound( aLines.begin(), aLines.end(), low ) - aLines.begin() ), last );
        int       i1 = std::min( int( std::lower_bound( aLines.begin(), aLines.end(), high ) - aLines.begin() ), last );

        if( i1 <= i0 && aHigh > aLow && aLow < aRaw.back() && aHigh > aRaw.front() )
            i1 = std::min( i0 + 1, last );

        return std::make_pair( i0, i1 );
    };

    auto spanX = [&]( const BOX2I& aBox ) { return span( xs, rawX, repX, aBox.GetLeft(), aBox.GetRight() ); };
    auto spanY = [&]( const BOX2I& aBox ) { return span( ys, rawY, repY, aBox.GetTop(), aBox.GetBottom() ); };

    for( int j = 0; j < ny; ++j )
    {
        for( int i = 0; i < nx; ++i )
        {
            const VECTOR2I c( int( ( int64_t( xs[i] ) + xs[i + 1] ) / 2 ), int( ( int64_t( ys[j] ) + ys[j + 1] ) / 2 ) );
            at( i, j ).inside = usable.Contains( c );
            at( i, j ).kind = at( i, j ).inside ? CELL_FREE : CELL_OUT;
        }
    }

    for( size_t k = 0; k < keepouts.size(); ++k )
    {
        const auto [i0, i1] = spanX( keepouts[k].box );
        const auto [j0, j1] = spanY( keepouts[k].box );

        for( int j = j0; j < j1; ++j )
        {
            for( int i = i0; i < i1; ++i )
            {
                CELL& c = at( i, j );
                c.blocked |= keepouts[k].layers;

                if( ( c.blocked & allBits ) == allBits && c.kind == CELL_FREE )
                {
                    c.kind = CELL_WALL;
                    c.wall = (int) k;
                }
            }
        }
    }

    std::vector<std::vector<int>> boxCells( boxes.size() );
    std::vector<std::array<int, 4>> boxSpan( boxes.size() );   ///< cells i0, i1, j0, j1 (ends exclusive)

    for( size_t k = 0; k < boxes.size(); ++k )
    {
        const auto [i0, i1] = spanX( boxes[k].box );
        const auto [j0, j1] = spanY( boxes[k].box );
        const int  b = boxes[k].block;
        boxSpan[k] = { i0, i1, j0, j1 };

        for( int j = j0; j < j1; ++j )
        {
            for( int i = i0; i < i1; ++i )
            {
                CELL& c = at( i, j );
                c.kind = CELL_BLOCK;
                c.boxes.push_back( (int) k );

                if( std::find( c.blocks.begin(), c.blocks.end(), b ) == c.blocks.end() )
                    c.blocks.push_back( b );

                boxCells[k].push_back( j * nx + i );
            }
        }
    }

    // channels: maximal free runs of cell boundaries along each grid line
    std::vector<CUT> cuts;
    std::vector<int> vEdge( size_t( std::max( 0, nx - 1 ) ) * ny, -1 );   // (i, j) | (i + 1, j)
    std::vector<int> hEdge( size_t( nx ) * std::max( 0, ny - 1 ), -1 );   // (i, j) | (i, j + 1)

    auto endName = [&]( const CELL* aOne, const CELL* aOther )
    {
        if( !aOne )
            return std::string( "board edge" );

        std::string n = obstacleName( *aOne, blocks, keepouts );
        return n.empty() ? obstacleName( *aOther, blocks, keepouts ) : n;
    };

    for( int line = 0; line + 1 < nx; ++line )
    {
        for( int j = 0; j < ny; )
        {
            auto freeAt = [&]( int r ) { return at( line, r ).kind == CELL_FREE && at( line + 1, r ).kind == CELL_FREE; };

            if( !freeAt( j ) )
            {
                ++j;
                continue;
            }

            CUT cut;
            cut.vertical = true;
            cut.at = xs[line + 1];
            cut.from = ys[j];
            cut.free.assign( nl, 0 );
            const int j0 = j;

            for( ; j < ny && freeAt( j ); ++j )
            {
                const int      len = ys[j + 1] - ys[j];
                const uint64_t blocked = at( line, j ).blocked | at( line + 1, j ).blocked;

                for( int k = 0; k < nl; ++k )
                {
                    if( !( blocked & ( uint64_t( 1 ) << k ) ) )
                        cut.free[k] += len;
                }

                if( !( blocked & allBits ) )
                    cut.allLayers += len;

                vEdge[size_t( j ) * ( nx - 1 ) + line] = (int) cuts.size();
            }

            cut.to = ys[j];
            cut.low = j0 > 0 ? endName( &at( line, j0 - 1 ), &at( line + 1, j0 - 1 ) ) : "board edge";
            cut.high = j < ny ? endName( &at( line, j ), &at( line + 1, j ) ) : "board edge";

            for( int f : cut.free )
                cut.total += f;

            cuts.push_back( std::move( cut ) );
        }
    }

    for( int line = 0; line + 1 < ny; ++line )
    {
        for( int i = 0; i < nx; )
        {
            auto freeAt = [&]( int c ) { return at( c, line ).kind == CELL_FREE && at( c, line + 1 ).kind == CELL_FREE; };

            if( !freeAt( i ) )
            {
                ++i;
                continue;
            }

            CUT cut;
            cut.vertical = false;
            cut.at = ys[line + 1];
            cut.from = xs[i];
            cut.free.assign( nl, 0 );
            const int i0 = i;

            for( ; i < nx && freeAt( i ); ++i )
            {
                const int      len = xs[i + 1] - xs[i];
                const uint64_t blocked = at( i, line ).blocked | at( i, line + 1 ).blocked;

                for( int k = 0; k < nl; ++k )
                {
                    if( !( blocked & ( uint64_t( 1 ) << k ) ) )
                        cut.free[k] += len;
                }

                if( !( blocked & allBits ) )
                    cut.allLayers += len;

                hEdge[size_t( line ) * nx + i] = (int) cuts.size();
            }

            cut.to = xs[i];
            cut.low = i0 > 0 ? endName( &at( i0 - 1, line ), &at( i0 - 1, line + 1 ) ) : "board edge";
            cut.high = i < nx ? endName( &at( i, line ), &at( i, line + 1 ) ) : "board edge";

            for( int f : cut.free )
                cut.total += f;

            cuts.push_back( std::move( cut ) );
        }
    }

    auto capacity = [&]( const CUT& aCut ) { return toMm( double( aCut.total ) ); };

    const auto gridDone = clock::now();

    // regions: rectangles of cells alike (free, or the same boxes), rows' runs merged downwards
    struct REGION
    {
        int              i0, i1, j0, j1;
        int              cell;
        std::vector<int> portals;
        int              vOff = 0, vCount = 0, hOff = 0, hCount = 0;   ///< inner lines (free regions)
    };

    struct PORTAL
    {
        int    r0, r1;     ///< r0 left of / above r1
        int    cut;
        bool   vertical;   ///< at x = at, from lo to hi in y (else at y = at, lo .. hi in x), mm
        double at, lo, hi;
    };

    auto alike = []( const CELL& aOne, const CELL& aOther )
    {
        if( aOne.kind != aOther.kind )
            return false;

        return aOne.kind == CELL_FREE || ( aOne.inside == aOther.inside && aOne.boxes == aOther.boxes );
    };

    auto passableKind = []( const CELL& aCell ) { return aCell.kind == CELL_FREE || aCell.kind == CELL_BLOCK; };

    const size_t        n = cells.size();
    std::vector<int>    regionOf( n, -1 );
    std::vector<REGION> regions;

    for( int j = 0; j < ny; ++j )
    {
        for( int i = 0; i < nx; )
        {
            const CELL& c = at( i, j );

            if( !passableKind( c ) )
            {
                ++i;
                continue;
            }

            const int i0 = i;

            while( i < nx && alike( at( i, j ), c ) )
                ++i;

            int r = j > 0 ? regionOf[size_t( j - 1 ) * nx + i0] : -1;

            if( r >= 0 && regions[r].i0 == i0 && regions[r].i1 == i && regions[r].j1 == j && alike( cells[regions[r].cell], c ) )
            {
                regions[r].j1 = j + 1;
            }
            else
            {
                r = (int) regions.size();
                regions.push_back( { i0, i, j, j + 1, j * nx + i0, {}, 0, 0, 0, 0 } );
            }

            for( int k = i0; k < i; ++k )
                regionOf[size_t( j ) * nx + k] = r;
        }
    }

    // portals: the boundary two regions share (one segment, on one channel when both are free)
    std::vector<PORTAL> portals;

    auto addPortal = [&]( int aR0, int aR1, int aCut, bool aVertical, int aAt, int aLow, int aHigh )
    {
        regions[aR0].portals.push_back( (int) portals.size() );
        regions[aR1].portals.push_back( (int) portals.size() );
        portals.push_back( { aR0, aR1, aCut, aVertical, toMm( aAt ), toMm( aLow ), toMm( aHigh ) } );
    };

    for( int i = 0; i + 1 < nx; ++i )
    {
        for( int j = 0; j < ny; )
        {
            const int r0 = regionOf[size_t( j ) * nx + i], r1 = regionOf[size_t( j ) * nx + i + 1];

            if( r0 < 0 || r1 < 0 || r0 == r1 )
            {
                ++j;
                continue;
            }

            const int j0 = j;

            while( j < ny && regionOf[size_t( j ) * nx + i] == r0 && regionOf[size_t( j ) * nx + i + 1] == r1 )
                ++j;

            addPortal( r0, r1, vEdge[size_t( j0 ) * ( nx - 1 ) + i], true, xs[i + 1], ys[j0], ys[j] );
        }
    }

    for( int j = 0; j + 1 < ny; ++j )
    {
        for( int i = 0; i < nx; )
        {
            const int r0 = regionOf[size_t( j ) * nx + i], r1 = regionOf[size_t( j + 1 ) * nx + i];

            if( r0 < 0 || r1 < 0 || r0 == r1 )
            {
                ++i;
                continue;
            }

            const int i0 = i;

            while( i < nx && regionOf[size_t( j ) * nx + i] == r0 && regionOf[size_t( j + 1 ) * nx + i] == r1 )
                ++i;

            addPortal( r0, r1, hEdge[size_t( j ) * nx + i0], false, ys[j + 1], xs[i0], xs[i] );
        }
    }

    // a free region's inner grid lines: one channel each over the region (crossed inside it)
    std::vector<int>    lineCut;
    std::vector<double> lineAt, linePre;

    for( REGION& r : regions )
    {
        if( cells[r.cell].kind != CELL_FREE )
            continue;

        r.vOff = (int) lineCut.size();
        r.vCount = r.i1 - r.i0 - 1;

        for( int i = r.i0 + 1; i < r.i1; ++i )
        {
            lineCut.push_back( vEdge[size_t( r.j0 ) * ( nx - 1 ) + i - 1] );
            lineAt.push_back( toMm( xs[i] ) );
        }

        lineCut.push_back( -1 );
        lineAt.push_back( 0 );
        r.hOff = (int) lineCut.size();
        r.hCount = r.j1 - r.j0 - 1;

        for( int j = r.j0 + 1; j < r.j1; ++j )
        {
            lineCut.push_back( hEdge[size_t( j - 1 ) * nx + r.i0] );
            lineAt.push_back( toMm( ys[j] ) );
        }

        lineCut.push_back( -1 );
        lineAt.push_back( 0 );
    }

    linePre.assign( lineCut.size(), 0 );

    std::vector<std::vector<int>> boxRegions( boxes.size() );

    for( size_t k = 0; k < boxes.size(); ++k )
    {
        for( int c : boxCells[k] )
        {
            if( regionOf[c] >= 0 )
                boxRegions[k].push_back( regionOf[c] );
        }

        std::sort( boxRegions[k].begin(), boxRegions[k].end() );
        boxRegions[k].erase( std::unique( boxRegions[k].begin(), boxRegions[k].end() ), boxRegions[k].end() );
    }

    const auto regionsDone = clock::now();

    // one bundle's path: A* over portals (entered towards one of their regions) from its pads' boxes in
    // one block to those in the other; a node's point on its portal is the one nearest the point before
    // (at the start: nearest the target)
    const size_t          nodes = portals.size() * 2;
    std::vector<double>   cost( nodes ), cutCostAt( cuts.size() ), pointX( nodes ), pointY( nodes );
    std::vector<int>      prev( nodes );
    std::vector<unsigned> seenAt( nodes, 0 ), doneAt( nodes, 0 ), targetAt( regions.size(), 0 ),
            preAt( regions.size(), 0 ), cutAt( cuts.size(), 0 ), factorAt( regions.size(), 0 );
    std::vector<double>   regionFactor( regions.size(), 1.0 );
    std::vector<char>     regionFree( regions.size() );

    for( size_t r = 0; r < regions.size(); ++r )
        regionFree[r] = cells[regions[r].cell].kind == CELL_FREE;
    unsigned              epoch = 0;

    auto has = []( const CELL& aCell, int aBlock )
    {
        return std::find( aCell.blocks.begin(), aCell.blocks.end(), aBlock ) != aCell.blocks.end();
    };

    auto entered = [&]( int aNode ) { return aNode & 1 ? portals[aNode >> 1].r1 : portals[aNode >> 1].r0; };

    auto nearest = []( const PORTAL& aP, double aX, double aY )
    {
        return aP.vertical ? std::make_pair( aP.at, std::clamp( aY, aP.lo, aP.hi ) )
                           : std::make_pair( std::clamp( aX, aP.lo, aP.hi ), aP.at );
    };

    // inner lines of a list between two coordinates: [t0, t1)
    auto innerRange = [&]( int aOff, int aCount, double aOne, double aOther )
    {
        const double lo = std::min( aOne, aOther ), hi = std::max( aOne, aOther );
        const auto   b = lineAt.begin() + aOff, e = b + aCount;
        return std::make_pair( aOff + int( std::upper_bound( b, e, lo ) - b ), aOff + int( std::lower_bound( b, e, hi ) - b ) );
    };

    // the side of a box a portal lies on, -1: none
    auto portalSide = [&]( const PORTAL& aP, int aBox )
    {
        const std::array<int, 4>& sp = boxSpan[aBox];

        if( aP.vertical )
            return aP.at == toMm( xs[sp[0]] ) ? 0 : aP.at == toMm( xs[sp[1]] ) ? 1 : -1;

        return aP.at == toMm( ys[sp[2]] ) ? 2 : aP.at == toMm( ys[sp[3]] ) ? 3 : -1;
    };

    bool anyHistory = false;

    auto routeLink = [&]( LINK& aLink, double aPresent, bool aSides )
    {
        const bool   aOn = aSides && aLink.aPort >= 0, bOn = aSides && aLink.bPort >= 0;
        const std::set<int> fromBoxes = aOn                   ? std::set<int>{ aLink.aPort }
                                        : aLink.aBoxes.empty() ? std::set<int>( blocks[aLink.a].boxes.begin(), blocks[aLink.a].boxes.end() )
                                                               : aLink.aBoxes;
        const std::set<int> toBoxes = bOn                   ? std::set<int>{ aLink.bPort }
                                      : aLink.bBoxes.empty() ? std::set<int>( blocks[aLink.b].boxes.begin(), blocks[aLink.b].boxes.end() )
                                                             : aLink.bBoxes;
        BOX2I target = boxes[*toBoxes.begin()].box;

        for( int k : toBoxes )
            target.Merge( boxes[k].box );

        const double tx0 = toMm( target.GetLeft() ), tx1 = toMm( target.GetRight() );
        const double ty0 = toMm( target.GetTop() ), ty1 = toMm( target.GetBottom() );

        auto h = [&]( double aX, double aY )
        {
            const double dx = aX < tx0 ? tx0 - aX : aX > tx1 ? aX - tx1 : 0;
            const double dy = aY < ty0 ? ty0 - aY : aY > ty1 ? aY - ty1 : 0;
            return dx + dy;
        };

        ++epoch;

        auto cutCost = [&]( int aCut )
        {
            if( aCut < 0 )
                return 0.0;

            if( cutAt[aCut] != epoch )
            {
                const CUT&   cut = cuts[aCut];
                const double over = std::max( 0.0, cut.use + aLink.demand - capacity( cut ) ) / defaultPitch;
                cutCostAt[aCut] = ( aPresent * over + cut.hist ) * overflowCost;
                cutAt[aCut] = epoch;
            }

            return cutCostAt[aCut];
        };

        auto prefix = [&]( int aRegion )
        {
            REGION& r = regions[aRegion];

            if( preAt[aRegion] == epoch )
                return;

            preAt[aRegion] = epoch;

            for( auto [off, count] : { std::make_pair( r.vOff, r.vCount ), std::make_pair( r.hOff, r.hCount ) } )
            {
                double sum = 0;

                for( int t = 0; t <= count; ++t )
                {
                    linePre[off + t] = sum;

                    if( t < count )
                        sum += cutCost( lineCut[off + t] );
                }
            }
        };

        // the cost of the inner lines crossed going from one point to another in a free region
        auto innerCost = [&]( int aRegion, double aX0, double aY0, double aX1, double aY1 )
        {
            const REGION& r = regions[aRegion];

            if( ( r.vCount <= 0 && r.hCount <= 0 ) || ( aPresent <= 0 && !anyHistory ) )
                return 0.0;

            prefix( aRegion );
            const auto [v0, v1] = innerRange( r.vOff, r.vCount, aX0, aX1 );
            const auto [h0, h1] = innerRange( r.hOff, r.hCount, aY0, aY1 );
            return linePre[v1] - linePre[v0] + linePre[h1] - linePre[h0];
        };

        // 0: no way in; else the cost factor per mm inside
        auto factorOf = [&]( int aRegion )
        {
            if( factorAt[aRegion] != epoch )
            {
                const CELL& c = cells[regions[aRegion].cell];
                factorAt[aRegion] = epoch;
                regionFactor[aRegion] = c.kind != CELL_BLOCK || has( c, aLink.a ) || has( c, aLink.b ) ? 1.0
                                        : c.inside                                                  ? foreignPenalty
                                                                                                    : 0.0;
            }

            return regionFactor[aRegion];
        };

        aLink.cuts.clear();
        aLink.corridor.clear();
        aLink.crosses.clear();
        aLink.length = 0;
        aLink.found = false;
        aLink.aExit = aLink.bExit = aLink.aExitSide = aLink.bExitSide = -1;

        // a node into the target by its side only
        auto intoTarget = [&]( int aNode )
        {
            return targetAt[entered( aNode )] == epoch
                   && ( !bOn || portalSide( portals[aNode >> 1], aLink.bPort ) == aLink.bSide );
        };

        auto barred = [&]( int aNode )
        {
            return factorOf( entered( aNode ) ) <= 0 || ( targetAt[entered( aNode )] == epoch && !intoTarget( aNode ) );
        };

        std::vector<int> starts;

        for( int k : toBoxes )
        {
            for( int r : boxRegions[k] )
                targetAt[r] = epoch;
        }

        for( int k : fromBoxes )
        {
            for( int r : boxRegions[k] )
            {
                if( targetAt[r] == epoch )
                {
                    aLink.found = true;
                    return;
                }

                starts.push_back( r );
            }
        }

        using ENTRY = std::pair<double, int>;
        std::priority_queue<ENTRY, std::vector<ENTRY>, std::greater<ENTRY>> open;

        auto relax = [&]( int aNode, double aCost, int aPrev, std::pair<double, double> aAt )
        {
            if( seenAt[aNode] != epoch || aCost < cost[aNode] )
            {
                seenAt[aNode] = epoch;
                cost[aNode] = aCost;
                prev[aNode] = aPrev;
                pointX[aNode] = aAt.first;
                pointY[aNode] = aAt.second;
                open.emplace( aCost + h( aAt.first, aAt.second ) * ( 1 + 1e-6 ), aNode );   // ties: nearer the target first
            }
        };

        const double cx = ( tx0 + tx1 ) / 2, cy = ( ty0 + ty1 ) / 2;

        for( int r : starts )
        {
            for( int p : regions[r].portals )
            {
                const int node = 2 * p + ( portals[p].r0 == r ? 1 : 0 );

                if( aOn && portalSide( portals[p], aLink.aPort ) != aLink.aSide )
                    continue;

                if( !barred( node ) )
                    relax( node, cutCost( portals[p].cut ), -1, nearest( portals[p], cx, cy ) );
            }
        }

        int reached = -1;

        while( !open.empty() )
        {
            const int node = open.top().second;
            open.pop();

            if( doneAt[node] == epoch )
                continue;

            doneAt[node] = epoch;
            const int r = entered( node );

            if( intoTarget( node ) )
            {
                reached = node;
                break;
            }

            const double  x = pointX[node], y = pointY[node];
            const double  factor = factorOf( r );
            const bool    inFree = regionFree[r];

            for( int q : regions[r].portals )
            {
                if( q == ( node >> 1 ) )
                    continue;

                const PORTAL& to = portals[q];
                const int     next = 2 * q + ( to.r0 == r ? 1 : 0 );

                if( doneAt[next] == epoch || barred( next ) )
                    continue;

                const auto   at = nearest( to, x, y );
                const double step = ( std::abs( at.first - x ) + std::abs( at.second - y ) ) * factor
                                    + ( inFree ? innerCost( r, x, y, at.first, at.second ) : 0.0 ) + cutCost( to.cut );
                relax( next, cost[node] + step, node, at );
            }
        }

        if( reached < 0 )
            return;

        aLink.found = true;
        std::vector<int> path;

        for( int k = reached; k >= 0; k = prev[k] )
            path.push_back( k );

        std::reverse( path.begin(), path.end() );

        // the boxes and sides it leaves and enters by
        auto boxOf = [&]( const std::set<int>& aBoxes, int aRegion )
        {
            for( int k : aBoxes )
            {
                if( std::binary_search( boxRegions[k].begin(), boxRegions[k].end(), aRegion ) )
                    return k;
            }

            return -1;
        };

        const PORTAL& first = portals[path.front() >> 1];
        const PORTAL& last = portals[path.back() >> 1];
        aLink.aExit = boxOf( fromBoxes, path.front() & 1 ? first.r0 : first.r1 );
        aLink.bExit = boxOf( toBoxes, entered( path.back() ) );
        aLink.aExitSide = aLink.aExit >= 0 ? portalSide( first, aLink.aExit ) : -1;
        aLink.bExitSide = aLink.bExit >= 0 ? portalSide( last, aLink.bExit ) : -1;

        for( size_t k = 0; k < path.size(); ++k )
        {
            const PORTAL& to = portals[path[k] >> 1];
            aLink.corridor.push_back( { pointX[path[k]], pointY[path[k]], double( path[k] >> 1 ), double( entered( path[k] ) ) } );

            if( k > 0 )
            {
                const int    r = entered( path[k - 1] );
                const double x0 = pointX[path[k - 1]], y0 = pointY[path[k - 1]];
                const double x1 = pointX[path[k]], y1 = pointY[path[k]];
                const CELL&  c = cells[regions[r].cell];
                aLink.length += std::abs( x1 - x0 ) + std::abs( y1 - y0 );

                if( c.kind == CELL_BLOCK && !has( c, aLink.a ) && !has( c, aLink.b ) )
                    aLink.crosses.insert( c.blocks[0] );

                if( c.kind == CELL_FREE )
                {
                    const REGION& reg = regions[r];
                    const auto [v0, v1] = innerRange( reg.vOff, reg.vCount, x0, x1 );
                    const auto [h0, h1] = innerRange( reg.hOff, reg.hCount, y0, y1 );

                    // in the order a path crosses them
                    std::vector<int> inner;

                    for( int t = v0; t < v1; ++t )
                        inner.push_back( t );

                    if( x1 < x0 )
                        std::reverse( inner.begin(), inner.end() );

                    for( int t : inner )
                    {
                        if( lineCut[t] >= 0 )
                            aLink.cuts.push_back( lineCut[t] );
                    }

                    inner.clear();

                    for( int t = h0; t < h1; ++t )
                        inner.push_back( t );

                    if( y1 < y0 )
                        std::reverse( inner.begin(), inner.end() );

                    for( int t : inner )
                    {
                        if( lineCut[t] >= 0 )
                            aLink.cuts.push_back( lineCut[t] );
                    }
                }
            }

            if( to.cut >= 0 )
                aLink.cuts.push_back( to.cut );
        }
    };

    auto apply = [&]( const LINK& aLink, double aSign )
    {
        for( int c : aLink.cuts )
            cuts[c].use += aSign * aLink.demand;
    };

    std::vector<int> order( links.size() );

    for( size_t k = 0; k < links.size(); ++k )
        order[k] = (int) k;

    std::stable_sort( order.begin(), order.end(), [&]( int x, int y ) { return links[x].demand > links[y].demand; } );

    auto overflowOf = [&]( double& aTotal )
    {
        int count = 0;
        aTotal = 0;

        for( const CUT& c : cuts )
        {
            const double over = c.use - capacity( c );

            if( over > 1e-6 )
            {
                ++count;
                aTotal += over;
            }
        }

        return count;
    };

    std::vector<LINK> best;
    int               bestCount = std::numeric_limits<int>::max();
    double            bestTotal = 0;
    int               bestIteration = 0;
    nlohmann::json    history = nlohmann::json::array();
    const int         iterations = negotiate ? maxIterations : 1;
    int               ran = 0;

    for( int it = 0; it < iterations; ++it )
    {
        const double present = it == 0 ? 0.0 : std::pow( 2.0, it - 1 );

        int rerouted = 0;

        for( int k : order )
        {
            if( it > 0 && !rerouteAll )
            {
                bool over = false;

                for( int c : links[k].cuts )
                {
                    if( cuts[c].use - capacity( cuts[c] ) > 1e-6 )
                    {
                        over = true;
                        break;
                    }
                }

                if( !over )
                    continue;
            }

            apply( links[k], -1 );
            const bool sided = links[k].sidesKept && ( links[k].aPort >= 0 || links[k].bPort >= 0 );
            routeLink( links[k], present, links[k].sidesKept );

            if( sided && !links[k].found )
            {
                routeLink( links[k], present, false );
                links[k].sidesKept = false;
            }

            apply( links[k], +1 );
            ++rerouted;
        }

        ++ran;
        double    total = 0;
        const int count = overflowOf( total );
        history.push_back( { { "iteration", it + 1 }, { "overflows", count }, { "over_mm", r2( total ) }, { "rerouted", rerouted } } );

        if( count < bestCount || ( count == bestCount && total < bestTotal - 1e-9 ) )
        {
            best = links;
            bestCount = count;
            bestTotal = total;
            bestIteration = it + 1;
        }

        if( count == 0 )
            break;

        for( CUT& c : cuts )
        {
            const double over = c.use - capacity( c );

            if( over > 1e-6 )
            {
                c.hist += over / defaultPitch;
                anyHistory = true;
            }
        }
    }

    links = best;
    const auto routed = clock::now();

    for( CUT& c : cuts )
    {
        c.use = 0;
        c.links.clear();
    }

    for( size_t k = 0; k < links.size(); ++k )
    {
        apply( links[k], +1 );

        for( int c : links[k].cuts )
        {
            if( cuts[c].links.empty() || cuts[c].links.back() != (int) k )
                cuts[c].links.push_back( (int) k );
        }
    }

    // ports: a box side the bundles leave or enter by, its facing free length per layer the capacity
    struct PORT
    {
        double           demand = 0;
        int              nets = 0;
        bool             given = false;
        std::vector<int> links;
    };

    std::map<std::pair<int, int>, PORT> ports;

    for( size_t k = 0; k < links.size(); ++k )
    {
        const LINK& l = links[k];

        for( auto [box, side, port] : { std::make_tuple( l.aExit, l.aExitSide, l.aPort ),
                                        std::make_tuple( l.bExit, l.bExitSide, l.bPort ) } )
        {
            if( !l.found || box < 0 || side < 0 )
                continue;

            PORT& pt = ports[{ box, side }];
            pt.demand += l.demand;
            pt.nets += (int) l.nets.size();
            pt.given |= l.sidesKept && port == box;
            pt.links.push_back( (int) k );
        }
    }

    // the facing length a side may be left by (free, or another box: tracks over it), per layer
    auto portFree = [&]( int aBox, int aSide, int& aLength, int& aBoxes )
    {
        const std::array<int, 4>& sp = boxSpan[aBox];
        std::vector<int>          facing( nl, 0 );
        aLength = 0;
        aBoxes = 0;
        const bool across = aSide < 2;   // west / east: along y
        const int  line = aSide == 0 ? sp[0] - 1 : aSide == 1 ? sp[1] : aSide == 2 ? sp[2] - 1 : sp[3];

        if( line < 0 || line >= ( across ? nx : ny ) )
            return facing;

        for( int t = across ? sp[2] : sp[0]; t < ( across ? sp[3] : sp[1] ); ++t )
        {
            const CELL& c = across ? at( line, t ) : at( t, line );

            if( c.kind != CELL_FREE && c.kind != CELL_BLOCK )
                continue;

            const int len = across ? ys[t + 1] - ys[t] : xs[t + 1] - xs[t];
            aLength += len;

            if( c.kind == CELL_BLOCK )
                aBoxes += len;

            for( int k = 0; k < nl; ++k )
            {
                if( !( c.blocked & ( uint64_t( 1 ) << k ) ) )
                    facing[k] += len;
            }
        }

        return facing;
    };

    // answer
    auto netName = [&]( int aCode )
    {
        NETINFO_ITEM* net = board->FindNet( aCode );
        return net ? str( UnescapeString( net->GetNetname() ) ) : std::string();
    };

    auto linkName = [&]( const LINK& aLink )
    {
        return ( aLink.bus.empty() ? std::string( "singles" ) : aLink.bus ) + ": " + blocks[aLink.a].name + " - "
               + blocks[aLink.b].name;
    };

    auto cutId = [&]( int aCut ) { return "C" + std::to_string( aCut + 1 ); };

    std::set<std::string> usedClasses = { "Default" };

    for( const LINK& l : links )
        usedClasses.insert( l.classes.begin(), l.classes.end() );

    auto channelJson = [&]( int aCut )
    {
        const CUT&     c = cuts[aCut];
        nlohmann::json perLayer = nlohmann::json::object();

        for( int k = 0; k < nl; ++k )
            perLayer[str( board->GetLayerName( routing[k] ) )] = r2( toMm( c.free[k] ) );

        nlohmann::json tracks = nlohmann::json::object();

        for( const std::string& name : usedClasses )
        {
            const FP_CLASS& rule = classes[name];
            const int       pitch = std::max( 1, rule.width + rule.clearance );
            int             sum = 0;
            nlohmann::json  layers = nlohmann::json::object();

            for( int k = 0; k < nl; ++k )
            {
                const int t = c.free[k] / pitch;
                layers[str( board->GetLayerName( routing[k] ) )] = t;
                sum += t;
            }

            tracks[name] = { { "per_layer", layers }, { "total", sum } };
        }

        const double cap = capacity( c );
        nlohmann::json linkList = nlohmann::json::array();
        int            nets = 0;

        for( int k : c.links )
        {
            nets += (int) links[k].nets.size();

            if( (int) linkList.size() < listLimit )
                linkList.push_back( linkName( links[k] ) );
        }

        const BOX2I area = c.vertical ? BOX2I( VECTOR2I( c.at, c.from ), VECTOR2I( 0, c.to - c.from ) )
                                      : BOX2I( VECTOR2I( c.from, c.at ), VECTOR2I( c.to - c.from, 0 ) );

        return nlohmann::json{ { "id", cutId( aCut ) },
                               { "cut", c.vertical ? "vertical" : "horizontal" },
                               { "line_mm", boxMm( area ) },
                               { "length_mm", r2( toMm( c.to - c.from ) ) },
                               { "between", { c.low, c.high } },
                               { "free_mm", perLayer },
                               { "capacity_mm", r2( cap ) },
                               { "tracks", tracks },
                               { "vias_across", c.allLayers / std::max( 1, defaults.via + defaults.clearance ) },
                               { "demand_mm", r2( c.use ) },
                               { "demand_nets", nets },
                               { "use", cap > 0 ? r2( c.use / cap ) : ( c.use > 0 ? 1e9 : 0.0 ) },
                               { "links", linkList } };
    };

    std::vector<int> listed;

    for( size_t k = 0; k < cuts.size(); ++k )
    {
        if( allChannels || cuts[k].use > 0 )
            listed.push_back( (int) k );
    }

    auto useOf = [&]( int k ) { return capacity( cuts[k] ) > 0 ? cuts[k].use / capacity( cuts[k] ) : ( cuts[k].use > 0 ? 1e9 : 0 ); };
    std::stable_sort( listed.begin(), listed.end(), [&]( int x, int y ) { return useOf( x ) > useOf( y ); } );

    nlohmann::json channels = nlohmann::json::array();
    nlohmann::json overflows = nlohmann::json::array();
    nlohmann::json show = nlohmann::json::array();

    for( int k : listed )
    {
        const bool over = cuts[k].use - capacity( cuts[k] ) > 1e-6;

        if( over )
        {
            nlohmann::json o = channelJson( k );
            o["over_mm"] = r2( cuts[k].use - capacity( cuts[k] ) );
            o["over_tracks"] = (int) std::ceil( ( cuts[k].use - capacity( cuts[k] ) ) / defaultPitch - 1e-9 );
            overflows.push_back( o );
        }

        if( (int) channels.size() < listLimit )
        {
            channels.push_back( channelJson( k ) );
            const CUT& c = cuts[k];
            const int  half = iu( 0.15 );
            const BOX2I area = c.vertical ? BOX2I( VECTOR2I( c.at - half, c.from ), VECTOR2I( 2 * half, c.to - c.from ) )
                                          : BOX2I( VECTOR2I( c.from, c.at - half ), VECTOR2I( c.to - c.from, 2 * half ) );
            const double u = useOf( k );
            show.push_back( { { "kind", "region" }, { "area", boxMm( area ) }, { "group", "floorplan" },
                              { "color", over ? "#E71D36" : u >= 0.7 ? "#FF9F1C" : "#3CC85A" }, { "fill_opacity", 0.6 },
                              { "width_px", 2 },
                              { "text", cutId( k ) + " " + std::to_string( int( std::round( u * 100 ) ) ) + "%" } } );
        }
    }

    nlohmann::json linkList = nlohmann::json::array();
    int            linksOver = 0, linksMissing = 0, linksThrough = 0, topologyMembers = 0;

    std::vector<int> byDemand( links.size() );

    for( size_t k = 0; k < links.size(); ++k )
        byDemand[k] = (int) k;

    std::stable_sort( byDemand.begin(), byDemand.end(), [&]( int x, int y ) { return links[x].demand > links[y].demand; } );

    // where a link crosses each portal of its path: the point, the portal's line and its stretch free
    // on both sides (portals in line with it between alike regions), the region the path enters: a
    // bus's corridor (pcb_route bus_paths)
    std::map<std::pair<bool, double>, std::vector<int>> portalsOnLine;

    for( size_t q = 0; q < portals.size(); ++q )
        portalsOnLine[{ portals[q].vertical, portals[q].at }].push_back( (int) q );

    auto freeStretch = [&]( int aPortal )
    {
        const PORTAL& p = portals[aPortal];
        double        lo = p.lo, hi = p.hi;
        const std::vector<int>& line = portalsOnLine[{ p.vertical, p.at }];

        for( bool grew = true; grew; )
        {
            grew = false;

            for( int q : line )
            {
                const PORTAL& o = portals[q];

                if( !alike( cells[regions[o.r0].cell], cells[regions[p.r0].cell] )
                    || !alike( cells[regions[o.r1].cell], cells[regions[p.r1].cell] ) )
                {
                    continue;
                }

                if( std::abs( o.hi - lo ) < 1e-6 && o.lo < lo )
                {
                    lo = o.lo;
                    grew = true;
                }
                else if( std::abs( o.lo - hi ) < 1e-6 && o.hi > hi )
                {
                    hi = o.hi;
                    grew = true;
                }
            }
        }

        return std::make_pair( lo, hi );
    };

    auto corridorJson = [&]( const LINK& aLink )
    {
        nlohmann::json list = nlohmann::json::array();

        for( const std::array<double, 4>& c : aLink.corridor )
        {
            const PORTAL& p = portals[(size_t) c[2]];
            const REGION& r = regions[(size_t) c[3]];
            const auto [lo, hi] = freeStretch( (int) c[2] );
            list.push_back( { { "at_mm", { r2( c[0] ), r2( c[1] ) } }, { "along", p.vertical ? "y" : "x" },
                              { "lo_mm", r2( lo ) }, { "hi_mm", r2( hi ) },
                              { "into_mm", { r2( toMm( xs[r.i0] ) ), r2( toMm( ys[r.j0] ) ), r2( toMm( xs[r.i1] ) ), r2( toMm( ys[r.j1] ) ) } } } );
        }

        return list;
    };

    for( int li : byDemand )
    {
        const LINK&    l = links[li];
        nlohmann::json path = nlohmann::json::array();
        bool           over = false;

        for( int c : l.cuts )
        {
            path.push_back( cutId( c ) );
            over |= cuts[c].use - capacity( cuts[c] ) > 1e-6;
        }

        nlohmann::json names = nlohmann::json::array();

        for( size_t k = 0; k < l.nets.size() && (int) k < listLimit; ++k )
            names.push_back( netName( l.nets[k] ) );

        nlohmann::json alone = nlohmann::json::array();

        for( int code : l.topologyOnly )
            alone.push_back( netName( code ) );

        topologyMembers += (int) l.topologyOnly.size();

        nlohmann::json through = nlohmann::json::array();

        for( int b : l.crosses )
            through.push_back( blocks[b].name );

        auto sideJson = [&]( int aSide ) { return aSide >= 0 ? nlohmann::json( SIDES[aSide] ) : nlohmann::json(); };

        nlohmann::json j = { { "a", blocks[l.a].name }, { "b", blocks[l.b].name }, { "bus", l.bus.empty() ? nlohmann::json() : nlohmann::json( l.bus ) },
                             { "a_side", sideJson( l.aExitSide ) }, { "b_side", sideJson( l.bExitSide ) },
                             { "sides_from", l.aPort >= 0 || l.bPort >= 0 ? ( l.sidesKept ? "pcb_layout_graph" : "path (no way by the graph's sides)" ) : "path" },
                             { "nets", l.nets.size() }, { "net_names", names }, { "demand_mm", r2( l.demand ) },
                             { "named_nets", l.bus.empty() ? 0 : l.nets.size() - l.topologyOnly.size() },
                             { "topology_only_nets", alone },
                             { "classes", l.classes }, { "found", l.found }, { "path", path },
                             { "corridor", corridorJson( l ) },
                             { "length_mm", r2( l.length ) }, { "through_blocks", through }, { "overflow", over } };
        linksOver += over;
        linksMissing += !l.found;
        linksThrough += !l.crosses.empty();
        linkList.push_back( j );
    }

    nlohmann::json portList = nlohmann::json::array();
    int            portsOver = 0;
    std::vector<std::pair<double, nlohmann::json>> portRows;

    for( const auto& [key, pt] : ports )
    {
        const auto [box, side] = key;
        int        length = 0, intoBoxes = 0;
        const std::vector<int> facing = portFree( box, side, length, intoBoxes );
        nlohmann::json         perLayer = nlohmann::json::object();
        long long              total = 0;
        int                    tracks = 0;

        for( int k = 0; k < nl; ++k )
        {
            perLayer[str( board->GetLayerName( routing[k] ) )] = r2( toMm( facing[k] ) );
            total += facing[k];
            tracks += facing[k] / std::max( 1, defaults.width + defaults.clearance );
        }

        const double   cap = toMm( double( total ) );
        const double   use = cap > 0 ? pt.demand / cap : 1e9;
        nlohmann::json names = nlohmann::json::array();

        for( int k : pt.links )
        {
            if( (int) names.size() < listLimit )
                names.push_back( linkName( links[k] ) );
        }

        nlohmann::json row = { { "block", blocks[boxes[box].block].name }, { "box_mm", boxMm( boxes[box].box ) },
                               { "side", SIDES[side] }, { "from", pt.given ? "pcb_layout_graph" : "path" },
                               { "length_mm", r2( toMm( length ) ) }, { "facing_boxes_mm", r2( toMm( intoBoxes ) ) },
                               { "free_mm", perLayer }, { "capacity_mm", r2( cap ) },
                               { "default_tracks", tracks }, { "demand_mm", r2( pt.demand ) }, { "demand_nets", pt.nets },
                               { "use", r2( use ) }, { "links", names } };

        if( pt.demand - cap > 1e-6 )
        {
            ++portsOver;
            row["over_tracks"] = (int) std::ceil( ( pt.demand - cap ) / defaultPitch - 1e-9 );
        }

        portRows.emplace_back( use, std::move( row ) );
    }

    std::stable_sort( portRows.begin(), portRows.end(), []( const auto& x, const auto& y ) { return x.first > y.first; } );

    for( auto& [use, row] : portRows )
    {
        if( (int) portList.size() < listLimit )
            portList.push_back( std::move( row ) );
    }

    nlohmann::json blockList = nlohmann::json::array();

    for( const FP_BLOCK& b : blocks )
    {
        nlohmann::json list = nlohmann::json::array();

        for( int k : b.boxes )
            list.push_back( boxMm( boxes[k].box ) );

        blockList.push_back( { { "name", b.name }, { "box_mm", boxMm( b.box ) }, { "boxes_mm", list }, { "parts", b.parts } } );
    }

    int       slivers = 0, sliversUsed = 0, zeroLength = 0;
    const int pitchIU = defaults.width + defaults.clearance;

    for( const CUT& c : cuts )
    {
        if( c.to - c.from < pitchIU )
        {
            ++slivers;
            sliversUsed += c.use > 0;
        }
    }

    for( const LINK& l : links )
        zeroLength += l.found && l.length <= 0;

    nlohmann::json keepoutList = nlohmann::json::array();

    for( const FP_KEEPOUT& k : keepouts )
    {
        nlohmann::json layers = nlohmann::json::array();

        for( int i = 0; i < nl; ++i )
        {
            if( k.layers & ( uint64_t( 1 ) << i ) )
                layers.push_back( str( board->GetLayerName( routing[i] ) ) );
        }

        keepoutList.push_back( { { "name", k.name }, { "box_mm", boxMm( k.box ) }, { "layers", layers }, { "source", k.source } } );
    }

    nlohmann::json classList = nlohmann::json::array();

    for( const std::string& name : usedClasses )
    {
        const FP_CLASS& c = classes[name];
        classList.push_back( { { "name", name }, { "track_mm", r3( toMm( c.width ) ) },
                               { "clearance_mm", r3( toMm( c.clearance ) ) },
                               { "pitch_mm", r3( toMm( c.width + c.clearance ) ) },
                               { "via_mm", r3( toMm( c.via ) ) } } );
    }

    size_t freeCells = 0;

    for( const CELL& c : cells )
        freeCells += c.kind == CELL_FREE;

    if( aArgs.value( "show", false ) )
        overlayShow( aCtx, show );

    const auto   now = clock::now();
    const double ms = std::chrono::duration<double, std::milli>( now - started ).count();
    const double floorplanMs = std::chrono::duration<double, std::milli>( now - analysed ).count();

    auto msBetween = []( clock::time_point aFrom, clock::time_point aTo )
    {
        return std::chrono::duration<double, std::milli>( aTo - aFrom ).count();
    };

    return KOPENAPI_RESULT::Ok(
            { { "layers", layerList },
              { "classes", classList },
              { "blocks", blockList },
              { "keepouts", keepoutList },
              { "grid", { { "columns", nx }, { "rows", ny }, { "cells", cells.size() }, { "free_cells", freeCells },
                          { "lines_before_snap", rawLines }, { "snap_mm", r3( toMm( snapTol ) ) },
                          { "regions", regions.size() }, { "portals", portals.size() } } },
              { "channel_count", cuts.size() },
              { "channels_used", std::count_if( cuts.begin(), cuts.end(), []( const CUT& c ) { return c.use > 0; } ) },
              { "channels", channels },
              { "overflows", overflows },
              { "links", linkList },
              { "ports", portList },
              { "summary", { { "links", links.size() },
                             { "overflowing_links", linksOver },
                             { "links_without_path", linksMissing },
                             { "links_through_blocks", linksThrough },
                             { "links_zero_length", zeroLength },
                             { "boxes", boxes.size() },
                             { "sliver_channels", slivers },
                             { "sliver_channels_used", sliversUsed },
                             { "overflows", overflows.size() },
                             { "ports_used", ports.size() },
                             { "topology_only_members", topologyMembers },
                             { "topology_only_split", splitTopology },
                             { "port_overflows", portsOver },
                             { "over_mm", r2( bestTotal ) } } },
              { "negotiation", { { "enabled", negotiate }, { "iterations", ran }, { "best_iteration", bestIteration },
                                 { "history", history } } },
              { "ms", r2( ms ) },
              { "floorplan_ms", r2( floorplanMs ) },
              { "timing_ms", { { "grid", r2( msBetween( analysed, gridDone ) ) },
                               { "regions", r2( msBetween( gridDone, regionsDone ) ) },
                               { "routing", r2( msBetween( regionsDone, routed ) ) },
                               { "answer", r2( msBetween( routed, now ) ) } } } } );
}


KOPENAPI_REGISTER( "pcb_floorplan",
                   "The third level of laying a board out (L3, floorplan and buses): the channels between the "
                   "blocks (each block's member courtyards merged into a few boxes, or its pcb_blocks bbox, as rigid "
                   "obstacles, with keep-outs that bar tracks and the board outside) and how much of them the links between blocks need. A channel is a cross-section of "
                   "the free space between two obstacles (between: their names, board edge included) along the "
                   "line of an obstacle's edge; per channel its free length per routing copper layer (inner layers "
                   "mostly under a rail pour are planes, not counted), the tracks of each net class that cross it "
                   "per layer, the vias that fit across it. Links: each pcb_layout_graph bus between two blocks, and "
                   "the single nets between two blocks, as one bundle with its width (track + clearance of each "
                   "net's class), taking one shortest path through the channels; overflows negotiated over the "
                   "channels (present overuse and history costs) and the best round kept. Answers channels by use, "
                   "overflows (demand over capacity, tracks over), links with their channel path, corridor (per portal crossed: the point, the "
                   "portal's axis, its stretch free on both sides and the region entered; pcb_route bus_paths floorplan routes a bus along it), length, blocks "
                   "they must cross (boxes overlapping or no way round) and the box sides they leave and enter by (a bus by "
                   "its end's exit side, pcb_layout_graph ends: an edge connector away from the board edge, else the pcb_fanout_ports side); ports: per box side used its facing "
                   "length (free or another box's, facing_boxes_mm) per layer as capacity against the bundles' width "
                   "through it. A bus link names its topology-only members (pcb_layout_graph net_source topology: in the bus "
                   "by their end parts only). show: channels in "
                   "the GUI coloured by use. "
                   "Read only",
                   R"json({"type":"object","properties":{
                        "by":{"type":"string","enum":["auto","sheet"],"default":"auto","description":"blocks as pcb_blocks: auto around ICs and connectors, sheet one per schematic sheet"},
                        "blocks":{"type":"array","items":{"type":"object","properties":{"name":{"type":"string"},"parts":{"type":"array","items":{"type":"string"}}}},"description":"these blocks instead [{name, parts: refs or globs}]"},
                        "max_part_pins":{"type":"integer","default":3},
                        "depth":{"type":"integer","default":2,"minimum":1,"maximum":6},
                        "max_fanout":{"type":"integer","default":30},
                        "min_bus":{"type":"integer","default":3,"minimum":2,"description":"lines a bundle needs to be a bus (pcb_net_roles min_bundle)"},
                        "layers":{"type":"array","items":{"type":"string"},"description":"copper layers to route on; default every signal layer (inner rail planes left out)"},
                        "keepouts":{"type":"array","items":{"type":"object","properties":{
                            "name":{"type":"string"},
                            "rect":{"type":"array","items":{"type":"number"},"description":"[x0, y0, x1, y1] mm"},
                            "layers":{"type":"array","items":{"type":"string"},"description":"default every routing layer"}}},"description":"more areas that bar tracks, besides the board's rule areas"},
                        "edge_clearance_mm":{"type":"number","description":"default: the board's copper-to-edge clearance"},
                        "obstacles":{"type":"string","enum":["parts","bbox"],"default":"parts","description":"parts: each block's member courtyards, merged into boxes while their union stays full; bbox: one box per block (pcb_blocks bbox)"},
                        "merge_gap_mm":{"type":"number","description":"courtyards closer than this may merge (default one Default track pitch)"},
                        "merge_fill":{"type":"number","default":0.7,"minimum":0,"maximum":1,"description":"two boxes merge when their areas fill this much of their union"},
                        "max_boxes":{"type":"integer","default":8,"minimum":1,"maximum":64,"description":"boxes per block at most"},
                        "snap_mm":{"type":"number","minimum":0,"description":"obstacle edges closer than this share one grid line (default one Default track pitch; 0: exact). Gaps narrower close, obstacles thinner keep both edges"},
                        "negotiate":{"type":"boolean","default":true,"description":"false: every bundle on its shortest path, one round"},
                        "split_topology_only":{"type":"boolean","default":false,"description":"bus nets in it by their end parts only (pcb_layout_graph net_source topology) go with the single nets between the blocks"},
                        "port_sides":{"type":"boolean","default":true,"description":"a bus leaves and enters its end blocks by its ends' exit sides (pcb_layout_graph ends); false: by any side"},
                        "reroute":{"type":"string","enum":["overflowed","all"],"default":"overflowed","description":"after the first round reroute only the bundles on an overflowing channel, or all"},
                        "iterations":{"type":"integer","default":8,"minimum":1,"maximum":64,"description":"negotiation rounds at most"},
                        "overflow_cost_mm":{"type":"number","default":5,"description":"detour worth one track over a channel's capacity, first round (doubles each round)"},
                        "through_block_factor":{"type":"number","default":10,"minimum":1,"description":"cost of a path through another block's box per mm, against free space"},
                        "all_channels":{"type":"boolean","default":false,"description":"list every channel, not only the used ones"},
                        "list_limit":{"type":"integer","default":40,"description":"channels listed, net names and links per entry"},"full":{"type":"boolean","default":false,"description":"everything: every item (no page limit) and every entry of every list in the answer (no list_limit); large on a big design"},
                        "show":{"type":"boolean","default":false,"description":"GUI: overlay group 'floorplan'"}}})json"_json,
                   false, h_pcb_floorplan, 120 );
