/// @file methods_pcb_floorplan.cpp
/// @brief kicadopenapi floorplan analysis (L3): the channels between the blocks of the board, their
/// capacity per copper layer and the demand of the buses and links between blocks on them.  Read only.
///
/// Blocks (their members' courtyards merged into a few boxes), all-layer keep-outs and the board outside
/// are obstacles.  Their box edges cut the board into a non-uniform grid of cells; a channel is a maximal
/// free run of cell boundaries along one grid line (a cross-section of the free space between two
/// obstacles).  Every bundle (the nets of one bus between two blocks, or the single nets between them)
/// takes one cell path from its pads' boxes in one block to those in the other; each channel it crosses
/// carries its width (track + clearance of each net's class).  Overflows are negotiated over channels
/// (PathFinder costs: present overuse and history).
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
    std::vector<int> cuts;
    std::vector<int> cells;
    std::set<int>    crosses;
    double           length = 0;
    bool             found = false;
};


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
    const int listLimit = std::max( 1, aArgs.value( "list_limit", 40 ) );
    const bool allChannels = aArgs.value( "all_channels", false );

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

    for( const nlohmann::json& bus : graphResult.body.value( "buses", nlohmann::json::array() ) )
    {
        const std::string name = bus.value( "name", std::string() );

        for( const nlohmann::json& n : bus.value( "nets", nlohmann::json::array() ) )
        {
            if( NETINFO_ITEM* net = n.is_string() ? KopenapiFindNet( board, n.get<std::string>() ) : nullptr )
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

    addLines( ub );

    for( const FP_BOX& b : boxes )
        addLines( b.box );

    for( const FP_KEEPOUT& k : keepouts )
        addLines( k.box );

    for( std::vector<int>* v : { &xs, &ys } )
    {
        std::sort( v->begin(), v->end() );
        v->erase( std::unique( v->begin(), v->end() ), v->end() );
    }

    const int nx = (int) xs.size() - 1, ny = (int) ys.size() - 1;

    if( nx < 1 || ny < 1 )
        return KOPENAPI_RESULT::Error( 409, "the board outline has no area" );

    std::vector<CELL> cells( size_t( nx ) * ny );
    auto at = [&]( int i, int j ) -> CELL& { return cells[size_t( j ) * nx + i]; };

    auto span = []( const std::vector<int>& aLines, int aLow, int aHigh )
    {
        const int last = (int) aLines.size() - 1;
        const int i0 = int( std::lower_bound( aLines.begin(), aLines.end(), aLow ) - aLines.begin() );
        const int i1 = int( std::lower_bound( aLines.begin(), aLines.end(), aHigh ) - aLines.begin() );
        return std::make_pair( std::min( i0, last ), std::min( i1, last ) );   // cells i0 .. i1 - 1
    };

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
        const auto [i0, i1] = span( xs, keepouts[k].box.GetLeft(), keepouts[k].box.GetRight() );
        const auto [j0, j1] = span( ys, keepouts[k].box.GetTop(), keepouts[k].box.GetBottom() );

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

    for( size_t k = 0; k < boxes.size(); ++k )
    {
        const auto [i0, i1] = span( xs, boxes[k].box.GetLeft(), boxes[k].box.GetRight() );
        const auto [j0, j1] = span( ys, boxes[k].box.GetTop(), boxes[k].box.GetBottom() );
        const int  b = boxes[k].block;

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

    // one bundle's cell path: A* from its first block's cells to the other's
    const size_t        n = cells.size();
    std::vector<double> cost( n ), cx( nx ), cy( ny );
    std::vector<int>    prev( n );
    std::vector<unsigned> seenAt( n, 0 ), doneAt( n, 0 ), targetAt( n, 0 );
    unsigned              epoch = 0;

    for( int i = 0; i < nx; ++i )
        cx[i] = toMm( ( double( xs[i] ) + xs[i + 1] ) / 2 );

    for( int j = 0; j < ny; ++j )
        cy[j] = toMm( ( double( ys[j] ) + ys[j + 1] ) / 2 );

    auto has = []( const CELL& aCell, int aBlock )
    {
        return std::find( aCell.blocks.begin(), aCell.blocks.end(), aBlock ) != aCell.blocks.end();
    };

    auto routeLink = [&]( LINK& aLink, double aPresent )
    {
        const std::set<int> fromBoxes = aLink.aBoxes.empty() ? std::set<int>( blocks[aLink.a].boxes.begin(), blocks[aLink.a].boxes.end() )
                                                             : aLink.aBoxes;
        const std::set<int> toBoxes = aLink.bBoxes.empty() ? std::set<int>( blocks[aLink.b].boxes.begin(), blocks[aLink.b].boxes.end() )
                                                           : aLink.bBoxes;
        BOX2I target = boxes[*toBoxes.begin()].box;

        for( int k : toBoxes )
            target.Merge( boxes[k].box );

        const double tx0 = toMm( target.GetLeft() ), tx1 = toMm( target.GetRight() );
        const double ty0 = toMm( target.GetTop() ), ty1 = toMm( target.GetBottom() );

        auto h = [&]( int i, int j )
        {
            const double dx = cx[i] < tx0 ? tx0 - cx[i] : cx[i] > tx1 ? cx[i] - tx1 : 0;
            const double dy = cy[j] < ty0 ? ty0 - cy[j] : cy[j] > ty1 ? cy[j] - ty1 : 0;
            return dx + dy;
        };

        ++epoch;

        for( int k : toBoxes )
        {
            for( int c : boxCells[k] )
                targetAt[c] = epoch;
        }

        using ENTRY = std::pair<double, int>;
        std::priority_queue<ENTRY, std::vector<ENTRY>, std::greater<ENTRY>> open;

        for( int k : fromBoxes )
        {
            for( int c : boxCells[k] )
            {
                cost[c] = 0;
                prev[c] = -1;
                seenAt[c] = epoch;
                open.emplace( h( c % nx, c / nx ), c );
            }
        }

        auto cutCost = [&]( int aCut )
        {
            if( aCut < 0 )
                return 0.0;

            const CUT&   cut = cuts[aCut];
            const double over = std::max( 0.0, cut.use + aLink.demand - capacity( cut ) ) / defaultPitch;
            return ( aPresent * over + cut.hist ) * overflowCost;
        };

        int reached = -1;

        while( !open.empty() )
        {
            const int c = open.top().second;
            open.pop();

            if( doneAt[c] == epoch )
                continue;

            doneAt[c] = epoch;
            const int i = c % nx, j = c / nx;

            if( targetAt[c] == epoch )
            {
                reached = c;
                break;
            }

            const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };

            for( int k = 0; k < 4; ++k )
            {
                const int ni = i + di[k], nj = j + dj[k];

                if( ni < 0 || nj < 0 || ni >= nx || nj >= ny )
                    continue;

                const int   d = nj * nx + ni;
                const CELL& cell = cells[d];
                double      factor = 1;

                if( cell.kind == CELL_WALL || cell.kind == CELL_OUT )
                    continue;

                if( cell.kind == CELL_BLOCK && !has( cell, aLink.a ) && !has( cell, aLink.b ) )
                {
                    if( !cell.inside )
                        continue;

                    factor = foreignPenalty;
                }

                int edge = -1;

                if( k == 0 )
                    edge = vEdge[size_t( j ) * ( nx - 1 ) + i];
                else if( k == 1 )
                    edge = vEdge[size_t( j ) * ( nx - 1 ) + ni];
                else if( k == 2 )
                    edge = hEdge[size_t( j ) * nx + i];
                else
                    edge = hEdge[size_t( nj ) * nx + i];

                const double step = ( std::abs( cx[ni] - cx[i] ) + std::abs( cy[nj] - cy[j] ) ) * factor + cutCost( edge );
                const double next = cost[c] + step;

                if( seenAt[d] != epoch || next < cost[d] )
                {
                    seenAt[d] = epoch;
                    cost[d] = next;
                    prev[d] = c;
                    open.emplace( next + h( ni, nj ), d );
                }
            }
        }

        aLink.cuts.clear();
        aLink.cells.clear();
        aLink.crosses.clear();
        aLink.length = 0;
        aLink.found = reached >= 0;

        if( !aLink.found )
            return;

        for( int c = reached; c >= 0; c = prev[c] )
            aLink.cells.push_back( c );

        std::reverse( aLink.cells.begin(), aLink.cells.end() );

        for( size_t k = 0; k < aLink.cells.size(); ++k )
        {
            const int   c = aLink.cells[k];
            const CELL& cell = cells[c];

            if( cell.kind == CELL_BLOCK && !has( cell, aLink.a ) && !has( cell, aLink.b ) )
                aLink.crosses.insert( cell.blocks[0] );

            if( k == 0 )
                continue;

            const int p = aLink.cells[k - 1];
            const int pi = p % nx, pj = p / nx, ci = c % nx, cj = c / nx;
            aLink.length += std::abs( cx[ci] - cx[pi] ) + std::abs( cy[cj] - cy[pj] );
            int edge = -1;

            if( cj == pj )
                edge = vEdge[size_t( cj ) * ( nx - 1 ) + std::min( ci, pi )];
            else
                edge = hEdge[size_t( std::min( cj, pj ) ) * nx + ci];

            if( edge >= 0 )
                aLink.cuts.push_back( edge );
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

        for( int k : order )
        {
            apply( links[k], -1 );
            routeLink( links[k], present );
            apply( links[k], +1 );
        }

        ++ran;
        double    total = 0;
        const int count = overflowOf( total );
        history.push_back( { { "iteration", it + 1 }, { "overflows", count }, { "over_mm", r2( total ) } } );

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
                c.hist += over / defaultPitch;
        }
    }

    links = best;

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
    int            linksOver = 0, linksMissing = 0, linksThrough = 0;

    std::vector<int> byDemand( links.size() );

    for( size_t k = 0; k < links.size(); ++k )
        byDemand[k] = (int) k;

    std::stable_sort( byDemand.begin(), byDemand.end(), [&]( int x, int y ) { return links[x].demand > links[y].demand; } );

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

        nlohmann::json through = nlohmann::json::array();

        for( int b : l.crosses )
            through.push_back( blocks[b].name );

        nlohmann::json j = { { "a", blocks[l.a].name }, { "b", blocks[l.b].name }, { "bus", l.bus.empty() ? nlohmann::json() : nlohmann::json( l.bus ) },
                             { "nets", l.nets.size() }, { "net_names", names }, { "demand_mm", r2( l.demand ) },
                             { "classes", l.classes }, { "found", l.found }, { "path", path },
                             { "length_mm", r2( l.length ) }, { "through_blocks", through }, { "overflow", over } };
        linksOver += over;
        linksMissing += !l.found;
        linksThrough += !l.crosses.empty();
        linkList.push_back( j );
    }

    nlohmann::json blockList = nlohmann::json::array();

    for( const FP_BLOCK& b : blocks )
    {
        nlohmann::json list = nlohmann::json::array();

        for( int k : b.boxes )
            list.push_back( boxMm( boxes[k].box ) );

        blockList.push_back( { { "name", b.name }, { "box_mm", boxMm( b.box ) }, { "boxes_mm", list } } );
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
        zeroLength += l.found && l.cells.size() <= 1;

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

    return KOPENAPI_RESULT::Ok(
            { { "layers", layerList },
              { "classes", classList },
              { "blocks", blockList },
              { "keepouts", keepoutList },
              { "grid", { { "columns", nx }, { "rows", ny }, { "cells", cells.size() }, { "free_cells", freeCells } } },
              { "channel_count", cuts.size() },
              { "channels_used", std::count_if( cuts.begin(), cuts.end(), []( const CUT& c ) { return c.use > 0; } ) },
              { "channels", channels },
              { "overflows", overflows },
              { "links", linkList },
              { "summary", { { "links", links.size() },
                             { "overflowing_links", linksOver },
                             { "links_without_path", linksMissing },
                             { "links_through_blocks", linksThrough },
                             { "links_zero_length", zeroLength },
                             { "boxes", boxes.size() },
                             { "sliver_channels", slivers },
                             { "sliver_channels_used", sliversUsed },
                             { "overflows", overflows.size() },
                             { "over_mm", r2( bestTotal ) } } },
              { "negotiation", { { "enabled", negotiate }, { "iterations", ran }, { "best_iteration", bestIteration },
                                 { "history", history } } },
              { "ms", r2( ms ) },
              { "floorplan_ms", r2( floorplanMs ) } } );
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
                   "overflows (demand over capacity, tracks over), links with their channel path, length, blocks "
                   "they must cross (boxes overlapping or no way round). show: channels in the GUI coloured by use. "
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
                        "negotiate":{"type":"boolean","default":true,"description":"false: every bundle on its shortest path, one round"},
                        "iterations":{"type":"integer","default":8,"minimum":1,"maximum":64,"description":"negotiation rounds at most"},
                        "overflow_cost_mm":{"type":"number","default":5,"description":"detour worth one track over a channel's capacity, first round (doubles each round)"},
                        "through_block_factor":{"type":"number","default":10,"minimum":1,"description":"cost of a path through another block's box per mm, against free space"},
                        "all_channels":{"type":"boolean","default":false,"description":"list every channel, not only the used ones"},
                        "list_limit":{"type":"integer","default":40,"description":"channels listed, net names and links per entry"},
                        "show":{"type":"boolean","default":false,"description":"GUI: overlay group 'floorplan'"}}})json"_json,
                   false, h_pcb_floorplan, 120 );
