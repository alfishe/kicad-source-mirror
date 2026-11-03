/// @file methods_pcb_space.cpp
/// @brief kicadopenapi board space analysis: pcb_blocks (functional blocks of the placed board),
/// pcb_space (occupied / free area, largest free rectangles, trying another outline).
///
/// Both read only.  Blocks come from the board alone: anchors (ICs, connectors), small parts reached
/// from them through signal nets, rail capacitors given to the nearest anchor pad on their supply.
/// Areas are unions of courtyards (no double counting); free rectangles are maximal rectangles on a
/// grid over the usable outline.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_segment.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <netclass.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <string_utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <set>


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


double mm( double aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 100.0 ) / 100.0;
}


double mm2( double aIU2 )
{
    return std::round( aIU2 / ( pcbIUScale.IU_PER_MM * pcbIUScale.IU_PER_MM ) * 10.0 ) / 10.0;
}


nlohmann::json boxJson( const BOX2I& aBox )
{
    return { mm( aBox.GetLeft() ), mm( aBox.GetTop() ), mm( aBox.GetRight() ), mm( aBox.GetBottom() ) };
}


SHAPE_POLY_SET rectPoly( const BOX2I& aBox )
{
    SHAPE_POLY_SET p;
    p.NewOutline();
    p.Append( aBox.GetLeft(), aBox.GetTop() );
    p.Append( aBox.GetRight(), aBox.GetTop() );
    p.Append( aBox.GetRight(), aBox.GetBottom() );
    p.Append( aBox.GetLeft(), aBox.GetBottom() );
    return p;
}


/// @brief [x0, y0, x1, y1] mm or [[x, y], ...] mm into a polygon; nullopt when malformed
std::optional<SHAPE_POLY_SET> polyArg( const nlohmann::json& aRect, const nlohmann::json& aPoints )
{
    if( !aRect.is_null() )
    {
        if( !aRect.is_array() || aRect.size() != 4
            || !std::all_of( aRect.begin(), aRect.end(), []( const nlohmann::json& v ) { return v.is_number(); } ) )
        {
            return std::nullopt;
        }

        const double x0 = std::min( aRect[0].get<double>(), aRect[2].get<double>() );
        const double x1 = std::max( aRect[0].get<double>(), aRect[2].get<double>() );
        const double y0 = std::min( aRect[1].get<double>(), aRect[3].get<double>() );
        const double y1 = std::max( aRect[1].get<double>(), aRect[3].get<double>() );

        if( x1 - x0 <= 0 || y1 - y0 <= 0 )
            return std::nullopt;

        return rectPoly( BOX2I( VECTOR2I( iu( x0 ), iu( y0 ) ), VECTOR2I( iu( x1 - x0 ), iu( y1 - y0 ) ) ) );
    }

    if( !aPoints.is_array() || aPoints.size() < 3 )
        return std::nullopt;

    SHAPE_POLY_SET p;
    p.NewOutline();

    for( const nlohmann::json& pt : aPoints )
    {
        if( !pt.is_array() || pt.size() != 2 || !pt[0].is_number() || !pt[1].is_number() )
            return std::nullopt;

        p.Append( iu( pt[0].get<double>() ), iu( pt[1].get<double>() ) );
    }

    return p;
}


SHAPE_POLY_SET courtyardOf( FOOTPRINT* aFp )
{
    const SHAPE_POLY_SET& cy = aFp->GetCourtyard( aFp->IsFlipped() ? B_CrtYd : F_CrtYd );

    if( cy.OutlineCount() > 0 )
        return cy;

    return rectPoly( aFp->GetBoundingBox( false ) );
}


struct PART
{
    FOOTPRINT*       fp = nullptr;
    std::string      ref;
    std::string      sheet;
    std::vector<int> nets;        ///< distinct net codes of its pads (> 0)
    int              pads = 0;    ///< pads with a number (copper connections)
    bool             connector = false;
    bool             mechanical = false;
    bool             back = false;
    SHAPE_POLY_SET   courtyard;
    VECTOR2I         centre;
};


struct BLOCK
{
    std::string                        name;
    std::string                        kind;     ///< ic, connector, sheet, given
    int                                anchor = -1;
    std::vector<int>                   members;  ///< part indexes, anchor first
    std::map<int, std::string>         by;       ///< member -> signal / rail / loose / given
};


/// @brief The board's parts, rails and functional blocks: everything pcb_blocks answers and pcb_space
/// groups its findings by
struct BOARD_BLOCKS
{
    std::vector<PART>               parts;
    std::map<const FOOTPRINT*, int> index;
    std::set<int>                   rails, grounds;
    std::vector<BLOCK>              blocks;
    std::vector<int>                partBlock;   ///< part -> block, -1 none
    std::vector<int>                mechanical;
    std::vector<int>                unassigned;
};


std::string sheetOf( FOOTPRINT* aFp )
{
    std::string file = str( aFp->GetSheetfile() );
    std::string name = str( aFp->GetSheetname() );

    if( !name.empty() )
        return name;

    return file.empty() ? "/" : file;
}


/// @brief Rails (ground / power nets) as pcb_net_roles decides them, else from the names
void findRails( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, std::set<int>& aRails, std::set<int>& aGrounds )
{
    std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" );

    if( roles )
    {
        KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

        if( r.status == 200 )
        {
            for( const nlohmann::json& n : r.body.value( "nets", nlohmann::json::array() ) )
            {
                const std::string role = n.value( "role", std::string() );

                if( role != "ground" && role != "power" && role != "power_hc" )
                    continue;

                if( NETINFO_ITEM* net = KopenapiFindNet( aBoard, n.value( "net", std::string() ) ) )
                {
                    aRails.insert( net->GetNetCode() );

                    if( role == "ground" )
                        aGrounds.insert( net->GetNetCode() );
                }
            }

            return;
        }
    }

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
    {
        if( net->GetNetCode() <= 0 )
            continue;

        const std::string role = KopenapiNetRoleFromName( str( UnescapeString( net->GetNetname() ) ) ).value( "role", "" );

        if( role == "ground" || role == "power" )
        {
            aRails.insert( net->GetNetCode() );

            if( role == "ground" )
                aGrounds.insert( net->GetNetCode() );
        }
    }
}


struct GROUP_OPTIONS
{
    int  maxPartPins = 3;
    int  depth = 2;
    int  maxFanout = 30;
    bool bySheet = false;
};


double dist( const VECTOR2I& a, const VECTOR2I& b )
{
    return ( a - b ).EuclideanNorm();
}


/// @brief Parts and blocks of the board.  aGiven: [{name, parts}] evaluated instead of the automatic
/// split; aError set when it names unknown parts.
BOARD_BLOCKS buildBlocks( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, const GROUP_OPTIONS& aOpt, const nlohmann::json& aGiven,
                          std::string& aError )
{
    BOARD_BLOCKS b;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        PART p;
        p.fp = fp;
        p.ref = str( fp->GetReference() );
        p.sheet = sheetOf( fp );
        p.back = fp->IsFlipped();
        p.courtyard = courtyardOf( fp );
        p.centre = p.courtyard.BBox().Centre();

        std::set<int> nets;

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNumber().IsEmpty() )
                continue;

            p.pads++;

            if( pad->GetNetCode() > 0 )
                nets.insert( pad->GetNetCode() );
        }

        p.nets.assign( nets.begin(), nets.end() );

        const std::string lib = str( fp->GetFPID().GetLibItemName() ) + " " + str( fp->GetFPID().GetLibNickname() );
        const std::string prefix = KopenapiRefPrefix( p.ref );
        p.connector = KopenapiIsConnectorRef( p.ref ) || lib.find( "Connector" ) != std::string::npos;
        p.mechanical = nets.empty() || prefix == "H" || prefix == "MH" || prefix == "FID"
                       || lib.find( "MountingHole" ) != std::string::npos || lib.find( "Fiducial" ) != std::string::npos;

        if( p.mechanical )
            p.connector = false;

        b.index[fp] = (int) b.parts.size();
        b.parts.push_back( std::move( p ) );
    }

    b.partBlock.assign( b.parts.size(), -1 );
    findRails( aCtx, aBoard, b.rails, b.grounds );

    std::map<int, std::vector<int>> netParts;
    std::map<int, int>              netPads;

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        for( int n : b.parts[i].nets )
            netParts[n].push_back( (int) i );

        for( PAD* pad : b.parts[i].fp->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        if( b.parts[i].mechanical )
            b.mechanical.push_back( (int) i );
    }

    auto assign = [&]( int aBlock, int aPart, const std::string& aBy )
    {
        b.partBlock[aPart] = aBlock;
        b.blocks[aBlock].members.push_back( aPart );
        b.blocks[aBlock].by[aPart] = aBy;
    };

    if( aGiven.is_array() && !aGiven.empty() )
    {
        for( const nlohmann::json& g : aGiven )
        {
            BLOCK block;
            block.name = g.value( "name", std::string() );
            block.kind = "given";
            b.blocks.push_back( block );
            const int id = (int) b.blocks.size() - 1;

            if( b.blocks[id].name.empty() )
                b.blocks[id].name = "block " + std::to_string( id + 1 );

            for( const nlohmann::json& r : g.value( "parts", nlohmann::json::array() ) )
            {
                const std::string pattern = r.is_string() ? r.get<std::string>() : std::string();
                bool              hit = false;

                for( size_t i = 0; i < b.parts.size(); ++i )
                {
                    if( b.partBlock[i] < 0 && !pattern.empty() && KopenapiGlob( pattern, b.parts[i].ref ) )
                    {
                        assign( id, (int) i, "given" );
                        hit = true;
                    }
                }

                if( !hit )
                {
                    aError = "blocks: no (unassigned) part matches " + r.dump() + " in " + b.blocks[id].name;
                    return b;
                }
            }

            // the anchor: the member with the most pads
            int best = -1;

            for( int m : b.blocks[id].members )
            {
                if( best < 0 || b.parts[m].pads > b.parts[best].pads )
                    best = m;
            }

            b.blocks[id].anchor = best;
        }

        for( size_t i = 0; i < b.parts.size(); ++i )
        {
            if( b.partBlock[i] < 0 && !b.parts[i].mechanical )
                b.unassigned.push_back( (int) i );
        }

        return b;
    }

    // anchors: bigger parts and connectors
    std::vector<int> anchors;

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        const PART& p = b.parts[i];

        if( !p.mechanical && ( p.connector || p.pads > aOpt.maxPartPins ) )
            anchors.push_back( (int) i );
    }

    std::sort( anchors.begin(), anchors.end(),
               [&]( int x, int y ) { return KopenapiNaturalLess( b.parts[x].ref, b.parts[y].ref ); } );

    for( int a : anchors )
    {
        BLOCK block;
        const PART& p = b.parts[a];
        block.name = p.ref + " " + str( p.fp->GetValue() );
        block.kind = p.connector ? "connector" : "ic";
        block.anchor = a;
        b.blocks.push_back( block );
        assign( (int) b.blocks.size() - 1, a, "anchor" );
    }

    auto isSmall = [&]( int i )
    {
        const PART& p = b.parts[i];
        return !p.mechanical && !p.connector && p.pads <= aOpt.maxPartPins;
    };

    auto signalNet = [&]( int n )
    {
        return n > 0 && !b.rails.count( n ) && ( aOpt.maxFanout <= 0 || netPads[n] <= aOpt.maxFanout );
    };

    // a supply of one anchor and a few small parts (a relay's coil net, switched by its driver
    // transistor): the anchor reaches those parts through it
    auto localRail = [&]( int n )
    {
        if( n <= 0 || !b.rails.count( n ) || b.grounds.count( n ) )
            return false;

        int anchorsOn = 0;
        int smallOn = 0;

        for( int o : netParts[n] )
            ( isSmall( o ) ? smallOn : anchorsOn ) += b.parts[o].mechanical ? 0 : 1;

        return anchorsOn == 1 && smallOn <= 3;
    };

    // signal members: breadth-first from each anchor through small parts
    struct REACH
    {
        int block = -1;
        int steps = 1 << 20;
        bool sameSheet = false;
        double distance = 1e18;
    };

    std::vector<REACH> reach( b.parts.size() );

    for( size_t bi = 0; bi < b.blocks.size(); ++bi )
    {
        const int                a = b.blocks[bi].anchor;
        std::map<int, int>       level = { { a, 0 } };
        std::deque<int>          queue = { a };

        while( !queue.empty() )
        {
            const int cur = queue.front();
            queue.pop_front();

            if( level[cur] >= aOpt.depth )
                continue;

            for( int n : b.parts[cur].nets )
            {
                if( !signalNet( n ) && !( cur == a && localRail( n ) ) )
                    continue;

                for( int o : netParts[n] )
                {
                    if( level.count( o ) || !isSmall( o ) )
                        continue;

                    level[o] = level[cur] + 1;
                    queue.push_back( o );

                    REACH cand{ (int) bi, level[o], b.parts[o].sheet == b.parts[a].sheet,
                                dist( b.parts[o].centre, b.parts[a].centre ) };
                    REACH& r = reach[o];
                    const bool better = r.block < 0 || cand.steps < r.steps
                                        || ( cand.steps == r.steps && cand.sameSheet && !r.sameSheet )
                                        || ( cand.steps == r.steps && cand.sameSheet == r.sameSheet && cand.distance < r.distance );

                    if( better )
                        r = cand;
                }
            }
        }
    }

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        if( reach[i].block >= 0 && b.partBlock[i] < 0 )
            assign( reach[i].block, (int) i, "signal" );
    }

    // rail-only parts (decoupling, bulk): the anchor with the nearest pad on their supply
    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        if( b.partBlock[i] >= 0 || !isSmall( (int) i ) )
            continue;

        const PART& p = b.parts[i];

        if( p.nets.empty() || !std::all_of( p.nets.begin(), p.nets.end(), [&]( int n ) { return b.rails.count( n ) > 0; } ) )
            continue;

        int supply = -1;

        for( int n : p.nets )
        {
            if( !b.grounds.count( n ) && ( supply < 0 || netPads[n] < netPads[supply] ) )
                supply = n;
        }

        if( supply < 0 )
            continue;

        int    best = -1;
        bool   bestSheet = false;
        double bestDist = 1e18;

        for( size_t bi = 0; bi < b.blocks.size(); ++bi )
        {
            const PART& a = b.parts[b.blocks[bi].anchor];

            for( PAD* pad : a.fp->Pads() )
            {
                if( pad->GetNetCode() != supply )
                    continue;

                const bool   sheet = a.sheet == p.sheet;
                const double d = dist( pad->GetPosition(), p.centre );

                if( best < 0 || ( sheet && !bestSheet ) || ( sheet == bestSheet && d < bestDist ) )
                {
                    best = (int) bi;
                    bestSheet = sheet;
                    bestDist = d;
                }
            }
        }

        if( best >= 0 )
            assign( best, (int) i, "rail" );
    }

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        if( b.partBlock[i] < 0 && !b.parts[i].mechanical )
            b.unassigned.push_back( (int) i );
    }

    if( aOpt.bySheet )
    {
        std::map<std::string, int> sheetBlock;
        std::vector<BLOCK>         sheets;
        std::vector<int>           partSheet( b.parts.size(), -1 );

        for( size_t i = 0; i < b.parts.size(); ++i )
        {
            if( b.parts[i].mechanical )
                continue;

            auto [it, fresh] = sheetBlock.try_emplace( b.parts[i].sheet, (int) sheets.size() );

            if( fresh )
            {
                BLOCK s;
                s.name = b.parts[i].sheet;
                s.kind = "sheet";
                sheets.push_back( s );
            }

            BLOCK& s = sheets[it->second];
            s.members.push_back( (int) i );
            s.by[(int) i] = b.partBlock[i] >= 0 ? b.blocks[b.partBlock[i]].by[(int) i] : "loose";
            partSheet[i] = it->second;

            if( s.anchor < 0 || b.parts[i].pads > b.parts[s.anchor].pads )
                s.anchor = (int) i;
        }

        b.blocks = std::move( sheets );
        b.partBlock = std::move( partSheet );
        b.unassigned.clear();
    }

    return b;
}


/// @brief Union of the members' courtyards on one side
SHAPE_POLY_SET unionOf( const BOARD_BLOCKS& aB, const std::vector<int>& aParts, bool aBack )
{
    SHAPE_POLY_SET u;

    for( int i : aParts )
    {
        if( aB.parts[i].back == aBack )
            u.Append( aB.parts[i].courtyard );
    }

    u.Simplify();   // one union of all of them
    return u;
}


const std::vector<std::string> PALETTE = { "#FF9F1C", "#2EC4B6", "#E71D36", "#9B5DE5", "#00BBF9", "#F15BB5",
                                           "#80ED99", "#FEE440", "#4361EE", "#F77F00", "#06D6A0", "#EF476F" };


void overlay( KOPENAPI_CONTEXT& aCtx, const std::string& aGroup, const nlohmann::json& aAdd )
{
    if( aCtx.headless )
        return;

    std::optional<KOPENAPI_METHOD> m = KOPENAPI_REGISTRY::Get().Find( "pcb_overlay" );

    if( !m )
        return;

    m->handler( aCtx, { { "clear", aGroup } } );

    if( !aAdd.empty() )
        m->handler( aCtx, { { "add", aAdd } } );
}


std::optional<std::string> groupOptions( const nlohmann::json& aArgs, GROUP_OPTIONS& aOpt )
{
    const std::string by = aArgs.value( "by", std::string( "auto" ) );

    if( by != "auto" && by != "sheet" )
        return "by: auto or sheet";

    aOpt.bySheet = by == "sheet";
    aOpt.maxPartPins = std::clamp( aArgs.value( "max_part_pins", 3 ), 1, 64 );
    aOpt.depth = std::clamp( aArgs.value( "depth", 2 ), 1, 6 );
    aOpt.maxFanout = aArgs.value( "max_fanout", 30 );
    return std::nullopt;
}


// ---- grid of free cells: scanline fill and maximal rectangles

struct FREE_GRID
{
    int               nx = 0, ny = 0;
    int               step = 0;
    VECTOR2I          origin;
    std::vector<char> cell;   ///< 1 = free

    char& at( int x, int y ) { return cell[size_t( y ) * nx + x]; }
};


/// @brief x of the crossings of a horizontal line with every contour (outlines and holes) of aPoly
std::vector<int> crossings( const SHAPE_POLY_SET& aPoly, int aY )
{
    std::vector<int> xs;

    auto chain = [&]( const SHAPE_LINE_CHAIN& c )
    {
        const int n = c.PointCount();

        for( int i = 0; i < n; ++i )
        {
            const VECTOR2I& p = c.CPoint( i );
            const VECTOR2I& q = c.CPoint( ( i + 1 ) % n );

            if( ( p.y <= aY && q.y > aY ) || ( q.y <= aY && p.y > aY ) )
                xs.push_back( int( p.x + double( aY - p.y ) * ( q.x - p.x ) / double( q.y - p.y ) ) );
        }
    };

    for( int o = 0; o < aPoly.OutlineCount(); ++o )
    {
        chain( aPoly.COutline( o ) );

        for( int h = 0; h < aPoly.HoleCount( o ); ++h )
            chain( aPoly.CHole( o, h ) );
    }

    std::sort( xs.begin(), xs.end() );
    return xs;
}


/// @brief Cells fully inside aInside (sampled at the cell's top, middle and bottom) and clear of aBlocked
FREE_GRID rasterise( const SHAPE_POLY_SET& aInside, const SHAPE_POLY_SET& aBlocked, int aStep )
{
    FREE_GRID         g;
    const BOX2I  box = aInside.BBox();
    g.step = aStep;
    g.origin = box.GetOrigin();
    g.nx = std::max( 1, int( box.GetWidth() / aStep ) );
    g.ny = std::max( 1, int( box.GetHeight() / aStep ) );
    g.cell.assign( size_t( g.nx ) * g.ny, 1 );

    for( int y = 0; y < g.ny; ++y )
    {
        const int top = g.origin.y + y * aStep;
        std::vector<char> inside( g.nx, 1 ), blocked( g.nx, 0 );

        for( int s : { top + 1, top + aStep / 2, top + aStep - 1 } )
        {
            std::vector<char> row( g.nx, 0 );
            std::vector<int>  xs = crossings( aInside, s );

            for( size_t k = 0; k + 1 < xs.size(); k += 2 )
            {
                // cells wholly within [xs[k], xs[k+1]]
                const int c0 = int( std::ceil( double( xs[k] - g.origin.x ) / aStep ) );
                const int c1 = int( std::floor( double( xs[k + 1] - g.origin.x ) / aStep ) ) - 1;

                for( int c = std::max( 0, c0 ); c <= std::min( g.nx - 1, c1 ); ++c )
                    row[c] = 1;
            }

            for( int c = 0; c < g.nx; ++c )
                inside[c] &= row[c];

            std::vector<int> bs = crossings( aBlocked, s );

            for( size_t k = 0; k + 1 < bs.size(); k += 2 )
            {
                // cells touching [bs[k], bs[k+1]]
                const int c0 = int( std::floor( double( bs[k] - g.origin.x ) / aStep ) );
                const int c1 = int( std::ceil( double( bs[k + 1] - g.origin.x ) / aStep ) ) - 1;

                for( int c = std::max( 0, c0 ); c <= std::min( g.nx - 1, c1 ); ++c )
                    blocked[c] = 1;
            }
        }

        for( int c = 0; c < g.nx; ++c )
            g.at( c, y ) = inside[c] && !blocked[c];
    }

    return g;
}


struct FREE_RECT
{
    int x0, y0, x1, y1;   ///< cells, x1 / y1 exclusive
};


/// @brief Largest free rectangles, each taken out of the grid before the next; sides >= aMinCells
std::vector<FREE_RECT> freeRects( FREE_GRID& aGrid, int aCount, int aMinCells )
{
    std::vector<FREE_RECT> out;
    std::vector<int>       height( aGrid.nx );

    while( (int) out.size() < aCount )
    {
        long      bestArea = 0;
        FREE_RECT best{};
        std::fill( height.begin(), height.end(), 0 );

        for( int y = 0; y < aGrid.ny; ++y )
        {
            for( int x = 0; x < aGrid.nx; ++x )
                height[x] = aGrid.at( x, y ) ? height[x] + 1 : 0;

            // largest rectangle in the histogram, both sides at least aMinCells
            std::vector<int> stack;

            for( int x = 0; x <= aGrid.nx; ++x )
            {
                const int h = x < aGrid.nx ? height[x] : 0;

                while( !stack.empty() && height[stack.back()] >= h )
                {
                    const int top = height[stack.back()];
                    stack.pop_back();
                    const int left = stack.empty() ? 0 : stack.back() + 1;
                    const int w = x - left;
                    const long area = long( top ) * w;

                    if( top >= aMinCells && w >= aMinCells && area > bestArea )
                    {
                        bestArea = area;
                        best = { left, y - top + 1, x, y + 1 };
                    }
                }

                if( x < aGrid.nx )
                    stack.push_back( x );
            }
        }

        if( bestArea == 0 )
            break;

        for( int y = best.y0; y < best.y1; ++y )
        {
            for( int x = best.x0; x < best.x1; ++x )
                aGrid.at( x, y ) = 0;
        }

        out.push_back( best );
    }

    return out;
}


nlohmann::json blockJson( const BOARD_BLOCKS& aB, size_t aBlock, const std::map<const FOOTPRINT*, double>* aHeights,
                          const std::vector<std::map<int, int>>& aLinks, bool aDetail )
{
    const BLOCK&   blk = aB.blocks[aBlock];
    nlohmann::json parts = nlohmann::json::array();
    std::set<int>  rails;
    double         wx = 0, wy = 0, wsum = 0;
    BOX2I          bbox;
    bool           first = true;

    std::vector<int> members = blk.members;
    std::sort( members.begin(), members.end(),
               [&]( int x, int y )
               {
                   if( ( x == blk.anchor ) != ( y == blk.anchor ) )
                       return x == blk.anchor;

                   return KopenapiNaturalLess( aB.parts[x].ref, aB.parts[y].ref );
               } );

    for( int m : members )
    {
        const PART& p = aB.parts[m];

        if( aDetail )
            parts.push_back( { { "ref", p.ref }, { "by", blk.by.count( m ) ? blk.by.at( m ) : "given" } } );
        else
            parts.push_back( p.ref );

        for( int n : p.nets )
        {
            if( aB.rails.count( n ) )
                rails.insert( n );
        }

        const BOX2I  cb = p.courtyard.BBox();
        const double a = double( cb.GetWidth() ) * cb.GetHeight();
        wx += cb.Centre().x * a;
        wy += cb.Centre().y * a;
        wsum += a;

        if( first )
            bbox = cb;
        else
            bbox.Merge( cb );

        first = false;
    }

    nlohmann::json out = { { "name", blk.name }, { "kind", blk.kind }, { "parts", parts }, { "count", members.size() } };

    if( blk.anchor >= 0 )
    {
        out["anchor"] = aB.parts[blk.anchor].ref;
        out["sheet"] = aB.parts[blk.anchor].sheet;
    }

    if( first )
        return out;

    const double front = unionOf( aB, members, false ).Area();
    const double back = unionOf( aB, members, true ).Area();
    out["area_mm2"] = { { "front", mm2( front ) }, { "back", mm2( back ) } };
    out["bbox_mm"] = boxJson( bbox );
    out["bbox_fill"] = std::round( ( front + back ) / ( double( bbox.GetWidth() ) * bbox.GetHeight() ) * 100 ) / 100;

    if( wsum > 0 )
        out["centroid_mm"] = { mm( wx / wsum ), mm( wy / wsum ) };

    if( aHeights )
    {
        int    tallest = -1;
        double h = -1;

        for( int m : members )
        {
            auto it = aHeights->find( aB.parts[m].fp );

            if( it != aHeights->end() && it->second > h )
            {
                h = it->second;
                tallest = m;
            }
        }

        if( tallest >= 0 )
            out["tallest"] = { { "ref", aB.parts[tallest].ref }, { "height_mm", h } };
    }

    nlohmann::json railList = nlohmann::json::array();

    for( int n : rails )
    {
        if( NETINFO_ITEM* net = aB.parts[members[0]].fp->GetBoard()->FindNet( n ) )
            railList.push_back( str( UnescapeString( net->GetNetname() ) ) );
    }

    out["rails"] = railList;

    std::vector<std::pair<int, int>> links( aLinks[aBlock].begin(), aLinks[aBlock].end() );
    std::sort( links.begin(), links.end(), []( auto& x, auto& y ) { return x.second > y.second; } );
    nlohmann::json linkList = nlohmann::json::array();

    for( size_t k = 0; k < links.size() && k < 8; ++k )
        linkList.push_back( { { "block", aB.blocks[links[k].first].name }, { "nets", links[k].second } } );

    out["links"] = linkList;

    if( blk.kind == "connector" && blk.anchor >= 0 )
    {
        nlohmann::json edge = KopenapiPanelEdgeJson( aB.parts[blk.anchor].fp );

        if( !edge.is_null() )
            out["panel_edge"] = edge;
    }

    if( blk.kind == "ic" && members.size() > 1 )
    {
        nlohmann::json rest = nlohmann::json::array();

        for( int m : members )
        {
            if( m != blk.anchor )
                rest.push_back( aB.parts[m].ref );
        }

        out["place_group"] = { { "anchor", aB.parts[blk.anchor].ref }, { "parts", rest } };
    }

    return out;
}


/// @brief Signal nets shared between blocks: block -> other block -> nets
std::vector<std::map<int, int>> blockLinks( const BOARD_BLOCKS& aB )
{
    std::vector<std::map<int, int>> links( aB.blocks.size() );
    std::map<int, std::set<int>>    netBlocks;

    for( size_t i = 0; i < aB.parts.size(); ++i )
    {
        if( aB.partBlock[i] < 0 )
            continue;

        for( int n : aB.parts[i].nets )
        {
            if( !aB.rails.count( n ) )
                netBlocks[n].insert( aB.partBlock[i] );
        }
    }

    for( const auto& [net, blocks] : netBlocks )
    {
        for( int x : blocks )
        {
            for( int y : blocks )
            {
                if( x != y )
                    links[x][y]++;
            }
        }
    }

    return links;
}


nlohmann::json refList( const BOARD_BLOCKS& aB, const std::vector<int>& aParts )
{
    std::vector<std::string> refs;

    for( int i : aParts )
        refs.push_back( aB.parts[i].ref );

    std::sort( refs.begin(), refs.end(), KopenapiNaturalLess );
    return refs;
}


/// @brief The side of the outline's box (left / right / top / bottom) an edge connector sits on: its
/// courtyard reaches that side (within aTol) or beyond; at a corner the panel edge decides (its facing,
/// else the side its line runs along, else the courtyard's longer side); empty when it reaches none
std::string edgeSide( FOOTPRINT* aFp, const BOX2I& aCb, const BOX2I& aOb, int aTol )
{
    std::vector<std::string> touch;

    if( aCb.GetLeft() <= aOb.GetLeft() + aTol )
        touch.push_back( "left" );

    if( aCb.GetRight() >= aOb.GetRight() - aTol )
        touch.push_back( "right" );

    if( aCb.GetTop() <= aOb.GetTop() + aTol )
        touch.push_back( "top" );

    if( aCb.GetBottom() >= aOb.GetBottom() - aTol )
        touch.push_back( "bottom" );

    if( touch.size() <= 1 )
        return touch.empty() ? std::string() : touch[0];

    nlohmann::json pe = KopenapiPanelEdgeJson( aFp );

    if( pe.is_object() && pe.contains( "facing" ) && pe["facing"].is_string() )
    {
        std::string f = pe["facing"];
        f = f == "up" ? "top" : f == "down" ? "bottom" : f;

        if( std::find( touch.begin(), touch.end(), f ) != touch.end() )
            return f;
    }

    bool horizontal = aCb.GetWidth() >= aCb.GetHeight();

    if( pe.is_object() && pe.contains( "line_mm" ) && pe["line_mm"].is_array() && pe["line_mm"].size() == 2 )
    {
        const nlohmann::json& l = pe["line_mm"];
        horizontal = std::abs( l[1][0].get<double>() - l[0][0].get<double>() )
                     >= std::abs( l[1][1].get<double>() - l[0][1].get<double>() );
    }

    for( const std::string& s : touch )
    {
        if( ( s == "top" || s == "bottom" ) == horizontal )
            return s;
    }

    return touch[0];
}

} // namespace


static KOPENAPI_RESULT h_pcb_blocks( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*        board = context->GetBoard();
    GROUP_OPTIONS opt;

    if( std::optional<std::string> bad = groupOptions( aArgs, opt ) )
        return KOPENAPI_RESULT::Error( 400, *bad );

    const nlohmann::json given = aArgs.value( "blocks", nlohmann::json() );

    if( !given.is_null() && !given.is_array() )
        return KOPENAPI_RESULT::Error( 400, "blocks: [{name, parts: [refs or globs]}]" );

    std::string  error;
    BOARD_BLOCKS b = buildBlocks( aCtx, board, opt, given, error );

    if( !error.empty() )
        return KOPENAPI_RESULT::Error( 404, error );

    std::map<const FOOTPRINT*, double> heights;

    if( aArgs.value( "heights", true ) )
        heights = KopenapiFootprintHeights( board );

    const auto     links = blockLinks( b );
    nlohmann::json blocks = nlohmann::json::array();
    nlohmann::json show = nlohmann::json::array();
    const bool     detail = aArgs.value( "detail", false );
    const double   fill = std::clamp( aArgs.value( "show_fill", 0.22 ), 0.0, 1.0 );

    for( size_t i = 0; i < b.blocks.size(); ++i )
    {
        blocks.push_back( blockJson( b, i, aArgs.value( "heights", true ) ? &heights : nullptr, links, detail ) );
        const nlohmann::json& j = blocks.back();

        if( j.contains( "bbox_mm" ) && ( b.blocks[i].members.size() > 1 || b.blocks[i].kind != "connector" ) )
        {
            show.push_back( { { "kind", "region" }, { "area", j["bbox_mm"] }, { "text", b.blocks[i].name },
                              { "color", PALETTE[i % PALETTE.size()] }, { "fill_opacity", fill }, { "width_px", 3 },
                              { "group", "blocks" } } );
        }
    }

    // the board and its parts as a whole
    SHAPE_POLY_SET outline;
    double         boardArea = 0;

    if( board->GetBoardPolygonOutlines( outline, true ) && outline.OutlineCount() > 0 )
        boardArea = outline.Area();

    std::vector<int> all;

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        if( !b.parts[i].mechanical )
            all.push_back( (int) i );
    }

    const double front = unionOf( b, all, false ).Area();
    const double back = unionOf( b, all, true ).Area();

    if( aArgs.value( "show", false ) )
        overlay( aCtx, "blocks", show );

    return KOPENAPI_RESULT::Ok(
            { { "by", opt.bySheet ? "sheet" : given.is_array() && !given.empty() ? "given" : "auto" },
              { "board_area_mm2", mm2( boardArea ) },
              { "parts_area_mm2", { { "front", mm2( front ) }, { "back", mm2( back ) } } },
              { "parts_fill", boardArea > 0 ? std::round( front / boardArea * 100 ) / 100 : 0.0 },
              { "blocks", blocks },
              { "mechanical", refList( b, b.mechanical ) },
              { "unassigned", refList( b, b.unassigned ) },
              { "heights_known", heights.size() },
              { "note", "rail members (by: rail, detail: true) are given to the anchor pad nearest on their supply as "
                        "placed now: a guess for decoupling; pass blocks to fix the split" } } );
}


/// @brief Where the nets of a bus or link leave one block: the pads on its anchor (else on any member
/// holding them), the side of the part each is on (board frame) and the nets clockwise round the part
struct LINK_END
{
    int                      block = -1;
    std::string              part;
    std::map<std::string, int> sides;
    std::string              side;     ///< the side most of them are on
    std::vector<std::string> order;    ///< nets clockwise round the part (board view, y down)
    int                      pads = 0;
    int                      partIndex = -1;
    std::string              exit;     ///< the side the nets leave the part by
    std::string              exitFrom;
};


/// @brief The pads of a part on the given nets: the side of the part each is on (west / east / north /
/// south / centre, board frame), the side most are on and the nets clockwise round the part
struct PART_ORDER
{
    std::map<std::string, int>   sides;
    std::string                  side;
    std::vector<std::string>     order;    ///< nets clockwise round the part (board view, y down)
    std::map<std::string, std::string> padSide;   ///< net -> side of its first pad
    int                          pads = 0;
};


const char* padSideOf( const BOX2I& aExtent, const VECTOR2I& aAt )
{
    const VECTOR2I d = aAt - aExtent.Centre();
    const double   rx = aExtent.GetWidth() > 0 ? double( d.x ) / aExtent.GetWidth() : 0;
    const double   ry = aExtent.GetHeight() > 0 ? double( d.y ) / aExtent.GetHeight() : 0;

    return std::abs( rx ) < 0.2 && std::abs( ry ) < 0.2 ? "centre"
           : std::abs( rx ) >= std::abs( ry )          ? ( rx < 0 ? "west" : "east" )
                                                        : ( ry < 0 ? "north" : "south" );
}


BOX2I padExtent( FOOTPRINT* aFp )
{
    BOX2I extent;
    bool  first = true;

    for( PAD* pad : aFp->Pads() )
    {
        if( pad->GetNumber().IsEmpty() )
            continue;

        if( first )
            extent = BOX2I( pad->GetPosition(), VECTOR2I( 0, 0 ) );
        else
            extent.Merge( pad->GetPosition() );

        first = false;
    }

    return extent;
}


PART_ORDER partOrder( FOOTPRINT* aFp, const std::set<int>& aNets )
{
    PART_ORDER        po;
    std::vector<PAD*> pads;

    for( PAD* pad : aFp->Pads() )
    {
        if( !pad->GetNumber().IsEmpty() && aNets.count( pad->GetNetCode() ) )
            pads.push_back( pad );
    }

    if( pads.empty() )
        return po;

    const BOX2I    extent = padExtent( aFp );
    const VECTOR2I c = extent.Centre();
    double         sx = 0, sy = 0;

    for( PAD* pad : pads )
    {
        const std::string side = padSideOf( extent, pad->GetPosition() );
        const std::string net = str( UnescapeString( pad->GetNetname() ) );
        po.sides[side]++;
        po.padSide.emplace( net, side );

        const VECTOR2I d = pad->GetPosition() - c;
        const double   len = std::hypot( double( d.x ), double( d.y ) );

        if( len > 0 )
        {
            sx += d.x / len;
            sy += d.y / len;
        }
    }

    // clockwise from the side opposite the pads' mean direction, so a row reads end to end
    const double mean = std::atan2( sy, sx );
    std::vector<std::pair<double, std::string>> around;

    for( PAD* pad : pads )
    {
        const VECTOR2I d = pad->GetPosition() - c;
        double         a = std::atan2( double( d.y ), double( d.x ) ) - mean;

        while( a <= -M_PI )
            a += 2 * M_PI;

        while( a > M_PI )
            a -= 2 * M_PI;

        around.emplace_back( a, str( UnescapeString( pad->GetNetname() ) ) );
    }

    std::stable_sort( around.begin(), around.end(), []( const auto& x, const auto& y ) { return x.first < y.first; } );

    for( const auto& [a, net] : around )
    {
        if( std::find( po.order.begin(), po.order.end(), net ) == po.order.end() )
            po.order.push_back( net );
    }

    po.pads = (int) pads.size();
    int most = 0;

    for( const auto& [side, n] : po.sides )
    {
        if( n > most )
        {
            most = n;
            po.side = side;
        }
    }

    return po;
}


LINK_END linkEnd( const BOARD_BLOCKS& aB, int aBlock, const std::set<int>& aNets )
{
    LINK_END     end;
    const BLOCK& blk = aB.blocks[aBlock];
    end.block = aBlock;

    std::vector<int> order = blk.members;

    if( blk.anchor >= 0 )
    {
        order.erase( std::remove( order.begin(), order.end(), blk.anchor ), order.end() );
        order.insert( order.begin(), blk.anchor );
    }

    for( int m : order )
    {
        PART_ORDER po = partOrder( aB.parts[m].fp, aNets );

        if( po.pads == 0 )
            continue;

        end.part = aB.parts[m].ref;
        end.partIndex = m;
        end.sides = po.sides;
        end.side = po.side;
        end.order = po.order;
        end.pads = po.pads;
        break;
    }

    return end;
}


/// @brief Pairs of nets whose order at one end is not the mirror of the other's: tracks that cross
/// between two parts facing each other (clockwise at one end is counter-clockwise at the other)
int crossingsBetween( const std::vector<std::string>& aA, const std::vector<std::string>& aB )
{
    std::map<std::string, int> at;

    for( size_t k = 0; k < aB.size(); ++k )
        at[aB[k]] = (int) ( aB.size() - 1 - k );

    std::vector<int> seq;

    for( const std::string& n : aA )
    {
        if( auto it = at.find( n ); it != at.end() )
            seq.push_back( it->second );
    }

    int inversions = 0;

    for( size_t i = 0; i < seq.size(); ++i )
    {
        for( size_t j = i + 1; j < seq.size(); ++j )
            inversions += seq[i] > seq[j];
    }

    return inversions;
}


/// @brief The side a part at the board edge is left by: away from that edge (the panel edge it faces,
/// else the outline edge of aEdges its courtyard lies on, within 2 mm); empty when it is at no edge.
/// aFrom: why
std::string edgeExit( const PART& aPart, const BOX2I& aEdges, std::string& aFrom )
{
    if( aEdges.GetWidth() <= 0 )
        return std::string();

    const BOX2I body = aPart.courtyard.OutlineCount() ? aPart.courtyard.BBox() : aPart.fp->GetBoundingBox( false );
    const std::pair<std::string, long long> gap[] = {
        { "west", (long long) body.GetLeft() - aEdges.GetLeft() },
        { "east", (long long) aEdges.GetRight() - body.GetRight() },
        { "north", (long long) body.GetTop() - aEdges.GetTop() },
        { "south", (long long) aEdges.GetBottom() - body.GetBottom() } };
    std::string edge;
    long long   nearest = std::numeric_limits<long long>::max();

    for( const auto& [side, d] : gap )
    {
        if( d < nearest )
        {
            nearest = d;
            edge = side;
        }
    }

    const nlohmann::json pe = KopenapiPanelEdgeJson( aPart.fp );
    const std::string    facing = pe.is_object() ? pe.value( "facing", nlohmann::json() ).is_string()
                                                           ? pe["facing"].get<std::string>() : std::string()
                                                     : std::string();
    const std::string    faced = facing == "up" ? "north" : facing == "down" ? "south"
                                 : facing == "left" ? "west" : facing == "right" ? "east" : std::string();

    if( !faced.empty() && faced == edge )
        aFrom = "away from the board edge it faces (" + pe.value( "source", std::string( "panel edge" ) ) + ")";
    else if( nearest <= pcbIUScale.mmToIU( 2.0 ) )
        aFrom = "away from the board edge it lies on";
    else
        return std::string();

    static const std::map<std::string, std::string> opposite = {
        { "west", "east" }, { "east", "west" }, { "north", "south" }, { "south", "north" } };
    return opposite.at( edge );
}


/// @brief The side a bus leaves its end part by: a connector at the board edge away from that edge
/// (edgeExit); else as pcb_fanout_ports plans these pads: the side they are on, pads inside a grid
/// towards the far end
void setExit( const BOARD_BLOCKS& aB, const BOX2I& aEdges, const VECTOR2I& aFar, LINK_END& aEnd )
{
    if( aEnd.partIndex < 0 )
        return;

    const PART& part = aB.parts[aEnd.partIndex];

    if( part.connector )
    {
        if( std::string exit = edgeExit( part, aEdges, aEnd.exitFrom ); !exit.empty() )
        {
            aEnd.exit = exit;
            return;
        }
    }

    if( aEnd.side == "west" || aEnd.side == "east" || aEnd.side == "north" || aEnd.side == "south" )
    {
        aEnd.exit = aEnd.side;
        aEnd.exitFrom = "pcb_fanout_ports (the pads' side)";
        return;
    }

    const VECTOR2I d = aFar - padExtent( part.fp ).Centre();
    aEnd.exit = std::abs( d.x ) >= std::abs( d.y ) ? ( d.x < 0 ? "west" : "east" ) : ( d.y < 0 ? "north" : "south" );
    aEnd.exitFrom = "pcb_fanout_ports (towards the far end)";
}


nlohmann::json endJson( const BOARD_BLOCKS& aB, const LINK_END& aEnd )
{
    return { { "block", aB.blocks[aEnd.block].name }, { "part", aEnd.part }, { "side", aEnd.side },
             { "sides", aEnd.sides }, { "order", aEnd.order }, { "pads", aEnd.pads },
             { "exit", aEnd.exit }, { "exit_from", aEnd.exitFrom } };
}


static KOPENAPI_RESULT h_pcb_layout_graph( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const auto    started = std::chrono::steady_clock::now();
    BOARD*        board = context->GetBoard();
    GROUP_OPTIONS opt;

    if( std::optional<std::string> bad = groupOptions( aArgs, opt ) )
        return KOPENAPI_RESULT::Error( 400, *bad );

    std::string  error;
    BOARD_BLOCKS b = buildBlocks( aCtx, board, opt, aArgs.value( "blocks", nlohmann::json() ), error );

    if( !error.empty() )
        return KOPENAPI_RESULT::Error( 404, error );

    std::map<std::string, int> partOf;

    for( size_t i = 0; i < b.parts.size(); ++i )
        partOf[b.parts[i].ref] = (int) i;

    auto blockOfRef = [&]( const std::string& aRef )
    {
        auto it = partOf.find( aRef );
        return it == partOf.end() ? -1 : b.partBlock[it->second];
    };

    auto netName = [&]( int aCode )
    {
        NETINFO_ITEM* net = board->FindNet( aCode );
        return net ? str( UnescapeString( net->GetNetname() ) ) : std::string();
    };

    // the blocks each signal net reaches
    std::map<int, std::set<int>> netBlocks;

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        if( b.partBlock[i] < 0 )
            continue;

        for( int n : b.parts[i].nets )
        {
            if( !b.rails.count( n ) )
                netBlocks[n].insert( b.partBlock[i] );
        }
    }

    // buses: pcb_net_roles bundles of one name that share an end part, as one
    struct BUS
    {
        std::string           key;
        std::set<std::string> ends;      ///< parts
        std::set<int>         nets;
        std::set<std::string> sources;
        std::vector<std::string> bundles;
        std::map<std::string, int> endUse;   ///< part -> bundles it ends
        std::map<int, std::set<std::string>> netSources;
    };

    std::vector<BUS> buses;
    nlohmann::json   rolesBody;

    if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
    {
        KOPENAPI_RESULT r = roles->handler( aCtx, { { "min_bundle", aArgs.value( "min_bus", 3 ) } } );

        if( r.status == 200 )
            rolesBody = r.body;
    }

    for( const nlohmann::json& bundle : rolesBody.value( "bundles", nlohmann::json::array() ) )
    {
        const std::string name = bundle.value( "name", std::string() );
        const std::string key = name.substr( 0, name.find( '@' ) );
        const std::string source = bundle.value( "source", std::string( "topology" ) );
        std::set<std::string> ends;
        std::set<int>         nets;
        std::map<int, std::string> netSource;
        std::set<std::string> byStem;   ///< topology bundle: members of the name stem it is named by

        for( const nlohmann::json& e : bundle.value( "ends", nlohmann::json::array() ) )
            ends.insert( e.get<std::string>() );

        for( const nlohmann::json& g : bundle.value( "name_groups", nlohmann::json::array() ) )
        {
            if( g.value( "stem", std::string() ) != key )
                continue;

            for( const nlohmann::json& m : g.value( "members", nlohmann::json::array() ) )
            {
                if( m.is_string() )
                    byStem.insert( m.get<std::string>() );
            }
        }

        // a net alone in its name series: by its own name when that is the bus's (PM_SD_CS, or a
        // number on it: SPI_RX0)
        auto namedBy = []( const std::string& aNet, const std::string& aKey )
        {
            const std::string leaf = aNet.substr( aNet.rfind( '/' ) == std::string::npos ? 0 : aNet.rfind( '/' ) + 1 );

            return !aKey.empty() && leaf.compare( 0, aKey.size(), aKey ) == 0
                   && std::all_of( leaf.begin() + (long) aKey.size(), leaf.end(), []( char ch ) { return ch >= '0' && ch <= '9'; } );
        };

        for( const nlohmann::json& m : bundle.value( "members", nlohmann::json::array() ) )
        {
            const std::string how = source != "topology" ? source
                                    : byStem.count( m.value( "member", std::string() ) ) ? "name" : "topology";

            for( const nlohmann::json& n : m.value( "nets", nlohmann::json::array() ) )
            {
                if( NETINFO_ITEM* net = KopenapiFindNet( board, n.get<std::string>() ) )
                {
                    nets.insert( net->GetNetCode() );
                    netSource[net->GetNetCode()] = how == "topology" && namedBy( n.get<std::string>(), key ) ? "name" : how;
                }
            }
        }

        BUS* into = nullptr;

        for( BUS& bus : buses )
        {
            if( bus.key != key )
                continue;

            for( const std::string& e : ends )
            {
                if( bus.ends.count( e ) )
                    into = &bus;
            }
        }

        if( !into )
        {
            buses.emplace_back();
            buses.back().key = key;
            into = &buses.back();
        }

        into->ends.insert( ends.begin(), ends.end() );
        into->nets.insert( nets.begin(), nets.end() );
        into->sources.insert( source );
        into->bundles.push_back( name );

        for( const auto& [n, how] : netSource )
            into->netSources[n].insert( how );

        for( const std::string& e : ends )
            into->endUse[e]++;
    }

    // two buses of one name joined later by a bundle that shares an end with both
    for( bool merged = true; merged; )
    {
        merged = false;

        for( size_t i = 0; i < buses.size() && !merged; ++i )
        {
            for( size_t j = i + 1; j < buses.size() && !merged; ++j )
            {
                if( buses[i].key != buses[j].key )
                    continue;

                for( const std::string& e : buses[j].ends )
                {
                    if( buses[i].ends.count( e ) )
                    {
                        merged = true;
                        break;
                    }
                }

                if( merged )
                {
                    BUS& to = buses[i];
                    BUS& from = buses[j];
                    to.ends.insert( from.ends.begin(), from.ends.end() );
                    to.nets.insert( from.nets.begin(), from.nets.end() );
                    to.sources.insert( from.sources.begin(), from.sources.end() );
                    to.bundles.insert( to.bundles.end(), from.bundles.begin(), from.bundles.end() );

                    for( const auto& [e, n] : from.endUse )
                        to.endUse[e] += n;

                    for( const auto& [n, src] : from.netSources )
                        to.netSources[n].insert( src.begin(), src.end() );

                    buses.erase( buses.begin() + (long) j );
                }
            }
        }
    }

    std::map<int, int> busOf;   // net -> bus
    nlohmann::json     busList = nlohmann::json::array();
    const BOX2I        boardEdges = board->GetBoardEdgesBoundingBox();

    for( size_t k = 0; k < buses.size(); ++k )
    {
        BUS& bus = buses[k];

        // the trunk: the end part in most of its bundles, then the one with most of its nets
        std::string trunk;
        int         best = -1;

        for( const auto& [e, n] : bus.endUse )
        {
            if( n > best )
            {
                best = n;
                trunk = e;
            }
        }

        std::vector<std::string> names;

        for( int n : bus.nets )
        {
            busOf[n] = (int) k;
            names.push_back( netName( n ) );
        }

        std::sort( names.begin(), names.end(), []( const std::string& x, const std::string& y ) { return KopenapiNaturalLess( x, y ); } );

        std::set<int> blocks;

        for( const std::string& e : bus.ends )
        {
            if( int blk = blockOfRef( e ); blk >= 0 )
                blocks.insert( blk );
        }

        const int      trunkBlock = blockOfRef( trunk );
        nlohmann::json ends = nlohmann::json::array();
        LINK_END       trunkEnd;

        if( trunkBlock >= 0 )
            trunkEnd = linkEnd( b, trunkBlock, bus.nets );

        std::vector<LINK_END> linkEnds;
        VECTOR2I              others( 0, 0 );

        for( int blk : blocks )
        {
            if( blk == trunkBlock )
                continue;

            linkEnds.push_back( linkEnd( b, blk, bus.nets ) );

            if( linkEnds.back().partIndex >= 0 )
                others += b.parts[linkEnds.back().partIndex].centre / std::max<int>( 1, (int) blocks.size() - 1 );
        }

        if( trunkEnd.partIndex >= 0 )
            setExit( b, boardEdges, others, trunkEnd );

        for( LINK_END& end : linkEnds )
            setExit( b, boardEdges, trunkEnd.partIndex >= 0 ? b.parts[trunkEnd.partIndex].centre : others, end );

        size_t nextEnd = 0;

        for( int blk : blocks )
        {
            const LINK_END end = blk == trunkBlock ? trunkEnd : linkEnds[nextEnd++];
            nlohmann::json j = endJson( b, end );

            if( blk != trunkBlock && trunkBlock >= 0 )
                j["crossings_to_trunk"] = crossingsBetween( trunkEnd.order, end.order );

            ends.push_back( j );
        }

        nlohmann::json sources = nlohmann::json::array();

        for( const std::string& src : bus.sources )
            sources.push_back( src );

        // a net in the bus by a bundle of names (a project or schematic bus, a topology bundle's name
        // stem) is in it by that; by its end parts only otherwise
        nlohmann::json netSource = nlohmann::json::object();

        for( const auto& [n, src] : bus.netSources )
        {
            std::string pick = "topology";

            for( const std::string& v : src )
            {
                if( v != "topology" )
                    pick = v;
            }

            netSource[netName( n )] = pick;
        }

        busList.push_back( { { "name", bus.key + "@" + trunk }, { "width", bus.nets.size() }, { "nets", names },
                             { "trunk", trunkBlock >= 0 ? b.blocks[trunkBlock].name : trunk }, { "ends", ends },
                             { "bundles", bus.bundles }, { "source", sources }, { "net_source", netSource } } );
    }

    // edges between blocks: every signal net two blocks share, by bus or alone
    std::map<std::pair<int, int>, std::set<int>> pairNets;

    for( const auto& [net, blocks] : netBlocks )
    {
        if( blocks.size() < 2 )
            continue;

        for( int x : blocks )
        {
            for( int y : blocks )
            {
                if( x < y )
                    pairNets[{ x, y }].insert( net );
            }
        }
    }

    const int      listLimit = KopenapiListLimit( aArgs, 16, 1 );
    nlohmann::json edges = nlohmann::json::array();
    std::vector<std::pair<std::pair<int, int>, std::set<int>>> sortedPairs( pairNets.begin(), pairNets.end() );
    std::stable_sort( sortedPairs.begin(), sortedPairs.end(),
                      []( const auto& x, const auto& y ) { return x.second.size() > y.second.size(); } );

    for( const auto& [pair, nets] : sortedPairs )
    {
        nlohmann::json singles = nlohmann::json::array();
        int            alone = 0;
        std::set<int>  through;

        for( int n : nets )
        {
            if( auto it = busOf.find( n ); it != busOf.end() )
            {
                through.insert( it->second );
                continue;
            }

            if( alone++ < listLimit )
                singles.push_back( netName( n ) );
        }

        nlohmann::json via = nlohmann::json::array();

        for( int k : through )
            via.push_back( busList[k]["name"] );

        nlohmann::json e = { { "a", b.blocks[pair.first].name }, { "b", b.blocks[pair.second].name },
                             { "width", nets.size() }, { "buses", via }, { "singles", alone } };

        if( alone > 0 )
            e["single_nets"] = singles;

        edges.push_back( e );
    }

    // clusters: the schematic sheet of each block's anchor
    std::map<std::string, std::vector<int>> sheets;

    for( size_t i = 0; i < b.blocks.size(); ++i )
    {
        const BLOCK& blk = b.blocks[i];
        sheets[blk.anchor >= 0 ? b.parts[blk.anchor].sheet : "/"].push_back( (int) i );
    }

    nlohmann::json clusters = nlohmann::json::array();

    for( const auto& [sheet, members] : sheets )
    {
        nlohmann::json names = nlohmann::json::array();

        for( int i : members )
            names.push_back( b.blocks[i].name );

        clusters.push_back( { { "sheet", sheet }, { "blocks", names } } );
    }

    nlohmann::json blocks = nlohmann::json::array();

    for( size_t i = 0; i < b.blocks.size(); ++i )
    {
        const BLOCK& blk = b.blocks[i];
        BOX2I        box;
        bool         first = true;

        for( int m : blk.members )
        {
            const BOX2I cb = b.parts[m].courtyard.BBox();

            if( first )
                box = cb;
            else
                box.Merge( cb );

            first = false;
        }

        nlohmann::json j = { { "name", blk.name }, { "kind", blk.kind }, { "parts", blk.members.size() } };

        if( blk.anchor >= 0 )
        {
            j["anchor"] = b.parts[blk.anchor].ref;
            j["sheet"] = b.parts[blk.anchor].sheet;
        }

        if( !first )
            j["bbox_mm"] = boxJson( box );

        blocks.push_back( j );
    }

    const double ms = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();

    return KOPENAPI_RESULT::Ok( { { "blocks", blocks }, { "clusters", clusters }, { "buses", busList },
                                  { "edges", edges }, { "unassigned", refList( b, b.unassigned ) },
                                  { "ms", std::round( ms ) } } );
}


/// @brief Positions in a sequence that form its longest increasing run (kept in order), the rest out
std::vector<bool> longestIncreasing( const std::vector<int>& aSeq )
{
    const size_t     n = aSeq.size();
    std::vector<int> len( n, 1 ), prev( n, -1 );

    for( size_t i = 0; i < n; ++i )
    {
        for( size_t j = 0; j < i; ++j )
        {
            if( aSeq[j] < aSeq[i] && len[j] + 1 > len[i] )
            {
                len[i] = len[j] + 1;
                prev[i] = (int) j;
            }
        }
    }

    std::vector<bool> in( n, false );
    int               at = -1;

    for( size_t i = 0; i < n; ++i )
    {
        if( at < 0 || len[i] > len[at] )
            at = (int) i;
    }

    for( ; at >= 0; at = prev[at] )
        in[at] = true;

    return in;
}


int inversions( const std::vector<int>& aSeq )
{
    int n = 0;

    for( size_t i = 0; i < aSeq.size(); ++i )
    {
        for( size_t j = i + 1; j < aSeq.size(); ++j )
            n += aSeq[i] > aSeq[j];
    }

    return n;
}


static KOPENAPI_RESULT h_pcb_fanout_ports( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const auto    started = std::chrono::steady_clock::now();
    BOARD*        board = context->GetBoard();
    GROUP_OPTIONS opt;

    if( std::optional<std::string> bad = groupOptions( aArgs, opt ) )
        return KOPENAPI_RESULT::Error( 400, *bad );

    const std::string ref = aArgs.value( "ref", std::string() );

    if( ref.empty() )
        return KOPENAPI_RESULT::Error( 400, "ref: the package to fan out (an IC with its pads on its sides)" );

    std::string  error;
    BOARD_BLOCKS b = buildBlocks( aCtx, board, opt, nlohmann::json(), error );

    if( !error.empty() )
        return KOPENAPI_RESULT::Error( 404, error );

    int anchor = -1;

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        if( b.parts[i].ref == ref )
            anchor = (int) i;
    }

    if( anchor < 0 )
        return KOPENAPI_RESULT::Error( 404, "no footprint " + ref );

    const PART&    a = b.parts[anchor];
    const BOX2I    extent = padExtent( a.fp );
    const VECTOR2I centre = extent.Centre();
    const int      ownBlock = b.partBlock[anchor];

    // an edge connector (gold fingers included) is left away from the board edge it is at: no pin
    // leaves towards that edge, and the edge fixes its turn
    const nlohmann::json panel = KopenapiPanelEdgeJson( a.fp );
    const bool           fingers = panel.is_object() && panel.value( "source", std::string() ) == "board edge (fingers)";
    std::string          exitFrom;
    const std::string    exit = a.connector || fingers
                                        ? edgeExit( a, board->GetBoardEdgesBoundingBox(), exitFrom ) : std::string();

    // the sides clockwise in the board view (y down); a quarter turn clockwise moves a pad one on
    static const std::vector<std::string> sides = { "west", "north", "east", "south" };

    auto sideIndex = []( const std::string& aSide ) -> int
    {
        auto it = std::find( sides.begin(), sides.end(), aSide );
        return it == sides.end() ? -1 : (int) ( it - sides.begin() );
    };

    // the nets in the order a track along the side meets them, clockwise (board view, y down): along
    // the exit side when there is one (a finger row is one line, not a ring round a centre), else
    // round the package
    auto orderHere = [&]( const std::set<int>& aNets ) -> std::vector<std::string>
    {
        if( exit.empty() )
            return partOrder( a.fp, aNets ).order;

        std::vector<std::pair<int, std::string>> row;

        for( PAD* pad : a.fp->Pads() )
        {
            if( pad->GetNumber().IsEmpty() || !aNets.count( pad->GetNetCode() ) )
                continue;

            const VECTOR2I at = pad->GetPosition();
            const int      along = exit == "north" ? at.x : exit == "east" ? at.y : exit == "south" ? -at.x : -at.y;
            row.emplace_back( along, str( UnescapeString( pad->GetNetname() ) ) );
        }

        std::stable_sort( row.begin(), row.end(), []( const auto& x, const auto& y ) { return x.first < y.first; } );
        std::vector<std::string> order;

        for( const auto& [along, net] : row )
        {
            if( std::find( order.begin(), order.end(), net ) == order.end() )
                order.push_back( net );
        }

        return order;
    };

    auto towards = [&]( const VECTOR2I& aTo ) -> std::string
    {
        const VECTOR2I d = aTo - centre;
        return std::abs( d.x ) >= std::abs( d.y ) ? ( d.x < 0 ? "west" : "east" ) : ( d.y < 0 ? "north" : "south" );
    };

    // the part each signal net goes to: the nearest in another block, else the nearest in its own
    struct PORT
    {
        PAD*        pad = nullptr;
        std::string net;
        int         code = 0;
        std::string side;      ///< the side it leaves by
        std::string padSide;   ///< the side of the package its pad is on
        int         to = -1;
        std::string toSide;
        std::string kind;   ///< signal, local, rail
        bool        via = false;
        std::string why;
        int         turn = -1;   ///< 0 straight, 1 corner, 2 round the part
        int         next = -1;   ///< round the part: the side it passes first
    };

    // the layer a pad is on: F / B for SMD, THT where every layer reaches it (a layer change is free there)
    auto padLayer = []( PAD* aPad ) -> std::string
    {
        if( aPad->GetAttribute() == PAD_ATTRIB::PTH || aPad->GetAttribute() == PAD_ATTRIB::NPTH )
            return "THT";

        return aPad->IsOnLayer( F_Cu ) ? "F" : "B";
    };

    // a group: nets from one side of the package to one part, arriving on one side and layer of it
    struct GROUP_KEY
    {
        int         farPart;
        std::string side, farSide, farLayer;

        bool operator<( const GROUP_KEY& o ) const
        {
            return std::tie( farPart, side, farSide, farLayer ) < std::tie( o.farPart, o.side, o.farSide, o.farLayer );
        }
    };

    std::vector<PORT>                  ports;
    std::map<GROUP_KEY, std::set<int>> toNets;
    std::map<GROUP_KEY, bool>          thtEnd;
    std::set<int>                      seen;

    for( PAD* pad : a.fp->Pads() )
    {
        const int code = pad->GetNetCode();

        if( pad->GetNumber().IsEmpty() || code <= 0 )
            continue;

        PORT port;
        port.pad = pad;
        port.code = code;
        port.net = str( UnescapeString( pad->GetNetname() ) );
        port.padSide = padSideOf( extent, pad->GetPosition() );
        port.side = exit.empty() ? port.padSide : exit;

        if( b.rails.count( code ) )
        {
            port.kind = "rail";
            port.why = "to its plane: a via in or by the pad";
            ports.push_back( port );
            continue;
        }

        double best = std::numeric_limits<double>::max();
        bool   bestOther = false;

        for( size_t j = 0; j < b.parts.size(); ++j )
        {
            if( (int) j == anchor || std::find( b.parts[j].nets.begin(), b.parts[j].nets.end(), code ) == b.parts[j].nets.end() )
                continue;

            const bool   other = b.partBlock[j] != ownBlock;
            const double d = ( b.parts[j].centre - centre ).EuclideanNorm();

            if( ( other && !bestOther ) || ( other == bestOther && d < best ) )
            {
                best = d;
                bestOther = other;
                port.to = (int) j;
            }
        }

        if( port.to < 0 )
            continue;

        port.kind = bestOther ? "signal" : "local";
        port.toSide = towards( b.parts[port.to].centre );

        if( seen.insert( code ).second )
        {
            PAD* farPad = nullptr;

            for( PAD* fp : b.parts[port.to].fp->Pads() )
            {
                if( fp->GetNetCode() == code && !fp->GetNumber().IsEmpty() )
                {
                    farPad = fp;
                    break;
                }
            }

            if( farPad )
            {
                const GROUP_KEY key{ port.to, port.side, padSideOf( padExtent( b.parts[port.to].fp ), farPad->GetPosition() ),
                                     padLayer( farPad ) };
                toNets[key].insert( code );
                thtEnd[key] = thtEnd[key] || key.farLayer == "THT" || padLayer( pad ) == "THT";
            }
        }

        ports.push_back( port );
    }

    // per far part with two nets or more: the order at the anchor against the far end's, mirrored as two
    // parts facing each other are; the longest run in order stays on the pads' layer, the rest change layer
    // by a via; what still crosses among those is left for L2 / L3 (another side, the part mirrored)
    nlohmann::json groups = nlohmann::json::array();
    std::set<int>  viaNets;
    int            crossingsLeft = 0;

    for( const auto& [key, nets] : toNets )
    {
        const int farPart = key.farPart;

        if( nets.size() < 2 )
            continue;

        const std::vector<std::string> here = orderHere( nets );
        const PART_ORDER               there = partOrder( b.parts[farPart].fp, nets );
        std::map<std::string, int> at;

        for( size_t k = 0; k < there.order.size(); ++k )
            at[there.order[k]] = (int) ( there.order.size() - 1 - k );

        std::vector<int>         seq;
        std::vector<std::string> names;

        for( const std::string& n : here )
        {
            if( auto it = at.find( n ); it != at.end() )
            {
                seq.push_back( it->second );
                names.push_back( n );
            }
        }

        const std::vector<bool> keep = longestIncreasing( seq );
        std::vector<int>         rest;
        nlohmann::json           kept = nlohmann::json::array(), changed = nlohmann::json::array();

        for( size_t k = 0; k < seq.size(); ++k )
        {
            if( keep[k] )
            {
                kept.push_back( names[k] );
            }
            else
            {
                rest.push_back( seq[k] );
                changed.push_back( names[k] );

                if( NETINFO_ITEM* net = KopenapiFindNet( board, names[k] ); net && !thtEnd[key] )
                    viaNets.insert( net->GetNetCode() );
            }
        }

        const int left = inversions( rest );
        crossingsLeft += left;

        groups.push_back( { { "to", b.parts[farPart].ref }, { "side", key.side }, { "to_pads_side", key.farSide },
                            { "to_layer", key.farLayer }, { "layer_change_at_pin", thtEnd[key] },
                            { "to_block", b.partBlock[farPart] >= 0 ? b.blocks[b.partBlock[farPart]].name : std::string() },
                            { "to_side", towards( b.parts[farPart].centre ) }, { "width", seq.size() },
                            { "order_here", names }, { "order_there", there.order },
                            { "crossings", inversions( seq ) }, { "kept_on_layer", kept }, { "via_to_other_layer", changed },
                            { "crossings_left", left } } );
    }

    // the turn each signal takes from the side its pad is on to the side its far part lies: straight out,
    // round one corner, or round the part; and the quarter turns of the part that make them cheapest
    auto turnOf = [&]( int aOwn, int aTo )
    {
        const int d = ( ( aTo - aOwn ) % 4 + 4 ) % 4;
        return d == 0 ? 0 : d == 2 ? 2 : 1;
    };

    nlohmann::json byTurn = nlohmann::json::array();
    int            bestTurns = 0;
    int            bestCost = std::numeric_limits<int>::max();

    for( int q = 0; q < ( exit.empty() ? 4 : 1 ); ++q )
    {
        int cost = 0, straight = 0, corner = 0, round = 0;

        for( const PORT& p : ports )
        {
            const int own = sideIndex( p.side ), to = sideIndex( p.toSide );

            if( p.kind != "signal" || own < 0 || to < 0 )
                continue;

            const int t = turnOf( ( own + q ) % 4, to );
            cost += t == 0 ? 0 : t == 1 ? 1 : 3;
            ( t == 0 ? straight : t == 1 ? corner : round )++;
        }

        byTurn.push_back( { { "quarter_turns_cw", q }, { "straight", straight }, { "corner", corner },
                            { "round_the_part", round }, { "cost", cost } } );

        if( cost < bestCost )
        {
            bestCost = cost;
            bestTurns = q;
        }
    }

    // the ring's geometry: lines a given offset past the courtyard, per side (0 west, 1 north, 2 east, 3 south)
    const BOX2I           court = courtyardOf( a.fp ).BBox();
    static const VECTOR2I outward[4] = { { -1, 0 }, { 0, -1 }, { 1, 0 }, { 0, 1 } };

    auto lineAt = [&]( int aSide, int aOff ) -> int
    {
        switch( aSide )
        {
        case 0: return court.GetLeft() - aOff;
        case 1: return court.GetTop() - aOff;
        case 2: return court.GetRight() + aOff;
        default: return court.GetBottom() + aOff;
        }
    };

    auto onSide = [&]( int aSide, int aOff, const VECTOR2I& aAt )
    {
        return aSide % 2 == 0 ? VECTOR2I( lineAt( aSide, aOff ), aAt.y ) : VECTOR2I( aAt.x, lineAt( aSide, aOff ) );
    };

    auto cornerOf = [&]( int aSide1, int aSide2, int aOff )
    {
        const int across = aSide1 % 2 == 0 ? aSide1 : aSide2, along = aSide1 % 2 == 0 ? aSide2 : aSide1;
        return VECTOR2I( lineAt( across, aOff ), lineAt( along, aOff ) );
    };

    // each pin's nearest far pin of its net
    auto farPinOf = [&]( const PORT& p )
    {
        VECTOR2I farPin = b.parts[p.to].centre;
        double   nearest = std::numeric_limits<double>::max();

        for( PAD* fp : b.parts[p.to].fp->Pads() )
        {
            if( fp->GetNetCode() == p.code && ( fp->GetPosition() - p.pad->GetPosition() ).EuclideanNorm() < nearest )
            {
                nearest = ( fp->GetPosition() - p.pad->GetPosition() ).EuclideanNorm();
                farPin = fp->GetPosition();
            }
        }

        return farPin;
    };

    // the way round the part per side and far part, from geometry: each pin's path modelled as its lane would
    // lie (out to its offset, the pin nearest the corner it turns at innermost, a track and a clearance apart,
    // round both corners, then straight to its far pin), against the straight ways out of the package's other
    // pins on its layer. The way whose paths cross fewer (each other or those ways out) is taken, then the one
    // whose paths are shorter in total
    struct WAY_PATH
    {
        PCB_LAYER_ID          layer;
        std::vector<VECTOR2I> pts;
    };

    // the pins that change layer at their fan-out via: out of order with their far end, or a far end only on the
    // other side's copper
    for( PORT& p : ports )
    {
        if( viaNets.count( p.code ) )
        {
            p.via = true;
            p.why = "out of order with its far end: a via to the other layer";
        }
        else if( p.to >= 0 && p.kind != "rail" && padLayer( p.pad ) != "THT" )
        {
            // a far end only on the other side's copper: the pin changes layer at its fan-out via
            for( PAD* far : b.parts[p.to].fp->Pads() )
            {
                if( far->GetNetCode() == p.code && padLayer( far ) != "THT" && padLayer( far ) != padLayer( p.pad ) )
                {
                    p.via = true;
                    p.why = "its far end is on the other layer: a via to it";
                    break;
                }
            }
        }
    }

    std::map<std::pair<int, int>, std::vector<const PORT*>> roundGroups;   // (own side, far part) -> its pins
    std::vector<WAY_PATH>                                   waysOut;      // the other pins' straight ways out

    for( const PORT& p : ports )
    {
        const int own = sideIndex( p.side ), to = sideIndex( p.toSide );

        if( own < 0 || p.pad->GetAttribute() != PAD_ATTRIB::SMD )
            continue;

        if( p.to >= 0 && to >= 0 && turnOf( own, to ) == 2 )
        {
            roundGroups[{ own, p.to }].push_back( &p );
        }
        else if( p.kind != "rail" && !p.via && ( p.to < 0 || to < 0 || turnOf( own, to ) == 0 ) )
        {
            const VECTOR2I at = p.pad->GetPosition();
            waysOut.push_back( { p.pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu, { at, onSide( own, 0, at ) + outward[own] * ( court.GetWidth() + court.GetHeight() ) } } );
        }
    }

    std::map<std::pair<int, int>, int> roundWay;   // (own side, far part) -> the side it passes first
    nlohmann::json                     roundChoice = nlohmann::json::array();

    for( const auto& [key, pins] : roundGroups )
    {
        const int own = key.first, to = ( own + 2 ) % 4;
        int       bestNext = -1;
        int       bestCross = std::numeric_limits<int>::max();
        double    bestLength = std::numeric_limits<double>::max();
        nlohmann::json ways = nlohmann::json::array();

        for( int next : { ( own + 1 ) % 4, ( own + 3 ) % 4 } )
        {
            const VECTOR2I        corner = cornerOf( own, next, 0 );
            std::vector<WAY_PATH> paths;
            std::vector<std::pair<int64_t, const PORT*>> byCorner;

            for( const PORT* p : pins )
                byCorner.emplace_back( ( p->pad->GetPosition() - corner ).EuclideanNorm(), p );

            std::sort( byCorner.begin(), byCorner.end(),
                       []( const auto& x, const auto& y ) { return x.first < y.first; } );

            double length = 0;

            for( size_t rank = 0; rank < byCorner.size(); ++rank )
            {
                const PORT*  p = byCorner[rank].second;
                NETCLASS*    netclass = p->pad->GetEffectiveNetClass();
                const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
                const int    clearance = std::max( { ds.m_MinClearance, netclass && netclass->HasClearance() ? netclass->GetClearance() : 0, 1 } );
                const int    width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : ds.GetCurrentTrackWidth();
                const int    off = clearance + width / 2 + (int) rank * ( width + clearance );
                const VECTOR2I at = p->pad->GetPosition();
                WAY_PATH     path{ p->pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu,
                                   { at, onSide( own, off, at ), cornerOf( own, next, off ), cornerOf( next, to, off ), farPinOf( *p ) } };

                for( size_t i = 0; i + 1 < path.pts.size(); ++i )
                    length += ( path.pts[i + 1] - path.pts[i] ).EuclideanNorm();

                paths.push_back( std::move( path ) );
            }

            auto crossCount = [&]( const WAY_PATH& x, const WAY_PATH& y )
            {
                int n = 0;

                for( size_t i = 0; x.layer == y.layer && i + 1 < x.pts.size(); ++i )
                {
                    for( size_t k = 0; k + 1 < y.pts.size(); ++k )
                    {
                        // the first run of each leaves its own pad: two pins' ways out never meet there
                        if( i == 0 && k == 0 )
                            continue;

                        n += SEG( x.pts[i], x.pts[i + 1] ).Intersects( SEG( y.pts[k], y.pts[k + 1] ) ) ? 1 : 0;
                    }
                }

                return n;
            };

            int cross = 0;

            for( size_t x = 0; x < paths.size(); ++x )
            {
                for( size_t y = x + 1; y < paths.size(); ++y )
                    cross += crossCount( paths[x], paths[y] );

                for( const WAY_PATH& w : waysOut )
                    cross += crossCount( paths[x], w );
            }

            ways.push_back( { { "sides", { sides[own], sides[next], sides[to] } }, { "crossings", cross },
                              { "length_mm", std::round( length / pcbIUScale.IU_PER_MM * 100 ) / 100 } } );

            if( cross < bestCross || ( cross == bestCross && length < bestLength ) )
            {
                bestCross = cross;
                bestLength = length;
                bestNext = next;
            }
        }

        roundWay[key] = bestNext;
        roundChoice.push_back( { { "side", sides[own] }, { "to", b.parts[key.second].ref }, { "pins", pins.size() },
                                 { "round", bestNext == ( own + 1 ) % 4 ? "cw" : "ccw" }, { "ways", ways } } );
    }

    auto roundNext = [&]( int aOwn, int aTo ) -> int
    {
        auto it = roundWay.find( { aOwn, aTo } );
        return it != roundWay.end() ? it->second : ( aOwn + 1 ) % 4;
    };

    static const char* turnName[3] = { "straight", "corner", "round_the_part" };
    std::map<std::string, nlohmann::json> bySide;
    std::map<std::string, int>            counts;

    for( PORT& p : ports )
    {
        nlohmann::json j = { { "pad", str( p.pad->GetNumber() ) }, { "net", p.net }, { "kind", p.kind },
                             { "layer", p.via ? "other" : "pad" } };

        if( p.padSide != p.side )
            j["pad_side"] = p.padSide;

        if( p.to >= 0 )
        {
            const int own = sideIndex( p.side ), to = sideIndex( p.toSide );
            j["to"] = b.parts[p.to].ref;
            j["to_side"] = p.toSide;

            if( own >= 0 && to >= 0 )
            {
                p.turn = turnOf( own, to );
                j["turn"] = turnName[turnOf( own, to )];
                counts[turnName[turnOf( own, to )]] += p.kind == "signal";

                // round the part: one way for all the pins of a side going to one far part (a bus keeps its
                // order only so), by the side their far pins lie off the line through the centre, else by
                // their pins' half of the side
                if( turnOf( own, to ) == 2 )
                {
                    const int next = roundNext( own, p.to );
                    p.next = next;
                    j["round"] = next == ( own + 1 ) % 4 ? "cw" : "ccw";
                    j["sides"] = { sides[own], sides[next], sides[to] };
                }
            }
        }

        if( !p.why.empty() )
            j["why"] = p.why;

        if( !bySide.count( p.side ) )
            bySide[p.side] = nlohmann::json::array();

        bySide[p.side].push_back( j );
    }

    // lanes (asked): the ring round the package each signal leaving on its pad's layer runs in to its
    // turn, planned before any route. A lane runs out of its pad to its offset, along the courtyard (cw
    // or ccw) to the corner it turns at (round the part: on along the next side to the corner after)
    // and out there. A lane whose run takes in another's lies outside it (the pin nearest a corner turns
    // innermost), a pitch out from every lane its run overlaps; a lane meeting other parts' pads or other
    // nets' copper moves out a pitch at a time, the lanes outside it with it
    nlohmann::json lanesOut;

    if( aArgs.value( "lanes", false ) )
    {
        const BOX2I                  edges = board->GetBoardEdgesBoundingBox();
        const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
        const int64_t                wide = court.GetWidth(), high = court.GetHeight(), loop = 2 * ( wide + high );
        const int64_t                sideStart[5] = { 0, high, high + wide, 2 * high + wide, loop };   // clockwise from the south-west corner

        auto perimeter = [&]( int aSide, const VECTOR2I& aAt ) -> int64_t
        {
            switch( aSide )
            {
            case 0: return std::clamp<int64_t>( court.GetBottom() - aAt.y, 0, high );
            case 1: return high + std::clamp<int64_t>( aAt.x - court.GetLeft(), 0, wide );
            case 2: return high + wide + std::clamp<int64_t>( aAt.y - court.GetTop(), 0, high );
            default: return 2 * high + wide + std::clamp<int64_t>( court.GetRight() - aAt.x, 0, wide );
            }
        };

        struct LANE
        {
            PORT*                 port = nullptr;
            PCB_LAYER_ID          layer = F_Cu;
            int                   own = 0, next = 0, to = 0, turn = 0;
            bool                  cw = true;
            int                   width = 0, clearance = 0;
            int64_t               lo = 0, hi = 0;   ///< its run along the courtyard (clockwise place)
            VECTOR2I              start;
            int                   rank = 0, off = 0, nominal = 0;
            bool                  blocked = false;
            std::vector<VECTOR2I> pts;
            std::set<std::string> blockedBy;
        };

        std::vector<LANE> lanes;
        int               pitch = 1, base = 0;

        for( PORT& p : ports )
        {
            if( p.kind != "signal" || p.via || p.turn < 0 || p.pad->GetAttribute() != PAD_ATTRIB::SMD )
                continue;

            LANE l;
            l.port = &p;
            l.layer = p.pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
            l.own = sideIndex( p.side );
            l.to = sideIndex( p.toSide );
            l.turn = p.turn;
            l.next = p.turn == 2 ? p.next : l.to;

            if( l.own < 0 || l.to < 0 || l.next < 0 )
                continue;

            l.cw = l.turn == 0 || l.next == ( l.own + 1 ) % 4;

            NETCLASS*   netclass = p.pad->GetEffectiveNetClass();
            const BOX2I box = p.pad->GetBoundingBox();
            const int   half = l.own % 2 == 0 ? box.GetWidth() / 2 : box.GetHeight() / 2;
            const int   across = l.own % 2 == 0 ? box.GetHeight() : box.GetWidth();
            l.clearance = std::max( { ds.m_MinClearance, netclass && netclass->HasClearance() ? netclass->GetClearance() : 0, 1 } );
            l.width = std::min( across, netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : ds.GetCurrentTrackWidth() );
            l.start = box.Centre() + outward[l.own] * half;

            const int64_t at = perimeter( l.own, p.pad->GetPosition() );
            const int     last = l.turn == 2 ? l.next : l.own;   // the side it leaves the ring at the end of

            if( l.turn == 0 )
            {
                l.lo = l.hi = at;
            }
            else if( l.cw )
            {
                l.lo = at;
                l.hi = sideStart[last + 1] < at ? sideStart[last + 1] + loop : sideStart[last + 1];
            }
            else
            {
                l.hi = at;
                l.lo = sideStart[last] > at ? sideStart[last] - loop : sideStart[last];
            }

            if( l.turn > 0 )
            {
                pitch = std::max( pitch, l.width + l.clearance );
                base = std::max( base, l.clearance + l.width / 2 );
            }

            lanes.push_back( l );
        }

        // the room round the package as pcb_floorplan's boxes give it: the parts' courtyards, those within a
        // pitch of each other one box; on each side from the courtyard to the nearest other box facing it, or
        // to the board's edge
        int64_t room[4] = { court.GetLeft() - (int64_t) edges.GetLeft(), court.GetTop() - (int64_t) edges.GetTop(),
                            edges.GetRight() - (int64_t) court.GetRight(), edges.GetBottom() - (int64_t) court.GetBottom() };
        {
            std::vector<BOX2I> boxes;
            std::vector<bool>  own;

            for( FOOTPRINT* fp : board->Footprints() )
            {
                boxes.push_back( courtyardOf( fp ).BBox() );
                own.push_back( fp == a.fp );
            }

            for( bool grew = true; grew; )
            {
                grew = false;

                for( size_t i = 0; i < boxes.size(); ++i )
                {
                    for( size_t j = 0; !own[i] && j < boxes.size(); ++j )
                    {
                        BOX2I grown = boxes[j];
                        grown.Inflate( pitch );

                        if( own[j] && grown.Intersects( boxes[i] ) )
                        {
                            own[i] = true;
                            grew = true;
                        }
                    }
                }
            }

            for( size_t i = 0; i < boxes.size(); ++i )
            {
                const BOX2I& x = boxes[i];

                if( own[i] )
                    continue;

                const bool acrossY = x.GetBottom() > court.GetTop() && x.GetTop() < court.GetBottom();
                const bool acrossX = x.GetRight() > court.GetLeft() && x.GetLeft() < court.GetRight();

                if( acrossY && x.GetRight() <= court.GetLeft() )
                    room[0] = std::min<int64_t>( room[0], court.GetLeft() - x.GetRight() );

                if( acrossX && x.GetBottom() <= court.GetTop() )
                    room[1] = std::min<int64_t>( room[1], court.GetTop() - x.GetBottom() );

                if( acrossY && x.GetLeft() >= court.GetRight() )
                    room[2] = std::min<int64_t>( room[2], x.GetLeft() - court.GetRight() );

                if( acrossX && x.GetTop() >= court.GetBottom() )
                    room[3] = std::min<int64_t>( room[3], x.GetTop() - court.GetBottom() );
            }
        }

        // how far past the courtyard the ways out of each side's pins reach, as pcb_route's escape fronts have
        // them (KopenapiPadEscapeDepth past the pad's end)
        int64_t wayOut[4] = { 0, 0, 0, 0 };

        for( const PORT& q : ports )
        {
            const int qSide = sideIndex( q.padSide );

            if( qSide < 0 )
                continue;

            const int64_t depth = KopenapiPadEscapeDepth( board, q.pad );
            const BOX2I   box = q.pad->GetBoundingBox();
            const int64_t end = qSide == 0 ? court.GetLeft() - (int64_t) box.GetLeft() : qSide == 1 ? court.GetTop() - (int64_t) box.GetTop()
                                : qSide == 2 ? box.GetRight() - (int64_t) court.GetRight() : box.GetBottom() - (int64_t) court.GetBottom();
            wayOut[qSide] = std::max( wayOut[qSide], end + depth );
        }

        // the farthest offset a lane may take: its track and a clearance inside the room of every side it runs by
        auto limit = [&]( const LANE& l ) -> int64_t
        {
            const int64_t r = l.turn == 2 ? std::min( room[l.own], room[l.next] ) : room[l.own];
            return r - l.width / 2 - l.clearance;
        };

        auto overlaps = [&]( const LANE& x, const LANE& y )
        {
            for( int64_t k : { -loop, (int64_t) 0, loop } )
            {
                if( x.lo <= y.hi + k && y.lo + k <= x.hi )
                    return true;
            }

            return false;
        };

        auto build = [&]( LANE& l, int aOff )
        {
            l.off = aOff;
            l.pts = { l.start, onSide( l.own, aOff, l.start ) };

            if( l.turn == 0 )
                return;

            l.pts.push_back( cornerOf( l.own, l.next, aOff ) );

            if( l.turn == 2 )
                l.pts.push_back( cornerOf( l.next, l.to, aOff ) );

            l.pts.push_back( l.pts.back() + outward[l.to] * pitch );
        };

        // what a lane meets past the courtyard: other parts' pads and other nets' tracks and vias on its layer
        auto meets = [&]( const LANE& l, std::set<std::string>& aWho )
        {
            std::vector<VECTOR2I> pts = l.pts;
            pts[0] = onSide( l.own, 0, pts[0] );

            for( size_t i = 0; i + 1 < pts.size(); ++i )
            {
                const SHAPE_SEGMENT seg( pts[i], pts[i + 1], l.width );
                BOX2I               box = seg.BBox();
                box.Inflate( l.clearance );

                for( FOOTPRINT* fp : board->Footprints() )
                {
                    if( fp == a.fp || !fp->GetBoundingBox().Intersects( box ) )
                        continue;

                    for( PAD* pad : fp->Pads() )
                    {
                        if( pad->GetNetCode() != l.port->code && pad->IsOnLayer( l.layer )
                            && pad->GetBoundingBox().Intersects( box )
                            && pad->GetEffectiveShape( l.layer )->Collide( &seg, l.clearance ) )
                        {
                            aWho.insert( str( fp->GetReference() ) );
                        }
                    }
                }

                for( PCB_TRACK* t : board->Tracks() )
                {
                    if( t->GetNetCode() != l.port->code && t->IsOnLayer( l.layer ) && t->GetBoundingBox().Intersects( box )
                        && t->GetEffectiveShape( l.layer )->Collide( &seg, l.clearance ) )
                    {
                        aWho.insert( t->Type() == PCB_VIA_T ? "via" : "track" );
                    }
                }
            }

            return !aWho.empty();
        };

        // the shortest runs first: each a pitch outside every lane its run overlaps
        std::vector<size_t> order;

        for( size_t k = 0; k < lanes.size(); ++k )
        {
            if( lanes[k].turn > 0 )
                order.push_back( k );
        }

        std::stable_sort( order.begin(), order.end(), [&]( size_t x, size_t y )
                          {
                              const int64_t rx = lanes[x].hi - lanes[x].lo, ry = lanes[y].hi - lanes[y].lo;
                              return rx != ry ? rx < ry : lanes[x].lo < lanes[y].lo;
                          } );

        // the via spots of the package's fan-out (pcb_fanout ports, dry run: via_spots): the rails' and the moved
        // pins' laid, and for every other pin not turning, where its via goes should it leave by one. The lanes
        // keep clear of them, past their pads
        struct SPOT
        {
            VECTOR2I   at;
            int        dia = 0;
            int        net = 0;
            const PAD* pad = nullptr;
        };

        std::vector<SPOT>    spots;
        std::set<const PAD*> spotPads;
        nlohmann::json       spotsOut = nlohmann::json::array();

        if( std::optional<KOPENAPI_METHOD> fanout = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout" ) )
        {
            std::set<const PAD*> turning;

            for( const LANE& l : lanes )
            {
                if( l.turn > 0 )
                    turning.insert( l.port->pad );
            }

            const KOPENAPI_RESULT fan = fanout->handler( aCtx, { { "ref", str( a.fp->GetReference() ) }, { "ports", true }, { "dry_run", true },
                                                                 { "full", true }, { "zones", "refill" } } );
            const nlohmann::json  noSpots = nlohmann::json::array();
            const nlohmann::json& rows = fan.status == 200 && fan.body.contains( "via_spots" ) && fan.body["via_spots"].is_array()
                                                 ? fan.body["via_spots"] : noSpots;
            const int             dia = fan.status == 200 && fan.body.contains( "via" )
                                                ? pcbIUScale.mmToIU( fan.body["via"].value( "diameter_mm", 0.0 ) ) : 0;

            for( const nlohmann::json& row : rows )
            {
                const std::string name = row.value( "pad", std::string() );
                PAD*              pad = nullptr;

                for( PAD* q : a.fp->Pads() )
                {
                    if( str( a.fp->GetReference() ) + "." + str( q->GetNumber() ) == name )
                        pad = q;
                }

                if( !pad || !row.contains( "at_mm" ) || ( !row.value( "laid", false ) && turning.count( pad ) ) )
                    continue;

                SPOT spot;
                spot.at = VECTOR2I( pcbIUScale.mmToIU( row["at_mm"][0].get<double>() ), pcbIUScale.mmToIU( row["at_mm"][1].get<double>() ) );
                spot.dia = dia;
                spot.net = pad->GetNetCode();
                spot.pad = pad;
                spots.push_back( spot );
                spotPads.insert( pad );
                spotsOut.push_back( { { "pad", name }, { "at_mm", row["at_mm"] } } );
            }
        }

        // a lane's runs past its pad keep a via's half, a clearance and their half width from each spot
        auto clearsSpots = [&]( const LANE& l )
        {
            for( const SPOT& spot : spots )
            {
                if( spot.net == l.port->code )
                    continue;

                const int apart = spot.dia / 2 + l.clearance + l.width / 2;

                for( size_t i = 1; i + 1 < l.pts.size(); ++i )
                {
                    if( SEG( l.pts[i], l.pts[i + 1] ).Distance( spot.at ) < apart )
                        return false;
                }
            }

            return true;
        };

        for( size_t m = 0; m < order.size(); ++m )
        {
            LANE& l = lanes[order[m]];
            int   off = base;

            l.nominal = off;   // where it lies with nothing in the way

            for( size_t q = 0; q < m; ++q )
            {
                const LANE& inner = lanes[order[q]];

                if( overlaps( l, inner ) )
                {
                    l.rank = std::max( l.rank, inner.rank + 1 );
                    off = std::max( off, inner.off + pitch );
                    l.nominal = std::max( l.nominal, inner.nominal + pitch );
                }
            }

            // out past the fan-out's vias: where it lies with nothing else in the way
            for( build( l, off ); !clearsSpots( l ) && off + pitch <= limit( l ); build( l, off ) )
            {
                off += pitch;
                l.nominal += pitch;
            }

            // out past other copper while the room lasts
            for( int step = 0;; ++step )
            {
                build( l, off + step * pitch );
                std::set<std::string> who;

                if( !meets( l, who ) && off + step * pitch <= limit( l ) )
                    break;

                l.blockedBy.insert( who.begin(), who.end() );

                if( off + ( step + 1 ) * pitch > limit( l ) )
                {
                    l.blockedBy.insert( "room" );

                    l.blocked = true;
                    build( l, off );
                    break;
                }
            }
        }

        auto mmAt = []( const VECTOR2I& aAt )
        {
            return nlohmann::json::array( { std::round( aAt.x / pcbIUScale.IU_PER_MM * 1000.0 ) / 1000.0,
                                            std::round( aAt.y / pcbIUScale.IU_PER_MM * 1000.0 ) / 1000.0 } );
        };

        std::set<size_t>      yielded;   // lanes that yield: onto the other layer by the pin's fan-out via
        std::set<const PORT*> dives;     // pins whose straight way out yields: down by their fan-out via

        // the lanes leaving by one corner to one side end on one front, the next slot (a pitch) past the
        // outermost and clear of the ways out of that side's pins, within the room on that side: they leave it
        // side by side, none cutting across the ends of those inside it nor across a pin's way out; a straight
        // one runs out past the lanes running by its pin
        auto finish = [&]()
        {
            std::map<std::pair<int, int>, int> front;

            for( size_t k : order )
            {
                if( yielded.count( k ) )
                    continue;

                build( lanes[k], lanes[k].off );
                const std::pair<int, int> key( lanes[k].turn == 2 ? lanes[k].next : lanes[k].own, lanes[k].to );
                front[key] = std::max( front[key], lanes[k].off );
            }

            for( size_t k : order )
            {
                if( yielded.count( k ) )
                    continue;

                LANE&     l = lanes[k];
                const int64_t outer = front[{ l.turn == 2 ? l.next : l.own, l.to }];
                const int64_t past = std::max<int64_t>( outer + pitch, wayOut[l.to] + l.clearance + l.width / 2 );
                const int     far = lineAt( l.to, (int) std::max<int64_t>( outer, std::min<int64_t>( past, room[l.to] - l.width / 2 - l.clearance ) ) );

                // published to the micron (polyline_mm): rounded outward, so the front holds as read
                const double  um = pcbIUScale.mmToIU( 0.001 );
                const int     farLine = KiROUND( ( outward[l.to].x + outward[l.to].y > 0 ? std::ceil( far / um ) : std::floor( far / um ) ) * um );

                if( l.to % 2 == 0 )
                    l.pts.back().x = farLine;
                else
                    l.pts.back().y = farLine;
            }

            for( size_t x = 0; x < lanes.size(); ++x )
            {
                LANE& l = lanes[x];

                if( l.turn != 0 || yielded.count( x ) )
                    continue;

                int off = base;

                for( size_t k : order )
                {
                    if( !yielded.count( k ) && overlaps( l, lanes[k] ) )
                        off = std::max( off, lanes[k].off + pitch );
                }

                build( l, off );
                l.pts.push_back( l.pts.back() + outward[l.own] * pitch );
                l.blockedBy.clear();
                meets( l, l.blockedBy );
                l.blocked = !l.blockedBy.empty();
            }
        };

        auto within = [&]( int64_t aAt, int64_t aLo, int64_t aHi )
        {
            for( int64_t k : { -loop, (int64_t) 0, loop } )
            {
                if( aLo < aAt + k && aAt + k < aHi )
                    return true;
            }

            return false;
        };

        std::map<const PORT*, size_t> laneOf;

        for( size_t k = 0; k < lanes.size(); ++k )
            laneOf[lanes[k].port] = k;

        // the pins a turning lane runs in front of on its layer whose way out is straight out there (not a
        // lane nested inside it the same way round, not a rail pin, not one on the other layer)
        auto walls = [&]( size_t aLane )
        {
            const LANE&              l = lanes[aLane];
            std::vector<const PORT*> walled;

            for( const PORT& q : ports )
            {
                const int qSide = sideIndex( q.padSide );

                if( &q == l.port || q.kind == "rail" || q.via || dives.count( &q ) || qSide < 0
                    || q.pad->GetAttribute() != PAD_ATTRIB::SMD || !q.pad->IsOnLayer( l.layer )
                    || !within( perimeter( qSide, q.pad->GetPosition() ), l.lo, l.hi ) )
                {
                    continue;
                }

                auto it = laneOf.find( &q );

                if( it != laneOf.end() && yielded.count( it->second ) )
                    continue;

                if( it != laneOf.end() && lanes[it->second].turn > 0 && lanes[it->second].cw == l.cw
                    && within( lanes[it->second].lo, l.lo - 1, l.hi + 1 ) && within( lanes[it->second].hi, l.lo - 1, l.hi + 1 ) )
                {
                    continue;
                }

                walled.push_back( &q );
            }

            return walled;
        };

        auto crossAt = [&]( size_t x, size_t y ) -> std::optional<VECTOR2I>
        {
            if( lanes[x].layer != lanes[y].layer )
                return std::nullopt;

            for( size_t i = 0; i + 1 < lanes[x].pts.size(); ++i )
            {
                for( size_t k = 0; k + 1 < lanes[y].pts.size(); ++k )
                {
                    if( OPT_VECTOR2I hit = SEG( lanes[x].pts[i], lanes[x].pts[i + 1] ).Intersect( SEG( lanes[y].pts[k], lanes[y].pts[k + 1] ) ) )
                        return *hit;
                }
            }

            return std::nullopt;
        };

        auto crossesAny = [&]( size_t x )
        {
            for( size_t y = 0; y < lanes.size(); ++y )
            {
                if( y != x && !yielded.count( y ) && crossAt( x, y ) )
                    return true;
            }

            return false;
        };

        auto lengthOf = []( const LANE& l )
        {
            double length = 0;

            for( size_t i = 1; i < l.pts.size(); ++i )
                length += ( l.pts[i] - l.pts[i - 1] ).EuclideanNorm();

            return length;
        };

        // room: the lane laid where it is planned, clear of other copper and on the board
        auto hasRoom = [&]( const LANE& l )
        {
            if( l.blocked )
                return false;

            for( const VECTOR2I& pt : l.pts )
            {
                if( !edges.Contains( pt ) )
                    return false;
            }

            return true;
        };

        // the vias a pin takes to leave on the other layer: down by its fan-out via, up again by another where
        // its far pad is on the pad's layer only
        auto viasOf = [&]( const PORT& p )
        {
            if( p.to < 0 )
                return 1;

            for( PAD* f : b.parts[p.to].fp->Pads() )
            {
                if( f->GetNetCode() == p.code && f->GetAttribute() == PAD_ATTRIB::SMD && f->IsOnLayer( p.pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu ) )
                    return 2;
            }

            return 1;
        };

        auto padName = [&]( const PORT& p ) { return str( a.fp->GetReference() ) + "." + str( p.pad->GetNumber() ); };

        nlohmann::json yields = nlohmann::json::array(), dropped = nlohmann::json::array();

        // a straight way out yields to a ring: the pin dives by its fan-out via
        auto dive = [&]( const PORT& q, const LANE& ring, const char* aWhy )
        {
            dives.insert( &q );

            if( auto it = laneOf.find( &q ); it != laneOf.end() )
                yielded.insert( it->second );

            yields.push_back( { { "pad", str( q.pad->GetNumber() ) }, { "net", q.net }, { "to", padName( *ring.port ) },
                                { "conflict", aWhy }, { "by", "via" }, { "vias", viasOf( q ) }, { "length_mm", 0.0 } } );
        };

        // a lane yields onto the other layer
        auto lift = [&]( size_t x, const std::string& aTo, const char* aWhy )
        {
            yielded.insert( x );
            dives.insert( lanes[x].port );
            yields.push_back( { { "pad", str( lanes[x].port->pad->GetNumber() ) }, { "net", lanes[x].port->net }, { "to", aTo },
                                { "conflict", aWhy }, { "by", "via" }, { "vias", viasOf( *lanes[x].port ) }, { "length_mm", 0.0 } } );
            dropped.push_back( { { "net", lanes[x].port->net }, { "pad", str( lanes[x].port->pad->GetNumber() ) },
                                 { aWhy == std::string( "walls_in" ) ? "walls_in" : "crosses", { aTo } }, { "layer", "other" } } );
        };

        // a ring and a straight way out in each other's way: the ring keeps its way where it has room and the
        // pin has a via spot, else the ring goes onto the other layer
        auto ringOrStraight = [&]( size_t aRing, const PORT& q, const char* aWhy )
        {
            if( hasRoom( lanes[aRing] ) && spotPads.count( q.pad ) )
                dive( q, lanes[aRing], aWhy );
            else
                lift( aRing, padName( q ), aWhy );
        };

        // two lanes in each other's way: the one that yields costs less, by its vias, then its added length;
        // a ring yields deeper (out past the other, clear of every lane, the fan-out's vias and other copper)
        // or onto the other layer
        auto byCost = [&]( size_t x, size_t y, const char* aWhy )
        {
            struct OPTION
            {
                size_t lane;
                int    off;   ///< deeper to; -1: onto the other layer
                int    vias;
                double length;
            };

            std::vector<OPTION> options;
            const size_t        outer = lanes[x].off >= lanes[y].off ? x : y;

            for( size_t z : { outer, outer == x ? y : x } )
            {
                const size_t other = z == x ? y : x;
                LANE&        l = lanes[z];

                if( l.turn == 0 )
                    continue;

                const int    saved = l.off;
                const double before = lengthOf( l );

                for( int64_t off = lanes[other].off + pitch; off <= limit( l ); off += pitch )
                {
                    l.off = (int) off;
                    finish();
                    std::set<std::string> who;

                    if( !crossesAny( z ) && clearsSpots( l ) && !meets( l, who ) && hasRoom( l ) )
                    {
                        options.push_back( { z, (int) off, 0, lengthOf( l ) - before } );
                        break;
                    }
                }

                l.off = saved;
                finish();
            }

            for( size_t z : { outer, outer == x ? y : x } )
                options.push_back( { z, -1, viasOf( *lanes[z].port ), 0.0 } );

            const OPTION* best = &options.front();

            for( const OPTION& o : options )
            {
                if( o.vias < best->vias || ( o.vias == best->vias && o.length < best->length ) )
                    best = &o;
            }

            const size_t other = best->lane == x ? y : x;

            if( best->off < 0 )
            {
                lift( best->lane, padName( *lanes[other].port ), aWhy );
                return;
            }

            LANE& l = lanes[best->lane];
            l.off = best->off;
            l.rank = std::max( l.rank, lanes[other].rank + 1 );
            yields.push_back( { { "pad", str( l.port->pad->GetNumber() ) }, { "net", l.port->net }, { "to", padName( *lanes[other].port ) },
                                { "conflict", aWhy }, { "by", "deeper" }, { "vias", 0 },
                                { "length_mm", std::round( best->length / pcbIUScale.IU_PER_MM * 100 ) / 100 } } );
        };

        finish();

        // each conflict yields one lane or pin, so the conflicts run out within one per lane and pin and
        // one per deepening (a deepened lane crosses none)
        for( size_t pass = 0; pass <= lanes.size() * ( lanes.size() + 1 ) + ports.size(); ++pass )
        {
            bool resolved = false;

            for( size_t k : order )
            {
                if( yielded.count( k ) )
                    continue;

                if( const std::vector<const PORT*> walled = walls( k ); !walled.empty() )
                {
                    auto it = laneOf.find( walled.front() );

                    if( it != laneOf.end() && lanes[it->second].turn > 0 )
                        byCost( k, it->second, "walls_in" );
                    else
                        ringOrStraight( k, *walled.front(), "walls_in" );

                    resolved = true;
                    break;
                }
            }

            for( size_t x = 0; !resolved && x < lanes.size(); ++x )
            {
                for( size_t y = x + 1; !resolved && y < lanes.size(); ++y )
                {
                    if( yielded.count( x ) || yielded.count( y ) || !crossAt( x, y ) )
                        continue;

                    if( ( lanes[x].turn == 0 ) != ( lanes[y].turn == 0 ) )
                        ringOrStraight( lanes[x].turn == 0 ? y : x, *lanes[lanes[x].turn == 0 ? x : y].port, "crosses" );
                    else
                        byCost( x, y, "crosses" );

                    resolved = true;
                }
            }

            finish();

            if( !resolved )
                break;
        }

        nlohmann::json         list = nlohmann::json::array(), crossings = nlohmann::json::array();
        std::vector<int>       crossCount( lanes.size(), 0 );
        int                    shifted = 0, blocked = 0, offBoard = 0;

        for( size_t x = 0; x < lanes.size(); ++x )
        {
            for( size_t y = x + 1; y < lanes.size(); ++y )
            {
                if( yielded.count( x ) || yielded.count( y ) )
                    continue;

                if( std::optional<VECTOR2I> at = crossAt( x, y ) )
                {
                    crossCount[x]++;
                    crossCount[y]++;
                    crossings.push_back( { { "pads", { str( lanes[x].port->pad->GetNumber() ), str( lanes[y].port->pad->GetNumber() ) } },
                                           { "at_mm", mmAt( *at ) } } );
                }
            }
        }

        for( PORT& p : ports )
        {
            if( !dives.count( &p ) )
                continue;

            p.via = true;

            for( auto& [side, rows] : bySide )
            {
                for( nlohmann::json& row : rows )
                {
                    if( row.value( "pad", std::string() ) == str( p.pad->GetNumber() ) )
                    {
                        row["layer"] = "other";
                        row["why"] = "yields to the lanes round the part: its fan-out via to the other layer";
                        row["yields"] = true;
                    }
                }
            }
        }

        for( size_t x = 0; x < lanes.size(); ++x )
        {
            if( yielded.count( x ) )
                continue;

            const LANE&    l = lanes[x];
            nlohmann::json poly = nlohmann::json::array();
            bool           out = false;

            for( const VECTOR2I& pt : l.pts )
            {
                poly.push_back( mmAt( pt ) );
                out = out || !edges.Contains( pt );
            }

            nlohmann::json j = { { "pad", str( l.port->pad->GetNumber() ) }, { "net", l.port->net }, { "side", sides[l.own] },
                                 { "turn", turnName[l.turn] }, { "layer", l.layer == F_Cu ? "F" : "B" },
                                 { "offset_mm", mm( l.off ) }, { "polyline_mm", poly } };

            // the same lane as a corridor (corridor.md): its corners free points, its slot along each
            // side the room from one to the next
            nlohmann::json gates = nlohmann::json::array();

            for( size_t i = 0; i < l.pts.size(); ++i )
            {
                nlohmann::json gate = { { "at_mm", mmAt( l.pts[i] ) } };

                if( i + 1 < l.pts.size() )
                {
                    BOX2I room( l.pts[i], VECTOR2I( 0, 0 ) );
                    room.Merge( l.pts[i + 1] );
                    room.Inflate( pitch / 2 );
                    gate["rooms_mm"] = { { mm( room.GetLeft() ), mm( room.GetTop() ), mm( room.GetRight() ), mm( room.GetBottom() ) } };
                }

                gates.push_back( std::move( gate ) );
            }

            j["corridor"] = { { "name", l.port->net + "@" + str( a.fp->GetReference() ) }, { "nets", { l.port->net } },
                              { "a", str( a.fp->GetReference() ) }, { "points_mm", gates } };

            if( l.turn > 0 )
            {
                j["way"] = l.cw ? "cw" : "ccw";
                j["rank"] = l.rank;
                j["sides"] = l.turn == 2 ? nlohmann::json( { sides[l.own], sides[l.next], sides[l.to] } )
                                         : nlohmann::json( { sides[l.own], sides[l.to] } );

                if( l.off != l.nominal && !l.blocked )
                {
                    j["shift_mm"] = mm( l.off - l.nominal );
                    shifted++;
                }
            }

            if( !l.blockedBy.empty() )
                j[l.blocked ? "blocked_by" : "moved_by"] = l.blockedBy;

            blocked += l.blocked;

            if( out )
            {
                j["off_board"] = true;
                offBoard++;
            }

            if( crossCount[x] )
                j["crossings"] = crossCount[x];

            list.push_back( j );

            for( auto& [side, rows] : bySide )
            {
                for( nlohmann::json& row : rows )
                {
                    if( row.value( "pad", std::string() ) == j["pad"].get<std::string>() )
                        row["lane"] = { { "rank", j.value( "rank", 0 ) }, { "offset_mm", j["offset_mm"] }, { "polyline_mm", poly } };
                }
            }
        }

        lanesOut = { { "pitch_mm", mm( pitch ) }, { "base_mm", mm( base ) }, { "count", lanes.size() - yielded.size() },
                     { "shifted", shifted }, { "blocked", blocked }, { "off_board", offBoard },
                     { "crossings", crossings }, { "dropped", dropped }, { "yields", yields }, { "via_spots", spotsOut }, { "lanes", list } };
    }

    // ports along each side in the order a track leaving it meets them (clockwise)
    nlohmann::json sideList = nlohmann::json::object();
    std::set<int>  all;

    for( const PORT& p : ports )
        all.insert( p.code );

    const std::vector<std::string> cw = orderHere( all );

    for( auto& [side, list] : bySide )
    {
        std::vector<nlohmann::json> rows( list.begin(), list.end() );
        std::stable_sort( rows.begin(), rows.end(),
                          [&]( const nlohmann::json& x, const nlohmann::json& y )
                          {
                              auto ix = std::find( cw.begin(), cw.end(), x["net"].get<std::string>() );
                              auto iy = std::find( cw.begin(), cw.end(), y["net"].get<std::string>() );
                              return ix < iy;
                          } );
        sideList[side] = rows;
    }

    // the sides the signals leave by, to be kept open for them by the support parts
    std::set<std::string> open;

    for( const PORT& p : ports )
    {
        if( p.kind == "signal" )
            open.insert( p.toSide );
    }

    const double ms = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count();

    nlohmann::json rotation = { { "best_quarter_turns_cw", bestTurns }, { "by_turn", byTurn } };
    nlohmann::json out = { { "ref", ref }, { "block", ownBlock >= 0 ? b.blocks[ownBlock].name : std::string() },
                           { "ports", sideList }, { "groups", groups }, { "vias_for_order", viaNets.size() },
                           { "crossings_left", crossingsLeft }, { "turns", counts }, { "keep_open", open } };

    if( !exit.empty() )
    {
        out["exit"] = { { "side", exit }, { "from", exitFrom } };
        rotation["fixed"] = "the board edge it is at sets its turn";
    }

    out["rotation"] = rotation;
    out["ms"] = std::round( ms );

    if( !lanesOut.is_null() )
        out["lanes"] = lanesOut;

    if( !roundChoice.empty() )
        out["round_ways"] = roundChoice;

    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_space( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    SHAPE_POLY_SET current;

    if( !board->GetBoardPolygonOutlines( current, true ) || current.OutlineCount() == 0 )
        return KOPENAPI_RESULT::Error( 409, "the board has no outline" );

    // the outline to evaluate
    SHAPE_POLY_SET outline = current;
    std::string    source = "board";

    if( aArgs.contains( "rect" ) || aArgs.contains( "polygon" ) )
    {
        std::optional<SHAPE_POLY_SET> p = polyArg( aArgs.value( "rect", nlohmann::json() ), aArgs.value( "polygon", nlohmann::json() ) );

        if( !p )
            return KOPENAPI_RESULT::Error( 400, "rect: [x0, y0, x1, y1] mm, or polygon: [[x, y], ...] mm (3 points or more)" );

        outline = *p;
        source = aArgs.contains( "rect" ) ? "rect" : "polygon";
    }
    else if( aArgs.contains( "trim" ) )
    {
        const nlohmann::json& t = aArgs["trim"];

        if( !t.is_object() )
            return KOPENAPI_RESULT::Error( 400, "trim: {top_mm, bottom_mm, left_mm, right_mm}" );

        BOX2I box = current.BBox();
        const int top = iu( t.value( "top_mm", 0.0 ) ), bottom = iu( t.value( "bottom_mm", 0.0 ) );
        const int left = iu( t.value( "left_mm", 0.0 ) ), right = iu( t.value( "right_mm", 0.0 ) );

        if( top < 0 || bottom < 0 || left < 0 || right < 0 || top + bottom >= box.GetHeight() || left + right >= box.GetWidth() )
            return KOPENAPI_RESULT::Error( 400, "trim: non-negative, less than the outline's size" );

        BOX2I keep( VECTOR2I( box.GetLeft() + left, box.GetTop() + top ),
                    VECTOR2I( box.GetWidth() - left - right, box.GetHeight() - top - bottom ) );
        outline.BooleanIntersection( rectPoly( keep ) );
        source = "trim";
    }

    const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
    const int clearance = iu( aArgs.value( "edge_clearance_mm", mm( ds.m_CopperEdgeClearance ) ) );
    SHAPE_POLY_SET usable = outline;

    if( clearance > 0 )
        usable.Deflate( clearance, CORNER_STRATEGY::ROUND_ALL_CORNERS, ds.m_MaxError );

    // keep-outs: without max_height_mm nothing may go there; with it, only lower parts
    struct KEEPOUT
    {
        std::string            name;
        SHAPE_POLY_SET         area;
        std::optional<double>  maxHeight;
        std::string            side;
    };

    std::vector<KEEPOUT> keepouts;

    for( const nlohmann::json& k : aArgs.value( "keepouts", nlohmann::json::array() ) )
    {
        std::optional<SHAPE_POLY_SET> p = k.is_object() ? polyArg( k.value( "rect", nlohmann::json() ), k.value( "polygon", nlohmann::json() ) )
                                                        : std::nullopt;

        if( !p )
            return KOPENAPI_RESULT::Error( 400, "keepouts: [{name, rect | polygon, side, max_height_mm}]" );

        KEEPOUT ko{ k.value( "name", "keepout " + std::to_string( keepouts.size() + 1 ) ), *p, std::nullopt,
                    k.value( "side", std::string( "front" ) ) };

        if( k.contains( "max_height_mm" ) && k["max_height_mm"].is_number() )
            ko.maxHeight = k["max_height_mm"].get<double>();

        if( ko.side != "front" && ko.side != "back" && ko.side != "both" )
            return KOPENAPI_RESULT::Error( 400, "keepouts: side front, back or both" );

        keepouts.push_back( std::move( ko ) );
    }

    GROUP_OPTIONS opt;

    if( std::optional<std::string> bad = groupOptions( aArgs, opt ) )
        return KOPENAPI_RESULT::Error( 400, *bad );

    std::string  error;
    BOARD_BLOCKS b = buildBlocks( aCtx, board, opt, aArgs.value( "blocks_given", nlohmann::json() ), error );

    if( !error.empty() )
        return KOPENAPI_RESULT::Error( 404, error );

    const bool                         needHeights = std::any_of( keepouts.begin(), keepouts.end(),
                                                                  []( const KEEPOUT& k ) { return k.maxHeight.has_value(); } );
    std::map<const FOOTPRINT*, double> heights;

    if( needHeights )
        heights = KopenapiFootprintHeights( board );

    // which parts stay: the part of each courtyard that lies on the present board must lie in the
    // new outline (an edge connector hanging over the old edge may hang over the new one)
    std::vector<int> stay, misfit, needsEdge;
    std::vector<char> crossingNow( b.parts.size(), 0 );

    for( size_t i = 0; i < b.parts.size(); ++i )
    {
        const PART& p = b.parts[i];

        if( p.mechanical && p.fp->Pads().empty() )
            continue;   // logos, drawings

        if( source == "board" )
        {
            stay.push_back( (int) i );
            continue;
        }

        SHAPE_POLY_SET cy = p.courtyard;
        const double   cyArea = cy.Area();
        SHAPE_POLY_SET onBoard = p.courtyard;
        onBoard.BooleanIntersection( current );
        SHAPE_POLY_SET beyond = p.courtyard;
        beyond.BooleanSubtract( current );
        crossingNow[i] = beyond.Area() > 0.01 * cyArea;

        SHAPE_POLY_SET out = onBoard;
        out.BooleanSubtract( outline );

        if( out.Area() <= std::max( 0.01 * cyArea, double( iu( 0.05 ) ) * iu( 0.05 ) ) )
            stay.push_back( (int) i );
        else if( crossingNow[i] || ( p.connector && !edgeSide( p.fp, p.courtyard.BBox(), current.BBox(), iu( 1.0 ) ).empty() ) )
            needsEdge.push_back( (int) i );
        else
            misfit.push_back( (int) i );
    }

    // grid step: 0.25 mm, coarser for big boards (at most 4 M cells)
    const BOX2I ub = usable.BBox();
    double      step = std::max( 0.05, aArgs.value( "grid_mm", 0.25 ) );

    while( ( ub.GetWidth() / double( iu( step ) ) ) * ( ub.GetHeight() / double( iu( step ) ) ) > 4e6 )
        step *= 2;

    const double freeMin = std::max( step, aArgs.value( "free_min_mm", 3.0 ) );
    const int    freeCount = std::clamp( aArgs.value( "free_count", 10 ), 0, 100 );

    nlohmann::json sides = nlohmann::json::object();
    nlohmann::json show = nlohmann::json::array();
    const double   fill = std::clamp( aArgs.value( "show_fill", 0.22 ), 0.0, 1.0 );
    std::map<std::string, std::vector<nlohmann::json>> rectsBySide;

    for( const bool back : { false, true } )
    {
        const std::string side = back ? "back" : "front";
        SHAPE_POLY_SET    room = usable;

        for( const KEEPOUT& k : keepouts )
        {
            if( !k.maxHeight && ( k.side == "both" || k.side == side ) )
                room.BooleanSubtract( k.area );
        }

        // occupied: parts on this side, and THT pads of the other side's parts. Every part standing
        // there now counts, also one that crosses a tried outline (it occupies its cells until moved);
        // staying parts alone give the room left once the misfits are re-placed.
        auto occupiedBy = [&]( const std::vector<int>& aParts, SHAPE_POLY_SET& aOut )
        {
            for( int i : aParts )
            {
                const PART& p = b.parts[i];

                if( p.back == back )
                {
                    aOut.Append( p.courtyard );
                }
                else
                {
                    for( PAD* pad : p.fp->Pads() )
                    {
                        if( pad->HasHole() )
                            pad->TransformShapeToPolygon( aOut, back ? B_Cu : F_Cu, 0, ds.m_MaxError, ERROR_OUTSIDE );
                    }
                }
            }
        };

        SHAPE_POLY_SET occupied, occupiedStaying;
        occupiedBy( stay, occupiedStaying );
        occupied = occupiedStaying;
        occupiedBy( misfit, occupied );
        occupiedBy( needsEdge, occupied );

        occupied.Simplify();
        occupied.BooleanIntersection( room );
        occupiedStaying.Simplify();
        occupiedStaying.BooleanIntersection( room );
        const double roomArea = room.Area();
        const double occupiedArea = occupied.Area();
        const double stayingArea = occupiedStaying.Area();

        FREE_GRID                   g = rasterise( room, occupied, iu( step ) );
        std::vector<FREE_RECT> rects = freeRects( g, freeCount, int( std::ceil( freeMin / step - 1e-9 ) ) );
        nlohmann::json         rectList = nlohmann::json::array();

        for( const FREE_RECT& r : rects )
        {
            const BOX2I box( VECTOR2I( g.origin.x + r.x0 * g.step, g.origin.y + r.y0 * g.step ),
                             VECTOR2I( ( r.x1 - r.x0 ) * g.step, ( r.y1 - r.y0 ) * g.step ) );
            nlohmann::json j = { { "box_mm", boxJson( box ) }, { "w_mm", mm( box.GetWidth() ) }, { "h_mm", mm( box.GetHeight() ) },
                                 { "area_mm2", mm2( double( box.GetWidth() ) * box.GetHeight() ) } };
            rectList.push_back( j );
            rectsBySide[side].push_back( j );

            if( !back )
            {
                show.push_back( { { "kind", "region" }, { "area", j["box_mm"] }, { "color", "#3CC85A" }, { "group", "space" },
                                  { "fill_opacity", fill }, { "width_px", 3 },
                                  { "text", "free " + std::to_string( int( std::round( mm( box.GetWidth() ) ) ) ) + " x "
                                                    + std::to_string( int( std::round( mm( box.GetHeight() ) ) ) ) + " mm" } } );
            }
        }

        sides[side] = { { "usable_mm2", mm2( roomArea ) },
                        { "occupied_mm2", mm2( occupiedArea ) },
                        { "free_mm2", mm2( roomArea - occupiedArea ) },
                        { "fill", roomArea > 0 ? std::round( occupiedArea / roomArea * 100 ) / 100 : 0.0 },
                        { "free_rects", rectList } };

        if( source != "board" )
        {
            sides[side]["occupied_staying_mm2"] = mm2( stayingArea );
            sides[side]["free_when_moved_mm2"] = mm2( roomArea - stayingArea );
        }
    }

    nlohmann::json answer = { { "outline", { { "source", source },
                                             { "area_mm2", mm2( outline.Area() ) },
                                             { "usable_mm2", mm2( usable.Area() ) },
                                             { "bbox_mm", boxJson( outline.BBox() ) },
                                             { "edge_clearance_mm", mm( clearance ) } } },
                              { "grid_mm", step },
                              { "sides", sides } };

    // what does not fit, by block
    if( source != "board" )
    {
        std::map<int, std::vector<int>> byBlock;

        // -1: parts of no block, -2: mechanical
        for( int i : misfit )
            byBlock[b.parts[i].mechanical ? -2 : b.partBlock[i]].push_back( i );

        nlohmann::json misfits = nlohmann::json::array();
        double         need[2] = { 0, 0 };

        for( auto& [blk, parts] : byBlock )
        {
            const double front = unionOf( b, parts, false ).Area();
            const double back = unionOf( b, parts, true ).Area();
            need[0] += front;
            need[1] += back;

            BOX2I bbox = b.parts[parts[0]].courtyard.BBox();

            for( int i : parts )
                bbox.Merge( b.parts[i].courtyard.BBox() );

            nlohmann::json j = { { "block", blk >= 0 ? b.blocks[blk].name : blk == -2 ? "mechanical" : "(no block)" },
                                 { "parts", refList( b, parts ) },
                                 { "area_mm2", { { "front", mm2( front ) }, { "back", mm2( back ) } } },
                                 { "bbox_mm", boxJson( bbox ) } };

            if( blk >= 0 )
                j["block_parts"] = b.blocks[blk].members.size();

            // free rectangles holding the block as it is laid out now (either way round)
            nlohmann::json fits = nlohmann::json::array();
            const double   bw = mm( bbox.GetWidth() ), bh = mm( bbox.GetHeight() );

            for( const nlohmann::json& r : rectsBySide[back > front ? "back" : "front"] )
            {
                const double w = r["w_mm"], h = r["h_mm"];

                if( ( w >= bw && h >= bh ) || ( w >= bh && h >= bw ) )
                    fits.push_back( r["box_mm"] );
            }

            j["fits_as_laid_in"] = fits;
            misfits.push_back( j );

            show.push_back( { { "kind", "region" }, { "area", j["bbox_mm"] }, { "color", "#E71D36" }, { "group", "space" },
                              { "fill_opacity", 0.0 }, { "width_px", 4 },
                              { "text", "out: " + j["block"].get<std::string>() + " (" + std::to_string( parts.size() ) + ")" } } );

            for( int i : parts )
            {
                show.push_back( { { "kind", "highlight" }, { "target", b.parts[i].ref }, { "color", "#E71D36" }, { "group", "space" },
                                  { "fill_opacity", 0.45 }, { "width_px", 2 } } );
            }
        }

        nlohmann::json edge = nlohmann::json::array();

        for( int i : needsEdge )
        {
            const PART&    p = b.parts[i];
            const BOX2I    cb = p.courtyard.BBox();
            nlohmann::json j = { { "ref", p.ref }, { "value", str( p.fp->GetValue() ) }, { "bbox_mm", boxJson( cb ) },
                                 { "block", b.partBlock[i] >= 0 ? b.blocks[b.partBlock[i]].name : "" } };
            const std::string side = edgeSide( p.fp, cb, current.BBox(), iu( 1.0 ) );

            if( !side.empty() )
            {
                j["side"] = side;
                j["along_mm"] = ( side == "left" || side == "right" ) ? mm( cb.GetHeight() ) : mm( cb.GetWidth() );
            }

            edge.push_back( j );
            show.push_back( { { "kind", "highlight" }, { "target", p.ref }, { "color", "#FF9F1C" }, { "group", "space" },
                              { "fill_opacity", 0.45 }, { "width_px", 4 },
                              { "text", p.ref + ( side.empty() ? std::string( " to the new edge" ) : " to the " + side + " edge" ) } } );
        }

        nlohmann::json fit = nlohmann::json::object();

        for( int s = 0; s < 2; ++s )
        {
            const std::string side = s ? "back" : "front";
            const double      freeArea = sides[side]["free_when_moved_mm2"].get<double>();
            fit[side] = { { "to_place_mm2", mm2( need[s] ) },
                          { "free_mm2", freeArea },
                          { "free_after_mm2", std::round( ( freeArea - mm2( need[s] ) ) * 10 ) / 10 },
                          { "fill_after", sides[side]["usable_mm2"].get<double>() > 0
                                                  ? std::round( ( sides[side]["occupied_staying_mm2"].get<double>() + mm2( need[s] ) )
                                                                / sides[side]["usable_mm2"].get<double>() * 100 ) / 100
                                                  : 0.0 } };
        }

        answer["misfits"] = misfits;
        answer["misfit_parts"] = misfit.size();
        answer["needs_edge"] = edge;
        answer["fit"] = fit;
        answer["staying_parts"] = stay.size();
    }

    // edge connectors along each side of the outline's box (displaced ones on the side they left)
    {
        const BOX2I    ob = outline.BBox();
        nlohmann::json edges = nlohmann::json::object();
        std::map<std::string, std::pair<double, nlohmann::json>> per;

        for( const char* s : { "left", "right", "top", "bottom" } )
            per[s] = { 0.0, nlohmann::json::array() };

        std::vector<int> conns = stay;
        conns.insert( conns.end(), needsEdge.begin(), needsEdge.end() );

        for( int i : conns )
        {
            const PART& p = b.parts[i];
            const bool  displaced = std::find( needsEdge.begin(), needsEdge.end(), i ) != needsEdge.end();

            if( !p.connector )
                continue;

            const BOX2I       cb = p.courtyard.BBox();
            const std::string side = edgeSide( p.fp, cb, displaced ? current.BBox() : ob, iu( 1.0 ) );

            if( side.empty() )
                continue;

            const double along = ( side == "left" || side == "right" ) ? mm( cb.GetHeight() ) : mm( cb.GetWidth() );
            per[side].first += along;
            per[side].second.push_back( { { "ref", p.ref }, { "along_mm", along }, { "displaced", displaced } } );
        }

        for( auto& [side, v] : per )
        {
            const double len = ( side == "left" || side == "right" ) ? mm( ob.GetHeight() ) : mm( ob.GetWidth() );
            edges[side] = { { "length_mm", len },
                            { "connectors_mm", std::round( v.first * 100 ) / 100 },
                            { "connectors", v.second } };
        }

        answer["edges"] = edges;
    }

    // keep-outs that allow only low parts
    if( needHeights )
    {
        nlohmann::json violations = nlohmann::json::array();

        for( const KEEPOUT& k : keepouts )
        {
            if( !k.maxHeight )
                continue;

            for( size_t i = 0; i < b.parts.size(); ++i )
            {
                const PART& p = b.parts[i];

                if( k.side != "both" && ( k.side == "back" ) != p.back )
                    continue;

                SHAPE_POLY_SET in = p.courtyard;
                in.BooleanIntersection( k.area );

                if( in.Area() <= 0 )
                    continue;

                auto it = heights.find( p.fp );

                if( it == heights.end() )
                    violations.push_back( { { "keepout", k.name }, { "ref", p.ref }, { "height_mm", nullptr }, { "why", "no 3D model to measure" } } );
                else if( it->second > *k.maxHeight )
                    violations.push_back( { { "keepout", k.name }, { "ref", p.ref }, { "height_mm", it->second }, { "max_height_mm", *k.maxHeight } } );
            }
        }

        answer["keepout_violations"] = violations;
    }

    if( aArgs.value( "show", false ) )
    {
        nlohmann::json pts = nlohmann::json::array();
        const BOX2I    ob = outline.BBox();
        char              buf[64];
        std::snprintf( buf, sizeof( buf ), "%.1f x %.1f mm", mm( ob.GetWidth() ), mm( ob.GetHeight() ) );
        const std::string size = buf;

        // a tried outline darkens what falls outside it
        show.push_back( { { "kind", source == "board" ? "region" : "spotlight" }, { "area", boxJson( ob ) }, { "color", "#FFFFFF" },
                          { "group", "space" }, { "width_px", 4 }, { "fill_opacity", source == "board" ? 0.0 : 0.6 },
                          { "text", source == "board" ? "board " + size : "outline tried (" + source + ") " + size } } );

        for( const KEEPOUT& k : keepouts )
            show.push_back( { { "kind", "region" }, { "area", boxJson( k.area.BBox() ) }, { "color", "#00BBF9" }, { "group", "space" },
                              { "fill_opacity", fill }, { "width_px", 3 }, { "text", k.name } } );

        overlay( aCtx, "space", show );
    }

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_blocks",
                   "Functional blocks of the placed board (groups of components, clusters, modules): each IC / "
                   "connector with the small parts reached through its signal nets (and a supply of its own that only "
                   "a few small parts share: a relay coil and its driver) and its decoupling (rail "
                   "capacitors given to the nearest supply pad), or one block per schematic sheet; per block "
                   "parts, courtyard area as a union (front / back), bbox and how full it is, centroid, tallest "
                   "part (3D model height), rails, links to other blocks (shared nets), panel edge of connectors, "
                   "pcb_place_group arguments; board area and parts area; mechanical and unassigned parts; or the "
                   "same figures for given blocks. show: coloured regions in the GUI",
                   R"json({"type":"object","properties":{
                        "by":{"type":"string","enum":["auto","sheet"],"default":"auto","description":"auto: around ICs and connectors; sheet: one block per schematic sheet"},
                        "blocks":{"type":"array","items":{"type":"object","properties":{"name":{"type":"string"},"parts":{"type":"array","items":{"type":"string"},"description":"refs or globs"}}},"description":"evaluate these groups instead"},
                        "max_part_pins":{"type":"integer","default":3,"description":"parts with more pads are anchors, not members"},
                        "depth":{"type":"integer","default":2,"minimum":1,"maximum":6,"description":"steps through small parts from an anchor"},
                        "max_fanout":{"type":"integer","default":30,"description":"do not grow through nets with more pads; 0 = no limit"},
                        "heights":{"type":"boolean","default":true,"description":"tallest part per block from the 3D models (cached)"},
                        "detail":{"type":"boolean","default":false,"description":"parts as {ref, by: anchor / signal / rail / given}"},
                        "show":{"type":"boolean","default":false,"description":"GUI: overlay group 'blocks'"},
                        "show_fill":{"type":"number","default":0.22,"minimum":0,"maximum":1,"description":"GUI: fill opacity of the shown regions"}}})json"_json,
                   false, h_pcb_blocks, 120 );

KOPENAPI_REGISTER( "pcb_layout_graph",
                   "The board as blocks and the links between them, the first level of laying it out: blocks (an "
                   "IC or connector with its small parts, as pcb_blocks finds them), clusters (blocks by schematic "
                   "sheet), buses (pcb_net_roles bundles of one name that share an end part, joined: the MB bus of "
                   "three bundles from one MCU to four buffers is one bus of 12 nets) with their trunk (the end in "
                   "most bundles) and per end block the part the nets leave from, its side (west / east / north / "
                   "south, board frame), exit (the side the bus leaves the part by: a connector at the board edge away "
                   "from that edge, else as pcb_fanout_ports plans those pads: their side, pads inside a grid "
                   "towards the far end) and exit_from, the nets clockwise round the part and crossings_to_trunk (pairs of nets "
                   "whose order is not the mirror of the trunk's: tracks that cross between them as placed), "
                   "net_source per net (project / schematic: a bus set there; name: the name stem a topology bundle "
                   "is named by; topology: by its end parts only); edges: "
                   "every pair of blocks sharing signal nets, widest first, with the buses between them and the "
                   "nets that go alone (single_nets cut to list_limit). Read only",
                   R"json({"type":"object","properties":{
                        "by":{"type":"string","enum":["auto","sheet"],"default":"auto","description":"blocks as pcb_blocks: auto around ICs and connectors, sheet one per schematic sheet"},
                        "blocks":{"type":"array","items":{"type":"object","properties":{"name":{"type":"string"},"parts":{"type":"array","items":{"type":"string"}}}},"description":"these blocks instead"},
                        "max_part_pins":{"type":"integer","default":3},
                        "depth":{"type":"integer","default":2,"minimum":1,"maximum":6},
                        "max_fanout":{"type":"integer","default":30},
                        "min_bus":{"type":"integer","default":3,"minimum":2,"description":"lines a bundle needs (pcb_net_roles min_bundle)"},
                        "list_limit":{"type":"integer","default":16,"description":"single nets listed per edge"},"full":{"type":"boolean","default":false,"description":"everything: every item (no page limit) and every entry of every list in the answer (no list_limit); large on a big design"}}})json"_json,
                   false, h_pcb_layout_graph, 120 );

KOPENAPI_REGISTER( "pcb_fanout_ports",
                   "The second level of laying a board out: where each pin of a large package must leave it, "
                   "before anything is placed round it. Per pad a port: the side it is on, the part its net goes "
                   "to (the nearest in another block, else in its own: local) and the side that part lies, the "
                   "turn between them (straight, corner, round_the_part; round the part also round: cw / ccw in the board view "
                   "and sides: the sides it passes, by the side the far pin lies off the centre line, else the pin's "
                   "half of its side), and the layer: pins whose order "
                   "matches their far end's stay on the pad's layer, the rest of a group (two or more nets to "
                   "one part; the longest run in order kept) change layer by a via; rails go to their plane. "
                   "groups: per far part, side of the package and side and layer the nets arrive on (F, B, THT), the order "
                   "at both ends, crossings, the nets kept and moved (at a THT end the pin itself changes layer: "
                   "no via), "
                   "crossings_left (pairs that still cross after the vias: another side or the part mirrored, "
                   "L2 / L3). rotation: the quarter turns of the package that make the turns cheapest "
                   "(straight 0, corner 1, round the part 3); a turn keeps the order round the part, so it "
                   "never changes crossings. keep_open: the sides signals leave by. An edge connector (gold fingers "
                   "too) is left away from the board edge it is at: every port leaves by that side (exit: side and "
                   "why, as pcb_layout_graph's bus ends; pad_side: the side its pad is on), its rotation fixed. "
                   "lanes (asked): the ring each signal "
                   "on its pad's layer is to run in round the package, planned before any route: per lane the pad, "
                   "net, side, turn, way (cw / ccw), sides passed, rank (0 innermost: the pin nearest its corner), "
                   "offset_mm past the courtyard, polyline_mm (pad end, out to the offset, the corners, out past the "
                   "last to one front for the lanes leaving there, a pitch past the outermost and a clearance past the ways out of that side's pins as pcb_route's escape fronts have them, within the room on that side), shift_mm and moved_by (other parts' pads, other nets' tracks or vias in its way: "
                   "moved out a pitch at a time, the lanes outside it with it), blocked_by (still in the way where the "
                   "room ends, or room: the room round the package as pcb_floorplan's boxes give it, courtyards a pitch "
                   "apart one box, from the courtyard to the nearest other box or the board's edge), clear of the vias the package's fan-out lays by its pads (pcb_fanout dry run: the rails' and moved pins' as "
                   "planned, every other pin not turning where its via would go; via_spots: pad, at_mm) by a via's half, a clearance and its "
                   "half width along its runs past its pad, off_board, crossings with other lanes on its layer; straight ones run out past the "
                   "lanes going by their pin. The pins of one side going round the part to one far part all go one way round: the way whose "
                   "modelled paths (out to their offsets, round both corners, straight on to their far pins) cross fewer of each other and of "
                   "the package's straight ways out on their layer, then the shorter in total (round_ways: per side and far part both ways' "
                   "crossings and length_mm, the one taken). yields: where "
                   "two ways are in each other's way (a turning lane running in front of a pin whose way out is straight, or two lanes "
                   "crossing on one layer) one yields: a straight way out yields to a ring (the pin dives by its fan-out via, its port "
                   "layer other, yields true: its via at its spot in pcb_fanout ports via_spots, which the lanes keep clear of) unless the ring has no room (blocked, off the board) or the pin no via spot; of two lanes the one "
                   "whose yield costs fewer vias, then less added length, deeper (out past the other, clear of all) or onto the other "
                   "layer (pad, net, to, conflict walls_in or crosses, by deeper or via, vias, length_mm). dropped: the lanes gone onto "
                   "the other layer (walls_in or crosses, layer other). Also each port's lane (rank, offset_mm, polyline_mm) and each lane as a corridor (corridor.md: its corners the gates, its slot the rooms). pitch_mm, "
                   "base_mm, counts and the crossings (pads, at_mm). Read only",

                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string","description":"the package"},
                        "by":{"type":"string","enum":["auto","sheet"],"default":"auto"},
                        "max_part_pins":{"type":"integer","default":3},
                        "depth":{"type":"integer","default":2,"minimum":1,"maximum":6},
                        "max_fanout":{"type":"integer","default":30},
                        "lanes":{"type":"boolean","default":false,"description":"also the planned ring lanes (no routing)"}}})json"_json,
                   false, h_pcb_fanout_ports, 120 );

KOPENAPI_REGISTER( "pcb_space",
                   "Room on the board, and trying another outline (smaller / lower card, trim, resize): usable "
                   "area, occupied (union of courtyards, THT pads on the other side), free area and fill per "
                   "side, the largest free rectangles; for an outline given as rect, polygon or trim of the "
                   "present one: the parts that no longer fit, grouped by functional block (pcb_blocks) with "
                   "their area and the free rectangles they fit in, edge connectors that must move to the new "
                   "edge, area to re-place vs free area; connector length along each edge; keep-outs (nothing, "
                   "or only parts up to max_height_mm, e.g. under a daughterboard) with the parts that break "
                   "them. show: in the GUI, outside a tried outline darkened, misfit blocks framed red, edge connectors orange, free rectangles green with their size",
                   R"json({"type":"object","properties":{
                        "rect":{"type":"array","items":{"type":"number"},"description":"outline to try: [x0, y0, x1, y1] mm"},
                        "polygon":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"outline to try: [[x, y], ...] mm"},
                        "trim":{"type":"object","properties":{"top_mm":{"type":"number"},"bottom_mm":{"type":"number"},"left_mm":{"type":"number"},"right_mm":{"type":"number"}},"description":"cut this much off the present outline's sides"},
                        "edge_clearance_mm":{"type":"number","description":"default: the board's copper-to-edge clearance"},
                        "keepouts":{"type":"array","items":{"type":"object","properties":{
                            "name":{"type":"string"},
                            "rect":{"type":"array","items":{"type":"number"}},
                            "polygon":{"type":"array","items":{"type":"array","items":{"type":"number"}}},
                            "side":{"type":"string","enum":["front","back","both"],"default":"front"},
                            "max_height_mm":{"type":"number","description":"only parts up to this height; absent = nothing"}}}},
                        "free_min_mm":{"type":"number","default":3,"description":"smallest side of a free rectangle"},
                        "free_count":{"type":"integer","default":10,"maximum":100},
                        "grid_mm":{"type":"number","default":0.25,"description":"free-space grid (coarser on big boards)"},
                        "by":{"type":"string","enum":["auto","sheet"],"default":"auto","description":"blocks for grouping the misfits"},
                        "blocks_given":{"type":"array","items":{"type":"object"},"description":"own blocks [{name, parts}] for grouping"},
                        "show":{"type":"boolean","default":false,"description":"GUI: overlay group 'space'"},
                        "show_fill":{"type":"number","default":0.22,"minimum":0,"maximum":1,"description":"GUI: fill opacity of the shown regions"}}})json"_json,
                   false, h_pcb_space, 120 );
