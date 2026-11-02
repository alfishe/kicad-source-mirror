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
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <netinfo.h>
#include <pad.h>
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
                if( !signalNet( n ) )
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

    static const char* turnName[3] = { "straight", "corner", "round_the_part" };
    std::map<std::string, nlohmann::json> bySide;
    std::map<std::string, int>            counts;

    for( PORT& p : ports )
    {
        if( viaNets.count( p.code ) )
        {
            p.via = true;
            p.why = "out of order with its far end: a via to the other layer";
        }

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
                j["turn"] = turnName[turnOf( own, to )];
                counts[turnName[turnOf( own, to )]] += p.kind == "signal";

                // round the part: by the side the far pin lies off the line through the centre, else by the
                // pin's half of its side
                if( turnOf( own, to ) == 2 )
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

                    const bool     across = own == 0 || own == 2;
                    const VECTOR2I off = farPin - centre;
                    const VECTOR2I at = p.pad->GetPosition() - centre;
                    const int      tol = ( across ? extent.GetHeight() : extent.GetWidth() ) / 4;
                    const int      lean = across ? ( std::abs( off.y ) > tol ? off.y : at.y ) : ( std::abs( off.x ) > tol ? off.x : at.x );
                    const int      next = across ? ( lean <= 0 ? 1 : 3 ) : ( lean <= 0 ? 0 : 2 );
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
                   "connector with the small parts reached through its signal nets and its decoupling (rail "
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
                   "why, as pcb_layout_graph's bus ends; pad_side: the side its pad is on), its rotation fixed. Read only",
                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string","description":"the package"},
                        "by":{"type":"string","enum":["auto","sheet"],"default":"auto"},
                        "max_part_pins":{"type":"integer","default":3},
                        "depth":{"type":"integer","default":2,"minimum":1,"maximum":6},
                        "max_fanout":{"type":"integer","default":30}}})json"_json,
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
