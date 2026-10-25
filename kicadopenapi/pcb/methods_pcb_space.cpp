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


std::string refPrefix( const std::string& aRef )
{
    size_t n = 0;

    while( n < aRef.size() && std::isalpha( (unsigned char) aRef[n] ) )
        ++n;

    return aRef.substr( 0, n );
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
        const std::string prefix = refPrefix( p.ref );
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
