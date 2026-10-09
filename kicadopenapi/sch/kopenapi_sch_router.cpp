#include "kopenapi_sch_router.h"

#include <sch_bus_entry.h>
#include <sch_junction.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_no_connect.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>

#include <algorithm>
#include <cstdlib>

namespace kopenapi_sch
{

namespace
{

bool onSegment( const VECTOR2I& aP, const VECTOR2I& aA, const VECTOR2I& aB )
{
    const int64_t cross = int64_t( aB.x - aA.x ) * ( aP.y - aA.y ) - int64_t( aB.y - aA.y ) * ( aP.x - aA.x );

    return cross == 0 && aP.x >= std::min( aA.x, aB.x ) && aP.x <= std::max( aA.x, aB.x )
           && aP.y >= std::min( aA.y, aB.y ) && aP.y <= std::max( aA.y, aB.y );
}


/// Two axis-aligned segments run along each other for more than a point
bool overlapAlong( const VECTOR2I& aA, const VECTOR2I& aB, const VECTOR2I& aC, const VECTOR2I& aD )
{
    if( aA.y == aB.y && aC.y == aD.y && aA.y == aC.y )
    {
        const int lo = std::max( std::min( aA.x, aB.x ), std::min( aC.x, aD.x ) );
        const int hi = std::min( std::max( aA.x, aB.x ), std::max( aC.x, aD.x ) );
        return hi > lo;
    }

    if( aA.x == aB.x && aC.x == aD.x && aA.x == aC.x )
    {
        const int lo = std::max( std::min( aA.y, aB.y ), std::min( aC.y, aD.y ) );
        const int hi = std::min( std::max( aA.y, aB.y ), std::max( aC.y, aD.y ) );
        return hi > lo;
    }

    return false;
}


/// An axis-aligned segment enters the open interior of a box
bool crossesBody( const VECTOR2I& aA, const VECTOR2I& aB, const BOX2I& aBox )
{
    const int left = aBox.GetLeft(), right = aBox.GetRight(), top = aBox.GetTop(), bottom = aBox.GetBottom();

    if( aA.y == aB.y )
    {
        if( aA.y <= top || aA.y >= bottom )
            return false;

        return std::max( aA.x, aB.x ) > left && std::min( aA.x, aB.x ) < right;
    }

    if( aA.x <= left || aA.x >= right )
        return false;

    return std::max( aA.y, aB.y ) > top && std::min( aA.y, aB.y ) < bottom;
}


int length( const std::vector<VECTOR2I>& aPolyline )
{
    int total = 0;

    for( size_t i = 0; i + 1 < aPolyline.size(); ++i )
        total += std::abs( aPolyline[i + 1].x - aPolyline[i].x ) + std::abs( aPolyline[i + 1].y - aPolyline[i].y );

    return total;
}

} // namespace


WIRE_ROUTER::WIRE_ROUTER( SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath, int aGrid ) :
        m_grid( aGrid )
{
    for( SCH_ITEM* item : aScreen->Items() )
    {
        switch( item->Type() )
        {
        case SCH_LINE_T:
        {
            SCH_LINE* line = static_cast<SCH_LINE*>( item );

            if( line->IsWire() || line->IsBus() )
                m_wires.emplace_back( line->GetStartPoint(), line->GetEndPoint() );

            break;
        }
        case SCH_SYMBOL_T:
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            for( SCH_PIN* pin : symbol->GetPins( &aPath ) )
                m_points.push_back( pin->GetPosition() );

            m_bodies.push_back( symbol->GetBodyBoundingBox() );
            break;
        }
        case SCH_SHEET_T:
            for( SCH_SHEET_PIN* pin : static_cast<SCH_SHEET*>( item )->GetPins() )
                m_points.push_back( pin->GetPosition() );

            m_bodies.push_back( item->GetBoundingBox() );
            break;

        case SCH_JUNCTION_T:
        case SCH_NO_CONNECT_T:
        case SCH_LABEL_T:
        case SCH_GLOBAL_LABEL_T:
        case SCH_HIER_LABEL_T:
        case SCH_DIRECTIVE_LABEL_T:
            m_points.push_back( item->GetPosition() );
            break;

        case SCH_BUS_WIRE_ENTRY_T:
        case SCH_BUS_BUS_ENTRY_T:
            m_points.push_back( item->GetPosition() );
            m_points.push_back( static_cast<SCH_BUS_ENTRY_BASE*>( item )->GetEnd() );
            break;

        default:
            break;
        }
    }
}


void WIRE_ROUTER::AddWire( const VECTOR2I& aStart, const VECTOR2I& aEnd )
{
    m_wires.emplace_back( aStart, aEnd );
}


bool WIRE_ROUTER::clear( const std::vector<VECTOR2I>& aPolyline, const VECTOR2I& aFrom, const VECTOR2I& aTo ) const
{
    auto isEnd = [&]( const VECTOR2I& p ) { return p == aFrom || p == aTo; };

    for( size_t i = 0; i + 1 < aPolyline.size(); ++i )
    {
        const VECTOR2I& p = aPolyline[i];
        const VECTOR2I& q = aPolyline[i + 1];

        if( p == q )
            return false;

        for( const auto& [s, e] : m_wires )
        {
            if( overlapAlong( p, q, s, e ) )
                return false;

            // another wire's end on this segment, or a corner of ours on another wire
            for( const VECTOR2I& end : { s, e } )
            {
                if( onSegment( end, p, q ) && !isEnd( end ) )
                    return false;
            }

            for( const VECTOR2I& mine : { p, q } )
            {
                if( !isEnd( mine ) && onSegment( mine, s, e ) )
                    return false;
            }
        }

        for( const VECTOR2I& point : m_points )
        {
            if( !isEnd( point ) && onSegment( point, p, q ) )
                return false;
        }

        for( const BOX2I& body : m_bodies )
        {
            if( crossesBody( p, q, body ) )
                return false;
        }
    }

    return true;
}


std::optional<std::vector<VECTOR2I>> WIRE_ROUTER::Route( const VECTOR2I& aFrom, const VECTOR2I& aTo,
                                                         const std::string& aPrefer ) const
{
    std::vector<std::vector<VECTOR2I>> candidates;

    if( aFrom.x == aTo.x || aFrom.y == aTo.y )
        candidates.push_back( { aFrom, aTo } );

    const std::vector<VECTOR2I> hv = { aFrom, VECTOR2I( aTo.x, aFrom.y ), aTo };
    const std::vector<VECTOR2I> vh = { aFrom, VECTOR2I( aFrom.x, aTo.y ), aTo };

    if( aFrom.x != aTo.x && aFrom.y != aTo.y )
    {
        candidates.push_back( aPrefer == "vh" ? vh : hv );
        candidates.push_back( aPrefer == "vh" ? hv : vh );
    }

    // Z shapes: a vertical (or horizontal) middle segment shifted along the grid, nearest the
    // middle first, up to 40 steps beyond the two ends
    constexpr int STEPS = 40;
    std::vector<int> xs, ys;

    for( int x = std::min( aFrom.x, aTo.x ) - STEPS * m_grid; x <= std::max( aFrom.x, aTo.x ) + STEPS * m_grid; x += m_grid )
        xs.push_back( x );

    for( int y = std::min( aFrom.y, aTo.y ) - STEPS * m_grid; y <= std::max( aFrom.y, aTo.y ) + STEPS * m_grid; y += m_grid )
        ys.push_back( y );

    const int midX = ( aFrom.x + aTo.x ) / 2, midY = ( aFrom.y + aTo.y ) / 2;
    std::sort( xs.begin(), xs.end(), [&]( int a, int b ) { return std::abs( a - midX ) < std::abs( b - midX ); } );
    std::sort( ys.begin(), ys.end(), [&]( int a, int b ) { return std::abs( a - midY ) < std::abs( b - midY ); } );

    for( int x : xs )
    {
        if( x != aFrom.x && x != aTo.x )
            candidates.push_back( { aFrom, VECTOR2I( x, aFrom.y ), VECTOR2I( x, aTo.y ), aTo } );
    }

    for( int y : ys )
    {
        if( y != aFrom.y && y != aTo.y )
            candidates.push_back( { aFrom, VECTOR2I( aFrom.x, y ), VECTOR2I( aTo.x, y ), aTo } );
    }

    std::optional<std::vector<VECTOR2I>> best;

    for( std::vector<VECTOR2I>& candidate : candidates )
    {
        // collapse zero-length legs (a Z whose start or end is already aligned)
        candidate.erase( std::unique( candidate.begin(), candidate.end() ), candidate.end() );

        if( !clear( candidate, aFrom, aTo ) )
            continue;

        if( !best || candidate.size() < best->size()
            || ( candidate.size() == best->size() && length( candidate ) < length( *best ) ) )
        {
            best = candidate;
        }

        // straight and L candidates come first: a clear one of those cannot be beaten
        if( best->size() <= 3 )
            break;
    }

    return best;
}

} // namespace kopenapi_sch
