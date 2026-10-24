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
#include <cmath>
#include <cstdlib>
#include <set>
#include <string>

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


/// @brief Two axis-aligned segments run along each other for more than a point
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




int length( const std::vector<VECTOR2I>& aPolyline )
{
    int total = 0;

    for( size_t i = 0; i + 1 < aPolyline.size(); ++i )
        total += std::abs( aPolyline[i + 1].x - aPolyline[i].x ) + std::abs( aPolyline[i + 1].y - aPolyline[i].y );

    return total;
}

} // namespace


/// @brief An axis-aligned segment enters the open interior of a box
bool SegmentCrossesBody( const VECTOR2I& aA, const VECTOR2I& aB, const BOX2I& aBox )
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


WIRE_ROUTER::WIRE_ROUTER( SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath, int aGrid,
                          const std::set<const SCH_ITEM*>& aIgnore ) :
        m_grid( aGrid )
{
    for( SCH_ITEM* item : aScreen->Items() )
    {
        if( aIgnore.count( item ) )
            continue;

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

            const BOX2I body = symbol->GetBodyBoundingBox();

            for( SCH_PIN* pin : symbol->GetPins( &aPath ) )
            {
                const VECTOR2I pos = pin->GetPosition();
                VECTOR2I       out( 0, 0 );

                if( pos.x < body.GetLeft() )
                    out = VECTOR2I( -1, 0 );
                else if( pos.x > body.GetRight() )
                    out = VECTOR2I( 1, 0 );
                else if( pos.y < body.GetTop() )
                    out = VECTOR2I( 0, -1 );
                else if( pos.y > body.GetBottom() )
                    out = VECTOR2I( 0, 1 );

                m_points.push_back( pos );
                m_pinOwner.push_back( { pos, m_bodies.size(), out } );
            }

            m_bodies.push_back( body );
            m_bodyNames.push_back( symbol->GetRef( &aPath, false ).ToStdString( wxConvUTF8 ) );
            break;
        }
        case SCH_SHEET_T:
        {
            SCH_SHEET* sheet = static_cast<SCH_SHEET*>( item );

            // sheet pins sit on the border: a wire leaving one may touch its own sheet
            for( SCH_SHEET_PIN* pin : sheet->GetPins() )
            {
                m_points.push_back( pin->GetPosition() );
                m_pinOwner.push_back( { pin->GetPosition(), m_bodies.size(), VECTOR2I( 0, 0 ) } );
            }

            m_bodies.push_back( BOX2I( sheet->GetPosition(), sheet->GetSize() ) );
            m_bodyNames.push_back( "sheet " + sheet->GetName().ToStdString( wxConvUTF8 ) );
            break;
        }

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


std::string WIRE_ROUTER::obstacle( const std::vector<VECTOR2I>& aPolyline, const VECTOR2I& aFrom,
                                   const VECTOR2I& aTo ) const
{
    auto isEnd = [&]( const VECTOR2I& p ) { return p == aFrom || p == aTo; };
    auto mm = []( const VECTOR2I& p )
    {
        return "(" + std::to_string( std::lround( p.x / 100.0 ) / 100.0 ).substr( 0, 7 ) + ", "
               + std::to_string( std::lround( p.y / 100.0 ) / 100.0 ).substr( 0, 7 ) + ") mm";
    };

    // A pin standing out of its part is reached from outside only: the end segment never runs back
    // along the pin into the part (out of the tip or sideways is fine, its body is an obstacle like
    // any other).  A pin on the body edge (sheet pins, power symbols) may graze its own body.
    std::set<size_t> own;
    const size_t     n = aPolyline.size();

    for( const PIN_END& pin : m_pinOwner )
    {
        if( pin.pos != aFrom && pin.pos != aTo )
            continue;

        if( pin.out == VECTOR2I( 0, 0 ) )
        {
            own.insert( pin.body );
            continue;
        }

        const VECTOR2I next = pin.pos == aFrom ? aPolyline[1] : aPolyline[n - 2];
        const VECTOR2I step = next - pin.pos;
        const VECTOR2I dir( ( step.x > 0 ) - ( step.x < 0 ), ( step.y > 0 ) - ( step.y < 0 ) );

        if( dir == VECTOR2I( -pin.out.x, -pin.out.y ) )
            return "reaches the pin at " + mm( pin.pos ) + " from inside "
                   + ( pin.body < m_bodyNames.size() ? m_bodyNames[pin.body] : std::string( "its part" ) );
    }

    for( size_t i = 0; i + 1 < aPolyline.size(); ++i )
    {
        const VECTOR2I& p = aPolyline[i];
        const VECTOR2I& q = aPolyline[i + 1];
        const bool      endSegment = i == 0 || i + 2 == aPolyline.size();

        if( p == q )
            return "zero-length segment";

        for( const auto& [s, e] : m_wires )
        {
            if( overlapAlong( p, q, s, e ) )
                return "runs along the wire " + mm( s ) + " - " + mm( e );

            for( const VECTOR2I& end : { s, e } )
            {
                if( onSegment( end, p, q ) && !isEnd( end ) )
                    return "touches the end of the wire " + mm( s ) + " - " + mm( e ) + " at " + mm( end );
            }

            for( const VECTOR2I& mine : { p, q } )
            {
                if( !isEnd( mine ) && onSegment( mine, s, e ) )
                    return "corner " + mm( mine ) + " lies on the wire " + mm( s ) + " - " + mm( e );
            }
        }

        for( const VECTOR2I& point : m_points )
        {
            if( !isEnd( point ) && onSegment( point, p, q ) )
                return "passes the connection point (pin end / label / junction) at " + mm( point );
        }

        for( size_t b = 0; b < m_bodies.size(); ++b )
        {
            if( endSegment && own.count( b ) )
                continue;

            if( SegmentCrossesBody( p, q, m_bodies[b] ) )
                return "crosses the body of " + ( b < m_bodyNames.size() ? m_bodyNames[b] : std::string( "a symbol" ) );
        }
    }

    return std::string();
}


std::optional<std::vector<VECTOR2I>> WIRE_ROUTER::Route( const VECTOR2I& aFrom, const VECTOR2I& aTo,
                                                         const std::string& aPrefer, std::string* aWhy ) const
{
    std::vector<std::vector<VECTOR2I>> candidates;

    // Straight, L, then Z shapes (a middle segment shifted along the grid, nearest the middle
    // first, up to 40 steps beyond the two ends) between two points
    auto shapes = [&]( const VECTOR2I& aA, const VECTOR2I& aB, const std::vector<VECTOR2I>& aHead,
                       const std::vector<VECTOR2I>& aTail )
    {
        auto add = [&]( std::vector<VECTOR2I> aMiddle )
        {
            std::vector<VECTOR2I> polyline = aHead;
            polyline.insert( polyline.end(), aMiddle.begin(), aMiddle.end() );
            polyline.insert( polyline.end(), aTail.begin(), aTail.end() );
            candidates.push_back( std::move( polyline ) );
        };

        if( aA.x == aB.x || aA.y == aB.y )
            add( { aA, aB } );

        const std::vector<VECTOR2I> hv = { aA, VECTOR2I( aB.x, aA.y ), aB };
        const std::vector<VECTOR2I> vh = { aA, VECTOR2I( aA.x, aB.y ), aB };

        if( aA.x != aB.x && aA.y != aB.y )
        {
            add( aPrefer == "vh" ? vh : hv );
            add( aPrefer == "vh" ? hv : vh );
        }

        constexpr int STEPS = 40;
        std::vector<int> xs, ys;

        for( int x = std::min( aA.x, aB.x ) - STEPS * m_grid; x <= std::max( aA.x, aB.x ) + STEPS * m_grid; x += m_grid )
            xs.push_back( x );

        for( int y = std::min( aA.y, aB.y ) - STEPS * m_grid; y <= std::max( aA.y, aB.y ) + STEPS * m_grid; y += m_grid )
            ys.push_back( y );

        const int midX = ( aA.x + aB.x ) / 2, midY = ( aA.y + aB.y ) / 2;
        std::sort( xs.begin(), xs.end(), [&]( int a, int b ) { return std::abs( a - midX ) < std::abs( b - midX ); } );
        std::sort( ys.begin(), ys.end(), [&]( int a, int b ) { return std::abs( a - midY ) < std::abs( b - midY ); } );

        for( int x : xs )
        {
            if( x != aA.x && x != aB.x )
                add( { aA, VECTOR2I( x, aA.y ), VECTOR2I( x, aB.y ), aB } );
        }

        for( int y : ys )
        {
            if( y != aA.y && y != aB.y )
                add( { aA, VECTOR2I( aA.x, y ), VECTOR2I( aB.x, y ), aB } );
        }
    };

    shapes( aFrom, aTo, {}, {} );

    // Pins facing away from each other: leave each pin straight out first, then the shapes above
    auto outOf = [&]( const VECTOR2I& aPin )
    {
        for( const PIN_END& pin : m_pinOwner )
        {
            if( pin.pos == aPin )
                return pin.out;
        }

        return VECTOR2I( 0, 0 );
    };

    const VECTOR2I outFrom = outOf( aFrom ), outTo = outOf( aTo );

    if( outFrom != VECTOR2I( 0, 0 ) || outTo != VECTOR2I( 0, 0 ) )
    {
        for( int steps = 1; steps <= 4; ++steps )
        {
            const VECTOR2I a = aFrom + outFrom * ( steps * m_grid );
            const VECTOR2I b = aTo + outTo * ( steps * m_grid );
            shapes( a, b, a == aFrom ? std::vector<VECTOR2I>() : std::vector<VECTOR2I>{ aFrom },
                    b == aTo ? std::vector<VECTOR2I>() : std::vector<VECTOR2I>{ aTo } );
        }
    }

    std::optional<std::vector<VECTOR2I>> best;

    for( std::vector<VECTOR2I>& candidate : candidates )
    {
        // collapse zero-length legs (a Z whose start or end is already aligned) and straight joints
        candidate.erase( std::unique( candidate.begin(), candidate.end() ), candidate.end() );

        for( size_t k = 1; k + 1 < candidate.size(); )
        {
            const VECTOR2I& p = candidate[k - 1];
            const VECTOR2I& q = candidate[k + 1];

            if( ( p.x == candidate[k].x && q.x == candidate[k].x ) || ( p.y == candidate[k].y && q.y == candidate[k].y ) )
                candidate.erase( candidate.begin() + k );
            else
                ++k;
        }

        if( candidate.size() < 2 || candidate.front() != aFrom || candidate.back() != aTo )
            continue;

        const std::string blocked = obstacle( candidate, aFrom, aTo );

        if( !blocked.empty() )
        {
            // Report why the simple shapes (straight, L) failed: that is what the agent expects
            if( aWhy && aWhy->empty() && candidate.size() <= 3 )
                *aWhy = blocked;

            continue;
        }

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
