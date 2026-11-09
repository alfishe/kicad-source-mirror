#include "tier_plan.h"

#include <base_units.h>
#include <board.h>
#include <footprint.h>
#include <math/box2.h>
#include <math/vector2d.h>
#include <geometry/shape_circle.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_segment.h>
#include <pad.h>
#include <pcb_track.h>
#include <zone.h>
#include <string_utils.h>

#include <algorithm>
#include <cmath>
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <map>
#include <set>
#include <vector>

namespace
{
const char* const SIDES[4] = { "north", "east", "south", "west" };

std::string str( const wxString& aText )
{
    return std::string( aText.ToUTF8() );
}

int sideOf( const VECTOR2D& aDir )
{
    return std::abs( aDir.x ) >= std::abs( aDir.y ) ? ( aDir.x >= 0 ? 1 : 3 ) : ( aDir.y >= 0 ? 2 : 0 );
}

VECTOR2D unitOf( int aSide )
{
    static const VECTOR2D U[4] = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 } };
    return U[aSide];
}

double dot( const VECTOR2D& aA, const VECTOR2D& aB )
{
    return aA.x * aB.x + aA.y * aB.y;
}

std::string netOf( const PAD* aPad )
{
    return aPad->GetNetCode() > 0 ? str( UnescapeString( aPad->GetNetname() ) ) : std::string();
}

// a long pad (a connector's finger): reached straight along it, whatever side of the part it is on
// an edge finger: a long pad whose outer end lies within its length of the board's outline
bool isFinger( const PAD* aPad, const SHAPE_POLY_SET& aOutline )
{
    const VECTOR2I size = aPad->GetSize( PADSTACK::ALL_LAYERS );

    if( std::max( size.x, size.y ) <= 3 * std::min( size.x, size.y ) || aOutline.OutlineCount() == 0 )
        return false;

    const BOX2I    bb = aPad->GetBoundingBox();
    const bool     wide = bb.GetWidth() > bb.GetHeight();
    const int      reach = std::max( bb.GetWidth(), bb.GetHeight() );
    const VECTOR2I ends[2] = { wide ? VECTOR2I( bb.GetLeft(), bb.GetCenter().y ) : VECTOR2I( bb.GetCenter().x, bb.GetTop() ),
                               wide ? VECTOR2I( bb.GetRight(), bb.GetCenter().y ) : VECTOR2I( bb.GetCenter().x, bb.GetBottom() ) };

    for( const VECTOR2I& end : ends )
    {
        for( int i = 0; i < aOutline.OutlineCount(); ++i )
        {
            if( aOutline.COutline( i ).SquaredDistance( end, true ) < (SEG::ecoord) reach * reach )
                return true;
        }
    }

    return false;
}

// a two-row part: its pads on two lines only (two x for rows west and east, two y for north and south); -1 else,
// 0 rows along y (west/east), 1 along x (north/south)
int rowsOf( const FOOTPRINT* aFp )
{
    std::set<int> xs, ys;
    const int     tol = pcbIUScale.mmToIU( 0.01 );

    for( PAD* p : aFp->Pads() )
    {
        xs.insert( p->GetPosition().x / tol );
        ys.insert( p->GetPosition().y / tol );
    }

    if( aFp->Pads().size() >= 4 && xs.size() == 2 && ys.size() > 2 )
        return 0;

    if( aFp->Pads().size() >= 4 && ys.size() == 2 && xs.size() > 2 )
        return 1;

    return -1;
}

bool twoRows( const FOOTPRINT* aFp )
{
    return rowsOf( aFp ) >= 0;
}

// the side of its part a pad is on: its row's for a two-row part, else by where it lies
int padSide( const PAD* aPad )
{
    const FOOTPRINT* fp = aPad->GetParentFootprint();
    const VECTOR2D   d( aPad->GetPosition() - fp->GetPosition() );

    switch( rowsOf( fp ) )
    {
    case 0: return d.x >= 0 ? 1 : 3;
    case 1: return d.y >= 0 ? 2 : 0;
    default: return sideOf( d );
    }
}

const char* entryOf( const PAD* aPad, int aFrom, const SHAPE_POLY_SET& aOutline )
{
    if( isFinger( aPad, aOutline ) )
        return "direct";

    const FOOTPRINT* fp = aPad->GetParentFootprint();
    const int        side = padSide( aPad );

    if( side == aFrom )
        return "direct";

    // the far row lies behind the facing row: reached only on the other layer, by a via under the body; a
    // two-row part entered from one of its ends reaches both rows from outside them
    if( side == ( aFrom + 2 ) % 4 )
        return "via_under";

    return twoRows( fp ) ? "direct" : "via_out";
}

// the same pin entered on the layer without its pads: a via beside it, or under the body for a far row
const char* viaEntryOf( const PAD* aPad, int aFrom )
{
    const FOOTPRINT* fp = aPad->GetParentFootprint();
    const int        side = padSide( aPad );
    return side == ( aFrom + 2 ) % 4 ? "via_under" : "via_out";
}

// a segment through a box (Liang-Barsky)
bool crosses( const VECTOR2D& aA, const VECTOR2D& aB, const BOX2I& aBox )
{
    double       t0 = 0, t1 = 1;
    const double dx = aB.x - aA.x, dy = aB.y - aA.y;
    const double p[4] = { -dx, dx, -dy, dy };
    const double q[4] = { aA.x - aBox.GetLeft(), aBox.GetRight() - aA.x, aA.y - aBox.GetTop(), aBox.GetBottom() - aA.y };

    for( int i = 0; i < 4; ++i )
    {
        if( p[i] == 0 )
        {
            if( q[i] < 0 )
                return false;

            continue;
        }

        const double r = q[i] / p[i];

        if( p[i] < 0 )
            t0 = std::max( t0, r );
        else
            t1 = std::min( t1, r );

        if( t0 > t1 )
            return false;
    }

    return true;
}

struct PART
{
    FOOTPRINT* fp = nullptr;
    BOX2I      body;
    int        lanes[2] = { 0, 0 };   ///< lanes left under it for traffic along x, along y
    int        signals = 0;
    int        pitch = 0;
};

// the axis of a run: 0 along x, 1 along y
int axisOf( int aSide )
{
    return aSide % 2 == 1 ? 0 : 1;
}

// a band beside aSpan on aSide, aDepth deep
BOX2I besideOf( const BOX2I& aSpan, int aSide, int aDepth )
{
    switch( aSide )
    {
    case 0: return BOX2I( VECTOR2I( aSpan.GetLeft(), aSpan.GetTop() - aDepth ), VECTOR2I( aSpan.GetWidth(), aDepth ) );
    case 1: return BOX2I( VECTOR2I( aSpan.GetRight(), aSpan.GetTop() ), VECTOR2I( aDepth, aSpan.GetHeight() ) );
    case 2: return BOX2I( VECTOR2I( aSpan.GetLeft(), aSpan.GetBottom() ), VECTOR2I( aSpan.GetWidth(), aDepth ) );
    default: return BOX2I( VECTOR2I( aSpan.GetLeft() - aDepth, aSpan.GetTop() ), VECTOR2I( aDepth, aSpan.GetHeight() ) );
    }
}
} // namespace


nlohmann::json PlanTiers( BOARD* aBoard, const std::vector<TIER_LINK>& aLinks,
                          const std::function<int( const PAD* )>& aPitch, const TIER_RULES& aRules )
{
    std::map<std::string, std::vector<PAD*>> netPads;
    std::map<std::string, FOOTPRINT*>        byRef;
    SHAPE_POLY_SET                           outline;

    if( !aBoard->GetBoardPolygonOutlines( outline, false ) )
        outline.RemoveAllContours();

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        byRef[str( fp->GetReference() )] = fp;

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 )
                netPads[netOf( pad )].push_back( pad );
        }
    }

    // a signal: a net on two parts or more with at most four pads (rails and buses' commons have more)
    auto isSignal = [&]( const PAD* aPad )
    {
        auto it = netPads.find( netOf( aPad ) );

        if( it == netPads.end() || it->second.size() > 4 )
            return false;

        for( const PAD* p : it->second )
        {
            if( p->GetParentFootprint() != aPad->GetParentFootprint() )
                return true;
        }

        return false;
    };

    std::map<const FOOTPRINT*, PART> parts;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        PART& part = parts[fp];
        part.fp = fp;
        part.body = fp->GetBoundingBox( false );

        for( PAD* pad : fp->Pads() )
        {
            if( isSignal( pad ) )
            {
                part.signals++;
                part.pitch = std::max( part.pitch, aPitch( pad ) );
            }
        }

        // lanes under it across an axis: its pad rows along that axis without a signal pin (a via stands at
        // each), or for fewer rows the body's width at the pitch less three per signal
        const int pitch = std::max( part.pitch, pcbIUScale.mmToIU( 0.4 ) );

        for( int axis = 0; axis < 2; ++axis )
        {
            std::map<int, bool> rows;   ///< cross coordinate -> holds a signal

            for( PAD* pad : fp->Pads() )
            {
                const int at = axis == 0 ? pad->GetPosition().y : pad->GetPosition().x;
                rows[at] = rows[at] || isSignal( pad );
            }

            int free = 0;

            for( const auto& [at, signal] : rows )
                free += signal ? 0 : 1;

            const int extent = axis == 0 ? part.body.GetHeight() : part.body.GetWidth();
            part.lanes[axis] = rows.size() >= 3 ? free : std::max( 0, extent / pitch - 3 * part.signals );
        }
    }

    // every connection of the board's signals as a segment, pad to the nearest pad of its net on another part
    std::vector<std::tuple<std::string, VECTOR2D, VECTOR2D>> segments;

    for( const auto& [net, pads] : netPads )
    {
        if( pads.size() > 4 )
            continue;

        for( const PAD* a : pads )
        {
            const PAD* best = nullptr;
            double     bestD = 0;

            for( const PAD* b : pads )
            {
                const double d = ( b->GetPosition() - a->GetPosition() ).EuclideanNorm();

                if( b->GetParentFootprint() != a->GetParentFootprint() && ( !best || d < bestD ) )
                    best = b, bestD = d;
            }

            if( best )
                segments.emplace_back( net, VECTOR2D( a->GetPosition() ), VECTOR2D( best->GetPosition() ) );
        }
    }

    struct PLAN
    {
        const TIER_LINK*        link;
        FOOTPRINT*              a = nullptr;
        FOOTPRINT*              b = nullptr;
        int                     runs = 1;
        double                  length = 0;
        BOX2I                   band;
        std::vector<FOOTPRINT*> between;   ///< parts in its band, nearest a first
        int                     side = -1;   ///< its detour side, -1 straight
        int                     depth = 0;
    };

    std::vector<PLAN> plans;

    auto endOf = [&]( const std::set<std::string>& aRefs, const TIER_LINK& aLink ) -> FOOTPRINT*
    {
        for( const std::string& ref : aRefs )
        {
            auto it = byRef.find( ref );

            if( it == byRef.end() )
                continue;

            for( PAD* pad : it->second->Pads() )
            {
                if( std::find( aLink.nets.begin(), aLink.nets.end(), netOf( pad ) ) != aLink.nets.end() )
                    return it->second;
            }
        }

        return nullptr;
    };

    for( const TIER_LINK& link : aLinks )
    {
        PLAN plan;
        plan.link = &link;
        plan.a = endOf( link.aParts, link );
        plan.b = endOf( link.bParts, link );

        if( !plan.a || !plan.b || plan.a == plan.b )
            continue;

        const VECTOR2D d( plan.b->GetPosition() - plan.a->GetPosition() );
        plan.runs = sideOf( d );
        plan.length = d.EuclideanNorm();
        plan.depth = (int) link.nets.size() * std::max( parts[plan.a].pitch, pcbIUScale.mmToIU( 0.4 ) ) + pcbIUScale.mmToIU( 1.0 );

        // the band between the two bodies, across both
        const BOX2I& ba = plan.a->GetBoundingBox( false );
        const BOX2I& bb = plan.b->GetBoundingBox( false );
        BOX2I        span = ba;
        span.Merge( bb );

        if( axisOf( plan.runs ) == 0 )
            plan.band = BOX2I( VECTOR2I( std::min( ba.GetRight(), bb.GetRight() ), span.GetTop() ),
                               VECTOR2I( std::max( ba.GetLeft(), bb.GetLeft() ) - std::min( ba.GetRight(), bb.GetRight() ), span.GetHeight() ) );
        else
            plan.band = BOX2I( VECTOR2I( span.GetLeft(), std::min( ba.GetBottom(), bb.GetBottom() ) ),
                               VECTOR2I( span.GetWidth(), std::max( ba.GetTop(), bb.GetTop() ) - std::min( ba.GetBottom(), bb.GetBottom() ) ) );

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( fp != plan.a && fp != plan.b && fp->GetBoundingBox( false ).Intersects( plan.band ) )
                plan.between.push_back( fp );
        }

        std::sort( plan.between.begin(), plan.between.end(), [&]( FOOTPRINT* x, FOOTPRINT* y )
                   { return ( x->GetPosition() - plan.a->GetPosition() ).EuclideanNorm() < ( y->GetPosition() - plan.a->GetPosition() ).EuclideanNorm(); } );
        plans.push_back( std::move( plan ) );
    }

    std::vector<size_t> order( plans.size() );

    for( size_t i = 0; i < order.size(); ++i )
        order[i] = i;

    std::stable_sort( order.begin(), order.end(), [&]( size_t x, size_t y ) { return plans[x].length < plans[y].length; } );

    std::map<std::pair<const FOOTPRINT*, int>, int> detourOf;   ///< (part, axis) -> the side its detours took
    std::vector<std::pair<BOX2I, int>>              taken;      ///< detour bands so far, with their lanes

    for( size_t i : order )
    {
        PLAN&     plan = plans[i];
        const int axis = axisOf( plan.runs );
        const int need = (int) plan.link->nets.size();
        bool      fits = true;

        for( FOOTPRINT* fp : plan.between )
            fits = fits && parts[fp].lanes[axis] >= need;

        // an end part with a neighbour in the band closer than a via row leaves no room for its stack there
        const int room = aRules.viaDiameter + 2 * aRules.clearance;

        for( FOOTPRINT* end : { plan.a, plan.b } )
        {
            const BOX2I eb = end->GetBoundingBox( false );

            for( FOOTPRINT* fp : plan.between )
            {
                const BOX2I fb = fp->GetBoundingBox( false );
                const int   gap = axis == 0 ? std::max( fb.GetLeft() - eb.GetRight(), eb.GetLeft() - fb.GetRight() )
                                            : std::max( fb.GetTop() - eb.GetBottom(), eb.GetTop() - fb.GetBottom() );

                if( gap < room )
                    fits = false;
            }
        }

        if( fits )
        {
            for( FOOTPRINT* fp : plan.between )
                parts[fp].lanes[axis] -= need;

            continue;
        }

        const int across[2] = { ( plan.runs + 3 ) % 4, ( plan.runs + 1 ) % 4 };
        int       best = -1;
        double    bestCost = 0;

        for( int side : across )
        {
            double cost = 0;
            auto   ia = detourOf.find( { plan.a, axis } );
            auto   ib = detourOf.find( { plan.b, axis } );

            if( ( ia != detourOf.end() && ia->second != side ) || ( ib != detourOf.end() && ib->second != side ) )
                cost += 1e6;

            BOX2I span = plan.a->GetBoundingBox( false );
            span.Merge( plan.b->GetBoundingBox( false ) );

            for( FOOTPRINT* fp : plan.between )
                span.Merge( fp->GetBoundingBox( false ) );

            const BOX2I band = besideOf( span, side, plan.depth );

            for( const auto& [net, a, b] : segments )
            {
                if( std::find( plan.link->nets.begin(), plan.link->nets.end(), net ) == plan.link->nets.end() && crosses( a, b, band ) )
                    cost += 1;
            }

            for( const auto& [box, lanes] : taken )
            {
                if( box.Intersects( band ) )
                    cost += lanes;
            }

            for( FOOTPRINT* fp : aBoard->Footprints() )
            {
                if( fp != plan.a && fp != plan.b && fp->GetBoundingBox( false ).Intersects( band ) )
                    cost += 1000;
            }

            // a detour on that side walls in another's end: one end among the parts it passes round, the other
            // beyond them
            for( size_t j : order )
            {
                const PLAN& other = plans[j];

                if( j == i || other.side != side )
                    continue;

                auto walls = [&]( const PLAN& aOuter, const PLAN& aInner )
                {
                    auto inside = [&]( FOOTPRINT* aFp ) { return std::find( aOuter.between.begin(), aOuter.between.end(), aFp ) != aOuter.between.end(); };
                    auto within = [&]( FOOTPRINT* aFp ) { return inside( aFp ) || aFp == aOuter.a || aFp == aOuter.b; };
                    return ( inside( aInner.a ) && !within( aInner.b ) ) || ( inside( aInner.b ) && !within( aInner.a ) );
                };

                if( walls( plan, other ) || walls( other, plan ) )
                    cost += 1e7;
            }

            if( best < 0 || cost < bestCost )
                best = side, bestCost = cost;
        }

        plan.side = best;
        detourOf[{ plan.a, axis }] = best;
        detourOf[{ plan.b, axis }] = best;

        BOX2I span = plan.a->GetBoundingBox( false );
        span.Merge( plan.b->GetBoundingBox( false ) );
        taken.emplace_back( besideOf( span, best, plan.depth ), need );
    }

    struct END
    {
        FOOTPRINT* fp;
        int        leaves;
    };

    struct DECIDED
    {
        int  from[2];
        bool direct;
    };

    std::vector<DECIDED> decided( plans.size() );

    for( size_t i = 0; i < plans.size(); ++i )
    {
        const PLAN& plan = plans[i];

        // the side it leaves an end by: across, when the part next to that end leaves no room for the end's whole
        // stack to turn; else along its run
        auto leaves = [&]( FOOTPRINT* aEnd, int aToward ) -> int
        {
            if( plan.side < 0 || plan.between.empty() )
                return aToward;

            // a straight link through under the part takes the way toward the run: a two-row part with an end
            // toward the detour leaves by that end
            if( rowsOf( aEnd ) >= 0 && plan.side % 2 == rowsOf( aEnd ) )
            {
                for( const PLAN& other : plans )
                {
                    if( other.side < 0 && std::find( other.between.begin(), other.between.end(), aEnd ) != other.between.end() )
                        return plan.side;
                }
            }

            FOOTPRINT*   next = aEnd == plan.a ? plan.between.front() : plan.between.back();
            const BOX2I& e = aEnd->GetBoundingBox( false );
            const BOX2I& n = next->GetBoundingBox( false );
            const int    gap = axisOf( aToward ) == 0 ? std::max( n.GetLeft() - e.GetRight(), e.GetLeft() - n.GetRight() )
                                                      : std::max( n.GetTop() - e.GetBottom(), e.GetTop() - n.GetBottom() );
            // the stack's vias and sort zone, then its ribbon turning: a lane per signal at the via spacing
            const double lane = std::max<double>( aRules.trackWidth + aRules.clearance, aRules.viaDiameter / 2.0 + aRules.clearance + aRules.trackWidth / 2.0 );
            const int    stack = KiROUND( parts[aEnd].signals * std::max<double>( lane, std::max( parts[aEnd].pitch, pcbIUScale.mmToIU( 0.4 ) ) )
                                          + 2.0 * ( aRules.viaDiameter + aRules.clearance ) ) + pcbIUScale.mmToIU( 1.0 );
            return gap < stack ? plan.side : aToward;
        };

        DECIDED& d = decided[i];
        d.from[0] = leaves( plan.a, plan.runs );
        d.from[1] = leaves( plan.b, ( plan.runs + 2 ) % 4 );

        // F.Cu when it passes under nothing and every pin is entered directly from where it arrives
        d.direct = plan.side >= 0 || plan.between.empty();

        for( const std::string& net : plan.link->nets )
        {
            for( PAD* pad : netPads[net] )
            {
                FOOTPRINT* fp = pad->GetParentFootprint();

                if( fp == plan.a || fp == plan.b )
                    d.direct = d.direct && std::string( entryOf( pad, fp == plan.a ? d.from[0] : d.from[1], outline ) ) == "direct";
            }
        }

        // into a two-row part's end with pins on both rows: the rows come out on either side, in no order the other
        // end shares; through a stack
        for( int end = 0; end < 2 && d.direct; ++end )
        {
            FOOTPRINT* fp = end == 0 ? plan.a : plan.b;
            std::set<int> rows;

            for( const std::string& net : plan.link->nets )
            {
                for( PAD* pad : netPads[net] )
                {
                    if( pad->GetParentFootprint() == fp )
                        rows.insert( padSide( pad ) );
                }
            }

            if( rowsOf( fp ) >= 0 && d.from[end] % 2 == rowsOf( fp ) && rows.size() > 1 )
                d.direct = false;
        }
    }

    // one stack per part side: a link through a stack takes every link leaving there with it
    for( bool changed = true; changed; )
    {
        changed = false;
        std::set<std::pair<const FOOTPRINT*, int>> stacked;

        for( size_t i = 0; i < plans.size(); ++i )
        {
            if( !decided[i].direct )
            {
                stacked.insert( { plans[i].a, decided[i].from[0] } );
                stacked.insert( { plans[i].b, decided[i].from[1] } );
            }
        }

        for( size_t i = 0; i < plans.size(); ++i )
        {
            if( decided[i].direct && ( stacked.count( { plans[i].a, decided[i].from[0] } ) || stacked.count( { plans[i].b, decided[i].from[1] } ) ) )
            {
                decided[i].direct = false;
                changed = true;
            }
        }
    }

    // the stacks on B.Cu: per part and side, every link end leaving there, its pins' vias and slots together
    struct SLOT
    {
        std::vector<VECTOR2I> padBends;   ///< on the pad's layer, before the via
        VECTOR2I              via, slot;
        std::vector<VECTOR2I> bends;      ///< on B.Cu, after the via
    };

    struct STACK_PIN
    {
        const PAD* pad;
        size_t     link;
        double     farT;   ///< its pin at the link's other end, across this stack
        const PAD* there;  ///< that pin
    };

    struct PASSENGER
    {
        const PAD* pad;
        bool       left;   ///< it turns to the left of the stack's way out
    };

    std::map<const PAD*, SLOT>                                          slots;
    std::map<const PAD*, bool>                                          ready;   ///< pins whose own copper leaves them: true when it reaches another part
    std::set<const PAD*>                                                offLayer;   ///< ready pins whose copper ends off the link's layer
    std::map<std::pair<const FOOTPRINT*, int>, std::vector<STACK_PIN>> stackPins;
    std::map<std::pair<const FOOTPRINT*, int>, std::vector<PASSENGER>> passengers;
    nlohmann::json                                                      stacksWhy = nlohmann::json::array();
    nlohmann::json                                                      passengersOut = nlohmann::json::array();
    auto mmOf = []( const VECTOR2I& p ) { return nlohmann::json::array( { pcbIUScale.IUTomm( p.x ), pcbIUScale.IUTomm( p.y ) } ); };

    // a pin's own copper already on the board: its tracks and vias from the pad on, as far as they go (the end
    // farthest along them), and whether they reach a pad of another part
    std::map<int, std::vector<PCB_TRACK*>> netCopper;

    for( PCB_TRACK* item : aBoard->Tracks() )
        netCopper[item->GetNetCode()].push_back( item );

    struct OWN_END
    {
        VECTOR2I at;
        bool     other;   ///< it reaches a pad of another part
        LSET     on;
        VECTOR2D way;     ///< the way its last track goes at its end
    };

    auto copperEnd = [&]( const PAD* aPad ) -> std::optional<OWN_END>
    {
        auto it = netCopper.find( aPad->GetNetCode() );

        if( it == netCopper.end() || aPad->GetNetCode() <= 0 )
            return std::nullopt;

        auto onPad = [&]( const PAD* aOn, const VECTOR2I& p )
        {
            return ( aOn->IsOnLayer( F_Cu ) && aOn->GetEffectiveShape( F_Cu )->Collide( p, 0 ) )
                   || ( aOn->IsOnLayer( B_Cu ) && aOn->GetEffectiveShape( B_Cu )->Collide( p, 0 ) );
        };

        std::map<VECTOR2I, double>   reached;
        std::map<VECTOR2I, VECTOR2I> before;
        std::vector<VECTOR2I>      todo;

        for( PCB_TRACK* item : it->second )
        {
            for( const VECTOR2I& p : { item->GetStart(), item->GetEnd() } )
            {
                if( onPad( aPad, p ) && !reached.count( p ) )
                {
                    reached[p] = 0;
                    todo.push_back( p );
                }
            }
        }

        if( todo.empty() )
            return std::nullopt;

        bool other = false;

        while( !todo.empty() )
        {
            const VECTOR2I p = todo.back();
            todo.pop_back();

            for( const PAD* there : netPads[netOf( aPad )] )
                other = other || ( there->GetParentFootprint() != aPad->GetParentFootprint() && onPad( there, p ) );

            for( PCB_TRACK* item : it->second )
            {
                if( item->Type() == PCB_VIA_T || ( item->GetStart() != p && item->GetEnd() != p ) )
                    continue;

                const VECTOR2I q = item->GetStart() == p ? item->GetEnd() : item->GetStart();
                const double   d = reached[p] + ( VECTOR2D( q ) - VECTOR2D( p ) ).EuclideanNorm();

                if( !reached.count( q ) || reached[q] > d + 1 )
                {
                    reached[q] = d;
                    before[q] = p;
                    todo.push_back( q );
                }
            }
        }

        auto farthest = std::max_element( reached.begin(), reached.end(), []( const auto& x, const auto& y ) { return x.second < y.second; } );

        if( farthest->second <= 0 )
            return std::nullopt;

        // the layers it ends on: a via's both, else its last track's
        LSET on;

        for( PCB_TRACK* item : it->second )
        {
            if( item->Type() == PCB_VIA_T ? item->GetPosition() == farthest->first : item->GetStart() == farthest->first || item->GetEnd() == farthest->first )
                on |= item->Type() == PCB_VIA_T ? LSET( { F_Cu, B_Cu } ) : LSET( { item->GetLayer() } );
        }

        VECTOR2D way( 0, 0 );

        if( before.count( farthest->first ) )
            way = ( VECTOR2D( farthest->first ) - VECTOR2D( before[farthest->first] ) ).Resize( 1.0 );

        return OWN_END{ farthest->first, other, on, way };
    };

    for( size_t i = 0; i < plans.size(); ++i )
    {
        if( decided[i].direct )
            continue;

        for( int end = 0; end < 2; ++end )
        {
            FOOTPRINT* fp = end == 0 ? plans[i].a : plans[i].b;
            FOOTPRINT* other = end == 0 ? plans[i].b : plans[i].a;
            const int  side = decided[i].from[end];

            for( const std::string& net : plans[i].link->nets )
            {
                const PAD* here = nullptr;
                const PAD* there = nullptr;

                for( PAD* pad : netPads[net] )
                {
                    if( pad->GetParentFootprint() == fp && !here )
                        here = pad;
                    else if( pad->GetParentFootprint() == other && !there )
                        there = pad;
                }

                if( here && there )
                {
                    if( auto own = copperEnd( here ); own && ( own->other || own->on.Contains( B_Cu ) ) )
                    {
                        ready[here] = own->other;
                        SLOT sl;
                        sl.via = own->at;
                        sl.slot = own->at;
                        slots[here] = sl;
                    }
                    else if( own )
                    {
                        // its own copper ends on F.Cu: a via a via's width on along its way takes it on to B.Cu
                        const VECTOR2I via( VECTOR2D( own->at ) + own->way * (double) aRules.viaDiameter );
                        ready[here] = false;
                        slots[here] = SLOT{ { own->at }, via, via, {} };
                        offLayer.insert( here );
                    }
                    else
                    {
                        stackPins[{ fp, side }].push_back( { here, i, 0, there } );
                    }
                }
            }
        }
    }

    // the facing row's other routed signals (passengers): out of the row on F.Cu past the stack's facing vias, then
    // straight to the side they turn to, ahead of the stack's own F.Cu copper
    for( auto& [key, pins] : stackPins )
    {
        const FOOTPRINT* fp = key.first;
        const int        side = key.second;
        std::set<const PAD*> in;

        for( const STACK_PIN& sp : pins )
            in.insert( sp.pad );

        for( PAD* pad : fp->Pads() )
        {
            if( in.count( pad ) || !aRules.routed.count( netOf( pad ) ) || padSide( pad ) != side )
                continue;

            VECTOR2D dest( 0, 0 );
            int      count = 0;

            for( const PAD* other : netPads[netOf( pad )] )
            {
                if( other->GetParentFootprint() != fp )
                {
                    dest += VECTOR2D( other->GetPosition() );
                    count++;
                }
            }

            if( count > 0 )
                passengers[key].push_back( { pad, dot( dest / (double) count - VECTOR2D( pad->GetPosition() ), unitOf( ( side + 3 ) % 4 ) ) > 0 } );
        }
    }

    const double viaR = aRules.viaDiameter / 2.0;
    const double clr = aRules.clearance;
    const double halfTrack = aRules.trackWidth / 2.0;
    const double pitchT = aRules.trackWidth + aRules.clearance;
    const double viaGap = 2.0 * viaR + clr;                 ///< via centre to via centre
    const double viaTrack = viaR + clr + halfTrack;         ///< via centre to track centre

    // a stack pin's copper (its pad to the via on the pad's layer, the via, B.Cu on to the slot) clear of the board's
    // tracks, vias, pads and zones of other nets and of rule areas without tracks or vias
    std::string lastHit;   ///< what clearOf last found in the way
    std::optional<std::pair<VECTOR2I, int>> lastVia;   ///< the via it found there, its centre and radius

    std::set<const PAD*> readyVias;   ///< the ready pins whose via past their copper's end is planned

    auto hitsOf = [&]( const PAD* aPad, const SHAPE& aShape, PCB_LAYER_ID aLayer, bool aVia )
    {
        const int net = aPad->GetNetCode();
        const int gap = aRules.clearance;
        BOX2I around = aShape.BBox();
        around.Inflate( gap + 1 );
        lastVia.reset();

        for( const PAD* other : readyVias )
        {
            if( other->GetNetCode() == net )
                continue;

            const SLOT&         sl = slots[other];
            const SHAPE_CIRCLE  via( sl.via, aRules.viaDiameter / 2 );
            const SHAPE_SEGMENT leg( sl.padBends[0], sl.via, aRules.trackWidth );

            if( aShape.Collide( &via, gap ) || ( aLayer == F_Cu && aShape.Collide( &leg, gap ) ) )
            {
                lastHit = str( aPad->GetParentFootprint()->GetReference() ) + "." + str( aPad->GetNumber() ) + ": the planned via of "
                          + str( other->GetParentFootprint()->GetReference() ) + "." + str( other->GetNumber() );
                return true;
            }
        }

        for( PCB_TRACK* item : aBoard->Tracks() )
        {
            if( !around.Intersects( item->GetBoundingBox() ) )
                continue;

            if( item->GetNetCode() != net && item->IsOnLayer( aLayer ) && item->GetEffectiveShape( aLayer )->Collide( &aShape, gap ) )
            {
                lastHit = str( aPad->GetParentFootprint()->GetReference() ) + "." + str( aPad->GetNumber() ) + ": " + item->GetNetname().ToStdString()
                          + ( item->Type() == PCB_VIA_T ? " via" : " track" ) + " on " + aBoard->GetLayerName( aLayer ).ToStdString();

                if( item->Type() == PCB_VIA_T )
                    lastVia = std::make_pair( item->GetPosition(), item->GetEffectiveShape( aLayer )->BBox().GetWidth() / 2 );

                return true;
            }
        }

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( !around.Intersects( fp->GetBoundingBox( false ) ) )
                continue;

            for( PAD* pad : fp->Pads() )
            {
                if( pad->GetNetCode() != net && pad->IsOnLayer( aLayer ) && pad->GetEffectiveShape( aLayer )->Collide( &aShape, gap ) )
                {
                    lastHit = str( aPad->GetParentFootprint()->GetReference() ) + "." + str( aPad->GetNumber() ) + ": pad " + str( fp->GetReference() ) + "."
                              + str( pad->GetNumber() ) + " on " + aBoard->GetLayerName( aLayer ).ToStdString();
                    return true;
                }
            }
        }

        for( ZONE* zone : aBoard->Zones() )
        {
            if( !zone->IsOnLayer( aLayer ) )
                continue;

            const bool rule = zone->GetIsRuleArea() && ( aVia ? zone->GetDoNotAllowVias() : zone->GetDoNotAllowTracks() );

            if( rule && zone->Outline()->Collide( &aShape, 0 ) )
            {
                lastHit = str( aPad->GetParentFootprint()->GetReference() ) + "." + str( aPad->GetNumber() ) + ": rule area "
                          + zone->GetZoneName().ToStdString() + " on " + aBoard->GetLayerName( aLayer ).ToStdString();
                return true;
            }
        }

        return false;
    };

    auto clearOf = [&]( const PAD* aPad, const SLOT& aSlot, bool aVia = true )
    {
        const int width = aRules.trackWidth;

        auto hits = [&]( const SHAPE& aShape, PCB_LAYER_ID aLayer, bool aVia ) { return hitsOf( aPad, aShape, aLayer, aVia ); };

        std::vector<VECTOR2I> front = { aPad->GetPosition() };
        front.insert( front.end(), aSlot.padBends.begin(), aSlot.padBends.end() );
        front.push_back( aSlot.via );
        std::vector<VECTOR2I> back = { aSlot.via };

        if( !aVia )
            back.clear();
        back.insert( back.end(), aSlot.bends.begin(), aSlot.bends.end() );
        back.push_back( aSlot.slot );
        const PCB_LAYER_ID padLayer = aPad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;

        for( size_t k = 1; k < front.size(); ++k )
        {
            if( front[k] != front[k - 1] && hits( SHAPE_SEGMENT( front[k - 1], front[k], width ), padLayer, false ) )
                return false;
        }

        for( size_t k = 1; k < back.size(); ++k )
        {
            if( back[k] != back[k - 1] && hits( SHAPE_SEGMENT( back[k - 1], back[k], width ), B_Cu, false ) )
                return false;
        }

        const SHAPE_CIRCLE via( aSlot.via, aRules.viaDiameter / 2 );
        return !aVia || ( !hits( via, F_Cu, true ) && !hits( via, B_Cu, true ) );
    };

    // the ready pins' vias and their F.Cu legs clear of each other: a pin whose via or leg comes too near another's
    std::map<const PAD*, std::string> crowded;

    for( const PAD* x : offLayer )
    {
        for( const PAD* y : offLayer )
        {
            if( x == y || x->GetNetCode() == y->GetNetCode() )
                continue;

            const SLOT& a = slots[x];
            const SLOT& b = slots[y];
            const SEG   legB( b.padBends[0], b.via );

            if( ( VECTOR2D( a.via ) - VECTOR2D( b.via ) ).EuclideanNorm() < viaGap || legB.Distance( a.via ) < viaTrack )
                crowded[x] = str( y->GetParentFootprint()->GetReference() ) + "." + str( y->GetNumber() ) + "'s";
        }
    }

    // a via of the pin's net there clear of the board's copper of other nets on both layers
    auto viaClear = [&]( const PAD* aPad, const VECTOR2I& aAt )
    {
        const SHAPE_CIRCLE via( aAt, aRules.viaDiameter / 2 );
        return !hitsOf( aPad, via, F_Cu, true ) && !hitsOf( aPad, via, B_Cu, true );
    };

    // the ready vias that fit, before the stacks: the stacks' copper keeps clear of them
    std::map<const PAD*, std::string> readyMiss;

    for( const PAD* pad : offLayer )
    {
        const SLOT&         sl = slots[pad];
        const SHAPE_SEGMENT leg( sl.padBends[0], sl.via, aRules.trackWidth );
        const std::string   name = str( pad->GetParentFootprint()->GetReference() ) + "." + str( pad->GetNumber() );

        if( auto c = crowded.find( pad ); c != crowded.end() )
            readyMiss[pad] = name + ": its own copper ends on F.Cu, its via there too near " + c->second;
        else if( sl.via == sl.padBends[0] || !viaClear( pad, sl.via ) || hitsOf( pad, leg, F_Cu, false ) )
            readyMiss[pad] = name + ": its own copper ends on F.Cu, no room for a via there (" + lastHit + ")";
    }

    for( const PAD* pad : offLayer )
    {
        if( !readyMiss.count( pad ) )
            readyVias.insert( pad );
    }

    // a stack pin's copper clear of the stack's other pins' copper: tracks on each layer, vias
    auto stackClear = [&]( const PAD* aPad, const SLOT& aSlot, const std::map<const PAD*, SLOT>& aStack )
    {
        auto chains = []( const PAD* aOwner, const SLOT& aSl, std::vector<SEG>& aFront, std::vector<SEG>& aBack )
        {
            std::vector<VECTOR2I> f = { aOwner->GetPosition() };
            f.insert( f.end(), aSl.padBends.begin(), aSl.padBends.end() );
            f.push_back( aSl.via );
            std::vector<VECTOR2I> b = { aSl.via };
            b.insert( b.end(), aSl.bends.begin(), aSl.bends.end() );
            b.push_back( aSl.slot );

            for( size_t k = 1; k < f.size(); ++k )
                aFront.emplace_back( f[k - 1], f[k] );

            for( size_t k = 1; k < b.size(); ++k )
                aBack.emplace_back( b[k - 1], b[k] );
        };

        std::vector<SEG> mine[2];
        chains( aPad, aSlot, mine[0], mine[1] );
        const int slack = pcbIUScale.mmToIU( 0.001 );
        const int track = aRules.trackWidth + clr - slack;

        for( const auto& [pad, sl] : aStack )
        {
            if( pad == aPad )
                continue;

            std::vector<SEG> theirs[2];
            chains( pad, sl, theirs[0], theirs[1] );

            if( ( VECTOR2D( sl.via ) - VECTOR2D( aSlot.via ) ).EuclideanNorm() < viaGap - slack )
                return false;

            for( int l = 0; l < 2; ++l )
            {
                for( const SEG& m : mine[l] )
                {
                    if( m.Distance( sl.via ) < viaTrack - slack )
                        return false;

                    for( const SEG& o : theirs[l] )
                    {
                        if( m.Distance( o ) < track )
                            return false;
                    }
                }

                for( const SEG& o : theirs[l] )
                {
                    if( o.Distance( aSlot.via ) < viaTrack - slack )
                        return false;
                }
            }
        }

        return true;
    };

    // a stack pin's copper that hits the board's moved out of the way, clear of the stack's other copper: its B.Cu
    // track bent round a via in its way at 45 degrees, or its via moved along the track it comes in on (further on, the
    // pad's layer on to it; back, the other layer on from it)
    std::string repairMiss;   ///< why the last repair failed

    auto repaired = [&]( const PAD* aPad, const SLOT& aSlot, const std::map<const PAD*, SLOT>& aStack ) -> std::optional<SLOT>
    {
        SLOT sl = aSlot;
        repairMiss = "no via in the way";

        for( int round = 0; round < 4; ++round )
        {
            if( clearOf( aPad, sl ) )
            {
                if( stackClear( aPad, sl, aStack ) )
                    return sl;

                repairMiss = "bent into the stack's own copper";
                break;
            }

            repairMiss = lastHit;

            if( !lastVia || !aPad->IsOnLayer( F_Cu ) )
                break;

            const auto [centre, radius] = *lastVia;
            const double need = radius + clr + halfTrack + pcbIUScale.mmToIU( 0.01 );
            std::vector<VECTOR2I> back = { sl.via };
            back.insert( back.end(), sl.bends.begin(), sl.bends.end() );
            back.push_back( sl.slot );
            bool bent = false;

            for( size_t k = 1; k < back.size() && !bent; ++k )
            {
                const SEG seg( back[k - 1], back[k] );

                if( seg.Distance( centre ) >= need - halfTrack || seg.Length() < 1 )
                    continue;

                const VECTOR2D a( seg.A ), d = VECTOR2D( seg.B - seg.A ).Resize( 1.0 );
                const VECTOR2D nrm( -d.y, d.x );
                const double   len = seg.Length();
                const double   t = ( VECTOR2D( centre ) - a ).Dot( d );
                const double   side = ( VECTOR2D( centre ) - a ).Dot( nrm );
                const double   off = ( side > 0 ? -1.0 : 1.0 ) * ( need - std::abs( side ) );
                const double   flat = std::sqrt( std::max( 0.0, need * need - ( need - std::abs( off ) ) * ( need - std::abs( off ) ) ) );
                const double   t0 = t - flat - std::abs( off ), t1 = t + flat + std::abs( off );

                if( t0 < 0 || t1 > len || std::abs( off ) > pcbIUScale.mmToIU( 0.3 ) )
                {
                    repairMiss = "no room to bend round " + lastHit;
                    break;
                }

                const std::vector<VECTOR2I> detour = { VECTOR2I( a + d * t0 ), VECTOR2I( a + d * ( t0 + std::abs( off ) ) + nrm * off ),
                                                       VECTOR2I( a + d * ( t1 - std::abs( off ) ) + nrm * off ), VECTOR2I( a + d * t1 ) };
                back.insert( back.begin() + k, detour.begin(), detour.end() );
                sl.bends.assign( back.begin() + 1, back.end() - 1 );
                bent = true;
            }

            if( !bent )
                break;
        }

        const VECTOR2I prev = aSlot.padBends.empty() ? aPad->GetPosition() : aSlot.padBends.back();
        const VECTOR2D in = VECTOR2D( aSlot.via ) - VECTOR2D( prev );
        const double   length = in.EuclideanNorm();

        if( length < 1 )
            return std::nullopt;

        const VECTOR2D dir = in / length;

        for( int step = 1; step <= 4; ++step )
        {
            const double d = step * pcbIUScale.mmToIU( 0.25 );

            for( int way : { 1, -1 } )
            {
                if( way < 0 && d > length - viaR - clr )
                    continue;

                SLOT           moved = aSlot;
                const VECTOR2I at( VECTOR2D( aSlot.via ) + dir * ( d * way ) );

                if( way > 0 )
                    moved.padBends.push_back( aSlot.via );

                moved.bends.insert( moved.bends.begin(), aSlot.via );
                moved.via = at;

                if( clearOf( aPad, moved ) && stackClear( aPad, moved, aStack ) )
                    return moved;
            }
        }

        return std::nullopt;
    };

    // the stacks out of a part's end first (their order is the least free), then the larger ones; a link's order at
    // its other end is then the order of its slots there
    std::vector<std::pair<const FOOTPRINT*, int>> stackOrder;

    for( const auto& [key, pins] : stackPins )
        stackOrder.push_back( key );

    auto atEnd = []( const std::pair<const FOOTPRINT*, int>& aKey )
    {
        const int rows = rowsOf( aKey.first );
        return rows >= 0 && aKey.second % 2 == rows;
    };

    // a pin's place in the ribbon from its link's other end: the slots there across the way that end leaves by, seen
    // from this end (a ribbon keeps its order); -1e18 while that end has no slots
    auto farSlotT = [&]( const STACK_PIN& aPin, const FOOTPRINT* aHere )
    {
        auto it = slots.find( aPin.there );

        if( it == slots.end() )
            return -1e18;

        const int otherSide = decided[aPin.link].from[plans[aPin.link].a == aHere ? 1 : 0];
        return -dot( VECTOR2D( it->second.slot ), unitOf( ( otherSide + 1 ) % 4 ) );
    };

    std::stable_sort( stackOrder.begin(), stackOrder.end(), [&]( const auto& x, const auto& y )
    {
        if( atEnd( x ) != atEnd( y ) )
            return atEnd( x );

        if( stackPins[x].size() != stackPins[y].size() )
            return stackPins[x].size() > stackPins[y].size();

        const std::string rx = str( x.first->GetReference() ), ry = str( y.first->GetReference() );
        return rx != ry ? rx < ry : x.second < y.second;
    } );

    for( const auto& key : stackOrder )
    {
        std::vector<STACK_PIN>& pins = stackPins[key];
        const FOOTPRINT* fp = key.first;
        const int        side = key.second;
        const std::string where = str( fp->GetReference() ) + " " + SIDES[side];

        auto refuse = [&]( const char* aWhy ) { stacksWhy.push_back( { { "stack", where }, { "not_covered", aWhy } } ); };

        if( !twoRows( fp ) || std::any_of( fp->Pads().begin(), fp->Pads().end(), []( PAD* p ) { return p->GetAttribute() != PAD_ATTRIB::SMD; } ) )
        {
            refuse( "not a two-row SMD part" );
            continue;
        }

        const VECTOR2D u = unitOf( side );
        const VECTOR2D t = unitOf( ( side + 1 ) % 4 );
        const VECTOR2D c( fp->GetPosition() );
        auto           along = [&]( const VECTOR2D& p ) { return dot( p - c, u ); };
        auto           across = [&]( const VECTOR2D& p ) { return dot( p - c, t ); };
        auto           at = [&]( double aU, double aT ) { return VECTOR2I( c + u * aU + t * aT ); };

        // the rows: facing (on the side), far (opposite); their ends along u, the pads' extent across
        double faceOut = -1e18, faceIn = 1e18, farIn = -1e18, farOut = 1e18, tMin = 1e18, tMax = -1e18;
        bool   rowsOk = true;

        for( PAD* pad : fp->Pads() )
        {
            const int   ps = padSide( pad );
            const BOX2I bb = pad->GetBoundingBox();
            double      lo = 1e18, hi = -1e18;

            for( const VECTOR2I& corner : { bb.GetOrigin(), bb.GetEnd(), VECTOR2I( bb.GetLeft(), bb.GetBottom() ), VECTOR2I( bb.GetRight(), bb.GetTop() ) } )
            {
                lo = std::min( lo, along( VECTOR2D( corner ) ) );
                hi = std::max( hi, along( VECTOR2D( corner ) ) );
                tMin = std::min( tMin, across( VECTOR2D( corner ) ) );
                tMax = std::max( tMax, across( VECTOR2D( corner ) ) );
            }

            if( ps == side )
            {
                faceOut = std::max( faceOut, hi );
                faceIn = std::min( faceIn, lo );
            }
            else if( ps == ( side + 2 ) % 4 )
            {
                farIn = std::max( farIn, hi );
                farOut = std::min( farOut, lo );
            }
            else
            {
                rowsOk = false;
            }
        }

        // an end of the part: the pins leave along the rows, out of their rows or into the channel between them on
        // F.Cu to a via past the end, or through a via beside or under their pad on B.Cu (under the body, or on across
        // under the other row to its outside); each pin's way is a leg at its row to its column, then up the column
        // past the end; the columns stand in the stack's order
        if( !rowsOk && faceOut < -1e17 && farIn < -1e17 )
        {
            const int leftSide = ( side + 3 ) % 4;
            double    lOut = 1e18, lIn = -1e18, rIn = 1e18, rOut = -1e18, uEnd = -1e18;

            for( PAD* pad : fp->Pads() )
            {
                const BOX2I bb = pad->GetBoundingBox();

                for( const VECTOR2I& corner : { bb.GetOrigin(), bb.GetEnd(), VECTOR2I( bb.GetLeft(), bb.GetBottom() ), VECTOR2I( bb.GetRight(), bb.GetTop() ) } )
                {
                    const double a = along( VECTOR2D( corner ) ), x = across( VECTOR2D( corner ) );
                    uEnd = std::max( uEnd, a );

                    if( padSide( pad ) == leftSide )
                        lOut = std::min( lOut, x ), lIn = std::max( lIn, x );
                    else
                        rIn = std::min( rIn, x ), rOut = std::max( rOut, x );
                }
            }

            // the order: the links turning left first, then those turning right; leaving into its detour, a longer
            // link runs in an outer tier, so it lies away from the side it turns to; inside a link its pins in the
            // order they arrive at its other end after the turn
            auto towardOf = [&]( const STACK_PIN& aPin )
            {
                const PLAN& plan = plans[aPin.link];
                return plan.a == fp ? plan.runs : ( plan.runs + 2 ) % 4;
            };

            auto endClass = [&]( const STACK_PIN& aPin )
            {
                const int toward = towardOf( aPin );
                return toward == leftSide ? 0 : toward == ( side + 1 ) % 4 ? 2 : 1;
            };

            // inside a link whose other end has no slots yet, the order this end's rows leave in without crossing
            // into the channel: the left row's pins nearest the end last, the right row's first; the other end
            // follows these slots
            std::set<size_t> farFree;

            for( STACK_PIN& sp : pins )
            {
                const double slotT = farSlotT( sp, fp );
                sp.farT = slotT > -1e17 ? slotT : dot( VECTOR2D( sp.there->GetPosition() ), unitOf( ( towardOf( sp ) + 1 ) % 4 ) );

                if( slotT < -1e17 )
                    farFree.insert( sp.link );
            }

            std::map<size_t, std::pair<double, int>> linkT;   ///< per link its pins' farT sum and count

            for( const STACK_PIN& sp : pins )
            {
                linkT[sp.link].first += sp.farT;
                linkT[sp.link].second++;
            }

            auto meanT = [&]( size_t aLink ) { return linkT[aLink].first / linkT[aLink].second; };

            auto rowKey = [&]( const STACK_PIN& aPin )
            {
                const double a = along( VECTOR2D( aPin.pad->GetPosition() ) );
                return padSide( aPin.pad ) == leftSide ? std::make_pair( 0, -a ) : std::make_pair( 1, a );
            };

            const size_t n = pins.size();

            if( n > 10 )
            {
                refuse( "more than 10 pins at an end" );
                continue;
            }

            nlohmann::json endMisses = nlohmann::json::array();
            bool           endPlaced = false;

            // per link whose other end is free: in the order its pins come in there, else the one the rows leave in
            // without crossing; fewer links by their rows first
            std::vector<size_t> freeLinks( farFree.begin(), farFree.end() );
            std::vector<int>    masks;

            for( int mask = 0; mask < ( 1 << std::min<size_t>( freeLinks.size(), 4 ) ); ++mask )
                masks.push_back( mask );

            auto bits = []( int aMask )
            {
                int count = 0;

                for( ; aMask; aMask &= aMask - 1 )
                    ++count;

                return count;
            };

            std::stable_sort( masks.begin(), masks.end(), [&]( int x, int y ) { return bits( x ) < bits( y ); } );

            for( size_t pass = 0; pass < masks.size() && !endPlaced; ++pass )
            {
                auto byRows = [&]( size_t aLink )
                {
                    for( size_t f = 0; f < freeLinks.size() && f < 4; ++f )
                    {
                        if( freeLinks[f] == aLink )
                            return ( masks[pass] >> f & 1 ) != 0;
                    }

                    return true;
                };
                std::stable_sort( pins.begin(), pins.end(), [&]( const STACK_PIN& x, const STACK_PIN& y )
                {
                    const int cx = endClass( x ), cy = endClass( y );

                    if( cx != cy )
                        return cx < cy;

                    const double lx = plans[x.link].length, ly = plans[y.link].length;

                    if( x.link != y.link && std::abs( lx - ly ) > pcbIUScale.mmToIU( 0.5 ) )
                        return cx == 2 ? lx > ly : lx < ly;

                    if( x.link != y.link )
                        return std::make_pair( meanT( x.link ), x.link ) < std::make_pair( meanT( y.link ), y.link );

                    if( farFree.count( x.link ) && byRows( x.link ) )
                        return rowKey( x ) < rowKey( y );

                    return x.farT < y.farT;
                } );


                std::vector<double> v( n );
                std::vector<bool>   onLeft( n );

                for( size_t k = 0; k < n; ++k )
                {
                    v[k] = along( VECTOR2D( pins[k].pad->GetPosition() ) );
                    onLeft[k] = padSide( pins[k].pad ) == leftSide;
                }

                // per pin 0 out of its row on F.Cu, 1 into the channel on F.Cu, 2 a via beside its pad, 3 a via under the
                // body, 4 a via under the body and across under the other row; the region its column stands in: 0 left
                // of the rows, 1 between them, 2 right of them
                auto regionOf = [&]( size_t k, int aKind )
                {
                    if( aKind == 1 || aKind == 3 )
                        return 1;

                    return ( aKind == 4 ) != (bool) onLeft[k] ? 0 : 2;
                };

                // out of its row on F.Cu, a pin's leg walls in the routed pads of that row nearer the end that are not in
                // the stack
                auto wallsIn = [&]( size_t k )
                {
                    for( PAD* pad : fp->Pads() )
                    {
                        if( padSide( pad ) != padSide( pins[k].pad ) || along( VECTOR2D( pad->GetPosition() ) ) <= v[k] + 1
                            || !aRules.routed.count( netOf( pad ) ) )
                            continue;

                        if( std::none_of( pins.begin(), pins.end(), [&]( const STACK_PIN& sp ) { return sp.pad == pad; } ) )
                            return true;
                    }

                    return false;
                };

                struct END_TRY
                {
                    int              cost;
                    std::vector<int> kinds;
                };

                std::vector<END_TRY> tries;
                std::vector<int>     kindOf( n );

                std::function<void( size_t, int, int )> walk = [&]( size_t k, int aRegion, int aCost )
                {
                    if( tries.size() >= 20000 )
                        return;

                    if( k == n )
                    {
                        tries.push_back( { aCost, kindOf } );
                        return;
                    }

                    for( int kind = 0; kind < 5; ++kind )
                    {
                        const int region = regionOf( k, kind );

                        if( region < aRegion || ( kind == 0 && wallsIn( k ) ) )
                            continue;

                        kindOf[k] = kind;
                        walk( k + 1, region, aCost + ( kind == 4 ? 2 : 1 ) );
                    }
                };

                walk( 0, 0, 0 );
                std::stable_sort( tries.begin(), tries.end(), []( const END_TRY& x, const END_TRY& y ) { return x.cost < y.cost; } );

                const double   width = aRules.trackWidth;

                for( size_t tried = 0; tried < std::min<size_t>( tries.size(), 4096 ) && !endPlaced; ++tried )
                {
                    const std::vector<int>& kind = tries[tried].kinds;
                    auto isF = [&]( size_t k ) { return kind[k] <= 1; };

                    // the vias beside and under the pads, staggered along a row
                    std::vector<double> viaT( n, 0 );

                    for( int zone = 0; zone < 4; ++zone )   ///< left outside, left under, right under, right outside
                    {
                        const bool   left = zone < 2, outside = zone == 0 || zone == 3;
                        const double base = outside ? ( left ? lOut - clr - viaR : rOut + clr + viaR ) : ( left ? lIn + clr + viaR : rIn - clr - viaR );
                        const double away = outside == left ? -1 : 1;   ///< the way the stagger goes
                        std::vector<size_t> idx;

                        for( size_t k = 0; k < n; ++k )
                        {
                            if( onLeft[k] == left && ( outside ? kind[k] == 2 : kind[k] >= 3 ) )
                                idx.push_back( k );
                        }

                        std::sort( idx.begin(), idx.end(), [&]( size_t x, size_t y ) { return v[x] < v[y]; } );
                        double lastV = -1e18, lastT = 1e18;

                        for( size_t k : idx )
                        {
                            const double dv = v[k] - lastV;
                            viaT[k] = dv < viaGap && std::abs( lastT - base ) < 1 ? base + away * ( std::sqrt( std::max( 0.0, viaGap * viaGap - dv * dv ) ) + pcbIUScale.mmToIU( 0.05 ) ) : base;
                            lastV = v[k];
                            lastT = viaT[k];
                        }
                    }

                    // the regions' bounds: clear of the pads and of the vias in them
                    double leftMax = lOut - clr - halfTrack, insideMin = lIn + clr + halfTrack, insideMax = rIn - clr - halfTrack, rightMin = rOut + clr + halfTrack;

                    for( size_t k = 0; k < n; ++k )
                    {
                        if( kind[k] == 2 && onLeft[k] )
                            leftMax = std::min( leftMax, viaT[k] - viaTrack );
                        else if( kind[k] == 2 )
                            rightMin = std::max( rightMin, viaT[k] + viaTrack );
                        else if( kind[k] >= 3 && onLeft[k] )
                            insideMin = std::max( insideMin, viaT[k] + viaTrack );
                        else if( kind[k] >= 3 )
                            insideMax = std::min( insideMax, viaT[k] - viaTrack );
                    }

                    // the columns in the stack's order; next to an F.Cu column (its via past the end) a track needs room
                    // for a via
                    auto gapOf = [&]( size_t a, size_t b ) { return isF( a ) || isF( b ) ? viaTrack : pitchT; };
                    std::vector<double> col( n, 0 );
                    std::vector<size_t> byRegion[3];

                    for( size_t k = 0; k < n; ++k )
                        byRegion[regionOf( k, kind[k] )].push_back( k );

                    for( size_t g = byRegion[0].size(); g-- > 0; )
                        col[byRegion[0][g]] = g + 1 == byRegion[0].size() ? leftMax : col[byRegion[0][g + 1]] - gapOf( byRegion[0][g], byRegion[0][g + 1] );

                    for( size_t g = 0; g < byRegion[2].size(); ++g )
                        col[byRegion[2][g]] = g == 0 ? rightMin : col[byRegion[2][g - 1]] + gapOf( byRegion[2][g - 1], byRegion[2][g] );

                    bool fits = true;

                    if( !byRegion[1].empty() )
                    {
                        double span = 0;

                        for( size_t g = 1; g < byRegion[1].size(); ++g )
                            span += gapOf( byRegion[1][g - 1], byRegion[1][g] );

                        double at0 = ( insideMin + insideMax - span ) / 2;
                        fits = at0 >= insideMin - 1;

                        for( size_t g = 0; g < byRegion[1].size(); ++g )
                        {
                            col[byRegion[1][g]] = at0;

                            if( g + 1 < byRegion[1].size() )
                                at0 += gapOf( byRegion[1][g], byRegion[1][g + 1] );
                        }
                    }

                    if( !fits )
                        continue;

                    // the F.Cu columns' vias past the end, neighbours staggered
                    const double uVia = uEnd + clr + viaR;
                    const double stagger = std::sqrt( std::max( 0.0, viaGap * viaGap - viaTrack * viaTrack ) ) + pcbIUScale.mmToIU( 0.05 );
                    const double uSlot = uVia + stagger + viaR + clr + halfTrack;
                    std::vector<double> viaU( n, uVia );
                    bool                up = false;

                    for( size_t k = 0; k < n; ++k )
                    {
                        if( !isF( k ) )
                        {
                            up = false;
                            continue;
                        }

                        viaU[k] = up ? uVia + stagger : uVia;
                        up = !up && k + 1 < n && isF( k + 1 ) && col[k + 1] - col[k] < viaGap;
                    }

                    std::map<const PAD*, SLOT> local;

                    for( size_t k = 0; k < n; ++k )
                    {
                        SLOT sl;

                        if( isF( k ) )
                        {
                            sl.padBends.push_back( at( v[k], col[k] ) );
                            sl.via = at( viaU[k], col[k] );
                        }
                        else
                        {
                            sl.via = at( v[k], viaT[k] );
                            sl.bends.push_back( at( v[k], col[k] ) );
                        }

                        sl.slot = at( uSlot, col[k] );
                        local[pins[k].pad] = sl;
                    }

                    // the stack's own copper apart: tracks on one layer, vias from everything
                    auto pathOf = [&]( size_t k, bool aFront )
                    {
                        const SLOT&           sl = local[pins[k].pad];
                        std::vector<VECTOR2I> pts;

                        if( aFront )
                        {
                            pts.push_back( pins[k].pad->GetPosition() );
                            pts.insert( pts.end(), sl.padBends.begin(), sl.padBends.end() );
                            pts.push_back( sl.via );
                        }
                        else
                        {
                            pts.push_back( sl.via );
                            pts.insert( pts.end(), sl.bends.begin(), sl.bends.end() );
                            pts.push_back( sl.slot );
                        }

                        std::vector<SEG> segs;

                        for( size_t i = 1; i < pts.size(); ++i )
                        {
                            if( pts[i] != pts[i - 1] )
                                segs.emplace_back( pts[i - 1], pts[i] );
                        }

                        return segs;
                    };

                    std::string selfHit;

                    for( size_t a = 0; a < n && selfHit.empty(); ++a )
                    {
                        for( size_t b = a + 1; b < n && selfHit.empty(); ++b )
                        {
                            const VECTOR2I va = local[pins[a].pad].via, vb = local[pins[b].pad].via;

                            if( ( va - vb ).EuclideanNorm() < viaGap - 1 )
                                selfHit = "vias";

                            for( bool front : { true, false } )
                            {
                                for( const SEG& sa : pathOf( a, front ) )
                                {
                                    if( sa.Distance( vb ) < viaTrack - 1 )
                                        selfHit = "a via";

                                    for( const SEG& sb : pathOf( b, front ) )
                                    {
                                        if( sa.Distance( sb ) < width + clr - 1 )
                                            selfHit = front ? "F.Cu tracks" : "B.Cu tracks";
                                    }
                                }

                                for( const SEG& sb : pathOf( b, front ) )
                                {
                                    if( sb.Distance( va ) < viaTrack - 1 )
                                        selfHit = "a via";
                                }
                            }
                        }
                    }

                    bool clear = selfHit.empty();

                    if( !clear )
                        lastHit = "the stack's own " + selfHit;

                    for( const auto& [pad, sl] : local )
                        clear = clear && clearOf( pad, sl );

                    if( !clear )
                    {
                        std::string kinds;

                        for( int kk : kind )
                            kinds += "OIWVX"[kk];

                        if( endMisses.size() < 32 )
                            endMisses.push_back( { { "kinds", kinds }, { "hit", lastHit } } );

                        continue;
                    }

                    slots.insert( local.begin(), local.end() );
                    endPlaced = true;
                }

                if( !endPlaced && pass + 1 == masks.size() )
                {
                    refuse( tries.empty() ? "no order at its end" : "its copper would hit the board's" );
                    nlohmann::json order = nlohmann::json::array();

                    for( size_t k = 0; k < n; ++k )
                        order.push_back( { { "pad", str( fp->GetReference() ) + "." + str( pins[k].pad->GetNumber() ) }, { "net", netOf( pins[k].pad ) },
                                           { "row", pcbIUScale.IUTomm( v[k] ) }, { "left", (bool) onLeft[k] }, { "turn", endClass( pins[k] ) } } );

                    stacksWhy.back()["order"] = order;

                    if( !endMisses.empty() )
                        stacksWhy.back()["tried"] = endMisses;
                }
            }

            continue;
        }

        if( !rowsOk || faceOut < -1e17 || farIn < -1e17 )
        {
            refuse( "left along its rows" );
            continue;
        }

        // the order the stack leaves in: the links turning to the left of the way out first, then the straight ones,
        // then those turning right; a longer detour runs in an outer tier, so it lies further toward its side; inside
        // a link its pins in the order of their pins at its other end
        const int left = ( side + 3 ) % 4;

        for( STACK_PIN& sp : pins )
        {
            const double slotT = farSlotT( sp, fp );
            sp.farT = slotT > -1e17 ? slotT : dot( VECTOR2D( sp.there->GetPosition() ), t );
        }

        auto classOf = [&]( const STACK_PIN& aPin )
        {
            return plans[aPin.link].side < 0 ? 1 : plans[aPin.link].side == left ? 0 : 2;
        };

        std::stable_sort( pins.begin(), pins.end(), [&]( const STACK_PIN& x, const STACK_PIN& y )
        {
            const int cx = classOf( x ), cy = classOf( y );

            if( cx != cy )
                return cx < cy;

            if( x.link != y.link )
            {
                const double lx = plans[x.link].length, ly = plans[y.link].length;
                return cx == 2 ? lx < ly : lx > ly;
            }

            return x.farT < y.farT;
        } );

        const size_t        n = pins.size();
        std::vector<bool>   isFar( n );

        std::vector<double> rowT( n );

        for( size_t k = 0; k < n; ++k )
        {
            isFar[k] = padSide( pins[k].pad ) != side;
            rowT[k] = across( VECTOR2D( pins[k].pad->GetPosition() ) );
        }

        // the passengers: those turning left above those turning right, or they cross
        std::vector<std::pair<double, const PAD*>> lefts, rights;

        for( const PASSENGER& ps : passengers[key] )
            ( ps.left ? lefts : rights ).push_back( { across( VECTOR2D( ps.pad->GetPosition() ) ), ps.pad } );

        std::sort( lefts.begin(), lefts.end(), []( const auto& x, const auto& y ) { return x.first < y.first; } );
        std::sort( rights.begin(), rights.end(), []( const auto& x, const auto& y ) { return x.first > y.first; } );

        if( !lefts.empty() && !rights.empty() && lefts.back().first > rights.back().first )
        {
            refuse( "its other signals there would cross" );
            continue;
        }

        // each pin on B.Cu (a far pin's via under the body, a facing pin's just outside its pad or under the body,
        // then across at its row) or on F.Cu (a facing pin out from its pad, a far pin along a channel out of the
        // part's end, between the rows or round the outside of the far row, with its via at the slot or just past the
        // part's end); on each layer the pins in the stack's order must be in their rows' order, no row holds a B.Cu
        // pin of both rows, and a channel stays clear of the copper it would cross on its layer; cheapest first
        const size_t limit = 16;

        struct STACK_TRY
        {
            int              cost;
            std::vector<int> kinds;     ///< per pin 0 B.Cu, 1 F.Cu facing, 2 channel top, 3 channel bottom, 4 facing via under the body
            bool             round;     ///< the channels outside the far row instead of between the rows
            bool             endVias;   ///< the channels' vias just past the part's end, on B.Cu from there
            int              push;      ///< the facing vias and the channels round the ends that many quarter millimetres further out
        };

        std::vector<STACK_TRY> candidates;

        // the far row's other routed pads: a channel round the outside walls them in
        std::vector<double> farOthers;

        for( PAD* pad : fp->Pads() )
        {
            if( padSide( pad ) == ( side + 2 ) % 4 && aRules.routed.count( netOf( pad ) )
                && std::none_of( pins.begin(), pins.end(), [&]( const STACK_PIN& sp ) { return sp.pad == pad; } ) )
            {
                farOthers.push_back( across( VECTOR2D( pad->GetPosition() ) ) );
            }
        }

        if( n > limit )
        {
            refuse( "more than 16 pins" );
            continue;
        }

        // the options per pin: far B.Cu, channel top, channel bottom; facing B.Cu outside, F.Cu, B.Cu under the body
        // (a large stack: far B.Cu or a channel on the side its place gives, facing B.Cu or F.Cu)
        const bool              small = n <= 10;
        std::vector<int>        kind( n );
        std::vector<int>        digit( n, 0 );
        std::vector<int>        radix( n );

        for( size_t k = 0; k < n; ++k )
            radix[k] = !small ? 2 : isFar[k] || n > 8 ? 3 : 5;

        auto isB = []( int aKind ) { return aKind == 0 || aKind == 4; };

        for( bool more = true; more; )
        {
            for( size_t k = 0; k < n; ++k )
                kind[k] = isFar[k] ? ( digit[k] == 0 ? 0 : digit[k] == 1 ? 2 : 3 ) : std::array<int, 5>{ 0, 1, 4, 2, 3 }[digit[k]];

            if( !small )
            {
                // the old positional rule: a far F.Cu pin is a top channel before every facing F.Cu pin, else bottom
                int firstF = -1;

                for( size_t k = 0; k < n; ++k )
                    firstF = firstF < 0 && kind[k] == 1 ? (int) k : firstF;

                for( size_t k = 0; k < n; ++k )
                    kind[k] = kind[k] == 2 && firstF >= 0 && (int) k > firstF ? 3 : kind[k];
            }

            more = false;

            for( size_t k = 0; k < n && !more; ++k )
            {
                if( ++digit[k] < radix[k] )
                    more = true;
                else
                    digit[k] = 0;
            }

            int  cost = 0;
            bool ok = true, channels = false, facingChannels = false;

            for( size_t k = 0; k < n; ++k )
            {
                cost += kind[k] == 1 ? 1 : kind[k] == 4 ? 2 : kind[k] >= 2 ? 3 : 0;
                channels = channels || kind[k] == 2 || kind[k] == 3;
                facingChannels = facingChannels || ( !isFar[k] && ( kind[k] == 2 || kind[k] == 3 ) );
            }

            // the B.Cu pins in their rows' order, the F.Cu facing ones too
            double lastB = -1e18, lastF = -1e18;

            for( size_t k = 0; k < n && ok; ++k )
            {
                if( isB( kind[k] ) )
                {
                    ok = rowT[k] > lastB;
                    lastB = rowT[k];
                }
                else if( kind[k] == 1 )
                {
                    ok = rowT[k] > lastF;
                    lastF = rowT[k];
                }
            }

            // a row with B.Cu pins of both rows; a bottom channel above a top one; a passenger's F.Cu leg past
            // facing F.Cu pins beyond its row
            for( size_t k = 0; k < n && ok; ++k )
            {
                if( kind[k] == 1 )
                    ok = ( rights.empty() || rowT[k] < rights.back().first ) && ( lefts.empty() || rowT[k] > lefts.back().first );

                for( size_t m = 0; m < n && ok; ++m )
                {
                    if( kind[k] == 0 && isFar[k] && isB( kind[m] ) && !isFar[m] && std::abs( rowT[k] - rowT[m] ) < viaTrack )
                        ok = false;

                    if( kind[k] == 2 && kind[m] == 3 && rowT[m] < rowT[k] )
                        ok = false;
                }
            }

            if( !ok )
                continue;

            for( bool round : { false, true } )
            {
                for( bool endVias : { false, true } )
                {
                    // a facing pin enters the channel from its own row: only between the rows, with its via past the end
                    if( ( ( round || endVias ) && !channels ) || ( facingChannels && ( round || !endVias ) ) )
                        continue;

                    bool   fits = true;
                    double last[2] = { -1e18, -1e18 };

                    for( size_t k = 0; k < n && fits; ++k )
                    {
                        if( kind[k] != 2 && kind[k] != 3 )
                            continue;

                        // the channels in their rows' order, the other way round outside the far row; with vias past
                        // the part's end the vias set the order
                        const double key = round ? -rowT[k] : rowT[k];
                        fits = endVias || key > last[kind[k] - 2];
                        last[kind[k] - 2] = key;

                        for( double other : farOthers )
                            fits = fits && ( !round || ( kind[k] == 2 ? other > rowT[k] : other < rowT[k] ) );

                        // top channels ahead of the copper they cross on their layer (F.Cu facing pins, or with vias
                        // past the part's end the B.Cu pins), bottom ones after it; on F.Cu up to the slots they
                        // cross a passenger's leg on their side
                        for( size_t m = 0; m < n && fits; ++m )
                        {
                            const bool crossed = endVias ? isB( kind[m] ) : kind[m] == 1;

                            if( crossed && ( kind[k] == 2 ? m < k : m > k ) )
                                fits = false;
                        }

                        if( !endVias )
                            fits = fits && ( kind[k] == 2 ? lefts.empty() : rights.empty() );
                    }

                    if( !fits )
                        continue;

                    for( int push : { 0, 1, 2, 4 } )
                        candidates.push_back( { cost + ( round ? 1 : 0 ) + ( endVias ? 1 : 0 ), kind, round, endVias, push } );
                }
            }
        }

        auto orderOut = [&]()
        {
            nlohmann::json order = nlohmann::json::array();

            for( size_t k = 0; k < n; ++k )
                order.push_back( { { "pad", str( fp->GetReference() ) + "." + str( pins[k].pad->GetNumber() ) }, { "net", netOf( pins[k].pad ) },
                                   { "row", pcbIUScale.IUTomm( rowT[k] ) }, { "far", (bool) isFar[k] }, { "turn", classOf( pins[k] ) } } );

            stacksWhy.back()["order"] = order;
        };

        if( candidates.empty() )
        {
            refuse( "no order on two layers" );
            orderOut();
            continue;
        }

        // the cheapest that has room and whose copper is clear of the board's
        std::stable_sort( candidates.begin(), candidates.end(), []( const STACK_TRY& x, const STACK_TRY& y )
        {
            return x.cost != y.cost ? x.cost < y.cost : x.push < y.push;
        } );
        const char* why = "no room under the body";
        nlohmann::json misses = nlohmann::json::array();   ///< per candidate its pins' kinds (B.Cu, F.Cu, channel top / bottom) and what it hit
        bool        placed = false;

        for( size_t tried = 0; tried < std::min<size_t>( candidates.size(), 4096 ) && !placed; ++tried )
        {
        kind = candidates[tried].kinds;
        const bool   round = candidates[tried].round;
        const bool   endVias = candidates[tried].endVias;
        const double chP = endVias ? viaGap : pitchT;     ///< channel spacing: vias side by side past the end need more
        const double chU = endVias ? viaTrack : pitchT;   ///< between the rows: the vias past the end stand at other rows, a track passes a via
        const double push = candidates[tried].push * pcbIUScale.mmToIU( 0.25 );
        std::map<const PAD*, SLOT> local;

        // under the body: the far B.Cu pins' vias (staggered), then the channels, lower rows outer for the top ones
        std::vector<double> viaU( n, 0 );
        double              lastFarVia = -1e18, lastFarT = -1e18;
        double              farVias = farIn;

        for( size_t k : [&]
             {
                 std::vector<size_t> idx;

                 for( size_t k = 0; k < n; ++k )
                 {
                     if( kind[k] == 0 && isFar[k] )
                         idx.push_back( k );
                 }

                 std::sort( idx.begin(), idx.end(), [&]( size_t x, size_t y ) { return rowT[x] < rowT[y]; } );
                 return idx;
             }() )
        {
            const double base = farIn + clr + viaR;
            const double dt = rowT[k] - lastFarT;
            viaU[k] = dt < viaGap && std::abs( lastFarVia - base ) < 1 ? base + std::sqrt( std::max( 0.0, viaGap * viaGap - dt * dt ) ) + pcbIUScale.mmToIU( 0.05 ) : base;
            lastFarVia = viaU[k];
            lastFarT = rowT[k];
            farVias = std::max( farVias, viaU[k] + viaR );
        }

        // the facing pins' vias under the body (staggered inward), clear of the far ones
        double lastUnderVia = 1e18, lastUnderT = -1e18, faceInner = faceIn;

        for( size_t k = 0; k < n; ++k )
        {
            if( kind[k] != 4 )
                continue;

            const double base = faceIn - clr - viaR;
            const double dt = rowT[k] - lastUnderT;
            viaU[k] = dt < viaGap && std::abs( lastUnderVia - base ) < 1 ? base - std::sqrt( std::max( 0.0, viaGap * viaGap - dt * dt ) ) - pcbIUScale.mmToIU( 0.05 ) : base;
            lastUnderVia = viaU[k];
            lastUnderT = rowT[k];
            faceInner = std::min( faceInner, viaU[k] - viaR );
        }

        if( farVias + clr > faceInner )
            continue;

        std::vector<size_t> tops, bottoms;

        for( size_t k = 0; k < n; ++k )
        {
            if( kind[k] == 2 )
                tops.push_back( k );
            else if( kind[k] == 3 )
                bottoms.push_back( k );
        }

        std::sort( tops.begin(), tops.end(), [&]( size_t x, size_t y ) { return rowT[x] < rowT[y]; } );
        std::sort( bottoms.begin(), bottoms.end(), [&]( size_t x, size_t y ) { return rowT[x] > rowT[y]; } );

        std::vector<double> chanU( n, 0 ), chanT( n, 0 );

        if( round )
        {
            // outside the far row: the pin nearest the part's end innermost, the next ones round it
            double u1 = farOut - clr - halfTrack;

            for( const std::vector<size_t>* group : { &tops, &bottoms } )
            {
                for( size_t g = 0; g < group->size(); ++g )
                {
                    const size_t k = ( *group )[g];
                    chanU[k] = u1 - g * chP;
                    chanT[k] = group == &tops ? tMin - clr - halfTrack - push - g * chP : tMax + clr + halfTrack + push + g * chP;
                }

                u1 -= group->size() * chP;
            }
        }
        else
        {
            // far pins from the far side, the rows nearest the end nearest it; facing pins from the facing side, the
            // same the other way round
            const double edge = endVias ? viaR : halfTrack;   ///< a channel's widest copper: its via past the end, or its track
            double       u0 = std::max( farVias, farIn ) + clr + edge;
            double       u1 = faceInner - clr - edge;

            for( const std::vector<size_t>* group : { &tops, &bottoms } )
            {
                int farG = 0, faceG = 0;

                for( size_t g = 0; g < group->size(); ++g )
                {
                    const size_t k = ( *group )[g];
                    chanU[k] = isFar[k] ? u0 + farG++ * chU : u1 - faceG++ * chU;
                    chanT[k] = group == &tops ? tMin - clr - edge - push - ( group->size() - 1 - g ) * chP
                                              : tMax + clr + edge + push + ( group->size() - 1 - g ) * chP;
                }

                // vias past the end: in the stack's order away from the part's end, the first outermost at the top
                if( endVias )
                {
                    std::vector<size_t> byStack = *group;
                    std::sort( byStack.begin(), byStack.end() );

                    for( size_t g = 0; g < byStack.size(); ++g )
                    {
                        chanT[byStack[g]] = group == &tops ? tMin - clr - edge - push - ( byStack.size() - 1 - g ) * chP
                                                           : tMax + clr + edge + push + g * chP;
                    }
                }

                u0 += farG * chU;
                u1 -= faceG * chU;
            }

            if( endVias ? u0 > u1 + chU + 1 : u0 - chP + edge + clr > u1 + chP - edge - clr )
                continue;
        }

        // past the facing row: its B.Cu pins' vias at their rows (staggered), the sort zone, the F.Cu pins' vias at
        // their slots (staggered), the slots
        double lastFaceVia = -1e18, lastFaceT = -1e18, faceVias = faceOut + push;

        for( size_t k = 0; k < n; ++k )
        {
            if( kind[k] != 0 || isFar[k] )
                continue;

            const double base = faceOut + clr + viaR + push;
            const double dt = rowT[k] - lastFaceT;
            viaU[k] = dt < viaGap && std::abs( lastFaceVia - base ) < 1 ? base + std::sqrt( std::max( 0.0, viaGap * viaGap - dt * dt ) ) + pcbIUScale.mmToIU( 0.05 ) : base;
            lastFaceVia = viaU[k];
            lastFaceT = rowT[k];
            faceVias = std::max( faceVias, viaU[k] + viaR );
        }

        const double spacing = std::max( pitchT, viaTrack );
        double       centre = 0;

        for( size_t k = 0; k < n; ++k )
            centre += kind[k] == 2 || kind[k] == 3 ? chanT[k] : rowT[k];

        centre /= (double) n;
        std::vector<double> slotT( n );

        for( size_t k = 0; k < n; ++k )
            slotT[k] = centre + ( k - ( n - 1 ) / 2.0 ) * spacing;

        const double legs = faceVias + clr + halfTrack;
        const double zone = legs + std::max( lefts.size(), rights.size() ) * pitchT;
        double       jog = 0;
        auto         fromOf = [&]( size_t k ) { return kind[k] == 2 || kind[k] == 3 ? chanT[k] : rowT[k]; };

        for( size_t k = 0; k < n; ++k )
            jog = std::max( jog, std::abs( slotT[k] - fromOf( k ) ) );

        // a jog's start: the pins jogging the same way start one after the other, the one ahead in the jog's
        // direction first, so neighbours' diagonals keep their pitch
        const double        step = pitchT * ( std::sqrt( 2.0 ) - 1.0 ) + pcbIUScale.mmToIU( 0.05 );
        std::vector<double> jogAt( n, zone );
        double              lastJog = zone;
        int                 down = 0, up = 0;

        for( size_t k = n; k-- > 0; )
        {
            if( slotT[k] - fromOf( k ) > 1 )
                jogAt[k] = zone + step * down++;
        }

        for( size_t k = 0; k < n; ++k )
        {
            if( fromOf( k ) - slotT[k] > 1 )
                jogAt[k] = zone + step * up++;

            lastJog = std::max( lastJog, jogAt[k] );
        }

        const double fVias = lastJog + jog + viaR + clr;
        double       column = fVias + viaR + clr + halfTrack;
        bool         staggered = false;

        for( size_t k = 0; k < n; ++k )
        {
            if( kind[k] == 0 || kind[k] == 4 || ( endVias && kind[k] >= 2 ) )
                continue;

            const bool tight = k > 0 && ( kind[k - 1] == 1 || ( !endVias && ( kind[k - 1] == 2 || kind[k - 1] == 3 ) ) ) && !staggered;
            viaU[k] = tight ? fVias + std::sqrt( std::max( 0.0, viaGap * viaGap - spacing * spacing ) ) + pcbIUScale.mmToIU( 0.05 ) : fVias;
            staggered = tight;
            column = std::max( column, viaU[k] + viaR + clr + halfTrack );
        }

        for( size_t k = 0; k < n; ++k )
        {
            const double row = rowT[k];
            SLOT         sl;
            sl.slot = at( column, slotT[k] );

            auto jogTo = [&]( std::vector<VECTOR2I>& aPts, double aFromT )
            {
                aPts.push_back( at( jogAt[k], aFromT ) );

                if( std::abs( slotT[k] - aFromT ) > 1 )
                    aPts.push_back( at( jogAt[k] + std::abs( slotT[k] - aFromT ), slotT[k] ) );
            };

            if( kind[k] == 0 || kind[k] == 4 )
            {
                sl.via = at( viaU[k], row );
                jogTo( sl.bends, row );
            }
            else if( kind[k] == 1 )
            {
                jogTo( sl.padBends, row );
                sl.via = at( viaU[k], slotT[k] );
            }
            else if( endVias )
            {
                sl.padBends.push_back( at( chanU[k], row ) );
                sl.via = at( chanU[k], chanT[k] );
                jogTo( sl.bends, chanT[k] );
            }
            else
            {
                sl.padBends.push_back( at( chanU[k], row ) );
                sl.padBends.push_back( at( chanU[k], chanT[k] ) );
                jogTo( sl.padBends, chanT[k] );
                sl.via = at( viaU[k], slotT[k] );
            }

            local[pins[k].pad] = sl;
        }

        // the passengers' legs: out at their rows past the facing vias, then across beyond the part's end, the one
        // nearest that end turning first
        std::map<const PAD*, SLOT> legsOf;

        for( const auto& [group, end] : { std::make_pair( &lefts, tMin - clr - halfTrack ), std::make_pair( &rights, tMax + clr + halfTrack ) } )
        {
            for( size_t g = 0; g < group->size(); ++g )
            {
                const auto& [row, pad] = ( *group )[g];
                SLOT        sl;
                sl.padBends.push_back( at( legs + g * pitchT, row ) );
                sl.via = at( legs + g * pitchT, end );
                sl.slot = sl.via;
                legsOf[pad] = sl;
            }
        }

        // a leg clear of the stack's F.Cu copper and vias
        auto legHits = [&]( const PAD* aPad, const SLOT& aLeg )
        {
            const SEG    out( aPad->GetPosition(), aLeg.padBends[0] ), turn( aLeg.padBends[0], aLeg.via );
            const double track = aRules.trackWidth + clr;

            for( const auto& [pad, sl] : local )
            {
                std::vector<VECTOR2I> front = { pad->GetPosition() };
                front.insert( front.end(), sl.padBends.begin(), sl.padBends.end() );
                front.push_back( sl.via );

                for( const SEG& leg : { out, turn } )
                {
                    if( leg.Distance( sl.via ) < viaR + clr + halfTrack )
                        return true;

                    for( size_t k = 1; k < front.size(); ++k )
                    {
                        if( leg.Distance( SEG( front[k - 1], front[k] ) ) < track )
                            return true;
                    }
                }
            }

            return false;
        };

        bool           clear = true;
        nlohmann::json missed;   ///< the pin whose copper hit the board's: its copper, the via it hit

        for( auto& [pad, sl] : local )
        {
            if( !clear || clearOf( pad, sl ) )
                continue;

            const std::string hit = lastHit;
            const auto        hitVia = lastVia;
            std::optional<SLOT> moved = repaired( pad, sl, local );
            clear = moved.has_value();

            if( moved )
            {
                sl = *moved;
                continue;
            }

            lastHit = hit;
            nlohmann::json pts = nlohmann::json::array();

            for( const VECTOR2I& p : sl.padBends )
                pts.push_back( mmOf( p ) );

            missed = { { "pad_bends_mm", pts }, { "via_mm", mmOf( sl.via ) }, { "slot_mm", mmOf( sl.slot ) }, { "repair", repairMiss } };
            pts = nlohmann::json::array();

            for( const VECTOR2I& p : sl.bends )
                pts.push_back( mmOf( p ) );

            missed["bends_mm"] = pts;

            if( hitVia )
                missed["hit_via_mm"] = mmOf( hitVia->first );
        }

        for( const auto& [pad, sl] : legsOf )
        {
            if( clear && clearOf( pad, sl, false ) && legHits( pad, sl ) )
                lastHit = str( fp->GetReference() ) + "." + str( pad->GetNumber() ) + ": the stack's own copper";

            clear = clear && clearOf( pad, sl, false ) && !legHits( pad, sl );
        }

        if( !clear )
        {
            why = "its copper would hit the board's";
            std::string kinds;

            for( int kk : kind )
                kinds += ( round ? "BFtuV" : "BFTUV" )[kk];

            if( endVias )
                kinds += "*";

            if( misses.size() < 32 && push == 0 )
                misses.push_back( { { "kinds", kinds }, { "push_mm", pcbIUScale.IUTomm( push ) }, { "hit", lastHit }, { "copper", missed } } );
            continue;
        }

        slots.insert( local.begin(), local.end() );
        placed = true;

        if( legsOf.empty() )
            continue;

        nlohmann::json riders = nlohmann::json::array(), ids = nlohmann::json::array();
        std::set<size_t> links;

        for( const STACK_PIN& sp : pins )
            links.insert( plans[sp.link].link->id );

        for( size_t id : links )
            ids.push_back( id );

        for( const auto& [pad, sl] : legsOf )
        {
            riders.push_back( { { "pad", str( fp->GetReference() ) + "." + str( pad->GetNumber() ) }, { "net", netOf( pad ) },
                                { "pad_bends_mm", nlohmann::json::array( { mmOf( sl.padBends[0] ) } ) }, { "slot_mm", mmOf( sl.slot ) },
                                { "layer", "F.Cu" } } );
        }

        passengersOut.push_back( { { "stack", where }, { "links", ids }, { "pins", riders } } );
        }

        if( !placed )
        {
            refuse( why );
            orderOut();

            if( !misses.empty() )
                stacksWhy.back()["tried"] = misses;
        }
    }

    nlohmann::json                                  linksOut = nlohmann::json::array();
    std::map<const FOOTPRINT*, std::map<int, nlohmann::json>> stacks;   ///< part -> side -> its link ends

    auto mm = []( const VECTOR2I& p ) { return nlohmann::json::array( { pcbIUScale.IUTomm( p.x ), pcbIUScale.IUTomm( p.y ) } ); };

    // the centrelines laid so far, each with its band's half width: the detours first, then the straight links'
    // steps where they keep clear of them
    std::vector<std::pair<std::vector<VECTOR2I>, double>> laid;
    const double laneSpacing = std::max( pitchT, viaTrack );

    auto clearOfLaid = [&]( const std::vector<VECTOR2I>& aLine, double aHalf )
    {
        for( const auto& [other, half] : laid )
        {
            for( size_t k = 1; k < aLine.size(); ++k )
            {
                for( size_t m = 1; m < other.size(); ++m )
                {
                    if( SEG( aLine[k - 1], aLine[k] ).Distance( SEG( other[m - 1], other[m] ) ) < aHalf + half + clr )
                        return false;
                }
            }
        }

        return true;
    };

    std::vector<size_t> lineOrder;

    // the detours inner first: the shorter ones, outer tiers past them
    for( size_t i : order )
    {
        if( plans[i].side >= 0 )
            lineOrder.push_back( i );
    }

    // of the same length, the one starting farther from the side is inner: the outer turns out first and in last
    auto startAcross = [&]( size_t i )
    {
        const PLAN& plan = plans[i];
        double      sum = 0;
        int         count = 0;

        for( const std::string& net : plan.link->nets )
        {
            for( PAD* pad : netPads[net] )
            {
                if( pad->GetParentFootprint() != plan.a )
                    continue;

                auto it = slots.find( pad );
                sum += dot( VECTOR2D( it != slots.end() ? it->second.slot : pad->GetPosition() ), unitOf( plan.side ) );
                count++;
            }
        }

        return count ? sum / count : 0.0;
    };

    for( size_t g = 0; g < lineOrder.size(); )
    {
        size_t e = g + 1;

        while( e < lineOrder.size() && plans[lineOrder[e]].side == plans[lineOrder[g]].side
               && std::abs( plans[lineOrder[e]].length - plans[lineOrder[g]].length ) <= pcbIUScale.mmToIU( 0.5 ) )
            ++e;

        std::stable_sort( lineOrder.begin() + g, lineOrder.begin() + e, [&]( size_t x, size_t y ) { return startAcross( x ) < startAcross( y ); } );
        g = e;
    }

    struct BAND
    {
        int side;
        int from, to;   ///< along the run's axis
        int at;         ///< across: the band's outer edge
        int low, high;  ///< along: its turns out of and into the band, the outer edges of the turning ribbons
    };

    std::vector<BAND> bands;

    for( size_t i = 0; i < plans.size(); ++i )
    {
        if( plans[i].side < 0 )
            lineOrder.push_back( i );
    }

    for( size_t i : lineOrder )
    {
        const PLAN&    plan = plans[i];
        const DECIDED& d = decided[i];
        const int      fromA = d.from[0], fromB = d.from[1];
        const char*    layer = d.direct ? "F.Cu" : "B.Cu";
        nlohmann::json entries = nlohmann::json::array();
        bool           covered = true;
        std::string    whyNot;
        std::vector<VECTOR2I> ends[2];
        std::vector<std::pair<double, std::string>> across[2];   ///< per end its pins left to right on the way

        // on F.Cu a pin entered from outside its row has its slot just past its pad, a finger past its inner end (the
        // one toward the board's middle); the way out of the pad with it
        const VECTOR2D middle( aBoard->GetBoardEdgesBoundingBox().GetCenter() );

        auto directSlot = [&]( const PAD* aPad, int aFrom ) -> std::optional<std::pair<VECTOR2I, VECTOR2D>>
        {
            const BOX2I    bb = aPad->GetBoundingBox();
            const VECTOR2D at( aPad->GetPosition() );
            VECTOR2D       u = unitOf( aFrom );

            if( isFinger( aPad, outline ) )
            {
                const bool wide = bb.GetWidth() > bb.GetHeight();
                u = wide ? VECTOR2D( middle.x >= at.x ? 1 : -1, 0 ) : VECTOR2D( 0, middle.y >= at.y ? 1 : -1 );
            }
            else if( padSide( aPad ) != aFrom )
            {
                return std::nullopt;
            }

            const double reach = ( u.x != 0 ? bb.GetWidth() : bb.GetHeight() ) / 2.0 + clr + halfTrack;
            return std::make_pair( VECTOR2I( at + u * reach ), u );
        };

        for( const std::string& net : plan.link->nets )
        {
            for( PAD* pad : netPads[net] )
            {
                FOOTPRINT* fp = pad->GetParentFootprint();

                if( fp != plan.a && fp != plan.b )
                    continue;

                const int      from = fp == plan.a ? fromA : fromB;
                nlohmann::json e = { { "pad", str( fp->GetReference() ) + "." + str( pad->GetNumber() ) }, { "net", net },
                                     { "entry", d.direct ? entryOf( pad, from, outline ) : viaEntryOf( pad, from ) } };

                std::optional<OWN_END> own = d.direct ? copperEnd( pad ) : std::nullopt;

                if( own && !own->other && !own->on.Contains( F_Cu ) )
                {
                    covered = false;
                    whyNot = str( fp->GetReference() ) + "." + str( pad->GetNumber() ) + ": its own copper ends on B.Cu";
                }
                else if( own )
                {
                    e["entry"] = own->other ? "routed" : "ready";
                    e["slot_mm"] = mm( own->at );

                    if( !own->other )
                        ends[fp == plan.a ? 0 : 1].push_back( own->at );
                }
                else if( d.direct )
                {
                    if( auto at = directSlot( pad, from ) )
                    {
                        e["slot_mm"] = mm( at->first );
                        ends[fp == plan.a ? 0 : 1].push_back( at->first );

                        // left of the way: out of a, into b
                        const VECTOR2D way = fp == plan.a ? at->second : at->second * -1.0;
                        across[fp == plan.a ? 0 : 1].emplace_back( VECTOR2D( at->first ).Dot( VECTOR2D( way.y, -way.x ) ), net );
                    }
                    else
                    {
                        covered = false;
                        whyNot = str( fp->GetReference() ) + "." + str( pad->GetNumber() ) + " not entered from outside its row";
                    }
                }
                else if( offLayer.count( pad ) )
                {
                    const SLOT& sl = slots[pad];

                    if( readyVias.count( pad ) )
                    {
                        e["copper_end_mm"] = mm( sl.padBends[0] );
                        e["entry"] = "ready";
                        e["slot_mm"] = mm( sl.slot );
                        e["via_mm"] = mm( sl.via );
                        ends[fp == plan.a ? 0 : 1].push_back( sl.slot );
                    }
                    else
                    {
                        covered = false;
                        whyNot = readyMiss[pad];
                    }
                }
                else if( auto r = ready.find( pad ); r != ready.end() )
                {
                    // its own copper is its way out: the route goes on from where it ends
                    e["entry"] = r->second ? "routed" : "ready";
                    e["slot_mm"] = mm( slots[pad].slot );

                    if( !r->second )
                        ends[fp == plan.a ? 0 : 1].push_back( slots[pad].slot );
                }
                else if( auto it = slots.find( pad ); it != slots.end() )
                {
                    e["pad_bends_mm"] = nlohmann::json::array();

                    for( const VECTOR2I& b : it->second.padBends )
                        e["pad_bends_mm"].push_back( mm( b ) );

                    e["via_mm"] = mm( it->second.via );
                    e["slot_mm"] = mm( it->second.slot );
                    e["bends_mm"] = nlohmann::json::array();

                    for( const VECTOR2I& b : it->second.bends )
                        e["bends_mm"].push_back( mm( b ) );

                    ends[fp == plan.a ? 0 : 1].push_back( it->second.slot );
                }
                else
                {
                    covered = false;
                    whyNot = str( fp->GetReference() ) + "." + str( pad->GetNumber() ) + " has no slot in a stack";
                }

                entries.push_back( e );
            }
        }

        // on one layer the ribbon keeps its order: the same left to right at both ends
        if( d.direct && covered )
        {
            // the pins with a slot at both ends
            std::set<std::string> both[2];

            for( int end = 0; end < 2; ++end )
            {
                for( const auto& [t, net] : across[end] )
                    both[end].insert( net );
            }

            for( int end = 0; end < 2; ++end )
            {
                std::erase_if( across[end], [&]( const auto& x ) { return !both[1 - end].count( x.second ); } );
                std::sort( across[end].begin(), across[end].end(), []( const auto& x, const auto& y ) { return x.first > y.first; } );
            }

            covered = across[0].size() == across[1].size()
                      && std::equal( across[0].begin(), across[0].end(), across[1].begin(),
                                     []( const auto& x, const auto& y ) { return x.second == y.second; } );

            if( !covered )
                whyNot = "its pins are in other orders at its two ends";
        }

        nlohmann::json round = nlohmann::json::array();

        for( FOOTPRINT* fp : plan.between )
            round.push_back( str( fp->GetReference() ) );

        nlohmann::json row = { { "id", plan.link->id }, { "name", plan.link->name }, { "nets", plan.link->nets },
                               { "a", str( plan.a->GetReference() ) }, { "b", str( plan.b->GetReference() ) },
                               { "runs", SIDES[plan.runs] }, { "route", plan.side < 0 ? "straight" : "detour" },
                               { plan.side < 0 ? "under" : "round", round },
                               { "leaves_a", SIDES[fromA] }, { "leaves_b", SIDES[fromB] }, { "layer", layer }, { "entries", entries },
                               { "covered", covered } };

        if( plan.side >= 0 )
            row["side"] = SIDES[plan.side];

        if( !covered && !whyNot.empty() )
            row["not_covered"] = whyNot;

        // its centreline: from the middle of its slots at a to theirs at b, along its run; a detour out to its
        // band beside the parts between
        if( covered && !ends[0].empty() && !ends[1].empty() )
        {
            auto mid = []( const std::vector<VECTOR2I>& aPts )
            {
                VECTOR2D sum( 0, 0 );

                for( const VECTOR2I& p : aPts )
                    sum += VECTOR2D( p );

                return VECTOR2I( sum / (double) aPts.size() );
            };

            const VECTOR2I pa = mid( ends[0] ), pb = mid( ends[1] );
            std::vector<VECTOR2I> line = { pa };

            if( plan.side >= 0 )
            {
                BOX2I span;
                bool  first = true;

                for( FOOTPRINT* fp : plan.between )
                {
                    if( first )
                        span = fp->GetBoundingBox( false );
                    else
                        span.Merge( fp->GetBoundingBox( false ) );

                    first = false;
                }

                // with the other links' copper at those parts: their pins' bends, vias and slots
                for( const auto& [pad, sl] : slots )
                {
                    if( std::find( plan.between.begin(), plan.between.end(), pad->GetParentFootprint() ) == plan.between.end() )
                        continue;

                    const int grow = KiROUND( viaR + clr );

                    for( const std::vector<VECTOR2I>* pts : { &sl.padBends, &sl.bends } )
                    {
                        for( const VECTOR2I& pt : *pts )
                            span.Merge( BOX2I( pt ).Inflate( grow ) );
                    }

                    span.Merge( BOX2I( sl.via ).Inflate( grow ) );
                    span.Merge( BOX2I( sl.slot ).Inflate( grow ) );
                }

                const VECTOR2D us = unitOf( plan.side );
                const int      half = KiROUND( ( ends[0].size() - 1 ) / 2.0 * laneSpacing + halfTrack );
                const int      off = plan.depth / 2 + clr;
                const int      ua = axisOf( plan.runs );
                const double   sign = us.x + us.y;   ///< +1 when the side lies toward larger coordinates
                int            bandAt = us.y < 0 ? span.GetTop() - off : us.y > 0 ? span.GetBottom() + off
                                                                         : us.x < 0 ? span.GetLeft() - off : span.GetRight() + off;

                // outside the bands laid on that side along the same stretch
                const int from = ua == 0 ? std::min( pa.x, pb.x ) : std::min( pa.y, pb.y );
                const int to = ua == 0 ? std::max( pa.x, pb.x ) : std::max( pa.y, pb.y );

                // past the ends' slots that leave toward the side
                for( int end = 0; end < 2; ++end )
                {
                    if( ( end == 0 ? fromA : fromB ) != plan.side )
                        continue;

                    for( const VECTOR2I& p : ends[end] )
                    {
                        const int c = us.x != 0 ? p.x : p.y;
                        bandAt = sign > 0 ? std::max<int>( bandAt, c + half + clr ) : std::min<int>( bandAt, c - half - clr );
                    }
                }

                int lowMax = 1 << 30, highMin = -( 1 << 30 );

                for( const BAND& b : bands )
                {
                    if( b.side == plan.side && b.from < to && from < b.to )
                    {
                        bandAt = sign > 0 ? std::max<int>( bandAt, b.at + half + clr ) : std::min<int>( bandAt, b.at - half - clr );
                        lowMax = std::min<int>( lowMax, b.low - half - clr );
                        highMin = std::max<int>( highMin, b.high + half + clr );
                    }
                }

                auto nest = [&]( int aLow, int aHigh )
                {
                    bands.push_back( { plan.side, from, to, KiROUND( bandAt + sign * half ), std::min( aLow, lowMax ) - half,
                                       std::max( aHigh, highMin ) + half } );
                    return std::make_pair( std::min( aLow, lowMax ), std::max( aHigh, highMin ) );
                };

                if( ua == 0 )
                {
                    const int x0 = plan.runs == 1 ? span.GetLeft() - plan.depth : span.GetRight() + plan.depth;
                    const int x1 = plan.runs == 1 ? span.GetRight() + plan.depth : span.GetLeft() - plan.depth;
                    const int turn = half + KiROUND( clr );   ///< the ribbon turns past its slots
                    int       ta = plan.runs == 1 ? std::max( x0, pa.x + turn ) : std::min( x0, pa.x - turn );
                    int       tb = plan.runs == 1 ? std::min( x1, pb.x - turn ) : std::max( x1, pb.x + turn );
                    const auto [lo, hi] = nest( std::min( fromA == plan.side ? pa.x : ta, fromB == plan.side ? pb.x : tb ),
                                                std::max( fromA == plan.side ? pa.x : ta, fromB == plan.side ? pb.x : tb ) );
                    ( plan.runs == 1 ? ta : tb ) = lo;
                    ( plan.runs == 1 ? tb : ta ) = hi;
                    line.push_back( VECTOR2I( fromA == plan.side ? pa.x : ta, pa.y ) );
                    line.push_back( VECTOR2I( line.back().x, bandAt ) );
                    line.push_back( VECTOR2I( fromB == plan.side ? pb.x : tb, bandAt ) );
                    line.push_back( VECTOR2I( line.back().x, pb.y ) );
                }
                else
                {
                    const int y0 = plan.runs == 2 ? span.GetTop() - plan.depth : span.GetBottom() + plan.depth;
                    const int y1 = plan.runs == 2 ? span.GetBottom() + plan.depth : span.GetTop() - plan.depth;
                    const int turn = half + KiROUND( clr );
                    int       ta = plan.runs == 2 ? std::max( y0, pa.y + turn ) : std::min( y0, pa.y - turn );
                    int       tb = plan.runs == 2 ? std::min( y1, pb.y - turn ) : std::max( y1, pb.y + turn );
                    const auto [lo, hi] = nest( std::min( fromA == plan.side ? pa.y : ta, fromB == plan.side ? pb.y : tb ),
                                                std::max( fromA == plan.side ? pa.y : ta, fromB == plan.side ? pb.y : tb ) );
                    ( plan.runs == 2 ? ta : tb ) = lo;
                    ( plan.runs == 2 ? tb : ta ) = hi;
                    line.push_back( VECTOR2I( pa.x, fromA == plan.side ? pa.y : ta ) );
                    line.push_back( VECTOR2I( bandAt, line.back().y ) );
                    line.push_back( VECTOR2I( bandAt, fromB == plan.side ? pb.y : tb ) );
                    line.push_back( VECTOR2I( pb.x, line.back().y ) );
                }
            }
            else if( axisOf( plan.runs ) == 0 ? pa.y != pb.y : pa.x != pb.x )
            {
                // the step from a's offset to b's: half way, else nearer an end where it keeps clear of the
                // lines laid
                const bool along = axisOf( plan.runs ) == 0;
                std::vector<VECTOR2I> best;

                for( double f : { 0.5, 0.35, 0.65, 0.2, 0.8, 0.1, 0.9 } )
                {
                    std::vector<VECTOR2I> trial = { pa };
                    const int             at = KiROUND( along ? pa.x + ( pb.x - pa.x ) * f : pa.y + ( pb.y - pa.y ) * f );
                    trial.push_back( along ? VECTOR2I( at, pa.y ) : VECTOR2I( pa.x, at ) );
                    trial.push_back( along ? VECTOR2I( at, pb.y ) : VECTOR2I( pb.x, at ) );
                    trial.push_back( pb );

                    if( best.empty() )
                        best = trial;

                    if( clearOfLaid( trial, ( ends[0].size() - 1 ) / 2.0 * laneSpacing + halfTrack ) )
                    {
                        best = trial;
                        break;
                    }
                }

                line.assign( best.begin(), best.end() - 1 );
            }

            line.push_back( pb );
            line.erase( std::unique( line.begin(), line.end() ), line.end() );

            // the ribbon clear of the rule areas without tracks on its layer
            const PCB_LAYER_ID onLayer = d.direct ? F_Cu : B_Cu;
            const int          ribbon = KiROUND( ( ends[0].size() - 1 ) * laneSpacing + aRules.trackWidth );
            std::string        blocked;

            for( size_t k = 1; k < line.size() && blocked.empty(); ++k )
            {
                const SHAPE_SEGMENT seg( line[k - 1], line[k], ribbon );

                for( ZONE* zone : aBoard->Zones() )
                {
                    if( zone->GetIsRuleArea() && zone->GetDoNotAllowTracks() && zone->IsOnLayer( onLayer ) && zone->Outline()->Collide( &seg, 0 ) )
                    {
                        blocked = zone->GetZoneName().ToStdString();
                        break;
                    }
                }
            }

            if( !blocked.empty() )
            {
                row["covered"] = false;
                row["not_covered"] = "its line crosses the rule area " + blocked;
            }
            else
            {
                laid.emplace_back( line, ( ends[0].size() - 1 ) / 2.0 * laneSpacing + halfTrack );
                row["line_mm"] = nlohmann::json::array();

                for( const VECTOR2I& p : line )
                    row["line_mm"].push_back( mm( p ) );
            }
        }
        else
        {
            row["covered"] = false;

            if( covered )
                row["not_covered"] = "its pins at one end are all routed already";
        }

        stacks[plan.a][fromA].push_back( linksOut.size() );
        stacks[plan.b][fromB].push_back( linksOut.size() );
        linksOut.push_back( row );
    }

    nlohmann::json partsOut = nlohmann::json::array();

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        auto it = stacks.find( fp );

        if( it == stacks.end() )
            continue;

        nlohmann::json sides = nlohmann::json::object();

        for( const auto& [side, ends] : it->second )
            sides[SIDES[side]] = ends;

        partsOut.push_back( { { "ref", str( fp->GetReference() ) }, { "signals", parts[fp].signals },
                              { "lanes_left", { parts[fp].lanes[0], parts[fp].lanes[1] } }, { "stacks", sides } } );
    }

    return { { "links", linksOut }, { "parts", partsOut }, { "stacks", stacksWhy }, { "passengers", passengersOut } };
}
