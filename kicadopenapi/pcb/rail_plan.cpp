#include "rail_plan.h"
#include "kopenapi_pcb.h"

#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <drc/drc_engine.h>
#include <footprint.h>
#include <netclass.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <project/net_settings.h>
#include <zone.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <limits>
#include <tuple>


std::optional<VECTOR2I> outwardOf( const PAD* aPad, const BOX2I& aCentres )
{
    const VECTOR2I c = aPad->GetPosition();
    const BOX2I    pb = aPad->GetBoundingBox();
    const bool     onX = c.x == aCentres.GetLeft() || c.x == aCentres.GetRight();
    const bool     onY = c.y == aCentres.GetTop() || c.y == aCentres.GetBottom();

    if( onX && ( !onY || pb.GetWidth() >= pb.GetHeight() ) )
        return VECTOR2I( c.x == aCentres.GetLeft() ? -1 : 1, 0 );

    if( onY )
        return VECTOR2I( 0, c.y == aCentres.GetTop() ? -1 : 1 );

    return std::nullopt;
}


std::optional<UNDER_PART> underPartArea( BOARD* aBoard, FOOTPRINT* aFp )
{
    if( aFp->Pads().size() < 8 )
        return std::nullopt;

    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    std::shared_ptr<NETCLASS>    defaultClass = bds.m_NetSettings ? bds.m_NetSettings->GetDefaultNetclass() : nullptr;
    const int clearance = std::max( { bds.m_MinClearance, defaultClass && defaultClass->HasClearance() ? defaultClass->GetClearance() : 0, 0 } );

    BOX2I centres;
    int   padLength = 0;
    bool  first = true;

    for( PAD* pad : aFp->Pads() )
    {
        if( first )
            centres = BOX2I( pad->GetPosition(), VECTOR2I( 0, 0 ) );
        else
            centres.Merge( pad->GetPosition() );

        first = false;
    }

    // the pad rows: the pads on the box of the centres; their length sets how far in the ring starts
    for( PAD* pad : aFp->Pads() )
    {
        const VECTOR2I c = pad->GetPosition();

        if( c.x == centres.GetLeft() || c.x == centres.GetRight() || c.y == centres.GetTop() || c.y == centres.GetBottom() )
        {
            const BOX2I pb = pad->GetBoundingBox();
            padLength = std::max( { padLength, (int) pb.GetWidth(), (int) pb.GetHeight() } );
        }
    }

    BOX2I inner = centres;
    inner.Inflate( -( padLength / 2 + pcbIUScale.mmToIU( 0.1 ) ) );

    if( inner.GetWidth() <= 0 || inner.GetHeight() <= 0 )
        return std::nullopt;

    // pads on all four sides (a quad package); two rows (SOIC, headers) leave room between
    int sides[4] = { 0, 0, 0, 0 };

    for( PAD* pad : aFp->Pads() )
    {
        const VECTOR2I c = pad->GetPosition();
        const bool     spanX = c.x >= inner.GetLeft() && c.x <= inner.GetRight();
        const bool     spanY = c.y >= inner.GetTop() && c.y <= inner.GetBottom();
        sides[0] += c.x < inner.GetLeft() && spanY;
        sides[1] += c.x > inner.GetRight() && spanY;
        sides[2] += c.y < inner.GetTop() && spanX;
        sides[3] += c.y > inner.GetBottom() && spanX;
    }

    if( std::any_of( std::begin( sides ), std::end( sides ), []( int n ) { return n < 2; } ) )
        return std::nullopt;

    // pads inside: an exposed pad (one or two numbers, its paste / thermal pieces) or an array
    std::vector<PAD*>  insidePads;
    std::set<wxString> insideNumbers;

    for( PAD* pad : aFp->Pads() )
    {
        if( inner.Contains( pad->GetPosition() ) )
        {
            insidePads.push_back( pad );
            insideNumbers.insert( pad->GetNumber() );
        }
    }

    if( insideNumbers.size() > 2 )
        return std::nullopt;

    SHAPE_POLY_SET area;
    area.NewOutline();
    area.Append( inner.GetLeft(), inner.GetTop() );
    area.Append( inner.GetRight(), inner.GetTop() );
    area.Append( inner.GetRight(), inner.GetBottom() );
    area.Append( inner.GetLeft(), inner.GetBottom() );

    if( !insidePads.empty() )
    {
        SHAPE_POLY_SET open;

        for( PAD* pad : insidePads )
        {
            for( PCB_LAYER_ID l : pad->GetLayerSet().CuStack() )
            {
                if( l == ( aFp->GetLayer() == B_Cu ? B_Cu : F_Cu ) )
                    pad->TransformShapeToPolygon( open, l, clearance, ARC_LOW_DEF, ERROR_OUTSIDE );
            }
        }

        BOX2I openBox = open.BBox();
        openBox.Inflate( clearance );
        SHAPE_POLY_SET hole;
        hole.NewOutline();
        hole.Append( openBox.GetLeft(), openBox.GetTop() );
        hole.Append( openBox.GetRight(), openBox.GetTop() );
        hole.Append( openBox.GetRight(), openBox.GetBottom() );
        hole.Append( openBox.GetLeft(), openBox.GetBottom() );
        area.BooleanSubtract( hole );

        if( area.OutlineCount() == 0 )
            return std::nullopt;
    }

    return UNDER_PART{ aFp, aFp->GetLayer() == B_Cu ? B_Cu : F_Cu, std::move( area ), !insidePads.empty() };
}


std::vector<UNDER_PART> underPartAreas( BOARD* aBoard )
{
    std::vector<UNDER_PART> out;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( std::optional<UNDER_PART> u = underPartArea( aBoard, fp ) )
            out.push_back( std::move( *u ) );
    }

    return out;
}


bool boardMinVia( BOARD* aBoard, NETINFO_ITEM* aNet, int aClassDia, int aClassDrill, int& aDia, int& aDrill )
{
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    int                          drill = bds.m_MinThroughDrill;
    int                          ring = bds.m_ViasMinAnnularWidth;
    int                          dia = bds.m_ViasMinSize;

    if( std::shared_ptr<DRC_ENGINE> drc = bds.m_DRCEngine )
    {
        PCB_VIA probe( aBoard );
        probe.SetFlags( ROUTER_TRANSIENT );
        probe.SetViaType( VIATYPE::THROUGH );
        probe.SetLayerPair( F_Cu, B_Cu );
        probe.SetNet( aNet );

        auto minOf = [&]( DRC_CONSTRAINT_T aType, int aFloor )
        {
            DRC_CONSTRAINT c = drc->EvalRules( aType, &probe, nullptr, F_Cu );
            return c.GetValue().HasMin() ? std::max( c.GetValue().Min(), aFloor ) : aFloor;
        };

        drill = minOf( HOLE_SIZE_CONSTRAINT, drill );
        ring = minOf( ANNULAR_WIDTH_CONSTRAINT, ring );
        dia = minOf( VIA_DIAMETER_CONSTRAINT, dia );
    }

    dia = std::max( dia, drill + 2 * ring );

    if( drill > 0 && dia > drill && dia < aClassDia )
    {
        aDia = dia;
        aDrill = std::min( drill, aClassDrill );
        return true;
    }

    return false;
}


RAIL_PLAN_OPTIONS railPlanOptions( BOARD* aBoard, NETINFO_ITEM* aNet, const FOOTPRINT* aFp )
{
    RAIL_PLAN_OPTIONS       options;
    const KOPENAPI_RULE_VIA via = KopenapiRuleVia( aBoard, aNet ? aNet->GetNetCode() : 0, aFp ? aFp->GetPosition() : VECTOR2I(),
                                                   aFp && aFp->GetLayer() == B_Cu ? B_Cu : F_Cu );

    options.classViaDia = via.dia;
    options.classViaDrill = via.drill;

    if( aNet )
        boardMinVia( aBoard, aNet, options.classViaDia, options.classViaDrill, options.minViaDia, options.minViaDrill );

    return options;
}


namespace
{

/// The via joined to aPad directly or by its stub, or none
PCB_VIA* viaOfPin( CONNECTIVITY_DATA& aConn, const PAD* aPad )
{
    for( BOARD_CONNECTED_ITEM* c : aConn.GetConnectedItems( aPad ) )
    {
        if( c->Type() == PCB_VIA_T )
            return static_cast<PCB_VIA*>( c );

        if( c->Type() == PCB_TRACE_T || c->Type() == PCB_ARC_T )
        {
            for( BOARD_CONNECTED_ITEM* o : aConn.GetConnectedItems( c ) )
            {
                if( o->Type() == PCB_VIA_T )
                    return static_cast<PCB_VIA*>( o );
            }
        }
    }

    return nullptr;
}

} // namespace


std::vector<RAIL_PIN_PLAN> planRowRailVias( BOARD* aBoard, FOOTPRINT* aFp, const RAIL_PLAN_OPTIONS& aOptions )
{
    std::vector<RAIL_PIN_PLAN> out;

    if( !aBoard || !aFp || aFp->Pads().size() < 4 )
        return out;

    const std::optional<UNDER_PART> ring = underPartArea( aBoard, aFp );

    if( !ring )
        return out;

    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    const int                    minClearance = bds.m_MinClearance;
    const int                    holeClearance = bds.m_HoleClearance;
    const int                    holeToHole = bds.m_HoleToHoleMin;
    const int                    stubMin = bds.m_TrackMinWidth;

    // the part's outer rows: fine-pitch pads (a neighbour within 0.65 mm) and their way out
    BOX2I centres( aFp->Pads().front()->GetPosition(), VECTOR2I( 0, 0 ) );

    for( PAD* pad : aFp->Pads() )
        centres.Merge( pad->GetPosition() );

    std::map<const PAD*, VECTOR2I> outward;
    std::set<const PAD*>           fine;

    for( PAD* pad : aFp->Pads() )
    {
        double nearest = std::numeric_limits<double>::max();

        for( PAD* other : aFp->Pads() )
        {
            if( other != pad && other->GetPosition() != pad->GetPosition() )
                nearest = std::min( nearest, (double) ( other->GetPosition() - pad->GetPosition() ).EuclideanNorm() );
        }

        if( nearest <= pcbIUScale.mmToIU( 0.65 ) )
            fine.insert( pad );

        if( std::optional<VECTOR2I> dir = outwardOf( pad, centres ) )
            outward[pad] = *dir;
    }

    // the exposed pad: the first of its pieces by address, as the plan has always taken it
    const PAD* ep = nullptr;

    for( PAD* pad : aFp->Pads() )
    {
        if( ring->exposedPad && !outward.count( pad ) && pad->IsOnLayer( ring->layer ) && ( !ep || std::less<const PAD*>()( pad, ep ) ) )
            ep = pad;
    }

    std::set<int> poured;

    for( ZONE* z : aBoard->Zones() )
    {
        if( !z->GetIsRuleArea() )
            poured.insert( z->GetNetCode() );
    }

    std::vector<const ZONE*> ruleAreas;

    for( ZONE* z : aBoard->Zones() )
    {
        if( z->GetIsRuleArea() && ( z->GetDoNotAllowVias() || z->GetDoNotAllowTracks() ) )
            ruleAreas.push_back( z );
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( ZONE* z : fp->Zones() )
        {
            if( z->GetIsRuleArea() && ( z->GetDoNotAllowVias() || z->GetDoNotAllowTracks() ) )
                ruleAreas.push_back( z );
        }
    }

    auto clearOf = [&]( const PAD* aPad )
    {
        NETCLASS* nc = aPad->GetNet() ? aPad->GetNet()->GetNetClass() : nullptr;
        return std::max( minClearance, nc && nc->HasClearance() ? nc->GetClearance() : 0 );
    };

    // the centre distance two vias of other nets keep; a via's centre to a stub's axis
    auto viaApart = [&]( int aDia, int aDrill, int bDia, int bDrill, int aClear )
    {
        return std::max( { ( aDia + bDia ) / 2 + aClear, aDrill / 2 + holeClearance + bDia / 2, bDrill / 2 + holeClearance + aDia / 2,
                           ( aDrill + bDrill ) / 2 + holeToHole } );
    };

    auto stubApart = [&]( int aDia, int aDrill, int aClear )
    {
        return stubMin / 2 + std::max( aDia / 2 + aClear, aDrill / 2 + holeClearance );
    };

    auto firstClass = [&]( const PAD* aPad )
    {
        return poured.count( aPad->GetNetCode() ) || ( ep && ep->GetNetCode() == aPad->GetNetCode() );
    };

    auto railOf = [&]( const PAD* aPad )
    {
        return aPad->GetNetCode() > 0 && ( firstClass( aPad ) || aPad->GetPinType().Contains( wxT( "power" ) ) );
    };

    std::shared_ptr<CONNECTIVITY_DATA> conn = aBoard->GetConnectivity();
    std::map<const PAD*, RAIL_PIN_PLAN> plans;

    // pins already joined to a via keep it: their neighbours are planned around it
    std::vector<const RAIL_PIN_PLAN*> fixed;

    for( const auto& [pad, dir] : outward )
    {
        if( !fine.count( pad ) || !railOf( pad ) )
            continue;

        RAIL_PIN_PLAN& p = plans[pad];
        p.pad = pad;
        p.outward = dir;

        if( PCB_VIA* via = conn ? viaOfPin( *conn, pad ) : nullptr )
        {
            p.planned = true;
            p.spot = via->GetPosition();
            p.dia = via->GetWidth( F_Cu );
            p.drill = via->GetDrillValue();
            p.stub = SEG( pad->GetPosition(), p.spot );
            p.pattern = "fixed";
            p.reason = "a via joined to the pin already: kept";
        }
    }

    for( const auto& [pad, p] : plans )
    {
        if( p.pattern == "fixed" )
            fixed.push_back( &p );
    }

    // the rule area a via at aSpot, or its stub on the pad's layers, would sit in
    auto barredBy = [&]( const PAD* aPad, const VECTOR2I& aSpot, int aDia ) -> const ZONE*
    {
        const SEG stub( aPad->GetPosition(), aSpot );

        for( const ZONE* z : ruleAreas )
        {
            if( z->GetDoNotAllowVias() && ( z->GetLayerSet() & LSET::AllCuMask() ).any() && z->Outline()->Collide( aSpot, aDia / 2 ) )
                return z;

            if( z->GetDoNotAllowTracks() && ( z->GetLayerSet() & aPad->GetLayerSet() & LSET::AllCuMask() ).any()
                && z->Outline()->Collide( stub, stubMin / 2 ) )
            {
                return z;
            }
        }

        return nullptr;
    };

    // the fixed via a via at aSpot with its stub from aPad would come too near
    auto nearFixed = [&]( const PAD* aPad, const VECTOR2I& aSpot, int aDia, int aDrill ) -> const RAIL_PIN_PLAN*
    {
        const SEG stub( aPad->GetPosition(), aSpot );

        for( const RAIL_PIN_PLAN* f : fixed )
        {
            if( f->pad == aPad || f->pad->GetNetCode() == aPad->GetNetCode() )
                continue;

            const int clear = std::max( clearOf( aPad ), clearOf( f->pad ) );

            if( ( aSpot - f->spot ).EuclideanNorm() < viaApart( aDia, aDrill, f->dia, f->drill, clear )
                || f->stub.Distance( aSpot ) < stubApart( aDia, aDrill, clear )
                || stub.Distance( f->spot ) < stubApart( f->dia, f->drill, clear ) )
            {
                return f;
            }
        }

        return nullptr;
    };

    std::map<std::pair<int, int>, std::vector<const PAD*>> rows;

    for( const auto& [pad, dir] : outward )
    {
        if( fine.count( pad ) )
            rows[{ dir.x, dir.y }].push_back( pad );
    }

    const std::vector<std::pair<int, int>> sizes = aOptions.minViaDia ? std::vector<std::pair<int, int>>{ { aOptions.classViaDia, aOptions.classViaDrill },
                                                                                                          { aOptions.minViaDia, aOptions.minViaDrill } }
                                                                      : std::vector<std::pair<int, int>>{ { aOptions.classViaDia, aOptions.classViaDrill } };

    for( auto& [key, row] : rows )
    {
        const VECTOR2I dir( key.first, key.second );
        const VECTOR2I across( -dir.y, dir.x );
        auto           along = [&]( const PAD* aPad ) { return (int64_t) aPad->GetPosition().x * across.x + (int64_t) aPad->GetPosition().y * across.y; };

        std::sort( row.begin(), row.end(), [&]( const PAD* x, const PAD* y ) { return along( x ) < along( y ); } );

        std::vector<std::vector<const PAD*>> runs( 1 );

        for( size_t i = 0; i < row.size(); ++i )
        {
            const PAD* p = row[i];
            const bool apart = i > 0 && along( p ) - along( row[i - 1] ) > pcbIUScale.mmToIU( 0.65 );

            if( !railOf( p ) || apart )
                runs.emplace_back();

            if( !railOf( p ) )
                continue;

            if( !runs.back().empty() && runs.back().back()->GetNetCode() == p->GetNetCode() )
            {
                RAIL_PIN_PLAN& joins = plans[p];

                if( joins.pattern != "fixed" )
                    joins.reason = "beside pin " + std::string( runs.back().back()->GetNumber().utf8_str() ) + " of its own net: joins that pin's via";

                continue;
            }

            runs.back().push_back( p );
        }

        for( const std::vector<const PAD*>& run : runs )
        {
            if( run.empty() )
                continue;

            if( run.size() < 2 )
            {
                if( plans[run.front()].pattern != "fixed" )
                    plans[run.front()].reason = "no other rail pin beside it: no run pattern";

                continue;
            }

            int clear = 0;

            for( const PAD* p : run )
                clear = std::max( clear, clearOf( p ) );

            const BOX2I pb = run.front()->GetBoundingBox();
            const int   padHalf = dir.x ? pb.GetWidth() / 2 : pb.GetHeight() / 2;
            // inward to the exposed pad, or without one to the package's middle (the opposite row's
            // vias come from the other side)
            const VECTOR2I mid = centres.Centre();
            int            room = dir.x ? std::abs( run.front()->GetPosition().x - mid.x ) : std::abs( run.front()->GetPosition().y - mid.y );

            if( ep )
            {
                const BOX2I    eb = ep->GetBoundingBox();
                const VECTOR2I c = run.front()->GetPosition();
                room = dir.x > 0 ? c.x - eb.GetRight() : dir.x < 0 ? eb.GetLeft() - c.x : dir.y > 0 ? c.y - eb.GetBottom()
                                                                                                   : eb.GetTop() - c.y;
            }

            std::vector<int> gaps;

            for( size_t i = 0; i + 1 < run.size(); ++i )
                gaps.push_back( (int) ( along( run[i + 1] ) - along( run[i] ) ) );

            auto place = [&]( const PAD* aPad, int aDepth, int aDia, int aDrill, const char* aPattern, const std::string& aReason )
            {
                RAIL_PIN_PLAN& p = plans[aPad];

                if( p.pattern == "fixed" )
                    return;

                p.planned = true;
                p.spot = aPad->GetPosition() - dir * aDepth;
                p.dia = aDia;
                p.drill = aDrill;
                p.stub = SEG( aPad->GetPosition(), p.spot );
                p.pattern = aPattern;
                p.reason = aReason;
            };

            auto skip = [&]( const PAD* aPad, const char* aPattern, const std::string& aReason )
            {
                RAIL_PIN_PLAN& p = plans[aPad];

                if( p.pattern == "fixed" )
                    return;

                p.skipped = true;
                p.pattern = aPattern;
                p.reason = aReason;
            };

            bool planned = false;

            for( const auto& [dia, drill] : sizes )
            {
                const int depth = padHalf + dia / 2 + minClearance;
                const int pair = viaApart( dia, drill, dia, drill, clear );
                const int pass = stubApart( dia, drill, clear );

                if( depth + dia / 2 + clear > room )
                    continue;

                if( std::all_of( gaps.begin(), gaps.end(), [&]( int g ) { return g >= pair; } ) )
                {
                    for( const PAD* p : run )
                        place( p, depth, dia, drill, "one_depth", "a via at each pin of the run, one depth" );

                    planned = true;
                    break;
                }

                bool alternate = true;
                int  rise = 0;

                for( size_t i = 0; i < gaps.size(); ++i )
                {
                    alternate &= gaps[i] >= pass && ( i + 1 >= gaps.size() || gaps[i] + gaps[i + 1] >= pair );

                    if( gaps[i] < pair )
                        rise = std::max( rise, (int) std::ceil( std::sqrt( (double) pair * pair - (double) gaps[i] * gaps[i] ) ) );
                }

                if( alternate && depth + rise + dia / 2 + clear <= room )
                {
                    for( size_t i = 0; i < run.size(); ++i )
                    {
                        place( run[i], depth + ( i % 2 ? rise : 0 ), dia, drill, "alternating",
                               "a via at each pin of the run, depths alternating (two vias side by side do not fit)" );
                    }

                    planned = true;
                    break;
                }
            }

            if( !planned )
            {
                // every other pin: poured nets first, each pin as far from the chosen as one via
                // from the next
                const auto [dia, drill] = sizes.back();
                const int  depth = padHalf + dia / 2 + minClearance;
                const int  pair = viaApart( dia, drill, dia, drill, clear );

                if( depth + dia / 2 + clear > room )
                {
                    for( const PAD* p : run )
                    {
                        if( plans[p].pattern != "fixed" )
                            plans[p].reason = "no room inward of the row for a via (the exposed pad, or without one the package's middle, is too near)";
                    }

                    continue;
                }

                std::vector<size_t> order( run.size() );
                std::iota( order.begin(), order.end(), 0 );
                std::stable_sort( order.begin(), order.end(),
                                  [&]( size_t x, size_t y ) { return firstClass( run[x] ) && !firstClass( run[y] ); } );
                std::vector<size_t> chosen;

                for( size_t i : order )
                {
                    if( std::all_of( chosen.begin(), chosen.end(),
                                     [&]( size_t j ) { return std::abs( along( run[i] ) - along( run[j] ) ) >= pair; } ) )
                    {
                        chosen.push_back( i );
                    }
                }

                // the class's via where the chosen leave room for it
                const int  classPair = viaApart( aOptions.classViaDia, aOptions.classViaDrill, aOptions.classViaDia,
                                                 aOptions.classViaDrill, clear );
                const bool roomy = std::all_of( chosen.begin(), chosen.end(),
                                                [&]( size_t i )
                                                {
                                                    return std::all_of( chosen.begin(), chosen.end(),
                                                                        [&]( size_t j )
                                                                        { return i == j || std::abs( along( run[i] ) - along( run[j] ) ) >= classPair; } );
                                                } )
                                   && padHalf + aOptions.classViaDia + minClearance + clear <= room;

                for( size_t i = 0; i < run.size(); ++i )
                {
                    if( std::find( chosen.begin(), chosen.end(), i ) == chosen.end() )
                    {
                        skip( run[i], "every_other",
                              "a pin of a run of rail pins too tight for a via at each (pitch, via and hole clearance): its "
                              "neighbours of poured nets got them, it is left to the router" );
                    }
                    else if( roomy )
                    {
                        place( run[i], padHalf + aOptions.classViaDia / 2 + minClearance, aOptions.classViaDia, aOptions.classViaDrill,
                               "every_other", "a via at every other pin of the run (two vias a pin apart do not fit)" );
                    }
                    else
                    {
                        place( run[i], depth, dia, drill, "every_other", "a via at every other pin of the run (two vias a pin apart do not fit)" );
                    }
                }
            }

            // around the fixed vias and out of rule areas: the board's smallest via at the same
            // spot where the planned one does not fit, else the pin is left to the router
            for( const PAD* pad : run )
            {
                RAIL_PIN_PLAN& p = plans[pad];

                if( !p.planned || p.pattern == "fixed" )
                    continue;

                const RAIL_PIN_PLAN* f = nearFixed( pad, p.spot, p.dia, p.drill );
                const ZONE*          z = f ? nullptr : barredBy( pad, p.spot, p.dia );

                if( !f && !z )
                    continue;

                if( aOptions.minViaDia && p.dia > aOptions.minViaDia && !nearFixed( pad, p.spot, aOptions.minViaDia, aOptions.minViaDrill )
                    && !barredBy( pad, p.spot, aOptions.minViaDia ) )
                {
                    p.dia = aOptions.minViaDia;
                    p.drill = aOptions.minViaDrill;
                    p.reason += f ? "; the board's smallest via, beside the via of pin " + std::string( f->pad->GetNumber().utf8_str() )
                                  : "; the board's smallest via, beside rule area " + std::string( z->GetZoneName().utf8_str() );
                    continue;
                }

                p.planned = false;
                p.skipped = true;
                p.reason = f ? "its spot is too near the via of pin " + std::string( f->pad->GetNumber().utf8_str() ) + ": left to the router"
                             : "its spot or stub is in rule area '" + std::string( z->GetZoneName().utf8_str() ) + "': left to the router";
            }
        }
    }

    out.reserve( plans.size() );

    for( auto& [pad, p] : plans )
        out.push_back( std::move( p ) );

    return out;
}
