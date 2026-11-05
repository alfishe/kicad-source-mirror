/// @file methods_pcb_end_plan.cpp
/// @brief kicadopenapi end plan: per part, every link that ends there planned at once (layout level 3):
/// which layer each link leaves on, the band of slots it takes on the port line round the part, the
/// order of the bands, and the vias of the pins that change layer. Read only.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <geometry/convex_hull.h>
#include <geometry/shape_poly_set.h>
#include <kicadopenapi_registry.h>
#include <netclass.h>
#include <netinfo.h>
#include <pad.h>
#include <string_utils.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>


namespace
{

const char* const SIDE_NAMES[4] = { "north", "east", "south", "west" };   // clockwise in the board view
constexpr double   QUARTER = 1.5707963267948966;                          // radians per side


std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return std::round( pcbIUScale.IUTomm( aIU ) * 1000.0 ) / 1000.0;
}


nlohmann::json pointMm( const VECTOR2I& aPt )
{
    return nlohmann::json::array( { toMm( aPt.x ), toMm( aPt.y ) } );
}


/// @brief One pad of the part on a signal net
struct PIN
{
    PAD*        pad = nullptr;
    std::string net;
    int         link = -1;
    int         side = 0;
    double      s = 0;            ///< place round the part: side + fraction along it, clockwise, [0, 4)
    bool        tht = false;
    PCB_LAYER_ID layer = F_Cu;    ///< the pad's layer (F_Cu for a through-hole pad)
    PCB_LAYER_ID slotLayer = F_Cu;
    bool        moved = false;    ///< leaves on the other layer than its pad's (a via at an SMD pad)
    std::optional<VECTOR2I> via;
    VECTOR2I    slot;
    int         slotSide = 0;
    int         rank = 0;          ///< place in its link's band
};


/// @brief A bus, or a single net, that ends at the part
struct LINK
{
    std::string      name;
    std::string      kind;       ///< bus / single
    std::vector<int> pins;
    std::set<std::string> nets;
    bool             fast = false;
    VECTOR2I         arrives;    ///< where it comes from: its far pads' centre, or given
    bool             arrivesGiven = false;
    double           arrivesS = 0;
    int              padLayer = 0;   ///< 0 F, 1 B, 2 through-hole only
    int              farLayer = 2;   ///< 0 F, 1 B, 2 both / unknown
    int              pitch = 1;      ///< widest track + clearance of its nets
    int              layer = 0;      ///< chosen: 0 F, 1 B
    int              homeSide = 0;
    std::vector<std::string> farParts;
};


/// @brief The cyclic distance from a to b going clockwise round the part
double ahead( double aFrom, double aTo )
{
    double d = std::fmod( aTo - aFrom, 4.0 );
    return d < 0 ? d + 4.0 : d;
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


/// @brief A layout's quality on one layer: runs of a link broken by another's pins (breaks) and pairs of
/// links whose order round the part is not their arrivals' order (crossings, the fewest over the
/// cyclic shifts)
struct LAYER_SCORE
{
    int breaks = 0;
    int crossings = 0;
};


LAYER_SCORE scoreLayer( const std::vector<PIN>& aPins, const std::vector<LINK>& aLinks,
                        const std::vector<int>& aOnLayer )
{
    LAYER_SCORE score;
    std::vector<std::pair<double, int>> order;   // (s, link) of the pins of the links on the layer

    for( int l : aOnLayer )
    {
        for( int p : aLinks[l].pins )
            order.emplace_back( aPins[p].s, l );
    }

    if( aOnLayer.size() < 2 || order.empty() )
        return score;

    std::sort( order.begin(), order.end() );
    std::map<int, int>                        runs;
    std::map<int, std::pair<double, double>>  runSpan;   // first run's start and end (contiguous links)

    for( size_t i = 0; i < order.size(); ++i )
    {
        const int prev = order[( i + order.size() - 1 ) % order.size()].second;

        if( order[i].second != prev )
            runs[order[i].second]++;
    }

    for( int l : aOnLayer )
        score.breaks += std::max( 0, runs[l] - 1 );

    // each link's middle: of its run when it is one, else the circular mean of its pins
    std::vector<std::pair<double, int>> mids;

    for( int l : aOnLayer )
    {
        double sx = 0, sy = 0;

        for( int p : aLinks[l].pins )
        {
            sx += std::cos( aPins[p].s * QUARTER );
            sy += std::sin( aPins[p].s * QUARTER );
        }

        double mid = std::atan2( sy, sx ) / QUARTER;

        if( runs[l] == 1 )
        {
            for( size_t i = 0; i < order.size(); ++i )
            {
                const size_t prev = ( i + order.size() - 1 ) % order.size();

                if( order[i].second == l && order[prev].second != l )
                {
                    size_t last = i;

                    while( order[( last + 1 ) % order.size()].second == l && ( last + 1 ) % order.size() != i )
                        last = ( last + 1 ) % order.size();

                    mid = order[i].first + ahead( order[i].first, order[last].first ) / 2;
                    break;
                }
            }
        }

        mids.emplace_back( std::fmod( mid + 4.0, 4.0 ), l );
    }

    std::sort( mids.begin(), mids.end() );
    std::vector<std::pair<double, int>> byArrival;

    for( int l : aOnLayer )
        byArrival.emplace_back( aLinks[l].arrivesS, l );

    std::sort( byArrival.begin(), byArrival.end() );
    std::map<int, int> rank;

    for( size_t i = 0; i < byArrival.size(); ++i )
        rank[byArrival[i].second] = (int) i;

    const int k = (int) mids.size();
    int       best = std::numeric_limits<int>::max();

    for( int c = 0; c < k; ++c )
    {
        std::vector<int> seq;

        for( const auto& [mid, l] : mids )
            seq.push_back( ( rank[l] - c + k ) % k );

        best = std::min( best, inversions( seq ) );
    }

    score.crossings = best;
    return score;
}


/// @brief The chosen layers' score, lexicographic: breaks, crossings, fast links moved, vias, links off
/// their far end's layer, the width of the links moved
using SCORE = std::array<int, 6>;


SCORE scoreOf( const std::vector<PIN>& aPins, const std::vector<LINK>& aLinks, const std::vector<int>& aBuses,
               const std::vector<int>& aLayerOf )
{
    SCORE            score = { 0, 0, 0, 0, 0, 0 };
    std::vector<int> onLayer[2];

    for( size_t i = 0; i < aBuses.size(); ++i )
    {
        const LINK& link = aLinks[aBuses[i]];
        const int   layer = aLayerOf[i];
        int         vias = 0;

        onLayer[layer].push_back( aBuses[i] );

        for( int p : link.pins )
            vias += !aPins[p].tht && ( aPins[p].layer == F_Cu ? 0 : 1 ) != layer;

        score[2] += link.fast && vias > 0;
        score[3] += vias;
        score[4] += link.farLayer != 2 && link.farLayer != layer;
        score[5] += vias > 0 ? (int) link.pins.size() : 0;
    }

    for( const std::vector<int>& on : onLayer )
    {
        const LAYER_SCORE ls = scoreLayer( aPins, aLinks, on );
        score[0] += ls.breaks;
        score[1] += ls.crossings;
    }

    return score;
}


/// @brief The port rectangle round the part: per side its line, as a point at a perimeter distance
struct PORT_RECT
{
    BOX2I rect;

    double length( int aSide ) const
    {
        return aSide % 2 == 0 ? rect.GetWidth() : rect.GetHeight();
    }

    double perimeter() const { return 2.0 * ( rect.GetWidth() + rect.GetHeight() ); }

    /// @brief perimeter distance from the north-west corner, clockwise, of side + fraction
    double distanceOf( double aS ) const
    {
        const int    side = std::clamp( (int) std::floor( aS ), 0, 3 );
        double       d = 0;

        for( int i = 0; i < side; ++i )
            d += length( i );

        return d + ( aS - side ) * length( side );
    }

    std::pair<VECTOR2I, int> pointAt( double aDistance ) const
    {
        double d = std::fmod( aDistance, perimeter() );

        if( d < 0 )
            d += perimeter();

        const int x0 = rect.GetLeft(), x1 = rect.GetRight(), y0 = rect.GetTop(), y1 = rect.GetBottom();

        for( int side = 0; side < 4; ++side )
        {
            if( d <= length( side ) || side == 3 )
            {
                const int along = (int) std::lround( d );

                switch( side )
                {
                case 0: return { VECTOR2I( x0 + along, y0 ), 0 };
                case 1: return { VECTOR2I( x1, y0 + along ), 1 };
                case 2: return { VECTOR2I( x1 - along, y1 ), 2 };
                default: return { VECTOR2I( x0, y1 - along ), 3 };
                }
            }

            d -= length( side );
        }

        return { rect.GetOrigin(), 0 };
    }

    /// @brief side + fraction of the point where the ray from the centre to aAt leaves the rectangle
    double rayS( const VECTOR2I& aAt ) const
    {
        const VECTOR2I c = rect.Centre();
        const double   dx = aAt.x - c.x, dy = aAt.y - c.y;
        const double   hw = std::max( 1.0, rect.GetWidth() / 2.0 ), hh = std::max( 1.0, rect.GetHeight() / 2.0 );

        if( dx == 0 && dy == 0 )
            return 0.5;

        if( std::abs( dy ) / hh >= std::abs( dx ) / hw )
        {
            const double x = dx * hh / std::abs( dy );   // where the ray meets the north / south line

            return dy < 0 ? 0 + std::clamp( ( x + hw ) / ( 2 * hw ), 0.0, 0.999999 )
                          : 2 + std::clamp( ( hw - x ) / ( 2 * hw ), 0.0, 0.999999 );
        }

        const double y = dy * hw / std::abs( dx );

        return dx > 0 ? 1 + std::clamp( ( y + hh ) / ( 2 * hh ), 0.0, 0.999999 )
                      : 3 + std::clamp( ( hh - y ) / ( 2 * hh ), 0.0, 0.999999 );
    }

    /// @brief side + fraction of a point on aSide projected onto that side
    double projectS( const VECTOR2I& aAt, int aSide ) const
    {
        const double w = std::max<double>( 1, rect.GetWidth() ), h = std::max<double>( 1, rect.GetHeight() );
        double       f = 0;

        switch( aSide )
        {
        case 0: f = ( aAt.x - rect.GetLeft() ) / w; break;
        case 1: f = ( aAt.y - rect.GetTop() ) / h; break;
        case 2: f = ( rect.GetRight() - aAt.x ) / w; break;
        default: f = ( rect.GetBottom() - aAt.y ) / h; break;
        }

        return aSide + std::clamp( f, 0.0, 0.999999 );
    }
};


/// @brief Order-keeping packing on a line: items wanting aWant (left ends) with widths aWidth keep their
/// order and do not overlap, as near their wishes as can be (least squares, pool adjacent violators)
std::vector<double> pack( const std::vector<double>& aWant, const std::vector<double>& aWidth )
{
    const size_t        n = aWant.size();
    std::vector<double> offset( n, 0 ), target( n );

    for( size_t i = 1; i < n; ++i )
        offset[i] = offset[i - 1] + aWidth[i - 1];

    for( size_t i = 0; i < n; ++i )
        target[i] = aWant[i] - offset[i];

    struct BLOCK
    {
        double sum;
        int    count;
        size_t first;
    };

    std::vector<BLOCK> blocks;

    for( size_t i = 0; i < n; ++i )
    {
        blocks.push_back( { target[i], 1, i } );

        while( blocks.size() > 1
               && blocks[blocks.size() - 2].sum / blocks[blocks.size() - 2].count > blocks.back().sum / blocks.back().count )
        {
            BLOCK last = blocks.back();
            blocks.pop_back();
            blocks.back().sum += last.sum;
            blocks.back().count += last.count;
        }
    }

    std::vector<double> at( n );

    for( size_t b = 0; b < blocks.size(); ++b )
    {
        const size_t end = b + 1 < blocks.size() ? blocks[b + 1].first : n;

        for( size_t i = blocks[b].first; i < end; ++i )
            at[i] = blocks[b].sum / blocks[b].count + offset[i];
    }

    return at;
}


KOPENAPI_RESULT h_pcb_end_plan( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const auto started = std::chrono::steady_clock::now();

    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string ref = aArgs.value( "ref", std::string() );
    FOOTPRINT*        fp = nullptr;

    for( FOOTPRINT* f : board->Footprints() )
    {
        if( str( f->GetReference() ) == ref )
            fp = f;
    }

    if( !fp )
        return KOPENAPI_RESULT::Error( 404, "ref: no part " + ref );

    const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
    const int                    maxLinks = aArgs.value( "max_links", 10 );

    // roles: rails stay out of the plan, pairs / clocks / matched nets make their link fast
    std::set<std::string> rails, fastNets;

    if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
    {
        KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

        if( r.status == 200 )
        {
            for( const nlohmann::json& n : r.body.value( "nets", nlohmann::json::array() ) )
            {
                const std::string role = n.value( "role", std::string() );
                const std::string name = n.value( "net", std::string() );

                if( role == "ground" || role == "power" || role == "power_hc" || role == "local_supply" || role == "nc" )
                    rails.insert( name );
                else if( role == "pair" || role == "clock" || role == "matched" )
                    fastNets.insert( name );
            }
        }
    }

    // the part's signal pins: nets with a pad on another part
    std::map<std::string, std::vector<PAD*>> otherPads;

    for( FOOTPRINT* f : board->Footprints() )
    {
        if( f == fp )
            continue;

        for( PAD* pad : f->Pads() )
        {
            if( pad->GetNetCode() > 0 )
                otherPads[str( UnescapeString( pad->GetNetname() ) )].push_back( pad );
        }
    }

    std::vector<PIN> pins;
    BOX2I            padBox;
    bool             first = true;

    for( PAD* pad : fp->Pads() )
    {
        if( pad->GetNumber().IsEmpty() || pad->GetNetCode() <= 0 )
            continue;

        const std::string net = str( UnescapeString( pad->GetNetname() ) );

        if( first )
            padBox = BOX2I( pad->GetPosition(), VECTOR2I( 0, 0 ) );
        else
            padBox.Merge( pad->GetPosition() );

        first = false;

        if( rails.count( net ) || !otherPads.count( net ) )
            continue;

        PIN pin;
        pin.pad = pad;
        pin.net = net;
        pin.tht = pad->IsOnLayer( F_Cu ) && pad->IsOnLayer( B_Cu );
        pin.layer = pin.tht || pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
        pins.push_back( pin );
    }

    if( pins.empty() )
        return KOPENAPI_RESULT::Error( 409, "ref " + ref + ": no signal pins whose nets go to other parts" );

    std::map<std::string, std::vector<int>> pinsOfNet;

    for( size_t i = 0; i < pins.size(); ++i )
        pinsOfNet[pins[i].net].push_back( (int) i );

    // links: the buses that end here (pcb_layout_graph), or the ones given; every other signal net alone
    std::vector<LINK> links;
    std::set<std::string> taken;

    auto addLink = [&]( const std::string& aName, const std::string& aKind, const std::vector<std::string>& aNets ) -> LINK*
    {
        LINK link;
        link.name = aName;
        link.kind = aKind;

        for( const std::string& net : aNets )
        {
            if( taken.count( net ) || !pinsOfNet.count( net ) )
                continue;

            link.nets.insert( net );

            for( int p : pinsOfNet[net] )
                link.pins.push_back( p );
        }

        if( link.pins.empty() || ( aKind == "bus" && link.nets.size() < 2 ) )
            return nullptr;

        for( const std::string& net : link.nets )
            taken.insert( net );

        links.push_back( link );
        return &links.back();
    };

    if( aArgs.contains( "links" ) && aArgs["links"].is_array() )
    {
        for( const nlohmann::json& given : aArgs["links"] )
        {
            std::vector<std::string> nets;

            for( const nlohmann::json& n : given.value( "nets", nlohmann::json::array() ) )
                nets.push_back( n.get<std::string>() );

            const std::string name = given.value( "name", "link" + std::to_string( links.size() + 1 ) );

            if( LINK* link = addLink( name, nets.size() > 1 ? "bus" : "single", nets ) )
            {
                if( given.contains( "from_mm" ) && given["from_mm"].is_array() && given["from_mm"].size() == 2 )
                {
                    link->arrives = VECTOR2I( pcbIUScale.mmToIU( given["from_mm"][0].get<double>() ),
                                              pcbIUScale.mmToIU( given["from_mm"][1].get<double>() ) );
                    link->arrivesGiven = true;
                }
            }
        }
    }
    else if( std::optional<KOPENAPI_METHOD> graph = KOPENAPI_REGISTRY::Get().Find( "pcb_layout_graph" ) )
    {
        KOPENAPI_RESULT r = graph->handler( aCtx, { { "full", true } } );

        if( r.status == 200 )
        {
            std::vector<std::pair<int, nlohmann::json>> buses;

            for( const nlohmann::json& bus : r.body.value( "buses", nlohmann::json::array() ) )
                buses.emplace_back( bus.value( "width", 0 ), bus );

            std::stable_sort( buses.begin(), buses.end(),
                              []( const auto& x, const auto& y ) { return x.first > y.first; } );

            for( const auto& [width, bus] : buses )
            {
                bool here = false;

                for( const nlohmann::json& end : bus.value( "ends", nlohmann::json::array() ) )
                    here = here || end.value( "part", std::string() ) == ref;

                if( !here )
                    continue;

                std::vector<std::string> nets;

                for( const nlohmann::json& n : bus.value( "nets", nlohmann::json::array() ) )
                    nets.push_back( n.get<std::string>() );

                addLink( bus.value( "name", std::string( "bus" ) ), "bus", nets );
            }
        }
    }

    for( const PIN& pin : pins )
    {
        if( !taken.count( pin.net ) )
            addLink( pin.net, "single", { pin.net } );
    }

    // per link: where it comes from, its far end's layer, fast, its pitch
    int pitchMax = 1;

    for( size_t l = 0; l < links.size(); ++l )
    {
        LINK& link = links[l];
        int   onF = 0, onB = 0, padF = 0, padB = 0;
        VECTOR2D sum( 0, 0 );
        int      count = 0;
        std::set<std::string> parts;

        for( const std::string& net : link.nets )
        {
            link.fast = link.fast || fastNets.count( net );

            for( PAD* pad : otherPads[net] )
            {
                sum += VECTOR2D( pad->GetPosition() );
                count++;
                const bool f = pad->IsOnLayer( F_Cu ), b = pad->IsOnLayer( B_Cu );
                onF += f && !b;
                onB += b && !f;

                if( FOOTPRINT* parent = pad->GetParentFootprint() )
                    parts.insert( str( parent->GetReference() ) );
            }
        }

        link.farParts.assign( parts.begin(), parts.end() );
        link.farLayer = onF > onB ? 0 : onB > onF ? 1 : 2;

        if( !link.arrivesGiven && count > 0 )
            link.arrives = VECTOR2I( (int) std::lround( sum.x / count ), (int) std::lround( sum.y / count ) );

        for( int p : link.pins )
        {
            pins[p].link = (int) l;

            if( !pins[p].tht )
                ( pins[p].layer == F_Cu ? padF : padB )++;

            NETCLASS* nc = pins[p].pad->GetEffectiveNetClass();
            const int clearance = std::max( { ds.m_MinClearance, nc && nc->HasClearance() ? nc->GetClearance() : 0, 1 } );
            const int width = nc && nc->HasTrackWidth() ? nc->GetTrackWidth() : ds.GetCurrentTrackWidth();
            link.pitch = std::max( link.pitch, width + clearance );
        }

        link.padLayer = padF + padB == 0 ? 2 : padF >= padB ? 0 : 1;
        pitchMax = std::max( pitchMax, link.pitch );
    }

    // sides: by the pad's place in the pads' extent; a single row faces the side its links come from
    const VECTOR2I centre = padBox.Centre();
    const double   hw = padBox.GetWidth() / 2.0, hh = padBox.GetHeight() / 2.0;

    for( PIN& pin : pins )
    {
        const VECTOR2I d = pin.pad->GetPosition() - centre;
        const VECTOR2I a = links[pin.link].arrives - centre;

        const BOX2I land = pin.pad->GetBoundingBox();   // a row's pad is long across its side

        if( hh < hw / 8 )
            pin.side = a.y < 0 ? 0 : 2;
        else if( hw < hh / 8 )
            pin.side = a.x > 0 ? 1 : 3;
        else if( land.GetWidth() > 1.2 * land.GetHeight() && std::abs( d.x ) > hw / 2 )
            pin.side = d.x > 0 ? 1 : 3;
        else if( land.GetHeight() > 1.2 * land.GetWidth() && std::abs( d.y ) > hh / 2 )
            pin.side = d.y > 0 ? 2 : 0;
        else if( std::abs( d.x ) / hw >= std::abs( d.y ) / hh )
            pin.side = d.x > 0 ? 1 : 3;
        else
            pin.side = d.y > 0 ? 2 : 0;
    }

    // the port rectangle before the vias: the courtyard a pitch out
    const SHAPE_POLY_SET& courtyard = fp->GetCourtyard( fp->IsFlipped() ? B_CrtYd : F_CrtYd );
    BOX2I                 base = courtyard.OutlineCount() ? courtyard.BBox() : fp->GetBoundingBox( false );
    base.Merge( padBox );
    PORT_RECT             port{ base };
    port.rect.Inflate( pitchMax );

    for( PIN& pin : pins )
        pin.s = port.projectS( pin.pad->GetPosition(), pin.side );

    for( LINK& link : links )
        link.arrivesS = port.rayS( link.arrives );

    // the layer of each bus: every choice tried up to max_links buses, else one bus at a time widest first
    std::vector<int> buses, singles;

    for( size_t l = 0; l < links.size(); ++l )
        ( links[l].kind == "bus" ? buses : singles ).push_back( (int) l );

    std::vector<int> layerOf( buses.size(), 0 );

    for( size_t i = 0; i < buses.size(); ++i )
        layerOf[i] = links[buses[i]].padLayer == 1 ? 1 : 0;

    std::string search = "exact";
    int         variants = 0;
    SCORE       best = scoreOf( pins, links, buses, layerOf );

    if( (int) buses.size() <= maxLinks )
    {
        const std::vector<int> start = layerOf;

        for( int mask = 0; mask < ( 1 << buses.size() ); ++mask )
        {
            std::vector<int> tryOf( buses.size() );

            for( size_t i = 0; i < buses.size(); ++i )
                tryOf[i] = ( mask >> i ) & 1;

            variants++;
            const SCORE s = scoreOf( pins, links, buses, tryOf );

            if( s < best || ( s == best && tryOf == start ) )
            {
                best = s;
                layerOf = tryOf;
            }
        }
    }
    else
    {
        search = "greedy";

        for( size_t i = 0; i < buses.size(); ++i )
        {
            std::vector<int> other = layerOf;
            other[i] = 1 - other[i];
            variants++;
            const SCORE s = scoreOf( pins, links, buses, other );

            if( s < best )
            {
                best = s;
                layerOf = other;
            }
        }
    }

    for( size_t i = 0; i < buses.size(); ++i )
        links[buses[i]].layer = layerOf[i];

    // the buses' arcs per layer: a single inside one goes to the other layer, if that one is free there
    auto arcsOn = [&]( int aLayer )
    {
        std::vector<std::pair<double, double>> arcs;

        for( size_t i = 0; i < buses.size(); ++i )
        {
            if( layerOf[i] != aLayer )
                continue;

            std::vector<double> s;

            for( int p : links[buses[i]].pins )
                s.push_back( pins[p].s );

            std::sort( s.begin(), s.end() );
            double gap = -1;
            size_t at = 0;

            for( size_t k = 0; k < s.size(); ++k )
            {
                const double g = ahead( s[k], s[( k + 1 ) % s.size()] );

                if( s.size() == 1 || g > gap )
                {
                    gap = g;
                    at = k;
                }
            }

            const double from = s[( at + 1 ) % s.size()];
            arcs.emplace_back( from, ahead( from, s[at] ) );
        }

        return arcs;
    };

    auto inside = [&]( const std::vector<std::pair<double, double>>& aArcs, double aS )
    {
        for( const auto& [from, length] : aArcs )
        {
            const double d = ahead( from, aS );

            if( d > 1e-9 && d < length - 1e-9 )
                return true;
        }

        return false;
    };

    const std::vector<std::pair<double, double>> arcs[2] = { arcsOn( 0 ), arcsOn( 1 ) };
    nlohmann::json                               boxedSingles = nlohmann::json::array();

    for( int l : singles )
    {
        LINK&      link = links[l];
        const PIN& pin = pins[link.pins[0]];
        const int  own = pin.tht ? ( link.farLayer == 1 ? 1 : 0 ) : ( pin.layer == F_Cu ? 0 : 1 );
        link.layer = own;

        if( inside( arcs[own], pin.s ) )
        {
            if( !inside( arcs[1 - own], pin.s ) )
                link.layer = 1 - own;
            else
                boxedSingles.push_back( link.name );
        }
    }

    for( LINK& link : links )
    {
        for( int p : link.pins )
        {
            PIN& pin = pins[p];
            pin.slotLayer = link.layer == 0 ? F_Cu : B_Cu;
            pin.moved = !pin.tht && pin.slotLayer != pin.layer;
        }

        std::map<int, int> bySide;

        for( int p : link.pins )
            bySide[pins[p].side]++;

        link.homeSide = std::max_element( bySide.begin(), bySide.end(),
                                          []( const auto& x, const auto& y ) { return x.second < y.second; } )->first;
    }

    // vias of the pins that change layer: where pcb_fanout would put them
    nlohmann::json noSpot = nlohmann::json::array();
    int            viaDia = ds.m_ViasMinSize;
    std::vector<std::string> movedNets;

    for( const PIN& pin : pins )
    {
        if( pin.moved )
            movedNets.push_back( pin.net );
    }

    if( !movedNets.empty() )
    {
        std::optional<KOPENAPI_METHOD> fan = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout" );
        std::map<std::string, VECTOR2I> spots;
        std::map<std::string, std::string> skipWhy;

        if( fan )
        {
            KOPENAPI_RESULT r = fan->handler( aCtx, { { "ref", ref }, { "nets", movedNets }, { "dry_run", true }, { "full", true } } );

            if( r.status == 200 )
            {
                if( r.body.contains( "via" ) )
                    viaDia = pcbIUScale.mmToIU( r.body["via"].value( "diameter_mm", toMm( viaDia ) ) );

                for( const nlohmann::json& row : r.body.value( "pins", nlohmann::json::array() ) )
                {
                    if( row.contains( "row" ) && row.contains( "end_mm" ) )
                    {
                        spots[row.value( "pad", std::string() )] = VECTOR2I( pcbIUScale.mmToIU( row["end_mm"][0].get<double>() ),
                                                                             pcbIUScale.mmToIU( row["end_mm"][1].get<double>() ) );
                    }
                }

                for( const nlohmann::json& row : r.body.value( "skipped_pins", nlohmann::json::array() ) )
                    skipWhy[row.value( "pad", std::string() )] = row.value( "reason", std::string() );
            }
            else
            {
                skipWhy[""] = "pcb_fanout: " + r.Message();
            }
        }

        for( PIN& pin : pins )
        {
            if( !pin.moved )
                continue;

            const std::string name = ref + "." + str( pin.pad->GetNumber() );

            if( auto it = spots.find( name ); it != spots.end() )
                pin.via = it->second;
            else if( auto why = skipWhy.find( name ); why != skipWhy.end() )
                noSpot.push_back( { { "pad", name }, { "reason", why->second } } );
            else
                noSpot.push_back( { { "pad", name }, { "reason", skipWhy.count( "" ) ? skipWhy[""] : "not fanned" } } );
        }
    }

    // the port line past the vias: per side a via's half, a clearance and a pitch past the outermost
    int out[4] = { pitchMax, pitchMax, pitchMax, pitchMax };

    for( const PIN& pin : pins )
    {
        if( !pin.via )
            continue;

        const int reach = viaDia / 2 + std::max( ds.m_MinClearance, 1 ) + pitchMax / 2;
        const VECTOR2I& v = *pin.via;
        out[0] = std::max( out[0], base.GetTop() - v.y + reach );
        out[1] = std::max( out[1], v.x - base.GetRight() + reach );
        out[2] = std::max( out[2], v.y - base.GetBottom() + reach );
        out[3] = std::max( out[3], base.GetLeft() - v.x + reach );
    }

    PORT_RECT line;
    line.rect = BOX2I( VECTOR2I( base.GetLeft() - out[3], base.GetTop() - out[0] ),
                       VECTOR2I( base.GetWidth() + out[1] + out[3], base.GetHeight() + out[0] + out[2] ) );

    // bands and single slots per layer: order round the part kept, packed on the port line
    nlohmann::json overflow = nlohmann::json::array();

    for( int layer = 0; layer < 2; ++layer )
    {
        struct ITEM
        {
            int                 link;
            std::vector<int>    pins;   ///< in order round the part
            double              want;   ///< middle, perimeter distance
            double              width;
        };

        std::vector<ITEM> items;

        for( size_t l = 0; l < links.size(); ++l )
        {
            if( links[l].layer != layer )
                continue;

            ITEM item{ (int) l, links[l].pins, 0, 0 };
            std::sort( item.pins.begin(), item.pins.end(), [&]( int x, int y ) { return pins[x].s < pins[y].s; } );

            // start after the widest gap between its pins
            double gap = -1;
            size_t at = 0;

            for( size_t k = 0; k < item.pins.size(); ++k )
            {
                const double g = ahead( pins[item.pins[k]].s, pins[item.pins[( k + 1 ) % item.pins.size()]].s );

                if( item.pins.size() == 1 || g > gap )
                {
                    gap = g;
                    at = k;
                }
            }

            std::rotate( item.pins.begin(), item.pins.begin() + ( at + 1 ) % item.pins.size(), item.pins.end() );
            const double from = line.distanceOf( pins[item.pins.front()].s );
            double       span = line.distanceOf( pins[item.pins.back()].s ) - from;

            if( span < 0 )
                span += line.perimeter();

            item.width = (double) item.pins.size() * links[l].pitch;
            item.want = std::fmod( from + span / 2, line.perimeter() );
            items.push_back( item );
        }

        if( items.empty() )
            continue;

        std::sort( items.begin(), items.end(), []( const ITEM& x, const ITEM& y ) { return x.want < y.want; } );

        // cut the ring at the widest gap between neighbouring items
        size_t cut = 0;
        double widest = -1;

        for( size_t k = 0; k < items.size(); ++k )
        {
            const ITEM& a = items[k];
            const ITEM& b = items[( k + 1 ) % items.size()];
            double      g = b.want - b.width / 2 - ( a.want + a.width / 2 );

            if( k + 1 == items.size() )
                g += line.perimeter();

            if( g > widest )
            {
                widest = g;
                cut = ( k + 1 ) % items.size();
            }
        }

        std::rotate( items.begin(), items.begin() + cut, items.end() );
        std::vector<double> want, width;
        double              lastWant = -1e18, total = 0;

        for( const ITEM& item : items )
        {
            double w = item.want - item.width / 2;

            while( w < lastWant - line.perimeter() / 2 )
                w += line.perimeter();

            want.push_back( w );
            width.push_back( item.width );
            lastWant = w;
            total += item.width;
        }

        const std::vector<double> at = pack( want, width );

        for( size_t k = 0; k < items.size(); ++k )
        {
            const ITEM& item = items[k];
            const LINK& link = links[item.link];

            for( size_t j = 0; j < item.pins.size(); ++j )
            {
                auto [pt, side] = line.pointAt( at[k] + link.pitch * ( j + 0.5 ) );
                pins[item.pins[j]].slot = pt;
                pins[item.pins[j]].slotSide = side;
                pins[item.pins[j]].rank = (int) j;
            }
        }

        // a side short of room: the items at home there need more than its length
        double need[4] = { 0, 0, 0, 0 };

        for( const ITEM& item : items )
            need[links[item.link].homeSide] += item.width;

        for( int side = 0; side < 4; ++side )
        {
            if( need[side] > line.length( side ) )
            {
                nlohmann::json names = nlohmann::json::array();

                for( const ITEM& item : items )
                {
                    if( links[item.link].homeSide == side )
                        names.push_back( links[item.link].name );
                }

                overflow.push_back( { { "side", SIDE_NAMES[side] }, { "layer", layer == 0 ? "F.Cu" : "B.Cu" },
                                      { "need_mm", toMm( need[side] ) }, { "have_mm", toMm( line.length( side ) ) },
                                      { "links", names } } );
            }
        }

        if( total > line.perimeter() )
        {
            overflow.push_back( { { "side", "all" }, { "layer", layer == 0 ? "F.Cu" : "B.Cu" },
                                  { "need_mm", toMm( total ) }, { "have_mm", toMm( line.perimeter() ) } } );
        }
    }

    // the answer
    nlohmann::json linksOut = nlohmann::json::array(), pinsOut = nlohmann::json::array();
    const bool     withPaths = aArgs.value( "lanes", false );

    for( const LINK& link : links )
    {
        int            vias = 0;
        nlohmann::json slots = nlohmann::json::array();
        std::vector<VECTOR2I> hullPts;
        std::vector<int> ordered = link.pins;
        std::sort( ordered.begin(), ordered.end(), [&]( int x, int y ) { return pins[x].rank < pins[y].rank; } );

        for( int p : ordered )
        {
            const PIN& pin = pins[p];
            vias += pin.moved;
            hullPts.push_back( pin.pad->GetPosition() );
            hullPts.push_back( pin.slot );

            if( pin.via )
                hullPts.push_back( *pin.via );

            nlohmann::json row = { { "pad", ref + "." + str( pin.pad->GetNumber() ) }, { "net", pin.net },
                                   { "link", link.name }, { "side", SIDE_NAMES[pin.side] },
                                   { "pad_layer", pin.tht ? "THT" : pin.layer == F_Cu ? "F.Cu" : "B.Cu" },
                                   { "layer", pin.slotLayer == F_Cu ? "F.Cu" : "B.Cu" },
                                   { "slot_mm", pointMm( pin.slot ) }, { "slot_side", SIDE_NAMES[pin.slotSide] },
                                   { "gate", pin.moved ? "via" : "point" } };

            if( pin.via )
                row["via_mm"] = pointMm( *pin.via );

            if( withPaths )
            {
                nlohmann::json path = nlohmann::json::array( { pointMm( pin.pad->GetPosition() ) } );

                if( pin.via )
                    path.push_back( pointMm( *pin.via ) );

                path.push_back( pointMm( pin.slot ) );
                row["path_mm"] = path;
            }

            pinsOut.push_back( row );
            slots.push_back( pointMm( pin.slot ) );
        }

        std::vector<VECTOR2I> hull;
        BuildConvexHull( hull, hullPts );
        SHAPE_POLY_SET room;
        room.NewOutline();

        for( const VECTOR2I& pt : hull )
            room.Append( pt );

        if( hull.size() >= 3 )
            room.Inflate( link.pitch / 2, CORNER_STRATEGY::CHAMFER_ALL_CORNERS, ARC_LOW_DEF );

        nlohmann::json roomOut = nlohmann::json::array();

        if( room.OutlineCount() && room.COutline( 0 ).PointCount() >= 3 )
        {
            for( int i = 0; i < room.COutline( 0 ).PointCount(); ++i )
                roomOut.push_back( pointMm( room.COutline( 0 ).CPoint( i ) ) );
        }

        nlohmann::json nets = nlohmann::json::array();

        for( int p : ordered )
            nets.push_back( pins[p].net );

        const int arrivesSide = std::clamp( (int) std::floor( link.arrivesS ), 0, 3 );
        linksOut.push_back( { { "name", link.name }, { "kind", link.kind }, { "width", link.pins.size() },
                              { "nets", nets }, { "fast", link.fast }, { "to", link.farParts },
                              { "arrives_mm", pointMm( link.arrives ) }, { "arrives_side", SIDE_NAMES[arrivesSide] },
                              { "arrives_from", link.arrivesGiven ? "given" : "far pads" },
                              { "far_layer", link.farLayer == 0 ? "F.Cu" : link.farLayer == 1 ? "B.Cu" : "both" },
                              { "layer", link.layer == 0 ? "F.Cu" : "B.Cu" }, { "vias", vias },
                              { "home_side", SIDE_NAMES[link.homeSide] }, { "pitch_mm", toMm( link.pitch ) },
                              { "slots_mm", slots }, { "room_mm", roomOut } } );
    }

    nlohmann::json sides = nlohmann::json::object();

    for( int side = 0; side < 4; ++side )
    {
        const auto a = line.pointAt( line.distanceOf( side ) ).first;
        const auto b = line.pointAt( line.distanceOf( side ) + line.length( side ) ).first;
        sides[SIDE_NAMES[side]] = { { "from_mm", pointMm( a ) }, { "to_mm", pointMm( b ) },
                                    { "length_mm", toMm( line.length( side ) ) } };
    }

    int vias = 0;

    for( const PIN& pin : pins )
        vias += pin.moved;

    nlohmann::json answer = {
        { "ref", ref },
        { "links", linksOut },
        { "pins", pinsOut },
        { "port_line", sides },
        { "vias", vias },
        { "breaks_left", best[0] },
        { "crossings_left", best[1] },
        { "fast_moved", best[2] },
        { "ends_overflow", overflow },
        { "singles_boxed", boxedSingles },
        { "no_spot", noSpot },
        { "search", search },
        { "variants", variants },
        { "via_diameter_mm", toMm( viaDia ) },
        { "ms", (int) std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - started ).count() }
    };

    return KOPENAPI_RESULT::Ok( answer );
}

} // namespace


KOPENAPI_REGISTER( "pcb_end_plan",
                   "Layout level 3, the end plan of one part: every link that ends there planned at once, before "
                   "the corridors are laid. Links: the buses of pcb_layout_graph that end at the part (or links "
                   "given: name, nets, from_mm, the point it arrives from, e.g. its floorplan portal), every other "
                   "signal net alone (single); rails are left out. Each bus leaves on one outer layer (F.Cu / B.Cu): "
                   "every choice is scored (all of them up to max_links buses, else one bus at a time), in order: breaks (a "
                   "bus's pins split on its layer by another bus's), crossings (pairs of buses whose order round "
                   "the part is not the order they arrive from), fast links moved (pcb_net_roles pair, clock, "
                   "matched), vias, links off their far end's layer, the width moved. A pin that leaves on the "
                   "other layer than its pad's gets a via where pcb_fanout puts it (dry run; no_spot when it has "
                   "none). A single whose pin lies inside a bus's run on its layer goes to the other layer "
                   "(singles_boxed when both are taken). The port line runs round the courtyard a pitch out, past "
                   "the vias; per layer each link takes one band of slots at its pitch (track + clearance), the "
                   "bands in their order round the part, each as near its pins as the others let it. Answers per "
                   "link its layer, vias, nets in slot order, slots, room (pads to slots), arrives_mm / side; per "
                   "pin its slot, slot side, layer, via and gate (via / point), path_mm with lanes; ends_overflow "
                   "(a side whose bands at home need more than its length, per layer); breaks_left, "
                   "crossings_left. Read only",
                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string","description":"the part"},
                        "links":{"type":"array","items":{"type":"object","properties":{
                            "name":{"type":"string"},
                            "nets":{"type":"array","items":{"type":"string"}},
                            "from_mm":{"type":"array","items":{"type":"number"},"description":"[x, y]: where the link arrives from (default: its far pads' centre)"}}},
                            "description":"these links instead of pcb_layout_graph's buses; the part's other signal nets go alone"},
                        "max_links":{"type":"integer","default":10,"minimum":1,"maximum":14,"description":"buses whose layer choices are all tried; more: one at a time"},
                        "lanes":{"type":"boolean","default":false,"description":"per pin path_mm: pad, via, slot"}}})json"_json,
                   false, h_pcb_end_plan, 60 );
