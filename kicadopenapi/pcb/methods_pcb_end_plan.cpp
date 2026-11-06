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
#include <geometry/seg.h>
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
    VECTOR2I    knee;              ///< straight out from the pad past the vias and the pads' ways out
    std::vector<VECTOR2I> bends;   ///< its corners between the knee and the slot
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
    std::optional<int> fixedLayer;   ///< given: the layer it leaves on (0 F.Cu, 1 B.Cu), not searched
    double           arrivesS = 0;
    int              padLayer = 0;   ///< 0 F, 1 B, 2 through-hole only
    int              farLayer = 2;   ///< 0 F, 1 B, 2 both / unknown
    int              pitch = 1;      ///< widest track + clearance of its nets
    int              layer = 0;      ///< chosen: 0 F, 1 B
    int              homeSide = 0;
    std::vector<std::string> farParts;
    std::vector<std::string> order;   ///< given: its nets in the order their slots should come clockwise round the part
    int              orderInversions = 0;   ///< pairs of its slots against that order
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


/// @brief A layout's quality on one layer: runs of a link broken by another's pins (breaks), pairs of
/// links whose order round the part is not their arrivals' order (crossings, the fewest over the cyclic
/// shifts) and the pairs more that cross when the ring is cut in its widest stretch with no link and no
/// arrival (wraps: a band that has to go round the part)
struct LAYER_SCORE
{
    int breaks = 0;
    int crossings = 0;
    int wraps = 0;
};


LAYER_SCORE scoreLayer( const std::vector<PIN>& aPins, const std::vector<LINK>& aLinks,
                        const std::map<int, std::vector<int>>& aOnLayer )
{
    LAYER_SCORE score;
    std::vector<std::pair<double, int>> order;   // (s, link) of the pins on the layer

    for( const auto& [l, onPins] : aOnLayer )
    {
        for( int p : onPins )
            order.emplace_back( aPins[p].s, l );
    }

    if( aOnLayer.size() < 2 || order.empty() )
        return score;

    std::sort( order.begin(), order.end() );
    std::map<int, int> runs;

    for( size_t i = 0; i < order.size(); ++i )
    {
        const int prev = order[( i + order.size() - 1 ) % order.size()].second;

        if( order[i].second != prev )
            runs[order[i].second]++;
    }

    for( const auto& [l, onPins] : aOnLayer )
        score.breaks += std::max( 0, runs[l] - 1 );

    // each link's middle: of its run when it is one, else the circular mean of its pins
    std::vector<std::pair<double, int>> mids;

    for( const auto& [l, onPins] : aOnLayer )
    {
        double sx = 0, sy = 0;

        for( int p : onPins )
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

    // arrivals ranked, the ones at one place sharing a rank
    auto ranked = [&]( double aCut )
    {
        std::vector<std::pair<double, int>> byArrival;

        for( const auto& [mid, l] : mids )
            byArrival.emplace_back( std::fmod( aLinks[l].arrivesS - aCut + 8.0, 4.0 ), l );

        std::sort( byArrival.begin(), byArrival.end() );
        std::map<int, int> rank;

        for( size_t i = 0; i < byArrival.size(); ++i )
        {
            rank[byArrival[i].second] = i > 0 && byArrival[i].first - byArrival[i - 1].first < 1e-6
                                                ? rank[byArrival[i - 1].second] : (int) i;
        }

        return rank;
    };

    const int k = (int) mids.size();
    int       best = std::numeric_limits<int>::max();
    {
        std::map<int, int> rank = ranked( 0 );

        for( int c = 0; c < k; ++c )
        {
            std::vector<int> seq;

            for( const auto& [mid, l] : mids )
                seq.push_back( ( rank[l] - c + k ) % k );

            best = std::min( best, inversions( seq ) );
        }
    }

    // the ring cut in its widest stretch with no link's middle and no arrival: both orders read from there
    std::vector<double> marks;

    for( const auto& [mid, l] : mids )
    {
        marks.push_back( mid );
        marks.push_back( std::fmod( aLinks[l].arrivesS + 4.0, 4.0 ) );
    }

    std::sort( marks.begin(), marks.end() );
    double cut = 0, widest = -1;

    for( size_t i = 0; i < marks.size(); ++i )
    {
        const double next = i + 1 < marks.size() ? marks[i + 1] : marks.front() + 4.0;

        if( next - marks[i] > widest )
        {
            widest = next - marks[i];
            cut = marks[i] + widest / 2;
        }
    }

    std::vector<std::pair<double, int>> byMid;

    for( const auto& [mid, l] : mids )
        byMid.emplace_back( std::fmod( mid - cut + 8.0, 4.0 ), l );

    std::sort( byMid.begin(), byMid.end() );
    std::map<int, int> rank = ranked( cut );
    std::vector<int>   seq;

    for( const auto& [mid, l] : byMid )
        seq.push_back( rank[l] );

    score.wraps = std::max( 0, inversions( seq ) - best );
    score.crossings = best;
    return score;
}


/// @brief The chosen layers' score, lexicographic: breaks, crossings, wraps, fast links moved, vias, links
/// off their far end's layer, the width of the links moved
using SCORE = std::array<int, 7>;


/// @brief A bus pin's layer: its link's, or the other one where aFlip says so (a pin of an interleaved run
/// taken across so that each link's pins on a layer lie together)
int pinLayer( const std::vector<int>& aFlip, int aPin, int aLinkLayer )
{
    return aFlip.empty() || !aFlip[aPin] ? aLinkLayer : 1 - aLinkLayer;
}


SCORE scoreOf( const std::vector<PIN>& aPins, const std::vector<LINK>& aLinks, const std::vector<int>& aBuses,
               const std::vector<int>& aLayerOf, const std::vector<int>& aFlip = {} )
{
    SCORE                        score = { 0, 0, 0, 0, 0, 0, 0 };
    std::map<int, std::vector<int>> onLayer[2];

    for( size_t i = 0; i < aBuses.size(); ++i )
    {
        const LINK& link = aLinks[aBuses[i]];
        const int   layer = aLayerOf[i];
        int         vias = 0;

        for( int p : link.pins )
        {
            const int on = aPins[p].tht ? layer : pinLayer( aFlip, p, layer );
            onLayer[on][aBuses[i]].push_back( p );
            vias += !aPins[p].tht && ( aPins[p].layer == F_Cu ? 0 : 1 ) != on;
        }

        score[3] += link.fast && vias > 0;
        score[4] += vias;
        score[5] += link.farLayer != 2 && link.farLayer != layer;
        score[6] += vias > 0 ? (int) link.pins.size() : 0;
    }

    for( const std::map<int, std::vector<int>>& on : onLayer )
    {
        const LAYER_SCORE ls = scoreLayer( aPins, aLinks, on );
        score[0] += ls.breaks;
        score[1] += ls.crossings;
        score[2] += ls.wraps;
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

                if( given.contains( "layer" ) && given["layer"].is_string() )
                {
                    const std::string layer = given["layer"].get<std::string>();

                    if( layer != "F.Cu" && layer != "B.Cu" )
                        return KOPENAPI_RESULT::Error( 400, "links[].layer: F.Cu or B.Cu, not " + layer );

                    link->fixedLayer = layer == "B.Cu" ? 1 : 0;
                }

                for( const nlohmann::json& n : given.value( "order", nlohmann::json::array() ) )
                {
                    if( !n.is_string() || !link->nets.count( n.get<std::string>() ) )
                        return KOPENAPI_RESULT::Error( 400, "links[].order: net names of the link's nets" );

                    link->order.push_back( n.get<std::string>() );
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
        layerOf[i] = links[buses[i]].fixedLayer ? *links[buses[i]].fixedLayer : links[buses[i]].padLayer == 1 ? 1 : 0;

    auto fixedKept = [&]( const std::vector<int>& aOf )
    {
        for( size_t i = 0; i < buses.size(); ++i )
        {
            if( links[buses[i]].fixedLayer && aOf[i] != *links[buses[i]].fixedLayer )
                return false;
        }

        return true;
    };

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

            if( !fixedKept( tryOf ) )
                continue;

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
            if( links[buses[i]].fixedLayer )
                continue;

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

    std::vector<int> flip( pins.size(), 0 );

    // the buses' runs (arcs) per layer, each pin on its layer
    auto arcsOn = [&]( int aLayer )
    {
        std::vector<std::pair<double, double>> arcs;

        for( size_t i = 0; i < buses.size(); ++i )
        {
            std::vector<double> s;

            for( int p : links[buses[i]].pins )
            {
                if( ( pins[p].tht ? layerOf[i] : pinLayer( flip, p, layerOf[i] ) ) == aLayer )
                    s.push_back( pins[p].s );
            }

            if( s.empty() )
                continue;

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

    // a single inside a bus's run on its pad's layer goes to the other layer, if that one is free there
    {
        const std::vector<std::pair<double, double>> arcs[2] = { arcsOn( 0 ), arcsOn( 1 ) };

        for( int l : singles )
        {
            LINK&      link = links[l];
            const PIN& pin = pins[link.pins[0]];
            const int  own = pin.tht ? ( link.farLayer == 1 ? 1 : 0 ) : ( pin.layer == F_Cu ? 0 : 1 );
            link.layer = own;

            if( link.fixedLayer )
                link.layer = *link.fixedLayer;
            else if( inside( arcs[own], pin.s ) && !inside( arcs[1 - own], pin.s ) )
                link.layer = 1 - own;
        }
    }

    // breaks left (links alternating pin by pin on a side, e.g. a mux's inputs and outputs): single SMD pins,
    // of buses and singles, taken across to the other layer one at a time while that betters the score of
    // every link; crossings or wraps left with no break: only singles' pins (a single crossing a bus passes
    // under it)
    std::vector<int> scored = buses, scoredLayer = layerOf;

    for( int l : singles )
    {
        scored.push_back( l );
        scoredLayer.push_back( links[l].layer );
    }

    std::set<int> busSides;

    for( int l : buses )
    {
        for( int p : links[l].pins )
            busSides.insert( pins[p].side );
    }

    SCORE whole = scoreOf( pins, links, scored, scoredLayer, flip );
    int   flipped = 0;

    for( int round = 0; ( whole[0] > 0 || whole[1] > 0 || whole[2] > 0 ) && round < (int) pins.size(); ++round )
    {
        SCORE bestFlip = whole;
        int   at = -1;

        for( size_t i = whole[0] > 0 ? 0 : buses.size(); i < scored.size(); ++i )
        {
            if( links[scored[i]].fixedLayer )
                continue;

            for( int p : links[scored[i]].pins )
            {
                if( pins[p].tht || !busSides.count( pins[p].side ) )
                    continue;

                flip[p] = 1 - flip[p];
                const SCORE s = scoreOf( pins, links, scored, scoredLayer, flip );
                flip[p] = 1 - flip[p];
                variants++;

                if( s < bestFlip )
                {
                    bestFlip = s;
                    at = p;
                }
            }
        }

        if( at < 0 )
            break;

        flip[at] = 1 - flip[at];
        flipped += flip[at] ? 1 : -1;
        whole = bestFlip;
    }

    // under the body: a link's pins taken to the layer without the part's pads go back from their vias under
    // the body, each on its own leg along its row, and leave at a side without pins (a TSSOP's ends), the pin
    // nearest that side on the leg nearest its row. Taken while that betters the score of every link
    std::vector<int>    underTo( pins.size(), -1 );
    std::vector<double> legAt( pins.size(), 0 );   // the leg's place across its row (x for a west / east row)
    std::vector<double> homeS( pins.size(), 0 );
    int                 underLinks = 0;

    for( size_t p = 0; p < pins.size(); ++p )
        homeS[p] = pins[p].s;

    {
        // each pad of the part on its side; the sides with none are free, the rows' inner edges bound the legs
        bool   padded[4] = { false, false, false, false };
        double inner[4] = { -1e18, 1e18, 1e18, -1e18 };   // N: max bottom, E: min left, S: min top, W: max right

        auto sideOf = [&]( const PAD* aPad )
        {
            const VECTOR2I d = aPad->GetPosition() - centre;
            const BOX2I    land = aPad->GetBoundingBox();

            if( land.GetWidth() > 1.2 * land.GetHeight() && std::abs( d.x ) > hw / 2 )
                return d.x > 0 ? 1 : 3;

            if( land.GetHeight() > 1.2 * land.GetWidth() && std::abs( d.y ) > hh / 2 )
                return d.y > 0 ? 2 : 0;

            if( std::abs( d.x ) / std::max( hw, 1.0 ) >= std::abs( d.y ) / std::max( hh, 1.0 ) )
                return d.x > 0 ? 1 : 3;

            return d.y > 0 ? 2 : 0;
        };

        for( PAD* pad : fp->Pads() )
        {
            const int   side = sideOf( pad );
            const BOX2I land = pad->GetBoundingBox();
            padded[side] = true;

            switch( side )
            {
            case 0: inner[0] = std::max( inner[0], (double) land.GetBottom() ); break;
            case 1: inner[1] = std::min( inner[1], (double) land.GetLeft() ); break;
            case 2: inner[2] = std::min( inner[2], (double) land.GetTop() ); break;
            default: inner[3] = std::max( inner[3], (double) land.GetRight() ); break;
            }
        }

        // a free row side: its legs bounded by the body's far edge
        if( !padded[0] ) inner[0] = base.GetTop();
        if( !padded[1] ) inner[1] = base.GetRight();
        if( !padded[2] ) inner[2] = base.GetBottom();
        if( !padded[3] ) inner[3] = base.GetLeft();

        // no pad of the part on a layer between the rows: the body's underside free there
        bool open[2] = { true, true };
        const BOX2I channel( VECTOR2I( (int) inner[3], (int) inner[0] ),
                             VECTOR2I( (int) ( inner[1] - inner[3] ), (int) ( inner[2] - inner[0] ) ) );

        for( PAD* pad : fp->Pads() )
        {
            for( int layer = 0; layer < 2; ++layer )
            {
                if( pad->IsOnLayer( layer == 0 ? F_Cu : B_Cu ) && channel.Intersects( pad->GetBoundingBox() ) )
                    open[layer] = false;
            }
        }

        // the legs of the pins sent under: false when two rows' legs would meet
        auto lay = [&]()
        {
            for( size_t p = 0; p < pins.size(); ++p )
                pins[p].s = homeS[p];

            std::map<std::tuple<int, int, int>, std::vector<int>> groups;

            for( size_t p = 0; p < pins.size(); ++p )
            {
                if( underTo[p] >= 0 )
                    groups[{ pins[p].side, underTo[p], pinLayer( flip, (int) p, links[pins[p].link].layer ) }].push_back( (int) p );
            }

            double deepest[4][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };

            for( auto& [key, group] : groups )
            {
                const int    side = std::get<0>( key ), to = std::get<1>( key ), layer = std::get<2>( key );
                const double toward = to == 0 ? -1e18 : to == 1 ? 1e18 : to == 2 ? 1e18 : -1e18;
                const bool   rowAcross = side % 2 == 1;   // a west / east row: legs at an x

                std::sort( group.begin(), group.end(),
                           [&]( int x, int y )
                           {
                               const VECTOR2I a = pins[x].pad->GetPosition(), b = pins[y].pad->GetPosition();
                               const double   ax = rowAcross ? a.y : a.x, bx = rowAcross ? b.y : b.x;
                               return toward > 0 ? ax > bx : ax < bx;
                           } );

                const double inward = side == 0 || side == 3 ? 1 : -1;
                double       depth = 0;

                for( int p : group )
                {
                    depth += links[pins[p].link].pitch;
                    legAt[p] = inner[side] + inward * depth;
                    const VECTOR2I exit = rowAcross ? VECTOR2I( (int) legAt[p], to == 0 ? port.rect.GetTop() : port.rect.GetBottom() )
                                                    : VECTOR2I( to == 1 ? port.rect.GetRight() : port.rect.GetLeft(), (int) legAt[p] );
                    pins[p].s = port.projectS( exit, to );
                }

                deepest[side][layer] = std::max( deepest[side][layer], depth + pitchMax );
            }

            for( int layer = 0; layer < 2; ++layer )
            {
                if( deepest[3][layer] + deepest[1][layer] > inner[1] - inner[3] || deepest[0][layer] + deepest[2][layer] > inner[2] - inner[0] )
                    return false;
            }

            return true;
        };

        std::vector<bool> under( links.size(), false );

        // a link's pins that may go under to aTo: on the layer without pads, by a via, on a row next to aTo
        auto pinsUnder = [&]( int aLink, int aTo )
        {
            std::vector<int> out;

            for( int p : links[aLink].pins )
            {
                const PIN& pin = pins[p];
                const int  layer = pinLayer( flip, p, links[aLink].layer );

                if( pin.tht || ( pin.layer == F_Cu ? 0 : 1 ) == layer || !open[layer] || ( pin.side + 2 ) % 4 == aTo
                    || pin.side == aTo )
                {
                    continue;
                }

                out.push_back( p );
            }

            return out;
        };

        for( int round = 0; ( whole[0] > 0 || whole[1] > 0 || whole[2] > 0 ) && round < (int) links.size(); ++round )
        {
            SCORE bestUnder = whole;
            int   atLink = -1, atTo = -1;

            for( int l : scored )
            {
                if( under[l] )
                    continue;

                for( int to = 0; to < 4; ++to )
                {
                    if( padded[to] )
                        continue;

                    const std::vector<int> moving = pinsUnder( l, to );

                    if( moving.empty() )
                        continue;

                    for( int p : moving )
                        underTo[p] = to;

                    if( lay() )
                    {
                        const SCORE s = scoreOf( pins, links, scored, scoredLayer, flip );
                        variants++;

                        if( s < bestUnder )
                        {
                            bestUnder = s;
                            atLink = l;
                            atTo = to;
                        }
                    }

                    for( int p : moving )
                        underTo[p] = -1;
                }
            }

            if( atLink < 0 )
                break;

            for( int p : pinsUnder( atLink, atTo ) )
                underTo[p] = atTo;

            under[atLink] = true;
            underLinks++;
            whole = bestUnder;
        }

        lay();
    }

    best[0] = whole[0];
    best[1] = whole[1];
    best[2] = whole[2];

    auto layerOfPin = [&]( int aPin )
    {
        return pins[aPin].tht ? links[pins[aPin].link].layer : pinLayer( flip, aPin, links[pins[aPin].link].layer );
    };

    // a single still inside a bus's run on its layer: boxed in
    nlohmann::json boxedSingles = nlohmann::json::array();
    {
        const std::vector<std::pair<double, double>> arcs[2] = { arcsOn( 0 ), arcsOn( 1 ) };

        for( int l : singles )
        {
            const int p = links[l].pins[0];

            if( inside( arcs[layerOfPin( p )], pins[p].s ) )
                boxedSingles.push_back( links[l].name );
        }
    }

    for( LINK& link : links )
    {
        for( int p : link.pins )
        {
            PIN& pin = pins[p];
            pin.slotLayer = layerOfPin( p ) == 0 ? F_Cu : B_Cu;
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

            // no via: it cannot go under
            if( const size_t p = &pin - pins.data(); !pin.via && underTo[p] >= 0 )
            {
                underTo[p] = -1;
                pin.s = homeS[p];
            }
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

    // and past every SMD pad's way out (pcb_route keeps it free of other nets till the pad's route is in)
    for( const PIN& pin : pins )
    {
        if( pin.tht )
            continue;

        const BOX2I land = pin.pad->GetBoundingBox();
        const int   reach = KopenapiPadEscapeDepth( board, pin.pad ) + std::max( ds.m_MinClearance, 1 ) + pitchMax / 2;

        switch( pin.side )
        {
        case 0: out[0] = std::max( out[0], base.GetTop() - land.GetTop() + reach ); break;
        case 1: out[1] = std::max( out[1], land.GetRight() - base.GetRight() + reach ); break;
        case 2: out[2] = std::max( out[2], land.GetBottom() - base.GetBottom() + reach ); break;
        default: out[3] = std::max( out[3], base.GetLeft() - land.GetLeft() + reach ); break;
        }
    }

    // a facing neighbour's pads: half the gap is this part's, the port line kept inside it (not nearer than its vias)
    int room[4] = { INT_MAX, INT_MAX, INT_MAX, INT_MAX };   // per side: how far out this part's half reaches

    {
        int inside[4] = { 0, 0, 0, 0 };   // the vias' reach, the least the port line can be

        for( const PIN& pin : pins )
        {
            if( !pin.via )
                continue;

            const int reach = viaDia / 2 + std::max( ds.m_MinClearance, 1 ) + pitchMax / 2;
            inside[0] = std::max( inside[0], base.GetTop() - pin.via->y + reach );
            inside[1] = std::max( inside[1], pin.via->x - base.GetRight() + reach );
            inside[2] = std::max( inside[2], pin.via->y - base.GetBottom() + reach );
            inside[3] = std::max( inside[3], base.GetLeft() - pin.via->x + reach );
        }

        int gap[4] = { INT_MAX, INT_MAX, INT_MAX, INT_MAX };

        for( FOOTPRINT* other : board->Footprints() )
        {
            if( other == fp )
                continue;

            // its courtyard as this part's (with its pads): the space between the two halved
            const SHAPE_POLY_SET& yard = other->GetCourtyard( other->IsFlipped() ? B_CrtYd : F_CrtYd );
            BOX2I                 b = yard.OutlineCount() ? yard.BBox() : other->GetBoundingBox( false );

            for( PAD* pad : other->Pads() )
                b.Merge( pad->GetBoundingBox() );

            if( b.Intersects( base ) )
                continue;

            if( b.GetRight() > base.GetLeft() && b.GetLeft() < base.GetRight() )
            {
                if( b.GetBottom() <= base.GetTop() )
                    gap[0] = std::min( gap[0], base.GetTop() - b.GetBottom() );
                else if( b.GetTop() >= base.GetBottom() )
                    gap[2] = std::min( gap[2], b.GetTop() - base.GetBottom() );
            }

            if( b.GetBottom() > base.GetTop() && b.GetTop() < base.GetBottom() )
            {
                if( b.GetLeft() >= base.GetRight() )
                    gap[1] = std::min( gap[1], b.GetLeft() - base.GetRight() );
                else if( b.GetRight() <= base.GetLeft() )
                    gap[3] = std::min( gap[3], base.GetLeft() - b.GetRight() );
            }
        }

        for( int side = 0; side < 4; ++side )
        {
            if( gap[side] != INT_MAX )
            {
                room[side] = gap[side] / 2;
                out[side] = std::max( inside[side], std::min( out[side], room[side] ) );
            }
        }
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
            ITEM item{ (int) l, {}, 0, 0 };

            for( int p : links[l].pins )
            {
                if( layerOfPin( p ) == layer )
                    item.pins.push_back( p );
            }

            if( item.pins.empty() )
                continue;

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

            size_t first = ( at + 1 ) % item.pins.size();

            // an order given: the start round the part whose slots go most nearly in that order (the band
            // round the part the other way), the widest gap's when as good; a start only where the pins break
            // (a corner, or a gap wider than their pitch), never inside a row, which would wrap the band round
            if( !links[l].order.empty() && item.pins.size() >= 2 )
            {
                const size_t n = item.pins.size();
                const size_t widest = first;
                double       pitch = 1e18;

                for( size_t k = 0; k < n; ++k )
                    pitch = std::min( pitch, ahead( pins[item.pins[( k + n - 1 ) % n]].s, pins[item.pins[k]].s ) );

                auto breaksAt = [&]( size_t k )
                {
                    const int a = item.pins[( k + n - 1 ) % n], b = item.pins[k];
                    return k == widest || pins[a].side != pins[b].side || ahead( pins[a].s, pins[b].s ) > 1.5 * pitch;
                };

                std::map<std::string, int> place;

                for( size_t k = 0; k < links[l].order.size(); ++k )
                    place[links[l].order[k]] = (int) k;

                auto against = [&]( size_t aFirst )
                {
                    std::vector<int> seq;

                    for( size_t k = 0; k < item.pins.size(); ++k )
                    {
                        auto it = place.find( pins[item.pins[( aFirst + k ) % item.pins.size()]].net );

                        if( it != place.end() )
                            seq.push_back( it->second );
                    }

                    return inversions( seq );
                };

                int best = against( first );

                for( size_t k = 0; k < n; ++k )
                {
                    if( const int inv = breaksAt( k ) ? against( k ) : best; inv < best )
                    {
                        best = inv;
                        first = k;
                    }
                }

                links[l].orderInversions += best;
            }

            std::rotate( item.pins.begin(), item.pins.begin() + first, item.pins.end() );
            const double from = line.distanceOf( pins[item.pins.front()].s );
            double       span = line.distanceOf( pins[item.pins.back()].s ) - from;

            if( span < 0 )
                span += line.perimeter();

            item.width = (double) item.pins.size() * links[l].pitch;
            item.want = std::fmod( from + span / 2, line.perimeter() );

            // a bus on the side facing away from where it arrives from: round the corner nearer that,
            // its slots on the next side just past the corner
            const int side = pins[item.pins.front()].side;
            const int arrivesSide = std::clamp( (int) std::floor( links[l].arrivesS ), 0, 3 );

            if( links[l].kind == "bus" && arrivesSide == ( side + 2 ) % 4
                && std::all_of( item.pins.begin(), item.pins.end(),
                                [&]( int p ) { return pins[p].side == side && underTo[p] < 0; } ) )
            {
                const double f = links[l].arrivesS - arrivesSide;
                const double past = item.width / 2 + links[l].pitch;

                if( f < 0.5 )
                    item.want = line.distanceOf( ( side + 1 ) % 4 ) + past;
                else
                    item.want = std::fmod( line.distanceOf( ( side + 3 ) % 4 ) + line.length( ( side + 3 ) % 4 ) - past,
                                           line.perimeter() );
            }

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
                pins[item.pins[j]].slotSide = underTo[item.pins[j]] >= 0 ? underTo[item.pins[j]] : side;
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

    // each pin straight out to its knee, then at 45 degrees to its bend in its slot's line, then straight out to
    // the slot. Per side and layer the diagonals are parallel: a pin behind another going the same way turns
    // further out, so that the two stay a pitch apart. A slot on the next side: round the corner on a leg.
    // The port line goes out per side as far as the furthest bend.
    {
        const BOX2I               clear = line.rect;
        static const VECTOR2I     outward[4] = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 } };
        std::vector<double>       turn( pins.size(), 0 );   // knee: this far out past the clear line
        std::vector<bool>         bent( pins.size(), false );
        double                    extra[4][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };   // per side and slot layer (F.Cu, other)

        auto layerOf = []( PCB_LAYER_ID aLayer ) { return aLayer == F_Cu ? 0 : 1; };

        auto along = [&]( const VECTOR2I& aPt, int aSide ) { return (double) ( aSide % 2 == 0 ? aPt.x : aPt.y ); };

        auto kneeAt = [&]( const PIN& aPin, double aOut )
        {
            const VECTOR2I at = aPin.pad->GetPosition();
            const int      o = (int) std::lround( aOut );

            switch( aPin.side )
            {
            case 0: return VECTOR2I( at.x, clear.GetTop() - o );
            case 1: return VECTOR2I( clear.GetRight() + o, at.y );
            case 2: return VECTOR2I( at.x, clear.GetBottom() + o );
            default: return VECTOR2I( clear.GetLeft() - o, at.y );
            }
        };

        // the straight legs, knee to slot, as far out as the widest step across
        double straight[4] = { 0, 0, 0, 0 };

        for( const PIN& pin : pins )
        {
            if( pin.link >= 0 && pin.slotSide == pin.side )
                straight[pin.side] = std::max( straight[pin.side], std::abs( along( pin.slot, pin.side ) - along( pin.pad->GetPosition(), pin.side ) ) );
        }

        auto straightLeg = [&]( const PIN& aPin )
        {
            return SEG( kneeAt( aPin, 0 ), aPin.slot + outward[aPin.side] * (int) std::lround( straight[aPin.side] ) );
        };

        for( int side = 0; side < 4; ++side )
        {
            for( PCB_LAYER_ID layer : { F_Cu, B_Cu } )
            {
                for( int dir : { 1, -1 } )
                {
                    // the pins going dir along the side, the one ahead first
                    std::vector<int> group;

                    for( size_t p = 0; p < pins.size(); ++p )
                    {
                        const PIN& pin = pins[p];

                        if( pin.link < 0 || pin.side != side || pin.slotSide != side || pin.slotLayer != layer )
                            continue;

                        const double shift = along( pin.slot, side ) - along( pin.pad->GetPosition(), side );

                        if( shift * dir > 0 )
                            group.push_back( (int) p );
                    }

                    std::sort( group.begin(), group.end(),
                               [&]( int x, int y )
                               {
                                   return along( pins[x].pad->GetPosition(), side ) * dir > along( pins[y].pad->GetPosition(), side ) * dir;
                               } );

                    // straight legs that come nearer than a pitch: the group goes by parallel diagonals
                    bool closer = false;

                    for( size_t k = 1; k < group.size() && !closer; ++k )
                    {
                        const PIN& ahead = pins[group[k - 1]];
                        const PIN& behind = pins[group[k]];
                        const int  pitch = std::max( links[ahead.link].pitch, links[behind.link].pitch );
                        closer = straightLeg( ahead ).Distance( straightLeg( behind ) ) < pitch - pcbIUScale.mmToIU( 0.001 );
                    }

                    if( !closer )
                        continue;

                    for( int p : group )
                        bent[p] = true;

                    for( size_t k = 1; k < group.size(); ++k )
                    {
                        const PIN&   ahead = pins[group[k - 1]];
                        const PIN&   behind = pins[group[k]];
                        const double gap = std::abs( along( ahead.pad->GetPosition(), side ) - along( behind.pad->GetPosition(), side ) );
                        const double pitch = std::max( links[ahead.link].pitch, links[behind.link].pitch );
                        turn[group[k]] = std::max( 0.0, turn[group[k - 1]] - gap + pitch * std::sqrt( 2.0 ) );
                    }
                }
            }
        }

        // pins whose slots lie on the next side round the corner on nested legs, the pin nearest the corner
        // innermost (its slot the furthest along: the order round the part is kept)
        std::vector<double> round( pins.size(), 0 );

        struct LEGS
        {
            int    side, layer;
            double lo, hi;   ///< along the side: from the furthest of its pins to the corner
            double push;     ///< past the clear line, the outermost leg and a pitch
        };

        std::vector<LEGS> legs;
        {
            std::map<std::tuple<int, int, int>, std::vector<int>> corners;

            for( size_t p = 0; p < pins.size(); ++p )
            {
                const PIN& pin = pins[p];

                if( pin.link >= 0 && underTo[p] < 0 && pin.slotSide != pin.side && ( pin.slotSide + 2 ) % 4 != pin.side )
                    corners[{ pin.side, pin.slotSide, pin.slotLayer == F_Cu ? 0 : 1 }].push_back( (int) p );
            }

            for( auto& [key, group] : corners )
            {
                const int side = std::get<0>( key ), to = std::get<1>( key ), layer = std::get<2>( key );

                // the corner's place along the side: the slot side's clear line
                const double cornerAt = to == 0 ? clear.GetTop() : to == 1 ? clear.GetRight() : to == 2 ? clear.GetBottom() : clear.GetLeft();

                std::sort( group.begin(), group.end(),
                           [&]( int x, int y )
                           {
                               return std::abs( along( pins[x].pad->GetPosition(), side ) - cornerAt )
                                      < std::abs( along( pins[y].pad->GetPosition(), side ) - cornerAt );
                           } );

                double o = 0;

                for( int p : group )
                {
                    o += links[pins[p].link].pitch;
                    round[p] = o;
                    extra[side][layer] = std::max( extra[side][layer], o + links[pins[p].link].pitch );
                    extra[to][layer] = std::max( extra[to][layer], o + links[pins[p].link].pitch );
                }

                const double farthest = along( pins[group.back()].pad->GetPosition(), side );
                legs.push_back( { side, layer, std::min( farthest, cornerAt ), std::max( farthest, cornerAt ),
                                  o + links[pins[group.back()].link].pitch } );
            }
        }

        std::vector<double> own( pins.size(), 0 );   // a pin's push for its own bend along its side

        for( size_t p = 0; p < pins.size(); ++p )
        {
            PIN&           pin = pins[p];
            const VECTOR2I at = pin.pad->GetPosition();

            pin.knee = kneeAt( pin, 0 );

            if( pin.link >= 0 && pin.slotSide == pin.side )
            {
                const double shift = std::abs( along( pin.slot, pin.side ) - along( at, pin.side ) );
                double& e = extra[pin.side][layerOf( pin.slotLayer )];
                e = std::max( e, turn[p] + shift );
                own[p] = turn[p] + shift;
            }
        }

        // one port line per side where the furthest bend's push fits this part's half of a facing gap,
        // else a pin on its own side pushed only past the legs in front of it (and its own bend), the others
        // as far as their layer's bends
        bool reach[4];

        for( int side = 0; side < 4; ++side )
            reach[side] = room[side] == INT_MAX || out[side] + std::max( extra[side][0], extra[side][1] ) <= room[side];

        for( size_t p = 0; p < pins.size(); ++p )
        {
            PIN& pin = pins[p];

            if( pin.link < 0 )
                continue;

            if( pin.slotSide == pin.side )
            {
                const VECTOR2I out = outward[pin.side];
                const int      shift = (int) std::lround( along( pin.slot, pin.side ) - along( pin.knee, pin.side ) );
                pin.knee += out * (int) std::lround( turn[p] );

                if( shift != 0 && bent[p] )
                {
                    const VECTOR2I across = pin.side % 2 == 0 ? VECTOR2I( shift, 0 ) : VECTOR2I( 0, shift );
                    pin.bends.push_back( pin.knee + across + out * std::abs( shift ) );
                }
            }
            else if( underTo[p] >= 0 )
            {
                // back from the via under the body to its leg, along the leg out past the free side
                const VECTOR2I at = pin.pad->GetPosition();
                const int      leg = (int) std::lround( legAt[p] );
                const bool     rowAcross = pin.side % 2 == 1;
                pin.knee = rowAcross ? VECTOR2I( leg, at.y ) : VECTOR2I( at.x, leg );

                switch( pin.slotSide )
                {
                case 0: pin.slot = VECTOR2I( leg, clear.GetTop() ); break;
                case 1: pin.slot = VECTOR2I( clear.GetRight(), leg ); break;
                case 2: pin.slot = VECTOR2I( leg, clear.GetBottom() ); break;
                default: pin.slot = VECTOR2I( clear.GetLeft(), leg ); break;
                }
            }
            else if( round[p] > 0 )
            {
                // round the corner: out to its leg, along the leg past the corner, along the slot's side, out
                const double o = round[p];
                pin.knee = kneeAt( pin, o );
                const VECTOR2I d = pin.slot + outward[pin.slotSide] * (int) std::lround( o );
                const VECTOR2I c = pin.side % 2 == 0 ? VECTOR2I( d.x, pin.knee.y ) : VECTOR2I( pin.knee.x, d.y );
                pin.bends.push_back( c );

                if( d != c )
                    pin.bends.push_back( d );
            }

            const double* e = extra[pin.slotSide];
            double        push = reach[pin.slotSide] ? std::max( e[0], e[1] ) : e[layerOf( pin.slotLayer )];

            if( !reach[pin.slotSide] && pin.slotSide == pin.side && round[p] == 0 && underTo[p] < 0 )
            {
                const double a = along( pin.pad->GetPosition(), pin.side );
                push = own[p];

                for( const LEGS& leg : legs )
                {
                    if( leg.side == pin.side && leg.layer == layerOf( pin.slotLayer ) && a >= leg.lo && a <= leg.hi )
                        push = std::max( push, leg.push );
                }
            }

            pin.slot += outward[pin.slotSide] * (int) std::lround( push );
        }

        int e[4];

        for( int side = 0; side < 4; ++side )
            e[side] = (int) std::lround( std::max( extra[side][0], extra[side][1] ) );

        line.rect = BOX2I( VECTOR2I( clear.GetLeft() - e[3], clear.GetTop() - e[0] ),
                           VECTOR2I( clear.GetWidth() + e[1] + e[3], clear.GetHeight() + e[0] + e[2] ) );
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

            if( underTo[p] >= 0 )
                row["under"] = true;

            if( pin.via )
                row["via_mm"] = pointMm( *pin.via );

            row["knee_mm"] = pointMm( pin.knee );

            if( !pin.bends.empty() )
            {
                row["bends_mm"] = nlohmann::json::array();

                for( const VECTOR2I& b : pin.bends )
                    row["bends_mm"].push_back( pointMm( b ) );
            }

            if( withPaths )
            {
                nlohmann::json path = nlohmann::json::array( { pointMm( pin.pad->GetPosition() ) } );

                if( pin.via )
                    path.push_back( pointMm( *pin.via ) );

                path.push_back( pointMm( pin.knee ) );

                for( const VECTOR2I& b : pin.bends )
                    path.push_back( pointMm( b ) );

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

        if( !link.order.empty() )
            linksOut.back()["order_inversions"] = link.orderInversions;
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
        { "wraps_left", best[2] },
        { "fast_moved", best[3] },
        { "pins_across", flipped },
        { "links_under", underLinks },
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
                   "the part is not the order they arrive from), wraps (pairs more that cross when the ring round the part is cut where "
                   "no link is: a band that has to go round the part), fast links moved (pcb_net_roles pair, clock, "
                   "matched), vias, links off their far end's layer, the width moved. A pin that leaves on the "
                   "other layer than its pad's gets a via where pcb_fanout puts it (dry run; no_spot when it has "
                   "none). A single whose pin lies inside a bus's run on its layer goes to the other layer. Breaks then left "
                   "(links alternating pin by pin on a side, e.g. a mux's inputs and outputs; singles counted) are taken "
                   "out one pin at a time: an SMD pin of a side with bus pins crosses to the other layer through its via "
                   "while that betters the score of every link (pins_across); crossings or wraps then left: singles' pins only. Then, "
                   "while that betters the score, a link whose pins left on the layer without the part's pads goes back "
                   "from its vias under the body, each pin on its own leg along its row, and leaves at a side without "
                   "pins (a TSSOP's ends; the pin nearest that side on the leg nearest its row; links_under, pins' "
                   "under); a bus whose pins all face away from its arrival rounds the corner nearer it, its slots "
                   "just past it; a facing part's pads: the port line inside this part's half of the gap between the "
                   "courtyards; singles_boxed: singles still inside a "
                   "bus's run on their layer. Each pin goes straight out to its knee, past the courtyard "
                   "a pitch, the vias and the pads' ways out (as pcb_route keeps them), then at 45 degrees to its bend "
                   "in line with its slot, then straight out to the slot (bends_mm) where the straight ways of the pins "
                   "going one way on a side and layer would come nearer than a pitch, else straight to the slot; such "
                   "diagonals are parallel, a pin behind another turning further out so that the two stay a pitch "
                   "apart; a pin whose slot lies on the next side rounds the corner on its own leg (bends_mm: the corner, "
                   "then in line with the slot), the pin nearest the corner innermost; the port line lies past the furthest bend; per layer each link takes one band of slots at its pitch (track + clearance), the "
                   "bands in their order round the part, each as near its pins as the others let it. Answers per "
                   "link its layer, vias, nets in slot order, slots, room (pads to slots), arrives_mm / side; per "
                   "pin its slot, slot side, layer, via, knee and gate (via / point), path_mm with lanes; ends_overflow "
                   "(a side whose bands at home need more than its length, per layer); breaks_left, "
                   "crossings_left, wraps_left. Read only",
                   R"json({"type":"object","required":["ref"],"properties":{
                        "ref":{"type":"string","description":"the part"},
                        "links":{"type":"array","items":{"type":"object","properties":{
                            "name":{"type":"string"},
                            "nets":{"type":"array","items":{"type":"string"}},
                            "from_mm":{"type":"array","items":{"type":"number"},"description":"[x, y]: where the link arrives from (default: its far pads' centre)"},
                            "layer":{"type":"string","enum":["F.Cu","B.Cu"],"description":"a bus's layer fixed (e.g. the layer its other end's plan chose): not searched, its pins on the other layer through vias"},
                            "order":{"type":"array","items":{"type":"string"},"description":"its nets in the order their slots should come clockwise round the part (e.g. the order the bus arrives in from its other end): its band starts round the part where its slots go most nearly so, at a break of its pins only (a corner, or a gap wider than their pitch; never inside a row, which would wrap the band round the part) (order_inversions: the pairs still against it)"}}},
                            "description":"these links instead of pcb_layout_graph's buses; the part's other signal nets go alone"},
                        "max_links":{"type":"integer","default":10,"minimum":1,"maximum":14,"description":"buses whose layer choices are all tried; more: one at a time"},
                        "lanes":{"type":"boolean","default":false,"description":"per pin path_mm: pad, via, knee, bends, slot"}}})json"_json,
                   false, h_pcb_end_plan, 60 );
