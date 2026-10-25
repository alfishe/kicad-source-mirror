/// @file methods_pcb_net_roles.cpp
/// @brief kicadopenapi pcb_net_roles / pcb_net_roles_set: what each net is for the router
/// (docs/inprogress/2026-10-09-routing-profiles/requirements.md §1).  A primary role per net with
/// tags, the source and basis of the decision; differential pairs, matched groups, rails and bundles
/// (buses).  Sources, highest first: the project's own data (pcb_net_roles_set, `kicadopenapi.net_roles`
/// in the .kicad_pro), the board's custom DRC rules, net classes, pins, schematic bus aliases and group
/// names, net names, topology (lines traced through series parts).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <drc/drc_rule.h>
#include <footprint.h>
#include <kiway.h>
#include <kicadopenapi_util.h>
#include <netclass.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <project.h>
#include <project/project_file.h>
#include <string_utils.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <regex>
#include <set>


static const char* const PROJECT_KEY = "kicadopenapi.net_roles";

static const std::set<std::string> ROLES = { "ground", "power", "power_hc", "pair", "clock", "matched", "bus",
                                             "analog", "analog_hs", "switching", "signal", "nc", "fixed" };


static std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


static std::string netName( const wxString& aName )
{
    return str( UnescapeString( aName ) );
}


static std::string upper( std::string aText )
{
    for( char& c : aText )
        c = static_cast<char>( std::toupper( static_cast<unsigned char>( c ) ) );

    return aText;
}


static double mm( int aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1e4 ) / 1e4;
}


/// @brief The net name without its hierarchical path
static std::string bareName( const std::string& aName )
{
    const size_t slash = aName.size() > 1 ? aName.find_last_of( '/', aName.size() - 2 ) : std::string::npos;
    return slash == std::string::npos ? aName : aName.substr( slash + 1 );
}


static std::string refPrefix( const std::string& aRef )
{
    size_t n = 0;

    while( n < aRef.size() && std::isalpha( static_cast<unsigned char>( aRef[n] ) ) )
        n++;

    return upper( aRef.substr( 0, n ) );
}


namespace
{
struct NET_FACTS
{
    std::string              name;
    std::string              role = "signal";
    std::string              source = "default";
    std::set<std::string>    tags;
    nlohmann::json           basis = nlohmann::json::array();
    nlohmann::json           rules = nlohmann::json::object();
    std::set<std::string>    refs;          ///< footprints with a pad on the net
    const PAD*               pad = nullptr;
    std::string              netClass;
    double                   widthMm = 0;
    double                   currentA = -1;
    nlohmann::json           edge;                  ///< driver edge data from the project (pcb_net_roles_set)
    nlohmann::json           termination;
    bool                     fixedRole = false;     ///< set by the project: nothing else changes it

    void Set( const std::string& aRole, const std::string& aSource, const std::string& aWhy )
    {
        if( fixedRole )
            return;

        role = aRole;
        source = aSource;
        basis.push_back( aWhy );
    }
};


struct UNION_FIND
{
    std::vector<int> parent;

    explicit UNION_FIND( size_t aCount ) : parent( aCount ) { std::iota( parent.begin(), parent.end(), 0 ); }

    int Find( int aI ) { return parent[aI] == aI ? aI : parent[aI] = Find( parent[aI] ); }

    void Join( int aA, int aB ) { parent[Find( aA )] = Find( aB ); }
};
} // namespace


/// @brief A part that passes a signal straight through: 2-pad resistor / ferrite / 0 ohm, or a
/// resistor array (pad i to pad n + 1 - i)
static bool isSeriesPart( const FOOTPRINT* aFp )
{
    const std::string p = refPrefix( str( aFp->GetReference() ) );
    const size_t      pads = aFp->Pads().size();

    if( pads == 2 && ( p == "R" || p == "FB" ) )
        return true;

    return pads >= 4 && pads % 2 == 0 && ( p == "RN" || p == "RA" || p == "RP" );
}


/// @brief A passive that never ends a bundle (series parts, shunt capacitors, pull-ups)
static bool isPassive( const FOOTPRINT* aFp )
{
    static const std::set<std::string> passive = { "R", "RN", "RA", "RP", "C", "CN", "L", "FB", "TP" };
    return passive.count( refPrefix( str( aFp->GetReference() ) ) ) > 0;
}


/// @brief Pairs of pads a series part joins
static std::vector<std::pair<const PAD*, const PAD*>> seriesLinks( const FOOTPRINT* aFp )
{
    std::vector<const PAD*> pads( aFp->Pads().begin(), aFp->Pads().end() );
    std::sort( pads.begin(), pads.end(),
               []( const PAD* a, const PAD* b ) { return KopenapiNaturalLess( str( a->GetNumber() ), str( b->GetNumber() ) ); } );

    std::vector<std::pair<const PAD*, const PAD*>> out;

    for( size_t i = 0; i < pads.size() / 2; i++ )
        out.emplace_back( pads[i], pads[pads.size() - 1 - i] );

    return out;
}


/// @brief Stem of a series name: EVE_G3 -> EVE_G, D[7] -> D
static std::string nameStem( const std::string& aBare )
{
    static const std::regex re( R"(^(.*?[^0-9])\[?([0-9]+)\]?$)" );
    std::smatch             m;

    return std::regex_match( aBare, m, re ) ? m[1].str() : aBare;
}


/// @brief Members of a bus alias entry: "IO[0..3]" -> IO0..IO3, a name as it is
static std::vector<std::string> expandMember( const std::string& aMember )
{
    static const std::regex re( R"(^(.*)\[([0-9]+)\.\.([0-9]+)\]$)" );
    std::smatch             m;

    if( !std::regex_match( aMember, m, re ) )
        return { aMember };

    std::vector<std::string> out;
    const int                a = std::stoi( m[2].str() ), b = std::stoi( m[3].str() );

    for( int k = std::min( a, b ); k <= std::max( a, b ); k++ )
        out.push_back( m[1].str() + std::to_string( k ) );

    return out;
}


static bool isPowerRole( const std::string& aRole )
{
    return aRole == "ground" || aRole == "power" || aRole == "power_hc";
}


static PROJECT* boardProject( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard )
{
    if( aBoard->GetProject() )
        return aBoard->GetProject();

    return aCtx.kiway ? &aCtx.kiway->Prj() : nullptr;
}


static nlohmann::json projectRoles( PROJECT* aProject )
{
    if( aProject && aProject->GetProjectFile().Contains( PROJECT_KEY ) )
    {
        std::optional<nlohmann::json> j = aProject->GetProjectFile().GetJson( PROJECT_KEY );

        if( j && j->is_object() )
            return *j;
    }

    return { { "nets", nlohmann::json::object() }, { "bundles", nlohmann::json::object() } };
}


static KOPENAPI_RESULT h_pcb_net_roles( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    PROJECT*          project = boardProject( aCtx, board );
    const nlohmann::json stored = projectRoles( project );
    const std::string filter = aArgs.value( "nets", std::string() );
    const int         minBundle = std::max( 2, aArgs.value( "min_bundle", 3 ) );
    const double      hcWidth = aArgs.value( "power_hc_width_mm", 0.5 );
    const double      hcCurrent = aArgs.value( "power_hc_current_a", 0.5 );

    std::vector<NET_FACTS>     nets;
    std::map<int, int>         index;     // net code -> nets[]
    std::map<std::string, int> byName;

    for( NETINFO_ITEM* net : board->GetNetInfo() )
    {
        if( net->GetNetCode() <= 0 )
            continue;

        NET_FACTS f;
        f.name = netName( net->GetNetname() );

        nlohmann::json fromName = KopenapiNetRoleFromName( f.name );
        f.role = fromName["role"];
        f.basis = fromName["basis"];
        f.source = f.role == "signal" ? "default" : "name";

        if( f.role == "reset" )
        {
            f.role = "signal";
            f.source = "default";
            f.tags.insert( "reset" );
        }

        if( const NETCLASS* nc = net->GetNetClass() )
        {
            f.netClass = str( nc->GetHumanReadableName() );
            f.widthMm = nc->GetTrackWidth() > 0 ? nc->GetTrackWidth() / pcbIUScale.IU_PER_MM : 0;
        }

        index[net->GetNetCode()] = static_cast<int>( nets.size() );
        byName[f.name] = static_cast<int>( nets.size() );
        nets.push_back( std::move( f ) );
    }

    // pads: footprints per net, power pins, crystals, switch nodes
    for( FOOTPRINT* fp : board->Footprints() )
    {
        const std::string ref = str( fp->GetReference() );
        const std::string prefix = refPrefix( ref );

        for( PAD* pad : fp->Pads() )
        {
            auto it = index.find( pad->GetNetCode() );

            if( it == index.end() )
                continue;

            NET_FACTS& f = nets[it->second];
            f.refs.insert( ref );

            if( !f.pad )
                f.pad = pad;

            const std::string type = str( pad->GetPinType() );
            const std::string function = upper( str( pad->GetPinFunction() ) );

            if( type.rfind( "power", 0 ) == 0 && f.role == "signal" )
                f.Set( "power", "pins", "pin " + ref + "." + str( pad->GetNumber() ) + " is " + type );

            if( ( prefix == "Y" || prefix == "X" ) && !KopenapiIsConnectorRef( ref ) )
                f.tags.insert( "crystal:" + ref );

            if( function == "SW" || function == "LX" || function.rfind( "SW", 0 ) == 0 )
                f.tags.insert( "sw_pin:" + ref );
        }
    }

    // net classes
    for( NET_FACTS& f : nets )
    {
        const std::string cls = upper( f.netClass );

        if( f.role == "signal" && ( cls.find( "POW" ) != std::string::npos || cls.find( "PWR" ) != std::string::npos
                                    || cls.find( "SUPPLY" ) != std::string::npos ) )
        {
            f.Set( "power", "net_class", "net class " + f.netClass );
        }
    }

    // the project's own data: roles and currents that nothing below overrides
    for( auto it = stored["nets"].begin(); it != stored["nets"].end(); ++it )
    {
        auto at = byName.find( it.key() );

        if( at == byName.end() )
            continue;

        NET_FACTS& f = nets[at->second];

        if( it->contains( "current_a" ) )
        {
            f.currentA = ( *it )["current_a"].get<double>();
            f.basis.push_back( "current " + std::to_string( f.currentA ).substr( 0, 5 ) + " A (project)" );
        }

        if( it->contains( "role" ) )
        {
            f.Set( ( *it )["role"].get<std::string>(), "project", "set in the project (pcb_net_roles_set)" );
            f.fixedRole = true;
        }

        for( const auto& t : it->value( "tags", nlohmann::json::array() ) )
            f.tags.insert( t.get<std::string>() );

        if( it->contains( "edge" ) )
            f.edge = ( *it )["edge"];

        if( it->contains( "termination" ) )
            f.termination = ( *it )["termination"];
    }

    // rails: high current by the project's current or the class width
    for( NET_FACTS& f : nets )
    {
        if( f.role == "power" && f.currentA >= hcCurrent )
            f.Set( "power_hc", "project", "current >= " + std::to_string( hcCurrent ).substr( 0, 4 ) + " A" );
        else if( f.role == "power" && f.currentA < 0 && f.widthMm >= hcWidth - 1e-9 )
            f.Set( "power_hc", "net_class", "class width " + std::to_string( f.widthMm ).substr( 0, 4 ) + " mm >= "
                                                    + std::to_string( hcWidth ).substr( 0, 4 ) );

        bool sw = false, inductor = false;

        for( const std::string& t : f.tags )
            sw = sw || t.rfind( "sw_pin:", 0 ) == 0;

        for( const std::string& r : f.refs )
            inductor = inductor || refPrefix( r ) == "L";

        if( sw && inductor && !isPowerRole( f.role ) )
            f.Set( "switching", "pins", "switch pin and an inductor on the net" );
    }

    // the board's custom DRC rules (not the implicit net class ones): pairs, lengths, skews
    std::shared_ptr<DRC_ENGINE> drc = board->GetDesignSettings().m_DRCEngine;

    if( drc )
    {
        for( NET_FACTS& f : nets )
        {
            if( !f.pad || isPowerRole( f.role ) )
                continue;

            PCB_TRACK probe( board );
            probe.SetLayer( F_Cu );
            probe.SetStart( f.pad->GetPosition() );
            probe.SetEnd( f.pad->GetPosition() );
            probe.SetNet( f.pad->GetNet() );

            auto custom = [&]( DRC_CONSTRAINT_T aType ) -> std::optional<DRC_CONSTRAINT>
            {
                DRC_CONSTRAINT c = drc->EvalRules( aType, &probe, nullptr, F_Cu );

                if( c.GetParentRule() && !c.GetParentRule()->IsImplicit() )
                    return c;

                return std::nullopt;
            };

            auto range = [&]( const DRC_CONSTRAINT& c )
            {
                const MINOPTMAX<int>& v = c.GetValue();
                return nlohmann::json{ { "min_mm", v.HasMin() ? nlohmann::json( mm( v.Min() ) ) : nlohmann::json() },
                                       { "opt_mm", v.HasOpt() ? nlohmann::json( mm( v.Opt() ) ) : nlohmann::json() },
                                       { "max_mm", v.HasMax() ? nlohmann::json( mm( v.Max() ) ) : nlohmann::json() },
                                       { "rule", str( c.GetParentRule()->m_Name ) } };
            };

            if( std::optional<DRC_CONSTRAINT> c = custom( DIFF_PAIR_GAP_CONSTRAINT ) )
            {
                f.rules["diff_pair_gap"] = range( *c );
                f.tags.insert( "rule_pair" );
            }

            if( std::optional<DRC_CONSTRAINT> c = custom( LENGTH_CONSTRAINT ) )
                f.rules["length"] = range( *c );

            if( std::optional<DRC_CONSTRAINT> c = custom( SKEW_CONSTRAINT ) )
                f.rules["skew"] = range( *c );

            if( f.rules.contains( "length" ) || f.rules.contains( "skew" ) )
            {
                const std::string rule = f.rules.contains( "skew" ) ? f.rules["skew"]["rule"] : f.rules["length"]["rule"];
                f.tags.insert( "matched:" + rule );

                if( f.role == "signal" || f.role == "clock" )
                    f.Set( "matched", "drc_rule", "length / skew rule '" + rule + "'" );
            }
        }
    }

    // differential pairs: KiCad's suffix rule plus DP / DM and D+ / D-; a DRC diff-pair rule alone
    // makes a pair too when the complement exists
    nlohmann::json        pairs = nlohmann::json::array();
    std::set<std::string> paired;

    for( size_t i = 0; i < nets.size(); i++ )
    {
        NET_FACTS& f = nets[i];

        if( isPowerRole( f.role ) || paired.count( f.name ) || ( f.fixedRole && f.role != "pair" ) )
            continue;

        std::string other;
        wxString    complement;

        if( board->MatchDpSuffix( wxString::FromUTF8( f.name ), complement ) == 1 && byName.count( str( complement ) ) )
        {
            other = str( complement );
        }
        else
        {
            for( const auto& [p, n] : std::vector<std::pair<std::string, std::string>>{
                         { "DP", "DM" }, { "D+", "D-" }, { "_DP", "_DN" } } )
            {
                const std::string u = upper( f.name );

                if( u.size() > p.size() && u.compare( u.size() - p.size(), p.size(), p ) == 0 )
                {
                    other = f.name.substr( 0, f.name.size() - p.size() ) + n;
                    break;
                }
            }
        }

        if( other.empty() || !byName.count( other ) )
            continue;

        NET_FACTS&            g = nets[byName[other]];
        std::set<std::string> common;
        std::set_intersection( f.refs.begin(), f.refs.end(), g.refs.begin(), g.refs.end(),
                               std::inserter( common, common.begin() ) );

        const bool byRule = f.tags.count( "rule_pair" ) && g.tags.count( "rule_pair" );

        if( common.empty() && !byRule )
            continue;

        for( NET_FACTS* x : { &f, &g } )
        {
            x->Set( "pair", byRule ? "drc_rule" : "name", "pair " + f.name + " / " + other
                                                                  + ( byRule ? " (diff-pair rule)" : "" ) );
            paired.insert( x->name );
        }

        nlohmann::json row = { { "p", f.name }, { "n", other }, { "common_parts", common },
                               { "source", byRule ? "drc_rule" : "name" } };

        if( f.rules.contains( "diff_pair_gap" ) )
            row["gap"] = f.rules["diff_pair_gap"];

        pairs.push_back( row );
    }

    for( NET_FACTS& f : nets )
    {
        bool crystal = false;

        for( const std::string& t : f.tags )
            crystal = crystal || t.rfind( "crystal:", 0 ) == 0;

        if( crystal && !isPowerRole( f.role ) && f.role != "pair" )
        {
            f.Set( "analog_hs", "pins", "crystal net" );
        }
        else if( f.role == "signal" )
        {
            const std::string b = upper( bareName( f.name ) );

            for( const char* token : { "SCK", "SCLK", "PCLK", "MCLK", "BCLK", "LRCLK" } )
            {
                if( b.find( token ) != std::string::npos )
                {
                    f.Set( "clock", "name", std::string( "net name has " ) + token );
                    break;
                }
            }
        }
    }

    // lines: nets joined through series parts (a pull-up to a rail is no series link)
    UNION_FIND                              lines( nets.size() );
    std::map<int, std::vector<std::string>> viaParts;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( !isSeriesPart( fp ) )
            continue;

        for( const auto& [a, b] : seriesLinks( fp ) )
        {
            auto ia = index.find( a->GetNetCode() );
            auto ib = index.find( b->GetNetCode() );

            if( ia == index.end() || ib == index.end() || ia->second == ib->second )
                continue;

            if( isPowerRole( nets[ia->second].role ) || isPowerRole( nets[ib->second].role ) )
                continue;

            lines.Join( ia->second, ib->second );
            viaParts[ia->second].push_back( str( fp->GetReference() ) );
            viaParts[ib->second].push_back( str( fp->GetReference() ) );
        }
    }

    std::map<int, std::vector<int>> lineNets;

    for( size_t i = 0; i < nets.size(); i++ )
    {
        if( !isPowerRole( nets[i].role ) && nets[i].role != "unconnected" && nets[i].refs.size() >= 2 )
            lineNets[lines.Find( static_cast<int>( i ) )].push_back( static_cast<int>( i ) );
    }

    std::map<std::string, const FOOTPRINT*> fpByRef;

    for( FOOTPRINT* fp : board->Footprints() )
        fpByRef[str( fp->GetReference() )] = fp;

    auto lineEnds = [&]( int aRoot )
    {
        std::set<std::string> ends;

        for( int i : lineNets[aRoot] )
        {
            for( const std::string& r : nets[i].refs )
            {
                auto it = fpByRef.find( r );

                if( it != fpByRef.end() && !isPassive( it->second ) )
                    ends.insert( r );
            }
        }

        return ends;
    };

    auto lineName = [&]( int aRoot )
    {
        // the member net with the most parts names the line
        int best = lineNets[aRoot].front();

        for( int i : lineNets[aRoot] )
        {
            if( nets[i].refs.size() > nets[best].refs.size()
                || ( nets[i].refs.size() == nets[best].refs.size() && KopenapiNaturalLess( nets[i].name, nets[best].name ) ) )
            {
                best = i;
            }
        }

        return best;
    };

    auto rootOf = [&]( const std::string& aNet ) -> int
    {
        auto it = byName.find( aNet );

        if( it == byName.end() )
            return -1;

        const int root = lines.Find( it->second );
        return lineNets.count( root ) ? root : -1;
    };

    std::set<int>  claimed;
    nlohmann::json bundles = nlohmann::json::array();

    auto makeBundle = [&]( std::string aName, std::vector<int> aRoots, const std::string& aSource,
                           nlohmann::json aBasis )
    {
        std::vector<std::pair<std::string, int>> named;
        std::set<std::string>                    ends;

        for( int root : aRoots )
        {
            named.emplace_back( nets[lineName( root )].name, root );

            for( const std::string& e : lineEnds( root ) )
                ends.insert( e );

            claimed.insert( root );
        }

        std::sort( named.begin(), named.end(), []( const auto& a, const auto& b ) { return KopenapiNaturalLess( a.first, b.first ); } );

        // order at each end: the members' pads there, in the pads' order
        nlohmann::json order = nlohmann::json::object();

        for( const std::string& end : ends )
        {
            std::vector<std::pair<std::string, std::string>> pins;     // pad number, member

            for( const auto& [name, root] : named )
            {
                for( int i : lineNets[root] )
                {
                    for( PAD* pad : fpByRef[end]->Pads() )
                    {
                        auto it = index.find( pad->GetNetCode() );

                        if( it != index.end() && it->second == i )
                            pins.emplace_back( str( pad->GetNumber() ), name );
                    }
                }
            }

            std::sort( pins.begin(), pins.end(), []( const auto& a, const auto& b ) { return KopenapiNaturalLess( a.first, b.first ); } );
            nlohmann::json row = nlohmann::json::array();

            for( const auto& [pin, member] : pins )
                row.push_back( { { "pin", pin }, { "member", member } } );

            order[end] = row;
        }

        nlohmann::json                                  members = nlohmann::json::array();
        std::map<std::string, std::vector<std::string>> series;
        bool                                            throughSeries = false;

        for( const auto& [name, root] : named )
        {
            nlohmann::json        chain = nlohmann::json::array();
            std::set<std::string> parts;

            for( int i : lineNets[root] )
            {
                chain.push_back( nets[i].name );

                for( const std::string& p : viaParts[i] )
                    parts.insert( p );
            }

            throughSeries = throughSeries || !parts.empty();
            members.push_back( { { "member", name }, { "nets", chain }, { "series_parts", parts } } );
            series[nameStem( upper( bareName( name ) ) )].push_back( name );
        }

        nlohmann::json groups = nlohmann::json::array();
        size_t         biggest = 0;
        std::string    stem;

        for( const auto& [s, names] : series )
        {
            if( names.size() >= 2 )
                groups.push_back( { { "stem", s }, { "members", names } } );

            if( names.size() > biggest )
            {
                biggest = names.size();
                stem = s;
            }
        }

        std::vector<std::string> endList( ends.begin(), ends.end() );
        std::sort( endList.begin(), endList.end(), KopenapiNaturalLess );

        if( aName.empty() )
        {
            aName = ( stem.empty() ? std::string( "bus" ) : stem ) + "@";

            for( size_t k = 0; k < endList.size(); k++ )
                aName += ( k ? "-" : "" ) + endList[k];
        }

        if( throughSeries )
            aBasis.push_back( "traced through series parts" );

        for( const auto& [name, root] : named )
        {
            for( int i : lineNets[root] )
            {
                nets[i].tags.insert( "bus:" + aName );

                if( nets[i].role == "signal" && !nets[i].fixedRole )
                {
                    nets[i].role = "bus";
                    nets[i].source = aSource;
                }
            }
        }

        bundles.push_back( { { "name", aName },
                             { "source", aSource },
                             { "ends", endList },
                             { "members", members },
                             { "name_groups", groups },
                             { "order", order },
                             { "basis", aBasis } } );
    };

    auto rootsOf = [&]( const std::vector<std::string>& aNets, nlohmann::json& aMissing )
    {
        std::vector<int> roots;

        for( const std::string& n : aNets )
        {
            const int root = rootOf( n );

            if( root < 0 )
                aMissing.push_back( n );
            else if( !claimed.count( root ) && std::find( roots.begin(), roots.end(), root ) == roots.end() )
                roots.push_back( root );
        }

        return roots;
    };

    nlohmann::json warnings = nlohmann::json::array();

    // 1. the project's bundles
    for( auto it = stored["bundles"].begin(); it != stored["bundles"].end(); ++it )
    {
        nlohmann::json   missing = nlohmann::json::array();
        std::vector<int> roots = rootsOf( it->get<std::vector<std::string>>(), missing );

        if( !missing.empty() )
            warnings.push_back( { { "bundle", it.key() }, { "nets_not_on_board", missing } } );

        if( !roots.empty() )
            makeBundle( it.key(), roots, "project", nlohmann::json::array( { "set in the project (pcb_net_roles_set)" } ) );
    }

    // 2. the schematic's bus aliases and group names (QSPI.SCK, QSPI.IO0 ...)
    std::map<std::string, std::vector<std::string>> schematicGroups;

    if( project )
    {
        for( const auto& [alias, members] : project->GetProjectFile().m_BusAliases )
        {
            std::set<std::string> wanted;

            for( const wxString& m : members )
            {
                for( const std::string& e : expandMember( str( m ) ) )
                    wanted.insert( upper( e ) );
            }

            for( const NET_FACTS& f : nets )
            {
                std::string b = upper( bareName( f.name ) );
                const size_t dot = b.rfind( '.' );

                if( wanted.count( b ) || ( dot != std::string::npos && wanted.count( b.substr( dot + 1 ) ) ) )
                    schematicGroups["alias:" + str( alias )].push_back( f.name );
            }
        }
    }

    for( const NET_FACTS& f : nets )
    {
        const std::string b = bareName( f.name );
        const size_t      dot = b.find( '.' );

        if( dot != std::string::npos && dot > 0 && dot + 1 < b.size() && !isPowerRole( f.role ) )
            schematicGroups["group:" + b.substr( 0, dot )].push_back( f.name );
    }

    for( const auto& [key, members] : schematicGroups )
    {
        nlohmann::json   missing = nlohmann::json::array();
        std::vector<int> roots = rootsOf( members, missing );

        if( static_cast<int>( roots.size() ) >= std::min( minBundle, 2 ) )
        {
            const bool alias = key.rfind( "alias:", 0 ) == 0;
            makeBundle( key.substr( key.find( ':' ) + 1 ), roots, "schematic",
                        nlohmann::json::array( { alias ? "schematic bus alias " + key.substr( 6 )
                                                       : "schematic bus group " + key.substr( 6 ) } ) );
        }
    }

    // 3. topology: lines reaching the same non-passive parts
    std::map<std::set<std::string>, std::vector<int>> byEnds;

    for( const auto& [root, members] : lineNets )
    {
        if( claimed.count( root ) )
            continue;

        std::set<std::string> ends = lineEnds( root );

        if( ends.size() >= 2 )
            byEnds[ends].push_back( root );
    }

    for( auto& [ends, roots] : byEnds )
    {
        if( static_cast<int>( roots.size() ) >= minBundle )
            makeBundle( "", roots, "topology",
                        nlohmann::json::array( { std::to_string( roots.size() ) + " lines join the same parts" } ) );
    }

    nlohmann::json rows = nlohmann::json::array();
    nlohmann::json counts = nlohmann::json::object();
    nlohmann::json rails = nlohmann::json::array();
    nlohmann::json matched = nlohmann::json::object();

    std::vector<int> order( nets.size() );
    std::iota( order.begin(), order.end(), 0 );
    std::sort( order.begin(), order.end(), [&]( int a, int b ) { return KopenapiNaturalLess( nets[a].name, nets[b].name ); } );

    for( int i : order )
    {
        const NET_FACTS& f = nets[i];

        for( const std::string& t : f.tags )
        {
            if( t.rfind( "matched:", 0 ) == 0 )
                matched[t.substr( 8 )].push_back( f.name );
        }

        if( !filter.empty() && !KopenapiGlob( filter, f.name ) )
            continue;

        counts[f.role] = counts.value( f.role, 0 ) + 1;
        nlohmann::json row = { { "net", f.name }, { "role", f.role }, { "source", f.source }, { "class", f.netClass },
                               { "parts", f.refs.size() }, { "basis", f.basis } };

        if( !f.tags.empty() )
            row["tags"] = f.tags;

        if( !f.rules.empty() )
            row["rules"] = f.rules;

        if( f.currentA >= 0 )
            row["current_a"] = f.currentA;

        if( f.edge.is_object() )
            row["edge"] = f.edge;

        if( f.termination.is_object() )
            row["termination"] = f.termination;

        rows.push_back( row );

        if( isPowerRole( f.role ) )
        {
            nlohmann::json rail = { { "net", f.name }, { "role", f.role }, { "class", f.netClass }, { "width_mm", f.widthMm } };

            if( f.currentA >= 0 )
                rail["current_a"] = f.currentA;

            rails.push_back( rail );
        }
    }

    nlohmann::json groups = nlohmann::json::array();

    for( auto it = matched.begin(); it != matched.end(); ++it )
        groups.push_back( { { "rule", it.key() }, { "nets", *it } } );

    nlohmann::json out = { { "counts", counts },
                           { "nets", rows },
                           { "pairs", pairs },
                           { "matched_groups", groups },
                           { "rails", rails },
                           { "bundles", bundles } };

    if( !warnings.empty() )
        out["warnings"] = warnings;

    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_net_roles_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*   board = context->GetBoard();
    PROJECT* project = boardProject( aCtx, board );

    if( !project || project->GetProjectPath().IsEmpty() )
        return KOPENAPI_RESULT::Error( 409, "the board has no project (save it in a project first)" );

    nlohmann::json stored = projectRoles( project );

    if( aArgs.value( "clear", false ) )
        stored = { { "nets", nlohmann::json::object() }, { "bundles", nlohmann::json::object() } };

    if( !stored.contains( "nets" ) || !stored["nets"].is_object() )
        stored["nets"] = nlohmann::json::object();

    if( !stored.contains( "bundles" ) || !stored["bundles"].is_object() )
        stored["bundles"] = nlohmann::json::object();

    nlohmann::json unknown = nlohmann::json::array();

    auto onBoard = [&]( const std::string& aNet )
    {
        return board->FindNet( EscapeString( wxString::FromUTF8( aNet ), CTX_NETNAME ) ) != nullptr
               || board->FindNet( wxString::FromUTF8( aNet ) ) != nullptr;
    };

    for( const auto& row : aArgs.value( "nets", nlohmann::json::array() ) )
    {
        const std::string net = row.value( "net", std::string() );

        if( net.empty() )
            return KOPENAPI_RESULT::Error( 400, "nets[]: each needs 'net'" );

        if( !onBoard( net ) )
            unknown.push_back( net );

        nlohmann::json& entry = stored["nets"][net];

        if( row.contains( "role" ) )
        {
            const std::string role = row["role"].get<std::string>();

            if( !ROLES.count( role ) )
                return KOPENAPI_RESULT::Error( 400, "role '" + role + "' is none of ground, power, power_hc, pair, clock, "
                                                    "matched, bus, analog, analog_hs, switching, signal, nc, fixed" );

            entry["role"] = role;
        }

        if( row.contains( "current_a" ) )
        {
            const double a = row["current_a"].get<double>();

            if( a < 0 )
                return KOPENAPI_RESULT::Error( 400, "current_a must be >= 0" );

            entry["current_a"] = a;
        }

        if( row.contains( "tags" ) )
            entry["tags"] = row["tags"];

        for( const char* key : { "edge", "termination" } )
        {
            if( !row.contains( key ) )
                continue;

            if( row[key].is_null() )
            {
                entry.erase( key );
                continue;
            }

            const std::string error = std::string( key ) == "edge" ? KopenapiEdgeCheck( row[key] )
                                                                   : KopenapiTerminationCheck( row[key] );

            if( !error.empty() )
                return KOPENAPI_RESULT::Error( 400, "nets[" + net + "]." + key + ": " + error );

            entry[key] = row[key];
        }

        if( entry.empty() )
            stored["nets"].erase( net );
    }

    for( const auto& n : aArgs.value( "remove_nets", nlohmann::json::array() ) )
        stored["nets"].erase( n.get<std::string>() );

    for( const auto& b : aArgs.value( "bundles", nlohmann::json::array() ) )
    {
        const std::string name = b.value( "name", std::string() );
        const auto        members = b.value( "members", std::vector<std::string>() );

        if( name.empty() || members.size() < 2 )
            return KOPENAPI_RESULT::Error( 400, "bundles[]: each needs 'name' and at least 2 'members'" );

        for( const std::string& m : members )
        {
            if( !onBoard( m ) )
                unknown.push_back( m );
        }

        stored["bundles"][name] = members;
    }

    for( const auto& n : aArgs.value( "remove_bundles", nlohmann::json::array() ) )
        stored["bundles"].erase( n.get<std::string>() );

    // accept detected bundles as they are now: their member nets go into the project
    if( aArgs.contains( "accept_bundles" ) )
    {
        KOPENAPI_RESULT now = h_pcb_net_roles( aCtx, nlohmann::json::object() );
        std::set<std::string> wanted;

        for( const auto& n : aArgs["accept_bundles"] )
            wanted.insert( n.get<std::string>() );

        for( const auto& b : now.body["bundles"] )
        {
            if( !wanted.count( b["name"].get<std::string>() ) )
                continue;

            std::vector<std::string> members;

            for( const auto& m : b["members"] )
            {
                for( const auto& n : m["nets"] )
                    members.push_back( n.get<std::string>() );
            }

            stored["bundles"][b["name"].get<std::string>()] = members;
            wanted.erase( b["name"].get<std::string>() );
        }

        if( !wanted.empty() )
            return KOPENAPI_RESULT::Error( 404, "no detected bundle named " + *wanted.begin() + " (see pcb_net_roles)" );
    }

    if( stored["nets"].empty() && stored["bundles"].empty() )
        project->GetProjectFile().Set( PROJECT_KEY, nlohmann::json() );
    else
        project->GetProjectFile().Set( PROJECT_KEY, stored );

    project->GetProjectFile().SaveToFile( project->GetProjectPath(), true );

    nlohmann::json out = { { "stored", stored } };

    if( !unknown.empty() )
        out["nets_not_on_board"] = unknown;

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_net_roles",
                   "What each net is for the router, without routing: one primary role per net (ground, power, "
                   "power_hc, pair, clock, matched, analog_hs (crystal), switching, bus, signal, unconnected) with "
                   "tags, source (project, drc_rule, net_class, pins, schematic, name, topology) and basis; "
                   "differential pairs (KiCad's suffix rule plus DP / DM, D+ / D-, sharing a part, or named by a "
                   "custom diff-pair rule); matched groups and per-net rules (length, skew, diff-pair gap from the "
                   "board's custom DRC rules); rails (current from the project); bundles (buses): from the project "
                   "(pcb_net_roles_set), from the schematic (bus aliases, group names QSPI.SCK), else >= min_bundle "
                   "lines reaching the same non-passive parts; a line is traced through series resistors / arrays / "
                   "ferrites (PICO_SCK -RN1- EVE_SCK is one member); members with their nets and series parts, name "
                   "groups, pin order at each end. nets: glob filters the per-net rows",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"string","description":"glob over net names for the per-net rows"},
                        "min_bundle":{"type":"integer","default":3,"minimum":2},
                        "power_hc_width_mm":{"type":"number","default":0.5,"description":"rails whose class width is at least this are power_hc"},
                        "power_hc_current_a":{"type":"number","default":0.5,"description":"rails with a project current of at least this are power_hc"}}})json"_json,
                   false, h_pcb_net_roles );

KOPENAPI_REGISTER( "pcb_net_roles_set",
                   "Store net roles, rail currents, driver edge data and terminations and bundles in the project "
                   "(.kicad_pro, kicadopenapi.net_roles); pcb_net_roles puts them above everything it detects: nets "
                   "[{net, role?, current_a?, tags?, edge?, termination?}] (edge: {family, tr_ps, tr_def 10-90 / "
                   "20-80, rout_ohm, swing_v, nm_v, driver ref.pin}, see pcb_signal_check; null removes), "
                   "remove_nets, bundles [{name, members: nets}], remove_bundles, accept_bundles (names of detected "
                   "bundles, stored with their member nets as they are now), clear. Answers what is stored and the "
                   "nets not on the board",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"object","properties":{
                            "net":{"type":"string"},
                            "role":{"type":"string","enum":["ground","power","power_hc","pair","clock","matched","bus","analog","analog_hs","switching","signal","nc","fixed"]},
                            "current_a":{"type":"number","minimum":0},
                            "tags":{"type":"array","items":{"type":"string"}},
                            "edge":{"type":["object","null"],"description":"driver edge: {family (lvcmos33, lvcmos18, hc, ahc, ac, f, mcu_fast, mcu_slow, fpga_fast, fpga_slow, sstl, lvds, hcsl, cml, usb2_hs), tr_ps, tr_def (10-90 default, 20-80), rout_ohm, swing_v, nm_v, driver (ref.pin)}; given values win over the family defaults"},
                            "termination":{"type":["object","null"],"description":"{kind: none / series / end / thevenin / rc / on_die / both, dc_balanced?}; overrides what pcb_signal_check detects"}},"required":["net"]}},
                        "remove_nets":{"type":"array","items":{"type":"string"}},
                        "bundles":{"type":"array","items":{"type":"object","properties":{
                            "name":{"type":"string"},
                            "members":{"type":"array","items":{"type":"string"}}},"required":["name","members"]}},
                        "remove_bundles":{"type":"array","items":{"type":"string"}},
                        "accept_bundles":{"type":"array","items":{"type":"string"}},
                        "clear":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_net_roles_set );

KOPENAPI_MARK_EDITING( "pcb_net_roles_set" );
