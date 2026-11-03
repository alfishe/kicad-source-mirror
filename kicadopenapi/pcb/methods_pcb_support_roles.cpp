/// @file methods_pcb_support_roles.cpp
/// @brief kicadopenapi pcb_support_roles: the support role of each part for L2 placement
/// (docs/inprogress/2026-10-31-router-architecture/support-templates.md §1).  The role comes from the
/// footprint's `Support.Role` field (from its symbol), else from connectivity for unambiguous topologies,
/// else the part is unassigned.  No value thresholds.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <footprint.h>
#include <kicadopenapi_util.h>
#include <kopenapi_support_roles.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_field.h>

#include <algorithm>
#include <cctype>
#include <map>
#include <set>




static std::string str( const wxString& aText )
{
    return std::string( aText.ToUTF8() );
}


/// Capacitance in farads from a value text (100n, 0.1uF, 4u7, 22 pF, 1µF), or < 0.
double KopenapiCapacitance( const std::string& aValue )
{
    std::string s;

    for( size_t i = 0; i < aValue.size(); ++i )
    {
        if( aValue.compare( i, 2, "\xC2\xB5" ) == 0 )   // µ
        {
            s += 'u';
            ++i;
        }
        else if( aValue[i] != ' ' )
        {
            s += aValue[i];
        }
    }

    size_t      k = 0;
    std::string digits;

    while( k < s.size() && ( isdigit( (unsigned char) s[k] ) || s[k] == '.' || s[k] == ',' ) )
    {
        digits += s[k] == ',' ? '.' : s[k];
        ++k;
    }

    if( digits.empty() )
        return -1;

    static const std::map<char, double> scale = { { 'p', 1e-12 }, { 'n', 1e-9 }, { 'u', 1e-6 }, { 'm', 1e-3 } };
    double mult = 1.0;
    std::string rest = s.substr( k );

    if( !rest.empty() && scale.count( rest[0] ) )
    {
        mult = scale.at( rest[0] );
        std::string frac;
        size_t      j = 1;

        while( j < rest.size() && isdigit( (unsigned char) rest[j] ) )
            frac += rest[j++];

        if( !frac.empty() && digits.find( '.' ) == std::string::npos )
            digits += "." + frac;

        rest = rest.substr( j );
    }

    if( !rest.empty() && rest != "F" && rest != "f" )
        return -1;

    try
    {
        return std::stod( digits ) * mult;
    }
    catch( ... )
    {
        return -1;
    }
}


static bool signalLike( const std::string& aRole )
{
    return aRole == "signal" || aRole == "clock" || aRole == "pair" || aRole == "matched" || aRole == "bus"
           || aRole == "analog";
}


static bool powerRole( const std::string& aRole )
{
    return aRole == "power" || aRole == "power_hc";
}


nlohmann::json KopenapiSupportRoles( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, const std::string& aRefs )
{
    std::map<std::string, std::string> netRole;   // net name -> pcb_net_roles role

    if( std::optional<KOPENAPI_METHOD> roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" ) )
    {
        const KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

        if( r.status < 400 && r.body.contains( "nets" ) && r.body["nets"].is_array() )
        {
            for( const nlohmann::json& row : r.body["nets"] )
                netRole[row.value( "net", std::string() )] = row.value( "role", std::string() );
        }
    }

    auto roleOf = [&]( int aNet ) -> std::string
    {
        NETINFO_ITEM* info = aBoard->FindNet( aNet );
        auto          it = info ? netRole.find( str( info->GetNetname() ) ) : netRole.end();
        return it == netRole.end() ? std::string() : it->second;
    };

    auto netName = [&]( int aNet )
    {
        NETINFO_ITEM* info = aBoard->FindNet( aNet );
        return info ? str( info->GetNetname() ) : std::string();
    };

    struct PART
    {
        FOOTPRINT*         fp = nullptr;
        std::string        ref;
        std::string        field;
        std::set<int>      nets;
        int                pads = 0;
        bool               crystal = false;
    };

    std::vector<PART> parts;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        PART p;
        p.fp = fp;
        p.ref = str( fp->GetReference() );

        if( PCB_FIELD* f = fp->GetField( wxString( KOPENAPI_SUPPORT_ROLE_FIELD ) ) )
        {
            wxString text = f->GetText();
            p.field = str( text.Trim().Trim( false ).Lower() );
        }

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 )
            {
                p.nets.insert( pad->GetNetCode() );
                ++p.pads;
            }
        }

        std::set<int> live;

        for( int n : p.nets )
        {
            if( roleOf( n ) != "ground" )
                live.insert( n );
        }

        p.crystal = live.size() == 2 && p.pads >= 2
                    && std::all_of( live.begin(), live.end(), [&]( int n ) { return roleOf( n ) == "analog_hs"; } );
        parts.push_back( p );
    }

    // an anchor: a part of three or more connected pads that is not a support part (an IC, a connector, a regulator)
    std::map<int, std::vector<std::pair<std::string, std::string>>> anchorPins;   // net -> (ref, pin)

    for( const PART& p : parts )
    {
        const bool support = !p.field.empty() && p.field != "none";

        if( p.pads < 3 || p.crystal || support )
            continue;

        for( PAD* pad : p.fp->Pads() )
        {
            if( pad->GetNetCode() > 0 )
                anchorPins[pad->GetNetCode()].push_back( { p.ref, str( pad->GetNumber() ) } );
        }
    }

    nlohmann::json rows = nlohmann::json::array();
    nlohmann::json unassigned = nlohmann::json::array();
    nlohmann::json counts = nlohmann::json::object();

    for( const PART& p : parts )
    {
        if( !aRefs.empty() && !KopenapiGlob( aRefs, p.ref ) )
            continue;

        nlohmann::json row = { { "ref", p.ref }, { "value", str( p.fp->GetValue() ) } };
        const double cap = KopenapiCapacitance( str( p.fp->GetValue() ) );

        if( cap > 0 )
            row["capacitance_f"] = cap;

        std::string role, reason;

        if( !p.field.empty() )
        {
            if( !KopenapiSupportRoleValues().count( p.field ) )
            {
                unassigned.push_back( { { "ref", p.ref }, { "reason", "unknown Support.Role '" + p.field + "'" } } );
                continue;
            }

            role = p.field;
            row["source"] = "field";
            reason = "Support.Role field";
        }
        else if( p.crystal )
        {
            role = "crystal";
            row["source"] = "connectivity";
            reason = "two crystal nets (analog_hs)";
        }
        else if( p.pads == 2 && p.nets.size() == 2 )
        {
            const int         a = *p.nets.begin();
            const int         b = *p.nets.rbegin();
            const std::string ra = roleOf( a ), rb = roleOf( b );
            auto has = [&]( auto aTest, int& aNet, int& aOther )
            {
                if( aTest( ra ) )
                {
                    aNet = a;
                    aOther = b;
                    return true;
                }

                if( aTest( rb ) )
                {
                    aNet = b;
                    aOther = a;
                    return true;
                }

                return false;
            };

            int x = 0, y = 0;
            auto isGround = []( const std::string& r ) { return r == "ground"; };

            if( has( isGround, x, y ) && powerRole( roleOf( y ) ) )
            {
                if( !anchorPins.count( y ) )
                {
                    reason = "between " + netName( y ) + " and " + netName( x ) + ", no IC pin on " + netName( y );
                }
                else
                {
                    role = "decoupling";
                    const auto& pin = anchorPins[y].front();
                    row["anchor"] = pin.first + "." + pin.second;
                    reason = "between " + netName( y ) + " and " + netName( x ) + " at " + pin.first + "." + pin.second;
                }
            }
            else if( has( []( const std::string& r ) { return r == "switching"; }, x, y ) && powerRole( roleOf( y ) ) )
            {
                role = "inductor";
                reason = "in series from switching " + netName( x ) + " to " + netName( y );
            }
            else if( has( []( const std::string& r ) { return r == "analog_hs"; }, x, y ) && roleOf( y ) == "ground" )
            {
                role = "crystal_load";
                reason = "from crystal net " + netName( x ) + " to " + netName( y );
            }
            else if( has( signalLike, x, y ) && ( powerRole( roleOf( y ) ) || roleOf( y ) == "ground" ) )
            {
                role = "pullup";
                reason = "from " + netName( x ) + " to " + netName( y );
            }
            else if( signalLike( ra ) && signalLike( rb ) && anchorPins.count( a ) && anchorPins.count( b ) )
            {
                std::string far;

                for( const auto& pa : anchorPins[a] )
                {
                    for( const auto& pb : anchorPins[b] )
                    {
                        if( pa.first != pb.first && far.empty() )
                            far = pa.first + "." + pa.second + " - " + pb.first + "." + pb.second;
                    }
                }

                if( !far.empty() )
                {
                    role = "series";
                    reason = "in series on " + far;
                }
            }

            if( !role.empty() )
                row["source"] = "connectivity";
            else if( reason.empty() )
                reason = "two pins on " + netName( a ) + " (" + ( ra.empty() ? "?" : ra ) + ") and " + netName( b )
                         + " (" + ( rb.empty() ? "?" : rb ) + "): no unambiguous topology, no Support.Role field";
        }
        else
        {
            continue;   // not a two-terminal part and no field: an anchor or a normal part
        }

        if( role.empty() )
        {
            unassigned.push_back( { { "ref", p.ref }, { "reason", reason } } );
            continue;
        }

        row["role"] = role;
        row["reason"] = reason;
        counts[role] = counts.value( role, 0 ) + 1;
        rows.push_back( row );
    }

    return { { "parts", rows }, { "unassigned", unassigned }, { "counts", counts } };
}


static KOPENAPI_RESULT h_pcb_support_roles( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    return KOPENAPI_RESULT::Ok( KopenapiSupportRoles( aCtx, context->GetBoard(), aArgs.value( "refs", std::string() ) ) );
}


KOPENAPI_REGISTER( "pcb_support_roles",
                   "The support role of each part for L2 placement: decoupling, bulk, filter, inductor, series, "
                   "pullup, crystal, crystal_load, esd or none. Source field: the footprint's Support.Role field "
                   "(from its symbol). Source connectivity, for two-terminal parts without the field: between a "
                   "supply and ground with an IC pin on the supply -> decoupling; switching net to a supply -> "
                   "inductor; crystal net to ground -> crystal_load; signal to a supply or ground -> pullup; in "
                   "series between pins of two different ICs -> series; a part on two crystal nets -> crystal. "
                   "bulk, filter and esd come only from the field; no value thresholds. Answers parts [{ref, value, "
                   "capacitance_f, role, source, reason, anchor}], unassigned [{ref, reason}] (two-terminal parts "
                   "with no field and no unambiguous topology, and unknown field values) and counts",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"string","description":"glob over references"}}})json"_json,
                   false, h_pcb_support_roles );
