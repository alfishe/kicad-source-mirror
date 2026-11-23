/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * pcb_route_endgame: the nets a routed board leaves split, one spot at a time: the copper the router
 * laid inside a box round a split net's open connection taken up with the other nets' copper there,
 * the nets negotiated together by pcb_route_oracle_joint over that box, the unshared paths laid, the
 * router taking the rest; a step stays when fewer nets are split (docs: router-architecture/endgame-cpp.md).
 */

#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <connectivity/connectivity_algo.h>
#include <connectivity/connectivity_data.h>
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <netinfo.h>
#include <pcb_track.h>
#include <ratsnest/ratsnest_data.h>
#include <string_utils.h>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>


void refillPours( PCB_CONTEXT& aContext, BOARD* aBoard );
std::optional<VECTOR2I> KopenapiInnerJoint( const std::shared_ptr<CONNECTIVITY_DATA>& aConn, PCB_TRACK* aTrack,
                                            const VECTOR2I& aFree );


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( int aIU )
{
    return pcbIUScale.IUTomm( aIU );
}


using CLOCK = std::chrono::steady_clock;

double msSince( CLOCK::time_point aStart )
{
    return std::chrono::duration<double, std::milli>( CLOCK::now() - aStart ).count();
}


KOPENAPI_RESULT invoke( KOPENAPI_CONTEXT& aCtx, const char* aName, const nlohmann::json& aArgs )
{
    std::optional<KOPENAPI_METHOD> m = KOPENAPI_REGISTRY::Get().Find( aName );

    if( !m )
        return KOPENAPI_RESULT::Error( 500, std::string( "no method " ) + aName );

    KOPENAPI_JOB_NESTED nested( aName );
    return m->handler( aCtx, aArgs );
}


struct OPEN
{
    VECTOR2I a, b;

    int64_t Span() const { return std::max( std::abs( (int64_t) a.x - b.x ), std::abs( (int64_t) a.y - b.y ) ); }
};


/// The open connections of the target nets now, per net
std::map<int, std::vector<OPEN>> opensNow( BOARD* aBoard, const std::set<int>& aTargets )
{
    aBoard->BuildConnectivity();
    std::shared_ptr<CONNECTIVITY_DATA> conn = aBoard->GetConnectivity();
    conn->RecalculateRatsnest();

    std::map<int, std::vector<OPEN>> out;

    for( int net : aTargets )
    {
        RN_NET* rn = conn->GetRatsnestForNet( net );

        if( !rn )
            continue;

        for( const CN_EDGE& e : rn->GetEdges() )
        {
            if( e.GetSourceNode() && e.GetTargetNode() )
                out[net].push_back( { e.GetSourcePos(), e.GetTargetPos() } );
        }
    }

    return out;
}


std::set<int> netsOf( const std::map<int, std::vector<OPEN>>& aOpens )
{
    std::set<int> s;

    for( const auto& [net, list] : aOpens )
        s.insert( net );

    return s;
}


/// A step's undo log: the ids of the board's tracks when it began, and its own copy of every track of
/// the nets it works on (saved before it touches them). The router may shove only those nets' copper
/// (the rest is held locked while it runs), so the copies and the ids take the step back exactly
struct UNDO
{
    std::set<KIID>                         before;
    std::set<int>                          nets;
    std::map<KIID, std::unique_ptr<PCB_TRACK>> saved;

    void Begin( BOARD* aBoard )
    {
        before.clear();
        nets.clear();
        saved.clear();

        for( PCB_TRACK* t : aBoard->Tracks() )
            before.insert( t->m_Uuid );
    }

    void Save( BOARD* aBoard, const std::set<int>& aNets )
    {
        for( PCB_TRACK* t : aBoard->Tracks() )
        {
            if( aNets.count( t->GetNetCode() ) && !nets.count( t->GetNetCode() ) && before.count( t->m_Uuid ) )
                saved.emplace( t->m_Uuid, static_cast<PCB_TRACK*>( t->Clone() ) );
        }

        nets.insert( aNets.begin(), aNets.end() );
    }

    void TakeUp( PCB_CONTEXT& aContext, const std::vector<PCB_TRACK*>& aItems )
    {
        if( aItems.empty() )
            return;

        BOARD_COMMIT commit( aContext.GetToolManager() );

        for( PCB_TRACK* t : aItems )
            commit.Remove( t );

        commit.Push( _( "Endgame: take up (API)" ) );
    }

    /// aCall with every track the step may not change locked (the copper of other nets it found)
    template <typename FN>
    void Fixed( BOARD* aBoard, FN aCall )
    {
        std::set<KIID> locked;

        for( PCB_TRACK* t : aBoard->Tracks() )
        {
            if( !t->IsLocked() && before.count( t->m_Uuid ) && !nets.count( t->GetNetCode() ) )
            {
                t->SetLocked( true );
                locked.insert( t->m_Uuid );
            }
        }

        aCall();

        for( PCB_TRACK* t : aBoard->Tracks() )
        {
            if( locked.count( t->m_Uuid ) )
                t->SetLocked( false );
        }
    }

    /// the board's copper back to where the step found it, the pours refilled
    void Restore( PCB_CONTEXT& aContext, BOARD* aBoard )
    {
        BOARD_COMMIT commit( aContext.GetToolManager() );
        bool         any = false;

        for( PCB_TRACK* t : aBoard->Tracks() )
        {
            if( !before.count( t->m_Uuid ) || saved.count( t->m_Uuid ) )
            {
                commit.Remove( t );
                any = true;
            }
        }

        for( auto& [id, t] : saved )
        {
            commit.Add( t.release() );
            any = true;
        }

        saved.clear();

        if( any )
        {
            commit.Push( _( "Endgame: undo a step (API)" ) );
            KOPENAPI_JOB_NESTED nested( "refill" );
            refillPours( aContext, aBoard );
        }
    }
};

/// The added copper of aNets left with a free end (the step took up the copper it led to, or routed
/// round it): cut back to where the net joins it, else removed, until none is left; their count
template <typename ADDED>
int sweepStubs( PCB_CONTEXT& aContext, BOARD* aBoard, const std::set<int>& aNets, ADDED aAdded )
{
    int  stubs = 0;
    bool again = true;

    while( again )
    {
        again = false;
        aBoard->BuildConnectivity();
        std::shared_ptr<CONNECTIVITY_DATA> conn = aBoard->GetConnectivity();
        BOARD_COMMIT                       sweep( aContext.GetToolManager() );

        for( PCB_TRACK* track : aBoard->Tracks() )
        {
            if( !aNets.count( track->GetNetCode() ) || !aAdded( track ) )
                continue;

            VECTOR2I freeEnd;

            if( !conn->TestTrackEndpointDangling( track, true, &freeEnd ) )
                continue;

            const std::optional<VECTOR2I> joint = track->Type() == PCB_TRACE_T ? KopenapiInnerJoint( conn, track, freeEnd )
                                                                               : std::nullopt;

            if( joint )
            {
                sweep.Modify( track );

                if( track->GetStart() == freeEnd )
                    track->SetStart( *joint );
                else
                    track->SetEnd( *joint );
            }
            else
            {
                sweep.Remove( track );
            }

            stubs++;
            again = true;
        }

        if( again )
            sweep.Push( _( "Endgame: remove dangling copper (API)" ) );
        else
            sweep.Revert();
    }

    return stubs;
}

} // namespace


static KOPENAPI_RESULT h_pcb_route_endgame( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*     board = context->GetBoard();
    const auto started = CLOCK::now();

    std::set<int> targets;

    for( const nlohmann::json& n : aArgs.value( "nets", nlohmann::json::array() ) )
    {
        const NETINFO_ITEM* info = KopenapiFindNet( board, n.get<std::string>() );

        if( !info || info->GetNetCode() <= 0 )
            return KOPENAPI_RESULT::Error( 404, "no net " + n.get<std::string>() );

        targets.insert( info->GetNetCode() );
    }

    if( targets.empty() )
    {
        for( NETINFO_ITEM* info : board->GetNetInfo() )
        {
            if( info->GetNetCode() > 0 )
                targets.insert( info->GetNetCode() );
        }
    }

    // the copper the endgame never takes up: the base board's (the fan-out, the supply copper)
    std::set<KIID> base;

    if( aArgs.contains( "base" ) )
    {
        std::string error;

        if( !KopenapiBoardCopperIds( board, aArgs["base"].get<std::string>(), base, error ) )
            return KOPENAPI_RESULT::Error( 422, "base: " + error );
    }

    const double         marginMm = aArgs.value( "margin_mm", 2.0 );
    const double         repairMarginMm = aArgs.value( "repair_margin_mm", 2.0 );
    const int            maxIter = aArgs.value( "max_iter", 40 );
    const int            maxTries = aArgs.value( "max_tries", 3 );
    const int            repairRounds = aArgs.value( "repair_rounds", 3 );
    const int            retryRounds = aArgs.value( "retry_rounds", 150 );
    const bool           sideways = aArgs.value( "sideways", true );
    const bool           wholeLast = aArgs.value( "whole_last", true );
    const nlohmann::json jointArgs = aArgs.value( "joint", nlohmann::json( { { "cell_mm", 0.05 }, { "rounds", 60 }, { "claim_slack_mm", 0 } } ) );
    const BOX2I          edges = board->GetBoardEdgesBoundingBox();

    auto netName = [&]( int aNet ) { return str( UnescapeString( board->FindNet( aNet )->GetNetname() ) ); };

    auto names = [&]( const std::set<int>& aNets )
    {
        nlohmann::json a = nlohmann::json::array();

        for( int n : aNets )
            a.push_back( netName( n ) );

        return a;
    };

    auto boxJson = [&]( const BOX2I& b )
    {
        return nlohmann::json( { toMm( b.GetLeft() ), toMm( b.GetTop() ), toMm( b.GetRight() ), toMm( b.GetBottom() ) } );
    };

    auto boxAround = [&]( const OPEN& aOpen, double aMarginMm )
    {
        BOX2I b( aOpen.a, VECTOR2I( 0, 0 ) );
        b.Merge( aOpen.b );
        b.Inflate( pcbIUScale.mmToIU( aMarginMm ) );
        return b.Intersect( edges );
    };

    auto added = [&]( PCB_TRACK* t ) { return !base.count( t->m_Uuid ) && !t->IsLocked() && targets.count( t->GetNetCode() ); };

    // one spot: the target nets with copper laid since the base touching the box or an open end in it
    // joined with net aNet; their copper inside the box taken up, negotiated over the box, the unshared
    // paths laid, the router takes the nets left split. Returns the spot's record
    auto spot = [&]( UNDO& aUndo, const std::set<int>& aFirst, const std::vector<BOX2I>& aBoxes,
                     const std::map<int, std::vector<OPEN>>& aOpens, bool aWhole ) -> nlohmann::json
    {
        const auto     t0 = CLOCK::now();
        std::set<int>  group = aFirst;
        std::vector<PCB_TRACK*> inside;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( !added( t ) )
                continue;

            const BOX2I bb = t->GetBoundingBox();

            for( const BOX2I& box : aBoxes )
            {
                if( box.Intersects( bb ) )
                    group.insert( t->GetNetCode() );
            }
        }

        for( const auto& [net, list] : aOpens )
        {
            for( const OPEN& o : list )
            {
                for( const BOX2I& box : aBoxes )
                {
                    if( box.Contains( o.a ) && box.Contains( o.b ) )
                        group.insert( net );
                }
            }
        }

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( !added( t ) || !group.count( t->GetNetCode() ) )
                continue;

            const BOX2I bb = t->GetBoundingBox();

            if( aWhole || std::any_of( aBoxes.begin(), aBoxes.end(), [&]( const BOX2I& b ) { return b.Contains( bb ); } ) )
                inside.push_back( t );
        }

        aUndo.Save( board, group );
        aUndo.TakeUp( *context, inside );

        nlohmann::json args = jointArgs;
        args["nets"] = names( group );
        if( !aWhole )
        {
            args["region"] = nlohmann::json::array();

            for( const BOX2I& box : aBoxes )
                args["region"].push_back( boxJson( box ) );
        }

        const double    takeMs = msSince( t0 );
        const auto      tJoint = CLOCK::now();
        KOPENAPI_RESULT j = invoke( aCtx, "pcb_route_oracle_joint", args );
        bool            retried = false;

        if( j.status == 200 && !j.body.value( "converged", false ) && retryRounds > args.value( "rounds", 40 ) )
        {
            args["rounds"] = retryRounds;
            j = invoke( aCtx, "pcb_route_oracle_joint", args );
            retried = true;
        }

        const double   jointWallMs = msSince( tJoint );
        const auto     tLay = CLOCK::now();
        std::set<int>  laidNets, sharing;
        bool           anyLaid = false;
        int            layFailed = 0;
        nlohmann::json failures = nlohmann::json::array();

        for( const nlohmann::json& n : j.status == 200 ? j.body.value( "result", nlohmann::json::array() ) : nlohmann::json::array() )
        {
            const NETINFO_ITEM* info = KopenapiFindNet( board, n.value( "net", std::string() ) );

            if( !info )
                continue;

            if( n.value( "cells_shared", 0 ) > 0 || !n.value( "joined", false ) )
            {
                sharing.insert( info->GetNetCode() );
                continue;
            }

            bool ok = true;

            for( const nlohmann::json& path : n.value( "paths", nlohmann::json::array() ) )
            {
                nlohmann::json pts = nlohmann::json::array();

                for( size_t k = 0; k < path.size(); ++k )
                {
                    if( k && path[k]["at"] == path[k - 1]["at"] )
                        pts.back() = { { "at", path[k]["at"] }, { "via", path[k]["layer"] } };
                    else
                        pts.push_back( path[k]["at"] );
                }

                if( pts.size() < 2 )
                    continue;

                nlohmann::json add = { { "net", n["net"] }, { "points", pts }, { "layer", path[0]["layer"] },
                                       { "width_mm", n["width_mm"] }, { "zones", "defer" } };

                if( n.contains( "via_mm" ) && n["via_mm"].is_array() )
                {
                    add["via_diameter_mm"] = n["via_mm"][0];
                    add["via_drill_mm"] = n["via_mm"][1];
                }

                const KOPENAPI_RESULT r = invoke( aCtx, "pcb_track_add", add );

                if( r.status == 200 )
                    anyLaid = true;
                else
                {
                    ok = false;
                    layFailed++;

                    if( failures.size() < 3 )
                        failures.push_back( { { "net", n["net"] }, { "status", r.status },
                                              { "why", r.body.value( "violations", nlohmann::json::array() ).size()
                                                               ? r.body["violations"][0]
                                                               : r.body.value( "error", nlohmann::json() ) } } );
                }
            }

            ( ok ? laidNets : sharing ).insert( info->GetNetCode() );
        }

        // the paths were laid with the pours left as they were: refilled once for them all
        if( anyLaid )
        {
            KOPENAPI_JOB_NESTED nested( "refill" );
            refillPours( *context, board );
        }

        // the router takes the group's nets still split
        std::set<int> rest;

        for( const auto& [net, list] : opensNow( board, group ) )
            rest.insert( net );

        const double   layMs = msSince( tLay );
        const auto     tRoute = CLOCK::now();
        nlohmann::json routed;

        if( !rest.empty() && !KopenapiJobCancelled() )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Ok( {} );
            aUndo.Fixed( board, [&] { r = invoke( aCtx, "pcb_route", { { "nets", names( rest ) } } ); } );
            routed = { { "nets", rest.size() }, { "status", r.status }, { "routed", r.body.value( "routed", 0 ) },
                       { "connections", r.body.value( "connections", 0 ) } };
        }

        const double routeMs = msSince( tRoute );
        const int    stubs = sweepStubs( *context, board, group, added );

        nlohmann::json rec = { { "boxes", nlohmann::json::array() }, { "whole", aWhole }, { "group", group.size() }, { "taken_up", inside.size() },
                               { "converged", j.status == 200 && j.body.value( "converged", false ) }, { "retried", retried },
                               { "joint_ms", j.body.value( "ms", 0.0 ) }, { "sharing", names( sharing ) },
                               { "laid", laidNets.size() }, { "lay_failed", layFailed }, { "routed", routed },
                               { "stubs_removed", stubs },
                               { "ms", msSince( t0 ) },
                               { "stage_ms", { { "take_up", takeMs }, { "joint", jointWallMs }, { "lay", layMs },
                                               { "route", routeMs }, { "sweep", msSince( tRoute ) - routeMs } } } };

        if( j.status != 200 )
            rec["joint_error"] = j.body;

        if( !failures.empty() )
            rec["lay_failures"] = failures;

        for( const BOX2I& box : aBoxes )
            rec["boxes"].push_back( boxJson( box ) );

        return rec;
    };

    auto shortest = []( const std::vector<OPEN>& aList )
    {
        return *std::min_element( aList.begin(), aList.end(), []( const OPEN& a, const OPEN& b ) { return a.Span() < b.Span(); } );
    };

    std::map<int, std::vector<OPEN>> opens = opensNow( board, targets );
    const std::set<int>              startSplit = netsOf( opens );
    std::set<std::set<int>>          seen = { startSplit };
    std::map<int, int>               tries;
    std::set<int>                    gaveUp;
    nlohmann::json                   steps = nlohmann::json::array();
    bool                             stopped = false;

    for( int it = 0; it < maxIter; ++it )
    {
        std::vector<int> todo;

        for( const auto& [net, list] : opens )
        {
            if( !gaveUp.count( net ) )
                todo.push_back( net );
        }

        if( todo.empty() )
            break;

        std::sort( todo.begin(), todo.end(), [&]( int a, int b )
                   {
                       const int64_t sa = shortest( opens[a] ).Span(), sb = shortest( opens[b] ).Span();
                       return std::tie( tries[a], sa, a ) < std::tie( tries[b], sb, b );
                   } );

        const int net = todo.front();

        if( !KopenapiJobStep( "endgame", it, maxIter, netName( net ),
                              { { "split", opens.size() }, { "start", startSplit.size() }, { "given_up", gaveUp.size() } } ) )
        {
            stopped = true;
            break;
        }

        const std::set<int> before = netsOf( opens );
        UNDO                undo;
        undo.Begin( board );

        nlohmann::json step = { { "iter", it }, { "net", netName( net ) }, { "try", tries[net] } };
        // the last try: the group's copper since the base taken up whole, negotiated over the whole board
        const bool whole = wholeLast && tries[net] >= maxTries - 1;
        step["spot"] = spot( undo, { net }, { boxAround( shortest( opens[net] ), marginMm * ( 1 + tries[net] ) ) }, opens, whole );

        std::map<int, std::vector<OPEN>> now = opensNow( board, targets );
        nlohmann::json                   repairs = nlohmann::json::array();

        // the nets the step newly split: a box round each one's open connection, wider each round
        for( int r = 0; r < repairRounds && now.size() >= before.size() && !KopenapiJobCancelled(); ++r )
        {
            std::set<int>      fresh;
            std::vector<BOX2I> boxes;

            for( const auto& [n, list] : now )
            {
                if( before.count( n ) )
                    continue;

                fresh.insert( n );
                const BOX2I b = boxAround( shortest( list ), repairMarginMm * ( 1 + r ) );

                if( std::none_of( boxes.begin(), boxes.end(), [&]( const BOX2I& o ) { return o.Intersects( b ); } ) )
                    boxes.push_back( b );
            }

            if( fresh.empty() )
                break;

            const size_t   splitBefore = now.size();
            nlohmann::json rep = spot( undo, fresh, boxes, now, false );
            rep["nets"] = names( fresh );
            now = opensNow( board, targets );
            rep["split"] = { splitBefore, now.size() };
            repairs.push_back( rep );
        }

        if( !repairs.empty() )
            step["repairs"] = repairs;

        const std::set<int> after = netsOf( now );
        const bool          better = after.size() < before.size();
        const bool          side = !better && sideways && after.size() == before.size() && !seen.count( after );

        step["split"] = { before.size(), after.size() };
        step["joined"] = names( [&] { std::set<int> s; std::set_difference( before.begin(), before.end(), after.begin(), after.end(), std::inserter( s, s.end() ) ); return s; }() );
        step["split_new"] = names( [&] { std::set<int> s; std::set_difference( after.begin(), after.end(), before.begin(), before.end(), std::inserter( s, s.end() ) ); return s; }() );

        if( better || side )
        {
            step["kept"] = better ? "better" : "sideways";
            seen.insert( after );
            opens = std::move( now );

            if( opens.count( net ) && ++tries[net] >= maxTries )
                gaveUp.insert( net );
        }
        else
        {
            step["kept"] = false;
            undo.Restore( *context, board );
            opens = opensNow( board, targets );

            if( ++tries[net] >= maxTries )
                gaveUp.insert( net );
        }

        step["ms"] = msSince( started );
        steps.push_back( step );

        if( KopenapiJobCancelled() )
        {
            stopped = true;
            break;
        }
    }

    const std::set<int> endSplit = netsOf( opens );

    return KOPENAPI_RESULT::Ok( { { "split", { { "start", startSplit.size() }, { "end", endSplit.size() } } },
                                  { "split_nets", names( endSplit ) },
                                  { "given_up", names( gaveUp ) },
                                  { "steps", steps },
                                  { "stopped", stopped },
                                  { "ms", msSince( started ) } } );
}


KOPENAPI_REGISTER( "pcb_route_endgame",
                   "EXPERIMENTAL. The nets a routed board leaves split, one spot at a time (docs "
                   "router-architecture/endgame-cpp.md): the box round a split net's shortest open connection "
                   "(margin_mm, wider each failed try; the last try the whole board, whole_last); the target nets with copper laid since the base board touching "
                   "it join the group, their copper inside the box is taken up; pcb_route_oracle_joint negotiates the "
                   "group over the box (once more with retry_rounds when it does not converge); the joined, unshared "
                   "paths are laid by pcb_track_add, the router takes the group's nets still split. The nets the step "
                   "split anew get repair rounds (a box round each, wider each round). A step stays when fewer target nets are split, or "
                   "as many but a set not held before (sideways); otherwise the copper goes back as it was. "
                   "Answer: split {start, end}, split_nets, given_up, steps[] (net, try, spot {boxes, group, taken_up, "
                   "converged, retried, sharing, laid, lay_failed, routed}, repairs, split [before, after], joined, "
                   "split_new, kept)",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"array","items":{"type":"string"},"description":"the target nets (default every net)"},
                        "base":{"type":"string","description":"a board file: its tracks and vias (by uuid) are never taken up (the fan-out and supply copper before routing)"},
                        "margin_mm":{"type":"number","default":2},
                        "repair_margin_mm":{"type":"number","default":2},
                        "repair_rounds":{"type":"integer","default":3},
                        "max_iter":{"type":"integer","default":40},
                        "max_tries":{"type":"integer","default":3,"description":"tries per net before it is given up"},
                        "retry_rounds":{"type":"integer","default":150},
                        "sideways":{"type":"boolean","default":true},
                        "whole_last":{"type":"boolean","default":true,"description":"a net's last try takes up the group's copper since the base whole and negotiates over the whole board"},
                        "joint":{"type":"object","description":"pcb_route_oracle_joint arguments (default cell_mm 0.05, rounds 60, claim_slack_mm 0)"}},
                        "required":[]})json"_json,
                   false, h_pcb_route_endgame, 7200 );

KOPENAPI_MARK_EDITING( "pcb_route_endgame" );
