/// @file methods_pcb_route_timeline.cpp
/// @brief pcb_route_timeline: replay of the last grid route recorded in this process
/// (docs/inprogress/2026-10-19-route-replay/design.md): each variant's steps, the copper at any step
/// as a difference against the step the client shows, the heat of the places fought over.
#include "kopenapi_pcb.h"
#include "route_timeline.h"

#include <kicadopenapi_registry.h>

#include <base_units.h>
#include <lset.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>


namespace
{

// set on the main thread, read there and by event streams on HTTP threads
std::mutex                     g_lastMutex;
std::shared_ptr<GRID_TIMELINE> g_last;

double mm( double aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1000.0 ) / 1000.0;
}


nlohmann::json pointMm( const VECTOR2I& aP )
{
    return { mm( aP.x ), mm( aP.y ) };
}


nlohmann::json boxMm( const GRID_TIMELINE& aT, const int* aBox )
{
    if( aBox[0] < 0 )
        return nullptr;

    const VECTOR2I a = aT.grid.Centre( aBox[0], aBox[1] ), b = aT.grid.Centre( aBox[2], aBox[3] );
    return { mm( a.x ), mm( a.y ), mm( b.x ), mm( b.y ) };
}


nlohmann::json stepJson( const GRID_TIMELINE& aT, const GRID_STEP& aStep, size_t aIndex )
{
    return { { "step", aIndex },
             { "stage", GRID_TIMELINE::STAGES[aStep.stage] },
             { "round", aStep.round },
             { "net", aT.netNames[aStep.net] },
             { "path", aStep.path == GRID_STEP::NONE ? nlohmann::json() : nlohmann::json( aStep.path ) },
             { "crowded", aStep.crowded },
             { "expanded", aStep.expanded },
             { "ms", std::round( aStep.ms * 10.0f ) / 10.0 },
             { "search_box_mm", boxMm( aT, aStep.box ) } };
}


nlohmann::json pathJson( const GRID_TIMELINE& aT, const GRID_PATH& aPath )
{
    const GRID_NET_RESULT g = aT.Geometry( aPath );
    nlohmann::json        tracks = nlohmann::json::array();
    nlohmann::json        vias = nlohmann::json::array();

    for( const GRID_TRACK& t : g.tracks )
    {
        nlohmann::json pts = nlohmann::json::array();

        for( const VECTOR2I& p : t.points )
            pts.push_back( pointMm( p ) );

        tracks.push_back( { { "layer", LSET::Name( t.layer ).ToStdString() }, { "points_mm", pts } } );
    }

    for( const VECTOR2I& v : g.vias )
        vias.push_back( pointMm( v ) );

    return { { "tracks", tracks }, { "vias_mm", vias }, { "complete", g.complete } };
}


/// @brief A step index from the request: negative counts from the end, "none" / -inf before the first
bool stepArg( const nlohmann::json& aArgs, const char* aKey, const GRID_LANE& aLane, size_t& aStep, bool& aGiven )
{
    aGiven = aArgs.contains( aKey ) && !aArgs[aKey].is_null();

    if( !aGiven )
        return true;

    if( !aArgs[aKey].is_number_integer() )
        return false;

    long long v = aArgs[aKey].get<long long>();
    const long long n = (long long) aLane.steps.size();

    if( v < 0 )
        v += n;

    if( v < 0 || v >= n )
        return false;

    aStep = (size_t) v;
    return true;
}


/// @brief Any thread: reads the lanes under their locks; the winner only once the route has ended
KOPENAPI_RESULT timelineQuery( const std::shared_ptr<GRID_TIMELINE>& t, const nlohmann::json& aArgs )
{
    const auto started = std::chrono::steady_clock::now();

    if( !t )
        return KOPENAPI_RESULT::Error( 404, "no grid route recorded in this process: pcb_route {engine: 'grid', record: true}" );

    if( aArgs.contains( "run" ) && aArgs["run"] != t->run )
        return KOPENAPI_RESULT::Error( 404, "only the last recorded run is kept: run " + std::to_string( t->run ) );

    const std::string verb = aArgs.value( "verb", "status" );
    nlohmann::json    out = { { "run", t->run } };

    if( verb == "status" )
    {
        const bool     live = t->live;   // the winner and abandoned lanes are known once ended
        nlohmann::json lanes = nlohmann::json::array();

        for( size_t k = 0; k < t->LaneCount(); ++k )
        {
            const GRID_LANE&                      l = t->Lane( k );
            std::lock_guard<std::recursive_mutex> lock( l.Mutex() );
            lanes.push_back( { { "lane", k }, { "variant", l.name }, { "steps", l.steps.size() }, { "paths", l.paths.size() },
                               { "rounds", l.histByRound.size() }, { "abandoned", !live && l.abandoned },
                               { "winner", !live && (int) k == t->winner },
                               { "ms", l.steps.empty() ? 0.0 : std::round( l.steps.back().ms ) },
                               { "bytes", l.Bytes() } } );

            if( l.parent >= 0 )
            {
                lanes.back()["parent"] = l.parent;
                lanes.back()["parent_step"] = l.parentStep;
                lanes.back()["running"] = l.running;
            }
        }

        out["nets"] = t->netNames.size();
        out["cell_mm"] = mm( t->grid.cell );
        out["tile_mm"] = mm( (double) t->grid.cell * t->tile );
        out["grid_mm"] = { mm( t->grid.Centre( 0, 0 ).x ), mm( t->grid.Centre( 0, 0 ).y ),
                           mm( t->grid.Centre( t->grid.nx - 1, t->grid.ny - 1 ).x ),
                           mm( t->grid.Centre( t->grid.nx - 1, t->grid.ny - 1 ).y ) };
        out["layers"] = [&]
        {
            nlohmann::json names = nlohmann::json::array();

            for( PCB_LAYER_ID l : t->layers )
                names.push_back( LSET::Name( l ).ToStdString() );

            return names;
        }();
        out["winner"] = live ? -1 : t->winner;
        out["applied_lane"] = t->appliedLane;
        out["world"] = [&]
        {
            char hex[17];
            snprintf( hex, sizeof( hex ), "%016llx", (unsigned long long) t->world );
            return std::string( hex );
        }();
        out["live"] = live;
        out["bytes"] = t->Bytes();
        out["lanes"] = lanes;
        out["stages"] = GRID_TIMELINE::STAGES;
        return KOPENAPI_RESULT::Ok( out );
    }

    int lane = 0;

    if( !aArgs.contains( "lane" ) )
    {
        if( !t->live )
            lane = std::max( t->winner, 0 );
    }
    else
    {
        if( aArgs["lane"].is_number_integer() )
        {
            lane = aArgs["lane"].get<int>();
        }
        else if( aArgs["lane"].is_string() )
        {
            lane = -1;

            for( size_t k = 0; k < t->LaneCount(); ++k )
            {
                if( t->Lane( k ).name == aArgs["lane"].get<std::string>() )
                    lane = (int) k;
            }
        }
    }

    if( lane < 0 || lane >= (int) t->LaneCount() )
        return KOPENAPI_RESULT::Error( 404, "no such lane (see verb status)" );

    const GRID_LANE&                      L = t->Lane( lane );
    std::lock_guard<std::recursive_mutex> lock( L.Mutex() );
    out["live"] = t->live.load();
    out["lane"] = lane;
    out["variant"] = L.name;
    out["steps"] = L.steps.size();

    if( verb == "series" )
    {
        nlohmann::json stage = nlohmann::json::array(), round = nlohmann::json::array(), net = nlohmann::json::array(),
                       crowded = nlohmann::json::array(), expanded = nlohmann::json::array(), ms = nlohmann::json::array(),
                       places = nlohmann::json::array();

        for( const GRID_STEP& s : L.steps )
        {
            stage.push_back( s.stage );
            round.push_back( s.round );
            net.push_back( s.net );
            crowded.push_back( s.crowded );
            expanded.push_back( s.expanded );
            ms.push_back( std::round( s.ms * 10.0f ) / 10.0 );
            places.push_back( L.conflictSets.empty() ? 0 : L.conflictSets[s.conflicts].size() );
        }

        out["nets"] = t->netNames;
        out["series"] = { { "stage", stage }, { "round", round }, { "net", net }, { "crowded", crowded },
                          { "expanded", expanded }, { "ms", ms }, { "crowded_tiles", places } };
        return KOPENAPI_RESULT::Ok( out );
    }

    if( verb == "markers" )
    {
        nlohmann::json marks = nlohmann::json::array();

        for( size_t k = 0; k < L.steps.size(); ++k )
        {
            const GRID_STEP& s = L.steps[k];
            const GRID_STEP* p = k ? &L.steps[k - 1] : nullptr;

            if( !p || s.stage != p->stage )
                marks.push_back( { { "step", k }, { "kind", "stage" }, { "stage", GRID_TIMELINE::STAGES[s.stage] } } );
            else if( s.round != p->round )
                marks.push_back( { { "step", k }, { "kind", "round" }, { "round", s.round }, { "crowded", s.crowded } } );

            if( s.stage == std::size( GRID_TIMELINE::STAGES ) - 1 )
                marks.push_back( { { "step", k }, { "kind", "dropped" }, { "net", t->netNames[s.net] } } );
        }

        if( !L.steps.empty() )
            marks.push_back( { { "step", L.steps.size() - 1 }, { "kind", t->live ? "head" : L.abandoned ? "abandoned" : "end" } } );

        out["markers"] = marks;
        return KOPENAPI_RESULT::Ok( out );
    }

    if( verb != "frame" )
        return KOPENAPI_RESULT::Error( 400, "verb: status, frame, series or markers" );

    if( L.steps.empty() )
        return KOPENAPI_RESULT::Error( 404, "the lane has no steps" );

    size_t at = L.steps.size() - 1, since = SIZE_MAX;
    bool   atGiven = false, sinceGiven = false;

    if( !stepArg( aArgs, "at", L, at, atGiven ) || !stepArg( aArgs, "since", L, since, sinceGiven ) )
        return KOPENAPI_RESULT::Error( 400, "at / since: a step of the lane (0 … steps - 1, negative from the end)" );

    std::vector<uint32_t> now, before;
    L.TableAt( at, now );
    L.TableAt( since, before );

    const bool     geometry = aArgs.value( "geometry", true );
    nlohmann::json changed = nlohmann::json::array();
    nlohmann::json cleared = nlohmann::json::array();

    for( size_t n = 0; n < now.size(); ++n )
    {
        if( now[n] == before[n] )
            continue;

        if( now[n] == GRID_STEP::NONE )
        {
            cleared.push_back( t->netNames[n] );
            continue;
        }

        nlohmann::json row = { { "net", t->netNames[n] }, { "path", now[n] } };

        if( geometry )
            row.update( pathJson( *t, L.paths[now[n]] ) );

        changed.push_back( row );
    }

    out["at"] = stepJson( *t, L.steps[at], at );
    out["since"] = sinceGiven ? nlohmann::json( since ) : nlohmann::json();
    out["changed"] = changed;
    out["cleared"] = cleared;

    // the heat of the last round finished by then, when it differs from the one at 'since'
    const int round = t->HeatRound( L, at );
    const int roundBefore = !sinceGiven ? 0 : t->HeatRound( L, since );

    if( aArgs.value( "heat", false ) && round > 0 && ( round != roundBefore || !sinceGiven ) )
    {
        const std::vector<uint16_t>& h = L.heatByRound[std::min<size_t>( round, L.heatByRound.size() ) - 1];
        out["heat"] = { { "round", round }, { "nx", t->tnx }, { "ny", t->tny },
                        { "tile_mm", mm( (double) t->grid.cell * t->tile ) },
                        { "origin_mm", pointMm( t->grid.Centre( 0, 0 ) ) },
                        { "max", h.empty() ? 0 : *std::max_element( h.begin(), h.end() ) }, { "values", h } };

        const std::vector<uint8_t> age = t->FightAge( L, round );
        out["heat"]["age"] = age;
    }

    // the places crowded right after the step: tile index and crowded states there
    if( aArgs.value( "heat", false ) && !L.conflictSets.empty() )
    {
        nlohmann::json tiles = nlohmann::json::array(), counts = nlohmann::json::array();

        for( uint32_t v : L.conflictSets[L.steps[at].conflicts] )
        {
            tiles.push_back( v >> 8 );
            counts.push_back( v & 0xff );
        }

        out["crowded"] = { { "tiles", tiles }, { "counts", counts } };
    }

    out["server_us"] = std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - started ).count();
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT h_pcb_route_timeline( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    (void) aCtx;
    return timelineQuery( GridTimelineLast(), aArgs );
}


/// @brief Event stream of a route's timeline: 'start' when the run is there, 'frame' (verb frame
/// from the step sent before) whenever the lane's steps grow, 'end' once the route has ended
KOPENAPI_RESULT openTimelineEvents( const nlohmann::json& aArgs, KOPENAPI_EVENT_POLL& aPoll )
{
    struct STATE
    {
        std::shared_ptr<GRID_TIMELINE>        timeline;
        int                                   afterRun = -1;
        nlohmann::json                        lane;
        bool                                  geometry = true;
        bool                                  heat = false;
        size_t                                sent = SIZE_MAX;
        std::chrono::steady_clock::time_point waitUntil;
    };

    auto st = std::make_shared<STATE>();

    if( aArgs.contains( "after_run" ) && !aArgs["after_run"].is_number_integer() )
        return KOPENAPI_RESULT::Error( 400, "after_run: a run id" );

    if( aArgs.contains( "lane" ) && !aArgs["lane"].is_number_integer() && !aArgs["lane"].is_string() )
        return KOPENAPI_RESULT::Error( 400, "lane: index or variant name" );

    if( aArgs.contains( "since" ) && !aArgs["since"].is_number_integer() )
        return KOPENAPI_RESULT::Error( 400, "since: a step" );

    st->afterRun = aArgs.value( "after_run", -1 );
    st->lane = aArgs.value( "lane", nlohmann::json( 0 ) );
    st->geometry = aArgs.value( "geometry", true );
    st->heat = aArgs.value( "heat", false );
    st->waitUntil = std::chrono::steady_clock::now() + std::chrono::seconds( std::clamp( aArgs.value( "wait_s", 60 ), 0, 3600 ) );

    if( aArgs.contains( "since" ) && aArgs["since"].get<long long>() >= 0 )
        st->sent = aArgs["since"].get<size_t>();

    aPoll = [st]( std::vector<KOPENAPI_EVENT>& aOut )
    {
        if( !st->timeline )
        {
            std::shared_ptr<GRID_TIMELINE> t = GridTimelineLast();

            if( !t || t->run <= st->afterRun )
            {
                if( std::chrono::steady_clock::now() < st->waitUntil )
                    return true;

                aOut.push_back( { "end", { { "reason", "no run recorded after run " + std::to_string( st->afterRun ) } } } );
                return false;
            }

            KOPENAPI_RESULT status = timelineQuery( t, { { "verb", "status" } } );
            nlohmann::json  lanes = status.body["lanes"];
            int             lane = -1;

            for( const nlohmann::json& l : lanes )
            {
                if( l["lane"] == st->lane || l["variant"] == st->lane )
                    lane = l["lane"].get<int>();
            }

            if( lane < 0 )
            {
                aOut.push_back( { "error", { { "error", "no such lane in run " + std::to_string( t->run ) } } } );
                return false;
            }

            st->timeline = t;
            st->lane = lane;
            status.body.erase( "lanes" );
            status.body.erase( "winner" );
            status.body.erase( "bytes" );
            status.body["lane"] = lane;
            status.body["variant"] = lanes[lane]["variant"];
            aOut.push_back( { "start", status.body } );
        }

        const GRID_TIMELINE& t = *st->timeline;
        const GRID_LANE&     L = t.Lane( st->lane.get<int>() );
        const bool           live = t.live;   // before the steps: once ended, every step is in
        size_t               steps = 0;

        {
            std::lock_guard<std::recursive_mutex> lock( L.Mutex() );
            steps = L.steps.size();
        }

        if( steps && ( st->sent == SIZE_MAX || st->sent < steps - 1 ) )
        {
            nlohmann::json q = { { "verb", "frame" }, { "lane", st->lane }, { "at", steps - 1 },
                                 { "geometry", st->geometry }, { "heat", st->heat } };

            if( st->sent != SIZE_MAX )
                q["since"] = st->sent;

            KOPENAPI_RESULT frame = timelineQuery( st->timeline, q );
            aOut.push_back( { frame.status == 200 ? "frame" : "error", frame.body } );
            st->sent = steps - 1;
        }

        if( live )
            return true;

        aOut.push_back( { "end", { { "run", t.run }, { "lane", st->lane }, { "steps", steps }, { "winner", t.winner },
                                   { "abandoned", L.abandoned } } } );
        return false;
    };

    return KOPENAPI_RESULT::Ok( nlohmann::json::object() );
}

} // namespace


void GridTimelineKeep( std::shared_ptr<GRID_TIMELINE> aTimeline )
{
    {
        std::lock_guard<std::mutex> lock( g_lastMutex );
        g_last = std::move( aTimeline );
    }

    KopenapiEventNotify( "pcb_route_timeline" );
}


std::shared_ptr<GRID_TIMELINE> GridTimelineLast()
{
    std::lock_guard<std::mutex> lock( g_lastMutex );
    return g_last;
}


KOPENAPI_REGISTER_EVENT_TOPIC( "pcb_route_timeline",
                               "a grid route's timeline (pcb_route record: true) as it grows: 'start' (run, lane, "
                               "variant, nets, grid) once a run newer than after_run (default: any) is recorded "
                               "(waits wait_s, default 60), 'frame' (pcb_route_timeline verb frame: the nets changed "
                               "since the step sent before, or since 'since') whenever the lane's steps grow, 'end' "
                               "(winner, steps) when the route has ended. lane: index or variant name, default 0; "
                               "geometry (default true), heat (default false) as in pcb_route_timeline",
                               openTimelineEvents );


KOPENAPI_REGISTER( "pcb_route_timeline",
                   "EXPERIMENTAL. Replay of the last grid route recorded in this process (pcb_route {engine: "
                   "'grid', record: true}): each variant (lane) took nets one by one - routed in a negotiation "
                   "round, legalised, routed short again, with fewer vias, pulled tight, dropped - and every "
                   "step is kept. verb status: lanes, steps, sizes; frame: the copper at step 'at' (default the "
                   "last) as the nets that changed since step 'since' (default: from nothing) with their tracks "
                   "and vias (geometry: false for path ids only; a path id is the same copper wherever it "
                   "appears) and the nets cleared, the step's stage / round / net / crowded / search box, the "
                   "heat of the places fought over (heat: true; max history cost per tile, and age: rounds "
                   "since the tile was last fought over, 255 never) when the round changed, and the places crowded right after the step (heat: true; crowded.tiles indexes "
                   "the heat grid, counts the crowded states); series: stage / round / net / crowded / "
                   "expanded / ms / crowded_tiles per step; markers: stage and "
                   "round starts, dropped nets, the end (or the head while live). lane: index or variant name, default the "
                   "winner (lane 0 while live). The timeline is readable while the route runs (live: true; "
                   "steps grow; poll frame {since} from the client's step, or over REST receive the frames "
                   "pushed: GET /api/v1/events?topic=pcb_route_timeline, Server-Sent Events)",
                   R"json({"type":"object","properties":{
                        "verb":{"type":"string","enum":["status","frame","series","markers"],"default":"status"},
                        "run":{"type":"integer","description":"the run id from status (only the last run is kept)"},
                        "lane":{"description":"lane index or variant name; default the winner"},
                        "at":{"type":"integer","description":"step; negative from the end; default the last"},
                        "since":{"type":"integer","description":"the step the client shows; default none (every net)"},
                        "geometry":{"type":"boolean","default":true},
                        "heat":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_route_timeline );
