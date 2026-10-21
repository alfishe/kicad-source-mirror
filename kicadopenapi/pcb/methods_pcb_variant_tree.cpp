/// @file methods_pcb_variant_tree.cpp
/// @brief pcb_variant_tree: places on the board's history and the router's recorded runs to come
/// back to (docs/inprogress/2026-10-19-route-replay/design-tree.md). A node is a run, lane and step
/// plus the board's edit-history entry; nodes on the same run form the tree of its lanes, a run
/// hangs from the node it started from.
#include "kopenapi_pcb.h"
#include "route_timeline.h"
#include "variant_tree.h"

#include <kicadopenapi_history.h>
#include <kicadopenapi_registry.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <mutex>
#include <set>


namespace
{

struct VARIANT_NODE
{
    int                      id = 0;
    int                      parent = 0;    ///< start / edit nodes; a run's nodes: from the lanes (parentOf)
    std::string              name;
    std::string              kind;          ///< start, edit, end, branch, pause, step
    std::vector<std::string> tags;
    int                      run = 0;       ///< 0: a board state outside any run
    int                      lane = -1;
    size_t                   step = 0;
    std::string              variant;
    bool                     warm = false;
    /// @brief The lanes the node's state went through: (lane, its last step in the chain), the
    /// node's own lane first
    std::vector<std::pair<int, size_t>> chain;
    long                     boardSeq = 0;  ///< the edit-history entry the board is after (0: as opened)
    nlohmann::json           metrics;
    long long                created = 0;   ///< ms since the epoch
};


std::mutex                g_mutex;
std::vector<VARIANT_NODE> g_nodes;
std::map<int, int>        g_runOrigin;     ///< run: the node it started from
std::map<int, long>       g_runStartSeq;   ///< run: the board's edit-history entry it started after
int                       g_nextId = 1;
int                       g_current = 0;   ///< the node the board was last made / found at
long                      g_currentSeq = -1;


VARIANT_NODE* findNode( int aId )
{
    for( VARIANT_NODE& n : g_nodes )
    {
        if( n.id == aId )
            return &n;
    }

    return nullptr;
}


/// @brief The board is still the current node's: no edit since, the document not reopened
int currentNode()
{
    KOPENAPI_HISTORY& h = KOPENAPI_HISTORY::Get();

    if( !g_current || g_currentSeq != h.LastSeq() || ( g_currentSeq > 0 && !h.Holds( g_currentSeq, "pcb" ) ) )
        return 0;

    return findNode( g_current ) ? g_current : 0;
}


/// @brief The board after the running editing call (or now, outside one) is aId's
void setCurrent( int aId )
{
    KOPENAPI_HISTORY& h = KOPENAPI_HISTORY::Get();
    g_current = aId;
    g_currentSeq = h.PendingSeq() > 0 ? h.PendingSeq() : h.LastSeq();
}


long long nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() ).count();
}


/// @brief A step shared with the parent lane (up to the branch point) belongs to the parent
void normalize( const GRID_TIMELINE& aT, int& aLane, size_t& aStep )
{
    while( aT.Lane( aLane ).parent >= 0 && aStep <= aT.Lane( aLane ).parentStep )
        aLane = aT.Lane( aLane ).parent;
}


std::vector<std::pair<int, size_t>> chainOf( const GRID_TIMELINE& aT, int aLane, size_t aStep )
{
    std::vector<std::pair<int, size_t>> chain = { { aLane, aStep } };

    for( int l = aLane; aT.Lane( l ).parent >= 0; l = aT.Lane( l ).parent )
        chain.push_back( { aT.Lane( l ).parent, aT.Lane( l ).parentStep } );

    return chain;
}


bool warmChain( const GRID_TIMELINE& aT, const std::vector<std::pair<int, size_t>>& aChain )
{
    for( const auto& [lane, step] : aChain )
    {
        if( aT.Lane( lane ).world )
            return true;
    }

    return false;
}


nlohmann::json stepMetrics( const GRID_TIMELINE& aT, int aLane, size_t aStep )
{
    const GRID_LANE&                      L = aT.Lane( aLane );
    std::lock_guard<std::recursive_mutex> lock( L.Mutex() );

    if( aStep >= L.steps.size() )
        return nlohmann::json::object();

    std::vector<uint32_t> table;
    L.TableAt( aStep, table );
    const GRID_STEP& s = L.steps[aStep];

    return { { "stage", GRID_TIMELINE::STAGES[s.stage] }, { "round", s.round }, { "crowded", s.crowded },
             { "expanded", s.expanded }, { "ms", std::round( s.ms ) },
             { "nets_with_copper", std::count_if( table.begin(), table.end(), []( uint32_t p ) { return p != GRID_STEP::NONE; } ) },
             { "nets", table.size() } };
}


/// @brief Under g_mutex: the node at a run's place, made when there is none
int nodeAt( const GRID_TIMELINE& aT, int aLane, size_t aStep, const std::string& aKind, const nlohmann::json& aMetrics )
{
    normalize( aT, aLane, aStep );

    for( const VARIANT_NODE& n : g_nodes )
    {
        if( n.run == aT.run && n.lane == aLane && n.step == aStep )
            return n.id;
    }

    VARIANT_NODE n;
    n.id = g_nextId++;
    n.kind = aKind;
    n.run = aT.run;
    n.lane = aLane;
    n.step = aStep;
    n.variant = aT.Lane( aLane ).name;
    n.chain = chainOf( aT, aLane, aStep );
    n.warm = warmChain( aT, n.chain );
    n.name = n.variant + " @ step " + std::to_string( aStep );
    n.boardSeq = g_runStartSeq.count( aT.run ) ? g_runStartSeq[aT.run] : 0;
    n.metrics = stepMetrics( aT, aLane, aStep );
    n.metrics.update( aMetrics );
    n.created = nowMs();
    g_nodes.push_back( n );
    return n.id;
}


/// @brief Under g_mutex: a board state outside any run (start, edit) hung from the node last current
int boardNode( const std::string& aKind, const std::string& aName )
{
    VARIANT_NODE n;
    n.id = g_nextId++;
    n.parent = findNode( g_current ) ? g_current : 0;
    n.kind = aKind;
    n.name = aName;
    n.boardSeq = KOPENAPI_HISTORY::Get().LastSeq();
    n.created = nowMs();
    g_nodes.push_back( n );
    return n.id;
}


/// @brief Under g_mutex: the node's parent; a run's node hangs from the deepest node before it on its
/// lane chain, else from the node the run started from
int parentOf( const VARIANT_NODE& aNode )
{
    if( !aNode.run )
        return aNode.parent;

    int    best = 0;
    size_t bestStep = 0;

    for( const VARIANT_NODE& n : g_nodes )
    {
        if( n.run != aNode.run || n.id == aNode.id )
            continue;

        for( const auto& [lane, last] : aNode.chain )
        {
            if( n.lane == lane && n.step <= last && !( lane == aNode.lane && n.step >= aNode.step )
                && ( !best || n.step > bestStep ) )
            {
                best = n.id;
                bestStep = n.step;
            }
        }
    }

    if( best )
        return best;

    auto it = g_runOrigin.find( aNode.run );
    return it != g_runOrigin.end() && findNode( it->second ) ? it->second : 0;
}


nlohmann::json nodeJson( const VARIANT_NODE& n, const std::set<int>& aKept, int aCurrent )
{
    KOPENAPI_HISTORY& h = KOPENAPI_HISTORY::Get();
    nlohmann::json    j = { { "id", n.id }, { "parent", parentOf( n ) }, { "name", n.name }, { "kind", n.kind },
                            { "tags", n.tags }, { "board_seq", n.boardSeq },
                            { "board", n.boardSeq == 0 || h.Holds( n.boardSeq, "pcb" ) ? "held" : "lost" },
                            { "pinned", false }, { "current", n.id == aCurrent }, { "created_ms", n.created },
                            { "metrics", n.metrics } };

    if( n.run )
    {
        j["run"] = n.run;
        j["lane"] = n.lane;
        j["variant"] = n.variant;
        j["step"] = n.step;
        j["warm"] = n.warm;
        j["timeline"] = aKept.count( n.run ) ? "memory" : "evicted";
    }

    return j;
}


std::set<int> keptRuns( size_t* aBytes = nullptr, nlohmann::json* aRows = nullptr )
{
    std::set<int> runs;

    for( const std::shared_ptr<GRID_TIMELINE>& t : GridTimelinesKept() )
    {
        runs.insert( t->run );
        const size_t b = t->Bytes();

        if( aBytes )
            *aBytes += b;

        if( aRows )
            aRows->push_back( { { "run", t->run }, { "bytes", b }, { "live", t->live.load() }, { "lanes", t->LaneCount() } } );
    }

    return runs;
}


KOPENAPI_RESULT treeList()
{
    size_t         bytes = 0;
    nlohmann::json timelines = nlohmann::json::array();
    std::set<int>  kept = keptRuns( &bytes, &timelines );
    std::lock_guard<std::mutex> lock( g_mutex );
    const int      current = currentNode();
    nlohmann::json nodes = nlohmann::json::array();

    for( const VARIANT_NODE& n : g_nodes )
        nodes.push_back( nodeJson( n, kept, current ) );

    return KOPENAPI_RESULT::Ok( { { "nodes", nodes },
                                  { "current", current ? nlohmann::json( current ) : nlohmann::json() },
                                  { "edited_since", current || !g_current ? nlohmann::json() : nlohmann::json( g_current ) },
                                  { "timelines", timelines }, { "timeline_bytes", bytes },
                                  { "budget_mb", GridTimelineBudget() >> 20 } } );
}


KOPENAPI_RESULT treeNode( const nlohmann::json& aArgs )
{
    const nlohmann::json at = aArgs.value( "at", nlohmann::json( "current" ) );
    int                  id = 0;

    if( at.is_string() && at.get<std::string>() == "current" )
    {
        std::lock_guard<std::mutex> lock( g_mutex );
        id = currentNode();

        if( !id )
        {
            id = boardNode( "edit", "edited" );
            setCurrent( id );
        }
    }
    else if( at.is_object() && at.contains( "run" ) && at["run"].is_number_integer() && at.contains( "step" )
             && at["step"].is_number_integer() )
    {
        std::shared_ptr<GRID_TIMELINE> t = GridTimelineFind( at["run"].get<int>() );

        if( !t )
            return KOPENAPI_RESULT::Error( 404, "run " + at["run"].dump() + " is not kept (list: timelines)" );

        if( t->live )
            return KOPENAPI_RESULT::Error( 409, "run " + std::to_string( t->run ) + " is still routing" );

        int lane = at.contains( "lane" ) ? -1 : t->DefaultLane();

        for( size_t k = 0; k < t->LaneCount() && lane < 0; ++k )
        {
            if( ( at["lane"].is_number_integer() && at["lane"].get<int>() == (int) k )
                || ( at["lane"].is_string() && at["lane"].get<std::string>() == t->Lane( k ).name ) )
            {
                lane = (int) k;
            }
        }

        if( lane < 0 || t->Lane( lane ).deleted )
            return KOPENAPI_RESULT::Error( 404, "at.lane: no such lane in run " + std::to_string( t->run ) );

        long long step = at["step"].get<long long>();
        const long long steps = (long long) t->Lane( lane ).steps.size();

        if( step < 0 )
            step += steps;

        if( step < 0 || step >= steps )
            return KOPENAPI_RESULT::Error( 400, "at.step: 0.." + std::to_string( steps - 1 ) + " (negative from the end)" );

        std::lock_guard<std::mutex> lock( g_mutex );
        id = nodeAt( *t, lane, (size_t) step, "step", nlohmann::json::object() );
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "at: \"current\" or {run, lane, step}" );
    }

    std::lock_guard<std::mutex> lock( g_mutex );
    VARIANT_NODE*               n = findNode( id );

    if( aArgs.contains( "name" ) && aArgs["name"].is_string() )
        n->name = aArgs["name"].get<std::string>();

    for( const nlohmann::json& tag : aArgs.value( "tags", nlohmann::json::array() ) )
    {
        if( tag.is_string() && std::find( n->tags.begin(), n->tags.end(), tag.get<std::string>() ) == n->tags.end() )
            n->tags.push_back( tag.get<std::string>() );
    }

    std::set<int> kept = keptRuns();
    return KOPENAPI_RESULT::Ok( { { "node", nodeJson( *n, kept, currentNode() ) } } );
}


KOPENAPI_RESULT treeEdit( const std::string& aVerb, const nlohmann::json& aArgs )
{
    if( !aArgs.contains( "id" ) || !aArgs["id"].is_number_integer() )
        return KOPENAPI_RESULT::Error( 400, "id: a node (verb list)" );

    std::lock_guard<std::mutex> lock( g_mutex );
    const int                   id = aArgs["id"].get<int>();
    VARIANT_NODE*               n = findNode( id );

    if( !n )
        return KOPENAPI_RESULT::Error( 404, "no node " + std::to_string( id ) );

    if( aVerb == "rename" )
    {
        if( !aArgs.contains( "name" ) || !aArgs["name"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "name: a string" );

        n->name = aArgs["name"].get<std::string>();
        return KOPENAPI_RESULT::Ok( { { "id", id }, { "name", n->name } } );
    }

    if( aVerb == "tag" )
    {
        for( const nlohmann::json& tag : aArgs.value( "add", nlohmann::json::array() ) )
        {
            if( tag.is_string() && std::find( n->tags.begin(), n->tags.end(), tag.get<std::string>() ) == n->tags.end() )
                n->tags.push_back( tag.get<std::string>() );
        }

        for( const nlohmann::json& tag : aArgs.value( "remove", nlohmann::json::array() ) )
        {
            if( tag.is_string() )
                n->tags.erase( std::remove( n->tags.begin(), n->tags.end(), tag.get<std::string>() ), n->tags.end() );
        }

        return KOPENAPI_RESULT::Ok( { { "id", id }, { "tags", n->tags } } );
    }

    // delete: the node and the nodes under it
    if( id == currentNode() )
        return KOPENAPI_RESULT::Error( 409, "node " + std::to_string( id ) + " is the board now: make or reach another node first" );

    std::map<int, int> parents;

    for( const VARIANT_NODE& m : g_nodes )
        parents[m.id] = parentOf( m );

    std::set<int> gone = { id };

    for( bool grew = true; grew; )
    {
        grew = false;

        for( const auto& [m, p] : parents )
        {
            if( gone.count( p ) && gone.insert( m ).second )
                grew = true;
        }
    }

    for( const VARIANT_NODE& m : g_nodes )
    {
        if( gone.count( m.id ) && std::find( m.tags.begin(), m.tags.end(), "locked" ) != m.tags.end() )
            return KOPENAPI_RESULT::Error( 409, "node " + std::to_string( m.id ) + " is locked (tag locked)" );
    }

    g_nodes.erase( std::remove_if( g_nodes.begin(), g_nodes.end(), [&]( const VARIANT_NODE& m ) { return gone.count( m.id ) > 0; } ),
                   g_nodes.end() );

    if( gone.count( g_current ) )
        g_current = 0;

    // a run whose start node went keeps its lanes' nodes hanging from the root
    for( auto& [run, origin] : g_runOrigin )
    {
        if( gone.count( origin ) )
            origin = 0;
    }

    return KOPENAPI_RESULT::Ok( { { "deleted", gone } } );
}


KOPENAPI_RESULT treeBudget( const nlohmann::json& aArgs )
{
    std::vector<int> evicted;

    if( aArgs.contains( "mb" ) )
    {
        if( !aArgs["mb"].is_number() || aArgs["mb"].get<double>() < 0 )
            return KOPENAPI_RESULT::Error( 400, "mb: the memory the kept runs' timelines may take" );

        evicted = GridTimelineSetBudget( (size_t) ( aArgs["mb"].get<double>() * 1048576.0 ) );
    }

    size_t         bytes = 0;
    nlohmann::json timelines = nlohmann::json::array();
    keptRuns( &bytes, &timelines );
    return KOPENAPI_RESULT::Ok( { { "budget_mb", GridTimelineBudget() / 1048576.0 }, { "evicted", evicted },
                                  { "timelines", timelines }, { "timeline_bytes", bytes } } );
}


KOPENAPI_RESULT h_pcb_variant_tree( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    (void) aCtx;
    const std::string verb = aArgs.value( "verb", "list" );

    if( verb == "list" )
        return treeList();

    if( verb == "node" )
        return treeNode( aArgs );

    if( verb == "rename" || verb == "tag" || verb == "delete" )
        return treeEdit( verb, aArgs );

    if( verb == "budget" )
        return treeBudget( aArgs );

    return KOPENAPI_RESULT::Error( 400, "verb: list, node, rename, tag, delete or budget" );
}

} // namespace


int VariantTreeRunStarted( const std::shared_ptr<GRID_TIMELINE>& aTimeline )
{
    std::lock_guard<std::mutex> lock( g_mutex );
    int                         origin = currentNode();

    if( !origin )
    {
        origin = boardNode( "start", "before run " + std::to_string( aTimeline->run ) );
        g_current = origin;
        g_currentSeq = KOPENAPI_HISTORY::Get().LastSeq();
    }

    g_runOrigin[aTimeline->run] = origin;
    g_runStartSeq[aTimeline->run] = KOPENAPI_HISTORY::Get().LastSeq();
    return origin;
}


int VariantTreeAutoNode( const std::shared_ptr<GRID_TIMELINE>& aTimeline, int aLane, size_t aStep,
                         const std::string& aKind, const nlohmann::json& aMetrics, bool aCurrent )
{
    std::lock_guard<std::mutex> lock( g_mutex );
    const int                   id = nodeAt( *aTimeline, aLane, aStep, aKind, aMetrics );

    if( aCurrent )
    {
        VARIANT_NODE* n = findNode( id );
        KOPENAPI_HISTORY& h = KOPENAPI_HISTORY::Get();
        n->boardSeq = h.PendingSeq() > 0 ? h.PendingSeq() : h.LastSeq();
        n->metrics.update( aMetrics );
        setCurrent( id );
    }

    return id;
}


bool VariantTreeNodeAt( int aId, int& aRun, int& aLane, size_t& aStep, std::string& aError )
{
    std::lock_guard<std::mutex> lock( g_mutex );
    const VARIANT_NODE*         n = findNode( aId );

    if( !n )
    {
        aError = "no node " + std::to_string( aId ) + " (pcb_variant_tree list)";
        return false;
    }

    if( !n->run )
    {
        aError = "node " + std::to_string( aId ) + " is a board state, not a place in a run";
        return false;
    }

    aRun = n->run;
    aLane = n->lane;
    aStep = n->step;
    return true;
}


KOPENAPI_REGISTER( "pcb_variant_tree",
                   "EXPERIMENTAL. The tree of places to come back to on the board's routing: nodes made "
                   "automatically at every recorded grid route's end (pcb_route {record: true}; the board "
                   "with its copper), every branch's start and end (pcb_route {from}) and every route stopped "
                   "with job_cancel {keep: true} (its best lane's last step), or on request. A node: run, "
                   "lane (variant) and step of a recorded route, the board's edit-history entry (board_seq; "
                   "board held while that entry is not rolled back), metrics (stage, round, crowded, nets with "
                   "copper; routed / connections / vias / length / route_ms at a route's end). parent: the "
                   "node before it on its lane and the lanes it branched from, else the node its run started "
                   "from (a 'start' node: the board before the run; an 'edit' node: the board edited since a "
                   "node). verb list: nodes, the current one (the board is still that node's) or "
                   "edited_since, the runs kept in memory (timelines: the last runs within budget_mb, least "
                   "recently used out first; a node of an evicted run keeps its metrics, timeline: evicted); "
                   "node {at: \"current\" | {run, lane, step}, name, tags}: a node there (an existing one at the "
                   "same place is returned); rename {id, name}; tag {id, add, remove}; delete {id}: the node "
                   "and the nodes under it (not the current node, not a node tagged locked); budget {mb}: the "
                   "timelines' memory budget (default 256), runs evicted to fit. pcb_route {from: {node}} goes "
                   "on from a node",
                   R"json({"type":"object","properties":{
                        "verb":{"type":"string","enum":["list","node","rename","tag","delete","budget"],"default":"list"},
                        "at":{"description":"node: \"current\" (default) or {run, lane (index or variant; default the shown lane), step (negative from the end)}"},
                        "id":{"type":"integer"},
                        "name":{"type":"string"},
                        "tags":{"type":"array","items":{"type":"string"}},
                        "add":{"type":"array","items":{"type":"string"}},
                        "remove":{"type":"array","items":{"type":"string"}},
                        "mb":{"type":"number","description":"budget: memory for the kept runs' timelines"}}})json"_json,
                   false, h_pcb_variant_tree );
