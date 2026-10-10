/*
 * kicadopenapi placement (ROADMAP task 3.2): pcb_footprint_move, pcb_placement_check,
 * pcb_place_auto.
 *
 * The agent places with pcb_footprint_move (several footprints in one undo step) and judges with
 * pcb_placement_check (courtyard overlaps, parts off the board, connectors away from the edge,
 * ratsnest length).  pcb_place_auto is a starting point: an outline sized from the parts if there
 * is none, connectors along the board edges, the rest by KiCad's own autoplacer (AR_AUTOPLACER:
 * a placement matrix inside the outline, ratsnest cost).
 */
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <autorouter/ar_autoplacer.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <ratsnest/ratsnest_data.h>
#include <footprint.h>
#include <geometry/shape_poly_set.h>
#include <geometry/seg.h>
#include <kicadopenapi_glow_view.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <pad.h>
#include <pcb_edit_frame.h>
#include <pcb_shape.h>
#include <string_utils.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <view/view_overlay.h>

#include <cmath>
#include <map>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


int toIU( double aMm )
{
    return pcbIUScale.mmToIU( aMm );
}


double toMm( double aIU )
{
    return std::round( pcbIUScale.IUTomm( aIU ) * 1000.0 ) / 1000.0;
}


/// Area in mm2 from internal units squared
double areaMm2( double aIU2 )
{
    return aIU2 / ( pcbIUScale.IU_PER_MM * pcbIUScale.IU_PER_MM );
}


/// Board editor glow (no frame headless)
struct PLACE_GLOW_TRAITS
{
    using FRAME = PCB_EDIT_FRAME;

    static FRAME* Frame( KIWAY* aKiway )
    {
        return aKiway ? dynamic_cast<PCB_EDIT_FRAME*>( aKiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;
    }

    static EDA_ITEM* Resolve( FRAME* aFrame, const KIID& aId )
    {
        return aFrame->GetBoard() ? aFrame->GetBoard()->ResolveItem( aId, true ) : nullptr;
    }

    static void Brighten( FRAME* aFrame, EDA_ITEM* aItem, bool aOn )
    {
        KIGFX::VIEW* view = aFrame->GetCanvas()->GetView();

        auto one = [&]( EDA_ITEM* aOne )
        {
            if( aOn )
                aOne->SetBrightened();
            else
                aOne->ClearBrightened();

            view->Update( aOne, KIGFX::REPAINT );
        };

        one( aItem );

        if( aItem->Type() == PCB_FOOTPRINT_T )
            static_cast<FOOTPRINT*>( aItem )->RunOnChildren( [&]( BOARD_ITEM* aChild ) { one( aChild ); }, RECURSE_MODE::RECURSE );
    }

    static BOX2I Box( EDA_ITEM* aItem ) { return aItem->GetBoundingBox(); }
    static int   Mm() { return pcbIUScale.mmToIU( 1.0 ); }
    static void  ItemColour( FRAME*, bool, std::optional<KIGFX::COLOR4D>& ) {}
};


void glow( KOPENAPI_CONTEXT& aCtx, const std::vector<KIID>& aItems )
{
    if( !aCtx.headless )
        KopenapiGlow<PLACE_GLOW_TRAITS>( aCtx.kiway, aItems, aItems.size() > 1 ? 300 : 0 );
}


FOOTPRINT* findFootprint( BOARD* aBoard, const std::string& aRef )
{
    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( str( fp->GetReference() ) == aRef )
            return fp;
    }

    return nullptr;
}


/// Courtyard of a footprint on its side (its body box when it has none)
SHAPE_POLY_SET courtyard( FOOTPRINT* aFootprint )
{
    const PCB_LAYER_ID   layer = aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd;
    const SHAPE_POLY_SET& cy = aFootprint->GetCourtyard( layer );

    if( cy.OutlineCount() > 0 )
        return cy;

    SHAPE_POLY_SET box;
    BOX2I          bbox = aFootprint->GetBoundingBox( false );
    box.NewOutline();
    box.Append( bbox.GetLeft(), bbox.GetTop() );
    box.Append( bbox.GetRight(), bbox.GetTop() );
    box.Append( bbox.GetRight(), bbox.GetBottom() );
    box.Append( bbox.GetLeft(), bbox.GetBottom() );
    return box;
}


bool isConnector( FOOTPRINT* aFootprint )
{
    const std::string ref = str( aFootprint->GetReference() );
    const std::string lib = str( aFootprint->GetFPID().GetLibNickname() );

    return ( !ref.empty() && ( ref[0] == 'J' || ref[0] == 'P' ) && ref.size() > 1 && std::isdigit( (unsigned char) ref[1] ) )
           || lib.rfind( "Connector", 0 ) == 0 || lib.rfind( "TerminalBlock", 0 ) == 0;
}


/// Placement findings over the whole board
nlohmann::json placementReport( BOARD* aBoard )
{
    SHAPE_POLY_SET outline;
    const bool     hasOutline = aBoard->GetBoardPolygonOutlines( outline, true ) && outline.OutlineCount() > 0;
    const BOX2I    edges = aBoard->GetBoardEdgesBoundingBox();

    std::vector<FOOTPRINT*> fps( aBoard->Footprints().begin(), aBoard->Footprints().end() );
    std::vector<SHAPE_POLY_SET> areas;

    for( FOOTPRINT* fp : fps )
        areas.push_back( courtyard( fp ) );

    nlohmann::json overlaps = nlohmann::json::array();

    for( size_t i = 0; i < fps.size(); ++i )
    {
        for( size_t j = i + 1; j < fps.size(); ++j )
        {
            if( fps[i]->IsFlipped() != fps[j]->IsFlipped() )
                continue;

            SHAPE_POLY_SET both = areas[i];
            both.BooleanIntersection( areas[j] );

            if( both.OutlineCount() > 0 && areaMm2( both.Area() ) > 0.01 )
            {
                overlaps.push_back( { { "a", str( fps[i]->GetReference() ) }, { "b", str( fps[j]->GetReference() ) },
                                      { "area_mm2", std::round( areaMm2( both.Area() ) * 100.0 ) / 100.0 } } );
            }
        }
    }

    nlohmann::json outside = nlohmann::json::array();
    nlohmann::json connectors = nlohmann::json::array();

    for( size_t i = 0; i < fps.size(); ++i )
    {
        if( hasOutline )
        {
            SHAPE_POLY_SET out = areas[i];
            out.BooleanSubtract( outline );

            if( out.OutlineCount() > 0 && areaMm2( out.Area() ) > 0.01 )
                outside.push_back( str( fps[i]->GetReference() ) );
        }

        if( isConnector( fps[i] ) && edges.GetWidth() > 0 )
        {
            const BOX2I box = areas[i].BBox();
            const int   gap = std::min( { box.GetLeft() - edges.GetLeft(), edges.GetRight() - box.GetRight(),
                                          box.GetTop() - edges.GetTop(), edges.GetBottom() - box.GetBottom() } );
            connectors.push_back( { { "ref", str( fps[i]->GetReference() ) }, { "to_edge_mm", toMm( gap ) } } );
        }
    }

    // Ratsnest: what is left to route and how long the airwires are (shorter = better placement)
    // Built from scratch: after a netlist update headless the incremental data can be stale
    aBoard->BuildConnectivity();
    std::shared_ptr<CONNECTIVITY_DATA> connectivity = aBoard->GetConnectivity();
    connectivity->RecalculateRatsnest();

    double airwires = 0;
    int    edgesCount = 0;

    for( int net = 1; net < (int) aBoard->GetNetCount(); ++net )
    {
        if( RN_NET* rn = connectivity->GetRatsnestForNet( net ) )
        {
            for( const CN_EDGE& edge : rn->GetEdges() )
            {
                if( edge.GetSourceNode() && edge.GetTargetNode() )
                {
                    airwires += ( edge.GetTargetPos() - edge.GetSourcePos() ).EuclideanNorm();
                    edgesCount++;
                }
            }
        }
    }

    nlohmann::json report = { { "footprints", fps.size() },
                              { "courtyard_overlaps", overlaps },
                              { "outside_board", outside },
                              { "connectors", connectors },
                              { "unrouted", connectivity->GetUnconnectedCount( true ) },
                              { "airwire_length_mm", toMm( airwires ) },
                              { "airwires", edgesCount },
                              { "ok", overlaps.empty() && outside.empty() && hasOutline } };

    if( hasOutline )
        report["board_mm"] = { toMm( edges.GetWidth() ), toMm( edges.GetHeight() ) };
    else
        report["hint"] = "no board outline: pcb_outline_set, or pcb_place_auto sizes one";

    return report;
}


/**
 * The panel edge a connector footprint marks for its front (KiCad's convention: a line on
 * Dwgs.User where the board / panel edge goes, the mating side beyond it): the longest such
 * line, and the direction from the pads towards it (where the connector faces).
 */
std::optional<std::pair<SEG, VECTOR2D>> panelEdge( FOOTPRINT* aFootprint )
{
    std::optional<SEG> best;

    for( BOARD_ITEM* item : aFootprint->GraphicalItems() )
    {
        if( item->Type() != PCB_SHAPE_T || item->GetLayer() != Dwgs_User )
            continue;

        PCB_SHAPE* shape = static_cast<PCB_SHAPE*>( item );

        if( shape->GetShape() != SHAPE_T::SEGMENT )
            continue;

        SEG seg( shape->GetStart(), shape->GetEnd() );

        if( !best || seg.Length() > best->Length() )
            best = seg;
    }

    if( !best || aFootprint->Pads().empty() )
        return std::nullopt;

    VECTOR2D centroid( 0, 0 );

    for( PAD* pad : aFootprint->Pads() )
        centroid += VECTOR2D( pad->GetPosition() );

    centroid = centroid * ( 1.0 / aFootprint->Pads().size() );

    const VECTOR2D d( best->B - best->A );
    VECTOR2D       n( -d.y, d.x );
    n = n.Resize( 1.0 );

    if( ( VECTOR2D( best->Center() ) - centroid ).Dot( n ) < 0 )
        n = -n;

    return std::make_pair( *best, n );
}


void refreshGui( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard )
{
    if( aCtx.headless )
        return;

    if( auto* frame = PLACE_GLOW_TRAITS::Frame( aCtx.kiway ) )
    {
        if( frame->GetBoard() == aBoard )
        {
            frame->GetToolManager()->RunAction( ACTIONS::selectionClear );
            frame->GetCanvas()->Refresh();
        }
    }
}

} // namespace


static KOPENAPI_RESULT h_pcb_footprint_move( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( !aArgs.contains( "moves" ) || !aArgs["moves"].is_array() || aArgs["moves"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'moves': [{ref, x_mm, y_mm, rotation_deg?, side?}]" );

    // Validate everything first: all or nothing
    std::vector<std::pair<FOOTPRINT*, nlohmann::json>> plan;

    for( const nlohmann::json& m : aArgs["moves"] )
    {
        FOOTPRINT* fp = m.is_object() ? findFootprint( board, m.value( "ref", std::string() ) ) : nullptr;

        if( !fp )
            return KOPENAPI_RESULT::Error( 404, "footprint not found: " + m.dump() );

        if( m.contains( "side" ) && m["side"] != "top" && m["side"] != "bottom" )
            return KOPENAPI_RESULT::Error( 400, "side: top or bottom" );

        if( m.contains( "facing" ) && m["facing"] != "left" && m["facing"] != "right" && m["facing"] != "up" && m["facing"] != "down" )
            return KOPENAPI_RESULT::Error( 400, "facing: left, right, up or down" );

        if( fp->IsLocked() && !aArgs.value( "override_locks", false ) )
            return KOPENAPI_RESULT::Error( 409, str( fp->GetReference() ) + " is locked (override_locks: true)" );

        plan.emplace_back( fp, m );
    }

    BOARD_COMMIT      commit( context->GetToolManager() );
    std::vector<KIID> moved;

    for( auto& [fp, m] : plan )
    {
        commit.Modify( fp );

        if( m.contains( "side" ) && ( m["side"] == "bottom" ) != fp->IsFlipped() )
            fp->Flip( fp->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );   // as the editor flips a part

        if( m.contains( "rotation_deg" ) && m["rotation_deg"].is_number() )
            fp->SetOrientation( EDA_ANGLE( m["rotation_deg"].get<double>(), DEGREES_T ) );

        // facing: turn so the front (beyond the footprint's panel-edge line) points that way
        if( m.contains( "facing" ) )
        {
            std::optional<std::pair<SEG, VECTOR2D>> edge = panelEdge( fp );

            if( !edge )
            {
                commit.Revert();
                return KOPENAPI_RESULT::Error( 422, str( fp->GetReference() ) + " has no panel-edge line (Dwgs.User) to face with" );
            }

            static const std::map<std::string, double> want = { { "right", 0 }, { "down", 90 }, { "left", 180 }, { "up", 270 } };
            const double now = std::atan2( edge->second.y, edge->second.x ) * 180.0 / M_PI;
            const double turn = now - want.at( m["facing"].get<std::string>() );   // CCW on screen lowers the angle
            fp->SetOrientation( fp->GetOrientation() + EDA_ANGLE( turn, DEGREES_T ) );
        }

        VECTOR2I pos( m.contains( "x_mm" ) ? toIU( m["x_mm"].get<double>() ) : fp->GetPosition().x,
                      m.contains( "y_mm" ) ? toIU( m["y_mm"].get<double>() ) : fp->GetPosition().y );

        // anchor center: the position names the middle of the courtyard (connectors' origin is
        // often a pin far from their body)
        if( m.value( "anchor", std::string( "origin" ) ) == "center" )
            pos -= courtyard( fp ).BBox().Centre() - fp->GetPosition();
        else if( m.value( "anchor", std::string( "origin" ) ) == "panel_edge" )
        {
            std::optional<std::pair<SEG, VECTOR2D>> edge = panelEdge( fp );

            if( !edge )
            {
                commit.Revert();
                return KOPENAPI_RESULT::Error( 422, str( fp->GetReference() ) + " has no panel-edge line (Dwgs.User) to anchor on" );
            }

            pos -= edge->first.Center() - fp->GetPosition();

            // edge_offset_mm: along the facing direction - positive sticks out past the edge,
            // negative sinks the connector in (case / panel specifics)
            const double offset = m.value( "edge_offset_mm", 0.0 );
            pos += VECTOR2I( KiROUND( edge->second.x * toIU( offset ) ), KiROUND( edge->second.y * toIU( offset ) ) );
        }

        fp->SetPosition( pos );
        fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );   // placed now: connectivity counts it
        moved.push_back( fp->m_Uuid );
    }

    commit.Push( _( "Move footprints (API)" ) );
    refreshGui( aCtx, board );
    glow( aCtx, moved );

    nlohmann::json report = placementReport( board );
    nlohmann::json placed = nlohmann::json::array();

    for( auto& [fp, m] : plan )
    {
        placed.push_back( { { "ref", str( fp->GetReference() ) },
                            { "x_mm", toMm( fp->GetPosition().x ) },
                            { "y_mm", toMm( fp->GetPosition().y ) },
                            { "rotation_deg", fp->GetOrientation().AsDegrees() },
                            { "side", fp->IsFlipped() ? "bottom" : "top" } } );
    }

    report["moved"] = placed;
    return KOPENAPI_RESULT::Ok( report );
}


static KOPENAPI_RESULT h_pcb_placement_check( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    return KOPENAPI_RESULT::Ok( placementReport( context->GetBoard() ) );
}


static KOPENAPI_RESULT h_pcb_place_auto( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*       board = context->GetBoard();
    BOARD_COMMIT commit( context->GetToolManager() );
    nlohmann::json notes = nlohmann::json::array();

    std::vector<FOOTPRINT*> all;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( !fp->IsLocked() )
            all.push_back( fp );
    }

    if( all.empty() )
        return KOPENAPI_RESULT::Error( 409, "no unlocked footprints to place" );

    // 1. The outline: kept, given, or sized from the parts (courtyard area x fill factor)
    BOX2I edges = board->GetBoardEdgesBoundingBox();
    const bool resize = aArgs.contains( "width_mm" ) || edges.GetWidth() == 0 || edges.GetHeight() == 0;

    if( resize )
    {
        double w = aArgs.value( "width_mm", 0.0 ), h = aArgs.value( "height_mm", 0.0 );

        if( w <= 0 || h <= 0 )
        {
            double area = 0;

            for( FOOTPRINT* fp : all )
                area += areaMm2( courtyard( fp ).Area() );

            const double aspect = aArgs.value( "aspect", 1.5 );
            const double fill = aArgs.value( "density", 0.35 );   // share of the board the parts cover
            w = std::ceil( std::sqrt( area / fill * aspect ) );
            h = std::ceil( area / fill / w );
            notes.push_back( "outline sized from the parts: " + std::to_string( (int) w ) + " x " + std::to_string( (int) h )
                             + " mm (courtyards " + std::to_string( (int) std::round( area ) ) + " mm2, density "
                             + std::to_string( fill ).substr( 0, 4 ) + ")" );
        }

        for( BOARD_ITEM* item : std::vector<BOARD_ITEM*>( board->Drawings().begin(), board->Drawings().end() ) )
        {
            if( item->GetLayer() == Edge_Cuts )
                commit.Remove( item );
        }

        const VECTOR2I origin( toIU( aArgs.value( "x_mm", 20.0 ) ), toIU( aArgs.value( "y_mm", 20.0 ) ) );
        auto*          outline = new PCB_SHAPE( board, SHAPE_T::RECTANGLE );
        outline->SetLayer( Edge_Cuts );
        outline->SetStroke( STROKE_PARAMS( toIU( 0.1 ), LINE_STYLE::SOLID ) );
        outline->SetStart( origin );
        outline->SetEnd( origin + VECTOR2I( toIU( w ), toIU( h ) ) );
        commit.Add( outline );
        commit.Push( _( "Board outline (API)" ) );
        commit = BOARD_COMMIT( context->GetToolManager() );
        edges = board->GetBoardEdgesBoundingBox();
    }

    // 2. Connectors along the edges: left, right, top, bottom in turn, courtyard inset 0.5 mm
    std::vector<FOOTPRINT*> rest;
    std::vector<KIID>       placed;
    const int               inset = toIU( 0.5 );
    int                     side = 0;
    std::map<int, int>      along;   // next free offset along each edge

    for( FOOTPRINT* fp : all )
    {
        if( !aArgs.value( "connectors_to_edge", true ) || !isConnector( fp ) )
        {
            rest.push_back( fp );
            continue;
        }

        commit.Modify( fp );
        const BOX2I box = courtyard( fp ).BBox();
        const VECTOR2I toOrigin = fp->GetPosition() - box.GetOrigin();
        const int      s = side++ % 4;
        int&           offset = along[s];
        VECTOR2I       corner;

        if( s == 0 )
            corner = VECTOR2I( edges.GetLeft() + inset, edges.GetTop() + inset + offset );
        else if( s == 1 )
            corner = VECTOR2I( edges.GetRight() - inset - (int) box.GetWidth(), edges.GetTop() + inset + offset );
        else if( s == 2 )
            corner = VECTOR2I( edges.GetLeft() + inset + offset, edges.GetTop() + inset );
        else
            corner = VECTOR2I( edges.GetLeft() + inset + offset, edges.GetBottom() - inset - (int) box.GetHeight() );

        offset += ( s < 2 ? box.GetHeight() : box.GetWidth() ) + toIU( 1.0 );
        fp->SetPosition( corner + toOrigin );
        fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
        placed.push_back( fp->m_Uuid );
    }

    if( !placed.empty() )
    {
        commit.Push( _( "Connectors to the edges (API)" ) );
        commit = BOARD_COMMIT( context->GetToolManager() );
    }

    // 3. The rest: KiCad's autoplacer (matrix inside the outline, ratsnest cost); footprints a
    //    netlist update just added count for connectivity only once placed
    for( FOOTPRINT* fp : rest )
    {
        if( fp->GetAttributes() & FP_JUST_ADDED )
        {
            commit.Modify( fp );
            fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
        }
    }

    board->BuildConnectivity();
    AR_AUTOPLACER autoplacer( board );
    autoplacer.SetOverlay( std::make_shared<KIGFX::VIEW_OVERLAY>() );   // it draws its matrix unconditionally

    const AR_RESULT result = rest.empty() ? AR_COMPLETED : autoplacer.AutoplaceFootprints( rest, &commit, false );

    if( result == AR_COMPLETED )
    {
        if( !rest.empty() )
            commit.Push( _( "Autoplace footprints (API)" ) );
    }
    else
    {
        commit.Revert();
        notes.push_back( "KiCad's autoplacer did not complete" );
    }

    for( FOOTPRINT* fp : rest )
        placed.push_back( fp->m_Uuid );

    refreshGui( aCtx, board );
    glow( aCtx, placed );

    nlohmann::json report = placementReport( board );
    report["notes"] = notes;
    report["autoplacer"] = result == AR_COMPLETED ? "completed" : "failed";
    return KOPENAPI_RESULT::Ok( report );
}


KOPENAPI_REGISTER( "pcb_footprint_move",
                   "Place footprints: several at once in one undo step - position (mm) of the origin, the "
                   "courtyard's middle (anchor center) or a connector's panel-edge line (anchor panel_edge), "
                   "rotation_deg or facing (connectors: mating side towards left / right / up / down), side "
                   "top / bottom; locked ones need override_locks; answers where they are now and the "
                   "placement check (overlaps, outside the board, airwire length)",
                   R"json({"type":"object","required":["moves"],"properties":{
                        "moves":{"type":"array","items":{"type":"object","required":["ref"],"properties":{
                            "ref":{"type":"string"},"x_mm":{"type":"number"},"y_mm":{"type":"number"},
                            "rotation_deg":{"type":"number"},"side":{"type":"string","enum":["top","bottom"]},
                            "anchor":{"type":"string","enum":["origin","center","panel_edge"],"default":"origin","description":"center: x / y are the courtyard's middle; panel_edge: the middle of the connector's panel-edge line (put it on the board edge)"},
                            "edge_offset_mm":{"type":"number","default":0,"description":"anchor panel_edge: shift along the facing direction (+ out past the edge, - recessed)"},
                            "facing":{"type":"string","enum":["left","right","up","down"],"description":"connectors: turn so the mating side (beyond the footprint's panel-edge line on Dwgs.User) faces this way"}}}},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_footprint_move );

KOPENAPI_REGISTER( "pcb_placement_check",
                   "Judge the placement: courtyard overlaps (same side), footprints outside the board, "
                   "connectors' distance to the board edge, unrouted connections and total airwire "
                   "length (shorter = better placement for routing), board size; ok flag",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_placement_check );

KOPENAPI_REGISTER( "pcb_place_auto",
                   "A first placement: board outline kept, given (width_mm / height_mm) or sized from the "
                   "parts (density, aspect); connectors along the edges; the rest by KiCad's autoplacer "
                   "(inside the outline, short airwires); unlocked footprints only; answers the placement "
                   "check. Refine with pcb_footprint_move",
                   R"json({"type":"object","properties":{
                        "width_mm":{"type":"number"},"height_mm":{"type":"number"},
                        "x_mm":{"type":"number","default":20},"y_mm":{"type":"number","default":20},
                        "density":{"type":"number","default":0.35},"aspect":{"type":"number","default":1.5},
                        "connectors_to_edge":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_place_auto, 600 );

KOPENAPI_MARK_EDITING( "pcb_footprint_move", "pcb_place_auto" );
