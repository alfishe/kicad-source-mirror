/// @file methods_pcb_place.cpp
/// @brief kicadopenapi placement (ROADMAP task 3.2): pcb_footprint_move, pcb_placement_check,
/// pcb_place_auto.
///
/// The agent places with pcb_footprint_move (several footprints in one undo step) and judges with
/// pcb_placement_check (courtyard overlaps, parts off the board, connectors away from the edge,
/// ratsnest length).  pcb_place_auto is a starting point: an outline sized from the parts if there
/// is none, connectors along the board edges, the rest by KiCad's own autoplacer (AR_AUTOPLACER:
/// a placement matrix inside the outline, ratsnest cost).
#include "kopenapi_pcb.h"

#include <kicadopenapi_registry.h>
#include <netinfo.h>

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
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
#include <pcb_text.h>
#include <string_utils.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <view/view_overlay.h>

#include <algorithm>
#include <numeric>
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
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


/// @brief Area in mm2 from internal units squared
double areaMm2( double aIU2 )
{
    return aIU2 / ( pcbIUScale.IU_PER_MM * pcbIUScale.IU_PER_MM );
}


/// @brief Board editor glow (no frame headless)
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


/// @brief Courtyard of a footprint on its side (its body box when it has none)
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


struct PANEL_EDGE;
std::optional<PANEL_EDGE> panelEdge( FOOTPRINT* aFootprint, const std::string& aMethod = "auto" );


/// @brief Edge connector facing against the board outline.
/// @return facing (left/right/up/down/angle), the line's distance to the outline edge it faces
/// (mm, + = inside the board), and whether it faces out over that edge
nlohmann::json edgeFacing( FOOTPRINT* aFootprint, const BOX2I& aEdges );


/// @brief Placement findings: courtyard overlaps, parts outside the board, connectors, ratsnest.
/// @param aOnly when given, overlaps / outside / connectors only for these footprints (the ones just
/// moved); the ratsnest figures are always for the whole board
nlohmann::json placementReport( BOARD* aBoard, const std::set<const FOOTPRINT*>* aOnly = nullptr )
{
    SHAPE_POLY_SET outline;
    const bool     hasOutline = aBoard->GetBoardPolygonOutlines( outline, true ) && outline.OutlineCount() > 0;
    const BOX2I    edges = aBoard->GetBoardEdgesBoundingBox();

    std::vector<FOOTPRINT*>     fps( aBoard->Footprints().begin(), aBoard->Footprints().end() );
    std::vector<SHAPE_POLY_SET> areas;
    std::vector<BOX2I>          boxes;

    areas.reserve( fps.size() );
    boxes.reserve( fps.size() );

    for( FOOTPRINT* fp : fps )
    {
        areas.push_back( courtyard( fp ) );
        boxes.push_back( areas.back().BBox() );
    }

    auto wanted = [&]( size_t i ) { return !aOnly || aOnly->count( fps[i] ); };

    // pairs whose boxes meet (only those can overlap), found by a sweep over the left edges and
    // checked in the board's order
    std::vector<size_t> byLeft( fps.size() );
    std::iota( byLeft.begin(), byLeft.end(), 0 );
    std::sort( byLeft.begin(), byLeft.end(),
               [&]( size_t a, size_t b ) { return boxes[a].GetLeft() < boxes[b].GetLeft(); } );

    std::vector<std::pair<size_t, size_t>> pairs;

    for( size_t si = 0; si < byLeft.size(); ++si )
    {
        const size_t i = byLeft[si];

        for( size_t sj = si + 1; sj < byLeft.size() && boxes[byLeft[sj]].GetLeft() <= boxes[i].GetRight(); ++sj )
        {
            const size_t j = byLeft[sj];

            if( fps[i]->IsFlipped() != fps[j]->IsFlipped() || !boxes[i].Intersects( boxes[j] )
                || !( wanted( i ) || wanted( j ) ) )
            {
                continue;
            }

            pairs.emplace_back( std::min( i, j ), std::max( i, j ) );
        }
    }

    std::sort( pairs.begin(), pairs.end() );

    nlohmann::json overlaps = nlohmann::json::array();

    for( const auto& [i, j] : pairs )
    {
        SHAPE_POLY_SET both = areas[i];
        both.BooleanIntersection( areas[j] );

        if( both.OutlineCount() > 0 && areaMm2( both.Area() ) > 0.01 )
        {
            overlaps.push_back( { { "a", str( fps[i]->GetReference() ) }, { "b", str( fps[j]->GetReference() ) },
                                  { "area_mm2", std::round( areaMm2( both.Area() ) * 100.0 ) / 100.0 } } );
        }
    }

    // a courtyard box no outline edge (cut-outs included) enters, with its centre inside the board,
    // lies wholly inside it: nothing to subtract
    std::vector<SEG> outlineEdges;

    for( auto it = outline.CIterateSegmentsWithHoles(); it; it++ )
        outlineEdges.push_back( *it );

    auto surelyInside = [&]( size_t i )
    {
        const BOX2I& box = boxes[i];

        const SEG sides[4] = { SEG( box.GetOrigin(), VECTOR2I( box.GetRight(), box.GetTop() ) ),
                               SEG( VECTOR2I( box.GetRight(), box.GetTop() ), box.GetEnd() ),
                               SEG( box.GetEnd(), VECTOR2I( box.GetLeft(), box.GetBottom() ) ),
                               SEG( VECTOR2I( box.GetLeft(), box.GetBottom() ), box.GetOrigin() ) };

        for( const SEG& seg : outlineEdges )
        {
            BOX2I segBox( seg.A, VECTOR2I( 0, 0 ) );
            segBox.Merge( seg.B );

            if( !box.Intersects( segBox ) )
                continue;

            if( box.Contains( seg.A ) || box.Contains( seg.B ) )
                return false;

            for( const SEG& side : sides )
            {
                if( side.Intersect( seg ) )
                    return false;
            }
        }

        return outline.Contains( box.Centre() );
    };

    nlohmann::json outside = nlohmann::json::array();
    nlohmann::json connectors = nlohmann::json::array();

    for( size_t i = 0; i < fps.size(); ++i )
    {
        if( !wanted( i ) )
            continue;

        if( hasOutline && !surelyInside( i ) )
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
            nlohmann::json row = { { "ref", str( fps[i]->GetReference() ) }, { "to_edge_mm", toMm( gap ) } };
            row.update( edgeFacing( fps[i], edges ) );
            connectors.push_back( row );
        }
    }

    // edge connectors facing out over their edge overhang by design: not "outside"
    nlohmann::json problems = nlohmann::json::array();
    nlohmann::json overhanging = nlohmann::json::array();

    for( const nlohmann::json& c : connectors )
    {
        if( !c.contains( "faces_out" ) )
            continue;

        auto it = std::find( outside.begin(), outside.end(), c["ref"] );

        if( c["faces_out"].get<bool>() && std::abs( c["line_to_edge_mm"].get<double>() ) <= 1.0 )
        {
            if( it != outside.end() )
            {
                outside.erase( it );
                overhanging.push_back( c["ref"] );
            }
        }
        else if( c["faces_out"].get<bool>() )
        {
            problems.push_back( { { "ref", c["ref"] }, { "problem", "connector_not_on_edge" },
                                  { "line_to_edge_mm", c["line_to_edge_mm"] },
                                  { "hint", "pcb_footprint_move {ref, edge}" } } );
        }
        else
        {
            problems.push_back( { { "ref", c["ref"] }, { "problem", "connector_facing_inward" },
                                  { "facing", c["facing"] },
                                  { "hint", "pcb_footprint_move {ref, edge} puts it on an edge facing out" } } );
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
                              { "connector_problems", problems },
                              { "edge_overhang", overhanging },
                              { "unrouted", connectivity->GetUnconnectedCount( true ) },
                              { "airwire_length_mm", toMm( airwires ) },
                              { "airwires", edgesCount },
                              { "ok", overlaps.empty() && outside.empty() && problems.empty() && hasOutline } };

    if( !overhanging.empty() )
        report["edge_overhang_note"] = "connectors placed with their mating side over the board edge on purpose (the "
                                       "opening at the edge, the body's front beyond it, as the footprint draws it): "
                                       "not outside_board; edge_offset_mm in pcb_footprint_move sets it back";

    if( hasOutline )
        report["board_mm"] = { toMm( edges.GetWidth() ), toMm( edges.GetHeight() ) };
    else
        report["hint"] = "no board outline: pcb_outline_set, or pcb_place_auto sizes one";

    return report;
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

/// @brief The panel / board edge line of an edge connector (first) with its outward normal (second)
struct PANEL_EDGE
{
    SEG         first;
    VECTOR2D    second;
    std::string source;       ///< marker convention, or "inferred"
    std::string confidence;   ///< "marked", or high / medium / low for inferred edges
};


/// @brief Front of an unmarked horizontal connector: the side where the courtyard reaches
/// furthest beyond the signal pads (the mating body sticks out of the soldered area).
/// @return the edge flush with the courtyard front, or nullopt for vertical / undecidable parts
std::optional<PANEL_EDGE> inferEdge( FOOTPRINT* aFootprint )
{
    const wxString name = aFootprint->GetFPID().GetLibItemName().wx_str().Lower();

    if( name.Contains( wxS( "vertical" ) ) || name.Contains( wxS( "topentry" ) ) || aFootprint->Pads().empty() )
        return std::nullopt;

    const VECTOR2I   origin = aFootprint->GetPosition();
    const EDA_ANGLE  orient = aFootprint->GetOrientation();
    auto local = [&]( VECTOR2I aPt )
    {
        VECTOR2I p = aPt - origin;
        RotatePoint( p, -orient );
        return p;
    };
    auto world = [&]( VECTOR2I aPt )
    {
        RotatePoint( aPt, orient );
        return aPt + origin;
    };

    BOX2I pads;
    bool  any = false;

    for( PAD* pad : aFootprint->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::NPTH || pad->GetNumber().IsEmpty() )
            continue;

        const VECTOR2I p = local( pad->GetPosition() );

        if( !any )
            pads = BOX2I( p, VECTOR2I( 0, 0 ) ), any = true;
        else
            pads.Merge( p );
    }

    const SHAPE_POLY_SET& crt = aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd );

    if( !any || crt.OutlineCount() == 0 )
        return std::nullopt;

    BOX2I body;
    bool  first = true;

    for( auto it = crt.CIterate(); it; ++it )
    {
        const VECTOR2I p = local( *it );

        if( first )
            body = BOX2I( p, VECTOR2I( 0, 0 ) ), first = false;
        else
            body.Merge( p );
    }

    // reach beyond the pads: right, down, left, up (local frame)
    const double reach[4] = { (double) body.GetRight() - pads.GetRight(), (double) body.GetBottom() - pads.GetBottom(),
                              (double) pads.GetLeft() - body.GetLeft(), (double) pads.GetTop() - body.GetTop() };
    int best = 0, second = -1;

    for( int k = 1; k < 4; ++k )
    {
        if( reach[k] > reach[best] )
            best = k;
    }

    for( int k = 0; k < 4; ++k )
    {
        if( k != best && ( second < 0 || reach[k] > reach[second] ) )
            second = k;
    }

    const double mm = pcbIUScale.IU_PER_MM;

    if( reach[best] < 3.0 * mm )
        return std::nullopt;

    const double ratio = reach[best] / std::max( reach[second], 0.5 * mm );
    std::string  confidence = ratio >= 2.5 ? "high" : ratio >= 1.5 ? "medium" : "low";

    VECTOR2I a, b;
    VECTOR2D n;

    switch( best )
    {
    case 0: a = { body.GetRight(), body.GetTop() };    b = { body.GetRight(), body.GetBottom() }; n = { 1, 0 };  break;
    case 1: a = { body.GetLeft(), body.GetBottom() };  b = { body.GetRight(), body.GetBottom() }; n = { 0, 1 };  break;
    case 2: a = { body.GetLeft(), body.GetTop() };     b = { body.GetLeft(), body.GetBottom() };  n = { -1, 0 }; break;
    default: a = { body.GetLeft(), body.GetTop() };    b = { body.GetRight(), body.GetTop() };    n = { 0, -1 }; break;
    }

    const VECTOR2I tip = a + VECTOR2I( KiROUND( n.x * 1000000 ), KiROUND( n.y * 1000000 ) );
    const VECTOR2D nw = VECTOR2D( world( tip ) - world( a ) ).Resize( 1.0 );

    return PANEL_EDGE{ SEG( world( a ), world( b ) ), nw, "inferred", confidence };
}


/// @brief The panel edge a connector footprint marks for its front, and the direction from the pads
/// towards it (where the connector faces).  Marked by a line on Dwgs.User, or by a "PCB Edge" text
/// next to a line on another layer: longest Dwgs.User segment first, else the one nearest the text.
std::optional<PANEL_EDGE> markedEdge( FOOTPRINT* aFootprint )
{
    std::optional<SEG>      best;
    std::string             source;
    std::vector<VECTOR2I>   marks;
    std::vector<SEG>        others;

    for( BOARD_ITEM* item : aFootprint->GraphicalItems() )
    {
        if( item->Type() == PCB_TEXT_T )
        {
            const wxString text = static_cast<PCB_TEXT*>( item )->GetText().Lower();

            if( text.Contains( wxS( "edge" ) ) && ( text.Contains( wxS( "pcb" ) ) || text.Contains( wxS( "board" ) ) ) )
                marks.push_back( item->GetPosition() );

            continue;
        }

        if( item->Type() != PCB_SHAPE_T || static_cast<PCB_SHAPE*>( item )->GetShape() != SHAPE_T::SEGMENT )
            continue;

        PCB_SHAPE* shape = static_cast<PCB_SHAPE*>( item );
        SEG        seg( shape->GetStart(), shape->GetEnd() );

        if( item->GetLayer() == Dwgs_User )
        {
            if( !best || seg.Length() > best->Length() )
                best = seg, source = "Dwgs.User line";
        }
        else if( !IsCopperLayer( item->GetLayer() ) && seg.Length() >= pcbIUScale.mmToIU( 2.0 ) )
        {
            others.push_back( seg );
        }
    }

    if( !best && !marks.empty() )
    {
        // nearest long segment to the mark; among equally near ones (within 0.5 mm) the longest
        double nearest = std::numeric_limits<double>::max();

        for( const SEG& seg : others )
            nearest = std::min( nearest, (double) seg.Distance( marks.front() ) );

        for( const SEG& seg : others )
        {
            if( seg.Distance( marks.front() ) <= nearest + pcbIUScale.mmToIU( 0.5 )
                && seg.Distance( marks.front() ) <= pcbIUScale.mmToIU( 3.0 )
                && ( !best || seg.Length() > best->Length() ) )
            {
                best = seg, source = "\"PCB Edge\" text";
            }
        }
    }

    if( !best || aFootprint->Pads().empty() )
        return std::nullopt;

    // a panel line spans the body; short Dwgs.User strokes are drawings (arrows, notes)
    const BOX2I body = aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd ).OutlineCount()
                               ? aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd ).BBox()
                               : aFootprint->GetBoundingBox( false );
    const VECTOR2D dir = VECTOR2D( best->B - best->A ).Resize( 1.0 );
    const double   span = std::abs( dir.x ) * body.GetWidth() + std::abs( dir.y ) * body.GetHeight();

    if( best->Length() < 0.5 * span )
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

    return PANEL_EDGE{ *best, n, source, "marked" };
}


/// @param aMethod auto (marker, else inference), marker, infer
std::optional<PANEL_EDGE> panelEdge( FOOTPRINT* aFootprint, const std::string& aMethod )
{
    if( aMethod != "infer" )
    {
        if( std::optional<PANEL_EDGE> marked = markedEdge( aFootprint ) )
            return marked;
    }

    return aMethod == "marker" ? std::nullopt : inferEdge( aFootprint );
}

} // namespace


nlohmann::json KopenapiPanelEdgeJson( FOOTPRINT* aFootprint, const std::string& aMethod )
{
    const std::optional<PANEL_EDGE> edge = panelEdge( aFootprint, aMethod );

    if( !edge )
        return nullptr;

    const auto mmv = []( double aIU ) { return std::round( pcbIUScale.IUTomm( 1 ) * aIU * 1000.0 ) / 1000.0; };
    const VECTOR2D n = edge->second;
    const VECTOR2D c( edge->first.Center() );

    // depth behind the line (into the board) and overhang past it: courtyard, else bounding box
    double         behind = 0, past = 0;
    SHAPE_POLY_SET body = aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd );

    if( body.OutlineCount() == 0 )
    {
        const BOX2I b = aFootprint->GetBoundingBox( false );
        body.NewOutline();
        for( VECTOR2I p : { b.GetOrigin(), VECTOR2I( b.GetRight(), b.GetTop() ), b.GetEnd(), VECTOR2I( b.GetLeft(), b.GetBottom() ) } )
            body.Append( p );
    }

    for( auto it = body.CIterate(); it; ++it )
    {
        const double d = ( VECTOR2D( *it ) - c ).Dot( n );
        behind = std::max( behind, -d );
        past = std::max( past, d );
    }

    const double deg = std::fmod( std::atan2( n.y, n.x ) * 180.0 / M_PI + 360.0, 360.0 );
    nlohmann::json facing = std::round( deg * 10 ) / 10;

    for( const auto& [name, a] : std::vector<std::pair<const char*, double>>{ { "right", 0 }, { "down", 90 }, { "left", 180 }, { "up", 270 }, { "right", 360 } } )
    {
        if( std::abs( deg - a ) < 1.0 )
            facing = name;
    }

    return { { "line_mm", { { mmv( edge->first.A.x ), mmv( edge->first.A.y ) }, { mmv( edge->first.B.x ), mmv( edge->first.B.y ) } } },
             { "facing", facing },
             { "depth_mm", mmv( behind ) },
             { "overhang_mm", mmv( past ) },
             { "source", edge->source },
             { "confidence", edge->confidence } };
}


namespace
{

nlohmann::json edgeFacing( FOOTPRINT* aFootprint, const BOX2I& aEdges )
{
    const std::optional<PANEL_EDGE> edge = panelEdge( aFootprint, "auto" );

    if( !edge )
        return nlohmann::json::object();

    const VECTOR2D n = edge->second;
    const VECTOR2I c = edge->first.Center();

    // the outline edge the connector faces and the line's distance to it (inside = positive)
    double      dist = 0;
    std::string facing;

    if( std::abs( n.x ) >= std::abs( n.y ) )
        facing = n.x > 0 ? "right" : "left", dist = n.x > 0 ? aEdges.GetRight() - c.x : c.x - aEdges.GetLeft();
    else
        facing = n.y > 0 ? "down" : "up", dist = n.y > 0 ? aEdges.GetBottom() - c.y : c.y - aEdges.GetTop();

    // facing out = the faced edge is the nearest outline edge to the line
    const double nearest = std::min( { std::abs( (double) c.x - aEdges.GetLeft() ), std::abs( (double) aEdges.GetRight() - c.x ),
                                       std::abs( (double) c.y - aEdges.GetTop() ), std::abs( (double) aEdges.GetBottom() - c.y ) } );

    return { { "facing", facing },
             { "line_to_edge_mm", std::round( dist / pcbIUScale.IU_PER_MM * 100.0 ) / 100.0 },
             { "faces_out", std::abs( dist ) <= nearest + pcbIUScale.mmToIU( 0.01 ) },
             { "edge_source", edge->source },
             { "edge_confidence", edge->confidence } };
}

} // namespace


namespace
{

struct NEAR_OPTIONS
{
    double                gapMm = 0.25;     ///< between courtyards
    double                maxMm = 25.0;     ///< how far from the target pad to look
    std::optional<double> rotationDeg;      ///< fixed; else the four turns in the target footprint's frame
    std::string           direction = "auto";
    std::string           partPad;          ///< pad number of the part to bring to the target; else by net
    double                hostRingMm = 0;   ///< keep this far from the target footprint's courtyard (fan-out room)
    bool                  gapGiven = false; ///< gap_mm asked for (else a default that may be narrowed)
};

struct NEAR_PLACEMENT
{
    bool        ok = false;
    std::string reason;
    PAD*        partPad = nullptr;
    double      padDistanceMm = 0;
    std::string direction;
};


/// @brief A pad as REF.NUMBER (the first of several pads sharing a number)
PAD* findPadByName( BOARD* aBoard, const std::string& aRefPad )
{
    const size_t dot = aRefPad.rfind( '.' );

    if( dot == std::string::npos )
        return nullptr;

    FOOTPRINT* fp = findFootprint( aBoard, aRefPad.substr( 0, dot ) );

    if( !fp )
        return nullptr;

    for( PAD* pad : fp->Pads() )
    {
        if( str( pad->GetNumber() ) == aRefPad.substr( dot + 1 ) )
            return pad;
    }

    return nullptr;
}


/// @brief Puts aPart (already staged in a commit) so that its pad on the target's net is as close to
/// aTarget as the courtyards of the parts on its side allow: every turn, every direction from the
/// pad, sliding along the neighbour when the straight spot is taken.  The part keeps its place when
/// nothing is free.
NEAR_PLACEMENT placeNear( BOARD* aBoard, FOOTPRINT* aPart, PAD* aTarget, const NEAR_OPTIONS& aOptions )
{
    NEAR_PLACEMENT   result;
    FOOTPRINT*       host = aTarget->GetParentFootprint();
    const VECTOR2I   target = aTarget->GetPosition();
    const EDA_ANGLE  originalAngle = aPart->GetOrientation();
    const VECTOR2I   originalPos = aPart->GetPosition();

    std::vector<wxString> padNumbers;

    for( PAD* pad : aPart->Pads() )
    {
        const bool wanted = aOptions.partPad.empty() ? ( aTarget->GetNetCode() > 0 && pad->GetNetCode() == aTarget->GetNetCode() )
                                                     : str( pad->GetNumber() ) == aOptions.partPad;

        if( wanted && std::find( padNumbers.begin(), padNumbers.end(), pad->GetNumber() ) == padNumbers.end() )
            padNumbers.push_back( pad->GetNumber() );
    }

    if( padNumbers.empty() )
    {
        result.reason = aOptions.partPad.empty() ? "no pad of " + str( aPart->GetReference() ) + " is on the target's net "
                                                           + str( UnescapeString( aTarget->GetNetname() ) ) + " (name one: near_pad)"
                                                 : "no pad " + aOptions.partPad + " on " + str( aPart->GetReference() );
        return result;
    }

    const int reach = toIU( aOptions.maxMm );
    const int gap = toIU( aOptions.gapMm );

    // the parts on the same side within reach
    std::vector<std::pair<SHAPE_POLY_SET, BOX2I>> obstacles;
    BOX2I                                         around( target, VECTOR2I( 0, 0 ) );

    around.Inflate( reach + std::max( aPart->GetBoundingBox( false ).GetWidth(), aPart->GetBoundingBox( false ).GetHeight() ) );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( fp == aPart || fp->IsFlipped() != aPart->IsFlipped() )
            continue;

        SHAPE_POLY_SET area = courtyard( fp );

        // the target's package keeps a ring free for the tracks leaving its pins
        if( fp == host && aOptions.hostRingMm > 0 )
            area.Inflate( toIU( aOptions.hostRingMm ), CORNER_STRATEGY::ALLOW_ACUTE_CORNERS, ARC_HIGH_DEF );

        const BOX2I box = area.BBox();

        if( box.Intersects( around ) )
            obstacles.emplace_back( std::move( area ), box );
    }

    const BOX2I edges = aBoard->GetBoardEdgesBoundingBox();
    const bool  hasEdges = edges.GetWidth() > 0 && edges.GetHeight() > 0;

    // directions: along the target footprint's axes, the side its pad is nearest to first
    const EDA_ANGLE base = host->GetOrientation();
    std::vector<std::pair<std::string, VECTOR2D>> directions;

    if( aOptions.direction == "auto" )
    {
        const BOX2I hostBox = courtyard( host ).BBox();
        const std::vector<std::pair<int, std::pair<std::string, VECTOR2D>>> sides = {
            { std::abs( target.x - hostBox.GetLeft() ), { "left", VECTOR2D( -1, 0 ) } },
            { std::abs( hostBox.GetRight() - target.x ), { "right", VECTOR2D( 1, 0 ) } },
            { std::abs( target.y - hostBox.GetTop() ), { "up", VECTOR2D( 0, -1 ) } },
            { std::abs( hostBox.GetBottom() - target.y ), { "down", VECTOR2D( 0, 1 ) } } };
        std::vector<size_t> order = { 0, 1, 2, 3 };

        std::stable_sort( order.begin(), order.end(), [&]( size_t a, size_t b ) { return sides[a].first < sides[b].first; } );

        for( size_t i : order )
            directions.push_back( sides[i].second );
    }
    else
    {
        static const std::map<std::string, VECTOR2D> named = { { "left", VECTOR2D( -1, 0 ) }, { "right", VECTOR2D( 1, 0 ) },
                                                               { "up", VECTOR2D( 0, -1 ) }, { "down", VECTOR2D( 0, 1 ) } };
        directions.emplace_back( aOptions.direction, named.at( aOptions.direction ) );
    }

    std::vector<EDA_ANGLE> turns;

    if( aOptions.rotationDeg )
    {
        turns.push_back( EDA_ANGLE( *aOptions.rotationDeg, DEGREES_T ) );
    }
    else
    {
        for( int quarter = 0; quarter < 4; quarter++ )
            turns.push_back( base + EDA_ANGLE( 90.0 * quarter, DEGREES_T ) );
    }

    static const double laterals[] = { 0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 3.0, -3.0, 4.0, -4.0, 6.0, -6.0 };
    const int           step = toIU( 0.1 );

    double    best = std::numeric_limits<double>::max();
    EDA_ANGLE bestAngle;
    VECTOR2I  bestPos;

    for( const EDA_ANGLE& turn : turns )
    {
        aPart->SetOrientation( turn );
        aPart->SetPosition( originalPos );

        SHAPE_POLY_SET area = courtyard( aPart );

        if( gap > 0 )
            area.Inflate( gap, CORNER_STRATEGY::ALLOW_ACUTE_CORNERS, ARC_HIGH_DEF );

        const BOX2I areaBox = area.BBox();

        auto free = [&]( const VECTOR2I& aDelta )
        {
            BOX2I box = areaBox;
            box.Move( aDelta );

            if( hasEdges && !edges.Contains( box ) )
                return false;

            for( const auto& [obstacle, obstacleBox] : obstacles )
            {
                if( !obstacleBox.Intersects( box ) )
                    continue;

                SHAPE_POLY_SET moved = area;
                moved.Move( aDelta );
                moved.BooleanIntersection( obstacle );

                if( !moved.IsEmpty() )
                    return false;
            }

            return true;
        };

        for( PAD* pad : aPart->Pads() )
        {
            if( std::find( padNumbers.begin(), padNumbers.end(), pad->GetNumber() ) == padNumbers.end() )
                continue;

            const VECTOR2I padOffset = pad->GetPosition() - originalPos;

            for( const auto& [name, direction] : directions )
            {
                const VECTOR2D across( -direction.y, direction.x );

                for( double lateral : laterals )
                {
                    const double lateralIU = toIU( std::abs( lateral ) ) * ( lateral < 0 ? -1.0 : 1.0 );

                    if( std::abs( lateralIU ) >= best )
                        break;

                    for( int along = 0; along <= reach; along += step )
                    {
                        const double distance = std::hypot( double( along ), lateralIU );

                        if( distance >= best )
                            break;

                        const VECTOR2I padAt = target + VECTOR2I( KiROUND( direction.x * along + across.x * lateralIU ),
                                                                  KiROUND( direction.y * along + across.y * lateralIU ) );
                        const VECTOR2I pos = padAt - padOffset;

                        if( free( pos - originalPos ) )
                        {
                            best = distance;
                            bestAngle = turn;
                            bestPos = pos;
                            result.partPad = pad;
                            result.direction = name;
                            break;
                        }
                    }
                }
            }
        }
    }

    if( best == std::numeric_limits<double>::max() )
    {
        aPart->SetOrientation( originalAngle );
        aPart->SetPosition( originalPos );
        result.partPad = nullptr;
        result.reason = "no free place within " + std::to_string( aOptions.maxMm ).substr( 0, 5 ) + " mm of "
                        + str( host->GetReference() ) + "." + str( aTarget->GetNumber() ) + " on that side of the board";
        return result;
    }

    // the pad pointer is of the last turn tried: take it again after the final turn
    const wxString number = result.partPad->GetNumber();

    aPart->SetOrientation( bestAngle );
    aPart->SetPosition( bestPos );

    for( PAD* pad : aPart->Pads() )
    {
        if( pad->GetNumber() == number
            && ( !result.ok || ( pad->GetPosition() - target ).EuclideanNorm() < ( result.partPad->GetPosition() - target ).EuclideanNorm() ) )
        {
            result.partPad = pad;
            result.ok = true;
        }
    }

    result.padDistanceMm = toMm( ( result.partPad->GetPosition() - target ).EuclideanNorm() );
    return result;
}


nlohmann::json nearJson( const NEAR_PLACEMENT& aPlacement, PAD* aTarget )
{
    return { { "target", str( aTarget->GetParentFootprint()->GetReference() ) + "." + str( aTarget->GetNumber() ) },
             { "net", str( UnescapeString( aTarget->GetNetname() ) ) },
             { "pad", str( aPlacement.partPad->GetNumber() ) },
             { "pad_distance_mm", std::round( aPlacement.padDistanceMm * 100 ) / 100 },
             { "direction", aPlacement.direction } };
}


std::optional<std::string> nearOptions( const nlohmann::json& aFrom, NEAR_OPTIONS& aOptions, double aDefaultGap = 0.25 )
{
    aOptions.gapMm = aFrom.value( "gap_mm", aDefaultGap );
    aOptions.gapGiven = aFrom.contains( "gap_mm" );
    aOptions.hostRingMm = aFrom.value( "fanout_ring_mm", -1.0 );
    aOptions.maxMm = aFrom.value( "max_mm", 25.0 );
    aOptions.direction = aFrom.value( "near_direction", std::string( "auto" ) );
    aOptions.partPad = aFrom.value( "near_pad", std::string() );

    if( aFrom.contains( "rotation_deg" ) && aFrom["rotation_deg"].is_number() )
        aOptions.rotationDeg = aFrom["rotation_deg"].get<double>();

    if( aOptions.direction != "auto" && aOptions.direction != "left" && aOptions.direction != "right"
        && aOptions.direction != "up" && aOptions.direction != "down" )
    {
        return "near_direction: auto, left, right, up or down (from the target pad, on the board)";
    }

    if( aOptions.gapMm < 0 || aOptions.gapMm > 10 || aOptions.maxMm <= 0 || aOptions.maxMm > 200 )
        return "gap_mm 0 … 10, max_mm up to 200";

    return std::nullopt;
}


/// @brief Pads 0.65 mm apart or closer (QFP, QFN, LGA, BGA)
bool finePitch( FOOTPRINT* aFootprint )
{
    for( PAD* a : aFootprint->Pads() )
    {
        for( PAD* b : aFootprint->Pads() )
        {
            if( a != b && ( a->GetPosition() - b->GetPosition() ).EuclideanNorm() <= toIU( 0.65 ) )
                return true;
        }
    }

    return false;
}


/// @brief Places the parts of a functional block around its main part: each next to the pad of the
/// anchor it belongs to.  A part's pad goes to the anchor pad of its most specific shared net (the
/// net with the fewest pads on the board); a part sharing two nets with the anchor (a decoupling
/// capacitor between a supply pin and ground) takes a free supply pad whose ground pad is nearest.
/// Parts are staged in aCommit; aOut gets placed / unplaced.
void placeGroup( BOARD* aBoard, BOARD_COMMIT& aCommit, FOOTPRINT* aAnchor, std::vector<FOOTPRINT*> aParts,
                 const NEAR_OPTIONS& aOptions, bool aOpposite, nlohmann::json& aOut, std::vector<KIID>& aMoved )
{
    std::map<int, int> netPads;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    std::map<int, std::vector<PAD*>> anchorPads;

    for( PAD* pad : aAnchor->Pads() )
    {
        if( pad->GetNetCode() > 0 )
            anchorPads[pad->GetNetCode()].push_back( pad );
    }

    struct MEMBER
    {
        FOOTPRINT*       part;
        std::vector<int> nets;      // shared with the anchor, most specific first
        double           area;
        int              ownNets = 0;   // of them, nets with few pads (not the rails)
    };

    std::vector<MEMBER>     members;
    std::vector<FOOTPRINT*> indirect;   // no net in common with the anchor: placed at a member they share one with
    nlohmann::json          unplaced = nlohmann::json::array();

    for( FOOTPRINT* part : aParts )
    {
        MEMBER member{ part, {}, courtyard( part ).Area() };

        for( PAD* pad : part->Pads() )
        {
            if( anchorPads.count( pad->GetNetCode() )
                && std::find( member.nets.begin(), member.nets.end(), pad->GetNetCode() ) == member.nets.end() )
            {
                member.nets.push_back( pad->GetNetCode() );
            }
        }

        std::stable_sort( member.nets.begin(), member.nets.end(), [&]( int a, int b ) { return netPads[a] < netPads[b]; } );

        for( int net : member.nets )
            member.ownNets += netPads[net] <= 4 ? 1 : 0;

        // one rail in common is no bond with the anchor (everything shares ground): such a part
        // belongs to a member, if to anything; two nets in common is a part across the anchor's pins
        if( member.ownNets == 0 && member.nets.size() < 2 )
        {
            indirect.push_back( part );
            continue;
        }

        members.push_back( member );
    }

    // the most bound first: a crystal on two nets of its own, then parts on one such net, the
    // capacitors on the rails last; among equals the most specific net, then big before small
    std::stable_sort( members.begin(), members.end(),
                      [&]( const MEMBER& a, const MEMBER& b )
                      {
                          if( a.ownNets != b.ownNets )
                              return a.ownNets > b.ownNets;

                          if( netPads[a.nets.front()] != netPads[b.nets.front()] )
                              return netPads[a.nets.front()] < netPads[b.nets.front()];

                          return a.area > b.area;
                      } );

    std::set<PAD*>          taken;
    nlohmann::json          placed = nlohmann::json::array();
    std::vector<FOOTPRINT*> done;

    for( const MEMBER& member : members )
    {
        const int net = member.nets.front();
        PAD*      target = nullptr;
        double    bestScore = std::numeric_limits<double>::max();

        for( PAD* candidate : anchorPads[net] )
        {
            double score = taken.count( candidate ) ? 1e6 : 0;

            if( member.nets.size() > 1 )
            {
                double nearest = std::numeric_limits<double>::max();

                for( PAD* other : anchorPads[member.nets[1]] )
                    nearest = std::min( nearest, double( ( other->GetPosition() - candidate->GetPosition() ).EuclideanNorm() ) );

                score += toMm( nearest );
            }

            if( score < bestScore )
            {
                bestScore = score;
                target = candidate;
            }
        }

        // a small part already placed on that net (a crystal before its load capacitors): next to
        // its pad rather than across it at the anchor's
        PAD* viaPad = nullptr;
        int  netPads = 0;   // a small net only (a crystal's, 2 - 4 pads): never a rail

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
                netPads += pad->GetNetCode() == net;
        }

        for( FOOTPRINT* placedPart : netPads <= 4 ? done : std::vector<FOOTPRINT*>() )
        {
            if( placedPart->Pads().size() > 4 )
                continue;

            for( PAD* pad : placedPart->Pads() )
            {
                if( pad->GetNetCode() == net && !taken.count( pad ) )
                {
                    viaPad = pad;
                    break;
                }
            }

            if( viaPad )
                break;
        }

        if( viaPad )
            target = viaPad;

        taken.insert( target );
        aCommit.Modify( member.part );

        if( ( aAnchor->IsFlipped() != member.part->IsFlipped() ) != aOpposite )
            member.part->Flip( member.part->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );

        NEAR_OPTIONS options = aOptions;
        options.partPad.clear();

        // a fine-pitch anchor (0.65 mm or less) keeps a ring for its fan-out, unless asked otherwise
        if( options.hostRingMm < 0 )
            options.hostRingMm = finePitch( aAnchor ) ? 1.0 : 0.0;

        NEAR_PLACEMENT placement = placeNear( aBoard, member.part, target, options );

        if( !placement.ok && options.hostRingMm > 0 )
        {
            options.hostRingMm = 0;   // no room with the ring: as close as the courtyards allow
            placement = placeNear( aBoard, member.part, target, options );
        }

        if( !placement.ok && !options.gapGiven && options.gapMm > 0.5 )
        {
            options.gapMm = 0.5;      // no room with the default gap: the narrow one
            placement = placeNear( aBoard, member.part, target, options );
        }

        if( !placement.ok )
        {
            unplaced.push_back( { { "ref", str( member.part->GetReference() ) }, { "reason", placement.reason } } );
            continue;
        }

        member.part->SetAttributes( member.part->GetAttributes() & ~FP_JUST_ADDED );
        aMoved.push_back( member.part->m_Uuid );
        done.push_back( member.part );

        nlohmann::json row = nearJson( placement, target );
        row["ref"] = str( member.part->GetReference() );

        if( viaPad )
            row["via"] = str( viaPad->GetParentFootprint()->GetReference() );
        row["x_mm"] = toMm( member.part->GetPosition().x );
        row["y_mm"] = toMm( member.part->GetPosition().y );
        row["rotation_deg"] = member.part->GetOrientation().AsDegrees();
        row["side"] = member.part->IsFlipped() ? "bottom" : "top";
        placed.push_back( row );
    }

    // parts bound to a member, not to the anchor (the second capacitor of a filter, a LED behind its
    // resistor): next to the member's pad on their common net (not a rail), until none is left
    bool progress = true;

    while( progress && !indirect.empty() )
    {
        progress = false;

        for( auto it = indirect.begin(); it != indirect.end(); )
        {
            FOOTPRINT* part = *it;
            PAD*       target = nullptr;

            for( FOOTPRINT* host : done )
            {
                for( PAD* hostPad : host->Pads() )
                {
                    if( hostPad->GetNetCode() <= 0 || netPads[hostPad->GetNetCode()] > 4 )
                        continue;

                    for( PAD* pad : part->Pads() )
                    {
                        if( pad->GetNetCode() == hostPad->GetNetCode()
                            && ( !target || netPads[hostPad->GetNetCode()] < netPads[target->GetNetCode()] ) )
                        {
                            target = hostPad;
                        }
                    }
                }
            }

            if( !target )
            {
                ++it;
                continue;
            }

            aCommit.Modify( part );

            if( ( aAnchor->IsFlipped() != part->IsFlipped() ) != aOpposite )
                part->Flip( part->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );

            NEAR_OPTIONS options = aOptions;
            options.partPad.clear();

            const NEAR_PLACEMENT placement = placeNear( aBoard, part, target, options );

            if( placement.ok )
            {
                part->SetAttributes( part->GetAttributes() & ~FP_JUST_ADDED );
                aMoved.push_back( part->m_Uuid );
                done.push_back( part );

                nlohmann::json row = nearJson( placement, target );
                row["ref"] = str( part->GetReference() );
                row["x_mm"] = toMm( part->GetPosition().x );
                row["y_mm"] = toMm( part->GetPosition().y );
                row["rotation_deg"] = part->GetOrientation().AsDegrees();
                row["side"] = part->IsFlipped() ? "bottom" : "top";
                row["via"] = str( target->GetParentFootprint()->GetReference() );
                placed.push_back( row );
            }
            else
            {
                unplaced.push_back( { { "ref", str( part->GetReference() ) }, { "reason", placement.reason } } );
            }

            it = indirect.erase( it );
            progress = true;
        }
    }

    for( FOOTPRINT* part : indirect )
    {
        unplaced.push_back( { { "ref", str( part->GetReference() ) },
                              { "reason", "shares no net with " + str( aAnchor->GetReference() )
                                                  + " or a placed part of the block (rails aside)" } } );
    }

    aOut["anchor"] = str( aAnchor->GetReference() );
    aOut["placed"] = placed;
    aOut["unplaced"] = unplaced;
}


/// @brief The parts of a group request: named ones, or (none named) every unlocked part with a pad on a
/// small net of the anchor (at most max_net_pads pads on the board: a crystal and its capacitors,
/// pull-ups, a feedback divider - not everything on the rails)
/// @brief Decoupling capacitors of the anchor guessed from the schematic: two-pad C parts across two
/// rails of the anchor, the ones drawn nearest to it on its sheet, as many per supply rail as the
/// anchor has pads on it.  Appended to aParts; aNote says what was taken.
std::optional<KOPENAPI_RESULT> addDecoupling( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, FOOTPRINT* aAnchor,
                                              std::vector<FOOTPRINT*>& aParts, bool aOverrideLocks, nlohmann::json& aNote )
{
    std::optional<KOPENAPI_METHOD> list = KOPENAPI_REGISTRY::Get().Find( "sch_symbol_list" );

    if( !list )
        return KOPENAPI_RESULT::Error( 503, "decoupling: the schematic editor module is not loaded" );

    struct SYMBOL_SPOT
    {
        std::string sheet;
        double      x, y;
    };

    std::map<std::string, SYMBOL_SPOT> drawn;
    nlohmann::json               query = { { "limit", 1000 } };

    while( true )
    {
        KOPENAPI_RESULT page = list->handler( aCtx, query );

        if( page.status != 200 )
        {
            return KOPENAPI_RESULT::Error( page.status == 409 ? 409 : 422,
                                           "decoupling: schematic: the schematic is not open (sch_open), or name the "
                                           "capacitors in parts" );
        }

        for( const nlohmann::json& row : page.body.value( "items", nlohmann::json::array() ) )
        {
            const std::string ref = row.value( "ref", std::string() );

            if( !drawn.count( ref ) )
                drawn[ref] = { row.value( "sheet", std::string() ), row.value( "x_mm", 0.0 ), row.value( "y_mm", 0.0 ) };
        }

        if( !page.body.contains( "next_cursor" ) || page.body["next_cursor"].is_null() )
            break;

        query["cursor"] = page.body["next_cursor"];
    }

    auto home = drawn.find( str( aAnchor->GetReference() ) );

    if( home == drawn.end() )
        return KOPENAPI_RESULT::Error( 422, "decoupling: " + str( aAnchor->GetReference() ) + " is not in the open schematic" );

    // the anchor's rails as the schematic sees them (power / ground nets), ground told apart
    std::set<int>                  rails, grounds;
    std::optional<KOPENAPI_METHOD> subcircuit = KOPENAPI_REGISTRY::Get().Find( "sch_subcircuit" );

    if( subcircuit )
    {
        KOPENAPI_RESULT block = subcircuit->handler( aCtx, { { "ref", str( aAnchor->GetReference() ) } } );

        if( block.status == 200 )
        {
            for( const nlohmann::json& rail : block.body.value( "rails", nlohmann::json::array() ) )
            {
                if( NETINFO_ITEM* net = KopenapiFindNet( aBoard, rail.value( "net", std::string() ) ) )
                {
                    rails.insert( net->GetNetCode() );

                    if( rail.value( "role", std::string() ) == "ground" )
                        grounds.insert( net->GetNetCode() );
                }
            }
        }
    }

    std::map<int, int> netPads, anchorPads;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    for( PAD* pad : aAnchor->Pads() )
    {
        if( pad->GetNetCode() > 0 )
            anchorPads[pad->GetNetCode()]++;
    }

    struct RAIL_CAPACITOR
    {
        FOOTPRINT* part;
        int        supply;      // the rail of the two that is not ground
        double     distance;
    };

    std::vector<RAIL_CAPACITOR> candidates;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        const std::string ref = str( fp->GetReference() );

        if( fp == aAnchor || fp->Pads().size() != 2 || ref.empty() || ref[0] != 'C' || ( fp->IsLocked() && !aOverrideLocks )
            || std::find( aParts.begin(), aParts.end(), fp ) != aParts.end() )
        {
            continue;
        }

        const int a = fp->Pads()[0]->GetNetCode(), b = fp->Pads()[1]->GetNetCode();

        // across two rails of the anchor
        if( a == b || !anchorPads.count( a ) || !anchorPads.count( b ) || !rails.count( a ) || !rails.count( b ) )
            continue;

        auto at = drawn.find( ref );

        if( at == drawn.end() || at->second.sheet != home->second.sheet )
            continue;

        const int supply = grounds.count( a ) && !grounds.count( b ) ? b
                           : grounds.count( b ) && !grounds.count( a ) ? a
                           : netPads[a] <= netPads[b] ? a : b;

        candidates.push_back( { fp, supply,
                                std::hypot( at->second.x - home->second.x, at->second.y - home->second.y ) } );
    }

    std::stable_sort( candidates.begin(), candidates.end(),
                      []( const RAIL_CAPACITOR& x, const RAIL_CAPACITOR& y ) { return x.distance < y.distance; } );

    std::map<int, int> taken;
    nlohmann::json     chosen = nlohmann::json::array();

    for( const RAIL_CAPACITOR& c : candidates )
    {
        if( taken[c.supply] >= anchorPads[c.supply] )
            continue;

        taken[c.supply]++;
        aParts.push_back( c.part );
        chosen.push_back( str( c.part->GetReference() ) );
    }

    aNote = { { "chosen", chosen }, { "candidates", candidates.size() },
              { "by", "capacitors across two rails of the anchor, nearest to it on its schematic sheet, one per supply pad: "
                      "a guess - check it, or name them in parts" } };
    return std::nullopt;
}


std::optional<KOPENAPI_RESULT> groupParts( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, const nlohmann::json& aGroup,
                                           FOOTPRINT*& aAnchor, std::vector<FOOTPRINT*>& aParts, bool aOverrideLocks )
{
    aAnchor = aGroup.is_object() ? findFootprint( aBoard, aGroup.value( "anchor", std::string() ) ) : nullptr;

    if( !aAnchor )
        return KOPENAPI_RESULT::Error( 404, "anchor: the reference of the block's main part" );

    if( aGroup.value( "from_schematic", false ) && !aGroup.contains( "parts" ) )
    {
        std::optional<KOPENAPI_METHOD> subcircuit = KOPENAPI_REGISTRY::Get().Find( "sch_subcircuit" );

        if( !subcircuit )
            return KOPENAPI_RESULT::Error( 503, "from_schematic: the schematic editor module is not loaded" );

        nlohmann::json query = { { "ref", str( aAnchor->GetReference() ) } };

        if( aGroup.contains( "depth" ) )
            query["depth"] = aGroup["depth"];

        KOPENAPI_RESULT block = subcircuit->handler( aCtx, query );

        if( block.status != 200 )
        {
            return KOPENAPI_RESULT::Error( block.status == 409 ? 409 : 422,
                                           "from_schematic: sch_subcircuit failed for " + str( aAnchor->GetReference() ) + " ("
                                                   + block.body.dump().substr( 0, 200 ) + "); open the schematic (sch_open) or name the parts" );
        }

        for( const nlohmann::json& part : block.body.value( "parts", nlohmann::json::array() ) )
        {
            if( part.value( "kind", std::string() ) != "member" )
                continue;

            FOOTPRINT* fp = findFootprint( aBoard, part.value( "ref", std::string() ) );

            if( fp && fp != aAnchor && ( !fp->IsLocked() || aOverrideLocks ) && !isConnector( fp ) )
                aParts.push_back( fp );
        }

        return std::nullopt;
    }

    if( aGroup.contains( "parts" ) )
    {
        if( !aGroup["parts"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "parts: references" );

        for( const nlohmann::json& ref : aGroup["parts"] )
        {
            FOOTPRINT* part = ref.is_string() ? findFootprint( aBoard, ref.get<std::string>() ) : nullptr;

            if( !part || part == aAnchor )
                return KOPENAPI_RESULT::Error( 404, "parts: no such footprint (or the anchor itself): " + ref.dump() );

            if( part->IsLocked() && !aOverrideLocks )
                return KOPENAPI_RESULT::Error( 409, str( part->GetReference() ) + " is locked (override_locks: true)" );

            aParts.push_back( part );
        }

        return std::nullopt;
    }

    const int          limit = std::clamp( aGroup.value( "max_net_pads", 4 ), 2, 64 );
    std::map<int, int> netPads;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    std::set<int> small;

    for( PAD* pad : aAnchor->Pads() )
    {
        if( pad->GetNetCode() > 0 && netPads[pad->GetNetCode()] <= limit )
            small.insert( pad->GetNetCode() );
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        // connectors belong to the board edge, not to a block
        if( fp == aAnchor || fp->IsLocked() || isConnector( fp ) )
            continue;

        for( PAD* pad : fp->Pads() )
        {
            if( small.count( pad->GetNetCode() ) )
            {
                aParts.push_back( fp );
                break;
            }
        }
    }

    return std::nullopt;
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

    const std::string reportScope = aArgs.value( "report", std::string( "moved" ) );

    if( reportScope != "moved" && reportScope != "full" && reportScope != "none" )
        return KOPENAPI_RESULT::Error( 400, "report: moved, full or none" );

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

        if( m.contains( "with" ) )
        {
            if( !m["with"].is_array() )
                return KOPENAPI_RESULT::Error( 400, "with: references of the parts that keep their place relative to this one" );

            for( const nlohmann::json& ref : m["with"] )
            {
                FOOTPRINT* carried = ref.is_string() ? findFootprint( board, ref.get<std::string>() ) : nullptr;

                if( !carried || carried == fp )
                    return KOPENAPI_RESULT::Error( 404, "with: no such footprint (or the moved one itself): " + ref.dump() );

                if( carried->IsLocked() && !aArgs.value( "override_locks", false ) )
                    return KOPENAPI_RESULT::Error( 409, str( carried->GetReference() ) + " is locked (override_locks: true)" );

                for( const nlohmann::json& other : aArgs["moves"] )
                {
                    if( other.is_object() && other.value( "ref", std::string() ) == ref.get<std::string>() )
                        return KOPENAPI_RESULT::Error( 400, ref.get<std::string>() + " is carried (with) and moved on its own: one of the two" );
                }
            }

            if( m.contains( "side" ) && ( m["side"] == "bottom" ) != fp->IsFlipped() )
                return KOPENAPI_RESULT::Error( 400, "with: not together with a change of side (move the parts, then place the block again)" );
        }

        nlohmann::json move = m;

        if( m.contains( "near" ) )
        {
            PAD*         target = m["near"].is_string() ? findPadByName( board, m["near"].get<std::string>() ) : nullptr;
            NEAR_OPTIONS options;

            if( !target )
                return KOPENAPI_RESULT::Error( 404, "near: a pad as REF.PAD (pcb_footprint_get lists them): " + m["near"].dump() );

            if( target->GetParentFootprint() == fp )
                return KOPENAPI_RESULT::Error( 422, "near: a pad of another footprint" );

            if( std::optional<std::string> wrong = nearOptions( m, options ) )
                return KOPENAPI_RESULT::Error( 400, *wrong );

            options.hostRingMm = std::max( 0.0, options.hostRingMm );

            if( m.contains( "x_mm" ) || m.contains( "y_mm" ) || m.contains( "edge" ) )
                return KOPENAPI_RESULT::Error( 400, "near places the part itself: leave out x_mm / y_mm / edge" );
        }

        // edge: on the board outline, facing out; the coordinate along the edge stays the caller's
        if( m.contains( "edge" ) )
        {
            static const std::map<std::string, std::string> facing = { { "left", "left" }, { "right", "right" },
                                                                        { "top", "up" }, { "bottom", "down" } };

            if( !m["edge"].is_string() || !facing.count( m["edge"].get<std::string>() ) )
                return KOPENAPI_RESULT::Error( 400, "edge: left, right, top or bottom" );

            const BOX2I outline = board->GetBoardEdgesBoundingBox();

            if( outline.GetWidth() <= 0 )
                return KOPENAPI_RESULT::Error( 409, "edge needs a board outline (pcb_outline_set)" );

            const std::string e = m["edge"];
            move["facing"] = facing.at( e );
            move["anchor"] = "panel_edge";

            if( e == "left" || e == "right" )
                move["x_mm"] = toMm( e == "left" ? outline.GetLeft() : outline.GetRight() );
            else
                move["y_mm"] = toMm( e == "top" ? outline.GetTop() : outline.GetBottom() );
        }

        if( ( move.contains( "facing" ) || move.value( "anchor", std::string() ) == "panel_edge" ) && !panelEdge( fp ) )
        {
            return KOPENAPI_RESULT::Error( 409, str( fp->GetReference() ) + " (" + str( fp->GetFPID().Format() )
                                                        + "): no panel edge known - neither a marker in the footprint nor "
                                                          "an inferable front; place it with rotation_deg and check a render" );
        }

        plan.emplace_back( fp, move );
    }

    BOARD_COMMIT      commit( context->GetToolManager() );
    std::vector<KIID> moved;
    std::map<FOOTPRINT*, nlohmann::json> nearPlaced;

    // a flip or rotation can hide the front the plan found: nothing is moved then
    auto noEdgeAfterTurn = [&]( FOOTPRINT* aFp )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Error( 409, str( aFp->GetReference() ) + ": no panel edge known after the flip / "
                                                                         "rotation; place it with rotation_deg" );
    };

    // parts carried with a moved one: the same turn about its origin and the same shift
    std::map<FOOTPRINT*, nlohmann::json> carriedWith;

    auto carry = [&]( FOOTPRINT* aFp, const nlohmann::json& aMove, const VECTOR2I& aFrom, const EDA_ANGLE& aAngleBefore )
    {
        if( !aMove.contains( "with" ) )
            return;

        const EDA_ANGLE turn = aFp->GetOrientation() - aAngleBefore;
        const VECTOR2I  shift = aFp->GetPosition() - aFrom;
        nlohmann::json  rows = nlohmann::json::array();

        for( const nlohmann::json& ref : aMove["with"] )
        {
            FOOTPRINT* carried = findFootprint( board, ref.get<std::string>() );

            commit.Modify( carried );

            if( !turn.IsZero() )
                carried->Rotate( aFrom, turn );

            carried->Move( shift );
            carried->SetAttributes( carried->GetAttributes() & ~FP_JUST_ADDED );
            moved.push_back( carried->m_Uuid );
            rows.push_back( { { "ref", str( carried->GetReference() ) }, { "x_mm", toMm( carried->GetPosition().x ) },
                              { "y_mm", toMm( carried->GetPosition().y ) },
                              { "rotation_deg", carried->GetOrientation().AsDegrees() } } );
        }

        carriedWith[aFp] = rows;
    };

    for( auto& [fp, m] : plan )
    {
        commit.Modify( fp );

        const VECTOR2I  posBefore = fp->GetPosition();
        const EDA_ANGLE angleBefore = fp->GetOrientation();

        if( m.contains( "side" ) && ( m["side"] == "bottom" ) != fp->IsFlipped() )
            fp->Flip( fp->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );   // as the editor flips a part

        if( m.contains( "rotation_deg" ) && m["rotation_deg"].is_number() )
            fp->SetOrientation( EDA_ANGLE( m["rotation_deg"].get<double>(), DEGREES_T ) );

        // facing: turn so the front (beyond the footprint's panel-edge line) points that way
        if( m.contains( "facing" ) )
        {
            std::optional<PANEL_EDGE> edge = panelEdge( fp );

            if( !edge )
                return noEdgeAfterTurn( fp );

            static const std::map<std::string, double> want = { { "right", 0 }, { "down", 90 }, { "left", 180 }, { "up", 270 } };
            const double now = std::atan2( edge->second.y, edge->second.x ) * 180.0 / M_PI;
            const double turn = now - want.at( m["facing"].get<std::string>() );   // CCW on screen lowers the angle
            fp->SetOrientation( fp->GetOrientation() + EDA_ANGLE( turn, DEGREES_T ) );
        }

        // near: next to a pad of another part, as close as the courtyards allow; all or nothing
        if( m.contains( "near" ) )
        {
            PAD*         target = findPadByName( board, m["near"].get<std::string>() );
            NEAR_OPTIONS options;

            nearOptions( m, options );
            options.hostRingMm = std::max( 0.0, options.hostRingMm );

            const NEAR_PLACEMENT placement = placeNear( board, fp, target, options );

            if( !placement.ok )
            {
                commit.Revert();
                return KOPENAPI_RESULT::Error( 422, str( fp->GetReference() ) + ": " + placement.reason + "; nothing was moved" );
            }

            nearPlaced[fp] = nearJson( placement, target );
            fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
            moved.push_back( fp->m_Uuid );
            carry( fp, m, posBefore, angleBefore );
            continue;
        }

        VECTOR2I pos( m.contains( "x_mm" ) ? toIU( m["x_mm"].get<double>() ) : fp->GetPosition().x,
                      m.contains( "y_mm" ) ? toIU( m["y_mm"].get<double>() ) : fp->GetPosition().y );

        // anchor center: the position names the middle of the courtyard (connectors' origin is
        // often a pin far from their body)
        if( m.value( "anchor", std::string( "origin" ) ) == "center" )
            pos -= courtyard( fp ).BBox().Centre() - fp->GetPosition();
        else if( m.value( "anchor", std::string( "origin" ) ) == "panel_edge" )
        {
            std::optional<PANEL_EDGE> edge = panelEdge( fp );

            if( !edge )
                return noEdgeAfterTurn( fp );

            pos -= edge->first.Center() - fp->GetPosition();

            // edge_offset_mm: along the facing direction, + out past the edge, - recessed
            const double offset = m.value( "edge_offset_mm", 0.0 );
            pos += VECTOR2I( KiROUND( edge->second.x * toIU( offset ) ), KiROUND( edge->second.y * toIU( offset ) ) );
        }

        fp->SetPosition( pos );
        fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );   // placed now: connectivity counts it
        moved.push_back( fp->m_Uuid );
        carry( fp, m, posBefore, angleBefore );
    }

    commit.Push( _( "Move footprints (API)" ) );
    refreshGui( aCtx, board );
    glow( aCtx, moved );

    // the placement check for the footprints just moved (the whole board: report full; none: skip)
    nlohmann::json report = nlohmann::json::object();

    if( reportScope == "moved" )
    {
        std::set<const FOOTPRINT*> movedFps;

        for( auto& [fp, m] : plan )
        {
            movedFps.insert( fp );

            for( const nlohmann::json& ref : m.value( "with", nlohmann::json::array() ) )
                movedFps.insert( findFootprint( board, ref.get<std::string>() ) );
        }

        report = placementReport( board, &movedFps );
    }
    else if( reportScope == "full" )
    {
        report = placementReport( board );
    }

    report["report"] = reportScope;
    nlohmann::json placed = nlohmann::json::array();

    for( auto& [fp, m] : plan )
    {
        nlohmann::json row = { { "ref", str( fp->GetReference() ) },
                               { "x_mm", toMm( fp->GetPosition().x ) },
                               { "y_mm", toMm( fp->GetPosition().y ) },
                               { "rotation_deg", fp->GetOrientation().AsDegrees() },
                               { "side", fp->IsFlipped() ? "bottom" : "top" } };

        if( nearPlaced.count( fp ) )
            row["near"] = nearPlaced[fp];

        if( carriedWith.count( fp ) )
            row["with"] = carriedWith[fp];

        if( m.contains( "facing" ) )
        {
            const nlohmann::json edge = KopenapiPanelEdgeJson( fp );
            row["panel_edge"] = { { "source", edge["source"] }, { "confidence", edge["confidence"] } };

            if( edge["confidence"] == "low" || edge["confidence"] == "medium" )
                row["check"] = "front inferred from the body (" + edge["confidence"].get<std::string>()
                               + " confidence): verify with pcb_render_3d";
        }

        placed.push_back( row );
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
        for( BOARD_ITEM* item : board->Drawings() )
        {
            if( item->GetLayer() == Edge_Cuts && item->IsLocked() )
                return KOPENAPI_RESULT::Error( 409, "the board outline is locked: leave out width_mm / height_mm, or pcb_unlock it" );
        }

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

        // a frameless board's page follows its outline (as pcb_outline_set does)
        if( board->GetPageSettings().GetType() == PAGE_SIZE_TYPE::User )
            KopenapiFitPage( board, edges );
    }

    // 2. Connectors along the edges: left, right, top, bottom in turn, courtyard inset 0.5 mm; a
    //    connector whose mating side is known turns to face out over its edge
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
        const int s = side++ % 4;

        if( std::optional<PANEL_EDGE> edge = panelEdge( fp ) )
        {
            static const double want[4] = { 180, 0, 270, 90 };   // left, right, up, down: facing out
            const double        now = std::atan2( edge->second.y, edge->second.x ) * 180.0 / M_PI;
            fp->SetOrientation( fp->GetOrientation() + EDA_ANGLE( now - want[s], DEGREES_T ) );
        }

        const BOX2I    box = courtyard( fp ).BBox();
        const VECTOR2I toOrigin = fp->GetPosition() - box.GetOrigin();
        int&           offset = along[s];
        VECTOR2I       corner;

        // on the top / bottom edges the connectors start from the middle outwards
        const int middleX = edges.Centre().x - (int) box.GetWidth() / 2;

        if( s == 0 )
            corner = VECTOR2I( edges.GetLeft() + inset, edges.Centre().y - (int) box.GetHeight() / 2 + offset );
        else if( s == 1 )
            corner = VECTOR2I( edges.GetRight() - inset - (int) box.GetWidth(), edges.Centre().y - (int) box.GetHeight() / 2 + offset );
        else if( s == 2 )
            corner = VECTOR2I( middleX + offset, edges.GetTop() + inset );
        else
            corner = VECTOR2I( middleX + offset, edges.GetBottom() - inset - (int) box.GetHeight() );

        offset += ( s < 2 ? box.GetHeight() : box.GetWidth() ) + toIU( 1.0 );
        fp->SetPosition( corner + toOrigin );

        // a connector with a known front: its panel line on the edge (the body inside, the mating face out)
        if( std::optional<PANEL_EDGE> edge = panelEdge( fp ) )
        {
            const VECTOR2I line = edge->first.Center();
            VECTOR2I       shift( 0, 0 );

            if( s == 0 )
                shift.x = edges.GetLeft() - line.x;
            else if( s == 1 )
                shift.x = edges.GetRight() - line.x;
            else if( s == 2 )
                shift.y = edges.GetTop() - line.y;
            else
                shift.y = edges.GetBottom() - line.y;

            fp->Move( shift );
        }

        fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
        placed.push_back( fp->m_Uuid );
    }

    if( !placed.empty() )
    {
        commit.Push( _( "Connectors to the edges (API)" ) );
        commit = BOARD_COMMIT( context->GetToolManager() );
    }

    // 2b. The main parts of the given groups spread over the board (largest nearest the middle),
    //     their parts left to the groups below; the autoplacer gets the others
    std::set<FOOTPRINT*> anchors, members;

    for( const nlohmann::json& group : aArgs.value( "groups", nlohmann::json::array() ) )
    {
        FOOTPRINT*              anchor = nullptr;
        std::vector<FOOTPRINT*> parts;

        if( groupParts( aCtx, board, group, anchor, parts, false ) )
            continue;

        if( !isConnector( anchor ) && !anchor->IsLocked() )
            anchors.insert( anchor );

        for( FOOTPRINT* part : parts )
            members.insert( part );
    }

    if( !anchors.empty() )
    {
        std::vector<FOOTPRINT*> order( anchors.begin(), anchors.end() );

        std::sort( order.begin(), order.end(),
                   [&]( FOOTPRINT* x, FOOTPRINT* y )
                   {
                       const double ax = courtyard( x ).Area(), ay = courtyard( y ).Area();
                       return ax != ay ? ax > ay : str( x->GetReference() ) < str( y->GetReference() );
                   } );

        BOX2I inner = edges;
        inner.Inflate( -toIU( 3.0 ) );

        // the main parts stay out of the strips the connectors took along the edges
        int left = inner.GetLeft(), right = inner.GetRight(), top = inner.GetTop(), bottom = inner.GetBottom();

        for( FOOTPRINT* fp : all )
        {
            if( !isConnector( fp ) || !aArgs.value( "connectors_to_edge", true ) )
                continue;

            const BOX2I c = courtyard( fp ).BBox();
            const int   room = toIU( 2.0 ), near = toIU( 1.0 );

            if( c.GetLeft() <= edges.GetLeft() + near )
                left = std::max( left, c.GetRight() + room );
            else if( c.GetRight() >= edges.GetRight() - near )
                right = std::min( right, c.GetLeft() - room );
            else if( c.GetTop() <= edges.GetTop() + near )
                top = std::max( top, c.GetBottom() + room );
            else if( c.GetBottom() >= edges.GetBottom() - near )
                bottom = std::min( bottom, c.GetTop() - room );
        }

        // a board too small for the strips keeps the plain inner box
        if( right - left > toIU( 5.0 ) && bottom - top > toIU( 5.0 ) )
        {
            inner.SetOrigin( left, top );
            inner.SetEnd( right, bottom );
        }

        const int cols = std::max( 1, (int) std::ceil( std::sqrt( order.size() * double( inner.GetWidth() ) / std::max<double>( 1.0, inner.GetHeight() ) ) ) );
        const int rows = std::max( 1, (int) std::ceil( order.size() / double( cols ) ) );

        std::vector<VECTOR2I> cells;

        for( int r = 0; r < rows; ++r )
        {
            for( int c = 0; c < cols; ++c )
                cells.emplace_back( inner.GetLeft() + (int) ( ( c + 0.5 ) * inner.GetWidth() / cols ),
                                    inner.GetTop() + (int) ( ( r + 0.5 ) * inner.GetHeight() / rows ) );
        }

        std::stable_sort( cells.begin(), cells.end(),
                          [&]( const VECTOR2I& x, const VECTOR2I& y )
                          { return ( x - inner.Centre() ).EuclideanNorm() < ( y - inner.Centre() ).EuclideanNorm(); } );

        for( size_t i = 0; i < order.size(); ++i )
        {
            FOOTPRINT* fp = order[i];
            commit.Modify( fp );
            fp->Move( cells[i] - courtyard( fp ).BBox().Centre() );
            fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
            placed.push_back( fp->m_Uuid );
        }

        commit.Push( _( "Main parts spread (API)" ) );
        commit = BOARD_COMMIT( context->GetToolManager() );

        rest.erase( std::remove_if( rest.begin(), rest.end(),
                                    [&]( FOOTPRINT* fp ) { return anchors.count( fp ) || members.count( fp ); } ),
                    rest.end() );
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
    KOPENAPI_KEEPALIVE_REPORTER reporter;
    autoplacer.SetProgressReporter( &reporter );
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

    // The autoplacer keeps pads inside the outline, not courtyards: pull back what pokes out
    {
        BOARD_COMMIT pull( context->GetToolManager() );
        int          pulled = 0;
        BOX2I        inner = edges;

        inner.Inflate( -toIU( 0.25 ) );

        for( FOOTPRINT* fp : rest )
        {
            const BOX2I box = courtyard( fp ).BBox();
            VECTOR2I    shift( 0, 0 );

            if( box.GetWidth() > inner.GetWidth() || box.GetHeight() > inner.GetHeight() )
                continue;

            if( box.GetLeft() < inner.GetLeft() )
                shift.x = inner.GetLeft() - box.GetLeft();
            else if( box.GetRight() > inner.GetRight() )
                shift.x = inner.GetRight() - box.GetRight();

            if( box.GetTop() < inner.GetTop() )
                shift.y = inner.GetTop() - box.GetTop();
            else if( box.GetBottom() > inner.GetBottom() )
                shift.y = inner.GetBottom() - box.GetBottom();

            if( shift.x || shift.y )
            {
                pull.Modify( fp );
                fp->Move( shift );
                pulled++;
            }
        }

        if( pulled )
        {
            pull.Push( _( "Footprints inside the outline (API)" ) );
            notes.push_back( std::to_string( pulled ) + " footprint(s) moved back inside the outline (courtyard)" );
        }
    }

    // 4. Functional blocks: their parts go next to the pads of their main part
    nlohmann::json groups = nlohmann::json::array();

    for( const nlohmann::json& group : aArgs.value( "groups", nlohmann::json::array() ) )
    {
        FOOTPRINT*              anchor = nullptr;
        std::vector<FOOTPRINT*> parts;
        NEAR_OPTIONS            options;

        if( groupParts( aCtx, board, group, anchor, parts, false ) || nearOptions( group, options, 1.0 ) )
        {
            notes.push_back( "group skipped (anchor / parts not found or locked): " + group.dump() );
            continue;
        }

        BOARD_COMMIT   groupCommit( context->GetToolManager() );
        nlohmann::json row = nlohmann::json::object();

        placeGroup( board, groupCommit, anchor, parts, options, group.value( "side", std::string( "same" ) ) == "opposite", row,
                    placed );
        groupCommit.Push( _( "Place group (API)" ) );
        groups.push_back( row );
    }

    refreshGui( aCtx, board );
    glow( aCtx, placed );

    nlohmann::json report = placementReport( board );
    report["notes"] = notes;

    if( !groups.empty() )
        report["groups"] = groups;
    report["autoplacer"] = result == AR_COMPLETED ? "completed" : "failed";
    return KOPENAPI_RESULT::Ok( report );
}


static KOPENAPI_RESULT h_pcb_place_group( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                  board = context->GetBoard();
    FOOTPRINT*              anchor = nullptr;
    std::vector<FOOTPRINT*> parts;
    NEAR_OPTIONS            options;

    if( std::optional<KOPENAPI_RESULT> error = groupParts( aCtx, board, aArgs, anchor, parts, aArgs.value( "override_locks", false ) ) )
        return *error;

    if( std::optional<std::string> wrong = nearOptions( aArgs, options, 1.0 ) )
        return KOPENAPI_RESULT::Error( 400, *wrong );

    nlohmann::json decoupling;

    if( aArgs.contains( "decoupling" ) )
    {
        if( aArgs["decoupling"] != "schematic" )
            return KOPENAPI_RESULT::Error( 400, "decoupling: \"schematic\" (guess the anchor's rail capacitors from the open schematic)" );

        if( std::optional<KOPENAPI_RESULT> error = addDecoupling( aCtx, board, anchor, parts, aArgs.value( "override_locks", false ),
                                                                  decoupling ) )
        {
            return *error;
        }
    }

    const std::string side = aArgs.value( "side", std::string( "same" ) );

    if( side != "same" && side != "opposite" )
        return KOPENAPI_RESULT::Error( 400, "side: same (as the anchor) or opposite (under it, on the other side of the board)" );

    if( parts.empty() )
        return KOPENAPI_RESULT::Error( 422, "no parts: none named, none found in the schematic block and none on a small net of "
                                                    + str( anchor->GetReference() ) + " (name them: parts)" );

    BOARD_COMMIT      commit( context->GetToolManager() );
    std::vector<KIID> moved;
    nlohmann::json    out = nlohmann::json::object();

    placeGroup( board, commit, anchor, parts, options, side == "opposite", out, moved );

    if( !decoupling.is_null() )
        out["decoupling"] = decoupling;

    if( moved.empty() )
    {
        commit.Revert();
        out["applied"] = false;
        return KOPENAPI_RESULT::Ok( out );
    }

    if( aArgs.value( "dry_run", false ) )
    {
        commit.Revert();
        out["applied"] = false;
        return KOPENAPI_RESULT::Ok( out );
    }

    commit.Push( _( "Place group (API)" ) );
    refreshGui( aCtx, board );
    glow( aCtx, moved );

    std::set<const FOOTPRINT*> movedFps;

    for( FOOTPRINT* part : parts )
        movedFps.insert( part );

    out["applied"] = true;
    out["check"] = placementReport( board, &movedFps );
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_place_group",
                   "Place the parts of a functional block around its main part (anchor), each next to the "
                   "anchor pad it belongs to and turned so its own pad faces it: a crystal and its "
                   "capacitors at the oscillator pins, pull-ups at their pins, a regulator's parts, and "
                   "decoupling capacitors - each at its own supply pin (a part between two nets of the "
                   "anchor takes a free supply pad with the nearest ground pad). parts: the references "
                   "(e.g. from sch_subcircuit); without it, every unlocked part (connectors aside) on a small "
                   "net of the anchor (max_net_pads); from_schematic: the members of the anchor's block in the "
                   "open schematic. A part bound to a member rather than to the anchor goes next to that "
                   "member. decoupling: \"schematic\" adds the rail capacitors drawn nearest to the anchor (a guess, "
                   "listed in the answer). side opposite: on the other side of the board, under the pads. "
                   "One undo step; answers per part the target pad, pad distance, position, and the "
                   "placement check",
                   R"json({"type":"object","required":["anchor"],"properties":{
                        "anchor":{"type":"string","description":"reference of the main part; it does not move"},
                        "parts":{"type":"array","items":{"type":"string"}},
                        "from_schematic":{"type":"boolean","default":false,"description":"without parts: take the block's members from the open schematic (sch_subcircuit of the anchor)"},
                        "depth":{"type":"integer","description":"from_schematic: how far sch_subcircuit grows (default 2)"},
                        "decoupling":{"type":"string","enum":["schematic"],"description":"add the anchor's decoupling capacitors guessed from the open schematic: C parts across two of its rails, nearest to it on its sheet, one per supply pad (the answer's decoupling.chosen lists them)"},
                        "max_net_pads":{"type":"integer","default":4,"description":"without parts: nets of the anchor with at most this many pads on the board select the parts"},
                        "side":{"type":"string","enum":["same","opposite"],"default":"same"},
                        "gap_mm":{"type":"number","default":1.0,"description":"between courtyards (room for tracks between the parts; TC-13 2 layers: 0.5 mm routed 68 of 81, 1.0 mm 73); by default a part that finds no place at 1.0 mm takes 0.5"},
                        "fanout_ring_mm":{"type":"number","description":"keep this far from the anchor's courtyard (room for the tracks leaving its pins); default 1.0 for a fine-pitch anchor (pads 0.65 mm apart or closer), else 0; parts that find no place with it come as close as the courtyards allow"},
                        "max_mm":{"type":"number","default":25,"description":"how far from the anchor's pad a part may end up"},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_place_group, 120 );

KOPENAPI_REGISTER( "pcb_footprint_move",
                   "Place footprints: several at once in one undo step - position (mm) of the origin, the "
                   "courtyard's middle (anchor center) or a connector's panel-edge line (anchor panel_edge), "
                   "rotation_deg or facing (connectors: mating side towards left / right / up / down), side "
                   "top / bottom; or near: \"U1.7\" - next to that pad of another part, as close as the "
                   "courtyards allow, turned so the part's pad on that net faces it (no coordinates to "
                   "compute; a whole block: pcb_place_group); with: parts that move and turn together with it (a "
                   "placed block as one); locked ones need override_locks; answers where they are now and the "
                   "placement check of the moved footprints (overlaps, outside the board, airwire length; report: "
                   "full for the whole board)",
                   R"json({"type":"object","required":["moves"],"properties":{
                        "moves":{"type":"array","items":{"type":"object","required":["ref"],"properties":{
                            "ref":{"type":"string"},"x_mm":{"type":"number"},"y_mm":{"type":"number"},
                            "with":{"type":"array","items":{"type":"string"},"description":"references that keep their place relative to this footprint: they turn about its origin and shift with it (a placed block moves as one)"},
                            "near":{"type":"string","description":"REF.PAD of another part: place this one next to that pad (instead of x_mm / y_mm); with side bottom / top it may sit under the pad on the other side"},
                            "near_pad":{"type":"string","description":"near: the pad of this part to bring to the target (default: its pad on the target's net)"},
                            "near_direction":{"type":"string","enum":["auto","left","right","up","down"],"default":"auto","description":"near: which way from the target pad (on the board); auto tries all four, the side the pad is on first"},
                            "gap_mm":{"type":"number","default":0.25,"description":"near: gap between courtyards"},
                            "max_mm":{"type":"number","default":25,"description":"near: give up beyond this pad-to-pad distance"},
                            "rotation_deg":{"type":"number"},"side":{"type":"string","enum":["top","bottom"]},
                            "anchor":{"type":"string","enum":["origin","center","panel_edge"],"default":"origin","description":"center: x / y are the courtyard's middle; panel_edge: the middle of the connector's panel-edge line (put it on the board edge)"},
                            "edge_offset_mm":{"type":"number","default":0,"description":"anchor panel_edge: shift along the facing direction (+ out past the edge, - recessed)"},
                            "edge":{"type":"string","enum":["left","right","top","bottom"],"description":"put the connector on that board edge facing out (facing + anchor panel_edge + the edge coordinate); give the other coordinate"},
                            "facing":{"type":"string","enum":["left","right","up","down"],"description":"connectors: turn so the mating side faces this way (panel edge from the footprint's marker or inferred from its body, see pcb_footprint_get panel_edge)"}}}},
                        "override_locks":{"type":"boolean","default":false},
                        "report":{"type":"string","enum":["moved","full","none"],"default":"moved","description":"placement check in the answer: for the footprints moved (overlaps with any part, outside the board, connector facing; ratsnest of the whole board), the whole board, or none"}}})json"_json,
                   false, h_pcb_footprint_move );

KOPENAPI_REGISTER( "pcb_placement_check",
                   "Judge the placement: courtyard overlaps (same side), footprints outside the board, "
                   "connectors' distance to the board edge, unrouted connections and total airwire "
                   "length (shorter = better placement for routing), board size; ok flag",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_placement_check );

KOPENAPI_REGISTER( "pcb_place_auto",
                   "A first placement: board outline kept, given (width_mm / height_mm) or sized from the "
                   "parts (density, aspect); connectors along the edges; the rest by KiCad's autoplacer "
                   "(inside the outline, short airwires), then the given groups (a main part with the parts "
                   "that belong at its pins); unlocked footprints only; answers the placement check. Refine "
                   "with pcb_place_group and pcb_footprint_move (near)",
                   R"json({"type":"object","properties":{
                        "width_mm":{"type":"number"},"height_mm":{"type":"number"},
                        "x_mm":{"type":"number","default":20},"y_mm":{"type":"number","default":20},
                        "density":{"type":"number","default":0.35},"aspect":{"type":"number","default":1.5},
                        "connectors_to_edge":{"type":"boolean","default":true},
                        "groups":{"type":"array","description":"functional blocks to pull together after the first placement, each as pcb_place_group takes it: {anchor, parts?, side?, gap_mm?, max_mm?}","items":{"type":"object"}}}})json"_json,
                   false, h_pcb_place_auto, 600 );

KOPENAPI_MARK_EDITING( "pcb_footprint_move", "pcb_place_auto", "pcb_place_group" );
