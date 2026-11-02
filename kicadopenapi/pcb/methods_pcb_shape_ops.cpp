/// @file methods_pcb_shape_ops.cpp
/// @brief kicadopenapi shape editing and copies: pcb_shapes_edit, pcb_array, pcb_duplicate.
///
/// Polygon booleans and Fillet / Chamfer / Dogbone Lines on board graphics (the edit tool's
/// routines), Create Array (grid or circular) and Duplicate (and Increment) of footprints,
/// drawings, tracks, vias and zones; one commit per call.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <array_options.h>
#include <board.h>
#include <board_commit.h>
#include <core/kicad_algo.h>
#include <footprint.h>
#include <frame_type.h>
#include <increment.h>
#include <kicadopenapi_glow_view.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <pcb_group.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <tool/tool_manager.h>
#include <tools/item_modification_routine.h>

#include <cmath>
#include <map>
#include <optional>
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


nlohmann::json rectMm( const BOX2I& aBox )
{
    auto mm = []( int v ) { return std::round( v / pcbIUScale.IU_PER_MM * 1000 ) / 1000; };
    return { mm( aBox.GetLeft() ), mm( aBox.GetTop() ), mm( aBox.GetRight() ), mm( aBox.GetBottom() ) };
}


struct OPS_GLOW_TRAITS
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

    static void Brighten( FRAME* aFrame, EDA_ITEM* aItem, bool aOn ) { KopenapiPcbBrighten( aFrame, aItem, aOn ); }

    static BOX2I Box( EDA_ITEM* aItem ) { return aItem->GetBoundingBox(); }
    static int   Mm() { return pcbIUScale.mmToIU( 1.0 ); }
    static void  ItemColour( FRAME*, bool, std::optional<KIGFX::COLOR4D>& ) {}
};


void glow( KOPENAPI_CONTEXT& aCtx, const std::vector<KIID>& aIds )
{
    if( aCtx.headless )
        return;

    if( auto* frame = OPS_GLOW_TRAITS::Frame( aCtx.kiway ) )
        frame->GetCanvas()->Refresh();

    KopenapiGlow<OPS_GLOW_TRAITS>( aCtx.kiway, aIds, aIds.size() > 1 ? 120 : 0 );
}


/// @brief The board items named by refs or uuids (prefixes too); a footprint's own child
/// stands for the footprint
std::vector<BOARD_ITEM*> itemsOf( BOARD* aBoard, const nlohmann::json& aNames, nlohmann::json& aMissing )
{
    std::vector<BOARD_ITEM*> out;
    std::set<BOARD_ITEM*>    seen;

    for( const nlohmann::json& n : aNames )
    {
        BOARD_ITEM* item = n.is_string() ? KopenapiFindBoardItem( aBoard, n.get<std::string>() ) : nullptr;

        if( item && item->GetParentFootprint() )
            item = item->GetParentFootprint();

        if( !item )
            aMissing.push_back( n );
        else if( seen.insert( item ).second )
            out.push_back( item );
    }

    return out;
}


std::optional<KOPENAPI_RESULT> lockedRefusal( const std::vector<BOARD_ITEM*>& aItems, const nlohmann::json& aArgs )
{
    if( aArgs.value( "override_locks", false ) )
        return std::nullopt;

    nlohmann::json locked = nlohmann::json::array();

    for( BOARD_ITEM* item : aItems )
    {
        if( item->IsLocked() )
            locked.push_back( str( item->m_Uuid.AsString() ) );
    }

    if( locked.empty() )
        return std::nullopt;

    KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "locked items are in the request (pcb_unlock, or override_locks: true)" );
    refused.body["locked"] = locked;
    return refused;
}


bool duplicable( const BOARD_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case PCB_FOOTPRINT_T:
    case PCB_SHAPE_T:
    case PCB_TEXT_T:
    case PCB_TEXTBOX_T:
    case PCB_TABLE_T:
    case PCB_BARCODE_T:
    case PCB_REFERENCE_IMAGE_T:
    case PCB_TRACE_T:
    case PCB_ARC_T:
    case PCB_VIA_T:
    case PCB_ZONE_T:
    case PCB_TARGET_T:
    case PCB_DIM_ALIGNED_T:
    case PCB_DIM_CENTER_T:
    case PCB_DIM_RADIAL_T:
    case PCB_DIM_ORTHOGONAL_T:
    case PCB_DIM_LEADER_T:
    case PCB_GROUP_T:
        return true;
    default:
        return false;
    }
}


/// @brief Hands out the next free reference per prefix (R7 -> R8, ...), past every one on the
/// board and every one handed out before
class REF_SOURCE
{
public:
    explicit REF_SOURCE( BOARD* aBoard )
    {
        for( FOOTPRINT* fp : aBoard->Footprints() )
            m_used.insert( fp->GetReference() );
    }

    wxString Next( const wxString& aRef )
    {
        wxString prefix = aRef;

        while( !prefix.IsEmpty() && ( wxIsdigit( prefix.Last() ) || prefix.Last() == '?' ) )
            prefix.RemoveLast();

        int& n = m_next[prefix];

        if( n == 0 )
            n = 1;

        wxString ref;

        do
            ref = wxString::Format( wxS( "%s%d" ), prefix, n++ );
        while( m_used.count( ref ) );

        m_used.insert( ref );
        return ref;
    }

private:
    std::set<wxString>      m_used;
    std::map<wxString, int> m_next;
};


/// @brief A duplicate of aItem added to aCommit (a group with its members); nullptr for kinds
/// that do not copy
BOARD_ITEM* duplicate( BOARD_ITEM* aItem, BOARD_COMMIT& aCommit )
{
    BOARD_ITEM* dupe = nullptr;

    if( aItem->Type() == PCB_GROUP_T )
    {
        dupe = static_cast<PCB_GROUP*>( aItem )->DeepDuplicate( true, &aCommit );
        dupe->RunOnChildren( [&]( BOARD_ITEM* aChild ) { aCommit.Add( aChild ); }, RECURSE_MODE::RECURSE );
    }
    else
    {
        dupe = aItem->Duplicate( true, &aCommit );
    }

    dupe->ClearSelected();
    aCommit.Add( dupe );
    return dupe;
}


/// @brief New references for the footprints among aItems (and in their groups)
void renumber( BOARD_ITEM* aItem, REF_SOURCE& aRefs, nlohmann::json& aRenamed )
{
    auto one = [&]( BOARD_ITEM* aChild )
    {
        if( aChild->Type() != PCB_FOOTPRINT_T )
            return;

        FOOTPRINT*     fp = static_cast<FOOTPRINT*>( aChild );
        const wxString ref = aRefs.Next( fp->GetReference() );
        aRenamed.push_back( { { "from", str( fp->GetReference() ) }, { "to", str( ref ) } } );
        fp->SetReference( ref );
    };

    one( aItem );

    if( aItem->Type() == PCB_GROUP_T )
        aItem->RunOnChildren( one, RECURSE_MODE::RECURSE );
}


/// @brief Increments the free texts among aItems (and in their groups) like Edit > Increment
void increment( BOARD_ITEM* aItem, int aDelta )
{
    STRING_INCREMENTER incrementer;
    incrementer.SetSkipIOSQXZ( true );

    auto one = [&]( BOARD_ITEM* aChild )
    {
        if( aChild->Type() != PCB_TEXT_T )
            return;

        PCB_TEXT& text = static_cast<PCB_TEXT&>( *aChild );

        if( std::optional<wxString> next = incrementer.Increment( text.GetText(), aDelta, 0 ) )
            text.SetText( *next );
    };

    one( aItem );

    if( aItem->Type() == PCB_GROUP_T )
        aItem->RunOnChildren( one, RECURSE_MODE::RECURSE );
}


nlohmann::json describe( BOARD_ITEM* aItem )
{
    nlohmann::json row = { { "uuid", str( aItem->m_Uuid.AsString() ) },
                           { "kind", str( aItem->GetClass() ) },
                           { "bbox_mm", rectMm( aItem->GetBoundingBox() ) } };

    if( aItem->Type() == PCB_FOOTPRINT_T )
        row["ref"] = str( static_cast<FOOTPRINT*>( aItem )->GetReference() );
    else if( aItem->Type() == PCB_TEXT_T )
        row["text"] = str( static_cast<PCB_TEXT*>( aItem )->GetText() );

    return row;
}

} // namespace


static KOPENAPI_RESULT h_pcb_shapes_edit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string op = aArgs.value( "op", std::string() );
    const bool        boolean = op == "merge" || op == "subtract" || op == "intersect";
    const bool        lines = op == "fillet" || op == "chamfer" || op == "dogbone";

    if( !boolean && !lines )
        return KOPENAPI_RESULT::Error( 400, "op: merge, subtract, intersect, fillet, chamfer or dogbone" );

    if( !aArgs.contains( "uuids" ) || !aArgs["uuids"].is_array() || aArgs["uuids"].empty() )
        return KOPENAPI_RESULT::Error( 400, "uuids: the board shapes (pcb_drawing_list)" );

    nlohmann::json          missing = nlohmann::json::array();
    std::vector<PCB_SHAPE*> shapes;

    for( BOARD_ITEM* item : itemsOf( board, aArgs["uuids"], missing ) )
    {
        auto* shape = dynamic_cast<PCB_SHAPE*>( item );

        if( !shape || shape->GetParentFootprint() || shape->Type() != PCB_SHAPE_T )
        {
            missing.push_back( str( item->m_Uuid.AsString() ) );
            continue;
        }

        const SHAPE_T kind = shape->GetShape();
        const bool    fits = boolean ? ( kind == SHAPE_T::POLY || kind == SHAPE_T::RECTANGLE || kind == SHAPE_T::CIRCLE )
                                     : ( kind == SHAPE_T::SEGMENT || kind == SHAPE_T::POLY || kind == SHAPE_T::RECTANGLE );

        if( !fits )
        {
            KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error(
                    400, boolean ? "booleans take polygons, rectangles and circles"
                                 : "fillet / chamfer / dogbone take lines, polygons and rectangles" );
            refused.body["uuid"] = str( shape->m_Uuid.AsString() );
            refused.body["shape"] = str( shape->ShowShape() );
            return refused;
        }

        shapes.push_back( shape );
    }

    if( !missing.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 404, "not board shapes (pcb_drawing_list)" );
        refused.body["missing"] = missing;
        return refused;
    }

    if( boolean && shapes.size() < 2 )
        return KOPENAPI_RESULT::Error( 400, op + ": at least two shapes" );

    if( std::optional<KOPENAPI_RESULT> refused = lockedRefusal( { shapes.begin(), shapes.end() }, aArgs ) )
        return *refused;

    BOARD_COMMIT             commit( context->GetToolManager() );
    std::vector<BOARD_ITEM*> created;
    std::set<BOARD_ITEM*>    modified;
    std::set<KIID>           removed;

    ITEM_MODIFICATION_ROUTINE::CALLABLE_BASED_HANDLER handler(
            [&]( std::unique_ptr<BOARD_ITEM> aItem )
            {
                created.push_back( aItem.get() );
                commit.Add( aItem.release() );
            },
            [&]( BOARD_ITEM& aItem )
            {
                commit.Modify( &aItem );
                modified.insert( &aItem );
            },
            [&]( BOARD_ITEM& aItem )
            {
                aItem.SetFlags( STRUCT_DELETED );
                removed.insert( aItem.m_Uuid );
                commit.Remove( &aItem );
            } );

    unsigned successes = 0;
    unsigned failures = 0;

    if( boolean )
    {
        std::unique_ptr<POLYGON_BOOLEAN_ROUTINE> routine;

        if( op == "merge" )
            routine = std::make_unique<POLYGON_MERGE_ROUTINE>( board, handler );
        else if( op == "subtract" )
            routine = std::make_unique<POLYGON_SUBTRACT_ROUTINE>( board, handler );
        else
            routine = std::make_unique<POLYGON_INTERSECT_ROUTINE>( board, handler );

        // The first shape is the base (subtract: the one cut) and gives the result its properties
        for( PCB_SHAPE* shape : shapes )
            routine->ProcessShape( *shape );

        routine->Finalize();
        successes = routine->GetSuccesses();
        failures = routine->GetFailures();
    }
    else
    {
        const double size = aArgs.value( "radius_mm", aArgs.value( "setback_mm", 1.0 ) );

        if( size <= 0 )
            return KOPENAPI_RESULT::Error( 400, "radius_mm / setback_mm must be positive" );

        // Rectangles and polygons become their sides, as Fillet / Chamfer Lines does
        std::vector<PCB_SHAPE*> segments;

        for( PCB_SHAPE* shape : shapes )
        {
            if( shape->GetShape() == SHAPE_T::SEGMENT )
            {
                segments.push_back( shape );
                continue;
            }

            std::vector<VECTOR2I> pts;

            if( shape->GetShape() == SHAPE_T::RECTANGLE )
            {
                const VECTOR2I s = shape->GetStart();
                const VECTOR2I e = shape->GetEnd();
                pts = { s, VECTOR2I( e.x, s.y ), e, VECTOR2I( s.x, e.y ) };
            }
            else
            {
                for( int i = 0; i < shape->GetPolyShape().VertexCount(); ++i )
                    pts.push_back( shape->GetPolyShape().CVertex( i ) );
            }

            for( size_t i = 0; i < pts.size(); ++i )
            {
                auto* side = new PCB_SHAPE( board, SHAPE_T::SEGMENT );
                side->SetStart( pts[i] );
                side->SetEnd( pts[( i + 1 ) % pts.size()] );
                side->SetStroke( shape->GetStroke() );
                side->SetLayer( shape->GetLayer() );
                commit.Add( side );
                created.push_back( side );
                segments.push_back( side );
            }

            removed.insert( shape->m_Uuid );
            commit.Remove( shape );
        }

        if( segments.size() < 2 )
        {
            commit.Revert();
            return KOPENAPI_RESULT::Error( 400, op + ": at least two lines" );
        }

        std::unique_ptr<PAIRWISE_LINE_ROUTINE> routine;

        if( op == "fillet" )
        {
            routine = std::make_unique<LINE_FILLET_ROUTINE>( board, handler, toIU( size ) );
        }
        else if( op == "chamfer" )
        {
            routine = std::make_unique<LINE_CHAMFER_ROUTINE>( board, handler,
                                                              CHAMFER_PARAMS{ toIU( size ), toIU( size ) } );
        }
        else
        {
            routine = std::make_unique<DOGBONE_CORNER_ROUTINE>(
                    board, handler, DOGBONE_CORNER_ROUTINE::PARAMETERS{ toIU( size ), aArgs.value( "add_slots", false ) } );
        }

        for( PCB_SHAPE* s : segments )
            s->ClearFlags( STRUCT_DELETED );

        alg::for_all_pairs( segments.begin(), segments.end(),
                            [&]( PCB_SHAPE* a, PCB_SHAPE* b )
                            {
                                if( !( a->GetFlags() & STRUCT_DELETED ) && !( b->GetFlags() & STRUCT_DELETED ) )
                                    routine->ProcessLinePair( *a, *b );
                            } );

        successes = routine->GetSuccesses();
        failures = routine->GetFailures();
    }

    if( successes == 0 )
    {
        commit.Revert();

        for( PCB_SHAPE* shape : shapes )
            shape->ClearFlags( STRUCT_DELETED );

        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 422, op + ": nothing changed (shapes apart, or no corner)" );
        refused.body["failures"] = failures;
        return refused;
    }

    commit.Push( wxString::Format( wxS( "%s (API)" ), wxString::FromUTF8( op ) ) );

    nlohmann::json    result = nlohmann::json::array();
    std::vector<KIID> ids;

    std::set<KIID> listed;

    auto add = [&]( BOARD_ITEM* aItem )
    {
        if( removed.count( aItem->m_Uuid ) || !listed.insert( aItem->m_Uuid ).second )
            return;

        nlohmann::json row = describe( aItem );

        if( auto* s = dynamic_cast<PCB_SHAPE*>( aItem ) )
            row["shape"] = str( s->ShowShape() );

        result.push_back( row );
        ids.push_back( aItem->m_Uuid );
    };

    for( BOARD_ITEM* item : modified )
        add( item );

    for( BOARD_ITEM* item : created )
        add( item );

    nlohmann::json gone = nlohmann::json::array();

    for( const KIID& id : removed )
        gone.push_back( str( id.AsString() ) );

    glow( aCtx, ids );

    return KOPENAPI_RESULT::Ok( { { "op", op }, { "items", result }, { "removed", gone },
                                  { "corners", successes }, { "failures", failures } } );
}


static KOPENAPI_RESULT h_pcb_array( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( !aArgs.contains( "items" ) || !aArgs["items"].is_array() || aArgs["items"].empty() )
        return KOPENAPI_RESULT::Error( 400, "items: refs or uuids of what to copy" );

    const bool grid = aArgs.contains( "grid" );

    if( grid == aArgs.contains( "circular" ) )
        return KOPENAPI_RESULT::Error( 400, "grid or circular (one of them)" );

    std::unique_ptr<ARRAY_OPTIONS> opts;

    if( grid )
    {
        const nlohmann::json& g = aArgs["grid"];
        auto                  o = std::make_unique<ARRAY_GRID_OPTIONS>();
        o->m_nx = g.value( "nx", 1 );
        o->m_ny = g.value( "ny", 1 );
        o->m_delta = VECTOR2I( toIU( g.value( "dx_mm", 0.0 ) ), toIU( g.value( "dy_mm", 0.0 ) ) );
        o->m_offset = VECTOR2I( toIU( g.value( "row_offset_mm", 0.0 ) ), toIU( g.value( "column_offset_mm", 0.0 ) ) );
        o->m_stagger = std::max( 1, g.value( "stagger", 1 ) );
        o->m_stagger_rows = g.value( "stagger_rows", true );
        o->m_horizontalThenVertical = g.value( "rows_first", true );

        if( o->m_nx < 1 || o->m_ny < 1 || o->m_nx * o->m_ny < 2 )
            return KOPENAPI_RESULT::Error( 400, "grid: nx, ny >= 1 and at least two cells" );

        opts = std::move( o );
    }
    else
    {
        const nlohmann::json& c = aArgs["circular"];
        auto                  o = std::make_unique<ARRAY_CIRCULAR_OPTIONS>();
        const nlohmann::json  centre = c.value( "centre", c.value( "center", nlohmann::json() ) );

        if( !centre.is_array() || centre.size() != 2 || !centre[0].is_number() || !centre[1].is_number() )
            return KOPENAPI_RESULT::Error( 400, "circular: centre [x_mm, y_mm]" );

        o->m_centre = VECTOR2I( toIU( centre[0].get<double>() ), toIU( centre[1].get<double>() ) );
        o->m_nPts = c.value( "count", 0 );
        o->m_angle = EDA_ANGLE( c.value( "angle_deg", 0.0 ), DEGREES_T );
        o->m_clockwise = c.value( "clockwise", false );
        o->m_rotateItems = c.value( "rotate_items", true );

        if( o->m_nPts < 2 )
            return KOPENAPI_RESULT::Error( 400, "circular: count >= 2" );

        opts = std::move( o );
    }

    const int count = opts->GetArraySize();

    if( count > 10000 )
        return KOPENAPI_RESULT::Error( 400, "an array of more than 10000 copies" );

    nlohmann::json           missing = nlohmann::json::array();
    std::vector<BOARD_ITEM*> items = itemsOf( board, aArgs["items"], missing );

    if( !missing.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 404, "no such board items" );
        refused.body["missing"] = missing;
        return refused;
    }

    for( BOARD_ITEM* item : items )
    {
        if( !duplicable( item ) )
            return KOPENAPI_RESULT::Error( 400, "cannot copy a " + str( item->GetClass() ) );
    }

    BOARD_COMMIT   commit( context->GetToolManager() );
    REF_SOURCE     refs( board );
    const bool     reannotate = aArgs.value( "reannotate", true );
    const int      step = aArgs.value( "increment", 0 );
    nlohmann::json cells = nlohmann::json::array();
    nlohmann::json renamed = nlohmann::json::array();
    std::vector<KIID> ids;

    // The originals stay as cell 0; each further cell is a copy of them all
    for( int n = 1; n < count; ++n )
    {
        nlohmann::json cell = nlohmann::json::array();

        for( BOARD_ITEM* item : items )
        {
            BOARD_ITEM*                    dupe = duplicate( item, commit );
            const ARRAY_OPTIONS::TRANSFORM t = opts->GetTransform( n, item->GetPosition() );

            dupe->Move( t.m_offset );
            dupe->Rotate( dupe->GetPosition(), t.m_rotation );

            if( reannotate )
                renumber( dupe, refs, renamed );

            if( step )
                increment( dupe, step * n );

            cell.push_back( describe( dupe ) );
            ids.push_back( dupe->m_Uuid );
        }

        cells.push_back( cell );
    }

    commit.Push( _( "Create Array (API)" ) );
    glow( aCtx, ids );

    return KOPENAPI_RESULT::Ok( { { "cells", count }, { "copies", cells }, { "renamed", renamed } } );
}


static KOPENAPI_RESULT h_pcb_duplicate( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( !aArgs.contains( "items" ) || !aArgs["items"].is_array() || aArgs["items"].empty() )
        return KOPENAPI_RESULT::Error( 400, "items: refs or uuids of what to duplicate" );

    nlohmann::json           missing = nlohmann::json::array();
    std::vector<BOARD_ITEM*> items = itemsOf( board, aArgs["items"], missing );

    if( !missing.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 404, "no such board items" );
        refused.body["missing"] = missing;
        return refused;
    }

    for( BOARD_ITEM* item : items )
    {
        if( !duplicable( item ) )
            return KOPENAPI_RESULT::Error( 400, "cannot duplicate a " + str( item->GetClass() ) );
    }

    const VECTOR2I delta( toIU( aArgs.value( "dx_mm", 0.0 ) ), toIU( aArgs.value( "dy_mm", 0.0 ) ) );
    const int      step = aArgs.value( "increment", 0 );
    const bool     reannotate = aArgs.value( "reannotate", true );
    BOARD_COMMIT   commit( context->GetToolManager() );
    REF_SOURCE     refs( board );
    nlohmann::json copies = nlohmann::json::array();
    nlohmann::json renamed = nlohmann::json::array();
    std::vector<KIID> ids;

    for( BOARD_ITEM* item : items )
    {
        BOARD_ITEM* dupe = duplicate( item, commit );
        dupe->Move( delta );

        if( reannotate )
            renumber( dupe, refs, renamed );

        if( step )
            increment( dupe, step );

        copies.push_back( describe( dupe ) );
        ids.push_back( dupe->m_Uuid );
    }

    commit.Push( _( "Duplicate (API)" ) );
    glow( aCtx, ids );

    return KOPENAPI_RESULT::Ok( { { "copies", copies }, { "renamed", renamed } } );
}


KOPENAPI_REGISTER( "pcb_shapes_edit",
                   "Edit board graphics like Edit > Shape Modification: merge / subtract / intersect "
                   "polygons, rectangles and circles (the first uuid is the base, subtract cuts the others "
                   "from it, the result keeps its layer and stroke), fillet / chamfer / dogbone the corners "
                   "where lines meet (rectangles and polygons become their sides); one undo step; answers "
                   "the resulting shapes, the removed uuids and the corners done (422 when nothing changed)",
                   R"json({"type":"object","required":["op","uuids"],"properties":{
                        "op":{"type":"string","enum":["merge","subtract","intersect","fillet","chamfer","dogbone"]},
                        "uuids":{"type":"array","items":{"type":"string"},"description":"board shapes (pcb_drawing_list)"},
                        "radius_mm":{"type":"number","default":1,"description":"fillet / dogbone radius"},
                        "setback_mm":{"type":"number","default":1,"description":"chamfer setback"},
                        "add_slots":{"type":"boolean","default":false,"description":"dogbone: slots for a cutter wider than the gap"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_shapes_edit );

KOPENAPI_REGISTER( "pcb_array",
                   "Create Array: copies of footprints, drawings, tracks, vias, zones or groups (items: refs "
                   "or uuids) on a grid (nx x ny, dx_mm / dy_mm pitch, stagger) or a circle (centre, count, "
                   "angle_deg step or 0 for an even spread, rotate_items); the originals are the first cell; "
                   "copied footprints get the next free references (reannotate) and free texts can count up "
                   "(increment per cell); one undo step; answers each cell's copies",
                   R"json({"type":"object","required":["items"],"properties":{
                        "items":{"type":"array","items":{"type":"string"}},
                        "grid":{"type":"object","properties":{
                            "nx":{"type":"integer","minimum":1},"ny":{"type":"integer","minimum":1},
                            "dx_mm":{"type":"number"},"dy_mm":{"type":"number"},
                            "row_offset_mm":{"type":"number","description":"x shift per row"},
                            "column_offset_mm":{"type":"number","description":"y shift per column"},
                            "stagger":{"type":"integer","minimum":1},"stagger_rows":{"type":"boolean","default":true},
                            "rows_first":{"type":"boolean","default":true}}},
                        "circular":{"type":"object","properties":{
                            "centre":{"type":"array","items":{"type":"number"}},
                            "count":{"type":"integer","minimum":2},
                            "angle_deg":{"type":"number","default":0},
                            "clockwise":{"type":"boolean","default":false},
                            "rotate_items":{"type":"boolean","default":true}}},
                        "reannotate":{"type":"boolean","default":true},
                        "increment":{"type":"integer","default":0,"description":"free texts count up by this per cell"}}})json"_json,
                   false, h_pcb_array );

KOPENAPI_REGISTER( "pcb_duplicate",
                   "Duplicate (and Duplicate and Increment) footprints, drawings, tracks, vias, zones or "
                   "groups (items: refs or uuids), moved by dx_mm / dy_mm; copied footprints get the next "
                   "free references (reannotate); increment counts free texts up (D1 -> D2); one undo step; "
                   "answers the copies",
                   R"json({"type":"object","required":["items"],"properties":{
                        "items":{"type":"array","items":{"type":"string"}},
                        "dx_mm":{"type":"number","default":0},"dy_mm":{"type":"number","default":0},
                        "reannotate":{"type":"boolean","default":true},
                        "increment":{"type":"integer","default":0}}})json"_json,
                   false, h_pcb_duplicate );

KOPENAPI_MARK_EDITING( "pcb_shapes_edit", "pcb_array", "pcb_duplicate" );
