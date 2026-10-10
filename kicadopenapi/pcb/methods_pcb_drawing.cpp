/// @file methods_pcb_drawing.cpp
/// @brief kicadopenapi board text and graphics: pcb_text_add, pcb_graphic_add.
///
/// Free text and shapes on any board layer (silkscreen art, fab notes, keep-out drawings): one
/// commit per call, the new items glow in the GUI.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <frame_type.h>
#include <kicadopenapi_glow_view.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <geometry/shape_compound.h>
#include <stroke_params.h>
#include <tool/tool_manager.h>

#include <cmath>


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


struct DRAW_GLOW_TRAITS
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
        if( aOn )
            aItem->SetBrightened();
        else
            aItem->ClearBrightened();

        aFrame->GetCanvas()->GetView()->Update( aItem, KIGFX::REPAINT );
    }

    static BOX2I Box( EDA_ITEM* aItem ) { return aItem->GetBoundingBox(); }
    static int   Mm() { return pcbIUScale.mmToIU( 1.0 ); }
    static void  ItemColour( FRAME*, bool, std::optional<KIGFX::COLOR4D>& ) {}
};


/// @brief A board layer by name (F.SilkS, B.Fab, Edge.Cuts, User.Drawings, ...), or nullopt
std::optional<PCB_LAYER_ID> layerOf( BOARD* aBoard, const nlohmann::json& aArgs, const char* aDefault )
{
    const std::string name = aArgs.value( "layer", std::string( aDefault ) );
    const int         id = aBoard->GetLayerID( wxString::FromUTF8( name ) );

    if( id < 0 || !aBoard->IsLayerEnabled( (PCB_LAYER_ID) id ) )
        return std::nullopt;

    return (PCB_LAYER_ID) id;
}


VECTOR2I point( const nlohmann::json& aPoint )
{
    return VECTOR2I( toIU( aPoint[0].get<double>() ), toIU( aPoint[1].get<double>() ) );
}


bool isPoint( const nlohmann::json& aPoint )
{
    return aPoint.is_array() && aPoint.size() == 2 && aPoint[0].is_number() && aPoint[1].is_number();
}


void finish( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, BOARD_COMMIT& aCommit, const std::vector<KIID>& aIds,
             const wxString& aMessage )
{
    aCommit.Push( aMessage );

    if( !aCtx.headless )
    {
        if( auto* frame = DRAW_GLOW_TRAITS::Frame( aCtx.kiway ) )
        {
            frame->GetCanvas()->Refresh();
            KopenapiRefresh3D( frame );   // an open 3D viewer shows the change right away
        }

        KopenapiGlow<DRAW_GLOW_TRAITS>( aCtx.kiway, aIds, 0 );
    }
}

} // namespace


static KOPENAPI_RESULT h_pcb_text_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                      board = context->GetBoard();
    std::optional<PCB_LAYER_ID> layer = layerOf( board, aArgs, "F.SilkS" );

    if( !layer )
        return KOPENAPI_RESULT::Error( 400, "layer: an enabled board layer (pcb_layer_list)" );

    const std::string text = aArgs.value( "text", std::string() );

    if( text.empty() )
        return KOPENAPI_RESULT::Error( 400, "text is empty" );

    const double size = aArgs.value( "size_mm", 1.5 );

    auto* item = new PCB_TEXT( board );
    item->SetText( wxString::FromUTF8( text ) );
    item->SetLayer( *layer );
    item->SetTextSize( VECTOR2I( toIU( aArgs.value( "width_mm", size ) ), toIU( size ) ) );
    item->SetTextThickness( toIU( aArgs.value( "thickness_mm", size * 0.15 ) ) );
    item->SetBold( aArgs.value( "bold", false ) );
    item->SetItalic( aArgs.value( "italic", false ) );
    item->SetTextAngle( EDA_ANGLE( aArgs.value( "angle_deg", 0.0 ), DEGREES_T ) );

    const std::string justify = aArgs.value( "justify", std::string( "center" ) );
    item->SetHorizJustify( justify == "left" ? GR_TEXT_H_ALIGN_LEFT
                           : justify == "right" ? GR_TEXT_H_ALIGN_RIGHT : GR_TEXT_H_ALIGN_CENTER );
    item->SetVertJustify( GR_TEXT_V_ALIGN_CENTER );
    item->SetIsKnockout( aArgs.value( "knockout", false ) );
    item->SetMirrored( IsBackLayer( *layer ) );   // reads correctly from below
    item->SetPosition( VECTOR2I( toIU( aArgs.value( "x_mm", 0.0 ) ), toIU( aArgs.value( "y_mm", 0.0 ) ) ) );

    BOARD_COMMIT commit( context->GetToolManager() );
    commit.Add( item );
    finish( aCtx, *context, commit, { item->m_Uuid }, _( "Add text (API)" ) );

    const BOX2I                     box = item->GetBoundingBox();
    std::shared_ptr<SHAPE_COMPOUND> shape = item->GetEffectiveTextShape( false );
    const BOX2I                     ink = shape && !shape->Shapes().empty() ? shape->BBox() : box;
    auto                            mm = []( int v ) { return std::round( v / pcbIUScale.IU_PER_MM * 100 ) / 100; };
    auto                            rect = [&]( const BOX2I& b ) -> nlohmann::json
    {
        return { mm( b.GetLeft() ), mm( b.GetTop() ), mm( b.GetRight() ), mm( b.GetBottom() ) };
    };

    return KOPENAPI_RESULT::Ok( { { "uuid", str( item->m_Uuid.AsString() ) },
                                  { "layer", str( board->GetLayerName( *layer ) ) },
                                  { "bbox_mm", rect( box ) },
                                  { "ink_mm", rect( ink ) } } );
}


static KOPENAPI_RESULT h_pcb_graphic_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                      board = context->GetBoard();
    std::optional<PCB_LAYER_ID> layer = layerOf( board, aArgs, "F.SilkS" );

    if( !layer )
        return KOPENAPI_RESULT::Error( 400, "layer: an enabled board layer (pcb_layer_list)" );

    const std::string kind = aArgs.value( "shape", std::string() );
    const nlohmann::json points = aArgs.value( "points", nlohmann::json::array() );

    for( const nlohmann::json& p : points )
    {
        if( !isPoint( p ) )
            return KOPENAPI_RESULT::Error( 400, "points: [[x_mm, y_mm], ...]" );
    }

    const STROKE_PARAMS stroke( toIU( aArgs.value( "width_mm", 0.15 ) ), LINE_STYLE::SOLID );
    const bool          filled = aArgs.value( "filled", false );
    std::vector<PCB_SHAPE*> shapes;

    auto make = [&]( SHAPE_T aType )
    {
        auto* s = new PCB_SHAPE( board, aType );
        s->SetLayer( *layer );
        s->SetStroke( stroke );
        shapes.push_back( s );
        return s;
    };

    if( kind == "line" || kind == "polyline" )
    {
        // an open polyline is a chain of segments (KiCad's polygons are closed)
        if( points.size() < 2 )
            return KOPENAPI_RESULT::Error( 400, kind + ": at least 2 points" );

        for( size_t i = 0; i + 1 < points.size(); ++i )
        {
            PCB_SHAPE* s = make( SHAPE_T::SEGMENT );
            s->SetStart( point( points[i] ) );
            s->SetEnd( point( points[i + 1] ) );
        }
    }
    else if( kind == "rect" )
    {
        if( points.size() != 2 )
            return KOPENAPI_RESULT::Error( 400, "rect: two corners" );

        PCB_SHAPE* s = make( SHAPE_T::RECTANGLE );
        s->SetStart( point( points[0] ) );
        s->SetEnd( point( points[1] ) );
        s->SetFilled( filled );
    }
    else if( kind == "circle" )
    {
        if( points.size() != 1 || !aArgs.contains( "radius_mm" ) )
            return KOPENAPI_RESULT::Error( 400, "circle: one point (centre) and radius_mm" );

        PCB_SHAPE* s = make( SHAPE_T::CIRCLE );
        s->SetCenter( point( points[0] ) );
        s->SetEnd( point( points[0] ) + VECTOR2I( toIU( aArgs["radius_mm"].get<double>() ), 0 ) );
        s->SetFilled( filled );
    }
    else if( kind == "arc" )
    {
        if( points.size() != 3 )
            return KOPENAPI_RESULT::Error( 400, "arc: start, middle, end" );

        PCB_SHAPE* s = make( SHAPE_T::ARC );
        s->SetArcGeometry( point( points[0] ), point( points[1] ), point( points[2] ) );
    }
    else if( kind == "polygon" )
    {
        if( points.size() < 3 )
            return KOPENAPI_RESULT::Error( 400, "polygon: at least 3 points" );

        std::vector<VECTOR2I> pts;

        for( const nlohmann::json& p : points )
            pts.push_back( point( p ) );

        PCB_SHAPE* s = make( SHAPE_T::POLY );
        s->SetPolyPoints( pts );
        s->SetFilled( filled );
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "shape: line, polyline, rect, circle, arc or polygon" );
    }

    BOARD_COMMIT      commit( context->GetToolManager() );
    std::vector<KIID> ids;
    nlohmann::json    uuids = nlohmann::json::array();

    for( PCB_SHAPE* s : shapes )
    {
        commit.Add( s );
        ids.push_back( s->m_Uuid );
        uuids.push_back( str( s->m_Uuid.AsString() ) );
    }

    finish( aCtx, *context, commit, ids, _( "Add graphics (API)" ) );

    return KOPENAPI_RESULT::Ok( { { "uuids", uuids }, { "items", shapes.size() },
                                  { "layer", str( board->GetLayerName( *layer ) ) } } );
}


namespace
{

nlohmann::json rectMm( const BOX2I& aBox )
{
    auto mm = []( int v ) { return std::round( v / pcbIUScale.IU_PER_MM * 100 ) / 100; };
    return { mm( aBox.GetLeft() ), mm( aBox.GetTop() ), mm( aBox.GetRight() ), mm( aBox.GetBottom() ) };
}


/// @brief Board-level drawings (texts, shapes) by uuid; footprints' own items are not included
std::vector<BOARD_ITEM*> drawingsByUuid( BOARD* aBoard, const nlohmann::json& aUuids, nlohmann::json& aMissing )
{
    std::vector<BOARD_ITEM*> out;

    for( const nlohmann::json& u : aUuids )
    {
        const KIID id( wxString::FromUTF8( u.get<std::string>() ) );
        BOARD_ITEM* found = nullptr;

        for( BOARD_ITEM* item : aBoard->Drawings() )
        {
            if( item->m_Uuid == id )
                found = item;
        }

        if( found )
            out.push_back( found );
        else
            aMissing.push_back( u );
    }

    return out;
}

} // namespace


static KOPENAPI_RESULT h_pcb_drawing_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string layerName = aArgs.value( "layer", std::string() );
    const std::string kind = aArgs.value( "kind", std::string() );
    nlohmann::json    items = nlohmann::json::array();
    auto              mm = []( int v ) { return std::round( v / pcbIUScale.IU_PER_MM * 100 ) / 100; };

    for( BOARD_ITEM* item : board->Drawings() )
    {
        const std::string layer = str( board->GetLayerName( item->GetLayer() ) );

        if( !layerName.empty() && layer != layerName && str( LayerName( item->GetLayer() ) ) != layerName )
            continue;

        nlohmann::json row = { { "uuid", str( item->m_Uuid.AsString() ) }, { "layer", layer },
                               { "bbox_mm", rectMm( item->GetBoundingBox() ) } };

        if( auto* t = dynamic_cast<PCB_TEXT*>( item ) )
        {
            if( !kind.empty() && kind != "text" )
                continue;

            std::shared_ptr<SHAPE_COMPOUND> ink = t->GetEffectiveTextShape( false );
            row["kind"] = "text";
            row["text"] = str( t->GetText() );
            row["x_mm"] = mm( t->GetPosition().x );
            row["y_mm"] = mm( t->GetPosition().y );
            row["size_mm"] = mm( t->GetTextHeight() );
            row["width_mm"] = mm( t->GetTextWidth() );
            row["thickness_mm"] = mm( t->GetTextThickness() );
            row["angle_deg"] = t->GetTextAngle().AsDegrees();
            row["ink_mm"] = rectMm( ink && !ink->Shapes().empty() ? ink->BBox() : t->GetBoundingBox() );
        }
        else if( auto* sh = dynamic_cast<PCB_SHAPE*>( item ) )
        {
            if( !kind.empty() && kind != "shape" )
                continue;

            row["kind"] = "shape";
            row["shape"] = str( sh->ShowShape() );
            row["width_mm"] = mm( sh->GetWidth() );
            row["filled"] = sh->IsSolidFill();
        }
        else
        {
            if( !kind.empty() )
                continue;

            row["kind"] = str( item->GetClass() );
        }

        items.push_back( row );
    }

    return KOPENAPI_RESULT::Ok( { { "items", items }, { "total", items.size() } } );
}


static KOPENAPI_RESULT h_pcb_item_delete( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( !aArgs.contains( "uuids" ) || !aArgs["uuids"].is_array() || aArgs["uuids"].empty() )
        return KOPENAPI_RESULT::Error( 400, "uuids: the drawings to delete (pcb_drawing_list)" );

    nlohmann::json           missing = nlohmann::json::array();
    std::vector<BOARD_ITEM*> items = drawingsByUuid( context->GetBoard(), aArgs["uuids"], missing );

    if( items.empty() )
        return KOPENAPI_RESULT::Error( 404, "no board drawing with these uuids" );

    BOARD_COMMIT commit( context->GetToolManager() );

    for( BOARD_ITEM* item : items )
        commit.Remove( item );

    commit.Push( _( "Delete drawings (API)" ) );

    if( !aCtx.headless )
    {
        if( auto* frame = DRAW_GLOW_TRAITS::Frame( aCtx.kiway ) )
        {
            frame->GetCanvas()->Refresh();
            KopenapiRefresh3D( frame );   // an open 3D viewer shows the change right away
        }
    }

    return KOPENAPI_RESULT::Ok( { { "deleted", items.size() }, { "missing", missing } } );
}


static KOPENAPI_RESULT h_pcb_items_move( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( !aArgs.contains( "uuids" ) || !aArgs["uuids"].is_array() || aArgs["uuids"].empty() )
        return KOPENAPI_RESULT::Error( 400, "uuids: the drawings to move (pcb_drawing_list)" );

    nlohmann::json           missing = nlohmann::json::array();
    std::vector<BOARD_ITEM*> items = drawingsByUuid( context->GetBoard(), aArgs["uuids"], missing );

    if( items.empty() )
        return KOPENAPI_RESULT::Error( 404, "no board drawing with these uuids" );

    const VECTOR2I delta( toIU( aArgs.value( "dx_mm", 0.0 ) ), toIU( aArgs.value( "dy_mm", 0.0 ) ) );
    BOARD_COMMIT   commit( context->GetToolManager() );
    std::vector<KIID> ids;
    BOX2I          box;

    for( BOARD_ITEM* item : items )
    {
        commit.Modify( item );
        item->Move( delta );
        ids.push_back( item->m_Uuid );
        box.Merge( item->GetBoundingBox() );
    }

    finish( aCtx, *context, commit, ids, _( "Move drawings (API)" ) );
    return KOPENAPI_RESULT::Ok( { { "moved", items.size() }, { "missing", missing }, { "bbox_mm", rectMm( box ) } } );
}


static KOPENAPI_RESULT h_pcb_text_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    nlohmann::json           missing = nlohmann::json::array();
    std::vector<BOARD_ITEM*> found = drawingsByUuid( context->GetBoard(), nlohmann::json::array( { aArgs.value( "uuid", std::string() ) } ),
                                                     missing );
    auto*                    t = found.empty() ? nullptr : dynamic_cast<PCB_TEXT*>( found.front() );

    if( !t )
        return KOPENAPI_RESULT::Error( 404, "uuid: a board text (pcb_drawing_list kind text)" );

    BOARD_COMMIT commit( context->GetToolManager() );
    commit.Modify( t );

    if( aArgs.contains( "text" ) )
        t->SetText( wxString::FromUTF8( aArgs["text"].get<std::string>() ) );

    // a new height keeps the glyph proportions and the stroke ratio unless they are given too
    if( aArgs.contains( "size_mm" ) )
    {
        const double k = toIU( aArgs["size_mm"].get<double>() ) / double( std::max( 1, t->GetTextHeight() ) );
        t->SetTextSize( VECTOR2I( int( t->GetTextWidth() * k ), toIU( aArgs["size_mm"].get<double>() ) ) );
        t->SetTextThickness( int( t->GetTextThickness() * k ) );
    }

    if( aArgs.contains( "width_mm" ) )
        t->SetTextWidth( toIU( aArgs["width_mm"].get<double>() ) );

    if( aArgs.contains( "thickness_mm" ) )
        t->SetTextThickness( toIU( aArgs["thickness_mm"].get<double>() ) );

    if( aArgs.contains( "angle_deg" ) )
        t->SetTextAngle( EDA_ANGLE( aArgs["angle_deg"].get<double>(), DEGREES_T ) );

    if( aArgs.contains( "bold" ) )
        t->SetBold( aArgs["bold"].get<bool>() );

    VECTOR2I pos = t->GetPosition();

    if( aArgs.contains( "x_mm" ) )
        pos.x = toIU( aArgs["x_mm"].get<double>() );

    if( aArgs.contains( "y_mm" ) )
        pos.y = toIU( aArgs["y_mm"].get<double>() );

    pos += VECTOR2I( toIU( aArgs.value( "dx_mm", 0.0 ) ), toIU( aArgs.value( "dy_mm", 0.0 ) ) );
    t->SetPosition( pos );

    finish( aCtx, *context, commit, { t->m_Uuid }, _( "Edit text (API)" ) );

    std::shared_ptr<SHAPE_COMPOUND> ink = t->GetEffectiveTextShape( false );
    return KOPENAPI_RESULT::Ok( { { "uuid", str( t->m_Uuid.AsString() ) },
                                  { "bbox_mm", rectMm( t->GetBoundingBox() ) },
                                  { "ink_mm", rectMm( ink && !ink->Shapes().empty() ? ink->BBox() : t->GetBoundingBox() ) } } );
}


KOPENAPI_REGISTER( "pcb_drawing_list",
                   "List the board's own drawings (not footprints'): texts (text, position, size, width, "
                   "stroke, angle, drawn box ink_mm) and shapes (shape, stroke, fill) with uuid, layer and box; "
                   "filter by layer and kind",
                   R"json({"type":"object","properties":{
                        "layer":{"type":"string","description":"e.g. F.SilkS or F.Silkscreen"},
                        "kind":{"type":"string","enum":["text","shape"]}}})json"_json,
                   false, h_pcb_drawing_list );

KOPENAPI_REGISTER( "pcb_item_delete",
                   "Delete board drawings (texts, shapes) by uuid (pcb_drawing_list); one undo step",
                   R"json({"type":"object","required":["uuids"],"properties":{
                        "uuids":{"type":"array","items":{"type":"string"}}}})json"_json,
                   false, h_pcb_item_delete );

KOPENAPI_REGISTER( "pcb_items_move",
                   "Move board drawings (texts, shapes; e.g. a logo made of several) together by dx / dy mm; "
                   "one undo step; answers their box",
                   R"json({"type":"object","required":["uuids"],"properties":{
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "dx_mm":{"type":"number","default":0},"dy_mm":{"type":"number","default":0}}})json"_json,
                   false, h_pcb_items_move );

KOPENAPI_REGISTER( "pcb_text_update",
                   "Edit a board text by uuid: text, position (x_mm / y_mm or dx_mm / dy_mm), height "
                   "(size_mm keeps the glyph proportions and stroke ratio), glyph width (narrower / wider), "
                   "stroke, angle, bold; answers its boxes (check with pcb_silk_fit / pcb_drc)",
                   R"json({"type":"object","required":["uuid"],"properties":{
                        "uuid":{"type":"string"},
                        "text":{"type":"string"},
                        "x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "dx_mm":{"type":"number"},"dy_mm":{"type":"number"},
                        "size_mm":{"type":"number"},"width_mm":{"type":"number"},"thickness_mm":{"type":"number"},
                        "angle_deg":{"type":"number"},"bold":{"type":"boolean"}}})json"_json,
                   false, h_pcb_text_update );


KOPENAPI_REGISTER( "pcb_text_add",
                   "Put text on the board (silkscreen title, labels, pinouts, fab notes) on any layer "
                   "(default F.SilkS): position, size, thickness, bold, italic, angle, justify, knockout; "
                   "mirrored automatically on back layers; answers uuid, text box (bbox_mm) and drawn strokes (ink_mm)",
                   R"json({"type":"object","required":["text","x_mm","y_mm"],"properties":{
                        "text":{"type":"string"},
                        "x_mm":{"type":"number"}, "y_mm":{"type":"number"},
                        "layer":{"type":"string","default":"F.SilkS"},
                        "size_mm":{"type":"number","default":1.5,"description":"glyph height"},
                        "width_mm":{"type":"number","description":"glyph width; default the height"},
                        "thickness_mm":{"type":"number","description":"stroke; default 15 % of the height"},
                        "bold":{"type":"boolean","default":false},
                        "italic":{"type":"boolean","default":false},
                        "angle_deg":{"type":"number","default":0},
                        "justify":{"type":"string","enum":["left","center","right"],"default":"center"},
                        "knockout":{"type":"boolean","default":false,"description":"text cut out of a filled box"}}})json"_json,
                   false, h_pcb_text_add );

KOPENAPI_REGISTER( "pcb_graphic_add",
                   "Draw on the board (silkscreen art, frames, arrows, fab drawings) on any layer "
                   "(default F.SilkS): line / polyline (open, points), rect (two corners), circle (centre "
                   "+ radius), arc (start, middle, end), polygon (closed, optionally filled); stroke width; "
                   "answers the uuids",
                   R"json({"type":"object","required":["shape","points"],"properties":{
                        "shape":{"type":"string","enum":["line","polyline","rect","circle","arc","polygon"]},
                        "points":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"[[x_mm, y_mm], ...]"},
                        "radius_mm":{"type":"number"},
                        "layer":{"type":"string","default":"F.SilkS"},
                        "width_mm":{"type":"number","default":0.15},
                        "filled":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_graphic_add );

KOPENAPI_MARK_EDITING( "pcb_text_add", "pcb_graphic_add", "pcb_item_delete", "pcb_items_move", "pcb_text_update" );
