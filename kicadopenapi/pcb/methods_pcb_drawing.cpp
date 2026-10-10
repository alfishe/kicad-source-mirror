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
            frame->GetCanvas()->Refresh();

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

    const BOX2I box = item->GetBoundingBox();
    auto        mm = []( int v ) { return std::round( v / pcbIUScale.IU_PER_MM * 100 ) / 100; };

    return KOPENAPI_RESULT::Ok( { { "uuid", str( item->m_Uuid.AsString() ) },
                                  { "layer", str( board->GetLayerName( *layer ) ) },
                                  { "bbox_mm", { mm( box.GetLeft() ), mm( box.GetTop() ), mm( box.GetRight() ),
                                                 mm( box.GetBottom() ) } } } );
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


KOPENAPI_REGISTER( "pcb_text_add",
                   "Put text on the board (silkscreen title, labels, pinouts, fab notes) on any layer "
                   "(default F.SilkS): position, size, thickness, bold, italic, angle, justify, knockout; "
                   "mirrored automatically on back layers; answers uuid and bounding box",
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

KOPENAPI_MARK_EDITING( "pcb_text_add", "pcb_graphic_add" );
