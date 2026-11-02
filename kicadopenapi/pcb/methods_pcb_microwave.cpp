/// @file methods_pcb_microwave.cpp
/// @brief pcb_microwave_add: the board editor's RF tools (gap, stub, arc stub, polygonal shape, S-shaped inductor)
/// through the same core functions the microwave tool uses.

#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <frame_type.h>
#include <kiway.h>
#include <microwave/microwave_core.h>
#include <pad.h>
#include <pcb_edit_frame.h>

#include <memory>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


int iu( double aMm )
{
    return pcbIUScale.mmToIU( aMm );
}


std::optional<VECTOR2I> pointArg( const nlohmann::json& aArgs, const char* aKey )
{
    if( !aArgs.contains( aKey ) )
        return std::nullopt;

    const nlohmann::json& p = aArgs[aKey];

    if( !p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number() )
        return std::nullopt;

    return VECTOR2I( iu( p[0].get<double>() ), iu( p[1].get<double>() ) );
}


KOPENAPI_RESULT h_pcb_microwave_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                     board = context->GetBoard();
    const std::string          kind = aArgs.value( "kind", std::string() );
    std::unique_ptr<FOOTPRINT> footprint;
    wxString                   error;
    const int                  trackWidth = board->GetDesignSettings().GetCurrentTrackWidth();

    try
    {
        if( kind == "gap" || kind == "stub" || kind == "arc_stub" )
        {
            MICROWAVE_FOOTPRINT_SHAPE shape = kind == "gap"    ? MICROWAVE_FOOTPRINT_SHAPE::GAP
                                              : kind == "stub" ? MICROWAVE_FOOTPRINT_SHAPE::STUB
                                                               : MICROWAVE_FOOTPRINT_SHAPE::STUB_ARC;

            // the editor proposes the current track width
            int size = aArgs.contains( "size" ) ? iu( aArgs["size"].get<double>() ) : trackWidth;

            footprint.reset( CreateMicrowaveFootprint( board, shape, size,
                                                       EDA_ANGLE( aArgs.value( "angle", 0.0 ), DEGREES_T ),
                                                       &error ) );
        }
        else if( kind == "inductor" )
        {
            std::optional<VECTOR2I> start = pointArg( aArgs, "start" ), end = pointArg( aArgs, "end" );

            if( !start || !end )
                return KOPENAPI_RESULT::Error( 400, "inductor: start and end [x_mm, y_mm] (the two pads)" );

            MICROWAVE_INDUCTOR_PATTERN pattern;
            pattern.m_Start = *start;
            pattern.m_End = *end;
            pattern.m_Width = aArgs.contains( "width" ) ? iu( aArgs["width"].get<double>() ) : trackWidth;
            pattern.m_Length = aArgs.contains( "length" ) ? iu( aArgs["length"].get<double>() )
                                                          : ( *end - *start ).EuclideanNorm();

            footprint.reset( CreateMicrowaveInductor( board, pattern,
                                                      wxString::FromUTF8( aArgs.value( "value", std::string( "L" ) ) ),
                                                      &error ) );
        }
        else if( kind == "polygon" )
        {
            MICROWAVE_SHAPE_FILE shape;

            if( aArgs.contains( "shape_file" ) )
            {
                if( !ReadMicrowaveShapeFile( wxString::FromUTF8( aArgs["shape_file"].get<std::string>() ), shape,
                                             &error ) )
                {
                    return KOPENAPI_RESULT::Error( 422, str( error ) );
                }
            }
            else if( aArgs.contains( "points" ) && aArgs["points"].is_array() )
            {
                for( const nlohmann::json& p : aArgs["points"] )
                    shape.points.emplace_back( p.at( 0 ).get<double>(), p.at( 1 ).get<double>() );
            }
            else
            {
                return KOPENAPI_RESULT::Error( 400, "polygon: shape_file (the editor's shape description file) or "
                                                    "points [[x, y]] normalized 0..1" );
            }

            // the editor fills the size from the file's scale
            VECTOR2I size( KiROUND( shape.scale.x ), KiROUND( shape.scale.y ) );

            if( aArgs.contains( "size" ) )
            {
                std::optional<VECTOR2I> s = pointArg( aArgs, "size" );

                if( !s )
                    return KOPENAPI_RESULT::Error( 400, "size: [x_mm, y_mm]" );

                size = *s;
            }

            const std::string mode = aArgs.value( "mode", std::string( "normal" ) );
            MICROWAVE_POLYGON_MODE polyMode = mode == "symmetrical" ? MICROWAVE_POLYGON_MODE::SYMMETRICAL
                                              : mode == "mirrored"  ? MICROWAVE_POLYGON_MODE::MIRRORED
                                                                    : MICROWAVE_POLYGON_MODE::NORMAL;

            if( mode != "normal" && polyMode == MICROWAVE_POLYGON_MODE::NORMAL )
                return KOPENAPI_RESULT::Error( 400, "mode: normal | symmetrical | mirrored" );

            footprint.reset( CreateMicrowavePolygon( board, shape.points, size, polyMode, &error ) );
        }
        else
        {
            return KOPENAPI_RESULT::Error( 400, "kind: gap | stub | arc_stub | polygon | inductor" );
        }
    }
    catch( const nlohmann::json::exception& e )
    {
        return KOPENAPI_RESULT::Error( 400, std::string( "bad argument: " ) + e.what() );
    }

    if( !footprint )
        return KOPENAPI_RESULT::Error( 422, str( error ) );

    if( kind != "inductor" )
    {
        std::optional<VECTOR2I> at = pointArg( aArgs, "at" );

        if( !at )
            return KOPENAPI_RESULT::Error( 400, "at: [x_mm, y_mm] where to place it" );

        footprint->SetPosition( *at );
    }

    if( aArgs.contains( "rotation" ) )
        footprint->SetOrientation( EDA_ANGLE( aArgs["rotation"].get<double>(), DEGREES_T ) );

    if( aArgs.value( "side", std::string( "front" ) ) == "back" )
        footprint->Flip( footprint->GetPosition(), FLIP_DIRECTION::TOP_BOTTOM );

    FOOTPRINT*     added = footprint.get();
    nlohmann::json pads = nlohmann::json::array();

    for( PAD* pad : added->Pads() )
    {
        pads.push_back( { { "number", str( pad->GetNumber() ) },
                          { "at", { pcbIUScale.IUTomm( pad->GetPosition().x ), pcbIUScale.IUTomm( pad->GetPosition().y ) } },
                          { "size", { pcbIUScale.IUTomm( pad->GetSize( PADSTACK::ALL_LAYERS ).x ),
                                      pcbIUScale.IUTomm( pad->GetSize( PADSTACK::ALL_LAYERS ).y ) } } } );
    }

    BOARD_COMMIT commit( context->GetToolManager() );
    commit.Add( footprint.release() );
    commit.Push( kind == "inductor" ? _( "Add Microwave Inductor (API)" ) : _( "Place microwave feature (API)" ) );

    if( !aCtx.headless && aCtx.kiway )
    {
        if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
            frame->GetCanvas()->Refresh();
    }

    return KOPENAPI_RESULT::Ok( { { "uuid", str( added->m_Uuid.AsString() ) },
                                  { "reference", str( added->GetReference() ) },
                                  { "value", str( added->GetValue() ) },
                                  { "footprint", str( added->GetFPID().GetLibItemName().wx_str() ) },
                                  { "at", { pcbIUScale.IUTomm( added->GetPosition().x ),
                                            pcbIUScale.IUTomm( added->GetPosition().y ) } },
                                  { "pads", pads },
                                  { "graphics", added->GraphicalItems().size() } } );
}

} // namespace


KOPENAPI_REGISTER( "pcb_microwave_add",
                   "The board editor's RF (microwave) tools, as footprints added in one undo step. kind gap: two "
                   "pads size mm apart; stub: pad 2 size mm long; arc_stub: a pad shaped as an arc sector of radius "
                   "size and angle deg (0..180) - size defaults to the current track width, pads are the track "
                   "width; polygon: a filled F.Cu shape between two pads from shape_file (the editor's shape "
                   "description file: Unit, XScale, YScale, $COORD x y normalized 0..1 $ENDCOORD) or points, "
                   "scaled to size [x_mm, y_mm] (default the file's scale), mode normal | symmetrical | mirrored; "
                   "inductor: an S-shaped coil of track width between start and end (its pads 2 and 1), length mm "
                   "(at least their distance), value (also the footprint name). at [x_mm, y_mm] places all but the "
                   "inductor; rotation deg, side front | back. Errors (a length that cannot be built, no points, "
                   "null size) come back 422 with the editor's message",
                   R"json({"type":"object","required":["kind"],"properties":{
                        "kind":{"type":"string","enum":["gap","stub","arc_stub","polygon","inductor"]},
                        "at":{"type":"array","items":{"type":"number"}},
                        "size":{"description":"mm (gap, stub, arc_stub) or [x_mm, y_mm] (polygon)"},
                        "angle":{"type":"number","description":"arc_stub sweep, degrees"},
                        "shape_file":{"type":"string"},
                        "points":{"type":"array","items":{"type":"array","items":{"type":"number"}}},
                        "mode":{"type":"string","enum":["normal","symmetrical","mirrored"],"default":"normal"},
                        "start":{"type":"array","items":{"type":"number"}},
                        "end":{"type":"array","items":{"type":"number"}},
                        "length":{"type":"number"},
                        "width":{"type":"number"},
                        "value":{"type":"string","default":"L"},
                        "rotation":{"type":"number"},
                        "side":{"type":"string","enum":["front","back"],"default":"front"}}})json"_json,
                   false, h_pcb_microwave_add );

KOPENAPI_MARK_EDITING( "pcb_microwave_add" );
