/// @file methods_sch_view.cpp
/// @brief kicadopenapi schematic view: sch_view_zoom (fit the drawing, an area, symbols; smooth).
#include "kopenapi_sch.h"

#include <frame_type.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_gal_render.h>
#include <kicadopenapi_steadycam.h>
#include <kicadopenapi_view_motion.h>
#include <kiway.h>
#include <sch_edit_frame.h>
#include <sch_screen.h>
#include <sch_symbol.h>

#include <cmath>


static KOPENAPI_RESULT h_sch_view_zoom( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;

    if( !frame || !frame->GetCanvas() || !frame->IsShown() || !frame->GetScreen() )
        return KOPENAPI_RESULT::Error( 409, "no schematic editor window open" );

    KOPENAPI_REMOTE_GUARD remote( frame );
    SCH_SCREEN*  screen = frame->GetScreen();
    const double margin = schIUScale.mmToIU( aArgs.value( "margin_mm", 5.0 ) );
    BOX2I        area;
    bool         any = false;

    if( aArgs.contains( "area_mm" ) )
    {
        const nlohmann::json& a = aArgs["area_mm"];

        if( !a.is_array() || a.size() != 4 )
            return KOPENAPI_RESULT::Error( 400, "area_mm: [x0, y0, x1, y1]" );

        const VECTOR2I p0( schIUScale.mmToIU( a[0].get<double>() ), schIUScale.mmToIU( a[1].get<double>() ) );
        const VECTOR2I p1( schIUScale.mmToIU( a[2].get<double>() ), schIUScale.mmToIU( a[3].get<double>() ) );
        area = BOX2I( p0, p1 - p0 );
        area.Normalize();
        any = true;
    }
    else
    {
        // refs: those symbols; otherwise everything drawn on the sheet (not the empty paper)
        std::set<std::string> refs;

        for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
            refs.insert( r.get<std::string>() );

        for( SCH_ITEM* item : screen->Items() )
        {
            if( !refs.empty() )
            {
                if( item->Type() != SCH_SYMBOL_T )
                    continue;

                auto* symbol = static_cast<SCH_SYMBOL*>( item );

                if( !refs.count( symbol->GetRef( &frame->GetCurrentSheet() ).ToStdString( wxConvUTF8 ) ) )
                    continue;
            }

            const BOX2I b = item->GetBoundingBox();
            area = any ? area.Merge( b ) : b;
            any = true;
        }
    }

    if( !any )
        return KOPENAPI_RESULT::Error( 404, aArgs.contains( "refs" ) ? "none of refs is on this sheet" : "the sheet is empty" );

    area.Inflate( margin );
    KopenapiMoveView( frame->GetCanvas(), BOX2D( area.GetOrigin(), area.GetSize() ),
                      std::clamp( aArgs.value( "animate_ms", 600 ), 0, 5000 ) );

    const BOX2D v = frame->GetCanvas()->GetView()->GetViewport();
    auto        mm = []( double x ) { return std::round( x / schIUScale.IU_PER_MM * 100 ) / 100; };
    return KOPENAPI_RESULT::Ok( { { "viewport_mm", { mm( v.GetLeft() ), mm( v.GetTop() ), mm( v.GetRight() ), mm( v.GetBottom() ) } } } );
}


/// @brief Recordings' steadycam for the schematic canvas
KOPENAPI_REGISTER_CANVAS_STEADY( KopenapiSteadyGal );

/// @brief Recordings: the schematic canvas at the video's size (composed frame scaled on the GPU)
KOPENAPI_REGISTER_CANVAS_RENDER( KopenapiRenderGal );


KOPENAPI_REGISTER( "sch_view_zoom",
                   "Move the schematic editor's view (camera, zoom, pan): fit what is drawn on the sheet, an "
                   "area in mm, or symbols by reference; margin; smooth by default (animate_ms, 0 jumps); "
                   "manual control paused for the move; GUI only",
                   R"json({"type":"object","properties":{
                        "area_mm":{"type":"array","items":{"type":"number"},"description":"[x0, y0, x1, y1]"},
                        "refs":{"type":"array","items":{"type":"string"}},
                        "margin_mm":{"type":"number","default":5},
                        "animate_ms":{"type":"integer","default":600,"maximum":5000}}})json"_json,
                   true, h_sch_view_zoom, 300 );   // animations record on video time: slow while recording
