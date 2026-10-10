/// @file methods_pcb_silk.cpp
/// @brief kicadopenapi silkscreen layout: pcb_silk_tidy.
///
/// Reference designators go where they stay readable on the assembled board: off every
/// courtyard (bodies cover silk), off exposed copper (pads, vias), clear of other silk and inside
/// the board, as close to their own part as possible, upright (0 or 90 degrees).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <frame_type.h>
#include <kicadopenapi_glow_view.h>
#include <kiway.h>
#include <pad.h>
#include <pcb_edit_frame.h>
#include <pcb_field.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <pcb_track.h>
#include <geometry/shape_compound.h>
#include <geometry/shape_poly_set.h>
#include <tool/tool_manager.h>

#include <algorithm>
#include <cmath>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


struct SILK_GLOW_TRAITS
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


/// @brief Obstacles for silk on one side of the board
struct OBSTACLES
{
    std::vector<std::pair<BOX2I, const FOOTPRINT*>> courtyards;   ///< bodies (with their owner)
    std::vector<BOX2I>                              copper;       ///< pads, vias (exposed)
    std::vector<BOX2I>                              silk;         ///< footprint silk art, placed texts
    SHAPE_POLY_SET                                  inside;       ///< board outline shrunk by the edge clearance
};


/// @brief The text's drawn strokes (glyphs with their width), tighter than its text box
BOX2I inkBox( const PCB_FIELD* aText )
{
    std::shared_ptr<SHAPE_COMPOUND> ink = aText->GetEffectiveTextShape( false );
    return ink && !ink->Shapes().empty() ? ink->BBox() : aText->GetBoundingBox();
}


OBSTACLES collect( BOARD* aBoard, bool aBottom, int aGap, const std::set<const PCB_FIELD*>& aMoving )
{
    OBSTACLES       o;
    const PCB_LAYER_ID silk = aBottom ? B_SilkS : F_SilkS;
    const PCB_LAYER_ID crtyd = aBottom ? B_CrtYd : F_CrtYd;
    const PCB_LAYER_ID cu = aBottom ? B_Cu : F_Cu;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        const SHAPE_POLY_SET& c = fp->GetCourtyard( crtyd );

        if( c.OutlineCount() )
            o.courtyards.emplace_back( c.BBox(), fp );

        for( PAD* pad : fp->Pads() )
        {
            if( pad->IsOnLayer( cu ) )
                o.copper.push_back( pad->GetBoundingBox().GetInflated( aGap ) );
        }

        for( BOARD_ITEM* item : fp->GraphicalItems() )
        {
            if( item->IsOnLayer( silk ) )
                o.silk.push_back( item->GetBoundingBox().GetInflated( aGap ) );
        }

        for( PCB_FIELD* field : fp->GetFields() )
        {
            if( field->IsVisible() && field->IsOnLayer( silk ) && !aMoving.count( field ) )
                o.silk.push_back( inkBox( field ).GetInflated( aGap ) );
        }
    }

    for( PCB_TRACK* t : aBoard->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
            o.copper.push_back( t->GetBoundingBox().GetInflated( aGap ) );
    }

    for( BOARD_ITEM* item : aBoard->Drawings() )
    {
        if( item->IsOnLayer( silk ) )
            o.silk.push_back( item->GetBoundingBox().GetInflated( aGap ) );
    }

    if( aBoard->GetBoardPolygonOutlines( o.inside, true ) && o.inside.OutlineCount() )
        o.inside.Deflate( aBoard->GetDesignSettings().m_SilkClearance + pcbIUScale.mmToIU( 0.3 ),
                          CORNER_STRATEGY::ROUND_ALL_CORNERS, ARC_HIGH_DEF );

    return o;
}


bool insideBoard( const OBSTACLES& aObs, const BOX2I& aBox )
{
    if( !aObs.inside.OutlineCount() )
        return true;

    for( const VECTOR2I& p : { aBox.GetOrigin(), VECTOR2I( aBox.GetRight(), aBox.GetTop() ), aBox.GetEnd(),
                               VECTOR2I( aBox.GetLeft(), aBox.GetBottom() ) } )
    {
        if( !aObs.inside.Contains( p ) )
            return false;
    }

    return true;
}


nlohmann::json boxMm( const BOX2I& aBox )
{
    auto mm = []( int v ) { return std::round( v / pcbIUScale.IU_PER_MM * 100 ) / 100; };
    return { mm( aBox.GetLeft() ), mm( aBox.GetTop() ), mm( aBox.GetRight() ), mm( aBox.GetBottom() ) };
}


/// @brief Problems of a text at its current place (for the report)
/// @param aExplain receives the text box and what it hits (optional)
std::vector<std::string> problems( const OBSTACLES& aObs, const PCB_FIELD* aText, nlohmann::json* aExplain = nullptr )
{
    std::vector<std::string> out;
    const BOX2I              box = inkBox( aText );

    if( aExplain )
        ( *aExplain )["text_box_mm"] = boxMm( box );

    for( const auto& [c, owner] : aObs.courtyards )
    {
        if( c.Intersects( box ) )
        {
            out.push_back( owner == aText->GetParentFootprint() ? "on its own body" : "on another body" );

            if( aExplain )
                ( *aExplain )["body_mm"] = boxMm( c );

            break;
        }
    }

    for( const BOX2I& b : aObs.copper )
    {
        if( b.Intersects( box ) )
        {
            out.push_back( "on copper" );

            if( aExplain )
                ( *aExplain )["copper_mm"] = boxMm( b );

            break;
        }
    }

    for( const BOX2I& b : aObs.silk )
    {
        if( b.Intersects( box ) )
        {
            out.push_back( "on silk" );

            if( aExplain )
                ( *aExplain )["silk_mm"] = boxMm( b );

            break;
        }
    }

    if( !insideBoard( aObs, box ) )
        out.push_back( "off the board" );

    return out;
}

} // namespace


static KOPENAPI_RESULT h_pcb_silk_tidy( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*     board = context->GetBoard();
    const bool dryRun = aArgs.value( "dry_run", false );
    const int  gap = pcbIUScale.mmToIU( aArgs.value( "gap_mm", 0.15 ) );
    const int  reach = pcbIUScale.mmToIU( aArgs.value( "max_distance_mm", 4.0 ) );
    const bool onlyBad = aArgs.value( "only_problems", true );

    std::set<std::string> refs;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
        refs.insert( r.get<std::string>() );

    // largest parts first: they have the fewest good spots
    std::vector<FOOTPRINT*> order;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        PCB_FIELD& ref = fp->Reference();

        if( ref.IsVisible() && ref.IsOnLayer( fp->IsFlipped() ? B_SilkS : F_SilkS )
            && ( refs.empty() || refs.count( str( fp->GetReference() ) ) ) )
        {
            order.push_back( fp );
        }
    }

    std::sort( order.begin(), order.end(),
               []( FOOTPRINT* a, FOOTPRINT* b )
               { return a->GetBoundingBox( false ).GetArea() > b->GetBoundingBox( false ).GetArea(); } );

    BOARD_COMMIT      commit( context->GetToolManager() );
    nlohmann::json    moved = nlohmann::json::array();
    nlohmann::json    unresolved = nlohmann::json::array();
    std::vector<KIID> glowIds;
    int               kept = 0;

    for( bool bottom : { false, true } )
    {
        std::set<const PCB_FIELD*> moving;

        for( FOOTPRINT* fp : order )
        {
            if( fp->IsFlipped() == bottom )
                moving.insert( &fp->Reference() );
        }

        OBSTACLES obs = collect( board, bottom, gap, moving );

        for( FOOTPRINT* fp : order )
        {
            if( fp->IsFlipped() != bottom )
                continue;

            PCB_FIELD&               ref = fp->Reference();
            nlohmann::json           why;
            std::vector<std::string> before = problems( obs, &ref, aArgs.value( "explain", false ) ? &why : nullptr );

            if( onlyBad && before.empty() )
            {
                obs.silk.push_back( inkBox( &ref ).GetInflated( gap ) );
                kept++;
                continue;
            }

            const SHAPE_POLY_SET& crt = fp->GetCourtyard( bottom ? B_CrtYd : F_CrtYd );
            const BOX2I           body = crt.OutlineCount() ? crt.BBox() : fp->GetBoundingBox( false );
            const VECTOR2I        centre = body.Centre();

            // candidates: around the body, nearest first; horizontal text first, then vertical
            struct SPOT
            {
                VECTOR2I  pos;
                EDA_ANGLE angle;
                double    cost;
            };

            std::vector<SPOT> candidates;
            const VECTOR2I         startPos = ref.GetPosition();
            const EDA_ANGLE        startAngle = ref.GetTextAngle();

            for( EDA_ANGLE angle : { ANGLE_0, ANGLE_90 } )
            {
                ref.SetTextAngle( angle );
                ref.SetPosition( centre );
                const BOX2I tb = inkBox( &ref );
                const int   hw = tb.GetWidth() / 2;
                const int   hh = tb.GetHeight() / 2;

                for( int ring = 0; ring * pcbIUScale.mmToIU( 0.25 ) <= reach; ++ring )
                {
                    const int d = gap + ring * pcbIUScale.mmToIU( 0.25 );
                    const int top = body.GetTop() - d - hh;
                    const int bot = body.GetBottom() + d + hh;
                    const int lft = body.GetLeft() - d - hw;
                    const int rgt = body.GetRight() + d + hw;

                    // along each side: centre first, then shifted towards the corners
                    for( int s : { 0, -1, 1, -2, 2 } )
                    {
                        const int sx = s * body.GetWidth() / 4;
                        const int sy = s * body.GetHeight() / 4;

                        for( VECTOR2I p : { VECTOR2I( centre.x + sx, top ), VECTOR2I( centre.x + sx, bot ),
                                            VECTOR2I( lft, centre.y + sy ), VECTOR2I( rgt, centre.y + sy ) } )
                        {
                            const double cost = d + std::abs( s ) * pcbIUScale.mmToIU( 0.3 )
                                                + ( angle == ANGLE_90 ? pcbIUScale.mmToIU( 0.6 ) : 0 );
                            candidates.push_back( { p, angle, cost } );
                        }
                    }
                }
            }

            std::stable_sort( candidates.begin(), candidates.end(),
                              []( const SPOT& a, const SPOT& b ) { return a.cost < b.cost; } );

            bool placed = false;

            for( const SPOT& c : candidates )
            {
                ref.SetTextAngle( c.angle );
                ref.SetPosition( c.pos );

                if( problems( obs, &ref ).empty() )
                {
                    placed = true;
                    break;
                }
            }

            const VECTOR2I newPos = ref.GetPosition();
            const EDA_ANGLE newAngle = ref.GetTextAngle();

            // restore before committing (the commit records the old state)
            ref.SetTextAngle( startAngle );
            ref.SetPosition( startPos );

            if( !placed )
            {
                unresolved.push_back( { { "ref", str( fp->GetReference() ) }, { "problems", before } } );
                obs.silk.push_back( inkBox( &ref ).GetInflated( gap ) );
                continue;
            }

            if( !dryRun )
            {
                commit.Modify( &ref, nullptr, RECURSE_MODE::NO_RECURSE );
                ref.SetTextAngle( newAngle );
                ref.SetPosition( newPos );
                glowIds.push_back( ref.m_Uuid );
            }

            const double dist = ( newPos - startPos ).EuclideanNorm() / pcbIUScale.IU_PER_MM;
            moved.push_back( { { "ref", str( fp->GetReference() ) }, { "was", before },
                               { "moved_mm", std::round( dist * 100 ) / 100 },
                               { "angle_deg", newAngle.AsDegrees() } } );

            if( !why.is_null() )
                moved.back()["explain"] = why;

            ref.SetTextAngle( newAngle );
            ref.SetPosition( newPos );
            obs.silk.push_back( inkBox( &ref ).GetInflated( gap ) );

            if( dryRun )
            {
                ref.SetTextAngle( startAngle );
                ref.SetPosition( startPos );
            }
        }
    }

    if( !dryRun && !moved.empty() )
    {
        commit.Push( _( "Tidy silkscreen (API)" ) );

        if( !aCtx.headless )
        {
            if( auto* frame = SILK_GLOW_TRAITS::Frame( aCtx.kiway ) )
                frame->GetCanvas()->Refresh();

            KopenapiGlow<SILK_GLOW_TRAITS>( aCtx.kiway, glowIds, glowIds.size() > 1 ? 120 : 0 );
        }
    }

    return KOPENAPI_RESULT::Ok( { { "moved", moved }, { "unresolved", unresolved }, { "kept", kept },
                                  { "dry_run", dryRun } } );
}


KOPENAPI_REGISTER( "pcb_silk_tidy",
                   "Tidy the silkscreen: move reference designators that sit on bodies (courtyards), "
                   "exposed copper (pads, vias), other silk or off the board to the nearest free spot "
                   "around their part, upright (0 / 90 degrees); answers what moved and what found no "
                   "place; dry_run to preview",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"},"description":"only these parts; default all"},
                        "only_problems":{"type":"boolean","default":true,"description":"false: re-place every reference"},
                        "gap_mm":{"type":"number","default":0.15},
                        "max_distance_mm":{"type":"number","default":4.0},
                        "dry_run":{"type":"boolean","default":false},
                        "explain":{"type":"boolean","default":false,"description":"answer the text box and what it hits (mm)"}}})json"_json,
                   false, h_pcb_silk_tidy, 120 );

KOPENAPI_MARK_EDITING( "pcb_silk_tidy" );
