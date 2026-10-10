/// @file methods_pcb_silk.cpp
/// @brief kicadopenapi silkscreen layout: pcb_silk_tidy, pcb_free_area, pcb_silk_fit.
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
#include <functional>
#include <optional>
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
            {
                frame->GetCanvas()->Refresh();
                KopenapiRefresh3D( frame );
            }

            KopenapiGlow<SILK_GLOW_TRAITS>( aCtx.kiway, glowIds, glowIds.size() > 1 ? 120 : 0 );
        }
    }

    return KOPENAPI_RESULT::Ok( { { "moved", moved }, { "unresolved", unresolved }, { "kept", kept },
                                  { "dry_run", dryRun } } );
}


namespace
{

bool boxInside( const SHAPE_POLY_SET& aPoly, const BOX2I& aBox );


struct FREE_SPOT
{
    BOX2I  box;
    double score;
    double room;
};


/// @brief Distinct free places of aW x aH on one side: clear of bodies, exposed copper, silk and
/// the board edge (each kept aGap away); best first (most room, or nearest to aNear)
std::vector<FREE_SPOT> findFree( BOARD* aBoard, bool aBottom, int aGap, int aW, int aH, const std::optional<VECTOR2I>& aNear,
                                 int aCount, int aStep, const SHAPE_POLY_SET* aRegion = nullptr, bool aIgnoreObstacles = false,
                                 bool aAvoidTracks = true )
{
    const OBSTACLES    obs = collect( aBoard, aBottom, aGap, {} );
    std::vector<BOX2I> blocked = obs.copper;
    blocked.insert( blocked.end(), obs.silk.begin(), obs.silk.end() );

    for( const auto& [c, owner] : obs.courtyards )
        blocked.push_back( c.GetInflated( aGap ) );

    // silk may cross tracks under the mask, but art reads better clear of them
    if( aAvoidTracks )
    {
        const PCB_LAYER_ID cu = aBottom ? B_Cu : F_Cu;

        for( PCB_TRACK* t : aBoard->Tracks() )
        {
            if( t->Type() != PCB_VIA_T && t->IsOnLayer( cu ) )
                blocked.push_back( t->GetBoundingBox().GetInflated( aGap ) );
        }
    }

    if( aIgnoreObstacles )
        blocked.clear();

    BOX2I area = aRegion ? aRegion->BBox() : aBoard->GetBoardEdgesBoundingBox();

    // a rectangular region needs no polygon test per candidate
    const bool regionIsRect = aRegion && aRegion->OutlineCount() == 1 && aRegion->COutline( 0 ).PointCount() == 4
                              && std::abs( SHAPE_POLY_SET( *aRegion ).Area() - double( area.GetWidth() ) * area.GetHeight() ) < 1.0;
    const int   cap = pcbIUScale.mmToIU( 10.0 );

    // room around a free box: distance to the nearest obstacle or the board edge (capped)
    auto room = [&]( const BOX2I& aBox )
    {
        double best = cap;

        for( const BOX2I& b : blocked )
        {
            const int dx = std::max( { b.GetLeft() - aBox.GetRight(), aBox.GetLeft() - b.GetRight(), 0 } );
            const int dy = std::max( { b.GetTop() - aBox.GetBottom(), aBox.GetTop() - b.GetBottom(), 0 } );
            best = std::min( best, std::hypot( double( dx ), double( dy ) ) );
        }

        if( obs.inside.OutlineCount() )
        {
            for( const VECTOR2I& p : { aBox.GetOrigin(), VECTOR2I( aBox.GetRight(), aBox.GetTop() ), aBox.GetEnd(),
                                       VECTOR2I( aBox.GetLeft(), aBox.GetBottom() ) } )
            {
                best = std::min( best, std::sqrt( double( obs.inside.SquaredDistance( p ) ) ) );
            }
        }

        return best;
    };

    std::vector<FREE_SPOT> spots;

    if( area.GetWidth() <= 0 || aW <= 0 || aH <= 0 )
        return spots;

    for( int y = area.GetTop(); y + aH <= area.GetBottom(); y += aStep )
    {
        for( int x = area.GetLeft(); x + aW <= area.GetRight(); x += aStep )
        {
            const BOX2I box( VECTOR2I( x, y ), VECTOR2I( aW, aH ) );

            if( ( !aRegion && !insideBoard( obs, box ) )
                || std::any_of( blocked.begin(), blocked.end(), [&]( const BOX2I& b ) { return b.Intersects( box ); } )
                || ( aRegion && !regionIsRect && !boxInside( *aRegion, box ) ) )
            {
                continue;
            }

            const double r = room( box );
            spots.push_back( { box, aNear ? -( box.Centre() - *aNear ).EuclideanNorm() : r, r } );
        }
    }

    std::sort( spots.begin(), spots.end(), []( const FREE_SPOT& a, const FREE_SPOT& b ) { return a.score > b.score; } );

    // a candidate overlapping a better one is the same place
    std::vector<FREE_SPOT> out;

    for( const FREE_SPOT& s : spots )
    {
        if( (int) out.size() >= aCount )
            break;

        if( std::none_of( out.begin(), out.end(), [&]( const FREE_SPOT& o ) { return o.box.Intersects( s.box ); } ) )
            out.push_back( s );
    }

    return out;
}


double mmOf( int aIu )
{
    return std::round( aIu / pcbIUScale.IU_PER_MM * 100 ) / 100;
}


std::optional<VECTOR2I> pointArg( const nlohmann::json& aArgs, const char* aKey )
{
    if( !aArgs.contains( aKey ) || !aArgs[aKey].is_array() || aArgs[aKey].size() != 2 )
        return std::nullopt;

    return VECTOR2I( pcbIUScale.mmToIU( aArgs[aKey][0].get<double>() ), pcbIUScale.mmToIU( aArgs[aKey][1].get<double>() ) );
}


/// @brief Is the box fully inside the polygon
bool boxInside( const SHAPE_POLY_SET& aPoly, const BOX2I& aBox )
{
    SHAPE_POLY_SET rect;
    rect.NewOutline();
    rect.Append( aBox.GetLeft(), aBox.GetTop() );
    rect.Append( aBox.GetRight(), aBox.GetTop() );
    rect.Append( aBox.GetRight(), aBox.GetBottom() );
    rect.Append( aBox.GetLeft(), aBox.GetBottom() );
    rect.BooleanSubtract( aPoly );
    return rect.Area() < 1.0;
}

} // namespace


static KOPENAPI_RESULT h_pcb_free_area( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*       board = context->GetBoard();
    const double wMm = aArgs.value( "w_mm", 0.0 );
    const double hMm = aArgs.value( "h_mm", 0.0 );

    if( wMm <= 0 || hMm <= 0 )
        return KOPENAPI_RESULT::Error( 400, "w_mm and h_mm: the size wanted" );

    if( board->GetBoardEdgesBoundingBox().GetWidth() <= 0 )
        return KOPENAPI_RESULT::Error( 409, "the board has no outline" );

    const auto iu = []( double aMm ) { return pcbIUScale.mmToIU( aMm ); };
    const bool bottom = aArgs.value( "side", std::string( "front" ) ) == "back";

    std::vector<FREE_SPOT> spots = findFree( board, bottom, iu( std::max( 0.0, aArgs.value( "gap_mm", 0.5 ) ) ), iu( wMm ),
                                             iu( hMm ), pointArg( aArgs, "near_mm" ), std::clamp( aArgs.value( "count", 5 ), 1, 50 ),
                                             iu( std::clamp( aArgs.value( "step_mm", 0.5 ), 0.1, 5.0 ) ), nullptr, false,
                                             aArgs.value( "avoid_tracks", true ) );
    nlohmann::json out = nlohmann::json::array();

    for( const FREE_SPOT& s : spots )
    {
        out.push_back( { { "box_mm", boxMm( s.box ) },
                         { "center_mm", { mmOf( s.box.Centre().x ), mmOf( s.box.Centre().y ) } },
                         { "room_mm", mmOf( int( s.room ) ) } } );
    }

    return KOPENAPI_RESULT::Ok( { { "areas", out }, { "side", bottom ? "back" : "front" },
                                  { "searched_mm", boxMm( board->GetBoardEdgesBoundingBox() ) } } );
}


static KOPENAPI_RESULT h_pcb_silk_fit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                       board = context->GetBoard();
    const BOARD_DESIGN_SETTINGS& ds = board->GetDesignSettings();
    const auto                   iu = []( double aMm ) { return pcbIUScale.mmToIU( aMm ); };
    const bool                   bottom = aArgs.value( "side", std::string( "front" ) ) == "back";
    const std::string            text = aArgs.value( "text", std::string() );
    const bool                   isText = !text.empty();

    // the item at the asked size: its ink box (strokes included) and where that sits relative to
    // the item's position
    double size = aArgs.value( "size_mm", 1.5 );
    double width = aArgs.value( "width_mm", size );
    double thickness = aArgs.value( "thickness_mm", size * 0.15 );
    BOX2I  ink;

    if( isText )
    {
        PCB_TEXT probe( board );
        probe.SetText( wxString::FromUTF8( text ) );
        probe.SetLayer( bottom ? B_SilkS : F_SilkS );
        probe.SetTextSize( VECTOR2I( iu( width ), iu( size ) ) );
        probe.SetTextThickness( iu( thickness ) );
        probe.SetBold( aArgs.value( "bold", false ) );
        probe.SetItalic( aArgs.value( "italic", false ) );
        probe.SetTextAngle( EDA_ANGLE( aArgs.value( "angle_deg", 0.0 ), DEGREES_T ) );
        probe.SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
        probe.SetVertJustify( GR_TEXT_V_ALIGN_CENTER );
        probe.SetMirrored( bottom );
        probe.SetPosition( VECTOR2I( 0, 0 ) );
        std::shared_ptr<SHAPE_COMPOUND> shape = probe.GetEffectiveTextShape( false );
        ink = shape && !shape->Shapes().empty() ? shape->BBox() : probe.GetBoundingBox();
    }
    else if( aArgs.contains( "art_mm" ) && aArgs["art_mm"].is_array() && aArgs["art_mm"].size() == 2 )
    {
        const int w = iu( aArgs["art_mm"][0].get<double>() ), h = iu( aArgs["art_mm"][1].get<double>() );
        ink = BOX2I( VECTOR2I( -w / 2, -h / 2 ), VECTOR2I( w, h ) );
        thickness = aArgs.value( "line_width_mm", 0.15 );
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "give text (with its size) or art_mm [w, h] with line_width_mm" );
    }

    if( ink.GetWidth() <= 0 || ink.GetHeight() <= 0 )
        return KOPENAPI_RESULT::Error( 400, "nothing to measure" );

    // the silkscreen's resolution: the board's minimum text height and stroke (DRC), or stricter
    const double minHeight = std::max( mmOf( ds.m_MinSilkTextHeight ), aArgs.value( "min_text_height_mm", 0.0 ) );
    const double minStroke = std::max( mmOf( ds.m_MinSilkTextThickness ), aArgs.value( "min_stroke_mm", 0.0 ) );
    double       minScale = thickness > 0 ? minStroke / thickness : 0.0;

    if( isText )
        minScale = std::max( { minScale, minHeight / size, minHeight / width } );

    const double maxScale = std::max( aArgs.value( "max_scale", 1.0 ), 1e-3 );
    const int    gap = iu( std::max( 0.0, aArgs.value( "gap_mm", 0.5 ) ) );

    auto scaled = [&]( double aS ) { return VECTOR2I( int( ink.GetWidth() * aS ), int( ink.GetHeight() * aS ) ); };

    // the best scale in [minScale, maxScale] for which aFits finds a place (bisection; larger is better)
    double                 chosen = 0;
    std::optional<BOX2I>   place;
    nlohmann::json         candidates = nlohmann::json::array();
    std::string            target;

    auto search = [&]( const std::function<std::optional<BOX2I>( double )>& aFits )
    {
        if( minScale > maxScale )
            return;

        if( auto at = aFits( maxScale ) )
        {
            chosen = maxScale;
            place = at;
            return;
        }

        double lo = minScale, hi = maxScale;
        auto   atLo = aFits( lo );

        if( !atLo )
            return;

        place = atLo;

        for( int i = 0; i < 14 && hi - lo > 0.005 * hi; ++i )
        {
            const double mid = ( lo + hi ) / 2;

            if( auto at = aFits( mid ) )
            {
                lo = mid;
                place = at;
            }
            else
            {
                hi = mid;
            }
        }

        chosen = lo;
    };

    // the region to fit into: a box, a polygon or the whole board; obstacles inside it are avoided
    SHAPE_POLY_SET region;
    const bool     hasBox = aArgs.contains( "box_mm" ) && aArgs["box_mm"].is_array() && aArgs["box_mm"].size() == 4;
    const bool     hasPoly = aArgs.contains( "polygon_mm" ) && aArgs["polygon_mm"].is_array() && aArgs["polygon_mm"].size() >= 3;

    if( hasBox )
    {
        target = "box";
        const auto& b = aArgs["box_mm"];
        region.NewOutline();
        region.Append( iu( b[0].get<double>() ), iu( b[1].get<double>() ) );
        region.Append( iu( b[2].get<double>() ), iu( b[1].get<double>() ) );
        region.Append( iu( b[2].get<double>() ), iu( b[3].get<double>() ) );
        region.Append( iu( b[0].get<double>() ), iu( b[3].get<double>() ) );
    }
    else if( hasPoly )
    {
        target = "polygon";
        region.NewOutline();

        for( const nlohmann::json& p : aArgs["polygon_mm"] )
            region.Append( iu( p[0].get<double>() ), iu( p[1].get<double>() ) );
    }
    else
    {
        target = "free";
    }

    // in a region: centred in it unless near_mm says otherwise
    std::optional<VECTOR2I> near = pointArg( aArgs, "near_mm" );

    if( !near && region.OutlineCount() )
        near = region.BBox().Centre();

    const bool ignore = aArgs.value( "ignore_obstacles", false );
    const int  stepMm = iu( std::clamp( aArgs.value( "step_mm", region.OutlineCount() ? 0.1 : 0.5 ), 0.05, 5.0 ) );

    search( [&]( double aS ) -> std::optional<BOX2I>
            {
                const VECTOR2I               sz = scaled( aS );
                const std::vector<FREE_SPOT> f = findFree( board, bottom, gap, sz.x, sz.y, near, 1, stepMm,
                                                           region.OutlineCount() ? &region : nullptr, ignore,
                                                           aArgs.value( "avoid_tracks", true ) );
                return f.empty() ? std::nullopt : std::optional<BOX2I>( f.front().box );
            } );

    const auto     r3 = []( double v ) { return std::round( v * 1000 ) / 1000; };
    nlohmann::json answer = { { "target", target },
                              { "ink_mm", { mmOf( ink.GetWidth() ), mmOf( ink.GetHeight() ) } },
                              { "min_scale", std::round( minScale * 1000 ) / 1000 },
                              { "silk_minimums", { { "text_height_mm", minHeight }, { "stroke_mm", minStroke } } },
                              { "fits", place.has_value() } };

    if( isText )
        answer["min_size_mm"] = r3( size * minScale );

    if( !place )
    {
        answer["why"] = minScale > maxScale ? "the asked size is below the silkscreen's resolution (min_scale > max_scale)"
                                            : "no place even at the smallest size the silkscreen resolves (min_scale)";
        return KOPENAPI_RESULT::Ok( answer );
    }

    // the item's position: the place's centre less the ink's offset from the item's anchor
    const VECTOR2I inkOffset( int( ink.Centre().x * chosen ), int( ink.Centre().y * chosen ) );
    const VECTOR2I pos = place->Centre() - inkOffset;
    answer["scale"] = r3( chosen );
    answer["box_mm"] = boxMm( *place );
    answer["center_mm"] = { mmOf( pos.x ), mmOf( pos.y ) };

    if( isText )
    {
        nlohmann::json add = { { "text", text }, { "layer", bottom ? "B.SilkS" : "F.SilkS" },
                               { "x_mm", mmOf( pos.x ) }, { "y_mm", mmOf( pos.y ) },
                               { "size_mm", r3( size * chosen ) }, { "width_mm", r3( width * chosen ) },
                               { "thickness_mm", r3( thickness * chosen ) } };

        for( const char* key : { "bold", "italic", "angle_deg" } )
        {
            if( aArgs.contains( key ) )
                add[key] = aArgs[key];
        }

        answer["text_add_args"] = add;   // ready for pcb_text_add
    }
    else
    {
        answer["line_width_mm"] = r3( thickness * chosen );
    }

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_free_area",
                   "Find free places of a given size on the board for silkscreen art, logos, labels: "
                   "clear of bodies (courtyards), exposed copper (pads, vias), existing silk and the board "
                   "edge on one side; best first: the most room around, or nearest to near_mm; distinct "
                   "places (no overlaps between answers)",
                   R"json({"type":"object","required":["w_mm","h_mm"],"properties":{
                        "w_mm":{"type":"number"},
                        "h_mm":{"type":"number"},
                        "side":{"type":"string","enum":["front","back"],"default":"front"},
                        "gap_mm":{"type":"number","default":0.5,"description":"clearance kept to every obstacle"},
                        "near_mm":{"type":"array","items":{"type":"number"},"description":"[x, y]: prefer places closest to this point"},
                        "count":{"type":"integer","default":5,"maximum":50},
                        "avoid_tracks":{"type":"boolean","default":true,"description":"keep clear of tracks on that side too"},
                        "step_mm":{"type":"number","default":0.5,"description":"search grid"}}})json"_json,
                   false, h_pcb_free_area, 60 );

KOPENAPI_REGISTER( "pcb_silk_fit",
                   "Will it fit on the silkscreen, and at what size: measures a text (its drawn strokes) or "
                   "an art box, then finds the largest scale (up to max_scale, never below the "
                   "silkscreen's resolution: the board's minimum silk text height and stroke) at which it "
                   "fits into box_mm, into polygon_mm, or (default) anywhere on the board, always clear of "
                   "bodies, exposed copper, other silk and the edge; answers fits, scale, min_scale, the place and, for "
                   "text, ready pcb_text_add arguments",
                   R"json({"type":"object","properties":{
                        "text":{"type":"string"},
                        "size_mm":{"type":"number","default":1.5},
                        "width_mm":{"type":"number","description":"glyph width; default the height"},
                        "thickness_mm":{"type":"number","description":"default 15 % of the height"},
                        "bold":{"type":"boolean"},"italic":{"type":"boolean"},"angle_deg":{"type":"number"},
                        "art_mm":{"type":"array","items":{"type":"number"},"description":"[w, h] of graphics instead of a text"},
                        "line_width_mm":{"type":"number","default":0.15,"description":"art: its stroke (scales with it)"},
                        "box_mm":{"type":"array","items":{"type":"number"},"description":"[x0, y0, x1, y1] to fit into"},
                        "polygon_mm":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"[[x, y], ...] to fit into"},
                        "near_mm":{"type":"array","items":{"type":"number"},"description":"free place: prefer near [x, y]"},
                        "side":{"type":"string","enum":["front","back"],"default":"front"},
                        "gap_mm":{"type":"number","default":0.5},
                        "max_scale":{"type":"number","default":1,"description":"> 1 lets it grow to fill the target"},
                        "min_text_height_mm":{"type":"number","description":"stricter than the board's rule (e.g. the fab's)"},
                        "min_stroke_mm":{"type":"number"},
                        "avoid_tracks":{"type":"boolean","default":true,"description":"keep clear of tracks on that side too (legal under the mask, but art reads better)"},
                        "ignore_obstacles":{"type":"boolean","default":false,"description":"box / polygon: fit the region only, not the parts inside it"},
                        "step_mm":{"type":"number","description":"search grid; default 0.1 in a box / polygon, 0.5 on the board"}}})json"_json,
                   false, h_pcb_silk_fit, 120 );

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
