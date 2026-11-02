/// @file methods_pcb_align.cpp
/// @brief kicadopenapi pcb_items_align: the editor's Align / Distribute (top, bottom, left, right,
/// centres; distribute by centres or gaps) on footprints, pads (their footprints move), tracks,
/// vias, drawings, zones and groups.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <geometry/distribute.h>
#include <kicadopenapi_util.h>
#include <pad.h>
#include <pcb_group.h>
#include <tool/tool_manager.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


/// @brief What moves (a pad's footprint) and the box that is aligned (the pad's own, as the editor)
struct ENTRY
{
    BOARD_ITEM* item;
    BOX2I       box;
    std::string spec;
};


BOX2I itemBox( BOARD_ITEM* aItem )
{
    if( aItem->Type() == PCB_FOOTPRINT_T )
        return static_cast<FOOTPRINT*>( aItem )->GetBoundingBox( false );

    return aItem->GetBoundingBox();
}


nlohmann::json boxJson( const BOX2I& aBox )
{
    return { toMm( aBox.GetLeft() ), toMm( aBox.GetTop() ), toMm( aBox.GetRight() ), toMm( aBox.GetBottom() ) };
}


KOPENAPI_RESULT h_pcb_items_align( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string align = aArgs.value( "align", std::string() );
    const std::string distribute = aArgs.value( "distribute", std::string() );
    const std::string by = aArgs.value( "by", std::string( "centers" ) );
    const bool        overrideLocks = aArgs.value( "override_locks", false );

    static const std::vector<std::string> aligns = { "top", "bottom", "left", "right", "center_x", "center_y" };

    if( align.empty() == distribute.empty() )
        return KOPENAPI_RESULT::Error( 400, "one of align (top, bottom, left, right, center_x, center_y) or distribute "
                                            "(horizontal, vertical)" );

    if( !align.empty() && std::find( aligns.begin(), aligns.end(), align ) == aligns.end() )
        return KOPENAPI_RESULT::Error( 400, "align: top, bottom, left, right, center_x or center_y" );

    if( !distribute.empty() && distribute != "horizontal" && distribute != "vertical" )
        return KOPENAPI_RESULT::Error( 400, "distribute: horizontal or vertical" );

    if( by != "centers" && by != "gaps" )
        return KOPENAPI_RESULT::Error( 400, "by: centers or gaps (equal spaces between the boxes)" );

    if( !aArgs.contains( "items" ) || !aArgs["items"].is_array() )
        return KOPENAPI_RESULT::Error( 400, "items: REF, REF.PAD (its footprint moves) or uuids of tracks, vias, "
                                            "drawings, zones, groups" );

    auto resolve = [&]( const nlohmann::json& aSpec, ENTRY& aOut, std::string& aError ) -> bool
    {
        BOARD_ITEM* item = aSpec.is_string() ? KopenapiFindBoardItem( board, aSpec.get<std::string>() ) : nullptr;

        if( !item )
        {
            aError = "no item " + aSpec.dump() + " on the board";
            return false;
        }

        aOut.spec = aSpec.get<std::string>();
        aOut.box = itemBox( item );

        const bool pad = item->Type() == PCB_PAD_T;

        if( item->GetParentFootprint() )
            item = item->GetParentFootprint();

        if( !pad && item->Type() == PCB_FOOTPRINT_T )
            aOut.box = itemBox( item );

        // a grouped item moves its group, as a click in the editor picks the group
        if( EDA_GROUP* group = item->GetParentGroup() )
        {
            while( group->AsEdaItem()->GetParentGroup() )
                group = group->AsEdaItem()->GetParentGroup();

            item = static_cast<BOARD_ITEM*>( group->AsEdaItem() );

            if( !pad )
                aOut.box = item->GetBoundingBox();
        }

        aOut.item = item;
        return true;
    };

    std::vector<ENTRY> entries;

    for( const nlohmann::json& spec : aArgs["items"] )
    {
        ENTRY       e;
        std::string error;

        if( !resolve( spec, e, error ) )
            return KOPENAPI_RESULT::Error( 404, "items: " + error );

        if( std::none_of( entries.begin(), entries.end(), [&]( const ENTRY& x ) { return x.item == e.item; } ) )
            entries.push_back( e );
    }

    const size_t need = align.empty() ? 3 : 2;

    if( entries.size() < need )
        return KOPENAPI_RESULT::Error( 400, "items: " + std::to_string( need ) + " or more distinct items to "
                                            + ( align.empty() ? "distribute (one at each end, one between)" : "align" ) );

    std::vector<ENTRY> movable, locked;

    for( const ENTRY& e : entries )
        ( e.item->IsLocked() && !overrideLocks ? locked : movable ).push_back( e );

    nlohmann::json lockedJson = nlohmann::json::array();

    for( const ENTRY& e : locked )
        lockedJson.push_back( e.spec );

    // the moves: item -> delta
    std::vector<std::pair<ENTRY, VECTOR2I>> moves;

    if( !align.empty() )
    {
        std::function<int( const BOX2I& )> edge;
        std::function<bool( int, int )>    first;   ///< the editor's sort: its first item is the default target
        const bool                         vertical = align == "top" || align == "bottom" || align == "center_y";

        if( align == "top" )
            edge = []( const BOX2I& b ) { return b.GetTop(); }, first = std::less<int>();
        else if( align == "bottom" )
            edge = []( const BOX2I& b ) { return b.GetBottom(); }, first = std::greater<int>();
        else if( align == "left" )
            edge = []( const BOX2I& b ) { return b.GetLeft(); }, first = std::less<int>();
        else if( align == "right" )
            edge = []( const BOX2I& b ) { return b.GetRight(); }, first = std::greater<int>();
        else if( align == "center_x" )
            edge = []( const BOX2I& b ) { return b.Centre().x; }, first = std::less<int>();
        else
            edge = []( const BOX2I& b ) { return b.Centre().y; }, first = std::less<int>();

        int         target = 0;
        std::string targetFrom;

        if( aArgs.contains( "to_mm" ) )
        {
            if( !aArgs["to_mm"].is_number() )
                return KOPENAPI_RESULT::Error( 400, "to_mm: the coordinate to align to (y for top / bottom / center_y, else x)" );

            target = pcbIUScale.mmToIU( aArgs["to_mm"].get<double>() );
            targetFrom = "to_mm";
        }
        else if( aArgs.contains( "anchor" ) )
        {
            ENTRY       a;
            std::string error;

            if( !resolve( aArgs["anchor"], a, error ) )
                return KOPENAPI_RESULT::Error( 404, "anchor: " + error );

            target = edge( a.box );
            targetFrom = a.spec;
            std::erase_if( movable, [&]( const ENTRY& e ) { return e.item == a.item; } );
        }
        else
        {
            std::vector<ENTRY>& pool = locked.empty() ? movable : locked;
            auto best = std::min_element( pool.begin(), pool.end(), [&]( const ENTRY& x, const ENTRY& y )
                                          { return first( edge( x.box ), edge( y.box ) ); } );
            target = edge( best->box );
            targetFrom = best->spec;
        }

        for( const ENTRY& e : movable )
        {
            const int d = target - edge( e.box );

            if( d != 0 )
                moves.emplace_back( e, vertical ? VECTOR2I( 0, d ) : VECTOR2I( d, 0 ) );
        }

        const bool dryRun = aArgs.value( "dry_run", false );
        nlohmann::json movedJson = nlohmann::json::array();
        BOARD_COMMIT   commit( context->GetToolManager() );

        for( auto& [e, d] : moves )
        {
            if( !dryRun )
            {
                commit.Stage( e.item, CHT_MODIFY );
                e.item->Move( d );
            }

            BOX2I box = e.box;
            box.Move( d );
            movedJson.push_back( { { "item", e.spec }, { "dx_mm", toMm( d.x ) }, { "dy_mm", toMm( d.y ) }, { "box_mm", boxJson( box ) } } );
        }

        if( !dryRun && !moves.empty() )
        {
            commit.Push( _( "Align (API)" ) );
            context->SetContentModified();
        }

        return KOPENAPI_RESULT::Ok( { { "align", align },
                                      { "target_mm", toMm( target ) },
                                      { "target_from", targetFrom },
                                      { "moved", movedJson },
                                      { "locked", lockedJson },
                                      { "dry_run", dryRun } } );
    }

    // distribute: the editor refuses locked items
    if( !locked.empty() )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "locked items do not move: unlock them or override_locks" );
        r.body["locked"] = lockedJson;
        return r;
    }

    const bool x = distribute == "horizontal";
    std::vector<ENTRY>& items = movable;
    std::vector<int>    deltas;

    if( by == "gaps" )
    {
        std::sort( items.begin(), items.end(), [&]( const ENTRY& a, const ENTRY& b )
                   { return x ? a.box.GetLeft() < b.box.GetLeft() : a.box.GetTop() < b.box.GetTop(); } );

        std::vector<std::pair<int, int>> spans;

        for( const ENTRY& e : items )
            spans.emplace_back( x ? e.box.GetLeft() : e.box.GetTop(), x ? e.box.GetRight() : e.box.GetBottom() );

        deltas = GetDeltasForDistributeByGaps( spans );
    }
    else
    {
        std::sort( items.begin(), items.end(), [&]( const ENTRY& a, const ENTRY& b )
                   { return x ? a.box.Centre().x < b.box.Centre().x : a.box.Centre().y < b.box.Centre().y; } );

        std::vector<int> centres;

        for( const ENTRY& e : items )
            centres.push_back( x ? e.box.Centre().x : e.box.Centre().y );

        deltas = GetDeltasForDistributeByPoints( centres );
    }

    const bool     dryRun = aArgs.value( "dry_run", false );
    nlohmann::json movedJson = nlohmann::json::array();
    BOARD_COMMIT   commit( context->GetToolManager() );

    for( size_t i = 1; i + 1 < items.size(); ++i )
    {
        if( deltas[i] == 0 )
            continue;

        const VECTOR2I d = x ? VECTOR2I( deltas[i], 0 ) : VECTOR2I( 0, deltas[i] );

        if( !dryRun )
        {
            commit.Stage( items[i].item, CHT_MODIFY );
            items[i].item->Move( d );
        }

        BOX2I box = items[i].box;
        box.Move( d );
        movedJson.push_back( { { "item", items[i].spec }, { "dx_mm", toMm( d.x ) }, { "dy_mm", toMm( d.y ) }, { "box_mm", boxJson( box ) } } );
    }

    if( !dryRun && !movedJson.empty() )
    {
        commit.Push( _( "Distribute (API)" ) );
        context->SetContentModified();
    }

    nlohmann::json order = nlohmann::json::array();

    for( const ENTRY& e : items )
        order.push_back( e.spec );

    return KOPENAPI_RESULT::Ok( { { "distribute", distribute }, { "by", by }, { "order", order },
                                  { "moved", movedJson }, { "dry_run", dryRun } } );
}

} // namespace


KOPENAPI_REGISTER( "pcb_items_align",
                   "Align or distribute items, as the editor's Align / Distribute: items are REF (footprint), "
                   "REF.PAD (the pad's box is aligned, its footprint moves), or uuids of tracks, vias, drawings, "
                   "zones; a grouped item moves its group. align: top, bottom, left, right, center_x, center_y - to "
                   "to_mm, else the anchor item's edge, else a locked item's, else the outermost (top: topmost; "
                   "center_x / center_y: the leftmost / topmost centre); locked items stay (override_locks moves "
                   "them). distribute: horizontal or vertical over 3+ items, the two ends stay, by centers (equal "
                   "pitch) or gaps (equal spaces between boxes); locked items: 409. Footprint boxes without texts; "
                   "tracks are not dragged. dry_run answers the moves only",
                   R"json({"type":"object","required":["items"],"properties":{
                        "items":{"type":"array","items":{"type":"string"}},
                        "align":{"type":"string","enum":["top","bottom","left","right","center_x","center_y"]},
                        "distribute":{"type":"string","enum":["horizontal","vertical"]},
                        "by":{"type":"string","enum":["centers","gaps"],"default":"centers"},
                        "anchor":{"type":"string"},"to_mm":{"type":"number"},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_items_align );

KOPENAPI_MARK_EDITING( "pcb_items_align" );
