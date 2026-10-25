/// @file methods_pcb_board.cpp
/// @brief kicadopenapi board from schematic (ROADMAP task 3.1): pcb_new, pcb_netlist_apply,
/// pcb_outline_set.
///
/// pcb_netlist_apply is KiCad's own "Update PCB from Schematic": a KiCad netlist (from
/// sch_netlist_kicad; design_update_board chains both) read with KICAD_NETLIST_READER, footprints
/// loaded from the libraries, then BOARD_NETLIST_UPDATER through the document context — one undo
/// step in the GUI, the same in headless.  New footprints all land on one insertion point; they
/// are laid out in rows beside the board so nothing overlaps until placement (task 3.2).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <geometry/shape_poly_set.h>
#include <kicadopenapi_util.h>
#include <netlist_reader/board_netlist_updater.h>
#include <netlist_reader/netlist_reader.h>
#include <netlist_reader/pcb_netlist.h>
#include <netlist_reader/pcb_netlist_utils.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <netclass.h>
#include <project/net_settings.h>
#include <netinfo.h>
#include <pcb_draw_panel_gal.h>
#include <pcb_edit_frame.h>
#include <kicadopenapi_glow_view.h>
#include <pad.h>
#include <pcb_field.h>
#include <page_info.h>
#include <pgm_base.h>
#include <settings/settings_manager.h>
#include <project.h>
#include <project/project_file.h>
#include <wildcards_and_files_ext.h>
#include <drc/drc_engine.h>
#include <pcb_shape.h>
#include <tool/actions.h>
#include <kiway.h>
#include <pcb_track.h>
#include <reporter.h>
#include <richio.h>
#include <string_utils.h>
#include <tool/tool_manager.h>

#include <wx/ffile.h>
#include <wx/filename.h>
#include <wx/tokenzr.h>

#include <algorithm>
#include <map>
#include <fstream>
#include <functional>
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


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


/// @brief Footprint extent for packing: courtyard if it has one, else its body / pads
BOX2I footprintBox( FOOTPRINT* aFootprint )
{
    BOX2I box = aFootprint->GetCourtyard( F_CrtYd ).BBox();

    if( box.GetWidth() <= 0 || box.GetHeight() <= 0 )
        box = aFootprint->GetBoundingBox( false );

    return box;
}


/// @brief Board outline extent, if there is one
std::optional<BOX2I> outlineBox( BOARD* aBoard )
{
    std::optional<BOX2I> box;

    for( BOARD_ITEM* item : aBoard->Drawings() )
    {
        if( item->GetLayer() == Edge_Cuts )
        {
            if( box )
                box->Merge( item->GetBoundingBox() );
            else
                box = item->GetBoundingBox();
        }
    }

    return box;
}

/// @brief Board editor side of the GUI glow (kicadopenapi_glow_view.h); no frame headless
struct PCB_GLOW_TRAITS
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

    static int Mm() { return pcbIUScale.mmToIU( 1.0 ); }

    static void ItemColour( FRAME*, bool, std::optional<KIGFX::COLOR4D>& ) {}
};


/// @brief Glow what a call changed (GUI); a batch lights up one item after another
void glow( KOPENAPI_CONTEXT& aCtx, const std::vector<KIID>& aItems )
{
    if( !aCtx.headless )
        KopenapiGlow<PCB_GLOW_TRAITS>( aCtx.kiway, aItems, aItems.size() > 1 ? 300 : 0 );
}


/// @brief What a footprint looks like to the netlist update: changes in it make it glow
std::string footprintState( FOOTPRINT* aFootprint )
{
    std::string state = str( aFootprint->GetReference() ) + "|" + str( aFootprint->GetValue() ) + "|"
                        + str( aFootprint->GetFPID().Format() );

    for( PCB_FIELD* field : aFootprint->GetFields() )
        state += "|" + str( field->GetName() ) + "=" + str( field->GetText() );

    for( PAD* pad : aFootprint->Pads() )
        state += "|" + str( pad->GetNumber() ) + ":" + str( pad->GetNetname() );

    return state;
}

} // namespace


/// @brief No drawing frame on a board: an empty drawing sheet next to it, and a page that fits the
/// board plus 10 mm (as the barycenter conversions do)
void KopenapiFitPage( BOARD* aBoard, const BOX2I& aOutline )
{
    PAGE_INFO page( PAGE_SIZE_TYPE::User );
    page.SetWidthMils( std::max( 1000.0, pcbIUScale.IUToMils( aOutline.GetRight() ) + 10.0 * 1000.0 / 25.4 ) );
    page.SetHeightMils( std::max( 1000.0, pcbIUScale.IUToMils( aOutline.GetBottom() ) + 10.0 * 1000.0 / 25.4 ) );
    aBoard->SetPageSettings( page );
}


static KOPENAPI_RESULT h_pcb_new( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string path = aArgs.value( "path", std::string() );
    wxFileName        fn( wxString::FromUTF8( path ) );

    if( path.empty() || fn.GetExt() != wxS( "kicad_pcb" ) || !fn.IsAbsolute() )
        return KOPENAPI_RESULT::Error( 400, "give an absolute 'path' ending in .kicad_pcb (next to the .kicad_sch for one project)" );

    if( fn.FileExists() && !aArgs.value( "overwrite", false ) )
        return KOPENAPI_RESULT::Error( 409, "file exists (overwrite: true to replace it, or pcb_open)" );

    const int layers = aArgs.value( "layers", 2 );

    if( layers < 2 || layers > 32 || layers % 2 )
        return KOPENAPI_RESULT::Error( 400, "layers: an even copper layer count, 2..32" );

    if( !fn.DirExists() && !wxFileName::Mkdir( fn.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
        return KOPENAPI_RESULT::Error( 500, "cannot create the directory" );

    // No drawing frame unless asked: an empty drawing sheet for this board, named in the project
    const bool frame = aArgs.value( "frame", false );
    wxFileName sheet( fn );
    sheet.SetName( fn.GetName() + wxS( "-board" ) );
    sheet.SetExt( wxS( "kicad_wks" ) );

    if( !frame )
    {
        wxFFile wks( sheet.GetFullPath(), wxS( "w" ) );

        if( !wks.IsOpened()
            || !wks.Write( wxS( "(kicad_wks (version 20220228) (generator \"kicadopenapi\")\n"
                                "  (setup (textsize 1.5 1.5) (linewidth 0.15) (textlinewidth 0.15)\n"
                                "    (left_margin 0) (right_margin 0) (top_margin 0) (bottom_margin 0))\n)\n" ) ) )
        {
            return KOPENAPI_RESULT::Error( 500, "cannot write the drawing sheet " + str( sheet.GetFullPath() ) );
        }

        wxFileName projectFile( fn );
        projectFile.SetExt( FILEEXT::ProjectFileExtension );
        SETTINGS_MANAGER& settings = Pgm().GetSettingsManager();

        if( PROJECT* project = settings.GetProject( projectFile.GetFullPath() ) )
        {
            project->GetProjectFile().m_BoardDrawingSheetFile = sheet.GetFullName();
            settings.SaveProject( projectFile.GetFullPath(), project );
        }
        else
        {
            // no project loaded yet: written into the project file pcb_open is about to load
            nlohmann::json pro = nlohmann::json::object();

            if( projectFile.FileExists() )
            {
                std::ifstream in( projectFile.GetFullPath().ToStdString( wxConvUTF8 ) );
                pro = nlohmann::json::parse( in, nullptr, false );

                if( pro.is_discarded() )
                    pro = nlohmann::json::object();
            }

            pro["pcbnew"]["page_layout_descr_file"] = str( sheet.GetFullName() );
            std::ofstream out( projectFile.GetFullPath().ToStdString( wxConvUTF8 ) );
            out << pro.dump( 2 );
        }
    }

    try
    {
        BOARD board;
        board.SetCopperLayerCount( layers );

        if( !frame )
            KopenapiFitPage( &board, BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( toIU( 100 ), toIU( 80 ) ) ) );

        board.SetEnabledLayers( board.GetEnabledLayers() | LSET::AllCuMask( layers ) );
        PCB_IO_KICAD_SEXPR().SaveBoard( fn.GetFullPath(), board );
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 500, "cannot write the board: " + str( ioe.What() ) );
    }

    std::optional<KOPENAPI_METHOD> open = KOPENAPI_REGISTRY::Get().Find( "pcb_open" );

    if( !open )
        return KOPENAPI_RESULT::Error( 503, "pcb_open is not available" );

    return open->handler( aCtx, { { "path", path }, { "discard", aArgs.value( "discard", false ) } } );
}


static KOPENAPI_RESULT h_pcb_netlist_apply( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const std::string text = aArgs.value( "netlist", std::string() );

    if( text.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'netlist': KiCad netlist text (sch_netlist_kicad), or use design_update_board" );

    BOARD*             board = context->GetBoard();
    WX_STRING_REPORTER reporter;
    NETLIST            netlist;
    const bool         dryRun = aArgs.value( "dry_run", false );
    const bool         byReference = aArgs.value( "match", std::string( "uuid" ) ) == "reference";

    // Match footprints to symbols by the symbol's uuid (KiCad's default), or by reference
    netlist.SetFindByTimeStamp( !byReference );
    netlist.SetReplaceFootprints( aArgs.value( "replace_footprints", true ) );

    try
    {
        KICAD_NETLIST_READER reader( new STRING_LINE_READER( text, wxS( "schematic netlist" ) ), &netlist );
        reader.LoadNetlist();
        LoadNetlistFootprints( board, netlist, reporter );
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 400, "netlist not readable: " + str( ioe.What() ) );
    }

    std::map<KIID, std::string> before;

    for( FOOTPRINT* fp : board->Footprints() )
        before[fp->m_Uuid] = footprintState( fp );

    std::unique_ptr<BOARD_NETLIST_UPDATER> updater = context->MakeNetlistUpdater();
    updater->SetReporter( &reporter );
    updater->SetIsDryRun( dryRun );
    updater->SetLookupByTimestamp( !byReference );
    updater->SetDeleteUnusedFootprints( aArgs.value( "delete_extra", true ) );
    updater->SetReplaceFootprints( aArgs.value( "replace_footprints", true ) );
    updater->SetUpdateFields( true );
    updater->SetOverrideLocks( aArgs.value( "override_locks", false ) );

    const bool success = updater->UpdateNetlist( netlist );

    if( !dryRun && success )
    {
        context->OnNetlistChanged( *updater );

        // new nets take their classes from the patterns set before (pcb_rules_set ahead of the
        // netlist), and the rule engine sees them
        board->SynchronizeNetsAndNetClasses( false );

        if( std::shared_ptr<DRC_ENGINE> engine = board->GetDesignSettings().m_DRCEngine )
        {
            try
            {
                engine->InitEngine( board->GetDesignRulesPath() );
            }
            catch( const PARSE_ERROR& )
            {
                // a broken custom rules file: reported by pcb_drc
            }
        }
    }

    // New footprints: in rows beside the board (right of the outline or the other parts), sorted
    // by reference, so they do not lie on each other before placement
    nlohmann::json added = nlohmann::json::array();

    if( !dryRun && success && aArgs.value( "spread", true ) )
    {
        std::vector<FOOTPRINT*> fresh = updater->GetAddedFootprints();
        std::sort( fresh.begin(), fresh.end(), []( FOOTPRINT* a, FOOTPRINT* b )
                   { return KopenapiNaturalLess( str( a->GetReference() ), str( b->GetReference() ) ); } );

        std::optional<BOX2I> anchor = outlineBox( board );

        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( std::find( fresh.begin(), fresh.end(), fp ) != fresh.end() )
                continue;

            if( anchor )
                anchor->Merge( footprintBox( fp ) );
            else
                anchor = footprintBox( fp );
        }

        const int gap = toIU( 2.0 );
        const int left = anchor ? anchor->GetRight() + toIU( 10.0 ) : toIU( 50.0 );
        const int top = anchor ? anchor->GetTop() : toIU( 50.0 );
        const int rowWidth = toIU( aArgs.value( "spread_width_mm", 80.0 ) );
        int       x = left, y = top, rowHeight = 0;

        BOARD_COMMIT commit( context->GetToolManager() );

        for( FOOTPRINT* fp : fresh )
        {
            const BOX2I box = footprintBox( fp );

            if( x > left && x + box.GetWidth() > left + rowWidth )
            {
                x = left;
                y += rowHeight + gap;
                rowHeight = 0;
            }

            commit.Modify( fp );
            fp->Move( VECTOR2I( x, y ) - box.GetOrigin() );

            // KiCad marks what a netlist update added as "just added" until the editor's move
            // tool places it, and connectivity / ratsnest skip such footprints: laid out here,
            // they count
            fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
            x += box.GetWidth() + gap;
            rowHeight = std::max( rowHeight, (int) box.GetHeight() );

            added.push_back( { { "ref", str( fp->GetReference() ) },
                               { "footprint", str( fp->GetFPID().Format() ) },
                               { "x_mm", toMm( fp->GetPosition().x ) },
                               { "y_mm", toMm( fp->GetPosition().y ) } } );
        }

        if( !fresh.empty() )
        {
            commit.Push( _( "Spread new footprints (API)" ) );
            board->BuildConnectivity();
            board->GetConnectivity()->RecalculateRatsnest();
        }
    }

    // GUI: KiCad's OnNetlistChanged leaves the new footprints selected for an interactive drag
    // (which an API call never starts) and the canvas did not show them until the board was
    // reloaded; drop the selection and rebuild the view from the board
    if( !dryRun && success && !aCtx.headless && aCtx.kiway )
    {
        if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
        {
            if( frame->GetBoard() == board )
            {
                frame->GetToolManager()->RunAction( ACTIONS::selectionClear );
                frame->GetCanvas()->DisplayBoard( board );
                frame->GetCanvas()->SyncLayersVisibility( board );
                frame->GetToolManager()->RunAction( ACTIONS::zoomFitScreen );
                frame->GetCanvas()->Refresh();
            }
        }
    }

    // Glow what the update added or changed (GUI)
    if( !dryRun && success )
    {
        std::vector<KIID> changed;

        std::vector<FOOTPRINT*> sorted( board->Footprints().begin(), board->Footprints().end() );
        std::sort( sorted.begin(), sorted.end(), []( FOOTPRINT* a, FOOTPRINT* b )
                   { return KopenapiNaturalLess( str( a->GetReference() ), str( b->GetReference() ) ); } );

        for( FOOTPRINT* fp : sorted )
        {
            auto old = before.find( fp->m_Uuid );

            if( old == before.end() || old->second != footprintState( fp ) )
                changed.push_back( fp->m_Uuid );
        }

        glow( aCtx, changed );
    }

    // The report, one line per change / problem
    nlohmann::json lines = nlohmann::json::array();
    wxStringTokenizer tokens( reporter.GetMessages(), wxS( "\n" ) );

    while( tokens.HasMoreTokens() )
    {
        wxString line = tokens.GetNextToken().Trim().Trim( false );

        // "Processing symbol ..." is progress, not a change
        if( !line.IsEmpty() && !line.StartsWith( wxS( "Processing " ) ) )
            lines.push_back( str( line ) );
    }

    nlohmann::json result = { { "success", success },
                              { "dry_run", dryRun },
                              { "errors", updater->GetErrorCount() },
                              { "warnings", updater->GetWarningCount() },
                              { "new_footprints", updater->GetNewFootprintCount() },
                              { "footprints", board->Footprints().size() },
                              { "nets", std::max( 0, (int) board->GetNetCount() - 1 ) } };
    const KOPENAPI_LIST_CAP cap( aArgs, 50 );
    cap.Put( result, "report", lines );

    if( !added.empty() )
        cap.Put( result, "added", added );

    if( !success )
        result["hint"] = "see report: missing footprints (sch_footprint_check), unannotated parts";

    if( nlohmann::json trim = KopenapiLeadTrimRefresh( aCtx, *context ); !trim.is_null() )
        result["lead_trim"] = trim["trimmed"].size();

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_outline_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    BOX2I          rect;
    SHAPE_POLY_SET shape;   // polygon / trim: the new outline; empty for a rectangle

    if( aArgs.contains( "polygon" ) )
    {
        const nlohmann::json& pts = aArgs["polygon"];

        if( !pts.is_array() || pts.size() < 3 )
            return KOPENAPI_RESULT::Error( 400, "polygon: [[x_mm, y_mm], ...], at least 3 points" );

        SHAPE_LINE_CHAIN chain;

        for( const nlohmann::json& pt : pts )
        {
            if( !pt.is_array() || pt.size() != 2 || !pt[0].is_number() || !pt[1].is_number() )
                return KOPENAPI_RESULT::Error( 400, "polygon: [[x_mm, y_mm], ...]" );

            chain.Append( VECTOR2I( toIU( pt[0].get<double>() ), toIU( pt[1].get<double>() ) ) );
        }

        chain.SetClosed( true );
        shape.AddOutline( chain );
        shape.Simplify();

        if( shape.OutlineCount() != 1 )
            return KOPENAPI_RESULT::Error( 400, "polygon: one simple closed outline (it crosses itself)" );
    }
    else if( aArgs.contains( "trim" ) )
    {
        const nlohmann::json& t = aArgs["trim"];

        if( !t.is_object() )
            return KOPENAPI_RESULT::Error( 400, "trim: {top_mm, bottom_mm, left_mm, right_mm}" );

        SHAPE_POLY_SET present;

        if( !board->GetBoardPolygonOutlines( present, false ) || present.OutlineCount() == 0 )
            return KOPENAPI_RESULT::Error( 409, "trim needs a closed board outline" );

        const BOX2I  box = present.BBox();
        const double top = t.value( "top_mm", 0.0 ), bottom = t.value( "bottom_mm", 0.0 );
        const double left = t.value( "left_mm", 0.0 ), right = t.value( "right_mm", 0.0 );

        if( std::min( { top, bottom, left, right } ) < 0 || toIU( top + bottom ) >= box.GetHeight()
            || toIU( left + right ) >= box.GetWidth() )
        {
            return KOPENAPI_RESULT::Error( 400, "trim: non-negative, less than the outline's size" );
        }

        SHAPE_POLY_SET   keep;
        SHAPE_LINE_CHAIN k;
        const VECTOR2I   a( box.GetLeft() + toIU( left ), box.GetTop() + toIU( top ) );
        const VECTOR2I   b( box.GetRight() - toIU( right ), box.GetBottom() - toIU( bottom ) );
        k.Append( a );
        k.Append( VECTOR2I( b.x, a.y ) );
        k.Append( b );
        k.Append( VECTOR2I( a.x, b.y ) );
        k.SetClosed( true );
        keep.AddOutline( k );
        shape = present;
        shape.BooleanIntersection( keep );
        shape.Simplify();

        if( shape.OutlineCount() == 0 )
            return KOPENAPI_RESULT::Error( 400, "trim leaves no board" );
    }
    else if( aArgs.value( "fit", false ) )
    {
        // Around the parts with a margin, snapped outwards to 0.5 mm
        std::optional<BOX2I> parts;

        for( FOOTPRINT* fp : board->Footprints() )
        {
            if( parts )
                parts->Merge( footprintBox( fp ) );
            else
                parts = footprintBox( fp );
        }

        if( !parts )
            return KOPENAPI_RESULT::Error( 409, "no footprints to fit the outline around" );

        const int margin = toIU( aArgs.value( "margin_mm", 3.0 ) );
        const int snap = toIU( 0.5 );
        auto      down = [&]( int v ) { return int( std::floor( double( v ) / snap ) ) * snap; };
        auto      up = [&]( int v ) { return int( std::ceil( double( v ) / snap ) ) * snap; };

        const VECTOR2I a( down( parts->GetLeft() - margin ), down( parts->GetTop() - margin ) );
        const VECTOR2I b( up( parts->GetRight() + margin ), up( parts->GetBottom() + margin ) );
        rect = BOX2I( a, b - a );
    }
    else
    {
        for( const char* key : { "x_mm", "y_mm", "width_mm", "height_mm" } )
        {
            if( !aArgs.contains( key ) || !aArgs[key].is_number() )
                return KOPENAPI_RESULT::Error( 400, "give x_mm, y_mm, width_mm, height_mm (top-left corner, size), or fit: true" );
        }

        if( aArgs["width_mm"].get<double>() <= 0 || aArgs["height_mm"].get<double>() <= 0 )
            return KOPENAPI_RESULT::Error( 400, "width_mm and height_mm must be positive" );

        rect = BOX2I( VECTOR2I( toIU( aArgs["x_mm"] ), toIU( aArgs["y_mm"] ) ),
                      VECTOR2I( toIU( aArgs["width_mm"] ), toIU( aArgs["height_mm"] ) ) );
    }

    if( shape.OutlineCount() > 0 )
        rect = shape.BBox();

    const double radius = shape.OutlineCount() > 0 ? 0.0 : aArgs.value( "corner_radius_mm", 0.0 );

    if( radius < 0 || toIU( radius ) * 2 > std::min( rect.GetWidth(), rect.GetHeight() ) )
        return KOPENAPI_RESULT::Error( 400, "corner_radius_mm: 0 .. half the shorter side" );

    // a locked outline stays: the new one would replace it
    if( !aArgs.value( "override_locks", false ) )
    {
        for( BOARD_ITEM* item : board->Drawings() )
        {
            if( item->GetLayer() == Edge_Cuts && item->IsLocked() )
                return KOPENAPI_RESULT::Error( 409, "the board outline is locked (pcb_unlock, or override_locks: true)" );
        }
    }

    BOARD_COMMIT commit( context->GetToolManager() );
    int          removed = 0;

    for( BOARD_ITEM* item : std::vector<BOARD_ITEM*>( board->Drawings().begin(), board->Drawings().end() ) )
    {
        if( item->GetLayer() == Edge_Cuts )
        {
            commit.Remove( item );
            removed++;
        }
    }

    const STROKE_PARAMS     stroke( toIU( aArgs.value( "line_width_mm", 0.1 ) ), LINE_STYLE::SOLID );
    std::vector<PCB_SHAPE*> added;

    if( shape.OutlineCount() > 0 )
    {
        // each outline and each hole its own closed polygon on Edge.Cuts
        for( int i = 0; i < shape.OutlineCount(); ++i )
        {
            for( int h = -1; h < shape.HoleCount( i ); ++h )
            {
                auto* poly = new PCB_SHAPE( board, SHAPE_T::POLY );
                poly->SetLayer( Edge_Cuts );
                poly->SetStroke( stroke );
                poly->SetFilled( false );
                poly->SetPolyShape( SHAPE_POLY_SET( h < 0 ? shape.Outline( i ) : shape.Hole( i, h ) ) );
                added.push_back( poly );
            }
        }
    }
    else
    {
        auto* rectangle = new PCB_SHAPE( board, SHAPE_T::RECTANGLE );
        rectangle->SetLayer( Edge_Cuts );
        rectangle->SetStroke( stroke );
        rectangle->SetStart( rect.GetOrigin() );
        rectangle->SetEnd( rect.GetEnd() );

        if( radius > 0 )
            rectangle->SetCornerRadius( toIU( radius ) );

        added.push_back( rectangle );
    }

    for( PCB_SHAPE* item : added )
        commit.Add( item );

    PCB_SHAPE* outline = added.front();
    commit.Push( _( "Board outline (API)" ) );

    // A frameless board's page follows its outline
    if( board->GetPageSettings().GetType() == PAGE_SIZE_TYPE::User )
    {
        KopenapiFitPage( board, rect );

        if( auto* frame = PCB_GLOW_TRAITS::Frame( aCtx.headless ? nullptr : aCtx.kiway ) )
        {
            frame->GetCanvas()->DisplayBoard( board );
            frame->GetToolManager()->RunAction( ACTIONS::zoomFitScreen );
            frame->GetCanvas()->Refresh();
        }
    }

    glow( aCtx, { outline->m_Uuid } );

    nlohmann::json outside = nlohmann::json::array();

    for( FOOTPRINT* fp : board->Footprints() )
    {
        const BOX2I fb = footprintBox( fp );
        bool        in = rect.Contains( fb );

        if( in && shape.OutlineCount() > 0 )
        {
            for( const VECTOR2I& c : { fb.GetOrigin(), VECTOR2I( fb.GetRight(), fb.GetTop() ), fb.GetEnd(),
                                       VECTOR2I( fb.GetLeft(), fb.GetBottom() ) } )
            {
                in = in && shape.Contains( c, -1, 0, true );
            }
        }

        if( !in )
            outside.push_back( str( fp->GetReference() ) );
    }

    nlohmann::json result = { { "uuid", str( outline->m_Uuid.AsString() ) },
                              { "x_mm", toMm( rect.GetX() ) },
                              { "y_mm", toMm( rect.GetY() ) },
                              { "width_mm", toMm( rect.GetWidth() ) },
                              { "height_mm", toMm( rect.GetHeight() ) },
                              { "corner_radius_mm", radius },
                              { "replaced_edge_items", removed } };

    if( shape.OutlineCount() > 0 )
    {
        result["kind"] = aArgs.contains( "trim" ) ? "trim" : "polygon";
        result["area_mm2"] = shape.Area() / ( pcbIUScale.IU_PER_MM * pcbIUScale.IU_PER_MM );
        result["points"] = shape.Outline( 0 ).PointCount();
    }

    if( !outside.empty() )
        result["footprints_outside"] = outside;

    return KOPENAPI_RESULT::Ok( result );
}


namespace
{

/// @brief The board-wide constraints the rules methods read and write: name -> member of the settings
std::vector<std::pair<const char*, int BOARD_DESIGN_SETTINGS::*>> constraintFields()
{
    return { { "min_clearance_mm", &BOARD_DESIGN_SETTINGS::m_MinClearance },
             { "min_track_width_mm", &BOARD_DESIGN_SETTINGS::m_TrackMinWidth },
             { "min_via_diameter_mm", &BOARD_DESIGN_SETTINGS::m_ViasMinSize },
             { "min_via_annular_mm", &BOARD_DESIGN_SETTINGS::m_ViasMinAnnularWidth },
             { "min_through_drill_mm", &BOARD_DESIGN_SETTINGS::m_MinThroughDrill },
             { "copper_edge_clearance_mm", &BOARD_DESIGN_SETTINGS::m_CopperEdgeClearance },
             { "hole_clearance_mm", &BOARD_DESIGN_SETTINGS::m_HoleClearance },
             { "hole_to_hole_mm", &BOARD_DESIGN_SETTINGS::m_HoleToHoleMin } };
}


/// @brief Net class values: name -> getter / setter in mm
struct CLASS_FIELD
{
    const char*                             name;
    std::function<std::optional<int>( NETCLASS& )> get;
    std::function<void( NETCLASS&, int )>   set;
};


const std::vector<CLASS_FIELD>& classFields()
{
    static const std::vector<CLASS_FIELD> fields = {
        { "clearance_mm", []( NETCLASS& c ) { return c.HasClearance() ? std::optional<int>( c.GetClearance() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetClearance( v ); } },
        { "track_width_mm", []( NETCLASS& c ) { return c.HasTrackWidth() ? std::optional<int>( c.GetTrackWidth() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetTrackWidth( v ); } },
        { "via_diameter_mm", []( NETCLASS& c ) { return c.HasViaDiameter() ? std::optional<int>( c.GetViaDiameter() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetViaDiameter( v ); } },
        { "via_drill_mm", []( NETCLASS& c ) { return c.HasViaDrill() ? std::optional<int>( c.GetViaDrill() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetViaDrill( v ); } },
        { "diff_pair_width_mm", []( NETCLASS& c ) { return c.HasDiffPairWidth() ? std::optional<int>( c.GetDiffPairWidth() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetDiffPairWidth( v ); } },
        { "diff_pair_gap_mm", []( NETCLASS& c ) { return c.HasDiffPairGap() ? std::optional<int>( c.GetDiffPairGap() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetDiffPairGap( v ); } } };

    return fields;
}


// the project's custom design rules (.kicad_dru), "" when there are none
std::string customRules( BOARD* aBoard )
{
    const wxString path = aBoard->GetDesignRulesPath();
    wxString       text;
    wxFFile        file;

    if( path.IsEmpty() || !wxFileName::FileExists( path ) || !file.Open( path, wxS( "rb" ) ) || !file.ReadAll( &text, wxConvUTF8 ) )
        return {};

    return str( text );
}


bool writeCustomRules( const wxString& aPath, const std::string& aText )
{
    if( aText.empty() )
        return !wxFileName::FileExists( aPath ) || wxRemoveFile( aPath );

    wxFFile file( aPath, wxS( "wb" ) );
    return file.IsOpened() && file.Write( wxString::FromUTF8( aText ), wxConvUTF8 ) && file.Close();
}


// member nets listed per class by default
constexpr size_t RULES_LIST_CAP = 20;


nlohmann::json rulesJson( BOARD* aBoard, const KOPENAPI_LIST_CAP& aCap )
{
    BOARD_DESIGN_SETTINGS&         bds = aBoard->GetDesignSettings();
    std::shared_ptr<NET_SETTINGS>& net = bds.m_NetSettings;
    nlohmann::json                 constraints = nlohmann::json::object();

    for( const auto& [name, member] : constraintFields() )
        constraints[name] = toMm( bds.*member );

    auto classJson = [&]( const std::shared_ptr<NETCLASS>& aClass )
    {
        nlohmann::json c = { { "name", str( aClass->GetName() ) } };

        for( const CLASS_FIELD& f : classFields() )
        {
            if( std::optional<int> v = f.get( *aClass ) )
                c[f.name] = toMm( *v );
        }

        return c;
    };

    nlohmann::json classes = nlohmann::json::array();
    classes.push_back( classJson( net->GetDefaultNetclass() ) );

    for( const auto& [name, netclass] : net->GetNetclasses() )
        classes.push_back( classJson( netclass ) );

    nlohmann::json patterns = nlohmann::json::array();

    for( const auto& [matcher, netclass] : net->GetNetclassPatternAssignments() )
        patterns.push_back( { { "pattern", str( matcher->GetPattern() ) }, { "netclass", str( netclass ) } } );

    // Which class each net ends up in
    std::map<std::string, nlohmann::json> members;

    for( NETINFO_ITEM* item : aBoard->GetNetInfo() )
    {
        if( item->GetNetCode() <= 0 )
            continue;

        // The effective class is a composite ("Power" + "Default"): list the net under each
        // explicit class it belongs to, under Default only when it has no other
        const std::string netName = str( UnescapeString( item->GetNetname() ) );
        NETCLASS*         effective = item->GetNetClass();
        std::vector<std::string> names;

        if( effective )
        {
            for( NETCLASS* part : effective->GetConstituentNetclasses() )
            {
                if( part->GetName() != NETCLASS::Default )
                    names.push_back( str( part->GetName() ) );
            }
        }

        if( names.empty() )
            names.push_back( str( NETCLASS::Default ) );

        for( const std::string& name : names )
            members[name].push_back( netName );
    }

    for( nlohmann::json& c : classes )
    {
        nlohmann::json nets = members.count( c["name"] ) ? members[c["name"]] : nlohmann::json::array();
        c["net_count"] = nets.size();
        aCap.Put( c, "nets", std::move( nets ) );
    }

    nlohmann::json out = { { "copper_layers", aBoard->GetCopperLayerCount() },
                           { "thickness_mm", toMm( bds.GetBoardThickness() ) },
                           { "constraints", constraints },
                           { "netclasses", classes },
                           { "custom_rules", customRules( aBoard ) } };
    aCap.Put( out, "netclass_patterns", std::move( patterns ) );
    return out;
}

} // namespace


static KOPENAPI_RESULT h_pcb_rules_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    return KOPENAPI_RESULT::Ok( rulesJson( context->GetBoard(), KOPENAPI_LIST_CAP( aArgs, RULES_LIST_CAP ) ) );
}


static KOPENAPI_RESULT h_pcb_rules_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                         board = context->GetBoard();
    BOARD_DESIGN_SETTINGS&         bds = board->GetDesignSettings();
    std::shared_ptr<NET_SETTINGS>& net = bds.m_NetSettings;

    // Custom rules first: they must compile, or nothing changes
    if( aArgs.contains( "custom_rules" ) )
    {
        if( !aArgs["custom_rules"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "custom_rules: the text of a .kicad_dru file (\"\" removes it)" );

        const wxString path = board->GetDesignRulesPath();

        if( path.IsEmpty() )
            return KOPENAPI_RESULT::Error( 409, "custom_rules need a saved board (the rules file sits next to it)" );

        const std::string before = customRules( board );
        const std::string after = aArgs["custom_rules"].get<std::string>();

        if( !writeCustomRules( path, after ) )
            return KOPENAPI_RESULT::Error( 500, "cannot write " + str( path ) );

        if( std::shared_ptr<DRC_ENGINE> engine = bds.m_DRCEngine )
        {
            try
            {
                engine->InitEngine( path );
            }
            catch( const PARSE_ERROR& error )
            {
                writeCustomRules( path, before );

                try
                {
                    engine->InitEngine( path );
                }
                catch( const PARSE_ERROR& )
                {
                }

                return KOPENAPI_RESULT::Error( 422, "custom_rules do not compile: " + str( error.What() ) );
            }
        }
    }

    // Validate everything first: all or nothing
    auto positive = [&]( const nlohmann::json& aValue ) { return aValue.is_number() && aValue.get<double>() >= 0; };

    if( aArgs.contains( "copper_layers" ) )
    {
        const int layers = aArgs["copper_layers"].is_number_integer() ? aArgs["copper_layers"].get<int>() : -1;

        if( layers < 2 || layers > 32 || layers % 2 )
            return KOPENAPI_RESULT::Error( 400, "copper_layers: an even count, 2..32" );

        // Copper on layers that would go away blocks the change
        if( layers < board->GetCopperLayerCount() )
        {
            LSET dropped = LSET::AllCuMask( board->GetCopperLayerCount() ) & ~LSET::AllCuMask( layers );

            for( BOARD_ITEM* item : board->Tracks() )
            {
                if( dropped.Contains( item->GetLayer() ) )
                    return KOPENAPI_RESULT::Error( 409, "tracks on copper layers that would be removed" );
            }
        }
    }

    if( aArgs.contains( "constraints" ) )
    {
        if( !aArgs["constraints"].is_object() )
            return KOPENAPI_RESULT::Error( 400, "constraints: object of name -> mm" );

        for( const auto& [name, value] : aArgs["constraints"].items() )
        {
            const auto fields = constraintFields();

            if( std::none_of( fields.begin(), fields.end(), [&]( const auto& f ) { return name == f.first; } ) )
                return KOPENAPI_RESULT::Error( 400, "unknown constraint '" + name + "' (see pcb_rules_get)" );

            if( !positive( value ) )
                return KOPENAPI_RESULT::Error( 400, name + ": a number >= 0 (mm)" );
        }
    }

    if( aArgs.contains( "netclasses" ) )
    {
        if( !aArgs["netclasses"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "netclasses: array of {name, clearance_mm, track_width_mm, ..., nets}" );

        for( const nlohmann::json& c : aArgs["netclasses"] )
        {
            if( !c.is_object() || !c.contains( "name" ) || !c["name"].is_string() || c["name"].get<std::string>().empty() )
                return KOPENAPI_RESULT::Error( 400, "every netclass needs a 'name' (\"Default\" edits the default class)" );

            for( const CLASS_FIELD& f : classFields() )
            {
                if( c.contains( f.name ) && !positive( c[f.name] ) )
                    return KOPENAPI_RESULT::Error( 400, std::string( f.name ) + ": a number >= 0 (mm)" );
            }

            if( c.contains( "nets" ) && !c["nets"].is_array() )
                return KOPENAPI_RESULT::Error( 400, "nets: array of net names or patterns (*, ?)" );

            if( c.value( "delete", false ) && c.value( "name", std::string() ) == str( NETCLASS::Default ) )
                return KOPENAPI_RESULT::Error( 400, "the Default class cannot be deleted" );
        }
    }

    // Apply
    if( aArgs.contains( "copper_layers" ) )
    {
        board->SetCopperLayerCount( aArgs["copper_layers"] );
        board->SetEnabledLayers( board->GetEnabledLayers() | LSET::AllCuMask( aArgs["copper_layers"] ) );
    }

    if( aArgs.contains( "thickness_mm" ) && positive( aArgs["thickness_mm"] ) )
        bds.SetBoardThickness( toIU( aArgs["thickness_mm"] ) );

    if( aArgs.contains( "constraints" ) )
    {
        for( const auto& [name, member] : constraintFields() )
        {
            if( aArgs["constraints"].contains( name ) )
                bds.*member = toIU( aArgs["constraints"][name].get<double>() );
        }
    }

    if( aArgs.contains( "netclasses" ) )
    {
        for( const nlohmann::json& c : aArgs["netclasses"] )
        {
            const wxString           name = wxString::FromUTF8( c["name"].get<std::string>() );
            std::shared_ptr<NETCLASS> netclass;

            // the class's patterns, or the patterns of other classes naming the same nets, go
            auto dropPatterns = [&]( const std::function<bool( const wxString&, const wxString& )>& aDrop )
            {
                auto& assignments = net->GetNetclassPatternAssignments();
                std::vector<std::pair<std::unique_ptr<EDA_COMBINED_MATCHER>, wxString>> kept;

                for( auto& [matcher, cls] : assignments )
                {
                    if( !aDrop( matcher->GetPattern(), cls ) )
                        kept.emplace_back( std::move( matcher ), cls );
                }

                net->SetNetclassPatternAssignments( std::move( kept ) );
            };

            if( c.value( "delete", false ) )
            {
                dropPatterns( [&]( const wxString&, const wxString& aClass ) { return aClass == name; } );

                std::map<wxString, std::shared_ptr<NETCLASS>> classes = net->GetNetclasses();
                classes.erase( name );
                net->SetNetclasses( classes );
                continue;
            }

            if( c.contains( "nets" ) )
            {
                std::set<wxString> named;

                for( const nlohmann::json& pattern : c["nets"] )
                {
                    if( pattern.is_string() )
                        named.insert( wxString::FromUTF8( pattern.get<std::string>() ) );
                }

                dropPatterns( [&]( const wxString& aPattern, const wxString& aClass )
                              { return aClass == name || named.count( aPattern ); } );
            }

            if( name == NETCLASS::Default )
            {
                netclass = net->GetDefaultNetclass();
            }
            else if( net->GetNetclasses().count( name ) )
            {
                netclass = net->GetNetclasses().at( name );
            }
            else
            {
                netclass = std::make_shared<NETCLASS>( name, false );
                net->SetNetclass( name, netclass );
            }

            for( const CLASS_FIELD& f : classFields() )
            {
                if( c.contains( f.name ) )
                    f.set( *netclass, toIU( c[f.name].get<double>() ) );
            }

            for( const nlohmann::json& pattern : c.value( "nets", nlohmann::json::array() ) )
            {
                if( pattern.is_string() )
                    net->SetNetclassPatternAssignment( wxString::FromUTF8( pattern.get<std::string>() ), name );
            }
        }

        net->ClearAllCaches();
    }

    board->SynchronizeNetsAndNetClasses( false );

    // The rule engine (clearances and widths the router and DRC use) follows the new classes
    if( std::shared_ptr<DRC_ENGINE> engine = bds.m_DRCEngine )
    {
        try
        {
            engine->InitEngine( board->GetDesignRulesPath() );
        }
        catch( const PARSE_ERROR& )
        {
            // a broken custom rules file: reported by pcb_drc
        }
    }

    context->SetContentModified();

    nlohmann::json result = rulesJson( board, KOPENAPI_LIST_CAP( aArgs, RULES_LIST_CAP ) );

    // values the board's minimums override: KiCad's router and DRC use the larger one
    nlohmann::json warnings = nlohmann::json::array();

    auto below = [&]( const wxString& aClass, const std::shared_ptr<NETCLASS>& aCls )
    {
        if( aCls->HasTrackWidth() && aCls->GetTrackWidth() < bds.m_TrackMinWidth )
            warnings.push_back( "class " + str( aClass ) + ": track_width_mm " + std::to_string( toMm( aCls->GetTrackWidth() ) )
                                + " is below the board's minimum " + std::to_string( toMm( bds.m_TrackMinWidth ) )
                                + ": tracks come out that wide (lower constraints.min_track_width_mm)" );

        if( aCls->HasClearance() && aCls->GetClearance() < bds.m_MinClearance )
            warnings.push_back( "class " + str( aClass ) + ": clearance_mm " + std::to_string( toMm( aCls->GetClearance() ) )
                                + " is below the board's minimum " + std::to_string( toMm( bds.m_MinClearance ) )
                                + ": the minimum holds (lower constraints.min_clearance_mm)" );
    };

    below( NETCLASS::Default, net->GetDefaultNetclass() );

    for( const auto& [cname, cls] : net->GetNetclasses() )
        below( cname, cls );

    if( !warnings.empty() )
        result["warnings"] = warnings;
    result["note"] = "net classes and constraints are saved with the board's project (pcb_save)";
    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "pcb_new",
                   "Create a new empty board file (layers: copper count, default 2) and open it; put it "
                   "next to the schematic (same name, .kicad_pcb) so both share the project; then "
                   "design_update_board to bring the parts and nets in",
                   R"json({"type":"object","required":["path"],"properties":{
                        "path":{"type":"string","description":"absolute path ending in .kicad_pcb"},
                        "layers":{"type":"integer","default":2},
                        "frame":{"type":"boolean","default":false,"description":"KiCad's drawing frame and title block; default none (page fits the board, pcb_outline_set resizes it)"},
                        "overwrite":{"type":"boolean","default":false},
                        "discard":{"type":"boolean","default":false,"description":"drop unsaved changes of the open board"}}})json"_json,
                   false, h_pcb_new, 120 );

KOPENAPI_REGISTER( "pcb_netlist_apply",
                   "Update the open board from a KiCad netlist (text from sch_netlist_kicad) - KiCad's "
                   "Update PCB from Schematic: adds / replaces / removes footprints, sets nets, values, "
                   "fields; new footprints are laid out in rows beside the board; dry_run reports only; "
                   "match symbols to footprints by uuid (default) or reference; report / added cut to "
                   "list_limit (default 50; full: true for all). Usually called through design_update_board",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","required":["netlist"],"properties":{
                        "netlist":{"type":"string"},
                        "dry_run":{"type":"boolean","default":false},
                        "match":{"type":"string","enum":["uuid","reference"],"default":"uuid"},
                        "delete_extra":{"type":"boolean","default":true,"description":"remove footprints without a symbol"},
                        "replace_footprints":{"type":"boolean","default":true,"description":"swap footprints whose library id changed"},
                        "override_locks":{"type":"boolean","default":false,"description":"locked footprints are not replaced or removed otherwise (their nets and fields are still updated)"},
                        "spread":{"type":"boolean","default":true},
                        "spread_width_mm":{"type":"number","default":80}}})json"_json, 50 ),
                   false, h_pcb_netlist_apply, 600 );

KOPENAPI_REGISTER( "pcb_outline_set",
                   "Set the board outline (Edge.Cuts), replacing the old one: a rectangle by top-left "
                   "corner and size in mm, or fit: true around the footprints with margin_mm (optional "
                   "rounded corners); a polygon; or trim: the present outline cut by this much off its "
                   "sides (a lower / shorter card keeping its shape: edge fingers, bevels); reports "
                   "footprints lying outside",
                   R"json({"type":"object","properties":{
                        "polygon":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"[[x_mm, y_mm], ...]: a closed outline"},
                        "trim":{"type":"object","properties":{"top_mm":{"type":"number"},"bottom_mm":{"type":"number"},"left_mm":{"type":"number"},"right_mm":{"type":"number"}},"description":"cut this much off the present outline's sides (its bounding box)"},
                        "x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "width_mm":{"type":"number"},"height_mm":{"type":"number"},
                        "fit":{"type":"boolean","default":false},
                        "margin_mm":{"type":"number","default":3},
                        "corner_radius_mm":{"type":"number","default":0},
                        "line_width_mm":{"type":"number","default":0.1},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_outline_set );

KOPENAPI_REGISTER( "pcb_rules_get",
                   "Board design rules: copper layer count, thickness, board-wide minimum constraints "
                   "(clearance, track width, via, drill, edge and hole clearances, mm), net classes with "
                   "their values and member nets (net_count; nets capped at list_limit, full: true for all), net class pattern "
                   "assignments, the custom rules (.kicad_dru text)",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{}})json"_json, RULES_LIST_CAP ), false, h_pcb_rules_get );

KOPENAPI_REGISTER( "pcb_rules_set",
                   "Set board design rules (all or nothing): copper_layers, thickness_mm, constraints {name: "
                   "mm} as listed by pcb_rules_get, netclasses [{name ('Default' edits the default), "
                   "clearance_mm, track_width_mm, via_diameter_mm, via_drill_mm, diff_pair_width_mm, "
                   "diff_pair_gap_mm, nets: [names or patterns like '+*V*', 'GND']}] - e.g. a Power class "
                   "0.5 mm wide for the rails; custom_rules (KiCad's rule language); answers the rules as now in force "
                   "(as pcb_rules_get: member nets capped at list_limit)",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "copper_layers":{"type":"integer"},
                        "thickness_mm":{"type":"number"},
                        "custom_rules":{"type":"string","description":"the project's custom rules (.kicad_dru text, e.g. (version 1) (rule ...)); written at once, refused (422, nothing changed) when it does not compile; \"\" removes them"},
                        "constraints":{"type":"object","description":"min_clearance_mm, min_track_width_mm, min_via_diameter_mm, min_via_annular_mm, min_through_drill_mm, copper_edge_clearance_mm, hole_clearance_mm, hole_to_hole_mm"},
                        "netclasses":{"type":"array","items":{"type":"object","required":["name"],"properties":{
                            "name":{"type":"string"},"clearance_mm":{"type":"number"},"track_width_mm":{"type":"number"},
                            "via_diameter_mm":{"type":"number"},"via_drill_mm":{"type":"number"},
                            "diff_pair_width_mm":{"type":"number"},"diff_pair_gap_mm":{"type":"number"},
                            "nets":{"type":"array","items":{"type":"string"},"description":"the class's nets / patterns: replaces its earlier ones, and the nets leave other classes; [] empties it"},
                            "delete":{"type":"boolean","description":"remove this class and its net patterns"}}}}}})json"_json, RULES_LIST_CAP ),
                   false, h_pcb_rules_set );

KOPENAPI_MARK_EDITING( "pcb_netlist_apply", "pcb_outline_set", "pcb_rules_set" );
KOPENAPI_MARK_CONCURRENT_READS( "pcb_rules_get" );
