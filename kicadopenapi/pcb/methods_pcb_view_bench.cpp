/// @file methods_pcb_view_bench.cpp
/// @brief view_benchmark on the board editor's canvas: the board's own scenarios (high contrast
/// option, layer visibility through the appearance panel, footprint selection, net highlight, a
/// footprint moved and undone) for the generic driver in kicadopenapi_view_bench.h.
#include "kopenapi_pcb.h"

#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <gal/painter.h>
#include <frame_type.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_view_bench.h>
#include <kiway.h>
#include <pad.h>
#include <pcb_edit_frame.h>
#include <pcb_track.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <tools/pcb_actions.h>
#include <tools/pcb_selection_tool.h>
#include <widgets/appearance_controls.h>
#include <zone.h>

#include <algorithm>
#include <map>
#include <memory>


static KOPENAPI_RESULT h_pcb_view_benchmark( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;

    if( !frame || !frame->IsShown() || !frame->GetBoard() || !frame->GetCanvas() )
        return KOPENAPI_RESULT::Error( 409, "no board editor window open (pcb_open)" );

    KOPENAPI_REMOTE_GUARD remote( frame );
    BOARD*                board = frame->GetBoard();
    TOOL_MANAGER*         tools = frame->GetToolManager();
    KIGFX::VIEW*          view = frame->GetCanvas()->GetView();
    PCB_SELECTION_TOOL*   selTool = tools->GetTool<PCB_SELECTION_TOOL>();

    std::vector<FOOTPRINT*> footprints( board->Footprints().begin(), board->Footprints().end() );
    std::sort( footprints.begin(), footprints.end(),
               []( FOOTPRINT* a, FOOTPRINT* b )
               {
                   return a->GetReference() != b->GetReference() ? a->GetReference() < b->GetReference()
                                                                 : a->m_Uuid < b->m_Uuid;
               } );

    // the user's state that the scenarios touch
    EDA_ITEMS userSelection;

    if( selTool )
    {
        for( EDA_ITEM* item : selTool->GetSelection() )
            userSelection.push_back( item );
    }

    KIGFX::RENDER_SETTINGS* settings = view->GetPainter()->GetSettings();
    const std::set<int>     userHighlight = settings->GetHighlightNetCodes();
    const bool              userHighlightOn = settings->IsHighlightEnabled();
    const bool              userModified = frame->GetScreen()->IsContentModified();

    KOPENAPI_BENCH_HOOKS hooks;
    const BOX2I extents = frame->GetDocumentExtents( false );
    hooks.fitBox = BOX2D( VECTOR2D( extents.GetOrigin() ), VECTOR2D( extents.GetSize() ) );
    hooks.fitBox.Inflate( hooks.fitBox.GetWidth() * 0.02, hooks.fitBox.GetHeight() * 0.02 );

    hooks.option = "high contrast mode toggled (View > High Contrast), twice per cycle";
    hooks.setOption = [&]( bool ) { tools->RunAction( ACTIONS::highContrastMode ); };

    if( APPEARANCE_CONTROLS* panel = frame->GetAppearancePanel() )
    {
        for( PCB_LAYER_ID layer : { F_Cu, B_Cu, In1_Cu, F_SilkS, F_Mask } )
        {
            if( !board->IsLayerEnabled( layer ) || !board->IsLayerVisible( layer ) )
                continue;

            hooks.layers.emplace_back( board->GetLayerName( layer ).ToStdString( wxConvUTF8 ),
                                       [panel, layer]( bool aVisible ) { panel->SetLayerVisible( layer, aVisible ); } );
        }
    }

    // a fixed tenth of the footprints (1..50), first by reference
    EDA_ITEMS selectSet;

    for( size_t i = 0; i < footprints.size() && selectSet.size() < std::clamp<size_t>( footprints.size() / 10, 1, 50 ); ++i )
        selectSet.push_back( footprints[i] );

    if( selTool && !selectSet.empty() )
    {
        hooks.selectWhat = std::to_string( selectSet.size() ) + " footprints selected / selection cleared";
        hooks.select = [&]( bool aOn )
        {
            if( aOn )
                tools->RunAction<EDA_ITEMS*>( ACTIONS::selectItems, &selectSet );
            else
                tools->RunAction( ACTIONS::selectionClear );
        };
    }

    // the net with the most pads
    std::map<int, int> padsPerNet;

    for( FOOTPRINT* fp : footprints )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 )
                padsPerNet[pad->GetNetCode()]++;
        }
    }

    if( !padsPerNet.empty() )
    {
        const auto best = std::max_element( padsPerNet.begin(), padsPerNet.end(),
                                            []( const auto& a, const auto& b ) { return a.second < b.second; } );
        const int  net = best->first;

        hooks.highlightWhat = "net " + board->FindNet( net )->GetNetname().ToStdString( wxConvUTF8 ) + " ("
                              + std::to_string( best->second ) + " pads) highlighted / cleared";
        hooks.highlight = [&, net]( bool aOn )
        {
            if( aOn )
                tools->RunAction<int>( PCB_ACTIONS::highlightNet, net );
            else
                tools->RunAction( PCB_ACTIONS::clearHighlight );
        };
    }

    // the first unlocked footprint moved 1 mm, then undone
    auto movable = std::find_if( footprints.begin(), footprints.end(), []( FOOTPRINT* fp ) { return !fp->IsLocked(); } );

    if( movable != footprints.end() )
    {
        FOOTPRINT* fp = *movable;
        hooks.editWhat = fp->GetReference().ToStdString( wxConvUTF8 ) + " moved 1 mm (one commit), then undone";
        hooks.edit = [frame, fp]()
        {
            BOARD_COMMIT commit( frame->GetToolManager() );
            commit.Modify( fp );
            fp->Move( VECTOR2I( pcbIUScale.mmToIU( 1.0 ), 0 ) );
            commit.Push( _( "Benchmark move" ) );
            return true;
        };
        hooks.undo = [frame]()
        {
            const int before = frame->GetUndoCommandCount();
            frame->GetToolManager()->RunAction( ACTIONS::undo );
            return frame->GetUndoCommandCount() < before;
        };
    }

    hooks.restore = [&]()
    {
        tools->RunAction( ACTIONS::selectionClear );

        if( !userSelection.empty() )
            tools->RunAction<EDA_ITEMS*>( ACTIONS::selectItems, &userSelection );

        std::set<int> highlight = userHighlight;
        settings->SetHighlight( highlight, userHighlightOn );
        view->UpdateAllLayersColor();

        // the edit scenario's undo left a redo step and maybe the modified mark
        frame->ClearUndoORRedoList( EDA_BASE_FRAME::REDO_LIST );

        if( !userModified )
        {
            frame->GetScreen()->SetContentModified( false );
            frame->UpdateTitle();
        }
    };

    int tracks = 0, vias = 0;

    for( PCB_TRACK* t : board->Tracks() )
        ( t->Type() == PCB_VIA_T ? vias : tracks )++;

    int pads = 0;

    for( FOOTPRINT* fp : footprints )
        pads += (int) fp->Pads().size();

    nlohmann::json info = {
        { "editor", "pcb" },
        { "document", frame->GetBoard()->GetFileName().ToStdString( wxConvUTF8 ) },
        { "items",
          { { "footprints", footprints.size() },
            { "pads", pads },
            { "tracks", tracks },
            { "vias", vias },
            { "zones", board->Zones().size() },
            { "drawings", board->Drawings().size() },
            { "copper_layers", board->GetCopperLayerCount() },
            { "nets", board->GetNetCount() } } } };

    return KopenapiViewBenchmark( frame->GetCanvas(), hooks, aArgs, info );
}


KOPENAPI_REGISTER_VIEW_BENCHMARK( "pcb", h_pcb_view_benchmark );
