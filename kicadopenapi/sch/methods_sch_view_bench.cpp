/// @file methods_sch_view_bench.cpp
/// @brief view_benchmark on the schematic editor's canvas: the schematic's own scenarios (hidden
/// fields option, symbol selection, net highlight, a symbol moved and undone) for the generic driver
/// in kicadopenapi_view_bench.h. Runs on the sheet shown, or switches to another one and back.
#include "kopenapi_sch.h"

#include <frame_type.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <kicadopenapi_view_bench.h>
#include <kiway.h>
#include <sch_commit.h>
#include <sch_edit_frame.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <tools/sch_actions.h>
#include <tools/sch_editor_control.h>
#include <tools/sch_selection_tool.h>

#include <algorithm>
#include <map>
#include <optional>

#include <wx/filename.h>


static KOPENAPI_RESULT h_sch_view_benchmark( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;

    if( !frame || !frame->IsShown() || !frame->GetScreen() || !frame->GetCanvas() )
        return KOPENAPI_RESULT::Error( 409, "no schematic editor window open (sch_open)" );

    KOPENAPI_REMOTE_GUARD remote( frame );
    TOOL_MANAGER*         tools = frame->GetToolManager();
    SCH_SELECTION_TOOL*   selTool = tools->GetTool<SCH_SELECTION_TOOL>();
    SCH_EDITOR_CONTROL*   control = tools->GetTool<SCH_EDITOR_CONTROL>();

    EDA_ITEMS userSelection;

    if( selTool )
    {
        for( EDA_ITEM* item : selTool->GetSelection() )
            userSelection.push_back( item );
    }

    // the sheet to measure: the one shown, the one with the most items, or by path / file name
    SCH_SHEET_PATH    userSheet = frame->GetCurrentSheet();
    const std::string want = aArgs.value( "sheet", std::string( "current" ) );
    bool              switched = false;

    if( want != "current" )
    {
        std::optional<SCH_SHEET_PATH> target;
        size_t                        most = 0;

        for( const SCH_SHEET_PATH& path : frame->Schematic().Hierarchy() )
        {
            SCH_SCREEN* pathScreen = path.LastScreen();

            if( !pathScreen )
                continue;

            if( want == "busiest" )
            {
                if( pathScreen->Items().size() > most )
                {
                    most = pathScreen->Items().size();
                    target = path;
                }
            }
            else if( KopenapiGlob( want, path.PathHumanReadable().ToStdString( wxConvUTF8 ) )
                     || KopenapiGlob( want, wxFileName( pathScreen->GetFileName() ).GetFullName().ToStdString( wxConvUTF8 ) ) )
            {
                target = path;
                break;
            }
        }

        if( !target )
            return KOPENAPI_RESULT::Error( 404, "no sheet matches: " + want + " (sch_sheet_list)" );

        if( *target != userSheet )
        {
            tools->RunAction<SCH_SHEET_PATH*>( SCH_ACTIONS::changeSheet, &*target );
            switched = true;
        }
    }

    SCH_SCREEN*           screen = frame->GetScreen();
    const SCH_SHEET_PATH& sheet = frame->GetCurrentSheet();

    std::vector<SCH_SYMBOL*> symbols;
    std::map<wxString, int>  wiresPerNet;
    int                      wires = 0, labels = 0, items = 0;

    for( SCH_ITEM* item : screen->Items() )
    {
        items++;

        if( item->Type() == SCH_SYMBOL_T )
        {
            symbols.push_back( static_cast<SCH_SYMBOL*>( item ) );
        }
        else if( item->Type() == SCH_LINE_T && static_cast<SCH_LINE*>( item )->IsWire() )
        {
            wires++;
            const std::optional<wxString> net = item->GetConnectionName( &sheet );

            if( net && !net->IsEmpty() )
                wiresPerNet[*net]++;
        }
        else if( dynamic_cast<SCH_LABEL_BASE*>( item ) )
        {
            labels++;
        }
    }

    std::sort( symbols.begin(), symbols.end(),
               [&]( SCH_SYMBOL* a, SCH_SYMBOL* b )
               {
                   const wxString ra = a->GetRef( &sheet ), rb = b->GetRef( &sheet );
                   return ra != rb ? ra < rb : a->m_Uuid < b->m_Uuid;
               } );

    const wxString userHighlight = frame->GetHighlightedConnection();
    const bool     userModified = screen->IsContentModified();

    KOPENAPI_BENCH_HOOKS hooks;
    const BOX2I extents = frame->GetDocumentExtents( false );
    hooks.fitBox = BOX2D( VECTOR2D( extents.GetOrigin() ), VECTOR2D( extents.GetSize() ) );
    hooks.fitBox.Inflate( hooks.fitBox.GetWidth() * 0.02, hooks.fitBox.GetHeight() * 0.02 );

    hooks.option = "hidden fields shown / hidden (View > Show Hidden Fields), twice per cycle";
    hooks.setOption = [&]( bool ) { tools->RunAction( SCH_ACTIONS::toggleHiddenFields ); };

    // a fixed tenth of the symbols (1..50), first by reference
    EDA_ITEMS selectSet;

    for( size_t i = 0; i < symbols.size() && selectSet.size() < std::clamp<size_t>( symbols.size() / 10, 1, 50 ); ++i )
        selectSet.push_back( symbols[i] );

    if( selTool && !selectSet.empty() )
    {
        hooks.selectWhat = std::to_string( selectSet.size() ) + " symbols selected / selection cleared";
        hooks.select = [&]( bool aOn )
        {
            if( aOn )
                tools->RunAction<EDA_ITEMS*>( ACTIONS::selectItems, &selectSet );
            else
                tools->RunAction( ACTIONS::selectionClear );
        };
    }

    auto setHighlight = [frame, control]( const wxString& aNet )
    {
        frame->SetHighlightedConnection( aNet );

        TOOL_EVENT dummy;
        control->UpdateNetHighlighting( dummy );
    };

    // the net with the most wire segments on this sheet
    if( control && !wiresPerNet.empty() )
    {
        const auto     best = std::max_element( wiresPerNet.begin(), wiresPerNet.end(),
                                                []( const auto& a, const auto& b ) { return a.second < b.second; } );
        const wxString net = best->first;

        hooks.highlightWhat = "net " + net.ToStdString( wxConvUTF8 ) + " (" + std::to_string( best->second )
                              + " wires on the sheet) highlighted / cleared";
        hooks.highlight = [setHighlight, net]( bool aOn ) { setHighlight( aOn ? net : wxString() ); };
    }

    // the first symbol moved 1.27 mm (one grid step), then undone
    if( !symbols.empty() )
    {
        SCH_SYMBOL* symbol = symbols.front();
        hooks.editWhat = symbol->GetRef( &sheet ).ToStdString( wxConvUTF8 ) + " moved 1.27 mm (one commit), then undone";
        hooks.edit = [frame, symbol, screen]()
        {
            SCH_COMMIT commit( frame->GetToolManager() );
            commit.Modify( symbol, screen );
            symbol->Move( VECTOR2I( schIUScale.mmToIU( 1.27 ), 0 ) );
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
        if( switched )
            tools->RunAction<SCH_SHEET_PATH*>( SCH_ACTIONS::changeSheet, &userSheet );

        tools->RunAction( ACTIONS::selectionClear );

        if( !userSelection.empty() )
            tools->RunAction<EDA_ITEMS*>( ACTIONS::selectItems, &userSelection );

        if( control )
            setHighlight( userHighlight );

        // the edit scenario's undo left a redo step and maybe the modified mark
        frame->ClearUndoORRedoList( EDA_BASE_FRAME::REDO_LIST );

        if( !userModified )
            screen->SetContentModified( false );
    };

    nlohmann::json info = {
        { "editor", "sch" },
        { "document", screen->GetFileName().ToStdString( wxConvUTF8 ) },
        { "sheet", sheet.PathHumanReadable().ToStdString( wxConvUTF8 ) },
        { "items",
          { { "on_sheet", items },
            { "symbols", symbols.size() },
            { "wires", wires },
            { "labels", labels },
            { "nets_with_wires", wiresPerNet.size() } } } };

    return KopenapiViewBenchmark( frame->GetCanvas(), hooks, aArgs, info );
}


KOPENAPI_REGISTER_VIEW_BENCHMARK( "sch", h_sch_view_benchmark );
