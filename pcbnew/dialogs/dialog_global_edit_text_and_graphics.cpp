/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2012 Jean-Pierre Charras, jp.charras at wanadoo.fr
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <string_utils.h>
#include <board_commit.h>
#include <pcb_edit_frame.h>
#include <footprint_edit_frame.h>
#include <pcb_layer_box_selector.h>
#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <pcb_group.h>
#include <pcb_barcode.h>
#include <pcb_dimension.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <widgets/unit_binder.h>
#include <widgets/font_choice.h>
#include <widgets/bitmap_button.h>
#include <bitmaps.h>
#include <gr_text.h>
#include <tool/tool_manager.h>
#include <tools/global_edit_tool.h>
#include <tools/footprint_editor_control.h>
#include <dialog_global_edit_text_and_graphics_base.h>
#include <global_edit_text_and_graphics.h>

// Columns of layer classes grid
enum
{
    COL_CLASS_NAME = 0,
    COL_LINE_THICKNESS,
    COL_TEXT_WIDTH,
    COL_TEXT_HEIGHT,
    COL_TEXT_THICKNESS,
    COL_TEXT_ITALIC,
    COL_TEXT_UPRIGHT
};

enum
{
    ROW_HEADER = 0,
    ROW_SILK,
    ROW_COPPER,
    ROW_EDGES,
    ROW_COURTYARD,
    ROW_FAB,
    ROW_OTHERS
};


static wxString   g_referenceFilter;
static wxString   g_footprintFilter;


class DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS : public DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS_BASE
{
public:
    DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS( PCB_BASE_EDIT_FRAME* parent );
    ~DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS() override;

protected:
    void onActionButtonChange( wxCommandEvent& event ) override;
    void onSpecifiedValueUpdateUI( wxUpdateUIEvent& event ) override;
    void onDimensionItemCheckbox( wxCommandEvent& aEvent ) override;
    void onAutoTextThickness( wxCommandEvent& aEvent ) override;
    void onTextSize( wxCommandEvent& aEvent ) override;

    void OnLayerFilterSelect( wxCommandEvent& event ) override
    {
        m_layerFilterOpt->SetValue( true );
    }
    void OnReferenceFilterText( wxCommandEvent& event ) override
    {
        m_referenceFilterOpt->SetValue( true );
    }
    void OnFootprintFilterText( wxCommandEvent& event ) override
    {
        m_footprintFilterOpt->SetValue( true );
    }

    bool TransferDataToWindow() override;
    bool TransferDataFromWindow() override;


private:
    PCB_BASE_EDIT_FRAME*   m_parent;
    BOARD_DESIGN_SETTINGS* m_brdSettings;
    PCB_SELECTION          m_selection;
    bool                   m_isBoardEditor;

    UNIT_BINDER            m_lineWidth;
    UNIT_BINDER            m_textWidth;
    UNIT_BINDER            m_textHeight;
    UNIT_BINDER            m_thickness;
};


DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS( PCB_BASE_EDIT_FRAME* parent ) :
        DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS_BASE( parent ),
        m_lineWidth( parent, m_lineWidthLabel, m_LineWidthCtrl, m_lineWidthUnits ),
        m_textWidth( parent, m_SizeXlabel, m_SizeXCtrl, m_SizeXunit ),
        m_textHeight( parent, m_SizeYlabel, m_SizeYCtrl, m_SizeYunit ),
        m_thickness( parent, m_ThicknessLabel, m_ThicknessCtrl, m_ThicknessUnit )
{
    m_parent = parent;
    m_brdSettings = &m_parent->GetDesignSettings();
    m_isBoardEditor = dynamic_cast<PCB_EDIT_FRAME*>( m_parent ) != nullptr;

    if( !m_isBoardEditor )
    {
        m_footprintTexts->SetLabel( _( "Text items" ) );
        m_footprintGraphics->SetLabel( _( "Graphic items" ) );
        m_footprintDimensions->SetLabel( _( "Dimension items" ) );

        m_boardText->Show( false );
        m_boardGraphics->Show( false );
        m_boardDimensions->Show( false );

        m_referenceFilterOpt->Show( false );
        m_referenceFilter->Show( false );
        m_footprintFilterOpt->Show( false );
        m_footprintFilter->Show( false );
    }

    // Allow indeterminate state in font control and rebuild list
    m_fontCtrl->SetHasIndeterminateChoice();
    m_fontCtrl->RefreshFonts();

    m_autoTextThickness->SetIsCheckButton();
    m_autoTextThickness->SetBitmap( KiBitmapBundle( BITMAPS::edit_cmp_symb_links ) );

    m_layerFilter->SetBoardFrame( m_parent );
    m_layerFilter->SetLayersHotkeys( false );
    m_layerFilter->Resync();

    m_LayerCtrl->SetBoardFrame( m_parent );
    m_LayerCtrl->SetLayersHotkeys( false );
    m_LayerCtrl->SetUndefinedLayerName( INDETERMINATE_ACTION );
    m_LayerCtrl->Resync();

    m_grid->SetCellHighlightPenWidth( 0 );
    m_grid->SetDefaultCellFont( KIUI::GetInfoFont( this ) );
    m_grid->SetDefaultRowSize( m_grid->GetDefaultRowSize() - FromDIP( 2 ) );

    SetupStandardButtons( { { wxID_OK, _( "Apply and Close" ) },
                            { wxID_CANCEL, _( "Close" ) } } );

    finishDialogSettings();
}


DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::~DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS()
{
    if( m_isBoardEditor )
    {
        g_referenceFilter = m_referenceFilter->GetValue();
        g_footprintFilter = m_footprintFilter->GetValue();
    }
}


bool DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::TransferDataToWindow()
{
    PCB_SELECTION_TOOL* selTool = m_parent->GetToolManager()->GetTool<PCB_SELECTION_TOOL>();
    m_selection                 = selTool->GetSelection();

    if( m_isBoardEditor )
    {
        // SetValue() generates events, ChangeValue() does not
        m_referenceFilter->ChangeValue( g_referenceFilter );
        m_footprintFilter->ChangeValue( g_footprintFilter );
    }

    m_lineWidth.SetValue( INDETERMINATE_ACTION );

    m_fontCtrl->SetStringSelection( INDETERMINATE_ACTION );

    m_textWidth.SetValue( INDETERMINATE_ACTION );
    m_textHeight.SetValue( INDETERMINATE_ACTION );
    m_thickness.SetValue( INDETERMINATE_ACTION );
    m_autoTextThickness->Check( false );
    m_thickness.Enable( true );
    m_bold->Set3StateValue( wxCHK_UNDETERMINED );
    m_italic->Set3StateValue( wxCHK_UNDETERMINED );
    m_keepUpright->Set3StateValue( wxCHK_UNDETERMINED );
    m_visible->Set3StateValue( wxCHK_UNDETERMINED );
    m_LayerCtrl->SetLayerSelection( UNDEFINED_LAYER );

    wxCommandEvent dummy;
    onActionButtonChange( dummy );

#define SET_INT_VALUE( aRow, aCol, aValue ) \
        m_grid->SetCellValue( aRow, aCol, m_parent->StringFromValue( aValue, true ) )

#define SET_BOOL_VALUE( aRow, aCol, aValue ) \
        attr = new wxGridCellAttr; \
        attr->SetRenderer( new wxGridCellBoolRenderer() ); \
        attr->SetAlignment( wxALIGN_CENTER, wxALIGN_CENTER ); \
        attr->SetReadOnly(); \
        m_grid->SetAttr( aRow, aCol, attr ); \
        m_grid->SetCellValue( aRow, aCol, ( aValue ) ? wxT( "1" ) : wxT( "" ) )

    const BOARD_DESIGN_SETTINGS& bds = m_parent->GetBoard()->GetDesignSettings();
    wxGridCellAttr* attr;

    m_grid->SetCellValue( ROW_SILK,      COL_CLASS_NAME, _( "Silk Layers" ) );
    m_grid->SetCellValue( ROW_COPPER,    COL_CLASS_NAME, _( "Copper Layers" ) );
    m_grid->SetCellValue( ROW_EDGES,     COL_CLASS_NAME, _( "Edge Cuts" ) );
    m_grid->SetCellValue( ROW_COURTYARD, COL_CLASS_NAME, _( "Courtyards" ) );
    m_grid->SetCellValue( ROW_FAB,       COL_CLASS_NAME, _( "Fab Layers" ) );
    m_grid->SetCellValue( ROW_OTHERS,    COL_CLASS_NAME, _( "Other Layers" ) );

    m_grid->SetCellValue( ROW_HEADER, COL_LINE_THICKNESS, _( "Line Thickness" ) );
    SET_INT_VALUE( ROW_SILK,      COL_LINE_THICKNESS, bds.m_LineThickness[ LAYER_CLASS_SILK ] );
    SET_INT_VALUE( ROW_COPPER,    COL_LINE_THICKNESS, bds.m_LineThickness[ LAYER_CLASS_COPPER ] );
    SET_INT_VALUE( ROW_EDGES,     COL_LINE_THICKNESS, bds.m_LineThickness[ LAYER_CLASS_EDGES ] );
    SET_INT_VALUE( ROW_COURTYARD, COL_LINE_THICKNESS, bds.m_LineThickness[ LAYER_CLASS_COURTYARD ] );
    SET_INT_VALUE( ROW_FAB,       COL_LINE_THICKNESS, bds.m_LineThickness[ LAYER_CLASS_FAB ] );
    SET_INT_VALUE( ROW_OTHERS,    COL_LINE_THICKNESS, bds.m_LineThickness[ LAYER_CLASS_OTHERS ] );

    m_grid->SetCellValue( ROW_HEADER, COL_TEXT_WIDTH, _( "Text Width" ) );
    SET_INT_VALUE( ROW_SILK,   COL_TEXT_WIDTH, bds.m_TextSize[ LAYER_CLASS_SILK ].x );
    SET_INT_VALUE( ROW_COPPER, COL_TEXT_WIDTH, bds.m_TextSize[ LAYER_CLASS_COPPER ].x );
    SET_INT_VALUE( ROW_FAB,    COL_TEXT_WIDTH, bds.m_TextSize[ LAYER_CLASS_FAB ].x );
    SET_INT_VALUE( ROW_OTHERS, COL_TEXT_WIDTH, bds.m_TextSize[ LAYER_CLASS_OTHERS ].x );

    m_grid->SetCellValue( ROW_HEADER, COL_TEXT_HEIGHT, _( "Text Height" ) );
    SET_INT_VALUE( ROW_SILK,   COL_TEXT_HEIGHT, bds.m_TextSize[ LAYER_CLASS_SILK ].y );
    SET_INT_VALUE( ROW_COPPER, COL_TEXT_HEIGHT, bds.m_TextSize[ LAYER_CLASS_COPPER ].y );
    SET_INT_VALUE( ROW_FAB,    COL_TEXT_HEIGHT, bds.m_TextSize[ LAYER_CLASS_FAB ].y );
    SET_INT_VALUE( ROW_OTHERS, COL_TEXT_HEIGHT, bds.m_TextSize[ LAYER_CLASS_OTHERS ].y );

    m_grid->SetCellValue( ROW_HEADER, COL_TEXT_THICKNESS, _( "Text Thickness" ) );
    SET_INT_VALUE( ROW_SILK,   COL_TEXT_THICKNESS, bds.m_TextThickness[ LAYER_CLASS_SILK ] );
    SET_INT_VALUE( ROW_COPPER, COL_TEXT_THICKNESS, bds.m_TextThickness[ LAYER_CLASS_COPPER ] );
    SET_INT_VALUE( ROW_FAB,    COL_TEXT_THICKNESS, bds.m_TextThickness[ LAYER_CLASS_FAB ] );
    SET_INT_VALUE( ROW_OTHERS, COL_TEXT_THICKNESS, bds.m_TextThickness[ LAYER_CLASS_OTHERS ] );

    attr = new wxGridCellAttr;
    attr->SetAlignment( wxALIGN_CENTER, wxALIGN_CENTER ); \
    m_grid->SetAttr( ROW_HEADER, COL_TEXT_ITALIC, attr ); \
    m_grid->SetCellValue(  ROW_HEADER, COL_TEXT_ITALIC, _( "Italic" ) );
    SET_BOOL_VALUE(  ROW_SILK,   COL_TEXT_ITALIC, bds.m_TextItalic[ LAYER_CLASS_SILK ] );
    SET_BOOL_VALUE(  ROW_COPPER, COL_TEXT_ITALIC, bds.m_TextItalic[ LAYER_CLASS_COPPER ] );
    SET_BOOL_VALUE(  ROW_FAB,    COL_TEXT_ITALIC, bds.m_TextItalic[ LAYER_CLASS_FAB ] );
    SET_BOOL_VALUE(  ROW_OTHERS, COL_TEXT_ITALIC, bds.m_TextItalic[ LAYER_CLASS_OTHERS ] );

    attr = new wxGridCellAttr;
    attr->SetAlignment( wxALIGN_CENTER, wxALIGN_CENTER ); \
    m_grid->SetAttr( ROW_HEADER, COL_TEXT_UPRIGHT, attr ); \
    m_grid->SetCellValue(  ROW_HEADER, COL_TEXT_UPRIGHT, _( "Keep Upright" ) );
    SET_BOOL_VALUE(  ROW_SILK,   COL_TEXT_UPRIGHT, bds.m_TextUpright[ LAYER_CLASS_SILK ] );
    SET_BOOL_VALUE(  ROW_COPPER, COL_TEXT_UPRIGHT, bds.m_TextUpright[ LAYER_CLASS_COPPER ] );
    SET_BOOL_VALUE(  ROW_FAB,    COL_TEXT_UPRIGHT, bds.m_TextUpright[ LAYER_CLASS_FAB ] );
    SET_BOOL_VALUE(  ROW_OTHERS, COL_TEXT_UPRIGHT, bds.m_TextUpright[ LAYER_CLASS_OTHERS ] );

    return true;

#undef SET_INT_VALUE
#undef SET_BOOL_VALUE
}


void DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::onActionButtonChange( wxCommandEvent& event )
{
    // Update the UNIT_BINDER controls if the action to take is changed
    bool enable = m_setToSpecifiedValues->GetValue();

    m_LayerLabel->Enable( enable );
    m_LayerCtrl->Enable( enable );

    m_lineWidth.Enable( enable );
    m_textWidth.Enable( enable );
    m_textHeight.Enable( enable );
    m_thickness.Enable( enable && !m_autoTextThickness->IsChecked() );

    m_fontLabel->Enable( enable );
    m_fontCtrl->Enable( enable );

    m_centerOnFP->Enable( enable );

    enable = !enable;
    m_grid->Enable( enable );
}


void DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::onSpecifiedValueUpdateUI( wxUpdateUIEvent& event )
{
    // Update the UI for the elements inside the use specified values sizer
    event.Enable( m_setToSpecifiedValues->GetValue() );
}


void DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::onDimensionItemCheckbox( wxCommandEvent& aEvent )
{
    if( m_footprintDimensions->GetValue() || m_boardDimensions->GetValue() )
        m_setToLayerDefaults->SetLabel( _( "Set to layer and dimension default values:" ) );
    else
        m_setToLayerDefaults->SetLabel( _( "Set to layer default values:" ) );
}


void DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::onAutoTextThickness( wxCommandEvent& aEvent )
{
    if( aEvent.IsChecked() )
    {
        m_autoTextThickness->Check( true );

        wxCommandEvent dummy;
        onTextSize( dummy );

        if( m_textWidth.IsIndeterminate() || m_textHeight.IsIndeterminate() )
            m_thickness.SetValue( _( "(auto)" ) );

        m_thickness.Enable( false );
    }
    else
    {
        m_thickness.Enable( true );
        m_thickness.SetValue( INDETERMINATE_ACTION );
    }
}


void DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::onTextSize( wxCommandEvent& aEvent )
{
    if( !m_autoTextThickness->IsChecked() )
        return;

    if( m_textWidth.IsIndeterminate() || m_textHeight.IsIndeterminate() )
        return;

    int size = std::min( m_textWidth.GetValue(), m_textHeight.GetValue() );

    m_thickness.SetValue( GetPenSizeForNormal( size ) );
}


bool DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::TransferDataFromWindow()
{
    int minTextSize = pcbIUScale.mmToIU( TEXT_MIN_SIZE_MM );
    int maxTextSize = pcbIUScale.mmToIU( TEXT_MAX_SIZE_MM );

    if( !m_textWidth.Validate( minTextSize, maxTextSize )
        || !m_textHeight.Validate( minTextSize, maxTextSize ) )
    {
        return false;
    }

    GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS opt;

    opt.references = m_references->GetValue();
    opt.values = m_values->GetValue();
    opt.otherFootprintFields = m_otherFootprintFields->GetValue();
    opt.footprintTexts = m_footprintTexts->GetValue();
    opt.footprintGraphics = m_footprintGraphics->GetValue();
    opt.footprintDimensions = m_footprintDimensions->GetValue();
    opt.boardTexts = m_boardText->GetValue();
    opt.boardGraphics = m_boardGraphics->GetValue();
    opt.boardDimensions = m_boardDimensions->GetValue();

    opt.selectedOnly = m_selectedItemsFilter->GetValue();

    if( m_layerFilterOpt->GetValue() && m_layerFilter->GetLayerSelection() != UNDEFINED_LAYER )
        opt.layerFilter = ToLAYER_ID( m_layerFilter->GetLayerSelection() );

    if( m_isBoardEditor && m_referenceFilterOpt->GetValue() )
        opt.referenceFilter = m_referenceFilter->GetValue();

    if( m_isBoardEditor && m_footprintFilterOpt->GetValue() )
        opt.footprintFilter = m_footprintFilter->GetValue();

    opt.setToLayerDefaults = !m_setToSpecifiedValues->GetValue();

    if( m_LayerCtrl->GetLayerSelection() != UNDEFINED_LAYER )
        opt.layer = ToLAYER_ID( m_LayerCtrl->GetLayerSelection() );

    if( !m_lineWidth.IsIndeterminate() )
        opt.lineWidth = m_lineWidth.GetIntValue();

    if( !m_textWidth.IsIndeterminate() )
        opt.textWidth = m_textWidth.GetIntValue();

    if( !m_textHeight.IsIndeterminate() )
        opt.textHeight = m_textHeight.GetIntValue();

    opt.autoTextThickness = m_autoTextThickness->IsChecked();

    if( !m_thickness.IsIndeterminate() )
        opt.textThickness = m_thickness.GetIntValue();

    if( m_bold->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.bold = m_bold->GetValue();

    if( m_italic->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.italic = m_italic->GetValue();

    if( m_keepUpright->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.keepUpright = m_keepUpright->GetValue();

    if( m_visible->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.visible = m_visible->GetValue();

    opt.centerOnFootprint = m_centerOnFP->GetValue() == wxCHK_CHECKED;

    if( m_fontCtrl->GetStringSelection() != INDETERMINATE_ACTION )
    {
        opt.font = [this]( bool aBold, bool aItalic )
        {
            return m_fontCtrl->GetFontSelection( aBold, aItalic );
        };
    }

    BOARD_COMMIT commit( m_parent );

    GlobalEditTextAndGraphics( *m_parent->GetBoard(), commit, *m_brdSettings, opt, m_isBoardEditor );

    commit.Push( _( "Edit Text and Graphics" ) );
    m_parent->GetCanvas()->Refresh();

    return true;
}


int GLOBAL_EDIT_TOOL::EditTextAndGraphics( const TOOL_EVENT& aEvent )
{
    PCB_EDIT_FRAME* editFrame = getEditFrame<PCB_EDIT_FRAME>();
    DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS dlg( editFrame );

    dlg.ShowModal();
    return 0;
}


int FOOTPRINT_EDITOR_CONTROL::EditTextAndGraphics( const TOOL_EVENT& aEvent )
{
    FOOTPRINT_EDIT_FRAME* editFrame = getEditFrame<FOOTPRINT_EDIT_FRAME>();
    DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS dlg( editFrame );

    dlg.ShowModal();
    return 0;
}


