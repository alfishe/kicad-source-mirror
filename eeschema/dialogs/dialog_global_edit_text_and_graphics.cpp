/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
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

#include <dialog_global_edit_text_and_graphics_base.h>
#include <string_utils.h>
#include <sch_symbol.h>
#include <sch_connection.h>
#include <sch_edit_frame.h>
#include <sch_shape.h>
#include <sch_line.h>
#include <sch_junction.h>
#include <sch_sheet.h>
#include <sch_sheet_pin.h>
#include <schematic.h>
#include <sch_commit.h>
#include <sch_global_edit_text_and_graphics.h>
#include <tool/tool_manager.h>
#include <tools/sch_selection_tool.h>
#include <tools/sch_edit_tool.h>
#include <widgets/unit_binder.h>
#include <widgets/font_choice.h>

static wxString   g_fieldnameFilter;
static wxString   g_referenceFilter;
static wxString   g_symbolFilter;
static wxString   g_netFilter;


class DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS : public DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS_BASE
{
public:
    DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS( SCH_EDIT_FRAME* parent );
    ~DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS() override;

protected:
    void OnReferenceFilterText( wxCommandEvent& event ) override
    {
        m_referenceFilterOpt->SetValue( true );
    }
    void OnSymbolFilterText( wxCommandEvent& event ) override
    {
        m_symbolFilterOpt->SetValue( true );
    }
    void OnFieldNameFilterText( wxCommandEvent& event ) override
    {
        m_fieldnameFilterOpt->SetValue( true );
    }
    void OnNetFilterText( wxCommandEvent& event ) override
    {
        m_netFilterOpt->SetValue( true );
    }

    bool TransferDataToWindow() override;
    bool TransferDataFromWindow() override;

private:
    SCH_EDIT_FRAME*        m_parent;
    SCH_SELECTION          m_selection;

    UNIT_BINDER            m_textSize;
    UNIT_BINDER            m_lineWidth;
    UNIT_BINDER            m_junctionSize;
};


DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS( SCH_EDIT_FRAME* parent ) :
        DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS_BASE( parent ),
        m_textSize( parent, m_textSizeLabel, m_textSizeCtrl, m_textSizeUnits, true ),
        m_lineWidth( parent, m_lineWidthLabel, m_LineWidthCtrl, m_lineWidthUnits, true ),
        m_junctionSize( parent, m_dotSizeLabel, m_dotSizeCtrl, m_dotSizeUnits, true )
{
    m_parent = parent;

    m_lineStyle->Append( INDETERMINATE_ACTION );

    // Allow indeterminate state in font control and rebuild list
    m_fontCtrl->SetHasIndeterminateChoice();
    m_fontCtrl->RefreshFonts();

    m_textColorSwatch->SetSwatchColor( COLOR4D::UNSPECIFIED, false );
    m_textColorSwatch->SetDefaultColor( COLOR4D::UNSPECIFIED );
    m_colorSwatch->SetSwatchColor( COLOR4D::UNSPECIFIED, false );
    m_colorSwatch->SetDefaultColor( COLOR4D::UNSPECIFIED );
    m_fillColorSwatch->SetSwatchColor( COLOR4D::UNSPECIFIED, false );
    m_fillColorSwatch->SetDefaultColor( COLOR4D::UNSPECIFIED );
    m_dotColorSwatch->SetSwatchColor( COLOR4D::UNSPECIFIED, false );
    m_dotColorSwatch->SetDefaultColor( COLOR4D::UNSPECIFIED );

    m_staticText12->SetFont( KIUI::GetSmallInfoFont( this ).Italic() );
    m_staticText13->SetFont( KIUI::GetSmallInfoFont( this ).Italic() );
    m_staticText14->SetFont( KIUI::GetSmallInfoFont( this ).Italic() );
    m_staticText15->SetFont( KIUI::GetSmallInfoFont( this ).Italic() );
    m_staticText16->SetFont( KIUI::GetSmallInfoFont( this ).Italic() );

    SetupStandardButtons();

    finishDialogSettings();
}


DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::~DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS()
{
    g_fieldnameFilter = m_fieldnameFilter->GetValue();
    g_referenceFilter = m_referenceFilter->GetValue();
    g_symbolFilter = m_symbolFilter->GetValue();
    g_netFilter = m_netFilter->GetValue();
}


bool DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::TransferDataToWindow()
{
    SCH_SELECTION_TOOL* selectionTool = m_parent->GetToolManager()->GetTool<SCH_SELECTION_TOOL>();
    m_selection = selectionTool->GetSelection();

    // SetValue() generates events, ChangeValue() does not
    m_fieldnameFilter->ChangeValue( g_fieldnameFilter );
    m_referenceFilter->ChangeValue( g_referenceFilter );
    m_symbolFilter->ChangeValue( g_symbolFilter );
    m_netFilter->ChangeValue( g_netFilter );

    if( g_netFilter.IsEmpty() && !m_parent->GetHighlightedConnection().IsEmpty() )
        m_netFilter->SetValue( m_parent->GetHighlightedConnection() );

    m_fontCtrl->SetStringSelection( INDETERMINATE_ACTION );

    m_textSize.SetValue( INDETERMINATE_ACTION );
    m_orientation->SetStringSelection( INDETERMINATE_ACTION );
    m_hAlign->SetStringSelection( INDETERMINATE_ACTION );
    m_vAlign->SetStringSelection( INDETERMINATE_ACTION );
    m_italic->Set3StateValue( wxCHK_UNDETERMINED );
    m_bold->Set3StateValue( wxCHK_UNDETERMINED );
    m_visible->Set3StateValue( wxCHK_UNDETERMINED );
    m_showFieldNames->Set3StateValue( wxCHK_UNDETERMINED );
    m_lineWidth.SetValue( INDETERMINATE_ACTION );
    m_lineStyle->SetStringSelection( INDETERMINATE_ACTION );
    m_junctionSize.SetValue( INDETERMINATE_ACTION );
    m_setColor->SetValue( false );
    m_setFillColor->SetValue( false );
    m_setDotColor->SetValue( false );

    return true;
}


bool DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS::TransferDataFromWindow()
{
    if( !m_textSize.Validate( 1.0, 10000.0, EDA_UNITS::MILS ) )  // 1 mil .. 10 inches
        return false;

    SCH_GLOBAL_EDIT_OPTIONS opt;

    opt.references = m_references->GetValue();
    opt.values = m_values->GetValue();
    opt.otherFields = m_otherFields->GetValue();
    opt.wires = m_wires->GetValue();
    opt.buses = m_buses->GetValue();
    opt.globalLabels = m_globalLabels->GetValue();
    opt.hierLabels = m_hierLabels->GetValue();
    opt.labelFields = m_labelFields->GetValue();
    opt.sheetTitles = m_sheetTitles->GetValue();
    opt.sheetFields = m_sheetFields->GetValue();
    opt.sheetPins = m_sheetPins->GetValue();
    opt.sheetBorders = m_sheetBorders->GetValue();
    opt.textAndGraphics = m_schTextAndGraphics->GetValue();

    opt.selectedOnly = m_selectedFilterOpt->GetValue();

    if( m_fieldnameFilterOpt->GetValue() )
        opt.fieldNameFilter = m_fieldnameFilter->GetValue();

    if( m_referenceFilterOpt->GetValue() )
        opt.referenceFilter = m_referenceFilter->GetValue();

    if( m_symbolFilterOpt->GetValue() )
        opt.symbolFilter = m_symbolFilter->GetValue();

    if( m_netFilterOpt->GetValue() )
        opt.netFilter = m_netFilter->GetValue();

    if( m_typeFilterOpt->GetValue() )
        opt.powerSymbols = m_typeFilter->GetSelection() == 1;

    if( !m_textSize.IsIndeterminate() )
        opt.textSize = m_textSize.GetIntValue();

    if( m_setTextColor->GetValue() )
        opt.textColor = m_textColorSwatch->GetSwatchColor();

    if( m_hAlign->GetStringSelection() != INDETERMINATE_ACTION )
        opt.hAlign = EDA_TEXT::MapHorizJustify( m_hAlign->GetSelection() - 1 );

    if( m_vAlign->GetStringSelection() != INDETERMINATE_ACTION )
        opt.vAlign = EDA_TEXT::MapVertJustify( m_vAlign->GetSelection() - 1 );

    if( m_italic->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.italic = m_italic->GetValue();

    if( m_bold->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.bold = m_bold->GetValue();

    if( m_fontCtrl->GetStringSelection() != INDETERMINATE_ACTION )
    {
        opt.font = [this]( bool aBold, bool aItalic )
        {
            return m_fontCtrl->GetFontSelection( aBold, aItalic );
        };
    }

    if( m_orientation->GetStringSelection() != INDETERMINATE_ACTION )
        opt.labelOrientation = (SPIN_STYLE::SPIN) m_orientation->GetSelection();

    if( m_visible->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.visible = m_visible->GetValue();

    if( m_showFieldNames->Get3StateValue() != wxCHK_UNDETERMINED )
        opt.showFieldNames = m_showFieldNames->GetValue();

    if( !m_lineWidth.IsIndeterminate() )
        opt.lineWidth = m_lineWidth.GetIntValue();

    if( m_lineStyle->GetStringSelection() != INDETERMINATE_ACTION )
        opt.lineStyle = (LINE_STYLE) m_lineStyle->GetSelection();

    if( m_setColor->GetValue() )
        opt.lineColor = m_colorSwatch->GetSwatchColor();

    if( m_setFillColor->GetValue() )
        opt.fillColor = m_fillColorSwatch->GetSwatchColor();

    if( !m_junctionSize.IsIndeterminate() )
        opt.junctionSize = m_junctionSize.GetIntValue();

    if( m_setDotColor->GetValue() )
        opt.junctionColor = m_dotColorSwatch->GetSwatchColor();

    SCH_SHEET_PATH currentSheet = m_parent->GetCurrentSheet();
    SCH_COMMIT     commit( m_parent );

    SchGlobalEditTextAndGraphics( m_parent->Schematic(), commit, opt,
                                  [this]( const SCH_SHEET_PATH& aSheet )
                                  {
                                      m_parent->SetCurrentSheet( aSheet );
                                  } );

    if( !commit.Empty() )
    {
        commit.Push( _( "Edit Text and Graphics" ) );
        m_parent->HardRedraw();
    }

    // Reset the view to where we left the user
    m_parent->SetCurrentSheet( currentSheet );
    m_parent->Refresh();

    return true;
}


int SCH_EDIT_TOOL::GlobalEdit( const TOOL_EVENT& aEvent )
{
    DIALOG_GLOBAL_EDIT_TEXT_AND_GRAPHICS dlg( m_frame );
    dlg.ShowModal();
    return 0;
}


