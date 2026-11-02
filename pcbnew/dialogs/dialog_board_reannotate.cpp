/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2020 Brian Piccioni brian@documenteddesigns.com
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 * @author Brian Piccioni <brian@documenteddesigns.com>
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

#include "dialog_board_reannotate.h"

#include <algorithm>

#include <wx/tokenzr.h>
#include <wx/valtext.h>

#include <base_units.h>
#include <bitmaps.h>
#include <board_commit.h>
#include <confirm.h>
#include <ctype.h>
#include <footprint.h>
#include <gal/graphics_abstraction_layer.h>
#include <string_utils.h>  // StrNumCmp
#include <kiface_base.h>
#include <pcbnew_settings.h>
#include <refdes_utils.h>
#include <richio.h>
#include <tool/grid_menu.h>
#include <widgets/wx_html_report_panel.h>


wxString ActionMessage[] = {
    "",                                 // UPDATE_REFDES
    _( "(not updated)" ),               // EMPTY_REFDES
    _( "(unannotated; not updated)" ),  // INVALID_REFDES
    _( "(excluded)" )                   // EXCLUDE_REFDES
};


DIALOG_BOARD_REANNOTATE::DIALOG_BOARD_REANNOTATE( PCB_EDIT_FRAME* aParentFrame ) :
        DIALOG_BOARD_REANNOTATE_BASE( aParentFrame ),
        m_frame( aParentFrame ),
        m_footprints( aParentFrame->GetBoard()->Footprints() )
{
    // Init bitmaps associated to some wxRadioButton
    reannotate_down_right_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_right_down ) );
    reannotate_right_down_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_left_down ) );
    reannotate_down_left_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_right_up ) );
    reannotate_left_down_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_left_up ) );
    reannotate_up_right_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_down_left ) );
    reannotate_right_up_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_up_left ) );
    reannotate_up_left_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_down_right ) );
    reannotate_left_up_bitmap->SetBitmap( KiBitmapBundle( BITMAPS::reannotate_up_right ) );

    m_FrontRefDesStart->SetValidator( wxTextValidator( wxFILTER_DIGITS ) );
    m_BackRefDesStart->SetValidator( wxTextValidator( wxFILTER_DIGITS ) );

    SetupStandardButtons( { { wxID_OK,     _( "Reannotate PCB" ) },
                            { wxID_CANCEL, _( "Close" )          } } );

    wxArrayString gridslist;
    GRID_MENU::BuildChoiceList( &gridslist, m_frame->GetWindowSettings( m_frame->config() ), aParentFrame );

    m_GridChoice->Set( gridslist );

    int gridIndex = m_frame->config()->m_Window.grid.last_size_idx;

    if( gridIndex >= 0 && gridIndex < (int) m_GridChoice->GetCount() )
        m_GridChoice->SetSelection( gridIndex );
    else
        m_GridChoice->SetSelection( 0 );

    m_ExcludeList->SetToolTip( m_ExcludeListText->GetToolTipText() );
    m_GridChoice->SetToolTip( m_SortGridText->GetToolTipText() );

    m_MessageWindow->SetFileName( Prj().GetProjectPath() + wxT( "report.txt" ) );

    finishDialogSettings();
}


void DIALOG_BOARD_REANNOTATE::OnCloseClick( wxCommandEvent& event )
{
    EndDialog( wxID_OK );
}


bool DIALOG_BOARD_REANNOTATE::TransferDataToWindow()
{
    PCB_SELECTION selection = m_frame->GetToolManager()->GetTool<PCB_SELECTION_TOOL>()->GetSelection();

    if( !selection.Empty() )
        m_AnnotateSelection->SetValue( true );

    // Ensure m_GridChoice selection validity
    // If not, the choice 0 is arbitrary
    if( m_GridChoice->GetSelection() < 0 || m_GridChoice->GetSelection() >= (int)m_GridChoice->GetCount() )
        m_GridChoice->SetSelection( 0 );

    return true;
}


void DIALOG_BOARD_REANNOTATE::FilterPrefix( wxTextCtrl* aPrefix )
{
    std::string tmps = VALIDPREFIX;

    if( aPrefix->GetValue().empty() )
        return; //Should never happen

    char lastc = aPrefix->GetValue().Last();

    if( isalnum( (int) lastc ) )
        return;

    if( tmps.find( lastc ) != std::string::npos )
        return;

    tmps = aPrefix->GetValue();
    aPrefix->Clear();
    tmps.pop_back();
    aPrefix->AppendText( tmps );
}


void DIALOG_BOARD_REANNOTATE::FilterFrontPrefix( wxCommandEvent& event )
{
    FilterPrefix( m_FrontPrefix );
}


void DIALOG_BOARD_REANNOTATE::FilterBackPrefix( wxCommandEvent& event )
{
    FilterPrefix( m_BackPrefix );
}


void DIALOG_BOARD_REANNOTATE::OnApplyClick( wxCommandEvent& event )
{
    m_MessageWindow->SetLazyUpdate( true );

    if( ReannotateBoard() )
    {
        ShowReport( _( "PCB successfully reannotated" ), RPT_SEVERITY_ACTION );
        ShowReport( _( "PCB annotation changes should be synchronized with schematic using "
                       "\"Update Schematic from PCB\"." ), RPT_SEVERITY_WARNING );
    }

    m_MessageWindow->SetLazyUpdate( false );
    m_MessageWindow->Flush( false );
    m_frame->GetCanvas()->Refresh(); // Redraw
    m_frame->OnModify();             // Need to save file on exit.
}


wxString empty_str()
{
    return wxT( "<i>" ) + _( "unannotated footprint" ) + wxT( "</i>" );
}


wxString unknown_str()
{
    return wxT( "<i>" ) + _( "unknown" ) + wxT( "</i>" );
}


wxString DIALOG_BOARD_REANNOTATE::CoordTowxString( int aX, int aY )
{
    return wxString::Format( wxT( "%s, %s" ),
                             m_frame->MessageTextFromValue( aX ),
                             m_frame->MessageTextFromValue( aY ) );
}


void DIALOG_BOARD_REANNOTATE::ShowReport( const wxString& aMessage, SEVERITY aSeverity )
{
    wxStringTokenizer msgs( aMessage, "\n" );

    while( msgs.HasMoreTokens() )
        m_MessageWindow->Report( msgs.GetNextToken(), aSeverity );
}


void DIALOG_BOARD_REANNOTATE::LogChangePlan()
{
    wxString message;

    message = _( "Reference Designator Prefixes in Use" );
    message += wxT( "<br/>-------------------------------------------------------------<br/>" );

    int i = 1;

    for( const REFDES_PREFIX_INFO& info : m_reannotator->PrefixInfos() ) // Show all the types of refdes
        message += info.RefDesPrefix + ( ( i++ % 16 ) == 0 ? wxT( "<br/>" ) : wxS( " " ) );

    message += wxT( "<br/>" );

    if( !m_excludeArray.empty() )
    {
        wxString excludes;

        for( wxString& exclude : m_excludeArray ) // Show the refdes we are excluding
            excludes += exclude + wxS( " " );

        message += wxString::Format( _( "(Excluding %s from reannotation.)" ), excludes );
    }

    ShowReport( message + wxT( "<br/>" ), RPT_SEVERITY_INFO );

    message = _( "Change Log" );
    message += wxT( "<br/>-------------------------------------------------------------<br/>" );

    for( const REFDES_CHANGE& change : m_reannotator->Changes() )
    {
        if( change.Action != UPDATE_REFDES )
        {
            message += wxString::Format( wxT( "%s  <i>%s</i><br/>" ),
                                         change.OldRefDesString.IsEmpty() ? empty_str() : change.OldRefDesString,
                                         ActionMessage[change.Action] );
        }
        else
        {
            message += wxString::Format( wxT( "%s -> %s<br/>" ),
                                         change.OldRefDesString.IsEmpty() ? empty_str() : change.OldRefDesString,
                                         change.NewRefDes.IsEmpty() ? empty_str() : change.NewRefDes );
        }
    }

    ShowReport( message, RPT_SEVERITY_ACTION );
}


void DIALOG_BOARD_REANNOTATE::LogFootprints( const std::vector<REFDES_INFO>& aFootprints )
{
    wxString message = aFootprints.front().Front ? _( "Front Footprints" ) : _( "Back Footprints" );
    message += ' ';

    if( m_locationChoice->GetSelection() == 0 )
        message += _( "(sorted by footprint location)" );
    else
        message += _( "(sorted by reference designator location)" );

    message += wxT( "<br/>-------------------------------------------------------------" );

    int i = 1;

    for( const REFDES_INFO& fp : aFootprints )
    {
        message += wxString::Format( _( "<br/>%d %s at %s (rounded to %s)" ),
                                     i++,
                                     fp.RefDesString.IsEmpty() ? empty_str() : fp.RefDesString,
                                     CoordTowxString( fp.x, fp.y ),
                                     CoordTowxString( fp.roundedx, fp.roundedy ) );
    }

    ShowReport( message + wxT( "<br/>" ), RPT_SEVERITY_INFO );
}


bool DIALOG_BOARD_REANNOTATE::ReannotateBoard()
{
    wxString message1, message2, badrefdes;

    if( !BuildFootprintList() )
    {
        ShowReport( _( "Selected options resulted in errors! Change them and try again." ), RPT_SEVERITY_ERROR );
        return false;
    }

    const std::vector<REFDES_INFO>& badRefDes = m_reannotator->BadRefDes();

    if( !badRefDes.empty() )
    {
        message1.Printf( _( "PCB has %d empty or invalid reference designations." ), (int) badRefDes.size() );
        message2.Printf( _( "You may wish to run DRC with 'Test for parity between PCB and schematic' checked." ) );

        for( const REFDES_INFO& mod : badRefDes )
        {
            badrefdes += wxString::Format( _( "<br/>    RefDes: %s; footprint: %s at %s on PCB." ),
                                           mod.RefDesString.IsEmpty() ? empty_str() : mod.RefDesString,
                                           mod.FPID.IsValid() ? wxString( mod.FPID.Format().c_str() ) : unknown_str(),
                                           CoordTowxString( mod.x, mod.y ) );
        }

        ShowReport( message1 + wxT( "<br/>" ) + message2 + badrefdes + wxT( "<br/>" ), RPT_SEVERITY_WARNING );

        if( !IsOK( m_frame, message1 + "\n" + message2 + "\n \n" + _( "Reannotate anyway?" ) ) )
            return false;
    }

    BOARD_COMMIT commit( m_frame );

    for( FOOTPRINT* footprint : m_footprints )
    {
        const REFDES_CHANGE* newref = m_reannotator->GetNewRefDes( footprint );

        if( nullptr == newref )
        {
            ShowReport( _( "Footprint not found in changelist" ) + wxS( " " ) + footprint->GetReference(),
                        RPT_SEVERITY_ERROR );
            return false;
        }

        commit.Modify( footprint );                           // Make a copy for undo
        footprint->SetReference( newref->NewRefDes );         // Update the PCB reference
        m_frame->GetCanvas()->GetView()->Update( footprint ); // Touch the footprint
    }

    commit.Push( _( "Annotation" ) );
    return true;
}


bool DIALOG_BOARD_REANNOTATE::BuildFootprintList()
{
    REANNOTATE_OPTIONS options;

    if( m_AnnotateSelection->GetValue() )
        options.Scope = REANNOTATE_OPTIONS::SELECTION;
    else if( m_AnnotateFront->GetValue() )
        options.Scope = REANNOTATE_OPTIONS::FRONT;
    else if( m_AnnotateBack->GetValue() )
        options.Scope = REANNOTATE_OPTIONS::BACK;

    options.IsSelected = []( const FOOTPRINT* aFootprint ) { return aFootprint->IsSelected(); };
    options.ExcludeLocked = m_ExcludeLocked->GetValue();
    options.UseFootprintLocation = m_locationChoice->GetSelection() == 0;

    GRID sortGridMils = m_frame->config()->m_Window.grid.grids[ m_GridChoice->GetSelection() ];
    options.SortGrid.x = (int) EDA_UNIT_UTILS::UI::ValueFromString( pcbIUScale, EDA_UNITS::MILS, sortGridMils.x );
    options.SortGrid.y = (int) EDA_UNIT_UTILS::UI::ValueFromString( pcbIUScale, EDA_UNITS::MILS, sortGridMils.y );

    m_excludeArray.clear();
    m_footprints = m_frame->GetBoard()->Footprints();

    wxStringTokenizer tokenizer( m_ExcludeList->GetValue(), ", \t\r\n", wxTOKEN_STRTOK );

    while( tokenizer.HasMoreTokens() )
        m_excludeArray.push_back( tokenizer.GetNextToken() );

    options.Exclude = m_excludeArray;

    int sortCode = 0; // Convert radio button to sort direction code

    for( wxRadioButton* sortbuttons : m_sortButtons )
    {
        if( sortbuttons->GetValue() )
            break;

        sortCode++;
    }

    options.Direction = sortCode >= (int) m_sortButtons.size() ? 0 : sortCode;
    options.FrontStart = wxAtoi( m_FrontRefDesStart->GetValue() );
    options.BackStart = wxAtoi( m_BackRefDesStart->GetValue() );
    options.FrontPrefix = m_FrontPrefix->GetValue();
    options.BackPrefix = m_BackPrefix->GetValue();
    options.RemoveFrontPrefix = m_RemoveFrontPrefix->GetValue();
    options.RemoveBackPrefix = m_RemoveBackPrefix->GetValue();

    m_reannotator = std::make_unique<BOARD_REANNOTATOR>( options );

    const bool ok = m_reannotator->Plan( std::vector<FOOTPRINT*>( m_footprints.begin(), m_footprints.end() ) );

    if( !m_reannotator->FrontFootprints().empty() )
        LogFootprints( m_reannotator->FrontFootprints() );

    if( !m_reannotator->BackFootprints().empty() )
        LogFootprints( m_reannotator->BackFootprints() );

    LogChangePlan();

    for( const wxString& duplicate : m_reannotator->Duplicates() )
        ShowReport( wxString::Format( _( "Duplicate instances of %s" ), duplicate ), RPT_SEVERITY_ERROR );

    if( m_reannotator->TooManyErrors() )
        ShowReport( _( "Aborted: too many errors" ), RPT_SEVERITY_ERROR );

    return ok;
}
