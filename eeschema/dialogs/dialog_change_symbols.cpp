/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2020-2021 CERN
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * @author Wayne Stambaugh <stambaughw@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "settings/settings_manager.h"
#include <algorithm>

#include <bitmaps.h>
#include <connection_graph.h>
#include <string_utils.h>   // WildCompareString
#include <kiway.h>
#include <refdes_utils.h>
#include <core/kicad_algo.h>
#include <dialog_change_symbols.h>
#include <sch_symbol.h>
#include <sch_edit_frame.h>
#include <sch_screen.h>
#include <schematic.h>
#include <template_fieldnames.h>
#include <widgets/wx_html_report_panel.h>
#include <widgets/std_bitmap_button.h>
#include <sch_commit.h>


DIALOG_CHANGE_SYMBOLS::DIALOG_CHANGE_SYMBOLS( SCH_EDIT_FRAME* aParent, SCH_SYMBOL* aSymbol, MODE aMode ) :
        DIALOG_CHANGE_SYMBOLS_BASE( aParent ),
        m_symbol( aSymbol),
        m_mode( aMode )
{
    wxASSERT( aParent );

    if( m_mode == MODE::UPDATE )
    {
        m_newIdSizer->Show( false );
    }
    else
    {
        m_matchAll->SetLabel( _( "Change all symbols in schematic" ) );
        SetTitle( _( "Change Symbols" ) );
        m_matchSizer->FindItem( m_matchAll )->Show( false );

        m_matchByReference->SetLabel( _( "Change symbols matching reference designator:" ) );
        m_matchByValue->SetLabel( _( "Change symbols matching value:" ) );
        m_matchById->SetLabel( _( "Change symbols matching library identifier:" ) );

        m_updateFieldsSizer->GetStaticBox()->SetLabel( _( "Update Fields" ) );
        m_removeExtraBox->SetLabel( _( "Remove fields if not in new symbol" ) );
        m_resetEmptyFields->SetLabel( _( "Reset fields if empty in new symbol" ) );
        m_resetFieldText->SetLabel( _( "Update field text" ) );
        m_resetFieldVisibilities->SetLabel( _( "Update field visibilities" ) );
        m_resetFieldEffects->SetLabel( _( "Update field sizes and styles" ) );
        m_resetFieldPositions->SetLabel( _( "Update field positions" ) );
        m_resetAttributes->SetLabel( _( "Update symbol attributes" ) );
        m_resetPinTextVisibility->SetLabel( _( "Update pin name/number visibilities" ) );
        m_resetAlternatePin->SetLabel( _( "Reset alternate pin functions" ) );
        m_resetCustomPower->SetLabel( _( "Reset custom power symbols" ) );

        if( m_symbol )
            m_matchBySelection->SetLabel( _( "Change selected symbol(s)" ) );
    }

    if( !m_symbol )
        m_matchSizer->FindItem( m_matchBySelection )->Show( false );

    m_matchIdBrowserButton->SetBitmap( KiBitmapBundle( BITMAPS::small_library ) );
    m_newIdBrowserButton->SetBitmap( KiBitmapBundle( BITMAPS::small_library ) );

    m_matchSizer->SetEmptyCellSize( wxSize( 0, 0 ) );
    m_matchSizer->Layout();

    bool selectReference = false;
    bool selectValue = false;

    if( EESCHEMA_SETTINGS* cfg = GetAppSettings<EESCHEMA_SETTINGS>( "eeschema" ) )
    {
        selectReference = cfg->m_ChangeSymbols.updateReferences;
        selectValue = cfg->m_ChangeSymbols.updateValues;
    }

    for( FIELD_T fieldId : MANDATORY_FIELDS )
    {
        int listIdx = (int) m_fieldsBox->GetCount();

        m_fieldsBox->Append( GetDefaultFieldName( fieldId, TRANSLATED ) );

        // List boxes aren't currently handled in DIALOG_SHIM's control-state-save/restore
        if( fieldId == FIELD_T::REFERENCE )
            m_fieldsBox->Check( listIdx, selectReference );
        else if( fieldId == FIELD_T::VALUE )
            m_fieldsBox->Check( listIdx, selectValue );
        else
            m_fieldsBox->Check( listIdx, true );

        m_mandatoryFieldListIndexes[fieldId] = listIdx;
    }

    // initialize controls based on m_mode in case there is no saved state yet
    m_removeExtraBox->SetValue( false );
    m_resetEmptyFields->SetValue( false );
    m_resetFieldText->SetValue( true );
    m_resetCustomPower->SetValue( false );
    m_resetFieldVisibilities->SetValue( ( m_mode == MODE::CHANGE ) ? true  : false );
    m_resetFieldEffects->SetValue(      ( m_mode == MODE::CHANGE ) ? true  : false );
    m_resetFieldPositions->SetValue(    ( m_mode == MODE::CHANGE ) ? true  : false );
    m_resetAttributes->SetValue(        ( m_mode == MODE::CHANGE ) ? true  : false );
    m_resetPinTextVisibility->SetValue( ( m_mode == MODE::CHANGE ) ? true  : false );
    m_resetAlternatePin->SetValue(      ( m_mode == MODE::CHANGE ) ? true  : false );
    m_resetPinMapOverrides->SetValue( ( m_mode == MODE::CHANGE ) ? true : false );

    m_messagePanel->SetLazyUpdate( true );
    m_messagePanel->SetFileName( Prj().GetProjectPath() + wxT( "report.txt" ) );

    // DIALOG_SHIM needs a unique hash_key because classname is not sufficient
    // because the update and change versions of this dialog have different controls.
    m_hash_key = TO_UTF8( GetTitle() );

    wxString okLabel = m_mode == MODE::CHANGE ? _( "Change" ) : _( "Update" );

    SetupStandardButtons( { { wxID_OK,     okLabel      },
                            { wxID_CANCEL, _( "Close" ) } } );

    // Now all widgets have the size fixed, call FinishDialogSettings
    finishDialogSettings();
}


bool DIALOG_CHANGE_SYMBOLS::TransferDataToWindow()
{
    if( m_symbol )
    {
        SCH_SHEET_PATH* currentSheet = &m_symbol->Schematic()->CurrentSheet();

        m_specifiedReference->ChangeValue( m_symbol->GetRef( currentSheet ) );
        m_specifiedValue->ChangeValue( UnescapeString( m_symbol->GetField( FIELD_T::VALUE )->GetText() ) );
        m_specifiedId->ChangeValue( UnescapeString( m_symbol->GetLibId().Format() ) );
    }

    if( m_symbol && m_symbol->IsSelected() )
        m_matchBySelection->SetValue( true );
    else if( m_mode == MODE::UPDATE )
        m_matchAll->SetValue( true );
    else
        m_matchByReference->SetValue( true );

    updateFieldsList();

    return true;
}


void DIALOG_CHANGE_SYMBOLS::onMatchByAll( wxCommandEvent& aEvent )
{
    updateFieldsList();
}


void DIALOG_CHANGE_SYMBOLS::onMatchBySelected( wxCommandEvent& aEvent )
{
    updateFieldsList();
}


void DIALOG_CHANGE_SYMBOLS::onMatchByReference( wxCommandEvent& aEvent )
{
    updateFieldsList();
    m_specifiedReference->SetFocus();
}


void DIALOG_CHANGE_SYMBOLS::onMatchByValue( wxCommandEvent& aEvent )
{
    updateFieldsList();
    m_specifiedValue->SetFocus();
}


void DIALOG_CHANGE_SYMBOLS::onMatchById( wxCommandEvent& aEvent )
{
    updateFieldsList();
    m_specifiedId->SetFocus();
}


void DIALOG_CHANGE_SYMBOLS::onMatchTextKillFocus( wxFocusEvent& event )
{
    updateFieldsList();
    event.Skip();       // Mandatory in wxFocusEvent
}


void DIALOG_CHANGE_SYMBOLS::onMatchIDKillFocus( wxFocusEvent& event )
{
    updateFieldsList();
    event.Skip();       // Mandatory in wxFocusEvent
}


void DIALOG_CHANGE_SYMBOLS::onNewLibIDKillFocus( wxFocusEvent& event )
{
    updateFieldsList();
    event.Skip();       // Mandatory in wxFocusEvent
}


DIALOG_CHANGE_SYMBOLS::~DIALOG_CHANGE_SYMBOLS()
{
    // List boxes aren't currently handled in DIALOG_SHIM's control-state-save/restore
    if( EESCHEMA_SETTINGS* cfg = GetAppSettings<EESCHEMA_SETTINGS>( "eeschema" ) )
    {
        EESCHEMA_SETTINGS::DIALOG_CHANGE_SYMBOLS& dlg_cfg = cfg->m_ChangeSymbols;

        dlg_cfg.updateReferences = m_fieldsBox->IsChecked( m_mandatoryFieldListIndexes[FIELD_T::REFERENCE] );
        dlg_cfg.updateValues = m_fieldsBox->IsChecked( m_mandatoryFieldListIndexes[FIELD_T::VALUE] );
    }
}


wxString getLibIdValue( const wxTextCtrl* aCtrl )
{
    wxString rawValue = aCtrl->GetValue();
    wxString itemName;
    wxString libName = rawValue.BeforeFirst( ':', &itemName );

    return EscapeString( libName, CTX_LIBID ) + ':' + EscapeString( itemName, CTX_LIBID );
}


void DIALOG_CHANGE_SYMBOLS::launchMatchIdSymbolBrowser( wxCommandEvent& aEvent )
{
    wxString newName = getLibIdValue( m_specifiedId );

    if( KIWAY_PLAYER* frame = Kiway().Player( FRAME_SYMBOL_CHOOSER, true, this ) )
    {
        if( frame->ShowModal( &newName, this ) )
        {
            m_specifiedId->SetValue( UnescapeString( newName ) );
            updateFieldsList();
        }

        frame->Destroy();
    }
}


void DIALOG_CHANGE_SYMBOLS::launchNewIdSymbolBrowser( wxCommandEvent& aEvent )
{
    wxString newName = getLibIdValue( m_newId );

    if( KIWAY_PLAYER* frame = Kiway().Player( FRAME_SYMBOL_CHOOSER, true, this ) )
    {
        if( frame->ShowModal( &newName, this ) )
        {
            m_newId->SetValue( UnescapeString( newName ) );
            updateFieldsList();
        }

        frame->Destroy();
    }
}


void DIALOG_CHANGE_SYMBOLS::updateFieldsList()
{
    SCH_EDIT_FRAME* frame = dynamic_cast<SCH_EDIT_FRAME*>( GetParent() );

    wxCHECK( frame, /* void */ );

    // Load non-mandatory fields from all matching symbols and their library symbols
    std::set<wxString> fieldNames;

    for( SCH_SHEET_PATH& instance : frame->Schematic().Hierarchy() )
    {
        SCH_SCREEN* screen = instance.LastScreen();

        wxCHECK2( screen, continue );

        for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = dynamic_cast<SCH_SYMBOL*>( item );

            wxCHECK2( symbol, continue );

            if( !isMatch( symbol, &instance ) )
                continue;

            for( SCH_FIELD& field : symbol->GetFields() )
            {
                if( !field.IsMandatory() && !field.IsPrivate() )
                    fieldNames.insert( field.GetName() );
            }

            if( m_mode == MODE::UPDATE && symbol->GetLibId().IsValid() )
            {
                LIB_SYMBOL* libSymbol = frame->GetLibSymbol( symbol->GetLibId() );

                if( libSymbol )
                {
                    std::unique_ptr<LIB_SYMBOL> flattenedSymbol = libSymbol->Flatten();
                    std::vector<SCH_FIELD*>     orderedLibFields;

                    flattenedSymbol->GetFields( orderedLibFields );

                    for( SCH_FIELD* libField : orderedLibFields )
                    {
                        if( !libField->IsMandatory() && !libField->IsPrivate() )
                            fieldNames.insert( libField->GetName() );
                    }
                }
            }
        }
    }

    // Load non-mandatory fields from the change-to library symbol
    if( m_mode == MODE::CHANGE )
    {
        LIB_ID newId;

        newId.Parse( getLibIdValue( m_newId ) );

        if( newId.IsValid() )
        {
            LIB_SYMBOL* libSymbol = frame->GetLibSymbol( newId );

            if( libSymbol )
            {
                std::unique_ptr<LIB_SYMBOL> flattenedSymbol = libSymbol->Flatten();
                std::vector<SCH_FIELD*>     orderedLibFields;

                flattenedSymbol->GetFields( orderedLibFields );

                for( SCH_FIELD* libField : orderedLibFields )
                {
                    if( !libField->IsMandatory() && !libField->IsPrivate() )
                        fieldNames.insert( libField->GetName() );
                }
            }
        }
    }

    // Update the listbox widget
    wxArrayInt    checkedItems;
    wxArrayString checkedNames;

    m_fieldsBox->GetCheckedItems( checkedItems );

    for( int ii : checkedItems )
        checkedNames.push_back( m_fieldsBox->GetString( ii ) );

    bool allChecked = true;

    for( int ii = 0; ii < (int) m_fieldsBox->GetCount(); ++ii )
    {
        if( !m_fieldsBox->IsChecked( ii )
                && ii != m_mandatoryFieldListIndexes[FIELD_T::REFERENCE]
                && ii != m_mandatoryFieldListIndexes[FIELD_T::VALUE] )
        {
            allChecked = false;
        }
    }

    auto isMandatoryField =
            [&]( int listbox_idx )
            {
                for( FIELD_T fieldId : MANDATORY_FIELDS )
                {
                    if( m_mandatoryFieldListIndexes[fieldId] == listbox_idx )
                        return true;
                }

                return false;
            };

    for( int ii = (int) m_fieldsBox->GetCount() - 1; ii >= 0; --ii )
    {
        if( isMandatoryField( ii ) )
            continue;

        m_fieldsBox->Delete( ii );
    }

    for( const wxString& fieldName : fieldNames )
    {
        m_fieldsBox->Append( fieldName );

        if( allChecked || alg::contains( checkedNames, fieldName ) )
            m_fieldsBox->Check( m_fieldsBox->GetCount() - 1, true );
    }
}


void DIALOG_CHANGE_SYMBOLS::selectAll( bool aSelect )
{
    for( unsigned i = 0; i < m_fieldsBox->GetCount(); ++i )
        m_fieldsBox->Check( i, aSelect );
}


void DIALOG_CHANGE_SYMBOLS::checkAll( bool aCheck )
{
    m_removeExtraBox->SetValue( aCheck );
    m_resetEmptyFields->SetValue( aCheck );
    m_resetFieldText->SetValue( aCheck );
    m_resetFieldVisibilities->SetValue( aCheck );
    m_resetFieldEffects->SetValue( aCheck );
    m_resetFieldPositions->SetValue( aCheck );
    m_resetPinTextVisibility->SetValue( aCheck );
    m_resetAlternatePin->SetValue( aCheck );
    m_resetPinMapOverrides->SetValue( aCheck );
    m_resetAttributes->SetValue( aCheck );
    m_resetCustomPower->SetValue( aCheck );
}


void DIALOG_CHANGE_SYMBOLS::onOkButtonClicked( wxCommandEvent& aEvent )
{
    SCH_EDIT_FRAME* parent = dynamic_cast<SCH_EDIT_FRAME*>( GetParent() );

    wxCHECK( parent, /* void */ );

    wxBusyCursor    dummy;
    SCH_COMMIT      commit( parent );

    m_messagePanel->Clear();
    m_messagePanel->Flush( false );

    // Create the set of fields to be updated. Use untranslated names for mandatory fields.
    m_updateFields.clear();

    for( unsigned ii = 0; ii < m_fieldsBox->GetCount(); ++ii )
    {
        if( m_fieldsBox->IsChecked( ii ) )
            m_updateFields.insert( m_fieldsBox->GetString( ii ) );
    }

    for( FIELD_T fieldId : MANDATORY_FIELDS )
    {
        if( m_fieldsBox->IsChecked( m_mandatoryFieldListIndexes[fieldId] ) )
            m_updateFields.insert( GetDefaultFieldName( fieldId, UNTRANSLATED ) );
    }

    if( processMatchingSymbols( &commit) )
        commit.Push( m_mode == MODE::CHANGE ? _( "Change Symbols" ) : _( "Update Symbols" ) );

    m_messagePanel->Flush( false );
}


bool DIALOG_CHANGE_SYMBOLS::isMatch( SCH_SYMBOL* aSymbol, SCH_SHEET_PATH* aInstance )
{
    SCH_EDIT_FRAME* frame = dynamic_cast<SCH_EDIT_FRAME*>( GetParent() );

    wxCHECK( frame, false );

    if( !aSymbol )
        return false;

    if( m_matchAll->GetValue() )
    {
        return true;
    }
    else if( m_matchBySelection->GetValue() )
    {
        return aSymbol == m_symbol || aSymbol->IsSelected();
    }
    else if( m_matchByReference->GetValue() )
    {
        return WildCompareString( m_specifiedReference->GetValue(),
                                  UnescapeString( aSymbol->GetRef( aInstance, false ) ),
                                  false );
    }
    else if( m_matchByValue->GetValue() )
    {
        return WildCompareString( m_specifiedValue->GetValue(),
                                  UnescapeString( aSymbol->GetField( FIELD_T::VALUE )->GetText() ),
                                  false );
    }
    else if( m_matchById )
    {
        LIB_ID id;

        id.Parse( getLibIdValue( m_specifiedId ) );
        return aSymbol->GetLibId() == id;
    }

    return false;
}


int DIALOG_CHANGE_SYMBOLS::processMatchingSymbols( SCH_COMMIT* aCommit )
{
    SCH_EDIT_FRAME* frame = dynamic_cast<SCH_EDIT_FRAME*>( GetParent() );

    wxCHECK( frame, false );

    LIB_ID      newId;
    int         matchesProcessed = 0;
    SCH_SYMBOL* symbol = nullptr;

    if( m_mode == MODE::CHANGE )
    {
        newId.Parse( getLibIdValue( m_newId ) );

        if( !newId.IsValid() )
            return false;
    }

    std::map<SCH_SYMBOL*, SYMBOL_CHANGE_INFO> symbols;

    for( SCH_SHEET_PATH& instance : frame->Schematic().Hierarchy() )
    {
        SCH_SCREEN* screen = instance.LastScreen();

        wxCHECK2( screen, continue );

        // Fetch all the symbols that meet the change criteria.
        for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            symbol = static_cast<SCH_SYMBOL*>( item );

            wxCHECK2( symbol, continue );

            if( !isMatch( symbol, &instance ) )
                continue;

            if( m_mode == MODE::UPDATE )
                newId = symbol->GetLibId();

            auto it = symbols.find( symbol );

            if( it == symbols.end() )
            {
                SYMBOL_CHANGE_INFO info;

                info.m_Instances.emplace_back( instance );
                info.m_LibId = newId;
                symbols.insert( { symbol, info } );
            }
            else
            {
                it->second.m_Instances.emplace_back( instance );
            }
        }
    }

    if( symbols.size() > 0 )
        matchesProcessed += processSymbols( aCommit, symbols );
    else
        m_messagePanel->Report( _( "*** No symbols matching criteria found ***" ), RPT_SEVERITY_ERROR );

    frame->GetCurrentSheet().UpdateAllScreenReferences();

    return matchesProcessed;
}


int DIALOG_CHANGE_SYMBOLS::processSymbols( SCH_COMMIT* aCommit, const std::map<SCH_SYMBOL*,
                                           SYMBOL_CHANGE_INFO>& aSymbols )
{
    wxCHECK( !aSymbols.empty(), 0 );

    int             matchesProcessed = 0;
    SCH_EDIT_FRAME* frame = dynamic_cast<SCH_EDIT_FRAME*>( GetParent() );
    wxString        msg;

    wxCHECK( frame, 0 );

    SYMBOL_CHANGE_OPTIONS options;
    options.m_UpdateFields = m_updateFields;
    options.m_RemoveExtraFields = m_removeExtraBox->GetValue();
    options.m_ResetEmptyFields = m_resetEmptyFields->GetValue();
    options.m_ResetFieldText = m_resetFieldText->GetValue();
    options.m_ResetFieldVisibilities = m_resetFieldVisibilities->GetValue();
    options.m_ResetFieldEffects = m_resetFieldEffects->GetValue();
    options.m_ResetFieldPositions = m_resetFieldPositions->GetValue();
    options.m_ResetAttributes = m_resetAttributes->GetValue();
    options.m_ResetPinTextVisibility = m_resetPinTextVisibility->GetValue();
    options.m_ResetAlternatePins = m_resetAlternatePin->IsChecked();
    options.m_ResetCustomPower = m_resetCustomPower->IsChecked();
    options.m_ResetPinMapOverrides = m_resetPinMapOverrides->IsChecked();
    options.m_AutoplaceNewFields = frame->eeconfig()->m_AutoplaceFields.enable;

    SYMBOL_CHANGER changer(
            [frame]( const LIB_ID& aId )
            {
                return frame->GetLibSymbol( aId );
            },
            options );

    std::map<SCH_SYMBOL*, wxString> oldLinks;

    for( const auto& [symbol, info] : aSymbols )
        oldLinks[symbol] = UnescapeString( symbol->GetLibId().Format() );

    for( const SYMBOL_CHANGE_RESULT& result : changer.Apply( aCommit, aSymbols ) )
    {
        if( !result.m_Error.IsEmpty() )
        {
            msg = getSymbolReferences( *result.m_Symbol, result.m_NewLibId );
            msg << wxT( ": *** " ) << result.m_Error << wxT( " ***" );
            m_messagePanel->Report( msg, RPT_SEVERITY_ERROR );
            continue;
        }

        frame->GetCanvas()->GetView()->Update( result.m_Symbol );

        msg = getSymbolReferences( *result.m_Symbol, result.m_NewLibId, &oldLinks[result.m_Symbol] );
        msg += wxS( ": OK" );
        m_messagePanel->Report( msg, RPT_SEVERITY_ACTION );
        matchesProcessed += 1;
    }

    return matchesProcessed;
}


wxString DIALOG_CHANGE_SYMBOLS::getSymbolReferences( SCH_SYMBOL& aSymbol, const LIB_ID& aNewId,
                                                     const wxString* aOldLibLinkName )
{
    wxString      msg;
    wxString      references;
    const LIB_ID& oldId = aSymbol.GetLibId();
    wxString      oldLibLinkName;    // For report

    if( aOldLibLinkName )
        oldLibLinkName = *aOldLibLinkName;
    else
        oldLibLinkName = UnescapeString( oldId.Format() );

    SCH_EDIT_FRAME* parent = dynamic_cast< SCH_EDIT_FRAME* >( GetParent() );

    wxCHECK( parent, msg );

    SCH_SHEET_LIST sheets = parent->Schematic().Hierarchy();

    for( const SCH_SYMBOL_INSTANCE& instance : aSymbol.GetInstances() )
    {
        // Only include the symbol instances for the current project.
        if( !sheets.HasPath( instance.m_Path ) )
            continue;

        if( references.IsEmpty() )
            references = instance.m_Reference;
        else
            references += wxT( " " ) + instance.m_Reference;
    }

    if( m_mode == MODE::UPDATE )
    {
        if( aSymbol.GetInstances().size() == 1 )
        {
            msg.Printf( _( "Update symbol %s from '%s' to '%s'" ),
                        references,
                        oldLibLinkName,
                        UnescapeString( aNewId.Format() ) );
        }
        else
        {
            msg.Printf( _( "Update symbols %s from '%s' to '%s'" ),
                        references,
                        oldLibLinkName,
                        UnescapeString( aNewId.Format() ) );
        }
    }
    else    // mode is MODE::CHANGE
    {
        if( aSymbol.GetInstances().size() == 1 )
        {
            msg.Printf( _( "Change symbol %s from '%s' to '%s'" ),
                        references,
                        oldLibLinkName,
                        UnescapeString( aNewId.Format() ) );
        }
        else
        {
            msg.Printf( _( "Change symbols %s from '%s' to '%s'" ),
                        references,
                        oldLibLinkName,
                        UnescapeString( aNewId.Format() ) );
        }
    }

    return msg;
}
