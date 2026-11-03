/**
 * @file dialog_symbol_remap.cpp
 */

/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2017 Wayne Stambaugh <stambaughw@gmail.com>
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
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

#include <macros.h>
#include <pgm_base.h>
#include <kiface_base.h>
#include <project.h>
#include <confirm.h>
#include <reporter.h>
#include <wildcards_and_files_ext.h>
#include <wx_filename.h>
#include "widgets/wx_html_report_panel.h"

#include <libraries/legacy_symbol_library.h>
#include <libraries/symbol_library_adapter.h>
#include <core/kicad_algo.h>
#include <symbol_viewer_frame.h>
#include <project_rescue.h>
#include <sch_io/sch_io_mgr.h>
#include <sch_symbol.h>
#include <sch_screen.h>
#include <sch_edit_frame.h>
#include <schematic.h>
#include <settings/settings_manager.h>
#include <env_paths.h>
#include <project_sch.h>
#include <wx/msgdlg.h>

#include <dialog_symbol_remap.h>
#include <symbol_remapper.h>


DIALOG_SYMBOL_REMAP::DIALOG_SYMBOL_REMAP( SCH_EDIT_FRAME* aParent ) :
    DIALOG_SYMBOL_REMAP_BASE( aParent ),
    m_frame( aParent )
{
    m_remapped = false;

    wxString notWritable = SYMBOL_REMAPPER( Prj(), aParent->Schematic(), false ).CheckWritable();

    if( !notWritable.IsEmpty() )
    {
        DisplayInfoMessage( this, notWritable );

        // Disable the remap button.
        m_remapped = true;
    }

    wxString text;

    text = _( "This schematic currently uses the project symbol library list look up method "
              "for loading library symbols.  KiCad will attempt to map the existing symbols "
              "to use the new symbol library table.  Remapping will change some project files "
              "and schematics may not be compatible with older versions of KiCad.  All files "
              "that are changed will be backed up to the \"rescue-backup\" folder in the project "
              "folder should you need to revert any changes.  If you choose to skip this step, "
              "you will be responsible for manually remapping the symbols." );

    m_htmlCtrl->AppendToPage( text );

    m_messagePanel->SetFileName( Prj().GetProjectPath() + wxT( "report.txt" ) );
}


void DIALOG_SYMBOL_REMAP::OnRemapSymbols( wxCommandEvent& aEvent )
{
    SCH_EDIT_FRAME* parent = dynamic_cast< SCH_EDIT_FRAME* >( GetParent() );

    wxCHECK_RET( parent != nullptr, "Parent window is not type SCH_EDIT_FRAME." );

    SYMBOL_REMAPPER remapper( Prj(), parent->Schematic(), false );

    if( !backupProject( remapper, m_messagePanel->Reporter() ) )
        return;

    // Ignore the never show rescue setting for one last rescue of legacy symbol
    // libraries before remapping to the symbol library table.  This ensures the
    // best remapping results.

    LEGACY_RESCUER rescuer( Prj(), &parent->Schematic(), &parent->GetCurrentSheet(),
                            parent->GetCanvas()->GetBackend() );

    if( RESCUER::RescueProject( this, rescuer, false ) )
    {
        wxBusyCursor busy;

        auto viewer = (SYMBOL_VIEWER_FRAME*) parent->Kiway().Player( FRAME_SCH_VIEWER, false );

        if( viewer )
            viewer->ReCreateLibList();

        parent->ClearUndoORRedoList( EDA_BASE_FRAME::UNDO_LIST, 1 );
        parent->SyncView();
        parent->GetCanvas()->Refresh();
        parent->OnModify();
    }

    {
        wxBusyCursor busy;
        remapper.Remap( m_messagePanel->Reporter() );
    }

    Raise();
    m_remapped = true;
}


bool DIALOG_SYMBOL_REMAP::backupProject( SYMBOL_REMAPPER& aRemapper, REPORTER& aReporter )
{
    wxString errors;
    bool     created;

    {
        wxBusyCursor busy;
        created = aRemapper.BackupProject( aReporter, errors );
    }

    if( errors.IsEmpty() )
        return true;

    KICAD_MESSAGE_DIALOG dlg( this, created ? _( "Some of the project files could not be backed up." ) : errors,
                              _( "Backup Error" ), wxYES_NO | wxCENTRE | wxRESIZE_BORDER | wxICON_QUESTION );

    if( created )
        dlg.SetExtendedMessage( errors );

    dlg.SetYesNoLabels( KICAD_MESSAGE_DIALOG::ButtonLabel( _( "Continue with Rescue" ) ),
                        KICAD_MESSAGE_DIALOG::ButtonLabel( _( "Abort Rescue" ) ) );

    return dlg.ShowModal() != wxID_NO;
}


void DIALOG_SYMBOL_REMAP::OnUpdateUIRemapButton( wxUpdateUIEvent& aEvent )
{
    aEvent.Enable( !m_remapped );
}
