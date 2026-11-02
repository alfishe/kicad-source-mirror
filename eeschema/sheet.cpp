/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2015 Jean-Pierre Charras, jp.charras at wanadoo.fr
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

#include <map>
#include <core/utf8.h>
#include <sch_draw_panel.h>
#include <common.h>
#include <confirm.h>
#include <kiface_base.h>
#include <project.h>
#include <math/vector2wx.h>
#include <string_utils.h>
#include <trace_helpers.h>
#include <wildcards_and_files_ext.h>
#include <wx_filename.h>
#include <tool/tool_manager.h>
#include <project_sch.h>
#include <sch_edit_frame.h>
#include <sch_io/sch_io.h>
#include <sch_io/sch_io_mgr.h>
#include <sch_io/kicad_legacy/sch_io_kicad_legacy.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_view.h>
#include <sch_painter.h>
#include <schematic.h>
#include <sch_sheet_loader.h>
#include <dialogs/dialog_sheet_properties.h>
#include <tool/actions.h>

#include <wx/clipbrd.h>
#include <wx/dcmemory.h>
#include <wx/log.h>
#include <wx/msgdlg.h>
#include <wx/richmsgdlg.h>

#include <advanced_config.h>
#include <pgm_base.h>
#include <libraries/symbol_library_adapter.h>

#include "printing/sch_printout.h"


bool SCH_EDIT_FRAME::CheckSheetForRecursion( SCH_SHEET* aSheet, SCH_SHEET_PATH* aCurrentSheet )
{
    wxASSERT( aSheet && aCurrentSheet );

    wxString msg;
    SCH_SHEET_LIST schematicSheets = Schematic().Hierarchy();
    SCH_SHEET_LIST loadedSheets( aSheet );  // This is the schematicSheets of the loaded file.

    wxString destFilePath = aCurrentSheet->LastScreen()->GetFileName();

    if( destFilePath.IsEmpty() )
    {
        // If file is unsaved then there can't (yet) be any recursion.
        return false;
    }

    // SCH_SCREEN object file paths are expected to be absolute.  If this assert fires,
    // something is seriously broken.
    wxASSERT_MSG( wxFileName( destFilePath ).IsAbsolute(), destFilePath + wxS( " is not absolute" ) );

    if( schematicSheets.TestForRecursion( loadedSheets, destFilePath ) )
    {
        msg.Printf( _( "The sheet changes cannot be made because the destination sheet already "
                       "has the sheet '%s' or one of its subsheets as a parent somewhere in the "
                       "schematic hierarchy." ),
                    destFilePath );
        DisplayError( this, msg );
        return true;
    }

    return false;
}


bool SCH_EDIT_FRAME::checkForNoFullyDefinedLibIds( SCH_SHEET* aSheet )
{
    wxASSERT( aSheet && aSheet->GetScreen() );

    wxString msg;
    SCH_SCREENS newScreens( aSheet );

    if( newScreens.HasNoFullyDefinedLibIds() )
    {
        msg.Printf( _( "The schematic '%s' has not had its symbol library links remapped "
                       "to the symbol library table.  The project this schematic belongs to "
                       "must first be remapped before it can be imported into the current "
                       "project." ),
                    aSheet->GetScreen()->GetFileName() );
        DisplayInfoMessage( this, msg );
        return true;
    }

    return false;
}


void SCH_EDIT_FRAME::InitSheet( SCH_SHEET* aSheet, const wxString& aNewFilename )
{
    SCH_SCREEN* newScreen = new SCH_SCREEN( &Schematic() );
    aSheet->SetScreen( newScreen );
    aSheet->GetScreen()->SetContentModified();
    aSheet->GetScreen()->SetFileName( aNewFilename );

    EESCHEMA_SETTINGS* cfg = eeconfig();
    wxCHECK( cfg, /* void */ );

    if( cfg->m_PageSettings.export_paper )
        newScreen->SetPageSettings( GetScreen()->GetPageSettings() );

    const TITLE_BLOCK& tb1 = GetScreen()->GetTitleBlock();
    TITLE_BLOCK        tb2 = newScreen->GetTitleBlock();

    if( cfg->m_PageSettings.export_revision )
        tb2.SetRevision( tb1.GetRevision() );

    if( cfg->m_PageSettings.export_date )
        tb2.SetDate( tb1.GetDate() );

    if( cfg->m_PageSettings.export_title )
        tb2.SetTitle( tb1.GetTitle() );

    if( cfg->m_PageSettings.export_company )
        tb2.SetCompany( tb1.GetCompany() );

    if( cfg->m_PageSettings.export_comment1 )
        tb2.SetComment( 0, tb1.GetComment( 0 ) );

    if( cfg->m_PageSettings.export_comment2 )
        tb2.SetComment( 1, tb1.GetComment( 1 ) );

    if( cfg->m_PageSettings.export_comment3 )
        tb2.SetComment( 2, tb1.GetComment( 2 ) );

    if( cfg->m_PageSettings.export_comment4 )
        tb2.SetComment( 3, tb1.GetComment( 3 ) );

    if( cfg->m_PageSettings.export_comment5 )
        tb2.SetComment( 4, tb1.GetComment( 4 ) );

    if( cfg->m_PageSettings.export_comment6 )
        tb2.SetComment( 5, tb1.GetComment( 5 ) );

    if( cfg->m_PageSettings.export_comment7 )
        tb2.SetComment( 6, tb1.GetComment( 6 ) );

    if( cfg->m_PageSettings.export_comment8 )
        tb2.SetComment( 7, tb1.GetComment( 7 ) );

    if( cfg->m_PageSettings.export_comment9 )
        tb2.SetComment( 8, tb1.GetComment( 8 ) );

    newScreen->SetTitleBlock( tb2 );
}


bool SCH_EDIT_FRAME::ChangeSheetFile( SCH_SHEET* aSheet, const wxString& aNewFilename,
                                      bool* aClearAnnotationNewItems, bool* aIsUndoable,
                                      const wxString* aSourceSheetFilename )
{
    wxString    msg;
    SCHEMATIC&  schematic = Schematic();

    // Resolve text variables before touching disk. The field keeps the raw text for portability.
    wxFileName  sheetFileName( ExpandTextVars( aNewFilename, &schematic.Project(), INTERNAL ) );
    SCH_SCREEN* currentScreen = GetCurrentSheet().LastScreen();

    wxCHECK( currentScreen, false );

    // SCH_SCREEN file names are always absolute.
    wxFileName currentScreenFileName = currentScreen->GetFileName();
    wxFileName screenFileName( sheetFileName );

    if( !screenFileName.Normalize( FN_NORMALIZE_FLAGS | wxPATH_NORM_ENV_VARS,
                                   currentScreenFileName.GetPath() ) )
    {
        msg = wxString::Format( _( "Cannot normalize new sheet schematic file path:\n"
                                   "'%s'\n"
                                   "against parent sheet schematic file path:\n"
                                   "'%s'." ),
                                sheetFileName.GetPath(),
                                currentScreenFileName.GetPath() );
        DisplayError( this, msg );
        return false;
    }

    wxString newAbsoluteFilename = screenFileName.GetFullPath();
    newAbsoluteFilename.Replace( wxT( "\\" ), wxT( "/" ) );

    if( !AllowCaseSensitiveFileNameClashes( aSheet->GetFileName(), newAbsoluteFilename ) )
        return false;

    SCH_SHEET_LIST fullHierarchy = schematic.Hierarchy();

    bool        renameFile = false;
    bool        loadFromFile = false;
    bool        clearAnnotation = false;
    bool        isExistingSheet = false;
    SCH_SCREEN* useScreen = nullptr;
    SCH_SCREEN* oldScreen = nullptr;

    // Search for a schematic file already in use in the hierarchy or on disk
    if( !schematic.Root().SearchHierarchy( newAbsoluteFilename, &useScreen ) )
    {
        loadFromFile = wxFileExists( newAbsoluteFilename );

        wxLogTrace( tracePathsAndFiles, "\n    Sheet requested file '%s', %s",
                                        newAbsoluteFilename,
                                        loadFromFile ? "found" : "not found" );
    }

    if( aSheet->GetScreen() == nullptr )
    {
        // New sheet with no screen yet
        if( useScreen || loadFromFile )
        {
            clearAnnotation = true;

            if( !IsOK( this, wxString::Format( _( "'%s' already exists." ),
                                               sheetFileName.GetFullName() )
                             + wxT( "\n\n" )
                             + wxString::Format( _( "Link '%s' to this file?" ),
                                                 newAbsoluteFilename ) ) )
            {
                return false;
            }
        }
        else if( aSourceSheetFilename && !aSourceSheetFilename->IsEmpty() )
        {
            // Design block / sheet import -- copy source file to destination
            loadFromFile = true;

            if( !wxCopyFile( *aSourceSheetFilename, newAbsoluteFilename, false ) )
            {
                msg.Printf( _( "Failed to copy schematic file '%s' to destination '%s'." ),
                            *aSourceSheetFilename,
                            newAbsoluteFilename );
                DisplayError( this, msg );
                return false;
            }
        }
        else
        {
            InitSheet( aSheet, newAbsoluteFilename );
        }
    }
    else
    {
        // Existing sheet
        isExistingSheet = true;

        wxString oldAbsoluteFilename = aSheet->GetScreen()->GetFileName();
        oldAbsoluteFilename.Replace( wxT( "\\" ), wxT( "/" ) );

        if( newAbsoluteFilename.Cmp( oldAbsoluteFilename ) != 0 )
        {
            // Sheet file name changes cannot be undone
            if( aIsUndoable )
                *aIsUndoable = false;

            if( useScreen || loadFromFile )
            {
                clearAnnotation = true;
                oldScreen = aSheet->GetScreen();

                if( !IsOK( this, wxString::Format( _( "Change '%s' link from '%s' to '%s'?" ),
                                                    newAbsoluteFilename,
                                                    aSheet->GetFileName(),
                                                    sheetFileName.GetFullName() )
                                 + wxT( "\n\n" )
                                 + _( "This action cannot be undone." ) ) )
                {
                    return false;
                }

                if( loadFromFile )
                    aSheet->SetScreen( nullptr );
            }
            else
            {
                // Save current content to new file name
                if( aSheet->GetScreenCount() > 1 )
                {
                    if( !IsOK( this, wxString::Format( _( "Create new file '%s' with contents "
                                                           "of '%s'?" ),
                                                        sheetFileName.GetFullName(),
                                                        aSheet->GetFileName() )
                                     + wxT( "\n\n" )
                                     + _( "This action cannot be undone." ) ) )
                    {
                        return false;
                    }
                }

                renameFile = true;
            }
        }

        if( renameFile )
        {
            IO_RELEASER<SCH_IO> pi( SCH_IO_MGR::FindPlugin( SCH_IO_MGR::SCH_KICAD ) );

            // Only update the screen filename when this is the sole user
            if( aSheet->GetScreenCount() <= 1 )
                aSheet->GetScreen()->SetFileName( newAbsoluteFilename );

            try
            {
                pi->SaveSchematicFile( newAbsoluteFilename, aSheet, &schematic );
            }
            catch( const IO_ERROR& ioe )
            {
                msg = wxString::Format( _( "Error occurred saving schematic file '%s'." ),
                                        newAbsoluteFilename );
                DisplayErrorMessage( this, msg, ioe.What() );

                msg = wxString::Format( _( "Failed to save schematic '%s'" ),
                                        newAbsoluteFilename );
                SetMsgPanel( wxEmptyString, msg );
                return false;
            }

            // Shared screen needs reload to maintain correct reference counting
            if( aSheet->GetScreenCount() > 1 )
            {
                oldScreen = aSheet->GetScreen();
                aSheet->SetScreen( nullptr );
                loadFromFile = true;
            }
        }
    }

    SCH_SHEET_PATH& currentSheet = GetCurrentSheet();

    if( useScreen )
    {
        // Recursion test with a temporary sheet
        std::unique_ptr<SCH_SHEET> tmpSheet = std::make_unique<SCH_SHEET>( &schematic );
        tmpSheet->GetField( FIELD_T::SHEET_FILENAME )->SetText( sheetFileName.GetFullPath() );
        tmpSheet->SetScreen( useScreen );

        if( CheckSheetForRecursion( tmpSheet.get(), &currentSheet ) )
            return false;

        aSheet->SetScreen( useScreen );

        SCH_SHEET_LIST sheetHierarchy( aSheet );
        sheetHierarchy.AddNewSymbolInstances( currentSheet, Prj().GetProjectName() );
        sheetHierarchy.AddNewSheetInstances( currentSheet,
                                             fullHierarchy.GetLastVirtualPageNumber() );
    }
    else if( loadFromFile )
    {
        bool restoreSheet = false;

        if( isExistingSheet )
        {
            restoreSheet = true;
            currentSheet.LastScreen()->Remove( aSheet );
        }

        if( !LoadSheetFromFile( aSheet, &currentSheet, newAbsoluteFilename, false, true )
            || CheckSheetForRecursion( aSheet, &currentSheet ) )
        {
            if( restoreSheet )
            {
                if( oldScreen )
                    aSheet->SetScreen( oldScreen );

                currentSheet.LastScreen()->Append( aSheet );
            }

            return false;
        }

        if( restoreSheet )
            currentSheet.LastScreen()->Append( aSheet );
    }

    if( aClearAnnotationNewItems )
        *aClearAnnotationNewItems = clearAnnotation;

    RecalculateConnections( nullptr, GLOBAL_CLEANUP );

    SCH_SHEET_LIST repairedList;
    repairedList.BuildSheetList( &schematic.Root(), true );

    return true;
}


namespace
{

/// The schematic editor's answers to a sheet load: its dialogs
class FRAME_SHEET_LOAD_HANDLER : public SCH_SHEET_LOAD_HANDLER
{
public:
    explicit FRAME_SHEET_LOAD_HANDLER( SCH_EDIT_FRAME* aFrame ) : m_frame( aFrame ) {}

    bool Continue( const wxString& aTitle, const wxString& aMessage, const wxString& aDetail,
                   const wxString& aContinueLabel ) override
    {
        KICAD_MESSAGE_DIALOG dlg( m_frame, aMessage, aTitle,
                                  wxOK | wxCANCEL | wxCANCEL_DEFAULT | wxCENTER | wxICON_QUESTION );
        dlg.SetOKCancelLabels( KICAD_MESSAGE_DIALOG::ButtonLabel( aContinueLabel ),
                               KICAD_MESSAGE_DIALOG::ButtonLabel( _( "Cancel Load" ) ) );

        if( !aDetail.IsEmpty() )
            dlg.SetExtendedMessage( aDetail );

        return dlg.ShowModal() != wxID_CANCEL;
    }

    void Error( const wxString& aMessage, const wxString& aDetail ) override
    {
        DisplayErrorMessage( m_frame, aMessage, aDetail );
    }

    void Info( const wxString& aMessage ) override { DisplayInfoMessage( m_frame, aMessage ); }

    void LoadFailed( const wxString& aFileName ) override
    {
        m_frame->SetMsgPanel( wxEmptyString, wxString::Format( _( "Failed to load '%s'." ), aFileName ) );
    }

private:
    SCH_EDIT_FRAME* m_frame;
};

} // namespace


bool SCH_EDIT_FRAME::LoadSheetFromFile( SCH_SHEET* aSheet, SCH_SHEET_PATH* aCurrentSheet,
                                        const wxString& aFileName, bool aSkipRecursionCheck,
                                        bool aSkipLibCheck )
{
    wxASSERT( aSheet && aCurrentSheet );

    FRAME_SHEET_LOAD_HANDLER handler( this );

    return SchLoadSheetFromFile( Schematic(), aSheet, aCurrentSheet, aFileName, aSkipRecursionCheck,
                                 aSkipLibCheck, handler );
}


bool SCH_EDIT_FRAME::EditSheetProperties( SCH_SHEET* aSheet, SCH_SHEET_PATH* aHierarchy,
                                          bool* aIsUndoable, bool* aClearAnnotationNewItems,
                                          bool* aUpdateHierarchyNavigator,
                                          wxString* aSourceSheetFilename )
{
    if( aSheet == nullptr || aHierarchy == nullptr )
        return false;

    // Get the new texts
    DIALOG_SHEET_PROPERTIES dlg( this, aSheet, aIsUndoable, aClearAnnotationNewItems,
                                 aUpdateHierarchyNavigator, aSourceSheetFilename );

    if( dlg.ShowModal() == wxID_CANCEL )
        return false;

    m_toolManager->ProcessEvent( EVENTS::SelectedItemsModified );

    return true;
}


class PRINT_TITLEBLOCK_CONTEXT
{
public:
    PRINT_TITLEBLOCK_CONTEXT( SCH_EDIT_FRAME* aFrame, bool aPrintTitleBlock ) :
            m_frame( aFrame )
    {
        m_originalValue = m_frame->eeconfig()->m_Printing.title_block;
        m_frame->eeconfig()->m_Printing.title_block = aPrintTitleBlock;
    }

    ~PRINT_TITLEBLOCK_CONTEXT()
    {
        m_frame->eeconfig()->m_Printing.title_block = m_originalValue;
    }

private:
    SCH_EDIT_FRAME*  m_frame;
    bool             m_originalValue;
};


void SCH_EDIT_FRAME::DrawCurrentSheetToClipboard()
{
    wxRect       drawArea;
    BASE_SCREEN* screen = GetScreen();

    drawArea.SetSize( ToWxSize( GetPageSizeIU() ) );

    // Calculate a reasonable dc size, in pixels, and the dc scale to fit
    // the drawings into the dc size
    // scale is the ratio resolution (in PPI) / internal units
    double ppi = 300;   // Use 300 pixels per inch to create bitmap images on start
    double inch2Iu = 1000.0 * schIUScale.IU_PER_MILS;
    double scale = ppi / inch2Iu;

    wxSize dcsize = drawArea.GetSize();

    int maxdim = std::max( dcsize.x, dcsize.y );

    // the max size in pixels of the bitmap used to build the sheet copy
    const int maxbitmapsize = 5600;

    while( int( maxdim * scale ) > maxbitmapsize )
    {
        ppi = ppi / 1.5;
        scale = ppi / inch2Iu;
    }

    dcsize.x *= scale;
    dcsize.y *= scale;

    // Set draw offset, zoom... to values needed to draw in the memory DC
    // after saving initial values:
    VECTOR2I tmp_startvisu = screen->m_StartVisu;
    VECTOR2I old_org = screen->m_DrawOrg;
    screen->m_DrawOrg.x = screen->m_DrawOrg.y = 0;
    screen->m_StartVisu.x = screen->m_StartVisu.y = 0;

    wxMemoryDC dc;
    wxBitmap image( dcsize );
    dc.SelectObject( image );
    dc.Clear();

    GRResetPenAndBrush( &dc );
    GRForceBlackPen( false );
    dc.SetUserScale( scale, scale );

    SCH_RENDER_SETTINGS* cfg = GetRenderSettings();

    cfg->SetPrintDC( &dc );

    // Init the color of the layer actually used to print the drawing sheet:
    cfg->SetLayerColor( LAYER_DRAWINGSHEET, cfg->GetLayerColor( LAYER_SCHEMATIC_DRAWINGSHEET ) );

    cfg->SetDefaultFont( eeconfig()->m_Appearance.default_font );

    try
    {
        dc.SetUserScale( 1.0, 1.0 );
        SCH_PRINTOUT printout( this, wxEmptyString );

        // Ensure title block will be when printed on clipboard, regardless of the current Cairo print option
        PRINT_TITLEBLOCK_CONTEXT raii( this, true );

        if( !printout.PrintPage( GetScreen(), cfg->GetPrintDC(), false ) )
            wxLogMessage( _( "Cannot create the schematic image") );
    }
    catch( ... )
    {
        wxLogMessage( "printout internal error" );
    }

    // Deselect Bitmap from DC before using the bitmap
    dc.SelectObject( wxNullBitmap );

    {
        wxLogNull doNotLog; // disable logging of failed clipboard actions

        if( wxTheClipboard->Open() )
        {
            // This data objects are held by the clipboard, so do not delete them in the app.
            wxBitmapDataObject* clipbrd_data = new wxBitmapDataObject( image );
            wxTheClipboard->SetData( clipbrd_data );
            wxTheClipboard->Flush(); // Allow data to be available after closing KiCad
            wxTheClipboard->Close();
        }
    }

    GRForceBlackPen( false );

    screen->m_StartVisu = tmp_startvisu;
    screen->m_DrawOrg   = old_org;
}


bool SCH_EDIT_FRAME::AllowCaseSensitiveFileNameClashes( const wxString& aOldName,
                                                        const wxString& aSchematicFileName )
{
    wxString       msg;
    SCH_SHEET_LIST sheets = Schematic().Hierarchy();
    wxFileName     fn = aSchematicFileName;

    wxCHECK( fn.IsAbsolute(), false );

    auto can_cause_issues = [&]() -> bool
    {
        wxFileName lhs;
        wxFileName rhs = aSchematicFileName;
        wxFileName old = aOldName;
        wxString   oldLower = old.GetFullName().Lower();
        wxString   rhsLower = rhs.GetFullName().Lower();
        wxString   lhsLower;

        size_t     count = 0;

        wxCHECK( rhs.IsAbsolute(), false );

        for( SCH_SHEET_PATH& sheet : sheets )
        {
            lhs = sheet.LastScreen()->GetFileName();

            if( lhs.GetPath() != rhs.GetPath() )
                continue;

            lhsLower = lhs.GetFullName().Lower();

            if( lhsLower == rhsLower && lhs.GetFullName() != rhs.GetFullName() )
                count++;
        }

        // If we are renaming a sheet that is only used once, then we are not going to cause
        // a case sensitivity issue.
        if( oldLower == rhsLower )
            return count > 1;

        return count > 0;
    };

    if( eeconfig()->m_Appearance.show_sheet_filename_case_sensitivity_dialog && can_cause_issues() )
    {
        msg.Printf( _( "The file name '%s' can cause issues with an existing file name\n"
                       "already defined in the schematic on systems that support case\n"
                       "insensitive file names.  This will cause issues if you copy this\n"
                       "project to an operating system that supports case insensitive file\n"
                       "names.\n\nDo you wish to continue?" ),
                    fn.GetName() );

        wxRichMessageDialog dlg( this, msg, _( "Warning" ),
                                 wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION );
        dlg.ShowCheckBox( _( "Do not show this message again." ) );
        dlg.SetYesNoLabels( wxMessageDialog::ButtonLabel( _( "Create New Sheet" ) ),
                            wxMessageDialog::ButtonLabel( _( "Cancel" ) ) );

        if( dlg.ShowModal() == wxID_NO )
            return false;

        eeconfig()->m_Appearance.show_sheet_filename_case_sensitivity_dialog =
                                                            !dlg.IsCheckBoxChecked();
    }

    return true;
}
