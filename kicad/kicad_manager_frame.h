/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2013 CERN (www.cern.ch)
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

#ifndef KICAD_H
#define KICAD_H

#include <kiway_player.h>

#include <settings/kicad_settings.h>
#include <wx/timer.h>

class ACTION_TOOLBAR;
class BITMAP_BUTTON;
class EDA_BASE_FRAME;
class KICAD_SETTINGS;
class PANEL_KICAD_LAUNCHER;
class PLUGIN_CONTENT_MANAGER;
class PROJECT_TREE;
class PROJECT_TREE_PANE;
class LOCAL_HISTORY_PANE;
class UPDATE_MANAGER;

/**
 * The main KiCad project manager frame.  It is not a KIWAY_PLAYER.
 */
class KICAD_MANAGER_FRAME : public EDA_BASE_FRAME
{
public:
    KICAD_MANAGER_FRAME( wxWindow* parent, const wxString& title,
                         const wxPoint& pos, const wxSize& size );

    ~KICAD_MANAGER_FRAME();

    void OnIdle( wxIdleEvent& event );

    bool canCloseWindow( wxCloseEvent& aCloseEvent ) override;
    void doCloseWindow() override;
    void OnSize( wxSizeEvent& event ) override;

    void UnarchiveFiles();
    void RestoreLocalHistory();
    void RestoreCommitFromHistory( const wxString& aHash );
    void ToggleLocalHistory();
    bool HistoryPanelShown();

    void OnOpenFileInTextEditor( wxCommandEvent& event );
    void OnEditAdvancedCfg( wxCommandEvent& event );

    /// Open DIALOG_GIT_MR_REVIEW so the user can pick two refs and review
    /// the changed-file list between them.
    void OnCompareProjectBranches( wxCommandEvent& event );

    void OnFileHistory( wxCommandEvent& event );
    void OnClearFileHistory( wxCommandEvent& aEvent );
    void OnExit( wxCommandEvent& event );

    /** Create the status line (like a wxStatusBar). This is actually a KISTATUSBAR status bar.
     * the specified number of fields is the extra number of fields, not the full field count.
     * @return a KISTATUSBAR (derived from wxStatusBar)
     */
    wxStatusBar* OnCreateStatusBar( int number, long style, wxWindowID id,
                                    const wxString& name ) override;

    /**
     * Hides the tabs for Editor notebook if there is only 1 page
     */
    void HideTabsIfNeeded();

    wxString GetCurrentFileName() const override;

    /**
     * @brief Creates a project and imports a non-KiCad Schematic and PCB
     * @param aWindowTitle to display to the user when opening the files
     * @param aFilesWildcard that includes both PCB and Schematic files (from
     * wildcards_and_files_ext.h)
     * @param aSchFileExtensions e.g. { "sch" } or { "csa" }. Specify { "INPUT" } to copy input file.
     * @param aPcbFileExtensions e.g. { "brd" } or { "cpa" }. Specify { "INPUT" } to copy input file.
     * @param aSchFileType Type of Schematic File to import (from SCH_IO_MGR::SCH_FILE_T)
     * @param aPcbFileType Type of PCB File to import (from IO_MGR::PCB_FILE_T)
    */
    void ImportNonKiCadProject( const wxString& aWindowTitle, const wxString& aFilesWildcard,
                                const std::vector<std::string>& aSchFileExtensions,
                                const std::vector<std::string>& aPcbFileExtensions,
                                int aSchFileType, int aPcbFileType );

    /**
     * Open dialog to import Altium project files.
     */
    void OnImportAltiumProjectFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import CADSTAR Schematic and PCB Archive files.
     */
    void OnImportCadstarArchiveFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import Eagle schematic and board files.
     */
    void OnImportEagleFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import EasyEDA Std schematic and board files.
     */
    void OnImportEasyEdaFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import EasyEDA Pro schematic and board files.
     */
    void OnImportEasyEdaProFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import PADS Logic schematic and PCB files.
     */
    void OnImportPadsProjectFiles( wxCommandEvent& event );
    void OnImportPcadProjectFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import gEDA/gaf schematic and PCB files.
     */
    void OnImportGedaFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import DipTrace schematic and board files.
     */
    void OnImportDipTraceFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import an Easy-PC / DesignSpark PCB project, schematic or board.
     */
    void OnImportEasyPcFiles( wxCommandEvent& event );

    /**
     *  Open dialog to import an OrCAD schematic file.
     */
    void OnImportOrcadFiles( wxCommandEvent& event );

    /**
     * Prints the current working directory name and the project name on the text panel.
     */
    void PrintPrjInfo();

    void RefreshProjectTree();

    /**
     * Creates a new project by setting up and initial project, schematic, and board files.
     *
     * The project file is copied from the kicad.pro template file if possible.  Otherwise,
     * a minimal project file is created from an empty project.  A minimal schematic and
     * board file are created to prevent the schematic and board editors from complaining.
     * If any of these files already exist, they are not overwritten.
     *
     * @param aProjectFileName is the absolute path of the project file name.
     * @param aCreateStubFiles specifies if an empty PCB and schematic should be created
     */
    void CreateNewProject( const wxFileName& aProjectFileName, bool aCreateStubFiles = true );

    /**
     * Closes the project, and saves it if aSave is true;
     */
    bool CloseProject( bool aSave );

    /**
     * Loads a new project
     * @param aProjectFileName is the path to the project to load
     * @return true if the project was successfully loaded
     */
    bool LoadProject( const wxFileName& aProjectFileName );

    /// What to open: from the command line, the file manager or app_restart.
    struct OPEN_PLAN
    {
        wxString              project;     ///< project to load, empty: none
        std::vector<wxString> documents;   ///< schematic / board files of that project
        std::vector<wxString> editors;     ///< sch, pcb, 3d, symbol_editor, footprint_editor
        bool                  exact = false;   ///< only documents + editors: no session restore

        /// @brief True when the plan opens editor windows by itself.
        bool HasEditors() const { return !documents.empty() || !editors.empty(); }
    };

    /// @brief Turn file arguments into a plan.  Files that do not exist, are no KiCad project or
    /// document, or belong to another project than the first one are reported to the journal
    /// and dropped (one project per process).
    /// @param aFiles .kicad_pro / .kicad_sch / .kicad_pcb paths (relative to the working directory).
    /// @param aEditors editor names, see OPEN_PLAN::editors; unknown names are reported and dropped.
    /// @param aExact open exactly this (no session restore, no "open project shows editors").
    static OPEN_PLAN PlanOpen( const std::vector<wxString>& aFiles, const std::vector<wxString>& aEditors,
                               bool aExact );

    /// @brief Whether the project manager window is shown after start-up with aPlan.
    static bool ShowOnStart( PM_SHOW_ON_START aMode, PM_OPEN_PROJECT_SHOWS aProjectShows, const OPEN_PLAN& aPlan );

    /// @brief Open the documents and editors of aPlan; its project must be loaded already.  A
    /// plan without editors opens the project's editors when the window stays hidden or the
    /// settings say opening a project shows the editors.
    /// @return the number of editor windows opened.
    int OpenPlanned( const OPEN_PLAN& aPlan );

    /// @brief Open files at run time (file manager): their project first if it is not the
    /// current one, then the documents.
    void OpenFiles( const std::vector<wxString>& aFiles );

    /// @brief Start-up opening is done: from now on closing the last editor may quit and project
    /// switches with the window hidden open editors or show the window.
    void SetStartupDone() { m_startupDone = true; }

    /// @brief Called for wxEVT_CLOSE_WINDOW / wxEVT_SHOW of any window (the application's event
    /// filter): quits when the last editor went away and the settings say so.
    void TopLevelWindowEvent( wxEvent& aEvent );

    /// @brief Show this window when no KiCad window is on screen (macOS dock click).
    void ShowIfNothingShown();

    /// @brief True when a top-level frame other than this one is on screen.
    bool OtherWindowShown() const;

    void OpenJobsFile( const wxFileName& aFileName, bool aCreate = false,
                       bool aResaveProjectPreferences = true );


    void LoadSettings( APP_SETTINGS_BASE* aCfg ) override;

    void SaveSettings( APP_SETTINGS_BASE* aCfg ) override;

    void ShowChangedLanguage() override;
    void CommonSettingsChanged( int aFlags ) override;
    void ProjectChanged() override;

    void PreloadAllLibraries();

    /**
     * Called by sending a event with id = ID_INIT_WATCHED_PATHS
     * rebuild the list of watched paths
     */
    void OnChangeWatchedPaths( wxCommandEvent& aEvent );

    const wxString GetProjectFileName() const;

    bool IsProjectActive();
    // read only accessors
    const wxString SchFileName();
    const wxString SchLegacyFileName();
    const wxString PcbFileName();
    const wxString PcbLegacyFileName();

    void ReCreateTreePrj();

    /**
     * @param aIsExplicitUserSave is true to indicate the user ran a Save Project action explicitly
     *        Note that this parameter should currently *always* be false, because there is no
     *        explicit Save Project action in the project manager.  This means that anytime the
     *        project manager saves project local settings, it is an implicit save (and should not
     *        actually save the file if it was migrated)
     */
    void SaveOpenJobSetsToLocalSettings( bool aIsExplicitUserSave = false );

    wxWindow* GetToolCanvas() const override;

    std::shared_ptr<PLUGIN_CONTENT_MANAGER> GetPcm() { return m_pcm; };

    void SetPcmButton( BITMAP_BUTTON* aButton );

    void CreatePCM();   // creates the PLUGIN_CONTENT_MANAGER

    // Used only on Windows: stores the info message about file watcher
    wxString m_FileWatcherInfo;

    DECLARE_EVENT_TABLE()

protected:
    virtual void setupUIConditions() override;

    void doReCreateMenuBar() override;

    void onToolbarSizeChanged();

    void onNotebookPageCloseRequest( wxAuiNotebookEvent& evt );

    void onNotebookPageCountChanged( wxAuiNotebookEvent& evt );

private:
    void setupTools();
    void setupActions();

    void DoWithAcceptedFiles() override;

    APP_SETTINGS_BASE* config() const override;

    KICAD_SETTINGS* kicadSettings() const;

    const SEARCH_STACK& sys_search() override;

    wxString help_name() override;

    void updatePcmButtonBadge();

    /// @brief Reopen the editors open in the project's last session (remember_open_files), else
    /// the schematic (or the board when there is no schematic).
    /// @return the number of editors opened.
    int openProjectEditors();

    /// @brief Session restore loop (project local settings' open files).
    /// @return the number of editors opened.
    int restoreSessionEditors();

    /// @brief A project was loaded while this window is hidden: show its editors or this window.
    void showLoadedProject();

    void checkQuitWithLastEditor();
    void onQuitCheckTimer( wxTimerEvent& aEvent );

    static constexpr int QUIT_CHECK_DELAY_MS = 250;

private:
    bool                  m_openSavedWindows;
    bool                  m_skipSessionRestore = false;   ///< next session restore: only clear the file state
    bool                  m_startupDone = false;
    wxTimer               m_quitCheckTimer;                ///< quit-with-last-editor check after a close
    bool                  m_apiClosedWindow = false;      ///< the last window close came from an API call
    bool                  m_restoredFromHistory;  ///< Set after restore to mark editors dirty
    int                   m_leftWinWidth;
    bool                  m_active_project;
    bool                  m_showHistoryPanel;

    PROJECT_TREE_PANE*    m_projectTreePane;
    LOCAL_HISTORY_PANE*   m_historyPane;
    wxAuiNotebook*        m_notebook;
    PANEL_KICAD_LAUNCHER* m_launcher;
    int                   m_lastToolbarIconSize;

    std::shared_ptr<PLUGIN_CONTENT_MANAGER> m_pcm;
    BITMAP_BUTTON*                          m_pcmButton;
    int                                     m_pcmUpdateCount;
    std::unique_ptr<UPDATE_MANAGER>         m_updateManager;
};


// The C++ project manager includes a single PROJECT in its link image.
class PROJECT;
extern PROJECT& Prj();

#endif
