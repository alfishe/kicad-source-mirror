/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2020 Jon Evans <jon@craftyjon.com>
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

#ifndef _KICAD_SETTINGS_H
#define _KICAD_SETTINGS_H

#include <map>
#include <set>
#include <settings/app_settings.h>
#define PCM_DEFAULT_REPOSITORY_URL "https://repository.kicad.org/repository.json"


/**
 * Per-library override flags for libraries in read-only nested tables.
 * These are keyed by the table file path and library nickname so they survive
 * table updates while preserving user choices.
 */
struct LIB_OVERRIDE
{
    bool disabled = false;
    bool hidden = false;
};


/// When the project manager window is shown at start-up.
enum class PM_SHOW_ON_START
{
    ALWAYS,             ///< always (classic behaviour)
    WITHOUT_DOCUMENT,   ///< only when no schematic / board is opened on the command line
    NEVER               ///< never; shown only when nothing else could be opened
};


/// What opening a project (command line, file manager, editors' File menu) shows.
enum class PM_OPEN_PROJECT_SHOWS
{
    MANAGER,            ///< the project manager (classic behaviour)
    EDITORS             ///< the editors open in the last session, else the schematic
};


/// Whether closing the last editor window ends KiCad.
enum class PM_QUIT_WITH_LAST_EDITOR
{
    WHEN_HIDDEN,        ///< only while the project manager window is hidden
    ALWAYS,
    NEVER
};


class KICOMMON_API KICAD_SETTINGS : public APP_SETTINGS_BASE
{
public:
    KICAD_SETTINGS();

    virtual ~KICAD_SETTINGS() {}

    virtual bool MigrateFromLegacy( wxConfigBase* aLegacyConfig ) override;

    int m_LeftWinWidth;
    bool m_ShowHistoryPanel;


    std::vector<wxString> m_OpenProjects;

    wxString m_lastDesignBlockLibDir;

    std::vector<std::pair<wxString, wxString>> m_PcmRepositories;
    wxString                                   m_PcmLastDownloadDir;

    // This controls background update check for PCM.
    // It is set according to m_updateCheck on first start.
    bool m_PcmUpdateCheck;
    // Auto add libs to global table
    bool m_PcmLibAutoAdd;
    // Auto remove libs
    bool m_PcmLibAutoRemove;
    // Generated library nickname prefix
    wxString m_PcmLibPrefix;
    // Last used repository (for pre-selection in dialog)
    wxString m_PcmLastSelectedRepoId;

    bool     m_KiCadUpdateCheck;
    wxString m_lastUpdateCheckTime;
    wxString m_lastReceivedUpdate;

    // Last position of the template window
    wxPoint m_TemplateWindowPos;
    // Last size of the template window
    wxSize m_TemplateWindowSize;
    // Last used project template path (for pre-selection in dialog)
    wxString m_LastUsedTemplate;

    std::vector<wxString> m_RecentTemplates;
    int                   m_TemplateFilterChoice = 0;
    // Most recently browsed external template directory (restored in the template selector
    // so users can keep using templates from arbitrary locations across sessions).
    wxString              m_BrowsedTemplatesPath;

    /// Overrides for libraries in read-only nested tables.
    /// Outer key is normalized table file path, inner key is library nickname.
    std::map<wxString, std::map<wxString, LIB_OVERRIDE>> m_LibOverrides;

    /// Project manager window behaviour (kicad.json "project_manager").
    struct PROJECT_MANAGER_BEHAVIOUR
    {
        PM_SHOW_ON_START         show_on_start = PM_SHOW_ON_START::ALWAYS;
        PM_OPEN_PROJECT_SHOWS    open_project_shows = PM_OPEN_PROJECT_SHOWS::MANAGER;
        PM_QUIT_WITH_LAST_EDITOR quit_with_last_editor = PM_QUIT_WITH_LAST_EDITOR::WHEN_HIDDEN;
    };

    PROJECT_MANAGER_BEHAVIOUR m_ProjectManager;

    /// @brief Names of the project manager settings as stored in kicad.json.
    static std::string ToString( PM_SHOW_ON_START aValue );
    static std::string ToString( PM_OPEN_PROJECT_SHOWS aValue );
    static std::string ToString( PM_QUIT_WITH_LAST_EDITOR aValue );

    /// @brief Parse a stored name.
    /// @return false (aValue unchanged) when the name is unknown.
    static bool FromString( const std::string& aName, PM_SHOW_ON_START& aValue );
    static bool FromString( const std::string& aName, PM_OPEN_PROJECT_SHOWS& aValue );
    static bool FromString( const std::string& aName, PM_QUIT_WITH_LAST_EDITOR& aValue );

protected:
    virtual std::string getLegacyFrameName() const override { return "KicadFrame"; }
};

#endif
