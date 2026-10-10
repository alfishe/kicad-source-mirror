// This program source code file is part of KiCad, a free EDA CAD application.
//
// Copyright The KiCad Developers, see AUTHORS.txt for contributors.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/// @file project_manager_menu.h
/// @brief Project items of the editors' File menu in project mode (New / Open / Recent Project),
/// run by the project manager even while its window is hidden.

#ifndef PROJECT_MANAGER_MENU_H
#define PROJECT_MANAGER_MENU_H

#include <tool/action_menu.h>
#include <tool/tool_action.h>

#include <vector>

class EDA_BASE_FRAME;
class KIWAY;


/// Editor-side actions forwarded to the project manager (COMMON_CONTROL::ForwardToProjectManager).
class PROJECT_MANAGER_ACTIONS
{
public:
    static TOOL_ACTION newProject;
    static TOOL_ACTION openProject;
    static TOOL_ACTION openRecentProject;   ///< parameter: wxString* project file
};


/// Recent projects of the project manager; built when the menu is created (an editor lives
/// within one project, a project switch recreates it).
class RECENT_PROJECTS_MENU : public ACTION_MENU
{
public:
    /// @param aManager the project manager frame whose file history is listed.
    RECENT_PROJECTS_MENU( TOOL_INTERACTIVE* aTool, EDA_BASE_FRAME* aManager );

protected:
    ACTION_MENU*   create() const override;
    OPT_TOOL_EVENT eventHandler( const wxMenuEvent& aEvent ) override;

private:
    EDA_BASE_FRAME*       m_manager;
    std::vector<wxString> m_projects;
};


/// @brief Append New Project / Open Project / Open Recent Project to an editor's File menu when
/// the editor runs under the project manager; nothing in a stand-alone editor.
/// @param aFileMenu the File menu being built.
/// @param aTool the tool owning the menu (selection tool).
/// @param aKiway the editor's KIWAY.
/// @return true when items were added (the caller may add a separator).
bool AddProjectManagerItems( ACTION_MENU* aFileMenu, TOOL_INTERACTIVE* aTool, KIWAY& aKiway );

#endif
