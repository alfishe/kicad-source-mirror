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

#include <tool/project_manager_menu.h>

#include <bitmaps.h>
#include <eda_base_frame.h>
#include <file_history.h>
#include <frame_type.h>
#include <id.h>
#include <kiway.h>
#include <tool/tool_event.h>

#include <wx/filename.h>


// Action names are translated where they are shown
#undef _
#define _( s ) s

TOOL_ACTION PROJECT_MANAGER_ACTIONS::newProject( TOOL_ACTION_ARGS()
        .Name( "common.Control.newProjectInManager" )
        .Scope( AS_GLOBAL )
        .FriendlyName( _( "New Project..." ) )
        .Tooltip( _( "Create a new project (closes the editors of this one)" ) )
        .Icon( BITMAPS::new_project_from_template ) );

TOOL_ACTION PROJECT_MANAGER_ACTIONS::openProject( TOOL_ACTION_ARGS()
        .Name( "common.Control.openProjectInManager" )
        .Scope( AS_GLOBAL )
        .FriendlyName( _( "Open Project..." ) )
        .Tooltip( _( "Open another project (closes the editors of this one)" ) )
        .Icon( BITMAPS::open_project ) );

TOOL_ACTION PROJECT_MANAGER_ACTIONS::openRecentProject( TOOL_ACTION_ARGS()
        .Name( "common.Control.openRecentProjectInManager" )
        .Scope( AS_GLOBAL )
        .Parameter<wxString*>( nullptr ) );

#undef _
#define _( s ) wxGetTranslation( ( s ) )


/// @brief The project manager frame of the process, nullptr for a stand-alone editor
static EDA_BASE_FRAME* projectManager( KIWAY& aKiway )
{
    // static_cast: dynamic_cast across the KIWAY fails on macOS (see COMMON_CONTROL)
    EDA_BASE_FRAME* top = static_cast<EDA_BASE_FRAME*>( aKiway.GetTop() );

    return top && top->GetFrameType() == KICAD_MAIN_FRAME_T ? top : nullptr;
}


RECENT_PROJECTS_MENU::RECENT_PROJECTS_MENU( TOOL_INTERACTIVE* aTool, EDA_BASE_FRAME* aManager ) :
        ACTION_MENU( false, aTool ),
        m_manager( aManager )
{
    SetTitle( _( "Open Recent Project" ) );
    SetIcon( BITMAPS::recent );

    if( !m_manager )
        return;

    FILE_HISTORY& history = m_manager->GetFileHistory();

    for( size_t i = 0; i < history.GetCount() && i < MAX_FILE_HISTORY_SIZE; i++ )
    {
        wxString path = history.GetHistoryFile( i );

        // menu ids local to this menu: its eventHandler maps them back before anyone else sees them
        Add( wxFileName( path ).GetName(), path, ID_POPUP_MENU_START + static_cast<int>( i ), BITMAPS::INVALID_BITMAP );
        m_projects.push_back( path );
    }
}


ACTION_MENU* RECENT_PROJECTS_MENU::create() const
{
    return new RECENT_PROJECTS_MENU( m_tool, m_manager );
}


OPT_TOOL_EVENT RECENT_PROJECTS_MENU::eventHandler( const wxMenuEvent& aEvent )
{
    const int index = aEvent.GetId() - ID_POPUP_MENU_START;

    if( index < 0 || index >= static_cast<int>( m_projects.size() ) )
        return OPT_TOOL_EVENT();

    TOOL_EVENT evt = PROJECT_MANAGER_ACTIONS::openRecentProject.MakeEvent();
    evt.SetParameter<wxString*>( &m_projects[index] );
    return evt;
}


bool AddProjectManagerItems( ACTION_MENU* aFileMenu, TOOL_INTERACTIVE* aTool, KIWAY& aKiway )
{
    EDA_BASE_FRAME* manager = projectManager( aKiway );

    if( !manager )
        return false;

    aFileMenu->Add( PROJECT_MANAGER_ACTIONS::newProject );
    aFileMenu->Add( PROJECT_MANAGER_ACTIONS::openProject );

    RECENT_PROJECTS_MENU* recent = new RECENT_PROJECTS_MENU( aTool, manager );
    wxMenuItem*           item = aFileMenu->Add( recent );

    item->Enable( recent->GetMenuItemCount() > 0 );
    return true;
}
