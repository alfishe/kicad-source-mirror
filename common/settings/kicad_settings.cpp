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
#include <wx/aui/framemanager.h>    // ensure class wxAuiPaneInfo is defined for other includes

#include "settings/kicad_settings.h"
#include <json_common.h>
#include <settings/aui_settings.h>
#include <settings/parameters.h>


///! Update the schema version whenever a migration is required
const int kicadSchemaVersion = 0;


const nlohmann::json PCM_DEFAULT_REPOSITORIES = nlohmann::json::array( {
    nlohmann::json( {
        { "name", "KiCad official repository" },
        { "url", PCM_DEFAULT_REPOSITORY_URL },
    } )
} );


KICAD_SETTINGS::KICAD_SETTINGS() :
        APP_SETTINGS_BASE( "kicad", kicadSchemaVersion ), m_LeftWinWidth( 200 ),
        m_ShowHistoryPanel( false )
{
    m_params.emplace_back( new PARAM<int>( "appearance.left_frame_width", &m_LeftWinWidth, 200 ) );
    m_params.emplace_back( new PARAM<bool>( "aui.show_history_panel", &m_ShowHistoryPanel, false ) );

    m_params.emplace_back( new PARAM_LIST<wxString>( "system.open_projects", &m_OpenProjects, {} ) );

    m_params.emplace_back( new PARAM<wxString>( "system.last_design_block_lib_dir", &m_lastDesignBlockLibDir, "" ) );

    m_params.emplace_back( new PARAM<wxString>( "system.last_update_check_time", &m_lastUpdateCheckTime, "" ) );
    m_params.emplace_back( new PARAM<wxString>( "system.last_received_update", &m_lastReceivedUpdate, "" ) );
    m_params.emplace_back( new PARAM<bool>( "system.check_for_kicad_updates", &m_KiCadUpdateCheck, true ) );

    m_params.emplace_back( new PARAM<wxPoint>( "template.window.pos", &m_TemplateWindowPos, wxDefaultPosition ) );
    m_params.emplace_back( new PARAM<wxSize>( "template.window.size", &m_TemplateWindowSize, wxDefaultSize ) );
    m_params.emplace_back( new PARAM<wxString>( "template.last_used", &m_LastUsedTemplate, "" ) );
    m_params.emplace_back( new PARAM_LIST<wxString>( "template.recent_templates", &m_RecentTemplates, {} ) );

    m_params.emplace_back( new PARAM<int>( "template.filter", &m_TemplateFilterChoice, 0 ) );

    m_params.emplace_back( new PARAM<wxString>( "template.browsed_path", &m_BrowsedTemplatesPath, "" ) );

    m_params.emplace_back( new PARAM_LAMBDA<nlohmann::json>(
            "pcm.repositories",
            [&]() -> nlohmann::json
            {
                nlohmann::json js = nlohmann::json::array();

                for( const auto& pair : m_PcmRepositories )
                {
                    js.push_back( nlohmann::json( { { "name", pair.first.ToUTF8() },
                                                    { "url", pair.second.ToUTF8() } } ) );
                }

                return js;
            },
            [&]( const nlohmann::json aObj )
            {
                m_PcmRepositories.clear();

                if( !aObj.is_array() )
                    return;

                for( const auto& entry : aObj )
                {
                    if( entry.empty() || !entry.is_object() )
                        continue;

                    m_PcmRepositories.emplace_back(
                            std::make_pair( wxString( entry["name"].get<std::string>() ),
                                            wxString( entry["url"].get<std::string>() ) ) );
                }
            },
            PCM_DEFAULT_REPOSITORIES ) );

    m_params.emplace_back( new PARAM_LAMBDA<nlohmann::json>(
            "libraries.overrides",
            [&]() -> nlohmann::json
            {
                nlohmann::json js = nlohmann::json::object();

                for( const auto& [tablePath, libs] : m_LibOverrides )
                {
                    nlohmann::json tableJs = nlohmann::json::object();

                    for( const auto& [nickname, override_] : libs )
                    {
                        nlohmann::json entry = nlohmann::json::object();

                        if( override_.disabled )
                            entry["disabled"] = true;

                        if( override_.hidden )
                            entry["hidden"] = true;

                        if( !entry.empty() )
                            tableJs[std::string( nickname.ToUTF8() )] = entry;
                    }

                    if( !tableJs.empty() )
                        js[std::string( tablePath.ToUTF8() )] = tableJs;
                }

                return js;
            },
            [&]( const nlohmann::json& aObj )
            {
                m_LibOverrides.clear();

                if( !aObj.is_object() )
                    return;

                for( const auto& [tablePath, tableObj] : aObj.items() )
                {
                    if( !tableObj.is_object() )
                        continue;

                    std::map<wxString, LIB_OVERRIDE>& libs =
                            m_LibOverrides[wxString::FromUTF8( tablePath )];

                    for( const auto& [nickname, entry] : tableObj.items() )
                    {
                        if( !entry.is_object() )
                            continue;

                        LIB_OVERRIDE override_;

                        if( entry.contains( "disabled" ) && entry["disabled"].is_boolean() )
                            override_.disabled = entry["disabled"].get<bool>();

                        if( entry.contains( "hidden" ) && entry["hidden"].is_boolean() )
                            override_.hidden = entry["hidden"].get<bool>();

                        libs[wxString::FromUTF8( nickname )] = override_;
                    }
                }
            },
            nlohmann::json::object() ) );

    // The getter omits an override once both flags clear, so the save has to delete the key
    m_params.back()->SetClearUnknownKeys();

    m_params.emplace_back(
            new PARAM<wxString>( "pcm.last_download_dir", &m_PcmLastDownloadDir, "" ) );

    m_params.emplace_back( new PARAM<bool>( "pcm.check_for_updates", &m_PcmUpdateCheck, true ) );

    m_params.emplace_back( new PARAM<bool>( "pcm.lib_auto_add", &m_PcmLibAutoAdd, true ) );

    m_params.emplace_back( new PARAM<bool>( "pcm.lib_auto_remove", &m_PcmLibAutoRemove, true ) );

    m_params.emplace_back( new PARAM<wxString>( "pcm.lib_prefix", &m_PcmLibPrefix,
                                                wxS( "PCM_" ) ) );

    m_params.emplace_back( new PARAM<wxString>( "pcm.last_selected_repo_id", &m_PcmLastSelectedRepoId, "" ) );

    m_params.emplace_back( new PARAM_LAMBDA<std::string>(
            "project_manager.show_on_start",
            [&]() { return ToString( m_ProjectManager.show_on_start ); },
            [&]( const std::string& aName )
            {
                m_ProjectManager.show_on_start = PM_SHOW_ON_START::ALWAYS;
                FromString( aName, m_ProjectManager.show_on_start );
            },
            ToString( PM_SHOW_ON_START::ALWAYS ) ) );

    m_params.emplace_back( new PARAM_LAMBDA<std::string>(
            "project_manager.open_project_shows",
            [&]() { return ToString( m_ProjectManager.open_project_shows ); },
            [&]( const std::string& aName )
            {
                m_ProjectManager.open_project_shows = PM_OPEN_PROJECT_SHOWS::MANAGER;
                FromString( aName, m_ProjectManager.open_project_shows );
            },
            ToString( PM_OPEN_PROJECT_SHOWS::MANAGER ) ) );

    m_params.emplace_back( new PARAM_LAMBDA<std::string>(
            "project_manager.quit_with_last_editor",
            [&]() { return ToString( m_ProjectManager.quit_with_last_editor ); },
            [&]( const std::string& aName )
            {
                m_ProjectManager.quit_with_last_editor = PM_QUIT_WITH_LAST_EDITOR::WHEN_HIDDEN;
                FromString( aName, m_ProjectManager.quit_with_last_editor );
            },
            ToString( PM_QUIT_WITH_LAST_EDITOR::WHEN_HIDDEN ) ) );
}


std::string KICAD_SETTINGS::ToString( PM_SHOW_ON_START aValue )
{
    switch( aValue )
    {
    case PM_SHOW_ON_START::WITHOUT_DOCUMENT: return "without_document";
    case PM_SHOW_ON_START::NEVER:            return "never";
    default:                                 return "always";
    }
}


std::string KICAD_SETTINGS::ToString( PM_OPEN_PROJECT_SHOWS aValue )
{
    return aValue == PM_OPEN_PROJECT_SHOWS::EDITORS ? "editors" : "manager";
}


std::string KICAD_SETTINGS::ToString( PM_QUIT_WITH_LAST_EDITOR aValue )
{
    switch( aValue )
    {
    case PM_QUIT_WITH_LAST_EDITOR::ALWAYS: return "always";
    case PM_QUIT_WITH_LAST_EDITOR::NEVER:  return "never";
    default:                               return "when_hidden";
    }
}


bool KICAD_SETTINGS::FromString( const std::string& aName, PM_SHOW_ON_START& aValue )
{
    for( PM_SHOW_ON_START v : { PM_SHOW_ON_START::ALWAYS, PM_SHOW_ON_START::WITHOUT_DOCUMENT,
                                PM_SHOW_ON_START::NEVER } )
    {
        if( aName == ToString( v ) )
        {
            aValue = v;
            return true;
        }
    }

    return false;
}


bool KICAD_SETTINGS::FromString( const std::string& aName, PM_OPEN_PROJECT_SHOWS& aValue )
{
    for( PM_OPEN_PROJECT_SHOWS v : { PM_OPEN_PROJECT_SHOWS::MANAGER, PM_OPEN_PROJECT_SHOWS::EDITORS } )
    {
        if( aName == ToString( v ) )
        {
            aValue = v;
            return true;
        }
    }

    return false;
}


bool KICAD_SETTINGS::FromString( const std::string& aName, PM_QUIT_WITH_LAST_EDITOR& aValue )
{
    for( PM_QUIT_WITH_LAST_EDITOR v : { PM_QUIT_WITH_LAST_EDITOR::WHEN_HIDDEN, PM_QUIT_WITH_LAST_EDITOR::ALWAYS,
                                        PM_QUIT_WITH_LAST_EDITOR::NEVER } )
    {
        if( aName == ToString( v ) )
        {
            aValue = v;
            return true;
        }
    }

    return false;
}


bool KICAD_SETTINGS::MigrateFromLegacy( wxConfigBase* aCfg )
{
    bool ret = APP_SETTINGS_BASE::MigrateFromLegacy( aCfg );

    ret &= fromLegacy<int>( aCfg, "LeftWinWidth", "appearance.left_frame_width" );

    // Override the size parameters to ensure the new PCM button is always shown.
    // This will make the window take the default size instead of the migrated one.
    Set( "window.size_x", 0 );
    Set( "window.size_y", 0 );

    return ret;
}
