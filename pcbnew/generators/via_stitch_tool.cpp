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
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

#include "via_stitch_tool.h"

#include <board_commit.h>
#include <pcb_track.h>
#include <generators/pcb_via_stitch.h>
#include <tool/conditional_menu.h>
#include <tool/tool_manager.h>
#include <tool/tool_menu.h>
#include <tools/generator_tool.h>
#include <tools/pcb_actions.h>
#include <tools/pcb_selection_tool.h>


bool VIA_STITCH_TOOL::Init()
{
    auto hasExclusionsCondition = []( const SELECTION& aSel )
    {
        for( EDA_ITEM* item : aSel )
        {
            if( PCB_VIA_STITCH* stitch = dynamic_cast<PCB_VIA_STITCH*>( item ) )
            {
                if( stitch->HasExclusions() )
                    return true;
            }
        }

        return false;
    };

    if( PCB_SELECTION_TOOL* selTool = m_toolMgr->GetTool<PCB_SELECTION_TOOL>() )
    {
        CONDITIONAL_MENU& menu = selTool->GetToolMenu().GetMenu();

        menu.AddItem( PCB_ACTIONS::clearStitchViaExclusions, hasExclusionsCondition, 100 );
    }

    return true;
}


int VIA_STITCH_TOOL::ExcludeStitchVia( const TOOL_EVENT& aEvent )
{
    PCB_SELECTION_TOOL*  selTool = m_toolMgr->GetTool<PCB_SELECTION_TOOL>();
    const PCB_SELECTION& sel = selTool->GetSelection();

    std::vector<PCB_VIA*> vias;

    for( EDA_ITEM* item : sel )
    {
        if( item->Type() == PCB_VIA_T && PCB_VIA_STITCH::StitchingOf( static_cast<PCB_VIA*>( item ) ) )
            vias.push_back( static_cast<PCB_VIA*>( item ) );
    }

    if( vias.empty() )
        return 0;

    selTool->ClearSelection();

    BOARD_COMMIT commit( this );
    PCB_VIA_STITCH::ExcludeVias( vias, m_toolMgr->GetTool<GENERATOR_TOOL>(), board(), commit );
    commit.Push( _( "Exclude Via From Stitching" ) );
    frame()->RefreshCanvas();
    return 0;
}


int VIA_STITCH_TOOL::ClearStitchViaExclusions( const TOOL_EVENT& aEvent )
{
    PCB_SELECTION_TOOL*  selTool = m_toolMgr->GetTool<PCB_SELECTION_TOOL>();
    const PCB_SELECTION& sel = selTool->GetSelection();

    std::vector<PCB_VIA_STITCH*> stitchings;

    for( EDA_ITEM* item : sel )
    {
        if( PCB_VIA_STITCH* stitch = dynamic_cast<PCB_VIA_STITCH*>( item ) )
            stitchings.push_back( stitch );
    }

    BOARD_COMMIT commit( this );

    if( PCB_VIA_STITCH::ClearExclusions( stitchings, m_toolMgr->GetTool<GENERATOR_TOOL>(), board(), commit ).empty() )
        return 0;

    commit.Push( _( "Clear Via Stitching Exclusions" ) );
    frame()->RefreshCanvas();
    return 0;
}


void VIA_STITCH_TOOL::setTransitions()
{
    Go( &VIA_STITCH_TOOL::ExcludeStitchVia, PCB_ACTIONS::excludeStitchVia.MakeEvent() );
    Go( &VIA_STITCH_TOOL::ClearStitchViaExclusions, PCB_ACTIONS::clearStitchViaExclusions.MakeEvent() );
}
