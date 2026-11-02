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

#pragma once

#include <memory>

#include <board_reannotate.h>
#include <dialogs/dialog_board_reannotate_base.h>
#include <layer_ids.h>
#include <netlist_reader/pcb_netlist.h>
#include <pcb_base_frame.h>
#include <pcb_edit_frame.h>
#include <project.h>
#include <frame_type.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <tools/pcb_selection_tool.h>

class DIALOG_BOARD_REANNOTATE : public DIALOG_BOARD_REANNOTATE_BASE
{
public:
    DIALOG_BOARD_REANNOTATE( PCB_EDIT_FRAME* aParentFrame );
    ~DIALOG_BOARD_REANNOTATE() = default;

private:
    std::vector<wxRadioButton*> m_sortButtons = {
            m_Down_Right,
            m_Right_Down,
            m_Down_Left,
            m_Left_Down,
            m_Up_Right,
            m_Right_Up,
            m_Up_Left,
            m_Left_Up
    };

    std::vector<wxRadioButton*> m_scopeRadioButtons = {
            m_AnnotateAll,
            m_AnnotateFront,
            m_AnnotateBack,
            m_AnnotateSelection
    };

    std::vector<wxStaticBitmap*> Bitmaps = {
            reannotate_down_right_bitmap,
            reannotate_right_down_bitmap,
            reannotate_down_left_bitmap,
            reannotate_left_down_bitmap,
            reannotate_up_right_bitmap,
            reannotate_right_up_bitmap,
            reannotate_up_left_bitmap,
            reannotate_left_up_bitmap
    };

    bool TransferDataToWindow() override;

    void OnApplyClick( wxCommandEvent& event ) override;
    void OnCloseClick( wxCommandEvent& event ) override;
    void FilterFrontPrefix( wxCommandEvent& event ) override;
    void FilterBackPrefix( wxCommandEvent& event ) override;

    /// Break report into strings separated by \n and sent to the reporter.
    void ShowReport( const wxString& aMessage, SEVERITY aSeverity );

	/// Create a list of the footprints and their coordinates.
    void LogFootprints( const std::vector<REFDES_INFO>& aFootprints );

    /// Create an audit trail of the changes.
    void LogChangePlan( void );

    /// Actually reannotate the board.
    /// @return false if fail, true if success.
    bool ReannotateBoard( void );

    /// Plan the change list from the dialog's options and log it.
    /// @return true if success, false if errors
    bool BuildFootprintList();

    /// Convert coordinates to wxString.
    /// @return the string
    wxString CoordTowxString( int aX, int aY );

    /// Check to make sure the prefix (if there is one) is properly constructed.
    void FilterPrefix( wxTextCtrl* aPrefix );

private:
    PCB_EDIT_FRAME*  m_frame;
    FOOTPRINTS       m_footprints;

    std::unique_ptr<BOARD_REANNOTATOR> m_reannotator;
    std::vector<wxString>              m_excludeArray;
};
