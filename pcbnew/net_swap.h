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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <wx/string.h>

class BOARD;
class BOARD_COMMIT;
class BOARD_CONNECTED_ITEM;
class PAD;


/**
 * Swap Pad Nets / Swap Gate Nets planned without the editor: the nets the swapped pads take
 * (a cyclic rotation in the given order) and the copper connected to them that follows.
 */
struct NET_SWAP_PLAN
{
    std::vector<std::pair<PAD*, int>>              PadNewNets;     ///< the swapped pads, in order
    std::unordered_set<PAD*>                       SwapPads;
    std::unordered_map<BOARD_CONNECTED_ITEM*, int> ItemNewNets;    ///< pads and connected copper
    std::vector<PAD*>                              ConnectedPads;  ///< other pads the nets reach
    std::vector<int>                               Units;          ///< gate swap: unit indices, in order
    wxString                                       Error;          ///< why there is no plan

    /// Copper (or, with aIncludeConnectedPads, other pads) that would change and is locked.
    std::vector<BOARD_CONNECTED_ITEM*> LockedItems( bool aIncludeConnectedPads ) const;

    /// Set the planned nets through aCommit; ConnectedPads only with aIncludeConnectedPads.
    void Apply( BOARD_COMMIT& aCommit, bool aIncludeConnectedPads ) const;
};


/**
 * Swap Pad Nets: each pad takes the net of the next one (the last the first's), the copper
 * connected to each pad on its old net follows.
 * @return false with aPlan.Error set, or with no error when the pads all share one net.
 */
bool PlanPadNetSwap( BOARD* aBoard, const std::vector<PAD*>& aPads, NET_SWAP_PLAN& aPlan );

/**
 * Swap Gate Nets: the units (gates) of one multi-unit footprint that aPads touch, in order, each
 * take the nets of the next one pin by pin; one unit and aTargetUnit (a unit name with as many
 * pins) swaps the two.
 * @return false with aPlan.Error set, or with no error when the gates' nets are the same.
 */
bool PlanGateNetSwap( BOARD* aBoard, const std::vector<PAD*>& aPads, const wxString& aTargetUnit,
                      NET_SWAP_PLAN& aPlan );
