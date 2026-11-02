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

#ifndef CONSTRAINT_EDIT_H
#define CONSTRAINT_EDIT_H

#include <kiid.h>
#include <wx/string.h>

#include <memory>
#include <optional>
#include <set>
#include <vector>

class BOARD;
class BOARD_COMMIT;
class PCB_CONSTRAINT;
class PCB_SHAPE;
struct BOARD_CONSTRAINT_DIAGNOSTICS;


/// Why a constraint just added cannot do its job (it stays on the board)
enum class CONSTRAINT_ADD_ISSUE
{
    NONE,
    ALL_LOCKED,     ///< every member is locked: nothing may move
    CONFLICTING,    ///< over-constrains its cluster
    NOT_APPLICABLE  ///< a member is missing or of a kind the type cannot take
};

/// The editor's infobar text for an issue; empty for NONE
wxString ConstraintAddIssueText( CONSTRAINT_ADD_ISSUE aIssue );

/// True when every item the constraint references resolves and is locked
bool ConstraintMembersLocked( BOARD* aBoard, const PCB_CONSTRAINT* aConstraint );

/// The shapes a constraint references
std::vector<PCB_SHAPE*> ConstraintMemberShapes( BOARD* aBoard, const PCB_CONSTRAINT* aConstraint );

/**
 * Snap the geometry of a constraint just pushed to the board, staging what moves in @p aCommit
 * (the editor pushes it with APPEND_UNDO so adding stays one step).  A fixed position has nothing
 * to snap.
 */
void SnapAddedConstraint( BOARD* aBoard, PCB_CONSTRAINT* aConstraint, BOARD_COMMIT& aCommit );

/// The issue of a constraint just added and snapped, given a fresh board diagnosis
CONSTRAINT_ADD_ISSUE CheckAddedConstraint( BOARD* aBoard, const PCB_CONSTRAINT* aConstraint,
                                           const BOARD_CONSTRAINT_DIAGNOSTICS& aDiag );

/**
 * Change a dimensional constraint's value and driving flag and snap the geometry to it, all
 * staged in @p aCommit.  @p aValue is in IU for a length, degrees for an angle.
 * @return false when the constraint has no value.
 */
bool SetConstraintValue( BOARD* aBoard, PCB_CONSTRAINT* aConstraint, std::optional<double> aValue,
                         std::optional<bool> aDriving, BOARD_COMMIT& aCommit );

/// The constraints of the board (and of its first footprint with @p aFootprintEditor) that
/// reference any of @p aItems
std::vector<PCB_CONSTRAINT*> ConstraintsReferencing( BOARD* aBoard, const std::set<KIID>& aItems,
                                                     bool aFootprintEditor );

#endif
