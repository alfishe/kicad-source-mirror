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

#include <constraints/constraint_edit.h>

#include <board.h>
#include <board_commit.h>
#include <constraints/board_constraint_adapter.h>
#include <constraints/pcb_constraint.h>
#include <core/kicad_algo.h>
#include <footprint.h>
#include <pcb_shape.h>
#include <wx/intl.h>

#include <algorithm>


wxString ConstraintAddIssueText( CONSTRAINT_ADD_ISSUE aIssue )
{
    switch( aIssue )
    {
    case CONSTRAINT_ADD_ISSUE::ALL_LOCKED:
        return _( "All items referenced by this constraint are locked, so the constraint cannot move any "
                  "geometry." );
    case CONSTRAINT_ADD_ISSUE::CONFLICTING:
        return _( "The new geometric constraint conflicts with existing constraints and could not be satisfied." );
    case CONSTRAINT_ADD_ISSUE::NOT_APPLICABLE:
        return _( "The new geometric constraint could not be applied to the referenced items." );
    case CONSTRAINT_ADD_ISSUE::NONE:
        break;
    }

    return wxEmptyString;
}


bool ConstraintMembersLocked( BOARD* aBoard, const PCB_CONSTRAINT* aConstraint )
{
    if( !aBoard || !aConstraint || aConstraint->GetMembers().empty() )
        return false;

    return std::ranges::all_of( aConstraint->GetMembers(),
                                [&]( const CONSTRAINT_MEMBER& aMember )
                                {
                                    BOARD_ITEM* item = aBoard->ResolveItem( aMember.m_item, true );
                                    return item && ConstraintItemIsLocked( item );
                                } );
}


std::vector<PCB_SHAPE*> ConstraintMemberShapes( BOARD* aBoard, const PCB_CONSTRAINT* aConstraint )
{
    std::vector<PCB_SHAPE*> shapes;

    for( const CONSTRAINT_MEMBER& member : aConstraint->GetMembers() )
    {
        if( PCB_SHAPE* shape = dynamic_cast<PCB_SHAPE*>( aBoard->ResolveItem( member.m_item, true ) ) )
            shapes.push_back( shape );
    }

    return shapes;
}


void SnapAddedConstraint( BOARD* aBoard, PCB_CONSTRAINT* aConstraint, BOARD_COMMIT& aCommit )
{
    // A fix is enforced by dropping its point from the unknowns; re-solving would only expose the
    // pinned shape to the cluster's other constraints with its own stay-put pin lifted.
    if( aConstraint->GetConstraintType() == PCB_CONSTRAINT_TYPE::FIXED_POSITION )
        return;

    std::vector<PCB_SHAPE*> modified;

    ApplyConstraintImmediately( aBoard, aConstraint, &modified,
                                [&]( BOARD_ITEM* aItem )
                                {
                                    aCommit.Modify( aItem );
                                },
                                ConstraintReferenceShapes( aBoard, aConstraint ) );
}


CONSTRAINT_ADD_ISSUE CheckAddedConstraint( BOARD* aBoard, const PCB_CONSTRAINT* aConstraint,
                                           const BOARD_CONSTRAINT_DIAGNOSTICS& aDiag )
{
    // a fix asks for exactly what a locked item gives
    if( aConstraint->GetConstraintType() != PCB_CONSTRAINT_TYPE::FIXED_POSITION
        && ConstraintMembersLocked( aBoard, aConstraint ) )
    {
        return CONSTRAINT_ADD_ISSUE::ALL_LOCKED;
    }

    if( alg::contains( aDiag.conflicting, aConstraint->m_Uuid ) )
        return CONSTRAINT_ADD_ISSUE::CONFLICTING;

    if( alg::contains( aDiag.errored, aConstraint->m_Uuid ) )
        return CONSTRAINT_ADD_ISSUE::NOT_APPLICABLE;

    return CONSTRAINT_ADD_ISSUE::NONE;
}


bool SetConstraintValue( BOARD* aBoard, PCB_CONSTRAINT* aConstraint, std::optional<double> aValue,
                         std::optional<bool> aDriving, BOARD_COMMIT& aCommit )
{
    if( !aConstraint || !aConstraint->HasValue() )
        return false;

    aCommit.Modify( aConstraint );

    if( aValue )
        aConstraint->SetValue( *aValue );

    if( aDriving )
        aConstraint->SetDriving( *aDriving );

    // the shapes follow a driving value in the same commit, as creation does
    ApplyConstraintImmediately( aBoard, aConstraint, nullptr,
                                [&]( BOARD_ITEM* aItem )
                                {
                                    aCommit.Modify( aItem );
                                } );
    return true;
}


std::vector<PCB_CONSTRAINT*> ConstraintsReferencing( BOARD* aBoard, const std::set<KIID>& aItems,
                                                     bool aFootprintEditor )
{
    std::vector<PCB_CONSTRAINT*> found;

    auto scan = [&]( const CONSTRAINTS& aConstraints )
    {
        for( PCB_CONSTRAINT* constraint : aConstraints )
        {
            if( std::ranges::any_of( constraint->GetMembers(),
                                     [&]( const CONSTRAINT_MEMBER& aMember )
                                     {
                                         return aItems.contains( aMember.m_item );
                                     } ) )
            {
                found.push_back( constraint );
            }
        }
    };

    if( !aBoard )
        return found;

    scan( aBoard->Constraints() );

    if( aFootprintEditor && aBoard->GetFirstFootprint() )
        scan( aBoard->GetFirstFootprint()->Constraints() );

    return found;
}
