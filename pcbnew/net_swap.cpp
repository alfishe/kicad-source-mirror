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

#include <net_swap.h>

#include <algorithm>

#include <board.h>
#include <board_commit.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <pad.h>
#include <wx/translation.h>


namespace
{

// Makes sure we don't try to assign two different new nets to an item. Assigning
// the same net to the item multiples times is fine.
bool scheduleNetSwap( BOARD_CONNECTED_ITEM* aItem, int aNewNet, NET_SWAP_PLAN& aPlan )
{
    auto [it, inserted] = aPlan.ItemNewNets.emplace( aItem, aNewNet );

    if( !inserted && it->second != aNewNet )
    {
        aPlan.Error = _( "Cannot swap nets: connected items would receive conflicting nets." );
        return false;
    }

    return true;
}


// The copper connected to aPad on aFromNet follows it to aToNet. Zones are left alone: the user
// probably doesn't want to change zone nets.
bool scheduleConnected( CONNECTIVITY_DATA& aConnectivity, PAD* aPad, int aFromNet, int aToNet,
                        NET_SWAP_PLAN& aPlan )
{
    for( BOARD_CONNECTED_ITEM* ci : aConnectivity.GetConnectedItems( aPad, EXCLUDE_ZONES ) )
    {
        switch( ci->Type() )
        {
        case PCB_TRACE_T:
        case PCB_ARC_T:
        case PCB_VIA_T:
        case PCB_PAD_T:
        case PCB_SHAPE_T:
            break;

        default:
            continue;
        }

        if( ci->GetNetCode() != aFromNet )
            continue;

        if( !scheduleNetSwap( ci, aToNet, aPlan ) )
            return false;

        if( ci->Type() == PCB_PAD_T )
        {
            PAD* other = static_cast<PAD*>( ci );

            if( !aPlan.SwapPads.count( other )
                && std::find( aPlan.ConnectedPads.begin(), aPlan.ConnectedPads.end(), other )
                           == aPlan.ConnectedPads.end() )
            {
                aPlan.ConnectedPads.push_back( other );
            }
        }
    }

    return true;
}

} // namespace


std::vector<BOARD_CONNECTED_ITEM*> NET_SWAP_PLAN::LockedItems( bool aIncludeConnectedPads ) const
{
    std::vector<BOARD_CONNECTED_ITEM*> locked;

    for( const auto& [item, newNet] : ItemNewNets )
    {
        if( item->GetNetCode() == newNet )
            continue;

        if( item->Type() == PCB_PAD_T && !aIncludeConnectedPads && !SwapPads.count( static_cast<PAD*>( item ) ) )
            continue;

        if( item->IsLocked() )
            locked.push_back( item );
    }

    return locked;
}


void NET_SWAP_PLAN::Apply( BOARD_COMMIT& aCommit, bool aIncludeConnectedPads ) const
{
    // Selected pads get their new nets directly
    for( const auto& [pad, newNet] : PadNewNets )
    {
        if( pad->GetNetCode() == newNet )
            continue;

        aCommit.Modify( pad );
        pad->SetNetCode( newNet );
    }

    // Connected items propagate, depending on the choice for unselected pads
    for( const auto& [item, newNet] : ItemNewNets )
    {
        if( item->Type() == PCB_PAD_T )
        {
            PAD* p = static_cast<PAD*>( item );

            if( SwapPads.count( p ) )
                continue;

            if( !aIncludeConnectedPads )
                continue;
        }

        if( item->GetNetCode() == newNet )
            continue;

        aCommit.Modify( item );
        item->SetNetCode( newNet );
    }
}


bool PlanPadNetSwap( BOARD* aBoard, const std::vector<PAD*>& aPads, NET_SWAP_PLAN& aPlan )
{
    aPlan = NET_SWAP_PLAN();

    const size_t     padsCount = aPads.size();
    std::vector<int> originalNets( padsCount );

    if( padsCount < 2 )
        return false;

    for( size_t i = 0; i < padsCount; ++i )
    {
        originalNets[i] = aPads[i]->GetNetCode();
        aPlan.SwapPads.insert( aPads[i] );
    }

    if( std::ranges::all_of( originalNets,
                             [&]( int net )
                             {
                                 return net == originalNets.front();
                             } ) )
    {
        return false;
    }

    // Desired new nets are a cyclic rotation of original nets (like Swap positions)
    auto newNetForIndex =
            [&]( size_t i )
            {
                return originalNets[( i + 1 ) % padsCount];
            };

    // Include the direct pad changes so connectivity cannot assign them a conflicting net.
    for( size_t i = 0; i < padsCount; ++i )
    {
        aPlan.PadNewNets.emplace_back( aPads[i], newNetForIndex( i ) );

        if( !scheduleNetSwap( aPads[i], newNetForIndex( i ), aPlan ) )
            return false;
    }

    std::shared_ptr<CONNECTIVITY_DATA> connectivity = aBoard->GetConnectivity();

    for( size_t i = 0; i < padsCount; ++i )
    {
        if( !scheduleConnected( *connectivity, aPads[i], originalNets[i], newNetForIndex( i ), aPlan ) )
            return false;
    }

    return true;
}


bool PlanGateNetSwap( BOARD* aBoard, const std::vector<PAD*>& aPads, const wxString& aTargetUnit,
                      NET_SWAP_PLAN& aPlan )
{
    aPlan = NET_SWAP_PLAN();

    const wxString notOneFootprint = _( "Gate swapping must be performed on pads within one multi-gate footprint." );
    FOOTPRINT*     targetFp = nullptr;

    for( PAD* pad : aPads )
    {
        FOOTPRINT* fp = pad->GetParentFootprint();

        if( !targetFp )
        {
            targetFp = fp;
        }
        else if( fp && targetFp != fp )
        {
            aPlan.Error = notOneFootprint;
            return false;
        }
    }

    if( !targetFp || targetFp->GetUnitInfo().size() < 2 )
    {
        aPlan.Error = notOneFootprint;
        return false;
    }

    const auto& units = targetFp->GetUnitInfo();

    // Collect unit hits and ordered unit list based on the pads' order
    std::vector<bool> unitHit( units.size(), false );
    std::vector<int>  unitOrder;

    for( PAD* pad : aPads )
    {
        const wxString& padNum = pad->GetNumber();
        int             unitIdx = -1;

        for( size_t i = 0; i < units.size(); ++i )
        {
            for( const auto& p : units[i].m_pins )
            {
                if( p == padNum )
                {
                    unitIdx = static_cast<int>( i );

                    if( !unitHit[i] )
                        unitOrder.push_back( unitIdx );

                    unitHit[i] = true;
                    break;
                }
            }

            if( unitIdx >= 0 )
                break;
        }
    }

    // Determine active units: 0 -> bail, 1 -> needs a target unit name, 2+ -> cycle
    std::vector<int> activeUnitIdx;

    if( unitOrder.size() >= 2 )
    {
        activeUnitIdx = unitOrder;
    }
    else if( unitOrder.size() == 1 && !aTargetUnit.IsEmpty() )
    {
        const int sourceIdx = unitOrder.front();
        int       targetIdx = -1;

        for( size_t i = 0; i < units.size(); ++i )
        {
            if( static_cast<int>( i ) == sourceIdx )
                continue;

            if( units[i].m_pins.size() == units[sourceIdx].m_pins.size() && units[i].m_unitName == aTargetUnit )
                targetIdx = static_cast<int>( i );
        }

        if( targetIdx < 0 )
        {
            aPlan.Error = notOneFootprint;
            return false;
        }

        activeUnitIdx.push_back( sourceIdx );
        activeUnitIdx.push_back( targetIdx );
    }
    else
    {
        aPlan.Error = notOneFootprint;
        return false;
    }

    aPlan.Units = activeUnitIdx;

    // Verify equal pin counts across all active units
    const size_t pinCount = units[activeUnitIdx.front()].m_pins.size();

    for( int idx : activeUnitIdx )
    {
        if( units[idx].m_pins.size() != pinCount )
        {
            aPlan.Error = _( "Gate swapping must be performed on gates with equal pin counts." );
            return false;
        }
    }

    // Build per-unit pad arrays and net vectors
    const size_t                   unitCount = activeUnitIdx.size();
    std::vector<std::vector<PAD*>> unitPads( unitCount );
    std::vector<std::vector<int>>  unitNets( unitCount );

    for( size_t ui = 0; ui < unitCount; ++ui )
    {
        const auto& pins = units[activeUnitIdx[ui]].m_pins;

        for( size_t pi = 0; pi < pinCount; ++pi )
        {
            PAD* p = targetFp->FindPadByNumber( pins[pi] );

            if( !p )
            {
                aPlan.Error = _( "Gate swapping failed: pad in unit missing from footprint." );
                return false;
            }

            unitPads[ui].push_back( p );
            unitNets[ui].push_back( p->GetNetCode() );
        }
    }

    if( std::ranges::all_of( unitNets,
                             [&]( const auto& nets )
                             {
                                 return nets == unitNets.front();
                             } ) )
    {
        aPlan.Error = _( "Gate swapping has no effect: all selected gates have identical nets." );
        return false;
    }

    for( const auto& v : unitPads )
        aPlan.SwapPads.insert( v.begin(), v.end() );

    // Rotate nets among units forward, per pin position
    for( size_t pi = 0; pi < pinCount; ++pi )
    {
        for( size_t ui = 0; ui < unitCount; ++ui )
        {
            const size_t toIdx = ( ui + 1 ) % unitCount;

            aPlan.PadNewNets.emplace_back( unitPads[ui][pi], unitNets[toIdx][pi] );

            if( !scheduleNetSwap( unitPads[ui][pi], unitNets[toIdx][pi], aPlan ) )
                return false;
        }
    }

    std::shared_ptr<CONNECTIVITY_DATA> connectivity = aBoard->GetConnectivity();

    for( size_t pi = 0; pi < pinCount; ++pi )
    {
        for( size_t ui = 0; ui < unitCount; ++ui )
        {
            const size_t toIdx = ( ui + 1 ) % unitCount;

            if( !scheduleConnected( *connectivity, unitPads[ui][pi], unitNets[ui][pi], unitNets[toIdx][pi], aPlan ) )
                return false;
        }
    }

    return true;
}
