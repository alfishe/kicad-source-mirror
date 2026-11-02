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

#include <zone_merge.h>

#include <board.h>
#include <board_commit.h>
#include <zone.h>

#include <algorithm>


ZONE_MERGE_PLAN PlanZoneMerge( BOARD* aBoard, const std::vector<ZONE*>& aZones )
{
    ZONE_MERGE_PLAN plan;

    if( aZones.empty() )
    {
        plan.error = _( "Select two or more zones to merge." );
        return plan;
    }

    ZONE*              first = aZones.front();
    std::vector<ZONE*> pending;

    plan.zones.push_back( first );

    for( size_t i = 1; i < aZones.size(); i++ )
    {
        ZONE* zone = aZones[i];

        if( zone == first || std::find( pending.begin(), pending.end(), zone ) != pending.end() )
            continue;

        if( zone->GetIsRuleArea() != first->GetIsRuleArea() )
            plan.skipped.emplace_back( zone, ZONE_MERGE_SKIP::OTHER_KIND );
        else if( zone->GetNetCode() != first->GetNetCode() )
            plan.skipped.emplace_back( zone, ZONE_MERGE_SKIP::OTHER_NET );
        else if( zone->GetLayerSet() != first->GetLayerSet() )
            plan.skipped.emplace_back( zone, ZONE_MERGE_SKIP::OTHER_LAYERS );
        else
            pending.push_back( zone );
    }

    // overlapping any zone already taken, in any order
    for( bool grew = true; grew && !pending.empty(); )
    {
        grew = false;

        for( auto it = pending.begin(); it != pending.end(); )
        {
            const bool touches = std::any_of( plan.zones.begin(), plan.zones.end(),
                                              [&]( ZONE* aTaken )
                                              {
                                                  return aBoard->TestZoneIntersection( *it, aTaken );
                                              } );

            if( touches )
            {
                plan.zones.push_back( *it );
                it = pending.erase( it );
                grew = true;
            }
            else
            {
                ++it;
            }
        }
    }

    for( ZONE* zone : pending )
        plan.skipped.emplace_back( zone, ZONE_MERGE_SKIP::NO_OVERLAP );

    if( plan.zones.size() < 2 )
    {
        plan.error = _( "No zone to merge with the first one." );
        return plan;
    }

    plan.outline = first->Outline()->CloneDropTriangulation();
    plan.outline.ClearArcs();
    plan.priority = first->GetAssignedPriority();

    for( size_t i = 1; i < plan.zones.size(); i++ )
    {
        SHAPE_POLY_SET other = plan.zones[i]->Outline()->CloneDropTriangulation();
        other.ClearArcs();
        plan.outline.BooleanAdd( other );

        // the most aggressive fill ordering of the merged zones
        plan.priority = std::max( plan.priority, plan.zones[i]->GetAssignedPriority() );
    }

    plan.outline.Simplify();

    // One polygon, possibly with holes: two polygons mean the zones met in a point or inside a hole
    if( plan.outline.IsSelfIntersecting() || plan.outline.OutlineCount() > 1 )
        plan.error = _( "Zones have insufficient overlap for merging." );

    return plan;
}


void ApplyZoneMerge( BOARD_COMMIT& aCommit, const ZONE_MERGE_PLAN& aPlan )
{
    if( !aPlan.error.IsEmpty() || aPlan.zones.size() < 2 )
        return;

    ZONE* first = aPlan.zones.front();

    aCommit.Modify( first );
    *first->Outline() = aPlan.outline;
    first->SetAssignedPriority( aPlan.priority );

    for( size_t i = 1; i < aPlan.zones.size(); i++ )
        aCommit.Remove( aPlan.zones[i] );

    first->SetNeedRefill( true );
    first->SetLocalFlags( 1 );
    first->HatchBorder();
    first->CacheTriangulation();
}
