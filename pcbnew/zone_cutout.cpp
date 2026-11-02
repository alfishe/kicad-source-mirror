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

#include <zone_cutout.h>

#include <board_commit.h>
#include <geometry/shape_poly_set.h>
#include <zone.h>


std::vector<ZONE*> ZoneCutout( BOARD_COMMIT& aCommit, ZONE& aZone, const SHAPE_POLY_SET& aCutout )
{
    SHAPE_POLY_SET outline( *aZone.Outline() );
    SHAPE_POLY_SET cutout( aCutout );

    // Clipper2 cannot carry arcs through boolean operations when either operand has holes or the
    // clip side has outlines; arc-bearing outlines would come out with corrupt arc endpoints.
    outline.ClearArcs();
    cutout.ClearArcs();
    outline.BooleanSubtract( cutout );

    std::vector<ZONE*> zones;

    if( outline.OutlineCount() == 0 )
    {
        aCommit.Remove( &aZone );
        return zones;
    }

    // a zone has one main outline: one zone per outline left
    auto polygon = [&]( int aOutline )
    {
        SHAPE_POLY_SET* poly = new SHAPE_POLY_SET;
        poly->AddOutline( outline.Outline( aOutline ) );

        for( int hole = 0; hole < outline.HoleCount( aOutline ); hole++ )
            poly->AddHole( outline.CHole( aOutline, hole ) );

        return poly;
    };

    auto finish = []( ZONE* aResult )
    {
        aResult->SetLocalFlags( 1 );
        aResult->HatchBorder();
        aResult->UnFill();
    };

    for( int ii = 1; ii < outline.OutlineCount(); ii++ )
    {
        ZONE* zone = new ZONE( aZone );
        zone->ResetUuidDirect();
        zone->ClearSelected();
        zone->SetOutline( polygon( ii ) );
        finish( zone );
        zones.push_back( zone );
    }

    aCommit.Modify( &aZone );
    aZone.SetOutline( polygon( 0 ) );
    finish( &aZone );
    zones.insert( zones.begin(), &aZone );

    for( size_t ii = 1; ii < zones.size(); ii++ )
        aCommit.Add( zones[ii] );

    return zones;
}
