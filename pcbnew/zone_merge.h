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

#ifndef ZONE_MERGE_H
#define ZONE_MERGE_H

#include <geometry/shape_poly_set.h>
#include <wx/string.h>

#include <utility>
#include <vector>

class BOARD;
class BOARD_COMMIT;
class ZONE;

enum class ZONE_MERGE_SKIP
{
    OTHER_NET,
    OTHER_KIND,      ///< a rule area with a copper zone
    OTHER_LAYERS,
    NO_OVERLAP
};

/// What Merge Zones would do with a list of zones: the first one is kept and takes the others' outlines
struct ZONE_MERGE_PLAN
{
    std::vector<ZONE*>                           zones;     ///< the zones merged, the kept one first
    std::vector<std::pair<ZONE*, ZONE_MERGE_SKIP>> skipped;
    SHAPE_POLY_SET                               outline;   ///< the union
    unsigned                                     priority = 0;
    wxString                                     error;     ///< why nothing can be merged
};

/**
 * Merge Zones, planned without changing the board: zones of the first zone's net, kind and layer
 * set that overlap it or a zone taken before; the union must be one polygon.
 */
ZONE_MERGE_PLAN PlanZoneMerge( BOARD* aBoard, const std::vector<ZONE*>& aZones );

/// The plan in \a aCommit (not pushed): the first zone takes the union and the highest priority, the
/// others are removed
void ApplyZoneMerge( BOARD_COMMIT& aCommit, const ZONE_MERGE_PLAN& aPlan );

#endif // ZONE_MERGE_H
