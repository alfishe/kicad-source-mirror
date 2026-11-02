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

#ifndef ZONE_CUTOUT_H
#define ZONE_CUTOUT_H

#include <vector>

class BOARD_COMMIT;
class SHAPE_POLY_SET;
class ZONE;

/**
 * Add Zone Cutout: subtract \a aCutout from the outline of \a aZone in \a aCommit (not pushed).
 * The zone keeps the first outline left with its holes; every further outline becomes a copy of
 * the zone with its own uuid; nothing left removes the zone.  The zones are unfilled.
 *
 * @return the zones that make up the result, \a aZone first; empty when it was removed.
 */
std::vector<ZONE*> ZoneCutout( BOARD_COMMIT& aCommit, ZONE& aZone, const SHAPE_POLY_SET& aCutout );

#endif // ZONE_CUTOUT_H
