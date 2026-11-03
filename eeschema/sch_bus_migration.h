/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SCH_BUS_MIGRATION_H
#define SCH_BUS_MIGRATION_H

#include <wx/string.h>

#include <vector>

class CONNECTION_SUBGRAPH;
class SCH_COMMIT;
class SCHEMATIC;


/**
 * Before KiCad 6.0 several differently named vector bus labels on one bus subgraph made those
 * buses equivalent by vector index (PCA[0..15], ADR[0..7] and BUS[5..10]: PCA5, ADR5 and BUS5
 * connect).  That is no longer allowed: each such subgraph gets one name that spans every
 * attached bus.  DIALOG_MIGRATE_BUSES and the API drive this.
 */
struct BUS_MIGRATION_STATUS
{
    const CONNECTION_SUBGRAPH* subgraph;
    std::vector<wxString>      labels;
    std::vector<wxString>      possible_labels;
    wxString                   approved_label;
    bool                       approved;
};


namespace SCH_BUS_MIGRATION
{
/// The subgraphs to migrate, each with its proposed names (builds the connection graph; rebuild the
/// schematic's connectivity afterwards)
std::vector<BUS_MIGRATION_STATUS> Find( SCHEMATIC& aSchematic );

/// One name per label: its prefix with the vector range covering every label
std::vector<wxString> ProposedLabels( SCHEMATIC& aSchematic, const std::vector<wxString>& aLabelList );

/// Every vector bus label of aStatus's subgraph renamed aLabel (recorded in aCommit when given);
/// connectivity has to be rebuilt afterwards
void Apply( BUS_MIGRATION_STATUS& aStatus, const wxString& aLabel, SCH_COMMIT* aCommit );
}

#endif
