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

#include <sch_bus_migration.h>

#include <climits>

#include <advanced_config.h>
#include <connection_graph.h>
#include <sch_commit.h>
#include <sch_connection.h>
#include <sch_label.h>
#include <sch_sheet_path.h>
#include <schematic.h>


std::vector<BUS_MIGRATION_STATUS> SCH_BUS_MIGRATION::Find( SCHEMATIC& aSchematic )
{
    std::vector<BUS_MIGRATION_STATUS> items;

    // The subgraphs and their labels come from the connection graph, which the connectivity
    // engine does not keep up to date.
    if( ADVANCED_CFG::GetCfg().m_ConnectivityEngine )
        aSchematic.ConnectionGraph()->Recalculate( aSchematic.Hierarchy(), true );

    for( const CONNECTION_SUBGRAPH* subgraph : aSchematic.ConnectionGraph()->GetBusesNeedingMigration() )
    {
        BUS_MIGRATION_STATUS status;

        status.subgraph = subgraph;
        status.approved = false;

        std::vector<SCH_ITEM*> labels = subgraph->GetVectorBusLabels();
        wxASSERT( labels.size() > 1 );

        for( SCH_ITEM* label : labels )
            status.labels.push_back( static_cast<SCH_LABEL_BASE*>( label )->GetText() );

        status.possible_labels = ProposedLabels( aSchematic, status.labels );
        items.push_back( status );
    }

    return items;
}


std::vector<wxString> SCH_BUS_MIGRATION::ProposedLabels( SCHEMATIC& aSchematic,
                                                         const std::vector<wxString>& aLabelList )
{
    int lowest_start = INT_MAX;
    int highest_end = -1;
    int widest_bus = -1;

    SCH_CONNECTION conn( aSchematic.ConnectionGraph() );

    for( const wxString& label : aLabelList )
    {
        conn.ConfigureFromLabel( label );

        int start = conn.VectorStart();
        int end = conn.VectorEnd();

        if( start < lowest_start )
            lowest_start = start;

        if( end > highest_end )
            highest_end = end;

        if( end - start + 1 > widest_bus )
            widest_bus = end - start + 1;
    }

    std::vector<wxString> proposals;

    for( const wxString& label : aLabelList )
    {
        conn.ConfigureFromLabel( label );
        wxString proposal = conn.VectorPrefix();
        proposal << "[" << highest_end << ".." << lowest_start << "]";
        proposals.push_back( proposal );
    }

    return proposals;
}


void SCH_BUS_MIGRATION::Apply( BUS_MIGRATION_STATUS& aStatus, const wxString& aLabel, SCH_COMMIT* aCommit )
{
    aStatus.approved_label = aLabel;
    aStatus.approved = true;

    SCH_SCREEN* screen = aStatus.subgraph->GetSheet().LastScreen();

    for( SCH_ITEM* label : aStatus.subgraph->GetVectorBusLabels() )
    {
        if( aCommit )
            aCommit->Modify( label, screen );

        static_cast<SCH_LABEL_BASE*>( label )->SetText( aLabel );
        label->SetConnectivityDirty();
    }
}
