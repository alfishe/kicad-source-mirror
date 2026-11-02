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

#include <board_netlist_payload.h>

#include <board.h>
#include <footprint.h>
#include <netlist_reader/pcb_netlist.h>
#include <pad.h>
#include <pcb_field.h>
#include <richio.h>


std::string BoardBackAnnotationNetlist( BOARD* aBoard )
{
    NETLIST          netlist;
    STRING_FORMATTER sf;

    for( FOOTPRINT* footprint : aBoard->Footprints() )
    {
        if( footprint->GetAttributes() & FP_BOARD_ONLY )
            continue; // Don't add board-only footprints to the netlist

        COMPONENT* component = new COMPONENT( footprint->GetFPID(), footprint->GetReference(),
                                              footprint->GetValue(), footprint->GetPath(), {} );

        for( PAD* pad : footprint->Pads() )
        {
            const wxString& netname = pad->GetShortNetname();

            if( !netname.IsEmpty() )
            {
                component->AddNet( pad->GetNumber(), netname, pad->GetPinFunction(),
                                   pad->GetPinType() );
            }
        }

        nlohmann::ordered_map<wxString, wxString> fields;

        for( PCB_FIELD* field : footprint->GetFields() )
        {
            wxCHECK2( field, continue );

            fields[field->GetUntranslatedName()] = field->GetText();
        }

        component->SetFields( fields );

        // Add DNP and exclusion properties
        std::map<wxString, wxString> properties;

        if( footprint->GetAttributes() & FP_DNP )
            properties.emplace( "dnp", "" );

        if( footprint->GetAttributes() & FP_EXCLUDE_FROM_BOM )
            properties.emplace( "exclude_from_bom", "" );

        if( footprint->GetAttributes() & FP_EXCLUDE_FROM_SIM )
            properties.emplace( "exclude_from_sim", "" );

        if( footprint->GetAttributes() & FP_EXCLUDE_FROM_POS_FILES )
            properties.emplace( "exclude_from_pos_files", "" );

        component->SetProperties( properties );

        netlist.AddComponent( component );
    }

    netlist.Format( "pcb_netlist", &sf, 0, CTL_OMIT_FILTERS );
    return sf.GetString();
}
