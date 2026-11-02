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

#include "board_netlist_export.h"

#include <board.h>
#include <footprint.h>
#include <netlist_reader/pcb_netlist.h>
#include <pad.h>
#include <pcb_field.h>
#include <richio.h>
#include <wx/filename.h>
#include <wx/intl.h>


wxString ExportBoardNetlist( const BOARD& aBoard, const wxString& aPath, int aCtl )
{
    wxFileName fn( aPath );

    if( !fn.IsDirWritable() )
        return wxString::Format( _( "Insufficient permissions to folder '%s'." ), fn.GetPath() );

    NETLIST netlist;

    for( const FOOTPRINT* footprint : aBoard.Footprints() )
    {
        COMPONENT* component = new COMPONENT( footprint->GetFPID(), footprint->GetReference(),
                                              footprint->GetValue(), footprint->GetPath(),
                                              { footprint->m_Uuid } );

        for( const PAD* pad : footprint->Pads() )
        {
            const wxString& netname = pad->GetShortNetname();

            if( !netname.IsEmpty() )
                component->AddNet( pad->GetNumber(), netname, pad->GetPinFunction(), pad->GetPinType() );
        }

        nlohmann::ordered_map<wxString, wxString> fields;

        for( PCB_FIELD* field : footprint->GetFields() )
        {
            wxCHECK2( field, continue );

            fields[field->GetUntranslatedName()] = field->GetText();
        }

        component->SetFields( fields );

        netlist.AddComponent( component );
    }

    try
    {
        FILE_OUTPUTFORMATTER formatter( fn.GetFullPath() );

        netlist.Format( "pcb_netlist", &formatter, 0, aCtl );
        formatter.Finish();
    }
    catch( const IO_ERROR& ioe )
    {
        return wxString::Format( _( "Failed to export netlist to '%s': %s" ), fn.GetFullPath(), ioe.What() );
    }

    return wxEmptyString;
}
