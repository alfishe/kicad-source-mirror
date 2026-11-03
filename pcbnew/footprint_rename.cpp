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

#include <footprint_rename.h>

#include <footprint.h>
#include <lib_id.h>
#include <wx/intl.h>


FOOTPRINT_RENAME_CHECK CheckFootprintRename( const wxString& aOldName, const wxString& aNewName,
                                             const std::function<bool( const wxString& )>& aExists )
{
    if( aNewName.IsEmpty() )
        return FOOTPRINT_RENAME_CHECK::EMPTY;

    if( aNewName == aOldName )
        return FOOTPRINT_RENAME_CHECK::UNCHANGED;

    if( aExists && aExists( aNewName ) )
        return FOOTPRINT_RENAME_CHECK::EXISTS;

    return FOOTPRINT_RENAME_CHECK::OK;
}


wxString FootprintRenameMessage( FOOTPRINT_RENAME_CHECK aCheck, const wxString& aNewName,
                                 const wxString& aLibrary )
{
    switch( aCheck )
    {
    case FOOTPRINT_RENAME_CHECK::EMPTY:
        return _( "Footprint must have a name." );

    case FOOTPRINT_RENAME_CHECK::EXISTS:
        return wxString::Format( _( "Footprint '%s' already exists in library '%s'." ), aNewName, aLibrary );

    default:
        return wxEmptyString;
    }
}


void ApplyFootprintRename( FOOTPRINT& aFootprint, const wxString& aOldName, const wxString& aNewName )
{
    LIB_ID id = aFootprint.GetFPID();
    id.SetLibItemName( aNewName );
    aFootprint.SetFPID( id );

    if( aFootprint.GetValue() == aOldName )
        aFootprint.SetValue( aNewName );
}
