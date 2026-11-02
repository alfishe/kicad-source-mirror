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

#include <libraries/library_table_edit.h>

#include <ki_exception.h>
#include <lib_id.h>
#include <libraries/library_manager.h>
#include <pgm_base.h>

#include <set>


wxString VerifyLibraryTable( LIBRARY_TABLE& aTable )
{
    const bool          visibility = aTable.Type() != LIBRARY_TABLE_TYPE::FOOTPRINT;
    std::set<wxString>  nicknames;
    auto&               rows = aTable.Rows();

    for( auto it = rows.begin(); it != rows.end(); )
    {
        wxString nick = it->Nickname();
        wxString uri = it->URI();

        nick.Trim( false ).Trim();
        uri.Trim( false ).Trim();

        if( uri.IsEmpty() )
        {
            it = rows.erase( it );
            continue;
        }

        if( nick.IsEmpty() )
            return _( "Library must have a nickname." );

        if( unsigned illegal = LIB_ID::FindIllegalLibraryNameChar( nick ) )
            return wxString::Format( _( "Illegal character '%c' in nickname '%s'." ), illegal, nick );

        if( !nicknames.insert( nick ).second )
            return wxString::Format( _( "Multiple libraries cannot share the same nickname ('%s')." ), nick );

        it->SetNickname( nick );
        it->SetURI( uri );

        if( !visibility )
            it->SetHidden( false );

        ++it;
    }

    return wxEmptyString;
}


bool EditLibraryTable( LIBRARY_TABLE_TYPE aType, LIBRARY_TABLE_SCOPE aScope,
                       const std::function<wxString( LIBRARY_TABLE& )>& aEdit, wxString& aError )
{
    LIBRARY_MANAGER&              manager = Pgm().GetLibraryManager();
    std::optional<LIBRARY_TABLE*> table;

    try
    {
        table = manager.Table( aType, aScope );
    }
    catch( const IO_ERROR& ioe )
    {
        aError = ioe.What();
        return false;
    }

    if( !table || !*table )
    {
        aError = _( "No library table to edit." );
        return false;
    }

    LIBRARY_TABLE edited = **table;

    aError = aEdit( edited );

    if( aError.IsEmpty() )
        aError = VerifyLibraryTable( edited );

    if( !aError.IsEmpty() )
        return false;

    if( edited == **table )
        return true;

    **table = edited;
    bool saved = true;

    ( *table )->Save().map_error(
            [&]( const LIBRARY_ERROR& aLibError )
            {
                aError = _( "Error saving library table:\n\n" ) + aLibError.message;
                saved = false;
            } );

    manager.ReloadTables( aScope, { aType } );
    return saved;
}
