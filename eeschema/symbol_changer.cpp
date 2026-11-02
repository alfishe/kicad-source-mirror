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
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search for one by writing to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include <symbol_changer.h>

#include <core/kicad_algo.h>
#include <lib_symbol.h>
#include <refdes_utils.h>
#include <sch_commit.h>
#include <sch_field.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <string_utils.h>
#include <wx/intl.h>


std::vector<SYMBOL_CHANGE_RESULT> SYMBOL_CHANGER::Apply( SCH_COMMIT* aCommit,
                                                         const std::map<SCH_SYMBOL*, SYMBOL_CHANGE_INFO>& aSymbols )
{
    std::vector<SYMBOL_CHANGE_RESULT>         results;
    std::map<SCH_SYMBOL*, SYMBOL_CHANGE_INFO> symbols = aSymbols;

    auto refuse = [&]( SCH_SYMBOL* aSymbol, const LIB_ID& aNewId, const wxString& aWhy )
    {
        results.push_back( { aSymbol, aSymbol->GetLibId(), aNewId, aWhy } );
    };

    // Drop the symbols without a valid library symbol link or with too few units for it
    for( auto it = symbols.begin(); it != symbols.end(); )
    {
        SCH_SYMBOL* symbol = it->first;

        if( !symbol || it->second.m_Instances.empty() )
        {
            it = symbols.erase( it );
            continue;
        }

        if( !it->second.m_LibId.IsValid() )
        {
            refuse( symbol, it->second.m_LibId, _( "symbol lib id not valid" ) );
            it = symbols.erase( it );
            continue;
        }

        LIB_SYMBOL* libSymbol = m_lookup( it->second.m_LibId );

        if( !libSymbol )
        {
            refuse( symbol, it->second.m_LibId, _( "symbol not found" ) );
            it = symbols.erase( it );
            continue;
        }

        if( libSymbol->Flatten()->GetUnitCount() < symbol->GetUnit() )
        {
            refuse( symbol, it->second.m_LibId, _( "new symbol has too few units" ) );
            it = symbols.erase( it );
            continue;
        }

        ++it;
    }

    // Removing the symbol needs to be done before the LIB_SYMBOL is changed to prevent stale
    // library symbols in the schematic file.
    for( const auto& [symbol, info] : symbols )
    {
        SCH_SCREEN* screen = info.m_Instances[0].LastScreen();

        wxCHECK2( screen, continue );

        screen->Remove( symbol );
        SCH_SYMBOL* symbol_copy = static_cast<SCH_SYMBOL*>( symbol->Clone() );
        aCommit->Modified( symbol, symbol_copy, screen );
    }

    const SYMBOL_CHANGE_OPTIONS& o = m_options;

    for( const auto& [symbol, info] : symbols )
    {
        const LIB_ID oldId = symbol->GetLibId();
        LIB_SYMBOL*  libSymbol = m_lookup( info.m_LibId );
        SCH_SCREEN*  screen = info.m_Instances[0].LastScreen();

        wxCHECK2( libSymbol && screen, continue );

        if( info.m_LibId != symbol->GetLibId() )
            symbol->SetLibId( info.m_LibId );

        std::unique_ptr<LIB_SYMBOL> flattenedSymbol = libSymbol->Flatten();

        const bool keepPinMaps = !o.m_ResetPinMapOverrides;

        PIN_MAP_SET                       savedPinMaps;
        std::vector<ASSOCIATED_FOOTPRINT> savedAssociations;

        if( keepPinMaps && symbol->GetLibSymbolRef() )
        {
            savedPinMaps = symbol->GetLibSymbolRef()->GetPinMaps();
            savedAssociations = symbol->GetLibSymbolRef()->GetAssociatedFootprints();
        }

        symbol->SetLibSymbol( flattenedSymbol.release() );

        if( keepPinMaps && symbol->GetLibSymbolRef() )
        {
            symbol->GetLibSymbolRef()->SetPinMaps( savedPinMaps );
            symbol->GetLibSymbolRef()->SetAssociatedFootprints( savedAssociations );
        }

        if( o.m_ResetAttributes )
        {
            // Fetch the attributes from the *flattened* library symbol.  They are not supported
            // in derived symbols.
            symbol->SetExcludedFromSim( symbol->GetLibSymbolRef()->GetExcludedFromSim() );
            symbol->SetExcludedFromBOM( symbol->GetLibSymbolRef()->GetExcludedFromBOM() );
            symbol->SetExcludedFromBoard( symbol->GetLibSymbolRef()->GetExcludedFromBoard() );
            symbol->SetExcludedFromPosFiles( symbol->GetLibSymbolRef()->GetExcludedFromPosFiles() );
        }

        if( o.m_ResetPinTextVisibility )
        {
            symbol->SetShowPinNames( symbol->GetLibSymbolRef()->GetShowPinNames() );
            symbol->SetShowPinNumbers( symbol->GetLibSymbolRef()->GetShowPinNumbers() );
        }

        for( int ii = (int) symbol->GetFields().size() - 1; ii >= 0; ii-- )
        {
            SCH_FIELD& field = symbol->GetFields()[ii];
            SCH_FIELD* libField = nullptr;
            bool       doUpdate = field.IsPrivate();

            // Mandatory fields always exist in the update set, but these names can be
            // translated, so use GetUntranslatedName().
            doUpdate |= alg::contains( o.m_UpdateFields, field.GetUntranslatedName() );

            if( !doUpdate )
                continue;

            if( field.IsMandatory() )
                libField = symbol->GetLibSymbolRef()->GetField( field.GetId() );
            else
                libField = symbol->GetLibSymbolRef()->GetField( field.GetName() );

            if( libField )
            {
                field.SetPrivate( libField->IsPrivate() );

                bool resetText = libField->GetText().IsEmpty() ? o.m_ResetEmptyFields : o.m_ResetFieldText;

                if( resetText )
                {
                    if( field.GetId() == FIELD_T::REFERENCE )
                    {
                        wxString prefix = UTIL::GetRefDesPrefix( libField->GetText() );

                        for( const SCH_SHEET_PATH& instance : info.m_Instances )
                        {
                            wxString ref = symbol->GetRef( &instance );
                            int      number = UTIL::GetRefDesNumber( ref );

                            if( number >= 0 )
                                ref.Printf( wxS( "%s%d" ), prefix, number );
                            else
                                ref = UTIL::GetRefDesUnannotated( prefix );

                            symbol->SetRef( &instance, ref );
                        }
                    }
                    else if( field.GetId() == FIELD_T::VALUE )
                    {
                        if( !symbol->IsPower() || o.m_ResetCustomPower )
                            symbol->SetValueFieldText( UnescapeString( libField->GetText() ) );
                    }
                    else
                    {
                        field.SetText( libField->GetText() );
                    }
                }

                if( o.m_ResetFieldVisibilities )
                    field.SetVisible( libField->IsVisible() );

                if( o.m_ResetFieldEffects )
                {
                    // Careful: the visible bit and position are also set by SetAttributes()
                    bool     visible = field.IsVisible();
                    VECTOR2I pos = field.GetPosition();

                    field.SetAttributes( *libField );

                    field.SetVisible( visible );
                    field.SetPosition( pos );
                    field.SetNameShown( libField->IsNameShown() );
                    field.SetCanAutoplace( libField->CanAutoplace() );
                }

                if( o.m_ResetFieldPositions )
                    field.SetTextPos( symbol->GetPosition() + libField->GetTextPos() );
            }
            else if( !field.IsMandatory() && o.m_RemoveExtraFields )
            {
                symbol->RemoveField( field.GetName() );
            }
        }

        std::vector<SCH_FIELD*> libFields;
        symbol->GetLibSymbolRef()->GetFields( libFields );

        for( SCH_FIELD* libField : libFields )
        {
            if( libField->IsMandatory() )
                continue;

            if( !alg::contains( o.m_UpdateFields, libField->GetUntranslatedName() ) )
                continue;

            if( !symbol->GetField( libField->GetName() ) )
            {
                SCH_FIELD* schField = symbol->AddField( SCH_FIELD( symbol, FIELD_T::USER, libField->GetName() ) );

                // SetAttributes() also covers text angle, size, italic and bold
                schField->SetAttributes( *libField );
                schField->SetVisible( libField->IsVisible() );
                schField->SetText( libField->GetText() );
                schField->SetTextPos( symbol->GetPosition() + libField->GetTextPos() );
                schField->SetPrivate( libField->IsPrivate() );
            }

            if( o.m_ResetFieldPositions && o.m_AutoplaceNewFields )
            {
                AUTOPLACE_ALGO fieldsAutoplaced = symbol->GetFieldsAutoplaced();

                if( fieldsAutoplaced == AUTOPLACE_AUTO || fieldsAutoplaced == AUTOPLACE_MANUAL )
                    symbol->AutoplaceFields( screen, fieldsAutoplaced );
            }
        }

        symbol->SetSchSymbolLibraryName( wxEmptyString );
        screen->Append( symbol );

        if( o.m_ResetFieldPositions )
        {
            AUTOPLACE_ALGO fieldsAutoplaced = symbol->GetFieldsAutoplaced();

            if( fieldsAutoplaced == AUTOPLACE_AUTO || fieldsAutoplaced == AUTOPLACE_MANUAL )
                symbol->AutoplaceFields( screen, fieldsAutoplaced );
        }

        // Clear alternate pins as required.
        for( const SCH_SHEET_PATH& instance : info.m_Instances )
        {
            for( SCH_PIN* pin : symbol->GetPins( &instance ) )
            {
                if( !pin->GetAlt().IsEmpty() )
                {
                    // There is a bug that allows the alternate pin name to be set to the default
                    // pin name which causes library symbol comparison errors.  Clear the alternate
                    // pin name in this case even if the reset option is not checked.  Also clear
                    // the alternate pin name if it no longer exists in the alternate pin map.
                    if( o.m_ResetAlternatePins || ( pin->GetAlt() == pin->GetBaseName() )
                        || ( pin->GetAlternates().count( pin->GetAlt() ) == 0 ) )
                    {
                        pin->SetAlt( wxEmptyString );
                    }
                }
            }
        }

        if( o.m_ResetPinMapOverrides )
        {
            for( const SCH_SHEET_PATH& instance : info.m_Instances )
                symbol->SetPinMapOverride( PIN_MAP_INSTANCE_OVERRIDE(), &instance );
        }

        results.push_back( { symbol, oldId, info.m_LibId, wxEmptyString } );
    }

    return results;
}
