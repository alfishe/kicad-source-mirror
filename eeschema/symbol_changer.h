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

#ifndef SYMBOL_CHANGER_H
#define SYMBOL_CHANGER_H

#include <functional>
#include <map>
#include <set>
#include <vector>

#include <lib_id.h>
#include <sch_sheet_path.h>

class LIB_SYMBOL;
class SCH_COMMIT;
class SCH_SYMBOL;


struct SYMBOL_CHANGE_INFO
{
    std::vector<SCH_SHEET_PATH> m_Instances;
    LIB_ID                      m_LibId;
};


/// What Update / Change Symbols from Library carries over from the library symbol
struct SYMBOL_CHANGE_OPTIONS
{
    /// Untranslated names of the fields to update (mandatory ones included); private fields
    /// always update
    std::set<wxString> m_UpdateFields;

    bool m_RemoveExtraFields = false;
    bool m_ResetEmptyFields = false;
    bool m_ResetFieldText = true;
    bool m_ResetFieldVisibilities = false;
    bool m_ResetFieldEffects = false;
    bool m_ResetFieldPositions = false;
    bool m_ResetAttributes = false;
    bool m_ResetPinTextVisibility = false;
    bool m_ResetAlternatePins = false;
    bool m_ResetCustomPower = false;
    bool m_ResetPinMapOverrides = false;

    /// Autoplace fields of new library fields when positions reset (the editor's autoplace setting)
    bool m_AutoplaceNewFields = false;
};


/// One symbol's outcome: changed, or refused with the reason
struct SYMBOL_CHANGE_RESULT
{
    SCH_SYMBOL* m_Symbol = nullptr;
    LIB_ID      m_OldLibId;
    LIB_ID      m_NewLibId;
    wxString    m_Error;     ///< empty when changed
};


/**
 * The work of Update / Change Symbols from Library (DIALOG_CHANGE_SYMBOLS) without the dialog:
 * each symbol gets its (new) library symbol, flattened, and the fields, attributes, pin text
 * visibility, alternate pins and pin map overrides the options ask for.
 */
class SYMBOL_CHANGER
{
public:
    using LIB_LOOKUP = std::function<LIB_SYMBOL*( const LIB_ID& )>;

    SYMBOL_CHANGER( LIB_LOOKUP aLookup, const SYMBOL_CHANGE_OPTIONS& aOptions ) :
            m_lookup( std::move( aLookup ) ),
            m_options( aOptions )
    {
    }

    /// Symbols with an invalid lib id, a library symbol not found or too few units are refused;
    /// the rest are changed in aCommit.  One result per symbol, refused ones first.
    std::vector<SYMBOL_CHANGE_RESULT> Apply( SCH_COMMIT* aCommit,
                                             const std::map<SCH_SYMBOL*, SYMBOL_CHANGE_INFO>& aSymbols );

private:
    LIB_LOOKUP            m_lookup;
    SYMBOL_CHANGE_OPTIONS m_options;
};

#endif // SYMBOL_CHANGER_H
