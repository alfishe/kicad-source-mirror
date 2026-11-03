/**
 * @file dialog_symbol_remap.h
 */

/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2017 Wayne Stambaugh <stambaughw@verizon.net>
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <dialog_symbol_remap_base.h>


#ifndef _DIALOG_SYMBOL_REMAP_H_
#define _DIALOG_SYMBOL_REMAP_H_


class REPORTER;
class SYMBOL_REMAPPER;


class DIALOG_SYMBOL_REMAP : public DIALOG_SYMBOL_REMAP_BASE
{
public:
    DIALOG_SYMBOL_REMAP( SCH_EDIT_FRAME* aParent );

    void OnRemapSymbols( wxCommandEvent& aEvent ) override;

protected:
    void OnUpdateUIRemapButton( wxUpdateUIEvent& aEvent ) override;

private:
    /// The backup before the remap; on errors asks whether to continue
    bool backupProject( SYMBOL_REMAPPER& aRemapper, REPORTER& aReporter );

    bool m_remapped;

    SCH_EDIT_FRAME* m_frame;
};

#endif  // _DIALOG_SYMBOL_REMAP_H_
