/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2020 Brian Piccioni brian@documenteddesigns.com
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 * @author Brian Piccioni <brian@documenteddesigns.com>
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

#pragma once

#include <functional>
#include <set>
#include <vector>

#include <kiid.h>
#include <lib_id.h>
#include <math/vector2d.h>
#include <wx/string.h>

class FOOTPRINT;

#define SORTXFIRST 0b000       // Sort on X
#define SORTYFIRST 0b100       // Sort on Y
#define ASCENDINGFIRST 0b000   // Sort low to high
#define DESCENDINGFIRST 0b010  // Sort high to low
#define ASCENDINGSECOND 0b000  // Sort low to high
#define DESCENDINGSECOND 0b001 // Sort high to low

#define MINGRID 1000
#define MAXERROR 10
#define VALIDPREFIX "_-+=/\\"  // Prefixs can be alpha or these symbols

enum ACTION_CODE
{
    UPDATE_REFDES,
    EMPTY_REFDES,
    INVALID_REFDES,
    EXCLUDE_REFDES
};

struct REFDES_CHANGE
{
    KIID         Uuid;
    wxString     NewRefDes;         // The new reference designation (F_U21)
    wxString     OldRefDesString;   // What the old refdes preamble + number was
    bool         Front;             // True if on the front of the board
    ACTION_CODE  Action;            // Used to skip (if #, etc)
};

struct REFDES_INFO
{
    KIID        Uuid;
    bool        Front;              // True if on the front of the board
    wxString    RefDesString;       // What its refdes is R1, C2
    wxString    RefDesPrefix;       // ie R, C, etc
    int         x, y;               // The coordinates
    int         roundedx, roundedy; // The coordinates after rounding.
    ACTION_CODE Action;             // Used to skip (if #, etc)
    LIB_ID      FPID;
};

struct REFDES_PREFIX_INFO
{
    wxString               RefDesPrefix;
    unsigned int           LastUsedRefDes;
    std::set<unsigned int> UnavailableRefs;
};


/**
 * Geographical reannotation of a board's footprints, as Tools > Geographical Reannotate: the
 * footprints sorted along a direction on a grid, front and back apart, and numbered per prefix.
 */
struct REANNOTATE_OPTIONS
{
    enum SCOPE
    {
        ALL,
        FRONT,
        BACK,
        SELECTION
    };

    SCOPE    Scope = ALL;
    int      Direction = 0;         ///< the dialog's sort buttons: down-right, right-down, down-left,
                                    ///< left-down, up-right, right-up, up-left, left-up
    bool     UseFootprintLocation = true;   ///< else the reference text's location
    bool     ExcludeLocked = false;
    VECTOR2I SortGrid;              ///< IU; 0 uses MINGRID
    int      FrontStart = 1;
    int      BackStart = 0;         ///< 0: continue after the front
    wxString FrontPrefix;
    wxString BackPrefix;
    bool     RemoveFrontPrefix = false;
    bool     RemoveBackPrefix = false;
    std::vector<wxString> Exclude;  ///< prefixes, or references with a trailing *

    std::function<bool( const FOOTPRINT* )> IsSelected;
};


class BOARD_REANNOTATOR
{
public:
    explicit BOARD_REANNOTATOR( const REANNOTATE_OPTIONS& aOptions ) : m_options( aOptions ) {}

    /**
     * Build the footprint lists, sort them, filter for excludes, then build the change list.
     * @return false if the new references repeat (see Duplicates()).
     */
    bool Plan( const std::vector<FOOTPRINT*>& aFootprints );

    /// @return the change for this footprint, nullptr if it was not planned.
    const REFDES_CHANGE* GetNewRefDes( const FOOTPRINT* aFootprint ) const;

    static int RoundToGrid( int aCoord, int aGrid );

    const std::vector<REFDES_CHANGE>&      Changes() const { return m_changeArray; }
    const std::vector<REFDES_INFO>&        FrontFootprints() const { return m_frontFootprints; }
    const std::vector<REFDES_INFO>&        BackFootprints() const { return m_backFootprints; }
    const std::vector<REFDES_INFO>&        BadRefDes() const { return m_badRefDes; }
    const std::vector<REFDES_PREFIX_INFO>& PrefixInfos() const { return m_refDesPrefixInfos; }
    const std::vector<wxString>&           Duplicates() const { return m_duplicates; }

    /// True when Plan() stopped counting duplicates after MAXERROR.
    bool TooManyErrors() const { return m_tooManyErrors; }

private:
    void buildUnavailableRefsList();

    void buildChangeArray( std::vector<REFDES_INFO>& aFootprints, unsigned int aStartRefDes,
                           const wxString& aPrefix, bool aRemovePrefix );

    REFDES_PREFIX_INFO* getOrBuildRefDesInfo( const wxString& aRefDesPrefix, int aStartRefDes = 1 );

    REANNOTATE_OPTIONS              m_options;
    std::vector<REFDES_CHANGE>      m_changeArray;
    std::vector<REFDES_INFO>        m_frontFootprints;
    std::vector<REFDES_INFO>        m_backFootprints;
    std::vector<REFDES_INFO>        m_badRefDes;
    std::vector<REFDES_PREFIX_INFO> m_refDesPrefixInfos;
    std::vector<wxString>           m_duplicates;
    bool                            m_tooManyErrors = false;
};
