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

#include <board_reannotate.h>

#include <algorithm>

#include <footprint.h>
#include <pcb_field.h>
#include <refdes_utils.h>
#include <string_utils.h>  // StrNumCmp


namespace
{

struct SORT_CODES
{
    bool YFirst;
    bool DescendingFirst;
    bool DescendingSecond;
};

//
// This converts the index into a sort code. Note that Back sort code will have left and
// right swapped.
//
const int FrontDirectionsArray[] = {
    SORTYFIRST + ASCENDINGFIRST + ASCENDINGSECOND,   // "Top to bottom, left to right",  //  100
    SORTYFIRST + ASCENDINGFIRST + DESCENDINGSECOND,  // "Top to bottom, right to left",  //  101
    SORTYFIRST + DESCENDINGFIRST + ASCENDINGSECOND,  // "Back to Front, left to right",  //  110
    SORTYFIRST + DESCENDINGFIRST + DESCENDINGSECOND, // "Back to Front, right to left",  //  111
    SORTXFIRST + ASCENDINGFIRST + ASCENDINGSECOND,   // "Left to right, Front to Back",  //  000
    SORTXFIRST + ASCENDINGFIRST + DESCENDINGSECOND,  // "Left to right, Back to Front",  //  001
    SORTXFIRST + DESCENDINGFIRST + ASCENDINGSECOND,  // "Right to left, Front to Back",  //  010
    SORTXFIRST + DESCENDINGFIRST + DESCENDINGSECOND  // "Right to left, Back to Front",  //  011
};

//
// Back Left/Right is opposite because it is a mirror image (coordinates are from the top)
//
const int BackDirectionsArray[] = {
    SORTYFIRST + ASCENDINGFIRST + DESCENDINGSECOND,  // "Top to bottom, left to right",  //  101
    SORTYFIRST + ASCENDINGFIRST + ASCENDINGSECOND,   // "Top to bottom, right to left",  //  100
    SORTYFIRST + DESCENDINGFIRST + DESCENDINGSECOND, // "Bottom to top, left to right",  //  111
    SORTYFIRST + DESCENDINGFIRST + ASCENDINGSECOND,  // "Bottom to top, right to left",  //  110
    SORTXFIRST + DESCENDINGFIRST + ASCENDINGSECOND,  // "Left to right, top to bottom",  //  010
    SORTXFIRST + DESCENDINGFIRST + DESCENDINGSECOND, // "Left to right, bottom to top",  //  011
    SORTXFIRST + ASCENDINGFIRST + ASCENDINGSECOND,   // "Right to left, top to bottom",  //  000
    SORTXFIRST + ASCENDINGFIRST + DESCENDINGSECOND   // "Right to left, bottom to top",  //  001
};


SORT_CODES sortCodes( const int* aDirArray, int aCode )
{
    return { ( aDirArray[aCode] & SORTYFIRST ) != 0, ( aDirArray[aCode] & DESCENDINGFIRST ) != 0,
             ( aDirArray[aCode] & DESCENDINGSECOND ) != 0 };
}


/// @return true if the first coordinate should be before the second coordinate
bool footprintCompare( const SORT_CODES& aCodes, const REFDES_INFO& aA, const REFDES_INFO& aB )
{
    int X0 = aA.roundedx, X1 = aB.roundedx, Y0 = aA.roundedy, Y1 = aB.roundedy;

    if( aCodes.YFirst ) //If sorting by Y then X, swap X and Y
    {
        std::swap( X0, Y0 );
        std::swap( X1, Y1 );
    }

    // If descending, same compare just swap directions
    if( aCodes.DescendingFirst )
        std::swap( X0, X1 );

    if( aCodes.DescendingSecond )
        std::swap( Y0, Y1 );

    if( X0 < X1 )
        return true;    // yes, it's smaller
    else if( X0 > X1 )
        return false;   // no, it's not
    else if( Y0 < Y1 )
        return true;
    else
        return false;
}


/// @return true is A < B
bool changeArrayCompare( const REFDES_CHANGE& aA, const REFDES_CHANGE& aB )
{
    return ( StrNumCmp( aA.OldRefDesString, aB.OldRefDesString ) < 0 );
}

} // namespace


int BOARD_REANNOTATOR::RoundToGrid( int aCoord, int aGrid )
{
    if( 0 == aGrid )
        aGrid = MINGRID;

    int rounder;
    rounder = aCoord % aGrid;
    aCoord -= rounder;

    if( abs( rounder ) > ( aGrid / 2 ) )
        aCoord += ( aCoord < 0 ? -aGrid : aGrid );

    return ( aCoord );
}


REFDES_PREFIX_INFO* BOARD_REANNOTATOR::getOrBuildRefDesInfo( const wxString& aRefDesPrefix, int aStartRefDes )
{
    for( size_t i = 0; i < m_refDesPrefixInfos.size(); i++ ) // See if it is in the info array
    {
        if( m_refDesPrefixInfos[i].RefDesPrefix == aRefDesPrefix ) // Found it!
            return &m_refDesPrefixInfos[i];
    }

    // Wasn't in the info array so add it
    REFDES_PREFIX_INFO newtype;
    newtype.RefDesPrefix = aRefDesPrefix;
    newtype.LastUsedRefDes = std::max( aStartRefDes - 1, 0 );
    m_refDesPrefixInfos.push_back( newtype );

    return &m_refDesPrefixInfos.back();
}


bool BOARD_REANNOTATOR::Plan( const std::vector<FOOTPRINT*>& aFootprints )
{
    const REANNOTATE_OPTIONS::SCOPE scope = m_options.Scope;
    int                             errorcount = 0;
    size_t                          firstnum = 0;

    m_frontFootprints.clear();
    m_backFootprints.clear();
    m_badRefDes.clear();
    m_duplicates.clear();
    m_tooManyErrors = false;

    REFDES_INFO fpData;

    for( FOOTPRINT* footprint : aFootprints )
    {
        fpData.Uuid         = footprint->m_Uuid;
        fpData.RefDesString = footprint->GetReference();
        fpData.FPID         = footprint->GetFPID();
        fpData.x            = m_options.UseFootprintLocation ? footprint->GetPosition().x
                                                             : footprint->Reference().GetPosition().x;
        fpData.y            = m_options.UseFootprintLocation ? footprint->GetPosition().y
                                                             : footprint->Reference().GetPosition().y;
        fpData.roundedx     = RoundToGrid( fpData.x, m_options.SortGrid.x ); // Round to sort
        fpData.roundedy     = RoundToGrid( fpData.y, m_options.SortGrid.y );
        fpData.Front        = footprint->GetLayer() == F_Cu;
        fpData.Action       = UPDATE_REFDES; // Usually good

        if( fpData.RefDesString.IsEmpty() )
        {
            fpData.Action = EMPTY_REFDES;
        }
        else
        {
            firstnum = fpData.RefDesString.find_first_of( wxT( "0123456789" ) );

            if( std::string::npos == firstnum )
                fpData.Action = INVALID_REFDES;
        }

        // Get the type (R, C, etc)
        fpData.RefDesPrefix = fpData.RefDesString.substr( 0, firstnum );

        for( const wxString& excluded : m_options.Exclude )
        {
            // If exclusion ends in *, apply it to entire refdes
            if( excluded.EndsWith( '*' ) )
            {
                if( fpData.RefDesString.Matches( excluded ) )
                {
                    fpData.Action = EXCLUDE_REFDES;
                    break;
                }
            }
            else if( excluded == fpData.RefDesPrefix )
            {
                fpData.Action = EXCLUDE_REFDES;
                break;
            }
        }

        // Lock and scope only narrow the footprints to update: excluded, empty and invalid
        // references stay as they are
        if( fpData.Action == UPDATE_REFDES )
        {
            if( footprint->IsLocked() && m_options.ExcludeLocked )
                fpData.Action = EXCLUDE_REFDES;
            else if( scope == REANNOTATE_OPTIONS::SELECTION )
                fpData.Action = m_options.IsSelected && m_options.IsSelected( footprint ) ? UPDATE_REFDES
                                                                                          : EXCLUDE_REFDES;
            else if( scope == REANNOTATE_OPTIONS::FRONT )
                fpData.Action = fpData.Front ? UPDATE_REFDES : EXCLUDE_REFDES;
            else if( scope == REANNOTATE_OPTIONS::BACK )
                fpData.Action = fpData.Front ? EXCLUDE_REFDES : UPDATE_REFDES;
        }

        if( fpData.Front )
            m_frontFootprints.push_back( fpData );
        else
            m_backFootprints.push_back( fpData );
    }

    int sortCode = m_options.Direction;

    if( sortCode < 0 || sortCode >= (int) std::size( FrontDirectionsArray ) )
        sortCode = 0;

    // Sort the front footprints, then the back ones (mirrored left / right)
    const SORT_CODES front = sortCodes( FrontDirectionsArray, sortCode );
    const SORT_CODES back = sortCodes( BackDirectionsArray, sortCode );

    std::sort( m_frontFootprints.begin(), m_frontFootprints.end(),
               [&]( const REFDES_INFO& aA, const REFDES_INFO& aB )
               {
                   return footprintCompare( front, aA, aB );
               } );

    std::sort( m_backFootprints.begin(), m_backFootprints.end(),
               [&]( const REFDES_INFO& aA, const REFDES_INFO& aB )
               {
                   return footprintCompare( back, aA, aB );
               } );

    m_refDesPrefixInfos.clear();
    m_changeArray.clear();

    buildUnavailableRefsList();

    if( !m_frontFootprints.empty() )
    {
        buildChangeArray( m_frontFootprints, m_options.FrontStart, m_options.FrontPrefix,
                          m_options.RemoveFrontPrefix );
    }

    if( !m_backFootprints.empty() )
    {
        buildChangeArray( m_backFootprints, m_options.BackStart, m_options.BackPrefix,
                          m_options.RemoveBackPrefix );
    }

    if( !m_changeArray.empty() )
        std::sort( m_changeArray.begin(), m_changeArray.end(), changeArrayCompare );

    size_t changearraysize = m_changeArray.size();

    for( size_t i = 0; i < changearraysize; i++ ) // Scan through for duplicates if update or skip
    {
        if( m_changeArray[i].Action != EMPTY_REFDES && m_changeArray[i].Action != INVALID_REFDES )
        {
            for( size_t j = i + 1; j < changearraysize; j++ )
            {
                if( m_changeArray[i].NewRefDes == m_changeArray[j].NewRefDes )
                {
                    m_duplicates.push_back( m_changeArray[j].NewRefDes );

                    if( errorcount++ > MAXERROR )
                    {
                        m_tooManyErrors = true;
                        break;
                    }
                }
            }
        }

        if( errorcount > MAXERROR )
            break;
    }

    return ( errorcount == 0 );
}


void BOARD_REANNOTATOR::buildUnavailableRefsList()
{
    std::vector<REFDES_INFO> excludedFootprints;

    for( const REFDES_INFO& fpData : m_frontFootprints )
    {
        if( fpData.Action == EXCLUDE_REFDES )
            excludedFootprints.push_back( fpData );
    }

    for( const REFDES_INFO& fpData : m_backFootprints )
    {
        if( fpData.Action == EXCLUDE_REFDES )
            excludedFootprints.push_back( fpData );
    }

    for( const REFDES_INFO& fpData : excludedFootprints )
    {
        if( fpData.Action == EXCLUDE_REFDES )
        {
            REFDES_PREFIX_INFO* refDesInfo = getOrBuildRefDesInfo( fpData.RefDesPrefix );
            refDesInfo->UnavailableRefs.insert( UTIL::GetRefDesNumber( fpData.RefDesString ) );
        }
    }
}


void BOARD_REANNOTATOR::buildChangeArray( std::vector<REFDES_INFO>& aFootprints, unsigned int aStartRefDes,
                                          const wxString& aPrefix, bool aRemovePrefix )
{
    size_t   prefixsize = aPrefix.size();

    bool haveprefix = ( 0 != prefixsize );         // Do I have a prefix?
    bool addprefix  = haveprefix & !aRemovePrefix; // Yes- and I'm not removing it
    aRemovePrefix &= haveprefix;                   // Only remove if I have a prefix

    bool prefixpresent; // Prefix found

    if( aStartRefDes != 0 ) // Initialize the change array if present
    {
        for( REFDES_PREFIX_INFO& prefixInfo : m_refDesPrefixInfos )
            prefixInfo.LastUsedRefDes = aStartRefDes - 1;
    }

    for( REFDES_INFO fpData : aFootprints )
    {
        REFDES_CHANGE change;

        change.Uuid            = fpData.Uuid;
        change.Action          = fpData.Action;
        change.OldRefDesString = fpData.RefDesString;
        change.NewRefDes       = fpData.RefDesString;
        change.Front           = fpData.Front;

        if( fpData.RefDesString.IsEmpty() )
            fpData.Action = EMPTY_REFDES;

        if( ( change.Action == EMPTY_REFDES ) || ( change.Action == INVALID_REFDES ) )
        {
            m_changeArray.push_back( change );
            m_badRefDes.push_back( fpData );
            continue;
        }

        if( change.Action == UPDATE_REFDES )
        {
            prefixpresent = ( fpData.RefDesPrefix.find( aPrefix ) == 0 );

            if( addprefix && !prefixpresent )
                fpData.RefDesPrefix.insert( 0, aPrefix ); // Add prefix once only

            if( aRemovePrefix && prefixpresent ) // If there is a prefix remove it
                fpData.RefDesPrefix.erase( 0, prefixsize );

            REFDES_PREFIX_INFO* refDesInfo = getOrBuildRefDesInfo( fpData.RefDesPrefix, aStartRefDes );
            unsigned int  newRefDesNumber = refDesInfo->LastUsedRefDes + 1;

            while( refDesInfo->UnavailableRefs.count( newRefDesNumber ) )
                newRefDesNumber++;

            change.NewRefDes = refDesInfo->RefDesPrefix + std::to_string( newRefDesNumber );
            refDesInfo->LastUsedRefDes = newRefDesNumber;
        }

        m_changeArray.push_back( change );
    }
}


const REFDES_CHANGE* BOARD_REANNOTATOR::GetNewRefDes( const FOOTPRINT* aFootprint ) const
{
    for( const REFDES_CHANGE& change : m_changeArray )
    {
        if( aFootprint->m_Uuid == change.Uuid )
            return &change;
    }

    return nullptr;
}
