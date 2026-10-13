/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.TXT for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
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

/**
 * A native schematic written by an older version can carry an implied junction that was
 * never persisted; the fixup must restore it when the file is opened through the
 * headless/CLI load path, not only through the legacy GUI loader.  issue18346.kicad_sch
 * (file version 20240602, older than current) has a label placed where two wires cross;
 * the label splits both wires, leaving four wire ends at that point with no junction dot,
 * which SCH_SCREEN reports as a needed junction until FixupJunctionsAfterImport runs.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <eeschema_test_utils.h>

#include <deque>
#include <set>
#include <eeschema_helpers.h>
#include <locale_io.h>
#include <sch_screen.h>
#include <schematic.h>


BOOST_AUTO_TEST_CASE( Issue18346JunctionFixupOnLoad )
{
    LOCALE_IO dummy;

    wxString schPath = wxString::FromUTF8( KI_TEST::GetEeschemaTestDataDir() ) +
                       wxS( "issue18346.kicad_sch" );

    std::unique_ptr<SCHEMATIC> sch( EESCHEMA_HELPERS::LoadSchematic( schPath, true, true ) );
    BOOST_REQUIRE( sch != nullptr );

    SCH_SCREENS screens( sch->Root() );
    int         needed = 0;

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        std::deque<EDA_ITEM*> allItems;

        for( SCH_ITEM* item : screen->Items() )
            allItems.push_back( item );

        needed += static_cast<int>( screen->GetNeededJunctions( allItems ).size() );
    }

    BOOST_CHECK_MESSAGE( needed == 0,
                         "Normal load path left " << needed
                         << " junction(s) missing; FixupJunctionsAfterImport did not run" );
}


// The junctions and wire segments the repair adds have the same ids on every load
BOOST_AUTO_TEST_CASE( JunctionFixupOnLoadGivesStableIds )
{
    LOCALE_IO dummy;

    wxString schPath = wxString::FromUTF8( KI_TEST::GetEeschemaTestDataDir() ) +
                       wxS( "issue18346.kicad_sch" );

    auto itemIds = [&]()
    {
        std::unique_ptr<SCHEMATIC> sch( EESCHEMA_HELPERS::LoadSchematic( schPath, true, true ) );
        BOOST_REQUIRE( sch != nullptr );

        std::set<KIID> ids;
        SCH_SCREENS    screens( sch->Root() );

        for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
        {
            for( SCH_ITEM* item : screen->Items() )
                ids.insert( item->m_Uuid );
        }

        return ids;
    };

    const std::set<KIID> first = itemIds();
    const std::set<KIID> second = itemIds();

    BOOST_REQUIRE( !first.empty() );
    BOOST_CHECK( first == second );
}
