/*
 * This program source code file is part of KiCad, a free EDA CAD application.
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

// Concurrent reads of a loaded schematic from reader threads

#include <qa_utils/wx_utils/unit_test_utils.h>
#include "eeschema_test_utils.h"

#include <model_thread.h>
#include <sch_junction.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <wildcards_and_files_ext.h>

#include <atomic>
#include <functional>
#include <thread>
#include <vector>


namespace
{

constexpr int THREADS = 4;


void runReaders( const std::function<void()>& aFn )
{
    std::vector<std::thread> threads;

    for( int i = 0; i < THREADS; ++i )
    {
        threads.emplace_back(
                [&]()
                {
                    MODEL_THREAD::SCOPE reader( MODEL_THREAD::KIND::READER );
                    aFn();
                } );
    }

    for( std::thread& t : threads )
        t.join();
}

} // namespace


class EESCHEMA_THREADING_FIXTURE : public KI_TEST::SCHEMATIC_TEST_FIXTURE
{
public:
    EESCHEMA_THREADING_FIXTURE()
    {
        wxFileName fn( KI_TEST::GetEeschemaTestDataDir(), wxS( "api_kitchen_sink" ),
                       FILEEXT::KiCadSchematicFileExtension );
        LoadSchematic( fn.GetFullPath() );
    }

    std::vector<SCH_SHEET_PATH> Sheets() const
    {
        SCH_SHEET_LIST list = m_schematic->Hierarchy();
        return std::vector<SCH_SHEET_PATH>( list.begin(), list.end() );
    }
};


BOOST_FIXTURE_TEST_SUITE( EeschemaThreading, EESCHEMA_THREADING_FIXTURE )


BOOST_AUTO_TEST_CASE( ConnectivityItemIndexBuiltOnce )
{
    std::vector<SCH_SHEET_PATH> sheets = Sheets();
    BOOST_REQUIRE( !sheets.empty() );

    std::atomic<int> misses{ 0 };
    std::atomic<int> lookups{ 0 };

    runReaders(
            [&]()
            {
                for( const SCH_SHEET_PATH& path : sheets )
                {
                    const SCH_SCREEN* screen = path.LastScreen();

                    for( SCH_ITEM* item : screen->Items() )
                    {
                        ++lookups;

                        if( screen->GetConnectivityItem( item->m_Uuid ) != item )
                            ++misses;
                    }
                }
            } );

    BOOST_CHECK_GT( lookups.load(), 0 );
    BOOST_CHECK_EQUAL( misses.load(), 0 );
}


BOOST_AUTO_TEST_CASE( PinLayoutCacheFilledConcurrently )
{
    std::vector<SCH_SHEET_PATH> sheets = Sheets();
    std::vector<SCH_PIN*>       pins;

    for( const SCH_SHEET_PATH& path : sheets )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( item )->GetPins( &path ) )
                pins.push_back( pin );
        }
    }

    BOOST_REQUIRE( !pins.empty() );

    std::vector<std::vector<BOX2I>> seen( THREADS );
    std::atomic<int>                slot{ 0 };

    runReaders(
            [&]()
            {
                std::vector<BOX2I>& mine = seen[slot++];

                for( SCH_PIN* pin : pins )
                    mine.push_back( pin->GetBoundingBox( false, true, true ) );
            } );

    for( size_t i = 0; i < pins.size(); ++i )
    {
        BOX2I expected = pins[i]->GetBoundingBox( false, true, true );

        for( const std::vector<BOX2I>& mine : seen )
            BOOST_CHECK( mine[i] == expected );
    }
}


BOOST_AUTO_TEST_CASE( ResolvedColorsAgreeAcrossReaders )
{
    std::vector<SCH_ITEM*> items;

    for( const SCH_SHEET_PATH& path : Sheets() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items() )
        {
            if( item->Type() == SCH_LINE_T || item->Type() == SCH_JUNCTION_T
                || dynamic_cast<SCH_LABEL_BASE*>( item ) )
            {
                items.push_back( item );
            }
        }
    }

    BOOST_REQUIRE( !items.empty() );

    auto colorOf =
            []( const SCH_ITEM* aItem ) -> KIGFX::COLOR4D
            {
                if( const SCH_LINE* line = dynamic_cast<const SCH_LINE*>( aItem ) )
                    return line->GetLineColor();

                if( const SCH_JUNCTION* junction = dynamic_cast<const SCH_JUNCTION*>( aItem ) )
                    return junction->GetJunctionColor();

                return static_cast<const SCH_LABEL_BASE*>( aItem )->GetLabelColor();
            };

    std::vector<KIGFX::COLOR4D> expected;

    for( SCH_ITEM* item : items )
        expected.push_back( colorOf( item ) );

    std::atomic<int> mismatches{ 0 };

    runReaders(
            [&]()
            {
                for( size_t i = 0; i < items.size(); ++i )
                {
                    if( colorOf( items[i] ) != expected[i] )
                        ++mismatches;
                }
            } );

    BOOST_CHECK_EQUAL( mismatches.load(), 0 );
}


BOOST_AUTO_TEST_SUITE_END()
