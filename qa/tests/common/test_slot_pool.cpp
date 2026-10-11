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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <boost/test/unit_test.hpp>

#include <core/slot_pool.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <set>
#include <thread>
#include <vector>


BOOST_AUTO_TEST_SUITE( SlotPool )


BOOST_AUTO_TEST_CASE( SlotsAreAlignedDistinctAndCounted )
{
    KISLOT_POOL pool( 24, 1 );
    BOOST_CHECK_EQUAL( pool.SlotSize(), 32u );

    std::set<void*> seen;

    for( int i = 0; i < 1000; ++i )
    {
        void* slot = pool.Alloc();
        BOOST_REQUIRE( slot );
        BOOST_CHECK_EQUAL( reinterpret_cast<uintptr_t>( slot ) % 16, 0u );
        BOOST_CHECK( seen.insert( slot ).second );
    }

    BOOST_CHECK_EQUAL( pool.GetStats().live, 1000u );

    for( void* slot : seen )
        KISLOT_POOL::Free( slot );

    KISLOT_POOL::STATS stats = pool.GetStats();
    BOOST_CHECK_EQUAL( stats.live, 0u );
    BOOST_CHECK_EQUAL( stats.allocs, 1000u );
    BOOST_CHECK_EQUAL( stats.frees, 1000u );
}


// Emptied chunks go back, except one kept for the next allocations
BOOST_AUTO_TEST_CASE( EmptyChunksAreReleased )
{
    KISLOT_POOL       pool( 64, 1 );
    std::vector<void*> slots;

    for( size_t i = 0; i < 5 * ( KISLOT_POOL::CHUNK_SIZE / 64 ); ++i )
        slots.push_back( pool.Alloc() );

    BOOST_CHECK_GE( pool.GetStats().chunks, 5u );

    std::shuffle( slots.begin(), slots.end(), std::mt19937( 7 ) );

    for( void* slot : slots )
        KISLOT_POOL::Free( slot );

    BOOST_CHECK_LE( pool.GetStats().chunks, 1u );
    BOOST_CHECK_GE( pool.GetStats().peakChunks, 5u );
}


// Freed slots are reused before new chunks are taken
BOOST_AUTO_TEST_CASE( FreedSlotsAreReused )
{
    KISLOT_POOL pool( 32, 1 );
    void*       a = pool.Alloc();
    KISLOT_POOL::Free( a );
    void* b = pool.Alloc();

    BOOST_CHECK_EQUAL( a, b );
    BOOST_CHECK_EQUAL( pool.GetStats().chunks, 1u );
    KISLOT_POOL::Free( b );
}


// Objects may die on another thread than the one that made them
BOOST_AUTO_TEST_CASE( CrossThreadFrees )
{
    KISLOT_POOL               pool( 48, 1 );
    std::vector<std::thread>  threads;
    std::vector<void*>        made[4];

    for( int t = 0; t < 4; ++t )
    {
        threads.emplace_back(
                [&, t]()
                {
                    for( int i = 0; i < 20000; ++i )
                        made[t].push_back( pool.Alloc() );
                } );
    }

    for( std::thread& thread : threads )
        thread.join();

    threads.clear();

    for( int t = 0; t < 4; ++t )
    {
        threads.emplace_back(
                [&, t]()
                {
                    for( void* slot : made[( t + 1 ) % 4] )
                        KISLOT_POOL::Free( slot );
                } );
    }

    for( std::thread& thread : threads )
        thread.join();

    BOOST_CHECK_EQUAL( pool.GetStats().live, 0u );
    BOOST_CHECK_LE( pool.GetStats().chunks, 1u );
}


BOOST_AUTO_TEST_SUITE_END()
