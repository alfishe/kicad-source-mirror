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

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <advanced_config.h>
#include <board.h>
#include <drc/drc_engine.h>
#include <drc/drc_test_provider.h>
#include <font/font.h>
#include <pcbexpr_evaluator.h>
#include <pcb_track.h>
#include <connectivity/connectivity_data.h>
#include <ratsnest/ratsnest_data.h>
#include <settings/settings_manager.h>
#include <pcbnew_utils/board_test_utils.h>
#include <thread_pool.h>
#include <zone.h>
#include <router/pns_kicad_iface.h>
#include <router/pns_node.h>
#include <router/pns_router.h>
#include <router/pns_routing_settings.h>
#include <router/pns_segment.h>

#include <algorithm>
#include <atomic>
#include <memory>
#include <set>
#include <thread>
#include <vector>


BOOST_AUTO_TEST_SUITE( PNSThreading )


BOOST_AUTO_TEST_CASE( ThreadScopeKeepsEditorRouter )
{
    auto editor = std::make_unique<PNS::ROUTER>();
    BOOST_CHECK_EQUAL( PNS::ROUTER::GetInstance(), editor.get() );

    std::atomic<bool> sawNoneBefore{ false };
    std::atomic<bool> sawOwn{ false };
    std::atomic<bool> sawNoneAfter{ false };

    std::thread worker(
            [&]()
            {
                PNS::ROUTER::THREAD_SCOPE scope;
                sawNoneBefore = PNS::ROUTER::GetInstance() == nullptr;

                {
                    PNS::ROUTER own;
                    sawOwn = PNS::ROUTER::GetInstance() == &own;
                }

                sawNoneAfter = PNS::ROUTER::GetInstance() == nullptr;
            } );

    worker.join();

    BOOST_CHECK( sawNoneBefore );
    BOOST_CHECK( sawOwn );
    BOOST_CHECK( sawNoneAfter );
    BOOST_CHECK_EQUAL( PNS::ROUTER::GetInstance(), editor.get() );
}


BOOST_AUTO_TEST_CASE( ThreadScopeNestsAndRestores )
{
    auto editor = std::make_unique<PNS::ROUTER>();

    {
        PNS::ROUTER::THREAD_SCOPE outer;
        PNS::ROUTER               first;

        {
            PNS::ROUTER::THREAD_SCOPE inner;
            PNS::ROUTER               second;
            BOOST_CHECK_EQUAL( PNS::ROUTER::GetInstance(), &second );
        }

        BOOST_CHECK_EQUAL( PNS::ROUTER::GetInstance(), &first );
    }

    BOOST_CHECK_EQUAL( PNS::ROUTER::GetInstance(), editor.get() );
}


BOOST_AUTO_TEST_CASE( DestroyingAnotherRouterKeepsCurrent )
{
    auto editor = std::make_unique<PNS::ROUTER>();
    auto other = std::make_unique<PNS::ROUTER>();

    PNS::ROUTER::SetInstance( editor.get() );
    other.reset();

    BOOST_CHECK_EQUAL( PNS::ROUTER::GetInstance(), editor.get() );

    editor.reset();
    BOOST_CHECK( PNS::ROUTER::GetInstance() == nullptr );
}


BOOST_AUTO_TEST_CASE( LinkedItemUidsUniqueAcrossThreads )
{
    constexpr int threads = 8;
    constexpr int perThread = 20000;

    std::vector<std::vector<PNS::LINKED_ITEM::UNIQ_ID>> uids( threads );
    std::vector<std::thread>                            workers;

    for( int t = 0; t < threads; ++t )
    {
        workers.emplace_back(
                [&uids, t]()
                {
                    uids[t].reserve( perThread );

                    for( int i = 0; i < perThread; ++i )
                    {
                        PNS::SEGMENT seg;
                        uids[t].push_back( seg.Uid() );
                    }
                } );
    }

    for( std::thread& w : workers )
        w.join();

    std::set<PNS::LINKED_ITEM::UNIQ_ID> all;

    for( const auto& v : uids )
        all.insert( v.begin(), v.end() );

    BOOST_CHECK_EQUAL( all.size(), static_cast<size_t>( threads * perThread ) );
}


BOOST_AUTO_TEST_CASE( ClusterTimeoutPerSettings )
{
    PNS::ROUTING_SETTINGS settings( nullptr, "" );
    const int             global = ADVANCED_CFG::GetCfg().m_PNSProcessClusterTimeout;

    BOOST_CHECK_EQUAL( settings.ClusterTimeout(), global );

    settings.SetClusterTimeout( global + 1234 );
    BOOST_CHECK_EQUAL( settings.ClusterTimeout(), global + 1234 );
    BOOST_CHECK_EQUAL( ADVANCED_CFG::GetCfg().m_PNSProcessClusterTimeout, global );
}


class ZONE_SYNC_IFACE : public PNS_KICAD_IFACE_BASE
{
public:
    void HideItem( PNS::ITEM* ) override {}
    void DisplayItem( const PNS::ITEM*, int, bool, int ) override {}

    bool TestSyncZone( PNS::NODE* aWorld, ZONE* aZone ) { return syncZone( aWorld, aZone, nullptr ); }
};


BOOST_AUTO_TEST_CASE( SyncZoneLeavesZoneOutlineUntouched )
{
    BOARD board;
    board.SetCopperLayerCount( 2 );

    ZONE* zone = new ZONE( &board );
    zone->SetIsRuleArea( true );
    zone->SetDoNotAllowTracks( true );
    zone->SetLayerSet( LSET( { F_Cu } ) );
    zone->AppendCorner( VECTOR2I( 0, 0 ), -1 );
    zone->AppendCorner( VECTOR2I( 1000000, 0 ), -1 );
    zone->AppendCorner( VECTOR2I( 1000000, 1000000 ), -1 );
    zone->AppendCorner( VECTOR2I( 0, 1000000 ), -1 );
    board.Add( zone );

    BOOST_REQUIRE( !zone->Outline()->IsTriangulationUpToDate() );

    std::atomic<int> synced{ 0 };
    std::vector<std::thread> workers;

    for( int t = 0; t < 4; ++t )
    {
        workers.emplace_back(
                [&]()
                {
                    ZONE_SYNC_IFACE iface;
                    iface.SetBoard( &board );
                    PNS::NODE world;

                    if( iface.TestSyncZone( &world, zone ) )
                        synced++;
                } );
    }

    for( std::thread& w : workers )
        w.join();

    BOOST_CHECK_EQUAL( synced.load(), 4 );
    BOOST_CHECK( !zone->Outline()->IsTriangulationUpToDate() );
}


BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE( PcbnewSharedStateThreading )


BOOST_AUTO_TEST_CASE( DrcProvidersPerEngine )
{
    BOARD board;

    auto first = std::make_unique<DRC_ENGINE>( &board, &board.GetDesignSettings() );
    auto second = std::make_unique<DRC_ENGINE>( &board, &board.GetDesignSettings() );

    first->InitEngine( wxFileName() );
    second->InitEngine( wxFileName() );

    std::vector<DRC_TEST_PROVIDER*> a = first->GetTestProviders();
    std::vector<DRC_TEST_PROVIDER*> b = second->GetTestProviders();

    BOOST_REQUIRE( !a.empty() );
    BOOST_CHECK_EQUAL( a.size(), b.size() );

    std::set<DRC_TEST_PROVIDER*> seen( a.begin(), a.end() );

    for( DRC_TEST_PROVIDER* provider : b )
        BOOST_CHECK( !seen.count( provider ) );

    first.reset();

    for( DRC_TEST_PROVIDER* provider : b )
        BOOST_CHECK( !provider->GetName().IsEmpty() );
}


BOOST_AUTO_TEST_CASE( ExprBuiltinLookupDoesNotInsert )
{
    const PCBEXPR_BUILTIN_FUNCTIONS& registry = PCBEXPR_BUILTIN_FUNCTIONS::Instance();
    const size_t                     before = registry.GetSignatures().size();

    BOOST_CHECK( !registry.Get( wxS( "no_such_function_anywhere" ) ) );
    BOOST_CHECK( registry.Get( wxS( "intersectsarea" ) ) );
    BOOST_CHECK_EQUAL( registry.GetSignatures().size(), before );
}


BOOST_AUTO_TEST_CASE( ConcurrentFontLookup )
{
    std::vector<std::thread>    workers;
    std::vector<KIFONT::FONT*>  fonts( 8, nullptr );

    for( int t = 0; t < 8; ++t )
    {
        workers.emplace_back(
                [&fonts, t]()
                {
                    for( int i = 0; i < 200; ++i )
                    {
                        wxString name = wxString::Format( wxS( "kopenapi-missing-font-%d" ), i % 16 );
                        fonts[t] = KIFONT::FONT::GetFont( name, ( i & 1 ) != 0, false );
                    }
                } );
    }

    for( std::thread& w : workers )
        w.join();

    for( KIFONT::FONT* font : fonts )
        BOOST_CHECK( font != nullptr );
}


BOOST_AUTO_TEST_CASE( ConcurrentThreadPoolFirstUse )
{
    InvalidateKiCadThreadPool();

    std::vector<std::thread>   workers;
    std::vector<thread_pool*>  pools( 8, nullptr );

    for( int t = 0; t < 8; ++t )
        workers.emplace_back( [&pools, t]() { pools[t] = &GetKiCadThreadPool(); } );

    for( std::thread& w : workers )
        w.join();

    for( thread_pool* pool : pools )
        BOOST_CHECK_EQUAL( pool, pools.front() );
}


BOOST_AUTO_TEST_CASE( RatsnestReadsDoNotReorderEdges )
{
    SETTINGS_MANAGER       settings;
    std::unique_ptr<BOARD> board;
    KI_TEST::LoadBoard( settings, "complex_hierarchy", board );

    std::vector<PCB_TRACK*> tracks( board->Tracks().begin(), board->Tracks().end() );

    for( PCB_TRACK* track : tracks )
    {
        board->Remove( track );
        delete track;
    }

    board->BuildConnectivity();

    std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();
    const unsigned int                 expected = conn->GetUnconnectedCount( false );
    BOOST_REQUIRE_GT( expected, 0u );

    for( int code = 0; code < (int) board->GetNetCount(); ++code )
    {
        if( const RN_NET* net = conn->GetRatsnestForNet( code ) )
        {
            const std::vector<CN_EDGE>& edges = net->GetEdges();
            BOOST_CHECK( std::is_sorted( edges.begin(), edges.end(),
                                         []( const CN_EDGE& a, const CN_EDGE& b )
                                         {
                                             return a.StableSortCompare( b );
                                         } ) );
        }
    }

    std::atomic<int>         mismatches{ 0 };
    std::vector<std::thread> readers;

    for( int t = 0; t < 8; ++t )
    {
        readers.emplace_back(
                [&]()
                {
                    for( int i = 0; i < 200; ++i )
                    {
                        if( conn->GetUnconnectedCount( false ) != expected )
                            ++mismatches;
                    }
                } );
    }

    for( std::thread& r : readers )
        r.join();

    BOOST_CHECK_EQUAL( mismatches.load(), 0 );
}


BOOST_AUTO_TEST_SUITE_END()
