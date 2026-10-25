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

// kicadopenapi threading model: owner strand, read leases, change sets, document version

#include <boost/test/unit_test.hpp>

#include <kicadopenapi_change_feed.h>
#include <threading/kicadopenapi_change_set.h>
#include <threading/kicadopenapi_doc_gate.h>
#include <threading/kicadopenapi_doc_owner.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <numeric>
#include <set>
#include <thread>
#include <vector>

using namespace std::chrono_literals;


namespace
{

using MICROS = std::chrono::microseconds;

KOPENAPI_DOC_GATE::OPTIONS gateOptions( MICROS aBudget, MICROS aStuckAfter )
{
    KOPENAPI_DOC_GATE::OPTIONS options;
    options.leaseBudget = aBudget;
    options.stuckAfter = aStuckAfter;
    return options;
}


/// Collects violations from the writer's thread
struct VIOLATIONS
{
    std::mutex                            mutex;
    std::vector<KOPENAPI_LEASE_VIOLATION> list;

    KOPENAPI_DOC_GATE::VIOLATION_SINK Sink()
    {
        return [this]( const KOPENAPI_LEASE_VIOLATION& aViolation )
        {
            std::lock_guard<std::mutex> lock( mutex );
            list.push_back( aViolation );
        };
    }

    size_t Count( KOPENAPI_LEASE_VIOLATION::KIND aKind )
    {
        std::lock_guard<std::mutex> lock( mutex );
        return std::count_if( list.begin(), list.end(),
                              [&]( const KOPENAPI_LEASE_VIOLATION& aV ) { return aV.kind == aKind; } );
    }
};


/// Spins until aCondition holds or 5 s pass
template <typename FN>
bool waitFor( FN aCondition )
{
    auto until = std::chrono::steady_clock::now() + 5s;

    while( !aCondition() )
    {
        if( std::chrono::steady_clock::now() > until )
            return false;

        std::this_thread::sleep_for( 100us );
    }

    return true;
}

} // namespace


BOOST_AUTO_TEST_SUITE( KicadOpenApiThreading )


BOOST_AUTO_TEST_CASE( ChangeSetFolding )
{
    KOPENAPI_CHANGE_SET changes( "api:test" );

    changes.Added( "a" );
    changes.Changed( "a" );     // still added
    changes.Changed( "b" );
    changes.Removed( "b" );     // removed
    changes.Added( "c" );
    changes.Removed( "c" );     // gone
    changes.Removed( "d" );
    changes.Added( "d" );       // re-added: changed
    changes.Changed( "e" );

    BOOST_CHECK_EQUAL( changes.Size(), 4u );
    BOOST_CHECK( changes.Ids( KOPENAPI_CHANGE::ADDED ) == std::vector<std::string>{ "a" } );
    BOOST_CHECK( changes.Ids( KOPENAPI_CHANGE::REMOVED ) == std::vector<std::string>{ "b" } );
    BOOST_CHECK( ( changes.Ids( KOPENAPI_CHANGE::CHANGED ) == std::vector<std::string>{ "d", "e" } ) );

    // Order of first record kept after a fold removed an entry
    BOOST_CHECK_EQUAL( changes.Items()[0].first, "a" );
    BOOST_CHECK_EQUAL( changes.Items()[1].first, "b" );
    BOOST_CHECK_EQUAL( changes.Items()[2].first, "d" );
    BOOST_CHECK_EQUAL( changes.Items()[3].first, "e" );

    KOPENAPI_CHANGE_SET first, second, coalesced;
    first.Added( "x" );
    first.SetVersions( 4, 5 );
    second.Removed( "x" );
    second.Changed( "y" );
    second.SetVersions( 5, 6 );

    coalesced.Merge( first );
    coalesced.Merge( second );

    BOOST_CHECK_EQUAL( coalesced.FromVersion(), 4u );
    BOOST_CHECK_EQUAL( coalesced.ToVersion(), 6u );
    BOOST_CHECK_EQUAL( coalesced.Size(), 1u );
    BOOST_CHECK( coalesced.Ids( KOPENAPI_CHANGE::CHANGED ) == std::vector<std::string>{ "y" } );
}


BOOST_AUTO_TEST_CASE( StrandOrderAndThread )
{
    KOPENAPI_DOC_OWNER owner( "order" );

    constexpr int              threads = 4;
    constexpr int              perThread = 500;
    std::vector<std::pair<int, int>> seen;   // strand only
    std::atomic<int>           offStrand{ 0 };
    std::vector<std::thread>   posters;

    for( int t = 0; t < threads; ++t )
    {
        posters.emplace_back(
                [&, t]()
                {
                    for( int i = 0; i < perThread; ++i )
                    {
                        owner.Post(
                                [&, t, i]()
                                {
                                    if( !owner.IsOwnerThread() )
                                        offStrand++;

                                    seen.emplace_back( t, i );
                                } );
                    }
                } );
    }

    for( std::thread& poster : posters )
        poster.join();

    owner.Read( []() { return 0; } ).get();   // after every post above

    BOOST_CHECK_EQUAL( offStrand.load(), 0 );
    BOOST_REQUIRE_EQUAL( seen.size(), size_t( threads * perThread ) );

    std::map<int, int> next;

    for( const auto& [t, i] : seen )
    {
        BOOST_CHECK_EQUAL( i, next[t] );
        next[t] = i + 1;
    }

    BOOST_CHECK( !owner.IsOwnerThread() );
}


BOOST_AUTO_TEST_CASE( VersionMonotonic )
{
    KOPENAPI_DOC_OWNER owner( "version" );
    std::vector<std::pair<uint64_t, uint64_t>> published;
    std::atomic<int>                           offStrand{ 0 };

    owner.Subscribe(
            [&]( const KOPENAPI_CHANGE_SET& aChanges )
            {
                if( !owner.IsOwnerThread() )
                    offStrand++;

                published.emplace_back( aChanges.FromVersion(), aChanges.ToVersion() );
            } );

    BOOST_CHECK_EQUAL( owner.Version(), 0u );

    BOOST_CHECK_EQUAL( owner.Edit( "api:1", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "a" ); } ).get(), 1u );

    // Nothing changed: no new version, nothing published
    BOOST_CHECK_EQUAL( owner.Edit( "api:2", []( KOPENAPI_CHANGE_SET& ) {} ).get(), 1u );

    // Abandoned edit: no new version, the error reaches the caller
    std::future<uint64_t> failed = owner.Edit( "api:3",
                                               []( KOPENAPI_CHANGE_SET& c )
                                               {
                                                   c.Changed( "a" );
                                                   throw std::runtime_error( "bad" );
                                               } );
    BOOST_CHECK_THROW( failed.get(), std::runtime_error );
    BOOST_CHECK_EQUAL( owner.Version(), 1u );

    for( int i = 0; i < 100; ++i )
        owner.Edit( "api:n", [i]( KOPENAPI_CHANGE_SET& c ) { c.Changed( std::to_string( i ) ); } );

    BOOST_CHECK_EQUAL( owner.Edit( "api:last", []( KOPENAPI_CHANGE_SET& c ) { c.Removed( "a" ); } ).get(), 102u );

    owner.Read( []() { return 0; } ).get();

    BOOST_CHECK_EQUAL( offStrand.load(), 0 );
    BOOST_REQUIRE_EQUAL( published.size(), 102u );

    for( size_t i = 0; i < published.size(); ++i )
    {
        BOOST_CHECK_EQUAL( published[i].first, i );
        BOOST_CHECK_EQUAL( published[i].second, i + 1 );
    }
}


BOOST_AUTO_TEST_CASE( EditOnStrandRunsInline )
{
    KOPENAPI_DOC_OWNER owner( "inline" );

    uint64_t inlineVersion = owner.Read(
            [&]()
            {
                return owner.Edit( "inner", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "a" ); } ).get();
            } ).get();

    BOOST_CHECK_EQUAL( inlineVersion, 1u );

    std::atomic<bool> innerRefused{ false };

    std::future<uint64_t> nested = owner.Edit( "outer",
            [&]( KOPENAPI_CHANGE_SET& c )
            {
                c.Changed( "a" );
                std::future<uint64_t> inner = owner.Edit( "inner", []( KOPENAPI_CHANGE_SET& ) {} );

                try
                {
                    inner.get();
                }
                catch( const std::logic_error& )
                {
                    innerRefused = true;
                }
            } );

    BOOST_CHECK_EQUAL( nested.get(), 2u );
    BOOST_CHECK( innerRefused );
}


BOOST_AUTO_TEST_CASE( LeaseExcludesWrite )
{
    KOPENAPI_DOC_OWNER owner( "exclusion", gateOptions( 1s, 5s ) );
    std::atomic<bool>  leaseHeld{ false };
    std::atomic<bool>  released{ false };
    std::atomic<bool>  writeBeforeRelease{ false };

    std::thread reader(
            [&]()
            {
                KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "reader" );
                leaseHeld = true;
                waitFor( [&]() { return owner.Gate().Stats().writersWaiting == 1; } );
                std::this_thread::sleep_for( 15ms );
                released = true;
                lease.Release();
            } );

    BOOST_REQUIRE( waitFor( [&]() { return leaseHeld.load(); } ) );

    owner.Edit( "api:w",
                [&]( KOPENAPI_CHANGE_SET& c )
                {
                    if( !released )
                        writeBeforeRelease = true;

                    c.Added( "a" );
                } ).get();

    reader.join();

    BOOST_CHECK( !writeBeforeRelease );
    BOOST_CHECK_GE( owner.Gate().Stats().maxWriteWait.count(), 10000 );

    // During a write a lease waits; with a wait limit it gives up
    std::atomic<bool> inWrite{ false };
    std::atomic<bool> finishWrite{ false };

    std::future<uint64_t> slow = owner.Edit( "api:slow",
                                             [&]( KOPENAPI_CHANGE_SET& c )
                                             {
                                                 inWrite = true;
                                                 waitFor( [&]() { return finishWrite.load(); } );
                                                 c.Changed( "a" );
                                             } );

    BOOST_REQUIRE( waitFor( [&]() { return inWrite.load(); } ) );
    BOOST_CHECK( !owner.Gate().Acquire( "impatient", std::nullopt, MICROS( 5000 ) ).Valid() );

    std::atomic<uint64_t> patientVersion{ 0 };

    std::thread waiter(
            [&]()
            {
                KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "patient" );

                if( lease.Valid() )
                    patientVersion = lease.Version();
            } );

    std::this_thread::sleep_for( 5ms );
    finishWrite = true;
    BOOST_CHECK_EQUAL( slow.get(), 2u );
    waiter.join();
    BOOST_CHECK_EQUAL( patientVersion.load(), 2u );
}


BOOST_AUTO_TEST_CASE( WriterPreference )
{
    KOPENAPI_DOC_OWNER owner( "preference", gateOptions( 1s, 5s ) );
    std::atomic<bool>  firstHeld{ false };
    std::atomic<bool>  releaseFirst{ false };
    std::atomic<int>   order{ 0 };
    std::atomic<int>   writeAt{ 0 };
    std::atomic<int>   secondAt{ 0 };
    std::atomic<uint64_t> secondVersion{ 0 };

    std::thread first(
            [&]()
            {
                KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "first" );
                firstHeld = true;
                waitFor( [&]() { return releaseFirst.load(); } );
            } );

    BOOST_REQUIRE( waitFor( [&]() { return firstHeld.load(); } ) );

    std::future<uint64_t> write = owner.Edit( "api:w",
                                              [&]( KOPENAPI_CHANGE_SET& c )
                                              {
                                                  writeAt = ++order;
                                                  c.Added( "a" );
                                              } );

    // The writer is waiting on the first lease: a new lease from another thread waits too
    BOOST_REQUIRE( waitFor( [&]() { return owner.Gate().Stats().writersWaiting == 1; } ) );

    std::thread second(
            [&]()
            {
                KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "second" );
                secondAt = ++order;
                secondVersion = lease.Version();
            } );

    std::this_thread::sleep_for( 10ms );
    BOOST_CHECK_EQUAL( secondAt.load(), 0 );

    releaseFirst = true;
    first.join();
    write.get();
    second.join();

    BOOST_CHECK_EQUAL( writeAt.load(), 1 );
    BOOST_CHECK_EQUAL( secondAt.load(), 2 );
    BOOST_CHECK_EQUAL( secondVersion.load(), 1u );
}


BOOST_AUTO_TEST_CASE( ReadersBetweenBackToBackWrites )
{
    KOPENAPI_DOC_OWNER    owner( "fair", gateOptions( 1s, 5s ) );
    std::atomic<bool>     inFirst{ false };
    std::atomic<bool>     finishFirst{ false };
    std::atomic<uint64_t> readVersion{ 0 };

    std::future<uint64_t> first = owner.Edit( "api:first",
                                              [&]( KOPENAPI_CHANGE_SET& c )
                                              {
                                                  inFirst = true;
                                                  waitFor( [&]() { return finishFirst.load(); } );
                                                  c.Added( "a" );
                                              } );

    std::future<uint64_t> second = owner.Edit( "api:second", []( KOPENAPI_CHANGE_SET& c ) { c.Changed( "a" ); } );

    BOOST_REQUIRE( waitFor( [&]() { return inFirst.load(); } ) );

    std::thread reader(
            [&]()
            {
                KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "between" );

                if( lease )
                    readVersion = lease.Version();
            } );

    BOOST_REQUIRE( waitFor( [&]() { return owner.Gate().Stats().readersWaiting == 1; } ) );
    finishFirst = true;

    BOOST_CHECK_EQUAL( first.get(), 1u );
    BOOST_CHECK_EQUAL( second.get(), 2u );
    reader.join();

    // It waited through the first write and went before the second
    BOOST_CHECK_EQUAL( readVersion.load(), 1u );
}


BOOST_AUTO_TEST_CASE( NestedLeaseWhileWriterWaits )
{
    KOPENAPI_DOC_OWNER owner( "nested", gateOptions( 1s, 5s ) );
    std::atomic<bool>  outerHeld{ false };
    std::atomic<bool>  nestedGranted{ false };
    std::atomic<bool>  heldHere{ false };

    std::thread reader(
            [&]()
            {
                KOPENAPI_READ_LEASE outer = owner.Gate().Acquire( "outer" );
                outerHeld = true;
                waitFor( [&]() { return owner.Gate().Stats().writersWaiting == 1; } );

                KOPENAPI_READ_LEASE inner = owner.Gate().Acquire( "inner", std::nullopt, MICROS( 500000 ) );
                nestedGranted = inner.Valid();
                heldHere = owner.Gate().HeldByThisThread();
            } );

    BOOST_REQUIRE( waitFor( [&]() { return outerHeld.load(); } ) );
    owner.Edit( "api:w", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "a" ); } ).get();
    reader.join();

    BOOST_CHECK( nestedGranted );
    BOOST_CHECK( heldHere );
    BOOST_CHECK( !owner.Gate().HeldByThisThread() );
}


BOOST_AUTO_TEST_CASE( BudgetReclaimAndJournal )
{
    KOPENAPI_DOC_OWNER owner( "budget", gateOptions( MICROS( 5000 ), 5s ) );
    VIOLATIONS         violations;
    std::atomic<bool>  held{ false };
    std::atomic<bool>  sawRevoke{ false };
    std::atomic<bool>  released{ false };
    std::atomic<bool>  writeBeforeRelease{ false };

    owner.Gate().SetViolationSink( violations.Sink() );

    std::thread reader(
            [&]()
            {
                KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "pcb_route:sync_world" );
                held = true;

                try
                {
                    for( int unit = 0; unit < 10000; ++unit )
                    {
                        lease.Check();
                        std::this_thread::sleep_for( 1ms );
                    }
                }
                catch( const KOPENAPI_LEASE_REVOKED& )
                {
                    sawRevoke = true;
                }

                released = true;
            } );

    BOOST_REQUIRE( waitFor( [&]() { return held.load(); } ) );

    owner.Edit( "api:w",
                [&]( KOPENAPI_CHANGE_SET& c )
                {
                    if( !released )
                        writeBeforeRelease = true;

                    c.Added( "a" );
                } ).get();

    reader.join();

    BOOST_CHECK( sawRevoke );
    BOOST_CHECK( !writeBeforeRelease );
    BOOST_CHECK_EQUAL( violations.Count( KOPENAPI_LEASE_VIOLATION::KIND::REVOKED ), 1u );
    BOOST_CHECK_EQUAL( violations.Count( KOPENAPI_LEASE_VIOLATION::KIND::STUCK ), 0u );
    BOOST_REQUIRE( !violations.list.empty() );
    BOOST_CHECK_EQUAL( violations.list[0].holder, "pcb_route:sync_world" );
    BOOST_CHECK_GE( violations.list[0].held.count(), 5000 );
    BOOST_CHECK_EQUAL( violations.list[0].budget.count(), 5000 );
    BOOST_CHECK_EQUAL( owner.Gate().Stats().revoked, 1u );

    // A lease within its budget is never revoked
    KOPENAPI_READ_LEASE quick;
    std::thread         quickReader( [&]() { quick = owner.Gate().Acquire( "quick", MICROS( 1000000 ) ); } );
    quickReader.join();
    std::future<uint64_t> write = owner.Edit( "api:w2", []( KOPENAPI_CHANGE_SET& c ) { c.Changed( "a" ); } );
    std::this_thread::sleep_for( 5ms );
    BOOST_CHECK( !quick.Revoked() );
    quick.Release();
    write.get();
    BOOST_CHECK_EQUAL( owner.Gate().Stats().revoked, 1u );
}


BOOST_AUTO_TEST_CASE( StuckLeaseNeverWrittenUnder )
{
    KOPENAPI_DOC_OWNER owner( "stuck", gateOptions( MICROS( 2000 ), MICROS( 20000 ) ) );
    VIOLATIONS         violations;
    std::atomic<bool>  held{ false };
    std::atomic<bool>  released{ false };
    std::atomic<bool>  writeBeforeRelease{ false };
    std::atomic<bool>  revokedSeen{ false };
    std::atomic<bool>  checkThrew{ false };

    owner.Gate().SetViolationSink( violations.Sink() );

    std::thread reader(
            [&]()
            {
                KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "ignores_revoke" );
                held = true;
                waitFor( [&]() { return owner.Gate().Stats().writersWaiting == 1; } );
                std::this_thread::sleep_for( 80ms );
                revokedSeen = lease.Revoked();

                try
                {
                    lease.Check();
                }
                catch( const KOPENAPI_LEASE_REVOKED& )
                {
                    checkThrew = true;
                }

                released = true;
            } );

    BOOST_REQUIRE( waitFor( [&]() { return held.load(); } ) );

    owner.Edit( "api:w",
                [&]( KOPENAPI_CHANGE_SET& c )
                {
                    if( !released )
                        writeBeforeRelease = true;

                    c.Added( "a" );
                } ).get();

    reader.join();

    BOOST_CHECK( !writeBeforeRelease );
    BOOST_CHECK( revokedSeen );
    BOOST_CHECK( checkThrew );
    BOOST_CHECK_EQUAL( violations.Count( KOPENAPI_LEASE_VIOLATION::KIND::REVOKED ), 1u );
    BOOST_CHECK_EQUAL( violations.Count( KOPENAPI_LEASE_VIOLATION::KIND::STUCK ), 1u );
}


BOOST_AUTO_TEST_CASE( ShortClaimDefersAndReplays )
{
    KOPENAPI_DOC_OWNER       owner( "short" );
    std::vector<std::string> applied;   // strand only
    std::set<std::string>    items;     // the "document", strand only

    uint64_t token = owner.BeginClaim( KOPENAPI_CLAIM_KIND::SHORT, "job:route" ).get();

    auto addItem = [&]( const std::string& aSource, const std::string& aId, uint64_t aClaim = 0 )
    {
        return owner.Edit( aSource,
                           [&, aSource, aId]( KOPENAPI_CHANGE_SET& c )
                           {
                               items.insert( aId );
                               applied.push_back( aSource );
                               c.Added( aId );
                           },
                           aClaim );
    };

    auto removeItem = [&]( const std::string& aSource, const std::string& aId, uint64_t aClaim = 0 )
    {
        return owner.Edit( aSource,
                           [&, aSource, aId]( KOPENAPI_CHANGE_SET& c )
                           {
                               if( !items.count( aId ) )
                                   throw std::invalid_argument( aId + " not found" );

                               items.erase( aId );
                               applied.push_back( aSource );
                               c.Removed( aId );
                           },
                           aClaim );
    };

    std::future<uint64_t> own1 = addItem( "job:route", "t1", token );
    std::future<uint64_t> user1 = addItem( "gui", "u1" );
    std::future<uint64_t> own2 = addItem( "job:route", "t2", token );
    std::future<uint64_t> userBad = removeItem( "api:delete", "t1" );   // valid now, invalid at replay
    std::future<uint64_t> own3 = removeItem( "job:route", "t1", token );
    std::future<uint64_t> user2 = addItem( "api:add", "u2" );

    BOOST_CHECK_EQUAL( own3.get(), 3u );
    BOOST_CHECK( user1.wait_for( 0s ) == std::future_status::timeout );

    owner.EndClaim( token ).get();

    BOOST_CHECK_EQUAL( own1.get(), 1u );
    BOOST_CHECK_EQUAL( own2.get(), 2u );
    BOOST_CHECK_EQUAL( user1.get(), 4u );
    BOOST_CHECK_THROW( userBad.get(), std::invalid_argument );
    BOOST_CHECK_EQUAL( user2.get(), 5u );

    BOOST_CHECK( ( applied == std::vector<std::string>{ "job:route", "job:route", "job:route", "gui", "api:add" } ) );
    BOOST_CHECK( ( items == std::set<std::string>{ "t2", "u1", "u2" } ) );
    BOOST_CHECK_EQUAL( owner.Stats().deferred, 3u );
    BOOST_CHECK_EQUAL( owner.Stats().replayed, 3u );
    BOOST_CHECK( !owner.Claim() );
}


BOOST_AUTO_TEST_CASE( LongClaimRefusesOthersServesReads )
{
    KOPENAPI_DOC_OWNER owner( "long" );

    uint64_t token = owner.BeginClaim( KOPENAPI_CLAIM_KIND::LONG, "job:fill" ).get();

    BOOST_CHECK_THROW( owner.BeginClaim( KOPENAPI_CLAIM_KIND::SHORT, "job:other" ).get(), KOPENAPI_DOC_BUSY );

    std::future<uint64_t> user = owner.Edit( "gui", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "u" ); } );

    try
    {
        user.get();
        BOOST_FAIL( "edit during a long claim was not refused" );
    }
    catch( const KOPENAPI_DOC_BUSY& busy )
    {
        BOOST_CHECK_EQUAL( busy.Claim().source, "job:fill" );
        BOOST_CHECK( busy.Claim().kind == KOPENAPI_CLAIM_KIND::LONG );
    }

    BOOST_CHECK_EQUAL( owner.Read( [&]() { return owner.Version(); } ).get(), 0u );

    KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "reader" );
    BOOST_CHECK( lease.Valid() );
    lease.Release();

    BOOST_CHECK_EQUAL( owner.Edit( "job:fill", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "z" ); }, token ).get(), 1u );

    owner.EndClaim( token ).get();
    BOOST_CHECK_THROW( owner.EndClaim( token ).get(), std::invalid_argument );
    BOOST_CHECK_EQUAL( owner.Edit( "gui", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "u" ); } ).get(), 2u );
    BOOST_CHECK_EQUAL( owner.Stats().refused, 1u );
}


BOOST_AUTO_TEST_CASE( StopDropsPending )
{
    std::future<uint64_t> pending;
    std::future<uint64_t> afterStop;
    std::atomic<bool>     started{ false };

    {
        KOPENAPI_DOC_OWNER owner( "stop" );

        owner.Post(
                [&]()
                {
                    started = true;
                    std::this_thread::sleep_for( 200ms );
                } );

        BOOST_REQUIRE( waitFor( [&]() { return started.load(); } ) );
        pending = owner.Edit( "api:late", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "a" ); } );
        owner.Stop();

        afterStop = owner.Edit( "api:after", []( KOPENAPI_CHANGE_SET& c ) { c.Added( "b" ); } );
        BOOST_CHECK_EQUAL( owner.Version(), 0u );
    }

    BOOST_CHECK_THROW( pending.get(), std::future_error );
    BOOST_CHECK_THROW( afterStop.get(), std::future_error );
}


// Readers under leases must always see the invariant the owner keeps: every slot holds the
// same value.  A write between two slots would break it.
BOOST_AUTO_TEST_CASE( StressReadersAndWriter )
{
    KOPENAPI_DOC_OWNER owner( "stress", gateOptions( MICROS( 200000 ), 5s ) );
    std::vector<int>   document( 4096, 0 );
    std::atomic<bool>  done{ false };
    std::atomic<int>   torn{ 0 };
    std::atomic<int>   versionMoved{ 0 };
    std::atomic<long>  reads{ 0 };
    std::atomic<int>   started{ 0 };
    std::atomic<int>   versionsSeen{ 0 };   // reads whose version differs from the reader's last
    std::vector<std::thread> readers;

    for( int r = 0; r < 6; ++r )
    {
        readers.emplace_back(
                [&, r]()
                {
                    uint64_t last = 0;
                    bool     fresh = true;

                    while( !done )
                    {
                        KOPENAPI_READ_LEASE lease = owner.Gate().Acquire( "reader" + std::to_string( r ) );

                        if( !lease )
                            continue;

                        const int first = document[0];

                        for( int value : document )
                        {
                            if( value != first )
                            {
                                torn++;
                                break;
                            }
                        }

                        if( lease.Version() != owner.Version() )
                            versionMoved++;

                        if( !fresh && lease.Version() != last )
                            versionsSeen++;

                        if( fresh )
                            started++;

                        fresh = false;
                        last = lease.Version();
                        reads++;
                    }
                } );
    }

    BOOST_REQUIRE( waitFor( [&]() { return started.load() == 6; } ) );

    for( int i = 1; i <= 300; ++i )
    {
        owner.Edit( "api:fill",
                    [&, i]( KOPENAPI_CHANGE_SET& c )
                    {
                        for( int& slot : document )
                            slot = i;

                        c.Changed( "document" );
                    } );
    }

    BOOST_CHECK_EQUAL( owner.Read( [&]() { return owner.Version(); } ).get(), 300u );
    done = true;

    for( std::thread& reader : readers )
        reader.join();

    BOOST_CHECK_EQUAL( torn.load(), 0 );
    BOOST_CHECK_EQUAL( versionMoved.load(), 0 );
    BOOST_CHECK_GT( reads.load(), 0 );
    BOOST_CHECK_GT( versionsSeen.load(), 0 );   // readers got in between the writes
    BOOST_CHECK_EQUAL( owner.Gate().Stats().revoked, 0u );
}


BOOST_AUTO_TEST_CASE( ChangeFeedSinceAndFold )
{
    const std::string domain = "qa_feed";
    const uint64_t    start = KOPENAPI_CHANGE_FEED::Version( domain );

    KOPENAPI_CHANGE_FEED::Record( domain, { "a" }, {}, {} );
    KOPENAPI_CHANGE_FEED::Record( domain, {}, { "a", "b" }, {} );
    KOPENAPI_CHANGE_FEED::Record( domain, {}, {}, { "a" } );
    KOPENAPI_CHANGE_FEED::Record( domain, {}, {}, {} );   // empty: not a version

    BOOST_CHECK_EQUAL( KOPENAPI_CHANGE_FEED::Version( domain ), start + 3 );

    nlohmann::json all = KOPENAPI_CHANGE_FEED::Since( domain, start, 100, false );
    BOOST_REQUIRE_EQUAL( all["changes"].size(), 3 );
    BOOST_CHECK_EQUAL( all["changes"][0]["added"][0], "a" );
    BOOST_CHECK_EQUAL( all["changes"][1]["from"], start + 1 );
    BOOST_CHECK_EQUAL( all["changes"][2]["removed"][0], "a" );
    BOOST_CHECK_EQUAL( all["truncated"], false );
    BOOST_CHECK_EQUAL( all["more"], false );

    // folded: a was added and removed in the span, b only changed
    nlohmann::json folded = KOPENAPI_CHANGE_FEED::Since( domain, start, 100, true );
    BOOST_REQUIRE_EQUAL( folded["changes"].size(), 1 );
    BOOST_CHECK( folded["changes"][0]["added"].empty() );
    BOOST_CHECK( folded["changes"][0]["removed"].empty() );
    BOOST_CHECK_EQUAL( folded["changes"][0]["changed"], nlohmann::json::array( { "b" } ) );
    BOOST_CHECK_EQUAL( folded["changes"][0]["to"], start + 3 );

    nlohmann::json page = KOPENAPI_CHANGE_FEED::Since( domain, start, 2, false );
    BOOST_CHECK_EQUAL( page["changes"].size(), 2 );
    BOOST_CHECK_EQUAL( page["more"], true );
    BOOST_CHECK_EQUAL( page["next_since"], start + 2 );

    // only the last KEEP change sets are kept
    for( size_t i = 0; i < KOPENAPI_CHANGE_FEED::KEEP + 5; ++i )
        KOPENAPI_CHANGE_FEED::Record( domain, {}, { "c" }, {} );

    BOOST_CHECK_EQUAL( KOPENAPI_CHANGE_FEED::Since( domain, start, 1, false )["truncated"], true );
    BOOST_CHECK_EQUAL( KOPENAPI_CHANGE_FEED::Since( domain, start + 1000000, 1, false )["truncated"], true );

    KOPENAPI_CHANGE_FEED::Reset( domain );
    nlohmann::json reset = KOPENAPI_CHANGE_FEED::Since( domain, KOPENAPI_CHANGE_FEED::Version( domain ) - 1, 10, false );
    BOOST_REQUIRE_EQUAL( reset["changes"].size(), 1 );
    BOOST_CHECK_EQUAL( reset["changes"][0]["reset"], true );
}


BOOST_AUTO_TEST_SUITE_END()
