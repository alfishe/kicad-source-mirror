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

#include <boost/test/unit_test.hpp>

#include <model_thread.h>

#include <string>
#include <thread>


namespace
{

struct STATE
{
    bool model;
    bool write;
    bool scoped;
};


STATE current()
{
    return { MODEL_THREAD::IsModelThread(), MODEL_THREAD::MayWrite(), MODEL_THREAD::InScope() };
}


template <typename FN>
void onWorker( FN&& aFn )
{
    std::thread worker( std::forward<FN>( aFn ) );
    worker.join();
}

} // namespace


BOOST_AUTO_TEST_SUITE( ModelThread )


BOOST_AUTO_TEST_CASE( MainThreadMayWriteOutsideScopes )
{
    STATE s = current();
    BOOST_CHECK( s.model );
    BOOST_CHECK( s.write );
    BOOST_CHECK( !s.scoped );
}


BOOST_AUTO_TEST_CASE( MainReaderScopeBlocksCacheWrites )
{
    {
        MODEL_THREAD::SCOPE reader( MODEL_THREAD::KIND::READER );
        STATE s = current();
        BOOST_CHECK( s.model );
        BOOST_CHECK( !s.write );
        BOOST_CHECK( s.scoped );
    }

    BOOST_CHECK( MODEL_THREAD::MayWrite() );
    BOOST_CHECK( !MODEL_THREAD::InScope() );
}


BOOST_AUTO_TEST_CASE( WorkerOutsideScopesIsNotModelThread )
{
    STATE s{ true, true, true };
    onWorker( [&]() { s = current(); } );
    BOOST_CHECK( !s.model );
    BOOST_CHECK( !s.write );
    BOOST_CHECK( !s.scoped );
}


BOOST_AUTO_TEST_CASE( WorkerScopes )
{
    STATE reader{}, writer{}, nested{}, after{};

    onWorker(
            [&]()
            {
                {
                    MODEL_THREAD::SCOPE r( MODEL_THREAD::KIND::READER );
                    reader = current();
                }
                {
                    MODEL_THREAD::SCOPE w( MODEL_THREAD::KIND::WRITER );
                    writer = current();
                    MODEL_THREAD::SCOPE r( MODEL_THREAD::KIND::READER );
                    nested = current();
                }
                after = current();
            } );

    BOOST_CHECK( reader.model && !reader.write && reader.scoped );
    BOOST_CHECK( writer.model && writer.write && writer.scoped );
    BOOST_CHECK( nested.model && nested.write && nested.scoped );
    BOOST_CHECK( !after.model && !after.write && !after.scoped );
}


BOOST_AUTO_TEST_CASE( ScopesAreThreadLocal )
{
    MODEL_THREAD::SCOPE reader( MODEL_THREAD::KIND::READER );
    STATE worker{ true, true, true };

    onWorker(
            [&]()
            {
                MODEL_THREAD::SCOPE w( MODEL_THREAD::KIND::WRITER );
                worker = current();
            } );

    BOOST_CHECK( worker.write );
    BOOST_CHECK( !MODEL_THREAD::MayWrite() );
}


BOOST_AUTO_TEST_CASE( OwnerWriteSectionAndReadWindow )
{
    std::string log;
    MODEL_THREAD::SetWriteHooks( [&]() { log += 'B'; }, [&]() { log += 'E'; } );

    STATE owner{}, section{}, nested{}, window{}, closed{}, after{};

    onWorker(
            [&]()
            {
                MODEL_THREAD::SCOPE o( MODEL_THREAD::KIND::OWNER );
                owner = current();

                {
                    MODEL_THREAD::WRITE_SECTION ws;
                    section = current();

                    {
                        MODEL_THREAD::WRITE_SECTION inner;
                        nested = current();
                    }

                    {
                        MODEL_THREAD::READ_WINDOW rw;
                        window = current();
                        rw.Close();
                        closed = current();
                    }
                }

                after = current();
            } );

    MODEL_THREAD::SetWriteHooks( {}, {} );

    BOOST_CHECK_EQUAL( log, "BEBE" );
    BOOST_CHECK( owner.model && !owner.write && owner.scoped );
    BOOST_CHECK( section.write );
    BOOST_CHECK( nested.write );
    BOOST_CHECK( window.model && !window.write );
    BOOST_CHECK( closed.write );
    BOOST_CHECK( after.model && !after.write );
}


BOOST_AUTO_TEST_CASE( SectionsOutsideOwnerDoNothing )
{
    int calls = 0;
    MODEL_THREAD::SetWriteHooks( [&]() { ++calls; }, [&]() { ++calls; } );

    {
        MODEL_THREAD::WRITE_SECTION ws;
        MODEL_THREAD::READ_WINDOW   rw;
        BOOST_CHECK( MODEL_THREAD::MayWrite() );
    }

    onWorker(
            [&]()
            {
                MODEL_THREAD::SCOPE         w( MODEL_THREAD::KIND::WRITER );
                MODEL_THREAD::SCOPE         o( MODEL_THREAD::KIND::OWNER );
                MODEL_THREAD::WRITE_SECTION ws;
            } );

    MODEL_THREAD::SetWriteHooks( {}, {} );
    BOOST_CHECK_EQUAL( calls, 0 );
}


BOOST_AUTO_TEST_SUITE_END()
