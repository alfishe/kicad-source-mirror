/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2023 Jon Evans <jon@craftyjon.com>
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

#include <wx/filename.h>
#include <wx/stdpaths.h>

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <qa_utils/file_utils.h>

#include <kinng.h>
#include <nng/nng.h>

#include <chrono>
#include <thread>

#include <import_export.h>
#include <api/common/envelope.pb.h>

BOOST_AUTO_TEST_SUITE( KiNNG )

BOOST_AUTO_TEST_CASE( CreateIPCResponder )
{
    KI_TEST::SCOPED_TEMP_DIR tempDir( "kicad_qa_kinng" );

    KINNG_REQUEST_SERVER server( tempDir.Path().string() );
}

// A request through a REQ socket reaches the server's callback and its reply comes back, for a short
// message and for one of half a megabyte
BOOST_AUTO_TEST_CASE( RequestReplyRoundTrip )
{
    KI_TEST::SCOPED_TEMP_DIR tempDir( "kicad_qa_kinng" );

#ifdef _WIN32
    const std::string url = "ipc://kicad-qa-kinng-" + tempDir.Path().filename().string();
#else
    const std::string url = "ipc://" + ( tempDir.Path() / "kinng.sock" ).string();
#endif

    KINNG_REQUEST_SERVER server( url );
    server.SetCallback( [&]( std::string* aMessage ) { server.Reply( "echo:" + *aMessage ); } );
    BOOST_REQUIRE( server.Start() );

    nng_socket client;
    BOOST_REQUIRE_EQUAL( nng_init( nullptr ), 0 );
    BOOST_REQUIRE_EQUAL( nng_req0_open( &client ), 0 );
    nng_socket_set_ms( client, NNG_OPT_RECVTIMEO, 5000 );
    nng_socket_set_ms( client, NNG_OPT_SENDTIMEO, 5000 );

    int dialed = -1;

    for( int attempt = 0; attempt < 50 && dialed != 0; ++attempt )
    {
        dialed = nng_dial( client, KinngTransportUrl( url ).c_str(), nullptr, 0 );

        if( dialed != 0 )
            std::this_thread::sleep_for( std::chrono::milliseconds( 20 ) );
    }

    BOOST_REQUIRE_EQUAL( dialed, 0 );

    for( const std::string& request : { std::string( "hello" ), std::string( 512 * 1024, 'k' ) } )
    {
        BOOST_REQUIRE_EQUAL( nng_send( client, request.data(), request.size(), 0 ), 0 );

        nng_msg* reply = nullptr;
        BOOST_REQUIRE_EQUAL( nng_recvmsg( client, &reply, 0 ), 0 );
        const std::string got( static_cast<const char*>( nng_msg_body( reply ) ), nng_msg_len( reply ) );
        nng_msg_free( reply );

        BOOST_CHECK( got == "echo:" + request );
    }

    nng_socket_close( client );
    server.Stop();
}

BOOST_AUTO_TEST_SUITE_END()
