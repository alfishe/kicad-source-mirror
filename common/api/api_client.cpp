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

#include <api/api_client.h>
#include <kinng.h>


#include <fmt/format.h>
#include <wx/log.h>
#include <wx/translation.h>


KICAD_API_CLIENT::KICAD_API_CLIENT( int aTimeoutMs, bool aBlock ) :
        m_socket(),
        m_isOpen( false ),
        m_isConnected( false ),
        m_block( aBlock )
{
    int ret = nng_init( nullptr );

    if( ret == 0 || ret == NNG_EBUSY )
        ret = nng_req0_open( &m_socket );

    if( ret == 0 )
    {
        m_isOpen = true;
        nng_socket_set_ms( m_socket, NNG_OPT_RECVTIMEO, aTimeoutMs );
        nng_socket_set_ms( m_socket, NNG_OPT_SENDTIMEO, aTimeoutMs );
    }
    else
    {
        m_lastError = wxString::Format( wxS( "nng_req0_open failed: %s" ), wxString::FromUTF8( nng_strerror( static_cast<nng_err>( ret ) ) ) );
    }
}


KICAD_API_CLIENT::~KICAD_API_CLIENT()
{
    if( m_isOpen )
        nng_socket_close( m_socket );
}


bool KICAD_API_CLIENT::Connect( const wxString& aSocketUrl )
{
    if( !m_isOpen )
        return false;

    if( m_isConnected )
        return true;

    int ret = nng_dial( m_socket, KinngTransportUrl( aSocketUrl.ToStdString() ).c_str(), nullptr, 0 );

    if( ret != 0 )
    {
        m_lastError = wxString::Format( wxS( "nng_dial failed: %s" ), wxString::FromUTF8( nng_strerror( static_cast<nng_err>( ret ) ) ) );
        return false;
    }

    m_isConnected = true;
    return true;
}


void KICAD_API_CLIENT::Disconnect()
{
    if( m_isOpen )
    {
        nng_socket_close( m_socket );
        m_isOpen = false;
        m_isConnected = false;

        int ret = nng_req0_open( &m_socket );

        if( ret == 0 )
        {
            m_isOpen = true;
            nng_socket_set_ms( m_socket, NNG_OPT_RECVTIMEO, 10000 );
            nng_socket_set_ms( m_socket, NNG_OPT_SENDTIMEO, 10000 );
        }
    }
}


bool KICAD_API_CLIENT::Send( const google::protobuf::Message& aRequest, kiapi::common::ApiResponse& aResponse,
                             const std::string& aClientName )
{
    if( !IsConnected() )
    {
        m_lastError = wxS( "API client is not connected" );
        return false;
    }

    kiapi::common::ApiRequest request;
    request.mutable_header()->set_client_name( aClientName );

    if( !request.mutable_message()->PackFrom( aRequest ) )
    {
        m_lastError = wxS( "Failed to pack command into ApiRequest" );
        return false;
    }

    std::string requestStr = request.SerializeAsString();

    int ret = nng_send( m_socket, requestStr.data(), requestStr.size(), 0 );

    if( ret != 0 )
    {
        m_lastError = wxString::Format( wxS( "nng_send failed: %s" ), wxString::FromUTF8( nng_strerror( static_cast<nng_err>( ret ) ) ) );
        return false;
    }

    nng_msg* reply = nullptr;

    ret = nng_recvmsg( m_socket, &reply, m_block ? 0 : NNG_FLAG_NONBLOCK );

    if( ret != 0 )
    {
        m_lastError = wxString::Format( wxS( "nng_recvmsg failed: %s" ), wxString::FromUTF8( nng_strerror( static_cast<nng_err>( ret ) ) ) );
        return false;
    }

    const bool parsed = aResponse.ParseFromArray( nng_msg_body( reply ), (int) nng_msg_len( reply ) );
    nng_msg_free( reply );

    if( !parsed )
    {
        m_lastError = wxS( "Failed to parse reply from KiCad" );
        return false;
    }

    return true;
}
