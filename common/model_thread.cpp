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
#include <model_thread.h>

#include <wx/log.h>
#include <wx/thread.h>

#include <atomic>


namespace MODEL_THREAD
{

static thread_local int  tlReaders = 0;
static thread_local int  tlWriters = 0;
static thread_local int  tlOwners = 0;
static thread_local bool tlSection = false;

static thread_local bool tlRefuse = false;

static std::atomic<int>      s_readersOut{ 0 };    ///< READER scopes and open READ_WINDOWs, all threads
static std::atomic<uint64_t> s_violations{ 0 };
static std::atomic<uint64_t> s_refused{ 0 };

static std::function<void()> s_beginWrite;
static std::function<void()> s_endWrite;


bool IsModelThread()
{
    return tlReaders > 0 || tlWriters > 0 || tlOwners > 0 || wxThread::IsMain();
}


bool MayWrite()
{
    // a writer holds the document exclusively, so a read nested in an edit may write caches too
    return tlWriters > 0 || tlSection || ( tlReaders == 0 && tlOwners == 0 && wxThread::IsMain() );
}


void CheckWrite( const char* aWhat )
{
    // with nobody reading alongside, a thread writes only data of its own (a board it loads)
    if( MayWrite() || s_readersOut.load() == 0 )
        return;

    const uint64_t seen = s_violations++;

    if( tlRefuse )
    {
        s_refused++;
        throw WRITE_REFUSED( aWhat );
    }

    // the first ones only: a loop must not flood the log
    if( seen < 20 )
        wxLogWarning( "%s written off the document's owner thread (a read that writes)", aWhat );
}


uint64_t WriteViolations()
{
    return s_violations.load();
}


uint64_t WritesRefused()
{
    return s_refused.load();
}


REFUSE_WRITES::REFUSE_WRITES() :
        m_before( tlRefuse )
{
    tlRefuse = true;
}


REFUSE_WRITES::~REFUSE_WRITES()
{
    tlRefuse = m_before;
}


bool InScope()
{
    return tlReaders > 0 || tlWriters > 0 || tlOwners > 0;
}


static int& counter( KIND aKind )
{
    switch( aKind )
    {
    case KIND::WRITER: return tlWriters;
    case KIND::OWNER:  return tlOwners;
    default:           return tlReaders;
    }
}


SCOPE::SCOPE( KIND aKind ) :
        m_kind( aKind )
{
    ++counter( m_kind );

    if( m_kind == KIND::READER )
        ++s_readersOut;
}


SCOPE::~SCOPE()
{
    if( m_kind == KIND::READER )
        --s_readersOut;

    --counter( m_kind );
}


void SetWriteHooks( std::function<void()> aBegin, std::function<void()> aEnd )
{
    s_beginWrite = std::move( aBegin );
    s_endWrite = std::move( aEnd );
}


static void beginWrite()
{
    if( s_beginWrite )
        s_beginWrite();

    tlSection = true;
}


static void endWrite()
{
    tlSection = false;

    if( s_endWrite )
        s_endWrite();
}


WRITE_SECTION::WRITE_SECTION() :
        m_entered( tlOwners > 0 && !tlSection && tlWriters == 0 )
{
    if( m_entered )
        beginWrite();
}


WRITE_SECTION::~WRITE_SECTION()
{
    if( m_entered )
        endWrite();
}


READ_WINDOW::READ_WINDOW() :
        m_opened( tlSection )
{
    if( m_opened )
    {
        endWrite();
        ++s_readersOut;
    }
}


READ_WINDOW::~READ_WINDOW()
{
    Close();
}


void READ_WINDOW::Close()
{
    if( m_opened )
    {
        m_opened = false;
        --s_readersOut;
        beginWrite();
    }
}

} // namespace MODEL_THREAD
