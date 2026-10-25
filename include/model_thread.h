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
#pragma once

#include <kicommon.h>

#include <cstdint>
#include <functional>
#include <stdexcept>

/**
 * Which thread may touch a document's model, and which may also write its caches.
 *
 * The main thread always may.  A thread holding a read lease is inside a READER scope.  A document
 * owner thread (headless) runs inside an OWNER scope: it writes only inside a WRITE_SECTION, which
 * takes the document exclusively through the installed write hooks (readers drained); a
 * READ_WINDOW inside a section hands the document back to readers for a stretch of pure reading
 * (a router search).  Checks that used to say "main thread only" ask IsModelThread(); lazy caches
 * filled from const getters are written only when MayWrite().
 */
namespace MODEL_THREAD
{

enum class KIND
{
    READER,
    WRITER,
    OWNER
};

/// @brief True on the main thread and inside any SCOPE
KICOMMON_API bool IsModelThread();

/// @brief True inside a WRITER scope or a WRITE_SECTION, and on the main thread outside any scope
KICOMMON_API bool MayWrite();

/// @brief True inside any SCOPE
KICOMMON_API bool InScope();

/// @brief Thrown by CheckWrite under REFUSE_WRITES, before anything is written
class KICOMMON_API WRITE_REFUSED : public std::runtime_error
{
public:
    explicit WRITE_REFUSED( const char* aWhat ) : std::runtime_error( aWhat ) {}
};


/// @brief Called where shared derived data (board connectivity) is about to be written.  Where
/// MayWrite() is false while readers may read (a READER scope or READ_WINDOW on any thread) the
/// write is counted and logged; under REFUSE_WRITES it throws WRITE_REFUSED instead, so the call
/// can go to the owner
KICOMMON_API void CheckWrite( const char* aWhat );

/// @brief Writes CheckWrite found where MayWrite() was false, refused ones included
KICOMMON_API uint64_t WriteViolations();

/// @brief Writes CheckWrite refused (REFUSE_WRITES)
KICOMMON_API uint64_t WritesRefused();


/// @brief On this thread from construction to destruction, CheckWrite refuses writes (a read
/// under a lease, which the owner answers instead when it would write)
class KICOMMON_API REFUSE_WRITES
{
public:
    REFUSE_WRITES();
    ~REFUSE_WRITES();

    REFUSE_WRITES( const REFUSE_WRITES& ) = delete;
    REFUSE_WRITES& operator=( const REFUSE_WRITES& ) = delete;

private:
    bool m_before;
};


class KICOMMON_API SCOPE
{
public:
    explicit SCOPE( KIND aKind = KIND::READER );
    ~SCOPE();

    SCOPE( const SCOPE& ) = delete;
    SCOPE& operator=( const SCOPE& ) = delete;

private:
    KIND m_kind;
};


/// @brief Installs what a WRITE_SECTION on an owner thread calls to take (aBegin) and give back
/// (aEnd) the document exclusively.  Set before any owner thread starts; empty functions clear
KICOMMON_API void SetWriteHooks( std::function<void()> aBegin, std::function<void()> aEnd );


/// @brief An owner thread writes the document: exclusive from construction to destruction.
/// Nested sections and sections outside an OWNER scope (the main thread) do nothing
class KICOMMON_API WRITE_SECTION
{
public:
    WRITE_SECTION();
    ~WRITE_SECTION();

    WRITE_SECTION( const WRITE_SECTION& ) = delete;
    WRITE_SECTION& operator=( const WRITE_SECTION& ) = delete;

private:
    bool m_entered;
};


/// @brief Inside a WRITE_SECTION: the owner only reads from construction to destruction, and
/// readers may read meanwhile; the section is taken again on destruction.  Outside a section it
/// does nothing
class KICOMMON_API READ_WINDOW
{
public:
    READ_WINDOW();
    ~READ_WINDOW();

    READ_WINDOW( const READ_WINDOW& ) = delete;
    READ_WINDOW& operator=( const READ_WINDOW& ) = delete;

    /// @brief Takes the section again now (before writing), instead of on destruction
    void Close();

private:
    bool m_opened;
};

} // namespace MODEL_THREAD
