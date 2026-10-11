// This program source code file is part of KiCad, a free EDA CAD application.
//
// Copyright The KiCad Developers, see AUTHORS.txt for contributors.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#if defined( _MSC_VER ) && ( defined( _M_X64 ) || defined( _M_IX86 ) )
#include <xmmintrin.h>
#elif defined( _MSC_VER ) && defined( _M_ARM64 )
#include <intrin.h>
#endif

/// @brief Hint the CPU to load the cache line at aAddress (no effect where the compiler has no hint)
inline void KiPrefetch( const void* aAddress )
{
#if defined( __GNUC__ ) || defined( __clang__ )
    __builtin_prefetch( aAddress );
#elif defined( _MSC_VER ) && ( defined( _M_X64 ) || defined( _M_IX86 ) )
    _mm_prefetch( static_cast<const char*>( aAddress ), _MM_HINT_T0 );
#elif defined( _MSC_VER ) && defined( _M_ARM64 )
    __prefetch( aAddress );
#else
    (void) aAddress;
#endif
}
