#if defined(__linux__)

// jemalloc as the process allocator on Linux: linked in (KICAD_ALLOCATOR=jemalloc) or preloaded.
// Found at run time through mallctl, so no build dependency on its headers.

#include "platform_linux.h"
#include "platform.h"

#include <cstdlib>
#include <dlfcn.h>
#include <string>
#include <sys/types.h>

namespace kopenapi::platform
{

namespace
{

using MALLCTL = int ( * )( const char*, void*, size_t*, void*, size_t );

MALLCTL mallctlFn()
{
    static const MALLCTL fn = reinterpret_cast<MALLCTL>( dlsym( RTLD_DEFAULT, "mallctl" ) );
    return fn;
}

constexpr unsigned ARENAS_ALL = 4096;   ///< MALLCTL_ARENAS_ALL

int64_t readSize( MALLCTL aCtl, const char* aName )
{
    size_t value = 0;
    size_t len = sizeof( value );
    return aCtl( aName, &value, &len, nullptr, 0 ) == 0 ? (int64_t) value : -1;
}

} // namespace


bool JemallocActive()
{
    return mallctlFn() != nullptr;
}


bool JemallocStats( AllocatorInfo& aInfo )
{
    MALLCTL ctl = mallctlFn();

    if( !ctl )
        return false;

    uint64_t epoch = 1;   // the stats are a snapshot refreshed by writing the epoch
    ctl( "epoch", nullptr, nullptr, &epoch, sizeof( epoch ) );

    aInfo.allocator = "jemalloc";
    aInfo.live = readSize( ctl, "stats.allocated" );
    aInfo.mapped = readSize( ctl, "stats.resident" );
    return true;
}


bool JemallocTrim()
{
    MALLCTL ctl = mallctlFn();

    if( !ctl )
        return false;

    const std::string purge = "arena." + std::to_string( ARENAS_ALL ) + ".purge";
    ctl( purge.c_str(), nullptr, nullptr, nullptr, 0 );
    return true;
}


#if defined( KICAD_ALLOCATOR_JEMALLOC )

namespace
{

/// Idle memory goes back to the OS within about a second (POC-003): dirty pages decay after 1 s,
/// no muzzy stage, a background thread does the purging. MALLOC_CONF set by the user wins.
__attribute__( ( constructor ) ) void configureJemalloc()
{
    MALLCTL ctl = mallctlFn();

    if( !ctl || std::getenv( "MALLOC_CONF" ) )
        return;

    ssize_t dirty = 1000;
    ssize_t muzzy = 0;
    ctl( "arenas.dirty_decay_ms", nullptr, nullptr, &dirty, sizeof( dirty ) );
    ctl( "arenas.muzzy_decay_ms", nullptr, nullptr, &muzzy, sizeof( muzzy ) );

    unsigned narenas = 0;
    size_t   len = sizeof( narenas );

    if( ctl( "arenas.narenas", &narenas, &len, nullptr, 0 ) == 0 )
    {
        for( unsigned i = 0; i < narenas; ++i )
        {
            const std::string arena = "arena." + std::to_string( i ) + ".";
            ctl( ( arena + "dirty_decay_ms" ).c_str(), nullptr, nullptr, &dirty, sizeof( dirty ) );
            ctl( ( arena + "muzzy_decay_ms" ).c_str(), nullptr, nullptr, &muzzy, sizeof( muzzy ) );
        }
    }

    bool background = true;
    ctl( "background_thread", nullptr, nullptr, &background, sizeof( background ) );
}

} // namespace

#endif // KICAD_ALLOCATOR_JEMALLOC

} // namespace kopenapi::platform

#endif // __linux__
