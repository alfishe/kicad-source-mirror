#if defined(__linux__)

// jemalloc as the process allocator on Linux: linked in (KICAD_ALLOCATOR, thirdparty/jemalloc, its
// settings compiled in) or preloaded. Found at run time through mallctl: no build dependency on it.

#include "platform_linux.h"
#include "platform.h"

#include <dlfcn.h>
#include <string>

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

} // namespace kopenapi::platform

#endif // __linux__
