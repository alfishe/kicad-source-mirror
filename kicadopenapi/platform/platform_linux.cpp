#if defined(__linux__)

#include "platform_linux.h"
#include "platform.h"

#include <cstdio>
#include <cstring>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace kopenapi::platform
{

std::filesystem::path ExecutablePathLinux()
{
    std::error_code ec;
    std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    return ec ? std::filesystem::path() : exe;
}

ProcessMemoryInfo ProcessMemoryLinux()
{
    ProcessMemoryInfo info;
    info.footprintKind = "rss_anon+swap";

    FILE* status = std::fopen("/proc/self/status", "r");

    if (!status)
    {
        return info;
    }

    char    line[256];
    int64_t rssAnon = -1;

    while (std::fgets(line, sizeof(line), status))
    {
        long long value = 0;

        auto field = [&](const char* aName, int64_t& aOut)
        {
            const size_t length = std::strlen(aName);

            if (std::strncmp(line, aName, length) == 0 && std::sscanf(line + length, "%lld", &value) == 1)
            {
                aOut = (int64_t) value * 1024;   // kB
            }
        };

        field("VmRSS:", info.rss);
        field("VmHWM:", info.peakRss);
        field("VmSize:", info.virtualSize);
        field("VmSwap:", info.swapped);
        field("RssAnon:", rssAnon);

        if (std::strncmp(line, "Threads:", 8) == 0)
        {
            std::sscanf(line + 8, "%d", &info.threads);
        }
    }

    std::fclose(status);

    if (rssAnon >= 0)
    {
        info.footprint = rssAnon + (info.swapped > 0 ? info.swapped : 0);
    }

    return info;
}

AllocatorInfo AllocatorStatsLinux()
{
    AllocatorInfo info;
#if defined(__GLIBC__) && ( __GLIBC__ > 2 || ( __GLIBC__ == 2 && __GLIBC_MINOR__ >= 33 ) )
    info.allocator = "glibc";
    const struct mallinfo2 mi = mallinfo2();
    info.live = (int64_t) ( mi.uordblks + mi.hblkhd );
    info.mapped = (int64_t) ( mi.arena + mi.hblkhd );
#endif
    return info;
}

int64_t AllocatorTrimLinux()
{
#if defined(__GLIBC__)
    malloc_trim(0);
#endif
    return -1;
}

void LowerCurrentThreadPriorityLinux()
{
    // nice applies per thread on Linux
    setpriority(PRIO_PROCESS, (id_t) syscall(SYS_gettid), 10);
}

} // namespace kopenapi::platform

#endif // __linux__
