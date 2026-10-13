#if defined(__APPLE__)

#include "platform_macos.h"
#include "platform.h"

#include <algorithm>
#include <climits>
#include <cstdint>
#include <libproc.h>
#include <mach-o/dyld.h>
#include <mach/mach.h>
#include <malloc/malloc.h>
#include <pthread.h>
#include <sys/qos.h>
#include <sys/resource.h>
#include <sys/sysctl.h>
#include <unistd.h>

namespace kopenapi::platform
{

std::filesystem::path TempRootMacOS()
{
    return "/tmp";
}

std::filesystem::path ExecutablePathMacOS()
{
    char buffer[PATH_MAX];
    uint32_t size = sizeof(buffer);
    if (_NSGetExecutablePath(buffer, &size) != 0)
    {
        return {};
    }
    std::error_code ec;
    std::filesystem::path exe = std::filesystem::canonical(buffer, ec);
    return ec ? std::filesystem::path() : exe;
}

SystemMemoryInfo SystemMemoryMacOS()
{
    SystemMemoryInfo info;
    uint64_t         memsize = 0;
    size_t           len = sizeof( memsize );

    if( sysctlbyname( "hw.memsize", &memsize, &len, nullptr, 0 ) == 0 )
        info.total = (int64_t) memsize;

    vm_statistics64_data_t vm{};
    mach_msg_type_number_t count = HOST_VM_INFO64_COUNT;

    if( host_statistics64( mach_host_self(), HOST_VM_INFO64, (host_info64_t) &vm, &count ) == KERN_SUCCESS )
    {
        vm_size_t page = 0;
        host_page_size( mach_host_self(), &page );
        info.available = (int64_t) ( vm.free_count + vm.inactive_count + vm.purgeable_count ) * (int64_t) page;
    }

    // the kernel's own view: 1 normal, 2 warning, 4 critical
    int level = 0;
    len = sizeof( level );

    if( sysctlbyname( "kern.memorystatus_vm_pressure_level", &level, &len, nullptr, 0 ) == 0 )
        info.pressure = level >= 4 ? 2 : level >= 2 ? 1 : 0;

    return info;
}

ProcessMemoryInfo ProcessMemoryMacOS()
{
    ProcessMemoryInfo info;
    info.footprintKind = "phys_footprint";

    task_vm_info_data_t    vm{};
    mach_msg_type_number_t count = TASK_VM_INFO_COUNT;

    if (task_info(mach_task_self(), TASK_VM_INFO, (task_info_t) &vm, &count) == KERN_SUCCESS)
    {
        info.rss = (int64_t) vm.resident_size;
        info.peakRss = (int64_t) vm.resident_size_peak;
        info.footprint = (int64_t) vm.phys_footprint;
        info.virtualSize = (int64_t) vm.virtual_size;
        info.compressed = (int64_t) vm.compressed;
    }

    rusage_info_v4 usage{};

    if (proc_pid_rusage(getpid(), RUSAGE_INFO_V4, (rusage_info_t*) &usage) == 0)
    {
        info.peakFootprint = (int64_t) usage.ri_lifetime_max_phys_footprint;
    }

    proc_taskinfo task{};

    if (proc_pidinfo(getpid(), PROC_PIDTASKINFO, 0, &task, sizeof(task)) == (int) sizeof(task))
    {
        info.threads = task.pti_threadnum;
    }

    return info;
}

AllocatorInfo AllocatorStatsMacOS()
{
    AllocatorInfo info;
    info.allocator = "macos-malloc";

    vm_address_t* zones = nullptr;
    unsigned      count = 0;

    if (malloc_get_all_zones(mach_task_self(), nullptr, &zones, &count) != KERN_SUCCESS)
    {
        return info;
    }

    info.live = info.mapped = info.peakLive = 0;

    for (unsigned i = 0; i < count; ++i)
    {
        malloc_statistics_t stats{};
        malloc_zone_statistics((malloc_zone_t*) zones[i], &stats);
        // a zone can report less reserved (or a lower peak) than it has in use: large blocks are
        // counted in size_in_use only
        info.live += (int64_t) stats.size_in_use;
        info.mapped += (int64_t) std::max(stats.size_allocated, stats.size_in_use);
        info.peakLive += (int64_t) std::max(stats.max_size_in_use, stats.size_in_use);
    }

    return info;
}

int64_t AllocatorTrimMacOS()
{
    return (int64_t) malloc_zone_pressure_relief(nullptr, 0);
}

void LowerCurrentThreadPriorityMacOS()
{
    pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
}

} // namespace kopenapi::platform

#endif // __APPLE__
