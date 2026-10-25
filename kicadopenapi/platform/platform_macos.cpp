#if defined(__APPLE__)

#include "platform_macos.h"
#include "platform.h"

#include <algorithm>
#include <climits>
#include <dlfcn.h>
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
#include <sys/clonefile.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

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

void KeepProcessResponsiveMacOS()
{
    // -[NSProcessInfo beginActivityWithOptions:reason:] through the Objective-C runtime: this file
    // is plain C++ and the bridge links no Foundation (there the lookups fail and nothing happens)
    static bool held = false;

    if (held)
        return;

    using GetClass = void* (*)(const char*);
    using RegisterName = void* (*)(const char*);
    using PoolPush = void* (*)();
    using PoolPop = void (*)(void*);

    auto getClass = reinterpret_cast<GetClass>(dlsym(RTLD_DEFAULT, "objc_getClass"));
    auto selector = reinterpret_cast<RegisterName>(dlsym(RTLD_DEFAULT, "sel_registerName"));
    auto poolPush = reinterpret_cast<PoolPush>(dlsym(RTLD_DEFAULT, "objc_autoreleasePoolPush"));
    auto poolPop = reinterpret_cast<PoolPop>(dlsym(RTLD_DEFAULT, "objc_autoreleasePoolPop"));
    void* msgSend = dlsym(RTLD_DEFAULT, "objc_msgSend");

    if (!getClass || !selector || !poolPush || !poolPop || !msgSend)
        return;

    void* processInfoClass = getClass("NSProcessInfo");
    void* stringClass = getClass("NSString");

    if (!processInfoClass || !stringClass)
        return;

    // NSActivityUserInitiatedAllowingIdleSystemSleep
    constexpr uint64_t options = 0x00FFFFFFULL & ~(1ULL << 20);

    void* pool = poolPush();
    void* processInfo = reinterpret_cast<void* (*)(void*, void*)>(msgSend)(processInfoClass, selector("processInfo"));
    void* reason = reinterpret_cast<void* (*)(void*, void*, const char*)>(msgSend)(
            stringClass, selector("stringWithUTF8String:"), "kicadopenapi: driven by an agent");
    void* activity = processInfo ? reinterpret_cast<void* (*)(void*, void*, uint64_t, void*)>(msgSend)(
                                           processInfo, selector("beginActivityWithOptions:reason:"), options, reason)
                                 : nullptr;

    // held for the process's lifetime
    if (activity)
    {
        reinterpret_cast<void* (*)(void*, void*)>(msgSend)(activity, selector("retain"));
        held = true;
    }

    poolPop(pool);
}

std::filesystem::path RunSnapshotMacOS(const std::filesystem::path& exe, std::string& note)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path        bundle;

    for (fs::path p = exe.parent_path(); !p.empty() && p != p.root_path(); p = p.parent_path())
    {
        if (p.extension() == ".app")
        {
            bundle = p;
            break;
        }
    }

    if (bundle.empty())
    {
        note = "not in an .app bundle: runs as is";
        return exe;
    }

    // FNV-1a over the program files' names, sizes and times
    uint64_t h = 1469598103934665603ull;
    auto     mix = [&](const std::string& text)
    {
        for (unsigned char c : text)
            h = (h ^ c) * 1099511628211ull;
    };

    for (const char* dir : {"Contents/MacOS", "Contents/PlugIns", "Contents/Frameworks"})
    {
        std::vector<std::string> entries;

        for (fs::recursive_directory_iterator it(bundle / dir, ec), end; !ec && it != end; it.increment(ec))
        {
            if (!it->is_regular_file(ec))
                continue;

            const long long when = (long long) it->last_write_time(ec).time_since_epoch().count();
            entries.push_back(it->path().lexically_relative(bundle).string() + ":" + std::to_string((unsigned long long) it->file_size(ec))
                              + ":" + std::to_string(when));
        }

        std::sort(entries.begin(), entries.end());

        for (const std::string& e : entries)
            mix(e);
    }

    char stamp[17];
    std::snprintf(stamp, sizeof(stamp), "%016llx", (unsigned long long) h);

    const fs::path root = bundle.parent_path() / "run-snapshots";
    const fs::path home = root / stamp;
    const fs::path copy = home / bundle.filename();
    const fs::path target = copy / exe.lexically_relative(bundle);

    if (fs::exists(home / "complete", ec) && fs::exists(target, ec))
    {
        note = "runs from snapshot " + home.string();
        return target;
    }

    fs::create_directories(root, ec);
    const fs::path partial = root / (std::string(stamp) + ".partial-" + std::to_string(getpid()));
    fs::remove_all(partial, ec);
    fs::create_directories(partial, ec);

    if (clonefile(bundle.c_str(), (partial / bundle.filename()).c_str(), 0) != 0)
    {
        note = "snapshot failed (clonefile errno " + std::to_string(errno) + "): runs from the build";
        fs::remove_all(partial, ec);
        return exe;
    }

    if (FILE* f = std::fopen((partial / "complete").c_str(), "w"))
        std::fclose(f);

    fs::rename(partial, home, ec);

    if (ec)   // another process made it meanwhile
        fs::remove_all(partial, ec);

    // older snapshots: keep the newest three, and any younger than a week (a process may run from it)
    std::vector<std::pair<fs::file_time_type, fs::path>> old;

    for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec))
    {
        if (it->is_directory(ec) && it->path() != home)
            old.emplace_back(it->last_write_time(ec), it->path());
    }

    std::sort(old.begin(), old.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    const auto weekAgo = fs::file_time_type::clock::now() - std::chrono::hours(24 * 7);

    const auto hourAgo = fs::file_time_type::clock::now() - std::chrono::hours(1);

    for (size_t k = 0; k < old.size(); ++k)
    {
        const bool partialLeft = old[k].second.filename().string().find(".partial-") != std::string::npos
                                 && old[k].first < hourAgo;

        if (partialLeft || (k >= 2 && old[k].first < weekAgo))
            fs::remove_all(old[k].second, ec);
    }

    note = "runs from snapshot " + home.string();
    return fs::exists(target, ec) ? target : exe;
}

} // namespace kopenapi::platform

#endif // __APPLE__
