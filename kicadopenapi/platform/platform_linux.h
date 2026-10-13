#pragma once
#if defined(__linux__)

// Linux-specific parts of the kicadopenapi platform layer

#include <cstdint>
#include <filesystem>

namespace kopenapi::platform
{

std::filesystem::path ExecutablePathLinux();

struct ProcessMemoryInfo;
struct AllocatorInfo;
ProcessMemoryInfo ProcessMemoryLinux();
struct SystemMemoryInfo;
SystemMemoryInfo SystemMemoryLinux();
AllocatorInfo AllocatorStatsLinux();
int64_t AllocatorTrimLinux();
void LowerCurrentThreadPriorityLinux();

/// jemalloc (linked or preloaded), found through mallctl at run time
bool JemallocActive();
bool JemallocStats( AllocatorInfo& aInfo );
bool JemallocTrim();

} // namespace kopenapi::platform

#endif // __linux__
