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
AllocatorInfo AllocatorStatsLinux();
int64_t AllocatorTrimLinux();
void LowerCurrentThreadPriorityLinux();

} // namespace kopenapi::platform

#endif // __linux__
