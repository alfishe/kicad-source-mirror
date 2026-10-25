#pragma once
#if defined(__APPLE__)

// macOS-specific parts of the kicadopenapi platform layer

#include <cstdint>
#include <filesystem>
#include <string>

namespace kopenapi::platform
{

/// @brief /tmp: $TMPDIR is per-user and may be missing in sanitized environments (MCP clients)
std::filesystem::path TempRootMacOS();

std::filesystem::path ExecutablePathMacOS();

struct ProcessMemoryInfo;
struct AllocatorInfo;
ProcessMemoryInfo ProcessMemoryMacOS();
struct SystemMemoryInfo;
SystemMemoryInfo SystemMemoryMacOS();
AllocatorInfo AllocatorStatsMacOS();
int64_t AllocatorTrimMacOS();
void LowerCurrentThreadPriorityMacOS();
void KeepProcessResponsiveMacOS();
std::filesystem::path RunSnapshotMacOS(const std::filesystem::path& exe, std::string& note);

} // namespace kopenapi::platform

#endif // __APPLE__
