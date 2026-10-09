#pragma once
#if defined(_WIN32)

// Windows implementations of the kicadopenapi platform layer

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace kopenapi::platform
{

std::filesystem::path ExecutablePathWindows();
long CurrentPidWindows();
bool ProcessAliveWindows(long pid);
long SpawnDetachedWindows(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                          std::string& error);
void KillProcessWindows(long pid);
void ConfigureListenSocketWindows(std::uintptr_t socket);
void InstallTerminationHandlerWindows(void (*handler)());

} // namespace kopenapi::platform

#endif // _WIN32
