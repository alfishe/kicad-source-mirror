#pragma once
#if !defined(_WIN32)

// POSIX (macOS + Linux) implementations of the kicadopenapi platform layer

#include <filesystem>
#include <string>
#include <vector>

namespace kopenapi::platform
{

long CurrentPidPosix();
bool ProcessAlivePosix(long pid);
long SpawnDetachedPosix(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                        std::string& error);
void KillProcessPosix(long pid);
void InstallTerminationHandlerPosix(void (*handler)());

} // namespace kopenapi::platform

#endif // !_WIN32
