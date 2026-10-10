#pragma once
#if defined(_WIN32)

// Windows implementations of the kicadopenapi platform layer

#include <cstddef>
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
struct PipeProcess;
PipeProcess SpawnWithStdinPipeWindows(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                                      std::string& error);
bool WritePipeWindows(std::intptr_t handle, const void* data, std::size_t size);
void ClosePipeWindows(std::intptr_t handle);
int WaitProcessWindows(long pid, int timeoutMs);

} // namespace kopenapi::platform

#endif // _WIN32
