#pragma once
#if defined(_WIN32)

// Windows implementations of the kicadopenapi platform layer

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace kopenapi::platform
{

std::filesystem::path ExecutablePathWindows();
long CurrentPidWindows();
bool ProcessAliveWindows(long pid);
void SetEnvWindows(const std::string& name, const std::string& value);
long SpawnDetachedWindows(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                          std::string& error);
void KillProcessWindows(long pid);
void ConfigureListenSocketWindows(std::uintptr_t socket);
void InstallTerminationHandlerWindows(void (*handler)());
void InstallStopRequestHandlerWindows(void (*handler)());
struct PipeProcess;
PipeProcess SpawnWithStdinPipeWindows(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                                      std::string& error);
bool WritePipeWindows(std::intptr_t handle, const void* data, std::size_t size);
void ClosePipeWindows(std::intptr_t handle);
int WaitProcessWindows(long pid, int timeoutMs);

struct ProcessMemoryInfo;
struct AllocatorInfo;
ProcessMemoryInfo ProcessMemoryWindows();
struct SystemMemoryInfo;
SystemMemoryInfo SystemMemoryWindows();
AllocatorInfo AllocatorStatsWindows();
int64_t AllocatorTrimWindows();
void LowerCurrentThreadPriorityWindows();
void KeepProcessResponsiveWindows();

/// @brief Runs and deletes *body on a new thread; the handle for JoinThreadWindows, nullptr on failure
void* StartThreadWindows(std::size_t stackBytes, std::function<void()>* body);
void JoinThreadWindows(void* handle);

} // namespace kopenapi::platform

#endif // _WIN32
