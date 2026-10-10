#pragma once
#if !defined(_WIN32)

// POSIX (macOS + Linux) implementations of the kicadopenapi platform layer

#include <cstddef>
#include <cstdint>
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
void ConfigureListenSocketPosix(std::uintptr_t socket);
void InstallTerminationHandlerPosix(void (*handler)());
struct PipeProcess;
PipeProcess SpawnWithStdinPipePosix(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                                    std::string& error);
bool WritePipePosix(std::intptr_t handle, const void* data, std::size_t size);
void ClosePipePosix(std::intptr_t handle);
int WaitProcessPosix(long pid, int timeoutMs);

} // namespace kopenapi::platform

#endif // !_WIN32
