#pragma once

// kicadopenapi platform layer — shared by the in-process service (kicommon) and
// kicad-mcp-bridge. Generic entry points live in platform.cpp and dispatch to one
// implementation per OS (platform_windows / platform_posix / platform_macos /
// platform_linux), each file compiled only on its platform. No KiCad or wx dependencies.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace kopenapi::platform
{

/// @brief Root for KiCad runtime files: /tmp on macOS (as the IPC API does), the OS temp dir elsewhere
std::filesystem::path TempRoot();

/// @brief <TempRoot>/kicad/openapi — one <pid>.json per running kicadopenapi service
std::filesystem::path DiscoveryDir();

/// @brief Directory of the running executable (argv0 is the fallback)
std::filesystem::path ExecutableDir(const char* argv0);

/// @brief "kicad-cli" → "kicad-cli.exe" on Windows
std::string ExecutableName(const std::string& base);

long CurrentPid();

/// @brief Process exists and has not exited (on POSIX also collects our own exited children)
bool ProcessAlive(long pid);

/// @brief Sets an environment variable of this process (inherited by processes it starts)
void SetEnv(const std::string& name, const std::string& value);

/// @brief Starts argv[0] with argv detached from our stdio: stdin from the null device, stdout and
/// stderr appended to logFile, own process group, default signal handling. Returns the pid,
/// or 0 with error set.
long SpawnDetached(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                   std::string& error);

/// @brief Hard stop: SIGTERM on POSIX, TerminateProcess on Windows. Async-signal-safe on POSIX.
void KillProcess(long pid);

/// @brief Make a TCP listening socket exclusive before bind(): a second process must fail to bind the
/// same port (so port probing works).  POSIX: SO_REUSEADDR (TIME_WAIT reuse only; a live
/// listener still blocks the port).  Windows: SO_EXCLUSIVEADDRUSE (SO_REUSEADDR there would
/// let another process take over a live port).
void ConfigureListenSocket(std::uintptr_t socket);

/// @brief Calls handler (must be async-signal-safe) on SIGINT/SIGTERM/SIGHUP or console close /
/// Ctrl-C; the process then exits. Also makes writes to a closed pipe non-fatal.
void InstallTerminationHandler(void (*handler)());

/// @brief A child process fed through its stdin (e.g. a video encoder)
struct PipeProcess
{
    long          pid = 0;
    std::intptr_t stdinHandle = -1;   ///< write end of the child's stdin; -1 when not running
};

/// @brief Starts argv[0] with stdin from a new pipe, stdout and stderr appended to logFile, own process
/// group, default signal handling.
/// @return pid and the pipe's write end; pid 0 with error set on failure
PipeProcess SpawnWithStdinPipe(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                               std::string& error);

/// @brief Writes all bytes to a pipe.
/// @return false when the reader is gone
bool WritePipe(std::intptr_t handle, const void* data, std::size_t size);

/// @brief Closes a pipe end (the reader sees end of file)
void ClosePipe(std::intptr_t handle);

/// @brief Waits for a child to exit.
/// @return its exit code, or -1 when it still runs after timeoutMs
int WaitProcess(long pid, int timeoutMs);

/// @brief Finds an executable on PATH and in the OS's usual tool directories (GUI apps on macOS
/// do not get the shell's PATH: /opt/homebrew/bin, /usr/local/bin are searched too).
/// @return the full path, or empty
std::filesystem::path FindExecutable(const std::string& name);

} // namespace kopenapi::platform
