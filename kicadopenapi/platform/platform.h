#pragma once

// kicadopenapi platform layer — shared by the in-process service (kicommon) and
// kicad-mcp-bridge. Generic entry points live in platform.cpp and dispatch to one
// implementation per OS (platform_windows / platform_posix / platform_macos /
// platform_linux), each file compiled only on its platform. No KiCad or wx dependencies.

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace kopenapi::platform
{

/// Root for KiCad runtime files: /tmp on macOS (as the IPC API does), the OS temp dir elsewhere
std::filesystem::path TempRoot();

/// <TempRoot>/kicad/openapi — one <pid>.json per running kicadopenapi service
std::filesystem::path DiscoveryDir();

/// Directory of the running executable (argv0 is the fallback)
std::filesystem::path ExecutableDir(const char* argv0);

/// "kicad-cli" → "kicad-cli.exe" on Windows
std::string ExecutableName(const std::string& base);

long CurrentPid();

/// Process exists and has not exited (on POSIX also collects our own exited children)
bool ProcessAlive(long pid);

/// Starts argv[0] with argv detached from our stdio: stdin from the null device, stdout and
/// stderr appended to logFile, own process group, default signal handling. Returns the pid,
/// or 0 with error set.
long SpawnDetached(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                   std::string& error);

/// Hard stop: SIGTERM on POSIX, TerminateProcess on Windows. Async-signal-safe on POSIX.
void KillProcess(long pid);

/**
 * Make a TCP listening socket exclusive before bind(): a second process must fail to bind the
 * same port (so port probing works).  POSIX: SO_REUSEADDR (TIME_WAIT reuse only; a live
 * listener still blocks the port).  Windows: SO_EXCLUSIVEADDRUSE (SO_REUSEADDR there would
 * let another process take over a live port).
 */
void ConfigureListenSocket(std::uintptr_t socket);

/// Calls handler (must be async-signal-safe) on SIGINT/SIGTERM/SIGHUP or console close /
/// Ctrl-C; the process then exits. Also makes writes to a closed pipe non-fatal.
void InstallTerminationHandler(void (*handler)());

} // namespace kopenapi::platform
