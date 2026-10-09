// Generic platform entry points — dispatch to the per-OS implementations

#include "platform.h"

#if defined(_WIN32)
#include "platform_windows.h"
#else
#include "platform_posix.h"
#endif

#if defined(__APPLE__)
#include "platform_macos.h"
#elif defined(__linux__)
#include "platform_linux.h"
#endif

namespace fs = std::filesystem;

namespace kopenapi::platform
{

fs::path TempRoot()
{
#if defined(__APPLE__)
    return TempRootMacOS();
#else
    std::error_code ec;
    fs::path tmp = fs::temp_directory_path(ec);
    return ec ? fs::path("/tmp") : tmp;
#endif
}

fs::path DiscoveryDir()
{
    return TempRoot() / "kicad" / "openapi";
}

fs::path ExecutableDir(const char* argv0)
{
    fs::path exe;
#if defined(_WIN32)
    exe = ExecutablePathWindows();
#elif defined(__APPLE__)
    exe = ExecutablePathMacOS();
#elif defined(__linux__)
    exe = ExecutablePathLinux();
#endif
    if (exe.empty())
    {
        std::error_code ec;
        exe = fs::absolute(argv0 ? argv0 : "", ec);
    }
    return exe.parent_path();
}

std::string ExecutableName(const std::string& base)
{
#if defined(_WIN32)
    return base + ".exe";
#else
    return base;
#endif
}

long CurrentPid()
{
#if defined(_WIN32)
    return CurrentPidWindows();
#else
    return CurrentPidPosix();
#endif
}

bool ProcessAlive(long pid)
{
    if (pid <= 0)
    {
        return false;
    }
#if defined(_WIN32)
    return ProcessAliveWindows(pid);
#else
    return ProcessAlivePosix(pid);
#endif
}

long SpawnDetached(const std::vector<std::string>& argv, const fs::path& logFile, std::string& error)
{
    if (argv.empty())
    {
        error = "empty command";
        return 0;
    }
#if defined(_WIN32)
    return SpawnDetachedWindows(argv, logFile, error);
#else
    return SpawnDetachedPosix(argv, logFile, error);
#endif
}

void KillProcess(long pid)
{
    if (pid <= 0)
    {
        return;
    }
#if defined(_WIN32)
    KillProcessWindows(pid);
#else
    KillProcessPosix(pid);
#endif
}

void ConfigureListenSocket(std::uintptr_t socket)
{
#if defined(_WIN32)
    ConfigureListenSocketWindows(socket);
#else
    ConfigureListenSocketPosix(socket);
#endif
}

void InstallTerminationHandler(void (*handler)())
{
#if defined(_WIN32)
    InstallTerminationHandlerWindows(handler);
#else
    InstallTerminationHandlerPosix(handler);
#endif
}

} // namespace kopenapi::platform
