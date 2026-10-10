// Generic platform entry points — dispatch to the per-OS implementations

#include "platform.h"

#include <algorithm>
#include <cstdlib>

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

PipeProcess SpawnWithStdinPipe(const std::vector<std::string>& argv, const fs::path& logFile, std::string& error)
{
    if (argv.empty())
    {
        error = "empty command";
        return {};
    }
#if defined(_WIN32)
    return SpawnWithStdinPipeWindows(argv, logFile, error);
#else
    return SpawnWithStdinPipePosix(argv, logFile, error);
#endif
}

bool WritePipe(std::intptr_t handle, const void* data, std::size_t size)
{
    if (handle == -1)
    {
        return false;
    }
#if defined(_WIN32)
    return WritePipeWindows(handle, data, size);
#else
    return WritePipePosix(handle, data, size);
#endif
}

void ClosePipe(std::intptr_t handle)
{
    if (handle == -1)
    {
        return;
    }
#if defined(_WIN32)
    ClosePipeWindows(handle);
#else
    ClosePipePosix(handle);
#endif
}

int WaitProcess(long pid, int timeoutMs)
{
    if (pid <= 0)
    {
        return -1;
    }
#if defined(_WIN32)
    return WaitProcessWindows(pid, timeoutMs);
#else
    return WaitProcessPosix(pid, timeoutMs);
#endif
}

fs::path FindExecutable(const std::string& name)
{
    std::vector<fs::path> dirs;
#if defined(_WIN32)
    const char separator = ';';
#else
    const char separator = ':';
#endif
    if (const char* path = std::getenv("PATH"))
    {
        std::string all = path;
        size_t start = 0;
        while (start <= all.size())
        {
            const size_t end = std::min(all.find(separator, start), all.size());
            if (end > start)
            {
                dirs.emplace_back(all.substr(start, end - start));
            }
            start = end + 1;
        }
    }
#if defined(__APPLE__)
    dirs.emplace_back("/opt/homebrew/bin");
    dirs.emplace_back("/usr/local/bin");
#elif defined(__linux__)
    dirs.emplace_back("/usr/local/bin");
    dirs.emplace_back("/usr/bin");
#endif
    for (const fs::path& dir : dirs)
    {
        std::error_code ec;
        const fs::path candidate = dir / ExecutableName(name);
        if (fs::is_regular_file(candidate, ec))
        {
            return candidate;
        }
    }
    return {};
}

} // namespace kopenapi::platform
