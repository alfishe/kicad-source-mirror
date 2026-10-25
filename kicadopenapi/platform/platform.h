#pragma once

// kicadopenapi platform layer — shared by the in-process service (kicommon) and
// kicad-mcp-bridge. Generic entry points live in platform.cpp and dispatch to one
// implementation per OS (platform_windows / platform_posix / platform_macos /
// platform_linux), each file compiled only on its platform. No KiCad or wx dependencies.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
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

/// @brief Memory the OS charges this process for, in bytes; -1 where the OS does not report a field
struct ProcessMemoryInfo
{
    int64_t     rss = -1;             ///< resident set / working set
    int64_t     footprint = -1;       ///< what memory pressure counts (see footprintKind)
    int64_t     peakRss = -1;
    int64_t     peakFootprint = -1;
    int64_t     virtualSize = -1;
    int64_t     compressed = -1;      ///< macOS: compressed pages of this process
    int64_t     swapped = -1;         ///< Linux: VmSwap
    int         threads = -1;
    const char* footprintKind = "";   ///< phys_footprint (macOS), rss_anon+swap (Linux), private_usage (Windows)
};

/// @brief A few microseconds; safe on any thread
ProcessMemoryInfo ProcessMemory();

/// @brief The machine's memory: physical RAM, what is still available, and the OS's pressure level
struct SystemMemoryInfo
{
    int64_t total = -1;       ///< physical RAM (bytes)
    int64_t available = -1;   ///< free + reclaimable (bytes), -1 when unknown
    int     pressure = 0;     ///< 0 normal, 1 warning, 2 critical
};

/// @brief A few microseconds; safe on any thread
SystemMemoryInfo SystemMemory();

/// @brief The system allocator: bytes in live blocks and bytes it holds from the OS; -1 when unknown
struct AllocatorInfo
{
    const char* allocator = "unknown";   ///< macos-malloc, glibc, jemalloc, windows-heap
    int64_t     live = -1;
    int64_t     mapped = -1;
    int64_t     peakLive = -1;
};

/// @brief Takes the allocator's locks briefly (microseconds to a millisecond); safe on any thread
AllocatorInfo AllocatorStats();

/// @brief Gives memory the allocator holds but does not use back to the OS. Pauses the calling
/// thread (macOS ~20 ms + 0.1 ms per MB released, glibc ~1 ms + 0.1 ms per MB; POC-001) and briefly
/// blocks allocations on other threads.
/// @return bytes released where the allocator reports it (macOS), else -1
int64_t AllocatorTrim();

/// @brief Runs the calling thread at background priority (macOS QoS utility, Linux nice 10,
/// Windows below normal)
void LowerCurrentThreadPriority();

/// @brief A thread with a stack of the given size: std::thread takes the platform default (512 KB on
/// macOS), too small for KiCad code written for the main thread's 8 MB.  Joined on destruction
class THREAD
{
public:
    THREAD() = default;
    THREAD(std::size_t stackBytes, std::function<void()> body);
    ~THREAD();

    THREAD(const THREAD&) = delete;
    THREAD& operator=(const THREAD&) = delete;

    /// @brief false when the thread could not be started (body not run)
    bool Started() const { return m_handle != nullptr; }

    void Join();

private:
    void* m_handle = nullptr;
};

/// @brief Keeps the OS from throttling this process while its windows are hidden or in the
/// background (macOS App Nap, Windows power throttling): an agent drives a KiCad that is rarely
/// frontmost.  Idempotent; idle system sleep stays allowed.  No-op on Linux.
void KeepProcessResponsive();

/// @brief The program a process should be started from so that rebuilding the original does not
/// pull files from under it: on macOS a clone of exe's .app bundle (APFS clonefile, no data
/// copied) in <bundle dir>/run-snapshots/<stamp>/, made once per build (stamp: the bundle's
/// executables, plug-ins and frameworks by size and time), and exe's path inside it; snapshots
/// older than a week beyond the newest three are removed.  exe itself where there is no bundle or
/// the clone fails (note says why).
std::filesystem::path RunSnapshot(const std::filesystem::path& exe, std::string& note);

} // namespace kopenapi::platform
