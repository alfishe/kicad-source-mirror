#if !defined(_WIN32)

#include "platform_posix.h"
#include "platform.h"

#include <cstdlib>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <thread>

extern char** environ;

namespace kopenapi::platform
{

static void (*s_terminationHandler)() = nullptr;

extern "C" void kopenapiOnTerminationSignal(int sig)
{
    if (s_terminationHandler)
    {
        s_terminationHandler();
    }
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

long CurrentPidPosix()
{
    return static_cast<long>(getpid());
}

void SetEnvPosix(const std::string& name, const std::string& value)
{
    setenv(name.c_str(), value.c_str(), 1);
}

bool ProcessAlivePosix(long pid)
{
    // Our own exited children stay zombies (kill() succeeds on them) until collected
    int status = 0;
    if (waitpid(static_cast<pid_t>(pid), &status, WNOHANG) == static_cast<pid_t>(pid))
    {
        return false;
    }
    return kill(static_cast<pid_t>(pid), 0) == 0 || errno == EPERM;
}

long SpawnDetachedPosix(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                        std::string& error)
{
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, logFile.c_str(), O_WRONLY | O_CREAT | O_APPEND,
                                     0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);

    // Own process group: terminal signals aimed at us do not hit the child. Default signal
    // dispositions: we may ignore SIGPIPE, the child must not inherit that.
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    sigset_t defaults;
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGPIPE);
    sigaddset(&defaults, SIGINT);
    sigaddset(&defaults, SIGTERM);
    sigaddset(&defaults, SIGHUP);
    posix_spawnattr_setsigdefault(&attr, &defaults);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGDEF);
    posix_spawnattr_setpgroup(&attr, 0);

    std::vector<char*> args;
    for (const std::string& arg : argv)
    {
        args.push_back(const_cast<char*>(arg.c_str()));
    }
    args.push_back(nullptr);

    pid_t pid = 0;
    const int rc = posix_spawn(&pid, argv[0].c_str(), &actions, &attr, args.data(), environ);

    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attr);

    if (rc != 0)
    {
        error = "cannot start " + argv[0] + ": " + std::strerror(rc);
        return 0;
    }
    return static_cast<long>(pid);
}

void KillProcessPosix(long pid)
{
    kill(static_cast<pid_t>(pid), SIGTERM);
}

void ConfigureListenSocketPosix(std::uintptr_t socket)
{
    int yes = 1;
    setsockopt(static_cast<int>(socket), SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
}

void InstallTerminationHandlerPosix(void (*handler)())
{
    s_terminationHandler = handler;
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, kopenapiOnTerminationSignal);
    std::signal(SIGTERM, kopenapiOnTerminationSignal);
    std::signal(SIGHUP, kopenapiOnTerminationSignal);
}

PipeProcess SpawnWithStdinPipePosix(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                                    std::string& error)
{
    int fds[2];
    if (pipe(fds) != 0)
    {
        error = std::string("pipe: ") + std::strerror(errno);
        return {};
    }
    fcntl(fds[1], F_SETFD, FD_CLOEXEC);   // the write end stays ours only

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, fds[0], STDIN_FILENO);
    posix_spawn_file_actions_addclose(&actions, fds[0]);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, logFile.c_str(), O_WRONLY | O_CREAT | O_APPEND,
                                     0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);

    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    sigset_t defaults;
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGPIPE);
    sigaddset(&defaults, SIGINT);
    sigaddset(&defaults, SIGTERM);
    sigaddset(&defaults, SIGHUP);
    posix_spawnattr_setsigdefault(&attr, &defaults);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGDEF);
    posix_spawnattr_setpgroup(&attr, 0);

    std::vector<char*> args;
    for (const std::string& arg : argv)
    {
        args.push_back(const_cast<char*>(arg.c_str()));
    }
    args.push_back(nullptr);

    pid_t pid = 0;
    const int rc = posix_spawn(&pid, argv[0].c_str(), &actions, &attr, args.data(), environ);

    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attr);
    close(fds[0]);

    if (rc != 0)
    {
        close(fds[1]);
        error = "cannot start " + argv[0] + ": " + std::strerror(rc);
        return {};
    }
    return {static_cast<long>(pid), static_cast<std::intptr_t>(fds[1])};
}

bool WritePipePosix(std::intptr_t handle, const void* data, std::size_t size)
{
    const char* p = static_cast<const char*>(data);
    while (size > 0)
    {
        const ssize_t n = write(static_cast<int>(handle), p, size);
        if (n < 0 && errno == EINTR)
        {
            continue;
        }
        if (n <= 0)
        {
            return false;   // EPIPE: our hosts ignore SIGPIPE
        }
        p += n;
        size -= static_cast<std::size_t>(n);
    }
    return true;
}

void ClosePipePosix(std::intptr_t handle)
{
    close(static_cast<int>(handle));
}

int WaitProcessPosix(long pid, int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    for (;;)
    {
        int status = 0;
        const pid_t r = waitpid(static_cast<pid_t>(pid), &status, WNOHANG);
        if (r == static_cast<pid_t>(pid))
        {
            return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
        }
        if (r < 0)
        {
            return errno == ECHILD ? 0 : -1;   // collected elsewhere already
        }
        if (std::chrono::steady_clock::now() >= deadline)
        {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace kopenapi::platform

#endif // !_WIN32
