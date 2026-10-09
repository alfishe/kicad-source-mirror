#if !defined(_WIN32)

#include "platform_posix.h"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

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

void InstallTerminationHandlerPosix(void (*handler)())
{
    s_terminationHandler = handler;
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, kopenapiOnTerminationSignal);
    std::signal(SIGTERM, kopenapiOnTerminationSignal);
    std::signal(SIGHUP, kopenapiOnTerminationSignal);
}

} // namespace kopenapi::platform

#endif // !_WIN32
