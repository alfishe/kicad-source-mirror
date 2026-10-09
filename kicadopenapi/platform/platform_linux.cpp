#if defined(__linux__)

#include "platform_linux.h"

namespace kopenapi::platform
{

std::filesystem::path ExecutablePathLinux()
{
    std::error_code ec;
    std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    return ec ? std::filesystem::path() : exe;
}

} // namespace kopenapi::platform

#endif // __linux__
