#if defined(__APPLE__)

#include "platform_macos.h"

#include <climits>
#include <cstdint>
#include <mach-o/dyld.h>

namespace kopenapi::platform
{

std::filesystem::path TempRootMacOS()
{
    return "/tmp";
}

std::filesystem::path ExecutablePathMacOS()
{
    char buffer[PATH_MAX];
    uint32_t size = sizeof(buffer);
    if (_NSGetExecutablePath(buffer, &size) != 0)
    {
        return {};
    }
    std::error_code ec;
    std::filesystem::path exe = std::filesystem::canonical(buffer, ec);
    return ec ? std::filesystem::path() : exe;
}

} // namespace kopenapi::platform

#endif // __APPLE__
