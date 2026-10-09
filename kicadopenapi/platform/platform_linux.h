#pragma once
#if defined(__linux__)

// Linux-specific parts of the kicadopenapi platform layer

#include <filesystem>

namespace kopenapi::platform
{

std::filesystem::path ExecutablePathLinux();

} // namespace kopenapi::platform

#endif // __linux__
