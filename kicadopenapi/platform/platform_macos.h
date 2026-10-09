#pragma once
#if defined(__APPLE__)

// macOS-specific parts of the kicadopenapi platform layer

#include <filesystem>

namespace kopenapi::platform
{

/// /tmp: $TMPDIR is per-user and may be missing in sanitized environments (MCP clients)
std::filesystem::path TempRootMacOS();

std::filesystem::path ExecutablePathMacOS();

} // namespace kopenapi::platform

#endif // __APPLE__
