#pragma once

// KiCad instance discovery, selection, and lifecycle for kicad-mcp-bridge. Platform-neutral;
// OS specifics live in kicadopenapi/platform.
//
// Every kicadopenapi service writes <tmp>/kicad/openapi/<pid>.json while it runs; this
// module reads those files, applies selection policies, and starts/stops KiCad processes.

#include <chrono>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bridge
{

struct Instance
{
    long pid = 0;
    int port = 0;
    bool headless = false;
    std::string app;
    std::string url;     // http://127.0.0.1:<port>
    std::string mcpUrl;  // http://127.0.0.1:<port>/mcp
    std::filesystem::file_time_type started;
};

/// Live instances (files of dead processes are ignored)
std::vector<Instance> Discover(const std::filesystem::path& dir);

std::optional<Instance> FindByPid(const std::filesystem::path& dir, long pid);

/// Policy: newest | oldest | gui | headless | pid:<n> | port:<n>
std::optional<Instance> Select(const std::vector<Instance>& instances, const std::string& policy);

/// Waits until the process publishes its discovery file; nullopt if it exits or times out
std::optional<Instance> WaitForInstance(const std::filesystem::path& dir, long pid,
                                        std::chrono::seconds timeout);

/// Graceful stop: POST <url>/api/v1/shutdown, then a hard kill if still alive after grace
void Stop(const Instance& inst, std::chrono::seconds grace);

/// Deletes regular files in dir older than maxAge (old child logs)
void PruneOldFiles(const std::filesystem::path& dir, std::chrono::hours maxAge);

} // namespace bridge
