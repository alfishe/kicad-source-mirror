// KiCad instance discovery and lifecycle — see instances.h

#include "instances.h"
#include "mini-http.h"

#include <platform.h>

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <fstream>
#include <thread>

namespace fs = std::filesystem;
namespace platform = kopenapi::platform;

namespace bridge
{

static std::optional<Instance> ReadInstance(const fs::path& file)
{
    std::ifstream in(file);
    if (!in)
    {
        return std::nullopt;
    }
    const nlohmann::json info = nlohmann::json::parse(in, nullptr, false);
    if (info.is_discarded() || !info.is_object())
    {
        return std::nullopt;
    }

    // a stranger's or a broken file in the shared directory must not end the bridge
    for (const char* key : {"pid", "port"})
    {
        if (info.contains(key) && !info[key].is_number_integer())
        {
            return std::nullopt;
        }
    }
    for (const char* key : {"app", "url", "mcp"})
    {
        if (info.contains(key) && !info[key].is_string())
        {
            return std::nullopt;
        }
    }
    if (info.contains("headless") && !info["headless"].is_boolean())
    {
        return std::nullopt;
    }

    Instance inst;
    inst.pid = info.value("pid", 0L);
    inst.port = info.value("port", 0);
    inst.headless = info.value("headless", false);
    inst.app = info.value("app", std::string());
    inst.url = info.value("url", std::string());
    inst.mcpUrl = info.value("mcp", std::string());

    std::error_code ec;
    inst.started = fs::last_write_time(file, ec);

    if (inst.mcpUrl.empty() || !platform::ProcessAlive(inst.pid))
    {
        return std::nullopt;
    }
    return inst;
}

std::vector<Instance> Discover(const fs::path& dir)
{
    std::vector<Instance> found;
    std::error_code ec;
    for (const fs::directory_entry& entry : fs::directory_iterator(dir, ec))
    {
        if (entry.path().extension() == ".json" && entry.is_regular_file(ec))
        {
            if (std::optional<Instance> inst = ReadInstance(entry.path()))
            {
                found.push_back(*inst);
            }
        }
    }
    return found;
}

std::optional<Instance> FindByPid(const fs::path& dir, long pid)
{
    return ReadInstance(dir / (std::to_string(pid) + ".json"));
}

std::optional<Instance> Select(const std::vector<Instance>& instances, const std::string& policy)
{
    std::optional<Instance> best;
    for (const Instance& inst : instances)
    {
        if (policy.rfind("pid:", 0) == 0 && inst.pid != std::atol(policy.c_str() + 4))
        {
            continue;
        }
        if (policy.rfind("port:", 0) == 0 && inst.port != std::atoi(policy.c_str() + 5))
        {
            continue;
        }
        if ((policy == "gui" && inst.headless) || (policy == "headless" && !inst.headless))
        {
            continue;
        }
        const bool older = best && inst.started < best->started;
        const bool newer = best && inst.started > best->started;
        if (!best || (policy == "oldest" ? older : newer))
        {
            best = inst;
        }
    }
    return best;
}

std::optional<Instance> WaitForInstance(const fs::path& dir, long pid, std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (std::optional<Instance> inst = FindByPid(dir, pid))
        {
            return inst;
        }
        if (!platform::ProcessAlive(pid))
        {
            return std::nullopt;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return std::nullopt;
}

void Stop(const Instance& inst, std::chrono::seconds grace)
{
    std::string host;
    int port = 0;
    std::string path;
    if (ParseUrl(inst.url + "/api/v1/shutdown", host, port, path))
    {
        Post(host, port, path, "{}");
    }

    const auto deadline = std::chrono::steady_clock::now() + grace;
    while (platform::ProcessAlive(inst.pid) && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (platform::ProcessAlive(inst.pid))
    {
        platform::KillProcess(inst.pid);
    }
}

void PruneOldFiles(const fs::path& dir, std::chrono::hours maxAge)
{
    std::error_code ec;
    const auto now = fs::file_time_type::clock::now();
    for (const fs::directory_entry& entry : fs::directory_iterator(dir, ec))
    {
        if (entry.is_regular_file(ec) && now - entry.last_write_time(ec) > maxAge)
        {
            fs::remove(entry.path(), ec);
        }
    }
}

} // namespace bridge
