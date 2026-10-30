// kicad-mcp-bridge — stdio MCP server that fronts any number of KiCad processes
//
// Speaks newline-delimited JSON-RPC 2.0 on stdin/stdout. Derived from unreal-ng's
// unreal-mcp-bridge (byte pipe to a Streamable-HTTP /mcp endpoint), extended with instance
// management because one MCP client entry must serve every KiCad on the machine:
//
//   * initialize / ping / notifications / tools/list are answered by the bridge itself
//     (kicadopenapi's /mcp is stateless, so instances can come and go under one session);
//   * exactly two tools are advertised, `search` and `invoke` (shared/mcp_tools.h);
//   * search = the bound instance's methods + the bridge's instance_list / instance_select /
//     instance_start; invoke of those runs in the bridge, everything else is forwarded
//     verbatim to the bound instance.
//
// Binding: --url / KICAD_MCP_URL pins one endpoint. Otherwise the bridge picks a live
// instance from the discovery files (<tmp>/kicad/openapi/<pid>.json) by policy
// (--select / KICAD_MCP_SELECT: newest | oldest | gui | headless | pid:N | port:N); if
// none exists it starts one (--autostart / KICAD_MCP_AUTOSTART: headless | gui | off).
// Binding is sticky until instance_select / instance_start or the instance dies. A
// request whose instance died gets an error and is never retried elsewhere.
//
// Idle reaping: headless instances started by the bridge are terminated after
// --idle / KICAD_MCP_IDLE (e.g. 600, 90s, 10m, 1m30s; 0 = never) without requests, unless
// their status reports unsaved changes. The next request starts a fresh one.

#include "instances.h"
#include "mini-http.h"

#include <mcp_search.h>
#include <mcp_tools.h>
#include <platform.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>


using nlohmann::json;
namespace fs = std::filesystem;
namespace platform = kopenapi::platform;
using Clock = std::chrono::steady_clock;

namespace
{

constexpr const char* kVersion = "0.2.0";

/// @brief region <Configuration>

struct Config
{
    std::string pinnedUrl;
    std::string policy = "newest";
    std::string autostart = "headless";
    std::chrono::seconds idle{600};
    std::chrono::seconds startTimeout{60};
    fs::path discoveryDir = platform::DiscoveryDir();
    fs::path kicad;
    fs::path kicadCli;
};

/// @brief "600", "90s", "10m", "1m30s", "1h" → seconds; nullopt on garbage
std::optional<std::chrono::seconds> ParseDuration(const std::string& text)
{
    if (text == "off" || text == "never")
    {
        return std::chrono::seconds(0);
    }
    long total = 0;
    long number = -1;
    for (char c : text)
    {
        if (c >= '0' && c <= '9')
        {
            number = (number < 0 ? 0 : number * 10) + (c - '0');
            continue;
        }
        if (number < 0)
        {
            return std::nullopt;
        }
        if (c == 'h')
            total += number * 3600;
        else if (c == 'm')
            total += number * 60;
        else if (c == 's')
            total += number;
        else
            return std::nullopt;
        number = -1;
    }
    if (number >= 0)
    {
        total += number;
    }
    return std::chrono::seconds(total);
}

const char* EnvOrNull(const char* name)
{
    const char* value = std::getenv(name);
    return (value != nullptr && *value != '\0') ? value : nullptr;
}

void Usage()
{
    std::cerr << "usage: kicad-mcp-bridge [options]\n"
                 "  --url URL               pin one endpoint (env KICAD_MCP_URL)\n"
                 "  --select POLICY         newest|oldest|gui|headless|pid:N|port:N (env KICAD_MCP_SELECT)\n"
                 "  --autostart MODE        headless|gui|off when no instance runs (env KICAD_MCP_AUTOSTART)\n"
                 "  --idle DURATION         reap idle bridge-started headless instances, 0=never (env KICAD_MCP_IDLE)\n"
                 "  --start-timeout DUR     wait for a started instance (env KICAD_MCP_START_TIMEOUT)\n"
                 "  --kicad PATH            GUI executable (env KICAD_MCP_KICAD; default: next to the bridge)\n"
                 "  --kicad-cli PATH        kicad-cli executable (env KICAD_MCP_KICAD_CLI; default: next to the bridge)\n"
                 "  --discovery-dir DIR     discovery directory (default <tmp>/kicad/openapi)\n"
                 "Instances start from a snapshot of a macOS .app bundle (<bundle dir>/run-snapshots/), so a\n"
                 "rebuild does not change files under them; env KICAD_MCP_RUN_SNAPSHOT=0 starts the bundle itself.\n";
}

std::optional<Config> ParseConfig(int argc, char** argv)
{
    Config cfg;
    const fs::path exeDir = platform::ExecutableDir(argv[0]);
    cfg.kicad = exeDir / platform::ExecutableName("kicad");
    cfg.kicadCli = exeDir / platform::ExecutableName("kicad-cli");

    std::map<std::string, std::string> values;
    const std::pair<const char*, const char*> envs[] = {
        {"url", "KICAD_MCP_URL"},         {"select", "KICAD_MCP_SELECT"},
        {"autostart", "KICAD_MCP_AUTOSTART"}, {"idle", "KICAD_MCP_IDLE"},
        {"start-timeout", "KICAD_MCP_START_TIMEOUT"}, {"kicad", "KICAD_MCP_KICAD"},
        {"kicad-cli", "KICAD_MCP_KICAD_CLI"}};
    for (const auto& [key, env] : envs)
    {
        if (const char* value = EnvOrNull(env))
        {
            values[key] = value;
        }
    }
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg.rfind("--", 0) != 0 || i + 1 >= argc)
        {
            Usage();
            return std::nullopt;
        }
        values[arg.substr(2)] = argv[++i];
    }

    for (const auto& [key, value] : values)
    {
        if (key == "url")
            cfg.pinnedUrl = value;
        else if (key == "select")
            cfg.policy = value;
        else if (key == "autostart" && (value == "headless" || value == "gui" || value == "off"))
            cfg.autostart = value;
        else if (key == "discovery-dir")
            cfg.discoveryDir = value;
        else if (key == "kicad")
            cfg.kicad = value;
        else if (key == "kicad-cli")
            cfg.kicadCli = value;
        else if ((key == "idle" || key == "start-timeout") && ParseDuration(value))
            (key == "idle" ? cfg.idle : cfg.startTimeout) = *ParseDuration(value);
        else
        {
            std::cerr << "kicad-mcp-bridge: bad option --" << key << " " << value << "\n";
            Usage();
            return std::nullopt;
        }
    }
    return cfg;
}

/// @brief endregion </Configuration>

/// @brief region <Signal-safe child registry>

// Headless children the bridge started; killed when the bridge is terminated by a signal or
// console close (graceful shutdown happens on stdin EOF instead)
constexpr int kMaxChildren = 64;
std::atomic<long> g_headlessChildren[kMaxChildren];

void TrackHeadless(long pid)
{
    for (std::atomic<long>& slot : g_headlessChildren)
    {
        long empty = 0;
        if (slot.compare_exchange_strong(empty, pid))
        {
            return;
        }
    }
}

void UntrackHeadless(long pid)
{
    for (std::atomic<long>& slot : g_headlessChildren)
    {
        long expected = pid;
        slot.compare_exchange_strong(expected, 0);
    }
}

void TerminateHeadlessChildren()
{
    for (std::atomic<long>& slot : g_headlessChildren)
    {
        platform::KillProcess(slot.exchange(0));
    }
}

/// @brief Termination signal / console close: async-signal-safe, the process exits afterwards
void OnTermination()
{
    TerminateHeadlessChildren();
}

/// @brief endregion </Signal-safe child registry>

/// @brief region <JSON-RPC helpers>

std::mutex g_emitMutex;  // progress notifications come from a watcher thread

void Emit(const json& message)
{
    std::lock_guard<std::mutex> lock(g_emitMutex);
    std::cout << message.dump(-1, ' ', false, json::error_handler_t::replace) << std::endl;
}

void EmitRaw(const std::string& line)
{
    std::lock_guard<std::mutex> lock(g_emitMutex);
    std::cout << line << std::endl;
}

/// @brief One line for a progress notification: method, phase, done / total, item, the scalar counts
/// (one level of nesting flattened: violations.error=3), elapsed and eta
std::string ProgressMessage(const json& running)
{
    const json progress = running.value("progress", json::object());
    std::string text = running.value("method", std::string()) + " " + progress.value("phase", std::string());

    if (progress.value("total", 0) > 0)
    {
        text += " " + std::to_string(progress.value("done", 0)) + "/" + std::to_string(progress.value("total", 0));
    }
    if (!progress.value("item", std::string()).empty())
    {
        text += " " + progress["item"].get<std::string>();
    }

    std::string counts;
    auto add = [&](const std::string& key, const json& value)
    {
        if (value.is_primitive() && !value.is_null())
        {
            counts += (counts.empty() ? "" : " ") + key + "=" + (value.is_string() ? value.get<std::string>() : value.dump());
        }
    };
    if (progress.contains("counts") && progress["counts"].is_object())
    {
        for (const auto& [key, value] : progress["counts"].items())
        {
            if (value.is_object())
            {
                for (const auto& [sub, subValue] : value.items())
                {
                    add(key + "." + sub, subValue);
                }
            }
            else
            {
                add(key, value);
            }
        }
    }
    if (!counts.empty())
    {
        text += " | " + counts;
    }

    if (progress.contains("elapsed_s"))
    {
        text += " | " + progress["elapsed_s"].dump() + " s";
    }
    if (progress.contains("eta_s"))
    {
        text += ", eta " + progress["eta_s"].dump() + " s";
    }
    if (running.value("paused", false))
    {
        text += " (paused)";
    }
    return text;
}

/// @brief While a forwarded tools/call runs: the instance's progress (GET /api/v1/progress, answered
/// on its HTTP thread) as MCP notifications/progress for the request's progressToken, on each change,
/// polled every 500 ms; stops (joined) before the call's answer is written
class ProgressWatch
{
public:
    ProgressWatch(const std::string& host, int port, const json& token)
    {
        if (!token.is_string() && !token.is_number())
        {
            return;
        }
        m_thread = std::thread([this, host, port, token]() { Run(host, port, token); });
    }

    ~ProgressWatch()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stop = true;
        }
        m_wake.notify_all();
        if (m_thread.joinable())
        {
            m_thread.join();
        }
    }

    ProgressWatch(const ProgressWatch&) = delete;
    ProgressWatch& operator=(const ProgressWatch&) = delete;

private:
    void Run(const std::string& host, int port, const json& token)
    {
        json last;
        long sent = 0;
        std::unique_lock<std::mutex> lock(m_mutex);

        while (!m_wake.wait_for(lock, std::chrono::milliseconds(500), [this] { return m_stop; }))
        {
            lock.unlock();
            const bridge::HttpResult result = bridge::Get(host, port, "/api/v1/progress", 2);
            lock.lock();

            if (m_stop)
            {
                break;
            }
            const json parsed = result.ok ? json::parse(result.body, nullptr, false) : json();
            const json running = parsed.is_object() ? parsed.value("running", json()) : json();
            if (!running.is_object())
            {
                continue;
            }

            json key = running;
            key.erase("rev");
            if (key.contains("progress") && key["progress"].is_object())
            {
                for (const char* volatileKey : {"elapsed_s", "eta_s", "phase_eta_s"})
                {
                    key["progress"].erase(volatileKey);
                }
            }
            if (key == last)
            {
                continue;
            }
            last = key;

            Emit({{"jsonrpc", "2.0"},
                  {"method", "notifications/progress"},
                  {"params", {{"progressToken", token}, {"progress", ++sent}, {"message", ProgressMessage(running)}}}});
        }
    }

    std::mutex m_mutex;
    std::condition_variable m_wake;
    bool m_stop = false;
    std::thread m_thread;
};

json Result(const json& id, const json& result)
{
    return {{"jsonrpc", "2.0"}, {"id", id}, {"result", result}};
}

json Error(const json& id, int code, const std::string& message)
{
    return {{"jsonrpc", "2.0"}, {"id", id}, {"error", {{"code", code}, {"message", "bridge: " + message}}}};
}

json ToolResult(const json& data, bool isError = false)
{
    return {{"content", json::array({{{"type", "text"}, {"text", data.dump(2, ' ', false, json::error_handler_t::replace)}}})},
            {"structuredContent", data},
            {"isError", isError}};
}

std::string Lower(std::string text)
{
    for (char& c : text)
    {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text;
}

/// @brief Same matching idea as the server's registry search, for the bridge's own methods
bool Matches(const json& method, const std::string& query)
{
    std::string hay = Lower(method["name"].get<std::string>() + " " + method["summary"].get<std::string>());
    std::string term;
    bool any = false;
    for (char c : Lower(query) + " ")
    {
        if (std::isalnum(static_cast<unsigned char>(c)))
        {
            term += c;
        }
        else if (!term.empty())
        {
            any = true;
            if (hay.find(term) != std::string::npos)
            {
                return true;
            }
            term.clear();
        }
    }
    return !any;  // empty query matches everything
}

std::vector<std::string> ExtractSseDataLines(const std::string& sseBody)
{
    std::vector<std::string> payloads;
    size_t pos = 0;
    while (pos < sseBody.size())
    {
        size_t lineEnd = sseBody.find('\n', pos);
        const size_t next = (lineEnd == std::string::npos) ? sseBody.size() : lineEnd + 1;
        std::string line = sseBody.substr(pos, (lineEnd == std::string::npos ? sseBody.size() : lineEnd) - pos);
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        if (line.compare(0, 5, "data:") == 0)
        {
            size_t start = (line.size() > 5 && line[5] == ' ') ? 6 : 5;
            if (start < line.size())
            {
                payloads.push_back(line.substr(start));
            }
        }
        pos = next;
    }
    return payloads;
}

json InstanceJson(const bridge::Instance& inst)
{
    return {{"pid", inst.pid}, {"port", inst.port}, {"headless", inst.headless}, {"app", inst.app},
            {"url", inst.url}};
}

/// @brief endregion </JSON-RPC helpers>

// Instance management methods, owned by the bridge. They are not MCP tools: like every KiCad
// method they are found with `search` and called with `invoke`.
const json kBridgeMethods = json::parse(R"json([
  {"name": "instance_list", "gui_only": false,
   "summary": "List running KiCad instances (GUI and headless) this MCP session can talk to; marks the current one. Check it before mutating after an instance change.",
   "inputSchema": {"type": "object", "properties": {}}},
  {"name": "instance_select", "gui_only": false,
   "summary": "Route all following calls to another running KiCad instance, by pid or port.",
   "inputSchema": {"type": "object", "properties": {
       "pid": {"type": "integer"}, "port": {"type": "integer"}}}},
  {"name": "instance_stop", "gui_only": false,
   "summary": "Stop a headless KiCad instance (by pid or port), started by this bridge or not: it is asked to shut down, killed after 5 s. Refused while it has unsaved changes unless discard: true. A GUI instance is closed through its own menu (app_quit), never here.",
   "inputSchema": {"type": "object", "properties": {
       "pid": {"type": "integer"}, "port": {"type": "integer"},
       "discard": {"type": "boolean", "default": false, "description": "stop it even with unsaved changes (they are lost)"}}}},
  {"name": "instance_start", "gui_only": false,
   "summary": "Start a new KiCad instance and route following calls to it. mode=headless (no windows, fast; stopped after the idle timeout unless it has unsaved changes) or gui (visible editor windows, never stopped by the bridge).",
   "inputSchema": {"type": "object", "required": ["mode"], "properties": {
       "mode": {"type": "string", "enum": ["headless", "gui"]},
       "path": {"type": "string", "description": "Optional .kicad_pro/.kicad_sch/.kicad_pcb to open"},
       "build": {"type": "string", "description": "Optional build to run instead of the bridge's own: a build directory (its kicad/KiCad.app on macOS, kicad/ elsewhere), an .app bundle or the directory of the executables. The answer gives the executable started and its build time"},
       "select": {"type": "boolean", "default": true}}}}
])json");

bool IsBridgeMethod(const std::string& name)
{
    for (const json& m : kBridgeMethods)
    {
        if (m["name"] == name)
        {
            return true;
        }
    }
    return false;
}

class Bridge
{
public:
    explicit Bridge(Config cfg) : m_cfg(std::move(cfg)), m_logDir(m_cfg.discoveryDir.parent_path() / "openapi-logs")
    {
        std::error_code ec;
        fs::create_directories(m_logDir, ec);
        bridge::PruneOldFiles(m_logDir, std::chrono::hours(24));

        if (!m_cfg.pinnedUrl.empty())
        {
            m_url = m_cfg.pinnedUrl;
            Log("pinned to " + m_url);
        }
    }

    void HandleLine(const std::string& line);

    /// @brief Reaper thread body: collects exited children and stops idle headless instances
    void Housekeeping();

    /// @brief Graceful stop of the headless instances this bridge started (stdin EOF)
    void Shutdown();

private:
    struct Child
    {
        bool headless = false;
        Clock::time_point lastUse;
        fs::path log;
    };

    void Log(const std::string& text) { std::cerr << "kicad-mcp-bridge: " << text << std::endl; }

    // All below run with m_mutex held
    void BindTo(const bridge::Instance& inst, const std::string& why);
    void Unbind();
    /// @brief After app_restart: wait for the new process on the same port and bind to it
    bool FollowRestart();
    bool EnsureBound(std::string& error);
    std::optional<bridge::Instance> Start(const std::string& mode, const std::string& path, std::string& error,
                                          const std::string& build = {}, json* started = nullptr);

    /// @brief Forward one request line; std::nullopt on transport failure (error already emitted)
    /// aProgressToken (string or number): the instance's progress meanwhile as notifications/progress
    std::optional<bridge::HttpResult> Forward(const std::string& line, const json& id, const json& aProgressToken = json());

    /// @brief After a receive timeout: the job the server runs now and how to get its result
    std::string RunningWork(const std::string& host, int port);

    json CallBridgeMethod(const std::string& name, const json& args);

    /// @brief MCP `search`: the bound instance's methods + matching bridge methods
    json Search(const std::string& line, const json& args);
    json ListInstances();

    fs::path NewLogFile(const std::string& mode);

    Config m_cfg;
    fs::path m_logDir;  // <tmp>/kicad/openapi-logs: stdout/stderr of started instances
    int m_logCounter = 0;
    std::vector<long> m_dying;  // stopped instances not yet collected
    std::mutex m_mutex;
    std::string m_url;  // bound endpoint, empty = unbound
    long m_pid = 0;     // bound pid (0 when pinned)
    std::map<long, Child> m_children;
    bool m_inFlight = false;
    long m_lostPid = 0;  // bound instance died unexpectedly; the next request reports it
    std::string m_lostUrl;     // its endpoint: a restart comes back on the same port
    std::string m_restartUrl;  // app_restart was forwarded to this endpoint
};

void Bridge::BindTo(const bridge::Instance& inst, const std::string& why)
{
    m_url = inst.mcpUrl;
    m_pid = inst.pid;
    Log("bound to pid " + std::to_string(inst.pid) + " " + inst.mcpUrl + " (" + why + ")");
}

void Bridge::Unbind()
{
    if (m_cfg.pinnedUrl.empty())
    {
        m_url.clear();
        m_pid = 0;
    }
}

fs::path Bridge::NewLogFile(const std::string& mode)
{
    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&now));
    return m_logDir / (mode + "-" + stamp + "-" + std::to_string(platform::CurrentPid()) + "-"
                       + std::to_string(++m_logCounter) + ".log");
}

/// @brief The executable aName of the build aBuild (see instance_start "build"); empty if none is there
fs::path BuildExecutable(const fs::path& aBuild, const std::string& aName)
{
    const std::string exe = platform::ExecutableName(aName);
    const fs::path    tries[] = {aBuild / "kicad" / "KiCad.app" / "Contents" / "MacOS" / exe,
                                 aBuild / "Contents" / "MacOS" / exe, aBuild / "kicad" / exe, aBuild / exe};
    std::error_code   ec;

    for (const fs::path& p : tries)
    {
        if (fs::is_regular_file(p, ec))
        {
            return p;
        }
    }
    return {};
}

std::optional<bridge::Instance> Bridge::Start(const std::string& mode, const std::string& path, std::string& error,
                                              const std::string& build, json* started)
{
    fs::path exe = mode == "headless" ? m_cfg.kicadCli : m_cfg.kicad;
    if (!build.empty())
    {
        exe = BuildExecutable(build, mode == "headless" ? "kicad-cli" : "kicad");
        if (exe.empty())
        {
            error = "no " + std::string(mode == "headless" ? "kicad-cli" : "kicad") + " executable in build " + build;
            return std::nullopt;
        }
    }

    std::vector<std::string> argv;
    if (mode == "headless")
    {
        // --parent-pid: the server exits when the bridge dies without cleaning up (hard kill)
        argv = {exe.string(), "openapi-server", "--parent-pid", std::to_string(platform::CurrentPid())};
    }
    else
    {
        argv = {exe.string()};
    }

    if (started)
    {
        std::error_code ec;
        const auto      when = fs::last_write_time(exe, ec);
        (*started)["executable"] = exe.string();
        if (!ec)
        {
            const auto sys = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                    when - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
            const std::time_t t = std::chrono::system_clock::to_time_t(sys);
            char              text[32] = {};
            std::strftime(text, sizeof(text), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
            (*started)["built"] = text;
        }
    }
    if (!path.empty())
    {
        argv.push_back(path);
    }

    // started from a snapshot of the build: a rebuild does not change the files under it
    if (const char* snap = std::getenv("KICAD_MCP_RUN_SNAPSHOT"); !snap || std::string(snap) != "0")
    {
        std::string note;
        argv[0] = platform::RunSnapshot(argv[0], note).string();
        Log(note);
    }

    const fs::path log = NewLogFile(mode);
    const long pid = platform::SpawnDetached(argv, log, error);
    if (!pid)
    {
        return std::nullopt;
    }
    Log("started " + mode + " pid " + std::to_string(pid) + ", log " + log.string());

    std::optional<bridge::Instance> inst = bridge::WaitForInstance(m_cfg.discoveryDir, pid, m_cfg.startTimeout);
    if (!inst)
    {
        platform::KillProcess(pid);
        m_dying.push_back(pid);
        error = "started " + argv[0] + " (pid " + std::to_string(pid) + ") but it did not publish a "
                + "kicadopenapi endpoint within " + std::to_string(m_cfg.startTimeout.count()) + " s; see " + log.string();
        return std::nullopt;
    }

    m_children[pid] = {mode == "headless", Clock::now(), log};
    if (mode == "headless")
    {
        TrackHeadless(pid);
    }
    return inst;
}

bool Bridge::FollowRestart()
{
    const std::string url = m_url;
    const long        old = m_pid;
    const auto        deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(m_cfg.startTimeout);

    while (Clock::now() < deadline)
    {
        for (const bridge::Instance& inst : bridge::Discover(m_cfg.discoveryDir))
        {
            if (inst.mcpUrl == url && inst.pid != old)
            {
                BindTo(inst, "pid " + std::to_string(old) + " restarted on the same port");
                m_restartUrl.clear();
                m_lostPid = 0;
                m_lostUrl.clear();
                return true;
            }
        }
        m_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        m_mutex.lock();
    }
    m_restartUrl.clear();
    return false;
}

bool Bridge::EnsureBound(std::string& error)
{
    if (!m_cfg.pinnedUrl.empty())
    {
        return true;
    }
    if (!m_url.empty() && platform::ProcessAlive(m_pid))
    {
        return true;
    }
    if (!m_url.empty())
    {
        m_lostPid = m_pid;
        m_lostUrl = m_url;
    }
    Unbind();

    // the same design came back on the same port (app_restart, or a restart by hand): follow it;
    // after app_restart wait for the new process to publish its endpoint
    if (m_lostPid && !m_lostUrl.empty())
    {
        const bool restarting = m_restartUrl == m_lostUrl;
        const auto deadline = Clock::now() + (restarting ? std::chrono::duration_cast<Clock::duration>(m_cfg.startTimeout)
                                                         : Clock::duration::zero());
        for (;;)
        {
            for (const bridge::Instance& inst : bridge::Discover(m_cfg.discoveryDir))
            {
                if (inst.mcpUrl == m_lostUrl && inst.pid != m_lostPid)
                {
                    BindTo(inst, "pid " + std::to_string(m_lostPid) + " restarted on the same port");
                    m_lostPid = 0;
                    m_lostUrl.clear();
                    m_restartUrl.clear();
                    return true;
                }
            }
            if (Clock::now() >= deadline)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        m_restartUrl.clear();
    }

    if (m_lostPid)
    {
        // Never continue silently on another instance: it holds a different design
        error = "the KiCad instance (pid " + std::to_string(m_lostPid) + ") is gone — the next call binds to "
                + "another instance (policy '" + m_cfg.policy + "'); check instance_list before mutating";
        m_lostPid = 0;
        m_lostUrl.clear();
        return false;
    }

    if (std::optional<bridge::Instance> inst = bridge::Select(bridge::Discover(m_cfg.discoveryDir), m_cfg.policy))
    {
        BindTo(*inst, "policy " + m_cfg.policy);
        return true;
    }
    if (m_cfg.autostart == "off")
    {
        error = "no running KiCad instance matches policy '" + m_cfg.policy + "' and autostart is off";
        return false;
    }
    std::string startError;
    if (std::optional<bridge::Instance> inst = Start(m_cfg.autostart, "", startError))
    {
        BindTo(*inst, "autostart " + m_cfg.autostart);
        return true;
    }
    error = "no running KiCad instance and autostart failed: " + startError;
    return false;
}

/// @brief How long a forwarded call waits for its answer; env KICAD_MCP_RECEIVE_TIMEOUT_S (tests)
static int ReceiveTimeoutSeconds()
{
    static const int seconds = []
    {
        const char* env = std::getenv("KICAD_MCP_RECEIVE_TIMEOUT_S");
        const int value = env ? std::atoi(env) : 0;
        return value > 0 ? value : bridge::kPostReceiveTimeoutSeconds;
    }();
    return seconds;
}

/// @brief The receive timeout of one call: a call that asks the server to wait (job_get wait_s) gets
/// that long plus a margin when it is longer than the default
static int ReceiveTimeoutFor(const std::string& line)
{
    const int base = ReceiveTimeoutSeconds();
    const json msg = json::parse(line, nullptr, false);

    if (!msg.is_object() || !msg.contains("params") || !msg["params"].is_object())
        return base;

    const json& params = msg["params"];
    const json* args = params.contains("arguments") && params["arguments"].is_object() ? &params["arguments"] : nullptr;

    for (const char* key : {"arguments", "args"})
    {
        if (args && args->contains(key) && (*args)[key].is_object())
        {
            args = &(*args)[key];
            break;
        }
    }

    if (!args || !args->contains("wait_s") || !(*args)["wait_s"].is_number())
        return base;

    return std::max(base, (*args)["wait_s"].get<int>() + 30);
}

std::optional<bridge::HttpResult> Bridge::Forward(const std::string& line, const json& id, const json& aProgressToken)
{
    std::string host;
    int port = 0;
    std::string path;
    if (!bridge::ParseUrl(m_url, host, port, path))
    {
        Emit(Error(id, -32603, "invalid endpoint URL '" + m_url + "'"));
        return std::nullopt;
    }

    // long calls must not block housekeeping bookkeeping; re-locked even if Post throws, so the
    // caller's lock never unlocks a mutex it does not hold
    auto postUnlocked = [&]()
    {
        struct RELOCK
        {
            std::mutex& m;
            ~RELOCK() { m.lock(); }
        } relock{m_mutex};
        m_mutex.unlock();
        ProgressWatch watch(host, port, aProgressToken);
        return bridge::Post(host, port, path, line, ReceiveTimeoutFor(line));
    };

    long pid = m_pid;
    m_inFlight = true;
    bridge::HttpResult result = postUnlocked();
    m_inFlight = false;

    // app_restart was sent here: the old process may still be quitting while its server is gone;
    // wait for the new one on the same port and send the call once more
    if (!result.ok && !m_restartUrl.empty() && m_restartUrl == m_url && FollowRestart())
    {
        pid = m_pid;
        m_inFlight = true;
        result = postUnlocked();
        m_inFlight = false;
    }

    if (auto it = m_children.find(pid); it != m_children.end())
    {
        it->second.lastUse = Clock::now();
    }

    if (result.ok && line.find("\"app_restart\"") != std::string::npos)
    {
        m_restartUrl = m_url;  // the next call follows the new process on this port
    }

    if (!result.ok)
    {
        std::string message = "cannot reach " + m_url + " (" + result.error + ")";
        if (result.error == "receive timeout" && platform::ProcessAlive(pid))
        {
            message = "no answer within " + std::to_string(ReceiveTimeoutFor(line))
                      + " s; the call may still run on the server and its changes will stand. " + RunningWork(host, port);
        }
        else if (m_cfg.pinnedUrl.empty() && !platform::ProcessAlive(pid))
        {
            message += "; the instance is gone — the next call binds to another instance (policy '"
                       + m_cfg.policy + "'); check instance_list before mutating";
            Unbind();
            m_lostPid = 0;  // reported right here
        }
        if (!id.is_null())
        {
            Emit(Error(id, -32603, message));
        }
        return std::nullopt;
    }
    return result;
}

std::string Bridge::RunningWork(const std::string& host, int port)
{
    bridge::HttpResult jobs = bridge::Post(host, port, "/api/v1/job_list", "{}", 5);
    if (jobs.ok && jobs.status == 200)
    {
        json body = json::parse(jobs.body, nullptr, false);
        if (body.is_object() && body.contains("jobs") && body["jobs"].is_array())
        {
            for (const json& job : body["jobs"])
            {
                const std::string state = job.value("state", std::string());
                if (state == "queued" || state == "running")
                {
                    const std::string jobId = job.value("job_id", std::string());
                    return "It runs as " + jobId + " (" + job.value("method", std::string()) + ", " + state
                           + "): job_get {job_id: \"" + jobId + "\", wait_s: 60} gives its result, job_cancel stops it";
                }
            }
        }
    }
    return "No job runs now: edit_log and api_trace show what it did; api_call_result gives a late call's result";
}

json Bridge::ListInstances()
{
    json list = json::array();
    for (const bridge::Instance& inst : bridge::Discover(m_cfg.discoveryDir))
    {
        json entry = InstanceJson(inst);
        entry["current"] = inst.pid == m_pid;
        auto child = m_children.find(inst.pid);
        entry["started_by_bridge"] = child != m_children.end();
        if (child != m_children.end())
        {
            entry["log"] = child->second.log.string();
        }

        std::string host;
        int port = 0;
        std::string path;
        if (bridge::ParseUrl(inst.url + "/api/v1/status", host, port, path))
        {
            bridge::HttpResult status = bridge::Get(host, port, path, 2);
            json parsed = status.ok ? json::parse(status.body, nullptr, false) : json();
            if (!parsed.is_discarded() && parsed.is_object())
            {
                entry["status"] = parsed;
            }
        }
        list.push_back(entry);
    }
    return {{"instances", list}, {"policy", m_cfg.policy}, {"pinned", !m_cfg.pinnedUrl.empty()},
            {"idle_timeout_s", m_cfg.idle.count()}};
}

json Bridge::CallBridgeMethod(const std::string& name, const json& args)
{
    if (name == "instance_list")
    {
        return ToolResult(ListInstances());
    }

    if (!m_cfg.pinnedUrl.empty())
    {
        return ToolResult({{"error", "bridge is pinned to " + m_cfg.pinnedUrl + " (--url / KICAD_MCP_URL)"}}, true);
    }

    if (name == "instance_select")
    {
        const std::string policy = args.contains("pid")    ? "pid:" + std::to_string(args.value("pid", 0L))
                                   : args.contains("port") ? "port:" + std::to_string(args.value("port", 0))
                                                           : "";
        if (policy.empty())
        {
            return ToolResult({{"error", "give pid or port"}}, true);
        }
        std::optional<bridge::Instance> inst = bridge::Select(bridge::Discover(m_cfg.discoveryDir), policy);
        if (!inst)
        {
            return ToolResult({{"error", "no running instance with " + policy}}, true);
        }
        BindTo(*inst, "instance_select");
        return ToolResult({{"selected", InstanceJson(*inst)}});
    }

    if (name == "instance_stop")
    {
        const std::string policy = args.contains("pid")    ? "pid:" + std::to_string(args.value("pid", 0L))
                                   : args.contains("port") ? "port:" + std::to_string(args.value("port", 0))
                                                           : "";
        if (policy.empty())
        {
            return ToolResult({{"error", "give pid or port"}}, true);
        }
        std::optional<bridge::Instance> inst = bridge::Select(bridge::Discover(m_cfg.discoveryDir), policy);
        if (!inst)
        {
            return ToolResult({{"error", "no running instance with " + policy}}, true);
        }
        if (!inst->headless)
        {
            return ToolResult({{"error", "pid " + std::to_string(inst->pid) + " is a GUI instance: close it through its "
                                         "menu (app_quit)"}}, true);
        }

        std::string host;
        int         port = 0;
        std::string path;
        json        status;
        if (bridge::ParseUrl(inst->url + "/api/v1/status", host, port, path))
        {
            bridge::HttpResult got = bridge::Get(host, port, path, 2);
            status = got.ok ? json::parse(got.body, nullptr, false) : json();
        }
        const bool unsaved = status.is_object() && status.value("unsaved", false);
        if (unsaved && !args.value("discard", false))
        {
            return ToolResult({{"error", "pid " + std::to_string(inst->pid) + " has unsaved changes: save them, or "
                                         "instance_stop {discard: true}"},
                               {"documents", status.value("documents", json::array())}}, true);
        }

        // the caller holds m_mutex
        if (inst->pid == m_pid)
        {
            Unbind();
        }
        if (m_children.erase(inst->pid))
        {
            UntrackHeadless(inst->pid);
        }
        m_dying.push_back(inst->pid);
        bridge::Stop(*inst, std::chrono::seconds(5));
        return ToolResult({{"stopped", InstanceJson(*inst)}, {"discarded", unsaved},
                           {"exited", !platform::ProcessAlive(inst->pid)}});
    }

    // instance_start
    const std::string mode = args.value("mode", std::string());
    if (mode != "headless" && mode != "gui")
    {
        return ToolResult({{"error", "mode must be 'headless' or 'gui'"}}, true);
    }
    std::string error;
    json                            build = json::object();
    std::optional<bridge::Instance> inst =
            Start(mode, args.value("path", std::string()), error, args.value("build", std::string()), &build);
    if (!inst)
    {
        return ToolResult({{"error", error}}, true);
    }
    if (args.value("select", true))
    {
        BindTo(*inst, "instance_start");
    }
    return ToolResult({{"started", InstanceJson(*inst)}, {"build", build}, {"selected", args.value("select", true)}});
}

json Bridge::Search(const std::string& line, const json& args)
{
    const std::string query = args.value("query", std::string());
    json methods = json::array();
    json more = json::array();
    json instance = nullptr;
    std::string note;

    std::string error;
    if (EnsureBound(error))
    {
        if (std::optional<bridge::HttpResult> result = Forward(line, nullptr))
        {
            json parsed = json::parse(result->body, nullptr, false);
            if (!parsed.is_discarded() && parsed.contains("result")
                && parsed["result"].contains("structuredContent")
                && parsed["result"]["structuredContent"].contains("methods"))
            {
                const json& found = parsed["result"]["structuredContent"];
                methods = found["methods"];
                if (found.contains("more") && found["more"].is_array())
                {
                    more = found["more"];
                }
            }
        }
        if (std::optional<bridge::Instance> inst = bridge::FindByPid(m_cfg.discoveryDir, m_pid))
        {
            instance = InstanceJson(*inst);
        }
    }
    else
    {
        note = error;
        Log(error);
    }

    // The bridge's own methods, formatted like the server's (compact unless detail: full or
    // asked for by name)
    const bool full = args.value("detail", std::string("brief")) == "full";
    const bool byName = args.contains("names") && args["names"].is_array();
    json notFound = json::array();

    if (byName)
    {
        // the server answered with not_found for names it does not know: some may be ours
        for (const json& name : args["names"])
        {
            bool ours = false;
            for (const json& m : kBridgeMethods)
            {
                if (m["name"] == name)
                {
                    methods.push_back(kopenapi::mcp::FormatMethod(m["name"], m["summary"], m["inputSchema"], false, true));
                    ours = true;
                }
            }
            bool served = false;
            for (const json& m : methods)
            {
                served |= m["name"] == name;
            }
            if (!ours && !served)
            {
                notFound.push_back(name);
            }
        }
    }
    else
    {
        for (const json& m : kBridgeMethods)
        {
            if (Matches(m, query))
            {
                methods.push_back(kopenapi::mcp::FormatMethod(m["name"], m["summary"], m["inputSchema"], false, full));
            }
        }
    }

    json data = {{"methods", methods},
                 {"count", methods.size()},
                 {"total", methods.size() + more.size()},
                 {"more", more},
                 {"instance", instance}};
    if (byName)
    {
        data.erase("more");
        data.erase("total");
        data["not_found"] = notFound;
    }
    if (!note.empty())
    {
        data["note"] = note;
    }
    return ToolResult(data);
}

void Bridge::HandleLine(const std::string& line)
{
    std::unique_lock<std::mutex> lock(m_mutex);

    const json msg = json::parse(line, nullptr, false);
    if (msg.is_discarded())
    {
        Emit(Error(nullptr, -32700, "parse error"));
        return;
    }

    const bool isRequest = msg.is_object() && msg.contains("id");
    const json id = isRequest ? msg["id"] : json();
    const std::string method = msg.is_object() ? msg.value("method", std::string()) : std::string();

    // Session-level messages are the bridge's own: instances are stateless and may change,
    // and the tool list is fixed (search + invoke)
    if (method == "initialize")
    {
        std::string version = msg.contains("params") ? msg["params"].value("protocolVersion", std::string()) : "";
        bool supported = false;
        for (const char* v : kopenapi::mcp::kSupportedVersions)
        {
            supported |= version == v;
        }
        Emit(Result(id, {{"protocolVersion", supported ? version : kopenapi::mcp::kProtocolVersion},
                         {"capabilities", {{"tools", json::object()}}},
                         {"serverInfo", {{"name", "kicad-mcp-bridge"}, {"version", kVersion}}},
                         {"instructions",
                          "Use search to find KiCad methods (also instance_list / instance_select / "
                          "instance_start for choosing the KiCad instance), then invoke to call them. "
                          "Save your work: idle headless instances started by the bridge are stopped "
                          "(never with unsaved changes)."}}));
        return;
    }
    if (method == "ping")
    {
        Emit(Result(id, json::object()));
        return;
    }
    if (msg.is_object() && !isRequest)
    {
        return;  // notifications (initialized, cancelled, ...) are not forwarded
    }
    if (method == "tools/list")
    {
        Emit(Result(id, {{"tools", json::parse(kopenapi::mcp::kToolsJson)}}));
        return;
    }

    if (method == "tools/call")
    {
        const json params = msg.value("params", json::object());
        const std::string tool = params.value("name", std::string());
        const json args = params.value("arguments", json::object());

        if (tool == kopenapi::mcp::kSearchTool)
        {
            Emit(Result(id, Search(line, args.is_object() ? args : json::object())));
            return;
        }
        if (tool == kopenapi::mcp::kInvokeTool && args.is_object() && args.contains("name")
            && args["name"].is_string() && IsBridgeMethod(args["name"].get<std::string>()))
        {
            json methodArgs = args.contains("arguments") ? args["arguments"] : args.value("args", json::object());
            Emit(Result(id, CallBridgeMethod(args["name"].get<std::string>(),
                                             methodArgs.is_object() ? methodArgs : json::object())));
            return;
        }
        if (tool != kopenapi::mcp::kInvokeTool)
        {
            Emit(Error(id, -32602, "unknown tool '" + tool + "' (tools: search, invoke)"));
            return;
        }
    }

    std::string error;
    if (!EnsureBound(error))
    {
        Emit(Error(id, -32603, error));
        return;
    }

    const json meta = msg.is_object() && msg.contains("params") && msg["params"].is_object()
                              ? msg["params"].value("_meta", json::object())
                              : json::object();
    const json progressToken = method == "tools/call" && meta.is_object() ? meta.value("progressToken", json()) : json();

    std::optional<bridge::HttpResult> result = Forward(line, id, progressToken);
    if (!result)
    {
        return;
    }
    if (result->contentType.find("text/event-stream") != std::string::npos)
    {
        for (const std::string& payload : ExtractSseDataLines(result->body))
        {
            EmitRaw(payload);
        }
    }
    else if (!result->body.empty())
    {
        EmitRaw(result->body);
    }
}

void Bridge::Housekeeping()
{
    std::vector<bridge::Instance> toStop;
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        for (auto it = m_dying.begin(); it != m_dying.end();)
        {
            it = platform::ProcessAlive(*it) ? it + 1 : m_dying.erase(it);
        }

        for (auto it = m_children.begin(); it != m_children.end();)
        {
            const long pid = it->first;
            if (!platform::ProcessAlive(pid))
            {
                Log("pid " + std::to_string(pid) + " exited");
                UntrackHeadless(pid);
                if (pid == m_pid)
                {
                    m_lostPid = pid;  // unexpected: the next request reports it (unless it restarted)
                    m_lostUrl = m_url;
                    Unbind();
                }
                it = m_children.erase(it);
                continue;
            }

            const bool idle = it->second.headless && m_cfg.idle.count() > 0 && !(m_inFlight && pid == m_pid)
                              && Clock::now() - it->second.lastUse > m_cfg.idle;
            std::optional<bridge::Instance> inst = idle ? bridge::FindByPid(m_cfg.discoveryDir, pid) : std::nullopt;
            if (!inst)
            {
                ++it;
                continue;
            }

            // Never drop unsaved work: the status endpoint reports "unsaved": true when dirty
            std::string host;
            int port = 0;
            std::string path;
            bool unsaved = false;
            if (bridge::ParseUrl(inst->url + "/api/v1/status", host, port, path))
            {
                bridge::HttpResult status = bridge::Get(host, port, path, 2);
                json parsed = status.ok ? json::parse(status.body, nullptr, false) : json();
                unsaved = parsed.is_object() && parsed.value("unsaved", false);
            }
            if (unsaved)
            {
                it->second.lastUse = Clock::now();  // re-check after another idle period
                ++it;
                continue;
            }

            // Planned stop, not a loss: the next request transparently gets a fresh instance
            Log("stopping idle headless pid " + std::to_string(pid));
            if (pid == m_pid)
            {
                Unbind();
            }
            UntrackHeadless(pid);
            m_dying.push_back(pid);
            toStop.push_back(*inst);
            it = m_children.erase(it);
        }
    }

    // Outside the lock: a graceful stop may take seconds
    for (const bridge::Instance& inst : toStop)
    {
        bridge::Stop(inst, std::chrono::seconds(5));
    }
}

void Bridge::Shutdown()
{
    std::vector<bridge::Instance> toStop;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (const auto& [pid, child] : m_children)
        {
            if (child.headless)
            {
                if (std::optional<bridge::Instance> inst = bridge::FindByPid(m_cfg.discoveryDir, pid))
                {
                    toStop.push_back(*inst);
                }
                else
                {
                    platform::KillProcess(pid);
                }
                UntrackHeadless(pid);
            }
        }
    }
    for (const bridge::Instance& inst : toStop)
    {
        bridge::Stop(inst, std::chrono::seconds(5));
    }
}

} // namespace

int main(int argc, char** argv)
{
    platform::InstallTerminationHandler(OnTermination);

    std::optional<Config> cfg = ParseConfig(argc, argv);
    if (!cfg)
    {
        return 2;
    }

    // KiCad processes this bridge starts work for an agent: they never stop on a modal question
    platform::SetEnv("KICAD_OPENAPI_AGENT", "1");

    // the agent's identity for remote control sessions (remote_session): one per bridge process
    bridge::SetExtraHeader("X-Kicad-Session", "bridge-" + std::to_string(platform::CurrentPid()) + "-"
                                                  + std::to_string(static_cast<long long>(std::time(nullptr))));

    Bridge bridgeState(*cfg);
    std::atomic<bool> running{true};
    std::thread housekeeping([&]() {
        while (running.load())
        {
            bridgeState.Housekeeping();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    std::string line;
    while (std::getline(std::cin, line))
    {
        if (!line.empty())
        {
            try
            {
                bridgeState.HandleLine(line);
            }
            catch (const std::exception& e)
            {
                // wrong JSON types in a request (nlohmann type_error) must not end the session
                const json msg = json::parse(line, nullptr, false);
                const json id = msg.is_object() && msg.contains("id") ? msg["id"] : json();
                Emit(Error(id, -32600, std::string("invalid request: ") + e.what()));
            }
        }
    }

    running.store(false);
    housekeeping.join();
    bridgeState.Shutdown();
    return 0;  // stdin closed (client exited)
}
