#if defined(_WIN32)

#include "platform_windows.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <iterator>

namespace kopenapi::platform
{

static void (*s_terminationHandler)() = nullptr;

static BOOL WINAPI onConsoleControl(DWORD /*event*/)
{
    if (s_terminationHandler)
    {
        s_terminationHandler();
    }
    return FALSE; // continue with the default handler (process exit)
}

static std::wstring widen(const std::string& text)
{
    if (text.empty())
    {
        return std::wstring();
    }
    const int len = MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
    std::wstring out(static_cast<size_t>(len), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), out.data(), len);
    return out;
}

/// Quotes one argument per the CommandLineToArgvW rules
static std::wstring quoteArg(const std::wstring& arg)
{
    if (!arg.empty() && arg.find_first_of(L" \t\n\v\"") == std::wstring::npos)
    {
        return arg;
    }
    std::wstring out = L"\"";
    for (size_t i = 0;; ++i)
    {
        size_t backslashes = 0;
        while (i < arg.size() && arg[i] == L'\\')
        {
            ++i;
            ++backslashes;
        }
        if (i == arg.size())
        {
            out.append(backslashes * 2, L'\\');
            break;
        }
        out.append(arg[i] == L'"' ? backslashes * 2 + 1 : backslashes, L'\\');
        out.push_back(arg[i]);
    }
    out.push_back(L'"');
    return out;
}

std::filesystem::path ExecutablePathWindows()
{
    wchar_t buffer[MAX_PATH * 4];
    const DWORD len = GetModuleFileNameW(nullptr, buffer, static_cast<DWORD>(std::size(buffer)));
    if (len == 0 || len >= std::size(buffer))
    {
        return {};
    }
    return std::filesystem::path(std::wstring(buffer, len));
}

long CurrentPidWindows()
{
    return static_cast<long>(GetCurrentProcessId());
}

bool ProcessAliveWindows(long pid)
{
    HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (process == nullptr)
    {
        return false;
    }
    DWORD code = 0;
    const bool alive = GetExitCodeProcess(process, &code) && code == STILL_ACTIVE;
    CloseHandle(process);
    return alive;
}

long SpawnDetachedWindows(const std::vector<std::string>& argv, const std::filesystem::path& logFile,
                          std::string& error)
{
    std::wstring cmdline;
    for (const std::string& arg : argv)
    {
        if (!cmdline.empty())
        {
            cmdline += L' ';
        }
        cmdline += quoteArg(widen(arg));
    }

    SECURITY_ATTRIBUTES sa{sizeof(sa), nullptr, TRUE};
    HANDLE nul = CreateFileW(L"NUL", GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa,
                             OPEN_EXISTING, 0, nullptr);
    HANDLE log = CreateFileW(logFile.wstring().c_str(), FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa,
                             OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (log == INVALID_HANDLE_VALUE)
    {
        log = nul;
    }

    // Inherit only these handles — never our stdio pipes, or an MCP client would not see EOF
    // on them while the child keeps running
    HANDLE inherit[2] = {nul, log};
    SIZE_T attrSize = 0;
    InitializeProcThreadAttributeList(nullptr, 1, 0, &attrSize);
    std::vector<char> attrBuffer(attrSize);
    auto attrs = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attrBuffer.data());
    InitializeProcThreadAttributeList(attrs, 1, 0, &attrSize);
    UpdateProcThreadAttribute(attrs, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST, inherit,
                              (log == nul ? 1 : 2) * sizeof(HANDLE), nullptr, nullptr);

    STARTUPINFOEXW si{};
    si.StartupInfo.cb = sizeof(si);
    si.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    si.StartupInfo.hStdInput = nul;
    si.StartupInfo.hStdOutput = log;
    si.StartupInfo.hStdError = log;
    si.lpAttributeList = attrs;

    PROCESS_INFORMATION pi{};
    const BOOL ok = CreateProcessW(widen(argv[0]).c_str(), cmdline.data(), nullptr, nullptr, TRUE,
                                   EXTENDED_STARTUPINFO_PRESENT | CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW,
                                   nullptr, nullptr, &si.StartupInfo, &pi);
    const DWORD lastError = GetLastError();

    DeleteProcThreadAttributeList(attrs);
    if (log != nul)
    {
        CloseHandle(log);
    }
    CloseHandle(nul);

    if (!ok)
    {
        error = "cannot start " + argv[0] + " (Windows error " + std::to_string(lastError) + ")";
        return 0;
    }
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return static_cast<long>(pi.dwProcessId);
}

void KillProcessWindows(long pid)
{
    HANDLE process = OpenProcess(PROCESS_TERMINATE, FALSE, static_cast<DWORD>(pid));
    if (process != nullptr)
    {
        TerminateProcess(process, 1);
        CloseHandle(process);
    }
}

void InstallTerminationHandlerWindows(void (*handler)())
{
    s_terminationHandler = handler;
    SetConsoleCtrlHandler(onConsoleControl, TRUE);
}

} // namespace kopenapi::platform

#endif // _WIN32
