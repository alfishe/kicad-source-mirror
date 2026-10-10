/// @file kicadopenapi_service.h
/// @brief kicadopenapi — in-process Web API service, one per KiCad process.
///
/// Independent of the IPC API: own HTTP transport (cpp-httplib), own method registry, direct
/// object-model access.  Hosted by the project manager, by standalone editors, and headless
/// by `kicad-cli openapi-server`.  Binds 127.0.0.1 only.
///
/// Served surface (and nothing else — the OpenAPI manifest is generated from it):
///   GET  /                      swagger-ui over the manifest
///   GET  /api/v1/status         service/process info
///   GET  /api/v1/openapi.json   manifest
///   POST /api/v1/{method}       registry methods (KOPENAPI_REGISTRY)
///   POST /api/v1/shutdown       only when the host installed a shutdown handler (headless)
///
/// Threading: HTTP handlers run on httplib worker threads and never touch the model.  Every
/// registry method is marshalled to the main thread via wxTheApp->CallAfter and awaited with
/// a timeout; Stop() unblocks waiting workers before joining them.
#ifndef KICADOPENAPI_SERVICE_H
#define KICADOPENAPI_SERVICE_H

#include <functional>
#include <memory>

#include <kicommon.h>

class KIWAY;


class KICOMMON_API KICAD_OPENAPI_SERVICE
{
public:
    static constexpr int DEFAULT_PORT = 4242;
    static constexpr int PORT_PROBE_COUNT = 10;

    KICAD_OPENAPI_SERVICE( KIWAY* aKiway, bool aHeadless );
    ~KICAD_OPENAPI_SERVICE();

    /// @brief Bind and start serving.  Tries aPort, then the next PORT_PROBE_COUNT - 1 ports.
    /// A negative aPort means DefaultPort().  Returns false if no port could be bound.
    bool Start( int aPort = -1 );

    /// @brief Stop serving; idempotent.  Must be called before the KIWAY/frames go away.
    void Stop();

    /// @brief Custom main loops (headless host): called from any thread right after work was queued
    /// for the main thread, so an idle loop can wake at once instead of polling.
    /// Set before Start().
    void SetMainLoopWaker( std::function<void()> aWaker );

    /// @brief Enables POST /api/v1/shutdown (headless host only; GUI processes are never shut down
    /// over HTTP).  The handler runs on an HTTP worker and must only signal the host.
    /// Set before Start().
    void SetShutdownHandler( std::function<void()> aHandler );

    /// @brief Start loading the editor kifaces in the background (queued on the main loop, not
    /// awaited), so the first request does not pay for it.  Headless hosts call this after
    /// Start(); GUI hosts load lazily to keep the UI responsive.
    void PreloadKifaces();

    /// @brief The host finished opening what it was started with (documents, editors).  After
    /// app_restart the service announces itself (discovery file) only then, or once the
    /// documents it was asked to reopen are open, whichever comes first.
    void StartupOpened();

    /// @brief True while a registry method runs on the main thread (work an agent asked for, as
    /// opposed to user interaction).  Main thread only.
    static bool InMainThreadCall();

    bool Running() const;

    /// @brief Bound port, or 0 when not running.
    int Port() const;

    /// @brief Port of this process's running service, 0 when none
    static int CurrentPort();

    /// @brief No modal questions in this process now: an API call is running, or an agent started
    /// the process (KICAD_OPENAPI_AGENT, set by the bridge and app_restart). KiCad's prompts take
    /// their API answer and report to the journal instead.
    static bool NoModalUi();

    /// @brief The answer to "open a file someone else has open" while NoModalUi(): pcb_open /
    /// sch_open override_lock sets it for the duration of the call
    static void SetOverrideLock( bool aOverride );
    static bool OverrideLock();

    /// @brief KICAD_OPENAPI_PORT env override, else DEFAULT_PORT.
    static int DefaultPort();

private:
    struct IMPL;
    std::unique_ptr<IMPL> m_impl;
};

#endif
