/// @file kicadopenapi_registry.h
/// @brief kicadopenapi method registry.
///
/// Every Web API method is declared once, next to its handler, and registers itself at
/// static initialization:
///
///   static KOPENAPI_RESULT h_open_pcb( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );
///
///   KOPENAPI_REGISTER( "open_pcb", "Open a .kicad_pcb in the PCB editor window",
///                      R"json({"type":"object","required":["path"],
///                          "properties":{"path":{"type":"string"}}})json"_json,
///                      true, h_open_pcb );
///
/// The registry is the single source of truth: the service routes POST /api/v1/{name}
/// through it and generates /api/v1/openapi.json from it, so the manifest can only list
/// what is actually served.
///
/// The registry lives in kicommon so that methods registered from kiface DSOs land in the
/// same (process-wide) instance.
#ifndef KICADOPENAPI_REGISTRY_H
#define KICADOPENAPI_REGISTRY_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <set>
#include <map>
#include <stdexcept>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <json_common.h>
#include <kicommon.h>

class KIWAY;


/// @brief Process-level surface handed to every handler; handlers always run on the main thread.
struct KOPENAPI_CONTEXT
{
    KIWAY* kiway = nullptr;
    bool   headless = false;
};


/// @brief Handler outcome: HTTP status + JSON body.  Errors use { "error": { code, message } }.
struct KOPENAPI_RESULT
{
    int            status = 200;
    nlohmann::json body = nlohmann::json::object();

    static KOPENAPI_RESULT Ok( nlohmann::json aBody ) { return { 200, std::move( aBody ) }; }

    static KOPENAPI_RESULT Error( int aStatus, const std::string& aMessage )
    {
        return { aStatus, { { "error", { { "code", aStatus }, { "message", aMessage } } } } };
    }
};


/// @brief Thrown by a handler that cannot answer from complete data right now.  The service
/// answers it with 503, the message, "retry_after_s" in the body and a Retry-After header, so
/// a caller never gets partial data with 200.
class KICOMMON_API KOPENAPI_RETRY_LATER : public std::runtime_error
{
public:
    KOPENAPI_RETRY_LATER( const std::string& aMessage, int aRetryAfterSec, nlohmann::json aDetail = nullptr );
    ~KOPENAPI_RETRY_LATER() override;

    int RetryAfterSec() const { return m_retryAfterSec; }

    KOPENAPI_RESULT Result() const;

private:
    int            m_retryAfterSec;
    nlohmann::json m_detail;
};


/// @brief Reports the document a kiface currently serves for its domain ("sch", "pcb"):
/// { "domain", "path", "project", "unsaved", "mode": "gui"|"headless" }, or null when none
/// is open.  Runs on the main thread.  Feeds /api/v1/status ("documents", "unsaved") and the
/// `documents` method.
using KOPENAPI_DOC_STATUS = std::function<nlohmann::json( KOPENAPI_CONTEXT& aCtx )>;

/// @brief Drops in-memory (headless) documents; called on the main thread when the service stops
using KOPENAPI_DOC_RELEASE = std::function<void()>;


/// @brief A point in a GUI editor's undo stack.  The depth alone does not name it: once the stack
/// is at its limit every new command drops the oldest and the depth stays the same.  The command
/// on top (its serial, PICKED_ITEMS_LIST::GetSerial()) names it while it is in the stack; an empty
/// stack is named by how many commands the frame had dropped by then.
struct KOPENAPI_UNDO_MARK
{
    int      depth = 0;     ///< undo commands at the mark (informational)
    uint64_t top = 0;       ///< serial of the command on top; 0: the stack was empty
    uint64_t dropped = 0;   ///< EDA_BASE_FRAME::GetUndoDropCount() at the mark
};


/// @brief How a kiface rolls its open document back (edit_log / checkpoint / rollback).  GUI editors
/// have an undo stack: a mark names a point in it and rolling back undoes down to it (every change
/// counts: API calls, the router, the netlist updater, edits by hand).  Headless documents have
/// none: a mark is a copy of the document written into a directory, rolling back loads it again
/// under the document's own path (unsaved).  Every function runs on the main thread.
///
/// Rolling back is exact or refused: undoTarget finds where a mark is in the stack now, and
/// nothing is undone in any document unless every document's mark is found.
struct KOPENAPI_DOC_HISTORY
{
    /// @brief The point in the undo stack of the open GUI document; nullopt headless or when none
    std::function<std::optional<KOPENAPI_UNDO_MARK>( KOPENAPI_CONTEXT& )> undoMark;

    /// @brief How deep the undo stack must be to be at aMark now; nullopt and aWhy when the stack no
    /// longer reaches it (dropped at the limit, cleared, or replaced after an undo)
    std::function<std::optional<int>( KOPENAPI_CONTEXT&, const KOPENAPI_UNDO_MARK& aMark, std::string& aWhy )>
            undoTarget;

    /// @brief GUI: undo until the stack is that deep; false when it cannot get there
    std::function<bool( KOPENAPI_CONTEXT&, int aDepth )> undoTo;

    /// @brief Headless: write a copy of the open document into aDir; false when none is open
    std::function<bool( KOPENAPI_CONTEXT&, const std::string& aDir )> snapshot;

    /// @brief Headless: load the copy from aDir back as the open document (its own path, unsaved)
    std::function<bool( KOPENAPI_CONTEXT&, const std::string& aDir )> restore;
};


using KOPENAPI_HANDLER =
        std::function<KOPENAPI_RESULT( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )>;


struct KOPENAPI_METHOD
{
    std::string      name;         ///< [a-z0-9_]+, served at POST /api/v1/{name}
    std::string      summary;
    nlohmann::json   inputSchema;  ///< JSON Schema of the request body (type: object)
    bool             guiOnly;      ///< refused (501) when the process is headless
    KOPENAPI_HANDLER handler;
    int              timeoutSec = 15;   ///< main-thread call cap; raise only for known-slow methods
};


/// @brief One event of a stream (GET /api/v1/events): the SSE event name and its JSON data
struct KOPENAPI_EVENT
{
    std::string    name;
    nlohmann::json data;
};

/// @brief An open stream, polled on its HTTP thread when its topic changed (KopenapiEventNotify),
/// at most ~30 times a second, and once a second otherwise: appends the events due now (the latest
/// state only) and returns false after the last one.  Never touches documents
using KOPENAPI_EVENT_POLL = std::function<bool( std::vector<KOPENAPI_EVENT>& aOut )>;

/// @brief A topic opens a stream from the query parameters (HTTP thread); an error refuses it
using KOPENAPI_EVENT_OPEN = std::function<KOPENAPI_RESULT( const nlohmann::json& aArgs, KOPENAPI_EVENT_POLL& aPoll )>;

struct KOPENAPI_EVENT_TOPIC
{
    std::string         name;
    std::string         summary;
    KOPENAPI_EVENT_OPEN open;
};

/// @brief What a topic's streams show has changed (any thread, cheap): they are woken to poll
KICOMMON_API void KopenapiEventNotify( const std::string& aTopic );

/// @brief The topic's changes so far
KICOMMON_API uint64_t KopenapiEventSeq( const std::string& aTopic );

/// @brief Waits until the topic's changes differ from aSeen, aUntil, or KopenapiEventWakeAll
KICOMMON_API void KopenapiEventWait( const std::string& aTopic, uint64_t aSeen,
                                     std::chrono::steady_clock::time_point aUntil );

/// @brief Wakes every waiting stream (the service stops)
KICOMMON_API void KopenapiEventWakeAll();


class wxWindow;
class wxImage;

/// @brief Reads the pixels of a canvas window a kiface owns (OpenGL schematic / board / 3D canvases),
/// which toolkit window rendering leaves blank.  Returns false for windows it does not own.
using KOPENAPI_CANVAS_CAPTURE = std::function<bool( wxWindow* aWindow, wxImage& aImage )>;

/// @brief Renders a canvas the kiface owns offscreen at a given pixel size (e.g. the 3D viewer at a
/// video's 4K), from the scene it already holds. Returns false for windows it does not own.
/// aSupersample > 1 draws that many times larger and averages down (antialiasing).
using KOPENAPI_CANVAS_RENDER = std::function<bool( wxWindow* aWindow, int aWidth, int aHeight, int aSupersample, wxImage& aImage )>;

/// @brief How a recording's steadycam moves the view to where it was put
struct KOPENAPI_STEADY
{
    enum MODE
    {
        OFF,      ///< jumps stay jumps
        FILTER,   ///< follows through a smoothing filter: no jump, arrives later (time stretched)
        TIMED     ///< a smooth path to the target that arrives exactly durationSeconds later
    };

    MODE   mode = TIMED;
    double smoothSeconds = 0.45;     ///< FILTER: time constant of the two stages together
    double durationSeconds = 0.6;    ///< TIMED: every move takes exactly this long
};

/// @brief Steadycam step for a canvas the kiface owns (recordings): puts the view where the
/// recorded picture should be on its way to the view's real state (as aSteady says; exactly there
/// when aExact: an API animation, already smooth) and sets aRestore, which the recorder calls
/// after the frame to put the real state back. Returns false for windows it does not own.
using KOPENAPI_CANVAS_STEADY = std::function<bool( wxWindow* aWindow, double aDt, const KOPENAPI_STEADY& aSteady,
                                                   bool aExact, std::function<void()>& aRestore )>;


/// @brief Thread-safe: kiface DSOs register while HTTP workers read; readers get copies.
class KICOMMON_API KOPENAPI_REGISTRY
{
public:
    static KOPENAPI_REGISTRY& Get();

    static bool Add( KOPENAPI_METHOD aMethod );

    /// @brief Names registered more than once (the first registration is kept)
    std::vector<std::string> Duplicates() const;

    static bool AddDocumentProvider( const std::string& aDomain, KOPENAPI_DOC_STATUS aStatus,
                                     KOPENAPI_DOC_RELEASE aRelease );

    std::vector<std::pair<std::string, KOPENAPI_DOC_STATUS>> DocumentProviders() const;

    /// @brief Release all in-memory documents (main thread, service stop)
    void ReleaseDocuments() const;

    /// @brief Open documents of all providers (main thread)
    nlohmann::json Documents( KOPENAPI_CONTEXT& aCtx ) const;

    std::vector<KOPENAPI_METHOD> Snapshot() const;

    /// @brief Kifaces register how to read their canvases (window_capture composes them)
    static bool AddCanvasCapture( KOPENAPI_CANVAS_CAPTURE aCapture );
    static bool AddCanvasRender( KOPENAPI_CANVAS_RENDER aRender );
    std::vector<KOPENAPI_CANVAS_RENDER> CanvasRenders() const;

    static bool AddCanvasSteady( KOPENAPI_CANVAS_STEADY aSteady );
    std::vector<KOPENAPI_CANVAS_STEADY> CanvasSteadies() const;

    std::vector<KOPENAPI_CANVAS_CAPTURE> CanvasCaptures() const;

    /// @brief Kifaces register the render benchmark of their editor's canvas ("pcb", "sch"); the
    /// view_benchmark method dispatches to it
    static bool AddViewBenchmark( const std::string& aDomain, KOPENAPI_HANDLER aHandler );

    std::map<std::string, KOPENAPI_HANDLER> ViewBenchmarks() const;

    /// @brief Event streams served at GET /api/v1/events?topic=name
    static bool AddEventTopic( KOPENAPI_EVENT_TOPIC aTopic );

    std::map<std::string, KOPENAPI_EVENT_TOPIC> EventTopics() const;

    /// @brief Kifaces register how their documents roll back (see KOPENAPI_DOC_HISTORY)
    static bool AddDocumentHistory( const std::string& aDomain, KOPENAPI_DOC_HISTORY aHistory );

    std::map<std::string, KOPENAPI_DOC_HISTORY> DocumentHistories() const;

    /// @brief Methods that change documents: logged, and history is marked before each call
    static bool MarkEditing( std::initializer_list<const char*> aNames );

    bool IsEditing( const std::string& aName ) const;

    std::optional<KOPENAPI_METHOD> Find( const std::string& aName ) const;

    /// @brief Keyword search for the MCP `search` tool: query tokens are matched against method
    /// names (weight 3), summaries (2, words run together too: "netclass" finds "net classes")
    /// and input property names (1); best first, then by name.  An empty query returns all methods by name.
    std::vector<KOPENAPI_METHOD> Search( const std::string& aQuery, size_t aLimit,
                                         bool aIncludeGuiOnly ) const;

    /// @brief Names to offer for an unknown method: a slip of one or two letters first, then methods whose
    /// name or summary carries its topic words (the domain prefix and generic verbs such as
    /// list / get / set dropped) within the same domain when it is one, then looser spellings
    std::vector<std::string> Similar( const std::string& aName, size_t aLimit, bool aIncludeGuiOnly ) const;

private:
    KOPENAPI_REGISTRY() = default;

    mutable std::mutex                     m_mutex;
    std::map<std::string, KOPENAPI_METHOD>     m_methods;
    std::vector<std::string>                   m_duplicates;
    std::map<std::string, KOPENAPI_DOC_STATUS>  m_docProviders;
    std::map<std::string, KOPENAPI_DOC_RELEASE> m_docReleasers;
    std::vector<KOPENAPI_CANVAS_CAPTURE>        m_canvasCaptures;
    std::vector<KOPENAPI_CANVAS_RENDER>         m_canvasRenders;
    std::vector<KOPENAPI_CANVAS_STEADY>         m_canvasSteadies;
    std::map<std::string, KOPENAPI_DOC_HISTORY> m_docHistories;
    std::map<std::string, KOPENAPI_HANDLER>     m_viewBenchmarks;
    std::map<std::string, KOPENAPI_EVENT_TOPIC> m_eventTopics;
    std::set<std::string>                       m_editing;
};


#define KOPENAPI_REGISTER_IMPL2( line, ... )                                             \
    static const bool kopenapi_reg_##line = KOPENAPI_REGISTRY::Add( KOPENAPI_METHOD{ __VA_ARGS__ } )
#define KOPENAPI_REGISTER_IMPL( line, ... ) KOPENAPI_REGISTER_IMPL2( line, __VA_ARGS__ )
#define KOPENAPI_REGISTER( ... ) KOPENAPI_REGISTER_IMPL( __LINE__, __VA_ARGS__ )

#define KOPENAPI_REGISTER_DOCUMENTS( aDomain, aStatus, aRelease )                         \
    static const bool kopenapi_docs_reg =                                                  \
            KOPENAPI_REGISTRY::AddDocumentProvider( aDomain, aStatus, aRelease )

#define KOPENAPI_REGISTER_HISTORY( aDomain, aHistory )                                     \
    static const bool kopenapi_history_reg = KOPENAPI_REGISTRY::AddDocumentHistory( aDomain, aHistory )

/// @brief The methods of this file that change documents: KOPENAPI_MARK_EDITING( "sch_wire", ... )
#define KOPENAPI_MARK_EDITING( ... )                                                        \
    static const bool kopenapi_editing_reg = KOPENAPI_REGISTRY::MarkEditing( { __VA_ARGS__ } )

#define KOPENAPI_REGISTER_VIEW_BENCHMARK( aDomain, aHandler )                              \
    static const bool kopenapi_view_bench_reg = KOPENAPI_REGISTRY::AddViewBenchmark( aDomain, aHandler )

#define KOPENAPI_REGISTER_EVENT_TOPIC( aName, aSummary, aOpen )                           \
    static const bool kopenapi_event_topic_reg =                                           \
            KOPENAPI_REGISTRY::AddEventTopic( KOPENAPI_EVENT_TOPIC{ aName, aSummary, aOpen } )

#define KOPENAPI_REGISTER_CANVAS_CAPTURE( aCapture )                                       \
    static const bool kopenapi_canvas_reg = KOPENAPI_REGISTRY::AddCanvasCapture( aCapture )

#define KOPENAPI_REGISTER_CANVAS_RENDER( aRender )                                         \
    static const bool kopenapi_canvas_render_reg = KOPENAPI_REGISTRY::AddCanvasRender( aRender )

#define KOPENAPI_REGISTER_CANVAS_STEADY( aSteady )                                         \
    static const bool kopenapi_canvas_steady_reg = KOPENAPI_REGISTRY::AddCanvasSteady( aSteady )

#endif
