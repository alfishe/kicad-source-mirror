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

#include <functional>
#include <initializer_list>
#include <set>
#include <map>
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


/// @brief Reports the document a kiface currently serves for its domain ("sch", "pcb"):
/// { "domain", "path", "project", "unsaved", "mode": "gui"|"headless" }, or null when none
/// is open.  Runs on the main thread.  Feeds /api/v1/status ("documents", "unsaved") and the
/// `documents` method.
using KOPENAPI_DOC_STATUS = std::function<nlohmann::json( KOPENAPI_CONTEXT& aCtx )>;

/// @brief Drops in-memory (headless) documents; called on the main thread when the service stops
using KOPENAPI_DOC_RELEASE = std::function<void()>;


/// @brief How a kiface rolls its open document back (edit_log / checkpoint / rollback).  GUI editors
/// have an undo stack: a mark is its depth and rolling back undoes down to it (every change
/// counts: API calls, the router, the netlist updater, edits by hand).  Headless documents have
/// none: a mark is a copy of the document written into a directory, rolling back loads it again
/// under the document's own path (unsaved).  Every function runs on the main thread.
struct KOPENAPI_DOC_HISTORY
{
    /// @brief Undo stack depth of the open GUI document; nullopt headless or when none is open
    std::function<std::optional<int>( KOPENAPI_CONTEXT& )> undoDepth;

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


class wxWindow;
class wxImage;

/// @brief Reads the pixels of a canvas window a kiface owns (OpenGL schematic / board / 3D canvases),
/// which toolkit window rendering leaves blank.  Returns false for windows it does not own.
using KOPENAPI_CANVAS_CAPTURE = std::function<bool( wxWindow* aWindow, wxImage& aImage )>;


/// @brief Thread-safe: kiface DSOs register while HTTP workers read; readers get copies.
class KICOMMON_API KOPENAPI_REGISTRY
{
public:
    static KOPENAPI_REGISTRY& Get();

    static bool Add( KOPENAPI_METHOD aMethod );

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

    std::vector<KOPENAPI_CANVAS_CAPTURE> CanvasCaptures() const;

    /// @brief Kifaces register how their documents roll back (see KOPENAPI_DOC_HISTORY)
    static bool AddDocumentHistory( const std::string& aDomain, KOPENAPI_DOC_HISTORY aHistory );

    std::map<std::string, KOPENAPI_DOC_HISTORY> DocumentHistories() const;

    /// @brief Methods that change documents: logged, and history is marked before each call
    static bool MarkEditing( std::initializer_list<const char*> aNames );

    bool IsEditing( const std::string& aName ) const;

    std::optional<KOPENAPI_METHOD> Find( const std::string& aName ) const;

    /// @brief Keyword search for the MCP `search` tool: query tokens are matched against method
    /// names (weight 3), summaries (2) and input property names (1); best first, then by
    /// name.  An empty query returns all methods by name.
    std::vector<KOPENAPI_METHOD> Search( const std::string& aQuery, size_t aLimit,
                                         bool aIncludeGuiOnly ) const;

private:
    KOPENAPI_REGISTRY() = default;

    mutable std::mutex                     m_mutex;
    std::map<std::string, KOPENAPI_METHOD>     m_methods;
    std::map<std::string, KOPENAPI_DOC_STATUS>  m_docProviders;
    std::map<std::string, KOPENAPI_DOC_RELEASE> m_docReleasers;
    std::vector<KOPENAPI_CANVAS_CAPTURE>        m_canvasCaptures;
    std::map<std::string, KOPENAPI_DOC_HISTORY> m_docHistories;
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

#define KOPENAPI_REGISTER_CANVAS_CAPTURE( aCapture )                                       \
    static const bool kopenapi_canvas_reg = KOPENAPI_REGISTRY::AddCanvasCapture( aCapture )

#endif
