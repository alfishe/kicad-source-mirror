/// @file kicadopenapi_history.h
/// @brief kicadopenapi edit history: a log of the API calls that changed documents, a mark before each
/// of them, named checkpoints, and rollback to a checkpoint or N steps back - for the schematic
/// and the board together, GUI and headless alike (KOPENAPI_DOC_HISTORY per kiface: the editor's
/// undo stack in the GUI, document copies headless).  Main thread only.
#ifndef KICADOPENAPI_HISTORY_H
#define KICADOPENAPI_HISTORY_H

#include <kicadopenapi_registry.h>
#include <kicommon.h>


class KICOMMON_API KOPENAPI_HISTORY
{
public:
    static KOPENAPI_HISTORY& Get();

    /// @brief Before an editing call: mark every open document (the step's "before" state)
    void BeforeEdit( KOPENAPI_CONTEXT& aCtx, const std::string& aMethod, const nlohmann::json& aArgs );

    /// @brief After it: the log entry gets the outcome (failed calls are logged, but are no step)
    void AfterEdit( KOPENAPI_CONTEXT& aCtx, const KOPENAPI_RESULT& aResult );

    KOPENAPI_RESULT Log( const nlohmann::json& aArgs ) const;

    /// @brief The undo-stack points of the open GUI documents and the history epoch (bumped when a
    /// document is opened, closed, reverted or rolled back) as one string.  A call not marked
    /// editing must leave it unchanged unless the epoch moved (the service checks and reports drift)
    std::string UndoFingerprint( KOPENAPI_CONTEXT& aCtx ) const;
    KOPENAPI_RESULT Checkpoint( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );
    KOPENAPI_RESULT Checkpoints() const;
    KOPENAPI_RESULT Rollback( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );

    /// @brief The editing call running now: its log entry's number once it ends (0: none)
    long PendingSeq() const;

    /// @brief The last successful editing call logged (0: none)
    long LastSeq() const;

    /// @brief The successful editing call aSeq is still logged with its mark of aDomain: not rolled
    /// back, the document not opened / closed / reverted since
    bool Holds( long aSeq, const std::string& aDomain ) const;

    /// @brief From inside an editing call: rolls the last logged call back, and the running call's
    /// "before" becomes the state rolled back to (undoing it later goes there)
    KOPENAPI_RESULT RollbackLastInCall( KOPENAPI_CONTEXT& aCtx );

    /// @brief The document of aDomain was opened / closed / reverted: its earlier marks no longer
    /// apply (the other domain's marks stay; log entries stay)
    void Reset( const std::string& aDomain, const std::string& aReason );

private:
    KOPENAPI_HISTORY() = default;

    struct IMPL;
    IMPL* impl();
    IMPL* m_impl = nullptr;
};

/// @brief Main thread: an API method run from KiCad's own UI (a key in an overlay) as an API call
/// is: an editing one marked and logged (edit_log, rollback), its progress shown (api_trace), its log
/// records attributed to "ui:<name>"
KICOMMON_API KOPENAPI_RESULT KopenapiInvokeInProcess( KOPENAPI_CONTEXT& aCtx, const std::string& aName,
                                                      const nlohmann::json& aArgs );

#endif
