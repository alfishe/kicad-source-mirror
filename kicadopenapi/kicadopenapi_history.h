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
    KOPENAPI_RESULT Checkpoint( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );
    KOPENAPI_RESULT Checkpoints() const;
    KOPENAPI_RESULT Rollback( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs );

    /// @brief The document of aDomain was opened / closed / reverted: its earlier marks no longer
    /// apply (the other domain's marks stay; log entries stay)
    void Reset( const std::string& aDomain, const std::string& aReason );

private:
    KOPENAPI_HISTORY() = default;

    struct IMPL;
    IMPL* impl();
    IMPL* m_impl = nullptr;
};

#endif
