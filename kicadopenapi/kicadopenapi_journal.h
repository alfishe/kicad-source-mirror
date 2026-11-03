/// @file kicadopenapi_journal.h
/// @brief kicadopenapi error journal.
///
/// A process-wide wxLog target that replaces the GUI log dialogs: nothing is ever shown in a
/// window.  Errors and warnings are collected (with the operation that caused them, the
/// thread, and the source location) and served through the `errors` method and
/// /api/v1/status; every record, at any level, is also written to a log file and stderr.
#ifndef KICADOPENAPI_JOURNAL_H
#define KICADOPENAPI_JOURNAL_H

#include <cstdint>
#include <string>

#include <json_common.h>
#include <kicommon.h>

class wxLog;


class KICOMMON_API KOPENAPI_JOURNAL
{
public:
    /// @brief Install as the active wxLog target (idempotent).  Call as early as possible on the
    /// main thread; the current operation starts as "startup".
    static void Install( const std::string& aAppName );

    /// @brief A journal log target for wxLog::SetThreadActiveTarget on a thread that works on the
    /// documents (records go to the journal at once, not buffered for the main thread); never
    /// deleted
    static wxLog* ThreadTarget();

    /// @brief Records an error (or a warning) of aOperation at once, from any thread (wxLog on a
    /// thread without its own target would hold it until the main loop flushes)
    static void Record( bool aError, const std::string& aOperation, const std::string& aMessage );

    /// @brief Operation that log records are attributed to when their thread is in no SCOPED_OPERATION
    static void SetOperation( const std::string& aOperation );

    /// @brief The operation this thread's log records are attributed to now
    static std::string Operation();

    /// @brief Sets the operation of this thread's log records for a scope and restores the previous one
    class KICOMMON_API SCOPED_OPERATION
    {
    public:
        explicit SCOPED_OPERATION( const std::string& aOperation );
        ~SCOPED_OPERATION();

    private:
        std::string m_previous;
    };

    /// @brief Collected records with seq > aSince, oldest first, at most aLimit, of at least the
    /// given severity ("warning" or "error").
    static nlohmann::json Entries( uint64_t aSince, size_t aLimit, const std::string& aMinLevel );

    /// @brief { errors, warnings, last_seq, log_file }
    static nlohmann::json Summary();

    static void Clear();
};


/// @brief The journal from here on: what a load reported, for the API's load result
class KICOMMON_API KOPENAPI_JOURNAL_MARK
{
public:
    KOPENAPI_JOURNAL_MARK();

    /// @brief The errors this operation recorded since the mark into aBody["load_errors"], the
    /// warnings into aBody["warnings"] (their messages, each once, after what the arrays hold;
    /// nothing added when there are none)
    void AddTo( nlohmann::json& aBody ) const;

private:
    uint64_t    m_seq;
    std::string m_operation;
};

#endif
