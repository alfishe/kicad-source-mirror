#include "kicadopenapi_journal.h"

#include <platform.h>
#include "kicadopenapi_jobs.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <deque>
#include <fstream>
#include <mutex>

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <wx/log.h>
#include <wx/thread.h>

namespace fs = std::filesystem;

static constexpr size_t MAX_ENTRIES = 2000;


namespace
{

struct JOURNAL_STATE
{
    std::mutex                 mutex;
    std::deque<nlohmann::json> entries;
    uint64_t                   lastSeq = 0;
    uint64_t                   errors = 0;
    uint64_t                   warnings = 0;
    std::string                operation = "startup";
    std::string                appName;
    fs::path                   logFile;
    std::ofstream              log;
};


JOURNAL_STATE& state()
{
    static JOURNAL_STATE s;
    return s;
}


const char* levelName( wxLogLevel aLevel )
{
    switch( aLevel )
    {
    case wxLOG_FatalError: return "fatal";
    case wxLOG_Error:      return "error";
    case wxLOG_Warning:    return "warning";
    case wxLOG_Message:    return "message";
    case wxLOG_Status:     return "status";
    case wxLOG_Info:       return "info";
    case wxLOG_Debug:      return "debug";
    default:               return "trace";
    }
}


std::string isoTime( long long aMillis )
{
    // fmt::gmtime is the portable thread-safe UTC conversion
    return fmt::format( "{:%Y-%m-%dT%H:%M:%S}.{:03d}Z",
                        fmt::gmtime( static_cast<std::time_t>( aMillis / 1000 ) ),
                        static_cast<int>( aMillis % 1000 ) );
}


/// @brief The operation of this thread's SCOPED_OPERATION; empty: the process-wide one
thread_local std::string tlOperation;


/// @brief Replaces wxLogGui (dialogs) and wxLogStderr for the whole process
class JOURNAL_LOG_TARGET : public wxLog
{
protected:
    void DoLogRecord( wxLogLevel aLevel, const wxString& aMsg, const wxLogRecordInfo& aInfo ) override
    {
        const long long millis =
                aInfo.timestampMS ? static_cast<long long>( aInfo.timestampMS )
                                  : std::chrono::duration_cast<std::chrono::milliseconds>(
                                            std::chrono::system_clock::now().time_since_epoch() )
                                            .count();

        const std::string message = aMsg.ToStdString( wxConvUTF8 );
        const std::string time = isoTime( millis );
        const char*       level = levelName( aLevel );
        const char*       thread = wxThread::IsMain() ? "main" : KopenapiOnDispatchThread() ? "owner" : "worker";

        JOURNAL_STATE&              s = state();
        std::lock_guard<std::mutex> lock( s.mutex );

        const std::string operation = tlOperation.empty() ? s.operation : tlOperation;
        const std::string line = time + " " + level + " [" + operation + "] " + message;

        if( s.log.is_open() )
            s.log << line << std::endl;

        std::fprintf( stderr, "%s\n", line.c_str() );

        if( aLevel > wxLOG_Warning )
            return;   // only errors and warnings are collected

        nlohmann::json source = nlohmann::json::object();

        if( aInfo.filename )
            source["file"] = aInfo.filename;

        if( aInfo.line )
            source["line"] = aInfo.line;

        if( aInfo.func )
            source["function"] = aInfo.func;

        nlohmann::json entry = { { "seq", ++s.lastSeq },
                                 { "time", time },
                                 { "level", level },
                                 { "message", message },
                                 { "operation", operation },
                                 { "thread", thread },
                                 { "source", source } };

        if( aLevel == wxLOG_Warning )
            s.warnings++;
        else
            s.errors++;

        s.entries.push_back( std::move( entry ) );

        if( s.entries.size() > MAX_ENTRIES )
            s.entries.pop_front();
    }
};

} // namespace


void KOPENAPI_JOURNAL::Install( const std::string& aAppName )
{
    JOURNAL_STATE& s = state();

    {
        std::lock_guard<std::mutex> lock( s.mutex );

        if( !s.appName.empty() )
            return;

        s.appName = aAppName.empty() ? std::string( "kicad" ) : aAppName;

        std::error_code ec;
        fs::path        dir = kopenapi::platform::TempRoot() / "kicad" / "openapi-logs";
        fs::create_directories( dir, ec );

        // Keep a day of logs
        const auto now = fs::file_time_type::clock::now();

        for( const fs::directory_entry& entry : fs::directory_iterator( dir, ec ) )
        {
            if( entry.is_regular_file( ec ) && now - entry.last_write_time( ec ) > std::chrono::hours( 24 ) )
                fs::remove( entry.path(), ec );
        }

        s.logFile = dir / ( s.appName + "-" + std::to_string( kopenapi::platform::CurrentPid() ) + ".log" );
        s.log.open( s.logFile, std::ios::app );
    }

    delete wxLog::SetActiveTarget( new JOURNAL_LOG_TARGET() );

    // once wx has deleted our target at exit it would make a wxLogOutputBest (a message box on MSW)
    // for the records still buffered by threads
    wxLog::DontCreateOnDemand();
}


wxLog* KOPENAPI_JOURNAL::ThreadTarget()
{
    static JOURNAL_LOG_TARGET* target = new JOURNAL_LOG_TARGET();
    return target;
}


void KOPENAPI_JOURNAL::SetOperation( const std::string& aOperation )
{
    std::lock_guard<std::mutex> lock( state().mutex );
    state().operation = aOperation;
}


std::string KOPENAPI_JOURNAL::Operation()
{
    if( !tlOperation.empty() )
        return tlOperation;

    std::lock_guard<std::mutex> lock( state().mutex );
    return state().operation;
}


KOPENAPI_JOURNAL::SCOPED_OPERATION::SCOPED_OPERATION( const std::string& aOperation ) :
        m_previous( tlOperation )
{
    tlOperation = aOperation;
}


KOPENAPI_JOURNAL::SCOPED_OPERATION::~SCOPED_OPERATION()
{
    tlOperation = m_previous;
}


nlohmann::json KOPENAPI_JOURNAL::Entries( uint64_t aSince, size_t aLimit, const std::string& aMinLevel )
{
    JOURNAL_STATE&              s = state();
    std::lock_guard<std::mutex> lock( s.mutex );
    nlohmann::json              out = nlohmann::json::array();
    const bool                  errorsOnly = aMinLevel == "error";

    for( const nlohmann::json& entry : s.entries )
    {
        if( out.size() >= aLimit )
            break;

        if( entry["seq"].get<uint64_t>() <= aSince )
            continue;

        if( errorsOnly && entry["level"] == "warning" )
            continue;

        out.push_back( entry );
    }

    return out;
}


nlohmann::json KOPENAPI_JOURNAL::Summary()
{
    JOURNAL_STATE&              s = state();
    std::lock_guard<std::mutex> lock( s.mutex );

    return { { "errors", s.errors },
             { "warnings", s.warnings },
             { "last_seq", s.lastSeq },
             { "log_file", s.logFile.string() } };
}


void KOPENAPI_JOURNAL::Clear()
{
    JOURNAL_STATE&              s = state();
    std::lock_guard<std::mutex> lock( s.mutex );
    s.entries.clear();
    s.errors = 0;
    s.warnings = 0;
}
