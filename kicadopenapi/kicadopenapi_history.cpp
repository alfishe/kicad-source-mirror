#include "kicadopenapi_history.h"
#include "kicadopenapi_jobs.h"
#include "kicadopenapi_journal.h"

#include <platform.h>

#include <wx/dir.h>
#include <wx/filename.h>
#include <wx/utils.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <deque>
#include <map>


namespace
{

using CLOCK = std::chrono::system_clock;

/// @brief One document's state at a mark: a point in the undo stack (GUI) or a copy (headless)
struct DOC_MARK
{
    std::optional<KOPENAPI_UNDO_MARK> undo;
    std::string                       snapshot;   ///< directory with the copy
};

/// @brief Every open document at one moment
struct MARK
{
    std::map<std::string, DOC_MARK> docs;   ///< by domain
};

struct ENTRY
{
    long           seq;
    std::string    method;
    nlohmann::json args;
    std::string    time;
    int            status = 0;
    std::string    error;
    MARK           before;
};

struct CHECKPOINT
{
    std::string name;
    long        afterSeq;   ///< the log entry it follows (0: before any)
    std::string time;
    MARK        mark;
};


std::string now()
{
    const auto t = CLOCK::to_time_t( CLOCK::now() );
    char       buf[32];
    std::strftime( buf, sizeof( buf ), "%Y-%m-%dT%H:%M:%S", std::localtime( &t ) );
    return buf;
}


/// @brief Arguments as logged: long strings (netlists, images) shortened
nlohmann::json brief( const nlohmann::json& aArgs )
{
    if( aArgs.is_string() && aArgs.get<std::string>().size() > 200 )
        return aArgs.get<std::string>().substr( 0, 80 ) + "... (" + std::to_string( aArgs.get<std::string>().size() ) + " chars)";

    if( aArgs.is_object() || aArgs.is_array() )
    {
        nlohmann::json out = aArgs.is_object() ? nlohmann::json::object() : nlohmann::json::array();

        for( auto it = aArgs.begin(); it != aArgs.end(); ++it )
        {
            if( aArgs.is_object() )
                out[it.key()] = brief( it.value() );
            else
                out.push_back( brief( it.value() ) );
        }

        return out;
    }

    return aArgs;
}

} // namespace


struct KOPENAPI_HISTORY::IMPL
{
    static constexpr size_t MAX_STEPS = 50;   ///< marks kept for step rollback (headless copies)

    std::deque<ENTRY>       log;
    std::vector<CHECKPOINT> checkpoints;
    long                    seq = 0;
    long                    snapshotCount = 0;
    std::optional<ENTRY>    pending;
    std::string             resetReason;
    long                    epoch = 0;   ///< documents opened / closed / reverted / rolled back

    std::string snapshotDir()
    {
        wxFileName dir( wxFileName::GetTempDir(), wxEmptyString );
        dir.AppendDir( wxS( "kicadopenapi-history" ) );

        // once per process: the snapshots of processes that are gone are left-overs
        static bool swept = false;

        if( !swept )
        {
            swept = true;
            std::error_code ec;

            for( const auto& entry : std::filesystem::directory_iterator( dir.GetPath().ToStdString( wxConvUTF8 ), ec ) )
            {
                const long pid = std::atol( entry.path().filename().string().c_str() );

                if( entry.is_directory() && pid > 0 && !kopenapi::platform::ProcessAlive( pid ) )
                    std::filesystem::remove_all( entry.path(), ec );
            }
        }

        dir.AppendDir( wxString::Format( wxS( "%lu" ), wxGetProcessId() ) );
        dir.AppendDir( wxString::Format( wxS( "%ld" ), ++snapshotCount ) );
        dir.Mkdir( wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL );
        return dir.GetPath().ToStdString( wxConvUTF8 );
    }

    static void removeDir( const std::string& aDir )
    {
        if( !aDir.empty() )
            wxFileName::Rmdir( wxString::FromUTF8( aDir ), wxPATH_RMDIR_RECURSIVE );
    }

    static void drop( const MARK& aMark )
    {
        for( const auto& [domain, doc] : aMark.docs )
            removeDir( doc.snapshot );
    }

    MARK mark( KOPENAPI_CONTEXT& aCtx )
    {
        MARK m;

        for( const auto& [domain, history] : KOPENAPI_REGISTRY::Get().DocumentHistories() )
        {
            DOC_MARK doc;

            if( history.undoMark )
                doc.undo = history.undoMark( aCtx );

            if( !doc.undo && history.snapshot )
            {
                std::string dir = snapshotDir();

                if( history.snapshot( aCtx, dir ) )
                    doc.snapshot = dir;
                else
                    removeDir( dir );
            }

            if( doc.undo || !doc.snapshot.empty() )
                m.docs[domain] = doc;
        }

        return m;
    }

    /// @brief Bring every marked document back to the mark; per domain what happened.  Exact or
    /// refused: first every GUI mark is found in its undo stack, and when any is not, nothing is
    /// undone anywhere and aRefused says why (a partly rolled back schematic / board pair would be
    /// a state that never existed).
    nlohmann::json restore( KOPENAPI_CONTEXT& aCtx, const MARK& aMark, std::string& aRefused )
    {
        nlohmann::json done = nlohmann::json::object();
        const auto     histories = KOPENAPI_REGISTRY::Get().DocumentHistories();
        std::map<std::string, int> depths;

        for( const auto& [domain, doc] : aMark.docs )
        {
            auto it = histories.find( domain );

            if( it == histories.end() || !doc.undo )
                continue;

            std::string        why = "this document cannot locate undo marks";
            std::optional<int> depth = it->second.undoTarget ? it->second.undoTarget( aCtx, *doc.undo, why )
                                                             : std::nullopt;

            if( !depth )
            {
                done[domain] = "not reachable: " + why;
                aRefused += ( aRefused.empty() ? "" : "; " ) + domain + ": " + why;
                continue;
            }

            depths[domain] = *depth;
        }

        if( !aRefused.empty() )
            return done;

        for( const auto& [domain, doc] : aMark.docs )
        {
            auto it = histories.find( domain );

            if( it == histories.end() )
                continue;

            bool ok = false;

            if( doc.undo && it->second.undoTo )
                ok = it->second.undoTo( aCtx, depths[domain] );
            else if( !doc.snapshot.empty() && it->second.restore )
                ok = it->second.restore( aCtx, doc.snapshot );

            done[domain] = ok ? ( doc.undo ? "undone in the editor" : "restored from the copy" ) : "failed";
        }

        return done;
    }

    /// @brief Whether a mark taken now would equal aBefore: true only when every document is a GUI
    /// document whose undo stack has the same command on top (headless copies cannot tell cheaply)
    bool unchangedSince( KOPENAPI_CONTEXT& aCtx, const MARK& aBefore )
    {
        if( aBefore.docs.empty() )
            return false;

        const auto histories = KOPENAPI_REGISTRY::Get().DocumentHistories();

        for( const auto& [domain, doc] : aBefore.docs )
        {
            auto it = histories.find( domain );

            if( !doc.undo || it == histories.end() || !it->second.undoMark )
                return false;

            std::optional<KOPENAPI_UNDO_MARK> now = it->second.undoMark( aCtx );

            if( !now || now->top != doc.undo->top || now->dropped != doc.undo->dropped
                || now->depth != doc.undo->depth )
            {
                return false;
            }
        }

        return true;
    }

    /// @brief Keep marks of the last MAX_STEPS steps (headless copies cost disk)
    void trim()
    {
        while( log.size() > MAX_STEPS )
        {
            drop( log.front().before );
            log.pop_front();
        }
    }
};


KOPENAPI_HISTORY& KOPENAPI_HISTORY::Get()
{
    static KOPENAPI_HISTORY* history = new KOPENAPI_HISTORY();
    return *history;
}


KOPENAPI_HISTORY::IMPL* KOPENAPI_HISTORY::impl()
{
    if( !m_impl )
        m_impl = new IMPL();

    return m_impl;
}


void KOPENAPI_HISTORY::BeforeEdit( KOPENAPI_CONTEXT& aCtx, const std::string& aMethod, const nlohmann::json& aArgs )
{
    IMPL* d = impl();

    if( aArgs.is_object() && aArgs.value( "dry_run", false ) )
        return;   // changes nothing

    ENTRY entry;
    entry.seq = ++d->seq;
    entry.method = aMethod;
    entry.args = brief( aArgs );
    entry.time = now();
    entry.before = d->mark( aCtx );
    d->pending = std::move( entry );
}


void KOPENAPI_HISTORY::AfterEdit( KOPENAPI_CONTEXT& aCtx, const KOPENAPI_RESULT& aResult )
{
    IMPL* d = impl();

    for( const auto& [domain, history] : KOPENAPI_REGISTRY::Get().DocumentHistories() )
    {
        if( history.afterEdit )
            history.afterEdit( aCtx );
    }

    if( !d->pending )
        return;

    ENTRY entry = std::move( *d->pending );
    d->pending.reset();
    entry.status = aResult.status;

    // A successful call that changed nothing (a pointer move, a zoom through pcb_action) is no
    // step: it would only push real steps out of the log
    if( aResult.status < 400 && d->unchangedSince( aCtx, entry.before ) )
        return;

    if( aResult.status >= 400 && aResult.body.contains( "error" ) )
        entry.error = aResult.body["error"].value( "message", std::string() );

    d->log.push_back( std::move( entry ) );
    d->trim();
}


KOPENAPI_RESULT KOPENAPI_HISTORY::Log( const nlohmann::json& aArgs ) const
{
    IMPL*          d = const_cast<KOPENAPI_HISTORY*>( this )->impl();
    const size_t   limit = std::max( 1, aArgs.value( "limit", 50 ) );
    nlohmann::json entries = nlohmann::json::array();

    for( auto it = d->log.rbegin(); it != d->log.rend() && entries.size() < limit; ++it )
    {
        nlohmann::json e = { { "seq", it->seq }, { "method", it->method }, { "args", it->args }, { "time", it->time },
                             { "status", it->status } };

        if( !it->error.empty() )
            e["error"] = it->error;

        entries.push_back( e );
    }

    nlohmann::json checkpoints = nlohmann::json::array();

    for( const CHECKPOINT& c : d->checkpoints )
        checkpoints.push_back( { { "name", c.name }, { "after_seq", c.afterSeq } } );

    nlohmann::json result = { { "entries", entries },   // newest first
                              { "steps_kept", d->log.size() },
                              { "checkpoints", checkpoints } };

    if( !d->resetReason.empty() )
        result["history_reset"] = d->resetReason;

    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_RESULT KOPENAPI_HISTORY::Checkpoint( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    IMPL*             d = impl();
    const std::string name = aArgs.value( "name", std::string() );

    if( name.empty() )
        return KOPENAPI_RESULT::Error( 400, "give the checkpoint a 'name'" );

    CHECKPOINT c{ name, d->log.empty() ? 0 : d->log.back().seq, now(), d->mark( aCtx ) };

    if( c.mark.docs.empty() )
        return KOPENAPI_RESULT::Error( 409, "no open document to checkpoint" );

    // Same name again: the newer one wins
    for( auto it = d->checkpoints.begin(); it != d->checkpoints.end(); )
    {
        if( it->name == name )
        {
            IMPL::drop( it->mark );
            it = d->checkpoints.erase( it );
        }
        else
        {
            ++it;
        }
    }

    nlohmann::json docs = nlohmann::json::object();

    for( const auto& [domain, doc] : c.mark.docs )
        docs[domain] = doc.undo ? "editor undo stack" : "copy";

    d->checkpoints.push_back( std::move( c ) );
    return KOPENAPI_RESULT::Ok( { { "name", name }, { "after_seq", d->checkpoints.back().afterSeq }, { "documents", docs } } );
}


KOPENAPI_RESULT KOPENAPI_HISTORY::Checkpoints() const
{
    IMPL*          d = const_cast<KOPENAPI_HISTORY*>( this )->impl();
    nlohmann::json list = nlohmann::json::array();

    for( const CHECKPOINT& c : d->checkpoints )
    {
        nlohmann::json docs = nlohmann::json::array();

        for( const auto& [domain, doc] : c.mark.docs )
            docs.push_back( domain );

        list.push_back( { { "name", c.name }, { "after_seq", c.afterSeq }, { "time", c.time }, { "documents", docs } } );
    }

    return KOPENAPI_RESULT::Ok( { { "checkpoints", list } } );
}


long KOPENAPI_HISTORY::PendingSeq() const
{
    const IMPL* d = const_cast<KOPENAPI_HISTORY*>( this )->impl();
    return d->pending ? d->pending->seq : 0;
}


bool KOPENAPI_HISTORY::Holds( long aSeq, const std::string& aDomain ) const
{
    const IMPL* d = const_cast<KOPENAPI_HISTORY*>( this )->impl();

    for( const ENTRY& e : d->log )
    {
        if( e.seq == aSeq )
            return e.status < 400 && e.before.docs.count( aDomain );
    }

    return false;
}


long KOPENAPI_HISTORY::LastSeq() const
{
    const IMPL* d = const_cast<KOPENAPI_HISTORY*>( this )->impl();

    for( auto it = d->log.rbegin(); it != d->log.rend(); ++it )
    {
        if( it->status < 400 )
            return it->seq;
    }

    return 0;
}


KOPENAPI_RESULT KOPENAPI_HISTORY::RollbackLastInCall( KOPENAPI_CONTEXT& aCtx )
{
    KOPENAPI_RESULT result = Rollback( aCtx, { { "steps", 1 } } );
    IMPL*           d = impl();

    if( result.status == 200 && d->pending )
    {
        IMPL::drop( d->pending->before );
        d->pending->before = d->mark( aCtx );
    }

    return result;
}


KOPENAPI_RESULT KOPENAPI_HISTORY::Rollback( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    IMPL*       d = impl();
    const MARK* target = nullptr;
    long        keepUpTo = 0;   // log entries after this go
    std::string what;

    if( aArgs.contains( "checkpoint" ) )
    {
        const std::string name = aArgs.value( "checkpoint", std::string() );

        for( const CHECKPOINT& c : d->checkpoints )
        {
            if( c.name == name )
            {
                target = &c.mark;
                keepUpTo = c.afterSeq;
            }
        }

        if( !target )
            return KOPENAPI_RESULT::Error( 404, "no checkpoint '" + name + "' (checkpoint_list)" );

        what = "checkpoint " + name;
    }
    else
    {
        const int steps = aArgs.value( "steps", 1 );

        // Steps are successful editing calls, newest first
        std::vector<const ENTRY*> done;

        for( auto it = d->log.rbegin(); it != d->log.rend(); ++it )
        {
            if( it->status < 400 )
                done.push_back( &*it );
        }

        if( steps < 1 || steps > (int) done.size() )
            return KOPENAPI_RESULT::Error( 400, "steps: 1.." + std::to_string( done.size() ) + " (edit_log)" );

        const ENTRY* oldest = done[steps - 1];
        target = &oldest->before;
        keepUpTo = oldest->seq - 1;
        what = std::to_string( steps ) + " step(s)";
    }

    std::string    refused;
    nlohmann::json documents = d->restore( aCtx, *target, refused );

    // Not reachable: nothing was undone, the log and the checkpoints stay as they are
    if( !refused.empty() )
    {
        return KOPENAPI_RESULT{ 409, { { "error", { { "code", 409 },
                                                    { "message", "cannot roll back to " + what
                                                                         + " exactly, nothing was undone: "
                                                                         + refused } } },
                                       { "documents", documents } } };
    }

    d->epoch++;   // the documents moved: calls running during this one are not "unmarked edits"

    // The undone steps leave the log; checkpoints after the target are dropped
    nlohmann::json undone = nlohmann::json::array();

    while( !d->log.empty() && d->log.back().seq > keepUpTo )
    {
        undone.push_back( { { "seq", d->log.back().seq }, { "method", d->log.back().method } } );
        IMPL::drop( d->log.back().before );
        d->log.pop_back();
    }

    for( auto it = d->checkpoints.begin(); it != d->checkpoints.end(); )
    {
        if( it->afterSeq > keepUpTo )
        {
            IMPL::drop( it->mark );
            it = d->checkpoints.erase( it );
        }
        else
        {
            ++it;
        }
    }

    const bool ok = std::none_of( documents.begin(), documents.end(), []( const nlohmann::json& v ) { return v == "failed"; } );

    nlohmann::json result = { { "rolled_back", what }, { "documents", documents }, { "undone", undone } };

    if( !ok )
        return KOPENAPI_RESULT{ 500, { { "error", { { "code", 500 }, { "message", "rollback failed for a document" } } },
                                       { "documents", documents } } };

    return KOPENAPI_RESULT::Ok( result );
}


std::string KOPENAPI_HISTORY::UndoFingerprint( KOPENAPI_CONTEXT& aCtx ) const
{
    IMPL*       d = const_cast<KOPENAPI_HISTORY*>( this )->impl();
    std::string fp = "e" + std::to_string( d->epoch ) + "|";

    for( const auto& [domain, history] : KOPENAPI_REGISTRY::Get().DocumentHistories() )
    {
        if( !history.undoMark )
            continue;

        if( std::optional<KOPENAPI_UNDO_MARK> m = history.undoMark( aCtx ) )
        {
            fp += domain + ":" + std::to_string( m->depth ) + "/" + std::to_string( m->top ) + "/"
                  + std::to_string( m->dropped ) + ";";
        }
    }

    return fp;
}


void KOPENAPI_HISTORY::Reset( const std::string& aDomain, const std::string& aReason )
{
    IMPL* d = impl();
    d->epoch++;

    auto forget = [&]( MARK& aMark )
    {
        auto it = aMark.docs.find( aDomain );

        if( it != aMark.docs.end() )
        {
            IMPL::removeDir( it->second.snapshot );
            aMark.docs.erase( it );
        }
    };

    for( ENTRY& e : d->log )
        forget( e.before );

    if( d->pending )
        forget( d->pending->before );

    for( CHECKPOINT& c : d->checkpoints )
        forget( c.mark );

    d->resetReason = aDomain + ": " + aReason + " (earlier marks of that document dropped)";
}


// ---- methods -------------------------------------------------------------------------------

static KOPENAPI_RESULT h_edit_log( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    return KOPENAPI_HISTORY::Get().Log( aArgs );
}


static KOPENAPI_RESULT h_checkpoint( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return KOPENAPI_HISTORY::Get().Checkpoint( aCtx, aArgs );
}


static KOPENAPI_RESULT h_checkpoint_list( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    return KOPENAPI_HISTORY::Get().Checkpoints();
}


static KOPENAPI_RESULT h_rollback( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    if( const std::string job = KOPENAPI_JOBS::Get().Busy(); !job.empty() )
        return KOPENAPI_RESULT::Error( 409, "job " + job + " is queued or running and owns the documents: "
                                            "job_get waits for it, job_cancel stops it" );

    return KOPENAPI_HISTORY::Get().Rollback( aCtx, aArgs );
}


KOPENAPI_REGISTER( "edit_log",
                   "Log of the API calls that changed the open schematic / board (newest first): method, "
                   "arguments, time, status; the checkpoints between them. Steps for rollback are the "
                   "successful entries",
                   R"json({"type":"object","properties":{"limit":{"type":"integer","default":50}}})json"_json,
                   false, h_edit_log );

KOPENAPI_REGISTER( "checkpoint",
                   "Mark the current state of the open schematic and board under a name, to roll back to "
                   "later (rollback); GUI: a point in the editors' undo stacks, headless: a copy of each "
                   "document",
                   R"json({"type":"object","required":["name"],"properties":{"name":{"type":"string"}}})json"_json,
                   false, h_checkpoint, 120 );

KOPENAPI_REGISTER( "checkpoint_list", "Named checkpoints of this session with the log entry each follows",
                   R"json({"type":"object","properties":{}})json"_json, false, h_checkpoint_list );

KOPENAPI_REGISTER( "rollback",
                   "Roll the open schematic and board back to a checkpoint (by name) or N editing steps "
                   "(edit_log); GUI: undo in the editors (edits by hand since then are undone too), "
                   "headless: the copies are loaded back (unsaved). Later log entries and checkpoints go. "
                   "Exact or refused (409, nothing undone): when an editor's undo stack no longer reaches "
                   "the target (dropped at the undo limit, cleared, or replaced after an undo)",
                   R"json({"type":"object","properties":{
                        "checkpoint":{"type":"string"},
                        "steps":{"type":"integer","default":1}}})json"_json,
                   false, h_rollback, 300 );


KOPENAPI_RESULT KopenapiInvokeInProcess( KOPENAPI_CONTEXT& aCtx, const std::string& aName, const nlohmann::json& aArgs )
{
    const std::optional<KOPENAPI_METHOD> method = KOPENAPI_REGISTRY::Get().Find( aName );

    if( !method )
        return KOPENAPI_RESULT::Error( 404, "no method " + aName );

    const bool editing = KOPENAPI_REGISTRY::Get().IsEditing( aName );

    if( editing && ( KopenapiJobYielding() || !KOPENAPI_JOBS::Get().Busy().empty() ) )
        return KOPENAPI_RESULT::Error( 409, "a call or job is running and owns the documents" );

    KOPENAPI_JOURNAL::SCOPED_OPERATION scope( "ui:" + aName );
    KopenapiCallBegin( aName );

    if( editing )
        KOPENAPI_HISTORY::Get().BeforeEdit( aCtx, aName, aArgs );

    KOPENAPI_RESULT result;

    try
    {
        result = method->handler( aCtx, aArgs );
    }
    catch( const KOPENAPI_RETRY_LATER& e )
    {
        result = e.Result();
    }

    if( editing )
        KOPENAPI_HISTORY::Get().AfterEdit( aCtx, result );

    KopenapiCallEnd();
    return result;
}
