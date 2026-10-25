/// @file kicadopenapi_doc_owner.h
/// @brief The owner strand of one document: the only thread that writes it.
///
/// Tasks run one at a time in the order posted.  An edit runs inside the gate's write scope,
/// records what it changed in a KOPENAPI_CHANGE_SET and, when that is not empty, bumps the
/// version and publishes the change set to the listeners (on the owner thread, after the write
/// ended, so a listener may take leases or read).
///
/// A compute task claims the document as its only writer:
/// - SHORT: edits of other sources are deferred and replayed in order when the claim ends;
///   each is validated by its own function then and answered then.
/// - LONG: edits of other sources are refused at once with KOPENAPI_DOC_BUSY (409, with the
///   claim); reads are served throughout.
/// Claims begin and end in strand order, so edits posted before a claim are applied before it.
#ifndef KICADOPENAPI_DOC_OWNER_H
#define KICADOPENAPI_DOC_OWNER_H

#include "kicadopenapi_change_set.h"
#include "kicadopenapi_doc_gate.h"

#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>


enum class KOPENAPI_CLAIM_KIND
{
    SHORT,
    LONG
};


struct KOPENAPI_CLAIM_INFO
{
    uint64_t            token = 0;
    KOPENAPI_CLAIM_KIND kind = KOPENAPI_CLAIM_KIND::SHORT;
    std::string         source;   ///< the job or method holding the claim
};


/// @brief An edit refused while a long claim holds the document (answered as 409).
class KOPENAPI_THREADING_API KOPENAPI_DOC_BUSY : public std::runtime_error
{
public:
    explicit KOPENAPI_DOC_BUSY( KOPENAPI_CLAIM_INFO aClaim );

    const KOPENAPI_CLAIM_INFO& Claim() const { return m_claim; }

private:
    KOPENAPI_CLAIM_INFO m_claim;
};


class KOPENAPI_THREADING_API KOPENAPI_DOC_OWNER
{
public:
    using EDIT_FN = std::function<void( KOPENAPI_CHANGE_SET& )>;
    using LISTENER = std::function<void( const KOPENAPI_CHANGE_SET& )>;

    struct STATS
    {
        uint64_t tasks = 0;
        uint64_t edits = 0;      ///< edits applied with a non-empty change set
        uint64_t deferred = 0;   ///< edits deferred by a short claim
        uint64_t replayed = 0;   ///< deferred edits run when their claim ended
        uint64_t refused = 0;    ///< edits refused by a long claim
    };

    explicit KOPENAPI_DOC_OWNER( std::string aName,
                                 KOPENAPI_DOC_GATE::OPTIONS aGateOptions = KOPENAPI_DOC_GATE::OPTIONS() );
    ~KOPENAPI_DOC_OWNER();

    KOPENAPI_DOC_OWNER( const KOPENAPI_DOC_OWNER& ) = delete;
    KOPENAPI_DOC_OWNER& operator=( const KOPENAPI_DOC_OWNER& ) = delete;

    const std::string& Name() const { return m_name; }

    KOPENAPI_DOC_GATE& Gate() { return m_gate; }

    uint64_t Version() const { return m_gate.Version(); }

    bool IsOwnerThread() const { return std::this_thread::get_id() == m_threadId; }

    /// @brief Queues a task for the strand.  Tasks run in posting order.
    void Post( std::function<void()> aTask );

    /// @brief Runs aFn on the strand (inline when called on it) and returns its result.
    template <typename FN>
    auto Read( FN&& aFn ) -> std::future<std::invoke_result_t<FN>>
    {
        using RESULT = std::invoke_result_t<FN>;

        auto task = std::make_shared<std::packaged_task<RESULT()>>( std::forward<FN>( aFn ) );
        std::future<RESULT> future = task->get_future();

        if( IsOwnerThread() )
            ( *task )();
        else
            Post( [task]() { ( *task )(); } );

        return future;
    }

    /// @brief Applies an edit on the strand.
    /// @param aSource recorded in the change set ("api:<call>", "job:<id>", "gui")
    /// @param aFn records every item it adds, changes or removes; throws to abandon the edit
    ///            (it must leave the document as it found it then)
    /// @param aClaim the token of the claim the edit belongs to, 0 for none
    /// @return the document version after the edit
    std::future<uint64_t> Edit( std::string aSource, EDIT_FN aFn, uint64_t aClaim = 0 );

    /// @brief Claims the document for one writer, in strand order.
    /// @return the claim's token; KOPENAPI_DOC_BUSY when another claim holds it
    std::future<uint64_t> BeginClaim( KOPENAPI_CLAIM_KIND aKind, std::string aSource );

    /// @brief Ends a claim in strand order; deferred edits run before anything posted later.
    std::future<void> EndClaim( uint64_t aToken );

    std::optional<KOPENAPI_CLAIM_INFO> Claim() const;

    uint64_t Subscribe( LISTENER aListener );
    void     Unsubscribe( uint64_t aId );

    /// @brief Stops after the running task; tasks not started are dropped and their futures
    /// report a broken promise.  Not callable from the strand itself.
    void Stop();

    STATS Stats() const;

private:
    struct EDIT
    {
        std::string             source;
        EDIT_FN                 fn;
        uint64_t                claim = 0;
        std::promise<uint64_t>  promise;
    };

    void run();
    void dispatchEdit( const std::shared_ptr<EDIT>& aEdit );
    void applyEdit( const std::shared_ptr<EDIT>& aEdit );
    void publish( const KOPENAPI_CHANGE_SET& aChanges );

    std::string                        m_name;
    KOPENAPI_DOC_GATE                  m_gate;

    mutable std::mutex                 m_queueMutex;
    std::condition_variable            m_queueCv;
    std::deque<std::function<void()>>  m_queue;
    bool                               m_stopping = false;

    std::thread                        m_thread;
    std::thread::id                    m_threadId;
    bool                               m_inEdit = false;   ///< strand only

    mutable std::mutex                 m_stateMutex;       ///< claim, listeners, stats
    std::optional<KOPENAPI_CLAIM_INFO> m_claim;
    uint64_t                           m_nextClaim = 1;
    std::deque<std::shared_ptr<EDIT>>  m_deferred;         ///< strand only
    std::map<uint64_t, LISTENER>       m_listeners;
    uint64_t                           m_nextListener = 1;
    STATS                              m_stats;
};

#endif
