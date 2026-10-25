/// @file kicadopenapi_doc_gate.h
/// @brief Document version, read leases and the owner's exclusive write scope.
///
/// One gate per document.  The owner thread writes the live document only inside a
/// KOPENAPI_WRITE_SCOPE; any other thread reads it only while it holds a KOPENAPI_READ_LEASE.
/// Leases are shared, the write is exclusive, and a waiting writer holds back new leases (writer
/// preference), except a nested lease on a thread that already holds one (no self-deadlock) and
/// the leases that waited through the previous write (phase fairness: back-to-back writes never
/// starve readers).
///
/// A lease has a budget.  When a writer waits on a lease past its budget the gate revokes it
/// and reports a violation; the holder sees Revoked() (Check() throws) at its next check and
/// releases.  A lease still held long after its budget is reported once more as stuck.  The
/// write starts only after the last lease is released: a revoked lease is never written under.
///
/// The version counts committed write scopes; it never changes while any lease is held.
#ifndef KICADOPENAPI_DOC_GATE_H
#define KICADOPENAPI_DOC_GATE_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#ifndef KOPENAPI_THREADING_API
#include <kicommon.h>
#define KOPENAPI_THREADING_API KICOMMON_API
#endif


/// @brief A lease that held the document past its budget while a writer waited.
struct KOPENAPI_LEASE_VIOLATION
{
    enum class KIND
    {
        REVOKED, ///< budget exceeded while a writer waited: revoked
        STUCK    ///< still held after the stuck limit following its budget
    };

    KIND                      kind = KIND::REVOKED;
    std::string               holder;  ///< who took the lease (method, job, worker)
    std::chrono::microseconds budget{ 0 };
    std::chrono::microseconds held{ 0 }; ///< hold time when reported
    uint64_t                  version = 0;
};


/// @brief Thrown by KOPENAPI_READ_LEASE::Check() once the owner revoked the lease.
class KOPENAPI_THREADING_API KOPENAPI_LEASE_REVOKED : public std::runtime_error
{
public:
    explicit KOPENAPI_LEASE_REVOKED( const std::string& aHolder );
};


class KOPENAPI_DOC_GATE;


/// @brief Shared read access to the live document; move-only, released on destruction.
class KOPENAPI_THREADING_API KOPENAPI_READ_LEASE
{
public:
    KOPENAPI_READ_LEASE() = default;
    KOPENAPI_READ_LEASE( KOPENAPI_READ_LEASE&& aOther ) noexcept;
    KOPENAPI_READ_LEASE& operator=( KOPENAPI_READ_LEASE&& aOther ) noexcept;
    KOPENAPI_READ_LEASE( const KOPENAPI_READ_LEASE& ) = delete;
    KOPENAPI_READ_LEASE& operator=( const KOPENAPI_READ_LEASE& ) = delete;
    ~KOPENAPI_READ_LEASE();

    /// @return false when not granted (wait limit hit) or already released
    bool Valid() const { return m_gate != nullptr; }

    explicit operator bool() const { return Valid(); }

    /// @return the document version the lease reads; constant while it is held
    uint64_t Version() const { return m_version; }

    /// @return true once the owner revoked the lease: stop reading and release
    bool Revoked() const;

    /// @brief Throws KOPENAPI_LEASE_REVOKED when revoked; call at every unit of reading.
    void Check() const;

    void Release();

private:
    friend class KOPENAPI_DOC_GATE;

    struct STATE;

    KOPENAPI_READ_LEASE( KOPENAPI_DOC_GATE* aGate, std::shared_ptr<STATE> aState, uint64_t aVersion );

    KOPENAPI_DOC_GATE*     m_gate = nullptr;
    std::shared_ptr<STATE> m_state;
    uint64_t               m_version = 0;
};


/// @brief The owner's exclusive write; bumps the version on Commit(), not when abandoned.
class KOPENAPI_THREADING_API KOPENAPI_WRITE_SCOPE
{
public:
    KOPENAPI_WRITE_SCOPE( KOPENAPI_WRITE_SCOPE&& aOther ) noexcept;
    KOPENAPI_WRITE_SCOPE( const KOPENAPI_WRITE_SCOPE& ) = delete;
    KOPENAPI_WRITE_SCOPE& operator=( const KOPENAPI_WRITE_SCOPE& ) = delete;
    KOPENAPI_WRITE_SCOPE& operator=( KOPENAPI_WRITE_SCOPE&& ) = delete;
    ~KOPENAPI_WRITE_SCOPE();

    /// @return the version before this write
    uint64_t FromVersion() const { return m_fromVersion; }

    /// @brief Marks the write as changing the document; the version is bumped on End().
    /// @return the new version
    uint64_t Commit();

    /// @brief Ends the write and lets leases in again (also on destruction).
    void End();

private:
    friend class KOPENAPI_DOC_GATE;

    KOPENAPI_WRITE_SCOPE( KOPENAPI_DOC_GATE* aGate, uint64_t aFromVersion );

    KOPENAPI_DOC_GATE* m_gate = nullptr;
    uint64_t           m_fromVersion = 0;
    bool               m_committed = false;
};


class KOPENAPI_THREADING_API KOPENAPI_DOC_GATE
{
public:
    using CLOCK = std::chrono::steady_clock;
    using VIOLATION_SINK = std::function<void( const KOPENAPI_LEASE_VIOLATION& )>;

    struct OPTIONS
    {
        std::chrono::microseconds leaseBudget{ 20000 };    ///< default per-lease budget
        std::chrono::microseconds stuckAfter{ 1000000 };   ///< after the budget, reported stuck
    };

    struct STATS
    {
        uint64_t                  leasesGranted = 0;
        uint64_t                  leasesActive = 0;
        uint64_t                  readersWaiting = 0;      ///< blocked in Acquire() now
        uint64_t                  writersWaiting = 0;      ///< blocked in BeginWrite() now
        uint64_t                  writes = 0;              ///< write scopes ended
        uint64_t                  revoked = 0;
        uint64_t                  stuck = 0;
        std::chrono::microseconds maxLeaseHold{ 0 };
        std::chrono::microseconds totalLeaseHold{ 0 };
        std::chrono::microseconds maxWriteWait{ 0 };       ///< longest wait of a writer on leases
    };

    KOPENAPI_DOC_GATE();
    explicit KOPENAPI_DOC_GATE( OPTIONS aOptions );
    ~KOPENAPI_DOC_GATE();

    KOPENAPI_DOC_GATE( const KOPENAPI_DOC_GATE& ) = delete;
    KOPENAPI_DOC_GATE& operator=( const KOPENAPI_DOC_GATE& ) = delete;

    /// @brief Receives violations, called without the gate's lock on the writer's thread.
    void SetViolationSink( VIOLATION_SINK aSink );

    uint64_t Version() const { return m_version.load( std::memory_order_acquire ); }

    /// @brief Waits while a write runs or is pending, then grants a shared lease.
    /// @param aHolder named in violations
    /// @param aBudget the lease's budget (default OPTIONS::leaseBudget)
    /// @param aWaitLimit longest wait for the grant; an invalid lease when it passes
    KOPENAPI_READ_LEASE Acquire( const std::string& aHolder,
                                 std::optional<std::chrono::microseconds> aBudget = std::nullopt,
                                 std::optional<std::chrono::microseconds> aWaitLimit = std::nullopt );

    /// @brief Waits for every lease to be released (revoking those past their budget), then
    /// grants the exclusive write.  Writes do not nest.
    KOPENAPI_WRITE_SCOPE BeginWrite();

    /// @return true while the calling thread holds a lease of this gate
    bool HeldByThisThread() const;

    STATS Stats() const;

    const OPTIONS& Options() const { return m_options; }

private:
    friend class KOPENAPI_READ_LEASE;
    friend class KOPENAPI_WRITE_SCOPE;

    void release( const std::shared_ptr<KOPENAPI_READ_LEASE::STATE>& aState );
    void endWrite( bool aCommitted );

    OPTIONS                                                         m_options;
    mutable std::mutex                                              m_mutex;
    std::condition_variable                                         m_cv;
    std::map<uint64_t, std::shared_ptr<KOPENAPI_READ_LEASE::STATE>> m_leases;
    uint64_t                                                        m_nextLeaseId = 1;
    bool                                                            m_writing = false;
    int                                                             m_writersWaiting = 0;
    int                                                             m_readersWaiting = 0;
    int                                                             m_admitted = 0;   ///< readers let in before the next write
    std::atomic<uint64_t>                                           m_version{ 0 };
    STATS                                                           m_stats;
    VIOLATION_SINK                                                  m_sink;
};

#endif
