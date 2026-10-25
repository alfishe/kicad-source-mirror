/// @file kicadopenapi_doc_gate.cpp
/// @brief Read leases and the exclusive write scope of one document.

#include "kicadopenapi_doc_gate.h"

#include <algorithm>
#include <vector>


struct KOPENAPI_READ_LEASE::STATE
{
    uint64_t                      id = 0;
    std::string                   holder;
    std::thread::id               thread;
    KOPENAPI_DOC_GATE::CLOCK::time_point granted;
    std::chrono::microseconds     budget{ 0 };
    std::atomic<bool>             revoked{ false };
    bool                          stuckReported = false;   ///< guarded by the gate's mutex
};


KOPENAPI_LEASE_REVOKED::KOPENAPI_LEASE_REVOKED( const std::string& aHolder ) :
        std::runtime_error( "read lease of " + aHolder + " revoked by the document owner" )
{
}


KOPENAPI_READ_LEASE::KOPENAPI_READ_LEASE( KOPENAPI_DOC_GATE* aGate, std::shared_ptr<STATE> aState,
                                          uint64_t aVersion ) :
        m_gate( aGate ),
        m_state( std::move( aState ) ),
        m_version( aVersion )
{
}


KOPENAPI_READ_LEASE::KOPENAPI_READ_LEASE( KOPENAPI_READ_LEASE&& aOther ) noexcept :
        m_gate( aOther.m_gate ),
        m_state( std::move( aOther.m_state ) ),
        m_version( aOther.m_version )
{
    aOther.m_gate = nullptr;
}


KOPENAPI_READ_LEASE& KOPENAPI_READ_LEASE::operator=( KOPENAPI_READ_LEASE&& aOther ) noexcept
{
    if( this != &aOther )
    {
        Release();
        m_gate = aOther.m_gate;
        m_state = std::move( aOther.m_state );
        m_version = aOther.m_version;
        aOther.m_gate = nullptr;
    }

    return *this;
}


KOPENAPI_READ_LEASE::~KOPENAPI_READ_LEASE()
{
    Release();
}


bool KOPENAPI_READ_LEASE::Revoked() const
{
    return m_state && m_state->revoked.load( std::memory_order_acquire );
}


void KOPENAPI_READ_LEASE::Check() const
{
    if( Revoked() )
        throw KOPENAPI_LEASE_REVOKED( m_state->holder );
}


void KOPENAPI_READ_LEASE::Release()
{
    if( !m_gate )
        return;

    KOPENAPI_DOC_GATE* gate = m_gate;
    m_gate = nullptr;
    gate->release( m_state );
    m_state.reset();
}


KOPENAPI_WRITE_SCOPE::KOPENAPI_WRITE_SCOPE( KOPENAPI_DOC_GATE* aGate, uint64_t aFromVersion ) :
        m_gate( aGate ),
        m_fromVersion( aFromVersion )
{
}


KOPENAPI_WRITE_SCOPE::KOPENAPI_WRITE_SCOPE( KOPENAPI_WRITE_SCOPE&& aOther ) noexcept :
        m_gate( aOther.m_gate ),
        m_fromVersion( aOther.m_fromVersion ),
        m_committed( aOther.m_committed )
{
    aOther.m_gate = nullptr;
}


KOPENAPI_WRITE_SCOPE::~KOPENAPI_WRITE_SCOPE()
{
    End();
}


uint64_t KOPENAPI_WRITE_SCOPE::Commit()
{
    m_committed = true;
    return m_fromVersion + 1;
}


void KOPENAPI_WRITE_SCOPE::End()
{
    if( !m_gate )
        return;

    KOPENAPI_DOC_GATE* gate = m_gate;
    m_gate = nullptr;
    gate->endWrite( m_committed );
}


KOPENAPI_DOC_GATE::KOPENAPI_DOC_GATE() :
        KOPENAPI_DOC_GATE( OPTIONS() )
{
}


KOPENAPI_DOC_GATE::KOPENAPI_DOC_GATE( OPTIONS aOptions ) :
        m_options( aOptions )
{
}


KOPENAPI_DOC_GATE::~KOPENAPI_DOC_GATE() = default;


void KOPENAPI_DOC_GATE::SetViolationSink( VIOLATION_SINK aSink )
{
    std::lock_guard<std::mutex> lock( m_mutex );
    m_sink = std::move( aSink );
}


bool KOPENAPI_DOC_GATE::HeldByThisThread() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    const std::thread::id       self = std::this_thread::get_id();

    return std::any_of( m_leases.begin(), m_leases.end(),
                        [&]( const auto& aEntry ) { return aEntry.second->thread == self; } );
}


KOPENAPI_READ_LEASE KOPENAPI_DOC_GATE::Acquire( const std::string& aHolder,
                                                std::optional<std::chrono::microseconds> aBudget,
                                                std::optional<std::chrono::microseconds> aWaitLimit )
{
    std::unique_lock<std::mutex> lock( m_mutex );
    const std::thread::id        self = std::this_thread::get_id();

    // A writer waits for this thread's lease, so a nested lease must not wait for the writer
    const bool nested = std::any_of( m_leases.begin(), m_leases.end(),
                                     [&]( const auto& aEntry ) { return aEntry.second->thread == self; } );

    auto blocked = [&]()
    {
        if( nested )
            return false;

        return m_writing || ( m_writersWaiting > 0 && m_admitted == 0 );
    };

    if( blocked() )
    {
        m_readersWaiting++;

        bool granted = true;

        if( aWaitLimit )
            granted = m_cv.wait_for( lock, *aWaitLimit, [&]() { return !blocked(); } );
        else
            m_cv.wait( lock, [&]() { return !blocked(); } );

        m_readersWaiting--;

        if( granted && m_writersWaiting > 0 && m_admitted > 0 )
            m_admitted--;

        m_admitted = std::min( m_admitted, m_readersWaiting );

        if( !granted )
        {
            lock.unlock();
            m_cv.notify_all();
            return KOPENAPI_READ_LEASE();
        }
    }

    auto state = std::make_shared<KOPENAPI_READ_LEASE::STATE>();
    state->id = m_nextLeaseId++;
    state->holder = aHolder;
    state->thread = self;
    state->granted = CLOCK::now();
    state->budget = aBudget.value_or( m_options.leaseBudget );

    m_leases.emplace( state->id, state );
    m_stats.leasesGranted++;
    m_stats.leasesActive = m_leases.size();

    const uint64_t version = m_version.load( std::memory_order_relaxed );
    const bool     writerWaits = m_writersWaiting > 0;
    lock.unlock();

    // A waiting writer times this lease's budget
    if( writerWaits )
        m_cv.notify_all();

    return KOPENAPI_READ_LEASE( this, std::move( state ), version );
}


void KOPENAPI_DOC_GATE::release( const std::shared_ptr<KOPENAPI_READ_LEASE::STATE>& aState )
{
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        auto held = std::chrono::duration_cast<std::chrono::microseconds>( CLOCK::now() - aState->granted );

        m_leases.erase( aState->id );
        m_stats.leasesActive = m_leases.size();
        m_stats.totalLeaseHold += held;
        m_stats.maxLeaseHold = std::max( m_stats.maxLeaseHold, held );
    }

    m_cv.notify_all();
}


KOPENAPI_WRITE_SCOPE KOPENAPI_DOC_GATE::BeginWrite()
{
    std::unique_lock<std::mutex> lock( m_mutex );

    if( m_writing )
        throw std::logic_error( "kicadopenapi: document writes do not nest" );

    const CLOCK::time_point waitStart = CLOCK::now();
    m_writersWaiting++;

    while( !m_leases.empty() || m_admitted > 0 )
    {
        const CLOCK::time_point               now = CLOCK::now();
        CLOCK::time_point                     wake = CLOCK::time_point::max();
        std::vector<KOPENAPI_LEASE_VIOLATION> found;

        for( auto& [id, lease] : m_leases )
        {
            const CLOCK::time_point due = lease->granted + lease->budget;
            const CLOCK::time_point stuck = due + m_options.stuckAfter;
            auto held = std::chrono::duration_cast<std::chrono::microseconds>( now - lease->granted );

            if( now >= due && !lease->revoked.load( std::memory_order_relaxed ) )
            {
                lease->revoked.store( true, std::memory_order_release );
                m_stats.revoked++;
                found.push_back( { KOPENAPI_LEASE_VIOLATION::KIND::REVOKED, lease->holder, lease->budget,
                                   held, m_version.load( std::memory_order_relaxed ) } );
            }

            if( now >= stuck && !lease->stuckReported )
            {
                lease->stuckReported = true;
                m_stats.stuck++;
                found.push_back( { KOPENAPI_LEASE_VIOLATION::KIND::STUCK, lease->holder, lease->budget,
                                   held, m_version.load( std::memory_order_relaxed ) } );
            }

            if( now < due )
                wake = std::min( wake, due );
            else if( !lease->stuckReported )
                wake = std::min( wake, stuck );
        }

        if( !found.empty() && m_sink )
        {
            VIOLATION_SINK sink = m_sink;
            lock.unlock();

            for( const KOPENAPI_LEASE_VIOLATION& violation : found )
                sink( violation );

            lock.lock();
            continue;
        }

        if( m_leases.empty() || wake == CLOCK::time_point::max() )
            m_cv.wait( lock );
        else
            m_cv.wait_until( lock, wake );
    }

    m_writersWaiting--;
    m_writing = true;

    auto waited = std::chrono::duration_cast<std::chrono::microseconds>( CLOCK::now() - waitStart );
    m_stats.maxWriteWait = std::max( m_stats.maxWriteWait, waited );

    return KOPENAPI_WRITE_SCOPE( this, m_version.load( std::memory_order_relaxed ) );
}


void KOPENAPI_DOC_GATE::endWrite( bool aCommitted )
{
    {
        std::lock_guard<std::mutex> lock( m_mutex );

        if( aCommitted )
            m_version.fetch_add( 1, std::memory_order_acq_rel );

        m_writing = false;
        m_stats.writes++;

        // Readers that waited through this write go before the next one (phase fairness)
        m_admitted = m_readersWaiting;
    }

    m_cv.notify_all();
}


KOPENAPI_DOC_GATE::STATS KOPENAPI_DOC_GATE::Stats() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    STATS                       stats = m_stats;
    stats.readersWaiting = static_cast<uint64_t>( m_readersWaiting );
    stats.writersWaiting = static_cast<uint64_t>( m_writersWaiting );
    return stats;
}
