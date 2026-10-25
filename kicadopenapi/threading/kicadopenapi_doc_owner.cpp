/// @file kicadopenapi_doc_owner.cpp
/// @brief The owner strand of one document.

#include "kicadopenapi_doc_owner.h"


KOPENAPI_DOC_BUSY::KOPENAPI_DOC_BUSY( KOPENAPI_CLAIM_INFO aClaim ) :
        std::runtime_error( aClaim.source + " holds the document; edits are refused until it ends" ),
        m_claim( std::move( aClaim ) )
{
}


KOPENAPI_DOC_OWNER::KOPENAPI_DOC_OWNER( std::string aName, KOPENAPI_DOC_GATE::OPTIONS aGateOptions ) :
        m_name( std::move( aName ) ),
        m_gate( aGateOptions )
{
    std::lock_guard<std::mutex> lock( m_queueMutex );
    m_thread = std::thread( [this]() { run(); } );
    m_threadId = m_thread.get_id();
}


KOPENAPI_DOC_OWNER::~KOPENAPI_DOC_OWNER()
{
    Stop();
}


void KOPENAPI_DOC_OWNER::Post( std::function<void()> aTask )
{
    {
        std::lock_guard<std::mutex> lock( m_queueMutex );

        if( m_stopping )
            return;

        m_queue.push_back( std::move( aTask ) );
    }

    m_queueCv.notify_one();
}


void KOPENAPI_DOC_OWNER::run()
{
    for( ;; )
    {
        std::function<void()> task;

        {
            std::unique_lock<std::mutex> lock( m_queueMutex );
            m_queueCv.wait( lock, [this]() { return m_stopping || !m_queue.empty(); } );

            if( m_stopping )
                return;

            task = std::move( m_queue.front() );
            m_queue.pop_front();
        }

        {
            std::lock_guard<std::mutex> lock( m_stateMutex );
            m_stats.tasks++;
        }

        // Every task reports its own failure through its promise
        try
        {
            task();
        }
        catch( ... )
        {
        }
    }
}


void KOPENAPI_DOC_OWNER::Stop()
{
    if( IsOwnerThread() )
        throw std::logic_error( "kicadopenapi: the owner strand cannot stop itself" );

    std::deque<std::function<void()>> dropped;

    {
        std::lock_guard<std::mutex> lock( m_queueMutex );
        m_stopping = true;
        dropped.swap( m_queue );
    }

    m_queueCv.notify_all();

    if( m_thread.joinable() )
        m_thread.join();

    dropped.clear();
    m_deferred.clear();
}


std::future<uint64_t> KOPENAPI_DOC_OWNER::Edit( std::string aSource, EDIT_FN aFn, uint64_t aClaim )
{
    auto edit = std::make_shared<EDIT>();
    edit->source = std::move( aSource );
    edit->fn = std::move( aFn );
    edit->claim = aClaim;

    std::future<uint64_t> future = edit->promise.get_future();

    if( IsOwnerThread() )
        dispatchEdit( edit );
    else
        Post( [this, edit]() { dispatchEdit( edit ); } );

    return future;
}


void KOPENAPI_DOC_OWNER::dispatchEdit( const std::shared_ptr<EDIT>& aEdit )
{
    std::optional<KOPENAPI_CLAIM_INFO> claim = Claim();

    if( claim && aEdit->claim != claim->token )
    {
        if( claim->kind == KOPENAPI_CLAIM_KIND::LONG )
        {
            {
                std::lock_guard<std::mutex> lock( m_stateMutex );
                m_stats.refused++;
            }

            aEdit->promise.set_exception( std::make_exception_ptr( KOPENAPI_DOC_BUSY( *claim ) ) );
            return;
        }

        {
            std::lock_guard<std::mutex> lock( m_stateMutex );
            m_stats.deferred++;
        }

        m_deferred.push_back( aEdit );
        return;
    }

    applyEdit( aEdit );
}


void KOPENAPI_DOC_OWNER::applyEdit( const std::shared_ptr<EDIT>& aEdit )
{
    if( m_inEdit )
    {
        aEdit->promise.set_exception( std::make_exception_ptr(
                std::logic_error( "kicadopenapi: an edit cannot run inside another edit" ) ) );
        return;
    }

    KOPENAPI_CHANGE_SET  changes( aEdit->source );
    KOPENAPI_WRITE_SCOPE scope = m_gate.BeginWrite();
    const uint64_t       from = scope.FromVersion();

    m_inEdit = true;

    try
    {
        aEdit->fn( changes );
    }
    catch( ... )
    {
        m_inEdit = false;
        scope.End();
        aEdit->promise.set_exception( std::current_exception() );
        return;
    }

    m_inEdit = false;

    if( changes.Empty() )
    {
        scope.End();
        aEdit->promise.set_value( from );
        return;
    }

    const uint64_t to = scope.Commit();
    scope.End();
    changes.SetVersions( from, to );

    {
        std::lock_guard<std::mutex> lock( m_stateMutex );
        m_stats.edits++;
    }

    publish( changes );
    aEdit->promise.set_value( to );
}


void KOPENAPI_DOC_OWNER::publish( const KOPENAPI_CHANGE_SET& aChanges )
{
    std::vector<LISTENER> listeners;

    {
        std::lock_guard<std::mutex> lock( m_stateMutex );

        for( const auto& [id, listener] : m_listeners )
            listeners.push_back( listener );
    }

    for( const LISTENER& listener : listeners )
    {
        try
        {
            listener( aChanges );
        }
        catch( ... )
        {
        }
    }
}


std::future<uint64_t> KOPENAPI_DOC_OWNER::BeginClaim( KOPENAPI_CLAIM_KIND aKind, std::string aSource )
{
    auto promise = std::make_shared<std::promise<uint64_t>>();
    std::future<uint64_t> future = promise->get_future();

    auto begin = [this, promise, aKind, source = std::move( aSource )]()
    {
        std::lock_guard<std::mutex> lock( m_stateMutex );

        if( m_claim )
        {
            promise->set_exception( std::make_exception_ptr( KOPENAPI_DOC_BUSY( *m_claim ) ) );
            return;
        }

        m_claim = KOPENAPI_CLAIM_INFO{ m_nextClaim++, aKind, source };
        promise->set_value( m_claim->token );
    };

    if( IsOwnerThread() )
        begin();
    else
        Post( begin );

    return future;
}


std::future<void> KOPENAPI_DOC_OWNER::EndClaim( uint64_t aToken )
{
    auto promise = std::make_shared<std::promise<void>>();
    std::future<void> future = promise->get_future();

    auto end = [this, promise, aToken]()
    {
        {
            std::lock_guard<std::mutex> lock( m_stateMutex );

            if( !m_claim || m_claim->token != aToken )
            {
                promise->set_exception( std::make_exception_ptr(
                        std::invalid_argument( "kicadopenapi: no such claim" ) ) );
                return;
            }

            m_claim.reset();
        }

        std::deque<std::shared_ptr<EDIT>> replay;
        replay.swap( m_deferred );

        for( const std::shared_ptr<EDIT>& edit : replay )
        {
            {
                std::lock_guard<std::mutex> lock( m_stateMutex );
                m_stats.replayed++;
            }

            applyEdit( edit );
        }

        promise->set_value();
    };

    if( IsOwnerThread() )
        end();
    else
        Post( end );

    return future;
}


std::optional<KOPENAPI_CLAIM_INFO> KOPENAPI_DOC_OWNER::Claim() const
{
    std::lock_guard<std::mutex> lock( m_stateMutex );
    return m_claim;
}


uint64_t KOPENAPI_DOC_OWNER::Subscribe( LISTENER aListener )
{
    std::lock_guard<std::mutex> lock( m_stateMutex );
    const uint64_t              id = m_nextListener++;
    m_listeners.emplace( id, std::move( aListener ) );
    return id;
}


void KOPENAPI_DOC_OWNER::Unsubscribe( uint64_t aId )
{
    std::lock_guard<std::mutex> lock( m_stateMutex );
    m_listeners.erase( aId );
}


KOPENAPI_DOC_OWNER::STATS KOPENAPI_DOC_OWNER::Stats() const
{
    std::lock_guard<std::mutex> lock( m_stateMutex );
    return m_stats;
}
