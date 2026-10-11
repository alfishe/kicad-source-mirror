/// @file kicadopenapi_model_events.cpp
/// @brief Guarded model-change notifications (see kicadopenapi_model_events.h).
#include <kicadopenapi_model_events.h>
#include <kicadopenapi_registry.h>

#include <wx/log.h>
#include <wx/string.h>
#include <wx/thread.h>

#include <chrono>
#include <map>


namespace
{
constexpr double SLOW_MS = 20.0;
constexpr double TOO_SLOW_MS = 200.0;
constexpr int    STRIKES = 3;


struct SUBSCRIBER
{
    std::string                    name;
    std::string                    domain;
    KOPENAPI_MODEL_EVENTS::CALLBACK callback;
    long                           delivered = 0;
    int                            failures = 0;
    int                            slow = 0;
    int                            tooSlow = 0;
    bool                           disabled = false;
};


struct HUB
{
    std::map<int, SUBSCRIBER> subscribers;
    int                       nextId = 1;
    bool                      delivering = false;
    long                      dropped = 0;   ///< events raised during a delivery
};


HUB& hub()
{
    static HUB* s_hub = new HUB();   // process lifetime: tokens may outlive static destruction order
    return *s_hub;
}


void disable( SUBSCRIBER& aSubscriber, const char* aWhy )
{
    aSubscriber.disabled = true;
    wxLogWarning( "model events: subscriber '%s' disabled (%s)", wxString::FromUTF8( aSubscriber.name ),
                  aWhy );
}
} // namespace


KOPENAPI_MODEL_EVENTS::TOKEN::~TOKEN()
{
    hub().subscribers.erase( m_id );
}


std::unique_ptr<KOPENAPI_MODEL_EVENTS::TOKEN> KOPENAPI_MODEL_EVENTS::Subscribe( const std::string& aName,
                                                                               const std::string& aDomain,
                                                                               CALLBACK aCallback )
{
    HUB&      h = hub();
    const int id = h.nextId++;

    h.subscribers[id] = SUBSCRIBER{ aName, aDomain, std::move( aCallback ) };
    return std::make_unique<TOKEN>( id );
}


void KOPENAPI_MODEL_EVENTS::Post( const EVENT& aEvent )
{
    HUB& h = hub();

    if( !wxThread::IsMain() )
        return;

    if( h.delivering )
    {
        if( h.dropped++ == 0 )
            wxLogWarning( "model events: a change raised while notifying was dropped (re-entrant subscriber)" );

        return;
    }

    h.delivering = true;

    // A callback may unsubscribe itself or others: iterate over the ids present now
    std::vector<int> ids;

    for( const auto& [id, subscriber] : h.subscribers )
        ids.push_back( id );

    for( int id : ids )
    {
        auto it = h.subscribers.find( id );

        if( it == h.subscribers.end() )
            continue;

        SUBSCRIBER& s = it->second;

        if( s.disabled || ( !s.domain.empty() && s.domain != aEvent.domain ) )
            continue;

        const auto start = std::chrono::steady_clock::now();
        bool       failed = false;
        const std::string name = s.name;

        // a copy: the callback may destroy its own subscription (and the stored function) while it runs
        const CALLBACK callback = s.callback;

        try
        {
            callback( aEvent );
        }
        catch( const std::exception& e )
        {
            failed = true;
            wxLogWarning( "model events: subscriber '%s' threw: %s", wxString::FromUTF8( name ), e.what() );
        }
        catch( ... )
        {
            failed = true;
            wxLogWarning( "model events: subscriber '%s' threw", wxString::FromUTF8( name ) );
        }

        // the callback may have destroyed its own subscription
        it = h.subscribers.find( id );

        if( it == h.subscribers.end() )
            continue;

        SUBSCRIBER& after = it->second;
        const double ms = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - start ).count();

        after.delivered++;

        if( failed && ++after.failures >= STRIKES )
            disable( after, "threw 3 times" );

        if( ms > SLOW_MS )
        {
            after.slow++;
            wxLogWarning( "model events: subscriber '%s' took %.1f ms", wxString::FromUTF8( after.name ), ms );
        }

        if( ms > TOO_SLOW_MS && ++after.tooSlow >= STRIKES && !after.disabled )
            disable( after, "slower than 200 ms 3 times" );
    }

    h.delivering = false;
}


nlohmann::json KOPENAPI_MODEL_EVENTS::Status()
{
    HUB&           h = hub();
    nlohmann::json list = nlohmann::json::array();

    for( const auto& [id, s] : h.subscribers )
    {
        list.push_back( { { "name", s.name },
                          { "domain", s.domain },
                          { "delivered", s.delivered },
                          { "failures", s.failures },
                          { "slow", s.slow },
                          { "disabled", s.disabled } } );
    }

    return { { "subscribers", list }, { "dropped_reentrant", h.dropped } };
}


static KOPENAPI_RESULT h_model_events( KOPENAPI_CONTEXT&, const nlohmann::json& )
{
    return KOPENAPI_RESULT::Ok( KOPENAPI_MODEL_EVENTS::Status() );
}


KOPENAPI_REGISTER( "model_events",
                   "Subscribers to model change notifications (e.g. the annotation layers): events delivered, "
                   "failures, slow deliveries, disabled ones (3 exceptions or 3 deliveries over 200 ms disable "
                   "a subscriber), events dropped because a subscriber changed the model while notified",
                   R"json({"type":"object","properties":{}})json"_json, false, h_model_events );
