/// @file kicadopenapi_call_ticket.h
/// @brief Who decides a queued main-thread call: the main thread starting it or the HTTP worker
/// giving up on it.  Exactly one of them wins, so a call whose caller was told it timed out
/// before it started never runs afterwards (a retried edit is not applied twice).
#ifndef KICADOPENAPI_CALL_TICKET_H
#define KICADOPENAPI_CALL_TICKET_H

#include <atomic>


class KOPENAPI_CALL_TICKET
{
public:
    enum STATE
    {
        QUEUED,
        STARTED,     ///< the main thread took it: it runs to the end
        ABANDONED    ///< the caller gave up first: it never runs
    };

    /// @brief Main thread, before running the call: true when it may run
    bool Start()
    {
        int expected = QUEUED;
        return m_state.compare_exchange_strong( expected, STARTED );
    }

    /// @brief Caller, on timeout: true when the call had not started and now never will
    bool Abandon()
    {
        int expected = QUEUED;
        return m_state.compare_exchange_strong( expected, ABANDONED );
    }

    STATE State() const { return static_cast<STATE>( m_state.load() ); }

private:
    std::atomic<int> m_state{ QUEUED };
};

#endif
