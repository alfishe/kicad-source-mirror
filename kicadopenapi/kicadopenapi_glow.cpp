#include "kicadopenapi_glow.h"

#include <wx/event.h>
#include <wx/timer.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <set>


class KOPENAPI_GLOW_MANAGER::IMPL : public wxEvtHandler
{
public:
    using CLOCK = std::chrono::steady_clock;
    using KEY = std::pair<KOPENAPI_GLOW_TARGET*, KIID>;

    static constexpr int TICK_MS = 50;

    struct ENTRY
    {
        KEY               key;
        CLOCK::time_point start;
        bool              lit = false;   ///< brightened (a staggered entry lights up at its start)
    };

    IMPL()
    {
        m_timer.SetOwner( this );
        Bind( wxEVT_TIMER, &IMPL::onTick, this );
    }

    void Add( KOPENAPI_GLOW_TARGET* aTarget, const std::vector<KIID>& aItems, int aStaggerMs )
    {
        if( !aTarget || aItems.empty() || !aTarget->Alive() )
            return;

        if( !m_active.count( aTarget ) )
        {
            aTarget->Begin();
            m_active.insert( aTarget );
        }

        const CLOCK::time_point now = CLOCK::now();

        // Staggered entries queue behind what is still waiting to light up
        CLOCK::time_point next = now;

        if( aStaggerMs > 0 && !m_entries.empty() && m_entries.back().start > now )
            next = m_entries.back().start + std::chrono::milliseconds( aStaggerMs );

        for( const KIID& id : aItems )
        {
            ENTRY entry{ { aTarget, id }, next, false };

            if( next <= now )
            {
                if( !aTarget->Brighten( id, true ) )
                    continue;

                entry.lit = true;
            }

            m_entries.push_back( entry );
            m_newest[{ aTarget, id }] = next;

            if( aStaggerMs > 0 )
                next += std::chrono::milliseconds( aStaggerMs );
        }

        redraw( now );

        if( !m_entries.empty() && !m_timer.IsRunning() )
            m_timer.Start( TICK_MS );
    }

    size_t Count() const { return m_newest.size(); }

private:
    /// Halos of every editor with something glowing (or just emptied)
    void redraw( CLOCK::time_point aNow )
    {
        std::map<KOPENAPI_GLOW_TARGET*, std::vector<std::pair<KIID, double>>> halos;

        for( KOPENAPI_GLOW_TARGET* target : m_active )
            halos[target];

        for( const auto& [key, start] : m_newest )
        {
            if( start > aNow )
                continue;   // not lit yet

            const double age = std::chrono::duration<double, std::milli>( aNow - start ).count() / GLOW_MS;
            halos[key.first].emplace_back( key.second, std::max( 0.0, 1.0 - age ) );
        }

        for( auto& [target, items] : halos )
        {
            if( target->Alive() )
                target->Halo( items );
        }
    }

    void onTick( wxTimerEvent& )
    {
        const CLOCK::time_point now = CLOCK::now();

        // Light up staggered entries whose turn has come
        for( ENTRY& entry : m_entries )
        {
            if( entry.lit || entry.start > now )
                continue;

            entry.lit = true;

            auto newest = m_newest.find( entry.key );

            if( newest != m_newest.end() && newest->second == entry.start && entry.key.first->Alive() )
                entry.key.first->Brighten( entry.key.second, true );
        }

        // Dim in the order the glows started
        while( !m_entries.empty() && now - m_entries.front().start >= std::chrono::milliseconds( GLOW_MS ) )
        {
            const ENTRY entry = m_entries.front();
            m_entries.pop_front();

            auto newest = m_newest.find( entry.key );

            if( newest == m_newest.end() || newest->second != entry.start )
                continue;   // glowed again later: its newer entry dims it

            m_newest.erase( newest );

            if( entry.key.first->Alive() )
                entry.key.first->Brighten( entry.key.second, false );
        }

        redraw( now );

        // Editors with nothing left: restore their colours
        for( auto it = m_active.begin(); it != m_active.end(); )
        {
            const bool busy = std::any_of( m_newest.begin(), m_newest.end(),
                                           [&]( const auto& n ) { return n.first.first == *it; } );

            if( busy )
            {
                ++it;
                continue;
            }

            if( ( *it )->Alive() )
                ( *it )->End();

            it = m_active.erase( it );
        }

        if( m_entries.empty() )
            m_timer.Stop();
    }

    wxTimer                                 m_timer;
    std::deque<ENTRY>                       m_entries;
    std::map<KEY, CLOCK::time_point>        m_newest;
    std::set<KOPENAPI_GLOW_TARGET*>         m_active;
};


KOPENAPI_GLOW_MANAGER& KOPENAPI_GLOW_MANAGER::Get()
{
    // Never destroyed: its timer is idle whenever nothing glows
    static KOPENAPI_GLOW_MANAGER* manager = new KOPENAPI_GLOW_MANAGER();
    return *manager;
}


KOPENAPI_GLOW_MANAGER::KOPENAPI_GLOW_MANAGER() : m_impl( new IMPL() )
{
}


void KOPENAPI_GLOW_MANAGER::Add( KOPENAPI_GLOW_TARGET* aTarget, const std::vector<KIID>& aItems, int aStaggerMs )
{
    m_impl->Add( aTarget, aItems, aStaggerMs );
}


size_t KOPENAPI_GLOW_MANAGER::Count() const
{
    return m_impl->Count();
}
