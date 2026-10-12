/// @file kicadopenapi_progress.cpp
/// @brief Lifetime of KiCad's progress windows (see kicadopenapi_progress.h).
#include <kicadopenapi_progress.h>

#include <chrono>
#include <ctime>
#include <deque>
#include <map>
#include <mutex>


namespace
{
using CLOCK = std::chrono::steady_clock;

const CLOCK::time_point g_start = CLOCK::now();

struct ENTRY
{
    uint64_t                         seq = 0;
    std::string                      title;
    double                           openedMs = 0.0;
    double                           closedMs = -1.0;
    std::chrono::system_clock::time_point openedWall;
};

std::mutex                   g_mutex;
std::map<const void*, ENTRY> g_open;
std::deque<ENTRY>            g_history;
uint64_t                     g_seq = 0;
constexpr size_t             HISTORY = 200;


double msNow()
{
    return std::chrono::duration<double, std::milli>( CLOCK::now() - g_start ).count();
}


std::string iso( std::chrono::system_clock::time_point aTime )
{
    const std::time_t t = std::chrono::system_clock::to_time_t( aTime );
    std::tm           tm{};
#if defined( _WIN32 )
    gmtime_s( &tm, &t );
#else
    gmtime_r( &t, &tm );
#endif
    char buf[32];
    std::strftime( buf, sizeof( buf ), "%Y-%m-%dT%H:%M:%SZ", &tm );
    return buf;
}


nlohmann::json toJson( const ENTRY& aEntry )
{
    nlohmann::json j = { { "seq", aEntry.seq },
                         { "title", aEntry.title },
                         { "opened_ms", aEntry.openedMs },
                         { "opened_at", iso( aEntry.openedWall ) } };

    if( aEntry.closedMs >= 0.0 )
    {
        j["closed_ms"] = aEntry.closedMs;
        j["duration_ms"] = aEntry.closedMs - aEntry.openedMs;
    }
    else
    {
        j["open_for_ms"] = msNow() - aEntry.openedMs;
    }

    return j;
}
} // namespace


void KOPENAPI_PROGRESS::Opened( const void* aKey, const wxString& aTitle )
{
    std::lock_guard<std::mutex> lock( g_mutex );
    ENTRY&                      e = g_open[aKey];
    e.seq = ++g_seq;
    e.title = aTitle.ToStdString( wxConvUTF8 );
    e.openedMs = msNow();
    e.openedWall = std::chrono::system_clock::now();
}


void KOPENAPI_PROGRESS::Closed( const void* aKey )
{
    std::lock_guard<std::mutex> lock( g_mutex );
    auto                        it = g_open.find( aKey );

    if( it == g_open.end() )
        return;

    it->second.closedMs = msNow();
    g_history.push_back( it->second );
    g_open.erase( it );

    if( g_history.size() > HISTORY )
        g_history.pop_front();
}


nlohmann::json KOPENAPI_PROGRESS::Snapshot( size_t aLimit )
{
    std::lock_guard<std::mutex> lock( g_mutex );
    nlohmann::json              open = nlohmann::json::array();
    nlohmann::json              history = nlohmann::json::array();

    for( const auto& [key, e] : g_open )
        open.push_back( toJson( e ) );

    const size_t first = g_history.size() > aLimit ? g_history.size() - aLimit : 0;

    for( size_t i = first; i < g_history.size(); ++i )
        history.push_back( toJson( g_history[i] ) );

    return { { "now_ms", msNow() }, { "open", open }, { "history", history } };
}
