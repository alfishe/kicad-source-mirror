#include <kicadopenapi_change_feed.h>
#include <kicadopenapi_jobs.h>
#include <kicadopenapi_journal.h>
#include "threading/kicadopenapi_change_set.h"

#include <deque>
#include <map>
#include <mutex>


namespace
{
struct ENTRY
{
    KOPENAPI_CHANGE_SET changes;
    std::string         job;
    bool                reset = false;
};


struct DOCUMENT_FEED
{
    uint64_t          version = 0;
    std::deque<ENTRY> entries;
};


std::mutex& feedMutex()
{
    static std::mutex m;
    return m;
}


std::map<std::string, DOCUMENT_FEED>& feeds()
{
    static std::map<std::string, DOCUMENT_FEED> f;
    return f;
}


std::string currentSource()
{
    const std::string operation = KOPENAPI_JOURNAL::Operation();

    if( operation.rfind( "api:", 0 ) == 0 || operation.rfind( "ui:", 0 ) == 0 )
        return operation;

    return "gui";
}


void append( const std::string& aDomain, ENTRY aEntry )
{
    std::lock_guard<std::mutex> lock( feedMutex() );
    DOCUMENT_FEED&              feed = feeds()[aDomain];

    aEntry.changes.SetVersions( feed.version, feed.version + 1 );
    feed.version++;
    feed.entries.push_back( std::move( aEntry ) );

    while( feed.entries.size() > KOPENAPI_CHANGE_FEED::KEEP )
        feed.entries.pop_front();
}


nlohmann::json toJson( const KOPENAPI_CHANGE_SET& aChanges )
{
    return { { "from", aChanges.FromVersion() },
             { "to", aChanges.ToVersion() },
             { "source", aChanges.Source() },
             { "added", aChanges.Ids( KOPENAPI_CHANGE::ADDED ) },
             { "changed", aChanges.Ids( KOPENAPI_CHANGE::CHANGED ) },
             { "removed", aChanges.Ids( KOPENAPI_CHANGE::REMOVED ) } };
}
} // namespace


void KOPENAPI_CHANGE_FEED::Record( const std::string& aDomain, const std::vector<std::string>& aAdded,
                                   const std::vector<std::string>& aChanged,
                                   const std::vector<std::string>& aRemoved )
{
    if( aAdded.empty() && aChanged.empty() && aRemoved.empty() )
        return;

    ENTRY entry{ KOPENAPI_CHANGE_SET( currentSource() ), KOPENAPI_JOBS::Get().Busy(), false };

    for( const std::string& id : aAdded )
        entry.changes.Added( id );

    for( const std::string& id : aChanged )
        entry.changes.Changed( id );

    for( const std::string& id : aRemoved )
        entry.changes.Removed( id );

    append( aDomain, std::move( entry ) );
}


void KOPENAPI_CHANGE_FEED::Reset( const std::string& aDomain )
{
    append( aDomain, ENTRY{ KOPENAPI_CHANGE_SET( currentSource() ), KOPENAPI_JOBS::Get().Busy(), true } );
}


uint64_t KOPENAPI_CHANGE_FEED::Version( const std::string& aDomain )
{
    std::lock_guard<std::mutex> lock( feedMutex() );
    auto                        it = feeds().find( aDomain );
    return it == feeds().end() ? 0 : it->second.version;
}


nlohmann::json KOPENAPI_CHANGE_FEED::Since( const std::string& aDomain, uint64_t aSince, size_t aLimit, bool aFold )
{
    std::lock_guard<std::mutex> lock( feedMutex() );
    const DOCUMENT_FEED&        feed = feeds()[aDomain];
    const uint64_t              oldest = feed.entries.empty() ? feed.version : feed.entries.front().changes.FromVersion();

    nlohmann::json out = { { "domain", aDomain }, { "version", feed.version }, { "since", aSince } };

    if( aSince > feed.version )
    {
        out["truncated"] = true;   // another process or a restart: the reader starts over
        out["changes"] = nlohmann::json::array();
        return out;
    }

    out["truncated"] = aSince < oldest;

    nlohmann::json      list = nlohmann::json::array();
    KOPENAPI_CHANGE_SET folded;
    bool                reset = false;
    uint64_t            to = aSince;
    size_t              taken = 0;

    for( const ENTRY& entry : feed.entries )
    {
        if( entry.changes.ToVersion() <= aSince )
            continue;

        if( taken == aLimit )
            break;

        taken++;
        to = entry.changes.ToVersion();

        if( aFold )
        {
            reset |= entry.reset;
            folded.Merge( entry.changes );
            continue;
        }

        nlohmann::json item = toJson( entry.changes );

        if( !entry.job.empty() )
            item["job"] = entry.job;

        if( entry.reset )
            item["reset"] = true;

        list.push_back( std::move( item ) );
    }

    if( aFold && taken )
    {
        nlohmann::json item = toJson( folded );
        item.erase( "source" );
        item["from"] = aSince;
        item["to"] = to;

        if( reset )
            item["reset"] = true;

        list.push_back( std::move( item ) );
    }

    out["changes"] = std::move( list );
    out["more"] = to < feed.version;
    out["next_since"] = to;
    return out;
}


nlohmann::json KOPENAPI_CHANGE_FEED::Status()
{
    std::lock_guard<std::mutex> lock( feedMutex() );
    nlohmann::json              out = nlohmann::json::object();

    for( const auto& [domain, feed] : feeds() )
        out[domain] = { { "version", feed.version }, { "kept", feed.entries.size() } };

    return out;
}
