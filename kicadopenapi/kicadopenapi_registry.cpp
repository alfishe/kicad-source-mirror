#include "kicadopenapi_registry.h"

#include <algorithm>
#include <cctype>
#include <condition_variable>
#include <cstdio>
#include <set>


KOPENAPI_REGISTRY& KOPENAPI_REGISTRY::Get()
{
    static KOPENAPI_REGISTRY instance;
    return instance;
}


bool KOPENAPI_REGISTRY::Add( KOPENAPI_METHOD aMethod )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    std::string name = aMethod.name;

    // two methods under one name: the first stays, the clash is reported (stderr now, the journal
    // when the service starts)
    if( reg.m_methods.count( name ) )
    {
        std::fprintf( stderr, "kicadopenapi: method '%s' registered twice; the first one is kept\n", name.c_str() );
        reg.m_duplicates.push_back( name );
        return false;
    }

    reg.m_methods[name] = std::move( aMethod );
    return true;
}


std::vector<std::string> KOPENAPI_REGISTRY::Duplicates() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_duplicates;
}


bool KOPENAPI_REGISTRY::AddDocumentProvider( const std::string& aDomain, KOPENAPI_DOC_STATUS aStatus,
                                             KOPENAPI_DOC_RELEASE aRelease )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    reg.m_docProviders[aDomain] = std::move( aStatus );
    reg.m_docReleasers[aDomain] = std::move( aRelease );
    return true;
}


bool KOPENAPI_REGISTRY::AddDocumentHistory( const std::string& aDomain, KOPENAPI_DOC_HISTORY aHistory )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    reg.m_docHistories[aDomain] = std::move( aHistory );
    return true;
}


std::map<std::string, KOPENAPI_DOC_HISTORY> KOPENAPI_REGISTRY::DocumentHistories() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_docHistories;
}


bool KOPENAPI_REGISTRY::AddViewBenchmark( const std::string& aDomain, KOPENAPI_HANDLER aHandler )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    reg.m_viewBenchmarks[aDomain] = std::move( aHandler );
    return true;
}


std::map<std::string, KOPENAPI_HANDLER> KOPENAPI_REGISTRY::ViewBenchmarks() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_viewBenchmarks;
}


bool KOPENAPI_REGISTRY::AddEventTopic( KOPENAPI_EVENT_TOPIC aTopic )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    reg.m_eventTopics[aTopic.name] = std::move( aTopic );
    return true;
}


std::map<std::string, KOPENAPI_EVENT_TOPIC> KOPENAPI_REGISTRY::EventTopics() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_eventTopics;
}


namespace
{

struct EVENT_HUB
{
    std::mutex                      mutex;
    std::condition_variable         changed;
    std::map<std::string, uint64_t> seq;
    uint64_t                        wakes = 0;
};

EVENT_HUB& eventHub()
{
    static EVENT_HUB hub;
    return hub;
}

} // namespace


void KopenapiEventNotify( const std::string& aTopic )
{
    EVENT_HUB& hub = eventHub();

    {
        std::lock_guard<std::mutex> lock( hub.mutex );
        ++hub.seq[aTopic];
    }

    hub.changed.notify_all();
}


uint64_t KopenapiEventSeq( const std::string& aTopic )
{
    EVENT_HUB&                  hub = eventHub();
    std::lock_guard<std::mutex> lock( hub.mutex );
    auto                        it = hub.seq.find( aTopic );

    return it == hub.seq.end() ? 0 : it->second;
}


void KopenapiEventWait( const std::string& aTopic, uint64_t aSeen, std::chrono::steady_clock::time_point aUntil )
{
    EVENT_HUB&                   hub = eventHub();
    std::unique_lock<std::mutex> lock( hub.mutex );
    const uint64_t               wakes = hub.wakes;

    hub.changed.wait_until( lock, aUntil,
                            [&]
                            {
                                auto it = hub.seq.find( aTopic );
                                return hub.wakes != wakes || ( it != hub.seq.end() && it->second != aSeen );
                            } );
}


void KopenapiEventWakeAll()
{
    EVENT_HUB& hub = eventHub();

    {
        std::lock_guard<std::mutex> lock( hub.mutex );
        ++hub.wakes;
    }

    hub.changed.notify_all();
}


bool KOPENAPI_REGISTRY::MarkEditing( std::initializer_list<const char*> aNames )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    for( const char* name : aNames )
        reg.m_editing.insert( name );

    return true;
}


bool KOPENAPI_REGISTRY::IsEditing( const std::string& aName ) const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_editing.count( aName ) > 0;
}


void KOPENAPI_REGISTRY::ReleaseDocuments() const
{
    std::vector<KOPENAPI_DOC_RELEASE> releasers;

    {
        std::lock_guard<std::mutex> lock( m_mutex );

        for( const auto& [domain, release] : m_docReleasers )
            releasers.push_back( release );
    }

    for( const KOPENAPI_DOC_RELEASE& release : releasers )
    {
        if( release )
            release();
    }
}


std::vector<std::pair<std::string, KOPENAPI_DOC_STATUS>> KOPENAPI_REGISTRY::DocumentProviders() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return { m_docProviders.begin(), m_docProviders.end() };
}


nlohmann::json KOPENAPI_REGISTRY::Documents( KOPENAPI_CONTEXT& aCtx ) const
{
    nlohmann::json docs = nlohmann::json::array();

    for( const auto& [domain, status] : DocumentProviders() )
    {
        nlohmann::json doc = status( aCtx );

        if( !doc.is_null() )
            docs.push_back( std::move( doc ) );
    }

    return docs;
}


std::vector<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Snapshot() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    std::vector<KOPENAPI_METHOD> out;

    for( const auto& [name, method] : m_methods )
        out.push_back( method );

    return out;
}


std::optional<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Find( const std::string& aName ) const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    auto it = m_methods.find( aName );

    if( it == m_methods.end() )
        return std::nullopt;

    return it->second;
}


static std::vector<std::string> tokenize( const std::string& aText )
{
    std::vector<std::string> tokens;
    std::string              current;

    for( char ch : aText )
    {
        if( std::isalnum( (unsigned char) ch ) )
        {
            current += (char) std::tolower( (unsigned char) ch );
        }
        else if( !current.empty() )
        {
            tokens.push_back( current );
            current.clear();
        }
    }

    if( !current.empty() )
        tokens.push_back( current );

    return tokens;
}


static std::string lower( std::string aText )
{
    for( char& ch : aText )
        ch = (char) std::tolower( (unsigned char) ch );

    return aText;
}


static std::string squeeze( const std::string& aText )
{
    std::string out;

    for( char ch : aText )
    {
        if( std::isalnum( (unsigned char) ch ) )
            out += (char) std::tolower( (unsigned char) ch );
    }

    return out;
}


static int scoreMethod( const KOPENAPI_METHOD& aMethod, const std::vector<std::string>& aTerms, bool aWithProperties )
{
    const std::string name = lower( aMethod.name );
    const std::string summary = lower( aMethod.summary );
    const std::string joined = squeeze( aMethod.summary );
    std::string       properties;

    if( aWithProperties && aMethod.inputSchema.contains( "properties" ) && aMethod.inputSchema["properties"].is_object() )
    {
        for( const auto& [key, value] : aMethod.inputSchema["properties"].items() )
            properties += lower( key ) + " ";
    }

    int score = 0;

    for( const std::string& term : aTerms )
    {
        if( name.find( term ) != std::string::npos )
            score += 3;

        if( summary.find( term ) != std::string::npos || joined.find( term ) != std::string::npos )
            score += 2;

        if( properties.find( term ) != std::string::npos )
            score += 1;
    }

    return score;
}


std::vector<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Search( const std::string& aQuery, size_t aLimit,
                                                        bool aIncludeGuiOnly ) const
{
    const std::vector<std::string> terms = tokenize( aQuery );
    std::vector<std::pair<int, KOPENAPI_METHOD>> scored;

    for( KOPENAPI_METHOD& method : Snapshot() )
    {
        if( method.guiOnly && !aIncludeGuiOnly )
            continue;

        const int score = terms.empty() ? 1 : scoreMethod( method, terms, true );

        if( score > 0 )
            scored.emplace_back( score, std::move( method ) );
    }

    std::stable_sort( scored.begin(), scored.end(),
                      []( const auto& a, const auto& b ) { return a.first > b.first; } );

    std::vector<KOPENAPI_METHOD> out;

    for( auto& [score, method] : scored )
    {
        if( out.size() >= aLimit )
            break;

        out.push_back( std::move( method ) );
    }

    return out;
}


static size_t editDistance( const std::string& a, const std::string& b )
{
    std::vector<size_t> prev( b.size() + 1 ), cur( b.size() + 1 );

    for( size_t j = 0; j <= b.size(); ++j )
        prev[j] = j;

    for( size_t i = 1; i <= a.size(); ++i )
    {
        cur[0] = i;

        for( size_t j = 1; j <= b.size(); ++j )
            cur[j] = std::min( { prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + ( a[i - 1] == b[j - 1] ? 0 : 1 ) } );

        std::swap( prev, cur );
    }

    return prev[b.size()];
}


std::vector<std::string> KOPENAPI_REGISTRY::Similar( const std::string& aName, size_t aLimit, bool aIncludeGuiOnly ) const
{
    static const std::set<std::string> generic = { "list", "get", "set", "add", "new", "remove", "delete", "del",
                                                   "edit", "info", "show", "all", "query", "find", "make", "create" };

    std::vector<KOPENAPI_METHOD> methods;

    for( KOPENAPI_METHOD& method : Snapshot() )
    {
        if( !method.guiOnly || aIncludeGuiOnly )
            methods.push_back( std::move( method ) );
    }

    std::vector<std::string> tokens = tokenize( aName );
    std::string              domain;

    if( !tokens.empty()
        && std::any_of( methods.begin(), methods.end(),
                        [&]( const KOPENAPI_METHOD& m ) { return m.name.rfind( tokens[0] + "_", 0 ) == 0; } ) )
    {
        domain = tokens[0] + "_";
    }

    std::vector<std::string> terms;

    for( size_t i = domain.empty() ? 0 : 1; i < tokens.size(); ++i )
    {
        if( !generic.count( tokens[i] ) && tokens[i].size() >= 3 )
            terms.push_back( tokens[i] );
    }

    const std::string               wanted = lower( aName );
    const size_t                    close = std::max<size_t>( 3, wanted.size() / 3 ) + 1;
    std::vector<std::pair<size_t, std::string>> spelt;
    std::vector<std::pair<int, std::string>>    topical;

    for( const KOPENAPI_METHOD& m : methods )
    {
        if( const size_t d = editDistance( wanted, m.name ); d < close )
            spelt.emplace_back( d, m.name );
        else if( !terms.empty() && ( domain.empty() || m.name.rfind( domain, 0 ) == 0 ) )
        {
            if( const int score = scoreMethod( m, terms, false ); score > 0 )
                topical.emplace_back( score, m.name );
        }
    }

    std::stable_sort( spelt.begin(), spelt.end() );
    std::stable_sort( topical.begin(), topical.end(), []( const auto& a, const auto& b ) { return a.first > b.first; } );

    // a slip of a letter or two, then the topic, then the looser spellings
    std::vector<std::string> out;

    for( const auto& [d, name] : spelt )
    {
        if( d <= 2 )
            out.push_back( name );
    }

    for( const auto& [score, name] : topical )
        out.push_back( name );

    for( const auto& [d, name] : spelt )
    {
        if( d > 2 )
            out.push_back( name );
    }

    if( out.size() > aLimit )
        out.resize( aLimit );

    return out;
}


bool KOPENAPI_REGISTRY::AddCanvasCapture( KOPENAPI_CANVAS_CAPTURE aCapture )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );
    reg.m_canvasCaptures.push_back( std::move( aCapture ) );
    return true;
}


std::vector<KOPENAPI_CANVAS_CAPTURE> KOPENAPI_REGISTRY::CanvasCaptures() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_canvasCaptures;
}


bool KOPENAPI_REGISTRY::AddCanvasRender( KOPENAPI_CANVAS_RENDER aRender )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );
    reg.m_canvasRenders.push_back( std::move( aRender ) );
    return true;
}


std::vector<KOPENAPI_CANVAS_RENDER> KOPENAPI_REGISTRY::CanvasRenders() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_canvasRenders;
}


bool KOPENAPI_REGISTRY::AddCanvasSteady( KOPENAPI_CANVAS_STEADY aSteady )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );
    reg.m_canvasSteadies.push_back( std::move( aSteady ) );
    return true;
}


std::vector<KOPENAPI_CANVAS_STEADY> KOPENAPI_REGISTRY::CanvasSteadies() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    return m_canvasSteadies;
}
