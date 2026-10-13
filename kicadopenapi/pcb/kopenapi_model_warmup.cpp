/// @file kopenapi_model_warmup.cpp
/// @brief Background warm-up of the 3D model cache (see kopenapi_model_warmup.h).
#include "kopenapi_model_warmup.h"

#include <3d_cache/3d_cache.h>
#include <board.h>
#include <filename_resolver.h>
#include <footprint.h>
#include <footprint_library_adapter.h>
#include <libraries/library_manager.h>
#include <pgm_base.h>
#include <project_pcb.h>
#include <kicadopenapi_memory.h>

#include <wx/filename.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>


namespace
{
constexpr auto    DELAY = std::chrono::seconds( 3 );           ///< let the editor settle first
constexpr int     WORKERS = 2;
constexpr int64_t BUDGET_CAP = int64_t( 1 ) << 30;             ///< model files read at most (bytes)
constexpr double  BUDGET_SHARE = 1.0 / 16;                     ///< ... and at most this share of RAM
constexpr double  FOOTPRINT_SHARE = 0.5;                       ///< stop once the process holds this much RAM

struct JOB
{
    wxString file;
    wxString basePath;
    int64_t  bytes = 0;
};

struct STATE
{
    std::mutex                mutex;
    std::vector<std::jthread> threads;
    std::vector<JOB>          jobs;
    std::atomic<size_t>       next{ 0 };
    std::atomic<size_t>       done{ 0 };
    std::atomic<int64_t>      bytes{ 0 };
    std::atomic<int>          alive{ 0 };
    std::string               stopped;   ///< why it ended early ("" while running / complete)
    bool                      running = false;
};


STATE& state()
{
    static STATE s;
    return s;
}


/// @return why loading must stop now, or nullptr
const char* mustStop( int64_t aBudget )
{
    if( KOPENAPI_MEMORY::UnderPressure( FOOTPRINT_SHARE ) )
        return "memory pressure";

    if( state().bytes.load() >= aBudget )
        return "budget spent";

    return nullptr;
}
} // namespace


void KopenapiStopWarmModels()
{
    STATE&                    s = state();
    std::vector<std::jthread> threads;

    {
        std::lock_guard<std::mutex> lock( s.mutex );
        threads = std::move( s.threads );
        s.threads.clear();
    }

    for( std::jthread& t : threads )
        t.request_stop();

    threads.clear();   // joins

    std::lock_guard<std::mutex> lock( s.mutex );
    s.running = false;
}


void KopenapiWarmModels( BOARD* aBoard )
{
    KopenapiStopWarmModels();

    if( !aBoard || !aBoard->GetProject() || !PgmOrNull() || !Pgm().IsGUI() )
        return;

    if( const char* env = std::getenv( "KICAD_3D_WARMUP" ); env && std::strcmp( env, "0" ) == 0 )
        return;

    S3D_CACHE* cache = PROJECT_PCB::Get3DCacheManager( aBoard->GetProject() );

    if( !cache )
        return;

    // the list now, on the main thread: the threads never touch the board
    std::vector<JOB>   jobs;
    std::set<wxString> seen;

    for( const FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( const FP_3DMODEL& m : fp->Models() )
        {
            if( !m.m_Show || m.m_Filename.empty() || m.m_Filename.StartsWith( wxS( "kicad-embed://" ) )
                || !seen.insert( m.m_Filename ).second )
            {
                continue;
            }

            JOB job{ m.m_Filename, wxString(), 0 };

            try
            {
                if( std::optional<LIBRARY_TABLE_ROW*> row =
                            PROJECT_PCB::FootprintLibAdapter( aBoard->GetProject() )->GetRow( fp->GetFPID().GetLibNickname() ) )
                {
                    job.basePath = LIBRARY_MANAGER::GetFullURI( *row, true );
                }
            }
            catch( ... )
            {
            }

            const wxString path = cache->GetResolver()->ResolvePath( job.file, job.basePath, {} );

            if( path.empty() )
                continue;

            job.bytes = (int64_t) wxFileName::GetSize( path ).GetValue();
            jobs.push_back( job );
        }
    }

    if( jobs.empty() )
        return;

    std::sort( jobs.begin(), jobs.end(), []( const JOB& a, const JOB& b ) { return a.bytes > b.bytes; } );

    const int64_t total = KOPENAPI_MEMORY::SystemTotal();
    const int64_t budget = total > 0 ? std::min( BUDGET_CAP, (int64_t) ( (double) total * BUDGET_SHARE ) ) : BUDGET_CAP;

    STATE&                      s = state();
    std::lock_guard<std::mutex> lock( s.mutex );

    s.jobs = std::move( jobs );
    s.next = 0;
    s.done = 0;
    s.bytes = 0;
    s.stopped.clear();
    s.running = true;
    s.alive = WORKERS;

    for( int w = 0; w < WORKERS; ++w )
    {
        s.threads.emplace_back(
                [cache, budget]( std::stop_token aStop )
                {
                    KOPENAPI_MEMORY::LowerThreadPriority();
                    STATE& st = state();

                    struct LAST
                    {
                        ~LAST()
                        {
                            if( --state().alive == 0 )
                            {
                                std::lock_guard<std::mutex> l( state().mutex );
                                state().running = false;
                            }
                        }
                    } last;

                    // the editor settles first (and a quick close stops us before any work)
                    for( auto until = std::chrono::steady_clock::now() + DELAY;
                         std::chrono::steady_clock::now() < until; )
                    {
                        if( aStop.stop_requested() )
                            return;

                        std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
                    }

                    for( size_t i = st.next++; i < st.jobs.size() && !aStop.stop_requested(); i = st.next++ )
                    {
                        if( const char* why = mustStop( budget ) )
                        {
                            std::lock_guard<std::mutex> l( st.mutex );
                            st.stopped = why;
                            break;
                        }

                        cache->GetModel( st.jobs[i].file, st.jobs[i].basePath, {} );
                        st.bytes += st.jobs[i].bytes;
                        st.done++;
                    }
                } );
    }
}


nlohmann::json KopenapiWarmModelsStatus()
{
    STATE&                      s = state();
    std::lock_guard<std::mutex> lock( s.mutex );

    return { { "running", s.running },
             { "models", s.jobs.size() },
             { "done", s.done.load() },
             { "bytes_read", s.bytes.load() },
             { "stopped", s.stopped.empty() ? nlohmann::json() : nlohmann::json( s.stopped ) } };
}
