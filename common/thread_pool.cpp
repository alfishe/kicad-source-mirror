/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see CHANGELOG.TXT for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <atomic>
#include <mutex>

#include <advanced_config.h>
#include <pgm_base.h>
#include <thread_pool.h>

static std::atomic<thread_pool*> tp{ nullptr };
static bool                      tp_owned = false;  // True if we created the thread pool ourselves
static std::mutex                tp_mutex;

thread_pool& GetKiCadThreadPool()
{
    if( thread_pool* pool = tp.load( std::memory_order_acquire ) )
        return *pool;

    std::lock_guard lock( tp_mutex );

    if( thread_pool* pool = tp.load( std::memory_order_relaxed ) )
        return *pool;

    thread_pool* pool = nullptr;

    // If we have a PGM_BASE, use its thread pool
    if( PGM_BASE* pgm = PgmOrNull() )
    {
        pool = &pgm->GetThreadPool();
        tp_owned = false;
    }
    else
    {
        // Otherwise, we are running in scripting or some other context where we don't have a
        // PGM_BASE so we need to create our own thread pool
        int num_threads = std::max( 0, ADVANCED_CFG::GetCfg().m_MaximumThreads );
        pool = new thread_pool( num_threads );
        tp_owned = true;
    }

    tp.store( pool, std::memory_order_release );
    return *pool;
}


void InvalidateKiCadThreadPool()
{
    std::lock_guard lock( tp_mutex );
    thread_pool*    pool = tp.exchange( nullptr );

    // If we own the thread pool (scripting context), delete it
    if( pool && tp_owned )
        delete pool;

    tp_owned = false;
}
