/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef KICAD_SLOT_POOL_H
#define KICAD_SLOT_POOL_H

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <new>
#include <thread>

/**
 * Fixed-size slots for one hot object type.
 *
 * Slots are carved from 64 KB chunks aligned to their size, so the chunk header (type tag, live
 * count, counters, free list) is found from any slot address by masking. A chunk that becomes
 * empty is released while the pool keeps another one with free slots. Thread safe (spin lock):
 * objects may die on another thread than the one that made them.
 */
class KISLOT_POOL
{
public:
    static constexpr size_t CHUNK_SIZE = 64 * 1024;

    struct STATS
    {
        size_t   live = 0;
        size_t   chunks = 0;
        size_t   peakChunks = 0;
        uint64_t allocs = 0;
        uint64_t frees = 0;
    };

    KISLOT_POOL( size_t aSlotSize, uint16_t aTypeTag ) :
            m_slotSize( roundSlot( aSlotSize ) ),
            m_typeTag( aTypeTag )
    {
    }

    KISLOT_POOL( const KISLOT_POOL& ) = delete;
    KISLOT_POOL& operator=( const KISLOT_POOL& ) = delete;

    /// Releases every chunk; the objects in them must be destroyed already.
    ~KISLOT_POOL()
    {
        CHUNK* c = m_all;

        while( c )
        {
            CHUNK* next = c->nextAll;
            ::operator delete( c, std::align_val_t( CHUNK_SIZE ) );
            c = next;
        }
    }

    size_t SlotSize() const { return m_slotSize; }

    void* Alloc()
    {
        LOCK  guard( m_lock );
        CHUNK* c = m_partial;

        if( !c )
            c = newChunk();

        void* slot;

        if( c->freeList )
        {
            slot = c->freeList;
            c->freeList = *static_cast<void**>( slot );
        }
        else
        {
            slot = c->slots() + size_t( c->bump++ ) * m_slotSize;
        }

        if( ++c->live == c->slotCount )
            unlinkPartial( c );

        m_live++;
        m_allocs++;
        return slot;
    }

    static void Free( void* aSlot )
    {
        if( !aSlot )
            return;

        CHUNK*       c = CHUNK::of( aSlot );
        KISLOT_POOL* pool = c->pool;
        assert( c->magic == CHUNK_MAGIC );

        LOCK guard( pool->m_lock );
        *static_cast<void**>( aSlot ) = c->freeList;
        c->freeList = aSlot;
        pool->m_live--;
        pool->m_frees++;

        if( !c->inPartial )
            pool->linkPartial( c );

        if( --c->live == 0 && ( c->prevPartial || c->nextPartial ) )
            pool->releaseChunk( c );
    }

    STATS GetStats() const
    {
        LOCK  guard( m_lock );
        STATS s;
        s.live = m_live;
        s.chunks = m_chunks;
        s.peakChunks = m_peakChunks;
        s.allocs = m_allocs;
        s.frees = m_frees;
        return s;
    }

private:
    static constexpr uint32_t CHUNK_MAGIC = 0x4B534C54; // "KSLT"

    struct alignas( 16 ) CHUNK
    {
        uint32_t     magic;
        uint16_t     typeTag;
        uint16_t     slotSize;
        uint32_t     slotCount;
        uint32_t     live;
        uint32_t     bump;
        bool         inPartial;
        void*        freeList;
        CHUNK*       prevPartial;
        CHUNK*       nextPartial;
        CHUNK*       prevAll;
        CHUNK*       nextAll;
        KISLOT_POOL* pool;

        static constexpr size_t headerSize() { return ( sizeof( CHUNK ) + 63 ) & ~size_t( 63 ); }

        char* slots() { return reinterpret_cast<char*>( this ) + headerSize(); }

        static CHUNK* of( const void* aSlot )
        {
            return reinterpret_cast<CHUNK*>( reinterpret_cast<uintptr_t>( aSlot ) & ~uintptr_t( CHUNK_SIZE - 1 ) );
        }
    };

    class SPIN
    {
    public:
        void lock()
        {
            while( m_flag.exchange( true, std::memory_order_acquire ) )
            {
                while( m_flag.load( std::memory_order_relaxed ) )
                    std::this_thread::yield();
            }
        }

        void unlock() { m_flag.store( false, std::memory_order_release ); }

    private:
        std::atomic<bool> m_flag{ false };
    };

    struct LOCK
    {
        explicit LOCK( SPIN& aSpin ) : m_spin( aSpin ) { m_spin.lock(); }
        ~LOCK() { m_spin.unlock(); }
        SPIN& m_spin;
    };

    /// Multiples of 16 bytes keep 16-byte alignment and keep a 32-byte head of every slot inside
    /// one cache line.
    static size_t roundSlot( size_t aSize )
    {
        return aSize < 16 ? 16 : ( aSize + 15 ) & ~size_t( 15 );
    }

    CHUNK* newChunk()
    {
        void*  mem = ::operator new( CHUNK_SIZE, std::align_val_t( CHUNK_SIZE ) );
        CHUNK* c = new( mem ) CHUNK();
        c->magic = CHUNK_MAGIC;
        c->typeTag = m_typeTag;
        c->slotSize = uint16_t( m_slotSize );
        c->slotCount = uint32_t( ( CHUNK_SIZE - CHUNK::headerSize() ) / m_slotSize );
        c->pool = this;
        c->nextAll = m_all;

        if( m_all )
            m_all->prevAll = c;

        m_all = c;

        if( ++m_chunks > m_peakChunks )
            m_peakChunks = m_chunks;

        linkPartial( c );
        return c;
    }

    void releaseChunk( CHUNK* c )
    {
        unlinkPartial( c );

        if( c->prevAll )
            c->prevAll->nextAll = c->nextAll;
        else
            m_all = c->nextAll;

        if( c->nextAll )
            c->nextAll->prevAll = c->prevAll;

        m_chunks--;
        ::operator delete( c, std::align_val_t( CHUNK_SIZE ) );
    }

    void linkPartial( CHUNK* c )
    {
        c->prevPartial = nullptr;
        c->nextPartial = m_partial;

        if( m_partial )
            m_partial->prevPartial = c;

        m_partial = c;
        c->inPartial = true;
    }

    void unlinkPartial( CHUNK* c )
    {
        if( c->prevPartial )
            c->prevPartial->nextPartial = c->nextPartial;
        else
            m_partial = c->nextPartial;

        if( c->nextPartial )
            c->nextPartial->prevPartial = c->prevPartial;

        c->prevPartial = c->nextPartial = nullptr;
        c->inPartial = false;
    }

    const size_t   m_slotSize;
    const uint16_t m_typeTag;
    mutable SPIN   m_lock;
    CHUNK*         m_partial = nullptr;
    CHUNK*         m_all = nullptr;
    size_t         m_chunks = 0;
    size_t         m_peakChunks = 0;
    size_t         m_live = 0;
    uint64_t       m_allocs = 0;
    uint64_t       m_frees = 0;
};

#endif // KICAD_SLOT_POOL_H
