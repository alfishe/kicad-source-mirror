/// @file kicadopenapi_change_set.h
/// @brief What one owner write changed: item ids by kind and the versions around it.
///
/// Ids only (KIID strings), never item copies: a reader of the change set looks the items up
/// in the live document under a read lease (or on the owner).  Several entries for one id fold
/// into one, so a change set always says what differs between FromVersion and ToVersion.
#ifndef KICADOPENAPI_CHANGE_SET_H
#define KICADOPENAPI_CHANGE_SET_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>


enum class KOPENAPI_CHANGE
{
    ADDED,
    CHANGED,
    REMOVED
};


class KOPENAPI_CHANGE_SET
{
public:
    KOPENAPI_CHANGE_SET() = default;

    explicit KOPENAPI_CHANGE_SET( std::string aSource ) :
            m_source( std::move( aSource ) )
    {
    }

    /// @brief Records a change of aId, folded with what was recorded for it before:
    /// added + changed = added; added + removed = nothing; removed + added = changed;
    /// changed + removed = removed.
    void Add( KOPENAPI_CHANGE aKind, const std::string& aId )
    {
        auto it = m_index.find( aId );

        if( it == m_index.end() )
        {
            m_index.emplace( aId, m_items.size() );
            m_items.emplace_back( aId, aKind );
            return;
        }

        std::optional<KOPENAPI_CHANGE> folded = fold( m_items[it->second].second, aKind );

        if( folded )
        {
            m_items[it->second].second = *folded;
        }
        else
        {
            const size_t gone = it->second;
            m_items.erase( m_items.begin() + static_cast<std::ptrdiff_t>( gone ) );
            m_index.erase( it );

            for( auto& [id, index] : m_index )
            {
                if( index > gone )
                    index--;
            }
        }
    }

    void Added( const std::string& aId ) { Add( KOPENAPI_CHANGE::ADDED, aId ); }
    void Changed( const std::string& aId ) { Add( KOPENAPI_CHANGE::CHANGED, aId ); }
    void Removed( const std::string& aId ) { Add( KOPENAPI_CHANGE::REMOVED, aId ); }

    /// @brief Folds a later change set into this one; the versions span both.
    void Merge( const KOPENAPI_CHANGE_SET& aLater )
    {
        for( const auto& [id, kind] : aLater.m_items )
            Add( kind, id );

        if( !m_hasVersions )
            m_fromVersion = aLater.m_fromVersion;

        m_toVersion = aLater.m_toVersion;
        m_hasVersions = true;
    }

    bool Empty() const { return m_items.empty(); }
    size_t Size() const { return m_items.size(); }

    /// @return entries in first-recorded order
    const std::vector<std::pair<std::string, KOPENAPI_CHANGE>>& Items() const { return m_items; }

    std::vector<std::string> Ids( KOPENAPI_CHANGE aKind ) const
    {
        std::vector<std::string> ids;

        for( const auto& [id, kind] : m_items )
        {
            if( kind == aKind )
                ids.push_back( id );
        }

        return ids;
    }

    const std::string& Source() const { return m_source; }
    void SetSource( std::string aSource ) { m_source = std::move( aSource ); }

    uint64_t FromVersion() const { return m_fromVersion; }
    uint64_t ToVersion() const { return m_toVersion; }

    void SetVersions( uint64_t aFrom, uint64_t aTo )
    {
        m_fromVersion = aFrom;
        m_toVersion = aTo;
        m_hasVersions = true;
    }

private:
    static std::optional<KOPENAPI_CHANGE> fold( KOPENAPI_CHANGE aFirst, KOPENAPI_CHANGE aThen )
    {
        switch( aFirst )
        {
        case KOPENAPI_CHANGE::ADDED:
            if( aThen == KOPENAPI_CHANGE::REMOVED )
                return std::nullopt;

            return KOPENAPI_CHANGE::ADDED;

        case KOPENAPI_CHANGE::CHANGED:
            return aThen == KOPENAPI_CHANGE::REMOVED ? KOPENAPI_CHANGE::REMOVED : KOPENAPI_CHANGE::CHANGED;

        case KOPENAPI_CHANGE::REMOVED:
            if( aThen == KOPENAPI_CHANGE::ADDED )
                return KOPENAPI_CHANGE::CHANGED;

            return KOPENAPI_CHANGE::REMOVED;
        }

        return aThen;
    }

    std::string                                          m_source;
    uint64_t                                             m_fromVersion = 0;
    uint64_t                                             m_toVersion = 0;
    bool                                                 m_hasVersions = false;
    std::vector<std::pair<std::string, KOPENAPI_CHANGE>> m_items;
    std::unordered_map<std::string, size_t>              m_index;
};

#endif
