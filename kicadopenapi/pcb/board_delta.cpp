/// @file board_delta.cpp
/// @brief A board's state as item s-expressions by KIID against a base, and the board made a state's
/// again in one commit (variant-tree nodes, docs/inprogress/2026-10-19-route-replay/design-tree.md §3)
#include "board_delta.h"

#include <board.h>
#include <board_commit.h>
#include <build_version.h>
#include <component_classes/component_class.h>
#include <eda_group.h>
#include <footprint.h>
#include <locale_io.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr_parser.h>
#include <pcb_point.h>
#include <pcb_track.h>
#include <richio.h>
#include <zone.h>


namespace
{

class DELTA_IO : public PCB_IO_KICAD_SEXPR
{
public:
    explicit DELTA_IO( const BOARD* aBoard ) : PCB_IO_KICAD_SEXPR( CTL_FOR_BOARD )
    {
        m_board = const_cast<BOARD*>( aBoard );
    }

    std::string Text( const BOARD_ITEM* aItem )
    {
        Format( aItem );
        return GetStringOutput( true );
    }

    /// @brief A board file's start with the board's layers: the items parse as a board
    std::string Header()
    {
        m_out->Print( "(kicad_pcb (version %d) (generator \"pcbnew\") (generator_version %s)", SEXPR_BOARD_FILE_VERSION,
                      m_out->Quotew( GetMajorMinorVersion() ).c_str() );
        formatBoardLayers( m_board );
        return GetStringOutput( true );
    }
};


template <class F>
void forEachItem( const BOARD* aBoard, F&& aFn )
{
    for( FOOTPRINT* fp : aBoard->Footprints() )
        aFn( fp );

    for( PCB_TRACK* t : aBoard->Tracks() )
        aFn( t );

    for( ZONE* z : aBoard->Zones() )
        aFn( z );

    for( BOARD_ITEM* d : aBoard->Drawings() )
        aFn( d );

    for( PCB_POINT* p : aBoard->Points() )
        aFn( p );
}


const std::string* targetOf( const BOARD_STATE& aState, const KIID& aId )
{
    if( auto it = aState.items.find( aId ); it != aState.items.end() )
        return it->second.get();

    if( aState.removed.count( aId ) )
        return nullptr;

    auto it = aState.base->items.find( aId );
    return it != aState.base->items.end() ? it->second.get() : nullptr;
}


/// @brief The board's items against a state: those to take out, those whose text differs, the
/// state's items not on the board
struct PLAN
{
    std::vector<BOARD_ITEM*>                               remove;
    std::vector<std::pair<BOARD_ITEM*, const std::string*>> replace;
    std::vector<std::pair<KIID, const std::string*>>       add;
};


PLAN plan( const BOARD* aBoard, const BOARD_STATE& aState )
{
    PLAN           p;
    DELTA_IO       io( aBoard );
    std::set<KIID> seen;

    forEachItem( aBoard,
                 [&]( BOARD_ITEM* aItem )
                 {
                     seen.insert( aItem->m_Uuid );
                     const std::string* t = targetOf( aState, aItem->m_Uuid );

                     if( !t )
                         p.remove.push_back( aItem );
                     else if( io.Text( aItem ) != *t )
                         p.replace.push_back( { aItem, t } );
                 } );

    for( const auto& [id, text] : aState.items )
    {
        if( !seen.count( id ) )
            p.add.push_back( { id, text.get() } );
    }

    for( const auto& [id, text] : aState.base->items )
    {
        if( !seen.count( id ) && !aState.removed.count( id ) && !aState.items.count( id ) )
            p.add.push_back( { id, text.get() } );
    }

    return p;
}

} // namespace


size_t BOARD_STATE::Bytes() const
{
    size_t b = removed.size() * sizeof( KIID );

    for( const auto& [id, text] : items )
        b += text->size() + sizeof( KIID ) + 48;

    return b;
}


std::shared_ptr<const BOARD_STATE> BoardStateTake( const BOARD* aBoard, std::shared_ptr<const BOARD_BASE>& aBase )
{
    LOCALE_IO toggle;
    DELTA_IO  io( aBoard );

    if( !aBase )
    {
        auto base = std::make_shared<BOARD_BASE>();
        base->file = aBoard->GetFileName().ToStdString();
        forEachItem( aBoard, [&]( BOARD_ITEM* aItem ) { base->items[aItem->m_Uuid] = std::make_shared<const std::string>( io.Text( aItem ) ); } );
        aBase = base;
        auto state = std::make_shared<BOARD_STATE>();
        state->base = aBase;
        return state;
    }

    auto           state = std::make_shared<BOARD_STATE>();
    std::set<KIID> seen;
    state->base = aBase;

    forEachItem( aBoard,
                 [&]( BOARD_ITEM* aItem )
                 {
                     seen.insert( aItem->m_Uuid );
                     std::string text = io.Text( aItem );
                     auto        it = aBase->items.find( aItem->m_Uuid );

                     if( it == aBase->items.end() || *it->second != text )
                         state->items[aItem->m_Uuid] = std::make_shared<const std::string>( std::move( text ) );
                 } );

    for( const auto& [id, text] : aBase->items )
    {
        if( !seen.count( id ) )
            state->removed.insert( id );
    }

    return state;
}


BOARD_STATE_APPLIED BoardStateApply( BOARD* aBoard, BOARD_COMMIT& aCommit, const BOARD_STATE& aState )
{
    LOCALE_IO           toggle;
    BOARD_STATE_APPLIED out;
    const PLAN          p = plan( aBoard, aState );

    std::map<KIID, BOARD_ITEM*> parsed;
    std::unique_ptr<BOARD>      temp;

    if( !p.replace.empty() || !p.add.empty() )
    {
        DELTA_IO    io( aBoard );
        std::string text = io.Header();

        for( const auto& [item, t] : p.replace )
            text += *t;

        for( const auto& [id, t] : p.add )
            text += *t;

        text += ")\n";

        try
        {
            STRING_LINE_READER        reader( text, wxT( "variant" ) );
            PCB_IO_KICAD_SEXPR_PARSER parser( &reader, nullptr, nullptr );
            temp.reset( dynamic_cast<BOARD*>( parser.Parse() ) );
        }
        catch( const IO_ERROR& e )
        {
            out.error = "the node's items do not parse: " + e.What().ToStdString();
            return out;
        }

        if( !temp )
        {
            out.error = "the node's items do not parse as a board";
            return out;
        }

        std::vector<BOARD_ITEM*> items;
        forEachItem( temp.get(), [&]( BOARD_ITEM* aItem ) { items.push_back( aItem ); } );

        auto remap = [&]( BOARD_CONNECTED_ITEM* aItem )
        {
            const wxString name = aItem->GetNetname();
            NETINFO_ITEM*  net = name.IsEmpty() ? NETINFO_LIST::OrphanedItem() : aBoard->FindNet( name );

            if( !net )
            {
                out.netsMissing++;
                net = NETINFO_LIST::OrphanedItem();
            }

            aItem->SetNet( net );
        };

        for( BOARD_ITEM* item : items )
        {
            temp->Remove( item, REMOVE_MODE::BULK );
            parsed[item->m_Uuid] = item;

            if( FOOTPRINT* fp = dynamic_cast<FOOTPRINT*>( item ) )
            {
                for( PAD* pad : fp->Pads() )
                    remap( pad );

                // the parsed class belongs to the temp board's manager
                std::unordered_set<wxString> classes;

                if( const COMPONENT_CLASS* cc = fp->GetStaticComponentClass() )
                {
                    for( const COMPONENT_CLASS* c : cc->GetConstituentClasses() )
                        classes.insert( c->GetName() );
                }

                fp->ResolveComponentClassNames( aBoard, classes );
                fp->InvalidateComponentClassCache();
            }
            else if( BOARD_CONNECTED_ITEM* c = dynamic_cast<BOARD_CONNECTED_ITEM*>( item ) )
            {
                remap( c );
            }
        }
    }

    auto take = [&]( const KIID& aId ) -> BOARD_ITEM*
    {
        auto it = parsed.find( aId );

        if( it == parsed.end() )
            return nullptr;

        BOARD_ITEM* item = it->second;
        parsed.erase( it );
        return item;
    };

    for( BOARD_ITEM* item : p.remove )
    {
        aCommit.Remove( item );
        out.removed++;
    }

    for( const auto& [old, t] : p.replace )
    {
        BOARD_ITEM* item = take( old->m_Uuid );

        if( !item )
            continue;

        if( EDA_GROUP* group = old->GetParentGroup() )
        {
            aCommit.Modify( group->AsEdaItem() );
            group->AddItem( item );
        }

        aCommit.Remove( old );
        aCommit.Add( item );
        out.replaced++;
    }

    for( const auto& [id, t] : p.add )
    {
        if( BOARD_ITEM* item = take( id ) )
        {
            aCommit.Add( item );
            out.added++;
        }
    }

    // parsed but not asked for (an item's text with another KIID): not put on the board
    for( auto& [id, item] : parsed )
        delete item;

    return out;
}


nlohmann::json BoardStateDiff( const BOARD* aBoard, const BOARD_STATE& aState )
{
    LOCALE_IO      toggle;
    const PLAN     p = plan( aBoard, aState );
    nlohmann::json add = nlohmann::json::array(), remove = nlohmann::json::array(), changed = nlohmann::json::array();

    for( size_t k = 0; k < p.add.size() && k < 5; ++k )
        add.push_back( p.add[k].first.AsStdString() );

    for( size_t k = 0; k < p.remove.size() && k < 5; ++k )
        remove.push_back( p.remove[k]->m_Uuid.AsStdString() );

    for( size_t k = 0; k < p.replace.size() && k < 5; ++k )
        changed.push_back( p.replace[k].first->m_Uuid.AsStdString() );

    return { { "same", p.add.empty() && p.remove.empty() && p.replace.empty() },
             { "to_add", p.add.size() }, { "to_remove", p.remove.size() }, { "changed", p.replace.size() },
             { "sample", { { "to_add", add }, { "to_remove", remove }, { "changed", changed } } } };
}
