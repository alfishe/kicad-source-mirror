/// @file methods_sch_overlay.cpp
/// @brief sch_overlay: the schematic editor's annotation layer (kicadopenapi_annotations.h), and the
/// schematic's change notifications forwarded to KOPENAPI_MODEL_EVENTS.
#include "kopenapi_sch.h"

#include <kicadopenapi_annotations.h>
#include <kicadopenapi_model_events.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>

#include <base_units.h>
#include <frame_type.h>
#include <kiway.h>
#include <sch_edit_frame.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <schematic.h>


namespace
{
/// Schematic changes -> model events (domain "sch")
class SCHEMATIC_EVENTS : public SCHEMATIC_LISTENER
{
public:
    void OnSchItemsAdded( SCHEMATIC&, std::vector<SCH_ITEM*>& aItems ) override
    {
        post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::ADDED, aItems );
    }

    void OnSchItemsRemoved( SCHEMATIC&, std::vector<SCH_ITEM*>& aItems ) override
    {
        post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::REMOVED, aItems );
    }

    void OnSchItemsChanged( SCHEMATIC&, std::vector<SCH_ITEM*>& aItems ) override
    {
        post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::CHANGED, aItems );
    }

    void OnSchSheetChanged( SCHEMATIC& ) override
    {
        KOPENAPI_MODEL_EVENTS::Post( { "sch", KOPENAPI_MODEL_EVENTS::EVENT::KIND::RESET, {} } );
    }

    /// @brief Listens to the editor's schematic (again after another one was loaded)
    void Follow( SCHEMATIC* aSchematic )
    {
        if( !aSchematic || aSchematic == m_schematic )
            return;

        aSchematic->AddListener( this );
        m_schematic = aSchematic;
    }

private:
    static void post( KOPENAPI_MODEL_EVENTS::EVENT::KIND aKind, const std::vector<SCH_ITEM*>& aItems )
    {
        KOPENAPI_MODEL_EVENTS::EVENT event{ "sch", aKind, {} };

        for( const SCH_ITEM* item : aItems )
            event.items.push_back( item->m_Uuid );

        KOPENAPI_MODEL_EVENTS::Post( event );
    }

    SCHEMATIC* m_schematic = nullptr;
};


SCHEMATIC_EVENTS& schematicEvents()
{
    static SCHEMATIC_EVENTS* s_events = new SCHEMATIC_EVENTS();   // schematics keep a pointer to it
    return *s_events;
}


std::optional<BOX2I> resolveSchematicItem( SCH_EDIT_FRAME* aFrame, const std::string& aRef )
{
    SCH_ITEM* item = KopenapiFindSchematicItem( aFrame, aRef );
    return item ? std::optional<BOX2I>( item->GetBoundingBox() ) : std::nullopt;
}


/// @brief A symbol's orientation and mirroring as a linear map (others have none)
std::optional<std::array<double, 4>> schematicItemLinear( SCH_EDIT_FRAME* aFrame, const std::string& aRef )
{
    auto* symbol = dynamic_cast<SCH_SYMBOL*>( KopenapiFindSchematicItem( aFrame, aRef ) );

    if( !symbol )
        return std::nullopt;

    const TRANSFORM& t = symbol->GetTransform();
    return std::array<double, 4>{ double( t.x1 ), double( t.y1 ), double( t.x2 ), double( t.y2 ) };
}


KOPENAPI_RESULT h_sch_overlay( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, false ) ) : nullptr;

    if( !frame || !frame->GetScreen() )
        return KOPENAPI_RESULT::Error( 409, "no schematic editor window open" );

    schematicEvents().Follow( &frame->Schematic() );

    KOPENAPI_ANNOTATION_HOST host;
    host.domain = "sch";
    host.scale = &schIUScale;
    host.frame = frame;
    host.resolve = [frame]( const std::string& aRef ) { return resolveSchematicItem( frame, aRef ); };
    host.linear = [frame]( const std::string& aRef ) { return schematicItemLinear( frame, aRef ); };

    return KopenapiAnnotate( host, aArgs );
}
} // namespace


SCH_ITEM* KopenapiFindSchematicItem( SCH_EDIT_FRAME* aFrame, const std::string& aRef )
{
    SCH_SCREEN* screen = aFrame ? aFrame->GetScreen() : nullptr;

    if( !screen )
        return nullptr;

    const wxString ref = wxString::FromUTF8( aRef );

    if( KIID::SniffTest( ref ) )
    {
        const KIID id( ref );

        for( SCH_ITEM* item : screen->Items() )
        {
            if( item->m_Uuid == id )
                return item;
        }

        return nullptr;
    }

    const wxString  designator = ref.BeforeFirst( '.' );
    SCH_SHEET_PATH& sheet = aFrame->GetCurrentSheet();

    for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

        if( symbol->GetRef( &sheet ) != designator )
            continue;

        if( ref.Contains( wxS( "." ) ) )
            return symbol->GetPin( ref.AfterFirst( '.' ) );

        return symbol;
    }

    // no such reference: a uuid prefix of an item of the sheet
    if( ref.Contains( wxS( "." ) ) )
        return nullptr;

    std::vector<SCH_ITEM*> items;
    std::string            error;

    for( SCH_ITEM* item : screen->Items() )
        items.push_back( item );

    return KopenapiSchMatchUuid( items, aRef, error );
}


SCH_ITEM* KopenapiSchMatchUuid( const std::vector<SCH_ITEM*>& aScope, const std::string& aId, std::string& aError )
{
    std::vector<std::string> uuids;
    uuids.reserve( aScope.size() );

    for( SCH_ITEM* item : aScope )
        uuids.push_back( item->m_Uuid.AsStdString() );

    const long hit = KopenapiUuidMatch( uuids, aId, aError,
                                        [&]( size_t aIndex )
                                        {
                                            return aScope[aIndex]->GetFriendlyName().ToStdString( wxConvUTF8 );
                                        } );
    return hit < 0 ? nullptr : aScope[hit];
}


bool KopenapiSchExpandUuids( const std::vector<SCH_ITEM*>& aScope, std::set<std::string>& aWanted, std::string& aError )
{
    std::set<std::string> expanded;

    for( const std::string& id : aWanted )
    {
        std::string error;

        if( SCH_ITEM* item = KopenapiSchMatchUuid( aScope, id, error ) )
        {
            expanded.insert( item->m_Uuid.AsStdString() );
        }
        else if( error.find( "ambiguous" ) != std::string::npos )
        {
            aError = error;
            return false;
        }
        else
        {
            expanded.insert( id );
        }
    }

    aWanted = std::move( expanded );
    return true;
}


KOPENAPI_REGISTER( "sch_overlay",
                   "Annotation layer over the schematic editor's canvas (the shown sheet), for teaching and "
                   "demos: highlight, arrow, label, region, spotlight, numbered step, key hint. Anchors are "
                   "references (R5), pins (U1.3), uuids or [x, y] mm; anchored annotations follow their items, "
                   "a deleted item orphans them. Not part of the schematic: tools, ERC, plots and files never "
                   "see it; captures and recordings do. add / update / remove / clear; GUI only",
                   KopenapiAnnotateSchema(), true, h_sch_overlay );
