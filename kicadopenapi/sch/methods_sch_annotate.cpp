/// @file methods_sch_annotate.cpp
/// @brief sch_annotate: the schematic editor's annotation layer (kicadopenapi_annotations.h), and the
/// schematic's change notifications forwarded to KOPENAPI_MODEL_EVENTS.
#include <kicadopenapi_annotations.h>
#include <kicadopenapi_model_events.h>
#include <kicadopenapi_registry.h>

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


/// @brief An item of the shown sheet by uuid, a symbol by reference, a pin as REF.NUMBER: its box
std::optional<BOX2I> resolveSchematicItem( SCH_EDIT_FRAME* aFrame, const std::string& aRef )
{
    SCH_SCREEN* screen = aFrame ? aFrame->GetScreen() : nullptr;

    if( !screen )
        return std::nullopt;

    const wxString ref = wxString::FromUTF8( aRef );

    if( KIID::SniffTest( ref ) )
    {
        const KIID id( ref );

        for( SCH_ITEM* item : screen->Items() )
        {
            if( item->m_Uuid == id )
                return item->GetBoundingBox();
        }

        return std::nullopt;
    }

    const wxString designator = ref.BeforeFirst( '.' );
    SCH_SHEET_PATH& sheet = aFrame->GetCurrentSheet();

    for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

        if( symbol->GetRef( &sheet ) != designator )
            continue;

        if( !ref.Contains( wxS( "." ) ) )
            return symbol->GetBoundingBox();

        if( SCH_PIN* pin = symbol->GetPin( ref.AfterFirst( '.' ) ) )
            return pin->GetBoundingBox();

        return std::nullopt;
    }

    return std::nullopt;
}


KOPENAPI_RESULT h_sch_annotate( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
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

    return KopenapiAnnotate( host, aArgs );
}
} // namespace


KOPENAPI_REGISTER( "sch_annotate",
                   "Annotation layer over the schematic editor's canvas (the shown sheet), for teaching and "
                   "demos: highlight, arrow, label, region, spotlight, numbered step, key hint. Anchors are "
                   "references (R5), pins (U1.3), uuids or [x, y] mm; anchored annotations follow their items, "
                   "a deleted item orphans them. Not part of the schematic: tools, ERC, plots and files never "
                   "see it; captures and recordings do. add / update / remove / clear; GUI only",
                   KopenapiAnnotateSchema(), true, h_sch_annotate );
