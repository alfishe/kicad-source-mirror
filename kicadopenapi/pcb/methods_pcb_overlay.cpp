/// @file methods_pcb_overlay.cpp
/// @brief pcb_overlay: the board editor's annotation layer (kicadopenapi_annotations.h), and the
/// board's change notifications forwarded to KOPENAPI_MODEL_EVENTS.
#include <kicadopenapi_annotations.h>
#include <kicadopenapi_model_events.h>
#include <kicadopenapi_registry.h>

#include <base_units.h>
#include <board.h>
#include <footprint.h>
#include <frame_type.h>
#include <kiway.h>
#include <pad.h>
#include <pcb_edit_frame.h>


namespace
{
/// Board changes -> model events (domain "pcb"); one listener for the board shown in the editor
class BOARD_EVENTS : public BOARD_LISTENER
{
public:
    void OnBoardItemsAdded( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override
    {
        post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::ADDED, aItems );
    }

    void OnBoardItemsRemoved( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override
    {
        post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::REMOVED, aItems );
    }

    void OnBoardItemsChanged( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override
    {
        post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::CHANGED, aItems );
    }

    /// Commits report here (added, removed and changed together)
    void OnBoardCompositeUpdate( BOARD&, std::vector<BOARD_ITEM*>& aAdded, std::vector<BOARD_ITEM*>& aRemoved,
                                 std::vector<BOARD_ITEM*>& aChanged ) override
    {
        if( !aAdded.empty() )
            post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::ADDED, aAdded );

        if( !aRemoved.empty() )
            post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::REMOVED, aRemoved );

        if( !aChanged.empty() )
            post( KOPENAPI_MODEL_EVENTS::EVENT::KIND::CHANGED, aChanged );
    }

    void OnBoardItemAdded( BOARD&, BOARD_ITEM* aItem ) override { postOne( KOPENAPI_MODEL_EVENTS::EVENT::KIND::ADDED, aItem ); }
    void OnBoardItemRemoved( BOARD&, BOARD_ITEM* aItem ) override { postOne( KOPENAPI_MODEL_EVENTS::EVENT::KIND::REMOVED, aItem ); }
    void OnBoardItemChanged( BOARD&, BOARD_ITEM* aItem ) override { postOne( KOPENAPI_MODEL_EVENTS::EVENT::KIND::CHANGED, aItem ); }

    /// @brief Listens to the editor's board (again after a board was loaded)
    void Follow( BOARD* aBoard )
    {
        if( !aBoard || aBoard == m_board )
            return;

        // the previous board, if it still exists, keeps a listener that only posts harmless redraws
        aBoard->AddListener( this );
        m_board = aBoard;
        KOPENAPI_MODEL_EVENTS::Post( { "pcb", KOPENAPI_MODEL_EVENTS::EVENT::KIND::RESET, {} } );
    }

private:
    static void post( KOPENAPI_MODEL_EVENTS::EVENT::KIND aKind, const std::vector<BOARD_ITEM*>& aItems )
    {
        KOPENAPI_MODEL_EVENTS::EVENT event{ "pcb", aKind, {} };

        for( const BOARD_ITEM* item : aItems )
            event.items.push_back( item->m_Uuid );

        KOPENAPI_MODEL_EVENTS::Post( event );
    }

    static void postOne( KOPENAPI_MODEL_EVENTS::EVENT::KIND aKind, const BOARD_ITEM* aItem )
    {
        KOPENAPI_MODEL_EVENTS::Post( { "pcb", aKind, { aItem->m_Uuid } } );
    }

    BOARD* m_board = nullptr;
};


BOARD_EVENTS& boardEvents()
{
    static BOARD_EVENTS* s_events = new BOARD_EVENTS();   // boards keep a pointer to it: never freed
    return *s_events;
}


/// @brief A board item by uuid, a footprint by reference, a pad as REF.NUMBER: its bounding box
std::optional<BOX2I> resolveBoardItem( PCB_EDIT_FRAME* aFrame, const std::string& aRef )
{
    BOARD* board = aFrame ? aFrame->GetBoard() : nullptr;

    if( !board )
        return std::nullopt;

    const wxString ref = wxString::FromUTF8( aRef );

    if( KIID::SniffTest( ref ) )
    {
        BOARD_ITEM* item = board->ResolveItem( KIID( ref ), true );

        if( !item || item == DELETED_BOARD_ITEM::GetInstance() )
            return std::nullopt;

        if( item->Type() == PCB_FOOTPRINT_T )
            return static_cast<FOOTPRINT*>( item )->GetBoundingBox( false );

        return item->GetBoundingBox();
    }

    const wxString designator = ref.BeforeFirst( '.' );

    if( FOOTPRINT* footprint = board->FindFootprintByReference( designator ) )
    {
        if( ref.Contains( wxS( "." ) ) )
        {
            if( PAD* pad = footprint->FindPadByNumber( ref.AfterFirst( '.' ) ) )
                return pad->GetBoundingBox();

            return std::nullopt;
        }

        return footprint->GetBoundingBox( false );
    }

    return std::nullopt;
}


KOPENAPI_RESULT h_pcb_overlay( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;

    if( !frame || !frame->GetBoard() )
        return KOPENAPI_RESULT::Error( 409, "no board editor window open" );

    boardEvents().Follow( frame->GetBoard() );

    KOPENAPI_ANNOTATION_HOST host;
    host.domain = "pcb";
    host.scale = &pcbIUScale;
    host.frame = frame;
    host.resolve = [frame]( const std::string& aRef ) { return resolveBoardItem( frame, aRef ); };

    return KopenapiAnnotate( host, aArgs );
}
} // namespace


KOPENAPI_REGISTER( "pcb_overlay",
                   "Annotation layer over the board editor's canvas, for teaching and demos: highlight, arrow, "
                   "label, region, spotlight (dims the rest), numbered step, key hint. Anchors are references "
                   "(U1), pads (U1.3), uuids or [x, y] mm; anchored annotations follow their items (moves, "
                   "drags, edits), a deleted item orphans them (on_orphan keep / hide / remove). Not part of "
                   "the board: tools, selection, DRC, plots and files never see it; captures and recordings "
                   "do. add / update / remove / clear (group or *); answers every annotation with its "
                   "resolved box; GUI only",
                   KopenapiAnnotateSchema(), true, h_pcb_overlay );
