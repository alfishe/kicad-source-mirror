/// @file methods_pcb_global_edit.cpp
/// @brief kicadopenapi pcb_text_graphics_edit (Edit Text and Graphics) and pcb_graphics_cleanup
/// (Cleanup Graphics), through the editor's own code (GlobalEditTextAndGraphics, GRAPHICS_CLEANER).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <cleanup_item.h>
#include <eda_text.h>
#include <embedded_files.h>
#include <font/font.h>
#include <font/kicad_font_name.h>
#include <footprint.h>
#include <global_edit_text_and_graphics.h>
#include <graphics_cleaner.h>
#include <kicadopenapi_util.h>
#include <tool/tool_manager.h>

#include <map>
#include <optional>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


std::optional<PCB_LAYER_ID> namedLayer( BOARD* aBoard, const nlohmann::json& aName )
{
    if( !aName.is_string() )
        return std::nullopt;

    const int id = aBoard->GetLayerID( wxString::FromUTF8( aName.get<std::string>() ) );

    if( id < 0 || !aBoard->IsLayerEnabled( (PCB_LAYER_ID) id ) )
        return std::nullopt;

    return (PCB_LAYER_ID) id;
}


nlohmann::json describe( BOARD_ITEM* aItem )
{
    nlohmann::json row = { { "uuid", str( aItem->m_Uuid.AsString() ) },
                           { "kind", str( aItem->GetClass() ) },
                           { "layer", str( aItem->GetBoard()->GetLayerName( aItem->GetLayer() ) ) } };

    if( FOOTPRINT* fp = aItem->GetParentFootprint() )
        row["footprint"] = str( fp->GetReference() );

    if( EDA_TEXT* text = dynamic_cast<EDA_TEXT*>( aItem ) )
        row["text"] = str( text->GetText() );

    return row;
}

} // namespace


static KOPENAPI_RESULT h_pcb_text_graphics_edit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                                board = context->GetBoard();
    GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS opt;

    static const std::map<std::string, bool GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::*> scopes = {
        { "references", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::references },
        { "values", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::values },
        { "other_fields", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::otherFootprintFields },
        { "footprint_texts", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::footprintTexts },
        { "footprint_graphics", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::footprintGraphics },
        { "footprint_dimensions", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::footprintDimensions },
        { "board_texts", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::boardTexts },
        { "board_graphics", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::boardGraphics },
        { "board_dimensions", &GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS::boardDimensions }
    };

    const nlohmann::json items = aArgs.value( "items", nlohmann::json::array() );

    if( !items.is_array() || items.empty() )
        return KOPENAPI_RESULT::Error( 400, "items: the kinds to edit (references, values, other_fields, "
                                            "footprint_texts, footprint_graphics, footprint_dimensions, "
                                            "board_texts, board_graphics, board_dimensions)" );

    for( const nlohmann::json& kind : items )
    {
        auto it = kind.is_string() ? scopes.find( kind.get<std::string>() ) : scopes.end();

        if( it == scopes.end() )
            return KOPENAPI_RESULT::Error( 400, "items: unknown kind " + kind.dump() );

        opt.*( it->second ) = true;
    }

    opt.selectedOnly = aArgs.value( "selected_only", false );

    if( aArgs.contains( "layer_filter" ) )
    {
        opt.layerFilter = namedLayer( board, aArgs["layer_filter"] );

        if( !opt.layerFilter )
            return KOPENAPI_RESULT::Error( 400, "layer_filter: an enabled board layer" );
    }

    opt.referenceFilter = wxString::FromUTF8( aArgs.value( "reference_filter", std::string() ) );
    opt.footprintFilter = wxString::FromUTF8( aArgs.value( "footprint_filter", std::string() ) );

    static const char* const specified[] = { "layer", "line_width_mm", "text_width_mm", "text_height_mm",
                                             "text_thickness_mm", "auto_thickness", "bold", "italic",
                                             "keep_upright", "visible", "center_on_footprint", "font" };
    bool anySpecified = false;

    for( const char* key : specified )
        anySpecified |= aArgs.contains( key );

    opt.setToLayerDefaults = aArgs.value( "layer_defaults", false );

    if( opt.setToLayerDefaults == anySpecified )
        return KOPENAPI_RESULT::Error( 400, "either layer_defaults: true (Board Setup's text and graphics "
                                            "defaults per layer class) or the values to set" );

    if( aArgs.contains( "layer" ) )
    {
        opt.layer = namedLayer( board, aArgs["layer"] );

        if( !opt.layer )
            return KOPENAPI_RESULT::Error( 400, "layer: an enabled board layer" );
    }

    const int minText = pcbIUScale.mmToIU( TEXT_MIN_SIZE_MM );
    const int maxText = pcbIUScale.mmToIU( TEXT_MAX_SIZE_MM );
    std::string bad;

    auto size = [&]( const char* aKey, std::optional<int>& aOut, int aMin, int aMax )
    {
        if( !aArgs.contains( aKey ) )
            return;

        if( !aArgs[aKey].is_number() )
        {
            bad = aKey;
            return;
        }

        const int iu = pcbIUScale.mmToIU( aArgs[aKey].get<double>() );

        if( iu < aMin || iu > aMax )
            bad = aKey;
        else
            aOut = iu;
    };

    size( "line_width_mm", opt.lineWidth, 0, maxText );
    size( "text_width_mm", opt.textWidth, minText, maxText );
    size( "text_height_mm", opt.textHeight, minText, maxText );
    size( "text_thickness_mm", opt.textThickness, 0, maxText );

    if( !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad + ": out of range (text sizes " + std::to_string( TEXT_MIN_SIZE_MM )
                                                + " .. " + std::to_string( TEXT_MAX_SIZE_MM ) + " mm)" );

    opt.autoTextThickness = aArgs.value( "auto_thickness", false );

    auto flag = [&]( const char* aKey, std::optional<bool>& aOut )
    {
        if( aArgs.contains( aKey ) && aArgs[aKey].is_boolean() )
            aOut = aArgs[aKey].get<bool>();
    };

    flag( "bold", opt.bold );
    flag( "italic", opt.italic );
    flag( "keep_upright", opt.keepUpright );
    flag( "visible", opt.visible );
    opt.centerOnFootprint = aArgs.value( "center_on_footprint", false );

    if( aArgs.contains( "font" ) )
    {
        if( !aArgs["font"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "font: a font name (\"KiCad Font\" for the stroke font)" );

        const wxString name = wxString::FromUTF8( aArgs["font"].get<std::string>() );
        EMBEDDED_FILES* embedded = board->GetEmbeddedFiles();

        opt.font = [name, embedded]( bool aBold, bool aItalic )
        {
            if( name == KICAD_FONT_NAME )
                return KIFONT::FONT::GetFont( KICAD_FONT_NAME, aBold, aItalic );

            return KIFONT::FONT::GetFont( name, aBold, aItalic, embedded ? embedded->GetFontFiles() : nullptr );
        };
    }

    BOARD_COMMIT             commit( context->GetToolManager() );
    std::vector<BOARD_ITEM*> changed = GlobalEditTextAndGraphics( *board, commit, board->GetDesignSettings(), opt,
                                                                  true );

    KOPENAPI_LIST_CAP              cap( aArgs, 50 );
    std::map<std::string, int>     byKind;
    std::vector<nlohmann::json>    rows;

    for( BOARD_ITEM* item : changed )
    {
        byKind[str( item->GetClass() )]++;
        rows.push_back( describe( item ) );
    }

    if( !changed.empty() )
        commit.Push( _( "Edit Text and Graphics" ) );

    nlohmann::json out = { { "changed", changed.size() }, { "by_kind", byKind } };
    cap.Put( out, "items", std::move( rows ) );
    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_graphics_cleanup( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*     board = context->GetBoard();
    const bool mergeRects = aArgs.value( "merge_rects", false );
    const bool deleteRedundant = aArgs.value( "delete_redundant", false );
    const bool fixOutlines = aArgs.value( "fix_board_outlines", false );
    const bool dryRun = aArgs.value( "dry_run", false );
    const double toleranceMm = aArgs.value( "tolerance_mm", 2.0 );

    if( !mergeRects && !deleteRedundant && !fixOutlines )
        return KOPENAPI_RESULT::Error( 400, "nothing to do: merge_rects, delete_redundant, fix_board_outlines" );

    if( toleranceMm < 0 )
        return KOPENAPI_RESULT::Error( 400, "tolerance_mm: >= 0" );

    BOARD_COMMIT                               commit( context->GetToolManager() );
    GRAPHICS_CLEANER                           cleaner( board->Drawings(), nullptr, commit, context->GetToolManager() );
    std::vector<std::shared_ptr<CLEANUP_ITEM>> found;

    board->CompileRatsnest();
    cleaner.CleanupBoard( dryRun, &found, mergeRects, deleteRedundant, false, fixOutlines,
                          pcbIUScale.mmToIU( toleranceMm ) );

    static const std::map<int, std::string> kinds = { { CLEANUP_NULL_GRAPHIC, "null_graphic" },
                                                      { CLEANUP_DUPLICATE_GRAPHIC, "duplicate_graphic" },
                                                      { CLEANUP_LINES_TO_RECT, "lines_to_rect" } };

    KOPENAPI_LIST_CAP           cap( aArgs, 50 );
    std::map<std::string, int>  counts;
    std::vector<nlohmann::json> rows;

    for( const std::shared_ptr<CLEANUP_ITEM>& item : found )
    {
        auto              it = kinds.find( item->GetErrorCode() );
        const std::string kind = it != kinds.end() ? it->second : str( item->GetSettingsKey() );
        nlohmann::json    ids = nlohmann::json::array();

        for( const KIID& id : item->GetIDs() )
            ids.push_back( str( id.AsString() ) );

        counts[kind]++;
        rows.push_back( { { "kind", kind }, { "items", ids } } );
    }

    const int edits = commit.Checkpoint();

    if( !dryRun && !commit.Empty() )
        commit.Push( _( "Cleanup Graphics" ) );

    nlohmann::json out = { { "dry_run", dryRun }, { "counts", counts }, { "changes", dryRun ? 0 : edits } };
    cap.Put( out, "items", std::move( rows ) );
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_text_graphics_edit",
                   "Edit Text and Graphics, as the board editor's dialog, in one undo step. items: the kinds "
                   "(references, values, other_fields, footprint_texts, footprint_graphics, "
                   "footprint_dimensions, board_texts, board_graphics, board_dimensions; a ${REFERENCE} / "
                   "${VALUE} text goes with references / values). Filters: layer_filter, reference_filter / "
                   "footprint_filter (wildcards; board items pass), selected_only. Then either layer_defaults: "
                   "true (each item takes Board Setup's defaults for its layer class) or the values to set: "
                   "layer, line_width_mm (shapes, dimensions), text_width_mm / text_height_mm / "
                   "text_thickness_mm or auto_thickness, bold, italic, font (\"KiCad Font\" or a font name), "
                   "keep_upright and center_on_footprint (footprint texts), visible (fields). Answers changed, "
                   "by_kind and the items (uuid, kind, layer, footprint, text)",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","required":["items"],"properties":{
                        "items":{"type":"array","items":{"type":"string","enum":["references","values","other_fields",
                            "footprint_texts","footprint_graphics","footprint_dimensions","board_texts",
                            "board_graphics","board_dimensions"]}},
                        "layer_filter":{"type":"string"},
                        "reference_filter":{"type":"string"},"footprint_filter":{"type":"string"},
                        "selected_only":{"type":"boolean","default":false},
                        "layer_defaults":{"type":"boolean","default":false},
                        "layer":{"type":"string"},
                        "line_width_mm":{"type":"number"},
                        "text_width_mm":{"type":"number"},"text_height_mm":{"type":"number"},
                        "text_thickness_mm":{"type":"number"},"auto_thickness":{"type":"boolean"},
                        "bold":{"type":"boolean"},"italic":{"type":"boolean"},"font":{"type":"string"},
                        "keep_upright":{"type":"boolean"},"visible":{"type":"boolean"},
                        "center_on_footprint":{"type":"boolean"}}})json"_json, 50 ),
                   false, h_pcb_text_graphics_edit );

KOPENAPI_REGISTER( "pcb_graphics_cleanup",
                   "Cleanup Graphics, as the board editor's dialog, on the board's own drawings in one undo "
                   "step: merge_rects (four lines forming a rectangle become one), delete_redundant (null and "
                   "duplicate graphics), fix_board_outlines (close Edge.Cuts gaps up to tolerance_mm, default "
                   "2). dry_run lists what would change. Answers counts per kind (null_graphic, "
                   "duplicate_graphic, lines_to_rect), the items (kind, uuids) and the edits made",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "merge_rects":{"type":"boolean","default":false},
                        "delete_redundant":{"type":"boolean","default":false},
                        "fix_board_outlines":{"type":"boolean","default":false},
                        "tolerance_mm":{"type":"number","default":2},
                        "dry_run":{"type":"boolean","default":false}}})json"_json, 50 ),
                   false, h_pcb_graphics_cleanup );

KOPENAPI_MARK_EDITING( "pcb_text_graphics_edit", "pcb_graphics_cleanup" );
