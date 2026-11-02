/// @file methods_sch_bulk_edit.cpp
/// @brief kicadopenapi sch_text_graphics_edit (the schematic editor's Edit Text and Graphics, through
/// SchGlobalEditTextAndGraphics) and sch_cleanup (Cleanup Sheet Pins and the editor's wire cleanup).
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <base_units.h>
#include <font/font.h>
#include <font/kicad_font_name.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <sch_commit.h>
#include <sch_field.h>
#include <sch_global_edit_text_and_graphics.h>
#include <sch_junction.h>
#include <sch_line.h>
#include <sch_no_connect.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <schematic.h>

#include <map>
#include <set>
#include <unordered_set>

using namespace kopenapi_sch;


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


wxString wx( const std::string& aText )
{
    return wxString::FromUTF8( aText );
}


/// "#rrggbb", "#rrggbbaa" or a CSS rgb() / rgba(); "default" (or "") is the unspecified colour
std::optional<KIGFX::COLOR4D> parseColor( const nlohmann::json& aValue )
{
    if( !aValue.is_string() )
        return std::nullopt;

    const wxString text = wx( aValue.get<std::string>() );

    if( text.IsEmpty() || text == wxS( "default" ) )
        return KIGFX::COLOR4D::UNSPECIFIED;

    KIGFX::COLOR4D color;

    if( text.StartsWith( wxS( "#" ) ) ? color.SetFromHexString( text ) : color.SetFromWxString( text ) )
        return color;

    return std::nullopt;
}


nlohmann::json describe( const SCH_GLOBAL_EDIT_CHANGE& aChange )
{
    SCH_ITEM*      item = aChange.m_Item;
    nlohmann::json row = { { "kind", str( item->GetClass() ) }, { "sheet", sheetPath( aChange.m_Sheet ) } };

    if( SCH_FIELD* field = dynamic_cast<SCH_FIELD*>( item ) )
    {
        row["field"] = str( field->GetName() );

        if( SCH_ITEM* parent = dynamic_cast<SCH_ITEM*>( field->GetParent() ) )
        {
            row["uuid"] = str( parent->m_Uuid.AsString() );

            if( parent->Type() == SCH_SYMBOL_T )
                row["ref"] = str( static_cast<SCH_SYMBOL*>( parent )->GetRef( &aChange.m_Sheet, false ) );
        }

        row["text"] = str( field->GetText() );
        return row;
    }

    row["uuid"] = str( item->m_Uuid.AsString() );

    if( EDA_TEXT* text = dynamic_cast<EDA_TEXT*>( item ) )
        row["text"] = str( text->GetText() );

    return row;
}


KOPENAPI_RESULT h_sch_text_graphics_edit( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC*              schematic = context->GetSchematic();
    SCH_GLOBAL_EDIT_OPTIONS opt;

    static const std::map<std::string, bool SCH_GLOBAL_EDIT_OPTIONS::*> scopes = {
        { "references", &SCH_GLOBAL_EDIT_OPTIONS::references },
        { "values", &SCH_GLOBAL_EDIT_OPTIONS::values },
        { "other_fields", &SCH_GLOBAL_EDIT_OPTIONS::otherFields },
        { "wires", &SCH_GLOBAL_EDIT_OPTIONS::wires },
        { "buses", &SCH_GLOBAL_EDIT_OPTIONS::buses },
        { "global_labels", &SCH_GLOBAL_EDIT_OPTIONS::globalLabels },
        { "hier_labels", &SCH_GLOBAL_EDIT_OPTIONS::hierLabels },
        { "label_fields", &SCH_GLOBAL_EDIT_OPTIONS::labelFields },
        { "sheet_titles", &SCH_GLOBAL_EDIT_OPTIONS::sheetTitles },
        { "sheet_fields", &SCH_GLOBAL_EDIT_OPTIONS::sheetFields },
        { "sheet_pins", &SCH_GLOBAL_EDIT_OPTIONS::sheetPins },
        { "sheet_borders", &SCH_GLOBAL_EDIT_OPTIONS::sheetBorders },
        { "text_and_graphics", &SCH_GLOBAL_EDIT_OPTIONS::textAndGraphics }
    };

    const nlohmann::json items = aArgs.value( "items", nlohmann::json::array() );

    if( !items.is_array() || items.empty() )
        return KOPENAPI_RESULT::Error( 400, "items: the kinds to edit (references, values, other_fields, wires, "
                                            "buses, global_labels, hier_labels, label_fields, sheet_titles, "
                                            "sheet_fields, sheet_pins, sheet_borders, text_and_graphics)" );

    for( const nlohmann::json& kind : items )
    {
        auto it = kind.is_string() ? scopes.find( kind.get<std::string>() ) : scopes.end();

        if( it == scopes.end() )
            return KOPENAPI_RESULT::Error( 400, "items: unknown kind " + kind.dump() );

        opt.*( it->second ) = true;
    }

    opt.selectedOnly = aArgs.value( "selected_only", false );
    opt.fieldNameFilter = wx( aArgs.value( "field_name_filter", std::string() ) );
    opt.referenceFilter = wx( aArgs.value( "reference_filter", std::string() ) );
    opt.symbolFilter = wx( aArgs.value( "symbol_filter", std::string() ) );
    opt.netFilter = wx( aArgs.value( "net_filter", std::string() ) );

    if( aArgs.contains( "symbol_type" ) )
    {
        const std::string type = aArgs.value( "symbol_type", std::string() );

        if( type != "power" && type != "non_power" )
            return KOPENAPI_RESULT::Error( 400, "symbol_type: power or non_power" );

        opt.powerSymbols = type == "power";
    }

    bool        anySet = false;
    std::string bad;

    auto size = [&]( const char* aKey, std::optional<int>& aOut, double aMinMm, double aMaxMm )
    {
        if( !aArgs.contains( aKey ) )
            return;

        anySet = true;

        if( !aArgs[aKey].is_number() || aArgs[aKey].get<double>() < aMinMm || aArgs[aKey].get<double>() > aMaxMm )
            bad = aKey;
        else
            aOut = schIUScale.mmToIU( aArgs[aKey].get<double>() );
    };

    // the dialog's bounds: text 1 mil .. 10 inches
    size( "text_size_mm", opt.textSize, 0.0254, 254.0 );
    size( "line_width_mm", opt.lineWidth, 0.0, 254.0 );
    size( "junction_size_mm", opt.junctionSize, 0.0, 254.0 );

    if( !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad + ": out of range (text 0.0254 .. 254 mm, widths 0 .. 254 mm)" );

    auto flag = [&]( const char* aKey, std::optional<bool>& aOut )
    {
        if( !aArgs.contains( aKey ) )
            return;

        anySet = true;

        if( aArgs[aKey].is_boolean() )
            aOut = aArgs[aKey].get<bool>();
        else
            bad = aKey;
    };

    flag( "italic", opt.italic );
    flag( "bold", opt.bold );
    flag( "visible", opt.visible );
    flag( "show_field_names", opt.showFieldNames );

    auto color = [&]( const char* aKey, std::optional<KIGFX::COLOR4D>& aOut )
    {
        if( !aArgs.contains( aKey ) )
            return;

        anySet = true;
        aOut = parseColor( aArgs[aKey] );

        if( !aOut )
            bad = aKey;
    };

    color( "text_color", opt.textColor );
    color( "line_color", opt.lineColor );
    color( "fill_color", opt.fillColor );
    color( "junction_color", opt.junctionColor );

    if( !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad + ": a boolean / a colour (#rrggbb, #rrggbbaa, rgb(), default)" );

    auto choice = [&]<typename T>( const char* aKey, std::optional<T>& aOut,
                                   const std::map<std::string, T>& aChoices )
    {
        if( !aArgs.contains( aKey ) )
            return;

        anySet = true;
        auto it = aArgs[aKey].is_string() ? aChoices.find( aArgs[aKey].get<std::string>() ) : aChoices.end();

        if( it == aChoices.end() )
            bad = aKey;
        else
            aOut = it->second;
    };

    choice( "h_align", opt.hAlign,
            std::map<std::string, GR_TEXT_H_ALIGN_T>{ { "left", GR_TEXT_H_ALIGN_LEFT },
                                                      { "center", GR_TEXT_H_ALIGN_CENTER },
                                                      { "right", GR_TEXT_H_ALIGN_RIGHT } } );
    choice( "v_align", opt.vAlign,
            std::map<std::string, GR_TEXT_V_ALIGN_T>{ { "top", GR_TEXT_V_ALIGN_TOP },
                                                      { "center", GR_TEXT_V_ALIGN_CENTER },
                                                      { "bottom", GR_TEXT_V_ALIGN_BOTTOM } } );
    choice( "label_orientation", opt.labelOrientation,
            std::map<std::string, SPIN_STYLE::SPIN>{ { "left", SPIN_STYLE::LEFT },
                                                     { "up", SPIN_STYLE::UP },
                                                     { "right", SPIN_STYLE::RIGHT },
                                                     { "bottom", SPIN_STYLE::BOTTOM } } );
    choice( "line_style", opt.lineStyle,
            std::map<std::string, LINE_STYLE>{ { "solid", LINE_STYLE::SOLID },
                                               { "dash", LINE_STYLE::DASH },
                                               { "dot", LINE_STYLE::DOT },
                                               { "dash_dot", LINE_STYLE::DASHDOT },
                                               { "dash_dot_dot", LINE_STYLE::DASHDOTDOT } } );

    if( !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad + ": not one of the choices (see the schema)" );

    if( aArgs.contains( "font" ) )
    {
        anySet = true;

        if( !aArgs["font"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "font: a font name (\"KiCad Font\" for the stroke font, "
                                                "\"default\" for the schematic's default)" );

        const wxString name = wx( aArgs["font"].get<std::string>() );

        opt.font = [name]( bool aBold, bool aItalic ) -> KIFONT::FONT*
        {
            if( name.IsEmpty() || name == wxS( "default" ) )
                return nullptr;

            return KIFONT::FONT::GetFont( name, aBold, aItalic );
        };
    }

    if( !anySet )
        return KOPENAPI_RESULT::Error( 400, "nothing to set: text_size_mm, text_color, h_align, v_align, italic, "
                                            "bold, font, visible, show_field_names, label_orientation, "
                                            "line_width_mm, line_style, line_color, fill_color, "
                                            "junction_size_mm, junction_color" );

    SCH_COMMIT                          commit( context->GetToolManager() );
    std::vector<SCH_GLOBAL_EDIT_CHANGE> changed = SchGlobalEditTextAndGraphics( *schematic, commit, opt );

    KOPENAPI_LIST_CAP           cap( aArgs, 50 );
    std::map<std::string, int>  byKind;
    std::vector<nlohmann::json> rows;

    for( const SCH_GLOBAL_EDIT_CHANGE& change : changed )
    {
        byKind[str( change.m_Item->GetClass() )]++;
        rows.push_back( describe( change ) );
    }

    if( !commit.Empty() )
        KopenapiSchPushEdit( commit, schematic, _( "Edit Text and Graphics" ), aCtx.kiway, {} );

    nlohmann::json out = { { "changed", changed.size() }, { "by_kind", byKind } };
    cap.Put( out, "items", std::move( rows ) );
    return KOPENAPI_RESULT::Ok( out );
}


struct WIRE_COUNTS
{
    int wires = 0;
    int buses = 0;
    int junctions = 0;
    int noConnects = 0;
};


WIRE_COUNTS countWiring( SCHEMATIC* aSchematic )
{
    WIRE_COUNTS                     counts;
    std::unordered_set<SCH_SCREEN*> seen;

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        SCH_SCREEN* screen = path.LastScreen();

        if( !screen || !seen.insert( screen ).second )
            continue;

        for( SCH_ITEM* item : screen->Items() )
        {
            if( item->Type() == SCH_LINE_T && item->GetLayer() == LAYER_WIRE )
                counts.wires++;
            else if( item->Type() == SCH_LINE_T && item->GetLayer() == LAYER_BUS )
                counts.buses++;
            else if( item->Type() == SCH_JUNCTION_T )
                counts.junctions++;
            else if( item->Type() == SCH_NO_CONNECT_T )
                counts.noConnects++;
        }
    }

    return counts;
}


KOPENAPI_RESULT h_sch_cleanup( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC* schematic = context->GetSchematic();
    const bool sheetPins = aArgs.value( "sheet_pins", false );
    const bool wires = aArgs.value( "wires", false );
    const bool dryRun = aArgs.value( "dry_run", false );

    if( !sheetPins && !wires )
        return KOPENAPI_RESULT::Error( 400, "nothing to do: sheet_pins, wires" );

    SCH_COMMIT     commit( context->GetToolManager() );
    nlohmann::json out = { { "dry_run", dryRun } };

    if( sheetPins )
    {
        // Cleanup Sheet Pins on every sheet symbol with pins its sub-sheet has no label for
        nlohmann::json                 rows = nlohmann::json::array();
        std::unordered_set<SCH_SHEET*> seen;
        int                            removed = 0;

        for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
        {
            SCH_SCREEN* screen = path.LastScreen();

            for( SCH_ITEM* item : screen->Items().OfType( SCH_SHEET_T ) )
            {
                SCH_SHEET* sheet = static_cast<SCH_SHEET*>( item );

                if( !seen.insert( sheet ).second || !sheet->HasUndefinedPins() )
                    continue;

                std::set<wxString> before;

                for( SCH_SHEET_PIN* pin : sheet->GetPins() )
                    before.insert( pin->GetText() );

                commit.Modify( sheet, screen );
                sheet->CleanupSheet();

                for( SCH_SHEET_PIN* pin : sheet->GetPins() )
                    before.erase( pin->GetText() );

                nlohmann::json pins = nlohmann::json::array();

                for( const wxString& name : before )
                    pins.push_back( str( name ) );

                removed += (int) before.size();
                rows.push_back( { { "sheet", str( sheet->GetName() ) },
                                  { "uuid", str( sheet->m_Uuid.AsString() ) },
                                  { "pins_removed", pins } } );
            }
        }

        out["sheet_pins"] = { { "removed", removed }, { "sheets", rows } };
    }

    if( wires )
    {
        // the editor's cleanup after an edit, on every screen: junctions not needed, repeated
        // junctions and no-connects, null and repeated wires, overlapping or touching collinear
        // wires merged
        const WIRE_COUNTS before = countWiring( schematic );

        schematic->CleanUpConnections( &commit, GLOBAL_CLEANUP );

        const WIRE_COUNTS after = countWiring( schematic );

        out["wiring"] = { { "wires", { { "before", before.wires }, { "after", after.wires } } },
                          { "buses", { { "before", before.buses }, { "after", after.buses } } },
                          { "junctions", { { "before", before.junctions }, { "after", after.junctions } } },
                          { "no_connects", { { "before", before.noConnects }, { "after", after.noConnects } } } };
    }

    out["changed"] = !commit.Empty();

    if( dryRun )
    {
        commit.Revert();
        schematic->RebuildConnectivity();
    }
    else if( !commit.Empty() )
    {
        KopenapiSchPushEdit( commit, schematic, _( "Cleanup Schematic" ), aCtx.kiway, {} );
    }

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "sch_text_graphics_edit",
                   "Edit Text and Graphics, as the schematic editor's dialog, over every sheet in one undo step. "
                   "items: the kinds (references, values, other_fields, wires, buses, global_labels, hier_labels, "
                   "label_fields, sheet_titles, sheet_fields, sheet_pins, sheet_borders, text_and_graphics: "
                   "notes, text boxes, shapes). Filters: field_name_filter (other / sheet / label fields), "
                   "reference_filter and symbol_filter (lib id; wildcards, symbols only), net_filter (items on a "
                   "matching net), symbol_type (power / non_power), selected_only. Values: text_size_mm, "
                   "text_color, h_align, v_align (left/center/right, top/center/bottom; mirrored symbols' fields "
                   "flip as in the dialog), italic, bold, font (a name, \"KiCad Font\", \"default\"), visible and "
                   "show_field_names (fields), label_orientation (left/up/right/bottom), line_width_mm, "
                   "line_style, line_color, fill_color (default: no fill), junction_size_mm, junction_color. "
                   "Colours: #rrggbb, #rrggbbaa, rgb(), default. Answers changed, by_kind and the items (kind, "
                   "sheet, uuid, field and ref for fields, text)",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","required":["items"],"properties":{
                        "items":{"type":"array","items":{"type":"string","enum":["references","values",
                            "other_fields","wires","buses","global_labels","hier_labels","label_fields",
                            "sheet_titles","sheet_fields","sheet_pins","sheet_borders","text_and_graphics"]}},
                        "field_name_filter":{"type":"string"},"reference_filter":{"type":"string"},
                        "symbol_filter":{"type":"string"},"net_filter":{"type":"string"},
                        "symbol_type":{"type":"string","enum":["power","non_power"]},
                        "selected_only":{"type":"boolean","default":false},
                        "text_size_mm":{"type":"number"},"text_color":{"type":"string"},
                        "h_align":{"type":"string","enum":["left","center","right"]},
                        "v_align":{"type":"string","enum":["top","center","bottom"]},
                        "italic":{"type":"boolean"},"bold":{"type":"boolean"},"font":{"type":"string"},
                        "visible":{"type":"boolean"},"show_field_names":{"type":"boolean"},
                        "label_orientation":{"type":"string","enum":["left","up","right","bottom"]},
                        "line_width_mm":{"type":"number"},
                        "line_style":{"type":"string","enum":["solid","dash","dot","dash_dot","dash_dot_dot"]},
                        "line_color":{"type":"string"},"fill_color":{"type":"string"},
                        "junction_size_mm":{"type":"number"},"junction_color":{"type":"string"}}})json"_json, 50 ),
                   false, h_sch_text_graphics_edit );

KOPENAPI_REGISTER( "sch_cleanup",
                   "Schematic cleanups in one undo step. sheet_pins: Cleanup Sheet Pins on every sheet symbol "
                   "whose sub-sheet has no hierarchical label for some of its pins (answers the sheets and the "
                   "pins removed). wires: the editor's wiring cleanup on every sheet (junctions not needed, "
                   "repeated junctions / no-connects, null and repeated wires removed, overlapping or touching "
                   "collinear wires merged; answers wires, buses, junctions, no_connects before / after). "
                   "dry_run answers the same and changes nothing",
                   R"json({"type":"object","properties":{
                        "sheet_pins":{"type":"boolean","default":false},
                        "wires":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_cleanup );

KOPENAPI_MARK_EDITING( "sch_text_graphics_edit", "sch_cleanup" );
