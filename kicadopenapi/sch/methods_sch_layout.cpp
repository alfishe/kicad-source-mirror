/// @file methods_sch_layout.cpp
/// @brief kicadopenapi schematic layout check: sch_layout_check.
///
/// Readability problems a render shows but ERC does not: texts lying on each other, a part's
/// reference / value text over another part's body, part bodies overlapping.  Each finding names
/// both items (reference, field or label text, uuid) and where, so an agent can move one of them
/// (sch_symbol_update field_positions / position) without having to read the picture.
///
/// A wire through a part's body (a pin reached from behind) is a finding.  Wiring a reader cannot
/// follow is a finding too: two local labels of one net close to each other
/// (a wire would show the link) and a sheet whose part pins are joined mostly by labels on the pins.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"
#include "kopenapi_sch_router.h"

#include <api/sch_context.h>
#include <base_units.h>
#include <kicadopenapi_util.h>
#include <sch_field.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <sch_text.h>
#include <schematic.h>
#include <string_utils.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

using namespace kopenapi_sch;


namespace
{

struct BOXED
{
    BOX2I       box;
    std::string kind;    ///< "text", "body" or "wire"
    bool        field = false;   ///< a part's field text (labels sit on wires by design)
    std::string owner;   ///< reference of the part it belongs to ("" for labels / texts)
    nlohmann::json what;
};


double mm( double aIU )
{
    return std::round( ( aIU / schIUScale.IU_PER_MM ) * 100.0 ) / 100.0;
}

} // namespace


static KOPENAPI_RESULT h_sch_layout_check( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const SCH_SHEET_LIST          hierarchy = context->GetSchematic()->Hierarchy();
    const std::string             wanted = aArgs.value( "sheet", std::string() );
    std::optional<SCH_SHEET_PATH> sheet;

    for( const SCH_SHEET_PATH& path : hierarchy )
    {
        if( wanted.empty() ? !sheet.has_value() : sheetPath( path ) == wanted )
            sheet = path;
    }

    if( !wanted.empty() && !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    if( wanted.empty() && context->GetCurrentSheet() )
        sheet = context->GetCurrentSheet();

    std::vector<BOXED> items;

    for( SCH_ITEM* item : sheet->LastScreen()->Items() )
    {
        if( item->Type() == SCH_SYMBOL_T )
        {
            SCH_SYMBOL*       symbol = static_cast<SCH_SYMBOL*>( item );
            const std::string ref = str( symbol->GetRef( &*sheet, false ) );
            const std::string uuid = str( symbol->m_Uuid.AsString() );

            if( !symbol->IsPower() )
                items.push_back( { symbol->GetBodyBoundingBox(), "body", false, ref, { { "ref", ref }, { "uuid", uuid } } } );

            std::vector<SCH_FIELD*> fields;
            symbol->GetFields( fields, true );

            for( SCH_FIELD* field : fields )
            {
                if( field->GetShownText( &*sheet, FOR_GUI ).IsEmpty() )
                    continue;

                items.push_back( { field->GetBoundingBox(), "text", true, ref,
                                   { { "ref", ref }, { "field", str( field->GetName() ) },
                                     { "text", str( field->GetShownText( &*sheet, FOR_GUI ) ) }, { "uuid", uuid } } } );
            }
        }
        else if( labelKind( item->Type() ) || item->Type() == SCH_TEXT_T )
        {
            const std::string text = str( UnescapeString( static_cast<SCH_TEXT*>( item )->GetText() ) );
            items.push_back( { item->GetBoundingBox(), "text", false, std::string(),
                               { { "label", labelKind( item->Type() ) ? labelKind( item->Type() ) : "text" },
                                 { "text", text }, { "uuid", str( item->m_Uuid.AsString() ) } } } );
        }
    }

    // Wires, for part texts lying on them (labels are meant to sit on wires)
    std::vector<SCH_LINE*> wires;

    for( SCH_ITEM* item : sheet->LastScreen()->Items().OfType( SCH_LINE_T ) )
    {
        if( static_cast<SCH_LINE*>( item )->IsWire() || static_cast<SCH_LINE*>( item )->IsBus() )
            wires.push_back( static_cast<SCH_LINE*>( item ) );
    }

    // Overlaps worth fixing: text on text, text on another part's body, body on body, a part's
    // text on a wire.  A part's own texts over its own body are its own design.
    const double                minArea = std::pow( schIUScale.mmToIU( 0.3 ), 2 );
    std::vector<nlohmann::json> findings;

    for( size_t i = 0; i < items.size(); ++i )
    {
        for( size_t j = i + 1; j < items.size(); ++j )
        {
            const BOXED& a = items[i];
            const BOXED& b = items[j];

            if( !a.owner.empty() && a.owner == b.owner )
                continue;

            if( !a.box.Intersects( b.box ) )
                continue;

            BOX2I        overlap = BOX2I( a.box ).Intersect( b.box );
            const double area = double( overlap.GetWidth() ) * overlap.GetHeight();

            if( area < minArea )
                continue;

            const std::string kind = a.kind == "body" && b.kind == "body" ? "body_on_body"
                                     : a.kind == "text" && b.kind == "text" ? "text_on_text"
                                                                            : "text_on_body";

            findings.push_back( { { "kind", kind },
                                  { "a", a.what },
                                  { "b", b.what },
                                  { "at_mm", { mm( overlap.Centre().x ), mm( overlap.Centre().y ) } },
                                  { "area_mm2", std::round( schIUScale.IUTomm( overlap.GetWidth() )
                                                            * schIUScale.IUTomm( overlap.GetHeight() ) * 100.0 ) / 100.0 } } );
        }
    }

    for( const BOXED& text : items )
    {
        if( !text.field )
            continue;

        for( SCH_LINE* wire : wires )
        {
            // shrink the text box a little: a wire grazing the edge of the text is fine
            BOX2I inner = text.box;
            inner.Inflate( -schIUScale.mmToIU( 0.2 ) );

            if( inner.GetWidth() <= 0 || inner.GetHeight() <= 0 || !wire->HitTest( inner, false, 0 ) )
                continue;

            const VECTOR2I a = wire->GetStartPoint(), b = wire->GetEndPoint();
            findings.push_back( { { "kind", "text_on_wire" },
                                  { "a", text.what },
                                  { "b", { { "wire", str( wire->m_Uuid.AsString() ) },
                                           { "from", { mm( a.x ), mm( a.y ) } }, { "to", { mm( b.x ), mm( b.y ) } } } },
                                  { "at_mm", { mm( inner.Centre().x ), mm( inner.Centre().y ) } } } );
            break;
        }
    }

    // A wire through a part's body: it reads as a connection inside the part, or reaches a pin from
    // behind instead of at its free end
    for( const BOXED& body : items )
    {
        if( body.kind != "body" )
            continue;

        for( SCH_LINE* wire : wires )
        {
            const VECTOR2I a = wire->GetStartPoint(), b = wire->GetEndPoint();

            if( ( a.x != b.x && a.y != b.y ) || !SegmentCrossesBody( a, b, body.box ) )
                continue;

            const BOX2I overlap = BOX2I( body.box ).Intersect( BOX2I::ByCorners( a, b ).Inflate( 1 ) );
            findings.push_back( { { "kind", "wire_through_body" },
                                  { "a", body.what },
                                  { "b", { { "wire", str( wire->m_Uuid.AsString() ) },
                                           { "from", { mm( a.x ), mm( a.y ) } }, { "to", { mm( b.x ), mm( b.y ) } } } },
                                  { "at_mm", { mm( overlap.Centre().x ), mm( overlap.Centre().y ) } } } );
        }
    }

    // Power symbols of different rails on one net: a short (+3V3 on GND)
    for( const NET_ENTRY& net : collectNets( context->GetSchematic() ) )
    {
        std::map<std::string, std::string> rails;   // value -> uuid of one symbol on this sheet
        bool                               here = false;

        for( const auto& [pin, path] : netPins( net ) )
        {
            SCH_SYMBOL* symbol = pinSymbol( pin );

            if( !symbol || !isRailSymbol( symbol ) )
                continue;

            const std::string value = str( symbol->GetValue( &path, FOR_GUI ) );

            const bool onSheet = path == *sheet;
            here |= onSheet;

            if( !rails.count( value ) || onSheet )
                rails[value] = onSheet ? str( symbol->m_Uuid.AsString() ) : std::string();
        }

        if( rails.size() < 2 || !here )
            continue;

        nlohmann::json values = nlohmann::json::array();

        for( const auto& [value, uuid] : rails )
            values.push_back( { { "value", value }, { "uuid", uuid } } );

        findings.push_back( { { "kind", "power_nets_joined" },
                              { "a", { { "net", net.name } } },
                              { "b", { { "rails", values } } },
                              { "at_mm", nlohmann::json::array() } } );
    }

    // Wiring by labels: a local label is a name a reader has to search for, fine for a far or
    // another block's end, not for parts side by side
    const double nearMm = aArgs.value( "near_mm", 25.4 );
    std::map<wxString, std::vector<SCH_LABEL_BASE*>> locals;
    std::set<std::pair<int, int>>                    labelPoints, powerPoints;

    for( SCH_ITEM* item : sheet->LastScreen()->Items().OfType( SCH_LABEL_T ) )
    {
        SCH_LABEL_BASE* label = static_cast<SCH_LABEL_BASE*>( item );
        locals[label->GetText()].push_back( label );
        labelPoints.insert( { label->GetPosition().x, label->GetPosition().y } );
    }

    std::set<std::pair<SCH_LABEL_BASE*, SCH_LABEL_BASE*>> reported;

    for( const auto& [name, labels] : locals )
    {
        for( SCH_LABEL_BASE* label : labels )
        {
            SCH_LABEL_BASE* nearest = nullptr;
            double          best = nearMm;

            for( SCH_LABEL_BASE* other : labels )
            {
                const double d = schIUScale.IUTomm( ( other->GetPosition() - label->GetPosition() ).EuclideanNorm() );

                if( other != label && d <= best )
                {
                    best = d;
                    nearest = other;
                }
            }

            if( !nearest || !reported.insert( std::minmax( label, nearest ) ).second )
                continue;

            const VECTOR2I a = label->GetPosition(), b = nearest->GetPosition();
            findings.push_back( { { "kind", "labels_instead_of_wire" },
                                  { "a", { { "label", "local" }, { "text", str( UnescapeString( name ) ) },
                                           { "uuid", str( label->m_Uuid.AsString() ) }, { "at_mm", { mm( a.x ), mm( a.y ) } } } },
                                  { "b", { { "label", "local" }, { "text", str( UnescapeString( name ) ) },
                                           { "uuid", str( nearest->m_Uuid.AsString() ) }, { "at_mm", { mm( b.x ), mm( b.y ) } } } },
                                  { "distance_mm", std::round( best * 100.0 ) / 100.0 },
                                  { "at_mm", { mm( ( a.x + b.x ) / 2.0 ), mm( ( a.y + b.y ) / 2.0 ) } } } );
        }
    }

    // How the parts' pins are joined: by a wire, by a label put on the pin, by a power symbol
    int wired = 0, labelled = 0, powered = 0;

    for( SCH_ITEM* item : sheet->LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
    {
        if( static_cast<SCH_SYMBOL*>( item )->IsPower() )
        {
            for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( item )->GetPins( &*sheet ) )
                powerPoints.insert( { pin->GetPosition().x, pin->GetPosition().y } );
        }
    }

    for( SCH_ITEM* item : sheet->LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

        if( symbol->IsPower() )
            continue;

        for( SCH_PIN* pin : symbol->GetPins( &*sheet ) )
        {
            const VECTOR2I at = pin->GetPosition();

            if( std::any_of( wires.begin(), wires.end(), [&]( SCH_LINE* w ) { return w->HitTest( at, 0 ); } ) )
                wired++;
            else if( labelPoints.count( { at.x, at.y } ) )
                labelled++;
            else if( powerPoints.count( { at.x, at.y } ) )
                powered++;
        }
    }

    if( labelled >= 8 && labelled > wired )
    {
        findings.push_back( { { "kind", "wiring_by_labels" },
                              { "a", { { "pins_labelled", labelled }, { "pins_wired", wired }, { "pins_on_power", powered } } },
                              { "b", nlohmann::json::object() },
                              { "at_mm", nlohmann::json::array() } } );
    }

    nlohmann::json result = KopenapiPage( findings, aArgs );
    result["sheet"] = sheetPath( *sheet );
    result["pins"] = { { "wired", wired }, { "labelled", labelled }, { "on_power", powered } };
    result["clean"] = findings.empty();
    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "sch_layout_check",
                   "Readability check of a sheet (what ERC does not see): texts lying on each other, a "
                   "part's reference / value text over another part's body or on a wire, overlapping part "
                   "bodies, a wire through a part's body (wire_through_body: a pin reached from behind), "
                   "power symbols of different rails on one net (power_nets_joined: a short), "
                   "wiring a reader cannot follow (labels_instead_of_wire: two local labels of a net "
                   "within near_mm, draw a wire; wiring_by_labels: most part pins joined by labels on the pins "
                   "rather than wires; pins: how pins are joined); each "
                   "finding names both items (ref + field, or label text, uuid) and where (mm), so one can be "
                   "moved with sch_symbol_update (field_positions / x_mm, y_mm); clean flag; paginated",
                   KopenapiPagedSchema( R"json({"sheet":{"type":"string"},
                                                     "near_mm":{"type":"number","default":25.4,"description":"local labels of one net closer than this are a finding"}})json"_json ),
                   false, h_sch_layout_check );
