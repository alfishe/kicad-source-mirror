/*
 * kicadopenapi schematic layout check: sch_layout_check.
 *
 * Readability problems a render shows but ERC does not: texts lying on each other, a part's
 * reference / value text over another part's body, part bodies overlapping.  Each finding names
 * both items (reference, field or label text, uuid) and where, so an agent can move one of them
 * (sch_symbol_update field_positions / position) without having to read the picture.
 */
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <base_units.h>
#include <kicadopenapi_util.h>
#include <sch_field.h>
#include <sch_label.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <sch_text.h>
#include <schematic.h>
#include <string_utils.h>

#include <cmath>

using namespace kopenapi_sch;


namespace
{

struct BOXED
{
    BOX2I       box;
    std::string kind;    ///< "text" or "body"
    std::string owner;   ///< reference of the part it belongs to ("" for labels / texts)
    nlohmann::json what;
};


double mm( double aIU )
{
    return std::round( schIUScale.IUTomm( aIU ) * 100.0 ) / 100.0;
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
                items.push_back( { symbol->GetBodyBoundingBox(), "body", ref, { { "ref", ref }, { "uuid", uuid } } } );

            std::vector<SCH_FIELD*> fields;
            symbol->GetFields( fields, true );

            for( SCH_FIELD* field : fields )
            {
                if( field->GetShownText( &*sheet, FOR_GUI ).IsEmpty() )
                    continue;

                items.push_back( { field->GetBoundingBox(), "text", ref,
                                   { { "ref", ref }, { "field", str( field->GetName() ) },
                                     { "text", str( field->GetShownText( &*sheet, FOR_GUI ) ) }, { "uuid", uuid } } } );
            }
        }
        else if( labelKind( item->Type() ) || item->Type() == SCH_TEXT_T )
        {
            const std::string text = str( UnescapeString( static_cast<SCH_TEXT*>( item )->GetText() ) );
            items.push_back( { item->GetBoundingBox(), "text", std::string(),
                               { { "label", labelKind( item->Type() ) ? labelKind( item->Type() ) : "text" },
                                 { "text", text }, { "uuid", str( item->m_Uuid.AsString() ) } } } );
        }
    }

    // Overlaps worth fixing: text on text, text on another part's body, body on body.  A part's
    // own texts over its own body are its own design.
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

    nlohmann::json result = KopenapiPage( findings, aArgs );
    result["sheet"] = sheetPath( *sheet );
    result["clean"] = findings.empty();
    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "sch_layout_check",
                   "Readability check of a sheet (what ERC does not see): texts lying on each other, a "
                   "part's reference / value text over another part's body, overlapping part bodies; each "
                   "finding names both items (ref + field, or label text, uuid) and where (mm), so one can be "
                   "moved with sch_symbol_update (field_positions / x_mm, y_mm); clean flag; paginated",
                   KopenapiPagedSchema( R"json({"sheet":{"type":"string"}})json"_json ),
                   false, h_sch_layout_check );
