/// @file methods_sch_bom.cpp
/// @brief kicadopenapi bill of materials: sch_bom, built on the data model of KiCad's Symbol Fields
/// Table / Export BOM (the same grouping, quantities, reference ranges and presets as kicad-cli sch
/// export bom).

#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kicadopenapi_util.h>
#include <project.h>
#include <project/project_file.h>
#include <sch_reference_list.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <schematic_settings.h>
#include <settings/bom_settings.h>
#include <symbol_fields_data_model.h>
#include <template_fieldnames.h>

#include <wx/file.h>

#include <set>

using namespace kopenapi_sch;


/// @brief Field names as the BOM takes them: bare generated names ("QUANTITY") as "${QUANTITY}"
static wxString bomFieldName( SYMBOL_FIELDS_EDITOR_GRID_DATA_MODEL& aModel, const std::string& aName )
{
    wxString name = wxString::FromUTF8( aName );
    wxString wrapped = wxS( "${" ) + name + wxS( "}" );

    if( !IsGeneratedField( name ) && IsGeneratedField( wrapped ) && aModel.GetFieldNameCol( wrapped ) != -1 )
        return wrapped;

    return name;
}


static KOPENAPI_RESULT h_sch_bom( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC* sch = context->GetSchematic();

    SCH_REFERENCE_LIST references;
    sch->Hierarchy().GetSymbols( references, SYMBOL_FILTER_NON_POWER, false );

    SYMBOL_FIELDS_EDITOR_GRID_DATA_MODEL model( references );
    const wxString variant = wxString::FromUTF8( aArgs.value( "variant", std::string() ) );
    model.SetVariantNames( { variant } );
    model.SetCurrentVariant( variant );

    for( FIELD_T fieldId : MANDATORY_FIELDS )
        model.AddColumn( GetDefaultFieldName( fieldId, UNTRANSLATED ), GetDefaultFieldName( fieldId, TRANSLATED ),
                         false );

    for( const wxString& generated : { SYMBOL_FIELDS_EDITOR_GRID_DATA_MODEL::QUANTITY_VARIABLE,
                                       SYMBOL_FIELDS_EDITOR_GRID_DATA_MODEL::ITEM_NUMBER_VARIABLE,
                                       wxString( wxS( "${DNP}" ) ), wxString( wxS( "${EXCLUDE_FROM_BOM}" ) ),
                                       wxString( wxS( "${EXCLUDE_FROM_BOARD}" ) ),
                                       wxString( wxS( "${EXCLUDE_FROM_SIM}" ) ) } )
    {
        model.AddColumn( generated, GetGeneratedFieldDisplayName( generated ), false );
    }

    std::set<wxString> userFields;

    for( size_t i = 0; i < references.GetCount(); ++i )
    {
        for( SCH_FIELD& field : references[i].GetSymbol()->GetFields() )
        {
            if( !field.IsMandatory() && !field.IsPrivate() )
                userFields.insert( field.GetName() );
        }
    }

    for( const wxString& name : userFields )
        model.AddColumn( name, GetGeneratedFieldDisplayName( name ), true );

    for( const TEMPLATE_FIELDNAME& templateField :
         sch->Project().GetProjectFile().m_TemplateFieldNames.GetResolvedTemplateFieldNames() )
    {
        if( !userFields.count( templateField.m_Name ) )
            model.AddColumn( templateField.m_Name, GetGeneratedFieldDisplayName( templateField.m_Name ), false );
    }

    BOM_PRESET preset;
    const std::string presetName = aArgs.value( "preset", std::string() );

    if( !presetName.empty() )
    {
        const BOM_PRESET* found = nullptr;

        for( const BOM_PRESET& p : BOM_PRESET::BuiltInPresets() )
        {
            if( str( p.name ) == presetName )
                found = &p;
        }

        for( const BOM_PRESET& p : sch->Settings().m_BomPresets )
        {
            if( str( p.name ) == presetName )
                found = &p;
        }

        if( !found )
        {
            std::vector<std::string> names;

            for( const BOM_PRESET& p : BOM_PRESET::BuiltInPresets() )
                names.push_back( str( p.name ) );

            for( const BOM_PRESET& p : sch->Settings().m_BomPresets )
                names.push_back( str( p.name ) );

            KOPENAPI_RESULT result = KOPENAPI_RESULT::Error( 404, "BOM preset '" + presetName + "' not found" );
            result.body["error"]["presets"] = names;
            return result;
        }

        preset = *found;
    }
    else
    {
        std::vector<std::string> fields = aArgs.value(
                "fields", std::vector<std::string>{ "Reference", "Value", "Footprint", "${QUANTITY}", "${DNP}" } );
        std::vector<std::string> groupBy =
                aArgs.value( "group_by", std::vector<std::string>{ "Value", "Footprint", "${DNP}" } );

        auto addField = [&]( const wxString& aName, bool aShow )
        {
            for( const BOM_FIELD& f : preset.fieldsOrdered )
            {
                if( f.name == aName )
                    return;
            }

            preset.fieldsOrdered.push_back(
                    { aName, IsGeneratedField( aName ) ? GetGeneratedFieldDisplayName( aName ) : aName, aShow, false } );
        };

        for( const std::string& name : fields )
        {
            if( name == "*" )
            {
                for( const BOM_FIELD& f : model.GetFieldsOrdered() )
                    addField( f.name, true );
            }
            else
            {
                addField( bomFieldName( model, name ), true );
            }
        }

        for( const std::string& name : groupBy )
        {
            const wxString field = bomFieldName( model, name );
            addField( field, false );

            for( BOM_FIELD& f : preset.fieldsOrdered )
            {
                if( f.name == field )
                    f.groupBy = true;
            }
        }

        preset.sortField = bomFieldName( model, aArgs.value( "sort_field", std::string( "Reference" ) ) );
        preset.sortAsc = aArgs.value( "sort_asc", true );
        preset.filterString = wxString::FromUTF8( aArgs.value( "filter", std::string() ) );
        preset.groupSymbols = aArgs.value( "group", true );
        preset.excludeDNP = aArgs.value( "exclude_dnp", false );
        preset.includeExcludedFromBOM = aArgs.value( "include_excluded_from_bom", false );
    }

    model.UpdateReferences( model.GetReferenceList() );
    model.ApplyBomPreset( preset );

    std::vector<int>  shown;
    nlohmann::json    columns = nlohmann::json::array();

    for( int col = 0; col < model.GetNumberCols(); ++col )
    {
        if( !model.GetShowColumn( col ) )
            continue;

        shown.push_back( col );
        columns.push_back( { { "field", str( model.GetColFieldName( col ) ) },
                             { "label", str( model.GetColLabelValue( col ) ) },
                             { "group_by", model.GetGroupColumn( col ) } } );
    }

    std::vector<nlohmann::json> rows;
    int                         parts = 0;

    for( int row = 0; row < model.GetNumberRows(); ++row )
    {
        if( model.GetRowState( row ) == ROW_STATE::EXPANDED_CHILD )
            continue;

        nlohmann::json item = nlohmann::json::object();

        for( int col : shown )
        {
            const std::string value = str( model.GetExportValue( row, col, wxS( "," ), wxS( "-" ) ) );

            if( model.ColIsReference( col ) )
            {
                std::vector<std::string> refs;
                std::set<std::string>    seen;

                for( const SCH_REFERENCE& ref : model.GetRowReferences( row ) )
                {
                    const std::string name = str( ref.GetFullRef() );

                    if( seen.insert( name ).second )
                        refs.push_back( name );
                }

                std::sort( refs.begin(), refs.end(), KopenapiNaturalLess );
                item["refs"] = refs;
                parts += static_cast<int>( refs.size() );
            }

            item[str( model.GetColFieldName( col ) )] = value;
        }

        rows.push_back( std::move( item ) );
    }

    nlohmann::json answer = KopenapiPage( rows, aArgs, 1000 );
    answer["columns"] = columns;
    answer["parts"] = parts;

    const std::string path = aArgs.value( "path", std::string() );

    if( !path.empty() )
    {
        BOM_FMT_PRESET fmt = BOM_FMT_PRESET::CSV();
        const std::string format = aArgs.value( "format", std::string( "csv" ) );

        if( format == "tsv" )
            fmt = BOM_FMT_PRESET::TSV();
        else if( format == "semicolons" )
            fmt = BOM_FMT_PRESET::Semicolons();

        wxFile file;

        if( !file.Open( wxString::FromUTF8( path ), wxFile::write ) || !file.Write( model.Export( fmt ) ) )
            return KOPENAPI_RESULT::Error( 500, "cannot write " + path );

        answer["path"] = path;
    }

    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "sch_bom",
                   "Bill of materials of the open schematic, as KiCad's Symbol Fields Table / Export BOM "
                   "(kicad-cli sch export bom): rows grouped by group_by fields (default Value, Footprint, DNP), "
                   "columns = fields in order (any symbol field, e.g. LCSC or Manufacturer_Part_Number, '*' for "
                   "all; generated ones as QUANTITY / ITEM_NUMBER / DNP), or a named preset (built-in "
                   "\"Grouped By Value\"... or the project's); each row has refs (list) plus field -> text "
                   "(Reference as ranges, R1-R4); path writes the file (csv / tsv / semicolons); paginated",
                   KopenapiPagedSchema( R"json({
                        "fields":{"type":"array","items":{"type":"string"}},
                        "group_by":{"type":"array","items":{"type":"string"}},
                        "group":{"type":"boolean","default":true},
                        "sort_field":{"type":"string","default":"Reference"},
                        "sort_asc":{"type":"boolean","default":true},
                        "filter":{"type":"string","description":"KiCad's filter text (reference scope)"},
                        "exclude_dnp":{"type":"boolean","default":false},
                        "include_excluded_from_bom":{"type":"boolean","default":false},
                        "preset":{"type":"string","description":"named BOM preset instead of fields / group_by / ..."},
                        "variant":{"type":"string"},
                        "path":{"type":"string","description":"also write the BOM file here"},
                        "format":{"type":"string","enum":["csv","tsv","semicolons"],"default":"csv"}})json"_json ),
                   false, h_sch_bom );
