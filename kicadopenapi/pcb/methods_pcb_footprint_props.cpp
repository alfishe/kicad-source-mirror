/// @file methods_pcb_footprint_props.cpp
/// @brief kicadopenapi footprint properties and library parity: pcb_footprint_update,
/// pcb_footprint_lib_diff, pcb_footprint_lib_update, pcb_parts_check (placed footprint <->
/// library <-> 3D model files and their metadata).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <footprint_library_adapter.h>
#include <kicadopenapi_keepalive.h>
#include <lib_id.h>
#include <netlist_reader/pcb_netlist_utils.h>
#include <project_pcb.h>
#include <reporter.h>
#include <tool/tool_manager.h>
#include <3d_cache/3d_cache.h>
#include <filename_resolver.h>
#include <kiplatform/io.h>
#include <kicadopenapi_util.h>
#include <libraries/library_manager.h>
#include <pad.h>
#include <geometry/shape.h>
#include <kiway.h>
#include <frame_type.h>
#include <pcb_edit_frame.h>
#include <eda_group.h>
#include <pcb_field.h>
#include <pcb_track.h>
#include <project/project_file.h>
#include <string_utils.h>
#include <template_fieldnames.h>
#include <kicadopenapi_registry.h>
#include <wildcards_and_files_ext.h>

#include <picosha2.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


const std::map<std::string, FOOTPRINT_TYPE>& footprintTypes()
{
    static const std::map<std::string, FOOTPRINT_TYPE> types = { { "smd", FOOTPRINT_TYPE::SMD },
                                                                 { "through_hole", FOOTPRINT_TYPE::THROUGH_HOLE },
                                                                 { "unspecified", FOOTPRINT_TYPE::UNSPECIFIED } };
    return types;
}


std::string typeName( FOOTPRINT_TYPE aType )
{
    for( const auto& [name, type] : footprintTypes() )
    {
        if( type == aType )
            return name;
    }

    return "unspecified";
}


// footprints named by ref / refs; every footprint when neither is given and aAllByDefault
std::vector<FOOTPRINT*> named( BOARD* aBoard, const nlohmann::json& aArgs, bool aAllByDefault, std::vector<std::string>& aMissing )
{
    std::vector<std::string> refs;

    if( aArgs.contains( "ref" ) && aArgs["ref"].is_string() )
        refs.push_back( aArgs["ref"].get<std::string>() );

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
    {
        if( r.is_string() )
            refs.push_back( r.get<std::string>() );
    }

    std::vector<FOOTPRINT*> out;

    if( refs.empty() )
    {
        if( aAllByDefault )
            out.assign( aBoard->Footprints().begin(), aBoard->Footprints().end() );

        return out;
    }

    std::map<std::string, FOOTPRINT*> byRef;

    for( FOOTPRINT* fp : aBoard->Footprints() )
        byRef.emplace( str( fp->GetReference() ), fp );

    for( const std::string& ref : refs )
    {
        if( auto it = byRef.find( ref ); it != byRef.end() )
            out.push_back( it->second );
        else
            aMissing.push_back( ref );
    }

    return out;
}


// the library copy of aFootprint's lib id; aStatus says why there is none
std::shared_ptr<FOOTPRINT> libraryCopy( FOOTPRINT_LIBRARY_ADAPTER* aAdapter, const LIB_ID& aId, std::string& aStatus,
                                        std::map<LIB_ID, std::shared_ptr<FOOTPRINT>>& aCache )
{
    const wxString lib = aId.GetLibNickname();

    if( lib.IsEmpty() )
    {
        aStatus = "no_library_name";
        return nullptr;
    }

    if( !aAdapter->GetRow( lib ) || !aAdapter->HasLibrary( lib, true ) || !aAdapter->IsLibraryLoaded( lib ) )
    {
        aStatus = "library_missing";
        return nullptr;
    }

    if( auto it = aCache.find( aId ); it != aCache.end() )
        return it->second;

    std::shared_ptr<FOOTPRINT> fp;

    try
    {
        fp.reset( aAdapter->LoadFootprint( lib, aId.GetLibItemName(), true ) );
    }
    catch( const IO_ERROR& )
    {
    }

    if( !fp )
        aStatus = "not_in_library";

    aCache[aId] = fp;
    return fp;
}


nlohmann::json reportLines( wxString aText )
{
    nlohmann::json out = nlohmann::json::array();
    aText.Replace( wxS( "\r" ), wxEmptyString );

    for( wxString line : wxSplit( aText, '\n', '\0' ) )
    {
        if( !line.Trim().IsEmpty() )
            out.push_back( str( line ) );
    }

    return out;
}


// KiCad's comparison (pads, graphics, attributes, rule areas, 3D models)
nlohmann::json differences( FOOTPRINT* aFootprint, FOOTPRINT* aLib )
{
    WX_STRING_REPORTER reporter;

    if( !aFootprint->FootprintNeedsUpdate( aLib, BOARD_ITEM::COMPARE_FLAGS::DRC, &reporter ) )
        return nlohmann::json::array();

    nlohmann::json out = reportLines( reporter.GetMessages() );

    if( out.empty() )
        out.push_back( "differs" );

    return out;
}


// the 3D model lines of differences()
nlohmann::json modelDifferences( const FOOTPRINT* aFootprint, const FOOTPRINT* aLib )
{
    WX_STRING_REPORTER reporter;

    if( !aFootprint->ModelsNeedUpdate( aLib, &reporter ) )
        return nlohmann::json::array();

    return reportLines( reporter.GetMessages() );
}


const wxString EMBED_URI = wxString( FILEEXT::KiCadUriPrefix ) + wxS( "://" );


/// sha256 of a file, cached per path and file timestamp
std::string fileSha256( const std::filesystem::path& aFile )
{
    static std::mutex                                             mutex;
    static std::map<std::string, std::pair<long long, std::string>> cache;

    const long long stamp = KIPLATFORM::IO::TimestampFile( wxString::FromUTF8( aFile.string() ) );

    {
        std::lock_guard<std::mutex> lock( mutex );

        if( auto it = cache.find( aFile.string() ); it != cache.end() && it->second.first == stamp )
            return it->second.second;
    }

    std::ifstream in( aFile, std::ios::binary );

    if( !in )
        return {};

    std::vector<unsigned char> hash( picosha2::k_digest_size );
    picosha2::hash256( std::istreambuf_iterator<char>( in ), std::istreambuf_iterator<char>(), hash.begin(),
                       hash.end() );
    const std::string hex = picosha2::bytes_to_hex_string( hash.begin(), hash.end() );

    std::lock_guard<std::mutex> lock( mutex );
    cache[aFile.string()] = { stamp, hex };
    return hex;
}


struct MODEL_META
{
    bool        present = false;
    std::string sha256;
    int         leads = -1;     ///< solids of the part named "lead"
};


/// <model>.meta.json beside the model file, cached per path and file timestamp
MODEL_META modelMeta( const std::filesystem::path& aModel )
{
    static std::mutex                                            mutex;
    static std::map<std::string, std::pair<long long, MODEL_META>> cache;

    std::filesystem::path metaFile = aModel;
    metaFile.replace_extension( ".meta.json" );

    const long long stamp = KIPLATFORM::IO::TimestampFile( wxString::FromUTF8( metaFile.string() ) );

    if( stamp == 0 )
        return {};

    {
        std::lock_guard<std::mutex> lock( mutex );

        if( auto it = cache.find( metaFile.string() ); it != cache.end() && it->second.first == stamp )
            return it->second.second;
    }

    MODEL_META     meta;
    std::ifstream  in( metaFile );
    nlohmann::json json = nlohmann::json::parse( in, nullptr, false );

    if( json.is_object() )
    {
        meta.present = true;
        meta.sha256 = json.value( "model_sha256", std::string() );

        const nlohmann::json lead = json.value( "/geometry/parts/lead"_json_pointer, nlohmann::json() );

        if( lead.is_object() && lead.value( "solids", -1 ) > 0 )
            meta.leads = lead.value( "solids", -1 );
    }

    std::lock_guard<std::mutex> lock( mutex );
    cache[metaFile.string()] = { stamp, meta };
    return meta;
}


/// @brief The Change / Update Footprints dialog's options; aChange: the change dialog's defaults,
/// else the update dialog's
struct EXCHANGE_OPTIONS
{
    bool matchPads = true;
    bool removeExtraTexts = false;
    bool textLayers;
    bool textEffects;
    bool textPositions;
    bool textContent;
    bool fabAttrs;
    bool clearances = true;
    bool models = true;
    bool transform;

    EXCHANGE_OPTIONS( const nlohmann::json& aArgs, bool aChange ) :
            matchPads( aArgs.value( "match_pad_positions", true ) ),
            removeExtraTexts( aArgs.value( "remove_extra_texts", false ) ),
            textLayers( aArgs.value( "reset_text_layers", aChange ) ),
            textEffects( aArgs.value( "reset_text_effects", aChange ) ),
            textPositions( aArgs.value( "reset_text_positions", aChange ) ),
            textContent( aArgs.value( "reset_text_content", aChange ) ),
            fabAttrs( aArgs.value( "reset_fabrication_attrs", aChange ) ),
            clearances( aArgs.value( "reset_clearance_overrides", true ) ),
            models( aArgs.value( "reset_3d_models", true ) ),
            transform( aArgs.value( "reset_transform", aChange ) )
    {
    }

    void Exchange( BOARD* aBoard, FOOTPRINT* aOld, FOOTPRINT* aNew, BOARD_COMMIT& aCommit, bool* aUpdated,
                   bool* aShifted ) const
    {
        aBoard->ExchangeFootprint( aOld, aNew, aCommit, matchPads, removeExtraTexts, textLayers, textEffects,
                                   textPositions, textContent, fabAttrs, clearances, models, transform, aUpdated,
                                   aShifted );
    }
};


const char* const EXCHANGE_SCHEMA_PROPERTIES = R"json(
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "all":{"type":"boolean","default":false},
                        "match_ref":{"type":"string","description":"wildcards * ?"},
                        "match_value":{"type":"string","description":"wildcards * ?"},
                        "match_lib_id":{"type":"string","description":"\"Library:Footprint\""},
                        "match_pad_positions":{"type":"boolean","default":true},
                        "remove_extra_texts":{"type":"boolean","default":false},
                        "reset_text_layers":{"type":"boolean"},"reset_text_effects":{"type":"boolean"},
                        "reset_text_positions":{"type":"boolean"},"reset_text_content":{"type":"boolean"},
                        "reset_fabrication_attrs":{"type":"boolean"},
                        "reset_clearance_overrides":{"type":"boolean","default":true},
                        "reset_3d_models":{"type":"boolean","default":true},
                        "reset_transform":{"type":"boolean"},
                        "dry_run":{"type":"boolean","default":false})json";


/// @brief The footprints the dialog's match mode picks: ref / refs, all, or match_ref / match_value
/// (wildcards, case-insensitive) / match_lib_id; in board order. aError when the selector is bad
std::vector<FOOTPRINT*> matching( BOARD* aBoard, const nlohmann::json& aArgs, std::vector<std::string>& aMissing,
                                  std::string& aError )
{
    const int modes = ( aArgs.contains( "ref" ) || aArgs.contains( "refs" ) ? 1 : 0 )
                      + ( aArgs.value( "all", false ) ? 1 : 0 ) + ( aArgs.contains( "match_ref" ) ? 1 : 0 )
                      + ( aArgs.contains( "match_value" ) ? 1 : 0 ) + ( aArgs.contains( "match_lib_id" ) ? 1 : 0 );

    if( modes != 1 )
    {
        aError = "one of ref / refs, all, match_ref, match_value, match_lib_id";
        return {};
    }

    if( aArgs.contains( "ref" ) || aArgs.contains( "refs" ) )
        return named( aBoard, aArgs, false, aMissing );

    std::vector<FOOTPRINT*> out;
    LIB_ID                  id;

    if( aArgs.contains( "match_lib_id" )
        && ( !aArgs["match_lib_id"].is_string()
             || id.Parse( wxString::FromUTF8( aArgs["match_lib_id"].get<std::string>() ) ) >= 0 || !id.IsValid() ) )
    {
        aError = "match_lib_id: \"Library:Footprint\"";
        return {};
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        bool match = true;

        if( aArgs.contains( "match_ref" ) )
            match = WildCompareString( wxString::FromUTF8( aArgs.value( "match_ref", std::string() ) ),
                                       fp->GetReference(), false );
        else if( aArgs.contains( "match_value" ) )
            match = WildCompareString( wxString::FromUTF8( aArgs.value( "match_value", std::string() ) ),
                                       fp->GetValue(), false );
        else if( aArgs.contains( "match_lib_id" ) )
            match = fp->GetFPID() == id;

        if( match )
            out.push_back( fp );
    }

    if( out.empty() )
        aError = "no footprint matches";

    return out;
}

} // namespace


nlohmann::json KopenapiModelFindings( BOARD* aBoard, const FOOTPRINT* aFootprint )
{
    namespace fs = std::filesystem;

    nlohmann::json out = nlohmann::json::array();
    PROJECT*       project = aBoard->GetProject();
    S3D_CACHE*     cache = project ? PROJECT_PCB::Get3DCacheManager( project ) : nullptr;
    wxString       basePath;
    bool           shown = false;

    if( project )
    {
        try
        {
            if( std::optional<LIBRARY_TABLE_ROW*> row =
                        PROJECT_PCB::FootprintLibAdapter( project )->GetRow( aFootprint->GetFPID().GetLibNickname() ) )
            {
                basePath = LIBRARY_MANAGER::GetFullURI( *row, true );
            }
        }
        catch( ... )
        {
        }
    }

    auto finding = [&]( const std::string& aKind, const std::string& aSeverity, int aModel, const wxString& aFile,
                        const std::string& aMessage )
    {
        out.push_back( { { "kind", aKind }, { "severity", aSeverity }, { "model", aModel }, { "file", str( aFile ) },
                         { "message", aMessage } } );
    };

    std::set<wxString> numberedPads;

    for( const PAD* pad : aFootprint->Pads() )
    {
        if( !pad->GetNumber().IsEmpty() )
            numberedPads.insert( pad->GetNumber() );
    }

    int index = 0;

    for( const FP_3DMODEL& m : aFootprint->Models() )
    {
        index++;

        if( m.m_Filename.IsEmpty() )
            continue;

        shown |= m.m_Show;

        if( m.m_Filename.StartsWith( EMBED_URI ) )
        {
            const wxString name = m.m_Filename.Mid( EMBED_URI.length() );
            const bool     carried = ( aFootprint->GetEmbeddedFiles() && aFootprint->GetEmbeddedFiles()->GetEmbeddedFile( name ) )
                                 || ( aBoard->GetEmbeddedFiles() && aBoard->GetEmbeddedFiles()->GetEmbeddedFile( name ) );

            if( !carried )
                finding( "model_missing", "error", index, m.m_Filename, "embedded model file is not in the footprint or the board" );

            continue;
        }

        const wxString resolved = cache ? cache->GetResolver()->ResolvePath( m.m_Filename, basePath, {} ) : wxString();

        if( resolved.IsEmpty() || !wxFileName::FileExists( resolved ) )
        {
            std::string message = "model file not found (KiCad's 3D search paths)";

            if( m.m_Filename.Lower().EndsWith( wxS( ".wrl" ) ) && cache )
            {
                wxString step = m.m_Filename.BeforeLast( '.' ) + wxS( ".step" );

                if( !cache->GetResolver()->ResolvePath( step, basePath, {} ).IsEmpty() )
                    message += "; a .step of the same name exists (the GUI offers to migrate, kicad-cli substitutes it)";
            }

            finding( "model_missing", m.m_Show ? "error" : "warning", index, m.m_Filename, message );
            continue;
        }

        const fs::path   file( str( resolved ) );
        const MODEL_META meta = modelMeta( file );

        if( !meta.present )
            continue;

        // the hash is read only when the model is newer than its metadata
        std::error_code ec;
        fs::path        metaFile = file;
        metaFile.replace_extension( ".meta.json" );

        const auto modelTime = fs::last_write_time( file, ec );
        const auto metaTime = ec ? modelTime : fs::last_write_time( metaFile, ec );

        if( !meta.sha256.empty() && !ec && modelTime > metaTime && fileSha256( file ) != meta.sha256 )
        {
            finding( "model_meta_stale", "warning", index, m.m_Filename,
                     "the model changed after its metadata (" + metaFile.filename().string()
                             + " model_sha256 differs): seating, heights and colours may be wrong" );
        }

        if( meta.leads > 0 && !numberedPads.empty() && meta.leads > (int) numberedPads.size() )
        {
            finding( "model_lead_count", "warning", index, m.m_Filename,
                     "the model has " + std::to_string( meta.leads ) + " leads, the footprint "
                             + std::to_string( numberedPads.size() ) + " numbered pads: model of another package?" );
        }
    }

    if( !shown && !aFootprint->IsBoardOnly() && !numberedPads.empty() )
        finding( "no_model", "info", 0, wxEmptyString, "no shown 3D model" );

    return out;
}


FOOTPRINT_LIBRARY_ADAPTER* KopenapiFootprintLibraries( PROJECT* aProject )
{
    FOOTPRINT_LIBRARY_ADAPTER* adapter = PROJECT_PCB::FootprintLibAdapter( aProject );
    adapter->AsyncLoad();
    KopenapiWaitLibraries( adapter );
    return adapter;
}


static KOPENAPI_RESULT h_pcb_footprint_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                   board = context->GetBoard();
    std::vector<std::string> missing;
    std::vector<FOOTPRINT*>  fps = named( board, aArgs, false, missing );

    if( fps.empty() )
        return KOPENAPI_RESULT::Error( 404, missing.empty() ? "give ref or refs" : "no footprint " + missing.front() );

    if( aArgs.contains( "type" ) && ( !aArgs["type"].is_string() || !footprintTypes().count( aArgs["type"].get<std::string>() ) ) )
        return KOPENAPI_RESULT::Error( 400, "type: smd, through_hole or unspecified" );

    LIB_ID libId;

    if( aArgs.contains( "lib_id" ) )
    {
        if( !aArgs["lib_id"].is_string() || libId.Parse( wxString::FromUTF8( aArgs["lib_id"].get<std::string>() ) ) >= 0
            || libId.GetLibNickname().empty() || libId.GetLibItemName().empty() )
        {
            return KOPENAPI_RESULT::Error( 400, "lib_id: \"Library:Footprint\"" );
        }
    }

    for( const char* key : { "board_only", "exclude_from_bom", "exclude_from_pos", "dnp" } )
    {
        if( aArgs.contains( key ) && !aArgs[key].is_boolean() )
            return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": true or false" );
    }

    // the field edits of Footprint Properties / Bulk Edit Footprint Fields: name -> text, null removes
    std::map<wxString, std::optional<wxString>> fieldEdits;
    std::map<wxString, bool>                    showEdits;
    const wxString refName = GetDefaultFieldName( FIELD_T::REFERENCE, UNTRANSLATED );
    const wxString valueName = GetDefaultFieldName( FIELD_T::VALUE, UNTRANSLATED );
    const wxString fpName = GetDefaultFieldName( FIELD_T::FOOTPRINT, UNTRANSLATED );

    if( aArgs.contains( "fields" ) )
    {
        if( !aArgs["fields"].is_object() )
            return KOPENAPI_RESULT::Error( 400, "fields: {name: text, or null to remove}" );

        for( const auto& [key, val] : aArgs["fields"].items() )
        {
            const wxString name = wxString::FromUTF8( key );

            if( name.IsEmpty() || ( !val.is_string() && !val.is_null() ) )
                return KOPENAPI_RESULT::Error( 400, "fields: {name: text, or null to remove}" );

            if( name.CmpNoCase( fpName ) == 0 )
                return KOPENAPI_RESULT::Error( 400, "fields: Footprint is the library link: lib_id" );

            if( name.CmpNoCase( refName ) == 0 )
                return KOPENAPI_RESULT::Error( 400, "fields: the reference is renamed with reference" );

            if( val.is_null() )
                fieldEdits[name] = std::nullopt;
            else
                fieldEdits[name] = wxString::FromUTF8( val.get<std::string>() );
        }
    }

    if( aArgs.contains( "value" ) )
    {
        if( !aArgs["value"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "value: text" );

        fieldEdits[valueName] = wxString::FromUTF8( aArgs["value"].get<std::string>() );
    }

    for( const auto& [name, text] : fieldEdits )
    {
        if( !text && ( name.CmpNoCase( valueName ) == 0
                       || name.CmpNoCase( GetDefaultFieldName( FIELD_T::DATASHEET, UNTRANSLATED ) ) == 0
                       || name.CmpNoCase( GetDefaultFieldName( FIELD_T::DESCRIPTION, UNTRANSLATED ) ) == 0 ) )
        {
            return KOPENAPI_RESULT::Error( 400, "fields: " + str( name ) + " is mandatory and cannot be removed" );
        }
    }

    if( aArgs.contains( "show_fields" ) )
    {
        if( !aArgs["show_fields"].is_object() )
            return KOPENAPI_RESULT::Error( 400, "show_fields: {name: true or false}" );

        for( const auto& [key, val] : aArgs["show_fields"].items() )
        {
            if( !val.is_boolean() )
                return KOPENAPI_RESULT::Error( 400, "show_fields: {name: true or false}" );

            showEdits[wxString::FromUTF8( key )] = val.get<bool>();
        }
    }

    wxString newRef;

    if( aArgs.contains( "reference" ) )
    {
        if( !aArgs["reference"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "reference: the new reference designator" );

        newRef = wxString::FromUTF8( aArgs["reference"].get<std::string>() );

        if( fps.size() != 1 )
            return KOPENAPI_RESULT::Error( 400, "reference: renames one footprint (ref), not several" );

        if( newRef.IsEmpty() || newRef.find_first_of( wxS( " \t\r\n" ) ) != wxString::npos )
            return KOPENAPI_RESULT::Error( 400, "reference: not empty, no whitespace" );

        if( newRef != fps.front()->GetReference() && !aArgs.value( "allow_duplicate", false ) )
        {
            for( FOOTPRINT* other : board->Footprints() )
            {
                if( other != fps.front() && other->GetReference() == newRef )
                {
                    KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "reference " + str( newRef ) + " is taken "
                                                                     "(allow_duplicate: true keeps both)" );
                    r.body["taken_by"] = str( other->m_Uuid.AsString() );
                    return r;
                }
            }
        }
    }

    // field values go to a variant as in the editor: the board's current one unless variant is given
    wxString variant = board->GetCurrentVariant();

    if( aArgs.contains( "variant" ) )
    {
        if( !aArgs["variant"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "variant: a variant name, \"\" for the base values" );

        variant = wxString::FromUTF8( aArgs["variant"].get<std::string>() );

        if( !variant.IsEmpty() && variant.CmpNoCase( GetDefaultVariantName() ) != 0 && !board->HasVariant( variant ) )
            return KOPENAPI_RESULT::Error( 404, "variant: no variant " + str( variant ) );
    }

    const bool baseValues = variant.IsEmpty() || variant.CmpNoCase( GetDefaultVariantName() ) == 0;

    if( !baseValues )
    {
        for( const auto& [name, text] : fieldEdits )
        {
            if( !text )
                return KOPENAPI_RESULT::Error( 400, "fields: a field is removed from the base values (variant \"\")" );
        }
    }

    TEMPLATES* templates = board->GetProject() ? &board->GetProject()->GetProjectFile().m_TemplateFieldNames : nullptr;

    BOARD_COMMIT   commit( context->GetToolManager() );
    nlohmann::json items = nlohmann::json::array();
    nlohmann::json warnings = nlohmann::json::array();
    std::string    oldRef;
    bool           renamed = false, valueChanged = false, fieldsChanged = false;

    for( FOOTPRINT* fp : fps )
    {
        commit.Modify( fp );

        if( !newRef.IsEmpty() && newRef != fp->GetReference() )
        {
            oldRef = str( fp->GetReference() );
            fp->SetReference( newRef );
            renamed = true;
        }

        for( const auto& [name, text] : fieldEdits )
        {
            PCB_FIELD* field = nullptr;

            for( PCB_FIELD* candidate : fp->GetFields() )
            {
                if( candidate->GetName().CmpNoCase( name ) == 0 )
                    field = candidate;
            }

            if( !text )
            {
                if( field && !field->IsMandatory() )
                {
                    if( EDA_GROUP* parentGroup = field->GetParentGroup() )
                        parentGroup->RemoveItem( field );

                    fp->Remove( field );
                    delete field;
                    fieldsChanged = true;
                }
                else if( !field )
                {
                    warnings.push_back( str( fp->GetReference() ) + " has no field " + str( name ) );
                }

                continue;
            }

            if( !field )
            {
                field = new PCB_FIELD( fp, FIELD_T::USER, name );
                field->SetLayer( fp->GetLayer() == F_Cu ? F_Fab : B_Fab );
                field->SetFPRelativePosition( { 0, 0 } );
                field->StyleFromSettings( board->GetDesignSettings(), true );

                const TEMPLATE_FIELDNAME* tmpl = templates ? templates->GetFieldName( name ) : nullptr;
                field->SetVisible( tmpl ? tmpl->m_Visible : false );
                fp->Add( field );
            }

            if( field->IsPrivate() )
                field->SetPrivate( false );

            const wxString newText = board->ConvertCrossReferencesToKIIDs( *text );

            if( fp->GetFieldValueForVariant( variant, field->GetName() ) != newText )
            {
                if( baseValues )
                    field->SetText( newText );
                else if( FOOTPRINT_VARIANT* v = fp->AddVariant( variant ) )
                    v->SetFieldValue( field->GetName(), newText );

                ( name.CmpNoCase( valueName ) == 0 ? valueChanged : fieldsChanged ) = true;
            }
        }

        for( const auto& [name, show] : showEdits )
        {
            PCB_FIELD* field = nullptr;

            for( PCB_FIELD* candidate : fp->GetFields() )
            {
                if( candidate->GetName().CmpNoCase( name ) == 0 )
                    field = candidate;
            }

            if( field )
                field->SetVisible( show );
            else
                warnings.push_back( str( fp->GetReference() ) + " has no field " + str( name ) );
        }

        if( aArgs.contains( "type" ) )
            fp->SetFootprintType( footprintTypes().at( aArgs["type"].get<std::string>() ) );

        if( aArgs.contains( "lib_id" ) )
            fp->SetFPID( libId );

        if( aArgs.contains( "board_only" ) )
            fp->SetBoardOnly( aArgs["board_only"].get<bool>() );

        if( aArgs.contains( "exclude_from_bom" ) )
            fp->SetExcludedFromBOM( aArgs["exclude_from_bom"].get<bool>() );

        if( aArgs.contains( "exclude_from_pos" ) )
            fp->SetExcludedFromPosFiles( aArgs["exclude_from_pos"].get<bool>() );

        if( aArgs.contains( "dnp" ) )
            fp->SetDNP( aArgs["dnp"].get<bool>() );

        items.push_back( { { "ref", str( fp->GetReference() ) },
                           { "lib_id", str( fp->GetFPID().Format() ) },
                           { "type", typeName( fp->GetFootprintType() ) },
                           { "board_only", fp->IsBoardOnly() },
                           { "exclude_from_bom", fp->IsExcludedFromBOM() },
                           { "exclude_from_pos", fp->IsExcludedFromPosFiles() },
                           { "dnp", fp->IsDNP() } } );

        if( !fieldEdits.empty() || !showEdits.empty() || !newRef.IsEmpty() )
        {
            nlohmann::json fields = nlohmann::json::object();

            for( const PCB_FIELD* field : fp->GetFields() )
            {
                fields[str( field->GetName() )] = { { "text", str( board->ConvertKIIDsToCrossReferences(
                                                                    fp->GetFieldValueForVariant( variant, field->GetName() ) ) ) },
                                                    { "visible", field->IsVisible() } };
            }

            items.back()["value"] = str( fp->GetFieldValueForVariant( variant, valueName ) );
            items.back()["fields"] = fields;

            if( !oldRef.empty() && fps.size() == 1 )
                items.back()["old_ref"] = oldRef;
        }
    }

    commit.Push( _( "Update footprint properties (API)" ) );
    context->SetContentModified();

    nlohmann::json result = { { "updated", items.size() }, { "items", items } };

    if( !baseValues )
        result["variant"] = str( variant );

    if( !missing.empty() )
        result["missing"] = missing;

    if( !warnings.empty() )
        result["warnings"] = warnings;

    // references, values and fields reach the schematic through Update Schematic from PCB
    const nlohmann::json backAnnotate = aArgs.value( "back_annotate", nlohmann::json( "auto" ) );

    const bool schematicSide = renamed || valueChanged || fieldsChanged;

    if( schematicSide && baseValues && backAnnotate != false )
    {
        std::optional<KOPENAPI_METHOD> update = KOPENAPI_REGISTRY::Get().Find( "design_update_schematic" );
        KOPENAPI_RESULT                sch = update ? update->handler( aCtx, { { "references", renamed },
                                                                              { "footprints", false },
                                                                              { "values", valueChanged },
                                                                              { "net_names", false },
                                                                              { "attributes", false },
                                                                              { "other_fields", fieldsChanged } } )
                                                    : KOPENAPI_RESULT::Error( 503, "design_update_schematic is not available" );

        if( sch.status == 200 )
        {
            result["schematic"] = { { "changes", sch.body.value( "changes", 0 ) },
                                    { "errors", sch.body.value( "errors", 0 ) } };

            if( sch.body.value( "errors", 0 ) > 0 )
                result["schematic"]["messages"] = sch.body["messages"];
        }
        else
        {
            result["schematic"] = { { "skipped", sch.body.value( "error", nlohmann::json::object() )
                                                          .value( "message", std::string( "no schematic open" ) ) } };
        }
    }

    if( schematicSide && baseValues && !result.contains( "schematic" ) )
        result["next"] = "the schematic keeps the old text: design_update_schematic (KiCad's Update Schematic from PCB)";
    else if( result.contains( "schematic" ) && result["schematic"].contains( "skipped" ) )
        result["next"] = "the schematic keeps the old text: open it (sch_open), then design_update_schematic";

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_footprint_delete( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    std::vector<std::string> refs;

    if( aArgs.contains( "ref" ) && aArgs["ref"].is_string() )
        refs.push_back( aArgs["ref"].get<std::string>() );

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
    {
        if( r.is_string() )
            refs.push_back( r.get<std::string>() );
    }

    if( refs.empty() )
        return KOPENAPI_RESULT::Error( 400, "ref or refs: the footprints to delete (pcb_footprint_list)" );

    BOARD*                   board = context->GetBoard();
    std::set<std::string>    wanted( refs.begin(), refs.end() );
    std::set<std::string>    found;
    std::vector<FOOTPRINT*>  fps;

    // every footprint with that reference: a duplicated one goes as a whole
    for( FOOTPRINT* fp : board->Footprints() )
    {
        const std::string ref = str( fp->GetReference() );

        if( wanted.count( ref ) )
        {
            fps.push_back( fp );
            found.insert( ref );
        }
    }

    nlohmann::json missing = nlohmann::json::array();

    for( const std::string& ref : wanted )
    {
        if( !found.count( ref ) )
            missing.push_back( ref );
    }

    if( fps.empty() )
        return KOPENAPI_RESULT::Error( 404, "no footprint " + missing.front().get<std::string>() );

    if( !aArgs.value( "override_locks", false ) )
    {
        nlohmann::json locked = nlohmann::json::array();

        for( FOOTPRINT* fp : fps )
        {
            if( fp->IsLocked() )
                locked.push_back( str( fp->GetReference() ) );
        }

        if( !locked.empty() )
        {
            KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "locked footprints are in the request (pcb_unlock, "
                                                                   "or override_locks: true)" );
            refused.body["locked"] = locked;
            return refused;
        }
    }

    // copper ending on a pad of its own net: it stays unless rip_attached
    std::vector<PAD*> pads;

    for( FOOTPRINT* fp : fps )
        pads.insert( pads.end(), fp->Pads().begin(), fp->Pads().end() );

    const bool               dryRun = aArgs.value( "dry_run", false );
    const bool               rip = aArgs.value( "rip_attached", false );
    std::vector<PCB_TRACK*>  attached;

    for( PCB_TRACK* track : board->Tracks() )
    {
        for( PAD* pad : pads )
        {
            if( pad->GetNetCode() <= 0 || pad->GetNetCode() != track->GetNetCode() )
                continue;

            bool touches = false;

            for( PCB_LAYER_ID layer : track->GetLayerSet().CuStack() )
            {
                if( !pad->IsOnLayer( layer ) )
                    continue;

                const std::shared_ptr<SHAPE> shape = pad->GetEffectiveShape( layer );

                if( track->Type() == PCB_VIA_T )
                    touches = shape->Collide( track->GetPosition() );
                else
                    touches = shape->Collide( track->GetStart() ) || shape->Collide( track->GetEnd() );

                if( touches )
                    break;
            }

            if( touches )
            {
                attached.push_back( track );
                break;
            }
        }
    }

    nlohmann::json items = nlohmann::json::array();
    int            padCount = 0;

    for( FOOTPRINT* fp : fps )
    {
        std::set<std::string> nets;

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 )
                nets.insert( str( pad->GetNetname() ) );
        }

        padCount += int( fp->Pads().size() );
        items.push_back( { { "ref", str( fp->GetReference() ) },
                           { "uuid", str( fp->m_Uuid.AsString() ) },
                           { "lib_id", str( fp->GetFPID().Format() ) },
                           { "pads", fp->Pads().size() },
                           { "nets", nets } } );
    }

    nlohmann::json attachedIds = nlohmann::json::array();

    for( PCB_TRACK* track : attached )
        attachedIds.push_back( str( track->m_Uuid.AsString() ) );

    if( !dryRun )
    {
        BOARD_COMMIT commit( context->GetToolManager() );

        for( FOOTPRINT* fp : fps )
            commit.Remove( fp );

        if( rip )
        {
            for( PCB_TRACK* track : attached )
                commit.Remove( track );
        }

        commit.Push( _( "Delete footprints (API)" ) );

        if( !aCtx.headless )
        {
            if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway ? aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) : nullptr ) )
            {
                frame->GetCanvas()->Refresh();
                KopenapiRefresh3D( frame );
            }
        }
    }

    nlohmann::json result = { { "deleted", dryRun ? 0 : int( fps.size() ) },
                              { "dry_run", dryRun },
                              { "footprints", fps.size() },
                              { "pads", padCount },
                              { "items", items },
                              { "attached_copper", attachedIds },
                              { "attached_removed", !dryRun && rip ? int( attached.size() ) : 0 },
                              { "missing", missing } };

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_footprint_lib_diff( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                     board = context->GetBoard();
    std::vector<std::string>   missing;
    std::vector<FOOTPRINT*>    fps = named( board, aArgs, true, missing );
    FOOTPRINT_LIBRARY_ADAPTER* adapter = KopenapiFootprintLibraries( board->GetProject() );
    const bool                 all = aArgs.value( "include_same", false );

    std::map<LIB_ID, std::shared_ptr<FOOTPRINT>> cache;
    std::map<std::string, int>                   counts;
    nlohmann::json                               items = nlohmann::json::array();

    for( FOOTPRINT* fp : fps )
    {
        std::string                status;
        std::shared_ptr<FOOTPRINT> lib = libraryCopy( adapter, fp->GetFPID(), status, cache );
        nlohmann::json             diff = nlohmann::json::array();

        if( lib )
        {
            diff = differences( fp, lib.get() );
            status = diff.empty() ? "same" : "differs";
        }

        counts[status]++;

        if( status == "same" && !all )
            continue;

        nlohmann::json row = { { "ref", str( fp->GetReference() ) }, { "lib_id", str( fp->GetFPID().Format() ) },
                               { "status", status } };

        if( !diff.empty() )
            row["differences"] = diff;

        items.push_back( row );
    }

    nlohmann::json result = { { "checked", fps.size() }, { "counts", counts }, { "items", items } };

    if( !missing.empty() )
        result["missing"] = missing;

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_footprint_change( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();
    LIB_ID newId;

    if( !aArgs.contains( "lib_id" ) || !aArgs["lib_id"].is_string()
        || newId.Parse( wxString::FromUTF8( aArgs["lib_id"].get<std::string>() ) ) >= 0 || !newId.IsValid() )
    {
        return KOPENAPI_RESULT::Error( 400, "lib_id: the new footprint, \"Library:Footprint\"" );
    }

    std::vector<std::string> missing;
    std::string              error;
    std::vector<FOOTPRINT*>  fps = matching( board, aArgs, missing, error );

    if( !error.empty() )
        return KOPENAPI_RESULT::Error( error == "no footprint matches" ? 404 : 400, error );

    if( fps.empty() )
        return KOPENAPI_RESULT::Error( 404, "no footprint " + missing.front() );

    KopenapiFootprintLibraries( board->GetProject() );

    std::unique_ptr<FOOTPRINT> probe( LoadFootprintFromProject( board, newId ) );

    if( !probe )
        return KOPENAPI_RESULT::Error( 404, "lib_id: no such footprint in the library tables" );

    const EXCHANGE_OPTIONS options( aArgs, true );
    const bool             dryRun = aArgs.value( "dry_run", false );
    BOARD_COMMIT           commit( context->GetToolManager() );
    nlohmann::json         items = nlohmann::json::array();

    for( FOOTPRINT* fp : fps )
    {
        nlohmann::json row = { { "ref", str( fp->GetReference() ) },
                               { "from", str( fp->GetFPID().Format() ) },
                               { "to", str( newId.Format() ) } };

        if( dryRun )
        {
            row["status"] = "would_change";
            items.push_back( row );
            continue;
        }

        FOOTPRINT* newFp = LoadFootprintFromProject( board, newId );
        bool       updated = true;
        bool       shifted = false;

        options.Exchange( board, fp, newFp, commit, &updated, &shifted );

        row["status"] = "changed";
        row["uuid"] = str( newFp->m_Uuid.AsString() );
        row["pads"] = newFp->Pads().size();

        if( shifted )
            row["shifted"] = true;

        items.push_back( row );
    }

    if( !dryRun )
        commit.Push( _( "Change Footprints" ) );

    nlohmann::json result = { { "changed", dryRun ? 0 : (int) fps.size() }, { "dry_run", dryRun }, { "items", items } };

    if( !missing.empty() )
        result["missing"] = missing;

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_footprint_lib_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                   board = context->GetBoard();
    std::vector<std::string> missing;
    std::vector<FOOTPRINT*>  fps;

    if( aArgs.value( "all", false ) || aArgs.contains( "match_ref" ) || aArgs.contains( "match_value" )
        || aArgs.contains( "match_lib_id" ) )
    {
        std::string error;
        fps = matching( board, aArgs, missing, error );

        if( !error.empty() )
            return KOPENAPI_RESULT::Error( error == "no footprint matches" ? 404 : 400, error );
    }
    else
    {
        fps = named( board, aArgs, false, missing );
    }

    if( fps.empty() )
        return KOPENAPI_RESULT::Error( 404, missing.empty() ? "give ref or refs (the footprints to update)" : "no footprint " + missing.front() );

    KopenapiFootprintLibraries( board->GetProject() );

    const bool dryRun = aArgs.value( "dry_run", false );
    const bool keepTexts = aArgs.value( "keep_texts", true );
    const bool keepModels = aArgs.value( "keep_3d_models", false );

    // keep_texts / keep_3d_models are the short forms; a reset_* flag given wins
    EXCHANGE_OPTIONS options( aArgs, false );
    auto unless = [&]( const char* aKey, bool& aFlag, bool aValue )
    {
        if( !aArgs.contains( aKey ) )
            aFlag = aValue;
    };

    unless( "reset_fabrication_attrs", options.fabAttrs, true );
    unless( "reset_3d_models", options.models, !keepModels );

    if( !keepTexts )
    {
        unless( "remove_extra_texts", options.removeExtraTexts, true );
        unless( "reset_text_layers", options.textLayers, true );
        unless( "reset_text_effects", options.textEffects, true );
        unless( "reset_text_positions", options.textPositions, true );
        unless( "reset_text_content", options.textContent, true );
    }

    BOARD_COMMIT   commit( context->GetToolManager() );
    nlohmann::json items = nlohmann::json::array();
    int            changed = 0;

    for( FOOTPRINT* fp : fps )
    {
        nlohmann::json row = { { "ref", str( fp->GetReference() ) }, { "lib_id", str( fp->GetFPID().Format() ) } };
        FOOTPRINT*     lib = LoadFootprintFromProject( board, fp->GetFPID() );

        if( !lib )
        {
            row["status"] = "not_in_library";
            items.push_back( row );
            continue;
        }

        nlohmann::json diff = differences( fp, lib );

        if( !options.models )
        {
            nlohmann::json models = modelDifferences( fp, lib );

            if( !models.empty() )
            {
                row["models_kept"] = models;
                nlohmann::json rest = nlohmann::json::array();

                for( const nlohmann::json& line : diff )
                {
                    if( std::find( models.begin(), models.end(), line ) == models.end() )
                        rest.push_back( line );
                }

                diff = rest;
            }
        }

        if( diff.empty() || dryRun )
        {
            row["status"] = diff.empty() ? "same" : "would_update";

            if( !diff.empty() )
                row["differences"] = diff;

            delete lib;
            items.push_back( row );
            continue;
        }

        bool updated = true;
        bool shifted = false;

        options.Exchange( board, fp, lib, commit, &updated, &shifted );

        row["status"] = "updated";
        row["differences"] = diff;

        if( shifted )
            row["shifted"] = true;

        changed++;
        items.push_back( row );
    }

    if( changed )
        commit.Push( _( "Update footprints from library (API)" ) );

    nlohmann::json result = { { "updated", changed }, { "dry_run", dryRun }, { "items", items } };

    if( !missing.empty() )
        result["missing"] = missing;

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_parts_check( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                     board = context->GetBoard();
    std::vector<std::string>   missing;
    std::vector<FOOTPRINT*>    fps = named( board, aArgs, true, missing );
    const bool                 checkLibrary = aArgs.value( "library", true );
    const bool                 problemsOnly = aArgs.value( "problems_only", true );
    FOOTPRINT_LIBRARY_ADAPTER* adapter = checkLibrary ? KopenapiFootprintLibraries( board->GetProject() ) : nullptr;

    std::map<LIB_ID, std::shared_ptr<FOOTPRINT>> cache;
    std::map<std::string, int>                   counts;
    std::vector<nlohmann::json>                  rows;
    int                                          problems = 0;

    for( FOOTPRINT* fp : fps )
    {
        nlohmann::json row = { { "ref", str( fp->GetReference() ) }, { "lib_id", str( fp->GetFPID().Format() ) } };
        nlohmann::json findings = nlohmann::json::array();

        if( adapter )
        {
            std::string                status;
            std::shared_ptr<FOOTPRINT> lib = libraryCopy( adapter, fp->GetFPID(), status, cache );

            if( lib )
            {
                nlohmann::json diff = differences( fp, lib.get() );

                if( !diff.empty() )
                {
                    findings.push_back( { { "kind", "library_differs" }, { "severity", "warning" },
                                          { "message", "placed footprint differs from its library copy" },
                                          { "differences", diff } } );
                }
            }
            else if( status != "no_library_name" || !fp->IsBoardOnly() )
            {
                findings.push_back( { { "kind", "library_" + status }, { "severity", "error" },
                                      { "message", status == "not_in_library" ? "footprint not in its library"
                                                   : status == "library_missing" ? "library not in the library tables"
                                                                                 : "footprint has no library name" } } );
            }
        }

        for( const nlohmann::json& f : KopenapiModelFindings( board, fp ) )
            findings.push_back( f );

        bool problem = false;

        for( const nlohmann::json& f : findings )
        {
            counts[f["kind"].get<std::string>()]++;
            problem |= f["severity"] != "info";
        }

        problems += problem ? 1 : 0;

        if( problemsOnly && !problem )
            continue;

        row["findings"] = findings;
        rows.push_back( std::move( row ) );
    }

    nlohmann::json result = KopenapiPage( rows, aArgs );
    result["checked"] = fps.size();
    result["with_problems"] = problems;
    result["counts"] = counts;

    if( !missing.empty() )
        result["missing"] = missing;

    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "pcb_footprint_update",
                   "Change a placed footprint's properties (ref or refs): type (smd / through_hole / "
                   "unspecified), lib_id (relink to another library footprint; the copper stays), board_only, "
                   "exclude_from_bom, exclude_from_pos, dnp; as Footprint Properties / Bulk Edit Footprint "
                   "Fields: reference (rename one footprint; a taken reference 409 unless allow_duplicate), "
                   "value, fields {name: text (a new field is added hidden on Fab), null removes a user field}, "
                   "show_fields {name: visible}; field values go to the board's current variant unless variant "
                   "(\"\" = base values). One undo step; answers the new values and fields. back_annotate (default "
                   "auto: a schematic is open) then runs Update Schematic from PCB with references, values and "
                   "other fields",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "type":{"type":"string","enum":["smd","through_hole","unspecified"]},
                        "lib_id":{"type":"string","description":"\"Library:Footprint\""},
                        "board_only":{"type":"boolean"},"exclude_from_bom":{"type":"boolean"},
                        "exclude_from_pos":{"type":"boolean"},"dnp":{"type":"boolean"},
                        "reference":{"type":"string","description":"new reference designator (one footprint)"},
                        "allow_duplicate":{"type":"boolean","default":false},
                        "value":{"type":"string"},
                        "fields":{"type":"object","additionalProperties":{"type":["string","null"]},
                                  "description":"name -> text; null removes a user field"},
                        "show_fields":{"type":"object","additionalProperties":{"type":"boolean"}},
                        "variant":{"type":"string","description":"default: the board's current variant; \"\" = base"},
                        "back_annotate":{"oneOf":[{"type":"boolean"},{"type":"string","enum":["auto"]}],"default":"auto"}}})json"_json,
                   false, h_pcb_footprint_update );

KOPENAPI_REGISTER( "pcb_footprint_delete",
                   "Delete placed footprints (ref or refs; every footprint with that reference) in one undo "
                   "step, as Delete in the board editor; locked ones refuse (409) unless override_locks. Tracks "
                   "and vias ending on their pads (same net) stay and are listed in attached_copper; "
                   "rip_attached: true removes them in the same step. dry_run tells what would go. Answers per "
                   "footprint ref, uuid, lib_id, pads and nets; missing refs. The schematic still has the "
                   "symbols: design_update_board puts them back unless they are removed there too",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "rip_attached":{"type":"boolean","default":false},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_footprint_delete );

KOPENAPI_REGISTER( "pcb_footprint_lib_diff",
                   "How placed footprints differ from their library copies (DRC's lib_footprint_mismatch / "
                   "lib_footprint_issues, explained): per footprint status (same, differs, not_in_library, "
                   "library_missing, no_library_name) and KiCad's differences (pads, graphics, attributes, "
                   "rule areas, 3D models: file, offset, rotation, scale, opacity, shown, embedded content), as "
                   "DRC lib_footprint_mismatch sees them; always against the library files as they are on disk now; "
                   "default every footprint, only those not the same",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "include_same":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_footprint_lib_diff, 300 );

KOPENAPI_REGISTER( "pcb_footprint_lib_update",
                   "Replace placed footprints by their library copies, as \"Update Footprints from Library\": "
                   "ref / refs, or the dialog's match modes (all, match_ref / match_value with wildcards, "
                   "match_lib_id); position, side, nets and fields stay, pads matched by position; texts keep "
                   "the board's place and look (keep_texts false: the library's); 3D models from the "
                   "library unless keep_3d_models (then model differences alone update nothing and are listed "
                   "as models_kept); fabrication attributes and clearance overrides from the library. The "
                   "dialog's reset_* / remove_extra_texts / match_pad_positions flags set each item and win "
                   "over the short forms. Always the library files as they are on disk now; dry_run lists what "
                   "would change; one undo step",
                   nlohmann::json::parse( std::string( R"json({"type":"object","properties":{
                        "keep_texts":{"type":"boolean","default":true},
                        "keep_3d_models":{"type":"boolean","default":false},)json" )
                                          + EXCHANGE_SCHEMA_PROPERTIES + "}}" ),
                   false, h_pcb_footprint_lib_update, 300 );

KOPENAPI_REGISTER( "pcb_footprint_change",
                   "Change Footprints, as the board editor's dialog: the matched footprints (ref / refs, all, "
                   "match_ref / match_value with wildcards * ?, match_lib_id) are replaced by lib_id from the "
                   "library tables in one undo step: position, side, reference, value, fields and nets stay, "
                   "pads matched by number and, with match_pad_positions, moved / rotated so the pads land "
                   "where the old ones were. The dialog's options with its defaults: text layers, sizes, "
                   "positions and content, fabrication attributes, clearance overrides, 3D models and the "
                   "transform from the new footprint (reset_* false keeps the board's); remove_extra_texts "
                   "drops texts the new one lacks. dry_run lists the matches. Answers per footprint ref, from, "
                   "to, the new uuid and pad count, shifted. The schematic keeps its footprint field until "
                   "sch_update_from_pcb",
                   nlohmann::json::parse( std::string( R"json({"type":"object","required":["lib_id"],"properties":{
                        "lib_id":{"type":"string","description":"the new footprint, \"Library:Footprint\""},)json" )
                                          + EXCHANGE_SCHEMA_PROPERTIES + "}}" ),
                   false, h_pcb_footprint_change, 300 );

KOPENAPI_REGISTER( "pcb_parts_check",
                   "Placed footprint <-> library <-> 3D model, one fast pass (default every footprint, "
                   "only those with problems): library_differs (KiCad's comparison incl. 3D models: file, "
                   "offset, rotation, scale, opacity, shown, embedded content; the library file as it is on "
                   "disk now), library_not_in_library / library_library_missing / library_no_library_name, "
                   "model_missing (not resolved through KiCad's 3D search paths, embedded file absent), "
                   "model_meta_stale (<model>.meta.json model_sha256 differs from the model), "
                   "model_lead_count (metadata leads > numbered pads), no_model (info). Symbol <-> footprint: "
                   "sch_footprint_check (library) and design_parity (placed); pcb_drc and design_parity include "
                   "this check",
                   KopenapiPagedSchema( R"json({
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "library":{"type":"boolean","default":true,"description":"compare with the library copies"},
                        "problems_only":{"type":"boolean","default":true}})json"_json ),
                   false, h_pcb_parts_check, 300 );

KOPENAPI_MARK_EDITING( "pcb_footprint_update", "pcb_footprint_lib_update", "pcb_footprint_delete",
                       "pcb_footprint_change" );
