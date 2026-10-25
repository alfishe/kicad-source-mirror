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
#include <wildcards_and_files_ext.h>

#include <picosha2.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
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

    BOARD_COMMIT   commit( context->GetToolManager() );
    nlohmann::json items = nlohmann::json::array();

    for( FOOTPRINT* fp : fps )
    {
        commit.Modify( fp );

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
    }

    commit.Push( _( "Update footprint properties (API)" ) );

    nlohmann::json result = { { "updated", items.size() }, { "items", items } };

    if( !missing.empty() )
        result["missing"] = missing;

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


static KOPENAPI_RESULT h_pcb_footprint_lib_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                   board = context->GetBoard();
    std::vector<std::string> missing;
    std::vector<FOOTPRINT*>  fps = named( board, aArgs, false, missing );

    if( fps.empty() )
        return KOPENAPI_RESULT::Error( 404, missing.empty() ? "give ref or refs (the footprints to update)" : "no footprint " + missing.front() );

    KopenapiFootprintLibraries( board->GetProject() );

    const bool dryRun = aArgs.value( "dry_run", false );
    const bool keepTexts = aArgs.value( "keep_texts", true );
    const bool keepModels = aArgs.value( "keep_3d_models", false );

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

        if( keepModels )
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

        // texts keep their place, layer, look and content (the board's), fabrication attributes and
        // clearance overrides come from the library, as "Update Footprints from Library" offers
        board->ExchangeFootprint( fp, lib, commit, true, !keepTexts, !keepTexts, !keepTexts, !keepTexts, !keepTexts,
                                  true, true, !keepModels, false, &updated, &shifted );

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
                   "exclude_from_bom, exclude_from_pos, dnp; one undo step; answers the new values",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "type":{"type":"string","enum":["smd","through_hole","unspecified"]},
                        "lib_id":{"type":"string","description":"\"Library:Footprint\""},
                        "board_only":{"type":"boolean"},"exclude_from_bom":{"type":"boolean"},
                        "exclude_from_pos":{"type":"boolean"},"dnp":{"type":"boolean"}}})json"_json,
                   false, h_pcb_footprint_update );

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
                   "Replace placed footprints (ref or refs) by their library copies, as \"Update Footprints "
                   "from Library\": position, side, nets and fields stay, pads matched by position; texts keep "
                   "the board's place and look (keep_texts false: the library's); 3D models from the "
                   "library unless keep_3d_models (then model differences alone update nothing and are listed "
                   "as models_kept); always the library files as they are on disk now; dry_run lists what would "
                   "change; one undo step",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "keep_texts":{"type":"boolean","default":true},
                        "keep_3d_models":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_footprint_lib_update, 300 );

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

KOPENAPI_MARK_EDITING( "pcb_footprint_update", "pcb_footprint_lib_update" );
