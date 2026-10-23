/// @file methods_pcb_footprint_props.cpp
/// @brief kicadopenapi footprint properties and library parity: pcb_footprint_update,
/// pcb_footprint_lib_diff, pcb_footprint_lib_update.
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

#include <map>
#include <memory>
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


FOOTPRINT_LIBRARY_ADAPTER* loadedLibraries( BOARD* aBoard )
{
    FOOTPRINT_LIBRARY_ADAPTER* adapter = PROJECT_PCB::FootprintLibAdapter( aBoard->GetProject() );
    adapter->AsyncLoad();
    KopenapiWaitLibraries( adapter );
    return adapter;
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


nlohmann::json differences( FOOTPRINT* aFootprint, FOOTPRINT* aLib )
{
    WX_STRING_REPORTER reporter;
    nlohmann::json     out = nlohmann::json::array();

    if( !aFootprint->FootprintNeedsUpdate( aLib, BOARD_ITEM::COMPARE_FLAGS::DRC, &reporter ) )
        return out;

    wxString text = reporter.GetMessages();
    text.Replace( wxS( "\r" ), wxEmptyString );

    for( wxString line : wxSplit( text, '\n', '\0' ) )
    {
        if( !line.Trim().IsEmpty() )
            out.push_back( str( line ) );
    }

    if( out.empty() )
        out.push_back( "differs" );

    return out;
}

} // namespace


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
    FOOTPRINT_LIBRARY_ADAPTER* adapter = loadedLibraries( board );
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

    loadedLibraries( board );

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
                   "library_missing, no_library_name) and the differences KiCad finds (pads, graphics, "
                   "attributes, 3D models); default every footprint, only those not the same",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "include_same":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_footprint_lib_diff, 300 );

KOPENAPI_REGISTER( "pcb_footprint_lib_update",
                   "Replace placed footprints (ref or refs) by their library copies, as \"Update Footprints "
                   "from Library\": position, side, nets and fields stay, pads matched by position; texts keep "
                   "the board's place and look (keep_texts false: the library's); 3D models from the "
                   "library unless keep_3d_models; dry_run lists what would change; one undo step",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}},
                        "keep_texts":{"type":"boolean","default":true},
                        "keep_3d_models":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_footprint_lib_update, 300 );

KOPENAPI_MARK_EDITING( "pcb_footprint_update", "pcb_footprint_lib_update" );
