/*
 * kicadopenapi footprint libraries: pcb_lib_list, pcb_lib_footprint_search, pcb_lib_footprint_get.
 *
 * Libraries come from the same tables KiCad uses (global fp-lib-table + the project's), via the
 * project's FOOTPRINT_LIBRARY_ADAPTER; load errors are reported, not skipped.  Search accepts a
 * symbol's footprint filters (KiCad's `ki_fp_filters` patterns) and pad count, so an agent can
 * go from a symbol card straight to the footprints that fit it.
 */
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <footprint.h>
#include <footprint_library_adapter.h>
#include <kicadopenapi_libraries.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <libraries/library_table.h>
#include <pad.h>
#include <project_pcb.h>

#include <algorithm>
#include <map>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double mm( double aIU )
{
    return std::round( pcbIUScale.IUTomm( aIU ) * 10000.0 ) / 10000.0;
}


PROJECT* libraryProject( KOPENAPI_CONTEXT& aCtx )
{
    if( std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx ) )
        return context->GetBoard()->GetProject();

    return aCtx.kiway ? &aCtx.kiway->Prj() : nullptr;
}


/**
 * The footprints of a loaded library.  FOOTPRINT_LIBRARY_ADAPTER keeps its enumerated footprints
 * in statics of pcbcommon, a static library linked into pcbnew *and* cvpcb: the adapter is one
 * object per process, created by whichever kiface asks first.  When cvpcb created it (ERC's
 * footprint checks run through cvpcb), it fills cvpcb's copy of the cache and pcbnew's
 * GetFootprints() answers nothing for every library.  Then the library is read here once
 * (names + LoadFootprint) and kept until it changes on disk.
 */
std::vector<const FOOTPRINT*> libraryFootprints( FOOTPRINT_LIBRARY_ADAPTER* aAdapter, const wxString& aNickname )
{
    std::vector<const FOOTPRINT*> result;

    for( const FOOTPRINT* fp : aAdapter->GetFootprints( aNickname, true ) )
        result.push_back( fp );

    if( !result.empty() || !aAdapter->IsLibraryLoaded( aNickname ) )
        return result;

    struct CACHED
    {
        long long                               timestamp = 0;
        std::vector<std::unique_ptr<FOOTPRINT>> footprints;
    };

    static std::map<wxString, CACHED> cache;
    const long long                   timestamp = aAdapter->GenerateTimestamp( &aNickname );
    CACHED&                           entry = cache[aNickname];

    if( entry.timestamp != timestamp || entry.footprints.empty() )
    {
        entry.timestamp = timestamp;
        entry.footprints.clear();

        for( const wxString& name : aAdapter->GetFootprintNames( aNickname, true ) )
        {
            try
            {
                if( FOOTPRINT* fp = aAdapter->LoadFootprint( aNickname, name, false ) )
                    entry.footprints.emplace_back( fp );
            }
            catch( const IO_ERROR& )
            {
                // a broken file: skipped, as the adapter's own enumeration does
            }
        }
    }

    for( const std::unique_ptr<FOOTPRINT>& fp : entry.footprints )
        result.push_back( fp.get() );

    return result;
}


FOOTPRINT_LIBRARY_ADAPTER* loadedAdapter( KOPENAPI_CONTEXT& aCtx )
{
    PROJECT* project = libraryProject( aCtx );

    if( !project )
        return nullptr;

    FOOTPRINT_LIBRARY_ADAPTER* adapter = PROJECT_PCB::FootprintLibAdapter( project );
    adapter->AsyncLoad();
    adapter->BlockUntilLoaded();
    return adapter;
}


const char* scopeName( LIBRARY_TABLE_SCOPE aScope )
{
    switch( aScope )
    {
    case LIBRARY_TABLE_SCOPE::GLOBAL:  return "global";
    case LIBRARY_TABLE_SCOPE::PROJECT: return "project";
    default:                           return "unknown";
    }
}


const char* mountType( const FOOTPRINT* aFp )
{
    if( aFp->GetAttributes() & FP_THROUGH_HOLE )
        return "tht";

    if( aFp->GetAttributes() & FP_SMD )
        return "smd";

    return "unspecified";
}


/// Pads that take a net (numbered, not NPTH): what a symbol's pins map to
int connectablePads( const FOOTPRINT* aFp )
{
    std::set<wxString> numbers;

    for( const PAD* pad : aFp->Pads() )
    {
        if( pad->GetAttribute() != PAD_ATTRIB::NPTH && !pad->GetNumber().IsEmpty() )
            numbers.insert( pad->GetNumber() );
    }

    return static_cast<int>( numbers.size() );
}


/**
 * KiCad footprint filter semantics (as the footprint chooser applies ki_fp_filters): a pattern
 * with ':' matches LIB:NAME, otherwise the name; '*' and '?' wildcards; case-insensitive.
 */
bool matchesFilters( const std::vector<std::string>& aFilters, const std::string& aLib, const std::string& aName )
{
    if( aFilters.empty() )
        return true;

    for( const std::string& filter : aFilters )
    {
        const bool withLib = filter.find( ':' ) != std::string::npos;

        if( KopenapiGlob( filter, withLib ? aLib + ":" + aName : aName ) )
            return true;
    }

    return false;
}


std::map<std::string, int> usage( KOPENAPI_CONTEXT& aCtx )
{
    std::map<std::string, int> used;

    if( std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx ) )
    {
        for( const FOOTPRINT* fp : context->GetBoard()->Footprints() )
            used[str( fp->GetFPIDAsString() )]++;
    }

    return used;
}


nlohmann::json footprintRow( const wxString& aNickname, const FOOTPRINT* aFp )
{
    const BOX2I box = aFp->GetFpPadsLocalBbox();

    return { { "lib_id", str( aNickname ) + ":" + str( aFp->GetFPID().GetLibItemName() ) },
             { "description", str( aFp->GetLibDescription() ) },
             { "keywords", str( aFp->GetKeywords() ) },
             { "pads", connectablePads( aFp ) },
             { "pad_count", aFp->Pads().size() },
             { "type", mountType( aFp ) },
             { "pads_extent_mm", { mm( box.GetWidth() ), mm( box.GetHeight() ) } } };
}

} // namespace


static KOPENAPI_RESULT h_pcb_lib_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    FOOTPRINT_LIBRARY_ADAPTER* adapter = loadedAdapter( aCtx );

    if( !adapter )
        return KOPENAPI_RESULT::Error( 503, "no project to read library tables from" );

    const std::string           glob = aArgs.value( "name", std::string() );
    std::vector<nlohmann::json> rows;
    int                         errors = 0;

    for( const LIBRARY_TABLE_ROW* row : adapter->Rows( LIBRARY_TABLE_SCOPE::BOTH, true ) )
    {
        const std::string nickname = str( row->Nickname() );

        if( !KopenapiGlob( glob, nickname ) )
            continue;

        nlohmann::json out = { { "name", nickname },
                               { "scope", scopeName( row->Scope() ) },
                               { "type", str( row->Type() ) },
                               { "uri", str( row->URI() ) },
                               { "description", str( row->Description() ) },
                               { "enabled", !row->Disabled() } };

        std::optional<LIB_STATUS> status = adapter->GetLibraryStatus( row->Nickname() );

        if( !row->IsOk() )
        {
            out["status"] = "error";
            out["error"] = str( row->ErrorDescription() );
        }
        else if( status && status->load_status == LOAD_STATUS::LOAD_ERROR )
        {
            out["status"] = "error";
            out["error"] = status->error ? str( status->error->message ) : std::string();
        }
        else if( status && status->load_status == LOAD_STATUS::LOADED )
        {
            out["status"] = "loaded";
            out["footprints"] = adapter->GetFootprintNames( row->Nickname(), true ).size();
        }
        else
        {
            out["status"] = row->Disabled() ? "disabled" : "not_loaded";
        }

        errors += out["status"] == "error" ? 1 : 0;
        rows.push_back( std::move( out ) );
    }

    nlohmann::json result = KopenapiPage( rows, aArgs );
    result["tables"] = KopenapiLibraryTables( *adapter, errors );
    result["errors"] = errors;
    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_pcb_lib_footprint_search( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string query = aArgs.value( "query", std::string() );
    const std::string libGlob = aArgs.value( "lib", std::string() );
    const std::string type = aArgs.value( "type", std::string() );
    const int         pads = aArgs.value( "pads", -1 );

    std::vector<std::string> filters;

    if( aArgs.contains( "filters" ) && aArgs["filters"].is_array() )
    {
        for( const nlohmann::json& f : aArgs["filters"] )
        {
            if( f.is_string() && !f.get<std::string>().empty() )
                filters.push_back( f );
        }
    }

    if( query.empty() && libGlob.empty() && filters.empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'query' (words), 'filters' (footprint filters) and/or 'lib' (glob)" );

    FOOTPRINT_LIBRARY_ADAPTER* adapter = loadedAdapter( aCtx );

    if( !adapter )
        return KOPENAPI_RESULT::Error( 503, "no project to read library tables from" );

    const std::map<std::string, int>            used = usage( aCtx );
    std::vector<std::pair<KOPENAPI_TEXT_MATCH, nlohmann::json>> scored;

    for( const wxString& nickname : adapter->GetLibraryNames() )
    {
        const std::string lib = str( nickname );

        if( !KopenapiGlob( libGlob, lib ) )
            continue;

        for( const FOOTPRINT* fp : libraryFootprints( adapter, nickname ) )
        {
            if( !fp )
                continue;

            const std::string name = str( fp->GetFPID().GetLibItemName() );

            if( !matchesFilters( filters, lib, name ) )
                continue;

            if( pads >= 0 && connectablePads( fp ) != pads )
                continue;

            if( !type.empty() && type != mountType( fp ) )
                continue;

            const KOPENAPI_TEXT_MATCH match = KopenapiTextMatch( query, name, str( fp->GetKeywords() ), str( fp->GetLibDescription() ) );

            if( match.terms > 0 && match.matched == 0 )
                continue;

            nlohmann::json row = footprintRow( nickname, fp );

            auto u = used.find( row["lib_id"] );
            row["used_in_design"] = u == used.end() ? 0 : u->second;
            scored.emplace_back( match, std::move( row ) );
        }
    }

    return KOPENAPI_RESULT::Ok( KopenapiRankedPage( std::move( scored ), aArgs ) );
}


static KOPENAPI_RESULT h_pcb_lib_footprint_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string id = aArgs.value( "lib_id", std::string() );
    LIB_ID            libId;

    if( id.empty() || libId.Parse( wxString::FromUTF8( id ) ) >= 0 || libId.GetLibNickname().empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'lib_id' as LIBRARY:FOOTPRINT" );

    FOOTPRINT_LIBRARY_ADAPTER* adapter = loadedAdapter( aCtx );

    if( !adapter )
        return KOPENAPI_RESULT::Error( 503, "no project to read library tables from" );

    std::unique_ptr<FOOTPRINT> fp;

    try
    {
        fp.reset( adapter->LoadFootprint( libId, false ) );
    }
    catch( const IO_ERROR& error )
    {
        return KOPENAPI_RESULT::Error( 422, "library error: " + str( error.What() ) );
    }

    if( !fp )
        return KOPENAPI_RESULT::Error( 404, "footprint not found: " + id );

    nlohmann::json card = footprintRow( libId.GetLibNickname(), fp.get() );
    nlohmann::json padList = nlohmann::json::array();

    for( const PAD* pad : fp->Pads() )
    {
        const PCB_LAYER_ID layer = pad->GetPrincipalLayer();
        nlohmann::json     p = { { "number", str( pad->GetNumber() ) },
                                 { "attribute", pad->GetAttribute() == PAD_ATTRIB::SMD    ? "smd"
                                                : pad->GetAttribute() == PAD_ATTRIB::PTH  ? "pth"
                                                : pad->GetAttribute() == PAD_ATTRIB::NPTH ? "npth"
                                                                                          : "connector" },
                                 { "shape", str( pad->ShowPadShape( layer ) ) },
                                 { "x_mm", mm( pad->GetFPRelativePosition().x ) },
                                 { "y_mm", mm( pad->GetFPRelativePosition().y ) },
                                 { "size_mm", { mm( pad->GetSize( layer ).x ), mm( pad->GetSize( layer ).y ) } } };

        if( pad->HasHole() )
            p["drill_mm"] = { mm( pad->GetDrillSize().x ), mm( pad->GetDrillSize().y ) };

        padList.push_back( std::move( p ) );
    }

    std::sort( padList.begin(), padList.end(),
               []( const nlohmann::json& a, const nlohmann::json& b ) { return KopenapiNaturalLess( a["number"], b["number"] ); } );

    nlohmann::json models = nlohmann::json::array();

    for( const FP_3DMODEL& model : fp->Models() )
        models.push_back( str( model.m_Filename ) );

    BOX2I courtyard;
    bool  hasCourtyard = false;

    for( PCB_LAYER_ID layer : { F_CrtYd, B_CrtYd } )
    {
        const SHAPE_POLY_SET& poly = fp->GetCourtyard( layer );

        if( poly.OutlineCount() > 0 )
        {
            if( hasCourtyard )
                courtyard.Merge( poly.BBox() );
            else
                courtyard = poly.BBox();

            hasCourtyard = true;
        }
    }

    const std::map<std::string, int> used = usage( aCtx );
    auto                             u = used.find( card["lib_id"] );

    card["pad_list"] = padList;
    card["models_3d"] = models;
    card["courtyard_mm"] = hasCourtyard ? nlohmann::json( { mm( courtyard.GetWidth() ), mm( courtyard.GetHeight() ) } ) : nlohmann::json();
    card["reference"] = str( fp->GetReference() );
    card["value"] = str( fp->GetValue() );
    card["used_in_design"] = u == used.end() ? 0 : u->second;
    return KOPENAPI_RESULT::Ok( card );
}


KOPENAPI_REGISTER( "pcb_lib_list",
                   "List footprint libraries from KiCad's global and project fp-lib-tables: name, scope, "
                   "type, uri, status (loaded / error with message / disabled), footprint count; errors "
                   "counted; works without an open document; paginated",
                   KopenapiPagedSchema( R"json({"name":{"type":"string","description":"glob on the library name"}})json"_json ),
                   false, h_pcb_lib_list, 600 );

KOPENAPI_REGISTER( "pcb_lib_footprint_search",
                   "Search library footprints by words in name, keywords and description, ranked (e.g. "
                   "'sot-23', 'usb c receptacle', 'qfn 32 5x5'), or by a symbol's footprint filters "
                   "(from sch_lib_symbol_get) and pad count; filter by library glob, smd/tht; returns "
                   "lib_id, description, pads, type, pads extent, used_in_design; paginated",
                   KopenapiPagedSchema( R"json({
                        "query":{"type":"string","description":"words; items matching all of them, else best partial matches (partial, unmatched_terms)"},
                        "filters":{"type":"array","items":{"type":"string"},"description":"footprint filters, e.g. [\"SOT?23*\"]"},
                        "pads":{"type":"integer","description":"exact number of connectable pads (= symbol pins)"},
                        "type":{"type":"string","enum":["smd","tht","unspecified"]},
                        "lib":{"type":"string","description":"glob on the library name"}})json"_json ),
                   false, h_pcb_lib_footprint_search, 600 );

KOPENAPI_REGISTER( "pcb_lib_footprint_get",
                   "Library footprint card by lib_id (LIBRARY:FOOTPRINT): description, keywords, every pad "
                   "(number, attribute, shape, position, size, drill in mm), courtyard size, 3D models, "
                   "used_in_design",
                   R"json({"type":"object","required":["lib_id"],"properties":{
                        "lib_id":{"type":"string","description":"e.g. Package_TO_SOT_SMD:SOT-223-3_TabPin2"}}})json"_json,
                   false, h_pcb_lib_footprint_get, 600 );
