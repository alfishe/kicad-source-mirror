/// @file methods_pcb_footprint_export.cpp
/// @brief kicadopenapi pcb_footprints_export_lib: Export Footprints to Library / to New Library,
/// through the editor's own code (ExportFootprintsToLibrary, AddFootprintLibraryToTable).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <footprint_library_adapter.h>
#include <footprint_library_export.h>
#include <ki_exception.h>
#include <kicadopenapi_util.h>
#include <libraries/library_manager.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pgm_base.h>
#include <project.h>
#include <project_pcb.h>
#include <tool/tool_manager.h>
#include <wildcards_and_files_ext.h>

#include <wx/dir.h>
#include <wx/filename.h>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


/// The nickname of a table row whose library is at aPath, or empty
wxString nicknameOfPath( FOOTPRINT_LIBRARY_ADAPTER* aAdapter, const wxString& aPath )
{
    const wxString wanted = KopenapiCanonicalPath( aPath );

    for( const LIBRARY_TABLE_ROW* row : aAdapter->Rows( LIBRARY_TABLE_SCOPE::BOTH, true ) )
    {
        if( KopenapiCanonicalPath( LIBRARY_MANAGER::GetFullURI( row, true ) ) == wanted )
            return row->Nickname();
    }

    return wxEmptyString;
}


KOPENAPI_RESULT h_pcb_footprints_export_lib( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*   board = context->GetBoard();
    PROJECT* project = board->GetProject();

    if( !project )
        return KOPENAPI_RESULT::Error( 409, "the board has no project" );

    if( board->Footprints().empty() )
        return KOPENAPI_RESULT::Error( 422, "no footprints to export" );

    FOOTPRINT_LIBRARY_ADAPTER* adapter = PROJECT_PCB::FootprintLibAdapter( project );
    wxString                   library = wxString::FromUTF8( aArgs.value( "library", std::string() ) );

    if( library.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "library: a nickname of the footprint library tables (pcb_lib_list), or "
                                            "the absolute path of a .pretty directory (a new library)" );

    const std::string tableArg = aArgs.value( "table", std::string( "project" ) );

    if( tableArg != "project" && tableArg != "global" )
        return KOPENAPI_RESULT::Error( 400, "table: project or global" );

    nlohmann::json out = nlohmann::json::object();
    wxString       nickname;

    if( wxFileName libFn( library ); libFn.IsAbsolute() )
    {
        // Export Footprints to New Library: the library created and added to a table
        libFn.Normalize( wxPATH_NORM_DOTS | wxPATH_NORM_ABSOLUTE );
        library = libFn.GetFullPath();

        if(!library.EndsWith( wxString( wxS( "." ) ) + FILEEXT::KiCadFootprintLibPathExtension ) )
            return KOPENAPI_RESULT::Error( 400, "library: a new library is a .pretty directory" );

        nickname = nicknameOfPath( adapter, library );

        if( nickname.IsEmpty() )
        {
            const wxString name = wxFileName( library ).GetName();

            if( adapter->HasLibrary( name ) )
                return KOPENAPI_RESULT::Error( 409, "library: the tables already have a library named "
                                                            + str( name ) + " at another path" );

            if( !wxDir::Exists( library ) )
            {
                try
                {
                    PCB_IO_KICAD_SEXPR().CreateLibrary( library );
                }
                catch( const IO_ERROR& e )
                {
                    return KOPENAPI_RESULT::Error( 422, "library: " + str( e.What() ) );
                }

                out["created"] = true;
            }

            wxString error;

            if( !AddFootprintLibraryToTable( *project, library,
                                             tableArg == "global" ? LIBRARY_TABLE_SCOPE::GLOBAL
                                                                  : LIBRARY_TABLE_SCOPE::PROJECT,
                                             nickname, error ) )
            {
                return KOPENAPI_RESULT::Error( 422, "library: not added to the " + tableArg + " table"
                                                            + ( error.IsEmpty() ? std::string() : ": " + str( error ) ) );
            }

            out["added_to_table"] = tableArg;
        }
    }
    else
    {
        nickname = library;

        if( !adapter->HasLibrary( nickname ) )
            return KOPENAPI_RESULT::Error( 404, "library: no library " + str( nickname ) + " (pcb_lib_list)" );
    }

    if( !adapter->IsWritable( nickname ) )
        return KOPENAPI_RESULT::Error( 409, "library: " + str( nickname ) + " is read-only" );

    std::vector<FOOTPRINT_EXPORT_RESULT> results = ExportFootprintsToLibrary( *board, *adapter, nickname );

    const bool       relink = aArgs.value( "relink", false );
    BOARD_COMMIT     commit( context->GetToolManager() );
    KOPENAPI_LIST_CAP cap( aArgs, 50 );
    std::vector<nlohmann::json> rows;
    nlohmann::json   errors = nlohmann::json::array();
    int              saved = 0;
    int              relinked = 0;

    for( const FOOTPRINT_EXPORT_RESULT& result : results )
    {
        FOOTPRINT*     fp = result.m_Footprint;
        nlohmann::json row = { { "ref", str( fp->GetReference() ) },
                               { "name", str( fp->GetFPID().GetLibItemName().wx_str() ) },
                               { "saved", result.m_Saved } };

        if( !result.m_Error.IsEmpty() )
        {
            row["error"] = str( result.m_Error );
            errors.push_back( row );
        }

        saved += result.m_Saved ? 1 : 0;

        // relink only what was written, as the editor
        if( relink && result.m_Saved )
        {
            LIB_ID id = fp->GetFPID();
            id.SetLibNickname( nickname );

            if( id != fp->GetFPID() )
            {
                commit.Modify( fp );
                fp->SetFPID( id );
                relinked++;
            }

            row["lib_id"] = str( id.Format().wx_str() );
        }

        rows.push_back( std::move( row ) );
    }

    if( !commit.Empty() )
        commit.Push( _( "Export Footprints to Library" ) );

    std::optional<wxString> uri = Pgm().GetLibraryManager().GetFullURI( LIBRARY_TABLE_TYPE::FOOTPRINT, nickname, true );

    out["library"] = str( nickname );
    out["path"] = uri ? str( *uri ) : std::string();
    out["saved"] = saved;
    out["relinked"] = relinked;
    out["errors"] = errors;
    cap.Put( out, "footprints", std::move( rows ) );
    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_footprints_export_lib",
                   "Export Footprints to Library / to New Library, as the board editor's: every board footprint "
                   "with a library name is saved into the library in library form (REF**, no group; one of a "
                   "name overwritten). library: a nickname of the footprint library tables, or the absolute path "
                   "of a .pretty directory: created when missing and added to the table (project, default, or "
                   "global; nickname its name). relink: the board's footprints then refer to that library (one "
                   "undo step), as the editor's \"update board footprints\". Answers library, path, created, "
                   "added_to_table, saved, relinked, errors and the footprints (ref, name, saved, lib_id)",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","required":["library"],"properties":{
                        "library":{"type":"string"},
                        "table":{"type":"string","enum":["project","global"],"default":"project"},
                        "relink":{"type":"boolean","default":false}}})json"_json, 50 ),
                   false, h_pcb_footprints_export_lib, 300 );

KOPENAPI_MARK_EDITING( "pcb_footprints_export_lib" );
