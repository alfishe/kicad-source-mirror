/// @file methods_pcb_compare.cpp
/// @brief kicadopenapi pcb_compare: File > Compare Board with File / with Local History.
///
/// The other board is loaded read-only beside the live one (its own project slot, no editor
/// initialisation) and diffed by KiCad's PCB_DIFFER; nothing on the live board changes.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_loader.h>
#include <diff_merge/pcb_differ.h>
#include <kiway.h>
#include <local_history.h>
#include <pgm_base.h>
#include <pcb_io/pcb_io_mgr.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <wildcards_and_files_ext.h>

#include <wx/dir.h>
#include <wx/filename.h>
#include <wx/utils.h>

#include <atomic>
#include <cmath>
#include <map>
#include <memory>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


/// @brief A board loaded for comparison, with its project slot and temporary copy, released
/// in reverse order
struct OTHER_BOARD
{
    std::unique_ptr<BOARD> board;
    PROJECT*               project = nullptr;
    wxString               tempDir;

    ~OTHER_BOARD()
    {
        SETTINGS_MANAGER& mgr = Pgm().GetSettingsManager();

        if( board )
        {
            board->ClearProject();
            board.reset();
        }

        if( project && mgr.IsProjectLoaded( project ) )
            mgr.UnloadProject( project, false );

        if( !tempDir.IsEmpty() )
            wxFileName::Rmdir( tempDir, wxPATH_RMDIR_RECURSIVE );
    }
};


wxString newTempDir( const wxString& aTag )
{
    static std::atomic<int> s_serial{ 0 };
    wxFileName              dir;

    dir.AssignDir( wxFileName::GetTempDir() );
    dir.AppendDir( wxString::Format( wxS( "kicad-compare-%lu-%d-%s" ), wxGetProcessId(), ++s_serial, aTag ) );
    wxFileName::Mkdir( dir.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL );
    return dir.GetPath();
}


/// @brief Loads aBoardPath with the project aProjectPath as the editor's compare does; false and
/// aError when it cannot
bool loadOther( OTHER_BOARD& aOther, const wxString& aBoardPath, const wxString& aProjectPath, std::string& aError )
{
    SETTINGS_MANAGER& mgr = Pgm().GetSettingsManager();
    const bool        projectOk = mgr.LoadProject( aProjectPath, false );

    aOther.project = mgr.GetProject( aProjectPath );

    if( !aOther.project || ( !projectOk && wxFileName::FileExists( aProjectPath ) ) )
    {
        aError = "its project could not be loaded";
        return false;
    }

    BOARD_LOADER::OPTIONS options;
    options.initialize_after_load = false;

    try
    {
        aOther.board = BOARD_LOADER::Load( aBoardPath,
                                           PCB_IO_MGR::FindPluginTypeFromBoardPath( aBoardPath, KICTL_KICAD_ONLY ),
                                           aOther.project, options );
    }
    catch( const IO_ERROR& ioe )
    {
        aError = "could not be read: " + str( ioe.What() );
        return false;
    }
    catch( ... )
    {
        aError = "could not be read";
        return false;
    }

    if( !aOther.board )
    {
        aError = "could not be read";
        return false;
    }

    // Net classes and the other project-file settings the differ reads come from the project
    aOther.board->SetProject( aOther.project );
    return true;
}


/// @brief The board revisions of the local history, newest first, one per distinct board
std::vector<LOCAL_HISTORY_SNAPSHOT_INFO> boardSnapshots( LOCAL_HISTORY& aHistory, const wxString& aProjectPath )
{
    std::vector<LOCAL_HISTORY_SNAPSHOT_INFO> out;
    wxString                                 previous;

    for( const LOCAL_HISTORY_SNAPSHOT_INFO& s : aHistory.GetSnapshots( aProjectPath ) )
    {
        const wxString fingerprint = aHistory.TreeFingerprint( aProjectPath, s.hash, wxS( ".kicad_pcb" ) );

        if( fingerprint.IsEmpty() || fingerprint == previous )
            continue;

        previous = fingerprint;
        out.push_back( s );
    }

    return out;
}


nlohmann::json snapshotJson( const LOCAL_HISTORY_SNAPSHOT_INFO& aSnapshot, int aIndex )
{
    return { { "index", aIndex },
             { "hash", str( aSnapshot.hash ) },
             { "date", str( aSnapshot.date.FormatISOCombined( ' ' ) ) },
             { "summary", str( aSnapshot.summary.IsEmpty() ? aSnapshot.message.BeforeFirst( '\n' ) : aSnapshot.summary ) } };
}

} // namespace


static KOPENAPI_RESULT h_pcb_compare( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*     live = context->GetBoard();
    const bool byPath = aArgs.contains( "path" );
    const bool byHistory = aArgs.contains( "history" );

    if( byPath == byHistory )
        return KOPENAPI_RESULT::Error( 400, "path (a board file) or history (a snapshot: index, hash, or true for "
                                            "the newest; \"list\" lists them)" );

    OTHER_BOARD    other;
    std::string    error;
    wxString       otherLabel;
    nlohmann::json out = nlohmann::json::object();

    if( byPath )
    {
        wxFileName otherFn( wxString::FromUTF8( aArgs.value( "path", std::string() ) ) );

        if( !otherFn.IsAbsolute() )
            return KOPENAPI_RESULT::Error( 400, "path: an absolute path" );

        if( !otherFn.FileExists() )
            return KOPENAPI_RESULT::Error( 404, "path: no such file" );

        wxFileName projectFn = otherFn;
        projectFn.SetExt( FILEEXT::ProjectFileExtension );

        wxFileName liveProjectFn( live->GetProject() ? live->GetProject()->GetProjectFullName() : wxString() );
        wxString   boardPath = otherFn.GetFullPath();
        wxString   projectPath = projectFn.GetFullPath();

        // A board of the live project (its saved state, a copy beside it) loads from a temporary
        // copy: attaching the live project to a second board would rewrite its settings
        if( !liveProjectFn.GetFullName().IsEmpty() && projectFn.SameAs( liveProjectFn ) )
        {
            other.tempDir = newTempDir( wxS( "file" ) );
            boardPath = other.tempDir + wxFileName::GetPathSeparator() + otherFn.GetFullName();
            projectPath = other.tempDir + wxFileName::GetPathSeparator() + projectFn.GetFullName();

            if( !wxCopyFile( otherFn.GetFullPath(), boardPath )
                || ( projectFn.FileExists() && !wxCopyFile( projectFn.GetFullPath(), projectPath ) ) )
            {
                return KOPENAPI_RESULT::Error( 500, "could not copy the board for comparison" );
            }
        }

        if( !loadOther( other, boardPath, projectPath, error ) )
            return KOPENAPI_RESULT::Error( 422, "path: " + error );

        otherLabel = otherFn.GetFullPath();
        out["path"] = str( otherLabel );
    }
    else
    {
        if( !aCtx.kiway )
            return KOPENAPI_RESULT::Error( 409, "local history is not available in this host" );

        if( live->GetFileName().IsEmpty() || !live->GetProject() )
            return KOPENAPI_RESULT::Error( 409, "save the board before comparing against local history" );

        PROJECT*       project = live->GetProject();
        const wxString projectDir = project->GetProjectPath();
        LOCAL_HISTORY& history = aCtx.kiway->LocalHistory();
        std::vector<LOCAL_HISTORY_SNAPSHOT_INFO> snapshots = boardSnapshots( history, projectDir );
        const nlohmann::json& want = aArgs["history"];

        if( want.is_string() && want.get<std::string>() == "list" )
        {
            nlohmann::json list = nlohmann::json::array();

            for( size_t i = 0; i < snapshots.size(); ++i )
                list.push_back( snapshotJson( snapshots[i], (int) i ) );

            return KOPENAPI_RESULT::Ok( { { "snapshots", list } } );
        }

        if( snapshots.empty() )
            return KOPENAPI_RESULT::Error( 404, "no local history snapshot changes this board" );

        int index = -1;

        if( want.is_boolean() && want.get<bool>() )
        {
            index = 0;
        }
        else if( want.is_number_integer() )
        {
            index = want.get<int>();
        }
        else if( want.is_string() )
        {
            for( size_t i = 0; i < snapshots.size(); ++i )
            {
                if( snapshots[i].hash.StartsWith( wxString::FromUTF8( want.get<std::string>() ) ) )
                    index = (int) i;
            }
        }

        if( index < 0 || index >= (int) snapshots.size() )
            return KOPENAPI_RESULT::Error( 404, "history: no such snapshot (history: \"list\")" );

        wxFileName boardRel( live->GetFileName() );
        boardRel.MakeRelativeTo( projectDir );
        wxFileName projectRel( project->GetProjectFullName() );
        projectRel.MakeRelativeTo( projectDir );

        other.tempDir = newTempDir( snapshots[index].hash.Left( 8 ) );

        if( !history.ExtractAllFilesAtCommit( projectDir, snapshots[index].hash, other.tempDir,
                                              { wxS( ".kicad_pcb" ), wxS( ".kicad_pro" ) } ) )
        {
            return KOPENAPI_RESULT::Error( 422, "history: the snapshot could not be extracted" );
        }

        if( !loadOther( other, other.tempDir + wxS( "/" ) + boardRel.GetFullPath( wxPATH_UNIX ),
                        other.tempDir + wxS( "/" ) + projectRel.GetFullPath( wxPATH_UNIX ), error ) )
        {
            return KOPENAPI_RESULT::Error( 422, "history: the snapshot " + error );
        }

        otherLabel = snapshots[index].hash;
        out["snapshot"] = snapshotJson( snapshots[index], index );
    }

    KICAD_DIFF::PCB_DIFFER    differ( live, other.board.get(), otherLabel );
    KICAD_DIFF::DOCUMENT_DIFF diff = differ.Diff();

    std::map<std::string, int>                        byKind;
    std::map<std::string, std::map<std::string, int>> byType;
    nlohmann::json                                    changes = nlohmann::json::array();
    const size_t                                      limit = std::max( 0, aArgs.value( "limit", 200 ) );

    for( const KICAD_DIFF::ITEM_CHANGE& change : diff.changes )
    {
        const std::string kind = KICAD_DIFF::ChangeKindToString( change.kind );
        byKind[kind]++;
        byType[str( change.typeName )][kind]++;

        if( changes.size() < limit )
            changes.push_back( change.ToJson() );
    }

    out["identical"] = diff.Empty();
    out["total"] = diff.Size();
    out["summary"] = byKind;
    out["by_type"] = byType;
    out["changes"] = changes;

    if( diff.Size() > changes.size() )
        out["truncated"] = true;

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_compare",
                   "Compare Board with File / with Local History, as the editor's: the open board (before) against "
                   "another board file (path; the open board's own saved file works too) or a local-history "
                   "snapshot (history: index from the newest 0, a hash prefix, true for the newest; \"list\" "
                   "lists the revisions that change the board). Read-only. Answers identical, total, summary by "
                   "change kind (added / removed / modified), by item type, and the changes (KiCad's diff records: "
                   "id, type, kind, changed properties before / after, bbox, refdes, children; first limit)",
                   R"json({"type":"object","properties":{
                        "path":{"type":"string","description":"absolute path of a .kicad_pcb"},
                        "history":{"type":["integer","string","boolean"]},
                        "limit":{"type":"integer","default":200}}})json"_json,
                   false, h_pcb_compare, 600 );

