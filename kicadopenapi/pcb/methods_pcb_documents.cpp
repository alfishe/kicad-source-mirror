/// @file methods_pcb_documents.cpp
/// @brief kicadopenapi PCB document methods: pcb_open, pcb_close, pcb_save, pcb_revert, and the
/// "pcb" document provider.  Compiled into the pcbnew kiface; registered when it loads.
#include <kicadopenapi_util.h>
#include <kicadopenapi_change_feed.h>
#include <model_thread.h>
#include <kicadopenapi_legacy_save.h>
#include <kicadopenapi_lock.h>
#include <kicadopenapi_service.h>
#include "kopenapi_pcb.h"

#include <wx/log.h>

#include <api/headless_pcb_context.h>
#include <api/pcb_context.h>
#include <board.h>
#include <connectivity/connectivity_data.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <richio.h>
#include <board_loader.h>
#include <board_design_settings.h>
#include <api/board/board_rules.pb.h>
#include <drc/drc_engine.h>
#include <drc/drc_exclusion.h>
#include <drc/drc_item.h>
#include <pcb_marker.h>
#include <kiway.h>
#include <tool/tool_manager.h>
#include <tool/actions.h>
#include <kicadopenapi_history.h>
#include <kicadopenapi_undo.h>
#include <pcb_edit_frame.h>
#include <pcb_io/pcb_io_mgr.h>
#include <pcbnew_settings.h>
#include <pgm_base.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>


/// @brief Headless board; GUI hosts use the editor frame instead
static std::shared_ptr<HEADLESS_PCB_CONTEXT> s_headless;


KOPENAPI_KEEP_UNSAVED::KOPENAPI_KEEP_UNSAVED() :
        m_unsaved( s_headless && s_headless->IsContentModified() )
{
}


KOPENAPI_KEEP_UNSAVED::~KOPENAPI_KEEP_UNSAVED()
{
    if( s_headless )
        s_headless->SetContentModified( m_unsaved );
}


static PCB_EDIT_FRAME* guiFrame( KOPENAPI_CONTEXT& aCtx, bool aCreate )
{
    if( aCtx.headless || !aCtx.kiway )
        return nullptr;

    return static_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, aCreate ) );
}


/// @brief Headless: any change of a board item marks the board unsaved, whichever call made it
/// (DRC markers are not saved with the board)
class UNSAVED_WATCH : public BOARD_LISTENER
{
public:
    void OnBoardItemAdded( BOARD&, BOARD_ITEM* aItem ) override { touch( aItem ); }
    void OnBoardItemsAdded( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override { touch( aItems ); }
    void OnBoardItemRemoved( BOARD&, BOARD_ITEM* aItem ) override { touch( aItem ); }
    void OnBoardItemsRemoved( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override { touch( aItems ); }
    void OnBoardItemChanged( BOARD&, BOARD_ITEM* aItem ) override { touch( aItem ); }
    void OnBoardItemsChanged( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override { touch( aItems ); }
    void OnBoardNetSettingsChanged( BOARD& ) override { mark(); }

    void OnBoardCompositeUpdate( BOARD&, std::vector<BOARD_ITEM*>& aAdded, std::vector<BOARD_ITEM*>& aRemoved,
                                 std::vector<BOARD_ITEM*>& aChanged ) override
    {
        touch( aAdded );
        touch( aRemoved );
        touch( aChanged );
    }

private:
    static void mark()
    {
        if( s_headless )
            s_headless->SetContentModified( true );
    }

    static void touch( const BOARD_ITEM* aItem )
    {
        if( aItem && aItem->Type() != PCB_MARKER_T )
            mark();
    }

    static void touch( const std::vector<BOARD_ITEM*>& aItems )
    {
        for( const BOARD_ITEM* item : aItems )
        {
            if( item && item->Type() != PCB_MARKER_T )
                return mark();
        }
    }
};


/// @brief Records every commit of the board in the change feed (owner thread)
class BOARD_CHANGE_FEED : public BOARD_LISTENER
{
public:
    void OnBoardItemAdded( BOARD&, BOARD_ITEM* aItem ) override { KOPENAPI_CHANGE_FEED::Record( "pcb", { id( aItem ) }, {}, {} ); }
    void OnBoardItemRemoved( BOARD&, BOARD_ITEM* aItem ) override { KOPENAPI_CHANGE_FEED::Record( "pcb", {}, {}, { id( aItem ) } ); }
    void OnBoardItemChanged( BOARD&, BOARD_ITEM* aItem ) override { KOPENAPI_CHANGE_FEED::Record( "pcb", {}, { id( aItem ) }, {} ); }

    void OnBoardItemsAdded( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override
    {
        KOPENAPI_CHANGE_FEED::Record( "pcb", ids( aItems ), {}, {} );
    }

    void OnBoardItemsRemoved( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override
    {
        KOPENAPI_CHANGE_FEED::Record( "pcb", {}, {}, ids( aItems ) );
    }

    void OnBoardItemsChanged( BOARD&, std::vector<BOARD_ITEM*>& aItems ) override
    {
        KOPENAPI_CHANGE_FEED::Record( "pcb", {}, ids( aItems ), {} );
    }

    void OnBoardCompositeUpdate( BOARD&, std::vector<BOARD_ITEM*>& aAdded, std::vector<BOARD_ITEM*>& aRemoved,
                                 std::vector<BOARD_ITEM*>& aChanged ) override
    {
        KOPENAPI_CHANGE_FEED::Record( "pcb", ids( aAdded ), ids( aChanged ), ids( aRemoved ) );
    }

    /// @brief Listens to this board; a different board than before resets the feed
    void Follow( BOARD* aBoard )
    {
        if( !aBoard || !MODEL_THREAD::MayWrite() )
            return;

        aBoard->AddListener( this );

        if( aBoard != m_board )
        {
            m_board = aBoard;
            KOPENAPI_CHANGE_FEED::Reset( "pcb" );
        }
    }

private:
    static std::string id( const BOARD_ITEM* aItem ) { return aItem->m_Uuid.AsStdString(); }

    static std::vector<std::string> ids( const std::vector<BOARD_ITEM*>& aItems )
    {
        std::vector<std::string> out;
        out.reserve( aItems.size() );

        for( const BOARD_ITEM* item : aItems )
            out.push_back( id( item ) );

        return out;
    }

    BOARD* m_board = nullptr;
};


static BOARD_CHANGE_FEED& boardChangeFeed()
{
    static BOARD_CHANGE_FEED* s_feed = new BOARD_CHANGE_FEED();   // boards keep a pointer to it: never freed
    return *s_feed;
}


std::shared_ptr<PCB_CONTEXT> KopenapiPcbContext( KOPENAPI_CONTEXT& aCtx )
{
    std::shared_ptr<PCB_CONTEXT> context;

    if( aCtx.headless )
    {
        context = s_headless;

        static UNSAVED_WATCH watch;

        // the board object changes on open / revert / rollback; adding twice is a no-op
        if( context && context->GetBoard() && MODEL_THREAD::MayWrite() )
            context->GetBoard()->AddListener( &watch );
    }
    else if( PCB_EDIT_FRAME* frame = guiFrame( aCtx, false ) )
    {
        context = CreatePcbFrameContext( frame );

        if( context->GetCurrentFileName().IsEmpty() )
            context = nullptr;
    }

    if( context )
        boardChangeFeed().Follow( context->GetBoard() );

    return context;
}


KOPENAPI_RESULT KopenapiNoBoard()
{
    return KOPENAPI_RESULT::Error( 409, "no board is open (pcb_open)" );
}


static nlohmann::json documentStatus( KOPENAPI_CONTEXT& aCtx )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return nullptr;

    return { { "domain", "pcb" },
             { "path", context->GetCurrentFileName().ToStdString() },
             { "project", context->Prj().GetProjectFullName().ToStdString() },
             { "unsaved", context->IsContentModified() },
             { "mode", aCtx.headless ? "headless" : "gui" } };
}


/// @brief Another open document of this process belongs to a different project
static std::string projectConflict( KOPENAPI_CONTEXT& aCtx, const wxFileName& aProject )
{
    for( const nlohmann::json& doc : KOPENAPI_REGISTRY::Get().Documents( aCtx ) )
    {
        if( doc.value( "domain", "" ) == "pcb" )
            continue;

        const wxString other = wxString::FromUTF8( doc.value( "project", "" ) );

        // canonical on both sides: a symbolic link or "." in a path is the same project
        if( !other.IsEmpty() && KopenapiCanonicalPath( other ) != KopenapiCanonicalPath( aProject.GetFullPath() ) )
            return doc.value( "project", "" );
    }

    return {};
}


static bool schematicOpen( KOPENAPI_CONTEXT& aCtx )
{
    for( const nlohmann::json& doc : KOPENAPI_REGISTRY::Get().Documents( aCtx ) )
    {
        if( doc.value( "domain", "" ) != "pcb" )
            return true;
    }

    return false;
}


static KOPENAPI_RESULT openHeadless( KOPENAPI_CONTEXT& aCtx, const wxFileName& aBoard, const nlohmann::json& aArgs )
{
    wxFileName projectFile( aBoard );
    projectFile.SetExt( FILEEXT::ProjectFileExtension );

    if( std::string other = projectConflict( aCtx, projectFile ); !other.empty() )
    {
        if( !aArgs.value( "switch_project", false ) )
        {
            return KOPENAPI_RESULT::Error( 409, "this process serves project " + other
                                                        + "; sch_close first, pass switch_project: true, or start "
                                                          "another instance for a different project" );
        }

        // the other project's schematic goes first, as sch_close would close it
        std::optional<KOPENAPI_METHOD> close = KOPENAPI_REGISTRY::Get().Find( "sch_close" );

        if( !close )
            return KOPENAPI_RESULT::Error( 500, "sch_close is not available in this process" );

        KOPENAPI_RESULT closed = close->handler( aCtx, { { "discard", aArgs.value( "discard", false ) } } );

        if( closed.status >= 400 )
            return closed;
    }

    // Drop the current board first: activating another project destroys the old PROJECT,
    // which the old board still references (unsaved changes were checked by the caller)
    s_headless.reset();

    SETTINGS_MANAGER& settings = Pgm().GetSettingsManager();
    PROJECT*          project = settings.GetProject( projectFile.GetFullPath() );

    // discard rereads the project file too, unless a schematic of this process still uses it
    if( project && aArgs.value( "discard", false ) && !schematicOpen( aCtx ) )
    {
        settings.UnloadProject( project, false );
        project = nullptr;
    }

    if( !project )
    {
        settings.LoadProject( projectFile.GetFullPath(), true );
        project = settings.GetProject( projectFile.GetFullPath() );
    }

    if( !project )
        return KOPENAPI_RESULT::Error( 500, "could not load the project for this board" );

    PCB_IO_MGR::PCB_FILE_T type =
            PCB_IO_MGR::FindPluginTypeFromBoardPath( aBoard.GetFullPath(), KICTL_KICAD_ONLY );

    if( type == PCB_IO_MGR::FILE_TYPE_NONE )
        return KOPENAPI_RESULT::Error( 415, "not a KiCad board file" );

    std::unique_ptr<BOARD> board;

    try
    {
        board = BOARD_LOADER::Load( aBoard.GetFullPath(), type, project );
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 422, "board could not be loaded: " + ioe.What().ToStdString() );
    }

    if( !board )
        return KOPENAPI_RESULT::Error( 422, "board could not be loaded" );

    // As the editor does: a legacy .brd is saved in the current format under a .kicad_pcb name
    const bool legacy = aBoard.GetExt() == FILEEXT::LegacyPcbFileExtension;

    if( legacy )
    {
        wxFileName current( aBoard );
        current.SetExt( FILEEXT::KiCadPcbFileExtension );
        board->SetFileName( current.GetFullPath() );
    }

    s_headless = std::make_shared<HEADLESS_PCB_CONTEXT>(
            std::move( board ), project, GetAppSettings<PCBNEW_SETTINGS>( "pcbnew" ), aCtx.kiway );

    if( legacy )
        s_headless->SetContentModified( true );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT openGui( KOPENAPI_CONTEXT& aCtx, const wxFileName& aBoard )
{
    PCB_EDIT_FRAME* frame = guiFrame( aCtx, true );

    if( !frame )
        return KOPENAPI_RESULT::Error( 500, "could not create the PCB editor window" );

    // another process holds it (or its project) open: refused here, never asked in a dialog
    if( !KICAD_OPENAPI_SERVICE::OverrideLock() && frame->GetCurrentFileName() != aBoard.GetFullPath() )
    {
        const wxString ownProject = aCtx.kiway ? aCtx.kiway->Prj().GetProjectFullName() : wxString();

        if( const std::string owner = KopenapiLockedBy( aBoard, ownProject ); !owner.empty() )
        {
            return KOPENAPI_RESULT::Error( 409, "the board or its project is open in another process (" + owner
                                                        + "): close it there, or open with override_lock: true" );
        }
    }

    // the same file again (a reload): KiCad would find its own lock on it and refuse
    const bool same = frame->GetCurrentFileName() == aBoard.GetFullPath();

    if( same )
        frame->ReleaseFile();

    if( !frame->OpenProjectFiles( { aBoard.GetFullPath() } ) )
    {
        if( same )
            frame->LockFile( aBoard.GetFullPath() );

        return KOPENAPI_RESULT::Error( 422, "the PCB editor could not open the board" );
    }

    // OpenProjectFiles does not show the frame; mirror KICAD_MANAGER_CONTROL::ShowPlayer
    frame->Iconize( false );
    frame->Show( true );
    frame->Raise();

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT h_pcb_open( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_HISTORY::Get().Reset( "pcb", "pcb_open" );   // earlier marks belong to another document state

    if( !aArgs["path"].is_string() || aArgs["path"].get<std::string>().empty() )
        return KOPENAPI_RESULT::Error( 400, "'path' must be a non-empty string" );

    // one spelling per file (symbolic links, "."): the project is found by this path
    wxFileName board( KopenapiCanonicalPath( wxString::FromUTF8( aArgs["path"].get<std::string>() ) ) );
    board.MakeAbsolute();

    if( board.GetExt() == FILEEXT::ProjectFileExtension || board.GetExt() == FILEEXT::LegacyProjectFileExtension )
        board.SetExt( FILEEXT::KiCadPcbFileExtension );

    if( board.GetExt() != FILEEXT::KiCadPcbFileExtension && board.GetExt() != FILEEXT::LegacyPcbFileExtension )
        return KOPENAPI_RESULT::Error( 400, "expected a .kicad_pcb (or a KiCad 4 .brd) or a .kicad_pro / .pro file" );

    if( !board.FileExists() )
        return KOPENAPI_RESULT::Error( 404, "file not found: " + board.GetFullPath().ToStdString() );

    // Never let a "save changes?" dialog or a silent drop happen: unsaved work blocks
    if( std::shared_ptr<PCB_CONTEXT> current = KopenapiPcbContext( aCtx ) )
    {
        if( current->IsContentModified() )
        {
            if( !aArgs.value( "discard", false ) )
                return KOPENAPI_RESULT::Error( 409, "the open board has unsaved changes; pcb_save, "
                                                    "pcb_revert, or pass discard: true" );

            current->SetContentModified( false );
        }
    }

    if( aCtx.headless )
        return openHeadless( aCtx, board, aArgs );

    // a file another process has open: opened only when asked (no modal question)
    KICAD_OPENAPI_SERVICE::SetOverrideLock( aArgs.value( "override_lock", false ) );
    KOPENAPI_RESULT result = openGui( aCtx, board );
    KICAD_OPENAPI_SERVICE::SetOverrideLock( false );

    if( result.status != 200 )
        result.body["hint"] = "another process may have the file open (errors: journal); override_lock: true opens it anyway";

    return result;
}


static KOPENAPI_RESULT h_pcb_close( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_HISTORY::Get().Reset( "pcb", "pcb_close" );   // earlier marks belong to another document state

    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( context->IsContentModified() )
    {
        if( !aArgs.value( "discard", false ) )
            return KOPENAPI_RESULT::Error( 409, "the board has unsaved changes; pcb_save or pass discard: true" );

        wxLogWarning( "Unsaved changes to %s discarded (pcb_close discard)", context->GetCurrentFileName() );
        context->SetContentModified( false );
    }

    nlohmann::json closed = documentStatus( aCtx );

    if( aCtx.headless )
        s_headless.reset();
    else if( PCB_EDIT_FRAME* frame = guiFrame( aCtx, false ) )
        frame->Close( true );

    return KOPENAPI_RESULT::Ok( { { "closed", closed } } );
}


/// @brief A copy of the open board (and its project and rules) at aTarget, for an earlier KiCad.
/// The open document keeps its file and modified state.
/// DRC exclusions as 10.0 stores them: [ "key|x|y|...", comment ] (PCB_MARKER::SerializeToString of 10.0)
static nlohmann::json legacyDrcExclusions( const BOARD& aBoard )
{
    nlohmann::json out = nlohmann::json::array();

    for( const DRC_EXCLUSION& exclusion : aBoard.GetDesignSettings().m_DrcExclusions )
    {
        std::unique_ptr<PCB_MARKER> marker( PCB_MARKER::FromProto( exclusion.ToProto().marker() ) );

        if( !marker || !marker->GetRCItem() )
            continue;

        std::shared_ptr<RC_ITEM> item = marker->GetRCItem();
        const int                code = item->GetErrorCode();
        const VECTOR2I           pos = marker->GetPos();
        wxString                 text;

        if( code == DRCE_COPPER_SLIVER || code == DRCE_GENERIC_WARNING || code == DRCE_GENERIC_ERROR )
        {
            text = wxString::Format( wxT( "%s|%d|%d|%s|%s" ), item->GetSettingsKey(), pos.x, pos.y,
                                     item->GetMainItemID().AsString(), LayerName( marker->GetLayer() ) );
        }
        else if( code == DRCE_UNCONNECTED_ITEMS )
        {
            PCB_LAYER_ID layer = marker->GetLayer() == UNDEFINED_LAYER ? F_Cu : marker->GetLayer();
            text = wxString::Format( wxT( "%s|%d|%d|%s|%d|%s|%s" ), item->GetSettingsKey(), pos.x, pos.y,
                                     LayerName( layer ), (int) marker->GetMarkerType(),
                                     item->GetMainItemID().AsString(), item->GetAuxItemID().AsString() );
        }
        else if( code == DRCE_STARVED_THERMAL )
        {
            text = wxString::Format( wxT( "%s|%d|%d|%s|%s|%s" ), item->GetSettingsKey(), pos.x, pos.y,
                                     item->GetMainItemID().AsString(), item->GetAuxItemID().AsString(),
                                     LayerName( marker->GetLayer() ) );
        }
        else if( code == DRCE_UNRESOLVED_VARIABLE && marker->GetMarkerType() == MARKER_BASE::MARKER_DRAWING_SHEET )
        {
            text = wxString::Format( wxT( "%s|%d|%d||" ), item->GetSettingsKey(), pos.x, pos.y );
        }
        else
        {
            text = wxString::Format( wxT( "%s|%d|%d|%s|%s" ), item->GetSettingsKey(), pos.x, pos.y,
                                     item->GetMainItemID().AsString(), item->GetAuxItemID().AsString() );
        }

        out.push_back( { std::string( text.ToUTF8() ), std::string( exclusion.GetComment().ToUTF8() ) } );
    }

    return out;
}


static KOPENAPI_RESULT saveVersionCopy( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const wxFileName& aTarget,
                                       const std::string& aVersion )
{
    BOARD*     board = aContext.GetBoard();
    wxFileName current( aContext.GetCurrentFileName() );
    current.MakeAbsolute();

    if( !board )
        return KopenapiNoBoard();

    if( aTarget.GetFullPath() == current.GetFullPath() )
        return KOPENAPI_RESULT::Error( 409, "a copy for an earlier version cannot replace the open board's file" );

    wxFileName project( aTarget );
    project.SetExt( FILEEXT::ProjectFileExtension );
    wxFileName openProject( board->GetProject() ? board->GetProject()->GetProjectFullName() : wxString() );

    if( openProject.IsOk() && project.GetFullPath() == openProject.GetFullPath() )
        return KOPENAPI_RESULT::Error( 409, "the copy's project file would replace the open project; use another name or folder" );

    if( !aTarget.DirExists() && !wxFileName::Mkdir( aTarget.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
        return KOPENAPI_RESULT::Error( 500, "cannot create " + std::string( aTarget.GetPath().ToUTF8() ) );

    wxString outPath = aTarget.GetFullPath();

    if( !BOARD_LOADER::SaveBoard( outPath, *board ) )
        return KOPENAPI_RESULT::Error( 500, "writing the board failed" );

    nlohmann::json files = nlohmann::json::array();
    nlohmann::json report;
    wxString       error;

    if( !KopenapiLegacyConvertFile( outPath, aVersion, report, error ) )
        return KOPENAPI_RESULT::Error( 500, std::string( error.ToUTF8() ) );

    files.push_back( report );

    if( board->GetProject()
        && ( !Pgm().GetSettingsManager().SaveProjectCopy( project.GetFullPath(), board->GetProject() )
             || !project.FileExists() ) )
    {
        return KOPENAPI_RESULT::Error( 500, "writing the project copy failed" );
    }

    if( board->GetProject() )
    {
        nlohmann::json drc = legacyDrcExclusions( *board );
        nlohmann::json projectReport;

        if( !KopenapiLegacyConvertProject( project.GetFullPath(), aVersion, &drc, projectReport, error ) )
            return KOPENAPI_RESULT::Error( 500, std::string( error.ToUTF8() ) );

        files.push_back( projectReport );
    }

    wxFileName rules( board->GetDesignRulesPath() );
    wxFileName targetRules( aTarget );
    targetRules.SetExt( FILEEXT::DesignRulesFileExtension );

    if( rules.FileExists() && rules.GetFullPath() != targetRules.GetFullPath() )
        wxCopyFile( rules.GetFullPath(), targetRules.GetFullPath(), true );

    nlohmann::json status = documentStatus( aCtx );
    status["copy"] = { { "path", std::string( outPath.ToUTF8() ) },
                       { "version", aVersion },
                       { "project", std::string( project.GetFullPath().ToUTF8() ) },
                       { "files", files } };
    return KOPENAPI_RESULT::Ok( status );
}


static KOPENAPI_RESULT h_pcb_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    std::string version;
    std::string versionError;

    if( !KopenapiLegacyTarget( aArgs, version, versionError ) )
        return KOPENAPI_RESULT::Error( 400, versionError );

    if( !version.empty() )
    {
        if( !aArgs.contains( "path" ) || !aArgs["path"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "'version' writes a copy: 'path' (.kicad_pcb) is required" );

        wxFileName target( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
        target.MakeAbsolute();

        if( target.GetExt() != FILEEXT::KiCadPcbFileExtension )
            return KOPENAPI_RESULT::Error( 400, "expected a .kicad_pcb path" );

        return saveVersionCopy( aCtx, *context, target, version );
    }

    bool ok = false;

    if( aArgs.contains( "path" ) && aArgs["path"].is_string() )
    {
        wxFileName target( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
        target.MakeAbsolute();

        if( target.GetExt() != FILEEXT::KiCadPcbFileExtension )
            return KOPENAPI_RESULT::Error( 400, "expected a .kicad_pcb path" );

        // Save As switches the document (headless); GUI hosts write a copy and keep editing
        ok = aCtx.headless ? context->SaveBoardAs( target.GetFullPath() )
                           : context->SavePcbCopy( target.GetFullPath(), false, false );
    }
    else
    {
        ok = context->SaveBoard();
    }

    if( !ok )
        return KOPENAPI_RESULT::Error( 500, "saving the board failed" );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT h_pcb_revert( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    KOPENAPI_HISTORY::Get().Reset( "pcb", "pcb_revert" );   // earlier marks belong to another document state

    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    if( !context->RevertToSaved() )
        return KOPENAPI_RESULT::Error( 422, "reverting failed (no saved file?)" );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


// ---- edit history (kicadopenapi_history.h) --------------------------------------------------

/// @brief The internal board copy, not prettified but broken into lines before the lists of the first
/// nesting levels: the compact writer puts the whole board on one line, and the reader takes lines of
/// up to 16 MB only (LINE_READER_LINE_DEFAULT_MAX)
class LINED_FILE_OUTPUTFORMATTER : public FILE_OUTPUTFORMATTER
{
public:
    using FILE_OUTPUTFORMATTER::FILE_OUTPUTFORMATTER;

protected:
    void write( const char* aOutBuf, int aCount ) override
    {
        m_buf.clear();
        m_buf.reserve( (size_t) aCount + aCount / 16 + 1 );

        for( int i = 0; i < aCount; ++i )
        {
            const char c = aOutBuf[i];

            if( m_escape )
                m_escape = false;
            else if( m_quoted )
                m_escape = c == '\\', m_quoted = c != '"';
            else if( c == '"' )
                m_quoted = true;
            else if( c == '(' && m_depth++ < 4 && m_started )
                m_buf.push_back( '\n' );
            else if( c == ')' )
                m_depth--;

            m_started = true;
            m_buf.push_back( c );
        }

        FILE_OUTPUTFORMATTER::write( m_buf.data(), (int) m_buf.size() );
    }

private:
    std::string m_buf;
    int         m_depth = 0;
    bool        m_quoted = false;
    bool        m_escape = false;
    bool        m_started = false;
};


static KOPENAPI_DOC_HISTORY history()
{
    KOPENAPI_DOC_HISTORY h;

    h.undoMark = []( KOPENAPI_CONTEXT& aCtx ) -> std::optional<KOPENAPI_UNDO_MARK>
    {
        PCB_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame || !KopenapiPcbContext( aCtx ) )
            return std::nullopt;

        return KopenapiUndoMark( *frame );
    };

    h.undoTarget = []( KOPENAPI_CONTEXT& aCtx, const KOPENAPI_UNDO_MARK& aMark,
                       std::string& aWhy ) -> std::optional<int>
    {
        PCB_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame )
        {
            aWhy = "the editor window is closed";
            return std::nullopt;
        }

        return KopenapiUndoTarget( *frame, aMark, aWhy );
    };

    h.undoTo = []( KOPENAPI_CONTEXT& aCtx, int aDepth )
    {
        PCB_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame )
            return false;

        while( frame->GetUndoCommandCount() > aDepth )
        {
            const int before = frame->GetUndoCommandCount();
            frame->GetToolManager()->RunAction( ACTIONS::undo );

            if( frame->GetUndoCommandCount() >= before )
                return false;   // nothing undone: stop rather than loop
        }

        return frame->GetUndoCommandCount() == aDepth;
    };

    // A headless commit (no board editor) updates the connectivity of what it changed but leaves
    // the ratsnest to the editor, which is not there: the nets it touched are computed here, as the
    // editor's commit does
    h.afterEdit = []( KOPENAPI_CONTEXT& aCtx )
    {
        if( !aCtx.headless || !s_headless || !s_headless->GetBoard() )
            return;

        BOARD*                             board = s_headless->GetBoard();
        std::shared_ptr<CONNECTIVITY_DATA> conn = board->GetConnectivity();

        if( conn && !KopenapiConnectivityCurrent( *conn ) )
        {
            conn->RecalculateRatsnest();
            board->UpdateRatsnestExclusions();
        }
    };

    h.snapshot = []( KOPENAPI_CONTEXT& aCtx, const std::string& aDir )
    {
        if( !aCtx.headless || !s_headless )
            return false;

        const wxFileName target( wxString::FromUTF8( aDir ), wxFileName( s_headless->GetCurrentFileName() ).GetFullName() );

        // an internal copy read back only by restore: plain s-expressions, without the prettifying
        // pass of a saved file (half the time of a copy of a large board, taken before every edit)
        try
        {
            LINED_FILE_OUTPUTFORMATTER out( target.GetFullPath() );
            PCB_IO_KICAD_SEXPR         io;
            io.FormatBoardToFormatter( &out, s_headless->GetBoard() );

            if( !out.Finish() )   // written to a sibling temporary file until here
                return false;
        }
        catch( const IO_ERROR& )
        {
            return false;
        }

        return true;
    };

    h.restore = []( KOPENAPI_CONTEXT& aCtx, const std::string& aDir )
    {
        if( !aCtx.headless || !s_headless )
            return false;

        const wxString   original = s_headless->GetCurrentFileName();
        const wxFileName copy( wxString::FromUTF8( aDir ), wxFileName( original ).GetFullName() );
        PROJECT*         project = s_headless->GetBoard()->GetProject();
        std::unique_ptr<BOARD> board;

        try
        {
            board = BOARD_LOADER::Load( copy.GetFullPath(), PCB_IO_MGR::KICAD_SEXP, project );
        }
        catch( const IO_ERROR& e )
        {
            wxLogWarning( "Board copy %s not loaded back: %s", copy.GetFullPath(), e.What() );
            return false;
        }
        catch( const std::exception& e )
        {
            wxLogWarning( "Board copy %s not loaded back: %s", copy.GetFullPath(), e.what() );
            return false;
        }

        if( !board )
        {
            wxLogWarning( "Board copy %s not loaded back", copy.GetFullPath() );
            return false;
        }

        // Back under the document's own path, unsaved, with the custom rules beside it (the loader
        // looked for them beside the copy)
        board->SetFileName( original );

        if( std::shared_ptr<DRC_ENGINE> engine = board->GetDesignSettings().m_DRCEngine )
        {
            try
            {
                engine->InitEngine( board->GetDesignRulesPath() );
            }
            catch( ... )
            {
                // a broken custom rules file: reported by pcb_drc
            }
        }
        s_headless.reset();
        s_headless = std::make_shared<HEADLESS_PCB_CONTEXT>( std::move( board ), project,
                                                             GetAppSettings<PCBNEW_SETTINGS>( "pcbnew" ), aCtx.kiway );
        s_headless->SetContentModified( true );
        return true;
    };

    // BOARD_COMMIT marks only an editor frame modified
    h.edited = []( KOPENAPI_CONTEXT& aCtx )
    {
        if( aCtx.headless && s_headless )
            s_headless->SetContentModified( true );
    };

    return h;
}


KOPENAPI_REGISTER_HISTORY( "pcb", history() );

KOPENAPI_REGISTER_DOCUMENTS( "pcb", documentStatus,
                             []()
                             {
                                 // Stopping a headless server with unsaved work loses it: say so
                                 if( s_headless && s_headless->IsContentModified() )
                                     wxLogWarning( "Unsaved changes to %s discarded (headless server released the board)",
                                                   s_headless->GetCurrentFileName() );

                                 s_headless.reset();
                             } );

KOPENAPI_REGISTER( "pcb_open",
                   "Open a board (.kicad_pcb, a KiCad 4 .brd, or its .kicad_pro / .pro; a .brd is saved as .kicad_pcb): editor window in the GUI, in memory headless",
                   R"json({"type":"object","required":["path"],"properties":{
                        "path":{"type":"string"},
                        "discard":{"type":"boolean","default":false,
                                   "description":"Drop unsaved changes of the currently open board; headless, also reread the .kicad_pro from disk (unless a schematic of this process uses the project)"},
                        "switch_project":{"type":"boolean","default":false,
                                   "description":"Headless: when the open schematic belongs to another project, close it first (refused with its unsaved changes unless discard: true)"},
                        "override_lock":{"type":"boolean","default":false,
                                   "description":"GUI: open even when another process has the file open (never asked in a dialog; refused with a journal error otherwise)"}}})json"_json,
                   false, h_pcb_open, 120 );

KOPENAPI_REGISTER( "pcb_close", "Close the open board: refuses with unsaved changes unless discard: true, which drops "
                   "them without any dialog (a warning goes to the journal); GUI: the editor window closes",
                   R"json({"type":"object","properties":{"discard":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_close );

KOPENAPI_REGISTER( "pcb_save",
                   "Save the open board; with 'path' save as (headless switches to it, GUI writes a copy). "
                   "With 'version' (e.g. \"10.0\") write a copy at 'path' that an earlier KiCad opens, with its "
                   "project and rules; the open board stays as it is. The result lists what the earlier format "
                   "cannot hold (dropped or approximated)",
                   nlohmann::json( { { "type", "object" },
                                     { "properties",
                                       { { "path", { { "type", "string" } } },
                                         { "version",
                                           { { "type", "string" },
                                             { "enum", KopenapiLegacyTargets() },
                                             { "description", "earlier KiCad release to write for; needs 'path'" } } } } } } ),
                   false, h_pcb_save );

KOPENAPI_REGISTER( "pcb_revert", "Reload the open board from disk, dropping unsaved changes",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_revert, 120 );
