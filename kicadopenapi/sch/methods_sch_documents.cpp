/// @file methods_sch_documents.cpp
/// @brief kicadopenapi schematic document methods: sch_open, sch_close, sch_save, sch_revert, and
/// the "sch" document provider.  Compiled into the eeschema kiface; registered when it loads.
#include <kicadopenapi_util.h>
#include <kicadopenapi_change_feed.h>
#include <model_thread.h>
#include <kicadopenapi_legacy_save.h>
#include <kicadopenapi_lock.h>
#include <kicadopenapi_service.h>
#include "kopenapi_sch.h"

#include <wx/log.h>

#include <api/headless_sch_context.h>
#include <api/sch_api_save.h>
#include <api/sch_context.h>
#include <eeschema_helpers.h>
#include <local_history.h>
#include <sch_autosave_recovery.h>
#include <settings/common_settings.h>
#include <kiway.h>
#include <tool/tool_manager.h>
#include <tool/actions.h>
#include <sch_screen.h>
#include <kicadopenapi_history.h>
#include <kicadopenapi_undo.h>
#include <pgm_base.h>
#include <project.h>
#include <sch_edit_frame.h>
#include <sch_sheet_path.h>
#include <schematic.h>
#include <settings/settings_manager.h>
#include <wildcards_and_files_ext.h>

#include <set>

#include <wx/ffile.h>
#include <wx/filename.h>


/// @brief Headless schematic: the context reads it through this slot (RevertToSaved replaces it)
static SCHEMATIC*                            s_schematic = nullptr;
static std::shared_ptr<HEADLESS_SCH_CONTEXT> s_headless;


static void releaseHeadless()
{
    // Stopping a headless server with unsaved work loses it: leave a trace in the journal
    if( s_headless && s_schematic && s_schematic->HasHierarchy() && s_schematic->Hierarchy().IsModified() )
        wxLogWarning( "Unsaved changes to %s discarded (headless server released the schematic)",
                      s_headless->GetCurrentFileName() );

    s_headless.reset();

    // the project's ERC / schematic settings belong to the schematic that installed them
    if( s_schematic )
        s_schematic->SetProject( nullptr );

    delete s_schematic;
    s_schematic = nullptr;
}


static SCH_EDIT_FRAME* guiFrame( KOPENAPI_CONTEXT& aCtx, bool aCreate )
{
    if( aCtx.headless || !aCtx.kiway )
        return nullptr;

    return static_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, aCreate ) );
}


/// @brief Records every commit of the schematic in the change feed (owner thread); a commit
/// reports its added, removed and changed items as separate change sets
class SCHEMATIC_CHANGE_FEED : public SCHEMATIC_LISTENER
{
public:
    void OnSchItemsAdded( SCHEMATIC&, std::vector<SCH_ITEM*>& aItems ) override
    {
        KOPENAPI_CHANGE_FEED::Record( "sch", ids( aItems ), {}, {} );
    }

    void OnSchItemsRemoved( SCHEMATIC&, std::vector<SCH_ITEM*>& aItems ) override
    {
        KOPENAPI_CHANGE_FEED::Record( "sch", {}, {}, ids( aItems ) );
    }

    void OnSchItemsChanged( SCHEMATIC&, std::vector<SCH_ITEM*>& aItems ) override
    {
        KOPENAPI_CHANGE_FEED::Record( "sch", {}, ids( aItems ), {} );
    }

    /// @brief Listens to this schematic; a different one than before resets the feed
    void Follow( SCHEMATIC* aSchematic )
    {
        if( !aSchematic || !MODEL_THREAD::MayWrite() )
            return;

        aSchematic->AddListener( this );

        if( aSchematic != m_schematic )
        {
            m_schematic = aSchematic;
            KOPENAPI_CHANGE_FEED::Reset( "sch" );
        }
    }

private:
    static std::vector<std::string> ids( const std::vector<SCH_ITEM*>& aItems )
    {
        std::vector<std::string> out;
        out.reserve( aItems.size() );

        for( const SCH_ITEM* item : aItems )
            out.push_back( item->m_Uuid.AsStdString() );

        return out;
    }

    SCHEMATIC* m_schematic = nullptr;
};


static SCHEMATIC_CHANGE_FEED& schematicChangeFeed()
{
    static SCHEMATIC_CHANGE_FEED* s_feed = new SCHEMATIC_CHANGE_FEED();   // schematics keep a pointer to it
    return *s_feed;
}


std::shared_ptr<SCH_CONTEXT> KopenapiSchContext( KOPENAPI_CONTEXT& aCtx )
{
    std::shared_ptr<SCH_CONTEXT> context;

    if( aCtx.headless )
    {
        context = s_headless;
    }
    else if( SCH_EDIT_FRAME* frame = guiFrame( aCtx, false ) )
    {
        context = CreateSchFrameContext( frame );

        if( context->GetCurrentFileName().IsEmpty() )
            context = nullptr;
    }

    if( context )
        schematicChangeFeed().Follow( context->GetSchematic() );

    return context;
}


KOPENAPI_RESULT KopenapiNoSchematic()
{
    return KOPENAPI_RESULT::Error( 409, "no schematic is open (sch_open)" );
}


static bool isModified( const SCH_CONTEXT& aContext )
{
    SCHEMATIC* schematic = aContext.GetSchematic();
    return schematic && schematic->HasHierarchy() && schematic->Hierarchy().IsModified();
}


static void clearModified( const SCH_CONTEXT& aContext )
{
    if( SCHEMATIC* schematic = aContext.GetSchematic(); schematic && schematic->HasHierarchy() )
        schematic->Hierarchy().ClearModifyStatus();
}


static nlohmann::json documentStatus( KOPENAPI_CONTEXT& aCtx )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return nullptr;

    return { { "domain", "sch" },
             { "path", context->GetCurrentFileName().ToStdString() },
             { "project", context->Prj().GetProjectFullName().ToStdString() },
             { "unsaved", isModified( *context ) },
             { "mode", aCtx.headless ? "headless" : "gui" } };
}


/// @brief Another open document of this process belongs to a different project
static std::string projectConflict( KOPENAPI_CONTEXT& aCtx, const wxFileName& aProject )
{
    for( const nlohmann::json& doc : KOPENAPI_REGISTRY::Get().Documents( aCtx ) )
    {
        if( doc.value( "domain", "" ) == "sch" )
            continue;

        const wxString other = wxString::FromUTF8( doc.value( "project", "" ) );

        // canonical on both sides: a symbolic link or "." in a path is the same project
        if( !other.IsEmpty() && KopenapiCanonicalPath( other ) != KopenapiCanonicalPath( aProject.GetFullPath() ) )
            return doc.value( "project", "" );
    }

    return {};
}


static KOPENAPI_RESULT openHeadless( KOPENAPI_CONTEXT& aCtx, const wxFileName& aSchematic, const nlohmann::json& aArgs )
{
    wxFileName projectFile( aSchematic );
    projectFile.SetExt( FILEEXT::ProjectFileExtension );

    if( std::string other = projectConflict( aCtx, projectFile ); !other.empty() )
    {
        if( !aArgs.value( "switch_project", false ) )
        {
            return KOPENAPI_RESULT::Error( 409, "this process serves project " + other
                                                        + "; pcb_close first, pass switch_project: true, or start "
                                                          "another instance for a different project" );
        }

        // the other project's board goes first, as pcb_close would close it
        std::optional<KOPENAPI_METHOD> close = KOPENAPI_REGISTRY::Get().Find( "pcb_close" );

        if( !close )
            return KOPENAPI_RESULT::Error( 500, "pcb_close is not available in this process" );

        KOPENAPI_RESULT closed = close->handler( aCtx, { { "discard", aArgs.value( "discard", false ) } } );

        if( closed.status >= 400 )
            return closed;
    }

    // Drop the current schematic first: activating another project destroys the old PROJECT,
    // which the old schematic still references (unsaved changes were checked by the caller)
    releaseHeadless();

    SETTINGS_MANAGER& settings = Pgm().GetSettingsManager();
    PROJECT*          project = settings.GetProject( projectFile.GetFullPath() );

    if( !project )
    {
        settings.LoadProject( projectFile.GetFullPath(), true );
        project = settings.GetProject( projectFile.GetFullPath() );
    }

    if( !project )
        return KOPENAPI_RESULT::Error( 500, "could not load the project for this schematic" );

    SCHEMATIC* schematic = nullptr;

    try
    {
        schematic = EESCHEMA_HELPERS::LoadSchematic( aSchematic.GetFullPath(), false, false, project );
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 422, "schematic could not be loaded: " + ioe.What().ToStdString() );
    }
    catch( ... )
    {
        schematic = nullptr;
    }

    if( !schematic )
        return KOPENAPI_RESULT::Error( 422, "schematic could not be loaded (the cause, if KiCad gave one, is in error.log)" );

    s_schematic = schematic;
    s_headless = std::make_shared<HEADLESS_SCH_CONTEXT>( &s_schematic, project, aCtx.kiway );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT openGui( KOPENAPI_CONTEXT& aCtx, const wxFileName& aSchematic )
{
    SCH_EDIT_FRAME* frame = guiFrame( aCtx, true );

    if( !frame )
        return KOPENAPI_RESULT::Error( 500, "could not create the schematic editor window" );

    // another process holds it (or its project) open: refused here, never asked in a dialog
    if( !KICAD_OPENAPI_SERVICE::OverrideLock() && frame->GetCurrentFileName() != aSchematic.GetFullPath() )
    {
        const wxString ownProject = aCtx.kiway ? aCtx.kiway->Prj().GetProjectFullName() : wxString();

        if( const std::string owner = KopenapiLockedBy( aSchematic, ownProject ); !owner.empty() )
        {
            return KOPENAPI_RESULT::Error( 409, "the schematic or its project is open in another process (" + owner
                                                        + "): close it there, or open with override_lock: true" );
        }
    }

    // the same file again (a reload): KiCad would find its own lock on it and refuse
    const bool same = frame->GetCurrentFileName() == aSchematic.GetFullPath();

    if( same )
        frame->ReleaseFile();

    if( !frame->OpenProjectFiles( { aSchematic.GetFullPath() } ) )
    {
        if( same )
            frame->LockFile( aSchematic.GetFullPath() );

        return KOPENAPI_RESULT::Error( 422, "the schematic editor could not open the schematic" );
    }

    // OpenProjectFiles does not show the frame; mirror KICAD_MANAGER_CONTROL::ShowPlayer
    frame->Iconize( false );
    frame->Show( true );
    frame->Raise();

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


/// @brief The schematic's auto-saved copies newer than their files (the ones the editor offers on open)
static nlohmann::json newerAutosaves( KOPENAPI_CONTEXT& aCtx )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );
    nlohmann::json               out = nlohmann::json::array();

    if( !context || !context->GetSchematic() || !aCtx.kiway )
        return out;

    for( const AUTOSAVE_FILE_INFO& info : FindSchematicAutosaves( aCtx.kiway->LocalHistory(), *context->GetSchematic() ) )
    {
        if( info.newer )
            out.push_back( KopenapiAutosaveJson( info ) );
    }

    return out;
}


static KOPENAPI_RESULT h_sch_open( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_HISTORY::Get().Reset( "sch", "sch_open" );   // earlier marks belong to another document state

    if( !aArgs["path"].is_string() || aArgs["path"].get<std::string>().empty() )
        return KOPENAPI_RESULT::Error( 400, "'path' must be a non-empty string" );

    // one spelling per file (symbolic links, "."): the project is found by this path
    wxFileName schematic( KopenapiCanonicalPath( wxString::FromUTF8( aArgs["path"].get<std::string>() ) ) );
    schematic.MakeAbsolute();

    if( schematic.GetExt() == FILEEXT::ProjectFileExtension || schematic.GetExt() == FILEEXT::LegacyProjectFileExtension )
    {
        schematic.SetExt( FILEEXT::KiCadSchematicFileExtension );

        // a KiCad 5 project keeps its schematic in the legacy format
        wxFileName legacy( schematic );
        legacy.SetExt( FILEEXT::LegacySchematicFileExtension );

        if( !schematic.FileExists() && legacy.FileExists() )
            schematic = legacy;
    }

    if( schematic.GetExt() != FILEEXT::KiCadSchematicFileExtension
        && schematic.GetExt() != FILEEXT::LegacySchematicFileExtension )
    {
        return KOPENAPI_RESULT::Error( 400, "expected a .kicad_sch, a KiCad 5 .sch, or a .kicad_pro / .pro file" );
    }

    if( !schematic.FileExists() )
        return KOPENAPI_RESULT::Error( 404, "file not found: " + schematic.GetFullPath().ToStdString() );

    // Never let a "save changes?" dialog or a silent drop happen: unsaved work blocks
    if( std::shared_ptr<SCH_CONTEXT> current = KopenapiSchContext( aCtx ) )
    {
        if( isModified( *current ) )
        {
            if( !aArgs.value( "discard", false ) )
                return KOPENAPI_RESULT::Error( 409, "the open schematic has unsaved changes; sch_save, "
                                                    "sch_revert, or pass discard: true" );

            clearModified( *current );
        }
    }

    KOPENAPI_RESULT result;

    if( aCtx.headless )
    {
        result = openHeadless( aCtx, schematic, aArgs );
    }
    else
    {
        // a file another process has open: opened only when asked (no modal question)
        KICAD_OPENAPI_SERVICE::SetOverrideLock( aArgs.value( "override_lock", false ) );
        result = openGui( aCtx, schematic );
        KICAD_OPENAPI_SERVICE::SetOverrideLock( false );

        if( result.status != 200 )
            result.body["hint"] = "another process may have the file open (errors: journal); override_lock: true opens it anyway";
    }

    // the editor offers these in its infobar
    if( result.status == 200 )
    {
        if( nlohmann::json newer = newerAutosaves( aCtx ); !newer.empty() )
            result.body["autosaves"] = newer;
    }

    return result;
}


static KOPENAPI_RESULT h_sch_close( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    KOPENAPI_HISTORY::Get().Reset( "sch", "sch_close" );   // earlier marks belong to another document state

    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( isModified( *context ) )
    {
        if( !aArgs.value( "discard", false ) )
            return KOPENAPI_RESULT::Error( 409, "the schematic has unsaved changes; sch_save or pass discard: true" );

        wxLogWarning( "Unsaved changes to %s discarded (sch_close discard)", context->GetCurrentFileName() );
        clearModified( *context );
    }

    nlohmann::json closed = documentStatus( aCtx );
    context.reset();

    if( aCtx.headless )
        releaseHeadless();
    else if( SCH_EDIT_FRAME* frame = guiFrame( aCtx, false ) )
        frame->Close( true );

    return KOPENAPI_RESULT::Ok( { { "closed", closed } } );
}


/// @brief As the editor does on save: KiCad 5 sheets (.sch, no writer) are written in the current
/// format under .kicad_sch names next to them; the legacy files stay as they are.
/// @return the new file names
static nlohmann::json renameLegacySheets( SCHEMATIC& aSchematic )
{
    nlohmann::json renamed = nlohmann::json::array();
    SCH_SCREENS    screens( aSchematic.Root() );

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        for( SCH_ITEM* item : screen->Items().OfType( SCH_SHEET_T ) )
        {
            SCH_SHEET* sheet = static_cast<SCH_SHEET*>( item );
            wxFileName name( sheet->GetFileName() );

            if( name.IsOk() && name.GetExt() == FILEEXT::LegacySchematicFileExtension )
            {
                name.SetExt( FILEEXT::KiCadSchematicFileExtension );
                sheet->SetFileName( name.GetFullPath() );
            }
        }

        wxFileName file( screen->GetFileName() );

        if( file.IsOk() && file.GetExt() == FILEEXT::LegacySchematicFileExtension )
        {
            file.SetExt( FILEEXT::KiCadSchematicFileExtension );
            screen->SetFileName( file.GetFullPath() );
            renamed.push_back( std::string( file.GetFullPath().ToUTF8() ) );
        }
    }

    return renamed;
}


/// @brief Point the copied project's top-level sheet entries at the copy's root file
static void retargetProjectRoot( const wxString& aProjectFile, const wxString& aOldRoot, const wxString& aNewRoot )
{
    wxFFile in( aProjectFile, wxS( "rb" ) );
    wxString text;

    if( !in.IsOpened() || !in.ReadAll( &text, wxConvUTF8 ) )
        return;

    in.Close();

    nlohmann::json pro = nlohmann::json::parse( text.ToStdString( wxConvUTF8 ), nullptr, false );

    if( pro.is_discarded() || !pro.contains( "schematic" ) || !pro["schematic"].contains( "top_level_sheets" ) )
        return;

    bool changed = false;

    for( nlohmann::json& sheet : pro["schematic"]["top_level_sheets"] )
    {
        if( sheet.value( "filename", std::string() ) == std::string( aOldRoot.ToUTF8() ) )
        {
            sheet["filename"] = std::string( aNewRoot.ToUTF8() );
            changed = true;
        }
    }

    if( !changed )
        return;

    std::string out = pro.dump( 2 ) + "\n";
    wxFFile     file( aProjectFile, wxS( "wb" ) );

    if( file.IsOpened() )
        file.Write( out.data(), out.size() );
}


/// @brief A copy of every sheet (root at aTarget, the others at their paths relative to it) and of
/// the project, for an earlier KiCad. The open document keeps its files and modified state.
static KOPENAPI_RESULT saveVersionCopy( KOPENAPI_CONTEXT& aCtx, SCH_CONTEXT& aContext, const wxFileName& aTarget,
                                        const std::string& aVersion )
{
    SCHEMATIC* schematic = aContext.GetSchematic();

    if( !schematic || !schematic->RootScreen() )
        return KopenapiNoSchematic();

    renameLegacySheets( *schematic );

    wxFileName root( schematic->RootScreen()->GetFileName() );
    root.MakeAbsolute();
    const wxString sourceDir = root.GetPath();

    SCH_SCREENS                                  screens( schematic->Root() );
    std::vector<std::pair<size_t, wxString>>     plan;
    std::set<wxString>                           openFiles;

    for( size_t ii = 0; ii < screens.GetCount(); ++ii )
    {
        wxFileName file( screens.GetScreen( ii )->GetFileName() );

        if( file.IsOk() && file.HasName() )
        {
            file.MakeAbsolute( sourceDir );
            openFiles.insert( file.GetFullPath() );
        }
    }

    for( size_t ii = 0; ii < screens.GetCount(); ++ii )
    {
        SCH_SCREEN* screen = screens.GetScreen( ii );
        wxFileName  file( screen->GetFileName() );

        if( !file.IsOk() || !file.HasName() )
            continue;

        file.MakeAbsolute( sourceDir );
        wxFileName dest( aTarget );

        if( file.GetFullPath() != root.GetFullPath() )
        {
            file.MakeRelativeTo( sourceDir );
            dest = wxFileName( file.GetFullPath() );
            dest.MakeAbsolute( aTarget.GetPath() );
        }

        if( openFiles.count( dest.GetFullPath() ) )
        {
            return KOPENAPI_RESULT::Error( 409, "the copy would replace the open schematic's file "
                                                        + std::string( dest.GetFullPath().ToUTF8() )
                                                        + "; use another folder" );
        }

        plan.emplace_back( ii, dest.GetFullPath() );
    }

    wxFileName project( aTarget );
    project.SetExt( FILEEXT::ProjectFileExtension );

    if( project.GetFullPath() == wxFileName( aContext.Prj().GetProjectFullName() ).GetFullPath() )
        return KOPENAPI_RESULT::Error( 409, "the copy's project file would replace the open project; use another folder" );

    nlohmann::json files = nlohmann::json::array();

    for( const auto& [index, dest] : plan )
    {
        if( !SCH_API_SAVE::SaveSheetToFile( screens.GetSheet( index ), *schematic, dest ) )
            return KOPENAPI_RESULT::Error( 500, "writing " + std::string( dest.ToUTF8() ) + " failed" );

        nlohmann::json report;
        wxString       error;

        if( !KopenapiLegacyConvertFile( dest, aVersion, report, error ) )
            return KOPENAPI_RESULT::Error( 500, std::string( error.ToUTF8() ) );

        files.push_back( report );
    }

    if( !Pgm().GetSettingsManager().SaveProjectCopy( project.GetFullPath(), &aContext.Prj() ) || !project.FileExists() )
        return KOPENAPI_RESULT::Error( 500, "writing the project copy failed" );

    retargetProjectRoot( project.GetFullPath(), root.GetFullName(), aTarget.GetFullName() );

    nlohmann::json projectReport;
    wxString       projectError;

    if( !KopenapiLegacyConvertProject( project.GetFullPath(), aVersion, nullptr, projectReport, projectError ) )
        return KOPENAPI_RESULT::Error( 500, std::string( projectError.ToUTF8() ) );

    files.push_back( projectReport );

    nlohmann::json status = documentStatus( aCtx );
    status["copy"] = { { "path", std::string( aTarget.GetFullPath().ToUTF8() ) },
                       { "version", aVersion },
                       { "project", std::string( project.GetFullPath().ToUTF8() ) },
                       { "files", files } };
    return KOPENAPI_RESULT::Ok( status );
}


static KOPENAPI_RESULT h_sch_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::string version;
    std::string versionError;

    if( !KopenapiLegacyTarget( aArgs, version, versionError ) )
        return KOPENAPI_RESULT::Error( 400, versionError );

    if( !version.empty() )
    {
        if( !aArgs.contains( "path" ) || !aArgs["path"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "'version' writes a copy: 'path' (.kicad_sch) is required" );

        wxFileName target( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
        target.MakeAbsolute();

        if( target.GetExt() != FILEEXT::KiCadSchematicFileExtension )
            return KOPENAPI_RESULT::Error( 400, "expected a .kicad_sch path" );

        if( !target.DirExists() && !wxFileName::Mkdir( target.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
            return KOPENAPI_RESULT::Error( 500, "cannot create " + std::string( target.GetPath().ToUTF8() ) );

        return saveVersionCopy( aCtx, *context, target, version );
    }

    const bool asCopy = !aCtx.headless && aArgs.contains( "path" ) && aArgs["path"].is_string();
    bool       legacy = false;

    // a KiCad 5 sheet is written as .kicad_sch beside it, as the editor saves it: an existing file
    // of that name is replaced only when asked (the editor asks)
    if( !asCopy )
    {
        std::vector<wxString> sheetFiles;
        SCH_SCREENS           screens( context->GetSchematic()->Root() );

        for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
        {
            if( screen->GetFileName().IsEmpty() )
                continue;

            sheetFiles.push_back( screen->GetFileName() );
            legacy |= wxFileName( screen->GetFileName() ).GetExt() == FILEEXT::LegacySchematicFileExtension;
        }

        const std::vector<wxString> overwritten = SCH_API_SAVE::LegacySaveOverwrites( sheetFiles );

        if( !overwritten.empty() && !aArgs.value( "overwrite", false ) )
        {
            std::string names;

            for( const wxString& file : overwritten )
                names += ( names.empty() ? "" : ", " ) + std::string( file.ToUTF8() );

            return KOPENAPI_RESULT::Error( 409, "saving writes the KiCad 5 sheets as .kicad_sch and would overwrite "
                                                "existing files: " + names + "; pass overwrite: true" );
        }
    }

    // the editor's save converts the sheets and keeps its window (title, lock, history) in step
    if( legacy && !aCtx.headless && !aArgs.contains( "path" ) )
    {
        SCH_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame )
            return KopenapiNoSchematic();

        std::vector<wxString> before;
        SCH_SCREENS           screens( frame->Schematic().Root() );

        for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
            before.push_back( screen->GetFileName() );

        KICAD_OPENAPI_SERVICE::SetConfirmPrompts( aArgs.value( "overwrite", false ) );
        const bool saved = frame->SaveProject();
        KICAD_OPENAPI_SERVICE::SetConfirmPrompts( false );

        if( !saved )
            return KOPENAPI_RESULT::Error( 500, "saving the schematic failed (errors: journal)" );

        nlohmann::json status = documentStatus( aCtx );
        nlohmann::json renamed = nlohmann::json::array();

        for( const wxString& file : before )
        {
            if( wxFileName( file ).GetExt() == FILEEXT::LegacySchematicFileExtension )
            {
                wxFileName current( file );
                current.SetExt( FILEEXT::KiCadSchematicFileExtension );
                renamed.push_back( std::string( current.GetFullPath().ToUTF8() ) );
            }
        }

        status["converted_from_legacy"] = renamed;
        return KOPENAPI_RESULT::Ok( status );
    }

    nlohmann::json converted = aCtx.headless && s_schematic ? renameLegacySheets( *s_schematic )
                                                             : nlohmann::json::array();
    bool           ok = false;

    if( aArgs.contains( "path" ) && aArgs["path"].is_string() )
    {
        wxFileName target( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
        target.MakeAbsolute();

        if( target.GetExt() != FILEEXT::KiCadSchematicFileExtension )
            return KOPENAPI_RESULT::Error( 400, "expected a .kicad_sch path" );

        // Save As switches the document (headless); GUI hosts write a copy and keep editing
        ok = aCtx.headless ? context->SaveSchematicAs( target.GetFullPath() )
                           : context->SaveSchematicCopy( target.GetFullPath(), false );
    }
    else
    {
        ok = context->SaveSchematic();
    }

    if( !ok )
        return KOPENAPI_RESULT::Error( 500, "saving the schematic failed" );

    nlohmann::json status = documentStatus( aCtx );

    if( !converted.empty() )
        status["converted_from_legacy"] = converted;

    return KOPENAPI_RESULT::Ok( status );
}


static KOPENAPI_RESULT h_sch_revert( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    KOPENAPI_HISTORY::Get().Reset( "sch", "sch_revert" );   // earlier marks belong to another document state

    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !context->RevertToSaved() )
        return KOPENAPI_RESULT::Error( 422, "reverting failed (no saved file?)" );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


// ---- edit history (kicadopenapi_history.h) --------------------------------------------------

/// @brief Headless: load a copy of the schematic's files (root sheet aRootCopy, the other sheets
/// beside it as in the schematic) as the open schematic, under its own paths, unsaved
static bool loadHeadlessContent( KOPENAPI_CONTEXT& aCtx, const wxString& aRootCopy )
{
    if( !s_headless || !s_schematic )
        return false;

    const wxFileName original( s_schematic->GetFileName() );
    const wxFileName copy( aRootCopy );
    PROJECT*         project = &s_headless->Prj();
    SCHEMATIC*       loaded = nullptr;

    // the loaded copy installs its own ERC / schematic settings in the project: release the open
    // schematic's first (they would leak), and give them back if the copy does not load
    s_schematic->SetProject( nullptr );

    try
    {
        // read from the copy's directory under the schematic's own names: the project's
        // top-level sheets are found there too
        loaded = EESCHEMA_HELPERS::LoadSchematic( original.GetFullPath(),
                                                  SCH_IO_MGR::GuessPluginTypeFromSchPath( copy.GetFullPath() ),
                                                  false, false, project, true, nullptr, copy.GetPath() );
    }
    catch( ... )
    {
        loaded = nullptr;
    }

    if( !loaded )
    {
        s_schematic->SetProject( project );
        return false;
    }

    RetargetSchematicFiles( *loaded, copy.GetPath(), original.GetPath(), true );

    s_headless.reset();
    delete s_schematic;
    s_schematic = loaded;
    s_headless = std::make_shared<HEADLESS_SCH_CONTEXT>( &s_schematic, project, aCtx.kiway );
    return true;
}


/// @brief Headless: load the copy written into aDir as the open schematic, under its own paths, unsaved
static bool restoreHeadless( KOPENAPI_CONTEXT& aCtx, const std::string& aDir )
{
    if( !s_schematic )
        return false;

    const wxFileName copy( wxString::FromUTF8( aDir ), wxFileName( s_schematic->GetFileName() ).GetFullName() );
    return loadHeadlessContent( aCtx, copy.GetFullPath() );
}


// ---- auto-saved copies ----------------------------------------------------------------------

static KOPENAPI_RESULT h_sch_autosave_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context || !context->GetSchematic() )
        return KopenapiNoSchematic();

    if( !aCtx.kiway )
        return KOPENAPI_RESULT::Error( 409, "autosave files are not available in this host" );

    nlohmann::json list = nlohmann::json::array();

    for( const AUTOSAVE_FILE_INFO& info : FindSchematicAutosaves( aCtx.kiway->LocalHistory(), *context->GetSchematic() ) )
        list.push_back( KopenapiAutosaveJson( info ) );

    const COMMON_SETTINGS* cs = Pgm().GetCommonSettings();

    return KOPENAPI_RESULT::Ok(
            { { "autosaves", list },
              { "enabled", cs->m_Backup.enabled },
              { "format", cs->m_Backup.format == BACKUP_FORMAT::ZIP ? "zip" : "incremental" },
              { "location", cs->m_Backup.location == BACKUP_LOCATION::USER_DIR ? "user_dir" : "project_dir" },
              { "writes_autosave_files", cs->m_Backup.enabled && !cs->AutosaveUsesLocalHistory() } } );
}


/// @brief Rescue: the sheets' copies replace the open schematic under its own file names, unsaved
static bool rescueSchematic( KOPENAPI_CONTEXT& aCtx, const std::vector<AUTOSAVE_FILE_INFO>& aChosen,
                             std::string& aError )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );
    const wxString               dir = MakeSchematicStagingDir();
    wxString                     error;

    if( dir.IsEmpty() )
    {
        aError = "no temporary directory for the copies";
        return false;
    }

    const wxString root = StageSchematicAutosaves( *context->GetSchematic(), aChosen, dir, error );
    bool           ok = false;

    if( root.IsEmpty() )
    {
        aError = std::string( error.utf8_str() );
    }
    else if( aCtx.headless )
    {
        ok = loadHeadlessContent( aCtx, root );
    }
    else if( SCH_EDIT_FRAME* frame = guiFrame( aCtx, false ) )
    {
        ok = frame->LoadSchematicContentFrom( root );
    }

    if( !ok && aError.empty() )
        aError = "the auto-saved copies could not be loaded";

    wxFileName::Rmdir( dir, wxPATH_RMDIR_RECURSIVE );
    return ok;
}


/// Rescue and the choices of the editor's autosave infobar on open (EDA_BASE_FRAME::RecoverAutosaveFiles),
/// on the schematic's auto-saved copies
static KOPENAPI_RESULT h_sch_autosave( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context || !context->GetSchematic() )
        return KopenapiNoSchematic();

    if( !aCtx.kiway )
        return KOPENAPI_RESULT::Error( 409, "autosave files are not available in this host" );

    const std::string action = aArgs.value( "action", std::string() );

    if( action != "rescue" && action != "restore" && action != "keep_both" && action != "discard" )
        return KOPENAPI_RESULT::Error( 400, "action: rescue, restore, keep_both or discard" );

    std::vector<AUTOSAVE_FILE_INFO> all = FindSchematicAutosaves( aCtx.kiway->LocalHistory(), *context->GetSchematic() );
    std::vector<AUTOSAVE_FILE_INFO> chosen;

    if( aArgs.contains( "path" ) )
    {
        const wxFileName want( wxString::FromUTF8( aArgs.value( "path", std::string() ) ) );

        for( const AUTOSAVE_FILE_INFO& info : all )
        {
            if( wxFileName( info.autosavePath ).SameAs( want ) )
                chosen.push_back( info );
        }

        if( chosen.empty() )
            return KOPENAPI_RESULT::Error( 404, "path: not an auto-saved copy of this schematic (sch_autosave_list)" );
    }
    else if( action == "rescue" )
    {
        // the newest copy of each sheet file
        std::set<wxString> sources;

        for( const AUTOSAVE_FILE_INFO& info : all )
        {
            if( sources.insert( wxFileName( info.sourcePath ).GetFullPath() ).second )
                chosen.push_back( info );
        }
    }
    else if( action == "discard" )
    {
        chosen = all;
    }
    else
    {
        for( const AUTOSAVE_FILE_INFO& info : all )
        {
            if( info.newer )
                chosen.push_back( info );
        }
    }

    if( chosen.empty() )
        return KOPENAPI_RESULT::Error( 404, "no auto-saved copy of this schematic" + std::string( action == "rescue" || action == "discard" ? "" : " newer than its files" ) + " (sch_autosave_list)" );

    const bool replaces = action == "rescue" || action == "restore";

    if( replaces && isModified( *context ) && !aArgs.value( "discard", false ) )
        return KOPENAPI_RESULT::Error( 409, "the schematic has unsaved changes, which " + action + " drops; sch_save, or pass discard: true" );

    nlohmann::json used = nlohmann::json::array();

    for( const AUTOSAVE_FILE_INFO& info : chosen )
        used.push_back( KopenapiAutosaveJson( info ) );

    if( action == "rescue" )
    {
        KOPENAPI_HISTORY::Get().Reset( "sch", "sch_autosave" );   // another document state

        std::string error;

        if( !rescueSchematic( aCtx, chosen, error ) )
            return KOPENAPI_RESULT::Error( 422, error );

        nlohmann::json out = documentStatus( aCtx );
        out["action"] = action;
        out["autosaves"] = used;
        return KOPENAPI_RESULT::Ok( out );
    }

    AUTOSAVE_RECOVERY_CHOICE choice = action == "restore"     ? AUTOSAVE_RECOVERY_CHOICE::RESTORE
                                      : action == "keep_both" ? AUTOSAVE_RECOVERY_CHOICE::KEEP_BOTH
                                                              : AUTOSAVE_RECOVERY_CHOICE::KEEP_CURRENT;

    std::vector<std::pair<wxString, wxString>> pairs;

    for( const AUTOSAVE_FILE_INFO& info : chosen )
        pairs.emplace_back( info.autosavePath, info.sourcePath );

    std::vector<wxString> written;
    const wxString        errors = LOCAL_HISTORY::ResolveAutosaveFiles( pairs, choice, &written );
    bool                  reloaded = false;

    if( choice == AUTOSAVE_RECOVERY_CHOICE::RESTORE && !written.empty() )
    {
        KOPENAPI_HISTORY::Get().Reset( "sch", "sch_autosave" );
        clearModified( *context );
        reloaded = context->RevertToSaved();
    }

    nlohmann::json files = nlohmann::json::array();

    for( const wxString& file : written )
        files.push_back( std::string( file.utf8_str() ) );

    nlohmann::json out = documentStatus( aCtx );
    out["action"] = action;
    out["autosaves"] = used;
    out["written"] = files;

    if( choice == AUTOSAVE_RECOVERY_CHOICE::RESTORE )
        out["reloaded"] = reloaded;

    if( !errors.IsEmpty() )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 500, std::string( errors.utf8_str() ) );
        r.body.update( out );
        return r;
    }

    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_DOC_HISTORY history()
{
    KOPENAPI_DOC_HISTORY h;

    h.undoMark = []( KOPENAPI_CONTEXT& aCtx ) -> std::optional<KOPENAPI_UNDO_MARK>
    {
        SCH_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame || !KopenapiSchContext( aCtx ) )
            return std::nullopt;

        return KopenapiUndoMark( *frame );
    };

    h.undoTarget = []( KOPENAPI_CONTEXT& aCtx, const KOPENAPI_UNDO_MARK& aMark,
                       std::string& aWhy ) -> std::optional<int>
    {
        SCH_EDIT_FRAME* frame = guiFrame( aCtx, false );

        if( !frame )
        {
            aWhy = "the editor window is closed";
            return std::nullopt;
        }

        return KopenapiUndoTarget( *frame, aMark, aWhy );
    };

    h.undoTo = []( KOPENAPI_CONTEXT& aCtx, int aDepth )
    {
        SCH_EDIT_FRAME* frame = guiFrame( aCtx, false );

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

    h.snapshot = []( KOPENAPI_CONTEXT& aCtx, const std::string& aDir )
    {
        if( !aCtx.headless || !s_headless || !s_schematic )
            return false;

        // Every sheet file of the hierarchy under the same relative name (KiCad's
        // SaveSchematicCopy writes the root sheet only)
        const wxFileName root( s_schematic->GetFileName() );
        SCH_SCREENS      screens( s_schematic->Root() );
        bool             ok = screens.GetCount() > 0;

        for( size_t i = 0; i < screens.GetCount(); i++ )
        {
            SCH_SCREEN* screen = screens.GetScreen( i );
            SCH_SHEET*  sheet = screens.GetSheet( i );

            if( !screen || !sheet || wxFileName( screen->GetFileName() ).GetFullName().IsEmpty() )
                continue;   // the virtual root's screen has no file

            wxFileName target( screen->GetFileName() );
            target.MakeRelativeTo( root.GetPath() );
            target.MakeAbsolute( wxString::FromUTF8( aDir ) );

            if( !target.DirExists() )
                target.Mkdir( wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL );

            ok &= SCH_API_SAVE::SaveSheetToFile( sheet, *s_schematic, target.GetFullPath() );
        }

        return ok;
    };

    h.restore = []( KOPENAPI_CONTEXT& aCtx, const std::string& aDir )
    {
        return aCtx.headless && restoreHeadless( aCtx, aDir );
    };

    return h;
}


KOPENAPI_REGISTER_DOCUMENTS( "sch", documentStatus, releaseHeadless );
KOPENAPI_REGISTER_HISTORY( "sch", history() );

KOPENAPI_REGISTER( "sch_open",
                   "Open a schematic (.kicad_sch, a KiCad 5 .sch, or its .kicad_pro / .pro; a KiCad 5 schematic is saved as .kicad_sch): editor window in the GUI, in memory headless. "
                   "autosaves lists auto-saved copies of its sheets newer than their files (the editor offers them in its infobar; sch_autosave)",
                   R"json({"type":"object","required":["path"],"properties":{
                        "path":{"type":"string"},
                        "discard":{"type":"boolean","default":false,
                                   "description":"Drop unsaved changes of the currently open schematic"},
                        "switch_project":{"type":"boolean","default":false,
                                   "description":"Headless: when the open board belongs to another project, close it first (refused with its unsaved changes unless discard: true)"},
                        "override_lock":{"type":"boolean","default":false,
                                   "description":"GUI: open even when another process has the file open (never asked in a dialog; refused with a journal error otherwise)"}}})json"_json,
                   false, h_sch_open, 120 );

KOPENAPI_REGISTER( "sch_close", "Close the open schematic: refuses with unsaved changes unless discard: true, which "
                   "drops them without any dialog (a warning goes to the journal); GUI: the editor window closes",
                   R"json({"type":"object","properties":{"discard":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_close );

KOPENAPI_REGISTER( "sch_save",
                   "Save the open schematic; with 'path' save as (headless switches to it, GUI writes a copy). "
                   "With 'version' (e.g. \"10.0\") write a copy of every sheet and the project that an earlier "
                   "KiCad opens: root at 'path', sub-sheets at their relative paths; the open schematic stays as "
                   "it is. The result lists what the earlier format cannot hold (dropped or approximated). "
                   "KiCad 5 sheets (.sch) are written as .kicad_sch beside them (converted_from_legacy); an existing "
                   "file of that name is replaced only with overwrite: true (409 naming them otherwise)",
                   nlohmann::json( { { "type", "object" },
                                     { "properties",
                                       { { "path", { { "type", "string" } } },
                                         { "overwrite",
                                           { { "type", "boolean" },
                                             { "default", false },
                                             { "description", "replace existing .kicad_sch files the KiCad 5 sheets are converted to" } } },
                                         { "version",
                                           { { "type", "string" },
                                             { "enum", KopenapiLegacyTargets() },
                                             { "description", "earlier KiCad release to write for; needs 'path'" } } } } } } ),
                   false, h_sch_save );

KOPENAPI_REGISTER( "sch_revert", "Reload the open schematic from disk, dropping unsaved changes",
                   R"json({"type":"object","properties":{}})json"_json, false, h_sch_revert, 120 );

KOPENAPI_REGISTER( "sch_autosave_list",
                   "The auto-saved copies of the open schematic's sheet files (the autosave files KiCad writes "
                   "while the backup format is Zip; left-over ones are offered in any format), newest first: path, "
                   "source, time, source_time, newer (newer than the sheet file and different: offered on open). "
                   "Also the autosave settings (enabled, format, location, writes_autosave_files)",
                   R"json({"type":"object","properties":{}})json"_json, false, h_sch_autosave_list );

KOPENAPI_REGISTER( "sch_autosave",
                   "Recover from the open schematic's auto-saved copies. action: rescue (the newest copy of each "
                   "sheet, or path, replaces the open schematic under its own file names, unsaved; the sheet files "
                   "are not touched), or the choices the editor's infobar offers on open for the copies newer than "
                   "their files (or path): restore (the copies replace the sheet files, which are reloaded), "
                   "keep_both (each copy is moved beside its file as <name>.recovered.<time>.kicad_sch), discard "
                   "(deleted; every copy unless path). rescue and restore refuse unsaved changes unless discard: true",
                   R"json({"type":"object","required":["action"],"properties":{
                        "action":{"type":"string","enum":["rescue","restore","keep_both","discard"]},
                        "path":{"type":"string","description":"one copy (sch_autosave_list)"},
                        "discard":{"type":"boolean","default":false,"description":"drop unsaved changes (rescue, restore)"}}})json"_json,
                   false, h_sch_autosave, 120 );
