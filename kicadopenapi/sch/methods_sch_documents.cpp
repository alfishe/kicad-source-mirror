/*
 * kicadopenapi schematic document methods: sch_open, sch_close, sch_save, sch_revert, and
 * the "sch" document provider.  Compiled into the eeschema kiface; registered when it loads.
 */
#include "kopenapi_sch.h"

#include <api/headless_sch_context.h>
#include <api/sch_context.h>
#include <eeschema_helpers.h>
#include <kiway.h>
#include <pgm_base.h>
#include <project.h>
#include <sch_edit_frame.h>
#include <sch_sheet_path.h>
#include <schematic.h>
#include <settings/settings_manager.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>


/// Headless schematic: the context reads it through this slot (RevertToSaved replaces it)
static SCHEMATIC*                            s_schematic = nullptr;
static std::shared_ptr<HEADLESS_SCH_CONTEXT> s_headless;


static void releaseHeadless()
{
    s_headless.reset();
    delete s_schematic;
    s_schematic = nullptr;
}


static SCH_EDIT_FRAME* guiFrame( KOPENAPI_CONTEXT& aCtx, bool aCreate )
{
    if( aCtx.headless || !aCtx.kiway )
        return nullptr;

    return static_cast<SCH_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_SCH, aCreate ) );
}


std::shared_ptr<SCH_CONTEXT> KopenapiSchContext( KOPENAPI_CONTEXT& aCtx )
{
    if( aCtx.headless )
        return s_headless;

    SCH_EDIT_FRAME* frame = guiFrame( aCtx, false );

    if( !frame )
        return nullptr;

    std::shared_ptr<SCH_CONTEXT> context = CreateSchFrameContext( frame );
    return context->GetCurrentFileName().IsEmpty() ? nullptr : context;
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


/// Another open document of this process belongs to a different project
static std::string projectConflict( KOPENAPI_CONTEXT& aCtx, const wxFileName& aProject )
{
    for( const nlohmann::json& doc : KOPENAPI_REGISTRY::Get().Documents( aCtx ) )
    {
        if( doc.value( "domain", "" ) == "sch" )
            continue;

        wxFileName other( wxString::FromUTF8( doc.value( "project", "" ) ) );

        if( !other.GetFullPath().IsEmpty() && !other.SameAs( aProject ) )
            return doc.value( "project", "" );
    }

    return {};
}


static KOPENAPI_RESULT openHeadless( KOPENAPI_CONTEXT& aCtx, const wxFileName& aSchematic )
{
    wxFileName projectFile( aSchematic );
    projectFile.SetExt( FILEEXT::ProjectFileExtension );

    if( std::string other = projectConflict( aCtx, projectFile ); !other.empty() )
    {
        return KOPENAPI_RESULT::Error( 409, "this process serves project " + other
                                                    + "; start another instance for a different project" );
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
    catch( ... )
    {
        schematic = nullptr;
    }

    if( !schematic )
        return KOPENAPI_RESULT::Error( 422, "schematic could not be loaded" );

    s_schematic = schematic;
    s_headless = std::make_shared<HEADLESS_SCH_CONTEXT>( &s_schematic, project, aCtx.kiway );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT openGui( KOPENAPI_CONTEXT& aCtx, const wxFileName& aSchematic )
{
    SCH_EDIT_FRAME* frame = guiFrame( aCtx, true );

    if( !frame )
        return KOPENAPI_RESULT::Error( 500, "could not create the schematic editor window" );

    if( !frame->OpenProjectFiles( { aSchematic.GetFullPath() } ) )
        return KOPENAPI_RESULT::Error( 422, "the schematic editor could not open the schematic" );

    // OpenProjectFiles does not show the frame; mirror KICAD_MANAGER_CONTROL::ShowPlayer
    frame->Iconize( false );
    frame->Show( true );
    frame->Raise();

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT h_sch_open( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    if( !aArgs["path"].is_string() || aArgs["path"].get<std::string>().empty() )
        return KOPENAPI_RESULT::Error( 400, "'path' must be a non-empty string" );

    wxFileName schematic( wxString::FromUTF8( aArgs["path"].get<std::string>() ) );
    schematic.MakeAbsolute();

    if( schematic.GetExt() == FILEEXT::ProjectFileExtension )
        schematic.SetExt( FILEEXT::KiCadSchematicFileExtension );

    if( schematic.GetExt() != FILEEXT::KiCadSchematicFileExtension )
        return KOPENAPI_RESULT::Error( 400, "expected a .kicad_sch or .kicad_pro file" );

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

    return aCtx.headless ? openHeadless( aCtx, schematic ) : openGui( aCtx, schematic );
}


static KOPENAPI_RESULT h_sch_close( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( isModified( *context ) )
    {
        if( !aArgs.value( "discard", false ) )
            return KOPENAPI_RESULT::Error( 409, "the schematic has unsaved changes; sch_save or pass discard: true" );

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


static KOPENAPI_RESULT h_sch_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    bool ok = false;

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

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


static KOPENAPI_RESULT h_sch_revert( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !context->RevertToSaved() )
        return KOPENAPI_RESULT::Error( 422, "reverting failed (no saved file?)" );

    return KOPENAPI_RESULT::Ok( documentStatus( aCtx ) );
}


KOPENAPI_REGISTER_DOCUMENTS( "sch", documentStatus, releaseHeadless );

KOPENAPI_REGISTER( "sch_open",
                   "Open a schematic (.kicad_sch or its .kicad_pro): editor window in the GUI, in memory headless",
                   R"({"type":"object","required":["path"],"properties":{
                        "path":{"type":"string"},
                        "discard":{"type":"boolean","default":false,
                                   "description":"Drop unsaved changes of the currently open schematic"}}})"_json,
                   false, h_sch_open );

KOPENAPI_REGISTER( "sch_close", "Close the open schematic (refuses with unsaved changes unless discard)",
                   R"({"type":"object","properties":{"discard":{"type":"boolean","default":false}}})"_json,
                   false, h_sch_close );

KOPENAPI_REGISTER( "sch_save",
                   "Save the open schematic; with 'path' save as (headless switches to it, GUI writes a copy)",
                   R"({"type":"object","properties":{"path":{"type":"string"}}})"_json, false, h_sch_save );

KOPENAPI_REGISTER( "sch_revert", "Reload the open schematic from disk, dropping unsaved changes",
                   R"({"type":"object","properties":{}})"_json, false, h_sch_revert );
