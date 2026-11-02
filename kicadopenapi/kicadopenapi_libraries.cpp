#include "kicadopenapi_libraries.h"

#include <common.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <pgm_base.h>
#include <libraries/library_manager.h>
#include <libraries/library_table.h>
#include <libraries/library_table_edit.h>
#include <env_paths.h>
#include <project.h>
#include <wx/log.h>
#include <wx/filename.h>


static std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


static nlohmann::json tableJson( const LIBRARY_TABLE* aTable, const char* aScope, int& aErrors )
{
    nlohmann::json table = { { "scope", aScope },
                             { "path", str( aTable->Path() ) },
                             { "ok", aTable->IsOk() },
                             { "libraries", aTable->Rows().size() } };

    if( !aTable->IsOk() )
    {
        table["error"] = str( aTable->ErrorDescription() );
        aErrors++;
    }

    // Nested tables ("Table" rows, e.g. KiCad's default libraries): a missing file there leaves
    // the whole set of libraries out
    nlohmann::json nested = nlohmann::json::array();

    for( const LIBRARY_TABLE_ROW& row : aTable->Rows() )
    {
        if( row.Type() != wxS( "Table" ) )
            continue;

        nlohmann::json n = { { "name", str( row.Nickname() ) }, { "uri", str( row.URI() ) }, { "ok", row.IsOk() } };

        // The row may be "ok" syntactically while the file it names does not exist
        if( !row.IsOk() || !wxFileName::FileExists( ExpandEnvVarSubstitutions( row.URI(), nullptr ) ) )
        {
            n["ok"] = false;
            n["error"] = row.IsOk() ? "table file not found: " + str( row.URI() ) : str( row.ErrorDescription() );
            aErrors++;
        }

        nested.push_back( std::move( n ) );
    }

    if( !nested.empty() )
        table["nested_tables"] = nested;

    return table;
}


nlohmann::json KopenapiLibraryTables( const LIBRARY_MANAGER_ADAPTER& aAdapter, int& aErrors )
{
    nlohmann::json tables = nlohmann::json::array();

    if( const LIBRARY_TABLE* global = aAdapter.GlobalTable() )
        tables.push_back( tableJson( global, "global", aErrors ) );

    if( std::optional<LIBRARY_TABLE*> project = aAdapter.ProjectTable(); project && *project )
        tables.push_back( tableJson( *project, "project", aErrors ) );

    return tables;
}


void KopenapiCheckGlobalLibraryTables()
{
    for( const auto& [type, kind] : { std::pair{ LIBRARY_TABLE_TYPE::SYMBOL, "symbol" },
                                      std::pair{ LIBRARY_TABLE_TYPE::FOOTPRINT, "footprint" } } )
    {
        const wxString path = LIBRARY_MANAGER::DefaultGlobalTablePath( type );
        const wxString hint = wxString::Format( wxS( " - KiCad will see no global %s libraries "
                                                     "(fix: kicad-toolset/tools/kicad_dev_libraries.py --fix)" ),
                                                kind );

        if( !wxFileName::FileExists( path ) )
        {
            wxLogWarning( "Global %s library table missing: %s%s", kind, path, hint );
            continue;
        }

        LIBRARY_TABLE table( wxFileName( path ), LIBRARY_TABLE_SCOPE::GLOBAL );

        if( !table.IsOk() )
        {
            wxLogWarning( "Global %s library table %s unreadable: %s%s", kind, path, table.ErrorDescription(), hint );
            continue;
        }

        if( table.Rows().empty() )
        {
            wxLogWarning( "Global %s library table %s is empty%s", kind, path, hint );
            continue;
        }

        for( const LIBRARY_TABLE_ROW& row : table.Rows() )
        {
            const wxString uri = ExpandEnvVarSubstitutions( row.URI(), nullptr );

            if( row.Type() != wxS( "Table" ) )
                continue;

            if( !wxFileName::FileExists( uri ) )
            {
                wxLogWarning( "Global %s library table %s: nested table '%s' points to a missing file %s%s", kind,
                              path, row.Nickname(), uri, hint );
                continue;
            }

            // The nested table's own libraries usually use ${KICADn_SYMBOL_DIR} etc.: check the
            // first one resolves (a development build has no libraries in its own bundle)
            LIBRARY_TABLE nested( wxFileName( uri ), LIBRARY_TABLE_SCOPE::GLOBAL );

            for( const LIBRARY_TABLE_ROW& lib : nested.Rows() )
            {
                const wxString libPath = ExpandEnvVarSubstitutions( lib.URI(), nullptr );

                if( !wxFileName::Exists( libPath ) )
                {
                    wxLogWarning( "Global %s libraries from %s do not resolve: '%s' -> %s does not exist "
                                  "(set the library path variables to an installed KiCad's libraries)%s",
                                  kind, uri, lib.URI(), libPath, hint );
                }

                break;
            }
        }
    }
}


void KopenapiEnsureFootprintLibraries( KIWAY* aKiway )
{
    if( !PgmOrNull() )
        return;

    if( aKiway && !Pgm().GetLibraryManager().Adapter( LIBRARY_TABLE_TYPE::FOOTPRINT ) )
    {
        // pcbnew's library listing creates the adapter in pcbnew (and loads the libraries)
        aKiway->KiFACE( KIWAY::FACE_PCB );

        if( std::optional<KOPENAPI_METHOD> list = KOPENAPI_REGISTRY::Get().Find( "pcb_lib_list" ) )
        {
            KOPENAPI_CONTEXT context;
            context.kiway = aKiway;
            list->handler( context, { { "limit", 1 } } );
        }
    }

    if( std::optional<LIBRARY_MANAGER_ADAPTER*> adapter = Pgm().GetLibraryManager().Adapter( LIBRARY_TABLE_TYPE::FOOTPRINT );
        adapter && *adapter )
    {
        ( *adapter )->AsyncLoad();
        KopenapiWaitLibraries( *adapter );
    }
}


namespace
{

std::optional<LIBRARY_TABLE_SCOPE> tableScope( const nlohmann::json& aArgs, std::string& aError )
{
    const std::string table = aArgs.value( "table", std::string( "project" ) );

    if( table == "project" )
        return LIBRARY_TABLE_SCOPE::PROJECT;

    if( table == "global" )
        return LIBRARY_TABLE_SCOPE::GLOBAL;

    aError = "table: project or global";
    return std::nullopt;
}


const char* scopeText( LIBRARY_TABLE_SCOPE aScope )
{
    return aScope == LIBRARY_TABLE_SCOPE::GLOBAL ? "global" : "project";
}


/// The table URI of a path: as given when it holds a variable, else normalised to an environment
/// variable or (project table) the project, as Add Library
wxString tableUri( const wxString& aPath, LIBRARY_TABLE_SCOPE aScope, PROJECT& aProject, bool aNormalize )
{
    if( !aNormalize || aPath.Contains( wxS( "${" ) ) )
        return aPath;

    wxFileName fn( aPath );
    fn.Normalize( wxPATH_NORM_DOTS | wxPATH_NORM_ABSOLUTE );

    return NormalizePath( fn, &Pgm().GetLocalEnvVariables(),
                          aScope == LIBRARY_TABLE_SCOPE::PROJECT ? &aProject : nullptr );
}


nlohmann::json rowJson( const LIBRARY_TABLE_ROW& aRow, LIBRARY_TABLE_SCOPE aScope, PROJECT& aProject,
                        LIBRARY_MANAGER_ADAPTER* aAdapter )
{
    nlohmann::json out = { { "nickname", str( aRow.Nickname() ) },
                           { "table", scopeText( aScope ) },
                           { "uri", str( aRow.URI() ) },
                           { "path", str( LIBRARY_MANAGER::ExpandURI( aRow.URI(), aProject ) ) },
                           { "type", str( aRow.Type() ) },
                           { "options", str( aRow.Options() ) },
                           { "description", str( aRow.Description() ) },
                           { "enabled", !aRow.Disabled() },
                           { "visible", !aRow.Hidden() } };

    if( aAdapter )
    {
        std::optional<LIB_STATUS> status = aAdapter->GetLibraryStatus( aRow.Nickname() );

        if( status && status->load_status == LOAD_STATUS::LOADED )
            out["status"] = "loaded";
        else if( status && status->load_status == LOAD_STATUS::LOAD_ERROR )
        {
            out["status"] = "error";
            out["error"] = status->error ? str( status->error->message ) : std::string();
        }
    }

    return out;
}


/// The row fields of aArgs (uri / path, type, options, description, enabled, visible) applied to aRow
std::string applyFields( LIBRARY_TABLE_ROW& aRow, const KOPENAPI_LIB_TABLE_KIND& aKind, const nlohmann::json& aArgs,
                         LIBRARY_TABLE_SCOPE aScope, PROJECT& aProject )
{
    for( const char* key : { "path", "type", "options", "description" } )
    {
        if( aArgs.contains( key ) && !aArgs[key].is_string() )
            return std::string( key ) + ": a string";
    }

    for( const char* key : { "enabled", "visible", "normalize" } )
    {
        if( aArgs.contains( key ) && !aArgs[key].is_boolean() )
            return std::string( key ) + ": true or false";
    }

    if( aArgs.contains( "path" ) )
    {
        const wxString path = wxString::FromUTF8( aArgs["path"].get<std::string>() );

        if( path.IsEmpty() || ( !path.Contains( wxS( "${" ) ) && !wxFileName( path ).IsAbsolute() ) )
            return "path: an absolute path or a URI with a variable (${KIPRJMOD}/...)";

        aRow.SetURI( tableUri( path, aScope, aProject, aArgs.value( "normalize", true ) ) );

        if( !aArgs.contains( "type" ) )
            aRow.SetType( aKind.guessType( LIBRARY_MANAGER::ExpandURI( aRow.URI(), aProject ) ) );
    }

    if( aArgs.contains( "type" ) )
    {
        const wxString type = wxString::FromUTF8( aArgs["type"].get<std::string>() );

        if( type != LIBRARY_TABLE_ROW::TABLE_TYPE_NAME && !aKind.isType( type ) )
            return "type: not a library type of this table (" + str( type ) + ")";

        aRow.SetType( type );
    }

    if( aArgs.contains( "options" ) )
        aRow.SetOptions( wxString::FromUTF8( aArgs["options"].get<std::string>() ) );

    if( aArgs.contains( "description" ) )
        aRow.SetDescription( wxString::FromUTF8( aArgs["description"].get<std::string>() ) );

    if( aArgs.contains( "enabled" ) )
        aRow.SetDisabled( !aArgs["enabled"].get<bool>() );

    if( aArgs.contains( "visible" ) )
        aRow.SetHidden( !aArgs["visible"].get<bool>() );

    return std::string();
}


/// The table holding aNickname: the one named by "table", else the project's (its rows shadow the
/// global ones), else the global one
std::optional<LIBRARY_TABLE_SCOPE> holdingTable( const KOPENAPI_LIB_TABLE_KIND& aKind, const wxString& aNickname,
                                                 const nlohmann::json& aArgs, std::string& aError, int& aStatus )
{
    LIBRARY_MANAGER&               manager = Pgm().GetLibraryManager();
    std::vector<LIBRARY_TABLE_SCOPE> scopes;

    if( aArgs.contains( "table" ) )
    {
        std::optional<LIBRARY_TABLE_SCOPE> scope = tableScope( aArgs, aError );

        if( !scope )
        {
            aStatus = 400;
            return std::nullopt;
        }

        scopes.push_back( *scope );
    }
    else
    {
        scopes = { LIBRARY_TABLE_SCOPE::PROJECT, LIBRARY_TABLE_SCOPE::GLOBAL };
    }

    for( LIBRARY_TABLE_SCOPE scope : scopes )
    {
        std::optional<LIBRARY_TABLE*> table = manager.Table( aKind.adapter->Type(), scope );

        if( table && *table && ( *table )->HasRow( aNickname ) )
            return scope;
    }

    aStatus = 404;
    aError = "nickname: no library " + str( aNickname ) + " in the "
             + ( aArgs.contains( "table" ) ? aArgs.value( "table", std::string() ) + " table" : std::string( "tables" ) );
    return std::nullopt;
}

} // namespace


KOPENAPI_RESULT KopenapiLibTableAdd( PROJECT& aProject, const KOPENAPI_LIB_TABLE_KIND& aKind,
                                     const nlohmann::json& aArgs )
{
    std::string                        error;
    std::optional<LIBRARY_TABLE_SCOPE> scope = tableScope( aArgs, error );

    if( !scope )
        return KOPENAPI_RESULT::Error( 400, error );

    if( !aArgs.contains( "path" ) )
        return KOPENAPI_RESULT::Error( 400, "path: the library (an absolute path or a URI with a variable)" );

    if( aArgs.contains( "nickname" ) && !aArgs["nickname"].is_string() )
        return KOPENAPI_RESULT::Error( 400, "nickname: a string" );

    if( *scope == LIBRARY_TABLE_SCOPE::PROJECT && aProject.IsNullProject() )
        return KOPENAPI_RESULT::Error( 409, "table: no project open for a project table" );

    LIBRARY_TABLE_ROW row;
    row.SetScope( *scope );

    if( std::string fieldError = applyFields( row, aKind, aArgs, *scope, aProject ); !fieldError.empty() )
        return KOPENAPI_RESULT::Error( 400, fieldError );

    wxString nickname = aArgs.contains( "nickname" )
                                ? wxString::FromUTF8( aArgs["nickname"].get<std::string>() )
                                : aKind.nicknameOf( LIBRARY_MANAGER::ExpandURI( row.URI(), aProject ) );
    row.SetNickname( nickname );

    int      status = 400;
    wxString editError;

    const bool done = EditLibraryTable( aKind.adapter->Type(), *scope,
            [&]( LIBRARY_TABLE& aTable ) -> wxString
            {
                if( aTable.HasRow( nickname ) )
                {
                    status = 409;
                    return wxString::Format( wxS( "nickname: the %s table already has %s" ),
                                             scopeText( *scope ), nickname );
                }

                LIBRARY_TABLE_ROW& added = aTable.InsertRow();
                const wxString     nick = row.Nickname();
                added.SetNickname( nick );
                added.SetURI( row.URI() );
                added.SetType( row.Type() );
                added.SetOptions( row.Options() );
                added.SetDescription( row.Description() );
                added.SetDisabled( row.Disabled() );
                added.SetHidden( row.Hidden() );
                return wxEmptyString;
            },
            editError );

    if( !done )
        return KOPENAPI_RESULT::Error( editError.StartsWith( wxS( "Error saving" ) ) ? 422 : status,
                                       str( editError ) );

    if( row.Type() != LIBRARY_TABLE_ROW::TABLE_TYPE_NAME && !row.Disabled() )
        aKind.adapter->LoadLibraryEntry( nickname );

    std::optional<LIBRARY_TABLE*> table = Pgm().GetLibraryManager().Table( aKind.adapter->Type(), *scope );
    std::optional<LIBRARY_TABLE_ROW*> added = table ? ( *table )->Row( nickname ) : std::nullopt;

    if( !added )
        return KOPENAPI_RESULT::Error( 500, "the row is not in the reloaded table" );

    nlohmann::json out = rowJson( **added, *scope, aProject, aKind.adapter );
    out["exists"] = wxFileName::Exists( LIBRARY_MANAGER::ExpandURI( row.URI(), aProject ) );
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT KopenapiLibTableRemove( PROJECT& aProject, const KOPENAPI_LIB_TABLE_KIND& aKind,
                                        const nlohmann::json& aArgs )
{
    const wxString nickname = wxString::FromUTF8( aArgs.value( "nickname", std::string() ) );

    if( nickname.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "nickname: the library's nickname" );

    std::string                        error;
    int                                status = 400;
    std::optional<LIBRARY_TABLE_SCOPE> scope = holdingTable( aKind, nickname, aArgs, error, status );

    if( !scope )
        return KOPENAPI_RESULT::Error( status, error );

    nlohmann::json removed;
    wxString       editError;

    const bool done = EditLibraryTable( aKind.adapter->Type(), *scope,
            [&]( LIBRARY_TABLE& aTable ) -> wxString
            {
                auto& rows = aTable.Rows();

                for( auto it = rows.begin(); it != rows.end(); ++it )
                {
                    if( it->Nickname() == nickname )
                    {
                        removed = rowJson( *it, *scope, aProject, nullptr );
                        rows.erase( it );
                        break;
                    }
                }

                return wxEmptyString;
            },
            editError );

    if( !done )
        return KOPENAPI_RESULT::Error( 422, str( editError ) );

    nlohmann::json out = { { "removed", removed } };

    // a global row of that name shows again once the project's is gone
    if( aKind.adapter->HasLibrary( nickname ) )
        out["now_from"] = "global";

    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT KopenapiLibTableSet( PROJECT& aProject, const KOPENAPI_LIB_TABLE_KIND& aKind,
                                     const nlohmann::json& aArgs )
{
    const wxString nickname = wxString::FromUTF8( aArgs.value( "nickname", std::string() ) );

    if( nickname.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "nickname: the library's nickname" );

    if( aArgs.contains( "new_nickname" ) && !aArgs["new_nickname"].is_string() )
        return KOPENAPI_RESULT::Error( 400, "new_nickname: a string" );

    std::string                        error;
    int                                status = 400;
    std::optional<LIBRARY_TABLE_SCOPE> scope = holdingTable( aKind, nickname, aArgs, error, status );

    if( !scope )
        return KOPENAPI_RESULT::Error( status, error );

    bool any = aArgs.contains( "new_nickname" );

    for( const char* key : { "path", "type", "options", "description", "enabled", "visible" } )
        any = any || aArgs.contains( key );

    if( !any )
        return KOPENAPI_RESULT::Error( 400, "nothing to set: new_nickname, path, type, options, description, "
                                            "enabled or visible" );

    const wxString newNickname = aArgs.contains( "new_nickname" )
                                         ? wxString::FromUTF8( aArgs["new_nickname"].get<std::string>() )
                                         : nickname;
    wxString       editError;
    status = 400;

    const bool done = EditLibraryTable( aKind.adapter->Type(), *scope,
            [&]( LIBRARY_TABLE& aTable ) -> wxString
            {
                std::optional<LIBRARY_TABLE_ROW*> row = aTable.Row( nickname );

                if( !row )
                    return wxS( "nickname: not in the table" );

                if( newNickname != nickname && aTable.HasRow( newNickname ) )
                {
                    status = 409;
                    return wxString::Format( wxS( "new_nickname: the %s table already has %s" ),
                                             scopeText( *scope ), newNickname );
                }

                if( std::string fieldError = applyFields( **row, aKind, aArgs, *scope, aProject );
                    !fieldError.empty() )
                {
                    return wxString::FromUTF8( fieldError );
                }

                ( *row )->SetNickname( newNickname );
                return wxEmptyString;
            },
            editError );

    if( !done )
        return KOPENAPI_RESULT::Error( editError.StartsWith( wxS( "Error saving" ) ) ? 422 : status,
                                       str( editError ) );

    std::optional<LIBRARY_TABLE*>     table = Pgm().GetLibraryManager().Table( aKind.adapter->Type(), *scope );
    std::optional<LIBRARY_TABLE_ROW*> row = table ? ( *table )->Row( newNickname ) : std::nullopt;

    if( !row )
        return KOPENAPI_RESULT::Error( 500, "the row is not in the reloaded table" );

    if( ( *row )->Type() != LIBRARY_TABLE_ROW::TABLE_TYPE_NAME && !( *row )->Disabled() )
        aKind.adapter->LoadLibraryEntry( newNickname );

    return KOPENAPI_RESULT::Ok( rowJson( **row, *scope, aProject, aKind.adapter ) );
}


nlohmann::json KopenapiLibTableSchema( const std::string& aOperation )
{
    nlohmann::json props = {
        { "table", { { "type", "string" }, { "enum", { "project", "global" } } } }
    };
    nlohmann::json fields = {
        { "path", { { "type", "string" },
                    { "description", "the library: an absolute path (normalised to ${KIPRJMOD} or an environment "
                                     "variable unless normalize is false) or a URI with a variable" } } },
        { "normalize", { { "type", "boolean" }, { "default", true } } },
        { "type", { { "type", "string" }, { "description", "plugin type (KiCad, Legacy, Eagle...; Table for a "
                                                             "nested table); default guessed from the path" } } },
        { "options", { { "type", "string" } } },
        { "description", { { "type", "string" } } },
        { "enabled", { { "type", "boolean" } } },
        { "visible", { { "type", "boolean" } } }
    };
    nlohmann::json schema = { { "type", "object" } };

    if( aOperation == "add" )
    {
        props.update( fields );
        props["nickname"] = { { "type", "string" }, { "description", "default the library's file name" } };
        props["table"]["default"] = "project";
        schema["required"] = { "path" };
    }
    else
    {
        props["nickname"] = { { "type", "string" } };
        props["table"]["description"] = "default: the project table if it has the nickname, else the global";
        schema["required"] = { "nickname" };

        if( aOperation == "set" )
        {
            props.update( fields );
            props["new_nickname"] = { { "type", "string" } };
        }
    }

    schema["properties"] = props;
    return schema;
}
