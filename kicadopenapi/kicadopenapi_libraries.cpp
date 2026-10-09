#include "kicadopenapi_libraries.h"

#include <common.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <pgm_base.h>
#include <libraries/library_manager.h>
#include <libraries/library_table.h>
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
        ( *adapter )->BlockUntilLoaded();
    }
}
