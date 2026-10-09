#include "kicadopenapi_libraries.h"

#include <common.h>
#include <libraries/library_manager.h>
#include <libraries/library_table.h>
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
