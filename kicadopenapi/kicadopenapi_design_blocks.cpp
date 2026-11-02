/// @file kicadopenapi_design_blocks.cpp
/// @brief kicadopenapi design block libraries shared by pcb_design_block_* and sch_design_block_*
#include "kicadopenapi_design_blocks.h"

#include <design_block.h>
#include <design_block_io.h>
#include <design_block_library_adapter.h>
#include <io/io_mgr.h>
#include <ki_exception.h>
#include <kicadopenapi_keepalive.h>
#include <lib_id.h>
#include <project.h>
#include <wildcards_and_files_ext.h>

#include <wx/filename.h>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


wxString wx( const std::string& aText )
{
    return wxString::FromUTF8( aText );
}


DESIGN_BLOCK_LIBRARY_ADAPTER* tableLibraries( PROJECT* aProject )
{
    DESIGN_BLOCK_LIBRARY_ADAPTER* adapter = aProject ? aProject->DesignBlockLibs() : nullptr;

    if( adapter )
    {
        adapter->AsyncLoad();
        KopenapiWaitLibraries( adapter );
    }

    return adapter;
}


IO_RELEASER<DESIGN_BLOCK_IO> blockIo()
{
    return IO_RELEASER<DESIGN_BLOCK_IO>( DESIGN_BLOCK_IO_MGR::FindPlugin( DESIGN_BLOCK_IO_MGR::KICAD_SEXP ) );
}

} // namespace


std::string KOPENAPI_DB_LIBRARY::Name() const
{
    return str( IsPath() ? path : nickname );
}


bool KopenapiDesignBlockLibraryWritable( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib )
{
    return aLib.IsPath() || tableLibraries( aProject )->IsDesignBlockLibWritable( aLib.nickname );
}


const nlohmann::json& KopenapiDesignBlockListSchema()
{
    static const nlohmann::json schema = R"json({"type":"object","properties":{"library":{"type":"string"}}})json"_json;
    return schema;
}


std::optional<KOPENAPI_DB_LIBRARY> KopenapiDesignBlockLibrary( PROJECT* aProject, const nlohmann::json& aArgs,
                                                                bool aMustExist, const std::string& aListMethod,
                                                                std::string& aError, int& aStatus )
{
    const std::string spec = aArgs.value( "library", std::string() );
    aStatus = 400;

    if( spec.empty() )
    {
        aError = "library: a design block library nickname (" + aListMethod + ") or the absolute path of a ."
                 + FILEEXT::KiCadDesignBlockLibPathExtension + " directory";
        return std::nullopt;
    }

    wxFileName     dir = wxFileName::DirName( wx( spec ) );
    const wxString ext = wxS( "." ) + wxString( FILEEXT::KiCadDesignBlockLibPathExtension );

    if( wx( spec ).EndsWith( ext ) || spec.find_first_of( "/\\" ) != std::string::npos )
    {
        if( !dir.IsAbsolute() || !wx( spec ).EndsWith( ext ) )
        {
            aError = "library: an absolute path ending in ." + FILEEXT::KiCadDesignBlockLibPathExtension;
            return std::nullopt;
        }

        if( aMustExist && !dir.DirExists() )
        {
            aStatus = 404;
            aError = "library: no design block library at " + spec;
            return std::nullopt;
        }

        wxString path = dir.GetPath();
        return KOPENAPI_DB_LIBRARY{ wxFileName( path ).GetName(), path };
    }

    DESIGN_BLOCK_LIBRARY_ADAPTER* adapter = tableLibraries( aProject );

    if( !adapter || !adapter->HasLibrary( wx( spec ) ) )
    {
        std::string names;

        if( adapter )
        {
            for( const wxString& name : adapter->GetLibraryNames() )
                names += ( names.empty() ? "" : ", " ) + str( name );
        }

        aStatus = 404;
        aError = "library: no design block library '" + spec + "' in the library tables ("
                 + ( names.empty() ? std::string( "none" ) : names ) + "); or give the absolute path of a ."
                 + FILEEXT::KiCadDesignBlockLibPathExtension + " directory";
        return std::nullopt;
    }

    return KOPENAPI_DB_LIBRARY{ wx( spec ), wxString() };
}


bool KopenapiDesignBlockExists( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib, const wxString& aName )
{
    if( aLib.IsPath() )
        return wxFileName::DirExists( aLib.path + wxFileName::GetPathSeparator() + aName + wxS( "." )
                                      + wxString( FILEEXT::KiCadDesignBlockPathExtension ) );

    return tableLibraries( aProject )->DesignBlockExists( aLib.nickname, aName );
}


std::unique_ptr<DESIGN_BLOCK> KopenapiLoadDesignBlock( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib,
                                                       const wxString& aName, std::string& aError )
{
    std::unique_ptr<DESIGN_BLOCK> block;

    try
    {
        if( aLib.IsPath() )
            block.reset( blockIo()->DesignBlockLoad( aLib.path, aName ) );
        else
        {
            wxString error;
            block.reset( tableLibraries( aProject )->LoadDesignBlock( aLib.nickname, aName, false, &error ) );

            if( !block )
                aError = str( error );
        }
    }
    catch( const IO_ERROR& error )
    {
        aError = str( error.What() );
    }

    if( block )
        block->SetLibId( LIB_ID( aLib.nickname, aName ) );
    else if( aError.empty() )
        aError = "could not read the design block";

    return block;
}


void KopenapiSaveDesignBlock( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib, const DESIGN_BLOCK& aBlock )
{
    if( aLib.IsPath() )
    {
        IO_RELEASER<DESIGN_BLOCK_IO> io = blockIo();

        if( !wxFileName::DirExists( aLib.path ) )
            io->CreateLibrary( aLib.path );

        io->DesignBlockSave( aLib.path, &aBlock );
        return;
    }

    if( tableLibraries( aProject )->SaveDesignBlock( aLib.nickname, &aBlock ) != DESIGN_BLOCK_LIBRARY_ADAPTER::SAVE_OK )
        THROW_IO_ERROR( wxString::Format( wxS( "the library '%s' did not save the design block" ), aLib.nickname ) );
}


nlohmann::json KopenapiDesignBlockJson( const DESIGN_BLOCK& aBlock )
{
    return { { "name", str( aBlock.GetLibId().GetLibItemName() ) },
             { "lib_id", str( aBlock.GetLibId().Format() ) },
             { "description", str( aBlock.GetLibDescription() ) },
             { "keywords", str( aBlock.GetKeywords() ) },
             { "has_layout", !aBlock.GetBoardFile().IsEmpty() && wxFileName::FileExists( aBlock.GetBoardFile() ) },
             { "has_schematic",
               !aBlock.GetSchematicFile().IsEmpty() && wxFileName::FileExists( aBlock.GetSchematicFile() ) } };
}


KOPENAPI_RESULT KopenapiDesignBlockList( PROJECT* aProject, const nlohmann::json& aArgs, const std::string& aListMethod )
{
    if( !aArgs.contains( "library" ) )
    {
        DESIGN_BLOCK_LIBRARY_ADAPTER* adapter = tableLibraries( aProject );
        nlohmann::json                libs = nlohmann::json::array();

        if( adapter )
        {
            for( const wxString& name : adapter->GetLibraryNames() )
            {
                nlohmann::json lib = { { "nickname", str( name ) },
                                       { "blocks", adapter->GetDesignBlockNames( name ).size() } };

                if( std::optional<wxString> desc = adapter->GetLibraryDescription( name ) )
                    lib["description"] = str( *desc );

                libs.push_back( lib );
            }
        }

        return KOPENAPI_RESULT::Ok( { { "libraries", libs } } );
    }

    std::string                        error;
    int                                status = 400;
    std::optional<KOPENAPI_DB_LIBRARY> lib = KopenapiDesignBlockLibrary( aProject, aArgs, true, aListMethod, error, status );

    if( !lib )
        return KOPENAPI_RESULT::Error( status, error );

    wxArrayString names;

    try
    {
        if( lib->IsPath() )
            blockIo()->DesignBlockEnumerate( names, lib->path, true );
        else
        {
            for( const wxString& name : tableLibraries( aProject )->GetDesignBlockNames( lib->nickname ) )
                names.Add( name );
        }
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 422, "library: " + str( ioe.What() ) );
    }

    names.Sort();
    nlohmann::json blocks = nlohmann::json::array();

    for( const wxString& name : names )
    {
        std::string                   loadError;
        std::unique_ptr<DESIGN_BLOCK> block = KopenapiLoadDesignBlock( aProject, *lib, name, loadError );

        if( block )
            blocks.push_back( KopenapiDesignBlockJson( *block ) );
        else
            blocks.push_back( { { "name", str( name ) }, { "error", loadError } } );
    }

    return KOPENAPI_RESULT::Ok( { { "library", lib->Name() }, { "blocks", blocks } } );
}
