/// @file methods_pcb_lib_edit.cpp
/// @brief kicadopenapi footprint library authoring from the board: pcb_footprint_models_embed,
/// pcb_lib_footprint_save; KopenapiModelFile (a 3D model's file on disk, embedded ones unpacked).
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <common.h>
#include <embedded_files.h>
#include <footprint.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <project.h>
#include <tool/tool_manager.h>
#include <wildcards_and_files_ext.h>

#include <wx/dir.h>
#include <wx/filename.h>
#include <wx/stdpaths.h>

#include <fstream>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


const wxString EMBED_PREFIX = wxString( FILEEXT::KiCadUriPrefix ) + wxS( "://" );


std::set<FOOTPRINT*> targets( BOARD* aBoard, const nlohmann::json& aArgs )
{
    std::set<std::string> refs;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
        refs.insert( r.get<std::string>() );

    if( aArgs.contains( "ref" ) )
        refs.insert( aArgs["ref"].get<std::string>() );

    const std::string    libId = aArgs.value( "lib_id", std::string() );
    std::set<FOOTPRINT*> out;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( refs.count( str( fp->GetReference() ) ) || ( !libId.empty() && str( fp->GetFPID().Format() ) == libId ) )
            out.insert( fp );
    }

    return out;
}

} // namespace


std::filesystem::path KopenapiModelFile( const FOOTPRINT* aFootprint, const FP_3DMODEL& aModel, PROJECT* aProject )
{
    namespace fs = std::filesystem;

    // embedded in the footprint: unpacked once into the user cache, named by its hash
    if( aModel.m_Filename.StartsWith( EMBED_PREFIX ) )
    {
        const wxString name = aModel.m_Filename.Mid( EMBED_PREFIX.length() );
        EMBEDDED_FILES::EMBEDDED_FILE* file =
                const_cast<FOOTPRINT*>( aFootprint )->GetEmbeddedFiles()->GetEmbeddedFile( name );

        if( !file )
            return {};

        if( file->decompressedData.empty()
            && EMBEDDED_FILES::DecompressAndDecode( *file ) != EMBEDDED_FILES::RETURN_CODE::OK )
        {
            return {};
        }

        const fs::path dir = fs::path( str( wxStandardPaths::Get().GetUserDir( wxStandardPaths::Dir_Cache ) ) )
                             / "kicad" / "openapi" / "embedded";
        // only the last component of the name: an embedded "../x" or "/x" stays in the cache folder
        const std::string leaf = fs::path( str( name ) ).filename().string();

        if( leaf.empty() || leaf == "." || leaf == ".." )
            return {};

        const fs::path out = dir / ( file->data_hash.substr( 0, 16 ) + "-" + leaf );

        if( !fs::exists( out ) )
        {
            std::error_code ec;
            fs::create_directories( dir, ec );
            std::ofstream( out, std::ios::binary ).write( file->decompressedData.data(), file->decompressedData.size() );
        }

        return out;
    }

    fs::path p( str( ExpandEnvVarSubstitutions( aModel.m_Filename, aProject ) ) );

    if( p.is_relative() && aProject )
        p = fs::path( str( aProject->GetProjectPath() ) ) / p;

    if( p.extension() == ".wrl" || p.extension() == ".WRL" )
    {
        fs::path step = p;
        step.replace_extension( ".step" );

        if( fs::exists( step ) )
            p = step;
    }

    return p;
}


static KOPENAPI_RESULT h_pcb_footprint_models_embed( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*               board = context->GetBoard();
    std::set<FOOTPRINT*> fps = targets( board, aArgs );

    if( fps.empty() )
        return KOPENAPI_RESULT::Error( 404, "no footprint matches ref / refs / lib_id" );

    BOARD_COMMIT   commit( context->GetToolManager() );
    nlohmann::json embedded = nlohmann::json::array();
    nlohmann::json errors = nlohmann::json::array();

    for( FOOTPRINT* fp : fps )
    {
        bool modified = false;

        for( FP_3DMODEL& m : fp->Models() )
        {
            // cut copies (trimmed leads) are derived per project, never embedded
            if( m.m_Filename.StartsWith( EMBED_PREFIX ) || m.m_Filename.Contains( wxS( "/.trimmed/" ) ) )
                continue;

            const std::filesystem::path file = KopenapiModelFile( fp, m, board->GetProject() );

            if( !std::filesystem::exists( file ) )
            {
                errors.push_back( { { "ref", str( fp->GetReference() ) }, { "model", str( m.m_Filename ) },
                                    { "error", "file not found" } } );
                continue;
            }

            if( !modified )
                commit.Modify( fp );

            modified = true;
            EMBEDDED_FILES::EMBEDDED_FILE* ef =
                    fp->GetEmbeddedFiles()->AddFile( wxFileName( wxString::FromUTF8( file.string() ) ), true );

            if( !ef )
            {
                errors.push_back( { { "ref", str( fp->GetReference() ) }, { "model", str( m.m_Filename ) },
                                    { "error", "cannot embed" } } );
                continue;
            }

            m.m_Filename = ef->GetLink();
            embedded.push_back( { { "ref", str( fp->GetReference() ) }, { "file", str( ef->GetLink() ) },
                                  { "bytes", (int64_t) ef->decompressedData.size() } } );
        }
    }

    if( !embedded.empty() )
        commit.Push( _( "Embed 3D models (API)" ) );

    return KOPENAPI_RESULT::Ok( { { "embedded", embedded }, { "errors", errors } } );
}


static KOPENAPI_RESULT h_pcb_lib_footprint_save( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const wxString library = wxString::FromUTF8( aArgs.value( "library", std::string() ) );

    if( library.IsEmpty() || !library.EndsWith( wxS( ".pretty" ) ) || !wxFileName( library ).IsAbsolute() )
        return KOPENAPI_RESULT::Error( 400, "library: absolute path of a .pretty directory" );

    std::set<FOOTPRINT*> fps = targets( context->GetBoard(), aArgs );

    if( fps.size() != 1 )
        return KOPENAPI_RESULT::Error( 400, "give exactly one footprint (ref)" );

    FOOTPRINT* source = *fps.begin();
    const wxString name = aArgs.contains( "name" ) ? wxString::FromUTF8( aArgs["name"].get<std::string>() )
                                                   : source->GetFPID().GetLibItemName().wx_str();
    const wxString file = library + wxFileName::GetPathSeparator() + name + wxS( "." ) + FILEEXT::KiCadFootprintFileExtension;

    if( wxFileName::FileExists( file ) && !aArgs.value( "overwrite", false ) )
        return KOPENAPI_RESULT::Error( 409, str( file ) + " exists (overwrite: true)" );

    std::unique_ptr<FOOTPRINT> copy( static_cast<FOOTPRINT*>( source->Duplicate( IGNORE_PARENT_GROUP ) ) );
    copy->SetReference( wxS( "REF**" ) );
    copy->SetParentGroup( nullptr );

    // library state: no cut copies of models (they belong to a project), generic models shown
    std::vector<FP_3DMODEL> models;
    const bool hadCut = std::any_of( copy->Models().begin(), copy->Models().end(),
                                     []( const FP_3DMODEL& m ) { return m.m_Filename.Contains( wxS( "/.trimmed/" ) ); } );

    for( FP_3DMODEL m : copy->Models() )
    {
        if( m.m_Filename.Contains( wxS( "/.trimmed/" ) ) )
            continue;

        if( hadCut )
            m.m_Show = true;

        models.push_back( m );
    }

    copy->Models() = models;
    LIB_ID id = copy->GetFPID();
    id.SetLibItemName( name );
    copy->SetFPID( id );

    try
    {
        PCB_IO_KICAD_SEXPR io;

        if( !wxDir::Exists( library ) )
            io.CreateLibrary( library );

        io.FootprintSave( library, copy.get() );
    }
    catch( const IO_ERROR& e )
    {
        return KOPENAPI_RESULT::Error( 500, str( e.What() ) );
    }

    nlohmann::json modelList = nlohmann::json::array();

    for( const FP_3DMODEL& m : copy->Models() )
        modelList.push_back( str( m.m_Filename ) );

    return KOPENAPI_RESULT::Ok( { { "file", str( file ) }, { "name", str( name ) }, { "models", modelList },
                                  { "embedded_files", copy->GetEmbeddedFiles()->EmbeddedFileMap().size() } } );
}


KOPENAPI_REGISTER( "pcb_footprint_models_embed",
                   "Embed the 3D model files of board footprints into the footprints themselves "
                   "(kicad-embed://...): the footprint carries its model, no path variables needed; "
                   "refs or lib_id",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"}},
                        "lib_id":{"type":"string"}}})json"_json,
                   false, h_pcb_footprint_models_embed, 300 );

KOPENAPI_REGISTER( "pcb_lib_footprint_save",
                   "Save a board footprint into a footprint library (.pretty directory, created if "
                   "missing) in library form: origin, rotation 0, top side, REF**, no project-only "
                   "model copies; keeps its 3D models (embed them first for a self-contained footprint)",
                   R"json({"type":"object","required":["ref","library"],"properties":{
                        "ref":{"type":"string"},
                        "library":{"type":"string","description":"absolute path of the .pretty directory"},
                        "name":{"type":"string","description":"footprint name; default the board footprint's library name"},
                        "overwrite":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_lib_footprint_save, 300 );

KOPENAPI_MARK_EDITING( "pcb_footprint_models_embed" );
