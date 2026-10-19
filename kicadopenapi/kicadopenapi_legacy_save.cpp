#include "kicadopenapi_legacy_save.h"

#include "legacy_formats/legacy_formats.h"

#include <kicadopenapi_registry.h>

#include <embedded_files.h>
#include <io/kicad/kicad_io_utils.h>
#include <mmh3_hash.h>

#include <zstd.h>

#include <wx/base64.h>
#include <wx/dir.h>
#include <wx/ffile.h>
#include <wx/filefn.h>
#include <wx/filename.h>


/// MurmurHash3 as 10.0 and earlier computed it (tail bytes misaligned), over the zstd content
static bool legacyEmbeddedChecksum( const std::string& aEncoded, std::string& aChecksum )
{
    wxMemoryBuffer compressed = wxBase64Decode( aEncoded );

    if( compressed.IsEmpty() )
        return false;

    unsigned long long size = ZSTD_getFrameContentSize( compressed.GetData(), compressed.GetDataLen() );

    if( size == ZSTD_CONTENTSIZE_ERROR || size == ZSTD_CONTENTSIZE_UNKNOWN || size > 1000000000ULL )
        return false;

    std::vector<uint8_t> content( size );
    size_t done = ZSTD_decompress( content.data(), content.size(), compressed.GetData(), compressed.GetDataLen() );

    if( ZSTD_isError( done ) )
        return false;

    MMH3_HASH hash( EMBEDDED_FILES::Seed() );
    hash.addDataV1( content.data(), done );
    aChecksum = hash.digest().ToString();
    return true;
}


nlohmann::json KopenapiLegacyTargets()
{
    return LEGACY_FORMATS::Targets();
}


bool KopenapiLegacyTarget( const nlohmann::json& aArgs, std::string& aTarget, std::string& aError )
{
    aTarget.clear();

    if( !aArgs.contains( "version" ) || aArgs["version"].is_null() )
        return true;

    if( !aArgs["version"].is_string() )
    {
        aError = "'version' must be a string";
        return false;
    }

    std::string version = aArgs["version"].get<std::string>();

    for( const std::string& target : LEGACY_FORMATS::Targets() )
    {
        if( version == target )
        {
            aTarget = target;
            return true;
        }
    }

    aError = "unsupported version '" + version + "'; supported: " + KopenapiLegacyTargets().dump();
    return false;
}


bool KopenapiLegacyConvertFile( const wxString& aPath, const std::string& aTarget, nlohmann::json& aReport,
                                wxString& aError )
{
    wxFileName           fn( aPath );
    LEGACY_FORMATS::KIND kind;

    if( !LEGACY_FORMATS::KindFromExtension( fn.GetExt().ToStdString(), kind ) )
    {
        aError = wxS( "not a KiCad s-expression design file: " ) + aPath;
        return false;
    }

    std::string text;

    {
        wxFFile in( aPath, wxS( "rb" ) );

        if( !in.IsOpened() )
        {
            aError = wxS( "cannot read " ) + aPath;
            return false;
        }

        wxFileOffset length = in.Length();
        text.resize( length > 0 ? size_t( length ) : 0 );

        if( length > 0 && in.Read( text.data(), text.size() ) != text.size() )
        {
            aError = wxS( "cannot read " ) + aPath;
            return false;
        }
    }

    LEGACY_FORMATS::HOST host;
    host.legacyEmbeddedChecksum = legacyEmbeddedChecksum;

    LEGACY_FORMATS::RESULT result = LEGACY_FORMATS::Convert( kind, text, aTarget, host );

    if( !result.ok )
    {
        aError = aPath + wxS( ": " ) + wxString::FromUTF8( result.error );
        return false;
    }

    aReport = { { "file", aPath.ToUTF8().data() },
                { "from_version", result.fromVersion },
                { "to_version", result.toVersion },
                { "changes", result.changes } };

    if( result.toVersion == result.fromVersion )
        return true;

    KICAD_FORMAT::Prettify( result.text );

    wxString temp = aPath + wxS( ".tmp" );

    {
        wxFFile out( temp, wxS( "wb" ) );

        if( !out.IsOpened() || !out.Write( result.text.data(), result.text.size() ) || !out.Close() )
        {
            wxRemoveFile( temp );
            aError = wxS( "cannot write " ) + temp;
            return false;
        }
    }

    if( !wxRenameFile( temp, aPath, true ) )
    {
        wxRemoveFile( temp );
        aError = wxS( "cannot replace " ) + aPath;
        return false;
    }

    return true;
}


static bool copyForTarget( const wxString& aFrom, const wxString& aTo, const std::string& aTarget,
                           nlohmann::json& aFiles, wxString& aError )
{
    if( !wxCopyFile( aFrom, aTo, true ) )
    {
        aError = wxS( "cannot write " ) + aTo;
        return false;
    }

    nlohmann::json report;

    if( !KopenapiLegacyConvertFile( aTo, aTarget, report, aError ) )
        return false;

    aFiles.push_back( report );
    return true;
}


static KOPENAPI_RESULT h_file_save_version( KOPENAPI_CONTEXT&, const nlohmann::json& aArgs )
{
    std::string target;
    std::string error;

    if( !KopenapiLegacyTarget( aArgs, target, error ) )
        return KOPENAPI_RESULT::Error( 400, error );

    if( target.empty() )
        return KOPENAPI_RESULT::Error( 400, "'version' is required; supported: " + KopenapiLegacyTargets().dump() );

    const wxString from = wxString::FromUTF8( aArgs.value( "path", std::string() ) );
    const wxString to = wxString::FromUTF8( aArgs.value( "output", std::string() ) );

    if( from.IsEmpty() || to.IsEmpty() )
        return KOPENAPI_RESULT::Error( 400, "'path' and 'output' are required" );

    wxFileName fromName( from );
    wxFileName toName( to );
    fromName.Normalize( wxPATH_NORM_ABSOLUTE | wxPATH_NORM_DOTS );
    toName.Normalize( wxPATH_NORM_ABSOLUTE | wxPATH_NORM_DOTS );

    if( fromName.GetFullPath() == toName.GetFullPath() )
        return KOPENAPI_RESULT::Error( 409, "'output' must differ from 'path' (the copy is converted, the source kept)" );

    nlohmann::json files = nlohmann::json::array();
    wxString       failure;

    if( wxFileName::DirExists( fromName.GetFullPath() ) )
    {
        if( !wxFileName::DirExists( toName.GetFullPath() )
            && !wxFileName::Mkdir( toName.GetFullPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
            return KOPENAPI_RESULT::Error( 500, "cannot create " + std::string( toName.GetFullPath().ToUTF8() ) );

        wxArrayString entries;
        wxDir::GetAllFiles( fromName.GetFullPath(), &entries, wxEmptyString, wxDIR_FILES );
        entries.Sort();

        for( const wxString& entry : entries )
        {
            wxFileName           file( entry );
            LEGACY_FORMATS::KIND kind;
            wxFileName           out( toName.GetFullPath(), file.GetFullName() );

            if( !LEGACY_FORMATS::KindFromExtension( file.GetExt().ToStdString(), kind ) )
            {
                if( !wxCopyFile( entry, out.GetFullPath(), true ) )
                    return KOPENAPI_RESULT::Error( 500, "cannot write " + std::string( out.GetFullPath().ToUTF8() ) );

                continue;
            }

            if( !copyForTarget( entry, out.GetFullPath(), target, files, failure ) )
                return KOPENAPI_RESULT::Error( 500, std::string( failure.ToUTF8() ) );
        }
    }
    else if( wxFileName::FileExists( fromName.GetFullPath() ) )
    {
        LEGACY_FORMATS::KIND kind;

        if( !LEGACY_FORMATS::KindFromExtension( fromName.GetExt().ToStdString(), kind ) )
            return KOPENAPI_RESULT::Error( 400, "not a .kicad_sym / .kicad_mod / .kicad_pcb / .kicad_sch file" );

        if( !wxFileName::DirExists( toName.GetPath() )
            && !wxFileName::Mkdir( toName.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
            return KOPENAPI_RESULT::Error( 500, "cannot create " + std::string( toName.GetPath().ToUTF8() ) );

        if( !copyForTarget( fromName.GetFullPath(), toName.GetFullPath(), target, files, failure ) )
            return KOPENAPI_RESULT::Error( 500, std::string( failure.ToUTF8() ) );
    }
    else
    {
        return KOPENAPI_RESULT::Error( 404, "no file or directory " + std::string( fromName.GetFullPath().ToUTF8() ) );
    }

    return KOPENAPI_RESULT::Ok( { { "path", std::string( toName.GetFullPath().ToUTF8() ) },
                                  { "version", target },
                                  { "files", files } } );
}


KOPENAPI_REGISTER( "file_save_version",
                   "Write a copy of a design file this KiCad wrote (.kicad_sym, .kicad_mod, a .pretty "
                   "library directory, or a single .kicad_pcb / .kicad_sch) for an earlier stable KiCad "
                   "('version'); the source is kept. Report per file: from / to format version and what "
                   "was dropped or approximated. Open boards and schematics: pcb_save / sch_save with "
                   "'version' (they write the whole hierarchy and the project)",
                   nlohmann::json( { { "type", "object" },
                                     { "required", { "path", "output", "version" } },
                                     { "properties",
                                       { { "path", { { "type", "string" } } },
                                         { "output", { { "type", "string" },
                                                       { "description", "file, or directory for a .pretty source" } } },
                                         { "version", { { "type", "string" }, { "enum", KopenapiLegacyTargets() } } } } } } ),
                   false, h_file_save_version, 600 );


bool KopenapiLegacyConvertProject( const wxString& aPath, const std::string& aTarget,
                                   const nlohmann::json* aDrcExclusions, nlohmann::json& aReport,
                                   wxString& aError )
{
    nlohmann::json project;

    {
        wxFFile in( aPath, wxS( "rb" ) );
        wxString text;

        if( !in.IsOpened() || !in.ReadAll( &text, wxConvUTF8 ) )
        {
            aError = wxS( "cannot read " ) + aPath;
            return false;
        }

        project = nlohmann::json::parse( std::string( text.ToUTF8() ), nullptr, false );

        if( project.is_discarded() || !project.is_object() )
        {
            aError = wxS( "not a project file: " ) + aPath;
            return false;
        }
    }

    std::vector<std::string> changes;

    // Schema versions of 10.0: project 3, board design settings 2, ERC settings 0
    auto lowerVersion = [&]( nlohmann::json& aSection, int aVersion, const char* aWhat )
    {
        if( aSection.is_object() && aSection.contains( "meta" ) && aSection["meta"].is_object()
            && aSection["meta"].value( "version", 0 ) > aVersion )
        {
            aSection["meta"]["version"] = aVersion;
            changes.push_back( std::string( aWhat ) + " schema version " + std::to_string( aVersion ) );
        }
    };

    if( aTarget == "10.0" )
    {
        lowerVersion( project, 3, "project" );

        if( project.contains( "board" ) && project["board"].contains( "design_settings" ) )
        {
            nlohmann::json& bds = project["board"]["design_settings"];
            lowerVersion( bds, 2, "board design settings" );

            if( aDrcExclusions )
            {
                bds["drc_exclusions"] = *aDrcExclusions;

                if( !aDrcExclusions->empty() )
                    changes.push_back( "DRC exclusions written as 10.0 strings (" + std::to_string( aDrcExclusions->size() ) + ")" );
            }
            else if( bds.contains( "drc_exclusions" ) && bds["drc_exclusions"].is_array() )
            {
                nlohmann::json kept = nlohmann::json::array();

                for( const nlohmann::json& entry : bds["drc_exclusions"] )
                {
                    if( !entry.is_object() )
                        kept.push_back( entry );
                }

                if( kept.size() != bds["drc_exclusions"].size() )
                {
                    changes.push_back( "DRC exclusions dropped (save the board for 10.0 to convert them)" );
                    bds["drc_exclusions"] = kept;
                }
            }
        }

        if( project.contains( "erc" ) && project["erc"].is_object() )
        {
            nlohmann::json& erc = project["erc"];
            lowerVersion( erc, 0, "ERC settings" );

            if( erc.contains( "erc_exclusions" ) && erc["erc_exclusions"].is_array() )
            {
                nlohmann::json kept = nlohmann::json::array();

                for( const nlohmann::json& entry : erc["erc_exclusions"] )
                {
                    if( !entry.is_object() )
                        kept.push_back( entry );
                }

                if( kept.size() != erc["erc_exclusions"].size() )
                {
                    changes.push_back( "ERC exclusions dropped (" + std::to_string( erc["erc_exclusions"].size() - kept.size() )
                                       + "): 10.0 shows those violations again" );
                    erc["erc_exclusions"] = kept;
                }
            }
        }
    }

    aReport = { { "file", aPath.ToUTF8().data() }, { "changes", changes } };

    std::string text = project.dump( 2 ) + "\n";
    wxString    temp = aPath + wxS( ".tmp" );

    {
        wxFFile out( temp, wxS( "wb" ) );

        if( !out.IsOpened() || !out.Write( text.data(), text.size() ) || !out.Close() )
        {
            wxRemoveFile( temp );
            aError = wxS( "cannot write " ) + temp;
            return false;
        }
    }

    if( !wxRenameFile( temp, aPath, true ) )
    {
        wxRemoveFile( temp );
        aError = wxS( "cannot replace " ) + aPath;
        return false;
    }

    return true;
}
