/// @file kicadopenapi_design_blocks.h
/// @brief kicadopenapi design block libraries, shared by the board's and the schematic's design
/// block methods: a library by table nickname or .kicad_blocks path, its blocks listed, loaded and
/// saved.
#ifndef KICADOPENAPI_DESIGN_BLOCKS_H
#define KICADOPENAPI_DESIGN_BLOCKS_H

#include <json_common.h>
#include <kicadopenapi_registry.h>
#include <kicommon.h>

#include <wx/string.h>

#include <memory>
#include <optional>
#include <string>

class DESIGN_BLOCK;
class PROJECT;


/// @brief A design block library: a nickname of the project's / global table, or a .kicad_blocks
/// directory by its absolute path
struct KICOMMON_API KOPENAPI_DB_LIBRARY
{
    wxString nickname;   ///< the table's name, or the directory's name without extension
    wxString path;       ///< the directory (path form only)

    bool IsPath() const { return !path.IsEmpty(); }

    /// @brief The path, or the nickname
    std::string Name() const;
};


/// @brief The library of aArgs["library"]; an error text (aStatus 400 / 404) when it names neither
/// a table library nor an absolute .kicad_blocks path (aMustExist: the directory is there).
/// aListMethod is named in the error as where the libraries are listed.
KICOMMON_API std::optional<KOPENAPI_DB_LIBRARY> KopenapiDesignBlockLibrary( PROJECT* aProject, const nlohmann::json& aArgs,
                                                                         bool aMustExist, const std::string& aListMethod,
                                                                         std::string& aError, int& aStatus );

/// @brief The library can be written (a path always; a table library unless read-only)
KICOMMON_API bool KopenapiDesignBlockLibraryWritable( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib );

KICOMMON_API bool KopenapiDesignBlockExists( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib, const wxString& aName );

/// @brief The block, loaded (its files' paths in the library, its lib id set); nullptr with aError
KICOMMON_API std::unique_ptr<DESIGN_BLOCK> KopenapiLoadDesignBlock( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib,
                                                                    const wxString& aName, std::string& aError );

/// @brief Write aBlock into the library (its board / schematic files are copied from where they
/// are; a file it does not name stays as the library has it). Throws IO_ERROR.
KICOMMON_API void KopenapiSaveDesignBlock( PROJECT* aProject, const KOPENAPI_DB_LIBRARY& aLib, const DESIGN_BLOCK& aBlock );

/// @brief {name, lib_id, description, keywords, has_layout, has_schematic}
KICOMMON_API nlohmann::json KopenapiDesignBlockJson( const DESIGN_BLOCK& aBlock );

/// @brief The design block list method: without library the tables' libraries, with library its
/// blocks
KICOMMON_API KOPENAPI_RESULT KopenapiDesignBlockList( PROJECT* aProject, const nlohmann::json& aArgs,
                                                      const std::string& aListMethod );

/// @brief The list method's input schema
KICOMMON_API const nlohmann::json& KopenapiDesignBlockListSchema();

#endif
