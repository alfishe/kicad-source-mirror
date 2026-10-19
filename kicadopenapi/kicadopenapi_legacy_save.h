/// @file kicadopenapi_legacy_save.h
/// @brief Saving copies for an earlier stable KiCad: the file this build wrote is rewritten in
/// place by legacy_formats/ and prettified as KiCad writes it.
#ifndef KICADOPENAPI_LEGACY_SAVE_H
#define KICADOPENAPI_LEGACY_SAVE_H

#include <json_common.h>
#include <kicommon.h>

#include <wx/string.h>


/// @return the targets an earlier-version copy can be written for ("10.0", ...)
KICOMMON_API nlohmann::json KopenapiLegacyTargets();

/// @brief Rewrite aPath (.kicad_pcb / .kicad_sch / .kicad_sym / .kicad_mod) for aTarget.
/// @param aReport receives { file, from_version, to_version, changes[] }
/// @return false with aError set when the file cannot be read, converted or written
KICOMMON_API bool KopenapiLegacyConvertFile( const wxString& aPath, const std::string& aTarget,
                                             nlohmann::json& aReport, wxString& aError );

/// @brief Rewrite a project file written by SaveProjectCopy for aTarget: schema versions, and DRC
/// exclusions (aDrcExclusions: the target's [marker string, comment] pairs; null drops new-style
/// entries). ERC exclusions in the new form are dropped and reported.
KICOMMON_API bool KopenapiLegacyConvertProject( const wxString& aPath, const std::string& aTarget,
                                                const nlohmann::json* aDrcExclusions, nlohmann::json& aReport,
                                                wxString& aError );

/// @brief Version-save arguments: aTarget from args "version"; "" when absent.
/// @return false with aError when the version is not a supported target
KICOMMON_API bool KopenapiLegacyTarget( const nlohmann::json& aArgs, std::string& aTarget, std::string& aError );

#endif
