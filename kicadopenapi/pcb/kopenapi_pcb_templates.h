/// @file kopenapi_pcb_templates.h
/// @brief Support roles and support-part templates for L2 placement (methods_pcb_support_roles.cpp,
/// methods_pcb_templates.cpp).
#pragma once

#include <nlohmann/json.hpp>

#include <map>
#include <optional>
#include <string>

class BOARD;
class FOOTPRINT;
class PAD;
struct KOPENAPI_CONTEXT;

/// @brief pcb_support_roles' answer: parts [{ref, role, source, pin_net, pin_role, ...}], unassigned, counts
nlohmann::json KopenapiSupportRoles( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, const std::string& aRefs );

/// @brief Capacitance in farads from a value text, or < 0
double KopenapiCapacitance( const std::string& aValue );

/// @brief The merged templates of all layers
struct KOPENAPI_TEMPLATES
{
    nlohmann::json templates = nlohmann::json::array();   ///< [{pin_role, support_role, fields, source}]
    nlohmann::json files = nlohmann::json::object();       ///< layer -> where it is read
    nlohmann::json problems = nlohmann::json::array();     ///< skipped files / templates and why

    /// @brief The template of (pin role, support role), else the support role's "*" one, else nullptr
    const nlohmann::json* Find( const std::string& aPinRole, const std::string& aSupportRole ) const;
};

/// @brief Library, user, project and (when an array) call layers merged field by field
KOPENAPI_TEMPLATES KopenapiLoadTemplates( BOARD* aBoard, const nlohmann::json& aCall );

/// @brief <project>.kicad_templates of the board's project, or "" without a project file
std::string KopenapiTemplatesProjectFile( BOARD* aBoard );

/// @brief Empty when the distance expression is valid, else why not
std::string KopenapiTemplateExpressionError( const std::string& aExpr, bool aLiteralAllowed, bool aNeverAllowed );

/// @brief A distance expression resolved at an anchor pin for a part, in IU.  The rule tokens (via_pitch,
/// track_pitch, pad_to_pad, courtyard, escape) come from the board's rules; the geometry ones (ring, ports_band,
/// loop, rail_entry, reach) from aContext.  aTerms gets {term, mm, from} per term; nullopt with aError when a
/// token has no value here.
std::optional<int> KopenapiTemplateDistance( BOARD* aBoard, const PAD* aPin, const FOOTPRINT* aPart,
                                             const std::string& aExpr, const std::map<std::string, int>& aContext,
                                             nlohmann::json& aTerms, std::string& aError );
