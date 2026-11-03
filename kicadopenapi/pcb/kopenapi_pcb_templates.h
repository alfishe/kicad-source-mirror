/// @file kopenapi_pcb_templates.h
/// @brief Support roles and support-part templates for L2 placement (methods_pcb_support_roles.cpp,
/// methods_pcb_templates.cpp).
#pragma once

#include <nlohmann/json.hpp>

#include <string>

class BOARD;
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
