/// @file kopenapi_support_roles.h
/// @brief The Support.Role symbol / footprint field and its values, shared by pcb_support_roles and the
/// sch_support_roles methods.
#pragma once

#include <set>
#include <string>

inline const char* const KOPENAPI_SUPPORT_ROLE_FIELD = "Support.Role";

inline const std::set<std::string>& KopenapiSupportRoleValues()
{
    static const std::set<std::string> roles = { "decoupling", "bulk", "filter", "inductor", "series", "pullup",
                                                 "crystal", "crystal_load", "esd", "none" };
    return roles;
}
