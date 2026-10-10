/// @file kopenapi_occ.h
/// @brief 3D model geometry for inspection and seating (OpenCascade, pcbnew kiface).
#ifndef KOPENAPI_OCC_H
#define KOPENAPI_OCC_H

#include <glm/glm.hpp>

#include <filesystem>
#include <string>


/// @brief Geometry of one 3D model placed by its model matrix, in the footprint's 3D frame
/// (mm, x right, y up, z out of the board's top surface; the board top is z = 0)
struct KOPENAPI_MODEL_GEOMETRY
{
    bool        ok = false;
    std::string error;
    double      min[3] = { 0, 0, 0 };
    double      max[3] = { 0, 0, 0 };
    double      bodyBottom = 0;   ///< lowest z where the body starts (above leads / pins)
    bool        hasLeads = false; ///< thin parts (pins) reach below the body
    bool        hasBody = false;  ///< bodyBottom and hasLeads are measured (not for a box-only request)
    double      seconds = 0;      ///< time spent (0 when cached)
};


/// @brief Reads a STEP model, places it by aMatrix (CalcModelMatrix) and measures it; cached
/// per file (and its mtime) and matrix. Geometry only: callable off the main thread.
/// @param aStep STEP file
/// @param aMatrix model matrix (footprint frame)
/// @param aBody false measures the box only (bodyBottom / hasLeads unset)
KOPENAPI_MODEL_GEOMETRY KopenapiModelGeometry( const std::filesystem::path& aStep, const glm::mat4& aMatrix,
                                               bool aBody = true );

#endif
