/// @file kopenapi_occ.h
/// @brief 3D model geometry for inspection and seating (OpenCascade, pcbnew kiface).
#ifndef KOPENAPI_OCC_H
#define KOPENAPI_OCC_H

#include <glm/glm.hpp>

#include <filesystem>
#include <mutex>
#include <string>
#include <vector>


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
    double      seat = 0;         ///< lowest z outside the given drill holes: where it rests (needs aHoles)
    bool        hasSeat = false;
    double      seatAt[2] = { 0, 0 }; ///< x, y of that lowest point (footprint frame)
    double      namedBodyBottom = 0; ///< lowest z of the solids named "body" in the file (role-named models)
    bool        hasNamedBody = false;
    double      seconds = 0;      ///< time spent (0 when cached)
};


/// @brief Reads a STEP model, places it by aMatrix (CalcModelMatrix) and measures it; cached
/// per file (and its mtime) and matrix. Geometry only: callable off the main thread.
/// @param aStep STEP file
/// @param aMatrix model matrix (footprint frame)
/// @param aBody false measures the box only (bodyBottom / hasLeads unset)
/// @param aHoles the footprint's drill holes (x, y, radius in the footprint frame, mm): only through
/// them may the part cross the board; measures seat
/// @param aThickness board thickness (mm): below the board's bottom anything goes (clips, splayed legs)
/// @param aBoard the board outline (rings in the footprint frame, mm; even-odd) for a part that reaches
/// over the board edge: what hangs beyond the edge may go below the top; empty = all of it is on the board
KOPENAPI_MODEL_GEOMETRY KopenapiModelGeometry( const std::filesystem::path& aStep, const glm::mat4& aMatrix,
                                               bool aBody = true, const std::vector<glm::vec3>& aHoles = {},
                                               double aThickness = 1.6,
                                               const std::vector<std::vector<glm::vec2>>& aBoard = {} );

/// @brief The STEP translators (readers, writers, Interface_Static parameters, the XCAF
/// application's documents) keep global state: every STEP read / write and XCAF document
/// creation / close of the kicadopenapi holds this; the geometry work between them runs in parallel
std::mutex& KopenapiStepTranslatorMutex();

#endif
