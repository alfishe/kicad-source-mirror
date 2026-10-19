/// @file v10_common.h
/// @brief Rules shared by the board and schematic steps to 10.0
#ifndef LEGACY_FORMATS_V10_COMMON_H
#define LEGACY_FORMATS_V10_COMMON_H

#include "../legacy_formats_internal.h"


namespace LEGACY_FORMATS
{
namespace V10
{

constexpr int BOARD_VERSION = 20260206;
constexpr int SCHEMATIC_VERSION = 20260306;
constexpr int SYMBOL_LIB_VERSION = 20251024;

/// @brief A rule keyed by the bump that introduced what it undoes; apply == nullptr when the
/// unknown-token filter alone takes the construct out
struct RULE
{
    int         bump;
    const char* what;
    void ( *apply )( NODE& aRoot, CONTEXT& aCtx );
};

/// @brief Set (version N) and (generator_version "10.0") on the file's top-level list
void SetHeader( NODE& aRoot, int aVersion );

/// @brief Bold stroke-font text stores the drawn width again (thickness x 1.6)
void UnbakeBoldThickness( NODE& aRoot, long long aIuPerMm, REPORT& aReport );

/// @brief Image scale back to the pixel density the 10.0 reader computes
void RestoreImageScale( NODE& aRoot, REPORT& aReport );

/// @brief uuid string with the low 32 bits advanced by aOffset
std::string DerivedUuid( const std::string& aUuid, unsigned aOffset );

/// @brief Embedded file checksums as 10.0 computes them (through the host's hash service)
void EmbeddedChecksums( NODE& aNode, CONTEXT& aCtx );

bool BoardStep( NODE& aRoot, CONTEXT& aCtx, std::string& aError );
bool SchematicStep( NODE& aRoot, CONTEXT& aCtx, std::string& aError );

} // namespace V10
} // namespace LEGACY_FORMATS

#endif
