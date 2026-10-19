/// @file legacy_formats.h
/// @brief Rewrites a file written by this KiCad into the s-expression format an earlier stable
/// release reads. One entry point; per target a chain of down-steps (v10/: this tree -> 10.0,
/// v9/: 10.0 -> 9.0) keyed by the format version bumps. What a target cannot hold is dropped
/// or approximated and listed in the report.
#ifndef LEGACY_FORMATS_H
#define LEGACY_FORMATS_H

#include <functional>
#include <string>
#include <vector>


namespace LEGACY_FORMATS
{

enum class KIND
{
    BOARD,          ///< .kicad_pcb
    FOOTPRINT,      ///< .kicad_mod
    SCHEMATIC,      ///< .kicad_sch
    SYMBOL_LIB      ///< .kicad_sym
};


struct RESULT
{
    bool                     ok = false;
    std::string              text;          ///< converted file (compact; the caller prettifies)
    std::string              error;
    int                      fromVersion = 0;
    int                      toVersion = 0;
    std::vector<std::string> changes;       ///< what was dropped or approximated, with counts
};


/// @brief What the codec takes from its host (KiCad's base64 / zstd / hashes); without a service the
/// affected items are left as they are and the report says so.
struct HOST
{
    /// Checksum the target release computes for an embedded file, from its (data) text (base64 of
    /// the zstd frame, without the bars); false when the data cannot be decoded
    std::function<bool( const std::string& aEncoded, std::string& aChecksum )> legacyEmbeddedChecksum;
};


/// @return the targets a file can be written for, newest first ("10.0", ...)
std::vector<std::string> Targets();

/// @return the kind for a file extension (without dot); false when not an s-expression design file
bool KindFromExtension( const std::string& aExtension, KIND& aKind );

/// @brief Convert aText (a whole file of kind aKind, as this tree writes it) for aTarget
RESULT Convert( KIND aKind, const std::string& aText, const std::string& aTarget,
                const HOST& aHost = HOST() );

} // namespace LEGACY_FORMATS

#endif
