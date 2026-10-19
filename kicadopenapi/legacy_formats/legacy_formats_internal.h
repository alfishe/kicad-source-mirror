/// @file legacy_formats_internal.h
/// @brief Shared pieces of the legacy format rules: a lossless s-expression tree, the change
/// report, number formatting and the per-target step entry points.
#ifndef LEGACY_FORMATS_INTERNAL_H
#define LEGACY_FORMATS_INTERNAL_H

#include "legacy_formats.h"

#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>


namespace LEGACY_FORMATS
{

/// @brief An atom (raw text, quotes and escapes kept) or a list
struct NODE
{
    bool              isList = false;
    std::string       atom;
    std::vector<NODE> items;

    static NODE Atom( const std::string& aText );
    static NODE List( const std::string& aHead );

    /// @return the first atom of a list, "" otherwise
    const std::string& Head() const;
    bool               Is( const char* aHead ) const { return isList && Head() == aHead; }

    /// @return the first child list with that head, nullptr when none
    NODE*       Child( const char* aHead );
    const NODE* Child( const char* aHead ) const;

    /// @return atom i of a list, "" when absent or a list
    const std::string& AtomAt( size_t aIndex ) const;

    /// @return true when the list holds the bare atom
    bool HasAtom( const char* aAtom ) const;

    /// @brief Remove child lists with that head; @return how many
    int RemoveChildren( const char* aHead );

    /// @brief Remove bare atoms with that text; @return how many
    int RemoveAtoms( const char* aAtom );
};


bool        Parse( const std::string& aText, NODE& aRoot, std::string& aError );
std::string Write( const NODE& aNode );

std::string Unquote( const std::string& aRaw );
std::string Quote( const std::string& aText );

/// @brief Aggregates "what changed" lines with counts, in first-seen order
class REPORT
{
public:
    void                     Note( const std::string& aWhat, int aCount = 1 );
    std::vector<std::string> Lines() const;

private:
    std::vector<std::string>   m_order;
    std::map<std::string, int> m_counts;
};


struct CONTEXT
{
    KIND        kind;
    std::string target;     ///< target of the step being applied
    int         version;    ///< file version before the step
    REPORT&     report;
    const HOST& host;
};


bool        ParseNumber( const std::string& aAtom, double& aValue );

/// @brief Millimetres for an integer count of internal units (aIuPerMm 1e6 board, 1e4 schematic)
std::string FormatMM( long long aIu, long long aIuPerMm );
long long   ToIU( double aMM, long long aIuPerMm );
std::string FormatAngle( double aDegrees );

/// @brief KiCad's RotatePoint() on integer coordinates (exact for multiples of 90 degrees)
void RotatePoint( long long& aX, long long& aY, double aDegrees );

/// @brief Base64 decode, whitespace skipped; false on a bad character
bool Base64Decode( const std::string& aText, std::string& aBytes );

/// @brief Image density as KiCad's BITMAP_BASE reads it: ppi now (rounded pixels per cm)
/// and before 20260623 (pixels per cm truncated first). False when the image has none.
bool ImageDensity( const std::string& aBytes, int& aPpi, int& aLegacyPpi );

/// @brief Tokens a target's readers know; aSet "pcb", "plot" or "sch"
const std::set<std::string>& TargetKeywords( const std::string& aTarget, const std::string& aSet );

/// @brief Drop lists whose head the target does not know (generator properties excepted)
void DropUnknownTokens( NODE& aRoot, CONTEXT& aCtx );

/// @brief Point list on an ellipse (closed) or elliptical arc (open), in file coordinates.
/// aInvertY: the file stores y up (symbol bodies).
std::vector<std::pair<long long, long long>> EllipsePoints( long long aCx, long long aCy, long long aMajor,
                                                            long long aMinor, double aRotation, bool aArc,
                                                            double aStart, double aEnd, bool aInvertY,
                                                            long long aMaxError );

/// @brief The step from this tree's format to 10.0 (v10/)
bool StepTo10( NODE& aRoot, CONTEXT& aCtx, std::string& aError );

} // namespace LEGACY_FORMATS

#endif
