#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

/// @brief One pin's escape on a side's via line (part-fanout.md): where its pad is along the side, how wide it
/// is at the line, what it keeps clear, how far its leg may move it and the spans it may not take
struct PART_FAN_SLOT
{
    int64_t                                  pin = 0;         ///< the pad's centre along the side
    int                                      half = 0;        ///< half its width at the line (a via's radius, half a track)
    int                                      clearance = 0;   ///< what it keeps to its neighbours
    int64_t                                  reach = 0;       ///< how far along its leg may take it from its pin
    std::vector<std::pair<int64_t, int64_t>> blocked;         ///< spans its centre may not take (open intervals)
};

/// @brief The spots of a side's escapes on its via line, in pin order
struct PART_FAN_ASSIGNMENT
{
    std::vector<std::optional<int64_t>> at;   ///< per slot its spot along the line; none: no room for it
    int64_t                             moved = 0;   ///< the sum of the spots' distances from their pins
    int                                 left = 0;    ///< how many have no spot
};

/// @brief The spots of every slot of a side at once: in pin order (no two cross), each two placed neighbours
/// their halves plus the larger clearance apart and no nearer than their pins (their legs never converge), each
/// within its reach of its pin and inside [aLo, aHi], none in a blocked span; as few left out as can be, then the
/// least total move.  Exact on a grid of aStep along the line (dynamic programming over the slots and the grid)
PART_FAN_ASSIGNMENT PartFanAssign( const std::vector<PART_FAN_SLOT>& aSlots, int64_t aLo, int64_t aHi, int aStep );
