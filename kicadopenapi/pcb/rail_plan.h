#pragma once

#include <geometry/seg.h>
#include <geometry/shape_poly_set.h>
#include <layer_ids.h>
#include <math/vector2d.h>
#include <math/box2.h>
#include <optional>
#include <string>
#include <vector>

class BOARD;
class FOOTPRINT;
class NETINFO_ITEM;
class PAD;

/// @brief The way out of a pad of a part's outer row: the axis of its side, pointing away from the part
/// (none for a pad inside the rows: an exposed pad, a ball inside an array)
std::optional<VECTOR2I> outwardOf( const PAD* aPad, const BOX2I& aCentres );

/// @brief Under a quad package's body (QFP / QFN / LGA, pads on four sides): the area inside its
/// ring of pads on its side, where a track or via of another net walls its pins in.  With an
/// exposed pad the area is the ring between the pad rows and the exposed pad (the exposed pad and
/// its clearance stay open: its own vias go there); parts with an array of pads inside (BGA) have
/// no such area
struct UNDER_PART
{
    FOOTPRINT*     fp;
    PCB_LAYER_ID   layer;
    SHAPE_POLY_SET area;
    bool           exposedPad;
};

std::optional<UNDER_PART> underPartArea( BOARD* aBoard, FOOTPRINT* aFp );
std::vector<UNDER_PART>   underPartAreas( BOARD* aBoard );

/// @brief The board's smallest through via within its rules (min diameter, min drill + 2 x min
/// annular ring, the DRC rules' minimums for aNet), when smaller than the class's aClassDia
/// @return false when the board allows none smaller
bool boardMinVia( BOARD* aBoard, NETINFO_ITEM* aNet, int aClassDia, int aClassDrill, int& aDia, int& aDrill );

struct RAIL_PLAN_OPTIONS
{
    int classViaDia = 0;     ///< the via the stitched net's class gives
    int classViaDrill = 0;
    int minViaDia = 0;       ///< the board's smallest (boardMinVia); 0: none
    int minViaDrill = 0;
};

/// @brief One rail pin of a fine-pitch row of a quad package and its via inward
struct RAIL_PIN_PLAN
{
    const PAD*  pad = nullptr;
    VECTOR2I    outward;           ///< the axis of the pad's side, away from the part
    bool        planned = false;   ///< spot, dia, drill, stub set
    VECTOR2I    spot;
    int         dia = 0;
    int         drill = 0;
    SEG         stub;              ///< pad centre to spot, on the pad's layer
    std::string pattern;           ///< one_depth, alternating, every_other, fixed (a via there already)
    bool        skipped = false;   ///< left to the router (reason)
    std::string reason;
};

/// @brief The vias inward of the rail pins of aFp's fine-pitch rows (power pins, pins of a poured
/// net or of the exposed pad's net), from the board alone so that every caller sees the same plan.
/// Each run of rail pins side by side gets one pattern: one depth; alternating depths; or, where
/// the pitch and the hole clearance leave no stub between two vias a pin apart, a via at every
/// other pin (poured nets first, the others skipped).  The class's via is tried before the
/// board's smallest.  A pin already joined to a via (directly or by its stub) keeps it (fixed) and
/// its neighbours keep clear of it; spots and stubs stay out of rule areas that forbid them.
/// Pins of one net side by side: only the first is planned, the others join its via
std::vector<RAIL_PIN_PLAN> planRowRailVias( BOARD* aBoard, FOOTPRINT* aFp, const RAIL_PLAN_OPTIONS& aOptions );
