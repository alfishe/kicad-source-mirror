#pragma once

#include <set>
#include <string>
#include <utility>
#include <vector>

class BOARD;
class FOOTPRINT;

/// @brief How well the connections round a part get out of its neighbourhood: a quick negotiated
/// route of every non-rail connection with an end in the box round the part, on a grid of one track
/// pitch (the part's pad pitch when that is coarser, its pads on the cells' centres) over the routing layers, other nets' pads and copper and the rails' stitching vias as walls (a via takes its room on
/// every layer), the far ends outside the box
/// reached in a straight line from its edge
struct ESCAPE_ESTIMATE
{
    int    connections = 0;
    int    boxedIn = 0;       ///< no way at all from an end, other nets' copper all round it
    int    firstPassFailed = 0;   ///< no way left in one pass, shortest first, each round the ones before it
    int    overflowing = 0;   ///< still sharing a cell with another net after the negotiation
    int    vias = 0;
    int    railVias = 0;          ///< the rails' stitching vias taken as walls
    double lengthMm = 0;      ///< inside the box plus the straight rest to the far ends
    double cellMm = 0;
    int    cells = 0;
    int    rounds = 0;
    double ms = 0;
    std::vector<std::string> boxedPads;   ///< REF.PAD of the ends boxed in
    std::vector<std::pair<double, double>> railViaSpots;   ///< mm
};

/// @param aRails nets left out (they go to their planes)
/// @param aMarginMm the box: the part's courtyard grown by this much
ESCAPE_ESTIMATE EstimateEscape( BOARD* aBoard, FOOTPRINT* aPart, const std::set<int>& aRails, double aMarginMm );
