/// @file kopenapi_sch_router.h
/// @brief kicadopenapi schematic wire router: orthogonal routes on the connection grid that do not
/// change connectivity by accident.  A route must not run along another wire, touch another
/// wire's end or another connection point (pin end, label, junction, no-connect), put a corner on
/// a wire, or cross a symbol body.  Straight, L, then Z shapes are tried; the fewest segments and
/// then the shortest wins.
#ifndef KOPENAPI_SCH_ROUTER_H
#define KOPENAPI_SCH_ROUTER_H

#include <math/box2.h>
#include <math/vector2d.h>

#include <optional>
#include <set>
#include <string>
#include <vector>

class SCH_ITEM;
class SCH_SCREEN;
class SCH_SHEET_PATH;

namespace kopenapi_sch
{

class WIRE_ROUTER
{
public:
    /// @brief Obstacles of one sheet instance
    WIRE_ROUTER( SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath, int aGrid,
                 const std::set<const SCH_ITEM*>& aIgnore = {} );

    /// @brief Route from aFrom to aTo.  aPrefer is "hv" (horizontal first) or "vh" for the L shape
    /// tried first.  Returns the polyline (first = aFrom, last = aTo), or nothing if no clear route.
    std::optional<std::vector<VECTOR2I>> Route( const VECTOR2I& aFrom, const VECTOR2I& aTo,
                                                const std::string& aPrefer, std::string* aWhy = nullptr ) const;

    /// @brief Register wires just planned (not yet on the screen) as obstacles for later routes
    void AddWire( const VECTOR2I& aStart, const VECTOR2I& aEnd );

private:
    /// @brief Empty when clear, else what is in the way and where (for the caller's error message)
    std::string obstacle( const std::vector<VECTOR2I>& aPolyline, const VECTOR2I& aFrom, const VECTOR2I& aTo ) const;

    int                                            m_grid;
    std::vector<std::pair<VECTOR2I, VECTOR2I>>     m_wires;
    std::vector<VECTOR2I>                          m_points;
    std::vector<BOX2I>                             m_bodies;
    std::vector<std::string>                       m_bodyNames;   ///< reference of each body
    std::vector<std::pair<VECTOR2I, size_t>>       m_pinOwner;    ///< pin end -> index into m_bodies
};

} // namespace kopenapi_sch

#endif
