/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 * @author Jon Evans <jon@craftyjon.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CONVERT_TOOL_H_
#define CONVERT_TOOL_H_

#include <geometry/shape_poly_set.h>
#include <tools/pcb_tool_base.h>
#include <pcbnew_settings.h>

class BOARD_COMMIT;
class BOARD_DESIGN_SETTINGS;
class BOARD_ITEM_CONTAINER;
class CONDITIONAL_MENU;
class FOOTPRINT;
class PCB_SELECTION_TOOL;
class PCB_SHAPE;
class PCB_BASE_FRAME;
class ZONE;
class ZONE_SETTINGS;


class CONVERT_TOOL : public PCB_TOOL_BASE
{
    friend class CONVERT_TOOL_TEST_FIXTURE;

public:
    CONVERT_TOOL();
    virtual ~CONVERT_TOOL();

    /// @copydoc TOOL_INTERACTIVE::Reset()
    void Reset( RESET_REASON aReason ) override {}

    /// @copydoc TOOL_INTERACTIVE::Init()
    bool Init() override;

    /**
     * Convert selected lines to a polygon, if possible.
     */
    int CreatePolys( const TOOL_EVENT& aEvent );

    /**
     * Convert selected polygon-like object to graphic lines, if possible.
     */
    int CreateLines( const TOOL_EVENT& aEvent );

    /**
     * Convert selected segment (graphic or track) to an arc of the same type
     */
    int SegmentToArc( const TOOL_EVENT& aEvent );

    /**
     * Convert selected items to outset versions of themselves.
     */
    int OutsetItems( const TOOL_EVENT& aEvent );

    /// @copydoc TOOL_INTERACTIVE::setTransitions()
    void setTransitions() override;

    /**
     * Build the polygons Create Polygon / Zone / Rule Area makes from @a aItems.  The items
     * that went into a polygon get SKIP_STRUCT (the ones "delete originals" removes).
     *
     * @return false when there is no polygon
     */
    static bool MakePolys( const std::deque<EDA_ITEM*>& aItems, const CONVERT_SETTINGS& aSettings,
                           int aMaxError, std::vector<SHAPE_POLY_SET>& aPolys );

    /**
     * The settings Create Polygon builds with: a line width of 0 takes the layer class default
     * (except centerline), and a bounding hull gap grows by half the line width.
     */
    static CONVERT_SETTINGS ResolvePolySettings( const CONVERT_SETTINGS& aSettings,
                                                 const BOARD_DESIGN_SETTINGS& aBds, PCB_LAYER_ID aLayer );

    /**
     * Add one graphic polygon per entry of @a aPolys (Create Polygon).  Copy-line-width takes
     * the stroke of the top-left stroked item of @a aItems.
     */
    static std::vector<PCB_SHAPE*> AddPolyShapes( BOARD_COMMIT& aCommit, const std::deque<EDA_ITEM*>& aItems,
                                                  const std::vector<SHAPE_POLY_SET>& aPolys,
                                                  const CONVERT_SETTINGS& aResolved, FOOTPRINT* aParent,
                                                  PCB_LAYER_ID aLayer );

    /**
     * Add one zone (or rule area) per entry of @a aPolys with @a aZoneSettings (Create Zone /
     * Create Rule Area).
     */
    static std::vector<ZONE*> AddZones( BOARD_COMMIT& aCommit, BOARD_ITEM_CONTAINER* aParent,
                                        const std::vector<SHAPE_POLY_SET>& aPolys,
                                        const ZONE_SETTINGS& aZoneSettings );

    /**
     * Create Lines / Create Tracks from the outlines of @a aItems (polygons, rectangles, ellipses,
     * zones; graphic segments and arcs become tracks).  @a aGraphics true makes graphic lines
     * owned by @a aGraphicParent, else tracks owned by @a aTrackParent keeping the source's net.
     */
    static std::vector<BOARD_ITEM*> AddLines( BOARD_COMMIT& aCommit, const std::deque<EDA_ITEM*>& aItems,
                                              bool aGraphics, PCB_LAYER_ID aLayer,
                                              BOARD_ITEM_CONTAINER* aTrackParent, FOOTPRINT* aGraphicParent );

    /**
     * Create Arc from a segment (graphic or track) or swap an arc's kind (graphic <-> track).
     *
     * @return the new item, nullptr for an unsupported or zero-length source
     */
    static BOARD_ITEM* AddArc( BOARD_COMMIT& aCommit, BOARD_ITEM* aSource, BOARD_ITEM_CONTAINER* aParent );

private:
    /**
     * Retrieve the start and end points for a generic item.
     *
     * @param aItem is an item that has a start and end point.
     * @return a segment from start to end, or std::nullopt if invalid.
     */
    static std::optional<SEG> getStartEndPoints( EDA_ITEM* aItem );

    /**
     * Try to make polygons from chained segments in the selected items.
     *
     * Polygons are formed from chains of lines/arcs.  Each set containing two or more lines/arcs
     * that are connected will be added to the return SHAPE_POLY_SET as an outline.  No attempt
     * is made to guess at holes.
     *
     * @param aItems is a list of items to process.
     * @param aStrategy is the conversion strategy.
     * @return a #SHAPE_POLY_SET containing any polygons that were created.
     */
    static SHAPE_POLY_SET makePolysFromChainedSegs( const std::deque<EDA_ITEM*>& aItems,
                                                    CONVERT_STRATEGY aStrategy );

    /**
     * Make polygons from graphic shapes and zones.
     *
     * @param aItems is a list of items to process.
     * @param aGap
     * @return a #SHAPE_POLY_SET containing any polygons that were created.
     */
    static SHAPE_POLY_SET makePolysFromOpenGraphics( const std::deque<EDA_ITEM*>& aItems, int aGap );
    static SHAPE_POLY_SET makePolysFromClosedGraphics( const std::deque<EDA_ITEM*>& aItems,
                                                       CONVERT_STRATEGY aStrategy );

    /**
     * Initialize the user settings for the tool.
     */
    void initUserSettings();

private:
    PCB_SELECTION_TOOL* m_selectionTool;
    CONDITIONAL_MENU*   m_menu;
    PCB_BASE_FRAME*     m_frame;
    CONVERT_SETTINGS    m_userSettings;
};

#endif
