/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2013-2016 CERN
 * @author Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
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

#pragma once

#include <layer_ids.h>
#include <view/view.h>
#include <board_item.h>

class PCB_DISPLAY_OPTIONS;

namespace KIGFX {

class PCB_VIEW : public VIEW
{
public:
    PCB_VIEW();
    virtual ~PCB_VIEW();

    /// @copydoc VIEW::Add()
    virtual void Add( VIEW_ITEM* aItem, int aDrawPriority = -1 ) override;

    /// @copydoc VIEW::Remove()
    virtual void Remove( VIEW_ITEM* aItem ) override;

    /// @copydoc VIEW::Update()
    virtual void Update( const VIEW_ITEM* aItem, int aUpdateFlags ) const override;

    virtual void Update( const VIEW_ITEM* aItem ) const override;

    /**
     * Sets the KIGFX::REPAINT on all items matching \a aTypes which intersect \a aStaleAreas.
     */
    void UpdateCollidingItems( const std::vector<BOX2I>& aStaleAreas,
                               std::initializer_list<KICAD_T> aTypes );

    void UpdateDisplayOptions( const PCB_DISPLAY_OPTIONS& aOptions );

    /// Set the board whose items are shown, which enables the LOD cache.
    void SetBoard( const BOARD* aBoard ) { m_board = aBoard; }

protected:
    bool lodCacheable() const override { return m_board != nullptr; }
    bool lodContextChanged() override;
    void clearLODContext() override { m_board = nullptr; }

private:
    /// The render settings and board state the items' ViewGetLOD() reads.
    struct LOD_CONTEXT
    {
        const void*   settings = nullptr;
        bool          highContrast = false;
        std::set<int> highContrastLayers;
        int           contrastModeDisplay = 0;
        bool          printing = false;
        bool          forceShowFields = false;
        LSET          visibleLayers;
        LSET          enabledLayers;

        bool operator==( const LOD_CONTEXT& aOther ) const = default;
    };

    const BOARD* m_board = nullptr;
    LOD_CONTEXT  m_lodContext;
};

}
