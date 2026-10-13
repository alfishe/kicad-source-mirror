/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
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
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef LEAD_CLIP_H
#define LEAD_CLIP_H

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <plugins/3dapi/xv3d_types.h>
#include <glm/glm.hpp>
#include <wx/string.h>

class BOARD_ADAPTER;
class FOOTPRINT;
struct S3DMODEL;

/**
 * How a part mounts, from the `mount` section of the model's sidecar `<model>.meta.json`
 * (`X.step` -> `X.meta.json`).
 */
struct MODEL_MOUNT_META
{
    bool                                     found = false;
    std::string                              seat;         ///< shoulder / free / body / socket
    std::optional<std::pair<double, double>> a1_mm;        ///< body gap over the seating plane
    std::optional<double>                    seat_z_mm;    ///< seating plane in model coordinates
    std::string                              socket_ref;   ///< socket model the part sits in
    bool                                     below_board = false;  ///< body goes below the board by design
    std::string                              formed;       ///< "lying" for formed leads mounted lying
};

/**
 * Read the mount metadata beside a resolved model file; cached by path and modification time.
 */
MODEL_MOUNT_META ReadModelMountMeta( const wxString& aModelPath );

/**
 * Distinct vertices of a model (model units, mm) on a 0.05 mm grid: enough to tell leads from
 * bodies under the board.
 */
std::vector<SFVEC3F> ModelSamplePoints( const S3DMODEL& aModel );

/**
 * A plane cutting the leads of a through-hole part under the board (3D units): the part keeps
 * z >= m_z (top side) or z <= m_z (bottom side).
 */
struct LEAD_CLIP
{
    float m_z = 0.0f;
    bool  m_keepAbove = true;

    bool Below( const SFVEC3F& aPoint ) const { return m_keepAbove ? aPoint.z < m_z : aPoint.z > m_z; }
};

/**
 * Why a model's leads are cut or left.
 */
enum class LEAD_CLIP_STATUS
{
    CLIPPED,          ///< leads past the maximum protrusion: cut
    WITHIN_LIMITS,    ///< nothing past the maximum protrusion
    BELOW_BY_DESIGN,  ///< `below_board` in the model's metadata
    BODY_BELOW,       ///< something other than leads hangs below the board: left whole
    PEGS_BELOW,       ///< pegs / board locks (unplated or large holes) past the cut: left whole
    NO_HOLES          ///< not a through-hole footprint
};

struct LEAD_CLIP_RESULT
{
    LEAD_CLIP_STATUS         m_status = LEAD_CLIP_STATUS::NO_HOLES;
    std::optional<LEAD_CLIP> m_clip;
    double                   m_depthMm = 0.0;      ///< deepest lead past the board surface
    double                   m_pegDepthMm = 0.0;   ///< deepest peg / board lock past the board surface
    double                   m_targetMm = 0.0;     ///< protrusion the leads are cut to
};

/**
 * The lead clip of one model of a footprint.
 *
 * Leads longer than the maximum protrusion (IPC-A-610 Class 2: 2.5 mm) are cut 1.0 mm past the
 * board; a footprint field `protrusion_mm` sets both. Leads are what passes through plated holes up
 * to 2 mm; pegs and board locks (unplated or larger holes) are not soldered leads and are never
 * cut: a model whose pegs reach past the cut is left whole, as is one with anything else below the
 * board (or `below_board` in its metadata).
 *
 * @param aModelWorld model to world (3D units) matrix.
 * @param aPoints the model's ModelSamplePoints().
 */
LEAD_CLIP_RESULT ComputeLeadClip( const BOARD_ADAPTER& aAdapter, const FOOTPRINT& aFootprint,
                                  const glm::mat4& aModelWorld, const std::vector<SFVEC3F>& aPoints,
                                  const MODEL_MOUNT_META& aMeta );

#endif // LEAD_CLIP_H
