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

#include "lead_clip.h"

#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <unordered_set>

#include <json_common.h>
#include <wx/filefn.h>
#include <wx/filename.h>

#include <3d_canvas/board_adapter.h>
#include <base_units.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_field.h>
#include <plugins/3dapi/c3dmodel.h>


MODEL_MOUNT_META ReadModelMountMeta( const wxString& aModelPath )
{
    static std::mutex                                                  s_mutex;
    static std::map<wxString, std::pair<time_t, MODEL_MOUNT_META>> s_cache;

    wxFileName fn( aModelPath );
    fn.SetExt( wxS( "meta.json" ) );

    const wxString path = fn.GetFullPath();
    const time_t   mtime = fn.FileExists() ? wxFileModificationTime( path ) : 0;

    std::lock_guard<std::mutex> lock( s_mutex );

    if( auto it = s_cache.find( path ); it != s_cache.end() && it->second.first == mtime )
        return it->second.second;

    MODEL_MOUNT_META meta;

    if( mtime )
    {
        try
        {
            std::ifstream  in( path.fn_str() );
            nlohmann::json j = nlohmann::json::parse( in );

            if( j.contains( "mount" ) && j["mount"].is_object() )
            {
                const nlohmann::json& m = j["mount"];
                meta.found = true;
                meta.seat = m.value( "seat", "" );
                meta.socket_ref = m.value( "socket_ref", "" );
                meta.below_board = m.value( "below_board", false );
                meta.formed = m.value( "formed", "" );

                if( m.contains( "a1_mm" ) && m["a1_mm"].is_array() && m["a1_mm"].size() == 2 )
                    meta.a1_mm = std::make_pair( m["a1_mm"][0].get<double>(), m["a1_mm"][1].get<double>() );

                if( m.contains( "seat_z_mm" ) && m["seat_z_mm"].is_number() )
                    meta.seat_z_mm = m["seat_z_mm"].get<double>();
            }
        }
        catch( const std::exception& )
        {
            meta = MODEL_MOUNT_META();
        }
    }

    s_cache[path] = { mtime, meta };
    return meta;
}


std::vector<SFVEC3F> ModelSamplePoints( const S3DMODEL& aModel )
{
    const float                 grid = 0.05f;
    std::unordered_set<int64_t> seen;
    std::vector<SFVEC3F>        points;

    for( unsigned int m = 0; m < aModel.m_MeshesSize; ++m )
    {
        const SMESH& mesh = aModel.m_Meshes[m];

        for( unsigned int v = 0; v < mesh.m_VertexSize && mesh.m_Positions; ++v )
        {
            const SFVEC3F& p = mesh.m_Positions[v];
            const int64_t  key = ( (int64_t) std::lround( p.x / grid ) & 0x1FFFFF )
                                | ( ( (int64_t) std::lround( p.y / grid ) & 0x1FFFFF ) << 21 )
                                | ( ( (int64_t) std::lround( p.z / grid ) & 0x1FFFFF ) << 42 );

            if( seen.insert( key ).second )
                points.push_back( p );
        }
    }

    return points;
}


LEAD_CLIP_RESULT ComputeLeadClip( const BOARD_ADAPTER& aAdapter, const FOOTPRINT& aFootprint,
                                  const glm::mat4& aModelWorld, const std::vector<SFVEC3F>& aPoints,
                                  const MODEL_MOUNT_META& aMeta )
{
    LEAD_CLIP_RESULT result;

    const double biu = aAdapter.BiuTo3dUnits();
    const double mm = biu * pcbIUScale.IU_PER_MM;

    struct HOLE
    {
        float x, y, r;
        bool  lead;
    };

    std::vector<HOLE> holes;

    for( const PAD* pad : aFootprint.Pads() )
    {
        if( !pad->HasHole() )
            continue;

        const VECTOR2I pos = pad->GetPosition();
        const VECTOR2I drill = pad->GetDrillSize();
        const float    r = (float) ( std::max( drill.x, drill.y ) / 2 * biu );

        const bool lead = pad->GetAttribute() == PAD_ATTRIB::PTH && std::max( drill.x, drill.y ) <= pcbIUScale.mmToIU( 2.0 );

        holes.push_back( { (float) ( pos.x * biu ), (float) ( -pos.y * biu ), r, lead } );
    }

    if( holes.empty() || aPoints.empty() )
        return result;

    double maxProtrusion = 2.5;
    double target = 1.0;

    if( const PCB_FIELD* field = aFootprint.GetField( wxS( "protrusion_mm" ) ) )
    {
        double value = 0.0;

        if( field->GetText().ToCDouble( &value ) && value >= 0.0 )
            maxProtrusion = target = value;
    }

    result.m_targetMm = target;

    const bool  top = !aFootprint.IsFlipped();
    const float surface = top ? aAdapter.GetLayerBottomZPos( B_Cu ) : aAdapter.GetLayerTopZPos( F_Cu );

    LEAD_CLIP clip;
    clip.m_keepAbove = top;
    clip.m_z = (float) ( top ? surface - target * mm : surface + target * mm );

    bool bodyBelow = false;

    for( const SFVEC3F& p : aPoints )
    {
        const SFVEC3F w( aModelWorld * glm::vec4( p, 1.0f ) );
        const double  depth = ( top ? surface - w.z : w.z - surface ) / mm;

        if( depth <= 0.0 )
            continue;

        const HOLE* hole = nullptr;

        for( const HOLE& h : holes )
        {
            const float margin = std::max( (float) ( 0.15 * mm ), 0.5f * h.r );
            const float dx = w.x - h.x;
            const float dy = w.y - h.y;

            if( dx * dx + dy * dy <= ( h.r + margin ) * ( h.r + margin ) )
            {
                hole = &h;
                break;
            }
        }

        if( hole && hole->lead )
            result.m_depthMm = std::max( result.m_depthMm, depth );
        else if( hole )
            result.m_pegDepthMm = std::max( result.m_pegDepthMm, depth );
        else if( clip.Below( w ) )
            bodyBelow = true;
    }

    if( result.m_depthMm <= maxProtrusion + 0.05 )
        result.m_status = LEAD_CLIP_STATUS::WITHIN_LIMITS;
    else if( aMeta.below_board )
        result.m_status = LEAD_CLIP_STATUS::BELOW_BY_DESIGN;
    else if( bodyBelow )
        result.m_status = LEAD_CLIP_STATUS::BODY_BELOW;
    else if( result.m_pegDepthMm > target )
        result.m_status = LEAD_CLIP_STATUS::PEGS_BELOW;
    else
    {
        result.m_status = LEAD_CLIP_STATUS::CLIPPED;
        result.m_clip = clip;
    }

    return result;
}
