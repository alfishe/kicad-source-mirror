// This program source code file is part of KiCad, a free EDA CAD application.
//
// Copyright The KiCad Developers, see AUTHORS.txt for contributors.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/// @file color_palette.h
/// @brief Colours of cached groups kept out of their vertices.  A cached group whose colour is a
/// function of a painter key (layer, net, item class) carries a palette slot in its vertices
/// (VERTEX::shader[0] = mode + PALETTE_SLOT_STRIDE * slot); the vertex shader reads the colour of a
/// slot from a small texture, so recolouring all groups of a key is one texel.

#pragma once

#include <kicad_gl/kiglad.h>    // Must be included first

#include <gal/gal.h>

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace KIGFX
{

class COLOR4D;

/// shader[0] of a vertex: its shader mode plus this times its palette slot (exact in a float)
static constexpr float PALETTE_SLOT_STRIDE = 16.0f;

/// slots in a palette texture row
static constexpr int PALETTE_ROW = 256;

/// @return the shader mode of a vertex from its shader[0] (as kicad_vert.glsl decodes it)
inline uint8_t PaletteVertexMode( GLfloat aShader0 )
{
    const float slot = std::floor( ( aShader0 + 0.5f ) / PALETTE_SLOT_STRIDE );
    return static_cast<uint8_t>( std::floor( aShader0 - PALETTE_SLOT_STRIDE * slot + 0.5f ) );
}


class GAL_API COLOR_PALETTE
{
public:
    /// the largest slot number (slot 0: the vertex colour is used)
    static constexpr int MAX_SLOT = 65534;

    COLOR_PALETTE();
    ~COLOR_PALETTE();

    /**
     * @return the slot of aKey drawn in colour aRgba: the one taken before while it still has
     * that colour, else a new one with that colour; 0 when it changed colour or all are in use
     */
    int Acquire( uint64_t aKey, const GLubyte* aRgba );

    /// @brief A group now uses aSlot
    void AddRef( int aSlot ) { m_slots[aSlot].refs++; }

    /// @brief A group no longer uses aSlot: freed with its last group (or at once when no group
    /// took it)
    void Release( int aSlot );

    /// @return the RGBA bytes of aSlot, as VERTEX_MANAGER writes them into a vertex
    const GLubyte* Bytes( int aSlot ) const { return &m_bytes[size_t( aSlot ) * 4]; }

    void SetBytes( int aSlot, const GLubyte* aRgba );

    void SetColor( int aSlot, const COLOR4D& aColor );

    int Refs( int aSlot ) const { return m_slots[aSlot].refs; }

    /// @return one past the largest slot ever used
    int End() const { return int( m_slots.size() ); }

    /// @brief Every slot freed (the cache was cleared)
    void Clear();

    /// @brief Uploads the colours when changed and binds the texture to aUnit; must be called
    /// with the GL context current
    void Bind( int aUnit );

    /// @return true when colours changed since the last Bind()
    bool IsDirty() const { return m_dirty; }

    /// @return width and height of the texture in texels
    int Width() const { return PALETTE_ROW; }
    int Height() const { return m_texRows; }

    /// @brief Frees the texture (the GL context is current)
    void ReleaseTexture();

    /// @brief Forgets the texture without deleting it (its context is gone)
    void Abandon() { m_texture = 0; m_texRows = 0; m_dirty = true; }

private:
    /// a painter key and the colour its group was drawn in
    struct KEY
    {
        uint64_t key = 0;
        uint32_t rgba = 0;

        bool operator==( const KEY& aOther ) const { return key == aOther.key && rgba == aOther.rgba; }
    };

    struct KEY_HASH
    {
        size_t operator()( const KEY& aKey ) const
        {
            return std::hash<uint64_t>()( aKey.key ^ ( uint64_t( aKey.rgba ) * 0x9E3779B97F4A7C15ull ) );
        }
    };

    struct SLOT
    {
        KEY key;
        int refs = 0;
    };

    std::vector<SLOT>                     m_slots;   ///< by slot number; [0] unused
    std::vector<GLubyte>                  m_bytes;   ///< RGBA per slot
    std::vector<int>                      m_free;
    std::unordered_map<KEY, int, KEY_HASH> m_byKey;
    GLuint                            m_texture = 0;
    int                               m_texRows = 0;
    bool                              m_dirty = true;
};

} // namespace KIGFX
