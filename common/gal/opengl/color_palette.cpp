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

#include <gal/opengl/color_palette.h>
#include <gal/opengl/utils.h>
#include <gal/color4d.h>

#include <algorithm>
#include <cstring>

using namespace KIGFX;


COLOR_PALETTE::COLOR_PALETTE()
{
    Clear();
}


COLOR_PALETTE::~COLOR_PALETTE()
{
    // the texture belongs to the GL context, freed by ReleaseTexture while it is current
}


int COLOR_PALETTE::Acquire( uint64_t aKey, const GLubyte* aRgba )
{
    KEY key{ aKey, 0 };
    std::memcpy( &key.rgba, aRgba, 4 );

    auto it = m_byKey.find( key );

    if( it != m_byKey.end() )
        return std::memcmp( Bytes( it->second ), aRgba, 4 ) == 0 ? it->second : 0;

    int slot;

    if( !m_free.empty() )
    {
        slot = m_free.back();
        m_free.pop_back();
    }
    else if( m_slots.size() <= size_t( MAX_SLOT ) )
    {
        slot = int( m_slots.size() );
        m_slots.emplace_back();
        m_bytes.resize( m_slots.size() * 4, 0 );
    }
    else
    {
        return 0;
    }

    m_slots[slot] = SLOT{ key, 0 };
    m_byKey.emplace( key, slot );
    SetBytes( slot, aRgba );
    return slot;
}


void COLOR_PALETTE::Release( int aSlot )
{
    SLOT& s = m_slots[aSlot];

    if( s.refs > 0 && --s.refs > 0 )
        return;

    m_byKey.erase( s.key );
    s = SLOT();
    m_free.push_back( aSlot );
}


void COLOR_PALETTE::SetBytes( int aSlot, const GLubyte* aRgba )
{
    GLubyte* dst = &m_bytes[size_t( aSlot ) * 4];

    if( std::memcmp( dst, aRgba, 4 ) == 0 )
        return;

    std::memcpy( dst, aRgba, 4 );
    m_dirty = true;
}


void COLOR_PALETTE::SetColor( int aSlot, const COLOR4D& aColor )
{
    // the same conversion as VERTEX_MANAGER::Color, so a slot and a vertex agree to the byte
    const GLubyte rgba[4] = { GLubyte( aColor.r * 255.0 ), GLubyte( aColor.g * 255.0 ),
                              GLubyte( aColor.b * 255.0 ), GLubyte( aColor.a * 255.0 ) };
    SetBytes( aSlot, rgba );
}


void COLOR_PALETTE::Clear()
{
    m_slots.assign( 1, SLOT() );
    m_bytes.assign( 4, 0 );
    m_free.clear();
    m_byKey.clear();
    m_dirty = true;
}


void COLOR_PALETTE::Bind( int aUnit )
{
    glActiveTexture( GL_TEXTURE0 + aUnit );

    if( !m_texture )
    {
        glGenTextures( 1, &m_texture );
        glBindTexture( GL_TEXTURE_2D, m_texture );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
        m_texRows = 0;
        m_dirty = true;
    }
    else
    {
        glBindTexture( GL_TEXTURE_2D, m_texture );
    }

    if( m_dirty )
    {
        const int rows = std::max<int>( 1, ( int( m_slots.size() ) + PALETTE_ROW - 1 ) / PALETTE_ROW );
        std::vector<GLubyte> texels( size_t( rows ) * PALETTE_ROW * 4, 0 );
        std::memcpy( texels.data(), m_bytes.data(), m_bytes.size() );

        glPixelStorei( GL_UNPACK_ALIGNMENT, 4 );

        if( rows != m_texRows )
        {
            glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA8, PALETTE_ROW, rows, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                          texels.data() );
            m_texRows = rows;
        }
        else
        {
            glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, PALETTE_ROW, rows, GL_RGBA, GL_UNSIGNED_BYTE,
                             texels.data() );
        }

        checkGlError( "uploading the colour palette", __FILE__, __LINE__ );
        m_dirty = false;
    }

    glActiveTexture( GL_TEXTURE0 );
}


void COLOR_PALETTE::ReleaseTexture()
{
    if( m_texture && glDeleteTextures )
        glDeleteTextures( 1, &m_texture );

    m_texture = 0;
    m_texRows = 0;
    m_dirty = true;
}
