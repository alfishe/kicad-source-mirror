/*
 * This program source code file is part of KICAD, a free EDA CAD application.
 *
 * Copyright (C) 2012 Torsten Hueter, torstenhtr <at> gmx.de
 * Copyright (C) 2013 CERN
 * @author Maciej Suminski <maciej.suminski@cern.ch>
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gal/graphics_abstraction_layer.h>
#include <wx/string.h>
#include <wx/textfile.h>
#include <newstroke_font.h>
#include <font/glyph.h>
#include <font/stroke_font.h>
#include <geometry/shape_line_chain.h>
#include <trigo.h>

// The "official" name of the building Kicad stroke font (always existing)
#include <font/kicad_font_name.h>

#include <mutex>

using namespace KIFONT;


/// Scale factor for a glyph.
static constexpr double STROKE_FONT_SCALE = 1.0 / 21.0;

/// Offset (in stroke font units) to move the origin to the baseline.
static constexpr int FONT_OFFSET = -8;


/// Glyphs are parsed a block at a time, when a text first draws one of the block's glyphs.
static constexpr unsigned GLYPH_BLOCK = 256;

static const char* const*                   g_defaultFontData = nullptr;
static std::vector<std::shared_ptr<GLYPH>>  g_defaultFontGlyphs;
static std::unique_ptr<std::once_flag[]>    g_defaultFontBlocks;
static std::vector<BOX2D>*                  g_defaultFontGlyphBoundingBoxes = nullptr;
static double                               g_defaultFontMaxGlyphWidth = 0.0;
static std::once_flag                       g_defaultFontLoaded;


STROKE_FONT::STROKE_FONT() :
        m_glyphs( nullptr ),
        m_glyphBoundingBoxes( nullptr ),
        m_maxGlyphWidth( 0.0 )
{
}


STROKE_FONT* STROKE_FONT::LoadFont( const wxString& aFontName )
{
    if( aFontName.empty() )
    {
        STROKE_FONT* font = new STROKE_FONT();
        font->loadNewStrokeFont( newstroke_font, newstroke_font_bufsize );
        return font;
    }
    else
    {
        // If we ever supported other stroke fonts, the code would go here.
        return nullptr;
    }
}


/**
 * Parse one glyph of the stroke font: its bounding box, and its strokes when \a aGlyph is given.
 *
 * In stroke font, coordinates values are coded as <value> + 'R', where <value> is an ASCII char.
 * The first two values are the glyph's start and end X; " R" raises the pen.
 * Note:
 *  * the stroke coordinates are stored in reduced form (-1.0 to +1.0), and the actual size is
 *    stroke coordinate * glyph size
 *  * a few shapes have a height slightly bigger than 1.0 ( like '{' '[' )
 *  * FONT_OFFSET is here for historical reasons, due to the way the stroke font was built.
 *    It allows shapes coordinates like W M ... to be >= 0.  Only shapes like j y have
 *    coordinates < 0.
 */
static BOX2D parseGlyph( const char* aData, STROKE_GLYPH* aGlyph )
{
    if( !aData[0] || !aData[1] )
        return BOX2D();

    double glyphStartX = ( aData[0] - 'R' ) * STROKE_FONT_SCALE;
    double glyphWidth = ( aData[1] - 'R' ) * STROKE_FONT_SCALE - glyphStartX;
    double minY = 0.0;
    double maxY = 0.0;

    if( aGlyph )
    {
        int strokes = 0;

        for( int i = 2; aData[i]; i += 2 )
        {
            if( aData[i] == ' ' && aData[i + 1] == 'R' )
                strokes++;
        }

        aGlyph->reserve( strokes + 1 );
    }

    bool penIsDown = false;

    for( int i = 2; aData[i]; i += 2 )
    {
        if( aData[i] == ' ' && aData[i + 1] == 'R' )
        {
            if( aGlyph )
                aGlyph->RaisePen();

            penIsDown = false;
            continue;
        }

        VECTOR2D point( ( aData[i] - 'R' ) * STROKE_FONT_SCALE - glyphStartX,
                        ( aData[i + 1] - 'R' + FONT_OFFSET ) * STROKE_FONT_SCALE );

        minY = std::min( minY, point.y );
        maxY = std::max( maxY, point.y );

        if( !aGlyph )
            continue;

        if( !penIsDown )
        {
            // Most strokes have two or three points: reserve what this one has
            size_t count = 0;

            for( int k = i; aData[k] && !( aData[k] == ' ' && aData[k + 1] == 'R' ); k += 2 )
                count++;

            aGlyph->StartStroke( count );
            penIsDown = true;
        }

        aGlyph->AddPoint( point );
    }

    BOX2D bbox( VECTOR2D( 0.0, minY ), VECTOR2D( glyphWidth, maxY - minY ) );

    if( aGlyph )
    {
        aGlyph->Finalize();
        aGlyph->SetBoundingBox( bbox );
    }

    return bbox;
}


void STROKE_FONT::loadNewStrokeFont( const char* const aNewStrokeFont[], int aNewStrokeFontSize )
{
    // Only the bounding boxes now (texts are measured far more often than drawn, and most
    // of the font's 65,000 glyphs are never drawn): the glyphs themselves on first use.
    std::call_once( g_defaultFontLoaded,
            [&]()
            {
                g_defaultFontData = aNewStrokeFont;
                g_defaultFontGlyphs.resize( aNewStrokeFontSize );
                g_defaultFontBlocks.reset(
                        new std::once_flag[( aNewStrokeFontSize + GLYPH_BLOCK - 1 ) / GLYPH_BLOCK] );

                g_defaultFontGlyphBoundingBoxes = new std::vector<BOX2D>;
                g_defaultFontGlyphBoundingBoxes->reserve( aNewStrokeFontSize );

                for( int j = 0; j < aNewStrokeFontSize; j++ )
                {
                    BOX2D bbox = parseGlyph( aNewStrokeFont[j], nullptr );
                    g_defaultFontGlyphBoundingBoxes->push_back( bbox );
                    g_defaultFontMaxGlyphWidth = std::max( g_defaultFontMaxGlyphWidth, bbox.GetWidth() );
                }
            } );

    m_glyphs = &g_defaultFontGlyphs;
    m_glyphBoundingBoxes = g_defaultFontGlyphBoundingBoxes;
    m_maxGlyphWidth = g_defaultFontMaxGlyphWidth;
    m_fontName = KICAD_FONT_NAME;
    m_fontFileName = wxEmptyString;
}


STROKE_GLYPH* STROKE_FONT::glyph( unsigned aIndex ) const
{
    unsigned block = aIndex / GLYPH_BLOCK;

    std::call_once( g_defaultFontBlocks[block],
            [block]()
            {
                unsigned end = std::min<size_t>( ( block + 1 ) * GLYPH_BLOCK, g_defaultFontGlyphs.size() );

                for( unsigned j = block * GLYPH_BLOCK; j < end; j++ )
                {
                    auto glyph = std::make_shared<STROKE_GLYPH>();
                    parseGlyph( g_defaultFontData[j], glyph.get() );
                    g_defaultFontGlyphs[j] = std::move( glyph );
                }
            } );

    return static_cast<STROKE_GLYPH*>( ( *m_glyphs )[aIndex].get() );
}


double STROKE_FONT::GetInterline( double aGlyphHeight, const METRICS& aFontMetrics ) const
{
    static double LEGACY_FACTOR = 0.9583;   // Adjustment to match legacy spacing

    return aFontMetrics.GetInterline( aGlyphHeight ) * LEGACY_FACTOR;
}


VECTOR2I STROKE_FONT::GetTextAsGlyphs( BOX2I* aBBox, std::vector<std::unique_ptr<GLYPH>>* aGlyphs,
                                       const wxString& aText, const VECTOR2I& aSize,
                                       const VECTOR2I& aPosition, const EDA_ANGLE& aAngle,
                                       bool aMirror, const VECTOR2I& aOrigin,
                                       TEXT_STYLE_FLAGS aTextStyle ) const
{
    constexpr int    TAB_WIDTH = 4;
    constexpr double INTER_CHAR = 0.2;

    VECTOR2I cursor( aPosition );
    VECTOR2D glyphSize( aSize );
    double   tilt = ( aTextStyle & TEXT_STYLE::ITALIC ) ? ITALIC_TILT : 0.0;
    double   space_width = m_glyphBoundingBoxes->front().GetWidth(); // First char is space
    int      char_count = 0;

    if( aTextStyle & TEXT_STYLE::SUBSCRIPT || aTextStyle & TEXT_STYLE::SUPERSCRIPT )
    {
        glyphSize = glyphSize * GetSuperSubSizeMultiplier();
        cursor.y += glyphSize.y * GetSuperSubBaselineOffset( aTextStyle );
    }

    for( wxUniChar c : aText )
    {
        // Handle tabs as locked to the next 4th column (in base-widths).
        if( c == '\t' )
        {
            char_count = ( char_count / TAB_WIDTH + 1 ) * TAB_WIDTH - 1;

            int new_cursor = aPosition.x + aSize.x * char_count
                    + aSize.x * space_width;

            while( new_cursor <= cursor.x )
            {
                char_count += TAB_WIDTH;
                new_cursor += aSize.x * TAB_WIDTH;
            }

            cursor.x = new_cursor;
        }
        else if( c == ' ' )
        {
            // 'space' character - draw nothing, advance cursor position
            cursor.x += KiROUND( glyphSize.x * space_width );
        }
        else
        {
            // dd is the index into bounding boxes table
            int dd = (signed) c - ' ';

            // Filtering non existing glyphs and non printable chars
            if( dd < 0 || dd >= (int) m_glyphBoundingBoxes->size() )
            {
                c = '?';
                dd = (signed) c - ' ';
            }

            if( aGlyphs )
            {
                STROKE_GLYPH* source = glyph( dd );
                aGlyphs->push_back( source->Transform( glyphSize, cursor, tilt, aAngle, aMirror,
                                                       aOrigin ) );
            }

            VECTOR2D glyphExtents = ( *m_glyphBoundingBoxes )[dd].GetEnd();

            glyphExtents *= glyphSize;

            cursor.x += KiROUND( glyphExtents.x );
        }

        ++char_count;
    }

    if( aBBox )
    {
        aBBox->SetOrigin( aPosition );
        aBBox->SetEnd( cursor.x - KiROUND( glyphSize.x * INTER_CHAR ), cursor.y - glyphSize.y );
        aBBox->Normalize();
    }

    return VECTOR2I( cursor.x, aPosition.y );
}


unsigned STROKE_FONT::GetGlyphCount() const
{
    return m_glyphs ? m_glyphs->size() : 0;
}


const STROKE_GLYPH* STROKE_FONT::GetGlyph( unsigned aIndex ) const
{
    if( !m_glyphs || aIndex >= m_glyphs->size() )
        return nullptr;

    return glyph( aIndex );
}


const BOX2D& STROKE_FONT::GetGlyphBoundingBox( unsigned aIndex ) const
{
    static const BOX2D empty;

    if( !m_glyphBoundingBoxes || aIndex >= m_glyphBoundingBoxes->size() )
        return empty;

    return m_glyphBoundingBoxes->at( aIndex );
}
