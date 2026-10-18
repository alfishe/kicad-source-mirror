/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.TXT for contributors.
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

/**
 * @file
 * Test suite for general string functions
 */

#include <qa_utils/wx_utils/unit_test_utils.h>

// Code under test
#include <thread>
#include <vector>
#include <font/glyph.h>
#include <font/stroke_font.h>

/**
 * Declare the test suite
 */
BOOST_AUTO_TEST_SUITE( KicadStrokeFont )


/**
 * Test the tab spacing.
 */
BOOST_AUTO_TEST_CASE( TabCheck )
{
    using CASE = std::pair<wxString, wxString>;

    KIFONT::STROKE_FONT* font = KIFONT::STROKE_FONT::LoadFont( wxEmptyString );

    const std::vector<CASE> cases = {
        { "v34_STM23G491\t\tController STM32\t(column 3)",
          "v34_LPC1758\t\t\tController NXP\t\t(column 3)" },
        { "1\td9c1892a\t\t\tMOLEX\t\t\t1053071202\t\t\t\t\t12\t\tCONNECTOR",
          "REF\tPART NUMBER\t\t\tMANUFACTURER\tMANUFACTURER PART NUMBER\tQTY\t\tCONNECTOR" },
        { "5\tC-000208\t\t\tMCMASTER/CARR\t8054T13 BLUE\t\t\t\t3960MM\tCONNECTOR",
          "REF\tPART NUMBER\t\t\tMANUFACTURER\tMANUFACTURER PART NUMBER\tQTY\t\tCONNECTOR" },
        { "0\t\t\t\tlinvA\t\t\tL",
          "3\t\t\t\tlloadA\t\t\tL" },
        { "6\t\t\t\tVpccA\t\t\tL",
          "14\t\t\t\t--\t\t\t\tL" },
    };

    for( const auto& c : cases )
    {
        std::vector<std::unique_ptr<KIFONT::GLYPH>> glyphs;
        BOX2I bbox;
        wxString text1 = wxString::Format( c.first );
        wxString text2 = wxString::Format( c.second );

            VECTOR2I output1 = font->GetTextAsGlyphs( &bbox, &glyphs, text1,
                    VECTOR2I( 1000, 1000 ), VECTOR2I( 0, 0 ), ANGLE_0, false,
                    VECTOR2I( 0, 0 ), 0 );
            VECTOR2I output2 = font->GetTextAsGlyphs( &bbox, &glyphs, text2,
                    VECTOR2I( 1000, 1000 ), VECTOR2I( 0, 0 ), ANGLE_0, false,
                    VECTOR2I( 0, 0 ), 0 );

        BOOST_CHECK_MESSAGE( output1.x == output2.x, "Incorrect tab size for \n\t'" <<
                             text1.ToStdString() << "' and\n\t'" << text2.ToStdString() << "'" );
    }

    delete font;
}


/**
 * Glyphs parsed on first use, from several threads at once, match the bounding boxes the font
 * measures texts with, and measuring alone does not need them.
 */
BOOST_AUTO_TEST_CASE( GlyphsOnFirstUse )
{
    KIFONT::STROKE_FONT* font = KIFONT::STROKE_FONT::LoadFont( wxEmptyString );
    const unsigned       count = font->GetGlyphCount();

    BOOST_REQUIRE_GT( count, 60000u );

    BOX2I    bbox;
    VECTOR2I end = font->GetTextAsGlyphs( &bbox, nullptr, wxS( "Ab\u4e2d" ), VECTOR2I( 1000, 1000 ),
                                          VECTOR2I( 0, 0 ), ANGLE_0, false, VECTOR2I( 0, 0 ), 0 );
    BOOST_CHECK_GT( end.x, 0 );

    std::vector<int>         mismatches( 8, 0 );
    std::vector<std::thread> threads;

    for( int t = 0; t < 8; t++ )
    {
        threads.emplace_back(
                [&, t]()
                {
                    for( unsigned i = t; i < count; i += 8 )
                    {
                        const KIFONT::STROKE_GLYPH* glyph = font->GetGlyph( i );

                        if( !glyph
                            || const_cast<KIFONT::STROKE_GLYPH*>( glyph )->BoundingBox()
                                       != font->GetGlyphBoundingBox( i ) )
                        {
                            mismatches[t]++;
                        }
                    }
                } );
    }

    for( std::thread& thread : threads )
        thread.join();

    for( int t = 0; t < 8; t++ )
        BOOST_CHECK_EQUAL( mismatches[t], 0 );

    const KIFONT::STROKE_GLYPH* a = font->GetGlyph( 'A' - ' ' );
    BOOST_REQUIRE( a );
    BOOST_CHECK_EQUAL( a->size(), 2u );    // the slants in one stroke, the bar
    BOOST_CHECK_EQUAL( font->GetGlyph( count ), nullptr );

    delete font;
}


BOOST_AUTO_TEST_SUITE_END()
