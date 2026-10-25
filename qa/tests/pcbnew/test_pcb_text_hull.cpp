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

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <board.h>
#include <pcb_text.h>
#include <geometry/shape_poly_set.h>

#include <chrono>


namespace
{

const int MAX_ERROR = pcbIUScale.mmToIU( 0.005 );


/// The polygon's extent in the text's own (unrotated) frame
BOX2I frameBox( const PCB_TEXT& aText, const SHAPE_POLY_SET& aPoly )
{
    SHAPE_POLY_SET poly( aPoly );
    poly.Rotate( -aText.GetDrawRotation(), aText.GetDrawPos() );
    return poly.BBox();
}


/// The hull as the rendered strokes' union gives it: what TransformShapeToPolygon is to match
BOX2I unionHullBox( const PCB_TEXT& aText, int aClearance, ERROR_LOC aErrorLoc )
{
    SHAPE_POLY_SET poly;
    aText.TransformTextToPolySet( poly, 0, MAX_ERROR, aErrorLoc );
    BOX2I box = frameBox( aText, poly );
    box.Inflate( aClearance );
    return box;
}


BOX2I hullBox( const PCB_TEXT& aText, int aClearance, ERROR_LOC aErrorLoc )
{
    SHAPE_POLY_SET poly;
    aText.TransformShapeToPolygon( poly, aText.GetLayer(), aClearance, MAX_ERROR, aErrorLoc );
    BOOST_REQUIRE_EQUAL( poly.OutlineCount(), 1 );
    BOOST_REQUIRE_EQUAL( poly.Outline( 0 ).PointCount(), 4 );
    return frameBox( aText, poly );
}


struct TEXT_HULL_FIXTURE
{
    BOARD    m_board;
    PCB_TEXT m_text;

    TEXT_HULL_FIXTURE() :
            m_text( &m_board )
    {
        m_text.SetLayer( F_SilkS );
        m_text.SetTextSize( VECTOR2I( pcbIUScale.mmToIU( 1.0 ), pcbIUScale.mmToIU( 1.0 ) ) );
        m_text.SetTextThickness( pcbIUScale.mmToIU( 0.15 ) );
        m_text.SetTextPos( VECTOR2I( pcbIUScale.mmToIU( 12.3 ), pcbIUScale.mmToIU( -4.5 ) ) );
    }
};

} // namespace


BOOST_FIXTURE_TEST_SUITE( PcbTextHull, TEXT_HULL_FIXTURE )


// The hull equals the union's extent: exact stroke ends against an outer polygon approximation, so
// the two differ by at most the approximation error per side (two for ERROR_OUTSIDE, which inflates
// the union once more)
BOOST_AUTO_TEST_CASE( MatchesRenderedUnion )
{
    const int clearance = pcbIUScale.mmToIU( 0.2 );

    for( const wxString& text : { wxString( wxT( "U27" ) ), wxString( wxT( "SN74LVC257APW" ) ),
                                  wxString( wxT( "two\nlines" ) ), wxString( wxT( "~{RESET}_{1}" ) ) } )
    {
        for( double degrees : { 0.0, 30.0, 90.0, 217.5 } )
        {
            for( bool knockout : { false, true } )
            {
                for( ERROR_LOC errorLoc : { ERROR_INSIDE, ERROR_OUTSIDE } )
                {
                    m_text.SetText( text );
                    m_text.SetTextAngle( EDA_ANGLE( degrees, DEGREES_T ) );
                    m_text.SetIsKnockout( knockout );

                    BOOST_TEST_CONTEXT( text << " " << degrees << " deg knockout " << knockout << " outside "
                                             << ( errorLoc == ERROR_OUTSIDE ) )
                    {
                        const BOX2I expected = unionHullBox( m_text, clearance, errorLoc );
                        const BOX2I actual = hullBox( m_text, clearance, errorLoc );
                        const int   tolerance = 2 * MAX_ERROR + 4;

                        BOOST_CHECK_LE( std::abs( actual.GetLeft() - expected.GetLeft() ), tolerance );
                        BOOST_CHECK_LE( std::abs( actual.GetTop() - expected.GetTop() ), tolerance );
                        BOOST_CHECK_LE( std::abs( actual.GetRight() - expected.GetRight() ), tolerance );
                        BOOST_CHECK_LE( std::abs( actual.GetBottom() - expected.GetBottom() ), tolerance );
                    }
                }
            }
        }
    }
}


// The locked-item shadow of every footprint text is this hull, rebuilt on each move: it must not
// cost the stroke union (milliseconds per text, minutes for a batch move on a loaded host)
BOOST_AUTO_TEST_CASE( CheaperThanRenderedUnion )
{
    m_text.SetText( wxT( "SN74LVC257APW" ) );
    m_text.SetTextAngle( EDA_ANGLE( 30.0, DEGREES_T ) );

    auto seconds = [&]( auto&& aBuild )
    {
        const auto start = std::chrono::steady_clock::now();

        for( int i = 0; i < 50; ++i )
            aBuild();

        return std::chrono::duration<double>( std::chrono::steady_clock::now() - start ).count();
    };

    const double unionTime = seconds( [&] { unionHullBox( m_text, 0, ERROR_OUTSIDE ); } );
    const double hullTime = seconds( [&] { hullBox( m_text, 0, ERROR_OUTSIDE ); } );

    BOOST_TEST_MESSAGE( "union " << unionTime * 20 << " ms/text, hull " << hullTime * 20 << " ms/text" );
    BOOST_CHECK_LT( hullTime * 5, unionTime );
}


BOOST_AUTO_TEST_SUITE_END()
