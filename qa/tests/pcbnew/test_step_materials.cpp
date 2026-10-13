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

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <exporters/step/step_materials.h>


BOOST_AUTO_TEST_SUITE( StepMaterials )


BOOST_AUTO_TEST_CASE( NamedPlatingsAreMetal )
{
    for( const char* name : { "tin_bright", "tin_matte", "nickel_tin", "gold", "copper", "phosphor_bronze",
                              "stainless_steel", "aluminium_brushed" } )
    {
        std::optional<STEP_PBR> pbr = PbrFromColourName( name );
        BOOST_REQUIRE_MESSAGE( pbr, name );
        BOOST_CHECK_MESSAGE( pbr->metallic == 1.0, name );
    }

    BOOST_CHECK_CLOSE( PbrFromColourName( "tin_bright" )->roughness, 0.2, 1e-6 );
    BOOST_CHECK_CLOSE( PbrFromColourName( "tin_matte" )->roughness, 0.5, 1e-6 );
    BOOST_CHECK_CLOSE( PbrFromColourName( "nickel_tin" )->roughness, 0.5, 1e-6 );
}


BOOST_AUTO_TEST_CASE( FirstMaterialWordWins )
{
    // a laser mark on epoxy is the mark, a gold-coloured plastic is plastic, enamelled copper is copper
    BOOST_CHECK_CLOSE( PbrFromColourName( "laser_mark_epoxy" )->roughness, 0.85, 1e-6 );
    BOOST_CHECK_EQUAL( PbrFromColourName( "pc99_gold" )->metallic, 0.0 );
    BOOST_CHECK_EQUAL( PbrFromColourName( "copper_enamel" )->metallic, 1.0 );
    BOOST_CHECK_EQUAL( PbrFromColourName( "aluminium_black_anodized" )->metallic, 0.0 );
}


BOOST_AUTO_TEST_CASE( DielectricsAndClearParts )
{
    BOOST_CHECK_CLOSE( PbrFromColourName( "epoxy_black_matte" )->roughness, 0.6, 1e-6 );
    BOOST_CHECK_CLOSE( PbrFromColourName( "epoxy_black_glossy" )->roughness, 0.22, 1e-6 );
    BOOST_CHECK_EQUAL( PbrFromColourName( "ceramic_mlcc_tan" )->metallic, 0.0 );
    BOOST_CHECK_LT( PbrFromColourName( "epoxy_clear" )->alpha, 1.0 );
    BOOST_CHECK_LT( PbrFromColourName( "led_lens_green" )->alpha, 1.0 );
    BOOST_CHECK_EQUAL( PbrFromColourName( "glass_melf_band" )->alpha, 1.0 );
    BOOST_CHECK_CLOSE( PbrFromColourName( "silicon_die" )->metallic, 0.3, 1e-6 );
}


BOOST_AUTO_TEST_CASE( UnknownNamesSayNothing )
{
    BOOST_CHECK( !PbrFromColourName( "" ) );
    BOOST_CHECK( !PbrFromColourName( "Body color" ) );
    BOOST_CHECK( !PbrFromColourName( "default_grey" ) );
}


BOOST_AUTO_TEST_CASE( ColourGuessForUnnamedColours )
{
    // KiCad stock pin greys and gold, linear
    BOOST_CHECK_EQUAL( PbrFromColour( 0.64, 0.63, 0.57 )->guess, "tin_by_colour" );
    BOOST_CHECK_EQUAL( PbrFromColour( 0.71, 0.50, 0.21 )->guess, "gold_by_colour" );

    // black body, white plastic, saturated red
    BOOST_CHECK( !PbrFromColour( 0.02, 0.02, 0.02 ) );
    BOOST_CHECK( !PbrFromColour( 0.95, 0.95, 0.95 ) );
    BOOST_CHECK( !PbrFromColour( 0.8, 0.05, 0.05 ) );
}


BOOST_AUTO_TEST_SUITE_END()
