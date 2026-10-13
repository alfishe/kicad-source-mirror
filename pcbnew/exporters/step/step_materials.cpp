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

#include "step_materials.h"

#include <algorithm>
#include <cctype>
#include <vector>


namespace
{

struct BASE
{
    const char* word;
    bool        metal;
    double      roughness;
    double      alpha;
};

// Values follow common component material libraries (plating, mold compounds, ceramics)
const BASE BASES[] = {
    { "gold", true, 0.25, 1.0 },       { "silver", true, 0.2, 1.0 },
    { "copper", true, 0.3, 1.0 },      { "nickel", true, 0.35, 1.0 },
    { "tin", true, 0.35, 1.0 },        { "solder", true, 0.35, 1.0 },
    { "bronze", true, 0.35, 1.0 },     { "brass", true, 0.35, 1.0 },
    { "steel", true, 0.25, 1.0 },      { "stainless", true, 0.25, 1.0 },
    { "aluminium", true, 0.3, 1.0 },   { "aluminum", true, 0.3, 1.0 },
    { "zinc", true, 0.3, 1.0 },        { "chrome", true, 0.1, 1.0 },
    { "metal", true, 0.3, 1.0 },

    { "epoxy", false, 0.58, 1.0 },     { "ceramic", false, 0.72, 1.0 },
    { "mlcc", false, 0.72, 1.0 },      { "alumina", false, 0.7, 1.0 },
    { "ferrite", false, 0.8, 1.0 },    { "thickfilm", false, 0.4, 1.0 },
    { "ink", false, 0.6, 1.0 },        { "print", false, 0.6, 1.0 },
    { "laser", false, 0.85, 1.0 },     { "mark", false, 0.85, 1.0 },
    { "pbt", false, 0.5, 1.0 },        { "nylon", false, 0.55, 1.0 },
    { "lcp", false, 0.45, 1.0 },       { "pa", false, 0.5, 1.0 },
    { "pc99", false, 0.45, 1.0 },      { "usb3", false, 0.45, 1.0 },
    { "plastic", false, 0.5, 1.0 },    { "rubber", false, 0.8, 1.0 },
    { "sleeve", false, 0.35, 1.0 },    { "lacquer", false, 0.4, 1.0 },
    { "film", false, 0.4, 1.0 },       { "disc", false, 0.35, 1.0 },
    { "tantalum", false, 0.55, 1.0 },  { "iec60062", false, 0.45, 1.0 },
    { "soldermask", false, 0.35, 1.0 },
    { "glass", false, 0.05, 0.85 },    { "lens", false, 0.1, 0.6 },
};


std::vector<std::string> words( const std::string& aName )
{
    std::vector<std::string> out;
    std::string           w;

    for( char c : aName + ' ' )
    {
        if( std::isalnum( static_cast<unsigned char>( c ) ) )
        {
            w += static_cast<char>( std::tolower( static_cast<unsigned char>( c ) ) );
        }
        else if( !w.empty() )
        {
            out.push_back( w );
            w.clear();
        }
    }

    return out;
}

} // namespace


std::optional<STEP_PBR> PbrFromColourName( const std::string& aName )
{
    const std::vector<std::string> w = words( aName );
    auto has = [&]( const char* aWord ) { return std::find( w.begin(), w.end(), aWord ) != w.end(); };

    // the first material word names the material ("copper_enamel", "laser_mark_epoxy", "pc99_gold"),
    // the rest is finish or colour
    const BASE* base = nullptr;

    for( auto word = w.begin(); word != w.end() && !base; ++word )
    {
        for( const BASE& b : BASES )
        {
            if( *word == b.word )
            {
                base = &b;
                break;
            }
        }
    }

    if( has( "silicon" ) )
        return STEP_PBR{ 0.3, 0.2, 1.0 };

    if( !base )
        return std::nullopt;

    STEP_PBR pbr{ base->metal ? 1.0 : 0.0, base->roughness, base->alpha };

    if( base->metal && ( has( "anodized" ) || has( "anodised" ) ) )
        pbr = STEP_PBR{ 0.0, 0.45, 1.0 };

    // colour-band paint of a resistor: metallic look, not a plating
    if( has( "iec60062" ) && ( has( "gold" ) || has( "silver" ) ) )
        pbr = STEP_PBR{ 0.6, 0.35, 1.0 };

    // tin over nickel: a matte two-layer plating
    if( has( "nickel" ) && has( "tin" ) )
        pbr.roughness = 0.5;

    if( has( "print" ) )
        pbr.roughness = 0.6;

    if( has( "bright" ) || has( "polished" ) )
        pbr.roughness = 0.2;
    else if( has( "glossy" ) || has( "gloss" ) )
        pbr.roughness = 0.22;
    else if( has( "dimple" ) )
        pbr.roughness = 0.35;
    else if( has( "brushed" ) )
        pbr.roughness = 0.32;
    else if( has( "matte" ) || has( "matt" ) )
        pbr.roughness = pbr.metallic > 0.0 ? 0.5 : 0.6;
    else if( has( "dull" ) )
        pbr.roughness = 0.8;

    if( has( "clear" ) )
    {
        pbr.roughness = std::min( pbr.roughness, 0.08 );
        pbr.alpha = 0.35;
    }
    else if( has( "amber" ) )
    {
        pbr.alpha = 0.45;
    }

    if( has( "band" ) )
    {
        pbr.roughness = 0.4;
        pbr.alpha = 1.0;
    }

    return pbr;
}


std::optional<STEP_PBR> PbrFromColour( double aRed, double aGreen, double aBlue )
{
    const double hi = std::max( { aRed, aGreen, aBlue } );
    const double lo = std::min( { aRed, aGreen, aBlue } );

    if( hi >= 0.45 && hi <= 0.80 && hi - lo <= 0.11 )
        return STEP_PBR{ 1.0, 0.35, 1.0, "tin_by_colour" };

    if( aRed >= 0.4 && aGreen >= 0.4 * aRed && aGreen <= 0.8 * aRed && aBlue <= 0.3 * aRed )
        return STEP_PBR{ 1.0, 0.3, 1.0, "gold_by_colour" };

    return std::nullopt;
}
