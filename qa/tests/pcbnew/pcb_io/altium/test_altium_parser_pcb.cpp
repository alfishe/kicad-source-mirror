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
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

/**
 * @file test_altium_parser_pcb.cpp
 * Binary primitive records of older Altium files (short subrecords), built in memory.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <pcbnew/pcb_io/altium/altium_parser_pcb.h>
#include <common/io/altium/altium_binary_parser.h>
#include <ki_exception.h>

#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>


namespace
{

/// Little-endian record writer
struct RECORD_WRITER
{
    std::vector<char> data;

    void u8( uint8_t aValue ) { data.push_back( static_cast<char>( aValue ) ); }

    void u32( uint32_t aValue )
    {
        for( int i = 0; i < 4; i++ )
            u8( ( aValue >> ( 8 * i ) ) & 0xFF );
    }

    void f64( double aValue )
    {
        char bytes[8];
        std::memcpy( bytes, &aValue, sizeof( bytes ) );
        data.insert( data.end(), bytes, bytes + sizeof( bytes ) );
    }

    /// A subrecord: u32 length, then aBody padded with zeros to aLength
    void subrecord( const std::vector<char>& aBody, size_t aLength )
    {
        u32( aLength );
        std::vector<char> body( aBody );
        body.resize( aLength, 0 );
        data.insert( data.end(), body.begin(), body.end() );
    }

    void pascal( const std::string& aText )
    {
        u8( aText.size() );
        data.insert( data.end(), aText.begin(), aText.end() );
    }
};


std::unique_ptr<ALTIUM_BINARY_PARSER> makeParser( const std::vector<char>& aData,
                                                  std::unique_ptr<char[]>& aContent )
{
    aContent = std::make_unique<char[]>( aData.size() );
    std::memcpy( aContent.get(), aData.data(), aData.size() );
    return std::make_unique<ALTIUM_BINARY_PARSER>( aContent, aData.size() );
}


/// Text record: subrecord 1 of aLength bytes (common fields up to offset 40), subrecord 2 the text
std::vector<char> textRecord( size_t aLength, const std::string& aText, bool aWithFontFields )
{
    RECORD_WRITER sub;
    sub.u8( 33 );                // top overlay
    sub.data.resize( 13, 0 );
    sub.u32( 100000 );           // x: 10 mil
    sub.u32( 200000 );           // y: 20 mil
    sub.u32( 600000 );           // height: 60 mil
    sub.data.push_back( 1 );     // stroke font type
    sub.data.push_back( 0 );
    sub.f64( 90.0 );             // rotation
    sub.u8( 0 );                 // mirrored
    sub.u32( 100000 );           // stroke width: 10 mil

    if( aWithFontFields )
    {
        sub.u8( 0 );             // comment
        sub.u8( 1 );             // designator
        sub.u8( 0 );
        sub.u8( 0 );             // stroke font
        sub.u8( 1 );             // bold
        sub.u8( 0 );             // italic
    }

    RECORD_WRITER rec;
    rec.u8( static_cast<uint8_t>( ALTIUM_RECORD::TEXT ) );
    rec.subrecord( sub.data, aLength );

    RECORD_WRITER text;
    text.pascal( aText );
    rec.subrecord( text.data, text.data.size() );

    // Next record's first byte: the parser must stop exactly before it
    rec.u8( static_cast<uint8_t>( ALTIUM_RECORD::TEXT ) );
    return rec.data;
}


/// Pad record with subrecord 5 of aLength bytes
std::vector<char> padRecord( size_t aLength )
{
    RECORD_WRITER rec;
    rec.u8( static_cast<uint8_t>( ALTIUM_RECORD::PAD ) );

    RECORD_WRITER name;
    name.pascal( "7" );
    rec.subrecord( name.data, name.data.size() );

    for( int i = 0; i < 3; i++ )
        rec.subrecord( {}, 0 );

    RECORD_WRITER sub;
    sub.u8( 1 );                 // top layer
    sub.data.resize( 13, 0 );
    sub.u32( 500000 );           // x: 50 mil
    sub.u32( 0 );
    sub.u32( 600000 );           // top size: 60 x 40 mil
    sub.u32( 400000 );
    sub.u32( 600000 );
    sub.u32( 400000 );
    sub.u32( 600000 );
    sub.u32( 400000 );
    sub.u32( 0 );                // hole
    sub.u8( 2 );                 // rectangles
    sub.u8( 2 );
    sub.u8( 2 );
    sub.f64( 45.0 );             // rotation
    sub.u8( 0 );                 // not plated
    sub.data.resize( 101, 0 );
    sub.u8( 2 );                 // paste expansion: manual
    sub.u8( 1 );                 // mask expansion: rule

    rec.subrecord( sub.data, aLength );
    rec.subrecord( {}, 0 );      // subrecord 6
    return rec.data;
}

} // namespace


BOOST_AUTO_TEST_SUITE( AltiumParserPcb )


BOOST_AUTO_TEST_CASE( Text43BytesReadsTextAndStops )
{
    std::unique_ptr<char[]> content;
    std::vector<char>       data = textRecord( 43, "R1", false );
    auto                    parser = makeParser( data, content );
    std::map<uint32_t, wxString> strings = { { 0, wxS( "wrong" ) } };

    ATEXT6 text( *parser, strings );

    BOOST_CHECK_EQUAL( text.text, wxS( "R1" ) );
    BOOST_CHECK( text.fonttype == ALTIUM_TEXT_TYPE::STROKE );
    BOOST_CHECK_EQUAL( text.rotation, 90.0 );
    BOOST_CHECK_EQUAL( parser->GetRemainingBytes(), 1u );
}


BOOST_AUTO_TEST_CASE( Text123BytesReadsFontFields )
{
    std::unique_ptr<char[]> content;
    std::vector<char>       data = textRecord( 123, ".Designator", true );
    auto                    parser = makeParser( data, content );
    std::map<uint32_t, wxString> strings;

    ATEXT6 text( *parser, strings );

    BOOST_CHECK_EQUAL( text.text, wxS( ".Designator" ) );
    BOOST_CHECK( text.isDesignator );
    BOOST_CHECK( !text.isComment );
    BOOST_CHECK( text.isBold );
    BOOST_CHECK( text.fonttype == ALTIUM_TEXT_TYPE::STROKE );
    BOOST_CHECK( !text.isFrame );
    BOOST_CHECK_EQUAL( text.height, 1524000u );
    BOOST_CHECK_EQUAL( parser->GetRemainingBytes(), 1u );
}


BOOST_AUTO_TEST_CASE( PadShortSubrecord5 )
{
    for( size_t length : { 105, 106 } )
    {
        BOOST_TEST_CONTEXT( "subrecord 5 of " << length << " bytes" )
        {
            std::unique_ptr<char[]> content;
            std::vector<char>       data = padRecord( length );
            auto                    parser = makeParser( data, content );

            APAD6 pad( *parser );

            BOOST_CHECK_EQUAL( pad.name, wxS( "7" ) );
            BOOST_CHECK_EQUAL( pad.position.x, 1270000 );
            BOOST_CHECK_EQUAL( pad.topsize.x, 1524000 );
            BOOST_CHECK_EQUAL( pad.topsize.y, 1016000 );
            BOOST_CHECK( pad.topshape == ALTIUM_PAD_SHAPE::RECT );
            BOOST_CHECK_EQUAL( pad.direction, 45.0 );
            BOOST_CHECK( pad.pastemaskexpansionmode == ALTIUM_MODE::MANUAL );
            BOOST_CHECK( pad.soldermaskexpansionmode == ALTIUM_MODE::RULE );
            BOOST_CHECK_EQUAL( pad.holerotation, 0.0 );
            BOOST_CHECK( !pad.sizeAndShape );
            BOOST_CHECK_EQUAL( parser->GetRemainingBytes(), 0u );
        }
    }
}


BOOST_AUTO_TEST_CASE( PadSubrecord5TooShort )
{
    std::unique_ptr<char[]> content;
    std::vector<char>       data = padRecord( 100 );
    auto                    parser = makeParser( data, content );

    BOOST_CHECK_THROW( APAD6 pad( *parser ), IO_ERROR );
}


BOOST_AUTO_TEST_SUITE_END()
