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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <boost/test/data/test_case.hpp>

#include <map>
#include <set>

#include <board.h>
#include <kiid.h>
#include <footprint.h>
#include <pcb_field.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr_parser.h>
#include <richio.h>
#include <pcbnew_utils/board_file_utils.h>
#include <pcbnew_utils/board_test_utils.h>
#include <settings/settings_manager.h>


namespace
{

struct FOOTPRINT_LOAD_TEST_CASE
{
    // Which footprint to look at in the file
    KIID     m_footprintUuid;
    // Expected values
    bool     m_expectedLocked;
    VECTOR2I m_expectedPos;
};


struct FOOTPRINT_LOAD_BOARD_TEST_CASE: public KI_TEST::BOARD_LOAD_TEST_CASE
{
    // List of images to check
    std::vector<FOOTPRINT_LOAD_TEST_CASE> m_fpCases;
};


const std::vector<FOOTPRINT_LOAD_BOARD_TEST_CASE> FootprintLoadSave_testCases{
    {
            "footprints_load_save",
            std::nullopt,
            {
                    // From top to bottom in the board file
                    {
                            "898cf321-03c7-40bb-8d78-4bc5e52986c2",
                            false,
                            { 100, 80 },
                    },
                    {
                            "0775cd70-2e84-4592-a160-456c37a8f4f6",
                            true,
                            { 100, 100 },
                    },
            },
    },
};

} // namespace

/**
 * Simple tests cases that load a board file and check expected properties of the footprints.
 * This is not the same as FpLibLoadSave as it loads _board files_, and not footprint
 * files from a library.
 */
BOOST_DATA_TEST_CASE( FootprintLoadSave,
                      boost::unit_test::data::make( FootprintLoadSave_testCases ), testCase )
{
    const auto doBoardTest = [&]( const BOARD& aBoard )
    {
        for( const FOOTPRINT_LOAD_TEST_CASE& fooprintTestCase : testCase.m_fpCases )
        {
            BOOST_TEST_MESSAGE( "Checking for footprint with UUID: "
                                << fooprintTestCase.m_footprintUuid.AsString() );

            const auto& fp = static_cast<FOOTPRINT&>( KI_TEST::RequireBoardItemWithTypeAndId(
                    aBoard, PCB_FOOTPRINT_T, fooprintTestCase.m_footprintUuid ) );

            BOOST_CHECK_EQUAL( fp.IsLocked(), fooprintTestCase.m_expectedLocked );
            BOOST_CHECK_EQUAL( fp.GetPosition(), fooprintTestCase.m_expectedPos * 1000000 );
        }
    };

    KI_TEST::LoadAndTestBoardFile( testCase.m_BoardFileRelativePath, true, doBoardTest,
                                   testCase.m_ExpectedBoardVersion );
}

// Footprint fields have the same ids on every load: the file's where it has them, derived ones otherwise
BOOST_AUTO_TEST_CASE( FieldsWithoutIdLoadWithStableIds )
{
    const std::string data = R"(
(kicad_pcb (version 20241229) (generator "test") (generator_version "9.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (35 "F.Fab" user) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (footprint "lib:fp" (layer "F.Cu")
    (uuid "11111111-2222-4333-8444-555555555555")
    (at 10 10)
    (property "Reference" "R1" (at 0 -1 0) (layer "F.Fab")
      (uuid "aaaaaaaa-2222-4333-8444-555555555555")
      (effects (font (size 1 1) (thickness 0.15))))
    (property "Value" "10k" (at 0 1 0) (layer "F.Fab")
      (effects (font (size 1 1) (thickness 0.15))))
    (property "MPN" "X" (at 0 2 0) (layer "F.Fab")
      (effects (font (size 1 1) (thickness 0.15))))
    (fp_text user "note" (at 0 3 0) (layer "F.Fab") (hide yes)
      (uuid "bbbbbbbb-2222-4333-8444-555555555555")
      (effects (font (size 1 1) (thickness 0.15))))
  )
)
)";

    auto fieldIds = [&]()
    {
        STRING_LINE_READER        reader( data, wxT( "stable field ids" ) );
        PCB_IO_KICAD_SEXPR_PARSER parser( &reader, nullptr, nullptr );
        std::unique_ptr<BOARD>    board( dynamic_cast<BOARD*>( parser.Parse() ) );

        BOOST_REQUIRE( board && board->Footprints().size() == 1 );

        std::map<wxString, KIID> ids;

        for( PCB_FIELD* field : board->Footprints().front()->GetFields() )
            ids[field->GetName( false )] = field->m_Uuid;

        return ids;
    };

    const std::map<wxString, KIID> first = fieldIds();
    const std::map<wxString, KIID> second = fieldIds();

    BOOST_REQUIRE( first.size() >= 5 );
    BOOST_CHECK( first == second );
    BOOST_CHECK( first.at( wxS( "Reference" ) ) == KIID( "aaaaaaaa-2222-4333-8444-555555555555" ) );

    // a hidden footprint text becomes a field and keeps the id it has in the file
    bool hiddenTextKeepsId = false;

    for( const auto& [name, id] : first )
        hiddenTextKeepsId |= id == KIID( "bbbbbbbb-2222-4333-8444-555555555555" );

    BOOST_CHECK( hiddenTextKeepsId );

    std::set<KIID> distinct;

    for( const auto& [name, id] : first )
        distinct.insert( id );

    BOOST_CHECK_EQUAL( distinct.size(), first.size() );
}
