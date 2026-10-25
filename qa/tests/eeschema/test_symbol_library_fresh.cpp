/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * SCH_IO_KICAD_SEXPR::LoadSymbol() serves the current library file: a rewrite of the same
 * size within the same second is read again on the next load.
 */

#include <boost/test/unit_test.hpp>
#include <qa_utils/wx_utils/unit_test_utils.h>

#include <lib_symbol.h>
#include <sch_field.h>
#include <sch_pin.h>
#include <sch_io/kicad_sexpr/sch_io_kicad_sexpr.h>

#include <wx/filename.h>

#include <fstream>


namespace
{

void writeLibrary( const wxString& aPath, const char* aValue, const char* aPinNumber )
{
    std::ofstream out( aPath.ToStdString(), std::ios::binary | std::ios::trunc );

    out << "(kicad_symbol_lib (version 20241209) (generator \"qa\")\n"
           "  (symbol \"S\"\n"
           "    (property \"Reference\" \"U\" (at 0 0 0) (effects (font (size 1.27 1.27))))\n"
           "    (property \"Value\" \"" << aValue << "\" (at 0 0 0) (effects (font (size 1.27 1.27))))\n"
           "    (symbol \"S_1_1\"\n"
           "      (pin passive line (at 0 0 0) (length 2.54)\n"
           "        (name \"A\" (effects (font (size 1.27 1.27))))\n"
           "        (number \"" << aPinNumber << "\" (effects (font (size 1.27 1.27)))))))\n"
           ")\n";
}

} // namespace


BOOST_AUTO_TEST_SUITE( SymbolLibraryFresh )


BOOST_AUTO_TEST_CASE( LoadSymbolSeesSameSizeRewrite )
{
    wxString dir = wxFileName::CreateTempFileName( wxS( "kicad_qa_sym_fresh_" ) );
    wxRemoveFile( dir );
    wxFileName::Mkdir( dir );

    const wxString path = wxFileName( dir, wxS( "fresh.kicad_sym" ) ).GetFullPath();

    writeLibrary( path, "AAA", "1" );

    SCH_IO_KICAD_SEXPR io;
    LIB_SYMBOL*        sym = io.LoadSymbol( path, wxS( "S" ) );
    BOOST_REQUIRE( sym );
    BOOST_CHECK_EQUAL( sym->GetValueField().GetText(), wxS( "AAA" ) );
    BOOST_CHECK_EQUAL( sym->GetGraphicalPins().size(), 1u );

    // Same length, written right away: only a sub-second file time tells the versions apart
    writeLibrary( path, "BBB", "2" );

    sym = io.LoadSymbol( path, wxS( "S" ) );
    BOOST_REQUIRE( sym );
    BOOST_CHECK_EQUAL( sym->GetValueField().GetText(), wxS( "BBB" ) );
    BOOST_REQUIRE_EQUAL( sym->GetGraphicalPins().size(), 1u );
    BOOST_CHECK_EQUAL( sym->GetGraphicalPins()[0]->GetNumber(), wxS( "2" ) );

    wxFileName::Rmdir( dir, wxPATH_RMDIR_RECURSIVE );
}


BOOST_AUTO_TEST_SUITE_END()
