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
 * KIPLATFORM::IO::TimestampFile on the host's file system: absent / directory, size, sub-second
 * mtime steps, a same-size rewrite, non-ASCII paths.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <kiplatform/io.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>

namespace fs = std::filesystem;

namespace
{
struct TEMP_DIR
{
    TEMP_DIR()
    {
        std::random_device rd;
        path = fs::temp_directory_path() / ( "kicad_qa_timestamp_" + std::to_string( rd() ) );
        fs::create_directories( path );
    }

    ~TEMP_DIR()
    {
        std::error_code ec;
        fs::remove_all( path, ec );
    }

    fs::path path;
};


void writeFile( const fs::path& aPath, const std::string& aText )
{
    std::ofstream out( aPath, std::ios::binary | std::ios::trunc );
    out << aText;
}


wxString wxPath( const fs::path& aPath )
{
    const auto utf8 = aPath.u8string();
    return wxString::FromUTF8( reinterpret_cast<const char*>( utf8.c_str() ), utf8.size() );
}
} // namespace


BOOST_AUTO_TEST_SUITE( KiplatformTimestampFile )


BOOST_AUTO_TEST_CASE( AbsentAndDirectoryAreZero )
{
    TEMP_DIR dir;

    BOOST_CHECK_EQUAL( KIPLATFORM::IO::TimestampFile( wxPath( dir.path / "missing.kicad_mod" ) ), 0 );
    BOOST_CHECK_EQUAL( KIPLATFORM::IO::TimestampFile( wxPath( dir.path ) ), 0 );
    BOOST_CHECK_EQUAL( KIPLATFORM::IO::TimestampFile( wxString() ), 0 );
}


BOOST_AUTO_TEST_CASE( StableUntilChanged )
{
    TEMP_DIR       dir;
    const fs::path file = dir.path / "a.kicad_mod";

    writeFile( file, "(footprint \"A\")" );
    const long long first = KIPLATFORM::IO::TimestampFile( wxPath( file ) );

    BOOST_CHECK_NE( first, 0 );
    BOOST_CHECK_EQUAL( KIPLATFORM::IO::TimestampFile( wxPath( file ) ), first );

    // same mtime, other size
    const fs::file_time_type when = fs::last_write_time( file );
    writeFile( file, "(footprint \"AB\")" );
    fs::last_write_time( file, when );
    BOOST_CHECK_NE( KIPLATFORM::IO::TimestampFile( wxPath( file ) ), first );
}


BOOST_AUTO_TEST_CASE( SubSecondSameSizeRewrite )
{
    TEMP_DIR       dir;
    const fs::path file = dir.path / "b.kicad_sym";

    writeFile( file, "(kicad_symbol_lib 1)" );
    const fs::file_time_type when = fs::last_write_time( file );
    const long long          before = KIPLATFORM::IO::TimestampFile( wxPath( file ) );

    // a rewrite of the same size 1 ms later: a seconds-only stamp would miss it
    writeFile( file, "(kicad_symbol_lib 2)" );
    fs::last_write_time( file, when + std::chrono::milliseconds( 1 ) );
    const long long after = KIPLATFORM::IO::TimestampFile( wxPath( file ) );

    BOOST_CHECK_NE( before, 0 );
    BOOST_CHECK_NE( after, before );
}


BOOST_AUTO_TEST_CASE( NonAsciiPath )
{
    TEMP_DIR       dir;
    const fs::path sub = dir.path / fs::path( u8"библиотека" );
    const fs::path file = sub / fs::path( u8"résistance_µ.kicad_mod" );

    fs::create_directories( sub );
    writeFile( file, "(footprint \"R\")" );

    BOOST_CHECK_NE( KIPLATFORM::IO::TimestampFile( wxPath( file ) ), 0 );
}


BOOST_AUTO_TEST_SUITE_END()
