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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * FOOTPRINT::FootprintNeedsUpdate() compares the 3D models (ModelsNeedUpdate()), so DRC's
 * lib_footprint_mismatch, "Update Footprints from Library" and the footprint diff see a board
 * copy whose model differs from the library's.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <board.h>
#include <embedded_files.h>
#include <footprint.h>
#include <pad.h>
#include <reporter.h>
#include <wildcards_and_files_ext.h>

#include <functional>
#include <memory>


namespace
{

struct MODEL_PARITY_FIXTURE
{
    MODEL_PARITY_FIXTURE()
    {
        m_fp = new FOOTPRINT( &m_board );
        m_board.Add( m_fp );

        FP_3DMODEL model;
        model.m_Filename = wxS( "${KIPRJMOD}/3dmodels/part.step" );
        model.m_Offset = VECTOR3D( 0.1, 0.2, 0.3 );
        model.m_Rotation = VECTOR3D( 0, 0, 90 );
        m_fp->Models().push_back( model );
    }

    std::unique_ptr<FOOTPRINT> libCopy() const
    {
        std::unique_ptr<FOOTPRINT> lib( static_cast<FOOTPRINT*>( m_fp->Clone() ) );
        lib->SetParent( nullptr );
        lib->SetParentGroup( nullptr );
        return lib;
    }

    /// The report of FootprintNeedsUpdate() when aChange is applied to the library copy
    wxString differenceAfter( const std::function<void( FOOTPRINT& )>& aChange, bool* aDiffers = nullptr ) const
    {
        std::unique_ptr<FOOTPRINT> lib = libCopy();
        aChange( *lib );

        WX_STRING_REPORTER reporter;
        const bool differs = m_fp->FootprintNeedsUpdate( lib.get(), BOARD_ITEM::COMPARE_FLAGS::DRC, &reporter );

        // the early-return path without a reporter agrees
        BOOST_CHECK_EQUAL( differs, m_fp->FootprintNeedsUpdate( lib.get(), BOARD_ITEM::COMPARE_FLAGS::DRC ) );

        if( aDiffers )
            *aDiffers = differs;

        return reporter.GetMessages();
    }

    BOARD      m_board;
    FOOTPRINT* m_fp = nullptr;
};

} // namespace


BOOST_FIXTURE_TEST_SUITE( LibraryParityModels, MODEL_PARITY_FIXTURE )


BOOST_AUTO_TEST_CASE( SameModelsDoNotDiffer )
{
    bool differs = true;
    differenceAfter( []( FOOTPRINT& ) {}, &differs );
    BOOST_CHECK( !differs );
    BOOST_CHECK( !m_fp->ModelsNeedUpdate( libCopy().get() ) );
}


BOOST_AUTO_TEST_CASE( EachModelPropertyIsCompared )
{
    struct CASE
    {
        const char*                       what;
        std::function<void( FOOTPRINT& )> change;
        const char*                       report;
    };

    const std::vector<CASE> cases = {
        { "file", []( FOOTPRINT& f ) { f.Models()[0].m_Filename = wxS( "${KIPRJMOD}/3dmodels/part-v3.step" ); },
          "3D model 1 file differs" },
        { "offset", []( FOOTPRINT& f ) { f.Models()[0].m_Offset.z = 0.5; }, "3D model 1 offset differs" },
        { "rotation", []( FOOTPRINT& f ) { f.Models()[0].m_Rotation.z = 180; }, "3D model 1 rotation differs" },
        { "scale", []( FOOTPRINT& f ) { f.Models()[0].m_Scale.x = 2; }, "3D model 1 scale differs" },
        { "opacity", []( FOOTPRINT& f ) { f.Models()[0].m_Opacity = 0.5; }, "3D model 1 opacity differs" },
        { "show", []( FOOTPRINT& f ) { f.Models()[0].m_Show = false; }, "3D model 1 is shown here, hidden in library" },
        { "count", []( FOOTPRINT& f ) { f.Models().push_back( FP_3DMODEL() ); }, "3D model count differs (1 here, 2 in library)" },
        { "none", []( FOOTPRINT& f ) { f.Models().clear(); }, "3D model count differs (1 here, 0 in library)" },
    };

    for( const CASE& c : cases )
    {
        bool           differs = false;
        const wxString report = differenceAfter( c.change, &differs );

        BOOST_CHECK_MESSAGE( differs, c.what << ": a model difference must make the footprint differ" );
        BOOST_CHECK_MESSAGE( report.Contains( wxString::FromUTF8( c.report ) ),
                             c.what << ": report '" << report.ToStdString() << "' lacks '" << c.report << "'" );
    }
}


BOOST_AUTO_TEST_CASE( TinyFloatNoiseIsNotADifference )
{
    bool differs = true;
    differenceAfter( []( FOOTPRINT& f ) { f.Models()[0].m_Offset.x += 1e-9; }, &differs );
    BOOST_CHECK( !differs );
}


BOOST_AUTO_TEST_CASE( EmbeddedModelContentIsCompared )
{
    const wxString name = wxS( "part.step" );

    m_fp->Models()[0].m_Filename = wxString( FILEEXT::KiCadUriPrefix ) + wxS( "://" ) + name;

    EMBEDDED_FILES::EMBEDDED_FILE* file = new EMBEDDED_FILES::EMBEDDED_FILE;
    file->name = name;
    file->type = EMBEDDED_FILES::EMBEDDED_FILE::FILE_TYPE::MODEL;
    file->data_hash = "1111";
    file->compressedEncodedData = "AAAA";
    m_fp->GetEmbeddedFiles()->AddFile( file );

    bool differs = true;
    differenceAfter( []( FOOTPRINT& ) {}, &differs );
    BOOST_CHECK_MESSAGE( !differs, "the same embedded model must not differ" );

    const wxString report = differenceAfter(
            [&]( FOOTPRINT& f )
            {
                // a clone shares the file entries: replace, do not edit
                EMBEDDED_FILES::EMBEDDED_FILE* libFile = new EMBEDDED_FILES::EMBEDDED_FILE;
                libFile->name = name;
                libFile->type = EMBEDDED_FILES::EMBEDDED_FILE::FILE_TYPE::MODEL;
                libFile->data_hash = "2222";
                libFile->compressedEncodedData = "BBBB";
                f.GetEmbeddedFiles()->RemoveFile( name );
                f.GetEmbeddedFiles()->AddFile( libFile );
            },
            &differs );

    BOOST_CHECK( differs );
    BOOST_CHECK_MESSAGE( report.Contains( wxS( "embedded file 'part.step' differs" ) ), report.ToStdString() );
}


BOOST_AUTO_TEST_CASE( ParityLeavesTheBoardIndexAlone )
{
    PAD* pad = new PAD( m_fp );
    pad->SetNumber( wxS( "1" ) );
    m_fp->Add( pad );
    m_board.CacheItemById( pad );

    const size_t indexed = m_board.GetItemByIdCache().size();

    bool differs = true;
    differenceAfter( []( FOOTPRINT& ) {}, &differs );

    BOOST_CHECK( !differs );
    BOOST_CHECK_EQUAL( m_board.GetItemByIdCache().size(), indexed );
    BOOST_CHECK( m_board.GetItemByIdCache().count( pad->m_Uuid )
                 && m_board.GetItemByIdCache().at( pad->m_Uuid ) == pad );
    BOOST_CHECK( pad->IsIndexedInBoard() );
}


BOOST_AUTO_TEST_CASE( LeadTrimVariantIsNoDifference )
{
    // the board shows a cut copy and hides the generic model, the library has the generic one
    std::unique_ptr<FOOTPRINT> lib = libCopy();

    FP_3DMODEL cut = m_fp->Models()[0];
    cut.m_Filename = wxS( "${KIPRJMOD}/3dmodels/.trimmed/part-1a2b.step" );
    m_fp->Models()[0].m_Show = false;
    m_fp->Models().push_back( cut );

    BOOST_CHECK( !m_fp->ModelsNeedUpdate( lib.get() ) );

    lib->Models()[0].m_Offset.z = 0.5;
    BOOST_CHECK( m_fp->ModelsNeedUpdate( lib.get() ) );
}


BOOST_AUTO_TEST_SUITE_END()
