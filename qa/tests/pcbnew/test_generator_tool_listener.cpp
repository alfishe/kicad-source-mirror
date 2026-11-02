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
 * A property change on a generator regenerates it once, by the generator tool of its own board:
 * with two boards open (each with a generator tool), the other board's tool leaves it alone.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <api/headless_pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <pcb_generator.h>
#include <pcb_track.h>
#include <properties/property_mgr.h>
#include <settings/settings_manager.h>
#include <tool/tool_manager.h>
#include <tools/generator_tool.h>

#include <vector>


namespace
{

struct EDIT_CALL
{
    GENERATOR_TOOL* tool;
    BOARD*          board;
};


class RECORDING_GENERATOR : public PCB_GENERATOR
{
public:
    explicit RECORDING_GENERATOR( BOARD_ITEM* aParent ) :
            PCB_GENERATOR( aParent, F_Cu )
    {
    }

    void EditStart( GENERATOR_TOOL* aTool, BOARD* aBoard, BOARD_COMMIT* aCommit ) override
    {
        m_calls.push_back( { aTool, aBoard } );
    }

    bool Update( GENERATOR_TOOL* aTool, BOARD* aBoard, BOARD_COMMIT* aCommit ) override { return true; }
    void EditFinish( GENERATOR_TOOL* aTool, BOARD* aBoard, BOARD_COMMIT* aCommit ) override {}
    void EditCancel( GENERATOR_TOOL* aTool, BOARD* aBoard, BOARD_COMMIT* aCommit ) override {}
    void Remove( GENERATOR_TOOL* aTool, BOARD* aBoard, BOARD_COMMIT* aCommit ) override {}

    wxString GetPluralName() const override { return wxS( "Recording generators" ); }
    wxString GetCommitMessage() const override { return wxS( "Recording generator" ); }
    wxString GetClass() const override { return wxS( "RECORDING_GENERATOR" ); }

    EDA_ITEM* Clone() const override { return new RECORDING_GENERATOR( *this ); }

    std::vector<EDIT_CALL> m_calls;
};


struct GENERATOR_LISTENER_FIXTURE
{
    GENERATOR_LISTENER_FIXTURE()
    {
        PROPERTY_MANAGER& propMgr = PROPERTY_MANAGER::Instance();
        REGISTER_TYPE( RECORDING_GENERATOR );
        propMgr.AddTypeCast( new TYPE_CAST<RECORDING_GENERATOR, PCB_GENERATOR> );
        propMgr.AddTypeCast( new TYPE_CAST<RECORDING_GENERATOR, BOARD_ITEM> );
        propMgr.InheritsAfter( TYPE_HASH( RECORDING_GENERATOR ), TYPE_HASH( PCB_GENERATOR ) );
        propMgr.InheritsAfter( TYPE_HASH( RECORDING_GENERATOR ), TYPE_HASH( BOARD_ITEM ) );
        propMgr.Rebuild();

        m_settingsManager.LoadProject( "" );

        for( int i = 0; i < 2; i++ )
        {
            m_contexts.push_back( std::make_unique<HEADLESS_PCB_CONTEXT>( std::make_unique<BOARD>(),
                                                                          &m_settingsManager.Prj(), nullptr ) );
            m_contexts.back()->GetToolManager()->RegisterTool( new GENERATOR_TOOL );
        }
    }

    ~GENERATOR_LISTENER_FIXTURE()
    {
        m_contexts.clear();
    }

    GENERATOR_TOOL* tool( int aIndex )
    {
        return m_contexts[aIndex]->GetToolManager()->GetTool<GENERATOR_TOOL>();
    }

    BOARD* board( int aIndex ) { return m_contexts[aIndex]->GetBoard(); }

    SETTINGS_MANAGER                                   m_settingsManager;
    std::vector<std::unique_ptr<HEADLESS_PCB_CONTEXT>> m_contexts;
};

} // namespace


BOOST_FIXTURE_TEST_SUITE( GeneratorToolListener, GENERATOR_LISTENER_FIXTURE )


BOOST_AUTO_TEST_CASE( PropertyChangeRegeneratesOnItsOwnBoardOnly )
{
    BOOST_REQUIRE( tool( 0 ) && tool( 1 ) && tool( 0 ) != tool( 1 ) );

    auto* generator = new RECORDING_GENERATOR( board( 0 ) );
    board( 0 )->Add( generator );

    PROPERTY_MANAGER& propMgr = PROPERTY_MANAGER::Instance();
    PROPERTY_BASE*    property = propMgr.GetProperty( TYPE_HASH( BOARD_ITEM ), _HKI( "Position X" ) );
    BOOST_REQUIRE( property );

    {
        BOARD_COMMIT            commit( m_contexts[0]->GetToolManager() );
        PROPERTY_COMMIT_HANDLER handler( &commit );

        commit.Modify( generator );
        wxAny value = 1000000;
        BOOST_REQUIRE( generator->Set( property, value ) );
        commit.Push( wxS( "Move generator" ) );
    }

    BOOST_REQUIRE_EQUAL( generator->m_calls.size(), 1 );
    BOOST_CHECK( generator->m_calls[0].tool == tool( 0 ) );
    BOOST_CHECK( generator->m_calls[0].board == board( 0 ) );
}


BOOST_AUTO_TEST_CASE( PropertyChangeWithoutCommitIsIgnored )
{
    auto* generator = new RECORDING_GENERATOR( board( 1 ) );
    board( 1 )->Add( generator );

    PROPERTY_BASE* property = PROPERTY_MANAGER::Instance().GetProperty( TYPE_HASH( BOARD_ITEM ),
                                                                        _HKI( "Position X" ) );
    BOOST_REQUIRE( property );

    wxAny value = 2000000;
    BOOST_REQUIRE( generator->Set( property, value ) );

    BOOST_CHECK( generator->m_calls.empty() );
}


BOOST_AUTO_TEST_SUITE_END()
