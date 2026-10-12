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

#include <boost/test/unit_test.hpp>
#include <qa_utils/wx_utils/unit_test_utils.h>

#include <tool/tool_dispatcher.h>
#include <tool/tool_manager.h>
#include <tool/tool_interactive.h>
#include <tool/action_menu.h>
#include <view/view.h>
#include <view/view_controls.h>
#include <kiplatform/ui.h>

#include <wx/display.h>
#include <wx/evtloop.h>

/**
 * Tests for TOOL_DISPATCHER::ShouldDropAutoRepeat, the pure decision that discards backlogged
 * OS key auto-repeat events delivered after a hotkey was released (issue 24540).
 */

BOOST_AUTO_TEST_SUITE( ToolDispatcherAutoRepeat )

// A single key press whose key has already been released (a quick tap) must always run.
BOOST_AUTO_TEST_CASE( LeadingEdgeAlwaysRuns )
{
    int        lastKey = 0;
    wxLongLong lastTime = 0;

    bool drop = TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', /*now*/ 1000, /*keyIsDown*/ false,
                                                       lastKey, lastTime );

    BOOST_CHECK( !drop );
    BOOST_CHECK_EQUAL( lastKey, 'R' );
    BOOST_CHECK( lastTime == 1000 );
}

// Repeats that arrive within the burst window while the key is still held keep running.
BOOST_AUTO_TEST_CASE( HeldKeyRepeatsRun )
{
    int        lastKey = 0;
    wxLongLong lastTime = 0;

    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1000, true, lastKey, lastTime ) );
    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1030, true, lastKey, lastTime ) );
    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1060, true, lastKey, lastTime ) );
}

// The defect: repeats from the same burst that arrive after release must be dropped.
BOOST_AUTO_TEST_CASE( StaleRepeatAfterReleaseDropped )
{
    int        lastKey = 0;
    wxLongLong lastTime = 0;

    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1000, true, lastKey, lastTime ) );

    // Backlogged events still in the queue after the key came up.
    BOOST_CHECK( TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1030, false, lastKey, lastTime ) );
    BOOST_CHECK( TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1060, false, lastKey, lastTime ) );
}

// A second deliberate tap after a human-scale gap is a new burst and must run again.
BOOST_AUTO_TEST_CASE( SecondDeliberateTapRuns )
{
    int        lastKey = 0;
    wxLongLong lastTime = 0;

    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1000, false, lastKey, lastTime ) );

    // Well beyond the auto-repeat window, so this is treated as a fresh press, not stale backlog.
    wxLongLong later = 1000 + TOOL_DISPATCHER::AutoRepeatWindowMs + 1;
    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', later, false, lastKey, lastTime ) );
}

// Switching to a different key starts a new burst whose leading edge runs.
BOOST_AUTO_TEST_CASE( DifferentKeyIsNewBurst )
{
    int        lastKey = 0;
    wxLongLong lastTime = 0;

    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'R', 1000, true, lastKey, lastTime ) );

    // A different hotkey arriving immediately is not part of the R burst.
    BOOST_CHECK( !TOOL_DISPATCHER::ShouldDropAutoRepeat( 'F', 1010, false, lastKey, lastTime ) );
}

BOOST_AUTO_TEST_SUITE_END()


namespace
{
// TOOL_EVENT's constructor reads ACTIONS::cancelInteractive, a static in another translation
// unit, so the event has to be initialized on first use rather than at file scope
const TOOL_EVENT& armMenuEvent()
{
    static const TOOL_EVENT evt( TC_COMMAND, TA_ACTION, std::string( "test.armContextMenu" ) );

    return evt;
}


class YIELDING_EVENT_LOOP : public wxEventLoop
{
public:
    bool IsYielding() const override { return true; }
};


class CONTEXT_MENU_TEST_TOOL : public TOOL_INTERACTIVE
{
public:
    CONTEXT_MENU_TEST_TOOL() :
            TOOL_INTERACTIVE( "test.contextMenuTool" ),
            m_menuRan( false ),
            m_menu( true )
    {
    }

    void Reset( RESET_REASON aReason ) override {}

    int ArmMenu( const TOOL_EVENT& aEvent )
    {
        Activate();
        SetContextMenu( &m_menu, CMENU_NOW );
        Wait( TOOL_EVENT( TC_COMMAND, TA_CHOICE_MENU_CLOSED ) );
        m_menuRan = true;

        return 0;
    }

    bool m_menuRan;

private:
    void setTransitions() override { Go( &CONTEXT_MENU_TEST_TOOL::ArmMenu, armMenuEvent() ); }

    ACTION_MENU m_menu;
};
} // namespace


BOOST_AUTO_TEST_SUITE( ToolManagerContextMenu )

BOOST_AUTO_TEST_CASE( RightClickOpensMenuWhileEventLoopYields )
{
    if( !KI_TEST::CanDoDisplayTests() )
    {
        BOOST_TEST_MESSAGE( "Skipping test - no display available" );
        return;
    }

    TOOL_MANAGER            mgr;
    CONTEXT_MENU_TEST_TOOL* tool = new CONTEXT_MENU_TEST_TOOL();

    mgr.SetEnvironment( nullptr, nullptr, nullptr, nullptr, nullptr );
    mgr.RegisterTool( tool );
    mgr.ResetTools( TOOL_BASE::RUN );

    YIELDING_EVENT_LOOP  loop;
    wxEventLoopActivator activate( &loop );

    mgr.ProcessEvent( armMenuEvent() );
    BOOST_CHECK( !tool->m_menuRan );

    mgr.ProcessEvent( TOOL_EVENT( TC_MOUSE, TA_MOUSE_CLICK, BUT_RIGHT ) );
    BOOST_CHECK( tool->m_menuRan );
}

BOOST_AUTO_TEST_SUITE_END()


namespace
{
/// View controls with a pointer position the test sets (screen = world)
class FAKE_VIEW_CONTROLS : public KIGFX::VIEW_CONTROLS
{
public:
    explicit FAKE_VIEW_CONTROLS( KIGFX::VIEW* aView ) : KIGFX::VIEW_CONTROLS( aView ) {}

    void PinCursorInsideNonAutoscrollArea( bool aWarpMouseCursor ) override {}
    VECTOR2D GetMousePosition( bool aWorldCoordinates = true ) const override { return m_pos; }
    VECTOR2D GetRawCursorPosition( bool aSnappingEnabled = true ) const override { return m_pos; }
    VECTOR2D GetCursorPosition( bool aEnableSnapping ) const override { return m_pos; }
    void SetCursorPosition( const VECTOR2D& aPosition, bool aWarpView = true, bool aTriggeredByArrows = false,
                            long aArrowCommand = 0 ) override {}
    void SetCrossHairCursorPosition( const VECTOR2D& aPosition, bool aWarpView = true ) override {}
    void WarpMouseCursor( const VECTOR2D& aPosition, bool aWorldCoordinates = false,
                          bool aWarpView = false ) override {}
    void CenterOnCursor() override {}
    void ShowCursor( bool aEnabled ) override {}     // the base draws through the view's GAL

    VECTOR2D m_pos;
};


/// Records the mouse button events the tool manager delivers (motion left out)
class INPUT_RECORDER : public TOOL_INTERACTIVE
{
public:
    INPUT_RECORDER() : TOOL_INTERACTIVE( "test.inputRecorder" ) {}

    void Reset( RESET_REASON aReason ) override {}

    int Record( const TOOL_EVENT& aEvent )
    {
        if( aEvent.Category() == TC_MOUSE && aEvent.Action() != TA_MOUSE_MOTION )
            m_events.push_back( aEvent.Action() );

        return 0;
    }

    std::vector<int> m_events;

private:
    void setTransitions() override { Go( &INPUT_RECORDER::Record, TOOL_EVENT( TC_MOUSE, TA_ANY ) ); }
};
} // namespace


/// The real mouse path: wx events into TOOL_DISPATCHER::DispatchWxEvent, the buttons held as the
/// platform reports them (fed through the pointer gate, the physical mouse is not touched)
struct WX_INPUT_FIXTURE
{
    WX_INPUT_FIXTURE() : controls( &view ), dispatcher( &mgr )
    {
        recorder = new INPUT_RECORDER();
        mgr.SetEnvironment( nullptr, nullptr, &controls, nullptr, nullptr );
        mgr.RegisterTool( recorder );
        mgr.ResetTools( TOOL_BASE::RUN );
        KIPLATFORM::UI::SetInjectedPointer( &pointer );
    }

    ~WX_INPUT_FIXTURE() { KIPLATFORM::UI::SetInjectedPointer( nullptr ); }

    void send( wxEventType aType, int aX, int aY, bool aLeftHeld )
    {
        controls.m_pos = VECTOR2D( aX, aY );
        pointer.screenPosition = wxPoint( aX, aY );
        pointer.left = aLeftHeld;
        KIPLATFORM::UI::SetInjectedPointer( &pointer );

        wxMouseEvent event( aType );
        event.SetPosition( wxPoint( aX, aY ) );
        event.SetLeftDown( aLeftHeld );
        dispatcher.DispatchWxEvent( event );
    }

    KIGFX::VIEW                      view;
    FAKE_VIEW_CONTROLS               controls;
    TOOL_MANAGER                     mgr;
    TOOL_DISPATCHER                  dispatcher;
    INPUT_RECORDER*                  recorder = nullptr;
    KIPLATFORM::UI::INJECTED_POINTER pointer;
};


BOOST_FIXTURE_TEST_SUITE( ToolDispatcherWxInput, WX_INPUT_FIXTURE )

BOOST_AUTO_TEST_CASE( Click )
{
    send( wxEVT_LEFT_DOWN, 100, 100, true );
    send( wxEVT_LEFT_UP, 100, 100, false );

    BOOST_CHECK( recorder->m_events == std::vector<int>( { TA_MOUSE_DOWN, TA_MOUSE_CLICK } ) );
}

BOOST_AUTO_TEST_CASE( Drag )
{
    send( wxEVT_LEFT_DOWN, 100, 100, true );
    send( wxEVT_MOTION, 300, 100, true );
    send( wxEVT_LEFT_UP, 300, 100, false );

    BOOST_CHECK( recorder->m_events == std::vector<int>( { TA_MOUSE_DOWN, TA_MOUSE_DRAG, TA_MOUSE_UP } ) );
}

BOOST_AUTO_TEST_CASE( DoubleClick )
{
    send( wxEVT_LEFT_DOWN, 100, 100, true );
    send( wxEVT_LEFT_UP, 100, 100, false );
    send( wxEVT_LEFT_DCLICK, 100, 100, true );
    send( wxEVT_LEFT_UP, 100, 100, false );

    BOOST_CHECK( recorder->m_events == std::vector<int>( { TA_MOUSE_DOWN, TA_MOUSE_CLICK, TA_MOUSE_DBLCLICK } ) );
}

// The second press far from the first is a fast click-drag, not a double click
BOOST_AUTO_TEST_CASE( FastClickDrag )
{
    send( wxEVT_LEFT_DOWN, 100, 100, true );
    send( wxEVT_LEFT_UP, 100, 100, false );
    send( wxEVT_LEFT_DCLICK, 300, 100, true );
    send( wxEVT_MOTION, 400, 100, true );
    send( wxEVT_LEFT_UP, 400, 100, false );

    BOOST_CHECK( recorder->m_events
                 == std::vector<int>( { TA_MOUSE_DOWN, TA_MOUSE_CLICK, TA_MOUSE_DOWN, TA_MOUSE_DRAG, TA_MOUSE_UP } ) );
}

BOOST_AUTO_TEST_SUITE_END()
