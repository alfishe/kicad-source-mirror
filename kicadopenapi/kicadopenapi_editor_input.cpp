/// @file kicadopenapi_editor_input.cpp
/// @brief Editor actions and injected input for the API (see kicadopenapi_editor_input.h).
#include <kicadopenapi_annotations.h>
#include <kicadopenapi_editor_input.h>
#include <kicadopenapi_keepalive.h>

#include <base_units.h>
#include <class_draw_panel_gal.h>
#include <eda_draw_frame.h>
#include <hotkeys_basic.h>
#include <kiplatform/ui.h>
#include <math/util.h>
#include <tool/action_manager.h>
#include <tool/tool_action.h>
#include <tool/tool_dispatcher.h>
#include <tool/tool_manager.h>
#include <view/view.h>
#include <view/wx_view_controls.h>

#include <wx/app.h>
#include <wx/utils.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <map>
#include <thread>


namespace
{
using INPUT = TOOL_DISPATCHER::INPUT;


struct TARGET
{
    EDA_DRAW_FRAME*          frame = nullptr;
    EDA_DRAW_PANEL_GAL*      canvas = nullptr;
    TOOL_DISPATCHER*         dispatcher = nullptr;
    KIGFX::WX_VIEW_CONTROLS* controls = nullptr;
    const EDA_IU_SCALE*      scale = nullptr;
    const KOPENAPI_EDITOR_HOST* host = nullptr;
};


/// Buttons held by injected input between calls (a drag can span calls)
int g_heldButtons = 0;

/// Last injected pointer position in document units (mm), for the answer
VECTOR2D g_pointerMm;


std::string checkTarget( EDA_DRAW_FRAME* aFrame, const EDA_IU_SCALE& aScale, const KOPENAPI_EDITOR_HOST& aHost,
                         TARGET& aTarget )
{
    if( !aFrame || !aFrame->IsShown() )
        return "no editor window open";

    aTarget.frame = aFrame;
    aTarget.canvas = aFrame->GetCanvas();
    aTarget.scale = &aScale;
    aTarget.host = &aHost;

    if( !aTarget.canvas || !aFrame->GetToolManager() )
        return "the editor has no canvas";

    aTarget.dispatcher = aTarget.canvas->GetEventDispatcher();
    aTarget.controls = dynamic_cast<KIGFX::WX_VIEW_CONTROLS*>( aTarget.canvas->GetViewControls() );

    if( !aTarget.dispatcher || !aTarget.controls )
        return "the editor's canvas takes no tool input";

    return {};
}


/// Lets the tools' posted events, repaints and timers run (user input stays queued)
void pump()
{
    if( wxTheApp )
        wxTheApp->ProcessPendingEvents();

    KopenapiKeepUiAlive();
}


/// @return the document point of a step value: [x, y] mm, or an item (its centre), or an error
std::string pointMm( const TARGET& aTarget, const nlohmann::json& aValue, VECTOR2D& aMm )
{
    if( aValue.is_array() && aValue.size() == 2 && aValue[0].is_number() && aValue[1].is_number() )
    {
        aMm = VECTOR2D( aValue[0].get<double>(), aValue[1].get<double>() );
        return {};
    }

    if( aValue.is_string() && aTarget.host->resolve )
    {
        if( std::optional<BOX2I> box = aTarget.host->resolve( aValue.get<std::string>() ) )
        {
            const VECTOR2I c = box->Centre();
            aMm = VECTOR2D( aTarget.scale->IUTomm( c.x ), aTarget.scale->IUTomm( c.y ) );
            return {};
        }

        return "no item '" + aValue.get<std::string>() + "' (a reference, REF.NUMBER pad / pin, or a uuid)";
    }

    return "a point is [x, y] in mm or an item reference";
}


/// @return the client (canvas) position of a document point in mm, or an error
std::string toClient( const TARGET& aTarget, const VECTOR2D& aMm, wxPoint& aClient )
{
    const VECTOR2D world( aTarget.scale->mmToIU( aMm.x ), aTarget.scale->mmToIU( aMm.y ) );
    const VECTOR2D screen = aTarget.canvas->GetView()->ToScreen( world );
    const wxSize   size = aTarget.canvas->GetClientSize();

    aClient = wxPoint( KiROUND( screen.x ), KiROUND( screen.y ) );

    if( aClient.x < 0 || aClient.y < 0 || aClient.x >= size.x || aClient.y >= size.y )
    {
        const BOX2D view = aTarget.canvas->GetView()->GetViewport();
        return "point (" + std::to_string( aMm.x ) + ", " + std::to_string( aMm.y )
               + ") mm is outside the visible canvas (" + std::to_string( aTarget.scale->IUTomm( view.GetLeft() ) )
               + ".." + std::to_string( aTarget.scale->IUTomm( view.GetRight() ) ) + ", "
               + std::to_string( aTarget.scale->IUTomm( view.GetTop() ) ) + ".."
               + std::to_string( aTarget.scale->IUTomm( view.GetBottom() ) ) + " mm); pan or zoom first";
    }

    return {};
}


int parseModifiers( const nlohmann::json& aStep, std::string& aError )
{
    int mods = 0;

    for( const nlohmann::json& m : aStep.value( "mods", nlohmann::json::array() ) )
    {
        const std::string name = m.is_string() ? m.get<std::string>() : std::string();

        if( name == "shift" )
            mods |= MD_SHIFT;
        else if( name == "ctrl" || name == "cmd" )
            mods |= MD_CTRL;
        else if( name == "alt" )
            mods |= MD_ALT;
        else
            aError = "unknown modifier '" + name + "' (shift, ctrl / cmd, alt)";
    }

    return mods;
}


int parseButton( const nlohmann::json& aStep, std::string& aError )
{
    const std::string name = aStep.value( "button", std::string( "left" ) );

    if( name == "left" )
        return BUT_LEFT;

    if( name == "right" )
        return BUT_RIGHT;

    if( name == "middle" )
        return BUT_MIDDLE;

    aError = "unknown button '" + name + "' (left, right, middle)";
    return 0;
}


int parseKey( const std::string& aName, std::string& aError )
{
    static const std::map<std::string, int> names = {
        { "Escape", WXK_ESCAPE }, { "Enter", WXK_RETURN }, { "Tab", WXK_TAB }, { "Space", WXK_SPACE },
        { "Delete", WXK_DELETE }, { "Backspace", WXK_BACK }, { "Insert", WXK_INSERT }, { "Home", WXK_HOME },
        { "End", WXK_END }, { "PageUp", WXK_PAGEUP }, { "PageDown", WXK_PAGEDOWN }, { "Up", WXK_UP },
        { "Down", WXK_DOWN }, { "Left", WXK_LEFT }, { "Right", WXK_RIGHT }
    };

    if( auto it = names.find( aName ); it != names.end() )
        return it->second;

    if( aName.size() >= 2 && aName.size() <= 3 && aName[0] == 'F' && std::isdigit( (unsigned char) aName[1] ) )
    {
        const int n = std::stoi( aName.substr( 1 ) );

        if( n >= 1 && n <= 12 )
            return WXK_F1 + n - 1;
    }

    // One printable character: KiCad's hotkeys use upper-case letter codes
    if( aName.size() == 1 && std::isprint( (unsigned char) aName[0] ) )
        return std::toupper( (unsigned char) aName[0] );

    aError = "unknown key '" + aName + "' (one character, Escape, Enter, Tab, Space, Delete, Backspace, Insert, "
             "Home, End, PageUp, PageDown, Up, Down, Left, Right, F1..F12)";
    return 0;
}


/// Puts the injected pointer at a client position with the held buttons and modifiers
void injectPointer( const TARGET& aTarget, const wxPoint& aClient, int aModifiers )
{
    KIPLATFORM::UI::INJECTED_POINTER pointer;
    pointer.screenPosition = aTarget.canvas->ClientToScreen( aClient );
    pointer.left = g_heldButtons & BUT_LEFT;
    pointer.middle = g_heldButtons & BUT_MIDDLE;
    pointer.right = g_heldButtons & BUT_RIGHT;
    pointer.shift = aModifiers & MD_SHIFT;
    pointer.control = aModifiers & MD_CTRL;
    pointer.alt = aModifiers & MD_ALT;
    KIPLATFORM::UI::SetInjectedPointer( &pointer );
}


/// One pointer / key input into the tools
void dispatch( const TARGET& aTarget, INPUT::KIND aKind, int aButton, int aModifiers )
{
    INPUT input;
    input.kind = aKind;
    input.button = aButton;
    input.heldButtons = g_heldButtons;
    input.modifiers = aModifiers;
    aTarget.dispatcher->DispatchInput( input );
    pump();
}


void releasePointerIfIdle()
{
    if( !g_heldButtons )
        KIPLATFORM::UI::SetInjectedPointer( nullptr );
}


nlohmann::json state( const TARGET& aTarget )
{
    nlohmann::json out = { { "tool", aTarget.frame->ToolStackIsEmpty() ? nlohmann::json()
                                                                       : nlohmann::json( aTarget.frame->CurrentToolName() ) },
                           { "pointer", { g_pointerMm.x, g_pointerMm.y } },
                           { "held", nlohmann::json::array() } };

    for( auto [bit, name] : { std::pair{ BUT_LEFT, "left" }, std::pair{ BUT_RIGHT, "right" }, std::pair{ BUT_MIDDLE, "middle" } } )
    {
        if( g_heldButtons & bit )
            out["held"].push_back( name );
    }

    if( aTarget.host->state )
        aTarget.host->state( out );

    return out;
}


/// Moves the pointer to a document point: at once, or gliding there over aMs (eased, ~30 frames a
/// second), with the held buttons; every position goes through the tools as a real move does
std::string pointerTo( const TARGET& aTarget, const VECTOR2D& aMm, int aModifiers, int aMs )
{
    const VECTOR2D from = g_pointerMm;
    const int      frames = aMs > 0 ? std::max( 2, aMs / 33 ) : 1;

    for( int i = 1; i <= frames; ++i )
    {
        const double   t = double( i ) / frames;
        const double   eased = t * t * ( 3 - 2 * t );
        const VECTOR2D at = frames == 1 ? aMm : from + ( aMm - from ) * eased;
        wxPoint        client;

        if( std::string bad = toClient( aTarget, at, client ); !bad.empty() )
            return bad;

        g_pointerMm = at;
        injectPointer( aTarget, client, aModifiers );
        aTarget.controls->PointerMoved();
        dispatch( aTarget, INPUT::KIND::POINTER, 0, aModifiers );
        KopenapiAnnotatePointer( aTarget.frame, VECTOR2D( aTarget.scale->mmToIU( at.x ), aTarget.scale->mmToIU( at.y ) ),
                                 KOPENAPI_POINTER_EVENT::MOVE );

        if( frames > 1 )
        {
            const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds( aMs / frames );

            while( std::chrono::steady_clock::now() < until )
            {
                pump();
                std::this_thread::sleep_for( std::chrono::milliseconds( 4 ) );
            }
        }
    }

    return {};
}


/// A button change where the pointer is
void button( const TARGET& aTarget, INPUT::KIND aKind, int aButton, int aModifiers, KOPENAPI_POINTER_EVENT aShown )
{
    wxPoint client;
    toClient( aTarget, g_pointerMm, client );

    if( aKind == INPUT::KIND::BUTTON_DOWN || aKind == INPUT::KIND::BUTTON_DCLICK )
        g_heldButtons |= aButton;
    else if( aKind == INPUT::KIND::BUTTON_UP )
        g_heldButtons &= ~aButton;

    injectPointer( aTarget, client, aModifiers );
    dispatch( aTarget, aKind, aButton, aModifiers );
    KopenapiAnnotatePointer( aTarget.frame,
                             VECTOR2D( aTarget.scale->mmToIU( g_pointerMm.x ), aTarget.scale->mmToIU( g_pointerMm.y ) ),
                             aShown );
}


/// Runs one step; returns an error message or empty
std::string runStep( const TARGET& aTarget, const nlohmann::json& aStep )
{
    std::string error;
    const int   mods = parseModifiers( aStep, error );
    const int   ms = std::clamp( aStep.value( "ms", 0 ), 0, 20000 );

    if( !error.empty() )
        return error;

    // intents: what to do with which item, the helper finds how
    if( aStep.contains( "select" ) || aStep.contains( "drag" ) )
    {
        const bool        drag = aStep.contains( "drag" );
        const std::string ref = aStep.value( drag ? "drag" : "select", std::string() );
        VECTOR2D          grab;

        if( std::string bad = pointMm( aTarget, ref, grab ); !bad.empty() )
            return bad;

        if( !aTarget.host->select || !aTarget.host->select( ref ) )
            return "'" + ref + "' cannot be selected in this editor";

        pump();

        if( std::string bad = pointerTo( aTarget, grab, mods, drag ? ms / 3 : ms ); !bad.empty() )
            return bad;

        if( !drag )
            return {};

        VECTOR2D to;

        if( aStep.contains( "by" ) )
        {
            const nlohmann::json& by = aStep["by"];

            if( !by.is_array() || by.size() != 2 )
                return "by is [dx, dy] mm";

            to = grab + VECTOR2D( by[0].get<double>(), by[1].get<double>() );
        }
        else if( std::string bad = pointMm( aTarget, aStep.value( "to", nlohmann::json() ), to ); !bad.empty() )
        {
            return "drag needs to: [x, y] mm (or an item) or by: [dx, dy] mm";
        }

        // the selection is dragged from inside it (KiCad's "drag selected items"), so an item lying
        // on top of the part cannot take the drag
        const int b = parseButton( aStep, error );
        button( aTarget, INPUT::KIND::BUTTON_DOWN, b, mods, KOPENAPI_POINTER_EVENT::DOWN );

        if( std::string bad = pointerTo( aTarget, to, mods, std::max( ms, 1 ) ); !bad.empty() )
        {
            button( aTarget, INPUT::KIND::BUTTON_UP, b, mods, KOPENAPI_POINTER_EVENT::UP );
            return bad;
        }

        button( aTarget, INPUT::KIND::BUTTON_UP, b, mods, KOPENAPI_POINTER_EVENT::UP );
        return {};
    }

    for( const char* verb : { "move", "click", "dclick", "down", "up" } )
    {
        if( !aStep.contains( verb ) )
            continue;

        VECTOR2D at;

        if( std::string bad = pointMm( aTarget, aStep[verb], at ); !bad.empty() )
            return bad;

        const std::string v = verb;
        const int         b = v == "move" ? 0 : parseButton( aStep, error );

        if( !error.empty() )
            return error;

        // the pointer arrives first, as a real one does before its button changes
        if( std::string bad = pointerTo( aTarget, at, mods, ms ); !bad.empty() )
            return bad;

        if( v == "down" || v == "click" || v == "dclick" )
            button( aTarget, INPUT::KIND::BUTTON_DOWN, b, mods, KOPENAPI_POINTER_EVENT::DOWN );

        if( v == "up" || v == "click" || v == "dclick" )
            button( aTarget, INPUT::KIND::BUTTON_UP, b, mods, KOPENAPI_POINTER_EVENT::UP );

        if( v == "dclick" )   // the toolkit's order: down, up, double click (held), up
        {
            button( aTarget, INPUT::KIND::BUTTON_DCLICK, b, mods, KOPENAPI_POINTER_EVENT::DCLICK );
            button( aTarget, INPUT::KIND::BUTTON_UP, b, mods, KOPENAPI_POINTER_EVENT::UP );
        }

        return {};
    }

    if( aStep.contains( "key" ) )
    {
        const int key = aStep["key"].is_string() ? parseKey( aStep["key"].get<std::string>(), error ) : 0;

        if( !error.empty() || !key )
            return error.empty() ? "key is a name" : error;

        INPUT input;
        input.kind = INPUT::KIND::KEY;
        input.keyCode = key;
        input.modifiers = mods;
        input.heldButtons = g_heldButtons;
        aTarget.dispatcher->DispatchInput( input );

        wxString shown = wxString::FromUTF8( aStep["key"].get<std::string>() );

        for( auto [bit, name] : { std::pair{ MD_CTRL, "Ctrl+" }, std::pair{ MD_ALT, "Alt+" }, std::pair{ MD_SHIFT, "Shift+" } } )
        {
            if( mods & bit )
                shown = wxString( name ) + shown;
        }

        KopenapiAnnotatePointer( aTarget.frame, VECTOR2D(), KOPENAPI_POINTER_EVENT::KEY, shown );
        pump();
        return {};
    }

    if( aStep.contains( "wait_ms" ) )
    {
        const auto until = std::chrono::steady_clock::now()
                           + std::chrono::milliseconds( std::clamp( aStep.value( "wait_ms", 0 ), 0, 10000 ) );

        while( std::chrono::steady_clock::now() < until )
        {
            pump();
            std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
        }

        return {};
    }

    return "a step is one of move, click, dclick, down, up, select, drag, key, wait_ms";
}
} // namespace


KOPENAPI_RESULT KopenapiEditorInput( EDA_DRAW_FRAME* aFrame, const EDA_IU_SCALE& aScale,
                                     const nlohmann::json& aArgs, const KOPENAPI_EDITOR_HOST& aHost )
{
    TARGET target;

    if( std::string bad = checkTarget( aFrame, aScale, aHost, target ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 409, bad );

    const nlohmann::json steps = aArgs.value( "steps", nlohmann::json::array() );
    int                  done = 0;

    for( const nlohmann::json& step : steps )
    {
        if( std::string bad = runStep( target, step ); !bad.empty() )
        {
            releasePointerIfIdle();
            return KOPENAPI_RESULT::Error( 400, "step " + std::to_string( done ) + ": " + bad );
        }

        ++done;
    }

    if( aArgs.value( "release", false ) && g_heldButtons )
    {
        // let go of every held button where the pointer is
        const int held = g_heldButtons;

        for( int bit : { BUT_LEFT, BUT_RIGHT, BUT_MIDDLE } )
        {
            if( held & bit )
            {
                g_heldButtons &= ~bit;
                dispatch( target, INPUT::KIND::BUTTON_UP, bit, 0 );
            }
        }
    }

    releasePointerIfIdle();

    nlohmann::json out = state( target );
    out["steps"] = done;
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT KopenapiEditorAction( EDA_DRAW_FRAME* aFrame, const EDA_IU_SCALE& aScale,
                                      const nlohmann::json& aArgs, const KOPENAPI_EDITOR_HOST& aHost )
{
    TARGET target;

    if( std::string bad = checkTarget( aFrame, aScale, aHost, target ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 409, bad );

    ACTION_MANAGER* actions = aFrame->GetToolManager()->GetActionManager();

    if( aArgs.contains( "list" ) )
    {
        const wxString pattern = wxString::FromUTF8( aArgs.value( "list", std::string( "*" ) ) );
        nlohmann::json items = nlohmann::json::array();

        for( const auto& [name, action] : actions->GetActions() )
        {
            const wxString label = action->GetFriendlyName();

            if( !wxMatchWild( pattern, wxString::FromUTF8( name ), false ) && !wxMatchWild( pattern, label, false ) )
                continue;

            const int hotkey = actions->GetHotKey( *action );
            items.push_back( { { "name", name },
                               { "label", label.ToStdString( wxConvUTF8 ) },
                               { "hotkey", hotkey ? KeyNameFromKeyCode( hotkey ).ToStdString( wxConvUTF8 ) : "" } } );

            if( items.size() >= 300 )
                break;
        }

        return KOPENAPI_RESULT::Ok( { { "actions", items } } );
    }

    const std::string name = aArgs.value( "action", std::string() );
    TOOL_ACTION*      action = actions->FindAction( name );

    if( !action )
        return KOPENAPI_RESULT::Error( 404, "no action '" + name + "' in this editor (list: \"*router*\" finds names)" );

    if( aArgs.contains( "at" ) )
    {
        VECTOR2D at;
        std::string bad = pointMm( target, aArgs["at"], at );

        if( bad.empty() )
            bad = pointerTo( target, at, 0, 0 );

        if( !bad.empty() )
        {
            releasePointerIfIdle();
            return KOPENAPI_RESULT::Error( 400, bad );
        }
    }

    const bool ran = aFrame->GetToolManager()->RunAction( *action );
    pump();
    releasePointerIfIdle();

    nlohmann::json out = state( target );
    out["ran"] = ran;
    return KOPENAPI_RESULT::Ok( out );
}


const nlohmann::json& KopenapiEditorInputSchema()
{
    static const nlohmann::json schema = R"json({"type":"object","properties":{
        "steps":{"type":"array","items":{"type":"object","properties":{
            "move":{"description":"[x, y] mm or an item (reference, REF.NUMBER pad / pin, uuid): the pointer moves there"},
            "click":{"description":"[x, y] mm or an item: press and release there"},
            "dclick":{"description":"[x, y] mm or an item: double click"},
            "down":{"description":"[x, y] mm or an item: press (a drag: down, moves, up)"},
            "up":{"description":"[x, y] mm or an item: release"},
            "select":{"type":"string","description":"intent: select exactly this item (even under a track or another item), pointer to it"},
            "drag":{"type":"string","description":"intent: select this item and drag it (to / by); items on top of it cannot take the drag"},
            "to":{"description":"drag target: [x, y] mm or an item"},
            "by":{"type":"array","items":{"type":"number"},"description":"drag offset [dx, dy] mm"},
            "ms":{"type":"integer","description":"glide the pointer there over this long (eased; visible in recordings)"},
            "button":{"type":"string","enum":["left","right","middle"],"default":"left"},
            "key":{"type":"string","description":"one character (hotkey letters as typed) or Escape, Enter, Tab, Space, Delete, Backspace, Insert, Home, End, PageUp, PageDown, Up, Down, Left, Right, F1..F12"},
            "mods":{"type":"array","items":{"type":"string","enum":["shift","ctrl","cmd","alt"]}},
            "wait_ms":{"type":"integer","description":"let timers and repaints run (max 10000)"}}}},
        "release":{"type":"boolean","default":false,"description":"release buttons still held after the steps"}},
        "required":["steps"]})json"_json;
    return schema;
}


const nlohmann::json& KopenapiEditorActionSchema()
{
    static const nlohmann::json schema = R"json({"type":"object","properties":{
        "action":{"type":"string","description":"tool action name, e.g. pcbnew.InteractiveRouter.SingleTrack"},
        "at":{"type":"array","items":{"type":"number"},"description":"[x, y] mm: the pointer is put there first"},
        "list":{"type":"string","description":"instead of running: actions whose name or label match this glob (name, label, hotkey)"}}})json"_json;
    return schema;
}
