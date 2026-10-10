/// @file methods_pcb_view_style.cpp
/// @brief kicadopenapi board editor look for filming: pcb_view_style (opacities, layers shown),
/// temporary by default and restored on reset or when the editor closes.
#include "kopenapi_pcb.h"

#include <board.h>
#include <frame_type.h>
#include <kicadopenapi_registry.h>
#include <kiway.h>
#include <lset.h>
#include <pcb_display_options.h>
#include <pcb_edit_frame.h>
#include <widgets/appearance_controls.h>

#include <optional>


namespace
{

/// @brief The editor's look before the first temporary change
struct SAVED_LOOK
{
    PCB_DISPLAY_OPTIONS options;
    LSET                visible;
};

std::optional<SAVED_LOOK> s_saved;
PCB_EDIT_FRAME*           s_boundFrame = nullptr;


void setLayerVisible( PCB_EDIT_FRAME* aFrame, PCB_LAYER_ID aLayer, bool aVisible )
{
    if( APPEARANCE_CONTROLS* panel = aFrame->GetAppearancePanel() )
        panel->SetLayerVisible( aLayer, aVisible );
}


void restore( PCB_EDIT_FRAME* aFrame )
{
    if( !s_saved )
        return;

    aFrame->SetDisplayOptions( s_saved->options );

    for( PCB_LAYER_ID layer : aFrame->GetBoard()->GetEnabledLayers().Seq() )
        setLayerVisible( aFrame, layer, s_saved->visible.Contains( layer ) );

    s_saved.reset();
    aFrame->GetCanvas()->Refresh();
}

} // namespace


static KOPENAPI_RESULT h_pcb_view_style( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    auto* frame = aCtx.kiway ? dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;

    if( !frame || !frame->IsShown() || !frame->GetBoard() )
        return KOPENAPI_RESULT::Error( 409, "no board editor window open" );

    BOARD* board = frame->GetBoard();

    if( aArgs.value( "reset", false ) )
    {
        const bool had = s_saved.has_value();
        restore( frame );
        return KOPENAPI_RESULT::Ok( { { "restored", had } } );
    }

    const bool persist = aArgs.value( "persist", false );

    // temporary: remember the user's look once, put it back on reset or when the editor closes
    // (before KiCad saves its settings)
    if( !persist && !s_saved )
    {
        s_saved = SAVED_LOOK{ frame->GetDisplayOptions(), board->GetVisibleLayers() };

        if( s_boundFrame != frame )
        {
            s_boundFrame = frame;
            frame->Bind( wxEVT_CLOSE_WINDOW,
                         [frame]( wxCloseEvent& aEvent )
                         {
                             restore( frame );
                             aEvent.Skip();
                         } );
        }
    }

    PCB_DISPLAY_OPTIONS opts = frame->GetDisplayOptions();
    auto                opacity = [&]( const char* aKey, double& aField )
    {
        if( aArgs.contains( aKey ) )
            aField = std::clamp( aArgs[aKey].get<double>(), 0.0, 1.0 );
    };

    opacity( "zone_opacity", opts.m_ZoneOpacity );
    opacity( "track_opacity", opts.m_TrackOpacity );
    opacity( "via_opacity", opts.m_ViaOpacity );
    opacity( "pad_opacity", opts.m_PadOpacity );
    frame->SetDisplayOptions( opts );

    nlohmann::json unknown = nlohmann::json::array();

    for( const char* key : { "hide_layers", "show_layers" } )
    {
        for( const nlohmann::json& name : aArgs.value( key, nlohmann::json::array() ) )
        {
            const int id = board->GetLayerID( wxString::FromUTF8( name.get<std::string>() ) );

            if( id < 0 || !board->IsLayerEnabled( (PCB_LAYER_ID) id ) )
                unknown.push_back( name );
            else
                setLayerVisible( frame, (PCB_LAYER_ID) id, std::string( key ) == "show_layers" );
        }
    }

    frame->GetCanvas()->Refresh();

    nlohmann::json hidden = nlohmann::json::array();

    for( PCB_LAYER_ID layer : board->GetEnabledLayers().Seq() )
    {
        if( !board->GetVisibleLayers().Contains( layer ) )
            hidden.push_back( board->GetLayerName( layer ).ToStdString( wxConvUTF8 ) );
    }

    return KOPENAPI_RESULT::Ok( { { "zone_opacity", opts.m_ZoneOpacity },
                                  { "track_opacity", opts.m_TrackOpacity },
                                  { "via_opacity", opts.m_ViaOpacity },
                                  { "pad_opacity", opts.m_PadOpacity },
                                  { "hidden_layers", hidden },
                                  { "unknown_layers", unknown },
                                  { "temporary", s_saved.has_value() } } );
}


KOPENAPI_REGISTER( "pcb_view_style",
                   "Board editor look for filming / screenshots: zone, track, via, pad opacity (0..1), "
                   "layers to hide / show (e.g. courtyards, fab); temporary by default: reset: true or "
                   "closing the editor brings the user's look back; persist: true keeps it; GUI only",
                   R"json({"type":"object","properties":{
                        "zone_opacity":{"type":"number","minimum":0,"maximum":1},
                        "track_opacity":{"type":"number","minimum":0,"maximum":1},
                        "via_opacity":{"type":"number","minimum":0,"maximum":1},
                        "pad_opacity":{"type":"number","minimum":0,"maximum":1},
                        "hide_layers":{"type":"array","items":{"type":"string"},"description":"e.g. F.Courtyard, B.Courtyard, F.Fab, B.Fab"},
                        "show_layers":{"type":"array","items":{"type":"string"}},
                        "reset":{"type":"boolean","default":false,"description":"back to the look before the first temporary change"},
                        "persist":{"type":"boolean","default":false}}})json"_json,
                   true, h_pcb_view_style );
