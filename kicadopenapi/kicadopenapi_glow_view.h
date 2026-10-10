/// @file kicadopenapi_glow_view.h
/// @brief kicadopenapi GUI glow over an editor's view: the halo drawing and brightening an editor kiface
/// plugs into the shared KOPENAPI_GLOW_MANAGER (kicadopenapi_glow.h).  Header-only: each editor
/// instantiates KOPENAPI_VIEW_GLOW_TARGET with its traits.
#ifndef KICADOPENAPI_GLOW_VIEW_H
#define KICADOPENAPI_GLOW_VIEW_H

#include <kicadopenapi_glow.h>
#include <eda_draw_frame.h>
#include <class_draw_panel_gal.h>
#include <gal/color4d.h>
#include <kiid.h>
#include <view/view.h>
#include <view/view_overlay.h>

#include <wx/event.h>
#include <wx/timer.h>
#include <wx/utils.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <vector>

class KIWAY;


/// @brief Vivid blue: not among KiCad's default schematic colours; KOPENAPI_GLOW_COLOR=#RRGGBB overrides
inline KIGFX::COLOR4D KopenapiGlowColour( double aAlpha )
{
    static const KIGFX::COLOR4D base = []()
    {
        wxString hex;

        if( wxGetEnv( wxS( "KOPENAPI_GLOW_COLOR" ), &hex ) && hex.length() == 7 && hex[0] == '#' )
        {
            unsigned long rgb = 0;

            if( hex.Mid( 1 ).ToULong( &rgb, 16 ) )
                return KIGFX::COLOR4D( ( ( rgb >> 16 ) & 0xFF ) / 255.0, ( ( rgb >> 8 ) & 0xFF ) / 255.0, ( rgb & 0xFF ) / 255.0, 1.0 );
        }

        return KIGFX::COLOR4D( 0.10, 0.45, 1.0, 1.0 );
    }();

    return base.WithAlpha( aAlpha );
}


/// @brief An editor's glow target over its view, from its traits:
///   using FRAME = the editor frame type
///   static FRAME*    Frame( KIWAY* )                      the open editor, or nullptr (headless)
///   static EDA_ITEM* Resolve( FRAME*, const KIID& )       item by uuid, or nullptr
///   static void      Brighten( FRAME*, EDA_ITEM*, bool )  set / clear the item's brightening
///   static BOX2I     Box( EDA_ITEM* )                     extent for the halo
///   static int       Mm()                                 internal units per mm
///   static void      ItemColour( FRAME*, bool aGlow, std::optional<KIGFX::COLOR4D>& aSaved )
template <typename TRAITS>
class KOPENAPI_VIEW_GLOW_TARGET : public KOPENAPI_GLOW_TARGET
{
public:
    using FRAME = typename TRAITS::FRAME;

    /// @brief The editor's target (one per kiface); remembers the KIWAY to find its frame later
    static KOPENAPI_VIEW_GLOW_TARGET* Get( KIWAY* aKiway )
    {
        static KOPENAPI_VIEW_GLOW_TARGET* target = new KOPENAPI_VIEW_GLOW_TARGET();

        if( aKiway )
            target->m_kiway = aKiway;

        return target;
    }

    bool Alive() override { return TRAITS::Frame( m_kiway ) != nullptr; }

    bool Brighten( const KIID& aId, bool aOn ) override
    {
        FRAME*    frame = TRAITS::Frame( m_kiway );
        EDA_ITEM* item = frame ? TRAITS::Resolve( frame, aId ) : nullptr;

        if( item )
            TRAITS::Brighten( frame, item, aOn );

        return item != nullptr;
    }

    void Begin() override
    {
        if( FRAME* frame = TRAITS::Frame( m_kiway ) )
            TRAITS::ItemColour( frame, true, m_savedColour );
    }

    void End() override
    {
        if( FRAME* frame = TRAITS::Frame( m_kiway ) )
            TRAITS::ItemColour( frame, false, m_savedColour );

        m_overlay.reset();
        m_overlayView = nullptr;
    }

    /// @brief Nested rounded outlines around each glowing item, fading outwards and with age
    void Halo( const std::vector<std::pair<KIID, double>>& aItems ) override
    {
        FRAME* frame = TRAITS::Frame( m_kiway );

        if( !frame )
            return;

        KIGFX::VIEW* view = frame->GetCanvas()->GetView();

        if( !m_overlay || m_overlayView != view )
        {
            m_overlay = view->MakeOverlay();
            m_overlayView = view;
        }

        m_overlay->Clear();

        const int mm = TRAITS::Mm();

        for( const auto& [id, strength] : aItems )
        {
            EDA_ITEM* item = strength > 0 ? TRAITS::Resolve( frame, id ) : nullptr;

            if( !item )
                continue;

            const BOX2I box = TRAITS::Box( item );

            m_overlay->SetIsFill( true );
            m_overlay->SetIsStroke( false );
            m_overlay->SetFillColor( KopenapiGlowColour( 0.10 * strength ) );
            m_overlay->Rectangle( VECTOR2D( box.GetLeft() - mm, box.GetTop() - mm ),
                                  VECTOR2D( box.GetRight() + mm, box.GetBottom() + mm ) );

            m_overlay->SetIsFill( false );
            m_overlay->SetIsStroke( true );

            for( int ring = 0; ring < 5; ++ring )
            {
                const int margin = mm + ring * mm * 6 / 10;
                m_overlay->SetLineWidth( mm * 0.6 );
                m_overlay->SetStrokeColor( KopenapiGlowColour( ( 0.85 - ring * 0.17 ) * strength ) );
                m_overlay->Rectangle( VECTOR2D( box.GetLeft() - margin, box.GetTop() - margin ),
                                      VECTOR2D( box.GetRight() + margin, box.GetBottom() + margin ) );
            }
        }

        view->Update( m_overlay.get() );
        frame->GetCanvas()->Refresh();
    }

private:
    KIWAY*                               m_kiway = nullptr;
    std::shared_ptr<KIGFX::VIEW_OVERLAY> m_overlay;
    KIGFX::VIEW*                         m_overlayView = nullptr;
    std::optional<KIGFX::COLOR4D>        m_savedColour;
};


/// @brief Glow these items of the editor behind TRAITS (no-op headless); aStaggerMs: one after another
template <typename TRAITS>
void KopenapiGlow( KIWAY* aKiway, const std::vector<KIID>& aItems, int aStaggerMs = 0 )
{
    KOPENAPI_GLOW_MANAGER::Get().Add( KOPENAPI_VIEW_GLOW_TARGET<TRAITS>::Get( aKiway ), aItems, aStaggerMs );
}

#endif
