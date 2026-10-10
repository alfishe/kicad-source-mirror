/// @file kicadopenapi_gal_render.h
/// @brief Recordings: a schematic / board canvas (GAL) rendered at the video's size — the composed
/// frame scaled and letterboxed on the GPU, read back once. Header-only, used by the board and
/// schematic kifaces.
#ifndef KICADOPENAPI_GAL_RENDER_H
#define KICADOPENAPI_GAL_RENDER_H

#include <class_draw_panel_gal.h>

#include <wx/image.h>


/// @brief KOPENAPI_CANVAS_RENDER for GAL canvases (aSupersample: the window's own pixels already
/// are the source; it is not drawn larger)
inline bool KopenapiRenderGal( wxWindow* aWindow, int aWidth, int aHeight, int /*aSupersample*/, wxImage& aImage )
{
    auto* canvas = dynamic_cast<EDA_DRAW_PANEL_GAL*>( aWindow );

    if( !canvas || aWidth <= 0 || aHeight <= 0 )
        return false;

    aImage.Create( aWidth, aHeight, false );
    return canvas->RenderToImage( aImage.GetData(), aWidth, aHeight );
}

#endif
