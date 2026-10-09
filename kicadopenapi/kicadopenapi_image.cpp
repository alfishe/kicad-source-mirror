#include "kicadopenapi_image.h"

#include <wx/base64.h>
#include <wx/image.h>
#include <wx/mstream.h>

#include <algorithm>
#include <cstdlib>
#include <vector>


nlohmann::json KopenapiImageResult( wxImage aImage, int aMaxWidth )
{
    if( !aImage.IsOk() )
        return { { "error", "no image" } };

    if( aMaxWidth > 0 && aImage.GetWidth() > aMaxWidth )
        aImage.Rescale( aMaxWidth, aImage.GetHeight() * aMaxWidth / aImage.GetWidth(), wxIMAGE_QUALITY_HIGH );

    if( !wxImage::FindHandler( wxBITMAP_TYPE_PNG ) )
        wxImage::AddHandler( new wxPNGHandler );

    wxMemoryOutputStream png;
    aImage.SaveFile( png, wxBITMAP_TYPE_PNG );

    std::vector<char> bytes( png.GetSize() );
    png.CopyTo( bytes.data(), bytes.size() );

    return { { "width", aImage.GetWidth() },
             { "height", aImage.GetHeight() },
             { "mime_type", "image/png" },
             { "image_base64", wxBase64Encode( bytes.data(), bytes.size() ).ToStdString() } };
}


wxImage KopenapiCropToContent( const wxImage& aImage, int aMargin )
{
    if( !aImage.IsOk() )
        return aImage;

    const int            w = aImage.GetWidth(), h = aImage.GetHeight();
    const unsigned char* data = aImage.GetData();
    const unsigned char  r = data[0], g = data[1], b = data[2];
    int                  left = w, top = h, right = -1, bottom = -1;

    for( int y = 0; y < h; ++y )
    {
        const unsigned char* row = data + 3 * (size_t) y * w;

        for( int x = 0; x < w; ++x )
        {
            const unsigned char* px = row + 3 * x;

            if( std::abs( px[0] - r ) + std::abs( px[1] - g ) + std::abs( px[2] - b ) > 24 )
            {
                left = std::min( left, x );
                right = std::max( right, x );
                top = std::min( top, y );
                bottom = std::max( bottom, y );
            }
        }
    }

    if( right < 0 )
        return aImage;

    left = std::max( 0, left - aMargin );
    top = std::max( 0, top - aMargin );
    right = std::min( w - 1, right + aMargin );
    bottom = std::min( h - 1, bottom + aMargin );

    return aImage.GetSubImage( wxRect( left, top, right - left + 1, bottom - top + 1 ) );
}
