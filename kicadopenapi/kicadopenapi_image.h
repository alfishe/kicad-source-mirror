/// @file kicadopenapi_image.h
/// @brief kicadopenapi image results: methods returning a picture (renders, window / canvas captures)
/// answer { width, height, mime_type, image_base64, ... }; the MCP layer turns that into image
/// content.
#ifndef KICADOPENAPI_IMAGE_H
#define KICADOPENAPI_IMAGE_H

#include <json_common.h>
#include <kicommon.h>

class wxImage;

/// @brief PNG-encode aImage (scaled down to aMaxWidth if wider; 0 = keep) into an image result
KICOMMON_API nlohmann::json KopenapiImageResult( wxImage aImage, int aMaxWidth = 0 );

/// @brief Crop to the pixels that differ from the corner (background) colour, plus a margin
KICOMMON_API wxImage KopenapiCropToContent( const wxImage& aImage, int aMargin );

#endif
