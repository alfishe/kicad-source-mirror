/// @file kicadopenapi_annotations.h
/// @brief The annotation layer of an editor canvas, for teaching and demonstrations: highlights,
/// arrows, labels, regions / spotlights, numbered steps, key hints, and the pointer the
/// automation drives (cursor, trail, click ripples). A view overlay outside the model and the
/// tool system: tools, hit tests, selection, DRC, plots and files never see it. Annotations
/// anchor to model items by uuid or reference and are re-resolved on every redraw, so they follow
/// moves, resizes and edits (also mid-drag); a deleted item makes its annotations orphaned.
/// Model change notifications arrive through KOPENAPI_MODEL_EVENTS. GUI, main thread.
#ifndef KICADOPENAPI_ANNOTATIONS_H
#define KICADOPENAPI_ANNOTATIONS_H

#include <kicadopenapi_registry.h>

#include <math/box2.h>
#include <math/vector2d.h>
#include <wx/string.h>

#include <functional>
#include <optional>
#include <string>

class EDA_DRAW_FRAME;
struct EDA_IU_SCALE;


/// @brief An editor kiface's side of the layer
struct KOPENAPI_ANNOTATION_HOST
{
    std::string                                                domain;   ///< "pcb", "sch"
    const EDA_IU_SCALE*                                        scale = nullptr;
    EDA_DRAW_FRAME*                                            frame = nullptr;
    /// @brief Item by uuid or reference designator: its bounding box (internal units), or nothing
    std::function<std::optional<BOX2I>( const std::string& aRef )> resolve;
};


/// @brief pcb_annotate / sch_annotate: add, update, remove, clear, list
KOPENAPI_RESULT KopenapiAnnotate( const KOPENAPI_ANNOTATION_HOST& aHost, const nlohmann::json& aArgs );

const nlohmann::json& KopenapiAnnotateSchema();


enum class KOPENAPI_POINTER_EVENT
{
    MOVE,
    DOWN,
    UP,
    DCLICK,
    KEY
};

/// @brief Shows the injected pointer's action on the frame's canvas (cursor, trail, ripples,
/// pressed key); no-op when the layer is switched off (KOPENAPI_SHOW_POINTER=0)
void KopenapiAnnotatePointer( EDA_DRAW_FRAME* aFrame, const VECTOR2D& aWorld, KOPENAPI_POINTER_EVENT aEvent,
                              const wxString& aKey = wxEmptyString );

#endif
