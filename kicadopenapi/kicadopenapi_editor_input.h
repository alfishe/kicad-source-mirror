/// @file kicadopenapi_editor_input.h
/// @brief Drives an editor's tools from the API the way a user does: actions run as TOOL_EVENTs,
/// pointer and keys enter TOOL_DISPATCHER::DispatchInput() (the same logic real input takes after
/// the platform layer), the pointer the tools query comes from KIPLATFORM::UI's injected pointer.
/// Nothing touches the host system: no OS events, no accessibility permissions. GUI, main thread.
/// Shared by the board and schematic editors (libcommon).
#ifndef KICADOPENAPI_EDITOR_INPUT_H
#define KICADOPENAPI_EDITOR_INPUT_H

#include <kicadopenapi_registry.h>

#include <functional>
#include <optional>
#include <string>

#include <math/box2.h>

class EDA_DRAW_FRAME;
struct EDA_IU_SCALE;


/// @brief An editor kiface's side of the input: its state, and what makes intents possible
struct KOPENAPI_EDITOR_HOST
{
    /// @brief Editor-specific state added to every answer (e.g. the board editor's routing state)
    std::function<void( nlohmann::json& aOut )> state;

    /// @brief An item (reference, REF.NUMBER pad / pin, uuid): its extent in internal units
    std::function<std::optional<BOX2I>( const std::string& aRef )> resolve;

    /// @brief Selects exactly this item, as the user's selection clarification would
    std::function<bool( const std::string& aRef )> select;
};


/**
 * @brief Runs a tool action by name, optionally with the pointer at a document position first;
 * or lists actions matching a pattern.
 * @param aArgs { action, at: [x, y] mm } or { list: "glob" }
 */
KOPENAPI_RESULT KopenapiEditorAction( EDA_DRAW_FRAME* aFrame, const EDA_IU_SCALE& aScale,
                                      const nlohmann::json& aArgs, const KOPENAPI_EDITOR_HOST& aHost );

/**
 * @brief Feeds pointer / key steps to the editor's tools.
 * @param aArgs { steps: [ {move|click|dclick|down|up: [x, y] or a reference, button, mods, ms},
 *               {select: ref}, {drag: ref, to: [x, y] | by: [dx, dy], ms}, {key, mods}, {wait_ms} ],
 *               release }
 */
KOPENAPI_RESULT KopenapiEditorInput( EDA_DRAW_FRAME* aFrame, const EDA_IU_SCALE& aScale,
                                     const nlohmann::json& aArgs, const KOPENAPI_EDITOR_HOST& aHost );

/// @brief JSON schema shared by the pcb_ / sch_input methods
const nlohmann::json& KopenapiEditorInputSchema();

/// @brief JSON schema shared by the pcb_ / sch_action methods
const nlohmann::json& KopenapiEditorActionSchema();

#endif
