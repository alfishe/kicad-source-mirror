/// @file kopenapi_pcb.h
/// @brief kicadopenapi PCB access (compiled into the pcbnew kiface).
///
/// One board per process.  GUI hosts reach the board through the PCB editor frame, headless
/// hosts through a HEADLESS_PCB_CONTEXT owned here; both behind PCB_CONTEXT, so every pcb_*
/// method has a single code path.
#ifndef KOPENAPI_PCB_H
#define KOPENAPI_PCB_H

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>
#include <string>

#include <math/box2.h>

#include <kicadopenapi_registry.h>

class FOOTPRINT;
class FP_3DMODEL;
class PCB_CONTEXT;
class PROJECT;


/// @brief The open board's context, or nullptr when no board is open
std::shared_ptr<PCB_CONTEXT> KopenapiPcbContext( KOPENAPI_CONTEXT& aCtx );

/// @brief 409 "no board open" result for methods that need one
KOPENAPI_RESULT KopenapiNoBoard();

/// @brief A frameless board's page (User size) fitted to its outline plus 10 mm: plots and renders
/// draw the page
void KopenapiFitPage( BOARD* aBoard, const BOX2I& aOutline );

class PCB_BASE_FRAME;

/// @brief Refresh the board's open 3D viewer after an edit: the current picture stays until the
/// new scene is complete (no flashing); nothing when no viewer is open
void KopenapiRefresh3D( PCB_BASE_FRAME* aFrame );

/// @brief An edge connector's panel line for footprint cards.
/// @param aMethod auto (marker, else inferred from the body), marker, infer
/// @return line ends, facing (left/right/up/down or degrees), depth behind the line, overhang
/// past it, source, confidence; null when no edge is known
/// @brief A 3D model's file on disk: variables expanded, relative to the project, .wrl -> .step when
/// present, embedded models (kicad-embed://) unpacked into the user cache.
/// @return the path (may not exist), empty for an embedded file the footprint does not carry
std::filesystem::path KopenapiModelFile( const FOOTPRINT* aFootprint, const FP_3DMODEL& aModel, PROJECT* aProject );

class BOARD;

/// @brief The assembled board's box in mm (x, y on the board, z up from the top surface): outline,
/// thickness and every shown 3D model; for framing 3D views
bool KopenapiAssemblyBox( BOARD* aBoard, double aLo[3], double aHi[3] );

/// @brief Re-applies the project's THT lead trim rule (when enabled) after footprints or their
/// models changed.
/// @return what was trimmed, or null when the rule is off
nlohmann::json KopenapiLeadTrimRefresh( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext );

nlohmann::json KopenapiPanelEdgeJson( FOOTPRINT* aFootprint, const std::string& aMethod = "auto" );

class BOARD;
class BOARD_ITEM;

/// @brief A board item by uuid, a footprint by reference, a pad as REF.NUMBER; nullptr if none
BOARD_ITEM* KopenapiFindBoardItem( BOARD* aBoard, const std::string& aRef );

class PCB_TRACK;

/// @brief The tracks / vias a request names: uuids, or the ones passing every filter given (net / nets,
/// between two pads, region + region_mode, layer, kinds).  aNeedSelector: no filter at all is an
/// error.  aError is set when the request is wrong; aMissing gets unknown uuids.
std::vector<PCB_TRACK*> KopenapiSelectTracks( BOARD* aBoard, const nlohmann::json& aArgs, bool aNeedSelector,
                                              nlohmann::json& aMissing, std::optional<KOPENAPI_RESULT>& aError );

/// @brief The item's extent as a user sees it (a footprint without its texts)
BOX2I KopenapiBoardItemBox( const BOARD_ITEM* aItem );

#endif
