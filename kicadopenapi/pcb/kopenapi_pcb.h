/// @file kopenapi_pcb.h
/// @brief kicadopenapi PCB access (compiled into the pcbnew kiface).
///
/// One board per process.  GUI hosts reach the board through the PCB editor frame, headless
/// hosts through a HEADLESS_PCB_CONTEXT owned here; both behind PCB_CONTEXT, so every pcb_*
/// method has a single code path.
#ifndef KOPENAPI_PCB_H
#define KOPENAPI_PCB_H

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <vector>
#include <string>

#include <math/box2.h>
#include <wx/string.h>

#include <kicadopenapi_registry.h>

class BOARD;
class FOOTPRINT;
class PAD;
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

class ZONE;

/// @brief A zone's fill pieces per layer when a layer has more than one: box, area, the net's pads,
/// vias and tracks on each (an island with no via is cut off)
nlohmann::json KopenapiZoneIslands( BOARD* aBoard, ZONE* aZone );

class PCB_BASE_FRAME;

/// @brief Refresh the board's open 3D viewer after an edit: the current picture stays until the
/// new scene is complete (no flashing); nothing when no viewer is open
void KopenapiRefresh3D( PCB_BASE_FRAME* aFrame );

class EDA_ITEM;

/// @brief Set / clear an item's brightening in the board view (a footprint with its children);
/// a recolour only, the cached geometry stays
void KopenapiPcbBrighten( PCB_BASE_FRAME* aFrame, EDA_ITEM* aItem, bool aOn );

class FOOTPRINT_LIBRARY_ADAPTER;

/// @brief The project's footprint libraries, loaded (waits for a load in progress).  Footprints
/// read through the adapter are the files on disk now: LoadFootprint() re-reads a footprint
/// whose file changed (one file query per load).
FOOTPRINT_LIBRARY_ADAPTER* KopenapiFootprintLibraries( PROJECT* aProject );

/// @brief Findings on a placed footprint's 3D models, each {kind, severity, model (1-based),
/// file, message}: model_missing (not resolved through KiCad's 3D search paths, or an embedded
/// file neither the footprint nor the board carries), model_meta_stale (<model>.meta.json
/// model_sha256 differs; hashed only when the model is newer than its metadata),
/// model_lead_count (the metadata's lead solids exceed the numbered pads), no_model (info)
nlohmann::json KopenapiModelFindings( BOARD* aBoard, const FOOTPRINT* aFootprint );

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

/// @brief Height of every footprint with a shown 3D model, in mm from the board surface on its own
/// side (model boxes, cached geometry); footprints without a readable model are absent
std::map<const FOOTPRINT*, double> KopenapiFootprintHeights( BOARD* aBoard );

nlohmann::json KopenapiPanelEdgeJson( FOOTPRINT* aFootprint, const std::string& aMethod = "auto" );

class BOARD;
class BOARD_ITEM;

/// @brief A board item by uuid (or a unique uuid prefix), a footprint by reference, a pad as
/// REF.NUMBER; nullptr if none
BOARD_ITEM* KopenapiFindBoardItem( BOARD* aBoard, const std::string& aRef );

/// @brief The board item (footprint children and zones inside footprints included) whose uuid is
/// aId or starts with it (at least 4 characters, case-insensitive).  nullptr with aError set when
/// none matches, or when a prefix fits several ("ambiguous: ...", the candidates listed).
BOARD_ITEM* KopenapiResolveUuid( BOARD* aBoard, const std::string& aId, std::string& aError );

/// @brief KopenapiResolveUuid over the given items only (the zones of a zone method, the tracks of
/// a track method)
BOARD_ITEM* KopenapiMatchUuid( const std::vector<BOARD_ITEM*>& aItems, const std::string& aId, std::string& aError );

class SHAPE_POLY_SET;

/// @brief Where a net's pours put its copper on one copper layer, priorities resolved as the zone
/// filler does: the outlines of the net's zones (board and footprint ones), each less the outlines
/// of other nets' zones on that layer that win over it (ZONE::HigherPriority: priority, then uuid)
/// and of keep-outs barring pours there, clipped to the board.  Outlines, not fills: no
/// clearances around other copper.  Empty when the net has no zone there.
void KopenapiNetDomain( BOARD* aBoard, int aNetCode, int aLayer, SHAPE_POLY_SET& aOut );

/// @brief Open connections as KiCad's ratsnest counts them, the one figure every method reports:
/// connections = ratsnest edges (airwires) over every net after a connectivity rebuild, with the
/// zones as they are filled now; nets = nets with one or more; zone_fills: "none" (no copper zone),
/// "filled", "stale" (a zone changed since its fill) or "unfilled" (a copper zone without fill:
/// what it would join counts as open)
struct KOPENAPI_UNROUTED
{
    int         connections = 0;
    int         nets = 0;
    std::string zoneFills;
};

KOPENAPI_UNROUTED KopenapiUnrouted( BOARD* aBoard, bool aRebuild = true );

/// @brief KopenapiUnrouted as answer fields: unrouted_connections, unrouted_nets, zone_fills and
/// unrouted_left (the same number as unrouted_connections, kept for older callers)
nlohmann::json KopenapiUnroutedJson( BOARD* aBoard, bool aRebuild = true );

/// @brief Fill state of the board's copper zones as in KOPENAPI_UNROUTED::zoneFills; aStale gets
/// the zones (name or net, uuid) that are not filled or need a refill
std::string KopenapiZoneFillState( BOARD* aBoard, nlohmann::json* aStale = nullptr );

class PCB_TRACK;

/// @brief The tracks / vias a request names: uuids, or the ones passing every filter given (net / nets,
/// between two pads, region + region_mode, layer, kinds).  aNeedSelector: no filter at all is an
/// error.  aError is set when the request is wrong; aMissing gets unknown uuids.
std::vector<PCB_TRACK*> KopenapiSelectTracks( BOARD* aBoard, const nlohmann::json& aArgs, bool aNeedSelector,
                                              nlohmann::json& aMissing, std::optional<KOPENAPI_RESULT>& aError );

/// @brief The item's extent as a user sees it (a footprint without its texts)
BOX2I KopenapiBoardItemBox( const BOARD_ITEM* aItem );

class NETINFO_ITEM;

/**
 * @brief Board net by name, as the answers spell it (unescaped, e.g. "Net-(U1-X1/CLK)") or in KiCad's stored form
 */
NETINFO_ITEM* KopenapiFindNet( BOARD* aBoard, const std::string& aName );

/// @brief Board net names close to aName (KopenapiDidYouMean over the unescaped names)
std::vector<std::string> KopenapiNetSuggestions( BOARD* aBoard, const std::string& aName );

/// @brief 404 for a net name not on the board: "<aWhat>no net '<name>' on the board", with
/// error.did_you_mean (close names, e.g. the same name under its sheet prefix) and error.net
KOPENAPI_RESULT KopenapiNoSuchNet( BOARD* aBoard, const std::string& aName, const std::string& aWhat = std::string() );

/// @brief Two pads of a part that count for its pitch: on a copper layer both have, apart (the two
/// sides of an edge connector's finger, or a pad stacked on another, are not neighbours)
bool KopenapiPadNeighbours( const PAD* aA, const PAD* aB );

/**
 * @brief Validate a net's stored driver edge block ({family, tr_ps, tr_def, rout_ohm, swing_v, nm_v, driver})
 * @return empty when valid, else what is wrong
 */
std::string KopenapiEdgeCheck( const nlohmann::json& aEdge );

/**
 * @brief Validate a net's stored termination block ({kind, dc_balanced})
 * @return empty when valid, else what is wrong
 */
std::string KopenapiTerminationCheck( const nlohmann::json& aTermination );

#endif
