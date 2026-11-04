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
#include <set>
#include <vector>
#include <functional>
#include <string>

#include <layer_ids.h>
#include <math/box2.h>
#include <wx/string.h>

#include <kicadopenapi_registry.h>

class BOARD;
class CONNECTIVITY_DATA;
class FOOTPRINT;
class PAD;
class FP_3DMODEL;
class PCB_CONTEXT;
class PCB_SHAPE;
class PROJECT;
class SHAPE_POLY_SET;


/// @brief A polygon set's contours in mm, each [[x, y], ...]: every polygon's outline, then its holes
nlohmann::json KopenapiContoursMm( const SHAPE_POLY_SET& aPoly );

/// @brief A graphic shape's geometry in mm: points_mm (line, rect: start, end; arc: start, mid,
/// end; bezier: start, two controls, end), circles centre_mm and radius_mm, polygons contours_mm
void KopenapiShapeGeometry( const PCB_SHAPE& aShape, nlohmann::json& aRow );

/// @brief The open board's context, or nullptr when no board is open
std::shared_ptr<PCB_CONTEXT> KopenapiPcbContext( KOPENAPI_CONTEXT& aCtx );

/// @brief 409 "no board open" result for methods that need one
KOPENAPI_RESULT KopenapiNoBoard();

class FOOTPRINT;

/// @brief A mounting hole, fiducial or a part without nets
bool KopenapiIsMechanicalPart( const FOOTPRINT* aFp );

/// @brief A connector by its reference or library (never a mechanical part)
bool KopenapiIsConnectorPart( const FOOTPRINT* aFp );

/// @brief An anchor of a block: an IC or connector, a part of more than aMaxPartPins numbered pads or a
/// connector, not mechanical
bool KopenapiIsAnchorPart( const FOOTPRINT* aFp, int aMaxPartPins = 3 );

/// @brief A frameless board's page (User size) fitted to its outline plus 10 mm: plots and renders
/// draw the page
void KopenapiFitPage( BOARD* aBoard, const BOX2I& aOutline );

class ZONE;
class GENERATOR_TOOL;
class TOOL_MANAGER;
class SHAPE_LINE_CHAIN;

/// @brief The polygon of points [[x, y], ...] or rect [x0, y0, x1, y1] in mm, closed; empty with
/// aError set
SHAPE_LINE_CHAIN KopenapiOutlineArg( const nlohmann::json& aArgs, std::string& aError );

/// @brief The editor's generator tool: a GUI editor has one, a headless board gets one (registered
/// only, its router built by Reset)
GENERATOR_TOOL* KopenapiGeneratorTool( TOOL_MANAGER* aManager );

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

/// @brief The board's custom design rules as the file on disk is now: the DRC engine is set up
/// again when the .kicad_dru changed (written, added, removed) since this board's rules were last
/// read here, as KiCad's DRC dialog re-reads it on every run.  Returns false with aError when the
/// changed file does not compile (the engine keeps the rules it had).
bool KopenapiDesignRulesFromDisk( BOARD* aBoard, std::string* aError = nullptr );

/// @brief Headless: the board's unsaved flag as it was before this scope (for work that is not an
/// edit but makes KiCad report net settings changed, as setting the DRC engine up does)
class KOPENAPI_KEEP_UNSAVED
{
public:
    KOPENAPI_KEEP_UNSAVED();
    ~KOPENAPI_KEEP_UNSAVED();

private:
    bool m_unsaved;
};

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
/// @param aOnly when given, only these footprints
std::map<const FOOTPRINT*, double> KopenapiFootprintHeights( BOARD* aBoard,
                                                             const std::set<const FOOTPRINT*>* aOnly = nullptr );

nlohmann::json KopenapiPanelEdgeJson( FOOTPRINT* aFootprint, const std::string& aMethod = "auto" );

class BOARD;
class BOARD_ITEM;

/// @brief The N net of a differential pair whose P net is aNet, as KiCad's DRC and router pair them
/// (DRC_ENGINE::MatchDpSuffix) among the names aExists accepts; empty when none (aNet is not a P side)
std::string KopenapiPairPartner( BOARD* aBoard, const std::string& aNet,
                                 const std::function<bool( const std::string& )>& aExists );

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

class PADSTACK;

/// @brief A pad's or via's unconnected layer mode as the API names it: keep (all layers), remove
/// (unconnected ones), keep_outer (removed except the outer / start and end layers), start_end_only
std::string KopenapiUnusedLayersMode( const PADSTACK& aPadstack );

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

/// @brief True when no net of aConn awaits a connectivity or ratsnest update
bool KopenapiConnectivityCurrent( CONNECTIVITY_DATA& aConn );

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

class KOPENAPI_LIST_CAP;

/// @brief Board design rules as pcb_rules_get answers them: copper layers, thickness, constraints,
/// net classes (member nets capped by aCap), patterns, custom rules
nlohmann::json KopenapiRulesJson( BOARD* aBoard, const KOPENAPI_LIST_CAP& aCap );

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

/// The room between a chip's rows of pins when it has pins on four sides and is no ball or land
/// array (QFP, QFN, LGA): from the inner edge of each row's pads; nullopt otherwise. A pin is in a
/// row when its centre is within aTolerance of that side's outermost centres
std::optional<BOX2I> KopenapiPadRingInside( const FOOTPRINT* aFp, int aTolerance );

/// @brief A track as KiCad's rules size it: the narrowest they allow (a track_width min, the board
/// minimum) and the width they ask (opt, else the net class's, else the board's current), in IU
struct KOPENAPI_RULE_TRACK
{
    int min;
    int opt;
};

/// @brief A track of aNetCode at aAt on aLayer as KiCad's rules size it (DRC_ENGINE::EvalRules on a
/// probe track)
KOPENAPI_RULE_TRACK KopenapiRuleTrack( BOARD* aBoard, int aNetCode, const VECTOR2I& aAt, PCB_LAYER_ID aLayer );

/// @brief A via as KiCad's rules size it, in IU: diameter and drill (the rule's opt, not below its
/// min; else the net class's; else the board's current) and the clearance it keeps to an item
struct KOPENAPI_RULE_VIA
{
    int dia;
    int drill;
    int clearance;
};

/// @brief A via of aNetCode at aAt as KiCad's rules size it (EvalRules VIA_DIAMETER, HOLE_SIZE on a
/// probe via) and the clearance it keeps to aOther (nullptr: the net's own), not below the board minimum
KOPENAPI_RULE_VIA KopenapiRuleVia( BOARD* aBoard, int aNetCode, const VECTOR2I& aAt, PCB_LAYER_ID aLayer,
                                   BOARD_ITEM* aOther = nullptr );

/// @brief The mm of track a via costs a route: aArgs' via_cost_mm when given (not below 0), else the
/// routing room a via of aNetCode takes as the rules size it (KopenapiRuleTrack, KopenapiRuleVia), as
/// track: on each signal layer of the stack-up, the disc a track's centre keeps out of (via / 2 +
/// clearance + track / 2) over the band one mm of track keeps other tracks' centres out of
/// (2 x (track + clearance))
double KopenapiViaCostMm( BOARD* aBoard, const nlohmann::json& aArgs, int aNetCode = 0 );

/// @brief A fine-pitch pad, by the rules' sizes for its net: no track as wide as the rules ask passes
/// between it and its nearest neighbour (KopenapiPadNeighbours) at the clearance they ask, or two of
/// its vias do not fit side by side at the pitch (via + clearance to that neighbour); false without one
bool KopenapiPadFinePitch( BOARD* aBoard, PAD* aPad );

/// @brief How far past its outer end a pad's way out reaches as the rules size it: a via of its net with
/// the clearance to the pads of other nets that room comes within that clearance of (KopenapiRuleVia;
/// the row neighbours, not a pin across the package), plus the offset of a second, staggered via line
/// where two vias do not fit side by side at the row's pitch
int KopenapiPadEscapeDepth( BOARD* aBoard, PAD* aPad );

/// @brief A pad whose way out runs beside a wall: an other-net pad of its footprint across its way out
/// (on the part's outline, outward by its pad centres) nearer than one way needs, the rules' narrowest
/// track of its net plus its clearance to that pad on both sides; an inner pad needs such a pad on both
/// sides along the axis to its nearest neighbour. False for either pad of a two-pad part
bool KopenapiPadWayBetween( BOARD* aBoard, PAD* aPad );

/// @brief A row of pins' 45 degree fan to a row of spots, pin k to spot k, both in order along the row: out of
/// the pad to a knee, 45 degrees to the spot's line, straight on to the spot. Pins going the same way turn
/// the farther out the nearer they are to the middle, by what keeps two legs a track and a clearance apart;
/// the spots lie a via's radius, a clearance and half a track past the farthest diagonal's end
struct KOPENAPI_FAN
{
    std::vector<int> knee;    ///< per pin: its knee past the pins' tips
    std::vector<int> shift;   ///< per pin: its spot less its own position along the row
    int64_t          depth;   ///< the spots past the pins' tips
};

/// @brief The fan of pins at aPinsAlong to spots at aSpotsAlong (positions along the row, in order) with a
/// track aWidth wide, aClearance and vias aViaDia across
KOPENAPI_FAN KopenapiFan( const std::vector<int64_t>& aPinsAlong, const std::vector<int64_t>& aSpotsAlong, int aWidth,
                          int aClearance, int aViaDia );

/// @brief The letters a reference starts with, upper case ("C" for c12): the part's kind by convention
std::string KopenapiRefPrefix( const std::string& aRef );

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
