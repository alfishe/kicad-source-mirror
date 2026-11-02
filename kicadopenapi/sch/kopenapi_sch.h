/// @file kopenapi_sch.h
/// @brief kicadopenapi schematic access (compiled into the eeschema kiface).
///
/// One schematic per process.  GUI hosts reach it through the schematic editor frame,
/// headless hosts through a HEADLESS_SCH_CONTEXT owned here; both behind SCH_CONTEXT.
#ifndef KOPENAPI_SCH_H
#define KOPENAPI_SCH_H

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <kicadopenapi_registry.h>

class SCH_CONTEXT;


/// @brief The open schematic's context, or nullptr when no schematic is open
std::shared_ptr<SCH_CONTEXT> KopenapiSchContext( KOPENAPI_CONTEXT& aCtx );

/// @brief 409 "no schematic open" result for methods that need one
KOPENAPI_RESULT KopenapiNoSchematic();

class SCH_EDIT_FRAME;
class SCH_ITEM;

/// @brief An item of the shown sheet by uuid, a symbol by reference, a pin as REF.NUMBER; nullptr if none
SCH_ITEM* KopenapiFindSchematicItem( SCH_EDIT_FRAME* aFrame, const std::string& aRef );

/// @brief The item of aScope that aId names: its whole uuid or a prefix of 4+ characters (case
/// ignored; the same item listed twice counts once); nullptr and aError when none or several
SCH_ITEM* KopenapiSchMatchUuid( const std::vector<SCH_ITEM*>& aScope, const std::string& aId, std::string& aError );

/// @brief Every entry of aWanted that names one item of aScope (KopenapiSchMatchUuid) replaced by
/// that item's uuid; entries naming none are kept as given (for the caller's "not found").  False
/// and aError when an entry is an ambiguous prefix
bool KopenapiSchExpandUuids( const std::vector<SCH_ITEM*>& aScope, std::set<std::string>& aWanted, std::string& aError );

class KIID;
class KIWAY;
class SCH_COMMIT;
class SCH_SHEET_PATH;
class SCHEMATIC;
class wxString;

/// @brief Target sheet instance: args "sheet" (the path sch_sheet_list lists), else the editor's
/// current sheet, else the root
std::optional<SCH_SHEET_PATH> KopenapiSchTargetSheet( SCH_CONTEXT& aContext, const nlohmann::json& aArgs );

/// @brief First free reference number for a prefix ("C" -> "C4") over all sheet instances and aTaken
/// (the references handed out before, extended with the one returned)
wxString KopenapiSchNextFreeRef( SCHEMATIC* aSchematic, const wxString& aPrefix, std::set<wxString>& aTaken );

/// @brief Push an edit as one undo step, rebuild connectivity, and let the GUI glow aGlow
void KopenapiSchPushEdit( SCH_COMMIT& aCommit, SCHEMATIC* aSchematic, const wxString& aMessage, KIWAY* aKiway,
                          const std::vector<KIID>& aGlow );

#endif
