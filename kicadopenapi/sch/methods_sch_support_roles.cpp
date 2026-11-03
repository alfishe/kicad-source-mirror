/// @file methods_sch_support_roles.cpp
/// @brief kicadopenapi sch_support_roles_get / sch_support_roles_set: the Support.Role field of the schematic's
/// parts (docs/inprogress/2026-10-31-router-architecture/support-templates.md §1, §5).  The role is read from the
/// instance's field, else the library symbol's field, else the board's connectivity inference (pcb_support_roles).
#include "kopenapi_sch.h"

#include <api/sch_context.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <kopenapi_support_roles.h>
#include <lib_symbol.h>
#include <sch_commit.h>
#include <sch_field.h>
#include <sch_reference_list.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <tool/tool_manager.h>

#include <map>
#include <optional>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return std::string( aText.ToUTF8() );
}


std::string normalized( const SCH_FIELD* aField )
{
    if( !aField )
        return std::string();

    wxString text = aField->GetText();
    return str( text.Trim().Trim( false ).Lower() );
}


/// One part: all its units (one SCH_SYMBOL each) on their sheets.
struct PART
{
    std::string                                       ref;
    std::string                                       value;
    std::vector<std::pair<SCH_SYMBOL*, SCH_SCREEN*>> units;
    std::string                                       instance;   // the instance's field, normalized
    std::string                                       library;    // the library symbol's field, normalized
    bool                                              locked = false;
};


std::map<std::string, PART> collectParts( SCHEMATIC* aSchematic )
{
    std::map<std::string, PART> parts;
    SCH_REFERENCE_LIST          refs;

    aSchematic->Hierarchy().GetSymbols( refs, SYMBOL_FILTER_NON_POWER );

    for( size_t i = 0; i < refs.GetCount(); ++i )
    {
        SCH_SYMBOL*       symbol = refs[i].GetSymbol();
        const std::string ref = str( refs[i].GetRef() );
        PART&             part = parts[ref];

        part.ref = ref;
        part.value = str( refs[i].GetValue() );
        part.units.push_back( { symbol, refs[i].GetSheetPath().LastScreen() } );
        part.locked = part.locked || symbol->IsLocked();

        if( part.instance.empty() )
            part.instance = normalized( symbol->GetField( wxString( KOPENAPI_SUPPORT_ROLE_FIELD ) ) );

        if( part.library.empty() && symbol->GetLibSymbolRef() )
            part.library = normalized( symbol->GetLibSymbolRef()->GetField( wxString( KOPENAPI_SUPPORT_ROLE_FIELD ) ) );
    }

    return parts;
}


/// The board's inference (pcb_support_roles) by reference, or nullopt with a reason when no board is open.
std::optional<std::map<std::string, nlohmann::json>> inferred( KOPENAPI_CONTEXT& aCtx, std::string& aReason )
{
    std::optional<KOPENAPI_METHOD> method = KOPENAPI_REGISTRY::Get().Find( "pcb_support_roles" );

    if( !method )
    {
        aReason = "pcb_support_roles is not available";
        return std::nullopt;
    }

    const KOPENAPI_RESULT r = method->handler( aCtx, nlohmann::json::object() );

    if( r.status >= 400 )
    {
        aReason = "connectivity inference needs the board open (pcb_open)";
        return std::nullopt;
    }

    std::map<std::string, nlohmann::json> rows;

    for( const char* key : { "parts", "unassigned" } )
    {
        if( r.body.contains( key ) && r.body[key].is_array() )
        {
            for( const nlohmann::json& row : r.body[key] )
                rows[row.value( "ref", std::string() )] = row;
        }
    }

    return rows;
}


/// The role row of one part: role, source (instance / library / connectivity / none), reason.
nlohmann::json roleRow( const PART& aPart, const std::optional<std::map<std::string, nlohmann::json>>& aInferred,
                        const std::string& aNoBoard )
{
    nlohmann::json row = { { "ref", aPart.ref }, { "value", aPart.value } };

    auto fromField = [&]( const std::string& aRole, const char* aSource )
    {
        row["source"] = aSource;

        if( KopenapiSupportRoleValues().count( aRole ) )
            row["role"] = aRole;
        else
            row["reason"] = "unknown Support.Role '" + aRole + "'";
    };

    if( !aPart.instance.empty() )
    {
        fromField( aPart.instance, "instance" );

        if( !aPart.library.empty() && aPart.library == aPart.instance )
            row["source"] = "library";
    }
    else if( !aPart.library.empty() )
    {
        fromField( aPart.library, "library" );
    }
    else if( !aInferred )
    {
        row["source"] = "none";
        row["reason"] = aNoBoard;
    }
    else if( auto it = aInferred->find( aPart.ref ); it != aInferred->end() )
    {
        if( it->second.contains( "role" ) )
        {
            // "field" there: the footprint has a Support.Role the schematic lacks (the board is not updated)
            row["source"] = it->second.value( "source", std::string() ) == "field" ? "board" : "connectivity";
            row["role"] = it->second["role"];

            if( it->second.contains( "anchor" ) )
                row["anchor"] = it->second["anchor"];
        }
        else
        {
            row["source"] = "none";
        }

        row["reason"] = it->second.value( "reason", std::string() );
    }
    else
    {
        row["source"] = "none";
        row["reason"] = "not a two-terminal support part and no Support.Role field";
        row["support"] = false;
    }

    return row;
}

} // namespace


static KOPENAPI_RESULT h_sch_support_roles_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string glob = aArgs.value( "refs", std::string() );
    const bool        all = aArgs.value( "all", false );
    std::string       noBoard;
    const auto        guess = inferred( aCtx, noBoard );

    nlohmann::json rows = nlohmann::json::array();
    nlohmann::json unassigned = nlohmann::json::array();
    nlohmann::json counts = nlohmann::json::object();
    nlohmann::json sources = nlohmann::json::object();

    for( const auto& [ref, part] : collectParts( context->GetSchematic() ) )
    {
        if( !glob.empty() && !KopenapiGlob( glob, ref ) )
            continue;

        nlohmann::json row = roleRow( part, guess, noBoard );

        if( row.contains( "role" ) )
        {
            const std::string role = row["role"];
            counts[role] = counts.value( role, 0 ) + 1;
            const std::string source = row["source"];
            sources[source] = sources.value( source, 0 ) + 1;
            rows.push_back( row );
        }
        else if( !row.contains( "support" ) )
        {
            unassigned.push_back( { { "ref", ref }, { "source", row["source"] }, { "reason", row["reason"] } } );
        }
        else if( all )
        {
            row.erase( "support" );
            rows.push_back( row );
        }
    }

    nlohmann::json body = { { "parts", rows }, { "unassigned", unassigned }, { "counts", counts },
                            { "sources", sources } };

    if( !guess )
        body["connectivity"] = noBoard;

    return KOPENAPI_RESULT::Ok( body );
}


static KOPENAPI_RESULT h_sch_support_roles_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::map<std::string, PART>        parts = collectParts( context->GetSchematic() );
    std::map<std::string, std::string> wanted;   // ref -> role ("" clears the field)
    nlohmann::json                     refused = nlohmann::json::array();

    auto want = [&]( const std::string& aRef, std::string aRole, const char* aOrigin )
    {
        aRole = str( wxString::FromUTF8( aRole ).Trim().Trim( false ).Lower() );

        if( !parts.count( aRef ) )
            refused.push_back( { { "ref", aRef }, { "reason", "no such part" } } );
        else if( !aRole.empty() && !KopenapiSupportRoleValues().count( aRole ) )
            refused.push_back( { { "ref", aRef }, { "reason", std::string( aOrigin ) + ": unknown role '" + aRole + "'" } } );
        else
            wanted[aRef] = aRole;
    };

    if( aArgs.contains( "parts" ) && aArgs["parts"].is_array() )
    {
        for( const nlohmann::json& p : aArgs["parts"] )
        {
            if( p.is_object() && p.contains( "ref" ) && p.contains( "role" ) && p["role"].is_string() )
                want( p["ref"].get<std::string>(), p["role"].get<std::string>(), "parts" );
            else
                refused.push_back( { { "entry", p }, { "reason", "parts entries are {ref, role}" } } );
        }
    }

    const std::string glob = aArgs.value( "refs", std::string() );

    if( aArgs.contains( "role" ) )
    {
        if( glob.empty() || !aArgs["role"].is_string() )
            return KOPENAPI_RESULT::Error( 400, "role needs refs (a glob over references) and is a string" );

        for( const auto& [ref, part] : parts )
        {
            if( KopenapiGlob( glob, ref ) )
                want( ref, aArgs["role"].get<std::string>(), "role" );
        }
    }

    if( aArgs.value( "from_inferred", false ) )
    {
        std::string noBoard;
        const auto  guess = inferred( aCtx, noBoard );

        if( !guess )
            return KOPENAPI_RESULT::Error( 409, noBoard );

        for( const auto& [ref, part] : parts )
        {
            if( ( glob.empty() || KopenapiGlob( glob, ref ) ) && part.instance.empty() && part.library.empty()
                && !wanted.count( ref ) )
            {
                auto it = guess->find( ref );

                if( it != guess->end() && it->second.contains( "role" ) )
                    want( ref, it->second["role"].get<std::string>(), "from_inferred" );
            }
        }
    }

    if( wanted.empty() && refused.empty() )
        return KOPENAPI_RESULT::Error( 400, "give parts [{ref, role}], refs + role, or from_inferred: true" );

    const bool overrideLocks = aArgs.value( "override_locks", false );
    const bool dryRun = aArgs.value( "dry_run", false );

    SCH_COMMIT        commit( context->GetToolManager() );
    bool              touched = false;
    nlohmann::json    changed = nlohmann::json::array();
    nlohmann::json    kept = nlohmann::json::array();

    for( const auto& [ref, role] : wanted )
    {
        PART& part = parts[ref];

        if( part.locked && !overrideLocks )
        {
            refused.push_back( { { "ref", ref }, { "reason", "locked (sch_unlock, or override_locks: true)" } } );
            continue;
        }

        if( part.instance == role )
        {
            kept.push_back( ref );
            continue;
        }

        changed.push_back( { { "ref", ref }, { "from", part.instance }, { "to", role } } );

        if( dryRun )
            continue;

        for( auto& [symbol, screen] : part.units )
        {
            commit.Modify( symbol, screen );

            if( role.empty() )
            {
                symbol->RemoveField( wxString( KOPENAPI_SUPPORT_ROLE_FIELD ) );
            }
            else
            {
                SCH_FIELD* field = symbol->GetField( wxString( KOPENAPI_SUPPORT_ROLE_FIELD ) );

                if( !field )
                {
                    SCH_FIELD added( symbol, FIELD_T::USER, wxString( KOPENAPI_SUPPORT_ROLE_FIELD ) );
                    added.SetPosition( symbol->GetPosition() );
                    added.SetVisible( false );
                    field = symbol->AddField( added );
                }

                field->SetText( wxString::FromUTF8( role ) );
            }

            touched = true;
        }
    }

    if( touched )
        commit.Push( _( "Set support roles (API)" ) );

    return KOPENAPI_RESULT::Ok( { { "changed", changed }, { "unchanged", kept }, { "refused", refused },
                                  { "dry_run", dryRun } } );
}


KOPENAPI_REGISTER( "sch_support_roles_get",
                   "Every part's support role for L2 placement and where it comes from: source instance (the "
                   "symbol instance's Support.Role field), library (the library symbol's field, the same as on the "
                   "instance or not copied there), connectivity (inferred on the open board by pcb_support_roles; "
                   "none when no board is open, said in 'connectivity'), board (the footprint has a Support.Role "
                   "field the schematic lacks) or none. Answers parts [{ref, value, role, "
                   "source, reason, anchor}], unassigned [{ref, source, reason}] (support-like parts that got no "
                   "role, and unknown field values), counts per role and per source. all: true lists the "
                   "non-support parts too",
                   R"json({"type":"object","properties":{
                        "refs":{"type":"string","description":"glob over references"},
                        "all":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_support_roles_get );

KOPENAPI_REGISTER( "sch_support_roles_set",
                   "Set the Support.Role field on many symbol instances in one undo step (all units of a part; a new "
                   "field is hidden). Give parts [{ref, role}], or refs (glob) + role, or from_inferred: true to "
                   "write the board's inferred roles (pcb_support_roles) into the parts that have no field (refs "
                   "limits it). role \"\" removes the field. Roles: decoupling, bulk, filter, inductor, series, "
                   "pullup, crystal, crystal_load, esd, none. Locked parts are refused unless override_locks. "
                   "Answers changed [{ref, from, to}], unchanged, refused [{ref, reason}]; dry_run reports without "
                   "editing",
                   R"json({"type":"object","properties":{
                        "parts":{"type":"array","items":{"type":"object","properties":{
                            "ref":{"type":"string"},"role":{"type":"string"}},"required":["ref","role"]}},
                        "refs":{"type":"string","description":"glob over references"},
                        "role":{"type":"string"},
                        "from_inferred":{"type":"boolean","default":false},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_support_roles_set );

KOPENAPI_MARK_EDITING( "sch_support_roles_set" );
