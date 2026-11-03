/// @file methods_pcb_templates.cpp
/// @brief kicadopenapi support-part templates (docs/inprogress/2026-10-31-router-architecture/support-templates.md):
/// one template per (pin_role, support_role) says how L2 places that support part.  Layers, each replacing the
/// previous field by field: library (built in), user (<settings>/templates/*.kicad_templates), project
/// (<project>.kicad_templates) and call (templates: [...] on one call).
#include "kopenapi_pcb.h"
#include "kopenapi_pcb_templates.h"

#include <api/pcb_context.h>
#include <board.h>
#include <kicadopenapi_util.h>
#include <kopenapi_support_roles.h>
#include <paths.h>
#include <project.h>

#include <wx/dir.h>
#include <wx/ffile.h>
#include <wx/filename.h>

#include <algorithm>
#include <cctype>
#include <map>
#include <optional>
#include <set>
#include <sstream>


static const char* const TEMPLATES_EXT = "kicad_templates";

// The library layer: one default per support role, for any pin role ("*"), plus the pin-specific ones.
static const char* const LIBRARY_TEMPLATES = R"json([
  {"pin_role":"*","support_role":"decoupling","per":"pin_group","rank":"value_ascending",
   "reach":"via_pitch + pad_to_pad","side":["same","opposite"],"keep":["ports_band","pair_ways","fanout_vias"],
   "gap":"courtyard","resort":["band_edge","inside_ring","opposite_side","pending_spots"],"order":30,
   "report_past":"reach"},
  {"pin_role":"*","support_role":"bulk","per":"rail","rank":"value_descending","reach":"rail_entry",
   "side":["same"],"keep":["ports_band","pair_ways"],"gap":"courtyard","resort":["pending_spots"],"order":40,
   "report_past":"never"},
  {"pin_role":"*","support_role":"filter","per":"rail","rank":"none","reach":"via_pitch + pad_to_pad",
   "side":["same"],"keep":["ports_band","pair_ways"],"gap":"courtyard","resort":["band_edge","pending_spots"],
   "order":50,"report_past":"reach"},
  {"pin_role":"*","support_role":"inductor","per":"pin","rank":"none","reach":"loop","side":["same"],
   "keep":["ports_band"],"gap":"courtyard","resort":["inside_ring"],"order":20,"report_past":"2 x via_pitch"},
  {"pin_role":"*","support_role":"series","per":"port","rank":"none","reach":"escape","side":["same"],
   "keep":["ports_band","pair_ways"],"gap":"channel","resort":["kept_spot","band_edge"],"order":60,
   "report_past":"reach"},
  {"pin_role":"*","support_role":"pullup","per":"pin","rank":"none","reach":"2 x via_pitch","side":["same"],
   "keep":["ports_band","pair_ways"],"gap":"channel","resort":["band_edge","pending_spots"],"order":70,
   "report_past":"reach"},
  {"pin_role":"*","support_role":"crystal","per":"port","rank":"none","reach":"via_pitch + pad_to_pad",
   "side":["same"],"keep":["ports_band","pair_ways"],"gap":"courtyard","resort":["inside_ring"],"order":10,
   "report_past":"reach"},
  {"pin_role":"*","support_role":"crystal_load","per":"pin","rank":"none","reach":"via_pitch + pad_to_pad",
   "side":["same"],"keep":["ports_band"],"gap":"courtyard","resort":["inside_ring","pending_spots"],"order":15,
   "report_past":"reach"},
  {"pin_role":"*","support_role":"esd","per":"pin","rank":"none","reach":"via_pitch + pad_to_pad",
   "side":["same"],"keep":["pair_ways"],"gap":"courtyard","resort":["band_edge"],"order":80,
   "report_past":"reach"}
])json";

static const std::vector<std::string> LAYERS = { "library", "user", "project", "call" };

static const std::set<std::string> PIN_ROLES = { "*", "ground", "power", "power_hc", "local_supply", "pair", "clock",
                                                 "matched", "bus", "analog", "analog_hs", "switching", "signal" };

static const std::set<std::string> TOKENS = { "via_pitch", "track_pitch", "pad_to_pad", "courtyard", "ring",
                                              "ports_band", "escape", "loop", "rail_entry", "reach" };

static const std::vector<std::string> FIELDS = { "per", "rank", "reach", "side", "keep", "gap", "resort", "order",
                                                 "report_past" };


static std::string str( const wxString& aText )
{
    return std::string( aText.ToUTF8() );
}


static std::string trim( const std::string& aText )
{
    const size_t a = aText.find_first_not_of( " \t" );
    const size_t b = aText.find_last_not_of( " \t" );
    return a == std::string::npos ? std::string() : aText.substr( a, b - a + 1 );
}


/// A distance expression: terms joined by "+", each a token, "N x token" (also × or *) or, where literals are
/// allowed, "<number> mm".  Empty when valid, else why not.
std::string KopenapiTemplateExpressionError( const std::string& aExpr, bool aLiteralAllowed, bool aNeverAllowed )
{
    if( aNeverAllowed && trim( aExpr ) == "never" )
        return std::string();

    std::string text = aExpr;

    for( const std::string times : { "\xC3\x97", "*" } )   // ×
    {
        for( size_t at = text.find( times ); at != std::string::npos; at = text.find( times ) )
            text.replace( at, times.size(), " x " );
    }

    std::stringstream terms( text );
    std::string       term;
    int               count = 0;

    while( std::getline( terms, term, '+' ) )
    {
        term = trim( term );
        ++count;

        if( term.empty() )
            return "an empty term in '" + aExpr + "'";

        std::stringstream words( term );
        std::vector<std::string> w;

        for( std::string x; words >> x; )
            w.push_back( x );

        auto number = []( const std::string& aWord )
        {
            char*        end = nullptr;
            const double v = std::strtod( aWord.c_str(), &end );
            return end && *end == 0 && v >= 0;
        };

        if( w.size() == 1 && TOKENS.count( w[0] ) )
            continue;

        if( w.size() == 3 && number( w[0] ) && w[1] == "x" && TOKENS.count( w[2] ) )
            continue;

        if( w.size() == 2 && number( w[0] ) && w[1] == "mm" )
        {
            if( !aLiteralAllowed )
                return "a literal distance ('" + term + "') is allowed only in a project or call template";

            continue;
        }

        return "'" + term + "' is no token (via_pitch, track_pitch, pad_to_pad, courtyard, ring, ports_band, escape, "
               "loop, rail_entry, reach), 'N x token' or, in a project or call template, 'N mm'";
    }

    return count ? std::string() : "an empty expression";
}


/// Validates one (possibly partial) template of a layer.  Empty when valid, else why not.
static std::string templateError( const nlohmann::json& aT, const std::string& aLayer, bool aNullRemoves = false )
{
    if( !aT.is_object() )
        return "a template is an object";

    const std::string pin = aT.value( "pin_role", std::string() );
    const std::string role = aT.value( "support_role", std::string() );

    if( !PIN_ROLES.count( pin ) )
        return "pin_role '" + pin + "' is none of the pcb_net_roles roles or '*'";

    if( role == "none" || !KopenapiSupportRoleValues().count( role ) )
        return "support_role '" + role + "' is none of the Support.Role values (other than none)";

    const std::string where = pin + "/" + role + ": ";
    const bool        literal = aLayer == "project" || aLayer == "call";

    for( const auto& [key, value] : aT.items() )
    {
        if( key == "pin_role" || key == "support_role" || ( aNullRemoves && value.is_null() ) )
            continue;

        auto oneOf = [&]( const std::set<std::string>& aValues ) -> std::string
        {
            if( value.is_string() && aValues.count( value.get<std::string>() ) )
                return std::string();

            std::string all;

            for( const std::string& v : aValues )
                all += ( all.empty() ? "" : ", " ) + v;

            return where + key + " is one of " + all;
        };

        auto listOf = [&]( const std::set<std::string>& aValues, bool aNonEmpty ) -> std::string
        {
            if( !value.is_array() || ( aNonEmpty && value.empty() ) )
                return where + key + " is a" + std::string( aNonEmpty ? " non-empty" : "n" ) + " array";

            for( const nlohmann::json& v : value )
            {
                if( !v.is_string() || !aValues.count( v.get<std::string>() ) )
                    return where + key + ": '" + v.dump() + "' is not allowed";
            }

            return std::string();
        };

        std::string error;

        if( key == "per" )
            error = oneOf( { "pin", "pin_group", "rail", "port" } );
        else if( key == "rank" )
            error = oneOf( { "value_ascending", "value_descending", "none" } );
        else if( key == "gap" )
            error = oneOf( { "courtyard", "channel" } );
        else if( key == "side" )
            error = listOf( { "same", "opposite" }, true );
        else if( key == "keep" )
            error = listOf( { "ports_band", "pair_ways", "fanout_vias", "ring" }, false );
        else if( key == "resort" )
            error = listOf( { "band_edge", "inside_ring", "opposite_side", "pending_spots", "kept_spot" }, false );
        else if( key == "order" )
            error = value.is_number_integer() ? std::string() : where + "order is an integer";
        else if( key == "reach" || key == "report_past" )
        {
            if( !value.is_string() )
                error = where + key + " is an expression string";
            else if( key == "reach" && value.get<std::string>().find( "reach" ) != std::string::npos )
                error = where + "reach cannot refer to itself";
            else if( std::string e = KopenapiTemplateExpressionError( value.get<std::string>(), literal, key == "report_past" );
                     !e.empty() )
                error = where + key + ": " + e;
        }
        else
            error = where + "unknown field '" + key + "'";

        if( !error.empty() )
            return error;
    }

    return std::string();
}


static std::string keyOf( const nlohmann::json& aT )
{
    return aT.value( "pin_role", std::string() ) + "/" + aT.value( "support_role", std::string() );
}


static std::string userDir()
{
    wxFileName dir = wxFileName::DirName( PATHS::GetUserSettingsPath() );
    dir.AppendDir( wxS( "templates" ) );
    return str( dir.GetPath() );
}


std::string KopenapiTemplatesProjectFile( BOARD* aBoard )
{
    PROJECT* project = aBoard ? aBoard->GetProject() : nullptr;

    if( !project || project->GetProjectFullName().IsEmpty() )
        return std::string();

    wxFileName fn( project->GetProjectFullName() );
    fn.SetExt( wxString::FromUTF8( TEMPLATES_EXT ) );
    return str( fn.GetFullPath() );
}


/// The templates of a file: {"templates": [...]} or a bare array.  nullopt with aError when unreadable.
static std::optional<nlohmann::json> readFile( const std::string& aPath, std::string& aError )
{
    wxFFile  file( wxString::FromUTF8( aPath ), wxS( "rb" ) );
    wxString text;

    if( !file.IsOpened() || !file.ReadAll( &text, wxConvUTF8 ) )
    {
        aError = aPath + ": cannot read";
        return std::nullopt;
    }

    try
    {
        nlohmann::json j = nlohmann::json::parse( str( text ) );

        if( j.is_object() && j.contains( "templates" ) )
            j = j["templates"];

        if( !j.is_array() )
        {
            aError = aPath + ": not {\"templates\": [...]}";
            return std::nullopt;
        }

        return j;
    }
    catch( const std::exception& e )
    {
        aError = aPath + ": " + e.what();
        return std::nullopt;
    }
}


KOPENAPI_TEMPLATES KopenapiLoadTemplates( BOARD* aBoard, const nlohmann::json& aCall )
{
    KOPENAPI_TEMPLATES out;
    out.files = { { "library", "built in" }, { "user", userDir() } };

    if( std::string project = KopenapiTemplatesProjectFile( aBoard ); !project.empty() )
        out.files["project"] = project;

    std::vector<std::pair<std::string, nlohmann::json>> layers;   // (layer, templates) in order
    layers.push_back( { "library", nlohmann::json::parse( LIBRARY_TEMPLATES ) } );

    wxArrayString userFiles;

    if( wxDirExists( wxString::FromUTF8( userDir() ) ) )
        wxDir::GetAllFiles( wxString::FromUTF8( userDir() ), &userFiles, wxS( "*." ) + wxString( TEMPLATES_EXT ), wxDIR_FILES );

    userFiles.Sort();

    for( const wxString& f : userFiles )
    {
        std::string error;

        if( std::optional<nlohmann::json> t = readFile( str( f ), error ) )
            layers.push_back( { "user", *t } );
        else
            out.problems.push_back( { { "layer", "user" }, { "file", str( f ) }, { "error", error } } );
    }

    if( out.files.contains( "project" ) && wxFileExists( wxString::FromUTF8( out.files["project"].get<std::string>() ) ) )
    {
        std::string error;

        if( std::optional<nlohmann::json> t = readFile( out.files["project"], error ) )
            layers.push_back( { "project", *t } );
        else
            out.problems.push_back( { { "layer", "project" }, { "file", out.files["project"] }, { "error", error } } );
    }

    if( aCall.is_array() )
        layers.push_back( { "call", aCall } );

    std::map<std::string, nlohmann::json> merged;
    std::vector<std::string>              order;

    for( const auto& [layer, templates] : layers )
    {
        for( const nlohmann::json& t : templates )
        {
            if( std::string error = templateError( t, layer ); !error.empty() )
            {
                out.problems.push_back( { { "layer", layer }, { "error", error } } );
                continue;
            }

            const std::string key = keyOf( t );
            nlohmann::json&   m = merged[key];

            if( m.is_null() )
            {
                // a new key starts from the library default of its support role ("*/role")
                auto base = merged.find( "*/" + t["support_role"].get<std::string>() );
                m = base != merged.end() ? base->second : nlohmann::json::object();

                if( m.contains( "source" ) )
                {
                    for( auto& [field, from] : m["source"].items() )
                        from = from.get<std::string>() + " (" + "*/" + t["support_role"].get<std::string>() + ")";
                }

                order.push_back( key );
            }

            for( const auto& [field, value] : t.items() )
            {
                m[field] = value;

                if( field != "pin_role" && field != "support_role" )
                    m["source"][field] = layer;
            }
        }
    }

    for( const std::string& key : order )
        out.templates.push_back( merged[key] );

    return out;
}


const nlohmann::json* KOPENAPI_TEMPLATES::Find( const std::string& aPinRole, const std::string& aSupportRole ) const
{
    const nlohmann::json* fallback = nullptr;

    for( const nlohmann::json& t : templates )
    {
        if( t.value( "support_role", std::string() ) != aSupportRole )
            continue;

        if( t.value( "pin_role", std::string() ) == aPinRole )
            return &t;

        if( t.value( "pin_role", std::string() ) == "*" )
            fallback = &t;
    }

    return fallback;
}


static KOPENAPI_RESULT h_pcb_templates_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                   board = context->GetBoard();
    const KOPENAPI_TEMPLATES t = KopenapiLoadTemplates( board, aArgs.value( "templates", nlohmann::json() ) );
    nlohmann::json           body = { { "templates", t.templates }, { "files", t.files }, { "problems", t.problems } };

    if( aArgs.value( "parts", true ) )
    {
        const nlohmann::json roles = KopenapiSupportRoles( aCtx, board, aArgs.value( "refs", std::string() ) );
        nlohmann::json       parts = nlohmann::json::array();

        for( const nlohmann::json& p : roles["parts"] )
        {
            nlohmann::json row = { { "ref", p["ref"] }, { "role", p["role"] }, { "role_source", p["source"] } };
            const std::string pin = p.value( "pin_role", std::string() );

            if( !pin.empty() )
                row["pin_role"] = pin;

            if( const nlohmann::json* m = t.Find( pin, p["role"].get<std::string>() ) )
                row["template"] = keyOf( *m );

            parts.push_back( row );
        }

        body["parts"] = parts;
        body["unassigned"] = roles["unassigned"];
    }

    return KOPENAPI_RESULT::Ok( body );
}


static KOPENAPI_RESULT h_pcb_templates_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const std::string path = KopenapiTemplatesProjectFile( context->GetBoard() );

    if( path.empty() )
        return KOPENAPI_RESULT::Error( 409, "the board has no project file to put <project>.kicad_templates beside" );

    if( !aArgs.contains( "templates" ) || !aArgs["templates"].is_array() )
        return KOPENAPI_RESULT::Error( 400, "give templates: [{pin_role, support_role, fields...}]" );

    for( const nlohmann::json& t : aArgs["templates"] )
    {
        if( std::string error = templateError( t, "project", true ); !error.empty() )
            return KOPENAPI_RESULT::Error( 400, error );
    }

    const std::string mode = aArgs.value( "mode", std::string( "merge" ) );

    if( mode != "merge" && mode != "replace" )
        return KOPENAPI_RESULT::Error( 400, "mode is merge or replace" );

    // merge: the project file's templates, each given one merged field by field (null removes a field)
    std::map<std::string, nlohmann::json> byKey;
    std::vector<std::string>              order;

    if( mode == "merge" && wxFileExists( wxString::FromUTF8( path ) ) )
    {
        std::string error;
        std::optional<nlohmann::json> old = readFile( path, error );

        if( !old )
            return KOPENAPI_RESULT::Error( 422, error + " (mode: replace writes it anew)" );

        for( const nlohmann::json& t : *old )
        {
            if( t.is_object() && !byKey.count( keyOf( t ) ) )
                order.push_back( keyOf( t ) );

            byKey[keyOf( t )] = t;
        }
    }

    for( const nlohmann::json& t : aArgs["templates"] )
    {
        const std::string key = keyOf( t );

        if( !byKey.count( key ) )
        {
            order.push_back( key );
            byKey[key] = nlohmann::json::object();
        }

        for( const auto& [field, value] : t.items() )
        {
            if( value.is_null() )
                byKey[key].erase( field );
            else
                byKey[key][field] = value;
        }
    }

    nlohmann::json templates = nlohmann::json::array();

    for( const std::string& key : order )
    {
        if( byKey[key].size() > 2 || aArgs.value( "keep_empty", false ) )
            templates.push_back( byKey[key] );
    }

    wxFFile file( wxString::FromUTF8( path ), wxS( "wb" ) );

    if( !file.IsOpened() || !file.Write( wxString::FromUTF8( nlohmann::json( { { "templates", templates } } ).dump( 2 ) + "\n" ) ) )
        return KOPENAPI_RESULT::Error( 500, "cannot write " + path );

    file.Close();
    const KOPENAPI_TEMPLATES merged = KopenapiLoadTemplates( context->GetBoard(), nlohmann::json() );
    return KOPENAPI_RESULT::Ok( { { "file", path }, { "project", templates }, { "templates", merged.templates },
                                  { "problems", merged.problems } } );
}


static const char* const TEMPLATE_ITEM = R"json({"type":"object","required":["pin_role","support_role"],"properties":{
    "pin_role":{"type":"string","description":"a pcb_net_roles role of the anchor pin's net, or * for any"},
    "support_role":{"type":"string","enum":["decoupling","bulk","filter","inductor","series","pullup","crystal","crystal_load","esd"]},
    "per":{"type":"string","enum":["pin","pin_group","rail","port"]},
    "rank":{"type":"string","enum":["value_ascending","value_descending","none"]},
    "reach":{"type":"string","description":"distance expression: tokens via_pitch, track_pitch, pad_to_pad, courtyard, ring, ports_band, escape, loop, rail_entry joined by +, 'N x token'; 'N mm' only in project / call"},
    "side":{"type":"array","items":{"type":"string","enum":["same","opposite"]}},
    "keep":{"type":"array","items":{"type":"string","enum":["ports_band","pair_ways","fanout_vias","ring"]}},
    "gap":{"type":"string","enum":["courtyard","channel"]},
    "resort":{"type":"array","items":{"type":"string","enum":["band_edge","inside_ring","opposite_side","pending_spots","kept_spot"]}},
    "order":{"type":"integer"},
    "report_past":{"type":"string","description":"an expression (may use reach) or never"}}})json";


/// The set's template item: every field but the key may be null (removes it from the project template)
static nlohmann::json nullableItem()
{
    nlohmann::json item = nlohmann::json::parse( TEMPLATE_ITEM );

    for( auto& [name, prop] : item["properties"].items() )
    {
        if( name == "pin_role" || name == "support_role" )
            continue;

        prop["type"] = { prop["type"], "null" };

        if( prop.contains( "enum" ) )
            prop["enum"].push_back( nullptr );
    }

    return item;
}


KOPENAPI_REGISTER( "pcb_templates_get",
                   "The support-part templates L2 places by, merged from the library (built in), user "
                   "(<settings>/templates/*.kicad_templates), project (<project>.kicad_templates) and call "
                   "(templates) layers; each layer replaces the previous field by field per (pin_role, "
                   "support_role); a new pin_role starts from its support role's '*' template. Each template has "
                   "source: field -> layer. files: where each layer is read; problems: files or templates that "
                   "were skipped and why. parts (default on): each support part's role, its source and the "
                   "template it takes (by the role of its anchor pin's net), and the unassigned parts",
                   nlohmann::json( { { "type", "object" },
                                     { "properties",
                                       { { "templates", { { "type", "array" }, { "items", nlohmann::json::parse( TEMPLATE_ITEM ) } } },
                                         { "parts", { { "type", "boolean" }, { "default", true } } },
                                         { "refs", { { "type", "string" }, { "description", "glob over references" } } } } } } ),
                   false, h_pcb_templates_get );

KOPENAPI_REGISTER( "pcb_templates_set",
                   "Write the project's support-part templates (<project>.kicad_templates beside the .kicad_pro). "
                   "mode merge (default): each given template merges field by field into the file's one of the "
                   "same (pin_role, support_role), a null field removes it; replace: the file holds exactly the "
                   "given ones. Every field is checked (400 naming it). Answers the file, its templates and the "
                   "merged result",
                   nlohmann::json( { { "type", "object" },
                                     { "required", { "templates" } },
                                     { "properties",
                                       { { "templates", { { "type", "array" }, { "items", nullableItem() } } },
                                         { "mode", { { "type", "string" }, { "enum", { "merge", "replace" } }, { "default", "merge" } } } } } } ),
                   false, h_pcb_templates_set );

KOPENAPI_MARK_EDITING( "pcb_templates_set" );
