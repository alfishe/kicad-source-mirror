/// @file methods_pcb_reannotate.cpp
/// @brief kicadopenapi Geographical Reannotate: the board's references renumbered by position, as
/// the editor's Tools > Geographical Reannotate
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_reannotate.h>
#include <footprint.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_util.h>
#include <tool/tool_manager.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <set>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


/// The dialog's sort buttons in order: rows (top to bottom / bottom to top) or columns, then the
/// order within them
const std::vector<std::string> DIRECTIONS = { "top_bottom_left_right", "top_bottom_right_left",
                                              "bottom_top_left_right", "bottom_top_right_left",
                                              "left_right_top_bottom", "left_right_bottom_top",
                                              "right_left_top_bottom", "right_left_bottom_top" };


KOPENAPI_RESULT h_pcb_reannotate( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*             board = context->GetBoard();
    REANNOTATE_OPTIONS options;

    const std::string scope = aArgs.value( "scope", std::string( "all" ) );

    if( scope == "front" )
        options.Scope = REANNOTATE_OPTIONS::FRONT;
    else if( scope == "back" )
        options.Scope = REANNOTATE_OPTIONS::BACK;
    else if( scope != "all" )
        return KOPENAPI_RESULT::Error( 400, "scope: all, front or back (refs: only those footprints)" );

    std::set<const FOOTPRINT*> selected;

    if( aArgs.contains( "refs" ) )
    {
        if( !aArgs["refs"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "refs: an array of references or footprint uuids" );

        for( const nlohmann::json& spec : aArgs["refs"] )
        {
            BOARD_ITEM* item = spec.is_string() ? KopenapiFindBoardItem( board, spec.get<std::string>() ) : nullptr;

            if( !item || item->Type() != PCB_FOOTPRINT_T )
                return KOPENAPI_RESULT::Error( 404, "refs: no footprint " + spec.dump() );

            selected.insert( static_cast<FOOTPRINT*>( item ) );
        }

        if( scope != "all" )
            return KOPENAPI_RESULT::Error( 400, "refs or scope front / back, not both" );

        options.Scope = REANNOTATE_OPTIONS::SELECTION;
        options.IsSelected = [&]( const FOOTPRINT* aFootprint ) { return selected.count( aFootprint ) > 0; };
    }

    const std::string direction = aArgs.value( "direction", DIRECTIONS[0] );
    auto              dir = std::find( DIRECTIONS.begin(), DIRECTIONS.end(), direction );

    if( dir == DIRECTIONS.end() )
    {
        std::string names;

        for( const std::string& name : DIRECTIONS )
            names += ( names.empty() ? "" : ", " ) + name;

        return KOPENAPI_RESULT::Error( 400, "direction: one of " + names );
    }

    options.Direction = (int) ( dir - DIRECTIONS.begin() );

    const std::string location = aArgs.value( "location", std::string( "footprint" ) );

    if( location != "footprint" && location != "reference" )
        return KOPENAPI_RESULT::Error( 400, "location: footprint (its anchor) or reference (the reference text)" );

    options.UseFootprintLocation = location == "footprint";
    options.ExcludeLocked = aArgs.value( "exclude_locked", false );

    double gridX = 1.27, gridY = 1.27;

    if( aArgs.contains( "grid_mm" ) )
    {
        const nlohmann::json& g = aArgs["grid_mm"];

        if( g.is_number() )
            gridX = gridY = g.get<double>();
        else if( g.is_array() && g.size() == 2 && g[0].is_number() && g[1].is_number() )
            gridX = g[0].get<double>(), gridY = g[1].get<double>();
        else
            return KOPENAPI_RESULT::Error( 400, "grid_mm: a number or [x, y]" );

        if( gridX <= 0 || gridY <= 0 )
            return KOPENAPI_RESULT::Error( 400, "grid_mm: greater than 0" );
    }

    options.SortGrid = VECTOR2I( pcbIUScale.mmToIU( gridX ), pcbIUScale.mmToIU( gridY ) );
    options.FrontStart = aArgs.value( "front_start", 1 );
    options.BackStart = aArgs.value( "back_start", 0 );

    if( options.FrontStart < 0 || options.BackStart < 0 )
        return KOPENAPI_RESULT::Error( 400, "front_start / back_start: 0 or more" );

    for( const char* key : { "front_prefix", "back_prefix" } )
    {
        const std::string prefix = aArgs.value( key, std::string() );

        for( char c : prefix )
        {
            if( !std::isalnum( (unsigned char) c ) && std::string( VALIDPREFIX ).find( c ) == std::string::npos )
                return KOPENAPI_RESULT::Error( 400, std::string( key ) + ": letters, digits and " VALIDPREFIX " only" );
        }
    }

    options.FrontPrefix = wxString::FromUTF8( aArgs.value( "front_prefix", std::string() ) );
    options.BackPrefix = wxString::FromUTF8( aArgs.value( "back_prefix", std::string() ) );
    options.RemoveFrontPrefix = aArgs.value( "remove_front_prefix", false );
    options.RemoveBackPrefix = aArgs.value( "remove_back_prefix", false );

    if( aArgs.contains( "exclude" ) )
    {
        for( const nlohmann::json& e : aArgs["exclude"] )
        {
            if( !e.is_string() )
                return KOPENAPI_RESULT::Error( 400, "exclude: prefixes (R, C) or references ending in * (U1*)" );

            options.Exclude.push_back( wxString::FromUTF8( e.get<std::string>() ) );
        }
    }

    std::vector<FOOTPRINT*> footprints( board->Footprints().begin(), board->Footprints().end() );
    BOARD_REANNOTATOR       reannotator( options );
    const bool              planned = reannotator.Plan( footprints );

    nlohmann::json bad = nlohmann::json::array();

    for( const REFDES_INFO& info : reannotator.BadRefDes() )
    {
        bad.push_back( { { "ref", str( info.RefDesString ) },
                         { "uuid", str( info.Uuid.AsString() ) },
                         { "footprint", info.FPID.IsValid() ? std::string( info.FPID.Format().c_str() ) : "" } } );
    }

    if( !planned )
    {
        nlohmann::json dup = nlohmann::json::array();

        for( const wxString& d : reannotator.Duplicates() )
            dup.push_back( str( d ) );

        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error(
                422, "the options give the same reference twice (a start number or prefix that reaches excluded "
                     "references' numbers?): nothing changed" );
        r.body["duplicates"] = dup;
        return r;
    }

    if( !bad.empty() && !aArgs.value( "force", false ) && !aArgs.value( "dry_run", false ) )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error(
                409, "the board has empty or invalid references (they stay as they are): force: true reannotates "
                     "the others anyway, as the editor's \"Reannotate anyway?\"" );
        r.body["bad_refs"] = bad;
        return r;
    }

    nlohmann::json changes = nlohmann::json::array();
    nlohmann::json excluded = nlohmann::json::array();
    int            unchanged = 0;
    const bool     dryRun = aArgs.value( "dry_run", false );
    BOARD_COMMIT   commit( context->GetToolManager() );

    for( const REFDES_CHANGE& change : reannotator.Changes() )
    {
        if( change.Action == EXCLUDE_REFDES )
            excluded.push_back( str( change.OldRefDesString ) );
        else if( change.Action == UPDATE_REFDES && change.NewRefDes != change.OldRefDesString )
            changes.push_back( { { "old", str( change.OldRefDesString ) },
                                 { "new", str( change.NewRefDes ) },
                                 { "side", change.Front ? "front" : "back" } } );
        else if( change.Action == UPDATE_REFDES )
            unchanged++;
    }

    if( !dryRun && !changes.empty() )
    {
        for( FOOTPRINT* footprint : footprints )
        {
            const REFDES_CHANGE* change = reannotator.GetNewRefDes( footprint );

            if( change && change->Action == UPDATE_REFDES && footprint->GetReference() != change->NewRefDes )
            {
                commit.Modify( footprint );
                footprint->SetReference( change->NewRefDes );
            }
        }

        commit.Push( _( "Annotation" ) );
        context->SetContentModified();
    }

    nlohmann::json order = nlohmann::json::array();

    for( const std::vector<REFDES_INFO>* side : { &reannotator.FrontFootprints(), &reannotator.BackFootprints() } )
    {
        for( const REFDES_INFO& info : *side )
            order.push_back( str( info.RefDesString ) );
    }

    nlohmann::json out = { { "changed", changes.size() },
                           { "changes", changes },
                           { "unchanged", unchanged },
                           { "excluded", excluded },
                           { "order", order },
                           { "dry_run", dryRun } };

    if( !bad.empty() )
        out["bad_refs"] = bad;

    if( !dryRun && !changes.empty() )
        out["next"] = "the schematic keeps the old references: update it from the board (KiCad's Update Schematic "
                      "from PCB)";

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_reannotate",
                   "Geographical Reannotate, as the editor's: footprints sorted by position on a grid (grid_mm, "
                   "default 1.27; position of the footprint or of its reference text: location) along direction "
                   "(rows top_bottom / bottom_top, each left_right / right_left, or columns left_right / "
                   "right_left, each top_bottom / bottom_top; the back is seen from the back), front then back, "
                   "renumbered per prefix from front_start (default 1) and back_start (0 continues after the "
                   "front); front_prefix / back_prefix added (remove_*_prefix: removed). scope all / front / back, "
                   "or refs (only those). exclude: prefixes or references ending in *; exclude_locked; excluded "
                   "numbers are not reused. Empty / invalid references stay and are refused (409) unless force. "
                   "Answers the changes, the sort order, excluded; dry_run previews. One undo step",
                   R"json({"type":"object","properties":{
                        "scope":{"type":"string","enum":["all","front","back"],"default":"all"},
                        "refs":{"type":"array","items":{"type":"string"}},
                        "direction":{"type":"string","enum":["top_bottom_left_right","top_bottom_right_left",
                            "bottom_top_left_right","bottom_top_right_left","left_right_top_bottom",
                            "left_right_bottom_top","right_left_top_bottom","right_left_bottom_top"],
                            "default":"top_bottom_left_right"},
                        "location":{"type":"string","enum":["footprint","reference"],"default":"footprint"},
                        "grid_mm":{"oneOf":[{"type":"number"},{"type":"array","items":{"type":"number"}}]},
                        "front_start":{"type":"integer","default":1},"back_start":{"type":"integer","default":0},
                        "front_prefix":{"type":"string"},"back_prefix":{"type":"string"},
                        "remove_front_prefix":{"type":"boolean"},"remove_back_prefix":{"type":"boolean"},
                        "exclude":{"type":"array","items":{"type":"string"}},
                        "exclude_locked":{"type":"boolean","default":false},
                        "force":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_reannotate );

KOPENAPI_MARK_EDITING( "pcb_reannotate" );
