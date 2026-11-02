/// @file methods_pcb_rules_explain.cpp
/// @brief kicadopenapi pcb_rules_explain: the editor's Inspect > Clearance Resolution / Constraints
/// Resolution - which rule gives a constraint between two items (or for one), its value, and the
/// rule engine's own trace of how it got there.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_connected_item.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <drc/drc_rule.h>
#include <drc/drc_rule_condition.h>
#include <footprint.h>
#include <geometry/shape.h>
#include <kicadopenapi_util.h>
#include <ki_exception.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_track.h>
#include <reporter.h>
#include <string_utils.h>
#include <zone.h>

#include <wx/filename.h>
#include <wx/regex.h>
#include <wx/tokenzr.h>

#include <climits>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


struct KIND
{
    DRC_CONSTRAINT_T type;
    bool             pair;   ///< between two items (else of one item)
};


const std::map<std::string, KIND>& constraintKinds()
{
    static const std::map<std::string, KIND> kinds = {
        { "clearance", { CLEARANCE_CONSTRAINT, true } },
        { "physical_clearance", { PHYSICAL_CLEARANCE_CONSTRAINT, true } },
        { "hole_clearance", { HOLE_CLEARANCE_CONSTRAINT, true } },
        { "physical_hole_clearance", { PHYSICAL_HOLE_CLEARANCE_CONSTRAINT, true } },
        { "hole_to_hole", { HOLE_TO_HOLE_CONSTRAINT, true } },
        { "edge_clearance", { EDGE_CLEARANCE_CONSTRAINT, true } },
        { "courtyard_clearance", { COURTYARD_CLEARANCE_CONSTRAINT, true } },
        { "silk_clearance", { SILK_CLEARANCE_CONSTRAINT, true } },
        { "diff_pair_gap", { DIFF_PAIR_GAP_CONSTRAINT, false } },
        { "track_width", { TRACK_WIDTH_CONSTRAINT, false } },
        { "via_diameter", { VIA_DIAMETER_CONSTRAINT, false } },
        { "annular_width", { ANNULAR_WIDTH_CONSTRAINT, false } },
        { "hole_size", { HOLE_SIZE_CONSTRAINT, false } },
        { "length", { LENGTH_CONSTRAINT, false } },
        { "skew", { SKEW_CONSTRAINT, false } },
        { "thermal_relief_gap", { THERMAL_RELIEF_GAP_CONSTRAINT, false } },
        { "thermal_spoke_width", { THERMAL_SPOKE_WIDTH_CONSTRAINT, false } },
        { "solder_mask_expansion", { SOLDER_MASK_EXPANSION_CONSTRAINT, false } },
        { "text_height", { TEXT_HEIGHT_CONSTRAINT, false } },
        { "text_thickness", { TEXT_THICKNESS_CONSTRAINT, false } },
    };

    return kinds;
}


/// @brief What the Inspect dialogs show for these items when no constraint is asked for
std::vector<std::string> defaultKinds( const BOARD_ITEM* aA, const BOARD_ITEM* aB )
{
    auto hasHole = []( const BOARD_ITEM* aItem ) { return aItem && aItem->HasHole(); };
    auto onEdge = []( const BOARD_ITEM* aItem ) { return aItem && aItem->IsOnLayer( Edge_Cuts ); };

    if( aB )
    {
        if( onEdge( aA ) || onEdge( aB ) )
            return { "edge_clearance" };

        std::vector<std::string> kinds = { "clearance", "physical_clearance" };

        if( hasHole( aA ) || hasHole( aB ) )
            kinds.push_back( "hole_clearance" );

        if( hasHole( aA ) && hasHole( aB ) )
            kinds.push_back( "hole_to_hole" );

        if( aA->Type() == PCB_FOOTPRINT_T && aB->Type() == PCB_FOOTPRINT_T )
            kinds = { "courtyard_clearance" };

        return kinds;
    }

    switch( aA->Type() )
    {
    case PCB_TRACE_T:
    case PCB_ARC_T: return { "track_width" };
    case PCB_VIA_T: return { "via_diameter", "annular_width", "hole_size" };
    case PCB_PAD_T: return hasHole( aA ) ? std::vector<std::string>{ "hole_size", "annular_width", "solder_mask_expansion" }
                                         : std::vector<std::string>{ "solder_mask_expansion" };
    case PCB_ZONE_T: return { "thermal_relief_gap", "thermal_spoke_width" };
    default: return { "track_width" };
    }
}


nlohmann::json itemJson( BOARD_ITEM* aItem )
{
    nlohmann::json j = { { "uuid", str( aItem->m_Uuid.AsString() ) }, { "kind", str( aItem->GetClass() ) } };

    if( const BOARD_CONNECTED_ITEM* bci = dynamic_cast<const BOARD_CONNECTED_ITEM*>( aItem ) )
    {
        j["net"] = str( UnescapeString( bci->GetNetname() ) );

        if( NETCLASS* nc = bci->GetEffectiveNetClass() )
            j["netclass"] = str( nc->GetName() );
    }

    if( aItem->Type() == PCB_PAD_T )
        j["pad"] = str( static_cast<PAD*>( aItem )->GetParentFootprint()->GetReference() ) + "."
                   + str( static_cast<PAD*>( aItem )->GetNumber() );
    else if( aItem->Type() == PCB_FOOTPRINT_T )
        j["ref"] = str( static_cast<FOOTPRINT*>( aItem )->GetReference() );

    return j;
}


/// @brief HTML entities as text: named (&lt; &apos; &nbsp; ...) and numeric (&#39; &#x27;)
wxString unescapeEntities( const wxString& aText )
{
    static const std::map<wxString, wxString> named = { { wxS( "nbsp" ), wxS( " " ) }, { wxS( "lt" ), wxS( "<" ) },
                                                       { wxS( "gt" ), wxS( ">" ) },    { wxS( "amp" ), wxS( "&" ) },
                                                       { wxS( "quot" ), wxS( "\"" ) }, { wxS( "apos" ), wxS( "'" ) } };
    wxString out;
    size_t   i = 0;

    while( i < aText.length() )
    {
        const size_t semi = aText[i] == '&' ? aText.find( ';', i ) : wxString::npos;

        if( semi != wxString::npos && semi - i <= 10 )
        {
            const wxString name = aText.Mid( i + 1, semi - i - 1 );
            unsigned long  code = 0;

            if( auto it = named.find( name ); it != named.end() )
            {
                out += it->second;
                i = semi + 1;
                continue;
            }

            if( name.StartsWith( wxS( "#x" ) ) || name.StartsWith( wxS( "#X" ) ) ? name.Mid( 2 ).ToULong( &code, 16 )
                : name.StartsWith( wxS( "#" ) )                                  ? name.Mid( 1 ).ToULong( &code, 10 )
                                                                                 : false )
            {
                out += wxUniChar( code );
                i = semi + 1;
                continue;
            }
        }

        out += aText[i++];
    }

    return out;
}


/// @brief The engine's trace without its HTML (the Inspect dialogs show it as a web page)
nlohmann::json traceLines( const wxString& aText )
{
    nlohmann::json   lines = nlohmann::json::array();
    wxRegEx          tags( wxS( "<[^>]*>" ) );
    wxStringTokenizer tok( aText, "\n" );

    while( tok.HasMoreTokens() )
    {
        wxString line = tok.GetNextToken();
        tags.ReplaceAll( &line, wxEmptyString );
        line = unescapeEntities( line );
        line.Trim().Trim( false );

        if( !line.IsEmpty() )
            lines.push_back( str( line ) );
    }

    return lines;
}


/// @brief Where a constraint's value comes from: custom rule, net class, board setup, keepout,
/// footprint, pad, zone, net tie; with the rule's name and condition
nlohmann::json sourceJson( const DRC_CONSTRAINT& aConstraint )
{
    const MINOPTMAX<int>& v = aConstraint.GetValue();
    const DRC_RULE*       rule = v.HasMin() && aConstraint.GetMinRule() ? aConstraint.GetMinRule()
                                 : v.HasOpt() && aConstraint.GetOptRule() ? aConstraint.GetOptRule()
                                 : v.HasMax() && aConstraint.GetMaxRule() ? aConstraint.GetMaxRule()
                                                                          : aConstraint.GetParentRule();
    nlohmann::json        s;

    if( rule )
    {
        switch( rule->GetImplicitSource() )
        {
        case DRC_IMPLICIT_SOURCE::NONE: s["type"] = "custom"; break;
        case DRC_IMPLICIT_SOURCE::BOARD_SETUP_CONSTRAINT:
        case DRC_IMPLICIT_SOURCE::BARCODE_DEFAULTS: s["type"] = "board_setup"; break;
        case DRC_IMPLICIT_SOURCE::KEEPOUT: s["type"] = "keepout"; break;
        case DRC_IMPLICIT_SOURCE::NET_CLASS: s["type"] = "netclass"; break;
        case DRC_IMPLICIT_SOURCE::TUNING_PROFILE: s["type"] = "tuning_profile"; break;
        }

        s["rule"] = str( rule->m_Name );

        if( !rule->m_ImplicitNetclass.IsEmpty() )
            s["netclass"] = str( rule->m_ImplicitNetclass );

        if( rule->GetImplicitSource() == DRC_IMPLICIT_SOURCE::KEEPOUT && rule->m_ImplicitItemId != niluuid )
            s["zone"] = str( rule->m_ImplicitItemId.AsString() );

        if( rule->m_Condition && !rule->m_Condition->GetExpression().IsEmpty() )
            s["condition"] = str( rule->m_Condition->GetExpression() );

        if( !rule->m_LayerSource.IsEmpty() )
            s["layer"] = str( rule->m_LayerSource );

        return s;
    }

    // set by the engine itself: an item's own override or a board minimum
    const wxString name = aConstraint.GetName();
    const wxString footprintPrefix = wxString::Format( _( "footprint %s" ), wxEmptyString );

    if( name == _( "pad" ) )
        s["type"] = "pad";
    else if( name.StartsWith( footprintPrefix ) )
        s = { { "type", "footprint" }, { "ref", str( name.Mid( footprintPrefix.length() ) ) } };
    else if( name == _( "zone" ) )
        s["type"] = "zone";
    else if( name == _( "net tie" ) )
        s["type"] = "net_tie";
    else if( name.StartsWith( _( "board minimum" ) ) )
        s["type"] = "board_setup";
    else
        s["type"] = name.IsEmpty() ? "none" : "item";

    s["rule"] = str( name );
    return s;
}


/// @brief A constraint's value and source; equal answers compare equal
nlohmann::json valueJson( const DRC_CONSTRAINT& aConstraint )
{
    const MINOPTMAX<int>& v = aConstraint.GetValue();
    nlohmann::json        j = { { "rule", str( aConstraint.GetName() ) }, { "source", sourceJson( aConstraint ) } };

    if( v.HasMin() )
        j["min_mm"] = toMm( v.Min() );

    if( v.HasOpt() )
        j["opt_mm"] = toMm( v.Opt() );

    if( v.HasMax() )
        j["max_mm"] = toMm( v.Max() );

    if( aConstraint.GetSeverity() == RPT_SEVERITY_IGNORE )
        j["severity"] = "ignore";

    return j;
}


KOPENAPI_RESULT h_pcb_rules_explain( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    auto resolve = [&]( const char* aKey ) -> BOARD_ITEM*
    {
        return aArgs.contains( aKey ) && aArgs[aKey].is_string()
                       ? KopenapiFindBoardItem( board, aArgs[aKey].get<std::string>() )
                       : nullptr;
    };

    if( !aArgs.contains( "a" ) )
        return KOPENAPI_RESULT::Error( 400, "a: the item (REF.PAD, REF, or a uuid / uuid prefix of a track, via, zone, drawing)" );

    BOARD_ITEM* a = resolve( "a" );
    BOARD_ITEM* b = resolve( "b" );

    if( !a )
        return KOPENAPI_RESULT::Error( 404, "a: no item " + aArgs["a"].dump() + " on the board" );

    if( aArgs.contains( "b" ) && !b )
        return KOPENAPI_RESULT::Error( 404, "b: no item " + aArgs["b"].dump() + " on the board" );

    std::vector<std::string> kinds;

    if( aArgs.contains( "constraints" ) )
    {
        for( const nlohmann::json& k : aArgs["constraints"] )
        {
            const std::string name = k.is_string() ? k.get<std::string>() : std::string();
            auto              it = constraintKinds().find( name );

            if( it == constraintKinds().end() )
            {
                std::string known;

                for( const auto& [n, kind] : constraintKinds() )
                    known += ( known.empty() ? "" : ", " ) + n;

                return KOPENAPI_RESULT::Error( 400, "constraints: unknown '" + name + "' (" + known + ")" );
            }

            if( it->second.pair && !b )
                return KOPENAPI_RESULT::Error( 400, "constraints: " + name + " is between two items: give b" );

            kinds.push_back( name );
        }
    }
    else
    {
        kinds = defaultKinds( a, b );
    }

    // the layer: given, else one both items are on (copper first)
    PCB_LAYER_ID layer = UNDEFINED_LAYER;

    if( aArgs.contains( "layer" ) )
    {
        const int id = board->GetLayerID( wxString::FromUTF8( aArgs.value( "layer", std::string() ) ) );

        if( id < 0 )
            return KOPENAPI_RESULT::Error( 400, "layer: no such layer on this board (pcb_layer_list)" );

        layer = (PCB_LAYER_ID) id;
    }
    else
    {
        LSET common = a->GetLayerSet() & ( b ? b->GetLayerSet() : a->GetLayerSet() ) & board->GetEnabledLayers();

        if( common.none() )
            common = a->GetLayerSet() & board->GetEnabledLayers();

        LSEQ copper = LSET( common & LSET::AllCuMask() ).Seq();
        LSEQ any = common.Seq();
        layer = !copper.empty() ? copper.front() : !any.empty() ? any.front() : a->GetLayer();
    }

    // the board's live engine (the one the guard, the router and pcb_drc evaluate) answers; a fresh
    // one from the rules on disk says whether the live one is current
    std::shared_ptr<DRC_ENGINE> live = board->GetDesignSettings().m_DRCEngine;
    DRC_ENGINE                  disk( board, &board->GetDesignSettings() );
    std::string                 compileError;

    try
    {
        disk.InitEngine( board->GetDesignRulesPath() );
    }
    catch( const PARSE_ERROR& error )
    {
        compileError = str( error.What() );

        try
        {
            disk.InitEngine( wxFileName() );
        }
        catch( const PARSE_ERROR& )
        {
        }
    }

    if( live && !live->RulesValid() )
        live.reset();

    DRC_ENGINE& engine = live ? *live : disk;

    for( ZONE* zone : board->Zones() )
        zone->CacheBoundingBox();

    for( FOOTPRINT* footprint : board->Footprints() )
    {
        for( ZONE* zone : footprint->Zones() )
            zone->CacheBoundingBox();

        footprint->BuildCourtyardCaches();
    }

    const BOARD_CONNECTED_ITEM* ac = dynamic_cast<const BOARD_CONNECTED_ITEM*>( a );
    const BOARD_CONNECTED_ITEM* bc = dynamic_cast<const BOARD_CONNECTED_ITEM*>( b );
    const bool sameNet = ac && bc && ac->GetNetCode() > 0 && ac->GetNetCode() == bc->GetNetCode();

    nlohmann::json out = { { "a", itemJson( a ) }, { "layer", str( board->GetLayerName( layer ) ) } };

    if( b )
        out["b"] = itemJson( b );

    out["engine"] = live ? "live" : "disk";

    if( !compileError.empty() )
        out["custom_rules_error"] = compileError;

    // the distance between them on the layer (copper edge to copper edge)
    std::optional<int> actual;

    if( b && a->IsOnLayer( layer ) && b->IsOnLayer( layer ) )
    {
        std::shared_ptr<SHAPE> sa = a->GetEffectiveShape( layer );
        std::shared_ptr<SHAPE> sb = b->GetEffectiveShape( layer );
        int                    d = 0;

        if( sa && sb )
        {
            d = sa->Collide( sb.get(), INT_MAX, &d ) ? d : INT_MAX;

            if( d != INT_MAX )
                actual = d;
        }

        if( actual )
            out["actual_mm"] = toMm( *actual );
    }

    nlohmann::json results = nlohmann::json::array();
    bool           agree = true;

    for( const std::string& name : kinds )
    {
        const KIND&        kind = constraintKinds().at( name );
        nlohmann::json     r = { { "constraint", name } };
        WX_STRING_REPORTER reporter;

        if( kind.type == CLEARANCE_CONSTRAINT && sameNet )
        {
            r["same_net"] = true;
            r["min_mm"] = 0.0;
            r["rule"] = "items of one net";
            r["source"] = { { "type", "same_net" } };
            results.push_back( r );
            continue;
        }

        DRC_CONSTRAINT        c = engine.EvalRules( kind.type, a, kind.pair ? b : nullptr, layer, &reporter );
        const MINOPTMAX<int>& v = c.GetValue();

        r.update( valueJson( c ) );

        if( live )
        {
            const nlohmann::json fromDisk = valueJson( disk.EvalRules( kind.type, a, kind.pair ? b : nullptr, layer, nullptr ) );
            const bool           same = fromDisk == valueJson( c );

            r["matches_disk"] = same;
            agree &= same;

            if( !same )
                r["disk"] = fromDisk;
        }

        // DRC does not test an ignored constraint nor a clearance below zero
        const bool clearanceKind = kind.pair && kind.type != COURTYARD_CLEARANCE_CONSTRAINT;

        if( c.GetSeverity() == RPT_SEVERITY_IGNORE )
        {
            r["tested"] = false;
            r["untested_reason"] = "severity ignore";
        }
        else if( clearanceKind && v.HasMin() && v.Min() < 0 )
        {
            r["tested"] = false;
            r["untested_reason"] = "negative clearance: DRC does not test it";
        }
        else if( kind.pair && actual && v.HasMin()
                 && ( kind.type == CLEARANCE_CONSTRAINT || kind.type == PHYSICAL_CLEARANCE_CONSTRAINT ) )
        {
            r["ok"] = *actual >= v.Min();
        }

        if( aArgs.value( "trace", true ) )
            r["trace"] = traceLines( reporter.GetMessages() );

        results.push_back( r );
    }

    if( live )
        out["live_matches_disk"] = agree;

    out["constraints"] = results;
    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_rules_explain",
                   "Why a constraint has its value, as the editor's Inspect > Clearance Resolution / Constraints "
                   "Resolution: between two items a and b (REF.PAD, REF, or a uuid / prefix of a track, via, zone, "
                   "drawing) - clearance, physical_clearance, hole_clearance, hole_to_hole, edge_clearance, "
                   "courtyard_clearance, silk_clearance - or for one item a - track_width, via_diameter, "
                   "annular_width, hole_size, diff_pair_gap, length, skew, thermal_relief_gap, thermal_spoke_width, "
                   "solder_mask_expansion, text_height, text_thickness; default: what the dialog shows for these "
                   "items. Per constraint min / opt / max mm, the rule that sets it and its source (type custom, "
                   "netclass, board_setup, keepout, footprint, pad, zone, net_tie, same_net; the rule's condition, "
                   "net class, keepout zone), tested: false when DRC skips it (severity ignore, a negative "
                   "clearance), ok (the items' actual distance on the layer against the clearance) and the rule "
                   "engine's trace (trace: false leaves it out); layer: default one both are on, copper first. "
                   "Answered by the board's live rule engine (the one the guard, the router and pcb_drc use); "
                   "matches_disk / live_matches_disk say whether the rules on disk give the same (else disk: "
                   "their answer; pcb_drc reads them in)",
                   R"json({"type":"object","required":["a"],"properties":{
                        "a":{"type":"string"},"b":{"type":"string"},
                        "constraints":{"type":"array","items":{"type":"string"}},
                        "layer":{"type":"string"},
                        "trace":{"type":"boolean","default":true}}})json"_json,
                   false, h_pcb_rules_explain );

KOPENAPI_MARK_CONCURRENT_READS( "pcb_rules_explain" );
