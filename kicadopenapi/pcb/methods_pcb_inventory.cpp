/*
 * kicadopenapi PCB inventories and cards (DESIGN-ANALYSIS-API.md §3–§4):
 * pcb_footprint_list, pcb_net_list, pcb_net_get, pcb_footprint_get.
 *
 * All values in mm (KiCad coordinates: origin top-left, y grows downwards).  Lists are
 * filtered and paginated; cards carry facts, plus heuristic `inferred` data with its basis.
 */
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <kicadopenapi_util.h>
#include <netclass.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_field.h>
#include <pcb_track.h>
#include <ratsnest/ratsnest_data.h>
#include <zone.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>


static double mm( double aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1e6 ) / 1e6;
}


static std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


static const char* footprintType( const FOOTPRINT* aFp )
{
    if( aFp->GetAttributes() & FP_THROUGH_HOLE )
        return "tht";

    if( aFp->GetAttributes() & FP_SMD )
        return "smd";

    return "unspecified";
}


static const char* padAttribute( const PAD* aPad )
{
    switch( aPad->GetAttribute() )
    {
    case PAD_ATTRIB::PTH:  return "pth";
    case PAD_ATTRIB::SMD:  return "smd";
    case PAD_ATTRIB::CONN: return "connector";
    case PAD_ATTRIB::NPTH: return "npth";
    default:               return "unknown";
    }
}


/// Connector-like reference designators (where nets leave the board)
static bool isConnectorRef( const std::string& aRef )
{
    for( const char* prefix : { "J", "P", "X", "CN", "XS", "XP", "CON" } )
    {
        const size_t n = std::strlen( prefix );

        if( aRef.size() > n && aRef.compare( 0, n, prefix ) == 0 && std::isdigit( (unsigned char) aRef[n] ) )
            return true;
    }

    return false;
}


static nlohmann::json naturalSorted( const std::set<std::string>& aItems )
{
    std::vector<std::string> v( aItems.begin(), aItems.end() );
    std::sort( v.begin(), v.end(), KopenapiNaturalLess );
    return v;
}


static std::string upper( std::string aText )
{
    for( char& c : aText )
        c = static_cast<char>( std::toupper( static_cast<unsigned char>( c ) ) );

    return aText;
}


/// Per-net aggregates computed in one pass over the board
struct NET_INFO_AGG
{
    double                         length = 0.0;
    std::map<std::string, double>  lengthByLayer;
    int                            segments = 0;
    int                            vias = 0;
    int                            pads = 0;
    std::set<std::string>          footprints;
    std::vector<const ZONE*>       zones;
};


static std::map<int, NET_INFO_AGG> aggregate( BOARD* aBoard )
{
    std::map<int, NET_INFO_AGG> agg;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        NET_INFO_AGG& a = agg[track->GetNetCode()];

        if( track->Type() == PCB_VIA_T )
        {
            a.vias++;
            continue;
        }

        a.segments++;
        a.length += track->GetLength();
        a.lengthByLayer[str( aBoard->GetLayerName( track->GetLayer() ) )] += track->GetLength();
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            NET_INFO_AGG& a = agg[pad->GetNetCode()];
            a.pads++;
            a.footprints.insert( str( fp->GetReference() ) );
        }
    }

    for( ZONE* zone : aBoard->Zones() )
    {
        if( !zone->GetIsRuleArea() )
            agg[zone->GetNetCode()].zones.push_back( zone );
    }

    return agg;
}


static int unrouted( BOARD* aBoard, int aNetCode )
{
    std::shared_ptr<CONNECTIVITY_DATA> conn = aBoard->GetConnectivity();
    RN_NET*                            rn = conn ? conn->GetRatsnestForNet( aNetCode ) : nullptr;
    return rn ? static_cast<int>( rn->GetEdges().size() ) : 0;
}


static std::string netClassName( const NETINFO_ITEM* aNet )
{
    const NETCLASS* nc = aNet->GetNetClass();
    return nc ? str( nc->GetHumanReadableName() ) : std::string( "Default" );
}


/// Heuristic role of a net; every conclusion carries its basis
static nlohmann::json inferRole( BOARD* aBoard, const NETINFO_ITEM* aNet, const NET_INFO_AGG& aAgg,
                                 const std::vector<const PAD*>& aPads )
{
    const std::string name = upper( str( aNet->GetNetname() ) );
    const std::string bare = name.substr( name.find_last_of( '/' ) == std::string::npos ? 0 : name.find_last_of( '/' ) + 1 );

    std::string    role = "signal";
    nlohmann::json basis = nlohmann::json::array();

    auto has = [&]( const char* aToken ) { return bare.find( aToken ) != std::string::npos; };

    if( bare == "GND" || has( "GND" ) || bare == "VSS" || bare == "0V" || has( "AGND" ) || has( "DGND" ) )
    {
        role = "ground";
        basis.push_back( "net name " + bare );
    }
    else if( !bare.empty() && ( bare[0] == '+' || has( "VCC" ) || has( "VDD" ) || has( "VBAT" ) || has( "VIN" )
                                || ( bare.rfind( "V", 0 ) == 0 && bare.size() > 1 && std::isdigit( (unsigned char) bare[1] ) ) ) )
    {
        role = "power";
        basis.push_back( "net name " + bare );
    }
    else if( has( "CLK" ) || has( "CLOCK" ) || has( "XTAL" ) || has( "OSC" ) )
    {
        role = "clock";
        basis.push_back( "net name " + bare );
    }
    else if( has( "RESET" ) || has( "RST" ) )
    {
        role = "reset";
        basis.push_back( "net name " + bare );
    }

    for( const PAD* pad : aPads )
    {
        const std::string type = str( pad->GetPinType() );

        if( type.rfind( "power", 0 ) == 0 )
        {
            basis.push_back( "pin " + str( pad->GetParentFootprint()->GetReference() ) + "." + str( pad->GetNumber() )
                             + " is " + type );

            if( role == "signal" )
                role = "power";

            break;
        }
    }

    if( !aAgg.zones.empty() )
        basis.push_back( "has " + std::to_string( aAgg.zones.size() ) + " copper zone(s)" );

    nlohmann::json out = { { "role", role }, { "basis", basis } };

    // Differential pair: NAME_P/NAME_N or NAME+/NAME-
    for( const auto& [suffix, other] : std::vector<std::pair<std::string, std::string>>{
                 { "_P", "_N" }, { "_N", "_P" }, { "+", "-" }, { "-", "+" } } )
    {
        const std::string full = str( aNet->GetNetname() );

        if( full.size() > suffix.size() && upper( full ).compare( full.size() - suffix.size(), suffix.size(), suffix ) == 0 )
        {
            const std::string candidate = full.substr( 0, full.size() - suffix.size() ) + other;

            if( aBoard->FindNet( wxString::FromUTF8( candidate ) ) )
            {
                out["role"] = "diff_pair";
                out["counterpart"] = candidate;
                out["basis"].push_back( "counterpart net " + candidate + " exists" );
                break;
            }
        }
    }

    std::set<std::string> connectorSet;

    for( const std::string& ref : aAgg.footprints )
    {
        if( isConnectorRef( ref ) )
            connectorSet.insert( ref );
    }

    nlohmann::json connectors = naturalSorted( connectorSet );

    if( !connectors.empty() )
    {
        out["interface"] = connectors;
        out["basis"].push_back( "reaches connector(s)" );
    }

    return out;
}


static nlohmann::json padJson( const PAD* aPad, bool aWithFootprint )
{
    nlohmann::json pad = { { "number", str( aPad->GetNumber() ) },
                           { "net", str( aPad->GetNetname() ) },
                           { "function", str( aPad->GetPinFunction() ) },
                           { "pin_type", str( aPad->GetPinType() ) },
                           { "attribute", padAttribute( aPad ) },
                           { "x_mm", mm( aPad->GetPosition().x ) },
                           { "y_mm", mm( aPad->GetPosition().y ) } };

    const PCB_LAYER_ID layer = aPad->GetPrincipalLayer();
    pad["size_mm"] = { mm( aPad->GetSize( layer ).x ), mm( aPad->GetSize( layer ).y ) };

    if( aPad->HasHole() )
        pad["drill_mm"] = { mm( aPad->GetDrillSize().x ), mm( aPad->GetDrillSize().y ) };

    if( aWithFootprint )
    {
        const FOOTPRINT*  fp = aPad->GetParentFootprint();
        const std::string ref = str( fp->GetReference() );
        pad["ref"] = ref;
        pad["value"] = str( fp->GetValue() );
        pad["footprint_uuid"] = str( fp->m_Uuid.AsString() );

        // Board-only footprints often have no reference: no REF.PAD handle for them
        pad["pin"] = ref.empty() ? nlohmann::json() : nlohmann::json( ref + "." + str( aPad->GetNumber() ) );
    }

    return pad;
}


/// Brief row for lists and the footprint card header
static nlohmann::json footprintRow( const FOOTPRINT* aFp )
{
    return { { "ref", str( aFp->GetReference() ) },
             { "value", str( aFp->GetValue() ) },
             { "footprint", str( aFp->GetFPIDAsString() ) },
             { "side", aFp->GetSide() == B_Cu ? "back" : "front" },
             { "x_mm", mm( aFp->GetPosition().x ) },
             { "y_mm", mm( aFp->GetPosition().y ) },
             { "rotation_deg", aFp->GetOrientationDegrees() },
             { "type", footprintType( aFp ) },
             { "pads", aFp->Pads().size() },
             { "dnp", aFp->IsDNP() },
             { "uuid", str( aFp->m_Uuid.AsString() ) } };
}


/// Footprints with this reference (references are not guaranteed unique, may be empty)
static std::vector<FOOTPRINT*> footprintsByRef( BOARD* aBoard, const std::string& aRef )
{
    std::vector<FOOTPRINT*> found;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( str( fp->GetReference() ) == aRef )
            found.push_back( fp );
    }

    return found;
}


static KOPENAPI_RESULT h_pcb_footprint_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const std::string refGlob = aArgs.value( "ref", std::string() );
    const std::string valueGlob = aArgs.value( "value", std::string() );
    const std::string fpGlob = aArgs.value( "footprint", std::string() );
    const std::string side = aArgs.value( "side", std::string() );
    const std::string type = aArgs.value( "type", std::string() );

    std::vector<nlohmann::json> rows;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        nlohmann::json row = footprintRow( fp );

        if( !KopenapiGlob( refGlob, row["ref"] ) || !KopenapiGlob( valueGlob, row["value"] )
            || !KopenapiGlob( fpGlob, row["footprint"] ) || ( !side.empty() && row["side"] != side )
            || ( !type.empty() && row["type"] != type ) )
        {
            continue;
        }

        rows.push_back( std::move( row ) );
    }

    std::sort( rows.begin(), rows.end(),
               []( const nlohmann::json& a, const nlohmann::json& b )
               { return KopenapiNaturalLess( a["ref"], b["ref"] ); } );

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


static KOPENAPI_RESULT h_pcb_net_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                      board = context->GetBoard();
    std::map<int, NET_INFO_AGG> agg = aggregate( board );
    const std::string           nameGlob = aArgs.value( "name", std::string() );
    const std::string           classGlob = aArgs.value( "class", std::string() );
    const int                   minPads = aArgs.value( "min_pads", 0 );
    const bool                  unroutedOnly = aArgs.value( "unrouted_only", false );

    std::vector<nlohmann::json> rows;

    for( NETINFO_ITEM* net : board->GetNetInfo() )
    {
        if( net->GetNetCode() <= 0 )
            continue;

        const NET_INFO_AGG& a = agg[net->GetNetCode()];
        const std::string   name = str( net->GetNetname() );
        const std::string   cls = netClassName( net );
        const int           open = unrouted( board, net->GetNetCode() );

        if( !KopenapiGlob( nameGlob, name ) || !KopenapiGlob( classGlob, cls ) || a.pads < minPads
            || ( unroutedOnly && open == 0 ) )
        {
            continue;
        }

        rows.push_back( { { "name", name },
                          { "code", net->GetNetCode() },
                          { "class", cls },
                          { "pads", a.pads },
                          { "footprints", a.footprints.size() },
                          { "length_mm", mm( a.length ) },
                          { "vias", a.vias },
                          { "zones", a.zones.size() },
                          { "unrouted", open } } );
    }

    std::sort( rows.begin(), rows.end(),
               []( const nlohmann::json& a, const nlohmann::json& b )
               { return KopenapiNaturalLess( a["name"], b["name"] ); } );

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


static KOPENAPI_RESULT h_pcb_net_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*        board = context->GetBoard();
    NETINFO_ITEM* net = nullptr;

    if( aArgs.contains( "code" ) && aArgs["code"].is_number_integer() )
        net = board->FindNet( aArgs["code"].get<int>() );
    else if( aArgs.contains( "name" ) && aArgs["name"].is_string() )
        net = board->FindNet( wxString::FromUTF8( aArgs["name"].get<std::string>() ) );
    else if( aArgs.contains( "pin" ) && aArgs["pin"].is_string() )
    {
        // "REF.PAD"
        const std::string pin = aArgs["pin"].get<std::string>();
        const size_t      dot = pin.rfind( '.' );

        if( dot != std::string::npos && dot > 0 )
        {
            for( FOOTPRINT* fp : footprintsByRef( board, pin.substr( 0, dot ) ) )
            {
                if( PAD* pad = fp->FindPadByNumber( wxString::FromUTF8( pin.substr( dot + 1 ) ) ) )
                {
                    net = pad->GetNet();
                    break;
                }
            }
        }
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "give 'name', 'code' or 'pin' (REF.PAD)" );
    }

    if( !net || net->GetNetCode() <= 0 )
        return KOPENAPI_RESULT::Error( 404, "net not found" );

    std::map<int, NET_INFO_AGG> agg = aggregate( board );
    const NET_INFO_AGG&         a = agg[net->GetNetCode()];

    std::vector<const PAD*> pads;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() == net->GetNetCode() )
                pads.push_back( pad );
        }
    }

    std::sort( pads.begin(), pads.end(),
               []( const PAD* x, const PAD* y )
               {
                   const std::string rx = str( x->GetParentFootprint()->GetReference() );
                   const std::string ry = str( y->GetParentFootprint()->GetReference() );

                   if( rx != ry )
                       return KopenapiNaturalLess( rx, ry );

                   return KopenapiNaturalLess( str( x->GetNumber() ), str( y->GetNumber() ) );
               } );

    nlohmann::json padList = nlohmann::json::array();

    for( const PAD* pad : pads )
        padList.push_back( padJson( pad, true ) );

    nlohmann::json byLayer = nlohmann::json::object();

    for( const auto& [layer, length] : a.lengthByLayer )
        byLayer[layer] = mm( length );

    nlohmann::json zones = nlohmann::json::array();

    for( const ZONE* zone : a.zones )
    {
        nlohmann::json layers = nlohmann::json::array();

        for( PCB_LAYER_ID layer : zone->GetLayerSet().Seq() )
            layers.push_back( str( board->GetLayerName( layer ) ) );

        zones.push_back( { { "layers", layers }, { "filled", zone->IsFilled() }, { "name", str( zone->GetZoneName() ) } } );
    }

    const NETCLASS* nc = net->GetNetClass();
    nlohmann::json  netClass = { { "name", netClassName( net ) } };

    if( nc )
    {
        if( nc->GetClearance() > 0 )
            netClass["clearance_mm"] = mm( nc->GetClearance() );

        if( nc->GetTrackWidth() > 0 )
            netClass["track_width_mm"] = mm( nc->GetTrackWidth() );

        if( nc->GetViaDiameter() > 0 )
            netClass["via_diameter_mm"] = mm( nc->GetViaDiameter() );
    }

    return KOPENAPI_RESULT::Ok( { { "name", str( net->GetNetname() ) },
                                  { "code", net->GetNetCode() },
                                  { "class", netClass },
                                  { "pads", padList },
                                  { "footprints", naturalSorted( a.footprints ) },
                                  { "routing",
                                    { { "length_mm", mm( a.length ) },
                                      { "length_by_layer_mm", byLayer },
                                      { "segments", a.segments },
                                      { "vias", a.vias },
                                      { "unrouted", unrouted( board, net->GetNetCode() ) } } },
                                  { "zones", zones },
                                  { "inferred", inferRole( board, net, a, pads ) } } );
}


static KOPENAPI_RESULT h_pcb_footprint_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    FOOTPRINT* fp = nullptr;

    if( aArgs.contains( "uuid" ) && aArgs["uuid"].is_string() )
    {
        const std::string uuid = aArgs["uuid"].get<std::string>();

        for( FOOTPRINT* candidate : board->Footprints() )
        {
            if( str( candidate->m_Uuid.AsString() ) == uuid )
                fp = candidate;
        }
    }
    else if( aArgs.contains( "ref" ) && aArgs["ref"].is_string() && !aArgs["ref"].get<std::string>().empty() )
    {
        std::vector<FOOTPRINT*> found = footprintsByRef( board, aArgs["ref"].get<std::string>() );

        if( found.size() > 1 )
        {
            nlohmann::json uuids = nlohmann::json::array();

            for( FOOTPRINT* candidate : found )
                uuids.push_back( str( candidate->m_Uuid.AsString() ) );

            KOPENAPI_RESULT ambiguous = KOPENAPI_RESULT::Error( 409, "reference is not unique; use uuid" );
            ambiguous.body["candidates"] = uuids;
            return ambiguous;
        }

        fp = found.empty() ? nullptr : found.front();
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "give 'ref' (non-empty) or 'uuid'" );
    }

    if( !fp )
        return KOPENAPI_RESULT::Error( 404, "footprint not found" );

    nlohmann::json card = footprintRow( fp );

    nlohmann::json fields = nlohmann::json::object();

    for( const PCB_FIELD* field : fp->GetFields() )
        fields[str( field->GetName() )] = str( field->GetText() );

    card["fields"] = fields;
    card["description"] = str( fp->GetLibDescription() );
    card["attributes"] = { { "excluded_from_bom", fp->IsExcludedFromBOM() },
                           { "excluded_from_pos", ( fp->GetAttributes() & FP_EXCLUDE_FROM_POS_FILES ) != 0 },
                           { "board_only", fp->IsBoardOnly() } };

    const BOX2I bbox = fp->GetBoundingBox( false );
    card["bbox_mm"] = { { "x", mm( bbox.GetX() ) }, { "y", mm( bbox.GetY() ) },
                        { "width", mm( bbox.GetWidth() ) }, { "height", mm( bbox.GetHeight() ) } };

    std::vector<const PAD*> pads( fp->Pads().begin(), fp->Pads().end() );
    std::sort( pads.begin(), pads.end(),
               []( const PAD* x, const PAD* y ) { return KopenapiNaturalLess( str( x->GetNumber() ), str( y->GetNumber() ) ); } );

    nlohmann::json padList = nlohmann::json::array();
    std::set<std::string> nets;

    for( const PAD* pad : pads )
    {
        padList.push_back( padJson( pad, false ) );

        if( pad->GetNetCode() > 0 )
            nets.insert( str( pad->GetNetname() ) );
    }

    card["pad_list"] = padList;
    card["nets"] = naturalSorted( nets );

    nlohmann::json models = nlohmann::json::array();

    for( const FP_3DMODEL& model : fp->Models() )
        models.push_back( { { "file", str( model.m_Filename ) }, { "show", model.m_Show } } );

    card["models"] = models;
    return KOPENAPI_RESULT::Ok( card );
}


static nlohmann::json withPaging( nlohmann::json aProperties )
{
    for( const auto& [key, value] : KopenapiPageSchema().items() )
        aProperties[key] = value;

    return { { "type", "object" }, { "properties", aProperties } };
}


KOPENAPI_REGISTER( "pcb_footprint_list",
                   "List board footprints/components (ref, value, footprint, side, position, THT/SMD, DNP, uuid); "
                   "filter by ref/value/footprint glob, side, type; ref '?*' skips unnamed board-only "
                   "items; paginated, natural ref order",
                   withPaging( R"json({
                        "ref":{"type":"string","description":"glob, e.g. U* or R1?"},
                        "value":{"type":"string","description":"glob"},
                        "footprint":{"type":"string","description":"library id glob"},
                        "side":{"type":"string","enum":["front","back"]},
                        "type":{"type":"string","enum":["tht","smd","unspecified"]}})json"_json ),
                   false, h_pcb_footprint_list );

KOPENAPI_REGISTER( "pcb_net_list",
                   "List board nets with pad/footprint counts, routed length, vias, zones, unrouted "
                   "connections, net class; filter by name/class glob, min_pads, unrouted_only; paginated",
                   withPaging( R"json({
                        "name":{"type":"string","description":"glob, e.g. *CLK* or /CPU/D?"},
                        "class":{"type":"string","description":"net class glob"},
                        "min_pads":{"type":"integer","default":0},
                        "unrouted_only":{"type":"boolean","default":false}})json"_json ),
                   false, h_pcb_net_list );

KOPENAPI_REGISTER( "pcb_net_get",
                   "Net card: every pad (ref.pin, function, pin type), footprints, net class rules, routed "
                   "length per layer, vias, zones, unrouted connections, inferred role (power/ground/clock/"
                   "reset/diff_pair/interface) with basis; look up by name, code or pin REF.PAD",
                   R"json({"type":"object","properties":{
                        "name":{"type":"string"},
                        "code":{"type":"integer"},
                        "pin":{"type":"string","description":"REF.PAD, e.g. U3.14"}}})json"_json,
                   false, h_pcb_net_get );

KOPENAPI_REGISTER( "pcb_footprint_get",
                   "Footprint card: fields, library description, position/side/rotation, attributes, "
                   "bounding box, every pad with its net/function/size/drill, nets, 3D models",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string","description":"reference designator, e.g. U3 (must be unique)"},
                        "uuid":{"type":"string","description":"footprint uuid from pcb_footprint_list"}}})json"_json,
                   false, h_pcb_footprint_get );
