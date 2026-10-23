/// @file methods_pcb_signal.cpp
/// @brief kicadopenapi pcb_stackup_get / pcb_stackup_set / pcb_signal_check: the board stack-up with each copper
/// layer's reference planes, line impedance and delay, and the digital signal checks of
/// docs/pcb/routing-rules-digital-hsdd.md (DG-n) that the board's geometry and the nets' edge data decide.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <board_stackup_manager/board_stackup.h>
#include <footprint.h>
#include <geometry/seg.h>
#include <geometry/shape_poly_set.h>
#include <kicadopenapi_util.h>
#include <netinfo.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_track.h>
#include <project/net_settings.h>
#include <string_utils.h>
#include <transline_calculations/microstrip.h>
#include <transline_calculations/stripline.h>
#include <zone.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>


namespace
{

/// Copper conductivity the KiCad transmission line calculator uses (S/m)
constexpr double SIGMA_CU = 1.0 / 1.72e-8;

/// A copper layer is a plane when one net's fill covers at least this share of the board (ours)
constexpr double PLANE_MIN_FRACTION = 0.5;

/// Delay of light in vacuum, ps per mm
constexpr double PS_PER_MM_C0 = 3.33564;


struct FAMILY
{
    const char* name;
    double      trPs;       ///< fastest 10-90 % edge (ours, typical fast corner)
    double      routOhm;    ///< 0: not given
    double      swingV;
    double      nmV;        ///< 0: not given
};


/// Defaults of docs/pcb/routing-rules-digital-hsdd.md §0, used only when the net gives nothing
const FAMILY FAMILIES[] = {
    { "hc", 3000, 50, 5.0, 0 },          { "ahc", 1500, 35, 3.3, 0 },        { "ac", 1000, 30, 5.0, 0 },
    { "f", 1000, 30, 3.4, 0 },           { "lvcmos33", 800, 25, 3.3, 0.4 },  { "lvcmos18", 500, 30, 1.8, 0.18 },
    { "mcu_fast", 1000, 40, 3.3, 0 },    { "mcu_slow", 5000, 40, 3.3, 0 },   { "fpga_fast", 500, 35, 3.3, 0 },
    { "fpga_slow", 2000, 35, 3.3, 0 },   { "sstl", 300, 40, 1.2, 0 },        { "lvds", 300, 0, 0.35, 0.1 },
    { "hcsl", 300, 0, 0.8, 0.1 },        { "cml", 300, 0, 0.8, 0.1 },        { "usb2_hs", 500, 45, 0.4, 0 } };


const FAMILY* family( const std::string& aName )
{
    for( const FAMILY& f : FAMILIES )
    {
        if( aName == f.name )
            return &f;
    }

    return nullptr;
}


std::string familyNames()
{
    std::string out;

    for( const FAMILY& f : FAMILIES )
        out += ( out.empty() ? "" : ", " ) + std::string( f.name );

    return out;
}


const std::set<std::string> TERMINATIONS = { "none", "series", "end", "thevenin", "rc", "on_die", "both" };


std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( double aIU )
{
    return aIU / pcbIUScale.IU_PER_MM;
}


double round3( double aValue )
{
    return std::round( aValue * 1000.0 ) / 1000.0;
}


std::string fmt( double aValue, int aDigits = 2 )
{
    char buf[32];
    std::snprintf( buf, sizeof( buf ), "%.*f", aDigits, aValue );
    return buf;
}


std::string refPrefix( const std::string& aRef )
{
    size_t n = 0;

    while( n < aRef.size() && std::isalpha( static_cast<unsigned char>( aRef[n] ) ) )
        ++n;

    std::string out = aRef.substr( 0, n );

    for( char& c : out )
        c = static_cast<char>( std::toupper( static_cast<unsigned char>( c ) ) );

    return out;
}


bool isPassive( const std::string& aRef )
{
    static const std::set<std::string> passive = { "R", "RN", "RA", "C", "CN", "L", "FB", "F", "JP", "TP", "H", "MH",
                                                   "FID", "NT" };
    return passive.count( refPrefix( aRef ) ) > 0;
}


bool isGround( const std::string& aRole )
{
    return aRole == "ground";
}


bool isRail( const std::string& aRole )
{
    return aRole == "power" || aRole == "power_hc";
}


/// digital nets: analog, switching and fixed nets have their own rules
bool isSignalRole( const std::string& aRole )
{
    return aRole == "signal" || aRole == "clock" || aRole == "bus" || aRole == "matched" || aRole == "pair";
}


/// @brief Resistance from a footprint value ("33", "33R", "4k7", "49.9", "1K", "100R 1%"); < 0 when not a value
double ohms( const std::string& aValue )
{
    std::string head, tail;
    double      mult = 1.0;
    bool        letter = false;

    for( char c : aValue )
    {
        if( std::isdigit( static_cast<unsigned char>( c ) ) || c == '.' )
        {
            ( letter ? tail : head ) += c;
            continue;
        }

        if( letter || head.empty() )
            break;

        if( c == 'R' || c == 'r' )
            mult = 1.0;
        else if( c == 'k' || c == 'K' )
            mult = 1e3;
        else if( c == 'M' )
            mult = 1e6;
        else
            break;

        letter = true;
    }

    if( head.empty() )
        return -1;

    try
    {
        return std::stod( tail.empty() ? head : head + "." + tail ) * mult;
    }
    catch( ... )
    {
        return -1;
    }
}


// ------------------------------------------------------------------------------------------------- stack-up

struct DIELECTRIC
{
    double z0 = 0, z1 = 0;  ///< top and bottom, mm from the top of the stack
    double er = 4.5;
    double tand = 0.02;
};


struct COPPER
{
    PCB_LAYER_ID id = UNDEFINED_LAYER;
    std::string  name;
    double       zTop = 0;
    double       t = 0.035;
};


/// One copper layer as a line: its reference planes, model, impedance at a width and delay
struct LAYER_LINE
{
    std::string kind;           ///< plane, microstrip, stripline, embedded_microstrip, no_plane
    int         above = -1;     ///< copper index of the plane above, -1 none
    int         below = -1;
    double      hAbove = 0, hBelow = 0;
    double      er = 0;
    double      erEff = 0;
    double      delayPsMm = 0;
    double      z0 = 0;         ///< 0: not computed
    std::string note;
};


struct STACK
{
    bool                    fromBoard = false;
    double                  thickness = 0;
    std::vector<COPPER>     copper;           ///< top to bottom
    std::vector<DIELECTRIC> dielectrics;
    double                  maskT = 0, maskEr = 0, maskTand = 0;
    nlohmann::json          items = nlohmann::json::array();

    int Index( PCB_LAYER_ID aLayer ) const
    {
        for( size_t i = 0; i < copper.size(); ++i )
        {
            if( copper[i].id == aLayer )
                return static_cast<int>( i );
        }

        return -1;
    }

    /// thickness-weighted εr and tan δ of the dielectric between two depths
    std::pair<double, double> Between( double aZ0, double aZ1 ) const
    {
        double h = 0, er = 0, tand = 0;

        for( const DIELECTRIC& d : dielectrics )
        {
            const double lo = std::max( aZ0, d.z0 ), hi = std::min( aZ1, d.z1 );

            if( hi > lo )
            {
                h += hi - lo;
                er += d.er * ( hi - lo );
                tand += d.tand * ( hi - lo );
            }
        }

        return h > 0 ? std::make_pair( er / h, tand / h ) : std::make_pair( 4.5, 0.02 );
    }
};


std::string itemType( BOARD_STACKUP_ITEM_TYPE aType )
{
    switch( aType )
    {
    case BS_ITEM_TYPE_COPPER: return "copper";
    case BS_ITEM_TYPE_DIELECTRIC: return "dielectric";
    case BS_ITEM_TYPE_SOLDERPASTE: return "solder_paste";
    case BS_ITEM_TYPE_SOLDERMASK: return "solder_mask";
    case BS_ITEM_TYPE_SILKSCREEN: return "silkscreen";
    default: return "undefined";
    }
}


std::string itemName( const BOARD* aBoard, const BOARD_STACKUP_ITEM* aItem )
{
    if( aItem->GetType() == BS_ITEM_TYPE_DIELECTRIC )
        return "dielectric " + std::to_string( aItem->GetDielectricLayerId() );

    if( aItem->GetBrdLayerId() != UNDEFINED_LAYER )
        return str( aBoard->GetLayerName( aItem->GetBrdLayerId() ) );

    return str( aItem->GetLayerName() );
}


STACK readStack( BOARD* aBoard )
{
    STACK                 s;
    const BOARD_STACKUP   stackup = aBoard->GetStackupOrDefault();
    double                z = 0;
    bool                  topMask = true;

    s.fromBoard = aBoard->GetDesignSettings().m_HasStackup;
    s.thickness = toMm( aBoard->GetDesignSettings().GetBoardThickness() );

    for( const BOARD_STACKUP_ITEM* item : stackup.GetList() )
    {
        if( !item->IsEnabled() )
            continue;

        const std::string type = itemType( item->GetType() );

        for( int sub = 0; sub < std::max( 1, item->GetSublayersCount() ); ++sub )
        {
            const double t = item->IsThicknessEditable() || item->GetType() == BS_ITEM_TYPE_COPPER
                                     ? toMm( item->GetThickness( sub ) ) : 0.0;
            nlohmann::json row = { { "name", itemName( aBoard, item ) }, { "type", type }, { "thickness_mm", std::round( t * 1e4 ) / 1e4 },
                                   { "z_mm", std::round( z * 1e4 ) / 1e4 } };

            if( item->GetSublayersCount() > 1 )
                row["sublayer"] = sub;

            if( item->HasEpsilonRValue() )
                row["epsilon_r"] = item->GetEpsilonR( sub );

            if( item->HasLossTangentValue() )
                row["loss_tangent"] = item->GetLossTangent( sub );

            if( item->HasMaterialValue( sub ) )
                row["material"] = str( item->GetMaterial( sub ) );

            if( item->GetType() == BS_ITEM_TYPE_DIELECTRIC )
            {
                row["kind"] = str( item->GetTypeName() );
                s.dielectrics.push_back( { z, z + t, item->GetEpsilonR( sub ), item->GetLossTangent( sub ) } );
            }
            else if( item->GetType() == BS_ITEM_TYPE_COPPER && sub == 0 )
            {
                s.copper.push_back( { item->GetBrdLayerId(), itemName( aBoard, item ), z, t } );
                topMask = false;
            }
            else if( item->GetType() == BS_ITEM_TYPE_SOLDERMASK && topMask )
            {
                s.maskT = t;
                s.maskEr = item->GetEpsilonR( sub );
                s.maskTand = item->GetLossTangent( sub );
            }

            s.items.push_back( row );
            z += t;
        }
    }

    return s;
}


/// @brief Filled copper per net on each copper layer, as a share of the board area
std::vector<std::map<int, double>> planeShares( BOARD* aBoard, const STACK& aStack )
{
    std::vector<std::map<int, double>> shares( aStack.copper.size() );
    SHAPE_POLY_SET                     outline;
    double                             area = 0;

    if( aBoard->GetBoardPolygonOutlines( outline, true ) )
        area = outline.Area();

    if( area <= 0 )
        area = static_cast<double>( aBoard->GetBoardEdgesBoundingBox().GetArea() );

    if( area <= 0 )
        return shares;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone->GetIsRuleArea() || zone->GetNetCode() <= 0 || !zone->IsFilled() )
            continue;

        for( PCB_LAYER_ID layer : zone->GetLayerSet() )
        {
            const int i = aStack.Index( layer );

            if( i >= 0 && zone->HasFilledPolysForLayer( layer ) )
                shares[i][zone->GetNetCode()] += zone->GetFilledPolysList( layer )->Area() / area;
        }
    }

    return shares;
}


std::vector<int> planeNets( const std::vector<std::map<int, double>>& aShares )
{
    std::vector<int> nets( aShares.size(), 0 );

    for( size_t i = 0; i < aShares.size(); ++i )
    {
        double best = 0;

        for( const auto& [net, share] : aShares[i] )
        {
            if( share >= PLANE_MIN_FRACTION && share > best )
            {
                best = share;
                nets[i] = net;
            }
        }
    }

    return nets;
}


/// @brief Impedance and delay of a track of aWidthMm on each copper layer (KiCad's transmission line calculator:
/// microstrip with the solder mask over it, stripline between two planes)
std::vector<LAYER_LINE> layerLines( const STACK& aStack, const std::vector<int>& aPlaneNets, double aWidthMm,
                                    double aFreqHz )
{
    using TCP = TRANSLINE_PARAMETERS;
    const int               n = static_cast<int>( aStack.copper.size() );
    std::vector<LAYER_LINE> out( n );
    const double            skin = 1.0 / std::sqrt( M_PI * aFreqHz * 4e-7 * M_PI * SIGMA_CU );

    for( int i = 0; i < n; ++i )
    {
        LAYER_LINE&   line = out[i];
        const COPPER& cu = aStack.copper[i];

        if( aPlaneNets[i] > 0 )
        {
            line.kind = "plane";
            continue;
        }

        for( int j = i - 1; j >= 0 && line.above < 0; --j )
        {
            if( aPlaneNets[j] > 0 )
            {
                line.above = j;
                line.hAbove = cu.zTop - ( aStack.copper[j].zTop + aStack.copper[j].t );
            }
        }

        for( int j = i + 1; j < n && line.below < 0; ++j )
        {
            if( aPlaneNets[j] > 0 )
            {
                line.below = j;
                line.hBelow = aStack.copper[j].zTop - ( cu.zTop + cu.t );
            }
        }

        const bool outer = i == 0 || i == n - 1;

        if( line.above >= 0 && line.below >= 0 )
        {
            const auto [er, tand] = aStack.Between( cu.zTop - line.hAbove, cu.zTop + cu.t + line.hBelow );
            STRIPLINE  calc;
            calc.SetParameter( TCP::EPSILONR, er );
            calc.SetParameter( TCP::TAND, tand );
            calc.SetParameter( TCP::T, cu.t / 1000 );
            calc.SetParameter( TCP::STRIPLINE_A, line.hAbove / 1000 );
            calc.SetParameter( TCP::H, ( line.hAbove + cu.t + line.hBelow ) / 1000 );
            calc.SetParameter( TCP::PHYS_WIDTH, aWidthMm / 1000 );
            calc.SetParameter( TCP::PHYS_LEN, 1.0 );
            calc.SetParameter( TCP::FREQUENCY, aFreqHz );
            calc.SetParameter( TCP::SKIN_DEPTH, skin );
            calc.SetParameter( TCP::SIGMA, SIGMA_CU );
            calc.SetParameter( TCP::MURC, 1.0 );
            calc.SetParameter( TCP::ANG_L, 1.0 );
            calc.Analyse();
            auto& r = calc.GetAnalysisResults();
            line.kind = "stripline";
            line.er = er;
            line.erEff = r[TCP::EPSILON_EFF].first > 0 ? r[TCP::EPSILON_EFF].first : er;
            line.z0 = r[TCP::Z0].second == TRANSLINE_STATUS::OK ? r[TCP::Z0].first : 0;
            line.delayPsMm = PS_PER_MM_C0 * std::sqrt( line.erEff );
        }
        else if( line.above >= 0 || line.below >= 0 )
        {
            const double h = line.above >= 0 ? line.hAbove : line.hBelow;
            const auto [er, tand] = line.above >= 0 ? aStack.Between( cu.zTop - h, cu.zTop )
                                                    : aStack.Between( cu.zTop + cu.t, cu.zTop + cu.t + h );
            MICROSTRIP calc;
            calc.SetParameter( TCP::EPSILONR, er );
            calc.SetParameter( TCP::TAND, tand );
            calc.SetParameter( TCP::H, h / 1000 );
            calc.SetParameter( TCP::H_T, 1e20 );
            calc.SetParameter( TCP::T, cu.t / 1000 );
            calc.SetParameter( TCP::PHYS_WIDTH, aWidthMm / 1000 );
            calc.SetParameter( TCP::PHYS_LEN, 1.0 );
            calc.SetParameter( TCP::FREQUENCY, aFreqHz );
            calc.SetParameter( TCP::SKIN_DEPTH, skin );
            calc.SetParameter( TCP::SIGMA, SIGMA_CU );
            calc.SetParameter( TCP::ROUGH, 0.0 );
            calc.SetParameter( TCP::MUR, 1.0 );
            calc.SetParameter( TCP::MURC, 1.0 );
            calc.SetParameter( TCP::ANG_L, 1.0 );

            if( outer && aStack.maskT > 0 && aStack.maskEr > 0 )
            {
                calc.SetParameter( TCP::SOLDERMASK_PRESENT, 1.0 );
                calc.SetParameter( TCP::SOLDERMASK_THICKNESS, aStack.maskT / 1000 );
                calc.SetParameter( TCP::SOLDERMASK_EPSILONR, aStack.maskEr );
                calc.SetParameter( TCP::SOLDERMASK_TAND, aStack.maskTand );
            }

            calc.Analyse();
            auto& r = calc.GetAnalysisResults();
            line.er = er;
            line.z0 = r[TCP::Z0].second == TRANSLINE_STATUS::OK ? r[TCP::Z0].first : 0;

            if( outer )
            {
                line.kind = "microstrip";
                line.erEff = r[TCP::EPSILON_EFF].first > 0 ? r[TCP::EPSILON_EFF].first : ( er + 1 ) / 2;
            }
            else
            {
                // dielectric on both sides: the field sees εr, the impedance is the microstrip's estimate
                line.kind = "embedded_microstrip";
                line.erEff = er;
                line.note = "inner layer with one plane: impedance estimated as a microstrip, delay from εr";
            }

            line.delayPsMm = PS_PER_MM_C0 * std::sqrt( line.erEff );
        }
        else
        {
            const auto [er, tand] = aStack.Between( 0, 1e9 );
            line.kind = "no_plane";
            line.er = er;
            line.erEff = outer ? ( er + 1 ) / 2 : er;
            line.delayPsMm = PS_PER_MM_C0 * std::sqrt( line.erEff );
            line.note = "no plane: no defined impedance, delay estimated from εr";
        }
    }

    return out;
}


nlohmann::json lineJson( const STACK& aStack, const std::vector<LAYER_LINE>& aLines, const std::vector<int>& aPlaneNets,
                         const std::vector<std::map<int, double>>& aShares, BOARD* aBoard, double aWidthMm )
{
    nlohmann::json rows = nlohmann::json::array();

    auto netName = [&]( int aCode )
    {
        NETINFO_ITEM* net = aBoard->GetNetInfo().GetNetItem( aCode );
        return net ? str( UnescapeString( net->GetNetname() ) ) : std::string();
    };

    for( size_t i = 0; i < aLines.size(); ++i )
    {
        const LAYER_LINE& l = aLines[i];
        nlohmann::json    row = { { "layer", aStack.copper[i].name }, { "kind", l.kind },
                                  { "copper_mm", std::round( aStack.copper[i].t * 1e4 ) / 1e4 } };
        nlohmann::json    fills = nlohmann::json::array();

        for( const auto& [net, share] : aShares[i] )
            fills.push_back( { { "net", netName( net ) }, { "share", std::round( share * 1000 ) / 1000 } } );

        if( !fills.empty() )
            row["fills"] = fills;

        if( l.kind == "plane" )
        {
            row["plane_net"] = netName( aPlaneNets[i] );
            rows.push_back( row );
            continue;
        }

        if( l.above >= 0 )
            row["ref_above"] = { { "layer", aStack.copper[l.above].name }, { "net", netName( aPlaneNets[l.above] ) },
                                 { "h_mm", round3( l.hAbove ) } };

        if( l.below >= 0 )
            row["ref_below"] = { { "layer", aStack.copper[l.below].name }, { "net", netName( aPlaneNets[l.below] ) },
                                 { "h_mm", round3( l.hBelow ) } };

        row["epsilon_r"] = round3( l.er );
        row["epsilon_eff"] = round3( l.erEff );
        row["delay_ps_per_mm"] = round3( l.delayPsMm );
        row["width_mm"] = aWidthMm;

        if( l.z0 > 0 )
            row["z0_ohm"] = std::round( l.z0 * 10 ) / 10;

        if( !l.note.empty() )
            row["note"] = l.note;

        rows.push_back( row );
    }

    return rows;
}


double defaultWidth( BOARD* aBoard )
{
    std::shared_ptr<NET_SETTINGS> net = aBoard->GetDesignSettings().m_NetSettings;

    if( net && net->GetDefaultNetclass() && net->GetDefaultNetclass()->HasTrackWidth() )
        return toMm( net->GetDefaultNetclass()->GetTrackWidth() );

    return 0.25;
}

} // namespace


std::string KopenapiEdgeCheck( const nlohmann::json& aEdge )
{
    static const std::set<std::string> keys = { "family", "tr_ps", "tr_def", "rout_ohm", "swing_v", "nm_v", "driver" };

    if( !aEdge.is_object() )
        return "an object {family, tr_ps, tr_def, rout_ohm, swing_v, nm_v, driver}";

    for( const auto& [key, value] : aEdge.items() )
    {
        if( !keys.count( key ) )
            return "unknown key '" + key + "' (family, tr_ps, tr_def, rout_ohm, swing_v, nm_v, driver)";
    }

    if( aEdge.contains( "family" ) && ( !aEdge["family"].is_string() || !family( aEdge["family"].get<std::string>() ) ) )
        return "family is one of " + familyNames();

    auto positive = []( const nlohmann::json& aValue ) { return aValue.is_number() && aValue.get<double>() > 0; };

    auto pairOrNumber = [&]( const nlohmann::json& aValue, const char* aA, const char* aB )
    {
        if( positive( aValue ) )
            return true;

        return aValue.is_object() && aValue.size() == 2 && aValue.contains( aA ) && aValue.contains( aB )
               && positive( aValue[aA] ) && positive( aValue[aB] );
    };

    if( aEdge.contains( "tr_ps" ) && !positive( aEdge["tr_ps"] ) )
        return "tr_ps is a number > 0 (the driver's fastest edge, ps)";

    if( aEdge.contains( "tr_def" ) && aEdge["tr_def"] != "10-90" && aEdge["tr_def"] != "20-80" )
        return "tr_def is \"10-90\" or \"20-80\"";

    if( aEdge.contains( "rout_ohm" ) && !pairOrNumber( aEdge["rout_ohm"], "hi", "lo" ) )
        return "rout_ohm is a number > 0 or {hi, lo}";

    if( aEdge.contains( "swing_v" ) && !positive( aEdge["swing_v"] ) )
        return "swing_v is a number > 0";

    if( aEdge.contains( "nm_v" ) && !pairOrNumber( aEdge["nm_v"], "lo", "hi" ) )
        return "nm_v is a number > 0 or {lo, hi}";

    if( aEdge.contains( "driver" )
        && ( !aEdge["driver"].is_string() || aEdge["driver"].get<std::string>().find( '.' ) == std::string::npos ) )
    {
        return "driver is \"REF.PIN\" (U1.5)";
    }

    return {};
}


std::string KopenapiTerminationCheck( const nlohmann::json& aTermination )
{
    if( !aTermination.is_object() || !aTermination.contains( "kind" ) || !aTermination["kind"].is_string()
        || !TERMINATIONS.count( aTermination["kind"].get<std::string>() ) )
    {
        return "{kind: none / series / end / thevenin / rc / on_die / both, dc_balanced?}";
    }

    for( const auto& [key, value] : aTermination.items() )
    {
        if( key != "kind" && key != "dc_balanced" )
            return "unknown key '" + key + "' (kind, dc_balanced)";
    }

    if( aTermination.contains( "dc_balanced" ) && !aTermination["dc_balanced"].is_boolean() )
        return "dc_balanced is true or false";

    return {};
}


static KOPENAPI_RESULT h_pcb_stackup_get( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*       board = context->GetBoard();
    const double width = aArgs.value( "width_mm", defaultWidth( board ) );
    const double freq = aArgs.value( "frequency_hz", 1e9 );

    if( width <= 0 || freq <= 0 )
        return KOPENAPI_RESULT::Error( 400, "width_mm and frequency_hz are > 0" );

    const STACK                        stack = readStack( board );
    const std::vector<std::map<int, double>> shares = planeShares( board, stack );
    const std::vector<int>             planes = planeNets( shares );
    const std::vector<LAYER_LINE>      lines = layerLines( stack, planes, width, freq );
    bool                               anyFill = false;

    for( ZONE* zone : board->Zones() )
        anyFill = anyFill || zone->IsFilled();

    nlohmann::json out = { { "source", stack.fromBoard ? "board" : "default" },
                           { "thickness_mm", stack.thickness },
                           { "copper_layers", board->GetCopperLayerCount() },
                           { "layers", stack.items },
                           { "copper", lineJson( stack, lines, planes, shares, board, width ) } };

    nlohmann::json notes = nlohmann::json::array();

    if( !stack.fromBoard )
        notes.push_back( "the board has no stack-up of its own: KiCad's default for its thickness and layer count "
                         "(pcb_stackup_set gives real values)" );

    if( !board->Zones().empty() && !anyFill )
        notes.push_back( "zones are not filled: planes are found from fills (pcb_zones_fill)" );

    notes.push_back( "a copper layer is a plane when one net's fill covers >= 50 % of the board (ours); impedance "
                     "and delay from KiCad's transmission line calculator at frequency_hz" );
    out["notes"] = notes;
    return KOPENAPI_RESULT::Ok( out );
}


static KOPENAPI_RESULT h_pcb_stackup_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                 board = context->GetBoard();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    BOARD_STACKUP          stackup = board->GetStackupOrDefault();
    const nlohmann::json   rows = aArgs.value( "layers", nlohmann::json::array() );

    if( !rows.is_array() || ( rows.empty() && !aArgs.contains( "copper_thickness_mm" ) ) )
        return KOPENAPI_RESULT::Error( 400, "layers: [{name, thickness_mm?, epsilon_r?, loss_tangent?, material?}] "
                                            "(names from pcb_stackup_get), or copper_thickness_mm" );

    auto positive = []( const nlohmann::json& aRow, const char* aKey )
    { return !aRow.contains( aKey ) || ( aRow[aKey].is_number() && aRow[aKey].get<double>() > 0 ); };

    // validate everything first: all or nothing
    for( const nlohmann::json& row : rows )
    {
        const std::string name = row.value( "name", std::string() );
        BOARD_STACKUP_ITEM* found = nullptr;

        for( BOARD_STACKUP_ITEM* item : stackup.GetList() )
        {
            if( item->IsEnabled() && itemName( board, item ) == name )
                found = item;
        }

        if( !found )
            return KOPENAPI_RESULT::Error( 404, "no stack-up layer '" + name + "' (see pcb_stackup_get)" );

        if( !positive( row, "thickness_mm" ) || !positive( row, "epsilon_r" ) )
            return KOPENAPI_RESULT::Error( 400, name + ": thickness_mm and epsilon_r are > 0" );

        if( row.contains( "loss_tangent" ) && ( !row["loss_tangent"].is_number() || row["loss_tangent"].get<double>() < 0 ) )
            return KOPENAPI_RESULT::Error( 400, name + ": loss_tangent is >= 0" );

        if( ( row.contains( "epsilon_r" ) || row.contains( "loss_tangent" ) ) && !found->HasEpsilonRValue() )
            return KOPENAPI_RESULT::Error( 400, name + " has no dielectric constant (" + itemType( found->GetType() ) + ")" );

        if( row.contains( "thickness_mm" ) && !found->IsThicknessEditable() )
            return KOPENAPI_RESULT::Error( 400, name + " has no thickness (" + itemType( found->GetType() ) + ")" );

        if( row.contains( "material" ) && !row["material"].is_string() )
            return KOPENAPI_RESULT::Error( 400, name + ": material is a string" );
    }

    if( aArgs.contains( "copper_thickness_mm" ) && !positive( aArgs, "copper_thickness_mm" ) )
        return KOPENAPI_RESULT::Error( 400, "copper_thickness_mm is > 0" );

    for( const nlohmann::json& row : rows )
    {
        for( BOARD_STACKUP_ITEM* item : stackup.GetList() )
        {
            if( !item->IsEnabled() || itemName( board, item ) != row["name"].get<std::string>() )
                continue;

            for( int sub = 0; sub < std::max( 1, item->GetSublayersCount() ); ++sub )
            {
                if( row.contains( "thickness_mm" ) )
                    item->SetThickness( pcbIUScale.mmToIU( row["thickness_mm"].get<double>() / std::max( 1, item->GetSublayersCount() ) ), sub );

                if( row.contains( "epsilon_r" ) )
                    item->SetEpsilonR( row["epsilon_r"].get<double>(), sub );

                if( row.contains( "loss_tangent" ) )
                    item->SetLossTangent( row["loss_tangent"].get<double>(), sub );

                if( row.contains( "material" ) )
                    item->SetMaterial( wxString::FromUTF8( row["material"].get<std::string>() ), sub );
            }
        }
    }

    if( aArgs.contains( "copper_thickness_mm" ) )
    {
        for( BOARD_STACKUP_ITEM* item : stackup.GetList() )
        {
            if( item->GetType() == BS_ITEM_TYPE_COPPER )
                item->SetThickness( pcbIUScale.mmToIU( aArgs["copper_thickness_mm"].get<double>() ) );
        }
    }

    bds.GetStackupDescriptor() = stackup;
    bds.m_HasStackup = true;
    bds.SetBoardThickness( stackup.BuildBoardThicknessFromStackup() );
    context->SetContentModified();

    return h_pcb_stackup_get( aCtx, nlohmann::json::object() );
}


// --------------------------------------------------------------------------------------------- signal checks

namespace
{

/// Routed copper of one net as a graph: pads, vias and track ends are nodes, tracks are edges weighted by delay
struct NET_GRAPH
{
    struct EDGE
    {
        int    to;
        double ps;
        double mm;
    };

    std::vector<std::vector<EDGE>> adj;
    std::map<const PAD*, int>      padNode;

    int Add()
    {
        adj.emplace_back();
        return static_cast<int>( adj.size() ) - 1;
    }

    void Link( int aA, int aB, double aPs, double aMm )
    {
        adj[aA].push_back( { aB, aPs, aMm } );
        adj[aB].push_back( { aA, aPs, aMm } );
    }

    /// shortest delay (ps) and its length (mm) from aFrom to every node; < 0 unreached
    std::vector<std::pair<double, double>> From( int aFrom ) const
    {
        std::vector<std::pair<double, double>> dist( adj.size(), { -1.0, 0.0 } );
        using ITEM = std::pair<double, int>;
        std::priority_queue<ITEM, std::vector<ITEM>, std::greater<ITEM>> queue;
        dist[aFrom] = { 0.0, 0.0 };
        queue.push( { 0.0, aFrom } );

        while( !queue.empty() )
        {
            const auto [d, n] = queue.top();
            queue.pop();

            if( d > dist[n].first )
                continue;

            for( const EDGE& e : adj[n] )
            {
                if( dist[e.to].first < 0 || d + e.ps < dist[e.to].first - 1e-9 )
                {
                    dist[e.to] = { d + e.ps, dist[n].second + e.mm };
                    queue.push( { d + e.ps, e.to } );
                }
            }
        }

        return dist;
    }
};


struct NET_INFO
{
    int                      code = 0;
    std::string              name;
    std::string              role = "signal";
    nlohmann::json           edge;
    nlohmann::json           termination;
    std::vector<PAD*>        pads;
    std::vector<PCB_TRACK*>  tracks;
    std::vector<PCB_VIA*>    vias;

    double                   trPs = 0;          ///< 0: unknown
    std::string              trSource;
    double                   routOhm = 0;
    double                   tpPs = 0, lengthMm = 0;
    double                   z0 = 0;
    bool                     partial = false, estimate = false;
    std::string              regime;
    double                   rho = 0, trEff = 0;
    std::string              termKind = "none", termSource = "detected";
    nlohmann::json           termParts = nlohmann::json::array();
    const PAD*               driver = nullptr;
};


struct CHECKER
{
    BOARD*                              board = nullptr;
    STACK                               stack;
    std::vector<std::map<int, double>>  shares;
    std::vector<int>                    planes;
    std::vector<LAYER_LINE>             lines;
    double                              freq = 1e9;
    std::map<int, NET_INFO>             nets;
    std::map<std::pair<int, int>, double> z0Cache;    ///< (layer index, width nm) -> Ω
    nlohmann::json                      findings = nlohmann::json::array();
    std::map<std::string, int>          counts;
    std::set<std::string>               unknown;
    int                                 maxFindings = 500;
    std::string                         filter;     ///< glob over net names, empty: all

    void Find( const std::string& aCheck, const std::string& aSeverity, const std::string& aNet, const std::string& aText,
               const nlohmann::json& aExtra = nlohmann::json::object() )
    {
        if( !filter.empty() && !KopenapiGlob( filter, aNet ) )
            return;

        counts[aSeverity]++;

        if( static_cast<int>( findings.size() ) >= maxFindings )
            return;

        nlohmann::json f = { { "check", aCheck }, { "severity", aSeverity }, { "net", aNet }, { "text", aText } };

        for( const auto& [key, value] : aExtra.items() )
            f[key] = value;

        findings.push_back( f );
    }

    double Delay( PCB_LAYER_ID aLayer ) const
    {
        const int i = stack.Index( aLayer );
        return i >= 0 && lines[i].delayPsMm > 0 ? lines[i].delayPsMm : PS_PER_MM_C0 * std::sqrt( 4.5 );
    }

    /// slowest delay of the board's copper layers (the shortest edge length l)
    double SlowestDelay() const
    {
        double d = 0;

        for( const LAYER_LINE& l : lines )
            d = std::max( d, l.delayPsMm );

        return d > 0 ? d : PS_PER_MM_C0 * std::sqrt( 4.5 );
    }

    double Z0( PCB_LAYER_ID aLayer, int aWidth )
    {
        const int i = stack.Index( aLayer );

        if( i < 0 )
            return 0;

        const auto key = std::make_pair( i, aWidth );

        if( !z0Cache.count( key ) )
            z0Cache[key] = layerLines( stack, planes, toMm( aWidth ), freq )[i].z0;

        return z0Cache[key];
    }

    double H( PCB_LAYER_ID aLayer ) const
    {
        const int i = stack.Index( aLayer );

        if( i < 0 || lines[i].kind == "plane" || lines[i].kind == "no_plane" )
            return 0;

        const LAYER_LINE& l = lines[i];

        if( l.above >= 0 && l.below >= 0 )
            return std::min( l.hAbove, l.hBelow );

        return l.above >= 0 ? l.hAbove : l.hBelow;
    }

    int PlaneCount() const
    {
        return static_cast<int>( std::count_if( planes.begin(), planes.end(), []( int n ) { return n > 0; } ) );
    }
};


std::string padName( const PAD* aPad )
{
    const FOOTPRINT* fp = aPad->GetParentFootprint();
    return ( fp ? str( fp->GetReference() ) : std::string( "?" ) ) + "." + str( aPad->GetNumber() );
}


bool endpointPad( const PAD* aPad )
{
    const FOOTPRINT* fp = aPad->GetParentFootprint();
    return fp && !isPassive( str( fp->GetReference() ) );
}


/// @brief The driver edge of a net: its own values, then its family's defaults, then the assumed family
void resolveEdge( NET_INFO& aNet, const std::string& aAssume )
{
    const nlohmann::json& e = aNet.edge;
    const FAMILY*         fam = e.is_object() && e.contains( "family" ) ? family( e["family"].get<std::string>() ) : nullptr;

    if( e.is_object() && e.contains( "tr_ps" ) )
    {
        aNet.trPs = e["tr_ps"].get<double>() * ( e.value( "tr_def", std::string( "10-90" ) ) == "20-80" ? 1.5 : 1.0 );
        aNet.trSource = "net";
    }
    else if( fam )
    {
        aNet.trPs = fam->trPs;
        aNet.trSource = std::string( "family " ) + fam->name;
    }
    else if( !aAssume.empty() && isSignalRole( aNet.role ) && family( aAssume ) )
    {
        fam = family( aAssume );
        aNet.trPs = fam->trPs;
        aNet.trSource = "assumed family " + aAssume;
    }

    if( e.is_object() && e.contains( "rout_ohm" ) )
    {
        const nlohmann::json& r = e["rout_ohm"];
        aNet.routOhm = r.is_number() ? r.get<double>() : std::max( r["hi"].get<double>(), r["lo"].get<double>() );
    }
    else if( fam )
    {
        aNet.routOhm = fam->routOhm;
    }
}


NET_GRAPH buildGraph( CHECKER& aC, NET_INFO& aNet )
{
    NET_GRAPH g;

    for( PAD* pad : aNet.pads )
        g.padNode[pad] = g.Add();

    std::vector<int> viaNode;

    for( size_t i = 0; i < aNet.vias.size(); ++i )
        viaNode.push_back( g.Add() );

    struct END
    {
        int          node;
        VECTOR2I     at;
        PCB_LAYER_ID layer;
        PCB_TRACK*   track;
    };

    std::vector<END> ends;

    for( PCB_TRACK* t : aNet.tracks )
    {
        const int    a = g.Add(), b = g.Add();
        const double mm = toMm( t->GetLength() );
        g.Link( a, b, mm * aC.Delay( t->GetLayer() ), mm );
        ends.push_back( { a, t->GetStart(), t->GetLayer(), t } );
        ends.push_back( { b, t->GetEnd(), t->GetLayer(), t } );
    }

    for( size_t i = 0; i < ends.size(); ++i )
    {
        const END& e = ends[i];
        const int  half = e.track->GetWidth() / 2;

        for( PAD* pad : aNet.pads )
        {
            if( pad->IsOnLayer( e.layer ) && pad->HitTest( e.at, half ) )
                g.Link( e.node, g.padNode[pad], 0, 0 );
        }

        for( size_t v = 0; v < aNet.vias.size(); ++v )
        {
            if( aNet.vias[v]->IsOnLayer( e.layer ) && aNet.vias[v]->HitTest( e.at, half ) )
                g.Link( e.node, viaNode[v], 0, 0 );
        }

        for( size_t j = i + 1; j < ends.size(); ++j )
        {
            if( ends[j].layer == e.layer && ends[j].track != e.track
                && ( ends[j].at - e.at ).EuclideanNorm() <= std::max( half, ends[j].track->GetWidth() / 2 ) )
            {
                g.Link( e.node, ends[j].node, 0, 0 );
            }
        }

        // a track ending on the middle of another (a T): joined to both ends of that one at their distances
        for( PCB_TRACK* t : aNet.tracks )
        {
            if( t == e.track || t->GetLayer() != e.layer || t->Type() != PCB_TRACE_T )
                continue;

            const SEG seg( t->GetStart(), t->GetEnd() );

            if( seg.Distance( e.at ) > t->GetWidth() / 2 || ( t->GetStart() - e.at ).EuclideanNorm() <= half
                || ( t->GetEnd() - e.at ).EuclideanNorm() <= half )
            {
                continue;
            }

            const VECTOR2I p = seg.NearestPoint( e.at );
            const double   d = aC.Delay( e.layer );

            for( const END& other : ends )
            {
                if( other.track == t )
                {
                    const double mm = toMm( ( other.at - p ).EuclideanNorm() );
                    g.Link( e.node, other.node, mm * d, mm );
                }
            }
        }
    }

    return g;
}


/// @brief One-way delay of the net: from its driver to the farthest pad, or the widest pad pair when the driver is
/// not known; unrouted nets estimated from the pads' Manhattan spread on the outer layer
void measureNet( CHECKER& aC, NET_INFO& aNet )
{
    std::vector<PAD*> ends;

    for( PAD* pad : aNet.pads )
    {
        if( endpointPad( pad ) || aNet.pads.size() <= 2 )
            ends.push_back( pad );
    }

    if( ends.size() < 2 )
        ends = aNet.pads;

    if( aNet.tracks.empty() )
    {
        double spread = 0;

        for( PAD* a : ends )
        {
            for( PAD* b : ends )
            {
                const VECTOR2I d = a->GetPosition() - b->GetPosition();
                spread = std::max( spread, toMm( std::abs( d.x ) + std::abs( d.y ) ) );
            }
        }

        aNet.lengthMm = spread;
        aNet.tpPs = spread * aC.Delay( F_Cu );
        aNet.estimate = true;
        return;
    }

    NET_GRAPH         g = buildGraph( aC, aNet );
    std::vector<PAD*> from = aNet.driver ? std::vector<PAD*>{ const_cast<PAD*>( aNet.driver ) } : ends;
    std::set<PAD*>    reached;

    for( PAD* a : from )
    {
        const auto dist = g.From( g.padNode[a] );

        for( PAD* b : ends )
        {
            const auto [ps, mm] = dist[g.padNode[b]];

            if( ps < 0 )
                continue;

            reached.insert( a );
            reached.insert( b );

            if( ps > aNet.tpPs )
            {
                aNet.tpPs = ps;
                aNet.lengthMm = mm;
            }
        }
    }

    aNet.partial = reached.size() < ends.size();

    // impedance of the layer carrying most of the net
    std::map<std::pair<PCB_LAYER_ID, int>, double> byLayer;

    for( PCB_TRACK* t : aNet.tracks )
        byLayer[{ t->GetLayer(), t->GetWidth() }] += t->GetLength();

    if( !byLayer.empty() )
    {
        const auto best = std::max_element( byLayer.begin(), byLayer.end(),
                                            []( const auto& a, const auto& b ) { return a.second < b.second; } );
        aNet.z0 = aC.Z0( best->first.first, best->first.second );
    }
}


/// @brief Terminations on the net from its parts: series R to another signal, R to a rail (end), R pair to both
/// rails (Thevenin), R to a node with a C to ground (RC)
void detectTermination( CHECKER& aC, NET_INFO& aNet )
{
    if( aNet.termination.is_object() )
    {
        aNet.termKind = aNet.termination["kind"].get<std::string>();
        aNet.termSource = "project";
        return;
    }

    bool toGround = false, toRail = false, series = false, rc = false;

    for( PAD* pad : aNet.pads )
    {
        FOOTPRINT*        fp = pad->GetParentFootprint();
        const std::string ref = fp ? str( fp->GetReference() ) : std::string();

        if( !fp || refPrefix( ref ) != "R" || fp->Pads().size() != 2 )
            continue;

        const PAD*   other = fp->Pads()[0] == pad ? fp->Pads()[1] : fp->Pads()[0];
        const double r = ohms( str( fp->GetValue() ) );
        auto         it = aC.nets.find( other->GetNetCode() );

        if( it == aC.nets.end() )
            continue;

        const NET_INFO& on = it->second;

        if( ( isGround( on.role ) || isRail( on.role ) ) && ( r < 0 || ( r >= 10 && r <= 300 ) ) )
        {
            ( isGround( on.role ) ? toGround : toRail ) = true;
            aNet.termParts.push_back( ref );
        }
        else if( isSignalRole( on.role ) && ( r < 0 || r <= 200 ) )
        {
            bool        capToGround = false;
            bool        endpoint = false;

            for( PAD* q : on.pads )
            {
                FOOTPRINT* qf = q->GetParentFootprint();

                if( !qf || qf == fp )
                    continue;

                if( refPrefix( str( qf->GetReference() ) ) == "C" && qf->Pads().size() == 2 )
                {
                    const PAD* qo = qf->Pads()[0] == q ? qf->Pads()[1] : qf->Pads()[0];
                    auto       g = aC.nets.find( qo->GetNetCode() );
                    capToGround = capToGround || ( g != aC.nets.end() && isGround( g->second.role ) );
                }
                else if( endpointPad( q ) )
                {
                    endpoint = true;
                }
            }

            if( capToGround && !endpoint )
                rc = true;
            else if( endpoint )
                series = true;
            else
                continue;

            aNet.termParts.push_back( ref );
        }
    }

    if( toGround && toRail )
        aNet.termKind = "thevenin";
    else if( toGround || toRail )
        aNet.termKind = "end";
    else if( rc )
        aNet.termKind = "rc";

    if( series )
        aNet.termKind = aNet.termKind == "none" ? "series" : "both";
}


// --------------------------------------------------------------------------------------------- DG checks

void checkRegime( CHECKER& aC, NET_INFO& aNet, double aPinCPf, bool aReport )
{
    if( aNet.trPs <= 0 )
        return;

    // a weak driver into the line charges it like an RC: classify with the loaded edge (G1)
    aNet.trEff = aNet.trPs;

    if( aNet.routOhm > 0 && aNet.z0 > 0 && aNet.routOhm > aNet.z0 )
    {
        int loads = 0;

        for( PAD* pad : aNet.pads )
            loads += endpointPad( pad ) && pad != aNet.driver ? 1 : 0;

        const double cPf = aNet.tpPs / aNet.z0 + std::max( 1, loads - ( aNet.driver ? 0 : 1 ) ) * aPinCPf;
        aNet.trEff = std::hypot( aNet.trPs, 2.2 * aNet.routOhm * cPf );
    }

    aNet.rho = aNet.tpPs / aNet.trEff;
    aNet.regime = aNet.rho < 0.1 ? "lumped" : aNet.rho < 1.0 / 6 ? "near" : aNet.rho < 0.5 ? "distributed" : "ringing";

    if( aNet.regime == "near" && aReport )
    {
        aC.Find( "DG-1", "warn", aNet.name,
                 "near the line boundary: delay " + fmt( aNet.tpPs, 0 ) + " ps is " + fmt( aNet.rho * 100, 0 )
                         + " % of the edge " + fmt( aNet.trEff, 0 ) + " ps (lumped below 10 % (ours), a line from "
                           "17 % (src §1.3)); plan its termination",
                 { { "rho", round3( aNet.rho ) }, { "limit", 1.0 / 6 }, { "limit_source", "src" } } );
    }
}


void checkTermination( CHECKER& aC, NET_INFO& aNet )
{
    if( aNet.regime != "distributed" && aNet.regime != "ringing" )
        return;

    if( aNet.termKind == "none" )
    {
        aC.Find( "DG-5", "fail", aNet.name,
                 "a transmission line without termination: delay " + fmt( aNet.tpPs, 0 ) + " ps over "
                         + fmt( aNet.lengthMm, 1 ) + " mm against a " + fmt( aNet.trEff, 0 )
                         + " ps edge (rho " + fmt( aNet.rho, 2 ) + " >= 1/6, HSDD §1.3, §6); add a series R at the "
                           "driver or an end termination, or shorten it below " + fmt( aNet.trEff / 6 / ( aNet.tpPs / std::max( 1e-9, aNet.lengthMm ) ), 1 ) + " mm",
                 { { "rho", round3( aNet.rho ) }, { "length_mm", round3( aNet.lengthMm ) }, { "limit", 1.0 / 6 },
                   { "limit_source", "src" } } );
    }
}


void checkSeriesStub( CHECKER& aC, std::set<std::string>& aDone )
{
    for( auto& [code, net] : aC.nets )
    {
        if( net.termKind != "series" && net.termKind != "both" )
            continue;

        for( PAD* pad : net.pads )
        {
            FOOTPRINT* fp = pad->GetParentFootprint();

            if( !fp || refPrefix( str( fp->GetReference() ) ) != "R" || fp->Pads().size() != 2
                || aDone.count( str( fp->GetReference() ) ) )
            {
                continue;
            }

            const PAD* other = fp->Pads()[0] == pad ? fp->Pads()[1] : fp->Pads()[0];
            auto       on = aC.nets.find( other->GetNetCode() );

            if( on == aC.nets.end() || !isSignalRole( on->second.role ) )
                continue;

            aDone.insert( str( fp->GetReference() ) );

            // the driver side: the net whose endpoint pad is nearest the resistor along the copper
            double      best = -1, bestMm = 0, tr = std::max( net.trPs, on->second.trPs );
            std::string side, toPad;

            const bool anyDriver = net.driver || on->second.driver;

            for( NET_INFO* n : { &net, &on->second } )
            {
                const PAD* rPad = n == &net ? pad : other;
                NET_GRAPH  g = buildGraph( aC, *n );
                const auto dist = g.From( g.padNode[const_cast<PAD*>( rPad )] );

                for( PAD* q : n->pads )
                {
                    if( anyDriver && q != n->driver )
                        continue;

                    if( !endpointPad( q ) || dist[g.padNode[q]].first < 0 )
                        continue;

                    if( best < 0 || dist[g.padNode[q]].first < best )
                    {
                        best = dist[g.padNode[q]].first;
                        bestMm = dist[g.padNode[q]].second;
                        side = n->name;
                        toPad = padName( q );
                    }
                }
            }

            if( best < 0 )
                continue;

            if( tr <= 0 )
            {
                aC.unknown.insert( net.name );
                continue;
            }

            const std::string sev = best > tr / 6 ? "fail" : best > tr / 20 ? "warn" : "";

            if( !sev.empty() )
            {
                aC.Find( "DG-7", sev, side,
                         "series resistor " + str( fp->GetReference() ) + " is " + fmt( bestMm, 1 ) + " mm ("
                                 + fmt( best, 0 ) + " ps) from its driver " + toPad + ": over " + ( sev == "fail" ? "Tr/6 (src §6.2)" : "Tr/20 (ours)" )
                                 + " of the " + fmt( tr, 0 ) + " ps edge; move it to the driver pin",
                         { { "part", str( fp->GetReference() ) }, { "stub_mm", round3( bestMm ) }, { "stub_ps", round3( best ) },
                           { "limit_ps", round3( sev == "fail" ? tr / 6 : tr / 20 ) },
                           { "limit_source", sev == "fail" ? "src" : "ours" } } );
            }
        }
    }
}


void checkCrosstalk( CHECKER& aC, double aBudget, double aBudgetClock )
{
    struct PIECE
    {
        double alpha, mm, d;
        VECTOR2I at;
    };

    std::map<std::tuple<int, int, int>, std::vector<PIECE>> pairs;   // (layer, net a, net b)
    std::map<int, std::vector<PCB_TRACK*>>                   byLayer;

    for( auto& [code, net] : aC.nets )
    {
        if( !isSignalRole( net.role ) )
            continue;

        for( PCB_TRACK* t : net.tracks )
        {
            if( t->Type() == PCB_TRACE_T && aC.H( t->GetLayer() ) > 0 )
                byLayer[t->GetLayer()].push_back( t );
        }
    }

    for( auto& [layer, tracks] : byLayer )
    {
        const double h = aC.H( static_cast<PCB_LAYER_ID>( layer ) );
        const int    cell = std::max( 1, static_cast<int>( pcbIUScale.mmToIU( h * 10 ) ) );
        std::map<std::pair<int, int>, std::vector<size_t>> grid;

        for( size_t i = 0; i < tracks.size(); ++i )
        {
            const BOX2I box = tracks[i]->GetBoundingBox();

            for( int x = box.GetLeft() / cell - 1; x <= box.GetRight() / cell + 1; ++x )
            {
                for( int y = box.GetTop() / cell - 1; y <= box.GetBottom() / cell + 1; ++y )
                    grid[{ x, y }].push_back( i );
            }
        }

        std::set<std::pair<size_t, size_t>> seen;

        for( const auto& [key, idx] : grid )
        {
            for( size_t a = 0; a < idx.size(); ++a )
            {
                for( size_t b = a + 1; b < idx.size(); ++b )
                {
                    PCB_TRACK* ta = tracks[std::min( idx[a], idx[b] )];
                    PCB_TRACK* tb = tracks[std::max( idx[a], idx[b] )];

                    if( ta->GetNetCode() == tb->GetNetCode()
                        || ( aC.nets[ta->GetNetCode()].role == "pair" && aC.nets[tb->GetNetCode()].role == "pair" )
                        || !seen.insert( { std::min( idx[a], idx[b] ), std::max( idx[a], idx[b] ) } ).second )
                    {
                        continue;
                    }

                    const VECTOR2D a0( ta->GetStart() ), a1( ta->GetEnd() ), b0( tb->GetStart() ), b1( tb->GetEnd() );
                    const VECTOR2D da = a1 - a0, db = b1 - b0;
                    const double   la = da.EuclideanNorm(), lb = db.EuclideanNorm();

                    if( la <= 0 || lb <= 0 || std::abs( da.Dot( db ) ) / ( la * lb ) < 0.985 )
                        continue;

                    const VECTOR2D u = da / la;
                    const double   s0 = ( b0 - a0 ).Dot( u ), s1 = ( b1 - a0 ).Dot( u );
                    const double   lo = std::max( 0.0, std::min( s0, s1 ) ), hi = std::min( la, std::max( s0, s1 ) );

                    if( hi - lo <= 0 )
                        continue;

                    const VECTOR2D mid = a0 + u * ( ( lo + hi ) / 2 );
                    const double   dist = toMm( SEG( tb->GetStart(), tb->GetEnd() ).Distance( VECTOR2I( mid ) ) );

                    if( dist > 10 * h )
                        continue;

                    const double alpha = 1.0 / ( 1.0 + ( dist / h ) * ( dist / h ) );
                    const int    na = std::min( ta->GetNetCode(), tb->GetNetCode() );
                    const int    nb = std::max( ta->GetNetCode(), tb->GetNetCode() );
                    pairs[{ layer, na, nb }].push_back( { alpha, toMm( hi - lo ), dist, VECTOR2I( mid ) } );
                }
            }
        }
    }

    for( const auto& [key, pieces] : pairs )
    {
        const auto [layer, na, nb] = key;
        const double h = aC.H( static_cast<PCB_LAYER_ID>( layer ) );
        const double d = aC.Delay( static_cast<PCB_LAYER_ID>( layer ) );

        for( int dir = 0; dir < 2; ++dir )
        {
            NET_INFO&    victim = aC.nets[dir ? nb : na];
            NET_INFO&    aggressor = aC.nets[dir ? na : nb];
            const double budget = victim.role == "clock" ? aBudgetClock : aBudget;

            if( aggressor.trPs <= 0 )
            {
                aC.unknown.insert( aggressor.name );
                continue;
            }

            double      coupled = 0, alphaMax = 0, dMin = 1e9;
            VECTOR2I    at;

            for( const PIECE& p : pieces )
            {
                if( p.alpha >= budget )
                    coupled += p.mm;

                if( p.alpha > alphaMax )
                {
                    alphaMax = p.alpha;
                    dMin = p.d;
                    at = p.at;
                }
            }

            const double alphaEff = alphaMax * std::min( 1.0, 2 * coupled * d / aggressor.trPs );

            if( alphaEff <= budget )
                continue;

            aC.Find( "DG-14", "warn", victim.name,
                     "crosstalk from " + aggressor.name + " on " + aC.stack.copper[aC.stack.Index( static_cast<PCB_LAYER_ID>( layer ) )].name
                             + ": " + fmt( alphaEff * 100, 1 ) + " % (spacing " + fmt( dMin, 2 ) + " mm at H " + fmt( h, 2 )
                             + " mm over " + fmt( coupled, 1 ) + " mm) against a " + fmt( budget * 100, 0 )
                             + " % budget (ours); centre spacing >= " + fmt( h * std::sqrt( 1 / budget - 1 ), 2 ) + " mm (HSDD eq. 5.2)",
                     { { "aggressor", aggressor.name }, { "crosstalk", round3( alphaEff ) }, { "spacing_mm", round3( dMin ) },
                       { "h_mm", round3( h ) }, { "coupled_mm", round3( coupled ) }, { "at_mm", { toMm( at.x ), toMm( at.y ) } },
                       { "min_spacing_mm", round3( h * std::sqrt( 1 / budget - 1 ) ) }, { "limit", budget },
                       { "limit_source", "ours" } } );
        }
    }
}


/// @brief The plane net under a point on a copper layer: a fill around the point (its own antipad excluded)
int planeAt( CHECKER& aC, int aLayer, const VECTOR2I& aAt, int aRadius )
{
    std::map<int, int> hits;
    const PCB_LAYER_ID layer = aC.stack.copper[aLayer].id;

    for( ZONE* zone : aC.board->Zones() )
    {
        if( zone->GetIsRuleArea() || zone->GetNetCode() <= 0 || !zone->IsOnLayer( layer ) || !zone->IsFilled() )
            continue;

        for( int k = 0; k < 8; ++k )
        {
            const double   a = k * M_PI / 4;
            const VECTOR2I p = aAt + VECTOR2I( static_cast<int>( aRadius * std::cos( a ) ), static_cast<int>( aRadius * std::sin( a ) ) );

            if( zone->HitTestFilledArea( layer, p ) )
                hits[zone->GetNetCode()]++;
        }
    }

    int best = 0, count = 2;

    for( const auto& [net, n] : hits )
    {
        if( n > count )
        {
            best = net;
            count = n;
        }
    }

    return best;
}


void checkReferenceChange( CHECKER& aC, double aReturnMaxMm )
{
    if( aC.PlaneCount() < 2 )
        return;

    const int n = static_cast<int>( aC.stack.copper.size() );

    for( auto& [code, net] : aC.nets )
    {
        if( !isSignalRole( net.role ) || net.vias.empty() )
            continue;

        if( net.trPs <= 0 )
        {
            aC.unknown.insert( net.name );
            continue;
        }

        for( PCB_VIA* via : net.vias )
        {
            const VECTOR2I    at = via->GetPosition();
            std::set<int>     used;

            for( PCB_TRACK* t : net.tracks )
            {
                if( ( via->HitTest( t->GetStart(), t->GetWidth() / 2 ) || via->HitTest( t->GetEnd(), t->GetWidth() / 2 ) )
                    && aC.stack.Index( t->GetLayer() ) >= 0 )
                {
                    used.insert( aC.stack.Index( t->GetLayer() ) );
                }
            }

            for( PAD* pad : net.pads )
            {
                if( pad->GetAttribute() == PAD_ATTRIB::SMD && pad->HitTest( at ) )
                    used.insert( aC.stack.Index( pad->GetPrincipalLayer() ) );
            }

            used.erase( -1 );

            if( used.size() < 2 )
                continue;

            const int top = *used.begin(), bottom = *used.rbegin();
            const int radius = via->GetWidth( F_Cu ) / 2 + pcbIUScale.mmToIU( 0.6 );

            // the nearest plane each way from a layer, at this via
            auto refs = [&]( int aLayer )
            {
                std::map<int, int> out;   // copper index -> net

                for( int dir : { -1, 1 } )
                {
                    for( int j = aLayer + dir; j >= 0 && j < n; j += dir )
                    {
                        if( const int p = planeAt( aC, j, at, radius ) )
                        {
                            out[j] = p;
                            break;
                        }
                    }
                }

                return out;
            };

            const std::map<int, int> ra = refs( top ), rb = refs( bottom );
            bool                     shared = false;

            for( const auto& [layer, pnet] : ra )
                shared = shared || rb.count( layer );

            if( shared || ra.empty() || rb.empty() )
                continue;

            std::set<int> refNets;

            for( const auto& m : { ra, rb } )
            {
                for( const auto& [layer, pnet] : m )
                    refNets.insert( pnet );
            }

            const double l = net.trPs / std::max( aC.Delay( aC.stack.copper[top].id ), aC.Delay( aC.stack.copper[bottom].id ) );
            const double r = std::min( aReturnMaxMm, l / 12 );
            double       dRet = -1;
            std::string  by;

            if( refNets.size() == 1 )
            {
                const int ground = *refNets.begin();

                for( PCB_TRACK* t : aC.board->Tracks() )
                {
                    if( t->Type() == PCB_VIA_T && t->GetNetCode() == ground )
                    {
                        const double d = toMm( ( t->GetPosition() - at ).EuclideanNorm() );

                        if( dRet < 0 || d < dRet )
                        {
                            dRet = d;
                            by = "via";
                        }
                    }
                }

                for( FOOTPRINT* fp : aC.board->Footprints() )
                {
                    for( PAD* pad : fp->Pads() )
                    {
                        if( pad->GetNetCode() == ground && pad->GetAttribute() == PAD_ATTRIB::PTH )
                        {
                            const double d = toMm( ( pad->GetPosition() - at ).EuclideanNorm() );

                            if( dRet < 0 || d < dRet )
                            {
                                dRet = d;
                                by = padName( pad );
                            }
                        }
                    }
                }
            }
            else
            {
                for( FOOTPRINT* fp : aC.board->Footprints() )
                {
                    if( refPrefix( str( fp->GetReference() ) ) != "C" || fp->Pads().size() != 2 )
                        continue;

                    const int a = fp->Pads()[0]->GetNetCode(), b = fp->Pads()[1]->GetNetCode();

                    if( a == b || !refNets.count( a ) || !refNets.count( b ) )
                        continue;

                    for( PAD* pad : fp->Pads() )
                    {
                        const double d = toMm( ( pad->GetPosition() - at ).EuclideanNorm() );

                        if( dRet < 0 || d < dRet )
                        {
                            dRet = d;
                            by = str( fp->GetReference() );
                        }
                    }
                }
            }

            if( dRet >= 0 && dRet <= r )
                continue;

            std::string planesText;

            for( const auto& m : { ra, rb } )
            {
                for( const auto& [layer, pnet] : m )
                {
                    NETINFO_ITEM* ni = aC.board->GetNetInfo().GetNetItem( pnet );
                    planesText += ( planesText.empty() ? "" : ", " ) + aC.stack.copper[layer].name + " "
                                  + ( ni ? str( UnescapeString( ni->GetNetname() ) ) : std::string() );
                }
            }

            aC.Find( "DG-19", "fail", net.name,
                     "via at (" + fmt( toMm( at.x ) ) + ", " + fmt( toMm( at.y ) ) + ") takes the signal from "
                             + aC.stack.copper[top].name + " to " + aC.stack.copper[bottom].name
                             + ", which have no plane in common (" + planesText + "): the return current needs "
                             + ( refNets.size() == 1 ? "a via of the plane net" : "a capacitor between the plane nets" )
                             + " within " + fmt( r, 2 ) + " mm (min of return_via_max_mm and l/12, HSDD §7.4 / §7.3); "
                             + ( dRet < 0 ? std::string( "none found" ) : "nearest " + by + " at " + fmt( dRet, 2 ) + " mm" ),
                     { { "at_mm", { toMm( at.x ), toMm( at.y ) } }, { "from", aC.stack.copper[top].name },
                       { "to", aC.stack.copper[bottom].name }, { "return_mm", dRet < 0 ? nlohmann::json() : nlohmann::json( round3( dRet ) ) },
                       { "limit_mm", round3( r ) }, { "limit_source", r < aReturnMaxMm ? "src" : "ours" } } );
        }
    }
}


void checkDecoupling( CHECKER& aC )
{
    const double slowest = aC.SlowestDelay();

    for( FOOTPRINT* fp : aC.board->Footprints() )
    {
        const std::string ref = str( fp->GetReference() );

        if( isPassive( ref ) || KopenapiIsConnectorRef( ref ) || fp->Pads().size() < 3 )
            continue;

        double                          tr = 0;
        std::map<int, std::vector<PAD*>> rails;

        for( PAD* pad : fp->Pads() )
        {
            auto it = aC.nets.find( pad->GetNetCode() );

            if( it == aC.nets.end() )
                continue;

            if( isRail( it->second.role ) )
                rails[pad->GetNetCode()].push_back( pad );
            else if( isSignalRole( it->second.role ) && it->second.trPs > 0 )
                tr = tr <= 0 ? it->second.trPs : std::min( tr, it->second.trPs );
        }

        if( rails.empty() )
            continue;

        if( tr <= 0 )
        {
            aC.unknown.insert( ref );
            continue;
        }

        const double r = tr / slowest / 12;

        for( const auto& [rail, pins] : rails )
        {
            double      best = -1;
            std::string cap;

            for( FOOTPRINT* c : aC.board->Footprints() )
            {
                if( refPrefix( str( c->GetReference() ) ) != "C" || c->Pads().size() != 2 )
                    continue;

                PAD* p0 = c->Pads()[0];
                PAD* p1 = c->Pads()[1];
                PAD* onRail = p0->GetNetCode() == rail ? p0 : p1->GetNetCode() == rail ? p1 : nullptr;
                PAD* other = onRail == p0 ? p1 : p0;
                auto g = aC.nets.find( other->GetNetCode() );

                if( !onRail || g == aC.nets.end() || !isGround( g->second.role ) )
                    continue;

                for( PAD* pin : pins )
                {
                    const double d = toMm( ( onRail->GetPosition() - pin->GetPosition() ).EuclideanNorm() );

                    if( best < 0 || d < best )
                    {
                        best = d;
                        cap = str( c->GetReference() );
                    }
                }
            }

            if( best >= 0 && best <= r )
                continue;

            const std::string railName = aC.nets[rail].name;
            aC.Find( "DG-23", "fail", railName,
                     ref + ": no bypass capacitor on " + railName + " within " + fmt( r, 1 ) + " mm (l/12 of its "
                             + fmt( tr, 0 ) + " ps edge, HSDD §7.3 / §8.2.4)"
                             + ( best < 0 ? std::string( "; none on the rail" ) : "; nearest " + cap + " at " + fmt( best, 1 ) + " mm" ),
                     { { "part", ref }, { "rail", railName }, { "nearest", cap }, { "distance_mm", best < 0 ? nlohmann::json() : nlohmann::json( round3( best ) ) },
                       { "limit_mm", round3( r ) }, { "limit_source", "src" } } );
        }
    }
}

} // namespace


static KOPENAPI_RESULT h_pcb_signal_check( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    static const std::set<std::string> ALL = { "DG-1", "DG-5", "DG-7", "DG-14", "DG-19", "DG-23" };
    std::set<std::string>              checks = ALL;

    if( aArgs.contains( "checks" ) )
    {
        checks.clear();

        for( const auto& c : aArgs["checks"] )
        {
            if( !c.is_string() || !ALL.count( c.get<std::string>() ) )
                return KOPENAPI_RESULT::Error( 400, "checks: some of DG-1, DG-5, DG-7, DG-14, DG-19, DG-23" );

            checks.insert( c.get<std::string>() );
        }
    }

    const std::string assume = aArgs.value( "assume_family", std::string() );

    if( !assume.empty() && !family( assume ) )
        return KOPENAPI_RESULT::Error( 400, "assume_family is one of " + familyNames() );

    const double returnMax = aArgs.value( "return_via_max_mm", 1.0 );
    const double budget = aArgs.value( "xtalk_budget", 0.03 );
    const double budgetClock = aArgs.value( "xtalk_budget_clock", 0.01 );
    const double pinC = aArgs.value( "pin_c_pf", 5.0 );

    if( returnMax <= 0 || budget <= 0 || budget >= 1 || budgetClock <= 0 || budgetClock >= 1 || pinC < 0 )
        return KOPENAPI_RESULT::Error( 400, "return_via_max_mm > 0, xtalk budgets in (0, 1), pin_c_pf >= 0" );

    std::optional<KOPENAPI_METHOD> rolesMethod = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" );

    if( !rolesMethod )
        return KOPENAPI_RESULT::Error( 503, "pcb_net_roles is not available" );

    KOPENAPI_RESULT roles = rolesMethod->handler( aCtx, nlohmann::json::object() );

    if( roles.status != 200 )
        return roles;

    CHECKER c;
    c.board = context->GetBoard();
    c.freq = aArgs.value( "frequency_hz", 1e9 );
    c.maxFindings = std::max( 1, aArgs.value( "max_findings", 500 ) );
    c.stack = readStack( c.board );
    c.shares = planeShares( c.board, c.stack );
    c.planes = planeNets( c.shares );
    c.lines = layerLines( c.stack, c.planes, defaultWidth( c.board ), c.freq );

    std::map<std::string, const nlohmann::json*> rows;

    for( const nlohmann::json& row : roles.body["nets"] )
        rows[row["net"].get<std::string>()] = &row;

    for( NETINFO_ITEM* ni : c.board->GetNetInfo() )
    {
        if( ni->GetNetCode() <= 0 )
            continue;

        NET_INFO& net = c.nets[ni->GetNetCode()];
        net.code = ni->GetNetCode();
        net.name = str( UnescapeString( ni->GetNetname() ) );

        if( rows.count( net.name ) )
        {
            const nlohmann::json& row = *rows[net.name];
            net.role = row.value( "role", std::string( "signal" ) );
            net.edge = row.value( "edge", nlohmann::json() );
            net.termination = row.value( "termination", nlohmann::json() );
        }
    }

    for( FOOTPRINT* fp : c.board->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            auto it = c.nets.find( pad->GetNetCode() );

            if( it != c.nets.end() )
                it->second.pads.push_back( pad );
        }
    }

    for( PCB_TRACK* t : c.board->Tracks() )
    {
        auto it = c.nets.find( t->GetNetCode() );

        if( it == c.nets.end() )
            continue;

        if( t->Type() == PCB_VIA_T )
            it->second.vias.push_back( static_cast<PCB_VIA*>( t ) );
        else
            it->second.tracks.push_back( t );
    }

    const std::string filter = aArgs.value( "nets", std::string() );
    c.filter = filter;

    for( auto& [code, net] : c.nets )
    {
        if( net.edge.is_object() && net.edge.contains( "driver" ) )
        {
            const std::string d = net.edge["driver"].get<std::string>();

            for( PAD* pad : net.pads )
            {
                if( padName( pad ) == d )
                    net.driver = pad;
            }
        }

        if( !net.driver )
        {
            const PAD* only = nullptr;
            int        outputs = 0;

            for( PAD* pad : net.pads )
            {
                const std::string type = str( pad->GetPinType() );

                if( type == "output" || type == "tri_state" )
                {
                    only = pad;
                    ++outputs;
                }
            }

            if( outputs == 1 )
                net.driver = only;
        }

        resolveEdge( net, assume );
    }

    for( auto& [code, net] : c.nets )
        detectTermination( c, net );

    nlohmann::json netRows = nlohmann::json::array();

    for( auto& [code, net] : c.nets )
    {
        if( !isSignalRole( net.role ) || net.pads.size() < 2 )
            continue;

        if( !filter.empty() && !KopenapiGlob( filter, net.name ) )
            continue;

        measureNet( c, net );

        if( net.trPs <= 0 )
        {
            c.unknown.insert( net.name );
        }
        else
        {
            checkRegime( c, net, pinC, checks.count( "DG-1" ) > 0 );

            if( checks.count( "DG-5" ) )
                checkTermination( c, net );
        }

        nlohmann::json row = { { "net", net.name }, { "role", net.role }, { "length_mm", round3( net.lengthMm ) },
                               { "delay_ps", round3( net.tpPs ) }, { "termination", net.termKind },
                               { "termination_source", net.termSource } };

        if( !net.termParts.empty() )
            row["termination_parts"] = net.termParts;

        if( net.driver )
            row["driver"] = padName( net.driver );

        if( net.z0 > 0 )
            row["z0_ohm"] = std::round( net.z0 * 10 ) / 10;

        if( net.estimate )
            row["length_source"] = "estimate (unrouted: pads' Manhattan spread)";
        else if( net.partial )
            row["length_source"] = "partly routed";

        if( net.trPs > 0 )
        {
            row["tr_ps"] = net.trPs;
            row["tr_source"] = net.trSource;
            row["fknee_mhz"] = std::round( 0.5 / net.trPs * 1e6 );
            row["edge_length_mm"] = round3( net.trEff / ( net.lengthMm > 0 ? net.tpPs / net.lengthMm : c.Delay( F_Cu ) ) );
            row["rho"] = round3( net.rho );
            row["regime"] = net.regime;

            if( net.trEff > net.trPs * 1.0001 )
                row["tr_loaded_ps"] = std::round( net.trEff );
        }
        else
        {
            row["regime"] = "unknown";
        }

        netRows.push_back( row );
    }

    std::set<std::string> done;

    if( checks.count( "DG-7" ) )
        checkSeriesStub( c, done );

    if( checks.count( "DG-14" ) )
        checkCrosstalk( c, budget, budgetClock );

    if( checks.count( "DG-19" ) )
        checkReferenceChange( c, returnMax );

    if( checks.count( "DG-23" ) )
        checkDecoupling( c );

    bool slow = c.unknown.empty();

    for( const nlohmann::json& row : netRows )
        slow = slow && row["regime"] == "lumped";

    nlohmann::json notes = nlohmann::json::array();

    if( !c.unknown.empty() )
        notes.push_back( "nets and parts without a driver edge are not judged (unknown, never pass): give edge "
                         "{family | tr_ps} with pcb_net_roles_set, or assume_family" );

    if( c.PlaneCount() < 2 && checks.count( "DG-19" ) )
        notes.push_back( "DG-19 skipped: fewer than two planes (a reference change needs two, HSDD §7.4)" );

    notes.push_back( "rules, thresholds and their sources (src: the book, ours: our proposal): "
                     "docs/pcb/routing-rules-digital-hsdd.md" );

    nlohmann::json counts = { { "fail", c.counts["fail"] }, { "warn", c.counts["warn"] } };

    return KOPENAPI_RESULT::Ok( { { "slow_board", slow },
                                  { "planes", c.PlaneCount() },
                                  { "counts", counts },
                                  { "findings", c.findings },
                                  { "nets", netRows },
                                  { "unknown", c.unknown },
                                  { "notes", notes } } );
}


KOPENAPI_REGISTER( "pcb_stackup_get",
                   "The board's stack-up (layers top to bottom: copper, dielectric with thickness, εr, loss tangent, "
                   "material; mask) and each copper layer as a line: plane (one net's fill >= 50 % of the board) or "
                   "signal layer with its reference planes above / below and their distance h, kind (microstrip, "
                   "stripline, embedded_microstrip, no_plane), effective εr, delay per mm and impedance of a track "
                   "width_mm wide (default: the Default class width) from KiCad's transmission line calculator",
                   R"json({"type":"object","properties":{
                        "width_mm":{"type":"number","description":"track width for the impedance (default: Default class)"},
                        "frequency_hz":{"type":"number","default":1e9,"description":"frequency of the impedance and delay"}}})json"_json,
                   false, h_pcb_stackup_get );

KOPENAPI_REGISTER( "pcb_stackup_set",
                   "Set the board stack-up as the Board Setup dialog does: layers [{name (from pcb_stackup_get: F.Cu, "
                   "\"dielectric 1\", F.Mask), thickness_mm?, epsilon_r?, loss_tangent?, material?}], "
                   "copper_thickness_mm for all copper layers; all or nothing; the board thickness follows. Answers "
                   "as pcb_stackup_get",
                   R"json({"type":"object","properties":{
                        "layers":{"type":"array","items":{"type":"object","properties":{
                            "name":{"type":"string"},
                            "thickness_mm":{"type":"number"},
                            "epsilon_r":{"type":"number"},
                            "loss_tangent":{"type":"number"},
                            "material":{"type":"string"}},"required":["name"]}},
                        "copper_thickness_mm":{"type":"number"}}})json"_json,
                   false, h_pcb_stackup_set );

KOPENAPI_MARK_EDITING( "pcb_stackup_set" );

KOPENAPI_REGISTER( "pcb_signal_check",
                   "Digital signal checks of docs/pcb/routing-rules-digital-hsdd.md on the routed board, each only "
                   "where its condition holds. Per signal net: driver edge (pcb_net_roles_set edge, its family, or "
                   "assume_family), one-way delay from the driver (or the widest pad pair) along the copper with each "
                   "layer's delay, rho = delay / edge, regime (lumped < 1/10, near, distributed >= 1/6, ringing >= "
                   "1/2), termination found from the parts or set. Findings: DG-1 near the line boundary; DG-5 a "
                   "line without termination; DG-7 series resistor too far from its driver (Tr/6 fail, Tr/20 warn); "
                   "DG-14 parallel crosstalk over a plane 1/(1+(D/H)^2) against xtalk_budget (clock nets "
                   "xtalk_budget_clock), scaled below the saturation length; DG-19 a via between layers with no plane "
                   "in common and no return (plane-net via, or a capacitor between the plane nets) within "
                   "min(return_via_max_mm, l/12); DG-23 a part's rail without a bypass capacitor within l/12 of its "
                   "pins. Nets without an edge are listed as unknown, never passed; slow_board: every net known and "
                   "lumped",
                   R"json({"type":"object","properties":{
                        "nets":{"type":"string","description":"glob over net names"},
                        "checks":{"type":"array","items":{"type":"string","enum":["DG-1","DG-5","DG-7","DG-14","DG-19","DG-23"]}},
                        "assume_family":{"type":"string","description":"edge family for signal nets that have none (lvcmos33, mcu_fast, ...)"},
                        "return_via_max_mm":{"type":"number","default":1.0},
                        "xtalk_budget":{"type":"number","default":0.03},
                        "xtalk_budget_clock":{"type":"number","default":0.01},
                        "pin_c_pf":{"type":"number","default":5,"description":"receiver pin capacitance for the loaded edge of weak drivers"},
                        "frequency_hz":{"type":"number","default":1e9},
                        "max_findings":{"type":"integer","default":500}}})json"_json,
                   false, h_pcb_signal_check, 300 );
