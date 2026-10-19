/// @file schematic_v10.cpp
/// @brief Schematic and symbol library files: this tree's format -> 10.0 (20260306 / 20251024)
#include "v10_common.h"


namespace LEGACY_FORMATS
{
namespace V10
{

static constexpr long long IU_PER_MM = 10000;
static constexpr long long MAX_ERROR = 100;     // 10 um

/// Every schematic and symbol library bump after 10.0; tools/legacy_formats_refresh.py checks them
static const int RULE_BUMPS_SCHEMATIC[] = { 20260326, 20260508, 20260512, 20260622, 20260623, 20260629,
                                            20260722, 20260803, 20260818, 20260826, 20260830 };
static const int RULE_BUMPS_SYMBOL_LIB[] = { 20260508, 20260622, 20260629, 20260710, 20260826, 20260830 };


static NODE xyNode( long long aX, long long aY )
{
    NODE n = NODE::List( "xy" );
    n.items.push_back( NODE::Atom( FormatMM( aX, IU_PER_MM ) ) );
    n.items.push_back( NODE::Atom( FormatMM( aY, IU_PER_MM ) ) );
    return n;
}


// 20260508: ellipses -> polylines. Symbol bodies store y up.
static void ellipses( NODE& aRoot, CONTEXT& aCtx )
{
    std::function<void( NODE&, bool )> walk = [&]( NODE& aNode, bool aInvertY )
    {
        for( size_t ii = 1; ii < aNode.items.size(); ++ii )
        {
            NODE&              item = aNode.items[ii];
            const std::string& head = item.Head();

            if( !item.isList )
                continue;

            if( head != "ellipse" && head != "ellipse_arc" )
            {
                walk( item, aInvertY || head == "lib_symbols" || aCtx.kind == KIND::SYMBOL_LIB );
                continue;
            }

            bool              arc = head == "ellipse_arc";
            long long         cx = 0;
            long long         cy = 0;
            double            major = 0.0;
            double            minor = 0.0;
            double            rotation = 0.0;
            double            start = 0.0;
            double            end = 90.0;
            std::vector<NODE> keep;

            for( size_t jj = 1; jj < item.items.size(); ++jj )
            {
                NODE&  part = item.items[jj];
                double x = 0.0;
                double y = 0.0;

                if( part.Is( "center" ) && ParseNumber( part.AtomAt( 1 ), x ) && ParseNumber( part.AtomAt( 2 ), y ) )
                {
                    cx = ToIU( x, IU_PER_MM );
                    cy = ToIU( y, IU_PER_MM );
                }
                else if( part.Is( "major_radius" ) )
                    ParseNumber( part.AtomAt( 1 ), major );
                else if( part.Is( "minor_radius" ) )
                    ParseNumber( part.AtomAt( 1 ), minor );
                else if( part.Is( "rotation_angle" ) )
                    ParseNumber( part.AtomAt( 1 ), rotation );
                else if( part.Is( "start_angle" ) )
                    ParseNumber( part.AtomAt( 1 ), start );
                else if( part.Is( "end_angle" ) )
                    ParseNumber( part.AtomAt( 1 ), end );
                else if( arc && part.Is( "fill" ) )
                    continue;
                else
                    keep.push_back( part );
            }

            auto pts = EllipsePoints( cx, cy, ToIU( major, IU_PER_MM ), ToIU( minor, IU_PER_MM ), rotation, arc,
                                      start, end, aInvertY, MAX_ERROR );

            NODE poly = NODE::List( "polyline" );
            NODE list = NODE::List( "pts" );

            for( auto& [x, y] : pts )
                list.items.push_back( xyNode( x, y ) );

            if( !arc && !pts.empty() )
                list.items.push_back( xyNode( pts.front().first, pts.front().second ) );

            for( NODE& part : keep )
            {
                if( !part.isList )
                    poly.items.push_back( part );     // "private"
            }

            poly.items.push_back( list );

            for( NODE& part : keep )
            {
                if( part.isList )
                    poly.items.push_back( part );
            }

            item = poly;
            aCtx.report.Note( arc ? "elliptical arc -> polyline" : "ellipse -> polyline" );
        }
    };

    walk( aRoot, aCtx.kind == KIND::SYMBOL_LIB );
}


// 20260622: stacked pin numbers escape their special characters; 10.0 reads the escapes literally
static void stackedPinEscapes( NODE& aRoot, CONTEXT& aCtx )
{
    std::function<void( NODE& )> walk = [&]( NODE& aNode )
    {
        if( aNode.Is( "number" ) )
        {
            std::string text = Unquote( aNode.AtomAt( 1 ) );

            if( text.find( '[' ) != std::string::npos && text.find( '\\' ) != std::string::npos )
                aCtx.report.Note( "stacked pin number with escapes kept as written: " + text );

            return;
        }

        for( NODE& child : aNode.items )
        {
            if( child.isList )
                walk( child );
        }
    };

    walk( aRoot );
}


static void boldThickness( NODE& aRoot, CONTEXT& aCtx )
{
    UnbakeBoldThickness( aRoot, IU_PER_MM, aCtx.report );
}


static void imageScale( NODE& aRoot, CONTEXT& aCtx )
{
    RestoreImageScale( aRoot, aCtx.report );
}


static const RULE s_schematicRules[] = {
    { 20260326, "locking", nullptr },
    { 20260508, "native ellipse", ellipses },
    { 20260512, "net chains", nullptr },
    { 20260622, "escaped stacked pins", stackedPinEscapes },
    { 20260623, "reference image scale", imageScale },
    { 20260629, "pin maps", nullptr },
    { 20260722, "variant symbol_override", nullptr },
    { 20260803, "polygon closing edge", nullptr },
    { 20260818, "line endings", nullptr },
    { 20260826, "bold stroke width", boldThickness },
    { 20260830, "custom properties", nullptr },
};


static const RULE s_symbolLibRules[] = {
    { 20260508, "native ellipse", ellipses },
    { 20260622, "escaped stacked pins", stackedPinEscapes },
    { 20260629, "pin maps", nullptr },
    { 20260710, "line endings", nullptr },
    { 20260826, "bold stroke width", boldThickness },
    { 20260830, "custom properties", nullptr },
};


bool SchematicStep( NODE& aRoot, CONTEXT& aCtx, std::string& aError )
{
    (void) RULE_BUMPS_SCHEMATIC;
    (void) RULE_BUMPS_SYMBOL_LIB;

    const bool library = aCtx.kind == KIND::SYMBOL_LIB;

    if( !aRoot.Is( library ? "kicad_symbol_lib" : "kicad_sch" ) )
    {
        aError = library ? "not a symbol library file" : "not a schematic file";
        return false;
    }

    if( library )
    {
        for( const RULE& rule : s_symbolLibRules )
        {
            if( rule.apply && aCtx.version >= rule.bump )
                rule.apply( aRoot, aCtx );
        }
    }
    else
    {
        for( const RULE& rule : s_schematicRules )
        {
            if( rule.apply && aCtx.version >= rule.bump )
                rule.apply( aRoot, aCtx );
        }
    }

    DropUnknownTokens( aRoot, aCtx );
    SetHeader( aRoot, library ? SYMBOL_LIB_VERSION : SCHEMATIC_VERSION );
    return true;
}

} // namespace V10
} // namespace LEGACY_FORMATS
