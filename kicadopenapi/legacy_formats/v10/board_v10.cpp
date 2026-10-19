/// @file board_v10.cpp
/// @brief Board and footprint files: this tree's format -> 10.0 (20260206)
#include "v10_common.h"

#include <cmath>


namespace LEGACY_FORMATS
{
namespace V10
{

static constexpr long long IU_PER_MM = 1000000;
static constexpr long long MAX_ERROR = 5000;    // 5 um, the board's default arc approximation

/// Every board bump after 10.0; tools/legacy_formats_refresh.py checks this list
static const int RULE_BUMPS_BOARD[] = { 20260410, 20260508, 20260511, 20260512, 20260513, 20260521,
                                        20260603, 20260616, 20260623, 20260624, 20260728, 20260816,
                                        20260818, 20260826, 20260828, 20260830, 20260831, 20260901 };


static bool readXY( const NODE* aNode, long long& aX, long long& aY )
{
    double x = 0.0;
    double y = 0.0;

    if( !aNode || !ParseNumber( aNode->AtomAt( 1 ), x ) || !ParseNumber( aNode->AtomAt( 2 ), y ) )
        return false;

    aX = ToIU( x, IU_PER_MM );
    aY = ToIU( y, IU_PER_MM );
    return true;
}


static void writeXY( NODE& aNode, long long aX, long long aY )
{
    aNode.items[1].atom = FormatMM( aX, IU_PER_MM );
    aNode.items[2].atom = FormatMM( aY, IU_PER_MM );
}


static NODE xyNode( const char* aHead, long long aX, long long aY )
{
    NODE n = NODE::List( aHead );
    n.items.push_back( NODE::Atom( FormatMM( aX, IU_PER_MM ) ) );
    n.items.push_back( NODE::Atom( FormatMM( aY, IU_PER_MM ) ) );
    return n;
}


/// @brief Footprints in the file: the top-level list of a .kicad_mod, or the board's footprints
static std::vector<NODE*> footprints( NODE& aRoot, CONTEXT& aCtx )
{
    std::vector<NODE*> out;

    if( aCtx.kind == KIND::FOOTPRINT )
    {
        out.push_back( &aRoot );
        return out;
    }

    for( NODE& item : aRoot.items )
    {
        if( item.Is( "footprint" ) )
            out.push_back( &item );
    }

    return out;
}


// 20260616: (transform (translate) (rotate) (scale)) -> (at x y [angle]). Children stay in the
// footprint frame (text angles are absolute in both), except what 10.0 keeps in the board frame:
// zone outlines and points.
static void footprintTransform( NODE& aRoot, CONTEXT& aCtx )
{
    for( NODE* fp : footprints( aRoot, aCtx ) )
    {
        size_t index = 0;

        for( ; index < fp->items.size(); ++index )
        {
            if( fp->items[index].Is( "transform" ) )
                break;
        }

        if( index == fp->items.size() )
            continue;

        NODE&     transform = fp->items[index];
        long long px = 0;
        long long py = 0;
        double    rot = 0.0;
        double    sx = 1.0;
        double    sy = 1.0;

        readXY( transform.Child( "translate" ), px, py );

        if( const NODE* rotate = transform.Child( "rotate" ) )
            ParseNumber( rotate->AtomAt( 1 ), rot );

        if( const NODE* scale = transform.Child( "scale" ) )
        {
            ParseNumber( scale->AtomAt( 1 ), sx );
            ParseNumber( scale->AtomAt( 2 ), sy );
        }

        if( std::fabs( sx - 1.0 ) > 1e-9 || std::fabs( sy - 1.0 ) > 1e-9 )
            aCtx.report.Note( "footprint scale dropped: 10.0 has none (footprint drawn unscaled)" );

        NODE at = xyNode( "at", px, py );

        if( std::fmod( rot, 360.0 ) != 0.0 )
            at.items.push_back( NODE::Atom( FormatAngle( rot ) ) );

        fp->items[index] = at;

        auto toBoard = [&]( long long& aX, long long& aY )
        {
            RotatePoint( aX, aY, rot );
            aX += px;
            aY += py;
        };

        for( NODE& child : fp->items )
        {
            if( child.Is( "zone" ) )
            {
                for( NODE& polygon : child.items )
                {
                    if( !polygon.Is( "polygon" ) )
                        continue;

                    NODE* pts = polygon.Child( "pts" );

                    if( !pts )
                        continue;

                    for( NODE& xy : pts->items )
                    {
                        long long x = 0;
                        long long y = 0;

                        if( xy.Is( "xy" ) && readXY( &xy, x, y ) )
                        {
                            toBoard( x, y );
                            writeXY( xy, x, y );
                        }
                    }
                }
            }
            else if( child.Is( "point" ) )
            {
                NODE*     pointAt = child.Child( "at" );
                long long x = 0;
                long long y = 0;

                if( readXY( pointAt, x, y ) )
                {
                    toBoard( x, y );
                    writeXY( *pointAt, x, y );
                }
            }
        }
    }
}


// 20260508: ellipses -> a polygon (closed) or line segments (arc)
static void ellipses( NODE& aRoot, CONTEXT& aCtx )
{
    auto convert = [&]( NODE& aOwner )
    {
        std::vector<NODE> out;

        for( size_t ii = 0; ii < aOwner.items.size(); ++ii )
        {
            NODE&              item = aOwner.items[ii];
            const std::string& head = item.Head();
            bool closed = head == "gr_ellipse" || head == "fp_ellipse";
            bool arc = head == "gr_ellipse_arc" || head == "fp_ellipse_arc";

            if( ii == 0 || !( closed || arc ) )
            {
                out.push_back( std::move( item ) );
                continue;
            }

            std::string prefix = head.substr( 0, 2 );
            long long   cx = 0;
            long long   cy = 0;
            double      major = 0.0;
            double      minor = 0.0;
            double      rotation = 0.0;
            double      start = 0.0;
            double      end = 90.0;
            std::vector<NODE> keep;

            for( size_t jj = 1; jj < item.items.size(); ++jj )
            {
                NODE& part = item.items[jj];

                if( part.Is( "center" ) )
                    readXY( &part, cx, cy );
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
                else
                    keep.push_back( part );
            }

            auto pts = EllipsePoints( cx, cy, ToIU( major, IU_PER_MM ), ToIU( minor, IU_PER_MM ), rotation, arc,
                                      start, end, false, MAX_ERROR );

            if( closed )
            {
                NODE poly = NODE::List( prefix + "_poly" );
                NODE list = NODE::List( "pts" );

                for( auto& [x, y] : pts )
                    list.items.push_back( xyNode( "xy", x, y ) );

                poly.items.push_back( list );

                for( NODE& part : keep )
                    poly.items.push_back( part );

                out.push_back( poly );
                aCtx.report.Note( "ellipse -> polygon (" + std::to_string( pts.size() ) + " points)" );
            }
            else
            {
                for( size_t kk = 0; kk + 1 < pts.size(); ++kk )
                {
                    NODE line = NODE::List( prefix + "_line" );
                    line.items.push_back( xyNode( "start", pts[kk].first, pts[kk].second ) );
                    line.items.push_back( xyNode( "end", pts[kk + 1].first, pts[kk + 1].second ) );

                    for( const NODE& part : keep )
                    {
                        if( part.Is( "fill" ) )
                            continue;

                        NODE copy = part;

                        if( copy.Is( "uuid" ) && copy.items.size() > 1 && kk > 0 )
                            copy.items[1].atom = DerivedUuid( copy.items[1].atom, unsigned( kk ) );

                        line.items.push_back( copy );
                    }

                    out.push_back( line );
                }

                aCtx.report.Note( "elliptical arc -> line segments" );
            }
        }

        aOwner.items = std::move( out );
    };

    if( aCtx.kind == KIND::BOARD )
        convert( aRoot );

    for( NODE* fp : footprints( aRoot, aCtx ) )
        convert( *fp );
}


// 20260410: extruded 3D bodies: (model (type extruded) ...) has no file name
static void extrudedBodies( NODE& aRoot, CONTEXT& aCtx )
{
    for( NODE* fp : footprints( aRoot, aCtx ) )
    {
        for( size_t ii = 1; ii < fp->items.size(); )
        {
            NODE& item = fp->items[ii];

            if( item.Is( "model" ) && item.items.size() > 1 && item.items[1].isList )
            {
                fp->items.erase( fp->items.begin() + ii );
                aCtx.report.Note( "extruded 3D body dropped" );
                continue;
            }

            ++ii;
        }
    }
}


// 20260513: copper thieving fill has no 10.0 equivalent; a solid fill would flood the area
static void thievingZones( NODE& aRoot, CONTEXT& aCtx )
{
    auto drop = [&]( NODE& aOwner )
    {
        for( size_t ii = 1; ii < aOwner.items.size(); )
        {
            NODE& item = aOwner.items[ii];
            const NODE* fill = item.Is( "zone" ) ? item.Child( "fill" ) : nullptr;
            const NODE* mode = fill ? fill->Child( "mode" ) : nullptr;

            if( mode && mode->AtomAt( 1 ) == "thieving" )
            {
                aOwner.items.erase( aOwner.items.begin() + ii );
                aCtx.report.Note( "copper thieving zone dropped" );
                continue;
            }

            ++ii;
        }
    };

    if( aCtx.kind == KIND::BOARD )
        drop( aRoot );

    for( NODE* fp : footprints( aRoot, aCtx ) )
        drop( *fp );
}


// 20260603: 10.0 rejects knockout on table cells
static void tableCellKnockout( NODE& aRoot, CONTEXT& aCtx )
{
    std::function<void( NODE& )> walk = [&]( NODE& aNode )
    {
        if( aNode.Is( "table_cell" ) )
        {
            aCtx.report.Note( "table cell knockout dropped", aNode.RemoveChildren( "knockout" ) );
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


// 20260816 / 20260830: generators 10.0 cannot build (via stitching, via stacks); their member
// items stay as plain items
static void newGenerators( NODE& aRoot, CONTEXT& aCtx )
{
    auto drop = [&]( NODE& aOwner )
    {
        for( size_t ii = 1; ii < aOwner.items.size(); )
        {
            NODE&       item = aOwner.items[ii];
            const NODE* type = item.Is( "generated" ) ? item.Child( "type" ) : nullptr;

            if( type && type->AtomAt( 1 ) != "tuning_pattern" )
            {
                aCtx.report.Note( "generator '" + type->AtomAt( 1 ) + "' dropped, its items kept as plain items" );
                aOwner.items.erase( aOwner.items.begin() + ii );
                continue;
            }

            ++ii;
        }
    };

    if( aCtx.kind == KIND::BOARD )
        drop( aRoot );

    for( NODE* fp : footprints( aRoot, aCtx ) )
        drop( *fp );
}


// 20260828: exclude_from_sim footprint attribute
static void excludeFromSim( NODE& aRoot, CONTEXT& aCtx )
{
    for( NODE* fp : footprints( aRoot, aCtx ) )
    {
        if( NODE* attr = fp->Child( "attr" ) )
            aCtx.report.Note( "footprint exclude_from_sim attribute dropped", attr->RemoveAtoms( "exclude_from_sim" ) );
    }
}


static void boldThickness( NODE& aRoot, CONTEXT& aCtx )
{
    UnbakeBoldThickness( aRoot, IU_PER_MM, aCtx.report );
}


static void imageScale( NODE& aRoot, CONTEXT& aCtx )
{
    RestoreImageScale( aRoot, aCtx.report );
}


static const RULE s_rules[] = {
    { 20260410, "extruded 3D body", extrudedBodies },
    { 20260508, "native ellipse", ellipses },
    { 20260511, "dielectric models", nullptr },
    { 20260512, "net chains", nullptr },
    { 20260513, "copper thieving", thievingZones },
    { 20260521, "pad simulation types", nullptr },
    { 20260603, "table cell knockout", tableCellKnockout },
    { 20260616, "footprint transform", footprintTransform },
    { 20260623, "reference image scale", imageScale },
    { 20260624, "geometric constraints", nullptr },
    { 20260728, "grid items", nullptr },
    { 20260816, "via stitching", newGenerators },
    { 20260818, "line endings", nullptr },
    { 20260826, "bold stroke width", boldThickness },
    { 20260828, "exclude_from_sim attribute", excludeFromSim },
    { 20260830, "via stack generator", newGenerators },
    { 20260831, "custom properties", nullptr },
    { 20260901, "drill charts and maps", nullptr },
};


bool BoardStep( NODE& aRoot, CONTEXT& aCtx, std::string& aError )
{
    (void) RULE_BUMPS_BOARD;

    if( !aRoot.Is( aCtx.kind == KIND::BOARD ? "kicad_pcb" : "footprint" ) )
    {
        aError = "not a " + std::string( aCtx.kind == KIND::BOARD ? "board" : "footprint" ) + " file";
        return false;
    }

    for( const RULE& rule : s_rules )
    {
        if( rule.apply && aCtx.version >= rule.bump )
            rule.apply( aRoot, aCtx );
    }

    DropUnknownTokens( aRoot, aCtx );
    SetHeader( aRoot, BOARD_VERSION );
    return true;
}

} // namespace V10
} // namespace LEGACY_FORMATS
