/// @file sexpr_tree.cpp
/// @brief Lossless s-expression tree, change report and the number / geometry helpers the
/// rules share. Plain C++ so the rules can be tested without KiCad.
#include "legacy_formats_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <locale>
#include <sstream>


namespace LEGACY_FORMATS
{

static const std::string s_empty;
static constexpr double  PI = 3.14159265358979323846;


NODE NODE::Atom( const std::string& aText )
{
    NODE n;
    n.atom = aText;
    return n;
}


NODE NODE::List( const std::string& aHead )
{
    NODE n;
    n.isList = true;
    n.items.push_back( Atom( aHead ) );
    return n;
}


const std::string& NODE::Head() const
{
    if( isList && !items.empty() && !items[0].isList )
        return items[0].atom;

    return s_empty;
}


NODE* NODE::Child( const char* aHead )
{
    for( NODE& item : items )
    {
        if( item.Is( aHead ) )
            return &item;
    }

    return nullptr;
}


const NODE* NODE::Child( const char* aHead ) const
{
    for( const NODE& item : items )
    {
        if( item.Is( aHead ) )
            return &item;
    }

    return nullptr;
}


const std::string& NODE::AtomAt( size_t aIndex ) const
{
    if( isList && aIndex < items.size() && !items[aIndex].isList )
        return items[aIndex].atom;

    return s_empty;
}


bool NODE::HasAtom( const char* aAtom ) const
{
    for( size_t ii = 1; ii < items.size(); ++ii )
    {
        if( !items[ii].isList && items[ii].atom == aAtom )
            return true;
    }

    return false;
}


int NODE::RemoveChildren( const char* aHead )
{
    size_t before = items.size();
    items.erase( std::remove_if( items.begin() + ( items.empty() ? 0 : 1 ), items.end(),
                                 [&]( const NODE& n ) { return n.Is( aHead ); } ),
                 items.end() );
    return int( before - items.size() );
}


int NODE::RemoveAtoms( const char* aAtom )
{
    size_t before = items.size();
    items.erase( std::remove_if( items.begin() + ( items.empty() ? 0 : 1 ), items.end(),
                                 [&]( const NODE& n ) { return !n.isList && n.atom == aAtom; } ),
                 items.end() );
    return int( before - items.size() );
}


static bool isSpace( char c )
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}


bool Parse( const std::string& aText, NODE& aRoot, std::string& aError )
{
    std::vector<NODE> stack;
    bool              haveRoot = false;
    size_t            i = 0;
    const size_t      n = aText.size();

    while( i < n )
    {
        char c = aText[i];

        if( isSpace( c ) )
        {
            ++i;
        }
        else if( c == '(' )
        {
            if( haveRoot && stack.empty() )
            {
                aError = "text after the top-level list";
                return false;
            }

            NODE list;
            list.isList = true;
            stack.push_back( std::move( list ) );
            ++i;
        }
        else if( c == ')' )
        {
            if( stack.empty() )
            {
                aError = "unbalanced ')' at offset " + std::to_string( i );
                return false;
            }

            NODE done = std::move( stack.back() );
            stack.pop_back();

            if( stack.empty() )
            {
                aRoot = std::move( done );
                haveRoot = true;
            }
            else
            {
                stack.back().items.push_back( std::move( done ) );
            }

            ++i;
        }
        else
        {
            if( stack.empty() )
            {
                aError = "atom outside a list at offset " + std::to_string( i );
                return false;
            }

            size_t start = i;

            if( c == '"' )
            {
                ++i;

                while( i < n && aText[i] != '"' )
                    i += aText[i] == '\\' ? 2 : 1;

                if( i >= n )
                {
                    aError = "unterminated string at offset " + std::to_string( start );
                    return false;
                }

                ++i;
            }
            else
            {
                while( i < n && !isSpace( aText[i] ) && aText[i] != '(' && aText[i] != ')' )
                    ++i;
            }

            stack.back().items.push_back( NODE::Atom( aText.substr( start, i - start ) ) );
        }
    }

    if( !stack.empty() || !haveRoot )
    {
        aError = "unbalanced '(' or empty file";
        return false;
    }

    return true;
}


static void writeNode( const NODE& aNode, std::string& aOut )
{
    if( !aNode.isList )
    {
        aOut += aNode.atom;
        return;
    }

    aOut += '(';

    for( size_t ii = 0; ii < aNode.items.size(); ++ii )
    {
        if( ii )
            aOut += ' ';

        writeNode( aNode.items[ii], aOut );
    }

    aOut += ')';
}


std::string Write( const NODE& aNode )
{
    std::string out;
    writeNode( aNode, out );
    out += '\n';
    return out;
}


std::string Unquote( const std::string& aRaw )
{
    if( aRaw.size() < 2 || aRaw.front() != '"' || aRaw.back() != '"' )
        return aRaw;

    std::string out;

    for( size_t ii = 1; ii + 1 < aRaw.size(); ++ii )
    {
        char c = aRaw[ii];

        if( c == '\\' && ii + 2 < aRaw.size() )
        {
            char e = aRaw[++ii];
            out += e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e;
        }
        else
        {
            out += c;
        }
    }

    return out;
}


std::string Quote( const std::string& aText )
{
    std::string out = "\"";

    for( char c : aText )
    {
        if( c == '"' || c == '\\' )
        {
            out += '\\';
            out += c;
        }
        else if( c == '\n' )
        {
            out += "\\n";
        }
        else if( c == '\r' )
        {
            out += "\\r";
        }
        else
        {
            out += c;
        }
    }

    return out + "\"";
}


void REPORT::Note( const std::string& aWhat, int aCount )
{
    if( aCount <= 0 )
        return;

    if( !m_counts.count( aWhat ) )
        m_order.push_back( aWhat );

    m_counts[aWhat] += aCount;
}


std::vector<std::string> REPORT::Lines() const
{
    std::vector<std::string> out;

    for( const std::string& what : m_order )
    {
        int count = m_counts.at( what );
        out.push_back( count > 1 ? what + " (x" + std::to_string( count ) + ")" : what );
    }

    return out;
}


bool ParseNumber( const std::string& aAtom, double& aValue )
{
    if( aAtom.empty() || aAtom[0] == '"' )
        return false;

    std::istringstream in( aAtom );
    in.imbue( std::locale::classic() );
    in >> aValue;
    return !in.fail() && in.peek() == std::char_traits<char>::eof();
}


long long ToIU( double aMM, long long aIuPerMm )
{
    return std::llround( aMM * double( aIuPerMm ) );
}


static std::string formatFixed( long long aValue, long long aDenominator )
{
    std::string out = aValue < 0 ? "-" : "";
    unsigned long long v = aValue < 0 ? 0ULL - (unsigned long long) aValue : (unsigned long long) aValue;
    out += std::to_string( v / aDenominator );

    unsigned long long frac = v % aDenominator;

    if( frac )
    {
        std::string digits = std::to_string( frac );
        int         width = 0;

        for( long long d = aDenominator; d > 1; d /= 10 )
            ++width;

        digits = std::string( width - digits.size(), '0' ) + digits;

        while( !digits.empty() && digits.back() == '0' )
            digits.pop_back();

        out += "." + digits;
    }

    return out == "-0" ? "0" : out;
}


std::string FormatMM( long long aIu, long long aIuPerMm )
{
    return formatFixed( aIu, aIuPerMm );
}


std::string FormatAngle( double aDegrees )
{
    return formatFixed( std::llround( aDegrees * 1e6 ), 1000000 );
}


void RotatePoint( long long& aX, long long& aY, double aDegrees )
{
    double angle = std::fmod( aDegrees, 360.0 );

    if( angle < 0 )
        angle += 360.0;

    long long x = aX;
    long long y = aY;

    if( angle == 0.0 )
        return;

    if( angle == 90.0 )
    {
        aX = y;
        aY = -x;
    }
    else if( angle == 180.0 )
    {
        aX = -x;
        aY = -y;
    }
    else if( angle == 270.0 )
    {
        aX = -y;
        aY = x;
    }
    else
    {
        double rad = angle * PI / 180.0;
        double s = std::sin( rad );
        double c = std::cos( rad );
        aX = std::llround( y * s + x * c );
        aY = std::llround( y * c - x * s );
    }
}


bool Base64Decode( const std::string& aText, std::string& aBytes )
{
    auto value = []( char c ) -> int
    {
        if( c >= 'A' && c <= 'Z' )
            return c - 'A';
        if( c >= 'a' && c <= 'z' )
            return c - 'a' + 26;
        if( c >= '0' && c <= '9' )
            return c - '0' + 52;
        if( c == '+' )
            return 62;
        if( c == '/' )
            return 63;
        return -1;
    };

    aBytes.clear();
    unsigned int buffer = 0;
    int          bits = 0;

    for( char c : aText )
    {
        if( isSpace( c ) || c == '=' )
            continue;

        int v = value( c );

        if( v < 0 )
            return false;

        buffer = ( buffer << 6 ) | unsigned( v );
        bits += 6;

        if( bits >= 8 )
        {
            bits -= 8;
            aBytes += char( ( buffer >> bits ) & 0xFF );
        }
    }

    return true;
}


bool ImageDensity( const std::string& aBytes, int& aPpi, int& aLegacyPpi )
{
    static const char sig[] = "\x89PNG\r\n\x1a\n";

    if( aBytes.size() < 8 || aBytes.compare( 0, 8, sig, 8 ) != 0 )
        return false;   // only PNG carries a fractional pixels-per-cm density

    auto be32 = [&]( size_t aAt ) -> uint32_t
    {
        return ( uint32_t( uint8_t( aBytes[aAt] ) ) << 24 ) | ( uint32_t( uint8_t( aBytes[aAt + 1] ) ) << 16 )
               | ( uint32_t( uint8_t( aBytes[aAt + 2] ) ) << 8 ) | uint32_t( uint8_t( aBytes[aAt + 3] ) );
    };

    size_t at = 8;

    while( at + 8 <= aBytes.size() )
    {
        uint32_t    length = be32( at );
        std::string type = aBytes.substr( at + 4, 4 );

        if( type == "pHYs" && length >= 9 && at + 8 + 9 <= aBytes.size() )
        {
            uint32_t ppm = be32( at + 8 );
            uint8_t  unit = uint8_t( aBytes[at + 16] );

            if( unit != 1 )
                return false;   // unknown unit: both readers take the value as is

            double perCm = ppm / 100.0;

            if( perCm <= 1.0 )
                return false;

            aPpi = int( std::llround( perCm * 2.54 ) );
            long long truncated = (long long) perCm;
            aLegacyPpi = truncated > 1 ? int( std::llround( truncated * 2.54 ) ) : aPpi;
            return true;
        }

        if( type == "IDAT" || type == "IEND" )
            return false;

        at += 12 + length;
    }

    return false;
}


std::vector<std::pair<long long, long long>> EllipsePoints( long long aCx, long long aCy, long long aMajor,
                                                            long long aMinor, double aRotation, bool aArc,
                                                            double aStart, double aEnd, bool aInvertY,
                                                            long long aMaxError )
{
    double start = 0.0;
    double end = 360.0;

    if( aArc )
    {
        auto normalize = []( double a )
        {
            a = std::fmod( a, 360.0 );
            return a < 0 ? a + 360.0 : a;
        };

        start = aStart;
        end = aEnd;

        if( end == start )
            end = start + 360.0;

        if( start > end )
        {
            if( end < 0 )
                end = normalize( end );
            else
                start = normalize( start ) - 360.0;
        }
    }

    double a = double( std::max( aMajor, 1LL ) );
    double b = double( std::max( aMinor, 1LL ) );
    double step = 2.0 * std::acos( std::max( -1.0, 1.0 - double( aMaxError ) / a ) ) * 180.0 / PI;
    step = std::min( std::max( step, 0.5 ), 15.0 );
    int segments = std::max( aArc ? 2 : 8, int( std::ceil( ( end - start ) / step ) ) );

    double rot = aRotation * PI / 180.0;
    double cr = std::cos( rot );
    double sr = std::sin( rot );
    double cy = aInvertY ? -double( aCy ) : double( aCy );

    std::vector<std::pair<long long, long long>> pts;
    int count = aArc ? segments + 1 : segments;

    for( int ii = 0; ii < count; ++ii )
    {
        double t = ( start + ( end - start ) * ii / segments ) * PI / 180.0;
        double x = a * std::cos( t );
        double y = b * std::sin( t );
        double px = double( aCx ) + x * cr - y * sr;
        double py = cy + x * sr + y * cr;
        pts.emplace_back( std::llround( px ), std::llround( aInvertY ? -py : py ) );
    }

    return pts;
}


void DropUnknownTokens( NODE& aRoot, CONTEXT& aCtx )
{
    using WORDS = std::vector<const std::set<std::string>*>;

    const bool  board = aCtx.kind == KIND::BOARD || aCtx.kind == KIND::FOOTPRINT;
    const auto& main = TargetKeywords( aCtx.target, board ? "pcb" : "sch" );
    const auto& plot = TargetKeywords( aCtx.target, "plot" );
    const auto& stroke = TargetKeywords( aCtx.target, "stroke" );
    const auto& embedded = TargetKeywords( aCtx.target, "embedded" );

    auto known = []( const WORDS& aWords, const std::string& aHead )
    {
        for( const std::set<std::string>* words : aWords )
        {
            if( words->count( aHead ) )
                return true;
        }

        return false;
    };

    std::function<void( NODE&, const WORDS& )> walk = [&]( NODE& aNode, const WORDS& aWords )
    {
        for( size_t ii = 1; ii < aNode.items.size(); )
        {
            NODE& child = aNode.items[ii];

            if( !child.isList )
            {
                ++ii;
                continue;
            }

            const std::string& head = child.Head();
            bool symbol = !head.empty() && head[0] != '"' && !( head[0] >= '0' && head[0] <= '9' )
                          && head[0] != '-';

            if( symbol && !known( aWords, head ) )
            {
                aCtx.report.Note( "dropped (" + head + ") in (" + aNode.Head() + "): not in " + aCtx.target );
                aNode.items.erase( aNode.items.begin() + ii );
                continue;
            }

            if( board && head == "generated" )
            {
                ++ii;   // free-form generator properties
                continue;
            }

            if( head == "pcbplotparams" )
                walk( child, { &plot } );
            else if( head == "stroke" )
                walk( child, { &main, &stroke } );
            else if( head == "embedded_files" || head == "embedded_fonts" )
                walk( child, { &main, &embedded } );
            else
                walk( child, aWords );

            ++ii;
        }
    };

    walk( aRoot, { &main } );
}

} // namespace LEGACY_FORMATS
