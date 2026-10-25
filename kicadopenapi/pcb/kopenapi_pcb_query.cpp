/// @file kopenapi_pcb_query.cpp
/// @brief pcb_query engine: compile (KiCad's DRC expression language + query functions), scan,
/// evaluate, shape (select / group / aggregate / order / page / explain).

#include "kopenapi_pcb_query.h"

#include <board.h>
#include <footprint.h>
#include <kicadopenapi_keepalive.h>
#include <pad.h>
#include <pcb_group.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <pcb_textbox.h>
#include <pcb_track.h>
#include <pcbexpr_evaluator.h>
#include <properties/property.h>
#include <properties/property_mgr.h>
#include <string_utils.h>
#include <zone.h>
#include <base_units.h>
#include <geometry/shape_poly_set.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <unordered_map>


namespace
{

using CLOCK = std::chrono::steady_clock;

double msSince( CLOCK::time_point aStart )
{
    return std::chrono::duration<double, std::milli>( CLOCK::now() - aStart ).count();
}


double mmOf( double aIu )
{
    return aIu / pcbIUScale.IU_PER_MM;
}


// ---------------------------------------------------------------------------------------------
// Query-only functions (A.<name>(...)); the self argument is the PCBEXPR_VAR_REF of the item
// ---------------------------------------------------------------------------------------------

BOARD_ITEM* itemOf( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    PCBEXPR_VAR_REF* vref = static_cast<PCBEXPR_VAR_REF*>( aSelf );
    return vref ? vref->GetObject( aCtx ) : nullptr;
}


const FOOTPRINT* footprintOf( const BOARD_ITEM* aItem )
{
    if( !aItem )
        return nullptr;

    if( aItem->Type() == PCB_FOOTPRINT_T )
        return static_cast<const FOOTPRINT*>( aItem );

    return aItem->GetParentFootprint();
}


/// Pop aCount arguments into a vector in call order.  The compiler's preflight pushes literal
/// arguments left to right (as strings); generated code evaluates the parameter list from a stack,
/// so at run time the first argument is pushed last.
std::vector<LIBEVAL::VALUE*> popArgs( LIBEVAL::CONTEXT* aCtx, int aCount )
{
    std::vector<LIBEVAL::VALUE*> popped( aCount, nullptr );

    for( int i = 0; i < aCount; ++i )
        popped[i] = aCtx->Pop();

    const bool preflight = std::all_of( popped.begin(), popped.end(),
                                        []( const LIBEVAL::VALUE* v ) { return v && v->GetType() == LIBEVAL::VT_STRING; } );

    if( preflight )
        std::reverse( popped.begin(), popped.end() );

    return popped;
}


/// A coordinate argument in IU: plain numbers are mm, numbers with a unit are already converted
double argIu( const LIBEVAL::VALUE* aValue )
{
    if( !aValue )
        return 0.0;

    if( aValue->GetType() == LIBEVAL::VT_STRING )
        return wxAtof( aValue->AsString() ) * pcbIUScale.IU_PER_MM;

    if( aValue->GetUnits() != EDA_UNITS::UNSCALED )
        return aValue->AsDouble();

    return aValue->AsDouble() * pcbIUScale.IU_PER_MM;
}


void pushString( LIBEVAL::CONTEXT* aCtx, const wxString& aValue )
{
    LIBEVAL::VALUE* result = aCtx->AllocValue();
    result->Set( aValue );
    aCtx->Push( result );
}


void pushNumber( LIBEVAL::CONTEXT* aCtx, double aValue )
{
    LIBEVAL::VALUE* result = aCtx->AllocValue();
    result->Set( aValue );
    aCtx->Push( result );
}


wxString sideOf( const BOARD_ITEM* aItem )
{
    if( const FOOTPRINT* fp = footprintOf( aItem ) )
        return fp->IsFlipped() ? wxS( "bottom" ) : wxS( "top" );

    if( !aItem )
        return wxEmptyString;

    const LSET layers = aItem->GetLayerSet();
    const bool front = ( layers & LSET::FrontMask() ).any();
    const bool back = ( layers & LSET::BackMask() ).any();

    return front && back ? wxS( "both" ) : back ? wxS( "bottom" ) : front ? wxS( "top" ) : wxS( "inner" );
}


wxString mountOf( const FOOTPRINT* aFp )
{
    if( !aFp )
        return wxEmptyString;

    if( aFp->GetAttributes() & FP_THROUGH_HOLE )
        return wxS( "tht" );

    if( aFp->GetAttributes() & FP_SMD )
        return wxS( "smd" );

    bool smd = false;

    for( const PAD* pad : aFp->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::PTH )
            return wxS( "tht" );

        smd |= pad->GetAttribute() == PAD_ATTRIB::SMD;
    }

    return smd ? wxS( "smd" ) : wxS( "other" );
}


/// The item's outline for region tests: a footprint's courtyard on its side, else its box
SHAPE_POLY_SET outlineOf( const BOARD_ITEM* aItem )
{
    if( aItem->Type() == PCB_FOOTPRINT_T )
    {
        const FOOTPRINT*      fp = static_cast<const FOOTPRINT*>( aItem );
        const SHAPE_POLY_SET& courtyard = fp->GetCourtyard( fp->IsFlipped() ? B_CrtYd : F_CrtYd );

        if( courtyard.OutlineCount() > 0 )
            return courtyard;
    }

    const BOX2I    box = aItem->GetBoundingBox();
    SHAPE_POLY_SET poly;
    poly.NewOutline();
    poly.Append( box.GetLeft(), box.GetTop() );
    poly.Append( box.GetRight(), box.GetTop() );
    poly.Append( box.GetRight(), box.GetBottom() );
    poly.Append( box.GetLeft(), box.GetBottom() );
    return poly;
}


BOX2I boxOf( const BOARD_ITEM* aItem )
{
    if( aItem->Type() == PCB_FOOTPRINT_T )
    {
        const FOOTPRINT*      fp = static_cast<const FOOTPRINT*>( aItem );
        const SHAPE_POLY_SET& courtyard = fp->GetCourtyard( fp->IsFlipped() ? B_CrtYd : F_CrtYd );

        if( courtyard.OutlineCount() > 0 )
            return courtyard.BBox();
    }

    return aItem->GetBoundingBox();
}


void sideFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    pushString( aCtx, sideOf( itemOf( aCtx, aSelf ) ) );
}


void mountFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    pushString( aCtx, mountOf( footprintOf( itemOf( aCtx, aSelf ) ) ) );
}


void parentRefFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    const FOOTPRINT* fp = footprintOf( itemOf( aCtx, aSelf ) );
    pushString( aCtx, fp ? fp->GetReference() : wxString() );
}


/// @brief A track's or arc's length in mm; 0 for anything else
void lengthFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    BOARD_ITEM* item = itemOf( aCtx, aSelf );
    const bool  track = item && ( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T );
    pushNumber( aCtx, track ? mmOf( static_cast<PCB_TRACK*>( item )->GetLength() ) : 0.0 );
}


void courtyardFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    BOARD_ITEM* item = itemOf( aCtx, aSelf );

    if( !item )
    {
        pushString( aCtx, wxEmptyString );
        return;
    }

    const BOX2I box = boxOf( item );
    pushString( aCtx, wxString::Format( wxS( "%.4f %.4f %.4f %.4f" ), mmOf( box.GetLeft() ), mmOf( box.GetTop() ),
                                        mmOf( box.GetRight() ), mmOf( box.GetBottom() ) ) );
}


BOX2I rectArg( const std::vector<LIBEVAL::VALUE*>& aArgs )
{
    const double x0 = argIu( aArgs[0] ), y0 = argIu( aArgs[1] ), x1 = argIu( aArgs[2] ), y1 = argIu( aArgs[3] );
    BOX2I        rect( VECTOR2I( KiROUND( std::min( x0, x1 ) ), KiROUND( std::min( y0, y1 ) ) ),
                       VECTOR2L( KiROUND( std::abs( x1 - x0 ) ), KiROUND( std::abs( y1 - y0 ) ) ) );
    return rect;
}


void inRectFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    std::vector<LIBEVAL::VALUE*> args = popArgs( aCtx, 4 );
    BOARD_ITEM*                  item = itemOf( aCtx, aSelf );
    pushNumber( aCtx, item && rectArg( args ).Contains( boxOf( item ) ) ? 1.0 : 0.0 );
}


void touchesRectFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    std::vector<LIBEVAL::VALUE*> args = popArgs( aCtx, 4 );
    BOARD_ITEM*                  item = itemOf( aCtx, aSelf );
    pushNumber( aCtx, item && rectArg( args ).Intersects( boxOf( item ) ) ? 1.0 : 0.0 );
}


/// 'x0 y0, x1 y1, ...' in mm
bool parsePolygon( const wxString& aText, SHAPE_POLY_SET& aPoly )
{
    aPoly.RemoveAllContours();
    aPoly.NewOutline();

    for( const wxString& point : wxSplit( aText, ',' ) )
    {
        double x = 0.0, y = 0.0;
        wxString xs = point.Strip( wxString::both ).BeforeFirst( ' ' );
        wxString ys = point.Strip( wxString::both ).AfterFirst( ' ' ).Strip( wxString::both );

        if( !xs.ToCDouble( &x ) || !ys.ToCDouble( &y ) )
            return false;

        aPoly.Append( KiROUND( x * pcbIUScale.IU_PER_MM ), KiROUND( y * pcbIUScale.IU_PER_MM ) );
    }

    return aPoly.OutlineCount() == 1 && aPoly.COutline( 0 ).PointCount() >= 3;
}


void inPolygonFunc( LIBEVAL::CONTEXT* aCtx, void* aSelf )
{
    LIBEVAL::VALUE* arg = aCtx->Pop();
    BOARD_ITEM*     item = itemOf( aCtx, aSelf );
    SHAPE_POLY_SET  region;

    if( !arg || !parsePolygon( arg->AsString(), region ) )
    {
        if( aCtx->HasErrorCallback() )
            aCtx->ReportError( _( "inPolygon() needs 'x0 y0, x1 y1, x2 y2, ...' in mm" ) );

        pushNumber( aCtx, 0.0 );
        return;
    }

    if( !item )
    {
        pushNumber( aCtx, 0.0 );
        return;
    }

    if( !region.BBox().Contains( boxOf( item ) ) )
    {
        pushNumber( aCtx, 0.0 );
        return;
    }

    SHAPE_POLY_SET rest = outlineOf( item );
    rest.BooleanSubtract( region );
    pushNumber( aCtx, rest.Area() < 1.0 ? 1.0 : 0.0 );
}


struct QUERY_FUNCTION
{
    wxString               signature;
    LIBEVAL::FUNC_CALL_REF call;
};


const std::map<wxString, QUERY_FUNCTION>& queryFunctions()
{
    static const std::map<wxString, QUERY_FUNCTION> functions = [] {
        std::map<wxString, QUERY_FUNCTION> f;
        auto add = [&]( const wxString& aSignature, LIBEVAL::FUNC_CALL_REF aCall )
        {
            f[aSignature.BeforeFirst( '(' ).Lower()] = { aSignature, std::move( aCall ) };
        };

        add( wxS( "side()" ), sideFunc );
        add( wxS( "mount()" ), mountFunc );
        add( wxS( "parentRef()" ), parentRefFunc );
        add( wxS( "courtyard()" ), courtyardFunc );
        add( wxS( "length()" ), lengthFunc );
        add( wxS( "inRect(x0, y0, x1, y1)" ), inRectFunc );
        add( wxS( "touchesRect(x0, y0, x1, y1)" ), touchesRectFunc );
        add( wxS( "inPolygon('x0 y0, x1 y1, ...')" ), inPolygonFunc );
        return f;
    }();

    return functions;
}


bool layerNameMatches( const BOARD* aBoard, PCB_LAYER_ID aLayer, const LIBEVAL::VALUE* aPattern )
{
    const wxString& pattern = aPattern->AsString();

    for( const wxString& name : { LSET::Name( aLayer ), aBoard ? aBoard->GetLayerName( aLayer ) : wxString() } )
    {
        if( name.IsEmpty() )
            continue;

        if( aPattern->StringIsWildcard() ? WildCompareString( pattern, name, false ) : name.IsSameAs( pattern, false ) )
            return true;
    }

    return false;
}


/// A zone's copper layers as one value: equal to a layer name when the zone is on that layer
class ZONE_LAYERS_VALUE : public LIBEVAL::VALUE
{
public:
    explicit ZONE_LAYERS_VALUE( const ZONE* aZone ) :
            LIBEVAL::VALUE( aZone->GetBoard() ? aZone->GetBoard()->GetLayerName( aZone->GetFirstLayer() )
                                              : LSET::Name( aZone->GetFirstLayer() ) ),
            m_zone( aZone )
    {}

    bool EqualTo( LIBEVAL::CONTEXT* aCtx, const VALUE* b ) const override
    {
        if( !b || b->GetType() != LIBEVAL::VT_STRING )
            return LIBEVAL::VALUE::EqualTo( aCtx, b );

        for( PCB_LAYER_ID layer : m_zone->GetLayerSet().Seq() )
        {
            if( layerNameMatches( m_zone->GetBoard(), layer, b ) )
                return true;
        }

        return false;
    }

    bool NotEqualTo( LIBEVAL::CONTEXT* aCtx, const VALUE* b ) const override
    {
        if( !b || b->GetType() == LIBEVAL::VT_UNDEFINED )
            return false;

        return !EqualTo( aCtx, b );
    }

private:
    const ZONE* m_zone;
};


/// "Layer" on A/B: KiCad exposes it only for thieving zones; zones answer with their layer set
class ZONE_LAYER_REF : public LIBEVAL::VAR_REF
{
public:
    explicit ZONE_LAYER_REF( std::unique_ptr<LIBEVAL::VAR_REF> aInner ) :
            m_inner( std::move( aInner ) )
    {}

    LIBEVAL::VAR_TYPE_T GetType() const override { return LIBEVAL::VT_STRING; }

    LIBEVAL::VALUE* GetValue( LIBEVAL::CONTEXT* aCtx ) override
    {
        if( PCBEXPR_VAR_REF* ref = dynamic_cast<PCBEXPR_VAR_REF*>( m_inner.get() ) )
        {
            if( const ZONE* zone = dynamic_cast<const ZONE*>( ref->GetObject( aCtx ) ) )
                return new ZONE_LAYERS_VALUE( zone );
        }

        return m_inner->GetValue( aCtx );
    }

private:
    std::unique_ptr<LIBEVAL::VAR_REF> m_inner;
};


/// KiCad's DRC bindings plus the query functions; KiCad's own UCODE is final, so it is composed
class KOPENAPI_QUERY_UCODE : public LIBEVAL::UCODE
{
public:
    std::unique_ptr<LIBEVAL::VAR_REF> CreateVarRef( const wxString& aVar, const wxString& aField ) override
    {
        std::unique_ptr<LIBEVAL::VAR_REF> ref = m_pcb.CreateVarRef( aVar, aField );

        if( ref && ( aVar == wxS( "A" ) || aVar == wxS( "B" ) ) && aField.CmpNoCase( wxS( "Layer" ) ) == 0 )
            return std::make_unique<ZONE_LAYER_REF>( std::move( ref ) );

        return ref;
    }

    LIBEVAL::FUNC_CALL_REF CreateFuncCall( const wxString& aName ) override
    {
        const wxString lower = aName.Lower();

        if( lower == wxS( "inarea" ) )
            return m_pcb.CreateFuncCall( wxS( "enclosedByArea" ) );

        auto it = queryFunctions().find( lower );

        if( it != queryFunctions().end() )
            return it->second.call;

        return m_pcb.CreateFuncCall( aName );
    }

private:
    PCBEXPR_UCODE m_pcb;
};


// ---------------------------------------------------------------------------------------------
// Compilation (cached by text) and errors with suggestions
// ---------------------------------------------------------------------------------------------

struct COMPILE_ERROR
{
    std::string message;
    int         offset = -1;
};


size_t editDistance( const std::string& a, const std::string& b )
{
    std::vector<size_t> prev( b.size() + 1 ), cur( b.size() + 1 );

    for( size_t j = 0; j <= b.size(); ++j )
        prev[j] = j;

    for( size_t i = 1; i <= a.size(); ++i )
    {
        cur[0] = i;

        for( size_t j = 1; j <= b.size(); ++j )
        {
            const bool same = std::tolower( (unsigned char) a[i - 1] ) == std::tolower( (unsigned char) b[j - 1] );
            cur[j] = std::min( { prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + ( same ? 0 : 1 ) } );
        }

        std::swap( prev, cur );
    }

    return prev[b.size()];
}


/// Property names of the queried kinds (spaces as underscores) and function names
std::vector<std::string> knownNames()
{
    static std::vector<std::string> names = [] {
        std::set<std::string> set;
        PROPERTY_MANAGER&     propMgr = PROPERTY_MANAGER::Instance();

        for( TYPE_ID type : { TYPE_HASH( FOOTPRINT ), TYPE_HASH( PAD ), TYPE_HASH( PCB_TRACK ), TYPE_HASH( PCB_ARC ),
                              TYPE_HASH( PCB_VIA ), TYPE_HASH( ZONE ), TYPE_HASH( PCB_SHAPE ), TYPE_HASH( PCB_TEXT ),
                              TYPE_HASH( PCB_TEXTBOX ), TYPE_HASH( PCB_GROUP ) } )
        {
            for( PROPERTY_BASE* property : propMgr.GetProperties( type ) )
            {
                wxString name = property->Name();
                name.Replace( wxS( " " ), wxS( "_" ) );
                set.insert( name.ToStdString() );
            }
        }

        for( const auto& [name, fn] : queryFunctions() )
            set.insert( fn.signature.BeforeFirst( '(' ).ToStdString() + "()" );

        for( const wxString& signature : PCBEXPR_BUILTIN_FUNCTIONS::Instance().GetSignatures() )
            set.insert( signature.BeforeFirst( '(' ).ToStdString() + "()" );

        return std::vector<std::string>( set.begin(), set.end() );
    }();

    return names;
}


std::vector<std::string> suggestions( const std::string& aMessage )
{
    // "Unrecognized property 'Heigth'" / "Unrecognized function 'sid'"; close by spelling, or
    // containing the word (Number: Pad_Number)
    const size_t open = aMessage.find( '\'' );
    const size_t close = open == std::string::npos ? open : aMessage.find( '\'', open + 1 );

    if( close == std::string::npos )
        return {};

    const std::string word = aMessage.substr( open + 1, close - open - 1 );
    std::vector<std::pair<size_t, std::string>> scored;

    for( const std::string& name : knownNames() )
    {
        std::string bare = name;

        if( bare.size() > 2 && bare.compare( bare.size() - 2, 2, "()" ) == 0 )
            bare.resize( bare.size() - 2 );

        const size_t d = editDistance( word, bare );

        if( d <= std::max<size_t>( 2, word.size() / 3 ) || ( word.size() >= 3 && wxString::FromUTF8( bare ).Lower().Contains( wxString::FromUTF8( word ).Lower() ) ) )
            scored.emplace_back( d, name );
    }

    std::sort( scored.begin(), scored.end() );
    std::vector<std::string> out;

    for( size_t i = 0; i < scored.size() && i < 5; ++i )
        out.push_back( scored[i].second );

    return out;
}


using PROGRAM = std::shared_ptr<KOPENAPI_QUERY_UCODE>;


/// @brief Plain numbers in inRect( ... ) / touchesRect( ... ) are millimetres: KiCad's expression
/// grammar takes a function argument as a string or a number with units only
std::string withRectUnits( const std::string& aExpression )
{
    static const std::regex call( R"(((?:inRect|touchesRect)\s*\()([^()]*)(\)))" );
    static const std::regex bare( R"(^\s*-?\d+(?:\.\d+)?\s*$)" );

    std::string out;
    auto        from = aExpression.cbegin();

    for( std::sregex_iterator it( aExpression.begin(), aExpression.end(), call ), end; it != end; ++it )
    {
        const std::smatch& m = *it;
        out.append( from, m[0].first );
        out += m[1].str();

        std::stringstream args( m[2].str() );
        std::string       arg;
        bool              first = true;

        while( std::getline( args, arg, ',' ) )
        {
            out += ( first ? "" : "," ) + arg + ( std::regex_match( arg, bare ) ? "mm" : "" );
            first = false;
        }

        out += m[3].str();
        from = m[0].second;
    }

    out.append( from, aExpression.cend() );
    return out;
}


/// @brief A function called with no item ("inRect(...)") applies to A: KiCad's grammar has no
/// bare calls (it reports "Unknown parent of function parameters")
std::string withItem( const std::string& aExpression )
{
    std::string out;
    char        quote = 0;

    for( size_t i = 0; i < aExpression.size(); ++i )
    {
        const char ch = aExpression[i];

        if( quote )
        {
            quote = ch == quote ? 0 : quote;
            out += ch;
            continue;
        }

        if( ch == '\'' || ch == '"' )
        {
            quote = ch;
            out += ch;
            continue;
        }

        const bool starts = ( std::isalpha( (unsigned char) ch ) || ch == '_' )
                            && ( i == 0 || !( std::isalnum( (unsigned char) aExpression[i - 1] ) || aExpression[i - 1] == '_' ) );

        if( !starts )
        {
            out += ch;
            continue;
        }

        size_t end = i;

        while( end < aExpression.size() && ( std::isalnum( (unsigned char) aExpression[end] ) || aExpression[end] == '_' ) )
            ++end;

        size_t next = end;

        while( next < aExpression.size() && std::isspace( (unsigned char) aExpression[next] ) )
            ++next;

        size_t prev = i;

        while( prev > 0 && std::isspace( (unsigned char) aExpression[prev - 1] ) )
            --prev;

        if( next < aExpression.size() && aExpression[next] == '(' && ( prev == 0 || aExpression[prev - 1] != '.' ) )
            out += "A.";

        out.append( aExpression, i, end - i );
        i = end - 1;
    }

    return out;
}


PROGRAM compile( const std::string& aRawExpression, COMPILE_ERROR& aError )
{
    const std::string aExpression = withRectUnits( withItem( aRawExpression ) );

    static std::list<std::pair<std::string, PROGRAM>>                                           lru;
    static std::unordered_map<std::string, std::list<std::pair<std::string, PROGRAM>>::iterator> index;

    if( auto it = index.find( aExpression ); it != index.end() )
    {
        lru.splice( lru.begin(), lru, it->second );
        return it->second->second;
    }

    PCBEXPR_COMPILER compiler( new PCBEXPR_UNIT_RESOLVER() );
    compiler.SetErrorCallback(
            [&]( const wxString& aMessage, int aOffset )
            {
                if( aError.message.empty() )
                {
                    aError.message = aMessage.BeforeFirst( '|' ).ToStdString();
                    aError.offset = aOffset;
                }
            } );

    auto            program = std::make_shared<KOPENAPI_QUERY_UCODE>();
    PCBEXPR_CONTEXT preflight( 0, F_Cu );
    const bool      ok = compiler.Compile( wxString::FromUTF8( aExpression ), program.get(), &preflight );

    if( !ok || !aError.message.empty() )
    {
        if( aError.message.empty() )
            aError.message = "malformed expression";

        return nullptr;
    }

    lru.emplace_front( aExpression, program );
    index[aExpression] = lru.begin();

    if( lru.size() > 64 )
    {
        index.erase( lru.back().first );
        lru.pop_back();
    }

    return program;
}


/// Top-level `&&` terms of a condition (outside parentheses and quotes)
std::vector<std::string> splitAnd( const std::string& aExpression )
{
    std::vector<std::string> terms;
    int                      depth = 0;
    char                     quote = 0;
    size_t                   start = 0;

    for( size_t i = 0; i < aExpression.size(); ++i )
    {
        const char c = aExpression[i];

        if( quote )
        {
            if( c == quote )
                quote = 0;
        }
        else if( c == '\'' || c == '"' )
        {
            quote = c;
        }
        else if( c == '(' )
        {
            depth++;
        }
        else if( c == ')' )
        {
            depth--;
        }
        else if( depth == 0 && c == '&' && i + 1 < aExpression.size() && aExpression[i + 1] == '&' )
        {
            terms.push_back( aExpression.substr( start, i - start ) );
            start = i + 2;
            i++;
        }
    }

    terms.push_back( aExpression.substr( start ) );

    for( std::string& t : terms )
    {
        t.erase( 0, t.find_first_not_of( " \t\n" ) );
        t.erase( t.find_last_not_of( " \t\n" ) + 1 );
    }

    return terms;
}


bool truthy( KOPENAPI_QUERY_UCODE& aProgram, PCBEXPR_CONTEXT& aCtx, BOARD_ITEM* aItem )
{
    aCtx.Reset();
    aCtx.SetLayer( UNDEFINED_LAYER );
    aCtx.SetItems( aItem );
    return aProgram.Run( &aCtx )->AsDouble() != 0.0;
}


// ---------------------------------------------------------------------------------------------
// Candidates
// ---------------------------------------------------------------------------------------------

const std::vector<std::string>& allKinds()
{
    static const std::vector<std::string> kinds = { "footprints", "pads", "tracks", "vias", "zones",
                                                    "drawings", "texts", "groups" };
    return kinds;
}


std::string kindOf( const BOARD_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case PCB_FOOTPRINT_T: return "footprint";
    case PCB_PAD_T:       return "pad";
    case PCB_TRACE_T:     return "track";
    case PCB_ARC_T:       return "arc";
    case PCB_VIA_T:       return "via";
    case PCB_ZONE_T:      return "zone";
    case PCB_SHAPE_T:     return "drawing";
    case PCB_TEXT_T:
    case PCB_TEXTBOX_T:   return "text";
    case PCB_GROUP_T:     return "group";
    default:              return "item";
    }
}


/// Visit the items of the asked kinds; the visitor returns false to stop
void forEachItem( BOARD* aBoard, const std::set<std::string>& aKinds, const std::function<bool( BOARD_ITEM* )>& aVisit )
{
    auto has = [&]( const char* k ) { return aKinds.count( k ) > 0; };

    if( has( "footprints" ) || has( "pads" ) )
    {
        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( has( "footprints" ) && !aVisit( fp ) )
                return;

            if( has( "pads" ) )
            {
                for( PAD* pad : fp->Pads() )
                {
                    if( !aVisit( pad ) )
                        return;
                }
            }
        }
    }

    if( has( "tracks" ) || has( "vias" ) )
    {
        for( PCB_TRACK* track : aBoard->Tracks() )
        {
            const bool via = track->Type() == PCB_VIA_T;

            if( ( via ? has( "vias" ) : has( "tracks" ) ) && !aVisit( track ) )
                return;
        }
    }

    if( has( "zones" ) )
    {
        for( ZONE* zone : aBoard->Zones() )
        {
            if( !aVisit( zone ) )
                return;
        }

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            for( ZONE* zone : fp->Zones() )
            {
                if( !aVisit( zone ) )
                    return;
            }
        }
    }

    if( has( "drawings" ) || has( "texts" ) )
    {
        for( BOARD_ITEM* item : aBoard->Drawings() )
        {
            const bool text = item->Type() == PCB_TEXT_T || item->Type() == PCB_TEXTBOX_T;
            const bool shape = item->Type() == PCB_SHAPE_T;

            if( ( ( text && has( "texts" ) ) || ( shape && has( "drawings" ) ) ) && !aVisit( item ) )
                return;
        }
    }

    if( has( "groups" ) )
    {
        for( PCB_GROUP* group : aBoard->Groups() )
        {
            if( !aVisit( group ) )
                return;
        }
    }
}


// ---------------------------------------------------------------------------------------------
// Fields: properties (KiCad's Properties panel names, spaces as underscores) or value functions
// ---------------------------------------------------------------------------------------------

nlohmann::json propertyJson( BOARD_ITEM* aItem, PROPERTY_BASE* aProperty )
{
    const PROPERTY_DISPLAY display = aProperty->Display();

    if( display == PT_NET )
    {
        if( BOARD_CONNECTED_ITEM* connected = dynamic_cast<BOARD_CONNECTED_ITEM*>( aItem ) )
            return connected->GetNetname().ToStdString( wxConvUTF8 );

        return nullptr;
    }

    wxAny value = aItem->Get( aProperty );

    auto number = [&]( double v ) -> nlohmann::json
    {
        switch( display )
        {
        case PT_SIZE:
        case PT_COORD:      return std::round( mmOf( v ) * 1e6 ) / 1e6;
        case PT_AREA:       return mmOf( mmOf( v ) );
        case PT_DECIDEGREE: return v / 10.0;
        default:            return v;
        }
    };

    if( aProperty->HasChoices() )
    {
        int v = 0;

        if( value.GetAs<int>( &v ) )
        {
            const wxPGChoices& choices = aProperty->Choices();
            const int          idx = choices.Index( v );

            if( idx >= 0 )
                return choices.GetLabel( idx ).ToStdString( wxConvUTF8 );

            return v;
        }
    }

    if( value.CheckType<bool>() )
        return value.As<bool>();

    if( value.CheckType<int>() )
        return number( value.As<int>() );

    if( value.CheckType<long>() )
        return number( (double) value.As<long>() );

    if( value.CheckType<long long>() )
        return number( (double) value.As<long long>() );

    if( value.CheckType<unsigned>() )
        return number( value.As<unsigned>() );

    if( value.CheckType<double>() )
        return number( value.As<double>() );

    if( value.CheckType<EDA_ANGLE>() )
        return value.As<EDA_ANGLE>().AsDegrees();

    if( value.CheckType<std::optional<int>>() )
    {
        std::optional<int> v = value.As<std::optional<int>>();
        return v ? number( *v ) : nlohmann::json();
    }

    if( value.CheckType<std::optional<double>>() )
    {
        std::optional<double> v = value.As<std::optional<double>>();
        return v ? number( *v ) : nlohmann::json();
    }

    wxString text;

    if( value.GetAs<wxString>( &text ) )
        return text.ToStdString( wxConvUTF8 );

    return nullptr;
}


struct FIELD
{
    std::string name;       ///< as asked (output key)
    bool        function;   ///< a value expression (A.side(), ...)
    wxString    property;   ///< property display name (spaces)
    PROGRAM     program;
    std::string builtin;    ///< uuid / kind / owner: what every row carries
};


/// @brief The item types of the queried kinds
std::vector<TYPE_ID> kindTypes( const std::set<std::string>& aKinds )
{
    static const std::map<std::string, std::vector<TYPE_ID>> types = {
        { "footprints", { TYPE_HASH( FOOTPRINT ) } },
        { "pads", { TYPE_HASH( PAD ) } },
        { "tracks", { TYPE_HASH( PCB_TRACK ), TYPE_HASH( PCB_ARC ) } },
        { "vias", { TYPE_HASH( PCB_VIA ) } },
        { "zones", { TYPE_HASH( ZONE ) } },
        { "drawings", { TYPE_HASH( PCB_SHAPE ) } },
        { "texts", { TYPE_HASH( PCB_TEXT ), TYPE_HASH( PCB_TEXTBOX ) } },
        { "groups", { TYPE_HASH( PCB_GROUP ) } }
    };

    std::vector<TYPE_ID> out;

    for( const std::string& kind : aKinds )
    {
        if( auto it = types.find( kind ); it != types.end() )
            out.insert( out.end(), it->second.begin(), it->second.end() );
    }

    return out;
}


/// @brief A length property of the queried kinds compared with a plain number ("A.Position_X > 83"):
/// KiCad reads the number in internal units (nm), so the condition silently matches nothing or
/// everything.  Zero is fine.  Returns the offending term, empty if none
std::string unitlessLengthTerm( const std::set<std::string>& aKinds, const std::string& aExpression, int& aOffset )
{
    // quoted strings blanked, offsets kept
    std::string text = aExpression;
    char        quote = 0;

    for( char& ch : text )
    {
        if( quote )
        {
            quote = ch == quote ? 0 : quote;
            ch = ' ';
        }
        else if( ch == '\'' || ch == '"' )
        {
            quote = ch;
            ch = ' ';
        }
    }

    static const std::regex propFirst( R"(\b([AB])\.([A-Za-z_]\w*)\s*(==|!=|<=|>=|<|>)\s*(-?\d+(?:\.\d+)?)(?![\w.]))" );
    static const std::regex numberFirst( R"((?:^|[^\w.])(-?\d+(?:\.\d+)?)\s*(==|!=|<=|>=|<|>)\s*([AB])\.([A-Za-z_]\w*)(?!\s*\())" );

    PROPERTY_MANAGER& propMgr = PROPERTY_MANAGER::Instance();

    auto isLength = [&]( std::string aName )
    {
        std::replace( aName.begin(), aName.end(), '_', ' ' );

        for( TYPE_ID type : kindTypes( aKinds ) )
        {
            if( PROPERTY_BASE* property = propMgr.GetProperty( type, wxString::FromUTF8( aName ) ) )
                return property->Display() == PT_SIZE || property->Display() == PT_COORD;
        }

        return false;
    };

    for( std::sregex_iterator it( text.begin(), text.end(), propFirst ), end; it != end; ++it )
    {
        const std::smatch& m = *it;
        const size_t       after = (size_t) ( m.position( 0 ) + m.length( 0 ) );

        if( after < text.size() && text[after] == '(' )
            continue;

        if( std::stod( m[4].str() ) != 0.0 && isLength( m[2].str() ) )
        {
            aOffset = (int) m.position( 0 );
            return aExpression.substr( (size_t) m.position( 0 ), (size_t) m.length( 0 ) );
        }
    }

    for( std::sregex_iterator it( text.begin(), text.end(), numberFirst ), end; it != end; ++it )
    {
        const std::smatch& m = *it;

        if( std::stod( m[1].str() ) != 0.0 && isLength( m[4].str() ) )
        {
            aOffset = (int) m.position( 1 );
            return aExpression.substr( (size_t) m.position( 1 ), (size_t) ( m.position( 0 ) + m.length( 0 ) - m.position( 1 ) ) );
        }
    }

    return std::string();
}


/// @brief A property of some item of the queried kinds: a class property, or a dynamic one an item
/// carries (footprint fields)
bool knownProperty( BOARD* aBoard, const std::set<std::string>& aKinds, const wxString& aProperty )
{
    PROPERTY_MANAGER& propMgr = PROPERTY_MANAGER::Instance();

    for( TYPE_ID type : kindTypes( aKinds ) )
    {
        if( propMgr.GetProperty( type, aProperty ) )
            return true;
    }

    bool found = false;

    forEachItem( aBoard, aKinds,
                 [&]( BOARD_ITEM* aItem ) -> bool
                 {
                     found = propMgr.GetProperty( aItem, aProperty ) != nullptr;
                     return !found;
                 } );

    return found;
}


/// @brief A field: a property name (spaces as underscores), A.func(...), or a bare name of a query
/// function without arguments (parentRef as A.parentRef()).  aKinds given: a name that is none of
/// these for the queried kinds is an error
bool prepareField( const std::string& aName, FIELD& aField, COMPILE_ERROR& aError, BOARD* aBoard = nullptr,
                   const std::set<std::string>* aKinds = nullptr )
{
    aField.name = aName;
    aField.function = aName.find( '(' ) != std::string::npos;
    std::string name = aName.rfind( "A.", 0 ) == 0 ? aName.substr( 2 ) : aName;

    for( const char* builtin : { "uuid", "kind", "owner" } )
    {
        if( wxString::FromUTF8( name ).CmpNoCase( builtin ) == 0 )
        {
            aField.function = false;
            aField.builtin = builtin;
            return true;
        }
    }

    if( !aField.function )
    {
        auto fn = queryFunctions().find( wxString::FromUTF8( name ).Lower() );

        if( fn != queryFunctions().end() && fn->second.signature.EndsWith( wxS( "()" ) ) )
        {
            aField.function = true;
            aField.program = compile( "A." + fn->second.signature.ToStdString(), aError );
            return aField.program != nullptr;
        }
    }

    if( aField.function )
    {
        const std::string expr = aName.rfind( "A.", 0 ) == 0 ? aName : "A." + aName;
        aField.program = compile( expr, aError );
        return aField.program != nullptr;
    }

    aField.property = wxString::FromUTF8( name );
    aField.property.Replace( wxS( "_" ), wxS( " " ) );

    if( aKinds && aBoard && !knownProperty( aBoard, *aKinds, aField.property ) )
    {
        std::string kinds;

        for( const std::string& kind : *aKinds )
            kinds += ( kinds.empty() ? "" : ", " ) + kind;

        aError.message = "Unrecognized property '" + name + "' (not a property of " + kinds
                         + ", nor a function such as A.parentRef())";
        return false;
    }

    return true;
}


nlohmann::json fieldValue( const FIELD& aField, BOARD_ITEM* aItem, PCBEXPR_CONTEXT& aCtx )
{
    if( aField.builtin == "uuid" )
        return aItem->m_Uuid.AsStdString();

    if( aField.builtin == "kind" )
        return kindOf( aItem );

    if( aField.builtin == "owner" )
    {
        const FOOTPRINT* owner = aItem->GetParentFootprint();
        return owner ? owner->GetReference().ToStdString( wxConvUTF8 ) : std::string( "board" );
    }

    if( aField.function )
    {
        aCtx.Reset();
        aCtx.SetLayer( UNDEFINED_LAYER );
        aCtx.SetItems( aItem );
        LIBEVAL::VALUE* v = aField.program->Run( &aCtx );

        if( v->GetType() == LIBEVAL::VT_STRING )
            return v->AsString().ToStdString( wxConvUTF8 );

        if( v->GetType() == LIBEVAL::VT_NULL || v->GetType() == LIBEVAL::VT_UNDEFINED )
            return nullptr;

        return v->AsDouble();
    }

    if( const ZONE* zone = dynamic_cast<const ZONE*>( aItem ); zone && aField.property.CmpNoCase( wxS( "Layer" ) ) == 0 )
    {
        nlohmann::json layers = nlohmann::json::array();

        for( PCB_LAYER_ID layer : zone->GetLayerSet().Seq() )
            layers.push_back( ( zone->GetBoard() ? zone->GetBoard()->GetLayerName( layer ) : LSET::Name( layer ) )
                                      .ToStdString( wxConvUTF8 ) );

        if( layers.size() == 1 )
            return layers[0];

        return layers;
    }

    PROPERTY_MANAGER& propMgr = PROPERTY_MANAGER::Instance();
    PROPERTY_BASE*    property = propMgr.GetProperty( aItem, aField.property );

    if( !property || !propMgr.IsAvailableFor( TYPE_HASH( *aItem ), property, aItem ) )
        return nullptr;

    return propertyJson( aItem, property );
}


std::vector<std::string> defaultFields( const std::set<std::string>& aKinds )
{
    std::vector<std::string> fields = { "Reference", "Value", "Library_Link" };

    if( aKinds.count( "footprints" ) )
    {
        fields.push_back( "A.side()" );
        fields.push_back( "A.mount()" );
    }

    if( aKinds.count( "pads" ) )
    {
        fields.push_back( "A.parentRef()" );
        fields.push_back( "Pad_Number" );
    }

    for( const char* f : { "Net", "Layer", "Width", "Position_X", "Position_Y", "Orientation", "Start_X", "Start_Y",
                           "End_X", "End_Y" } )
    {
        fields.push_back( f );
    }

    return fields;
}


// ---------------------------------------------------------------------------------------------
// Ordering
// ---------------------------------------------------------------------------------------------

int compareJson( const nlohmann::json& a, const nlohmann::json& b )
{
    if( a.is_null() || b.is_null() )
        return a.is_null() == b.is_null() ? 0 : a.is_null() ? 1 : -1;   // nulls last

    if( a.is_number() && b.is_number() )
    {
        const double x = a.get<double>(), y = b.get<double>();
        return x < y ? -1 : x > y ? 1 : 0;
    }

    if( a.is_string() && b.is_string() )
        return StrNumCmp( wxString::FromUTF8( a.get<std::string>() ), wxString::FromUTF8( b.get<std::string>() ), true );

    const std::string x = a.dump(), y = b.dump();
    return x < y ? -1 : x > y ? 1 : 0;
}


struct ORDER_KEY
{
    std::string name;
    bool        descending = false;
};


std::vector<ORDER_KEY> orderKeys( const nlohmann::json& aArgs )
{
    std::vector<ORDER_KEY> keys;

    if( aArgs.contains( "order_by" ) )
    {
        for( const auto& k : aArgs["order_by"] )
        {
            std::string s = k.get<std::string>();
            ORDER_KEY   key;
            key.descending = !s.empty() && s[0] == '-';
            key.name = key.descending ? s.substr( 1 ) : s;
            keys.push_back( key );
        }
    }

    return keys;
}


/// uuids sort as text: a natural sort reads their hex digits as numbers
int compareKey( const ORDER_KEY& aKey, const nlohmann::json& aX, const nlohmann::json& aY )
{
    if( aX.is_string() && aY.is_string() && wxString( aKey.name ).IsSameAs( "uuid", false ) )
        return aX.get_ref<const std::string&>().compare( aY.get_ref<const std::string&>() );

    return compareJson( aX, aY );
}


void sortRows( std::vector<nlohmann::json>& aRows, const std::vector<ORDER_KEY>& aKeys )
{
    if( aKeys.empty() )
        return;

    std::stable_sort( aRows.begin(), aRows.end(),
                      [&]( const nlohmann::json& a, const nlohmann::json& b )
                      {
                          for( const ORDER_KEY& k : aKeys )
                          {
                              const nlohmann::json& x = a.contains( k.name ) ? a[k.name] : nlohmann::json();
                              const nlohmann::json& y = b.contains( k.name ) ? b[k.name] : nlohmann::json();
                              const int             c = compareKey( k, x, y );

                              if( c != 0 )
                                  return k.descending ? c > 0 : c < 0;
                          }

                          return false;
                      } );
}


KOPENAPI_RESULT compileError( const std::string& aWhat, const std::string& aExpression, const COMPILE_ERROR& aError )
{
    KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 400, aWhat + ": " + aError.message );
    r.body["error"]["expression"] = aExpression;
    r.body["error"]["offset"] = aError.offset;
    r.body["error"]["near"] = suggestions( aError.message );
    return r;
}

} // namespace


KOPENAPI_RESULT KopenapiPcbQuery( BOARD* aBoard, const nlohmann::json& aArgs )
{
    const CLOCK::time_point t0 = CLOCK::now();
    const double            deadline = aArgs.value( "deadline_ms", 0.0 );

    // kinds
    std::set<std::string> kinds;

    if( aArgs.contains( "from" ) )
    {
        const nlohmann::json from = aArgs["from"].is_array() ? aArgs["from"] : nlohmann::json::array( { aArgs["from"] } );

        for( const auto& k : from )
        {
            const std::string kind = k.get<std::string>();

            if( kind == "items" )
            {
                kinds.insert( allKinds().begin(), allKinds().end() );
            }
            else if( std::find( allKinds().begin(), allKinds().end(), kind ) != allKinds().end() )
            {
                kinds.insert( kind );
            }
            else
            {
                return KOPENAPI_RESULT::Error( 400, "from: unknown kind '" + kind
                                                            + "' (footprints, pads, tracks, vias, zones, drawings, "
                                                              "texts, groups, items)" );
            }
        }
    }
    else
    {
        kinds.insert( "footprints" );
    }

    // where, with its top-level terms for explain
    const std::string where = aArgs.value( "where", std::string() );
    std::string       typeHint;
    const nlohmann::json explainArg = aArgs.value( "explain", nlohmann::json( false ) );
    const bool        explainMisses = explainArg.is_string() && explainArg.get<std::string>() == "misses";
    const bool        explain = explainMisses || ( explainArg.is_boolean() && explainArg.get<bool>() );

    PROGRAM                  whereProgram;
    std::vector<std::string> terms;
    std::vector<PROGRAM>     termPrograms;
    COMPILE_ERROR            error;

    if( !where.empty() )
    {
        whereProgram = compile( where, error );

        if( !whereProgram )
            return compileError( "where", where, error );

        int               termOffset = -1;
        const std::string unitless = unitlessLengthTerm( kinds, where, termOffset );

        // A.Type == 'X' for a type none of the kinds in from has (vias are not in from: tracks)
        {
            static const std::map<std::string, std::string> kindOfType = {
                { "via", "vias" },       { "track", "tracks" },   { "pad", "pads" },          { "footprint", "footprints" },
                { "zone", "zones" },     { "graphic", "drawings" }, { "text", "texts" },      { "text box", "texts" },
                { "group", "groups" } };
            static const std::regex typeEq( R"(\bType\s*==\s*['"]([^'"]+)['"]|['"]([^'"]+)['"]\s*==\s*[AB]\.Type\b)", std::regex::icase );

            for( auto it = std::sregex_iterator( where.begin(), where.end(), typeEq ); it != std::sregex_iterator(); ++it )
            {
                std::string name = ( *it )[1].matched ? ( *it )[1].str() : ( *it )[2].str();
                std::string lower = name;
                std::transform( lower.begin(), lower.end(), lower.begin(), []( unsigned char c ) { return (char) std::tolower( c ); } );
                auto k = kindOfType.find( lower );

                if( k != kindOfType.end() && !kinds.count( k->second ) )
                {
                    typeHint = "A.Type '" + name + "' is never true in from " + nlohmann::json( std::vector<std::string>( kinds.begin(), kinds.end() ) ).dump()
                               + ": those are in from '" + k->second + "' (add it, or from: 'items')";
                    break;
                }
            }
        }

        if( !unitless.empty() )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Error(
                    400, "where: '" + unitless + "' compares a length with a plain number, which KiCad reads in "
                         "internal units (nm): add the unit (mm / mil / in), e.g. 83mm" );
            r.body["error"]["expression"] = where;
            r.body["error"]["term"] = unitless;
            r.body["error"]["offset"] = termOffset;
            return r;
        }

        if( explain )
        {
            terms = splitAnd( where );

            for( const std::string& term : terms )
            {
                COMPILE_ERROR termError;
                PROGRAM       p = compile( term, termError );

                if( !p )
                    return compileError( "where term", term, termError );

                termPrograms.push_back( p );
            }
        }
    }

    // fields
    std::vector<std::string> selectNames;

    if( aArgs.contains( "select" ) )
    {
        for( const auto& s : aArgs["select"] )
            selectNames.push_back( s.get<std::string>() );
    }
    else
    {
        selectNames = defaultFields( kinds );
    }

    std::vector<std::string> groupNames;

    static const std::string AGGREGATE_FORM = "aggregate is {output name: function}, function count, sum(x), min(x), "
                                              "max(x), avg(x) or distinct(x) of a field x; the groups come from the "
                                              "top-level group_by (field names), e.g. {from: 'tracks', group_by: 'Net', "
                                              "aggregate: {tracks: 'count', length: 'sum(Length)'}} (Length: A.length(), mm)";

    // group_by: a field name or a list of them
    if( aArgs.contains( "group_by" ) )
    {
        const nlohmann::json& g = aArgs["group_by"];

        if( g.is_string() )
        {
            groupNames.push_back( g.get<std::string>() );
        }
        else if( g.is_array() && std::all_of( g.begin(), g.end(), []( const nlohmann::json& v ) { return v.is_string(); } ) )
        {
            for( const auto& v : g )
                groupNames.push_back( v.get<std::string>() );
        }
        else
        {
            return KOPENAPI_RESULT::Error( 400, "group_by: a field name or a list of them (e.g. 'Net' or ['Net', 'Layer'])" );
        }
    }

    struct AGGREGATE
    {
        std::string out;
        std::string fn;     ///< count, sum, min, max, avg, distinct
        std::string field;  ///< empty for count
    };

    std::vector<AGGREGATE> aggregates;

    if( aArgs.contains( "aggregate" ) )
    {
        if( !aArgs["aggregate"].is_object() )
            return KOPENAPI_RESULT::Error( 400, AGGREGATE_FORM );

        for( const auto& [out, spec] : aArgs["aggregate"].items() )
        {
            if( out == "group_by" )
                return KOPENAPI_RESULT::Error( 400, "aggregate: group_by is not an aggregate but a top-level argument; " + AGGREGATE_FORM );

            if( !spec.is_string() )
                return KOPENAPI_RESULT::Error( 400, "aggregate '" + out + "': " + AGGREGATE_FORM );

            const std::string s = spec.get<std::string>();
            AGGREGATE         a;
            a.out = out;

            if( s == "count" || s == "count()" )
            {
                a.fn = "count";
            }
            else
            {
                const size_t open = s.find( '(' );

                if( open == std::string::npos || s.back() != ')' )
                    return KOPENAPI_RESULT::Error( 400, "aggregate '" + out + "': '" + s + "' is no function; " + AGGREGATE_FORM );

                a.fn = s.substr( 0, open );
                a.field = s.substr( open + 1, s.size() - open - 2 );

                if( a.fn != "sum" && a.fn != "min" && a.fn != "max" && a.fn != "avg" && a.fn != "distinct" )
                    return KOPENAPI_RESULT::Error( 400, "aggregate '" + out + "': unknown function '" + a.fn + "'; " + AGGREGATE_FORM );
            }

            aggregates.push_back( a );
        }
    }

    std::map<std::string, FIELD> fields;

    auto needField = [&]( const std::string& aName, bool aCheck = true ) -> bool
    {
        if( aName.empty() || fields.count( aName ) )
            return true;

        FIELD f;

        if( !prepareField( aName, f, error, aBoard, aCheck ? &kinds : nullptr ) )
            return false;

        fields[aName] = f;
        return true;
    };

    for( const std::string& n : selectNames )
    {
        if( !needField( n, aArgs.contains( "select" ) ) )
            return compileError( "select", n, error );
    }

    for( const std::string& n : groupNames )
    {
        if( !needField( n ) )
            return compileError( "group_by", n, error );
    }

    for( const AGGREGATE& a : aggregates )
    {
        if( !needField( a.field ) )
            return compileError( "aggregate", a.field, error );
    }

    const std::vector<ORDER_KEY> order = orderKeys( aArgs );

    for( const ORDER_KEY& k : order )
    {
        bool isAggregate = std::any_of( aggregates.begin(), aggregates.end(), [&]( const AGGREGATE& a ) { return a.out == k.name; } );

        if( !isAggregate && !needField( k.name ) )
            return compileError( "order_by", k.name, error );
    }

    const double compileMs = msSince( t0 );

    // scan
    const CLOCK::time_point   t1 = CLOCK::now();
    PCBEXPR_CONTEXT           ctx( 0, UNDEFINED_LAYER );
    std::vector<BOARD_ITEM*>  matched;
    std::vector<std::vector<bool>> matchedTerms;
    std::vector<std::pair<BOARD_ITEM*, std::vector<bool>>> misses;
    size_t                    scanned = 0;
    bool                      timedOut = false;

    forEachItem( aBoard, kinds,
                 [&]( BOARD_ITEM* aItem ) -> bool
                 {
                     if( ( ++scanned & 4095 ) == 0 )
                     {
                         KopenapiKeepUiAlive();

                         if( deadline > 0 && msSince( t0 ) > deadline )
                         {
                             timedOut = true;
                             return false;
                         }
                     }

                     const bool hit = !whereProgram || truthy( *whereProgram, ctx, aItem );

                     if( explain )
                     {
                         std::vector<bool> results;

                         for( PROGRAM& p : termPrograms )
                             results.push_back( truthy( *p, ctx, aItem ) );

                         if( hit )
                         {
                             matched.push_back( aItem );
                             matchedTerms.push_back( results );
                         }
                         else if( explainMisses && misses.size() < 100
                                  && std::count( results.begin(), results.end(), false ) == 1 )
                         {
                             misses.emplace_back( aItem, results );
                         }
                     }
                     else if( hit )
                     {
                         matched.push_back( aItem );
                     }

                     return true;
                 } );

    const double scanMs = msSince( t1 );

    if( timedOut )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 504, "deadline_ms reached before the scan ended" );
        r.body["scanned"] = scanned;
        r.body["matched_so_far"] = matched.size();
        r.body["timing_ms"] = { { "compile", compileMs }, { "scan", scanMs } };
        return r;
    }

    // shape
    const CLOCK::time_point t2 = CLOCK::now();
    nlohmann::json          answer = { { "count", matched.size() }, { "scanned", scanned } };

    if( !typeHint.empty() )
        answer["hint"] = typeHint;

    auto itemRow = [&]( BOARD_ITEM* aItem, const std::vector<std::string>& aNames ) -> nlohmann::json
    {
        nlohmann::json row = { { "kind", kindOf( aItem ) }, { "uuid", aItem->m_Uuid.AsStdString() } };

        if( aItem->Type() == PCB_ZONE_T )
        {
            const FOOTPRINT* owner = aItem->GetParentFootprint();
            row["owner"] = owner ? owner->GetReference().ToStdString( wxConvUTF8 ) : std::string( "board" );
        }

        for( const std::string& n : aNames )
            row[n] = fieldValue( fields[n], aItem, ctx );

        return row;
    };

    if( !groupNames.empty() || !aggregates.empty() )
    {
        struct ACC
        {
            nlohmann::json                key;
            size_t                        count = 0;
            std::map<std::string, double> sum, min, max;
            std::map<std::string, size_t> n;
            std::map<std::string, std::set<std::string>> distinct;
        };

        std::map<std::string, ACC> groups;
        std::vector<std::string>   groupOrder;

        for( BOARD_ITEM* item : matched )
        {
            nlohmann::json key = nlohmann::json::object();

            for( const std::string& g : groupNames )
                key[g] = fieldValue( fields[g], item, ctx );

            const std::string keyText = key.dump();
            auto [it, inserted] = groups.try_emplace( keyText );

            if( inserted )
            {
                it->second.key = key;
                groupOrder.push_back( keyText );
            }

            ACC& acc = it->second;
            acc.count++;

            for( const AGGREGATE& a : aggregates )
            {
                if( a.fn == "count" )
                    continue;

                const nlohmann::json v = fieldValue( fields[a.field], item, ctx );

                if( a.fn == "distinct" )
                {
                    if( !v.is_null() )
                        acc.distinct[a.out].insert( v.is_string() ? v.get<std::string>() : v.dump() );

                    continue;
                }

                if( !v.is_number() )
                    continue;

                const double d = v.get<double>();
                acc.sum[a.out] += d;
                acc.min[a.out] = acc.n[a.out] ? std::min( acc.min[a.out], d ) : d;
                acc.max[a.out] = acc.n[a.out] ? std::max( acc.max[a.out], d ) : d;
                acc.n[a.out]++;
            }
        }

        std::vector<nlohmann::json> rows;

        for( const std::string& k : groupOrder )
        {
            const ACC&     acc = groups[k];
            nlohmann::json row = { { "key", acc.key }, { "count", acc.count } };

            for( const auto& [name, value] : acc.key.items() )
                row[name] = value;

            for( const AGGREGATE& a : aggregates )
            {
                if( a.fn == "count" )
                {
                    row[a.out] = acc.count;
                }
                else if( a.fn == "distinct" )
                {
                    std::vector<std::string> values;

                    if( auto d = acc.distinct.find( a.out ); d != acc.distinct.end() )
                        values.assign( d->second.begin(), d->second.end() );

                    std::sort( values.begin(), values.end(),
                               []( const std::string& x, const std::string& y )
                               { return StrNumCmp( wxString::FromUTF8( x ), wxString::FromUTF8( y ), true ) < 0; } );
                    row[a.out] = values;
                }
                else
                {
                    auto   n = acc.n.find( a.out );
                    size_t count = n == acc.n.end() ? 0 : n->second;

                    if( count == 0 )
                        row[a.out] = nullptr;
                    else if( a.fn == "sum" )
                        row[a.out] = acc.sum.at( a.out );
                    else if( a.fn == "min" )
                        row[a.out] = acc.min.at( a.out );
                    else if( a.fn == "max" )
                        row[a.out] = acc.max.at( a.out );
                    else
                        row[a.out] = acc.sum.at( a.out ) / count;
                }
            }

            rows.push_back( row );
        }

        sortRows( rows, order );
        answer["groups"] = rows;
    }
    else
    {
        // order on the needed keys first, then build only the page's rows
        std::vector<size_t> idx( matched.size() );

        for( size_t i = 0; i < idx.size(); ++i )
            idx[i] = i;

        if( !order.empty() )
        {
            std::vector<nlohmann::json> keys( matched.size() );

            for( size_t i = 0; i < matched.size(); ++i )
            {
                for( const ORDER_KEY& k : order )
                    keys[i][k.name] = fieldValue( fields[k.name], matched[i], ctx );
            }

            std::stable_sort( idx.begin(), idx.end(),
                              [&]( size_t a, size_t b )
                              {
                                  for( const ORDER_KEY& k : order )
                                  {
                                      const int c = compareKey( k, keys[a][k.name], keys[b][k.name] );

                                      if( c != 0 )
                                          return k.descending ? c > 0 : c < 0;
                                  }

                                  return false;
                              } );
        }

        const size_t limit = std::min<size_t>( aArgs.value( "limit", 1000 ), 100000 );
        size_t       offset = 0;

        if( aArgs.contains( "cursor" ) && aArgs["cursor"].is_string() )
            offset = std::stoul( aArgs["cursor"].get<std::string>() );

        nlohmann::json items = nlohmann::json::array();

        for( size_t i = offset; i < idx.size() && i < offset + limit; ++i )
        {
            nlohmann::json row = itemRow( matched[idx[i]], selectNames );

            if( explain )
            {
                nlohmann::json e = nlohmann::json::object();

                for( size_t t = 0; t < terms.size(); ++t )
                    e[terms[t]] = (bool) matchedTerms[idx[i]][t];

                row["explain"] = e;
            }

            items.push_back( row );
        }

        answer["items"] = items;
        answer["next_cursor"] = offset + limit < idx.size() ? nlohmann::json( std::to_string( offset + limit ) )
                                                            : nlohmann::json();

        if( explainMisses )
        {
            nlohmann::json nearby = nlohmann::json::array();

            for( auto& [item, results] : misses )
            {
                nlohmann::json row = itemRow( item, selectNames );
                nlohmann::json e = nlohmann::json::object();

                for( size_t t = 0; t < terms.size(); ++t )
                    e[terms[t]] = (bool) results[t];

                row["explain"] = e;
                nearby.push_back( row );
            }

            answer["near_misses"] = nearby;
        }
    }

    answer["unknown"] = nlohmann::json::object();
    answer["timing_ms"] = { { "compile", compileMs }, { "scan", scanMs }, { "shape", msSince( t2 ) },
                            { "total", msSince( t0 ) } };
    return KOPENAPI_RESULT::Ok( answer );
}
