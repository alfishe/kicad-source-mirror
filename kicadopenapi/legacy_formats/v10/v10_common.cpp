/// @file v10_common.cpp
/// @brief Rules shared by the board and schematic steps to 10.0
#include "v10_common.h"

#include <cmath>
#include <cstdio>
#include <functional>


namespace LEGACY_FORMATS
{
namespace V10
{

void SetHeader( NODE& aRoot, int aVersion )
{
    if( NODE* version = aRoot.Child( "version" ); version && version->items.size() > 1 )
        version->items[1].atom = std::to_string( aVersion );

    if( NODE* generator = aRoot.Child( "generator_version" ); generator && generator->items.size() > 1 )
        generator->items[1].atom = "\"10.0\"";
}


static bool isBold( const NODE& aFont )
{
    if( aFont.HasAtom( "bold" ) )
        return true;

    const NODE* bold = aFont.Child( "bold" );
    return bold && ( bold->items.size() < 2 || bold->AtomAt( 1 ) == "yes" );
}


static bool isStrokeFont( const NODE& aFont )
{
    const NODE* face = aFont.Child( "face" );
    return !face || Unquote( face->AtomAt( 1 ) ) == "KiCad Font";
}


void UnbakeBoldThickness( NODE& aRoot, long long aIuPerMm, REPORT& aReport )
{
    std::function<void( NODE& )> walk = [&]( NODE& aNode )
    {
        if( aNode.Is( "font" ) && isBold( aNode ) && isStrokeFont( aNode ) )
        {
            NODE*  thickness = aNode.Child( "thickness" );
            double mm = 0.0;

            if( thickness && ParseNumber( thickness->AtomAt( 1 ), mm ) )
            {
                long long iu = ToIU( mm, aIuPerMm );

                if( iu > 1 )
                {
                    thickness->items[1].atom = FormatMM( std::llround( iu * 1.6 ), aIuPerMm );
                    aReport.Note( "bold text: thickness written as the drawn width (x1.6)" );
                }
            }

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


void RestoreImageScale( NODE& aRoot, REPORT& aReport )
{
    std::function<void( NODE& )> walk = [&]( NODE& aNode )
    {
        if( aNode.Is( "image" ) )
        {
            NODE* data = aNode.Child( "data" );

            if( !data )
                return;

            std::string encoded;

            for( size_t ii = 1; ii < data->items.size(); ++ii )
                encoded += Unquote( data->items[ii].atom );

            std::string bytes;
            int         ppi = 0;
            int         legacyPpi = 0;

            if( !Base64Decode( encoded, bytes ) || !ImageDensity( bytes, ppi, legacyPpi ) || ppi == legacyPpi )
                return;

            NODE*  scale = aNode.Child( "scale" );
            double value = 1.0;

            if( scale )
                ParseNumber( scale->AtomAt( 1 ), value );

            char text[64];
            std::snprintf( text, sizeof( text ), "%.10g", value * legacyPpi / ppi );
            std::string atom = text;

            for( char& c : atom )
            {
                if( c == ',' )
                    c = '.';
            }

            if( scale )
            {
                scale->items[1].atom = atom;
            }
            else
            {
                NODE added = NODE::List( "scale" );
                added.items.push_back( NODE::Atom( atom ) );
                aNode.items.insert( aNode.items.begin() + 1, added );
            }

            aReport.Note( "reference image: scale for the 10.0 pixel density reading" );
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


std::string DerivedUuid( const std::string& aUuid, unsigned aOffset )
{
    std::string raw = Unquote( aUuid );

    if( raw.size() < 8 )
        return aUuid;

    unsigned long low = std::strtoul( raw.substr( raw.size() - 8 ).c_str(), nullptr, 16 );
    char          hex[9];
    std::snprintf( hex, sizeof( hex ), "%08lx", ( low + aOffset ) & 0xFFFFFFFFUL );
    return Quote( raw.substr( 0, raw.size() - 8 ) + hex );
}

void EmbeddedChecksums( NODE& aNode, CONTEXT& aCtx )
{
    for( NODE& child : aNode.items )
    {
        if( !child.isList )
            continue;

        if( !child.Is( "embedded_files" ) )
        {
            EmbeddedChecksums( child, aCtx );
            continue;
        }

        for( NODE& file : child.items )
        {
            NODE*       data = file.isList && file.Is( "file" ) ? file.Child( "data" ) : nullptr;
            NODE*       checksum = data ? file.Child( "checksum" ) : nullptr;
            std::string encoded;

            if( !checksum || checksum->items.size() < 2 )
                continue;

            for( size_t ii = 1; ii < data->items.size(); ++ii )
            {
                for( char c : data->items[ii].atom )
                {
                    if( c != '|' )
                        encoded += c;
                }
            }

            std::string hash;

            if( !aCtx.host.legacyEmbeddedChecksum )
            {
                aCtx.report.Note( "embedded file checksum kept (no hash service): " + aCtx.target
                                  + " may refuse the file" );
            }
            else if( !aCtx.host.legacyEmbeddedChecksum( encoded, hash ) )
            {
                aCtx.report.Note( "embedded file checksum kept (data not decodable)" );
            }
            else if( Unquote( checksum->items[1].atom ) != hash )
            {
                checksum->items[1].atom = Quote( hash );
                aCtx.report.Note( "embedded file checksum written with the " + aCtx.target + " hash" );
            }
        }
    }
}

} // namespace V10


bool StepTo10( NODE& aRoot, CONTEXT& aCtx, std::string& aError )
{
    bool ok = aCtx.kind == KIND::BOARD || aCtx.kind == KIND::FOOTPRINT ? V10::BoardStep( aRoot, aCtx, aError )
                                                                        : V10::SchematicStep( aRoot, aCtx, aError );

    // Not a format bump: 10.0 hashes embedded files with the MurmurHash3 tail bug master fixed
    if( ok )
        V10::EmbeddedChecksums( aRoot, aCtx );

    return ok;
}

} // namespace LEGACY_FORMATS
