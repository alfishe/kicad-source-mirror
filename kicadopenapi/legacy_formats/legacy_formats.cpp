/// @file legacy_formats.cpp
/// @brief Entry point: picks the chain of steps from the file's version to the target's
#include "legacy_formats_internal.h"
#include "v10/v10_common.h"


namespace LEGACY_FORMATS
{

/// Targets with a complete chain; tools/legacy_formats_refresh.py checks their bumps
static const char* SUPPORTED_TARGETS[] = { "10.0" };


std::vector<std::string> Targets()
{
    return std::vector<std::string>( std::begin( SUPPORTED_TARGETS ), std::end( SUPPORTED_TARGETS ) );
}


bool KindFromExtension( const std::string& aExtension, KIND& aKind )
{
    if( aExtension == "kicad_pcb" )
        aKind = KIND::BOARD;
    else if( aExtension == "kicad_mod" )
        aKind = KIND::FOOTPRINT;
    else if( aExtension == "kicad_sch" )
        aKind = KIND::SCHEMATIC;
    else if( aExtension == "kicad_sym" )
        aKind = KIND::SYMBOL_LIB;
    else
        return false;

    return true;
}


static int targetVersion( KIND aKind, const std::string& aTarget )
{
    if( aTarget == "10.0" )
    {
        switch( aKind )
        {
        case KIND::BOARD:
        case KIND::FOOTPRINT:  return V10::BOARD_VERSION;
        case KIND::SCHEMATIC:  return V10::SCHEMATIC_VERSION;
        case KIND::SYMBOL_LIB: return V10::SYMBOL_LIB_VERSION;
        }
    }

    return 0;
}


RESULT Convert( KIND aKind, const std::string& aText, const std::string& aTarget, const HOST& aHost )
{
    RESULT result;
    int    target = targetVersion( aKind, aTarget );

    if( !target )
    {
        result.error = "unsupported target '" + aTarget + "'";
        return result;
    }

    NODE root;

    if( !Parse( aText, root, result.error ) )
        return result;

    const NODE* version = root.Child( "version" );
    double      value = 0.0;

    if( !version || !ParseNumber( version->AtomAt( 1 ), value ) )
    {
        result.error = "file has no (version)";
        return result;
    }

    result.fromVersion = int( value );
    result.toVersion = result.fromVersion;

    if( result.fromVersion <= target )
    {
        result.ok = true;
        result.text = aText;
        return result;
    }

    REPORT  report;
    CONTEXT ctx{ aKind, aTarget, result.fromVersion, report, aHost };

    if( !StepTo10( root, ctx, result.error ) )
        return result;

    result.ok = true;
    result.toVersion = target;
    result.text = Write( root );
    result.changes = report.Lines();
    return result;
}

} // namespace LEGACY_FORMATS
