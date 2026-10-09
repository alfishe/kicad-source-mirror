#include "kicadopenapi_checks.h"

#include <kicadopenapi_util.h>
#include <widgets/report_severity.h>

#include <algorithm>
#include <map>


bool KopenapiSeverityMask( const nlohmann::json& aArgs, int& aMask )
{
    aMask = RPT_SEVERITY_ERROR | RPT_SEVERITY_WARNING;

    if( !aArgs.contains( "severity" ) )
        return true;

    if( !aArgs["severity"].is_array() )
        return false;

    aMask = 0;

    for( const nlohmann::json& name : aArgs["severity"] )
    {
        if( name == "error" )
            aMask |= RPT_SEVERITY_ERROR;
        else if( name == "warning" )
            aMask |= RPT_SEVERITY_WARNING;
        else if( name == "exclusion" )
            aMask |= RPT_SEVERITY_EXCLUSION;
        else
            return false;
    }

    return aMask != 0;
}


nlohmann::json KopenapiCheckSchema( nlohmann::json aExtra )
{
    nlohmann::json properties = R"json({
        "severity":{"type":"array","items":{"type":"string","enum":["error","warning","exclusion"]},
                    "default":["error","warning"]},
        "type":{"type":"string","description":"glob on the violation type, e.g. pin_not_connected or clearance"},
        "unconnected_limit":{"type":"integer","default":50,"description":"unconnected items listed"}})json"_json;

    for( const auto& [key, value] : aExtra.items() )
        properties[key] = value;

    return KopenapiPagedSchema( properties );
}


nlohmann::json KopenapiCheckResult( const std::vector<nlohmann::json>& aViolations,
                                    const std::vector<nlohmann::json>& aUnconnected,
                                    const nlohmann::json& aIgnoredChecks, const nlohmann::json& aArgs )
{
    int                        errors = 0, warnings = 0, excluded = 0;
    std::map<std::string, int> byType;
    const std::string          typeGlob = aArgs.value( "type", std::string() );
    std::vector<nlohmann::json> rows;

    for( const nlohmann::json& v : aViolations )
    {
        const std::string type = v.value( "type", std::string() );

        if( v.value( "excluded", false ) )
            excluded++;
        else if( v.value( "severity", std::string() ) == "error" )
            errors++;
        else
            warnings++;

        byType[type]++;

        if( KopenapiGlob( typeGlob, type ) )
            rows.push_back( v );
    }

    nlohmann::json types = nlohmann::json::object();

    for( const auto& [type, count] : byType )
        types[type] = count;

    const size_t   limit = (size_t) std::clamp( aArgs.value( "unconnected_limit", 50 ), 0, 100000 );
    nlohmann::json unconnected = nlohmann::json::array();

    for( size_t i = 0; i < aUnconnected.size() && i < limit; ++i )
        unconnected.push_back( aUnconnected[i] );

    nlohmann::json result = KopenapiPage( rows, aArgs );
    result["summary"] = { { "errors", errors },
                          { "warnings", warnings },
                          { "excluded", excluded },
                          { "by_type", types },
                          { "unconnected_items", aUnconnected.size() } };
    result["unconnected"] = unconnected;
    result["ignored_checks"] = aIgnoredChecks;
    return result;
}
