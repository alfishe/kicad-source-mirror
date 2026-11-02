/// @file kicadopenapi_netclasses.cpp
/// @brief kicadopenapi net classes in the project's net settings (pcb_rules_set, sch_netclass_set)
#include "kicadopenapi_netclasses.h"

#include <base_units.h>
#include <eda_pattern_match.h>
#include <gal/color4d.h>
#include <netclass.h>
#include <project/net_settings.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <vector>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


/// A class length in mm: name, getter, setter, the unit scale of the editor it belongs to
struct LENGTH_FIELD
{
    const char*                                    name;
    std::function<std::optional<int>( NETCLASS& )> get;
    std::function<void( NETCLASS&, int )>          set;
    bool                                           schematic;
};


const std::vector<LENGTH_FIELD>& lengthFields()
{
    static const std::vector<LENGTH_FIELD> fields = {
        { "clearance_mm", []( NETCLASS& c ) { return c.HasClearance() ? std::optional<int>( c.GetClearance() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetClearance( v ); }, false },
        { "track_width_mm", []( NETCLASS& c ) { return c.HasTrackWidth() ? std::optional<int>( c.GetTrackWidth() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetTrackWidth( v ); }, false },
        { "via_diameter_mm", []( NETCLASS& c ) { return c.HasViaDiameter() ? std::optional<int>( c.GetViaDiameter() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetViaDiameter( v ); }, false },
        { "via_drill_mm", []( NETCLASS& c ) { return c.HasViaDrill() ? std::optional<int>( c.GetViaDrill() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetViaDrill( v ); }, false },
        { "diff_pair_width_mm", []( NETCLASS& c ) { return c.HasDiffPairWidth() ? std::optional<int>( c.GetDiffPairWidth() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetDiffPairWidth( v ); }, false },
        { "diff_pair_gap_mm", []( NETCLASS& c ) { return c.HasDiffPairGap() ? std::optional<int>( c.GetDiffPairGap() ) : std::nullopt; },
          []( NETCLASS& c, int v ) { c.SetDiffPairGap( v ); }, false },
        { "wire_width_mm", []( NETCLASS& c ) { return c.GetWireWidthOpt(); },
          []( NETCLASS& c, int v ) { c.SetWireWidth( v ); }, true },
        { "bus_width_mm", []( NETCLASS& c ) { return c.GetBusWidthOpt(); },
          []( NETCLASS& c, int v ) { c.SetBusWidth( v ); }, true } };

    return fields;
}


const EDA_IU_SCALE& scaleOf( const LENGTH_FIELD& aField )
{
    return aField.schematic ? schIUScale : pcbIUScale;
}


const std::vector<std::string>& lineStyles()
{
    // LINE_STYLE::SOLID .. DASHDOTDOT
    static const std::vector<std::string> styles = { "solid", "dash", "dot", "dash_dot", "dash_dot_dot" };
    return styles;
}

} // namespace


const nlohmann::json& KopenapiNetclassBoardSchema()
{
    static const nlohmann::json schema = R"json({
        "clearance_mm":{"type":"number"},"track_width_mm":{"type":"number"},
        "via_diameter_mm":{"type":"number"},"via_drill_mm":{"type":"number"},
        "diff_pair_width_mm":{"type":"number"},"diff_pair_gap_mm":{"type":"number"}})json"_json;
    return schema;
}


const nlohmann::json& KopenapiNetclassSchematicSchema()
{
    static const nlohmann::json schema = R"json({
        "wire_width_mm":{"type":"number"},"bus_width_mm":{"type":"number"},
        "color":{"type":"string","description":"#rrggbb or #rrggbbaa; \"\" for none (the theme's)"},
        "line_style":{"type":"string","enum":["solid","dash","dot","dash_dot","dash_dot_dot"]}})json"_json;
    return schema;
}


std::string KopenapiNetclassesCheck( const nlohmann::json& aList )
{
    if( !aList.is_array() )
        return "netclasses: array of {name, clearance_mm, track_width_mm, ..., nets}";

    for( const nlohmann::json& c : aList )
    {
        if( !c.is_object() || !c.contains( "name" ) || !c["name"].is_string() || c["name"].get<std::string>().empty() )
            return "every netclass needs a 'name' (\"Default\" edits the default class)";

        for( const LENGTH_FIELD& f : lengthFields() )
        {
            if( c.contains( f.name ) && ( !c[f.name].is_number() || c[f.name].get<double>() < 0 ) )
                return std::string( f.name ) + ": a number >= 0 (mm)";
        }

        if( c.contains( "color" ) )
        {
            KIGFX::COLOR4D color;

            if( !c["color"].is_string()
                || ( !c["color"].get<std::string>().empty() && !color.SetFromHexString( wxString::FromUTF8( c["color"].get<std::string>() ) ) ) )
            {
                return "color: #rrggbb, #rrggbbaa or \"\"";
            }
        }

        if( c.contains( "line_style" )
            && ( !c["line_style"].is_string()
                 || std::find( lineStyles().begin(), lineStyles().end(), c["line_style"].get<std::string>() ) == lineStyles().end() ) )
        {
            return "line_style: solid, dash, dot, dash_dot or dash_dot_dot";
        }

        if( c.contains( "nets" ) && !c["nets"].is_array() )
            return "nets: array of net names or patterns (*, ?)";

        if( c.value( "delete", false ) && c.value( "name", std::string() ) == std::string( NETCLASS::Default ) )
            return "the Default class cannot be deleted";
    }

    return std::string();
}


void KopenapiNetclassesApply( NET_SETTINGS& aSettings, const nlohmann::json& aList )
{
    for( const nlohmann::json& c : aList )
    {
        const wxString            name = wxString::FromUTF8( c["name"].get<std::string>() );
        std::shared_ptr<NETCLASS> netclass;

        // the class's patterns, or the patterns of other classes naming the same nets, go
        auto dropPatterns = [&]( const std::function<bool( const wxString&, const wxString& )>& aDrop )
        {
            auto& assignments = aSettings.GetNetclassPatternAssignments();
            std::vector<std::pair<std::unique_ptr<EDA_COMBINED_MATCHER>, wxString>> kept;

            for( auto& [matcher, cls] : assignments )
            {
                if( !aDrop( matcher->GetPattern(), cls ) )
                    kept.emplace_back( std::move( matcher ), cls );
            }

            aSettings.SetNetclassPatternAssignments( std::move( kept ) );
        };

        if( c.value( "delete", false ) )
        {
            dropPatterns( [&]( const wxString&, const wxString& aClass ) { return aClass == name; } );

            std::map<wxString, std::shared_ptr<NETCLASS>> classes = aSettings.GetNetclasses();
            classes.erase( name );
            aSettings.SetNetclasses( classes );
            continue;
        }

        if( c.contains( "nets" ) )
        {
            std::set<wxString> named;

            for( const nlohmann::json& pattern : c["nets"] )
            {
                if( pattern.is_string() )
                    named.insert( wxString::FromUTF8( pattern.get<std::string>() ) );
            }

            dropPatterns( [&]( const wxString& aPattern, const wxString& aClass )
                          { return aClass == name || named.count( aPattern ); } );
        }

        if( name == NETCLASS::Default )
        {
            netclass = aSettings.GetDefaultNetclass();
        }
        else if( aSettings.GetNetclasses().count( name ) )
        {
            netclass = aSettings.GetNetclasses().at( name );
        }
        else
        {
            netclass = std::make_shared<NETCLASS>( name, false );
            aSettings.SetNetclass( name, netclass );
        }

        for( const LENGTH_FIELD& f : lengthFields() )
        {
            if( c.contains( f.name ) )
                f.set( *netclass, scaleOf( f ).mmToIU( c[f.name].get<double>() ) );
        }

        if( c.contains( "color" ) )
        {
            KIGFX::COLOR4D color = KIGFX::COLOR4D::UNSPECIFIED;

            if( !c["color"].get<std::string>().empty() )
                color.SetFromHexString( wxString::FromUTF8( c["color"].get<std::string>() ) );

            netclass->SetSchematicColor( color );
        }

        if( c.contains( "line_style" ) )
        {
            const auto& styles = lineStyles();
            netclass->SetLineStyle( static_cast<int>( std::find( styles.begin(), styles.end(), c["line_style"].get<std::string>() )
                                                      - styles.begin() ) );
        }

        for( const nlohmann::json& pattern : c.value( "nets", nlohmann::json::array() ) )
        {
            if( pattern.is_string() )
                aSettings.SetNetclassPatternAssignment( wxString::FromUTF8( pattern.get<std::string>() ), name );
        }
    }

    aSettings.ClearAllCaches();
}


nlohmann::json KopenapiNetclassJson( NETCLASS& aClass, bool aSchematic )
{
    nlohmann::json c = { { "name", str( aClass.GetName() ) } };

    for( const LENGTH_FIELD& f : lengthFields() )
    {
        if( f.schematic && !aSchematic )
            continue;

        if( std::optional<int> v = f.get( aClass ) )
            c[f.name] = std::round( scaleOf( f ).IUTomm( *v ) * 1000.0 ) / 1000.0;
    }

    if( aSchematic )
    {
        if( aClass.GetSchematicColor() != KIGFX::COLOR4D::UNSPECIFIED )
            c["color"] = str( aClass.GetSchematicColor().ToHexString() );

        if( std::optional<int> style = aClass.GetLineStyleOpt();
            style && *style >= 0 && *style < static_cast<int>( lineStyles().size() ) )
        {
            c["line_style"] = lineStyles()[*style];
        }
    }

    return c;
}


nlohmann::json KopenapiNetclassesJson( NET_SETTINGS& aSettings, bool aSchematic )
{
    nlohmann::json classes = nlohmann::json::array();
    classes.push_back( KopenapiNetclassJson( *aSettings.GetDefaultNetclass(), aSchematic ) );

    for( const auto& [name, netclass] : aSettings.GetNetclasses() )
        classes.push_back( KopenapiNetclassJson( *netclass, aSchematic ) );

    nlohmann::json patterns = nlohmann::json::array();

    for( const auto& [matcher, netclass] : aSettings.GetNetclassPatternAssignments() )
        patterns.push_back( { { "pattern", str( matcher->GetPattern() ) }, { "netclass", str( netclass ) } } );

    return { { "netclasses", classes }, { "patterns", patterns } };
}
