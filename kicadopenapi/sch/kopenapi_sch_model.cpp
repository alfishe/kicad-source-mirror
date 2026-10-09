#include "kopenapi_sch_model.h"

#include <advanced_config.h>
#include <connection_graph.h>
#include <connectivity/conn_facade.h>
#include <kicadopenapi_util.h>
#include <sch_label.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <schematic.h>

#include <algorithm>
#include <map>
#include <set>

namespace kopenapi_sch
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


const char* labelKind( KICAD_T aType )
{
    switch( aType )
    {
    case SCH_LABEL_T:           return "local";
    case SCH_GLOBAL_LABEL_T:    return "global";
    case SCH_HIER_LABEL_T:      return "hierarchical";
    case SCH_DIRECTIVE_LABEL_T: return "directive";
    default:                    return nullptr;
    }
}


const char* portDirection( const SCH_SHEET_PIN* aPin )
{
    switch( aPin->GetShape() )
    {
    case LABEL_FLAG_SHAPE::L_INPUT:    return "input";
    case LABEL_FLAG_SHAPE::L_OUTPUT:   return "output";
    case LABEL_FLAG_SHAPE::L_BIDI:     return "bidirectional";
    case LABEL_FLAG_SHAPE::L_TRISTATE: return "tristate";
    default:                           return "passive";
    }
}


std::string sheetPath( const SCH_SHEET_PATH& aPath )
{
    return str( aPath.PathHumanReadable( false, true ) );
}


SCH_SYMBOL* pinSymbol( const SCH_PIN* aPin )
{
    return dynamic_cast<SCH_SYMBOL*>( const_cast<SCH_PIN*>( aPin )->GetParentSymbol() );
}


/// One pin of a net as seen on a given sheet instance
nlohmann::json pinJson( const SCH_PIN* aPin, const SCH_SHEET_PATH& aPath )
{
    SCH_SYMBOL*       sym = pinSymbol( aPin );
    const std::string ref = sym ? str( sym->GetRef( &aPath, false ) ) : std::string();

    return { { "pin", ref + "." + str( aPin->GetNumber() ) },
             { "ref", ref },
             { "number", str( aPin->GetNumber() ) },
             { "name", str( aPin->GetShownName() ) },
             { "type", str( aPin->GetElectricalTypeName() ) },
             { "value", sym ? str( sym->GetValue( &aPath, FOR_GUI ) ) : std::string() },
             { "power_symbol", sym && sym->IsPower() },
             { "sheet", sheetPath( aPath ) } };
}


std::vector<NET_ENTRY> collectNets( SCHEMATIC* aSchematic )
{
    std::vector<NET_ENTRY> nets;

    if( ADVANCED_CFG::GetCfg().m_ConnectivityEngine )
    {
        std::map<KIID_PATH, SCH_SHEET_PATH> paths;

        for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
            paths.emplace( path.Path(), path );

        for( const SCH_CONNECTIVITY::NET_GROUP& group : aSchematic->Connectivity().GetNetMap() )
        {
            NET_ENTRY entry;
            entry.name = str( group.name );

            for( const SCH_CONNECTIVITY::NET_VIEW& view : group.instances )
            {
                if( !view.IsNet() )
                    continue;

                auto path = paths.find( view.Instance() );

                if( path == paths.end() )
                    continue;

                entry.instances.push_back( { path->second, view.Items(), str( view.Name( true ) ) } );
            }

            nets.push_back( std::move( entry ) );
        }
    }
    else
    {
        for( const auto& [key, subgraphs] : aSchematic->ConnectionGraph()->GetNetMap() )
        {
            NET_ENTRY entry;
            entry.name = str( key.Name );

            for( const CONNECTION_SUBGRAPH* subgraph : subgraphs )
            {
                std::vector<SCH_ITEM*> items( subgraph->GetItems().begin(), subgraph->GetItems().end() );
                entry.instances.push_back( { subgraph->GetSheet(), items, str( subgraph->GetNetName() ) } );
            }

            nets.push_back( std::move( entry ) );
        }
    }

    std::sort( nets.begin(), nets.end(),
               []( const NET_ENTRY& a, const NET_ENTRY& b ) { return KopenapiNaturalLess( a.name, b.name ); } );

    for( size_t i = 0; i < nets.size(); ++i )
        nets[i].code = static_cast<int>( i + 1 );

    return nets;
}


std::string pinId( const SCH_PIN* aPin, const SCH_SHEET_PATH& aPath )
{
    SCH_SYMBOL* sym = pinSymbol( aPin );
    return ( sym ? str( sym->GetRef( &aPath, false ) ) : std::string() ) + "." + str( aPin->GetNumber() );
}


/**
 * Pins of a net across all its instances, one per REF.PIN: multi-unit symbols repeat shared
 * (e.g. power) pins in every unit; the netlist exporter lists them once as well.
 */
std::vector<std::pair<const SCH_PIN*, SCH_SHEET_PATH>> netPins( const NET_ENTRY& aNet )
{
    std::vector<std::pair<const SCH_PIN*, SCH_SHEET_PATH>> pins;
    std::set<std::string>                                  seen;

    for( const NET_INSTANCE& inst : aNet.instances )
    {
        for( SCH_ITEM* item : inst.items )
        {
            if( item->Type() == SCH_PIN_T && seen.insert( pinId( static_cast<SCH_PIN*>( item ), inst.path ) ).second )
                pins.emplace_back( static_cast<SCH_PIN*>( item ), inst.path );
        }
    }

    return pins;
}

} // namespace kopenapi_sch
