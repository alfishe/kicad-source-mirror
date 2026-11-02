/// @file methods_pcb_via_stack.cpp
/// @brief kicadopenapi microvia stacks (the Place Microvia Stack tool, Create Microvia Stack from
/// selected vias, the stack's properties dialog, the router's expansion of multi-hop microvias):
/// PCB_VIA_STACK's core functions, shared with the editor
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <footprint.h>
#include <generators/pcb_via_stack.h>
#include <layer_range.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_field.h>
#include <pcb_track.h>
#include <string_utils.h>
#include <tools/drawing_tool.h>
#include <tools/generator_tool.h>
#include <zone.h>

#include <cmath>
#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double toMm( long long aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1e4 ) / 1e4;
}


int fromMm( double aMm )
{
    return KiROUND( aMm * pcbIUScale.IU_PER_MM );
}


nlohmann::json pointJson( const VECTOR2I& aPoint )
{
    return { toMm( aPoint.x ), toMm( aPoint.y ) };
}


bool pointArg( const nlohmann::json& aValue, VECTOR2I& aPoint )
{
    if( !aValue.is_array() || aValue.size() != 2 || !aValue[0].is_number() || !aValue[1].is_number() )
        return false;

    aPoint = VECTOR2I( fromMm( aValue[0].get<double>() ), fromMm( aValue[1].get<double>() ) );
    return true;
}


nlohmann::json presetJson( BOARD* aBoard, const VIA_STACK_PRESET& aPreset )
{
    return { { "name", str( aPreset.m_Name ) },
             { "start_layer", str( aBoard->GetLayerName( aPreset.m_StartLayer ) ) },
             { "end_layer", str( aBoard->GetLayerName( aPreset.m_EndLayer ) ) },
             { "style", aPreset.m_Staggered ? "staggered" : "stacked" },
             { "pitch_mm", toMm( aPreset.m_Pitch ) },
             { "via_size_mm", toMm( aPreset.m_ViaSize ) },
             { "drill_mm", toMm( aPreset.m_ViaDrill ) },
             { "use_netclass", aPreset.m_UseNetclass },
             { "filled", aPreset.m_Filled },
             { "capped", aPreset.m_Capped } };
}


nlohmann::json stackJson( BOARD* aBoard, PCB_VIA_STACK* aStack )
{
    nlohmann::json answer = presetJson( aBoard, aStack->ToPreset() );
    nlohmann::json vias = nlohmann::json::array();
    size_t         tracks = 0;

    answer.erase( "name" );
    answer["preset"] = str( aStack->GetPresetName() );
    answer["uuid"] = str( aStack->m_Uuid.AsString() );

    NETINFO_ITEM* net = aStack->GetNetCode() > 0 ? aBoard->FindNet( aStack->GetNetCode() ) : nullptr;
    answer["net"] = net ? str( UnescapeString( net->GetNetname() ) ) : std::string();
    answer["at_mm"] = pointJson( aStack->GetPosition() );

    if( aStack->GetHops() )
    {
        nlohmann::json hops = nlohmann::json::array();

        for( const VECTOR2I& p : aStack->GetHops()->CPoints() )
            hops.push_back( pointJson( p ) );

        answer["hops_mm"] = hops;
    }

    for( BOARD_ITEM* item : aStack->GetBoardItems() )
    {
        if( PCB_VIA* via = dynamic_cast<PCB_VIA*>( item ) )
        {
            PCB_LAYER_ID top;
            PCB_LAYER_ID bottom;
            via->LayerPair( &top, &bottom );

            vias.push_back( { { "uuid", str( via->m_Uuid.AsString() ) },
                              { "at_mm", pointJson( via->GetPosition() ) },
                              { "layers", { str( aBoard->GetLayerName( top ) ), str( aBoard->GetLayerName( bottom ) ) } },
                              { "via_size_mm", toMm( via->GetWidth( PADSTACK::ALL_LAYERS ) ) },
                              { "drill_mm", toMm( via->GetDrillValue() ) } } );
        }
        else if( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T )
        {
            ++tracks;
        }
    }

    answer["via_list"] = vias;
    answer["tracks"] = tracks;
    answer["locked"] = aStack->IsLocked();
    return answer;
}


std::vector<PCB_VIA_STACK*> stacks( BOARD* aBoard )
{
    std::vector<PCB_VIA_STACK*> out;

    for( PCB_GENERATOR* generator : aBoard->Generators() )
    {
        if( PCB_VIA_STACK* stack = dynamic_cast<PCB_VIA_STACK*>( generator ) )
            out.push_back( stack );
    }

    return out;
}


PCB_VIA_STACK* findStack( BOARD* aBoard, const std::string& aUuid )
{
    for( PCB_VIA_STACK* stack : stacks( aBoard ) )
    {
        if( str( stack->m_Uuid.AsString() ) == aUuid )
            return stack;
    }

    return nullptr;
}


const VIA_STACK_PRESET* findPreset( BOARD* aBoard, const std::string& aName )
{
    for( const VIA_STACK_PRESET& preset : aBoard->GetDesignSettings().m_ViaStackPresets )
    {
        if( str( preset.m_Name ) == aName )
            return &preset;
    }

    return nullptr;
}


bool hasExplicitSettings( const nlohmann::json& aArgs )
{
    for( const char* key : { "start_layer", "end_layer", "style", "pitch_mm", "via_size_mm", "drill_mm",
                             "use_netclass", "filled", "capped" } )
    {
        if( aArgs.contains( key ) )
            return true;
    }

    return false;
}


/// @brief aArgs' explicit settings onto aPreset; an error text when one is malformed
std::string settingsArg( BOARD* aBoard, const nlohmann::json& aArgs, VIA_STACK_PRESET& aPreset )
{
    for( const char* key : { "start_layer", "end_layer" } )
    {
        if( !aArgs.contains( key ) )
            continue;

        const PCB_LAYER_ID layer = aArgs[key].is_string()
                                           ? aBoard->GetLayerID( wxString::FromUTF8( aArgs[key].get<std::string>() ) )
                                           : UNDEFINED_LAYER;

        if( layer == UNDEFINED_LAYER || !IsCopperLayer( layer ) )
            return std::string( key ) + ": a copper layer name of the board (F.Cu, In1.Cu, ...)";

        ( std::string( key ) == "start_layer" ? aPreset.m_StartLayer : aPreset.m_EndLayer ) = layer;
    }

    if( aArgs.contains( "style" ) )
    {
        const std::string style = aArgs["style"].is_string() ? aArgs["style"].get<std::string>() : std::string();

        if( style != "stacked" && style != "staggered" )
            return "style: stacked or staggered";

        aPreset.m_Staggered = style == "staggered";
    }

    for( const char* key : { "pitch_mm", "via_size_mm", "drill_mm" } )
    {
        if( !aArgs.contains( key ) )
            continue;

        if( !aArgs[key].is_number() || aArgs[key].get<double>() < 0 )
            return std::string( key ) + ": a number >= 0 (mm)";

        const int value = fromMm( aArgs[key].get<double>() );
        const std::string name( key );

        ( name == "pitch_mm" ? aPreset.m_Pitch : name == "via_size_mm" ? aPreset.m_ViaSize : aPreset.m_ViaDrill ) = value;
    }

    for( const char* key : { "use_netclass", "filled", "capped" } )
    {
        if( !aArgs.contains( key ) )
            continue;

        if( !aArgs[key].is_boolean() )
            return std::string( key ) + ": true or false";

        const std::string name( key );
        ( name == "use_netclass" ? aPreset.m_UseNetclass : name == "filled" ? aPreset.m_Filled : aPreset.m_Capped ) =
                aArgs[key].get<bool>();
    }

    // As the properties dialog: a stacked stack is always filled, netclass sizes are stored as 0
    if( !aPreset.m_Staggered )
        aPreset.m_Filled = true;

    if( aPreset.m_UseNetclass )
    {
        aPreset.m_ViaSize = 0;
        aPreset.m_ViaDrill = 0;
    }

    return std::string();
}


/// @brief The board items around aItem that the via tool's DRC check looks at (what the editor
/// takes from its view)
std::vector<BOARD_ITEM*> nearbyItems( BOARD* aBoard, const BOX2I& aBox )
{
    std::vector<BOARD_ITEM*> out;

    auto take = [&]( BOARD_ITEM* aItem )
    {
        if( aItem->GetBoundingBox().Intersects( aBox ) )
            out.push_back( aItem );
    };

    for( PCB_TRACK* track : aBoard->Tracks() )
        take( track );

    for( BOARD_ITEM* item : aBoard->Drawings() )
        take( item );

    for( ZONE* zone : aBoard->Zones() )
        take( zone );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( !fp->GetBoundingBox().Intersects( aBox ) )
            continue;

        for( PAD* pad : fp->Pads() )
            take( pad );

        for( BOARD_ITEM* item : fp->GraphicalItems() )
            take( item );

        for( PCB_FIELD* field : fp->GetFields() )
            take( field );

        for( ZONE* zone : fp->Zones() )
            take( zone );
    }

    return out;
}


KOPENAPI_RESULT h_pcb_via_stack_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                 board = context->GetBoard();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();

    nlohmann::json items = nlohmann::json::array();
    nlohmann::json presets = nlohmann::json::array();

    for( PCB_VIA_STACK* stack : stacks( board ) )
    {
        if( !aArgs.contains( "uuid" ) || aArgs.value( "uuid", std::string() ) == str( stack->m_Uuid.AsString() ) )
            items.push_back( stackJson( board, stack ) );
    }

    if( aArgs.contains( "uuid" ) && items.empty() )
        return KOPENAPI_RESULT::Error( 404, "no microvia stack " + aArgs.value( "uuid", std::string() ) + " (pcb_via_stack_list)" );

    for( const VIA_STACK_PRESET& preset : bds.m_ViaStackPresets )
        presets.push_back( presetJson( board, preset ) );

    nlohmann::json current;

    if( !bds.m_ViaStackPresets.empty() )
        current = str( bds.m_ViaStackPresets[std::clamp( bds.GetViaStackIndex(), 0, (int) bds.m_ViaStackPresets.size() - 1 )].m_Name );

    return KOPENAPI_RESULT::Ok( { { "items", items }, { "total", items.size() }, { "presets", presets },
                                  { "current_preset", current } } );
}


KOPENAPI_RESULT h_pcb_via_stack( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                 board = context->GetBoard();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    const std::string      op = aArgs.value( "op", std::string() );

    if( op == "select_preset" )
    {
        const std::string name = aArgs.value( "preset", std::string() );

        for( size_t i = 0; i < bds.m_ViaStackPresets.size(); ++i )
        {
            if( str( bds.m_ViaStackPresets[i].m_Name ) == name )
            {
                bds.SetViaStackIndex( (int) i );
                return KOPENAPI_RESULT::Ok( { { "current_preset", name }, { "index", i } } );
            }
        }

        return KOPENAPI_RESULT::Error( 404, "preset: no microvia stack preset " + nlohmann::json( name ).dump() + " (pcb_via_stack_list presets)" );
    }

    if( op == "set_presets" )
    {
        if( !aArgs.contains( "presets" ) || !aArgs["presets"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "presets: [{name, start_layer, end_layer, style, ...}], the board's whole list (Board Setup)" );

        std::vector<VIA_STACK_PRESET> presets;

        for( const nlohmann::json& entry : aArgs["presets"] )
        {
            if( !entry.is_object() || !entry.contains( "name" ) || !entry["name"].is_string() )
                return KOPENAPI_RESULT::Error( 400, "presets: each {name, ...}" );

            VIA_STACK_PRESET preset;
            preset.m_Name = wxString::FromUTF8( entry["name"].get<std::string>() );
            preset.m_Name.Trim( true ).Trim( false );

            if( preset.m_Name.IsEmpty() )
                return KOPENAPI_RESULT::Error( 400, "presets: a name is empty" );

            for( const VIA_STACK_PRESET& other : presets )
            {
                if( other.m_Name.CmpNoCase( preset.m_Name ) == 0 )
                    return KOPENAPI_RESULT::Error( 400, "presets: " + str( wxString::Format( _( "A preset named \"%s\" already exists." ), preset.m_Name ) ) );
            }

            if( std::string bad = settingsArg( board, entry, preset ); !bad.empty() )
                return KOPENAPI_RESULT::Error( 400, "presets " + str( preset.m_Name ) + ": " + bad );

            if( std::optional<wxString> bad = PCB_VIA_STACK::CheckSettings( board, preset ) )
                return KOPENAPI_RESULT::Error( 400, "presets " + str( preset.m_Name ) + ": " + str( *bad ) );

            presets.push_back( preset );
        }

        // As Board Setup: the current preset is followed by name
        wxString active;

        if( aArgs.contains( "current_preset" ) && aArgs["current_preset"].is_string() )
            active = wxString::FromUTF8( aArgs["current_preset"].get<std::string>() );
        else if( !bds.m_ViaStackPresets.empty() )
            active = bds.m_ViaStackPresets[std::clamp( bds.GetViaStackIndex(), 0, (int) bds.m_ViaStackPresets.size() - 1 )].m_Name;

        int index = 0;

        for( int i = 0; i < (int) presets.size(); ++i )
        {
            if( presets[i].m_Name == active )
                index = i;
        }

        bds.m_ViaStackPresets = presets;
        bds.SetViaStackIndex( index );
        context->SetContentModified();

        nlohmann::json out = nlohmann::json::array();

        for( const VIA_STACK_PRESET& preset : presets )
            out.push_back( presetJson( board, preset ) );

        return KOPENAPI_RESULT::Ok( { { "presets", out },
                                      { "current_preset", presets.empty() ? nlohmann::json() : nlohmann::json( str( presets[index].m_Name ) ) } } );
    }

    static const std::set<std::string> ops = { "place", "create", "update", "remove", "expand" };

    if( !ops.count( op ) )
        return KOPENAPI_RESULT::Error( 400, "op: select_preset, set_presets, place, create, update, remove or expand (pcb_via_stack_list lists)" );

    GENERATOR_TOOL* tool = KopenapiGeneratorTool( context->GetToolManager() );

    if( !tool )
        return KOPENAPI_RESULT::Error( 503, "the generator tool is not available" );

    if( aArgs.contains( "preset" ) && !( aArgs["preset"].is_string() && findPreset( board, aArgs["preset"].get<std::string>() ) ) )
        return KOPENAPI_RESULT::Error( 404, "preset: no microvia stack preset " + aArgs["preset"].dump() + " (pcb_via_stack_list presets)" );

    BOARD_COMMIT commit( context->GetToolManager() );

    if( op == "create" )
    {
        if( !aArgs.contains( "items" ) || !aArgs["items"].is_array() || aArgs["items"].empty() )
            return KOPENAPI_RESULT::Error( 400, "items: uuids of the microvias (and connecting tracks) that make the stack" );

        std::vector<BOARD_ITEM*> items;

        for( const nlohmann::json& id : aArgs["items"] )
        {
            std::string error;
            BOARD_ITEM* item = id.is_string() ? KopenapiResolveUuid( board, id.get<std::string>(), error ) : nullptr;

            if( !item )
                return KOPENAPI_RESULT::Error( 404, "items: " + ( error.empty() ? "no item " + id.dump() : error ) );

            if( item->Type() != PCB_VIA_T && item->Type() != PCB_TRACE_T )
                return KOPENAPI_RESULT::Error( 400, "items: " + id.dump() + " is not a via or a track" );

            items.push_back( item );
        }

        wxString       error;
        PCB_VIA_STACK* stack = PCB_VIA_STACK::MakeFromItems( items, board, commit, &error );

        if( !stack )
            return KOPENAPI_RESULT::Error( 422, str( error ) );

        commit.Push( _( "Create Microvia Stack" ) );
        context->SetContentModified();

        nlohmann::json answer = stackJson( board, stack );
        answer["description"] = "Create Microvia Stack";
        return KOPENAPI_RESULT::Ok( answer );
    }

    if( op == "expand" )
    {
        std::set<KIID> only;

        if( aArgs.contains( "vias" ) )
        {
            if( !aArgs["vias"].is_array() || aArgs["vias"].empty() )
                return KOPENAPI_RESULT::Error( 400, "vias: uuids of microvias spanning several layer steps" );

            for( const nlohmann::json& id : aArgs["vias"] )
            {
                std::string error;
                BOARD_ITEM* item = id.is_string() ? KopenapiResolveUuid( board, id.get<std::string>(), error ) : nullptr;

                if( !item || item->Type() != PCB_VIA_T )
                    return KOPENAPI_RESULT::Error( 404, "vias: no via " + id.dump() );

                only.insert( item->m_Uuid );
            }
        }

        const VIA_STACK_PRESET* preset = aArgs.contains( "preset" ) ? findPreset( board, aArgs["preset"].get<std::string>() ) : nullptr;
        const VIA_STACK_PRESET  plain = PCB_VIA_STACK( board ).ToPreset();
        std::set<KIID>          before;

        for( PCB_VIA_STACK* stack : stacks( board ) )
            before.insert( stack->m_Uuid );

        std::function<const VIA_STACK_PRESET*( PCB_VIA* )> matcher;

        if( preset || !only.empty() )
        {
            matcher = [&]( PCB_VIA* aVia ) -> const VIA_STACK_PRESET*
            {
                if( !only.empty() && !only.count( aVia->m_Uuid ) )
                    return nullptr;

                return preset ? preset : &plain;
            };
        }

        const int expanded = PCB_VIA_STACK::ExpandMultiHopMicrovias( board, &commit, matcher );

        if( expanded == 0 )
            return KOPENAPI_RESULT::Error( 422, "no microvia spanning several layer steps to expand" );

        commit.Push( _( "Expand Microvia Stacks" ) );
        context->SetContentModified();

        nlohmann::json items = nlohmann::json::array();

        for( PCB_VIA_STACK* stack : stacks( board ) )
        {
            if( !before.count( stack->m_Uuid ) )
                items.push_back( stackJson( board, stack ) );
        }

        return KOPENAPI_RESULT::Ok( { { "items", items }, { "expanded", expanded }, { "description", "Expand Microvia Stacks" } } );
    }

    if( op == "place" )
    {
        VIA_STACK_PRESET settings;

        if( aArgs.contains( "preset" ) )
            settings = *findPreset( board, aArgs["preset"].get<std::string>() );
        else if( !bds.m_ViaStackPresets.empty() )
            settings = bds.m_ViaStackPresets[std::clamp( bds.GetViaStackIndex(), 0, (int) bds.m_ViaStackPresets.size() - 1 )];
        else if( !hasExplicitSettings( aArgs ) )
            return KOPENAPI_RESULT::Error( 400, "no microvia stack presets on this board: give the settings (start_layer, end_layer, style, ...) or a preset" );
        else
            settings = PCB_VIA_STACK( board ).ToPreset();

        if( std::string bad = settingsArg( board, aArgs, settings ); !bad.empty() )
            return KOPENAPI_RESULT::Error( 400, bad );

        PCB_VIA_STACK stackSettings( board, F_Cu );
        stackSettings.ApplyPreset( settings );

        // As the placement tool: a staggered stack without a pitch steps two via diameters
        if( settings.m_Staggered && settings.m_Pitch <= 0 )
        {
            const int dia = settings.m_ViaSize > 0 ? settings.m_ViaSize : bds.GetCurrentViaSize();
            settings.m_Pitch = dia * 2;
            stackSettings.SetPitch( settings.m_Pitch );
        }

        if( std::optional<wxString> bad = PCB_VIA_STACK::CheckSettings( board, settings ) )
            return KOPENAPI_RESULT::Error( 400, str( *bad ) );

        int nHops = -1;

        for( PCB_LAYER_ID layer : LAYER_RANGE( settings.m_StartLayer, settings.m_EndLayer, board->GetCopperLayerCount() ) )
        {
            ( void ) layer;
            ++nHops;
        }

        VECTOR2I              at;
        std::vector<VECTOR2I> hops;

        if( aArgs.contains( "hops_mm" ) )
        {
            if( !settings.m_Staggered )
                return KOPENAPI_RESULT::Error( 400, "hops_mm: only for a staggered stack (at_mm places a stacked one)" );

            if( !aArgs["hops_mm"].is_array() || (int) aArgs["hops_mm"].size() != nHops )
            {
                return KOPENAPI_RESULT::Error( 400, "hops_mm: " + std::to_string( nHops ) + " points [x_mm, y_mm], one per hop of the span" );
            }

            for( const nlohmann::json& p : aArgs["hops_mm"] )
            {
                VECTOR2I point;

                if( !pointArg( p, point ) )
                    return KOPENAPI_RESULT::Error( 400, "hops_mm: points [x_mm, y_mm]" );

                hops.push_back( point );
            }

            at = hops.front();
        }
        else
        {
            if( !aArgs.contains( "at_mm" ) || !pointArg( aArgs["at_mm"], at ) )
                return KOPENAPI_RESULT::Error( 400, "at_mm: [x_mm, y_mm] (or hops_mm for a staggered stack)" );

            if( settings.m_Staggered )
            {
                // The tool's first hop at the click, each next one a pitch further in a direction
                const double angle = aArgs.value( "direction_deg", 0.0 ) * M_PI / 180.0;
                const VECTOR2I step( KiROUND( std::cos( angle ) * settings.m_Pitch ),
                                     KiROUND( std::sin( angle ) * settings.m_Pitch ) );

                for( int i = 0; i < nHops; ++i )
                    hops.push_back( at + step * i );
            }
        }

        std::shared_ptr<DRC_ENGINE> drcEngine = bds.m_DRCEngine;
        const int                   epsilon = bds.GetDRCEpsilon();
        const int                   worst = drcEngine ? ComputeWorstViaClearance( board, drcEngine.get() ) : 0;
        nlohmann::json              violations = nlohmann::json::array();

        auto violates = [&]( BOARD_CONNECTED_ITEM* aItem ) -> bool
        {
            if( !drcEngine )
                return false;

            BOX2I box = aItem->GetBoundingBox();
            box.Inflate( worst );

            if( !CheckItemDRCViolation( aItem, nearbyItems( board, box ), drcEngine.get(), epsilon ) )
                return false;

            violations.push_back( pointJson( aItem->GetPosition() ) );
            return true;
        };

        const bool     allow = aArgs.value( "allow_violations", false );
        PCB_VIA_STACK* stack = PCB_VIA_STACK::Place( board, stackSettings, at, hops, tool, commit, violates, allow );

        if( !stack )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "Microvia stack location violates DRC (allow_violations places it anyway)" );
            r.body["violations_at_mm"] = violations;
            return r;
        }

        commit.Push( _( "Place Microvia Stack" ) );
        context->SetContentModified();

        nlohmann::json answer = stackJson( board, stack );
        answer["description"] = "Place Microvia Stack";

        if( !violations.empty() )
            answer["violations_at_mm"] = violations;

        return KOPENAPI_RESULT::Ok( answer );
    }

    PCB_VIA_STACK* stack = aArgs.contains( "uuid" ) && aArgs["uuid"].is_string()
                                   ? findStack( board, aArgs["uuid"].get<std::string>() )
                                   : nullptr;

    if( !stack )
        return KOPENAPI_RESULT::Error( aArgs.contains( "uuid" ) ? 404 : 400, "uuid: a microvia stack (pcb_via_stack_list)" );

    if( stack->IsLocked() && !aArgs.value( "override_locks", false ) )
        return KOPENAPI_RESULT::Error( 409, "microvia stack " + aArgs["uuid"].get<std::string>() + " is locked (override_locks)" );

    if( op == "remove" )
    {
        const size_t vias = stackJson( board, stack )["via_list"].size();
        stack->Remove( tool, board, &commit );
        commit.Push( _( "Remove Microvia Stack" ) );
        context->SetContentModified();
        return KOPENAPI_RESULT::Ok( { { "removed", aArgs["uuid"] }, { "vias_removed", vias } } );
    }

    // update: the properties dialog's OK
    VIA_STACK_PRESET settings = aArgs.contains( "preset" ) ? *findPreset( board, aArgs["preset"].get<std::string>() )
                                                           : stack->ToPreset();

    if( std::string bad = settingsArg( board, aArgs, settings ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad );

    if( std::optional<wxString> bad = PCB_VIA_STACK::CheckSettings( board, settings ) )
        return KOPENAPI_RESULT::Error( 400, str( *bad ) );

    stack->ApplySettings( settings, tool, commit );
    commit.Push( stack->GetCommitMessage() );
    context->SetContentModified();

    nlohmann::json answer = stackJson( board, stack );
    answer["description"] = str( stack->GetCommitMessage() );
    return KOPENAPI_RESULT::Ok( answer );
}

} // namespace


KOPENAPI_REGISTER( "pcb_via_stack_list",
                   "Microvia stacks (single-hop microvias, plus connecting tracks when staggered, edited as one), uuid "
                   "for one: each stack's span, style, pitch, via size / drill, net, hops and vias, plus the board's "
                   "presets and the current one",
                   R"json({"type":"object","properties":{"uuid":{"type":"string"}}})json"_json,
                   false, h_pcb_via_stack_list );

KOPENAPI_REGISTER( "pcb_via_stack",
                   "Edit microvia stacks (single-hop microvias, plus connecting tracks when staggered, edited as one; "
                   "pcb_via_stack_list lists them): op select_preset (the toolbar's preset); op "
                   "set_presets (Board Setup's Microvia Stacks: presets, the whole list, and current_preset); op "
                   "place (Place Microvia Stack): at_mm (staggered: hops step pitch along direction_deg) or hops_mm "
                   "(staggered, one point per hop), with the current preset, preset, or explicit settings; the net is "
                   "the copper's under the first hop; refused (409) over foreign copper unless allow_violations; op "
                   "create (Create Microvia Stack): items, the uuids of the vias (and tracks); op update (uuid): the "
                   "properties dialog's settings or a preset, regenerated; op remove (uuid) with its vias; op expand "
                   "(the router's Expand Microvia Stacks): microvias spanning several layer steps (all, or vias) become "
                   "stacks, with preset's attributes if given. One undo step each",
                   R"json({"type":"object","required":["op"],"properties":{
                        "op":{"type":"string","enum":["select_preset","set_presets","place","create","update","remove","expand"]},
                        "uuid":{"type":"string"},
                        "preset":{"type":"string","description":"a microvia stack preset name (pcb_via_stack_list presets)"},
                        "presets":{"type":"array","items":{"type":"object"},"description":"set_presets: [{name, start_layer, end_layer, style, pitch_mm, via_size_mm, drill_mm, use_netclass, filled, capped}]"},
                        "current_preset":{"type":"string","description":"set_presets: the toolbar's preset"},
                        "at_mm":{"type":"array","items":{"type":"number"}},
                        "hops_mm":{"type":"array","items":{"type":"array","items":{"type":"number"}},"description":"staggered: one point per hop"},
                        "direction_deg":{"type":"number","default":0,"description":"staggered at_mm: the direction of the hops"},
                        "start_layer":{"type":"string"},"end_layer":{"type":"string"},
                        "style":{"type":"string","enum":["stacked","staggered"]},
                        "pitch_mm":{"type":"number"},"via_size_mm":{"type":"number"},"drill_mm":{"type":"number"},
                        "use_netclass":{"type":"boolean"},"filled":{"type":"boolean"},"capped":{"type":"boolean"},
                        "allow_violations":{"type":"boolean","default":false},
                        "items":{"type":"array","items":{"type":"string"},"description":"create: vias and tracks"},
                        "vias":{"type":"array","items":{"type":"string"},"description":"expand: only these vias"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_via_stack );

KOPENAPI_MARK_EDITING( "pcb_via_stack" );
