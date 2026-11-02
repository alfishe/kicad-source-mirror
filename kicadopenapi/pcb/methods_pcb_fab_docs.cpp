/// @file methods_pcb_fab_docs.cpp
/// @brief kicadopenapi fabrication documentation: pcb_dimension_add, pcb_fab_tables_add.
///
/// Dimensions (aligned, orthogonal, center, radial, leader) with the board's dimension defaults,
/// and the generated documentation of Place > Board Characteristics / Stackup Table / Drill
/// Chart / Drill Map; one commit per call.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <board_tables/generated_table_refresh.h>
#include <board_tables/generated_table_schema.h>
#include <frame_type.h>
#include <kicadopenapi_glow_view.h>
#include <kiway.h>
#include <pcb_dimension.h>
#include <pcb_drill_chart.h>
#include <pcb_drill_map.h>
#include <pcb_edit_frame.h>
#include <pcb_table.h>
#include <tool/tool_manager.h>

#include <cmath>
#include <optional>
#include <set>


PCB_TABLE* Build_Board_Stackup_Table( BOARD* aBoard, EDA_UNITS aDisplayUnits );
PCB_TABLE* Build_Board_Characteristics_Table( BOARD* aBoard, EDA_UNITS aDisplayUnits );


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


int toIU( double aMm )
{
    return pcbIUScale.mmToIU( aMm );
}


double mm( int aIU )
{
    return std::round( aIU / pcbIUScale.IU_PER_MM * 1000 ) / 1000;
}


nlohmann::json rectMm( const BOX2I& aBox )
{
    return { mm( aBox.GetLeft() ), mm( aBox.GetTop() ), mm( aBox.GetRight() ), mm( aBox.GetBottom() ) };
}


bool isPoint( const nlohmann::json& aPoint )
{
    return aPoint.is_array() && aPoint.size() == 2 && aPoint[0].is_number() && aPoint[1].is_number();
}


VECTOR2I point( const nlohmann::json& aPoint )
{
    return VECTOR2I( toIU( aPoint[0].get<double>() ), toIU( aPoint[1].get<double>() ) );
}


struct FAB_GLOW_TRAITS
{
    using FRAME = PCB_EDIT_FRAME;

    static FRAME* Frame( KIWAY* aKiway )
    {
        return aKiway ? dynamic_cast<PCB_EDIT_FRAME*>( aKiway->Player( FRAME_PCB_EDITOR, false ) ) : nullptr;
    }

    static EDA_ITEM* Resolve( FRAME* aFrame, const KIID& aId )
    {
        return aFrame->GetBoard() ? aFrame->GetBoard()->ResolveItem( aId, true ) : nullptr;
    }

    static void Brighten( FRAME* aFrame, EDA_ITEM* aItem, bool aOn ) { KopenapiPcbBrighten( aFrame, aItem, aOn ); }

    static BOX2I Box( EDA_ITEM* aItem ) { return aItem->GetBoundingBox(); }
    static int   Mm() { return pcbIUScale.mmToIU( 1.0 ); }
    static void  ItemColour( FRAME*, bool, std::optional<KIGFX::COLOR4D>& ) {}
};


void glow( KOPENAPI_CONTEXT& aCtx, const std::vector<KIID>& aIds )
{
    if( aCtx.headless )
        return;

    if( auto* frame = FAB_GLOW_TRAITS::Frame( aCtx.kiway ) )
        frame->GetCanvas()->Refresh();

    KopenapiGlow<FAB_GLOW_TRAITS>( aCtx.kiway, aIds, 0 );
}


/// @brief The named enabled layer, or nullopt
std::optional<PCB_LAYER_ID> namedLayer( BOARD* aBoard, const std::string& aName )
{
    const int id = aBoard->GetLayerID( wxString::FromUTF8( aName ) );

    if( id < 0 || !aBoard->IsLayerEnabled( (PCB_LAYER_ID) id ) )
        return std::nullopt;

    return (PCB_LAYER_ID) id;
}


std::optional<DIM_UNITS_MODE> dimUnits( const std::string& aName )
{
    if( aName == "mm" )
        return DIM_UNITS_MODE::MM;
    if( aName == "in" )
        return DIM_UNITS_MODE::INCH;
    if( aName == "mils" )
        return DIM_UNITS_MODE::MILS;
    if( aName == "auto" )
        return DIM_UNITS_MODE::AUTOMATIC;

    return std::nullopt;
}


const char* kindName( KICAD_T aType )
{
    switch( aType )
    {
    case PCB_DIM_ALIGNED_T:    return "aligned";
    case PCB_DIM_ORTHOGONAL_T: return "orthogonal";
    case PCB_DIM_CENTER_T:     return "center";
    case PCB_DIM_RADIAL_T:     return "radial";
    case PCB_DIM_LEADER_T:     return "leader";
    default:                   return "";
    }
}

} // namespace


static KOPENAPI_RESULT h_pcb_dimension_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                      board = context->GetBoard();
    BOARD_DESIGN_SETTINGS&      bds = board->GetDesignSettings();
    std::optional<PCB_LAYER_ID> layer = namedLayer( board, aArgs.value( "layer", std::string( "Dwgs.User" ) ) );

    if( !layer )
        return KOPENAPI_RESULT::Error( 400, "layer: an enabled board layer (pcb_layer_list)" );

    const std::string kind = aArgs.value( "kind", std::string() );
    const nlohmann::json start = aArgs.value( "start", nlohmann::json() );
    const nlohmann::json end = aArgs.value( "end", nlohmann::json() );

    if( !isPoint( start ) || !isPoint( end ) )
        return KOPENAPI_RESULT::Error( 400, "start and end: [x_mm, y_mm]" );

    if( point( start ) == point( end ) )
        return KOPENAPI_RESULT::Error( 400, "start and end are the same point" );

    std::optional<DIM_UNITS_MODE> units = bds.m_DimensionUnitsMode;

    if( aArgs.contains( "units" ) )
    {
        units = dimUnits( aArgs.value( "units", std::string() ) );

        if( !units )
            return KOPENAPI_RESULT::Error( 400, "units: mm, in, mils or auto" );
    }

    std::unique_ptr<PCB_DIMENSION_BASE> dim;
    const bool measuring = kind == "aligned" || kind == "orthogonal" || kind == "radial";

    if( kind == "aligned" )
        dim = std::make_unique<PCB_DIM_ALIGNED>( board );
    else if( kind == "orthogonal" )
        dim = std::make_unique<PCB_DIM_ORTHOGONAL>( board );
    else if( kind == "center" )
        dim = std::make_unique<PCB_DIM_CENTER>( board );
    else if( kind == "radial" )
        dim = std::make_unique<PCB_DIM_RADIAL>( board );
    else if( kind == "leader" )
        dim = std::make_unique<PCB_DIM_LEADER>( board );
    else
        return KOPENAPI_RESULT::Error( 400, "kind: aligned, orthogonal, center, radial or leader" );

    if( measuring )
    {
        dim->SetUnitsMode( *units );
        dim->SetUnitsFormat( bds.m_DimensionUnitsFormat );
        dim->SetPrecision( bds.m_DimensionPrecision );
        dim->SetSuppressZeroes( bds.m_DimensionSuppressZeroes );
        dim->SetTextPositionMode( bds.m_DimensionTextPosition );
        dim->SetKeepTextAligned( bds.m_DimensionKeepTextAligned );

        if( aArgs.contains( "precision" ) )
            dim->SetPrecision( (DIM_PRECISION) std::clamp( aArgs.value( "precision", 2 ), 0, 5 ) );
    }

    dim->SetLayer( *layer );
    dim->SetMirrored( board->IsBackLayer( *layer ) );
    dim->SetTextSize( bds.GetTextSize( *layer ) );
    dim->SetTextThickness( bds.GetTextThickness( *layer ) );
    dim->SetItalic( bds.GetTextItalic( *layer ) );
    dim->SetLineThickness( aArgs.contains( "line_width_mm" ) ? toIU( aArgs.value( "line_width_mm", 0.1 ) )
                                                             : bds.GetLineThickness( *layer ) );
    dim->SetArrowLength( bds.m_DimensionArrowLength );
    dim->SetExtensionOffset( bds.m_DimensionExtensionOffset );

    if( aArgs.contains( "text_size_mm" ) )
    {
        const int size = toIU( aArgs.value( "text_size_mm", 1.0 ) );
        dim->SetTextSize( VECTOR2I( size, size ) );
        dim->SetTextThickness( std::max( 1, size * 15 / 100 ) );
    }

    dim->SetStart( point( start ) );
    dim->SetEnd( point( end ) );

    if( aArgs.contains( "prefix" ) )
        dim->SetPrefix( wxString::FromUTF8( aArgs.value( "prefix", std::string() ) ) );

    if( aArgs.contains( "suffix" ) )
        dim->SetSuffix( wxString::FromUTF8( aArgs.value( "suffix", std::string() ) ) );

    const std::string text = aArgs.value( "text", std::string() );

    if( kind == "leader" && text.empty() )
        return KOPENAPI_RESULT::Error( 400, "leader: text is required" );

    if( !text.empty() )
        dim->ChangeOverrideText( wxString::FromUTF8( text ) );

    const VECTOR2I textOffset( dim->GetArrowLength() * 10 * ( dim->GetEnd().x < dim->GetStart().x ? -1 : 1 ), 0 );

    if( kind == "aligned" )
    {
        static_cast<PCB_DIM_ALIGNED*>( dim.get() )->SetHeight( toIU( aArgs.value( "height_mm", 2.0 ) ) );
    }
    else if( kind == "orthogonal" )
    {
        auto*       ortho = static_cast<PCB_DIM_ORTHOGONAL*>( dim.get() );
        const BOX2I box( dim->GetStart(), dim->GetEnd() - dim->GetStart() );
        std::string dir = aArgs.value( "orientation", std::string() );

        if( dir.empty() )
            dir = std::abs( box.GetWidth() ) < std::abs( box.GetHeight() ) ? "vertical" : "horizontal";

        if( dir != "horizontal" && dir != "vertical" )
            return KOPENAPI_RESULT::Error( 400, "orientation: horizontal or vertical" );

        ortho->SetOrientation( dir == "vertical" ? PCB_DIM_ORTHOGONAL::DIR::VERTICAL
                                                 : PCB_DIM_ORTHOGONAL::DIR::HORIZONTAL );
        ortho->SetHeight( toIU( aArgs.value( "height_mm", 2.0 ) ) );
    }
    else if( kind == "radial" )
    {
        auto* radial = static_cast<PCB_DIM_RADIAL*>( dim.get() );

        if( aArgs.contains( "leader_length_mm" ) )
            radial->SetLeaderLength( toIU( aArgs.value( "leader_length_mm", 1.0 ) ) );

        radial->SetTextPos( radial->GetKnee() + textOffset );
    }
    else if( kind == "leader" )
    {
        dim->SetTextPos( dim->GetEnd() + textOffset );
    }

    if( aArgs.contains( "text_pos" ) )
    {
        if( !isPoint( aArgs["text_pos"] ) )
            return KOPENAPI_RESULT::Error( 400, "text_pos: [x_mm, y_mm]" );

        if( measuring && kind != "radial" )
            dim->SetTextPositionMode( DIM_TEXT_POSITION::MANUAL );

        dim->SetTextPos( point( aArgs["text_pos"] ) );
    }

    dim->Update();

    PCB_DIMENSION_BASE* item = dim.release();
    BOARD_COMMIT        commit( context->GetToolManager() );
    commit.Add( item );
    commit.Push( _( "Add dimension (API)" ) );
    glow( aCtx, { item->m_Uuid } );

    nlohmann::json out = { { "uuid", str( item->m_Uuid.AsString() ) },
                           { "kind", kindName( item->Type() ) },
                           { "layer", str( board->GetLayerName( *layer ) ) },
                           { "text", str( item->GetShownText( FOR_GUI ) ) },
                           { "bbox_mm", rectMm( item->GetBoundingBox() ) } };

    if( measuring )
        out["measured_mm"] = mm( item->GetMeasuredValue() );

    return KOPENAPI_RESULT::Ok( out );
}


namespace
{

/// @brief A documentation layer for a generated table or map: the named one, else the first
/// enabled one that aAvailable accepts
std::optional<PCB_LAYER_ID> docLayer( BOARD* aBoard, const nlohmann::json& aArgs,
                                      const std::function<bool( PCB_LAYER_ID )>& aAvailable, std::string& aError,
                                      int& aCode )
{
    if( aArgs.contains( "layer" ) )
    {
        std::optional<PCB_LAYER_ID> layer = namedLayer( aBoard, aArgs.value( "layer", std::string() ) );

        if( !layer || !DocumentationLayers().Contains( *layer ) )
        {
            aError = "layer: an enabled documentation layer (User.*, Dwgs.User, Cmts.User, Eco*.User, *.Fab)";
            aCode = 400;
            return std::nullopt;
        }

        if( !aAvailable( *layer ) )
        {
            aError = "layer already holds a drill map";
            return std::nullopt;
        }

        return layer;
    }

    for( PCB_LAYER_ID layer : DocumentationLayers().Seq() )
    {
        if( aBoard->IsLayerEnabled( layer ) && aAvailable( layer ) )
            return layer;
    }

    aError = "no enabled documentation layer is free";
    return std::nullopt;
}


} // namespace


static KOPENAPI_RESULT h_pcb_fab_tables_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*         board = context->GetBoard();
    nlohmann::json kinds = aArgs.value( "kinds", nlohmann::json::array() );

    if( !kinds.is_array() || kinds.empty() )
        return KOPENAPI_RESULT::Error( 400, "kinds: one or more of characteristics, stackup, drill_chart, drill_map" );

    for( const nlohmann::json& k : kinds )
    {
        const std::string name = k.is_string() ? k.get<std::string>() : std::string();

        if( name != "characteristics" && name != "stackup" && name != "drill_chart" && name != "drill_map" )
            return KOPENAPI_RESULT::Error( 400, "kinds: characteristics, stackup, drill_chart or drill_map" );
    }

    const std::string unitsName = aArgs.value( "units", std::string( "mm" ) );
    EDA_UNITS         units = EDA_UNITS::MM;

    if( unitsName == "in" )
        units = EDA_UNITS::INCH;
    else if( unitsName == "mils" )
        units = EDA_UNITS::MILS;
    else if( unitsName != "mm" )
        return KOPENAPI_RESULT::Error( 400, "units: mm, in or mils" );

    auto anyDocLayer = [board]( PCB_LAYER_ID aLayer )
    {
        return DocumentationLayers().Contains( aLayer ) && board->IsLayerEnabled( aLayer );
    };

    std::set<PCB_LAYER_ID> mapLayers;

    auto freeForMap = [board, &anyDocLayer, &mapLayers]( PCB_LAYER_ID aLayer )
    {
        if( !anyDocLayer( aLayer ) || mapLayers.count( aLayer ) )
            return false;

        for( const BOARD_ITEM* item : board->Drawings() )
        {
            if( item->Type() == PCB_DRILL_MAP_T && item->GetLayer() == aLayer )
                return false;
        }

        return true;
    };

    BOARD_COMMIT            commit( context->GetToolManager() );
    GENERATED_TABLE_REFRESH refresh( *board );
    std::vector<KIID>       ids;
    nlohmann::json          placed = nlohmann::json::array();
    const int               gap = pcbIUScale.mmToIU( 5 );
    const bool              at = aArgs.contains( "x_mm" ) || aArgs.contains( "y_mm" );
    const BOX2I             edges = board->GetBoardEdgesBoundingBox();
    const VECTOR2I          base = at ? VECTOR2I( toIU( aArgs.value( "x_mm", 0.0 ) ), toIU( aArgs.value( "y_mm", 0.0 ) ) )
                                      : VECTOR2I( edges.GetRight() + pcbIUScale.mmToIU( 10 ), edges.GetTop() );
    int                     cascade = 0;

    // Right of the board, below the tables already there
    if( !at )
    {
        for( const BOARD_ITEM* item : board->Drawings() )
        {
            if( dynamic_cast<const PCB_TABLE*>( item ) && item->GetBoundingBox().GetLeft() > edges.GetRight() )
                cascade = std::max( cascade, item->GetBoundingBox().GetBottom() - edges.GetTop() + gap );
        }
    }

    for( const nlohmann::json& k : kinds )
    {
        const std::string kind = k.get<std::string>();
        std::string       error;
        int               code = 409;
        BOARD_ITEM*       item = nullptr;

        if( kind == "drill_map" )
        {
            std::optional<PCB_LAYER_ID> layer = docLayer( board, aArgs, freeForMap, error, code );

            if( !layer )
            {
                commit.Revert();
                return KOPENAPI_RESULT::Error( code, "drill_map: " + error );
            }

            auto* map = new PCB_DRILL_MAP( board );
            map->SetLayer( *layer );
            mapLayers.insert( *layer );
            commit.Add( map );
            item = map;
        }
        else
        {
            std::optional<PCB_LAYER_ID> layer = docLayer( board, aArgs, anyDocLayer, error, code );

            if( !layer )
            {
                commit.Revert();
                return KOPENAPI_RESULT::Error( code, kind + ": " + error );
            }

            PCB_TABLE* table = nullptr;

            if( kind == "characteristics" )
            {
                table = Build_Board_Characteristics_Table( board, units );
            }
            else if( kind == "stackup" )
            {
                table = Build_Board_Stackup_Table( board, units );
            }
            else
            {
                auto* chart = new PCB_DRILL_CHART( board );
                chart->SetUnits( units == EDA_UNITS::INCH   ? GENERATED_TABLE_UNITS::INCH
                                 : units == EDA_UNITS::MILS ? GENERATED_TABLE_UNITS::MILS
                                                            : GENERATED_TABLE_UNITS::MM );
                chart->SetLayer( *layer );
                chart->RebuildCells( *board, &refresh );
                table = chart;
            }

            table->SetLayer( *layer );
            table->Move( base + VECTOR2I( 0, cascade ) - table->GetBoundingBox().GetOrigin() );
            cascade += table->GetBoundingBox().GetHeight() + gap;

            commit.Add( table );
            item = table;
        }

        ids.push_back( item->m_Uuid );
        placed.push_back( { { "kind", kind },
                            { "uuid", str( item->m_Uuid.AsString() ) },
                            { "layer", str( board->GetLayerName( item->GetLayer() ) ) },
                            { "bbox_mm", rectMm( item->GetBoundingBox() ) } } );

        if( auto* table = dynamic_cast<PCB_TABLE*>( item ) )
        {
            placed.back()["rows"] = table->GetRowCount();
            placed.back()["cols"] = table->GetColCount();
        }
    }

    // The drill profile a chart refresh computed is design settings, outside the undo stack
    refresh.Commit();
    commit.Push( _( "Add fabrication tables (API)" ) );
    glow( aCtx, ids );

    return KOPENAPI_RESULT::Ok( { { "placed", placed } } );
}


KOPENAPI_REGISTER( "pcb_dimension_add",
                   "Draw a dimension (Place > Dimension): aligned (start, end, height_mm offset of the "
                   "crossbar), orthogonal (horizontal or vertical extent; orientation by the longer side "
                   "unless given), center (centre mark: start = centre, end = a corner of the cross), radial "
                   "(start = centre, end = a point on the arc), leader (arrow at start, knee at end, text "
                   "required); board dimension defaults (text, arrow, units, precision) unless given; on "
                   "Dwgs.User unless layer; answers uuid, kind, the shown text, measured_mm and box",
                   R"json({"type":"object","required":["kind","start","end"],"properties":{
                        "kind":{"type":"string","enum":["aligned","orthogonal","center","radial","leader"]},
                        "start":{"type":"array","items":{"type":"number"},"description":"[x_mm, y_mm]"},
                        "end":{"type":"array","items":{"type":"number"},"description":"[x_mm, y_mm]"},
                        "height_mm":{"type":"number","default":2,"description":"aligned / orthogonal: crossbar offset (signed)"},
                        "orientation":{"type":"string","enum":["horizontal","vertical"]},
                        "leader_length_mm":{"type":"number","description":"radial"},
                        "layer":{"type":"string","default":"Dwgs.User"},
                        "units":{"type":"string","enum":["mm","in","mils","auto"]},
                        "precision":{"type":"integer","minimum":0,"maximum":5,"description":"decimal places"},
                        "prefix":{"type":"string"},"suffix":{"type":"string"},
                        "text":{"type":"string","description":"leader text; on a measuring kind it replaces the value"},
                        "text_pos":{"type":"array","items":{"type":"number"},"description":"[x_mm, y_mm]; default the tool's spot"},
                        "text_size_mm":{"type":"number"},
                        "line_width_mm":{"type":"number"}}})json"_json,
                   false, h_pcb_dimension_add );

KOPENAPI_REGISTER( "pcb_fab_tables_add",
                   "Place fabrication documentation (Place > Board Characteristics / Stackup Table / Drill "
                   "Chart / Drill Map) in one undo step: tables right of the board outline, stacked (or at "
                   "x_mm / y_mm), on the first enabled documentation layer unless layer; a drill map draws on "
                   "the holes and needs a documentation layer without one (409 otherwise); units of the "
                   "table values; answers each item's uuid, layer, box and table size",
                   R"json({"type":"object","required":["kinds"],"properties":{
                        "kinds":{"type":"array","items":{"type":"string","enum":["characteristics","stackup","drill_chart","drill_map"]}},
                        "layer":{"type":"string","description":"a documentation layer, e.g. Cmts.User, User.1, F.Fab"},
                        "x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "units":{"type":"string","enum":["mm","in","mils"],"default":"mm"}}})json"_json,
                   false, h_pcb_fab_tables_add );

KOPENAPI_MARK_EDITING( "pcb_dimension_add", "pcb_fab_tables_add" );
