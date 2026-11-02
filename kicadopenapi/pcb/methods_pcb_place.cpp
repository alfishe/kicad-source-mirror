/// @file methods_pcb_place.cpp
/// @brief kicadopenapi placement (ROADMAP task 3.2): pcb_footprint_move, pcb_placement_check,
/// pcb_place_auto.
///
/// The agent places with pcb_footprint_move (several footprints in one undo step) and judges with
/// pcb_placement_check (courtyard overlaps, parts off the board, connectors away from the edge,
/// ratsnest length).  pcb_place_auto is a starting point: an outline sized from the parts if there
/// is none, connectors along the board edges, the rest by KiCad's own autoplacer (AR_AUTOPLACER:
/// a placement matrix inside the outline, ratsnest cost).
#include "kopenapi_pcb.h"
#include "pcb_escape_estimate.h"

#include <kicadopenapi_registry.h>
#include <netinfo.h>
#include <netclass.h>

#include <api/pcb_context.h>
#include <kicadopenapi_keepalive.h>
#include <autorouter/ar_autoplacer.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <chrono>
#include <optional>
#include <board_design_settings.h>
#include <convert_basic_shapes_to_polygon.h>
#include <connectivity/connectivity_data.h>
#include <ratsnest/ratsnest_data.h>
#include <footprint.h>
#include <geometry/shape_poly_set.h>
#include <geometry/seg.h>
#include <kicadopenapi_glow_view.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <pad.h>
#include <connectivity/connectivity_algo.h>
#include <drc/drc_engine.h>
#include <pcb_track.h>
#include <zone.h>
#include <pcb_edit_frame.h>
#include <pcb_field.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <string_utils.h>
#include <tool/actions.h>
#include <tool/tool_manager.h>
#include <view/view_overlay.h>

#include <algorithm>
#include <numeric>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <queue>
#include <set>


void refillPours( PCB_CONTEXT& aContext, BOARD* aBoard );   // methods_pcb_route.cpp


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


double toMm( double aIU )
{
    return std::round( ( aIU / pcbIUScale.IU_PER_MM ) * 1000.0 ) / 1000.0;
}


/// @brief Area in mm2 from internal units squared
double areaMm2( double aIU2 )
{
    return aIU2 / ( pcbIUScale.IU_PER_MM * pcbIUScale.IU_PER_MM );
}


/// @brief Board editor glow (no frame headless)
struct PLACE_GLOW_TRAITS
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


void glow( KOPENAPI_CONTEXT& aCtx, const std::vector<KIID>& aItems )
{
    if( !aCtx.headless )
        KopenapiGlow<PLACE_GLOW_TRAITS>( aCtx.kiway, aItems, aItems.size() > 1 ? 300 : 0 );
}


FOOTPRINT* findFootprint( BOARD* aBoard, const std::string& aRef )
{
    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( str( fp->GetReference() ) == aRef )
            return fp;
    }

    return nullptr;
}


/// @brief Courtyard of a footprint on its side (its body box when it has none)
SHAPE_POLY_SET courtyard( FOOTPRINT* aFootprint )
{
    const PCB_LAYER_ID   layer = aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd;
    const SHAPE_POLY_SET& cy = aFootprint->GetCourtyard( layer );

    if( cy.OutlineCount() > 0 )
        return cy;

    SHAPE_POLY_SET box;
    BOX2I          bbox = aFootprint->GetBoundingBox( false );
    box.NewOutline();
    box.Append( bbox.GetLeft(), bbox.GetTop() );
    box.Append( bbox.GetRight(), bbox.GetTop() );
    box.Append( bbox.GetRight(), bbox.GetBottom() );
    box.Append( bbox.GetLeft(), bbox.GetBottom() );
    return box;
}


bool isConnector( FOOTPRINT* aFootprint )
{
    const std::string ref = str( aFootprint->GetReference() );
    const std::string lib = str( aFootprint->GetFPID().GetLibNickname() );

    return ( !ref.empty() && ( ref[0] == 'J' || ref[0] == 'P' ) && ref.size() > 1 && std::isdigit( (unsigned char) ref[1] ) )
           || lib.rfind( "Connector", 0 ) == 0 || lib.rfind( "TerminalBlock", 0 ) == 0;
}


/// @brief Room under a footprint for lower parts on its side (a daughterboard over its headers):
/// its field max_height_under_mm
std::optional<double> roomUnder( const FOOTPRINT* aFootprint )
{
    double value = 0.0;

    if( const PCB_FIELD* field = aFootprint->GetField( wxS( "max_height_under_mm" ) );
        field && field->GetText().ToCDouble( &value ) && value > 0 )
    {
        return value;
    }

    return std::nullopt;
}


/// @brief What of a raised footprint is in the way of a part under it: its pads with the header body
/// around them (1 mm)
SHAPE_POLY_SET raisedArea( FOOTPRINT* aFootprint )
{
    SHAPE_POLY_SET area;

    for( PAD* pad : aFootprint->Pads() )
        pad->TransformShapeToPolygon( area, pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu, pcbIUScale.mmToIU( 1.0 ), ARC_HIGH_DEF,
                                      ERROR_OUTSIDE );

    area.Simplify();
    return area;
}


/// @brief Heights (3D models) of aParts, only when some footprint of the board has room under it
std::map<const FOOTPRINT*, double> heightsIfRoomUnder( BOARD* aBoard, const std::vector<FOOTPRINT*>& aParts )
{
    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( roomUnder( fp ) )
        {
            const std::set<const FOOTPRINT*> only( aParts.begin(), aParts.end() );
            return KopenapiFootprintHeights( aBoard, &only );
        }
    }

    return {};
}


/// @brief Nets that are rails by role, whatever their pad count: ground / power by name, or with a
/// power pin of a part on them
std::set<int> railNets( BOARD* aBoard )
{
    std::set<int> rails;

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
    {
        if( net->GetNetCode() <= 0 )
            continue;

        const std::string role = KopenapiNetRoleFromName( str( UnescapeString( net->GetNetname() ) ) ).value( "role", "" );

        if( role == "ground" || role == "power" )
            rails.insert( net->GetNetCode() );
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNetCode() > 0 && pad->GetPinType().StartsWith( wxS( "power" ) ) )
                rails.insert( pad->GetNetCode() );
        }
    }

    return rails;
}


/// @brief Groups of a footprint's pads that share a net and sit at most two pitches apart (supply
/// pins 59 / 60 take one capacitor): pad -> group id
std::map<PAD*, int> padGroups( FOOTPRINT* aFootprint )
{
    std::vector<PAD*> pads;

    for( PAD* pad : aFootprint->Pads() )
    {
        if( pad->GetNetCode() > 0 )
            pads.push_back( pad );
    }

    double pitch = std::numeric_limits<double>::max();

    for( size_t i = 0; i < pads.size(); ++i )
    {
        for( size_t j = i + 1; j < pads.size(); ++j )
            pitch = std::min( pitch, double( ( pads[i]->GetPosition() - pads[j]->GetPosition() ).EuclideanNorm() ) );
    }

    std::vector<size_t> parent( pads.size() );
    std::iota( parent.begin(), parent.end(), 0 );

    std::function<size_t( size_t )> root = [&]( size_t i ) { return parent[i] == i ? i : parent[i] = root( parent[i] ); };

    for( size_t i = 0; i < pads.size(); ++i )
    {
        for( size_t j = i + 1; j < pads.size(); ++j )
        {
            if( pads[i]->GetNetCode() == pads[j]->GetNetCode()
                && ( pads[i]->GetPosition() - pads[j]->GetPosition() ).EuclideanNorm() <= 2.0 * pitch + 1 )
            {
                parent[root( i )] = root( j );
            }
        }
    }

    std::map<PAD*, int> groups;

    for( size_t i = 0; i < pads.size(); ++i )
        groups[pads[i]] = int( root( i ) );

    return groups;
}


/// @brief Groups per net (pads of aFootprint)
std::map<int, int> groupsPerNet( FOOTPRINT* aFootprint )
{
    std::map<int, std::set<int>> seen;

    for( const auto& [pad, group] : padGroups( aFootprint ) )
        seen[pad->GetNetCode()].insert( group );

    std::map<int, int> count;

    for( const auto& [net, groups] : seen )
        count[net] = int( groups.size() );

    return count;
}


/// @brief A capacitor's value in farads from its Value text ("100n", "0.1uF", "4u7", "10uF");
/// none when it does not read
/// @brief An inductance value (3.3uH, 10 uH, 4u7H)
bool henryValue( const FOOTPRINT* aFootprint )
{
    const std::string text = str( aFootprint->GetValue() );
    size_t            i = 0;

    while( i < text.size() && ( std::isdigit( (unsigned char) text[i] ) || text[i] == '.' || text[i] == ',' ) )
        ++i;

    if( i == 0 )
        return false;

    while( i < text.size() && ( text[i] == ' ' || std::strchr( "pnum", text[i] ) || std::isdigit( (unsigned char) text[i] ) ) )
        ++i;

    if( text.compare( i, 2, "\xC2\xB5" ) == 0 )
        i += 2;

    while( i < text.size() && std::isdigit( (unsigned char) text[i] ) )
        ++i;

    return i < text.size() && ( text[i] == 'H' || text[i] == 'h' );
}


std::optional<double> faradsOf( const FOOTPRINT* aFootprint )
{
    const std::string text = str( aFootprint->GetValue() );
    std::string       number;
    size_t            i = 0;

    for( ; i < text.size() && ( std::isdigit( (unsigned char) text[i] ) || text[i] == '.' || text[i] == ',' ); ++i )
        number += text[i] == ',' ? '.' : text[i];

    if( number.empty() || number == "." )
        return std::nullopt;

    static const std::vector<std::pair<std::string, double>> prefixes = {
        { "p", 1e-12 }, { "n", 1e-9 }, { "u", 1e-6 }, { "\xC2\xB5", 1e-6 }, { "m", 1e-3 } };
    std::optional<double> prefix;

    for( const auto& [name, value] : prefixes )
    {
        if( text.compare( i, name.size(), name ) == 0 )
        {
            prefix = value;
            i += name.size();
            break;
        }
    }

    if( !prefix )
    {
        if( i < text.size() && ( text[i] == 'F' || text[i] == 'f' ) )
            return std::atof( number.c_str() );

        return std::nullopt;
    }

    // 4u7: the prefix stands for the decimal point
    std::string fraction;

    for( ; i < text.size() && std::isdigit( (unsigned char) text[i] ); ++i )
        fraction += text[i];

    if( !fraction.empty() )
    {
        if( number.find( '.' ) != std::string::npos )
            return std::nullopt;

        number += "." + fraction;
    }

    // another unit after the prefix: 3.3uH, 10mA
    if( i < text.size() && std::isalpha( (unsigned char) text[i] ) && text[i] != 'F' && text[i] != 'f' )
        return std::nullopt;

    return std::atof( number.c_str() ) * *prefix;
}


/// @brief The board outline and its edges, for "inside the outline" tests of many positions
struct OUTLINE
{
    SHAPE_POLY_SET   polygon;
    std::vector<SEG> edges;
    bool             ok = false;

    explicit OUTLINE( BOARD* aBoard )
    {
        ok = aBoard->GetBoardPolygonOutlines( polygon, true ) && polygon.OutlineCount() > 0;

        for( auto it = polygon.CIterateSegmentsWithHoles(); ok && it; it++ )
            edges.push_back( *it );
    }

    /// @brief aArea wholly inside: no edge crosses its box and its middle is inside, or nothing of
    /// it is left after taking the board away
    bool Contains( const SHAPE_POLY_SET& aArea ) const
    {
        if( !ok )
            return true;

        const BOX2I box = aArea.BBox();
        bool        crossed = false;

        for( const SEG& seg : edges )
        {
            BOX2I segBox( seg.A, VECTOR2I( 0, 0 ) );
            segBox.Merge( seg.B );

            if( box.Intersects( segBox ) )
            {
                crossed = true;
                break;
            }
        }

        if( !crossed )
            return polygon.Contains( box.Centre() );

        SHAPE_POLY_SET out = aArea;
        out.BooleanSubtract( polygon );
        return out.IsEmpty() || out.Area() < pcbIUScale.mmToIU( 0.01 ) * (double) pcbIUScale.mmToIU( 1.0 );
    }
};


struct PANEL_EDGE;
std::optional<PANEL_EDGE> panelEdge( FOOTPRINT* aFootprint, const std::string& aMethod = "auto" );


/// @brief Edge connector facing against the board outline.
/// @return facing (left/right/up/down/angle), the line's distance to the outline edge it faces
/// (mm, + = inside the board), and whether it faces out over that edge
nlohmann::json edgeFacing( FOOTPRINT* aFootprint, const BOX2I& aEdges );


/// @brief Copper of other nets under or within clearance of these footprints' pads: tracks, vias,
/// filled zones and other footprints' pads, the clearance as DRC has it for the pair (custom rules,
/// net classes, board minimum).  aItems: the conflicting tracks / vias, for on_copper rip
nlohmann::json copperConflicts( BOARD* aBoard, const std::set<const FOOTPRINT*>& aFootprints,
                                std::vector<PCB_TRACK*>* aItems = nullptr )
{
    nlohmann::json               out = nlohmann::json::array();
    const BOARD_DESIGN_SETTINGS& ds = aBoard->GetDesignSettings();
    std::shared_ptr<DRC_ENGINE>  drc = ds.m_DRCEngine;
    const int                    widest = std::max( { ds.GetBiggestClearanceValue(), ds.m_MinClearance, 1 } );
    std::set<PCB_TRACK*>         seenItems;

    auto required = [&]( BOARD_ITEM* aA, BOARD_ITEM* aB, PCB_LAYER_ID aLayer )
    {
        int need = ds.m_MinClearance;

        if( drc )
        {
            const DRC_CONSTRAINT c = drc->EvalRules( CLEARANCE_CONSTRAINT, aA, aB, aLayer );

            if( c.GetValue().HasMin() )
                need = c.GetValue().Min();
        }

        return std::max( need, 0 );
    };

    for( const FOOTPRINT* cfp : aFootprints )
    {
        if( !cfp )
            continue;

        FOOTPRINT* fp = const_cast<FOOTPRINT*>( cfp );
        BOX2I      fpReach;

        for( PAD* pad : fp->Pads() )
            fpReach.Merge( pad->GetBoundingBox() );

        fpReach.Inflate( widest );

        std::vector<PCB_TRACK*> nearTracks;
        std::vector<FOOTPRINT*> nearFps;
        std::vector<ZONE*>      nearZones;

        for( PCB_TRACK* track : aBoard->Tracks() )
        {
            if( fpReach.Intersects( track->GetBoundingBox() ) )
                nearTracks.push_back( track );
        }

        for( FOOTPRINT* otherFp : aBoard->Footprints() )
        {
            if( otherFp != fp && fpReach.Intersects( otherFp->GetBoundingBox( false ) ) )
                nearFps.push_back( otherFp );
        }

        for( ZONE* zone : aBoard->Zones() )
        {
            if( !zone->GetIsRuleArea() && fpReach.Intersects( zone->GetBoundingBox() ) )
                nearZones.push_back( zone );
        }

        for( PAD* pad : fp->Pads() )
        {
            BOX2I reach = pad->GetBoundingBox();
            reach.Inflate( widest );

            auto add = [&]( BOARD_ITEM* aOther, const wxString& aOtherNet, const std::string& aKind, PCB_LAYER_ID aLayer,
                            int aActual, int aNeed )
            {
                nlohmann::json row = { { "ref", str( fp->GetReference() ) },
                                       { "pad", str( pad->GetNumber() ) },
                                       { "net", str( UnescapeString( pad->GetNetname() ) ) },
                                       { "other_net", str( UnescapeString( aOtherNet ) ) },
                                       { "kind", aKind },
                                       { "item", str( aOther->m_Uuid.AsString() ) },
                                       { "layer", str( aBoard->GetLayerName( aLayer ) ) },
                                       { "distance_mm", toMm( aActual ) },
                                       { "required_mm", toMm( aNeed ) } };

                if( aKind == "pad" )
                    row["other_pad"] = str( static_cast<PAD*>( aOther )->GetParentFootprint()->GetReference() ) + "."
                                       + str( static_cast<PAD*>( aOther )->GetNumber() );

                if( aKind == "zone" )
                    row["hint"] = "the fill predates the move: pcb_zones_fill refills around the pad";

                out.push_back( row );
            };

            for( PCB_LAYER_ID layer : pad->GetLayerSet().CuStack() )
            {
                std::shared_ptr<SHAPE> shape = pad->GetEffectiveShape( layer );

                auto collides = [&]( BOARD_ITEM* aOther, const SHAPE* aOtherShape, int& aActual, int& aNeed )
                {
                    aNeed = required( pad, aOther, layer );
                    aActual = 0;
                    return shape->Collide( aOtherShape, std::max( aNeed - 1, 0 ), &aActual );
                };

                for( PCB_TRACK* track : nearTracks )
                {
                    if( !track->IsOnLayer( layer ) || ( pad->GetNetCode() > 0 && track->GetNetCode() == pad->GetNetCode() )
                        || !reach.Intersects( track->GetBoundingBox() ) )
                    {
                        continue;
                    }

                    std::shared_ptr<SHAPE> other = track->GetEffectiveShape( layer );
                    int                    actual = 0, need = 0;

                    if( collides( track, other.get(), actual, need ) )
                    {
                        add( track, track->GetNetname(), track->Type() == PCB_VIA_T ? "via" : "track", layer, actual, need );

                        if( aItems && seenItems.insert( track ).second )
                            aItems->push_back( track );
                    }
                }

                for( FOOTPRINT* otherFp : nearFps )
                {
                    if( !reach.Intersects( otherFp->GetBoundingBox( false ) ) )
                        continue;

                    for( PAD* otherPad : otherFp->Pads() )
                    {
                        if( !otherPad->IsOnLayer( layer ) || ( pad->GetNetCode() > 0 && otherPad->GetNetCode() == pad->GetNetCode() )
                            || !reach.Intersects( otherPad->GetBoundingBox() ) )
                        {
                            continue;
                        }

                        std::shared_ptr<SHAPE> other = otherPad->GetEffectiveShape( layer );
                        int                    actual = 0, need = 0;

                        // a pad pair is reported once, from the moved footprint first in the board's order
                        if( aFootprints.count( otherFp ) && otherFp->m_Uuid < fp->m_Uuid )
                            continue;

                        if( collides( otherPad, other.get(), actual, need ) )
                            add( otherPad, otherPad->GetNetname(), "pad", layer, actual, need );
                    }
                }

                for( ZONE* zone : nearZones )
                {
                    if( !zone->IsOnLayer( layer ) || !zone->HasFilledPolysForLayer( layer )
                        || ( pad->GetNetCode() > 0 && zone->GetNetCode() == pad->GetNetCode() )
                        || !reach.Intersects( zone->GetBoundingBox() ) )
                    {
                        continue;
                    }

                    const std::shared_ptr<SHAPE_POLY_SET>& fill = zone->GetFilledPolysList( layer );
                    const int                              need = required( pad, zone, layer );
                    int                                    actual = 0;

                    if( fill && fill->OutlineCount() > 0 && fill->Collide( shape.get(), std::max( need - 1, 0 ), &actual ) )
                        add( zone, zone->GetNetname(), "zone", layer, actual, need );
                }
            }
        }
    }

    return out;
}


/// @brief A track / via end that sat on a pad of a footprint about to move (the copper stays where
/// it is, the pad goes)
struct ATTACHED_END
{
    PCB_TRACK*   item = nullptr;
    VECTOR2I     at;
    PCB_LAYER_ID layer = UNDEFINED_LAYER;
    std::string  pad;   ///< REF.PAD it was on
};


static bool padCovers( PAD* aPad, const VECTOR2I& aAt, PCB_LAYER_ID aLayer )
{
    if( !aPad->IsOnLayer( aLayer ) )
        return false;

    std::shared_ptr<SHAPE> shape = aPad->GetEffectiveShape( aLayer );
    return shape && shape->Collide( aAt, 0 );
}


/// @brief Ends of tracks / vias on the pads of these footprints, of the pad's net, before they move
std::vector<ATTACHED_END> attachedEnds( BOARD* aBoard, const std::set<const FOOTPRINT*>& aFootprints )
{
    std::vector<ATTACHED_END> out;

    for( const FOOTPRINT* cfp : aFootprints )
    {
        if( !cfp )
            continue;

        BOX2I reach = const_cast<FOOTPRINT*>( cfp )->GetBoundingBox( false );

        for( PCB_TRACK* track : aBoard->Tracks() )
        {
            if( !reach.Intersects( track->GetBoundingBox() ) )
                continue;

            std::vector<std::pair<VECTOR2I, PCB_LAYER_ID>> ends;

            if( track->Type() == PCB_VIA_T )
            {
                for( PCB_LAYER_ID layer : LSET::AllCuMask().Seq() )
                {
                    if( track->IsOnLayer( layer ) )
                        ends.emplace_back( track->GetStart(), layer );
                }
            }
            else
            {
                ends = { { track->GetStart(), track->GetLayer() }, { track->GetEnd(), track->GetLayer() } };
            }

            std::set<std::pair<int64_t, int64_t>> done;

            for( const auto& [at, layer] : ends )
            {
                for( PAD* pad : cfp->Pads() )
                {
                    if( pad->GetNetCode() <= 0 || pad->GetNetCode() != track->GetNetCode() || !padCovers( pad, at, layer )
                        || !done.insert( { at.x, at.y } ).second )
                    {
                        continue;
                    }

                    out.push_back( { track, at, layer, str( cfp->GetReference() ) + "." + str( pad->GetNumber() ) } );
                }
            }
        }
    }

    return out;
}


/// @brief After the move: the attached ends that now sit on a pad of another net (a short the move
/// made) and those that no pad of their net holds any more (dangling)
void attachedAfterMove( BOARD* aBoard, const std::vector<ATTACHED_END>& aEnds, nlohmann::json& aShorts, nlohmann::json& aDangling )
{
    aShorts = nlohmann::json::array();
    aDangling = nlohmann::json::array();

    for( const ATTACHED_END& e : aEnds )
    {
        PAD* other = nullptr;
        bool held = false;

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( !fp->GetBoundingBox( false ).Contains( e.at ) )
                continue;

            for( PAD* pad : fp->Pads() )
            {
                if( !padCovers( pad, e.at, e.layer ) )
                    continue;

                if( pad->GetNetCode() == e.item->GetNetCode() )
                    held = true;
                else if( !other )
                    other = pad;
            }
        }

        nlohmann::json row = { { "item", str( e.item->m_Uuid.AsString() ) },
                               { "kind", e.item->Type() == PCB_VIA_T ? "via" : e.item->Type() == PCB_ARC_T ? "arc" : "track" },
                               { "net", str( UnescapeString( e.item->GetNetname() ) ) },
                               { "layer", str( aBoard->GetLayerName( e.layer ) ) },
                               { "at_mm", { toMm( e.at.x ), toMm( e.at.y ) } },
                               { "was_on", e.pad } };

        if( other )
        {
            row["pad"] = str( other->GetParentFootprint()->GetReference() ) + "." + str( other->GetNumber() );
            row["pad_net"] = str( UnescapeString( other->GetNetname() ) );
            aShorts.push_back( row );
        }
        else if( !held )
        {
            aDangling.push_back( row );
        }
    }
}


/// @brief The conflicts found before a push (aBefore) with those after it: a push propagates nets,
/// so floating copper under a moved pad takes the pad's net and would no longer show as another
/// net's; such rows say net_changed_to
nlohmann::json mergeConflicts( BOARD* aBoard, const nlohmann::json& aBefore, const nlohmann::json& aAfter )
{
    nlohmann::json        out = nlohmann::json::array();
    std::set<std::string> seen;

    for( nlohmann::json row : aBefore )
    {
        // ripped since: gone from the board
        if( !aBoard->ResolveItem( KIID( row["item"].get<std::string>() ), true ) )
            continue;

        const std::string key = row["ref"].get<std::string>() + "." + row["pad"].get<std::string>() + "|"
                                + row["item"].get<std::string>() + "|" + row["layer"].get<std::string>();

        if( BOARD_CONNECTED_ITEM* item = dynamic_cast<BOARD_CONNECTED_ITEM*>(
                    aBoard->ResolveItem( KIID( row["item"].get<std::string>() ), true ) ) )
        {
            const std::string now = str( UnescapeString( item->GetNetname() ) );

            if( now != row["other_net"].get<std::string>() )
                row["net_changed_to"] = now;
        }

        seen.insert( key );
        out.push_back( row );
    }

    for( const nlohmann::json& row : aAfter )
    {
        const std::string key = row["ref"].get<std::string>() + "." + row["pad"].get<std::string>() + "|"
                                + row["item"].get<std::string>() + "|" + row["layer"].get<std::string>();

        if( !seen.count( key ) )
            out.push_back( row );
    }

    return out;
}


/// @brief on_copper for moved footprints, before their commit is pushed: refuse (true: the caller
/// reverts), rip (the unlocked conflicting tracks / vias removed in the same commit, listed in
/// aOut.ripped; their nets in aRippedNets), report (nothing here)
bool onCopper( BOARD* aBoard, BOARD_COMMIT& aCommit, const std::set<const FOOTPRINT*>& aFootprints,
               const std::string& aMode, nlohmann::json& aOut, std::map<int, size_t>& aRippedNets )
{
    if( aMode == "report" )
        return false;

    std::vector<PCB_TRACK*> items;
    const nlohmann::json    conflicts = copperConflicts( aBoard, aFootprints, &items );

    if( conflicts.empty() )
        return false;

    if( aMode == "refuse" )
    {
        aOut["applied"] = false;
        aOut["ok"] = false;
        aOut["copper_conflicts"] = conflicts;
        aOut["reason"] = "on_copper refuse: the moved pads would touch copper of other nets; nothing was moved";
        return true;
    }

    aBoard->BuildConnectivity();
    std::shared_ptr<CONNECTIVITY_DATA> conn = aBoard->GetConnectivity();
    conn->RecalculateRatsnest();
    nlohmann::json ripped = nlohmann::json::array();

    for( PCB_TRACK* item : items )
    {
        if( item->IsLocked() )
            continue;

        if( !aRippedNets.count( item->GetNetCode() ) )
        {
            RN_NET* rn = conn->GetRatsnestForNet( item->GetNetCode() );
            aRippedNets[item->GetNetCode()] = rn ? rn->GetEdges().size() : 0;
        }

        ripped.push_back( { { "uuid", str( item->m_Uuid.AsString() ) },
                            { "kind", item->Type() == PCB_VIA_T ? "via" : "track" },
                            { "net", str( UnescapeString( item->GetNetname() ) ) } } );
        aCommit.Remove( item );
    }

    aOut["ripped"] = ripped;
    return false;
}


/// @brief After a rip: the connections the removed copper left open, per net (airwires now less before)
nlohmann::json openedByRip( BOARD* aBoard, const std::map<int, size_t>& aBefore )
{
    nlohmann::json out = nlohmann::json::array();

    if( aBefore.empty() )
        return out;

    aBoard->BuildConnectivity();
    std::shared_ptr<CONNECTIVITY_DATA> conn = aBoard->GetConnectivity();
    conn->RecalculateRatsnest();

    for( const auto& [net, before] : aBefore )
    {
        RN_NET*        rn = conn->GetRatsnestForNet( net );
        nlohmann::json ends = nlohmann::json::array();
        const size_t   now = rn ? rn->GetEdges().size() : 0;

        if( rn )
        {
            for( const CN_EDGE& edge : rn->GetEdges() )
            {
                std::string names[2];
                int         k = 0;

                for( const std::shared_ptr<const CN_ANCHOR>& node : { edge.GetSourceNode(), edge.GetTargetNode() } )
                {
                    BOARD_CONNECTED_ITEM* parent = node ? node->Parent() : nullptr;

                    if( parent && parent->Type() == PCB_PAD_T )
                        names[k] = str( static_cast<PAD*>( parent )->GetParentFootprint()->GetReference() ) + "."
                                   + str( static_cast<PAD*>( parent )->GetNumber() );
                    else if( parent )
                        names[k] = parent->Type() == PCB_VIA_T ? "via" : "track";

                    ++k;
                }

                if( ends.size() < 20 )
                    ends.push_back( { names[0], names[1] } );
            }
        }

        NETINFO_ITEM* info = aBoard->FindNet( net );
        out.push_back( { { "net", info ? str( UnescapeString( info->GetNetname() ) ) : std::string() },
                         { "opened", (int) now - (int) before },
                         { "airwires", ends } } );
    }

    return out;
}


/// @brief Placement findings: courtyard overlaps, parts outside the board, connectors, ratsnest.
/// @param aOnly when given, overlaps / outside / connectors only for these footprints (the ones just
/// moved); the ratsnest figures are always for the whole board
nlohmann::json placementReport( BOARD* aBoard, const std::set<const FOOTPRINT*>* aOnly = nullptr )
{
    // one connectivity rebuild: airwires and open connections as every other method counts them
    const KOPENAPI_UNROUTED open = KopenapiUnrouted( aBoard );
    SHAPE_POLY_SET outline;
    const bool     hasOutline = aBoard->GetBoardPolygonOutlines( outline, true ) && outline.OutlineCount() > 0;
    const BOX2I    edges = aBoard->GetBoardEdgesBoundingBox();

    std::vector<FOOTPRINT*>     fps( aBoard->Footprints().begin(), aBoard->Footprints().end() );
    std::vector<SHAPE_POLY_SET> areas;
    std::vector<BOX2I>          boxes;

    areas.reserve( fps.size() );
    boxes.reserve( fps.size() );

    for( FOOTPRINT* fp : fps )
    {
        areas.push_back( courtyard( fp ) );
        boxes.push_back( areas.back().BBox() );
    }

    auto wanted = [&]( size_t i ) { return !aOnly || aOnly->count( fps[i] ); };

    // pairs whose boxes meet (only those can overlap), found by a sweep over the left edges and
    // checked in the board's order
    std::vector<size_t> byLeft( fps.size() );
    std::iota( byLeft.begin(), byLeft.end(), 0 );
    std::sort( byLeft.begin(), byLeft.end(),
               [&]( size_t a, size_t b ) { return boxes[a].GetLeft() < boxes[b].GetLeft(); } );

    std::vector<std::pair<size_t, size_t>> pairs;
    std::shared_ptr<DRC_ENGINE>            drc = aBoard->GetDesignSettings().m_DRCEngine;
    int                                    maxCourtyardClearance = 0;

    if( DRC_CONSTRAINT worst; drc && drc->QueryWorstConstraint( COURTYARD_CLEARANCE_CONSTRAINT, worst ) )
        maxCourtyardClearance = std::max( worst.GetValue().Min(), 0 );

    // both sides' courtyards: DRC tests the front and the back ones of every pair
    std::vector<BOX2I> reach = boxes;

    for( size_t i = 0; i < fps.size(); ++i )
    {
        for( PCB_LAYER_ID side : { F_CrtYd, B_CrtYd } )
        {
            if( fps[i]->GetCourtyard( side ).OutlineCount() )
                reach[i].Merge( fps[i]->GetCourtyard( side ).BBox() );
        }
    }

    std::sort( byLeft.begin(), byLeft.end(), [&]( size_t a, size_t b ) { return reach[a].GetLeft() < reach[b].GetLeft(); } );

    for( size_t si = 0; si < byLeft.size(); ++si )
    {
        const size_t i = byLeft[si];

        for( size_t sj = si + 1; sj < byLeft.size() && reach[byLeft[sj]].GetLeft() <= reach[i].GetRight() + maxCourtyardClearance; ++sj )
        {
            const size_t j = byLeft[sj];

            if( !( wanted( i ) || wanted( j ) ) )
                continue;

            BOX2I bi = reach[i];
            bi.Inflate( maxCourtyardClearance );

            if( !bi.Intersects( reach[j] ) )
                continue;

            pairs.emplace_back( std::min( i, j ), std::max( i, j ) );
        }
    }

    std::sort( pairs.begin(), pairs.end() );

    // a part under a raised footprint (field max_height_under_mm) that fits and keeps off its pads
    std::map<size_t, double>   rooms;
    std::set<const FOOTPRINT*> lower;

    for( const auto& [i, j] : pairs )
    {
        for( const auto& [r, o] : { std::pair( i, j ), std::pair( j, i ) } )
        {
            if( std::optional<double> room = roomUnder( fps[r] ); room && !roomUnder( fps[o] ) )
            {
                rooms[r] = *room;
                lower.insert( fps[o] );
            }
        }
    }

    const std::map<const FOOTPRINT*, double> heights = lower.empty() ? std::map<const FOOTPRINT*, double>()
                                                                     : KopenapiFootprintHeights( aBoard, &lower );
    std::map<size_t, SHAPE_POLY_SET>         raisedAreas;
    nlohmann::json                           underRaised = nlohmann::json::array();

    auto fitsUnder = [&]( size_t aRaised, size_t aOther )
    {
        if( !rooms.count( aRaised ) || fps[aRaised]->IsFlipped() != fps[aOther]->IsFlipped() )
            return false;

        auto h = heights.find( fps[aOther] );

        if( h == heights.end() || h->second > rooms[aRaised] )
            return false;

        if( !raisedAreas.count( aRaised ) )
            raisedAreas[aRaised] = raisedArea( fps[aRaised] );

        SHAPE_POLY_SET both = areas[aOther];
        both.BooleanIntersection( raisedAreas[aRaised] );

        if( !both.IsEmpty() )
            return false;

        underRaised.push_back( { { "ref", str( fps[aOther]->GetReference() ) }, { "under", str( fps[aRaised]->GetReference() ) },
                                 { "height_mm", h->second }, { "max_height_under_mm", rooms[aRaised] } } );
        return true;
    };

    nlohmann::json overlaps = nlohmann::json::array();

    for( const auto& [i, j] : pairs )
    {
        if( fitsUnder( i, j ) || fitsUnder( j, i ) )
            continue;

        // as DRC's courtyard test: per side, the two courtyards closer than the courtyard clearance
        // rule (0: touching counts); a part without courtyards by its body box, on its own side
        bool           hit = false;
        nlohmann::json row = { { "a", str( fps[i]->GetReference() ) }, { "b", str( fps[j]->GetReference() ) } };

        for( PCB_LAYER_ID side : { F_CrtYd, B_CrtYd } )
        {
            const SHAPE_POLY_SET& ca = fps[i]->GetCourtyard( side );
            const SHAPE_POLY_SET& cb = fps[j]->GetCourtyard( side );

            if( hit || !ca.OutlineCount() || !cb.OutlineCount() )
                continue;

            int clearance = 0;

            if( drc )
            {
                const DRC_CONSTRAINT c = drc->EvalRules( COURTYARD_CLEARANCE_CONSTRAINT, fps[i], fps[j], side == F_CrtYd ? F_Cu : B_Cu );

                if( c.GetSeverity() == RPT_SEVERITY_IGNORE )
                    continue;

                clearance = c.GetValue().HasMin() ? c.GetValue().Min() : 0;

                if( clearance < 0 )
                    continue;
            }

            int actual = 0;

            if( ca.Collide( &cb, clearance, &actual ) )
            {
                hit = true;
                SHAPE_POLY_SET both = ca;
                both.BooleanIntersection( cb );
                row["area_mm2"] = std::round( areaMm2( both.Area() ) * 100.0 ) / 100.0;
                row["side"] = side == F_CrtYd ? "front" : "back";

                if( clearance > 0 )
                {
                    row["clearance_mm"] = toMm( clearance );
                    row["actual_mm"] = toMm( actual );
                }
            }
        }

        const bool bare = ( !fps[i]->GetCourtyard( F_CrtYd ).OutlineCount() && !fps[i]->GetCourtyard( B_CrtYd ).OutlineCount() )
                          || ( !fps[j]->GetCourtyard( F_CrtYd ).OutlineCount() && !fps[j]->GetCourtyard( B_CrtYd ).OutlineCount() );

        if( !hit && bare && fps[i]->IsFlipped() == fps[j]->IsFlipped() )
        {
            SHAPE_POLY_SET both = areas[i];
            both.BooleanIntersection( areas[j] );

            if( both.OutlineCount() > 0 && areaMm2( both.Area() ) > 0.01 )
            {
                hit = true;
                row["area_mm2"] = std::round( areaMm2( both.Area() ) * 100.0 ) / 100.0;
                row["by"] = "body box (no courtyard)";
            }
        }

        if( hit )
            overlaps.push_back( row );
    }

    // a courtyard box no outline edge (cut-outs included) enters, with its centre inside the board,
    // lies wholly inside it: nothing to subtract
    std::vector<SEG> outlineEdges;

    for( auto it = outline.CIterateSegmentsWithHoles(); it; it++ )
        outlineEdges.push_back( *it );

    auto surelyInside = [&]( size_t i )
    {
        const BOX2I& box = boxes[i];

        const SEG sides[4] = { SEG( box.GetOrigin(), VECTOR2I( box.GetRight(), box.GetTop() ) ),
                               SEG( VECTOR2I( box.GetRight(), box.GetTop() ), box.GetEnd() ),
                               SEG( box.GetEnd(), VECTOR2I( box.GetLeft(), box.GetBottom() ) ),
                               SEG( VECTOR2I( box.GetLeft(), box.GetBottom() ), box.GetOrigin() ) };

        for( const SEG& seg : outlineEdges )
        {
            BOX2I segBox( seg.A, VECTOR2I( 0, 0 ) );
            segBox.Merge( seg.B );

            if( !box.Intersects( segBox ) )
                continue;

            if( box.Contains( seg.A ) || box.Contains( seg.B ) )
                return false;

            for( const SEG& side : sides )
            {
                if( side.Intersect( seg ) )
                    return false;
            }
        }

        return outline.Contains( box.Centre() );
    };

    nlohmann::json outside = nlohmann::json::array();
    nlohmann::json connectors = nlohmann::json::array();

    for( size_t i = 0; i < fps.size(); ++i )
    {
        if( !wanted( i ) )
            continue;

        if( hasOutline && !surelyInside( i ) )
        {
            SHAPE_POLY_SET out = areas[i];
            out.BooleanSubtract( outline );

            if( out.OutlineCount() > 0 && areaMm2( out.Area() ) > 0.01 )
                outside.push_back( str( fps[i]->GetReference() ) );
        }

        if( isConnector( fps[i] ) && edges.GetWidth() > 0 )
        {
            const BOX2I box = areas[i].BBox();
            const int   gap = std::min( { box.GetLeft() - edges.GetLeft(), edges.GetRight() - box.GetRight(),
                                          box.GetTop() - edges.GetTop(), edges.GetBottom() - box.GetBottom() } );
            nlohmann::json row = { { "ref", str( fps[i]->GetReference() ) }, { "to_edge_mm", toMm( gap ) } };
            row.update( edgeFacing( fps[i], edges ) );
            connectors.push_back( row );
        }
    }

    // edge connectors facing out over their edge overhang by design: not "outside"
    nlohmann::json problems = nlohmann::json::array();
    nlohmann::json overhanging = nlohmann::json::array();

    for( const nlohmann::json& c : connectors )
    {
        if( !c.contains( "faces_out" ) )
            continue;

        auto it = std::find( outside.begin(), outside.end(), c["ref"] );

        if( c["faces_out"].get<bool>() && std::abs( c["line_to_edge_mm"].get<double>() ) <= 1.0 )
        {
            if( it != outside.end() )
            {
                outside.erase( it );
                overhanging.push_back( c["ref"] );
            }
        }
        else if( c["faces_out"].get<bool>() )
        {
            problems.push_back( { { "ref", c["ref"] }, { "problem", "connector_not_on_edge" },
                                  { "line_to_edge_mm", c["line_to_edge_mm"] },
                                  { "hint", "pcb_footprint_move {ref, edge}" } } );
        }
        else
        {
            problems.push_back( { { "ref", c["ref"] }, { "problem", "connector_facing_inward" },
                                  { "facing", c["facing"] },
                                  { "hint", "pcb_footprint_move {ref, edge} puts it on an edge facing out" } } );
        }
    }

    // Ratsnest: what is left to route and how long the airwires are (shorter = better placement)
    // Built from scratch: after a netlist update headless the incremental data can be stale
    aBoard->BuildConnectivity();
    std::shared_ptr<CONNECTIVITY_DATA> connectivity = aBoard->GetConnectivity();
    connectivity->RecalculateRatsnest();

    double airwires = 0;
    int    edgesCount = 0;

    for( int net = 1; net < (int) aBoard->GetNetCount(); ++net )
    {
        if( RN_NET* rn = connectivity->GetRatsnestForNet( net ) )
        {
            for( const CN_EDGE& edge : rn->GetEdges() )
            {
                if( edge.GetSourceNode() && edge.GetTargetNode() )
                {
                    airwires += ( edge.GetTargetPos() - edge.GetSourcePos() ).EuclideanNorm();
                    edgesCount++;
                }
            }
        }
    }

    nlohmann::json report = { { "footprints", fps.size() },
                              { "courtyard_overlaps", overlaps },
                              { "outside_board", outside },
                              { "connectors", connectors },
                              { "connector_problems", problems },
                              { "edge_overhang", overhanging },
                              { "unrouted", open.connections },
                              { "airwire_length_mm", toMm( airwires ) },
                              { "airwires", edgesCount },
                              { "ok", overlaps.empty() && outside.empty() && problems.empty() && hasOutline } };

    // the moved parts' pads against other nets' copper (the whole board's copper: DRC)
    if( aOnly )
    {
        report["copper_conflicts"] = copperConflicts( aBoard, *aOnly );

        if( !report["copper_conflicts"].empty() )
            report["ok"] = false;
    }

    if( !underRaised.empty() )
        report["under_raised"] = underRaised;

    if( !overhanging.empty() )
        report["edge_overhang_note"] = "connectors placed with their mating side over the board edge on purpose (the "
                                       "opening at the edge, the body's front beyond it, as the footprint draws it): "
                                       "not outside_board; edge_offset_mm in pcb_footprint_move sets it back";

    report.update( KopenapiUnroutedJson( aBoard, false ) );
    report.erase( "unrouted_left" );

    if( hasOutline )
        report["board_mm"] = { toMm( edges.GetWidth() ), toMm( edges.GetHeight() ) };
    else
        report["hint"] = "no board outline: pcb_outline_set, or pcb_place_auto sizes one";

    return report;
}


void refreshGui( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard )
{
    if( aCtx.headless )
        return;

    if( auto* frame = PLACE_GLOW_TRAITS::Frame( aCtx.kiway ) )
    {
        if( frame->GetBoard() == aBoard )
        {
            frame->GetToolManager()->RunAction( ACTIONS::selectionClear );
            frame->GetCanvas()->Refresh();
        }
    }
}

/// @brief The panel / board edge line of an edge connector (first) with its outward normal (second)
struct PANEL_EDGE
{
    SEG         first;
    VECTOR2D    second;
    std::string source;       ///< marker convention, or "inferred"
    std::string confidence;   ///< "marked", or high / medium / low for inferred edges
};


/// @brief Front of an unmarked horizontal connector: the side where the courtyard reaches
/// furthest beyond the signal pads (the mating body sticks out of the soldered area).
/// @return the edge flush with the courtyard front, or nullopt for vertical / undecidable parts
std::optional<PANEL_EDGE> inferEdge( FOOTPRINT* aFootprint )
{
    const wxString name = aFootprint->GetFPID().GetLibItemName().wx_str().Lower();

    if( name.Contains( wxS( "vertical" ) ) || name.Contains( wxS( "topentry" ) ) || aFootprint->Pads().empty() )
        return std::nullopt;

    const VECTOR2I   origin = aFootprint->GetPosition();
    const EDA_ANGLE  orient = aFootprint->GetOrientation();
    auto local = [&]( VECTOR2I aPt )
    {
        VECTOR2I p = aPt - origin;
        RotatePoint( p, -orient );
        return p;
    };
    auto world = [&]( VECTOR2I aPt )
    {
        RotatePoint( aPt, orient );
        return aPt + origin;
    };

    BOX2I pads;
    bool  any = false;

    for( PAD* pad : aFootprint->Pads() )
    {
        if( pad->GetAttribute() == PAD_ATTRIB::NPTH || pad->GetNumber().IsEmpty() )
            continue;

        const VECTOR2I p = local( pad->GetPosition() );

        if( !any )
            pads = BOX2I( p, VECTOR2I( 0, 0 ) ), any = true;
        else
            pads.Merge( p );
    }

    const SHAPE_POLY_SET& crt = aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd );

    if( !any || crt.OutlineCount() == 0 )
        return std::nullopt;

    BOX2I body;
    bool  first = true;

    for( auto it = crt.CIterate(); it; ++it )
    {
        const VECTOR2I p = local( *it );

        if( first )
            body = BOX2I( p, VECTOR2I( 0, 0 ) ), first = false;
        else
            body.Merge( p );
    }

    // reach beyond the pads: right, down, left, up (local frame)
    const double reach[4] = { (double) body.GetRight() - pads.GetRight(), (double) body.GetBottom() - pads.GetBottom(),
                              (double) pads.GetLeft() - body.GetLeft(), (double) pads.GetTop() - body.GetTop() };
    int best = 0, second = -1;

    for( int k = 1; k < 4; ++k )
    {
        if( reach[k] > reach[best] )
            best = k;
    }

    for( int k = 0; k < 4; ++k )
    {
        if( k != best && ( second < 0 || reach[k] > reach[second] ) )
            second = k;
    }

    const double mm = pcbIUScale.IU_PER_MM;

    if( reach[best] < 3.0 * mm )
        return std::nullopt;

    const double ratio = reach[best] / std::max( reach[second], 0.5 * mm );
    std::string  confidence = ratio >= 2.5 ? "high" : ratio >= 1.5 ? "medium" : "low";

    VECTOR2I a, b;
    VECTOR2D n;

    switch( best )
    {
    case 0: a = { body.GetRight(), body.GetTop() };    b = { body.GetRight(), body.GetBottom() }; n = { 1, 0 };  break;
    case 1: a = { body.GetLeft(), body.GetBottom() };  b = { body.GetRight(), body.GetBottom() }; n = { 0, 1 };  break;
    case 2: a = { body.GetLeft(), body.GetTop() };     b = { body.GetLeft(), body.GetBottom() };  n = { -1, 0 }; break;
    default: a = { body.GetLeft(), body.GetTop() };    b = { body.GetRight(), body.GetTop() };    n = { 0, -1 }; break;
    }

    const VECTOR2I tip = a + VECTOR2I( KiROUND( n.x * 1000000 ), KiROUND( n.y * 1000000 ) );
    const VECTOR2D nw = VECTOR2D( world( tip ) - world( a ) ).Resize( 1.0 );

    return PANEL_EDGE{ SEG( world( a ), world( b ) ), nw, "inferred", confidence };
}


/// @brief The panel edge a connector footprint marks for its front, and the direction from the pads
/// towards it (where the connector faces).  Marked by a line on Dwgs.User, or by a "PCB Edge" text
/// next to a line on another layer: longest Dwgs.User segment first, else the one nearest the text.
std::optional<PANEL_EDGE> markedEdge( FOOTPRINT* aFootprint )
{
    std::optional<SEG>      best;
    std::string             source;
    std::vector<VECTOR2I>   marks;
    std::vector<SEG>        others;

    for( BOARD_ITEM* item : aFootprint->GraphicalItems() )
    {
        if( item->Type() == PCB_TEXT_T )
        {
            const wxString text = static_cast<PCB_TEXT*>( item )->GetText().Lower();

            if( text.Contains( wxS( "edge" ) ) && ( text.Contains( wxS( "pcb" ) ) || text.Contains( wxS( "board" ) ) ) )
                marks.push_back( item->GetPosition() );

            continue;
        }

        if( item->Type() != PCB_SHAPE_T || static_cast<PCB_SHAPE*>( item )->GetShape() != SHAPE_T::SEGMENT )
            continue;

        PCB_SHAPE* shape = static_cast<PCB_SHAPE*>( item );
        SEG        seg( shape->GetStart(), shape->GetEnd() );

        if( item->GetLayer() == Dwgs_User )
        {
            if( !best || seg.Length() > best->Length() )
                best = seg, source = "Dwgs.User line";
        }
        else if( !IsCopperLayer( item->GetLayer() ) && seg.Length() >= pcbIUScale.mmToIU( 2.0 ) )
        {
            others.push_back( seg );
        }
    }

    if( !best && !marks.empty() )
    {
        // nearest long segment to the mark; among equally near ones (within 0.5 mm) the longest
        double nearest = std::numeric_limits<double>::max();

        for( const SEG& seg : others )
            nearest = std::min( nearest, (double) seg.Distance( marks.front() ) );

        for( const SEG& seg : others )
        {
            if( seg.Distance( marks.front() ) <= nearest + pcbIUScale.mmToIU( 0.5 )
                && seg.Distance( marks.front() ) <= pcbIUScale.mmToIU( 3.0 )
                && ( !best || seg.Length() > best->Length() ) )
            {
                best = seg, source = "\"PCB Edge\" text";
            }
        }
    }

    if( !best || aFootprint->Pads().empty() )
        return std::nullopt;

    // a panel line spans the body; short Dwgs.User strokes are drawings (arrows, notes)
    const BOX2I body = aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd ).OutlineCount()
                               ? aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd ).BBox()
                               : aFootprint->GetBoundingBox( false );
    const VECTOR2D dir = VECTOR2D( best->B - best->A ).Resize( 1.0 );
    const double   span = std::abs( dir.x ) * body.GetWidth() + std::abs( dir.y ) * body.GetHeight();

    if( best->Length() < 0.5 * span )
        return std::nullopt;

    VECTOR2D centroid( 0, 0 );

    for( PAD* pad : aFootprint->Pads() )
        centroid += VECTOR2D( pad->GetPosition() );

    centroid = centroid * ( 1.0 / aFootprint->Pads().size() );

    const VECTOR2D d( best->B - best->A );
    VECTOR2D       n( -d.y, d.x );
    n = n.Resize( 1.0 );

    if( ( VECTOR2D( best->Center() ) - centroid ).Dot( n ) < 0 )
        n = -n;

    return PANEL_EDGE{ *best, n, source, "marked" };
}


/// @brief The board edge an edge-finger connector (ISA / PCI card edge) sits on: four or more pads
/// without holes, most of them within 0.5 mm of one straight line of the board outline.  The line
/// spans the fingers; its normal points out of the board.
std::optional<PANEL_EDGE> fingerEdge( FOOTPRINT* aFootprint )
{
    BOARD*            board = aFootprint->GetBoard();
    std::vector<PAD*> fingers;

    for( PAD* pad : aFootprint->Pads() )
    {
        if( !pad->HasHole() && !pad->GetNumber().IsEmpty() )
            fingers.push_back( pad );
    }

    SHAPE_POLY_SET outline;

    if( !board || fingers.size() < 4 || !board->GetBoardPolygonOutlines( outline, true ) || outline.OutlineCount() == 0 )
        return std::nullopt;

    const int   touch = pcbIUScale.mmToIU( 0.5 );
    const BOX2I span = aFootprint->GetBoundingBox( false );
    const int   reach = span.GetWidth() + span.GetHeight();
    SEG         best;
    size_t      bestCount = 0;

    for( auto it = outline.CIterateSegmentsWithHoles(); it; it++ )
    {
        const SEG seg = *it;

        if( seg.Length() < pcbIUScale.mmToIU( 2.0 ) || seg.Distance( span.Centre() ) > reach )
            continue;

        // the whole straight line: a card edge cut by a notch is two segments of one line
        const VECTOR2I along = ( seg.B - seg.A ).Resize( reach );
        const SEG      line( seg.A - along, seg.B + along );
        size_t         count = 0;

        for( PAD* pad : fingers )
        {
            if( pad->GetEffectiveShape( pad->GetPrincipalLayer() )->Collide( line, touch ) )
                count++;
        }

        if( count > bestCount )
        {
            bestCount = count;
            best = line;
        }
    }

    if( bestCount < 4 || bestCount * 10 < fingers.size() * 6 )
        return std::nullopt;

    // the line over the fingers' extent
    const VECTOR2D dir = VECTOR2D( best.B - best.A ).Resize( 1.0 );
    double         lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();

    for( PAD* pad : fingers )
    {
        const BOX2I box = pad->GetBoundingBox();

        for( const VECTOR2I& corner : { box.GetOrigin(), box.GetEnd() } )
        {
            const double t = VECTOR2D( best.LineProject( corner ) - best.A ).Dot( dir );
            lo = std::min( lo, t );
            hi = std::max( hi, t );
        }
    }

    const VECTOR2I a = best.A + VECTOR2I( KiROUND( dir.x * lo ), KiROUND( dir.y * lo ) );
    const VECTOR2I b = best.A + VECTOR2I( KiROUND( dir.x * hi ), KiROUND( dir.y * hi ) );
    VECTOR2D       n( -dir.y, dir.x );
    const VECTOR2I probe = SEG( a, b ).Center() + VECTOR2I( KiROUND( n.x * touch ), KiROUND( n.y * touch ) );

    if( outline.Contains( probe ) )
        n = -n;

    return PANEL_EDGE{ SEG( a, b ), n, "board edge (fingers)", "high" };
}


/// @param aMethod auto (edge fingers on the outline, else marker, else inference), marker, infer
std::optional<PANEL_EDGE> panelEdge( FOOTPRINT* aFootprint, const std::string& aMethod )
{
    if( aMethod != "marker" )
    {
        if( std::optional<PANEL_EDGE> fingers = fingerEdge( aFootprint ) )
            return fingers;
    }

    if( aMethod != "infer" )
    {
        if( std::optional<PANEL_EDGE> marked = markedEdge( aFootprint ) )
            return marked;
    }

    return aMethod == "marker" ? std::nullopt : inferEdge( aFootprint );
}

} // namespace


nlohmann::json KopenapiPanelEdgeJson( FOOTPRINT* aFootprint, const std::string& aMethod )
{
    const std::optional<PANEL_EDGE> edge = panelEdge( aFootprint, aMethod );

    if( !edge )
        return nullptr;

    const auto mmv = []( double aIU ) { return std::round( pcbIUScale.IUTomm( 1 ) * aIU * 1000.0 ) / 1000.0; };
    const VECTOR2D n = edge->second;
    const VECTOR2D c( edge->first.Center() );

    // depth behind the line (into the board) and overhang past it: courtyard, else bounding box
    double         behind = 0, past = 0;
    SHAPE_POLY_SET body = aFootprint->GetCourtyard( aFootprint->IsFlipped() ? B_CrtYd : F_CrtYd );

    if( body.OutlineCount() == 0 )
    {
        const BOX2I b = aFootprint->GetBoundingBox( false );
        body.NewOutline();
        for( VECTOR2I p : { b.GetOrigin(), VECTOR2I( b.GetRight(), b.GetTop() ), b.GetEnd(), VECTOR2I( b.GetLeft(), b.GetBottom() ) } )
            body.Append( p );
    }

    for( auto it = body.CIterate(); it; ++it )
    {
        const double d = ( VECTOR2D( *it ) - c ).Dot( n );
        behind = std::max( behind, -d );
        past = std::max( past, d );
    }

    const double deg = std::fmod( std::atan2( n.y, n.x ) * 180.0 / M_PI + 360.0, 360.0 );
    nlohmann::json facing = std::round( deg * 10 ) / 10;

    for( const auto& [name, a] : std::vector<std::pair<const char*, double>>{ { "right", 0 }, { "down", 90 }, { "left", 180 }, { "up", 270 }, { "right", 360 } } )
    {
        if( std::abs( deg - a ) < 1.0 )
            facing = name;
    }

    return { { "line_mm", { { mmv( edge->first.A.x ), mmv( edge->first.A.y ) }, { mmv( edge->first.B.x ), mmv( edge->first.B.y ) } } },
             { "facing", facing },
             { "depth_mm", mmv( behind ) },
             { "overhang_mm", mmv( past ) },
             { "source", edge->source },
             { "confidence", edge->confidence } };
}


namespace
{

nlohmann::json edgeFacing( FOOTPRINT* aFootprint, const BOX2I& aEdges )
{
    const std::optional<PANEL_EDGE> edge = panelEdge( aFootprint, "auto" );

    if( !edge )
        return nlohmann::json::object();

    const VECTOR2D n = edge->second;
    const VECTOR2I c = edge->first.Center();

    // the outline edge the connector faces and the line's distance to it (inside = positive)
    double      dist = 0;
    std::string facing;

    if( std::abs( n.x ) >= std::abs( n.y ) )
        facing = n.x > 0 ? "right" : "left", dist = n.x > 0 ? aEdges.GetRight() - c.x : c.x - aEdges.GetLeft();
    else
        facing = n.y > 0 ? "down" : "up", dist = n.y > 0 ? aEdges.GetBottom() - c.y : c.y - aEdges.GetTop();

    // facing out = the faced edge is the nearest outline edge to the line
    const double nearest = std::min( { std::abs( (double) c.x - aEdges.GetLeft() ), std::abs( (double) aEdges.GetRight() - c.x ),
                                       std::abs( (double) c.y - aEdges.GetTop() ), std::abs( (double) aEdges.GetBottom() - c.y ) } );

    return { { "facing", facing },
             { "line_to_edge_mm", std::round( dist / pcbIUScale.IU_PER_MM * 100.0 ) / 100.0 },
             { "faces_out", std::abs( dist ) <= nearest + pcbIUScale.mmToIU( 0.01 ) },
             { "edge_source", edge->source },
             { "edge_confidence", edge->confidence } };
}

} // namespace


namespace
{

/// @brief The way of a differential pair or a clock net between two parts it joins, and the straight
/// escape of their pins there: parts of a block keep out of it
struct CORRIDOR
{
    std::set<int>  nets;
    std::string    name;    ///< "USB_D+/USB_D-"
    SHAPE_POLY_SET area;
    BOX2I          box;
};


/// @brief Shortest way from aFrom to aTo on a 0.25 mm grid around the parts in aWalls (courtyards
/// grown by aHalf), inside aBounds; the corner points, or the straight line when there is none
std::vector<VECTOR2I> wayAround( const VECTOR2I& aFrom, const VECTOR2I& aTo, const std::vector<SHAPE_POLY_SET>& aWalls,
                                 BOX2I aBounds )
{
    const int step = toIU( 0.25 );
    const int cols = aBounds.GetWidth() / step + 1;
    const int rows = aBounds.GetHeight() / step + 1;

    if( cols <= 1 || rows <= 1 || (long long) cols * rows > 4000000 )
        return { aFrom, aTo };

    auto cellOf = [&]( const VECTOR2I& aAt )
    {
        const int c = std::clamp( ( aAt.x - aBounds.GetLeft() + step / 2 ) / step, 0, cols - 1 );
        const int r = std::clamp( ( aAt.y - aBounds.GetTop() + step / 2 ) / step, 0, rows - 1 );
        return r * cols + c;
    };

    auto pointOf = [&]( int aCell ) { return VECTOR2I( aBounds.GetLeft() + ( aCell % cols ) * step, aBounds.GetTop() + ( aCell / cols ) * step ); };

    const int         start = cellOf( aFrom );
    const int         goal = cellOf( aTo );
    std::vector<char> wall( (size_t) cols * rows, 0 );

    for( const SHAPE_POLY_SET& w : aWalls )
    {
        const BOX2I box = w.BBox();

        for( int r = std::max( 0, ( box.GetTop() - aBounds.GetTop() ) / step ); r < rows && aBounds.GetTop() + r * step <= box.GetBottom(); ++r )
        {
            for( int c = std::max( 0, ( box.GetLeft() - aBounds.GetLeft() ) / step ); c < cols && aBounds.GetLeft() + c * step <= box.GetRight(); ++c )
            {
                if( !wall[r * cols + c] && w.Contains( pointOf( r * cols + c ) ) )
                    wall[r * cols + c] = 1;
            }
        }
    }

    wall[start] = wall[goal] = 0;

    std::vector<double> cost( wall.size(), std::numeric_limits<double>::max() );
    std::vector<int>    from( wall.size(), -1 );
    using ENTRY = std::pair<double, int>;
    std::priority_queue<ENTRY, std::vector<ENTRY>, std::greater<ENTRY>> open;

    cost[start] = 0;
    open.emplace( 0, start );

    while( !open.empty() )
    {
        const auto [c, cell] = open.top();
        open.pop();

        if( cell == goal )
            break;

        if( c > cost[cell] )
            continue;

        for( int dr = -1; dr <= 1; ++dr )
        {
            for( int dc = -1; dc <= 1; ++dc )
            {
                const int r = cell / cols + dr, col = cell % cols + dc;

                if( ( !dr && !dc ) || r < 0 || r >= rows || col < 0 || col >= cols || wall[r * cols + col] )
                    continue;

                const int    next = r * cols + col;
                const double nc = c + ( dr && dc ? M_SQRT2 : 1.0 );

                if( nc < cost[next] )
                {
                    cost[next] = nc;
                    from[next] = cell;
                    open.emplace( nc, next );
                }
            }
        }
    }

    if( from[goal] < 0 )
        return { aFrom, aTo };

    std::vector<int> cells;

    for( int cell = goal; cell >= 0; cell = from[cell] )
        cells.push_back( cell );

    std::reverse( cells.begin(), cells.end() );

    std::vector<VECTOR2I> way = { aFrom };

    for( size_t i = 1; i + 1 < cells.size(); ++i )
    {
        const int d1 = cells[i] - cells[i - 1], d2 = cells[i + 1] - cells[i];

        if( d1 != d2 )
            way.push_back( pointOf( cells[i] ) );
    }

    way.push_back( aTo );
    return way;
}


/// @brief Corridors of the board's pairs and clock nets (pcb_net_roles): between every two parts
/// (more than two pads) that carry all nets of the group, a band as wide as the pair (or track)
/// with its clearance along the shortest way around the parts that stay (more than two pads, not
/// in aMoving), plus each pin's straight escape aEscapeMm long.  Groups joining more than four
/// parts get none.
std::vector<CORRIDOR> signalCorridors( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, double aEscapeMm,
                                       const std::vector<FOOTPRINT*>& aMoving )
{
    std::vector<std::vector<std::string>> groups;
    std::optional<KOPENAPI_METHOD>        roles = KOPENAPI_REGISTRY::Get().Find( "pcb_net_roles" );

    if( !roles )
        return {};

    const KOPENAPI_RESULT r = roles->handler( aCtx, nlohmann::json::object() );

    if( r.status >= 400 )
        return {};

    for( const nlohmann::json& pr : r.body.value( "pairs", nlohmann::json::array() ) )
    {
        if( pr.is_object() && pr.value( "p", nlohmann::json() ).is_string() && pr.value( "n", nlohmann::json() ).is_string() )
            groups.push_back( { pr["p"].get<std::string>(), pr["n"].get<std::string>() } );
    }

    for( const nlohmann::json& row : r.body.value( "nets", nlohmann::json::array() ) )
    {
        if( row.is_object() && row.value( "role", std::string() ) == "clock" && row.value( "name", nlohmann::json() ).is_string() )
            groups.push_back( { row["name"].get<std::string>() } );
    }

    std::vector<CORRIDOR> out;
    const int             maxError = aBoard->GetDesignSettings().m_MaxError;
    const int             escape = toIU( aEscapeMm );

    for( const std::vector<std::string>& group : groups )
    {
        CORRIDOR      corridor;
        NETINFO_ITEM* first = nullptr;

        for( const std::string& name : group )
        {
            NETINFO_ITEM* net = KopenapiFindNet( aBoard, name );

            if( !net )
                break;

            first = first ? first : net;
            corridor.nets.insert( net->GetNetCode() );
            corridor.name += ( corridor.name.empty() ? "" : "/" ) + name;
        }

        if( !first || corridor.nets.size() != group.size() )
            continue;

        std::vector<std::vector<PAD*>> ends;

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( fp->Pads().size() <= 2 )
                continue;

            std::vector<PAD*> pads;
            std::set<int>     nets;

            for( PAD* pad : fp->Pads() )
            {
                if( corridor.nets.count( pad->GetNetCode() ) )
                {
                    pads.push_back( pad );
                    nets.insert( pad->GetNetCode() );
                }
            }

            if( nets == corridor.nets )
                ends.push_back( pads );
        }

        if( ends.size() < 2 || ends.size() > 4 )
            continue;

        NETCLASS* netclass = first->GetNetClass();
        const int width = netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth() : toIU( 0.2 );
        const int clearance = netclass && netclass->HasClearance() ? netclass->GetClearance() : toIU( 0.2 );
        const int gap = netclass && netclass->HasDiffPairGap() ? netclass->GetDiffPairGap() : clearance;
        const int band = group.size() == 2 ? 2 * width + gap + 2 * clearance : width + 2 * clearance;

        std::vector<SHAPE_POLY_SET> walls;
        BOX2I                       bounds = aBoard->GetBoardEdgesBoundingBox();

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            const bool end = std::any_of( ends.begin(), ends.end(),
                                          [&]( const std::vector<PAD*>& e ) { return e.front()->GetParentFootprint() == fp; } );

            if( end || fp->Pads().size() <= 2 || std::find( aMoving.begin(), aMoving.end(), fp ) != aMoving.end() )
                continue;

            SHAPE_POLY_SET wall = courtyard( fp );
            wall.Inflate( band / 2, CORNER_STRATEGY::ALLOW_ACUTE_CORNERS, maxError );
            walls.push_back( std::move( wall ) );
        }

        auto centre = []( const std::vector<PAD*>& aPads )
        {
            VECTOR2L sum( 0, 0 );

            for( PAD* pad : aPads )
                sum += VECTOR2L( pad->GetPosition() );

            return VECTOR2I( sum.x / (long long) aPads.size(), sum.y / (long long) aPads.size() );
        };

        for( size_t a = 0; a < ends.size(); ++a )
        {
            for( size_t b = a + 1; b < ends.size(); ++b )
            {
                const std::vector<VECTOR2I> way = wayAround( centre( ends[a] ), centre( ends[b] ), walls, bounds );

                for( size_t k = 1; k < way.size(); ++k )
                    TransformOvalToPolygon( corridor.area, way[k - 1], way[k], band, maxError, ERROR_OUTSIDE );
            }

            // each pin's escape: straight out of its part, along the axis the pin sits on
            for( PAD* pad : ends[a] )
            {
                const VECTOR2I off = pad->GetPosition() - pad->GetParentFootprint()->GetPosition();
                const VECTOR2I dir = std::abs( off.x ) >= std::abs( off.y ) ? VECTOR2I( off.x >= 0 ? 1 : -1, 0 )
                                                                            : VECTOR2I( 0, off.y >= 0 ? 1 : -1 );
                const BOX2I    pb = pad->GetBoundingBox();
                const int      half = dir.x ? pb.GetWidth() / 2 : pb.GetHeight() / 2;

                TransformOvalToPolygon( corridor.area, pad->GetPosition(), pad->GetPosition() + dir * ( half + escape ),
                                        width + 2 * clearance, maxError, ERROR_OUTSIDE );
            }
        }

        corridor.area.Simplify();
        corridor.box = corridor.area.BBox();
        out.push_back( std::move( corridor ) );
    }

    return out;
}


/// @brief The anchor's port sides from the layout plan (pcb_fanout_ports): per side, the pads whose nets go
/// to another block, as a band from the pads out past the courtyard by aKeepMm; the block's parts keep out
/// of it, so the signals leave the block without weaving between them
/// @brief ports_keep_mm, 3 when only open_sides is given
double portsKeep( const nlohmann::json& aSpec )
{
    return aSpec.value( "ports_keep_mm", aSpec.contains( "open_sides" ) ? 3.0 : 0.0 );
}


std::optional<std::string> openSidesError( const nlohmann::json& aSpec )
{
    if( !aSpec.contains( "open_sides" ) )
        return std::nullopt;

    const nlohmann::json& open = aSpec["open_sides"];

    if( open.is_number_integer() && open.get<int>() >= 0 && open.get<int>() <= 4 )
        return std::nullopt;

    if( open.is_array() )
    {
        for( const nlohmann::json& name : open )
        {
            if( !name.is_string() || ( name != "west" && name != "east" && name != "north" && name != "south" ) )
                return "open_sides: names are west, east, north, south";
        }

        return std::nullopt;
    }

    return "open_sides: a count 0..4 (the sides with the most ports first) or a list of west / east / north / south";
}


/// @param aOpen which sides: null - every side with ports; a count - that many, the most ports first;
/// a list of names - those (a named side with no ports: its whole courtyard edge)
std::vector<CORRIDOR> portCorridors( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, FOOTPRINT* aAnchor, double aKeepMm,
                                     const nlohmann::json& aOpen = nullptr )
{
    std::optional<KOPENAPI_METHOD> ports = KOPENAPI_REGISTRY::Get().Find( "pcb_fanout_ports" );

    if( !ports || !aAnchor || aKeepMm <= 0 )
        return {};

    const KOPENAPI_RESULT r = ports->handler( aCtx, { { "ref", str( aAnchor->GetReference() ) } } );

    if( r.status >= 400 )
        return {};

    std::map<wxString, PAD*> padOf;

    for( PAD* pad : aAnchor->Pads() )
        padOf.emplace( pad->GetNumber(), pad );

    const BOX2I           cy = courtyard( aAnchor ).BBox();
    const int             keep = toIU( aKeepMm );
    std::vector<CORRIDOR> out;

    const nlohmann::json  sides = r.body.value( "ports", nlohmann::json::object() );
    static const char*    names[] = { "west", "east", "north", "south" };
    std::vector<std::pair<std::string, int>> ranked;   // side, signal ports

    for( const char* name : names )
    {
        int count = 0;

        if( sides.contains( name ) && sides[name].is_array() )
        {
            for( const nlohmann::json& port : sides[name] )
                count += port.value( "kind", std::string() ) == "signal" ? 1 : 0;
        }

        ranked.emplace_back( name, count );
    }

    std::stable_sort( ranked.begin(), ranked.end(), []( const auto& a, const auto& b ) { return a.second > b.second; } );

    std::vector<std::string> chosen;

    if( aOpen.is_array() )
    {
        for( const nlohmann::json& name : aOpen )
        {
            if( name.is_string() )
                chosen.push_back( name.get<std::string>() );
        }
    }
    else if( aOpen.is_number_integer() )
    {
        const size_t count = size_t( std::clamp( aOpen.get<int>(), 0, 4 ) );

        for( const auto& [name, ports] : ranked )
        {
            if( ports > 0 && chosen.size() < count )
                chosen.push_back( name );
        }
    }
    else
    {
        for( const auto& item : sides.items() )
            chosen.push_back( item.key() );
    }

    for( const std::string& side : chosen )
    {
        if( side != "west" && side != "east" && side != "north" && side != "south" )
            continue;

        CORRIDOR              corridor;
        std::optional<BOX2I>  span;
        std::set<std::string> to;
        const nlohmann::json  list = sides.contains( side ) && sides[side].is_array() ? sides[side] : nlohmann::json::array();

        for( const nlohmann::json& port : list )
        {
            if( port.value( "kind", std::string() ) != "signal" )
                continue;

            auto it = padOf.find( wxString::FromUTF8( port.value( "pad", std::string() ) ) );

            if( it == padOf.end() )
                continue;

            corridor.nets.insert( it->second->GetNetCode() );
            to.insert( port.value( "to", std::string() ) );

            if( span )
                span->Merge( it->second->GetBoundingBox() );
            else
                span = it->second->GetBoundingBox();
        }

        if( ( !span || corridor.nets.size() < 2 ) && !aOpen.is_array() )
            continue;

        if( !span || corridor.nets.size() < 2 )
        {
            corridor.nets.clear();
            to.clear();
            span = cy;

            if( side == "west" )
                span->SetWidth( 0 );
            else if( side == "east" )
                span->SetX( cy.GetRight() ), span->SetWidth( 0 );
            else if( side == "north" )
                span->SetHeight( 0 );
            else
                span->SetY( cy.GetBottom() ), span->SetHeight( 0 );
        }

        BOX2I band = *span;

        if( side == "west" )
            band.SetX( cy.GetLeft() - keep ), band.SetWidth( span->GetRight() - ( cy.GetLeft() - keep ) );
        else if( side == "east" )
            band.SetWidth( cy.GetRight() + keep - span->GetLeft() );
        else if( side == "north" )
            band.SetY( cy.GetTop() - keep ), band.SetHeight( span->GetBottom() - ( cy.GetTop() - keep ) );
        else
            band.SetHeight( cy.GetBottom() + keep - span->GetTop() );

        corridor.area.NewOutline();
        corridor.area.Append( band.GetLeft(), band.GetTop() );
        corridor.area.Append( band.GetRight(), band.GetTop() );
        corridor.area.Append( band.GetRight(), band.GetBottom() );
        corridor.area.Append( band.GetLeft(), band.GetBottom() );
        corridor.box = band;
        corridor.name = "ports " + side + " (";

        for( const std::string& t : to )
            corridor.name += ( corridor.name.back() == '(' ? "" : ", " ) + t;

        corridor.name += to.empty() ? "open)" : ")";
        out.push_back( std::move( corridor ) );
    }

    return out;
}


/// A via to come with its stub from the pin: the part's pads keep off the via (its own net's may
/// touch it), others by the clearance
struct VIA_SPOT
{
    VECTOR2I at;
    int      net = 0;
    int      radius = 0;
    int      clearance = 0;
    VECTOR2I from;       // the pin the stub starts at
    int      stub = 0;   // the stub's half width

    bool Clear( const VECTOR2I& aAt, int aRadius, const SEG& aStub, int aStubHalf, int aClearance ) const
    {
        const double gap = std::max( clearance, aClearance );
        const SEG    own( from, at );

        return ( at - aAt ).EuclideanNorm() >= radius + aRadius + gap
               && own.Distance( aAt ) >= stub + aRadius + gap
               && aStub.Distance( at ) >= radius + aStubHalf + gap
               && own.Distance( aStub ) >= stub + aStubHalf + gap;
    }
};


/// The width of a stub from a pin: as wide as the net class allows, no wider than the pin
int stubWidth( BOARD* aBoard, PAD* aPin )
{
    NETCLASS*      netclass = aPin->GetEffectiveNetClass();
    const VECTOR2I size = aPin->GetSize( aPin->GetLayerSet().CuStack().front() );

    return std::min( netclass && netclass->HasTrackWidth() ? netclass->GetTrackWidth()
                                                           : aBoard->GetDesignSettings().GetCurrentTrackWidth(),
                     std::min( size.x, size.y ) );
}


struct NEAR_OPTIONS
{
    double                gapMm = 0.25;     ///< between courtyards
    double                maxMm = 25.0;     ///< how far from the target pad to look
    std::optional<double> rotationDeg;      ///< fixed; else the four turns in the target footprint's frame
    std::string           direction = "auto";
    std::string           partPad;          ///< pad number of the part to bring to the target; else by net
    double                hostRingMm = 0;   ///< keep this far from the target footprint's courtyard (fan-out room)
    bool                  gapGiven = false; ///< gap_mm asked for (else a default that may be narrowed)
    std::optional<double> partHeight;       ///< of the part (3D model): it may go under raised footprints with room
    const std::vector<CORRIDOR>* corridors = nullptr;   ///< kept free unless no other spot is found
    double                corridorYieldMm = -1;  ///< also enter one when only that brings the pad within this
    std::optional<VECTOR2I>      targetAt;           ///< the spot to bring the pad to, instead of the target pad's
    const std::vector<VIA_SPOT>* viaSpots = nullptr;   ///< vias to come, the part's pads kept clear of them
    const std::set<const FOOTPRINT*>* pending = nullptr;   ///< parts placed after this one: no obstacles
};

struct NEAR_PLACEMENT
{
    bool        ok = false;
    std::string reason;
    PAD*        partPad = nullptr;
    double      padDistanceMm = 0;
    std::string direction;
    std::string under;   ///< the raised footprint the part went under
    std::string corridor;   ///< the corridors the part had to enter
};


/// @brief Where a part may go on its side of the board: clear of the other courtyards (a raised
/// footprint with room for the part: of its pads only), and inside the board outline
class PLACE_SPACE
{
public:
    /// @param aAround the middle of the search, aReach how far from it
    /// @param aHost a footprint kept aHostRing clear (fan-out room), or null
    /// @param aCorridors kept free (those of a net the part has a pad on aside), or null
    PLACE_SPACE( BOARD* aBoard, FOOTPRINT* aPart, const VECTOR2I& aAround, int aReach, FOOTPRINT* aHost, int aHostRing,
                 std::optional<double> aPartHeight, const std::vector<CORRIDOR>* aCorridors = nullptr,
                 const std::vector<VIA_SPOT>* aViaSpots = nullptr,
                 const std::set<const FOOTPRINT*>* aPending = nullptr ) :
            m_outline( aBoard ),
            m_part( aPart ),
            m_viaSpots( aViaSpots )
    {
        for( size_t i = 0; aCorridors && i < aCorridors->size(); ++i )
        {
            const CORRIDOR& corridor = ( *aCorridors )[i];
            const bool      own = std::any_of( aPart->Pads().begin(), aPart->Pads().end(),
                                          [&]( PAD* aPad ) { return corridor.nets.count( aPad->GetNetCode() ) > 0; } );

            if( !own )
                m_corridors.push_back( &corridor );
        }

        BOX2I around( aAround, VECTOR2I( 0, 0 ) );
        around.Inflate( aReach + std::max( aPart->GetBoundingBox( false ).GetWidth(), aPart->GetBoundingBox( false ).GetHeight() ) );

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( fp == aPart || fp->IsFlipped() != aPart->IsFlipped() || ( aPending && aPending->count( fp ) ) )
                continue;

            SHAPE_POLY_SET area = courtyard( fp );
            bool           under = false;

            if( fp == aHost && aHostRing > 0 )
            {
                area.Inflate( aHostRing, CORNER_STRATEGY::ALLOW_ACUTE_CORNERS, ARC_HIGH_DEF );
            }
            else if( std::optional<double> room = roomUnder( fp ); room && aPartHeight && *aPartHeight <= *room )
            {
                m_raised.emplace_back( fp, area );
                area = raisedArea( fp );
                under = true;
            }

            const BOX2I box = area.BBox();

            if( box.Intersects( around ) || under )
                m_obstacles.emplace_back( std::move( area ), box );
        }

        m_edges = aBoard->GetBoardEdgesBoundingBox();
        m_layer = aPart->IsFlipped() ? B_Cu : F_Cu;

        // rule areas on the part's side: no courtyard where footprints are kept out, no pad where
        // tracks or pads are (a pad there can't be connected)
        for( ZONE* zone : aBoard->Zones() )
        {
            if( !zone->GetIsRuleArea() || !zone->IsOnLayer( m_layer ) || !zone->GetBoundingBox().Intersects( around ) )
                continue;

            if( zone->GetDoNotAllowFootprints() )
                m_obstacles.emplace_back( *zone->Outline(), zone->GetBoundingBox() );
            else if( zone->GetDoNotAllowTracks() || zone->GetDoNotAllowPads() )
                m_padKeepouts.emplace_back( *zone->Outline(), zone->GetBoundingBox() );
        }
    }

    /// @brief aArea (the part's courtyard grown by the gap) moved by aDelta: clear of the others and
    /// inside the board outline
    bool Free( const SHAPE_POLY_SET& aArea, const BOX2I& aAreaBox, const VECTOR2I& aDelta ) const
    {
        BOX2I box = aAreaBox;
        box.Move( aDelta );

        if( m_edges.GetWidth() > 0 && m_edges.GetHeight() > 0 && !m_edges.Contains( box ) )
            return false;

        for( size_t i = 0; m_viaSpots && i < m_viaSpots->size(); ++i )
        {
            const VIA_SPOT& spot = ( *m_viaSpots )[i];

            BOX2I reach = box;
            reach.Inflate( spot.radius + spot.clearance );

            if( !reach.Contains( spot.at ) )
                continue;

            for( PAD* pad : m_part->Pads() )
            {
                const int need = spot.radius + ( pad->GetNetCode() == spot.net && spot.net > 0 ? 0 : spot.clearance );

                if( pad->GetEffectiveShape( pad->GetLayerSet().CuStack().front() )->Collide( spot.at - aDelta, need ) )
                    return false;
            }
        }

        for( const auto& [keepout, keepoutBox] : m_padKeepouts )
        {
            if( !keepoutBox.Intersects( box ) )
                continue;

            for( PAD* pad : m_part->Pads() )
            {
                if( !pad->IsOnLayer( m_layer ) )
                    continue;

                BOX2I padBox = pad->GetBoundingBox();
                padBox.Move( aDelta );

                if( !keepoutBox.Intersects( padBox ) )
                    continue;

                SHAPE_POLY_SET copper;
                pad->TransformShapeToPolygon( copper, m_layer, 0, ARC_HIGH_DEF, ERROR_OUTSIDE );
                copper.Move( aDelta );
                copper.BooleanIntersection( keepout );

                if( !copper.IsEmpty() )
                    return false;
            }
        }

        for( const auto& [obstacle, obstacleBox] : m_obstacles )
        {
            if( !obstacleBox.Intersects( box ) )
                continue;

            SHAPE_POLY_SET moved = aArea;
            moved.Move( aDelta );
            moved.BooleanIntersection( obstacle );

            if( !moved.IsEmpty() )
                return false;
        }

        for( const CORRIDOR* corridor : m_corridors )
        {
            if( !corridor->box.Intersects( box ) )
                continue;

            SHAPE_POLY_SET moved = aArea;
            moved.Move( aDelta );
            moved.BooleanIntersection( corridor->area );

            if( !moved.IsEmpty() )
                return false;
        }

        SHAPE_POLY_SET moved = aArea;
        moved.Move( aDelta );
        return m_outline.Contains( moved );
    }

    /// @brief The raised footprint the part's courtyard (where it is now) is under, or empty
    std::string Under() const
    {
        const SHAPE_POLY_SET body = courtyard( m_part );

        for( const auto& [fp, area] : m_raised )
        {
            SHAPE_POLY_SET both = body;
            both.BooleanIntersection( area );

            if( !both.IsEmpty() )
                return str( fp->GetReference() );
        }

        return {};
    }

    /// @brief Names of the corridors the part's courtyard (where it is now) lies in, "; "-joined
    std::string InCorridors() const
    {
        const SHAPE_POLY_SET body = courtyard( m_part );
        std::string          names;

        for( const CORRIDOR* corridor : m_corridors )
        {
            SHAPE_POLY_SET both = body;
            both.BooleanIntersection( corridor->area );

            if( !both.IsEmpty() )
                names += ( names.empty() ? "" : "; " ) + corridor->name;
        }

        return names;
    }

private:
    OUTLINE                                              m_outline;
    FOOTPRINT*                                           m_part;
    const std::vector<VIA_SPOT>*                         m_viaSpots;
    std::vector<const CORRIDOR*>                         m_corridors;
    std::vector<std::pair<SHAPE_POLY_SET, BOX2I>>       m_obstacles;
    std::vector<std::pair<SHAPE_POLY_SET, BOX2I>>       m_padKeepouts;
    std::vector<std::pair<FOOTPRINT*, SHAPE_POLY_SET>>  m_raised;
    BOX2I                                                m_edges;
    PCB_LAYER_ID                                         m_layer = F_Cu;
};


/// @brief A pad as REF.NUMBER (the first of several pads sharing a number)
PAD* findPadByName( BOARD* aBoard, const std::string& aRefPad )
{
    const size_t dot = aRefPad.rfind( '.' );

    if( dot == std::string::npos )
        return nullptr;

    FOOTPRINT* fp = findFootprint( aBoard, aRefPad.substr( 0, dot ) );

    if( !fp )
        return nullptr;

    for( PAD* pad : fp->Pads() )
    {
        if( str( pad->GetNumber() ) == aRefPad.substr( dot + 1 ) )
            return pad;
    }

    return nullptr;
}


/// @brief Puts aPart (already staged in a commit) so that its pad on the target's net is as close to
/// aTarget as the courtyards of the parts on its side allow: every turn, every direction from the
/// pad, sliding along the neighbour when the straight spot is taken.  The part keeps its place when
/// nothing is free.
NEAR_PLACEMENT placeNear( BOARD* aBoard, FOOTPRINT* aPart, PAD* aTarget, const NEAR_OPTIONS& aOptions )
{
    NEAR_PLACEMENT   result;
    FOOTPRINT*       host = aTarget->GetParentFootprint();
    const VECTOR2I   target = aOptions.targetAt.value_or( aTarget->GetPosition() );
    const EDA_ANGLE  originalAngle = aPart->GetOrientation();
    const VECTOR2I   originalPos = aPart->GetPosition();

    std::vector<wxString> padNumbers;

    for( PAD* pad : aPart->Pads() )
    {
        const bool wanted = aOptions.partPad.empty() ? ( aTarget->GetNetCode() > 0 && pad->GetNetCode() == aTarget->GetNetCode() )
                                                     : str( pad->GetNumber() ) == aOptions.partPad;

        if( wanted && std::find( padNumbers.begin(), padNumbers.end(), pad->GetNumber() ) == padNumbers.end() )
            padNumbers.push_back( pad->GetNumber() );
    }

    if( padNumbers.empty() )
    {
        result.reason = aOptions.partPad.empty() ? "no pad of " + str( aPart->GetReference() ) + " is on the target's net "
                                                           + str( UnescapeString( aTarget->GetNetname() ) ) + " (name one: near_pad)"
                                                 : "no pad " + aOptions.partPad + " on " + str( aPart->GetReference() );
        return result;
    }

    const int         reach = toIU( aOptions.maxMm );
    const int         gap = toIU( aOptions.gapMm );
    const int         ring = toIU( std::max( 0.0, aOptions.hostRingMm ) );
    const PLACE_SPACE space( aBoard, aPart, target, reach, host, ring, aOptions.partHeight, nullptr, aOptions.viaSpots,
                             aOptions.pending );
    const PLACE_SPACE kept( aBoard, aPart, target, reach, host, ring, aOptions.partHeight, aOptions.corridors,
                            aOptions.viaSpots, aOptions.pending );

    // directions: along the target footprint's axes, the side its pad is nearest to first
    const EDA_ANGLE base = host->GetOrientation();
    std::vector<std::pair<std::string, VECTOR2D>> directions;

    if( aOptions.direction == "auto" )
    {
        const BOX2I hostBox = courtyard( host ).BBox();
        const std::vector<std::pair<int, std::pair<std::string, VECTOR2D>>> sides = {
            { std::abs( target.x - hostBox.GetLeft() ), { "left", VECTOR2D( -1, 0 ) } },
            { std::abs( hostBox.GetRight() - target.x ), { "right", VECTOR2D( 1, 0 ) } },
            { std::abs( target.y - hostBox.GetTop() ), { "up", VECTOR2D( 0, -1 ) } },
            { std::abs( hostBox.GetBottom() - target.y ), { "down", VECTOR2D( 0, 1 ) } } };
        std::vector<size_t> order = { 0, 1, 2, 3 };

        std::stable_sort( order.begin(), order.end(), [&]( size_t a, size_t b ) { return sides[a].first < sides[b].first; } );

        for( size_t i : order )
            directions.push_back( sides[i].second );
    }
    else
    {
        static const std::map<std::string, VECTOR2D> named = { { "left", VECTOR2D( -1, 0 ) }, { "right", VECTOR2D( 1, 0 ) },
                                                               { "up", VECTOR2D( 0, -1 ) }, { "down", VECTOR2D( 0, 1 ) } };
        directions.emplace_back( aOptions.direction, named.at( aOptions.direction ) );
    }

    std::vector<EDA_ANGLE> turns;

    if( aOptions.rotationDeg )
    {
        turns.push_back( EDA_ANGLE( *aOptions.rotationDeg, DEGREES_T ) );
    }
    else
    {
        for( int quarter = 0; quarter < 4; quarter++ )
            turns.push_back( base + EDA_ANGLE( 90.0 * quarter, DEGREES_T ) );
    }

    static const double laterals[] = { 0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 3.0, -3.0, 4.0, -4.0, 6.0, -6.0 };
    const int           step = toIU( 0.1 );

    struct SPOT
    {
        double      best = std::numeric_limits<double>::max();
        EDA_ANGLE   angle;
        VECTOR2I    pos;
        wxString    padNumber;
        std::string direction;
    };

    auto search = [&]( const PLACE_SPACE& aSpace )
    {
        SPOT       spot;
        double&    best = spot.best;
        EDA_ANGLE& bestAngle = spot.angle;
        VECTOR2I&  bestPos = spot.pos;

        for( const EDA_ANGLE& turn : turns )
        {
            aPart->SetOrientation( turn );
            aPart->SetPosition( originalPos );

            SHAPE_POLY_SET area = courtyard( aPart );

            if( gap > 0 )
                area.Inflate( gap, CORNER_STRATEGY::ALLOW_ACUTE_CORNERS, ARC_HIGH_DEF );

            const BOX2I areaBox = area.BBox();

            auto free = [&]( const VECTOR2I& aDelta ) { return aSpace.Free( area, areaBox, aDelta ); };

            for( PAD* pad : aPart->Pads() )
            {
                if( std::find( padNumbers.begin(), padNumbers.end(), pad->GetNumber() ) == padNumbers.end() )
                    continue;

                const VECTOR2I padOffset = pad->GetPosition() - originalPos;

                for( const auto& [name, direction] : directions )
                {
                    const VECTOR2D across( -direction.y, direction.x );

                    for( double lateral : laterals )
                    {
                        const double lateralIU = toIU( std::abs( lateral ) ) * ( lateral < 0 ? -1.0 : 1.0 );

                        if( std::abs( lateralIU ) >= best )
                            break;

                        for( int along = 0; along <= reach; along += step )
                        {
                            const double distance = std::hypot( double( along ), lateralIU );

                            if( distance >= best )
                                break;

                            const VECTOR2I padAt = target + VECTOR2I( KiROUND( direction.x * along + across.x * lateralIU ),
                                                                      KiROUND( direction.y * along + across.y * lateralIU ) );
                            const VECTOR2I pos = padAt - padOffset;

                            if( free( pos - originalPos ) )
                            {
                                best = distance;
                                bestAngle = turn;
                                bestPos = pos;
                                spot.padNumber = pad->GetNumber();
                                spot.direction = name;
                                break;
                            }
                        }
                    }
                }
            }
        }

        return spot;
    };

    // corridors of pairs and clocks kept free; entered only when nothing else is found (or, with a
    // yield distance, when only that brings the pad within it)
    const bool corridors = aOptions.corridors && !aOptions.corridors->empty();
    SPOT       chosen = search( corridors ? kept : space );
    bool       entered = false;

    if( corridors && ( chosen.best == std::numeric_limits<double>::max()
                       || ( aOptions.corridorYieldMm >= 0 && toMm( chosen.best ) > aOptions.corridorYieldMm ) ) )
    {
        SPOT open = search( space );

        if( open.best != std::numeric_limits<double>::max()
            && ( chosen.best == std::numeric_limits<double>::max() || toMm( open.best ) <= aOptions.corridorYieldMm ) )
        {
            chosen = open;
            entered = true;
        }
    }

    const double    best = chosen.best;
    const EDA_ANGLE bestAngle = chosen.angle;
    const VECTOR2I  bestPos = chosen.pos;
    result.direction = chosen.direction;

    if( best == std::numeric_limits<double>::max() )
    {
        aPart->SetOrientation( originalAngle );
        aPart->SetPosition( originalPos );
        result.partPad = nullptr;
        result.reason = "no free place within " + std::to_string( aOptions.maxMm ).substr( 0, 5 ) + " mm of "
                        + str( host->GetReference() ) + "." + str( aTarget->GetNumber() ) + " on that side of the board";
        return result;
    }

    // the pad is taken again after the final turn
    const wxString number = chosen.padNumber;

    aPart->SetOrientation( bestAngle );
    aPart->SetPosition( bestPos );

    for( PAD* pad : aPart->Pads() )
    {
        if( pad->GetNumber() == number
            && ( !result.ok || ( pad->GetPosition() - target ).EuclideanNorm() < ( result.partPad->GetPosition() - target ).EuclideanNorm() ) )
        {
            result.partPad = pad;
            result.ok = true;
        }
    }

    result.padDistanceMm = toMm( ( result.partPad->GetPosition() - aTarget->GetPosition() ).EuclideanNorm() );
    result.under = space.Under();

    if( entered )
        result.corridor = kept.InCorridors();

    return result;
}


nlohmann::json nearJson( const NEAR_PLACEMENT& aPlacement, PAD* aTarget )
{
    nlohmann::json row = { { "target", str( aTarget->GetParentFootprint()->GetReference() ) + "." + str( aTarget->GetNumber() ) },
             { "net", str( UnescapeString( aTarget->GetNetname() ) ) },
             { "pad", str( aPlacement.partPad->GetNumber() ) },
             { "pad_distance_mm", std::round( aPlacement.padDistanceMm * 100 ) / 100 },
             { "direction", aPlacement.direction } };

    if( !aPlacement.under.empty() )
        row["under"] = aPlacement.under;

    if( !aPlacement.corridor.empty() )
        row["in_corridor"] = aPlacement.corridor;

    return row;
}


std::optional<std::string> nearOptions( const nlohmann::json& aFrom, NEAR_OPTIONS& aOptions, double aDefaultGap = 0.25 )
{
    aOptions.gapMm = aFrom.value( "gap_mm", aDefaultGap );
    aOptions.gapGiven = aFrom.contains( "gap_mm" );
    aOptions.hostRingMm = aFrom.value( "fanout_ring_mm", -1.0 );
    aOptions.maxMm = aFrom.value( "max_mm", 25.0 );
    aOptions.direction = aFrom.value( "near_direction", std::string( "auto" ) );
    aOptions.partPad = aFrom.value( "near_pad", std::string() );

    if( aFrom.contains( "rotation_deg" ) && aFrom["rotation_deg"].is_number() )
        aOptions.rotationDeg = aFrom["rotation_deg"].get<double>();

    if( aOptions.direction != "auto" && aOptions.direction != "left" && aOptions.direction != "right"
        && aOptions.direction != "up" && aOptions.direction != "down" )
    {
        return "near_direction: auto, left, right, up or down (from the target pad, on the board)";
    }

    if( aOptions.gapMm < 0 || aOptions.gapMm > 10 || aOptions.maxMm <= 0 || aOptions.maxMm > 200 )
        return "gap_mm 0 … 10, max_mm up to 200";

    return std::nullopt;
}


/// @brief The least move that takes aPart (staged) off the others' courtyards (aGapMm clear) and
/// keeps it inside the outline: rings around where it is, 0.1 mm apart up to 2 mm, then 0.25 mm,
/// its turn kept.  The distance moved (0: it was free), or none when nothing is free within aMaxMm.
std::optional<double> nudge( BOARD* aBoard, FOOTPRINT* aPart, double aGapMm, double aMaxMm, std::optional<double> aHeight )
{
    const VECTOR2I    start = aPart->GetPosition();
    const PLACE_SPACE space( aBoard, aPart, start, toIU( aMaxMm ), nullptr, 0, aHeight );
    SHAPE_POLY_SET    area = courtyard( aPart );

    if( aGapMm > 0 )
        area.Inflate( toIU( aGapMm ), CORNER_STRATEGY::ALLOW_ACUTE_CORNERS, ARC_HIGH_DEF );

    const BOX2I box = area.BBox();

    if( space.Free( area, box, VECTOR2I( 0, 0 ) ) )
        return 0.0;

    for( double r = 0.1; r <= aMaxMm + 1e-9; r += r < 2.0 - 1e-9 ? 0.1 : 0.25 )
    {
        const double arc = r < 2.0 ? 0.1 : 0.25;
        const int    steps = 4 * std::max( 2, int( std::ceil( 2 * M_PI * r / arc / 4 ) ) );

        for( int k = 0; k < steps; ++k )
        {
            const double   angle = 2 * M_PI * k / steps;
            const VECTOR2I delta( KiROUND( toIU( r ) * std::cos( angle ) ), KiROUND( toIU( r ) * std::sin( angle ) ) );

            if( space.Free( area, box, delta ) )
            {
                aPart->SetPosition( start + delta );
                return r;
            }
        }
    }

    return std::nullopt;
}


/// @brief Pads 0.65 mm apart or closer (QFP, QFN, LGA, BGA)
bool finePitch( FOOTPRINT* aFootprint )
{
    for( PAD* a : aFootprint->Pads() )
    {
        for( PAD* b : aFootprint->Pads() )
        {
            if( KopenapiPadNeighbours( a, b ) && ( a->GetPosition() - b->GetPosition() ).EuclideanNorm() <= toIU( 0.65 ) )
                return true;
        }
    }

    return false;
}


/// @brief The spot for a via from aPin of a package to a part under it on the other side: on the line
/// from the pin into the body (along the package axis, then straight to its middle), just past the
/// pin's inner end, clear of other nets' copper (pads, tracks, vias, the spots in aTaken) by the
/// clearance; else just past the pin's outer end.  A spot of the net within 1.2 mm of the pin is
/// shared instead.  aDirection gets the board axis pointing into the body.
/// The way a pin of a quad row takes into its package: along the package axis, in the board's frame
VECTOR2D inwardAxis( PAD* aPin )
{
    FOOTPRINT* host = aPin->GetParentFootprint();
    VECTOR2D   local = VECTOR2D( aPin->GetPosition() - host->GetPosition() );
    RotatePoint( local, -host->GetOrientation() );
    VECTOR2D axis = std::abs( local.x ) >= std::abs( local.y ) ? VECTOR2D( local.x > 0 ? -1 : 1, 0 )
                                                               : VECTOR2D( 0, local.y > 0 ? -1 : 1 );
    RotatePoint( axis, host->GetOrientation() );
    return axis;
}


/// A rail pin's way into its package that a spot of another net must leave open
struct RAIL_WAY
{
    SEG way;
    int net;
    int half;
};


/// @param aWays other rail pins' ways inward: a spot keeps clear of them (with its stub), so no
/// rail pin is shut in by a neighbour's via; at fine pitch no spot is taken outside the body
std::optional<VECTOR2I> viaSpotUnder( BOARD* aBoard, PAD* aPin, int aViaDiameter, const std::vector<VIA_SPOT>& aTaken,
                                      const std::vector<RAIL_WAY>& aWays, std::string& aDirection )
{
    FOOTPRINT*                   host = aPin->GetParentFootprint();
    const BOARD_DESIGN_SETTINGS& ds = aBoard->GetDesignSettings();
    const int                    radius = aViaDiameter / 2;
    const int                    net = aPin->GetNetCode();
    NETCLASS*                    netclass = aPin->GetEffectiveNetClass();
    const int                    clearance = std::max( ds.m_MinClearance, netclass ? netclass->GetClearance() : 0 );
    const VECTOR2I               pin = aPin->GetPosition();
    const BOX2I                  body = courtyard( host ).BBox();
    const VECTOR2I               middle = body.GetCenter();

    BOX2I zone( pin, VECTOR2I( 0, 0 ) );
    zone.Inflate( std::max( body.GetWidth(), body.GetHeight() ) + aViaDiameter );

    std::vector<std::pair<std::shared_ptr<SHAPE>, int>> copper;   // other nets' shapes, their clearance

    auto clearanceOf = [&]( BOARD_CONNECTED_ITEM* aItem )
    {
        NETCLASS* nc = aItem->GetEffectiveNetClass();
        return std::max( clearance, nc ? nc->GetClearance() : 0 );
    };

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( !fp->GetBoundingBox( false ).Intersects( zone ) )
            continue;

        for( PAD* pad : fp->Pads() )
        {
            if( pad == aPin || ( net > 0 && pad->GetNetCode() == net ) || !pad->GetBoundingBox().Intersects( zone ) )
                continue;

            for( PCB_LAYER_ID layer : pad->GetLayerSet().CuStack() )
            {
                copper.emplace_back( pad->GetEffectiveShape( layer ), clearanceOf( pad ) );
                break;
            }
        }
    }

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->GetNetCode() != net && track->GetBoundingBox().Intersects( zone ) )
            copper.emplace_back( track->GetEffectiveShape( track->GetLayer() ), clearanceOf( track ) );
    }

    const std::shared_ptr<SHAPE> own = aPin->GetEffectiveShape( aPin->GetLayerSet().CuStack().front() );
    const int                    stubHalf = stubWidth( aBoard, aPin ) / 2;

    // the stub from the pin to a spot clear of other nets' spots, stubs and copper
    auto stubClear = [&]( const VECTOR2I& aAt )
    {
        const SEG stub( pin, aAt );

        for( const VIA_SPOT& taken : aTaken )
        {
            if( taken.net != net && !taken.Clear( aAt, radius, stub, stubHalf, clearance ) )
                return false;
        }

        for( const auto& [shape, gap] : copper )
        {
            if( shape->Collide( stub, stubHalf + gap ) )
                return false;
        }

        return true;
    };

    auto free = [&]( const VECTOR2I& aAt )
    {
        if( own->Collide( aAt, radius ) )
            return false;

        const SEG stub( pin, aAt );

        for( const RAIL_WAY& way : aWays )
        {
            if( way.net != net
                && ( way.way.Distance( aAt ) < radius + way.half + clearance || way.way.Distance( stub ) < stubHalf + way.half + clearance ) )
            {
                return false;
            }
        }

        for( const VIA_SPOT& taken : aTaken )
        {
            if( taken.net == net && ( taken.at - aAt ).EuclideanNorm() < radius + taken.radius )
                return false;
        }

        for( const auto& [shape, gap] : copper )
        {
            if( shape->Collide( aAt, radius + gap ) )
                return false;
        }

        return stubClear( aAt );
    };

    const VECTOR2D axis = inwardAxis( aPin );

    const VECTOR2D toMiddle = VECTOR2D( middle - pin );
    const double   span = toMiddle.EuclideanNorm();

    if( span < 1 )
        return std::nullopt;

    const int step = toIU( 0.05 );

    aDirection = std::abs( axis.x ) >= std::abs( axis.y ) ? ( axis.x > 0 ? "right" : "left" ) : ( axis.y > 0 ? "down" : "up" );

    // a neighbour pin of the net already has one close by: shared
    const VIA_SPOT* shared = nullptr;

    for( const VIA_SPOT& taken : aTaken )
    {
        if( net > 0 && taken.net == net && ( taken.at - pin ).EuclideanNorm() <= toIU( 1.2 ) && stubClear( taken.at )
            && ( !shared || ( taken.at - pin ).EuclideanNorm() < ( shared->at - pin ).EuclideanNorm() ) )
        {
            shared = &taken;
        }
    }

    if( shared )
        return shared->at;

    for( const VECTOR2D& ray : { axis, toMiddle / span } )
    {
        for( int along = step; along < span; along += step )
        {
            const VECTOR2I at = pin + VECTOR2I( KiROUND( ray.x * along ), KiROUND( ray.y * along ) );

            if( free( at ) )
                return at;
        }
    }

    for( int along = step; !finePitch( host ) && along < toIU( 2.0 ); along += step )
    {
        const VECTOR2I at = pin - VECTOR2I( KiROUND( axis.x * along ), KiROUND( axis.y * along ) );

        if( free( at ) )
            return at;
    }

    return std::nullopt;
}


/// @brief Places the parts of a functional block around its main part: each next to the pad of the
/// anchor it belongs to.  A part's pad goes to the anchor pad of its most specific shared net (the
/// net with the fewest pads on the board); a part across two rails of the anchor (a decoupling
/// capacitor) takes a free supply pad (adjacent pads of one rail count as one) whose ground pad is
/// nearest, the smallest value first.  Rails are told by role (railNets), not by pad count.  Order:
/// parts on two nets of their own (a crystal), decoupling, the rest.  Parts are staged in aCommit;
/// aOut gets placed / unplaced (and decoupling_far: capacitors over aDecapMaxMm from their pin).
void placeGroup( BOARD* aBoard, BOARD_COMMIT& aCommit, FOOTPRINT* aAnchor, std::vector<FOOTPRINT*> aParts,
                 const NEAR_OPTIONS& aOptions, bool aOpposite, double aDecapMaxMm, std::optional<bool> aDecapOtherSide,
                 nlohmann::json& aOut, std::vector<KIID>& aMoved, bool aDecapVias = true )
{
    const bool decapOtherSide = aDecapOtherSide.value_or( finePitch( aAnchor ) )
                                && aBoard->GetCopperLayerCount() >= 2;

    std::map<int, int> netPads;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    const std::set<int>                      rails = railNets( aBoard );
    const std::map<PAD*, int>                groupOf = padGroups( aAnchor );
    const std::map<const FOOTPRINT*, double> heights = heightsIfRoomUnder( aBoard, aParts );
    std::map<int, std::vector<PAD*>>         anchorPads;

    for( PAD* pad : aAnchor->Pads() )
    {
        if( pad->GetNetCode() > 0 )
            anchorPads[pad->GetNetCode()].push_back( pad );
    }

    struct MEMBER
    {
        FOOTPRINT*       part;
        std::vector<int> nets;      // shared with the anchor, most specific first
        double           area;
        int              ownNets = 0;          // of them, nets that are not rails
        bool             decoupling = false;   // across two rails of the anchor
        double           farads = 1.0;
        bool             capacitor = false;
        bool             inductor = false;      // a regulator's inductor at its switching pin
    };

    std::vector<MEMBER>     members;
    std::vector<FOOTPRINT*> indirect;   // no net in common with the anchor: placed at a member they share one with
    nlohmann::json          unplaced = nlohmann::json::array();

    for( FOOTPRINT* part : aParts )
    {
        MEMBER member{ part, {}, courtyard( part ).Area() };

        for( PAD* pad : part->Pads() )
        {
            if( anchorPads.count( pad->GetNetCode() )
                && std::find( member.nets.begin(), member.nets.end(), pad->GetNetCode() ) == member.nets.end() )
            {
                member.nets.push_back( pad->GetNetCode() );
            }
        }

        std::stable_sort( member.nets.begin(), member.nets.end(), [&]( int a, int b ) { return netPads[a] < netPads[b]; } );

        for( int net : member.nets )
            member.ownNets += rails.count( net ) ? 0 : 1;

        // one rail in common is no bond with the anchor (everything shares ground): such a part
        // belongs to a member, if to anything; two nets in common is a part across the anchor's pins
        if( member.ownNets == 0 && member.nets.size() < 2 )
        {
            indirect.push_back( part );
            continue;
        }

        member.decoupling = member.ownNets == 0;
        member.capacitor = faradsOf( part ).has_value() || KopenapiRefPrefix( str( part->GetReference() ) ) == "C";
        member.farads = faradsOf( part ).value_or( 1.0 );
        member.inductor = !member.capacitor && ( henryValue( part ) || KopenapiRefPrefix( str( part->GetReference() ) ) == "L" );

        // a decoupling capacitor belongs at its supply pin, its ground the nearest to that
        if( member.decoupling )
        {
            auto isGround = [&]( int aNet )
            {
                NETINFO_ITEM* info = aBoard->FindNet( aNet );
                return info && KopenapiNetRoleFromName( str( UnescapeString( info->GetNetname() ) ) ).value( "role", "" )
                                       == "ground";
            };

            std::stable_partition( member.nets.begin(), member.nets.end(), [&]( int n ) { return !isGround( n ); } );
        }

        members.push_back( member );
    }

    auto rank = []( const MEMBER& m ) { return m.ownNets >= 2 ? 0 : m.decoupling ? 1 : 2; };

    // the most bound first: a crystal on two nets of its own, a regulator's inductor across two
    // rails, the decoupling capacitors (smallest value first: nearest its pin), then parts on one
    // net of their own; among equals the most specific net, then big before small
    std::stable_sort( members.begin(), members.end(),
                      [&]( const MEMBER& a, const MEMBER& b )
                      {
                          if( rank( a ) != rank( b ) )
                              return rank( a ) < rank( b );

                          if( a.decoupling && b.decoupling && a.inductor != b.inductor )
                              return a.inductor;   // the switching loop before the caps

                          if( a.decoupling && a.farads != b.farads )
                              return a.farads < b.farads;

                          if( netPads[a.nets.front()] != netPads[b.nets.front()] )
                              return netPads[a.nets.front()] < netPads[b.nets.front()];

                          return a.area > b.area;
                      } );

    std::map<PAD*, int>     taken;   // parts placed at the pad
    std::set<int>           takenGroups;
    nlohmann::json          placed = nlohmann::json::array();
    nlohmann::json          farAway = nlohmann::json::array();
    std::vector<FOOTPRINT*> done;
    std::vector<VIA_SPOT>                 viaSpots;     // under the anchor, for the bottom caps
    std::map<FOOTPRINT*, VECTOR2I>        viaSpotOf;

    struct SPOT_AT
    {
        VIA_SPOT    spot;
        std::string inward;
    };

    std::map<PAD*, SPOT_AT> spotOfPin;   // reserved before the caps go

    auto spotFor = [&]( PAD* aPin ) -> std::optional<SPOT_AT>
    {
        NETCLASS*   netclass = aPin->GetEffectiveNetClass();
        const int   diameter = netclass && netclass->HasViaDiameter() ? netclass->GetViaDiameter()
                                                                      : aBoard->GetDesignSettings().GetCurrentViaSize();
        std::string inward;

        // the other rail pins' ways inward: to their spot once they have one, else to the middle
        std::vector<RAIL_WAY> ways;
        const VECTOR2I        middle = courtyard( aAnchor ).BBox().GetCenter();

        for( PAD* pad : aAnchor->Pads() )
        {
            if( pad == aPin || !rails.count( pad->GetNetCode() ) || pad->GetEffectiveShape( pad->GetLayerSet().CuStack().front() )->Collide( middle, 0 ) )
                continue;

            auto           reserved = spotOfPin.find( pad );
            const VECTOR2D axis = inwardAxis( pad );
            const double   depth = std::abs( VECTOR2D( middle - pad->GetPosition() ).Dot( axis ) );
            const VECTOR2I end = reserved != spotOfPin.end() ? reserved->second.spot.at
                                                             : pad->GetPosition() + VECTOR2I( KiROUND( axis.x * depth ), KiROUND( axis.y * depth ) );

            ways.push_back( { SEG( pad->GetPosition(), end ), pad->GetNetCode(), stubWidth( aBoard, pad ) / 2 } );
        }

        if( std::optional<VECTOR2I> at = viaSpotUnder( aBoard, aPin, diameter, viaSpots, ways, inward ) )
        {
            return SPOT_AT{ { *at, aPin->GetNetCode(), diameter / 2,
                              std::max( aBoard->GetDesignSettings().m_MinClearance, netclass ? netclass->GetClearance() : 0 ),
                              aPin->GetPosition(), stubWidth( aBoard, aPin ) / 2 },
                            inward };
        }

        return std::nullopt;
    };

    // a bottom cap's via at its spot, with a stub from the pin on the pin's layer; not laid when
    // the via or the stub would come too close to another net's copper or spots
    std::set<std::pair<int, int>> laid;
    std::set<FOOTPRINT*>          joined;   // caps whose pad is joined to their via

    auto layVia = [&]( PAD* aPin, const VIA_SPOT& aSpot )
    {
        NETCLASS*                    netclass = aPin->GetEffectiveNetClass();
        const BOARD_DESIGN_SETTINGS& ds = aBoard->GetDesignSettings();
        const PCB_LAYER_ID           layer = aPin->GetLayerSet().CuStack().front();
        const int                    width = stubWidth( aBoard, aPin );
        const SEG                    stub( aPin->GetPosition(), aSpot.at );

        for( const VIA_SPOT& other : viaSpots )
        {
            if( other.net != aPin->GetNetCode() && !other.Clear( aSpot.at, aSpot.radius, stub, width / 2, aSpot.clearance ) )
                return false;
        }

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
            {
                if( pad->GetNetCode() == aPin->GetNetCode() || !pad->IsOnLayer( layer ) )
                    continue;

                if( pad->GetEffectiveShape( layer )->Collide( stub, aSpot.clearance + width / 2 ) )
                    return false;
            }
        }

        for( PCB_TRACK* track : aBoard->Tracks() )
        {
            if( track->GetNetCode() == aPin->GetNetCode() )
                continue;

            if( ( track->IsOnLayer( layer ) && track->GetEffectiveShape( layer )->Collide( stub, aSpot.clearance + width / 2 ) )
                || track->GetEffectiveShape( track->GetLayer() )->Collide( aSpot.at, aSpot.clearance + aSpot.radius ) )
            {
                return false;
            }
        }

        PCB_VIA* via = new PCB_VIA( aBoard );
        via->SetPosition( aSpot.at );
        via->SetLayerPair( F_Cu, B_Cu );
        via->SetWidth( 2 * aSpot.radius );
        via->SetDrill( netclass && netclass->HasViaDrill() ? netclass->GetViaDrill() : ds.GetCurrentViaDrill() );
        via->SetNetCode( aPin->GetNetCode() );
        aCommit.Add( via );

        PCB_TRACK* track = new PCB_TRACK( aBoard );
        track->SetStart( aPin->GetPosition() );
        track->SetEnd( aSpot.at );
        track->SetLayer( layer );
        track->SetWidth( width );
        track->SetNetCode( aPin->GetNetCode() );
        aCommit.Add( track );
        return true;
    };

    // the cap's pad joined to its via on the cap's side, when the via is not under the pad already
    // and the track keeps clear of other nets' pads, tracks and vias to come
    std::vector<std::tuple<SEG, int, int>> capStubs;   // segment, net, half width

    auto joinCap = [&]( FOOTPRINT* aCap, const VIA_SPOT& aSpot )
    {
        PAD* pad = nullptr;

        for( PAD* candidate : aCap->Pads() )
        {
            if( candidate->GetNetCode() == aSpot.net )
                pad = candidate;
        }

        if( !pad )
            return false;

        const PCB_LAYER_ID layer = pad->GetLayerSet().CuStack().front();

        if( pad->GetEffectiveShape( layer )->Collide( aSpot.at, 0 ) )
            return true;

        const int half = stubWidth( aBoard, pad ) / 2;
        const int gap = aSpot.clearance + half;
        const SEG stub( aSpot.at, pad->GetPosition() );

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            for( PAD* other : fp->Pads() )
            {
                if( other->GetNetCode() != aSpot.net && other->IsOnLayer( layer )
                    && other->GetEffectiveShape( layer )->Collide( stub, gap ) )
                {
                    return false;
                }
            }
        }

        for( PCB_TRACK* track : aBoard->Tracks() )
        {
            if( track->GetNetCode() != aSpot.net && track->IsOnLayer( layer )
                && track->GetEffectiveShape( layer )->Collide( stub, gap ) )
            {
                return false;
            }
        }

        for( const VIA_SPOT& other : viaSpots )
        {
            if( other.net != aSpot.net && stub.Distance( other.at ) < other.radius + std::max( gap, other.clearance + half ) )
                return false;
        }

        for( const auto& [seg, net, width] : capStubs )
        {
            if( net != aSpot.net && stub.Distance( seg ) < half + width + aSpot.clearance )
                return false;
        }

        PCB_TRACK* track = new PCB_TRACK( aBoard );
        track->SetStart( aSpot.at );
        track->SetEnd( pad->GetPosition() );
        track->SetLayer( layer );
        track->SetWidth( 2 * half );
        track->SetNetCode( aSpot.net );
        aCommit.Add( track );
        capStubs.emplace_back( stub, aSpot.net, half );
        return true;
    };

    // every supply pin a bottom cap may take gets its spot first, so the caps keep off the spots of
    // the pins after them

    if( decapOtherSide )
    {
        std::set<int> supplies;

        for( const MEMBER& member : members )
        {
            if( member.decoupling && member.capacitor )
                supplies.insert( member.nets.front() );
        }

        for( PAD* pad : aAnchor->Pads() )
        {
            if( !supplies.count( pad->GetNetCode() ) )
                continue;

            if( std::optional<SPOT_AT> spot = spotFor( pad ) )
            {
                spotOfPin.emplace( pad, *spot );

                if( std::none_of( viaSpots.begin(), viaSpots.end(),
                                  [&]( const VIA_SPOT& v ) { return v.at == spot->spot.at && v.from == spot->spot.from; } ) )
                {
                    viaSpots.push_back( spot->spot );
                }
            }
        }
    }

    auto heightOf = [&]( FOOTPRINT* aPart ) -> std::optional<double>
    {
        auto it = heights.find( aPart );
        return it == heights.end() ? std::nullopt : std::optional<double>( it->second );
    };

    for( const MEMBER& member : members )
    {
        const int net = member.nets.front();
        PAD*      target = nullptr;
        double    bestScore = std::numeric_limits<double>::max();

        for( PAD* candidate : anchorPads[net] )
        {
            auto   group = groupOf.find( candidate );
            double score = taken.count( candidate ) ? 1e6 * taken[candidate] : group != groupOf.end() && takenGroups.count( group->second ) ? 1e5 : 0;

            if( member.nets.size() > 1 )
            {
                double nearest = std::numeric_limits<double>::max();

                for( PAD* other : anchorPads[member.nets[1]] )
                    nearest = std::min( nearest, double( ( other->GetPosition() - candidate->GetPosition() ).EuclideanNorm() ) );

                score += toMm( nearest );
            }

            if( score < bestScore )
            {
                bestScore = score;
                target = candidate;
            }
        }

        // a small part already placed on that net (a crystal before its load capacitors): next to
        // its pad rather than across it at the anchor's; never on a rail
        PAD* viaPad = nullptr;

        for( FOOTPRINT* placedPart : rails.count( net ) ? std::vector<FOOTPRINT*>() : done )
        {
            if( placedPart->Pads().size() > 4 )
                continue;

            for( PAD* pad : placedPart->Pads() )
            {
                if( pad->GetNetCode() == net && !taken.count( pad ) )
                {
                    viaPad = pad;
                    break;
                }
            }

            if( viaPad )
                break;
        }

        if( viaPad )
            target = viaPad;

        taken[target]++;

        if( auto group = groupOf.find( target ); group != groupOf.end() )
            takenGroups.insert( group->second );

        aCommit.Modify( member.part );

        if( ( aAnchor->IsFlipped() != member.part->IsFlipped() ) != aOpposite )
            member.part->Flip( member.part->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );

        NEAR_OPTIONS options = aOptions;
        options.partPad.clear();
        options.partHeight = heightOf( member.part );
        options.corridorYieldMm = member.decoupling ? aDecapMaxMm : -1;

        // a fine-pitch anchor (0.65 mm or less) keeps a ring for its fan-out, unless asked otherwise
        if( options.hostRingMm < 0 )
            options.hostRingMm = finePitch( aAnchor ) ? 1.0 : 0.0;

        NEAR_PLACEMENT placement = placeNear( aBoard, member.part, target, options );

        if( !placement.ok && options.hostRingMm > 0 )
        {
            options.hostRingMm = 0;   // no room with the ring: as close as the courtyards allow
            placement = placeNear( aBoard, member.part, target, options );
        }

        if( !placement.ok && !options.gapGiven && options.gapMm > 0.5 )
        {
            options.gapMm = 0.5;      // no room with the default gap: the narrow one
            placement = placeNear( aBoard, member.part, target, options );
        }

        // a decoupling capacitor kept out by the ring: inside it when that brings it within reach
        if( placement.ok && member.decoupling && placement.padDistanceMm > aDecapMaxMm && options.hostRingMm > 0 && !viaPad )
        {
            const VECTOR2I  pos = member.part->GetPosition();
            const EDA_ANGLE angle = member.part->GetOrientation();
            NEAR_OPTIONS    tight = options;

            tight.hostRingMm = 0;

            if( NEAR_PLACEMENT closer = placeNear( aBoard, member.part, target, tight );
                closer.ok && closer.padDistanceMm < placement.padDistanceMm )
            {
                placement = closer;
            }
            else
            {
                member.part->SetOrientation( angle );
                member.part->SetPosition( pos );
            }
        }

        // still out of reach (the inductor: always): in the old spots of the group's parts yet to go
        // (they move anyway)
        if( placement.ok && member.decoupling && ( placement.padDistanceMm > aDecapMaxMm || member.inductor ) && !viaPad )
        {
            const VECTOR2I             pos = member.part->GetPosition();
            const EDA_ANGLE            angle = member.part->GetOrientation();
            std::set<const FOOTPRINT*> pending;
            NEAR_OPTIONS               free = options;

            for( size_t i = size_t( &member - members.data() ) + 1; i < members.size(); ++i )
                pending.insert( members[i].part );

            free.pending = &pending;

            NEAR_PLACEMENT closer = placeNear( aBoard, member.part, target, free );

            // the inductor closes the switching loop: inside the ring and at the narrow gap whenever
            // that is closer
            if( closer.ok && ( closer.padDistanceMm > aDecapMaxMm || member.inductor )
                && ( free.hostRingMm > 0 || ( member.inductor && !free.gapGiven && free.gapMm > 0.25 ) ) )
            {
                const VECTOR2I  ringPos = member.part->GetPosition();
                const EDA_ANGLE ringAngle = member.part->GetOrientation();

                free.hostRingMm = 0;

                if( member.inductor && !free.gapGiven )
                    free.gapMm = std::min( free.gapMm, 0.25 );

                if( NEAR_PLACEMENT inside = placeNear( aBoard, member.part, target, free );
                    inside.ok && inside.padDistanceMm < closer.padDistanceMm )
                {
                    closer = inside;
                }
                else
                {
                    member.part->SetOrientation( ringAngle );
                    member.part->SetPosition( ringPos );
                }
            }

            if( closer.ok && closer.padDistanceMm < placement.padDistanceMm )
            {
                placement = closer;
            }
            else
            {
                member.part->SetOrientation( angle );
                member.part->SetPosition( pos );
            }
        }

        // still out of reach: under the anchor on the other side, when that brings it within reach (a
        // cap, or a filter resistor; the inductor stays on the regulator's side)
        if( placement.ok && member.decoupling && ( member.capacitor || ( !member.inductor && member.part->Pads().size() == 2 ) )
            && decapOtherSide && placement.padDistanceMm > aDecapMaxMm
            && !viaPad )
        {
            const VECTOR2I  pos = member.part->GetPosition();
            const EDA_ANGLE angle = member.part->GetOrientation();
            NEAR_OPTIONS    under = options;

            under.hostRingMm = 0;

            if( !under.gapGiven )
                under.gapMm = std::min( under.gapMm, 0.25 );   // no tracks run between the caps under a package

            member.part->Flip( member.part->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );

            auto                  reserved = spotOfPin.find( target );
            std::optional<SPOT_AT> spot = reserved != spotOfPin.end() ? std::optional<SPOT_AT>( reserved->second )
                                                                       : spotFor( target );
            const size_t          count = viaSpots.size();

            if( spot )
            {
                under.targetAt = spot->spot.at;
                under.direction = spot->inward;

                if( reserved == spotOfPin.end() )
                    viaSpots.push_back( spot->spot );
            }

            under.viaSpots = &viaSpots;

            NEAR_PLACEMENT flipped = placeNear( aBoard, member.part, target, under );

            if( spot && ( !flipped.ok || flipped.padDistanceMm > aDecapMaxMm ) )
            {
                under.direction = "auto";   // no room inward of the spot: on any side of it
                flipped = placeNear( aBoard, member.part, target, under );
            }

            if( flipped.ok && flipped.padDistanceMm <= aDecapMaxMm )
            {
                placement = flipped;

                if( spot )
                {
                    viaSpotOf[member.part] = spot->spot.at;

                    if( aDecapVias && !laid.count( { spot->spot.at.x, spot->spot.at.y } ) && layVia( target, spot->spot ) )
                        laid.insert( { spot->spot.at.x, spot->spot.at.y } );

                    if( laid.count( { spot->spot.at.x, spot->spot.at.y } ) && joinCap( member.part, spot->spot ) )
                        joined.insert( member.part );
                }
            }
            else
            {
                viaSpots.resize( count );
                member.part->Flip( member.part->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );
                member.part->SetOrientation( angle );
                member.part->SetPosition( pos );
            }
        }

        if( !placement.ok )
        {
            unplaced.push_back( { { "ref", str( member.part->GetReference() ) }, { "reason", placement.reason } } );
            continue;
        }

        member.part->SetAttributes( member.part->GetAttributes() & ~FP_JUST_ADDED );
        aMoved.push_back( member.part->m_Uuid );
        done.push_back( member.part );

        nlohmann::json row = nearJson( placement, target );
        row["ref"] = str( member.part->GetReference() );

        if( viaPad )
            row["via"] = str( viaPad->GetParentFootprint()->GetReference() );
        row["x_mm"] = toMm( member.part->GetPosition().x );
        row["y_mm"] = toMm( member.part->GetPosition().y );
        row["rotation_deg"] = member.part->GetOrientation().AsDegrees();
        row["side"] = member.part->IsFlipped() ? "bottom" : "top";

        if( auto spot = viaSpotOf.find( member.part ); spot != viaSpotOf.end() )
        {
            row["via_spot_mm"] = { toMm( spot->second.x ), toMm( spot->second.y ) };
            row["via_laid"] = laid.count( { spot->second.x, spot->second.y } ) > 0;
            row["via_joined"] = joined.count( member.part ) > 0;
        }

        if( member.decoupling )
        {
            row["decoupling"] = true;

            if( placement.padDistanceMm > aDecapMaxMm )
            {
                farAway.push_back( { { "ref", row["ref"] }, { "target", row["target"] }, { "pad_distance_mm", row["pad_distance_mm"] },
                                 { "decap_max_mm", aDecapMaxMm } } );
            }
        }

        placed.push_back( row );
    }

    // parts placed at the anchor before (by an earlier call): hosts for parts bound to them
    std::vector<FOOTPRINT*> earlier;
    BOX2I                   anchorBox = courtyard( aAnchor ).BBox();

    anchorBox.Inflate( toIU( aOptions.maxMm ) );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( fp == aAnchor || isConnector( fp ) || std::find( aParts.begin(), aParts.end(), fp ) != aParts.end()
            || !anchorBox.Contains( fp->GetPosition() ) )
        {
            continue;
        }

        for( PAD* pad : fp->Pads() )
        {
            if( anchorPads.count( pad->GetNetCode() ) && !rails.count( pad->GetNetCode() ) )
            {
                earlier.push_back( fp );
                break;
            }
        }
    }

    // parts bound to a member, not to the anchor (the second capacitor of a filter, a LED behind its
    // resistor): next to the member's pad on their common net (not a rail), until none is left
    bool progress = true;

    while( progress && !indirect.empty() )
    {
        progress = false;

        for( auto it = indirect.begin(); it != indirect.end(); )
        {
            FOOTPRINT* part = *it;
            PAD*       target = nullptr;

            for( const std::vector<FOOTPRINT*>* hosts : { &done, &earlier } )
            {
                for( FOOTPRINT* host : *hosts )
                {
                    for( PAD* hostPad : host->Pads() )
                    {
                        if( hostPad->GetNetCode() <= 0 || rails.count( hostPad->GetNetCode() ) )
                            continue;

                        for( PAD* pad : part->Pads() )
                        {
                            if( pad->GetNetCode() == hostPad->GetNetCode()
                                && ( !target || netPads[hostPad->GetNetCode()] < netPads[target->GetNetCode()] ) )
                            {
                                target = hostPad;
                            }
                        }
                    }
                }

                if( target )
                    break;
            }

            if( !target )
            {
                ++it;
                continue;
            }

            aCommit.Modify( part );

            if( ( aAnchor->IsFlipped() != part->IsFlipped() ) != aOpposite )
                part->Flip( part->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );

            NEAR_OPTIONS options = aOptions;
            options.partPad.clear();
            options.partHeight = heightOf( part );

            const NEAR_PLACEMENT placement = placeNear( aBoard, part, target, options );

            if( placement.ok )
            {
                part->SetAttributes( part->GetAttributes() & ~FP_JUST_ADDED );
                aMoved.push_back( part->m_Uuid );
                done.push_back( part );

                nlohmann::json row = nearJson( placement, target );
                row["ref"] = str( part->GetReference() );
                row["x_mm"] = toMm( part->GetPosition().x );
                row["y_mm"] = toMm( part->GetPosition().y );
                row["rotation_deg"] = part->GetOrientation().AsDegrees();
                row["side"] = part->IsFlipped() ? "bottom" : "top";
                row["via"] = str( target->GetParentFootprint()->GetReference() );
                placed.push_back( row );
            }
            else
            {
                unplaced.push_back( { { "ref", str( part->GetReference() ) }, { "reason", placement.reason } } );
            }

            it = indirect.erase( it );
            progress = true;
        }
    }

    for( FOOTPRINT* part : indirect )
    {
        unplaced.push_back( { { "ref", str( part->GetReference() ) },
                              { "reason", "shares no net with " + str( aAnchor->GetReference() )
                                                  + " or a placed part of the block (rails aside)" } } );
    }

    aOut["anchor"] = str( aAnchor->GetReference() );
    aOut["placed"] = placed;
    aOut["unplaced"] = unplaced;

    // the block's outline: the bounding box of the anchor's and the placed parts' courtyards, both sides
    BOX2I          outline = courtyard( aAnchor ).BBox();
    SHAPE_POLY_SET front, back;

    for( FOOTPRINT* fp : done )
        outline.Merge( courtyard( fp ).BBox() );

    for( FOOTPRINT* fp : done )
        ( fp->IsFlipped() ? back : front ).BooleanAdd( courtyard( fp ) );

    ( aAnchor->IsFlipped() ? back : front ).BooleanAdd( courtyard( aAnchor ) );

    auto round2 = []( double aValue ) { return std::round( aValue * 100.0 ) / 100.0; };

    aOut["block"] = { { "bbox_mm", { round2( toMm( outline.GetLeft() ) ), round2( toMm( outline.GetTop() ) ),
                                     round2( toMm( outline.GetRight() ) ), round2( toMm( outline.GetBottom() ) ) } },
                      { "width_mm", round2( toMm( outline.GetWidth() ) ) },
                      { "height_mm", round2( toMm( outline.GetHeight() ) ) },
                      { "area_mm2", round2( areaMm2( double( outline.GetWidth() ) * outline.GetHeight() ) ) },
                      { "courtyards_top_mm2", round2( areaMm2( front.Area() ) ) },
                      { "courtyards_bottom_mm2", round2( areaMm2( back.Area() ) ) } };

    if( !laid.empty() )
        aOut["vias_laid"] = laid.size();

    if( !laid.empty() || std::any_of( done.begin(), done.end(), []( FOOTPRINT* fp ) { return fp->IsFlipped(); } ) )
        aOut["pours_stale"] = true;   // the planes give way to the bottom caps and their vias

    if( !farAway.empty() )
        aOut["decoupling_far"] = farAway;
}


struct SYMBOL_SPOT
{
    std::string sheet;
    double      x, y;
};


/// @brief Where the open schematic draws each symbol, by reference; none when no schematic is open
std::optional<std::map<std::string, SYMBOL_SPOT>> schematicSpots( KOPENAPI_CONTEXT& aCtx )
{
    std::optional<KOPENAPI_METHOD> list = KOPENAPI_REGISTRY::Get().Find( "sch_symbol_list" );

    if( !list )
        return std::nullopt;

    std::map<std::string, SYMBOL_SPOT> drawn;
    nlohmann::json                     query = { { "limit", 1000 } };

    while( true )
    {
        KOPENAPI_RESULT page = list->handler( aCtx, query );

        if( page.status != 200 )
            return std::nullopt;

        for( const nlohmann::json& row : page.body.value( "items", nlohmann::json::array() ) )
        {
            const std::string ref = row.value( "ref", std::string() );

            if( !drawn.count( ref ) )
                drawn[ref] = { row.value( "sheet", std::string() ), row.value( "x_mm", 0.0 ), row.value( "y_mm", 0.0 ) };
        }

        if( !page.body.contains( "next_cursor" ) || page.body["next_cursor"].is_null() )
            break;

        query["cursor"] = page.body["next_cursor"];
    }

    return drawn;
}


/// @brief Decoupling capacitors of the anchor guessed from the schematic: two-pad C parts across two
/// rails of the anchor, the ones drawn nearest to it on its sheet, as many per supply rail as the
/// anchor has supply pin groups on it (adjacent pins count once).  Appended to aParts; aNote says
/// what was taken.
std::optional<KOPENAPI_RESULT> addDecoupling( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, FOOTPRINT* aAnchor,
                                              std::vector<FOOTPRINT*>& aParts, bool aOverrideLocks, nlohmann::json& aNote )
{
    if( !KOPENAPI_REGISTRY::Get().Find( "sch_symbol_list" ) )
        return KOPENAPI_RESULT::Error( 503, "decoupling: the schematic editor module is not loaded" );

    const std::optional<std::map<std::string, SYMBOL_SPOT>> spots = schematicSpots( aCtx );

    if( !spots )
    {
        return KOPENAPI_RESULT::Error( 409, "decoupling: schematic: the schematic is not open (sch_open), or name the "
                                            "capacitors in parts, or decoupling: per_pin" );
    }

    const std::map<std::string, SYMBOL_SPOT>& drawn = *spots;
    auto                                      home = drawn.find( str( aAnchor->GetReference() ) );

    if( home == drawn.end() )
        return KOPENAPI_RESULT::Error( 422, "decoupling: " + str( aAnchor->GetReference() ) + " is not in the open schematic" );

    // the anchor's rails as the schematic sees them (power / ground nets), ground told apart
    std::set<int>                  rails, grounds;
    std::optional<KOPENAPI_METHOD> subcircuit = KOPENAPI_REGISTRY::Get().Find( "sch_subcircuit" );

    if( subcircuit )
    {
        KOPENAPI_RESULT block = subcircuit->handler( aCtx, { { "ref", str( aAnchor->GetReference() ) } } );

        if( block.status == 200 )
        {
            for( const nlohmann::json& rail : block.body.value( "rails", nlohmann::json::array() ) )
            {
                if( NETINFO_ITEM* net = KopenapiFindNet( aBoard, rail.value( "net", std::string() ) ) )
                {
                    rails.insert( net->GetNetCode() );

                    if( rail.value( "role", std::string() ) == "ground" )
                        grounds.insert( net->GetNetCode() );
                }
            }
        }
    }

    std::map<int, int> netPads;
    std::map<int, int> anchorGroups = groupsPerNet( aAnchor );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    struct RAIL_CAPACITOR
    {
        FOOTPRINT* part;
        int        supply;      // the rail of the two that is not ground
        double     distance;
    };

    std::vector<RAIL_CAPACITOR> candidates;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        const std::string ref = str( fp->GetReference() );

        if( fp == aAnchor || fp->Pads().size() != 2 || ref.empty() || ref[0] != 'C' || ( fp->IsLocked() && !aOverrideLocks )
            || std::find( aParts.begin(), aParts.end(), fp ) != aParts.end() )
        {
            continue;
        }

        const int a = fp->Pads()[0]->GetNetCode(), b = fp->Pads()[1]->GetNetCode();

        // across two rails of the anchor
        if( a == b || !anchorGroups.count( a ) || !anchorGroups.count( b ) || !rails.count( a ) || !rails.count( b ) )
            continue;

        auto at = drawn.find( ref );

        if( at == drawn.end() || at->second.sheet != home->second.sheet )
            continue;

        const int supply = grounds.count( a ) && !grounds.count( b ) ? b
                           : grounds.count( b ) && !grounds.count( a ) ? a
                           : netPads[a] <= netPads[b] ? a : b;

        candidates.push_back( { fp, supply,
                                std::hypot( at->second.x - home->second.x, at->second.y - home->second.y ) } );
    }

    std::stable_sort( candidates.begin(), candidates.end(),
                      []( const RAIL_CAPACITOR& x, const RAIL_CAPACITOR& y ) { return x.distance < y.distance; } );

    std::map<int, int> taken;
    nlohmann::json     chosen = nlohmann::json::array();

    for( const RAIL_CAPACITOR& c : candidates )
    {
        if( taken[c.supply] >= anchorGroups[c.supply] )
            continue;

        taken[c.supply]++;
        aParts.push_back( c.part );
        chosen.push_back( str( c.part->GetReference() ) );
    }

    aNote = { { "chosen", chosen }, { "candidates", candidates.size() },
              { "by", "capacitors across two rails of the anchor, nearest to it on its schematic sheet, one per supply pin "
                      "group (adjacent pins of a rail count once): a guess - check it, name them in parts, or decoupling: "
                      "per_pin" } };
    return std::nullopt;
}


/// @brief Decoupling capacitors by supply pin: every C part across a supply rail and a ground goes to
/// one part on that rail (more than two pads, connectors aside) - the one drawn nearest to it in the
/// open schematic (on the board when none is open) that still has a supply pin group (adjacent pins
/// of a rail count once) without one.  Capacitors of 1 uF and more are bulk: one per part and rail.
/// So a row of capacitors drawn for several ICs is split by count.  The anchor's share is appended
/// to aParts; aNote lists it.
void addDecouplingPerPin( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, FOOTPRINT* aAnchor, std::vector<FOOTPRINT*>& aParts,
                          bool aOverrideLocks, nlohmann::json& aNote )
{
    const std::set<int> rails = railNets( aBoard );
    std::set<int>       grounds;

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
    {
        if( net->GetNetCode() > 0
            && KopenapiNetRoleFromName( str( UnescapeString( net->GetNetname() ) ) ).value( "role", "" ) == "ground" )
        {
            grounds.insert( net->GetNetCode() );
        }
    }

    const std::optional<std::map<std::string, SYMBOL_SPOT>> drawn = schematicSpots( aCtx );

    // the parts on each supply rail with their pin groups on it
    std::map<int, std::vector<std::pair<FOOTPRINT*, int>>> users;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( fp->Pads().size() <= 2 || isConnector( fp ) )
            continue;

        for( const auto& [net, groups] : groupsPerNet( fp ) )
        {
            if( rails.count( net ) && !grounds.count( net ) )
                users[net].emplace_back( fp, groups );
        }
    }

    struct CAPACITOR
    {
        FOOTPRINT* part;
        int        supply;
        bool       bulk;
    };

    std::vector<CAPACITOR> caps;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        const std::string ref = str( fp->GetReference() );

        if( fp == aAnchor || fp->Pads().size() != 2 || ref.empty() || ref[0] != 'C' || ( fp->IsLocked() && !aOverrideLocks )
            || std::find( aParts.begin(), aParts.end(), fp ) != aParts.end() )
        {
            continue;
        }

        const int a = fp->Pads()[0]->GetNetCode(), b = fp->Pads()[1]->GetNetCode();
        const int supply = grounds.count( a ) && !grounds.count( b ) ? b : grounds.count( b ) && !grounds.count( a ) ? a : 0;

        if( supply <= 0 || !rails.count( supply ) || !users.count( supply ) )
            continue;

        caps.push_back( { fp, supply, faradsOf( fp ).value_or( 0.0 ) >= 1e-6 } );
    }

    auto padDistance = [&]( FOOTPRINT* aCap, FOOTPRINT* aUser, int aNet )
    {
        double nearest = std::numeric_limits<double>::max();

        for( PAD* pad : aUser->Pads() )
        {
            if( pad->GetNetCode() == aNet )
                nearest = std::min( nearest, toMm( ( pad->GetPosition() - aCap->GetPosition() ).EuclideanNorm() ) );
        }

        return nearest;
    };

    auto distance = [&]( const CAPACITOR& aCap, FOOTPRINT* aUser )
    {
        if( drawn )
        {
            auto c = drawn->find( str( aCap.part->GetReference() ) );
            auto u = drawn->find( str( aUser->GetReference() ) );

            if( c != drawn->end() && u != drawn->end() && c->second.sheet == u->second.sheet )
                return std::hypot( c->second.x - u->second.x, c->second.y - u->second.y );

            return 1e6 + padDistance( aCap.part, aUser, aCap.supply );
        }

        return padDistance( aCap.part, aUser, aCap.supply );
    };

    struct OPTION
    {
        double     distance;
        size_t     cap;
        FOOTPRINT* user;
    };

    std::vector<OPTION> options;

    for( size_t i = 0; i < caps.size(); ++i )
    {
        for( const auto& [user, groups] : users[caps[i].supply] )
            options.push_back( { distance( caps[i], user ), i, user } );
    }

    std::stable_sort( options.begin(), options.end(), []( const OPTION& x, const OPTION& y ) { return x.distance < y.distance; } );

    std::map<std::pair<FOOTPRINT*, int>, int> quota, bulkQuota;

    for( const auto& [net, list] : users )
    {
        for( const auto& [user, groups] : list )
        {
            quota[{ user, net }] = groups;
            bulkQuota[{ user, net }] = 1;
        }
    }

    std::vector<FOOTPRINT*> owner( caps.size(), nullptr );

    for( const OPTION& o : options )
    {
        if( owner[o.cap] )
            continue;

        int& left = ( caps[o.cap].bulk ? bulkQuota : quota )[{ o.user, caps[o.cap].supply }];

        if( left > 0 )
        {
            left--;
            owner[o.cap] = o.user;
        }
    }

    nlohmann::json chosen = nlohmann::json::array();
    nlohmann::json spare = nlohmann::json::array();
    const auto     anchorGroups = groupsPerNet( aAnchor );

    for( size_t i = 0; i < caps.size(); ++i )
    {
        const std::string ref = str( caps[i].part->GetReference() );

        if( owner[i] == aAnchor )
        {
            aParts.push_back( caps[i].part );
            chosen.push_back( { { "ref", ref }, { "rail", str( UnescapeString( aBoard->FindNet( caps[i].supply )->GetNetname() ) ) },
                                { "value", str( caps[i].part->GetValue() ) }, { "bulk", caps[i].bulk } } );
        }
        else if( !owner[i] && anchorGroups.count( caps[i].supply ) )
        {
            spare.push_back( ref );
        }
    }

    nlohmann::json groups = nlohmann::json::object();

    for( const auto& [net, count] : anchorGroups )
    {
        if( rails.count( net ) && !grounds.count( net ) )
            groups[str( UnescapeString( aBoard->FindNet( net )->GetNetname() ) )] = count;
    }

    aNote = { { "chosen", chosen }, { "supply_pin_groups", groups }, { "by", drawn ? "schematic distance" : "board distance" },
              { "rule", "one capacitor under 1 uF per supply pin group (adjacent pins of a rail count once) and one bulk "
                        "capacitor per rail, each capacitor to the part on its rail nearest to it with a share left" } };

    if( !spare.empty() )
        aNote["unassigned"] = spare;
}


/// @brief The parts of a group request: named ones, or (none named) every unlocked part with a pad on a
/// small net of the anchor (at most max_net_pads pads on the board: a crystal and its capacitors,
/// pull-ups, a feedback divider - not everything on the rails)
std::optional<KOPENAPI_RESULT> groupParts( KOPENAPI_CONTEXT& aCtx, BOARD* aBoard, const nlohmann::json& aGroup,
                                           FOOTPRINT*& aAnchor, std::vector<FOOTPRINT*>& aParts, bool aOverrideLocks )
{
    aAnchor = aGroup.is_object() ? findFootprint( aBoard, aGroup.value( "anchor", std::string() ) ) : nullptr;

    if( !aAnchor )
        return KOPENAPI_RESULT::Error( 404, "anchor: the reference of the block's main part" );

    if( aGroup.value( "from_schematic", false ) && !aGroup.contains( "parts" ) )
    {
        std::optional<KOPENAPI_METHOD> subcircuit = KOPENAPI_REGISTRY::Get().Find( "sch_subcircuit" );

        if( !subcircuit )
            return KOPENAPI_RESULT::Error( 503, "from_schematic: the schematic editor module is not loaded" );

        nlohmann::json query = { { "ref", str( aAnchor->GetReference() ) } };

        if( aGroup.contains( "depth" ) )
            query["depth"] = aGroup["depth"];

        KOPENAPI_RESULT block = subcircuit->handler( aCtx, query );

        if( block.status != 200 )
        {
            return KOPENAPI_RESULT::Error( block.status == 409 ? 409 : 422,
                                           "from_schematic: sch_subcircuit failed for " + str( aAnchor->GetReference() ) + " ("
                                                   + block.body.dump().substr( 0, 200 ) + "); open the schematic (sch_open) or name the parts" );
        }

        for( const nlohmann::json& part : block.body.value( "parts", nlohmann::json::array() ) )
        {
            if( part.value( "kind", std::string() ) != "member" )
                continue;

            FOOTPRINT* fp = findFootprint( aBoard, part.value( "ref", std::string() ) );

            if( fp && fp != aAnchor && ( !fp->IsLocked() || aOverrideLocks ) && !isConnector( fp ) )
                aParts.push_back( fp );
        }

        return std::nullopt;
    }

    if( aGroup.contains( "parts" ) )
    {
        if( !aGroup["parts"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "parts: references" );

        for( const nlohmann::json& ref : aGroup["parts"] )
        {
            FOOTPRINT* part = ref.is_string() ? findFootprint( aBoard, ref.get<std::string>() ) : nullptr;

            if( !part || part == aAnchor )
                return KOPENAPI_RESULT::Error( 404, "parts: no such footprint (or the anchor itself): " + ref.dump() );

            if( part->IsLocked() && !aOverrideLocks )
                return KOPENAPI_RESULT::Error( 409, str( part->GetReference() ) + " is locked (override_locks: true)" );

            aParts.push_back( part );
        }

        return std::nullopt;
    }

    const int          limit = std::clamp( aGroup.value( "max_net_pads", 4 ), 2, 64 );
    std::map<int, int> netPads;

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
            netPads[pad->GetNetCode()]++;
    }

    std::set<int> small;

    for( PAD* pad : aAnchor->Pads() )
    {
        if( pad->GetNetCode() > 0 && netPads[pad->GetNetCode()] <= limit )
            small.insert( pad->GetNetCode() );
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        // connectors belong to the board edge, not to a block
        if( fp == aAnchor || fp->IsLocked() || isConnector( fp ) )
            continue;

        for( PAD* pad : fp->Pads() )
        {
            if( small.count( pad->GetNetCode() ) )
            {
                aParts.push_back( fp );
                break;
            }
        }
    }

    return std::nullopt;
}

/// @brief orientation of a block's anchor: keep (nothing), auto (chosen by routability, the turns set
/// later) or a count of quarter turns clockwise; a locked anchor turns only with override_locks
std::optional<KOPENAPI_RESULT> anchorTurns( const nlohmann::json& aSpec, FOOTPRINT* aAnchor, bool aOverrideLocks,
                                            std::optional<int>& aTurns )
{
    const nlohmann::json o = aSpec.value( "orientation", nlohmann::json( "keep" ) );

    if( o == "keep" )
        return std::nullopt;

    if( o != "auto" && !( o.is_number_integer() && o.get<int>() >= 0 && o.get<int>() <= 3 ) )
        return KOPENAPI_RESULT::Error( 400, "orientation: keep, auto (the quarter turn of the anchor the block routes best "
                                            "in) or 0..3 quarter turns clockwise" );

    if( o.is_number_integer() )
        aTurns = o.get<int>();

    if( aAnchor->IsLocked() && !aOverrideLocks && o != 0 )
        return KOPENAPI_RESULT::Error( 409, str( aAnchor->GetReference() ) + " is locked: orientation turns it only with "
                                                                              "override_locks: true" );

    return std::nullopt;
}


/// @brief Turn the anchor by quarter turns clockwise (board view) about its courtyard's centre
void turnAnchor( BOARD_COMMIT& aCommit, FOOTPRINT* aAnchor, int aTurns )
{
    if( aTurns % 4 == 0 )
        return;

    aCommit.Modify( aAnchor );
    aAnchor->Rotate( courtyard( aAnchor ).BBox().Centre(), EDA_ANGLE( -90.0 * aTurns, DEGREES_T ) );
}


} // namespace


static KOPENAPI_RESULT h_pcb_footprint_move( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();

    if( !aArgs.contains( "moves" ) || !aArgs["moves"].is_array() || aArgs["moves"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'moves': [{ref, x_mm, y_mm, rotation_deg?, side?}]" );

    const std::string reportScope = aArgs.value( "report", std::string( "moved" ) );
    const std::string onCopperMode = aArgs.value( "on_copper", std::string( "report" ) );

    if( onCopperMode != "report" && onCopperMode != "refuse" && onCopperMode != "rip" )
        return KOPENAPI_RESULT::Error( 400, "on_copper: report, refuse or rip" );

    if( reportScope != "moved" && reportScope != "full" && reportScope != "none" )
        return KOPENAPI_RESULT::Error( 400, "report: moved, full or none" );

    // Validate everything first: all or nothing
    std::vector<std::pair<FOOTPRINT*, nlohmann::json>> plan;

    for( const nlohmann::json& m : aArgs["moves"] )
    {
        FOOTPRINT* fp = m.is_object() ? findFootprint( board, m.value( "ref", std::string() ) ) : nullptr;

        if( !fp )
            return KOPENAPI_RESULT::Error( 404, "footprint not found: " + m.dump() );

        if( m.contains( "side" ) && m["side"] != "top" && m["side"] != "bottom" )
            return KOPENAPI_RESULT::Error( 400, "side: top or bottom" );

        if( m.contains( "facing" ) && m["facing"] != "left" && m["facing"] != "right" && m["facing"] != "up" && m["facing"] != "down" )
            return KOPENAPI_RESULT::Error( 400, "facing: left, right, up or down" );

        if( fp->IsLocked() && !aArgs.value( "override_locks", false ) )
            return KOPENAPI_RESULT::Error( 409, str( fp->GetReference() ) + " is locked (override_locks: true)" );

        if( m.contains( "with" ) )
        {
            if( !m["with"].is_array() )
                return KOPENAPI_RESULT::Error( 400, "with: references of the parts that keep their place relative to this one" );

            for( const nlohmann::json& ref : m["with"] )
            {
                FOOTPRINT* carried = ref.is_string() ? findFootprint( board, ref.get<std::string>() ) : nullptr;

                if( !carried || carried == fp )
                    return KOPENAPI_RESULT::Error( 404, "with: no such footprint (or the moved one itself): " + ref.dump() );

                if( carried->IsLocked() && !aArgs.value( "override_locks", false ) )
                    return KOPENAPI_RESULT::Error( 409, str( carried->GetReference() ) + " is locked (override_locks: true)" );

                for( const nlohmann::json& other : aArgs["moves"] )
                {
                    if( other.is_object() && other.value( "ref", std::string() ) == ref.get<std::string>() )
                        return KOPENAPI_RESULT::Error( 400, ref.get<std::string>() + " is carried (with) and moved on its own: one of the two" );
                }
            }

            if( m.contains( "side" ) && ( m["side"] == "bottom" ) != fp->IsFlipped() )
                return KOPENAPI_RESULT::Error( 400, "with: not together with a change of side (move the parts, then place the block again)" );
        }

        nlohmann::json move = m;

        if( m.contains( "near" ) )
        {
            PAD*         target = m["near"].is_string() ? findPadByName( board, m["near"].get<std::string>() ) : nullptr;
            NEAR_OPTIONS options;

            if( !target )
                return KOPENAPI_RESULT::Error( 404, "near: a pad as REF.PAD (pcb_footprint_get lists them): " + m["near"].dump() );

            if( target->GetParentFootprint() == fp )
                return KOPENAPI_RESULT::Error( 422, "near: a pad of another footprint" );

            if( std::optional<std::string> wrong = nearOptions( m, options ) )
                return KOPENAPI_RESULT::Error( 400, *wrong );

            options.hostRingMm = std::max( 0.0, options.hostRingMm );

            if( m.contains( "x_mm" ) || m.contains( "y_mm" ) || m.contains( "edge" ) )
                return KOPENAPI_RESULT::Error( 400, "near places the part itself: leave out x_mm / y_mm / edge" );
        }

        // edge: on the board outline, facing out; the coordinate along the edge stays the caller's
        if( m.contains( "edge" ) )
        {
            static const std::map<std::string, std::string> facing = { { "left", "left" }, { "right", "right" },
                                                                        { "top", "up" }, { "bottom", "down" } };

            if( !m["edge"].is_string() || !facing.count( m["edge"].get<std::string>() ) )
                return KOPENAPI_RESULT::Error( 400, "edge: left, right, top or bottom" );

            const BOX2I outline = board->GetBoardEdgesBoundingBox();

            if( outline.GetWidth() <= 0 )
                return KOPENAPI_RESULT::Error( 409, "edge needs a board outline (pcb_outline_set)" );

            const std::string e = m["edge"];
            move["facing"] = facing.at( e );
            move["anchor"] = "panel_edge";

            if( e == "left" || e == "right" )
                move["x_mm"] = toMm( e == "left" ? outline.GetLeft() : outline.GetRight() );
            else
                move["y_mm"] = toMm( e == "top" ? outline.GetTop() : outline.GetBottom() );
        }

        if( ( move.contains( "facing" ) || move.value( "anchor", std::string() ) == "panel_edge" ) && !panelEdge( fp ) )
        {
            return KOPENAPI_RESULT::Error( 409, str( fp->GetReference() ) + " (" + str( fp->GetFPID().Format() )
                                                        + "): no panel edge known - neither a marker in the footprint nor "
                                                          "an inferable front; place it with rotation_deg and check a render" );
        }

        plan.emplace_back( fp, move );
    }

    const bool ripAttached = aArgs.value( "rip_attached", false );
    std::vector<ATTACHED_END> attached;

    {
        std::set<const FOOTPRINT*> moving;

        for( auto& [fp, m] : plan )
        {
            moving.insert( fp );

            for( const nlohmann::json& ref : m.value( "with", nlohmann::json::array() ) )
                moving.insert( findFootprint( board, ref.get<std::string>() ) );
        }

        moving.erase( nullptr );
        attached = attachedEnds( board, moving );
    }

    BOARD_COMMIT      commit( context->GetToolManager() );
    std::vector<KIID> moved;
    std::map<FOOTPRINT*, nlohmann::json> nearPlaced;

    // a flip or rotation can hide the front the plan found: nothing is moved then
    auto noEdgeAfterTurn = [&]( FOOTPRINT* aFp )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Error( 409, str( aFp->GetReference() ) + ": no panel edge known after the flip / "
                                                                         "rotation; place it with rotation_deg" );
    };

    // parts carried with a moved one: the same turn about its origin and the same shift
    std::map<FOOTPRINT*, nlohmann::json> carriedWith;

    auto carry = [&]( FOOTPRINT* aFp, const nlohmann::json& aMove, const VECTOR2I& aFrom, const EDA_ANGLE& aAngleBefore )
    {
        if( !aMove.contains( "with" ) )
            return;

        const EDA_ANGLE turn = aFp->GetOrientation() - aAngleBefore;
        const VECTOR2I  shift = aFp->GetPosition() - aFrom;
        nlohmann::json  rows = nlohmann::json::array();

        for( const nlohmann::json& ref : aMove["with"] )
        {
            FOOTPRINT* carried = findFootprint( board, ref.get<std::string>() );

            commit.Modify( carried );

            if( !turn.IsZero() )
                carried->Rotate( aFrom, turn );

            carried->Move( shift );
            carried->SetAttributes( carried->GetAttributes() & ~FP_JUST_ADDED );
            moved.push_back( carried->m_Uuid );
            rows.push_back( { { "ref", str( carried->GetReference() ) }, { "x_mm", toMm( carried->GetPosition().x ) },
                              { "y_mm", toMm( carried->GetPosition().y ) },
                              { "rotation_deg", carried->GetOrientation().AsDegrees() } } );
        }

        carriedWith[aFp] = rows;
    };

    for( auto& [fp, m] : plan )
    {
        commit.Modify( fp );

        const VECTOR2I  posBefore = fp->GetPosition();
        const EDA_ANGLE angleBefore = fp->GetOrientation();

        if( m.contains( "side" ) && ( m["side"] == "bottom" ) != fp->IsFlipped() )
            fp->Flip( fp->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );   // as the editor flips a part

        if( m.contains( "rotation_deg" ) && m["rotation_deg"].is_number() )
            fp->SetOrientation( EDA_ANGLE( m["rotation_deg"].get<double>(), DEGREES_T ) );

        // facing: turn so the front (beyond the footprint's panel-edge line) points that way
        if( m.contains( "facing" ) )
        {
            std::optional<PANEL_EDGE> edge = panelEdge( fp );

            if( !edge )
                return noEdgeAfterTurn( fp );

            static const std::map<std::string, double> want = { { "right", 0 }, { "down", 90 }, { "left", 180 }, { "up", 270 } };
            const double now = std::atan2( edge->second.y, edge->second.x ) * 180.0 / M_PI;
            const double turn = now - want.at( m["facing"].get<std::string>() );   // CCW on screen lowers the angle
            fp->SetOrientation( fp->GetOrientation() + EDA_ANGLE( turn, DEGREES_T ) );
        }

        // near: next to a pad of another part, as close as the courtyards allow; all or nothing
        if( m.contains( "near" ) )
        {
            PAD*         target = findPadByName( board, m["near"].get<std::string>() );
            NEAR_OPTIONS options;

            nearOptions( m, options );
            options.hostRingMm = std::max( 0.0, options.hostRingMm );

            if( const auto heights = heightsIfRoomUnder( board, { fp } ); heights.count( fp ) )
                options.partHeight = heights.at( fp );

            const NEAR_PLACEMENT placement = placeNear( board, fp, target, options );

            if( !placement.ok )
            {
                commit.Revert();
                return KOPENAPI_RESULT::Error( 422, str( fp->GetReference() ) + ": " + placement.reason + "; nothing was moved" );
            }

            nearPlaced[fp] = nearJson( placement, target );
            fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
            moved.push_back( fp->m_Uuid );
            carry( fp, m, posBefore, angleBefore );
            continue;
        }

        VECTOR2I pos( m.contains( "x_mm" ) ? toIU( m["x_mm"].get<double>() ) : fp->GetPosition().x,
                      m.contains( "y_mm" ) ? toIU( m["y_mm"].get<double>() ) : fp->GetPosition().y );

        // anchor center: the position names the middle of the courtyard (connectors' origin is
        // often a pin far from their body)
        if( m.value( "anchor", std::string( "origin" ) ) == "center" )
            pos -= courtyard( fp ).BBox().Centre() - fp->GetPosition();
        else if( m.value( "anchor", std::string( "origin" ) ) == "panel_edge" )
        {
            std::optional<PANEL_EDGE> edge = panelEdge( fp );

            if( !edge )
                return noEdgeAfterTurn( fp );

            pos -= edge->first.Center() - fp->GetPosition();

            // edge_offset_mm: along the facing direction, + out past the edge, - recessed
            const double offset = m.value( "edge_offset_mm", 0.0 );
            pos += VECTOR2I( KiROUND( edge->second.x * toIU( offset ) ), KiROUND( edge->second.y * toIU( offset ) ) );
        }

        fp->SetPosition( pos );
        fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );   // placed now: connectivity counts it
        moved.push_back( fp->m_Uuid );
        carry( fp, m, posBefore, angleBefore );
    }

    std::set<const FOOTPRINT*> movedFps;

    for( auto& [fp, m] : plan )
    {
        movedFps.insert( fp );

        for( const nlohmann::json& ref : m.value( "with", nlohmann::json::array() ) )
            movedFps.insert( findFootprint( board, ref.get<std::string>() ) );
    }

    movedFps.erase( nullptr );

    nlohmann::json        copperOut = nlohmann::json::object();
    std::map<int, size_t> rippedNets;

    if( onCopper( board, commit, movedFps, onCopperMode, copperOut, rippedNets ) )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Ok( copperOut );
    }

    // the copper that stayed where the moved pads were: now on another net's pad, or on none
    nlohmann::json shortsCreated, danglingAtPads;
    attachedAfterMove( board, attached, shortsCreated, danglingAtPads );

    nlohmann::json ripAttachedOut = nlohmann::json::array();
    nlohmann::json keptLocked = nlohmann::json::array();

    if( ripAttached )
    {
        std::set<std::string> gone;

        for( const nlohmann::json& r : copperOut.value( "ripped", nlohmann::json::array() ) )
            gone.insert( r["uuid"].get<std::string>() );

        for( const ATTACHED_END& e : attached )
        {
            const std::string id = str( e.item->m_Uuid.AsString() );

            if( !gone.insert( id ).second )
                continue;

            nlohmann::json row = { { "uuid", id },
                                   { "kind", e.item->Type() == PCB_VIA_T ? "via" : e.item->Type() == PCB_ARC_T ? "arc" : "track" },
                                   { "net", str( UnescapeString( e.item->GetNetname() ) ) },
                                   { "was_on", e.pad } };

            if( e.item->IsLocked() )
            {
                keptLocked.push_back( row );
                continue;
            }

            ripAttachedOut.push_back( row );
            commit.Remove( e.item );
        }
    }

    const nlohmann::json conflictsBefore = copperConflicts( board, movedFps );

    commit.Push( _( "Move footprints (API)" ) );
    refreshGui( aCtx, board );
    glow( aCtx, moved );

    // the placement check for the footprints just moved (the whole board: report full; none: skip)
    nlohmann::json report = nlohmann::json::object();

    if( reportScope == "moved" )
        report = placementReport( board, &movedFps );
    else if( reportScope == "full" )
        report = placementReport( board );

    if( copperOut.contains( "ripped" ) )
    {
        report["ripped"] = copperOut["ripped"];
        report["opened"] = openedByRip( board, rippedNets );
    }

    // the copper check of the moved parts holds whatever the report scope
    report["copper_conflicts"] = mergeConflicts( board, conflictsBefore, copperConflicts( board, movedFps ) );

    if( !report["copper_conflicts"].empty() && report.contains( "ok" ) )
        report["ok"] = false;

    // copper left where the moved pads were (rip_attached: taken out in the same step)
    if( ripAttached )
    {
        report["ripped_attached"] = ripAttachedOut;

        if( !keptLocked.empty() )
            report["attached_locked"] = keptLocked;

        auto stillThere = [&]( const nlohmann::json& aRows )
        {
            nlohmann::json kept = nlohmann::json::array();

            for( const nlohmann::json& r : aRows )
            {
                if( board->ResolveItem( KIID( r["item"].get<std::string>() ), true ) )
                    kept.push_back( r );
            }

            return kept;
        };

        shortsCreated = stillThere( shortsCreated );
        danglingAtPads = stillThere( danglingAtPads );
    }

    report["shorts_created"] = shortsCreated;
    report["dangling_at_pads"] = danglingAtPads;

    if( !shortsCreated.empty() )
    {
        report["ok"] = false;
        report["hint"] = "copper of the old pad places now touches pads of other nets: rollback, or move again with "
                         "rip_attached: true (or pcb_tracks_delete those items)";
    }

    report["report"] = reportScope;
    nlohmann::json placed = nlohmann::json::array();

    for( auto& [fp, m] : plan )
    {
        nlohmann::json row = { { "ref", str( fp->GetReference() ) },
                               { "x_mm", toMm( fp->GetPosition().x ) },
                               { "y_mm", toMm( fp->GetPosition().y ) },
                               { "rotation_deg", fp->GetOrientation().AsDegrees() },
                               { "side", fp->IsFlipped() ? "bottom" : "top" } };

        if( nearPlaced.count( fp ) )
            row["near"] = nearPlaced[fp];

        if( carriedWith.count( fp ) )
            row["with"] = carriedWith[fp];

        if( m.contains( "facing" ) )
        {
            const nlohmann::json edge = KopenapiPanelEdgeJson( fp );
            row["panel_edge"] = { { "source", edge["source"] }, { "confidence", edge["confidence"] } };

            if( edge["confidence"] == "low" || edge["confidence"] == "medium" )
                row["check"] = "front inferred from the body (" + edge["confidence"].get<std::string>()
                               + " confidence): verify with pcb_render_3d";
        }

        placed.push_back( row );
    }

    report["moved"] = placed;
    return KOPENAPI_RESULT::Ok( report );
}


static KOPENAPI_RESULT h_pcb_placement_check( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    return KOPENAPI_RESULT::Ok( placementReport( context->GetBoard() ) );
}


// entries per list of pcb_place_auto's answer by default
constexpr size_t PLACE_AUTO_LIST_CAP = 50;


static KOPENAPI_RESULT h_pcb_place_auto( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*       board = context->GetBoard();
    BOARD_COMMIT commit( context->GetToolManager() );

    const std::string onCopperMode = aArgs.value( "on_copper", std::string( "report" ) );

    if( onCopperMode != "report" && onCopperMode != "refuse" && onCopperMode != "rip" )
        return KOPENAPI_RESULT::Error( 400, "on_copper: report, refuse or rip" );

    // where the unlocked parts were: on_copper refuse puts the conflicting ones back
    struct WAS
    {
        VECTOR2I     pos;
        EDA_ANGLE    angle;
        PCB_LAYER_ID side;
    };

    std::map<FOOTPRINT*, WAS> was;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( !fp->IsLocked() )
            was[fp] = { fp->GetPosition(), fp->GetOrientation(), fp->GetLayer() };
    }
    nlohmann::json notes = nlohmann::json::array();

    std::vector<FOOTPRINT*> all;

    for( FOOTPRINT* fp : board->Footprints() )
    {
        if( !fp->IsLocked() )
            all.push_back( fp );
    }

    if( all.empty() )
        return KOPENAPI_RESULT::Error( 409, "no unlocked footprints to place" );

    // 1. The outline: kept, given, or sized from the parts (courtyard area x fill factor)
    BOX2I edges = board->GetBoardEdgesBoundingBox();
    const bool resize = aArgs.contains( "width_mm" ) || edges.GetWidth() == 0 || edges.GetHeight() == 0;

    if( resize )
    {
        for( BOARD_ITEM* item : board->Drawings() )
        {
            if( item->GetLayer() == Edge_Cuts && item->IsLocked() )
                return KOPENAPI_RESULT::Error( 409, "the board outline is locked: leave out width_mm / height_mm, or pcb_unlock it" );
        }

        double w = aArgs.value( "width_mm", 0.0 ), h = aArgs.value( "height_mm", 0.0 );

        if( w <= 0 || h <= 0 )
        {
            double area = 0;

            for( FOOTPRINT* fp : all )
                area += areaMm2( courtyard( fp ).Area() );

            const double aspect = aArgs.value( "aspect", 1.5 );
            const double fill = aArgs.value( "density", 0.35 );   // share of the board the parts cover
            w = std::ceil( std::sqrt( area / fill * aspect ) );
            h = std::ceil( area / fill / w );
            notes.push_back( "outline sized from the parts: " + std::to_string( (int) w ) + " x " + std::to_string( (int) h )
                             + " mm (courtyards " + std::to_string( (int) std::round( area ) ) + " mm2, density "
                             + std::to_string( fill ).substr( 0, 4 ) + ")" );
        }

        for( BOARD_ITEM* item : std::vector<BOARD_ITEM*>( board->Drawings().begin(), board->Drawings().end() ) )
        {
            if( item->GetLayer() == Edge_Cuts )
                commit.Remove( item );
        }

        const VECTOR2I origin( toIU( aArgs.value( "x_mm", 20.0 ) ), toIU( aArgs.value( "y_mm", 20.0 ) ) );
        auto*          outline = new PCB_SHAPE( board, SHAPE_T::RECTANGLE );
        outline->SetLayer( Edge_Cuts );
        outline->SetStroke( STROKE_PARAMS( toIU( 0.1 ), LINE_STYLE::SOLID ) );
        outline->SetStart( origin );
        outline->SetEnd( origin + VECTOR2I( toIU( w ), toIU( h ) ) );
        commit.Add( outline );
        commit.Push( _( "Board outline (API)" ) );
        KopenapiKeepUiAlive();
        commit = BOARD_COMMIT( context->GetToolManager() );
        edges = board->GetBoardEdgesBoundingBox();

        // a frameless board's page follows its outline (as pcb_outline_set does)
        if( board->GetPageSettings().GetType() == PAGE_SIZE_TYPE::User )
            KopenapiFitPage( board, edges );
    }

    // 2. Connectors along the edges: left, right, top, bottom in turn, courtyard inset 0.5 mm; a
    //    connector whose mating side is known turns to face out over its edge
    std::vector<FOOTPRINT*> rest;
    std::vector<KIID>       placed;
    const int               inset = toIU( 0.5 );
    int                     side = 0;
    std::map<int, int>      along;   // next free offset along each edge

    for( FOOTPRINT* fp : all )
    {
        if( !aArgs.value( "connectors_to_edge", true ) || !isConnector( fp ) )
        {
            rest.push_back( fp );
            continue;
        }

        commit.Modify( fp );
        const int s = side++ % 4;

        if( std::optional<PANEL_EDGE> edge = panelEdge( fp ) )
        {
            static const double want[4] = { 180, 0, 270, 90 };   // left, right, up, down: facing out
            const double        now = std::atan2( edge->second.y, edge->second.x ) * 180.0 / M_PI;
            fp->SetOrientation( fp->GetOrientation() + EDA_ANGLE( now - want[s], DEGREES_T ) );
        }

        const BOX2I    box = courtyard( fp ).BBox();
        const VECTOR2I toOrigin = fp->GetPosition() - box.GetOrigin();
        int&           offset = along[s];
        VECTOR2I       corner;

        // on the top / bottom edges the connectors start from the middle outwards
        const int middleX = edges.Centre().x - (int) box.GetWidth() / 2;

        if( s == 0 )
            corner = VECTOR2I( edges.GetLeft() + inset, edges.Centre().y - (int) box.GetHeight() / 2 + offset );
        else if( s == 1 )
            corner = VECTOR2I( edges.GetRight() - inset - (int) box.GetWidth(), edges.Centre().y - (int) box.GetHeight() / 2 + offset );
        else if( s == 2 )
            corner = VECTOR2I( middleX + offset, edges.GetTop() + inset );
        else
            corner = VECTOR2I( middleX + offset, edges.GetBottom() - inset - (int) box.GetHeight() );

        offset += ( s < 2 ? box.GetHeight() : box.GetWidth() ) + toIU( 1.0 );
        fp->SetPosition( corner + toOrigin );

        // a connector with a known front: its panel line on the edge (the body inside, the mating face out)
        if( std::optional<PANEL_EDGE> edge = panelEdge( fp ) )
        {
            const VECTOR2I line = edge->first.Center();
            VECTOR2I       shift( 0, 0 );

            if( s == 0 )
                shift.x = edges.GetLeft() - line.x;
            else if( s == 1 )
                shift.x = edges.GetRight() - line.x;
            else if( s == 2 )
                shift.y = edges.GetTop() - line.y;
            else
                shift.y = edges.GetBottom() - line.y;

            fp->Move( shift );
        }

        fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
        placed.push_back( fp->m_Uuid );
    }

    if( !placed.empty() )
    {
        commit.Push( _( "Connectors to the edges (API)" ) );
        KopenapiKeepUiAlive();
        commit = BOARD_COMMIT( context->GetToolManager() );
    }

    // 2b. The main parts of the given groups spread over the board (largest nearest the middle),
    //     their parts left to the groups below; the autoplacer gets the others
    std::set<FOOTPRINT*> anchors, members;

    for( const nlohmann::json& group : aArgs.value( "groups", nlohmann::json::array() ) )
    {
        FOOTPRINT*              anchor = nullptr;
        std::vector<FOOTPRINT*> parts;

        if( groupParts( aCtx, board, group, anchor, parts, false ) )
            continue;

        if( !isConnector( anchor ) && !anchor->IsLocked() )
            anchors.insert( anchor );

        for( FOOTPRINT* part : parts )
            members.insert( part );
    }

    if( !anchors.empty() )
    {
        std::vector<FOOTPRINT*> order( anchors.begin(), anchors.end() );

        std::sort( order.begin(), order.end(),
                   [&]( FOOTPRINT* x, FOOTPRINT* y )
                   {
                       const double ax = courtyard( x ).Area(), ay = courtyard( y ).Area();
                       return ax != ay ? ax > ay : str( x->GetReference() ) < str( y->GetReference() );
                   } );

        BOX2I inner = edges;
        inner.Inflate( -toIU( 3.0 ) );

        // the main parts stay out of the strips the connectors took along the edges
        int left = inner.GetLeft(), right = inner.GetRight(), top = inner.GetTop(), bottom = inner.GetBottom();

        for( FOOTPRINT* fp : all )
        {
            if( !isConnector( fp ) || !aArgs.value( "connectors_to_edge", true ) )
                continue;

            const BOX2I c = courtyard( fp ).BBox();
            const int   room = toIU( 2.0 ), nearby = toIU( 1.0 );

            if( c.GetLeft() <= edges.GetLeft() + nearby )
                left = std::max( left, c.GetRight() + room );
            else if( c.GetRight() >= edges.GetRight() - nearby )
                right = std::min( right, c.GetLeft() - room );
            else if( c.GetTop() <= edges.GetTop() + nearby )
                top = std::max( top, c.GetBottom() + room );
            else if( c.GetBottom() >= edges.GetBottom() - nearby )
                bottom = std::min( bottom, c.GetTop() - room );
        }

        // a board too small for the strips keeps the plain inner box
        if( right - left > toIU( 5.0 ) && bottom - top > toIU( 5.0 ) )
        {
            inner.SetOrigin( left, top );
            inner.SetEnd( right, bottom );
        }

        const int cols = std::max( 1, (int) std::ceil( std::sqrt( order.size() * double( inner.GetWidth() ) / std::max<double>( 1.0, inner.GetHeight() ) ) ) );
        const int rows = std::max( 1, (int) std::ceil( order.size() / double( cols ) ) );

        std::vector<VECTOR2I> cells;

        for( int r = 0; r < rows; ++r )
        {
            for( int c = 0; c < cols; ++c )
                cells.emplace_back( inner.GetLeft() + (int) ( ( c + 0.5 ) * inner.GetWidth() / cols ),
                                    inner.GetTop() + (int) ( ( r + 0.5 ) * inner.GetHeight() / rows ) );
        }

        std::stable_sort( cells.begin(), cells.end(),
                          [&]( const VECTOR2I& x, const VECTOR2I& y )
                          { return ( x - inner.Centre() ).EuclideanNorm() < ( y - inner.Centre() ).EuclideanNorm(); } );

        for( size_t i = 0; i < order.size(); ++i )
        {
            FOOTPRINT* fp = order[i];
            commit.Modify( fp );
            fp->Move( cells[i] - courtyard( fp ).BBox().Centre() );
            fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
            placed.push_back( fp->m_Uuid );
        }

        commit.Push( _( "Main parts spread (API)" ) );
        KopenapiKeepUiAlive();
        commit = BOARD_COMMIT( context->GetToolManager() );

        rest.erase( std::remove_if( rest.begin(), rest.end(),
                                    [&]( FOOTPRINT* fp ) { return anchors.count( fp ) || members.count( fp ); } ),
                    rest.end() );
    }

    // 3. The rest: KiCad's autoplacer (matrix inside the outline, ratsnest cost); footprints a
    //    netlist update just added count for connectivity only once placed
    for( FOOTPRINT* fp : rest )
    {
        if( fp->GetAttributes() & FP_JUST_ADDED )
        {
            commit.Modify( fp );
            fp->SetAttributes( fp->GetAttributes() & ~FP_JUST_ADDED );
        }
    }

    board->BuildConnectivity();
    KopenapiKeepUiAlive();

    // the placer's own connectivity of every footprint: built from the board, which it only reads
    std::optional<AR_AUTOPLACER> placer;
    KopenapiCompute( [&]() { placer.emplace( board ); } );
    AR_AUTOPLACER& autoplacer = *placer;

    KOPENAPI_KEEPALIVE_REPORTER reporter;
    autoplacer.SetProgressReporter( &reporter );

    // The search for each part's place only reads the board: off the GUI thread, readers alongside.
    // Each part moves here; the moves go to the board in short commits (one undo step in the end),
    // so the editor shows them as they come and readers see a consistent board between parts
    autoplacer.SetComputeRunner( []( const std::function<void()>& aSearch ) { KopenapiCompute( aSearch ); } );

    using CLOCK = std::chrono::steady_clock;
    constexpr auto BATCH = std::chrono::milliseconds( 30 );
    CLOCK::time_point lastPush = CLOCK::now();
    bool              pushed = false;
    int               batches = 0;

    auto pushBatch = [&]()
    {
        if( commit.Empty() )
            return;

        commit.Push( _( "Autoplace footprints (API)" ), pushed ? APPEND_UNDO : 0 );
        pushed = true;
        batches++;
        lastPush = CLOCK::now();
    };

    autoplacer.SetRefreshCallback(
            [&]( FOOTPRINT* aFootprint )
            {
                KopenapiKeepUiAlive();

                if( aFootprint && CLOCK::now() - lastPush >= BATCH )
                {
                    pushBatch();
                    KopenapiKeepUiAlive();   // a push refreshes connectivity: paint before the readers
                }

                KopenapiReadPoint();
                KopenapiKeepUiAlive();
                return 0;
            } );
    autoplacer.SetOverlay( std::make_shared<KIGFX::VIEW_OVERLAY>() );   // it draws its matrix unconditionally

    const AR_RESULT result = rest.empty() ? AR_COMPLETED : autoplacer.AutoplaceFootprints( rest, &commit, false );

    if( result == AR_COMPLETED )
    {
        pushBatch();
    }
    else if( result == AR_CANCELLED )
    {
        pushBatch();
        notes.push_back( "stopped: the parts placed so far are kept" );
    }
    else
    {
        commit.Revert();
        notes.push_back( "KiCad's autoplacer did not complete" );
    }


    for( FOOTPRINT* fp : rest )
        placed.push_back( fp->m_Uuid );

    // The autoplacer keeps pads inside the outline, not courtyards: pull back what pokes out
    {
        BOARD_COMMIT pull( context->GetToolManager() );
        int          pulled = 0;
        BOX2I        inner = edges;

        inner.Inflate( -toIU( 0.25 ) );

        for( FOOTPRINT* fp : rest )
        {
            const BOX2I box = courtyard( fp ).BBox();
            VECTOR2I    shift( 0, 0 );

            if( box.GetWidth() > inner.GetWidth() || box.GetHeight() > inner.GetHeight() )
                continue;

            if( box.GetLeft() < inner.GetLeft() )
                shift.x = inner.GetLeft() - box.GetLeft();
            else if( box.GetRight() > inner.GetRight() )
                shift.x = inner.GetRight() - box.GetRight();

            if( box.GetTop() < inner.GetTop() )
                shift.y = inner.GetTop() - box.GetTop();
            else if( box.GetBottom() > inner.GetBottom() )
                shift.y = inner.GetBottom() - box.GetBottom();

            if( shift.x || shift.y )
            {
                pull.Modify( fp );
                fp->Move( shift );
                pulled++;
            }
        }

        if( pulled )
        {
            pull.Push( _( "Footprints inside the outline (API)" ) );
            notes.push_back( std::to_string( pulled ) + " footprint(s) moved back inside the outline (courtyard)" );
        }
    }

    // 4. Functional blocks: their parts go next to the pads of their main part
    nlohmann::json groups = nlohmann::json::array();
    bool           poursStale = false;   // the planes give way to the bottom caps and their vias

    for( const nlohmann::json& group : aArgs.value( "groups", nlohmann::json::array() ) )
    {
        FOOTPRINT*              anchor = nullptr;
        std::vector<FOOTPRINT*> parts;
        NEAR_OPTIONS            options;

        if( groupParts( aCtx, board, group, anchor, parts, false ) || nearOptions( group, options, 1.0 ) || openSidesError( group ) )
        {
            notes.push_back( "group skipped (anchor / parts not found or locked): " + group.dump() );
            continue;
        }

        BOARD_COMMIT                groupCommit( context->GetToolManager() );
        nlohmann::json              row = nlohmann::json::object();
        const double                escapeKeep = group.value( "escape_keep_mm", 2.0 );
        std::vector<CORRIDOR>       corridors = escapeKeep > 0 ? signalCorridors( aCtx, board, escapeKeep, parts ) : std::vector<CORRIDOR>();

        for( CORRIDOR& c : portCorridors( aCtx, board, anchor, portsKeep( group ), group.value( "open_sides", nlohmann::json() ) ) )
            corridors.push_back( std::move( c ) );

        options.corridors = &corridors;

        placeGroup( board, groupCommit, anchor, parts, options, group.value( "side", std::string( "same" ) ) == "opposite",
                    group.value( "decap_max_mm", 3.0 ),
                    group.contains( "decap_bottom" ) ? std::optional<bool>( group.value( "decap_bottom", false ) ) : std::nullopt,
                    row, placed, group.value( "decap_vias", true ) );

        for( const CORRIDOR& corridor : corridors )
            row["corridors"].push_back( corridor.name );

        groupCommit.Push( _( "Place group (API)" ) );
        poursStale = poursStale || row.contains( "pours_stale" );
        row.erase( "pours_stale" );
        groups.push_back( row );
    }

    if( poursStale )
        refillPours( *context, board );

    // the placed parts' pads against other nets' copper
    std::set<const FOOTPRINT*> placedFps;

    for( const KIID& id : placed )
    {
        if( FOOTPRINT* fp = dynamic_cast<FOOTPRINT*>( board->ResolveItem( id, true ) ) )
            placedFps.insert( fp );
    }

    nlohmann::json        copperOut = nlohmann::json::object();
    std::map<int, size_t> rippedNets;

    if( onCopperMode == "refuse" )
    {
        std::set<FOOTPRINT*> back;

        for( const nlohmann::json& c : copperConflicts( board, placedFps ) )
        {
            if( FOOTPRINT* fp = board->FindFootprintByReference( wxString::FromUTF8( c["ref"].get<std::string>() ) );
                fp && was.count( fp ) )
            {
                back.insert( fp );
            }
        }

        if( !back.empty() )
        {
            BOARD_COMMIT   restore( context->GetToolManager() );
            nlohmann::json refused = nlohmann::json::array();

            for( FOOTPRINT* fp : back )
            {
                restore.Modify( fp );

                if( fp->GetLayer() != was[fp].side )
                    fp->Flip( fp->GetPosition(), FLIP_DIRECTION::LEFT_RIGHT );

                fp->SetOrientation( was[fp].angle );
                fp->SetPosition( was[fp].pos );
                refused.push_back( str( fp->GetReference() ) );
                placedFps.erase( fp );
            }

            restore.Push( _( "Autoplace: parts back off other nets' copper (API)" ) );
            copperOut["refused"] = refused;
        }
    }
    else if( onCopperMode == "rip" )
    {
        BOARD_COMMIT rip( context->GetToolManager() );
        onCopper( board, rip, placedFps, "rip", copperOut, rippedNets );

        if( !rippedNets.empty() )
            rip.Push( _( "Autoplace: rip other nets' copper (API)" ) );
        else
            rip.Revert();
    }

    refreshGui( aCtx, board );
    KopenapiKeepUiAlive();
    glow( aCtx, placed );

    nlohmann::json report = placementReport( board, nullptr );
    report["copper_conflicts"] = copperConflicts( board, placedFps );

    if( !report["copper_conflicts"].empty() )
        report["ok"] = false;

    for( const char* key : { "refused", "ripped" } )
    {
        if( copperOut.contains( key ) )
            report[key] = copperOut[key];
    }

    if( !rippedNets.empty() )
        report["opened"] = openedByRip( board, rippedNets );

    report["notes"] = notes;

    if( !groups.empty() )
        report["groups"] = groups;
    report["autoplacer"] = result == AR_COMPLETED ? "completed" : result == AR_CANCELLED ? "stopped" : "failed";
    report["autoplacer_commits"] = batches;

    const KOPENAPI_LIST_CAP cap( aArgs, PLACE_AUTO_LIST_CAP );

    for( const char* key : { "courtyard_overlaps", "outside_board", "connectors", "connector_problems", "edge_overhang",
                             "notes", "groups", "copper_conflicts" } )
    {
        if( report.contains( key ) )
            cap.Put( report, key, report[key] );
    }

    return KOPENAPI_RESULT::Ok( report );
}


static KOPENAPI_RESULT h_pcb_place_group( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*            board = context->GetBoard();
    const bool        batch = aArgs.contains( "groups" );
    const bool        overrideLocks = aArgs.value( "override_locks", false );
    const std::string onCopperMode = aArgs.value( "on_copper", std::string( "report" ) );

    if( onCopperMode != "report" && onCopperMode != "refuse" && onCopperMode != "rip" )
        return KOPENAPI_RESULT::Error( 400, "on_copper: report, refuse or rip" );

    if( batch == aArgs.contains( "anchor" ) )
        return KOPENAPI_RESULT::Error( 400, "give anchor (one block) or groups (several, in order), not both" );

    if( batch && ( !aArgs["groups"].is_array() || aArgs["groups"].empty() ) )
        return KOPENAPI_RESULT::Error( 400, "groups: [{anchor, parts | from_schematic, decoupling, ...}, ...]" );

    const nlohmann::json      specs = batch ? aArgs["groups"] : nlohmann::json::array( { aArgs } );
    BOARD_COMMIT              commit( context->GetToolManager() );
    std::vector<KIID>         moved;
    std::set<const FOOTPRINT*> movedFps;
    nlohmann::json            rows = nlohmann::json::array();

    for( size_t i = 0; i < specs.size(); ++i )
    {
        const nlohmann::json&   spec = specs[i];
        const std::string       where = batch ? "groups/" + std::to_string( i ) + ": " : "";
        FOOTPRINT*              anchor = nullptr;
        std::vector<FOOTPRINT*> parts;
        NEAR_OPTIONS            options;

        auto fail = [&]( const KOPENAPI_RESULT& aError )
        {
            commit.Revert();
            KOPENAPI_RESULT error = aError;

            if( batch && error.body.contains( "error" ) && error.body["error"].value( "message", nlohmann::json() ).is_string() )
                error.body["error"]["message"] = where + error.body["error"]["message"].get<std::string>();

            return error;
        };

        if( std::optional<KOPENAPI_RESULT> error = groupParts( aCtx, board, spec, anchor, parts, overrideLocks ) )
            return fail( *error );

        // a block of a batch leaves the parts an earlier block placed where they are, unless it names them
        if( batch && !spec.contains( "parts" ) )
        {
            std::erase_if( parts, [&]( FOOTPRINT* fp ) { return movedFps.count( fp ) > 0; } );
        }

        if( std::optional<std::string> wrong = nearOptions( spec, options, 1.0 ) )
            return fail( KOPENAPI_RESULT::Error( 400, *wrong ) );

        const std::string side = spec.value( "side", std::string( "same" ) );
        const double      decapMax = spec.value( "decap_max_mm", 3.0 );

        if( side != "same" && side != "opposite" )
            return fail( KOPENAPI_RESULT::Error( 400, "side: same (as the anchor) or opposite (under it, on the other side of the board)" ) );

        if( std::optional<std::string> wrong = openSidesError( spec ) )
            return fail( KOPENAPI_RESULT::Error( 400, *wrong ) );

        nlohmann::json decoupling;

        if( spec.contains( "decoupling" ) )
        {
            if( spec["decoupling"] == "per_pin" )
            {
                addDecouplingPerPin( aCtx, board, anchor, parts, overrideLocks, decoupling );
            }
            else if( spec["decoupling"] == "schematic" )
            {
                if( std::optional<KOPENAPI_RESULT> error = addDecoupling( aCtx, board, anchor, parts, overrideLocks, decoupling ) )
                    return fail( *error );
            }
            else
            {
                return fail( KOPENAPI_RESULT::Error( 400, "decoupling: \"per_pin\" (each supply pin group of the anchor its share of "
                                                          "its rail's capacitors) or \"schematic\" (the capacitors drawn nearest to it)" ) );
            }
        }

        nlohmann::json out = nlohmann::json::object();

        if( parts.empty() )
        {
            if( !batch )
            {
                return fail( KOPENAPI_RESULT::Error( 422, "no parts: none named, none found in the schematic block and none on a small "
                                                          "net of " + str( anchor->GetReference() ) + " (name them: parts)" ) );
            }

            out = { { "anchor", str( anchor->GetReference() ) }, { "placed", nlohmann::json::array() },
                    { "unplaced", nlohmann::json::array() }, { "note", "no parts left for this block" } };
        }
        else
        {
            const size_t                before = moved.size();
            const double                escapeKeep = spec.value( "escape_keep_mm", 2.0 );
            std::vector<CORRIDOR>       corridors;

            auto place = [&]( BOARD_COMMIT& aCommit, nlohmann::json& aOut, std::vector<KIID>& aMoved )
            {
                corridors = escapeKeep > 0 ? signalCorridors( aCtx, board, escapeKeep, parts ) : std::vector<CORRIDOR>();

                for( CORRIDOR& c : portCorridors( aCtx, board, anchor, portsKeep( spec ), spec.value( "open_sides", nlohmann::json() ) ) )
                    corridors.push_back( std::move( c ) );

                options.corridors = &corridors;
                placeGroup( board, aCommit, anchor, parts, options, side == "opposite", decapMax,
                            spec.contains( "decap_bottom" ) ? std::optional<bool>( spec.value( "decap_bottom", false ) ) : std::nullopt,
                            aOut, aMoved, spec.value( "decap_vias", true ) );
            };

            std::optional<int>   turns;
            nlohmann::json       orientation;

            if( std::optional<KOPENAPI_RESULT> error = anchorTurns( spec, anchor, overrideLocks, turns ) )
                return fail( *error );

            if( spec.value( "orientation", nlohmann::json( "keep" ) ) == "auto" )
            {
                // each quarter turn tried: the block placed in a step taken back, the way out of the
                // anchor's neighbourhood estimated, the cheapest kept
                const auto                  started = std::chrono::steady_clock::now();
                const std::set<int>         rails = railNets( board );
                const double                margin = spec.value( "orientation_margin_mm", 6.0 );
                nlohmann::json              byTurn = nlohmann::json::array();
                std::vector<int>            failed( 4, 0 );
                std::vector<double>         rest( 4, 0.0 );

                for( int q = 0; q < 4; ++q )
                {
                    BOARD_COMMIT      trial( context->GetToolManager() );
                    nlohmann::json    trialOut = nlohmann::json::object();
                    std::vector<KIID> trialMoved;

                    turnAnchor( trial, anchor, q );
                    place( trial, trialOut, trialMoved );

                    const ESCAPE_ESTIMATE e = EstimateEscape( board, anchor, rails, margin );
                    const size_t          unplaced = trialOut.value( "unplaced", nlohmann::json::array() ).size();

                    trial.Revert();

                    failed[q] = e.boxedIn + e.firstPassFailed + e.overflowing + (int) unplaced;
                    rest[q] = e.vias + e.lengthMm / 10.0;

                    byTurn.push_back( { { "quarter_turns_cw", q }, { "cost", std::round( ( 10.0 * failed[q] + rest[q] ) * 10 ) / 10 },
                                        { "failed", failed[q] }, { "connections", e.connections }, { "boxed_in", e.boxedIn },
                                        { "first_pass_failed", e.firstPassFailed }, { "overflowing", e.overflowing }, { "vias", e.vias },
                                        { "length_mm", e.lengthMm }, { "unplaced", unplaced }, { "boxed_in_at", e.boxedPads },
                                        { "ms", std::round( e.ms ) } } );
                    orientation["cell_mm"] = e.cellMm;
                }

                // fewest connections without a way first; among those the shortest (vias + length), but
                // the anchor stays as it is unless that saves a tenth
                const int fewest = *std::min_element( failed.begin(), failed.end() );
                int       best = -1;

                for( int q = 0; q < 4; ++q )
                {
                    if( failed[q] == fewest && ( best < 0 || rest[q] < rest[best] ) )
                        best = q;
                }

                if( failed[0] == fewest && rest[best] > 0.9 * rest[0] )
                {
                    best = 0;
                    orientation["why"] = "kept: no turn gets more connections out, none saves a tenth of the vias and length";
                }
                else
                {
                    orientation["why"] = failed[best] < failed[0] ? "fewer connections without a way out"
                                                                  : "as many ways out, a tenth or more shorter";
                }

                turns = best;
                orientation["by"] = "routability";
                orientation["by_turn"] = byTurn;
                orientation["ms"] = std::round( std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - started ).count() );
            }

            if( turns )
            {
                if( *turns != 0 )
                {
                    turnAnchor( commit, anchor, *turns );
                    moved.push_back( anchor->m_Uuid );
                }

                orientation["quarter_turns_cw"] = *turns;
                orientation["rotation_deg"] = anchor->GetOrientation().AsDegrees();
            }

            place( commit, out, moved );

            if( turns )
                out["orientation"] = orientation;

            for( const CORRIDOR& corridor : corridors )
                out["corridors"].push_back( corridor.name );

            for( size_t k = before; k < moved.size(); ++k )
            {
                if( FOOTPRINT* fp = dynamic_cast<FOOTPRINT*>( board->ResolveItem( moved[k], true ) ) )
                    movedFps.insert( fp );
            }
        }

        if( !decoupling.is_null() )
            out["decoupling"] = decoupling;

        rows.push_back( out );
    }

    bool stale = false;   // the planes give way to the bottom caps and their vias

    for( nlohmann::json& row : rows )
    {
        stale = stale || row.contains( "pours_stale" );
        row.erase( "pours_stale" );
    }

    nlohmann::json out = batch ? nlohmann::json( { { "groups", rows } } ) : rows[0];

    if( moved.empty() )
    {
        commit.Revert();
        out["applied"] = false;
        return KOPENAPI_RESULT::Ok( out );
    }

    if( aArgs.value( "dry_run", false ) )
    {
        out["copper_conflicts"] = copperConflicts( board, movedFps );
        commit.Revert();
        out["applied"] = false;
        return KOPENAPI_RESULT::Ok( out );
    }

    std::map<int, size_t> rippedNets;

    if( onCopper( board, commit, movedFps, onCopperMode, out, rippedNets ) )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Ok( out );
    }

    const nlohmann::json conflictsBefore = copperConflicts( board, movedFps );

    commit.Push( batch ? _( "Place groups (API)" ) : _( "Place group (API)" ) );

    if( stale )
        refillPours( *context, board );

    refreshGui( aCtx, board );
    glow( aCtx, moved );

    out["applied"] = true;
    out["check"] = placementReport( board, &movedFps );
    out["check"]["copper_conflicts"] = mergeConflicts( board, conflictsBefore, out["check"]["copper_conflicts"] );

    if( !out["check"]["copper_conflicts"].empty() )
        out["check"]["ok"] = false;

    if( out.contains( "ripped" ) )
        out["opened"] = openedByRip( board, rippedNets );

    return KOPENAPI_RESULT::Ok( out );
}


static const char* PLACE_GROUP_BLOCK = R"json({
    "anchor":{"type":"string","description":"reference of the main part; it does not move (orientation may turn it in place)"},
    "orientation":{"description":"the anchor's quarter turn: keep (default); auto - each of the four tried (the block placed, then a quick route of every non-rail connection with an end within orientation_margin_mm of the anchor's courtyard, the far ends reached straight from the box's edge, on a grid of the anchor's pad pitch (or one track pitch) over the routing layers, shortest first without rip-up, then negotiated), the turn with the fewest connections left without a way (boxed in, failed in the first pass, still sharing cells, parts unplaced), of those the one with the fewest vias + mm / 10; the anchor stays as it is unless a turn gets more connections out or saves a tenth (why says which); or 0..3 quarter turns clockwise. The anchor turns about its courtyard's centre; a locked one needs override_locks. Answer orientation: quarter_turns_cw, rotation_deg, why, by_turn (auto: per turn failed, its parts, vias, length_mm, cost = 10 x failed + vias + mm / 10)","oneOf":[{"type":"string","enum":["keep","auto"]},{"type":"integer","minimum":0,"maximum":3}]},
    "orientation_margin_mm":{"type":"number","default":6,"description":"orientation auto: how far round the anchor's courtyard the estimate routes"},
    "parts":{"type":"array","items":{"type":"string"}},
    "from_schematic":{"type":"boolean","default":false,"description":"without parts: take the block's members from the open schematic (sch_subcircuit of the anchor)"},
    "depth":{"type":"integer","description":"from_schematic: how far sch_subcircuit grows (default 2)"},
    "decoupling":{"type":"string","enum":["per_pin","schematic"],"description":"add the anchor's decoupling capacitors: per_pin - each supply pin group of the anchor (adjacent pins of a rail count once) one capacitor under 1 uF of its rail plus one bulk per rail, the rail's capacitors split among the parts on it by count, each to the part drawn nearest to it (schematic if open, else board); schematic - C parts across two of its rails drawn nearest to it on its sheet, one per supply pin group (a guess). The answer's decoupling.chosen lists them"},
    "decap_max_mm":{"type":"number","default":3,"description":"a decoupling capacitor farther than this from its pin is placed inside the fan-out ring if that is nearer, and listed in decoupling_far"},
    "escape_keep_mm":{"type":"number","default":2,"minimum":0,"description":"the way of differential pairs and clock nets (pcb_net_roles) between the parts they join, and each of their pins' straight escape this long, is kept free of the block's parts; a part enters it only when it finds no other place (a decoupling capacitor: when only that brings it within decap_max_mm), its row then names it (in_corridor); 0 off"},
    "decap_bottom":{"type":"boolean","description":"a decoupling capacitor still farther than decap_max_mm from its pin goes to the board's other side, under the anchor, when that brings it within reach: each supply pin gets a via spot just beyond its inner end, under the body, clear of the other nets' copper and spots by the clearance (else beyond its outer end; a neighbour pin of the net within 1.2 mm shares it), and the cap's pad goes inward of the spot where there is room, else on any side of it, its other pads clear of every spot; the row gives via_spot_mm; default on for a fine-pitch anchor (0.65 mm or less), off otherwise; the row says side"},
    "ports_keep_mm":{"type":"number","default":0,"minimum":0,"description":"layout level 2: the sides where the anchor's signals leave for other blocks (pcb_fanout_ports), as a band from those pads out past its courtyard this far, are kept free of the block's parts like escape_keep_mm (parts on those nets may enter; a decoupling capacitor only when that alone brings it within decap_max_mm); corridors lists them as ports <side> (<far parts>); 0 off"},
    "decap_vias":{"type":"boolean","default":true,"description":"decap_bottom: lay each bottom cap's via at its spot with a stub from the pin on the pin's layer (net class sizes, the stub no wider than the pin), in the same undo step, and the cap's pad joined to it on the cap's side where that track keeps clear; the rows say via_laid / via_joined, the answer vias_laid; the planes are refilled after"},
    "open_sides":{"description":"layout level 2 variants: which of the anchor's port sides ports_keep_mm keeps open (default 3 mm when only this is given): a count 0..4 - the sides with the most signal ports first; or a list of west / east / north / south - those, a side without ports along its whole courtyard edge; default every side with ports. corridors names the bands; block gives the outline each variant takes (bbox_mm, area_mm2)","oneOf":[{"type":"integer","minimum":0,"maximum":4},{"type":"array","items":{"type":"string","enum":["west","east","north","south"]}}]},
    "max_net_pads":{"type":"integer","default":4,"description":"without parts: nets of the anchor with at most this many pads on the board select the parts"},
    "side":{"type":"string","enum":["same","opposite"],"default":"same"},
    "gap_mm":{"type":"number","default":1.0,"description":"between courtyards (room for tracks between the parts; TC-13 2 layers: 0.5 mm routed 68 of 81, 1.0 mm 73); by default a part that finds no place at 1.0 mm takes 0.5"},
    "fanout_ring_mm":{"type":"number","description":"keep this far from the anchor's courtyard (room for the tracks leaving its pins); default 1.0 for a fine-pitch anchor (pads 0.65 mm apart or closer), else 0; parts that find no place with it come as close as the courtyards allow"},
    "max_mm":{"type":"number","default":25,"description":"how far from the anchor's pad a part may end up"}})json";


static nlohmann::json placeGroupSchema()
{
    nlohmann::json block = nlohmann::json::parse( PLACE_GROUP_BLOCK );
    nlohmann::json props = block;

    props["groups"] = { { "type", "array" },
                        { "description", "several blocks in one call and one undo step, placed in this order (a block "
                                         "sees the ones before it; parts an earlier block placed stay unless named)" },
                        { "items", { { "type", "object" }, { "required", { "anchor" } }, { "properties", block } } } };
    props["override_locks"] = { { "type", "boolean" }, { "default", false } };
    props["on_copper"] = nlohmann::json::parse( R"json({"type":"string","enum":["report","refuse","rip"],"default":"report","description":"the moved pads under or within clearance (DRC rules) of other nets' tracks, vias, pads or zone fills: report lists them as copper_conflicts (ok false); refuse moves nothing (applied false, the conflicts listed); rip removes the conflicting unlocked tracks and vias in the same undo step (ripped) and lists the connections that opened (opened); zone fills are listed, refill with pcb_zones_fill"})json" );
    props["dry_run"] = { { "type", "boolean" }, { "default", false } };

    return { { "type", "object" }, { "properties", props } };
}


KOPENAPI_REGISTER( "pcb_place_group",
                   "Place the parts of a functional block around its main part (anchor), each next to the "
                   "anchor pad it belongs to and turned so its own pad faces it: a crystal and its "
                   "capacitors at the oscillator pins, pull-ups at their pins, a regulator's parts, and "
                   "decoupling capacitors - each at its own supply pin group, smallest value first (a part "
                   "between two rails of the anchor takes a free supply pad with the nearest ground pad). "
                   "Rails are told by role (ground / power names, power pins), never by pad count: a "
                   "summing or bias net with many pads is a signal of the block. parts: the references "
                   "(e.g. from sch_subcircuit); without it, every unlocked part (connectors aside) on a small "
                   "net of the anchor (max_net_pads); from_schematic: the members of the anchor's block in the "
                   "open schematic. A part bound to a member rather than to the anchor goes next to that "
                   "member, or to a part placed at the anchor by an earlier call. decoupling: per_pin / "
                   "schematic (listed in the answer). Parts stay inside the board outline; a part low enough "
                   "(3D model) may go under a footprint with the field max_height_under_mm (a daughterboard "
                   "over its headers: only its pads are in the way; the row says under). side opposite: on "
                   "the other side of the board, under the pads. groups: several blocks in one call, in order. "
                   "One undo step; answers per part the target pad, pad distance, position, and the "
                   "placement check",
                   placeGroupSchema(), false, h_pcb_place_group, 120 );

static KOPENAPI_RESULT h_pcb_place_legalize( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*              board = context->GetBoard();
    KOPENAPI_REF_FILTER filter( aArgs );
    const double        gap = aArgs.value( "gap_mm", 0.1 );
    const double        maxMm = aArgs.value( "max_mm", 5.0 );
    const bool          overrideLocks = aArgs.value( "override_locks", false );

    if( !filter.Error().empty() )
        return KOPENAPI_RESULT::Error( 400, filter.Error() );

    if( gap < 0 || gap > 5 || maxMm <= 0 || maxMm > 50 )
        return KOPENAPI_RESULT::Error( 400, "gap_mm 0 … 5, max_mm up to 50" );

    std::vector<FOOTPRINT*> all( board->Footprints().begin(), board->Footprints().end() );
    std::map<FOOTPRINT*, bool> movable;

    for( FOOTPRINT* fp : all )
        movable[fp] = ( !fp->IsLocked() || overrideLocks ) && filter.Match( str( fp->GetReference() ) );

    const std::map<const FOOTPRINT*, double> heights = heightsIfRoomUnder( board, all );
    BOARD_COMMIT                             commit( context->GetToolManager() );
    std::map<FOOTPRINT*, VECTOR2I>           from;
    std::map<std::string, nlohmann::json>    stuck;

    // the mover of a pair: a part that may move, not a connector if the other may, the smaller one
    auto mover = [&]( FOOTPRINT* aA, FOOTPRINT* aB ) -> FOOTPRINT*
    {
        if( !movable[aA] || !movable[aB] )
            return movable[aA] ? aA : movable[aB] ? aB : nullptr;

        if( isConnector( aA ) != isConnector( aB ) )
            return isConnector( aA ) ? aB : aA;

        return courtyard( aA ).Area() <= courtyard( aB ).Area() ? aA : aB;
    };

    for( int round = 0; round < 6; ++round )
    {
        const nlohmann::json report = placementReport( board );
        std::set<FOOTPRINT*> movedNow;
        bool                 progress = false;

        for( const nlohmann::json& row : report["courtyard_overlaps"] )
        {
            FOOTPRINT* a = findFootprint( board, row.value( "a", std::string() ) );
            FOOTPRINT* b = findFootprint( board, row.value( "b", std::string() ) );

            if( !a || !b )
                continue;

            const std::string key = row["a"].get<std::string>() + "|" + row["b"].get<std::string>();
            FOOTPRINT*        part = mover( a, b );

            if( !part )
            {
                stuck[key] = { { "a", row["a"] }, { "b", row["b"] }, { "reason", "neither may move (locked, or not in ref / refs)" } };
                continue;
            }

            if( movedNow.count( a ) || movedNow.count( b ) )
                continue;

            auto tryMove = [&]( FOOTPRINT* aPart ) -> std::optional<double>
            {
                if( !from.count( aPart ) )
                {
                    commit.Modify( aPart );
                    from[aPart] = aPart->GetPosition();
                }

                auto h = heights.find( aPart );
                return nudge( board, aPart, gap, maxMm, h == heights.end() ? std::nullopt : std::optional<double>( h->second ) );
            };

            std::optional<double> moved = tryMove( part );
            FOOTPRINT*            other = part == a ? b : a;

            // the other one, when it may move and the first finds no place
            if( !moved && movable[other] )
            {
                moved = tryMove( other );

                if( moved )
                    part = other;
            }

            if( !moved )
            {
                stuck[key] = { { "a", row["a"] }, { "b", row["b"] }, { "moving", str( part->GetReference() ) },
                               { "reason", "no free place within max_mm" } };
                continue;
            }

            stuck.erase( key );

            if( *moved > 0 )
            {
                movedNow.insert( part );
                progress = true;
            }
        }

        if( !progress )
            break;
    }

    nlohmann::json            rows = nlohmann::json::array();
    std::vector<KIID>         moved;
    std::set<const FOOTPRINT*> movedFps;

    for( const auto& [fp, start] : from )
    {
        if( fp->GetPosition() == start )
            continue;

        rows.push_back( { { "ref", str( fp->GetReference() ) },
                          { "from_mm", { toMm( start.x ), toMm( start.y ) } },
                          { "to_mm", { toMm( fp->GetPosition().x ), toMm( fp->GetPosition().y ) } },
                          { "moved_mm", std::round( toMm( ( fp->GetPosition() - start ).EuclideanNorm() ) * 100 ) / 100 } } );
        moved.push_back( fp->m_Uuid );
        movedFps.insert( fp );
    }

    std::sort( rows.begin(), rows.end(), []( const nlohmann::json& x, const nlohmann::json& y )
               { return KopenapiNaturalLess( x["ref"].get<std::string>(), y["ref"].get<std::string>() ); } );

    nlohmann::json left = nlohmann::json::array();

    for( const auto& [key, row] : stuck )
        left.push_back( row );

    nlohmann::json out = { { "moved", rows }, { "moved_count", rows.size() }, { "left", left } };

    if( !filter.Empty() && !filter.Unmatched().empty() )
        out["unmatched"] = filter.Unmatched();

    if( moved.empty() || aArgs.value( "dry_run", false ) )
    {
        commit.Revert();
        out["applied"] = false;
        return KOPENAPI_RESULT::Ok( out );
    }

    commit.Push( _( "Legalize placement (API)" ) );
    refreshGui( aCtx, board );
    glow( aCtx, moved );

    out["applied"] = true;
    out["check"] = placementReport( board, &movedFps );
    out["courtyard_overlaps_left"] = placementReport( board )["courtyard_overlaps"].size();
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_REGISTER( "pcb_place_legalize",
                   "Take parts off courtyard overlaps by the least move: for each overlapping pair one part moves "
                   "(one that may: unlocked and in ref / refs; not a connector when the other may move; else the "
                   "smaller) to the nearest spot clear of every courtyard (gap_mm) and inside the board outline, "
                   "its rotation kept, a few rounds until nothing overlaps. A part low enough may stay under a "
                   "footprint with room under it (field max_height_under_mm). One undo step; answers moved "
                   "(from, to, moved_mm), left (pairs it could not part, why), the placement check of the moved "
                   "parts (copper conflicts included) and courtyard_overlaps_left",
                   R"json({"type":"object","properties":{
                        "ref":{"type":"string","description":"glob of the parts that may move (braces, !glob exclusions); default every unlocked part"},
                        "refs":{"type":"array","items":{"type":"string"}},
                        "gap_mm":{"type":"number","default":0.1,"description":"clearance between courtyards after the move"},
                        "max_mm":{"type":"number","default":5,"description":"the farthest a part may move"},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_place_legalize, 120 );


KOPENAPI_REGISTER( "pcb_footprint_move",
                   "Place footprints: several at once in one undo step - position (mm) of the origin, the "
                   "courtyard's middle (anchor center) or a connector's panel-edge line (anchor panel_edge), "
                   "rotation_deg or facing (connectors: mating side towards left / right / up / down), side "
                   "top / bottom; or near: \"U1.7\" - next to that pad of another part, as close as the "
                   "courtyards allow, turned so the part's pad on that net faces it (no coordinates to "
                   "compute; a whole block: pcb_place_group); with: parts that move and turn together with it (a "
                   "placed block as one); locked ones need override_locks; answers where they are now and the "
                   "placement check of the moved footprints (overlaps, outside the board, other nets' copper "
                   "under the moved pads: on_copper, airwire length; report: full for the whole board); tracks "
                   "and vias stay where they are: copper that ended on a moved pad and now sits on a pad of another "
                   "net is listed in shorts_created (ok false), copper no pad of its net holds any more in "
                   "dangling_at_pads (each with was_on REF.PAD); rip_attached: true removes it in the same step)",
                   R"json({"type":"object","required":["moves"],"properties":{
                        "moves":{"type":"array","items":{"type":"object","required":["ref"],"properties":{
                            "ref":{"type":"string"},"x_mm":{"type":"number"},"y_mm":{"type":"number"},
                            "with":{"type":"array","items":{"type":"string"},"description":"references that keep their place relative to this footprint: they turn about its origin and shift with it (a placed block moves as one)"},
                            "near":{"type":"string","description":"REF.PAD of another part: place this one next to that pad (instead of x_mm / y_mm); with side bottom / top it may sit under the pad on the other side"},
                            "near_pad":{"type":"string","description":"near: the pad of this part to bring to the target (default: its pad on the target's net)"},
                            "fanout_ring_mm":{"type":"number","default":0,"description":"near: keep this far from the target footprint's courtyard (room for the tracks leaving its pins)"},
                            "near_direction":{"type":"string","enum":["auto","left","right","up","down"],"default":"auto","description":"near: which way from the target pad (on the board); auto tries all four, the side the pad is on first"},
                            "gap_mm":{"type":"number","default":0.25,"description":"near: gap between courtyards"},
                            "max_mm":{"type":"number","default":25,"description":"near: give up beyond this pad-to-pad distance"},
                            "rotation_deg":{"type":"number"},"side":{"type":"string","enum":["top","bottom"]},
                            "anchor":{"type":"string","enum":["origin","center","panel_edge"],"default":"origin","description":"center: x / y are the courtyard's middle; panel_edge: the middle of the connector's panel-edge line (put it on the board edge)"},
                            "edge_offset_mm":{"type":"number","default":0,"description":"anchor panel_edge: shift along the facing direction (+ out past the edge, - recessed)"},
                            "edge":{"type":"string","enum":["left","right","top","bottom"],"description":"put the connector on that board edge facing out (facing + anchor panel_edge + the edge coordinate); give the other coordinate"},
                            "facing":{"type":"string","enum":["left","right","up","down"],"description":"connectors: turn so the mating side faces this way (panel edge from the footprint's marker or inferred from its body, see pcb_footprint_get panel_edge)"}}}},
                        "override_locks":{"type":"boolean","default":false},
                        "on_copper":{"type":"string","enum":["report","refuse","rip"],"default":"report","description":"the moved pads under or within clearance (DRC rules) of other nets' tracks, vias, pads or zone fills: report lists them as copper_conflicts (ok false); refuse moves nothing (applied false, the conflicts listed); rip removes the conflicting unlocked tracks and vias in the same undo step (ripped) and lists the connections that opened (opened); zone fills are listed, refill with pcb_zones_fill"},
                        "rip_attached":{"type":"boolean","default":false,"description":"remove the tracks and vias that ended on the moved parts' pads (they stay where the pads were) in the same undo step: ripped_attached (locked ones kept: attached_locked)"},
                        "report":{"type":"string","enum":["moved","full","none"],"default":"moved","description":"placement check in the answer: for the footprints moved (overlaps with any part, outside the board, connector facing; ratsnest of the whole board), the whole board, or none"}}})json"_json,
                   false, h_pcb_footprint_move );

KOPENAPI_REGISTER( "pcb_placement_check",
                   "Judge the placement: courtyard overlaps (same side), footprints outside the board, "
                   "connectors' distance to the board edge, unrouted connections and total airwire "
                   "length (shorter = better placement for routing), board size; ok flag",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_placement_check );

KOPENAPI_REGISTER( "pcb_place_auto",
                   "A first placement: board outline kept, given (width_mm / height_mm) or sized from the "
                   "parts (density, aspect); connectors along the edges; the rest by KiCad's autoplacer "
                   "(inside the outline, short airwires), then the given groups (a main part with the parts "
                   "that belong at its pins); unlocked footprints only; answers the placement check. Refine "
                   "with pcb_place_group and pcb_footprint_move (near); the answer's lists capped at list_limit "
                   "(counts whole), full: true for all",
                   KOPENAPI_LIST_CAP::Schema( R"json({"type":"object","properties":{
                        "width_mm":{"type":"number"},"height_mm":{"type":"number"},
                        "x_mm":{"type":"number","default":20},"y_mm":{"type":"number","default":20},
                        "density":{"type":"number","default":0.35},"aspect":{"type":"number","default":1.5},
                        "connectors_to_edge":{"type":"boolean","default":true},
                        "on_copper":{"type":"string","enum":["report","refuse","rip"],"default":"report","description":"placed pads under or within clearance of other nets' copper (tracks, vias, pads, zone fills): report lists them as copper_conflicts (ok false); refuse puts the parts with conflicts back where they were before the call (refused); rip removes the conflicting unlocked tracks and vias (ripped, opened)"},
                        "groups":{"type":"array","description":"functional blocks to pull together after the first placement, each as pcb_place_group takes it: {anchor, parts?, side?, gap_mm?, max_mm?}","items":{"type":"object"}}}})json"_json, PLACE_AUTO_LIST_CAP ),
                   false, h_pcb_place_auto, 600 );

KOPENAPI_MARK_EDITING( "pcb_footprint_move", "pcb_place_auto", "pcb_place_group" );
