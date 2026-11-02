/// @file methods_pcb_constraints.cpp
/// @brief pcb_constraint_list / pcb_constraint: the geometric constraints of the board editor's constraint tool
/// (add from items or anchors, remove, set value / driving), through the same core functions the tool uses.

#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <constraints/board_constraint_adapter.h>
#include <constraints/constraint_builder.h>
#include <constraints/constraint_edit.h>
#include <constraints/pcb_constraint.h>
#include <core/kicad_algo.h>
#include <frame_type.h>
#include <kiway.h>
#include <pcb_edit_frame.h>
#include <pcb_shape.h>

#include <set>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


double valueOut( const PCB_CONSTRAINT& aConstraint )
{
    double v = *aConstraint.GetValue();

    if( ConstraintValueIsLength( aConstraint.GetConstraintType() ) )
        return std::round( v / pcbIUScale.IU_PER_MM * 1e6 ) / 1e6;

    return std::round( v * 1e6 ) / 1e6;
}


double valueIn( PCB_CONSTRAINT_TYPE aType, double aValue )
{
    return ConstraintValueIsLength( aType ) ? aValue * pcbIUScale.IU_PER_MM : aValue;
}


bool isValued( PCB_CONSTRAINT_TYPE aType )
{
    switch( aType )
    {
    case PCB_CONSTRAINT_TYPE::FIXED_LENGTH:
    case PCB_CONSTRAINT_TYPE::FIXED_RADIUS:
    case PCB_CONSTRAINT_TYPE::ANGULAR_DIMENSION:
    case PCB_CONSTRAINT_TYPE::ARC_ANGLE:
        return true;
    default:
        return false;
    }
}


const char* issueToken( CONSTRAINT_ADD_ISSUE aIssue )
{
    switch( aIssue )
    {
    case CONSTRAINT_ADD_ISSUE::ALL_LOCKED:     return "all_locked";
    case CONSTRAINT_ADD_ISSUE::CONFLICTING:    return "conflicting";
    case CONSTRAINT_ADD_ISSUE::NOT_APPLICABLE: return "not_applicable";
    default:                                   return "";
    }
}


const char* stateToken( CONSTRAINT_STATE aState )
{
    switch( aState )
    {
    case CONSTRAINT_STATE::UNDER_CONSTRAINED: return "under";
    case CONSTRAINT_STATE::WELL_CONSTRAINED:  return "well";
    case CONSTRAINT_STATE::OVER_CONSTRAINED:  return "over";
    default:                                  return "unconstrained";
    }
}


nlohmann::json constraintJson( const PCB_CONSTRAINT& aConstraint, const BOARD_CONSTRAINT_DIAGNOSTICS& aDiag )
{
    nlohmann::json members = nlohmann::json::array();

    for( const CONSTRAINT_MEMBER& member : aConstraint.GetMembers() )
    {
        nlohmann::json m = { { "item", str( member.m_item.AsString() ) },
                             { "anchor", ConstraintAnchorToken( member.m_anchor ) } };

        if( member.m_anchor == CONSTRAINT_ANCHOR::VERTEX )
            m["index"] = member.m_index;

        members.push_back( m );
    }

    const KIID&    id = aConstraint.m_Uuid;
    nlohmann::json row = { { "uuid", str( id.AsString() ) },
                           { "type", ConstraintTypeToken( aConstraint.GetConstraintType() ) },
                           { "label", str( ConstraintTypeLabel( aConstraint.GetConstraintType() ) ) },
                           { "members", members } };

    if( aConstraint.HasValue() )
    {
        row["value"] = valueOut( aConstraint );
        row["unit"] = ConstraintValueIsLength( aConstraint.GetConstraintType() ) ? "mm" : "deg";
        row["driving"] = aConstraint.IsDriving();
    }

    if( alg::contains( aDiag.conflicting, id ) )
        row["status"] = "conflicting";
    else if( alg::contains( aDiag.redundant, id ) )
        row["status"] = "redundant";
    else if( alg::contains( aDiag.errored, id ) )
        row["status"] = "not_applicable";
    else
        row["status"] = "ok";

    return row;
}


nlohmann::json diagnosticsJson( const BOARD_CONSTRAINT_DIAGNOSTICS& aDiag )
{
    auto ids = []( const std::vector<KIID>& aIds )
    {
        nlohmann::json out = nlohmann::json::array();

        for( const KIID& id : aIds )
            out.push_back( str( id.AsString() ) );

        return out;
    };

    nlohmann::json shapes = nlohmann::json::object();

    for( const auto& [id, state] : aDiag.shapeStates )
        shapes[str( id.AsString() )] = stateToken( state );

    return { { "free_dof", aDiag.totalFreeDof }, { "conflicting", ids( aDiag.conflicting ) },
             { "redundant", ids( aDiag.redundant ) }, { "not_applicable", ids( aDiag.errored ) },
             { "shapes", shapes } };
}


void refreshEditor( KOPENAPI_CONTEXT& aCtx )
{
    if( aCtx.headless || !aCtx.kiway )
        return;

    if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
        frame->GetCanvas()->Refresh();
}


PCB_CONSTRAINT* findConstraint( BOARD* aBoard, const std::string& aId )
{
    for( PCB_CONSTRAINT* constraint : aBoard->Constraints() )
    {
        if( str( constraint->m_Uuid.AsString() ) == aId )
            return constraint;
    }

    return nullptr;
}


KOPENAPI_RESULT h_pcb_constraint_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                       board = context->GetBoard();
    BOARD_CONSTRAINT_DIAGNOSTICS diag = DiagnoseBoardConstraints( board );
    nlohmann::json               rows = nlohmann::json::array();

    for( const PCB_CONSTRAINT* constraint : board->Constraints() )
        rows.push_back( constraintJson( *constraint, diag ) );

    return KOPENAPI_RESULT::Ok( { { "constraints", rows }, { "total", rows.size() },
                                  { "diagnostics", diagnosticsJson( diag ) } } );
}


KOPENAPI_RESULT addConstraint( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const nlohmann::json& aArgs )
{
    BOARD*              board = aContext.GetBoard();
    PCB_CONSTRAINT_TYPE type = ConstraintTypeFromToken( wxString::FromUTF8( aArgs.value( "type", std::string() ) ) );

    if( type == PCB_CONSTRAINT_TYPE::UNDEFINED )
        return KOPENAPI_RESULT::Error( 400, "type: one of the constraint type tokens (pcb_constraint_list types)" );

    std::unique_ptr<PCB_CONSTRAINT> constraint;

    if( aArgs.contains( "items" ) )
    {
        if( !aArgs["items"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "items: uuids of the shapes, as the editor's selection" );

        std::vector<BOARD_ITEM*> items;

        for( const nlohmann::json& u : aArgs["items"] )
        {
            BOARD_ITEM* item = u.is_string() ? ResolveConstrainableItem( board, KIID( u.get<std::string>() ) )
                                             : nullptr;

            if( !item )
                return KOPENAPI_RESULT::Error( 404, "items: no constrainable item " + u.dump() );

            items.push_back( item );
        }

        constraint = BuildConstraintFromItems( board, type, items );

        if( !constraint )
        {
            wxString hint = ConstraintSelectionHint( type );
            return KOPENAPI_RESULT::Error( 422, hint.IsEmpty() ? std::string( "this type is authored from anchors: "
                                                                              "give members" )
                                                               : str( hint ) );
        }
    }
    else if( aArgs.contains( "members" ) && aArgs["members"].is_array() )
    {
        constraint = std::make_unique<PCB_CONSTRAINT>( board, type );

        for( const nlohmann::json& m : aArgs["members"] )
        {
            if( !m.is_object() || !m.contains( "item" ) || !m["item"].is_string() )
                return KOPENAPI_RESULT::Error( 400, "members: [{item: uuid, anchor, index}]" );

            KIID id( m["item"].get<std::string>() );

            if( !ResolveConstrainableItem( board, id ) )
                return KOPENAPI_RESULT::Error( 404, "members: no constrainable item " + m["item"].dump() );

            const std::string token = m.value( "anchor", "whole" );
            CONSTRAINT_ANCHOR anchor = ConstraintAnchorFromToken( wxString::FromUTF8( token ) );

            if( token != ConstraintAnchorToken( anchor ) )
                return KOPENAPI_RESULT::Error( 400, "members: anchor whole | start | end | mid | center | radius | vertex" );

            constraint->AddMember( id, anchor, m.value( "index", -1 ) );
        }

        if( isValued( type ) && !aArgs.contains( "value" ) )
            return KOPENAPI_RESULT::Error( 400, "value: a dimensional constraint built from members needs its value" );
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "say what: items (uuids, as the editor's selection) or members "
                                            "([{item, anchor, index}], as its click-to-pick)" );
    }

    if( ConstraintIsDuplicateOnBoard( board, constraint.get() ) )
        return KOPENAPI_RESULT::Error( 409, str( _( "An identical geometric constraint already exists." ) ) );

    if( aArgs.contains( "value" ) )
    {
        if( !isValued( type ) )
            return KOPENAPI_RESULT::Error( 400, "value: this type has none" );

        constraint->SetValue( valueIn( type, aArgs["value"].get<double>() ) );
    }

    if( aArgs.contains( "driving" ) && constraint->HasValue() )
        constraint->SetDriving( aArgs["driving"].get<bool>() );

    PCB_CONSTRAINT* added = constraint.get();

    BOARD_COMMIT commit( aContext.GetToolManager() );
    commit.Add( constraint.release() );
    commit.Push( _( "Add Geometric Constraint (API)" ) );

    BOARD_COMMIT snap( aContext.GetToolManager() );
    SnapAddedConstraint( board, added, snap );

    const bool moved = !snap.Empty();

    if( moved )
        snap.Push( _( "Apply Geometric Constraint (API)" ), APPEND_UNDO );

    BOARD_CONSTRAINT_DIAGNOSTICS diag = DiagnoseBoardConstraints( board );
    CONSTRAINT_ADD_ISSUE         issue = CheckAddedConstraint( board, added, diag );
    nlohmann::json               out = constraintJson( *added, diag );

    out["snapped"] = moved;

    if( issue != CONSTRAINT_ADD_ISSUE::NONE )
        out["issue"] = { { "kind", issueToken( issue ) }, { "message", str( ConstraintAddIssueText( issue ) ) } };

    refreshEditor( aCtx );
    return KOPENAPI_RESULT::Ok( out );
}


KOPENAPI_RESULT removeConstraints( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const nlohmann::json& aArgs )
{
    BOARD*                       board = aContext.GetBoard();
    std::vector<PCB_CONSTRAINT*> doomed;
    nlohmann::json               missing = nlohmann::json::array();

    if( aArgs.contains( "uuid" ) )
    {
        if( PCB_CONSTRAINT* c = findConstraint( board, aArgs.value( "uuid", std::string() ) ) )
            doomed.push_back( c );
        else
            missing.push_back( aArgs["uuid"] );
    }

    if( aArgs.contains( "items" ) && aArgs["items"].is_array() )
    {
        std::set<KIID> ids;

        for( const nlohmann::json& u : aArgs["items"] )
        {
            if( u.is_string() )
                ids.insert( KIID( u.get<std::string>() ) );
        }

        for( PCB_CONSTRAINT* c : ConstraintsReferencing( board, ids, false ) )
        {
            if( !alg::contains( doomed, c ) )
                doomed.push_back( c );
        }
    }
    else if( !aArgs.contains( "uuid" ) )
    {
        return KOPENAPI_RESULT::Error( 400, "say what: uuid (a constraint) or items (every constraint referencing "
                                            "them, as Remove Constraints on a selection)" );
    }

    if( doomed.empty() )
        return KOPENAPI_RESULT::Error( 404, "no such constraint" );

    std::vector<PCB_SHAPE*> members;
    nlohmann::json          removed = nlohmann::json::array();
    BOARD_COMMIT            commit( aContext.GetToolManager() );

    for( PCB_CONSTRAINT* c : doomed )
    {
        for( PCB_SHAPE* shape : ConstraintMemberShapes( board, c ) )
            members.push_back( shape );

        removed.push_back( str( c->m_Uuid.AsString() ) );
        commit.Remove( c );
    }

    commit.Push( doomed.size() == 1 ? _( "Remove Geometric Constraint (API)" )
                                    : _( "Remove Geometric Constraints (API)" ) );

    // the members' clusters re-settle without the removed relations, as after the editor's removal
    BOARD_COMMIT settle( aContext.GetToolManager() );

    if( BoardHasConstraints( board ) )
    {
        ReSolveShapeClusters( board, members, nullptr,
                              [&]( BOARD_ITEM* aItem )
                              {
                                  settle.Modify( aItem );
                              } );
    }

    if( !settle.Empty() )
        settle.Push( _( "Apply Geometric Constraint (API)" ), APPEND_UNDO );

    refreshEditor( aCtx );
    return KOPENAPI_RESULT::Ok( { { "removed", removed }, { "missing", missing } } );
}


KOPENAPI_RESULT setValue( KOPENAPI_CONTEXT& aCtx, PCB_CONTEXT& aContext, const nlohmann::json& aArgs )
{
    BOARD*          board = aContext.GetBoard();
    PCB_CONSTRAINT* constraint = findConstraint( board, aArgs.value( "uuid", std::string() ) );

    if( !constraint )
        return KOPENAPI_RESULT::Error( 404, "uuid: no such constraint" );

    if( !constraint->HasValue() )
        return KOPENAPI_RESULT::Error( 422, "this constraint has no value" );

    if( !aArgs.contains( "value" ) && !aArgs.contains( "driving" ) )
        return KOPENAPI_RESULT::Error( 400, "say what: value and / or driving" );

    std::optional<double> value;
    std::optional<bool>   driving;

    if( aArgs.contains( "value" ) )
        value = valueIn( constraint->GetConstraintType(), aArgs["value"].get<double>() );

    if( aArgs.contains( "driving" ) )
        driving = aArgs["driving"].get<bool>();

    BOARD_COMMIT commit( aContext.GetToolManager() );
    SetConstraintValue( board, constraint, value, driving, commit );
    commit.Push( _( "Edit Geometric Constraint (API)" ) );

    BOARD_CONSTRAINT_DIAGNOSTICS diag = DiagnoseBoardConstraints( board );

    refreshEditor( aCtx );
    return KOPENAPI_RESULT::Ok( constraintJson( *constraint, diag ) );
}


KOPENAPI_RESULT h_pcb_constraint( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const std::string action = aArgs.value( "action", std::string() );

    try
    {
        if( action == "add" )
            return addConstraint( aCtx, *context, aArgs );

        if( action == "remove" )
            return removeConstraints( aCtx, *context, aArgs );

        if( action == "set_value" )
            return setValue( aCtx, *context, aArgs );
    }
    catch( const nlohmann::json::exception& e )
    {
        return KOPENAPI_RESULT::Error( 400, std::string( "bad argument: " ) + e.what() );
    }

    return KOPENAPI_RESULT::Error( 400, "action: add | remove | set_value" );
}

} // namespace


KOPENAPI_REGISTER( "pcb_constraint_list",
                   "The board's geometric constraints (the constraint tool's): type token, members (item uuid, "
                   "anchor, vertex index), value (mm or deg) and driving for the dimensional ones, status (ok, "
                   "conflicting, redundant, not_applicable); diagnostics: free degrees of freedom and each "
                   "constrained shape's state (under / well / over)",
                   R"json({"type":"object","properties":{}})json"_json, false, h_pcb_constraint_list );

KOPENAPI_REGISTER( "pcb_constraint",
                   "Edit geometric constraints as the constraint tool does. add: type (coincident, horizontal, "
                   "vertical, parallel, perpendicular, collinear, symmetric, equal_length, equal_radius, "
                   "point_on_line, midpoint, fixed_position, fixed_length, concentric, fixed_radius, "
                   "angular_dimension, tangent, arc_angle) from items (shape uuids, as the selection; dimensional "
                   "values measured) or members ([{item, anchor: whole|start|end|mid|center|radius|vertex, "
                   "index}], as click-to-pick), optional value (mm / deg) and driving; the geometry snaps in the "
                   "same undo step; an identical constraint is refused 409; answers the constraint and an issue "
                   "(all_locked, conflicting, not_applicable) with the editor's message. remove: uuid, or items "
                   "(every constraint referencing them); the clusters re-settle. set_value: uuid, value and / or "
                   "driving; the shapes follow a driving value",
                   R"json({"type":"object","required":["action"],"properties":{
                        "action":{"type":"string","enum":["add","remove","set_value"]},
                        "type":{"type":"string"},
                        "items":{"type":"array","items":{"type":"string"}},
                        "members":{"type":"array","items":{"type":"object","properties":{
                            "item":{"type":"string"},"anchor":{"type":"string","default":"whole"},
                            "index":{"type":"integer"}}}},
                        "uuid":{"type":"string"},
                        "value":{"type":"number"},
                        "driving":{"type":"boolean"}}})json"_json,
                   false, h_pcb_constraint );

KOPENAPI_MARK_EDITING( "pcb_constraint" );
KOPENAPI_MARK_CONCURRENT_READS( "pcb_constraint_list" );
