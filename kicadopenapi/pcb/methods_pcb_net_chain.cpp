/// @file methods_pcb_net_chain.cpp
/// @brief kicadopenapi net chains: the chains of the board (nets joined through series parts, from the
/// schematic) with their terminal pads, and Set Terminal Pad (BOARD::SetNetChainTerminalPad)
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <footprint.h>
#include <frame_type.h>
#include <kiway.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_edit_frame.h>
#include <string_utils.h>
#include <view/view.h>

#include <map>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


nlohmann::json padJson( PAD* aPad )
{
    if( !aPad )
        return nlohmann::json();

    const FOOTPRINT* fp = aPad->GetParentFootprint();
    const std::string ref = fp ? str( fp->GetReference() ) : std::string();

    return { { "pin", ref + "." + str( aPad->GetNumber() ) }, { "uuid", str( aPad->m_Uuid.AsString() ) },
             { "net", str( UnescapeString( aPad->GetNetname() ) ) } };
}


/// @brief Chain name -> its nets in net code order
std::map<wxString, std::vector<NETINFO_ITEM*>> chainsOf( BOARD* aBoard )
{
    std::map<wxString, std::vector<NETINFO_ITEM*>> chains;

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
    {
        if( net && net->GetNetCode() > 0 && !net->GetNetChain().IsEmpty() )
            chains[net->GetNetChain()].push_back( net );
    }

    return chains;
}


nlohmann::json chainJson( const wxString& aName, const std::vector<NETINFO_ITEM*>& aNets )
{
    nlohmann::json nets = nlohmann::json::array();
    PAD*           terminals[2] = { nullptr, nullptr };

    for( NETINFO_ITEM* net : aNets )
    {
        nets.push_back( str( UnescapeString( net->GetNetname() ) ) );

        for( int i = 0; i < 2; ++i )
        {
            if( !terminals[i] )
                terminals[i] = net->GetTerminalPad( i );
        }
    }

    return { { "name", str( aName ) }, { "nets", nets }, { "terminal_a", padJson( terminals[0] ) },
             { "terminal_b", padJson( terminals[1] ) } };
}


/// @brief A pad by "REF.PAD" or uuid; aError says why not
PAD* findPad( BOARD* aBoard, const std::string& aPin, std::string& aError )
{
    if( aPin.find( '.' ) == std::string::npos )
    {
        if( PAD* pad = aBoard->FindPadByUuid( KIID( wxString::FromUTF8( aPin ) ) ) )
            return pad;
    }

    const size_t dot = aPin.rfind( '.' );

    if( dot != std::string::npos && dot > 0 )
    {
        const wxString ref = wxString::FromUTF8( aPin.substr( 0, dot ) );
        const wxString number = wxString::FromUTF8( aPin.substr( dot + 1 ) );
        PAD*           found = nullptr;

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            if( fp->GetReference() != ref )
                continue;

            if( found )
            {
                aError = "pad: two footprints are " + aPin.substr( 0, dot ) + " (give the pad's uuid)";
                return nullptr;
            }

            found = fp->FindPadByNumber( number );
        }

        if( found )
            return found;
    }

    aError = "pad: no pad " + aPin + " (REF.PAD or a pad uuid)";
    return nullptr;
}


KOPENAPI_RESULT h_pcb_net_chain_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const wxString only = wxString::FromUTF8( aArgs.value( "chain", std::string() ) );
    nlohmann::json items = nlohmann::json::array();

    for( const auto& [name, nets] : chainsOf( context->GetBoard() ) )
    {
        if( only.IsEmpty() || only == name )
            items.push_back( chainJson( name, nets ) );
    }

    if( !only.IsEmpty() && items.empty() )
        return KOPENAPI_RESULT::Error( 404, "no net chain " + str( only ) );

    return KOPENAPI_RESULT::Ok( { { "items", items }, { "total", items.size() } } );
}


KOPENAPI_RESULT h_pcb_net_chain_set_terminal( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD* board = context->GetBoard();
    auto   chains = chainsOf( board );

    const std::string terminal = aArgs.value( "terminal", std::string() );

    if( terminal != "A" && terminal != "B" )
        return KOPENAPI_RESULT::Error( 400, "terminal: A or B" );

    if( !aArgs.contains( "pad" ) || !( aArgs["pad"].is_string() || aArgs["pad"].is_null() ) )
        return KOPENAPI_RESULT::Error( 400, "pad: REF.PAD or a pad uuid of a net of the chain; null clears the terminal (with chain)" );

    PAD*     pad = nullptr;
    wxString chain = wxString::FromUTF8( aArgs.value( "chain", std::string() ) );

    if( aArgs["pad"].is_string() )
    {
        std::string error;
        pad = findPad( board, aArgs["pad"].get<std::string>(), error );

        if( !pad )
            return KOPENAPI_RESULT::Error( 404, error );

        const wxString padChain = pad->GetNet() ? pad->GetNet()->GetNetChain() : wxString();

        if( padChain.IsEmpty() )
            return KOPENAPI_RESULT::Error( 409, "pad " + aArgs["pad"].get<std::string>() + ": its net is in no net chain" );

        if( !chain.IsEmpty() && chain != padChain )
        {
            return KOPENAPI_RESULT::Error( 409, "pad " + aArgs["pad"].get<std::string>() + " is in net chain " + str( padChain )
                                                + ", not " + str( chain ) );
        }

        chain = padChain;
    }
    else if( chain.IsEmpty() )
    {
        return KOPENAPI_RESULT::Error( 400, "chain: the net chain whose terminal is cleared" );
    }

    if( !chains.count( chain ) )
        return KOPENAPI_RESULT::Error( 404, "no net chain " + str( chain ) );

    const int index = terminal == "A" ? 0 : 1;
    PAD*      previous = nullptr;

    for( NETINFO_ITEM* net : chains[chain] )
    {
        if( !previous )
            previous = net->GetTerminalPad( index );
    }

    const int changed = board->SetNetChainTerminalPad( chain, index, pad );

    if( changed > 0 )
    {
        context->SetContentModified();

        if( !aCtx.headless && aCtx.kiway )
        {
            if( auto* frame = dynamic_cast<PCB_EDIT_FRAME*>( aCtx.kiway->Player( FRAME_PCB_EDITOR, false ) ) )
            {
                for( PAD* repaint : { previous, pad } )
                {
                    if( repaint )
                        frame->GetCanvas()->GetView()->Update( repaint, KIGFX::REPAINT );
                }

                frame->GetCanvas()->Refresh();
            }
        }
    }

    nlohmann::json answer = chainJson( chain, chainsOf( board )[chain] );
    answer["changed_nets"] = changed;
    answer["previous"] = padJson( previous );
    answer["undoable"] = false;
    return KOPENAPI_RESULT::Ok( answer );
}

} // namespace


KOPENAPI_REGISTER( "pcb_net_chain_list",
                   "Net chains (nets joined through series parts, from the schematic): each chain's nets and its "
                   "terminal pads A and B (pin REF.PAD, uuid, net); chain for one",
                   R"json({"type":"object","properties":{
                        "chain":{"type":"string","description":"only this chain"}}})json"_json,
                   false, h_pcb_net_chain_list );

KOPENAPI_REGISTER( "pcb_net_chain_set_terminal",
                   "Set Terminal Pad: terminal A or B of the net chain of the pad's net becomes the pad (REF.PAD or "
                   "uuid); pad null with chain clears it. Not an undo step in the editor (as there); the board is "
                   "marked modified",
                   R"json({"type":"object","required":["terminal","pad"],"properties":{
                        "chain":{"type":"string","description":"the chain (checked against the pad's; needed to clear)"},
                        "terminal":{"type":"string","enum":["A","B"]},
                        "pad":{"type":["string","null"],"description":"REF.PAD or a pad uuid; null clears the terminal"}}})json"_json,
                   false, h_pcb_net_chain_set_terminal );

KOPENAPI_MARK_EDITING( "pcb_net_chain_set_terminal" );
