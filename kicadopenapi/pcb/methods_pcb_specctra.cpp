/// @file methods_pcb_specctra.cpp
/// @brief kicadopenapi Specctra exchange: pcb_export_dsn, pcb_import_ses.
///
/// The board out as a Specctra design (.dsn) for an external autorouter and its session (.ses)
/// back in, with KiCad's own exporter / importer; the import is one undo step.
#include "kopenapi_pcb.h"

#include <netinfo.h>

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <connectivity/connectivity_data.h>
#include <pcb_track.h>
#include <specctra_import_export/specctra.h>
#include <tool/tool_manager.h>

#include <set>


/// @brief A commit that takes only the session's copper of the given nets; the rest is dropped
class NETS_ONLY_COMMIT : public BOARD_COMMIT
{
public:
    NETS_ONLY_COMMIT( TOOL_MANAGER* aMgr, const std::set<wxString>& aNets ) :
            BOARD_COMMIT( aMgr ),
            m_nets( aNets )
    {
    }

    using BOARD_COMMIT::Stage;

    COMMIT& Stage( EDA_ITEM* aItem, CHANGE_TYPE aChangeType, BASE_SCREEN* aScreen = nullptr,
                   RECURSE_MODE aRecurse = RECURSE_MODE::NO_RECURSE ) override
    {
        if( ( aChangeType & CHT_TYPE ) == CHT_ADD && aItem->Type() != PCB_FOOTPRINT_T )
        {
            if( auto* item = dynamic_cast<BOARD_CONNECTED_ITEM*>( aItem ); item && !m_nets.count( item->GetNetname() ) )
            {
                delete aItem;
                m_dropped++;
                return *this;
            }
        }

        return BOARD_COMMIT::Stage( aItem, aChangeType, aScreen, aRecurse );
    }

    int Dropped() const { return m_dropped; }

private:
    std::set<wxString> m_nets;
    int                m_dropped = 0;
};


static KOPENAPI_RESULT h_pcb_export_dsn( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const std::string path = aArgs.value( "path", std::string() );

    if( path.empty() )
        return KOPENAPI_RESULT::Error( 400, "path: the .dsn file to write" );

    try
    {
        DSN::ExportBoardToSpecctraFile( context->GetBoard(), wxString::FromUTF8( path ) );
    }
    catch( const IO_ERROR& ioe )
    {
        return KOPENAPI_RESULT::Error( 422, "export failed: " + ioe.What().ToStdString() );
    }

    return KOPENAPI_RESULT::Ok( { { "path", path } } );
}


static KOPENAPI_RESULT h_pcb_import_ses( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const std::string path = aArgs.value( "path", std::string() );

    if( path.empty() )
        return KOPENAPI_RESULT::Error( 400, "path: the .ses file to read" );

    BOARD*             board = context->GetBoard();
    std::set<wxString> nets;

    for( const nlohmann::json& n : aArgs.value( "nets", nlohmann::json::array() ) )
    {
        if( n.is_string() )
        {
            NETINFO_ITEM* net = KopenapiFindNet( board, n.get<std::string>() );

            if( !net )
                return KopenapiNoSuchNet( board, n.get<std::string>(), "nets: " );

            nets.insert( net->GetNetname() );
        }
    }

    // nets given: the other nets' copper stays as it is (locked meanwhile, the import keeps locked
    // copper) and the session's copper of other nets is not taken
    std::vector<PCB_TRACK*> lockedHere;

    if( !nets.empty() )
    {
        for( PCB_TRACK* t : board->Tracks() )
        {
            if( !nets.count( t->GetNetname() ) && !t->IsLocked() )
            {
                t->SetLocked( true );
                lockedHere.push_back( t );
            }
        }
    }

    NETS_ONLY_COMMIT commit( context->GetToolManager(), nets );
    BOARD_COMMIT     all( context->GetToolManager() );
    BOARD_COMMIT&    used = nets.empty() ? all : commit;
    std::string      failure;

    try
    {
        DSN::ImportSpecctraSession( board, wxString::FromUTF8( path ), used );
    }
    catch( const IO_ERROR& ioe )
    {
        failure = ioe.What().ToStdString();
    }

    for( PCB_TRACK* t : lockedHere )
        t->SetLocked( false );

    if( !failure.empty() )
    {
        used.Revert();
        return KOPENAPI_RESULT::Error( 422, "import failed: " + failure );
    }

    used.Push( _( "Import Specctra session (API)" ) );
    board->BuildConnectivity();

    int tracks = 0, vias = 0;

    for( PCB_TRACK* t : board->Tracks() )
        ( t->Type() == PCB_VIA_T ? vias : tracks )++;

    nlohmann::json answer = { { "path", path },
                              { "tracks", tracks },
                              { "vias", vias },
                              { "other_nets_dropped", commit.Dropped() } };
    answer.update( KopenapiUnroutedJson( board ) );
    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_export_dsn",
                   "Write the open board as a Specctra design (.dsn) for an external autorouter",
                   R"json({"type":"object","required":["path"],"properties":{"path":{"type":"string"}}})json"_json,
                   false, h_pcb_export_dsn, 300 );

KOPENAPI_REGISTER( "pcb_import_ses",
                   "Read a Specctra session (.ses) back onto the open board (its routes replace the "
                   "board's tracks and vias; with nets: only those nets' copper is taken, the rest of the board "
                   "stays as it is), one undo step",
                   R"json({"type":"object","required":["path"],"properties":{"path":{"type":"string"},
                        "nets":{"type":"array","items":{"type":"string"}}}})json"_json,
                   false, h_pcb_import_ses, 300 );

KOPENAPI_MARK_EDITING( "pcb_import_ses" );
