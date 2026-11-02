/// @file methods_pcb_match_properties.cpp
/// @brief kicadopenapi Match Properties: pcb_match_properties.
///
/// The editor's Match Properties (Edit menu): copy the enabled properties of one item onto
/// others of a compatible kind, through the same catalog and core Apply the tool uses; the
/// enabled set is the editor's (its settings dialog) unless the call names one.
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <board.h>
#include <board_commit.h>
#include <pcbnew_settings.h>
#include <pgm_base.h>
#include <settings/settings_manager.h>
#include <tool/tool_manager.h>
#include <tools/match_properties.h>

#include <algorithm>


namespace
{

std::string str( const wxString& aText )
{
    return aText.ToStdString( wxConvUTF8 );
}


nlohmann::json keyRow( const wxString& aKey, bool aEnabled )
{
    nlohmann::json families = nlohmann::json::array();

    for( const wxString& family : MATCH_PROPERTIES_CATALOG::FamiliesFor( aKey ) )
        families.push_back( str( family ) );

    return { { "key", str( aKey ) },
             { "label", str( MATCH_PROPERTIES_CATALOG::PropertyLabel( aKey ) ) },
             { "families", families },
             { "enabled", aEnabled } };
}

} // namespace


static KOPENAPI_RESULT h_pcb_match_properties( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*           board = context->GetBoard();
    PCBNEW_SETTINGS* settings = GetAppSettings<PCBNEW_SETTINGS>( "pcbnew" );

    std::set<wxString> enabled = settings ? settings->m_MatchProperties : MATCH_PROPERTIES_CATALOG::DefaultKeys();

    const std::string sourceName = aArgs.value( "source", std::string() );
    BOARD_ITEM*       source = sourceName.empty() ? nullptr : KopenapiFindBoardItem( board, sourceName );
    const wxString    family = source ? MATCH_PROPERTIES_CATALOG::Family( *source ) : wxString();

    if( !source )
        return KOPENAPI_RESULT::Error( 404, "source: a board item (ref, ref.pad or uuid)" );

    if( family.IsEmpty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 400, "the source has no properties to match" );
        refused.body["kind"] = str( source->GetClass() );
        return refused;
    }

    // The keys this kind of source offers (the settings dialog's rows)
    std::vector<wxString> offered;

    for( const wxString& key : MATCH_PROPERTIES_CATALOG::AllSafeKeys() )
    {
        if( MATCH_PROPERTIES_CATALOG::FamiliesFor( key ).contains( family ) )
            offered.push_back( key );
    }

    if( aArgs.contains( "properties" ) )
    {
        if( !aArgs["properties"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "properties: [key, ...] (list: true names them)" );

        std::set<wxString> asked;
        nlohmann::json     unknown = nlohmann::json::array();

        for( const nlohmann::json& k : aArgs["properties"] )
        {
            const wxString key = k.is_string() ? wxString::FromUTF8( k.get<std::string>() ) : wxString();

            if( std::find( offered.begin(), offered.end(), key ) == offered.end() )
                unknown.push_back( k );
            else
                asked.insert( key );
        }

        if( !unknown.empty() )
        {
            KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 400, "properties this kind of source does not offer (list: true)" );
            refused.body["unknown"] = unknown;
            return refused;
        }

        // Like the settings dialog: only this kind's rows are answered for
        if( aArgs.value( "remember", false ) )
        {
            if( !settings )
                return KOPENAPI_RESULT::Error( 409, "no board editor settings to remember them in" );

            for( const wxString& key : offered )
            {
                if( asked.contains( key ) )
                    settings->m_MatchProperties.insert( key );
                else
                    settings->m_MatchProperties.erase( key );
            }

            Pgm().GetSettingsManager().Save( settings );
        }

        enabled = std::move( asked );
    }
    else if( aArgs.value( "remember", false ) )
    {
        return KOPENAPI_RESULT::Error( 400, "remember needs properties" );
    }

    nlohmann::json answer = { { "source", str( source->m_Uuid.AsString() ) }, { "family", str( family ) } };

    if( aArgs.value( "list", false ) )
    {
        nlohmann::json rows = nlohmann::json::array();

        for( const wxString& key : offered )
            rows.push_back( keyRow( key, enabled.contains( key ) ) );

        answer["properties"] = rows;
        return KOPENAPI_RESULT::Ok( answer );
    }

    if( !aArgs.contains( "targets" ) || !aArgs["targets"].is_array() || aArgs["targets"].empty() )
        return KOPENAPI_RESULT::Error( 400, "targets: the items to copy to (refs, ref.pad or uuids), or list: true" );

    std::vector<EDA_ITEM* > candidates;
    nlohmann::json          missing = nlohmann::json::array();

    for( const nlohmann::json& t : aArgs["targets"] )
    {
        BOARD_ITEM* item = t.is_string() ? KopenapiFindBoardItem( board, t.get<std::string>() ) : nullptr;

        if( !item )
            missing.push_back( t );
        else if( item != source && std::find( candidates.begin(), candidates.end(), item ) == candidates.end() )
            candidates.push_back( item );
    }

    if( !missing.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 404, "targets not on the board" );
        refused.body["missing"] = missing;
        return refused;
    }

    std::vector<EDA_ITEM*> targets = MATCH_PROPERTIES_CATALOG::CompatibleTargets( *source, candidates );
    nlohmann::json         skipped = nlohmann::json::array();

    for( EDA_ITEM* item : candidates )
    {
        if( std::find( targets.begin(), targets.end(), item ) == targets.end() )
            skipped.push_back( str( item->m_Uuid.AsString() ) );
    }

    answer["skipped"] = skipped;

    if( targets.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 422, "no target has properties in common with the source" );
        refused.body["skipped"] = skipped;
        return refused;
    }

    if( !aArgs.value( "override_locks", false ) )
    {
        nlohmann::json locked = nlohmann::json::array();

        for( EDA_ITEM* item : targets )
        {
            if( static_cast<BOARD_ITEM*>( item )->IsLocked() )
                locked.push_back( str( item->m_Uuid.AsString() ) );
        }

        if( !locked.empty() )
        {
            KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "locked items are in the request (pcb_unlock, or override_locks: true)" );
            refused.body["locked"] = locked;
            return refused;
        }
    }

    BOARD_COMMIT            commit( context->GetToolManager() );
    std::vector<int>        perTarget;
    MATCH_PROPERTIES_RESULT result = MATCH_PROPERTIES_CATALOG::Apply( *source, targets, enabled, commit, &perTarget );

    if( !result )
    {
        commit.Revert();
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 422, str( result.m_Error ) );
        refused.body["target"] = str( targets[perTarget.size()]->m_Uuid.AsString() );
        return refused;
    }

    nlohmann::json rows = nlohmann::json::array();

    for( size_t ii = 0; ii < targets.size(); ++ii )
        rows.push_back( { { "uuid", str( targets[ii]->m_Uuid.AsString() ) }, { "changed", perTarget[ii] } } );

    answer["targets"] = rows;
    answer["changed"] = result.m_Changed;

    if( result.m_Changed == 0 )
    {
        commit.Revert();
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 422, "nothing to copy: the targets already match the source" );
        refused.body["targets"] = rows;
        refused.body["skipped"] = skipped;
        return refused;
    }

    commit.Push( _( "Match Properties (API)" ) );
    return KOPENAPI_RESULT::Ok( answer );
}


KOPENAPI_REGISTER( "pcb_match_properties",
                   "Match Properties (Edit menu): copy properties of one board item (source: ref, ref.pad "
                   "or uuid) onto others (targets) that share them, e.g. a track's width onto tracks, a "
                   "text's font and size onto texts, a via's sizes onto vias; targets of other kinds are "
                   "skipped. The enabled properties are the editor's (its Match Properties settings) "
                   "unless properties names keys; remember stores them as the editor's for this kind; "
                   "list answers the keys this kind offers and which are enabled. One undo step; answers "
                   "the properties changed per target (422 when every target already matches)",
                   R"json({"type":"object","required":["source"],"properties":{
                        "source":{"type":"string"},
                        "targets":{"type":"array","items":{"type":"string"}},
                        "properties":{"type":"array","items":{"type":"string"},"description":"keys like common/Line Width (list: true)"},
                        "remember":{"type":"boolean","default":false,"description":"store properties as the editor's enabled set for this kind"},
                        "list":{"type":"boolean","default":false},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_match_properties );

KOPENAPI_MARK_EDITING( "pcb_match_properties" );
