/// @file methods_pcb_tuning.cpp
/// @brief kicadopenapi length tuning: the editor's Tune Length (single track, differential pair, skew)
/// tuning patterns placed, listed, regenerated (Update All Tuning Patterns) and removed
#include "kopenapi_pcb.h"

#include <api/pcb_context.h>
#include <base_units.h>
#include <board.h>
#include <board_commit.h>
#include <eda_group.h>
#include <pcb_track.h>  // before the tuning pattern header, which needs it complete
#include <generators/pcb_tuning_pattern.h>
#include <kicadopenapi_keepalive.h>
#include <kicadopenapi_util.h>
#include <netinfo.h>
#include <tool/tool_manager.h>
#include <tools/generator_tool.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>


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


/// @brief The editor's generator tool: a GUI editor has one, a headless board gets one (registered
/// only, its router built by Reset; InitTools would re-initialise every other tool)
GENERATOR_TOOL* generatorTool( TOOL_MANAGER* aManager )
{
    wxASSERT( !KopenapiInComputeWorker() );

    if( GENERATOR_TOOL* tool = aManager->GetTool<GENERATOR_TOOL>() )
        return tool;

    aManager->RegisterTool( new GENERATOR_TOOL );

    GENERATOR_TOOL* tool = aManager->GetTool<GENERATOR_TOOL>();

    if( tool )
        tool->Reset( TOOL_BASE::RUN );

    return tool;
}


const char* modeName( LENGTH_TUNING_MODE aMode )
{
    switch( aMode )
    {
    case LENGTH_TUNING_MODE::DIFF_PAIR: return "diff_pair";
    case LENGTH_TUNING_MODE::DIFF_PAIR_SKEW: return "skew";
    default: return "single";
    }
}


const char* statusName( PNS::MEANDER_PLACER_BASE::TUNING_STATUS aStatus )
{
    switch( aStatus )
    {
    case PNS::MEANDER_PLACER_BASE::TOO_LONG: return "too_long";
    case PNS::MEANDER_PLACER_BASE::TOO_SHORT: return "too_short";
    default: return "tuned";
    }
}


/// @brief The pattern as listed: net, mode, layer, span, the last result and the target it tunes to
nlohmann::json patternJson( BOARD* aBoard, PCB_TUNING_PATTERN* aPattern )
{
    PNS::MEANDER_SETTINGS& s = aPattern->GetSettings();
    const bool             time = s.m_isTimeDomain;
    const bool             skew = aPattern->GetTuningMode() == LENGTH_TUNING_MODE::DIFF_PAIR_SKEW;
    nlohmann::json         target;

    auto range =
            [&]( long long aMin, long long aOpt, long long aMax ) -> nlohmann::json
            {
                if( time )
                    return { { "min_ps", aMin / pcbIUScale.IU_PER_PS }, { "opt_ps", aOpt / pcbIUScale.IU_PER_PS },
                             { "max_ps", aMax / pcbIUScale.IU_PER_PS } };

                return { { "min_mm", toMm( aMin ) }, { "opt_mm", toMm( aOpt ) }, { "max_mm", toMm( aMax ) } };
            };

    if( skew )
    {
        target = time ? range( s.m_targetSkewDelay.Min(), s.m_targetSkewDelay.Opt(), s.m_targetSkewDelay.Max() )
                      : range( s.m_targetSkew.Min(), s.m_targetSkew.Opt(), s.m_targetSkew.Max() );
    }
    else if( time )
    {
        if( s.m_targetLengthDelay.Opt() != PNS::MEANDER_SETTINGS::DELAY_UNCONSTRAINED )
            target = range( s.m_targetLengthDelay.Min(), s.m_targetLengthDelay.Opt(), s.m_targetLengthDelay.Max() );
    }
    else if( s.m_targetLength.Opt() != PNS::MEANDER_SETTINGS::LENGTH_UNCONSTRAINED )
    {
        target = range( s.m_targetLength.Min(), s.m_targetLength.Opt(), s.m_targetLength.Max() );
    }

    nlohmann::json row = { { "uuid", str( aPattern->m_Uuid.AsString() ) },
                           { "net", str( aPattern->GetLastNetName() ) },
                           { "mode", modeName( aPattern->GetTuningMode() ) },
                           { "layer", str( aBoard->GetLayerName( aPattern->GetLayer() ) ) },
                           { "from_mm", { toMm( aPattern->GetPosition().x ), toMm( aPattern->GetPosition().y ) } },
                           { "to_mm", { toMm( aPattern->GetEnd().x ), toMm( aPattern->GetEnd().y ) } },
                           { "status", statusName( aPattern->GetTuningStatus() ) },
                           { "target", target.is_null() ? nlohmann::json( "unconstrained" ) : target },
                           { "items", aPattern->GetBoardItems().size() },
                           { "locked", aPattern->IsLocked() },
                           { "settings",
                             { { "min_amplitude_mm", toMm( s.m_minAmplitude ) },
                               { "max_amplitude_mm", toMm( s.m_maxAmplitude ) },
                               { "spacing_mm", toMm( s.m_spacing ) },
                               { "corner", s.m_cornerStyle == PNS::MEANDER_STYLE_ROUND ? "round" : "chamfer" },
                               { "corner_radius_percent", s.m_cornerRadiusPercentage },
                               { "single_sided", s.m_singleSided },
                               { "side", s.m_initialSide == PNS::MEANDER_SIDE_RIGHT ? "right" : "left" } } } };

    if( time )
        row[skew ? "skew_ps" : "delay_ps"] = aPattern->GetTuningLength() / pcbIUScale.IU_PER_PS;
    else
        row[skew ? "skew_mm" : "length_mm"] = toMm( aPattern->GetTuningLength() );

    return row;
}


bool readPoint( const nlohmann::json& aArgs, const char* aKey, VECTOR2I& aOut )
{
    if( !aArgs.contains( aKey ) )
        return true;

    const nlohmann::json& p = aArgs[aKey];

    if( !p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number() )
        return false;

    aOut = VECTOR2I( fromMm( p[0].get<double>() ), fromMm( p[1].get<double>() ) );
    return true;
}


/// @brief The settings a call may change (amplitudes, spacing, corners, side, the target); an
/// error text when one is malformed
std::string applySettings( PCB_TUNING_PATTERN* aPattern, const nlohmann::json& aArgs )
{
    PNS::MEANDER_SETTINGS& s = aPattern->GetSettings();
    const bool             skew = aPattern->GetTuningMode() == LENGTH_TUNING_MODE::DIFF_PAIR_SKEW;

    for( const char* key : { "target_mm", "min_mm", "max_mm", "min_amplitude_mm", "max_amplitude_mm", "spacing_mm",
                             "corner_radius_percent" } )
    {
        if( aArgs.contains( key ) && !aArgs[key].is_number() )
            return std::string( key ) + ": a number";
    }

    if( aArgs.contains( "target_mm" ) || aArgs.contains( "min_mm" ) || aArgs.contains( "max_mm" ) )
    {
        const double lo = aArgs.value( "min_mm", aArgs.value( "target_mm", 0.0 ) );
        const double hi = aArgs.value( "max_mm", aArgs.value( "target_mm", 0.0 ) );
        const double opt = aArgs.value( "target_mm", ( lo + hi ) / 2 );

        if( ( !skew && ( lo < 0 || opt <= 0 ) ) || hi < lo || opt < lo || opt > hi )
            return "target_mm / min_mm / max_mm: min <= target <= max" + std::string( skew ? "" : ", target > 0" );

        if( skew )
        {
            s.SetTargetSkew( fromMm( opt ) );

            if( aArgs.contains( "min_mm" ) || aArgs.contains( "max_mm" ) )
            {
                s.m_targetSkew.SetMin( fromMm( lo ) );
                s.m_targetSkew.SetMax( fromMm( hi ) );
            }

            s.SetTargetSkewDelay( MINOPTMAX<int>() );
        }
        else
        {
            aPattern->SetTargetLength( fromMm( opt ) );

            if( aArgs.contains( "min_mm" ) || aArgs.contains( "max_mm" ) )
            {
                s.m_targetLength.SetMin( fromMm( lo ) );
                s.m_targetLength.SetMax( fromMm( hi ) );
            }
        }

        s.m_isTimeDomain = false;
        s.m_overrideCustomRules = true;
    }

    if( aArgs.contains( "min_amplitude_mm" ) )
        aPattern->SetMinAmplitude( fromMm( aArgs["min_amplitude_mm"].get<double>() ) );

    if( aArgs.contains( "max_amplitude_mm" ) )
        aPattern->SetMaxAmplitude( fromMm( aArgs["max_amplitude_mm"].get<double>() ) );

    if( aArgs.contains( "spacing_mm" ) )
    {
        if( aArgs["spacing_mm"].get<double>() <= 0 )
            return "spacing_mm: > 0";

        aPattern->SetSpacing( fromMm( aArgs["spacing_mm"].get<double>() ) );
    }

    if( aArgs.contains( "corner" ) )
    {
        const std::string corner = aArgs["corner"].is_string() ? aArgs["corner"].get<std::string>() : "";

        if( corner != "round" && corner != "chamfer" )
            return "corner: round or chamfer";

        aPattern->SetRounded( corner == "round" );
    }

    if( aArgs.contains( "corner_radius_percent" ) )
    {
        const int pct = aArgs["corner_radius_percent"].get<int>();

        if( pct < 0 || pct > 100 )
            return "corner_radius_percent: 0..100";

        aPattern->SetCornerRadiusPercentage( pct );
    }

    if( aArgs.contains( "single_sided" ) )
    {
        if( !aArgs["single_sided"].is_boolean() )
            return "single_sided: true or false";

        aPattern->SetSingleSided( aArgs["single_sided"].get<bool>() );
    }

    if( aArgs.contains( "side" ) )
    {
        const std::string side = aArgs["side"].is_string() ? aArgs["side"].get<std::string>() : "";

        if( side != "left" && side != "right" )
            return "side: left or right (of the line from from_mm towards to_mm)";

        aPattern->SetInitialSide( side == "left" ? PNS::MEANDER_SIDE_LEFT : PNS::MEANDER_SIDE_RIGHT );
    }

    return std::string();
}


/// @brief Regenerate one pattern into aCommit as the editor does after an edit
bool regenerate( GENERATOR_TOOL* aTool, BOARD* aBoard, BOARD_COMMIT& aCommit, PCB_TUNING_PATTERN* aPattern )
{
    aPattern->EditStart( aTool, aBoard, &aCommit );
    const bool ok = aPattern->Update( aTool, aBoard, &aCommit );
    aPattern->EditFinish( aTool, aBoard, &aCommit );
    return ok;
}


PCB_TRACK* pickTrack( BOARD* aBoard, const nlohmann::json& aArgs, std::string& aError )
{
    if( aArgs.contains( "track" ) )
    {
        BOARD_ITEM* item = aArgs["track"].is_string() ? KopenapiFindBoardItem( aBoard, aArgs["track"].get<std::string>() )
                                                      : nullptr;

        if( !item || ( item->Type() != PCB_TRACE_T && item->Type() != PCB_ARC_T ) )
            aError = "track: no track or arc with that uuid (pcb_track_list)";

        return aError.empty() ? static_cast<PCB_TRACK*>( item ) : nullptr;
    }

    if( !aArgs.contains( "net" ) || !aArgs["net"].is_string() )
    {
        aError = "track (uuid) or net (+ at_mm): the line to tune";
        return nullptr;
    }

    NETINFO_ITEM* net = KopenapiFindNet( aBoard, aArgs["net"].get<std::string>() );

    if( !net )
    {
        aError = "net: no net " + aArgs["net"].get<std::string>();
        return nullptr;
    }

    VECTOR2I   at;
    const bool hasAt = aArgs.contains( "at_mm" );

    if( !readPoint( aArgs, "at_mm", at ) )
    {
        aError = "at_mm: [x, y]";
        return nullptr;
    }

    PCB_TRACK* best = nullptr;
    double     bestScore = std::numeric_limits<double>::max();

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->GetNetCode() != net->GetNetCode() || track->Type() == PCB_VIA_T )
            continue;

        // nearest to at_mm, else the longest
        const double score = hasAt ? track->GetEffectiveShape()->SquaredDistance( at ) : -track->GetLength();

        if( score < bestScore )
        {
            bestScore = score;
            best = track;
        }
    }

    if( !best )
        aError = "net " + aArgs["net"].get<std::string>() + " has no tracks to tune";

    return best;
}


KOPENAPI_RESULT h_pcb_tune_length( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*      board = context->GetBoard();
    std::string error;
    PCB_TRACK*  track = pickTrack( board, aArgs, error );

    if( !track )
        return KOPENAPI_RESULT::Error( error.rfind( "net:", 0 ) == 0 || error.rfind( "track:", 0 ) == 0 ? 404 : 400,
                                       error );

    if( dynamic_cast<PCB_TUNING_PATTERN*>( track->GetParentGroup() ) )
    {
        KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "the track is inside a tuning pattern: pcb_tuning_update "
                                                         "(change or remove it)" );
        r.body["pattern"] = str( track->GetParentGroup()->AsEdaItem()->m_Uuid.AsString() );
        return r;
    }

    const std::string modeArg = aArgs.value( "mode", std::string( "single" ) );
    LENGTH_TUNING_MODE mode = LENGTH_TUNING_MODE::SINGLE;

    if( modeArg == "diff_pair" )
        mode = LENGTH_TUNING_MODE::DIFF_PAIR;
    else if( modeArg == "skew" )
        mode = LENGTH_TUNING_MODE::DIFF_PAIR_SKEW;
    else if( modeArg != "single" )
        return KOPENAPI_RESULT::Error( 400, "mode: single, diff_pair or skew" );

    VECTOR2I from = track->GetStart();
    VECTOR2I to = track->GetEnd();

    if( !readPoint( aArgs, "from_mm", from ) || !readPoint( aArgs, "to_mm", to ) )
        return KOPENAPI_RESULT::Error( 400, "from_mm / to_mm: [x, y] along the line" );

    if( from == to )
        return KOPENAPI_RESULT::Error( 400, "from_mm / to_mm: the span is empty" );

    GENERATOR_TOOL* tool = generatorTool( context->GetToolManager() );

    if( !tool || !tool->Router() )
        return KOPENAPI_RESULT::Error( 503, "the generator tool is not available" );

    // as the editor's tool: settings from Board Setup and the length / skew rules, then the call's
    std::unique_ptr<PCB_TUNING_PATTERN> pattern( PCB_TUNING_PATTERN::CreateNew( tool, nullptr, track, mode ) );

    pattern->GetSettings().m_signalExtraLength = 0;
    pattern->GetSettings().m_signalExtraDelay = 0;

    int      dist = 0;
    VECTOR2I snapped = from;

    if( track->GetEffectiveShape()->Collide( from, std::numeric_limits<int>::max() / 2, &dist, &snapped ) )
        from = snapped;

    pattern->SetPosition( from );
    pattern->SetEnd( to );

    if( !aArgs.contains( "side" ) )
        pattern->UpdateSideFromEnd();

    if( std::string bad = applySettings( pattern.get(), aArgs ); !bad.empty() )
        return KOPENAPI_RESULT::Error( 400, bad );

    const PNS::MEANDER_SETTINGS& s = pattern->GetSettings();

    if( mode != LENGTH_TUNING_MODE::DIFF_PAIR_SKEW && !s.m_isTimeDomain
        && s.m_targetLength.Opt() == PNS::MEANDER_SETTINGS::LENGTH_UNCONSTRAINED )
    {
        return KOPENAPI_RESULT::Error( 422, "no target: target_mm (or min_mm / max_mm), or a length rule on the net" );
    }

    const bool   dryRun = aArgs.value( "dry_run", false );
    BOARD_COMMIT commit( context->GetToolManager() );

    PCB_TUNING_PATTERN* placed = pattern.release();   // the commit owns it from EditStart on
    bool                ok = false;

    try
    {
        ok = regenerate( tool, board, commit, placed );
    }
    catch( const std::exception& e )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Error( 500, std::string( "tuning failed: " ) + e.what() );
    }

    nlohmann::json row = patternJson( board, placed );

    if( !ok || placed->GetBoardItems().empty() )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Error( 422, "the router could not tune this line here (no baseline between from_mm "
                                            "and to_mm on " + str( board->GetLayerName( track->GetLayer() ) )
                                            + ( mode == LENGTH_TUNING_MODE::SINGLE ? std::string( ")" )
                                                                                   : std::string( ", or not a coupled pair)" ) ) );
    }

    if( dryRun )
    {
        commit.Revert();
        row.erase( "uuid" );
        row["dry_run"] = true;
        return KOPENAPI_RESULT::Ok( row );
    }

    commit.Push( _( "Tune" ) );
    context->SetContentModified();
    row["dry_run"] = false;

    if( row["status"] != "tuned" )
        row["next"] = "the line cannot reach the target within this span: a longer span (from_mm / to_mm), larger "
                      "amplitude or tighter spacing (pcb_tuning_update)";

    return KOPENAPI_RESULT::Ok( row );
}


std::vector<PCB_TUNING_PATTERN*> patterns( BOARD* aBoard )
{
    std::vector<PCB_TUNING_PATTERN*> out;

    for( PCB_GENERATOR* generator : aBoard->Generators() )
    {
        if( PCB_TUNING_PATTERN* pattern = dynamic_cast<PCB_TUNING_PATTERN*>( generator ) )
            out.push_back( pattern );
    }

    return out;
}


KOPENAPI_RESULT h_pcb_tuning_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    const std::string net = aArgs.value( "net", std::string() );
    nlohmann::json    items = nlohmann::json::array();

    for( PCB_TUNING_PATTERN* pattern : patterns( context->GetBoard() ) )
    {
        if( net.empty() || str( pattern->GetLastNetName() ) == net )
            items.push_back( patternJson( context->GetBoard(), pattern ) );
    }

    return KOPENAPI_RESULT::Ok( { { "count", items.size() }, { "items", items } } );
}


KOPENAPI_RESULT h_pcb_tuning_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<PCB_CONTEXT> context = KopenapiPcbContext( aCtx );

    if( !context )
        return KopenapiNoBoard();

    BOARD*                           board = context->GetBoard();
    std::vector<PCB_TUNING_PATTERN*> chosen;
    nlohmann::json                   missing = nlohmann::json::array();

    if( aArgs.contains( "uuids" ) )
    {
        if( !aArgs["uuids"].is_array() )
            return KOPENAPI_RESULT::Error( 400, "uuids: [pattern uuid, ...] (pcb_tuning_list)" );

        for( const nlohmann::json& id : aArgs["uuids"] )
        {
            BOARD_ITEM* item = id.is_string() ? KopenapiFindBoardItem( board, id.get<std::string>() ) : nullptr;

            if( PCB_TUNING_PATTERN* pattern = dynamic_cast<PCB_TUNING_PATTERN*>( item ) )
                chosen.push_back( pattern );
            else
                missing.push_back( id );
        }

        if( chosen.empty() )
            return KOPENAPI_RESULT::Error( 404, "uuids: no tuning pattern among them (pcb_tuning_list)" );
    }
    else
    {
        chosen = patterns( board );
    }

    const bool remove = aArgs.value( "remove", false );

    if( !aArgs.value( "override_locks", false ) )
    {
        nlohmann::json locked = nlohmann::json::array();

        for( PCB_TUNING_PATTERN* pattern : chosen )
        {
            if( pattern->IsLocked() )
                locked.push_back( str( pattern->m_Uuid.AsString() ) );
        }

        if( !locked.empty() )
        {
            KOPENAPI_RESULT r = KOPENAPI_RESULT::Error( 409, "locked tuning patterns (override_locks: true)" );
            r.body["locked"] = locked;
            return r;
        }
    }

    GENERATOR_TOOL* tool = generatorTool( context->GetToolManager() );

    if( !tool || !tool->Router() )
        return KOPENAPI_RESULT::Error( 503, "the generator tool is not available" );

    BOARD_COMMIT   commit( context->GetToolManager() );
    nlohmann::json items = nlohmann::json::array();
    nlohmann::json failed = nlohmann::json::array();

    for( PCB_TUNING_PATTERN* pattern : chosen )
    {
        const std::string id = str( pattern->m_Uuid.AsString() );

        if( remove )
        {
            pattern->Remove( tool, board, &commit );
            items.push_back( { { "uuid", id }, { "removed", true } } );
            continue;
        }

        commit.Modify( pattern );

        if( std::string bad = applySettings( pattern, aArgs ); !bad.empty() )
        {
            commit.Revert();
            return KOPENAPI_RESULT::Error( 400, bad );
        }

        if( !regenerate( tool, board, commit, pattern ) )
            failed.push_back( id );

        items.push_back( patternJson( board, pattern ) );
    }

    if( aArgs.value( "dry_run", false ) )
    {
        commit.Revert();
        return KOPENAPI_RESULT::Ok( { { "items", items }, { "failed", failed }, { "dry_run", true } } );
    }

    commit.Push( remove ? _( "Remove Tuning Patterns" ) : _( "Update Tuning Patterns" ) );
    context->SetContentModified();

    nlohmann::json out = { { "count", items.size() }, { "items", items }, { "failed", failed }, { "dry_run", false } };

    if( !missing.empty() )
        out["missing"] = missing;

    return KOPENAPI_RESULT::Ok( out );
}

} // namespace


KOPENAPI_REGISTER( "pcb_tune_length",
                   "Tune Length as the editor (Route > Tune Length / Diff Pair Length / Diff Pair Skew): a tuning "
                   "pattern on a track (uuid) or a net's track (nearest at_mm, else the longest), meandering the "
                   "line between from_mm and to_mm (default the track's ends; the meanders go to the side to_mm "
                   "lies on unless side). Target: target_mm (or min_mm / max_mm), else the net's length / skew rule "
                   "(422 without either); mode single / diff_pair / skew; min/max_amplitude_mm, spacing_mm, corner "
                   "round / chamfer, corner_radius_percent, single_sided. Answers the pattern (uuid, length_mm or "
                   "skew_mm, status tuned / too_short / too_long, target, items); dry_run previews. One undo step",
                   R"json({"type":"object","properties":{
                        "track":{"type":"string","description":"uuid of a track or arc (pcb_track_list)"},
                        "net":{"type":"string"},"at_mm":{"type":"array","items":{"type":"number"}},
                        "from_mm":{"type":"array","items":{"type":"number"}},"to_mm":{"type":"array","items":{"type":"number"}},
                        "mode":{"type":"string","enum":["single","diff_pair","skew"],"default":"single"},
                        "target_mm":{"type":"number"},"min_mm":{"type":"number"},"max_mm":{"type":"number"},
                        "min_amplitude_mm":{"type":"number"},"max_amplitude_mm":{"type":"number"},
                        "spacing_mm":{"type":"number"},"corner":{"type":"string","enum":["round","chamfer"]},
                        "corner_radius_percent":{"type":"integer"},"single_sided":{"type":"boolean"},
                        "side":{"type":"string","enum":["left","right"]},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_tune_length );

KOPENAPI_REGISTER( "pcb_tuning_list",
                   "Tuning patterns on the board (the editor's Generators Manager): uuid, net, mode, layer, span, "
                   "the last tuned length / skew and status, target, member items, settings; filter by net",
                   R"json({"type":"object","properties":{"net":{"type":"string"}}})json"_json,
                   false, h_pcb_tuning_list );

KOPENAPI_REGISTER( "pcb_tuning_update",
                   "Regenerate tuning patterns (all, or uuids) as Update All Tuning Patterns, optionally with new "
                   "settings (target_mm / min_mm / max_mm, amplitudes, spacing_mm, corner, side, single_sided); "
                   "remove: true takes them out and restores the line. Locked ones 409 unless override_locks. One "
                   "undo step; dry_run previews; answers the patterns (failed: the router could not regenerate)",
                   R"json({"type":"object","properties":{
                        "uuids":{"type":"array","items":{"type":"string"}},
                        "remove":{"type":"boolean","default":false},
                        "target_mm":{"type":"number"},"min_mm":{"type":"number"},"max_mm":{"type":"number"},
                        "min_amplitude_mm":{"type":"number"},"max_amplitude_mm":{"type":"number"},
                        "spacing_mm":{"type":"number"},"corner":{"type":"string","enum":["round","chamfer"]},
                        "corner_radius_percent":{"type":"integer"},"single_sided":{"type":"boolean"},
                        "side":{"type":"string","enum":["left","right"]},
                        "override_locks":{"type":"boolean","default":false},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_pcb_tuning_update );

KOPENAPI_MARK_EDITING( "pcb_tune_length", "pcb_tuning_update" );
