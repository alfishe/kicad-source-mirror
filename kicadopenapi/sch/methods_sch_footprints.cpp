/// @file methods_sch_footprints.cpp
/// @brief kicadopenapi symbol <-> footprint check: sch_footprint_check.
///
/// Before a board is made from the schematic: does every part have a footprint, does the
/// footprint exist, does every symbol pin find its pad (by number, like the netlist exporter)
/// and which numbered pads no pin uses.  Numbers only: a diode whose pins are 1 = K, 2 = A finds
/// pads 1 and 2 on any two-pad footprint; polarity is the footprint's convention (pad 1 = K for
/// KiCad's LED / diode footprints).
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"

#include <api/sch_context.h>
#include <kicadopenapi_util.h>
#include <lib_id.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>

#include <map>
#include <set>

using namespace kopenapi_sch;


static KOPENAPI_RESULT h_sch_footprint_check( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCHEMATIC*        schematic = context->GetSchematic();
    const std::string refGlob = aArgs.value( "ref", std::string() );
    const bool        onlyProblems = aArgs.value( "problems_only", true );
    PAD_RESOLVER      resolver( aCtx.kiway, &schematic->Project() );

    struct PART
    {
        std::string                         uuid;
        std::string                         footprint;
        std::map<std::string, std::string>  pinStatus;   // pin number -> mapped / unmapped / ...
        std::set<std::string>               padsUsed;
    };

    std::map<std::string, PART> parts;   // by reference (units of one part merge)

    for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( symbol->IsPower() || symbol->GetExcludedFromBoard() )
                continue;

            const std::string ref = str( symbol->GetRef( &path, false ) );

            if( !KopenapiGlob( refGlob, ref ) )
                continue;

            PART& part = parts[ref];

            if( part.uuid.empty() )
            {
                part.uuid = str( symbol->m_Uuid.AsString() );
                part.footprint = str( symbol->GetFootprintFieldText( &path, RESOLVED ) );
            }

            for( SCH_PIN* pin : symbol->GetPins( &path ) )
            {
                std::string status;
                std::vector<std::string> pads = resolver.Resolve( pin, path, status );
                part.pinStatus[str( pin->GetNumber() )] = status;
                part.padsUsed.insert( pads.begin(), pads.end() );
            }
        }
    }

    std::vector<nlohmann::json> rows;
    int                         problems = 0;

    for( const auto& [ref, part] : parts )
    {
        nlohmann::json row = { { "ref", ref }, { "uuid", part.uuid }, { "footprint", part.footprint } };
        std::string    status = "ok";

        if( part.footprint.empty() )
        {
            status = "no_footprint";
        }
        else
        {
            LIB_ID                    id;
            const std::set<wxString>* pads = nullptr;

            if( id.Parse( wxString::FromUTF8( part.footprint ), true ) < 0 )
                pads = &resolver.FootprintPads( id.GetUniStringLibId() );

            if( !pads || pads->empty() )
            {
                status = "footprint_not_found";
            }
            else
            {
                nlohmann::json missing = nlohmann::json::array();
                nlohmann::json unused = nlohmann::json::array();

                for( const auto& [pin, pinStatus] : part.pinStatus )
                {
                    if( pinStatus == "unmapped" )
                        missing.push_back( pin );
                }

                for( const wxString& pad : *pads )
                {
                    if( !part.padsUsed.count( str( pad ) ) )
                        unused.push_back( str( pad ) );
                }

                row["pins"] = part.pinStatus.size();
                row["pads"] = pads->size();

                if( !missing.empty() )
                {
                    status = "pins_without_pads";
                    row["pins_without_pads"] = missing;
                }

                // Unused pads are normal for some parts (thermal / mounting pads, NC pins left
                // off the symbol) — reported, not a problem by themselves
                if( !unused.empty() )
                    row["pads_without_pins"] = unused;
            }
        }

        row["status"] = status;

        if( status != "ok" )
            problems++;

        if( !onlyProblems || status != "ok" )
            rows.push_back( std::move( row ) );
    }

    nlohmann::json result = KopenapiPage( rows, aArgs );
    result["parts"] = parts.size();
    result["problems"] = problems;
    result["note"] = "pins match pads by number only: polarity (diode / LED K and A) is the footprint's convention";
    return KOPENAPI_RESULT::Ok( result );
}


KOPENAPI_REGISTER( "sch_footprint_check",
                   "Symbol <-> footprint check before making the board: per part, footprint missing / not "
                   "found in the libraries, symbol pins without a pad of that number, numbered pads no pin "
                   "uses; problems_only (default) lists only parts with a problem; filter by ref glob; "
                   "paginated",
                   KopenapiPagedSchema( R"json({"ref":{"type":"string"},
                        "problems_only":{"type":"boolean","default":true}})json"_json ),
                   false, h_sch_footprint_check, 600 );
