/*
 * kicadopenapi schematic edits (step d4): sch_new, sch_symbol_add, sch_symbol_update,
 * sch_item_delete, sch_connect, sch_no_connect, sch_annotate.
 *
 * Every method is one SCH_COMMIT through the context's tool manager: in the GUI it is one undo
 * step, repaints and marks the document modified, like an edit made by hand; headless the
 * commit recalculates connectivity the same way.  Connections are made with labels (or power
 * symbols) on pins, so an agent never routes wires geometrically.  Coordinates are mm on the
 * 1.27 mm (50 mil) connection grid.
 */
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"
#include "kopenapi_sch_router.h"

#include <api/sch_context.h>
#include <base_units.h>
#include <kicadopenapi_registry.h>
#include <kicadopenapi_util.h>
#include <kiway.h>
#include <lib_symbol.h>
#include <libraries/symbol_library_adapter.h>
#include <project_sch.h>
#include <sch_commit.h>
#include <sch_edit_frame.h>
#include <sch_junction.h>
#include <sch_line.h>
#include <sch_field.h>
#include <sch_file_versions.h>
#include <sch_label.h>
#include <sch_no_connect.h>
#include <sch_pin.h>
#include <sch_reference_list.h>
#include <sch_screen.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <string_utils.h>
#include <tool/tool_manager.h>
#include <view/view.h>
#include <view/view_overlay.h>
#include <gal/painter.h>
#include <layer_ids.h>

#include <wx/filename.h>
#include <wx/ffile.h>
#include <wx/timer.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <map>

using namespace kopenapi_sch;


namespace
{

constexpr double GRID_MM = 1.27;


int toIU( double aMm )
{
    return schIUScale.mmToIU( std::round( aMm / GRID_MM ) * GRID_MM );
}


double toMm( int aIU )
{
    return std::round( schIUScale.IUTomm( aIU ) * 10000.0 ) / 10000.0;
}


/// Target sheet instance: args "sheet" (human path as listed by sch_sheet_list), else the
/// editor's current sheet, else the root
std::optional<SCH_SHEET_PATH> targetSheet( SCH_CONTEXT& aContext, const nlohmann::json& aArgs )
{
    const SCH_SHEET_LIST hierarchy = aContext.GetSchematic()->Hierarchy();

    if( hierarchy.empty() )
        return std::nullopt;

    const std::string wanted = aArgs.value( "sheet", std::string() );

    if( wanted.empty() )
        return aContext.GetCurrentSheet().value_or( hierarchy.front() );

    for( const SCH_SHEET_PATH& path : hierarchy )
    {
        if( sheetPath( path ) == wanted || ( wanted == "/" && path.size() == hierarchy.front().size() ) )
            return path;
    }

    return std::nullopt;
}


LIB_SYMBOL* librarySymbol( KOPENAPI_CONTEXT& aCtx, SCH_CONTEXT& aContext, const std::string& aLibId, std::string& aError )
{
    LIB_ID id;

    if( id.Parse( wxString::FromUTF8( aLibId ) ) >= 0 || id.GetLibNickname().empty() )
    {
        aError = "lib_id must be LIBRARY:SYMBOL";
        return nullptr;
    }

    SYMBOL_LIBRARY_ADAPTER* adapter = PROJECT_SCH::SymbolLibAdapter( &aContext.GetSchematic()->Project() );
    adapter->AsyncLoad();
    adapter->BlockUntilLoaded();

    try
    {
        if( LIB_SYMBOL* symbol = adapter->LoadSymbol( id ) )
            return symbol;
    }
    catch( const IO_ERROR& error )
    {
        aError = "library error: " + str( error.What() );
        return nullptr;
    }

    aError = "symbol not found in the libraries: " + aLibId + " (see sch_lib_symbol_search)";
    return nullptr;
}


struct SYMBOL_AT
{
    SCH_SYMBOL*    symbol = nullptr;
    SCH_SHEET_PATH path;
};


/// Symbol by "uuid" or "ref" (unique among sheet instances); aError/aStatus set when not found
std::optional<SYMBOL_AT> findSymbol( SCHEMATIC* aSchematic, const nlohmann::json& aArgs, int& aStatus, std::string& aError )
{
    const std::string uuid = aArgs.value( "uuid", std::string() );
    const std::string ref = aArgs.value( "ref", std::string() );
    std::vector<SYMBOL_AT> found;

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( ( !uuid.empty() && str( symbol->m_Uuid.AsString() ) == uuid )
                || ( uuid.empty() && !ref.empty() && str( symbol->GetRef( &path, false ) ) == ref ) )
            {
                found.push_back( { symbol, path } );
            }
        }
    }

    if( uuid.empty() && ref.empty() )
    {
        aStatus = 400;
        aError = "give 'uuid' or 'ref'";
        return std::nullopt;
    }

    if( found.empty() )
    {
        aStatus = 404;
        aError = "symbol not found";
        return std::nullopt;
    }

    // A reused sheet shows one symbol under several paths: same object, fine.  Different
    // objects with one reference (multi-unit parts, unannotated "U?") need a uuid.
    for( const SYMBOL_AT& f : found )
    {
        if( f.symbol != found.front().symbol )
        {
            aStatus = 409;
            aError = "reference '" + ref + "' is not unique (multi-unit or unannotated): use uuid";
            return std::nullopt;
        }
    }

    return found.front();
}


struct PIN_AT
{
    SCH_SYMBOL*    symbol = nullptr;
    SCH_PIN*       pin = nullptr;
    SCH_SHEET_PATH path;
};


/// Pin by REF.PIN (any unit of a multi-unit part)
std::optional<PIN_AT> findPin( SCHEMATIC* aSchematic, const std::string& aPin )
{
    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            for( SCH_PIN* pin : symbol->GetPins( &path ) )
            {
                if( pinId( pin, path ) == aPin )
                    return PIN_AT{ symbol, pin, path };
            }
        }
    }

    return std::nullopt;
}


/// Unit vector pointing out of the symbol body at a pin (from the pin root to its connection end)
VECTOR2I outward( const SCH_PIN* aPin )
{
    const VECTOR2I d = aPin->GetPosition() - aPin->GetPinRoot();

    if( std::abs( d.x ) >= std::abs( d.y ) )
        return VECTOR2I( d.x < 0 ? -1 : 1, 0 );

    return VECTOR2I( 0, d.y < 0 ? -1 : 1 );
}


SPIN_STYLE spinFor( const VECTOR2I& aOut )
{
    if( aOut.x < 0 )
        return SPIN_STYLE::LEFT;

    if( aOut.x > 0 )
        return SPIN_STYLE::RIGHT;

    return aOut.y < 0 ? SPIN_STYLE::UP : SPIN_STYLE::BOTTOM;
}


int orientationFlags( const nlohmann::json& aArgs, std::string& aError )
{
    int flags = SYM_ORIENT_0;

    switch( aArgs.value( "rotation", 0 ) )
    {
    case 0:   flags = SYM_ORIENT_0;   break;
    case 90:  flags = SYM_ORIENT_90;  break;
    case 180: flags = SYM_ORIENT_180; break;
    case 270: flags = SYM_ORIENT_270; break;
    default:  aError = "rotation must be 0, 90, 180 or 270"; return -1;
    }

    const std::string mirror = aArgs.value( "mirror", std::string( "none" ) );

    if( mirror == "x" )
        flags |= SYM_MIRROR_X;
    else if( mirror == "y" )
        flags |= SYM_MIRROR_Y;
    else if( mirror != "none" )
    {
        aError = "mirror must be none, x or y";
        return -1;
    }

    return flags;
}


/// Apply value / footprint / free fields / flags from args to a symbol instance
void applyFields( SCH_SYMBOL* aSymbol, const SCH_SHEET_PATH& aPath, const nlohmann::json& aArgs )
{
    if( aArgs.contains( "value" ) && aArgs["value"].is_string() )
        aSymbol->SetValueFieldText( wxString::FromUTF8( aArgs["value"].get<std::string>() ) );

    if( aArgs.contains( "footprint" ) && aArgs["footprint"].is_string() )
        aSymbol->SetFootprintFieldText( wxString::FromUTF8( aArgs["footprint"].get<std::string>() ) );

    if( aArgs.contains( "fields" ) && aArgs["fields"].is_object() )
    {
        for( const auto& [name, value] : aArgs["fields"].items() )
        {
            if( !value.is_string() )
                continue;

            const wxString fieldName = wxString::FromUTF8( name );
            SCH_FIELD*     field = aSymbol->GetField( fieldName );

            if( !field )
                field = aSymbol->AddField( SCH_FIELD( aSymbol, FIELD_T::USER, fieldName ) );

            field->SetText( wxString::FromUTF8( value.get<std::string>() ) );
        }
    }

    if( aArgs.contains( "dnp" ) && aArgs["dnp"].is_boolean() )
        aSymbol->SetDNP( aArgs["dnp"].get<bool>() );

    if( aArgs.contains( "in_bom" ) && aArgs["in_bom"].is_boolean() )
        aSymbol->SetExcludedFromBOM( !aArgs["in_bom"].get<bool>() );

    if( aArgs.contains( "on_board" ) && aArgs["on_board"].is_boolean() )
        aSymbol->SetExcludedFromBoard( !aArgs["on_board"].get<bool>() );

    if( aArgs.contains( "ref" ) && aArgs["ref"].is_string() && !aArgs["ref"].get<std::string>().empty() )
        aSymbol->SetRef( &aPath, wxString::FromUTF8( aArgs["ref"].get<std::string>() ) );
}


/**
 * Annotate like the editor's "Annotate Schematic" with "keep existing annotation" (or reset):
 * all sheets, by X position, first free number; power symbols get #PWR numbers.  Only symbols
 * whose reference changes are touched (each recorded in the commit).
 */
int annotate( SCHEMATIC* aSchematic, SCH_COMMIT& aCommit, bool aReset )
{
    SCH_SHEET_LIST               sheets = aSchematic->Hierarchy();
    SCH_REFERENCE_LIST           references;
    SCH_MULTI_UNIT_REFERENCE_MAP locked;

    sheets.GetSymbols( references, SYMBOL_FILTER_ALL );
    sheets.GetMultiUnitSymbols( locked, SYMBOL_FILTER_NON_POWER );

    std::map<std::pair<SCH_SYMBOL*, wxString>, wxString> before;

    for( size_t i = 0; i < references.GetCount(); ++i )
    {
        SCH_REFERENCE& ref = references[i];
        before[{ ref.GetSymbol(), ref.GetSheetPath().PathAsString() }] = ref.GetSymbol()->GetRef( &ref.GetSheetPath() );
    }

    if( aReset )
        references.RemoveAnnotation();

    references.SetRefDesTracker( aSchematic->Settings().m_refDesTracker );
    references.SplitReferences();
    references.AnnotateByOptions( SORT_BY_X_POSITION, INCREMENTAL_BY_REF, 0, locked, SCH_REFERENCE_LIST(), false );

    int changed = 0;

    for( size_t i = 0; i < references.GetCount(); ++i )
    {
        SCH_REFERENCE&  ref = references[i];
        SCH_SYMBOL*     symbol = ref.GetSymbol();
        SCH_SHEET_PATH& path = ref.GetSheetPath();

        aCommit.Modify( symbol, path.LastScreen() );
        ref.Annotate();

        if( symbol->GetRef( &path ) != before[{ symbol, path.PathAsString() }] )
            changed++;
    }

    return changed;
}


/**
 * First free reference number for a prefix ("C" -> "C4"), over all sheet instances plus the
 * references already handed out in this commit (aTaken).  New symbols get their reference
 * before the commit, so placing and numbering stay one undo step.
 */
wxString nextFreeRef( SCHEMATIC* aSchematic, const wxString& aPrefix, std::set<wxString>& aTaken )
{
    std::set<long> used;

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
            aTaken.insert( static_cast<SCH_SYMBOL*>( item )->GetRef( &path, false ) );
    }

    for( const wxString& ref : aTaken )
    {
        long number = 0;

        if( ref.StartsWith( aPrefix ) && ref.Mid( aPrefix.length() ).ToLong( &number ) )
            used.insert( number );
    }

    long n = 1;

    while( used.count( n ) )
        ++n;

    wxString ref = aPrefix + wxString::Format( wxS( "%ld" ), n );
    aTaken.insert( ref );
    return ref;
}


/**
 * Push and bring connectivity fully up to date.  The incremental update after a commit misses
 * labels placed directly on a pin end without a wire (seen: two labels "USB_DP" on two pins
 * stayed unconnected until the file was reloaded), so every edit ends with a full rebuild —
 * the same the netlist exporter does.
 */
/**
 * GUI: what an API edit changed glows for a moment, so a person watching sees where the agent
 * worked.  The item itself turns a colour the schematic does not use (vivid blue) and a halo
 * of the same colour fades around it.
 *
 * One collection per process: every glowing item with the time its glow started, in that
 * order, and one timer that redraws the fading halo and dims entries in the same order once
 * their time is up.  An item glowing again gets a newer entry; only its newest entry dims it.
 * Items are looked up again by uuid on every tick, so an undo or a closed editor in between is
 * harmless.  Main thread only (API methods run there).
 */
class GLOW_TRACKER : public wxEvtHandler
{
public:
    static constexpr int GLOW_MS = 2000;

    /// Process-wide; never destroyed (its timer is idle whenever the collection is empty)
    static GLOW_TRACKER& Get()
    {
        static GLOW_TRACKER* tracker = new GLOW_TRACKER();
        return *tracker;
    }

    void Add( KIWAY* aKiway, const std::vector<KIID>& aItems )
    {
        SCH_EDIT_FRAME* frame = frameOf( aKiway );

        if( !frame || aItems.empty() )
            return;

        m_kiway = aKiway;
        setItemColour( frame, true );

        const CLOCK::time_point now = CLOCK::now();
        SCH_SHEET_LIST          hierarchy = frame->Schematic().Hierarchy();

        for( const KIID& id : aItems )
        {
            if( SCH_ITEM* item = hierarchy.ResolveItem( id, nullptr, true ) )
            {
                item->SetBrightened();
                frame->UpdateItem( item );
                m_entries.push_back( { id, now } );
                m_newest[id] = now;
            }
        }

        drawHalo( frame, now );

        if( !m_entries.empty() && !m_timer.IsRunning() )
            m_timer.Start( TICK_MS );
    }

private:
    using CLOCK = std::chrono::steady_clock;

    static constexpr int TICK_MS = 50;

    /// Vivid blue: not among KiCad's default schematic colours (green wires, dark red bodies,
    /// yellow fills, teal text, dark blue buses, magenta highlight); KOPENAPI_GLOW_COLOR=#RRGGBB
    /// overrides it
    static KIGFX::COLOR4D glowColour( double aAlpha )
    {
        static const KIGFX::COLOR4D base = []()
        {
            wxString hex;

            if( wxGetEnv( wxS( "KOPENAPI_GLOW_COLOR" ), &hex ) && hex.length() == 7 && hex[0] == '#' )
            {
                unsigned long rgb = 0;

                if( hex.Mid( 1 ).ToULong( &rgb, 16 ) )
                    return KIGFX::COLOR4D( ( ( rgb >> 16 ) & 0xFF ) / 255.0, ( ( rgb >> 8 ) & 0xFF ) / 255.0, ( rgb & 0xFF ) / 255.0, 1.0 );
            }

            return KIGFX::COLOR4D( 0.10, 0.45, 1.0, 1.0 );
        }();

        return base.WithAlpha( aAlpha );
    }

    struct ENTRY
    {
        KIID              id;
        CLOCK::time_point start;
    };

    GLOW_TRACKER()
    {
        m_timer.SetOwner( this );
        Bind( wxEVT_TIMER, &GLOW_TRACKER::onTick, this );
    }

    static SCH_EDIT_FRAME* frameOf( KIWAY* aKiway )
    {
        return aKiway ? static_cast<SCH_EDIT_FRAME*>( aKiway->Player( FRAME_SCH, false ) ) : nullptr;
    }

    /// Brightened items draw in the glow colour while anything glows; KiCad's own colour after
    void setItemColour( SCH_EDIT_FRAME* aFrame, bool aGlow )
    {
        KIGFX::RENDER_SETTINGS* settings = aFrame->GetCanvas()->GetView()->GetPainter()->GetSettings();

        if( aGlow && !m_savedColour )
        {
            m_savedColour = settings->GetLayerColor( LAYER_BRIGHTENED );
            settings->SetLayerColor( LAYER_BRIGHTENED, glowColour( 1.0 ) );
        }
        else if( !aGlow && m_savedColour )
        {
            settings->SetLayerColor( LAYER_BRIGHTENED, *m_savedColour );
            m_savedColour.reset();
        }
    }

    /// Halo: nested rounded outlines around each glowing item, fading outwards and with age
    void drawHalo( SCH_EDIT_FRAME* aFrame, CLOCK::time_point aNow )
    {
        KIGFX::VIEW* view = aFrame->GetCanvas()->GetView();

        if( !m_overlay || m_overlayView != view )
        {
            m_overlay = view->MakeOverlay();
            m_overlayView = view;
        }

        m_overlay->Clear();

        SCH_SHEET_LIST hierarchy = aFrame->Schematic().Hierarchy();
        const int      mm = schIUScale.mmToIU( 1.0 );

        for( const auto& [id, start] : m_newest )
        {
            SCH_ITEM* item = hierarchy.ResolveItem( id, nullptr, true );

            if( !item )
                continue;

            const double age = std::chrono::duration<double, std::milli>( aNow - start ).count() / GLOW_MS;
            const double strength = std::clamp( 1.0 - age, 0.0, 1.0 );

            if( strength <= 0 )
                continue;

            BOX2I box = item->GetBoundingBox();

            m_overlay->SetIsFill( true );
            m_overlay->SetIsStroke( false );
            m_overlay->SetFillColor( glowColour( 0.10 * strength ) );
            m_overlay->Rectangle( VECTOR2D( box.GetLeft() - mm, box.GetTop() - mm ),
                                  VECTOR2D( box.GetRight() + mm, box.GetBottom() + mm ) );

            m_overlay->SetIsFill( false );
            m_overlay->SetIsStroke( true );

            for( int ring = 0; ring < 5; ++ring )
            {
                const int margin = mm + ring * mm * 6 / 10;
                m_overlay->SetLineWidth( mm * 0.6 );
                m_overlay->SetStrokeColor( glowColour( ( 0.85 - ring * 0.17 ) * strength ) );
                m_overlay->Rectangle( VECTOR2D( box.GetLeft() - margin, box.GetTop() - margin ),
                                      VECTOR2D( box.GetRight() + margin, box.GetBottom() + margin ) );
            }
        }

        view->Update( m_overlay.get() );
        aFrame->GetCanvas()->Refresh();
    }

    void onTick( wxTimerEvent& )
    {
        const CLOCK::time_point       now = CLOCK::now();
        SCH_EDIT_FRAME*               frame = frameOf( m_kiway );
        std::optional<SCH_SHEET_LIST> hierarchy;

        // Dim in the order the glows started
        while( !m_entries.empty() && now - m_entries.front().start >= std::chrono::milliseconds( GLOW_MS ) )
        {
            const ENTRY entry = m_entries.front();
            m_entries.pop_front();

            auto newest = m_newest.find( entry.id );

            if( newest == m_newest.end() || newest->second != entry.start )
                continue;   // glowed again later: its newer entry dims it

            m_newest.erase( newest );

            if( !frame )
                continue;

            if( !hierarchy )
                hierarchy = frame->Schematic().Hierarchy();

            if( SCH_ITEM* item = hierarchy->ResolveItem( entry.id, nullptr, true ) )
            {
                item->ClearBrightened();
                frame->UpdateItem( item );
            }
        }

        if( frame )
        {
            drawHalo( frame, now );

            if( m_entries.empty() )
                setItemColour( frame, false );
        }

        if( m_entries.empty() )
        {
            m_timer.Stop();

            if( m_overlay && frame )
            {
                m_overlay->Clear();
                m_overlayView->Update( m_overlay.get() );
                frame->GetCanvas()->Refresh();
            }

            m_overlay.reset();
            m_overlayView = nullptr;
        }
    }

    KIWAY*                                  m_kiway = nullptr;
    wxTimer                                 m_timer;
    std::deque<ENTRY>                       m_entries;
    std::map<KIID, CLOCK::time_point>       m_newest;
    std::shared_ptr<KIGFX::VIEW_OVERLAY>    m_overlay;
    KIGFX::VIEW*                            m_overlayView = nullptr;
    std::optional<KIGFX::COLOR4D>           m_savedColour;
};


void glow( KIWAY* aKiway, const std::vector<KIID>& aItems )
{
    GLOW_TRACKER::Get().Add( aKiway, aItems );
}


void pushEdit( SCH_COMMIT& aCommit, SCHEMATIC* aSchematic, const wxString& aMessage, KIWAY* aKiway = nullptr,
               const std::vector<KIID>& aGlow = {} )
{
    aCommit.Push( aMessage );
    aSchematic->RebuildConnectivity();
    glow( aKiway, aGlow );
}


nlohmann::json symbolCard( SCH_SYMBOL* aSymbol, const SCH_SHEET_PATH& aPath )
{
    nlohmann::json pins = nlohmann::json::array();

    for( SCH_PIN* pin : aSymbol->GetPins( &aPath ) )
    {
        pins.push_back( { { "pin", pinId( pin, aPath ) },
                          { "name", str( pin->GetShownName() ) },
                          { "type", str( pin->GetElectricalTypeName() ) },
                          { "x_mm", toMm( pin->GetPosition().x ) },
                          { "y_mm", toMm( pin->GetPosition().y ) } } );
    }

    nlohmann::json fields = nlohmann::json::object();

    for( SCH_FIELD* field : { aSymbol->GetField( FIELD_T::REFERENCE ), aSymbol->GetField( FIELD_T::VALUE ) } )
    {
        if( field )
        {
            fields[str( field->GetName() )] = { { "x_mm", toMm( field->GetPosition().x ) },
                                                { "y_mm", toMm( field->GetPosition().y ) },
                                                { "visible", field->IsVisible() } };
        }
    }

    return { { "uuid", str( aSymbol->m_Uuid.AsString() ) },
             { "ref", str( aSymbol->GetRef( &aPath, false ) ) },
             { "fields_at", fields },
             { "rotation", [&]()
               {
                   switch( aSymbol->GetOrientationProp() )
                   {
                   case SYMBOL_ORIENTATION_PROP::SYMBOL_ANGLE_90:  return 90;
                   case SYMBOL_ORIENTATION_PROP::SYMBOL_ANGLE_180: return 180;
                   case SYMBOL_ORIENTATION_PROP::SYMBOL_ANGLE_270: return 270;
                   default:                                        return 0;
                   }
               }() },
             { "mirror", aSymbol->GetMirrorX() ? "x" : aSymbol->GetMirrorY() ? "y" : "none" },
             { "value", str( aSymbol->GetValue( &aPath, FOR_GUI ) ) },
             { "lib_id", str( aSymbol->GetLibId().Format() ) },
             { "footprint", str( aSymbol->GetFootprintFieldText( &aPath, FOR_GUI ) ) },
             { "unit", aSymbol->GetUnit() },
             { "sheet", sheetPath( aPath ) },
             { "x_mm", toMm( aSymbol->GetPosition().x ) },
             { "y_mm", toMm( aSymbol->GetPosition().y ) },
             { "pins", pins } };
}


/// The net a pin is on now (after a commit), for confirming connections
std::string netOfPin( SCHEMATIC* aSchematic, const std::string& aPin )
{
    for( const NET_ENTRY& net : collectNets( aSchematic ) )
    {
        for( const auto& [pin, path] : netPins( net ) )
        {
            if( pinId( pin, path ) == aPin )
                return net.name;
        }
    }

    return std::string();
}



/// A wire end: REF.PIN, or a point {x_mm, y_mm} on the target sheet
std::optional<VECTOR2I> wireEnd( SCHEMATIC* aSchematic, const nlohmann::json& aEnd, std::optional<SCH_SHEET_PATH>& aSheet,
                                 VECTOR2I& aOutward, std::string& aError )
{
    aOutward = VECTOR2I( 0, 0 );

    if( aEnd.is_string() )
    {
        std::optional<PIN_AT> at = findPin( aSchematic, aEnd.get<std::string>() );

        if( !at )
        {
            aError = "pin not found: " + aEnd.get<std::string>();
            return std::nullopt;
        }

        if( aSheet && *aSheet != at->path )
        {
            aError = "both ends must be on one sheet";
            return std::nullopt;
        }

        aSheet = at->path;
        aOutward = outward( at->pin );
        return at->pin->GetPosition();
    }

    if( aEnd.is_object() && aEnd.contains( "x_mm" ) && aEnd.contains( "y_mm" ) )
        return VECTOR2I( toIU( aEnd["x_mm"].get<double>() ), toIU( aEnd["y_mm"].get<double>() ) );

    aError = "a wire end is REF.PIN or {x_mm, y_mm}";
    return std::nullopt;
}

} // namespace


static KOPENAPI_RESULT h_sch_wire( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "from" ) || !aArgs.contains( "to" ) )
        return KOPENAPI_RESULT::Error( 400, "give 'from' and 'to' (REF.PIN or {x_mm, y_mm})" );

    SCHEMATIC*                    schematic = context->GetSchematic();
    std::optional<SCH_SHEET_PATH> sheet;
    std::string                   error;
    VECTOR2I                      outFrom, outTo;

    std::optional<VECTOR2I> a = wireEnd( schematic, aArgs["from"], sheet, outFrom, error );
    std::optional<VECTOR2I> b = a ? wireEnd( schematic, aArgs["to"], sheet, outTo, error ) : std::nullopt;

    if( !a || !b )
        return KOPENAPI_RESULT::Error( error.rfind( "pin not found", 0 ) == 0 ? 404 : 400, error );

    if( !sheet )
        sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    if( *a == *b )
        return KOPENAPI_RESULT::Error( 400, "the two ends are the same point" );

    // Route: straight, L, else Z, never along another wire, through another connection point or
    // a symbol body.  "auto" leaves the start pin along its own direction first.
    std::string route = aArgs.value( "route", std::string( "auto" ) );

    if( route == "auto" )
        route = outFrom.y != 0 ? "vh" : outFrom.x != 0 ? "hv" : outTo.y != 0 ? "hv" : "vh";

    if( route != "hv" && route != "vh" )
        return KOPENAPI_RESULT::Error( 400, "route must be auto, hv (horizontal first) or vh" );

    WIRE_ROUTER router( sheet->LastScreen(), *sheet, toIU( GRID_MM ) );
    std::string why;
    std::optional<std::vector<VECTOR2I>> path = router.Route( *a, *b, route, &why );

    if( !path )
        return KOPENAPI_RESULT::Error( 422, "no clear route between the two ends: the direct / one-corner wire "
                                            + ( why.empty() ? std::string( "is blocked" ) : why )
                                            + ", and no detour within 40 grid steps is clear. Move a part, "
                                              "or connect the pins with sch_connect labels" );

    const std::vector<VECTOR2I>& points = *path;

    SCH_COMMIT        commit( context->GetToolManager() );
    SCH_SCREEN*       screen = sheet->LastScreen();
    std::vector<KIID> added;
    nlohmann::json    segments = nlohmann::json::array();

    for( size_t i = 0; i + 1 < points.size(); ++i )
    {
        auto* wire = new SCH_LINE( points[i], LAYER_WIRE );
        wire->SetEndPoint( points[i + 1] );
        commit.Add( wire, screen );
        added.push_back( wire->m_Uuid );
        segments.push_back( { { "uuid", str( wire->m_Uuid.AsString() ) },
                              { "from", { toMm( points[i].x ), toMm( points[i].y ) } },
                              { "to", { toMm( points[i + 1].x ), toMm( points[i + 1].y ) } } } );
    }

    commit.Push( _( "Wire (API)" ), SKIP_CONNECTIVITY );

    // Junctions where the new wire ends meet a wire in its middle or three ends meet
    SCH_COMMIT     junctions( context->GetToolManager() );
    nlohmann::json dots = nlohmann::json::array();

    for( const VECTOR2I& point : { *a, *b } )
    {
        if( screen->IsExplicitJunctionNeeded( point ) )
        {
            auto* junction = new SCH_JUNCTION( point );
            junctions.Add( junction, screen );
            added.push_back( junction->m_Uuid );
            dots.push_back( { toMm( point.x ), toMm( point.y ) } );
        }
    }

    pushEdit( junctions, schematic, _( "Wire (API)" ), aCtx.kiway, added );

    nlohmann::json result = { { "segments", segments }, { "junctions", dots } };

    if( aArgs["from"].is_string() )
        result["net"] = netOfPin( schematic, aArgs["from"] );

    return KOPENAPI_RESULT::Ok( result );
}


namespace
{
} // namespace


static KOPENAPI_RESULT h_sch_new( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    const std::string path = aArgs.value( "path", std::string() );
    wxFileName        fn( wxString::FromUTF8( path ) );

    if( path.empty() || fn.GetExt() != wxS( "kicad_sch" ) || !fn.IsAbsolute() )
        return KOPENAPI_RESULT::Error( 400, "give an absolute 'path' ending in .kicad_sch" );

    if( fn.FileExists() && !aArgs.value( "overwrite", false ) )
        return KOPENAPI_RESULT::Error( 409, "file exists (overwrite: true to replace it)" );

    if( !fn.DirExists() && !wxFileName::Mkdir( fn.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
        return KOPENAPI_RESULT::Error( 500, "cannot create the directory" );

    // An empty schematic in the current file format, so the loader does not normalise it
    const std::string text = fmt::format( "(kicad_sch\n\t(version {})\n\t(generator \"kicadopenapi\")\n"
                                          "\t(uuid \"{}\")\n\t(paper \"{}\")\n\t(lib_symbols)\n"
                                          "\t(sheet_instances\n\t\t(path \"/\"\n\t\t\t(page \"1\")\n\t\t)\n\t)\n)\n",
                                          SEXPR_SCHEMATIC_FILE_VERSION, str( KIID().AsString() ),
                                          aArgs.value( "paper", std::string( "A4" ) ) );

    wxFFile file( fn.GetFullPath(), wxS( "w" ) );

    if( !file.IsOpened() || !file.Write( wxString::FromUTF8( text ) ) )
        return KOPENAPI_RESULT::Error( 500, "cannot write " + path );

    file.Close();

    std::optional<KOPENAPI_METHOD> open = KOPENAPI_REGISTRY::Get().Find( "sch_open" );

    if( !open )
        return KOPENAPI_RESULT::Error( 503, "sch_open is not available" );

    return open->handler( aCtx, { { "path", path }, { "discard", aArgs.value( "discard", false ) } } );
}


static KOPENAPI_RESULT h_sch_symbol_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "x_mm" ) || !aArgs.contains( "y_mm" ) )
        return KOPENAPI_RESULT::Error( 400, "give 'lib_id', 'x_mm' and 'y_mm'" );

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    std::string error;
    const int   orientation = orientationFlags( aArgs, error );

    if( orientation < 0 )
        return KOPENAPI_RESULT::Error( 400, error );

    LIB_SYMBOL* libSymbol = librarySymbol( aCtx, *context, aArgs.value( "lib_id", std::string() ), error );

    if( !libSymbol )
        return KOPENAPI_RESULT::Error( error.rfind( "lib_id", 0 ) == 0 ? 400 : 404, error );

    const int unit = aArgs.value( "unit", 1 );

    if( unit < 1 || unit > libSymbol->GetUnitCount() )
        return KOPENAPI_RESULT::Error( 400, fmt::format( "unit must be 1..{}", libSymbol->GetUnitCount() ) );

    SCHEMATIC* schematic = context->GetSchematic();
    LIB_ID     libId;
    libId.Parse( wxString::FromUTF8( aArgs.value( "lib_id", std::string() ) ) );

    std::unique_ptr<LIB_SYMBOL> flat = libSymbol->Flatten();
    auto*      symbol = new SCH_SYMBOL( *flat, libId, &*sheet, unit, 1,
                                        VECTOR2I( toIU( aArgs["x_mm"].get<double>() ), toIU( aArgs["y_mm"].get<double>() ) ),
                                        schematic );
    symbol->SetOrientation( orientation );
    applyFields( symbol, *sheet, aArgs );

    if( aArgs.value( "annotate", true ) && !aArgs.contains( "ref" ) )
    {
        // "U?" -> first free "U<n>"
        wxString           prefix = symbol->GetRef( &*sheet, false );
        std::set<wxString> taken;

        while( prefix.EndsWith( wxS( "?" ) ) )
            prefix.RemoveLast();

        symbol->SetRef( &*sheet, nextFreeRef( schematic, prefix, taken ) );
    }

    symbol->AutoplaceFields( nullptr, AUTOPLACE_AUTO );

    SCH_COMMIT commit( context->GetToolManager() );
    commit.Add( symbol, sheet->LastScreen() );
    pushEdit( commit, schematic, _( "Add symbol (API)" ), aCtx.kiway, { symbol->m_Uuid } );
    return KOPENAPI_RESULT::Ok( symbolCard( symbol, *sheet ) );
}


static KOPENAPI_RESULT h_sch_symbol_update( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    int         status = 0;
    std::string error;
    std::optional<SYMBOL_AT> at = findSymbol( context->GetSchematic(), aArgs, status, error );

    if( !at )
        return KOPENAPI_RESULT::Error( status, error );

    int orientation = -1;

    if( aArgs.contains( "rotation" ) || aArgs.contains( "mirror" ) )
    {
        orientation = orientationFlags( aArgs, error );

        if( orientation < 0 )
            return KOPENAPI_RESULT::Error( 400, error );
    }

    SCH_COMMIT  commit( context->GetToolManager() );
    SCH_SCREEN* screen = at->path.LastScreen();
    commit.Modify( at->symbol, screen );

    // Pin positions before the edit: what is attached there follows the pins (drag)
    std::map<wxString, VECTOR2I> before;

    for( SCH_PIN* pin : at->symbol->GetPins( &at->path ) )
        before[pin->GetNumber()] = pin->GetPosition();

    // "ref" here is the lookup key unless "new_ref" renames
    nlohmann::json changes = aArgs;
    changes.erase( "ref" );

    if( aArgs.contains( "new_ref" ) )
        changes["ref"] = aArgs["new_ref"];

    applyFields( at->symbol, at->path, changes );

    if( orientation >= 0 )
        at->symbol->SetOrientation( orientation );

    if( aArgs.contains( "x_mm" ) || aArgs.contains( "y_mm" ) )
    {
        const VECTOR2I pos = at->symbol->GetPosition();
        at->symbol->SetPosition( VECTOR2I( aArgs.contains( "x_mm" ) ? toIU( aArgs["x_mm"].get<double>() ) : pos.x,
                                           aArgs.contains( "y_mm" ) ? toIU( aArgs["y_mm"].get<double>() ) : pos.y ) );
    }

    std::vector<KIID> moved = { at->symbol->m_Uuid };

    if( orientation >= 0 || aArgs.contains( "x_mm" ) || aArgs.contains( "y_mm" ) )
    {
        at->symbol->AutoplaceFields( screen, AUTOPLACE_AUTO );

        // Drag: labels, no-connects, power symbols and wire ends sitting on a pin go with it
        std::set<SCH_ITEM*> done;

        for( SCH_PIN* pin : at->symbol->GetPins( &at->path ) )
        {
            const VECTOR2I from = before[pin->GetNumber()];
            const VECTOR2I to = pin->GetPosition();

            if( from == to )
                continue;

            std::vector<SCH_ITEM*> here;

            for( SCH_ITEM* item : screen->Items().Overlapping( from ) )
                here.push_back( item );

            for( SCH_ITEM* item : here )
            {
                if( item == at->symbol || done.count( item ) )
                    continue;

                if( labelKind( item->Type() ) || item->Type() == SCH_NO_CONNECT_T )
                {
                    if( item->GetPosition() != from )
                        continue;

                    commit.Modify( item, screen );
                    item->SetPosition( to );
                }
                else if( item->Type() == SCH_SYMBOL_T && static_cast<SCH_SYMBOL*>( item )->IsPower() )
                {
                    SCH_SYMBOL* power = static_cast<SCH_SYMBOL*>( item );
                    std::vector<SCH_PIN*> own = power->GetPins( &at->path );

                    if( own.empty() || own.front()->GetPosition() != from )
                        continue;

                    commit.Modify( item, screen );
                    power->SetPosition( power->GetPosition() + ( to - from ) );
                }
                else if( item->Type() == SCH_LINE_T && static_cast<SCH_LINE*>( item )->IsWire() )
                {
                    SCH_LINE* wire = static_cast<SCH_LINE*>( item );

                    if( wire->GetStartPoint() != from && wire->GetEndPoint() != from )
                        continue;

                    commit.Modify( item, screen );

                    if( wire->GetStartPoint() == from )
                        wire->SetStartPoint( to );
                    else
                        wire->SetEndPoint( to );
                }
                else
                {
                    continue;
                }

                done.insert( item );
                moved.push_back( item->m_Uuid );
            }
        }
    }

    if( aArgs.value( "autoplace_fields", false ) )
        at->symbol->AutoplaceFields( screen, AUTOPLACE_AUTO );

    // Field text placement: {"Reference": {"x_mm":..,"y_mm":..}, "Value": "hide"}
    if( aArgs.contains( "field_positions" ) && aArgs["field_positions"].is_object() )
    {
        for( const auto& [name, place] : aArgs["field_positions"].items() )
        {
            SCH_FIELD* field = at->symbol->GetField( wxString::FromUTF8( name ) );

            if( !field )
                return KOPENAPI_RESULT::Error( 400, "no field '" + name + "' on " + str( at->symbol->GetRef( &at->path, false ) ) );

            if( place.is_string() && ( place == "hide" || place == "show" ) )
                field->SetVisible( place == "show" );
            else if( place.is_object() && place.contains( "x_mm" ) && place.contains( "y_mm" ) )
                field->SetPosition( VECTOR2I( toIU( place["x_mm"].get<double>() ), toIU( place["y_mm"].get<double>() ) ) );
            else
                return KOPENAPI_RESULT::Error( 400, "field_positions values: {x_mm, y_mm}, \"hide\" or \"show\"" );
        }
    }

    // Wires dragged out of square: re-route them from their fixed end to the pin's new place
    nlohmann::json warnings = nlohmann::json::array();
    std::vector<std::pair<SCH_LINE*, std::pair<VECTOR2I, VECTOR2I>>> diagonal;

    for( SCH_ITEM* item : screen->Items().OfType( SCH_LINE_T ) )
    {
        SCH_LINE* wire = static_cast<SCH_LINE*>( item );

        if( wire->IsWire() && std::find( moved.begin(), moved.end(), wire->m_Uuid ) != moved.end()
            && wire->GetStartPoint().x != wire->GetEndPoint().x && wire->GetStartPoint().y != wire->GetEndPoint().y )
        {
            diagonal.push_back( { wire, { wire->GetStartPoint(), wire->GetEndPoint() } } );
        }
    }

    if( !diagonal.empty() )
    {
        std::set<const SCH_ITEM*> ignore;

        for( const auto& [wire, ends] : diagonal )
            ignore.insert( wire );

        WIRE_ROUTER router( screen, at->path, toIU( GRID_MM ), ignore );

        for( const auto& [wire, ends] : diagonal )
        {
            std::optional<std::vector<VECTOR2I>> path = router.Route( ends.first, ends.second, "hv" );

            if( !path )
            {
                warnings.push_back( "a wire stays diagonal: no clear orthogonal route from "
                                    + std::to_string( toMm( ends.first.x ) ) + ", " + std::to_string( toMm( ends.first.y ) ) );
                continue;
            }

            commit.Remove( wire, screen );

            for( size_t i = 0; i + 1 < path->size(); ++i )
            {
                auto* segment = new SCH_LINE( ( *path )[i], LAYER_WIRE );
                segment->SetEndPoint( ( *path )[i + 1] );
                commit.Add( segment, screen );
                router.AddWire( ( *path )[i], ( *path )[i + 1] );
                moved.push_back( segment->m_Uuid );
            }
        }
    }

    pushEdit( commit, context->GetSchematic(), _( "Edit symbol (API)" ), aCtx.kiway, moved );

    if( !warnings.empty() )
    {
        nlohmann::json card = symbolCard( at->symbol, at->path );
        card["warnings"] = warnings;
        return KOPENAPI_RESULT::Ok( card );
    }
    return KOPENAPI_RESULT::Ok( symbolCard( at->symbol, at->path ) );
}


static KOPENAPI_RESULT h_sch_item_delete( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "uuids" ) || !aArgs["uuids"].is_array() || aArgs["uuids"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'uuids' (array)" );

    std::set<std::string> wanted;

    for( const nlohmann::json& u : aArgs["uuids"] )
    {
        if( u.is_string() )
            wanted.insert( u.get<std::string>() );
    }

    SCH_COMMIT     commit( context->GetToolManager() );
    nlohmann::json deleted = nlohmann::json::array();
    SCH_SCREENS    screens( context->GetSchematic()->Root() );

    for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
    {
        std::vector<SCH_ITEM*> hits;

        for( SCH_ITEM* item : screen->Items() )
        {
            if( wanted.count( str( item->m_Uuid.AsString() ) ) )
                hits.push_back( item );
        }

        for( SCH_ITEM* item : hits )
        {
            deleted.push_back( str( item->m_Uuid.AsString() ) );
            wanted.erase( str( item->m_Uuid.AsString() ) );
            commit.Remove( item, screen );
        }
    }

    if( deleted.empty() )
        return KOPENAPI_RESULT::Error( 404, "none of the uuids found" );

    pushEdit( commit, context->GetSchematic(), _( "Delete (API)" ) );

    nlohmann::json missing = nlohmann::json::array();

    for( const std::string& u : wanted )
        missing.push_back( u );

    return KOPENAPI_RESULT::Ok( { { "deleted", deleted }, { "not_found", missing } } );
}


static KOPENAPI_RESULT h_sch_connect( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string net = aArgs.value( "net", std::string() );
    const std::string kind = aArgs.value( "kind", std::string( "local" ) );

    if( net.empty() || !aArgs.contains( "pins" ) || !aArgs["pins"].is_array() || aArgs["pins"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'net' and 'pins' (array of REF.PIN)" );

    if( kind != "local" && kind != "global" && kind != "power" )
        return KOPENAPI_RESULT::Error( 400, "kind must be local, global or power" );

    SCHEMATIC*  schematic = context->GetSchematic();
    LIB_SYMBOL* powerSymbol = nullptr;
    std::string error;

    if( kind == "power" )
    {
        powerSymbol = librarySymbol( aCtx, *context, aArgs.value( "power_lib_id", "power:" + net ), error );

        if( !powerSymbol || !powerSymbol->IsPower() )
            return KOPENAPI_RESULT::Error( 404, powerSymbol ? "not a power symbol: " + aArgs.value( "power_lib_id", "power:" + net )
                                                            : error + " (power_lib_id names the power symbol)" );
    }

    // Resolve every pin first: all or nothing
    std::vector<PIN_AT> pins;

    for( const nlohmann::json& p : aArgs["pins"] )
    {
        std::optional<PIN_AT> at = p.is_string() ? findPin( schematic, p.get<std::string>() ) : std::nullopt;

        if( !at )
            return KOPENAPI_RESULT::Error( 404, "pin not found: " + ( p.is_string() ? p.get<std::string>() : p.dump() ) );

        pins.push_back( *at );
    }

    SCH_COMMIT         commit( context->GetToolManager() );
    nlohmann::json     placed = nlohmann::json::array();
    std::set<wxString> takenRefs;
    std::vector<KIID>  extra;    // stubs, joining wires, junctions (glow too)

    // A power symbol at aPoint with its body away from the part: the direction from its pin to
    // the centre of its body must be aOut.  (Power pins have zero length, so the pin itself
    // has no direction to go by.)
    auto placePower = [&]( const VECTOR2I& aPoint, const VECTOR2I& aOut, const SCH_SHEET_PATH& aPath ) -> SCH_ITEM*
    {
        std::unique_ptr<LIB_SYMBOL> flat = powerSymbol->Flatten();
        auto* symbol = new SCH_SYMBOL( *flat, powerSymbol->GetLibId(), &aPath, 1, 1, aPoint, schematic );

        auto bodyDirection = [&]() -> VECTOR2I
        {
            std::vector<SCH_PIN*> own = symbol->GetPins( &aPath );
            const VECTOR2I        pin = own.empty() ? symbol->GetPosition() : own.front()->GetPosition();
            const VECTOR2I        d = symbol->GetBodyBoundingBox().Centre() - pin;

            if( std::abs( d.x ) >= std::abs( d.y ) )
                return VECTOR2I( d.x < 0 ? -1 : 1, 0 );

            return VECTOR2I( 0, d.y < 0 ? -1 : 1 );
        };

        for( int orientation : { SYM_ORIENT_0, SYM_ORIENT_90, SYM_ORIENT_180, SYM_ORIENT_270 } )
        {
            symbol->SetOrientation( orientation );

            if( bodyDirection() == aOut )
                break;
        }

        if( std::vector<SCH_PIN*> own = symbol->GetPins( &aPath ); !own.empty() )
            symbol->SetPosition( symbol->GetPosition() + ( aPoint - own.front()->GetPosition() ) );

        symbol->SetRef( &aPath, nextFreeRef( schematic, wxS( "#PWR" ), takenRefs ) );
        symbol->AutoplaceFields( nullptr, AUTOPLACE_AUTO );
        return symbol;
    };

    auto addWire = [&]( const VECTOR2I& aStart, const VECTOR2I& aEnd, SCH_SCREEN* aScreen )
    {
        auto* wire = new SCH_LINE( aStart, LAYER_WIRE );
        wire->SetEndPoint( aEnd );
        commit.Add( wire, aScreen );
        extra.push_back( wire->m_Uuid );
    };

    // Power: pins of one part facing one way (VCC + V3 on top, GND + shield below) share one
    // symbol: a short stub from each pin, the stubs joined, the symbol at the end of the row —
    // adjacent power symbols would overlap.  Other kinds: a label on each pin.
    std::map<std::tuple<SCH_SYMBOL*, wxString, int, int>, std::vector<const PIN_AT*>> groups;
    std::vector<std::vector<const PIN_AT*>>                                          order;

    for( const PIN_AT& at : pins )
    {
        const VECTOR2I out = outward( at.pin );
        auto key = std::make_tuple( kind == "power" ? at.symbol : nullptr, at.path.PathAsString(),
                                    kind == "power" ? out.x : 0, kind == "power" ? out.y : &at - &pins[0] );
        groups[key].push_back( &at );
    }

    for( auto& [key, group] : groups )
    {
        const PIN_AT&  first = *group.front();
        const VECTOR2I out = outward( first.pin );
        SCH_SCREEN*    screen = first.path.LastScreen();

        if( kind != "power" )
        {
            for( const PIN_AT* at : group )
            {
                const VECTOR2I  point = at->pin->GetPosition();
                SCH_LABEL_BASE* label = kind == "global" ? static_cast<SCH_LABEL_BASE*>( new SCH_GLOBALLABEL( point, wxString::FromUTF8( net ) ) )
                                                         : static_cast<SCH_LABEL_BASE*>( new SCH_LABEL( point, wxString::FromUTF8( net ) ) );
                label->SetSpinStyle( spinFor( outward( at->pin ) ) );
                commit.Add( label, screen );
                placed.push_back( { { "pin", pinId( at->pin, at->path ) }, { "uuid", str( label->m_Uuid.AsString() ) } } );
            }

            continue;
        }

        if( group.size() == 1 )
        {
            SCH_ITEM* symbol = placePower( first.pin->GetPosition(), out, first.path );
            commit.Add( symbol, screen );
            placed.push_back( { { "pin", pinId( first.pin, first.path ) }, { "uuid", str( symbol->m_Uuid.AsString() ) } } );
            continue;
        }

        // Row of stubs along the side, ordered across it
        std::vector<const PIN_AT*> row = group;
        const bool                 vertical = out.x == 0;   // pins on top/bottom: row runs along x

        std::sort( row.begin(), row.end(),
                   [vertical]( const PIN_AT* a, const PIN_AT* b )
                   {
                       return vertical ? a->pin->GetPosition().x < b->pin->GetPosition().x
                                       : a->pin->GetPosition().y < b->pin->GetPosition().y;
                   } );

        // Stubs end on one line: as far out as the farthest pin end plus two grid steps
        const int stub = toIU( 2 * GRID_MM );
        int       line = 0;

        for( size_t k = 0; k < row.size(); ++k )
        {
            const VECTOR2I p = row[k]->pin->GetPosition();
            const int      along = vertical ? p.y * out.y : p.x * out.x;
            line = k == 0 ? along : std::max( line, along );
        }

        line += stub;
        std::vector<VECTOR2I> ends;

        for( const PIN_AT* at : row )
        {
            const VECTOR2I p = at->pin->GetPosition();
            const VECTOR2I end = vertical ? VECTOR2I( p.x, line * out.y ) : VECTOR2I( line * out.x, p.y );
            addWire( p, end, screen );
            ends.push_back( end );
            placed.push_back( { { "pin", pinId( at->pin, at->path ) }, { "uuid", str( extra.back().AsString() ) } } );
        }

        for( size_t k = 0; k + 1 < ends.size(); ++k )
            addWire( ends[k], ends[k + 1], screen );

        // Three wire ends meet at the inner stub ends
        for( size_t k = 1; k + 1 < ends.size(); ++k )
        {
            auto* junction = new SCH_JUNCTION( ends[k] );
            commit.Add( junction, screen );
            extra.push_back( junction->m_Uuid );
        }

        SCH_ITEM* symbol = placePower( ends.front(), out, first.path );
        commit.Add( symbol, screen );
        extra.push_back( symbol->m_Uuid );
    }

    std::vector<KIID> added;

    for( const nlohmann::json& p : placed )
        added.emplace_back( wxString::FromUTF8( p["uuid"].get<std::string>() ) );

    added.insert( added.end(), extra.begin(), extra.end() );
    pushEdit( commit, schematic, _( "Connect pins (API)" ), aCtx.kiway, added );

    // Confirm: every pin is now on one net
    std::set<std::string> nets;

    for( const nlohmann::json& p : placed )
        nets.insert( netOfPin( schematic, p["pin"] ) );

    return KOPENAPI_RESULT::Ok( { { "net", nets.size() == 1 ? nlohmann::json( *nets.begin() ) : nlohmann::json() },
                                  { "connected", nets.size() == 1 && !nets.begin()->empty() },
                                  { "nets_seen", nets },
                                  { "placed", placed } } );
}


static KOPENAPI_RESULT h_sch_item_list( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    std::set<std::string> types;

    if( aArgs.contains( "types" ) && aArgs["types"].is_array() )
    {
        for( const nlohmann::json& t : aArgs["types"] )
        {
            if( t.is_string() )
                types.insert( t.get<std::string>() );
        }
    }

    const std::string       netGlob = aArgs.value( "net", std::string() );
    std::optional<VECTOR2I> near;
    const int               radius = toIU( aArgs.value( "radius_mm", 5.08 ) );

    if( aArgs.contains( "near" ) && aArgs["near"].is_object() )
        near = VECTOR2I( toIU( aArgs["near"].value( "x_mm", 0.0 ) ), toIU( aArgs["near"].value( "y_mm", 0.0 ) ) );

    // Net of every connectable item on this sheet instance
    std::map<const SCH_ITEM*, std::string> netOf;

    for( const NET_ENTRY& net : collectNets( context->GetSchematic() ) )
    {
        for( const NET_INSTANCE& inst : net.instances )
        {
            if( inst.path == *sheet )
            {
                for( SCH_ITEM* item : inst.items )
                    netOf[item] = net.name;
            }
        }
    }

    auto pt = []( const VECTOR2I& p ) { return nlohmann::json::array( { toMm( p.x ), toMm( p.y ) } ); };
    std::vector<nlohmann::json> rows;

    for( SCH_ITEM* item : sheet->LastScreen()->Items() )
    {
        nlohmann::json row = nlohmann::json::object();
        std::string    type;

        if( item->Type() == SCH_LINE_T )
        {
            SCH_LINE* line = static_cast<SCH_LINE*>( item );
            type = line->IsWire() ? "wire" : line->IsBus() ? "bus" : "graphic_line";
            row["from"] = pt( line->GetStartPoint() );
            row["to"] = pt( line->GetEndPoint() );
        }
        else if( item->Type() == SCH_JUNCTION_T || item->Type() == SCH_NO_CONNECT_T )
        {
            type = item->Type() == SCH_JUNCTION_T ? "junction" : "no_connect";
            row["at"] = pt( item->GetPosition() );
        }
        else if( item->Type() == SCH_SYMBOL_T )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( !symbol->IsPower() )
                continue;   // parts: sch_symbol_list

            type = "power";
            std::vector<SCH_PIN*> pins = symbol->GetPins( &*sheet );
            row["ref"] = str( symbol->GetRef( &*sheet, false ) );
            row["value"] = str( symbol->GetValue( &*sheet, FOR_GUI ) );
            row["at"] = pt( pins.empty() ? symbol->GetPosition() : pins.front()->GetPosition() );

            if( !pins.empty() && netOf.count( pins.front() ) )
                row["net"] = netOf[pins.front()];
        }
        else if( const char* kind = labelKind( item->Type() ) )
        {
            type = "label";
            row["kind"] = kind;
            row["text"] = str( UnescapeString( static_cast<SCH_LABEL_BASE*>( item )->GetText() ) );
            row["at"] = pt( item->GetPosition() );
        }
        else
        {
            continue;
        }

        if( !types.empty() && !types.count( type ) )
            continue;

        if( !row.contains( "net" ) && netOf.count( item ) )
            row["net"] = netOf[item];

        if( !KopenapiGlob( netGlob, row.value( "net", std::string() ) ) )
            continue;

        if( near )
        {
            const BOX2I probe( *near - VECTOR2I( radius, radius ), VECTOR2I( 2 * radius, 2 * radius ) );

            if( !item->GetBoundingBox().Intersects( probe ) )
                continue;
        }

        row["type"] = type;
        row["uuid"] = str( item->m_Uuid.AsString() );
        rows.push_back( std::move( row ) );
    }

    return KOPENAPI_RESULT::Ok( KopenapiPage( rows, aArgs ) );
}


static KOPENAPI_RESULT h_sch_label_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string text = aArgs.value( "text", std::string() );
    const std::string kind = aArgs.value( "kind", std::string( "local" ) );

    if( text.empty() || !aArgs.contains( "x_mm" ) || !aArgs.contains( "y_mm" ) )
        return KOPENAPI_RESULT::Error( 400, "give 'text', 'x_mm' and 'y_mm' (the end of a wire or a pin end)" );

    if( kind != "local" && kind != "global" && kind != "hierarchical" )
        return KOPENAPI_RESULT::Error( 400, "kind must be local, global or hierarchical" );

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    const VECTOR2I point( toIU( aArgs["x_mm"].get<double>() ), toIU( aArgs["y_mm"].get<double>() ) );
    SCH_SCREEN*    screen = sheet->LastScreen();

    // Text side: given, else away from the wire the label ends
    std::string side = aArgs.value( "side", std::string( "auto" ) );

    if( side == "auto" )
    {
        side = "right";

        for( SCH_ITEM* item : screen->Items().OfType( SCH_LINE_T ) )
        {
            SCH_LINE* wire = static_cast<SCH_LINE*>( item );

            if( !wire->IsWire() || ( wire->GetStartPoint() != point && wire->GetEndPoint() != point ) )
                continue;

            const VECTOR2I other = wire->GetStartPoint() == point ? wire->GetEndPoint() : wire->GetStartPoint();
            const VECTOR2I d = point - other;
            side = std::abs( d.x ) >= std::abs( d.y ) ? ( d.x < 0 ? "left" : "right" ) : ( d.y < 0 ? "up" : "down" );
            break;
        }
    }

    static const std::map<std::string, SPIN_STYLE> spins = { { "left", SPIN_STYLE::LEFT },
                                                             { "right", SPIN_STYLE::RIGHT },
                                                             { "up", SPIN_STYLE::UP },
                                                             { "down", SPIN_STYLE::BOTTOM } };
    auto spin = spins.find( side );

    if( spin == spins.end() )
        return KOPENAPI_RESULT::Error( 400, "side must be auto, left, right, up or down" );

    const wxString  wxText = wxString::FromUTF8( text );
    SCH_LABEL_BASE* label = nullptr;

    if( kind == "global" )
        label = new SCH_GLOBALLABEL( point, wxText );
    else if( kind == "hierarchical" )
        label = new SCH_HIERLABEL( point, wxText );
    else
        label = new SCH_LABEL( point, wxText );

    label->SetSpinStyle( spin->second );

    SCH_COMMIT commit( context->GetToolManager() );
    commit.Add( label, screen );
    pushEdit( commit, context->GetSchematic(), _( "Label (API)" ), aCtx.kiway, { label->m_Uuid } );

    // What it joined
    std::string net;
    int         pins = 0;

    for( const NET_ENTRY& entry : collectNets( context->GetSchematic() ) )
    {
        for( const NET_INSTANCE& inst : entry.instances )
        {
            if( inst.path == *sheet && std::find( inst.items.begin(), inst.items.end(), label ) != inst.items.end() )
            {
                net = entry.name;
                pins = static_cast<int>( netPins( entry ).size() );
            }
        }
    }

    nlohmann::json result = { { "uuid", str( label->m_Uuid.AsString() ) }, { "net", net }, { "pins_on_net", pins }, { "side", side } };

    if( pins == 0 )
        result["warning"] = "the label touches no wire end or pin: it connects nothing yet";

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_sch_no_connect( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "pins" ) || !aArgs["pins"].is_array() || aArgs["pins"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'pins' (array of REF.PIN)" );

    std::vector<PIN_AT> pins;

    for( const nlohmann::json& p : aArgs["pins"] )
    {
        std::optional<PIN_AT> at = p.is_string() ? findPin( context->GetSchematic(), p.get<std::string>() ) : std::nullopt;

        if( !at )
            return KOPENAPI_RESULT::Error( 404, "pin not found: " + ( p.is_string() ? p.get<std::string>() : p.dump() ) );

        pins.push_back( *at );
    }

    SCH_COMMIT     commit( context->GetToolManager() );
    nlohmann::json placed = nlohmann::json::array();

    for( const PIN_AT& at : pins )
    {
        auto* flag = new SCH_NO_CONNECT( at.pin->GetPosition() );
        commit.Add( flag, at.path.LastScreen() );
        placed.push_back( { { "pin", pinId( at.pin, at.path ) }, { "uuid", str( flag->m_Uuid.AsString() ) } } );
    }

    std::vector<KIID> added;

    for( const nlohmann::json& p : placed )
        added.emplace_back( wxString::FromUTF8( p["uuid"].get<std::string>() ) );

    pushEdit( commit, context->GetSchematic(), _( "No-connect (API)" ), aCtx.kiway, added );
    return KOPENAPI_RESULT::Ok( { { "placed", placed } } );
}


static KOPENAPI_RESULT h_sch_annotate( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    SCH_COMMIT commit( context->GetToolManager() );
    const int  changed = annotate( context->GetSchematic(), commit, aArgs.value( "reset", false ) );
    pushEdit( commit, context->GetSchematic(), _( "Annotate (API)" ) );

    return KOPENAPI_RESULT::Ok( { { "changed", changed } } );
}


static const char* EDIT_NOTE = " One commit = one undo step in the GUI.";

KOPENAPI_REGISTER( "sch_new",
                   "Create a new empty schematic file (current KiCad format) and open it; a project is "
                   "created around it",
                   R"json({"type":"object","required":["path"],"properties":{
                        "path":{"type":"string","description":"absolute path ending in .kicad_sch"},
                        "paper":{"type":"string","default":"A4"},
                        "overwrite":{"type":"boolean","default":false},
                        "discard":{"type":"boolean","default":false,"description":"drop unsaved changes of the open schematic"}}})json"_json,
                   false, h_sch_new );

KOPENAPI_REGISTER( "sch_symbol_add",
                   std::string( "Place a library symbol (part) on a sheet: lib_id from sch_lib_symbol_search, "
                                "position in mm (snapped to the 1.27 mm grid), rotation, mirror, unit, value, "
                                "footprint, fields, dnp; annotated automatically unless ref is given; returns "
                                "uuid, ref and every pin with its position." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["lib_id","x_mm","y_mm"],"properties":{
                        "lib_id":{"type":"string"},"x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "sheet":{"type":"string","description":"sheet path from sch_sheet_list; default current/root"},
                        "unit":{"type":"integer","default":1},
                        "rotation":{"type":"integer","enum":[0,90,180,270],"default":0},
                        "mirror":{"type":"string","enum":["none","x","y"],"default":"none"},
                        "ref":{"type":"string"},"value":{"type":"string"},"footprint":{"type":"string"},
                        "fields":{"type":"object","additionalProperties":{"type":"string"}},
                        "dnp":{"type":"boolean"},"in_bom":{"type":"boolean"},"on_board":{"type":"boolean"},
                        "annotate":{"type":"boolean","default":true}}})json"_json,
                   false, h_sch_symbol_add, 120 );

KOPENAPI_REGISTER( "sch_symbol_update",
                   std::string( "Edit a placed symbol by uuid or ref: value, footprint, fields, new_ref, "
                                "position, rotation, mirror, dnp / in_bom / on_board, field text positions "
                                "(field_positions, autoplace_fields); moving drags labels, power symbols and wires "
                                "on its pins and re-routes wires that would turn diagonal." ) + EDIT_NOTE,
                   R"json({"type":"object","properties":{
                        "uuid":{"type":"string"},"ref":{"type":"string","description":"lookup by reference"},
                        "new_ref":{"type":"string"},"value":{"type":"string"},"footprint":{"type":"string"},
                        "fields":{"type":"object","additionalProperties":{"type":"string"}},
                        "x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "rotation":{"type":"integer","enum":[0,90,180,270]},
                        "mirror":{"type":"string","enum":["none","x","y"]},
                        "dnp":{"type":"boolean"},"in_bom":{"type":"boolean"},"on_board":{"type":"boolean"},
                        "field_positions":{"type":"object","description":"per field name: {x_mm, y_mm}, \"hide\" or \"show\""},
                        "autoplace_fields":{"type":"boolean","default":false,"description":"let KiCad place reference / value text"}}})json"_json,
                   false, h_sch_symbol_update );

KOPENAPI_REGISTER( "sch_item_delete",
                   std::string( "Delete schematic items (symbols, labels, wires, no-connects...) by uuid." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["uuids"],"properties":{
                        "uuids":{"type":"array","items":{"type":"string"}}}})json"_json,
                   false, h_sch_item_delete );

KOPENAPI_REGISTER( "sch_connect",
                   std::string( "Connect pins into a net without drawing wires: puts a label named 'net' on "
                                "every pin (REF.PIN), oriented away from the part; kind local (this sheet), "
                                "global (all sheets) or power (a power symbol, e.g. power:GND / power:+3V3, so "
                                "ERC sees the rail); confirms the pins ended on one net." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["net","pins"],"properties":{
                        "net":{"type":"string"},
                        "pins":{"type":"array","items":{"type":"string"},"description":"REF.PIN, e.g. [\"U1.3\",\"C1.1\"]"},
                        "kind":{"type":"string","enum":["local","global","power"],"default":"local"},
                        "power_lib_id":{"type":"string","description":"power symbol, default power:<net>"}}})json"_json,
                   false, h_sch_connect, 120 );

KOPENAPI_REGISTER( "sch_wire",
                   std::string( "Draw a wire between two pins (REF.PIN) or points {x_mm, y_mm} on one sheet: "
                                "straight when aligned, else one corner (route auto/hv/vh); junction dots "
                                "where it meets other wires; returns the segments and the resulting net. "
                                "Use for short local connections, sch_connect labels for long ones." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["from","to"],"properties":{
                        "from":{"description":"REF.PIN or {x_mm, y_mm}"},
                        "to":{"description":"REF.PIN or {x_mm, y_mm}"},
                        "route":{"type":"string","enum":["auto","hv","vh"],"default":"auto"},
                        "sheet":{"type":"string","description":"for point-only wires"}}})json"_json,
                   false, h_sch_wire );

KOPENAPI_REGISTER( "sch_item_list",
                   "List the wiring items of a sheet with uuids (to inspect, move or delete them): wires and "
                   "buses (from, to), labels (kind, text, at), junctions, no-connects, power symbols; each with "
                   "its net; filter by types, net glob, near a point; paginated. Parts: sch_symbol_list",
                   KopenapiPagedSchema( R"json({
                        "sheet":{"type":"string"},
                        "types":{"type":"array","items":{"type":"string","enum":["wire","bus","graphic_line","label","junction","no_connect","power"]}},
                        "net":{"type":"string","description":"net name glob"},
                        "near":{"type":"object","properties":{"x_mm":{"type":"number"},"y_mm":{"type":"number"}}},
                        "radius_mm":{"type":"number","default":5.08}})json"_json ),
                   false, h_sch_item_list );

KOPENAPI_REGISTER( "sch_label_add",
                   std::string( "Place a net label at a point - the end of a wire or a pin end - so it names / "
                                "joins that net (local, global or hierarchical); text side auto (away from the "
                                "wire) or left/right/up/down; answers the net it joined and warns when it "
                                "touches nothing. Labels straight on pins: sch_connect." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["text","x_mm","y_mm"],"properties":{
                        "text":{"type":"string"},"x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "kind":{"type":"string","enum":["local","global","hierarchical"],"default":"local"},
                        "side":{"type":"string","enum":["auto","left","right","up","down"],"default":"auto"},
                        "sheet":{"type":"string"}}})json"_json,
                   false, h_sch_label_add );

KOPENAPI_REGISTER( "sch_no_connect",
                   std::string( "Mark pins (REF.PIN) as intentionally unconnected (no-connect flag), so ERC "
                                "does not report them." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["pins"],"properties":{
                        "pins":{"type":"array","items":{"type":"string"}}}})json"_json,
                   false, h_sch_no_connect );

KOPENAPI_REGISTER( "sch_annotate",
                   std::string( "Annotate references (U1, R2...) over all sheets, by X position, first free "
                                "number; keeps existing references unless reset; power symbols numbered "
                                "#PWR." ) + EDIT_NOTE,
                   R"json({"type":"object","properties":{"reset":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_annotate );
