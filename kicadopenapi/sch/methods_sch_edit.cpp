/// @file methods_sch_edit.cpp
/// @brief kicadopenapi schematic edits (step d4): sch_new, sch_symbol_add, sch_symbol_update,
/// sch_item_delete, sch_connect, sch_no_connect, sch_annotate, sch_sheet_add, sch_sheet_pin_add.
///
/// Every method is one SCH_COMMIT through the context's tool manager: in the GUI it is one undo
/// step, repaints and marks the document modified, like an edit made by hand; headless the
/// commit recalculates connectivity the same way.  Connections are made with labels (or power
/// symbols) on pins, so an agent never routes wires geometrically.  Coordinates are mm on the
/// 1.27 mm (50 mil) connection grid.
#include "kopenapi_sch.h"
#include "kopenapi_sch_model.h"
#include "kopenapi_sch_router.h"

#include <api/sch_api_save.h>
#include <api/sch_context.h>
#include <kicadopenapi_keepalive.h>
#include <base_units.h>
#include <kicadopenapi_glow_view.h>
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
#include <sch_io/sch_io.h>
#include <sch_io/sch_io_mgr.h>
#include <sch_file_versions.h>
#include <sch_label.h>
#include <sch_no_connect.h>
#include <sch_pin.h>
#include <sch_reference_list.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <string_utils.h>
#include <tool/tool_manager.h>
#include <tools/sch_tool_utils.h>
#include <view/view.h>
#include <view/view_overlay.h>
#include <gal/painter.h>
#include <layer_ids.h>
#include <project.h>
#include <wildcards_and_files_ext.h>
#include <wx_filename.h>

#include <wx/filename.h>
#include <wx/ffile.h>
#include <wx/timer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <set>

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


/// @brief Target sheet instance: args "sheet" (human path as listed by sch_sheet_list), else the
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
    KopenapiWaitLibraries( adapter );

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


/// @brief The items of a sheet's screen
std::vector<SCH_ITEM*> screenItems( SCH_SCREEN* aScreen )
{
    std::vector<SCH_ITEM*> items;

    for( SCH_ITEM* item : aScreen->Items() )
        items.push_back( item );

    return items;
}


/// @brief Symbol by "uuid" or "ref" (unique among sheet instances); aError/aStatus set when not found
std::optional<SYMBOL_AT> findSymbol( SCHEMATIC* aSchematic, const nlohmann::json& aArgs, int& aStatus, std::string& aError )
{
    std::string       uuid = aArgs.value( "uuid", std::string() );
    const std::string ref = aArgs.value( "ref", std::string() );
    std::vector<SYMBOL_AT> found;

    if( uuid.empty() && ref.empty() )
    {
        aStatus = 400;
        aError = "give 'uuid' or 'ref'";
        return std::nullopt;
    }

    if( !uuid.empty() )
    {
        std::vector<SCH_ITEM*> symbols;

        for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
        {
            for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
                symbols.push_back( item );
        }

        std::string error;

        if( SCH_ITEM* hit = KopenapiSchMatchUuid( symbols, uuid, error ) )
        {
            uuid = str( hit->m_Uuid.AsString() );
        }
        else if( error.find( "ambiguous" ) != std::string::npos )
        {
            aStatus = 400;
            aError = error;
            return std::nullopt;
        }
    }

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


/// @brief Pin by REF.PIN (any unit of a multi-unit part)
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


/// @brief Unit vector pointing out of the symbol body at a pin (from the pin root to its connection end)
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


/// @brief Apply value / footprint / free fields / flags from args to a symbol instance
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

    if( aArgs.contains( "in_pos" ) && aArgs["in_pos"].is_boolean() )
        aSymbol->SetExcludedFromPosFiles( !aArgs["in_pos"].get<bool>() );

    if( aArgs.contains( "ref" ) && aArgs["ref"].is_string() && !aArgs["ref"].get<std::string>().empty() )
        aSymbol->SetRef( &aPath, wxString::FromUTF8( aArgs["ref"].get<std::string>() ) );
}


/// @brief Annotate like the editor's "Annotate Schematic" with "keep existing annotation" (or reset):
/// all sheets, by X position, first free number; power symbols get #PWR numbers.  Only symbols
/// whose reference changes are touched (each recorded in the commit).
int annotate( SCHEMATIC* aSchematic, SCH_COMMIT& aCommit, bool aReset, bool aOverrideLocks = true, int* aLockedKept = nullptr )
{
    SCH_SHEET_LIST               sheets = aSchematic->Hierarchy();
    SCH_REFERENCE_LIST           all, references, keptLocked;
    SCH_MULTI_UNIT_REFERENCE_MAP locked;

    sheets.GetSymbols( all, SYMBOL_FILTER_ALL );
    sheets.GetMultiUnitSymbols( locked, SYMBOL_FILTER_NON_POWER );

    // a reset renumbers everything but the locked symbols: their references stay taken
    for( size_t i = 0; i < all.GetCount(); ++i )
    {
        if( aReset && !aOverrideLocks && all[i].GetSymbol()->IsLocked() )
            keptLocked.AddItem( all[i] );
        else
            references.AddItem( all[i] );
    }

    if( aLockedKept )
        *aLockedKept = int( keptLocked.GetCount() );

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
    references.AnnotateByOptions( SORT_BY_X_POSITION, INCREMENTAL_BY_REF, 0, locked, keptLocked, false );

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


/// @brief First free reference number for a prefix ("C" -> "C4"), over all sheet instances plus the
/// references already handed out in this commit (aTaken).  New symbols get their reference
/// before the commit, so placing and numbering stay one undo step.
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


/// @brief Push and bring connectivity fully up to date.  The incremental update after a commit misses
/// labels placed directly on a pin end without a wire (seen: two labels "USB_DP" on two pins
/// stayed unconnected until the file was reloaded), so every edit ends with a full rebuild —
/// the same the netlist exporter does.
/// Schematic editor side of the GUI glow (kicadopenapi_glow.h)
struct SCH_GLOW_TRAITS
{
    using FRAME = SCH_EDIT_FRAME;

    static FRAME* Frame( KIWAY* aKiway )
    {
        return aKiway ? static_cast<SCH_EDIT_FRAME*>( aKiway->Player( FRAME_SCH, false ) ) : nullptr;
    }

    static EDA_ITEM* Resolve( FRAME* aFrame, const KIID& aId )
    {
        return aFrame->Schematic().Hierarchy().ResolveItem( aId, nullptr, true );
    }

    static void Brighten( FRAME* aFrame, EDA_ITEM* aItem, bool aOn )
    {
        if( aOn )
            aItem->SetBrightened();
        else
            aItem->ClearBrightened();

        aFrame->UpdateItem( aItem );
    }

    static BOX2I Box( EDA_ITEM* aItem ) { return aItem->GetBoundingBox(); }

    static int Mm() { return schIUScale.mmToIU( 1.0 ); }

    /// @brief Brightened items draw in the glow colour while anything glows; KiCad's own colour after
    static void ItemColour( FRAME* aFrame, bool aGlow, std::optional<KIGFX::COLOR4D>& aSaved )
    {
        KIGFX::RENDER_SETTINGS* settings = aFrame->GetCanvas()->GetView()->GetPainter()->GetSettings();

        if( aGlow && !aSaved )
        {
            aSaved = settings->GetLayerColor( LAYER_BRIGHTENED );
            settings->SetLayerColor( LAYER_BRIGHTENED, KopenapiGlowColour( 1.0 ) );
        }
        else if( !aGlow && aSaved )
        {
            settings->SetLayerColor( LAYER_BRIGHTENED, *aSaved );
            aSaved.reset();
        }
    }
};


void glow( KIWAY* aKiway, const std::vector<KIID>& aItems )
{
    KopenapiGlow<SCH_GLOW_TRAITS>( aKiway, aItems );
}


/// @brief 409 when a move dragged locked wiring along (items in aTouched other than the moved ones),
/// unless the request overrides locks
std::optional<KOPENAPI_RESULT> lockedDragged( SCH_SCREEN* aScreen, const std::vector<KIID>& aTouched,
                                              const std::set<SCH_ITEM*>& aMoved, const nlohmann::json& aArgs )
{
    if( aArgs.value( "override_locks", false ) )
        return std::nullopt;

    const std::set<KIID> touched( aTouched.begin(), aTouched.end() );
    nlohmann::json       locked = nlohmann::json::array();

    for( SCH_ITEM* item : aScreen->Items() )
    {
        if( item->IsLocked() && !aMoved.count( item ) && touched.count( item->m_Uuid ) )
            locked.push_back( str( item->m_Uuid.AsString() ) );
    }

    if( locked.empty() )
        return std::nullopt;

    KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "the move would stretch locked wiring (sch_unlock it, or "
                                                           "override_locks: true); nothing was changed" );
    refused.body["locked"] = locked;
    return refused;
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
            const BOX2I box = field->GetBoundingBox();
            fields[str( field->GetName() )] = { { "x_mm", toMm( field->GetPosition().x ) },
                                                { "y_mm", toMm( field->GetPosition().y ) },
                                                { "visible", field->IsVisible() },
                                                { "box_mm", { toMm( box.GetLeft() ), toMm( box.GetTop() ),
                                                              toMm( box.GetRight() ), toMm( box.GetBottom() ) } } };
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


/// @brief Put a symbol's visible field texts (reference, value, ...) where they lie on nothing else: where
/// they are (unless aReset), KiCad's own placement, then the whole text block right of, below,
/// above or left of the part at a growing distance.  Obstacles: other parts (body and pins), their
/// texts, labels, wires.  False (texts left where KiCad put them) when no candidate is clear.
bool placeFieldsClear( SCH_SYMBOL* aSymbol, SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath, bool aReset )
{
    if( aReset )
        aSymbol->AutoplaceFields( aScreen, AUTOPLACE_AUTO );

    std::vector<SCH_FIELD*> fields;

    for( SCH_FIELD& field : aSymbol->GetFields() )
    {
        if( field.IsVisible() && !field.GetShownText( &aPath, FOR_GUI ).IsEmpty() )
            fields.push_back( &field );
    }

    if( fields.empty() )
        return true;

    std::vector<BOX2I> obstacles;

    for( SCH_ITEM* item : aScreen->Items() )
    {
        if( item == aSymbol )
            continue;

        if( item->Type() == SCH_SYMBOL_T )
        {
            SCH_SYMBOL* other = static_cast<SCH_SYMBOL*>( item );
            obstacles.push_back( other->GetBodyAndPinsBoundingBox() );

            for( SCH_FIELD& field : other->GetFields() )
            {
                if( field.IsVisible() && !field.GetShownText( &aPath, FOR_GUI ).IsEmpty() )
                    obstacles.push_back( field.GetBoundingBox() );
            }
        }
        else if( item->Type() == SCH_LINE_T || labelKind( item->Type() ) || item->Type() == SCH_TEXT_T
                 || item->Type() == SCH_SHEET_T || item->Type() == SCH_NO_CONNECT_T )
        {
            obstacles.push_back( item->GetBoundingBox() );
        }
    }

    const BOX2I own = aSymbol->GetBodyAndPinsBoundingBox();

    auto blockBox = [&]()
    {
        BOX2I block = fields.front()->GetBoundingBox();

        for( SCH_FIELD* field : fields )
            block.Merge( field->GetBoundingBox() );

        return block;
    };

    // texts keep a little air to lines and bodies, not just no overlap
    const int air = toIU( 0.5 );

    auto clear = [&]( const BOX2I& aBlock )
    {
        const BOX2I block = aBlock.GetInflated( air );

        if( block.Intersects( own ) )
            return false;

        for( const BOX2I& box : obstacles )
        {
            if( block.Intersects( box ) )
                return false;
        }

        return true;
    };

    if( !aReset && clear( blockBox() ) )
        return true;

    if( !aReset )
        aSymbol->AutoplaceFields( aScreen, AUTOPLACE_AUTO );

    const BOX2I start = blockBox();
    const int   blockW = static_cast<int>( start.GetWidth() ), blockH = static_cast<int>( start.GetHeight() );

    if( clear( start ) )
        return true;

    const int grid = toIU( GRID_MM );
    auto      snap = [&]( int v ) { return int( std::lround( double( v ) / grid ) ) * grid; };

    for( int gap : { 1, 2, 4, 6, 10 } )
    {
        const int g = gap * grid;
        const std::vector<VECTOR2I> targets = {
            // top-left corner of the block for: right, below, above, left
            { own.GetRight() + g, own.Centre().y - blockH / 2 },
            { own.Centre().x - blockW / 2, own.GetBottom() + g },
            { own.Centre().x - blockW / 2, own.GetTop() - g - blockH },
            { own.GetLeft() - g - blockW, own.Centre().y - blockH / 2 } };

        for( const VECTOR2I& target : targets )
        {
            const VECTOR2I delta( snap( target.x - start.GetLeft() ), snap( target.y - start.GetTop() ) );
            BOX2I          moved = start;
            moved.Move( delta );

            if( !clear( moved ) )
                continue;

            for( SCH_FIELD* field : fields )
                field->SetPosition( field->GetPosition() + delta );

            return true;
        }
    }

    return false;
}


/// @brief After a part lands: its own texts on a clear spot, and the texts of other parts that its
/// body now covers moved off it (all in aCommit)
void settleTexts( SCH_SYMBOL* aSymbol, SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath, SCH_COMMIT& aCommit )
{
    aCommit.Modify( aSymbol, aScreen );
    placeFieldsClear( aSymbol, aScreen, aPath, false );

    const BOX2I body = aSymbol->GetBodyAndPinsBoundingBox();

    for( SCH_ITEM* item : aScreen->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* other = static_cast<SCH_SYMBOL*>( item );

        if( other == aSymbol )
            continue;

        for( SCH_FIELD& field : other->GetFields() )
        {
            if( field.IsVisible() && !field.GetShownText( &aPath, FOR_GUI ).IsEmpty()
                && field.GetBoundingBox().Intersects( body ) )
            {
                aCommit.Modify( other, aScreen );
                placeFieldsClear( other, aScreen, aPath, false );
                break;
            }
        }
    }
}


/// @brief When a footprint was given: does it exist and does every pin find its pad (sch_footprint_check)
void attachFootprintCheck( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, nlohmann::json& aCard )
{
    if( aArgs.value( "footprint", std::string() ).empty() )
        return;

    std::optional<KOPENAPI_METHOD> check = KOPENAPI_REGISTRY::Get().Find( "sch_footprint_check" );

    if( !check )
        return;

    KOPENAPI_RESULT result = check->handler( aCtx, { { "ref", aCard.value( "ref", std::string() ) }, { "problems_only", false } } );

    if( result.status != 200 )
        return;

    const nlohmann::json& items = result.body["items"];
    aCard["footprint_check"] = items.empty() ? nlohmann::json( { { "status", "ok" } } ) : items[0];
}


/// @brief The net a pin is on now (after a commit), for confirming connections
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



/// @brief Direction from a power symbol's pin to the centre of its body (power pins have zero length)
VECTOR2I powerBodyDirection( SCH_SYMBOL* aSymbol, const SCH_SHEET_PATH& aPath )
{
    std::vector<SCH_PIN*> own = aSymbol->GetPins( &aPath );
    const VECTOR2I        pin = own.empty() ? aSymbol->GetPosition() : own.front()->GetPosition();
    const VECTOR2I        d = aSymbol->GetBodyBoundingBox().Centre() - pin;

    if( std::abs( d.x ) >= std::abs( d.y ) )
        return VECTOR2I( d.x < 0 ? -1 : 1, 0 );

    return VECTOR2I( 0, d.y < 0 ? -1 : 1 );
}


/// @brief Where a power symbol's body points in its library pose (GND down, +5V up, ...)
VECTOR2I powerNaturalDirection( LIB_SYMBOL* aLib, SCHEMATIC* aSchematic, const SCH_SHEET_PATH& aPath )
{
    std::unique_ptr<LIB_SYMBOL> flat = aLib->Flatten();
    SCH_SYMBOL                  probe( *flat, aLib->GetLibId(), &aPath, 1, 1, VECTOR2I( 0, 0 ), aSchematic );
    return powerBodyDirection( &probe, aPath );
}


/// @brief A power symbol with its pin on aPoint and its body pointing aBody, numbered #PWR<n>
SCH_SYMBOL* makePowerSymbol( LIB_SYMBOL* aLib, SCHEMATIC* aSchematic, const SCH_SHEET_PATH& aPath, const VECTOR2I& aPoint,
                             const VECTOR2I& aBody, std::set<wxString>& aTaken, const wxString& aNet = wxEmptyString )
{
    std::unique_ptr<LIB_SYMBOL> flat = aLib->Flatten();
    auto*                       symbol = new SCH_SYMBOL( *flat, aLib->GetLibId(), &aPath, 1, 1, aPoint, aSchematic );

    for( int orientation : { SYM_ORIENT_0, SYM_ORIENT_90, SYM_ORIENT_180, SYM_ORIENT_270 } )
    {
        symbol->SetOrientation( orientation );

        if( powerBodyDirection( symbol, aPath ) == aBody )
            break;
    }

    if( std::vector<SCH_PIN*> own = symbol->GetPins( &aPath ); !own.empty() )
        symbol->SetPosition( symbol->GetPosition() + ( aPoint - own.front()->GetPosition() ) );

    const bool flag = aLib->GetName().CmpNoCase( wxS( "PWR_FLAG" ) ) == 0;
    symbol->SetRef( &aPath, nextFreeRef( aSchematic, flag ? wxS( "#FLG" ) : wxS( "#PWR" ), aTaken ) );

    // a power symbol's value names its net: power:+3V3 drawn for the rail +3V3_PM
    if( !flag && !aNet.IsEmpty() && symbol->GetField( FIELD_T::VALUE )->GetText() != aNet )
        symbol->GetField( FIELD_T::VALUE )->SetText( aNet );

    symbol->AutoplaceFields( nullptr, AUTOPLACE_AUTO );
    return symbol;
}


/// @brief Carries the wiring along when parts move (sch_symbol_update, sch_items_move).
///
/// From every connection point that moved, the attachment tree is collected: wires, junctions,
/// labels, no-connects and power symbols (PWR_FLAG included) reachable without touching any other
/// part.  A tree that belongs to the moving items alone moves with them as a rigid body (a stub
/// with its power symbol stays a stub); a tree shared with other parts keeps its place — wire
/// ends on the moved points stretch, labels / no-connects sitting on the point follow — and wires
/// that turn diagonal are re-routed orthogonally.  A moving power symbol never drags another one.
class SHEET_DRAG
{
public:
    SHEET_DRAG( SCH_COMMIT& aCommit, SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath ) :
            m_commit( aCommit ), m_screen( aScreen ), m_path( aPath )
    {}

    /// @brief Items moved by the caller (not to be carried again)
    void SetAnchors( const std::set<SCH_ITEM*>& aAnchors ) { m_anchors = aAnchors; }

    /// @brief A connection point of a moved item went from aFrom to aTo; aPowerMover: it was a power symbol
    void AddPointMove( const VECTOR2I& aFrom, const VECTOR2I& aTo, bool aPowerMover )
    {
        if( aFrom != aTo )
            m_moves.push_back( { aFrom, aTo, aPowerMover } );
    }

    /// @brief Apply; returns warnings, fills aTouched with everything changed
    nlohmann::json Apply( std::vector<KIID>& aTouched )
    {
        // Decide everything on the geometry before the move: a tree moved first must not look
        // like a wire through the next point
        struct PLAN
        {
            const MOVE*            move;
            bool                   rigid;
            std::set<SCH_ITEM*>    tree;
            std::vector<SCH_ITEM*> here;
        };

        std::vector<PLAN> plans;

        for( const MOVE& move : m_moves )
        {
            PLAN plan{ &move, false, {}, {} };
            plan.rigid = !move.powerMover && collectTree( move.from, move.to - move.from, plan.tree );

            if( !plan.rigid )
            {
                plan.tree.clear();

                for( SCH_ITEM* item : m_screen->Items().Overlapping( move.from ) )
                    plan.here.push_back( item );
            }

            plans.push_back( std::move( plan ) );
        }

        std::vector<SCH_LINE*> stretched;

        for( PLAN& plan : plans )
        {
            const MOVE&    move = *plan.move;
            const VECTOR2I delta = move.to - move.from;

            if( plan.rigid )
            {
                for( SCH_ITEM* item : plan.tree )
                {
                    if( m_done.count( item ) )
                        continue;

                    m_commit.Modify( item, m_screen );
                    item->Move( delta );
                    m_done.insert( item );
                    aTouched.push_back( item->m_Uuid );
                }

                continue;
            }

            // Shared with other parts: stretch / follow at the point itself
            for( SCH_ITEM* item : plan.here )
            {
                // a wire between two moved points stretches at both ends
                const bool otherEnd = m_done.count( item ) && m_stretched.count( item );

                if( m_anchors.count( item ) || ( m_done.count( item ) && !otherEnd ) )
                    continue;

                if( labelKind( item->Type() ) || item->Type() == SCH_NO_CONNECT_T )
                {
                    if( item->GetPosition() != move.from )
                        continue;

                    m_commit.Modify( item, m_screen );
                    item->SetPosition( move.to );
                }
                else if( !move.powerMover && item->Type() == SCH_SYMBOL_T && static_cast<SCH_SYMBOL*>( item )->IsPower() )
                {
                    std::vector<SCH_PIN*> own = static_cast<SCH_SYMBOL*>( item )->GetPins( &m_path );

                    if( own.empty() || own.front()->GetPosition() != move.from )
                        continue;

                    m_commit.Modify( item, m_screen );
                    item->Move( delta );
                }
                else if( item->Type() == SCH_LINE_T && static_cast<SCH_LINE*>( item )->IsWire() )
                {
                    SCH_LINE* wire = static_cast<SCH_LINE*>( item );

                    if( wire->GetStartPoint() != move.from && wire->GetEndPoint() != move.from )
                        continue;

                    if( !otherEnd )
                        m_commit.Modify( item, m_screen );

                    if( wire->GetStartPoint() == move.from )
                        wire->SetStartPoint( move.to );
                    else
                        wire->SetEndPoint( move.to );

                    if( otherEnd )
                        continue;

                    stretched.push_back( wire );
                    m_stretched.insert( item );
                }
                else
                {
                    continue;
                }

                m_done.insert( item );
                aTouched.push_back( item->m_Uuid );
            }
        }

        return reroute( stretched, aTouched );
    }

private:
    struct MOVE
    {
        VECTOR2I from, to;
        bool     powerMover;
    };

    /// @brief Wiring reachable from aStart that touches no other part; false if it does
    bool collectTree( const VECTOR2I& aStart, const VECTOR2I& aDelta, std::set<SCH_ITEM*>& aTree )
    {
        std::vector<VECTOR2I> todo = { aStart };
        std::set<std::pair<int, int>> seen;

        while( !todo.empty() )
        {
            const VECTOR2I p = todo.back();
            todo.pop_back();

            if( !seen.insert( { p.x, p.y } ).second )
                continue;

            for( SCH_ITEM* item : m_screen->Items().Overlapping( p ) )
            {
                if( m_anchors.count( item ) )
                {
                    // the moving items' own other connection points: fine only if they move alike
                    if( item->Type() == SCH_SYMBOL_T && p != aStart && !movesBy( p, aDelta ) )
                    {
                        for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( item )->GetPins( &m_path ) )
                        {
                            if( pin->GetPosition() == p )
                                return false;
                        }
                    }

                    continue;
                }

                switch( item->Type() )
                {
                case SCH_LINE_T:
                {
                    SCH_LINE* line = static_cast<SCH_LINE*>( item );

                    if( !line->IsWire() )
                        return false;

                    if( line->GetStartPoint() != p && line->GetEndPoint() != p )
                        return false;   // the point is in the middle of a wire: a T into shared wiring

                    if( aTree.insert( item ).second )
                    {
                        todo.push_back( line->GetStartPoint() == p ? line->GetEndPoint() : line->GetStartPoint() );

                        // another wire ending in the middle of this one joins it
                        for( SCH_ITEM* other : m_screen->Items().OfType( SCH_LINE_T ) )
                        {
                            SCH_LINE* o = static_cast<SCH_LINE*>( other );

                            for( const VECTOR2I& end : { o->GetStartPoint(), o->GetEndPoint() } )
                            {
                                if( other != item && end != line->GetStartPoint() && end != line->GetEndPoint()
                                    && line->HitTest( end, 0 ) )
                                {
                                    return false;
                                }
                            }
                        }
                    }

                    break;
                }
                case SCH_SYMBOL_T:
                {
                    SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );
                    bool        pinHere = false;

                    for( SCH_PIN* pin : symbol->GetPins( &m_path ) )
                        pinHere |= pin->GetPosition() == p;

                    if( !pinHere )
                        break;   // only its body overlaps the point

                    if( !symbol->IsPower() )
                        return false;   // another part is on this wiring

                    aTree.insert( item );
                    break;
                }
                case SCH_JUNCTION_T:
                case SCH_NO_CONNECT_T:
                    if( item->GetPosition() == p )
                        aTree.insert( item );

                    break;

                case SCH_SHEET_T:
                case SCH_BUS_WIRE_ENTRY_T:
                case SCH_BUS_BUS_ENTRY_T:
                    return false;

                default:
                    if( labelKind( item->Type() ) && item->GetPosition() == p )
                        aTree.insert( item );

                    break;
                }
            }
        }

        return !aTree.empty();
    }

    bool movesBy( const VECTOR2I& aPoint, const VECTOR2I& aDelta ) const
    {
        for( const MOVE& m : m_moves )
        {
            if( m.from == aPoint )
                return m.to - m.from == aDelta;
        }

        return false;
    }

    /// @brief Stretched wires that turned diagonal: replace them by an orthogonal route
    nlohmann::json reroute( const std::vector<SCH_LINE*>& aStretched, std::vector<KIID>& aTouched )
    {
        nlohmann::json warnings = nlohmann::json::array();
        std::vector<SCH_LINE*> diagonal;

        for( SCH_LINE* wire : aStretched )
        {
            if( wire->GetStartPoint().x != wire->GetEndPoint().x && wire->GetStartPoint().y != wire->GetEndPoint().y )
                diagonal.push_back( wire );
        }

        if( diagonal.empty() )
            return warnings;

        std::set<const SCH_ITEM*> ignore( diagonal.begin(), diagonal.end() );
        WIRE_ROUTER router( m_screen, m_path, toIU( GRID_MM ), ignore );

        for( SCH_LINE* wire : diagonal )
        {
            const VECTOR2I a = wire->GetStartPoint(), b = wire->GetEndPoint();
            std::optional<std::vector<VECTOR2I>> path = router.Route( a, b, "hv" );

            if( !path )
            {
                warnings.push_back( "a wire stays diagonal: no clear orthogonal route from " + std::to_string( toMm( a.x ) ) + ", "
                                    + std::to_string( toMm( a.y ) ) );
                continue;
            }

            m_commit.Remove( wire, m_screen );

            for( size_t i = 0; i + 1 < path->size(); ++i )
            {
                auto* segment = new SCH_LINE( ( *path )[i], LAYER_WIRE );
                segment->SetEndPoint( ( *path )[i + 1] );
                m_commit.Add( segment, m_screen );
                router.AddWire( ( *path )[i], ( *path )[i + 1] );
                aTouched.push_back( segment->m_Uuid );
            }
        }

        return warnings;
    }

    SCH_COMMIT&           m_commit;
    SCH_SCREEN*           m_screen;
    SCH_SHEET_PATH        m_path;
    std::set<SCH_ITEM*>   m_anchors;
    std::set<SCH_ITEM*>   m_done;
    std::set<SCH_ITEM*>   m_stretched;   ///< wires stretched at one end so far
    std::vector<MOVE>     m_moves;
};


VECTOR2I sheetPinOutward( const SCH_SHEET_PIN* aPin )
{
    switch( aPin->GetSide() )
    {
    case SHEET_SIDE::LEFT:  return VECTOR2I( -1, 0 );
    case SHEET_SIDE::RIGHT: return VECTOR2I( 1, 0 );
    case SHEET_SIDE::TOP:   return VECTOR2I( 0, -1 );
    default:                return VECTOR2I( 0, 1 );
    }
}


std::string sheetPinName( const SCH_SHEET_PIN* aPin )
{
    return str( UnescapeString( aPin->GetText() ) );
}


/// @brief A named connection point: a symbol pin or a sheet pin
struct END_AT
{
    SCH_ITEM*      item = nullptr;
    SCH_SHEET_PATH path;
    VECTOR2I       position;
    VECTOR2I       out;   ///< unit vector pointing away from the part / sheet
};


/// @brief REF.PIN of a symbol, else SHEET.PIN of a sheet pin (sheet name or its path as sch_sheet_list
/// prints it, then the pin name); a sheet drawn on several instances is taken on aPrefer when given
std::optional<END_AT> findEnd( SCHEMATIC* aSchematic, const std::string& aName,
                               const std::optional<SCH_SHEET_PATH>& aPrefer = std::nullopt )
{
    if( std::optional<PIN_AT> at = findPin( aSchematic, aName ) )
        return END_AT{ at->pin, at->path, at->pin->GetPosition(), outward( at->pin ) };

    std::optional<END_AT> found;

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SHEET_T ) )
        {
            SCH_SHEET*     sheet = static_cast<SCH_SHEET*>( item );
            SCH_SHEET_PATH child = path;
            child.push_back( sheet );

            const std::string name = str( sheet->GetName() ), human = sheetPath( child );

            for( SCH_SHEET_PIN* pin : sheet->GetPins() )
            {
                const std::string pinName = sheetPinName( pin );

                if( aName != name + "." + pinName && aName != human + "." + pinName )
                    continue;

                if( !found || ( aPrefer && path == *aPrefer && found->path != *aPrefer ) )
                    found = END_AT{ pin, path, pin->GetPosition(), sheetPinOutward( pin ) };
            }
        }
    }

    return found;
}


/// @brief The net an item is on now on one sheet instance (after a commit)
std::string netOfItem( SCHEMATIC* aSchematic, const SCH_ITEM* aItem, const SCH_SHEET_PATH& aPath )
{
    for( const NET_ENTRY& net : collectNets( aSchematic ) )
    {
        for( const NET_INSTANCE& inst : net.instances )
        {
            if( inst.path == aPath && std::find( inst.items.begin(), inst.items.end(), aItem ) != inst.items.end() )
                return net.name;
        }
    }

    return std::string();
}


/// @brief A wire end: REF.PIN, SHEET.PIN, or a point {x_mm, y_mm} on the target sheet
std::optional<VECTOR2I> wireEnd( SCHEMATIC* aSchematic, const nlohmann::json& aEnd, std::optional<SCH_SHEET_PATH>& aSheet,
                                 VECTOR2I& aOutward, std::string& aError )
{
    aOutward = VECTOR2I( 0, 0 );

    if( aEnd.is_string() )
    {
        std::optional<END_AT> at = findEnd( aSchematic, aEnd.get<std::string>(), aSheet );

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
        aOutward = at->out;
        return at->position;
    }

    if( aEnd.is_object() && aEnd.contains( "x_mm" ) && aEnd.contains( "y_mm" ) )
        return VECTOR2I( toIU( aEnd["x_mm"].get<double>() ), toIU( aEnd["y_mm"].get<double>() ) );

    aError = "a wire end is REF.PIN, SHEET.PIN (a sheet pin) or {x_mm, y_mm}";
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

    // Part texts the new wire runs through move clear of it
    {
        SCH_COMMIT texts( context->GetToolManager() );
        bool       moved = false;

        for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );
            bool        hit = false;

            for( SCH_FIELD& field : symbol->GetFields() )
            {
                if( !field.IsVisible() || field.GetShownText( &*sheet, FOR_GUI ).IsEmpty() )
                    continue;

                BOX2I box = field.GetBoundingBox();
                box.Inflate( schIUScale.mmToIU( 0.5 ) );   // a wire passing close counts too

                for( size_t i = 0; !hit && i + 1 < points.size(); ++i )
                {
                    SCH_LINE probe( points[i], LAYER_WIRE );
                    probe.SetEndPoint( points[i + 1] );
                    hit = box.GetWidth() > 0 && box.GetHeight() > 0 && probe.HitTest( box, false, 0 );
                }
            }

            if( hit )
            {
                texts.Modify( symbol, screen );
                placeFieldsClear( symbol, screen, *sheet, false );
                moved = true;
            }
        }

        if( moved )
            pushEdit( texts, schematic, _( "Wire (API)" ) );
    }

    nlohmann::json result = { { "segments", segments }, { "junctions", dots } };

    if( aArgs["from"].is_string() )
    {
        std::string net = netOfPin( schematic, aArgs["from"] );

        if( std::optional<END_AT> end = net.empty() ? findEnd( schematic, aArgs["from"], sheet ) : std::nullopt )
            net = netOfItem( schematic, end->item, end->path );

        result["net"] = net;
    }

    return KOPENAPI_RESULT::Ok( result );
}


namespace
{

/// @brief Pins grouped by the net they are on (net names aside): equal before and after an edit =
/// the circuit is unchanged
std::set<std::vector<std::string>> pinPartition( SCHEMATIC* aSchematic )
{
    std::map<std::string, std::vector<std::string>> byNet;

    for( const NET_ENTRY& net : collectNets( aSchematic ) )
    {
        for( const auto& [pin, path] : netPins( net ) )
        {
            if( SCH_SYMBOL* symbol = pinSymbol( pin ); symbol && !symbol->IsPower() )
                byNet[net.name].push_back( pinId( pin, path ) );
        }
    }

    std::set<std::vector<std::string>> groups;

    for( auto& [name, pins] : byNet )
    {
        std::sort( pins.begin(), pins.end() );
        groups.insert( pins );
    }

    return groups;
}

} // namespace


/// @brief sch_rewire: local labels that join pins of parts near each other become wires.  Per net
/// name on the sheet, the label-carrying pins are linked by a shortest tree; each link up to max_mm
/// is routed with the wire router (no other pin, wire end, label or body touched); in every group
/// of pins the wires now join, one label stays (the net keeps its name, far groups stay linked).
/// The pin partition before and after must be equal, else everything is put back.
static KOPENAPI_RESULT h_sch_texts_settle( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    SCH_SCREEN*       screen = sheet->LastScreen();
    SCH_COMMIT        commit( context->GetToolManager() );
    std::vector<KIID> touched;
    std::set<std::string> moved, stuck;

    auto positions = []( SCH_SYMBOL* aSymbol )
    {
        std::vector<VECTOR2I> at;

        for( SCH_FIELD& field : aSymbol->GetFields() )
            at.push_back( field.GetPosition() );

        return at;
    };

    // twice: texts moved off a spot in the first pass may free another part's spot
    for( int pass = 0; pass < 2; ++pass )
    {
        for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( symbol->IsLocked() )
                continue;

            const std::vector<VECTOR2I> before = positions( symbol );
            const bool                  clear = placeFieldsClear( symbol, screen, *sheet, false );
            const std::vector<VECTOR2I> after = positions( symbol );
            const std::string           ref = str( symbol->GetRef( &*sheet, false ) );

            if( !clear )
                stuck.insert( ref );
            else
                stuck.erase( ref );

            if( after == before )
                continue;

            // the commit keeps the state before the change: put it back, record, redo
            size_t k = 0;

            for( SCH_FIELD& field : symbol->GetFields() )
                field.SetPosition( before[k++] );

            commit.Modify( symbol, screen );
            placeFieldsClear( symbol, screen, *sheet, false );
            moved.insert( ref );
            touched.push_back( symbol->m_Uuid );
        }
    }

    if( !commit.Empty() )
        pushEdit( commit, context->GetSchematic(), _( "Settle texts (API)" ), aCtx.kiway, touched );

    return KOPENAPI_RESULT::Ok( { { "sheet", sheetPath( *sheet ) },
                                  { "moved", std::vector<std::string>( moved.begin(), moved.end() ) },
                                  { "stuck", std::vector<std::string>( stuck.begin(), stuck.end() ) } } );
}


static KOPENAPI_RESULT h_sch_rewire( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    SCHEMATIC*   schematic = context->GetSchematic();
    SCH_SCREEN*  screen = sheet->LastScreen();
    const double maxMm = aArgs.value( "max_mm", 50.8 );
    const bool   dryRun = aArgs.value( "dry_run", false );
    const std::string only = aArgs.value( "net", std::string() );

    // pin ends of the sheet's parts, with the pin's REF.PIN
    std::map<std::pair<int, int>, std::string> pinAt;

    for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

        if( symbol->IsPower() )
            continue;

        for( SCH_PIN* pin : symbol->GetPins( &*sheet ) )
            pinAt[{ pin->GetPosition().x, pin->GetPosition().y }] = pinId( pin, *sheet );
    }

    struct PIN_LABEL
    {
        SCH_LABEL_BASE* label;
        std::string     pin;
        VECTOR2I        at;
        VECTOR2I        out;
    };

    std::map<wxString, std::vector<PIN_LABEL>> groups;

    for( SCH_ITEM* item : screen->Items().OfType( SCH_LABEL_T ) )
    {
        SCH_LABEL_BASE* label = static_cast<SCH_LABEL_BASE*>( item );
        const VECTOR2I  at = label->GetPosition();
        auto            pin = pinAt.find( { at.x, at.y } );

        if( pin == pinAt.end() || ( !only.empty() && str( UnescapeString( label->GetText() ) ) != only ) )
            continue;

        std::optional<END_AT> end = findEnd( schematic, pin->second, sheet );
        groups[label->GetText()].push_back( { label, pin->second, at, end ? end->out : VECTOR2I( 0, 0 ) } );
    }

    const std::set<std::vector<std::string>> before = pinPartition( schematic );
    WIRE_ROUTER router( screen, *sheet, toIU( GRID_MM ) );
    const double maxIU = schIUScale.mmToIU( maxMm );

    std::vector<std::vector<VECTOR2I>> routes;
    std::vector<SCH_LABEL_BASE*>       drop;
    nlohmann::json                     nets = nlohmann::json::array();

    for( auto& [name, members] : groups )
    {
        if( members.size() < 2 )
            continue;

        // shortest tree over the pins (Prim, Manhattan length): the links worth a wire
        const size_t        n = members.size();
        std::vector<bool>   in( n, false );
        std::vector<double> cost( n, std::numeric_limits<double>::max() );
        std::vector<int>    from( n, -1 );
        std::vector<std::pair<int, int>> links;
        cost[0] = 0;

        for( size_t step = 0; step < n; ++step )
        {
            int next = -1;

            for( size_t i = 0; i < n; ++i )
            {
                if( !in[i] && ( next < 0 || cost[i] < cost[next] ) )
                    next = int( i );
            }

            in[next] = true;

            if( from[next] >= 0 )
                links.emplace_back( from[next], next );

            for( size_t i = 0; i < n; ++i )
            {
                const VECTOR2I d = members[i].at - members[next].at;
                const double   len = std::abs( d.x ) + std::abs( d.y );

                if( !in[i] && len < cost[i] )
                {
                    cost[i] = len;
                    from[i] = next;
                }
            }
        }

        // the links routed; groups of pins they join (union-find)
        std::vector<int> root( n );
        std::iota( root.begin(), root.end(), 0 );
        std::function<int( int )> find = [&]( int i ) { return root[i] == i ? i : root[i] = find( root[i] ); };
        nlohmann::json failed = nlohmann::json::array();
        int            wired = 0;

        for( const auto& [a, b] : links )
        {
            const VECTOR2I d = members[b].at - members[a].at;

            if( std::abs( d.x ) + std::abs( d.y ) > maxIU )
                continue;

            const VECTOR2I& out = members[a].out;
            std::string     why;
            std::optional<std::vector<VECTOR2I>> path =
                    router.Route( members[a].at, members[b].at, out.y != 0 ? "vh" : "hv", &why );

            if( !path )
            {
                failed.push_back( { { "from", members[a].pin }, { "to", members[b].pin }, { "why", why } } );
                continue;
            }

            for( size_t i = 0; i + 1 < path->size(); ++i )
                router.AddWire( ( *path )[i], ( *path )[i + 1] );

            routes.push_back( *path );
            root[find( a )] = find( b );
            wired++;
        }

        // one label per group of joined pins stays: the leftmost, then topmost
        std::map<int, int> keep;

        for( size_t i = 0; i < n; ++i )
        {
            auto [it, fresh] = keep.emplace( find( int( i ) ), int( i ) );
            const VECTOR2I& cur = members[it->second].at;

            if( !fresh && ( members[i].at.x < cur.x || ( members[i].at.x == cur.x && members[i].at.y < cur.y ) ) )
                it->second = int( i );
        }

        for( size_t i = 0; i < n; ++i )
        {
            if( keep[find( int( i ) )] != int( i ) )
                drop.push_back( members[i].label );
        }

        nets.push_back( { { "net", str( UnescapeString( name ) ) },
                          { "pins", n },
                          { "wires", wired },
                          { "labels_left", keep.size() },
                          { "not_routed", failed } } );
    }

    nlohmann::json result = { { "sheet", sheetPath( *sheet ) }, { "nets", nets }, { "wires", routes.size() },
                              { "labels_removed", drop.size() }, { "dry_run", dryRun } };

    if( dryRun || routes.empty() )
        return KOPENAPI_RESULT::Ok( result );

    SCH_COMMIT                   commit( context->GetToolManager() );
    std::vector<KIID>            added;
    std::vector<SCH_LABEL_BASE*> saved;

    for( const std::vector<VECTOR2I>& points : routes )
    {
        for( size_t i = 0; i + 1 < points.size(); ++i )
        {
            auto* wire = new SCH_LINE( points[i], LAYER_WIRE );
            wire->SetEndPoint( points[i + 1] );
            commit.Add( wire, screen );
            added.push_back( wire->m_Uuid );
        }
    }

    for( SCH_LABEL_BASE* label : drop )
    {
        saved.push_back( static_cast<SCH_LABEL_BASE*>( label->Clone() ) );
        commit.Remove( label, screen );
    }

    commit.Push( _( "Labels to wires (API)" ), SKIP_CONNECTIVITY );

    SCH_COMMIT junctions( context->GetToolManager() );

    for( const std::vector<VECTOR2I>& points : routes )
    {
        for( const VECTOR2I& point : { points.front(), points.back() } )
        {
            if( screen->IsExplicitJunctionNeeded( point ) && !screen->GetItem( point, 0, SCH_JUNCTION_T ) )
            {
                auto* junction = new SCH_JUNCTION( point );
                junctions.Add( junction, screen );
                added.push_back( junction->m_Uuid );
            }
        }
    }

    pushEdit( junctions, schematic, _( "Labels to wires (API)" ), aCtx.kiway, added );

    if( pinPartition( schematic ) != before )
    {
        SCH_COMMIT undo( context->GetToolManager() );

        const std::set<KIID> ours( added.begin(), added.end() );

        for( SCH_ITEM* item : screen->Items() )
        {
            if( ours.count( item->m_Uuid ) )
                undo.Remove( item, screen );
        }

        for( SCH_LABEL_BASE* label : saved )
            undo.Add( label, screen );

        pushEdit( undo, schematic, _( "Labels to wires (API) undone" ) );
        return KOPENAPI_RESULT::Error( 500, "the wires would have changed the circuit; everything was put back "
                                            "(report this: the router let a wire touch another connection)" );
    }

    for( SCH_LABEL_BASE* label : saved )
        delete label;

    // part texts the new wires run through move clear of them
    SCH_COMMIT texts( context->GetToolManager() );
    bool       moved = false;

    for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );
        bool        hit = false;

        for( SCH_FIELD& field : symbol->GetFields() )
        {
            if( !field.IsVisible() || field.GetShownText( &*sheet, FOR_GUI ).IsEmpty() )
                continue;

            BOX2I box = field.GetBoundingBox();
            box.Inflate( schIUScale.mmToIU( 0.5 ) );

            for( const std::vector<VECTOR2I>& points : routes )
            {
                for( size_t i = 0; !hit && i + 1 < points.size(); ++i )
                {
                    SCH_LINE probe( points[i], LAYER_WIRE );
                    probe.SetEndPoint( points[i + 1] );
                    hit = box.GetWidth() > 0 && box.GetHeight() > 0 && probe.HitTest( box, false, 0 );
                }
            }
        }

        if( hit )
        {
            texts.Modify( symbol, screen );
            placeFieldsClear( symbol, screen, *sheet, false );
            moved = true;
        }
    }

    if( moved )
        pushEdit( texts, schematic, _( "Labels to wires (API)" ) );

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

    const bool fieldsClear = placeFieldsClear( symbol, sheet->LastScreen(), *sheet, true );

    SCH_COMMIT commit( context->GetToolManager() );
    commit.Add( symbol, sheet->LastScreen() );
    pushEdit( commit, schematic, _( "Add symbol (API)" ), aCtx.kiway, { symbol->m_Uuid } );

    // with the part on the sheet: neighbours' texts it covers move off, its own stay clear
    {
        SCH_COMMIT texts( context->GetToolManager() );
        settleTexts( symbol, sheet->LastScreen(), *sheet, texts );

        if( !texts.Empty() )
            pushEdit( texts, schematic, _( "Add symbol (API)" ) );
    }

    nlohmann::json card = symbolCard( symbol, *sheet );

    if( !fieldsClear )
        card["warnings"] = { "no clear spot for the reference / value texts nearby: they overlap something "
                             "(sch_layout_check; field_positions to place them)" };

    attachFootprintCheck( aCtx, aArgs, card );
    return KOPENAPI_RESULT::Ok( card );
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

    if( at->symbol->IsLocked() && !aArgs.value( "override_locks", false ) )
        return KOPENAPI_RESULT::Error( 409, "the symbol is locked (sch_unlock, or override_locks: true)" );

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
    nlohmann::json    warnings = nlohmann::json::array();

    const bool relocated = orientation >= 0 || aArgs.contains( "x_mm" ) || aArgs.contains( "y_mm" );

    if( relocated )
    {
        SHEET_DRAG drag( commit, screen, at->path );
        drag.SetAnchors( { at->symbol } );

        for( SCH_PIN* pin : at->symbol->GetPins( &at->path ) )
            drag.AddPointMove( before[pin->GetNumber()], pin->GetPosition(), at->symbol->IsPower() );

        warnings = drag.Apply( moved );

        if( std::optional<KOPENAPI_RESULT> refused = lockedDragged( screen, moved, { at->symbol }, aArgs ) )
        {
            commit.Revert();
            return *refused;
        }
    }

    // Texts re-placed clear of the neighbours after a move / turn, or when asked
    if( ( relocated && !at->symbol->IsPower() ) || aArgs.value( "autoplace_fields", false ) )
    {
        if( !placeFieldsClear( at->symbol, screen, at->path, aArgs.value( "autoplace_fields", false ) ) )
            warnings.push_back( "no clear spot for the reference / value texts nearby: they overlap something "
                                "(sch_layout_check; field_positions to place them)" );
    }

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

    pushEdit( commit, context->GetSchematic(), _( "Edit symbol (API)" ), aCtx.kiway, moved );

    nlohmann::json card = symbolCard( at->symbol, at->path );

    if( !warnings.empty() )
        card["warnings"] = warnings;

    attachFootprintCheck( aCtx, aArgs, card );
    return KOPENAPI_RESULT::Ok( card );
}


static KOPENAPI_RESULT h_sch_items_move( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "uuids" ) || !aArgs["uuids"].is_array() || aArgs["uuids"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'uuids' (array) and dx_mm / dy_mm" );

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    const VECTOR2I delta( toIU( aArgs.value( "dx_mm", 0.0 ) ), toIU( aArgs.value( "dy_mm", 0.0 ) ) );

    if( delta == VECTOR2I( 0, 0 ) )
        return KOPENAPI_RESULT::Error( 400, "dx_mm / dy_mm move by less than the 1.27 mm grid" );

    std::set<std::string> wanted;

    for( const nlohmann::json& u : aArgs["uuids"] )
    {
        if( u.is_string() )
            wanted.insert( u.get<std::string>() );
    }

    SCH_SCREEN*         screen = sheet->LastScreen();
    std::set<SCH_ITEM*> anchors;

    if( std::string error; !KopenapiSchExpandUuids( screenItems( screen ), wanted, error ) )
        return KOPENAPI_RESULT::Error( 400, error );

    for( SCH_ITEM* item : screen->Items() )
    {
        if( wanted.erase( str( item->m_Uuid.AsString() ) ) )
            anchors.insert( item );
    }

    if( !wanted.empty() )
    {
        nlohmann::json missing = nlohmann::json::array();

        for( const std::string& u : wanted )
            missing.push_back( u );

        return KOPENAPI_RESULT::Error( 404, "not on this sheet: " + missing.dump() );
    }

    if( !aArgs.value( "override_locks", false ) )
    {
        nlohmann::json locked = nlohmann::json::array();

        for( SCH_ITEM* item : anchors )
        {
            if( item->IsLocked() )
                locked.push_back( str( item->m_Uuid.AsString() ) );
        }

        if( !locked.empty() )
        {
            KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "locked items are in the request (sch_unlock, or override_locks: true)" );
            refused.body["locked"] = locked;
            return refused;
        }
    }

    SCH_COMMIT        commit( context->GetToolManager() );
    SHEET_DRAG        drag( commit, screen, *sheet );
    std::vector<KIID> touched;

    // Connection points of the moving items, before they move
    for( SCH_ITEM* item : anchors )
    {
        const bool power = item->Type() == SCH_SYMBOL_T && static_cast<SCH_SYMBOL*>( item )->IsPower();

        if( item->Type() == SCH_SYMBOL_T )
        {
            for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( item )->GetPins( &*sheet ) )
                drag.AddPointMove( pin->GetPosition(), pin->GetPosition() + delta, power );
        }
        else if( item->Type() == SCH_LINE_T )
        {
            SCH_LINE* line = static_cast<SCH_LINE*>( item );
            drag.AddPointMove( line->GetStartPoint(), line->GetStartPoint() + delta, false );
            drag.AddPointMove( line->GetEndPoint(), line->GetEndPoint() + delta, false );
        }
        else
        {
            drag.AddPointMove( item->GetPosition(), item->GetPosition() + delta, false );
        }

        commit.Modify( item, screen );
        item->Move( delta );
        touched.push_back( item->m_Uuid );
    }

    drag.SetAnchors( anchors );
    nlohmann::json warnings = drag.Apply( touched );

    if( std::optional<KOPENAPI_RESULT> refused = lockedDragged( screen, touched, anchors, aArgs ) )
    {
        commit.Revert();
        return *refused;
    }

    pushEdit( commit, context->GetSchematic(), _( "Move (API)" ), aCtx.kiway, touched );

    nlohmann::json result = { { "moved", anchors.size() }, { "touched", touched.size() } };

    if( !warnings.empty() )
        result["warnings"] = warnings;

    return KOPENAPI_RESULT::Ok( result );
}


namespace
{

/// @brief Paper sizes sch_page_set / sch_blocks_arrange pick from, smallest first (landscape, mm)
const std::vector<std::pair<PAGE_SIZE_TYPE, VECTOR2D>> ISO_PAPER = {
    { PAGE_SIZE_TYPE::A4, { 297, 210 } }, { PAGE_SIZE_TYPE::A3, { 420, 297 } }, { PAGE_SIZE_TYPE::A2, { 594, 420 } },
    { PAGE_SIZE_TYPE::A1, { 841, 594 } }, { PAGE_SIZE_TYPE::A0, { 1189, 841 } }
};

/// Drawing area of a page: inside the frame, above the title block strip
constexpr double PAGE_MARGIN_MM = 15.24;
constexpr double TITLE_STRIP_MM = 40.64;


VECTOR2D usableMm( const VECTOR2D& aPage )
{
    return { aPage.x - 2 * PAGE_MARGIN_MM, aPage.y - 2 * PAGE_MARGIN_MM - TITLE_STRIP_MM };
}


/// @brief The smallest ISO landscape paper whose drawing area holds aContent (mm); A0 if none does
std::pair<PAGE_SIZE_TYPE, VECTOR2D> paperFor( const VECTOR2D& aContent )
{
    for( const auto& paper : ISO_PAPER )
    {
        const VECTOR2D area = usableMm( paper.second );

        if( area.x >= aContent.x && area.y >= aContent.y )
            return paper;
    }

    return ISO_PAPER.back();
}


nlohmann::json pageCard( const PAGE_INFO& aPage )
{
    return { { "paper", str( aPage.GetTypeAsString() ) },
             { "portrait", aPage.IsPortrait() },
             { "width_mm", std::round( aPage.GetWidthMM() * 100 ) / 100 },
             { "height_mm", std::round( aPage.GetHeightMM() * 100 ) / 100 } };
}


/// @brief Set the page of aScreen; redraws the editor if one shows it
void applyPage( SCH_CONTEXT& aContext, SCH_SCREEN* aScreen, const PAGE_INFO& aPage )
{
    aScreen->SetPageSettings( aPage );
    aScreen->SetContentModified( true );

    TOOL_MANAGER* tools = aContext.GetToolManager();

    if( SCH_EDIT_FRAME* frame = tools ? dynamic_cast<SCH_EDIT_FRAME*>( tools->GetToolHolder() ) : nullptr )
    {
        if( frame->GetScreen() == aScreen )
            frame->HardRedraw();
    }
}


BOX2I sheetContent( SCH_SCREEN* aScreen )
{
    BOX2I box;
    bool  first = true;

    for( SCH_ITEM* item : aScreen->Items() )
    {
        if( first )
            box = item->GetBoundingBox();
        else
            box.Merge( item->GetBoundingBox() );

        first = false;
    }

    return box;
}


/// @brief A block of sch_blocks_arrange: its parts and everything attached to them
struct BLOCK
{
    std::string         name;
    std::set<SCH_ITEM*> items;
    BOX2I               box;
    VECTOR2I            delta;
};


/// @brief Points where aItem connects to wiring
std::vector<VECTOR2I> connectionPoints( SCH_ITEM* aItem, const SCH_SHEET_PATH& aPath )
{
    if( aItem->Type() == SCH_SYMBOL_T )
    {
        std::vector<VECTOR2I> points;

        for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( aItem )->GetPins( &aPath ) )
            points.push_back( pin->GetPosition() );

        return points;
    }

    if( aItem->Type() == SCH_LINE_T )
        return { static_cast<SCH_LINE*>( aItem )->GetStartPoint(), static_cast<SCH_LINE*>( aItem )->GetEndPoint() };

    return { aItem->GetPosition() };
}


/// @brief Wiring, labels, no-connects, junctions and power symbols reachable from aBlock's parts
/// without passing another non-power part; aOwners gets aBlock for each of them
void floodBlock( SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath, size_t aBlock, const std::set<SCH_ITEM*>& aParts,
                 std::map<SCH_ITEM*, std::set<size_t>>& aOwners )
{
    std::vector<VECTOR2I>         todo;
    std::set<std::pair<int, int>> seen;

    for( SCH_ITEM* part : aParts )
    {
        aOwners[part].insert( aBlock );

        for( const VECTOR2I& p : connectionPoints( part, aPath ) )
            todo.push_back( p );
    }

    auto take = [&]( SCH_ITEM* aItem )
    {
        if( !aOwners[aItem].insert( aBlock ).second )
            return;

        for( const VECTOR2I& p : connectionPoints( aItem, aPath ) )
            todo.push_back( p );

        // wires and pins ending on the middle of a wire join it
        if( aItem->Type() == SCH_LINE_T )
        {
            SCH_LINE* line = static_cast<SCH_LINE*>( aItem );

            for( SCH_ITEM* other : aScreen->Items().Overlapping( line->GetBoundingBox() ) )
            {
                for( const VECTOR2I& p : connectionPoints( other, aPath ) )
                {
                    if( line->HitTest( p, 0 ) )
                        todo.push_back( p );
                }
            }
        }
    };

    while( !todo.empty() )
    {
        const VECTOR2I p = todo.back();
        todo.pop_back();

        if( !seen.insert( { p.x, p.y } ).second )
            continue;

        for( SCH_ITEM* item : aScreen->Items().Overlapping( p ) )
        {
            switch( item->Type() )
            {
            case SCH_LINE_T:
                if( static_cast<SCH_LINE*>( item )->IsWire() && item->HitTest( p, 0 ) )
                    take( item );

                break;

            case SCH_SYMBOL_T:
            {
                SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

                if( !symbol->IsPower() || aParts.count( item ) )
                    break;

                for( SCH_PIN* pin : symbol->GetPins( &aPath ) )
                {
                    if( pin->GetPosition() == p )
                    {
                        take( item );
                        break;
                    }
                }

                break;
            }

            case SCH_JUNCTION_T:
            case SCH_NO_CONNECT_T:
                if( item->GetPosition() == p )
                    take( item );

                break;

            default:
                if( labelKind( item->Type() ) && item->GetPosition() == p )
                    aOwners[item].insert( aBlock );   // a label names the net, nothing beyond it is wiring

                break;
            }
        }
    }
}

} // namespace


static KOPENAPI_RESULT h_sch_page_set( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    SCH_SCREEN*       screen = sheet->LastScreen();
    PAGE_INFO         page = screen->GetPageSettings();
    const std::string paper = aArgs.value( "paper", std::string() );
    const bool        portrait = aArgs.value( "portrait", page.IsPortrait() );

    if( paper == "auto" )
    {
        const BOX2I    box = sheetContent( screen );
        const VECTOR2D content( schIUScale.IUTomm( box.GetWidth() ), schIUScale.IUTomm( box.GetHeight() ) );
        page.SetType( paperFor( content ).first, false );
    }
    else if( paper == "User" )
    {
        if( !aArgs.contains( "width_mm" ) || !aArgs.contains( "height_mm" ) )
            return KOPENAPI_RESULT::Error( 400, "paper User needs width_mm and height_mm" );

        PAGE_INFO::SetCustomWidthMils( aArgs["width_mm"].get<double>() * 1000 / 25.4 );
        PAGE_INFO::SetCustomHeightMils( aArgs["height_mm"].get<double>() * 1000 / 25.4 );
        page.SetType( PAGE_SIZE_TYPE::User );
    }
    else if( !paper.empty() )
    {
        if( !page.SetType( wxString::FromUTF8( paper ), portrait ) )
            return KOPENAPI_RESULT::Error( 400, "unknown paper '" + paper + "' (A5..A0, A..E, USLetter, USLegal, "
                                                "USLedger, User, auto)" );
    }

    if( paper != "auto" && paper != "User" )
        page.SetPortrait( portrait );

    nlohmann::json result = { { "before", pageCard( screen->GetPageSettings() ) } };
    applyPage( *context, screen, page );
    result["page"] = pageCard( page );

    const BOX2I content = sheetContent( screen );
    const BOX2I sheetBox( VECTOR2I( 0, 0 ), VECTOR2I( schIUScale.mmToIU( page.GetWidthMM() ),
                                                       schIUScale.mmToIU( page.GetHeightMM() ) ) );

    if( !sheetBox.Contains( content ) )
        result["warnings"] = { "the drawing reaches beyond the new page (sch_blocks_arrange packs it)" };

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_sch_blocks_arrange( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    if( !aArgs.contains( "blocks" ) || !aArgs["blocks"].is_array() || aArgs["blocks"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'blocks': [{name, refs: [...]}]" );

    std::optional<SCH_SHEET_PATH> sheet = targetSheet( *context, aArgs );

    if( !sheet )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    SCH_SCREEN* screen = sheet->LastScreen();

    std::map<std::string, SCH_SYMBOL*> byRef;

    for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

        if( !symbol->IsPower() )
            byRef[str( symbol->GetRef( &*sheet ) )] = symbol;
    }

    // Blocks and their parts
    std::vector<BLOCK>                    blocks;
    std::vector<std::set<SCH_ITEM*>>      parts;
    std::map<std::string, size_t>         blockOf;
    nlohmann::json                        unknown = nlohmann::json::array();

    for( const nlohmann::json& b : aArgs["blocks"] )
    {
        const std::string name = b.value( "name", "block" + std::to_string( blocks.size() + 1 ) );

        if( blockOf.count( name ) )
            return KOPENAPI_RESULT::Error( 400, "block name '" + name + "' twice" );

        blockOf[name] = blocks.size();
        blocks.push_back( { name, {}, {}, {} } );
        parts.emplace_back();

        for( const nlohmann::json& r : b.value( "refs", nlohmann::json::array() ) )
        {
            auto it = byRef.find( r.get<std::string>() );

            if( it == byRef.end() )
                unknown.push_back( r );
            else if( !parts.back().insert( it->second ).second )
                unknown.push_back( r );
        }
    }

    if( !unknown.empty() )
        return KOPENAPI_RESULT::Error( 400, "not a part on this sheet (or listed twice): " + unknown.dump() );

    std::map<SCH_ITEM*, std::set<size_t>> owners;
    nlohmann::json                        unassigned = nlohmann::json::array();

    for( const auto& [ref, symbol] : byRef )
    {
        size_t in = 0;

        for( const std::set<SCH_ITEM*>& p : parts )
            in += p.count( symbol );

        if( in > 1 )
            return KOPENAPI_RESULT::Error( 400, ref + " is in more than one block" );

        if( !in )
            unassigned.push_back( ref );
    }

    if( !unassigned.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 400, "every part of the sheet goes into a block" );
        refused.body["unassigned"] = unassigned;
        return refused;
    }

    for( size_t i = 0; i < blocks.size(); ++i )
        floodBlock( screen, *sheet, i, parts[i], owners );

    std::set<SCH_ITEM*> shared;

    for( const auto& [item, who] : owners )
    {
        if( who.size() == 1 )
            blocks[*who.begin()].items.insert( item );
        else
            shared.insert( item );
    }

    for( BLOCK& block : blocks )
    {
        bool first = true;

        for( SCH_ITEM* item : block.items )
        {
            if( first )
                block.box = item->GetBoundingBox();
            else
                block.box.Merge( item->GetBoundingBox() );

            first = false;
        }
    }

    // Wiring tied to no part (a PWR_FLAG on a power symbol, a stub with a label) goes with the nearest block
    for( SCH_ITEM* item : screen->Items() )
    {
        const bool wiring = item->Type() == SCH_LINE_T || item->Type() == SCH_JUNCTION_T
                            || item->Type() == SCH_NO_CONNECT_T || labelKind( item->Type() )
                            || ( item->Type() == SCH_SYMBOL_T && static_cast<SCH_SYMBOL*>( item )->IsPower() );

        if( !wiring || owners.count( item ) )
            continue;

        std::map<SCH_ITEM*, std::set<size_t>> cluster;
        floodBlock( screen, *sheet, 0, { item }, cluster );

        BOX2I box = item->GetBoundingBox();

        for( const auto& [member, who] : cluster )
            box.Merge( member->GetBoundingBox() );

        BLOCK* home = &blocks.front();

        for( BLOCK& block : blocks )
        {
            if( block.box.Distance( box ) < home->box.Distance( box ) )
                home = &block;
        }

        for( const auto& [member, who] : cluster )
        {
            if( !owners.count( member ) )
            {
                owners[member] = { size_t( home - blocks.data() ) };
                home->items.insert( member );
            }
        }
    }

    // Texts and graphics nobody owns go with the block they lie in
    nlohmann::json stayed = nlohmann::json::array();

    for( SCH_ITEM* item : screen->Items() )
    {
        if( owners.count( item ) )
            continue;

        const VECTOR2I centre = item->GetBoundingBox().GetCenter();
        BLOCK*         home = nullptr;

        for( BLOCK& block : blocks )
        {
            if( block.box.Contains( centre ) )
                home = &block;
        }

        if( home )
            home->items.insert( item );
        else
            stayed.push_back( { { "uuid", str( item->m_Uuid.AsString() ) }, { "type", str( item->GetClass() ) } } );
    }

    // Rows: given, or shelves about as wide as the drawing is tall
    const int gap = toIU( aArgs.value( "gap_mm", 12.7 ) );
    std::vector<std::vector<size_t>> rows;

    if( aArgs.contains( "rows" ) )
    {
        std::set<size_t> placed;

        for( const nlohmann::json& row : aArgs["rows"] )
        {
            rows.emplace_back();

            for( const nlohmann::json& n : row )
            {
                auto it = blockOf.find( n.get<std::string>() );

                if( it == blockOf.end() || !placed.insert( it->second ).second )
                    return KOPENAPI_RESULT::Error( 400, "rows: unknown block or placed twice: " + n.dump() );

                rows.back().push_back( it->second );
            }
        }

        if( placed.size() != blocks.size() )
            return KOPENAPI_RESULT::Error( 400, "rows must place every block" );
    }
    else
    {
        double area = 0;
        int    widest = 0;

        for( const BLOCK& block : blocks )
        {
            area += double( block.box.GetWidth() + gap ) * ( block.box.GetHeight() + gap );
            widest = std::max<int>( widest, block.box.GetWidth() );
        }

        const int width = std::max<int>( widest, std::sqrt( area * 1.414 ) );
        int       x = 0;

        for( size_t i = 0; i < blocks.size(); ++i )
        {
            if( rows.empty() || ( x > 0 && x + blocks[i].box.GetWidth() > width ) )
            {
                rows.emplace_back();
                x = 0;
            }

            rows.back().push_back( i );
            x += blocks[i].box.GetWidth() + gap;
        }
    }

    // Shelf layout from (0, 0): blocks top-aligned in a row, the rows centred on the widest one
    std::vector<VECTOR2I> at( blocks.size() );
    int                   y = 0, contentW = 0;
    std::vector<int>      rowW;

    for( const std::vector<size_t>& row : rows )
    {
        int w = 0;

        for( size_t i : row )
            w += blocks[i].box.GetWidth() + ( w ? gap : 0 );

        rowW.push_back( w );
        contentW = std::max( contentW, w );
    }

    for( size_t r = 0; r < rows.size(); ++r )
    {
        int x = aArgs.value( "align", std::string( "center" ) ) == "left" ? 0 : ( contentW - rowW[r] ) / 2;
        int h = 0;

        for( size_t i : rows[r] )
        {
            at[i] = { x, y };
            x += blocks[i].box.GetWidth() + gap;
            h = std::max<int>( h, blocks[i].box.GetHeight() );
        }

        y += h + gap;
    }

    const VECTOR2D content( schIUScale.IUTomm( contentW ), schIUScale.IUTomm( std::max( 0, y - gap ) ) );
    const std::string paper = aArgs.value( "paper", std::string( "auto" ) );
    PAGE_INFO         page = screen->GetPageSettings();
    nlohmann::json    warnings = nlohmann::json::array();

    if( paper == "auto" )
        page.SetType( paperFor( content ).first, false );
    else if( paper != "keep" && !page.SetType( wxString::FromUTF8( paper ), false ) )
        return KOPENAPI_RESULT::Error( 400, "unknown paper '" + paper + "' (auto, keep, A4..A0, ...)" );

    const VECTOR2D area = usableMm( { page.GetWidthMM(), page.GetHeightMM() } );

    if( area.x < content.x || area.y < content.y )
        warnings.push_back( "the blocks do not fit the drawing area of " + str( page.GetTypeAsString() ) );

    // Centre the drawing in the area above the title block
    const VECTOR2I origin( toIU( PAGE_MARGIN_MM + std::max( 0.0, area.x - content.x ) / 2 ),
                           toIU( PAGE_MARGIN_MM + std::max( 0.0, area.y - content.y ) / 2 ) );

    for( size_t i = 0; i < blocks.size(); ++i )
    {
        const VECTOR2I d = origin + at[i] - blocks[i].box.GetOrigin();
        blocks[i].delta = { toIU( schIUScale.IUTomm( d.x ) ), toIU( schIUScale.IUTomm( d.y ) ) };
    }

    SCH_COMMIT          commit( context->GetToolManager() );
    SHEET_DRAG          drag( commit, screen, *sheet );
    std::set<SCH_ITEM*> moved;
    std::vector<KIID>   touched;

    // Wiring between blocks: its ends on a block go with the block, the rest stretches
    for( SCH_ITEM* item : shared )
    {
        for( const VECTOR2I& p : connectionPoints( item, *sheet ) )
        {
            for( const BLOCK& block : blocks )
            {
                for( SCH_ITEM* other : screen->Items().Overlapping( p ) )
                {
                    if( !block.items.count( other ) )
                        continue;

                    for( const VECTOR2I& q : connectionPoints( other, *sheet ) )
                    {
                        if( q == p )
                            drag.AddPointMove( p, p + block.delta, false );
                    }
                }
            }
        }
    }

    for( const BLOCK& block : blocks )
    {
        for( SCH_ITEM* item : block.items )
        {
            commit.Modify( item, screen );
            item->Move( block.delta );
            moved.insert( item );
            touched.push_back( item->m_Uuid );
        }
    }

    drag.SetAnchors( moved );

    for( const nlohmann::json& w : drag.Apply( touched ) )
        warnings.push_back( w );

    if( std::optional<KOPENAPI_RESULT> refused = lockedDragged( screen, touched, moved, aArgs ) )
    {
        commit.Revert();
        return *refused;
    }

    pushEdit( commit, context->GetSchematic(), _( "Arrange blocks (API)" ), aCtx.kiway, {} );

    if( paper != "keep" )
        applyPage( *context, screen, page );

    nlohmann::json out = nlohmann::json::array();

    for( size_t i = 0; i < blocks.size(); ++i )
    {
        const BOX2I box( blocks[i].box.GetOrigin() + blocks[i].delta, blocks[i].box.GetSize() );
        out.push_back( { { "name", blocks[i].name },
                         { "parts", parts[i].size() },
                         { "items", blocks[i].items.size() },
                         { "dx_mm", toMm( blocks[i].delta.x ) },
                         { "dy_mm", toMm( blocks[i].delta.y ) },
                         { "box_mm", { toMm( box.GetLeft() ), toMm( box.GetTop() ), toMm( box.GetRight() ),
                                       toMm( box.GetBottom() ) } } } );
    }

    nlohmann::json result = { { "blocks", out },
                              { "shared_wiring", shared.size() },
                              { "page", pageCard( page ) },
                              { "content_mm", { content.x, content.y } } };

    if( !stayed.empty() )
        result["not_moved"] = stayed;

    if( !warnings.empty() )
        result["warnings"] = warnings;

    return KOPENAPI_RESULT::Ok( result );
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
    nlohmann::json lockedKept = nlohmann::json::array();
    const bool     overrideLocks = aArgs.value( "override_locks", false );
    SCH_SCREENS    screens( context->GetSchematic()->Root() );

    {
        std::vector<SCH_ITEM*> all;

        for( SCH_SCREEN* screen = screens.GetFirst(); screen; screen = screens.GetNext() )
        {
            for( SCH_ITEM* item : screen->Items() )
                all.push_back( item );
        }

        if( std::string error; !KopenapiSchExpandUuids( all, wanted, error ) )
            return KOPENAPI_RESULT::Error( 400, error );
    }

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
            wanted.erase( str( item->m_Uuid.AsString() ) );

            if( item->IsLocked() && !overrideLocks )
            {
                lockedKept.push_back( str( item->m_Uuid.AsString() ) );
                continue;
            }

            deleted.push_back( str( item->m_Uuid.AsString() ) );
            commit.Remove( item, screen );
        }
    }

    if( deleted.empty() && !lockedKept.empty() )
    {
        KOPENAPI_RESULT refused = KOPENAPI_RESULT::Error( 409, "the items are locked (sch_unlock, or override_locks: true)" );
        refused.body["locked"] = lockedKept;
        return refused;
    }

    if( deleted.empty() )
        return KOPENAPI_RESULT::Error( 404, "none of the uuids found" );

    pushEdit( commit, context->GetSchematic(), _( "Delete (API)" ) );

    nlohmann::json missing = nlohmann::json::array();

    for( const std::string& u : wanted )
        missing.push_back( u );

    return KOPENAPI_RESULT::Ok( { { "deleted", deleted }, { "not_found", missing }, { "locked_kept", lockedKept } } );
}


/// @brief sch_lock / sch_unlock: the lock flag of symbols (every unit and instance of a reference) and of
/// items by uuid on any sheet
static KOPENAPI_RESULT schSetLocked( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs, bool aLock )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    std::set<std::string> refs, uuids;

    for( const nlohmann::json& r : aArgs.value( "refs", nlohmann::json::array() ) )
    {
        if( r.is_string() )
            refs.insert( r.get<std::string>() );
    }

    for( const nlohmann::json& u : aArgs.value( "uuids", nlohmann::json::array() ) )
    {
        if( u.is_string() )
            uuids.insert( u.get<std::string>() );
    }

    if( refs.empty() && uuids.empty() )
        return KOPENAPI_RESULT::Error( 400, "say what: refs (symbols by reference, all their units) or uuids (sch_item_list, sch_symbol_list)" );

    SCHEMATIC*            schematic = context->GetSchematic();

    if( !uuids.empty() )
    {
        std::vector<SCH_ITEM*> all;

        for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
        {
            for( SCH_ITEM* item : path.LastScreen()->Items() )
                all.push_back( item );
        }

        if( std::string error; !KopenapiSchExpandUuids( all, uuids, error ) )
            return KOPENAPI_RESULT::Error( 400, error );
    }
    SCH_COMMIT            commit( context->GetToolManager() );
    std::set<SCH_ITEM*>   seen;
    std::set<std::string> foundRefs, foundUuids;
    int                   changed = 0, already = 0;
    std::vector<KIID>     touched;

    for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
    {
        SCH_SCREEN* screen = path.LastScreen();

        for( SCH_ITEM* item : screen->Items() )
        {
            bool wanted = uuids.count( str( item->m_Uuid.AsString() ) ) > 0;

            if( wanted )
                foundUuids.insert( str( item->m_Uuid.AsString() ) );

            if( item->Type() == SCH_SYMBOL_T )
            {
                const std::string ref = str( static_cast<SCH_SYMBOL*>( item )->GetRef( &path, false ) );

                if( refs.count( ref ) )
                {
                    wanted = true;
                    foundRefs.insert( ref );
                }
            }

            if( !wanted || !seen.insert( item ).second )
                continue;

            if( item->IsLocked() == aLock )
            {
                already++;
                continue;
            }

            commit.Modify( item, screen );
            item->SetLocked( aLock );
            touched.push_back( item->m_Uuid );
            changed++;
        }
    }

    nlohmann::json missing = nlohmann::json::array();

    for( const std::string& r : refs )
    {
        if( !foundRefs.count( r ) )
            missing.push_back( r );
    }

    for( const std::string& u : uuids )
    {
        if( !foundUuids.count( u ) )
            missing.push_back( u );
    }

    if( changed )
        pushEdit( commit, schematic, aLock ? _( "Lock (API)" ) : _( "Unlock (API)" ), aCtx.kiway, touched );

    return KOPENAPI_RESULT::Ok( { { aLock ? "locked" : "unlocked", changed }, { "already", already }, { "missing", missing },
                                  { "applied", changed > 0 } } );
}


static KOPENAPI_RESULT h_sch_lock( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return schSetLocked( aCtx, aArgs, true );
}


static KOPENAPI_RESULT h_sch_unlock( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    return schSetLocked( aCtx, aArgs, false );
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
        return makePowerSymbol( powerSymbol, schematic, aPath, aPoint, aOut, takenRefs, wxString::FromUTF8( net ) );
    };

    const bool upright = aArgs.value( "orientation", std::string( "upright" ) ) != "away";

    // Where the power symbol's body points in its library pose (e.g. GND down, +5V up)
    auto naturalDirection = [&]( const SCH_SHEET_PATH& aPath ) -> VECTOR2I
    {
        return powerSymbol ? powerNaturalDirection( powerSymbol, schematic, aPath ) : VECTOR2I( 0, 0 );
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

        // Upright: the symbol keeps its library pose (GND down, +V up).  On a pin facing that
        // way it sits on the pin; on a sideways pin a short stub leads out first; a pin facing
        // the opposite way gets the symbol turned away from the part.
        const VECTOR2I natural = naturalDirection( first.path );
        const bool     sideways = upright && natural != out && natural != VECTOR2I( -out.x, -out.y );
        const VECTOR2I body = upright && natural == out ? natural : upright && sideways ? natural : out;

        if( group.size() == 1 )
        {
            VECTOR2I at = first.pin->GetPosition();

            if( sideways )
            {
                const VECTOR2I end = at + VECTOR2I( out.x * toIU( 2 * GRID_MM ), out.y * toIU( 2 * GRID_MM ) );
                addWire( at, end, screen );
                at = end;
            }

            SCH_ITEM* symbol = placePower( at, body, first.path );
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

        // The symbol at the row end lying furthest in its body direction (the row runs along
        // it when the pins face sideways), else at the first stub
        VECTOR2I at = ends.front();

        if( sideways )
        {
            for( const VECTOR2I& end : ends )
            {
                if( ( end.x - at.x ) * natural.x + ( end.y - at.y ) * natural.y > 0 )
                    at = end;
            }
        }

        SCH_ITEM* symbol = placePower( at, body, first.path );
        commit.Add( symbol, screen );
        extra.push_back( symbol->m_Uuid );
    }

    std::vector<KIID> added;

    for( const nlohmann::json& p : placed )
        added.emplace_back( wxString::FromUTF8( p["uuid"].get<std::string>() ) );

    added.insert( added.end(), extra.begin(), extra.end() );
    pushEdit( commit, schematic, _( "Connect pins (API)" ), aCtx.kiway, added );

    // The parts' texts out of the way of what was just put next to their pins
    {
        SCH_COMMIT      texts( context->GetToolManager() );
        std::set<SCH_SYMBOL*> seen;

        for( const PIN_AT& at : pins )
        {
            if( !seen.insert( at.symbol ).second || at.symbol->IsPower() )
                continue;

            texts.Modify( at.symbol, at.path.LastScreen() );
            placeFieldsClear( at.symbol, at.path.LastScreen(), at.path, false );
        }

        if( !texts.Empty() )
            pushEdit( texts, schematic, _( "Connect pins (API)" ) );
    }

    // Confirm: every pin is now on one net
    std::set<std::string> nets;

    for( const nlohmann::json& p : placed )
        nets.insert( netOfPin( schematic, p["pin"] ) );

    return KOPENAPI_RESULT::Ok( { { "net", nets.size() == 1 ? nlohmann::json( *nets.begin() ) : nlohmann::json() },
                                  { "connected", nets.size() == 1 && !nets.begin()->empty() },
                                  { "nets_seen", nets },
                                  { "placed", placed } } );
}


static KOPENAPI_RESULT h_sch_power_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string id = aArgs.value( "lib_id", std::string() );

    if( id.empty() || !aArgs.contains( "at" ) )
        return KOPENAPI_RESULT::Error( 400, "give 'lib_id' (e.g. power:GND, power:PWR_FLAG) and 'at' (REF.PIN or {x_mm, y_mm})" );

    SCHEMATIC*  schematic = context->GetSchematic();
    std::string error;
    LIB_SYMBOL* lib = librarySymbol( aCtx, *context, id, error );

    if( !lib || !lib->IsPower() )
        return KOPENAPI_RESULT::Error( 404, lib ? "not a power symbol: " + id : error );

    std::optional<SCH_SHEET_PATH> sheet;

    if( !aArgs["at"].is_string() )
    {
        sheet = targetSheet( *context, aArgs );

        if( !sheet )
            return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );
    }

    VECTOR2I                out;
    std::optional<VECTOR2I> point = wireEnd( schematic, aArgs["at"], sheet, out, error );

    if( !point )
        return KOPENAPI_RESULT::Error( 404, error );

    SCH_SCREEN*    screen = sheet->LastScreen();
    const VECTOR2I natural = powerNaturalDirection( lib, schematic, *sheet );
    const int      grid = toIU( GRID_MM );

    // Something else already sits on the point (a power symbol, a label): step aside with a stub
    bool occupied = false;
    bool onWire = out != VECTOR2I( 0, 0 );

    for( SCH_ITEM* item : screen->Items().Overlapping( *point ) )
    {
        if( labelKind( item->Type() ) && item->GetPosition() == *point )
            occupied = true;
        else if( item->Type() == SCH_SYMBOL_T && static_cast<SCH_SYMBOL*>( item )->IsPower() )
        {
            for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( item )->GetPins( &*sheet ) )
                occupied |= pin->GetPosition() == *point;
        }
        else if( item->Type() == SCH_LINE_T && static_cast<SCH_LINE*>( item )->IsWire() && item->HitTest( *point, 0 ) )
            onWire = true;
    }

    if( !onWire )
        return KOPENAPI_RESULT::Error( 422, "nothing to attach to at that point: give a pin (REF.PIN) or a point on a wire" );

    const std::string stub = aArgs.value( "stub", std::string( "auto" ) );
    static const std::map<std::string, VECTOR2I> named = {
        { "left", { -1, 0 } }, { "right", { 1, 0 } }, { "up", { 0, -1 } }, { "down", { 0, 1 } } };

    std::vector<VECTOR2I> directions;

    if( stub == "none" )
    {
        if( occupied )
            return KOPENAPI_RESULT::Error( 422, "another power symbol or label sits on that point: use a stub" );
    }
    else if( named.count( stub ) )
    {
        directions.push_back( named.at( stub ) );
    }
    else if( stub == "auto" )
    {
        // A sideways pin: out first (the conventional short stub); a crowded point: sideways of
        // the body's direction, then against it
        const bool sideways = out != VECTOR2I( 0, 0 ) && out != natural;

        if( occupied || sideways )
        {
            if( out != VECTOR2I( 0, 0 ) && out != VECTOR2I( -natural.x, -natural.y ) )
                directions.push_back( out );

            for( const VECTOR2I& d : { VECTOR2I( natural.y, natural.x ), VECTOR2I( -natural.y, -natural.x ),
                                       VECTOR2I( -natural.x, -natural.y ) } )
            {
                if( std::find( directions.begin(), directions.end(), d ) == directions.end() && d != out * -1 )
                    directions.push_back( d );
            }
        }
    }
    else
    {
        return KOPENAPI_RESULT::Error( 400, "stub: auto, none, left, right, up or down" );
    }

    std::set<wxString> taken;
    VECTOR2I           end = *point;   // where the symbol's pin goes
    VECTOR2I           tap = *point;   // where its stub (if any) starts
    std::string        why;

    VECTOR2I body = natural;   // where the symbol's body points

    // The symbol there lies on no other part, symbol, text, label or foreign wire
    auto spotClear = [&]( const VECTOR2I& aAt, const VECTOR2I& aBody, const VECTOR2I& aTap )
    {
        std::unique_ptr<SCH_SYMBOL> probe( makePowerSymbol( lib, schematic, *sheet, aAt, aBody, taken ) );
        taken.clear();
        const BOX2I box = probe->GetBoundingBox();
        const BOX2I shape = probe->GetBodyBoundingBox();

        for( SCH_ITEM* item : screen->Items() )
        {
            // other parts: their body and each visible text (a part's pins reach out to the
            // point the symbol goes on, so not the whole bounding box)
            if( item->Type() == SCH_SYMBOL_T )
            {
                SCH_SYMBOL* other = static_cast<SCH_SYMBOL*>( item );

                if( other->GetBodyBoundingBox().Intersects( box ) )
                    return false;

                for( SCH_FIELD& field : other->GetFields() )
                {
                    if( field.IsVisible() && !field.GetShownText( &*sheet, FOR_GUI ).IsEmpty() && field.GetBoundingBox().Intersects( box ) )
                        return false;
                }
            }

            if( ( labelKind( item->Type() ) || item->Type() == SCH_NO_CONNECT_T ) && item->GetBoundingBox().Intersects( box ) )
                return false;

            if( item->Type() == SCH_LINE_T )
            {
                SCH_LINE* line = static_cast<SCH_LINE*>( item );

                if( line->HitTest( aTap, 0 ) || line->HitTest( aAt, 0 ) )
                    continue;   // the wire it hangs on

                if( line->HitTest( shape, false, 0 ) )
                    return false;
            }
        }

        return true;
    };

    // A free connection point: nothing but the given wire meets there
    auto freePoint = [&]( const VECTOR2I& aAt, const SCH_ITEM* aWire )
    {
        for( SCH_ITEM* item : screen->Items().Overlapping( aAt ) )
        {
            if( item == aWire )
                continue;

            if( item->Type() == SCH_SYMBOL_T )
            {
                for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( item )->GetPins( &*sheet ) )
                {
                    if( pin->GetPosition() == aAt )
                        return false;
                }
            }
            else if( item->Type() == SCH_LINE_T )
            {
                if( item->HitTest( aAt, 0 ) )
                    return false;
            }
            else if( item->GetPosition() == aAt )
            {
                return false;
            }
        }

        return true;
    };

    // Upright first, then upside down (PWR_FLAG below a rail is as common)
    const std::vector<VECTOR2I> bodies = { natural, VECTOR2I( -natural.x, -natural.y ) };
    // Right on the point, but its body or texts would lie on something: step aside like a taken point
    if( directions.empty() && stub == "auto" && !spotClear( *point, natural, *point ) )
    {
        if( out != VECTOR2I( 0, 0 ) && out != VECTOR2I( -natural.x, -natural.y ) )
            directions.push_back( out );

        for( const VECTOR2I& d : { VECTOR2I( natural.y, natural.x ), VECTOR2I( -natural.y, -natural.x ),
                                   VECTOR2I( -natural.x, -natural.y ) } )
        {
            if( std::find( directions.begin(), directions.end(), d ) == directions.end() && d != out * -1 )
                directions.push_back( d );
        }
    }

    bool                        found = directions.empty();
    SCH_LINE*                   tapped = nullptr;   // the wire the symbol hangs on (split there)

    // First choice when the point is taken, sideways or crowded: hang the symbol on the wiring
    // of this net (a T with a junction) - wires reached from the point through wire ends and
    // junctions, nearest first - on a wire across the symbol's direction
    if( !found && stub == "auto" )
    {
        struct TAP
        {
            int       distance;
            VECTOR2I  at;
            SCH_LINE* wire;
        };

        std::vector<TAP>              taps;
        std::vector<std::pair<VECTOR2I, int>> todo = { { *point, 0 } };
        std::set<SCH_LINE*>           walked;
        const int                     reach = 40 * grid;

        while( !todo.empty() )
        {
            const auto [from, distance] = todo.back();
            todo.pop_back();

            for( SCH_ITEM* item : screen->Items().Overlapping( SCH_LINE_T, from ) )
            {
                SCH_LINE* wire = static_cast<SCH_LINE*>( item );

                if( !wire->IsWire() || walked.count( wire ) || ( wire->GetStartPoint() != from && wire->GetEndPoint() != from ) )
                    continue;

                walked.insert( wire );
                const VECTOR2I other = wire->GetStartPoint() == from ? wire->GetEndPoint() : wire->GetStartPoint();
                const VECTOR2I along = other - from;
                const int      length = std::abs( along.x ) + std::abs( along.y );
                const VECTOR2I step( along.x == 0 ? 0 : ( along.x > 0 ? grid : -grid ), along.y == 0 ? 0 : ( along.y > 0 ? grid : -grid ) );

                for( int k = 1; k * grid < length; ++k )
                    taps.push_back( { distance + k * grid, from + step * k, wire } );

                // go on through a plain wire joint (not into another part's pin)
                if( distance + length < reach )
                {
                    bool pin = false;

                    for( SCH_ITEM* o : screen->Items().Overlapping( SCH_SYMBOL_T, other ) )
                    {
                        for( SCH_PIN* p : static_cast<SCH_SYMBOL*>( o )->GetPins( &*sheet ) )
                            pin |= p->GetPosition() == other && !static_cast<SCH_SYMBOL*>( o )->IsPower();
                    }

                    if( !pin )
                        todo.push_back( { other, distance + length } );
                }
            }
        }

        std::stable_sort( taps.begin(), taps.end(), []( const TAP& a, const TAP& b ) { return a.distance < b.distance; } );

        for( const VECTOR2I& b : bodies )
        {
            for( const TAP& t : taps )
            {
                const VECTOR2I along = t.wire->GetEndPoint() - t.wire->GetStartPoint();

                if( found || ( along.x != 0 ) == ( b.x != 0 ) )
                    continue;   // the wire runs the way the symbol points

                if( freePoint( t.at, t.wire ) && spotClear( t.at, b, t.at ) )
                {
                    end = tap = t.at;
                    body = b;
                    tapped = t.wire;
                    found = true;
                }
            }

            if( found )
                break;
        }
    }

    if( !found )
    {
        if( stub == "auto" && std::find( directions.begin(), directions.end(), natural ) == directions.end() )
            directions.push_back( natural );

        WIRE_ROUTER router( screen, *sheet, grid );

        // shortest stub first; at each length every direction, upright before upside down
        for( int steps : { 2, 4, 6, 8 } )
        {
            for( const VECTOR2I& b : bodies )
            {
                for( const VECTOR2I& d : directions )
                {
                    if( found || d == VECTOR2I( -b.x, -b.y ) )
                        continue;   // a body pointing back along its own stub

                    const VECTOR2I candidate = *point + d * ( steps * grid );
                    std::optional<std::vector<VECTOR2I>> path = router.Route( *point, candidate, "hv", &why );

                    if( path && path->size() == 2 && freePoint( candidate, nullptr ) && spotClear( candidate, b, *point ) )
                    {
                        end = candidate;
                        body = b;
                        found = true;
                    }
                }
            }

            if( found )
                break;
        }

        if( !found )
            return KOPENAPI_RESULT::Error( 422, "no free spot for a stub and the symbol near that point"
                                                + ( why.empty() ? std::string() : " (" + why + ")" ) + "; give stub explicitly or move parts" );
    }

    SCH_COMMIT        commit( context->GetToolManager() );
    std::vector<KIID> touched;

    if( tapped )
    {
        // wires connect at their ends: split the one hung on
        commit.Remove( tapped, screen );

        for( const VECTOR2I& distant : { tapped->GetStartPoint(), tapped->GetEndPoint() } )
        {
            auto* half = new SCH_LINE( distant, LAYER_WIRE );
            half->SetEndPoint( tap );
            commit.Add( half, screen );
            touched.push_back( half->m_Uuid );
        }
    }

    if( end != tap )
    {
        auto* wire = new SCH_LINE( tap, LAYER_WIRE );
        wire->SetEndPoint( end );
        commit.Add( wire, screen );
        touched.push_back( wire->m_Uuid );
    }

    SCH_SYMBOL* symbol = makePowerSymbol( lib, schematic, *sheet, end, body, taken );
    commit.Add( symbol, screen );
    touched.push_back( symbol->m_Uuid );
    pushEdit( commit, schematic, _( "Power symbol (API)" ), aCtx.kiway, touched );

    {
        SCH_COMMIT texts( context->GetToolManager() );
        settleTexts( symbol, screen, *sheet, texts );

        if( !texts.Empty() )
            pushEdit( texts, schematic, _( "Power symbol (API)" ) );
    }

    // A T onto a wire, or a stub from a point other wiring meets: a junction dot
    if( screen->IsExplicitJunctionNeeded( tap ) )
    {
        SCH_COMMIT dots( context->GetToolManager() );
        auto*      junction = new SCH_JUNCTION( tap );
        dots.Add( junction, screen );
        pushEdit( dots, schematic, _( "Power symbol (API)" ), aCtx.kiway, { junction->m_Uuid } );
    }

    std::string net;

    for( const NET_ENTRY& entry : collectNets( schematic ) )
    {
        for( const NET_INSTANCE& inst : entry.instances )
        {
            for( SCH_PIN* pin : symbol->GetPins( &*sheet ) )
            {
                if( inst.path == *sheet && std::find( inst.items.begin(), inst.items.end(), pin ) != inst.items.end() )
                    net = entry.name;
            }
        }
    }

    nlohmann::json result = { { "uuid", str( symbol->m_Uuid.AsString() ) },
                              { "ref", str( symbol->GetRef( &*sheet, false ) ) },
                              { "at", { toMm( end.x ), toMm( end.y ) } },
                              { "net", net } };

    auto dirName = []( const VECTOR2I& d )
    {
        return d.x > 0 ? "right" : d.x < 0 ? "left" : d.y < 0 ? "up" : "down";
    };

    result["body"] = dirName( body );

    if( end != tap )
    {
        result["stub"] = { { "from", { toMm( tap.x ), toMm( tap.y ) } }, { "to", { toMm( end.x ), toMm( end.y ) } },
                           { "direction", dirName( end - tap ) } };
    }
    else if( tap != *point )
        result["on_wire_at"] = { toMm( tap.x ), toMm( tap.y ) };

    return KOPENAPI_RESULT::Ok( result );
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
    std::optional<VECTOR2I> nearby;
    const int               radius = toIU( aArgs.value( "radius_mm", 5.08 ) );

    if( aArgs.contains( "near" ) && aArgs["near"].is_object() )
        nearby = VECTOR2I( toIU( aArgs["near"].value( "x_mm", 0.0 ) ), toIU( aArgs["near"].value( "y_mm", 0.0 ) ) );

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

        if( nearby )
        {
            const BOX2I probe( *nearby - VECTOR2I( radius, radius ), VECTOR2I( 2 * radius, 2 * radius ) );

            if( !item->GetBoundingBox().Intersects( probe ) )
                continue;
        }

        if( item->IsLocked() )
            row["locked"] = true;

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

    if( text.empty() || ( !aArgs.contains( "pin" ) && ( !aArgs.contains( "x_mm" ) || !aArgs.contains( "y_mm" ) ) ) )
        return KOPENAPI_RESULT::Error( 400, "give 'text' and either 'pin' (REF.PIN or SHEETNAME.PIN: names the net there) or 'x_mm' / 'y_mm' "
                                            "(a point on a wire or a pin end)" );

    if( kind != "local" && kind != "global" && kind != "hierarchical" )
        return KOPENAPI_RESULT::Error( 400, "kind must be local, global or hierarchical" );

    std::optional<SCH_SHEET_PATH> sheet;
    VECTOR2I                      point;
    std::string                   side = aArgs.value( "side", std::string( "auto" ) );

    if( aArgs.contains( "pin" ) )
    {
        // Name the net at a pin: on a free grid point of the wire leaving the pin (text along
        // the wire, away from the pin, clear of parts and texts), else on the pin end itself
        std::optional<END_AT> at = aArgs["pin"].is_string() ? findEnd( context->GetSchematic(), aArgs["pin"] ) : std::nullopt;

        if( !at )
            return KOPENAPI_RESULT::Error( 404, "pin not found: " + aArgs["pin"].dump() );

        sheet = at->path;
        point = at->position;

        SCH_SCREEN* screen = sheet->LastScreen();
        const int   grid = toIU( GRID_MM );
        bool        found = false;

        auto freeAt = [&]( const VECTOR2I& aAt, const SCH_ITEM* aWire )
        {
            for( SCH_ITEM* item : screen->Items().Overlapping( aAt ) )
            {
                if( item == aWire )
                    continue;

                if( item->Type() == SCH_LINE_T && item->HitTest( aAt, 0 ) )
                    return false;

                if( item->Type() == SCH_SYMBOL_T )
                {
                    for( SCH_PIN* pin : static_cast<SCH_SYMBOL*>( item )->GetPins( &*sheet ) )
                    {
                        if( pin->GetPosition() == aAt )
                            return false;
                    }
                }
                else if( item->Type() != SCH_LINE_T && item->GetPosition() == aAt )
                {
                    return false;
                }
            }

            return true;
        };

        for( SCH_ITEM* item : screen->Items().Overlapping( SCH_LINE_T, point ) )
        {
            SCH_LINE* wire = static_cast<SCH_LINE*>( item );

            if( found || !wire->IsWire() || ( wire->GetStartPoint() != point && wire->GetEndPoint() != point ) )
                continue;

            const VECTOR2I other = wire->GetStartPoint() == point ? wire->GetEndPoint() : wire->GetStartPoint();
            const VECTOR2I along = other - point;
            const int      length = std::abs( along.x ) + std::abs( along.y );
            const VECTOR2I step( along.x == 0 ? 0 : ( along.x > 0 ? grid : -grid ), along.y == 0 ? 0 : ( along.y > 0 ? grid : -grid ) );
            const std::string away = along.x > 0 ? "right" : along.x < 0 ? "left" : along.y < 0 ? "up" : "down";

            for( int k = 1; k * grid < length && !found; ++k )
            {
                const VECTOR2I candidate = point + step * k;

                if( !freeAt( candidate, wire ) )
                    continue;

                // the text must lie on no part and no other text
                SCH_LABEL probe( candidate, wxString::FromUTF8( text ) );
                probe.SetSpinStyle( away == "right" ? SPIN_STYLE::RIGHT : away == "left" ? SPIN_STYLE::LEFT
                                    : away == "up"  ? SPIN_STYLE::UP    : SPIN_STYLE::BOTTOM );
                const BOX2I box = probe.GetBoundingBox();
                bool        clear = true;

                for( SCH_ITEM* other : screen->Items().Overlapping( box ) )
                {
                    if( other->Type() == SCH_SYMBOL_T )
                        clear &= !static_cast<SCH_SYMBOL*>( other )->GetBodyAndPinsBoundingBox().Intersects( box );
                    else if( labelKind( other->Type() ) || other->Type() == SCH_TEXT_T )
                        clear = false;
                }

                if( clear )
                {
                    point = candidate;
                    found = true;

                    if( side == "auto" )
                        side = away;
                }
            }
        }

        if( !found && side == "auto" )
        {
            const VECTOR2I out = at->out;
            side = out.x > 0 ? "right" : out.x < 0 ? "left" : out.y < 0 ? "up" : "down";
        }
    }
    else
    {
        sheet = targetSheet( *context, aArgs );

        if( !sheet )
            return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

        point = VECTOR2I( toIU( aArgs["x_mm"].get<double>() ), toIU( aArgs["y_mm"].get<double>() ) );
    }

    SCH_SCREEN* screen = sheet->LastScreen();

    // Text side: given, else away from the wire the label ends
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

    nlohmann::json result = { { "uuid", str( label->m_Uuid.AsString() ) }, { "net", net }, { "pins_on_net", pins }, { "side", side },
                              { "at", { toMm( point.x ), toMm( point.y ) } } };

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
    int        lockedKept = 0;
    const int  changed = annotate( context->GetSchematic(), commit, aArgs.value( "reset", false ),
                                   aArgs.value( "override_locks", false ), &lockedKept );
    pushEdit( commit, context->GetSchematic(), _( "Annotate (API)" ) );

    return KOPENAPI_RESULT::Ok( { { "changed", changed }, { "locked_kept", lockedKept } } );
}


namespace
{

constexpr double SHEET_PIN_PITCH_MM = 2.54;


std::optional<LABEL_FLAG_SHAPE> portShape( const std::string& aDirection )
{
    static const std::map<std::string, LABEL_FLAG_SHAPE> shapes = { { "input", LABEL_FLAG_SHAPE::L_INPUT },
                                                                    { "output", LABEL_FLAG_SHAPE::L_OUTPUT },
                                                                    { "bidirectional", LABEL_FLAG_SHAPE::L_BIDI },
                                                                    { "tri_state", LABEL_FLAG_SHAPE::L_TRISTATE },
                                                                    { "tristate", LABEL_FLAG_SHAPE::L_TRISTATE },
                                                                    { "passive", LABEL_FLAG_SHAPE::L_UNSPECIFIED } };
    auto it = shapes.find( aDirection );
    return it == shapes.end() ? std::nullopt : std::optional<LABEL_FLAG_SHAPE>( it->second );
}


std::optional<SHEET_SIDE> sheetSide( const std::string& aSide )
{
    static const std::map<std::string, SHEET_SIDE> sides = { { "left", SHEET_SIDE::LEFT },
                                                             { "right", SHEET_SIDE::RIGHT },
                                                             { "top", SHEET_SIDE::TOP },
                                                             { "bottom", SHEET_SIDE::BOTTOM } };
    auto it = sides.find( aSide );
    return it == sides.end() ? std::nullopt : std::optional<SHEET_SIDE>( it->second );
}


const char* sideName( SHEET_SIDE aSide )
{
    switch( aSide )
    {
    case SHEET_SIDE::LEFT:  return "left";
    case SHEET_SIDE::RIGHT: return "right";
    case SHEET_SIDE::TOP:   return "top";
    default:                return "bottom";
    }
}


bool vertical( SHEET_SIDE aSide )
{
    return aSide == SHEET_SIDE::LEFT || aSide == SHEET_SIDE::RIGHT;
}


/// @brief Distance of a pin from the top / left corner along its side
int sideOffset( const SCH_SHEET_PIN* aPin )
{
    const VECTOR2I d = aPin->GetPosition() - aPin->GetParent()->GetPosition();
    return vertical( aPin->GetSide() ) ? d.y : d.x;
}


VECTOR2I sideAt( const SCH_SHEET* aSheet, SHEET_SIDE aSide, int aOffset )
{
    const VECTOR2I pos = aSheet->GetPosition(), size = aSheet->GetSize();

    switch( aSide )
    {
    case SHEET_SIDE::LEFT:  return VECTOR2I( pos.x, pos.y + aOffset );
    case SHEET_SIDE::RIGHT: return VECTOR2I( pos.x + size.x, pos.y + aOffset );
    case SHEET_SIDE::TOP:   return VECTOR2I( pos.x + aOffset, pos.y );
    default:                return VECTOR2I( pos.x + aOffset, pos.y + size.y );
    }
}


nlohmann::json sheetPinCard( const SCH_SHEET_PIN* aPin )
{
    return { { "name", sheetPinName( aPin ) },
             { "direction", portDirection( aPin ) },
             { "side", sideName( aPin->GetSide() ) },
             { "offset_mm", toMm( sideOffset( aPin ) ) },
             { "x_mm", toMm( aPin->GetPosition().x ) },
             { "y_mm", toMm( aPin->GetPosition().y ) },
             { "uuid", str( aPin->m_Uuid.AsString() ) } };
}


struct PIN_SPEC
{
    std::string             name;
    LABEL_FLAG_SHAPE        shape = LABEL_FLAG_SHAPE::L_INPUT;
    SHEET_SIDE              side = SHEET_SIDE::LEFT;
    std::optional<double>   offset;
};


/// @brief Pin requests: name, direction (default: the shape of the sheet file's hierarchical label of
/// that name, else input), side, offset_mm.  409 on a name the sheet has or the request repeats.
std::optional<KOPENAPI_RESULT> pinSpecs( const nlohmann::json& aPins, const SCH_SHEET* aSheet, const SCH_SCREEN* aScreen,
                                         std::vector<PIN_SPEC>& aSpecs )
{
    std::set<std::string> names;

    for( const SCH_SHEET_PIN* pin : aSheet->GetPins() )
        names.insert( sheetPinName( pin ) );

    for( const nlohmann::json& p : aPins )
    {
        PIN_SPEC spec;
        spec.name = p.is_object() ? p.value( "name", std::string() ) : std::string();

        if( spec.name.empty() )
            return KOPENAPI_RESULT::Error( 400, "every pin needs a 'name'" );

        if( !names.insert( spec.name ).second )
            return KOPENAPI_RESULT::Error( 409, "the sheet already has a pin '" + spec.name + "'" );

        if( p.contains( "direction" ) )
        {
            std::optional<LABEL_FLAG_SHAPE> shape = portShape( p.value( "direction", std::string() ) );

            if( !shape )
                return KOPENAPI_RESULT::Error( 400, "direction must be input, output, bidirectional, tri_state or passive" );

            spec.shape = *shape;
        }
        else if( aScreen )
        {
            for( SCH_ITEM* item : aScreen->Items().OfType( SCH_HIER_LABEL_T ) )
            {
                if( str( UnescapeString( static_cast<SCH_HIERLABEL*>( item )->GetText() ) ) == spec.name )
                {
                    spec.shape = static_cast<SCH_HIERLABEL*>( item )->GetShape();
                    break;
                }
            }
        }

        std::optional<SHEET_SIDE> side = sheetSide( p.value( "side", std::string( "left" ) ) );

        if( !side )
            return KOPENAPI_RESULT::Error( 400, "side must be left, right, top or bottom" );

        spec.side = *side;

        if( p.contains( "offset_mm" ) && p["offset_mm"].is_number() )
            spec.offset = p["offset_mm"].get<double>();

        aSpecs.push_back( spec );
    }

    return std::nullopt;
}


/// @brief Put a pin on a side: at the given offset (snapped to the grid, inside the side), else on the
/// next free slot of the 2.54 mm pitch; the sheet grows when the side is full.  The caller has the
/// sheet in its commit.
SCH_SHEET_PIN* addSheetPin( SCH_SHEET* aSheet, SCH_SCREEN* aScreen, const PIN_SPEC& aSpec, std::string& aError )
{
    const int pitch = toIU( SHEET_PIN_PITCH_MM );
    int       length = vertical( aSpec.side ) ? aSheet->GetSize().y : aSheet->GetSize().x;
    int       offset = 0;

    auto taken = [&]( int aOffset )
    {
        for( const SCH_SHEET_PIN* pin : aSheet->GetPins() )
        {
            if( pin->GetSide() == aSpec.side && std::abs( sideOffset( pin ) - aOffset ) < pitch / 2 )
                return true;
        }

        return false;
    };

    if( aSpec.offset )
    {
        offset = toIU( *aSpec.offset );

        if( offset <= 0 || offset >= length )
        {
            aError = fmt::format( "offset_mm {} is not inside the {} side (0 < offset < {} mm)", *aSpec.offset,
                                  sideName( aSpec.side ), toMm( length ) );
            return nullptr;
        }

        if( taken( offset ) )
        {
            aError = fmt::format( "a pin already sits at offset {} mm of the {} side", toMm( offset ), sideName( aSpec.side ) );
            return nullptr;
        }
    }
    else
    {
        offset = pitch;

        while( taken( offset ) )
            offset += pitch;

        if( offset + pitch > length )
        {
            const VECTOR2I size = aSheet->GetSize();
            aSheet->Resize( vertical( aSpec.side ) ? VECTOR2I( size.x, offset + pitch ) : VECTOR2I( offset + pitch, size.y ) );
        }
    }

    const int textSize = aSheet->Schematic()->Settings().m_DefaultTextSize;
    auto*     pin = new SCH_SHEET_PIN( aSheet, sideAt( aSheet, aSpec.side, offset ), wxString::FromUTF8( aSpec.name ) );

    pin->SetTextSize( VECTOR2I( textSize, textSize ) );
    pin->SetShape( aSpec.shape );
    pin->SetSide( aSpec.side );
    pin->ConstrainOnEdge( sideAt( aSheet, aSpec.side, offset ), false );
    aSheet->AddPin( pin );
    pin->AutoplaceFields( aScreen, AUTOPLACE_AUTO );
    return pin;
}


/// @brief A free spot for a hierarchical label in a sheet's own file: a column near the left page edge
/// (text left, connection point to the right) or, for pins on the right / bottom side, near the
/// right edge; clear of everything drawn and of the labels placed before (aTaken)
SCH_HIERLABEL* freeHierLabel( SCH_SCREEN* aScreen, const SCH_SHEET_PIN* aPin, std::vector<BOX2I>& aTaken )
{
    const VECTOR2D page = aScreen->GetPageSettings().GetSizeIU( schIUScale.IU_PER_MILS );
    const bool     right = aPin->GetSide() == SHEET_SIDE::RIGHT || aPin->GetSide() == SHEET_SIDE::BOTTOM;
    const int      x = right ? toIU( schIUScale.IUTomm( int( page.x ) ) - 50.8 ) : toIU( 50.8 );
    const int      textSize = aScreen->Schematic() ? aScreen->Schematic()->Settings().m_DefaultTextSize : toIU( 1.27 );

    auto make = [&]( int aY )
    {
        auto* label = new SCH_HIERLABEL( VECTOR2I( x, aY ), aPin->GetText() );
        label->SetTextSize( VECTOR2I( textSize, textSize ) );
        label->SetShape( aPin->GetShape() );
        label->SetSpinStyle( right ? SPIN_STYLE::RIGHT : SPIN_STYLE::LEFT );
        return label;
    };

    const int air = toIU( 1.27 );

    for( int y = toIU( 25.4 ); y < int( page.y ) - toIU( 50.8 ); y += toIU( SHEET_PIN_PITCH_MM ) )
    {
        SCH_HIERLABEL* label = make( y );
        const BOX2I    box = label->GetBoundingBox().GetInflated( air );
        bool           clear = true;

        for( SCH_ITEM* item : aScreen->Items().Overlapping( box ) )
        {
            if( item->GetBoundingBox().Intersects( box ) )
                clear = false;
        }

        for( const BOX2I& other : aTaken )
            clear &= !other.Intersects( box );

        if( clear )
        {
            aTaken.push_back( box );
            return label;
        }

        delete label;
    }

    return make( toIU( 25.4 ) );
}


/// @brief Sheet instance of a sheet symbol by its path (as sch_sheet_list prints it), its uuid, or its
/// name when that is unique; the root is no sheet symbol
std::optional<SCH_SHEET_PATH> sheetInstance( SCHEMATIC* aSchematic, const std::string& aWanted, int& aStatus, std::string& aError )
{
    std::vector<SCH_SHEET_PATH> byName;

    for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
    {
        if( path.size() < 2 || path.at( path.size() - 2 )->IsVirtualRootSheet() )
            continue;

        if( sheetPath( path ) == aWanted || str( path.Last()->m_Uuid.AsString() ) == aWanted )
            return path;

        if( str( path.Last()->GetName() ) == aWanted )
            byName.push_back( path );
    }

    if( byName.size() == 1 )
        return byName.front();

    if( byName.empty() )
    {
        std::vector<SCH_SHEET_PATH> paths;
        std::vector<SCH_ITEM*>      sheets;

        for( const SCH_SHEET_PATH& path : aSchematic->Hierarchy() )
        {
            if( path.size() < 2 || path.at( path.size() - 2 )->IsVirtualRootSheet() )
                continue;

            paths.push_back( path );
            sheets.push_back( path.Last() );
        }

        std::string error;

        if( SCH_ITEM* hit = KopenapiSchMatchUuid( sheets, aWanted, error ) )
        {
            for( size_t i = 0; i < sheets.size(); ++i )
            {
                if( sheets[i] == hit )
                    return paths[i];
            }
        }
        else if( error.find( "ambiguous" ) != std::string::npos )
        {
            aStatus = 400;
            aError = error;
            return std::nullopt;
        }
    }

    aStatus = byName.empty() ? 404 : 409;
    aError = byName.empty() ? "sheet not found: " + aWanted + " (sch_sheet_list paths, or the sheet uuid)"
                            : "sheet name '" + aWanted + "' is used by several sheets: give the path";
    return std::nullopt;
}


bool sameFile( const wxString& aA, const wxString& aB )
{
    return !aA.IsEmpty() && !aB.IsEmpty() && wxFileName( aA ) == wxFileName( aB );
}


/// @brief A screen's file and the files of every sheet below it
void collectFiles( const SCH_SCREEN* aScreen, std::vector<wxString>& aFiles, int aDepth = 0 )
{
    if( !aScreen || aDepth > 64 )
        return;

    aFiles.push_back( aScreen->GetFileName() );

    for( SCH_ITEM* item : aScreen->Items().OfType( SCH_SHEET_T ) )
        collectFiles( static_cast<SCH_SHEET*>( item )->GetScreen(), aFiles, aDepth + 1 );
}


bool startsWith( const SCH_SHEET_PATH& aPath, const SCH_SHEET_PATH& aPrefix )
{
    if( aPath.size() < aPrefix.size() )
        return false;

    for( size_t i = 0; i < aPrefix.size(); ++i )
    {
        if( aPath.at( i ) != aPrefix.at( i ) )
            return false;
    }

    return true;
}

} // namespace


static KOPENAPI_RESULT h_sch_sheet_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string name = aArgs.value( "name", std::string() );
    const std::string file = aArgs.value( "file", std::string() );

    if( name.empty() || file.empty() || !aArgs.contains( "x_mm" ) || !aArgs.contains( "y_mm" ) )
        return KOPENAPI_RESULT::Error( 400, "give 'name', 'file' (.kicad_sch), 'x_mm' and 'y_mm'" );

    const double widthMm = aArgs.value( "width_mm", 50.8 ), heightMm = aArgs.value( "height_mm", 25.4 );

    if( toIU( widthMm ) < toIU( 2 * SHEET_PIN_PITCH_MM ) || toIU( heightMm ) < toIU( 2 * SHEET_PIN_PITCH_MM ) )
        return KOPENAPI_RESULT::Error( 400, "width_mm and height_mm must be at least 5.08" );

    if( aArgs.contains( "pins" ) && !aArgs["pins"].is_array() )
        return KOPENAPI_RESULT::Error( 400, "'pins' is an array of {name, direction, side, offset_mm}" );

    std::optional<SCH_SHEET_PATH> parent = targetSheet( *context, aArgs );

    if( !parent )
        return KOPENAPI_RESULT::Error( 404, "sheet not found (see sch_sheet_list)" );

    SCHEMATIC*     schematic = context->GetSchematic();
    SCH_SCREEN*    parentScreen = parent->LastScreen();
    const wxString parentFile = parentScreen->GetFileName();

    if( parentFile.IsEmpty() )
        return KOPENAPI_RESULT::Error( 409, "the parent sheet has no file yet: save the schematic first (sch_save)" );

    // The file: relative names are taken from the project directory; the sheet stores it relative to
    // its parent's directory, as the editor does
    wxFileName fn( wxString::FromUTF8( file ) );

    if( fn.GetExt() != wxString::FromUTF8( FILEEXT::KiCadSchematicFileExtension ) )
        return KOPENAPI_RESULT::Error( 422, "file must be a KiCad schematic (.kicad_sch): " + file );

    const wxString projectDir = context->Prj().GetProjectPath();
    fn.Normalize( FN_NORMALIZE_FLAGS, projectDir.IsEmpty() ? wxFileName( parentFile ).GetPath() : projectDir );

    wxString absolute = fn.GetFullPath();
    absolute.Replace( wxT( "\\" ), wxT( "/" ) );

    wxFileName relative( absolute );
    wxString   stored = relative.MakeRelativeTo( wxFileName( parentFile ).GetPath() ) ? relative.GetFullPath() : absolute;
    stored.Replace( wxT( "\\" ), wxT( "/" ) );

    const wxString wxName = wxString::FromUTF8( name );

    for( SCH_ITEM* item : parentScreen->Items().OfType( SCH_SHEET_T ) )
    {
        if( static_cast<SCH_SHEET*>( item )->GetName().CmpNoCase( wxName ) == 0 )
            return KOPENAPI_RESULT::Error( 409, "the parent sheet already has a sheet named '" + name + "' (names are unique "
                                                "per sheet, case-insensitive)" );
    }

    // Recursion: the file must not be the parent or one of its ancestors, nor contain one of them
    std::vector<wxString> ancestors;

    for( size_t i = 0; i < parent->size(); ++i )
    {
        if( !parent->at( i )->IsVirtualRootSheet() && parent->at( i )->GetScreen() )
            ancestors.push_back( parent->at( i )->GetScreen()->GetFileName() );
    }

    auto recursion = [&]( const std::vector<wxString>& aFiles ) -> std::optional<KOPENAPI_RESULT>
    {
        for( const wxString& f : aFiles )
        {
            for( const wxString& a : ancestors )
            {
                if( sameFile( f, a ) )
                    return KOPENAPI_RESULT::Error( 422, "recursion: " + str( f ) + " is the parent sheet or one of its "
                                                        "ancestors, so the sheet would contain itself" );
            }
        }

        return std::nullopt;
    };

    if( std::optional<KOPENAPI_RESULT> refused = recursion( { absolute } ) )
        return *refused;

    SCH_SCREEN* reuse = nullptr;

    for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
    {
        const wxFileName other( path.LastScreen()->GetFileName() );

        if( sameFile( other.GetFullPath(), absolute ) )
        {
            reuse = path.LastScreen();
            break;
        }

        if( other.GetPath() == fn.GetPath() && other.GetFullName().CmpNoCase( fn.GetFullName() ) == 0 )
            return KOPENAPI_RESULT::Error( 409, "the file name differs only in case from " + str( other.GetFullPath() )
                                                + ": that breaks on case-insensitive file systems" );
    }

    auto sheet = std::make_unique<SCH_SHEET>( parent->Last(), VECTOR2I( toIU( aArgs["x_mm"].get<double>() ), toIU( aArgs["y_mm"].get<double>() ) ),
                                              VECTOR2I( toIU( widthMm ), toIU( heightMm ) ) );
    sheet->SetName( wxName );
    sheet->SetFileName( stored );

    nlohmann::json             warnings = nlohmann::json::array();
    const SCH_SHEET_LIST       before = schematic->Hierarchy();
    const wxString             projectName = context->Prj().GetProjectName();
    std::string                source = reuse ? "shared" : fn.FileExists() ? "loaded" : "created";
    std::unique_ptr<SCH_SHEET> loaded;
    SCH_SCREEN*                screen = reuse;

    if( source == "loaded" )
    {
        // Read like SCH_EDIT_FRAME::LoadSheetFromFile, without its dialogs
        IO_RELEASER<SCH_IO> pi( SCH_IO_MGR::FindPlugin( SCH_IO_MGR::SCH_KICAD ) );
        loaded = std::make_unique<SCH_SHEET>( schematic );
        const_cast<KIID&>( loaded->m_Uuid ) = sheet->m_Uuid;
        loaded->SetFileName( absolute );

        std::map<std::string, UTF8> props;
        props["hierarchical_sheet_load"] = "1";

        try
        {
            pi->LoadSchematicFile( absolute, schematic, loaded.get(), &props );
        }
        catch( const IO_ERROR& error )
        {
            return KOPENAPI_RESULT::Error( 422, "not a readable KiCad schematic: " + file + ": " + str( error.What() ) );
        }

        // The editor offers a partial schematic here; the API refuses instead
        if( !pi->GetError().IsEmpty() )
            return KOPENAPI_RESULT::Error( 422, "not a readable KiCad schematic (or a sheet in it is not): " + file + ": "
                                                + str( pi->GetError() ) );

        screen = loaded->GetScreen();

        if( !screen )
            return KOPENAPI_RESULT::Error( 422, "not a readable KiCad schematic: " + file );

        if( SCH_SCREENS( loaded.get() ).HasNoFullyDefinedLibIds() )
            return KOPENAPI_RESULT::Error( 422, "the file's symbols are not linked to library ids (a pre-6 schematic "
                                                "not remapped): remap it in its own project first" );
    }

    if( screen )
    {
        std::vector<wxString> files;
        collectFiles( screen, files );

        if( std::optional<KOPENAPI_RESULT> refused = recursion( files ) )
            return *refused;
    }

    // Pins asked for: checked on a copy before anything changes
    std::vector<PIN_SPEC> specs;
    std::string           error;

    if( aArgs.contains( "pins" ) )
    {
        if( std::optional<KOPENAPI_RESULT> refused = pinSpecs( aArgs["pins"], sheet.get(), screen, specs ) )
            return *refused;

        SCH_SHEET probe( *sheet );

        for( const PIN_SPEC& spec : specs )
        {
            if( !addSheetPin( &probe, nullptr, spec, error ) )
                return KOPENAPI_RESULT::Error( error.find( "already sits" ) != std::string::npos ? 409 : 400, error );
        }
    }

    if( source == "shared" )
    {
        // Another instance of a file already in the hierarchy shares its screen
        sheet->SetScreen( reuse );

        SCH_SHEET_LIST below( sheet.get() );

        if( !projectName.IsEmpty() )
            below.AddNewSymbolInstances( *parent, projectName );

        below.AddNewSheetInstances( *parent, before.GetLastVirtualPageNumber() );
    }
    else if( source == "loaded" )
    {
        // Sheets inside a file from another folder keep pointing at their files
        if( fn.GetPathWithSep() != projectDir )
        {
            for( const SCH_SHEET_PATH& path : SCH_SHEET_LIST( loaded.get() ) )
            {
                wxString lastPath = projectDir;

                for( unsigned i = 1; i < path.size(); i++ )
                {
                    SCH_SHEET*  sub = path.at( i );
                    SCH_SCREEN* subScreen = sub ? sub->GetScreen() : nullptr;

                    if( !subScreen || !wxFileName( subScreen->GetFileName() ).IsAbsolute() )
                        continue;

                    wxFileName subFile( subScreen->GetFileName() );
                    wxFileName tmp = subFile;
                    wxString   subName = tmp.MakeRelativeTo( lastPath ) ? tmp.GetFullPath() : subFile.GetFullPath();

                    subName.Replace( wxT( "\\" ), wxT( "/" ) );
                    sub->SetFileName( subName );
                    lastPath = subFile.GetPath();
                }
            }
        }

        SCH_SHEET_LIST below( loaded.get() );

        if( screen->GetFileFormatVersionAtLoad() < 20221002 && !screen->GetSymbolInstances().empty() )
            below.UpdateSymbolInstanceData( screen->GetSymbolInstances() );

        screen->MigrateSimModels();

        if( !projectName.IsEmpty() )
            below.AddNewSymbolInstances( *parent, projectName );

        below.AddNewSheetInstances( *parent, before.GetLastVirtualPageNumber() );
        sheet->SetScreen( screen );
        loaded.reset();
    }
    else
    {
        if( !fn.DirExists() && !wxFileName::Mkdir( fn.GetPath(), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL ) )
            return KOPENAPI_RESULT::Error( 500, "cannot create the directory of " + file );

        screen = new SCH_SCREEN( schematic );
        screen->SetFileName( absolute );
        screen->SetPageSettings( parentScreen->GetPageSettings() );
        sheet->SetScreen( screen );

        SCH_SHEET_PATH instance = *parent;
        instance.push_back( sheet.get() );
        instance.SetPageNumber( before.GetNextPageNumber() );
    }

    // Pins: those asked for, then one per hierarchical label of the file without a pin (outputs on the
    // right, the rest on the left, by name: like the editor's automatic sheet pins)
    for( const PIN_SPEC& spec : specs )
        addSheetPin( sheet.get(), parentScreen, spec, error );

    if( aArgs.value( "sync_pins", true ) )
    {
        std::vector<SCH_HIERLABEL*> labels = GetUnplacedSheetPinLabels( *sheet );

        std::sort( labels.begin(), labels.end(),
                   []( const SCH_HIERLABEL* a, const SCH_HIERLABEL* b ) { return a->GetText() < b->GetText(); } );

        for( SCH_HIERLABEL* label : labels )
        {
            PIN_SPEC spec;
            spec.name = str( UnescapeString( label->GetText() ) );
            spec.shape = label->GetShape();
            spec.side = label->GetShape() == LABEL_FLAG_SHAPE::L_OUTPUT ? SHEET_SIDE::RIGHT : SHEET_SIDE::LEFT;
            addSheetPin( sheet.get(), parentScreen, spec, error );
        }
    }

    sheet->AutoplaceFields( parentScreen, AUTOPLACE_AUTO );

    // On the screen before the commit so the hierarchy and the annotation see the new instance; the
    // commit then only records it (and adds it to the view)
    SCH_SHEET* placed = sheet.release();
    parentScreen->Append( placed );

    SCH_COMMIT commit( context->GetToolManager() );
    commit.Add( placed, parentScreen );
    schematic->RefreshHierarchy();

    if( source == "loaded" )
        SCH_SCREENS( schematic->Root() ).ReplaceDuplicateTimeStamps();

    SCH_SHEET_PATH full = *parent;
    full.push_back( placed );

    // The sheet's own page number comes from the properties dialog in the editor
    if( full.GetPageNumber().IsEmpty() )
    {
        full.SetPageNumber( schematic->Hierarchy().GetNextPageNumber() );
        schematic->RefreshHierarchy();
    }

    if( aArgs.value( "annotate", true ) )
    {
        // Like the editor's automatic annotation of a new sheet: its symbols renumbered for this instance
        for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
        {
            if( !startsWith( path, full ) )
                continue;

            for( SCH_ITEM* item : path.LastScreen()->Items().OfType( SCH_SYMBOL_T ) )
            {
                SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

                if( !symbol->IsPower() )
                    symbol->ClearAnnotation( &path, false );
            }
        }

        annotate( schematic, commit, false );
    }

    pushEdit( commit, schematic, _( "Add sheet (API)" ), aCtx.kiway, { placed->m_Uuid } );

    if( source == "created" && !SCH_API_SAVE::SaveSheetToFile( placed, *schematic, absolute ) )
        warnings.push_back( "could not write " + str( absolute ) + " yet: sch_save writes it" );

    for( const SCH_SHEET_PATH& path : schematic->Hierarchy() )
    {
        if( path.Path() == full.Path() )
            full = path;
    }

    nlohmann::json pins = nlohmann::json::array(), symbols = nlohmann::json::array();

    for( const SCH_SHEET_PIN* pin : placed->GetPins() )
        pins.push_back( sheetPinCard( pin ) );

    for( SCH_ITEM* item : placed->GetScreen()->Items().OfType( SCH_SYMBOL_T ) )
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

        if( !symbol->IsPower() )
            symbols.push_back( { { "ref", str( symbol->GetRef( &full, false ) ) },
                                 { "value", str( symbol->GetValue( &full, FOR_GUI ) ) },
                                 { "lib_id", str( symbol->GetLibId().Format() ) },
                                 { "uuid", str( symbol->m_Uuid.AsString() ) } } );
    }

    nlohmann::json result = { { "uuid", str( placed->m_Uuid.AsString() ) },
                              { "path", sheetPath( full ) },
                              { "name", name },
                              { "file", str( stored ) },
                              { "file_absolute", str( absolute ) },
                              { "source", source },
                              { "page", str( full.GetPageNumber() ) },
                              { "x_mm", toMm( placed->GetPosition().x ) },
                              { "y_mm", toMm( placed->GetPosition().y ) },
                              { "width_mm", toMm( placed->GetSize().x ) },
                              { "height_mm", toMm( placed->GetSize().y ) },
                              { "pins", pins },
                              { "symbols", symbols } };

    if( !warnings.empty() )
        result["warnings"] = warnings;

    return KOPENAPI_RESULT::Ok( result );
}


static KOPENAPI_RESULT h_sch_sheet_pin_add( KOPENAPI_CONTEXT& aCtx, const nlohmann::json& aArgs )
{
    std::shared_ptr<SCH_CONTEXT> context = KopenapiSchContext( aCtx );

    if( !context )
        return KopenapiNoSchematic();

    const std::string wanted = aArgs.value( "sheet_path", aArgs.value( "uuid", std::string() ) );

    if( wanted.empty() || !aArgs.contains( "pins" ) || !aArgs["pins"].is_array() || aArgs["pins"].empty() )
        return KOPENAPI_RESULT::Error( 400, "give 'sheet_path' (or the sheet's uuid) and 'pins' [{name, direction, side, offset_mm}]" );

    SCHEMATIC*                    schematic = context->GetSchematic();
    int                           status = 404;
    std::string                   error;
    std::optional<SCH_SHEET_PATH> instance = sheetInstance( schematic, wanted, status, error );

    if( !instance )
        return KOPENAPI_RESULT::Error( status, error );

    SCH_SHEET*     sheet = instance->Last();
    SCH_SHEET_PATH parent = *instance;
    parent.pop_back();

    SCH_SCREEN*           parentScreen = parent.LastScreen();
    SCH_SCREEN*           own = sheet->GetScreen();
    std::vector<PIN_SPEC> specs;

    if( std::optional<KOPENAPI_RESULT> refused = pinSpecs( aArgs["pins"], sheet, own, specs ) )
        return *refused;

    // Offsets checked on a copy first, so a bad one changes nothing
    {
        SCH_SHEET probe( *sheet );

        for( const PIN_SPEC& spec : specs )
        {
            if( !addSheetPin( &probe, nullptr, spec, error ) )
                return KOPENAPI_RESULT::Error( error.find( "already sits" ) != std::string::npos ? 409 : 400, error );
        }
    }

    const bool        createLabels = aArgs.value( "create_labels", false );
    SCH_COMMIT        commit( context->GetToolManager() );
    std::vector<KIID> glowing = { sheet->m_Uuid };
    std::vector<BOX2I> taken;
    nlohmann::json    pins = nlohmann::json::array();

    commit.Modify( sheet, parentScreen );

    for( const PIN_SPEC& spec : specs )
    {
        SCH_SHEET_PIN* pin = addSheetPin( sheet, parentScreen, spec, error );
        nlohmann::json card = sheetPinCard( pin );

        if( createLabels && own )
        {
            SCH_HIERLABEL* existing = nullptr;

            for( SCH_ITEM* item : own->Items().OfType( SCH_HIER_LABEL_T ) )
            {
                if( static_cast<SCH_HIERLABEL*>( item )->GetText() == pin->GetText() )
                    existing = static_cast<SCH_HIERLABEL*>( item );
            }

            SCH_HIERLABEL* label = existing ? existing : freeHierLabel( own, pin, taken );

            if( !existing )
                commit.Add( label, own );

            card["label"] = { { "uuid", str( label->m_Uuid.AsString() ) },
                              { "x_mm", toMm( label->GetPosition().x ) },
                              { "y_mm", toMm( label->GetPosition().y ) },
                              { "created", !existing } };
        }

        pins.push_back( card );
    }

    pushEdit( commit, schematic, _( "Add sheet pin (API)" ), aCtx.kiway, glowing );

    for( nlohmann::json& card : pins )
    {
        for( const SCH_SHEET_PIN* pin : sheet->GetPins() )
        {
            if( str( pin->m_Uuid.AsString() ) == card["uuid"].get<std::string>() )
                card["net"] = netOfItem( schematic, pin, parent );
        }
    }

    return KOPENAPI_RESULT::Ok( { { "sheet", sheetPath( *instance ) },
                                  { "uuid", str( sheet->m_Uuid.AsString() ) },
                                  { "parent", sheetPath( parent ) },
                                  { "width_mm", toMm( sheet->GetSize().x ) },
                                  { "height_mm", toMm( sheet->GetSize().y ) },
                                  { "pins", pins } } );
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
                                "uuid, ref, every pin with its position, field text boxes, and with a footprint "
                                "footprint_check (found, every pin has its pad)." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["lib_id","x_mm","y_mm"],"properties":{
                        "lib_id":{"type":"string"},"x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "sheet":{"type":"string","description":"sheet path from sch_sheet_list; default current/root"},
                        "unit":{"type":"integer","default":1},
                        "rotation":{"type":"integer","enum":[0,90,180,270],"default":0},
                        "mirror":{"type":"string","enum":["none","x","y"],"default":"none"},
                        "ref":{"type":"string"},"value":{"type":"string"},"footprint":{"type":"string"},
                        "fields":{"type":"object","additionalProperties":{"type":"string"}},
                        "dnp":{"type":"boolean"},"in_bom":{"type":"boolean"},"on_board":{"type":"boolean"},"in_pos":{"type":"boolean"},
                        "annotate":{"type":"boolean","default":true}}})json"_json,
                   false, h_sch_symbol_add, 120 );

KOPENAPI_REGISTER( "sch_symbol_update",
                   std::string( "Edit a placed symbol by uuid or ref: value, footprint, fields, new_ref, "
                                "position, rotation, mirror, dnp / in_bom / on_board / in_pos, field text positions "
                                "(field_positions, autoplace_fields); moving carries the part's own wiring (stubs, power "
                                "symbols, PWR_FLAGs, labels) and stretches / re-routes wiring shared with other "
                                "parts." ) + EDIT_NOTE,
                   R"json({"type":"object","properties":{
                        "uuid":{"type":"string","description":"whole uuid or a unique prefix (4+ characters)"},"ref":{"type":"string","description":"lookup by reference"},
                        "new_ref":{"type":"string"},"value":{"type":"string"},"footprint":{"type":"string"},
                        "fields":{"type":"object","additionalProperties":{"type":"string"}},
                        "x_mm":{"type":"number"},"y_mm":{"type":"number"},
                        "rotation":{"type":"integer","enum":[0,90,180,270]},
                        "mirror":{"type":"string","enum":["none","x","y"]},
                        "dnp":{"type":"boolean"},"in_bom":{"type":"boolean"},"on_board":{"type":"boolean"},
                        "in_pos":{"type":"boolean","description":"in the position (pick and place) files"},
                        "field_positions":{"type":"object","description":"per field name: {x_mm, y_mm}, \"hide\" or \"show\""},
                        "autoplace_fields":{"type":"boolean","default":false,"description":"re-place reference / value text clear of other parts, texts and wires (done anyway after a move)"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_symbol_update );

KOPENAPI_REGISTER( "sch_items_move",
                   std::string( "Move several schematic items together by dx / dy mm (symbols, wires, labels, "
                                "power symbols, junctions by uuid, e.g. a part with its own wiring): connections "
                                "among them are kept; wiring that belongs only to moved parts comes along; wires to "
                                "the rest stretch and are re-routed orthogonally (warnings if not)." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["uuids"],"properties":{
                        "uuids":{"type":"array","items":{"type":"string"},"description":"whole uuids or unique prefixes (4+ characters)"},
                        "dx_mm":{"type":"number","default":0},"dy_mm":{"type":"number","default":0},
                        "sheet":{"type":"string"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_items_move );

KOPENAPI_REGISTER( "sch_blocks_arrange",
                   std::string( "Lay a sheet out as compact blocks: every part goes into one of 'blocks' "
                                "(name + refs); each block moves as a whole with its own wiring, labels, "
                                "no-connects and power symbols; wires between blocks stretch and are re-routed. "
                                "Blocks are packed in 'rows' (block names per row; default: shelves about as wide "
                                "as tall) with gap_mm between them, centred in the page's drawing area; paper "
                                "auto picks the smallest ISO landscape page that holds them (keep: leave the "
                                "page). Texts and graphics inside a block's area go with it. The page change is "
                                "not on the undo stack." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["blocks"],"properties":{
                        "blocks":{"type":"array","items":{"type":"object","required":["refs"],"properties":{
                            "name":{"type":"string"},"refs":{"type":"array","items":{"type":"string"}}}}},
                        "rows":{"type":"array","items":{"type":"array","items":{"type":"string"}}},
                        "gap_mm":{"type":"number","default":12.7},
                        "align":{"type":"string","enum":["center","left"],"default":"center"},
                        "paper":{"type":"string","default":"auto","description":"auto, keep, or A4..A0, A..E, USLetter..."},
                        "sheet":{"type":"string"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_blocks_arrange );

KOPENAPI_REGISTER( "sch_page_set",
                   "Set a sheet's paper: A5..A0, A..E, USLetter, USLegal, USLedger, User (width_mm, height_mm) "
                   "or auto (the smallest ISO landscape page whose drawing area holds what is drawn); "
                   "portrait. Warns when the drawing reaches beyond the page. Not on the undo stack.",
                   R"json({"type":"object","properties":{
                        "paper":{"type":"string"},
                        "portrait":{"type":"boolean"},
                        "width_mm":{"type":"number"},"height_mm":{"type":"number"},
                        "sheet":{"type":"string"}}})json"_json,
                   false, h_sch_page_set );

KOPENAPI_REGISTER( "sch_item_delete",
                   std::string( "Delete schematic items (symbols, labels, wires, no-connects...) by uuid." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["uuids"],"properties":{
                        "uuids":{"type":"array","items":{"type":"string"},"description":"whole uuids or unique prefixes (4+ characters)"},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
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
                        "power_lib_id":{"type":"string","description":"power symbol, default power:<net>"},
                        "orientation":{"type":"string","enum":["upright","away"],"default":"upright",
                                       "description":"power: upright keeps GND down / +V up (a short stub from sideways pins); away points the body away from the part"}}})json"_json,
                   false, h_sch_connect, 120 );

KOPENAPI_REGISTER( "sch_power_add",
                   std::string( "Put one power symbol (power:GND, power:+5V, ...) or power:PWR_FLAG on a pin "
                                "(REF.PIN) or a point on a wire, upright; when the point is taken (another power "
                                "symbol, a label), the pin points sideways or the symbol would lie on a part or "
                                "text, it hangs on a wire already leaving the point (T + junction) or takes a "
                                "short stub to a free spot - auto tries: out of the pin, sideways, against the "
                                "body; upright before upside down; 2..8 grid - or give stub left/right/up/down/none; "
                                "answers where it went (stub direction, body). For ERC power_pin_not_driven: a "
                                "PWR_FLAG on the rail's connector pin." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["lib_id","at"],"properties":{
                        "lib_id":{"type":"string","description":"e.g. power:PWR_FLAG, power:GND"},
                        "at":{"description":"REF.PIN or {x_mm, y_mm} on a wire"},
                        "stub":{"type":"string","enum":["auto","none","left","right","up","down"],"default":"auto"},
                        "sheet":{"type":"string","description":"for a point"}}})json"_json,
                   false, h_sch_power_add );

KOPENAPI_REGISTER( "sch_wire",
                   std::string( "Draw a wire between two pins (REF.PIN, sheet pins SHEETNAME.PIN) or points {x_mm, y_mm} on one sheet: "
                                "straight when aligned, else one corner (route auto/hv/vh); junction dots "
                                "where it meets other wires; returns the segments and the resulting net. "
                                "Use for short local connections, sch_connect labels for long ones." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["from","to"],"properties":{
                        "from":{"description":"REF.PIN, SHEETNAME.PIN (a sheet pin) or {x_mm, y_mm}"},
                        "to":{"description":"REF.PIN, SHEETNAME.PIN (a sheet pin) or {x_mm, y_mm}"},
                        "route":{"type":"string","enum":["auto","hv","vh"],"default":"auto"},
                        "sheet":{"type":"string","description":"for point-only wires"}}})json"_json,
                   false, h_sch_wire );

KOPENAPI_REGISTER( "sch_texts_settle",
                   std::string( "Move the reference / value texts of every part on a sheet (power symbols too) that lie "
                                "on something - another text, a part, a wire, a label - to a clear spot next to their "
                                "part (where KiCad places them, else right / below / above / left at a growing "
                                "distance); texts already clear stay. Returns the parts moved and the ones with no "
                                "clear spot nearby (stuck: move the part). Run before sch_layout_check" ) + EDIT_NOTE,
                   R"json({"type":"object","properties":{"sheet":{"type":"string"}}})json"_json,
                   false, h_sch_texts_settle );


KOPENAPI_REGISTER( "sch_rewire",
                   std::string( "Turn label wiring into wires a reader can follow: on a sheet, local labels sitting on "
                                "part pins are linked per net by a shortest tree, every link up to max_mm is drawn "
                                "as a wire by the router (no other pin, wire end, label or part body touched; "
                                "junctions added), and of each group of pins the wires now join one label stays "
                                "(the net keeps its name; far groups stay linked by it). The circuit is checked "
                                "before / after and restored if it changed. Per net: pins, wires, labels left, "
                                "links not routed (why: move a part, then run it again). dry_run: plan only" ) + EDIT_NOTE,
                   R"json({"type":"object","properties":{
                        "sheet":{"type":"string"},
                        "net":{"type":"string","description":"only this net (label text)"},
                        "max_mm":{"type":"number","default":50.8,"description":"longest link drawn as a wire (Manhattan)"},
                        "dry_run":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_rewire );

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
                   std::string( "Place a net label so it names / joins a net (local, global or hierarchical): at "
                                "a point on a wire or a pin end (x_mm, y_mm), or by pin (REF.PIN: on a free spot of "
                                "the wire leaving that pin, text along the wire clear of parts, else on the pin "
                                "end) - 'name this net'; text side auto or left/right/up/down; answers the net it "
                                "joined and where, warns when it touches nothing. Labels on many pins at once: "
                                "sch_connect." ) + EDIT_NOTE,
                   R"json({"type":"object","required":["text"],"properties":{
                        "text":{"type":"string"},"pin":{"type":"string","description":"REF.PIN or SHEETNAME.PIN (a sheet pin), instead of x_mm / y_mm"},
                        "x_mm":{"type":"number"},"y_mm":{"type":"number"},
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
                                "number; keeps existing references unless reset; a reset keeps the references "
                                "of locked symbols (override_locks); power symbols numbered #PWR." ) + EDIT_NOTE,
                   R"json({"type":"object","properties":{"reset":{"type":"boolean","default":false},
                        "override_locks":{"type":"boolean","default":false}}})json"_json,
                   false, h_sch_annotate );

#define SCH_LOCK_SCHEMA R"json({"type":"object","properties":{
                        "refs":{"type":"array","items":{"type":"string"},"description":"symbols by reference: every unit and sheet instance"},
                        "uuids":{"type":"array","items":{"type":"string"},"description":"any items (symbols, wires, labels, junctions, sheets) on any sheet; whole uuids or unique prefixes (4+ characters)"}}})json"_json

KOPENAPI_REGISTER( "sch_lock",
                   "Lock schematic items (KiCad's lock flag, saved with the sheets): symbols by reference, any "
                   "item by uuid. Locked items are not changed by sch_symbol_update, sch_items_move or "
                   "sch_item_delete (409 naming them) unless override_locks; the editor does not select "
                   "them for moving either. One undo step",
                   SCH_LOCK_SCHEMA, false, h_sch_lock );

KOPENAPI_REGISTER( "sch_unlock", "Unlock schematic items (refs, uuids); one undo step", SCH_LOCK_SCHEMA, false, h_sch_unlock );

#define SCH_SHEET_PIN_SCHEMA R"json({"type":"object","required":["name"],"properties":{
                        "name":{"type":"string"},
                        "direction":{"type":"string","enum":["input","output","bidirectional","tri_state","passive"],
                                     "description":"default: the shape of the sheet file's hierarchical label of that name, else input"},
                        "side":{"type":"string","enum":["left","right","top","bottom"],"default":"left"},
                        "offset_mm":{"type":"number","description":"along the side from its top / left corner; default: next free slot on the 2.54 mm pitch (the sheet grows when full)"}}})json"

KOPENAPI_REGISTER( "sch_sheet_add",
                   std::string( "Add a hierarchical sheet (sub-sheet) to a sheet, like Draw Sheet: name, file (.kicad_sch, "
                                "relative to the project directory or absolute), top-left corner and size in mm. An existing "
                                "file is loaded (a file already in the hierarchy is shared: another instance of it, e.g. two "
                                "channels); a new file is created empty with the parent's paper and written at once. Sheet "
                                "pins: 'pins', plus with sync_pins one per hierarchical label of the file that has none "
                                "(direction from the label; outputs right, others left). Hierarchy, page number and "
                                "annotation of the new instance's symbols are updated. 409 name taken on that sheet, 422 "
                                "recursion or not a schematic. Answers uuid, path, page, pins with positions (wire to them "
                                "or address them as SHEETNAME.PIN in sch_wire / sch_label_add), symbols of the file." ) + EDIT_NOTE,
                   nlohmann::json::parse( std::string( R"json({"type":"object","required":["name","file","x_mm","y_mm"],"properties":{
                        "name":{"type":"string","description":"sheet name, unique among the sheets of the parent (case-insensitive)"},
                        "file":{"type":"string","description":".kicad_sch, relative to the project directory or absolute"},
                        "x_mm":{"type":"number","description":"top-left corner"},"y_mm":{"type":"number"},
                        "width_mm":{"type":"number","default":50.8},"height_mm":{"type":"number","default":25.4},
                        "sheet":{"type":"string","description":"parent sheet path from sch_sheet_list; default current/root"},
                        "pins":{"type":"array","items":)json" ) + SCH_SHEET_PIN_SCHEMA + R"json(},
                        "sync_pins":{"type":"boolean","default":true,"description":"a pin for every hierarchical label of the file without one"},
                        "annotate":{"type":"boolean","default":true,"description":"renumber the new instance's symbols (first free numbers), as the editor's automatic annotation does; false keeps the file's references"}}})json" ),
                   false, h_sch_sheet_add, 120 );

KOPENAPI_REGISTER( "sch_sheet_pin_add",
                   std::string( "Add pins to a sheet symbol (sheet_path as sch_sheet_list prints it, the sheet's uuid, or "
                                "its name when unique), like Add Sheet Pin: name, direction, side, offset_mm (default the "
                                "next free 2.54 mm slot; the sheet grows when a side is full). create_labels also puts a "
                                "matching hierarchical label in the sheet's own file on a free spot (an existing one is "
                                "reused). 409 on a pin name the sheet already has. Answers every pin's position in mm "
                                "(on the grid: sch_wire to {x_mm, y_mm} or to SHEETNAME.PIN) and the label positions inside." ) + EDIT_NOTE,
                   nlohmann::json::parse( std::string( R"json({"type":"object","required":["sheet_path","pins"],"properties":{
                        "sheet_path":{"type":"string","description":"e.g. \"project/Mixer\" (sch_sheet_list path), or the sheet uuid"},
                        "pins":{"type":"array","items":)json" ) + SCH_SHEET_PIN_SCHEMA + R"json(},
                        "create_labels":{"type":"boolean","default":false,"description":"a hierarchical label per pin inside the sheet's file"}}})json" ),
                   false, h_sch_sheet_pin_add );

KOPENAPI_MARK_EDITING( "sch_symbol_add", "sch_symbol_update", "sch_items_move", "sch_item_delete", "sch_connect", "sch_wire", "sch_rewire", "sch_texts_settle", "sch_power_add", "sch_label_add", "sch_no_connect", "sch_annotate", "sch_lock", "sch_unlock", "sch_sheet_add", "sch_sheet_pin_add", "sch_blocks_arrange", "sch_page_set" );
KOPENAPI_MARK_CONCURRENT_READS( "sch_item_list" );
