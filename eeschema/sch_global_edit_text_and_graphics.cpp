/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <sch_global_edit_text_and_graphics.h>

#include <connectivity/conn_facade.h>
#include <font/font.h>
#include <sch_commit.h>
#include <sch_junction.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_screen.h>
#include <sch_shape.h>
#include <sch_sheet.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <string_utils.h>


namespace
{

class EDITOR
{
public:
    EDITOR( SCH_COMMIT& aCommit, const SCH_GLOBAL_EDIT_OPTIONS& aOpt ) :
            m_commit( aCommit ),
            m_opt( aOpt )
    {
    }

    void Visit( const SCH_SHEET_PATH& aSheetPath, SCH_ITEM* aItem );

    std::vector<SCH_GLOBAL_EDIT_CHANGE> m_changed;

private:
    void process( const SCH_SHEET_PATH& aSheetPath, SCH_ITEM* aItem );

    bool fieldNameMatches( const wxString& aName ) const
    {
        return m_opt.fieldNameFilter.IsEmpty() || WildCompareString( m_opt.fieldNameFilter, aName, false );
    }

    SCH_COMMIT&                    m_commit;
    const SCH_GLOBAL_EDIT_OPTIONS& m_opt;
};


void EDITOR::process( const SCH_SHEET_PATH& aSheetPath, SCH_ITEM* aItem )
{
    if( m_opt.selectedOnly )
    {
        if( !aItem->IsSelected() && ( !aItem->GetParent() || !aItem->GetParent()->IsSelected() ) )
            return;
    }

    m_commit.Modify( aItem, aSheetPath.LastScreen() );
    m_changed.push_back( { aItem, aSheetPath } );

    if( EDA_TEXT* eda_text = dynamic_cast<EDA_TEXT*>( aItem ) )
    {
        if( m_opt.textSize )
            eda_text->SetTextSize( VECTOR2I( *m_opt.textSize, *m_opt.textSize ) );

        if( m_opt.textColor )
            eda_text->SetTextColor( *m_opt.textColor );

        if( m_opt.hAlign )
        {
            GR_TEXT_H_ALIGN_T hAlign = *m_opt.hAlign;
            SCH_SYMBOL*       parentSymbol = dynamic_cast<SCH_SYMBOL*>( aItem->GetParent() );

            if( parentSymbol && parentSymbol->GetTransform().x1 < 0 )
            {
                if( hAlign == GR_TEXT_H_ALIGN_LEFT )
                    hAlign = GR_TEXT_H_ALIGN_RIGHT;
                else if( hAlign == GR_TEXT_H_ALIGN_RIGHT )
                    hAlign = GR_TEXT_H_ALIGN_LEFT;
            }

            eda_text->SetHorizJustify( hAlign );
        }

        if( m_opt.vAlign )
        {
            GR_TEXT_V_ALIGN_T vAlign = *m_opt.vAlign;
            SCH_SYMBOL*       parentSymbol = dynamic_cast<SCH_SYMBOL*>( aItem->GetParent() );

            if( parentSymbol && parentSymbol->GetTransform().y1 < 0 )
            {
                if( vAlign == GR_TEXT_V_ALIGN_TOP )
                    vAlign = GR_TEXT_V_ALIGN_BOTTOM;
                else if( vAlign == GR_TEXT_V_ALIGN_BOTTOM )
                    vAlign = GR_TEXT_V_ALIGN_TOP;
            }

            eda_text->SetVertJustify( vAlign );
        }

        if( m_opt.italic )
            eda_text->SetItalic( *m_opt.italic );

        // Must come after SetTextSize()
        if( m_opt.bold )
            eda_text->SetBold( *m_opt.bold );

        // Must come after SetBold() & SetItalic()
        if( m_opt.font )
        {
            eda_text->SetFont( m_opt.font( eda_text->IsBold(), eda_text->IsItalic() ) );
        }
        else if( m_opt.italic || m_opt.bold )
        {
            if( !eda_text->GetFontName().IsEmpty() )
            {
                eda_text->SetFont( KIFONT::FONT::GetFont( eda_text->GetFontName(), eda_text->IsBold(),
                                                          eda_text->IsItalic() ) );
            }
        }
    }

    if( SCH_LABEL_BASE* sch_label = dynamic_cast<SCH_LABEL_BASE*>( aItem ) )
    {
        if( m_opt.labelOrientation )
            sch_label->SetSpinStyle( *m_opt.labelOrientation );
    }

    if( SCH_FIELD* sch_field = dynamic_cast<SCH_FIELD*>( aItem ) )
    {
        if( m_opt.visible )
            sch_field->SetVisible( *m_opt.visible );

        if( m_opt.showFieldNames )
            sch_field->SetNameShown( *m_opt.showFieldNames );
    }

    if( aItem->HasLineStroke() )
    {
        STROKE_PARAMS stroke = aItem->GetStroke();

        if( m_opt.lineWidth )
            stroke.SetWidth( *m_opt.lineWidth );

        if( m_opt.lineStyle )
            stroke.SetLineStyle( *m_opt.lineStyle );

        if( m_opt.lineColor )
            stroke.SetColor( *m_opt.lineColor );

        aItem->SetStroke( stroke );
    }

    if( SCH_SHAPE* shape = dynamic_cast<SCH_SHAPE*>( aItem ) )
    {
        if( m_opt.fillColor )
        {
            shape->SetFillColor( *m_opt.fillColor );

            if( *m_opt.fillColor == KIGFX::COLOR4D::UNSPECIFIED )
                shape->SetFillMode( FILL_T::NO_FILL );
            else
                shape->SetFillMode( FILL_T::FILLED_WITH_COLOR );
        }
    }

    if( SCH_JUNCTION* junction = dynamic_cast<SCH_JUNCTION*>( aItem ) )
    {
        if( m_opt.junctionSize )
            junction->SetDiameter( *m_opt.junctionSize );

        if( m_opt.junctionColor )
            junction->SetColor( *m_opt.junctionColor );
    }
}


void EDITOR::Visit( const SCH_SHEET_PATH& aSheetPath, SCH_ITEM* aItem )
{
    if( !m_opt.netFilter.IsEmpty() )
    {
        const std::optional<wxString> netName = aItem->GetConnectionName( &aSheetPath );

        if( !netName || !WildCompareString( m_opt.netFilter, *netName, false ) )
            return;
    }

    if( !m_opt.referenceFilter.IsEmpty() && aItem->Type() == SCH_SYMBOL_T )
    {
        wxString ref = static_cast<SCH_SYMBOL*>( aItem )->GetRef( &aSheetPath );

        if( !WildCompareString( m_opt.referenceFilter, ref, false ) )
            return;
    }

    if( !m_opt.symbolFilter.IsEmpty() && aItem->Type() == SCH_SYMBOL_T )
    {
        wxString id = UnescapeString( static_cast<SCH_SYMBOL*>( aItem )->GetLibId().Format() );

        if( !WildCompareString( m_opt.symbolFilter, id, false ) )
            return;
    }

    if( m_opt.powerSymbols && aItem->Type() == SCH_SYMBOL_T )
    {
        if( static_cast<SCH_SYMBOL*>( aItem )->IsPower() != *m_opt.powerSymbols )
            return;
    }

    static const std::vector<KICAD_T> wireLabelTypes = { SCH_LABEL_LOCATE_WIRE_T };
    static const std::vector<KICAD_T> busLabelTypes = { SCH_LABEL_LOCATE_BUS_T };

    switch( aItem->Type() )
    {
    case SCH_SYMBOL_T:
    {
        SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( aItem );

        if( m_opt.references )
            process( aSheetPath, symbol->GetField( FIELD_T::REFERENCE ) );

        if( m_opt.values )
            process( aSheetPath, symbol->GetField( FIELD_T::VALUE ) );

        if( m_opt.otherFields )
        {
            for( SCH_FIELD& field : symbol->GetFields() )
            {
                if( field.GetId() == FIELD_T::REFERENCE || field.GetId() == FIELD_T::VALUE )
                    continue;

                if( fieldNameMatches( field.GetName() ) )
                    process( aSheetPath, &field );
            }
        }

        break;
    }

    case SCH_SHEET_T:
    {
        SCH_SHEET* sheet = static_cast<SCH_SHEET*>( aItem );

        if( m_opt.sheetTitles )
            process( aSheetPath, sheet->GetField( FIELD_T::SHEET_NAME ) );

        if( m_opt.sheetFields )
        {
            for( SCH_FIELD& field : sheet->GetFields() )
            {
                if( field.GetId() == FIELD_T::SHEET_NAME )
                    continue;

                if( fieldNameMatches( field.GetName() ) )
                    process( aSheetPath, &field );
            }
        }

        if( m_opt.sheetBorders )
        {
            if( !m_opt.selectedOnly || sheet->IsSelected() )
            {
                m_commit.Modify( sheet, aSheetPath.LastScreen() );
                m_changed.push_back( { sheet, aSheetPath } );

                if( m_opt.lineWidth )
                    sheet->SetBorderWidth( *m_opt.lineWidth );

                if( m_opt.lineColor )
                    sheet->SetBorderColor( *m_opt.lineColor );

                if( m_opt.fillColor )
                    sheet->SetBackgroundColor( *m_opt.fillColor );
            }
        }

        if( m_opt.sheetPins )
        {
            for( SCH_SHEET_PIN* pin : sheet->GetPins() )
                process( aSheetPath, pin );
        }

        break;
    }

    case SCH_LINE_T:
        if( m_opt.textAndGraphics && aItem->GetLayer() == LAYER_NOTES )
            process( aSheetPath, aItem );
        else if( m_opt.wires && aItem->GetLayer() == LAYER_WIRE )
            process( aSheetPath, aItem );
        else if( m_opt.buses && aItem->GetLayer() == LAYER_BUS )
            process( aSheetPath, aItem );

        break;

    case SCH_LABEL_T:
    case SCH_GLOBAL_LABEL_T:
    case SCH_HIER_LABEL_T:
    case SCH_DIRECTIVE_LABEL_T:
        if( m_opt.wires && aItem->IsType( wireLabelTypes ) )
            process( aSheetPath, aItem );

        if( m_opt.buses && aItem->IsType( busLabelTypes ) )
            process( aSheetPath, aItem );

        if( m_opt.globalLabels && aItem->Type() == SCH_GLOBAL_LABEL_T )
            process( aSheetPath, aItem );

        if( m_opt.hierLabels && aItem->Type() == SCH_HIER_LABEL_T )
            process( aSheetPath, aItem );

        if( m_opt.labelFields )
        {
            for( SCH_FIELD& field : static_cast<SCH_LABEL_BASE*>( aItem )->GetFields() )
            {
                if( fieldNameMatches( field.GetName() ) )
                    process( aSheetPath, &field );
            }
        }

        break;

    case SCH_JUNCTION_T:
    {
        SCH_JUNCTION* junction = static_cast<SCH_JUNCTION*>( aItem );

        for( SCH_ITEM* item : junction->ConnectedItems( aSheetPath ) )
        {
            if( item->GetLayer() == LAYER_BUS && m_opt.buses )
            {
                process( aSheetPath, aItem );
                break;
            }
            else if( item->GetLayer() == LAYER_WIRE && m_opt.wires )
            {
                process( aSheetPath, aItem );
                break;
            }
        }

        break;
    }

    case SCH_TEXT_T:
    case SCH_TEXTBOX_T:
    case SCH_SHAPE_T:
        if( m_opt.textAndGraphics )
            process( aSheetPath, aItem );

        break;

    default:
        break;
    }
}

} // namespace


std::vector<SCH_GLOBAL_EDIT_CHANGE>
SchGlobalEditTextAndGraphics( SCHEMATIC& aSchematic, SCH_COMMIT& aCommit, const SCH_GLOBAL_EDIT_OPTIONS& aOptions,
                              const std::function<void( const SCH_SHEET_PATH& )>& aOnSheet )
{
    EDITOR editor( aCommit, aOptions );

    // Each staged edit hides published nets on its screen; filter the rest by pre-edit nets
    SCH_CONNECTIVITY::PUBLICATION_HOLD hold( aSchematic.Connectivity() );

    for( const SCH_SHEET_PATH& sheetPath : aSchematic.Hierarchy() )
    {
        SCH_SCREEN* screen = sheetPath.LastScreen();

        if( !screen )
            continue;

        if( aOnSheet )
            aOnSheet( sheetPath );

        for( SCH_ITEM* item : screen->Items() )
            editor.Visit( sheetPath, item );
    }

    return std::move( editor.m_changed );
}
