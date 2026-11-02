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

#include <global_edit_text_and_graphics.h>

#include <board.h>
#include <board_commit.h>
#include <board_design_settings.h>
#include <embedded_files.h>
#include <font/font.h>
#include <footprint.h>
#include <pcb_barcode.h>
#include <pcb_dimension.h>
#include <pcb_field.h>
#include <pcb_group.h>
#include <pcb_shape.h>
#include <pcb_text.h>
#include <string_utils.h>


namespace
{

void processItem( BOARD& aBoard, BOARD_COMMIT& aCommit, const BOARD_DESIGN_SETTINGS& aSettings,
                  const GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS& aOpt, BOARD_ITEM* aItem )
{
    aCommit.Modify( aItem );

    PCB_FIELD*          field = dynamic_cast<PCB_FIELD*>( aItem );
    EDA_TEXT*           text = dynamic_cast<EDA_TEXT*>( aItem );
    PCB_SHAPE*          shape = dynamic_cast<PCB_SHAPE*>( aItem );
    PCB_DIMENSION_BASE* dimension = dynamic_cast<PCB_DIMENSION_BASE*>( aItem );
    PCB_BARCODE*        barcode = dynamic_cast<PCB_BARCODE*>( aItem );
    FOOTPRINT*          parentFP = aItem->GetParentFootprint();

    if( aOpt.setToLayerDefaults )
    {
        aItem->StyleFromSettings( aSettings, false );
        return;
    }

    if( aOpt.layer )
        aItem->SetLayer( *aOpt.layer );

    if( text )
    {
        if( aOpt.textWidth )
            text->SetTextSize( VECTOR2I( *aOpt.textWidth, text->GetTextSize().y ) );

        if( aOpt.textHeight )
            text->SetTextSize( VECTOR2I( text->GetTextSize().x, *aOpt.textHeight ) );

        if( aOpt.autoTextThickness )
            text->SetAutoThickness( true );
        else if( aOpt.textThickness )
            text->SetTextThickness( *aOpt.textThickness );

        // Must be after SetTextSize()
        if( aOpt.bold )
            text->SetBold( *aOpt.bold );

        if( aOpt.italic )
            text->SetItalic( *aOpt.italic );

        // Must come after setting bold & italic
        if( aOpt.font )
        {
            text->SetFont( aOpt.font( text->IsBold(), text->IsItalic() ) );
        }
        else if( aOpt.italic || aOpt.bold )
        {
            if( !text->GetFontName().IsEmpty() )
            {
                text->SetFont( KIFONT::FONT::GetFont( text->GetFontName(), text->IsBold(), text->IsItalic(),
                                                      aBoard.GetEmbeddedFiles()->GetFontFiles() ) );
            }
        }

        if( parentFP )
        {
            if( aOpt.keepUpright )
                text->SetKeepUpright( *aOpt.keepUpright );

            if( aOpt.centerOnFootprint )
                text->SetTextPos( aItem->GetParent()->GetCenter() );
        }
    }

    if( barcode )
    {
        if( aOpt.textHeight )
            barcode->SetTextSize( *aOpt.textHeight );
        else if( aOpt.textWidth )
            barcode->SetTextSize( *aOpt.textWidth );
    }

    if( field && aOpt.visible )
        field->SetVisible( *aOpt.visible );

    if( aOpt.lineWidth )
    {
        if( shape )
        {
            STROKE_PARAMS stroke = shape->GetStroke();
            stroke.SetWidth( *aOpt.lineWidth );
            shape->SetStroke( stroke );
        }

        if( dimension )
            dimension->SetLineThickness( *aOpt.lineWidth );
    }
}


bool passesFilters( const GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS& aOpt, BOARD_ITEM* aItem, bool aBoardEditor )
{
    if( aOpt.selectedOnly )
    {
        EDA_ITEM* candidate = aItem;

        if( !candidate->IsSelected() )
        {
            if( candidate->GetParent() && candidate->GetParent()->Type() == PCB_FOOTPRINT_T )
                candidate = candidate->GetParent();
        }

        if( !candidate->IsSelected() )
        {
            candidate = ( candidate->GetParentGroup() ? candidate->GetParentGroup()->AsEdaItem() : nullptr );

            while( candidate && !candidate->IsSelected() && candidate->GetParentGroup() )
                candidate = candidate->GetParentGroup()->AsEdaItem();

            if( !candidate || !candidate->IsSelected() )
                return false;
        }
    }

    if( aOpt.layerFilter && aItem->GetLayer() != *aOpt.layerFilter )
        return false;

    if( aBoardEditor )
    {
        if( !aOpt.referenceFilter.IsEmpty() )
        {
            if( FOOTPRINT* fp = aItem->GetParentFootprint() )
            {
                if( !WildCompareString( aOpt.referenceFilter, fp->GetReference(), false ) )
                    return false;
            }
        }

        if( !aOpt.footprintFilter.IsEmpty() )
        {
            if( FOOTPRINT* fp = aItem->GetParentFootprint() )
            {
                if( !WildCompareString( aOpt.footprintFilter, fp->GetFPID().Format(), false ) )
                    return false;
            }
        }
    }

    return true;
}

} // namespace


std::vector<BOARD_ITEM*> GlobalEditTextAndGraphics( BOARD& aBoard, BOARD_COMMIT& aCommit,
                                                    const BOARD_DESIGN_SETTINGS& aSettings,
                                                    const GLOBAL_EDIT_TEXT_AND_GRAPHICS_OPTIONS& aOpt,
                                                    bool aBoardEditor )
{
    std::vector<BOARD_ITEM*> changed;

    auto visitItem =
            [&]( BOARD_ITEM* aItem )
            {
                if( !passesFilters( aOpt, aItem, aBoardEditor ) )
                    return;

                processItem( aBoard, aCommit, aSettings, aOpt, aItem );
                changed.push_back( aItem );
            };

    // Go through the footprints
    for( FOOTPRINT* fp : aBoard.Footprints() )
    {
        if( aOpt.references )
            visitItem( &fp->Reference() );

        if( aOpt.values )
            visitItem( &fp->Value() );

        for( PCB_FIELD* field : fp->GetFields() )
        {
            wxCHECK2( field, continue );

            if( field->IsReference() )
                continue;

            if( field->IsValue() )
                continue;

            if( aOpt.otherFootprintFields )
                visitItem( field );
            else if( aOpt.references && field->GetText() == wxT( "${REFERENCE}" ) )
                visitItem( field );
            else if( aOpt.values && field->GetText() == wxT( "${VALUE}" ) )
                visitItem( field );
        }

        // Go through all other footprint items
        for( BOARD_ITEM* boardItem : fp->GraphicalItems() )
        {
            KICAD_T itemType = boardItem->Type();

            if( itemType == PCB_TEXT_T || itemType == PCB_TEXTBOX_T )
            {
                EDA_TEXT* textItem = dynamic_cast<EDA_TEXT*>( boardItem );

                if( aOpt.footprintTexts )
                    visitItem( boardItem );
                else if( aOpt.references && textItem->GetText() == wxT( "${REFERENCE}" ) )
                    visitItem( boardItem );
                else if( aOpt.values && textItem->GetText() == wxT( "${VALUE}" ) )
                    visitItem( boardItem );
            }
            else if( itemType == PCB_TABLE_T )
            {
                boardItem->RunOnChildren(
                        [&]( BOARD_ITEM* child )
                        {
                            if( child->Type() == PCB_TABLECELL_T && aOpt.footprintTexts )
                                visitItem( child );
                        },
                        RECURSE_MODE::NO_RECURSE );
            }
            else if( BaseType( itemType ) == PCB_DIMENSION_T )
            {
                if( aOpt.footprintDimensions )
                    visitItem( boardItem );
            }
            else if( itemType == PCB_SHAPE_T || itemType == PCB_BARCODE_T )
            {
                if( aOpt.footprintGraphics )
                    visitItem( boardItem );
            }
        }
    }

    if( aBoardEditor )
    {
        // Go through the PCB text & graphic items
        for( BOARD_ITEM* boardItem : aBoard.Drawings() )
        {
            KICAD_T itemType = boardItem->Type();

            if( itemType == PCB_TEXT_T || itemType == PCB_TEXTBOX_T )
            {
                if( aOpt.boardTexts )
                    visitItem( boardItem );
            }
            else if( itemType == PCB_TABLE_T )
            {
                boardItem->RunOnChildren(
                        [&]( BOARD_ITEM* child )
                        {
                            if( child->Type() == PCB_TABLECELL_T && aOpt.boardTexts )
                                visitItem( child );
                        },
                        RECURSE_MODE::NO_RECURSE );
            }
            else if( BaseType( itemType ) == PCB_DIMENSION_T )
            {
                if( aOpt.boardDimensions )
                    visitItem( boardItem );
            }
            else if( itemType == PCB_SHAPE_T || itemType == PCB_BARCODE_T )
            {
                if( aOpt.boardGraphics )
                    visitItem( boardItem );
            }
        }
    }

    return changed;
}
