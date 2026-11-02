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


#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <pad.h>
#include <dialogs/dialog_text_entry.h>
#include <microwave/microwave_tool.h>
#include <new_footprint.h>
#include <pcb_edit_frame.h>
#include <trigo.h>


FOOTPRINT* CreateMicrowaveBaseFootprint( BOARD* aBoard, const wxString& aValue, int aTextSize, int aPadCount )
{
    FOOTPRINT* footprint = NewFootprintFromDefaults( aBoard, aValue, FP_SMD );

    footprint->SetAttributes( FP_EXCLUDE_FROM_POS_FILES | FP_EXCLUDE_FROM_BOM );

    if( aTextSize > 0 )
    {
        footprint->Reference().SetTextSize( VECTOR2I( aTextSize, aTextSize ) );
        footprint->Reference().SetTextThickness( aTextSize / 5 );
        footprint->Value().SetTextSize( VECTOR2I( aTextSize, aTextSize ) );
        footprint->Value().SetTextThickness( aTextSize / 5 );
    }

    // Create 2 pads used in gaps and stubs.  The gap is between these 2 pads
    // the stub is the pad 2
    int pad_num = 1;
    int tw = aBoard->GetDesignSettings().GetCurrentTrackWidth();

    while( aPadCount-- )
    {
        PAD* pad = new PAD( footprint );
        pad->SetPadstackMode( PADSTACK::MODE::NORMAL );

        footprint->Add( pad, ADD_MODE::INSERT );

        pad->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( tw, tw ) );

        pad->SetPosition( footprint->GetPosition() );
        pad->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::RECTANGLE );
        pad->SetAttribute( PAD_ATTRIB::SMD );
        pad->SetLayerSet( { F_Cu } );

        pad->SetNumber( wxString::Format( wxT( "%d" ), pad_num ) );
        pad_num++;
    }

    return footprint;
}


FOOTPRINT* CreateMicrowaveFootprint( BOARD* aBoard, MICROWAVE_FOOTPRINT_SHAPE aShape, int aSize,
                                     const EDA_ANGLE& aArcAngle, wxString* aError )
{
    wxString  cmp_name;
    int       pad_count = 2;
    int       text_size = 0;      // 0 keeps the board default
    EDA_ANGLE angle = aArcAngle;

    switch( aShape )
    {
    case MICROWAVE_FOOTPRINT_SHAPE::GAP:
        cmp_name  = wxT( "muwave_gap" );
        text_size = aSize;
        break;

    case MICROWAVE_FOOTPRINT_SHAPE::STUB:
        cmp_name  = wxT( "muwave_stub" );
        text_size = aSize;
        break;

    case MICROWAVE_FOOTPRINT_SHAPE::STUB_ARC:
        cmp_name  = wxT( "muwave_arcstub" );
        pad_count = 1;
        break;

    default:
        if( aError )
            *aError = _( "Not a gap, stub or arc stub." );

        return nullptr;
    }

    if( aSize <= 0 )
    {
        if( aError )
            *aError = _( "The size must be positive." );

        return nullptr;
    }

    if( angle < ANGLE_0 )
        angle = -angle;

    if( angle > ANGLE_180 )
        angle = ANGLE_180;

    FOOTPRINT* footprint = CreateMicrowaveBaseFootprint( aBoard, cmp_name, text_size, pad_count );
    auto       it = footprint->Pads().begin();
    PAD*       pad = *it;

    switch( aShape )
    {
    case MICROWAVE_FOOTPRINT_SHAPE::GAP:
    {
        int offsetX = -( aSize + pad->GetSize( PADSTACK::ALL_LAYERS ).x ) / 2;

        pad->SetX( pad->GetPosition().x + offsetX );

        pad = *( it + 1 );

        pad->SetX( pad->GetPosition().x + offsetX + aSize + pad->GetSize( PADSTACK::ALL_LAYERS ).x );
        break;
    }

    case MICROWAVE_FOOTPRINT_SHAPE::STUB:
    {
        pad->SetNumber( wxT( "1" ) );
        int offsetY = -( aSize + pad->GetSize( PADSTACK::ALL_LAYERS ).y ) / 2;

        // the pads were inserted at the front, so the second one is the stub, pad 2
        pad = *( it + 1 );
        pad->SetNumber( wxT( "2" ) );
        pad->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( pad->GetSize( PADSTACK::ALL_LAYERS ).x, aSize ) );
        pad->SetY( pad->GetPosition().y + offsetY );
        break;
    }

    case MICROWAVE_FOOTPRINT_SHAPE::STUB_ARC:     // Arc Stub created by a polygonal approach:
    {
        pad->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CUSTOM );
        pad->SetAnchorPadShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::RECTANGLE );

        int numPoints = ( angle.AsDegrees() / 5.0 ) + 3;
        std::vector<VECTOR2I> polyPoints;
        polyPoints.reserve( numPoints );

        polyPoints.emplace_back( VECTOR2I( 0, 0 ) );

        EDA_ANGLE theta = -angle / 2;

        for( int ii = 1; ii < numPoints - 1; ii++ )
        {
            VECTOR2I pt( 0, -aSize );
            RotatePoint( &pt.x, &pt.y, theta );
            polyPoints.push_back( pt );

            theta += EDA_ANGLE( 5.0, DEGREES_T );

            if( theta > angle / 2 )
                theta = angle / 2;
        }

        // Close the polygon:
        polyPoints.push_back( polyPoints[0] );

        pad->AddPrimitivePoly( PADSTACK::ALL_LAYERS, polyPoints, 0, true ); // add a polygonal basic shape
        break;
    }

    default:
        break;
    }

    return footprint;
}


FOOTPRINT* MICROWAVE_TOOL::createFootprint( MICROWAVE_FOOTPRINT_SHAPE aFootprintShape )
{
    wxString  msg;
    EDA_ANGLE angle = ANGLE_0;

    PCB_EDIT_FRAME* editFrame = getEditFrame<PCB_EDIT_FRAME>();

    // Enter the size of the gap or stub
    int gap_size = editFrame->GetDesignSettings().GetCurrentTrackWidth();

    switch( aFootprintShape )
    {
    case MICROWAVE_FOOTPRINT_SHAPE::GAP:      msg = _( "Gap Size:" );              break;
    case MICROWAVE_FOOTPRINT_SHAPE::STUB:     msg = _( "Stub Size:" );             break;
    case MICROWAVE_FOOTPRINT_SHAPE::STUB_ARC: msg = _( "Arc Stub Radius Value:" ); break;
    default:                                  msg = wxT( "???" );                  break;
    }

    wxString             value = editFrame->StringFromValue( gap_size );
    WX_TEXT_ENTRY_DIALOG dlg( editFrame, msg, _( "Create Microwave Footprint" ), value );

    if( dlg.ShowQuasiModal() != wxID_OK )
        return nullptr; // canceled by user

    gap_size = editFrame->ValueFromString( dlg.GetValue() );

    if( aFootprintShape == MICROWAVE_FOOTPRINT_SHAPE::STUB_ARC )
    {
        msg = wxT( "0.0" );
        WX_TEXT_ENTRY_DIALOG angledlg( editFrame, _( "Angle in degrees:" ),
                                       _( "Create Microwave Footprint" ), msg );

        if( angledlg.ShowQuasiModal() != wxID_OK )
            return nullptr; // canceled by user

        double fval;

        if( !angledlg.GetValue().ToDouble( &fval ) )
        {
            editFrame->ShowInfoBarError( _( "Incorrect number, abort" ) );
            return nullptr;
        }

        angle = EDA_ANGLE( fval, DEGREES_T );
    }

    wxString   error;
    FOOTPRINT* footprint = CreateMicrowaveFootprint( board(), aFootprintShape, gap_size, angle, &error );

    if( !footprint )
    {
        editFrame->ShowInfoBarError( error );
        return nullptr;
    }

    editFrame->SetMsgPanel( footprint );

    // Update the footprint and board
    editFrame->OnModify();

    return footprint;
}
