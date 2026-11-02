/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2015 Jean-Pierre Charras, jp.charras at wanadoo.fr
 * Copyright (C) 2012 SoftPLC Corporation, Dick Hollenbeck <dick@softplc.com>
 * Copyright (C) 2015-2016 Wayne Stambaugh <stambaughw@gmail.com>
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

#include <confirm.h>
#include <widgets/unit_binder.h>
#include <pcb_edit_frame.h>
#include <dialog_shim.h>
#include <locale_io.h>
#include <filter_reader.h>
#include <dialogs/dialog_text_entry.h>
#include <board.h>
#include <footprint.h>
#include <pcb_shape.h>
#include <microwave/microwave_tool.h>
#include <pad.h>
#include <math/util.h>      // for KiROUND

#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/filedlg.h>
#include <wx/radiobox.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

static MICROWAVE_SHAPE_FILE   g_ShapeFile;
static VECTOR2I               g_ShapeSize;
static MICROWAVE_POLYGON_MODE g_PolyShapeMode = MICROWAVE_POLYGON_MODE::NORMAL;


enum id_mw_cmd {
    ID_READ_SHAPE_FILE = 1000
};


class MWAVE_POLYGONAL_SHAPE_DLG : public DIALOG_SHIM
{
public:
    MWAVE_POLYGONAL_SHAPE_DLG( PCB_EDIT_FRAME* parent, const wxPoint& pos );

    ~MWAVE_POLYGONAL_SHAPE_DLG()
    {
        delete m_sizeX;
        delete m_sizeY;
    };

    bool TransferDataFromWindow() override;

private:
    void OnCancelClick( wxCommandEvent& event );

    /// Read a shape description file (ReadMicrowaveShapeFile); its scale fills the size
    void ReadDataShapeDescr( wxCommandEvent& event );

    DECLARE_EVENT_TABLE()

    PCB_EDIT_FRAME*  m_frame;
    wxRadioBox*      m_shapeOptionCtrl;
    UNIT_BINDER*     m_sizeX;
    UNIT_BINDER*     m_sizeY;
    wxStaticText*    m_status;
};


BEGIN_EVENT_TABLE( MWAVE_POLYGONAL_SHAPE_DLG, DIALOG_SHIM )
    EVT_BUTTON( wxID_CANCEL, MWAVE_POLYGONAL_SHAPE_DLG::OnCancelClick )
    EVT_BUTTON( ID_READ_SHAPE_FILE, MWAVE_POLYGONAL_SHAPE_DLG::ReadDataShapeDescr )
END_EVENT_TABLE()


MWAVE_POLYGONAL_SHAPE_DLG::MWAVE_POLYGONAL_SHAPE_DLG( PCB_EDIT_FRAME* parent,
                                                      const wxPoint&  framepos ) :
    DIALOG_SHIM( parent, -1, _( "Complex Shape" ), framepos, wxDefaultSize,
                 wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER ),
    m_sizeX(),
    m_sizeY()
{
    m_frame = parent;

    g_ShapeFile = MICROWAVE_SHAPE_FILE();

    wxBoxSizer* mainBoxSizer = new wxBoxSizer( wxVERTICAL );
    SetSizer( mainBoxSizer );

    // Controls

    wxBoxSizer* topBoxSizer  = new wxBoxSizer( wxHORIZONTAL );
    mainBoxSizer->Add( topBoxSizer, 1, wxGROW | wxALL, 5 );

    wxString shapelist[] = { _( "Normal" ), _( "Symmetrical" ), _( "Mirrored" ) };

    m_shapeOptionCtrl = new wxRadioBox( this, -1, _( "Shape" ),
                                        wxDefaultPosition, wxDefaultSize, 3,
                                        shapelist, 1,
                                        wxRA_SPECIFY_COLS );
    topBoxSizer->Add( m_shapeOptionCtrl, 1, wxGROW | wxALL, 5 );

    auto sizeSizer = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, _( "Size" ) ),
                                           wxVERTICAL );
    wxBoxSizer* xSizer = new wxBoxSizer( wxHORIZONTAL );
    wxBoxSizer* ySizer = new wxBoxSizer( wxHORIZONTAL );

    topBoxSizer->Add( sizeSizer, 1, wxGROW | wxALL, 5 );
    sizeSizer->Add( xSizer, 0, 0, 5 );
    sizeSizer->Add( ySizer, 0, 0, 5 );

    wxStaticText* xLabel = new wxStaticText( this, -1, _( "X:" ) );
    wxTextCtrl*   xCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString );
    wxStaticText* xUnits = new wxStaticText( this, -1, _( "units" ) );

    xSizer->Add( xLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5 );
    xSizer->Add( xCtrl, 1, wxALIGN_CENTER_VERTICAL, 5 );
    xSizer->Add( xUnits, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5 );
    m_sizeX = new UNIT_BINDER( m_frame, xLabel, xCtrl, xUnits );

    wxStaticText* yLabel = new wxStaticText( this, -1, _( "Y:" ) );
    wxTextCtrl*   yCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString );
    wxStaticText* yUnits = new wxStaticText( this, -1, _( "units" ) );

    ySizer->Add( yLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5 );
    ySizer->Add( yCtrl, 1, wxALIGN_CENTER_VERTICAL, 5 );
    ySizer->Add( yUnits, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5 );
    m_sizeY = new UNIT_BINDER( m_frame, yLabel, yCtrl, yUnits );

    // Buttons

    wxBoxSizer* buttonsBoxSizer = new wxBoxSizer( wxHORIZONTAL );
    mainBoxSizer->Add( buttonsBoxSizer, 0, wxALL, 5 );

    wxButton* btn = new wxButton( this, ID_READ_SHAPE_FILE, _( "Read Shape Description File..." ) );
    buttonsBoxSizer->Add( btn, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, 10 );
    buttonsBoxSizer->AddStretchSpacer();

    m_status = new wxStaticText( this, wxID_ANY, wxEmptyString );
    mainBoxSizer->Insert( 1, m_status, 0, wxEXPAND | wxLEFT | wxRIGHT, 10 );

    wxStdDialogButtonSizer* sdbSizer = new wxStdDialogButtonSizer();
    buttonsBoxSizer->Add( sdbSizer, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5 );
    wxButton* sdbSizerOK = new wxButton( this, wxID_OK );
   	sdbSizer->AddButton( sdbSizerOK );
   	wxButton* sdbSizerCancel = new wxButton( this, wxID_CANCEL );
   	sdbSizer->AddButton( sdbSizerCancel );
   	sdbSizer->Realize();

    GetSizer()->SetSizeHints( this );
}


void MWAVE_POLYGONAL_SHAPE_DLG::OnCancelClick( wxCommandEvent& event )
{
    g_ShapeFile = MICROWAVE_SHAPE_FILE();
    event.Skip();
}


bool MWAVE_POLYGONAL_SHAPE_DLG::TransferDataFromWindow()
{
    if( !wxDialog::TransferDataFromWindow() )
        return false;

    g_ShapeSize.x = m_sizeX->GetValue();
    g_ShapeSize.y = m_sizeY->GetValue();
    g_PolyShapeMode = static_cast<MICROWAVE_POLYGON_MODE>( m_shapeOptionCtrl->GetSelection() );

    return true;
}


void MWAVE_POLYGONAL_SHAPE_DLG::ReadDataShapeDescr( wxCommandEvent& event )
{
    static wxString s_lastpath;       // To remember the last open path during a session
    wxString fullFileName;
    wxString mask = wxFileSelectorDefaultWildcardStr;

    fullFileName = wxFileSelector( _( "Shape Description File" ), s_lastpath,
                                   fullFileName, wxEmptyString, mask, wxFD_OPEN, this );

    if( fullFileName.IsEmpty() )
        return;

    wxFileName fn( fullFileName );
    s_lastpath = fn.GetPath();

    wxString error;

    if( !ReadMicrowaveShapeFile( fullFileName, g_ShapeFile, &error ) )
    {
        g_ShapeFile = MICROWAVE_SHAPE_FILE();
        m_status->SetLabel( error );
        return;
    }

    m_status->SetLabel( wxString::Format( _( "%d points read from %s." ), (int) g_ShapeFile.points.size(),
                                          fn.GetFullName() ) );
    m_sizeX->SetValue( KiROUND( g_ShapeFile.scale.x ) );
    m_sizeY->SetValue( KiROUND( g_ShapeFile.scale.y ) );
}


bool ReadMicrowaveShapeFile( const wxString& aPath, MICROWAVE_SHAPE_FILE& aShape, wxString* aError )
{
    FILE* File = wxFopen( aPath, wxT( "rt" ) );

    if( File == nullptr )
    {
        if( aError )
            *aError = wxString::Format( _( "File not found: %s" ), aPath );

        return false;
    }

    double unitconv = pcbIUScale.IU_PER_MM;

    aShape = MICROWAVE_SHAPE_FILE();

    FILE_LINE_READER fileReader( File, aPath );
    FILTER_READER reader( fileReader );

    LOCALE_IO   toggle;

    while( reader.ReadLine() )
    {
        char* Line = reader.Line();
        char* param1 = strtok( Line, " =\n\r" );
        char* param2 = strtok( nullptr, " \t\n\r" );

        if( !param1 )
            continue;

        if( strncasecmp( param1, "Unit", 4 ) == 0 && param2 )
        {
            if( strncasecmp( param2, "inch", 4 ) == 0 )
                unitconv = pcbIUScale.IU_PER_MILS*1000;

            if( strncasecmp( param2, "mm", 2 ) == 0 )
                unitconv = pcbIUScale.IU_PER_MM;
        }

        if( strncasecmp( param1, "$ENDCOORD", 8 ) == 0 )
            break;

        if( strncasecmp( param1, "$COORD", 6 ) == 0 )
        {
            while( reader.ReadLine() )
            {
                Line = reader.Line();
                param1 = strtok( Line, " \t\n\r" );
                param2 = strtok( nullptr, " \t\n\r" );

                if( !param1 )
                    continue;

                if( strncasecmp( param1, "$ENDCOORD", 8 ) == 0 )
                    break;

                aShape.points.emplace_back( atof( param1 ), param2 ? atof( param2 ) : 0.0 );
            }
        }

        if( strncasecmp( Line, "XScale", 6 ) == 0 && param2 )
            aShape.scale.x = atof( param2 );

        if( strncasecmp( Line, "YScale", 6 ) == 0 && param2 )
            aShape.scale.y = atof( param2 );
    }

    aShape.scale.x *= unitconv;
    aShape.scale.y *= unitconv;

    if( aShape.points.empty() )
    {
        if( aError )
            *aError = _( "Shape has no points." );

        return false;
    }

    return true;
}


FOOTPRINT* CreateMicrowavePolygon( BOARD* aBoard, const std::vector<VECTOR2D>& aPoints, const VECTOR2I& aSize,
                                   MICROWAVE_POLYGON_MODE aMode, wxString* aError )
{
    VECTOR2I size = aSize;

    if( aMode == MICROWAVE_POLYGON_MODE::MIRRORED )
        size.y = -size.y;

    if( size.x == 0 || size.y == 0 )
    {
        if( aError )
            *aError = _( "Shape has a null size." );

        return nullptr;
    }

    if( aPoints.empty() )
    {
        if( aError )
            *aError = _( "Shape has no points." );

        return nullptr;
    }

    // Create a footprint with 2 pads, orientation = 0, pos 0
    FOOTPRINT* footprint = CreateMicrowaveBaseFootprint( aBoard, wxT( "muwave_polygon" ), 0, 2 );

    // We try to place the footprint anchor to the middle of the shape len
    VECTOR2I offset;
    offset.x = -size.x / 2;

    auto it = footprint->Pads().begin();

    PAD* pad1 = *it;
    pad1->SetX( offset.x );

    PAD* pad2 = *( ++it );
    pad2->SetX( offset.x + size.x );

    // Add a polygonal edge (corners will be added later) on copper layer
    PCB_SHAPE* shape = new PCB_SHAPE( footprint, SHAPE_T::POLY );
    shape->SetFilled( true );
    shape->SetLayer( F_Cu );

    footprint->Add( shape, ADD_MODE::INSERT );

    std::vector<VECTOR2I> polyPoints;
    polyPoints.reserve( aPoints.size() + 2 );

    // Init start point coord:
    polyPoints.emplace_back( VECTOR2I( offset.x, 0 ) );

    VECTOR2I last_coordinate;

    for( const VECTOR2D& pt : aPoints )
    {
        last_coordinate.x = KiROUND( pt.x * size.x );
        last_coordinate.y = -KiROUND( pt.y * size.y );
        last_coordinate += offset;
        polyPoints.push_back( last_coordinate );
    }

    // finish the polygonal shape
    if( last_coordinate.y != 0 )
        polyPoints.emplace_back( VECTOR2I( last_coordinate.x, 0 ) );

    if( aMode == MICROWAVE_POLYGON_MODE::SYMMETRICAL )
    {
        // add the outline mirrored about the X axis
        for( int ndx = (int) polyPoints.size() - 1; ndx >= 0; --ndx )
        {
            VECTOR2I pt = polyPoints[ndx];
            pt.y = -pt.y;
            polyPoints.push_back( pt );
        }
    }

    shape->SetPolyPoints( polyPoints );

    // Set the polygon outline thickness to 0, only the polygonal shape is filled
    // without extra thickness.
    shape->SetStroke( STROKE_PARAMS( 0, LINE_STYLE::SOLID ) );

    return footprint;
}


FOOTPRINT* MICROWAVE_TOOL::createPolygonShape()
{
    PCB_EDIT_FRAME& editFrame  = *getEditFrame<PCB_EDIT_FRAME>();

    MWAVE_POLYGONAL_SHAPE_DLG dlg( &editFrame, wxDefaultPosition );

    if( dlg.ShowModal() != wxID_OK )
    {
        g_ShapeFile = MICROWAVE_SHAPE_FILE();
        return nullptr;
    }

    wxString   error;
    FOOTPRINT* footprint = CreateMicrowavePolygon( board(), g_ShapeFile.points, g_ShapeSize, g_PolyShapeMode,
                                                   &error );
    g_ShapeFile = MICROWAVE_SHAPE_FILE();

    if( !footprint )
    {
        editFrame.ShowInfoBarError( error );
        return nullptr;
    }

    editFrame.SetMsgPanel( footprint );
    editFrame.OnModify();
    return footprint;
}
