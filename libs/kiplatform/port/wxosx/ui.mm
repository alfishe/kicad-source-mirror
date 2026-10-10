/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2020 Ian McInerney <Ian.S.McInerney at ieee.org>
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <kiplatform/ui.h>

#import <Cocoa/Cocoa.h>

#include <vector>

#include <wx/dialog.h>
#include <wx/image.h>
#include <wx/nonownedwnd.h>
#include <wx/toplevel.h>
#include <wx/button.h>
#include <wx/settings.h>


bool KIPLATFORM::UI::IsDarkTheme()
{
    // Disabled for now because it appears that the wxWidgets event goes out before the
    // NSAppearance name has been updated
#ifdef NOTYET
    // It appears the wxWidgets event goes out before the NSAppearance name has been updated
    NSString *appearanceName = [[NSAppearance currentAppearance] name];
    return !![appearanceName containsString:@"Dark"];
#else
    wxColour bg = wxSystemSettings::GetColour( wxSYS_COLOUR_WINDOW );

    // Weighted W3C formula
    double brightness = ( bg.Red() / 255.0 ) * 0.299 +
                        ( bg.Green() / 255.0 ) * 0.587 +
                        ( bg.Blue() / 255.0 ) * 0.117;

    return brightness < 0.5;
#endif
}


wxColour KIPLATFORM::UI::GetDialogBGColour()
{
     wxColor bg = wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE );

    if( KIPLATFORM::UI::IsDarkTheme() )
        bg = bg.ChangeLightness( 80 );
    else
        bg = bg.ChangeLightness( 160 );

    return bg;
}


void KIPLATFORM::UI::GetInfoBarColours( wxColour& aFGColour, wxColour& aBGColour )
{
    aFGColour = wxSystemSettings::GetColour( wxSYS_COLOUR_INFOTEXT );

    // wxWidgets hard-codes wxSYS_COLOUR_INFOBK to { 0xFF, 0xFF, 0xD3 } on Mac.
    if( KIPLATFORM::UI::IsDarkTheme() )
        aBGColour = wxColour( 28, 27, 20 );
    else
        aBGColour = wxColour( 255, 249, 189 );
}


void KIPLATFORM::UI::ForceFocus( wxWindow* aWindow )
{
    // On OSX we need to forcefully give the focus to the window
    [[aWindow->GetHandle() window] makeFirstResponder: aWindow->GetHandle()];
}


bool KIPLATFORM::UI::IsWindowActive( wxWindow* aWindow )
{
    // Just always return true
    return true;
}


void KIPLATFORM::UI::EnsureVisible( wxWindow* aWindow )
{
    NSView* view = (NSView*)aWindow->GetHandle();
    if( view )
    {
        NSWindow* nsWindow = [view window];
        if( nsWindow )
        {
            [nsWindow setCollectionBehavior:
                NSWindowCollectionBehaviorCanJoinAllSpaces];
        }
    }
}


void KIPLATFORM::UI::StabilizeWindowPosition( wxWindow* aWindow )
{
    // Not needed on this platform
}


void KIPLATFORM::UI::ReparentWindow( wxNonOwnedWindow* aWindow, wxTopLevelWindow* aParent )
{
    NSWindow* parentWindow = aParent->GetWXWindow();
    NSWindow* theWindow    = aWindow->GetWXWindow();

    if( parentWindow && theWindow )
        [parentWindow addChildWindow:theWindow ordered:NSWindowAbove];
}


void KIPLATFORM::UI::ReparentModal( wxNonOwnedWindow* aWindow )
{
    // Quietly return if no parent is found
    if( wxTopLevelWindow* parent = static_cast<wxTopLevelWindow*>( wxGetTopLevelParent( aWindow->GetParent() ) ) )
        ReparentWindow( aWindow, parent );
}


void KIPLATFORM::UI::FixupCancelButtonCmdKeyCollision( wxWindow *aWindow )
{
    wxButton* button = dynamic_cast<wxButton*>( wxWindow::FindWindowById( wxID_CANCEL, aWindow ) );

    if( button )
    {
        static const wxString placeholder = wxT( "{amp}" );

        wxString buttonLabel = button->GetLabel();
        buttonLabel.Replace( wxT( "&&" ), placeholder );
        buttonLabel.Replace( wxT( "&" ), wxEmptyString );
        buttonLabel.Replace( placeholder, wxT( "&" ) );
        button->SetLabel( buttonLabel );
    }
}


bool KIPLATFORM::UI::IsStockCursorOk( wxStockCursor aCursor )
{
    switch( aCursor )
    {
    case wxCURSOR_SIZING:
    case wxCURSOR_BULLSEYE:
    case wxCURSOR_HAND:
    case wxCURSOR_ARROW:
        return true;
    default:
        return false;
    }
}


void KIPLATFORM::UI::LargeChoiceBoxHack( wxChoice* aChoice )
{
    // Not implemented
}


void KIPLATFORM::UI::EllipsizeChoiceBox( wxChoice* aChoice )
{
    // Not implemented
}


double KIPLATFORM::UI::GetPixelScaleFactor( const wxWindow* aWindow )
{
    return aWindow->GetContentScaleFactor();
}


double KIPLATFORM::UI::GetContentScaleFactor( const wxWindow* aWindow )
{
    // Native GUI resolution on Retina displays
    return GetPixelScaleFactor( aWindow ) / 2.0;
}


wxSize KIPLATFORM::UI::GetUnobscuredSize( const wxWindow* aWindow )
{
    return wxSize( aWindow->GetSize().GetX() - wxSystemSettings::GetMetric( wxSYS_VSCROLL_X ),
                   aWindow->GetSize().GetY() - wxSystemSettings::GetMetric( wxSYS_HSCROLL_Y ) );
}


void KIPLATFORM::UI::SetOverlayScrolling( const wxWindow* aWindow, bool overlay )
{
    // Not implemented
}


bool KIPLATFORM::UI::AllowIconsInMenus()
{
    return true;
}


bool KIPLATFORM::UI::SupportsKeyState()
{
    return true;
}


wxPoint KIPLATFORM::UI::GetMousePosition()
{
    // wxOSX flips the pointer against screen 0, which raises NSRangeException with no window
    // server attached
    if( [[NSScreen screens] count] == 0 )
        return wxPoint( 0, 0 );

    return wxGetMousePosition();
}


bool KIPLATFORM::UI::WarpPointer( wxWindow* aWindow, int aX, int aY )
{
    aWindow->WarpPointer( aX, aY );
    return true;
}


void KIPLATFORM::UI::ImmControl( wxWindow* aWindow, bool aEnable )
{
}


void KIPLATFORM::UI::ImeNotifyCancelComposition( wxWindow* aWindow )
{
}


bool KIPLATFORM::UI::InfiniteDragPrepareWindow( wxWindow* aWindow )
{
    return true;
}


void KIPLATFORM::UI::InfiniteDragReleaseWindow()
{
    // Not needed on this platform
}


void KIPLATFORM::UI::SetFloatLevel( wxWindow* aWindow )
{
    // On OSX we need to forcefully give the focus to the window
    [[aWindow->GetHandle() window] setLevel:NSFloatingWindowLevel];
}

void KIPLATFORM::UI::ReleaseChildWindow( wxNonOwnedWindow* aWindow )
{
    if( wxTopLevelWindow* parent = static_cast<wxTopLevelWindow*>(
            wxGetTopLevelParent( aWindow->GetParent() ) ) )
    {
        NSWindow* parentWindow = parent->GetWXWindow();
        NSWindow* theWindow = aWindow->GetWXWindow();

        if( parentWindow && theWindow )
        {
            [parentWindow removeChildWindow:theWindow];
            [theWindow setLevel:NSFloatingWindowLevel];
        }
    }
}


void KIPLATFORM::UI::AllowNetworkFileSystems( wxDialog* aDialog )
{
    // Not needed on macOS - file dialogs show network filesystems by default
}


void KIPLATFORM::UI::CancelPendingScroll( wxDataViewCtrl* aCtrl )
{
    // Not needed on this platform; the native control does not race on model resets.
}


void KIPLATFORM::UI::SetWMClass( wxWindow* aWindow, const wxString& aClass )
{
    // Not applicable on macOS; the bundle identifier provides the application identity.
}


bool KIPLATFORM::UI::CaptureWindow( wxWindow* aWindow, wxImage& aImage )
{
    if( !aWindow )
        return false;

    NSView* view = (NSView*) aWindow->GetHandle();

    // A top-level window: its whole content view (toolbars, panels, canvas)
    if( aWindow->IsTopLevel() && [view window] )
        view = [[view window] contentView];

    if( !view )
        return false;

    const NSRect bounds = [view bounds];
    NSBitmapImageRep* rep = [view bitmapImageRepForCachingDisplayInRect:bounds];

    if( !rep )
        return false;

    [view cacheDisplayInRect:bounds toBitmapImageRep:rep];

    CGImageRef cg = [rep CGImage];

    if( !cg )
        return false;

    const size_t w = CGImageGetWidth( cg );
    const size_t h = CGImageGetHeight( cg );

    if( w == 0 || h == 0 )
        return false;

    // the usual cache rep is 8-bit RGB(A) in one plane: its pixels are taken directly, without
    // drawing them once more through Core Graphics
    const unsigned char* data = [rep bitmapData];
    const NSInteger      spp = [rep samplesPerPixel];

    if( data && ![rep isPlanar] && [rep bitsPerSample] == 8 && ( spp == 3 || spp == 4 )
        && !( [rep bitmapFormat] & NSBitmapFormatAlphaFirst ) && (size_t) [rep pixelsWide] == w
        && (size_t) [rep pixelsHigh] == h )
    {
        const size_t   stride = [rep bytesPerRow];
        unsigned char* rgb = (unsigned char*) malloc( w * h * 3 );

        for( size_t y = 0; y < h; ++y )
        {
            const unsigned char* s = data + y * stride;
            unsigned char*       d = rgb + y * w * 3;

            for( size_t x = 0; x < w; ++x )
            {
                d[x * 3 + 0] = s[x * spp + 0];
                d[x * 3 + 1] = s[x * spp + 1];
                d[x * 3 + 2] = s[x * spp + 2];
            }
        }

        aImage.SetData( rgb, (int) w, (int) h, false );
        return aImage.IsOk();
    }

    // Normalise whatever the rep holds to 8-bit RGBA
    std::vector<unsigned char> rgba( w * h * 4 );
    CGColorSpaceRef            space = CGColorSpaceCreateDeviceRGB();
    CGContextRef ctx = CGBitmapContextCreate( rgba.data(), w, h, 8, w * 4, space,
                                              kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big );
    CGColorSpaceRelease( space );

    if( !ctx )
        return false;

    CGContextDrawImage( ctx, CGRectMake( 0, 0, w, h ), cg );
    CGContextRelease( ctx );

    unsigned char* rgb = (unsigned char*) malloc( w * h * 3 );

    for( size_t i = 0; i < w * h; ++i )
    {
        rgb[i * 3 + 0] = rgba[i * 4 + 0];
        rgb[i * 3 + 1] = rgba[i * 4 + 1];
        rgb[i * 3 + 2] = rgba[i * 4 + 2];
    }

    aImage.SetData( rgb, (int) w, (int) h, false );
    return aImage.IsOk();
}
