/*
* This program source code file is part of KiCad, a free EDA CAD application.
*
* Copyright (C) 2020 Mark Roszko <mark.roszko@gmail.com>
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

#import <Cocoa/Cocoa.h>

#include <kiplatform/app.h>

#include <wx/string.h>
#include <wx/sysopt.h>


bool KIPLATFORM::APP::Init()
{
    // KiCad relies on showing the file type selector in a few places; force it to be shown
    wxSystemOptions::SetOption( wxS( "osx.openfiledialog.always-show-types" ), 1 );
    return true;
}


void KIPLATFORM::APP::SetBackgroundProcess()
{
    // The process registers with the window server when NSApplication is first created (wx
    // does that in wxAppConsole::Initialize), taking its activation policy from the main
    // bundle's Info.plist, which this executable shares with the GUI app.  Marking the
    // in-memory copy BackgroundOnly before that point keeps it out of the Dock entirely.
    @try
    {
        NSDictionary* info = [[NSBundle mainBundle] infoDictionary];

        if( [info isKindOfClass:[NSMutableDictionary class]] )
            [(NSMutableDictionary*) info setObject:@YES forKey:@"LSBackgroundOnly"];
    }
    @catch( NSException* )
    {
    }

    if( NSApp )
        [NSApp setActivationPolicy:NSApplicationActivationPolicyProhibited];
}


void KIPLATFORM::APP::DisableAutomaticFullScreen()
{
    // macOS state restoration re-opens windows in their saved state, including full screen
    // in its own (black until drawn) space.  KiCad restores its own window geometry.
    NSUserDefaults* defaults = [NSUserDefaults standardUserDefaults];
    [defaults setBool:YES forKey:@"ApplePersistenceIgnoreState"];
    [defaults setBool:NO forKey:@"NSQuitAlwaysKeepsWindows"];
}


void KIPLATFORM::APP::EnableDarkMode( bool aForce )
{
}


bool KIPLATFORM::APP::AttachConsole( bool aTryAlloc )
{
    // Not implemented on this platform
    return true;
}


bool KIPLATFORM::APP::IsOperatingSystemUnsupported()
{
    // Not implemented on this platform
    return false;
}


bool KIPLATFORM::APP::RegisterApplicationRestart( const wxString& aCommandLine )
{
    // Not implemented on this platform
    return true;
}


bool KIPLATFORM::APP::UnregisterApplicationRestart()
{
    // Not implemented on this platform
    return true;
}


bool KIPLATFORM::APP::SupportsShutdownBlockReason()
{
    return false;
}


void KIPLATFORM::APP::RemoveShutdownBlockReason( wxWindow* aWindow )
{
}


void KIPLATFORM::APP::SetShutdownBlockReason( wxWindow* aWindow, const wxString& aReason )
{
}


void KIPLATFORM::APP::ForceTimerMessagesToBeCreatedIfNecessary()
{
}


void KIPLATFORM::APP::AddDynamicLibrarySearchPath( const wxString& aPath )
{
}
