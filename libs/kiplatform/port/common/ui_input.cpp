/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
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

#include <wx/utils.h>

#include <optional>


namespace
{
std::optional<KIPLATFORM::UI::INJECTED_POINTER> g_injected;
} // namespace


void KIPLATFORM::UI::SetInjectedPointer( const INJECTED_POINTER* aPointer )
{
    if( aPointer )
        g_injected = *aPointer;
    else
        g_injected.reset();
}


const KIPLATFORM::UI::INJECTED_POINTER* KIPLATFORM::UI::GetInjectedPointer()
{
    return g_injected ? &*g_injected : nullptr;
}


wxMouseState KIPLATFORM::UI::GetMouseState()
{
    if( !g_injected )
        return wxGetMouseState();

    wxMouseState state;
    state.SetPosition( g_injected->screenPosition );
    state.SetLeftDown( g_injected->left );
    state.SetMiddleDown( g_injected->middle );
    state.SetRightDown( g_injected->right );
    state.SetShiftDown( g_injected->shift );
    state.SetControlDown( g_injected->control );
    state.SetAltDown( g_injected->alt );
    return state;
}
