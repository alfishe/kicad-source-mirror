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

#ifndef COMMAND_OPENAPI_RUN_H
#define COMMAND_OPENAPI_RUN_H

#include "command.h"

namespace CLI
{

/// @brief Runs a batch of Web API calls headless (kicad-cli openapi-run batch.json): the calls go
/// through the same path as REST (owner strand, jobs, read leases, journal), one JSON line per call
/// on stdout.  The service listens meanwhile, so other clients can read while the batch runs
class OPENAPI_RUN_COMMAND : public COMMAND
{
public:
    OPENAPI_RUN_COMMAND();

protected:
    int doPerform( KIWAY& aKiway ) override;
};

} // namespace CLI

#endif
