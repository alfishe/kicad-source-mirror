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

#ifndef SYMBOL_REMAPPER_H
#define SYMBOL_REMAPPER_H

#include <wx/string.h>

#include <vector>

class LEGACY_SYMBOL_LIB;
class LEGACY_SYMBOL_LIBS;
class PROJECT;
class REPORTER;
class SCH_SYMBOL;
class SCHEMATIC;


/**
 * Remap Symbols: moves a project whose symbols name no library (the legacy library list look up)
 * over to the symbol library table.  The rescue of legacy library symbols runs first (RESCUER).
 * DIALOG_SYMBOL_REMAP and the API drive it.
 */
class SYMBOL_REMAPPER
{
public:
    /// @param aQuiet library load errors go to LoadErrors() instead of a dialog
    SYMBOL_REMAPPER( PROJECT& aProject, SCHEMATIC& aSchematic, bool aQuiet );

    /// The project sub folder the backup goes to
    static const wxString& BackupFolder();

    /// Empty if the project folder is writable, else why remapping is not possible
    wxString CheckWritable() const;

    /**
     * Backup all of the files that could be modified by the remapping with a time stamp
     * appended to the file name into BackupFolder(): the schematics, the project file, the
     * symbol library table, the cache library and the rescue library.
     *
     * @param aErrors the files that could not be backed up (empty if none).
     * @return false if the backup folder cannot be created (aErrors says so).
     */
    bool BackupProject( REPORTER& aReporter, wxString& aErrors );

    /// A project symbol library table of the legacy libraries not in the global one, every
    /// symbol relinked through it, the legacy library list emptied
    void Remap( REPORTER& aReporter );

    const std::vector<SCH_SYMBOL*>& Remapped() const { return m_remapped; }
    const std::vector<SCH_SYMBOL*>& Unmapped() const { return m_unmapped; }
    const std::vector<wxString>&    AddedLibraries() const { return m_addedLibraries; }
    const wxString&                 LoadErrors() const { return m_loadErrors; }

private:
    LEGACY_SYMBOL_LIBS* legacyLibs();

    /**
     * Add libraries found in the legacy library list to \a aLibs that are not found in
     * the global symbol library table.  The cache library is ignored.
     *
     * @return the number of libraries found.
     */
    size_t getLibsNotInGlobalSymbolLibTable( std::vector<LEGACY_SYMBOL_LIB*>& aLibs );

    void createProjectSymbolLibTable( REPORTER& aReporter );

    void remapSymbolsToLibTable( REPORTER& aReporter );

    bool remapSymbolToLibTable( SCH_SYMBOL* aSymbol );

    PROJECT&                 m_project;
    SCHEMATIC&               m_schematic;
    bool                     m_quiet;
    wxString                 m_loadErrors;
    std::vector<SCH_SYMBOL*> m_remapped;
    std::vector<SCH_SYMBOL*> m_unmapped;
    std::vector<wxString>    m_addedLibraries;
};

#endif
