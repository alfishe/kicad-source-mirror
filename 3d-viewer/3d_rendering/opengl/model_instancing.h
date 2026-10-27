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

#ifndef MODEL_INSTANCING_H
#define MODEL_INSTANCING_H

#include <kicad_gl/kiglad.h> // Must be included first

#include <map>
#include <string>
#include <vector>
#include <glm/glm.hpp>

/**
 * Draws many copies of a model in one call per material: the model-view and normal matrices of
 * each copy are vertex attributes advancing once per instance, and a vertex shader does the
 * fixed-function lighting from the current GL state (lights, material, colour material).
 *
 * Each driver call of the fixed-function path re-validates and uploads the whole lighting and
 * matrix state; with instances the cost is paid once per model and material, not per footprint.
 */
class MODEL_INSTANCING
{
public:
    struct INSTANCE
    {
        glm::vec4 m_modelView[3];   ///< rows of the affine model-view
        glm::vec3 m_normal[3];      ///< rows of the inverse transpose of its 3x3
        bool      m_uniform;        ///< no uneven scale or shear: normals turn with the model-view
    };

    static INSTANCE Instance( const glm::mat4& aModelView );

    /**
     * Take the current lights (their kinds pick the programs, built in the current context on
     * first use; the rest is kept for Use()).
     *
     * @return false when the context has no instanced arrays or the shaders do not build: the
     *         caller draws one model at a time then.
     */
    bool Available();

    /**
     * Upload every instance of the frame, then draw ranges of it.
     */
    void Upload( const std::vector<INSTANCE>& aInstances );

    /**
     * Start drawing; the fixed-function vertex arrays stay as set.
     */
    void Begin();

    /**
     * Draw with the program for clipped instances (the current clip planes apply) or not, and
     * for instances that need the normal matrix or not; call SetInstances() after it.
     */
    void Use( bool aClip, bool aNormalMatrix );

    /**
     * Point the instance attributes at the uploaded instances from aFirst on.
     */
    void SetInstances( size_t aFirst );

    /**
     * Draw an index range of the bound model aCount times.
     */
    void DrawElements( GLsizei aIndexCount, GLenum aIndexType, uintptr_t aIndexOffset, GLsizei aCount ) const;

    void End();

    /**
     * Delete the GL objects; their context must be current.
     */
    void Release();

private:
    struct PROGRAM
    {
        GLuint id = 0;
        GLint  light = -1;
        GLint  half = -1;
        GLint  ambient = -1;
    };

    const PROGRAM& program( bool aClip, bool aNormalMatrix );

    bool                           m_tried = false;
    bool                           m_capable = false;
    bool                           m_arb = false;   ///< ARB entry points (GL below 3.3)
    std::string                    m_lights;        ///< per light 0..2: '-' off, 'D'irectional, 'P'oint, 'A'ttenuated
    std::map<std::string, PROGRAM> m_programs;      ///< by lights, clip, normal matrix; id 0 when it did not build
    const PROGRAM*                 m_current = nullptr;
    bool                           m_normalArrays = false;
    glm::vec3                      m_light[3] = {};
    glm::vec3                      m_half[3] = {};
    glm::vec4                      m_ambient = {};
    GLuint                         m_buffer = 0;
    size_t                         m_capacity = 0;
};

#endif // MODEL_INSTANCING_H
