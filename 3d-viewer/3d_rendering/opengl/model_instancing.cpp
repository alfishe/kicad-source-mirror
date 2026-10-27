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

#include <kicad_gl/kiglad.h> // Must be included first

#include "model_instancing.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <wx/log.h>


namespace
{
// Generic attributes kept clear of the ones aliased by gl_Vertex (0), gl_Normal (2), gl_Color (3)
// and gl_MultiTexCoord0 (8) on drivers that alias them
constexpr GLuint ATTR_MODELVIEW = 4;   ///< 4..6, rows of the affine model-view
constexpr GLuint ATTR_NORMAL = 9;      ///< 9..11, rows of the normal matrix

// The fixed-function per-vertex lighting (OpenGL 2.1, 2.14.1) for lights 0..2, front material,
// no local viewer, colour material tracking ambient and diffuse from the vertex colour. Each light
// is built in as off, directional, point or attenuated point; what does not change over a frame
// (directions, half vectors, the ambient of unattenuated lights) comes in uniforms. Normals turn
// with the model-view itself unless the instances scale unevenly (NORMAL_MATRIX); the clip vertex
// is written only for clipped instances (CLIP).
const char* const VERTEX_SHADER_HEAD = R"(
attribute vec4 a_mv0;
attribute vec4 a_mv1;
attribute vec4 a_mv2;
#ifdef NORMAL_MATRIX
attribute vec3 a_n0;
attribute vec3 a_n1;
attribute vec3 a_n2;
#endif
uniform vec3 u_light[3];      // directional: unit direction; point: eye position
uniform vec3 u_half[3];       // directional: half vector
uniform vec4 u_ambient;       // scene ambient + ambient of the unattenuated lights

void main()
{
    vec3 eye = vec3( dot( a_mv0, gl_Vertex ), dot( a_mv1, gl_Vertex ), dot( a_mv2, gl_Vertex ) );
#ifdef NORMAL_MATRIX
    vec3 n = normalize( vec3( dot( a_n0, gl_Normal ), dot( a_n1, gl_Normal ), dot( a_n2, gl_Normal ) ) );
#else
    vec3 n = normalize( vec3( dot( a_mv0.xyz, gl_Normal ), dot( a_mv1.xyz, gl_Normal ),
                              dot( a_mv2.xyz, gl_Normal ) ) );
#endif
    vec4 ambient = u_ambient;
    vec4 diffuse = vec4( 0.0 );
    vec4 specular = vec4( 0.0 );
)";

// one light: I is its index; vp, h and att as in 2.14.1
const char* const LIGHT_DIRECTIONAL = R"(
    {
        float nl = dot( n, u_light[I] );

        if( nl > 0.0 )
        {
            diffuse += nl * gl_LightSource[I].diffuse;
            float nh = dot( n, u_half[I] );

            if( nh > 0.0 )
                specular += pow( nh, gl_FrontMaterial.shininess ) * gl_LightSource[I].specular;
        }
    }
)";

const char* const LIGHT_POINT = R"(
    {
        vec3  vp = normalize( u_light[I] - eye );
        float nl = dot( n, vp );

        if( nl > 0.0 )
        {
            diffuse += nl * gl_LightSource[I].diffuse;
            float nh = dot( n, normalize( vp + vec3( 0.0, 0.0, 1.0 ) ) );

            if( nh > 0.0 )
                specular += pow( nh, gl_FrontMaterial.shininess ) * gl_LightSource[I].specular;
        }
    }
)";

const char* const LIGHT_ATTENUATED = R"(
    {
        vec3  d = u_light[I] - eye;
        float dist = length( d );
        vec3  vp = d / dist;
        float att = 1.0 / ( gl_LightSource[I].constantAttenuation + gl_LightSource[I].linearAttenuation * dist
                            + gl_LightSource[I].quadraticAttenuation * dist * dist );
        float nl = dot( n, vp );

        ambient += att * gl_LightSource[I].ambient;

        if( nl > 0.0 )
        {
            diffuse += att * nl * gl_LightSource[I].diffuse;
            float nh = dot( n, normalize( vp + vec3( 0.0, 0.0, 1.0 ) ) );

            if( nh > 0.0 )
                specular += att * pow( nh, gl_FrontMaterial.shininess ) * gl_LightSource[I].specular;
        }
    }
)";

const char* const VERTEX_SHADER_TAIL = R"(
    vec4 c = gl_FrontMaterial.emission + gl_Color * ( ambient + diffuse ) + specular * gl_FrontMaterial.specular;

    gl_FrontColor = vec4( clamp( c.rgb, 0.0, 1.0 ), gl_Color.a );
    gl_Position = gl_ProjectionMatrix * vec4( eye, 1.0 );
#ifdef CLIP
    gl_ClipVertex = vec4( eye, 1.0 );
#endif
}
)";

const char* const FRAGMENT_SHADER = R"(#version 120
void main()
{
    gl_FragColor = gl_Color;
}
)";


/// @param aLights per light 0..2: '-' off, 'D' directional, 'P' point, 'A' attenuated point
std::string vertexShader( const std::string& aLights, bool aClip, bool aNormalMatrix )
{
    std::string source = "#version 120\n";

    if( aClip )
        source += "#define CLIP\n";

    if( aNormalMatrix )
        source += "#define NORMAL_MATRIX\n";

    source += VERTEX_SHADER_HEAD;

    for( size_t i = 0; i < aLights.size(); ++i )
    {
        const char* light = aLights[i] == 'D' ? LIGHT_DIRECTIONAL
                            : aLights[i] == 'P' ? LIGHT_POINT
                            : aLights[i] == 'A' ? LIGHT_ATTENUATED
                                                : nullptr;

        if( !light )
            continue;

        std::string body = light;

        for( size_t at = body.find( "[I]" ); at != std::string::npos; at = body.find( "[I]", at ) )
            body.replace( at + 1, 1, std::to_string( i ) );

        source += body;
    }

    return source + VERTEX_SHADER_TAIL;
}


GLuint compile( GLenum aType, const char* aSource )
{
    GLuint shader = glCreateShader( aType );
    GLint  ok = GL_FALSE;

    glShaderSource( shader, 1, &aSource, nullptr );
    glCompileShader( shader );
    glGetShaderiv( shader, GL_COMPILE_STATUS, &ok );

    if( !ok )
    {
        char log[1024] = {};

        glGetShaderInfoLog( shader, sizeof( log ), nullptr, log );
        wxLogDebug( wxT( "3D model instancing shader: %s" ), log );
        glDeleteShader( shader );
        return 0;
    }

    return shader;
}


GLuint link( const std::string& aVertexShader )
{
    GLuint vs = compile( GL_VERTEX_SHADER, aVertexShader.c_str() );
    GLuint fs = vs ? compile( GL_FRAGMENT_SHADER, FRAGMENT_SHADER ) : 0;

    if( !vs || !fs )
    {
        if( vs )
            glDeleteShader( vs );

        return 0;
    }

    GLuint program = glCreateProgram();
    glAttachShader( program, vs );
    glAttachShader( program, fs );

    for( GLuint i = 0; i < 3; ++i )
    {
        glBindAttribLocation( program, ATTR_MODELVIEW + i, ( "a_mv" + std::to_string( i ) ).c_str() );
        glBindAttribLocation( program, ATTR_NORMAL + i, ( "a_n" + std::to_string( i ) ).c_str() );
    }

    glLinkProgram( program );
    glDeleteShader( vs );
    glDeleteShader( fs );

    GLint ok = GL_FALSE;
    glGetProgramiv( program, GL_LINK_STATUS, &ok );

    if( !ok )
    {
        char log[1024] = {};

        glGetProgramInfoLog( program, sizeof( log ), nullptr, log );
        wxLogDebug( wxT( "3D model instancing program: %s" ), log );
        glDeleteProgram( program );
        return 0;
    }

    return program;
}


void setDivisor( bool aArb, GLuint aAttribute, GLuint aDivisor )
{
    aArb ? glVertexAttribDivisorARB( aAttribute, aDivisor ) : glVertexAttribDivisor( aAttribute, aDivisor );
}
} // namespace


MODEL_INSTANCING::INSTANCE MODEL_INSTANCING::Instance( const glm::mat4& aModelView )
{
    INSTANCE instance;
    const glm::mat4 rows = glm::transpose( aModelView );

    for( int i = 0; i < 3; ++i )
        instance.m_modelView[i] = rows[i];

    const glm::mat3 m( aModelView );
    const float     x = glm::dot( m[0], m[0] );
    const float     y = glm::dot( m[1], m[1] );
    const float     z = glm::dot( m[2], m[2] );
    const float     eps = 1e-4f * std::max( { x, y, z } );

    instance.m_uniform = std::abs( x - y ) <= eps && std::abs( x - z ) <= eps && std::abs( glm::dot( m[0], m[1] ) ) <= eps
                         && std::abs( glm::dot( m[0], m[2] ) ) <= eps && std::abs( glm::dot( m[1], m[2] ) ) <= eps;

    const glm::mat3 normalRows = glm::transpose( glm::inverseTranspose( m ) );

    for( int i = 0; i < 3; ++i )
        instance.m_normal[i] = normalRows[i];

    return instance;
}


bool MODEL_INSTANCING::Available()
{
    if( !m_tried )
    {
        m_tried = true;
        m_capable = GLAD_GL_VERSION_2_0 && ( GLAD_GL_VERSION_3_3 || GLAD_GL_ARB_instanced_arrays );
        m_arb = !GLAD_GL_VERSION_3_3;

        if( m_capable )
            glGenBuffers( 1, &m_buffer );

        m_capable = m_capable && m_buffer;
    }

    if( !m_capable )
        return false;

    // the lights as they are now: their kinds pick the programs, the rest goes in uniforms
    GLfloat sceneAmbient[4];
    glGetFloatv( GL_LIGHT_MODEL_AMBIENT, sceneAmbient );

    m_lights.assign( 3, '-' );
    m_ambient = glm::make_vec4( sceneAmbient );

    for( int i = 0; i < 3; ++i )
    {
        const GLenum light = GL_LIGHT0 + i;

        if( !glIsEnabled( light ) )
            continue;

        GLfloat position[4], ambient[4], att[3];
        glGetLightfv( light, GL_POSITION, position );
        glGetLightfv( light, GL_AMBIENT, ambient );
        glGetLightfv( light, GL_CONSTANT_ATTENUATION, &att[0] );
        glGetLightfv( light, GL_LINEAR_ATTENUATION, &att[1] );
        glGetLightfv( light, GL_QUADRATIC_ATTENUATION, &att[2] );

        if( position[3] == 0.0f )
        {
            m_lights[i] = 'D';
            m_light[i] = glm::normalize( glm::vec3( position[0], position[1], position[2] ) );
            m_half[i] = glm::normalize( m_light[i] + glm::vec3( 0.0f, 0.0f, 1.0f ) );
        }
        else
        {
            m_lights[i] = att[0] == 1.0f && att[1] == 0.0f && att[2] == 0.0f ? 'P' : 'A';
            m_light[i] = glm::vec3( position[0], position[1], position[2] ) / position[3];
        }

        if( m_lights[i] != 'A' )
            m_ambient += glm::make_vec4( ambient );
    }

    return program( false, false ).id && program( true, true ).id;
}


const MODEL_INSTANCING::PROGRAM& MODEL_INSTANCING::program( bool aClip, bool aNormalMatrix )
{
    const std::string key = m_lights + ( aClip ? 'C' : '-' ) + ( aNormalMatrix ? 'N' : '-' );
    auto              it = m_programs.find( key );

    if( it == m_programs.end() )
    {
        PROGRAM program;
        program.id = link( vertexShader( m_lights, aClip, aNormalMatrix ) );

        if( program.id )
        {
            program.light = glGetUniformLocation( program.id, "u_light" );
            program.half = glGetUniformLocation( program.id, "u_half" );
            program.ambient = glGetUniformLocation( program.id, "u_ambient" );
        }

        it = m_programs.emplace( key, program ).first;
    }

    return it->second;
}


void MODEL_INSTANCING::Upload( const std::vector<INSTANCE>& aInstances )
{
    glBindBuffer( GL_ARRAY_BUFFER, m_buffer );

    const size_t bytes = aInstances.size() * sizeof( INSTANCE );

    // a new store each frame: the driver does not wait for the previous frame's draws
    if( bytes > m_capacity )
        m_capacity = bytes + bytes / 2;

    glBufferData( GL_ARRAY_BUFFER, m_capacity, nullptr, GL_STREAM_DRAW );
    glBufferSubData( GL_ARRAY_BUFFER, 0, bytes, aInstances.data() );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
}


void MODEL_INSTANCING::Begin()
{
    for( GLuint i = 0; i < 3; ++i )
    {
        glEnableVertexAttribArray( ATTR_MODELVIEW + i );
        setDivisor( m_arb, ATTR_MODELVIEW + i, 1 );
    }

    m_current = nullptr;
    m_normalArrays = false;
}


void MODEL_INSTANCING::Use( bool aClip, bool aNormalMatrix )
{
    const PROGRAM& use = program( aClip, aNormalMatrix );

    if( &use != m_current )
    {
        m_current = &use;
        glUseProgram( use.id );
        glUniform3fv( use.light, 3, &m_light[0].x );
        glUniform3fv( use.half, 3, &m_half[0].x );
        glUniform4fv( use.ambient, 1, &m_ambient.x );
    }

    if( aNormalMatrix != m_normalArrays )
    {
        m_normalArrays = aNormalMatrix;

        for( GLuint i = 0; i < 3; ++i )
        {
            aNormalMatrix ? glEnableVertexAttribArray( ATTR_NORMAL + i ) : glDisableVertexAttribArray( ATTR_NORMAL + i );
            setDivisor( m_arb, ATTR_NORMAL + i, aNormalMatrix ? 1 : 0 );
        }
    }
}


void MODEL_INSTANCING::SetInstances( size_t aFirst )
{
    glBindBuffer( GL_ARRAY_BUFFER, m_buffer );

    const uintptr_t base = aFirst * sizeof( INSTANCE );

    for( GLuint i = 0; i < 3; ++i )
    {
        glVertexAttribPointer( ATTR_MODELVIEW + i, 4, GL_FLOAT, GL_FALSE, sizeof( INSTANCE ),
                               reinterpret_cast<const void*>( base + offsetof( INSTANCE, m_modelView )
                                                              + i * sizeof( glm::vec4 ) ) );
    }

    if( m_normalArrays )
    {
        for( GLuint i = 0; i < 3; ++i )
        {
            glVertexAttribPointer( ATTR_NORMAL + i, 3, GL_FLOAT, GL_FALSE, sizeof( INSTANCE ),
                                   reinterpret_cast<const void*>( base + offsetof( INSTANCE, m_normal )
                                                                  + i * sizeof( glm::vec3 ) ) );
        }
    }
}


void MODEL_INSTANCING::DrawElements( GLsizei aIndexCount, GLenum aIndexType, uintptr_t aIndexOffset,
                                     GLsizei aCount ) const
{
    const void* offset = reinterpret_cast<const void*>( aIndexOffset );

    if( m_arb )
        glDrawElementsInstancedARB( GL_TRIANGLES, aIndexCount, aIndexType, offset, aCount );
    else
        glDrawElementsInstanced( GL_TRIANGLES, aIndexCount, aIndexType, offset, aCount );
}


void MODEL_INSTANCING::End()
{
    for( GLuint i = 0; i < 3; ++i )
    {
        setDivisor( m_arb, ATTR_MODELVIEW + i, 0 );
        glDisableVertexAttribArray( ATTR_MODELVIEW + i );
    }

    if( m_normalArrays )
    {
        for( GLuint i = 0; i < 3; ++i )
        {
            setDivisor( m_arb, ATTR_NORMAL + i, 0 );
            glDisableVertexAttribArray( ATTR_NORMAL + i );
        }
    }

    m_normalArrays = false;
    m_current = nullptr;
    glUseProgram( 0 );
}


void MODEL_INSTANCING::Release()
{
    for( const auto& [key, program] : m_programs )
    {
        if( program.id )
            glDeleteProgram( program.id );
    }

    if( m_buffer )
        glDeleteBuffers( 1, &m_buffer );

    m_programs.clear();
    m_current = nullptr;
    m_buffer = 0;
    m_capacity = 0;
    m_tried = false;
    m_capable = false;
}
