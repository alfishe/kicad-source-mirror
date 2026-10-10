/// @file kopenapi_occ.cpp
/// @brief 3D model geometry for inspection and seating (see kopenapi_occ.h).
#include "kopenapi_occ.h"

#include <nlohmann/json.hpp>
#include <wx/stdpaths.h>

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <vector>


namespace
{

std::mutex                                     g_cacheMutex;
std::map<std::string, KOPENAPI_MODEL_GEOMETRY> g_cache;
bool                                           g_cacheLoaded = false;


/// @brief Results survive the process: <user cache>/kicad/openapi/model-geometry.json (keyed by
/// file, mtime and placement, so a changed model is measured again)
std::filesystem::path cacheFile()
{
    const wxString dir = wxStandardPaths::Get().GetUserDir( wxStandardPaths::Dir_Cache );
    return std::filesystem::path( dir.ToStdString( wxConvUTF8 ) ) / "kicad" / "openapi" / "model-geometry.json";
}


void loadCache()
{
    if( g_cacheLoaded )
        return;

    g_cacheLoaded = true;
    std::ifstream in( cacheFile() );
    nlohmann::json all = nlohmann::json::parse( in, nullptr, false );

    if( !all.is_object() )
        return;

    for( auto it = all.begin(); it != all.end(); ++it )
    {
        const nlohmann::json& j = it.value();
        KOPENAPI_MODEL_GEOMETRY g;
        g.ok = true;

        for( int a = 0; a < 3; ++a )
        {
            g.min[a] = j["min"][a].get<double>();
            g.max[a] = j["max"][a].get<double>();
        }

        g.bodyBottom = j.value( "body_bottom", 0.0 );
        g.hasLeads = j.value( "has_leads", false );
        g.hasBody = j.value( "body", true );
        g_cache[it.key()] = g;
    }
}


void saveCache()
{
    nlohmann::json all = nlohmann::json::object();

    for( const auto& [key, g] : g_cache )
    {
        all[key] = { { "min", { g.min[0], g.min[1], g.min[2] } }, { "max", { g.max[0], g.max[1], g.max[2] } },
                     { "body_bottom", g.bodyBottom }, { "has_leads", g.hasLeads }, { "body", g.hasBody } };
    }

    std::error_code ec;
    std::filesystem::create_directories( cacheFile().parent_path(), ec );
    const std::filesystem::path tmp = cacheFile().string() + ".tmp";
    std::ofstream( tmp ) << all.dump();
    std::filesystem::rename( tmp, cacheFile(), ec );
}


/// @brief Triangle of the model's mesh in the footprint frame, vertices counterclockwise seen
/// from outside
typedef std::array<gp_XYZ, 3> TRIANGLE;

/// @brief Linear deflection of the mesh (mm); faces whose mesh box comes within ten times this
/// of the model's box get their exact box
constexpr double MESH_DEFLECTION = 0.01;
constexpr double MESH_ANGLE = 0.3;


/// @brief Volume of the meshed solids below z = aZ: the flux of F = ( x, 0, 0 ) (div F = 1) out
/// of the part of each triangle below the plane; the cut face adds nothing (its normal has no x)
double volumeBelow( const std::vector<TRIANGLE>& aMesh, double aZ )
{
    double volume = 0;

    for( const TRIANGLE& t : aMesh )
    {
        gp_XYZ poly[4];
        int    n = 0;

        for( int i = 0; i < 3; ++i )
        {
            const gp_XYZ& a = t[i];
            const gp_XYZ& b = t[( i + 1 ) % 3];

            if( a.Z() < aZ )
                poly[n++] = a;

            if( ( a.Z() < aZ ) != ( b.Z() < aZ ) )
                poly[n++] = a + ( b - a ) * ( ( aZ - a.Z() ) / ( b.Z() - a.Z() ) );
        }

        for( int i = 1; i + 1 < n; ++i )
        {
            const gp_XYZ u = poly[i] - poly[0];
            const gp_XYZ v = poly[i + 1] - poly[0];
            volume += ( u.Y() * v.Z() - u.Z() * v.Y() ) * ( poly[0].X() + poly[i].X() + poly[i + 1].X() ) / 6;
        }
    }

    return volume;
}


/// @brief aMatrix moves a shape without deforming it (a rotation and a translation)
bool isRigid( const glm::mat4& aMatrix )
{
    const glm::mat3 r( aMatrix );
    const glm::mat3 d = glm::transpose( r ) * r - glm::mat3( 1.0f );

    for( int c = 0; c < 3; ++c )
    {
        for( int k = 0; k < 3; ++k )
        {
            if( std::abs( d[c][k] ) > 1e-4 )
                return false;
        }
    }

    return glm::determinant( r ) > 0;
}

} // namespace


KOPENAPI_MODEL_GEOMETRY KopenapiModelGeometry( const std::filesystem::path& aStep, const glm::mat4& aMatrix, bool aBody )
{
    std::error_code    ec;
    std::ostringstream key;
    key << "v2|" << aStep.string() << '|'
        << std::chrono::duration_cast<std::chrono::seconds>(
                   std::filesystem::last_write_time( aStep, ec ).time_since_epoch() ).count();

    for( int c = 0; c < 4; ++c )
    {
        for( int r = 0; r < 4; ++r )
            key << '|' << std::round( aMatrix[c][r] * 1e4 );
    }

    {
        std::lock_guard<std::mutex> lock( g_cacheMutex );
        loadCache();

        if( auto it = g_cache.find( key.str() ); it != g_cache.end() && ( it->second.hasBody || !aBody ) )
        {
            KOPENAPI_MODEL_GEOMETRY g = it->second;
            g.seconds = 0;
            return g;
        }
    }

    const auto              t0 = std::chrono::steady_clock::now();
    KOPENAPI_MODEL_GEOMETRY g;

    TopoDS_Shape model;

    {
        // the STEP translator keeps global state: one reader at a time, the geometry after it runs
        // in parallel
        static std::mutex readMutex;
        std::lock_guard<std::mutex> lock( readMutex );

        STEPControl_Reader reader;

        if( !std::filesystem::exists( aStep ) || reader.ReadFile( aStep.string().c_str() ) != IFSelect_RetDone )
        {
            g.error = "cannot read " + aStep.string();
            return g;
        }

        reader.TransferRoots();
        model = reader.OneShape();
    }

    // Into the footprint frame: a rigid matrix only locates the shape and its mesh nodes; any
    // other is applied to a copy of the geometry
    gp_Trsf      place;
    TopoDS_Shape meshed = model;

    if( isRigid( aMatrix ) )
    {
        place.SetValues( aMatrix[0][0], aMatrix[1][0], aMatrix[2][0], aMatrix[3][0],   // glm is column-major
                         aMatrix[0][1], aMatrix[1][1], aMatrix[2][1], aMatrix[3][1],
                         aMatrix[0][2], aMatrix[1][2], aMatrix[2][2], aMatrix[3][2] );
        place.SetScaleFactor( 1.0 );
    }
    else
    {
        gp_GTrsf deform;

        for( int r = 0; r < 3; ++r )
        {
            for( int c = 0; c < 3; ++c )
                deform.SetValue( r + 1, c + 1, aMatrix[c][r] );

            deform.SetValue( r + 1, 4, aMatrix[3][r] );
        }

        meshed = BRepBuilderAPI_GTransform( model, deform, true ).Shape();
    }

    const TopLoc_Location placeLoc( place );
    const TopoDS_Shape    shape = meshed.Moved( placeLoc );

    // The mesh gives the volumes and a first box; faces near the box's sides then get their exact
    // (optimal) box
    BRepMesh_IncrementalMesh( meshed, MESH_DEFLECTION, false, MESH_ANGLE, false );

    struct FACE_BOX
    {
        TopoDS_Face face;
        gp_XYZ      lo, hi;
        bool        meshed;
    };

    std::vector<FACE_BOX> faces;
    std::vector<TRIANGLE> mesh;
    gp_XYZ                lo( 1e9, 1e9, 1e9 ), hi( -1e9, -1e9, -1e9 );

    for( TopExp_Explorer ex( meshed, TopAbs_FACE ); ex.More(); ex.Next() )
    {
        const TopoDS_Face&               face = TopoDS::Face( ex.Current() );
        TopLoc_Location                  loc;
        const Handle( Poly_Triangulation ) tri = BRep_Tool::Triangulation( face, loc );
        FACE_BOX                         fb{ face, gp_XYZ( 1e9, 1e9, 1e9 ), gp_XYZ( -1e9, -1e9, -1e9 ), !tri.IsNull() };

        if( fb.meshed )
        {
            const gp_Trsf trsf = place * loc.Transformation();
            const bool    reversed = face.Orientation() == TopAbs_REVERSED;
            const int     first = (int) mesh.size();

            for( int i = 1; i <= tri->NbTriangles(); ++i )
            {
                int n[3];
                tri->Triangle( i ).Get( n[0], n[1], n[2] );

                if( reversed )
                    std::swap( n[1], n[2] );

                TRIANGLE t;

                for( int k = 0; k < 3; ++k )
                {
                    t[k] = tri->Node( n[k] ).Transformed( trsf ).XYZ();
                    fb.lo.SetCoord( std::min( fb.lo.X(), t[k].X() ), std::min( fb.lo.Y(), t[k].Y() ), std::min( fb.lo.Z(), t[k].Z() ) );
                    fb.hi.SetCoord( std::max( fb.hi.X(), t[k].X() ), std::max( fb.hi.Y(), t[k].Y() ), std::max( fb.hi.Z(), t[k].Z() ) );
                }

                mesh.push_back( t );
            }

            if( (int) mesh.size() > first )
            {
                lo.SetCoord( std::min( lo.X(), fb.lo.X() ), std::min( lo.Y(), fb.lo.Y() ), std::min( lo.Z(), fb.lo.Z() ) );
                hi.SetCoord( std::max( hi.X(), fb.hi.X() ), std::max( hi.Y(), fb.hi.Y() ), std::max( hi.Z(), fb.hi.Z() ) );
            }
            else
            {
                fb.meshed = false;
            }
        }

        faces.push_back( fb );
    }

    Bnd_Box      box;
    const double margin = 10 * MESH_DEFLECTION;

    for( const FACE_BOX& fb : faces )
    {
        bool exact = !fb.meshed;

        for( int a = 1; a <= 3 && !exact; ++a )
            exact = fb.lo.Coord( a ) < lo.Coord( a ) + margin || fb.hi.Coord( a ) > hi.Coord( a ) - margin;

        if( exact )
            BRepBndLib::AddOptimal( fb.face.Moved( placeLoc ), box, false, false );
    }

    // free edges / vertices (construction wires some vendor files carry) are not drawn by the 3D
    // viewer: they stay out of the box

    if( !box.IsVoid() )
    {
        double b[6];
        box.Get( b[0], b[1], b[2], b[3], b[4], b[5] );
        lo.SetCoord( std::min( lo.X(), b[0] ), std::min( lo.Y(), b[1] ), std::min( lo.Z(), b[2] ) );
        hi.SetCoord( std::max( hi.X(), b[3] ), std::max( hi.Y(), b[4] ), std::max( hi.Z(), b[5] ) );
    }

    if( lo.X() > hi.X() )
    {
        g.error = "empty model";
        return g;
    }

    for( int a = 0; a < 3; ++a )
    {
        g.min[a] = lo.Coord( a + 1 );
        g.max[a] = hi.Coord( a + 1 );
    }

    if( aBody )
    {
        // Body bottom: walk up from the lowest point; leads / pins have a small cross section, the
        // body starts where it jumps (> 3x the leads' section and > 1 mm2 more). Coarse, then fine.
        const double height = g.max[2] - g.min[2];
        const double reach = std::min( height, 25.0 );

        // the slabs only need the lower part
        mesh.erase( std::remove_if( mesh.begin(), mesh.end(),
                                    [&]( const TRIANGLE& t )
                                    {
                                        return std::min( { t[0].Z(), t[1].Z(), t[2].Z() } ) > g.min[2] + reach + 0.6;
                                    } ),
                    mesh.end() );

        auto slab = [&]( double aLo, double aHi )
        {
            return volumeBelow( mesh, aHi ) - volumeBelow( mesh, aLo );
        };

        const double leads = slab( g.min[2] + 0.2, g.min[2] + 0.3 ) / 0.1;
        auto         isBody = [&]( double z, double dz )
        {
            const double area = slab( z, z + dz ) / dz;
            return area > 3 * leads && area > leads + 1.0;
        };

        g.bodyBottom = g.min[2];

        for( double z = g.min[2] + 0.5; z < g.min[2] + reach; z += 0.5 )
        {
            if( !isBody( z, 0.5 ) )
                continue;

            // the jump lies within ( z - 0.5, z + 0.5 ]
            double fine = z - 0.5;

            while( fine < z + 0.5 && !isBody( fine, 0.05 ) )
                fine += 0.05;

            g.bodyBottom = std::round( fine * 100 ) / 100;
            g.hasLeads = g.bodyBottom - g.min[2] > 0.3;
            break;
        }

        g.hasBody = true;
    }

    g.ok = true;
    g.seconds = std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();

    std::lock_guard<std::mutex> lock( g_cacheMutex );
    g_cache[key.str()] = g;
    saveCache();
    return g;
}
