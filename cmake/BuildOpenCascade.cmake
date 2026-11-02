# Builds the vendored OpenCASCADE (thirdparty/opencascade) as static libraries.
#
# OCCT is configured at KiCad configure time (it collects its headers into the build tree, so they
# exist before anything that includes them compiles) and reconfigured from scratch when its version,
# options or compiler change. The libraries are built at build time by an ExternalProject step that
# every module linking them waits for, and linked from OCCT's build tree (no install step: on macOS
# CMake runs ranlib on installed archives, so an install would relink every module on every build).
#
# Output: OCC_FOUND, OCC_VERSION_STRING, OCC_INCLUDE_DIR, OCC_LIBRARY_DIR and OCC_LIBRARIES (the
# INTERFACE target kicad_occt: the toolkits in link order plus their system libraries).

include( ExternalProject )

find_package( Threads REQUIRED )

set( OCCT_VERSION 7.9.3 )
set( OCCT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/opencascade/occt-${OCCT_VERSION} )
set( OCCT_BINARY_DIR ${CMAKE_BINARY_DIR}/thirdparty/opencascade/build )
set( OCCT_STAMP_FILE ${CMAKE_BINARY_DIR}/thirdparty/opencascade/kicad-configure.stamp )

# Every toolkit of the modules KiCad uses, dependents before their dependencies (adm/MODULES,
# src/*/EXTERNLIB)
set( OCCT_TOOLKITS
     TKXMesh TKOffset TKMeshVS TKFillet TKFeat TKDEVRML TKDESTL TKDESTEP TKDEPLY TKDEOBJ TKDEIGES
     TKXSBase TKDEGLTF TKRWMesh TKDECascade TKXmlXCAF TKXmlTObj TKXml TKXmlL TKStd TKStdL TKDE
     TKBool TKBinXCAF TKXCAF TKVCAF TKV3d TKHLR TKMesh TKService TKBinTObj TKTObj TKBin TKBinL
     TKCAF TKLCAF TKCDF TKBO TKShHealing TKPrim TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d
     TKMath TKernel )

set( OCCT_CXX_FLAGS "" )

# GCC 16 IPA constant propagation miscompiles BRepClass3d_SClassifier (upstream-issues A71)
if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16 )
    string( APPEND OCCT_CXX_FLAGS " -fno-ipa-cp" )
endif()

set( OCCT_CMAKE_ARGS
     -G ${CMAKE_GENERATOR}
     -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
     -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
     -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
     "-DCMAKE_C_COMPILER_LAUNCHER=${CMAKE_C_COMPILER_LAUNCHER}"
     "-DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}"
     "-DCMAKE_C_FLAGS=${OCCT_CXX_FLAGS}"
     "-DCMAKE_CXX_FLAGS=${OCCT_CXX_FLAGS}"
     "-DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}"
     "-DCMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET}"
     "-DCMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT}"
     -DCMAKE_BUILD_TYPE=Release
     -DCMAKE_POSITION_INDEPENDENT_CODE=ON
     -DBUILD_LIBRARY_TYPE=Static
     -DINSTALL_DIR_LAYOUT=Unix
     -DBUILD_MODULE_Draw=OFF
     -DBUILD_MODULE_DETools=OFF
     -DBUILD_DOC_Overview=OFF
     -DBUILD_USE_VCPKG=OFF
     -DUSE_FREETYPE=ON
     "-D3RDPARTY_FREETYPE_INCLUDE_DIR_ft2build=${FREETYPE_INCLUDE_DIR_ft2build}"
     "-D3RDPARTY_FREETYPE_INCLUDE_DIR_freetype2=${FREETYPE_INCLUDE_DIR_freetype2}"
     -DUSE_RAPIDJSON=ON
     -D3RDPARTY_RAPIDJSON_INCLUDE_DIR=${CMAKE_SOURCE_DIR}/thirdparty/rapidjson/include
     -DUSE_TK=OFF -DUSE_TCL=OFF -DUSE_TBB=OFF -DUSE_VTK=OFF -DUSE_XLIB=OFF -DUSE_OPENGL=OFF
     -DUSE_GLES2=OFF -DUSE_D3D=OFF -DUSE_FREEIMAGE=OFF -DUSE_FFMPEG=OFF -DUSE_OPENVR=OFF
     -DUSE_DRACO=OFF -DUSE_EIGEN=OFF )

string( SHA1 OCCT_STAMP "${OCCT_VERSION};${OCCT_CMAKE_ARGS};${CMAKE_CXX_COMPILER_VERSION}" )
set( OCCT_STAMP_OLD "" )

if( EXISTS ${OCCT_STAMP_FILE} )
    file( READ ${OCCT_STAMP_FILE} OCCT_STAMP_OLD )
endif()

if( NOT OCCT_STAMP_OLD STREQUAL OCCT_STAMP OR NOT EXISTS ${OCCT_BINARY_DIR}/CMakeCache.txt )
    message( STATUS "Configuring OpenCASCADE ${OCCT_VERSION} (thirdparty/opencascade)" )
    file( REMOVE_RECURSE ${OCCT_BINARY_DIR} ${OCCT_STAMP_FILE} )
    file( MAKE_DIRECTORY ${OCCT_BINARY_DIR} )

    execute_process( COMMAND ${CMAKE_COMMAND} -S ${OCCT_SOURCE_DIR} -B ${OCCT_BINARY_DIR}
                             ${OCCT_CMAKE_ARGS}
                     RESULT_VARIABLE result
                     OUTPUT_FILE ${OCCT_BINARY_DIR}/kicad-configure.log
                     ERROR_FILE ${OCCT_BINARY_DIR}/kicad-configure.log )

    if( NOT result EQUAL 0 )
        message( FATAL_ERROR "OpenCASCADE configure failed: ${OCCT_BINARY_DIR}/kicad-configure.log" )
    endif()

    file( WRITE ${OCCT_STAMP_FILE} "${OCCT_STAMP}" )
endif()

# OCCT's build-tree library directory: <os><bits>/<compiler>/lib (CMakeLists.txt "build directories",
# adm/cmake/occt_macros.cmake); its configure names custom_<compiler>_<bits>.sh / .bat the same way
math( EXPR OCCT_BITS "${CMAKE_SIZEOF_VOID_P} * 8" )

if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    set( OCCT_COMPILER gcc )
elseif( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
    set( OCCT_COMPILER clang )
else()
    message( FATAL_ERROR "KICAD_OCC=vendored: GCC or Clang only (use KICAD_OCC=system)" )
endif()

if( WIN32 )
    set( OCCT_OS win )
elseif( APPLE )
    set( OCCT_OS mac )
else()
    set( OCCT_OS lin )
endif()

file( GLOB OCCT_LAYOUT_SCRIPT ${OCCT_BINARY_DIR}/custom_${OCCT_COMPILER}_${OCCT_BITS}.* )

if( NOT OCCT_LAYOUT_SCRIPT )
    message( FATAL_ERROR "OpenCASCADE build tree layout is not ${OCCT_COMPILER}_${OCCT_BITS}: ${OCCT_BINARY_DIR}" )
endif()

set( OCC_INCLUDE_DIR ${OCCT_BINARY_DIR}/include/opencascade )
set( OCC_LIBRARY_DIR ${OCCT_BINARY_DIR}/${OCCT_OS}${OCCT_BITS}/${OCCT_COMPILER}/lib )

if( NOT EXISTS ${OCC_INCLUDE_DIR}/Standard_Version.hxx )
    message( FATAL_ERROR "OpenCASCADE configure collected no headers in ${OCC_INCLUDE_DIR}" )
endif()

set( OCCT_LIBRARY_FILES )
set( OCCT_LIBRARY_NAMES )

foreach( tk IN LISTS OCCT_TOOLKITS )
    set( lib ${CMAKE_STATIC_LIBRARY_PREFIX}${tk}${CMAKE_STATIC_LIBRARY_SUFFIX} )
    list( APPEND OCCT_LIBRARY_FILES ${OCC_LIBRARY_DIR}/${lib} )
    list( APPEND OCCT_LIBRARY_NAMES ${lib} )
endforeach()

include( ProcessorCount )
ProcessorCount( OCCT_JOBS )

ExternalProject_Add( occt
    SOURCE_DIR        ${OCCT_SOURCE_DIR}
    BINARY_DIR        ${OCCT_BINARY_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND     ${CMAKE_COMMAND} --build ${OCCT_BINARY_DIR} --parallel ${OCCT_JOBS}
    INSTALL_COMMAND   ""
    BUILD_ALWAYS      TRUE
    BUILD_BYPRODUCTS  ${OCCT_LIBRARY_FILES}
    LOG_BUILD         TRUE
    LOG_OUTPUT_ON_FAILURE TRUE )

add_library( kicad_occt INTERFACE )
add_dependencies( kicad_occt occt )
target_link_libraries( kicad_occt INTERFACE ${OCCT_LIBRARY_FILES} ${FREETYPE_LIBRARIES} )

if( WIN32 )
    # Without it the OCCT headers declare dllimport
    target_compile_definitions( kicad_occt INTERFACE OCCT_STATIC_BUILD )
    target_link_libraries( kicad_occt INTERFACE
        ole32 oleaut32 windowscodecs uuid shell32 comdlg32 winspool ws2_32 wsock32 gdi32 user32
        advapi32 psapi winmm opengl32 )
elseif( APPLE )
    target_link_libraries( kicad_occt INTERFACE objc "-framework AppKit" "-framework IOKit" )
else()
    # Each module keeps its own copy private: no OCCT symbol is exported or interposed across
    # the kiface and kicad_3d_import
    string( REPLACE ";" ":" OCCT_EXCLUDE_LIBS "${OCCT_LIBRARY_NAMES}" )
    target_link_libraries( kicad_occt INTERFACE Fontconfig::Fontconfig Threads::Threads
                           ${CMAKE_DL_LIBS} )
    target_link_options( kicad_occt INTERFACE "LINKER:--exclude-libs,${OCCT_EXCLUDE_LIBS}" )
endif()

set( OCC_LIBRARIES kicad_occt )
set( OCC_VERSION_STRING ${OCCT_VERSION} )
set( OCC_FOUND TRUE )
