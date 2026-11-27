# Builds the vendored nng (thirdparty/nng) as a static library at configure time, once per build directory
# and again when its version or options change. Output: NNG_LIBRARY (the imported target nng::nng with its
# system libraries) and NNG_INCLUDE_DIR.

set( NNG_VERSION 2.1.0 )
set( NNG_SOURCE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/nng/nng-${NNG_VERSION} )
set( NNG_BUILD_DIR ${CMAKE_BINARY_DIR}/thirdparty/nng/build )
set( NNG_INSTALL_DIR ${CMAKE_BINARY_DIR}/thirdparty/nng/install )

set( NNG_CMAKE_ARGS
     -DBUILD_SHARED_LIBS=OFF
     -DCMAKE_POSITION_INDEPENDENT_CODE=ON
     -DCMAKE_BUILD_TYPE=Release
     -DNNG_TESTS=OFF
     -DNNG_TOOLS=OFF
     -DNNG_ENABLE_NNGCAT=OFF
     -DNNG_ENABLE_TLS=OFF
     -DNNG_ENABLE_HTTP=OFF
     -DNNG_TRANSPORT_WS=OFF
     -DNNG_TRANSPORT_WSS=OFF
     -DCMAKE_INSTALL_LIBDIR=lib
     -DCMAKE_INSTALL_PREFIX=${NNG_INSTALL_DIR}
     -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} )

if( CMAKE_OSX_ARCHITECTURES )
    list( APPEND NNG_CMAKE_ARGS -DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES} )
endif()

if( CMAKE_OSX_DEPLOYMENT_TARGET )
    list( APPEND NNG_CMAKE_ARGS -DCMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET} )
endif()

string( SHA1 NNG_STAMP "${NNG_VERSION};${NNG_CMAKE_ARGS};${CMAKE_GENERATOR}" )
set( NNG_STAMP_FILE ${CMAKE_BINARY_DIR}/thirdparty/nng/kicad-build.stamp )
set( NNG_STAMP_OLD "" )

if( EXISTS ${NNG_STAMP_FILE} )
    file( READ ${NNG_STAMP_FILE} NNG_STAMP_OLD )
endif()

if( NOT EXISTS ${NNG_INSTALL_DIR}/lib/cmake/nng/nng-config.cmake OR NOT NNG_STAMP_OLD STREQUAL NNG_STAMP )
    message( STATUS "Building nng ${NNG_VERSION} (thirdparty/nng)" )
    file( REMOVE_RECURSE ${CMAKE_BINARY_DIR}/thirdparty/nng )
    file( MAKE_DIRECTORY ${NNG_BUILD_DIR} )
    set( NNG_LOG ${CMAKE_BINARY_DIR}/thirdparty/nng/build.log )

    execute_process( COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} -S ${NNG_SOURCE_DIR} -B ${NNG_BUILD_DIR}
                             ${NNG_CMAKE_ARGS}
                     RESULT_VARIABLE result OUTPUT_FILE ${NNG_LOG} ERROR_FILE ${NNG_LOG} )

    if( result EQUAL 0 )
        include( ProcessorCount )
        ProcessorCount( jobs )
        execute_process( COMMAND ${CMAKE_COMMAND} --build ${NNG_BUILD_DIR} --target install --parallel ${jobs}
                         RESULT_VARIABLE result OUTPUT_FILE ${NNG_LOG}.build ERROR_FILE ${NNG_LOG}.build )
    endif()

    if( NOT result EQUAL 0 OR NOT EXISTS ${NNG_INSTALL_DIR}/lib/cmake/nng/nng-config.cmake )
        message( FATAL_ERROR "nng build failed: ${NNG_LOG}" )
    endif()

    file( WRITE ${NNG_STAMP_FILE} "${NNG_STAMP}" )
endif()

find_package( nng ${NNG_VERSION} CONFIG REQUIRED PATHS ${NNG_INSTALL_DIR} NO_DEFAULT_PATH )

# plain -I, ahead of every -isystem: a host nng 1.x header (Homebrew's /opt/homebrew/include) must not win
set_target_properties( nng::nng PROPERTIES SYSTEM FALSE )

set( NNG_LIBRARY nng::nng )
set( NNG_INCLUDE_DIR ${NNG_INSTALL_DIR}/include )
