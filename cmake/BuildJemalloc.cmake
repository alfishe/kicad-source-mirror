# Builds the vendored jemalloc (thirdparty/jemalloc) at configure time, once per build directory
# and again when its version or options change. Output: JEMALLOC_STATIC_LIBRARY.

set( JEMALLOC_VERSION 5.3.0 )
set( JEMALLOC_SOURCE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/jemalloc/jemalloc-${JEMALLOC_VERSION} )
set( JEMALLOC_BUILD_DIR ${CMAKE_BINARY_DIR}/thirdparty/jemalloc )
set( JEMALLOC_STATIC_LIBRARY ${JEMALLOC_BUILD_DIR}/lib/libjemalloc_pic.a )

# Idle memory back to the OS within about a second (POC-003); MALLOC_CONF still overrides at run time
set( JEMALLOC_CONFIGURE_ARGS
     --with-malloc-conf=background_thread:true,dirty_decay_ms:1000,muzzy_decay_ms:0
     --disable-cxx --with-pic --disable-libdl )

string( SHA1 JEMALLOC_STAMP "${JEMALLOC_VERSION};${JEMALLOC_CONFIGURE_ARGS};${CMAKE_C_COMPILER}" )
set( JEMALLOC_STAMP_FILE ${JEMALLOC_BUILD_DIR}/kicad-build.stamp )

set( JEMALLOC_STAMP_OLD "" )

if( EXISTS ${JEMALLOC_STAMP_FILE} )
    file( READ ${JEMALLOC_STAMP_FILE} JEMALLOC_STAMP_OLD )
endif()

if( NOT EXISTS ${JEMALLOC_STATIC_LIBRARY} OR NOT JEMALLOC_STAMP_OLD STREQUAL JEMALLOC_STAMP )
    message( STATUS "Building jemalloc ${JEMALLOC_VERSION} (thirdparty/jemalloc)" )
    file( REMOVE_RECURSE ${JEMALLOC_BUILD_DIR} )
    file( MAKE_DIRECTORY ${JEMALLOC_BUILD_DIR} )

    execute_process( COMMAND ${CMAKE_COMMAND} -E env CC=${CMAKE_C_COMPILER}
                             sh ${JEMALLOC_SOURCE_DIR}/configure ${JEMALLOC_CONFIGURE_ARGS}
                     WORKING_DIRECTORY ${JEMALLOC_BUILD_DIR}
                     RESULT_VARIABLE result
                     OUTPUT_FILE ${JEMALLOC_BUILD_DIR}/configure.log
                     ERROR_FILE ${JEMALLOC_BUILD_DIR}/configure.log )

    if( NOT result EQUAL 0 )
        message( FATAL_ERROR "jemalloc configure failed: ${JEMALLOC_BUILD_DIR}/configure.log" )
    endif()

    include( ProcessorCount )
    ProcessorCount( jobs )

    execute_process( COMMAND make -j${jobs} build_lib_static
                     WORKING_DIRECTORY ${JEMALLOC_BUILD_DIR}
                     RESULT_VARIABLE result
                     OUTPUT_FILE ${JEMALLOC_BUILD_DIR}/build.log
                     ERROR_FILE ${JEMALLOC_BUILD_DIR}/build.log )

    if( NOT result EQUAL 0 OR NOT EXISTS ${JEMALLOC_STATIC_LIBRARY} )
        message( FATAL_ERROR "jemalloc build failed: ${JEMALLOC_BUILD_DIR}/build.log" )
    endif()

    file( WRITE ${JEMALLOC_STAMP_FILE} "${JEMALLOC_STAMP}" )
endif()
