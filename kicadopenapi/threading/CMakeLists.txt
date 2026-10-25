# kicadopenapi threading core (owner strand, read leases, change sets) built without KiCad:
# std + header-only Boost.Test, for quick and sanitizer runs on every OS.
#   cmake -S kicadopenapi/threading -B <dir> [-DKOPENAPI_SANITIZE=thread|address]
# In the KiCad tree the same sources are part of kicommon and the test is part of qa_common.
cmake_minimum_required( VERSION 3.16 )
project( kicadopenapi-threading LANGUAGES CXX )

if( POLICY CMP0167 )
    cmake_policy( SET CMP0167 NEW )
endif()

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

find_package( Boost REQUIRED )
find_package( Threads REQUIRED )

set( KOPENAPI_SANITIZE "" CACHE STRING "thread or address" )

add_library( kicadopenapi_threading STATIC
    kicadopenapi_doc_gate.cpp
    kicadopenapi_doc_owner.cpp
    )

target_compile_definitions( kicadopenapi_threading PUBLIC KOPENAPI_THREADING_API= )
target_include_directories( kicadopenapi_threading PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. )
target_link_libraries( kicadopenapi_threading PUBLIC Threads::Threads )

add_executable( qa_kicadopenapi_threading
    standalone_test_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../qa/tests/common/test_kicadopenapi_owner.cpp
    )

target_link_libraries( qa_kicadopenapi_threading PRIVATE kicadopenapi_threading Boost::boost )

if( MSVC )
    target_compile_options( kicadopenapi_threading PRIVATE /W4 )
else()
    target_compile_options( kicadopenapi_threading PRIVATE -Wall -Wextra -Wpedantic )
endif()

if( KOPENAPI_SANITIZE )
    foreach( target kicadopenapi_threading qa_kicadopenapi_threading )
        target_compile_options( ${target} PRIVATE -fsanitize=${KOPENAPI_SANITIZE} -g -O1 -fno-omit-frame-pointer )
        target_link_options( ${target} PRIVATE -fsanitize=${KOPENAPI_SANITIZE} )
    endforeach()
endif()
