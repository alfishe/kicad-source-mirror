# kicad-mcp-bridge: stdio MCP server fronting kicadopenapi /mcp endpoints of all KiCad
# processes (instance discovery/selection/start). std + sockets + header-only nlohmann/json.
# Builds standalone (cmake -S kicadopenapi/bridge) or as part of the KiCad tree.
cmake_minimum_required( VERSION 3.16 )

if( NOT DEFINED PROJECT_NAME )
    project( kicad-mcp-bridge LANGUAGES CXX )
endif()

set( KOPENAPI_PLATFORM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../platform )

# All platform files are listed; each compiles only on its OS (#if guards inside)
add_executable( kicad-mcp-bridge
    main.cpp
    instances.cpp
    mini-http.cpp
    ${KOPENAPI_PLATFORM_DIR}/platform.cpp
    ${KOPENAPI_PLATFORM_DIR}/platform_posix.cpp
    ${KOPENAPI_PLATFORM_DIR}/platform_macos.cpp
    ${KOPENAPI_PLATFORM_DIR}/platform_linux.cpp
    ${KOPENAPI_PLATFORM_DIR}/platform_windows.cpp
    )

# Header-only nlohmann/json from the KiCad tree (no link dependency)
target_include_directories( kicad-mcp-bridge PRIVATE
    ${KOPENAPI_PLATFORM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/nlohmann_json
    )

set_target_properties( kicad-mcp-bridge PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    )

if( WIN32 )
    target_link_libraries( kicad-mcp-bridge PRIVATE ws2_32 )
endif()

# Inside the KiCad tree: ship next to kicad / kicad-cli (the bridge's default lookup)
if( APPLE AND DEFINED OSX_BUNDLE_BUILD_BIN_DIR )
    set_target_properties( kicad-mcp-bridge PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${OSX_BUNDLE_BUILD_BIN_DIR}
        )
elseif( DEFINED KICAD_BIN )
    install( TARGETS kicad-mcp-bridge
        DESTINATION ${KICAD_BIN}
        COMPONENT binary
        )
endif()

