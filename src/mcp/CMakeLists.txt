# horizon-mcp: Horizon CAD's tools over the Model Context Protocol (stdio), for
# MCP clients to build and change parts and drawings with. The session (the
# document and its file tools) is a library, so it is tested without a
# process.
add_library(hz_mcp STATIC
    src/McpSession.cpp
)

target_include_directories(hz_mcp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_mcp
    PUBLIC
        Horizon::AI
    PRIVATE
        Horizon::Document
        Horizon::FileIO
        Horizon::Render
        nlohmann_json::nlohmann_json
)

add_library(Horizon::MCP ALIAS hz_mcp)

add_executable(horizon-mcp main.cpp)
target_link_libraries(horizon-mcp PRIVATE Horizon::MCP Horizon::Version)

# Beside the application in a Windows or Linux package. (A macOS bundle has
# one program; build horizon-mcp from source there.)
if (NOT APPLE)
    install(TARGETS horizon-mcp RUNTIME DESTINATION bin)
endif()
