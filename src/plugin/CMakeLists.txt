add_library(hz_plugin STATIC
    src/PluginRegistry.cpp
)

target_include_directories(hz_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_plugin
    PRIVATE
        nlohmann_json::nlohmann_json
)

add_library(Horizon::Plugin ALIAS hz_plugin)
