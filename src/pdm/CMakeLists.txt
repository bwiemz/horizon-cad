add_library(hz_pdm STATIC
    src/RevisionArchive.cpp
)

target_include_directories(hz_pdm
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_pdm
    PRIVATE
        nlohmann_json::nlohmann_json
)

add_library(Horizon::Pdm ALIAS hz_pdm)
