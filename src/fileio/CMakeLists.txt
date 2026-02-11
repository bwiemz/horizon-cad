add_library(hz_fileio STATIC
    src/NativeFormat.cpp
    src/StlExporter.cpp
)

target_include_directories(hz_fileio
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_fileio
    PUBLIC
        Horizon::Math
        Horizon::Geometry
        Horizon::Topology
        Horizon::Drafting
        Horizon::Constraint
        Horizon::Document
        nlohmann_json::nlohmann_json
)

add_library(Horizon::FileIO ALIAS hz_fileio)
