add_library(hz_topology STATIC
    src/TopologyID.cpp
    src/Solid.cpp
    src/EulerOps.cpp
    src/Queries.cpp
)

target_include_directories(hz_topology
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_topology
    PUBLIC
        Horizon::Math
        Horizon::Geometry
)

add_library(Horizon::Topology ALIAS hz_topology)
