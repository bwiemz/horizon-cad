add_library(hz_modeling STATIC
    src/PrimitiveFactory.cpp
    src/Extrude.cpp
)

target_include_directories(hz_modeling
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_modeling
    PUBLIC
        Horizon::Math
        Horizon::Geometry
        Horizon::Topology
)

add_library(Horizon::Modeling ALIAS hz_modeling)
