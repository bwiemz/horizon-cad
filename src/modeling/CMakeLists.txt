add_library(hz_modeling STATIC
    src/PrimitiveFactory.cpp
    src/ProfileValidator.cpp
    src/Extrude.cpp
    src/Revolve.cpp
    src/SolidTessellator.cpp
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
        Horizon::Render
        Horizon::Drafting
)

add_library(Horizon::Modeling ALIAS hz_modeling)
