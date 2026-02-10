add_library(hz_geometry STATIC
    src/curves/Curve2D.cpp
    src/curves/Line2D.cpp
    src/curves/Circle2D.cpp
    src/curves/Arc2D.cpp
    src/curves/Line3D.cpp
)

target_include_directories(hz_geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_geometry
    PUBLIC Horizon::Math
)

add_library(Horizon::Geometry ALIAS hz_geometry)
