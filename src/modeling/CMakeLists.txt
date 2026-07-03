add_library(hz_modeling STATIC
    src/PrimitiveFactory.cpp
    src/ProfileValidator.cpp
    src/Extrude.cpp
    src/Revolve.cpp
    src/SolidTessellator.cpp
    src/ExactPredicates.cpp
    src/SurfaceSurfaceIntersection.cpp
    src/BooleanOp.cpp
    src/FilletOp.cpp
    src/ChamferOp.cpp
    src/MateGeometry.cpp
    src/AssemblySolver.cpp
    src/RingStack.cpp
    src/Loft.cpp
    src/Sweep.cpp
    src/Draft.cpp
    src/Shell.cpp
    src/Pattern.cpp
    src/ReferenceGeometry.cpp
)

find_package(Eigen3 CONFIG REQUIRED)

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
    PRIVATE
        Eigen3::Eigen
)

add_library(Horizon::Modeling ALIAS hz_modeling)
