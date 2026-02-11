add_library(hz_constraint STATIC
    src/GeometryRef.cpp
    src/Constraint.cpp
    src/ConstraintSystem.cpp
    src/ParameterTable.cpp
    src/SketchSolver.cpp
)

target_include_directories(hz_constraint
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

find_package(Eigen3 CONFIG REQUIRED)

target_link_libraries(hz_constraint
    PUBLIC
        Horizon::Math
        Horizon::Geometry
        Horizon::Drafting
        Eigen3::Eigen
)

add_library(Horizon::Constraint ALIAS hz_constraint)
