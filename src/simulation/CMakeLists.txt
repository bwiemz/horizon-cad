add_library(hz_simulation STATIC
    src/Material.cpp
    src/LinearElement.cpp
)

find_package(Eigen3 CONFIG REQUIRED)

target_include_directories(hz_simulation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_simulation
    PUBLIC
        Horizon::Math
    PRIVATE
        Eigen3::Eigen
)

add_library(Horizon::Simulation ALIAS hz_simulation)
