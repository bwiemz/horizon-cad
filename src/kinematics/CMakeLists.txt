add_library(hz_kinematics STATIC
    src/ForwardKinematics.cpp
)

target_include_directories(hz_kinematics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_kinematics
    PUBLIC
        Horizon::Math
)

add_library(Horizon::Kinematics ALIAS hz_kinematics)
