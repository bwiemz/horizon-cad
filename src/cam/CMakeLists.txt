add_library(hz_cam STATIC
    src/Toolpath.cpp
    src/GcodeWriter.cpp
    src/FeedsAndSpeeds.cpp
)

target_include_directories(hz_cam
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_cam
    PUBLIC
        Horizon::Math
)

add_library(Horizon::Cam ALIAS hz_cam)
