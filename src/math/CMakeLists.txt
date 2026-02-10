add_library(hz_math STATIC
    src/Vec2.cpp
    src/Vec3.cpp
    src/Vec4.cpp
    src/Mat3.cpp
    src/Mat4.cpp
    src/Quaternion.cpp
    src/Transform.cpp
    src/BoundingBox.cpp
)

target_include_directories(hz_math
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(hz_math PUBLIC cxx_std_20)

add_library(Horizon::Math ALIAS hz_math)
