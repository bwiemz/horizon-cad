add_library(hz_math STATIC
    src/Vec2.cpp
    src/Vec3.cpp
    src/Vec4.cpp
    src/Mat3.cpp
    src/Mat4.cpp
    src/Quaternion.cpp
    src/Transform.cpp
    src/BoundingBox.cpp
    src/Expression.cpp
    src/Units.cpp
    src/Quantity.cpp
    src/CharConv.cpp
    src/RobustPredicates.cpp
)

# The exact predicates rely on error-free transformations; a fused a*b+c would break them.
set_source_files_properties(src/RobustPredicates.cpp PROPERTIES COMPILE_OPTIONS
    "$<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-ffp-contract=off>")

target_include_directories(hz_math
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(hz_math PUBLIC cxx_std_20)

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(hz_math PUBLIC nlohmann_json::nlohmann_json)

add_library(Horizon::Math ALIAS hz_math)
