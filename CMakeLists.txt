cmake_minimum_required(VERSION 3.28)

project(HorizonCAD
    VERSION 0.1.0
    LANGUAGES CXX
    DESCRIPTION "Open-source parametric 2D/3D CAD software"
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(HZ_BUILD_TESTS "Build unit and integration tests" ON)
option(HZ_ENABLE_SANITIZERS "Enable address/undefined sanitizers" OFF)

# CMake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(HorizonVersion)
include(CompilerWarnings)

if(HZ_ENABLE_SANITIZERS)
    include(Sanitizers)
endif()

# Find packages (provided by vcpkg)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets)
find_package(OpenGL REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# All source libraries
add_subdirectory(src)

# Tests
if(HZ_BUILD_TESTS)
    find_package(GTest CONFIG REQUIRED)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installer
include(CPack)
include(cmake/CPack.cmake)
