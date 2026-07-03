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
option(HZ_ENABLE_SCRIPTING "Build the embedded Python scripting module" ON)

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

# Optional embedded-Python scripting. Uses the system Python interpreter (found
# on both Linux and Windows CI runners) plus header-only pybind11, so it does
# not require vcpkg to build CPython from source.
set(HZ_SCRIPTING_ENABLED OFF)
if(HZ_ENABLE_SCRIPTING)
    find_package(Python3 COMPONENTS Interpreter Development.Embed QUIET)
    # Make pybind11 reuse the interpreter FindPython3 located rather than its
    # legacy FindPythonLibsNew, so the two never disagree (important on Windows).
    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 CONFIG QUIET)
    if(Python3_FOUND AND pybind11_FOUND)
        set(HZ_SCRIPTING_ENABLED ON)
        message(STATUS "Horizon scripting: ENABLED "
                       "(Python ${Python3_VERSION}, pybind11 ${pybind11_VERSION})")
    else()
        message(STATUS "Horizon scripting: DISABLED "
                       "(Python3 Development.Embed / pybind11 not found)")
    endif()
endif()

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
