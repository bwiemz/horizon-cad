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

# The project uses no C++20 modules. Scanning for them costs build time and, on
# CMake >= 3.28 with GCC >= 14, adds -fmodule-mapper/-fdeps-format flags to
# compile_commands.json that clang-tidy cannot parse.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

# Options
option(HZ_BUILD_TESTS "Build unit and integration tests" ON)
option(HZ_ENABLE_SANITIZERS "Enable address/undefined sanitizers" OFF)
option(HZ_ENABLE_SCRIPTING "Build the embedded Python scripting module" ON)

# CMake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(HorizonVersion)
include(CompilerWarnings)
include(Sanitizers)

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

# Warnings and sanitizers apply to every first-party target. They are applied
# here, once every subdirectory has declared its targets, so a new module or
# test binary is covered without having to opt in.
function(hz_collect_targets out dir)
    get_property(targets DIRECTORY "${dir}" PROPERTY BUILDSYSTEM_TARGETS)
    get_property(subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
    foreach(sub IN LISTS subdirs)
        hz_collect_targets(sub_targets "${sub}")
        list(APPEND targets ${sub_targets})
    endforeach()
    set(${out} ${targets} PARENT_SCOPE)
endfunction()

hz_collect_targets(_hz_targets "${CMAKE_CURRENT_SOURCE_DIR}")
foreach(_hz_target IN LISTS _hz_targets)
    get_target_property(_hz_type ${_hz_target} TYPE)
    if(_hz_type MATCHES "^(STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY|EXECUTABLE)$")
        hz_set_warnings(${_hz_target})
        if(HZ_ENABLE_SANITIZERS)
            hz_enable_sanitizers(${_hz_target})
        endif()
    endif()
endforeach()

# Installer
include(CPack)
include(cmake/CPack.cmake)
