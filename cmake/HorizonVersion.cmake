# The version: set once, in project(). It reaches the code through a generated
# header (Horizon::Version) and the installer through CPack.
set(HZ_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(HZ_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(HZ_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(HZ_VERSION_STRING "${PROJECT_VERSION}")

set(HZ_GENERATED_INCLUDE_DIR "${CMAKE_BINARY_DIR}/generated/include")
configure_file("${CMAKE_SOURCE_DIR}/cmake/Version.h.in"
               "${HZ_GENERATED_INCLUDE_DIR}/horizon/Version.h" @ONLY)

# The source revision, for Help > About. Written now, so the header exists as
# soon as CMake has configured (the Static Analysis job never builds), and
# again on every build, so a new commit shows without reconfiguring.
find_package(Git QUIET)
set(_hz_revision_args
    -DGIT=${GIT_EXECUTABLE}
    -DSRC=${CMAKE_SOURCE_DIR}
    -DOUT=${HZ_GENERATED_INCLUDE_DIR}/horizon/Revision.h
    -P ${CMAKE_SOURCE_DIR}/cmake/WriteRevision.cmake)
execute_process(COMMAND ${CMAKE_COMMAND} ${_hz_revision_args})
add_custom_target(hz_version_revision
    COMMAND ${CMAKE_COMMAND} ${_hz_revision_args}
    BYPRODUCTS ${HZ_GENERATED_INCLUDE_DIR}/horizon/Revision.h
    COMMENT "")

add_library(hz_version INTERFACE)
target_include_directories(hz_version INTERFACE $<BUILD_INTERFACE:${HZ_GENERATED_INCLUDE_DIR}>)
add_dependencies(hz_version hz_version_revision)
add_library(Horizon::Version ALIAS hz_version)
