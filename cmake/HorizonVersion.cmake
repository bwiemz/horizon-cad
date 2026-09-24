# Version information
set(HZ_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(HZ_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(HZ_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(HZ_VERSION_STRING "${HZ_VERSION_MAJOR}.${HZ_VERSION_MINOR}.${HZ_VERSION_PATCH}")

# The source revision, for Help > About. Taken when CMake configures, so a
# build made without reconfiguring shows the revision it was configured at.
set(HZ_GIT_REVISION "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE _hz_git_revision
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _hz_git_result)
    if(_hz_git_result EQUAL 0 AND _hz_git_revision)
        set(HZ_GIT_REVISION "${_hz_git_revision}")
    endif()
endif()
