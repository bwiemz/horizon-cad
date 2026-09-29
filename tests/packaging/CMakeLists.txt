# The install tree a package is made from holds what a package needs (Phase
# 116). A macOS install runs Qt's deployment tool (macdeployqt), and a Windows
# one windeployqt, so this test is their check in CI too; on Windows it also
# checks that every DLL is in the install tree, with dumpbin, which is beside
# the compiler.
set(_hz_dumpbin_option)
if(WIN32)
    get_filename_component(_hz_compiler_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    find_program(HZ_DUMPBIN dumpbin HINTS "${_hz_compiler_dir}")
    if(NOT HZ_DUMPBIN)
        message(STATUS "Packaging install tree test: SKIPPED (no dumpbin found)")
        return()
    endif()
    set(_hz_dumpbin_option "-DDUMPBIN=${HZ_DUMPBIN}")
endif()
add_test(NAME PackagingInstallTree
    COMMAND ${CMAKE_COMMAND}
        -DBUILD_DIR=${CMAKE_BINARY_DIR}
        -DCONFIG=$<CONFIG>
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/install-check
        ${_hz_dumpbin_option}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/CheckInstallTree.cmake)
