# The install tree a package is made from holds what a package needs (Phase
# 116). Not on Windows yet: a Windows install runs Qt's deployment tool, which
# the release workflow (Phase 117) exercises when it builds the installer.
if(NOT WIN32)
    add_test(NAME PackagingInstallTree
        COMMAND ${CMAKE_COMMAND}
            -DBUILD_DIR=${CMAKE_BINARY_DIR}
            -DCONFIG=$<CONFIG>
            -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/install-check
            -P ${CMAKE_CURRENT_SOURCE_DIR}/CheckInstallTree.cmake)
endif()
