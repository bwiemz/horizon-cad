# Installs the build into a scratch prefix and checks that what a package
# needs is there: the executable, its samples, the licence and notices, and on Linux the
# desktop entry, AppStream metadata and icons. The desktop entry is validated
# when desktop-file-validate is installed.
#   cmake -DBUILD_DIR=<build> -DCONFIG=<config> -DWORK_DIR=<scratch> -P CheckInstallTree.cmake
file(REMOVE_RECURSE "${WORK_DIR}")
execute_process(
    COMMAND ${CMAKE_COMMAND} --install "${BUILD_DIR}" --prefix "${WORK_DIR}" --config "${CONFIG}"
    RESULT_VARIABLE result
    OUTPUT_QUIET
    ERROR_VARIABLE errors)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "cmake --install failed:\n${errors}")
endif()

set(app_id "io.github.bwiemz.HorizonCAD")
set(expected
    "bin/horizon"
    "bin/samples/bracket.hzpart"
    "bin/samples/plate-and-pin.hzasm"
    "share/doc/horizon-cad/LICENSE"
    "share/doc/horizon-cad/THIRD_PARTY_NOTICES.md"
    "share/applications/${app_id}.desktop"
    "share/metainfo/${app_id}.metainfo.xml"
    "share/mime/packages/${app_id}.xml"
    "share/icons/hicolor/256x256/apps/${app_id}.png"
    "share/icons/hicolor/scalable/apps/${app_id}.svg")
if(APPLE)
    set(expected "bin/horizon" "bin/samples/bracket.hzpart" "bin/samples/plate-and-pin.hzasm"
        "share/doc/horizon-cad/LICENSE" "share/doc/horizon-cad/THIRD_PARTY_NOTICES.md")
endif()
foreach(file IN LISTS expected)
    if(NOT EXISTS "${WORK_DIR}/${file}")
        message(FATAL_ERROR "missing from the install tree: ${file}")
    endif()
endforeach()

# The metadata carries the version it was installed with.
if(NOT APPLE)
    file(READ "${WORK_DIR}/share/metainfo/${app_id}.metainfo.xml" metainfo)
    if(metainfo MATCHES "@[A-Z_]+@")
        message(FATAL_ERROR "the AppStream metadata was not configured: ${metainfo}")
    endif()
    find_program(DESKTOP_FILE_VALIDATE desktop-file-validate)
    if(DESKTOP_FILE_VALIDATE)
        execute_process(
            COMMAND ${DESKTOP_FILE_VALIDATE} "${WORK_DIR}/share/applications/${app_id}.desktop"
            RESULT_VARIABLE result
            OUTPUT_VARIABLE output
            ERROR_VARIABLE output)
        if(NOT result EQUAL 0)
            message(FATAL_ERROR "the desktop entry is not valid:\n${output}")
        endif()
    endif()
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "install tree complete")
