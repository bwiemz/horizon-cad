# Installs the build into a scratch prefix and checks that what a package
# needs is there: the executable, its samples, the licence and notices, and on
# Linux the desktop entry, AppStream metadata and icons, on macOS the bundle's
# Info.plist and icon. The desktop entry is validated when
# desktop-file-validate is installed.
#   cmake -DBUILD_DIR=<build> -DCONFIG=<config> -DWORK_DIR=<scratch> -P CheckInstallTree.cmake
file(REMOVE_RECURSE "${WORK_DIR}")
execute_process(
    COMMAND ${CMAKE_COMMAND} --install "${BUILD_DIR}" --prefix "${WORK_DIR}" --config "${CONFIG}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
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
    # The bundle, with what it ships in Contents/Resources.
    set(contents "HorizonCAD.app/Contents")
    set(expected
        "${contents}/MacOS/HorizonCAD"
        "${contents}/Info.plist"
        "${contents}/Resources/horizon-cad.icns"
        "${contents}/Resources/samples/bracket.hzpart"
        "${contents}/Resources/samples/plate-and-pin.hzasm"
        "${contents}/Resources/doc/LICENSE"
        "${contents}/Resources/doc/THIRD_PARTY_NOTICES.md"
        "${contents}/Resources/qt.conf"
        "${contents}/PlugIns/platforms/libqcocoa.dylib")
endif()
foreach(file IN LISTS expected)
    if(NOT EXISTS "${WORK_DIR}/${file}")
        # What is there, and what the install said (the deployment tool's
        # output with it), to see why.
        file(GLOB_RECURSE installed RELATIVE "${WORK_DIR}" LIST_DIRECTORIES false "${WORK_DIR}/*")
        list(FILTER installed EXCLUDE REGEX "\\.framework/|/third-party/")
        list(JOIN installed "\n  " installed)
        message(FATAL_ERROR "missing from the install tree: ${file}\n"
                            "It holds (frameworks and licences left out):\n  ${installed}\n"
                            "The install said:\n${output}\n${errors}")
    endif()
endforeach()

# The metadata carries the version it was installed with.
if(APPLE)
    file(READ "${WORK_DIR}/${contents}/Info.plist" plist)
    if(plist MATCHES "@[A-Z_]+@")
        message(FATAL_ERROR "the Info.plist was not configured: ${plist}")
    endif()
    if(NOT plist MATCHES "<key>CFBundleExecutable</key>[ \t\r\n]*<string>HorizonCAD</string>")
        message(FATAL_ERROR "the Info.plist does not name the executable: ${plist}")
    endif()
else()
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
