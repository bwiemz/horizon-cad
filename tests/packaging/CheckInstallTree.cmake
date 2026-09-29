# Installs the build into a scratch prefix and checks that what a package
# needs is there: the executable, its samples, the licence and notices, and on
# Linux the desktop entry, AppStream metadata and icons, on macOS the bundle's
# Info.plist and icon. The desktop entry is validated when
# desktop-file-validate is installed. On Windows, every DLL the executable,
# the DLLs and the Qt plugins installed need is found in the install tree or
# in Windows itself (DUMPBIN names the tool that reads them).
#   cmake -DBUILD_DIR=<build> -DCONFIG=<config> -DWORK_DIR=<scratch> [-DDUMPBIN=<dumpbin>]
#         -P CheckInstallTree.cmake
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
if(WIN32)
    set(expected
        "bin/horizon.exe"
        "bin/qt.conf"
        "bin/samples/bracket.hzpart"
        "bin/samples/plate-and-pin.hzasm"
        "LICENSE"
        "THIRD_PARTY_NOTICES.md")
endif()
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
    if(IS_SYMLINK "${WORK_DIR}/${file}" AND NOT EXISTS "${WORK_DIR}/${file}")
        file(READ_SYMLINK "${WORK_DIR}/${file}" target)
        message(FATAL_ERROR "a link to nothing in the install tree: ${file} -> ${target}")
    endif()
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

# Every DLL found without the build tree: in the install tree, or in Windows
# itself. The build tree has every DLL beside its executables, so the tests
# passed there while the installer's program could not start (0xC0000135).
# The search path is Windows's own, so no DLL is found through a PATH that a
# developer's or CI's environment set.
if(WIN32)
    if(NOT DUMPBIN)
        message(FATAL_ERROR "DUMPBIN is not set: the DLLs cannot be read")
    endif()
    set(CMAKE_GET_RUNTIME_DEPENDENCIES_PLATFORM "windows+pe")
    set(CMAKE_GET_RUNTIME_DEPENDENCIES_TOOL "dumpbin")
    set(CMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND "${DUMPBIN}")
    set(ENV{PATH} "$ENV{SystemRoot}/System32;$ENV{SystemRoot}")
    file(GLOB dlls "${WORK_DIR}/bin/*.dll")
    file(GLOB_RECURSE plugins "${WORK_DIR}/*/plugins/*.dll")
    file(GET_RUNTIME_DEPENDENCIES
        EXECUTABLES "${WORK_DIR}/bin/horizon.exe"
        LIBRARIES ${dlls} ${plugins}
        DIRECTORIES "${WORK_DIR}/bin"
        PRE_EXCLUDE_REGEXES "^[Aa][Pp][Ii]-[Mm][Ss]-" "^[Ee][Xx][Tt]-[Mm][Ss]-"
        RESOLVED_DEPENDENCIES_VAR found
        UNRESOLVED_DEPENDENCIES_VAR missing)
    if(missing)
        list(JOIN missing ", " missing)
        message(FATAL_ERROR "the install tree lacks DLLs its program needs: ${missing}")
    endif()
endif()

# The metadata carries the version it was installed with.
if(WIN32)
    # Nothing configured to check: the installer is CPack's.
elseif(APPLE)
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
