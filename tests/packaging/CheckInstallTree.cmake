# Installs the build into a scratch prefix and checks that what a package
# needs is there: the executable, its samples, the licence and notices, and on
# Linux the desktop entry, AppStream metadata and icons, on macOS the bundle's
# Info.plist and icon. The desktop entry is validated when
# desktop-file-validate is installed. On Windows, every DLL the executable,
# the DLLs and the Qt plugins installed need is found in the install tree or
# in Windows itself, and a release carries the Visual C++ runtime (DUMPBIN
# names the tool that reads them).
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
# CMake looks for a DLL as Windows does: beside what needs it, then in
# Windows's folders, then in DIRECTORIES; never on a PATH. A DLL in System32
# is taken as Windows's own, and what it needs in turn is not read: the check
# cannot tell one that other software put there. The Visual C++ runtime is
# the one it knows. It is not Windows's, though Visual Studio puts it in
# System32: a release carries it in bin, for a Windows that never had it
# installed. Its debug build may not be redistributed, so a Debug install
# takes it from Visual Studio's.
if(WIN32)
    if(NOT DUMPBIN)
        message(FATAL_ERROR "DUMPBIN is not set: the DLLs cannot be read")
    endif()
    if(POLICY CMP0207)
        cmake_policy(SET CMP0207 NEW)  # paths are matched with forward slashes
    endif()
    set(CMAKE_GET_RUNTIME_DEPENDENCIES_PLATFORM "windows+pe")
    set(CMAKE_GET_RUNTIME_DEPENDENCIES_TOOL "dumpbin")
    set(CMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND "${DUMPBIN}")
    set(vc_runtime "[/\\\\]([Mm][Ss][Vv][Cc][Pp]|[Vv][Cc][Rr][Uu][Nn][Tt][Ii][Mm][Ee]|[Cc][Oo][Nn][Cc][Rr][Tt]|[Vv][Cc][Oo][Mm][Pp]|[Vv][Cc][Cc][Oo][Rr][Ll][Ii][Bb])[0-9][^/\\\\]*$")
    file(GLOB dlls "${WORK_DIR}/bin/*.dll")
    file(GLOB_RECURSE plugins "${WORK_DIR}/*/plugins/*.dll")
    file(GET_RUNTIME_DEPENDENCIES
        EXECUTABLES "${WORK_DIR}/bin/horizon.exe"
        LIBRARIES ${dlls}
        MODULES ${plugins}
        DIRECTORIES "${WORK_DIR}/bin"
        PRE_EXCLUDE_REGEXES "^[Aa][Pp][Ii]-[Mm][Ss]-" "^[Ee][Xx][Tt]-[Mm][Ss]-"
        POST_INCLUDE_REGEXES "${vc_runtime}"
        POST_EXCLUDE_REGEXES "[/\\\\][Ss][Yy][Ss][Tt][Ee][Mm]32[/\\\\]"
        RESOLVED_DEPENDENCIES_VAR found
        UNRESOLVED_DEPENDENCIES_VAR missing
        CONFLICTING_DEPENDENCIES_PREFIX conflict)
    if(missing)
        list(JOIN missing ", " missing)
        message(FATAL_ERROR "the install tree lacks DLLs its program needs: ${missing}")
    endif()
    # A DLL found in two folders is found. A plugin, not beside the runtime
    # in bin, finds Windows's copy of it first; the program finds bin's.
    set(paths ${found})
    foreach(name IN LISTS conflict_FILENAMES)
        list(APPEND paths ${conflict_${name}})
        list(JOIN conflict_${name} ", " where)
        message(STATUS "${name} is in two places: ${where}")
    endforeach()
    if(NOT CONFIG STREQUAL "Debug")
        set(lacking)
        foreach(path IN LISTS paths)
            if(path MATCHES "${vc_runtime}")
                get_filename_component(name "${path}" NAME)
                if(NOT EXISTS "${WORK_DIR}/bin/${name}")
                    list(APPEND lacking "${name}")
                endif()
            endif()
        endforeach()
        if(lacking)
            list(REMOVE_DUPLICATES lacking)
            list(JOIN lacking ", " lacking)
            message(FATAL_ERROR "the install tree lacks the Visual C++ runtime's ${lacking}, "
                                "which a Windows without Visual Studio or its "
                                "redistributable does not have")
        endif()
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
