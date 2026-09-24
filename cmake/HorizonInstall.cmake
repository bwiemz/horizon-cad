# What `cmake --install` and CPack put in a package besides the executable
# (Phase 116): the licence and third-party notices, the licence of every
# library vcpkg built, and on Linux the desktop entry, AppStream metadata,
# MIME types and icons. Translations and the Qt runtime are installed by
# src/app/CMakeLists.txt, next to the executable.
include(GNUInstallDirs)

if(WIN32)
    set(HZ_INSTALL_DOCDIR ".")
else()
    set(HZ_INSTALL_DOCDIR "${CMAKE_INSTALL_DATADIR}/doc/horizon-cad")
endif()
install(FILES
    "${CMAKE_SOURCE_DIR}/LICENSE"
    "${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
    "${CMAKE_SOURCE_DIR}/README.md"
    "${CMAKE_SOURCE_DIR}/CHANGELOG.md"
    DESTINATION "${HZ_INSTALL_DOCDIR}")

# The licence text of every library vcpkg built into this one, as vcpkg
# records it. Ports that only build (GoogleTest, pkg-config, vcpkg's own CMake
# helpers) are not in a package.
if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
    file(GLOB _hz_copyrights "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/share/*/copyright")
    foreach(_hz_copyright IN LISTS _hz_copyrights)
        get_filename_component(_hz_port_dir "${_hz_copyright}" DIRECTORY)
        get_filename_component(_hz_port "${_hz_port_dir}" NAME)
        if(_hz_port MATCHES "^(gtest|pkgconf|pkgconfig|vcpkg-.*)$")
            continue()
        endif()
        install(FILES "${_hz_copyright}" DESTINATION "${HZ_INSTALL_DOCDIR}/third-party/${_hz_port}")
    endforeach()
endif()

if(UNIX AND NOT APPLE)
    # Desktop integration, under the application's reverse-DNS id.
    set(HZ_APP_ID "io.github.bwiemz.HorizonCAD")
    # The release date AppStream wants; honours SOURCE_DATE_EPOCH, so a
    # reproducible build stays reproducible.
    string(TIMESTAMP HZ_METAINFO_DATE "%Y-%m-%d" UTC)
    configure_file("${CMAKE_SOURCE_DIR}/packaging/linux/${HZ_APP_ID}.metainfo.xml"
                   "${CMAKE_BINARY_DIR}/packaging/${HZ_APP_ID}.metainfo.xml" @ONLY)
    install(FILES "${CMAKE_SOURCE_DIR}/packaging/linux/${HZ_APP_ID}.desktop"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/applications")
    install(FILES "${CMAKE_BINARY_DIR}/packaging/${HZ_APP_ID}.metainfo.xml"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/metainfo")
    install(FILES "${CMAKE_SOURCE_DIR}/packaging/linux/horizon-cad-mime.xml"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/mime/packages"
            RENAME "${HZ_APP_ID}.xml")
    foreach(_hz_size 16 24 32 48 64 128 256 512)
        install(FILES "${CMAKE_SOURCE_DIR}/packaging/icons/horizon-cad-${_hz_size}.png"
                DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/${_hz_size}x${_hz_size}/apps"
                RENAME "${HZ_APP_ID}.png")
    endforeach()
    install(FILES "${CMAKE_SOURCE_DIR}/packaging/icons/horizon-cad.svg"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps"
            RENAME "${HZ_APP_ID}.svg")
endif()
