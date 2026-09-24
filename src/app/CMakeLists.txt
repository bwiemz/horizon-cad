set(CMAKE_AUTORCC ON)

add_executable(horizon WIN32
    main.cpp
    resources/resources.qrc
)

target_link_libraries(horizon
    PRIVATE
        Horizon::UI
        Horizon::Version
        spdlog::spdlog
)

# UTF-8 as the process code page, so UTF-8 paths survive narrow Windows APIs.
# MSVC's linker merges a .manifest source into the executable's manifest.
if (MSVC)
    target_sources(horizon PRIVATE horizon.manifest)
endif()
# The executable's icon. The resource script names it by absolute path:
# rc.exe resolves a relative path against its working directory, which is the
# build directory under Ninja but the project's directory under Visual Studio.
if (WIN32)
    set(HZ_APP_ICON "${CMAKE_SOURCE_DIR}/packaging/icons/horizon-cad.ico")
    configure_file(horizon.rc.in ${CMAKE_CURRENT_BINARY_DIR}/horizon.rc @ONLY)
    target_sources(horizon PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/horizon.rc)
endif()

install(TARGETS horizon
    RUNTIME DESTINATION bin
)

# The Qt runtime next to the executable in a Windows package: Qt's DLLs and
# the plugins it needs (the build tree deploys them itself; see below). Linux
# packages get Qt from linuxdeploy's Qt plugin (packaging/linux).
if (WIN32 AND COMMAND qt_generate_deploy_app_script)
    qt_generate_deploy_app_script(
        TARGET horizon
        OUTPUT_SCRIPT _hz_deploy_script
        NO_TRANSLATIONS
        NO_UNSUPPORTED_PLATFORM_ERROR)
    install(SCRIPT ${_hz_deploy_script})
endif()

# Deploy the current Qt plugins next to the executable on every build. Without
# this, a Qt upgrade refreshes the copied Qt*.dll runtimes but can leave a
# STALE platform/style plugin behind (vcpkg's applocal does not overwrite an
# existing one), and Qt then refuses to load a plugin built against a different
# version — "no Qt platform plugin could be initialized" at startup. Copying
# from the plugins' imported targets with copy_if_different always overwrites a
# mismatched plugin with the one matching the linked Qt.
if (WIN32)
    set(_hz_qt_plugins "")
    if (TARGET Qt6::QWindowsIntegrationPlugin)          # platforms/qwindows[d].dll
        list(APPEND _hz_qt_plugins platforms Qt6::QWindowsIntegrationPlugin)
    endif()
    if (TARGET Qt6::QModernWindowsStylePlugin)          # styles/qmodernwindowsstyle[d].dll
        list(APPEND _hz_qt_plugins styles Qt6::QModernWindowsStylePlugin)
    endif()
    while (_hz_qt_plugins)
        list(POP_FRONT _hz_qt_plugins _hz_plugin_dir _hz_plugin_target)
        add_custom_command(TARGET horizon POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                    $<TARGET_FILE_DIR:horizon>/${_hz_plugin_dir}
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${_hz_plugin_target}>
                    $<TARGET_FILE_DIR:horizon>/${_hz_plugin_dir}/
            COMMENT "Deploying Qt ${_hz_plugin_dir} plugin next to horizon executable"
            VERBATIM)
    endwhile()
endif()

# Translations (Phase 77): compile translations/*.ts to .qm next to the
# executable when Qt Linguist tools are present. Quiet gating (mirrors the
# Vulkan/scripting pattern) — default builds don't require qttools; the
# shipped .ts sources are the ground truth either way.
find_package(Qt6 COMPONENTS LinguistTools QUIET)
if (TARGET Qt6::lrelease)
    message(STATUS "Horizon translations: ENABLED (lrelease found)")
    file(GLOB HZ_TS_FILES ${CMAKE_SOURCE_DIR}/translations/horizon_*.ts)
    qt6_add_translation(HZ_QM_FILES ${HZ_TS_FILES})
    target_sources(horizon PRIVATE ${HZ_QM_FILES})
    # Copy compiled catalogs next to the executable, where LocaleManager looks
    # (<exe dir>/translations). A generator expression in a source file's
    # OUTPUT_LOCATION property is NOT evaluated, so place them with an explicit
    # POST_BUILD copy (genexes are valid here) instead.
    add_custom_command(TARGET horizon POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
                $<TARGET_FILE_DIR:horizon>/translations
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${HZ_QM_FILES}
                $<TARGET_FILE_DIR:horizon>/translations
        COMMENT "Copying compiled .qm catalogs next to horizon executable")
    # ... and in a package, where LocaleManager looks for them the same way.
    install(FILES ${HZ_QM_FILES} DESTINATION bin/translations)
else()
    message(STATUS "Horizon translations: catalogs not compiled (Qt Linguist tools not found)")
endif()
