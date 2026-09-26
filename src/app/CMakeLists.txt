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
# executable. Qt's LinguistTools package is used when it is there; otherwise
# any lrelease on the system is (a distribution's Qt tools, or PySide6's), as a
# .qm catalog does not depend on the Qt that compiled it. A developer build may
# go without catalogs, the shipped .ts sources being the ground truth; a
# package must not (HZ_REQUIRE_TRANSLATIONS, which the release workflow sets).
file(GLOB HZ_TS_FILES ${CMAKE_SOURCE_DIR}/translations/horizon_*.ts)
set(HZ_QM_FILES)
find_package(Qt6 COMPONENTS LinguistTools QUIET)
if (TARGET Qt6::lrelease)
    qt6_add_translation(HZ_QM_FILES ${HZ_TS_FILES})
    set(HZ_LRELEASE_USED "Qt6::lrelease")
else()
    find_program(HZ_LRELEASE NAMES lrelease-qt6 lrelease pyside6-lrelease
                 HINTS /usr/lib/qt6/bin /usr/lib/x86_64-linux-gnu/qt6/bin)
    if (HZ_LRELEASE)
        foreach(ts IN LISTS HZ_TS_FILES)
            get_filename_component(catalog ${ts} NAME_WE)
            set(qm ${CMAKE_CURRENT_BINARY_DIR}/${catalog}.qm)
            add_custom_command(OUTPUT ${qm}
                COMMAND ${HZ_LRELEASE} ${ts} -qm ${qm}
                DEPENDS ${ts}
                COMMENT "Compiling ${catalog}.qm"
                VERBATIM)
            list(APPEND HZ_QM_FILES ${qm})
        endforeach()
        set(HZ_LRELEASE_USED ${HZ_LRELEASE})
    endif()
endif()
if (HZ_QM_FILES)
    message(STATUS "Horizon translations: ENABLED (${HZ_LRELEASE_USED})")
    # Compile the catalogs and copy them next to the executable, where
    # LocaleManager looks (<exe dir>/translations), in every build: a copy
    # hung on the executable's POST_BUILD ran only when it was relinked, so an
    # edited catalog did not reach it. A generator expression in a source
    # file's OUTPUT_LOCATION property is not evaluated; in a COMMAND it is.
    add_custom_target(horizon_translations ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory
                $<TARGET_FILE_DIR:horizon>/translations
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${HZ_QM_FILES}
                $<TARGET_FILE_DIR:horizon>/translations
        DEPENDS ${HZ_QM_FILES}
        COMMENT "Placing the .qm catalogs next to the horizon executable"
        VERBATIM)
    # ... and in a package, where LocaleManager looks for them the same way.
    install(FILES ${HZ_QM_FILES} DESTINATION bin/translations)
    # Every catalog is where the application looks for it.
    if (HZ_BUILD_TESTS)
        set(HZ_QM_NAMES)
        foreach(qm IN LISTS HZ_QM_FILES)
            get_filename_component(name ${qm} NAME)
            list(APPEND HZ_QM_NAMES ${name})
        endforeach()
        add_test(NAME TranslationsAreNextToTheExecutable
            COMMAND ${CMAKE_COMMAND}
                -DDIR=$<TARGET_FILE_DIR:horizon>/translations
                "-DNAMES=${HZ_QM_NAMES}"
                -P ${CMAKE_SOURCE_DIR}/cmake/CheckTranslations.cmake)
    endif()
elseif (HZ_REQUIRE_TRANSLATIONS)
    message(FATAL_ERROR "HZ_REQUIRE_TRANSLATIONS is ON, but the translations cannot be compiled: "
                        "no lrelease was found. Install Qt's Linguist tools (lrelease) or "
                        "PySide6 (pyside6-lrelease).")
else()
    message(STATUS "Horizon translations: catalogs not compiled (no lrelease found)")
endif()

# Samples (Phase 168b): documents made by a program that uses the same API as
# the application, so they always open in the version that ships them. Made
# at build time, placed next to the executable (<exe dir>/samples, where File
# > Open Sample looks), and installed there.
add_executable(hz_make_samples samples/MakeSamples.cpp)
target_link_libraries(hz_make_samples
    PRIVATE
        Horizon::FileIO
        Horizon::Document
        Horizon::Modeling
)
set(HZ_SAMPLE_DIR ${CMAKE_CURRENT_BINARY_DIR}/samples)
set(HZ_SAMPLE_FILES
    ${HZ_SAMPLE_DIR}/bracket.hzpart
    ${HZ_SAMPLE_DIR}/bracket-drawing.hzdwg
    ${HZ_SAMPLE_DIR}/plate.hzpart
    ${HZ_SAMPLE_DIR}/pin.hzpart
    ${HZ_SAMPLE_DIR}/plate-and-pin.hzasm
    ${HZ_SAMPLE_DIR}/gasket.hcad
)
add_custom_command(OUTPUT ${HZ_SAMPLE_FILES}
    COMMAND hz_make_samples ${HZ_SAMPLE_DIR}
    DEPENDS hz_make_samples
    COMMENT "Making the sample documents"
    VERBATIM)
add_custom_target(horizon_samples ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:horizon>/samples
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${HZ_SAMPLE_FILES}
            $<TARGET_FILE_DIR:horizon>/samples
    DEPENDS ${HZ_SAMPLE_FILES}
    COMMENT "Placing the samples next to the horizon executable"
    VERBATIM)
install(FILES ${HZ_SAMPLE_FILES} DESTINATION bin/samples)
