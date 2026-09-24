add_executable(hz_ui_tests
    test_LocaleManager.cpp
)

target_link_libraries(hz_ui_tests
    PRIVATE
        Horizon::UI
        GTest::gtest
        GTest::gtest_main
)

# Location of the shipped .ts catalog sources, for coverage checks.
target_compile_definitions(hz_ui_tests
    PRIVATE HZ_TRANSLATIONS_SRC_DIR="${CMAKE_SOURCE_DIR}/translations")

include(GoogleTest)
gtest_discover_tests(hz_ui_tests)

# MainWindow tests need a QApplication, and so a platform plugin. They use Qt's
# offscreen platform so they run headless on every CI runner.
if(TARGET Qt6::QOffscreenIntegrationPlugin)
    add_executable(hz_ui_window_tests
        window_tests_main.cpp
        test_Application.cpp
        test_MainWindowDocuments.cpp
        test_Recovery.cpp
        test_BodyFeatureCommands.cpp
        test_FeatureEditing.cpp
        test_SolidCommands.cpp
        test_ViewportGraphics.cpp
    )
    target_link_libraries(hz_ui_window_tests
        PRIVATE
            Horizon::UI
            GTest::gtest
    )
    get_target_property(_hz_qt_core_type Qt6::Core TYPE)
    if(_hz_qt_core_type STREQUAL "STATIC_LIBRARY")
        # Static Qt links platform plugins into the executable.
        qt_import_plugins(hz_ui_window_tests INCLUDE Qt6::QOffscreenIntegrationPlugin)
    elseif(WIN32)
        # DLL Qt looks for plugins next to the executable; src/app deploys the
        # windows platform plugin the same way.
        add_custom_command(TARGET hz_ui_window_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                    $<TARGET_FILE_DIR:hz_ui_window_tests>/platforms
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:Qt6::QOffscreenIntegrationPlugin>
                    $<TARGET_FILE_DIR:hz_ui_window_tests>/platforms/
            COMMENT "Deploying Qt offscreen platform plugin next to hz_ui_window_tests"
            VERBATIM)
    endif()
    gtest_discover_tests(hz_ui_window_tests)
else()
    message(STATUS "Horizon MainWindow tests: SKIPPED (Qt offscreen platform plugin not found)")
endif()
