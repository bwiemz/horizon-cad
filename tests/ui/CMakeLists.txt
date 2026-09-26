add_executable(hz_ui_tests
    test_CrashReport.cpp
    test_LocaleManager.cpp
    test_TypedPoint.cpp
    test_Workers.cpp
)

target_link_libraries(hz_ui_tests
    PRIVATE
        Horizon::UI
        GTest::gtest
        GTest::gtest_main
)

# A process that crashes on purpose, for the crash handler's test. It builds
# the handler itself, on QtCore alone: a small executable, not another one
# with all of Qt's widgets in it.
add_executable(hz_crash_child
    crash_child.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/src/CrashReport.cpp
)
target_include_directories(hz_crash_child PRIVATE ${CMAKE_SOURCE_DIR}/src/ui/include)
target_link_libraries(hz_crash_child PRIVATE Qt6::Core)
if(WIN32)
    target_link_libraries(hz_crash_child PRIVATE Dbghelp)
endif()
add_dependencies(hz_ui_tests hz_crash_child)

target_compile_definitions(hz_ui_tests
    PRIVATE
        # Location of the shipped .ts catalog sources, for coverage checks.
        HZ_TRANSLATIONS_SRC_DIR="${CMAKE_SOURCE_DIR}/translations"
        HZ_CRASH_CHILD="$<TARGET_FILE:hz_crash_child>")
# Where the compiled catalogs are placed, where the application looks (Phase
# 166; HZ_SHIPPED_DIR): a test loads each one. None there (no lrelease), and
# the test says so.
if(TARGET horizon)
    target_compile_definitions(hz_ui_tests
        PRIVATE HZ_SHIPPED_TRANSLATIONS_DIR="${HZ_SHIPPED_DIR}/translations")
endif()

include(GoogleTest)
gtest_discover_tests(hz_ui_tests)

# MainWindow tests need a QApplication, and so a platform plugin. They use Qt's
# offscreen platform so they run headless on every CI runner.
if(TARGET Qt6::QOffscreenIntegrationPlugin)
    add_executable(hz_ui_window_tests
        window_tests_main.cpp
        test_Application.cpp
        test_Accessibility.cpp
        test_MainWindowDocuments.cpp
        test_Recovery.cpp
        test_BodyFeatureCommands.cpp
        test_FeatureEditing.cpp
        test_SolidCommands.cpp
        test_ImportExport.cpp
        test_ViewportGraphics.cpp
        test_Drafting2D.cpp
        test_AppEssentials.cpp
        test_ToolEdits.cpp
        test_DraftingTools.cpp
        test_Smoke.cpp
        test_RenderEfficiency.cpp
        test_RebuildJob.cpp
        test_TypedLength.cpp
        test_DimensionsAndText.cpp
        test_Sketches.cpp
        test_Picking.cpp
        test_PartCommands.cpp
        test_Seeing.cpp
        test_GuiThreadFree.cpp
        test_Assemblies.cpp
        test_Workbenches.cpp
        test_Drawings.cpp
        test_Units.cpp
        test_Variables.cpp
        test_Help.cpp
        test_Samples.cpp
    )
    target_link_libraries(hz_ui_window_tests
        PRIVATE
            Horizon::UI
            Horizon::Version
            GTest::gtest
    )
    if(TARGET horizon)
        # The catalogs the application ships, to check a window in another
        # language.
        target_compile_definitions(hz_ui_window_tests
            PRIVATE HZ_SHIPPED_TRANSLATIONS_DIR="${HZ_SHIPPED_DIR}/translations")
    endif()
    if(TARGET horizon_samples)
        # The samples the application ships (Phase 168b), made before the
        # tests that open them.
        target_compile_definitions(hz_ui_window_tests
            PRIVATE HZ_BUILT_SAMPLES_DIR="${HZ_SHIPPED_DIR}/samples")
        add_dependencies(hz_ui_window_tests horizon_samples)
    endif()
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
    # Labelled, so a job can leave them out: the ThreadSanitizer job does,
    # since Qt is not instrumented and its own threads look raced.
    gtest_discover_tests(hz_ui_window_tests PROPERTIES LABELS window)
else()
    message(STATUS "Horizon MainWindow tests: SKIPPED (Qt offscreen platform plugin not found)")
endif()
