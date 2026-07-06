set(CMAKE_AUTORCC ON)

add_executable(horizon WIN32
    main.cpp
    resources/resources.qrc
)

target_link_libraries(horizon
    PRIVATE
        Horizon::UI
        spdlog::spdlog
)

install(TARGETS horizon
    RUNTIME DESTINATION bin
)

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
else()
    message(STATUS "Horizon translations: catalogs not compiled (Qt Linguist tools not found)")
endif()
