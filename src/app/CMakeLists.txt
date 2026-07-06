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
    set_source_files_properties(${HZ_TS_FILES} PROPERTIES
        OUTPUT_LOCATION $<TARGET_FILE_DIR:horizon>/translations)
    qt6_add_translation(HZ_QM_FILES ${HZ_TS_FILES})
    target_sources(horizon PRIVATE ${HZ_QM_FILES})
else()
    message(STATUS "Horizon translations: catalogs not compiled (Qt Linguist tools not found)")
endif()
