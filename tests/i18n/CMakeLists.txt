# Translation catalogs (Phase 166): every shipped catalog translated above a
# floor, its placeholders and mnemonics kept, and (where lupdate is found) no
# string in the sources missing from it. Python only; no network.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(NOT Python3_Interpreter_FOUND)
    message(STATUS "Horizon translations: catalog checks off (no Python 3)")
    return()
endif()

set(HZ_CATALOG_ARGS)
if(TARGET Qt6::lupdate)
    set(HZ_LUPDATE $<TARGET_FILE:Qt6::lupdate>)
else()
    # PySide6's, on Windows (CI installs it for lrelease); a distribution's
    # Qt tools on Linux.
    find_program(HZ_LUPDATE NAMES lupdate-qt6 lupdate pyside6-lupdate
                 HINTS /usr/lib/qt6/bin /usr/lib/x86_64-linux-gnu/qt6/bin)
endif()
if(HZ_LUPDATE)
    list(APPEND HZ_CATALOG_ARGS --lupdate ${HZ_LUPDATE} --src ${CMAKE_SOURCE_DIR}/src)
endif()

file(GLOB HZ_CATALOGS ${CMAKE_SOURCE_DIR}/translations/horizon_*.ts)
add_test(NAME TranslationCatalogs
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/check_translations.py
            --floor 0.95 ${HZ_CATALOG_ARGS} ${HZ_CATALOGS})
add_test(NAME TranslationCheckerSelfTest
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_check_translations.py)
