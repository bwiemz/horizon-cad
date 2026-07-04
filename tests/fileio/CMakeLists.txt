add_executable(hz_fileio_tests
    test_PartFormat.cpp
    test_AssemblyFormat.cpp
    test_DrawingExport.cpp
    test_DrawingDocumentIO.cpp
    test_DrawingDimensionRenderer.cpp
)

target_link_libraries(hz_fileio_tests
    PRIVATE
        Horizon::FileIO
        Horizon::Document
        Horizon::Modeling
        Horizon::Drafting
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_fileio_tests)
