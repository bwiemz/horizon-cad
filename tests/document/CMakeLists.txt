add_executable(hz_document_tests
    test_ParameterRegistry.cpp
    test_ExpressionEngine.cpp
    test_Sketch.cpp
    test_FeatureTree.cpp
    test_AssemblyDocument.cpp
    test_DocumentManager.cpp
    test_BillOfMaterials.cpp
)

target_link_libraries(hz_document_tests
    PRIVATE
        Horizon::Document
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_document_tests)
