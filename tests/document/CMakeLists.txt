add_executable(hz_document_tests
    test_ParameterRegistry.cpp
    test_ParameterExpressions.cpp
    test_CollaborationSession.cpp
    test_ExpressionEngine.cpp
    test_SheetMetalFeature.cpp
    test_Sketch.cpp
    test_FeatureTree.cpp
    test_BuildControl.cpp
    test_AssemblyDocument.cpp
    test_DocumentManager.cpp
    test_BillOfMaterials.cpp
    test_ConfigurationTable.cpp
    test_SketchFollow.cpp
    test_UndoStack.cpp
    test_BodyOperations.cpp
    test_FeatureReasons.cpp
    test_ModelCommands.cpp
    test_EntityCommands.cpp
    test_ExactCounterpart.cpp
    test_ExactConstruction.cpp
    test_BridgeNumbering.cpp
)

target_link_libraries(hz_document_tests
    PRIVATE
        Horizon::Document
        Horizon::FileIO
        GTest::gtest
        GTest::gtest_main
)

# The STEP text the reader's tests write, for parts of bodies imported as they are read.
target_include_directories(hz_document_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/tests/fileio
)

include(GoogleTest)
gtest_discover_tests(hz_document_tests DISCOVERY_TIMEOUT 120)
