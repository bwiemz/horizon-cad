add_executable(hz_render_tests
    test_RenderBackend.cpp
    test_OpenGLBackend.cpp
    test_VulkanBackend.cpp
)

target_link_libraries(hz_render_tests
    PRIVATE
        Horizon::Render
        Qt6::Gui
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_render_tests)
