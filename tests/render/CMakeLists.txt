add_executable(hz_render_tests
    test_RenderBackend.cpp
    test_InstanceBatcher.cpp
    test_FrustumCuller.cpp
    test_OpenGLBackend.cpp
    test_VulkanBackend.cpp
    test_GpuTessellator.cpp
    test_MaterialLibrary.cpp
    test_PathTracer.cpp
)

target_link_libraries(hz_render_tests
    PRIVATE
        Horizon::Render
        Horizon::Geometry
        Qt6::Gui
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_render_tests)
