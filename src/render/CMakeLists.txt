add_library(hz_render STATIC
    src/Camera.cpp
    src/ShaderProgram.cpp
    src/MeshBuffer.cpp
    src/GLRenderer.cpp
    src/Grid.cpp
    src/SceneGraph.cpp
    src/SelectionManager.cpp
    src/OpenGLBackend.cpp
)

# Vulkan backend (Phase 65): staged bring-up behind quiet SDK detection —
# default builds never require the Vulkan SDK (mirrors HZ_ENABLE_SCRIPTING).
find_package(Vulkan QUIET)
if (Vulkan_FOUND)
    message(STATUS "Horizon Vulkan backend: ENABLED")
    target_sources(hz_render PRIVATE src/VulkanBackend.cpp)
    target_compile_definitions(hz_render PUBLIC HZ_ENABLE_VULKAN)
    target_link_libraries(hz_render PUBLIC Vulkan::Vulkan)

    # GPU compute tessellation (Phase 66): compile the NURBS evaluation
    # kernel to SPIR-V at build time with the SDK's glslangValidator.
    find_program(HZ_GLSLANG glslangValidator HINTS $ENV{VULKAN_SDK}/Bin)
    if (HZ_GLSLANG)
        message(STATUS "Horizon GPU tessellation: ENABLED (${HZ_GLSLANG})")
        set(HZ_SPV_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
        add_custom_command(
            OUTPUT ${HZ_SPV_DIR}/nurbs_eval_spv.h
            COMMAND ${HZ_GLSLANG} --target-env vulkan1.1 -V --vn kNurbsEvalSpv
                    -o ${HZ_SPV_DIR}/nurbs_eval_spv.h
                    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/nurbs_eval.comp
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/nurbs_eval.comp
            COMMENT "glslangValidator: nurbs_eval.comp -> SPIR-V header")
        add_custom_target(hz_render_spv DEPENDS ${HZ_SPV_DIR}/nurbs_eval_spv.h)
        add_dependencies(hz_render hz_render_spv)
        target_include_directories(hz_render PRIVATE ${HZ_SPV_DIR})
        target_sources(hz_render PRIVATE src/GpuTessellator.cpp)
        target_compile_definitions(hz_render PUBLIC HZ_ENABLE_GPU_TESSELLATION)
    else()
        message(STATUS "Horizon GPU tessellation: DISABLED (glslangValidator not found)")
    endif()

else()
    message(STATUS "Horizon Vulkan backend: DISABLED (Vulkan SDK not found)")
endif()

target_include_directories(hz_render
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_render
    PUBLIC
        Horizon::Math
        Horizon::Geometry
        OpenGL::GL
        Qt6::OpenGLWidgets
)

# Copy shaders to build directory
add_custom_command(TARGET hz_render POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    $<TARGET_FILE_DIR:hz_render>/shaders
)

add_library(Horizon::Render ALIAS hz_render)
