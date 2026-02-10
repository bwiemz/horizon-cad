add_library(hz_render STATIC
    src/Camera.cpp
    src/ShaderProgram.cpp
    src/MeshBuffer.cpp
    src/GLRenderer.cpp
    src/Grid.cpp
    src/SceneGraph.cpp
    src/SelectionManager.cpp
)

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
