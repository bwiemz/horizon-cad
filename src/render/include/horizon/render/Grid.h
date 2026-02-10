#pragma once

#include "horizon/render/Camera.h"
#include "horizon/render/MeshBuffer.h"
#include "horizon/render/ShaderProgram.h"

class QOpenGLExtraFunctions;

namespace hz::render {

/// Renders an infinite ground grid on the XY plane (Z = 0) using a full-screen
/// quad approach with an analytical grid shader.
class Grid {
public:
    Grid();
    ~Grid();

    Grid(const Grid&) = delete;
    Grid& operator=(const Grid&) = delete;

    /// Initialize GPU resources. Must be called with a valid GL context.
    void initialize(QOpenGLExtraFunctions* gl);

    /// Render the grid given the current camera.
    void render(QOpenGLExtraFunctions* gl, const Camera& camera);

    bool isInitialized() const { return m_initialized; }

private:
    ShaderProgram m_shader;
    MeshBuffer m_quad;
    bool m_initialized = false;
};

}  // namespace hz::render
