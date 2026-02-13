#pragma once

#include "horizon/render/Camera.h"
#include "horizon/render/Grid.h"
#include "horizon/render/MeshBuffer.h"
#include "horizon/render/SceneGraph.h"
#include "horizon/render/ShaderProgram.h"

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec4.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

class QOpenGLExtraFunctions;

namespace hz::render {

/// Main OpenGL renderer for the Horizon CAD viewport.
/// All GL calls go through QOpenGLExtraFunctions.
class GLRenderer {
public:
    GLRenderer();
    ~GLRenderer();

    GLRenderer(const GLRenderer&) = delete;
    GLRenderer& operator=(const GLRenderer&) = delete;

    /// Initialize shaders and GPU resources. Call once with a valid GL context.
    void initialize(QOpenGLExtraFunctions* gl);

    /// Handle viewport resize.
    void resize(QOpenGLExtraFunctions* gl, int width, int height);

    /// Render the full scene (objects + grid).
    void renderScene(QOpenGLExtraFunctions* gl, const SceneGraph& scene,
                     const Camera& camera);

    /// Render only the ground grid.
    void renderGrid(QOpenGLExtraFunctions* gl, const Camera& camera);

    /// Background color.
    void setBackgroundColor(float r, float g, float b);

    /// Draw a set of line segments given flat vertex data (x,y,z per vertex, pairs).
    void drawLines(QOpenGLExtraFunctions* gl, const Camera& camera,
                   const std::vector<float>& lineVertices,
                   const math::Vec3& color, float lineWidth = 1.5f);

    /// Draw a circle approximation given flat vertex data (line segments forming the circle).
    void drawCircle(QOpenGLExtraFunctions* gl, const Camera& camera,
                    const std::vector<float>& circleVertices,
                    const math::Vec3& color, float lineWidth = 1.5f);

    /// Draw a filled, semi-transparent quad between two world-space corners.
    void drawFilledQuad(QOpenGLExtraFunctions* gl, const Camera& camera,
                        const math::Vec2& corner1, const math::Vec2& corner2,
                        const math::Vec4& color);

    bool isInitialized() const { return m_initialized; }

private:
    void uploadMesh(QOpenGLExtraFunctions* gl, const SceneNode* node);

    ShaderProgram m_phongShader;
    ShaderProgram m_lineShader;
    ShaderProgram m_fillShader;
    Grid m_grid;

    // Cached GPU mesh buffers keyed by node ID.
    std::unordered_map<uint32_t, std::unique_ptr<MeshBuffer>> m_meshCache;

    float m_bgColor[3] = {0.18f, 0.20f, 0.22f};
    int m_viewportWidth = 1;
    int m_viewportHeight = 1;
    bool m_initialized = false;
};

}  // namespace hz::render
