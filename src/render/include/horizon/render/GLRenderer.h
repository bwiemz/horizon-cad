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

    /// Render 3D mesh nodes from the scene graph (Phong-shaded).
    /// Unlike renderScene, this does NOT clear the framebuffer or draw the grid.
    void renderNodes(QOpenGLExtraFunctions* gl, const SceneGraph& scene,
                     const Camera& camera);

    /// Background color.
    void setBackgroundColor(float r, float g, float b);

    /// Draw a set of line segments given flat vertex data (x,y,z,dist per vertex, pairs).
    /// Vertex format: 4 floats per vertex (x, y, z, distance-along-entity).
    void drawLines(QOpenGLExtraFunctions* gl, const Camera& camera,
                   const std::vector<float>& lineVertices,
                   const math::Vec3& color, float lineWidth = 1.5f,
                   int lineType = 1, float patternScale = 1.0f);

    /// Draw a circle approximation given flat vertex data (line segments forming the circle).
    void drawCircle(QOpenGLExtraFunctions* gl, const Camera& camera,
                    const std::vector<float>& circleVertices,
                    const math::Vec3& color, float lineWidth = 1.5f,
                    int lineType = 1, float patternScale = 1.0f);

    /// Draw a filled, semi-transparent quad between two world-space corners.
    void drawFilledQuad(QOpenGLExtraFunctions* gl, const Camera& camera,
                        const math::Vec2& corner1, const math::Vec2& corner2,
                        const math::Vec4& color);

    // ---- Section Plane ----

    /// Set a clip plane for section-plane rendering (xyz=normal, w=offset).
    /// Fragments on the negative side of the plane are discarded.
    void setClipPlane(const math::Vec4& plane);

    /// Disable the section clip plane.
    void clearClipPlane();

    // ---- GPU Color-Picking ----

    /// Render the scene to an offscreen FBO with per-node ID colors.
    void renderPickingPass(QOpenGLExtraFunctions* gl, const SceneGraph& scene,
                           const Camera& camera);

    /// Read the picking FBO at the given screen pixel and return the node ID (0 = miss).
    uint32_t pickAtPixel(QOpenGLExtraFunctions* gl, int x, int y);

    /// Release the picking FBO resources.
    void destroyPickingFBO(QOpenGLExtraFunctions* gl);

    bool isInitialized() const { return m_initialized; }

private:
    void uploadMesh(QOpenGLExtraFunctions* gl, const SceneNode* node);

    /// Render edges of visible mesh nodes as wireframe overlay.
    void renderEdgeOverlay(QOpenGLExtraFunctions* gl,
                           const std::vector<SceneNode*>& nodes,
                           const math::Mat4& vp);

    void initPickingFBO(QOpenGLExtraFunctions* gl, int width, int height);

    ShaderProgram m_phongShader;
    ShaderProgram m_lineShader;
    ShaderProgram m_fillShader;
    ShaderProgram m_pickShader;
    ShaderProgram m_edgeShader;
    Grid m_grid;

    // Cached GPU mesh buffers keyed by node ID.
    std::unordered_map<uint32_t, std::unique_ptr<MeshBuffer>> m_meshCache;

    void destroyDynamicBuffers(QOpenGLExtraFunctions* gl);
    void uploadDynamic(QOpenGLExtraFunctions* gl, const void* data, size_t sizeBytes);

    float m_bgColor[3] = {0.18f, 0.20f, 0.22f};
    int m_viewportWidth = 1;
    int m_viewportHeight = 1;
    bool m_initialized = false;

    // Section plane (0,0,0,0 = disabled).
    math::Vec4 m_clipPlane{0.0, 0.0, 0.0, 0.0};

    // Persistent dynamic VAO/VBO for per-frame draw calls (lines, circles, quads).
    GLuint m_dynamicVAO = 0;
    GLuint m_dynamicVBO = 0;
    size_t m_dynamicVBOCapacity = 0;

    // GPU color-picking FBO.
    GLuint m_pickFBO = 0;
    GLuint m_pickColorTex = 0;
    GLuint m_pickDepthRBO = 0;
    int m_pickWidth = 0;
    int m_pickHeight = 0;
};

}  // namespace hz::render
