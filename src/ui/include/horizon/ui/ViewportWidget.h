#pragma once

#include <QOpenGLWidget>
#include <QPoint>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/Camera.h"

#include <memory>
#include <utility>
#include <vector>

class QOpenGLExtraFunctions;

namespace hz::render {
class GLRenderer;
class Grid;
}  // namespace hz::render

namespace hz::ui {

class Tool;

/// The main 2D/3D viewport widget backed by OpenGL.
///
/// Provides camera navigation (orbit, pan, zoom) and delegates left-click
/// interaction to the currently active Tool.
class ViewportWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

    // ---- Camera ----

    /// Access the viewport camera.
    render::Camera& camera() { return m_camera; }
    const render::Camera& camera() const { return m_camera; }

    // ---- Tools ----

    /// Set the active tool.  The viewport does NOT take ownership.
    void setActiveTool(Tool* tool);

    /// Returns the active tool, or nullptr.
    Tool* activeTool() const { return m_activeTool; }

    // ---- Geometry storage (Phase 1: simple in-memory lists) ----

    /// Add a line segment to the scene.
    void addLine(const math::Vec2& start, const math::Vec2& end);

    /// Add a circle to the scene.
    void addCircle(const math::Vec2& center, double radius);

    /// Clear all drawn geometry.
    void clearGeometry();

    const std::vector<std::pair<math::Vec2, math::Vec2>>& lines() const { return m_lines; }
    const std::vector<std::pair<math::Vec2, double>>& circles() const { return m_circles; }

    // ---- Coordinate helpers ----

    /// Project a screen-space position to the world XY plane (Z = 0).
    math::Vec2 worldPositionAtCursor(int screenX, int screenY) const;

signals:
    /// Emitted when the mouse moves.  Carries the world-space position on the XY plane.
    void mouseMoved(const hz::math::Vec2& worldPos);

protected:
    // QOpenGLWidget overrides
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    // Input events
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    // Rendering
    void renderLines(QOpenGLExtraFunctions* gl);
    void renderCircles(QOpenGLExtraFunctions* gl);
    void renderToolPreview(QOpenGLExtraFunctions* gl);

    /// Generate vertices for a circle approximation.
    std::vector<float> circleVertices(const math::Vec2& center, double radius, int segments = 64) const;

    // Camera
    render::Camera m_camera;

    // Renderer / Grid
    std::unique_ptr<render::GLRenderer> m_renderer;

    // Active tool
    Tool* m_activeTool = nullptr;

    // Navigation state
    bool m_orbiting = false;
    bool m_panning = false;
    QPoint m_lastMousePos;

    // Phase 1 geometry storage
    std::vector<std::pair<math::Vec2, math::Vec2>> m_lines;
    std::vector<std::pair<math::Vec2, double>> m_circles;
};

}  // namespace hz::ui
