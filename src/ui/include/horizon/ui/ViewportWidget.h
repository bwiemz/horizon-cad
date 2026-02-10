#pragma once

#include <QOpenGLWidget>
#include <QPoint>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/Camera.h"
#include "horizon/render/SelectionManager.h"
#include "horizon/drafting/SnapEngine.h"

#include <memory>
#include <vector>

class QOpenGLExtraFunctions;

namespace hz::render {
class GLRenderer;
}  // namespace hz::render

namespace hz::doc {
class Document;
}  // namespace hz::doc

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

    // ---- Document ----

    void setDocument(doc::Document* doc);
    doc::Document* document() const { return m_document; }

    // ---- Camera ----

    render::Camera& camera() { return m_camera; }
    const render::Camera& camera() const { return m_camera; }

    // ---- Selection ----

    render::SelectionManager& selectionManager() { return m_selectionManager; }
    const render::SelectionManager& selectionManager() const { return m_selectionManager; }

    // ---- Snapping ----

    draft::SnapEngine& snapEngine() { return m_snapEngine; }
    void setLastSnapResult(const draft::SnapResult& result) { m_lastSnapResult = result; }

    // ---- Tools ----

    /// Set the active tool.  The viewport does NOT take ownership.
    void setActiveTool(Tool* tool);

    /// Returns the active tool, or nullptr.
    Tool* activeTool() const { return m_activeTool; }

    // ---- Coordinate helpers ----

    /// Project a screen-space position to the world XY plane (Z = 0).
    math::Vec2 worldPositionAtCursor(int screenX, int screenY) const;

    /// Returns the world-space distance that corresponds to one pixel at the current zoom.
    double pixelToWorldScale() const;

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
    void renderEntities(QOpenGLExtraFunctions* gl);
    void renderToolPreview(QOpenGLExtraFunctions* gl);

    /// Generate vertices for a circle approximation.
    std::vector<float> circleVertices(const math::Vec2& center, double radius, int segments = 64) const;

    // Camera
    render::Camera m_camera;

    // Renderer
    std::unique_ptr<render::GLRenderer> m_renderer;

    // Document (non-owning)
    doc::Document* m_document = nullptr;

    // Selection
    render::SelectionManager m_selectionManager;

    // Snapping
    draft::SnapEngine m_snapEngine;
    draft::SnapResult m_lastSnapResult;

    // Active tool
    Tool* m_activeTool = nullptr;

    // Navigation state
    bool m_orbiting = false;
    bool m_panning = false;
    QPoint m_lastMousePos;
};

}  // namespace hz::ui
