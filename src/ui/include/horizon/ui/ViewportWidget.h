#pragma once

#include <QOpenGLWidget>
#include <QPoint>
#include <QPointF>
#include <QWidget>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/Camera.h"
#include "horizon/document/Sketch.h"
#include "horizon/render/SceneGraph.h"
#include "horizon/render/SelectionManager.h"
#include "horizon/drafting/SnapEngine.h"
#include "horizon/ui/OverlayRenderer.h"
#include "horizon/ui/ViewportInputHandler.h"
#include "horizon/ui/ViewportRenderer.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

class QOpenGLExtraFunctions;
class QImage;

namespace hz::render {
class GLRenderer;
}  // namespace hz::render

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::ui {

class Tool;

/// Saved camera state for restoring after sketch editing.
struct CameraState {
    math::Vec3 eye;
    math::Vec3 target;
    math::Vec3 up;
};

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

    // ---- Scene Graph (3D) ----

    render::SceneGraph& sceneGraph() { return m_sceneGraph; }
    const render::SceneGraph& sceneGraph() const { return m_sceneGraph; }

    // ---- Snapping ----

    draft::SnapEngine& snapEngine() { return m_snapEngine; }
    void setLastSnapResult(const draft::SnapResult& result) { m_lastSnapResult = result; }

    // ---- Tools ----

    /// Set the active tool.  The viewport does NOT take ownership.
    void setActiveTool(Tool* tool);

    /// Returns the active tool, or nullptr.
    Tool* activeTool() const { return m_activeTool; }

    // ---- Active Sketch ----

    /// Set the active sketch for mouse-to-plane projection.
    /// Passing a non-null sketch saves camera state and aligns the camera
    /// to the sketch plane.  Passing nullptr restores the saved camera.
    void setActiveSketch(doc::Sketch* sketch);

    /// Returns the currently active sketch, or nullptr.
    doc::Sketch* activeSketch() const { return m_activeSketch; }

    // ---- Overlay ----

    /// Access the overlay renderer (crosshair, snap markers, axis indicator).
    OverlayRenderer& overlayRenderer() { return m_overlayRenderer; }

    // ---- Coordinate helpers ----

    /// Project a screen-space position to the world XY plane (Z = 0).
    math::Vec2 worldPositionAtCursor(int screenX, int screenY) const;

    /// Returns the world-space distance that corresponds to one pixel at the current zoom.
    double pixelToWorldScale() const;

    /// Project a world-space 2D point to screen coordinates.
    QPointF worldToScreen(const math::Vec2& wp) const;

signals:
    /// Emitted when the mouse moves.  Carries the world-space position on the XY plane.
    void mouseMoved(const hz::math::Vec2& worldPos);

    /// Emitted when the selection changes.
    void selectionChanged();

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

    // 3D scene graph
    render::SceneGraph m_sceneGraph;

    // Active tool
    Tool* m_activeTool = nullptr;

    // Active sketch (non-owning)
    doc::Sketch* m_activeSketch = nullptr;

    // Saved camera state for restore on sketch exit
    std::optional<CameraState> m_savedCameraState;

    // GL overlay renderer (crosshair, snap markers, axis indicator)
    OverlayRenderer m_overlayRenderer;

    // Extracted input handler (pan, orbit, zoom, tool dispatch)
    ViewportInputHandler m_inputHandler;

    // Extracted renderer (entity batching, text overlay, grips, DOF, tool preview)
    ViewportRenderer m_viewportRenderer;
};

}  // namespace hz::ui
