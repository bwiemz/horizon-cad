#pragma once

#include <QOpenGLWidget>
#include <QPoint>
#include <QPointF>
#include <QWidget>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/Camera.h"
#include "horizon/render/SelectionManager.h"
#include "horizon/drafting/SnapEngine.h"

#include <memory>
#include <string>
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
class ViewportWidget;

/// Transparent overlay widget for text rendering on top of the GL viewport.
/// Uses a regular QWidget (not QOpenGLWidget) to avoid the Qt 6.10 Windows
/// qpixmap_win.cpp assertion when using QPainter on QOpenGLWidget.
class ViewportOverlay : public QWidget {
    Q_OBJECT

public:
    explicit ViewportOverlay(ViewportWidget* viewport);

    struct TextItem {
        math::Vec2 worldPos;
        std::string text;
        uint32_t color;
        int fontSize;
        bool bold;
    };

    void setItems(std::vector<TextItem> items);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    ViewportWidget* m_viewport;
    std::vector<TextItem> m_items;
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
    // Rendering
    void renderEntities(QOpenGLExtraFunctions* gl);
    void renderToolPreview(QOpenGLExtraFunctions* gl);
    void updateOverlayText();

    /// Dimension text data collected during renderEntities() for overlay.
    struct DimTextInfo {
        math::Vec2 worldPos;
        std::string text;
        uint32_t color;
    };
    std::vector<DimTextInfo> m_dimTexts;

    /// Generate vertices for a circle approximation.
    std::vector<float> circleVertices(const math::Vec2& center, double radius, int segments = 64) const;

    /// Generate vertices for an arc (partial circle).
    std::vector<float> arcVertices(const math::Vec2& center, double radius,
                                   double startAngle, double endAngle, int segments = 64) const;

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

    // Text overlay (non-owning, child widget)
    ViewportOverlay* m_overlay = nullptr;

    // Navigation state
    bool m_orbiting = false;
    bool m_panning = false;
    QPoint m_lastMousePos;
};

}  // namespace hz::ui
