#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/Tool.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Grid.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QOpenGLExtraFunctions>

#include <cmath>

namespace hz::ui {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

ViewportWidget::ViewportWidget(QWidget* parent)
    : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    // Default camera looking at origin from an isometric-ish angle.
    m_camera.setIsometricView();
}

ViewportWidget::~ViewportWidget() {
    // Make the context current before destroying GL resources.
    makeCurrent();
    m_renderer.reset();
    doneCurrent();
}

// ---------------------------------------------------------------------------
// Tool management
// ---------------------------------------------------------------------------

void ViewportWidget::setActiveTool(Tool* tool) {
    if (m_activeTool) {
        m_activeTool->deactivate();
    }
    m_activeTool = tool;
    if (m_activeTool) {
        m_activeTool->activate(this);
    }
}

// ---------------------------------------------------------------------------
// Geometry storage (Phase 1)
// ---------------------------------------------------------------------------

void ViewportWidget::addLine(const math::Vec2& start, const math::Vec2& end) {
    m_lines.emplace_back(start, end);
    update();  // request repaint
}

void ViewportWidget::addCircle(const math::Vec2& center, double radius) {
    m_circles.emplace_back(center, radius);
    update();
}

void ViewportWidget::clearGeometry() {
    m_lines.clear();
    m_circles.clear();
    update();
}

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------

math::Vec2 ViewportWidget::worldPositionAtCursor(int screenX, int screenY) const {
    // Flip Y: Qt has origin top-left, OpenGL bottom-left.
    int flippedY = height() - screenY;

    // Cast a ray from the screen point through the scene.
    auto [rayOrigin, rayDir] = m_camera.screenToRay(
        static_cast<double>(screenX),
        static_cast<double>(flippedY),
        width(), height());

    // Intersect with the XY plane (Z = 0).
    // Ray: P = origin + t * dir
    // Plane: Z = 0  =>  origin.z + t * dir.z = 0  =>  t = -origin.z / dir.z
    if (std::abs(rayDir.z) < 1e-12) {
        // Ray is parallel to the XY plane; return projection of origin.
        return {rayOrigin.x, rayOrigin.y};
    }

    double t = -rayOrigin.z / rayDir.z;
    math::Vec3 hit = rayOrigin + rayDir * t;
    return {hit.x, hit.y};
}

// ---------------------------------------------------------------------------
// OpenGL overrides
// ---------------------------------------------------------------------------

void ViewportWidget::initializeGL() {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    m_renderer = std::make_unique<render::GLRenderer>();
    m_renderer->initialize(gl);
    m_renderer->setBackgroundColor(0.18f, 0.18f, 0.20f);
}

void ViewportWidget::resizeGL(int w, int h) {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    m_renderer->resize(gl, w, h);

    double aspect = (h > 0) ? static_cast<double>(w) / static_cast<double>(h) : 1.0;
    m_camera.setPerspective(45.0, aspect, 0.1, 10000.0);
}

void ViewportWidget::paintGL() {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    // Clear with background color.
    gl->glClearColor(0.18f, 0.18f, 0.20f, 1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    gl->glEnable(GL_DEPTH_TEST);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Render the grid.
    m_renderer->renderGrid(gl, m_camera);

    // Render committed geometry.
    renderLines(gl);
    renderCircles(gl);

    // Render tool preview (rubber-band).
    renderToolPreview(gl);
}

// ---------------------------------------------------------------------------
// Input events
// ---------------------------------------------------------------------------

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
    m_lastMousePos = event->pos();

    if (event->button() == Qt::MiddleButton) {
        if (event->modifiers() & Qt::ShiftModifier) {
            m_panning = true;
        } else {
            m_orbiting = true;
        }
        return;
    }

    if (event->button() == Qt::LeftButton && m_activeTool) {
        math::Vec2 wp = worldPositionAtCursor(event->pos().x(), event->pos().y());
        if (m_activeTool->mousePressEvent(event, wp)) {
            update();
            return;
        }
    }

    QOpenGLWidget::mousePressEvent(event);
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    QPoint delta = event->pos() - m_lastMousePos;

    if (m_orbiting) {
        double yaw   = static_cast<double>(delta.x()) * 0.005;
        double pitch  = static_cast<double>(delta.y()) * 0.005;
        m_camera.orbit(yaw, pitch);
        m_lastMousePos = event->pos();
        update();
        return;
    }

    if (m_panning) {
        double dx = static_cast<double>(delta.x()) * 0.01;
        double dy = static_cast<double>(delta.y()) * 0.01;
        m_camera.pan(dx, -dy);  // negate Y because screen Y is flipped
        m_lastMousePos = event->pos();
        update();
        return;
    }

    // Compute the world position and emit signal for status bar.
    math::Vec2 wp = worldPositionAtCursor(event->pos().x(), event->pos().y());
    emit mouseMoved(wp);

    // Delegate to tool.
    if (m_activeTool) {
        if (m_activeTool->mouseMoveEvent(event, wp)) {
            update();
        }
    }

    m_lastMousePos = event->pos();
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton) {
        m_orbiting = false;
        m_panning = false;
        return;
    }

    if (event->button() == Qt::LeftButton && m_activeTool) {
        math::Vec2 wp = worldPositionAtCursor(event->pos().x(), event->pos().y());
        if (m_activeTool->mouseReleaseEvent(event, wp)) {
            update();
            return;
        }
    }

    QOpenGLWidget::mouseReleaseEvent(event);
}

void ViewportWidget::wheelEvent(QWheelEvent* event) {
    double delta = event->angleDelta().y() / 120.0;
    double factor = 1.0 + delta * 0.1;
    m_camera.zoom(factor);
    update();
}

void ViewportWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape && m_activeTool) {
        m_activeTool->cancel();
        update();
        return;
    }

    if (m_activeTool && m_activeTool->keyPressEvent(event)) {
        update();
        return;
    }

    QOpenGLWidget::keyPressEvent(event);
}

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------

void ViewportWidget::renderLines(QOpenGLExtraFunctions* gl) {
    if (m_lines.empty()) return;

    // Build vertex data: each line is 2 vertices with xyz (z=0).
    std::vector<float> verts;
    verts.reserve(m_lines.size() * 6);
    for (const auto& [start, end] : m_lines) {
        verts.push_back(static_cast<float>(start.x));
        verts.push_back(static_cast<float>(start.y));
        verts.push_back(0.0f);
        verts.push_back(static_cast<float>(end.x));
        verts.push_back(static_cast<float>(end.y));
        verts.push_back(0.0f);
    }

    math::Vec3 white{1.0, 1.0, 1.0};
    m_renderer->drawLines(gl, m_camera, verts, white);
}

void ViewportWidget::renderCircles(QOpenGLExtraFunctions* gl) {
    if (m_circles.empty()) return;

    math::Vec3 white{1.0, 1.0, 1.0};
    for (const auto& [center, radius] : m_circles) {
        auto verts = circleVertices(center, radius);
        m_renderer->drawCircle(gl, m_camera, verts, white);
    }
}

void ViewportWidget::renderToolPreview(QOpenGLExtraFunctions* gl) {
    if (!m_activeTool) return;

    // Preview lines (e.g. rubber-band for LineTool).
    auto previewLines = m_activeTool->getPreviewLines();
    if (!previewLines.empty()) {
        std::vector<float> verts;
        verts.reserve(previewLines.size() * 6);
        for (const auto& [start, end] : previewLines) {
            verts.push_back(static_cast<float>(start.x));
            verts.push_back(static_cast<float>(start.y));
            verts.push_back(0.0f);
            verts.push_back(static_cast<float>(end.x));
            verts.push_back(static_cast<float>(end.y));
            verts.push_back(0.0f);
        }
        math::Vec3 cyan{0.0, 0.8, 1.0};
        m_renderer->drawLines(gl, m_camera, verts, cyan);
    }

    // Preview circles (e.g. rubber-band for CircleTool).
    auto previewCircles = m_activeTool->getPreviewCircles();
    if (!previewCircles.empty()) {
        math::Vec3 cyan{0.0, 0.8, 1.0};
        for (const auto& [center, radius] : previewCircles) {
            auto verts = circleVertices(center, radius);
            m_renderer->drawCircle(gl, m_camera, verts, cyan);
        }
    }
}

std::vector<float> ViewportWidget::circleVertices(const math::Vec2& center, double radius,
                                                   int segments) const {
    // Generate a line-loop approximation as pairs of consecutive vertices (for GL_LINES).
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>(segments) * 6);

    const double step = 2.0 * 3.14159265358979323846 / static_cast<double>(segments);
    for (int i = 0; i < segments; ++i) {
        double a0 = step * static_cast<double>(i);
        double a1 = step * static_cast<double>((i + 1) % segments);

        float x0 = static_cast<float>(center.x + radius * std::cos(a0));
        float y0 = static_cast<float>(center.y + radius * std::sin(a0));
        float x1 = static_cast<float>(center.x + radius * std::cos(a1));
        float y1 = static_cast<float>(center.y + radius * std::sin(a1));

        verts.push_back(x0);
        verts.push_back(y0);
        verts.push_back(0.0f);
        verts.push_back(x1);
        verts.push_back(y1);
        verts.push_back(0.0f);
    }
    return verts;
}

}  // namespace hz::ui
