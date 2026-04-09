#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/Tool.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Grid.h"
#include "horizon/document/Document.h"
#include "horizon/math/Vec4.h"
#include "horizon/math/Mat4.h"

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
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    m_viewportRenderer.destroyGL(gl);
    if (m_renderer) {
        m_renderer->destroyPickingFBO(gl);
    }
    m_renderer.reset();
    doneCurrent();
}

// ---------------------------------------------------------------------------
// Document
// ---------------------------------------------------------------------------

void ViewportWidget::setDocument(doc::Document* doc) {
    m_document = doc;
    m_selectionManager.clearSelection();
    update();
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
        m_overlayRenderer.setCrosshairEnabled(m_activeTool->wantsCrosshair());
    } else {
        m_overlayRenderer.setCrosshairEnabled(false);
    }
}

void ViewportWidget::setActiveSketch(doc::Sketch* sketch) {
    if (sketch && !m_activeSketch) {
        // Entering sketch mode -- save current camera state.
        m_savedCameraState = CameraState{m_camera.eye(), m_camera.target(), m_camera.up()};
    }

    m_activeSketch = sketch;

    if (sketch) {
        // Align camera to the sketch plane.
        const auto& plane = sketch->plane();
        math::Vec3 center = plane.origin();
        double distance = 100.0;
        math::Vec3 eye = center + plane.normal() * distance;
        m_camera.lookAt(eye, center, plane.yAxis());
    } else if (m_savedCameraState) {
        // Exiting sketch mode -- restore saved camera.
        m_camera.lookAt(m_savedCameraState->eye, m_savedCameraState->target,
                        m_savedCameraState->up);
        m_savedCameraState.reset();
    }

    update();
}

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------

math::Vec2 ViewportWidget::worldPositionAtCursor(int screenX, int screenY) const {
    // screenToRay expects Qt-style coordinates (0 = top), so pass screenY directly.
    auto [rayOrigin, rayDir] = m_camera.screenToRay(
        static_cast<double>(screenX),
        static_cast<double>(screenY),
        width(), height());

    // If a sketch is active, project onto its plane (returns local 2D coordinates).
    if (m_activeSketch) {
        math::Vec2 local;
        if (m_activeSketch->plane().rayIntersect(rayOrigin, rayDir, local)) {
            return local;
        }
        // Fallback if ray is parallel to the sketch plane.
        return m_activeSketch->plane().worldToLocal(rayOrigin);
    }

    // Default: intersect with the XY plane (Z = 0).
    if (std::abs(rayDir.z) < 1e-12) {
        return {rayOrigin.x, rayOrigin.y};
    }

    double t = -rayOrigin.z / rayDir.z;
    math::Vec3 hit = rayOrigin + rayDir * t;
    return {hit.x, hit.y};
}

double ViewportWidget::pixelToWorldScale() const {
    int cx = width() / 2;
    int cy = height() / 2;
    math::Vec2 p0 = worldPositionAtCursor(cx, cy);
    math::Vec2 p1 = worldPositionAtCursor(cx + 1, cy);
    return p0.distanceTo(p1);
}

QPointF ViewportWidget::worldToScreen(const math::Vec2& wp) const {
    math::Vec4 clip = m_camera.viewProjectionMatrix()
                      * math::Vec4(math::Vec3(wp.x, wp.y, 0.0), 1.0);
    if (std::abs(clip.w) < 1e-15) return {0.0, 0.0};

    math::Vec3 ndc = clip.perspectiveDivide();
    double sx = (ndc.x + 1.0) * 0.5 * width();
    double sy = (1.0 - ndc.y) * 0.5 * height();  // flip Y for Qt screen coords
    return {sx, sy};
}

// ---------------------------------------------------------------------------
// OpenGL overrides
// ---------------------------------------------------------------------------

void ViewportWidget::initializeGL() {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    m_renderer = std::make_unique<render::GLRenderer>();
    m_renderer->initialize(gl);
    m_renderer->setBackgroundColor(0.18f, 0.18f, 0.20f);

    // Set up GL resources for text overlay (QImage -> texture -> quad).
    m_viewportRenderer.initTextOverlayGL(gl);
}

void ViewportWidget::resizeGL(int w, int h) {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    m_renderer->resize(gl, w, h);

    double aspect = (h > 0) ? static_cast<double>(w) / static_cast<double>(h) : 1.0;
    m_camera.setPerspective(45.0, aspect, 0.1, 10000.0);
}

void ViewportWidget::paintGL() {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    // Recompute DOF analysis every frame (small matrix -- fast enough for real-time).
    m_viewportRenderer.recomputeDOF(m_document);

    // Clear with background color.
    gl->glClearColor(0.18f, 0.18f, 0.20f, 1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    gl->glEnable(GL_DEPTH_TEST);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Render the grid.
    m_renderer->renderGrid(gl, m_camera);

    // Render document entities (collects dimension text info).
    if (m_document) {
        m_viewportRenderer.renderEntities(gl, *m_renderer, m_camera,
                                          *m_document, m_selectionManager);
    }

    // Render grip squares on selected entities.
    if (m_document) {
        m_viewportRenderer.renderGrips(gl, *m_renderer, m_camera,
                                       *m_document, m_selectionManager,
                                       pixelToWorldScale());
    }

    // Render 3D scene graph nodes (solid primitives with PBR-lite, edge overlay).
    if (!m_sceneGraph.nodes().empty()) {
        m_renderer->renderNodes(gl, m_sceneGraph, m_camera);

        // Update GPU color-picking FBO for 3D selection.
        m_renderer->renderPickingPass(gl, m_sceneGraph, m_camera);
    }

    // Render tool preview (rubber-band).
    m_viewportRenderer.renderToolPreview(gl, *m_renderer, m_camera, m_activeTool);

    // GL overlays: crosshair, snap markers, axis indicator.
    m_overlayRenderer.setSnapResult(m_lastSnapResult);
    m_overlayRenderer.render(gl, m_camera, m_renderer.get(),
                             width(), height(), pixelToWorldScale());

    // Render text overlay: paint to an offscreen QImage (QPainter on QImage
    // is pure CPU -- no Windows bitmap mask operations), then upload as a GL
    // texture and draw a fullscreen quad.  This avoids the Qt 6.10
    // qpixmap_win.cpp assertion triggered by QPainter on QOpenGLWidget.
    m_viewportRenderer.blitTextOverlay(gl, m_camera, m_document, m_selectionManager,
                                       width(), height(), pixelToWorldScale());
}

// ---------------------------------------------------------------------------
// Input events — delegated to ViewportInputHandler
// ---------------------------------------------------------------------------

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
    m_inputHandler.handleMousePress(event, this);
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    m_inputHandler.handleMouseMove(event, this);
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
    m_inputHandler.handleMouseRelease(event, this);
}

void ViewportWidget::wheelEvent(QWheelEvent* event) {
    m_inputHandler.handleWheel(event, this);
}

void ViewportWidget::keyPressEvent(QKeyEvent* event) {
    m_inputHandler.handleKeyPress(event, this);
}

}  // namespace hz::ui
