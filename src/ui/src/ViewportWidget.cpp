#include "horizon/ui/ViewportWidget.h"

#include <spdlog/spdlog.h>

#include <QKeyEvent>
#include <QMessageBox>
#include <QMouseEvent>
#include <QOpenGLExtraFunctions>
#include <QShowEvent>
#include <QSurfaceFormat>
#include <QTimer>
#include <QWheelEvent>
#include <cmath>

#include "horizon/document/Document.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec4.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Grid.h"
#include "horizon/ui/Tool.h"

namespace hz::ui {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

ViewportWidget::ViewportWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    // Default camera looking at origin from an isometric-ish angle.
    m_camera.setIsometricView();
}

ViewportWidget::~ViewportWidget() {
    // GL resources exist only if the widget was shown and initializeGL() ran;
    // a viewport that never got a context (never shown, or context creation
    // failed) has nothing to release and no context to make current.
    makeCurrent();
    QOpenGLContext* context = QOpenGLContext::currentContext();
    if (!context) return;
    auto* gl = context->extraFunctions();
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
    // A new document can be at the address of one just closed, with the same
    // undo revision: the analysis must not be taken for its.
    m_viewportRenderer.invalidateDOF();
    update();
}

// ---------------------------------------------------------------------------
// Tool management
// ---------------------------------------------------------------------------

void ViewportWidget::setActiveTool(Tool* tool) {
    if (m_activeTool) {
        m_activeTool->deactivate();
    }
    m_typedPoint.clear();  // typed for the tool before
    m_activeTool = tool;
    if (m_activeTool) {
        // Keys go to the tool: a point or a length typed straight after
        // picking the tool from the ribbon, which otherwise kept the focus.
        setFocus(Qt::OtherFocusReason);
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
        static_cast<double>(screenX), static_cast<double>(screenY), width(), height());

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
    // A viewport with no size yet (mid-layout, or never shown) projects to
    // nothing: keep the last good scale, so snaps and picks still reach.
    if (width() > 0 && height() > 0) {
        const int cx = width() / 2;
        const int cy = height() / 2;
        const double scale =
            worldPositionAtCursor(cx, cy).distanceTo(worldPositionAtCursor(cx + 1, cy));
        if (std::isfinite(scale) && scale > 0.0) m_lastPixelScale = scale;
    }
    return m_lastPixelScale;
}

namespace {

/// A unit vector at @p radians, with the components of the axis directions
/// exactly 0 and ±1, so a point held to an axis has exactly its base's other
/// coordinate.
math::Vec2 direction(double radians) {
    math::Vec2 u(std::cos(radians), std::sin(radians));
    for (double* c : {&u.x, &u.y}) {
        if (std::abs(*c) < 1e-12) *c = 0.0;
        if (std::abs(std::abs(*c) - 1.0) < 1e-12) *c = *c > 0.0 ? 1.0 : -1.0;
    }
    return u;
}

/// @p point projected onto the ray from @p base nearest its own direction
/// whose angle is a whole multiple of @p stepDegrees.
math::Vec2 alongNearestAngle(const math::Vec2& base, const math::Vec2& point, double stepDegrees) {
    const math::Vec2 d = point - base;
    if (d.length() < 1e-12 || !(stepDegrees > 0.0)) return point;
    const double step = stepDegrees * math::kDegToRad;
    const math::Vec2 u = direction(std::round(std::atan2(d.y, d.x) / step) * step);
    return base + u * d.dot(u);
}

}  // namespace

void ViewportWidget::setDraftingAids(const DraftingAids& aids) {
    m_aids = aids;
    m_snapEngine.setObjectSnapEnabled(aids.objectSnap);
    m_snapEngine.setGridSnapEnabled(aids.gridSnap);
}

draft::SnapResult ViewportWidget::snap(const math::Vec2& worldPos) {
    // A typed point is exactly where it was typed: no snap, no tracking.
    if (m_typedOverride) return {*m_typedOverride, draft::SnapType::None};
    if (m_document == nullptr) return {worldPos, draft::SnapType::None};
    m_snapEngine.setSnapTolerance(m_snapPixels * pixelToWorldScale());
    const auto& layers = m_document->layerManager();
    const auto& drawing = m_document->draftDocument();
    const auto snappable = [&layers](const draft::DraftEntity& entity) {
        const auto* layer = layers.getLayer(entity.layer());
        return layer != nullptr && layer->visible && !layer->locked;
    };
    draft::SnapResult result = m_snapEngine.snap(worldPos, drawing, snappable);

    // Ortho and polar tracking, from the point the tool measures from. An
    // entity snapped to wins over them: it is the point that was aimed at.
    const bool tracking = m_aids.ortho || m_aids.polar;
    const bool onEntity =
        result.type != draft::SnapType::None && result.type != draft::SnapType::Grid;
    if (tracking && !onEntity && m_activeTool != nullptr) {
        if (const auto base = m_activeTool->basePoint()) {
            result.point =
                alongNearestAngle(*base, result.point, m_aids.ortho ? 90.0 : m_aids.polarAngle);
            result.type = draft::SnapType::None;
        }
    }
    return result;
}

bool ViewportWidget::applyTypedPoint(const math::Vec2& point) {
    if (m_activeTool == nullptr) return false;
    const QPointF at = worldToScreen(point);
    const QPointF global = mapToGlobal(at);
    m_typedOverride = point;
    QMouseEvent press(QEvent::MouseButtonPress, at, global, Qt::LeftButton, Qt::LeftButton,
                      Qt::NoModifier);
    const bool used = m_activeTool->mousePressEvent(&press, point);
    if (m_activeTool != nullptr) {
        QMouseEvent release(QEvent::MouseButtonRelease, at, global, Qt::LeftButton, Qt::NoButton,
                            Qt::NoModifier);
        m_activeTool->mouseReleaseEvent(&release, point);
    }
    m_typedOverride.reset();
    // The rubber band follows the cursor again, from the point just placed.
    if (m_activeTool != nullptr && m_cursorWorld) {
        const QPointF cursor = worldToScreen(*m_cursorWorld);
        QMouseEvent move(QEvent::MouseMove, cursor, mapToGlobal(cursor), Qt::NoButton, Qt::NoButton,
                         Qt::NoModifier);
        m_activeTool->mouseMoveEvent(&move, *m_cursorWorld);
    }
    if (used) emit selectionChanged();
    update();
    return used;
}

QPointF ViewportWidget::worldToScreen(const math::Vec2& wp) const {
    math::Vec4 clip =
        m_camera.viewProjectionMatrix() * math::Vec4(math::Vec3(wp.x, wp.y, 0.0), 1.0);
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
    QOpenGLContext* context = QOpenGLContext::currentContext();
    auto* gl = context->extraFunctions();

    const QSurfaceFormat format = context->format();
    const auto* version = reinterpret_cast<const char*>(gl->glGetString(GL_VERSION));
    const auto* device = reinterpret_cast<const char*>(gl->glGetString(GL_RENDERER));
    spdlog::info("OpenGL {}.{} context: {} on {}", format.majorVersion(), format.minorVersion(),
                 version ? version : "unknown version", device ? device : "unknown device");

    // Everything past this point is OpenGL 3.3, whose functions an older
    // context may not have at all: calling one was a crash. Without it the
    // viewport only clears (paintGL), and checkGraphics() says why.
    m_glReady = false;
    if (format.version() < qMakePair(3, 3)) {
        m_graphicsProblem = tr("The graphics driver provides OpenGL %1.%2; Horizon CAD needs 3.3.")
                                .arg(format.majorVersion())
                                .arg(format.minorVersion());
        return;
    }
    m_renderer = std::make_unique<render::GLRenderer>();
    m_renderer->initialize(gl);
    if (!m_renderer->isInitialized()) {
        m_graphicsProblem = tr("The graphics driver could not compile Horizon CAD's shaders.");
        return;
    }
    // Deep canvas — darker than the panel chrome so the viewport reads as the
    // focal surface (panels #2d–#32, data surfaces #1e, viewport ~#1c1d21).
    m_renderer->setBackgroundColor(0.11f, 0.115f, 0.13f);

    // Set up GL resources for text overlay (QImage -> texture -> quad).
    m_viewportRenderer.initTextOverlayGL(gl);
    m_glReady = true;
}

void ViewportWidget::showEvent(QShowEvent* event) {
    QOpenGLWidget::showEvent(event);
    if (m_graphicsCheckScheduled) return;
    m_graphicsCheckScheduled = true;
    // The context is created on the first paint; give it that long.
    QTimer::singleShot(1500, this, &ViewportWidget::checkGraphics);
}

void ViewportWidget::checkGraphics() {
    // No initializeGL() at all: Qt could not create a context, or this
    // platform has no OpenGL (paintGL will never run, so the viewport would
    // simply stay blank without a word).
    if (m_graphicsProblem.isEmpty() && !isValid()) {
        m_graphicsProblem = tr("OpenGL is not available, so the viewport cannot draw.");
    }
    if (m_graphicsProblem.isEmpty() || m_graphicsProblemReported) return;
    m_graphicsProblemReported = true;

    spdlog::error("Viewport cannot draw: {}", m_graphicsProblem.toStdString());
    QMessageBox::warning(window(), tr("Graphics Problem"),
                         m_graphicsProblem + "\n\n" +
                             tr("You can still open and save documents, but the viewport will "
                                "stay blank. Updating the graphics driver usually fixes this."));
}

void ViewportWidget::resizeGL(int w, int h) {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    // The framebuffer is in device pixels; w and h are logical ones.
    const qreal dpr = devicePixelRatioF();
    if (m_glReady) m_renderer->resize(gl, qRound(w * dpr), qRound(h * dpr));

    double aspect = (h > 0) ? static_cast<double>(w) / static_cast<double>(h) : 1.0;
    m_camera.setPerspective(45.0, aspect, 0.1, 10000.0);
}

void ViewportWidget::paintGL() {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    if (!m_glReady) {
        // Only what every OpenGL has: the background.
        gl->glClearColor(0.11f, 0.115f, 0.13f, 1.0f);
        gl->glClear(GL_COLOR_BUFFER_BIT);
        return;
    }

    // The constraint analysis behind the DOF colours: only when the document
    // changed, not on every frame.
    m_viewportRenderer.recomputeDOF(m_document);

    // Clear with background color (kept in sync with setBackgroundColor above).
    gl->glClearColor(0.11f, 0.115f, 0.13f, 1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    gl->glEnable(GL_DEPTH_TEST);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Render the grid.
    m_renderer->renderGrid(gl, m_camera);

    // Render document entities (collects dimension text info).
    if (m_document) {
        m_viewportRenderer.renderEntities(gl, *m_renderer, m_camera, *m_document,
                                          m_selectionManager);
    }

    // Render grip squares on selected entities.
    if (m_document) {
        m_viewportRenderer.renderGrips(gl, *m_renderer, m_camera, *m_document, m_selectionManager,
                                       pixelToWorldScale());
    }

    // Render 3D scene graph nodes (solid primitives with PBR-lite, edge overlay).
    // Every frame, even with nothing to draw: renderNodes() also lets go of
    // the meshes of nodes that left the scene, and an emptied scene (a part's
    // tab switched for a drawing's) is when that matters most. No picking
    // pass here: nothing reads it; a pick renders one when it needs it.
    m_renderer->renderNodes(gl, m_sceneGraph, m_camera);

    // Render tool preview (rubber-band).
    m_viewportRenderer.renderToolPreview(gl, *m_renderer, m_camera, m_activeTool);

    // GL overlays: crosshair, snap markers, axis indicator.
    m_overlayRenderer.setSnapResult(m_lastSnapResult);
    m_overlayRenderer.render(gl, m_camera, m_renderer.get(), width(), height(),
                             pixelToWorldScale());

    // Render text overlay: paint to an offscreen QImage (QPainter on QImage
    // is pure CPU -- no Windows bitmap mask operations), then upload as a GL
    // texture and draw a fullscreen quad.  This avoids the Qt 6.10
    // qpixmap_win.cpp assertion triggered by QPainter on QOpenGLWidget.
    m_viewportRenderer.blitTextOverlay(gl, m_camera, m_document, m_selectionManager, width(),
                                       height(), pixelToWorldScale(), devicePixelRatioF());
}

// ---------------------------------------------------------------------------
// Input events — delegated to ViewportInputHandler
// ---------------------------------------------------------------------------

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
    m_viewCubeCapturedPress = false;
    // A left-click on the orientation gizmo snaps the view instead of drawing.
    if (event->button() == Qt::LeftButton) {
        const ViewCube::Region region =
            m_viewportRenderer.viewCube().hitTest(event->position().toPoint());
        if (region != ViewCube::Region::None) {
            m_viewCubeCapturedPress = true;
            applyViewCubeRegion(region);
            update();
            return;
        }
    }
    m_inputHandler.handleMousePress(event, this);
}

void ViewportWidget::applyViewCubeRegion(ViewCube::Region region) {
    using Region = ViewCube::Region;
    double dist = (m_camera.eye() - m_camera.target()).length();
    if (dist < 1e-10) dist = 10.0;
    const math::Vec3 t = m_camera.target();
    const math::Vec3 zUp(0.0, 0.0, 1.0);
    switch (region) {
        case Region::Front:
            m_camera.setFrontView();
            break;
        case Region::Top:
            m_camera.setTopView();
            break;
        case Region::Right:
            m_camera.setRightView();
            break;
        case Region::Iso:
            m_camera.setIsometricView();
            break;
        // Back / Bottom / Left have no camera preset; mirror the preset math.
        case Region::Back:
            m_camera.lookAt(t + math::Vec3(0.0, dist, 0.0), t, zUp);
            break;
        case Region::Bottom:
            m_camera.lookAt(t + math::Vec3(0.0, 0.0, -dist), t, math::Vec3(0.0, 1.0, 0.0));
            break;
        case Region::Left:
            m_camera.lookAt(t + math::Vec3(-dist, 0.0, 0.0), t, zUp);
            break;
        case Region::None:
            break;
    }
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    m_inputHandler.handleMouseMove(event, this);
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
    // Swallow the release that pairs with a view-cube press so the active tool
    // does not run a pick/clear at the release point.
    if (event->button() == Qt::LeftButton && m_viewCubeCapturedPress) {
        m_viewCubeCapturedPress = false;
        return;
    }
    m_inputHandler.handleMouseRelease(event, this);
}

void ViewportWidget::wheelEvent(QWheelEvent* event) {
    m_inputHandler.handleWheel(event, this);
}

void ViewportWidget::keyPressEvent(QKeyEvent* event) {
    m_inputHandler.handleKeyPress(event, this);
}

}  // namespace hz::ui
