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
#include <algorithm>
#include <cmath>
#include <limits>

#include "horizon/document/Document.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec4.h"
#include "horizon/modeling/Naming.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Grid.h"
#include "horizon/render/MeshPicker.h"
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
    setModelHover(std::nullopt);  // only the Select tool shows what a click would choose
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
    if (sketch == m_activeSketch) return;
    if (sketch && !m_activeSketch) {
        // Entering sketch mode -- save current camera state.
        m_savedCameraState = CameraState{m_camera.eye(), m_camera.target(), m_camera.up()};
    }

    m_activeSketch = sketch;
    m_selectionManager.clearSelection();  // what was selected is in another drawing
    m_modelHover.reset();
    clearModelSelection();

    if (sketch) {
        // The view works in the sketch's own coordinates: its plane is the
        // view's XY, and the solids are placed in that frame (MainWindow).
        // Look straight down on it, as far away as the view was.
        const double distance = std::max((m_camera.eye() - m_camera.target()).length(), 20.0);
        m_camera.lookAt(math::Vec3(0.0, 0.0, distance), math::Vec3(0.0, 0.0, 0.0),
                        math::Vec3(0.0, 1.0, 0.0));
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

    // Intersect with the XY plane (Z = 0): the drawing's, or, while a sketch
    // is edited, the sketch's own, since the view then works in its frame.
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
    const auto& drawing = m_document->activeDrawing();
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

QPointF ViewportWidget::projectToScreen(const math::Vec3& world) const {
    const math::Vec4 clip = m_camera.viewProjectionMatrix() * math::Vec4(world, 1.0);
    if (std::abs(clip.w) < 1e-15) return {0.0, 0.0};
    const math::Vec3 ndc = clip.perspectiveDivide();
    return {(ndc.x + 1.0) * 0.5 * width(), (1.0 - ndc.y) * 0.5 * height()};
}

// ---------------------------------------------------------------------------
// The part in 3D: picking and choosing its faces and edges
// ---------------------------------------------------------------------------

std::optional<ViewportWidget::ModelPick> ViewportWidget::pickModel(const QPointF& at) const {
    if (m_activeSketch || width() <= 0 || height() <= 0) return std::nullopt;
    // screenToRay expects Qt-style coordinates (0 = top).
    const auto [origin, direction] = m_camera.screenToRay(at.x(), at.y(), width(), height());
    const auto nodes = m_sceneGraph.collectVisibleMeshNodes();

    // The face the ray meets first, over every solid shown.
    const render::SceneNode* faceNode = nullptr;
    std::optional<render::MeshHit> face;
    for (const render::SceneNode* node : nodes) {
        const auto hit =
            render::MeshPicker::pickFace(node->mesh(), node->worldTransform(), origin, direction);
        if (hit && (!face || hit->distance < face->distance)) {
            face = hit;
            faceNode = node;
        }
    }
    // An edge near the cursor wins over the face under it: edges are thin,
    // and are what a click that close means. Not one the face hides.
    const double hiddenBeyond = face ? face->distance : std::numeric_limits<double>::infinity();
    const render::SceneNode* edgeNode = nullptr;
    std::optional<render::MeshHit> edge;
    for (const render::SceneNode* node : nodes) {
        const auto hit = render::MeshPicker::pickEdge(node->mesh(), node->worldTransform(),
                                                      m_camera, at.x(), at.y(), width(), height(),
                                                      kPickPixels * 0.6, hiddenBeyond);
        if (hit && (!edge || hit->distance < edge->distance)) {
            edge = hit;
            edgeNode = node;
        }
    }
    // What is picked is the whole curve or curved face a chord or facet is
    // part of (Stable names, Phase 139): one pick for a rim, not one of its
    // chords.
    if (edge && edgeNode) {
        const auto& edges = edgeNode->mesh().edges;
        return ModelPick{edgeNode->ownerId(),
                         model::logicalEdge(edges[static_cast<size_t>(edge->edge)].tag), true};
    }
    if (face && faceNode && face->face >= 0) {
        const auto& tags = faceNode->mesh().faceTags;
        if (static_cast<size_t>(face->face) < tags.size()) {
            return ModelPick{faceNode->ownerId(),
                             model::logicalFace(tags[static_cast<size_t>(face->face)]), false};
        }
    }
    return std::nullopt;
}

void ViewportWidget::chooseModel(const std::optional<ModelPick>& pick, bool add) {
    const auto before = m_modelSelection;
    if (!pick) {
        if (!add) m_modelSelection.clear();
    } else if (!add) {
        m_modelSelection = {*pick};
    } else {
        const auto it = std::find(m_modelSelection.begin(), m_modelSelection.end(), *pick);
        if (it != m_modelSelection.end()) {
            m_modelSelection.erase(it);
        } else {
            m_modelSelection.push_back(*pick);
        }
    }
    if (m_modelSelection != before) {
        update();
        emit modelSelectionChanged();
    }
}

void ViewportWidget::clearModelSelection() {
    if (m_modelSelection.empty()) return;
    m_modelSelection.clear();
    update();
    emit modelSelectionChanged();
}

void ViewportWidget::setModelHover(const std::optional<ModelPick>& pick) {
    if (pick == m_modelHover) return;
    m_modelHover = pick;
    update();
}

void ViewportWidget::drawDatums(QOpenGLExtraFunctions* gl) {
    if (!m_document) return;
    const auto& tree = m_document->featureTree();
    const int rollback = tree.rollbackIndex();
    // In the frame the view works in: the sketch's, while one is edited.
    const math::Mat4 frame =
        m_activeSketch ? m_activeSketch->plane().worldToLocalMatrix() : math::Mat4::identity();
    std::vector<float> lines;
    // Each segment's ends carry their distance along it (0 and its length):
    // the line shader dashes by that distance, and all zeros drew solid.
    const auto segment = [&](const math::Vec3& a, const math::Vec3& b) {
        const math::Vec3 p = frame.transformPoint(a);
        const math::Vec3 q = frame.transformPoint(b);
        lines.insert(lines.end(),
                     {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                      0.0f, static_cast<float>(q.x), static_cast<float>(q.y),
                      static_cast<float>(q.z), static_cast<float>((q - p).length())});
    };
    constexpr double kPlaneHalf = 15.0;  // a datum plane is drawn as a square this big
    constexpr double kAxisHalf = 60.0;
    constexpr double kPointHalf = 1.0;
    for (size_t i = 0; i < tree.featureCount(); ++i) {
        if (rollback >= 0 && static_cast<int>(i) > rollback) break;
        const auto* datum = dynamic_cast<const doc::DatumFeature*>(tree.feature(i));
        if (!datum || datum->isSuppressed()) continue;
        const math::Vec3& o = datum->origin();
        switch (datum->datumKind()) {
            case doc::DatumFeature::DatumKind::Plane: {
                const auto plane = datum->asPlane();
                const math::Vec3 u = plane.xAxis * kPlaneHalf;
                const math::Vec3 v = plane.yAxis() * kPlaneHalf;
                const math::Vec3 corners[4] = {o - u - v, o + u - v, o + u + v, o - u + v};
                for (int k = 0; k < 4; ++k) segment(corners[k], corners[(k + 1) % 4]);
                break;
            }
            case doc::DatumFeature::DatumKind::Axis:
                segment(o - datum->dirA() * kAxisHalf, o + datum->dirA() * kAxisHalf);
                break;
            case doc::DatumFeature::DatumKind::Point:
                for (const math::Vec3& d :
                     {math::Vec3::UnitX, math::Vec3::UnitY, math::Vec3::UnitZ}) {
                    segment(o - d * kPointHalf, o + d * kPointHalf);
                }
                break;
        }
    }
    // Dashes of about two units: the shader's are 0.5 on, 0.3 off.
    m_renderer->drawLines(gl, m_camera, lines, math::Vec3(0.95, 0.75, 0.3), 1.5f, 2, 0.25f);
}

void ViewportWidget::drawModelHighlights(QOpenGLExtraFunctions* gl) {
    if (m_modelSelection.empty() && !m_modelHover) return;
    const auto draw = [&](const ModelPick& pick, const math::Vec3& colour, float alpha) {
        for (const render::SceneNode* node : m_sceneGraph.collectVisibleMeshNodes()) {
            if (node->ownerId() != pick.owner) continue;
            const render::MeshData& mesh = node->mesh();
            const math::Mat4 model = node->worldTransform();
            const auto world = [&model](float x, float y, float z) {
                return model.transformPoint(math::Vec3(x, y, z));
            };
            if (pick.edge) {
                std::vector<float> lines;
                for (const auto& edge : mesh.edges) {
                    if (model::logicalEdge(edge.tag) != pick.tag) continue;  // every chord
                    const auto& p = edge.points;
                    for (size_t k = 0; k + 5 < p.size(); k += 3) {
                        for (const size_t at : {k, k + 3}) {
                            const math::Vec3 q = world(p[at], p[at + 1], p[at + 2]);
                            lines.insert(lines.end(),
                                         {static_cast<float>(q.x), static_cast<float>(q.y),
                                          static_cast<float>(q.z), 0.0f});
                        }
                    }
                }
                // Over the part's own line for the edge, at the same depth: an
                // equal depth must pass, or the highlight is hidden by it.
                gl->glDepthFunc(GL_LEQUAL);
                m_renderer->drawLines(gl, m_camera, lines, colour, 3.5f, 1, 1.0f, true);
                gl->glDepthFunc(GL_LESS);
                continue;
            }
            if (!mesh.hasFaces()) continue;
            // Every facet of the face picked.
            std::vector<bool> picked(mesh.faceTags.size(), false);
            bool any = false;
            for (size_t f = 0; f < mesh.faceTags.size(); ++f) {
                picked[f] = model::logicalFace(mesh.faceTags[f]) == pick.tag;
                any = any || picked[f];
            }
            if (!any) continue;
            std::vector<float> triangles;
            for (size_t t = 0; t < mesh.triangleFaces.size(); ++t) {
                const uint32_t index = mesh.triangleFaces[t];
                if (index >= picked.size() || !picked[index]) continue;
                for (size_t c = 0; c < 3; ++c) {
                    const size_t v = static_cast<size_t>(mesh.indices[t * 3 + c]) * 3;
                    if (v + 2 >= mesh.positions.size()) continue;
                    const math::Vec3 q =
                        world(mesh.positions[v], mesh.positions[v + 1], mesh.positions[v + 2]);
                    triangles.insert(triangles.end(),
                                     {static_cast<float>(q.x), static_cast<float>(q.y),
                                      static_cast<float>(q.z)});
                }
            }
            m_renderer->drawTriangles(gl, m_camera, triangles, math::Vec4(colour, alpha));
        }
    };
    for (const ModelPick& pick : m_modelSelection) draw(pick, math::Vec3(1.0, 0.55, 0.1), 0.45f);
    if (m_modelHover) draw(*m_modelHover, math::Vec3(0.35, 0.75, 1.0), 0.3f);
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

    applyProjection(w, h);
}

void ViewportWidget::applyProjection(int w, int h) {
    const double aspect = (h > 0) ? static_cast<double>(w) / static_cast<double>(h) : 1.0;
    if (!m_orthographic) {
        m_camera.setPerspective(45.0, aspect, 0.1, 10000.0);
        return;
    }
    // As tall as the view already is, as wide as the window now is.
    const double height = m_camera.projectionType() == render::ProjectionType::Orthographic
                              ? m_camera.orthoHeight()
                              : 2.0 * (m_camera.eye() - m_camera.target()).length() *
                                    std::tan(m_camera.fieldOfView() * math::kDegToRad / 2.0);
    m_camera.setOrthographic(height * aspect, height, -10000.0, 10000.0);
}

void ViewportWidget::setOrthographic(bool orthographic) {
    if (orthographic == m_orthographic) return;
    m_orthographic = orthographic;
    applyProjection(width(), height());
    update();
}

void ViewportWidget::setDisplayMode(render::DisplayMode mode) {
    m_displayMode = mode;
    update();
}

void ViewportWidget::setSectionPlane(const std::optional<math::Vec4>& plane) {
    m_sectionPlane = plane;
    update();
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
    // changed, and not in a paint. This frame shows the change at once with
    // the colours it had; the analysis runs next, and the colours follow.
    if (m_viewportRenderer.dofStale(m_document) && !m_dofQueued) {
        m_dofQueued = true;
        QTimer::singleShot(0, this, [this] {
            m_dofQueued = false;
            m_viewportRenderer.recomputeDOF(m_document);
            update();
        });
    }

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
    m_renderer->setDisplayMode(m_displayMode);
    if (m_sectionPlane) {
        m_renderer->setClipPlane(*m_sectionPlane);
    } else {
        m_renderer->clearClipPlane();
    }
    m_renderer->renderNodes(gl, m_sceneGraph, m_camera);
    drawDatums(gl);
    drawModelHighlights(gl);

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
            m_camera.setBackView();
            break;
        case Region::Bottom:
            m_camera.setBottomView();
            break;
        case Region::Left:
            m_camera.setLeftView();
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
