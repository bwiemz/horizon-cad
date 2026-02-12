#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/Tool.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Grid.h"
#include "horizon/document/Document.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/ui/GripManager.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec4.h"
#include "horizon/math/Mat4.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QOpenGLExtraFunctions>
#include <QPainter>
#include <QFont>
#include <QFontMetrics>
#include <QImage>

#include <cmath>

namespace {

static hz::math::Vec3 argbToVec3(uint32_t argb) {
    return { static_cast<double>((argb >> 16) & 0xFF) / 255.0,
             static_cast<double>((argb >> 8)  & 0xFF) / 255.0,
             static_cast<double>( argb        & 0xFF) / 255.0 };
}

struct BatchKey {
    uint32_t colorARGB;
    float lineWidth;
    bool operator==(const BatchKey& o) const {
        return colorARGB == o.colorARGB && lineWidth == o.lineWidth;
    }
};

}  // anonymous namespace

namespace hz::ui {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ViewportWidget
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
    if (m_textOverlayTex) gl->glDeleteTextures(1, &m_textOverlayTex);
    if (m_textOverlayVAO) gl->glDeleteVertexArrays(1, &m_textOverlayVAO);
    if (m_textOverlayVBO) gl->glDeleteBuffers(1, &m_textOverlayVBO);
    if (m_textOverlayShader) gl->glDeleteProgram(m_textOverlayShader);
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

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------

math::Vec2 ViewportWidget::worldPositionAtCursor(int screenX, int screenY) const {
    // screenToRay expects Qt-style coordinates (0 = top), so pass screenY directly.
    auto [rayOrigin, rayDir] = m_camera.screenToRay(
        static_cast<double>(screenX),
        static_cast<double>(screenY),
        width(), height());

    // Intersect with the XY plane (Z = 0).
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

// ---------------------------------------------------------------------------
// OpenGL overrides
// ---------------------------------------------------------------------------

void ViewportWidget::initializeGL() {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    m_renderer = std::make_unique<render::GLRenderer>();
    m_renderer->initialize(gl);
    m_renderer->setBackgroundColor(0.18f, 0.18f, 0.20f);

    // Set up GL resources for text overlay (QImage → texture → quad).
    initTextOverlayGL(gl);
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

    // Render document entities (collects dimension text info).
    m_dimTexts.clear();
    renderEntities(gl);

    // Render grip squares on selected entities.
    renderGrips(gl);

    // Render tool preview (rubber-band).
    renderToolPreview(gl);

    // GL overlays: crosshair, snap markers, axis indicator.
    m_overlayRenderer.setSnapResult(m_lastSnapResult);
    m_overlayRenderer.render(gl, m_camera, m_renderer.get(),
                             width(), height(), pixelToWorldScale());

    // Render text overlay: paint to an offscreen QImage (QPainter on QImage
    // is pure CPU — no Windows bitmap mask operations), then upload as a GL
    // texture and draw a fullscreen quad.  This avoids the Qt 6.10
    // qpixmap_win.cpp assertion triggered by QPainter on QOpenGLWidget.
    blitTextOverlay(gl);
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
            emit selectionChanged();
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

    // Update crosshair position for overlay renderer.
    m_overlayRenderer.setCrosshairWorldPos(wp);

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
        emit selectionChanged();
        update();
        return;
    }

    QOpenGLWidget::keyPressEvent(event);
}

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------

void ViewportWidget::renderEntities(QOpenGLExtraFunctions* gl) {
    if (!m_document) return;

    const auto& entities = m_document->draftDocument().entities();
    if (entities.empty()) return;

    const auto& layerMgr = m_document->layerManager();

    // Batches keyed by (color, lineWidth).
    std::vector<std::pair<BatchKey, std::vector<float>>> batches;

    auto findOrCreateBatch = [&](const BatchKey& key) -> std::vector<float>& {
        for (auto& [k, v] : batches) {
            if (k == key) return v;
        }
        batches.push_back({key, {}});
        return batches.back().second;
    };

    for (const auto& entity : entities) {
        // Layer visibility check.
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (lp && !lp->visible) continue;

        bool selected = m_selectionManager.isSelected(entity->id());

        // Resolve color.
        uint32_t resolvedColor;
        if (selected) {
            resolvedColor = 0xFFFF9900;  // orange
        } else if (entity->color() == 0x00000000) {
            resolvedColor = lp ? lp->color : 0xFFFFFFFF;
        } else {
            resolvedColor = entity->color();
        }

        // Resolve lineWidth.
        float resolvedWidth;
        if (entity->lineWidth() == 0.0) {
            resolvedWidth = lp ? static_cast<float>(lp->lineWidth) : 1.0f;
        } else {
            resolvedWidth = static_cast<float>(entity->lineWidth());
        }

        BatchKey key{resolvedColor, resolvedWidth};

        if (auto* line = dynamic_cast<const draft::DraftLine*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            verts.push_back(static_cast<float>(line->start().x));
            verts.push_back(static_cast<float>(line->start().y));
            verts.push_back(0.0f);
            verts.push_back(static_cast<float>(line->end().x));
            verts.push_back(static_cast<float>(line->end().y));
            verts.push_back(0.0f);
        } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
            auto verts = circleVertices(circle->center(), circle->radius());
            m_renderer->drawCircle(gl, m_camera, verts,
                                   argbToVec3(resolvedColor), resolvedWidth);
        } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(entity.get())) {
            auto verts = arcVertices(arc->center(), arc->radius(),
                                     arc->startAngle(), arc->endAngle());
            m_renderer->drawLines(gl, m_camera, verts,
                                  argbToVec3(resolvedColor), resolvedWidth);
        } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(entity.get())) {
            auto c = rect->corners();
            auto& verts = findOrCreateBatch(key);
            for (int i = 0; i < 4; ++i) {
                int j = (i + 1) % 4;
                verts.push_back(static_cast<float>(c[i].x));
                verts.push_back(static_cast<float>(c[i].y));
                verts.push_back(0.0f);
                verts.push_back(static_cast<float>(c[j].x));
                verts.push_back(static_cast<float>(c[j].y));
                verts.push_back(0.0f);
            }
        } else if (auto* polyline = dynamic_cast<const draft::DraftPolyline*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            const auto& pts = polyline->points();
            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                verts.push_back(static_cast<float>(pts[i].x));
                verts.push_back(static_cast<float>(pts[i].y));
                verts.push_back(0.0f);
                verts.push_back(static_cast<float>(pts[i + 1].x));
                verts.push_back(static_cast<float>(pts[i + 1].y));
                verts.push_back(0.0f);
            }
            if (polyline->closed() && pts.size() >= 2) {
                verts.push_back(static_cast<float>(pts.back().x));
                verts.push_back(static_cast<float>(pts.back().y));
                verts.push_back(0.0f);
                verts.push_back(static_cast<float>(pts[0].x));
                verts.push_back(static_cast<float>(pts[0].y));
                verts.push_back(0.0f);
            }
        } else if (auto* spline = dynamic_cast<const draft::DraftSpline*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            auto evalPts = spline->evaluate();
            for (size_t i = 0; i + 1 < evalPts.size(); ++i) {
                verts.push_back(static_cast<float>(evalPts[i].x));
                verts.push_back(static_cast<float>(evalPts[i].y));
                verts.push_back(0.0f);
                verts.push_back(static_cast<float>(evalPts[i + 1].x));
                verts.push_back(static_cast<float>(evalPts[i + 1].y));
                verts.push_back(0.0f);
            }
        } else if (auto* hatch = dynamic_cast<const draft::DraftHatch*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            // Draw boundary outline.
            const auto& bnd = hatch->boundary();
            for (size_t i = 0; i < bnd.size(); ++i) {
                size_t j = (i + 1) % bnd.size();
                verts.push_back(static_cast<float>(bnd[i].x));
                verts.push_back(static_cast<float>(bnd[i].y));
                verts.push_back(0.0f);
                verts.push_back(static_cast<float>(bnd[j].x));
                verts.push_back(static_cast<float>(bnd[j].y));
                verts.push_back(0.0f);
            }
            // Draw hatch fill lines.
            auto hatchLines = hatch->generateHatchLines();
            for (const auto& [a, b] : hatchLines) {
                verts.push_back(static_cast<float>(a.x));
                verts.push_back(static_cast<float>(a.y));
                verts.push_back(0.0f);
                verts.push_back(static_cast<float>(b.x));
                verts.push_back(static_cast<float>(b.y));
                verts.push_back(0.0f);
            }
        } else if (auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            auto evalPts = ellipse->evaluate();
            for (size_t i = 0; i + 1 < evalPts.size(); ++i) {
                verts.push_back(static_cast<float>(evalPts[i].x));
                verts.push_back(static_cast<float>(evalPts[i].y));
                verts.push_back(0.0f);
                verts.push_back(static_cast<float>(evalPts[i + 1].x));
                verts.push_back(static_cast<float>(evalPts[i + 1].y));
                verts.push_back(0.0f);
            }
        } else if (auto* dim = dynamic_cast<const draft::DraftDimension*>(entity.get())) {
            const auto& style = m_document->draftDocument().dimensionStyle();
            auto& verts = findOrCreateBatch(key);

            auto addSegments = [&](const std::vector<std::pair<math::Vec2, math::Vec2>>& segs) {
                for (const auto& [a, b] : segs) {
                    verts.push_back(static_cast<float>(a.x));
                    verts.push_back(static_cast<float>(a.y));
                    verts.push_back(0.0f);
                    verts.push_back(static_cast<float>(b.x));
                    verts.push_back(static_cast<float>(b.y));
                    verts.push_back(0.0f);
                }
            };

            addSegments(dim->extensionLines(style));
            addSegments(dim->dimensionLines(style));
            addSegments(dim->arrowheadLines(style));

            // Collect text for QPainter overlay.
            m_dimTexts.push_back({dim->textPosition(),
                                  dim->displayText(style), resolvedColor});
        } else if (auto* bref = dynamic_cast<const draft::DraftBlockRef*>(entity.get())) {
            // Render each sub-entity of the block definition, transformed to world space.
            for (const auto& subEnt : bref->definition()->entities) {
                // ByBlock color resolution: sub-entity color 0 → use block ref's resolved color.
                uint32_t subColor = subEnt->color();
                if (subColor == 0x00000000) subColor = resolvedColor;
                float subWidth = static_cast<float>(subEnt->lineWidth());
                if (subWidth == 0.0f) subWidth = resolvedWidth;
                BatchKey subKey{subColor, subWidth};

                if (auto* ln = dynamic_cast<const draft::DraftLine*>(subEnt.get())) {
                    auto p1 = bref->transformPoint(ln->start());
                    auto p2 = bref->transformPoint(ln->end());
                    auto& v = findOrCreateBatch(subKey);
                    v.push_back(static_cast<float>(p1.x)); v.push_back(static_cast<float>(p1.y)); v.push_back(0.0f);
                    v.push_back(static_cast<float>(p2.x)); v.push_back(static_cast<float>(p2.y)); v.push_back(0.0f);
                } else if (auto* ci = dynamic_cast<const draft::DraftCircle*>(subEnt.get())) {
                    auto wc = bref->transformPoint(ci->center());
                    double wr = ci->radius() * std::abs(bref->uniformScale());
                    auto cv = circleVertices(wc, wr);
                    m_renderer->drawCircle(gl, m_camera, cv, argbToVec3(subColor), subWidth);
                } else if (auto* ar = dynamic_cast<const draft::DraftArc*>(subEnt.get())) {
                    auto wc = bref->transformPoint(ar->center());
                    double wr = ar->radius() * std::abs(bref->uniformScale());
                    double sa = ar->startAngle() + bref->rotation();
                    double ea = ar->endAngle() + bref->rotation();
                    if (bref->uniformScale() < 0.0) {
                        // Mirrored: reverse arc direction.
                        double tmp = sa; sa = -ea; ea = -tmp;
                    }
                    auto av = arcVertices(wc, wr, sa, ea);
                    m_renderer->drawLines(gl, m_camera, av, argbToVec3(subColor), subWidth);
                } else if (auto* re = dynamic_cast<const draft::DraftRectangle*>(subEnt.get())) {
                    auto c = re->corners();
                    auto& v = findOrCreateBatch(subKey);
                    for (int i = 0; i < 4; ++i) {
                        auto wp1 = bref->transformPoint(c[i]);
                        auto wp2 = bref->transformPoint(c[(i + 1) % 4]);
                        v.push_back(static_cast<float>(wp1.x)); v.push_back(static_cast<float>(wp1.y)); v.push_back(0.0f);
                        v.push_back(static_cast<float>(wp2.x)); v.push_back(static_cast<float>(wp2.y)); v.push_back(0.0f);
                    }
                } else if (auto* pl = dynamic_cast<const draft::DraftPolyline*>(subEnt.get())) {
                    auto& v = findOrCreateBatch(subKey);
                    const auto& pts = pl->points();
                    for (size_t i = 0; i + 1 < pts.size(); ++i) {
                        auto wp1 = bref->transformPoint(pts[i]);
                        auto wp2 = bref->transformPoint(pts[i + 1]);
                        v.push_back(static_cast<float>(wp1.x)); v.push_back(static_cast<float>(wp1.y)); v.push_back(0.0f);
                        v.push_back(static_cast<float>(wp2.x)); v.push_back(static_cast<float>(wp2.y)); v.push_back(0.0f);
                    }
                    if (pl->closed() && pts.size() >= 2) {
                        auto wp1 = bref->transformPoint(pts.back());
                        auto wp2 = bref->transformPoint(pts[0]);
                        v.push_back(static_cast<float>(wp1.x)); v.push_back(static_cast<float>(wp1.y)); v.push_back(0.0f);
                        v.push_back(static_cast<float>(wp2.x)); v.push_back(static_cast<float>(wp2.y)); v.push_back(0.0f);
                    }
                } else if (auto* sp = dynamic_cast<const draft::DraftSpline*>(subEnt.get())) {
                    auto& v = findOrCreateBatch(subKey);
                    auto evalPts = sp->evaluate();
                    for (size_t i = 0; i + 1 < evalPts.size(); ++i) {
                        auto wp1 = bref->transformPoint(evalPts[i]);
                        auto wp2 = bref->transformPoint(evalPts[i + 1]);
                        v.push_back(static_cast<float>(wp1.x)); v.push_back(static_cast<float>(wp1.y)); v.push_back(0.0f);
                        v.push_back(static_cast<float>(wp2.x)); v.push_back(static_cast<float>(wp2.y)); v.push_back(0.0f);
                    }
                } else if (auto* el = dynamic_cast<const draft::DraftEllipse*>(subEnt.get())) {
                    auto& v = findOrCreateBatch(subKey);
                    auto evalPts = el->evaluate();
                    for (size_t i = 0; i + 1 < evalPts.size(); ++i) {
                        auto wp1 = bref->transformPoint(evalPts[i]);
                        auto wp2 = bref->transformPoint(evalPts[i + 1]);
                        v.push_back(static_cast<float>(wp1.x)); v.push_back(static_cast<float>(wp1.y)); v.push_back(0.0f);
                        v.push_back(static_cast<float>(wp2.x)); v.push_back(static_cast<float>(wp2.y)); v.push_back(0.0f);
                    }
                }
            }
        } else if (auto* txt = dynamic_cast<const draft::DraftText*>(entity.get())) {
            // Text entity — collect for QPainter overlay.
            m_dimTexts.push_back({txt->position(), txt->text(), resolvedColor,
                                  txt->textHeight(), txt->rotation(),
                                  static_cast<int>(txt->alignment())});
        }
    }

    // Draw all batches.
    for (const auto& [key, verts] : batches) {
        if (!verts.empty()) {
            m_renderer->drawLines(gl, m_camera, verts,
                                  argbToVec3(key.colorARGB), key.lineWidth);
        }
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

    // Preview arcs (e.g. rubber-band for ArcTool).
    auto previewArcs = m_activeTool->getPreviewArcs();
    if (!previewArcs.empty()) {
        math::Vec3 cyan{0.0, 0.8, 1.0};
        for (const auto& arc : previewArcs) {
            auto verts = arcVertices(arc.center, arc.radius,
                                     arc.startAngle, arc.endAngle);
            m_renderer->drawLines(gl, m_camera, verts, cyan);
        }
    }

    // Snap indicator is now rendered by OverlayRenderer in paintGL().
}

void ViewportWidget::renderGrips(QOpenGLExtraFunctions* gl) {
    if (!m_document) return;
    auto selectedIds = m_selectionManager.selectedIds();
    if (selectedIds.empty()) return;

    // Grip square size in world units (6 pixels).
    double s = 6.0 * pixelToWorldScale();

    std::vector<float> verts;
    math::Vec3 green{0.0, 1.0, 0.3};

    const auto& doc = m_document->draftDocument();
    for (uint64_t id : selectedIds) {
        for (const auto& e : doc.entities()) {
            if (e->id() != id) continue;
            auto grips = GripManager::gripPoints(*e);
            for (const auto& g : grips) {
                float gx = static_cast<float>(g.x);
                float gy = static_cast<float>(g.y);
                float hs = static_cast<float>(s * 0.5);
                // Draw a small square (4 line segments).
                verts.push_back(gx - hs); verts.push_back(gy - hs); verts.push_back(0.0f);
                verts.push_back(gx + hs); verts.push_back(gy - hs); verts.push_back(0.0f);

                verts.push_back(gx + hs); verts.push_back(gy - hs); verts.push_back(0.0f);
                verts.push_back(gx + hs); verts.push_back(gy + hs); verts.push_back(0.0f);

                verts.push_back(gx + hs); verts.push_back(gy + hs); verts.push_back(0.0f);
                verts.push_back(gx - hs); verts.push_back(gy + hs); verts.push_back(0.0f);

                verts.push_back(gx - hs); verts.push_back(gy + hs); verts.push_back(0.0f);
                verts.push_back(gx - hs); verts.push_back(gy - hs); verts.push_back(0.0f);
            }
            break;
        }
    }

    if (!verts.empty()) {
        m_renderer->drawLines(gl, m_camera, verts, green, 1.5f);
    }
}

std::vector<float> ViewportWidget::circleVertices(const math::Vec2& center, double radius,
                                                   int segments) const {
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

std::vector<float> ViewportWidget::arcVertices(const math::Vec2& center, double radius,
                                                double startAngle, double endAngle,
                                                int segments) const {
    double sweep = endAngle - startAngle;
    if (sweep <= 0.0) sweep += math::kTwoPi;

    int arcSegments = std::max(4, static_cast<int>(segments * sweep / math::kTwoPi));
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>(arcSegments) * 6);

    double step = sweep / static_cast<double>(arcSegments);
    for (int i = 0; i < arcSegments; ++i) {
        double a0 = startAngle + step * static_cast<double>(i);
        double a1 = startAngle + step * static_cast<double>(i + 1);

        float x0 = static_cast<float>(center.x + radius * std::cos(a0));
        float y0 = static_cast<float>(center.y + radius * std::sin(a0));
        float x1 = static_cast<float>(center.x + radius * std::cos(a1));
        float y1 = static_cast<float>(center.y + radius * std::sin(a1));

        verts.push_back(x0); verts.push_back(y0); verts.push_back(0.0f);
        verts.push_back(x1); verts.push_back(y1); verts.push_back(0.0f);
    }
    return verts;
}

// ---------------------------------------------------------------------------
// Dimension text rendering
// ---------------------------------------------------------------------------

QPointF ViewportWidget::worldToScreen(const math::Vec2& wp) const {
    math::Vec4 clip = m_camera.viewProjectionMatrix()
                      * math::Vec4(math::Vec3(wp.x, wp.y, 0.0), 1.0);
    if (std::abs(clip.w) < 1e-15) return {0.0, 0.0};

    math::Vec3 ndc = clip.perspectiveDivide();
    double sx = (ndc.x + 1.0) * 0.5 * width();
    double sy = (1.0 - ndc.y) * 0.5 * height();  // flip Y for Qt screen coords
    return {sx, sy};
}

void ViewportWidget::renderTextToImage(QImage& image) {
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);

    // Helper to draw a single text item at a world position.
    struct TextItem {
        math::Vec2 worldPos;
        std::string text;
        uint32_t color;
        int fontSize;
        bool bold;
        double rotation = 0.0;
        int alignment = 1;  // 0=Left, 1=Center, 2=Right
    };

    auto drawItem = [&](const TextItem& item) {
        QPointF sp = worldToScreen(item.worldPos);
        QColor qc(static_cast<int>((item.color >> 16) & 0xFF),
                   static_cast<int>((item.color >> 8) & 0xFF),
                   static_cast<int>(item.color & 0xFF));
        painter.setPen(qc);

        QFont font("Arial", item.fontSize);
        font.setBold(item.bold);
        painter.setFont(font);

        QString text = QString::fromUtf8(item.text.c_str());
        QFontMetrics fm(painter.font());
        int tw = fm.horizontalAdvance(text);
        int th = fm.ascent();

        if (std::abs(item.rotation) > 1e-6) {
            painter.save();
            painter.translate(sp);
            // QPainter rotates clockwise in degrees; world rotation is CCW radians.
            painter.rotate(-item.rotation * 180.0 / 3.14159265358979323846);
            double dx = 0.0;
            if (item.alignment == 0)      dx = 0.0;
            else if (item.alignment == 1)  dx = -tw * 0.5;
            else                           dx = -tw;
            painter.drawText(QPointF(dx, th * 0.25), text);
            painter.restore();
        } else {
            double dx = 0.0;
            if (item.alignment == 0)      dx = 0.0;
            else if (item.alignment == 1)  dx = -tw * 0.5;
            else                           dx = -tw;
            painter.drawText(QPointF(sp.x() + dx, sp.y() + th * 0.25), text);
        }
    };

    // --- Dimension + text entity text ---
    if (!m_dimTexts.empty() && m_document) {
        const auto& style = m_document->draftDocument().dimensionStyle();
        double pxPerWorld = 1.0 / pixelToWorldScale();
        int defaultFontSize = std::max(8, std::min(48,
            static_cast<int>(style.textHeight * pxPerWorld * 0.4)));

        for (const auto& dt : m_dimTexts) {
            int fs = defaultFontSize;
            if (dt.textHeight > 0.0) {
                fs = std::max(8, std::min(200,
                    static_cast<int>(dt.textHeight * pxPerWorld * 0.4)));
            }
            drawItem({dt.worldPos, dt.text, dt.color, fs, false,
                      dt.rotation, dt.alignment});
        }
    }

    // --- Constraint indicators ---
    if (m_document) {
        const auto& csys = m_document->constraintSystem();
        if (!csys.empty()) {
            const auto& entities = m_document->draftDocument().entities();
            double pxPerWorld = 1.0 / pixelToWorldScale();
            int fontSize = std::max(8, std::min(24, static_cast<int>(pxPerWorld * 0.3)));

            for (const auto& c : csys.constraints()) {
                math::Vec2 pos{0.0, 0.0};
                std::string symbol;
                uint32_t color = 0xFF00CC00;  // Green (satisfied)

                switch (c->type()) {
                    case cstr::ConstraintType::Coincident:  symbol = "\xE2\x97\x8F"; break;
                    case cstr::ConstraintType::Horizontal:  symbol = "H"; break;
                    case cstr::ConstraintType::Vertical:    symbol = "V"; break;
                    case cstr::ConstraintType::Perpendicular: symbol = "\xE2\x9F\x82"; break;
                    case cstr::ConstraintType::Parallel:    symbol = "\xE2\x88\xA5"; break;
                    case cstr::ConstraintType::Tangent:     symbol = "T"; break;
                    case cstr::ConstraintType::Equal:       symbol = "="; break;
                    case cstr::ConstraintType::Fixed:       symbol = "\xE2\x96\xA1"; break;
                    case cstr::ConstraintType::Distance: {
                        auto* dc = dynamic_cast<const cstr::DistanceConstraint*>(c.get());
                        if (dc) {
                            char buf[32];
                            snprintf(buf, sizeof(buf), "%.2f", dc->dimensionalValue());
                            symbol = buf;
                        }
                        color = 0xFF0066CC;
                        break;
                    }
                    case cstr::ConstraintType::Angle: {
                        auto* ac = dynamic_cast<const cstr::AngleConstraint*>(c.get());
                        if (ac) {
                            char buf[32];
                            snprintf(buf, sizeof(buf), "%.1f\xC2\xB0",
                                     ac->dimensionalValue() * 180.0 / 3.14159265358979323846);
                            symbol = buf;
                        }
                        color = 0xFF0066CC;
                        break;
                    }
                }

                // Compute indicator position: midpoint of referenced features.
                try {
                    auto ids = c->referencedEntityIds();
                    if (!ids.empty()) {
                        const auto* e1 = cstr::findEntity(ids[0], entities);
                        if (e1) {
                            auto snaps = e1->snapPoints();
                            if (!snaps.empty()) pos = snaps[0];
                            if (ids.size() > 1) {
                                const auto* e2 = cstr::findEntity(ids.back(), entities);
                                if (e2) {
                                    auto snaps2 = e2->snapPoints();
                                    if (!snaps2.empty()) {
                                        pos = {(pos.x + snaps2[0].x) * 0.5,
                                               (pos.y + snaps2[0].y) * 0.5};
                                    }
                                }
                            }
                        }
                    }
                } catch (...) {}

                // Offset slightly above the position.
                pos.y += pixelToWorldScale() * 12.0;

                drawItem({pos, symbol, color, fontSize, true});
            }
        }
    }

    painter.end();
}

// ---------------------------------------------------------------------------
// Text overlay GL resources — render text to QImage then blit via GL texture
// ---------------------------------------------------------------------------

void ViewportWidget::initTextOverlayGL(QOpenGLExtraFunctions* gl) {
    // --- Shader program ---
    const char* vertSrc = R"(
        #version 330 core
        layout(location = 0) in vec2 aPos;
        layout(location = 1) in vec2 aUV;
        out vec2 vUV;
        void main() {
            gl_Position = vec4(aPos, 0.0, 1.0);
            vUV = aUV;
        }
    )";
    const char* fragSrc = R"(
        #version 330 core
        in vec2 vUV;
        out vec4 FragColor;
        uniform sampler2D uTex;
        void main() {
            FragColor = texture(uTex, vUV);
        }
    )";

    auto compileShader = [&](unsigned int type, const char* src) -> unsigned int {
        unsigned int s = gl->glCreateShader(type);
        gl->glShaderSource(s, 1, &src, nullptr);
        gl->glCompileShader(s);
        return s;
    };

    unsigned int vs = compileShader(GL_VERTEX_SHADER, vertSrc);
    unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragSrc);
    m_textOverlayShader = gl->glCreateProgram();
    gl->glAttachShader(m_textOverlayShader, vs);
    gl->glAttachShader(m_textOverlayShader, fs);
    gl->glLinkProgram(m_textOverlayShader);
    gl->glDeleteShader(vs);
    gl->glDeleteShader(fs);

    // --- Fullscreen quad (NDC coords + UVs) ---
    // Two triangles covering [-1,1] in clip space.
    float quadVerts[] = {
        // pos        uv
        -1.f, -1.f,  0.f, 0.f,
         1.f, -1.f,  1.f, 0.f,
         1.f,  1.f,  1.f, 1.f,

        -1.f, -1.f,  0.f, 0.f,
         1.f,  1.f,  1.f, 1.f,
        -1.f,  1.f,  0.f, 1.f,
    };

    gl->glGenVertexArrays(1, &m_textOverlayVAO);
    gl->glGenBuffers(1, &m_textOverlayVBO);
    gl->glBindVertexArray(m_textOverlayVAO);
    gl->glBindBuffer(GL_ARRAY_BUFFER, m_textOverlayVBO);
    gl->glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));
    gl->glBindVertexArray(0);

    // --- Texture ---
    gl->glGenTextures(1, &m_textOverlayTex);
    gl->glBindTexture(GL_TEXTURE_2D, m_textOverlayTex);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    gl->glBindTexture(GL_TEXTURE_2D, 0);
}

void ViewportWidget::blitTextOverlay(QOpenGLExtraFunctions* gl) {
    int w = width();
    int h = height();
    if (w <= 0 || h <= 0) return;

    // 1. Render text to a QImage (QPainter on QImage is pure CPU).
    QImage image(w, h, QImage::Format_RGBA8888_Premultiplied);
    image.fill(Qt::transparent);
    renderTextToImage(image);

    // 2. Upload QImage pixels to the GL texture.
    gl->glBindTexture(GL_TEXTURE_2D, m_textOverlayTex);
    gl->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, image.constBits());

    // 3. Draw a fullscreen quad with alpha blending.
    // Use premultiplied-alpha blend (GL_ONE) since the QImage is premultiplied.
    gl->glDisable(GL_DEPTH_TEST);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    gl->glUseProgram(m_textOverlayShader);
    int loc = gl->glGetUniformLocation(m_textOverlayShader, "uTex");
    gl->glUniform1i(loc, 0);
    gl->glActiveTexture(GL_TEXTURE0);
    gl->glBindTexture(GL_TEXTURE_2D, m_textOverlayTex);

    gl->glBindVertexArray(m_textOverlayVAO);
    gl->glDrawArrays(GL_TRIANGLES, 0, 6);
    gl->glBindVertexArray(0);

    gl->glUseProgram(0);
    gl->glBindTexture(GL_TEXTURE_2D, 0);
    gl->glEnable(GL_DEPTH_TEST);
}

}  // namespace hz::ui
