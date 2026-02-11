#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/Tool.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Grid.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftDimension.h"
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

    // Render tool preview (rubber-band).
    renderToolPreview(gl);

    // QPainter overlay for dimension text.
    renderDimensionText();
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

    // Snap indicator (yellow cross).
    if (m_lastSnapResult.type != draft::SnapType::None) {
        math::Vec2 sp = m_lastSnapResult.point;
        float s = 0.15f;
        std::vector<float> indicator = {
            static_cast<float>(sp.x - s), static_cast<float>(sp.y), 0.0f,
            static_cast<float>(sp.x + s), static_cast<float>(sp.y), 0.0f,
            static_cast<float>(sp.x), static_cast<float>(sp.y - s), 0.0f,
            static_cast<float>(sp.x), static_cast<float>(sp.y + s), 0.0f,
        };
        math::Vec3 yellow{1.0, 1.0, 0.0};
        m_renderer->drawLines(gl, m_camera, indicator, yellow);
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

void ViewportWidget::renderDimensionText() {
    if (m_dimTexts.empty() || !m_document) return;

    // Compute a font size: textHeight in world units → screen pixels.
    const auto& style = m_document->draftDocument().dimensionStyle();
    double pxPerWorld = 1.0 / pixelToWorldScale();
    int fontSize = std::max(8, std::min(48,
        static_cast<int>(style.textHeight * pxPerWorld * 0.4)));

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    QFont font("Arial", fontSize);
    painter.setFont(font);
    QFontMetrics fm(font);

    for (const auto& dt : m_dimTexts) {
        QPointF sp = worldToScreen(dt.worldPos);

        // Color from ARGB.
        QColor qc(static_cast<int>((dt.color >> 16) & 0xFF),
                   static_cast<int>((dt.color >> 8)  & 0xFF),
                   static_cast<int>( dt.color        & 0xFF));
        painter.setPen(qc);

        QString text = QString::fromStdString(dt.text);
        int tw = fm.horizontalAdvance(text);
        int th = fm.ascent();

        // Draw centered on the projected position.
        painter.drawText(QPointF(sp.x() - tw * 0.5, sp.y() + th * 0.25), text);
    }

    painter.end();
}

}  // namespace hz::ui
