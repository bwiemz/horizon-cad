#include "horizon/ui/ViewportRenderer.h"

#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QOpenGLExtraFunctions>
#include <QPainter>
#include <QPointF>
#include <algorithm>
#include <cmath>
#include <optional>
#include <set>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/SketchSolver.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/PlotScene.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/math/Vec4.h"
#include "horizon/render/Camera.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/SelectionManager.h"
#include "horizon/ui/GripManager.h"
#include "horizon/ui/SelectTool.h"
#include "horizon/ui/Tool.h"

namespace {

static hz::math::Vec3 argbToVec3(uint32_t argb) {
    return {static_cast<double>((argb >> 16) & 0xFF) / 255.0,
            static_cast<double>((argb >> 8) & 0xFF) / 255.0,
            static_cast<double>(argb & 0xFF) / 255.0};
}

}  // anonymous namespace

namespace hz::ui {

// ---------------------------------------------------------------------------
// GL resource management
// ---------------------------------------------------------------------------

void ViewportRenderer::initTextOverlayGL(QOpenGLExtraFunctions* gl) {
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
    // Two triangles covering [-1,1] in clip space. The image's first row, its
    // top, is the texture's v = 0: the top of the view samples v = 0. (The
    // other way up, the overlay was drawn upside down: texts mirrored across
    // the view from what they label, the view cube at the bottom.)
    // clang-format off
    float quadVerts[] = {
        // pos        uv
        -1.f, -1.f,   0.f, 1.f,
         1.f, -1.f,   1.f, 1.f,
         1.f,  1.f,   1.f, 0.f,

        -1.f, -1.f,   0.f, 1.f,
         1.f,  1.f,   1.f, 0.f,
        -1.f,  1.f,   0.f, 0.f,
    };
    // clang-format on

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

void ViewportRenderer::destroyGL(QOpenGLExtraFunctions* gl) {
    for (auto& buffer : m_lineBuffers) render::GLRenderer::releaseLineBuffer(gl, buffer);
    m_lineBuffers.clear();
    m_uploadedBuild = 0;  // a new context is sent the cache afresh
    m_overlayUploaded = false;
    m_overlayTextureSize = QSize();
    if (m_textOverlayTex) gl->glDeleteTextures(1, &m_textOverlayTex);
    if (m_textOverlayVAO) gl->glDeleteVertexArrays(1, &m_textOverlayVAO);
    if (m_textOverlayVBO) gl->glDeleteBuffers(1, &m_textOverlayVBO);
    if (m_textOverlayShader) gl->glDeleteProgram(m_textOverlayShader);
    m_textOverlayTex = 0;
    m_textOverlayVAO = 0;
    m_textOverlayVBO = 0;
    m_textOverlayShader = 0;
}

// ---------------------------------------------------------------------------
// DOF visualization
// ---------------------------------------------------------------------------

void ViewportRenderer::recomputeDOF(doc::Document* doc) {
    const std::uint64_t revision = doc ? doc->undoStack().revision() : 0;
    // The drawing too: editing a sketch changes what is drawn, not the
    // document's revision.
    const draft::DraftDocument* drawing = doc ? &doc->activeDrawing() : nullptr;
    if (!m_dofDirty && doc == m_dofDocument && revision == m_dofRevision &&
        drawing == m_dofDrawing) {
        return;
    }
    m_dofDocument = doc;
    m_dofRevision = revision;
    m_dofDrawing = drawing;
    ++m_dofComputations;
    if (!doc) {
        m_dofAnalysis = {};
        m_dofDirty = false;
        return;
    }

    const auto& csys = doc->activeConstraints();
    if (csys.empty()) {
        m_dofAnalysis = {};
        m_dofDirty = false;
        return;
    }

    auto params = cstr::ParameterTable::buildFromEntities(doc->activeDrawing().entities(), csys);
    cstr::SketchSolver solver;
    m_dofAnalysis = solver.analyzeDOF(params, csys);
    m_dofDirty = false;
}

// ---------------------------------------------------------------------------
// Entity rendering
// ---------------------------------------------------------------------------

bool ViewportRenderer::prepareEntities(const doc::Document& doc,
                                       const render::SelectionManager& selection) {
    return m_drawing.update(doc, selection, m_dofAnalysis, m_dofComputations);
}

void ViewportRenderer::renderEntities(QOpenGLExtraFunctions* gl, render::GLRenderer& renderer,
                                      const render::Camera& camera, doc::Document& doc,
                                      const render::SelectionManager& selection) {
    prepareEntities(doc, selection);
    const auto& batches = m_drawing.batches();

    // The GPU's copy follows the cache: a buffer for each batch, sent again
    // only after a build.
    if (m_uploadedBuild != m_drawing.builds()) {
        for (size_t i = batches.size(); i < m_lineBuffers.size(); ++i) {
            render::GLRenderer::releaseLineBuffer(gl, m_lineBuffers[i]);
        }
        m_lineBuffers.resize(batches.size());
        for (size_t i = 0; i < batches.size(); ++i) {
            renderer.uploadLineBuffer(gl, m_lineBuffers[i], batches[i].vertices);
        }
        m_uploadedBuild = m_drawing.builds();
    }
    if (batches.empty()) return;

    // Only the chunks in view.
    const std::vector<bool> visible = m_drawing.visibleChunks(camera.viewProjectionMatrix());
    std::vector<render::GLRenderer::LineRun> runs;
    for (size_t i = 0; i < batches.size(); ++i) {
        runs.clear();
        for (const auto& range : DrawingCache::visibleRanges(batches[i], visible)) {
            runs.emplace_back(range.first, range.count);
        }
        const DrawingCache::Pen& pen = batches[i].pen;
        renderer.drawLineBuffer(gl, camera, m_lineBuffers[i], runs, argbToVec3(pen.color),
                                pen.width, pen.lineType);
    }
}

// ---------------------------------------------------------------------------
// Tool preview rendering
// ---------------------------------------------------------------------------

void ViewportRenderer::renderToolPreview(QOpenGLExtraFunctions* gl, render::GLRenderer& renderer,
                                         const render::Camera& camera, Tool* activeTool) {
    if (!activeTool) return;

    math::Vec3 previewCol = activeTool->previewColor();

    // Preview lines (e.g. rubber-band for LineTool, selection rectangle for SelectTool).
    auto previewLines = activeTool->getPreviewLines();
    if (!previewLines.empty()) {
        std::vector<float> verts;
        verts.reserve(previewLines.size() * 8);
        for (const auto& [start, end] : previewLines) {
            double len = start.distanceTo(end);
            verts.push_back(static_cast<float>(start.x));
            verts.push_back(static_cast<float>(start.y));
            verts.push_back(0.0f);
            verts.push_back(0.0f);
            verts.push_back(static_cast<float>(end.x));
            verts.push_back(static_cast<float>(end.y));
            verts.push_back(0.0f);
            verts.push_back(static_cast<float>(len));
        }
        renderer.drawLines(gl, camera, verts, previewCol);
    }

    // Preview circles (e.g. rubber-band for CircleTool).
    auto previewCircles = activeTool->getPreviewCircles();
    if (!previewCircles.empty()) {
        for (const auto& [center, radius] : previewCircles) {
            auto verts = circleVertices(center, radius);
            renderer.drawCircle(gl, camera, verts, previewCol);
        }
    }

    // Preview arcs (e.g. rubber-band for ArcTool).
    auto previewArcs = activeTool->getPreviewArcs();
    if (!previewArcs.empty()) {
        for (const auto& arc : previewArcs) {
            auto verts = arcVertices(arc.center, arc.radius, arc.startAngle, arc.endAngle);
            renderer.drawLines(gl, camera, verts, previewCol);
        }
    }

    // Filled selection rectangle for SelectTool box selection.
    if (auto* selectTool = dynamic_cast<SelectTool*>(activeTool)) {
        if (selectTool->isDraggingBox()) {
            bool isWindow = selectTool->isWindowSelection();
            math::Vec4 fillColor = isWindow ? math::Vec4{0.2, 0.4, 0.8, 0.12}   // Blue tint
                                            : math::Vec4{0.2, 0.8, 0.4, 0.12};  // Green tint
            renderer.drawFilledQuad(gl, camera, selectTool->boxCorner1(), selectTool->boxCorner2(),
                                    fillColor);
        }
    }

    // Snap indicator is now rendered by OverlayRenderer in paintGL().
}

// ---------------------------------------------------------------------------
// Grip rendering
// ---------------------------------------------------------------------------

void ViewportRenderer::renderGrips(QOpenGLExtraFunctions* gl, render::GLRenderer& renderer,
                                   const render::Camera& camera, doc::Document& doc,
                                   const render::SelectionManager& selection,
                                   double pixelToWorldScale) {
    auto selectedIds = selection.selectedIds();
    if (selectedIds.empty()) return;

    // Grip square size in world units (6 pixels).
    double s = 6.0 * pixelToWorldScale;

    std::vector<float> verts;
    math::Vec3 green{0.0, 1.0, 0.3};

    const auto& draftDoc = doc.activeDrawing();
    for (uint64_t id : selectedIds) {
        const draft::DraftEntity* e = draftDoc.findEntity(id);
        if (e == nullptr) continue;
        auto grips = GripManager::gripPoints(*e);
        for (const auto& g : grips) {
            float gx = static_cast<float>(g.x);
            float gy = static_cast<float>(g.y);
            float hs = static_cast<float>(s * 0.5);
            float side = hs * 2.0f;
            // Draw a small square (4 line segments) with distance attribute.
            verts.push_back(gx - hs);
            verts.push_back(gy - hs);
            verts.push_back(0.0f);
            verts.push_back(0.0f);
            verts.push_back(gx + hs);
            verts.push_back(gy - hs);
            verts.push_back(0.0f);
            verts.push_back(side);

            verts.push_back(gx + hs);
            verts.push_back(gy - hs);
            verts.push_back(0.0f);
            verts.push_back(side);
            verts.push_back(gx + hs);
            verts.push_back(gy + hs);
            verts.push_back(0.0f);
            verts.push_back(side * 2.0f);

            verts.push_back(gx + hs);
            verts.push_back(gy + hs);
            verts.push_back(0.0f);
            verts.push_back(side * 2.0f);
            verts.push_back(gx - hs);
            verts.push_back(gy + hs);
            verts.push_back(0.0f);
            verts.push_back(side * 3.0f);

            verts.push_back(gx - hs);
            verts.push_back(gy + hs);
            verts.push_back(0.0f);
            verts.push_back(side * 3.0f);
            verts.push_back(gx - hs);
            verts.push_back(gy - hs);
            verts.push_back(0.0f);
            verts.push_back(side * 4.0f);
        }
    }

    if (!verts.empty()) {
        renderer.drawLines(gl, camera, verts, green, 1.5f);
    }
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

std::vector<float> ViewportRenderer::circleVertices(const math::Vec2& center, double radius,
                                                    int segments) const {
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>(segments) * 8);

    const double step = 2.0 * 3.14159265358979323846 / static_cast<double>(segments);
    const double arcStep = radius * step;  // Arc-length per segment.
    for (int i = 0; i < segments; ++i) {
        double a0 = step * static_cast<double>(i);
        double a1 = step * static_cast<double>((i + 1) % segments);

        float x0 = static_cast<float>(center.x + radius * std::cos(a0));
        float y0 = static_cast<float>(center.y + radius * std::sin(a0));
        float x1 = static_cast<float>(center.x + radius * std::cos(a1));
        float y1 = static_cast<float>(center.y + radius * std::sin(a1));
        float d0 = static_cast<float>(arcStep * static_cast<double>(i));
        float d1 = static_cast<float>(arcStep * static_cast<double>(i + 1));

        verts.push_back(x0);
        verts.push_back(y0);
        verts.push_back(0.0f);
        verts.push_back(d0);
        verts.push_back(x1);
        verts.push_back(y1);
        verts.push_back(0.0f);
        verts.push_back(d1);
    }
    return verts;
}

std::vector<float> ViewportRenderer::arcVertices(const math::Vec2& center, double radius,
                                                 double startAngle, double endAngle,
                                                 int segments) const {
    double sweep = endAngle - startAngle;
    if (sweep <= 0.0) sweep += math::kTwoPi;

    int arcSegments = std::max(4, static_cast<int>(segments * sweep / math::kTwoPi));
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>(arcSegments) * 8);

    double step = sweep / static_cast<double>(arcSegments);
    double arcStep = radius * step;  // Arc-length per segment.
    for (int i = 0; i < arcSegments; ++i) {
        double a0 = startAngle + step * static_cast<double>(i);
        double a1 = startAngle + step * static_cast<double>(i + 1);

        float x0 = static_cast<float>(center.x + radius * std::cos(a0));
        float y0 = static_cast<float>(center.y + radius * std::sin(a0));
        float x1 = static_cast<float>(center.x + radius * std::cos(a1));
        float y1 = static_cast<float>(center.y + radius * std::sin(a1));
        float d0 = static_cast<float>(arcStep * static_cast<double>(i));
        float d1 = static_cast<float>(arcStep * static_cast<double>(i + 1));

        verts.push_back(x0);
        verts.push_back(y0);
        verts.push_back(0.0f);
        verts.push_back(d0);
        verts.push_back(x1);
        verts.push_back(y1);
        verts.push_back(0.0f);
        verts.push_back(d1);
    }
    return verts;
}

// ---------------------------------------------------------------------------
// Text overlay
// ---------------------------------------------------------------------------

QPointF ViewportRenderer::worldToScreen(const render::Camera& camera, const math::Vec2& wp,
                                        int viewportWidth, int viewportHeight) {
    math::Vec4 clip = camera.viewProjectionMatrix() * math::Vec4(math::Vec3(wp.x, wp.y, 0.0), 1.0);
    if (std::abs(clip.w) < 1e-15) return {0.0, 0.0};

    math::Vec3 ndc = clip.perspectiveDivide();
    double sx = (ndc.x + 1.0) * 0.5 * viewportWidth;
    double sy = (1.0 - ndc.y) * 0.5 * viewportHeight;  // flip Y for Qt screen coords
    return {sx, sy};
}

void ViewportRenderer::renderTextToImage(QImage& image, const render::Camera& camera,
                                         doc::Document* doc,
                                         const render::SelectionManager& selection,
                                         int viewportWidth, int viewportHeight,
                                         double pixelToWorldScale) {
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

    // The font is made again only when the size or weight changes: most
    // texts share one.
    int fontSize = -1;
    bool fontBold = false;
    std::optional<QFontMetrics> metrics;
    auto drawItem = [&](const TextItem& item) {
        QPointF sp = worldToScreen(camera, item.worldPos, viewportWidth, viewportHeight);
        // Off the view by more than the text could reach, it is not drawn: a
        // character is under two font sizes wide, and each takes a byte or
        // more.
        const double reach =
            2.0 * item.fontSize * (static_cast<double>(item.text.size()) + 1.0) + 2.0;
        if (sp.x() < -reach || sp.y() < -reach || sp.x() > viewportWidth + reach ||
            sp.y() > viewportHeight + reach) {
            return;
        }
        QColor qc(static_cast<int>((item.color >> 16) & 0xFF),
                  static_cast<int>((item.color >> 8) & 0xFF), static_cast<int>(item.color & 0xFF));
        painter.setPen(qc);

        if (item.fontSize != fontSize || item.bold != fontBold || !metrics) {
            QFont font("Arial", item.fontSize);
            font.setBold(item.bold);
            painter.setFont(font);
            metrics.emplace(painter.font());
            fontSize = item.fontSize;
            fontBold = item.bold;
        }

        QString text = QString::fromUtf8(item.text.c_str());
        int tw = metrics->horizontalAdvance(text);
        int th = metrics->ascent();

        if (std::abs(item.rotation) > 1e-6) {
            painter.save();
            painter.translate(sp);
            // QPainter rotates clockwise in degrees; world rotation is CCW radians.
            painter.rotate(-item.rotation * 180.0 / 3.14159265358979323846);
            double dx = 0.0;
            if (item.alignment == 0)
                dx = 0.0;
            else if (item.alignment == 1)
                dx = -tw * 0.5;
            else
                dx = -tw;
            painter.drawText(QPointF(dx, th * 0.25), text);
            painter.restore();
        } else {
            double dx = 0.0;
            if (item.alignment == 0)
                dx = 0.0;
            else if (item.alignment == 1)
                dx = -tw * 0.5;
            else
                dx = -tw;
            painter.drawText(QPointF(sp.x() + dx, sp.y() + th * 0.25), text);
        }
    };

    // --- Dimension + text entity text ---
    if (!m_drawing.texts().empty() && doc) {
        const auto& style = doc->activeDrawing().dimensionStyle();
        double pxPerWorld = 1.0 / pixelToWorldScale;
        int defaultFontSize =
            std::max(8, std::min(48, static_cast<int>(style.textHeight * pxPerWorld * 0.4)));

        for (const auto& dt : m_drawing.texts()) {
            int fs = defaultFontSize;
            if (dt.height > 0.0) {
                fs = std::max(8, std::min(200, static_cast<int>(dt.height * pxPerWorld * 0.4)));
            }
            drawItem({dt.position, dt.text, dt.color, fs, false, dt.rotation, dt.alignment});
        }
    }

    // --- Constraint annotation indicators ---
    if (doc) {
        const auto& csys = doc->activeConstraints();
        if (!csys.empty()) {
            // Yellow annotation color: QColor(255, 200, 0) = 0xFFFFC800
            constexpr uint32_t kAnnotationColor = 0xFFFFC800;

            // Collect selected entity IDs to skip their constraint annotations.
            auto selectedIds = selection.selectedIds();
            std::set<uint64_t> selectedSet(selectedIds.begin(), selectedIds.end());

            for (const auto& c : csys.constraints()) {
                // Skip annotations for constraints whose entities are selected.
                auto refIds = c->referencedEntityIds();
                bool anySelected = false;
                for (uint64_t eid : refIds) {
                    if (selectedSet.count(eid)) {
                        anySelected = true;
                        break;
                    }
                }
                if (anySelected) continue;

                math::Vec2 pos{0.0, 0.0};
                std::string symbol;
                uint32_t color = kAnnotationColor;

                switch (c->type()) {
                    case cstr::ConstraintType::Coincident:
                        symbol = "\xE2\x97\x8F";
                        break;
                    case cstr::ConstraintType::Horizontal:
                        symbol = "H";
                        break;
                    case cstr::ConstraintType::Vertical:
                        symbol = "V";
                        break;
                    case cstr::ConstraintType::Perpendicular:
                        symbol = "\xE2\x9F\x82";
                        break;
                    case cstr::ConstraintType::Parallel:
                        symbol = "//";
                        break;
                    case cstr::ConstraintType::Tangent:
                        symbol = "T";
                        break;
                    case cstr::ConstraintType::Equal:
                        symbol = "=";
                        break;
                    case cstr::ConstraintType::Fixed:
                        symbol = "F";
                        break;
                    case cstr::ConstraintType::Distance: {
                        auto* dc = dynamic_cast<const cstr::DistanceConstraint*>(c.get());
                        if (dc) {
                            char buf[32];
                            snprintf(buf, sizeof(buf), "%.2f", dc->dimensionalValue());
                            symbol = buf;
                        }
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
                        break;
                    }
                }

                // Compute indicator position: midpoint of referenced features.
                if (!refIds.empty()) {
                    const auto* e1 = doc->activeDrawing().findEntity(refIds[0]);
                    if (e1) {
                        auto snaps = e1->snapPoints();
                        if (!snaps.empty()) pos = snaps[0];
                        if (refIds.size() > 1) {
                            const auto* e2 = doc->activeDrawing().findEntity(refIds.back());
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

                // Offset slightly above the position.
                pos.y += pixelToWorldScale * 12.0;

                // Arial 9pt annotations.
                drawItem({pos, symbol, color, 9, true});
            }
        }
    }

    // Orientation gizmo (top-right) and view-mode badge (top-left).
    m_viewCube.paint(painter, viewportWidth, viewportHeight, camera);

    {
        const QString label = ViewCube::orientationLabel(camera);
        QFont badgeFont("Arial", 9);
        badgeFont.setBold(true);
        painter.setFont(badgeFont);
        QFontMetrics fm(badgeFont);
        const int tw = fm.horizontalAdvance(label);
        QRectF badge(12.0, 12.0, tw + 20.0, 22.0);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setBrush(QColor(40, 44, 52, 210));
        painter.setPen(QPen(QColor(90, 98, 112), 1.0));
        painter.drawRoundedRect(badge, 4.0, 4.0);
        painter.setPen(QColor(210, 216, 224));
        painter.drawText(badge, Qt::AlignCenter, label);
    }

    painter.end();
}

bool ViewportRenderer::prepareTextOverlay(const render::Camera& camera, doc::Document* doc,
                                          const render::SelectionManager& selection,
                                          int viewportWidth, int viewportHeight,
                                          double pixelToWorldScale, qreal devicePixelRatio) {
    if (viewportWidth <= 0 || viewportHeight <= 0) return false;
    const qreal dpr = devicePixelRatio > 0.0 ? devicePixelRatio : 1.0;

    // What it shows: the texts and annotations (the drawing cache is built
    // again when they change, the selection's included), where the view
    // puts them, and the view cube, which turns with the view.
    OverlayStamp stamp;
    const math::Mat4 viewProjection = camera.viewProjectionMatrix();
    std::copy(viewProjection.data(), viewProjection.data() + 16, stamp.viewProjection.begin());
    stamp.width = viewportWidth;
    stamp.height = viewportHeight;
    stamp.ratio = dpr;
    stamp.pixelToWorld = pixelToWorldScale;
    stamp.document = doc;
    stamp.drawing = m_drawing.builds();
    if (m_overlayPainted && stamp == m_overlayStamp) return false;
    m_overlayStamp = stamp;
    m_overlayPainted = true;
    ++m_overlayPaints;
    m_overlayUploaded = false;

    // QPainter on a QImage is pure CPU, at device pixels: the painter keeps
    // working in logical ones. The image is made anew only for a new size.
    const QSize size(qRound(viewportWidth * dpr), qRound(viewportHeight * dpr));
    if (m_overlayImage.size() != size) {
        m_overlayImage = QImage(size, QImage::Format_RGBA8888_Premultiplied);
    }
    m_overlayImage.setDevicePixelRatio(dpr);
    m_overlayImage.fill(Qt::transparent);
    renderTextToImage(m_overlayImage, camera, doc, selection, viewportWidth, viewportHeight,
                      pixelToWorldScale);
    return true;
}

void ViewportRenderer::blitTextOverlay(QOpenGLExtraFunctions* gl, const render::Camera& camera,
                                       doc::Document* doc,
                                       const render::SelectionManager& selection, int viewportWidth,
                                       int viewportHeight, double pixelToWorldScale,
                                       qreal devicePixelRatio) {
    if (viewportWidth <= 0 || viewportHeight <= 0) return;
    prepareTextOverlay(camera, doc, selection, viewportWidth, viewportHeight, pixelToWorldScale,
                       devicePixelRatio);
    if (m_overlayImage.isNull()) return;

    // The texture follows the image only when it was painted again.
    gl->glBindTexture(GL_TEXTURE_2D, m_textOverlayTex);
    if (!m_overlayUploaded) {
        if (m_overlayTextureSize == m_overlayImage.size()) {
            gl->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_overlayImage.width(),
                                m_overlayImage.height(), GL_RGBA, GL_UNSIGNED_BYTE,
                                m_overlayImage.constBits());
        } else {
            gl->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_overlayImage.width(),
                             m_overlayImage.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                             m_overlayImage.constBits());
            m_overlayTextureSize = m_overlayImage.size();
        }
        m_overlayUploaded = true;
    }

    // A fullscreen quad with alpha blending: premultiplied (GL_ONE), as the
    // image is.
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
