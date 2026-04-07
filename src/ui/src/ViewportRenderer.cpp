#include "horizon/ui/ViewportRenderer.h"
#include "horizon/ui/Tool.h"
#include "horizon/ui/SelectTool.h"
#include "horizon/ui/GripManager.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Camera.h"
#include "horizon/render/SelectionManager.h"
#include "horizon/document/Document.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/SketchSolver.h"
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
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec3.h"
#include "horizon/math/Vec4.h"
#include "horizon/math/Mat4.h"

#include <QOpenGLExtraFunctions>
#include <QPainter>
#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QPointF>

#include <cmath>
#include <set>

namespace {

static hz::math::Vec3 argbToVec3(uint32_t argb) {
    return { static_cast<double>((argb >> 16) & 0xFF) / 255.0,
             static_cast<double>((argb >> 8)  & 0xFF) / 255.0,
             static_cast<double>( argb        & 0xFF) / 255.0 };
}

struct BatchKey {
    uint32_t colorARGB;
    float lineWidth;
    int lineType;
    bool operator==(const BatchKey& o) const {
        return colorARGB == o.colorARGB && lineWidth == o.lineWidth &&
               lineType == o.lineType;
    }
};

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

void ViewportRenderer::destroyGL(QOpenGLExtraFunctions* gl) {
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
    if (!doc) {
        m_dofAnalysis = {};
        m_dofDirty = false;
        return;
    }

    const auto& csys = doc->constraintSystem();
    if (csys.empty()) {
        m_dofAnalysis = {};
        m_dofDirty = false;
        return;
    }

    auto params = cstr::ParameterTable::buildFromEntities(
        doc->draftDocument().entities(), csys);
    cstr::SketchSolver solver;
    m_dofAnalysis = solver.analyzeDOF(params, csys);
    m_dofDirty = false;
}

// ---------------------------------------------------------------------------
// Entity rendering
// ---------------------------------------------------------------------------

void ViewportRenderer::renderEntities(QOpenGLExtraFunctions* gl,
                                       render::GLRenderer& renderer,
                                       const render::Camera& camera,
                                       doc::Document& doc,
                                       const render::SelectionManager& selection) {
    m_dimTexts.clear();

    const auto& entities = doc.draftDocument().entities();
    if (entities.empty()) return;

    const auto& layerMgr = doc.layerManager();

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

        bool selected = selection.isSelected(entity->id());

        // Resolve color.
        uint32_t resolvedColor;
        if (selected) {
            resolvedColor = 0xFFFF9900;  // orange
        } else if (entity->color() == 0x00000000) {
            resolvedColor = lp ? lp->color : 0xFFFFFFFF;
        } else {
            resolvedColor = entity->color();
        }

        // Override color for DOF visualization (skip for selected entities).
        if (!selected) {
            auto dofIt = m_dofAnalysis.entityStatus.find(entity->id());
            if (dofIt != m_dofAnalysis.entityStatus.end()) {
                switch (dofIt->second) {
                    case cstr::EntityDOFStatus::Free:
                        resolvedColor = 0xFF00CC00;  // Green for under-constrained.
                        break;
                    case cstr::EntityDOFStatus::FullyConstrained:
                        // Keep normal color.
                        break;
                    case cstr::EntityDOFStatus::OverConstrained:
                        resolvedColor = 0xFFFF0000;  // Red for over-constrained.
                        break;
                }
            }
        }

        // Resolve lineWidth.
        float resolvedWidth;
        if (entity->lineWidth() == 0.0) {
            resolvedWidth = lp ? static_cast<float>(lp->lineWidth) : 1.0f;
        } else {
            resolvedWidth = static_cast<float>(entity->lineWidth());
        }

        // Resolve lineType.
        int resolvedLineType;
        if (entity->lineType() == 0) {
            resolvedLineType = lp ? lp->lineType : 1;
        } else {
            resolvedLineType = entity->lineType();
        }

        BatchKey key{resolvedColor, resolvedWidth, resolvedLineType};

        // Helper: emit a vertex (x, y, z=0, distance) into a batch.
        auto emitVert = [](std::vector<float>& v, double x, double y, float dist) {
            v.push_back(static_cast<float>(x));
            v.push_back(static_cast<float>(y));
            v.push_back(0.0f);
            v.push_back(dist);
        };

        // Helper: emit a line segment with per-vertex distance for a point sequence.
        auto emitPointSeq = [&emitVert](std::vector<float>& v,
                                         const std::vector<math::Vec2>& pts,
                                         bool closed) {
            double cumDist = 0.0;
            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                double segLen = pts[i].distanceTo(pts[i + 1]);
                emitVert(v, pts[i].x, pts[i].y, static_cast<float>(cumDist));
                cumDist += segLen;
                emitVert(v, pts[i + 1].x, pts[i + 1].y, static_cast<float>(cumDist));
            }
            if (closed && pts.size() >= 2) {
                double segLen = pts.back().distanceTo(pts[0]);
                emitVert(v, pts.back().x, pts.back().y, static_cast<float>(cumDist));
                cumDist += segLen;
                emitVert(v, pts[0].x, pts[0].y, static_cast<float>(cumDist));
            }
        };

        if (auto* line = dynamic_cast<const draft::DraftLine*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            double len = line->start().distanceTo(line->end());
            emitVert(verts, line->start().x, line->start().y, 0.0f);
            emitVert(verts, line->end().x, line->end().y, static_cast<float>(len));
        } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
            auto verts = circleVertices(circle->center(), circle->radius());
            renderer.drawCircle(gl, camera, verts,
                                argbToVec3(resolvedColor), resolvedWidth,
                                resolvedLineType);
        } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(entity.get())) {
            auto verts = arcVertices(arc->center(), arc->radius(),
                                     arc->startAngle(), arc->endAngle());
            renderer.drawLines(gl, camera, verts,
                               argbToVec3(resolvedColor), resolvedWidth,
                               resolvedLineType);
        } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(entity.get())) {
            auto c = rect->corners();
            auto& verts = findOrCreateBatch(key);
            double cumDist = 0.0;
            for (int i = 0; i < 4; ++i) {
                int j = (i + 1) % 4;
                double segLen = c[i].distanceTo(c[j]);
                emitVert(verts, c[i].x, c[i].y, static_cast<float>(cumDist));
                cumDist += segLen;
                emitVert(verts, c[j].x, c[j].y, static_cast<float>(cumDist));
            }
        } else if (auto* polyline = dynamic_cast<const draft::DraftPolyline*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            emitPointSeq(verts, polyline->points(), polyline->closed());
        } else if (auto* spline = dynamic_cast<const draft::DraftSpline*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            emitPointSeq(verts, spline->evaluate(), false);
        } else if (auto* hatch = dynamic_cast<const draft::DraftHatch*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            // Draw boundary outline with cumulative distance.
            const auto& bnd = hatch->boundary();
            double cumDist = 0.0;
            for (size_t i = 0; i < bnd.size(); ++i) {
                size_t j = (i + 1) % bnd.size();
                double segLen = bnd[i].distanceTo(bnd[j]);
                emitVert(verts, bnd[i].x, bnd[i].y, static_cast<float>(cumDist));
                cumDist += segLen;
                emitVert(verts, bnd[j].x, bnd[j].y, static_cast<float>(cumDist));
            }
            // Draw hatch fill lines — each starts at distance 0.
            auto hatchLines = hatch->generateHatchLines();
            for (const auto& [a, b] : hatchLines) {
                double segLen = a.distanceTo(b);
                emitVert(verts, a.x, a.y, 0.0f);
                emitVert(verts, b.x, b.y, static_cast<float>(segLen));
            }
        } else if (auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(entity.get())) {
            auto& verts = findOrCreateBatch(key);
            emitPointSeq(verts, ellipse->evaluate(), false);
        } else if (auto* dim = dynamic_cast<const draft::DraftDimension*>(entity.get())) {
            const auto& style = doc.draftDocument().dimensionStyle();
            auto& verts = findOrCreateBatch(key);

            auto addSegments = [&](const std::vector<std::pair<math::Vec2, math::Vec2>>& segs) {
                for (const auto& [a, b] : segs) {
                    double segLen = a.distanceTo(b);
                    emitVert(verts, a.x, a.y, 0.0f);
                    emitVert(verts, b.x, b.y, static_cast<float>(segLen));
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
                // ByBlock resolution: sub-entity value 0 → use block ref's resolved value.
                uint32_t subColor = subEnt->color();
                if (subColor == 0x00000000) subColor = resolvedColor;
                float subWidth = static_cast<float>(subEnt->lineWidth());
                if (subWidth == 0.0f) subWidth = resolvedWidth;
                int subLineType = subEnt->lineType();
                if (subLineType == 0) subLineType = resolvedLineType;
                BatchKey subKey{subColor, subWidth, subLineType};

                if (auto* ln = dynamic_cast<const draft::DraftLine*>(subEnt.get())) {
                    auto p1 = bref->transformPoint(ln->start());
                    auto p2 = bref->transformPoint(ln->end());
                    auto& v = findOrCreateBatch(subKey);
                    double len = p1.distanceTo(p2);
                    emitVert(v, p1.x, p1.y, 0.0f);
                    emitVert(v, p2.x, p2.y, static_cast<float>(len));
                } else if (auto* ci = dynamic_cast<const draft::DraftCircle*>(subEnt.get())) {
                    auto wc = bref->transformPoint(ci->center());
                    double wr = ci->radius() * std::abs(bref->uniformScale());
                    auto cv = circleVertices(wc, wr);
                    renderer.drawCircle(gl, camera, cv,
                                        argbToVec3(subColor), subWidth, subLineType);
                } else if (auto* ar = dynamic_cast<const draft::DraftArc*>(subEnt.get())) {
                    auto wc = bref->transformPoint(ar->center());
                    double wr = ar->radius() * std::abs(bref->uniformScale());
                    double sa = ar->startAngle() + bref->rotation();
                    double ea = ar->endAngle() + bref->rotation();
                    if (bref->uniformScale() < 0.0) {
                        double tmp = sa; sa = -ea; ea = -tmp;
                    }
                    auto av = arcVertices(wc, wr, sa, ea);
                    renderer.drawLines(gl, camera, av,
                                       argbToVec3(subColor), subWidth, subLineType);
                } else if (auto* re = dynamic_cast<const draft::DraftRectangle*>(subEnt.get())) {
                    auto c = re->corners();
                    auto& v = findOrCreateBatch(subKey);
                    double cumDist = 0.0;
                    for (int i = 0; i < 4; ++i) {
                        auto wp1 = bref->transformPoint(c[i]);
                        auto wp2 = bref->transformPoint(c[(i + 1) % 4]);
                        double segLen = wp1.distanceTo(wp2);
                        emitVert(v, wp1.x, wp1.y, static_cast<float>(cumDist));
                        cumDist += segLen;
                        emitVert(v, wp2.x, wp2.y, static_cast<float>(cumDist));
                    }
                } else if (auto* pl = dynamic_cast<const draft::DraftPolyline*>(subEnt.get())) {
                    auto& v = findOrCreateBatch(subKey);
                    const auto& pts = pl->points();
                    double cumDist = 0.0;
                    for (size_t i = 0; i + 1 < pts.size(); ++i) {
                        auto wp1 = bref->transformPoint(pts[i]);
                        auto wp2 = bref->transformPoint(pts[i + 1]);
                        double segLen = wp1.distanceTo(wp2);
                        emitVert(v, wp1.x, wp1.y, static_cast<float>(cumDist));
                        cumDist += segLen;
                        emitVert(v, wp2.x, wp2.y, static_cast<float>(cumDist));
                    }
                    if (pl->closed() && pts.size() >= 2) {
                        auto wp1 = bref->transformPoint(pts.back());
                        auto wp2 = bref->transformPoint(pts[0]);
                        double segLen = wp1.distanceTo(wp2);
                        emitVert(v, wp1.x, wp1.y, static_cast<float>(cumDist));
                        cumDist += segLen;
                        emitVert(v, wp2.x, wp2.y, static_cast<float>(cumDist));
                    }
                } else if (auto* sp = dynamic_cast<const draft::DraftSpline*>(subEnt.get())) {
                    auto& v = findOrCreateBatch(subKey);
                    auto evalPts = sp->evaluate();
                    double cumDist = 0.0;
                    for (size_t i = 0; i + 1 < evalPts.size(); ++i) {
                        auto wp1 = bref->transformPoint(evalPts[i]);
                        auto wp2 = bref->transformPoint(evalPts[i + 1]);
                        double segLen = wp1.distanceTo(wp2);
                        emitVert(v, wp1.x, wp1.y, static_cast<float>(cumDist));
                        cumDist += segLen;
                        emitVert(v, wp2.x, wp2.y, static_cast<float>(cumDist));
                    }
                } else if (auto* el = dynamic_cast<const draft::DraftEllipse*>(subEnt.get())) {
                    auto& v = findOrCreateBatch(subKey);
                    auto evalPts = el->evaluate();
                    double cumDist = 0.0;
                    for (size_t i = 0; i + 1 < evalPts.size(); ++i) {
                        auto wp1 = bref->transformPoint(evalPts[i]);
                        auto wp2 = bref->transformPoint(evalPts[i + 1]);
                        double segLen = wp1.distanceTo(wp2);
                        emitVert(v, wp1.x, wp1.y, static_cast<float>(cumDist));
                        cumDist += segLen;
                        emitVert(v, wp2.x, wp2.y, static_cast<float>(cumDist));
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
            renderer.drawLines(gl, camera, verts,
                               argbToVec3(key.colorARGB), key.lineWidth,
                               key.lineType);
        }
    }
}

// ---------------------------------------------------------------------------
// Tool preview rendering
// ---------------------------------------------------------------------------

void ViewportRenderer::renderToolPreview(QOpenGLExtraFunctions* gl,
                                          render::GLRenderer& renderer,
                                          const render::Camera& camera,
                                          Tool* activeTool) {
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
            auto verts = arcVertices(arc.center, arc.radius,
                                     arc.startAngle, arc.endAngle);
            renderer.drawLines(gl, camera, verts, previewCol);
        }
    }

    // Filled selection rectangle for SelectTool box selection.
    if (auto* selectTool = dynamic_cast<SelectTool*>(activeTool)) {
        if (selectTool->isDraggingBox()) {
            bool isWindow = selectTool->isWindowSelection();
            math::Vec4 fillColor = isWindow
                ? math::Vec4{0.2, 0.4, 0.8, 0.12}    // Blue tint
                : math::Vec4{0.2, 0.8, 0.4, 0.12};   // Green tint
            renderer.drawFilledQuad(gl, camera,
                                    selectTool->boxCorner1(),
                                    selectTool->boxCorner2(),
                                    fillColor);
        }
    }

    // Snap indicator is now rendered by OverlayRenderer in paintGL().
}

// ---------------------------------------------------------------------------
// Grip rendering
// ---------------------------------------------------------------------------

void ViewportRenderer::renderGrips(QOpenGLExtraFunctions* gl,
                                    render::GLRenderer& renderer,
                                    const render::Camera& camera,
                                    doc::Document& doc,
                                    const render::SelectionManager& selection,
                                    double pixelToWorldScale) {
    auto selectedIds = selection.selectedIds();
    if (selectedIds.empty()) return;

    // Grip square size in world units (6 pixels).
    double s = 6.0 * pixelToWorldScale;

    std::vector<float> verts;
    math::Vec3 green{0.0, 1.0, 0.3};

    const auto& draftDoc = doc.draftDocument();
    for (uint64_t id : selectedIds) {
        for (const auto& e : draftDoc.entities()) {
            if (e->id() != id) continue;
            auto grips = GripManager::gripPoints(*e);
            for (const auto& g : grips) {
                float gx = static_cast<float>(g.x);
                float gy = static_cast<float>(g.y);
                float hs = static_cast<float>(s * 0.5);
                float side = hs * 2.0f;
                // Draw a small square (4 line segments) with distance attribute.
                verts.push_back(gx - hs); verts.push_back(gy - hs); verts.push_back(0.0f); verts.push_back(0.0f);
                verts.push_back(gx + hs); verts.push_back(gy - hs); verts.push_back(0.0f); verts.push_back(side);

                verts.push_back(gx + hs); verts.push_back(gy - hs); verts.push_back(0.0f); verts.push_back(side);
                verts.push_back(gx + hs); verts.push_back(gy + hs); verts.push_back(0.0f); verts.push_back(side * 2.0f);

                verts.push_back(gx + hs); verts.push_back(gy + hs); verts.push_back(0.0f); verts.push_back(side * 2.0f);
                verts.push_back(gx - hs); verts.push_back(gy + hs); verts.push_back(0.0f); verts.push_back(side * 3.0f);

                verts.push_back(gx - hs); verts.push_back(gy + hs); verts.push_back(0.0f); verts.push_back(side * 3.0f);
                verts.push_back(gx - hs); verts.push_back(gy - hs); verts.push_back(0.0f); verts.push_back(side * 4.0f);
            }
            break;
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

        verts.push_back(x0); verts.push_back(y0); verts.push_back(0.0f); verts.push_back(d0);
        verts.push_back(x1); verts.push_back(y1); verts.push_back(0.0f); verts.push_back(d1);
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

        verts.push_back(x0); verts.push_back(y0); verts.push_back(0.0f); verts.push_back(d0);
        verts.push_back(x1); verts.push_back(y1); verts.push_back(0.0f); verts.push_back(d1);
    }
    return verts;
}

// ---------------------------------------------------------------------------
// Text overlay
// ---------------------------------------------------------------------------

QPointF ViewportRenderer::worldToScreen(const render::Camera& camera,
                                         const math::Vec2& wp,
                                         int viewportWidth, int viewportHeight) {
    math::Vec4 clip = camera.viewProjectionMatrix()
                      * math::Vec4(math::Vec3(wp.x, wp.y, 0.0), 1.0);
    if (std::abs(clip.w) < 1e-15) return {0.0, 0.0};

    math::Vec3 ndc = clip.perspectiveDivide();
    double sx = (ndc.x + 1.0) * 0.5 * viewportWidth;
    double sy = (1.0 - ndc.y) * 0.5 * viewportHeight;  // flip Y for Qt screen coords
    return {sx, sy};
}

void ViewportRenderer::renderTextToImage(QImage& image,
                                          const render::Camera& camera,
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

    auto drawItem = [&](const TextItem& item) {
        QPointF sp = worldToScreen(camera, item.worldPos, viewportWidth, viewportHeight);
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
    if (!m_dimTexts.empty() && doc) {
        const auto& style = doc->draftDocument().dimensionStyle();
        double pxPerWorld = 1.0 / pixelToWorldScale;
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

    // --- Constraint annotation indicators ---
    if (doc) {
        const auto& csys = doc->constraintSystem();
        if (!csys.empty()) {
            const auto& entities = doc->draftDocument().entities();
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
                    if (selectedSet.count(eid)) { anySelected = true; break; }
                }
                if (anySelected) continue;

                math::Vec2 pos{0.0, 0.0};
                std::string symbol;
                uint32_t color = kAnnotationColor;

                switch (c->type()) {
                    case cstr::ConstraintType::Coincident:  symbol = "\xE2\x97\x8F"; break;
                    case cstr::ConstraintType::Horizontal:  symbol = "H"; break;
                    case cstr::ConstraintType::Vertical:    symbol = "V"; break;
                    case cstr::ConstraintType::Perpendicular: symbol = "\xE2\x9F\x82"; break;
                    case cstr::ConstraintType::Parallel:    symbol = "//"; break;
                    case cstr::ConstraintType::Tangent:     symbol = "T"; break;
                    case cstr::ConstraintType::Equal:       symbol = "="; break;
                    case cstr::ConstraintType::Fixed:       symbol = "F"; break;
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
                try {
                    if (!refIds.empty()) {
                        const auto* e1 = cstr::findEntity(refIds[0], entities);
                        if (e1) {
                            auto snaps = e1->snapPoints();
                            if (!snaps.empty()) pos = snaps[0];
                            if (refIds.size() > 1) {
                                const auto* e2 = cstr::findEntity(refIds.back(), entities);
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
                pos.y += pixelToWorldScale * 12.0;

                // Arial 9pt annotations.
                drawItem({pos, symbol, color, 9, true});
            }
        }
    }

    painter.end();
}

void ViewportRenderer::blitTextOverlay(QOpenGLExtraFunctions* gl,
                                        const render::Camera& camera,
                                        doc::Document* doc,
                                        const render::SelectionManager& selection,
                                        int viewportWidth, int viewportHeight,
                                        double pixelToWorldScale) {
    if (viewportWidth <= 0 || viewportHeight <= 0) return;

    // 1. Render text to a QImage (QPainter on QImage is pure CPU).
    QImage image(viewportWidth, viewportHeight, QImage::Format_RGBA8888_Premultiplied);
    image.fill(Qt::transparent);
    renderTextToImage(image, camera, doc, selection,
                      viewportWidth, viewportHeight, pixelToWorldScale);

    // 2. Upload QImage pixels to the GL texture.
    gl->glBindTexture(GL_TEXTURE_2D, m_textOverlayTex);
    gl->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, viewportWidth, viewportHeight, 0,
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
