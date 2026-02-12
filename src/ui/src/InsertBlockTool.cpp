#include "horizon/ui/InsertBlockTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>

namespace hz::ui {

InsertBlockTool::InsertBlockTool(std::shared_ptr<draft::BlockDefinition> definition,
                                 double rotation, double scale)
    : m_definition(std::move(definition))
    , m_rotation(math::degToRad(rotation))
    , m_scale(scale) {}

void InsertBlockTool::deactivate() {
    cancel();
    Tool::deactivate();
}

bool InsertBlockTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Snap.
    math::Vec2 pos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        pos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    // Place a block reference.
    if (m_viewport && m_viewport->document()) {
        auto ref = std::make_shared<draft::DraftBlockRef>(m_definition, pos, m_rotation, m_scale);
        ref->setLayer(m_viewport->document()->layerManager().currentLayer());
        auto cmd = std::make_unique<doc::AddEntityCommand>(
            m_viewport->document()->draftDocument(), ref);
        m_viewport->document()->undoStack().push(std::move(cmd));
    }
    return true;
}

bool InsertBlockTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    math::Vec2 pos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        pos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    m_currentPos = pos;
    return true;
}

bool InsertBlockTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool InsertBlockTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void InsertBlockTool::cancel() {
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

// Helper to transform a point from definition space to preview world space.
static math::Vec2 xform(const math::Vec2& pt, const math::Vec2& basePoint,
                         const math::Vec2& insertPos, double rotation, double scale) {
    math::Vec2 local = (pt - basePoint) * scale;
    double c = std::cos(rotation), s = std::sin(rotation);
    return {insertPos.x + local.x * c - local.y * s,
            insertPos.y + local.x * s + local.y * c};
}

std::vector<std::pair<math::Vec2, math::Vec2>> InsertBlockTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    if (!m_definition) return lines;

    const auto& bp = m_definition->basePoint;
    for (const auto& ent : m_definition->entities) {
        if (auto* ln = dynamic_cast<const draft::DraftLine*>(ent.get())) {
            lines.emplace_back(xform(ln->start(), bp, m_currentPos, m_rotation, m_scale),
                               xform(ln->end(), bp, m_currentPos, m_rotation, m_scale));
        } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(ent.get())) {
            auto c = rect->corners();
            for (int i = 0; i < 4; ++i) {
                lines.emplace_back(xform(c[i], bp, m_currentPos, m_rotation, m_scale),
                                   xform(c[(i + 1) % 4], bp, m_currentPos, m_rotation, m_scale));
            }
        } else if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(ent.get())) {
            const auto& pts = poly->points();
            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                lines.emplace_back(xform(pts[i], bp, m_currentPos, m_rotation, m_scale),
                                   xform(pts[i + 1], bp, m_currentPos, m_rotation, m_scale));
            }
            if (poly->closed() && pts.size() >= 2) {
                lines.emplace_back(xform(pts.back(), bp, m_currentPos, m_rotation, m_scale),
                                   xform(pts[0], bp, m_currentPos, m_rotation, m_scale));
            }
        } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(ent.get())) {
            // Approximate arc as line segments for preview.
            auto wc = xform(arc->center(), bp, m_currentPos, m_rotation, m_scale);
            double wr = arc->radius() * std::abs(m_scale);
            double sa = arc->startAngle() + m_rotation;
            double ea = arc->endAngle() + m_rotation;
            double sweep = ea - sa;
            if (sweep <= 0.0) sweep += math::kTwoPi;
            int segs = std::max(4, static_cast<int>(32.0 * sweep / math::kTwoPi));
            double step = sweep / segs;
            for (int i = 0; i < segs; ++i) {
                double a0 = sa + step * i;
                double a1 = sa + step * (i + 1);
                lines.emplace_back(
                    math::Vec2(wc.x + wr * std::cos(a0), wc.y + wr * std::sin(a0)),
                    math::Vec2(wc.x + wr * std::cos(a1), wc.y + wr * std::sin(a1)));
            }
        }
    }
    return lines;
}

std::vector<std::pair<math::Vec2, double>> InsertBlockTool::getPreviewCircles() const {
    std::vector<std::pair<math::Vec2, double>> circles;
    if (!m_definition) return circles;

    const auto& bp = m_definition->basePoint;
    for (const auto& ent : m_definition->entities) {
        if (auto* ci = dynamic_cast<const draft::DraftCircle*>(ent.get())) {
            auto wc = xform(ci->center(), bp, m_currentPos, m_rotation, m_scale);
            circles.emplace_back(wc, ci->radius() * std::abs(m_scale));
        }
    }
    return circles;
}

std::string InsertBlockTool::promptText() const {
    return "Click to place block";
}

bool InsertBlockTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
