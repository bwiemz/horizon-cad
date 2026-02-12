#include "horizon/ui/SplineTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftSpline.h"

#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

void SplineTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_controlPoints.clear();
    m_active = false;
}

void SplineTool::deactivate() {
    m_controlPoints.clear();
    m_active = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
    Tool::deactivate();
}

bool SplineTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Double-click finishes the spline.
    if (event->type() == QEvent::MouseButtonDblClick) {
        if (m_controlPoints.size() > 1) {
            m_controlPoints.pop_back();
        }
        finishSpline();
        return true;
    }

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    m_controlPoints.push_back(snappedPos);
    m_currentPos = snappedPos;
    m_active = true;
    return true;
}

bool SplineTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_active) return false;

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    m_currentPos = snappedPos;
    return true;
}

bool SplineTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool SplineTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        finishSpline();
        return true;
    }
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void SplineTool::cancel() {
    m_controlPoints.clear();
    m_active = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

void SplineTool::finishSpline() {
    if (m_controlPoints.size() >= 4 && m_viewport && m_viewport->document()) {
        auto spline = std::make_shared<draft::DraftSpline>(m_controlPoints);
        spline->setLayer(m_viewport->document()->layerManager().currentLayer());
        auto cmd = std::make_unique<doc::AddEntityCommand>(
            m_viewport->document()->draftDocument(), spline);
        m_viewport->document()->undoStack().push(std::move(cmd));
    }
    m_controlPoints.clear();
    m_active = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

// ---------------------------------------------------------------------------
// Preview
// ---------------------------------------------------------------------------

/// Inline B-spline evaluation for preview (mirrors DraftSpline::evaluate logic).
static math::Vec2 bsplinePt(const math::Vec2& p0, const math::Vec2& p1,
                              const math::Vec2& p2, const math::Vec2& p3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double omt = 1.0 - t;
    double b0 = omt * omt * omt;
    double b1 = 3.0 * t3 - 6.0 * t2 + 4.0;
    double b2 = -3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0;
    double b3 = t3;
    double inv6 = 1.0 / 6.0;
    return {inv6 * (b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x),
            inv6 * (b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y)};
}

std::vector<math::Vec2> SplineTool::evaluatePreview(const std::vector<math::Vec2>& cps) {
    if (cps.size() < 4) return cps;
    const int sps = 16;
    size_t spans = cps.size() - 3;
    std::vector<math::Vec2> pts;
    pts.reserve(spans * sps + 1);
    for (size_t span = 0; span < spans; ++span) {
        int count = (span + 1 < spans) ? sps : sps + 1;
        for (int j = 0; j < count; ++j) {
            double t = static_cast<double>(j) / sps;
            pts.push_back(bsplinePt(cps[span], cps[span + 1], cps[span + 2], cps[span + 3], t));
        }
    }
    return pts;
}

std::vector<std::pair<math::Vec2, math::Vec2>> SplineTool::getPreviewLines() const {
    if (!m_active || m_controlPoints.empty()) return {};

    // Build a temporary control point list including the cursor.
    std::vector<math::Vec2> cps = m_controlPoints;
    cps.push_back(m_currentPos);

    auto pts = evaluatePreview(cps);

    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        lines.push_back({pts[i], pts[i + 1]});
    }
    return lines;
}

std::string SplineTool::promptText() const {
    return "Click to add control points, Enter to finish";
}

bool SplineTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
