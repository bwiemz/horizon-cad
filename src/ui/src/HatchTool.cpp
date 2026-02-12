#include "horizon/ui/HatchTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftCircle.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>

namespace hz::ui {

void HatchTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
}

void HatchTool::deactivate() {
    Tool::deactivate();
}

/// Try to extract a closed boundary polygon from the given entity.
/// Returns an empty vector if the entity is not a suitable boundary source.
static std::vector<math::Vec2> extractBoundary(const draft::DraftEntity* entity) {
    // Rectangle → 4 corners.
    if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(entity)) {
        auto c = rect->corners();
        return {c[0], c[1], c[2], c[3]};
    }

    // Closed polyline → its points.
    if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(entity)) {
        if (poly->closed() && poly->pointCount() >= 3) {
            return poly->points();
        }
        return {};
    }

    // Circle → approximate as 64-sided polygon.
    if (auto* circle = dynamic_cast<const draft::DraftCircle*>(entity)) {
        constexpr int N = 64;
        std::vector<math::Vec2> pts;
        pts.reserve(N);
        for (int i = 0; i < N; ++i) {
            double a = 2.0 * 3.14159265358979323846 * i / N;
            pts.push_back({circle->center().x + circle->radius() * std::cos(a),
                           circle->center().y + circle->radius() * std::sin(a)});
        }
        return pts;
    }

    return {};
}

bool HatchTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    const auto& layerMgr = m_viewport->document()->layerManager();

    // Hit-test to find a closed entity under the cursor.
    double pixelScale = m_viewport->pixelToWorldScale();
    const double tolerance = std::max(10.0 * pixelScale, 0.15);

    const draft::DraftEntity* hitEntity = nullptr;
    for (const auto& entity : doc.entities()) {
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        if (entity->hitTest(worldPos, tolerance)) {
            hitEntity = entity.get();
            break;
        }
    }

    if (!hitEntity) return true;  // Click on empty space — do nothing.

    // Try to extract a boundary from the hit entity.
    auto boundary = extractBoundary(hitEntity);
    if (boundary.empty()) return true;  // Not a valid boundary source.

    // Create the hatch entity on the current layer.
    auto hatch = std::make_shared<draft::DraftHatch>(boundary);
    hatch->setLayer(m_viewport->document()->layerManager().currentLayer());

    auto cmd = std::make_unique<doc::AddEntityCommand>(doc, hatch);
    m_viewport->document()->undoStack().push(std::move(cmd));
    m_viewport->update();
    return true;
}

bool HatchTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool HatchTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool HatchTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void HatchTool::cancel() {
    // Nothing to reset — each click is a complete action.
}

std::vector<std::pair<math::Vec2, math::Vec2>> HatchTool::getPreviewLines() const {
    return {};  // No preview for hatch tool.
}

std::string HatchTool::promptText() const {
    return "Click a closed entity to hatch";
}

bool HatchTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
