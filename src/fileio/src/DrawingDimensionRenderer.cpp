#include "horizon/fileio/DrawingDimensionRenderer.h"

#include <cmath>

#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingView.h"

namespace hz::io {

std::shared_ptr<draft::DraftLinearDimension> DrawingDimensionRenderer::render(
    const model::DrawingView& view, const model::LinearDimension& dim, double offset) {
    // Find the projected edge for this dimension's model edge in the view.
    const model::ProjectedEdge* edge = nullptr;
    for (const model::ProjectedEdge& e : view.edges) {
        if (e.sourceEdge == dim.edge) {
            edge = &e;
            break;
        }
    }
    if (edge == nullptr) return nullptr;

    // Map view-space coordinates onto the sheet (same mapping DrawingExport uses).
    auto toSheet = [&](const math::Vec2& p) {
        return math::Vec2((p.x - view.boundsMin.x) + view.placement.x,
                          (p.y - view.boundsMin.y) + view.placement.y);
    };
    const math::Vec2 p1 = toSheet(edge->a);
    const math::Vec2 p2 = toSheet(edge->b);

    // Place the dimension line offset perpendicular to the edge from its midpoint.
    const math::Vec2 mid((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5);
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double len = std::sqrt(dx * dx + dy * dy);
    math::Vec2 perp(0.0, 1.0);
    if (len > 1e-12) {
        perp = math::Vec2(-dy / len, dx / len);
    }
    const math::Vec2 dimLinePoint(mid.x + perp.x * offset, mid.y + perp.y * offset);

    return std::make_shared<draft::DraftLinearDimension>(
        p1, p2, dimLinePoint, draft::DraftLinearDimension::Orientation::Aligned);
}

}  // namespace hz::io
