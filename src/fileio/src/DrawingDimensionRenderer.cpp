#include "horizon/fileio/DrawingDimensionRenderer.h"

#include <cmath>

#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Naming.h"

namespace hz::io {

std::shared_ptr<draft::DraftLinearDimension> DrawingDimensionRenderer::render(
    const model::DrawingView& view, const model::LinearDimension& dim, double offset,
    const draft::DimensionStyle& style) {
    // The projected edge for this dimension's model edge in the view: all
    // of it, end to end. A partly hidden edge is drawn in several runs, and
    // the first alone put the dimension on part of the edge.
    const model::ProjectedEdge* first = nullptr;
    math::Vec2 lowEnd;
    math::Vec2 highEnd;
    double low = 0.0;
    double high = 0.0;
    math::Vec2 along;
    const std::string logical = model::logicalEdge(dim.edge.tag());
    for (const model::ProjectedEdge& e : view.edges) {
        if (e.sourceEdge != dim.edge && model::logicalEdge(e.sourceEdge.tag()) != logical) continue;
        if (first == nullptr) {
            first = &e;
            along = e.b - e.a;
            const double length = along.length();
            if (length < 1e-12) return nullptr;  // seen end-on: nothing to dimension
            along = along * (1.0 / length);
            lowEnd = e.a;
            highEnd = e.b;
            low = 0.0;
            high = length;
        }
        for (const math::Vec2& p : {e.a, e.b}) {
            const double t = (p - first->a).dot(along);
            if (t < low) {
                low = t;
                lowEnd = p;
            }
            if (t > high) {
                high = t;
                highEnd = p;
            }
        }
    }
    if (first == nullptr) return nullptr;

    // Map view-space coordinates onto the sheet (same mapping DrawingExport uses).
    const math::Vec2 p1 = view.toSheet(lowEnd);
    const math::Vec2 p2 = view.toSheet(highEnd);

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

    auto dimension = std::make_shared<draft::DraftLinearDimension>(
        p1, p2, dimLinePoint, draft::DraftLinearDimension::Orientation::Aligned);
    // On a view drawn at a scale, the sheet length is not the part's: state
    // the length at 1:1, as the view drawn full size would.
    if (std::abs(view.scale - 1.0) > 1e-12 && view.scale > 0.0) {
        dimension->setTextOverride(style.formatLength((p2 - p1).length() / view.scale));
    }
    return dimension;
}

}  // namespace hz::io
