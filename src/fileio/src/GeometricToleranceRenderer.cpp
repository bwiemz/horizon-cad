#include "horizon/fileio/GeometricToleranceRenderer.h"

#include <cmath>

#include "horizon/drafting/DraftText.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"

namespace hz::io {

namespace {

constexpr double kTextHeight = 2.5;

// Locate the projected edge in @p view whose source model edge is @p feature,
// and, if found, return the sheet-space point at which to place an annotation:
// the edge midpoint pushed out by @p offset along the edge's perpendicular.
// Returns false (leaving @p outPos untouched) if the feature is not in the view.
bool anchorPoint(const model::DrawingView& view, const topo::TopologyID& feature, double offset,
                 math::Vec2& outPos) {
    const model::ProjectedEdge* edge = nullptr;
    for (const model::ProjectedEdge& e : view.edges) {
        if (e.sourceEdge == feature) {
            edge = &e;
            break;
        }
    }
    if (edge == nullptr) return false;

    // Map view-space coordinates onto the sheet (same mapping DrawingExport uses).
    auto toSheet = [&](const math::Vec2& p) {
        return math::Vec2((p.x - view.boundsMin.x) + view.placement.x,
                          (p.y - view.boundsMin.y) + view.placement.y);
    };
    const math::Vec2 p1 = toSheet(edge->a);
    const math::Vec2 p2 = toSheet(edge->b);

    const math::Vec2 mid((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5);
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double len = std::sqrt(dx * dx + dy * dy);
    math::Vec2 perp(0.0, 1.0);
    if (len > 1e-12) {
        perp = math::Vec2(-dy / len, dx / len);
    }
    outPos = math::Vec2(mid.x + perp.x * offset, mid.y + perp.y * offset);
    return true;
}

}  // namespace

std::shared_ptr<draft::DraftText> GeometricToleranceRenderer::render(
    const model::DrawingView& view, const model::FeatureControlFrame& frame, double offset) {
    math::Vec2 pos;
    if (!anchorPoint(view, frame.feature, offset, pos)) return nullptr;
    return std::make_shared<draft::DraftText>(pos, model::Gdt::format(frame), kTextHeight);
}

std::shared_ptr<draft::DraftText> GeometricToleranceRenderer::render(
    const model::DrawingView& view, const model::DatumFeature& datum, double offset) {
    math::Vec2 pos;
    if (!anchorPoint(view, datum.feature, offset, pos)) return nullptr;
    return std::make_shared<draft::DraftText>(pos, model::Gdt::datumSymbol(datum), kTextHeight);
}

}  // namespace hz::io
