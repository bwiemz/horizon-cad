#include "horizon/fileio/BalloonRenderer.h"

#include <cmath>
#include <string>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/modeling/DrawingBalloon.h"
#include "horizon/modeling/DrawingView.h"

namespace hz::io {

std::vector<std::shared_ptr<draft::DraftEntity>> BalloonRenderer::render(
    const model::DrawingView& view, const model::DrawingBalloon& balloon) {
    // Find the projected edge for this balloon's model feature in the view.
    const model::ProjectedEdge* edge = nullptr;
    for (const model::ProjectedEdge& e : view.edges) {
        if (e.sourceEdge == balloon.feature) {
            edge = &e;
            break;
        }
    }
    if (edge == nullptr) return {};

    // Map view-space coordinates onto the sheet (same mapping DrawingExport uses).
    auto toSheet = [&](const math::Vec2& p) {
        return math::Vec2((p.x - view.boundsMin.x) + view.placement.x,
                          (p.y - view.boundsMin.y) + view.placement.y);
    };
    const math::Vec2 a = toSheet(edge->a);
    const math::Vec2 b = toSheet(edge->b);
    const math::Vec2 tip((a.x + b.x) * 0.5,
                         (a.y + b.y) * 0.5);  // leader points at the edge midpoint

    const math::Vec2 circleCenter(tip.x + balloon.offset.x, tip.y + balloon.offset.y);

    // Leader runs from the edge midpoint (arrow tip) to the circle boundary.
    const double dx = circleCenter.x - tip.x;
    const double dy = circleCenter.y - tip.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    math::Vec2 leaderEnd = circleCenter;
    if (dist > balloon.radius && dist > 1e-12) {
        leaderEnd = math::Vec2(circleCenter.x - dx / dist * balloon.radius,
                               circleCenter.y - dy / dist * balloon.radius);
    }

    std::vector<std::shared_ptr<draft::DraftEntity>> out;
    out.push_back(std::make_shared<draft::DraftLeader>(std::vector<math::Vec2>{tip, leaderEnd},
                                                       std::string{}));
    out.push_back(std::make_shared<draft::DraftCircle>(circleCenter, balloon.radius));

    auto number = std::make_shared<draft::DraftText>(circleCenter, std::to_string(balloon.item),
                                                     balloon.radius);
    number->setAlignment(draft::TextAlignment::Center);
    out.push_back(std::move(number));

    return out;
}

}  // namespace hz::io
