#include "horizon/modeling/DrawingView.h"

#include <algorithm>
#include <limits>

#include "horizon/topology/Solid.h"

namespace hz::model {

DrawingView DrawingGenerator::makeView(const topo::Solid& solid, StandardView view) {
    DrawingView dv;
    dv.kind = view;
    dv.edges = DrawingProjection::project(solid, DrawingProjection::standardView(view));

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    for (const ProjectedEdge& e : dv.edges) {
        for (const math::Vec2& p : {e.a, e.b}) {
            minX = std::min(minX, p.x);
            minY = std::min(minY, p.y);
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
        }
    }
    if (dv.edges.empty()) {
        dv.boundsMin = {0.0, 0.0};
        dv.boundsMax = {0.0, 0.0};
    } else {
        dv.boundsMin = {minX, minY};
        dv.boundsMax = {maxX, maxY};
    }
    return dv;
}

Drawing DrawingGenerator::standardViews(const topo::Solid& solid, double gap) {
    DrawingView front = makeView(solid, StandardView::Front);
    DrawingView top = makeView(solid, StandardView::Top);
    DrawingView right = makeView(solid, StandardView::Right);
    DrawingView iso = makeView(solid, StandardView::Isometric);

    // Lay out on a 2x2 grid anchored at the front view's lower-left, sized by the
    // front view so the columns/rows never overlap:
    //   top   | iso
    //   ------+------
    //   front | right
    const double colGap = front.width() + gap;
    const double rowGap = front.height() + gap;

    front.placement = {0.0, 0.0};
    right.placement = {colGap, 0.0};
    top.placement = {0.0, rowGap};
    iso.placement = {colGap, rowGap};

    Drawing d;
    d.views = {front, top, right, iso};
    return d;
}

}  // namespace hz::model
