#include "horizon/modeling/DrawingView.h"

#include <algorithm>
#include <limits>

#include "horizon/topology/Solid.h"

namespace hz::model {

DrawingView DrawingGenerator::makeView(const topo::Solid& solid, const ViewProjection& projection) {
    DrawingView dv;
    dv.projection = projection;
    dv.edges = DrawingProjection::project(solid, projection);

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

DrawingView DrawingGenerator::makeView(const topo::Solid& solid, StandardView view) {
    DrawingView dv = makeView(solid, DrawingProjection::standardView(view));
    dv.kind = view;
    return dv;
}

DrawingView DrawingGenerator::auxiliaryView(const topo::Solid& solid, const math::Vec3& faceNormal,
                                            const math::Vec3& up) {
    // Look opposite the outward normal so the face faces the viewer.
    ViewProjection view;
    view.origin = math::Vec3(0.0, 0.0, 0.0);
    view.dir = -faceNormal;
    view.up = up;
    return makeView(solid, view);
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
