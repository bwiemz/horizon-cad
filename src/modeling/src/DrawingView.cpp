#include "horizon/modeling/DrawingView.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "horizon/topology/Solid.h"

namespace hz::model {

namespace {

/// Recompute a view's 2D bounding box from its edges (empty → a zero box).
void computeBounds(DrawingView& dv) {
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
}

}  // namespace

DrawingView DrawingGenerator::makeView(const topo::Solid& solid, const ViewProjection& projection) {
    DrawingView dv;
    dv.projection = projection;
    dv.edges = DrawingProjection::project(solid, projection);
    computeBounds(dv);
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

DrawingView DrawingGenerator::detailView(const DrawingView& source, const math::Vec2& center,
                                         double radius, double scale) {
    DrawingView dv;
    dv.kind = source.kind;
    dv.projection = source.projection;

    const double r2 = radius * radius;
    auto toDetail = [&](const math::Vec2& p) {
        return math::Vec2(center.x + (p.x - center.x) * scale, center.y + (p.y - center.y) * scale);
    };

    for (const ProjectedEdge& e : source.edges) {
        // Clip segment a->b to the circle (center, radius): solve
        // |a + t*(b-a) - center|^2 = r^2 for the t-interval inside the circle,
        // then intersect it with [0, 1].
        const math::Vec2 d(e.b.x - e.a.x, e.b.y - e.a.y);
        const math::Vec2 f(e.a.x - center.x, e.a.y - center.y);
        const double A = d.x * d.x + d.y * d.y;
        const double B = 2.0 * (f.x * d.x + f.y * d.y);
        const double C = f.x * f.x + f.y * f.y - r2;

        double tLo = 0.0;
        double tHi = 1.0;
        if (A < 1e-18) {
            // Degenerate (zero-length) segment: keep only if the point is inside.
            if (C > 0.0) continue;
        } else {
            const double disc = B * B - 4.0 * A * C;
            if (disc < 0.0) continue;  // the whole line misses the circle
            const double s = std::sqrt(disc);
            const double t1 = (-B - s) / (2.0 * A);
            const double t2 = (-B + s) / (2.0 * A);
            tLo = std::max(0.0, t1);
            tHi = std::min(1.0, t2);
            if (tLo >= tHi) continue;  // no portion of the segment is inside
        }

        const math::Vec2 clippedA(e.a.x + d.x * tLo, e.a.y + d.y * tLo);
        const math::Vec2 clippedB(e.a.x + d.x * tHi, e.a.y + d.y * tHi);

        ProjectedEdge de;
        de.a = toDetail(clippedA);
        de.b = toDetail(clippedB);
        de.sourceEdge = e.sourceEdge;
        de.visibility = e.visibility;
        dv.edges.push_back(de);
    }

    computeBounds(dv);
    return dv;
}

}  // namespace hz::model
