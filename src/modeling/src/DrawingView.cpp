#include "horizon/modeling/DrawingView.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>

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

const std::vector<double>& DrawingGenerator::standardScales() {
    static const std::vector<double> scales{10.0, 5.0, 2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01};
    return scales;
}

std::string DrawingGenerator::scaleName(double scale) {
    const auto whole = [](double v) {
        const double r = std::round(v);
        if (std::abs(v - r) < 1e-9) return std::to_string(static_cast<long long>(r));
        // Not a standard scale: as few digits as say it ("2.5", not "2.500000").
        std::array<char, 32> text{};
        std::snprintf(text.data(), text.size(), "%.4g", v);
        return std::string(text.data());
    };
    if (scale >= 1.0) return whole(scale) + ":1";
    return "1:" + whole(1.0 / scale);
}

Drawing DrawingGenerator::sheetLayout(const topo::Solid& solid, const Sheet& sheet,
                                      const TitleBlock& titleBlock, double gap, double* chosenScale,
                                      double fixedScale) {
    DrawingView front = makeView(solid, StandardView::Front);
    DrawingView top = makeView(solid, StandardView::Top);
    DrawingView right = makeView(solid, StandardView::Right);
    DrawingView iso = makeView(solid, StandardView::Isometric);

    // The space the views have: inside the margin, above the title block.
    const double x0 = sheet.margin + gap;
    const double y0 = sheet.margin + titleBlock.height + gap;
    const double spaceW = sheet.widthMm() - sheet.margin - gap - x0;
    const double spaceH = sheet.heightMm() - sheet.margin - gap - y0;

    // A two-by-two grid: Front and Top share a column (their x agrees), Front
    // and Right a row (their z agrees).
    const double col1 = std::max(front.width(), top.width());
    const double col2 = std::max(right.width(), iso.width());
    const double row1 = std::max(front.height(), right.height());
    const double row2 = std::max(top.height(), iso.height());
    double scale = standardScales().back();
    if (std::isfinite(fixedScale) && fixedScale > 0.0) {
        scale = fixedScale;
    } else {
        for (const double s : standardScales()) {
            if (s * (col1 + col2) + gap <= spaceW && s * (row1 + row2) + gap <= spaceH) {
                scale = s;
                break;
            }
        }
    }
    const double usedW = scale * (col1 + col2) + gap;
    const double usedH = scale * (row1 + row2) + gap;
    const double left = x0 + std::max(0.0, (spaceW - usedW) / 2.0);
    const double bottom = y0 + std::max(0.0, (spaceH - usedH) / 2.0);
    front.placement = {left, bottom};
    right.placement = {left + scale * col1 + gap, bottom};
    top.placement = {left, bottom + scale * row1 + gap};
    iso.placement = {left + scale * col1 + gap, bottom + scale * row1 + gap};

    Drawing d;
    d.views = {front, top, right, iso};
    for (DrawingView& v : d.views) {
        v.scale = scale;
        v.showTangentEdges = false;
    }
    if (chosenScale != nullptr) *chosenScale = scale;
    return d;
}

std::pair<math::Vec2, math::Vec2> DrawingView::sheetFootprint() const {
    const math::Vec2 low = placement;
    const math::Vec2 high{placement.x + sheetWidth(), placement.y + sheetHeight()};
    const double caption = label.empty() ? 0.0 : kCaptionRoom;
    return {{low.x, low.y - caption}, high};
}

math::Vec2 DrawingGenerator::freePlacement(const Drawing& drawing, const DrawingView& view,
                                           const Sheet& sheet, const TitleBlock& titleBlock,
                                           double gap, bool* fits) {
    // The footprint wanted, as an offset from the view's placement.
    const double caption = view.label.empty() ? 0.0 : DrawingView::kCaptionRoom;
    const double w = view.sheetWidth();
    const double h = view.sheetHeight() + caption;

    // What is taken: the views with their captions, and the title block.
    struct Box {
        double x0, y0, x1, y1;
    };
    std::vector<Box> taken;
    for (const DrawingView& v : drawing.views) {
        const auto [low, high] = v.sheetFootprint();
        taken.push_back({low.x, low.y, high.x, high.y});
    }
    const double right = sheet.widthMm() - sheet.margin;
    taken.push_back(
        {right - titleBlock.width, sheet.margin, right, sheet.margin + titleBlock.height});

    const double minX = sheet.margin + gap;
    const double minY = sheet.margin + gap;
    const double maxX = sheet.widthMm() - sheet.margin - gap;
    const double maxY = sheet.heightMm() - sheet.margin - gap;
    const auto clear = [&](double x, double y) {
        if (x < minX - 1e-9 || y < minY - 1e-9 || x + w > maxX + 1e-9 || y + h > maxY + 1e-9) {
            return false;
        }
        for (const Box& b : taken) {
            if (x < b.x1 + gap - 1e-9 && b.x0 - gap < x + w - 1e-9 && y < b.y1 + gap - 1e-9 &&
                b.y0 - gap < y + h - 1e-9) {
                return false;
            }
        }
        return true;
    };

    // Candidates: against the border, and beside each thing taken.
    std::vector<double> xs{minX, maxX - w};
    std::vector<double> ys{maxY - h, minY};
    for (const Box& b : taken) {
        xs.push_back(b.x1 + gap);
        xs.push_back(b.x0 - gap - w);
        ys.push_back(b.y1 + gap);
        ys.push_back(b.y0 - gap - h);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end(), std::greater<>());
    for (const double y : ys) {
        for (const double x : xs) {
            if (clear(x, y)) {
                if (fits != nullptr) *fits = true;
                return {x, y + caption};
            }
        }
    }
    if (fits != nullptr) *fits = false;
    return {sheet.widthMm() + gap, maxY - h + caption};  // beside the sheet
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
        de.kind = e.kind;  // a silhouette is one in a detail too
        dv.edges.push_back(de);
    }

    computeBounds(dv);
    return dv;
}

}  // namespace hz::model
