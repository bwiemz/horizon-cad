#include "horizon/modeling/DrawingView.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include "horizon/modeling/MateGeometry.h"
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

/// The part of the segment @p a - @p b inside the circle (@p centre, radius
/// squared @p r2), in @p outA - @p outB. False when none of it is.
bool clipToCircle(const math::Vec2& a, const math::Vec2& b, const math::Vec2& centre, double r2,
                  math::Vec2& outA, math::Vec2& outB) {
    // Solve |a + t*(b-a) - centre|^2 = r^2 for the t-interval inside the
    // circle, then intersect it with [0, 1].
    const math::Vec2 d(b.x - a.x, b.y - a.y);
    const math::Vec2 f(a.x - centre.x, a.y - centre.y);
    const double A = d.x * d.x + d.y * d.y;
    const double B = 2.0 * (f.x * d.x + f.y * d.y);
    const double C = f.x * f.x + f.y * f.y - r2;
    double tLo = 0.0;
    double tHi = 1.0;
    if (A < 1e-18) {
        // Degenerate (zero-length) segment: keep only if the point is inside.
        if (C > 0.0) return false;
    } else {
        const double disc = B * B - 4.0 * A * C;
        if (disc < 0.0) return false;  // the whole line misses the circle
        const double s = std::sqrt(disc);
        tLo = std::max(0.0, (-B - s) / (2.0 * A));
        tHi = std::min(1.0, (-B + s) / (2.0 * A));
        if (tLo >= tHi) return false;  // no portion of the segment is inside
    }
    outA = {a.x + d.x * tLo, a.y + d.y * tLo};
    outB = {a.x + d.x * tHi, a.y + d.y * tHi};
    return true;
}

/// A cylinder the part has, gathered from the facets that stand in for it.
struct Cylinder {
    math::Vec3 origin;  ///< on its axis
    math::Vec3 axis;    ///< unit
    double radius = 0.0;
    std::vector<math::Vec3> points;  ///< its facets' corners
};

/// A cylinder's axis and centre come from fitting its facets, so they carry
/// its noise: a hole's axis came back 1e-6 off square, its centre 5e-6 off.
constexpr double kAxisAngle = 1e-4;  ///< radians, for end-on and side-on
constexpr double kFitTolerance = 1e-5;

bool sameAxis(const Cylinder& c, const math::Vec3& origin, const math::Vec3& axis, double tol) {
    if (c.axis.cross(axis).length() > kFitTolerance) return false;
    const math::Vec3 off = origin - c.origin;
    return (off - c.axis * off.dot(c.axis)).length() <= tol;
}

/// The centre lines of the holes and bosses of @p solid seen through
/// @p projection: those that go at least three quarters of the way round
/// (so not a fillet), a cross where seen end-on, an axis where seen side-on.
/// Coaxial ones (a counterbore) share one, the largest.
std::vector<std::pair<math::Vec2, math::Vec2>> centreLinesOf(const topo::Solid& solid,
                                                             const ViewProjection& projection) {
    std::vector<Cylinder> cylinders;
    for (const topo::Face& face : solid.faces()) {
        if (!face.analyticSurface) continue;
        const auto frame = MateGeometry::frameForFace(face);
        if (!frame || frame->kind != MateFrameKind::Cylindrical || frame->radius <= 0.0) continue;
        const double tol = kFitTolerance * std::max(1.0, frame->radius);
        Cylinder* into = nullptr;
        for (Cylinder& c : cylinders) {
            if (std::abs(c.radius - frame->radius) <= tol &&
                sameAxis(c, frame->origin, frame->direction, tol)) {
                into = &c;
                break;
            }
        }
        if (into == nullptr) {
            cylinders.push_back({frame->origin, frame->direction.normalized(), frame->radius, {}});
            into = &cylinders.back();
        }
        const topo::HalfEdge* start =
            face.outerLoop != nullptr ? face.outerLoop->halfEdge : nullptr;
        for (const topo::HalfEdge* he = start; he != nullptr;) {
            if (he->origin != nullptr) into->points.push_back(he->origin->point);
            he = he->next;
            if (he == start) break;
        }
    }

    // Kept: those most of the way round, each with its extent along its axis.
    struct Marked {
        math::Vec3 origin;
        math::Vec3 axis;
        double radius;
        double low;
        double high;
    };
    std::vector<Marked> marked;
    for (const Cylinder& c : cylinders) {
        if (c.points.size() < 3) continue;
        const math::Vec3 ref = std::abs(c.axis.x) < 0.9 ? math::Vec3(1, 0, 0) : math::Vec3(0, 1, 0);
        const math::Vec3 u = c.axis.cross(ref).normalized();
        const math::Vec3 w = c.axis.cross(u);
        std::vector<double> angles;
        double low = std::numeric_limits<double>::infinity();
        double high = -std::numeric_limits<double>::infinity();
        for (const math::Vec3& p : c.points) {
            const math::Vec3 off = p - c.origin;
            angles.push_back(std::atan2(off.dot(w), off.dot(u)));
            low = std::min(low, off.dot(c.axis));
            high = std::max(high, off.dot(c.axis));
        }
        std::sort(angles.begin(), angles.end());
        double widestGap = angles.front() + 2.0 * std::numbers::pi - angles.back();
        for (std::size_t i = 1; i < angles.size(); ++i) {
            widestGap = std::max(widestGap, angles[i] - angles[i - 1]);
        }
        if (2.0 * std::numbers::pi - widestGap < 1.5 * std::numbers::pi - 1e-9) continue;
        bool merged = false;
        for (Marked& m : marked) {
            Cylinder axisOf{m.origin, m.axis, m.radius, {}};
            if (!sameAxis(axisOf, c.origin, c.axis, kFitTolerance * std::max(1.0, c.radius))) {
                continue;
            }
            // Along the same axis: its extent measured from the one kept.
            const double shift = (c.origin - m.origin).dot(m.axis);
            const double sign = m.axis.dot(c.axis) < 0.0 ? -1.0 : 1.0;
            const double a = shift + sign * low;
            const double b = shift + sign * high;
            m.low = std::min(m.low, std::min(a, b));
            m.high = std::max(m.high, std::max(a, b));
            m.radius = std::max(m.radius, c.radius);
            merged = true;
            break;
        }
        if (!merged) marked.push_back({c.origin, c.axis, c.radius, low, high});
    }

    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    const math::Vec3 dir = projection.dir.normalized();
    for (const Marked& m : marked) {
        const double along = std::abs(m.axis.dot(dir));
        if (along > std::cos(kAxisAngle)) {
            // End-on: a cross to the outline.
            const math::Vec2 c = DrawingProjection::toView(projection, m.origin);
            lines.push_back({{c.x - m.radius, c.y}, {c.x + m.radius, c.y}});
            lines.push_back({{c.x, c.y - m.radius}, {c.x, c.y + m.radius}});
        } else if (along < std::sin(kAxisAngle)) {
            // Side-on: its axis, end to end.
            lines.push_back({DrawingProjection::toView(projection, m.origin + m.axis * m.low),
                             DrawingProjection::toView(projection, m.origin + m.axis * m.high)});
        }
    }
    return lines;
}

}  // namespace

DrawingView DrawingGenerator::makeView(const topo::Solid& solid, const ViewProjection& projection) {
    DrawingView dv;
    dv.projection = projection;
    dv.edges = DrawingProjection::project(solid, projection);
    dv.centreLines = centreLinesOf(solid, projection);
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

std::optional<DrawingBalloon> DrawingGenerator::balloonFor(const DrawingView& view,
                                                           const std::string& namePrefix,
                                                           int item) {
    // The balloon's leader goes to the middle of the first piece of its edge
    // the view has (BalloonRenderer): so the longest such first piece.
    std::vector<std::string> seen;
    const ProjectedEdge* best = nullptr;
    double longest = 0.0;
    for (const ProjectedEdge& e : view.edges) {
        const std::string& tag = e.sourceEdge.tag();
        if (tag.compare(0, namePrefix.size(), namePrefix) != 0) continue;
        if (std::find(seen.begin(), seen.end(), tag) != seen.end()) continue;
        seen.push_back(tag);
        if (e.visibility != ProjectedEdge::Visibility::Visible) continue;
        const double length = (e.b - e.a).length();
        if (length > longest) {
            longest = length;
            best = &e;
        }
    }
    if (best == nullptr) return std::nullopt;

    DrawingBalloon balloon;
    balloon.item = item;
    balloon.feature = best->sourceEdge;
    const math::Vec2 tip = view.toSheet((best->a + best->b) * 0.5);
    const math::Vec2 low = view.placement;
    const math::Vec2 high{low.x + view.sheetWidth(), low.y + view.sheetHeight()};
    // Straight out from the side of the view nearest the tip, and clear of
    // it: the leader short, and the circle off the view whatever the angle.
    const std::array<std::pair<double, math::Vec2>, 4> sides{{
        {tip.x - low.x, {-1.0, 0.0}},
        {high.x - tip.x, {1.0, 0.0}},
        {tip.y - low.y, {0.0, -1.0}},
        {high.y - tip.y, {0.0, 1.0}},
    }};
    const auto nearest = std::min_element(
        sides.begin(), sides.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    constexpr double kClearance = 8.0;
    balloon.offset =
        nearest->second * (std::max(nearest->first, 0.0) + kClearance + balloon.radius);
    return balloon;
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
        math::Vec2 clippedA;
        math::Vec2 clippedB;
        if (!clipToCircle(e.a, e.b, center, r2, clippedA, clippedB)) continue;

        ProjectedEdge de;
        de.a = toDetail(clippedA);
        de.b = toDetail(clippedB);
        de.sourceEdge = e.sourceEdge;
        de.visibility = e.visibility;
        de.kind = e.kind;  // a silhouette is one in a detail too
        dv.edges.push_back(de);
    }
    for (const auto& [a, b] : source.centreLines) {
        math::Vec2 clippedA;
        math::Vec2 clippedB;
        if (clipToCircle(a, b, center, r2, clippedA, clippedB)) {
            dv.centreLines.emplace_back(toDetail(clippedA), toDetail(clippedB));
        }
    }

    computeBounds(dv);
    return dv;
}

}  // namespace hz::model
