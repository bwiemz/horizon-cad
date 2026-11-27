#include "horizon/modeling/SheetMetalSolid.h"

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Extrude.h"

namespace hz::model {

using math::Vec2;

namespace {

Vec2 rotated(const Vec2& v, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

bool stripIsFoldable(const SheetMetalStrip& strip, const SheetMetalParams& params) {
    if (!params.isValid()) return false;
    if (strip.segments.empty()) return false;
    if (strip.bendAngles.size() + 1 != strip.segments.size()) return false;
    for (const double length : strip.segments) {
        if (length <= 0.0) return false;
    }
    bool hasBend = false;
    for (const double angle : strip.bendAngles) {
        if (std::abs(angle) >= math::kPi) return false;
        if (angle != 0.0) hasBend = true;
    }
    // A real fold needs a positive inside radius; a zero-radius bend would
    // collapse the arc polyline to coincident points (zero-length edges and
    // zero-normal faces). SheetMetalParams allows radius 0 for the flat-pattern
    // limit, so gate it here rather than in the shared params validator.
    if (hasBend && params.bendRadius <= 1e-9) return false;
    return true;
}

/// Walk one boundary of the folded strip: straight spans of the segment
/// lengths, joined by arc polylines. @p offset lifts the start off the
/// bottom surface (0 = bottom boundary, thickness = top boundary); the
/// bend radius per surface depends on the bend direction — the concave
/// (inside) surface uses params.bendRadius, the convex one adds thickness.
std::vector<Vec2> walkBoundary(const SheetMetalStrip& strip, const SheetMetalParams& params,
                               double offset, int segmentsPerBend) {
    std::vector<Vec2> points;
    Vec2 p(0.0, offset);
    Vec2 d(1.0, 0.0);
    points.push_back(p);

    for (size_t i = 0; i < strip.segments.size(); ++i) {
        p = p + d * strip.segments[i];
        points.push_back(p);

        if (i >= strip.bendAngles.size()) continue;
        const double angle = strip.bendAngles[i];
        if (angle == 0.0) continue;

        const double sign = angle > 0.0 ? 1.0 : -1.0;
        // Positive bends curl toward the top side, so their inside surface
        // is the top boundary (offset > 0) and the bottom is convex.
        const bool boundaryIsInside = (angle > 0.0) == (offset > 0.0);
        const double radius =
            boundaryIsInside ? params.bendRadius : params.bendRadius + params.thickness;

        const Vec2 left(-d.y, d.x);  // 90 deg counter-clockwise of travel
        const Vec2 center = p + left * (sign * radius);
        for (int k = 1; k <= segmentsPerBend; ++k) {
            const double swept = angle * static_cast<double>(k) / segmentsPerBend;
            points.push_back(center + rotated(p - center, swept));
        }
        p = points.back();
        d = rotated(d, angle);
    }
    return points;
}

/// One boundary of the folded strip as walkBoundary walks it, exactly: each straight span a line
/// from where it starts to where it ends, each bend an arc about its centre from where it starts
/// through its whole angle (counter-clockwise from its start for a positive bend, from its end
/// for a negative one), where walkBoundary cuts it into chords.
std::vector<std::shared_ptr<draft::DraftEntity>> walkExactly(const SheetMetalStrip& strip,
                                                             const SheetMetalParams& params,
                                                             double offset) {
    std::vector<std::shared_ptr<draft::DraftEntity>> pieces;
    Vec2 p(0.0, offset);
    Vec2 d(1.0, 0.0);
    for (size_t i = 0; i < strip.segments.size(); ++i) {
        const Vec2 q = p + d * strip.segments[i];
        pieces.push_back(std::make_shared<draft::DraftLine>(p, q));
        p = q;

        if (i >= strip.bendAngles.size()) continue;
        const double angle = strip.bendAngles[i];
        if (angle == 0.0) continue;

        // As walkBoundary bends: about the centre on the side the bend turns to, the inside
        // surface at the bend radius and the outside one a thickness further.
        const double sign = angle > 0.0 ? 1.0 : -1.0;
        const bool boundaryIsInside = (angle > 0.0) == (offset > 0.0);
        const double radius =
            boundaryIsInside ? params.bendRadius : params.bendRadius + params.thickness;
        const Vec2 left(-d.y, d.x);
        const Vec2 center = p + left * (sign * radius);
        const Vec2 end = center + rotated(p - center, angle);
        const Vec2 first = angle > 0.0 ? p : end;  // where it starts counter-clockwise
        const Vec2 last = angle > 0.0 ? end : p;
        pieces.push_back(std::make_shared<draft::DraftArc>(
            center, radius, std::atan2(first.y - center.y, first.x - center.x),
            std::atan2(last.y - center.y, last.x - center.x)));
        p = end;
        d = rotated(d, angle);
    }
    return pieces;
}

/// Where a boundary walkExactly walked ends: its last flat's end (every boundary ends in a flat).
Vec2 boundaryEnd(const std::vector<std::shared_ptr<draft::DraftEntity>>& boundary) {
    const auto* last = dynamic_cast<const draft::DraftLine*>(boundary.back().get());
    return last != nullptr ? last->end() : Vec2();
}

}  // namespace

std::vector<Vec2> SheetMetalSolid::crossSection(const SheetMetalStrip& strip,
                                                const SheetMetalParams& params,
                                                int segmentsPerBend) {
    if (!stripIsFoldable(strip, params) || segmentsPerBend < 1) return {};

    std::vector<Vec2> polygon = walkBoundary(strip, params, 0.0, segmentsPerBend);
    const std::vector<Vec2> top = walkBoundary(strip, params, params.thickness, segmentsPerBend);
    polygon.insert(polygon.end(), top.rbegin(), top.rend());
    return polygon;
}

std::vector<std::shared_ptr<draft::DraftEntity>> SheetMetalSolid::profile(
    const SheetMetalStrip& strip, const SheetMetalParams& params) {
    if (!stripIsFoldable(strip, params)) return {};
    std::vector<std::shared_ptr<draft::DraftEntity>> bottom = walkExactly(strip, params, 0.0);
    std::vector<std::shared_ptr<draft::DraftEntity>> top =
        walkExactly(strip, params, params.thickness);
    // The top walked back: its pieces in reverse, each line from its end to its start (an arc
    // has no way of its own to turn).
    std::vector<std::shared_ptr<draft::DraftEntity>> outline = std::move(bottom);
    const Vec2 farBottom = boundaryEnd(outline);
    const Vec2 farTop = boundaryEnd(top);
    outline.push_back(std::make_shared<draft::DraftLine>(farBottom, farTop));
    for (auto piece = top.rbegin(); piece != top.rend(); ++piece) {
        if (auto* line = dynamic_cast<draft::DraftLine*>(piece->get())) {
            const Vec2 from = line->start();
            line->setStart(line->end());
            line->setEnd(from);
        }
        outline.push_back(*piece);
    }
    outline.push_back(std::make_shared<draft::DraftLine>(Vec2(0.0, params.thickness), Vec2()));
    return outline;
}

std::unique_ptr<topo::Solid> SheetMetalSolid::fold(const SheetMetalStrip& strip,
                                                   const SheetMetalParams& params, double width,
                                                   const std::string& featureID,
                                                   int segmentsPerBend) {
    if (width <= 0.0) return nullptr;
    const std::vector<Vec2> polygon = crossSection(strip, params, segmentsPerBend);
    if (polygon.size() < 3) return nullptr;

    std::vector<std::shared_ptr<draft::DraftEntity>> profile;
    profile.reserve(polygon.size());
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[(i + 1) % polygon.size()];
        profile.push_back(std::make_shared<draft::DraftLine>(a, b));
    }

    const draft::SketchPlane plane;  // XY at origin; width extrudes along +Z
    return Extrude::execute(profile, plane, plane.normal(), width, featureID);
}

std::vector<Vec2> SheetMetalSolid::flatPattern(const SheetMetalStrip& strip,
                                               const SheetMetalParams& params, double width) {
    if (width <= 0.0 || !stripIsFoldable(strip, params)) return {};
    // developedLength adds a bend allowance only for positive angles; a
    // downward (negative) bend develops the same flat length as an upward one
    // of the same magnitude, so pass the magnitudes to avoid an undersized
    // blank that omits the down-bend allowances.
    SheetMetalStrip magnitudes = strip;
    for (double& angle : magnitudes.bendAngles) angle = std::abs(angle);
    const double length = developedLength(magnitudes, params);
    if (length <= 0.0) return {};
    return {{0.0, 0.0}, {length, 0.0}, {length, width}, {0.0, width}};
}

}  // namespace hz::model
