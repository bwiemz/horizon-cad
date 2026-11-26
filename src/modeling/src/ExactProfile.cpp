#include "ExactProfile.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <utility>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/math/Constants.h"

namespace hz::model::exact {

using math::Vec2;
using math::Vec3;

namespace {

/// A point as the profile's messages give one: "(x, y)", six figures.
std::string describe(const Vec2& p) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '(' << std::setprecision(6) << p.x << ", " << p.y << ')';
    return out.str();
}

/// Twice the area @p curve sweeps about the origin as the loop runs it: x dy - y dx along it.
double twiceSweptArea(const LoopCurve& curve) {
    if (!curve.curved()) return curve.start.x * curve.end.y - curve.end.x * curve.start.y;
    // Along the circle from angle a to b (b - a the signed sweep): r cx (sin b - sin a)
    // - r cy (cos b - cos a) + r^2 (b - a).
    const double a = curve.forward ? curve.ownStart : curve.ownStart + curve.sweep;
    const double b = curve.forward ? curve.ownStart + curve.sweep : curve.ownStart;
    const double r = curve.radius;
    return r * curve.center.x * (std::sin(b) - std::sin(a)) -
           r * curve.center.y * (std::cos(b) - std::cos(a)) + r * r * (b - a);
}

/// The loop @p loop chains, its corners master's: the first curve's start, then each curve's end
/// as the chain runs it, the last closing on the first's start. Nullopt with @p why for a curve
/// it cannot take.
std::optional<Loop> chained(const ProfileValidationResult& loop, std::string* why) {
    const auto refuse = [why](std::string reason) -> std::optional<Loop> {
        if (why) *why = std::move(reason);
        return std::nullopt;
    };
    Loop out;
    const auto& edges = loop.orderedEdges;
    if (edges.size() != loop.edgeSources.size()) return refuse("its profile lost track of a curve");
    out.curves.reserve(edges.size());
    Vec2 chainEnd;
    for (size_t k = 0; k < edges.size(); ++k) {
        LoopCurve c;
        c.source = loop.edgeSources[k];
        Vec2 s;
        Vec2 e;
        if (const auto* line = dynamic_cast<const draft::DraftLine*>(edges[k].get())) {
            c.kind = LoopCurve::Kind::Line;
            s = line->start();
            e = line->end();
        } else if (const auto* arc = dynamic_cast<const draft::DraftArc*>(edges[k].get())) {
            c.kind = LoopCurve::Kind::Arc;
            c.center = arc->center();
            c.radius = arc->radius();
            c.ownStart = arc->startAngle();
            c.sweep = arc->sweepAngle();
            s = arc->startPoint();
            e = arc->endPoint();
        } else if (const auto* circle = dynamic_cast<const draft::DraftCircle*>(edges[k].get())) {
            if (edges.size() != 1) return refuse("a circle is joined to other curves");
            c.kind = LoopCurve::Kind::Circle;
            c.center = circle->center();
            c.radius = circle->radius();
            c.ownStart = 0.0;
            c.sweep = math::kTwoPi;
            s = e = c.ownStartPoint();
        } else {
            return refuse("its profile has a curve that is not a line, an arc or a circle");
        }
        if (c.curved() && !(c.radius > 0.0 && std::isfinite(c.radius))) {
            return refuse("an arc of the profile at " + describe(c.center) + " has no radius");
        }
        // Each curve continues the chain from the end the one before it reached: run backwards
        // where its end is the nearer, as the faceted extrude chains them.
        const bool reversed = k > 0 && (chainEnd - e).length() < (chainEnd - s).length();
        c.forward = !c.curved() || !reversed;
        c.start = k == 0 ? s : chainEnd;
        c.end = reversed ? s : e;
        chainEnd = c.end;
        out.curves.push_back(std::move(c));
    }
    if (out.curves.empty()) return refuse("its profile has an empty loop");
    out.curves.back().end = out.curves.front().start;
    for (const LoopCurve& c : out.curves) {
        if (c.kind == LoopCurve::Kind::Line && !((c.end - c.start).length() > 0.0)) {
            return refuse("a line of the profile at " + describe(c.start) + " has no length");
        }
    }
    return out;
}

/// @p loop turned the other way round.
void reverse(Loop& loop) {
    loop.againstChain = !loop.againstChain;
    std::reverse(loop.curves.begin(), loop.curves.end());
    for (LoopCurve& c : loop.curves) {
        std::swap(c.start, c.end);
        if (c.curved()) c.forward = !c.forward;
    }
}

/// @p loop with its material on its left: counter-clockwise for an outer loop, clockwise for a
/// hole. Nullopt with @p why where it encloses no area.
std::optional<Loop> oriented(const ProfileValidationResult& from, bool hole, std::string* why) {
    auto loop = chained(from, why);
    if (!loop) return std::nullopt;
    double twice = 0.0;
    for (const LoopCurve& c : loop->curves) twice += twiceSweptArea(c);
    if (!(std::abs(twice) > 0.0) || !std::isfinite(twice)) {
        if (why) {
            *why = "a loop of the profile at " + describe(loop->curves.front().start) +
                   " encloses no area";
        }
        return std::nullopt;
    }
    if ((twice > 0.0) == hole) reverse(*loop);
    return loop;
}

}  // namespace

Vec2 LoopCurve::ownStartPoint() const {
    if (!curved()) return start;
    return {center.x + radius * std::cos(ownStart), center.y + radius * std::sin(ownStart)};
}

Vec2 LoopCurve::ownEndPoint() const {
    if (!curved()) return end;
    const double a = ownStart + sweep;
    return {center.x + radius * std::cos(a), center.y + radius * std::sin(a)};
}

Course<geo::Curve2D> LoopCurve::pcurve(bool along) const {
    Course<geo::Curve2D> out;
    if (!curved()) {
        out.curve = std::make_shared<geo::Line2D>(along ? start : end, along ? end : start);
        out.start = 0.0;
        out.end = 1.0;
        return out;
    }
    out.curve = std::make_shared<geo::Circle2D>(center, radius);
    const bool own = along == forward;
    out.start = own ? ownStart : ownStart + sweep;
    out.end = own ? ownStart + sweep : ownStart;
    return out;
}

geo::Frame LoopCurve::ownFrame(const draft::SketchPlane& plane, const Vec3& offset) const {
    geo::Frame f;
    f.origin = plane.localToWorld(center) + offset;
    f.x = plane.xAxis() * std::cos(ownStart) + plane.yAxis() * std::sin(ownStart);
    f.z = plane.normal();
    f.y = f.z.cross(f.x);
    return f;
}

Course<geo::Curve3> LoopCurve::curve(const draft::SketchPlane& plane, const Vec3& offset,
                                     bool along) const {
    Course<geo::Curve3> out;
    if (!curved()) {
        const Vec3 from = plane.localToWorld(along ? start : end) + offset;
        const Vec3 to = plane.localToWorld(along ? end : start) + offset;
        out.curve = std::make_shared<geo::LineCurve>(from, to - from);
        out.start = 0.0;
        out.end = (to - from).length();
        return out;
    }
    out.curve = std::make_shared<geo::CircleCurve>(ownFrame(plane, offset), radius);
    const bool own = along == forward;
    out.start = own ? 0.0 : sweep;
    out.end = own ? sweep : 0.0;
    return out;
}

std::optional<Region> regionOf(const ProfileRegion& region, std::string* why) {
    Region out;
    auto outer = oriented(region.outer, false, why);
    if (!outer) return std::nullopt;
    out.outer = std::move(*outer);
    for (const auto& hole : region.holes) {
        auto loop = oriented(hole, true, why);
        if (!loop) return std::nullopt;
        out.holes.push_back(std::move(*loop));
    }
    return out;
}

}  // namespace hz::model::exact
