#include "horizon/modeling/ProfileValidator.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/math/Vec2.h"

namespace hz::model {

using hz::math::Vec2;

// ---------------------------------------------------------------------------
// Helpers: extract start/end points from supported entity types.
// ---------------------------------------------------------------------------

struct EndpointPair {
    Vec2 start;
    Vec2 end;
    bool valid = false;
};

static EndpointPair getEndpoints(const std::shared_ptr<draft::DraftEntity>& entity) {
    if (auto* line = dynamic_cast<draft::DraftLine*>(entity.get())) {
        return {line->start(), line->end(), true};
    }
    if (auto* arc = dynamic_cast<draft::DraftArc*>(entity.get())) {
        return {arc->startPoint(), arc->endPoint(), true};
    }
    // DraftCircle has no start/end — handled separately as a single-entity closed loop.
    return {{}, {}, false};
}

/// What kind of entity this is, for a message about one that a profile cannot
/// use.
static const char* kindOf(const draft::DraftEntity& entity) {
    if (dynamic_cast<const draft::DraftCircle*>(&entity)) return "a circle";
    if (dynamic_cast<const draft::DraftEllipse*>(&entity)) return "an ellipse";
    if (dynamic_cast<const draft::DraftSpline*>(&entity)) return "a spline";
    if (dynamic_cast<const draft::DraftText*>(&entity)) return "text";
    if (dynamic_cast<const draft::DraftBlockRef*>(&entity)) {
        return "a block reference (explode it first)";
    }
    return "an entity of this kind";
}

/// Rectangles and polylines as the line segments they are drawn with; every
/// other entity as itself, but construction geometry (Phase 157), which is
/// left out.
/// The curves a profile is made of, each with the name of where it came from:
/// "e<id>" for a sketch entity used as it is, "e<id>.<k>" for side k of a
/// rectangle or polyline (which is read as its line segments, made anew each
/// time — so their own ids mean nothing, and the source's must be used).
struct Curves {
    std::vector<std::shared_ptr<draft::DraftEntity>> curves;
    std::vector<std::string> sources;
};

static Curves asCurves(const std::vector<std::shared_ptr<draft::DraftEntity>>& entities) {
    Curves out;
    out.curves.reserve(entities.size());
    out.sources.reserve(entities.size());
    for (const auto& entity : entities) {
        // Construction geometry guides the drawing; it is never the shape.
        if (!entity || entity->construction()) continue;
        const std::string name = "e" + std::to_string(entity->id());
        int side = 0;
        const auto addSegment = [&](const Vec2& a, const Vec2& b) {
            out.curves.push_back(std::make_shared<draft::DraftLine>(a, b));
            out.sources.push_back(name + "." + std::to_string(side++));
        };
        if (const auto* rect = dynamic_cast<const draft::DraftRectangle*>(entity.get())) {
            const Vec2 c1 = rect->corner1();
            const Vec2 c2 = rect->corner2();
            addSegment(c1, {c2.x, c1.y});
            addSegment({c2.x, c1.y}, c2);
            addSegment(c2, {c1.x, c2.y});
            addSegment({c1.x, c2.y}, c1);
        } else if (const auto* poly = dynamic_cast<const draft::DraftPolyline*>(entity.get())) {
            const auto& pts = poly->points();
            for (size_t i = 0; i + 1 < pts.size(); ++i) addSegment(pts[i], pts[i + 1]);
            if (poly->closed() && pts.size() > 2) addSegment(pts.back(), pts.front());
        } else {
            out.curves.push_back(entity);
            out.sources.push_back(name);
        }
    }
    return out;
}

static bool pointsMatch(const Vec2& a, const Vec2& b, double tolerance) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return (dx * dx + dy * dy) <= tolerance * tolerance;
}

/// "(3, 4.5)" — a point for a message, as a user would read it off the grid.
static std::string describePoint(const Vec2& p) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '(' << std::setprecision(6) << p.x << ", " << p.y << ')';
    return out.str();
}

/// The closed loop the ordered edges trace, from `start`, as a polygon: lines
/// as themselves, arcs sampled finely enough to find a crossing.
static std::vector<Vec2> loopPolygon(const std::vector<std::shared_ptr<draft::DraftEntity>>& edges,
                                     Vec2 start, double tolerance) {
    std::vector<Vec2> points;
    Vec2 at = start;
    for (const auto& edge : edges) {
        const EndpointPair ep = getEndpoints(edge);
        const bool forward = pointsMatch(at, ep.start, tolerance);
        points.push_back(at);
        if (const auto* arc = dynamic_cast<const draft::DraftArc*>(edge.get())) {
            constexpr int kSteps = 32;
            const double sweep = arc->sweepAngle();
            for (int k = 1; k < kSteps; ++k) {
                const double t = static_cast<double>(forward ? k : kSteps - k) / kSteps;
                const double angle = arc->startAngle() + sweep * t;
                points.emplace_back(arc->center().x + arc->radius() * std::cos(angle),
                                    arc->center().y + arc->radius() * std::sin(angle));
            }
        }
        at = forward ? ep.end : ep.start;
    }
    return points;
}

/// Where segments ab and cd meet (touching counts), if they do.
static std::optional<Vec2> segmentsMeet(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d,
                                        double tolerance) {
    const auto cross = [](const Vec2& u, const Vec2& v) { return u.x * v.y - u.y * v.x; };
    const Vec2 r = b - a;
    const Vec2 q = d - c;
    const double denom = cross(r, q);
    const double scale = std::max({r.length(), q.length(), 1e-300});
    if (std::abs(denom) > tolerance * scale) {
        const double t = cross(c - a, q) / denom;
        const double u = cross(c - a, r) / denom;
        const double slackT = tolerance / std::max(r.length(), 1e-300);
        const double slackU = tolerance / std::max(q.length(), 1e-300);
        if (t >= -slackT && t <= 1 + slackT && u >= -slackU && u <= 1 + slackU) {
            return a + r * std::clamp(t, 0.0, 1.0);
        }
        return std::nullopt;
    }
    // Parallel: they meet only if collinear and overlapping.
    if (std::abs(cross(c - a, r)) > tolerance * std::max(r.length(), 1e-300)) return std::nullopt;
    const double len2 = r.x * r.x + r.y * r.y;
    if (len2 <= 0.0) return std::nullopt;
    const auto along = [&](const Vec2& p) { return ((p - a).x * r.x + (p - a).y * r.y) / len2; };
    const double lo = std::max(0.0, std::min(along(c), along(d)));
    const double hi = std::min(1.0, std::max(along(c), along(d)));
    if (lo > hi + tolerance / std::sqrt(len2)) return std::nullopt;
    return a + r * lo;
}

/// Where the loop crosses or touches itself, if it does: a figure-eight, or a
/// boundary folded back onto itself, bounds no single region to extrude.
static std::optional<Vec2> selfCrossing(const std::vector<Vec2>& loop, double tolerance) {
    const size_t n = loop.size();
    if (n < 4) return std::nullopt;
    for (size_t i = 0; i < n; ++i) {
        const Vec2& a = loop[i];
        const Vec2& b = loop[(i + 1) % n];
        for (size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;  // the closing segment meets the first
            const Vec2& c = loop[j];
            const Vec2& d = loop[(j + 1) % n];
            if (auto where = segmentsMeet(a, b, c, d, tolerance)) return where;
        }
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// ProfileValidator::validate
// ---------------------------------------------------------------------------

ProfileValidationResult ProfileValidator::validate(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& input, double tolerance) {
    ProfileValidationResult result;
    const Curves curves = asCurves(input);
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities = curves.curves;

    if (entities.empty()) {
        result.errorMessage = "the profile is empty";
        return result;
    }

    // Special case: single circle is always a closed loop.
    if (entities.size() == 1) {
        if (dynamic_cast<draft::DraftCircle*>(entities[0].get()) != nullptr) {
            result.isClosed = true;
            result.orderedEdges = entities;
            result.edgeSources = curves.sources;
            return result;
        }
    }

    // Build an ordered chain from lines and arcs.
    // Start with the first entity; chain by matching endpoints.
    std::vector<bool> used(entities.size(), false);

    // Every curve in a chain must have two ends; a circle has none, so it is a
    // profile only on its own (handled above).
    for (const auto& entity : entities) {
        if (getEndpoints(entity).valid) continue;
        result.errorMessage =
            dynamic_cast<const draft::DraftCircle*>(entity.get())
                ? std::string(
                      "a circle cannot be joined to other curves in a profile; use it on "
                      "its own")
                : std::string(kindOf(*entity)) +
                      " cannot be used in a profile yet; profiles are made of lines, arcs, "
                      "rectangles, polylines, or a single circle";
        return result;
    }

    EndpointPair firstEP = getEndpoints(entities[0]);

    result.orderedEdges.push_back(entities[0]);
    result.edgeSources.push_back(curves.sources[0]);
    used[0] = true;

    Vec2 chainStart = firstEP.start;
    Vec2 chainEnd = firstEP.end;

    // Greedily chain entities.
    for (size_t iter = 1; iter < entities.size(); ++iter) {
        bool found = false;
        for (size_t i = 0; i < entities.size(); ++i) {
            if (used[i]) continue;
            EndpointPair ep = getEndpoints(entities[i]);
            if (!ep.valid) continue;

            if (pointsMatch(chainEnd, ep.start, tolerance)) {
                // Append: entity goes start→end.
                result.orderedEdges.push_back(entities[i]);
                result.edgeSources.push_back(curves.sources[i]);
                used[i] = true;
                chainEnd = ep.end;
                found = true;
                break;
            }
            if (pointsMatch(chainEnd, ep.end, tolerance)) {
                // Append reversed: entity goes end→start.
                result.orderedEdges.push_back(entities[i]);
                result.edgeSources.push_back(curves.sources[i]);
                used[i] = true;
                chainEnd = ep.start;
                found = true;
                break;
            }
        }
        if (!found) {
            result.errorMessage =
                "the profile has a gap: nothing continues from " + describePoint(chainEnd);
            return result;
        }
    }

    // Check closure: does chainEnd meet chainStart?
    if (!pointsMatch(chainEnd, chainStart, tolerance)) {
        result.errorMessage = "the profile is open: its ends at " + describePoint(chainStart) +
                              " and " + describePoint(chainEnd) + " do not meet";
        return result;
    }

    if (const auto where =
            selfCrossing(loopPolygon(result.orderedEdges, chainStart, tolerance), tolerance)) {
        result.errorMessage = "the profile crosses itself at " + describePoint(*where);
        return result;
    }
    result.isClosed = true;
    return result;
}

// ---------------------------------------------------------------------------
// ProfileValidator::regions
// ---------------------------------------------------------------------------

namespace {

/// A note on the sketch rather than part of its shape.
bool isAnnotation(const draft::DraftEntity& entity) {
    return dynamic_cast<const draft::DraftText*>(&entity) != nullptr ||
           dynamic_cast<const draft::DraftDimension*>(&entity) != nullptr ||  // leaders too
           dynamic_cast<const draft::DraftHatch*>(&entity) != nullptr;
}

/// A closed loop found among the curves, with its outline for nesting.
struct FoundLoop {
    ProfileValidationResult loop;
    std::vector<Vec2> polygon;
    double area = 0.0;  ///< unsigned
    int depth = 0;      ///< how many other loops it lies inside
    int parent = -1;    ///< the smallest loop it lies inside
};

double polygonArea(const std::vector<Vec2>& poly) {
    double twice = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % poly.size()];
        twice += a.x * b.y - b.x * a.y;
    }
    return std::abs(twice) / 2.0;
}

/// Even-odd point in polygon.
bool inside(const Vec2& p, const std::vector<Vec2>& poly) {
    bool in = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) {
            in = !in;
        }
    }
    return in;
}

/// A circle as a polygon, fine enough to nest and cross-check against.
std::vector<Vec2> circlePolygon(const draft::DraftCircle& circle) {
    constexpr int kSteps = 64;
    std::vector<Vec2> points;
    points.reserve(kSteps);
    for (int k = 0; k < kSteps; ++k) {
        const double a = 2.0 * 3.14159265358979323846 * static_cast<double>(k) / kSteps;
        points.emplace_back(circle.center().x + circle.radius() * std::cos(a),
                            circle.center().y + circle.radius() * std::sin(a));
    }
    return points;
}

/// Where two loops' outlines meet, if they do.
std::optional<Vec2> loopsMeet(const std::vector<Vec2>& a, const std::vector<Vec2>& b,
                              double tolerance) {
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < b.size(); ++j) {
            if (auto where = segmentsMeet(a[i], a[(i + 1) % a.size()], b[j], b[(j + 1) % b.size()],
                                          tolerance)) {
                return where;
            }
        }
    }
    return std::nullopt;
}

}  // namespace

ProfileRegions ProfileValidator::regions(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& input, double tolerance) {
    ProfileRegions result;
    std::vector<std::shared_ptr<draft::DraftEntity>> shape;
    for (const auto& entity : input) {
        if (entity && !isAnnotation(*entity) && !entity->construction()) shape.push_back(entity);
    }
    if (shape.empty()) {
        result.errorMessage = "the profile is empty";
        return result;
    }
    const Curves curves = asCurves(shape);
    const auto& all = curves.curves;

    std::vector<FoundLoop> loops;
    std::vector<bool> used(all.size(), false);
    for (size_t i = 0; i < all.size(); ++i) {
        if (used[i]) continue;
        const auto* circle = dynamic_cast<const draft::DraftCircle*>(all[i].get());
        if (circle != nullptr) {
            used[i] = true;
            FoundLoop found;
            found.loop.isClosed = true;
            found.loop.orderedEdges = {all[i]};
            found.loop.edgeSources = {curves.sources[i]};
            found.polygon = circlePolygon(*circle);
            loops.push_back(std::move(found));
            continue;
        }
        if (!getEndpoints(all[i]).valid) {
            result.errorMessage =
                std::string(kindOf(*all[i])) +
                " cannot be used in a profile yet; profiles are made of lines, arcs, "
                "rectangles, polylines and circles";
            return result;
        }
        // Chain from this curve until the chain closes on itself.
        FoundLoop found;
        found.loop.orderedEdges.push_back(all[i]);
        found.loop.edgeSources.push_back(curves.sources[i]);
        used[i] = true;
        const EndpointPair first = getEndpoints(all[i]);
        const Vec2 chainStart = first.start;
        Vec2 chainEnd = first.end;
        while (!pointsMatch(chainEnd, chainStart, tolerance)) {
            bool extended = false;
            for (size_t k = 0; k < all.size() && !extended; ++k) {
                if (used[k]) continue;
                const EndpointPair ep = getEndpoints(all[k]);
                if (!ep.valid) continue;
                if (pointsMatch(chainEnd, ep.start, tolerance)) {
                    chainEnd = ep.end;
                } else if (pointsMatch(chainEnd, ep.end, tolerance)) {
                    chainEnd = ep.start;
                } else {
                    continue;
                }
                found.loop.orderedEdges.push_back(all[k]);
                found.loop.edgeSources.push_back(curves.sources[k]);
                used[k] = true;
                extended = true;
            }
            if (!extended) {
                result.errorMessage =
                    found.loop.orderedEdges.size() == 1 ||
                            std::none_of(used.begin(), used.end(), [](bool u) { return !u; })
                        ? "the profile is open: its ends at " + describePoint(chainStart) +
                              " and " + describePoint(chainEnd) + " do not meet"
                        : "the profile has a gap: nothing continues from " +
                              describePoint(chainEnd);
                return result;
            }
        }
        found.loop.isClosed = true;
        found.polygon = loopPolygon(found.loop.orderedEdges, chainStart, tolerance);
        if (const auto where = selfCrossing(found.polygon, tolerance)) {
            result.errorMessage = "the profile crosses itself at " + describePoint(*where);
            return result;
        }
        loops.push_back(std::move(found));
    }

    // Loops may nest, but not cross or touch: that bounds no region cleanly.
    for (size_t i = 0; i < loops.size(); ++i) {
        loops[i].area = polygonArea(loops[i].polygon);
        for (size_t j = i + 1; j < loops.size(); ++j) {
            if (const auto where = loopsMeet(loops[i].polygon, loops[j].polygon, tolerance)) {
                result.errorMessage = "two loops of the profile meet at " + describePoint(*where) +
                                      "; loops must be apart, or one wholly inside another";
                return result;
            }
        }
    }
    for (size_t i = 0; i < loops.size(); ++i) {
        for (size_t j = 0; j < loops.size(); ++j) {
            if (i == j || !inside(loops[i].polygon.front(), loops[j].polygon)) continue;
            ++loops[i].depth;
            if (loops[i].parent < 0 ||
                loops[j].area < loops[static_cast<size_t>(loops[i].parent)].area) {
                loops[i].parent = static_cast<int>(j);
            }
        }
    }

    // Even depth: a region's outer loop. Odd: a hole in the loop around it.
    std::vector<int> regionOf(loops.size(), -1);
    for (size_t i = 0; i < loops.size(); ++i) {
        if (loops[i].depth % 2 != 0) continue;
        regionOf[i] = static_cast<int>(result.regions.size());
        result.regions.push_back({loops[i].loop, {}});
    }
    for (size_t i = 0; i < loops.size(); ++i) {
        if (loops[i].depth % 2 == 0) continue;
        const int region = regionOf[static_cast<size_t>(loops[i].parent)];
        result.regions[static_cast<size_t>(region)].holes.push_back(loops[i].loop);
    }
    return result;
}

}  // namespace hz::model
