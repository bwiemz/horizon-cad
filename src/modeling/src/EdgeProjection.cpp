#include "horizon/modeling/EdgeProjection.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Naming.h"

namespace hz::model {

using math::Vec2;
using math::Vec3;

namespace {

std::shared_ptr<draft::DraftEntity> nothing(std::string* why, const char* reason) {
    if (why != nullptr) *why = reason;
    return nullptr;
}

/// The circle through @p a, @p b and @p c: its centre; nullopt when they are
/// in a line.
std::optional<Vec2> circumcentre(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    const double scale = std::max({(b - a).length(), (c - a).length(), 1e-300});
    if (std::abs(d) < 1e-12 * scale * scale) return std::nullopt;
    const double a2 = a.x * a.x + a.y * a.y;
    const double b2 = b.x * b.x + b.y * b.y;
    const double c2 = c.x * c.x + c.y * c.y;
    return Vec2((a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d,
                (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d);
}

/// The chords' ends in order, end to end: from an end of the chain, or, for
/// one that closes, round from anywhere (and back to where it began).
/// Nullopt when they are not one chain (a curve a Boolean cut in two).
std::optional<std::vector<Vec3>> chain(const std::vector<std::pair<Vec3, Vec3>>& chords,
                                       double tolerance) {
    const auto same = [tolerance](const Vec3& a, const Vec3& b) {
        return (a - b).length() <= tolerance;
    };
    // Begin at an end the chain has only one chord at, if it has one.
    size_t first = 0;
    bool fromSecond = false;
    for (size_t i = 0; i < chords.size(); ++i) {
        for (const bool second : {false, true}) {
            const Vec3& end = second ? chords[i].second : chords[i].first;
            int touching = 0;
            for (const auto& [a, b] : chords)
                touching += (same(a, end) ? 1 : 0) + (same(b, end) ? 1 : 0);
            if (touching == 1) {
                first = i;
                fromSecond = second;
                i = chords.size();  // found: stop looking
                break;
            }
        }
    }
    std::vector<bool> used(chords.size(), false);
    std::vector<Vec3> points;
    Vec3 at = fromSecond ? chords[first].second : chords[first].first;
    points.push_back(at);
    for (size_t taken = 0; taken < chords.size(); ++taken) {
        bool moved = false;
        for (size_t i = 0; i < chords.size() && !moved; ++i) {
            if (used[i]) continue;
            if (same(chords[i].first, at)) {
                at = chords[i].second;
            } else if (same(chords[i].second, at)) {
                at = chords[i].first;
            } else {
                continue;
            }
            used[i] = true;
            points.push_back(at);
            moved = true;
        }
        if (!moved) return std::nullopt;  // a chord not joined to the rest
    }
    return points;
}

}  // namespace

std::string wholeEdgeName(const std::string& tag) {
    return wholeFaceName(logicalEdge(tag));
}

std::shared_ptr<draft::DraftEntity> projectEdge(const topo::Solid& solid, const std::string& edge,
                                                const draft::SketchPlane& plane, std::string* why) {
    // The chords: each edge whose logical name is @p edge, by its two ends.
    std::vector<std::pair<Vec3, Vec3>> chords;
    bool round = false;  // an ideal curve under the chords: a circle's
    double size = 0.0;
    for (const auto& e : solid.edges()) {
        const topo::HalfEdge* he = e.halfEdge;
        if (!e.topoId.isValid() || he == nullptr || he->origin == nullptr || he->next == nullptr ||
            he->next->origin == nullptr) {
            continue;
        }
        if (wholeEdgeName(e.topoId.tag()) != edge) continue;
        chords.emplace_back(he->origin->point, he->next->origin->point);
        size = std::max(size, (he->origin->point - he->next->origin->point).length());
        if (e.analyticCurve != nullptr) round = true;
    }
    if (chords.empty()) return nothing(why, "is not there");
    const double tolerance = 1e-9 * std::max(1.0, size);
    const auto points3 = chain(chords, tolerance);
    if (!points3) return nothing(why, "is in pieces");

    std::vector<Vec2> points;
    points.reserve(points3->size());
    for (const Vec3& p : *points3) points.push_back(plane.worldToLocal(p));
    const bool closed =
        points3->size() > 2 && (points3->front() - points3->back()).length() <= tolerance;
    double extent = 0.0;
    for (const Vec2& p : points) extent = std::max(extent, (p - points.front()).length());
    if (extent <= tolerance) return nothing(why, "is seen end on");
    const double flat = 1e-9 * std::max(1.0, extent);

    // A round edge in a plane parallel to the sketch is a circle there too.
    if (round && points.size() > 3) {
        double lo = (points3->front() - plane.origin()).dot(plane.normal());
        double hi = lo;
        for (const Vec3& p : *points3) {
            const double along = (p - plane.origin()).dot(plane.normal());
            lo = std::min(lo, along);
            hi = std::max(hi, along);
        }
        // Three points well apart: a third of the way round each for a closed
        // one (its last point is its first), the ends and middle of an arc.
        const size_t count = closed ? points.size() - 1 : points.size();
        const size_t middle = closed ? count / 3 : count / 2;
        const size_t last = closed ? 2 * count / 3 : count - 1;
        const auto centre = hi - lo <= flat
                                ? circumcentre(points.front(), points[middle], points[last])
                                : std::nullopt;
        if (centre) {
            const double radius = (points.front() - *centre).length();
            const bool onIt = std::all_of(points.begin(), points.end(), [&](const Vec2& p) {
                return std::abs((p - *centre).length() - radius) <= 1e-6 * std::max(1.0, radius);
            });
            if (onIt && closed) return std::make_shared<draft::DraftCircle>(*centre, radius);
            if (onIt) {
                const auto angle = [&centre](const Vec2& p) {
                    return std::atan2(p.y - centre->y, p.x - centre->x);
                };
                // An arc runs counterclockwise from its start: the chain's
                // way round, or the other.
                const Vec2 a = points.front() - *centre;
                const Vec2 b = points[middle] - *centre;
                const bool counterclockwise = a.x * b.y - a.y * b.x > 0.0;
                const Vec2& from = counterclockwise ? points.front() : points.back();
                const Vec2& to = counterclockwise ? points.back() : points.front();
                return std::make_shared<draft::DraftArc>(*centre, radius, angle(from), angle(to));
            }
        }
    }

    // Straight, however many chords it is in (a circle seen edge on is too):
    // a line between the two points furthest apart along it.
    const Vec2 base = points.front();
    Vec2 far = base;
    for (const Vec2& p : points) {
        if ((p - base).length() > (far - base).length()) far = p;
    }
    const Vec2 along = (far - base).normalized();
    const bool straight = std::all_of(points.begin(), points.end(), [&](const Vec2& p) {
        const Vec2 off = p - base;
        return std::abs(off.x * along.y - off.y * along.x) <= flat;
    });
    if (straight) {
        double lo = 0.0;
        double hi = 0.0;
        for (const Vec2& p : points) {
            const double t = (p - base).dot(along);
            lo = std::min(lo, t);
            hi = std::max(hi, t);
        }
        return std::make_shared<draft::DraftLine>(base + along * lo, base + along * hi);
    }
    if (closed) points.pop_back();  // a closed polyline joins its last point to its first
    return std::make_shared<draft::DraftPolyline>(points, closed);
}

}  // namespace hz::model
