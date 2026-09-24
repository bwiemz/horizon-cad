#include "horizon/modeling/ProfileValidator.h"

#include <cmath>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
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
    return "an entity of this kind";
}

/// Rectangles and polylines as the line segments they are drawn with; every
/// other entity as itself.
static std::vector<std::shared_ptr<draft::DraftEntity>> asCurves(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities) {
    std::vector<std::shared_ptr<draft::DraftEntity>> curves;
    curves.reserve(entities.size());
    const auto addSegment = [&](const Vec2& a, const Vec2& b) {
        curves.push_back(std::make_shared<draft::DraftLine>(a, b));
    };
    for (const auto& entity : entities) {
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
            curves.push_back(entity);
        }
    }
    return curves;
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

// ---------------------------------------------------------------------------
// ProfileValidator::validate
// ---------------------------------------------------------------------------

ProfileValidationResult ProfileValidator::validate(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& input, double tolerance) {
    ProfileValidationResult result;
    const std::vector<std::shared_ptr<draft::DraftEntity>> entities = asCurves(input);

    if (entities.empty()) {
        result.errorMessage = "the profile is empty";
        return result;
    }

    // Special case: single circle is always a closed loop.
    if (entities.size() == 1) {
        if (dynamic_cast<draft::DraftCircle*>(entities[0].get()) != nullptr) {
            result.isClosed = true;
            result.orderedEdges = entities;
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
                used[i] = true;
                chainEnd = ep.end;
                found = true;
                break;
            }
            if (pointsMatch(chainEnd, ep.end, tolerance)) {
                // Append reversed: entity goes end→start.
                result.orderedEdges.push_back(entities[i]);
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
    if (pointsMatch(chainEnd, chainStart, tolerance)) {
        result.isClosed = true;
    } else {
        result.errorMessage = "the profile is open: its ends at " + describePoint(chainStart) +
                              " and " + describePoint(chainEnd) + " do not meet";
    }

    return result;
}

}  // namespace hz::model
