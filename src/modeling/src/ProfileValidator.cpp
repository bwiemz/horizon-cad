#include "horizon/modeling/ProfileValidator.h"

#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
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
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities, double tolerance) {
    ProfileValidationResult result;

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

    // Find first usable entity.
    EndpointPair firstEP = getEndpoints(entities[0]);
    if (!firstEP.valid) {
        result.errorMessage =
            "a circle cannot be joined to other curves in a profile; extrude it on its own";
        return result;
    }

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
