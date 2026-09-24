#include "horizon/drafting/SnapEngine.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "horizon/drafting/Intersection.h"
#include "horizon/math/BoundingBox.h"

namespace hz::draft {

SnapEngine::SnapEngine() : m_gridSpacing(1.0), m_snapTolerance(0.5) {}

void SnapEngine::setGridSpacing(double spacing) {
    if (spacing > 0.0) {
        m_gridSpacing = spacing;
    }
}

void SnapEngine::setSnapTolerance(double tolerance) {
    if (tolerance > 0.0) {
        m_snapTolerance = tolerance;
    }
}

math::Vec2 SnapEngine::snapToGrid(const math::Vec2& point) const {
    double x = std::round(point.x / m_gridSpacing) * m_gridSpacing;
    double y = std::round(point.y / m_gridSpacing) * m_gridSpacing;
    return math::Vec2(x, y);
}

namespace {

/// Which kind of snap wins a tie: the most specific.
int rank(SnapType type) {
    switch (type) {
        case SnapType::Endpoint:
            return 0;
        case SnapType::Intersection:
            return 1;
        case SnapType::Midpoint:
            return 2;
        case SnapType::Center:
            return 3;
        case SnapType::Quadrant:
            return 4;
        default:
            return 5;
    }
}

}  // namespace

SnapResult SnapEngine::snapAmong(const math::Vec2& cursorWorld,
                                 const std::vector<const DraftEntity*>& near) const {
    SnapResult best;
    best.point = cursorWorld;
    double bestDist = m_snapTolerance;
    const auto consider = [&](const math::Vec2& p, SnapType type) {
        const double dist = cursorWorld.distanceTo(p);
        if (dist >= m_snapTolerance) return;
        const bool nearer = dist < bestDist - 1e-12;
        const bool tie = dist <= bestDist + 1e-12;
        if (best.type == SnapType::None || nearer || (tie && rank(type) < rank(best.type))) {
            best.point = p;
            best.type = type;
            bestDist = dist;
        }
    };

    // Only entities that pass within the tolerance can cross near the cursor.
    std::vector<const DraftEntity*> touching;
    for (const DraftEntity* entity : near) {
        for (const auto& sp : entity->typedSnapPoints()) consider(sp.point, sp.type);
        if (entity->hitTest(cursorWorld, m_snapTolerance)) touching.push_back(entity);
    }
    constexpr size_t kMaxTouching = 32;  // a bound on the pairs tried per cursor move
    const size_t n = std::min(touching.size(), kMaxTouching);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            for (const auto& p : intersect(*touching[i], *touching[j]).points) {
                consider(p, SnapType::Intersection);
            }
        }
    }
    if (best.type != SnapType::None) return best;

    // The grid only when no object snap is in reach.
    const math::Vec2 gridPt = snapToGrid(cursorWorld);
    if (cursorWorld.distanceTo(gridPt) < m_snapTolerance) {
        best.point = gridPt;
        best.type = SnapType::Grid;
    }
    return best;
}

SnapResult SnapEngine::snap(const math::Vec2& cursorWorld,
                            const std::vector<std::shared_ptr<DraftEntity>>& entities,
                            const Filter& accept) const {
    std::vector<const DraftEntity*> near;
    near.reserve(entities.size());
    for (const auto& entity : entities) {
        if (entity && (!accept || accept(*entity))) near.push_back(entity.get());
    }
    return snapAmong(cursorWorld, near);
}

SnapResult SnapEngine::snap(const math::Vec2& cursorWorld, const SpatialIndex& index,
                            const std::vector<std::shared_ptr<DraftEntity>>& entities,
                            const Filter& accept) const {
    const math::BoundingBox searchBox(
        math::Vec3(cursorWorld.x - m_snapTolerance, cursorWorld.y - m_snapTolerance, -1e9),
        math::Vec3(cursorWorld.x + m_snapTolerance, cursorWorld.y + m_snapTolerance, 1e9));
    const auto ids = index.query(searchBox);
    const std::unordered_set<uint64_t> candidates(ids.begin(), ids.end());

    // One pass over the entities: looking each candidate up by id in the
    // list was a pass per candidate.
    std::vector<const DraftEntity*> near;
    near.reserve(candidates.size());
    for (const auto& entity : entities) {
        if (!entity || candidates.count(entity->id()) == 0) continue;
        if (!accept || accept(*entity)) near.push_back(entity.get());
    }
    return snapAmong(cursorWorld, near);
}

}  // namespace hz::draft
