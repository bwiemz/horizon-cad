#include "horizon/drafting/SnapEngine.h"
#include "horizon/math/BoundingBox.h"
#include <cmath>
#include <limits>

namespace hz::draft {

SnapEngine::SnapEngine()
    : m_gridSpacing(1.0)
    , m_snapTolerance(0.5) {}

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

SnapResult SnapEngine::snap(const math::Vec2& cursorWorld,
                            const std::vector<std::shared_ptr<DraftEntity>>& entities) const {
    SnapResult best;
    best.point = cursorWorld;
    best.type = SnapType::None;
    double bestDist = std::numeric_limits<double>::max();

    // Check entity snap points (Endpoint, Midpoint, Center)
    for (const auto& entity : entities) {
        if (!entity) continue;
        std::vector<math::Vec2> pts = entity->snapPoints();
        for (const auto& pt : pts) {
            double dist = cursorWorld.distanceTo(pt);
            if (dist < m_snapTolerance && dist < bestDist) {
                bestDist = dist;
                best.point = pt;
                best.type = SnapType::Endpoint;
            }
        }
    }

    // Check grid snap
    math::Vec2 gridPt = snapToGrid(cursorWorld);
    double gridDist = cursorWorld.distanceTo(gridPt);
    if (gridDist < m_snapTolerance && gridDist < bestDist) {
        bestDist = gridDist;
        best.point = gridPt;
        best.type = SnapType::Grid;
    }

    return best;
}

SnapResult SnapEngine::snap(const math::Vec2& cursorWorld,
                            const SpatialIndex& index,
                            const std::vector<std::shared_ptr<DraftEntity>>& entities) const {
    SnapResult best;
    best.point = cursorWorld;
    best.type = SnapType::None;
    double bestDist = std::numeric_limits<double>::max();

    // Build a search box around cursor at snap tolerance.
    math::BoundingBox searchBox(
        math::Vec3(cursorWorld.x - m_snapTolerance, cursorWorld.y - m_snapTolerance, -1e9),
        math::Vec3(cursorWorld.x + m_snapTolerance, cursorWorld.y + m_snapTolerance, 1e9));

    auto candidateIds = index.query(searchBox);

    // Only check snap points of entities near cursor.
    for (uint64_t id : candidateIds) {
        for (const auto& entity : entities) {
            if (entity->id() != id) continue;
            std::vector<math::Vec2> pts = entity->snapPoints();
            for (const auto& pt : pts) {
                double dist = cursorWorld.distanceTo(pt);
                if (dist < m_snapTolerance && dist < bestDist) {
                    bestDist = dist;
                    best.point = pt;
                    best.type = SnapType::Endpoint;
                }
            }
            break;
        }
    }

    // Check grid snap.
    math::Vec2 gridPt = snapToGrid(cursorWorld);
    double gridDist = cursorWorld.distanceTo(gridPt);
    if (gridDist < m_snapTolerance && gridDist < bestDist) {
        best.point = gridPt;
        best.type = SnapType::Grid;
    }

    return best;
}

}  // namespace hz::draft
