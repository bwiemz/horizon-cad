#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "DraftEntity.h"
#include "SpatialIndex.h"
#include "horizon/math/Vec2.h"

namespace hz::draft {

struct SnapResult {
    math::Vec2 point;
    SnapType type = SnapType::None;
};

/// Snapping for a cursor: to a point on an entity near it (an endpoint,
/// midpoint, centre, quadrant, or where two entities cross), else to the grid.
/// The tolerance is a world distance; the viewport sets it from screen pixels.
class SnapEngine {
public:
    /// Which entities may be snapped to; every one when empty.
    using Filter = std::function<bool(const DraftEntity&)>;

    SnapEngine();

    void setGridSpacing(double spacing);
    double gridSpacing() const { return m_gridSpacing; }

    void setSnapTolerance(double tolerance);
    double snapTolerance() const { return m_snapTolerance; }

    /// The object snap nearest the cursor within the tolerance (on a tie, the
    /// more specific kind: an endpoint before an intersection, a midpoint, a
    /// centre, a quadrant); else the nearest grid point within it; else the
    /// cursor, with SnapType::None.
    SnapResult snap(const math::Vec2& cursorWorld,
                    const std::vector<std::shared_ptr<DraftEntity>>& entities,
                    const Filter& accept = {}) const;

    /// The same, looking only at entities the index finds near the cursor.
    SnapResult snap(const math::Vec2& cursorWorld, const SpatialIndex& index,
                    const std::vector<std::shared_ptr<DraftEntity>>& entities,
                    const Filter& accept = {}) const;

private:
    math::Vec2 snapToGrid(const math::Vec2& point) const;
    SnapResult snapAmong(const math::Vec2& cursorWorld,
                         const std::vector<const DraftEntity*>& near) const;

    double m_gridSpacing;
    double m_snapTolerance;
};

}  // namespace hz::draft
