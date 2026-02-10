#pragma once

#include "horizon/math/Vec2.h"
#include "DraftEntity.h"
#include <memory>
#include <vector>

namespace hz::draft {

enum class SnapType {
    None,
    Grid,
    Endpoint,
    Midpoint,
    Center
};

struct SnapResult {
    math::Vec2 point;
    SnapType type = SnapType::None;
};

class SnapEngine {
public:
    SnapEngine();

    void setGridSpacing(double spacing);
    double gridSpacing() const { return m_gridSpacing; }

    void setSnapTolerance(double tolerance);
    double snapTolerance() const { return m_snapTolerance; }

    SnapResult snap(const math::Vec2& cursorWorld,
                    const std::vector<std::shared_ptr<DraftEntity>>& entities) const;

private:
    math::Vec2 snapToGrid(const math::Vec2& point) const;

    double m_gridSpacing;
    double m_snapTolerance;
};

}  // namespace hz::draft
