#pragma once

#include "DraftEntity.h"
#include <vector>

namespace hz::draft {

class DraftPolyline : public DraftEntity {
public:
    explicit DraftPolyline(const std::vector<math::Vec2>& points, bool closed = false);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;

    const std::vector<math::Vec2>& points() const { return m_points; }
    void setPoints(const std::vector<math::Vec2>& points) { m_points = points; }

    bool closed() const { return m_closed; }
    void setClosed(bool closed) { m_closed = closed; }

    void addPoint(const math::Vec2& point);
    size_t pointCount() const { return m_points.size(); }

private:
    std::vector<math::Vec2> m_points;
    bool m_closed;
};

}  // namespace hz::draft
