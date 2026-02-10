#pragma once

#include "DraftEntity.h"

namespace hz::draft {

class DraftLine : public DraftEntity {
public:
    DraftLine(const math::Vec2& start, const math::Vec2& end);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;

    const math::Vec2& start() const { return m_start; }
    const math::Vec2& end() const { return m_end; }

    void setStart(const math::Vec2& start) { m_start = start; }
    void setEnd(const math::Vec2& end) { m_end = end; }

private:
    math::Vec2 m_start, m_end;
};

}  // namespace hz::draft
